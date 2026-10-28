"""Exercise the actual WebSocket bridge and C++ services without a GUI."""
import asyncio
import importlib.util
import json
import math
import os
import socket
import struct
import subprocess
import time
import urllib.request
import numpy as np
import pytest
from aiohttp import ClientSession, WSMsgType
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool
import rclpy
from rclpy.context import Context
from rclpy.qos import QoSProfile, DurabilityPolicy
from mid360_obstacle_avoidance.msg import RoiState, DetectorConfig
from test_obstacle_node import detector, line

spec = importlib.util.spec_from_file_location('mid360_web', os.environ['MID360_WEB_SCRIPT'])
web = importlib.util.module_from_spec(spec)
spec.loader.exec_module(web)


def test_pack_xyz():
    for endian in ('<', '>'):
        msg = PointCloud2(height=2, width=2, point_step=16, row_step=40,
                          is_bigendian=endian == '>')
        msg.fields = [PointField(name=n,offset=i*4,datatype=7,count=1)
                      for i,n in enumerate('xyz')]
        data = bytearray(80)
        for row,col,xyz in [(0,0,(4,2,3)),(0,1,(0,0,0)),
                            (1,0,(math.nan,1,2)),(1,1,(1,2,3))]:
            struct.pack_into(endian+'fff',data,row*40+col*16,*xyz)
        msg.data = bytes(data)
        assert struct.unpack('<ffffff',web.pack_xyz(msg)) == (4,2,3,1,2,3)
        msg.fields[0].offset=100
        with pytest.raises(ValueError):web.pack_xyz(msg)
    assert web.pack_xyz(PointCloud2()) == b''


def check_saved_restart(config, expected, algorithm=None):
    # A fresh process reads the saved YAML without ROI CLI overrides.
    domain = 220 + os.getpid() % 10
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    observer = rclpy.create_node('saved_roi_restart_probe', context=context)
    states, configs = [], []
    observer.create_subscription(RoiState, '/mid360_obstacle_avoidance/roi_state',
        states.append, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    observer.create_subscription(DetectorConfig, '/mid360_obstacle_avoidance/detector_config',
        configs.append, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    environment = os.environ.copy()
    environment['ROS_DOMAIN_ID'] = str(domain)
    child = subprocess.Popen([os.environ['MID360_NODE_EXECUTABLE'], '--ros-args',
        '--params-file', str(config), '-p', 'enable_web_editor:=true',
        '-p', 'roi_config_file:=' + str(config.resolve())], env=environment,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        deadline = time.monotonic() + 6
        while (not states or not configs) and time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.05)
            assert child.poll() is None, 'Saved YAML detector exited'
        assert states, 'No ROI state from restarted detector'
        assert {k: getattr(states[-1], k) for k in web.BOUNDS} == pytest.approx(expected)
        if algorithm:
            assert {k: getattr(configs[-1], k) for k in web.ALGORITHM} == pytest.approx(algorithm)
    finally:
        if child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill(); child.wait(timeout=3)
        observer.destroy_node()
        context.shutdown()


@pytest.mark.parametrize('detector',[{'enable_web_editor':True,
                                    'detection_enabled_on_start':False}],indirect=True)
def test_web_services_and_http(detector,tmp_path):
    h,_,_ = detector
    with socket.socket() as probe:
        probe.bind(('127.0.0.1',0))
        port=probe.getsockname()[1]
    log=(tmp_path/'web.log').open('w+')
    process=subprocess.Popen(['/usr/bin/python3',os.environ['MID360_WEB_SCRIPT'],
        '--ros-args','-p',f'web_port:={port}'],stdout=log,stderr=subprocess.STDOUT)
    url=f'http://127.0.0.1:{port}'
    try:
        def ready():
            if process.poll() is not None:
                log.seek(0);pytest.fail(log.read())
            try:return urllib.request.urlopen(url,timeout=.1).status == 200
            except OSError:return False
        h.wait(ready,timeout=10)
        h.wait(lambda:h.ros_node.count_subscribers('/mid360_obstacle_avoidance/pointcloud')>0)
        publishers=dict(h.ros_node.get_publisher_names_and_types_by_node('mid360_obstacle_avoidance','/'))
        services=dict(h.ros_node.get_service_names_and_types_by_node('mid360_obstacle_avoidance','/'))
        assert '/mid360_obstacle_avoidance/roi_state' in publishers
        assert not any('/roi_editor/' in n for n in (*publishers,*services))
        h.send(line(2.)+[(4.,0.,0.),(0.,0.,0.),(math.nan,0.,0.)])
        original=h.yaml_file.read_bytes()
        async def run():
            async with ClientSession() as session:
                async with session.get(url+'/app.js') as app:
                    assert app.status == 200 and len(await app.read()) > 8000
                    assert app.headers.get('Cache-Control') == 'no-cache'
                for path in ('/', '/status.js'):
                    async with session.get(url+path) as response:
                        assert response.status == 200
                        assert response.headers.get('Cache-Control') == 'no-cache'
                async with session.get(url+'/vendor/three.module.js') as response:
                    assert response.status==200 and 'three.core.js' in await response.text()
                async with session.ws_connect(url+'/ws') as ws:
                    async def until(predicate):
                        async def receive():
                            while True:
                                msg=await ws.receive()
                                if msg.type==WSMsgType.BINARY:value=np.frombuffer(msg.data,dtype='<f4').reshape(-1,3)
                                elif msg.type==WSMsgType.TEXT:value=json.loads(msg.data)
                                else:pytest.fail('Unexpected socket close')
                                if predicate(value):return value
                        return await asyncio.wait_for(receive(),8)
                    state=await until(lambda v:isinstance(v,dict) and v.get('roi') and
                        v.get('detection') and v.get('status'))
                    startup={k:state['roi'][k] for k in web.BOUNDS}
                    assert state['roi']['frame']=='livox_frame'
                    paused=state
                    assert paused['detection']['enabled'] is False
                    assert not paused['status']['stop_required']
                    assert paused['status']['obstacle_points']==0
                    assert paused['status']['nearest_distance'] is None
                    points=await until(lambda v:isinstance(v,np.ndarray))
                    assert len(points)==4 and tuple(points[-1])==(4.,0.,0.)
                    state=await until(lambda v:isinstance(v,dict) and v.get('topics') and
                        all(entry['sample'] for entry in v['topics'].values()))
                    assert len(state['topics']) == 7
                    topics = state['topics']
                    assert topics['status']['sample']['message']['nearest_distance'] == 'NaN'
                    assert set(topics['status']['sample']['message']) == {
                        'header','obstacle_detected','stop_required','data_valid',
                        'nearest_distance','obstacle_points','stop_zone_points'}
                    assert topics['pointcloud']['sample']['message']['width'] == 4
                    assert topics['pointcloud']['sample']['message']['header']['frame_id'] == 'livox_frame'
                    assert topics['stop_required']['sample']['message']['data'] is False
                    assert topics['roi_marker']['sample']['message']['scale']['x'] > 0
                    assert topics['detector_config']['latched']
                    sequence=0
                    async def command(name,**extra):
                        nonlocal sequence
                        sequence+=1
                        await ws.send_json(dict(command=name,id=sequence,**extra))
                        return await until(lambda v:isinstance(v,dict) and v.get('type')=='ack' and v.get('id')==sequence)
                    changed=dict(startup)
                    for boundary,value in [('min_x',.3),('max_x',1.5),('min_y',-.5),('max_y',.5),('min_z',-.2),('max_z',.8)]:
                        ack=await command('set',boundary='roi_'+boundary,value=value)
                        assert ack['success'];changed[boundary]=value
                        assert {k:ack['roi'][k] for k in web.BOUNDS}==pytest.approx(changed)
                    assert h.yaml_file.read_bytes()==original
                    assert (await command('save'))['success']
                    saved=h.yaml_file.read_bytes()
                    before=original.decode().splitlines();after=saved.decode().splitlines()
                    assert len(before)==len(after)
                    for a,b in zip(before,after):
                        key=a.strip().partition(':')[0]
                        if key.startswith('roi_') and key[4:] in web.BOUNDS:
                            assert float(b.partition(':')[2])==pytest.approx(changed[key[4:]])
                        else:assert a==b
                    await asyncio.to_thread(check_saved_restart, h.yaml_file, changed)
                    assert (await command('reset'))['success']
                    # Editing/saving while paused must not turn detection on or request a stop.
                    paused=await until(lambda v:isinstance(v,dict) and
                        (v.get('detection') or {}).get('enabled') is False and v.get('status'))
                    assert not paused['status']['stop_required']
                    # The backend must report live ROS control, not only its startup state.
                    h.control_publisher.publish(Bool(data=True))
                    enabled=await until(lambda v:isinstance(v,dict) and
                        (v.get('detection') or {}).get('enabled') is True)
                    assert enabled['detection']['age']<1
                    await asyncio.to_thread(h.send,[(4.,0.,0.)])
                    clear=await until(lambda v:isinstance(v,dict) and
                        (v.get('status') or {}).get('nearest_kind') == '+inf')
                    assert clear['topics']['status']['sample']['message']['nearest_distance'] == '+inf'
                    await asyncio.to_thread(h.send,line(0.8))
                    detected=await until(lambda v:isinstance(v,dict) and
                        (v.get('status') or {}).get('obstacle_detected') is True)
                    assert detected['status']['obstacle_points']==3
                    h.control_publisher.publish(Bool(data=False))
                    paused=await until(lambda v:isinstance(v,dict) and
                        (v.get('detection') or {}).get('enabled') is False and
                        (v.get('status') or {}).get('stop_required') is False)
                    assert not paused['status']['obstacle_detected']
                    assert paused['status']['nearest_distance'] is None
                    restored=await until(lambda v:isinstance(v,dict) and v.get('roi') and v['roi']['min_x']==startup['min_x'])
                    assert {k:restored['roi'][k] for k in web.BOUNDS}==pytest.approx(startup)
                    assert h.yaml_file.read_bytes()==saved
                    parameters = {k: restored['config'][k] for k in web.ALGORITHM}
                    parameters.update(min_obstacle_points=4, cloud_timeout=.6)
                    applied = await command('apply_config', parameters=parameters)
                    assert applied['success'] and applied['config']['min_obstacle_points'] == 4
                    saved = h.yaml_file.read_bytes()
                    await asyncio.to_thread(check_saved_restart, h.yaml_file, startup, parameters)
                    for bad in [dict(parameters, min_obstacle_points=1.5),
                                dict(parameters, cluster_distance_threshold=0),
                                dict(parameters, cloud_timeout=True),
                                dict(parameters, min_cluster_volume=-1),
                                dict(parameters, cluster_distance_threshold=1.e-300), {}]:
                        assert not (await command('apply_config', parameters=bad))['success']
                        assert h.yaml_file.read_bytes() == saved
                    assert not (await command('set',boundary='roi_bad_x',value=1))['success']
                    assert not (await command('set',boundary='roi_max_x',value=math.nan))['success']
                    for axis in 'xyz':
                        for side,value in [('min',100),('max',-100)]:
                            ack=await command('set',boundary=f'roi_{side}_{axis}',value=value)
                            assert ack['success']
                            assert ack['roi']['max_'+axis]-ack['roi']['min_'+axis]>=.01
                    ack=await command('set',boundary='roi_min_x',value=-10)
                    assert ack['roi']['min_x']==0
                    mode=h.yaml_file.stat().st_mode
                    try:
                        h.yaml_file.chmod(0o444)
                        assert not (await command('save'))['success']
                        failed = await command('apply_config', parameters=dict(parameters, min_obstacle_points=5))
                        assert not failed['success'] and failed['config']['min_obstacle_points'] == 4
                        assert h.yaml_file.read_bytes()==saved
                    finally:h.yaml_file.chmod(mode)
                    assert (await command('reset'))['success']
        asyncio.run(run())
    finally:
        process.terminate()
        try:process.wait(timeout=8)
        except subprocess.TimeoutExpired:process.kill();process.wait(timeout=3)
        log.close()
