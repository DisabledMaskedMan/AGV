import * as THREE from 'three';
import {OrbitControls} from './vendor/OrbitControls.js';
import {TransformControls} from './vendor/TransformControls.js';
import {statusView,topicView} from './status.js';
const $=id=>document.getElementById(id), keys=['min_x','max_x','min_y','max_y','min_z','max_z'];
const viewport=$('viewport'), scene=new THREE.Scene();
scene.background=new THREE.Color('#101821');
const camera=new THREE.PerspectiveCamera(55,1,.01,2000);camera.up.set(0,0,1);
const renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));viewport.prepend(renderer.domElement);
const orbit=new OrbitControls(camera,renderer.domElement);
function resetView(){camera.position.set(4,-4,3);orbit.target.set(1,0,.2);orbit.update();}
resetView();$('view').onclick=resetView;
const grid=new THREE.GridHelper(20,40,'#3f6178','#243746');grid.rotation.x=Math.PI/2;scene.add(grid,new THREE.AxesHelper(1));
const geometry=new THREE.BoxGeometry(1,1,1);
const box=new THREE.Mesh(geometry,new THREE.MeshBasicMaterial({color:'#24c8df',transparent:true,opacity:.12,depthWrite:false}));
box.add(new THREE.LineSegments(new THREE.EdgesGeometry(geometry),new THREE.LineBasicMaterial({color:'#58d9ec'})));scene.add(box);box.visible=false;
const cloudGeometry=new THREE.BufferGeometry(), cloud=new THREE.Points(cloudGeometry,new THREE.PointsMaterial({color:'#c5e9ff',size:.025,transparent:true}));
cloud.frustumCulled=false;scene.add(cloud);
let socket,connected=false,state=null,stateAt=0,currentRoi=null,dragging=null,actionBusy=false,requestId=0,running=null;
let currentConfig=null;
const algorithm=[
 ['cluster_distance_threshold','相邻点最大距离','m',0,false],
 ['min_obstacle_points','每群最少点数','点',1,true],
 ['min_cluster_diagonal','最小点群对角线','m',0,false],
 ['min_cluster_volume','最小包围盒体积','m³；0 关闭',0,false],
 ['cloud_timeout','点云超时','s',0,false]];
const algorithmInputs=new Map(),actualLabels=new Map(),dirty=new Set(),topicElements=new Map();
const pending=new Map(),requests=new Map(),handles=new Map(),inputs=new Map();
const message=text=>{$('message').textContent=text;};
for(const key of keys){
 const label=document.createElement('label');label.textContent=key.at(-1).toUpperCase()+(key.startsWith('min')?' 最小边界':' 最大边界');
 const english=document.createElement('small');english.textContent='（roi_'+key+'） · m';label.append(english);
 const input=document.createElement('input');input.type='number';input.step='.01';input.disabled=true;label.append(input);$('fields').append(label);inputs.set(key,input);
 input.onchange=()=>{if(Number.isFinite(input.valueAsNumber))queueBoundary(key,input.valueAsNumber);else message('请输入有限数值');};
 const axis=key.at(-1),color={x:0xf27777,y:0x79df96,z:0x82aaff}[axis];
 const mesh=new THREE.Mesh(new THREE.SphereGeometry(.045,14,10),new THREE.MeshBasicMaterial({color}));mesh.visible=false;scene.add(mesh);
 const control=new TransformControls(camera,renderer.domElement);control.setMode('translate');control.setSpace('world');control.setSize(.45);
 control.showX=axis==='x';control.showY=axis==='y';control.showZ=axis==='z';control.attach(mesh);control.enabled=false;scene.add(control.getHelper());control.getHelper().visible=false;
 control.addEventListener('dragging-changed',event=>{orbit.enabled=!event.value;dragging=event.value?key:null;if(!event.value)queueBoundary(key,mesh.position[axis]);updateEnabled();});
 control.addEventListener('objectChange',()=>{if(dragging===key)queueBoundary(key,mesh.position[axis]);});
 handles.set(key,{mesh,control});
}
for(const [key,title,unit,min,integer] of algorithm){
 const label=document.createElement('label');label.append(document.createTextNode(title));
 const english=document.createElement('small');english.textContent='（'+key+'） · '+unit;label.append(english);
 const input=document.createElement('input');input.type='number';input.step=integer?'1':'any';input.min=String(min);input.disabled=true;
 if(integer)input.max='4294967295';
 input.oninput=()=>{dirty.add(key);input.classList.add('dirty');};
 const actual=document.createElement('small');actual.textContent='当前生效：等待配置';label.append(input,actual);
 $('algorithm-fields').append(label);algorithmInputs.set(key,input);actualLabels.set(key,actual);
}
function applyConfig(config){
 if(!config||(currentConfig&&config.revision<currentConfig.revision))return;
 currentConfig=config;
 for(const [key] of algorithm){
  actualLabels.get(key).textContent='当前生效：'+config[key];
  if(!dirty.has(key)&&document.activeElement!==algorithmInputs.get(key))algorithmInputs.get(key).value=config[key];
 }
 $('config-revision').textContent='实际配置版本 revision：'+config.revision;updateEnabled();
}
function updateEnabled(){
 const allowed=connected&&!!currentRoi&&!actionBusy;
 $('save').disabled=$('reset').disabled=!allowed||!!dragging;
 for(const input of inputs.values())input.disabled=!allowed||!!dragging;
 for(const [key,item] of handles)item.control.enabled=allowed&&(!dragging||dragging===key);
 const canConfigure=allowed&&!!currentConfig&&!dragging;
 $('apply-config').disabled=!canConfigure;
 for(const input of algorithmInputs.values())input.disabled=!canConfigure;
}
function applyRoi(roi){
 if(!roi||(currentRoi&&roi.revision<currentRoi.revision))return;
 currentRoi=roi;
 const center=['x','y','z'].map(a=>(roi['min_'+a]+roi['max_'+a])/2);
 box.visible=true;box.position.fromArray(center);box.scale.set(...['x','y','z'].map(a=>roi['max_'+a]-roi['min_'+a]));
 for(const [key,{mesh,control}] of handles){
  mesh.visible=true;control.getHelper().visible=true;
  if(dragging!==key){mesh.position.fromArray(center);mesh.position[key.at(-1)]=roi[key];}
  if(document.activeElement!==inputs.get(key))inputs.get(key).value=Number(roi[key].toFixed(5));
 }
 $('frame').textContent=roi.frame;updateEnabled();
}
function command(name,extra={}){
 if(!connected)return Promise.reject(new Error('WebSocket 已断开'));
 const id=++requestId;
 return new Promise((resolve,reject)=>{
  const timer=setTimeout(()=>{requests.delete(id);reject(new Error('操作超时，请核实当前边界后再操作'));},7000);
  requests.set(id,{resolve,reject,timer});socket.send(JSON.stringify({id,command:name,...extra}));
 });
}
function queueBoundary(key,value){
 if(!connected||actionBusy||!Number.isFinite(value))return;
 pending.set(key,value);
 if(!running){
  running=(async()=>{
   while(pending.size){
    const [key,value]=pending.entries().next().value;pending.delete(key);
    const ack=await command('set',{boundary:'roi_'+key,value});applyRoi(ack.roi);
    if(!ack.success)throw new Error(ack.message);
   }
   message('ROI 已在内存生效，尚未保存。');
  })();
  running.catch(error=>{pending.clear();message(error.message);}).finally(()=>{running=null;if(!dragging&&currentRoi)applyRoi(currentRoi);});
 }
}
async function action(name,extra={}){
 actionBusy=true;updateEnabled();
 try{if(running)await running;const ack=await command(name,extra);if(!ack.success)throw new Error(ack.message);message(ack.message);return ack;}
 catch(error){message(error.message);}
 finally{actionBusy=false;updateEnabled();}
}
$('save').onclick=()=>action('save');$('reset').onclick=()=>action('reset');
$('apply-config').onclick=async()=>{
 const parameters={};
 for(const [key, , ,min,integer] of algorithm){
  const value=algorithmInputs.get(key).valueAsNumber;
  if(!Number.isFinite(value)||value<min||(integer&&(!Number.isInteger(value)||value>4294967295))||
    (['cluster_distance_threshold','cloud_timeout'].includes(key)&&value<=0)){message('参数数值非法：'+key);return;}
  parameters[key]=value;
 }
 const ack=await action('apply_config',{parameters});
 if(ack){dirty.clear();for(const input of algorithmInputs.values())input.classList.remove('dirty');applyConfig(ack.config);}
};
function connect(){
 socket=new WebSocket((location.protocol==='https:'?'wss://':'ws://')+location.host+'/ws');socket.binaryType='arraybuffer';
 socket.onopen=()=>{connected=true;$('connection').textContent='已连接板卡';$('connection').className='badge good';updateEnabled();};
 socket.onmessage=event=>{
  if(event.data instanceof ArrayBuffer){
   if(event.data.byteLength%12){message('点云数据长度错误');return;}
   const xyz=new Float32Array(event.data);let attribute=cloudGeometry.getAttribute('position');
   if(!attribute||attribute.array.length<xyz.length){cloudGeometry.dispose();attribute=new THREE.BufferAttribute(new Float32Array(Math.max(3,xyz.length*2)),3);attribute.setUsage(THREE.DynamicDrawUsage);cloudGeometry.setAttribute('position',attribute);}
   attribute.array.set(xyz);attribute.needsUpdate=true;cloudGeometry.setDrawRange(0,xyz.length/3);return;
  }
  let data;try{data=JSON.parse(event.data);}catch{return;}
  if(data.type==='ack'){
   if(data.roi)applyRoi(data.roi);if(data.config)applyConfig(data.config);const request=requests.get(data.id);
   if(request){clearTimeout(request.timer);requests.delete(data.id);request.resolve(data);}
  }else if(data.type==='state'){
   state=data;stateAt=performance.now();applyRoi(data.roi);applyConfig(data.config);const c=data.cloud;
   $('count').textContent=c?c.points.toLocaleString():'等待点云';
   $('stamp').textContent=c?c.sec+'.'+String(c.nanosec).padStart(9,'0'):'—';
  }
 };
 socket.onclose=()=>{
  connected=false;currentRoi=null;currentConfig=null;pending.clear();
  for(const request of requests.values()){clearTimeout(request.timer);request.reject(new Error('连接断开，请重新确认操作结果'));}
  requests.clear();$('connection').textContent='连接断开 · 正在重连';$('connection').className='badge bad';updateEnabled();setTimeout(connect,1500);
 };
 socket.onerror=()=>socket.close();
}
setInterval(()=>{
 const elapsed=(performance.now()-stateAt)/1000,view=statusView(state,connected,elapsed);
 for(const [id,display] of Object.entries(view)){
  if(id==='cloudOpacity'){cloud.material.opacity=display;continue;}
  $(id).textContent=display.text;if(display.className!==undefined)$(id).className=display.className;
 }
 for(const [key,entry] of Object.entries(state?.topics??{})){
  if(!topicElements.has(key)){
   const details=document.createElement('details'),summary=document.createElement('summary'),meta=document.createElement('div'),pre=document.createElement('pre');
   details.open=['status','stop_required','detection_enabled'].includes(key);
   summary.textContent=entry.topic;details.append(summary,meta,pre);$('topics').append(details);topicElements.set(key,{meta,pre});
  }
  const {meta,pre}=topicElements.get(key),v=topicView(entry,connected,elapsed,state?.roi?.cloud_timeout??.5);
  meta.className='topic-meta '+v.className;
  meta.textContent=entry.type+' · '+v.label+(v.age===null?'':' · '+v.age.toFixed(2)+' s 前 · 共 '+entry.sample.count+' 条 · '+new Date(entry.sample.received_time*1000).toLocaleString());
  const payload=entry.sample?JSON.stringify(entry.sample.message,null,2):'尚未收到消息';
  if(pre.textContent!==payload)pre.textContent=payload;
 }
},100);
new ResizeObserver(()=>{const w=viewport.clientWidth,h=viewport.clientHeight;renderer.setSize(w,h);camera.aspect=w/h;camera.updateProjectionMatrix();}).observe(viewport);
renderer.setAnimationLoop(()=>{orbit.update();renderer.render(scene,camera);});connect();
