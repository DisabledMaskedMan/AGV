// Pure presentation helpers shared by the live page and state regression tests.
export function statusView(state,connected,elapsed){
 const s=state?.status,age=state?.cloud?state.cloud.age+elapsed:Infinity;
 const known=!!(connected&&s&&s.age+elapsed<=1&&state?.detection&&state.detection.age+elapsed<=1);
 const enabled=state?.detection?.enabled,cloudFresh=connected&&age<=(state?.roi?.cloud_timeout??.5);
 const nearest=s?.nearest_kind??(s?.nearest_distance===null?'+inf':'finite');
 let safety='状态断开/超时 · 停车状态未知',style='bad';
 if(known){
  if(!enabled){safety='检测已停止 · 不请求停车';style='';}
  else if(s.stop_required){safety=s.obstacle_detected?'检测到障碍物 · 请求停车':'有效点云超时 · 请求停车';}
  else if(!s.data_valid){safety='等待有效点云 / 输入异常 · 未请求停车';style='';}
  else if(nearest==='NaN'){safety='等待新帧判定 · 未请求停车';style='';}
  else{safety='有效点云 · 无合格障碍群';style='good';}
 }
 return {
  detection:{text:known?(enabled?'检测已开始':'检测已停止'):'检测状态未知',className:'badge '+(!known?'bad':enabled?'good':'')},
  safety:{text:safety,className:'badge '+style},
  'cloud-health':{text:cloudFresh?'点云接收正常':connected?'点云断开/超时':'点云连接断开',className:'badge '+(cloudFresh?'good':'bad')},
  age:{text:Number.isFinite(age)?age.toFixed(2)+' s':'—'},
  accepted:{text:known?String(s.obstacle_points):'—'},
  nearest:{text:!known?'—':!enabled?'未检测':nearest==='NaN'?'NaN（尚未判定）':nearest==='+inf'?'+inf（无合格群）':nearest==='-inf'?'-inf':s.nearest_distance.toFixed(3)+' m'},
  cloudOpacity:cloudFresh?1:.2
 };
}

export function topicView(entry,connected,elapsed,timeout){
 if(!entry?.sample)return {label:connected?'尚未收到':'连接断开',className:'bad',age:null};
 const age=entry.sample.age+elapsed;
 if(!connected)return {label:'连接断开',className:'bad',age};
 if(!entry.latched&&age>(entry.topic.endsWith('/pointcloud')?timeout:1))
  return {label:'消息过期',className:'bad',age};
 return {label:entry.latched?'已收到 · 配置/标记按变更发布':'接收正常',className:'good',age};
}
