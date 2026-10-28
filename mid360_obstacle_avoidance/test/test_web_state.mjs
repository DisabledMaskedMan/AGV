import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
const source=readFileSync(new URL('../web/status.js',import.meta.url),'utf8');
const {statusView,topicView}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
function render(overrides={},connected=true,elapsed=0){
 const state={roi:{cloud_timeout:.5},cloud:{age:0},detection:{enabled:true,age:0},
  status:{age:0,data_valid:true,stop_required:false,obstacle_detected:false,
   obstacle_points:0,nearest_kind:'+inf',nearest_distance:null}};
 for(const [key,value] of Object.entries(overrides))state[key]={...state[key],...value};
 return statusView(state,connected,elapsed);
}
let v=render({detection:{enabled:false},status:{data_valid:false,stop_required:false},cloud:{age:5}});
assert.equal(v.detection.text,'检测已停止');assert.match(v.safety.text,/不请求停车/);
assert.equal(v.nearest.text,'未检测');assert.equal(v['cloud-health'].text,'点云断开/超时');
v=render({status:{nearest_kind:'NaN'}});assert.match(v.safety.text,/等待新帧.*未请求停车/);
assert.match(v.nearest.text,/NaN/);
v=render({status:{data_valid:false}});assert.match(v.safety.text,/输入异常.*未请求停车/);
v=render({status:{data_valid:false,stop_required:true}});assert.match(v.safety.text,/有效点云超时.*请求停车/);
v=render({status:{stop_required:true,obstacle_detected:true,nearest_kind:'finite',nearest_distance:.8,obstacle_points:3}});
assert.match(v.safety.text,/检测到障碍物/);assert.equal(v.nearest.text,'0.800 m');assert.equal(v.accepted.text,'3');
v=render();assert.match(v.safety.text,/无合格障碍/);assert.match(v.nearest.text,/[+]inf/);
assert.equal(render({status:{nearest_kind:'-inf'}}).nearest.text,'-inf');
for(const s of [{status:{age:2}},{detection:{age:2}}]){
 v=render(s);assert.equal(v.detection.text,'检测状态未知');assert.match(v.safety.text,/停车状态未知/);
}
assert.equal(render({},false).detection.text,'检测状态未知');
assert.equal(render({},true,2).detection.text,'检测状态未知');
assert.match(statusView(null,false,0).safety.text,/未知/);
const entry={topic:'/mid360_obstacle_avoidance/status',latched:false,sample:{age:2}};
assert.equal(topicView(entry,true,0,.5).label,'消息过期');
assert.match(topicView({...entry,latched:true},true,500,.5).label,/已收到/);
assert.equal(topicView(entry,false,0,.5).label,'连接断开');
assert.equal(topicView(null,true,0,.5).label,'尚未收到');
assert.equal(topicView({...entry,topic:'/mid360_obstacle_avoidance/pointcloud',sample:{age:.7}},true,0,.5).label,'消息过期');
// Exercise the page's real update handler: server refresh must preserve drafts/focus.
const app=readFileSync(new URL('../web/app.js',import.meta.url),'utf8');
const keys=['cluster_distance_threshold','min_obstacle_points','min_cluster_diagonal','min_cluster_volume','cloud_timeout'];
const inputs=new Map(keys.map(k=>[k,{value:'draft'}])),actual=new Map(keys.map(k=>[k,{}]));
const revision={};
const ctx={currentConfig:null,algorithm:keys.map(k=>[k]),actualLabels:actual,
 algorithmInputs:inputs,dirty:new Set(['min_obstacle_points']),
 document:{activeElement:inputs.get('cloud_timeout')},$:()=>revision,updateEnabled:()=>{}};
vm.createContext(ctx);
vm.runInContext(app.slice(app.indexOf('function applyConfig('),app.indexOf('function updateEnabled(')),ctx);
const config=Object.fromEntries(keys.map((k,i)=>[k,i+1]));config.revision=1;
ctx.applyConfig(config);
assert.equal(inputs.get('min_obstacle_points').value,'draft');
assert.equal(inputs.get('cloud_timeout').value,'draft');
assert.equal(inputs.get('min_cluster_diagonal').value,3);
assert.equal(actual.get('min_obstacle_points').textContent,'当前生效：2');
ctx.applyConfig({...config,revision:0,min_cluster_diagonal:99});
assert.equal(inputs.get('min_cluster_diagonal').value,3);
ctx.dirty.clear();ctx.document.activeElement=null;ctx.applyConfig(config);
assert.equal(inputs.get('min_obstacle_points').value,2);
console.log('Web status, topic freshness and draft preservation scenarios passed');
