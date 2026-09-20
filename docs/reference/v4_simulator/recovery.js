// Bounded recovery for motion-planning deadlocks. No true vehicle pose is read.
(function(G){
 const C=G.CarbotCore,V=G.CarbotVehicle;
 function roadClear(course,p,c,tolerance=0,pad=.005){
  const g=C.geometry(c),xs=[-g.rear-pad,g.front+pad],ys=[-g.w/2-pad,g.w/2+pad];
  const ok=(x,y)=>{const q=C.toWorld(p,{x,y});return course.clearance(q.x,q.y)>=-tolerance};
  for(let x=xs[0];x<=xs[1];x+=.025)for(const y of ys)if(!ok(x,y))return false;
  for(let y=ys[0];y<=ys[1];y+=.025)for(const x of xs)if(!ok(x,y))return false;
  return xs.every(x=>ys.every(y=>ok(x,y)));
 }
 function obstaclesClear(sim,p){
  const origin=C.toWorld(sim.estimate.pose,{x:.09,y:0}),g=C.geometry(sim.c);
  for(const hit of sim.lidar){if(hit.range>=1.999)continue;const a=sim.estimate.pose.a+hit.angle,w={x:origin.x+hit.range*Math.cos(a),y:origin.y+hit.range*Math.sin(a)},q=C.toLocal(p,w);if(q.x>-g.rear-.02&&q.x<g.front+.02&&Math.abs(q.y)<g.w/2+.02)return false;}
  return true;
 }
 function rearEvidence(sim,path){
  let known=0,total=0;const g=C.geometry(sim.c);
  for(let i=0;i<path.length;i+=5){const p=path[i];if(p.dir!==-1)continue;for(const y of [-g.w*.35,0,g.w*.35]){const w=C.toWorld(p,{x:-g.rear-.025,y}),e=G.CarbotGuidance.evidence(sim,w,3);total++;if(e&&(e.kind===1||(e.kind===2&&sim.course.clearance(w.x,w.y)>=-(sim.c.lineToleranceCm||0)/100)))known++;}}
  return total>0&&known/total>=.55;
 }
 function search(sim){
  const start=sim.estimate.pose,g=C.geometry(sim.c),maxReverse=sim.c.recoveryReverseCm/100,tol=sim.c.lineToleranceCm/100;
  const from=V.closest(sim.path,start,sim.ctrl.index).index,goals=[];let travelled=0,last=sim.path[from],next=.25;
  for(let i=from+1;i<sim.path.length&&goals.length<5;i++){const q=sim.path[i];travelled+=Math.hypot(q.x-last.x,q.y-last.y);last=q;if(travelled>=next){if(sim.course.bodyClear(q,sim.c,.008))goals.push({pose:q,index:i});next+=.15;}}
  const evaluated=[];
  for(const goal of goals){for(const q of G.CarbotRS.candidates(start,goal.pose,g.r)){
   let reverse=q.lengths.filter(d=>d<0).reduce((a,d)=>a-d,0),parts=V.splitGears(q.path),reason='';
   if(q.path[0].dir!==-1||q.path.at(-1).dir!==1)reason='Recovery must reverse first, then rejoin forward';
   else if(reverse<.03||reverse>maxReverse||parts.length!==2)reason='Reverse distance / gear-change limit';
   else if(q.cost>1.6)reason='Recovery manoeuvre too long';
   else if(!q.path.every(p=>roadClear(sim.course,p,sim.c,tol)))reason='Footprint exceeds paint allowance';
   else if(!q.path.every(p=>obstaclesClear(sim,p)))reason='LiDAR obstacle in swept footprint';
   else if(!rearEvidence(sim,q.path))reason='Insufficient observed rear road';
   const outside=q.path.reduce((a,p)=>a+Math.max(0,-sim.course.bodyMargin(p)),0)*.006;
   evaluated.push({id:evaluated.length,points:q.path,valid:!reason,reject:reason,cost:q.cost+reverse*.7+outside*150,reverse,goalIndex:goal.index,types:q.types});
  }}
  evaluated.sort((a,b)=>Number(b.valid)-Number(a.valid)||a.cost-b.cost);
  return {evaluated,winner:evaluated.find(q=>q.valid)||null};
 }
 class Recovery{
  constructor(c){this.c=c;this.active=false;this.state='IDLE';this.reason='';this.problemSince=null;this.attempts=0;this.completed=0;this.evaluated=[];this.path=[];this.lastFinish=-100;this.lastFinishPose=null;}
  start(sim){if(this.lastFinishPose&&Math.hypot(sim.estimate.pose.x-this.lastFinishPose.x,sim.estimate.pose.y-this.lastFinishPose.y)>.15)this.attempts=0;this.active=true;this.state='BRAKE';this.reason='Stopping before reverse-and-rejoin search';this.since=sim.t;sim.event('RECOVERY BRAKE',this.reason);}
  request(sim,problem,hardHold,permitted){
   if(!this.c.recoveryEnabled)return null;
   if(!this.active){if(!problem||!permitted||hardHold){this.problemSince=null;return null;}if(this.problemSince===null)this.problemSince=sim.t;if(sim.t-this.problemSince<.45)return null;this.start(sim);}
   const zero=()=>({speed:0,steer:0,source:'RECOVERY',stamp:sim.t,mode:'recovery'});
   if(!hardHold&&!problem&&(this.state==='WAIT'||this.state==='BRAKE')){this.active=false;this.state='IDLE';this.problemSince=null;sim.event('RECOVERY CANCELLED','A forward candidate is feasible again; normal planning resumes');return null;}
   if(hardHold){this.reason='Recovery paused: '+hardHold;return zero();}
   if(sim.t-sim.lastLidar>.35||sim.t-sim.perception.stamp>.3){this.reason='Waiting for fresh rear camera and LiDAR';return zero();}
   if(this.state==='BRAKE'||this.state==='WAIT'){
    if(Math.abs(sim.sensor.packet?.encoder||0)>.003)return zero();
    if(this.state==='WAIT'&&sim.t<this.retryAt)return zero();
    if(this.attempts>=3){this.reason='Recovery limit reached here; clear the obstruction or reset';return zero();}
    const result=search(sim);this.evaluated=result.evaluated;this.planStamp=sim.t;
    if(!result.winner){this.state='WAIT';this.reason='No checked reverse-and-rejoin route; rescanning every 2 s';this.retryAt=sim.t+2;sim.event('RECOVERY WAIT',this.reason);return zero();}
    this.attempts++;this.selected=result.winner;this.path=result.winner.points;this.parts=V.splitGears(this.path);this.current=this.parts.shift();this.goalIndex=result.winner.goalIndex;this.ctrl=new V.Controller({...this.c,parkingSpeedCms:3});this.ctrl.gear=this.current[0].dir;this.state='ALIGN';this.alignUntil=sim.t+.4;sim.event('RECOVERY PATH SELECTED',`${Math.round(result.winner.reverse*100)} cm reverse, then forward rejoin; ${result.evaluated.length} analytic connections evaluated.`);return zero();
   }
   if(this.state==='ALIGN'){if(sim.t<this.alignUntil)return {...zero(),steer:Math.atan(C.geometry(this.c).wb*(this.current[1]?.k||0))};this.state='TRACK';}
   const req=this.ctrl.update(this.current,sim.estimate.pose,sim.sensor.packet?.encoder||0,sim.t,true);req.source='RECOVERY';req.mode='recovery';req.speed=C.clamp(req.speed,-.03,.03);
   const look=this.current.slice(this.ctrl.index,Math.min(this.current.length,this.ctrl.index+18));
   if(look.some(p=>!roadClear(sim.course,p,this.c,this.c.lineToleranceCm/100)||!obstaclesClear(sim,p))){this.state='BRAKE';this.reason='Recovery clearance changed; stopping to recompute';return zero();}
   this.reason=this.current.at(-1).dir<0?'Reversing slowly along checked path':'Driving forward onto the mission route';
   if(req.arrived){
    if(this.parts.length){this.current=this.parts.shift();this.ctrl.reset();this.ctrl.gear=this.current[0].dir;this.state='ALIGN';this.alignUntil=sim.t+.4;sim.event('RECOVERY GEAR CHANGE','Stopped before changing from reverse to forward');return zero();}
    this.active=false;this.state='IDLE';this.problemSince=null;this.completed++;this.lastFinish=sim.t;this.lastFinishPose={...sim.estimate.pose};sim.ctrl.reset();sim.ctrl.index=Math.max(0,this.goalIndex-2);sim.lastLocalPlan=-1;sim.event('RECOVERY COMPLETE','Rejoined the forward route; local lane planner resumes');return zero();
   }
   return req;
  }
 }
 G.CarbotRecovery={roadClear,obstaclesClear,rearEvidence,search,Recovery};
})(globalThis);
