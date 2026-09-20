(function(G){'use strict';const C=G.CarbotCore;
class Plant{
 constructor(course,c){this.course=course;this.c=c;this.pose={...course.start};this.speed=0;this.steer=0;this.pitch=0;this.roll=0;this.z=0;this.force=0;this.updateTerrain()}
 updateTerrain(){let g=C.geometry(this.c),front=C.toWorld(this.pose,{x:g.wb,y:0}),rear=this.course.surface(this.pose.x,this.pose.y),fh=this.course.surface(front.x,front.y),left=C.toWorld(this.pose,{x:g.wb/2,y:g.track/2}),right=C.toWorld(this.pose,{x:g.wb/2,y:-g.track/2});this.z=rear;this.pitch=Math.atan2(fh-rear,g.wb);this.roll=Math.atan2(this.course.surface(left.x,left.y)-this.course.surface(right.x,right.y),g.track)}
 step(req,dt){let g=C.geometry(this.c),sd=C.clamp((req.steer-this.steer)/(this.c.steeringLagMs/1000),-this.c.steeringRateDeg*Math.PI/180,this.c.steeringRateDeg*Math.PI/180);this.steer=C.clamp(this.steer+sd*dt,-g.maxSteer,g.maxSteer);
 // Simplified speed regulator + finite motor force. No calibrated tyre/suspension claims.
 let targetForce=this.c.massKg*((req.speed-this.speed)/(this.c.speedLagMs/1000)+9.81*Math.sin(this.pitch)),limit=8;
 this.force=C.clamp(targetForce,-limit,limit);this.speed+=((this.force/this.c.massKg)-9.81*Math.sin(this.pitch))*dt;if(Math.abs(req.speed)<1e-8&&Math.abs(this.speed)<.001)this.speed=0;
 this.pose=C.bicycle(this.pose,this.speed*dt,Math.tan(this.steer)/g.wb);this.updateTerrain();return this.pose}
}
class Sensors{
 constructor(c){this.c=c;this.r=C.rng(c.seed);this.uwbR=C.rng(c.seed+1009);this.lastUwb=-1;this.packet=null;this.dropout=false;this.frozen=false;this.rawUwb=null;this.history=[]}
 sample(plant,t,dt){if(this.frozen)return this.packet;let n=this.c.noiseEnabled,gauss=()=>C.normal(this.r);let encoder=plant.speed*(1+(n?this.c.encoderScaleErrorPercent/100:0));let yaw=C.wrap(plant.pose.a+(n?gauss()*this.c.imuNoiseDeg*Math.PI/180:0)),uwb=null;if(!this.dropout&&t-this.lastUwb>=.1999){let sd=this.c.uwbNoiseEnabled?this.c.uwbNoiseCm/100:0;uwb={x:plant.pose.x+C.normal(this.uwbR)*sd,y:plant.pose.y+C.normal(this.uwbR)*sd};this.rawUwb={...uwb,stamp:t};this.history.push(this.rawUwb);if(this.history.length>100)this.history.shift();this.lastUwb=t}this.packet={stamp:t,encoder,yaw,pitch:plant.pitch,roll:plant.roll,uwb};return this.packet}
}
class Estimator{
 constructor(start,c){this.c=c;this.odom={...start};this.transform={x:0,y:0};this.pose={...start};this.globalOffset={x:0,y:0};this.globalPose={...start};this.globalVariance=.01**2;this.globalSigma=.01;this.sigma=.003;this.lastStamp=-1;this.uwbAge=0;this.residual=0;this.history=[];this.uwbGain=0;this.lastVisual=-1;this.visualMatches=0;this.visualRank=0;this.distance=0;}
 update(s,t,dt){if(!s||s.stamp===this.lastStamp){this.sigma+=dt*.004;return}this.lastStamp=s.stamp;
 const ds=s.encoder*dt;this.distance+=Math.abs(ds);this.odom.x+=ds*Math.cos(this.odom.a);this.odom.y+=ds*Math.sin(this.odom.a);this.odom.a=C.wrap(this.odom.a+.25*C.wrap(s.yaw-this.odom.a));
 this.sigma=Math.hypot(this.sigma,Math.abs(ds)*.025,Math.sqrt(dt)*.00025);this.uwbAge+=dt;
 this.globalVariance+=dt*.000008+Math.abs(ds)*.00002;
 this.refresh();
 if(s.uwb){let ex=s.uwb.x-this.globalPose.x,ey=s.uwb.y-this.globalPose.y,sd=this.c.uwbNoiseEnabled?this.c.uwbNoiseCm/100:0,R=Math.max(.005,sd)**2,S=this.globalVariance+R;
 this.residual=Math.hypot(ex,ey);this.lastAccepted=(ex*ex+ey*ey)/S<13.82;this.uwbGain=0;
 if(this.lastAccepted){let K=this.globalVariance/S;this.uwbGain=K;this.globalOffset.x+=K*ex;this.globalOffset.y+=K*ey;this.globalVariance=(1-K)*this.globalVariance;this.uwbAge=0}}
 this.refresh();}
 refresh(){this.pose={x:this.odom.x+this.transform.x,y:this.odom.y+this.transform.y,a:this.odom.a};this.globalSigma=Math.sqrt(this.globalVariance);this.globalPose={x:this.pose.x+this.globalOffset.x,y:this.pose.y+this.globalOffset.y,a:this.pose.a};}
 // Camera-to-prior boundary registration. UWB NEVER writes the local transform.
 visualUpdate(perception,course,t,enabled=true){if(!enabled)return;let rows=[];const n=perception.n,h=.012;
 for(let i=n+1;i<perception.kind.length-n-1;i+=2){if(!perception.grown[i])continue;let p=perception.localPoint(i);if(Math.hypot(p.x,p.y)>.70)continue;
 for(const j of [i-1,i+1,i-n,i+n]){if(perception.kind[j]!==2)continue;let q=perception.localPoint(j),edge={x:(p.x+q.x)/2,y:(p.y+q.y)/2},w=C.toWorld(this.pose,edge),d=course.clearance(w.x,w.y);if(Math.abs(d)>.045)continue;
 let gx=(course.clearance(w.x+h,w.y)-course.clearance(w.x-h,w.y))/(2*h),gy=(course.clearance(w.x,w.y+h)-course.clearance(w.x,w.y-h))/(2*h),norm=Math.hypot(gx,gy);if(norm<.75||norm>1.2)continue;rows.push({gx,gy,d});}}
 this.visualMatches=rows.length;if(rows.length<8){this.visualRank=0;return}
 let xx=.8,xy=0,yy=.8,bx=0,by=0;for(let r of rows){let w=Math.min(1,.015/Math.max(.001,Math.abs(r.d)));xx+=w*r.gx*r.gx;xy+=w*r.gx*r.gy;yy+=w*r.gy*r.gy;bx-=w*r.gx*r.d;by-=w*r.gy*r.d}
 let det=xx*yy-xy*xy,dx=(yy*bx-xy*by)/det,dy=(xx*by-xy*bx)/det,scale=Math.min(.12,.0015/Math.max(.00001,Math.hypot(dx,dy)));
 this.transform.x+=dx*scale;this.transform.y+=dy*scale;this.lastVisual=t;this.visualRank=4*det/(xx+yy)**2;
 // Scalar shown here is local lateral confidence, not absolute 2-D accuracy.
 this.sigma=Math.max(.004,this.sigma*.94);this.refresh();
 // A well-conditioned visual landmark also informs the coarse global estimate.
 if(this.visualRank>.25){const R=.025**2,K=this.globalVariance/(this.globalVariance+R);this.globalOffset.x*=1-K;this.globalOffset.y*=1-K;this.globalVariance*=1-K;this.refresh()}
 }
}
class LocalMemory{
 constructor(){this.cells=new Map();this.res=.025;this.last=0;this.fresh=0}
 integrate(grid,odom,t,distance=0){let count=0;for(let i=0;i<grid.kind.length;i++){let k=grid.kind[i];if(k!==1&&k!==2)continue;let p=grid.localPoint(i),w=C.toWorld(odom,p),key=Math.round(w.x/this.res)+','+Math.round(w.y/this.res);this.cells.set(key,{x:w.x,y:w.y,kind:k,stamp:t,distance});count++}this.last=t;this.fresh=count;if(this.cells.size>26000)this.expire(t);}
 expire(t){for(const [key,v]of this.cells)if(t-v.stamp>25)this.cells.delete(key)}
 query(x,y,t){let v=this.cells.get(Math.round(x/this.res)+','+Math.round(y/this.res));return v&&t-v.stamp<25?v:null}
}
function closest(path,p,from=0){let best=Infinity,index=from;for(let i=Math.max(0,from-12);i<Math.min(path.length,from+130);i++){let q=path[i],d=(q.x-p.x)**2+(q.y-p.y)**2;if(d<best){best=d;index=i}}return{index,error:Math.sqrt(best)}}
class Controller{
 constructor(c){this.c=c;this.index=0;this.gear=1;this.gearHold=0;this.error=0;this.target=null;this.lastPlan=0;this.request=null;this.candidates=[];this.localPath=[];this.finalGoal=null}
 reset(){this.index=0;this.gear=1;this.gearHold=0}
 update(path,pose,speed,t,parking=false){let found=closest(path,pose,this.index);this.index=found.index;this.error=found.error;let end=path.at(-1),remain=Math.hypot(pose.x-end.x,pose.y-end.y),endIndex=this.index>path.length-8;
 let endRelative=C.toLocal(end,pose),endDirection=path.at(-1).dir||1;
 if(endIndex&&(remain<(parking?.006:.020)||(endRelative.x*endDirection>-.004&&Math.abs(endRelative.y)<(parking?.060:.025)))){this.request={speed:0,steer:0,stamp:t,source:parking?'PARKING':'ROAD',mode:parking?'parking':'road'};return{...this.request,arrived:Math.abs(speed)<.002}}
 let direction=path[Math.min(this.index+2,path.length-1)].dir||1;if(direction!==this.gear){if(Math.abs(speed)>.006){return{speed:0,steer:0,stamp:t,source:'GEAR STOP',mode:'parking'}}if(!this.gearHold)this.gearHold=t+.35;if(t<this.gearHold)return{speed:0,steer:0,stamp:t,source:'GEAR STOP',mode:'parking'};this.gear=direction;this.gearHold=0;}
 let look=parking?.065:.095,k=this.index;while(k<path.length-1&&Math.hypot(path[k].x-pose.x,path[k].y-pose.y)<look&&(path[k+1].dir||1)===direction)k++;this.target={...path[k]};if(k===path.length-1&&remain<look){let d=look-remain;this.target.x+=direction*d*Math.cos(end.a);this.target.y+=direction*d*Math.sin(end.a)}let q=C.toLocal(pose,this.target),g=C.geometry(this.c),curv=2*q.y/Math.max(.0004,q.x*q.x+q.y*q.y),delta=C.clamp(Math.atan(g.wb*curv),-g.maxSteer,g.maxSteer);
 let v=(parking?this.c.parkingSpeedCms:this.c.maxSpeedCms)/100;v=Math.min(v,.07+.07/(1+Math.abs(curv)));if(endIndex)v=Math.min(v,Math.max(.018,remain*1.8));let gearEnd=path.findIndex((p,i)=>i>this.index&&(p.dir||1)!==direction);if(gearEnd>this.index&&gearEnd-this.index<14)v=Math.min(v,.025);
 this.localPath=[{...pose},...path.slice(this.index+1,Math.min(path.length,this.index+90))];this.finalGoal=end;this.lastPlan=t;this.request={speed:v*direction,steer:delta,stamp:t,source:parking?'PARKING':'ROAD',mode:parking?'parking':'road'};return this.request;
}
}
function arbitrate(request,t,safety,c){if(safety)return{speed:0,steer:0,winner:'SAFETY STOP',reason:safety};if(!request||t-request.stamp>c.expiryMs/1000)return{speed:0,steer:0,winner:'WATCHDOG',reason:'Request older than 200 ms'};return{...request,winner:request.source,reason:'Fresh authorised request'}}
function splitGears(path){let out=[],cur=[];for(let p of path){if(cur.length&&cur.at(-1).dir!==p.dir){out.push(cur);cur=[{...cur.at(-1),dir:p.dir}]}cur.push(p)}if(cur.length)out.push(cur);return out}
G.CarbotVehicle={Plant,Sensors,Estimator,LocalMemory,Controller,arbitrate,closest,splitGears};
})(globalThis);
