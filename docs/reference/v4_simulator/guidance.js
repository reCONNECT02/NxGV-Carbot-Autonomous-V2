// Route-conditioned camera corridor. No access to the simulated true pose.
(function(G){
 const C=G.CarbotCore;
 function evidence(sim,w,maxAge=3){
  const od={x:w.x-sim.estimate.transform.x,y:w.y-sim.estimate.transform.y};
  if(sim.t-sim.perception.stamp<.30){const rel=C.toLocal(sim.cameraOdom||sim.estimate.odom,od),k=sim.perception.support(rel);if(k)return {kind:k===1?1:2,age:sim.t-sim.perception.stamp,live:true};}
  const m=sim.memory.query(od.x,od.y,sim.t);if(m&&sim.t-m.stamp<maxAge){const uncertainty=.004+.006*Math.abs(sim.estimate.distance-(m.distance||0));if(uncertainty<.02)return {...m,age:sim.t-m.stamp,uncertainty,live:false};}
  return null;
 }
 function corridor(sim){
  const C=G.CarbotCore,start=sim.estimate.pose,path=sim.path,first=sim.ctrl.index,pts=[],centres=[];let travelled=0,previous=path[first];
  for(let j=first;j<path.length&&travelled<.9;j++){
   const q=path[j];if(previous)travelled+=Math.hypot(q.x-previous.x,q.y-previous.y);previous=q;
   if(j%5!==0&&j!==first)continue;
   let nx=-Math.sin(q.a),ny=Math.cos(q.a),edge=[];
   for(const sign of [-1,1]){let found=null;for(let d=.055;d<=.26;d+=.009){let w={x:q.x+sign*d*nx,y:q.y+sign*d*ny},e=evidence(sim,w,2);
    if(e?.kind===2){let inside={x:w.x-sign*.027*nx,y:w.y-sign*.027*ny};if(evidence(sim,inside,2)?.kind===1){found={d:sign*(d-.0045),w};break}}}
    edge.push(found);
   }
   let shift=0,seen=false;
   if(edge.every(Boolean)){
    const width=edge[1].d-edge[0].d;
    // Narrow lane observations only. Broad openings/dividers do not become fake boundaries.
    if(width>.23&&width<.38){let expected=[];
     for(const sign of [-1,1]){let d=.02;for(;d<.32;d+=.005)if(sim.course.clearance(q.x+sign*d*nx,q.y+sign*d*ny)<0)break;expected.push(sign*d)}
     const measuredMid=(edge[0].d+edge[1].d)/2,priorMid=(expected[0]+expected[1])/2;
     shift=C.clamp(measuredMid-priorMid,-.025,.025)*.5;seen=true;
     centres.push({x:q.x+measuredMid*nx,y:q.y+measuredMid*ny,a:q.a,width});
    }
   }
   pts.push({...q,x:q.x+shift*nx,y:q.y+shift*ny,observed:seen,shift});
  }
  // A common residual offset avoids a sawtooth target at camera occlusion gaps.
  const shifts=pts.filter(p=>p.observed).map(p=>p.shift).sort((a,b)=>a-b),offset=shifts.length?shifts[Math.floor(shifts.length/2)]:0;
  const guide=path.slice(first,Math.min(path.length,first+120)).map(q=>({...q,x:q.x-offset*Math.sin(q.a),y:q.y+offset*Math.cos(q.a)}));
  const age=sim.t-sim.perception.stamp;
  return {points:guide,centres,offset,observed:shifts.length,mode:age>.3?'REMEMBERED CORRIDOR':shifts.length>=3?'CAMERA CORRIDOR':'CAMERA ROAD + ROUTE BRANCH',stamp:sim.t};
 }
 function footprintEvidence(sim,pose){
  const g=C.geometry(sim.c);let road=0,paint=0,unknown=0,total=0;
  // Sample both long sides and interior; 2 cm grid spacing. Exclude crossable paint.
  for(let x=-g.rear+.012;x<=g.front-.012;x+=.035)for(let y of [-g.w/2+.012,0,g.w/2-.012]){
   const w=C.toWorld(pose,{x,y}),e=evidence(sim,w,3);total++;
   if(e?.kind===1)road++;else if(e?.kind===2){const divider=sim.course.markings?.some(r=>r.kind==='crossable'&&C.inRect(w.x,w.y,r,.025));if(!divider)paint++;else road++;}else unknown++;
  }
  return {road,paint,unknown,total};
 }
 function branchCheck(sim){
  const p=sim.estimate.pose,r=sim.course.round;
  const near=Math.hypot(p.x-(r.x-.6),p.y-r.y)<.35||Math.hypot(p.x-r.x,p.y-(r.y+.6))<.35||C.inRect(p.x,p.y,sim.course.laneChange,.1);
  if(!near)return {hold:false,label:'Following current corridor'};
  let seen=0,total=0;for(const q of sim.guidance.points){let d=Math.hypot(q.x-p.x,q.y-p.y);if(d<.16||d>.40)continue;total++;if(evidence(sim,q,2)?.kind===1)seen++}
  const agreement=Math.hypot(sim.estimate.globalOffset.x,sim.estimate.globalOffset.y)<Math.max(.25,3*sim.estimate.globalSigma);
  if(total>5&&seen/total<.35)return {hold:true,label:'Branch unconfirmed',reason:'Intended branch has insufficient local road evidence'};
  if(!agreement&&sim.estimate.visualRank<.25)return {hold:true,label:'Route ambiguous',reason:'Course location conflicts with local branch; waiting for landmark agreement'};
  return {hold:false,label:agreement?'Route + visible opening agree':'Visual landmark confirms route; UWB disagrees'};
 }
 G.CarbotGuidance={evidence,corridor,footprintEvidence,branchCheck};
})(globalThis);
