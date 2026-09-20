// Short-horizon bicycle rollouts. The selected lateral target is also used by
// the live pure-pursuit tracker: these are evaluated predictions, not decorations.
(function(G){
 const C=G.CarbotCore,V=G.CarbotVehicle;
 function rollouts(sim,tolerance=0){
  const {c,course}=sim,g=C.geometry(c),start=sim.estimate.pose,reference=sim.guidance?.points?.length?sim.guidance.points:sim.path;
  const offsets=[-.06,-.04,-.02,-.008,0,.008,.02,.04,.06],out=[];
  for(let id=0;id<offsets.length;id++){
   const offset=offsets[id],points=[{...start}],dir=1;
   let p={...start},steer=sim.steeringEstimate||0,index=sim.guidance?0:sim.ctrl.index,cost=0,seen=0,blocked=0,roadBlocked=0,paintBlocked=0,obstacleBlocked=0,minClear=1,initialSteer=0;
   const speed=Math.max(.055,Math.min(.12,Math.abs(sim.plant.speed))),ds=.01,dt=ds/speed;
   for(let n=0;n<55;n++){
    const hit=V.closest(reference,p,index);index=hit.index;
    let j=index;while(j<reference.length-1&&Math.hypot(reference[j].x-p.x,reference[j].y-p.y)<.095)j++;
    const q=reference[j];if(!q)break;
    let target={x:q.x-offset*Math.sin(q.a),y:q.y+offset*Math.cos(q.a)},rel=C.toLocal(p,target);
    const desired=C.clamp(Math.atan(g.wb*2*rel.y/Math.max(.002,rel.x*rel.x+rel.y*rel.y)),-g.maxSteer,g.maxSteer);
    if(n===0)initialSteer=desired;
    const rate=C.clamp((desired-steer)/(c.steeringLagMs/1000),-c.steeringRateDeg*Math.PI/180,c.steeringRateDeg*Math.PI/180);
    // Exponential update avoids numerical overshoot when rollout dt exceeds servo lag.
    steer+=C.clamp((desired-steer)*(1-Math.exp(-dt/(c.steeringLagMs/1000))),-Math.abs(rate*dt),Math.abs(rate*dt));
    p=C.bicycle(p,ds,Math.tan(steer)/g.wb);points.push({...p,dir,k:Math.tan(steer)/g.wb});
    if(n%2===0){let margin=course.bodyMargin(p);minClear=Math.min(minClear,margin);if(!(G.CarbotRecovery?G.CarbotRecovery.roadClear(course,p,c,tolerance):course.bodyClear(p,c,.005))){blocked++;roadBlocked++;}
     let local=C.toLocal(start,p),live=sim.perception.support(local),od=C.toWorld(sim.estimate.odom,local),mem=sim.memory.query(od.x,od.y,sim.t);if(tolerance>0&&G.CarbotRecovery&&!G.CarbotRecovery.obstaclesClear(sim,p)){blocked++;obstacleBlocked++;}let f=G.CarbotGuidance?.footprintEvidence(sim,p);if(f){cost+=f.paint*.07+f.unknown*.003;if(n<20&&f.paint>5&&tolerance===0){blocked++;paintBlocked++;}}
     if(live===1)seen++;cost+=hit.error**2*900+(live===1?0:mem?.kind===1?.006:.014)+.00012/Math.max(.003,margin)+Math.max(0,.03-margin)*140;
    }
    if(index>reference.length-5&&Math.hypot(p.x-reference.at(-1).x,p.y-reference.at(-1).y)<.035)break;
   }
   cost+=Math.abs(offset)*25+Math.max(0,-minClear)*1000;
   out.push({id,offset,points,cost,score:-cost,seen,blocked,valid:blocked===0,relaxed:tolerance>0,roadBlocked,paintBlocked,obstacleBlocked,minClear,commandSteer:initialSteer,reject:blocked?[roadBlocked?'Swept body exceeds road allowance':'',paintBlocked?'Camera paint overlaps footprint':'',obstacleBlocked?'LiDAR obstacle in footprint':''].filter(Boolean).join('; '):''});
  }
  out.sort((a,b)=>Number(b.valid)-Number(a.valid)||a.cost-b.cost);return out;
 }
 function candidates(sim){const strict=rollouts(sim,0);if(strict.some(q=>q.valid)||!(sim.c.lineToleranceCm>0))return strict;
 const relaxed=rollouts(sim,sim.c.lineToleranceCm/100);for(let i=0;i<relaxed.length;i++)relaxed[i].id+=9;
 for(const q of strict)q.reject=q.reject||'Strict pass';return [...strict,...relaxed].sort((a,b)=>Number(b.valid)-Number(a.valid)||a.cost-b.cost);
 }
 G.CarbotLocal={candidates};
})(globalThis);
