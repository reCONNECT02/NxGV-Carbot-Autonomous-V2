(function(G){'use strict';
const PI=Math.PI,TAU=2*PI,clamp=(x,a,b)=>Math.max(a,Math.min(b,x)),wrap=a=>Math.atan2(Math.sin(a),Math.cos(a));
const DEFAULTS={carLengthCm:30,carWidthCm:19.2,wheelbaseCm:21.6,rearOverhangCm:4.2,minimumTurningRadiusCm:40,wheelRadiusMm:33,wheelWidthMm:25,massKg:2,courseLengthCm:700,courseWidthCm:500,laneWidthCm:30,steeringLagMs:120,speedLagMs:180,steeringRateDeg:180,maxSpeedCms:18,parkingSpeedCms:6,uwbNoiseCm:1.2,uwbNoiseEnabled:true,encoderScaleErrorPercent:.4,imuNoiseDeg:.35,seed:2026,noiseEnabled:true,frontCameraHeightCm:18,sideCameraHeightCm:15,frontCameraPitchDeg:25,sideCameraPitchDeg:50,cameraHorizontalFovDeg:100,lidarHeightCm:22,vehicleHeightCm:24,lineToleranceCm:5,recoveryEnabled:true,recoveryReverseCm:20,expiryMs:200};
const PHOTO_DIMENSIONS={laneChangeLengthCm:80,laneChangeWidthCm:80,laneChangeDividerCm:3,parallelLengthCm:69,parallelWidthCm:47,parkingBorderCm:3,perpendicularDepthCm:47,perpendicularWidthCm:45,perpendicularLeftSpaceCm:17,perpendicularRightSpaceCm:24};
const PROVENANCE={measured:['carLengthCm','carWidthCm','wheelbaseCm','wheelRadiusMm','wheelWidthMm','laneWidthCm'],assumed:['minimumTurningRadiusCm','massKg','rearOverhangCm','steeringLagMs','speedLagMs','steeringRateDeg','uwbNoiseCm','encoderScaleErrorPercent','imuNoiseDeg','cameraHorizontalFovDeg','cameraVerticalFovDeg'],course:'Drawing reconstruction: 7 m main outer width; right spur extends to 7.5 m. Drawing width and height are not a uniform screenshot scaling.',height:'Lowered LiDAR mount is a design target. Actual mast remains a confirmed overheight rule violation.'};
function geometry(c){return {l:c.carLengthCm/100,w:c.carWidthCm/100,wb:c.wheelbaseCm/100,r:c.minimumTurningRadiusCm/100,rear:c.rearOverhangCm/100,front:(c.carLengthCm-c.rearOverhangCm)/100,track:(c.carWidthCm-c.wheelWidthMm/10)/100,maxSteer:Math.atan(c.wheelbaseCm/c.minimumTurningRadiusCm)}}
function bicycle(p,d,k){if(Math.abs(k)<1e-9)return {x:p.x+d*Math.cos(p.a),y:p.y+d*Math.sin(p.a),a:p.a};let a=p.a+d*k;return {x:p.x+(Math.sin(a)-Math.sin(p.a))/k,y:p.y+(-Math.cos(a)+Math.cos(p.a))/k,a:wrap(a)}}
function toWorld(p,q){let c=Math.cos(p.a),s=Math.sin(p.a);return{x:p.x+q.x*c-q.y*s,y:p.y+q.x*s+q.y*c}}
function toLocal(p,q){let x=q.x-p.x,y=q.y-p.y,c=Math.cos(p.a),s=Math.sin(p.a);return{x:x*c+y*s,y:-x*s+y*c}}
function footprint(p,c,pad=0){let g=geometry(c);return [[-g.rear-pad,-g.w/2-pad],[g.front+pad,-g.w/2-pad],[g.front+pad,g.w/2+pad],[-g.rear-pad,g.w/2+pad]].map(([x,y])=>toWorld(p,{x,y}))}
function segDist(x,y,a,b){let dx=b.x-a.x,dy=b.y-a.y,t=clamp(((x-a.x)*dx+(y-a.y)*dy)/(dx*dx+dy*dy),0,1);return Math.hypot(x-a.x-t*dx,y-a.y-t*dy)}
function inRect(x,y,r,pad=0){return x>=r.x0-pad&&x<=r.x1+pad&&y>=r.y0-pad&&y<=r.y1+pad}
function inPoly(x,y,p){let on=false;for(let i=0,j=p.length-1;i<p.length;j=i++){let a=p[i],b=p[j];if(((a.y>y)!==(b.y>y))&&x<(b.x-a.x)*(y-a.y)/(b.y-a.y)+a.x)on=!on}return on}
function sampleLine(a,b,step=.025){let n=Math.ceil(Math.hypot(b.x-a.x,b.y-a.y)/step),p=[];for(let i=0;i<=n;i++)p.push({x:a.x+(b.x-a.x)*i/n,y:a.y+(b.y-a.y)*i/n});return p}
function arc(cx,cy,r,a,b,step=.025){let n=Math.ceil(Math.abs(b-a)*r/step),p=[];for(let i=0;i<=n;i++)p.push({x:cx+r*Math.cos(a+(b-a)*i/n),y:cy+r*Math.sin(a+(b-a)*i/n)});return p}
class Course{
 constructor(c=DEFAULTS){this.c=c;this.sx=c.courseLengthCm/700;this.sy=c.courseWidthCm/500;this.w=7.65*this.sx;this.h=5.15*this.sy;this.lane=c.laneWidthCm/100;this.paths=[];const add=p=>this.paths.push(p);
 add(sampleLine({x:1,y:4.75},{x:6,y:4.75}));add(arc(1,4,.75,PI/2,PI));add(sampleLine({x:.25,y:4},{x:.25,y:1.5}));add(arc(1,1.5,.75,PI,1.5*PI));add(sampleLine({x:1,y:.75},{x:1.9,y:.75}));
 const D=PHOTO_DIMENSIONS,lcX=3.55,lcY=.65,lcL=D.laneChangeLengthCm/100,lcW=D.laneChangeWidthCm/100,tape=D.parkingBorderCm/100;this.upperLaneY=lcY+lcW-this.lane/2; // 30 cm approach lane, flush with the top of the 80 cm transition.
 add(arc(6,4,.75,0,PI/2));add(sampleLine({x:6.75,y:4},{x:6.75,y:2.05}));add(arc(6,2.05,.75,-PI/2,0));add(sampleLine({x:4.35,y:this.upperLaneY},{x:7.5,y:this.upperLaneY}));
 add(sampleLine({x:2.95,y:.8},{x:3.55,y:.8}));
 add(arc(2.4,.8,.60,0,TAU));add(sampleLine({x:2.4,y:1.35},{x:2.4,y:3.15}));add(arc(2.8,3.15,.4,PI/2,PI));add(sampleLine({x:2.8,y:3.55},{x:3.91,y:3.55}));
 this.paths=this.paths.map(p=>p.map(v=>({x:v.x*this.sx,y:v.y*this.sy})));this.round={x:2.4*this.sx,y:.8*this.sy,r:.6};
 const rect=(x0,x1,y0,y1)=>({x0:x0*this.sx,x1:x1*this.sx,y0:y0*this.sy,y1:y1*this.sy});
 this.laneChange=rect(lcX,lcX+lcL,lcY,lcY+lcW);
 const px=2.25-D.parallelWidthCm/100,py=2.20,pyTop=py+D.parallelLengthCm/100;this.parallel=rect(px,2.25,py,pyTop);this.parallelOuter=rect(px-tape,2.25+tape,py-tape,pyTop+tape);this.parallelOpening=rect(px,2.40,py,pyTop);
 const vx=3.34,vRight=vx+D.perpendicularWidthCm/100,vy=3.40-D.perpendicularDepthCm/100,vLeftZone=vx-tape-D.perpendicularLeftSpaceCm/100,vRightZone=vRight+tape+D.perpendicularRightSpaceCm/100;this.perpendicular=rect(vx,vRight,vy,3.40);this.perpendicularZone=rect(vLeftZone,vRightZone,vy,3.40);
 this.rects=[this.laneChange,this.parallelOpening,this.perpendicularZone,rect(2.8,4.06,3.40,3.70)];
 this.markings=[];const paint=(x0,x1,y0,y1,kind='border')=>this.markings.push({...rect(x0,x1,y0,y1),kind});
 // Dash length/gap are rendering assumptions. Clear dimensions and 3 cm divider follow the photos.
 for(let x=lcX;x<lcX+lcL;x+=.20)paint(x,Math.min(x+.14,lcX+lcL),lcY+lcW/2-D.laneChangeDividerCm/200,lcY+lcW/2+D.laneChangeDividerCm/200,'crossable');
 paint(px-tape,px,py-tape,pyTop+tape);paint(px-tape,2.25+tape,py-tape,py);paint(px-tape,2.25+tape,pyTop,pyTop+tape);
 for(let y=py;y<pyTop;y+=.21)paint(2.25,2.25+tape,y,Math.min(y+.12,pyTop),'crossable');
 for(let x of [vx-tape,vRight])for(let y=vy;y<3.40;y+=.18)paint(x,x+tape,y,Math.min(y+.11,3.40),'crossable');
 paint(vLeftZone-tape,vLeftZone,vy-tape,3.40);paint(vRightZone,vRightZone+tape,vy-tape,3.70+tape);paint(vLeftZone-tape,vRightZone+tape,vy-tape,vy);
 this.start={x:7.18*this.sx,y:this.upperLaneY*this.sy,a:PI};this.lightGoal={x:6.75*this.sx,y:3.80*this.sy,a:-PI/2};this.light={x:6.98*this.sx,y:3.0*this.sy,z:.28};this.gate={x:.99*this.sx,y:.75*this.sy};
 // Tapered junction mouths traced from the supplied drawing, not constant-width T crossings.
 const east=[{x:2.90,y:1.18},{x:3.18,y:.95},{x:3.5,y:.95},{x:3.5,y:.65},{x:3.18,y:.65},{x:2.9,y:.42}];
 this.flares=[east,east.map(p=>({x:4.8-p.x,y:p.y})),east.map(p=>({x:2.4-(p.y-.8),y:.8+(p.x-2.4)}))].map(poly=>poly.map(p=>({x:p.x*this.sx,y:p.y*this.sy})));
 this.res=.01;this.nx=Math.ceil(this.w/this.res)+1;this.ny=Math.ceil(this.h/this.res)+1;this.field=new Float32Array(this.nx*this.ny).fill(-10);
 // Distance-to-centreline raster, used only as environment / prior-map geometry.
 for(const p of this.paths)for(const q of p){let ix=Math.round(q.x/this.res),iy=Math.round(q.y/this.res),n=31;for(let dy=-n;dy<=n;dy++)for(let dx=-n;dx<=n;dx++){let x=ix+dx,y=iy+dy;if(x<0||y<0||x>=this.nx||y>=this.ny)continue;let d=this.lane/2-Math.hypot(x*this.res-q.x,y*this.res-q.y),i=y*this.nx+x;if(d>this.field[i])this.field[i]=d;}}
 for(let j=0;j<this.ny;j++)for(let i=0;i<this.nx;i++){let x=i*this.res,y=j*this.res;for(const r of this.rects){let d;if(inRect(x,y,r))d=Math.min(x-r.x0,r.x1-x,y-r.y0,r.y1-y);else d=-Math.hypot(Math.max(r.x0-x,0,x-r.x1),Math.max(r.y0-y,0,y-r.y1));this.field[j*this.nx+i]=Math.max(this.field[j*this.nx+i],d)}for(const p of this.flares){if(inPoly(x,y,p)){let d=Math.min(...p.map((a,k)=>segDist(x,y,a,p[(k+1)%p.length])));this.field[j*this.nx+i]=Math.max(this.field[j*this.nx+i],d)}}}
 }
 clearance(x,y){let xx=x/this.res,yy=y/this.res,i=Math.floor(xx),j=Math.floor(yy);if(i<0||j<0||i+1>=this.nx||j+1>=this.ny)return-10;let u=xx-i,v=yy-j,k=j*this.nx+i;return(1-v)*((1-u)*this.field[k]+u*this.field[k+1])+v*((1-u)*this.field[k+this.nx]+u*this.field[k+this.nx+1])}
 bodyClear(p,c=this.c,pad=0){let g=geometry(c);for(let x=-g.rear-pad;x<g.front+pad+.001;x+=.035)for(let y of [-g.w/2-pad,g.w/2+pad]){let q=toWorld(p,{x,y});if(this.clearance(q.x,q.y)<0)return false}for(let x of [-g.rear-pad,g.front+pad])for(let y=-g.w/2-pad;y<=g.w/2+pad+.001;y+=.03){let q=toWorld(p,{x,y});if(this.clearance(q.x,q.y)<0)return false}return footprint(p,c,pad).every(q=>this.clearance(q.x,q.y)>=0)}
 bodyMargin(p,c=this.c){return Math.min(...footprint(p,c).map(q=>this.clearance(q.x,q.y)))}
 surface(x,y){x/=this.sx;y/=this.sy;let z=0;if(y>4.49&&y<5.01&&x>1.9775&&x<3.9275){let d=x-1.9775;if(d<.725)z=.155*(1-Math.cos(PI*d/.725))/2;else if(d<1.225)z=.155;else z=.155*(1+Math.cos(PI*(d-1.225)/.725))/2}if(y>4.53&&y<4.97&&Math.abs(x-4.6)<.02)z=Math.max(z,.01*Math.sqrt(Math.max(0,1-((x-4.6)/.02)**2)));return z}
 tunnel(x,y){let xx=x/this.sx-1,yy=y/this.sy-1.5;return xx<=.03&&yy<=.02&&Math.hypot(xx,yy)>.485&&Math.hypot(xx,yy)<1.015}
 parkGoal(){let g=geometry(this.c),r=this.parallel;return{x:(r.x0+r.x1)/2,y:(r.y0+r.y1)/2-(g.front-g.rear)/2,a:PI/2}}
 parked(p){return Math.abs(wrap(p.a-PI/2))<.05&&footprint(p,this.c).every(q=>inRect(q.x,q.y,this.parallel,-.008))}
}
class Heap{constructor(){this.a=[]}push(v){let a=this.a,i=a.length;a.push(v);while(i){let p=(i-1)>>1;if(a[p].f<=v.f)break;a[i]=a[p];i=p}a[i]=v}pop(){let a=this.a,out=a[0],v=a.pop();if(a.length){let i=0;while(i*2+1<a.length){let j=i*2+1;if(j+1<a.length&&a[j+1].f<a[j].f)j++;if(a[j].f>=v.f)break;a[i]=a[j];i=j}a[i]=v}return out}get length(){return this.a.length}}
function hybridPlan(start,goal,course,c,{reverse=false,maxNodes=120000,bounds=null,clockwise=false}={}){
 let g=geometry(c),step=reverse?.025:.04,xy=reverse?.014:.012,angles=180,heap=new Heap(),seen=new Map(),best=null,connector=null,exp=0;
 const key=p=>`${Math.round(p.x/xy)},${Math.round(p.y/xy)},${Math.round((wrap(p.a)+PI)/TAU*angles)},${p.dir||1}`;
 const heuristic=p=>Math.hypot(goal.x-p.x,goal.y-p.y)+.12*Math.abs(wrap(goal.a-p.a));
 let root={...start,g:0,f:heuristic(start),parent:null,dir:1,k:0};heap.push(root);seen.set(key(root),0);
 while(heap.length&&exp++<maxNodes){let p=heap.pop();let d=Math.hypot(p.x-goal.x,p.y-goal.y),da=Math.abs(wrap(p.a-goal.a));if(reverse&&G.CarbotRS&&d<.75&&exp%12===0){let r=G.CarbotRS.plan(p,goal,course,c);if(r.path.length){best=p;connector=r.path;break}}if(d<(reverse?.01:.025)&&da<(reverse?.02:.035)){best=p;break}
 for(let dir of reverse?[1,-1]:[1])for(let k of [-1,-.5,0,.5,1].map(v=>v/g.r)){
 let n=bicycle(p,dir*step,k);n.dir=dir;n.k=k;if(bounds&&(n.x<bounds[0]||n.x>bounds[1]||n.y<bounds[2]||n.y>bounds[3]))continue;
 const planningMargin=reverse?.006:.008;
 if(!course.bodyClear(n,c,planningMargin)||!course.bodyClear(bicycle(p,dir*step/2,k),c,planningMargin))continue;
 if(clockwise){let rx=n.x-course.round.x,ry=n.y-course.round.y,rr=Math.hypot(rx,ry);if(rr<.80&&rr>.40){let dot=(Math.cos(n.a)*ry-Math.sin(n.a)*rx)*dir;if(dot<-.05)continue}}
 let margin=course.bodyMargin(n,c),cost=p.g+step*(dir<0?1.07:1)+Math.abs(k)*.0003+(p.dir!==dir?.08:0)+Math.abs(p.k-k)*.001+step*.003/Math.max(.006,margin);
 let id=key(n);if(seen.has(id)&&seen.get(id)<=cost)continue;seen.set(id,cost);n.g=cost;n.f=cost+heuristic(n)*1.25;n.parent=p;heap.push(n);
 }}
 if(!best)return{path:[],expanded:exp,reason:'No feasible path found within bounded search'};let chain=[];while(best){chain.push(best);best=best.parent}chain.reverse();let path=[{...start,dir:chain[1]?.dir||1,k:0}];for(let i=1;i<chain.length;i++){let a=chain[i-1],b=chain[i],n=4;for(let j=1;j<=n;j++)path.push({...bicycle(a,b.dir*step*j/n,b.k),dir:b.dir,k:b.k})}if(connector)path.push(...connector);return{path,expanded:exp,reason:connector?'Hybrid search + Reeds–Shepp connection':'Validated sampled full footprint'};
}
function buildMission(course,c,notify=()=>{}){const sx=course.sx,sy=course.sy,at=(x,y,a)=>({x:x*sx,y:y*sy,a});let checkpoints=[at(2.4,.22,PI),at(1.1,.75,PI),at(.25,2.5,PI/2),at(1.2,4.75,0),at(5.8,4.75,0),course.lightGoal];let start=course.start,routes=[[]],expanded=0;
 const append=(goal,opts={})=>{let r=hybridPlan(start,goal,course,c,opts);expanded+=r.expanded;if(!r.path.length)throw new Error(`Route cannot fit at (${goal.x.toFixed(2)},${goal.y.toFixed(2)}): ${r.reason}. Keep the radius or correct the geometry; the simulator will not force the car through.`);routes.at(-1).push(...r.path.slice(routes.at(-1).length?1:0));start=r.path.at(-1);notify(goal,r)};
 for(let p of checkpoints)append(p,{clockwise:true});routes.push([]);for(let p of [at(6.75,2.2,-PI/2),at(5.9,course.upperLaneY,PI),at(2.4,.22,PI),at(2.4,1.8,PI/2),at(2.4,3.02,PI/2)])append(p,{clockwise:true});return{routes,expanded,handoff:start};}
function parkingPlan(start,goal,course,c){let evaluated=[];
 const audit=(r,prefix=[],suffix=[],stage='direct')=>{for(const q of r.evaluated||[]){let points=[...prefix,...q.path,...suffix],valid=q.valid&&prefix.every(p=>course.bodyClear(p,c,.003))&&suffix.every(p=>course.bodyClear(p,c,.003));evaluated.push({id:evaluated.length,points,cost:q.cost+prefix.slice(1).reduce((a,p,i)=>a+Math.hypot(p.x-prefix[i].x,p.y-prefix[i].y),0)+suffix.slice(1).reduce((a,p,i)=>a+Math.hypot(p.x-suffix[i].x,p.y-suffix[i].y),0),valid,blocked:!valid,reject:valid?'':'Footprint leaves drivable area',stage,types:q.types})}};
 const finish=(r,stage)=>({...r,evaluated,stage});
 if(G.CarbotRS){
 // Prefer a feasible final docking straight; shortest feasible connector within that stage.
 for(let tail of [.15,.12,.08]){let pre=bicycle(goal,tail,0),r=G.CarbotRS.plan(start,pre,course,c);let end=sampleLine(pre,goal,.006).map(p=>({...p,a:goal.a,dir:-1,k:0}));let stage=`${Math.round(tail*100)} cm docking straight`;audit(r,[],end,stage);if(r.path.length&&end.every(p=>course.bodyClear(p,c,.003)))return finish({...r,path:[...r.path,...end],cost:r.cost+tail},stage);}
 let r=G.CarbotRS.plan(start,goal,course,c);audit(r);if(r.path.length)return finish(r,'direct');
 let best=null;for(let d of [-.10,-.05,.05,.10,.15,.20]){let mid=bicycle(start,d,0),approach=sampleLine(start,mid,.006).map(p=>({...p,a:start.a,dir:Math.sign(d),k:0}));if(!approach.every(p=>course.bodyClear(p,c,.003)))continue;let q=G.CarbotRS.plan(mid,goal,course,c);audit(q,approach,[],'handoff extension');if(q.path.length&&(!best||q.cost+Math.abs(d)<best.cost))best={...q,cost:q.cost+Math.abs(d),path:[...approach,...q.path]}}if(best)return finish(best,'handoff extension');
 }let r=hybridPlan(start,goal,course,c,{reverse:true,maxNodes:250000,bounds:[course.parallel.x0-.02,2.65*course.sx,1.7*course.sy,3.7*course.sy]});return finish(r,'hybrid search fallback');}
function rng(seed){let a=seed>>>0;return()=>{a+=0x6D2B79F5;let t=Math.imul(a^a>>>15,1|a);t^=t+Math.imul(t^t>>>7,61|t);return((t^t>>>14)>>>0)/4294967296}}
function normal(random){return Math.sqrt(-2*Math.log(Math.max(1e-9,random())))*Math.cos(TAU*random())}
G.CarbotCore={DEFAULTS,PHOTO_DIMENSIONS,PROVENANCE,Course,geometry,bicycle,toWorld,toLocal,footprint,segDist,inRect,arc,sampleLine,hybridPlan,parkingPlan,buildMission,clamp,wrap,rng,normal,Heap};
})(globalThis);
