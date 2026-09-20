/* Reeds–Shepp analytic families adapted from PythonRobotics, MIT.
 * Atsushi Sakai and Videh Patel. See THIRD_PARTY_NOTICES.txt.
 * Twelve base families × time reversal × reflection. Every accepted endpoint
 * is independently checked by bicycle integration before collision validation.
 */
(function(G){'use strict';const C=G.CarbotCore,pi=Math.PI,h=pi/2,m=C.wrap,polar=(x,y)=>[Math.hypot(x,y),Math.atan2(y,x)],sin=Math.sin,cos=Math.cos,sqrt=Math.sqrt,acos=x=>Math.acos(C.clamp(x,-1,1)),asin=x=>Math.asin(C.clamp(x,-1,1));
const F=[
 (x,y,p)=>{let[u,t]=polar(x-sin(p),y-1+cos(p)),v=m(p-t);if(t>=0&&t<=pi&&v>=0&&v<=pi)return[[t,u,v],'LSL']},
 (x,y,p)=>{let[r,b]=polar(x+sin(p),y-1-cos(p));if(r*r>=4){let u=sqrt(r*r-4),t=m(b+Math.atan2(2,u)),v=m(t-p);if(t>=0&&v>=0)return[[t,u,v],'LSR']}},
 (x,y,p)=>{let[r,b]=polar(x-sin(p),y-1+cos(p));if(r<=4){let a=acos(r/4),t=m(a+b+h),u=m(pi-2*a),v=m(p-t-u);return[[t,-u,v],'LRL']}},
 (x,y,p)=>{let[r,b]=polar(x-sin(p),y-1+cos(p));if(r<=4){let a=acos(r/4),t=m(a+b+h),u=m(pi-2*a),v=m(-p+t+u);return[[t,-u,-v],'LRL']}},
 (x,y,p)=>{let[r,b]=polar(x-sin(p),y-1+cos(p));if(r>1e-8&&r<=4){let u=acos(1-r*r/8),a=asin(2*sin(u)/r),t=m(-a+b+h),v=m(t-u-p);return[[t,u,-v],'LRL']}},
 (x,y,p)=>{let[r,b]=polar(x+sin(p),y-1-cos(p));if(r<=2){let a=acos((r+2)/4),t=m(b+a+h),u=m(a),v=m(p-t+2*u);if(t>=0&&u>=0&&v>=0)return[[t,u,-u,-v],'LRLR']}},
 (x,y,p)=>{let[r,b]=polar(x+sin(p),y-1-cos(p)),u2=(20-r*r)/16;if(u2>=0&&u2<=1&&r>1e-8){let u=acos(u2),a=asin(2*sin(u)/r),t=m(b+a+h),v=m(t-p);if(t>=0&&v>=0)return[[t,-u,-u,v],'LRLR']}},
 (x,y,p)=>{let[r,b]=polar(x-sin(p),y-1+cos(p));if(r>=2){let q=sqrt(r*r-4),u=q-2,a=Math.atan2(2,q),t=m(b+a+h),v=m(t-p+h);if(t>=0&&v>=0)return[[t,-h,-u,-v],'LRSL']}},
 (x,y,p)=>{let[r,b]=polar(x+sin(p),y-1-cos(p));if(r>=2){let t=m(b+h),u=r-2,v=m(p-t-h);if(t>=0&&v>=0)return[[t,-h,-u,-v],'LRSR']}},
 (x,y,p)=>{let[r,b]=polar(x-sin(p),y-1+cos(p));if(r>=2){let q=sqrt(r*r-4),u=q-2,a=Math.atan2(q,2),t=m(b-a+h),v=m(t-p-h);if(t>=0&&v>=0)return[[t,u,h,-v],'LSRL']}},
 (x,y,p)=>{let[r,b]=polar(x+sin(p),y-1-cos(p));if(r>=2){let t=m(b),u=r-2,v=m(p-t-h);if(t>=0&&v>=0)return[[t,u,h,-v],'LSLR']}},
 (x,y,p)=>{let[r,b]=polar(x+sin(p),y-1-cos(p));if(r>=4){let q=sqrt(r*r-4),u=q-4,a=Math.atan2(2,q),t=m(b+a+h),v=m(t-p);if(t>=0&&v>=0)return[[t,-h,-u,-h,v],'LRSLR']}}
];
function candidates(start,goal,r,step=.006){let local=C.toLocal(start,goal),x=local.x/r,y=local.y/r,phi=C.wrap(goal.a-start.a),out=[];for(let f of F)for(let variant=0;variant<4;variant++){let flip=variant===1||variant===3,reflect=variant>=2,q=f(flip?-x:x,reflect?-y:y,(flip!==reflect)?-phi:phi);if(!q)continue;let lengths=q[0].map(v=>v*r*(flip?-1:1)),types=q[1].split('').map(v=>reflect?(v==='L'?'R':v==='R'?'L':'S'):v),p={...start},path=[],cost=0;for(let j=0;j<lengths.length;j++){let d=lengths[j];if(Math.abs(d)<1e-8)continue;let dir=Math.sign(d),k=types[j]==='L'?1/r:types[j]==='R'?-1/r:0,n=Math.ceil(Math.abs(d)/step);if(!path.length)path.push({...p,dir,k});else path.push({...p,dir,k});let origin={...p};for(let i=1;i<=n;i++){p=C.bicycle(origin,d*i/n,k);path.push({...p,dir,k})}cost+=Math.abs(d)}if(!path.length||Math.hypot(p.x-goal.x,p.y-goal.y)>1e-5||Math.abs(C.wrap(p.a-goal.a))>1e-5)continue;out.push({path,cost,types:types.join(''),lengths,variant})}out.sort((a,b)=>a.cost-b.cost);return out}
function plan(start,goal,course,c){let all=candidates(start,goal,C.geometry(c).r);for(const q of all){q.valid=q.path.every(p=>course.bodyClear(p,c,.003));q.reject=q.valid?'':'Footprint leaves drivable area'}let winner=all.find(q=>q.valid);return winner?{...winner,expanded:48,reason:'Live Reeds–Shepp connection',candidates:all.length,evaluated:all}:{path:[],expanded:48,candidates:all.length,evaluated:all,reason:'No collision-free analytic connection'}}
G.CarbotRS={candidates,plan};
})(globalThis);
