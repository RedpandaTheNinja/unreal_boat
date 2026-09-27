'use strict';
const $=id=>document.getElementById(id), fmt=(v,n=2)=>Number.isFinite(v)?v.toFixed(n):'—';
const DEG=180/Math.PI, colors=['#40e1cf','#ffad66'];
let circuit=null, latest=null, lastFrame=-1, lastTime=-1, trail=[];
for(const [target,prefix] of [['accelAxes','a'],['gyroAxes','g']])
  $(target).innerHTML=['X','Y','Z'].map(a=>`<div class="axisrow"><span>${a}</span><div class="bar"><i id="${prefix}${a}bar"></i></div><b id="${prefix}${a}">—</b></div>`).join('');
$('wheels').innerHTML=['Front left','Front right','Rear left','Rear right'].map((n,i)=>`<div class="wheel"><span>${n}</span><strong id="w${i}">—</strong><span id="c${i}">Awaiting contact</span></div>`).join('');
function canvas(id){const el=$(id), r=el.getBoundingClientRect(), dpr=window.devicePixelRatio||1;
  if(el.width!==Math.round(r.width*dpr)||el.height!==Math.round(r.height*dpr)){el.width=Math.round(r.width*dpr);el.height=Math.round(r.height*dpr);}
  const ctx=el.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,r.width,r.height);return[ctx,r.width,r.height];}
function drawMap(p){const [ctx,w,h]=canvas('map');if(!circuit)return;
  const pts=circuit.centerline_m, xs=pts.map(p=>p[0]),ys=pts.map(p=>p[1]);
  const loX=Math.min(...xs),hiX=Math.max(...xs),loY=Math.min(...ys),hiY=Math.max(...ys);
  const scale=Math.min((w-45)/(hiX-loX),(h-45)/(hiY-loY));
  const xy=(x,y)=>[w/2+(x-(loX+hiX)/2)*scale,h/2-(y-(loY+hiY)/2)*scale];
  const path=(arr,stroke,width,close=false)=>{ctx.beginPath();arr.forEach((p,i)=>{const v=xy(p[0],p[1]);i?ctx.lineTo(...v):ctx.moveTo(...v)});if(close)ctx.closePath();ctx.strokeStyle=stroke;ctx.lineWidth=width;ctx.lineJoin='round';ctx.stroke()};
  path(pts,'#d3dfeb',Math.max(4,circuit.width_m*scale+2),true);path(pts,'#223141',Math.max(2,circuit.width_m*scale),true);
  if(trail.length>1)path(trail,'#40e1cf88',2);
  const start=xy(...pts[0]);ctx.fillStyle='#edf5fb';ctx.font='10px system-ui';ctx.fillText('START',start[0]+7,start[1]-8);
  if(p){const pos=xy(p.track.x_m,p.track.y_m);ctx.save();ctx.translate(...pos);ctx.rotate(-p.yaw_rad);ctx.beginPath();ctx.moveTo(9,0);ctx.lineTo(-6,-5);ctx.lineTo(-3,0);ctx.lineTo(-6,5);ctx.closePath();ctx.fillStyle=latest.status==='live'?colors[0]:'#93a9bb';ctx.fill();ctx.restore();}
}
function selectedPoints(p){const mode=$('lidarSource').value;return(p?.lidar_points_body_m||[]).filter(v=>mode==='both'||v[2]===Number(mode));}
function drawLidar(p){const [ctx,w,h]=canvas('lidar');const scale=Math.min(w,h)/8, cx=w/2,cy=h/2;
  ctx.font='10px system-ui';ctx.textAlign='center';ctx.fillStyle='#93a9bb';ctx.fillText('FORWARD',cx,12);
  for(const r of [1,2,3.5]){ctx.beginPath();ctx.arc(cx,cy,r*scale,0,Math.PI*2);ctx.strokeStyle='#293b4b';ctx.lineWidth=1;ctx.stroke();ctx.fillText(`${r} m`,cx+5,cy-r*scale+12);}
  ctx.beginPath();ctx.moveTo(cx,20);ctx.lineTo(cx,h-10);ctx.moveTo(10,cy);ctx.lineTo(w-10,cy);ctx.strokeStyle='#20313f';ctx.stroke();
  const threshold=Number($('threshold').value);ctx.fillStyle='#ffad6617';ctx.fillRect(cx-.35*scale,cy-(threshold+.3)*scale,.7*scale,threshold*scale);
  for(const [x,y,source]of selectedPoints(p)){ctx.fillStyle=colors[source];ctx.beginPath();ctx.arc(cx-y*scale,cy-x*scale,2,0,Math.PI*2);ctx.fill();}
  ctx.fillStyle='#cce8ef';ctx.fillRect(cx-.125*scale,cy-.3*scale,.25*scale,.6*scale);
  ctx.fillStyle='#40e1cf';ctx.fillRect(cx-.035*scale,cy-.3*scale,.07*scale,.13*scale);
}
function axes(prefix,vec,limit,factor=1){for(const axis of ['X','Y','Z']){const value=vec?.[axis.toLowerCase()]*factor;$(prefix+axis).textContent=fmt(value);const v=Number.isFinite(value)?Math.max(-1,Math.min(1,value/limit)):0;$(prefix+axis+'bar').style.left=`${50+Math.min(0,v*50)}%`;$(prefix+axis+'bar').style.width=`${Math.abs(v*50)}%`;}}
function render(state){latest=state;const p=state.telemetry, live=state.status==='live';
  $('status').className='status '+state.status;$('status').textContent=`● ${state.status.toUpperCase()}${state.age_s!==null?' · '+fmt(state.age_s,1)+'s age':''}`;
  $('kpis').classList.toggle('frozen',!live);$('error').classList.toggle('hidden',live);
  $('error').textContent=state.status==='paused'?'Unreal simulation time is not advancing. Readings below are held.':'Telemetry disconnected. Start Play in Unreal; readings below may be stale.';
  if(p){
    if(p.time_s<lastTime){trail=[];lastFrame=-1;}if(p.time_s!==lastTime){trail.push([p.track.x_m,p.track.y_m]);if(trail.length>600)trail.shift();lastTime=p.time_s;}
    $('speed').textContent=fmt(Math.abs(p.forward_speed_mps));$('speedKmh').textContent=`${fmt(Math.abs(p.forward_speed_mps)*3.6,1)} km/h · ${p.forward_speed_mps<-.01?'REVERSE':'FORWARD / STOP'}`;
    $('progress').textContent=fmt(p.track.progress_pct,1);$('distance').textContent=`${fmt(p.track.progress_m,1)} / ${fmt(p.track.lap_length_m,0)} m`;
    if(Math.min(p.track.progress_m,p.track.lap_length_m-p.track.progress_m)<.25){$('progress').textContent='0.0';$('distance').textContent=`At start/finish · ${fmt(p.track.lap_length_m,0)} m circuit`;}
    $('position').textContent=`${fmt(p.track.x_m,1)} / ${fmt(p.track.y_m,1)} m`;
    $('margin').textContent=fmt(p.track.estimated_edge_margin_m)+' m';$('margin').className=p.track.estimated_edge_margin_m<0?'danger':'';
    const imu=p.imu, imuFresh=imu?.valid&&p.time_s-imu.stamp_s<.5;
    $('yawRate').textContent=imuFresh?fmt(imu.angular_velocity_radps.z*DEG,1):'—';
    axes('a',imuFresh?imu.linear_acceleration_mps2:null,15);axes('g',imuFresh?imu.angular_velocity_radps:null,90,DEG);
    $('imuStamp').textContent=imuFresh?`${fmt(imu.stamp_s,2)} s · ideal sensor`:'STALE / INVALID';
    for(let i=0;i<4;i++){const wheel=p.wheels?.[i];$('w'+i).textContent=wheel?fmt(wheel.steering_rad*DEG,1)+'°':'—';$('w'+i).className=wheel&&Math.abs(wheel.steering_rad*DEG)>30.05?'danger':'';$('c'+i).innerHTML=`<i class="dot ${wheel?.in_contact?'':'off'}"></i>${wheel?.in_contact?'Contact':'No contact'}`;}
    const mode=$('lidarSource').value;
    const scans=(mode==='both'?[p.front_scan,p.rear_scan]:[mode==='0'?p.front_scan:p.rear_scan]);
    const valid=scans.every(s=>s?.valid&&p.time_s-s.stamp_s<.5);
    const hits=selectedPoints(p), candidates=hits.filter(v=>v[0]>=.30&&Math.abs(v[1])<=.35).map(v=>v[0]-.30);
    const nearest=candidates.length?Math.min(...candidates):null;
    $('obstacle').textContent=valid?(nearest===null?'—':fmt(nearest)):'—';
    $('obstacle').className=live&&nearest!==null&&nearest<Number($('threshold').value)?'danger':'';
    $('obstacleNote').textContent=!live?'Telemetry stale — clearance unknown':!valid?'LiDAR stale — clearance unknown':nearest===null?'No return in forward corridor':nearest<Number($('threshold').value)?'PROXIMITY ALERT · forward corridor':'Nearest forward-corridor return';
    $('lidarSummary').textContent=`${hits.length} valid returns · teal front / orange rear · ${valid?'scan current':'scan stale'}`;
    $('clock').textContent=`Simulation ${fmt(p.time_s,2)} s · ${p.physics_valid?'Physics active':'Physics unavailable'}`;
  }
  const cam=state.camera, cameraLive=live&&cam&&state.camera_age_s<1.5&&!state.camera_error;
  $('cameraOverlay').classList.toggle('hidden',!!cameraLive);
  $('cameraOverlay').textContent=state.camera_error?'Camera unavailable. Keep only one camera reader connected.':(!live?'Camera held — start or resume Play':'Waiting for a current camera frame');
  if(cam&&cameraLive&&cam.sequence!==lastFrame){lastFrame=cam.sequence;$('camera').src='/camera.jpg?sequence='+cam.sequence;}
  $('cameraInfo').textContent=cam?`FRAME ${cam.sequence} · SIM ${fmt(cam.stamp_s,2)} s`:'90° field of view';
  $('cameraAge').textContent=state.camera_age_s!==null?fmt(state.camera_age_s,1)+' s old':'—';
  drawMap(p);drawLidar(p);
}
for(const id of ['lidarSource','threshold'])$(id).addEventListener('change',()=>latest&&render(latest));
window.addEventListener('resize',()=>latest&&render(latest));
$('download').onclick=()=>{if(!latest?.telemetry)return;const a=document.createElement('a'),url=URL.createObjectURL(new Blob([JSON.stringify(latest,null,2)],{type:'application/json'}));a.href=url;a.download='f1-telemetry.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000)};
async function poll(){try{const r=await fetch('/api/state',{cache:'no-store'});if(!r.ok)throw Error('HTTP '+r.status);render(await r.json());}catch(e){$('status').className='status offline';$('status').textContent='● BRIDGE OFFLINE';$('error').classList.remove('hidden');$('error').textContent='Dashboard server unavailable. Restart launch_dashboard.ps1.';$('cameraOverlay').classList.remove('hidden');$('cameraOverlay').textContent='Connection lost — last image is stale';$('kpis').classList.add('frozen');}finally{setTimeout(poll,100)}}
fetch('/api/track').then(r=>r.json()).then(t=>{circuit=t;drawMap(null)}).catch(()=>{$('error').textContent='Track map unavailable.'});poll();
