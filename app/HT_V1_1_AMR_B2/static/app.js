const $=id=>document.getElementById(id);
const canvas=$('mapCanvas'),ctx=canvas.getContext('2d');
let latest=null,mapMeta=null,mapRaster=document.createElement('canvas'),mapRevision=null,scale=1,trail=[];
let ws=null,lastCoreKey='',lastEvents=new Set(),cameraTimer=null;

function clock(){const d=new Date();$('dateText').textContent=d.toLocaleDateString('vi-VN');$('timeText').textContent=d.toLocaleTimeString('vi-VN',{hour12:false});}
setInterval(clock,1000);clock();
function toast(t){const e=$('toast');e.textContent=t;e.classList.add('show');clearTimeout(window.__t);window.__t=setTimeout(()=>e.classList.remove('show'),2800)}
function log(t,cls=''){const d=document.createElement('div');d.className=`logLine ${cls}`;d.textContent=`[${new Date().toLocaleTimeString('vi-VN',{hour12:false})}] ${t}`;$('logBox').appendChild(d);while($('logBox').children.length>120)$('logBox').firstChild.remove();$('logBox').scrollTop=$('logBox').scrollHeight}
function clearTrail(){trail=[];draw();toast('Đã xóa vệt đã di chuyển trên bản đồ');log('Đã xóa vệt đã di chuyển trên bản đồ.','ok')}
function pill(id,text,type='gray'){const e=$(id);if(!e)return;e.textContent=text;e.className=`pill ${type}`}
function badge(id,text,type='gray'){const e=$(id);if(!e)return;e.textContent=text;e.className=`badge ${type}`}
function tSensor(v){return v==='CLEAR'?['KHÔNG CÓ VẬT','green']:v==='OBJECT'?['PHÁT HIỆN VẬT','red']:['CHƯA RÕ','amber']}
function tHw(v){return ['ONLINE','PRESENT'].includes(v)?['Hoạt động','green']:['Chưa sẵn sàng','gray']}
const stageVi={IDLE:'Đang chờ',STARTING:'Đang chuẩn bị',PREFLIGHT:'Tự kiểm tra',START_CHECK:'Kiểm tra vị trí',TO_B2:'Đang đến B2',ALIGN_B2:'Căn hướng tại B2',QR_SCAN:'Đang quét QR',QR_VALID:'QR hợp lệ',OPEN:'Đang mở kẹp',OPEN_DONE:'Đã mở kẹp',PICK_FORWARD:'Đang tiến vào lấy hàng',PICK:'Đang kẹp và nâng hàng',PICK_DONE:'Đã lấy hàng',PICK_EXIT:'Đang lùi khỏi vùng lấy hàng',TO_TH:'Đang đến TH',ALIGN_TH:'Căn hướng tại TH',DROP_APPROACH:'Đang tiến vào vị trí thả',DROP:'Đang hạ và thả hàng',DROP_DONE:'Đã thả hàng',DROP_RETREAT:'Đang lùi khỏi vị trí thả',CLOSE:'Đang đóng kẹp',CLOSE_DONE:'Đã đóng kẹp',RETURN_A_H:'Đang trở về A_H',ALIGN_A_H:'Căn hướng tại A_H',PASS:'Hoàn thành',COMPLETE:'Hoàn thành',FAIL:'Thất bại',STOPPING:'Đang dừng'};
const coreVi={DA_TAT:['ĐANG TẮT','gray'],DANG_KHOI_DONG:['ĐANG KHỞI ĐỘNG','amber'],SAN_SANG:['SẴN SÀNG','green'],DANG_THUC_HIEN:['ĐANG THỰC HIỆN','blue'],LOI:['CÓ LỖI','red'],CAN_KHOI_PHUC:['CẦN KHÔI PHỤC','amber'],DUNG_KHAN:['ĐÃ DỪNG KHẨN','red'],DA_DUNG:['ĐÃ DỪNG','gray']};

function decodeMap(m){mapMeta=m;mapRevision=m.revision;const arr=[];for(const [v,n] of (m.rle||[]))for(let i=0;i<n;i++)arr.push(v);mapRaster.width=m.width;mapRaster.height=m.height;const c=mapRaster.getContext('2d'),img=c.createImageData(m.width,m.height);for(let y=0;y<m.height;y++)for(let x=0;x<m.width;x++){const v=arr[y*m.width+x]??-1;let g=v<0?244:v>=65?35:255-Math.round(v*2.2);const p=((m.height-1-y)*m.width+x)*4;img.data[p]=g;img.data[p+1]=g;img.data[p+2]=g;img.data[p+3]=255}c.putImageData(img,0,0)}
function bounds(){if(mapMeta)return{x0:+mapMeta.origin_x,y0:+mapMeta.origin_y,x1:+mapMeta.origin_x+mapMeta.width*mapMeta.resolution,y1:+mapMeta.origin_y+mapMeta.height*mapMeta.resolution};return{x0:-1.5,y0:-4.79,x1:8.5,y1:4.5}}
function rect(w,h){const b=bounds(),m=20,ww=b.x1-b.x0,hh=b.y1-b.y0,f=Math.min((w-2*m)/ww,(h-2*m)/hh),pw=ww*f*scale,ph=hh*f*scale;return{left:(w-pw)/2,top:(h-ph)/2,width:pw,height:ph,b}}
function xy(x,y,w,h){const r=rect(w,h),b=r.b;return[r.left+(x-b.x0)/(b.x1-b.x0)*r.width,r.top+(1-(y-b.y0)/(b.y1-b.y0))*r.height]}
function path(pts,w,h,color,width,dash=[]){if(!pts||pts.length<2)return;ctx.save();ctx.strokeStyle=color;ctx.lineWidth=width;ctx.lineCap='round';ctx.lineJoin='round';ctx.setLineDash(dash);ctx.beginPath();pts.forEach((p,i)=>{const[a,b]=xy(+p.x,+p.y,w,h);i?ctx.lineTo(a,b):ctx.moveTo(a,b)});ctx.stroke();ctx.restore()}
function points(pts,w,h,color,size,alpha){ctx.save();ctx.fillStyle=color;ctx.globalAlpha=alpha;for(const p of pts||[]){const[a,b]=xy(+p.x,+p.y,w,h);ctx.beginPath();ctx.arc(a,b,size,0,Math.PI*2);ctx.fill()}ctx.restore()}
function draw(){const dpr=window.devicePixelRatio||1,w=canvas.clientWidth,h=canvas.clientHeight;if(!w||!h)return;if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);ctx.setTransform(dpr,0,0,dpr,0,0)}ctx.clearRect(0,0,w,h);ctx.fillStyle='#edf4f8';ctx.fillRect(0,0,w,h);for(let x=0;x<w;x+=32){ctx.strokeStyle='#dce8f0';ctx.lineWidth=.6;ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,h);ctx.stroke()}for(let y=0;y<h;y+=32){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke()}if(mapMeta){const r=rect(w,h);ctx.save();ctx.imageSmoothingEnabled=false;ctx.drawImage(mapRaster,r.left,r.top,r.width,r.height);ctx.strokeStyle='#a8bdcd';ctx.strokeRect(r.left,r.top,r.width,r.height);ctx.restore()}else{ctx.fillStyle='#718a9f';ctx.font='600 15px Segoe UI';ctx.textAlign='center';ctx.fillText('ĐANG CHỜ BẢN ĐỒ...',w/2,h/2);ctx.textAlign='left'}if(!latest)return;points(latest.global_costmap_obstacles,w,h,'#f59e0b',1.3,.28);points(latest.costmap_obstacles,w,h,'#ef4444',1.6,.45);points(latest.scan_points,w,h,'#e74545',1.25,.62);path(latest.global_path,w,h,'#1677d2',3,[9,6]);path(latest.local_path,w,h,'#23a447',3,[]);if(trail.length>1)path(trail,w,h,'#20a34a',2.2,[]);
for(const p of latest.route_markers||[]){const[a,b]=xy(+p.x,+p.y,w,h),lab=p.label;const col=lab==='B2'?'#e33b36':lab==='TH'?'#f39b17':'#1677d2';ctx.fillStyle='white';ctx.strokeStyle=col;ctx.lineWidth=4;ctx.beginPath();ctx.arc(a,b,9,0,Math.PI*2);ctx.fill();ctx.stroke();ctx.fillStyle=col;ctx.font='800 12px Segoe UI';ctx.fillText(lab,a+13,b-10)}
const p=latest.pose;if(p){const[a,b]=xy(+p.x,+p.y,w,h);ctx.save();ctx.translate(a,b);ctx.rotate(-(+p.yaw||0)*Math.PI/180);ctx.fillStyle='rgba(28,127,229,.12)';ctx.beginPath();ctx.arc(0,0,38,0,Math.PI*2);ctx.fill();ctx.fillStyle='rgba(236,64,64,.16)';ctx.beginPath();ctx.arc(0,0,22,0,Math.PI*2);ctx.fill();ctx.fillStyle='#343a40';ctx.strokeStyle='#111827';ctx.lineWidth=2;ctx.beginPath();ctx.roundRect?ctx.roundRect(-17,-11,34,22,5):ctx.rect(-17,-11,34,22);ctx.fill();ctx.stroke();ctx.fillStyle='#1778d3';ctx.beginPath();ctx.moveTo(24,0);ctx.lineTo(10,-7);ctx.lineTo(10,7);ctx.closePath();ctx.fill();ctx.restore();ctx.fillStyle='#0b4f91';ctx.font='800 10px Segoe UI';ctx.fillText('AMR',a-13,b-27)}}


function eventVi(tx){const u=String(tx||'').toUpperCase();
 if(u.includes('ESP32 ONLINE')||u.includes('ESP32 ĐÃ KẾT NỐI'))return 'ESP32 đã kết nối với hệ thống.';
 if(u.includes('SENSOR1 = OBJECT')||u.includes('CẢM BIẾN 1 = PHÁT HIỆN VẬT'))return 'Cảm biến 1 phát hiện vật.';
 if(u.includes('SENSOR1 = CLEAR')||u.includes('CẢM BIẾN 1 = KHÔNG CÓ VẬT'))return 'Cảm biến 1 không có vật.';
 if(u.includes('SENSOR2 = OBJECT')||u.includes('CẢM BIẾN 2 = PHÁT HIỆN VẬT'))return 'Cảm biến 2 phát hiện vật (chỉ giám sát).';
 if(u.includes('SENSOR2 = CLEAR')||u.includes('CẢM BIẾN 2 = KHÔNG CÓ VẬT'))return 'Cảm biến 2 không có vật.';
 if(u.includes('REQUEST_B2 ACCEPTED')||u.includes('ĐÃ NHẬN NHIỆM VỤ B2'))return 'Đã nhận nhiệm vụ B2.';
 if(u.includes('MISSION PASS')||u.includes('NHIỆM VỤ B2 HOÀN THÀNH'))return 'Nhiệm vụ B2 hoàn thành.';
 if(u.includes('MISSION FAIL')||u.includes('NHIỆM VỤ B2 THẤT BẠI'))return 'Nhiệm vụ B2 thất bại; cần kiểm tra và khôi phục.';
 if(u.includes('NAVIGATE -> B2'))return 'Robot đang di chuyển đến B2.';
 if(u.includes('NAVIGATE -> TH'))return 'Robot đang di chuyển đến vị trí TH.';
 if(u.includes('NAVIGATE -> A_H'))return 'Robot đang trở về A_H.';
 if(u.includes('QR_VALID'))return 'Mã QR hợp lệ.';
 if(u.includes('QR_INVALID')||u.includes('QR_NOT_FOUND'))return 'Mã QR không hợp lệ hoặc chưa nhận được.';
 if(u.includes('[ACT] >>> OPEN'))return 'Cơ cấu đang mở kẹp.';
 if(u.includes('[ACT] >>> PICK'))return 'Cơ cấu đang kẹp và nâng hàng.';
 if(u.includes('[ACT] >>> DROP'))return 'Cơ cấu đang hạ và thả hàng.';
 if(u.includes('[ACT] >>> CLOSE'))return 'Cơ cấu đang đóng kẹp.';
 if(u.includes('SAFE STOP'))return 'Hệ thống đang thực hiện dừng an toàn.';
 return null;}

function updateHealth(d){const h=d.hardware||{},core=d.core||{},s=d.supervisor||{};const order=['BAT_DAU','ESP32','T1','DINH_VI_A_H','NAV2','AN_TOAN_1','AN_TOAN_2','TU_KIEM_TRA','CAMERA','ESP32_SAN_SANG','SAN_SANG'];let pct=0;if(core.status==='SAN_SANG'||core.status==='DANG_THUC_HIEN')pct=100;else if(core.status==='DANG_KHOI_DONG'){const i=Math.max(0,order.indexOf(core.stage));pct=Math.min(95,Math.round((i/(order.length-1))*95))}else if(core.status==='LOI'||core.status==='DUNG_KHAN'||core.status==='DA_DUNG')pct=0;const ready=core.status==='SAN_SANG'||core.status==='DANG_THUC_HIEN';$('healthTitle').textContent=ready?'▥ TRẠNG THÁI SẴN SÀNG':'▥ TIẾN ĐỘ KHỞI ĐỘNG';$('healthPct').textContent=`${pct}%`;$('healthPct').parentElement.style.setProperty('--pct',`${pct}%`);const checks=[['ESP32',d.esp32==='ONLINE'],['LiDAR',h.lidar==='ONLINE'],['Định vị / Nav2',h.nav2==='ONLINE'],['An toàn va chạm',h.collision_monitor==='ONLINE'],['Camera',$('cameraFrame').classList.contains('live')],['Nhận nhiệm vụ',s.work==='1']];$('healthList').innerHTML=checks.map(([n,ok])=>`<div class="${ok?'ok':'no'}">${n}</div>`).join('')}
function update(d){latest=d;if(d.map&&d.map.revision!==mapRevision)decodeMap(d.map);const core=d.core||{},cv=coreVi[core.status]||[core.status||'ĐANG TẮT','gray'];pill('systemState',cv[0],cv[1]);const s=d.supervisor||{};pill('workState',s.work==='1'?'ĐÃ BẬT':'CHƯA BẬT',s.work==='1'?'green':'gray');pill('robotState',s.state==='BUSY'?'ĐANG THỰC HIỆN':'ĐANG CHỜ',s.state==='BUSY'?'blue':'gray');$('missionState').textContent=s.state==='BUSY'?'Đang thực hiện':'Đang chờ';const mm=d.mission_monitor||{};$('missionStage').textContent=stageVi[mm.stage]||stageVi[d.mission]||'--';let [t1,c1]=tSensor(d.sensor1);pill('sensor1',t1,c1);let[t2,c2]=tSensor(d.sensor2);pill('sensor2',t2,c2);const h=d.hardware||{};pill('lidarState',h.lidar==='ONLINE'?'ỔN ĐỊNH':'CHƯA SẴN SÀNG',h.lidar==='ONLINE'?'green':'gray');for(const [id,v] of [['lidarHw',h.lidar],['driveHw',h.stm32_drive],['actHw',h.stm32_actuator]]){const[t,c]=tHw(v);$(id).textContent=t;$(id).style.color=c==='green'?'#188049':'#6d8394'}$('esp32State').textContent=d.esp32==='ONLINE'?'Đã kết nối':'Mất kết nối';$('esp32State').style.color=d.esp32==='ONLINE'?'#188049':'#b33';$('cameraHw').textContent=h.camera==='PRESENT'?'Đã nhận thiết bị':'Chưa sẵn sàng';$('cameraHw').style.color=h.camera==='PRESENT'?'#188049':'#6d8394';badge('locBadge',h.nav2==='ONLINE'?'ĐỊNH VỊ TỐT':'ĐỊNH VỊ CHƯA SẴN SÀNG',h.nav2==='ONLINE'?'green':'gray');badge('scanBadge',h.lidar==='ONLINE'?'DỮ LIỆU LIDAR ỔN ĐỊNH':'LIDAR CHƯA SẴN SÀNG',h.lidar==='ONLINE'?'green':'gray');if(d.pose){$('poseX').textContent=(+d.pose.x).toFixed(3);$('poseY').textContent=(+d.pose.y).toFixed(3);$('poseYaw').textContent=(+d.pose.yaw).toFixed(1);const q=trail[trail.length-1];if(!q||Math.hypot(+d.pose.x-q.x,+d.pose.y-q.y)>.01)trail.push({x:+d.pose.x,y:+d.pose.y});if(trail.length>1200)trail.shift()}if(d.velocity){$('velV').textContent=(+d.velocity.linear||0).toFixed(3);$('velW').textContent=(+d.velocity.angular||0).toFixed(3)}const ready=core.status==='SAN_SANG'&&h.collision_monitor==='ONLINE'&&h.nav2==='ONLINE'&&d.esp32==='ONLINE'&&s.work==='1';badge('normalBadge',ready?'HOẠT ĐỘNG BÌNH THƯỜNG':cv[0],ready?'green':cv[1]);$('techDetails').textContent=`Điều phối: ${core.status||'-'}\nCông đoạn: ${core.stage||'-'}\nThông báo: ${core.message||'-'}\nNav2: ${h.nav2||'-'}\nCollision Monitor: ${h.collision_monitor||'-'}\nESP32: ${d.esp32||'-'}\nCamera thiết bị: ${h.camera||'-'}\nBridge age: ${d.bridge_age_s??'-'} s`;const key=`${core.status}|${core.stage}|${core.message}`;if(key!==lastCoreKey){lastCoreKey=key;if(core.message)log(core.message,core.status==='LOI'?'bad':core.status==='SAN_SANG'?'ok':'')}for(const ev of d.event_log||[]){if(lastEvents.has(ev.id))continue;lastEvents.add(ev.id);if(lastEvents.size>400)lastEvents=new Set(Array.from(lastEvents).slice(-250));const tx=String(ev.text||''),vi=eventVi(tx);if(vi)log(vi,tx.includes('FAIL')||tx.includes('THẤT BẠI')?'bad':tx.includes('PASS')||tx.includes('HOÀN THÀNH')?'ok':'')}updateHealth(d);draw()}

async function post(url){try{const r=await fetch(url,{method:'POST'}),d=await r.json();toast(d.message||d.action||'Đã gửi lệnh');log(d.message||'Đã gửi lệnh',d.ok?'ok':'bad');return d}catch(e){toast('Không kết nối được với giao diện điều khiển');log(String(e),'bad')}}
async function resetAH(){
  const ok=window.confirm(
    'CHỈ THỰC HIỆN KHI ROBOT ĐANG ĐỨNG YÊN TẠI ĐÚNG MỐC VẬT LÝ A_H.\n\n' +
    'Thao tác này chỉ đặt lại vị trí robot trên bản đồ, KHÔNG làm robot di chuyển.\n\n' +
    'Xác nhận robot đang đúng mốc A_H?'
  );
  if(!ok)return;
  try{
    const r=await fetch('/api/dat-lai-a-h',{
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({confirm_physical_ah:true})
    });
    const d=await r.json();
    toast(d.message||'Đã gửi lệnh');
    log(d.message||'Đã gửi lệnh',d.ok?'ok':'bad');
    if(d.ok){
      trail=[];
      draw();
    }
    return d;
  }catch(e){
    toast('Không kết nối được với giao diện điều khiển');
    log(String(e),'bad');
  }
}
$('btnStart').onclick=()=>post('/api/bat-dau');$('btnEmergency').onclick=()=>post('/api/dung-khan');$('btnPause').onclick=()=>post('/api/dung-nhan');$('btnResume').onclick=()=>post('/api/tiep-tuc');$('btnStopSystem').onclick=()=>post('/api/tat-he-thong');$('btnResetAH').onclick=()=>resetAH();$('btnResetAct').onclick=()=>post('/api/co-cau/dat-lai');$('btnOpen').onclick=()=>post('/api/co-cau/mo-kep');$('btnClose').onclick=()=>post('/api/co-cau/dong-kep');$('btnUp').onclick=()=>post('/api/co-cau/nang');$('btnDown').onclick=()=>post('/api/co-cau/ha');$('btnStopAct').onclick=()=>post('/api/co-cau/dung');$('btnClearTrail').onclick=()=>clearTrail();$('zoomIn').onclick=()=>{scale=Math.min(2.5,scale*1.14);draw()};$('zoomOut').onclick=()=>{scale=Math.max(.65,scale/1.14);draw()};$('zoomReset').onclick=()=>{scale=1;draw()};
function connect(){const proto=location.protocol==='https:'?'wss':'ws';ws=new WebSocket(`${proto}://${location.host}/ws/telemetry`);ws.onmessage=e=>{try{update(JSON.parse(e.data))}catch(x){}};ws.onclose=()=>setTimeout(connect,1000)}connect();
async function camera(){try{const r=await fetch('/api/camera/status',{cache:'no-store'}),d=await r.json(),img=$('cameraFrame');if(d.live){img.src=`/api/camera/frame?t=${Date.now()}`;img.classList.add('live');$('cameraPlaceholder').style.display='none';badge('cameraBadge','TRỰC TIẾP','green')}else{img.classList.remove('live');$('cameraPlaceholder').style.display='grid';badge('cameraBadge','CHƯA SẴN SÀNG','gray')}}catch(e){}}cameraTimer=setInterval(camera,700);camera();
window.addEventListener('resize',draw);log('Giao diện HT_V1.1 đã sẵn sàng. Nhấn “BẮT ĐẦU LÀM VIỆC B2” khi robot đang ở A_H.','ok');
