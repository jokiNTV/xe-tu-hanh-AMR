#!/usr/bin/env python3
from __future__ import annotations
import datetime, json, os, signal, socket, subprocess, threading, time
from pathlib import Path

HOST='0.0.0.0'; PORT=5005; LOCAL_CONTROL_IP='127.0.0.1'; OBJECT_HOLD_S=5.0
MISSION_FILE=Path(os.environ.get('AMR_B2_MISSION_FILE','/home/vohoaibao/Downloads/AMR_DEMO_FAST_V3_FIX11_B2_QR_PACKAGE/AMR_DEMO_FAST_V3_FIX11_B2_QR_YAW_RETRY1_ACT_V213C.py'))
MISSION_DIR=MISSION_FILE.parent
LOG_DIR=Path(os.environ.get('HT_V1_LOG_DIR',str(Path.home()/'AMR_LOGS'))); LOG_DIR.mkdir(parents=True,exist_ok=True)
CAM_STATUS=Path('/tmp/amr_b2_camera_broker/status.json'); CAM_FRAME=Path('/tmp/amr_b2_camera_broker/latest.jpg')
CAM_RUNNER=Path(os.environ.get('AMR_CAMERA_RUNNER',str(Path(__file__).resolve().parent/'HT_V1_CAM_RUN_AMR.py')))
state='IDLE'; trusted_esp32_ip=None; sensor1_object=None; sensor1_object_since=None; sensor1_clear_since=None; sensor1_latched=False; sensor2_object=False
work_enabled=os.environ.get('AMR_B2_WORK_AUTO','0')=='1'; mission_proc=None; mission_log_path=None


def now(): return datetime.datetime.now().strftime('%H:%M:%S')
def stamp(): return datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
def log(msg): print(f'[{now()}] {msg}',flush=True)
def sensor1_text(): return 'UNKNOWN' if sensor1_object is None else ('OBJECT' if sensor1_object else 'CLEAR')

def camera_ready():
    try:
        d=json.loads(CAM_STATUS.read_text(encoding='utf-8')); age=time.time()-float(d.get('timestamp') or 0)
        return d.get('state')=='LIVE' and age<1.5 and CAM_FRAME.exists()
    except Exception: return False

def trigger_remaining():
    if not work_enabled or sensor1_object is not True or sensor1_object_since is None or sensor1_latched: return '-'
    return f'{max(0.0,OBJECT_HOLD_S-(time.monotonic()-sensor1_object_since)):.1f}'

def local_status_text():
    mission_pid='-'
    if mission_proc is not None and mission_proc.poll() is None: mission_pid=str(mission_proc.pid)
    clear_age='-'
    if sensor1_object is False and sensor1_clear_since is not None: clear_age=f'{time.monotonic()-sensor1_clear_since:.1f}'
    return (f'STATUS state={state} work={int(work_enabled)} armed={int(work_enabled)} auto_cycle={int(work_enabled)} '
            f'rearm_pending=0 rearm_in_s=- trigger_in_s={trigger_remaining()} latched={int(sensor1_latched)} '
            f'sensor1={sensor1_text()} clear_age_s={clear_age} sensor2={"OBJECT" if sensor2_object else "CLEAR"} '
            f'esp32={trusted_esp32_ip or "-"} camera={"LIVE" if camera_ready() else "WAIT"} mission_pid={mission_pid}')

def stream_mission_output(proc,logfile):
    with open(logfile,'a',encoding='utf-8') as f:
        for line in proc.stdout:
            line=line.rstrip(); print(f'[B2] {line}',flush=True); f.write(line+'\n'); f.flush()

def start_b2():
    global state,mission_proc,mission_log_path
    if state!='IDLE': log('TỪ CHỐI B2: hệ thống đang bận'); return False
    if not MISSION_FILE.is_file(): log(f'TỪ CHỐI B2: thiếu tệp nhiệm vụ: {MISSION_FILE}'); return False
    if not CAM_RUNNER.is_file(): log(f'TỪ CHỐI B2: thiếu bộ chia sẻ camera: {CAM_RUNNER}'); return False
    if not camera_ready(): log('TỪ CHỐI B2: camera chưa sẵn sàng'); return False
    mission_log_path=LOG_DIR/f'MISSION_B2_{stamp()}.log'
    bash_cmd=f'''set -e
source /opt/ros/humble/setup.bash
source /home/vohoaibao/amr_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export AMR_CAMERA_BROKER_FRAME=/tmp/amr_b2_camera_broker/latest.jpg
cd "{MISSION_DIR}"
exec python3 "{CAM_RUNNER}" "{MISSION_FILE}"
'''
    log('='*68); log('ĐÃ NHẬN NHIỆM VỤ B2 - CẢM BIẾN 1 CÓ VẬT LIÊN TỤC 5 GIÂY'); log('TRẠNG THÁI: ĐANG CHỜ -> ĐANG THỰC HIỆN'); log(f'NHẬT KÝ: {mission_log_path}'); log('='*68)
    try: mission_proc=subprocess.Popen(['bash','-lc',bash_cmd],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
    except Exception as exc: log(f'KHÔNG KHỞI ĐỘNG ĐƯỢC B2: {exc}'); mission_proc=None; state='IDLE'; return False
    state='BUSY'; threading.Thread(target=stream_mission_output,args=(mission_proc,mission_log_path),daemon=True).start(); return True

def stop_mission_if_running(reason='DỪNG'):
    global mission_proc
    if mission_proc is None or mission_proc.poll() is not None: return
    log(f'{reason}: đang yêu cầu nhiệm vụ B2 dừng an toàn...')
    try: mission_proc.send_signal(signal.SIGINT); mission_proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        log('Nhiệm vụ chưa thoát sau SIGINT -> gửi terminate'); mission_proc.terminate()
    except Exception as exc: log(f'Cảnh báo khi dừng nhiệm vụ: {exc}')

def set_work(enabled,reason):
    global work_enabled,sensor1_object_since
    work_enabled=enabled
    if enabled:
        if sensor1_object is True and sensor1_object_since is None: sensor1_object_since=time.monotonic()
        log(f'CHẾ ĐỘ LÀM VIỆC = BẬT | Cảm biến 1 giữ vật {OBJECT_HOLD_S:.1f} giây -> chạy B2 | {reason}')
    else:
        sensor1_object_since=None; log(f'CHẾ ĐỘ LÀM VIỆC = TẮT | {reason}')

def service_work():
    global sensor1_latched,work_enabled
    if not work_enabled or state!='IDLE' or sensor1_latched: return
    if trusted_esp32_ip is None or sensor1_object is not True or sensor1_object_since is None: return
    if (time.monotonic()-sensor1_object_since)<OBJECT_HOLD_S: return
    sensor1_latched=True
    if not start_b2(): work_enabled=False; log('ĐÃ TẮT CHẾ ĐỘ LÀM VIỆC: không thể bắt đầu nhiệm vụ B2')

def handle_local(msg,addr,sock):
    if msg=='STATUS': sock.sendto(local_status_text().encode(),addr); return True
    if msg in ('WORK_ON','START_WORK_B2','ARM_B2'):
        set_work(True,'LỆNH NỘI BỘ'); sock.sendto(b'OK WORK_ON',addr); return True
    if msg in ('WORK_OFF','STOP_WORK_B2','DISARM_B2'):
        set_work(False,'LỆNH NỘI BỘ'); sock.sendto(b'OK WORK_OFF',addr); return True
    if msg=='EMERGENCY_STOP':
        set_work(False,'DỪNG KHẨN'); stop_mission_if_running('DỪNG KHẨN'); sock.sendto(b'OK EMERGENCY_STOP',addr); return True
    return False

sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); sock.bind((HOST,PORT)); sock.settimeout(.20)
print('='*72); print(' BỘ GIÁM SÁT NHIỆM VỤ HT_V1.1 - TUYẾN B2'); print(' CẢM BIẾN 1 CÓ VẬT LIÊN TỤC 5 GIÂY -> CHẠY B2'); print(' CẢM BIẾN 2 CHỈ GIÁM SÁT'); print('='*72)
log('TRẠNG THÁI = ĐANG CHỜ'); log(f'CHẾ ĐỘ LÀM VIỆC = {"BẬT" if work_enabled else "TẮT"}'); log('Đang chờ ESP32 bắt tay...')
try:
    while True:
        if state=='BUSY' and mission_proc is not None:
            rc=mission_proc.poll()
            if rc is not None:
                log('='*68); log('NHIỆM VỤ B2 HOÀN THÀNH' if rc==0 else f'NHIỆM VỤ B2 THẤT BẠI / ĐÃ DỪNG | mã={rc}'); log('TRẠNG THÁI: ĐANG THỰC HIỆN -> ĐANG CHỜ'); state='IDLE'; mission_proc=None
                if rc!=0: work_enabled=False; log('ĐÃ TẮT CHẾ ĐỘ LÀM VIỆC - cần khôi phục/kiểm tra trước khi chạy lại')
                elif sensor1_object is True: log('B2 hoàn thành nhưng Cảm biến 1 vẫn có vật -> chờ CLEAR trước nhiệm vụ mới')
                else: log('B2 hoàn thành -> sẵn sàng chờ đơn tiếp theo')
                log('='*68)
        service_work()
        try: data,addr=sock.recvfrom(1024)
        except socket.timeout: continue
        msg=data.decode(errors='replace').strip(); sender_ip=addr[0]
        if sender_ip==LOCAL_CONTROL_IP:
            if handle_local(msg,addr,sock): continue
            sock.sendto(f'ERR UNKNOWN LOCAL COMMAND: {msg}'.encode(),addr); continue
        if msg=='ESP32_ONLINE': trusted_esp32_ip=sender_ip; log(f'ESP32 ĐÃ KẾT NỐI | IP={sender_ip} PORT={addr[1]}'); sock.sendto(b'ACK:ESP32_ONLINE',addr); continue
        if trusted_esp32_ip is None: log(f'BỎ QUA: ESP32 chưa bắt tay | {addr} -> {msg}'); continue
        if sender_ip!=trusted_esp32_ip: log(f'BỎ QUA THIẾT BỊ LẠ {addr} -> {msg}'); continue
        sock.sendto(f'ACK:{msg}'.encode(),addr)
        if msg=='SENSOR1_CLEAR':
            changed=sensor1_object is not False; sensor1_object=False; sensor1_object_since=None
            if changed: sensor1_clear_since=time.monotonic(); sensor1_latched=False; log('CẢM BIẾN 1 = KHÔNG CÓ VẬT | SẴN SÀNG NHẬN ĐƠN MỚI')
            elif sensor1_clear_since is None: sensor1_clear_since=time.monotonic()
            continue
        if msg=='SENSOR1_OBJECT':
            sensor1_clear_since=None
            if sensor1_object is not True:
                sensor1_object=True; sensor1_object_since=time.monotonic(); log('CẢM BIẾN 1 = PHÁT HIỆN VẬT')
                if work_enabled and state=='IDLE' and not sensor1_latched: log(f'BẮT ĐẦU ĐẾM {OBJECT_HOLD_S:.1f} GIÂY')
            elif sensor1_object_since is None: sensor1_object_since=time.monotonic()
            if state=='BUSY': log('Cảm biến 1 có vật khi đang chạy -> chỉ ghi nhận')
            elif not work_enabled: log('Cảm biến 1 có vật nhưng chế độ làm việc đang TẮT -> không chạy nhiệm vụ')
            elif sensor1_latched: log('Vật hiện tại đã được ghi nhận -> phải CLEAR trước nhiệm vụ tiếp theo')
            continue
        if msg=='SENSOR2_OBJECT':
            if not sensor2_object: sensor2_object=True; log('CẢM BIẾN 2 = PHÁT HIỆN VẬT | CHỈ GIÁM SÁT')
            continue
        if msg=='SENSOR2_CLEAR':
            if sensor2_object: sensor2_object=False; log('CẢM BIẾN 2 = KHÔNG CÓ VẬT')
            continue
        log(f'TIN NHẮN KHÔNG XÁC ĐỊNH: {msg}')
except KeyboardInterrupt:
    log('Đang dừng bộ giám sát'); work_enabled=False; stop_mission_if_running()
finally: sock.close(); log('Bộ giám sát đã thoát')
