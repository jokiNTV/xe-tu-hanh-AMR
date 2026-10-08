#!/usr/bin/env python3
from __future__ import annotations
import hashlib, re, time
from pathlib import Path
from typing import Any

HOME=Path.home()
SUP_PAT=HOME/'Downloads'
MISSION_LOG_DIR=HOME/'AMR_LOGS'
CAM_BY_ID=Path('/dev/v4l/by-id/usb-XIFT_Web_Camera_SN00010-video-index0')
_cache_t=0.0
_cache:dict[str,Any]={}

def _newest(paths):
    xs=[p for p in paths if p.is_file()]
    if not xs: return None
    return max(xs,key=lambda p:p.stat().st_mtime)

def _tail(path:Path|None,max_lines=300,max_bytes=384*1024):
    if path is None or not path.is_file(): return []
    try:
        with path.open('rb') as f:
            f.seek(0,2); size=f.tell(); f.seek(max(0,size-max_bytes))
            raw=f.read()
        return raw.decode('utf-8','replace').splitlines()[-max_lines:]
    except Exception:
        return []

def _stage(lines, active:bool):
    stage='STARTING' if active else 'IDLE'; result='--'; failed='--'; actuator='IDLE'
    qr={'status':'IDLE','data':'--','hits':'--','expected':'BOX_001'}
    for line in lines:
        u=line.upper()
        if 'PREFLIGHT PASS' in u: stage='PREFLIGHT'
        if 'START MAP:' in u: stage='START_CHECK'
        if 'NAVIGATE -> B2' in u: stage='TO_B2'
        if 'ONE-SHOT YAW ALIGN -> B2' in u: stage='ALIGN_B2'
        if '[QR] DATA=' in line:
            stage='QR_SCAN'; qr['status']='SCANNING'
            m=re.search(r"DATA=(['\"])(.*?)\1",line); h=re.search(r'HITS=(\d+/\d+)',line)
            if m: qr['data']=m.group(2)
            if h: qr['hits']=h.group(1)
        if '[QR] QR_VALID' in line:
            stage='QR_VALID'; qr['status']='VALID'; result='QR_VALID'
            m=re.search(r"QR_VALID:\s*(['\"])(.*?)\1",line)
            if m: qr['data']=m.group(2)
        if '[QR] QR_INVALID' in line:
            stage='QR_REJECT'; qr['status']='INVALID'; result='QR_INVALID'
        if '[QR] QR_NOT_FOUND' in line:
            stage='QR_REJECT'; qr['status']='NOT_FOUND'; result='QR_NOT_FOUND'
        if '[ACT] >>> OPEN' in line: stage='OPEN'; actuator='OPENING'
        if 'OPEN_DONE' in line: stage='OPEN_DONE'; actuator='OPEN'
        if 'DRIVE ON HEADING: B2_PICK_FORWARD' in u: stage='PICK_FORWARD'
        if '[ACT] >>> PICK' in line: stage='PICK'; actuator='PICKING'
        if 'PICK_DONE' in line: stage='PICK_DONE'; actuator='LIFTED'
        if 'BACKUP: B2_PICK_EXIT_BACKUP' in u: stage='PICK_EXIT'
        if 'NAVIGATE -> TH' in u: stage='TO_TH'
        if 'ONE-SHOT YAW ALIGN -> TH' in u: stage='ALIGN_TH'
        if 'ODOM HEADING LOCK: TH_DROP_FORWARD_40CM' in u: stage='DROP_APPROACH'
        if '[ACT] >>> DROP' in line: stage='DROP'; actuator='DROPPING'
        if 'DROP_DONE' in line: stage='DROP_DONE'; actuator='OPEN'
        if 'BACKUP: TH_DROP_BACKUP_40CM' in u: stage='DROP_RETREAT'
        if '[ACT] >>> CLOSE' in line: stage='CLOSE'; actuator='CLOSING'
        if 'CLOSE_DONE' in line: stage='CLOSE_DONE'; actuator='CLOSED'
        if 'NAVIGATE -> A_H' in u: stage='RETURN_A_H'
        if 'ONE-SHOT YAW ALIGN -> A_H' in u: stage='ALIGN_A_H'
        if 'SAFE STOP START:' in u: stage='STOPPING'; result='STOPPED'
        if 'MISSION STOPPED AT STAGE:' in u:
            stage='FAIL'; result='FAIL'; failed=line.split(':',1)[-1].strip()
        if 'MISSION PASS:' in u:
            stage='PASS'; result='PASS'
        if 'MISSION COMPLETE:' in u:
            stage='COMPLETE'; result='QR_REJECT_COMPLETE' if 'NO PICK' in u or 'QR_' in u else 'COMPLETE'
    if not active:
        # Historical logs may still contain PICK/DROP states.  Never render those
        # as a current realtime actuator state after the supervisor is offline.
        current='IDLE'
        actuator='IDLE'
        qr={**qr, 'status':'IDLE'}
    else:
        current=stage
    return current, result, failed, actuator, qr

def _meaningful_sup(lines):
    keys=('SENSOR','RE-ARM','MISSION','STATE:','STATE =','ESP32','ARMED','DISARM','BLOCKED','PASS','FAIL','AUTO CYCLE','REQUEST_B2')
    return [x for x in lines if any(k in x.upper() for k in keys)][-32:]

def _meaningful_mission(lines):
    keys=('NAVIGATE ->','ONE-SHOT YAW ALIGN','[QR]','[ACT]','DRIVE ON HEADING','BACKUP:','ODOM HEADING LOCK','MISSION PASS','MISSION STOPPED','MISSION COMPLETE','SAFE STOP','PREFLIGHT PASS','START MAP:')
    return [x for x in lines if any(k in x.upper() for k in keys)][-38:]

def collect_runtime_monitor(supervisor:dict[str,str]|None=None)->dict[str,Any]:
    global _cache_t,_cache
    now=time.monotonic()
    if now-_cache_t < 0.45 and _cache:
        return _cache
    supervisor=supervisor or {}
    active=str(supervisor.get('mission_pid','-')) not in ('-','','None','none')
    sup_path=_newest(list((HOME/'AMR_LOGS').glob('*/SUPERVISOR.log')) + list(SUP_PAT.glob('AMR_STARTUP_V5_B2_FINAL_RC1_*/SUPERVISOR.log')))
    mission_path=_newest(list(MISSION_LOG_DIR.glob('*/MISSION_B2_*.log')) + list((HOME/'Downloads'/'AMR_SUPERVISOR_LOGS').glob('B2_WORKMODE_*.log'))) if MISSION_LOG_DIR.exists() else None
    sup_lines=_tail(sup_path,260)
    mission_lines=_tail(mission_path,520)
    stage,last_result,failed,actuator,qr=_stage(mission_lines,active)

    # Only label log lines as realtime when their files are fresh.  Old mission
    # logs remain available through last_result/failed_stage but are not replayed
    # as if they were happening now.
    def fresh(path, limit_s=20.0):
        if path is None or not path.is_file():
            return False
        try:
            return (time.time() - path.stat().st_mtime) <= limit_s
        except Exception:
            return False

    events=[]
    sources=[]
    if fresh(sup_path):
        sources.append(('SUP',_meaningful_sup(sup_lines)))
    if active and fresh(mission_path):
        sources.append(('B2',_meaningful_mission(mission_lines)))
    for src,lines in sources:
        for idx,line in enumerate(lines):
            text=line.strip()
            if not text: continue
            events.append({'id':hashlib.sha1(f'{src}|{text}|{idx}'.encode()).hexdigest()[:16],'source':src,'text':text})
    events=events[-60:]
    revision=hashlib.sha1('\n'.join(x['source']+'|'+x['text'] for x in events).encode()).hexdigest()[:12] if events else '0'
    camera_present=CAM_BY_ID.exists()
    _cache={
        'active':active,
        'stage':stage,
        'last_result':last_result,
        'failed_stage':failed,
        'actuator_state':actuator,
        'qr':qr,
        'camera':{
            'present':camera_present,
            'mode':'BROKER_SHARED',
            'preview':'LIVE_SHARED',
            'note':'Camera Broker là owner duy nhất: GUI + QR mission dùng chung frame; recording chạy liên tục.',
        },
        'supervisor_log':str(sup_path) if sup_path else '-',
        'mission_log':str(mission_path) if mission_path else '-',
        'events':events,
        'log_revision':revision,
    }
    _cache_t=now
    return _cache
