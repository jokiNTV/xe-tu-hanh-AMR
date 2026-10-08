#!/usr/bin/env python3
from __future__ import annotations
import json, os, signal, time
from datetime import datetime
from pathlib import Path
try:
    import cv2
except Exception as exc:
    raise SystemExit(f"camera broker requires cv2: {exc}")
CAM_BY_ID=Path('/dev/v4l/by-id/usb-XIFT_Web_Camera_SN00010-video-index0')
BASE=Path('/tmp/amr_b2_camera_broker'); FRAME=BASE/'latest.jpg'; STATUS=BASE/'status.json'
REC_DIR=Path.home()/'Downloads'/'AMR_CAMERA_RECORDINGS_B2'
WIDTH=640; HEIGHT=480; FPS=10.0; SEGMENT_S=900.0; MAX_SEGMENTS=20; RUN=True
def _sig(_s,_f):
    global RUN; RUN=False
signal.signal(signal.SIGINT,_sig); signal.signal(signal.SIGTERM,_sig)
def atomic_json(obj):
    BASE.mkdir(parents=True,exist_ok=True); tmp=STATUS.with_suffix('.tmp'); tmp.write_text(json.dumps(obj,ensure_ascii=False),encoding='utf-8'); os.replace(tmp,STATUS)
def atomic_jpg(buf):
    BASE.mkdir(parents=True,exist_ok=True); tmp=FRAME.with_suffix('.tmp'); tmp.write_bytes(buf.tobytes()); os.replace(tmp,FRAME)
def prune_recordings():
    REC_DIR.mkdir(parents=True,exist_ok=True)
    files=sorted([*REC_DIR.glob('AMR_B2_CAM_*.mp4'),*REC_DIR.glob('AMR_B2_CAM_*.avi')],key=lambda p:p.stat().st_mtime)
    while len(files)>=MAX_SEGMENTS:
        old=files.pop(0)
        try: old.unlink()
        except Exception: pass
def new_writer(w,h):
    REC_DIR.mkdir(parents=True,exist_ok=True); prune_recordings(); stamp=datetime.now().strftime('%Y%m%d_%H%M%S')
    mp4=REC_DIR/f'AMR_B2_CAM_{stamp}.mp4'; wr=cv2.VideoWriter(str(mp4),cv2.VideoWriter_fourcc(*'mp4v'),FPS,(w,h))
    if wr.isOpened(): return wr,mp4
    try: wr.release()
    except Exception: pass
    avi=REC_DIR/f'AMR_B2_CAM_{stamp}.avi'; wr=cv2.VideoWriter(str(avi),cv2.VideoWriter_fourcc(*'MJPG'),FPS,(w,h)); return wr,avi
def main():
    BASE.mkdir(parents=True,exist_ok=True)
    if not CAM_BY_ID.exists(): atomic_json({'state':'ERROR','note':f'camera missing: {CAM_BY_ID}','timestamp':time.time()}); return 2
    dev=os.path.realpath(str(CAM_BY_ID)); cap=cv2.VideoCapture(dev,cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG')); cap.set(cv2.CAP_PROP_FRAME_WIDTH,WIDTH); cap.set(cv2.CAP_PROP_FRAME_HEIGHT,HEIGHT); cap.set(cv2.CAP_PROP_FPS,FPS)
    if not cap.isOpened(): atomic_json({'state':'ERROR','note':f'cannot open {dev}','timestamp':time.time()}); return 3
    ok,frame=cap.read()
    if not ok or frame is None: cap.release(); atomic_json({'state':'ERROR','note':'initial frame failed','timestamp':time.time()}); return 4
    h,w=frame.shape[:2]; writer,rec_path=new_writer(w,h)
    if not writer.isOpened(): cap.release(); atomic_json({'state':'ERROR','note':'VideoWriter open failed','timestamp':time.time()}); return 5
    seg_start=time.monotonic(); frames=0; last_status=0.0
    try:
        while RUN:
            ok,frame=cap.read()
            if not ok or frame is None:
                atomic_json({'state':'ERROR','note':'frame read failed','timestamp':time.time(),'recording':str(rec_path)}); time.sleep(.05); continue
            okj,buf=cv2.imencode('.jpg',frame,[int(cv2.IMWRITE_JPEG_QUALITY),82])
            if okj: atomic_jpg(buf)
            rec=frame.copy(); ts=datetime.now().strftime('%Y-%m-%d %H:%M:%S'); cv2.putText(rec,ts,(10,h-14),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),2,cv2.LINE_AA); writer.write(rec); frames+=1
            if time.monotonic()-seg_start>=SEGMENT_S: writer.release(); writer,rec_path=new_writer(w,h); seg_start=time.monotonic()
            now=time.monotonic()
            if now-last_status>=.25:
                atomic_json({'state':'LIVE','note':'CAMERA BROKER LIVE • GUI + QR SHARE','timestamp':time.time(),'frame_fresh':True,'width':w,'height':h,'fps':FPS,'frames':frames,'device':dev,'recording':str(rec_path)}); last_status=now
    finally:
        try: writer.release()
        except Exception: pass
        try: cap.release()
        except Exception: pass
        FRAME.unlink(missing_ok=True); atomic_json({'state':'STOPPED','note':'camera broker stopped','timestamp':time.time(),'frame_fresh':False})
    return 0
if __name__=='__main__': raise SystemExit(main())
