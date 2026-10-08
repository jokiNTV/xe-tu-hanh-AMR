#!/usr/bin/env python3
from __future__ import annotations
import os, runpy, sys, time
from pathlib import Path

FRAME = Path(os.environ.get('AMR_CAMERA_BROKER_FRAME','/tmp/amr_b2_camera_broker/latest.jpg'))

def install_proxy():
    import cv2
    import numpy as np
    OriginalVideoCapture = cv2.VideoCapture

    class BrokerCapture:
        def __init__(self,*args,**kwargs):
            self._released=False; self._grab=None
        def isOpened(self):
            try:
                return (not self._released) and FRAME.exists() and (time.time()-FRAME.stat().st_mtime)<2.0
            except Exception:
                return False
        def set(self,*args,**kwargs): return True
        def get(self,prop):
            if prop==cv2.CAP_PROP_FRAME_WIDTH: return 640.0
            if prop==cv2.CAP_PROP_FRAME_HEIGHT: return 480.0
            if prop==cv2.CAP_PROP_FPS: return 10.0
            return 0.0
        def read(self):
            if self._released: return False,None
            deadline=time.monotonic()+1.5
            while time.monotonic()<deadline:
                try:
                    raw=FRAME.read_bytes()
                    arr=np.frombuffer(raw,dtype=np.uint8)
                    frame=cv2.imdecode(arr,cv2.IMREAD_COLOR)
                    if frame is not None: return True,frame
                except Exception:
                    pass
                time.sleep(.03)
            return False,None
        def grab(self):
            ok,frame=self.read(); self._grab=frame if ok else None; return ok
        def retrieve(self): return (self._grab is not None,self._grab)
        def release(self): self._released=True

    def proxy(source=0,*args,**kwargs):
        # The stable mission opens the XIFT camera by device path. Route camera
        # opens to broker frames; preserve any non-camera numeric/video uses.
        if isinstance(source,(str,os.PathLike)) or source==0:
            return BrokerCapture(source,*args,**kwargs)
        return OriginalVideoCapture(source,*args,**kwargs)
    cv2.VideoCapture=proxy

def main():
    if len(sys.argv)<2:
        print('usage: run_mission_with_camera_broker.py <mission.py> [args...]',file=sys.stderr); return 2
    mission=Path(sys.argv[1]).resolve()
    if not mission.is_file():
        print(f'mission not found: {mission}',file=sys.stderr); return 3
    if not FRAME.exists():
        print(f'camera broker frame missing: {FRAME}',file=sys.stderr); return 4
    install_proxy()
    sys.argv=[str(mission),*sys.argv[2:]]
    sys.path.insert(0,str(mission.parent))
    runpy.run_path(str(mission),run_name='__main__')
    return 0

if __name__=='__main__': raise SystemExit(main())
