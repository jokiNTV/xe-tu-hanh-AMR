#!/usr/bin/env python3
from __future__ import annotations

"""Bộ chạy nhiệm vụ B2 cho HT_V1.1.

Không sửa thuật toán nhiệm vụ ổn định. Chỉ bọc các lệnh `ros2 param get`
bằng retry hữu hạn để tránh fail giả do ROS2 CLI chậm trong lúc graph vừa ổn định.
Nếu sau nhiều lần vẫn không đọc được tham số, giữ nguyên fail-closed của mission.
"""

import os
import runpy
import subprocess
import sys
import time
from pathlib import Path

MISSION_GOC = Path(os.environ.get(
    "HT_V1_MISSION_GOC",
    "/home/vohoaibao/Downloads/AMR_DEMO_FAST_V3_FIX11_B2_QR_PACKAGE/AMR_DEMO_FAST_V3_FIX11_B2_QR_YAW_RETRY1_ACT_V213C.py",
))

_ORIG_CHECK_OUTPUT = subprocess.check_output


def _is_ros2_param_get(args) -> bool:
    try:
        a = list(args)
    except Exception:
        return False
    if not a:
        return False
    exe = os.path.basename(str(a[0]))
    return exe == "ros2" and len(a) >= 3 and str(a[1]) == "param" and str(a[2]) == "get"


def _check_output_retry(*popenargs, **kwargs):
    args = popenargs[0] if popenargs else kwargs.get("args")
    if not _is_ros2_param_get(args):
        return _ORIG_CHECK_OUTPUT(*popenargs, **kwargs)

    tries = int(os.environ.get("HT_V1_PARAM_ATTEMPTS", "3"))
    per_call = float(os.environ.get("HT_V1_PARAM_TIMEOUT_S", "6"))
    requested = kwargs.get("timeout")
    if requested is not None:
        try:
            per_call = min(per_call, float(requested))
        except Exception:
            pass

    last_exc: BaseException | None = None
    cmd_text = " ".join(str(x) for x in list(args))
    for i in range(1, tries + 1):
        k = dict(kwargs)
        k["timeout"] = per_call
        try:
            out = _ORIG_CHECK_OUTPUT(*popenargs, **k)
            if i > 1:
                print(f"[HT_V1.1] Đọc tham số ROS2 thành công ở lần {i}/{tries}: {cmd_text}", flush=True)
            return out
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError) as exc:
            last_exc = exc
            if i < tries:
                print(f"[HT_V1.1] Lệnh tham số chưa phản hồi ở lần {i}/{tries}; đang thử lại: {cmd_text}", flush=True)
                time.sleep(0.6 + 0.4 * i)

    assert last_exc is not None
    raise last_exc


def main() -> int:
    if not MISSION_GOC.is_file():
        print(f"KHÔNG TÌM THẤY NHIỆM VỤ B2 GỐC: {MISSION_GOC}", file=sys.stderr, flush=True)
        return 2

    subprocess.check_output = _check_output_retry
    os.environ["HT_V1_MISSION_GOC"] = str(MISSION_GOC)
    sys.argv = [str(MISSION_GOC), *sys.argv[1:]]
    try:
        runpy.run_path(str(MISSION_GOC), run_name="__main__")
        return 0
    except SystemExit as exc:
        code = exc.code
        if code is None:
            return 0
        if isinstance(code, int):
            return code
        print(code, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
