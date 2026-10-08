#!/usr/bin/env python3
from __future__ import annotations

import argparse
import fcntl
import json
import os
import time
from pathlib import Path

try:
    import serial
except Exception as exc:
    serial = None
    SERIAL_IMPORT_ERROR = str(exc)
else:
    SERIAL_IMPORT_ERROR = ""

BAUD = 115200
EXPECTED_VERSION = "AMR_ACTUATOR_F411_V2_12_PICK_LIFT_FIX"
DRIVE_TOKEN = "376B36513235"
ACTUATOR_PREFERRED = Path("/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_379336853235-if00")
DRIVE_BY_ID = Path("/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_376B36513235-if00")
MISSION_LOCK = Path("/tmp/amr_demo_fast_v3_fix8_actuator.lock")
GUI_LOCK = Path("/tmp/amr_gui_v1_6_actuator_recovery.lock")


def emit(ok: bool, **kw):
    d = {"ok": bool(ok)}
    d.update(kw)
    print(json.dumps(d, ensure_ascii=False))


def parse_status(line: str) -> dict:
    out = {}
    if not line.startswith("STATUS "):
        return out
    for tok in line.split()[1:]:
        if "=" not in tok:
            continue
        k, v = tok.split("=", 1)
        out[k] = v
    return out


def candidate_ports() -> list[str]:
    blocked_real = set()
    if DRIVE_BY_ID.exists():
        try:
            blocked_real.add(os.path.realpath(str(DRIVE_BY_ID)))
        except Exception:
            pass
    out: list[str] = []
    if ACTUATOR_PREFERRED.exists():
        out.append(str(ACTUATOR_PREFERRED))
    d = Path("/dev/serial/by-id")
    if d.exists():
        for p in sorted(d.iterdir(), key=lambda x: x.name):
            n = p.name
            if "STM32" not in n or DRIVE_TOKEN in n:
                continue
            try:
                real = os.path.realpath(str(p))
            except Exception:
                continue
            if real in blocked_real:
                continue
            if str(p) not in out:
                out.append(str(p))
    return out


def open_serial(path: str):
    if serial is None:
        raise RuntimeError(f"pyserial unavailable: {SERIAL_IMPORT_ERROR}")
    try:
        return serial.Serial(path, BAUD, timeout=0.08, write_timeout=0.5, exclusive=True)
    except TypeError:
        return serial.Serial(path, BAUD, timeout=0.08, write_timeout=0.5)


def send_line(ser, cmd: str):
    ser.write((cmd.strip() + "\n").encode("ascii"))
    ser.flush()


def read_lines(ser, seconds: float) -> list[str]:
    end = time.monotonic() + seconds
    lines: list[str] = []
    while time.monotonic() < end:
        raw = ser.readline()
        if not raw:
            continue
        s = raw.decode(errors="replace").strip()
        if s:
            lines.append(s)
    return lines


def query(ser, cmd: str, seconds: float = 0.9) -> list[str]:
    try:
        ser.reset_input_buffer()
    except Exception:
        pass
    send_line(ser, cmd)
    return read_lines(ser, seconds)


def find_status(lines: list[str]) -> dict:
    for line in reversed(lines):
        st = parse_status(line)
        if st:
            return st
    return {}


def select_actuator():
    errors = []
    for candidate in candidate_ports():
        ser = None
        try:
            ser = open_serial(candidate)
            time.sleep(0.35)
            lines = query(ser, "VERSION", 1.25)
            version = None
            for line in lines:
                if line.startswith("VERSION "):
                    version = line.replace("VERSION ", "", 1).strip()
                    break
            if version != EXPECTED_VERSION:
                errors.append(f"{candidate}: VERSION={version!r}")
                ser.close()
                continue
            return ser, candidate, os.path.realpath(candidate), version
        except Exception as exc:
            errors.append(f"{candidate}: {type(exc).__name__}: {exc}")
            try:
                if ser is not None:
                    ser.close()
            except Exception:
                pass
    raise RuntimeError("no verified STM32 #2 actuator: " + " | ".join(errors))


def acquire_lock(path: Path):
    fh = path.open("a+")
    fcntl.flock(fh.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    return fh



def legacy_jog_about_1s(ser, cmd: str):
    """Fallback an toàn cho kẹp hiện tại: 3 x JOG 300 ms ~= 0.9-1.0 s.
    Chỉ dùng cho JOG_OPEN/JOG_CLOSE đã field-test; tuyệt đối không dùng PICK/DROP thay NÂNG/HẠ.
    """
    all_lines=[]
    for _ in range(3):
        try:
            ser.reset_input_buffer()
        except Exception:
            pass
        send_line(ser, cmd)
        time.sleep(0.34)
        lines=read_lines(ser, 0.08)
        all_lines.extend(lines)
        if lines and not any(line.startswith(f"ACK {cmd}") for line in lines):
            if any("ERR" in line or "UNKNOWN_CMD" in line for line in lines):
                return False, all_lines
    return True, all_lines

def run(action: str) -> tuple[bool, dict]:
    # Same mission lock as stable mission. If a mission is active, recovery cannot take ownership.
    mission_fh = acquire_lock(MISSION_LOCK)
    gui_fh = acquire_lock(GUI_LOCK)

    # Keep both file descriptors alive for the entire recovery transaction.
    # Closing/releasing either lock early could allow a mission to acquire STM32 #2
    # concurrently with a manual recovery command.
    ser = None
    try:
        ser, port, real, version = select_actuator()
        status_lines = query(ser, "STATUS", 0.7)
        st = find_status(status_lines)
        if not st:
            return False, {"message": "Không nhận được trạng thái từ STM32 cơ cấu", "lines": status_lines, "port": port}

        busy = st.get("BUSY", "1") == "1"
        fault = int(st.get("FAULT", "-1")) if str(st.get("FAULT", "")).lstrip("-").isdigit() else -1

        if action == "status":
            return True, {"message": "Đã đọc trạng thái cơ cấu", "status": st, "port": port, "resolved_port": real, "version": version}

        if action == "stop":
            lines = query(ser, "STOP", 0.8)
            ok = any(line.startswith("ACK STOP") for line in lines)
            return ok, {"message": "Đã dừng cơ cấu" if ok else "Cơ cấu không xác nhận lệnh dừng", "lines": lines, "status_before": st, "port": port}

        if action == "reset_fault":
            if busy:
                return False, {"message": "Cơ cấu đang bận; hãy DỪNG CƠ CẤU trước.", "status": st}
            lines = query(ser, "RESET_FAULT", 0.9)
            ok = any(line.startswith("ACK RESET_FAULT") for line in lines)
            time.sleep(0.15)
            after = find_status(query(ser, "STATUS", 0.7))
            return ok, {"message": "Đã đặt lại lỗi cơ cấu" if ok else "Cơ cấu không xác nhận đặt lại lỗi", "lines": lines, "status_before": st, "status_after": after, "port": port}

        if action == "grip_open":
            if busy:
                return False, {"message": "Cơ cấu đang bận; hãy DỪNG CƠ CẤU trước.", "status": st}
            cmd = "RECOVER_OPEN" if fault != 0 and st.get("RECOVER_OPEN") == "1" else "JOG_OPEN_1S"
            if fault != 0 and cmd != "RECOVER_OPEN":
                return False, {"message": f"Cơ cấu đang có lỗi FAULT={fault}; firmware không cho phép mở phục hồi.", "status": st}
            lines = query(ser, cmd, 0.55)
            if cmd == "JOG_OPEN_1S" and any("UNKNOWN_CMD" in line for line in lines):
                ok, legacy_lines = legacy_jog_about_1s(ser, "JOG_OPEN")
                time.sleep(0.12)
                after = find_status(query(ser, "STATUS", 0.55))
                return ok, {"message": "Đã MỞ KẸP khoảng 1 giây bằng lệnh JOG_OPEN hiện có." if ok else "Không mở được kẹp.", "command": "JOG_OPEN x3", "lines": lines + legacy_lines, "status_before": st, "status_after": after, "port": port}
            ok = any((line.startswith("ACK JOG_OPEN_1S") if cmd == "JOG_OPEN_1S" else line.startswith("ACK RECOVER_OPEN")) for line in lines)
            if cmd == "JOG_OPEN_1S": time.sleep(1.05)
            after = find_status(query(ser, "STATUS", 0.55))
            return ok, {"message": "Đã MỞ KẸP 1 giây." if ok else "Cơ cấu không xác nhận lệnh mở kẹp.", "command": cmd, "lines": lines, "status_before": st, "status_after": after, "port": port}

        if action == "grip_close":
            if busy:
                return False, {"message": "Cơ cấu đang bận; hãy DỪNG CƠ CẤU trước.", "status": st}
            if fault != 0:
                return False, {"message": f"Cơ cấu đang có lỗi FAULT={fault}; hãy ĐẶT LẠI CƠ CẤU trước.", "status": st}
            lines = query(ser, "JOG_CLOSE_1S", 0.55)
            if any("UNKNOWN_CMD" in line for line in lines):
                ok, legacy_lines = legacy_jog_about_1s(ser, "JOG_CLOSE")
                time.sleep(0.12)
                after = find_status(query(ser, "STATUS", 0.55))
                return ok, {"message": "Đã ĐÓNG KẸP khoảng 1 giây bằng lệnh JOG_CLOSE hiện có." if ok else "Không đóng được kẹp.", "command": "JOG_CLOSE x3", "lines": lines + legacy_lines, "status_before": st, "status_after": after, "port": port}
            ok = any(line.startswith("ACK JOG_CLOSE_1S") for line in lines)
            time.sleep(1.05)
            after = find_status(query(ser, "STATUS", 0.55))
            return ok, {"message": "Đã ĐÓNG KẸP 1 giây." if ok else "Cơ cấu không xác nhận lệnh đóng kẹp.", "command": "JOG_CLOSE_1S", "lines": lines, "status_before": st, "status_after": after, "port": port}

        if action == "capabilities":
            lines = query(ser, "CAPS", 0.8)
            caps = " ".join(lines)
            supported = ("JOG_LIFT_UP" in caps and "JOG_LIFT_DOWN" in caps)
            return True, {"message":"Đã đọc khả năng firmware","capabilities":caps,"manual_lift_supported":supported,"lines":lines,"status":st,"port":port,"version":version}

        if action in ("lift_up", "lift_down"):
            if busy:
                return False, {"message":"Cơ cấu đang bận; hãy DỪNG CƠ CẤU trước.","status":st}
            if fault != 0:
                return False, {"message":f"Cơ cấu đang có lỗi FAULT={fault}; hãy ĐẶT LẠI CƠ CẤU trước.","status":st}
            cmd = "JOG_LIFT_UP_1S" if action == "lift_up" else "JOG_LIFT_DOWN_1S"
            lines = query(ser, cmd, 1.0)
            if any("UNKNOWN_CMD" in line for line in lines) or not lines:
                return False, {"message":f"Firmware hiện tại chưa có {cmd}. Cần bổ sung lệnh NÂNG/HẠ độc lập; không dùng PICK/DROP thay thế.","firmware_update_required":True,"command":cmd,"lines":lines,"status":st,"port":port}
            ok = any(line.startswith(f"ACK {cmd}") for line in lines)
            time.sleep(1.15)
            after = find_status(query(ser, "STATUS", 0.7))
            return ok, {"message":f"{cmd} 1000 ms ACK" if ok else f"{cmd} không ACK","command":cmd,"lines":lines,"status_before":st,"status_after":after,"port":port,"version":version}

        return False, {"message": f"unknown recovery action: {action}"}
    finally:
        try:
            if ser is not None:
                ser.close()
        except Exception:
            pass
        for _fh in (gui_fh, mission_fh):
            try:
                fcntl.flock(_fh.fileno(), fcntl.LOCK_UN)
            except Exception:
                pass
            try:
                _fh.close()
            except Exception:
                pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("action", choices=["status", "capabilities", "stop", "reset_fault", "grip_open", "grip_close", "lift_up", "lift_down"])
    args = ap.parse_args()
    try:
        ok, data = run(args.action)
        emit(ok, action=args.action, **data)
        return 0 if ok else 2
    except BlockingIOError:
        emit(False, action=args.action, message="Cơ cấu đang được nhiệm vụ hoặc thao tác khác sử dụng.")
        return 3
    except Exception as exc:
        emit(False, action=args.action, message=f"{type(exc).__name__}: {exc}")
        return 4


if __name__ == "__main__":
    raise SystemExit(main())
