import time
import serial
from threading import Event
from queue import Empty, Full
from typing import Optional

GPS_PORT = "/dev/serial0"
GPS_BAUD = 9600


def _q_put_latest(q, item) -> None:
    try:
        while True:
            q.get_nowait()
    except Exception:
        pass
    try:
        q.put_nowait(item)
    except Exception:
        pass


def _nmea_degmin_to_decimal(value_str: str, hemi: str) -> Optional[float]:
    if not value_str or not hemi:
        return None
    try:
        v = float(value_str)
    except ValueError:
        return None

    deg = int(v // 100)
    minutes = v - deg * 100.0
    dec = deg + (minutes / 60.0)

    hemi = hemi.upper()
    if hemi in ("S", "W"):
        dec = -dec
    return dec


def gps_worker(queues, stop_event: Event):
    print("[GPS] Iniciando GPS real")

    try:
        ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=1)
    except Exception as e:
        print(f"[GPS] Erro ao abrir GPS: {e}")
        # mantém vivo publicando "sem fix" com baixa taxa
        while not stop_event.is_set():
            now = time.time()
            pkt = {"timestamp": now, "lat": None, "lon": None, "alt": None, "fix": False}
            _q_put_latest(queues["gps_comm"], pkt)
            _q_put_latest(queues["gps_ai"], pkt)
            stop_event.wait(1.0)
        return

    try:
        last_publish = 0.0
        while not stop_event.is_set():
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue

            line_no_ck = line.split("*", 1)[0]
            if not (line_no_ck.startswith("$GPGGA") or line_no_ck.startswith("$GNGGA")):
                continue

            parts = line_no_ck.split(",")
            if len(parts) < 10:
                continue

            fix_quality = parts[6].strip() if parts[6] is not None else ""
            has_fix = fix_quality not in ("", "0")
            now = time.time()

            if not has_fix:
                if now - last_publish >= 1.0:
                    pkt = {"timestamp": now, "lat": None, "lon": None, "alt": None, "fix": False}
                    _q_put_latest(queues["gps_comm"], pkt)
                    _q_put_latest(queues["gps_ai"], pkt)
                    last_publish = now
                continue

            lat = _nmea_degmin_to_decimal(parts[2].strip(), parts[3].strip())
            lon = _nmea_degmin_to_decimal(parts[4].strip(), parts[5].strip())

            alt = None
            try:
                alt = float(parts[9]) if parts[9] else None
            except ValueError:
                alt = None

            if lat is None or lon is None:
                if now - last_publish >= 1.0:
                    pkt = {"timestamp": now, "lat": None, "lon": None, "alt": alt, "fix": False}
                    _q_put_latest(queues["gps_comm"], pkt)
                    _q_put_latest(queues["gps_ai"], pkt)
                    last_publish = now
                continue

            gps_data = {
                "timestamp": now,
                "lat": float(lat),
                "lon": float(lon),
                "alt": alt,
                "fix": True,
            }

            _q_put_latest(queues["gps_comm"], gps_data)
            _q_put_latest(queues["gps_ai"], gps_data)
            last_publish = now

    finally:
        try:
            ser.close()
        except Exception:
            pass
        print("[GPS] GPS encerrado")
