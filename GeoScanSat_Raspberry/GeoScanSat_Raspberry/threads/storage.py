import csv
import os
import time
from queue import Empty
from threading import Event
from typing import Dict, Any, Optional

FLUSH_PERIOD_S = 1.0
IDLE_WAIT_S = 0.2

TELEMETRY_FIELDS = [
    "rx_time_s",
    "src",
    "seq",
    "ts_ms_json",
    "ts_ms_frame",

    "accel_m_s2_x", "accel_m_s2_y", "accel_m_s2_z",
    "gyro_rad_s_x", "gyro_rad_s_y", "gyro_rad_s_z",
    "mag_uT_x", "mag_uT_y", "mag_uT_z",
    "bmp_temp_c",
    "pressure_pa",
    "altitude_m",
    "battery_v",

    "decode_error",
]

PAYLOAD_FIELDS = [
    "timestamp",
    "source",
    "decision",
    "lat",
    "lon",
    "found",
]


def _open_csv_append(path: str):
    exists = os.path.exists(path)
    f = open(path, "a", newline="", encoding="utf-8")
    is_empty = (not exists) or (os.path.getsize(path) == 0)
    return f, is_empty


def _safe_write_row(writer: csv.DictWriter, row: Dict[str, Any], *, tag: str) -> None:
    try:
        writer.writerow(row)
    except Exception as e:
        print(f"[STORAGE] erro gravando {tag}: {e} | row_type={type(row)}")


def storage_worker(queues: Dict, stop_event: Event):
    print("[STORAGE] iniciada")

    telemetry_file, telemetry_empty = _open_csv_append("telemetry.csv")
    buoy_file, buoy_empty           = _open_csv_append("buoy.csv")
    payload_file, payload_empty     = _open_csv_append("payload.csv")

    telemetry_writer: Optional[csv.DictWriter] = None
    buoy_writer: Optional[csv.DictWriter] = None
    payload_writer: Optional[csv.DictWriter] = None

    last_flush_m = time.monotonic()

    try:
        telemetry_writer = csv.DictWriter(
            telemetry_file,
            fieldnames=TELEMETRY_FIELDS,
            extrasaction="ignore"
        )
        if telemetry_empty:
            telemetry_writer.writeheader()
            telemetry_file.flush()

        payload_writer = csv.DictWriter(
            payload_file,
            fieldnames=PAYLOAD_FIELDS,
            extrasaction="ignore"
        )
        if payload_empty:
            payload_writer.writeheader()
            payload_file.flush()

        wrote_since_flush = False

        while not stop_event.is_set():
            wrote_any = False

            # -------- TELEMETRIA --------
            while True:
                try:
                    data = queues["telemetry"].get_nowait()
                except Empty:
                    break
                except Exception:
                    break

                if not isinstance(data, dict):
                    data = {"rx_time_s": time.time(), "decode_error": "non_dict_row"}

                _safe_write_row(telemetry_writer, data, tag="telemetry")
                wrote_any = True

            # -------- BOIA --------
            while True:
                try:
                    data = queues["buoy"].get_nowait()
                except Empty:
                    break
                except Exception:
                    break

                if not isinstance(data, dict):
                    data = {"rx_time_s": time.time(), "error": "non_dict_row"}

                if buoy_writer is None:
                    fieldnames = list(data.keys())
                    buoy_writer = csv.DictWriter(buoy_file, fieldnames=fieldnames, extrasaction="ignore")
                    if buoy_empty:
                        buoy_writer.writeheader()
                        buoy_file.flush()

                _safe_write_row(buoy_writer, data, tag="buoy")
                wrote_any = True

            # -------- PAYLOAD --------
            while True:
                try:
                    data = queues["payload"].get_nowait()
                except Empty:
                    break
                except Exception:
                    break

                if not isinstance(data, dict):
                    data = {
                        "timestamp": time.time(),
                        "source": "AI",
                        "decision": "PAYLOAD_NON_DICT",
                        "lat": None,
                        "lon": None,
                        "found": None
                    }

                if not isinstance(data.get("lat"), (int, float)):
                    data["lat"] = None
                if not isinstance(data.get("lon"), (int, float)):
                    data["lon"] = None

                _safe_write_row(payload_writer, data, tag="payload")
                wrote_any = True

            if wrote_any:
                wrote_since_flush = True

            # flush por janela
            now_m = time.monotonic()
            if wrote_since_flush and (now_m - last_flush_m) >= FLUSH_PERIOD_S:
                telemetry_file.flush()
                buoy_file.flush()
                payload_file.flush()
                last_flush_m = now_m
                wrote_since_flush = False

            # idle wait (sem busy-loop)
            if not wrote_any:
                stop_event.wait(IDLE_WAIT_S)
            else:
                # pequeno yield
                stop_event.wait(0.01)

    finally:
        print("[STORAGE] encerrando CSVs")
        try:
            telemetry_file.flush()
            buoy_file.flush()
            payload_file.flush()
        except Exception:
            pass
        try:
            telemetry_file.close()
        except Exception:
            pass
        try:
            buoy_file.close()
        except Exception:
            pass
        try:
            payload_file.close()
        except Exception:
            pass
        print("[STORAGE] finalizada")
