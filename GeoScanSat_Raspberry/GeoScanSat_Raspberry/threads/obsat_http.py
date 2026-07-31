import os
import time
import json
import urllib.request
import urllib.error
from queue import Empty
from threading import Event
from typing import Dict, Optional, Any, Tuple

#OBSAT_URL = os.getenv("OBSAT_URL", "https://obsat.org.br/teste_post/envio.php")
OBSAT_URL = os.getenv("OBSAT_URL", "https://obsat.org.br/servidor_testes/envio.php")
OBSAT_EQUIPE = int(os.getenv("OBSAT_TEAM", "60"))
SEND_PERIOD_S = int(os.getenv("OBSAT_PERIOD_S", "240"))  # 4 min
HTTP_TIMEOUT_S = float(os.getenv("OBSAT_HTTP_TIMEOUT_S", "6.0"))
HTTP_OFFSET_S = float(os.getenv("OBSAT_OFFSET_S", "2.0"))  # dá tempo da IA gerar payload antes do POST

PAYLOAD_MAX = 90  # bytes

OBSAT_E_ARGS = -1
OBSAT_E_NO_TELEM = -2
OBSAT_E_JSON = -3
OBSAT_E_HTTP = -4


def _round_i32(x: float) -> int:
    return int(round(x))


def _clamp_i16(x: int) -> int:
    if x < -32768:
        return -32768
    if x > 32767:
        return 32767
    return x


def _clamp_u16(x: int) -> int:
    if x < 0:
        return 0
    if x > 65535:
        return 65535
    return x


def _clamp_u8(x: int) -> int:
    if x < 0:
        return 0
    if x > 255:
        return 255
    return x


def _pack_lat_1e5(lat: Optional[float]) -> int:
    if not isinstance(lat, (int, float)):
        return 0
    if lat < -90.0:
        lat = -90.0
    if lat > 90.0:
        lat = 90.0
    return _round_i32(lat * 1e5)


def _pack_lon_1e5(lon: Optional[float]) -> int:
    if not isinstance(lon, (int, float)):
        return 0
    if lon < -180.0:
        lon = -180.0
    if lon > 180.0:
        lon = 180.0
    return _round_i32(lon * 1e5)


def _get_latest_now(q) -> Optional[dict]:
    last = None
    try:
        while True:
            last = q.get_nowait()
    except Empty:
        return last
    except Exception:
        return last


def _payload_json_str(vals) -> str:
    return json.dumps(vals, ensure_ascii=False, separators=(",", ":"))


def build_payload_string_like_c(
    s: Dict[str, Any],
    p: Optional[Dict[str, Any]],    # payload gerado pela IA (lat/lon/found do instante da foto)
    m: Optional[Dict[str, Any]],    # boia
) -> Tuple[int, str]:
    ts_ms = s.get("ts_ms_frame")
    if not isinstance(ts_ms, (int, float)):
        ts_ms = s.get("ts_ms_json")
    if not isinstance(ts_ms, (int, float)):
        ts_ms = 0.0

    ts_s = int(ts_ms // 1000)

    alt_m = s.get("altitude_m")
    if not isinstance(alt_m, (int, float)):
        alt_m = 0.0
    alt_dm = _round_i32(float(alt_m) * 10.0)

    mx = s.get("mag_uT_x"); my = s.get("mag_uT_y"); mz = s.get("mag_uT_z")
    mx_duT = _clamp_i16(_round_i32(float(mx) * 10.0)) if isinstance(mx, (int, float)) else 0
    my_duT = _clamp_i16(_round_i32(float(my) * 10.0)) if isinstance(my, (int, float)) else 0
    mz_duT = _clamp_i16(_round_i32(float(mz) * 10.0)) if isinstance(mz, (int, float)) else 0

    p_lat = p.get("lat") if isinstance(p, dict) else None
    p_lon = p.get("lon") if isinstance(p, dict) else None
    plat_1e5 = _pack_lat_1e5(p_lat)
    plon_1e5 = _pack_lon_1e5(p_lon)

    found = False
    if isinstance(p, dict) and "found" in p:
        found = bool(p["found"])
    pfound = 1 if found else 0

    m_lat = None
    m_lon = None
    ms = 0

    if isinstance(m, dict):
        m_lat = m.get("lat", m.get("latitude"))
        m_lon = m.get("lon", m.get("longitude"))

        if isinstance(m.get("s"), (int, float)):
            ms = int(m["s"])
        elif isinstance(m.get("status"), (int, float)):
            ms = int(m["status"])
        ms = _clamp_u8(ms)

    mlat_1e5 = _pack_lat_1e5(m_lat)
    mlon_1e5 = _pack_lon_1e5(m_lon)

    full = [ts_s, alt_dm, mx_duT, my_duT, mz_duT, plat_1e5, plon_1e5, pfound, mlat_1e5, mlon_1e5, ms]
    mid  = [ts_s, alt_dm, mx_duT, my_duT, mz_duT, plat_1e5, plon_1e5, pfound]
    mini = [ts_s, plat_1e5, plon_1e5, pfound]
    fb   = [ts_s, pfound]

    for code, arr in ((1, full), (2, mid), (3, mini), (4, fb)):
        ps = _payload_json_str(arr)
        if len(ps.encode("utf-8")) <= PAYLOAD_MAX:
            return code, ps

    return 0, "[]"


def obsat_http_format_json_like_c(
    cfg_equipe: int,
    s: Dict[str, Any],
    p: Optional[Dict[str, Any]],
    m: Optional[Dict[str, Any]],
) -> Tuple[int, str]:
    if not isinstance(cfg_equipe, int) or cfg_equipe <= 0:
        return (OBSAT_E_ARGS, "")

    if not isinstance(s, dict):
        return (OBSAT_E_NO_TELEM, "")

    _, payload_str = build_payload_string_like_c(s, p, m)

    batt_v = s.get("battery_v")
    bateria_mv = _clamp_u16(_round_i32(float(batt_v) * 1000.0)) if isinstance(batt_v, (int, float)) else 0
    #batt_v = 3.80
    #bateria_mv = _clamp_u16(_round_i32(batt_v * 1000.0))

    t_c = s.get("bmp_temp_c")
    temp_mC = _clamp_u16(_round_i32(float(t_c) * 1000.0)) if isinstance(t_c, (int, float)) else 0

    p_pa = s.get("pressure_pa")
    press_hpa = _clamp_u16(_round_i32(float(p_pa) / 100.0)) if isinstance(p_pa, (int, float)) else 0

    gx = s.get("gyro_rad_s_x"); gy = s.get("gyro_rad_s_y"); gz = s.get("gyro_rad_s_z")
    gx_mrad = _clamp_i16(_round_i32(float(gx) * 1000.0)) if isinstance(gx, (int, float)) else 0
    gy_mrad = _clamp_i16(_round_i32(float(gy) * 1000.0)) if isinstance(gy, (int, float)) else 0
    gz_mrad = _clamp_i16(_round_i32(float(gz) * 1000.0)) if isinstance(gz, (int, float)) else 0

    ax = s.get("accel_m_s2_x"); ay = s.get("accel_m_s2_y"); az = s.get("accel_m_s2_z")
    ax_mms2 = _clamp_i16(_round_i32(float(ax) * 1000.0)) if isinstance(ax, (int, float)) else 0
    ay_mms2 = _clamp_i16(_round_i32(float(ay) * 1000.0)) if isinstance(ay, (int, float)) else 0
    az_mms2 = _clamp_i16(_round_i32(float(az) * 1000.0)) if isinstance(az, (int, float)) else 0

    try:
        base = {
            "equipe": int(cfg_equipe),
            "bateria": int(bateria_mv),
            "temperatura": int(temp_mC),
            "pressao": int(press_hpa),
            "giroscopio": f"{gx_mrad},{gy_mrad},{gz_mrad}",
            "acelerometro": f"{ax_mms2},{ay_mms2},{az_mms2}",
        }
        base_str = json.dumps(base, ensure_ascii=False, separators=(",", ":"))
        json_str = base_str[:-1] + f',"payload":{payload_str}' + "}"
    except Exception:
        return (OBSAT_E_JSON, "")

    n = len(json_str.encode("utf-8"))
    return (n, json_str)


def obsat_http_post_json(json_str: str) -> int:
    try:
        data = json_str.encode("utf-8")
        req = urllib.request.Request(
            OBSAT_URL,
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT_S) as resp:
            return int(resp.getcode())
    except urllib.error.HTTPError as e:
        try:
            return int(e.code)
        except Exception:
            return OBSAT_E_HTTP
    except Exception:
        return OBSAT_E_HTTP


def obsat_http_worker(queues: Dict, stop_event: Event) -> None:
    print(f"[HTTP] iniciado | equipe={OBSAT_EQUIPE} | url={OBSAT_URL} | period={SEND_PERIOD_S}s")

    # caches (se no ciclo não chegar nada novo, envia o último conhecido)
    last_telem = None
    last_buoy = None
    last_payload = None

    # primeiro envio: pequeno offset pra IA rodar e publicar payload_http
    next_send = time.monotonic() + max(0.0, HTTP_OFFSET_S)

    while not stop_event.is_set():
        now = time.monotonic()
        wait_s = next_send - now
        if wait_s > 0:
            stop_event.wait(timeout=wait_s)
            continue

        next_send += SEND_PERIOD_S

        t = _get_latest_now(queues["telemetry_http"])
        if isinstance(t, dict):
            last_telem = t

        b = _get_latest_now(queues["buoy_http"])
        if isinstance(b, dict):
            last_buoy = b

        p = _get_latest_now(queues["payload_http"])
        if isinstance(p, dict):
            last_payload = p

        if not isinstance(last_telem, dict):
            print("[HTTP] sem telemetria ainda; pulando envio")
            continue

        n, js = obsat_http_format_json_like_c(
            OBSAT_EQUIPE, last_telem, last_payload, last_buoy
        )
        if n < 0:
            print(f"[HTTP] erro formatando JSON: {n}")
            continue

        status = obsat_http_post_json(js)
        print(f"[HTTP] POST status={status} bytes={n}")

    print("[HTTP] finalizado")
