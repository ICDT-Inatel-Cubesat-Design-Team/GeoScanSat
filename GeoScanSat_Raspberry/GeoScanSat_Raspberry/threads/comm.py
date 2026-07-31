import os
import time
import json
import serial
from serial import SerialException
import struct
import RPi.GPIO as GPIO
from queue import Empty, Full
from threading import Event as ThEvent
from typing import Optional, Dict, Any

PORT = os.getenv("COMM_PORT", "/dev/ttyAMA2")
BAUD = int(os.getenv("COMM_BAUD", "115200"))
GPIO_PIN = int(os.getenv("COMM_GPIO_PIN", "17"))  # BCM

TIMEOUT_AWK_ACK = float(os.getenv("COMM_TIMEOUT_AWK_ACK", "2.0"))
TIMEOUT_P = float(os.getenv("COMM_TIMEOUT_P", "2.0"))

# tempo máximo sem tráfego serial antes de permitir "resync" se GPIO pedir sessão
SESSION_STALL_S = float(os.getenv("COMM_SESSION_STALL_S", "1.5"))

# IDLE poll (baixo consumo): cobre casos onde ESP faz pulso curto e o event detect falha
IDLE_POLL_S = float(os.getenv("COMM_IDLE_POLL_S", "0.02"))  # 20 ms

# Se GPIO está LOW e não veio ACK(AWK), reenvia AWK a cada X s
AWK_RETRY_S = float(os.getenv("COMM_AWK_RETRY_S", "0.5"))

DEBUG = os.getenv("COMM_DEBUG", "0") == "1"

# sensor_frame_t (igual ao espelho.py)
SENSOR_FRAME_FMT  = "<I" + "f" * 13
SENSOR_FRAME_SIZE = struct.calcsize(SENSOR_FRAME_FMT)

SENSOR_FIELDS = [
    "ts_ms",
    "accel_m_s2_x", "accel_m_s2_y", "accel_m_s2_z",
    "gyro_rad_s_x", "gyro_rad_s_y", "gyro_rad_s_z",
    "mag_uT_x", "mag_uT_y", "mag_uT_z",
    "bmp_temp_c",
    "pressure_pa",
    "altitude_m",
    "battery_v",
]


def decode_sensor_frame_hex(hex_str: str) -> Dict[str, Any]:
    try:
        b = bytes.fromhex(hex_str)
    except ValueError:
        return {"error": "invalid_hex"}

    if len(b) < SENSOR_FRAME_SIZE:
        return {"error": "too_short", "len": len(b), "need": SENSOR_FRAME_SIZE}

    vals = struct.unpack(SENSOR_FRAME_FMT, b[:SENSOR_FRAME_SIZE])
    return {name: vals[i] for i, name in enumerate(SENSOR_FIELDS)}


def read_line(ser: serial.Serial) -> Optional[str]:
    raw = ser.readline()
    if not raw:
        return None
    return raw.decode("utf-8", errors="replace").replace("\r", "").strip() or None


def send_line(ser: serial.Serial, msg: str) -> None:
    ser.write((msg + "\n").encode("utf-8"))
    ser.flush()


def q_put_drop_oldest(q, item) -> None:
    try:
        q.put_nowait(item)
        return
    except Full:
        try:
            _ = q.get_nowait()
        except Exception:
            pass
        try:
            q.put_nowait(item)
        except Exception:
            pass
    except Exception:
        pass


def q_put_latest(q, item) -> None:
    try:
        while True:
            q.get_nowait()
    except Exception:
        pass
    try:
        q.put_nowait(item)
    except Exception:
        pass


def drain_latest(q, last: Optional[dict]) -> Optional[dict]:
    latest = last
    while True:
        try:
            latest = q.get_nowait()
        except Empty:
            break
        except Exception:
            break
    return latest


class State:
    IDLE = 0
    WAIT_ACK_AWK = 1
    SESSION = 2
    WAIT_ACK_P_START = 3
    WAIT_ACK_P_DATA = 4
    WAIT_ACK_P_END = 5


def make_payload_packet(last_gps: Optional[dict], last_ai: Optional[dict]) -> dict:
    lat = None
    lon = None

    if isinstance(last_gps, dict) and last_gps.get("fix") is True:
        g_lat = last_gps.get("lat")
        g_lon = last_gps.get("lon")
        if isinstance(g_lat, (int, float)) and isinstance(g_lon, (int, float)):
            lat = float(g_lat)
            lon = float(g_lon)

    found = False
    if isinstance(last_ai, dict) and "found" in last_ai:
        found = bool(last_ai["found"])

    return {"type": "payload", "lat": lat, "lon": lon, "found": found}


def conn_worker(queues: Dict, stop_event) -> None:
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GPIO_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    session_evt = ThEvent()

    def _on_falling(_pin):
        session_evt.set()

    # tenta event detect (baixo consumo)
    try:
        GPIO.add_event_detect(GPIO_PIN, GPIO.FALLING, callback=_on_falling, bouncetime=2)
    except Exception as e:
        if DEBUG:
            print(f"[COMM] add_event_detect falhou: {e} (fallback: poll leve)")

    ser = serial.Serial(
        port=PORT,
        baudrate=BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.2,          # menos wakeups
        write_timeout=1.0,
        exclusive=True          # impede outra thread/processo de abrir a mesma UART
    )
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    st = State.IDLE
    current_mode = None
    deadline = 0.0

    last_gps = None
    last_ai = None

    last_serial_activity = time.time()
    last_awk_sent = 0.0

    if DEBUG:
        lvl = GPIO.input(GPIO_PIN)
        print(f"[COMM] init | GPIO{GPIO_PIN}={'LOW' if lvl == GPIO.LOW else 'HIGH'} | {PORT}@{BAUD}")

    # se já está LOW no boot do programa, não dependa de borda
    if GPIO.input(GPIO_PIN) == GPIO.LOW:
        session_evt.set()

    def _force_resync(reason: str):
        nonlocal st, current_mode, deadline, last_awk_sent
        if DEBUG:
            print(f"[COMM] RESYNC: {reason}")
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass
        st = State.IDLE
        current_mode = None
        deadline = 0.0
        last_awk_sent = 0.0
        session_evt.set()

    try:
        while not stop_event.is_set():
            now = time.time()

            # caches (latest-only)
            last_gps = drain_latest(queues["gps_comm"], last_gps)
            last_ai = drain_latest(queues["ai_latest"], last_ai)

            gpio_level = GPIO.input(GPIO_PIN)

            # Se ESP resetou no meio (GPIO LOW pedindo sessão) e estamos "parados" -> resync
            if st != State.IDLE and gpio_level == GPIO.LOW:
                if (now - last_serial_activity) > SESSION_STALL_S:
                    _force_resync("GPIO LOW durante estado != IDLE e serial stall")
                    continue

            # ---------------- IDLE: inicia sessão quando GPIO pedir ----------------
            if st == State.IDLE:
                # 1) event
                fired = session_evt.wait(timeout=IDLE_POLL_S)
                if fired:
                    session_evt.clear()

                # 2) fallback por nível (pulso curto / edge perdido)
                if not fired and gpio_level != GPIO.LOW:
                    continue  # nada a fazer

                # evita flood de AWK se linha ficar LOW e ESP não responder
                if (now - last_awk_sent) < AWK_RETRY_S:
                    continue

                send_line(ser, "AWK")
                last_awk_sent = now
                st = State.WAIT_ACK_AWK
                deadline = now + TIMEOUT_AWK_ACK
                current_mode = None

                if DEBUG:
                    print("[COMM] IDLE -> TX='AWK' (start)")
                # segue para ler ACK no mesmo ciclo

            # ---------------- serial read ----------------
            line = read_line(ser)
            if line is None:
                # timeouts
                now = time.time()
                if st == State.WAIT_ACK_AWK and now > deadline:
                    # se GPIO ainda LOW, vamos tentar de novo (AWK_RETRY controla)
                    if DEBUG:
                        print("[COMM] timeout ACK(AWK)")
                    st = State.IDLE
                    current_mode = None
                    continue

                if st in (State.WAIT_ACK_P_START, State.WAIT_ACK_P_DATA, State.WAIT_ACK_P_END) and now > deadline:
                    if DEBUG:
                        print("[COMM] timeout payload -> SESSION")
                    st = State.SESSION
                    current_mode = None
                continue

            last_serial_activity = time.time()

            # ---------------- WAIT ACK AWK ----------------
            if st == State.WAIT_ACK_AWK:
                if line == "ACK":
                    if DEBUG:
                        print("[COMM] RX='ACK'(AWK) -> SESSION")
                    st = State.SESSION
                    current_mode = None
                else:
                    # lixo/desync: se chegou algo diferente de ACK, tenta resync
                    if DEBUG:
                        print(f"[COMM] esperado ACK(AWK), veio '{line}' -> resync")
                    _force_resync("ACK(AWK) mismatch")
                continue

            # ---------------- SESSION ----------------
            if st == State.SESSION:
                if line == "+T":
                    send_line(ser, "ACK")
                    current_mode = "T"
                    continue

                if line == "-T":
                    send_line(ser, "ACK")
                    current_mode = None
                    continue

                if line == "+M":
                    send_line(ser, "ACK")
                    current_mode = "M"
                    continue

                if line == "-M":
                    send_line(ser, "ACK")
                    current_mode = None
                    continue

                if line == "PAY":
                    send_line(ser, "ACK")
                    send_line(ser, "+P")
                    st = State.WAIT_ACK_P_START
                    deadline = time.time() + TIMEOUT_P
                    continue

                if line == "X":
                    send_line(ser, "ACK")
                    st = State.IDLE
                    current_mode = None
                    continue

                # JSON dentro do modo atual
                if current_mode is not None and line.startswith("{"):
                    try:
                        obj = json.loads(line)
                    except json.JSONDecodeError:
                        send_line(ser, "ACK")
                        continue

                    rx_time_s = time.time()

                    if current_mode == "T":
                        hex_str = obj.get("hex")
                        if isinstance(hex_str, str):
                            decoded = decode_sensor_frame_hex(hex_str)

                            row = {
                                "rx_time_s": rx_time_s,
                                "src": obj.get("src"),
                                "seq": obj.get("seq"),
                                "ts_ms_json": obj.get("ts_ms"),
                                "ts_ms_frame": decoded.get("ts_ms") if isinstance(decoded, dict) else None,
                            }

                            if isinstance(decoded, dict):
                                for k, v in decoded.items():
                                    if k == "ts_ms":
                                        continue
                                    row[k] = v
                                if "error" in decoded:
                                    row["decode_error"] = decoded["error"]

                            q_put_drop_oldest(queues["telemetry"], row)
                            q_put_latest(queues["telemetry_ai"], row)
                            q_put_latest(queues["telemetry_http"], row)
                        else:
                            row = {
                                "rx_time_s": rx_time_s,
                                "src": obj.get("src"),
                                "seq": obj.get("seq"),
                                "ts_ms_json": obj.get("ts_ms"),
                                "decode_error": "missing_hex",
                            }
                            q_put_drop_oldest(queues["telemetry"], row)
                            q_put_latest(queues["telemetry_ai"], row)
                            q_put_latest(queues["telemetry_http"], row)

                    elif current_mode == "M":
                        # Dados de módulos externos recebidos pelo LoRa no ESP32
                        row = dict(obj)
                        row["rx_time_s"] = rx_time_s

                        print(
                            "[LORA RX] "
                            + json.dumps(row, ensure_ascii=False, separators=(",", ":"))
                        )

                        # Filas já existentes no projeto
                        if "buoy" in queues:
                            q_put_drop_oldest(queues["buoy"], row)
                        if "buoy_http" in queues:
                            q_put_latest(queues["buoy_http"], row)

                        # Fila opcional específica para LoRa
                        if "lora_rx" in queues:
                            q_put_drop_oldest(queues["lora_rx"], row)

                    send_line(ser, "ACK")
                else:
                    # linha desconhecida em SESSION -> ACK opcional ou ignore.
                    # Para robustez: ACK se for algo curto (evita travar ESP esperando ACK).
                    if len(line) <= 8:
                        send_line(ser, "ACK")
                continue

            # ---------------- PAYLOAD EXCHANGE ----------------
            if st == State.WAIT_ACK_P_START:
                if line == "ACK":
                    payload = make_payload_packet(last_gps, last_ai)
                    pkt = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
                    send_line(ser, pkt)
                    st = State.WAIT_ACK_P_DATA
                    deadline = time.time() + TIMEOUT_P
                continue

            if st == State.WAIT_ACK_P_DATA:
                if line == "ACK":
                    send_line(ser, "-P")
                    st = State.WAIT_ACK_P_END
                    deadline = time.time() + TIMEOUT_P
                continue

            if st == State.WAIT_ACK_P_END:
                if line == "ACK":
                    st = State.SESSION
                    current_mode = None
                continue

    except SerialException as error:
        print(f"[COMM] Erro na porta serial {PORT}: {error}")
        print(
            "[COMM] Verifique se outra thread, minicom ou outro processo "
            "está usando a mesma porta."
        )
        stop_event.set()

    except Exception as error:
        print(f"[COMM] Erro inesperado: {error}")
        stop_event.set()

    finally:
        try:
            GPIO.remove_event_detect(GPIO_PIN)
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass
        GPIO.cleanup()
        if DEBUG:
            print("[COMM] finalizado")