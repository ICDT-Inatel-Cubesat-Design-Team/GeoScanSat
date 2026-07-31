import time
from threading import Event
from typing import Dict

def gps_worker(queues: Dict, stop_event: Event):

    print("Thread de GPS iniciada")

    while not stop_event.is_set():
        #Mock dos dados de GPS
        gps_data = {
            "source": "GPS",
            "timestamp": time.time(),
            "lat": 37.7749,
            "lon": -122.4194,
            "alt": 15.0,
            "fix": True
        }

        try:
            queues["telemetry"].put(gps_data, timeout=1)
            print(f"Dados de GPS enviados: {gps_data}")

        except Exception as e:
            print(f"Erro ao enviar dados de GPS: {e}")

        #Frequência do GPS
        time.sleep(1)

    print("Thread de GPS encerrada")