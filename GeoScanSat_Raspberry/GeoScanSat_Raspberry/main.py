import signal
import time
from threading import Event, Thread
from queues.queues import create_queues

from threads.comm import conn_worker
from threads.gps import gps_worker
from threads.AI import ai_worker
from threads.storage import storage_worker
from threads.obsat_http import obsat_http_worker


def main():
    stop_event = Event()

    def handle_sigint(sig, frame):
        print("Encerrando Sistema")
        stop_event.set()

    signal.signal(signal.SIGINT, handle_sigint)

    queues = create_queues()

    threads = [
        Thread(target=conn_worker,       name="CommThread",    args=(queues, stop_event), daemon=True),
        Thread(target=gps_worker,        name="GPSThread",     args=(queues, stop_event), daemon=True),
        Thread(target=ai_worker,         name="AIThread",      args=(queues, stop_event), daemon=True),
        Thread(target=storage_worker,    name="StorageThread", args=(queues, stop_event), daemon=True),
        Thread(target=obsat_http_worker, name="HTTPThread",    args=(queues, stop_event), daemon=True),
    ]

    print("Iniciando as Threads")
    for t in threads:
        print(f"[MAIN] -> {t.name}")
        t.start()

    try:
        while not stop_event.is_set():
            time.sleep(1)
    except KeyboardInterrupt:
        stop_event.set()

    print("Aguardando o término das Threads")
    for t in threads:
        t.join(timeout=2.0)

    print("Sistema finalizado com sucesso")


if __name__ == "__main__":
    main()
