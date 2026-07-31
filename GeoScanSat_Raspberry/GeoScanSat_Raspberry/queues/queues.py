from queue import Queue
from typing import Dict


def create_queues() -> Dict[str, Queue]:
    return {
        "telemetry": Queue(maxsize=8192),
        "buoy":      Queue(maxsize=2048),
        "payload":   Queue(maxsize=2048),

        "telemetry_ai": Queue(maxsize=1),

        "gps_ai":   Queue(maxsize=1),
        "gps_comm": Queue(maxsize=1),

        "ai_latest": Queue(maxsize=1),

        "telemetry_http": Queue(maxsize=1),
        "buoy_http":      Queue(maxsize=1),

        "payload_http":   Queue(maxsize=1),
    }
