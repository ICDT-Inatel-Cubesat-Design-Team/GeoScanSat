import os
import time
from datetime import datetime
from threading import Event
from queue import Full

import cv2
from ultralytics import YOLO


AI_PERIOD_S = int(os.getenv("AI_PERIOD_S", "240"))

MODEL_PATH = os.getenv(
    "AI_MODEL_PATH",
    "/home/ic/Desktop/GeoScanSat_Raspberry/models/mining.pt"
)

CAMERA_ID = int(os.getenv("AI_CAMERA_ID", "0"))
CONFIDENCE = float(os.getenv("AI_CONFIDENCE", "0.5"))

IMAGE_DIRECTORY = os.getenv(
    "AI_IMAGE_DIRECTORY",
    "/home/ic/Desktop/GeoScanSat_Raspberry/images_ai"
)


def put_latest(queue, item):
    """
    Remove os itens antigos e mantém somente
    o item mais recente na fila.
    """
    try:
        while True:
            queue.get_nowait()
    except Exception:
        pass

    try:
        queue.put_nowait(item)
    except Exception:
        pass


def create_image_directory():
    """
    Cria a pasta de imagens caso ela ainda não exista.
    """
    os.makedirs(
        IMAGE_DIRECTORY,
        exist_ok=True
    )

    print(
        f"[AI] Pasta das imagens: "
        f"{IMAGE_DIRECTORY}"
    )


def save_images(frame, results):
    """
    Salva a imagem original e a imagem anotada
    pelo modelo YOLO.
    """
    timestamp = datetime.now().strftime(
        "%Y-%m-%d_%H-%M-%S"
    )

    original_path = os.path.join(
        IMAGE_DIRECTORY,
        f"original_{timestamp}.jpg"
    )

    detected_path = os.path.join(
        IMAGE_DIRECTORY,
        f"detected_{timestamp}.jpg"
    )

    original_saved = cv2.imwrite(
        original_path,
        frame
    )

    if not original_saved:
        raise RuntimeError(
            f"Não foi possível salvar: {original_path}"
        )

    annotated_frame = results[0].plot()

    detected_saved = cv2.imwrite(
        detected_path,
        annotated_frame
    )

    if not detected_saved:
        raise RuntimeError(
            f"Não foi possível salvar: {detected_path}"
        )

    print(
        f"[AI] Imagem original salva: "
        f"{original_path}"
    )

    print(
        f"[AI] Imagem detectada salva: "
        f"{detected_path}"
    )

    return {
        "original_image": original_path,
        "detected_image": detected_path
    }


def capture_and_detect(model):
    """
    Abre a câmera, captura um frame, executa
    a detecção e salva as imagens.
    """
    camera = cv2.VideoCapture(CAMERA_ID)

    if not camera.isOpened():
        camera.release()

        print("[AI] Não foi possível abrir a câmera")

        return {
            "found": False,
            "detections": 0,
            "original_image": None,
            "detected_image": None,
            "error": "CAMERA_OPEN_ERROR"
        }

    try:
        frame = None

        # Descarta os primeiros frames para
        # permitir que a câmera estabilize.
        for _ in range(5):
            success, current_frame = camera.read()

            if success:
                frame = current_frame

            time.sleep(0.05)

        if frame is None:
            print("[AI] Não foi possível capturar a imagem")

            return {
                "found": False,
                "detections": 0,
                "original_image": None,
                "detected_image": None,
                "error": "CAMERA_CAPTURE_ERROR"
            }

        results = model.predict(
            source=frame,
            conf=CONFIDENCE,
            verbose=False
        )

        detections = 0

        for result in results:
            if result.boxes is not None:
                detections += len(result.boxes)

        found = detections > 0

        saved_images = save_images(
            frame,
            results
        )

        print(
            f"[AI] found={found} | "
            f"detections={detections}"
        )

        return {
            "found": found,
            "detections": detections,
            "original_image": saved_images["original_image"],
            "detected_image": saved_images["detected_image"],
            "error": None
        }

    except Exception as error:
        print(
            f"[AI] Erro durante captura/detecção: "
            f"{error}"
        )

        return {
            "found": False,
            "detections": 0,
            "original_image": None,
            "detected_image": None,
            "error": str(error)
        }

    finally:
        camera.release()


def publish_payload(queues, payload):
    """
    Publica o resultado nas filas do sistema.
    """
    try:
        queues["payload"].put_nowait(payload)

    except Full:
        try:
            queues["payload"].get_nowait()
        except Exception:
            pass

        try:
            queues["payload"].put_nowait(payload)
        except Exception:
            pass

    except Exception:
        pass

    put_latest(
        queues["payload_http"],
        payload
    )

    put_latest(
        queues["ai_latest"],
        {
            "timestamp": payload["timestamp"],
            "found": payload["found"],
            "detections": payload["detections"],
            "original_image": payload["original_image"],
            "detected_image": payload["detected_image"],
            "error": payload["error"]
        }
    )


def ai_worker(queues, stop_event: Event):
    """
    Worker responsável pela captura das imagens
    e pela execução do modelo de IA.
    """
    print(
        f"[AI] Thread iniciada | "
        f"período={AI_PERIOD_S}s"
    )

    print(f"[AI] Modelo: {MODEL_PATH}")
    print(f"[AI] Câmera: {CAMERA_ID}")
    print(f"[AI] Confiança: {CONFIDENCE}")

    create_image_directory()

    if not os.path.isfile(MODEL_PATH):
        print(
            "[AI] Arquivo do modelo não encontrado: "
            f"{MODEL_PATH}"
        )
        return

    try:
        model = YOLO(MODEL_PATH)

        print("[AI] Modelo carregado com sucesso")

    except Exception as error:
        print(
            "[AI] Erro ao carregar modelo: "
            f"{error}"
        )
        return

    while not stop_event.is_set():
        capture_time = time.time()

        result = capture_and_detect(model)

        payload = {
            "timestamp": capture_time,
            "source": "AI",
            "decision": (
                "DETECTED"
                if result["found"]
                else "NORMAL"
            ),
            "lat": None,
            "lon": None,
            "found": result["found"],
            "detections": result["detections"],
            "original_image": result["original_image"],
            "detected_image": result["detected_image"],
            "error": result["error"]
        }

        publish_payload(
            queues,
            payload
        )

        print(
            f"[AI] Payload publicado: {payload}"
        )

        stop_event.wait(
            AI_PERIOD_S
        )

    print("[AI] Thread finalizada")