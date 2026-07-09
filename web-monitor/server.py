"""BioScope local HTTP server and CSV task store.

The browser connects directly to the ESP32 WebSocket. During an active task it
posts sample batches to this local service, which persists them to data/*.csv.
No third-party Python packages are required.
"""

from __future__ import annotations

import csv
import json
import mimetypes
import re
import threading
import uuid
from datetime import datetime, timezone
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, quote, urlparse


ROOT = Path(__file__).resolve().parent
DATA_DIR = ROOT / "data"
DATA_DIR.mkdir(exist_ok=True)
STORE_LOCK = threading.Lock()
TASK_ID_RE = re.compile(r"^[0-9A-Za-z_-]+$")
CSV_HEADER = [
    "sample_index",
    "timestamp_us",
    "ecg_mv",
    "emg1_mv",
    "emg2_mv",
    "emg3_mv",
    "gyro_x_dps",
    "gyro_y_dps",
    "gyro_z_dps",
    "fall",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "temperature_c",
    "alarm_flags",
]


def iso_now() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def safe_task_id(value: str) -> str:
    if not TASK_ID_RE.fullmatch(value):
        raise ValueError("invalid task id")
    return value


def meta_path(task_id: str) -> Path:
    return DATA_DIR / f"{safe_task_id(task_id)}.json"


def csv_path(task_id: str) -> Path:
    return DATA_DIR / f"{safe_task_id(task_id)}.csv"


def load_meta(task_id: str) -> dict:
    with meta_path(task_id).open("r", encoding="utf-8") as file:
        return json.load(file)


def save_meta(meta: dict) -> None:
    target = meta_path(meta["id"])
    temporary = target.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    temporary.replace(target)


def public_meta(meta: dict) -> dict:
    return {
        key: meta.get(key)
        for key in (
            "id",
            "name",
            "note",
            "status",
            "createdAt",
            "startedAt",
            "endedAt",
            "sampleRate",
            "sampleCount",
            "alarmCount",
            "durationSeconds",
        )
    }


class BioScopeHandler(SimpleHTTPRequestHandler):
    server_version = "BioScope/0.1"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def log_message(self, fmt: str, *args) -> None:
        print(f"[BioScope] {self.address_string()} - {fmt % args}")

    def send_json(self, payload: object, status: int = HTTPStatus.OK) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def read_json(self) -> dict:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0 or length > 4 * 1024 * 1024:
            raise ValueError("invalid request body size")
        raw = self.rfile.read(length)
        payload = json.loads(raw.decode("utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("JSON object required")
        return payload

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        try:
            if path == "/api/health":
                self.send_json({"ok": True, "time": iso_now()})
                return
            if path == "/api/tasks":
                self.list_tasks()
                return
            match = re.fullmatch(r"/api/tasks/([^/]+)/data", path)
            if match:
                max_points = int(parse_qs(parsed.query).get("max", ["4000"])[0])
                self.task_data(match.group(1), max(100, min(max_points, 20000)))
                return
            match = re.fullmatch(r"/api/tasks/([^/]+)/csv", path)
            if match:
                self.download_csv(match.group(1))
                return
            super().do_GET()
        except (ValueError, FileNotFoundError, json.JSONDecodeError) as error:
            self.send_json({"error": str(error)}, HTTPStatus.NOT_FOUND)
        except Exception as error:  # Keep API errors structured for the UI.
            self.send_json({"error": str(error)}, HTTPStatus.INTERNAL_SERVER_ERROR)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        try:
            if path == "/api/tasks":
                self.create_task()
                return
            match = re.fullmatch(r"/api/tasks/([^/]+)/samples", path)
            if match:
                self.append_samples(match.group(1))
                return
            match = re.fullmatch(r"/api/tasks/([^/]+)/stop", path)
            if match:
                self.stop_task(match.group(1))
                return
            self.send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
        except (ValueError, json.JSONDecodeError) as error:
            self.send_json({"error": str(error)}, HTTPStatus.BAD_REQUEST)
        except FileNotFoundError as error:
            self.send_json({"error": str(error)}, HTTPStatus.NOT_FOUND)
        except Exception as error:
            self.send_json({"error": str(error)}, HTTPStatus.INTERNAL_SERVER_ERROR)

    def do_DELETE(self) -> None:
        path = urlparse(self.path).path
        match = re.fullmatch(r"/api/tasks/([^/]+)", path)
        if not match:
            self.send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            return
        try:
            task_id = safe_task_id(match.group(1))
            with STORE_LOCK:
                meta = load_meta(task_id)
                if meta.get("status") == "recording":
                    raise ValueError("正在采集的任务不能删除")
                meta_path(task_id).unlink(missing_ok=True)
                csv_path(task_id).unlink(missing_ok=True)
            self.send_json({"ok": True})
        except ValueError as error:
            self.send_json({"error": str(error)}, HTTPStatus.CONFLICT)
        except FileNotFoundError:
            self.send_json({"error": "task not found"}, HTTPStatus.NOT_FOUND)

    def create_task(self) -> None:
        payload = self.read_json()
        name = str(payload.get("name", "")).strip()
        note = str(payload.get("note", "")).strip()
        if not name:
            raise ValueError("任务名称不能为空")
        if len(name) > 80 or len(note) > 500:
            raise ValueError("任务名称或备注过长")

        now = datetime.now()
        task_id = f"{now:%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:6]}"
        meta = {
            "id": task_id,
            "name": name,
            "note": note,
            "status": "recording",
            "createdAt": iso_now(),
            "startedAt": iso_now(),
            "endedAt": None,
            "sampleRate": int(payload.get("sampleRate", 500)),
            "sampleCount": 0,
            "alarmCount": 0,
            "durationSeconds": 0,
        }
        with STORE_LOCK:
            with csv_path(task_id).open("w", newline="", encoding="utf-8-sig") as file:
                csv.writer(file).writerow(CSV_HEADER)
            save_meta(meta)
        self.send_json(public_meta(meta), HTTPStatus.CREATED)

    def append_samples(self, task_id: str) -> None:
        payload = self.read_json()
        samples = payload.get("samples")
        if not isinstance(samples, list) or len(samples) > 5000:
            raise ValueError("samples must be an array with at most 5000 rows")

        rows = []
        alarm_rows = 0
        with STORE_LOCK:
            meta = load_meta(task_id)
            if meta.get("status") != "recording":
                raise ValueError("task is not recording")
            start_index = int(meta.get("sampleCount", 0))
            for offset, sample in enumerate(samples):
                if not isinstance(sample, list) or len(sample) < 6:
                    raise ValueError("sample row must contain timestamp, four channels and flags")
                if len(sample) >= 14:
                    (
                        timestamp,
                        ecg,
                        emg1,
                        emg2,
                        emg3,
                        gyro_x,
                        gyro_y,
                        gyro_z,
                        fall,
                        roll,
                        pitch,
                        yaw,
                        temperature,
                        flags,
                    ) = sample[:14]
                elif len(sample) >= 10:
                    timestamp, ecg, emg1, emg2, emg3, gyro_x, gyro_y, gyro_z, fall, flags = sample[:10]
                    roll = pitch = yaw = temperature = 0
                else:
                    timestamp, ecg, emg1, emg2, emg3, flags = sample[:6]
                    gyro_x = gyro_y = gyro_z = 0
                    fall = 0
                    roll = pitch = yaw = temperature = 0
                flags = int(flags)
                alarm_rows += int(flags != 0)
                rows.append(
                    [
                        start_index + offset,
                        int(timestamp),
                        float(ecg),
                        float(emg1),
                        float(emg2),
                        float(emg3),
                        float(gyro_x),
                        float(gyro_y),
                        float(gyro_z),
                        int(fall),
                        float(roll),
                        float(pitch),
                        float(yaw),
                        float(temperature),
                        flags,
                    ]
                )
            with csv_path(task_id).open("a", newline="", encoding="utf-8") as file:
                csv.writer(file).writerows(rows)
            meta["sampleCount"] = start_index + len(rows)
            meta["alarmCount"] = int(meta.get("alarmCount", 0)) + alarm_rows
            save_meta(meta)
        self.send_json({"ok": True, "accepted": len(rows), "sampleCount": meta["sampleCount"]})

    def stop_task(self, task_id: str) -> None:
        with STORE_LOCK:
            meta = load_meta(task_id)
            if meta.get("status") == "recording":
                meta["status"] = "completed"
                meta["endedAt"] = iso_now()
                started = datetime.fromisoformat(meta["startedAt"])
                ended = datetime.fromisoformat(meta["endedAt"])
                meta["durationSeconds"] = max(0, round((ended - started).total_seconds(), 3))
                save_meta(meta)
        self.send_json(public_meta(meta))

    def list_tasks(self) -> None:
        tasks = []
        for path in DATA_DIR.glob("*.json"):
            try:
                tasks.append(public_meta(json.loads(path.read_text(encoding="utf-8"))))
            except (OSError, json.JSONDecodeError):
                continue
        tasks.sort(key=lambda item: item.get("createdAt") or "", reverse=True)
        self.send_json({"tasks": tasks})

    def task_data(self, task_id: str, max_points: int) -> None:
        meta = load_meta(task_id)
        total = int(meta.get("sampleCount", 0))
        stride = max(1, (total + max_points - 1) // max_points)
        samples = []
        with csv_path(task_id).open("r", newline="", encoding="utf-8-sig") as file:
            for row_index, row in enumerate(csv.DictReader(file)):
                if row_index % stride != 0 and row_index != total - 1:
                    continue
                samples.append(
                    [
                        int(row["sample_index"]),
                        int(row["timestamp_us"]),
                        float(row["ecg_mv"]),
                        float(row["emg1_mv"]),
                        float(row["emg2_mv"]),
                        float(row["emg3_mv"]),
                        float(row.get("gyro_x_dps") or 0),
                        float(row.get("gyro_y_dps") or 0),
                        float(row.get("gyro_z_dps") or 0),
                        int(row.get("fall") or 0),
                        float(row.get("roll_deg") or 0),
                        float(row.get("pitch_deg") or 0),
                        float(row.get("yaw_deg") or 0),
                        float(row.get("temperature_c") or 0),
                        int(row["alarm_flags"]),
                    ]
                )
        self.send_json({"task": public_meta(meta), "stride": stride, "samples": samples})

    def download_csv(self, task_id: str) -> None:
        meta = load_meta(task_id)
        target = csv_path(task_id)
        body = target.read_bytes()
        safe_name = re.sub(r'[<>:"/\\|?*]+', "_", meta.get("name", task_id))
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/csv; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header(
            "Content-Disposition",
            f"attachment; filename*=UTF-8''{quote(f'{safe_name}_{task_id}.csv')}",
        )
        self.end_headers()
        self.wfile.write(body)


def main() -> None:
    mimetypes.add_type("text/javascript", ".js")
    address = ("127.0.0.1", 8080)
    server = ThreadingHTTPServer(address, BioScopeHandler)
    print(f"BioScope running at http://{address[0]}:{address[1]}")
    print(f"CSV data directory: {DATA_DIR}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
