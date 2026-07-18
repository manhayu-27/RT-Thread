import csv
import io
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import server


class FakeResponse(io.BytesIO):
    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.close()


class ServerTest(unittest.TestCase):
    def test_signal_summary_and_ark_response(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(
            server, "DATA_DIR", Path(directory)
        ):
            task_id = "test_task"
            server.meta_path(task_id).write_text(
                json.dumps({"id": task_id, "name": "test", "sampleCount": 2}),
                encoding="utf-8",
            )
            with server.csv_path(task_id).open("w", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow(server.CSV_HEADER)
                writer.writerow([0, 1000000, -1, 1, 2, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0])
                writer.writerow([1, 1002000, 1, -1, -2, -3, 0, 0, 0, 0, 0, 0, 0, 0, 12])

            summary = server.summarize_task(task_id)
            self.assertEqual(summary["channels"]["ecg"]["rms"], 1.0)
            self.assertEqual(summary["alarmSampleCounts"]["ecgAbnormal"], 1)
            self.assertEqual(summary["alarmSampleCounts"]["emg1High"], 1)

        response = FakeResponse(b'{"choices":[{"message":{"content":"ok"}}]}')
        with patch.dict(os.environ, {"ARK_API_KEY": "test-key"}), patch.object(
            server, "urlopen", return_value=response
        ):
            self.assertEqual(server.call_ark([{"role": "user", "content": "hi"}]), "ok")


if __name__ == "__main__":
    unittest.main()
