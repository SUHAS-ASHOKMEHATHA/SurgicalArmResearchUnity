import json
import threading
import unittest
import urllib.request

from server import VlmBridge, build_server, extract_json_object, normalize_prediction


class VlmBridgeTests(unittest.TestCase):
    def test_extracts_fenced_json(self) -> None:
        result = extract_json_object('```json\n{"target":"brain"}\n```')
        self.assertEqual(result["target"], "brain")

    def test_normalizes_prediction_schema(self) -> None:
        prediction = normalize_prediction(
            "frame_1",
            "test-model",
            json.dumps(
                {
                    "target": "heart",
                    "tool": "navigation probe",
                    "stage": "mapping",
                    "contactState": "clear",
                    "safetyConcern": "none",
                    "confidence": 1.5,
                }
            ),
            25.0,
        )
        self.assertEqual(prediction["frameId"], "frame_1")
        self.assertEqual(prediction["confidence"], 1.0)
        self.assertTrue(prediction["validJson"])

    def test_dry_run_does_not_claim_model_inference(self) -> None:
        bridge = VlmBridge(dry_run=True)
        result = bridge.infer(
            {
                "frameId": "frame_2",
                "imageBase64": "aW1hZ2U=",
                "prompt": "Return JSON",
            }
        )
        self.assertEqual(result["model"], "dry-run-no-vlm")
        self.assertEqual(result["confidence"], 0.0)

    def test_dry_run_http_endpoint(self) -> None:
        server = build_server("127.0.0.1", 0, dry_run=True)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            request = urllib.request.Request(
                f"http://127.0.0.1:{server.server_port}/infer",
                data=json.dumps(
                    {
                        "frameId": "frame_http",
                        "imageBase64": "aW1hZ2U=",
                        "prompt": "Return JSON",
                    }
                ).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(request, timeout=2) as response:
                result = json.loads(response.read().decode("utf-8"))
            self.assertEqual(result["frameId"], "frame_http")
            self.assertEqual(result["model"], "dry-run-no-vlm")
            self.assertFalse(result.get("commandChannel", False))
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()