#!/usr/bin/env python3
"""Read-only bridge between Unity frame observations and a multimodal model API."""

from __future__ import annotations

import argparse
import base64
import json
import os
import re
import time
import urllib.error
import urllib.request
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict


MAX_REQUEST_BYTES = 12 * 1024 * 1024
DEFAULT_BASE_URL = "http://127.0.0.1:11434/v1"
DEFAULT_MODEL = "local-vlm"


def extract_json_object(content: str) -> Dict[str, Any]:
    """Extract one JSON object from plain text or a fenced model response."""
    text = (content or "").strip()
    if text.startswith("```"):
        text = re.sub(r"^```(?:json)?\s*", "", text, flags=re.IGNORECASE)
        text = re.sub(r"\s*```$", "", text)

    try:
        value = json.loads(text)
    except json.JSONDecodeError:
        start = text.find("{")
        end = text.rfind("}")
        if start < 0 or end <= start:
            raise ValueError("The model response does not contain a JSON object.")
        value = json.loads(text[start : end + 1])

    if not isinstance(value, dict):
        raise ValueError("The model response must be a JSON object.")
    return value


def normalize_prediction(
    frame_id: str,
    model: str,
    raw_content: str,
    latency_ms: float,
) -> Dict[str, Any]:
    parsed = extract_json_object(raw_content)
    confidence = parsed.get("confidence", 0.0)
    try:
        confidence = max(0.0, min(1.0, float(confidence)))
    except (TypeError, ValueError):
        confidence = 0.0

    return {
        "frameId": frame_id,
        "model": model,
        "target": str(parsed.get("target", "unknown")),
        "tool": str(parsed.get("tool", "unknown")),
        "stage": str(parsed.get("stage", "unknown")),
        "contactState": str(parsed.get("contactState", "unknown")),
        "safetyConcern": str(parsed.get("safetyConcern", "unknown")),
        "confidence": confidence,
        "latencyMs": max(0.0, latency_ms),
        "validJson": True,
        "rawResponse": raw_content,
    }


class VlmBridge:
    def __init__(self, dry_run: bool = False) -> None:
        self.dry_run = dry_run
        self.base_url = os.getenv("VLM_BASE_URL", DEFAULT_BASE_URL).rstrip("/")
        self.model = os.getenv("VLM_MODEL", DEFAULT_MODEL)
        self.api_key = os.getenv("VLM_API_KEY", "")
        self.timeout_seconds = float(os.getenv("VLM_TIMEOUT_SECONDS", "90"))

    @property
    def mode(self) -> str:
        return "dry-run" if self.dry_run else "vlm"

    def infer(self, request_data: Dict[str, Any]) -> Dict[str, Any]:
        frame_id = str(request_data.get("frameId", ""))
        image_base64 = str(request_data.get("imageBase64", ""))
        prompt = str(request_data.get("prompt", ""))
        if not frame_id or not image_base64 or not prompt:
            raise ValueError("frameId, imageBase64, and prompt are required.")

        try:
            base64.b64decode(image_base64, validate=True)
        except ValueError as exception:
            raise ValueError("imageBase64 is not valid base64 data.") from exception

        if self.dry_run:
            return normalize_prediction(
                frame_id,
                "dry-run-no-vlm",
                json.dumps(
                    {
                        "target": "none",
                        "tool": "no tool",
                        "stage": "dry-run bridge check",
                        "contactState": "clear",
                        "safetyConcern": "no model inference performed",
                        "confidence": 0.0,
                    }
                ),
                0.0,
            )

        payload = {
            "model": self.model,
            "temperature": 0,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{image_base64}"
                            },
                        },
                    ],
                }
            ],
        }
        headers = {"Content-Type": "application/json"}
        if self.api_key:
            headers["Authorization"] = f"Bearer {self.api_key}"

        upstream_request = urllib.request.Request(
            f"{self.base_url}/chat/completions",
            data=json.dumps(payload).encode("utf-8"),
            headers=headers,
            method="POST",
        )
        started = time.perf_counter()
        try:
            with urllib.request.urlopen(
                upstream_request, timeout=self.timeout_seconds
            ) as response:
                upstream_data = json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exception:
            body = exception.read().decode("utf-8", errors="replace")
            raise RuntimeError(
                f"VLM endpoint returned HTTP {exception.code}: {body[:500]}"
            ) from exception
        except urllib.error.URLError as exception:
            raise RuntimeError(f"VLM endpoint unavailable: {exception.reason}") from exception

        latency_ms = (time.perf_counter() - started) * 1000.0
        try:
            raw_content = upstream_data["choices"][0]["message"]["content"]
        except (KeyError, IndexError, TypeError) as exception:
            raise RuntimeError("Unexpected OpenAI-compatible response schema.") from exception
        return normalize_prediction(frame_id, self.model, raw_content, latency_ms)


class BridgeRequestHandler(BaseHTTPRequestHandler):
    bridge: VlmBridge

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        if self.path != "/health":
            self._write_json(HTTPStatus.NOT_FOUND, {"error": "not found"})
            return
        self._write_json(
            HTTPStatus.OK,
            {
                "status": "ok",
                "mode": self.bridge.mode,
                "model": self.bridge.model,
                "commandChannel": False,
            },
        )

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        if self.path != "/infer":
            self._write_json(HTTPStatus.NOT_FOUND, {"error": "not found"})
            return

        try:
            content_length = int(self.headers.get("Content-Length", "0"))
            if content_length <= 0 or content_length > MAX_REQUEST_BYTES:
                raise ValueError("Request size is invalid or exceeds 12 MB.")
            request_data = json.loads(self.rfile.read(content_length).decode("utf-8"))
            prediction = self.bridge.infer(request_data)
            self._write_json(HTTPStatus.OK, prediction)
        except (ValueError, json.JSONDecodeError) as exception:
            self._write_json(HTTPStatus.BAD_REQUEST, {"error": str(exception)})
        except Exception as exception:  # Bridge must return a controlled failure to Unity.
            self._write_json(HTTPStatus.BAD_GATEWAY, {"error": str(exception)})

    def log_message(self, format_string: str, *args: Any) -> None:
        print(f"[vlm-bridge] {self.address_string()} {format_string % args}")

    def _write_json(self, status: HTTPStatus, payload: Dict[str, Any]) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status.value)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def build_server(host: str, port: int, dry_run: bool) -> ThreadingHTTPServer:
    BridgeRequestHandler.bridge = VlmBridge(dry_run=dry_run)
    return ThreadingHTTPServer((host, port), BridgeRequestHandler)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8765, type=int)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate Unity-to-bridge plumbing without performing VLM inference.",
    )
    arguments = parser.parse_args()
    server = build_server(arguments.host, arguments.port, arguments.dry_run)
    bridge = BridgeRequestHandler.bridge
    print(
        f"VLM bridge listening on http://{arguments.host}:{arguments.port} "
        f"(mode={bridge.mode}, model={bridge.model}, command-channel=false)"
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()