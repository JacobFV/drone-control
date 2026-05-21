from __future__ import annotations

import json
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import Any


class HttpVLMError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class HttpVLMConfig:
    endpoint: str
    api_key: str = ""
    timeout_seconds: float = 5.0
    headers: dict[str, str] = field(default_factory=dict)


class HttpVLMClient:
    """
    Internet-side VLM coordinator adapter.

    The endpoint contract is intentionally simple: POST a JSON object containing
    mission and drone summaries; receive the structured mission-progress object
    that coordinator.vlm validates before applying constraints.
    """

    def __init__(self, config: HttpVLMConfig) -> None:
        if not config.endpoint:
            raise ValueError("VLM endpoint is required")
        self.config = config

    def step(self, payload: dict[str, Any]) -> dict[str, Any]:
        body = json.dumps(payload, separators=(",", ":")).encode()
        headers = {"Content-Type": "application/json", **self.config.headers}
        if self.config.api_key:
            headers.setdefault("Authorization", f"Bearer {self.config.api_key}")
        request = urllib.request.Request(self.config.endpoint, data=body, headers=headers, method="POST")
        try:
            with urllib.request.urlopen(request, timeout=self.config.timeout_seconds) as response:
                response_body = response.read()
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise HttpVLMError(f"VLM request failed: {exc}") from exc
        try:
            result = json.loads(response_body.decode())
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise HttpVLMError(f"VLM response was not JSON: {exc}") from exc
        if not isinstance(result, dict):
            raise HttpVLMError("VLM response must be an object")
        return result
