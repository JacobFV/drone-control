"""
LLM high-level swarm director.

Provider-agnostic tool-calling client (Anthropic Messages or OpenAI-compatible
chat completions) used as the *low-frequency* top of the control stack:

    drone <hi-freq> realtime controller <med-freq> VLA <low-freq> LLM director

Roughly once every few seconds the LLM is given each drone's state + the mission
objective and emits guidance **tool calls** (``set_target`` / ``set_trajectory``)
that flow onto the guidance bus and condition the medium-frequency VLA. There is
no analytic fallback — if no provider/key is configured, high-level control is
simply off.

No third-party SDKs: requests go over urllib so the only dependency is config.
"""

from __future__ import annotations

import json
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import Any


# Tool surface exposed to the LLM. Names + argument keys match
# coordinator.guidance.apply_tool_calls so results apply straight to the bus.
TOOLS: list[dict[str, Any]] = [
    {
        "name": "set_target",
        "description": "Send one drone to a single world-frame target point (metres). Clears any trajectory.",
        "schema": {
            "type": "object",
            "properties": {
                "droneId": {"type": "string"},
                "x": {"type": "number"},
                "y": {"type": "number"},
                "z": {"type": "number"},
            },
            "required": ["droneId", "x", "y", "z"],
        },
    },
    {
        "name": "set_trajectory",
        "description": "Give one drone an ordered list of world-frame waypoints to follow; set loop=true to patrol.",
        "schema": {
            "type": "object",
            "properties": {
                "droneId": {"type": "string"},
                "waypoints": {
                    "type": "array",
                    "items": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3},
                },
                "loop": {"type": "boolean"},
            },
            "required": ["droneId", "waypoints"],
        },
    },
]

SYSTEM_PROMPT = (
    "You are the high-level director of an autonomous drone swarm. You are called "
    "about once every 5 seconds with each drone's current state and the mission "
    "objective. You do NOT fly the drones directly — low-level controllers and a "
    "medium-level VLA handle flight. Your only job is to issue high-level guidance "
    "via the provided tools (set_target / set_trajectory) so the swarm accomplishes "
    "the objective: spread out to cover ground, converge on points of interest, "
    "patrol, or hold. Positions are in metres in a Z-up world frame. Issue tool "
    "calls only for drones whose guidance should change this cycle; it is fine to "
    "issue none. Be decisive and brief."
)


@dataclass(slots=True)
class LLMConfig:
    provider: str = "anthropic"          # "anthropic" | "openai"
    model: str = "claude-opus-4-8"
    api_key: str = ""
    base_url: str = ""                   # override (e.g. OpenAI-compatible gateways)
    temperature: float = 0.2
    max_tokens: int = 1024
    timeout_seconds: float = 30.0

    @property
    def configured(self) -> bool:
        return bool(self.api_key and self.model and self.provider)

    def as_public_dict(self) -> dict[str, Any]:
        """Config for the UI — never leak the key, just whether one is set."""
        return {
            "provider": self.provider,
            "model": self.model,
            "baseUrl": self.base_url,
            "temperature": self.temperature,
            "maxTokens": self.max_tokens,
            "hasApiKey": bool(self.api_key),
            "configured": self.configured,
        }


@dataclass(slots=True)
class LLMDirector:
    config: LLMConfig
    last_error: str | None = field(default=None)

    @property
    def available(self) -> bool:
        return self.config.configured

    def step(self, payload: dict[str, Any]) -> dict[str, Any]:
        """VLMCoordinator-compatible model_step: payload -> {state, assignments, notes, toolCalls}."""
        prompt = _build_user_prompt(payload)
        try:
            if self.config.provider == "openai":
                tool_calls, notes = self._call_openai(prompt)
            else:
                tool_calls, notes = self._call_anthropic(prompt)
            self.last_error = None
        except Exception as exc:  # network / API / parse
            self.last_error = str(exc)
            return {"state": "faulted", "assignments": [], "notes": [f"llm error: {exc}"], "toolCalls": []}
        return {
            "state": "running",
            "assignments": [],
            "notes": notes,
            "toolCalls": tool_calls,
        }

    # -- providers ---------------------------------------------------------

    def _call_anthropic(self, prompt: str) -> tuple[list[dict[str, Any]], list[str]]:
        base = self.config.base_url or "https://api.anthropic.com"
        body = {
            "model": self.config.model,
            "max_tokens": self.config.max_tokens,
            "temperature": self.config.temperature,
            "system": SYSTEM_PROMPT,
            "tools": [{"name": t["name"], "description": t["description"], "input_schema": t["schema"]} for t in TOOLS],
            "messages": [{"role": "user", "content": prompt}],
        }
        data = _post_json(
            f"{base.rstrip('/')}/v1/messages",
            body,
            headers={
                "x-api-key": self.config.api_key,
                "anthropic-version": "2023-06-01",
                "content-type": "application/json",
            },
            timeout=self.config.timeout_seconds,
        )
        tool_calls: list[dict[str, Any]] = []
        notes: list[str] = []
        for block in data.get("content", []):
            if block.get("type") == "tool_use":
                tool_calls.append({"name": block.get("name"), "arguments": block.get("input", {})})
            elif block.get("type") == "text" and block.get("text", "").strip():
                notes.append(block["text"].strip())
        return tool_calls, notes

    def _call_openai(self, prompt: str) -> tuple[list[dict[str, Any]], list[str]]:
        base = self.config.base_url or "https://api.openai.com"
        body = {
            "model": self.config.model,
            "temperature": self.config.temperature,
            "max_tokens": self.config.max_tokens,
            "tools": [{"type": "function", "function": {"name": t["name"], "description": t["description"], "parameters": t["schema"]}} for t in TOOLS],
            "tool_choice": "auto",
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": prompt},
            ],
        }
        data = _post_json(
            f"{base.rstrip('/')}/v1/chat/completions",
            body,
            headers={"authorization": f"Bearer {self.config.api_key}", "content-type": "application/json"},
            timeout=self.config.timeout_seconds,
        )
        message = (data.get("choices") or [{}])[0].get("message", {})
        tool_calls: list[dict[str, Any]] = []
        for call in message.get("tool_calls") or []:
            fn = call.get("function", {})
            try:
                args = json.loads(fn.get("arguments") or "{}")
            except json.JSONDecodeError:
                args = {}
            tool_calls.append({"name": fn.get("name"), "arguments": args})
        notes = [message["content"].strip()] if message.get("content") else []
        return tool_calls, notes


def _build_user_prompt(payload: dict[str, Any]) -> str:
    mission = payload.get("mission", {})
    drones = payload.get("drones", [])
    lines = [f"Objective: {mission.get('objective', 'operate safely')}"]
    context = mission.get("context")
    if context:
        lines.append(f"Context: {json.dumps(context)}")
    lines.append(f"Drones ({len(drones)}):")
    for d in drones:
        obs = d.get("observation", {})
        pose = obs.get("pose") or {}
        pos = pose.get("translation")
        lines.append(
            f"- {d.get('droneId')}: pos={[round(float(v),1) for v in pos] if pos else 'unknown'} "
            f"link={obs.get('linkState','?')} battery={obs.get('battery')}"
        )
    lines.append("Issue guidance tool calls as needed.")
    return "\n".join(lines)


def _post_json(url: str, body: dict[str, Any], headers: dict[str, str], timeout: float) -> dict[str, Any]:
    request = urllib.request.Request(url, data=json.dumps(body).encode(), headers=headers, method="POST")
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.loads(response.read().decode())
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode(errors="replace")[:300]
        raise RuntimeError(f"HTTP {exc.code}: {detail}") from exc
