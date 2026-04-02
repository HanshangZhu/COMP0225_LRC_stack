from __future__ import annotations

import json
import os
import re
import urllib.error
import urllib.request
from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass
class VLMResponse:
    """Parsed response from the VLM."""

    frontier_rep: tuple[int, int]  # representative cell coords (stable across replans)
    reasoning: str
    raw_response: str


class BaseVLMClient(ABC):
    @abstractmethod
    def query(
        self,
        goal_prompt: str,
        map_state: dict,
        image_b64: str,
    ) -> VLMResponse:
        """Query the VLM with the current map state.

        Parameters
        ----------
        goal_prompt : natural-language task description
        map_state   : dict from build_map_state_json()
        image_b64   : base64-encoded PNG of the current map frame
        """
        ...


# ---------------------------------------------------------------------------
# Shared prompt + parsing helpers
# ---------------------------------------------------------------------------

def _build_prompt(goal_prompt: str, map_state: dict) -> str:
    frontiers = map_state.get("frontiers", [])
    if frontiers:
        frontier_lines = "\n".join(
            f"  [{f['idx']}] position=({f['rep_x']},{f['rep_y']})  cluster_size={f['cluster_size']}"
            for f in frontiers
        )
    else:
        frontier_lines = "  (none available)"

    return (
        f"You are controlling a robot exploring a 2D grid map.\n"
        f"The image shows the current map state:\n"
        f"  Gray  = unknown / unexplored area\n"
        f"  White = explored free space\n"
        f"  Black = obstacle / wall\n"
        f"  Green X markers = frontier candidates (boundaries of unknown space)\n"
        f"  Red circle = the robot\n\n"
        f"GOAL: {goal_prompt}\n\n"
        f"The robot can navigate to {len(frontiers)} frontier region(s):\n"
        f"{frontier_lines}\n\n"
        f"Coverage so far: {map_state.get('coverage_pct', 0.0):.1f}%\n"
        f"Step: {map_state.get('step_idx', 0)}\n"
        f"Artifacts found: {map_state.get('artifacts_found', 0)} / {map_state.get('artifacts_total', 1)}\n\n"
        f"Based on the visual map and the goal, select the frontier index most likely to lead to the target.\n"
        f"Respond ONLY with a JSON object in this exact format (no markdown, no extra text):\n"
        f'{{\"frontier_index\": <integer 0 to {max(0, len(frontiers)-1)}>, \"reasoning\": \"<brief explanation>\"}}'
    )


def _parse_vlm_response(
    raw: str,
    frontiers: list[dict],
) -> tuple[int, str]:
    """Parse frontier_index from VLM text output.  Returns (index, reasoning)."""
    n = len(frontiers)
    if n == 0:
        return 0, "(no frontiers)"

    # Layer 1: find first JSON block containing frontier_index
    match = re.search(r'\{[^{}]*"frontier_index"[^{}]*\}', raw, re.DOTALL)
    if match:
        try:
            data = json.loads(match.group())
            idx = int(data["frontier_index"])
            idx = max(0, min(idx, n - 1))
            reasoning = str(data.get("reasoning", ""))
            return idx, reasoning
        except (json.JSONDecodeError, KeyError, ValueError):
            pass

    # Layer 2: scan for any standalone integer in valid range
    for tok in re.findall(r"\b(\d+)\b", raw):
        candidate = int(tok)
        if 0 <= candidate < n:
            return candidate, "(index parsed from text)"

    # Layer 3: safe fallback
    return 0, "(fallback — could not parse VLM response)"


# ---------------------------------------------------------------------------
# Gemini backend  (google-generativeai, free tier)
# ---------------------------------------------------------------------------

class GeminiVLMClient(BaseVLMClient):
    def __init__(self, api_key: str, model: str = "gemini-1.5-flash") -> None:
        try:
            import google.generativeai as genai  # type: ignore
        except ImportError as e:
            raise ImportError(
                "google-generativeai is required for the Gemini backend. "
                "Run: pip install google-generativeai"
            ) from e
        genai.configure(api_key=api_key)
        self._model = genai.GenerativeModel(model)

    def query(self, goal_prompt: str, map_state: dict, image_b64: str) -> VLMResponse:
        import base64

        prompt_text = _build_prompt(goal_prompt, map_state)
        image_part = {
            "mime_type": "image/png",
            "data": base64.b64decode(image_b64),
        }
        response = self._model.generate_content([prompt_text, image_part])
        raw = response.text or ""
        idx, reasoning = _parse_vlm_response(raw, map_state.get("frontiers", []))
        frontiers = map_state.get("frontiers", [])
        rep = (
            (frontiers[idx]["rep_x"], frontiers[idx]["rep_y"])
            if frontiers
            else (0, 0)
        )
        return VLMResponse(frontier_rep=rep, reasoning=reasoning, raw_response=raw)


# ---------------------------------------------------------------------------
# Claude backend  (anthropic SDK)
# ---------------------------------------------------------------------------

class ClaudeVLMClient(BaseVLMClient):
    def __init__(self, api_key: str, model: str = "claude-haiku-4-5-20251001") -> None:
        try:
            import anthropic  # type: ignore
        except ImportError as e:
            raise ImportError(
                "anthropic is required for the Claude backend. "
                "Run: pip install anthropic"
            ) from e
        self._client = anthropic.Anthropic(api_key=api_key)
        self._model = model

    def query(self, goal_prompt: str, map_state: dict, image_b64: str) -> VLMResponse:
        import anthropic  # type: ignore

        prompt_text = _build_prompt(goal_prompt, map_state)
        response = self._client.messages.create(
            model=self._model,
            max_tokens=512,
            messages=[
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image",
                            "source": {
                                "type": "base64",
                                "media_type": "image/png",
                                "data": image_b64,
                            },
                        },
                        {"type": "text", "text": prompt_text},
                    ],
                }
            ],
        )
        raw = response.content[0].text if response.content else ""
        idx, reasoning = _parse_vlm_response(raw, map_state.get("frontiers", []))
        frontiers = map_state.get("frontiers", [])
        rep = (
            (frontiers[idx]["rep_x"], frontiers[idx]["rep_y"])
            if frontiers
            else (0, 0)
        )
        return VLMResponse(frontier_rep=rep, reasoning=reasoning, raw_response=raw)


# ---------------------------------------------------------------------------
# xAI / Grok backend (OpenAI-compatible chat completions)
# ---------------------------------------------------------------------------

class GrokVLMClient(BaseVLMClient):
    """xAI Grok vision backend via the OpenAI-compatible REST API."""

    def __init__(
        self,
        api_key: str,
        model: str | None = None,
        base_url: str | None = None,
    ) -> None:
        self._api_key = api_key
        self._model = model or os.environ.get("XAI_VLM_MODEL", "grok-2-vision-latest")
        self._base_url = (base_url or os.environ.get("XAI_API_BASE_URL", "https://api.x.ai/v1")).rstrip("/")

    def query(self, goal_prompt: str, map_state: dict, image_b64: str) -> VLMResponse:
        prompt_text = _build_prompt(goal_prompt, map_state)
        payload = {
            "model": self._model,
            "max_tokens": 512,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt_text},
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/png;base64,{image_b64}"},
                        },
                    ],
                }
            ],
        }
        request = urllib.request.Request(
            url=f"{self._base_url}/chat/completions",
            data=json.dumps(payload).encode("utf-8"),
            headers={
                "Authorization": f"Bearer {self._api_key}",
                "Content-Type": "application/json",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=60) as response:
                body = response.read().decode("utf-8")
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"xAI API error {exc.code}: {detail[:400]}") from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(f"xAI API request failed: {exc.reason}") from exc

        data = json.loads(body)
        raw = (
            data.get("choices", [{}])[0]
            .get("message", {})
            .get("content", "")
        ) or ""
        idx, reasoning = _parse_vlm_response(raw, map_state.get("frontiers", []))
        frontiers = map_state.get("frontiers", [])
        rep = (
            (frontiers[idx]["rep_x"], frontiers[idx]["rep_y"])
            if frontiers
            else (0, 0)
        )
        return VLMResponse(frontier_rep=rep, reasoning=reasoning, raw_response=raw)


# ---------------------------------------------------------------------------
# Groq backend  (groq package — OpenAI-compatible, free tier)
# ---------------------------------------------------------------------------

class GroqVLMClient(BaseVLMClient):
    """Groq-hosted vision model (Llama 4 Scout / Maverick).

    Install: pip install groq
    Models with vision: meta-llama/llama-4-scout-17b-16e-instruct
                        meta-llama/llama-4-maverick-17b-128e-instruct
    """

    def __init__(
        self,
        api_key: str,
        model: str = "meta-llama/llama-4-scout-17b-16e-instruct",
    ) -> None:
        try:
            from groq import Groq  # type: ignore
        except ImportError as e:
            raise ImportError(
                "groq is required for the Groq backend. Run: pip install groq"
            ) from e
        self._client = Groq(api_key=api_key)  # type: ignore
        self._model = model

    def query(self, goal_prompt: str, map_state: dict, image_b64: str) -> VLMResponse:
        prompt_text = _build_prompt(goal_prompt, map_state)
        response = self._client.chat.completions.create(
            model=self._model,
            max_tokens=512,
            messages=[
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/png;base64,{image_b64}"},
                        },
                        {"type": "text", "text": prompt_text},
                    ],
                }
            ],
        )
        raw = response.choices[0].message.content or ""
        idx, reasoning = _parse_vlm_response(raw, map_state.get("frontiers", []))
        frontiers = map_state.get("frontiers", [])
        rep = (
            (frontiers[idx]["rep_x"], frontiers[idx]["rep_y"])
            if frontiers
            else (0, 0)
        )
        return VLMResponse(frontier_rep=rep, reasoning=reasoning, raw_response=raw)


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

def build_vlm_client(backend: str, api_key: str) -> BaseVLMClient:
    """Build a VLM client from a backend name and API key."""
    b = backend.lower().strip()
    if b in ("grok", "xai", "x.ai"):
        return GrokVLMClient(api_key=api_key)
    if b == "gemini":
        return GeminiVLMClient(api_key=api_key)
    if b in ("claude", "claude_haiku", "anthropic"):
        return ClaudeVLMClient(api_key=api_key)
    if b == "groq":
        return GroqVLMClient(api_key=api_key)
    raise ValueError(
        f"Unknown VLM backend: '{backend}'. Choose 'grok', 'groq', 'gemini', or 'claude'."
    )
