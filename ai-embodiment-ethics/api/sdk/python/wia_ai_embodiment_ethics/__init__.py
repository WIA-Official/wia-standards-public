"""WIA AI Embodiment Ethics — Python SDK.

Ethical-evaluation and consent tracking for embodied AI systems

Generated from openapi-3.0.yaml. Production endpoint:
    https://api.wia.live/embodiment-ethics/v1
"""

from __future__ import annotations

import os
from typing import Any, Optional

import requests


class AiEmbodimentEthicsClient:
    """HTTP client for WIA AI Embodiment Ethics."""

    DEFAULT_BASE_URL = "https://api.wia.live/embodiment-ethics/v1"

    def __init__(
        self,
        base_url: Optional[str] = None,
        api_key: Optional[str] = None,
        timeout: float = 30.0,
        session: Optional[requests.Session] = None,
    ):
        self.base_url = (base_url or os.environ.get("WIA_AI_EMBODIMENT_ETHICS_URL", self.DEFAULT_BASE_URL)).rstrip("/")
        self.api_key = api_key or os.environ.get("WIA_AI_EMBODIMENT_ETHICS_API_KEY")
        self.timeout = timeout
        self.session = session or requests.Session()

    def _request(self, method: str, path: str, **kw: Any) -> Any:
        headers = kw.pop("headers", {}) or {}
        headers.setdefault("Content-Type", "application/json")
        if self.api_key:
            headers["X-API-Key"] = self.api_key
        resp = self.session.request(method, self.base_url + path, headers=headers, timeout=self.timeout, **kw)
        resp.raise_for_status()
        if resp.status_code == 204 or not resp.content:
            return None
        return resp.json()

def list_agents(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/agents", params={"limit": limit, "offset": offset})

def get_agent(self, id: str) -> dict:
    return self._request("GET", f"/agents/{id}")

def create_agent(self, body: dict) -> dict:
    return self._request("POST", f"/agents", json=body)

def replace_agent(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/agents/{id}", json=body)

def delete_agent(self, id: str) -> None:
    self._request("DELETE", f"/agents/{id}")

def list_ethics_checks(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/ethics-checks", params={"limit": limit, "offset": offset})

def get_ethicscheck(self, id: str) -> dict:
    return self._request("GET", f"/ethics-checks/{id}")

def create_ethicscheck(self, body: dict) -> dict:
    return self._request("POST", f"/ethics-checks", json=body)

def replace_ethicscheck(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/ethics-checks/{id}", json=body)

def delete_ethicscheck(self, id: str) -> None:
    self._request("DELETE", f"/ethics-checks/{id}")

def list_consents(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/consents", params={"limit": limit, "offset": offset})

def get_consent(self, id: str) -> dict:
    return self._request("GET", f"/consents/{id}")

def create_consent(self, body: dict) -> dict:
    return self._request("POST", f"/consents", json=body)

def replace_consent(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/consents/{id}", json=body)

def delete_consent(self, id: str) -> None:
    self._request("DELETE", f"/consents/{id}")

def list_policies(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/policies", params={"limit": limit, "offset": offset})

def get_policy(self, id: str) -> dict:
    return self._request("GET", f"/policies/{id}")

def create_policy(self, body: dict) -> dict:
    return self._request("POST", f"/policies", json=body)

def replace_policy(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/policies/{id}", json=body)

def delete_policy(self, id: str) -> None:
    self._request("DELETE", f"/policies/{id}")


__all__ = ["AiEmbodimentEthicsClient"]
