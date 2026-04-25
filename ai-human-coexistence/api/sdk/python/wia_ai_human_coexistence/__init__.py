"""WIA AI Human Coexistence — Python SDK.

Human detection, zone management, and safety arbitration for shared AI/human spaces

Generated from openapi-3.0.yaml. Production endpoint:
    https://api.wia.live/ai-human-coexistence/v1
"""

from __future__ import annotations

import os
from typing import Any, Optional

import requests


class AiHumanCoexistenceClient:
    """HTTP client for WIA AI Human Coexistence."""

    DEFAULT_BASE_URL = "https://api.wia.live/ai-human-coexistence/v1"

    def __init__(
        self,
        base_url: Optional[str] = None,
        api_key: Optional[str] = None,
        timeout: float = 30.0,
        session: Optional[requests.Session] = None,
    ):
        self.base_url = (base_url or os.environ.get("WIA_AI_HUMAN_COEXISTENCE_URL", self.DEFAULT_BASE_URL)).rstrip("/")
        self.api_key = api_key or os.environ.get("WIA_AI_HUMAN_COEXISTENCE_API_KEY")
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

def list_detections(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/detections", params={"limit": limit, "offset": offset})

def get_detection(self, id: str) -> dict:
    return self._request("GET", f"/detections/{id}")

def create_detection(self, body: dict) -> dict:
    return self._request("POST", f"/detections", json=body)

def replace_detection(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/detections/{id}", json=body)

def delete_detection(self, id: str) -> None:
    self._request("DELETE", f"/detections/{id}")

def list_zones(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/zones", params={"limit": limit, "offset": offset})

def get_zone(self, id: str) -> dict:
    return self._request("GET", f"/zones/{id}")

def create_zone(self, body: dict) -> dict:
    return self._request("POST", f"/zones", json=body)

def replace_zone(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/zones/{id}", json=body)

def delete_zone(self, id: str) -> None:
    self._request("DELETE", f"/zones/{id}")

def list_incidents(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/incidents", params={"limit": limit, "offset": offset})

def get_incident(self, id: str) -> dict:
    return self._request("GET", f"/incidents/{id}")

def create_incident(self, body: dict) -> dict:
    return self._request("POST", f"/incidents", json=body)

def replace_incident(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/incidents/{id}", json=body)

def delete_incident(self, id: str) -> None:
    self._request("DELETE", f"/incidents/{id}")

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


__all__ = ["AiHumanCoexistenceClient"]
