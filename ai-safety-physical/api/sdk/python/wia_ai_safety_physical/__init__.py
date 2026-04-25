"""WIA AI Safety Physical — Python SDK.

Safety-zone definition and enforcement for physically-acting AI systems

Generated from openapi-3.0.yaml. Production endpoint:
    https://api.wia.live/ai-safety-physical/v1
"""

from __future__ import annotations

import os
from typing import Any, Optional

import requests


class AiSafetyPhysicalClient:
    """HTTP client for WIA AI Safety Physical."""

    DEFAULT_BASE_URL = "https://api.wia.live/ai-safety-physical/v1"

    def __init__(
        self,
        base_url: Optional[str] = None,
        api_key: Optional[str] = None,
        timeout: float = 30.0,
        session: Optional[requests.Session] = None,
    ):
        self.base_url = (base_url or os.environ.get("WIA_AI_SAFETY_PHYSICAL_URL", self.DEFAULT_BASE_URL)).rstrip("/")
        self.api_key = api_key or os.environ.get("WIA_AI_SAFETY_PHYSICAL_API_KEY")
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

def list_safety_zones(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/safety-zones", params={"limit": limit, "offset": offset})

def get_safetyzone(self, id: str) -> dict:
    return self._request("GET", f"/safety-zones/{id}")

def create_safetyzone(self, body: dict) -> dict:
    return self._request("POST", f"/safety-zones", json=body)

def replace_safetyzone(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/safety-zones/{id}", json=body)

def delete_safetyzone(self, id: str) -> None:
    self._request("DELETE", f"/safety-zones/{id}")

def list_hazards(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/hazards", params={"limit": limit, "offset": offset})

def get_hazard(self, id: str) -> dict:
    return self._request("GET", f"/hazards/{id}")

def create_hazard(self, body: dict) -> dict:
    return self._request("POST", f"/hazards", json=body)

def replace_hazard(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/hazards/{id}", json=body)

def delete_hazard(self, id: str) -> None:
    self._request("DELETE", f"/hazards/{id}")

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

def list_emergency_stops(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/emergency-stops", params={"limit": limit, "offset": offset})

def get_emergencystop(self, id: str) -> dict:
    return self._request("GET", f"/emergency-stops/{id}")

def create_emergencystop(self, body: dict) -> dict:
    return self._request("POST", f"/emergency-stops", json=body)

def replace_emergencystop(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/emergency-stops/{id}", json=body)

def delete_emergencystop(self, id: str) -> None:
    self._request("DELETE", f"/emergency-stops/{id}")


__all__ = ["AiSafetyPhysicalClient"]
