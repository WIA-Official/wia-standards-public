"""WIA Permafrost Protection — Python SDK.

Permafrost monitoring station registration, thermal observations, and degradation alerts

Generated from openapi-3.0.yaml. Production endpoint:
    https://api.wia.live/permafrost/v1
"""

from __future__ import annotations

import os
from typing import Any, Optional

import requests


class PermafrostProtectionClient:
    """HTTP client for WIA Permafrost Protection."""

    DEFAULT_BASE_URL = "https://api.wia.live/permafrost/v1"

    def __init__(
        self,
        base_url: Optional[str] = None,
        api_key: Optional[str] = None,
        timeout: float = 30.0,
        session: Optional[requests.Session] = None,
    ):
        self.base_url = (base_url or os.environ.get("WIA_PERMAFROST_PROTECTION_URL", self.DEFAULT_BASE_URL)).rstrip("/")
        self.api_key = api_key or os.environ.get("WIA_PERMAFROST_PROTECTION_API_KEY")
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

def list_stations(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/stations", params={"limit": limit, "offset": offset})

def get_station(self, id: str) -> dict:
    return self._request("GET", f"/stations/{id}")

def create_station(self, body: dict) -> dict:
    return self._request("POST", f"/stations", json=body)

def replace_station(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/stations/{id}", json=body)

def delete_station(self, id: str) -> None:
    self._request("DELETE", f"/stations/{id}")

def list_observations(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/observations", params={"limit": limit, "offset": offset})

def get_observation(self, id: str) -> dict:
    return self._request("GET", f"/observations/{id}")

def create_observation(self, body: dict) -> dict:
    return self._request("POST", f"/observations", json=body)

def replace_observation(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/observations/{id}", json=body)

def delete_observation(self, id: str) -> None:
    self._request("DELETE", f"/observations/{id}")

def list_alerts(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/alerts", params={"limit": limit, "offset": offset})

def get_alert(self, id: str) -> dict:
    return self._request("GET", f"/alerts/{id}")

def create_alert(self, body: dict) -> dict:
    return self._request("POST", f"/alerts", json=body)

def replace_alert(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/alerts/{id}", json=body)

def delete_alert(self, id: str) -> None:
    self._request("DELETE", f"/alerts/{id}")

def list_boreholes(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/boreholes", params={"limit": limit, "offset": offset})

def get_borehole(self, id: str) -> dict:
    return self._request("GET", f"/boreholes/{id}")

def create_borehole(self, body: dict) -> dict:
    return self._request("POST", f"/boreholes", json=body)

def replace_borehole(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/boreholes/{id}", json=body)

def delete_borehole(self, id: str) -> None:
    self._request("DELETE", f"/boreholes/{id}")


__all__ = ["PermafrostProtectionClient"]
