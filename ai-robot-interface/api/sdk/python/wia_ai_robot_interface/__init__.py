"""WIA AI Robot Interface — Python SDK.

Unified REST + WSS interface for state, sensor streams, and capability discovery

Generated from openapi-3.0.yaml. Production endpoint:
    https://api.wia.live/ai-robot-interface/v1
"""

from __future__ import annotations

import os
from typing import Any, Optional

import requests


class AiRobotInterfaceClient:
    """HTTP client for WIA AI Robot Interface."""

    DEFAULT_BASE_URL = "https://api.wia.live/ai-robot-interface/v1"

    def __init__(
        self,
        base_url: Optional[str] = None,
        api_key: Optional[str] = None,
        timeout: float = 30.0,
        session: Optional[requests.Session] = None,
    ):
        self.base_url = (base_url or os.environ.get("WIA_AI_ROBOT_INTERFACE_URL", self.DEFAULT_BASE_URL)).rstrip("/")
        self.api_key = api_key or os.environ.get("WIA_AI_ROBOT_INTERFACE_API_KEY")
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

def list_robots(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/robots", params={"limit": limit, "offset": offset})

def get_robot(self, id: str) -> dict:
    return self._request("GET", f"/robots/{id}")

def create_robot(self, body: dict) -> dict:
    return self._request("POST", f"/robots", json=body)

def replace_robot(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/robots/{id}", json=body)

def delete_robot(self, id: str) -> None:
    self._request("DELETE", f"/robots/{id}")

def list_states(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/states", params={"limit": limit, "offset": offset})

def get_state(self, id: str) -> dict:
    return self._request("GET", f"/states/{id}")

def create_state(self, body: dict) -> dict:
    return self._request("POST", f"/states", json=body)

def replace_state(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/states/{id}", json=body)

def delete_state(self, id: str) -> None:
    self._request("DELETE", f"/states/{id}")

def list_sensors(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/sensors", params={"limit": limit, "offset": offset})

def get_sensor(self, id: str) -> dict:
    return self._request("GET", f"/sensors/{id}")

def create_sensor(self, body: dict) -> dict:
    return self._request("POST", f"/sensors", json=body)

def replace_sensor(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/sensors/{id}", json=body)

def delete_sensor(self, id: str) -> None:
    self._request("DELETE", f"/sensors/{id}")

def list_capabilities(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/capabilities", params={"limit": limit, "offset": offset})

def get_capability(self, id: str) -> dict:
    return self._request("GET", f"/capabilities/{id}")

def create_capability(self, body: dict) -> dict:
    return self._request("POST", f"/capabilities", json=body)

def replace_capability(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/capabilities/{id}", json=body)

def delete_capability(self, id: str) -> None:
    self._request("DELETE", f"/capabilities/{id}")


__all__ = ["AiRobotInterfaceClient"]
