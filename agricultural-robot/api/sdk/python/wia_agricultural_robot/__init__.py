"""WIA Agricultural Robot — Python SDK.

Autonomous agricultural-robot fleet registration, telemetry, and task assignment

Generated from openapi-3.0.yaml. Production endpoint:
    https://api.wia.live/agricultural-robot/v1
"""

from __future__ import annotations

import os
from typing import Any, Optional

import requests


class AgriculturalRobotClient:
    """HTTP client for WIA Agricultural Robot."""

    DEFAULT_BASE_URL = "https://api.wia.live/agricultural-robot/v1"

    def __init__(
        self,
        base_url: Optional[str] = None,
        api_key: Optional[str] = None,
        timeout: float = 30.0,
        session: Optional[requests.Session] = None,
    ):
        self.base_url = (base_url or os.environ.get("WIA_AGRICULTURAL_ROBOT_URL", self.DEFAULT_BASE_URL)).rstrip("/")
        self.api_key = api_key or os.environ.get("WIA_AGRICULTURAL_ROBOT_API_KEY")
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

def list_telemetry(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/telemetry", params={"limit": limit, "offset": offset})

def get_telemetry(self, id: str) -> dict:
    return self._request("GET", f"/telemetry/{id}")

def create_telemetry(self, body: dict) -> dict:
    return self._request("POST", f"/telemetry", json=body)

def replace_telemetry(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/telemetry/{id}", json=body)

def delete_telemetry(self, id: str) -> None:
    self._request("DELETE", f"/telemetry/{id}")

def list_tasks(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/tasks", params={"limit": limit, "offset": offset})

def get_task(self, id: str) -> dict:
    return self._request("GET", f"/tasks/{id}")

def create_task(self, body: dict) -> dict:
    return self._request("POST", f"/tasks", json=body)

def replace_task(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/tasks/{id}", json=body)

def delete_task(self, id: str) -> None:
    self._request("DELETE", f"/tasks/{id}")

def list_field_maps(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/field-maps", params={"limit": limit, "offset": offset})

def get_fieldmap(self, id: str) -> dict:
    return self._request("GET", f"/field-maps/{id}")

def create_fieldmap(self, body: dict) -> dict:
    return self._request("POST", f"/field-maps", json=body)

def replace_fieldmap(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/field-maps/{id}", json=body)

def delete_fieldmap(self, id: str) -> None:
    self._request("DELETE", f"/field-maps/{id}")


__all__ = ["AgriculturalRobotClient"]
