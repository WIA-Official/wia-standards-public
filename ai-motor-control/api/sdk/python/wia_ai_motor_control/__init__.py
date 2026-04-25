"""WIA AI Motor Control — Python SDK.

Motor registration, command issuance, and trajectory tracking for AI-driven actuators

Generated from openapi-3.0.yaml. Production endpoint:
    https://api.wia.live/motor-control/v1
"""

from __future__ import annotations

import os
from typing import Any, Optional

import requests


class AiMotorControlClient:
    """HTTP client for WIA AI Motor Control."""

    DEFAULT_BASE_URL = "https://api.wia.live/motor-control/v1"

    def __init__(
        self,
        base_url: Optional[str] = None,
        api_key: Optional[str] = None,
        timeout: float = 30.0,
        session: Optional[requests.Session] = None,
    ):
        self.base_url = (base_url or os.environ.get("WIA_AI_MOTOR_CONTROL_URL", self.DEFAULT_BASE_URL)).rstrip("/")
        self.api_key = api_key or os.environ.get("WIA_AI_MOTOR_CONTROL_API_KEY")
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

def list_motors(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/motors", params={"limit": limit, "offset": offset})

def get_motor(self, id: str) -> dict:
    return self._request("GET", f"/motors/{id}")

def create_motor(self, body: dict) -> dict:
    return self._request("POST", f"/motors", json=body)

def replace_motor(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/motors/{id}", json=body)

def delete_motor(self, id: str) -> None:
    self._request("DELETE", f"/motors/{id}")

def list_commands(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/commands", params={"limit": limit, "offset": offset})

def get_command(self, id: str) -> dict:
    return self._request("GET", f"/commands/{id}")

def create_command(self, body: dict) -> dict:
    return self._request("POST", f"/commands", json=body)

def replace_command(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/commands/{id}", json=body)

def delete_command(self, id: str) -> None:
    self._request("DELETE", f"/commands/{id}")

def list_trajectories(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/trajectories", params={"limit": limit, "offset": offset})

def get_trajectory(self, id: str) -> dict:
    return self._request("GET", f"/trajectories/{id}")

def create_trajectory(self, body: dict) -> dict:
    return self._request("POST", f"/trajectories", json=body)

def replace_trajectory(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/trajectories/{id}", json=body)

def delete_trajectory(self, id: str) -> None:
    self._request("DELETE", f"/trajectories/{id}")

def list_calibrations(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/calibrations", params={"limit": limit, "offset": offset})

def get_calibration(self, id: str) -> dict:
    return self._request("GET", f"/calibrations/{id}")

def create_calibration(self, body: dict) -> dict:
    return self._request("POST", f"/calibrations", json=body)

def replace_calibration(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/calibrations/{id}", json=body)

def delete_calibration(self, id: str) -> None:
    self._request("DELETE", f"/calibrations/{id}")


__all__ = ["AiMotorControlClient"]
