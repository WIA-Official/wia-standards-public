"""WIA Pet Care Robot — Python SDK.

Companion-pet robotic care: feeding, monitoring, and welfare interaction

Production endpoint: https://api.wia.live/pet-care-robot/v1
"""

from __future__ import annotations

import os
from typing import Any, Optional

import requests


class PetCareRobotClient:
    """HTTP client for WIA Pet Care Robot."""

    DEFAULT_BASE_URL = "https://api.wia.live/pet-care-robot/v1"

    def __init__(
        self,
        base_url: Optional[str] = None,
        api_key: Optional[str] = None,
        timeout: float = 30.0,
        session: Optional[requests.Session] = None,
    ):
        self.base_url = (base_url or os.environ.get("WIA_PET_CARE_ROBOT_URL", self.DEFAULT_BASE_URL)).rstrip("/")
        self.api_key = api_key or os.environ.get("WIA_PET_CARE_ROBOT_API_KEY")
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
def list_pets(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/pets", params={"limit": limit, "offset": offset})

def get_pet(self, id: str) -> dict:
    return self._request("GET", f"/pets/{id}")

def create_pet(self, body: dict) -> dict:
    return self._request("POST", f"/pets", json=body)

def replace_pet(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/pets/{id}", json=body)

def delete_pet(self, id: str) -> None:
    self._request("DELETE", f"/pets/{id}")
def list_care_sessions(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/care-sessions", params={"limit": limit, "offset": offset})

def get_caresession(self, id: str) -> dict:
    return self._request("GET", f"/care-sessions/{id}")

def create_caresession(self, body: dict) -> dict:
    return self._request("POST", f"/care-sessions", json=body)

def replace_caresession(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/care-sessions/{id}", json=body)

def delete_caresession(self, id: str) -> None:
    self._request("DELETE", f"/care-sessions/{id}")
def list_schedules(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/schedules", params={"limit": limit, "offset": offset})

def get_schedule(self, id: str) -> dict:
    return self._request("GET", f"/schedules/{id}")

def create_schedule(self, body: dict) -> dict:
    return self._request("POST", f"/schedules", json=body)

def replace_schedule(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/schedules/{id}", json=body)

def delete_schedule(self, id: str) -> None:
    self._request("DELETE", f"/schedules/{id}")


__all__ = ["PetCareRobotClient"]
