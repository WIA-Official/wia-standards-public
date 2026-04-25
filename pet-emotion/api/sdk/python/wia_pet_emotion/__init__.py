"""WIA Pet Emotion — Python SDK.

Pet emotion detection and behavioral state classification

Production endpoint: https://api.wia.live/pet-emotion/v1
"""

from __future__ import annotations

import os
from typing import Any, Optional

import requests


class PetEmotionClient:
    """HTTP client for WIA Pet Emotion."""

    DEFAULT_BASE_URL = "https://api.wia.live/pet-emotion/v1"

    def __init__(
        self,
        base_url: Optional[str] = None,
        api_key: Optional[str] = None,
        timeout: float = 30.0,
        session: Optional[requests.Session] = None,
    ):
        self.base_url = (base_url or os.environ.get("WIA_PET_EMOTION_URL", self.DEFAULT_BASE_URL)).rstrip("/")
        self.api_key = api_key or os.environ.get("WIA_PET_EMOTION_API_KEY")
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
def list_emotion_readings(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/emotion-readings", params={"limit": limit, "offset": offset})

def get_emotionreading(self, id: str) -> dict:
    return self._request("GET", f"/emotion-readings/{id}")

def create_emotionreading(self, body: dict) -> dict:
    return self._request("POST", f"/emotion-readings", json=body)

def replace_emotionreading(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/emotion-readings/{id}", json=body)

def delete_emotionreading(self, id: str) -> None:
    self._request("DELETE", f"/emotion-readings/{id}")
def list_behavior_events(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/behavior-events", params={"limit": limit, "offset": offset})

def get_behaviorevent(self, id: str) -> dict:
    return self._request("GET", f"/behavior-events/{id}")

def create_behaviorevent(self, body: dict) -> dict:
    return self._request("POST", f"/behavior-events", json=body)

def replace_behaviorevent(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/behavior-events/{id}", json=body)

def delete_behaviorevent(self, id: str) -> None:
    self._request("DELETE", f"/behavior-events/{id}")
def list_models(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/models", params={"limit": limit, "offset": offset})

def get_model(self, id: str) -> dict:
    return self._request("GET", f"/models/{id}")

def create_model(self, body: dict) -> dict:
    return self._request("POST", f"/models", json=body)

def replace_model(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/models/{id}", json=body)

def delete_model(self, id: str) -> None:
    self._request("DELETE", f"/models/{id}")


__all__ = ["PetEmotionClient"]
