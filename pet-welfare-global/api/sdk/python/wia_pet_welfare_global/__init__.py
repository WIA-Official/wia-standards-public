"""WIA Pet Welfare Global — Python SDK.

Global companion-animal welfare indicators and incident reporting

Production endpoint: https://api.wia.live/pet-welfare-global/v1
"""

from __future__ import annotations

import os
from typing import Any, Optional

import requests


class PetWelfareGlobalClient:
    """HTTP client for WIA Pet Welfare Global."""

    DEFAULT_BASE_URL = "https://api.wia.live/pet-welfare-global/v1"

    def __init__(
        self,
        base_url: Optional[str] = None,
        api_key: Optional[str] = None,
        timeout: float = 30.0,
        session: Optional[requests.Session] = None,
    ):
        self.base_url = (base_url or os.environ.get("WIA_PET_WELFARE_GLOBAL_URL", self.DEFAULT_BASE_URL)).rstrip("/")
        self.api_key = api_key or os.environ.get("WIA_PET_WELFARE_GLOBAL_API_KEY")
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

def list_regions(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/regions", params={"limit": limit, "offset": offset})

def get_region(self, id: str) -> dict:
    return self._request("GET", f"/regions/{id}")

def create_region(self, body: dict) -> dict:
    return self._request("POST", f"/regions", json=body)

def replace_region(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/regions/{id}", json=body)

def delete_region(self, id: str) -> None:
    self._request("DELETE", f"/regions/{id}")
def list_welfare_indicators(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/welfare-indicators", params={"limit": limit, "offset": offset})

def get_welfareindicator(self, id: str) -> dict:
    return self._request("GET", f"/welfare-indicators/{id}")

def create_welfareindicator(self, body: dict) -> dict:
    return self._request("POST", f"/welfare-indicators", json=body)

def replace_welfareindicator(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/welfare-indicators/{id}", json=body)

def delete_welfareindicator(self, id: str) -> None:
    self._request("DELETE", f"/welfare-indicators/{id}")
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
def list_organizations(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/organizations", params={"limit": limit, "offset": offset})

def get_organization(self, id: str) -> dict:
    return self._request("GET", f"/organizations/{id}")

def create_organization(self, body: dict) -> dict:
    return self._request("POST", f"/organizations", json=body)

def replace_organization(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/organizations/{id}", json=body)

def delete_organization(self, id: str) -> None:
    self._request("DELETE", f"/organizations/{id}")


__all__ = ["PetWelfareGlobalClient"]
