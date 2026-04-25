"""WIA Pet Genome — Python SDK.

Companion-animal genome data submission and breed/disease screening

Production endpoint: https://api.wia.live/pet-genome/v1
"""

from __future__ import annotations

import os
from typing import Any, Optional

import requests


class PetGenomeClient:
    """HTTP client for WIA Pet Genome."""

    DEFAULT_BASE_URL = "https://api.wia.live/pet-genome/v1"

    def __init__(
        self,
        base_url: Optional[str] = None,
        api_key: Optional[str] = None,
        timeout: float = 30.0,
        session: Optional[requests.Session] = None,
    ):
        self.base_url = (base_url or os.environ.get("WIA_PET_GENOME_URL", self.DEFAULT_BASE_URL)).rstrip("/")
        self.api_key = api_key or os.environ.get("WIA_PET_GENOME_API_KEY")
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

def list_samples(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/samples", params={"limit": limit, "offset": offset})

def get_sample(self, id: str) -> dict:
    return self._request("GET", f"/samples/{id}")

def create_sample(self, body: dict) -> dict:
    return self._request("POST", f"/samples", json=body)

def replace_sample(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/samples/{id}", json=body)

def delete_sample(self, id: str) -> None:
    self._request("DELETE", f"/samples/{id}")
def list_sequences(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/sequences", params={"limit": limit, "offset": offset})

def get_sequence(self, id: str) -> dict:
    return self._request("GET", f"/sequences/{id}")

def create_sequence(self, body: dict) -> dict:
    return self._request("POST", f"/sequences", json=body)

def replace_sequence(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/sequences/{id}", json=body)

def delete_sequence(self, id: str) -> None:
    self._request("DELETE", f"/sequences/{id}")
def list_breed_reports(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/breed-reports", params={"limit": limit, "offset": offset})

def get_breedreport(self, id: str) -> dict:
    return self._request("GET", f"/breed-reports/{id}")

def create_breedreport(self, body: dict) -> dict:
    return self._request("POST", f"/breed-reports", json=body)

def replace_breedreport(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/breed-reports/{id}", json=body)

def delete_breedreport(self, id: str) -> None:
    self._request("DELETE", f"/breed-reports/{id}")
def list_health_markers(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/health-markers", params={"limit": limit, "offset": offset})

def get_healthmarker(self, id: str) -> dict:
    return self._request("GET", f"/health-markers/{id}")

def create_healthmarker(self, body: dict) -> dict:
    return self._request("POST", f"/health-markers", json=body)

def replace_healthmarker(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/health-markers/{id}", json=body)

def delete_healthmarker(self, id: str) -> None:
    self._request("DELETE", f"/health-markers/{id}")


__all__ = ["PetGenomeClient"]
