"""WIA Accessible UI — Python SDK.

WCAG 2.1 AAA accessibility validation, contrast checking, ARIA management

Generated from openapi-3.0.yaml. Production endpoint:
    https://api.wia.live/accessible-ui/v1
"""

from __future__ import annotations

import os
from typing import Any, Optional

import requests


class AccessibleUiClient:
    """HTTP client for WIA Accessible UI."""

    DEFAULT_BASE_URL = "https://api.wia.live/accessible-ui/v1"

    def __init__(
        self,
        base_url: Optional[str] = None,
        api_key: Optional[str] = None,
        timeout: float = 30.0,
        session: Optional[requests.Session] = None,
    ):
        self.base_url = (base_url or os.environ.get("WIA_ACCESSIBLE_UI_URL", self.DEFAULT_BASE_URL)).rstrip("/")
        self.api_key = api_key or os.environ.get("WIA_ACCESSIBLE_UI_API_KEY")
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

def list_audits(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/audits", params={"limit": limit, "offset": offset})

def get_audit(self, id: str) -> dict:
    return self._request("GET", f"/audits/{id}")

def create_audit(self, body: dict) -> dict:
    return self._request("POST", f"/audits", json=body)

def replace_audit(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/audits/{id}", json=body)

def delete_audit(self, id: str) -> None:
    self._request("DELETE", f"/audits/{id}")

def list_violations(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/violations", params={"limit": limit, "offset": offset})

def get_violation(self, id: str) -> dict:
    return self._request("GET", f"/violations/{id}")

def create_violation(self, body: dict) -> dict:
    return self._request("POST", f"/violations", json=body)

def replace_violation(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/violations/{id}", json=body)

def delete_violation(self, id: str) -> None:
    self._request("DELETE", f"/violations/{id}")

def list_components(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/components", params={"limit": limit, "offset": offset})

def get_component(self, id: str) -> dict:
    return self._request("GET", f"/components/{id}")

def create_component(self, body: dict) -> dict:
    return self._request("POST", f"/components", json=body)

def replace_component(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/components/{id}", json=body)

def delete_component(self, id: str) -> None:
    self._request("DELETE", f"/components/{id}")

def list_contrast_checks(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/contrast-checks", params={"limit": limit, "offset": offset})

def get_contrastcheck(self, id: str) -> dict:
    return self._request("GET", f"/contrast-checks/{id}")

def create_contrastcheck(self, body: dict) -> dict:
    return self._request("POST", f"/contrast-checks", json=body)

def replace_contrastcheck(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/contrast-checks/{id}", json=body)

def delete_contrastcheck(self, id: str) -> None:
    self._request("DELETE", f"/contrast-checks/{id}")


__all__ = ["AccessibleUiClient"]
