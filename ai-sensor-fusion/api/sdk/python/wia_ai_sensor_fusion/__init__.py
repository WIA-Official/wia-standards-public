"""WIA AI Sensor Fusion — Python SDK.

Multi-sensor fusion pipeline registration and fused-state retrieval

Generated from openapi-3.0.yaml. Production endpoint:
    https://api.wia.live/sensor-fusion/v1
"""

from __future__ import annotations

import os
from typing import Any, Optional

import requests


class AiSensorFusionClient:
    """HTTP client for WIA AI Sensor Fusion."""

    DEFAULT_BASE_URL = "https://api.wia.live/sensor-fusion/v1"

    def __init__(
        self,
        base_url: Optional[str] = None,
        api_key: Optional[str] = None,
        timeout: float = 30.0,
        session: Optional[requests.Session] = None,
    ):
        self.base_url = (base_url or os.environ.get("WIA_AI_SENSOR_FUSION_URL", self.DEFAULT_BASE_URL)).rstrip("/")
        self.api_key = api_key or os.environ.get("WIA_AI_SENSOR_FUSION_API_KEY")
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

def list_pipelines(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/pipelines", params={"limit": limit, "offset": offset})

def get_fusionpipeline(self, id: str) -> dict:
    return self._request("GET", f"/pipelines/{id}")

def create_fusionpipeline(self, body: dict) -> dict:
    return self._request("POST", f"/pipelines", json=body)

def replace_fusionpipeline(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/pipelines/{id}", json=body)

def delete_fusionpipeline(self, id: str) -> None:
    self._request("DELETE", f"/pipelines/{id}")

def list_fused_states(self, limit: int = 20, offset: int = 0) -> dict:
    return self._request("GET", f"/fused-states", params={"limit": limit, "offset": offset})

def get_fusedstate(self, id: str) -> dict:
    return self._request("GET", f"/fused-states/{id}")

def create_fusedstate(self, body: dict) -> dict:
    return self._request("POST", f"/fused-states", json=body)

def replace_fusedstate(self, id: str, body: dict) -> dict:
    return self._request("PUT", f"/fused-states/{id}", json=body)

def delete_fusedstate(self, id: str) -> None:
    self._request("DELETE", f"/fused-states/{id}")

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


__all__ = ["AiSensorFusionClient"]
