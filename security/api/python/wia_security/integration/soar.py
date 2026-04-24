"""
WIA Security SOAR Integration
Security Orchestration, Automation, and Response
Phantom, XSOAR, Shuffle
"""

import time
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any
from datetime import datetime

try:
    import httpx
    HAS_HTTPX = True
except ImportError:
    HAS_HTTPX = False

from ..types import WiaSecurityEvent


# ============================================================================
# Types
# ============================================================================

@dataclass
class SoarConfig:
    """Base SOAR configuration."""
    url: str
    api_key: Optional[str] = None
    username: Optional[str] = None
    password: Optional[str] = None
    timeout: float = 30.0


@dataclass
class PlaybookAction:
    """Playbook action definition."""
    id: str
    name: str
    app: str
    action: str
    parameters: Optional[Dict[str, Any]] = None
    status: Optional[str] = None  # pending, running, success, failed
    result: Optional[Any] = None


@dataclass
class Playbook:
    """Playbook definition."""
    id: str
    name: str
    description: Optional[str] = None
    actions: List[PlaybookAction] = field(default_factory=list)
    triggers: Optional[List[str]] = None
    enabled: bool = True


@dataclass
class Artifact:
    """Case artifact."""
    id: str
    type: str
    value: str
    tags: Optional[List[str]] = None
    source: Optional[str] = None


@dataclass
class SoarCase:
    """SOAR case/incident."""
    id: str
    name: str
    status: str  # new, open, in_progress, resolved, closed
    severity: str  # low, medium, high, critical
    owner: Optional[str] = None
    artifacts: Optional[List[Artifact]] = None
    events: Optional[List[str]] = None
    created_at: str = ""
    updated_at: str = ""


# ============================================================================
# Splunk Phantom Integration
# ============================================================================

@dataclass
class PhantomConfig(SoarConfig):
    """Phantom configuration."""
    verify_tls: bool = True


class PhantomClient:
    """Splunk Phantom SOAR client."""

    def __init__(self, config: PhantomConfig):
        if not HAS_HTTPX:
            raise ImportError("httpx package required. Install with: pip install httpx")

        self.config = config
        self._client = httpx.Client(
            timeout=config.timeout,
            headers={
                "ph-auth-token": config.api_key or "",
                "Content-Type": "application/json",
            },
            verify=config.verify_tls,
        )

    def close(self) -> None:
        self._client.close()

    def __enter__(self) -> "PhantomClient":
        return self

    def __exit__(self, *args) -> None:
        self.close()

    def create_container(self, event: WiaSecurityEvent) -> str:
        """Create container (case)."""
        container = {
            "name": event.description,
            "label": "wia_security",
            "severity": self._map_severity(event.severity),
            "status": "new",
            "source_data_identifier": event.id,
            "custom_fields": {
                "wia_event_type": event.type.value,
                "wia_event_id": event.id,
            },
        }

        response = self._client.post(
            f"{self.config.url}/rest/container",
            json=container,
        )
        response.raise_for_status()

        return str(response.json()["id"])

    def add_artifact(self, container_id: str, artifact: Artifact) -> str:
        """Add artifact to container."""
        phantom_artifact = {
            "container_id": int(container_id),
            "name": artifact.type,
            "source_data_identifier": artifact.id,
            "cef": {artifact.type: artifact.value},
            "tags": artifact.tags or [],
        }

        response = self._client.post(
            f"{self.config.url}/rest/artifact",
            json=phantom_artifact,
        )
        response.raise_for_status()

        return str(response.json()["id"])

    def run_playbook(
        self,
        playbook_name: str,
        container_id: str,
        scope: str = "all",
    ) -> str:
        """Run playbook."""
        response = self._client.post(
            f"{self.config.url}/rest/playbook_run",
            json={
                "container_id": int(container_id),
                "playbook_id": playbook_name,
                "scope": scope,
            },
        )
        response.raise_for_status()

        return str(response.json()["playbook_run_id"])

    def get_playbook_status(self, run_id: str) -> Dict[str, Any]:
        """Get playbook run status."""
        response = self._client.get(
            f"{self.config.url}/rest/playbook_run/{run_id}"
        )
        response.raise_for_status()

        result = response.json()
        return {
            "status": result.get("status"),
            "message": result.get("message"),
        }

    def get_container(self, container_id: str) -> SoarCase:
        """Get container."""
        response = self._client.get(
            f"{self.config.url}/rest/container/{container_id}"
        )
        response.raise_for_status()

        data = response.json()
        return SoarCase(
            id=str(data["id"]),
            name=data["name"],
            status=self._map_phantom_status(data.get("status", "")),
            severity=self._reverse_map_severity(data.get("severity", "medium")),
            owner=data.get("owner_name"),
            created_at=data.get("create_time", ""),
            updated_at=data.get("update_time", ""),
        )

    def update_container_status(self, container_id: str, status: str) -> None:
        """Update container status."""
        response = self._client.post(
            f"{self.config.url}/rest/container/{container_id}",
            json={"status": self._map_status_to_phantom(status)},
        )
        response.raise_for_status()

    def _map_severity(self, severity: float) -> str:
        if severity >= 9:
            return "critical"
        if severity >= 7:
            return "high"
        if severity >= 4:
            return "medium"
        return "low"

    def _reverse_map_severity(self, severity: str) -> str:
        return severity if severity in ("low", "medium", "high", "critical") else "medium"

    def _map_phantom_status(self, status: str) -> str:
        mapping = {
            "new": "new",
            "open": "open",
            "in progress": "in_progress",
            "resolved": "resolved",
            "closed": "closed",
        }
        return mapping.get(status.lower(), "open")

    def _map_status_to_phantom(self, status: str) -> str:
        mapping = {
            "new": "new",
            "open": "open",
            "in_progress": "in progress",
            "resolved": "resolved",
            "closed": "closed",
        }
        return mapping.get(status, "open")


# ============================================================================
# Palo Alto XSOAR Integration
# ============================================================================

@dataclass
class XsoarConfig(SoarConfig):
    """XSOAR configuration."""
    pass


class XsoarClient:
    """Palo Alto XSOAR client."""

    def __init__(self, config: XsoarConfig):
        if not HAS_HTTPX:
            raise ImportError("httpx package required. Install with: pip install httpx")

        self.config = config
        self._client = httpx.Client(
            timeout=config.timeout,
            headers={
                "Authorization": config.api_key or "",
                "Content-Type": "application/json",
                "Accept": "application/json",
            },
        )

    def close(self) -> None:
        self._client.close()

    def __enter__(self) -> "XsoarClient":
        return self

    def __exit__(self, *args) -> None:
        self.close()

    def create_incident(self, event: WiaSecurityEvent) -> str:
        """Create incident."""
        incident = {
            "name": event.description,
            "type": "WIA Security Event",
            "severity": self._map_severity(event.severity),
            "labels": [
                {"type": "wia_event_type", "value": event.type.value},
                {"type": "wia_event_id", "value": event.id},
            ],
            "rawJSON": event.to_dict(),
        }

        response = self._client.post(
            f"{self.config.url}/incident",
            json=incident,
        )
        response.raise_for_status()

        return response.json()["id"]

    def add_indicator(
        self,
        value: str,
        type: str,
        score: int,
        source: str,
    ) -> str:
        """Add indicator."""
        response = self._client.post(
            f"{self.config.url}/indicator/create",
            json={
                "indicator": {
                    "value": value,
                    "indicator_type": self._map_indicator_type(type),
                    "score": score,
                    "source": source,
                }
            },
        )
        response.raise_for_status()

        return response.json()["id"]

    def run_playbook(self, playbook_id: str, incident_id: str) -> str:
        """Run playbook."""
        response = self._client.post(
            f"{self.config.url}/incident/investigate",
            json={
                "id": incident_id,
                "playbookId": playbook_id,
            },
        )
        response.raise_for_status()

        return response.json()["investigationId"]

    def get_incident(self, incident_id: str) -> SoarCase:
        """Get incident."""
        response = self._client.get(
            f"{self.config.url}/incident/load/{incident_id}"
        )
        response.raise_for_status()

        data = response.json()
        return SoarCase(
            id=data["id"],
            name=data["name"],
            status=self._map_xsoar_status(data.get("status", 0)),
            severity=self._reverse_map_severity(data.get("severity", 2)),
            owner=data.get("owner"),
            created_at=data.get("created", ""),
            updated_at=data.get("modified", ""),
        )

    def close_incident(
        self,
        incident_id: str,
        close_reason: str,
        close_notes: Optional[str] = None,
    ) -> None:
        """Close incident."""
        response = self._client.post(
            f"{self.config.url}/incident/close",
            json={
                "id": incident_id,
                "closeReason": close_reason,
                "closeNotes": close_notes,
            },
        )
        response.raise_for_status()

    def _map_severity(self, severity: float) -> int:
        if severity >= 9:
            return 4  # Critical
        if severity >= 7:
            return 3  # High
        if severity >= 4:
            return 2  # Medium
        return 1  # Low

    def _reverse_map_severity(self, severity: int) -> str:
        mapping = {4: "critical", 3: "high", 2: "medium", 1: "low"}
        return mapping.get(severity, "medium")

    def _map_xsoar_status(self, status: int) -> str:
        mapping = {0: "new", 1: "open", 2: "in_progress", 3: "closed"}
        return mapping.get(status, "open")

    def _map_indicator_type(self, type: str) -> str:
        mapping = {
            "ipv4": "IP",
            "ipv6": "IPv6",
            "domain": "Domain",
            "url": "URL",
            "email": "Email",
            "md5": "File MD5",
            "sha1": "File SHA-1",
            "sha256": "File SHA-256",
        }
        return mapping.get(type, type)


# ============================================================================
# Shuffle SOAR Integration
# ============================================================================

@dataclass
class ShuffleConfig(SoarConfig):
    """Shuffle configuration."""
    pass


class ShuffleClient:
    """Shuffle SOAR client."""

    def __init__(self, config: ShuffleConfig):
        if not HAS_HTTPX:
            raise ImportError("httpx package required. Install with: pip install httpx")

        self.config = config
        self._client = httpx.Client(
            timeout=config.timeout,
            headers={
                "Authorization": f"Bearer {config.api_key}",
                "Content-Type": "application/json",
            },
        )

    def close(self) -> None:
        self._client.close()

    def __enter__(self) -> "ShuffleClient":
        return self

    def __exit__(self, *args) -> None:
        self.close()

    def execute_workflow(
        self,
        workflow_id: str,
        event: WiaSecurityEvent,
    ) -> str:
        """Execute workflow."""
        import json

        response = self._client.post(
            f"{self.config.url}/api/v1/workflows/{workflow_id}/execute",
            json={
                "execution_argument": json.dumps(event.to_dict()),
                "start": "",
            },
        )
        response.raise_for_status()

        return response.json()["execution_id"]

    def get_execution_status(self, execution_id: str) -> Dict[str, Any]:
        """Get execution status."""
        response = self._client.get(
            f"{self.config.url}/api/v1/streams/results/{execution_id}"
        )
        response.raise_for_status()

        data = response.json()
        return {
            "status": data.get("status"),
            "results": data.get("results"),
        }

    def list_workflows(self) -> List[Playbook]:
        """List workflows."""
        response = self._client.get(
            f"{self.config.url}/api/v1/workflows"
        )
        response.raise_for_status()

        return [
            Playbook(
                id=wf.get("id", ""),
                name=wf.get("name", ""),
                description=wf.get("description"),
                actions=[
                    PlaybookAction(
                        id=a.get("id", ""),
                        name=a.get("name", ""),
                        app=a.get("app_name", ""),
                        action=a.get("name", ""),
                        parameters=a.get("parameters"),
                    )
                    for a in wf.get("actions", [])
                ],
                triggers=[t.get("name", "") for t in wf.get("triggers", [])],
                enabled=wf.get("is_valid", False),
            )
            for wf in response.json()
        ]

    def create_webhook(
        self,
        workflow_id: str,
        name: str,
    ) -> Dict[str, str]:
        """Create webhook trigger."""
        response = self._client.post(
            f"{self.config.url}/api/v1/hooks/new",
            json={
                "name": name,
                "type": "webhook",
                "workflow": workflow_id,
            },
        )
        response.raise_for_status()

        data = response.json()
        return {
            "id": data["id"],
            "url": f"{self.config.url}/api/v1/hooks/webhook_{data['id']}",
        }


# ============================================================================
# Factory Functions
# ============================================================================

def create_phantom_client(config: PhantomConfig) -> PhantomClient:
    return PhantomClient(config)


def create_xsoar_client(config: XsoarConfig) -> XsoarClient:
    return XsoarClient(config)


def create_shuffle_client(config: ShuffleConfig) -> ShuffleClient:
    return ShuffleClient(config)
