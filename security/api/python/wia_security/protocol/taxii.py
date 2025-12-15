"""
WIA Security TAXII 2.1 Client
Threat Intelligence Sharing Protocol
"""

import base64
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any

try:
    import httpx
    HAS_HTTPX = True
except ImportError:
    HAS_HTTPX = False

from ..types import WiaSecurityEvent
from ..converter import to_stix_bundle


# ============================================================================
# Types
# ============================================================================

@dataclass
class TaxiiConfig:
    """TAXII 2.1 client configuration."""
    server_url: str
    api_root: Optional[str] = None
    username: Optional[str] = None
    password: Optional[str] = None
    api_key: Optional[str] = None
    timeout: float = 30.0


@dataclass
class TaxiiDiscovery:
    """TAXII server discovery information."""
    title: str
    description: Optional[str] = None
    contact: Optional[str] = None
    default: Optional[str] = None
    api_roots: Optional[List[str]] = None

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TaxiiDiscovery":
        return cls(
            title=data["title"],
            description=data.get("description"),
            contact=data.get("contact"),
            default=data.get("default"),
            api_roots=data.get("api_roots"),
        )


@dataclass
class TaxiiApiRoot:
    """TAXII API root information."""
    title: str
    versions: List[str]
    max_content_length: int
    description: Optional[str] = None

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TaxiiApiRoot":
        return cls(
            title=data["title"],
            versions=data["versions"],
            max_content_length=data["max_content_length"],
            description=data.get("description"),
        )


@dataclass
class TaxiiCollection:
    """TAXII collection information."""
    id: str
    title: str
    can_read: bool
    can_write: bool
    description: Optional[str] = None
    media_types: Optional[List[str]] = None

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TaxiiCollection":
        return cls(
            id=data["id"],
            title=data["title"],
            can_read=data["can_read"],
            can_write=data["can_write"],
            description=data.get("description"),
            media_types=data.get("media_types"),
        )


@dataclass
class TaxiiStatus:
    """TAXII add objects status."""
    id: str
    status: str  # pending, complete, failed
    request_timestamp: Optional[str] = None
    total_count: Optional[int] = None
    success_count: Optional[int] = None
    failure_count: Optional[int] = None
    pending_count: Optional[int] = None
    failures: Optional[List[Dict[str, str]]] = None

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TaxiiStatus":
        return cls(
            id=data["id"],
            status=data["status"],
            request_timestamp=data.get("request_timestamp"),
            total_count=data.get("total_count"),
            success_count=data.get("success_count"),
            failure_count=data.get("failure_count"),
            pending_count=data.get("pending_count"),
            failures=data.get("failures"),
        )


@dataclass
class TaxiiManifestEntry:
    """TAXII manifest entry."""
    id: str
    date_added: str
    version: Optional[str] = None
    media_type: Optional[str] = None

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TaxiiManifestEntry":
        return cls(
            id=data["id"],
            date_added=data["date_added"],
            version=data.get("version"),
            media_type=data.get("media_type"),
        )


# ============================================================================
# TAXII Client
# ============================================================================

class TaxiiClient:
    """TAXII 2.1 client for threat intelligence sharing."""

    def __init__(self, config: TaxiiConfig):
        if not HAS_HTTPX:
            raise ImportError("httpx package is required. Install with: pip install httpx")

        self.config = config
        self._headers = {
            "Accept": "application/taxii+json;version=2.1",
            "Content-Type": "application/taxii+json;version=2.1",
        }

        if config.api_key:
            self._headers["Authorization"] = f"Bearer {config.api_key}"
        elif config.username and config.password:
            auth = base64.b64encode(
                f"{config.username}:{config.password}".encode()
            ).decode()
            self._headers["Authorization"] = f"Basic {auth}"

        self._client = httpx.Client(
            timeout=config.timeout,
            headers=self._headers,
        )

    def close(self) -> None:
        """Close the client."""
        self._client.close()

    def __enter__(self) -> "TaxiiClient":
        return self

    def __exit__(self, *args) -> None:
        self.close()

    def discover(self) -> TaxiiDiscovery:
        """Discover TAXII server."""
        response = self._request("GET", "/taxii2/")
        return TaxiiDiscovery.from_dict(response)

    def get_api_root(self, api_root: Optional[str] = None) -> TaxiiApiRoot:
        """Get API root information."""
        root = api_root or self.config.api_root or ""
        response = self._request("GET", f"/{root}/")
        return TaxiiApiRoot.from_dict(response)

    def get_collections(self, api_root: Optional[str] = None) -> List[TaxiiCollection]:
        """List collections."""
        root = api_root or self.config.api_root or ""
        response = self._request("GET", f"/{root}/collections/")
        return [
            TaxiiCollection.from_dict(c)
            for c in response.get("collections", [])
        ]

    def get_collection(
        self,
        collection_id: str,
        api_root: Optional[str] = None
    ) -> TaxiiCollection:
        """Get collection by ID."""
        root = api_root or self.config.api_root or ""
        response = self._request("GET", f"/{root}/collections/{collection_id}/")
        return TaxiiCollection.from_dict(response)

    def get_manifest(
        self,
        collection_id: str,
        added_after: Optional[str] = None,
        limit: Optional[int] = None,
        next_token: Optional[str] = None,
        api_root: Optional[str] = None
    ) -> Dict[str, Any]:
        """Get collection manifest."""
        root = api_root or self.config.api_root or ""
        params = {}

        if added_after:
            params["added_after"] = added_after
        if limit:
            params["limit"] = str(limit)
        if next_token:
            params["next"] = next_token

        response = self._request(
            "GET",
            f"/{root}/collections/{collection_id}/manifest/",
            params=params
        )

        return {
            "objects": [TaxiiManifestEntry.from_dict(e) for e in response.get("objects", [])],
            "more": response.get("more"),
            "next": response.get("next"),
        }

    def get_objects(
        self,
        collection_id: str,
        added_after: Optional[str] = None,
        limit: Optional[int] = None,
        next_token: Optional[str] = None,
        types: Optional[List[str]] = None,
        ids: Optional[List[str]] = None,
        api_root: Optional[str] = None
    ) -> Dict[str, Any]:
        """Get objects from collection."""
        root = api_root or self.config.api_root or ""
        params = {}

        if added_after:
            params["added_after"] = added_after
        if limit:
            params["limit"] = str(limit)
        if next_token:
            params["next"] = next_token
        if types:
            params["type"] = types
        if ids:
            params["match[id]"] = ids

        response = self._request(
            "GET",
            f"/{root}/collections/{collection_id}/objects/",
            params=params
        )

        return response

    def get_object(
        self,
        collection_id: str,
        object_id: str,
        api_root: Optional[str] = None
    ) -> Dict[str, Any]:
        """Get object by ID."""
        root = api_root or self.config.api_root or ""
        return self._request(
            "GET",
            f"/{root}/collections/{collection_id}/objects/{object_id}/"
        )

    def add_objects(
        self,
        collection_id: str,
        bundle: Dict[str, Any],
        api_root: Optional[str] = None
    ) -> TaxiiStatus:
        """Add objects to collection."""
        root = api_root or self.config.api_root or ""
        response = self._request(
            "POST",
            f"/{root}/collections/{collection_id}/objects/",
            json=bundle
        )
        return TaxiiStatus.from_dict(response)

    def add_wia_event(
        self,
        collection_id: str,
        event: WiaSecurityEvent,
        api_root: Optional[str] = None
    ) -> TaxiiStatus:
        """Add WIA Security event to collection (converts to STIX)."""
        bundle = to_stix_bundle(event)
        return self.add_objects(collection_id, bundle, api_root)

    def add_wia_events(
        self,
        collection_id: str,
        events: List[WiaSecurityEvent],
        api_root: Optional[str] = None
    ) -> List[TaxiiStatus]:
        """Add multiple WIA Security events to collection."""
        results = []
        for event in events:
            status = self.add_wia_event(collection_id, event, api_root)
            results.append(status)
        return results

    def get_status(
        self,
        status_id: str,
        api_root: Optional[str] = None
    ) -> TaxiiStatus:
        """Get status of add request."""
        root = api_root or self.config.api_root or ""
        response = self._request("GET", f"/{root}/status/{status_id}/")
        return TaxiiStatus.from_dict(response)

    def delete_object(
        self,
        collection_id: str,
        object_id: str,
        api_root: Optional[str] = None
    ) -> None:
        """Delete object from collection."""
        root = api_root or self.config.api_root or ""
        self._request(
            "DELETE",
            f"/{root}/collections/{collection_id}/objects/{object_id}/"
        )

    # -------------------------------------------------------------------------
    # Private Methods
    # -------------------------------------------------------------------------

    def _request(
        self,
        method: str,
        path: str,
        params: Optional[Dict[str, Any]] = None,
        json: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """Make HTTP request."""
        url = f"{self.config.server_url}{path}"

        response = self._client.request(
            method,
            url,
            params=params,
            json=json
        )

        response.raise_for_status()

        content_type = response.headers.get("content-type", "")
        if "application/taxii+json" in content_type or "application/json" in content_type:
            return response.json()

        return {"text": response.text}


# ============================================================================
# Factory
# ============================================================================

def create_taxii_client(config: TaxiiConfig) -> TaxiiClient:
    """Create a TAXII client instance."""
    return TaxiiClient(config)
