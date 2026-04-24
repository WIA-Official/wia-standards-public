"""
WIA Security SIEM Adapters
Splunk, Elastic, QRadar integration
"""

import base64
import json
import asyncio
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any

try:
    import httpx
    HAS_HTTPX = True
except ImportError:
    HAS_HTTPX = False

from ..types import WiaSecurityEvent
from ..converter import to_splunk_event, to_elastic_event


# ============================================================================
# Splunk HEC Adapter
# ============================================================================

@dataclass
class SplunkConfig:
    """Splunk HEC configuration."""
    url: str
    token: str
    index: Optional[str] = None
    source: Optional[str] = None
    sourcetype: Optional[str] = None
    timeout: float = 30.0
    batch_size: int = 100
    flush_interval: float = 5.0


class SplunkAdapter:
    """Adapter for sending events to Splunk via HEC."""

    def __init__(self, config: SplunkConfig):
        if not HAS_HTTPX:
            raise ImportError("httpx package is required. Install with: pip install httpx")

        self.config = config
        self._buffer: List[Dict[str, Any]] = []
        self._client = httpx.Client(timeout=config.timeout)
        self._flush_task: Optional[asyncio.Task] = None

    def close(self) -> None:
        """Close the adapter."""
        self.flush()
        self._client.close()

    def __enter__(self) -> "SplunkAdapter":
        return self

    def __exit__(self, *args) -> None:
        self.close()

    def send(self, event: WiaSecurityEvent) -> None:
        """Send event to Splunk."""
        splunk_event = to_splunk_event(event, self.config.index)

        if self.config.source:
            splunk_event["source"] = self.config.source
        if self.config.sourcetype:
            splunk_event["sourcetype"] = self.config.sourcetype

        self._buffer.append(splunk_event)

        if len(self._buffer) >= self.config.batch_size:
            self.flush()

    def send_batch(self, events: List[WiaSecurityEvent]) -> None:
        """Send multiple events."""
        for event in events:
            self.send(event)

    def flush(self) -> None:
        """Flush buffer to Splunk."""
        if not self._buffer:
            return

        events = self._buffer.copy()
        self._buffer.clear()

        body = "\n".join(json.dumps(e) for e in events)

        response = self._client.post(
            f"{self.config.url}/services/collector/event",
            headers={
                "Authorization": f"Splunk {self.config.token}",
                "Content-Type": "application/json",
            },
            content=body,
        )

        response.raise_for_status()

    def search(
        self,
        query: str,
        earliest: str = "-24h",
        latest: str = "now",
        max_results: Optional[int] = None
    ) -> List[Dict[str, Any]]:
        """Search Splunk."""
        search_query = f"search {query}"
        params = {
            "search": search_query,
            "output_mode": "json",
            "earliest_time": earliest,
            "latest_time": latest,
        }

        if max_results:
            params["max_count"] = str(max_results)

        response = self._client.get(
            f"{self.config.url}/services/search/jobs/export",
            params=params,
            headers={
                "Authorization": f"Splunk {self.config.token}",
            },
        )

        response.raise_for_status()

        lines = response.text.strip().split("\n")
        return [json.loads(line) for line in lines if line]


# ============================================================================
# Elasticsearch Adapter
# ============================================================================

@dataclass
class ElasticConfig:
    """Elasticsearch configuration."""
    url: str
    username: Optional[str] = None
    password: Optional[str] = None
    api_key: Optional[str] = None
    index_prefix: str = "wia-security"
    timeout: float = 30.0
    batch_size: int = 100
    flush_interval: float = 5.0


class ElasticAdapter:
    """Adapter for indexing events to Elasticsearch."""

    def __init__(self, config: ElasticConfig):
        if not HAS_HTTPX:
            raise ImportError("httpx package is required. Install with: pip install httpx")

        self.config = config
        self._buffer: List[Dict[str, Any]] = []

        headers = {"Content-Type": "application/json"}

        if config.api_key:
            headers["Authorization"] = f"ApiKey {config.api_key}"
        elif config.username and config.password:
            auth = base64.b64encode(
                f"{config.username}:{config.password}".encode()
            ).decode()
            headers["Authorization"] = f"Basic {auth}"

        self._client = httpx.Client(timeout=config.timeout, headers=headers)

    def close(self) -> None:
        """Close the adapter."""
        self.flush()
        self._client.close()

    def __enter__(self) -> "ElasticAdapter":
        return self

    def __exit__(self, *args) -> None:
        self.close()

    def index(self, event: WiaSecurityEvent) -> None:
        """Index event."""
        doc = to_elastic_event(event, self.config.index_prefix)
        self._buffer.append(doc)

        if len(self._buffer) >= self.config.batch_size:
            self.flush()

    def index_batch(self, events: List[WiaSecurityEvent]) -> None:
        """Index multiple events."""
        for event in events:
            self.index(event)

    def flush(self) -> None:
        """Flush buffer to Elasticsearch."""
        if not self._buffer:
            return

        docs = self._buffer.copy()
        self._buffer.clear()

        # Build bulk request body
        lines = []
        for doc in docs:
            lines.append(json.dumps({
                "index": {
                    "_index": doc["_index"],
                    "_id": doc["_id"],
                }
            }))
            lines.append(json.dumps(doc["_source"]))

        body = "\n".join(lines) + "\n"

        response = self._client.post(
            f"{self.config.url}/_bulk",
            content=body,
        )

        response.raise_for_status()

        result = response.json()
        if result.get("errors"):
            errors = [
                item for item in result.get("items", [])
                if item.get("index", {}).get("error")
            ]
            if errors:
                print(f"Elasticsearch bulk errors: {errors}")

    def search(
        self,
        query: Dict[str, Any],
        index: Optional[str] = None,
        size: int = 100,
        from_: int = 0
    ) -> Dict[str, Any]:
        """Search events."""
        idx = index or f"{self.config.index_prefix}-*"
        body = {
            "query": query,
            "size": size,
            "from": from_,
            "sort": [{"@timestamp": "desc"}],
        }

        response = self._client.post(
            f"{self.config.url}/{idx}/_search",
            json=body,
        )

        response.raise_for_status()

        result = response.json()
        total = result["hits"]["total"]

        return {
            "hits": [h["_source"] for h in result["hits"]["hits"]],
            "total": total if isinstance(total, int) else total["value"],
        }

    def create_template(self, name: str = "wia-security") -> None:
        """Create index template."""
        template = {
            "index_patterns": [f"{self.config.index_prefix}-*"],
            "template": {
                "settings": {
                    "number_of_shards": 1,
                    "number_of_replicas": 0,
                },
                "mappings": {
                    "properties": {
                        "@timestamp": {"type": "date"},
                        "event.id": {"type": "keyword"},
                        "event.kind": {"type": "keyword"},
                        "event.category": {"type": "keyword"},
                        "event.severity": {"type": "integer"},
                        "wia.version": {"type": "keyword"},
                        "wia.type": {"type": "keyword"},
                        "host.hostname": {"type": "keyword"},
                        "host.ip": {"type": "ip"},
                        "user.name": {"type": "keyword"},
                        "source.ip": {"type": "ip"},
                        "destination.ip": {"type": "ip"},
                        "threat.framework": {"type": "keyword"},
                        "threat.tactic.id": {"type": "keyword"},
                        "threat.technique.id": {"type": "keyword"},
                    }
                }
            }
        }

        response = self._client.put(
            f"{self.config.url}/_index_template/{name}",
            json=template,
        )

        response.raise_for_status()


# ============================================================================
# QRadar Adapter
# ============================================================================

@dataclass
class QRadarConfig:
    """QRadar configuration."""
    url: str
    token: str
    timeout: float = 30.0


class QRadarAdapter:
    """Adapter for sending events to QRadar."""

    def __init__(self, config: QRadarConfig):
        if not HAS_HTTPX:
            raise ImportError("httpx package is required. Install with: pip install httpx")

        self.config = config
        self._client = httpx.Client(
            timeout=config.timeout,
            headers={
                "SEC": config.token,
                "Content-Type": "application/json",
                "Accept": "application/json",
            },
        )

    def close(self) -> None:
        """Close the adapter."""
        self._client.close()

    def __enter__(self) -> "QRadarAdapter":
        return self

    def __exit__(self, *args) -> None:
        self.close()

    def send_syslog(self, event: WiaSecurityEvent) -> None:
        """Send event as syslog (LEEF format)."""
        leef = self._to_leef(event)

        response = self._client.post(
            f"{self.config.url}/api/siem/log_sources/events",
            json={"events": [leef]},
        )

        response.raise_for_status()

    def search(self, aql: str) -> List[Dict[str, Any]]:
        """Search using AQL."""
        # Create search
        create_response = self._client.post(
            f"{self.config.url}/api/ariel/searches",
            json={"query_expression": aql},
        )

        create_response.raise_for_status()
        search_id = create_response.json()["search_id"]

        # Poll for results
        status = "EXECUTE"
        while status in ("EXECUTE", "SORTING"):
            import time
            time.sleep(1)

            status_response = self._client.get(
                f"{self.config.url}/api/ariel/searches/{search_id}"
            )
            status = status_response.json()["status"]

        # Get results
        results_response = self._client.get(
            f"{self.config.url}/api/ariel/searches/{search_id}/results"
        )

        return results_response.json().get("events", [])

    def _to_leef(self, event: WiaSecurityEvent) -> str:
        """Convert WIA event to LEEF format."""
        parts = [
            "LEEF:2.0",
            "WIA",
            "Security",
            "1.0.0",
            event.type.value,
        ]

        attrs = [
            f"devTime={event.timestamp}",
            f"sev={round(event.severity)}",
            f"cat={event.type.value}",
        ]

        if event.context and event.context.host and event.context.host.ip:
            attrs.append(f"src={event.context.host.ip[0]}")

        if event.context and event.context.user and event.context.user.name:
            attrs.append(f"usrName={event.context.user.name}")

        if event.mitre:
            if event.mitre.tactic:
                attrs.append(f"mitreTactic={event.mitre.tactic}")
            if event.mitre.technique:
                attrs.append(f"mitreTechnique={event.mitre.technique}")

        return f"{"|".join(parts)}\t{'\t'.join(attrs)}"


# ============================================================================
# Factory Functions
# ============================================================================

def create_splunk_adapter(config: SplunkConfig) -> SplunkAdapter:
    """Create a Splunk adapter instance."""
    return SplunkAdapter(config)


def create_elastic_adapter(config: ElasticConfig) -> ElasticAdapter:
    """Create an Elasticsearch adapter instance."""
    return ElasticAdapter(config)


def create_qradar_adapter(config: QRadarConfig) -> QRadarAdapter:
    """Create a QRadar adapter instance."""
    return QRadarAdapter(config)
