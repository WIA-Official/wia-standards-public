"""
WIA Security Client
Main client for interacting with WIA Security events
"""

import json
import uuid
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Callable, Dict, List, Optional, Union

from .types import WiaSecurityEvent, Source
from .validator import validate_event, ValidationResult
from .builder import (
    create_alert,
    create_threat_intel,
    create_vulnerability,
    create_incident,
    create_network_event,
    create_endpoint_event,
    create_auth_event,
    AlertBuilder,
    ThreatIntelBuilder,
    VulnerabilityBuilder,
    IncidentBuilder,
    NetworkEventBuilder,
    EndpointEventBuilder,
    AuthEventBuilder,
)
from .converter import (
    to_stix_bundle,
    to_ecs_event,
    to_ocsf_event,
    to_splunk_event,
    to_elastic_event,
)


# ============================================================================
# Client Configuration
# ============================================================================

@dataclass
class SecurityClientConfig:
    """Configuration for SecurityClient"""
    default_source: Optional[Source] = None
    auto_generate_ids: bool = True
    auto_timestamp: bool = True
    validate_on_create: bool = True
    on_event_created: Optional[Callable[[WiaSecurityEvent], None]] = None
    on_validation_error: Optional[Callable[[List[str]], None]] = None


# ============================================================================
# Event Filter
# ============================================================================

@dataclass
class EventFilter:
    """Filter for querying events"""
    type: Optional[Union[str, List[str]]] = None
    severity_min: Optional[float] = None
    severity_max: Optional[float] = None
    start_time: Optional[str] = None
    end_time: Optional[str] = None
    tags: Optional[List[str]] = None
    limit: Optional[int] = None
    offset: Optional[int] = None


# ============================================================================
# Event Store
# ============================================================================

class InMemoryStore:
    """In-memory event storage"""

    def __init__(self):
        self._events: Dict[str, WiaSecurityEvent] = {}

    def save(self, event: WiaSecurityEvent) -> None:
        """Save an event"""
        self._events[event.id] = event

    def get(self, event_id: str) -> Optional[WiaSecurityEvent]:
        """Get an event by ID"""
        return self._events.get(event_id)

    def query(self, filter: EventFilter) -> List[WiaSecurityEvent]:
        """Query events"""
        results = list(self._events.values())

        # Filter by type
        if filter.type:
            types = [filter.type] if isinstance(filter.type, str) else filter.type
            results = [e for e in results if e.type in types]

        # Filter by severity
        if filter.severity_min is not None:
            results = [e for e in results if e.severity >= filter.severity_min]
        if filter.severity_max is not None:
            results = [e for e in results if e.severity <= filter.severity_max]

        # Filter by time
        if filter.start_time:
            results = [e for e in results if e.timestamp >= filter.start_time]
        if filter.end_time:
            results = [e for e in results if e.timestamp <= filter.end_time]

        # Filter by tags
        if filter.tags:
            def has_tags(event: WiaSecurityEvent) -> bool:
                if event.meta and event.meta.tags:
                    return any(tag in event.meta.tags for tag in filter.tags)
                return False
            results = [e for e in results if has_tags(e)]

        # Sort by timestamp descending
        results.sort(key=lambda e: e.timestamp, reverse=True)

        # Apply pagination
        if filter.offset:
            results = results[filter.offset:]
        if filter.limit:
            results = results[:filter.limit]

        return results

    def delete(self, event_id: str) -> bool:
        """Delete an event"""
        if event_id in self._events:
            del self._events[event_id]
            return True
        return False


# ============================================================================
# Security Client
# ============================================================================

class SecurityClient:
    """
    WIA Security Client.

    Main client for creating, validating, and managing WIA Security events.
    """

    def __init__(self, config: Optional[SecurityClientConfig] = None, store: Optional[InMemoryStore] = None):
        self.config = config or SecurityClientConfig()
        self._store = store or InMemoryStore()

    # -------------------------------------------------------------------------
    # Event Creation
    # -------------------------------------------------------------------------

    def alert(self) -> AlertBuilder:
        """Create an alert builder"""
        builder = create_alert()
        if self.config.default_source:
            builder.source(self.config.default_source)
        return builder

    def threat_intel(self) -> ThreatIntelBuilder:
        """Create a threat intel builder"""
        builder = create_threat_intel()
        if self.config.default_source:
            builder.source(self.config.default_source)
        return builder

    def vulnerability(self) -> VulnerabilityBuilder:
        """Create a vulnerability builder"""
        builder = create_vulnerability()
        if self.config.default_source:
            builder.source(self.config.default_source)
        return builder

    def incident(self) -> IncidentBuilder:
        """Create an incident builder"""
        builder = create_incident()
        if self.config.default_source:
            builder.source(self.config.default_source)
        return builder

    def network_event(self) -> NetworkEventBuilder:
        """Create a network event builder"""
        builder = create_network_event()
        if self.config.default_source:
            builder.source(self.config.default_source)
        return builder

    def endpoint_event(self) -> EndpointEventBuilder:
        """Create an endpoint event builder"""
        builder = create_endpoint_event()
        if self.config.default_source:
            builder.source(self.config.default_source)
        return builder

    def auth_event(self) -> AuthEventBuilder:
        """Create an auth event builder"""
        builder = create_auth_event()
        if self.config.default_source:
            builder.source(self.config.default_source)
        return builder

    # -------------------------------------------------------------------------
    # Event Operations
    # -------------------------------------------------------------------------

    def validate(self, event: Union[WiaSecurityEvent, Dict[str, Any]]) -> ValidationResult:
        """Validate an event"""
        return validate_event(event)

    def save(self, event: WiaSecurityEvent) -> WiaSecurityEvent:
        """Save an event to the store"""
        # Auto-generate ID if needed
        if self.config.auto_generate_ids and not event.id:
            event.id = str(uuid.uuid4())

        # Auto-set timestamp if needed
        if self.config.auto_timestamp and not event.timestamp:
            event.timestamp = datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%S.000Z")

        # Validate if configured
        if self.config.validate_on_create:
            result = self.validate(event)
            if not result.valid:
                if self.config.on_validation_error:
                    self.config.on_validation_error(result.errors)
                raise ValueError(f"Validation failed: {', '.join(result.errors)}")

        self._store.save(event)

        if self.config.on_event_created:
            self.config.on_event_created(event)

        return event

    def get(self, event_id: str) -> Optional[WiaSecurityEvent]:
        """Get an event by ID"""
        return self._store.get(event_id)

    def query(self, filter: Optional[EventFilter] = None) -> List[WiaSecurityEvent]:
        """Query events"""
        return self._store.query(filter or EventFilter())

    def delete(self, event_id: str) -> bool:
        """Delete an event"""
        return self._store.delete(event_id)

    def get_alerts(self, **kwargs) -> List[WiaSecurityEvent]:
        """Get all alerts"""
        return self.query(EventFilter(type="alert", **kwargs))

    def get_threat_intel(self, **kwargs) -> List[WiaSecurityEvent]:
        """Get all threat intelligence"""
        return self.query(EventFilter(type="threat_intel", **kwargs))

    def get_vulnerabilities(self, **kwargs) -> List[WiaSecurityEvent]:
        """Get all vulnerabilities"""
        return self.query(EventFilter(type="vulnerability", **kwargs))

    def get_incidents(self, **kwargs) -> List[WiaSecurityEvent]:
        """Get all incidents"""
        return self.query(EventFilter(type="incident", **kwargs))

    def get_high_severity(self, min_severity: float = 7) -> List[WiaSecurityEvent]:
        """Get high severity events"""
        return self.query(EventFilter(severity_min=min_severity))

    # -------------------------------------------------------------------------
    # Export/Convert
    # -------------------------------------------------------------------------

    def to_stix(self, event: Union[WiaSecurityEvent, Dict[str, Any]]) -> Dict[str, Any]:
        """Export event to STIX 2.1 bundle"""
        return to_stix_bundle(event)

    def to_ecs(self, event: Union[WiaSecurityEvent, Dict[str, Any]]) -> Dict[str, Any]:
        """Export event to ECS format"""
        return to_ecs_event(event)

    def to_ocsf(self, event: Union[WiaSecurityEvent, Dict[str, Any]]) -> Dict[str, Any]:
        """Export event to OCSF format"""
        return to_ocsf_event(event)

    def to_splunk(self, event: Union[WiaSecurityEvent, Dict[str, Any]], index: Optional[str] = None) -> Dict[str, Any]:
        """Export event to Splunk HEC format"""
        return to_splunk_event(event, index)

    def to_elastic(self, event: Union[WiaSecurityEvent, Dict[str, Any]], index_prefix: str = "wia-security") -> Dict[str, Any]:
        """Export event to Elasticsearch document"""
        return to_elastic_event(event, index_prefix)

    def export_json(self, events: List[WiaSecurityEvent]) -> str:
        """Export multiple events to JSON"""
        return json.dumps([e.to_dict() for e in events], indent=2)

    def import_json(self, json_str: str) -> List[WiaSecurityEvent]:
        """Import events from JSON"""
        data = json.loads(json_str)
        events = []
        for item in data:
            event = WiaSecurityEvent.from_dict(item)
            events.append(self.save(event))
        return events

    # -------------------------------------------------------------------------
    # Statistics
    # -------------------------------------------------------------------------

    def get_stats(self) -> Dict[str, Any]:
        """Get event statistics"""
        events = self.query()

        by_type: Dict[str, int] = {}
        by_severity = {
            "info": 0,
            "low": 0,
            "medium": 0,
            "high": 0,
            "critical": 0
        }
        total_severity = 0

        for event in events:
            # Count by type
            by_type[event.type] = by_type.get(event.type, 0) + 1

            # Count by severity
            total_severity += event.severity
            if event.severity <= 2:
                by_severity["info"] += 1
            elif event.severity <= 4:
                by_severity["low"] += 1
            elif event.severity <= 6:
                by_severity["medium"] += 1
            elif event.severity <= 8:
                by_severity["high"] += 1
            else:
                by_severity["critical"] += 1

        return {
            "total": len(events),
            "by_type": by_type,
            "by_severity": by_severity,
            "avg_severity": total_severity / len(events) if events else 0
        }


# ============================================================================
# Factory Function
# ============================================================================

def create_client(config: Optional[SecurityClientConfig] = None, store: Optional[InMemoryStore] = None) -> SecurityClient:
    """Create a new SecurityClient"""
    return SecurityClient(config, store)
