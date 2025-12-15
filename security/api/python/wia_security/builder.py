"""
WIA Security Event Builder
Fluent API for creating WIA Security events
"""

import uuid
from datetime import datetime
from typing import Any, Dict, List, Optional, TypeVar

from .types import (
    WiaSecurityEvent,
    Source,
    Context,
    Mitre,
    Meta,
    Host,
    User,
    Cloud,
    NetworkContext,
    AlertData,
    ThreatIntelData,
    VulnerabilityData,
    IncidentData,
    NetworkEventData,
    EndpointEventData,
    AuthEventData,
    CVSS,
)

T = TypeVar('T', bound='EventBuilder')


# ============================================================================
# Base Event Builder
# ============================================================================

class EventBuilder:
    """Base builder for WIA Security events"""

    def __init__(self):
        self._event: Dict[str, Any] = {
            "$schema": "https://wia.live/security/v1/schema.json",
            "version": "1.0.0",
            "id": str(uuid.uuid4()),
            "timestamp": datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%S.000Z"),
            "severity": 5,
        }
        self._source: Optional[Source] = None
        self._context: Optional[Context] = None
        self._mitre: Optional[Mitre] = None
        self._meta: Optional[Meta] = None

    def id(self: T, event_id: str) -> T:
        """Set the event ID"""
        self._event["id"] = event_id
        return self

    def timestamp(self: T, ts: str) -> T:
        """Set the timestamp"""
        self._event["timestamp"] = ts
        return self

    def severity(self: T, severity: float) -> T:
        """Set the severity (0-10)"""
        self._event["severity"] = max(0, min(10, severity))
        return self

    def source(self: T, source: Source) -> T:
        """Set the source"""
        self._source = source
        return self

    def context(self: T, context: Context) -> T:
        """Set the context"""
        self._context = context
        return self

    def host(self: T, host: Host) -> T:
        """Set the host context"""
        if self._context is None:
            self._context = Context()
        self._context.host = host
        return self

    def user(self: T, user: User) -> T:
        """Set the user context"""
        if self._context is None:
            self._context = Context()
        self._context.user = user
        return self

    def cloud(self: T, cloud: Cloud) -> T:
        """Set the cloud context"""
        if self._context is None:
            self._context = Context()
        self._context.cloud = cloud
        return self

    def network(self: T, network: NetworkContext) -> T:
        """Set the network context"""
        if self._context is None:
            self._context = Context()
        self._context.network = network
        return self

    def mitre(self: T, mitre: Mitre) -> T:
        """Set MITRE ATT&CK mapping"""
        self._mitre = mitre
        return self

    def tactic(self: T, tactic_id: str, name: Optional[str] = None) -> T:
        """Set MITRE tactic"""
        if self._mitre is None:
            self._mitre = Mitre()
        self._mitre.tactic = tactic_id
        if name:
            self._mitre.tactic_name = name
        return self

    def technique(self: T, technique_id: str, name: Optional[str] = None) -> T:
        """Set MITRE technique"""
        if self._mitre is None:
            self._mitre = Mitre()
        self._mitre.technique = technique_id
        if name:
            self._mitre.technique_name = name
        return self

    def sub_technique(self: T, sub_technique_id: str, name: Optional[str] = None) -> T:
        """Set MITRE sub-technique"""
        if self._mitre is None:
            self._mitre = Mitre()
        self._mitre.sub_technique = sub_technique_id
        if name:
            self._mitre.sub_technique_name = name
        return self

    def meta(self: T, meta: Meta) -> T:
        """Set metadata"""
        self._meta = meta
        return self

    def confidence(self: T, confidence: float) -> T:
        """Set confidence score"""
        if self._meta is None:
            self._meta = Meta()
        self._meta.confidence = max(0, min(1, confidence))
        return self

    def tags(self: T, *tags: str) -> T:
        """Add tags"""
        if self._meta is None:
            self._meta = Meta()
        if self._meta.tags is None:
            self._meta.tags = []
        self._meta.tags.extend(tags)
        return self

    def labels(self: T, labels: Dict[str, str]) -> T:
        """Add labels"""
        if self._meta is None:
            self._meta = Meta()
        if self._meta.labels is None:
            self._meta.labels = {}
        self._meta.labels.update(labels)
        return self

    def correlation_id(self: T, corr_id: str) -> T:
        """Set correlation ID"""
        if self._meta is None:
            self._meta = Meta()
        self._meta.correlation_id = corr_id
        return self

    def raw(self: T, raw_log: str) -> T:
        """Set raw log"""
        if self._meta is None:
            self._meta = Meta()
        self._meta.raw = raw_log
        return self

    def build(self) -> WiaSecurityEvent:
        """Build the event"""
        if self._source is None:
            raise ValueError("Event source is required")
        if "type" not in self._event:
            raise ValueError("Event type is required")
        if "data" not in self._event:
            raise ValueError("Event data is required")

        return WiaSecurityEvent(
            version=self._event["version"],
            id=self._event["id"],
            type=self._event["type"],
            timestamp=self._event["timestamp"],
            severity=self._event["severity"],
            source=self._source,
            data=self._event["data"],
            schema=self._event.get("$schema"),
            context=self._context,
            mitre=self._mitre,
            meta=self._meta,
        )


# ============================================================================
# Alert Builder
# ============================================================================

class AlertBuilder(EventBuilder):
    """Builder for alert events"""

    def __init__(self):
        super().__init__()
        self._event["type"] = "alert"
        self._alert_data: Dict[str, Any] = {}

    def data(self, data: AlertData) -> "AlertBuilder":
        """Set alert data"""
        self._event["data"] = data.to_dict()
        return self

    def alert_id(self, alert_id: str) -> "AlertBuilder":
        """Set alert ID"""
        self._alert_data["alert_id"] = alert_id
        return self

    def title(self, title: str) -> "AlertBuilder":
        """Set alert title"""
        self._alert_data["title"] = title
        return self

    def description(self, description: str) -> "AlertBuilder":
        """Set alert description"""
        self._alert_data["description"] = description
        return self

    def category(self, category: str) -> "AlertBuilder":
        """Set alert category"""
        self._alert_data["category"] = category
        return self

    def status(self, status: str) -> "AlertBuilder":
        """Set alert status"""
        self._alert_data["status"] = status
        return self

    def priority(self, priority: str) -> "AlertBuilder":
        """Set alert priority"""
        self._alert_data["priority"] = priority
        return self

    def detection_rule(self, rule_id: str, name: str, version: Optional[str] = None) -> "AlertBuilder":
        """Set detection rule"""
        self._alert_data["detection_rule"] = {"id": rule_id, "name": name}
        if version:
            self._alert_data["detection_rule"]["version"] = version
        return self

    def add_indicator(self, indicator_type: str, value: str, context: Optional[str] = None) -> "AlertBuilder":
        """Add indicator"""
        if "indicators" not in self._alert_data:
            self._alert_data["indicators"] = []
        indicator = {"type": indicator_type, "value": value}
        if context:
            indicator["context"] = context
        self._alert_data["indicators"].append(indicator)
        return self

    def build(self) -> WiaSecurityEvent:
        """Build the event"""
        if self._alert_data:
            self._event["data"] = self._alert_data
        return super().build()


# ============================================================================
# Threat Intel Builder
# ============================================================================

class ThreatIntelBuilder(EventBuilder):
    """Builder for threat intelligence events"""

    def __init__(self):
        super().__init__()
        self._event["type"] = "threat_intel"
        self._threat_data: Dict[str, Any] = {}

    def data(self, data: ThreatIntelData) -> "ThreatIntelBuilder":
        """Set threat intel data"""
        self._event["data"] = data.to_dict()
        return self

    def threat_type(self, threat_type: str) -> "ThreatIntelBuilder":
        """Set threat type"""
        self._threat_data["threat_type"] = threat_type
        return self

    def threat_name(self, name: str) -> "ThreatIntelBuilder":
        """Set threat name"""
        self._threat_data["threat_name"] = name
        return self

    def threat_family(self, family: str) -> "ThreatIntelBuilder":
        """Set threat family"""
        self._threat_data["threat_family"] = family
        return self

    def status(self, status: str) -> "ThreatIntelBuilder":
        """Set threat status"""
        self._threat_data["status"] = status
        return self

    def target_sectors(self, *sectors: str) -> "ThreatIntelBuilder":
        """Set target sectors"""
        self._threat_data["target_sectors"] = list(sectors)
        return self

    def target_countries(self, *countries: str) -> "ThreatIntelBuilder":
        """Set target countries"""
        self._threat_data["target_countries"] = list(countries)
        return self

    def build(self) -> WiaSecurityEvent:
        """Build the event"""
        if self._threat_data:
            self._event["data"] = self._threat_data
        return super().build()


# ============================================================================
# Vulnerability Builder
# ============================================================================

class VulnerabilityBuilder(EventBuilder):
    """Builder for vulnerability events"""

    def __init__(self):
        super().__init__()
        self._event["type"] = "vulnerability"
        self._vuln_data: Dict[str, Any] = {}

    def data(self, data: VulnerabilityData) -> "VulnerabilityBuilder":
        """Set vulnerability data"""
        self._event["data"] = data.to_dict()
        return self

    def vuln_id(self, vuln_id: str) -> "VulnerabilityBuilder":
        """Set vulnerability ID (CVE)"""
        self._vuln_data["vuln_id"] = vuln_id
        return self

    def title(self, title: str) -> "VulnerabilityBuilder":
        """Set vulnerability title"""
        self._vuln_data["title"] = title
        return self

    def cvss(self, cvss: CVSS) -> "VulnerabilityBuilder":
        """Set CVSS"""
        self._vuln_data["cvss"] = cvss.to_dict()
        return self

    def cwe(self, *cwe_ids: str) -> "VulnerabilityBuilder":
        """Set CWE IDs"""
        self._vuln_data["cwe"] = list(cwe_ids)
        return self

    def exploit_available(self, available: bool) -> "VulnerabilityBuilder":
        """Set exploit availability"""
        self._vuln_data["exploit_available"] = available
        return self

    def patch_available(self, available: bool) -> "VulnerabilityBuilder":
        """Set patch availability"""
        self._vuln_data["patch_available"] = available
        return self

    def build(self) -> WiaSecurityEvent:
        """Build the event"""
        if self._vuln_data:
            self._event["data"] = self._vuln_data
        return super().build()


# ============================================================================
# Incident Builder
# ============================================================================

class IncidentBuilder(EventBuilder):
    """Builder for incident events"""

    def __init__(self):
        super().__init__()
        self._event["type"] = "incident"
        self._incident_data: Dict[str, Any] = {}

    def data(self, data: IncidentData) -> "IncidentBuilder":
        """Set incident data"""
        self._event["data"] = data.to_dict()
        return self

    def incident_id(self, incident_id: str) -> "IncidentBuilder":
        """Set incident ID"""
        self._incident_data["incident_id"] = incident_id
        return self

    def title(self, title: str) -> "IncidentBuilder":
        """Set incident title"""
        self._incident_data["title"] = title
        return self

    def category(self, category: str) -> "IncidentBuilder":
        """Set incident category"""
        self._incident_data["category"] = category
        return self

    def status(self, status: str) -> "IncidentBuilder":
        """Set incident status"""
        self._incident_data["status"] = status
        return self

    def priority(self, priority: str) -> "IncidentBuilder":
        """Set incident priority"""
        self._incident_data["priority"] = priority
        return self

    def impact(self, impact: Dict[str, str]) -> "IncidentBuilder":
        """Set impact"""
        self._incident_data["impact"] = impact
        return self

    def lead_analyst(self, analyst: str) -> "IncidentBuilder":
        """Set lead analyst"""
        self._incident_data["lead_analyst"] = analyst
        return self

    def team(self, *members: str) -> "IncidentBuilder":
        """Set team members"""
        self._incident_data["team"] = list(members)
        return self

    def build(self) -> WiaSecurityEvent:
        """Build the event"""
        if self._incident_data:
            self._event["data"] = self._incident_data
        return super().build()


# ============================================================================
# Network Event Builder
# ============================================================================

class NetworkEventBuilder(EventBuilder):
    """Builder for network events"""

    def __init__(self):
        super().__init__()
        self._event["type"] = "network_event"
        self._network_data: Dict[str, Any] = {}

    def data(self, data: NetworkEventData) -> "NetworkEventBuilder":
        """Set network event data"""
        self._event["data"] = data.to_dict()
        return self

    def event_type(self, event_type: str) -> "NetworkEventBuilder":
        """Set network event type"""
        self._network_data["event_type"] = event_type
        return self

    def protocol(self, protocol: str) -> "NetworkEventBuilder":
        """Set protocol"""
        self._network_data["protocol"] = protocol
        return self

    def direction(self, direction: str) -> "NetworkEventBuilder":
        """Set direction"""
        self._network_data["direction"] = direction
        return self

    def action(self, action: str) -> "NetworkEventBuilder":
        """Set action"""
        self._network_data["action"] = action
        return self

    def source_host(self, host: Dict[str, Any]) -> "NetworkEventBuilder":
        """Set source host"""
        self._network_data["source"] = host
        return self

    def dest_host(self, host: Dict[str, Any]) -> "NetworkEventBuilder":
        """Set destination host"""
        self._network_data["destination"] = host
        return self

    def build(self) -> WiaSecurityEvent:
        """Build the event"""
        if self._network_data:
            self._event["data"] = self._network_data
        return super().build()


# ============================================================================
# Endpoint Event Builder
# ============================================================================

class EndpointEventBuilder(EventBuilder):
    """Builder for endpoint events"""

    def __init__(self):
        super().__init__()
        self._event["type"] = "endpoint_event"
        self._endpoint_data: Dict[str, Any] = {}

    def data(self, data: EndpointEventData) -> "EndpointEventBuilder":
        """Set endpoint event data"""
        self._event["data"] = data.to_dict()
        return self

    def event_type(self, event_type: str) -> "EndpointEventBuilder":
        """Set endpoint event type"""
        self._endpoint_data["event_type"] = event_type
        return self

    def host(self, host: Dict[str, Any]) -> "EndpointEventBuilder":
        """Set host"""
        self._endpoint_data["host"] = host
        return self

    def process(self, process: Dict[str, Any]) -> "EndpointEventBuilder":
        """Set process info"""
        self._endpoint_data["process"] = process
        return self

    def file(self, file_info: Dict[str, Any]) -> "EndpointEventBuilder":
        """Set file info"""
        self._endpoint_data["file"] = file_info
        return self

    def registry(self, registry: Dict[str, Any]) -> "EndpointEventBuilder":
        """Set registry info"""
        self._endpoint_data["registry"] = registry
        return self

    def build(self) -> WiaSecurityEvent:
        """Build the event"""
        if self._endpoint_data:
            self._event["data"] = self._endpoint_data
        return super().build()


# ============================================================================
# Auth Event Builder
# ============================================================================

class AuthEventBuilder(EventBuilder):
    """Builder for authentication events"""

    def __init__(self):
        super().__init__()
        self._event["type"] = "auth_event"
        self._auth_data: Dict[str, Any] = {}

    def data(self, data: AuthEventData) -> "AuthEventBuilder":
        """Set auth event data"""
        self._event["data"] = data.to_dict()
        return self

    def event_type(self, event_type: str) -> "AuthEventBuilder":
        """Set auth event type"""
        self._auth_data["event_type"] = event_type
        return self

    def result(self, result: str) -> "AuthEventBuilder":
        """Set result"""
        self._auth_data["result"] = result
        return self

    def auth_user(self, user: Dict[str, Any]) -> "AuthEventBuilder":
        """Set user"""
        self._auth_data["user"] = user
        return self

    def target(self, target: Dict[str, Any]) -> "AuthEventBuilder":
        """Set target"""
        self._auth_data["target"] = target
        return self

    def auth_method(self, method: str) -> "AuthEventBuilder":
        """Set auth method"""
        self._auth_data["auth_method"] = method
        return self

    def risk_score(self, score: float) -> "AuthEventBuilder":
        """Set risk score"""
        self._auth_data["risk_score"] = max(0, min(1, score))
        return self

    def risk_factors(self, *factors: str) -> "AuthEventBuilder":
        """Set risk factors"""
        self._auth_data["risk_factors"] = list(factors)
        return self

    def build(self) -> WiaSecurityEvent:
        """Build the event"""
        if self._auth_data:
            self._event["data"] = self._auth_data
        return super().build()


# ============================================================================
# Factory Functions
# ============================================================================

def create_alert() -> AlertBuilder:
    """Create an alert builder"""
    return AlertBuilder()


def create_threat_intel() -> ThreatIntelBuilder:
    """Create a threat intel builder"""
    return ThreatIntelBuilder()


def create_vulnerability() -> VulnerabilityBuilder:
    """Create a vulnerability builder"""
    return VulnerabilityBuilder()


def create_incident() -> IncidentBuilder:
    """Create an incident builder"""
    return IncidentBuilder()


def create_network_event() -> NetworkEventBuilder:
    """Create a network event builder"""
    return NetworkEventBuilder()


def create_endpoint_event() -> EndpointEventBuilder:
    """Create an endpoint event builder"""
    return EndpointEventBuilder()


def create_auth_event() -> AuthEventBuilder:
    """Create an auth event builder"""
    return AuthEventBuilder()
