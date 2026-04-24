"""
WIA Security Event Types
Core type definitions for the WIA Security Standard
"""

from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Any, Dict, List, Literal, Optional, TypedDict, Union
import uuid


# ============================================================================
# Enums
# ============================================================================

class EventType(str, Enum):
    ALERT = "alert"
    THREAT_INTEL = "threat_intel"
    VULNERABILITY = "vulnerability"
    INCIDENT = "incident"
    NETWORK_EVENT = "network_event"
    ENDPOINT_EVENT = "endpoint_event"
    AUTH_EVENT = "auth_event"


class SourceType(str, Enum):
    SIEM = "siem"
    EDR = "edr"
    IDS = "ids"
    FIREWALL = "firewall"
    SCANNER = "scanner"
    CUSTOM = "custom"


class Priority(str, Enum):
    CRITICAL = "critical"
    HIGH = "high"
    MEDIUM = "medium"
    LOW = "low"
    INFO = "info"


class Direction(str, Enum):
    INBOUND = "inbound"
    OUTBOUND = "outbound"
    INTERNAL = "internal"


class CloudProvider(str, Enum):
    AWS = "aws"
    AZURE = "azure"
    GCP = "gcp"


class AlertCategory(str, Enum):
    MALWARE = "malware"
    INTRUSION = "intrusion"
    POLICY = "policy"
    RECONNAISSANCE = "reconnaissance"
    OTHER = "other"


class AlertStatus(str, Enum):
    NEW = "new"
    INVESTIGATING = "investigating"
    RESOLVED = "resolved"
    FALSE_POSITIVE = "false_positive"
    CLOSED = "closed"


class ThreatType(str, Enum):
    MALWARE = "malware"
    APT = "apt"
    CAMPAIGN = "campaign"
    BOTNET = "botnet"
    RANSOMWARE = "ransomware"
    PHISHING = "phishing"


class ThreatStatus(str, Enum):
    ACTIVE = "active"
    INACTIVE = "inactive"
    UNKNOWN = "unknown"


class CvssSeverity(str, Enum):
    CRITICAL = "critical"
    HIGH = "high"
    MEDIUM = "medium"
    LOW = "low"
    NONE = "none"


class IncidentCategory(str, Enum):
    MALWARE = "malware"
    PHISHING = "phishing"
    RANSOMWARE = "ransomware"
    DATA_BREACH = "data_breach"
    DDOS = "ddos"
    UNAUTHORIZED_ACCESS = "unauthorized_access"
    INSIDER_THREAT = "insider_threat"
    APT = "apt"
    OTHER = "other"


class IncidentStatus(str, Enum):
    NEW = "new"
    TRIAGING = "triaging"
    INVESTIGATING = "investigating"
    CONTAINING = "containing"
    ERADICATING = "eradicating"
    RECOVERING = "recovering"
    CLOSED = "closed"


class ImpactLevel(str, Enum):
    NONE = "none"
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"


class NetworkEventType(str, Enum):
    CONNECTION = "connection"
    DNS = "dns"
    HTTP = "http"
    TLS = "tls"
    SMTP = "smtp"
    FTP = "ftp"
    SSH = "ssh"
    RDP = "rdp"
    CUSTOM = "custom"


class NetworkProtocol(str, Enum):
    TCP = "TCP"
    UDP = "UDP"
    ICMP = "ICMP"
    GRE = "GRE"
    ESP = "ESP"


class NetworkAction(str, Enum):
    ALLOWED = "allowed"
    BLOCKED = "blocked"
    DROPPED = "dropped"
    RESET = "reset"


class EndpointEventType(str, Enum):
    PROCESS_CREATION = "process_creation"
    PROCESS_TERMINATION = "process_termination"
    FILE_CREATE = "file_create"
    FILE_MODIFY = "file_modify"
    FILE_DELETE = "file_delete"
    REGISTRY_CREATE = "registry_create"
    REGISTRY_MODIFY = "registry_modify"
    REGISTRY_DELETE = "registry_delete"
    NETWORK_CONNECTION = "network_connection"
    DLL_LOAD = "dll_load"
    DRIVER_LOAD = "driver_load"
    SERVICE_INSTALL = "service_install"


class AuthEventType(str, Enum):
    LOGIN_SUCCESS = "login_success"
    LOGIN_FAILURE = "login_failure"
    LOGOUT = "logout"
    PASSWORD_CHANGE = "password_change"
    ACCOUNT_LOCKED = "account_locked"
    ACCOUNT_UNLOCKED = "account_unlocked"
    MFA_SUCCESS = "mfa_success"
    MFA_FAILURE = "mfa_failure"
    PRIVILEGE_ESCALATION = "privilege_escalation"


class AuthResult(str, Enum):
    SUCCESS = "success"
    FAILURE = "failure"


class AuthMethod(str, Enum):
    PASSWORD = "password"
    SSO = "sso"
    CERTIFICATE = "certificate"
    BIOMETRIC = "biometric"
    MFA = "mfa"
    API_KEY = "api_key"
    OAUTH = "oauth"


# ============================================================================
# Data Classes
# ============================================================================

@dataclass
class Source:
    type: str
    name: str
    vendor: Optional[str] = None
    version: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        result = {"type": self.type, "name": self.name}
        if self.vendor:
            result["vendor"] = self.vendor
        if self.version:
            result["version"] = self.version
        return result


@dataclass
class OperatingSystem:
    name: Optional[str] = None
    version: Optional[str] = None
    build: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {k: v for k, v in {
            "name": self.name,
            "version": self.version,
            "build": self.build
        }.items() if v is not None}


@dataclass
class Host:
    hostname: Optional[str] = None
    ip: Optional[List[str]] = None
    mac: Optional[List[str]] = None
    os: Optional[OperatingSystem] = None
    domain: Optional[str] = None
    agent_id: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {}
        if self.hostname:
            result["hostname"] = self.hostname
        if self.ip:
            result["ip"] = self.ip
        if self.mac:
            result["mac"] = self.mac
        if self.os:
            result["os"] = self.os.to_dict()
        if self.domain:
            result["domain"] = self.domain
        if self.agent_id:
            result["agent_id"] = self.agent_id
        return result


@dataclass
class User:
    name: Optional[str] = None
    domain: Optional[str] = None
    email: Optional[str] = None
    employee_id: Optional[str] = None
    groups: Optional[List[str]] = None
    roles: Optional[List[str]] = None

    def to_dict(self) -> Dict[str, Any]:
        return {k: v for k, v in {
            "name": self.name,
            "domain": self.domain,
            "email": self.email,
            "employee_id": self.employee_id,
            "groups": self.groups,
            "roles": self.roles,
        }.items() if v is not None}


@dataclass
class Cloud:
    provider: Optional[str] = None
    account_id: Optional[str] = None
    region: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {k: v for k, v in {
            "provider": self.provider,
            "account_id": self.account_id,
            "region": self.region,
        }.items() if v is not None}


@dataclass
class NetworkEndpoint:
    ip: Optional[str] = None
    port: Optional[int] = None
    hostname: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {k: v for k, v in {
            "ip": self.ip,
            "port": self.port,
            "hostname": self.hostname,
        }.items() if v is not None}


@dataclass
class NetworkContext:
    source: Optional[NetworkEndpoint] = None
    destination: Optional[NetworkEndpoint] = None
    protocol: Optional[str] = None
    direction: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {}
        if self.source:
            result["source"] = self.source.to_dict()
        if self.destination:
            result["destination"] = self.destination.to_dict()
        if self.protocol:
            result["protocol"] = self.protocol
        if self.direction:
            result["direction"] = self.direction
        return result


@dataclass
class Context:
    host: Optional[Host] = None
    network: Optional[NetworkContext] = None
    user: Optional[User] = None
    cloud: Optional[Cloud] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {}
        if self.host:
            result["host"] = self.host.to_dict()
        if self.network:
            result["network"] = self.network.to_dict()
        if self.user:
            result["user"] = self.user.to_dict()
        if self.cloud:
            result["cloud"] = self.cloud.to_dict()
        return result


@dataclass
class Mitre:
    tactic: Optional[str] = None
    tactic_name: Optional[str] = None
    technique: Optional[str] = None
    technique_name: Optional[str] = None
    sub_technique: Optional[str] = None
    sub_technique_name: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {k: v for k, v in {
            "tactic": self.tactic,
            "tactic_name": self.tactic_name,
            "technique": self.technique,
            "technique_name": self.technique_name,
            "sub_technique": self.sub_technique,
            "sub_technique_name": self.sub_technique_name,
        }.items() if v is not None}


@dataclass
class Meta:
    confidence: Optional[float] = None
    tags: Optional[List[str]] = None
    labels: Optional[Dict[str, str]] = None
    raw: Optional[str] = None
    correlation_id: Optional[str] = None
    custom: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        return {k: v for k, v in {
            "confidence": self.confidence,
            "tags": self.tags,
            "labels": self.labels,
            "raw": self.raw,
            "correlation_id": self.correlation_id,
            "custom": self.custom,
        }.items() if v is not None}


@dataclass
class Indicator:
    type: str
    value: str
    context: Optional[str] = None
    confidence: Optional[float] = None
    first_seen: Optional[str] = None
    last_seen: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        result = {"type": self.type, "value": self.value}
        if self.context:
            result["context"] = self.context
        if self.confidence is not None:
            result["confidence"] = self.confidence
        if self.first_seen:
            result["first_seen"] = self.first_seen
        if self.last_seen:
            result["last_seen"] = self.last_seen
        return result


# ============================================================================
# Event Data Classes
# ============================================================================

@dataclass
class AlertData:
    alert_id: str
    title: str
    category: str
    status: str
    priority: str
    description: Optional[str] = None
    assignee: Optional[str] = None
    detection_rule: Optional[Dict[str, str]] = None
    indicators: Optional[List[Indicator]] = None
    first_seen: Optional[str] = None
    last_seen: Optional[str] = None
    count: Optional[int] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {
            "alert_id": self.alert_id,
            "title": self.title,
            "category": self.category,
            "status": self.status,
            "priority": self.priority,
        }
        if self.description:
            result["description"] = self.description
        if self.assignee:
            result["assignee"] = self.assignee
        if self.detection_rule:
            result["detection_rule"] = self.detection_rule
        if self.indicators:
            result["indicators"] = [i.to_dict() for i in self.indicators]
        if self.first_seen:
            result["first_seen"] = self.first_seen
        if self.last_seen:
            result["last_seen"] = self.last_seen
        if self.count is not None:
            result["count"] = self.count
        return result


@dataclass
class ThreatIntelData:
    threat_type: str
    threat_name: str
    status: str
    threat_family: Optional[str] = None
    aliases: Optional[List[str]] = None
    first_seen: Optional[str] = None
    last_seen: Optional[str] = None
    indicators: Optional[List[Dict[str, Any]]] = None
    ttps: Optional[List[Dict[str, str]]] = None
    target_sectors: Optional[List[str]] = None
    target_countries: Optional[List[str]] = None
    references: Optional[List[str]] = None
    report_id: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {
            "threat_type": self.threat_type,
            "threat_name": self.threat_name,
            "status": self.status,
        }
        if self.threat_family:
            result["threat_family"] = self.threat_family
        if self.aliases:
            result["aliases"] = self.aliases
        if self.first_seen:
            result["first_seen"] = self.first_seen
        if self.last_seen:
            result["last_seen"] = self.last_seen
        if self.indicators:
            result["indicators"] = self.indicators
        if self.ttps:
            result["ttps"] = self.ttps
        if self.target_sectors:
            result["target_sectors"] = self.target_sectors
        if self.target_countries:
            result["target_countries"] = self.target_countries
        if self.references:
            result["references"] = self.references
        if self.report_id:
            result["report_id"] = self.report_id
        return result


@dataclass
class CVSS:
    version: str
    score: float
    severity: str
    vector: Optional[str] = None
    base_score: Optional[float] = None
    temporal_score: Optional[float] = None
    environmental_score: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {
            "version": self.version,
            "score": self.score,
            "severity": self.severity,
        }
        if self.vector:
            result["vector"] = self.vector
        if self.base_score is not None:
            result["base_score"] = self.base_score
        if self.temporal_score is not None:
            result["temporal_score"] = self.temporal_score
        if self.environmental_score is not None:
            result["environmental_score"] = self.environmental_score
        return result


@dataclass
class VulnerabilityData:
    vuln_id: str
    title: str
    cvss: CVSS
    description: Optional[str] = None
    cwe: Optional[List[str]] = None
    affected_products: Optional[List[Dict[str, Any]]] = None
    exploit_available: Optional[bool] = None
    exploit_details: Optional[Dict[str, Any]] = None
    patch_available: Optional[bool] = None
    patch_details: Optional[Dict[str, Any]] = None
    references: Optional[List[str]] = None
    published: Optional[str] = None
    modified: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {
            "vuln_id": self.vuln_id,
            "title": self.title,
            "cvss": self.cvss.to_dict(),
        }
        if self.description:
            result["description"] = self.description
        if self.cwe:
            result["cwe"] = self.cwe
        if self.affected_products:
            result["affected_products"] = self.affected_products
        if self.exploit_available is not None:
            result["exploit_available"] = self.exploit_available
        if self.exploit_details:
            result["exploit_details"] = self.exploit_details
        if self.patch_available is not None:
            result["patch_available"] = self.patch_available
        if self.patch_details:
            result["patch_details"] = self.patch_details
        if self.references:
            result["references"] = self.references
        if self.published:
            result["published"] = self.published
        if self.modified:
            result["modified"] = self.modified
        return result


@dataclass
class IncidentData:
    incident_id: str
    title: str
    category: str
    status: str
    priority: str
    description: Optional[str] = None
    impact: Optional[Dict[str, str]] = None
    timeline: Optional[List[Dict[str, Any]]] = None
    affected_assets: Optional[List[Dict[str, Any]]] = None
    iocs: Optional[List[Dict[str, Any]]] = None
    response_actions: Optional[List[Dict[str, Any]]] = None
    root_cause: Optional[str] = None
    lessons_learned: Optional[str] = None
    created_at: Optional[str] = None
    updated_at: Optional[str] = None
    closed_at: Optional[str] = None
    lead_analyst: Optional[str] = None
    team: Optional[List[str]] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {
            "incident_id": self.incident_id,
            "title": self.title,
            "category": self.category,
            "status": self.status,
            "priority": self.priority,
        }
        for key, value in [
            ("description", self.description),
            ("impact", self.impact),
            ("timeline", self.timeline),
            ("affected_assets", self.affected_assets),
            ("iocs", self.iocs),
            ("response_actions", self.response_actions),
            ("root_cause", self.root_cause),
            ("lessons_learned", self.lessons_learned),
            ("created_at", self.created_at),
            ("updated_at", self.updated_at),
            ("closed_at", self.closed_at),
            ("lead_analyst", self.lead_analyst),
            ("team", self.team),
        ]:
            if value is not None:
                result[key] = value
        return result


@dataclass
class NetworkEventData:
    event_type: str
    protocol: str
    source: Dict[str, Any]
    destination: Dict[str, Any]
    direction: Optional[str] = None
    action: Optional[str] = None
    bytes_sent: Optional[int] = None
    bytes_received: Optional[int] = None
    packets_sent: Optional[int] = None
    packets_received: Optional[int] = None
    duration_ms: Optional[int] = None
    rule_matched: Optional[str] = None
    application: Optional[str] = None
    url: Optional[str] = None
    http: Optional[Dict[str, Any]] = None
    dns: Optional[Dict[str, Any]] = None
    tls: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {
            "event_type": self.event_type,
            "protocol": self.protocol,
            "source": self.source,
            "destination": self.destination,
        }
        for key, value in [
            ("direction", self.direction),
            ("action", self.action),
            ("bytes_sent", self.bytes_sent),
            ("bytes_received", self.bytes_received),
            ("packets_sent", self.packets_sent),
            ("packets_received", self.packets_received),
            ("duration_ms", self.duration_ms),
            ("rule_matched", self.rule_matched),
            ("application", self.application),
            ("url", self.url),
            ("http", self.http),
            ("dns", self.dns),
            ("tls", self.tls),
        ]:
            if value is not None:
                result[key] = value
        return result


@dataclass
class EndpointEventData:
    event_type: str
    host: Dict[str, Any]
    process: Optional[Dict[str, Any]] = None
    file: Optional[Dict[str, Any]] = None
    registry: Optional[Dict[str, Any]] = None
    network_connection: Optional[Dict[str, Any]] = None
    dll: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {
            "event_type": self.event_type,
            "host": self.host,
        }
        for key, value in [
            ("process", self.process),
            ("file", self.file),
            ("registry", self.registry),
            ("network_connection", self.network_connection),
            ("dll", self.dll),
        ]:
            if value is not None:
                result[key] = value
        return result


@dataclass
class AuthEventData:
    event_type: str
    result: str
    user: Dict[str, Any]
    target: Dict[str, Any]
    failure_reason: Optional[str] = None
    source: Optional[Dict[str, Any]] = None
    auth_method: Optional[str] = None
    mfa_used: Optional[bool] = None
    mfa_method: Optional[str] = None
    session_id: Optional[str] = None
    logon_type: Optional[str] = None
    attempt_count: Optional[int] = None
    previous_login: Optional[str] = None
    risk_score: Optional[float] = None
    risk_factors: Optional[List[str]] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {
            "event_type": self.event_type,
            "result": self.result,
            "user": self.user,
            "target": self.target,
        }
        for key, value in [
            ("failure_reason", self.failure_reason),
            ("source", self.source),
            ("auth_method", self.auth_method),
            ("mfa_used", self.mfa_used),
            ("mfa_method", self.mfa_method),
            ("session_id", self.session_id),
            ("logon_type", self.logon_type),
            ("attempt_count", self.attempt_count),
            ("previous_login", self.previous_login),
            ("risk_score", self.risk_score),
            ("risk_factors", self.risk_factors),
        ]:
            if value is not None:
                result[key] = value
        return result


# ============================================================================
# Main Event Class
# ============================================================================

@dataclass
class WiaSecurityEvent:
    version: str
    id: str
    type: str
    timestamp: str
    severity: float
    source: Source
    data: Union[AlertData, ThreatIntelData, VulnerabilityData, IncidentData,
                NetworkEventData, EndpointEventData, AuthEventData, Dict[str, Any]]
    schema: Optional[str] = None
    context: Optional[Context] = None
    mitre: Optional[Mitre] = None
    meta: Optional[Meta] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {
            "version": self.version,
            "id": self.id,
            "type": self.type,
            "timestamp": self.timestamp,
            "severity": self.severity,
            "source": self.source.to_dict() if isinstance(self.source, Source) else self.source,
            "data": self.data.to_dict() if hasattr(self.data, 'to_dict') else self.data,
        }
        if self.schema:
            result["$schema"] = self.schema
        if self.context:
            result["context"] = self.context.to_dict()
        if self.mitre:
            result["mitre"] = self.mitre.to_dict()
        if self.meta:
            result["meta"] = self.meta.to_dict()
        return result

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "WiaSecurityEvent":
        source = Source(**data["source"]) if isinstance(data["source"], dict) else data["source"]

        context = None
        if "context" in data and data["context"]:
            context = Context(
                host=Host(**data["context"]["host"]) if data["context"].get("host") else None,
                user=User(**data["context"]["user"]) if data["context"].get("user") else None,
                cloud=Cloud(**data["context"]["cloud"]) if data["context"].get("cloud") else None,
            )

        mitre = None
        if "mitre" in data and data["mitre"]:
            mitre = Mitre(**data["mitre"])

        meta = None
        if "meta" in data and data["meta"]:
            meta = Meta(**data["meta"])

        return cls(
            version=data["version"],
            id=data["id"],
            type=data["type"],
            timestamp=data["timestamp"],
            severity=data["severity"],
            source=source,
            data=data["data"],
            schema=data.get("$schema"),
            context=context,
            mitre=mitre,
            meta=meta,
        )
