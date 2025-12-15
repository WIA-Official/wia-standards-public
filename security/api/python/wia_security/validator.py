"""
WIA Security Event Validator
Validation utilities for WIA Security events
"""

import re
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Union

from .types import WiaSecurityEvent

# ============================================================================
# Validation Result
# ============================================================================

@dataclass
class ValidationResult:
    """Validation result container"""
    valid: bool = True
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)


# ============================================================================
# Validation Patterns
# ============================================================================

UUID_PATTERN = re.compile(r'^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$', re.I)
ISO8601_PATTERN = re.compile(r'^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(\.\d{3})?Z$')
SEMVER_PATTERN = re.compile(r'^\d+\.\d+\.\d+$')
TACTIC_PATTERN = re.compile(r'^TA\d{4}$')
TECHNIQUE_PATTERN = re.compile(r'^T\d{4}(\.\d{3})?$')
SUB_TECHNIQUE_PATTERN = re.compile(r'^T\d{4}\.\d{3}$')
CVE_PATTERN = re.compile(r'^CVE-\d{4}-\d{4,}$')
CWE_PATTERN = re.compile(r'^CWE-\d+$')

# ============================================================================
# Valid Values
# ============================================================================

VALID_EVENT_TYPES = ['alert', 'threat_intel', 'vulnerability', 'incident',
                     'network_event', 'endpoint_event', 'auth_event']
VALID_SOURCE_TYPES = ['siem', 'edr', 'ids', 'firewall', 'scanner', 'custom']
VALID_PRIORITIES = ['critical', 'high', 'medium', 'low', 'info']
VALID_DIRECTIONS = ['inbound', 'outbound', 'internal']
VALID_CLOUD_PROVIDERS = ['aws', 'azure', 'gcp']


# ============================================================================
# Main Validator
# ============================================================================

def validate_event(event: Union[WiaSecurityEvent, Dict[str, Any]]) -> ValidationResult:
    """
    Validate a WIA Security Event.

    Args:
        event: WiaSecurityEvent instance or dictionary

    Returns:
        ValidationResult with valid flag, errors, and warnings
    """
    result = ValidationResult()

    # Convert to dict if needed
    if isinstance(event, WiaSecurityEvent):
        data = event.to_dict()
    elif isinstance(event, dict):
        data = event
    else:
        result.valid = False
        result.errors.append('Event must be a WiaSecurityEvent or dictionary')
        return result

    # Validate required fields
    _validate_required_fields(data, result)

    # Validate source
    if 'source' in data:
        _validate_source(data['source'], result)

    # Validate event data
    if 'data' in data and 'type' in data:
        _validate_event_data(data['type'], data['data'], result)

    # Validate optional fields
    if 'context' in data:
        _validate_context(data['context'], result)

    if 'mitre' in data:
        _validate_mitre(data['mitre'], result)

    if 'meta' in data:
        _validate_meta(data['meta'], result)

    # Warnings
    if '$schema' not in data:
        result.warnings.append('Recommended field $schema is missing')

    result.valid = len(result.errors) == 0
    return result


# ============================================================================
# Field Validators
# ============================================================================

def _validate_required_fields(data: Dict[str, Any], result: ValidationResult) -> None:
    """Validate required fields"""
    # Version
    if 'version' not in data:
        result.errors.append('Missing required field: version')
    elif not isinstance(data['version'], str) or not SEMVER_PATTERN.match(data['version']):
        result.errors.append('Invalid version format. Expected SemVer (e.g., 1.0.0)')

    # ID
    if 'id' not in data:
        result.errors.append('Missing required field: id')
    elif not isinstance(data['id'], str) or not UUID_PATTERN.match(data['id']):
        result.errors.append('Invalid id format. Expected UUID v4')

    # Type
    if 'type' not in data:
        result.errors.append('Missing required field: type')
    elif data['type'] not in VALID_EVENT_TYPES:
        result.errors.append(f"Invalid event type: {data['type']}")

    # Timestamp
    if 'timestamp' not in data:
        result.errors.append('Missing required field: timestamp')
    elif not isinstance(data['timestamp'], str) or not ISO8601_PATTERN.match(data['timestamp']):
        result.errors.append('Invalid timestamp format. Expected ISO 8601')

    # Severity
    if 'severity' not in data:
        result.errors.append('Missing required field: severity')
    elif not isinstance(data['severity'], (int, float)) or data['severity'] < 0 or data['severity'] > 10:
        result.errors.append('Invalid severity. Expected number between 0 and 10')

    # Source
    if 'source' not in data:
        result.errors.append('Missing required field: source')

    # Data
    if 'data' not in data:
        result.errors.append('Missing required field: data')
    elif not isinstance(data['data'], dict):
        result.errors.append('Invalid data field. Expected object')


def _validate_source(source: Any, result: ValidationResult) -> None:
    """Validate source object"""
    if not isinstance(source, dict):
        result.errors.append('Source must be an object')
        return

    if 'type' not in source:
        result.errors.append('Missing required field in source: type')
    elif source['type'] not in VALID_SOURCE_TYPES:
        result.errors.append(f"Invalid source type: {source['type']}")

    if 'name' not in source:
        result.errors.append('Missing required field in source: name')


def _validate_context(context: Any, result: ValidationResult) -> None:
    """Validate context object"""
    if not isinstance(context, dict):
        result.errors.append('Context must be an object')
        return

    if 'host' in context and isinstance(context['host'], dict):
        host = context['host']
        if 'ip' in host and not isinstance(host['ip'], list):
            result.errors.append('context.host.ip must be an array')
        if 'mac' in host and not isinstance(host['mac'], list):
            result.errors.append('context.host.mac must be an array')

    if 'network' in context and isinstance(context['network'], dict):
        network = context['network']
        if 'direction' in network and network['direction'] not in VALID_DIRECTIONS:
            result.errors.append(f"Invalid network direction: {network['direction']}")

    if 'cloud' in context and isinstance(context['cloud'], dict):
        cloud = context['cloud']
        if 'provider' in cloud and cloud['provider'] not in VALID_CLOUD_PROVIDERS:
            result.errors.append(f"Invalid cloud provider: {cloud['provider']}")


def _validate_mitre(mitre: Any, result: ValidationResult) -> None:
    """Validate MITRE ATT&CK mapping"""
    if not isinstance(mitre, dict):
        result.errors.append('Mitre must be an object')
        return

    if 'tactic' in mitre and isinstance(mitre['tactic'], str):
        if not TACTIC_PATTERN.match(mitre['tactic']):
            result.errors.append(f"Invalid MITRE tactic ID format: {mitre['tactic']}")

    if 'technique' in mitre and isinstance(mitre['technique'], str):
        if not TECHNIQUE_PATTERN.match(mitre['technique']):
            result.errors.append(f"Invalid MITRE technique ID format: {mitre['technique']}")

    if 'sub_technique' in mitre and isinstance(mitre['sub_technique'], str):
        if not SUB_TECHNIQUE_PATTERN.match(mitre['sub_technique']):
            result.errors.append(f"Invalid MITRE sub-technique ID format: {mitre['sub_technique']}")


def _validate_meta(meta: Any, result: ValidationResult) -> None:
    """Validate meta object"""
    if not isinstance(meta, dict):
        result.errors.append('Meta must be an object')
        return

    if 'confidence' in meta:
        conf = meta['confidence']
        if not isinstance(conf, (int, float)) or conf < 0 or conf > 1:
            result.errors.append('meta.confidence must be a number between 0 and 1')

    if 'tags' in meta and not isinstance(meta['tags'], list):
        result.errors.append('meta.tags must be an array')

    if 'labels' in meta and not isinstance(meta['labels'], dict):
        result.errors.append('meta.labels must be an object')


def _validate_event_data(event_type: str, data: Dict[str, Any], result: ValidationResult) -> None:
    """Validate event-specific data"""
    validators = {
        'alert': _validate_alert_data,
        'threat_intel': _validate_threat_intel_data,
        'vulnerability': _validate_vulnerability_data,
        'incident': _validate_incident_data,
        'network_event': _validate_network_event_data,
        'endpoint_event': _validate_endpoint_event_data,
        'auth_event': _validate_auth_event_data,
    }

    if event_type in validators:
        validators[event_type](data, result)


def _validate_alert_data(data: Dict[str, Any], result: ValidationResult) -> None:
    """Validate alert data"""
    required = ['alert_id', 'title', 'category', 'status', 'priority']
    for f in required:
        if f not in data:
            result.errors.append(f'Missing required field in alert data: {f}')

    valid_categories = ['malware', 'intrusion', 'policy', 'reconnaissance', 'other']
    if 'category' in data and data['category'] not in valid_categories:
        result.errors.append(f"Invalid alert category: {data['category']}")

    valid_statuses = ['new', 'investigating', 'resolved', 'false_positive', 'closed']
    if 'status' in data and data['status'] not in valid_statuses:
        result.errors.append(f"Invalid alert status: {data['status']}")

    if 'priority' in data and data['priority'] not in VALID_PRIORITIES:
        result.errors.append(f"Invalid alert priority: {data['priority']}")


def _validate_threat_intel_data(data: Dict[str, Any], result: ValidationResult) -> None:
    """Validate threat intelligence data"""
    required = ['threat_type', 'threat_name', 'status']
    for f in required:
        if f not in data:
            result.errors.append(f'Missing required field in threat_intel data: {f}')

    valid_types = ['malware', 'apt', 'campaign', 'botnet', 'ransomware', 'phishing']
    if 'threat_type' in data and data['threat_type'] not in valid_types:
        result.errors.append(f"Invalid threat_type: {data['threat_type']}")

    valid_statuses = ['active', 'inactive', 'unknown']
    if 'status' in data and data['status'] not in valid_statuses:
        result.errors.append(f"Invalid threat_intel status: {data['status']}")


def _validate_vulnerability_data(data: Dict[str, Any], result: ValidationResult) -> None:
    """Validate vulnerability data"""
    required = ['vuln_id', 'title', 'cvss']
    for f in required:
        if f not in data:
            result.errors.append(f'Missing required field in vulnerability data: {f}')

    if 'vuln_id' in data and isinstance(data['vuln_id'], str):
        if not CVE_PATTERN.match(data['vuln_id']):
            result.errors.append(f"Invalid CVE format: {data['vuln_id']}")

    if 'cvss' in data and isinstance(data['cvss'], dict):
        cvss = data['cvss']
        if 'score' in cvss:
            score = cvss['score']
            if not isinstance(score, (int, float)) or score < 0 or score > 10:
                result.errors.append('cvss.score must be a number between 0 and 10')

    if 'cwe' in data and isinstance(data['cwe'], list):
        for cwe in data['cwe']:
            if not CWE_PATTERN.match(cwe):
                result.errors.append(f"Invalid CWE format: {cwe}")


def _validate_incident_data(data: Dict[str, Any], result: ValidationResult) -> None:
    """Validate incident data"""
    required = ['incident_id', 'title', 'category', 'status', 'priority']
    for f in required:
        if f not in data:
            result.errors.append(f'Missing required field in incident data: {f}')

    valid_categories = ['malware', 'phishing', 'ransomware', 'data_breach', 'ddos',
                       'unauthorized_access', 'insider_threat', 'apt', 'other']
    if 'category' in data and data['category'] not in valid_categories:
        result.errors.append(f"Invalid incident category: {data['category']}")

    valid_statuses = ['new', 'triaging', 'investigating', 'containing',
                     'eradicating', 'recovering', 'closed']
    if 'status' in data and data['status'] not in valid_statuses:
        result.errors.append(f"Invalid incident status: {data['status']}")


def _validate_network_event_data(data: Dict[str, Any], result: ValidationResult) -> None:
    """Validate network event data"""
    required = ['event_type', 'protocol', 'source', 'destination']
    for f in required:
        if f not in data:
            result.errors.append(f'Missing required field in network_event data: {f}')

    valid_types = ['connection', 'dns', 'http', 'tls', 'smtp', 'ftp', 'ssh', 'rdp', 'custom']
    if 'event_type' in data and data['event_type'] not in valid_types:
        result.errors.append(f"Invalid network event_type: {data['event_type']}")

    valid_protocols = ['TCP', 'UDP', 'ICMP', 'GRE', 'ESP']
    if 'protocol' in data and data['protocol'] not in valid_protocols:
        result.errors.append(f"Invalid network protocol: {data['protocol']}")


def _validate_endpoint_event_data(data: Dict[str, Any], result: ValidationResult) -> None:
    """Validate endpoint event data"""
    required = ['event_type', 'host']
    for f in required:
        if f not in data:
            result.errors.append(f'Missing required field in endpoint_event data: {f}')

    valid_types = [
        'process_creation', 'process_termination', 'file_create', 'file_modify', 'file_delete',
        'registry_create', 'registry_modify', 'registry_delete', 'network_connection',
        'dll_load', 'driver_load', 'service_install'
    ]
    if 'event_type' in data and data['event_type'] not in valid_types:
        result.errors.append(f"Invalid endpoint event_type: {data['event_type']}")


def _validate_auth_event_data(data: Dict[str, Any], result: ValidationResult) -> None:
    """Validate authentication event data"""
    required = ['event_type', 'result', 'user', 'target']
    for f in required:
        if f not in data:
            result.errors.append(f'Missing required field in auth_event data: {f}')

    valid_types = [
        'login_success', 'login_failure', 'logout', 'password_change',
        'account_locked', 'account_unlocked', 'mfa_success', 'mfa_failure', 'privilege_escalation'
    ]
    if 'event_type' in data and data['event_type'] not in valid_types:
        result.errors.append(f"Invalid auth event_type: {data['event_type']}")

    if 'result' in data and data['result'] not in ['success', 'failure']:
        result.errors.append(f"Invalid auth result: {data['result']}")

    if 'risk_score' in data:
        score = data['risk_score']
        if not isinstance(score, (int, float)) or score < 0 or score > 1:
            result.errors.append('risk_score must be a number between 0 and 1')


# ============================================================================
# Type Guards
# ============================================================================

def is_alert_event(event: Union[WiaSecurityEvent, Dict[str, Any]]) -> bool:
    """Check if event is an alert"""
    if isinstance(event, WiaSecurityEvent):
        return event.type == 'alert'
    return event.get('type') == 'alert'


def is_threat_intel_event(event: Union[WiaSecurityEvent, Dict[str, Any]]) -> bool:
    """Check if event is threat intelligence"""
    if isinstance(event, WiaSecurityEvent):
        return event.type == 'threat_intel'
    return event.get('type') == 'threat_intel'


def is_vulnerability_event(event: Union[WiaSecurityEvent, Dict[str, Any]]) -> bool:
    """Check if event is a vulnerability"""
    if isinstance(event, WiaSecurityEvent):
        return event.type == 'vulnerability'
    return event.get('type') == 'vulnerability'


def is_incident_event(event: Union[WiaSecurityEvent, Dict[str, Any]]) -> bool:
    """Check if event is an incident"""
    if isinstance(event, WiaSecurityEvent):
        return event.type == 'incident'
    return event.get('type') == 'incident'


def is_network_event(event: Union[WiaSecurityEvent, Dict[str, Any]]) -> bool:
    """Check if event is a network event"""
    if isinstance(event, WiaSecurityEvent):
        return event.type == 'network_event'
    return event.get('type') == 'network_event'


def is_endpoint_event(event: Union[WiaSecurityEvent, Dict[str, Any]]) -> bool:
    """Check if event is an endpoint event"""
    if isinstance(event, WiaSecurityEvent):
        return event.type == 'endpoint_event'
    return event.get('type') == 'endpoint_event'


def is_auth_event(event: Union[WiaSecurityEvent, Dict[str, Any]]) -> bool:
    """Check if event is an authentication event"""
    if isinstance(event, WiaSecurityEvent):
        return event.type == 'auth_event'
    return event.get('type') == 'auth_event'
