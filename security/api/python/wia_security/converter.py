"""
WIA Security Event Converter
Utilities for converting between WIA Security and other formats
"""

import uuid
from datetime import datetime
from typing import Any, Dict, List, Optional, Union

from .types import WiaSecurityEvent, Indicator


# ============================================================================
# STIX 2.1 Conversion
# ============================================================================

def to_stix_bundle(event: Union[WiaSecurityEvent, Dict[str, Any]]) -> Dict[str, Any]:
    """
    Convert WIA Security event to STIX 2.1 Bundle.

    Args:
        event: WiaSecurityEvent or dictionary

    Returns:
        STIX 2.1 Bundle dictionary
    """
    if isinstance(event, WiaSecurityEvent):
        data = event.to_dict()
    else:
        data = event

    bundle: Dict[str, Any] = {
        "type": "bundle",
        "id": f"bundle--{uuid.uuid4()}",
        "objects": []
    }

    timestamp = data.get("timestamp", datetime.utcnow().isoformat() + "Z")

    if data.get("type") == "threat_intel":
        event_data = data.get("data", {})

        # Convert indicators
        indicators = event_data.get("indicators", [])
        for indicator in indicators:
            bundle["objects"].append(to_stix_indicator(indicator, timestamp))

        # Add malware object if applicable
        threat_type = event_data.get("threat_type")
        if threat_type in ("malware", "ransomware"):
            bundle["objects"].append({
                "type": "malware",
                "spec_version": "2.1",
                "id": f"malware--{uuid.uuid4()}",
                "created": timestamp,
                "modified": timestamp,
                "name": event_data.get("threat_name", "Unknown"),
                "malware_types": [threat_type],
                "is_family": bool(event_data.get("threat_family")),
                "aliases": event_data.get("aliases", [])
            })

        # Add threat actor if APT
        if threat_type == "apt":
            bundle["objects"].append({
                "type": "threat-actor",
                "spec_version": "2.1",
                "id": f"threat-actor--{uuid.uuid4()}",
                "created": timestamp,
                "modified": timestamp,
                "name": event_data.get("threat_name", "Unknown"),
                "aliases": event_data.get("aliases", []),
                "threat_actor_types": ["nation-state"],
                "primary_motivation": "organizational-gain"
            })

    elif data.get("type") == "alert":
        event_data = data.get("data", {})

        # Convert to sighting
        bundle["objects"].append({
            "type": "sighting",
            "spec_version": "2.1",
            "id": f"sighting--{uuid.uuid4()}",
            "created": timestamp,
            "modified": timestamp,
            "first_seen": event_data.get("first_seen", timestamp),
            "last_seen": event_data.get("last_seen", timestamp),
            "count": event_data.get("count", 1),
            "description": event_data.get("description")
        })

        # Convert indicators
        indicators = event_data.get("indicators", [])
        for indicator in indicators:
            bundle["objects"].append(to_stix_indicator(indicator, timestamp))

    elif data.get("type") == "vulnerability":
        event_data = data.get("data", {})

        bundle["objects"].append({
            "type": "vulnerability",
            "spec_version": "2.1",
            "id": f"vulnerability--{uuid.uuid4()}",
            "created": timestamp,
            "modified": timestamp,
            "name": event_data.get("vuln_id", "Unknown"),
            "description": event_data.get("description", event_data.get("title", "")),
            "external_references": [
                {
                    "source_name": "cve",
                    "external_id": event_data.get("vuln_id", "")
                }
            ]
        })

    return bundle


def to_stix_indicator(indicator: Union[Indicator, Dict[str, Any]], timestamp: str) -> Dict[str, Any]:
    """
    Convert WIA indicator to STIX indicator.

    Args:
        indicator: Indicator or dictionary
        timestamp: ISO 8601 timestamp

    Returns:
        STIX indicator dictionary
    """
    if isinstance(indicator, Indicator):
        ind_type = indicator.type
        ind_value = indicator.value
        ind_context = indicator.context
        ind_first_seen = indicator.first_seen
    else:
        ind_type = indicator.get("type", "")
        ind_value = indicator.get("value", "")
        ind_context = indicator.get("context")
        ind_first_seen = indicator.get("first_seen")

    # Build pattern
    pattern_map = {
        "ip": f"[ipv4-addr:value = '{ind_value}']",
        "domain": f"[domain-name:value = '{ind_value}']",
        "url": f"[url:value = '{ind_value}']",
        "hash": f"[file:hashes.MD5 = '{ind_value}']",
        "file_hash_md5": f"[file:hashes.MD5 = '{ind_value}']",
        "file_hash_sha1": f"[file:hashes.'SHA-1' = '{ind_value}']",
        "file_hash_sha256": f"[file:hashes.'SHA-256' = '{ind_value}']",
        "email": f"[email-addr:value = '{ind_value}']",
    }
    pattern = pattern_map.get(ind_type, f"[x-custom:value = '{ind_value}']")

    return {
        "type": "indicator",
        "spec_version": "2.1",
        "id": f"indicator--{uuid.uuid4()}",
        "created": timestamp,
        "modified": timestamp,
        "name": ind_context or f"{ind_type}: {ind_value}",
        "pattern": pattern,
        "pattern_type": "stix",
        "valid_from": ind_first_seen or timestamp
    }


# ============================================================================
# ECS Conversion
# ============================================================================

def to_ecs_event(event: Union[WiaSecurityEvent, Dict[str, Any]]) -> Dict[str, Any]:
    """
    Convert WIA Security event to ECS format.

    Args:
        event: WiaSecurityEvent or dictionary

    Returns:
        ECS event dictionary
    """
    if isinstance(event, WiaSecurityEvent):
        data = event.to_dict()
    else:
        data = event

    event_type = data.get("type", "")

    # Map event kind
    kind_map = {
        "alert": "alert",
        "threat_intel": "enrichment",
        "vulnerability": "state",
        "incident": "alert",
        "network_event": "event",
        "endpoint_event": "event",
        "auth_event": "event"
    }

    # Map category
    category_map = {
        "alert": ["intrusion_detection"],
        "threat_intel": ["threat"],
        "vulnerability": ["vulnerability"],
        "incident": ["intrusion_detection"],
        "network_event": ["network"],
        "endpoint_event": ["host"],
        "auth_event": ["authentication"]
    }

    ecs: Dict[str, Any] = {
        "@timestamp": data.get("timestamp"),
        "event": {
            "id": data.get("id"),
            "kind": kind_map.get(event_type, "event"),
            "category": category_map.get(event_type, ["configuration"]),
            "type": ["info"],
            "severity": data.get("severity", 0),
            "created": data.get("timestamp"),
        },
        "wia": {
            "version": data.get("version"),
            "type": event_type,
            "data": data.get("data")
        }
    }

    # Map context
    context = data.get("context", {})
    if context.get("host"):
        host = context["host"]
        ecs["host"] = {
            "hostname": host.get("hostname"),
            "ip": host.get("ip"),
            "mac": host.get("mac"),
        }
        if host.get("os"):
            ecs["host"]["os"] = {
                "name": host["os"].get("name"),
                "version": host["os"].get("version")
            }

    if context.get("user"):
        user = context["user"]
        ecs["user"] = {
            "name": user.get("name"),
            "domain": user.get("domain"),
            "email": user.get("email")
        }

    if context.get("network"):
        network = context["network"]
        if network.get("source"):
            ecs["source"] = {
                "ip": network["source"].get("ip"),
                "port": network["source"].get("port"),
                "domain": network["source"].get("hostname")
            }
        if network.get("destination"):
            ecs["destination"] = {
                "ip": network["destination"].get("ip"),
                "port": network["destination"].get("port"),
                "domain": network["destination"].get("hostname")
            }

    # Map MITRE
    mitre = data.get("mitre", {})
    if mitre:
        ecs["threat"] = {
            "framework": "MITRE ATT&CK"
        }
        if mitre.get("tactic"):
            ecs["threat"]["tactic"] = {
                "id": [mitre["tactic"]],
                "name": [mitre["tactic_name"]] if mitre.get("tactic_name") else None
            }
        if mitre.get("technique"):
            ecs["threat"]["technique"] = {
                "id": [mitre["technique"]],
                "name": [mitre["technique_name"]] if mitre.get("technique_name") else None
            }

    # Map meta
    meta = data.get("meta", {})
    if meta.get("tags"):
        ecs["tags"] = meta["tags"]
    if meta.get("labels"):
        ecs["labels"] = meta["labels"]
    if meta.get("raw"):
        ecs["event"]["original"] = meta["raw"]

    return ecs


# ============================================================================
# OCSF Conversion
# ============================================================================

def to_ocsf_event(event: Union[WiaSecurityEvent, Dict[str, Any]]) -> Dict[str, Any]:
    """
    Convert WIA Security event to OCSF format.

    Args:
        event: WiaSecurityEvent or dictionary

    Returns:
        OCSF event dictionary
    """
    if isinstance(event, WiaSecurityEvent):
        data = event.to_dict()
    else:
        data = event

    event_type = data.get("type", "")

    # Map class
    class_map = {
        "alert": (2001, "Security Finding", 2, "Findings"),
        "threat_intel": (2001, "Security Finding", 2, "Findings"),
        "vulnerability": (2002, "Vulnerability Finding", 2, "Findings"),
        "incident": (2003, "Incident Finding", 2, "Findings"),
        "network_event": (4001, "Network Activity", 4, "Network Activity"),
        "endpoint_event": (1001, "Process Activity", 1, "System Activity"),
        "auth_event": (3001, "Authentication", 3, "Identity & Access"),
    }

    class_uid, class_name, category_uid, category_name = class_map.get(
        event_type, (0, "Unknown", 0, "Unknown")
    )

    # Map severity
    severity = data.get("severity", 0)
    if severity == 0:
        severity_id = 0
        severity_name = "Unknown"
    elif severity <= 2:
        severity_id = 1
        severity_name = "Informational"
    elif severity <= 4:
        severity_id = 2
        severity_name = "Low"
    elif severity <= 6:
        severity_id = 3
        severity_name = "Medium"
    elif severity <= 8:
        severity_id = 4
        severity_name = "High"
    else:
        severity_id = 5
        severity_name = "Critical"

    timestamp = data.get("timestamp", "")
    try:
        time_ms = int(datetime.fromisoformat(timestamp.replace("Z", "+00:00")).timestamp() * 1000)
    except (ValueError, AttributeError):
        time_ms = int(datetime.utcnow().timestamp() * 1000)

    ocsf: Dict[str, Any] = {
        "class_uid": class_uid,
        "class_name": class_name,
        "category_uid": category_uid,
        "category_name": category_name,
        "severity_id": severity_id,
        "severity": severity_name,
        "activity_id": 1,
        "activity_name": "Create",
        "time": time_ms,
        "metadata": {
            "version": "1.1.0",
            "product": {
                "name": data.get("source", {}).get("name", "Unknown"),
                "vendor_name": data.get("source", {}).get("vendor", "WIA")
            },
            "uid": data.get("id"),
            "original_time": timestamp
        },
        "unmapped": {
            "wia_version": data.get("version"),
            "wia_type": event_type,
            "wia_data": data.get("data")
        }
    }

    # Map MITRE ATT&CK
    mitre = data.get("mitre", {})
    if mitre:
        ocsf["attacks"] = [{
            "tactic": {
                "uid": mitre.get("tactic", ""),
                "name": mitre.get("tactic_name", "")
            },
            "technique": {
                "uid": mitre.get("technique", ""),
                "name": mitre.get("technique_name", "")
            }
        }]

    return ocsf


# ============================================================================
# SIEM Format Conversion
# ============================================================================

def to_splunk_event(event: Union[WiaSecurityEvent, Dict[str, Any]], index: Optional[str] = None) -> Dict[str, Any]:
    """
    Convert WIA Security event to Splunk HEC format.

    Args:
        event: WiaSecurityEvent or dictionary
        index: Optional Splunk index

    Returns:
        Splunk HEC event dictionary
    """
    if isinstance(event, WiaSecurityEvent):
        data = event.to_dict()
    else:
        data = event

    timestamp = data.get("timestamp", "")
    try:
        time_epoch = datetime.fromisoformat(timestamp.replace("Z", "+00:00")).timestamp()
    except (ValueError, AttributeError):
        time_epoch = datetime.utcnow().timestamp()

    result: Dict[str, Any] = {
        "time": time_epoch,
        "host": data.get("context", {}).get("host", {}).get("hostname", "unknown"),
        "source": data.get("source", {}).get("name", "wia-security"),
        "sourcetype": f"wia:security:{data.get('type', 'event')}",
        "event": {
            **data,
            "_wia_version": data.get("version"),
            "_wia_type": data.get("type")
        }
    }

    if index:
        result["index"] = index

    return result


def to_elastic_event(event: Union[WiaSecurityEvent, Dict[str, Any]], index_prefix: str = "wia-security") -> Dict[str, Any]:
    """
    Convert WIA Security event to Elasticsearch document format.

    Args:
        event: WiaSecurityEvent or dictionary
        index_prefix: Index prefix

    Returns:
        Elasticsearch document dictionary
    """
    if isinstance(event, WiaSecurityEvent):
        data = event.to_dict()
    else:
        data = event

    timestamp = data.get("timestamp", "")
    event_type = data.get("type", "event")

    # Generate index name with date
    try:
        dt = datetime.fromisoformat(timestamp.replace("Z", "+00:00"))
        date_str = dt.strftime("%Y.%m.%d")
    except (ValueError, AttributeError):
        date_str = datetime.utcnow().strftime("%Y.%m.%d")

    return {
        "_index": f"{index_prefix}-{event_type}-{date_str}",
        "_id": data.get("id"),
        "_source": {
            "@timestamp": timestamp,
            **to_ecs_event(data)
        }
    }
