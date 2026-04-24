//! WIA Security Event Converter

use serde_json::{json, Value};
use uuid::Uuid;

use crate::types::WiaSecurityEvent;

// ============================================================================
// STIX 2.1 Conversion
// ============================================================================

/// Convert WIA Security event to STIX 2.1 Bundle
pub fn to_stix_bundle(event: &WiaSecurityEvent) -> Value {
    let mut objects: Vec<Value> = Vec::new();
    let timestamp = &event.timestamp;

    match event.event_type {
        crate::types::EventType::ThreatIntel => {
            if let Some(indicators) = event.data.get("indicators").and_then(|v| v.as_array()) {
                for indicator in indicators {
                    objects.push(to_stix_indicator(indicator, timestamp));
                }
            }

            let threat_type = event.data.get("threat_type").and_then(|v| v.as_str());
            if matches!(threat_type, Some("malware") | Some("ransomware")) {
                objects.push(json!({
                    "type": "malware",
                    "spec_version": "2.1",
                    "id": format!("malware--{}", Uuid::new_v4()),
                    "created": timestamp,
                    "modified": timestamp,
                    "name": event.data.get("threat_name").and_then(|v| v.as_str()).unwrap_or("Unknown"),
                    "malware_types": [threat_type.unwrap_or("malware")],
                    "is_family": event.data.get("threat_family").is_some()
                }));
            }

            if threat_type == Some("apt") {
                objects.push(json!({
                    "type": "threat-actor",
                    "spec_version": "2.1",
                    "id": format!("threat-actor--{}", Uuid::new_v4()),
                    "created": timestamp,
                    "modified": timestamp,
                    "name": event.data.get("threat_name").and_then(|v| v.as_str()).unwrap_or("Unknown"),
                    "threat_actor_types": ["nation-state"],
                    "primary_motivation": "organizational-gain"
                }));
            }
        }
        crate::types::EventType::Alert => {
            objects.push(json!({
                "type": "sighting",
                "spec_version": "2.1",
                "id": format!("sighting--{}", Uuid::new_v4()),
                "created": timestamp,
                "modified": timestamp,
                "first_seen": event.data.get("first_seen").and_then(|v| v.as_str()).unwrap_or(timestamp),
                "last_seen": event.data.get("last_seen").and_then(|v| v.as_str()).unwrap_or(timestamp),
                "count": event.data.get("count").and_then(|v| v.as_u64()).unwrap_or(1),
                "description": event.data.get("description")
            }));

            if let Some(indicators) = event.data.get("indicators").and_then(|v| v.as_array()) {
                for indicator in indicators {
                    objects.push(to_stix_indicator(indicator, timestamp));
                }
            }
        }
        crate::types::EventType::Vulnerability => {
            objects.push(json!({
                "type": "vulnerability",
                "spec_version": "2.1",
                "id": format!("vulnerability--{}", Uuid::new_v4()),
                "created": timestamp,
                "modified": timestamp,
                "name": event.data.get("vuln_id").and_then(|v| v.as_str()).unwrap_or("Unknown"),
                "description": event.data.get("description").or_else(|| event.data.get("title")),
                "external_references": [{
                    "source_name": "cve",
                    "external_id": event.data.get("vuln_id").and_then(|v| v.as_str()).unwrap_or("")
                }]
            }));
        }
        _ => {}
    }

    json!({
        "type": "bundle",
        "id": format!("bundle--{}", Uuid::new_v4()),
        "objects": objects
    })
}

/// Convert indicator to STIX indicator
pub fn to_stix_indicator(indicator: &Value, timestamp: &str) -> Value {
    let ind_type = indicator
        .get("type")
        .and_then(|v| v.as_str())
        .unwrap_or("");
    let ind_value = indicator
        .get("value")
        .and_then(|v| v.as_str())
        .unwrap_or("");

    let pattern = match ind_type {
        "ip" => format!("[ipv4-addr:value = '{}']", ind_value),
        "domain" => format!("[domain-name:value = '{}']", ind_value),
        "url" => format!("[url:value = '{}']", ind_value),
        "hash" | "file_hash_md5" => format!("[file:hashes.MD5 = '{}']", ind_value),
        "file_hash_sha1" => format!("[file:hashes.'SHA-1' = '{}']", ind_value),
        "file_hash_sha256" => format!("[file:hashes.'SHA-256' = '{}']", ind_value),
        "email" => format!("[email-addr:value = '{}']", ind_value),
        _ => format!("[x-custom:value = '{}']", ind_value),
    };

    json!({
        "type": "indicator",
        "spec_version": "2.1",
        "id": format!("indicator--{}", Uuid::new_v4()),
        "created": timestamp,
        "modified": timestamp,
        "name": indicator.get("context").and_then(|v| v.as_str()).unwrap_or(&format!("{}: {}", ind_type, ind_value)),
        "pattern": pattern,
        "pattern_type": "stix",
        "valid_from": indicator.get("first_seen").and_then(|v| v.as_str()).unwrap_or(timestamp)
    })
}

// ============================================================================
// ECS Conversion
// ============================================================================

/// Convert WIA Security event to ECS format
pub fn to_ecs_event(event: &WiaSecurityEvent) -> Value {
    let kind = match event.event_type {
        crate::types::EventType::Alert => "alert",
        crate::types::EventType::ThreatIntel => "enrichment",
        crate::types::EventType::Vulnerability => "state",
        crate::types::EventType::Incident => "alert",
        _ => "event",
    };

    let category = match event.event_type {
        crate::types::EventType::Alert => vec!["intrusion_detection"],
        crate::types::EventType::ThreatIntel => vec!["threat"],
        crate::types::EventType::Vulnerability => vec!["vulnerability"],
        crate::types::EventType::Incident => vec!["intrusion_detection"],
        crate::types::EventType::NetworkEvent => vec!["network"],
        crate::types::EventType::EndpointEvent => vec!["host"],
        crate::types::EventType::AuthEvent => vec!["authentication"],
    };

    let mut ecs = json!({
        "@timestamp": event.timestamp,
        "event": {
            "id": event.id.to_string(),
            "kind": kind,
            "category": category,
            "type": ["info"],
            "severity": event.severity,
            "created": event.timestamp
        },
        "wia": {
            "version": event.version,
            "type": event.event_type.to_string(),
            "data": event.data
        }
    });

    // Map context
    if let Some(ref context) = event.context {
        if let Some(ref host) = context.host {
            ecs["host"] = json!({
                "hostname": host.hostname,
                "ip": host.ip,
                "mac": host.mac
            });
        }

        if let Some(ref user) = context.user {
            ecs["user"] = json!({
                "name": user.name,
                "domain": user.domain,
                "email": user.email
            });
        }
    }

    // Map MITRE
    if let Some(ref mitre) = event.mitre {
        ecs["threat"] = json!({
            "framework": "MITRE ATT&CK",
            "tactic": {
                "id": mitre.tactic.as_ref().map(|t| vec![t.clone()]),
                "name": mitre.tactic_name.as_ref().map(|n| vec![n.clone()])
            },
            "technique": {
                "id": mitre.technique.as_ref().map(|t| vec![t.clone()]),
                "name": mitre.technique_name.as_ref().map(|n| vec![n.clone()])
            }
        });
    }

    // Map meta
    if let Some(ref meta) = event.meta {
        if let Some(ref tags) = meta.tags {
            ecs["tags"] = json!(tags);
        }
        if let Some(ref labels) = meta.labels {
            ecs["labels"] = json!(labels);
        }
    }

    ecs
}

// ============================================================================
// OCSF Conversion
// ============================================================================

/// Convert WIA Security event to OCSF format
pub fn to_ocsf_event(event: &WiaSecurityEvent) -> Value {
    let (class_uid, class_name, category_uid, category_name) = match event.event_type {
        crate::types::EventType::Alert => (2001, "Security Finding", 2, "Findings"),
        crate::types::EventType::ThreatIntel => (2001, "Security Finding", 2, "Findings"),
        crate::types::EventType::Vulnerability => (2002, "Vulnerability Finding", 2, "Findings"),
        crate::types::EventType::Incident => (2003, "Incident Finding", 2, "Findings"),
        crate::types::EventType::NetworkEvent => (4001, "Network Activity", 4, "Network Activity"),
        crate::types::EventType::EndpointEvent => (1001, "Process Activity", 1, "System Activity"),
        crate::types::EventType::AuthEvent => (3001, "Authentication", 3, "Identity & Access"),
    };

    let (severity_id, severity_name) = if event.severity == 0.0 {
        (0, "Unknown")
    } else if event.severity <= 2.0 {
        (1, "Informational")
    } else if event.severity <= 4.0 {
        (2, "Low")
    } else if event.severity <= 6.0 {
        (3, "Medium")
    } else if event.severity <= 8.0 {
        (4, "High")
    } else {
        (5, "Critical")
    };

    let mut ocsf = json!({
        "class_uid": class_uid,
        "class_name": class_name,
        "category_uid": category_uid,
        "category_name": category_name,
        "severity_id": severity_id,
        "severity": severity_name,
        "activity_id": 1,
        "activity_name": "Create",
        "metadata": {
            "version": "1.1.0",
            "product": {
                "name": event.source.name,
                "vendor_name": event.source.vendor.as_deref().unwrap_or("WIA")
            },
            "uid": event.id.to_string(),
            "original_time": event.timestamp
        },
        "unmapped": {
            "wia_version": event.version,
            "wia_type": event.event_type.to_string(),
            "wia_data": event.data
        }
    });

    // Map MITRE ATT&CK
    if let Some(ref mitre) = event.mitre {
        ocsf["attacks"] = json!([{
            "tactic": {
                "uid": mitre.tactic,
                "name": mitre.tactic_name
            },
            "technique": {
                "uid": mitre.technique,
                "name": mitre.technique_name
            }
        }]);
    }

    ocsf
}

// ============================================================================
// SIEM Format Conversion
// ============================================================================

/// Convert WIA Security event to Splunk HEC format
pub fn to_splunk_event(event: &WiaSecurityEvent, index: Option<&str>) -> Value {
    let time = chrono::DateTime::parse_from_rfc3339(&event.timestamp)
        .map(|dt| dt.timestamp() as f64)
        .unwrap_or_else(|_| chrono::Utc::now().timestamp() as f64);

    let hostname = event
        .context
        .as_ref()
        .and_then(|c| c.host.as_ref())
        .and_then(|h| h.hostname.as_deref())
        .unwrap_or("unknown");

    let mut result = json!({
        "time": time,
        "host": hostname,
        "source": event.source.name,
        "sourcetype": format!("wia:security:{}", event.event_type),
        "event": {
            "version": event.version,
            "id": event.id.to_string(),
            "type": event.event_type.to_string(),
            "timestamp": event.timestamp,
            "severity": event.severity,
            "source": event.source,
            "data": event.data,
            "_wia_version": event.version,
            "_wia_type": event.event_type.to_string()
        }
    });

    if let Some(idx) = index {
        result["index"] = json!(idx);
    }

    result
}

/// Convert WIA Security event to Elasticsearch document format
pub fn to_elastic_event(event: &WiaSecurityEvent, index_prefix: &str) -> Value {
    let date_str = chrono::DateTime::parse_from_rfc3339(&event.timestamp)
        .map(|dt| dt.format("%Y.%m.%d").to_string())
        .unwrap_or_else(|_| chrono::Utc::now().format("%Y.%m.%d").to_string());

    json!({
        "_index": format!("{}-{}-{}", index_prefix, event.event_type, date_str),
        "_id": event.id.to_string(),
        "_source": {
            "@timestamp": event.timestamp,
            ..to_ecs_event(event).as_object().cloned().unwrap_or_default()
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builder::AlertBuilder;
    use crate::types::{Source, SourceType};

    #[test]
    fn test_to_stix_bundle() {
        let event = AlertBuilder::new()
            .source(Source::new(SourceType::Siem, "Splunk"))
            .alert_id("ALERT-001")
            .title("Test Alert")
            .category("malware")
            .status("new")
            .priority("high")
            .build()
            .unwrap();

        let bundle = to_stix_bundle(&event);
        assert_eq!(bundle["type"], "bundle");
    }
}
