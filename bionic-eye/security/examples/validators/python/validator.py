#!/usr/bin/env python3
"""
WIA Security Event Validator
Python implementation for validating WIA Security events
"""

import json
import re
import sys
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional
from datetime import datetime

# Validation patterns
UUID_PATTERN = re.compile(r'^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$', re.IGNORECASE)
ISO8601_PATTERN = re.compile(r'^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(\.\d{3})?Z$')
SEMVER_PATTERN = re.compile(r'^\d+\.\d+\.\d+$')
TACTIC_PATTERN = re.compile(r'^TA\d{4}$')
TECHNIQUE_PATTERN = re.compile(r'^T\d{4}(\.\d{3})?$')
SUB_TECHNIQUE_PATTERN = re.compile(r'^T\d{4}\.\d{3}$')
CVE_PATTERN = re.compile(r'^CVE-\d{4}-\d{4,}$')
CWE_PATTERN = re.compile(r'^CWE-\d+$')

# Valid values
VALID_EVENT_TYPES = ['alert', 'threat_intel', 'vulnerability', 'incident', 'network_event', 'endpoint_event', 'auth_event']
VALID_SOURCE_TYPES = ['siem', 'edr', 'ids', 'firewall', 'scanner', 'custom']
VALID_PRIORITIES = ['critical', 'high', 'medium', 'low', 'info']
VALID_DIRECTIONS = ['inbound', 'outbound', 'internal']
VALID_CLOUD_PROVIDERS = ['aws', 'azure', 'gcp']


@dataclass
class ValidationResult:
    """Validation result container"""
    valid: bool = True
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)


class WiaSecurityValidator:
    """Validator for WIA Security Events"""

    def validate(self, event: Any) -> ValidationResult:
        """Validate a WIA Security Event"""
        result = ValidationResult()

        if not isinstance(event, dict):
            result.valid = False
            result.errors.append('Event must be an object')
            return result

        # Required fields validation
        self._validate_required_fields(event, result)

        # Source validation
        if 'source' in event:
            self._validate_source(event['source'], result)

        # Data validation
        if 'data' in event and 'type' in event:
            self._validate_event_data(event['type'], event['data'], result)

        # Optional fields validation
        if 'context' in event:
            self._validate_context(event['context'], result)

        if 'mitre' in event:
            self._validate_mitre(event['mitre'], result)

        if 'meta' in event:
            self._validate_meta(event['meta'], result)

        # Warnings for recommended fields
        if '$schema' not in event:
            result.warnings.append('Recommended field $schema is missing')

        result.valid = len(result.errors) == 0
        return result

    def _validate_required_fields(self, event: Dict, result: ValidationResult) -> None:
        """Validate required fields"""
        # Version
        if 'version' not in event:
            result.errors.append('Missing required field: version')
        elif not isinstance(event['version'], str) or not SEMVER_PATTERN.match(event['version']):
            result.errors.append('Invalid version format. Expected SemVer (e.g., 1.0.0)')

        # ID
        if 'id' not in event:
            result.errors.append('Missing required field: id')
        elif not isinstance(event['id'], str) or not UUID_PATTERN.match(event['id']):
            result.errors.append('Invalid id format. Expected UUID v4')

        # Type
        if 'type' not in event:
            result.errors.append('Missing required field: type')
        elif event['type'] not in VALID_EVENT_TYPES:
            result.errors.append(f"Invalid event type: {event['type']}. Expected one of: {', '.join(VALID_EVENT_TYPES)}")

        # Timestamp
        if 'timestamp' not in event:
            result.errors.append('Missing required field: timestamp')
        elif not isinstance(event['timestamp'], str) or not ISO8601_PATTERN.match(event['timestamp']):
            result.errors.append('Invalid timestamp format. Expected ISO 8601 (e.g., 2025-12-14T00:00:00.000Z)')

        # Severity
        if 'severity' not in event:
            result.errors.append('Missing required field: severity')
        elif not isinstance(event['severity'], (int, float)) or event['severity'] < 0 or event['severity'] > 10:
            result.errors.append('Invalid severity. Expected number between 0 and 10')

        # Source
        if 'source' not in event:
            result.errors.append('Missing required field: source')

        # Data
        if 'data' not in event:
            result.errors.append('Missing required field: data')
        elif not isinstance(event['data'], dict):
            result.errors.append('Invalid data field. Expected object')

    def _validate_source(self, source: Any, result: ValidationResult) -> None:
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

    def _validate_context(self, context: Any, result: ValidationResult) -> None:
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

    def _validate_mitre(self, mitre: Any, result: ValidationResult) -> None:
        """Validate MITRE ATT&CK mapping"""
        if not isinstance(mitre, dict):
            result.errors.append('Mitre must be an object')
            return

        if 'tactic' in mitre and isinstance(mitre['tactic'], str):
            if not TACTIC_PATTERN.match(mitre['tactic']):
                result.errors.append(f"Invalid MITRE tactic ID format: {mitre['tactic']}. Expected TAxxxx")

        if 'technique' in mitre and isinstance(mitre['technique'], str):
            if not TECHNIQUE_PATTERN.match(mitre['technique']):
                result.errors.append(f"Invalid MITRE technique ID format: {mitre['technique']}. Expected Txxxx or Txxxx.xxx")

        if 'sub_technique' in mitre and isinstance(mitre['sub_technique'], str):
            if not SUB_TECHNIQUE_PATTERN.match(mitre['sub_technique']):
                result.errors.append(f"Invalid MITRE sub-technique ID format: {mitre['sub_technique']}. Expected Txxxx.xxx")

    def _validate_meta(self, meta: Any, result: ValidationResult) -> None:
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

    def _validate_event_data(self, event_type: str, data: Dict, result: ValidationResult) -> None:
        """Validate event-specific data"""
        validators = {
            'alert': self._validate_alert_data,
            'threat_intel': self._validate_threat_intel_data,
            'vulnerability': self._validate_vulnerability_data,
            'incident': self._validate_incident_data,
            'network_event': self._validate_network_event_data,
            'endpoint_event': self._validate_endpoint_event_data,
            'auth_event': self._validate_auth_event_data,
        }

        if event_type in validators:
            validators[event_type](data, result)

    def _validate_alert_data(self, data: Dict, result: ValidationResult) -> None:
        """Validate alert data"""
        required_fields = ['alert_id', 'title', 'category', 'status', 'priority']
        for field in required_fields:
            if field not in data:
                result.errors.append(f'Missing required field in alert data: {field}')

        valid_categories = ['malware', 'intrusion', 'policy', 'reconnaissance', 'other']
        if 'category' in data and data['category'] not in valid_categories:
            result.errors.append(f"Invalid alert category: {data['category']}")

        valid_statuses = ['new', 'investigating', 'resolved', 'false_positive', 'closed']
        if 'status' in data and data['status'] not in valid_statuses:
            result.errors.append(f"Invalid alert status: {data['status']}")

        if 'priority' in data and data['priority'] not in VALID_PRIORITIES:
            result.errors.append(f"Invalid alert priority: {data['priority']}")

    def _validate_threat_intel_data(self, data: Dict, result: ValidationResult) -> None:
        """Validate threat intelligence data"""
        required_fields = ['threat_type', 'threat_name', 'status']
        for field in required_fields:
            if field not in data:
                result.errors.append(f'Missing required field in threat_intel data: {field}')

        valid_types = ['malware', 'apt', 'campaign', 'botnet', 'ransomware', 'phishing']
        if 'threat_type' in data and data['threat_type'] not in valid_types:
            result.errors.append(f"Invalid threat_type: {data['threat_type']}")

        valid_statuses = ['active', 'inactive', 'unknown']
        if 'status' in data and data['status'] not in valid_statuses:
            result.errors.append(f"Invalid threat_intel status: {data['status']}")

    def _validate_vulnerability_data(self, data: Dict, result: ValidationResult) -> None:
        """Validate vulnerability data"""
        required_fields = ['vuln_id', 'title', 'cvss']
        for field in required_fields:
            if field not in data:
                result.errors.append(f'Missing required field in vulnerability data: {field}')

        if 'vuln_id' in data and isinstance(data['vuln_id'], str):
            if not CVE_PATTERN.match(data['vuln_id']):
                result.errors.append(f"Invalid CVE format: {data['vuln_id']}. Expected CVE-YYYY-NNNNN")

        if 'cvss' in data and isinstance(data['cvss'], dict):
            cvss = data['cvss']
            if 'score' in cvss:
                score = cvss['score']
                if not isinstance(score, (int, float)) or score < 0 or score > 10:
                    result.errors.append('cvss.score must be a number between 0 and 10')

            valid_severities = ['critical', 'high', 'medium', 'low', 'none']
            if 'severity' in cvss and cvss['severity'] not in valid_severities:
                result.errors.append(f"Invalid CVSS severity: {cvss['severity']}")

        if 'cwe' in data and isinstance(data['cwe'], list):
            for cwe in data['cwe']:
                if not CWE_PATTERN.match(cwe):
                    result.errors.append(f"Invalid CWE format: {cwe}. Expected CWE-NNN")

    def _validate_incident_data(self, data: Dict, result: ValidationResult) -> None:
        """Validate incident data"""
        required_fields = ['incident_id', 'title', 'category', 'status', 'priority']
        for field in required_fields:
            if field not in data:
                result.errors.append(f'Missing required field in incident data: {field}')

        valid_categories = ['malware', 'phishing', 'ransomware', 'data_breach', 'ddos',
                          'unauthorized_access', 'insider_threat', 'apt', 'other']
        if 'category' in data and data['category'] not in valid_categories:
            result.errors.append(f"Invalid incident category: {data['category']}")

        valid_statuses = ['new', 'triaging', 'investigating', 'containing',
                        'eradicating', 'recovering', 'closed']
        if 'status' in data and data['status'] not in valid_statuses:
            result.errors.append(f"Invalid incident status: {data['status']}")

    def _validate_network_event_data(self, data: Dict, result: ValidationResult) -> None:
        """Validate network event data"""
        required_fields = ['event_type', 'protocol', 'source', 'destination']
        for field in required_fields:
            if field not in data:
                result.errors.append(f'Missing required field in network_event data: {field}')

        valid_types = ['connection', 'dns', 'http', 'tls', 'smtp', 'ftp', 'ssh', 'rdp', 'custom']
        if 'event_type' in data and data['event_type'] not in valid_types:
            result.errors.append(f"Invalid network event_type: {data['event_type']}")

        valid_protocols = ['TCP', 'UDP', 'ICMP', 'GRE', 'ESP']
        if 'protocol' in data and data['protocol'] not in valid_protocols:
            result.errors.append(f"Invalid network protocol: {data['protocol']}")

    def _validate_endpoint_event_data(self, data: Dict, result: ValidationResult) -> None:
        """Validate endpoint event data"""
        required_fields = ['event_type', 'host']
        for field in required_fields:
            if field not in data:
                result.errors.append(f'Missing required field in endpoint_event data: {field}')

        valid_types = [
            'process_creation', 'process_termination', 'file_create', 'file_modify', 'file_delete',
            'registry_create', 'registry_modify', 'registry_delete', 'network_connection',
            'dll_load', 'driver_load', 'service_install'
        ]
        if 'event_type' in data and data['event_type'] not in valid_types:
            result.errors.append(f"Invalid endpoint event_type: {data['event_type']}")

    def _validate_auth_event_data(self, data: Dict, result: ValidationResult) -> None:
        """Validate authentication event data"""
        required_fields = ['event_type', 'result', 'user', 'target']
        for field in required_fields:
            if field not in data:
                result.errors.append(f'Missing required field in auth_event data: {field}')

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


def validate_file(filepath: str) -> ValidationResult:
    """Validate a JSON file"""
    with open(filepath, 'r', encoding='utf-8') as f:
        event = json.load(f)

    validator = WiaSecurityValidator()
    return validator.validate(event)


def main():
    """CLI entry point"""
    if len(sys.argv) < 2:
        print('Usage: python validator.py <event.json>')
        sys.exit(1)

    filepath = sys.argv[1]

    try:
        result = validate_file(filepath)

        print('\n=== WIA Security Event Validation ===\n')
        print(f'File: {filepath}')
        print(f'Valid: {"YES ✓" if result.valid else "NO ✗"}\n')

        if result.errors:
            print('Errors:')
            for error in result.errors:
                print(f'  ✗ {error}')

        if result.warnings:
            print('\nWarnings:')
            for warning in result.warnings:
                print(f'  ⚠ {warning}')

        if result.valid and not result.warnings:
            print('No issues found.')

        sys.exit(0 if result.valid else 1)

    except json.JSONDecodeError as e:
        print(f'JSON parsing error: {e}')
        sys.exit(1)
    except FileNotFoundError:
        print(f'File not found: {filepath}')
        sys.exit(1)
    except Exception as e:
        print(f'Error: {e}')
        sys.exit(1)


if __name__ == '__main__':
    main()
