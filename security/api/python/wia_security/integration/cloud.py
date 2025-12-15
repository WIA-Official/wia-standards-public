"""
WIA Security Cloud Integration
AWS Security Hub, Azure Sentinel, GCP Security Command Center
"""

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
class CloudSecurityConfig:
    """Base cloud security configuration."""
    region: Optional[str] = None
    project_id: Optional[str] = None
    subscription_id: Optional[str] = None
    tenant_id: Optional[str] = None
    client_id: Optional[str] = None
    client_secret: Optional[str] = None
    access_key_id: Optional[str] = None
    secret_access_key: Optional[str] = None
    session_token: Optional[str] = None


@dataclass
class CloudFinding:
    """Cloud security finding."""
    id: str
    title: str
    description: str
    severity: str  # INFORMATIONAL, LOW, MEDIUM, HIGH, CRITICAL
    type: str
    resource_type: Optional[str] = None
    resource_id: Optional[str] = None
    created_at: str = ""
    updated_at: str = ""
    status: str = "NEW"  # NEW, ACTIVE, RESOLVED, SUPPRESSED


# ============================================================================
# AWS Security Hub Integration
# ============================================================================

@dataclass
class AwsSecurityHubConfig(CloudSecurityConfig):
    """AWS Security Hub configuration."""
    account_id: str = ""
    region: str = "us-east-1"


class AwsSecurityHubClient:
    """AWS Security Hub client."""

    def __init__(self, config: AwsSecurityHubConfig):
        if not HAS_HTTPX:
            raise ImportError("httpx package required. Install with: pip install httpx")

        self.config = config
        self._endpoint = f"https://securityhub.{config.region}.amazonaws.com"
        self._client = httpx.Client(timeout=30.0)

    def close(self) -> None:
        self._client.close()

    def __enter__(self) -> "AwsSecurityHubClient":
        return self

    def __exit__(self, *args) -> None:
        self.close()

    def import_finding(self, event: WiaSecurityEvent) -> str:
        """Import finding from WIA Security event."""
        finding = self._to_aws_finding(event)

        response = self._signed_request(
            "POST",
            "/findings/import",
            {"Findings": [finding]},
        )

        return finding["Id"]

    def batch_import_findings(
        self,
        events: List[WiaSecurityEvent],
    ) -> Dict[str, int]:
        """Batch import findings."""
        findings = [self._to_aws_finding(e) for e in events]

        response = self._signed_request(
            "POST",
            "/findings/import",
            {"Findings": findings},
        )

        return {
            "success_count": response.get("SuccessCount", 0),
            "failed_count": response.get("FailedCount", 0),
        }

    def get_findings(
        self,
        severity_label: Optional[List[str]] = None,
        type: Optional[List[str]] = None,
        resource_type: Optional[List[str]] = None,
        max_results: int = 100,
    ) -> List[CloudFinding]:
        """Get findings."""
        filters: Dict[str, Any] = {}

        if severity_label:
            filters["SeverityLabel"] = [
                {"Value": s, "Comparison": "EQUALS"}
                for s in severity_label
            ]

        if type:
            filters["Type"] = [
                {"Value": t, "Comparison": "PREFIX"}
                for t in type
            ]

        response = self._signed_request(
            "POST",
            "/findings",
            {"Filters": filters, "MaxResults": max_results},
        )

        return [
            self._from_aws_finding(f)
            for f in response.get("Findings", [])
        ]

    def update_finding(
        self,
        finding_id: str,
        product_arn: str,
        note: Optional[str] = None,
        severity: Optional[str] = None,
        status: Optional[str] = None,
    ) -> None:
        """Update finding."""
        finding_identifier = {
            "Id": finding_id,
            "ProductArn": product_arn,
        }

        update: Dict[str, Any] = {}

        if note:
            update["Note"] = {
                "Text": note,
                "UpdatedBy": "WIA Security",
            }

        if severity:
            update["Severity"] = {"Label": severity}

        if status:
            update["Workflow"] = {"Status": status}

        self._signed_request(
            "PATCH",
            "/findings",
            {"FindingIdentifiers": [finding_identifier], **update},
        )

    def _to_aws_finding(self, event: WiaSecurityEvent) -> Dict[str, Any]:
        now = datetime.now().isoformat() + "Z"
        product_arn = (
            f"arn:aws:securityhub:{self.config.region}:"
            f"{self.config.account_id}:product/"
            f"{self.config.account_id}/wia-security"
        )

        resources = []
        if event.context and event.context.host:
            resource_id = (
                event.context.host.hostname
                or (event.context.host.ip[0] if event.context.host.ip else None)
                or "unknown"
            )
            resources.append({
                "Type": "AwsEc2Instance",
                "Id": resource_id,
                "Details": {
                    "Other": {
                        "hostname": event.context.host.hostname or "",
                        "ip": ",".join(event.context.host.ip or []),
                    }
                },
            })

        if not resources:
            resources.append({"Type": "Other", "Id": event.id})

        return {
            "SchemaVersion": "2018-10-08",
            "Id": event.id,
            "ProductArn": product_arn,
            "GeneratorId": "wia-security",
            "AwsAccountId": self.config.account_id,
            "Types": [self._map_event_type(event.type.value)],
            "CreatedAt": event.timestamp,
            "UpdatedAt": now,
            "Severity": {
                "Label": self._map_severity(event.severity),
                "Original": str(event.severity),
            },
            "Title": event.description[:256],
            "Description": event.description,
            "Resources": resources,
            "Workflow": {"Status": "NEW"},
            "RecordState": "ACTIVE",
        }

    def _from_aws_finding(self, finding: Dict[str, Any]) -> CloudFinding:
        return CloudFinding(
            id=finding.get("Id", ""),
            title=finding.get("Title", ""),
            description=finding.get("Description", ""),
            severity=finding.get("Severity", {}).get("Label", "MEDIUM"),
            type=finding.get("Types", ["Unknown"])[0],
            resource_type=finding.get("Resources", [{}])[0].get("Type"),
            resource_id=finding.get("Resources", [{}])[0].get("Id"),
            created_at=finding.get("CreatedAt", ""),
            updated_at=finding.get("UpdatedAt", ""),
            status=finding.get("Workflow", {}).get("Status", "NEW"),
        )

    def _map_event_type(self, type: str) -> str:
        mapping = {
            "alert": "Software and Configuration Checks/Vulnerabilities/CVE",
            "threat_intel": "TTPs/Initial Access",
            "vulnerability": "Software and Configuration Checks/Vulnerabilities/CVE",
            "incident": "Unusual Behaviors/VM/Intrusion",
            "network_event": "Effects/Network Effects",
            "endpoint_event": "Unusual Behaviors/Process",
            "auth_event": "Sensitive Data Identifications/Authentication",
        }
        return mapping.get(type, "Other")

    def _map_severity(self, severity: float) -> str:
        if severity >= 9:
            return "CRITICAL"
        if severity >= 7:
            return "HIGH"
        if severity >= 4:
            return "MEDIUM"
        if severity >= 1:
            return "LOW"
        return "INFORMATIONAL"

    def _signed_request(
        self,
        method: str,
        path: str,
        body: Dict[str, Any],
    ) -> Dict[str, Any]:
        """Make signed request to AWS."""
        # In production, use boto3 or proper AWS Signature V4
        import json

        headers = {
            "Content-Type": "application/json",
            "Host": f"securityhub.{self.config.region}.amazonaws.com",
        }

        response = self._client.request(
            method,
            f"{self._endpoint}{path}",
            headers=headers,
            json=body,
        )
        response.raise_for_status()

        return response.json()


# ============================================================================
# Azure Sentinel Integration
# ============================================================================

@dataclass
class AzureSentinelConfig(CloudSecurityConfig):
    """Azure Sentinel configuration."""
    workspace_id: str = ""
    subscription_id: str = ""
    resource_group: str = ""


class AzureSentinelClient:
    """Azure Sentinel client."""

    def __init__(self, config: AzureSentinelConfig):
        if not HAS_HTTPX:
            raise ImportError("httpx package required. Install with: pip install httpx")

        self.config = config
        self._access_token: Optional[str] = None
        self._token_expiry: Optional[datetime] = None
        self._client = httpx.Client(timeout=30.0)

    def close(self) -> None:
        self._client.close()

    def __enter__(self) -> "AzureSentinelClient":
        return self

    def __exit__(self, *args) -> None:
        self.close()

    def create_incident(self, event: WiaSecurityEvent) -> str:
        """Create incident from WIA Security event."""
        self._ensure_token()

        incident_id = f"wia-{event.id}"
        url = self._build_url(f"/incidents/{incident_id}")

        incident = {
            "properties": {
                "title": event.description[:256],
                "description": event.description,
                "severity": self._map_severity(event.severity),
                "status": "New",
                "labels": [
                    {"labelName": "wia-security"},
                    {"labelName": event.type.value},
                ],
            }
        }

        response = self._client.put(
            url,
            headers={
                "Authorization": f"Bearer {self._access_token}",
                "Content-Type": "application/json",
            },
            json=incident,
        )
        response.raise_for_status()

        return response.json()["name"]

    def list_incidents(
        self,
        severity: Optional[List[str]] = None,
        status: Optional[List[str]] = None,
        top: int = 100,
    ) -> List[CloudFinding]:
        """List incidents."""
        self._ensure_token()

        url = self._build_url("/incidents")
        params = {"$top": str(top)}

        if severity:
            filter_str = " or ".join(
                f"properties/severity eq '{s}'" for s in severity
            )
            params["$filter"] = filter_str

        response = self._client.get(
            url,
            params=params,
            headers={"Authorization": f"Bearer {self._access_token}"},
        )
        response.raise_for_status()

        return [
            self._from_azure_incident(i)
            for i in response.json().get("value", [])
        ]

    def update_incident(
        self,
        incident_id: str,
        status: Optional[str] = None,
        severity: Optional[str] = None,
        owner: Optional[str] = None,
        classification: Optional[str] = None,
    ) -> None:
        """Update incident."""
        self._ensure_token()

        url = self._build_url(f"/incidents/{incident_id}")

        # Get current incident
        response = self._client.get(
            url,
            headers={"Authorization": f"Bearer {self._access_token}"},
        )
        response.raise_for_status()
        current = response.json()

        # Update properties
        if status:
            current["properties"]["status"] = status
        if severity:
            current["properties"]["severity"] = severity
        if classification:
            current["properties"]["classification"] = classification
        if owner:
            current["properties"]["owner"] = {"assignedTo": owner}

        response = self._client.put(
            url,
            headers={
                "Authorization": f"Bearer {self._access_token}",
                "Content-Type": "application/json",
            },
            json=current,
        )
        response.raise_for_status()

    def add_incident_comment(self, incident_id: str, message: str) -> None:
        """Add comment to incident."""
        self._ensure_token()

        comment_id = f"wia-comment-{int(datetime.now().timestamp())}"
        url = self._build_url(f"/incidents/{incident_id}/comments/{comment_id}")

        response = self._client.put(
            url,
            headers={
                "Authorization": f"Bearer {self._access_token}",
                "Content-Type": "application/json",
            },
            json={"properties": {"message": message}},
        )
        response.raise_for_status()

    def _from_azure_incident(self, incident: Dict[str, Any]) -> CloudFinding:
        props = incident.get("properties", {})
        return CloudFinding(
            id=incident.get("name", ""),
            title=props.get("title", ""),
            description=props.get("description", ""),
            severity=props.get("severity", "Medium").upper(),
            type="AzureSentinelIncident",
            created_at=props.get("createdTimeUtc", ""),
            updated_at=props.get("lastModifiedTimeUtc", ""),
            status=self._map_azure_status(props.get("status", "")),
        )

    def _map_severity(self, severity: float) -> str:
        if severity >= 9:
            return "High"
        if severity >= 6:
            return "Medium"
        if severity >= 3:
            return "Low"
        return "Informational"

    def _map_azure_status(self, status: str) -> str:
        mapping = {
            "new": "NEW",
            "active": "ACTIVE",
            "closed": "RESOLVED",
        }
        return mapping.get(status.lower(), "NEW")

    def _build_url(self, path: str) -> str:
        return (
            f"https://management.azure.com/subscriptions/"
            f"{self.config.subscription_id}/resourceGroups/"
            f"{self.config.resource_group}/providers/"
            f"Microsoft.OperationalInsights/workspaces/"
            f"{self.config.workspace_id}/providers/"
            f"Microsoft.SecurityInsights{path}?api-version=2023-02-01"
        )

    def _ensure_token(self) -> None:
        if self._access_token and self._token_expiry:
            if self._token_expiry > datetime.now():
                return

        token_url = (
            f"https://login.microsoftonline.com/"
            f"{self.config.tenant_id}/oauth2/v2.0/token"
        )

        response = self._client.post(
            token_url,
            data={
                "client_id": self.config.client_id,
                "client_secret": self.config.client_secret,
                "scope": "https://management.azure.com/.default",
                "grant_type": "client_credentials",
            },
        )
        response.raise_for_status()

        data = response.json()
        self._access_token = data["access_token"]
        self._token_expiry = datetime.now()


# ============================================================================
# GCP Security Command Center Integration
# ============================================================================

@dataclass
class GcpSccConfig(CloudSecurityConfig):
    """GCP Security Command Center configuration."""
    project_id: str = ""
    organization_id: str = ""
    source_id: Optional[str] = None


class GcpSccClient:
    """GCP Security Command Center client."""

    def __init__(self, config: GcpSccConfig):
        if not HAS_HTTPX:
            raise ImportError("httpx package required. Install with: pip install httpx")

        self.config = config
        self._access_token: Optional[str] = None
        self._token_expiry: Optional[datetime] = None
        self._client = httpx.Client(timeout=30.0)

    def close(self) -> None:
        self._client.close()

    def __enter__(self) -> "GcpSccClient":
        return self

    def __exit__(self, *args) -> None:
        self.close()

    def create_finding(self, event: WiaSecurityEvent) -> str:
        """Create finding from WIA Security event."""
        self._ensure_token()

        source_id = self.config.source_id or "wia-security"
        finding_id = event.id.replace("-", "_")
        parent = f"organizations/{self.config.organization_id}/sources/{source_id}"
        url = (
            f"https://securitycenter.googleapis.com/v1/"
            f"{parent}/findings?findingId={finding_id}"
        )

        finding = {
            "state": "ACTIVE",
            "category": event.type.value,
            "severity": self._map_severity(event.severity),
            "eventTime": event.timestamp,
            "sourceProperties": {
                "wia_event_id": {"stringValue": event.id},
                "wia_event_type": {"stringValue": event.type.value},
                "description": {"stringValue": event.description},
            },
        }

        response = self._client.post(
            url,
            headers={
                "Authorization": f"Bearer {self._access_token}",
                "Content-Type": "application/json",
            },
            json=finding,
        )
        response.raise_for_status()

        return response.json()["name"]

    def list_findings(
        self,
        category: Optional[str] = None,
        severity: Optional[List[str]] = None,
        state: Optional[str] = None,
        page_size: int = 100,
    ) -> List[CloudFinding]:
        """List findings."""
        self._ensure_token()

        parent = f"organizations/{self.config.organization_id}/sources/-"
        url = f"https://securitycenter.googleapis.com/v1/{parent}/findings"

        params: Dict[str, str] = {"pageSize": str(page_size)}

        filter_parts = []
        if category:
            filter_parts.append(f'category="{category}"')
        if state:
            filter_parts.append(f'state="{state}"')
        if severity:
            sev_filter = " OR ".join(f'severity="{s}"' for s in severity)
            filter_parts.append(f"({sev_filter})")

        if filter_parts:
            params["filter"] = " AND ".join(filter_parts)

        response = self._client.get(
            url,
            params=params,
            headers={"Authorization": f"Bearer {self._access_token}"},
        )
        response.raise_for_status()

        return [
            self._from_gcp_finding(r["finding"])
            for r in response.json().get("listFindingsResults", [])
        ]

    def update_finding_state(
        self,
        finding_name: str,
        state: str,  # ACTIVE or INACTIVE
    ) -> None:
        """Update finding state."""
        self._ensure_token()

        url = f"https://securitycenter.googleapis.com/v1/{finding_name}:setState"

        response = self._client.post(
            url,
            headers={
                "Authorization": f"Bearer {self._access_token}",
                "Content-Type": "application/json",
            },
            json={
                "state": state,
                "startTime": datetime.now().isoformat() + "Z",
            },
        )
        response.raise_for_status()

    def add_security_marks(
        self,
        finding_name: str,
        marks: Dict[str, str],
    ) -> None:
        """Add security marks."""
        self._ensure_token()

        url = (
            f"https://securitycenter.googleapis.com/v1/"
            f"{finding_name}/securityMarks"
        )

        response = self._client.patch(
            url,
            headers={
                "Authorization": f"Bearer {self._access_token}",
                "Content-Type": "application/json",
            },
            json={"marks": marks},
        )
        response.raise_for_status()

    def _from_gcp_finding(self, finding: Dict[str, Any]) -> CloudFinding:
        source_props = finding.get("sourceProperties", {})
        description = source_props.get("description", {}).get("stringValue", "")

        return CloudFinding(
            id=finding.get("name", ""),
            title=finding.get("category", ""),
            description=description,
            severity=finding.get("severity", "MEDIUM"),
            type=finding.get("category", ""),
            resource_id=finding.get("resourceName"),
            created_at=finding.get("createTime", ""),
            updated_at=finding.get("eventTime", ""),
            status="ACTIVE" if finding.get("state") == "ACTIVE" else "RESOLVED",
        )

    def _map_severity(self, severity: float) -> str:
        if severity >= 9:
            return "CRITICAL"
        if severity >= 7:
            return "HIGH"
        if severity >= 4:
            return "MEDIUM"
        return "LOW"

    def _ensure_token(self) -> None:
        if self._access_token and self._token_expiry:
            if self._token_expiry > datetime.now():
                return

        # Try to get token from metadata server (GCE/GKE)
        try:
            response = self._client.get(
                "http://metadata.google.internal/computeMetadata/v1/"
                "instance/service-accounts/default/token",
                headers={"Metadata-Flavor": "Google"},
            )

            if response.status_code == 200:
                data = response.json()
                self._access_token = data["access_token"]
                self._token_expiry = datetime.now()
                return
        except Exception:
            pass

        raise RuntimeError(
            "GCP authentication required. Please provide credentials."
        )


# ============================================================================
# Factory Functions
# ============================================================================

def create_aws_security_hub_client(
    config: AwsSecurityHubConfig,
) -> AwsSecurityHubClient:
    return AwsSecurityHubClient(config)


def create_azure_sentinel_client(
    config: AzureSentinelConfig,
) -> AzureSentinelClient:
    return AzureSentinelClient(config)


def create_gcp_scc_client(config: GcpSccConfig) -> GcpSccClient:
    return GcpSccClient(config)
