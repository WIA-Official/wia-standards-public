"""
WIA Security Threat Intelligence Feed Integration
MISP, OTX, VirusTotal
"""

import base64
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any
from datetime import datetime

try:
    import httpx
    HAS_HTTPX = True
except ImportError:
    HAS_HTTPX = False

from ..types import WiaSecurityEvent, IndicatorType


# ============================================================================
# Types
# ============================================================================

@dataclass
class ThreatIntelFeedConfig:
    """Base configuration for threat intel feeds."""
    url: str
    api_key: str
    timeout: float = 30.0
    poll_interval: float = 300.0  # 5 minutes


@dataclass
class IoC:
    """Indicator of Compromise."""
    type: IndicatorType
    value: str
    confidence: Optional[float] = None
    first_seen: Optional[str] = None
    last_seen: Optional[str] = None
    tags: Optional[List[str]] = None
    source: Optional[str] = None


@dataclass
class ThreatFeed:
    """Threat intelligence feed."""
    id: str
    name: str
    provider: str
    last_updated: str
    indicators: List[IoC]


# ============================================================================
# MISP Integration
# ============================================================================

@dataclass
class MispConfig(ThreatIntelFeedConfig):
    """MISP configuration."""
    verify_tls: bool = True


class MispClient:
    """MISP threat intelligence client."""

    def __init__(self, config: MispConfig):
        if not HAS_HTTPX:
            raise ImportError("httpx package required. Install with: pip install httpx")

        self.config = config
        self._client = httpx.Client(
            timeout=config.timeout,
            headers={
                "Authorization": config.api_key,
                "Accept": "application/json",
                "Content-Type": "application/json",
            },
            verify=config.verify_tls,
        )

    def close(self) -> None:
        self._client.close()

    def __enter__(self) -> "MispClient":
        return self

    def __exit__(self, *args) -> None:
        self.close()

    def search_events(
        self,
        type: Optional[str] = None,
        value: Optional[str] = None,
        category: Optional[str] = None,
        org: Optional[str] = None,
        tags: Optional[List[str]] = None,
        from_date: Optional[str] = None,
        to_date: Optional[str] = None,
        limit: int = 100,
    ) -> ThreatFeed:
        """Search for events."""
        body: Dict[str, Any] = {
            "returnFormat": "json",
            "limit": limit,
        }

        if type:
            body["type"] = type
        if value:
            body["value"] = value
        if category:
            body["category"] = category
        if org:
            body["org"] = org
        if tags:
            body["tags"] = tags
        if from_date:
            body["from"] = from_date
        if to_date:
            body["to"] = to_date

        response = self._client.post(
            f"{self.config.url}/events/restSearch",
            json=body,
        )
        response.raise_for_status()

        return self._parse_misp_response(response.json())

    def get_attributes(self, type: str, limit: int = 100) -> List[IoC]:
        """Get attributes by type."""
        response = self._client.post(
            f"{self.config.url}/attributes/restSearch",
            json={
                "returnFormat": "json",
                "type": type,
                "limit": limit,
            },
        )
        response.raise_for_status()

        return self._parse_misp_attributes(response.json())

    def add_event(self, event: WiaSecurityEvent) -> str:
        """Add event to MISP."""
        misp_event = self._to_misp_event(event)

        response = self._client.post(
            f"{self.config.url}/events/add",
            json=misp_event,
        )
        response.raise_for_status()

        return response.json()["Event"]["uuid"]

    def _parse_misp_response(self, data: Dict[str, Any]) -> ThreatFeed:
        indicators: List[IoC] = []

        for event in data.get("response", []):
            e = event.get("Event", {})
            for attr in e.get("Attribute", []):
                indicators.append(IoC(
                    type=self._map_misp_type(attr.get("type", "")),
                    value=attr.get("value", ""),
                    confidence=80 if attr.get("to_ids") else 50,
                    first_seen=attr.get("first_seen"),
                    last_seen=attr.get("last_seen"),
                    tags=[t.get("name", "") for t in e.get("Tag", [])],
                    source="MISP",
                ))

        return ThreatFeed(
            id=f"misp-{int(datetime.now().timestamp())}",
            name="MISP Feed",
            provider="MISP",
            last_updated=datetime.now().isoformat(),
            indicators=indicators,
        )

    def _parse_misp_attributes(self, data: Dict[str, Any]) -> List[IoC]:
        return [
            IoC(
                type=self._map_misp_type(attr.get("type", "")),
                value=attr.get("value", ""),
                confidence=80 if attr.get("to_ids") else 50,
                first_seen=attr.get("first_seen"),
                last_seen=attr.get("last_seen"),
                source="MISP",
            )
            for attr in data.get("response", {}).get("Attribute", [])
        ]

    def _map_misp_type(self, misp_type: str) -> IndicatorType:
        mapping = {
            "ip-src": IndicatorType.IPV4,
            "ip-dst": IndicatorType.IPV4,
            "domain": IndicatorType.DOMAIN,
            "hostname": IndicatorType.DOMAIN,
            "url": IndicatorType.URL,
            "md5": IndicatorType.MD5,
            "sha1": IndicatorType.SHA1,
            "sha256": IndicatorType.SHA256,
            "email-src": IndicatorType.EMAIL,
            "email-dst": IndicatorType.EMAIL,
        }
        return mapping.get(misp_type, IndicatorType.OTHER)

    def _to_misp_event(self, event: WiaSecurityEvent) -> Dict[str, Any]:
        threat_level = 4  # Undefined
        if event.severity >= 9:
            threat_level = 1  # High
        elif event.severity >= 6:
            threat_level = 2  # Medium
        elif event.severity >= 3:
            threat_level = 3  # Low

        return {
            "Event": {
                "info": event.description,
                "threat_level_id": threat_level,
                "analysis": 2,
                "distribution": 0,
            }
        }


# ============================================================================
# AlienVault OTX Integration
# ============================================================================

@dataclass
class OtxConfig(ThreatIntelFeedConfig):
    """OTX configuration."""
    pass


class OtxClient:
    """AlienVault OTX client."""

    def __init__(self, config: OtxConfig):
        if not HAS_HTTPX:
            raise ImportError("httpx package required. Install with: pip install httpx")

        self.config = config
        if not config.url:
            config.url = "https://otx.alienvault.com"

        self._client = httpx.Client(
            timeout=config.timeout,
            headers={
                "X-OTX-API-KEY": config.api_key,
                "Accept": "application/json",
            },
        )

    def close(self) -> None:
        self._client.close()

    def __enter__(self) -> "OtxClient":
        return self

    def __exit__(self, *args) -> None:
        self.close()

    def get_subscribed_pulses(self, limit: int = 10) -> List[ThreatFeed]:
        """Get subscribed pulses."""
        response = self._client.get(
            f"{self.config.url}/api/v1/pulses/subscribed",
            params={"limit": limit},
        )
        response.raise_for_status()

        return [
            self._parse_pulse(pulse)
            for pulse in response.json().get("results", [])
        ]

    def get_pulse(self, pulse_id: str) -> ThreatFeed:
        """Get pulse by ID."""
        response = self._client.get(
            f"{self.config.url}/api/v1/pulses/{pulse_id}"
        )
        response.raise_for_status()

        return self._parse_pulse(response.json())

    def search_indicators(
        self,
        type: str,
        value: str,
    ) -> List[IoC]:
        """Search indicators."""
        response = self._client.get(
            f"{self.config.url}/api/v1/indicators/{type}/{value}/general"
        )
        response.raise_for_status()

        return self._parse_indicator_response(response.json(), type, value)

    def _parse_pulse(self, pulse: Dict[str, Any]) -> ThreatFeed:
        indicators = [
            IoC(
                type=self._map_otx_type(ind.get("type", "")),
                value=ind.get("indicator", ""),
                confidence=70,
                first_seen=ind.get("created"),
                tags=pulse.get("tags", []),
                source="OTX",
            )
            for ind in pulse.get("indicators", [])
        ]

        return ThreatFeed(
            id=pulse.get("id", ""),
            name=pulse.get("name", ""),
            provider="AlienVault OTX",
            last_updated=pulse.get("modified", ""),
            indicators=indicators,
        )

    def _parse_indicator_response(
        self,
        data: Dict[str, Any],
        type: str,
        value: str,
    ) -> List[IoC]:
        pulse_count = data.get("pulse_info", {}).get("count", 0)
        if pulse_count > 0:
            pulses = data.get("pulse_info", {}).get("pulses", [])
            tags = []
            for p in pulses:
                tags.extend(p.get("tags", []))

            return [IoC(
                type=self._map_otx_type(type),
                value=value,
                confidence=min(pulse_count * 10, 100),
                tags=list(set(tags)),
                source="OTX",
            )]
        return []

    def _map_otx_type(self, otx_type: str) -> IndicatorType:
        mapping = {
            "IPv4": IndicatorType.IPV4,
            "IPv6": IndicatorType.IPV6,
            "domain": IndicatorType.DOMAIN,
            "hostname": IndicatorType.DOMAIN,
            "url": IndicatorType.URL,
            "URL": IndicatorType.URL,
            "FileHash-MD5": IndicatorType.MD5,
            "FileHash-SHA1": IndicatorType.SHA1,
            "FileHash-SHA256": IndicatorType.SHA256,
            "email": IndicatorType.EMAIL,
        }
        return mapping.get(otx_type, IndicatorType.OTHER)


# ============================================================================
# VirusTotal Integration
# ============================================================================

@dataclass
class VirusTotalConfig(ThreatIntelFeedConfig):
    """VirusTotal configuration."""
    pass


class VirusTotalClient:
    """VirusTotal client."""

    def __init__(self, config: VirusTotalConfig):
        if not HAS_HTTPX:
            raise ImportError("httpx package required. Install with: pip install httpx")

        self.config = config
        if not config.url:
            config.url = "https://www.virustotal.com/api/v3"

        self._client = httpx.Client(
            timeout=config.timeout,
            headers={
                "x-apikey": config.api_key,
                "Accept": "application/json",
            },
        )

    def close(self) -> None:
        self._client.close()

    def __enter__(self) -> "VirusTotalClient":
        return self

    def __exit__(self, *args) -> None:
        self.close()

    def get_file_report(self, hash: str) -> Optional[IoC]:
        """Get file report by hash."""
        response = self._client.get(f"{self.config.url}/files/{hash}")

        if response.status_code == 404:
            return None

        response.raise_for_status()
        return self._parse_file_response(response.json())

    def get_url_report(self, url: str) -> Optional[IoC]:
        """Get URL report."""
        url_id = base64.urlsafe_b64encode(url.encode()).decode().rstrip("=")

        response = self._client.get(f"{self.config.url}/urls/{url_id}")

        if response.status_code == 404:
            return None

        response.raise_for_status()
        return self._parse_url_response(response.json(), url)

    def get_domain_report(self, domain: str) -> Optional[IoC]:
        """Get domain report."""
        response = self._client.get(f"{self.config.url}/domains/{domain}")

        if response.status_code == 404:
            return None

        response.raise_for_status()
        return self._parse_domain_response(response.json(), domain)

    def get_ip_report(self, ip: str) -> Optional[IoC]:
        """Get IP report."""
        response = self._client.get(f"{self.config.url}/ip_addresses/{ip}")

        if response.status_code == 404:
            return None

        response.raise_for_status()
        return self._parse_ip_response(response.json(), ip)

    def _parse_file_response(self, data: Dict[str, Any]) -> IoC:
        attrs = data.get("data", {}).get("attributes", {})
        stats = attrs.get("last_analysis_stats", {})
        malicious = stats.get("malicious", 0)
        total = sum(stats.values()) if stats else 0

        first_seen = attrs.get("first_submission_date")
        last_seen = attrs.get("last_analysis_date")

        return IoC(
            type=self._detect_hash_type(data.get("data", {}).get("id", "")),
            value=data.get("data", {}).get("id", ""),
            confidence=(malicious / total * 100) if total > 0 else 0,
            first_seen=datetime.fromtimestamp(first_seen).isoformat() if first_seen else None,
            last_seen=datetime.fromtimestamp(last_seen).isoformat() if last_seen else None,
            tags=attrs.get("tags", []),
            source="VirusTotal",
        )

    def _parse_url_response(self, data: Dict[str, Any], url: str) -> IoC:
        attrs = data.get("data", {}).get("attributes", {})
        stats = attrs.get("last_analysis_stats", {})
        malicious = stats.get("malicious", 0)
        total = sum(stats.values()) if stats else 0

        last_seen = attrs.get("last_analysis_date")

        return IoC(
            type=IndicatorType.URL,
            value=url,
            confidence=(malicious / total * 100) if total > 0 else 0,
            last_seen=datetime.fromtimestamp(last_seen).isoformat() if last_seen else None,
            tags=attrs.get("tags", []),
            source="VirusTotal",
        )

    def _parse_domain_response(self, data: Dict[str, Any], domain: str) -> IoC:
        attrs = data.get("data", {}).get("attributes", {})
        stats = attrs.get("last_analysis_stats", {})
        malicious = stats.get("malicious", 0)
        total = sum(stats.values()) if stats else 0

        last_seen = attrs.get("last_analysis_date")

        return IoC(
            type=IndicatorType.DOMAIN,
            value=domain,
            confidence=(malicious / total * 100) if total > 0 else 0,
            last_seen=datetime.fromtimestamp(last_seen).isoformat() if last_seen else None,
            tags=attrs.get("tags", []),
            source="VirusTotal",
        )

    def _parse_ip_response(self, data: Dict[str, Any], ip: str) -> IoC:
        attrs = data.get("data", {}).get("attributes", {})
        stats = attrs.get("last_analysis_stats", {})
        malicious = stats.get("malicious", 0)
        total = sum(stats.values()) if stats else 0

        last_seen = attrs.get("last_analysis_date")

        return IoC(
            type=IndicatorType.IPV6 if ":" in ip else IndicatorType.IPV4,
            value=ip,
            confidence=(malicious / total * 100) if total > 0 else 0,
            last_seen=datetime.fromtimestamp(last_seen).isoformat() if last_seen else None,
            source="VirusTotal",
        )

    def _detect_hash_type(self, hash: str) -> IndicatorType:
        length = len(hash)
        if length == 32:
            return IndicatorType.MD5
        elif length == 40:
            return IndicatorType.SHA1
        elif length == 64:
            return IndicatorType.SHA256
        return IndicatorType.OTHER


# ============================================================================
# Factory Functions
# ============================================================================

def create_misp_client(config: MispConfig) -> MispClient:
    return MispClient(config)


def create_otx_client(config: OtxConfig) -> OtxClient:
    return OtxClient(config)


def create_virustotal_client(config: VirusTotalConfig) -> VirusTotalClient:
    return VirusTotalClient(config)
