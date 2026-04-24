"""
WIA Security Dashboard Integration
Metrics, aggregation, and visualization support
"""

from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any, Set
from datetime import datetime, timedelta
from collections import defaultdict

from ..types import WiaSecurityEvent, EventType


# ============================================================================
# Types
# ============================================================================

@dataclass
class SeverityDistribution:
    """Severity distribution counts."""
    critical: int = 0
    high: int = 0
    medium: int = 0
    low: int = 0
    informational: int = 0


@dataclass
class TimeSeriesPoint:
    """Time series data point."""
    timestamp: str
    count: int
    severity: Optional[float] = None


@dataclass
class TacticCount:
    """MITRE ATT&CK tactic count."""
    tactic: str
    tactic_id: str
    count: int


@dataclass
class TechniqueCount:
    """MITRE ATT&CK technique count."""
    technique: str
    technique_id: str
    tactic: str
    count: int


@dataclass
class DashboardMetrics:
    """Dashboard metrics."""
    total_events: int
    events_by_type: Dict[str, int]
    events_by_severity: SeverityDistribution
    events_by_source: Dict[str, int]
    time_series_data: List[TimeSeriesPoint]
    top_mitre_tactics: List[TacticCount]
    top_mitre_techniques: List[TechniqueCount]
    mean_time_to_detect: Optional[float] = None
    mean_time_to_respond: Optional[float] = None


@dataclass
class AlertStats:
    """Alert statistics."""
    total: int = 0
    open: int = 0
    acknowledged: int = 0
    resolved: int = 0
    false_positives: int = 0


@dataclass
class ThreatLandscape:
    """Threat landscape summary."""
    active_threats: int = 0
    new_indicators: int = 0
    compromised_assets: int = 0
    top_threat_actors: List[str] = field(default_factory=list)
    top_malware_families: List[str] = field(default_factory=list)


@dataclass
class ControlStatus:
    """Compliance control status."""
    control_id: str
    name: str
    status: str  # compliant, non_compliant, partial, not_assessed
    score: float
    findings: int


@dataclass
class ComplianceStatus:
    """Compliance framework status."""
    framework: str
    controls: List[ControlStatus]
    overall_score: float
    last_assessment: str


# ============================================================================
# Dashboard Aggregator
# ============================================================================

class DashboardAggregator:
    """Aggregates security events for dashboard visualization."""

    def __init__(self, time_window_hours: int = 24):
        self._events: List[WiaSecurityEvent] = []
        self._time_window = timedelta(hours=time_window_hours)

    def add_event(self, event: WiaSecurityEvent) -> None:
        """Add event to aggregator."""
        self._events.append(event)
        self._prune_old_events()

    def add_events(self, events: List[WiaSecurityEvent]) -> None:
        """Add multiple events."""
        self._events.extend(events)
        self._prune_old_events()

    def get_metrics(self) -> DashboardMetrics:
        """Get dashboard metrics."""
        now = datetime.now()
        cutoff = now - self._time_window
        relevant_events = [
            e for e in self._events
            if datetime.fromisoformat(e.timestamp.replace("Z", "+00:00").replace("+00:00", "")) > cutoff
        ]

        return DashboardMetrics(
            total_events=len(relevant_events),
            events_by_type=self._count_by_type(relevant_events),
            events_by_severity=self._count_by_severity(relevant_events),
            events_by_source=self._count_by_source(relevant_events),
            time_series_data=self._build_time_series(relevant_events),
            top_mitre_tactics=self._get_top_tactics(relevant_events),
            top_mitre_techniques=self._get_top_techniques(relevant_events),
            mean_time_to_detect=self._calculate_mttd(relevant_events),
            mean_time_to_respond=self._calculate_mttr(relevant_events),
        )

    def get_alert_stats(self) -> AlertStats:
        """Get alert statistics."""
        alerts = [e for e in self._events if e.type == EventType.ALERT]

        stats = AlertStats(total=len(alerts))

        for alert in alerts:
            status = getattr(alert, "status", None)
            if status == "open":
                stats.open += 1
            elif status == "acknowledged":
                stats.acknowledged += 1
            elif status == "resolved":
                stats.resolved += 1
            elif status == "false_positive":
                stats.false_positives += 1

        return stats

    def get_threat_landscape(self) -> ThreatLandscape:
        """Get threat landscape."""
        threat_intel = [e for e in self._events if e.type == EventType.THREAT_INTEL]
        incidents = [e for e in self._events if e.type == EventType.INCIDENT]

        threat_actors: Set[str] = set()
        malware_families: Set[str] = set()

        for e in threat_intel:
            if hasattr(e, "threat_actor") and e.threat_actor:
                threat_actors.add(e.threat_actor)
            if hasattr(e, "malware_family") and e.malware_family:
                malware_families.add(e.malware_family)

        now = datetime.now()
        one_day_ago = now - timedelta(days=1)

        active_threats = sum(
            1 for t in threat_intel
            if not hasattr(t, "valid_until") or
            datetime.fromisoformat(t.valid_until.replace("Z", "")) > now
        )

        new_indicators = sum(
            1 for t in threat_intel
            if datetime.fromisoformat(t.timestamp.replace("Z", "")) > one_day_ago
        )

        affected_assets: Set[str] = set()
        for i in incidents:
            if hasattr(i, "affected_assets"):
                affected_assets.update(i.affected_assets or [])

        return ThreatLandscape(
            active_threats=active_threats,
            new_indicators=new_indicators,
            compromised_assets=len(affected_assets),
            top_threat_actors=list(threat_actors)[:5],
            top_malware_families=list(malware_families)[:5],
        )

    def get_severity_trend(self, intervals: int = 24) -> List[TimeSeriesPoint]:
        """Get severity trend."""
        now = datetime.now()
        interval_td = self._time_window / intervals
        points: List[TimeSeriesPoint] = []

        for i in range(intervals):
            start = now - (intervals - i) * interval_td
            end = start + interval_td

            interval_events = [
                e for e in self._events
                if start <= datetime.fromisoformat(
                    e.timestamp.replace("Z", "").replace("+00:00", "")
                ) < end
            ]

            avg_severity = (
                sum(e.severity for e in interval_events) / len(interval_events)
                if interval_events else 0
            )

            points.append(TimeSeriesPoint(
                timestamp=start.isoformat(),
                count=len(interval_events),
                severity=avg_severity,
            ))

        return points

    def export_for_visualization(self) -> Dict[str, Any]:
        """Export data for visualization."""
        return {
            "events": [e.to_dict() for e in self._events],
            "metrics": self.get_metrics().__dict__,
            "alert_stats": self.get_alert_stats().__dict__,
            "threat_landscape": self.get_threat_landscape().__dict__,
            "severity_trend": [p.__dict__ for p in self.get_severity_trend()],
        }

    def clear(self) -> None:
        """Clear all events."""
        self._events.clear()

    # -------------------------------------------------------------------------
    # Private Methods
    # -------------------------------------------------------------------------

    def _prune_old_events(self) -> None:
        cutoff = datetime.now() - self._time_window
        self._events = [
            e for e in self._events
            if datetime.fromisoformat(
                e.timestamp.replace("Z", "").replace("+00:00", "")
            ) > cutoff
        ]

    def _count_by_type(self, events: List[WiaSecurityEvent]) -> Dict[str, int]:
        counts: Dict[str, int] = defaultdict(int)
        for e in events:
            counts[e.type.value] += 1
        return dict(counts)

    def _count_by_severity(
        self,
        events: List[WiaSecurityEvent],
    ) -> SeverityDistribution:
        dist = SeverityDistribution()
        for e in events:
            if e.severity >= 9:
                dist.critical += 1
            elif e.severity >= 7:
                dist.high += 1
            elif e.severity >= 4:
                dist.medium += 1
            elif e.severity >= 1:
                dist.low += 1
            else:
                dist.informational += 1
        return dist

    def _count_by_source(self, events: List[WiaSecurityEvent]) -> Dict[str, int]:
        counts: Dict[str, int] = defaultdict(int)
        for e in events:
            source = e.source.name if e.source else "unknown"
            counts[source] += 1
        return dict(counts)

    def _build_time_series(
        self,
        events: List[WiaSecurityEvent],
        intervals: int = 24,
    ) -> List[TimeSeriesPoint]:
        now = datetime.now()
        interval_td = self._time_window / intervals
        points: List[TimeSeriesPoint] = []

        for i in range(intervals):
            start = now - (intervals - i) * interval_td
            end = start + interval_td

            count = sum(
                1 for e in events
                if start <= datetime.fromisoformat(
                    e.timestamp.replace("Z", "").replace("+00:00", "")
                ) < end
            )

            points.append(TimeSeriesPoint(
                timestamp=start.isoformat(),
                count=count,
            ))

        return points

    def _get_top_tactics(
        self,
        events: List[WiaSecurityEvent],
        limit: int = 10,
    ) -> List[TacticCount]:
        counts: Dict[str, int] = defaultdict(int)
        for e in events:
            if e.mitre and e.mitre.tactic:
                counts[e.mitre.tactic] += 1

        return [
            TacticCount(
                tactic=self._format_tactic_name(tactic_id),
                tactic_id=tactic_id,
                count=count,
            )
            for tactic_id, count in sorted(
                counts.items(), key=lambda x: x[1], reverse=True
            )[:limit]
        ]

    def _get_top_techniques(
        self,
        events: List[WiaSecurityEvent],
        limit: int = 10,
    ) -> List[TechniqueCount]:
        counts: Dict[str, Dict[str, Any]] = {}
        for e in events:
            if e.mitre and e.mitre.technique:
                tech_id = e.mitre.technique
                if tech_id not in counts:
                    counts[tech_id] = {
                        "tactic": e.mitre.tactic or "unknown",
                        "count": 0,
                    }
                counts[tech_id]["count"] += 1

        return [
            TechniqueCount(
                technique=tech_id,
                technique_id=tech_id,
                tactic=data["tactic"],
                count=data["count"],
            )
            for tech_id, data in sorted(
                counts.items(), key=lambda x: x[1]["count"], reverse=True
            )[:limit]
        ]

    def _calculate_mttd(
        self,
        events: List[WiaSecurityEvent],
    ) -> Optional[float]:
        detection_times: List[float] = []

        for e in events:
            if e.meta:
                detected_at = getattr(e.meta, "detected_at", None)
                occurred_at = getattr(e.meta, "occurred_at", None)
                if detected_at and occurred_at:
                    detected = datetime.fromisoformat(detected_at.replace("Z", ""))
                    occurred = datetime.fromisoformat(occurred_at.replace("Z", ""))
                    if detected > occurred:
                        detection_times.append(
                            (detected - occurred).total_seconds() / 60
                        )

        if not detection_times:
            return None
        return sum(detection_times) / len(detection_times)

    def _calculate_mttr(
        self,
        events: List[WiaSecurityEvent],
    ) -> Optional[float]:
        response_times: List[float] = []

        for e in events:
            if e.meta:
                detected_at = getattr(e.meta, "detected_at", None)
                resolved_at = getattr(e.meta, "resolved_at", None)
                if detected_at and resolved_at:
                    detected = datetime.fromisoformat(detected_at.replace("Z", ""))
                    resolved = datetime.fromisoformat(resolved_at.replace("Z", ""))
                    if resolved > detected:
                        response_times.append(
                            (resolved - detected).total_seconds() / 60
                        )

        if not response_times:
            return None
        return sum(response_times) / len(response_times)

    def _format_tactic_name(self, tactic_id: str) -> str:
        tactics = {
            "TA0001": "Initial Access",
            "TA0002": "Execution",
            "TA0003": "Persistence",
            "TA0004": "Privilege Escalation",
            "TA0005": "Defense Evasion",
            "TA0006": "Credential Access",
            "TA0007": "Discovery",
            "TA0008": "Lateral Movement",
            "TA0009": "Collection",
            "TA0010": "Exfiltration",
            "TA0011": "Command and Control",
            "TA0040": "Impact",
            "TA0042": "Resource Development",
            "TA0043": "Reconnaissance",
        }
        return tactics.get(tactic_id, tactic_id)


# ============================================================================
# Compliance Tracker
# ============================================================================

class ComplianceTracker:
    """Tracks compliance status across frameworks."""

    def __init__(self):
        self._frameworks: Dict[str, ComplianceStatus] = {}

    def update_framework(self, status: ComplianceStatus) -> None:
        """Add or update framework status."""
        self._frameworks[status.framework] = status

    def get_framework(self, framework: str) -> Optional[ComplianceStatus]:
        """Get framework status."""
        return self._frameworks.get(framework)

    def get_all_frameworks(self) -> List[ComplianceStatus]:
        """Get all frameworks."""
        return list(self._frameworks.values())

    def map_event_to_controls(
        self,
        event: WiaSecurityEvent,
        framework: str,
    ) -> List[str]:
        """Map WIA Security event to compliance controls."""
        mappings = {
            "NIST": {
                "auth_event": ["AC-2", "AC-7", "IA-5"],
                "network_event": ["SC-7", "SI-4", "AU-12"],
                "endpoint_event": ["CM-7", "SI-3", "SI-4"],
                "vulnerability": ["RA-5", "SI-2", "CM-8"],
                "incident": ["IR-4", "IR-5", "IR-6"],
                "threat_intel": ["RA-3", "SI-5", "PM-16"],
                "alert": ["SI-4", "AU-6", "IR-4"],
            },
            "CIS": {
                "auth_event": ["CIS-4.1", "CIS-4.2", "CIS-16.1"],
                "network_event": ["CIS-12.1", "CIS-13.1", "CIS-9.1"],
                "endpoint_event": ["CIS-2.1", "CIS-8.1", "CIS-10.1"],
                "vulnerability": ["CIS-3.1", "CIS-7.1", "CIS-3.4"],
                "incident": ["CIS-19.1", "CIS-19.2", "CIS-19.3"],
                "threat_intel": ["CIS-17.1", "CIS-17.2"],
                "alert": ["CIS-6.1", "CIS-6.2", "CIS-19.1"],
            },
            "ISO27001": {
                "auth_event": ["A.9.2", "A.9.4", "A.12.4"],
                "network_event": ["A.13.1", "A.12.4", "A.12.6"],
                "endpoint_event": ["A.12.2", "A.12.5", "A.8.3"],
                "vulnerability": ["A.12.6", "A.14.2", "A.18.2"],
                "incident": ["A.16.1", "A.16.2", "A.16.3"],
                "threat_intel": ["A.6.1", "A.18.1"],
                "alert": ["A.12.4", "A.16.1"],
            },
        }

        return mappings.get(framework, {}).get(event.type.value, [])

    def calculate_score_from_events(
        self,
        events: List[WiaSecurityEvent],
        framework: str,
    ) -> float:
        """Calculate compliance score from events."""
        controls_with_findings: Set[str] = set()
        all_controls: Set[str] = set()

        for e in events:
            controls = self.map_event_to_controls(e, framework)
            all_controls.update(controls)
            if e.severity >= 7:
                controls_with_findings.update(controls)

        if not all_controls:
            return 100.0

        affected_ratio = len(controls_with_findings) / len(all_controls)
        return round((1 - affected_ratio) * 100)


# ============================================================================
# Factory Functions
# ============================================================================

def create_dashboard_aggregator(
    time_window_hours: int = 24,
) -> DashboardAggregator:
    return DashboardAggregator(time_window_hours)


def create_compliance_tracker() -> ComplianceTracker:
    return ComplianceTracker()
