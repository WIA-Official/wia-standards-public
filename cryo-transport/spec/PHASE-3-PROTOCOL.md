# WIA Cryo-Transport Protocol Standard
## Phase 3 Specification

**Version**: 1.0.0  
**Status**: Complete  
**License**: MIT

---

## Pre-Transport Checklist

### Container Preparation
- [ ] Vacuum integrity verified (&lt;10^-4 torr)
- [ ] All sensors calibrated
- [ ] GPS trackers tested
- [ ] LN2 filled to 95%
- [ ] Temperature stable at -196°C
- [ ] Documentation complete

### Personnel Verification
- [ ] Driver certification current
- [ ] Hazmat training up-to-date
- [ ] Emergency contacts confirmed
- [ ] Route briefing completed

---

## In-Transit Monitoring

### Normal Operations
- Temperature check: Every 60 seconds
- GPS update: Every 5 minutes
- Driver check-in: Every 2 hours
- LN2 level: Every 4 hours

### Alert Conditions
- Temperature &gt; -190°C: Increase monitoring to 30-second intervals
- Temperature &gt; -180°C: Emergency protocol activation
- GPS signal lost &gt; 15 minutes: Switch to satellite backup
- Unplanned stop &gt; 1 hour: Contact coordinator

---

## Facility Handoff Protocol

### Receiving Procedure
1. Identity verification (photo ID + credentials)
2. Container visual inspection
3. Temperature verification
4. LN2 level check
5. Documentation review
6. Digital signatures
7. Subject transfer to storage
8. Handoff completion notification

---

## Detailed Operational Protocols

### Pre-Transport Quality Gate

Before a transport can be released for departure, the dispatcher SHALL verify each of the gates below. A failed gate blocks departure; documented overrides require dispatcher and quality-officer signatures and become part of the chain-of-custody record.

| Gate | Acceptance criterion | Reference |
|------|----------------------|-----------|
| Vacuum integrity | residual pressure < 10⁻⁴ torr maintained for 30 min | ISO 21973:2020 §6.2 |
| Sensor calibration | all temperature sensors within ±0.5 °C at LN2 reference | ISO 17025:2017 traceability |
| LN2 fill | level ≥ 95% at departure | manufacturer documentation |
| GPS fix | 3D fix with HDOP ≤ 2.0 | ICAO Annex 10 Vol I |
| Documentation | manifest, hazmat declarations, custody chain initialised | DOT 49 CFR §172.200, IATA DGR §8 |
| Driver certification | hazmat endorsement current; CDL valid | DOT 49 CFR §177.816 |
| Hazmat training record | within validity window | DOT 49 CFR §172.704 |
| Vehicle inspection | DVIR completed within prior 24 h | DOT 49 CFR §396.11 |
| Emergency contacts | dispatcher, regulatory POC, receiving facility | IATA DGR §9.1.1 |

### In-Transit Monitoring Cadence

| Condition | Telemetry interval | Custody-event interval |
|-----------|--------------------|------------------------|
| Nominal | 60 s | every 2 h |
| Temperature warning (>-190°C) | 30 s | every 30 min |
| Temperature alarm (>-180°C) | 10 s | every 10 min, plus emergency event |
| GPS signal degraded (HDOP > 5) | unchanged | dispatcher notified within 15 min |
| GPS signal lost > 15 min | switch to satellite backup | dispatcher notified within 5 min |
| Unplanned stop > 60 min | unchanged | mandatory custody event |

The telemetry interval changes are automatic when the sensor edge controller observes a threshold crossing; the dispatcher does not need to push a configuration change.

### Excursion Response

When a temperature excursion is detected, the response procedure has five stages:

1. **Immediate stabilisation** — Driver verifies LN2 level visually, seals secondary insulation if applicable, and avoids opening the container.
2. **Diagnostic** — Driver photographs the container exterior (per the WIA-CT photo protocol) and uploads via the operations app.
3. **Decision** — Dispatcher, in consultation with the receiving facility, decides among continue, divert to nearest backup facility, or return to origin.
4. **Documentation** — A chain-of-custody event of type `EMERGENCY_STOP` is created with witness signatures.
5. **Post-event** — A post-mortem is filed within 5 business days regardless of outcome; the post-mortem becomes part of the manifest's permanent record.

### Backup Facility Protocol

Routes are planned with backup facility spacing not exceeding 200 km along the planned path (configurable per subject class). When a divert is decided, the receiving backup facility receives a real-time notification with the manifest, sensor history, ETA, and the reason for divert. Acceptance by the backup facility is confirmed through a dual-signature handshake before the diversion is executed.

### Cross-Border Procedures

International transports follow:

- **IATA Dangerous Goods Regulations** for air carriage.
- **ADR 2025** for road carriage in Europe.
- **IMDG Code** for sea carriage.
- **CITES** when biological subject material is regulated.
- **National competent authority approvals** (FDA 21 CFR Part 1271 for US, EMA Tissues and Cells Directive 2004/23/EC for EU).

The custody chain spans border crossings. Customs and competent-authority approvals are appended as custody events; the chain MUST be unbroken from origin to destination across all jurisdictions.

### Receiving Procedure (Detailed)

The eight-step receiving procedure expands as follows:

1. **Identity verification** — Receiving officer verifies photo ID and credentials against the manifest's expected handoff actor.
2. **Container visual inspection** — Inspect for visible damage, condensation, structural deformation, valve integrity, and seal integrity.
3. **Temperature verification** — Read primary and secondary sensors; values within tolerance and consistent with the in-transit telemetry record.
4. **LN2 level check** — Read primary and secondary level indicators; cross-check against the in-transit telemetry trend.
5. **Documentation review** — Manifest, custody chain, hazmat declarations, transit anomalies (if any).
6. **Digital signatures** — Driver and receiving officer countersign the `DELIVERY` custody event under §6 of Phase 2 API.
7. **Subject transfer to storage** — Per the receiving facility's storage SOP. Storage placement is recorded with the storage location identifier.
8. **Handoff completion notification** — Notification dispatched to dispatcher, origin facility, and (where required) the regulatory authority.

### Unrecoverable Excursion

If the temperature excursion exceeds the recoverability threshold defined in the manifest's subject class, the procedure has three stages:

1. **Cease normal handling** — The container MUST not be opened; further handling SHALL be by quality and clinical officers only.
2. **Notify stakeholders** — Origin facility, destination facility, dispatcher, and regulatory authority are notified within the manifest-defined SLA (typically 1 h).
3. **Forensic preservation** — All telemetry, custody events, photographs, and physical materials are preserved per the manifest's retention policy.

---

## Reference Standards Alignment

| Concern | Reference |
|---------|-----------|
| Cell and tissue transport | ISO 21973:2020 |
| Quality management for medical devices | ISO 13485:2016 |
| Calibration traceability | ISO/IEC 17025:2017 |
| Hazmat ground transport (US) | DOT 49 CFR Parts 100–185 |
| Hazmat ground transport (EU) | ADR 2025 (UNECE) |
| Hazmat air transport | IATA Dangerous Goods Regulations 2025 |
| Hazmat sea transport | IMDG Code, IMO Resolution MSC.477(102) |
| Cell/tissue regulation (US) | FDA 21 CFR Part 1271 |
| Cell/tissue regulation (EU) | Directive 2004/23/EC |
| Driver records | DOT 49 CFR §396.11 |
| Hazmat training | DOT 49 CFR §172.704 |
| Time encoding | ISO 8601:2019 |
| GPS / GNSS receivers | ICAO Annex 10 Volume I; ICD-GPS-200 |

---

## Operational Appendix

### A. Driver Daily Procedures

The driver's day is structured around five mandatory tasks. Each task records evidence in the operations app and produces an event in the custody trail when applicable.

1. **Pre-trip inspection (per DOT 49 CFR §392.7 / §396.13)** — Mechanical, electrical, and cryogenic systems are checked. Non-conformances block departure until resolved.
2. **Hours-of-service logging (per DOT 49 CFR Part 395)** — Electronic logging device records driving time, on-duty time, off-duty time, and sleeper-berth time. Violations are surfaced to the dispatcher in real time.
3. **In-transit checks** — Driver responds to scheduled prompts at intervals defined in §In-Transit Monitoring Cadence.
4. **End-of-day summary** — A brief summary is filed describing route, anomalies, and rest plan for the next day.
5. **Post-trip vehicle inspection** — Recorded in the DVIR per DOT 49 CFR §396.11.

### B. Dispatcher Procedures

Dispatchers operate from a console that surfaces the active manifest, real-time telemetry, alert queue, and the regulatory contact list for the operating jurisdiction. Dispatcher procedures include:

- Approving or denying excursion-response decisions.
- Authorising backup-facility diversion.
- Notifying receiving facility and origin facility of schedule changes.
- Filing regulatory reports under the operating jurisdiction's incident-reporting rule.
- Coordinating with quality officer for any non-routine event.

### C. Receiving Facility Procedures

Receiving facility procedures expand on the eight-step Receiving Procedure with:

- Verification of the chain-of-custody hash chain (each event hash links to the prior event hash).
- Acceptance test of the container's recorded thermal envelope against the subject class's tolerance.
- Storage placement record entered into the receiving facility's storage management system.
- Notification to origin facility of acceptance or rejection.

If the chain-of-custody verification fails, the receiving facility MUST escalate to the quality officer before proceeding.

### D. Hazmat Documentation

Hazmat documentation accompanying every transport conforms to:

- **DOT 49 CFR §172.200 Subpart C** — Shipping papers.
- **IATA DGR §8** — Documentation, including the Shipper's Declaration for Dangerous Goods.
- **ADR §5.4** — Documentation for road transport.
- **IMDG Code §5.4** — Documentation for sea transport.

Documents are produced electronically and signed under the operating organisation's hazmat-documentation key. Paper copies are retained per the operating jurisdiction's recordkeeping rule.

### E. Subject Class Profiles

The protocol parameters (temperature thresholds, monitoring intervals, recoverability windows) vary by subject class. The reference subject classes are:

| Subject class | Temperature target | Warning threshold | Alarm threshold | Recoverability window |
|---------------|--------------------|-------------------|------------------|------------------------|
| Whole-body cryo-preservation | -196°C | -190°C | -180°C | per manifest, typically 0 |
| Cellular therapy product | -150°C | -130°C | -100°C | per manifest |
| Bone-marrow cellular product | -150°C | -130°C | -100°C | per manifest |
| Reproductive cellular material | -196°C (LN2) | -180°C | -150°C | per manifest |

Recoverability windows are established by the originating facility's clinical protocol and are recorded in the manifest. The transport protocol does not relax these windows; any tolerance change requires manifest amendment with originating-facility signature.

### F. Quality Records Retention

Records associated with each transport are retained per the longest applicable rule among:

- Operating organisation's QMS (typically ISO 13485:2016 §4.2.5).
- Regulatory rule (FDA 21 CFR §1271.270 for US tissue products; analogous EU and KR rules).
- Subject-class-specific clinical protocol.
- Customer contract.

A retention summary is emitted as a custody event of type `RECORDS_RETENTION_PROFILE` when the manifest is created.

---

© 2025 WIA
