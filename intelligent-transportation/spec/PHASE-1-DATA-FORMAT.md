# WIA-intelligent-transportation PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-intelligent-transportation
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-intelligent-transportation. The standard covers the persistent
record shapes that flow among road-operator traffic management
centres (TMCs), connected and automated vehicles (CAVs),
V2X (vehicle-to-everything) infrastructure, transit operators,
journey-planning services, and the regulators and certification
bodies that supervise the intelligent-transportation programme.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time)
- ISO 14813-1:2015 (ITS — reference model architecture for the
  ITS sector — service domains, service groups and services)
- ISO 14817 (ITS — central data registries — content)
- ISO 17572-1/2/3 (location referencing for geographic databases)
- ISO 19091:2019 (ITS — cooperative ITS — using V2I and I2V
  communications for applications related to signalised
  intersections)
- ISO 19297-1 (ITS — MapDB — application programming interface)
- ISO 21217:2020 (CALM — communications access for land mobiles —
  architecture)
- ISO 24102 series (ITS station management)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17025:2017 (calibration laboratories)
- IEEE 1609.0 / 1609.2 / 1609.3 / 1609.4 (WAVE family — networking,
  security, application services)
- IEEE 802.11-2020 (DSRC PHY/MAC, including 802.11p heritage)
- SAE J2735_202309 (V2X Communications Message Set Dictionary)
- SAE J2945/1, J2945/2, J2945/3 (V2X minimum performance
  requirements — basic safety, V2I, weather)
- SAE J3161/1 (C-V2X)
- ETSI EN 302 665 (ITS — communications architecture)
- ETSI EN 302 663 (ITS-G5 access layer)
- ETSI EN 302 637-2 (CAM) and ETSI EN 302 637-3 (DENM)
- ETSI TS 103 097 (security headers and certificate formats)
- IETF RFC 4122 (UUID), RFC 8259 (JSON), RFC 9457 (Problem Details)
- TMDD v3.1 (Traffic Management Data Dictionary)
- NTCIP 1202 / 1203 (signal controllers and DMS)
- GTFS-Realtime v2.0 (transit feed)
- SIRI v2.1 (CEN — Service Interface for Real-time Information)

---

## §1 Scope

This PHASE defines persistent record shapes for the artefacts
exchanged across an intelligent-transportation programme.
Implementations covered include:

- TMC platforms managing arterial signal control, freeway ramp
  metering, dynamic message signs, and incident response.
- Roadside units (RSUs) emitting V2I messages (SPaT, MAP, IVI,
  RSA, TIM) over DSRC or C-V2X.
- On-board units (OBUs) in passenger vehicles, transit vehicles,
  emergency vehicles, and freight vehicles emitting V2V messages
  (BSM, DENM, CAM).
- Cooperative-ITS application servers operating Day-1 and Day-2
  use cases (intersection movement assist, lane-change assist,
  vulnerable road user warnings).
- Transit dispatch systems publishing GTFS-RT and SIRI feeds.
- Multimodal journey-planning aggregators consuming the above.

Personal mobility (e-scooter, bike-share) telematics and
autonomous-truck platooning command-and-control are addressed in
adjacent WIA standards and are out of scope here.

## §2 Programme Identifier

```
programmeId        : string (uuidv7)
programmeOperator  : string (institutional identifier of the
                       road operator, transit agency, or
                       integrated mobility authority)
programmeRegistered: string (ISO 8601 / RFC 3339)
serviceDomains     : array of enum (matched to ISO 14813-1
                       service groups, e.g. "TI" — traveller
                       information, "TM" — traffic management,
                       "EM" — emergency management,
                       "PT" — public transport, "AVCS" —
                       advanced vehicle control)
jurisdictionScope  : array of string (ISO 3166-1 / 3166-2 codes)
programmeStatus    : enum  ("draft" | "operating" |
                       "decommissioning" | "archived")
```

## §3 Roadway and Network Reference Record

Locations are encoded per ISO 17572 location referencing so that
records remain interpretable across map vendors and across map
revisions.

```
networkReference:
  refId            : string (uuidv7)
  programmeId      : string (uuidv7)
  referenceMethod  : enum ("agora-c" | "openlr-3.0" |
                       "tpeg-loc" | "lat-lon-only" |
                       "user-defined")
  encodedLocation  : string (encoded reference per the chosen
                       method)
  decodedSegment   : object (centre-line geometry; carriageway
                       direction; functional class; ISO
                       14817-2 link identifier when present)
  mapVersionRef    : string (URI of the underlying map version)
  validFrom        : string (ISO 8601)
  validUntil       : string (ISO 8601; absent for current)
```

Map vendors consuming this record produce a map-vendor-specific
internal identifier; the encoded reference is the authoritative
inter-vendor cite.

## §4 Roadside Unit (RSU) Record

```
rsu:
  rsuId            : string (uuidv7)
  programmeId      : string (uuidv7)
  installationRef  : string (work-order reference for the
                       installation event)
  vendor           : string (institutional identifier)
  modelNumber      : string
  firmwareRef      : string (content-addressed firmware URI)
  radioStack       : array of enum ("dsrc-802.11p" |
                       "cv2x-pc5-mode-4" | "cv2x-pc5-mode-3" |
                       "lte-uu" | "5g-nr-uu")
  txPowerDbm       : number (per applicable regulator's mask)
  antennaPattern   : object (azimuth gain pattern reference)
  geographicScope  : object (centre-line of the protected
                       intersection or segment, expressed in
                       the network reference of §3)
  itsCertChainRef  : string (URI of the IEEE 1609.2 / ETSI
                       TS 103 097 certificate chain that the
                       RSU signs messages with)
  rsuStatus        : enum ("commissioned" | "operational" |
                       "fault" | "decommissioned")
```

## §5 On-Board Unit (OBU) Record

OBUs are bound to vehicles through the operator's fleet-management
system. The DATA-FORMAT layer never carries vehicle-owner identity;
it references the OBU through an opaque identifier and an
operator-internal vehicle role token.

```
obu:
  obuId            : string (uuidv7)
  programmeId      : string (uuidv7)
  vehicleRoleToken : string (opaque token mapped in the operator
                       fleet system to vehicle role, e.g.
                       "transit-bus", "ambulance", "snowplow",
                       "passenger-car-anonymous")
  vendor           : string
  modelNumber      : string
  firmwareRef      : string (content-addressed firmware URI)
  radioStack       : array of enum (same set as §4)
  itsCertChainRef  : string (per IEEE 1609.2 pseudonym certificate
                       set; the certificate-management policy
                       lives in PHASE-3 §5)
  bsmTransmitRate  : enum ("10-hz" | "5-hz" | "event-driven" |
                       "user-defined")
```

OBU certificate sets MAY rotate aggressively (typically a 5-minute
pseudonym change for passenger vehicles); the rotation policy is
recorded against the programme, not against the OBU.

## §6 V2X Message Capture Record

For evidence and incident-investigation purposes, programmes
capture representative V2X messages. The capture record is
content-addressed and links to the captured-message archive.

```
v2xCapture:
  captureId        : string (uuidv7)
  programmeId      : string (uuidv7)
  capturedAt       : string (ISO 8601 / RFC 3339)
  capturedNear     : string (network reference per §3)
  messageMix       : object (per-message-type counts; e.g.
                       {"BSM": 12450, "SPaT": 1800, "MAP": 1,
                        "DENM": 6, "TIM": 12})
  archiveRef       : string (content-addressed URI of the
                       PCAP-NG archive)
  archiveDigest    : string (SHA-256)
  retentionWindow  : enum ("incident-only" | "30-days" |
                       "180-days" | "regulatory-required")
```

Personal-data minimisation: capture archives MUST be processed
through the operator's privacy filter (PHASE-3 §6) before any
disclosure outside the programme.

## §7 Signal Control State Record

```
signalControlState:
  stateId          : string (uuidv7)
  intersectionRef  : string (network reference of the controlled
                       intersection)
  capturedAt       : string (ISO 8601)
  controllerVendor : string
  controllerModel  : string
  ntcipAttributes  : object (NTCIP 1202 v03 attributes captured;
                       e.g. phase status, ring/barrier, pedestrian
                       call status)
  spat             : object (SPaT message contents per SAE J2735;
                       per-movement state and confidence)
  preemptionState  : enum ("none" | "transit-priority" |
                       "emergency" | "rail")
  freeOrCoordinated: enum ("free" | "coordinated" | "manual")
```

## §8 Incident Record

Incidents (collisions, debris on roadway, weather impacts,
infrastructure faults) are recorded against the programme and
broadcast through DENM / TIM messages where applicable.

```
incident:
  incidentId       : string (uuidv7)
  programmeId      : string (uuidv7)
  detectedAt       : string (ISO 8601)
  reportedBy       : enum ("operator-cctv" | "obu-detected" |
                       "police-dispatch" | "user-reported" |
                       "automated-detection-system" |
                       "weather-feed")
  classification   : enum ("collision" | "vehicle-disabled" |
                       "debris" | "spilled-load" |
                       "wrong-way-driver" | "construction-zone" |
                       "weather-impact" | "infrastructure-fault" |
                       "other")
  severity         : enum ("info" | "minor" | "major" | "critical")
  affectedReference: string (network reference of the incident
                       location)
  detourPlanRef    : string (URI of the detour plan when one is
                       activated)
  resolvedAt       : string (ISO 8601; absent until resolved)
  rootCauseRef     : string (URI of the root-cause investigation
                       record, when prepared)
```

## §9 Transit Vehicle Position and Schedule Adherence

Programmes that integrate transit consume GTFS-RT VehiclePositions
and TripUpdates and re-publish them in WIA-native form so that
journey-planning aggregators can resolve positions against the
network reference of §3.

```
transitVehiclePosition:
  positionId       : string (uuidv7)
  programmeId      : string (uuidv7)
  agencyRef        : string (transit agency identifier)
  vehicleToken     : string (opaque transit vehicle token)
  routeRef         : string (GTFS route_id)
  tripRef          : string (GTFS trip_id)
  observedAt       : string (ISO 8601)
  position         : object (lat/lon/heading/speed)
  scheduleAdherenceS : integer (negative = early, positive = late)
```

## §10 Vulnerable Road User (VRU) Awareness Record

VRU awareness messages (pedestrian, cyclist, motorcyclist
warnings) are exchanged through Personal Safety Messages (PSM,
SAE J2735) emitted by personal devices, infrastructure detection
of VRUs at instrumented intersections, or ETSI VAM (VRU
Awareness Message). The WIA-native VRU awareness record captures
the operator-side aggregation.

```
vruAwareness:
  awarenessId      : string (uuidv7)
  programmeId      : string (uuidv7)
  observedAt       : string (ISO 8601 / RFC 3339)
  observedNear     : string (network reference per §3)
  vruClass         : enum ("pedestrian" | "cyclist" |
                       "motorcyclist" | "wheelchair-user" |
                       "personal-mobility-device" |
                       "user-defined")
  detectionSource  : enum ("psm-on-air" | "vam-on-air" |
                       "lidar-curbside" | "radar-curbside" |
                       "video-curbside" | "user-reported" |
                       "fused")
  trajectoryRef    : string (content-addressed URI of the
                       de-identified trajectory; trajectories
                       carry no PII per the privacy filter
                       described in PHASE-3 §5)
  alertIssued      : boolean (whether an active alert was
                       broadcast to nearby OBUs)
```

VRU awareness records are subject to privacy filtering before any
disclosure outside the programme; the filter version is recorded
against every disclosure (PHASE-3 §5).

## §11 Work-Zone Geometry Record

Construction-zone geometry is broadcast via TIM (Traveller
Information Messages) so that CAVs can adjust trajectory and
speed envelopes proactively.

```
workZone:
  workZoneId       : string (uuidv7)
  programmeId      : string (uuidv7)
  activatedAt      : string (ISO 8601)
  expectedEndAt    : string (ISO 8601)
  authorisingOrder : string (work-order or permit reference)
  affectedReference: string (network reference of the work zone)
  laneClosurePlan  : object (per-lane closure schedule with
                       per-period reduction in carriageway
                       capacity)
  speedEnvelopeKph : number (advisory or regulatory speed limit
                       within the work zone)
  vmsMessages      : array of string (DMS message texts in
                       active use)
  cavAdvisoryRef   : string (URI of the machine-readable
                       advisory consumed by CAV mapping
                       services)
```

Work-zone records support the operator's incident-management
workflow when an activated work zone overlaps an active incident.

## §12 Conformance

Implementations claiming PHASE-1 conformance emit each of the
records defined above for every operating programme and honour the
location-referencing rules in §3.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-intelligent-transportation
- **Last Updated:** 2026-04-28
