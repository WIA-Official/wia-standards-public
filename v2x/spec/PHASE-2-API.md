# WIA-AUTO-003 — Phase 2: API Interface

> V2X canonical Phase 2: API surface (registrations + MAP + message-stream + misbehaviour + CRL).

# WIA-AUTO-003: V2X - Vehicle-to-Everything Communication Specification v1.0

> **Standard ID:** WIA-AUTO-003
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [V2X Architecture](#2-v2x-architecture)
3. [Communication Technologies](#3-communication-technologies)
4. [Message Types and Formats](#4-message-types-and-formats)
5. [Security and Authentication](#5-security-and-authentication)
6. [Data Protocols](#6-data-protocols)
7. [API Interface](#7-api-interface)
8. [Latency and Performance Requirements](#8-latency-and-performance-requirements)
9. [Safety Protocols](#9-safety-protocols)
10. [References](#10-references)

---


## 7. API Interface

### 7.1 Core APIs

#### 7.1.1 Message Transmission API

```typescript
// Send Basic Safety Message
interface SendBSMRequest {
  position: GeoPosition;
  speed: number;          // m/s
  heading: number;        // degrees
  acceleration: Vector3;
  brakeStatus: BrakeStatus;
  vehicleSize: VehicleSize;
  partII?: BSMPartII;    // Optional extended data
}

interface SendBSMResponse {
  messageId: string;
  timestamp: number;
  transmissionStatus: 'success' | 'queued' | 'failed';
  error?: string;
}

async function sendBasicSafetyMessage(
  request: SendBSMRequest
): Promise<SendBSMResponse>;
```

#### 7.1.2 Message Reception API

```typescript
// Register message handler
interface MessageHandler {
  onBasicSafetyMessage(msg: BasicSafetyMessage): void;
  onSpatMessage(msg: SignalPhaseAndTiming): void;
  onMapMessage(msg: MapData): void;
  onHazardWarning(msg: HazardWarning): void;
}

function registerMessageHandler(handler: MessageHandler): void;

// Message filtering
interface MessageFilter {
  messageTypes?: MessageType[];
  geoFilter?: GeographicArea;
  senderId?: string[];
  minPriority?: number;
}

function setMessageFilter(filter: MessageFilter): void;
```

#### 7.1.3 Security API

```typescript
// Certificate management
interface CertificateInfo {
  certificateId: string;
  type: 'enrollment' | 'pseudonym' | 'authorization';
  validFrom: Date;
  validUntil: Date;
  permissions: Permission[];
}

async function getCertificates(): Promise<CertificateInfo[]>;
async function requestPseudonymCertificates(count: number): Promise<void>;
async function rotateCertificate(): Promise<void>;

// Message verification
interface VerificationResult {
  isValid: boolean;
  certificateValid: boolean;
  signatureValid: boolean;
  notRevoked: boolean;
  plausibilityCheck: boolean;
  errors?: string[];
}

async function verifyMessage(msg: V2XMessage): Promise<VerificationResult>;
```

#### 7.1.4 Communication Control API

```typescript
// Start/stop V2X communication
interface V2XConfig {
  vehicleId: string;
  technology: 'DSRC' | 'C-V2X' | 'Hybrid';
  frequency: number;        // MHz
  transmitPower: number;    // dBm
  bsmRate: number;          // Hz
  securityLevel: 'low' | 'medium' | 'high';
}

async function startV2XCommunication(config: V2XConfig): Promise<void>;
async function stopV2XCommunication(): Promise<void>;
async function getConnectionStatus(): Promise<ConnectionStatus>;
```

### 7.2 Application APIs

#### 7.2.1 Collision Warning API

```typescript
interface CollisionWarning {
  severity: 'low' | 'medium' | 'high' | 'critical';
  timeToCollision: number;  // seconds
  distance: number;         // meters
  relativeSpeed: number;    // m/s
  collisionType: 'frontal' | 'rear' | 'side' | 'intersection';
  otherVehicle: VehicleInfo;
  recommendedAction: 'brake' | 'steer-left' | 'steer-right' | 'none';
}

function onCollisionWarning(callback: (warning: CollisionWarning) => void): void;
```

#### 7.2.2 Traffic Signal API

```typescript
interface TrafficSignalInfo {
  intersectionId: string;
  currentPhase: 'green' | 'yellow' | 'red';
  timeRemaining: number;    // seconds
  nextPhase: string;
  distance: number;         // meters to intersection
  recommendedSpeed: number; // m/s for green wave
}

function onTrafficSignal(callback: (signal: TrafficSignalInfo) => void): void;
async function requestSignalPriority(reason: 'emergency' | 'transit'): Promise<boolean>;
```

#### 7.2.3 Platooning API

```typescript
interface PlatoonInfo {
  platoonId: string;
  leadVehicle: string;
  members: string[];
  targetSpeed: number;      // m/s
  targetSpacing: number;    // meters
  position: number;         // Position in platoon (0 = leader)
}

async function joinPlatoon(platoonId: string): Promise<boolean>;
async function leavePlatoon(): Promise<void>;
async function createPlatoon(): Promise<string>;
function onPlatoonUpdate(callback: (info: PlatoonInfo) => void): void;
```

### 7.3 Data Access APIs

#### 7.3.1 Vehicle State API

```typescript
interface VehicleState {
  position: GeoPosition;
  speed: number;
  heading: number;
  acceleration: Vector3;
  steeringAngle: number;
  brakes: BrakeStatus;
  lights: LightStatus;
  transmission: TransmissionState;
  vehicleType: VehicleType;
}

function getCurrentVehicleState(): VehicleState;
function subscribeToVehicleState(callback: (state: VehicleState) => void): void;
```

#### 7.3.2 Nearby Vehicles API

```typescript
interface NearbyVehicle {
  vehicleId: string;        // Temporary/pseudonym ID
  position: GeoPosition;
  relativePosition: Vector3; // Relative to ego vehicle
  speed: number;
  heading: number;
  vehicleType: VehicleType;
  lastUpdate: number;       // Timestamp
  confidence: number;       // 0-1
}

function getNearbyVehicles(maxDistance: number): NearbyVehicle[];
function subscribeToNearbyVehicles(
  callback: (vehicles: NearbyVehicle[]) => void,
  updateRate: number
): void;
```

---




---

## A.1 Endpoint reference

```http
POST /v2x/v1/registrations              # register vehicle or RSU station
GET  /v2x/v1/stations/{id}              # fetch station record
POST /v2x/v1/maps                       # publish intersection MAP record
GET  /v2x/v1/maps/{id}                  # fetch MAP record (by version)
WS   /v2x/v1/messages/stream            # multiplexed V2X message stream
POST /v2x/v1/misbehaviour-reports       # submit misbehaviour report
GET  /v2x/v1/crl                        # fetch certificate revocation list
GET  /v2x/v1/health/{stationId}         # liveness + signal health
```

Every endpoint follows the discovery convention at `/.well-known/wia-v2x`. Privileged endpoints require a IEEE 1609.2 enrolment certificate plus a fresh-quorum signature.

## A.2 Station-registration API

`POST /registrations` accepts a station-registration envelope per Phase 1 §A.2; the response carries the issued enrolment-certificate envelope plus the initial pseudonym-certificate batch. Pseudonym-certificate batches rotate per the operator's policy (typical 5 minutes per pseudonym, with batch reload before exhaustion); the rotation policy is captured in the envelope and signed by the issuing certificate authority. Station revocation follows the IEEE 1609.2.1 Security Credential Management System (SCMS) misbehaviour-report flow.

## A.3 MAP and SPaT API

`POST /maps` accepts an intersection MAP record per SAE J2735 with the per-lane geometry, allowed manoeuvres, and connection-to-signal mapping. MAP version control is incremental: each update carries a `revision` field, and SPaT broadcasts include the matching `intersectionId.revision` so consumers can detect MAP-revision skew and request the latest version. SPaT records are not persisted via REST (the broadcast is the canonical channel); the API exposes SPaT only for diagnostic and replay purposes.

## A.4 Message-stream WebSocket

The message-stream WebSocket multiplexes the J2735 / ETSI message catalogue with subscriber-side filtering by message type, geographic bounding box, and originating-station-class. The broker enforces the IEEE 1609.2 certificate validation chain and rejects messages with revoked or expired certificates per the ETSI TS 102 941 management protocol. Subscribers MUST validate the message signature against the publisher's certificate chain before processing; failure to validate is a conformance defect.

## A.5 Misbehaviour-report and CRL API

`POST /misbehaviour-reports` registers a misbehaviour report (e.g., position anomalies, signature failures, replay attempts, certificate misuse). The report is gated by an authenticated station credential and includes the offending station's pseudonym-certificate identifier, the observed misbehaviour type, the supporting evidence (the offending message envelope), and the reporter's evidence-chain. The SCMS processes reports asynchronously and publishes revoked certificates to the CRL at `/crl`; consumers fetch the CRL on a documented refresh cadence (default 1 hour, configurable per operator policy).

## A.6 Rate-limit and conformance envelope

V2X message broadcast follows IEEE 802.11p / C-V2X PC5 channel-access rules, not REST rate-limits. The REST surface (registration, MAP publication, misbehaviour reports) is rate-limited per IEEE 1609.2 enrolment-certificate: 100 req/h for unauthenticated read-only diagnostics, 1000 req/h for authenticated stations, 10000 req/h for authorised SCMS clients. WebSocket subscriptions to `/messages/stream` are bounded at 100 simultaneous subscriptions per credential.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/v2x/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-v2x-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/v2x-host:1.0.0` ships every v2x envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/v2x.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. V2x deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
