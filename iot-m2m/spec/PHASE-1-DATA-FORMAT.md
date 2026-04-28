# WIA-iot-m2m PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-iot-m2m
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-iot-m2m. The standard covers the persistent record shapes
exchanged among Internet-of-Things devices (sensors,
actuators, gateways), Machine-to-Machine (M2M) service
platforms (oneM2M-aligned, LwM2M-aligned, broker-based MQTT
deployments), application servers, and the regulators or
operators that govern an IoT-M2M deployment. The format is
intentionally agnostic of the deployment's vertical (smart-
city, smart-agriculture, asset-tracking, building automation)
and addresses the cross-cutting records that every deployment
emits.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 30141:2018 (IoT Reference Architecture)
- ISO/IEC 30161 (IoT Data exchange platform)
- ISO/IEC 21823-1/2/3 (IoT interoperability)
- IETF RFC 4122 (UUID URN)
- IETF RFC 7252 (Constrained Application Protocol — CoAP)
- IETF RFC 8132 (PATCH and FETCH methods for CoAP)
- IETF RFC 8259 (JSON)
- IETF RFC 8949 (Concise Binary Object Representation — CBOR)
- IETF RFC 9457 (Problem Details)
- IEEE 802.15.4 (Low-Rate Wireless Personal Area Networks)
- IEEE 802.11ah (Wi-Fi HaLow for IoT-class deployments)
- 3GPP TS 36.300 / TS 38.300 (Cellular IoT — Cat-M / NB-IoT
  / Reduced Capability NR)
- oneM2M TS-0001 (Functional Architecture)
- oneM2M TS-0004 (Service Layer Core Protocol)
- oneM2M TS-0008 / TS-0009 (CoAP / HTTP bindings)
- ETSI TS 102 690 (Machine-to-Machine communications;
  functional architecture)
- OMA Lightweight M2M (LwM2M) v1.2
- OASIS MQTT v5.0
- W3C Web of Things (WoT) Thing Description 1.1

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts an
IoT-M2M deployment produces and consumes. Implementations
covered include:

- Constrained-device firmware that emits sensor readings
  and consumes actuator commands.
- Gateway / proxy systems that bridge constrained-device
  protocols (CoAP, BLE, Zigbee, Z-Wave) to back-end
  service-layer protocols (oneM2M, LwM2M, MQTT, HTTP).
- Service-layer platforms (oneM2M-aligned Common Service
  Entities, LwM2M servers, MQTT brokers, custom IoT
  platforms) that mediate between devices and applications.
- Application servers that consume device telemetry and
  emit actuation requests.
- Lifecycle-management systems (device provisioning,
  firmware-over-the-air updates, key rotation, decommission).

Data analytics platforms downstream of the M2M layer (time-
series analytics, ML inference) are out of scope; this PHASE
defines the records that flow up to those platforms, not the
analytics outputs.

## §2 Deployment Identifier

```
deploymentId         : string (uuidv7)
deploymentOperator   : string (institutional identifier of
                         the operator)
deploymentRegistered : string (ISO 8601 / RFC 3339)
verticalDomain       : enum ("smart-city" |
                         "smart-agriculture" |
                         "smart-building" |
                         "industrial-plant-floor" |
                         "asset-tracking-logistics" |
                         "metering-utilities" |
                         "fleet-telematics" |
                         "healthcare-remote-monitoring" |
                         "consumer-home-automation" |
                         "user-defined")
serviceLayerKind     : enum ("oneM2M" | "LwM2M" |
                         "MQTT-broker" | "WoT-native" |
                         "custom")
deploymentStatus     : enum ("design" | "operating" |
                         "frozen" | "decommissioning" |
                         "archived")
```

## §3 Device Identifier and Lifecycle

```
device:
  deviceId           : string (uuidv7)
  deploymentId       : string (uuidv7)
  manufacturerRef    : string (institutional identifier)
  modelNumber        : string
  hardwareRevision   : string
  firmwareRef        : string (content-addressed firmware URI)
  deviceClass        : enum ("constrained-class-0" |
                         "constrained-class-1" |
                         "constrained-class-2" |
                         "non-constrained" |
                         "gateway-proxy")
  radioStack         : array of enum ("ieee-802-15-4-zigbee"
                         | "ieee-802-15-4-thread" |
                         "bluetooth-mesh" | "ble-classic" |
                         "z-wave" | "wi-fi-halow" |
                         "wi-fi-classic" |
                         "lorawan-1.1" | "sigfox" |
                         "nb-iot-cat-m" |
                         "5g-redcap" |
                         "wired-rs-485" |
                         "wired-ethernet" |
                         "user-defined")
  rootKeyDerivationRef : string (URI of the device's root-
                         key derivation record; the actual
                         key is held in HSM and never on
                         this API)
  deviceStatus       : enum ("manufactured" | "provisioned"
                         | "deployed-active" | "suspended"
                         | "fault" | "decommissioned")
```

## §4 Thing Description Record

Every device is described by a W3C WoT Thing Description so
that downstream consumers can discover the device's
properties, actions, and events without bespoke device
documentation.

```
thingDescription:
  tdId               : string (uuidv7)
  deviceRef          : string (device UUID)
  tdJsonLdRef        : string (content-addressed URI of the
                         WoT TD JSON-LD document conforming
                         to W3C WoT TD 1.1)
  tdContentDigest    : string (SHA-256)
  affordances        : object (per-affordance summary —
                         properties, actions, events;
                         redundant with the TD JSON-LD but
                         provided for indexable search)
  protocolBindings   : array of enum ("coap" | "http" |
                         "mqtt" | "websocket" | "modbus" |
                         "opcua" | "user-defined")
```

## §5 Resource Tree Record (oneM2M Pattern)

oneM2M-aligned deployments expose a hierarchical resource
tree (Common Service Entity, Application Entity, Container,
ContentInstance). This standard records the canonical
resource-tree path for each managed resource so that
consumers can resolve resource paths without bespoke service-
layer knowledge.

```
resourceTreeNode:
  nodeId             : string (uuidv7)
  deploymentId       : string (uuidv7)
  parentNodeId       : string (uuidv7; absent for the root)
  nodeKind           : enum ("CSE" | "AE" | "container" |
                         "contentInstance" |
                         "subscription" |
                         "accessControlPolicy" |
                         "node" | "remoteCSE" |
                         "user-defined")
  resourcePath       : string (absolute path under the
                         deployment's CSE base)
  contentTypeHint    : string (MIME type that the
                         contentInstance leaf carries; e.g.
                         "application/cbor",
                         "application/json",
                         "application/octet-stream")
  retentionPolicyRef : string (URI of the per-node retention
                         policy)
```

## §6 Telemetry Record

Telemetry is the sensor-reading flow from devices to
applications. Records are normalized so that downstream
consumers can ingest telemetry from heterogeneous device
populations without per-device parsing.

```
telemetry:
  telemetryId        : string (uuidv7)
  deviceRef          : string (device UUID)
  observedAt         : string (ISO 8601 / RFC 3339, with
                         millisecond precision when the
                         device's clock supports it)
  sensorAffordance   : string (matches a property name from
                         the device's Thing Description)
  numericValue       : number (when the affordance is
                         numeric)
  textValue          : string (when the affordance is
                         categorical / text)
  binaryRef          : string (content-addressed URI when
                         the affordance is a binary blob —
                         image, audio, raw waveform)
  unitCode           : string (UN/CEFACT recommendation 20
                         common code, where applicable)
  qualityFlag        : enum ("nominal" | "estimated" |
                         "uncertain" | "out-of-range" |
                         "device-fault")
  ingestionLatencyMs : integer (operator-side measure; the
                         delay from observedAt to
                         service-layer ingest)
```

## §7 Actuation Record

```
actuation:
  actuationId        : string (uuidv7)
  deviceRef          : string (device UUID)
  requestedAt        : string (ISO 8601)
  actionAffordance   : string (matches an action name from
                         the device's Thing Description)
  inputPayloadRef    : string (URI of the action input)
  acknowledgedAt     : string (ISO 8601; absent until
                         device acknowledges)
  completedAt        : string (ISO 8601; absent until device
                         signals completion)
  outcome            : enum ("succeeded" | "failed" |
                         "partial" | "timed-out" |
                         "rejected-by-policy")
  outputPayloadRef   : string (URI of the action output)
```

## §8 Firmware-Over-The-Air (FOTA) Record

```
fota:
  fotaId             : string (uuidv7)
  deviceRef          : string (device UUID)
  initiatedAt        : string (ISO 8601)
  fromFirmwareRef    : string (URI of prior firmware)
  toFirmwareRef      : string (URI of new firmware;
                         signed per the operator's
                         firmware-signing policy)
  fotaState          : enum ("staged" | "downloading" |
                         "verified" | "applied" |
                         "rolled-back" | "failed")
  rollbackEvidenceRef: string (URI of rollback evidence;
                         absent unless the FOTA rolled
                         back)
```

## §9 Subscription and Notification Record

The subscribe-and-notify pattern is fundamental to oneM2M
and LwM2M; subscriptions enable application servers to
receive asynchronous notifications when device state
changes without polling.

```
subscription:
  subscriptionId     : string (uuidv7)
  deploymentId       : string (uuidv7)
  subscriberRef      : string (application-entity identifier)
  resourceTreeNodeRef: string (the resource-tree node the
                         subscription targets)
  notificationKind   : enum ("any-update" |
                         "create-only" | "delete-only" |
                         "update-only" |
                         "value-threshold-crossed" |
                         "user-defined")
  notificationEndpointRef: string (URI of the subscriber's
                         notification endpoint)
  encodingPreference : enum ("json" | "cbor" | "xml")
  expiresAt          : string (ISO 8601; subscriptions
                         expire and require renewal)
```

## §10 Audit Log Record

```
auditLog:
  logId              : string (uuidv7)
  deploymentId       : string (uuidv7)
  occurredAt         : string (ISO 8601)
  actorRef           : string (operator identifier of the
                         actor — application server,
                         operator console, device, automated
                         workflow)
  action             : enum ("device-provisioned" |
                         "device-status-changed" |
                         "td-updated" |
                         "actuation-requested" |
                         "fota-initiated" |
                         "fota-rolled-back" |
                         "subscription-created" |
                         "subscription-expired" |
                         "key-rotated" |
                         "device-decommissioned" |
                         "user-defined")
  resourceRef        : string (URI of the affected resource)
  outcome            : enum ("succeeded" | "failed" |
                         "partial")
  ipAddressOpaque    : string (opaque, hashed network-
                         identifier; the operator records
                         actor network position for
                         security audit without exposing
                         raw IP)
```

## §11 Gateway Aggregation Record

Gateway devices aggregate telemetry from multiple
constrained children (Zigbee end-devices behind a Zigbee
coordinator gateway, BLE peripherals behind a BLE
gateway, RS-485 sensors behind an industrial gateway).
The aggregation record captures the parent-child binding.

```
gatewayAggregation:
  aggregationId      : string (uuidv7)
  gatewayDeviceRef   : string (gateway device UUID)
  childDeviceRefs    : array of string (device UUIDs)
  childProtocol      : enum ("zigbee" | "thread" |
                         "bluetooth-mesh" | "ble" |
                         "z-wave" | "modbus-rtu-485" |
                         "modbus-tcp" | "opcua" |
                         "user-defined")
  bindingEstablishedAt : string (ISO 8601)
  bindingRevokedAt   : string (ISO 8601; absent for
                         active bindings)
```

Gateway aggregation records support the operator's
device-population analytics (which children are behind
which gateway) and the operator's incident-response
workflows (an unreachable gateway implies its children
are also unreachable).

## §12 Battery and Energy Telemetry Record

```
batteryTelemetry:
  recordId           : string (uuidv7)
  deviceRef          : string (device UUID)
  capturedAt         : string (ISO 8601)
  batteryLevelPct    : number (0-100, where 0 indicates the
                         operator's defined minimum operating
                         voltage)
  batteryVoltageV    : number (volts)
  estimatedRemainingHours : number (per the operator's
                         per-device-class battery model)
  energyHarvestedSinceLastReportJ : number (joules,
                         for energy-harvesting devices;
                         absent for non-harvesting devices)
```

Battery telemetry feeds the operator's per-device-class
battery-life trajectory model (PHASE-3 §17), which drives
proactive replacement scheduling so that field operations
do not encounter unexpected dead devices.

## §13 Conformance

Implementations claiming PHASE-1 conformance emit each of
the records defined above for every managed device and
honour the W3C WoT TD content-addressing rule in §4.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-iot-m2m
- **Last Updated:** 2026-04-28
