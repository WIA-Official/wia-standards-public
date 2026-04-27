# WIA-infrastructure-integration PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-infrastructure-integration
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for WIA-
infrastructure-integration. The standard governs the records that an
accredited operator publishes when binding heterogeneous infrastructure
control and information systems — substation automation systems,
energy-management systems, building-management systems, water and
wastewater SCADA, transit operations centres, distributed-energy-
resource management systems, GIS portals, BIM Common Data
Environments, asset-management systems, and historian archives —
into a coherent integration fabric that survives across vendor
changes, protocol upgrades, and regulator audits.

References (CITATION-POLICY ALLOW only):

- ISO 15926 (industrial automation systems and integration —
  integration of life-cycle data)
- ISO 16739 (IFC) and IFC4.3 (infrastructure extension)
- ISO 19650 (BIM information delivery)
- ISO 8601 (date and time)
- ISO/IEC 11578 (UUID)
- ISO/IEC 27001:2022 (information security management)
- IEC 61850 (substation automation)
- IEC 60870-5-104 (telecontrol over TCP/IP)
- IEC 62541 (OPC Unified Architecture)
- IEC 61968 / 61970 (Common Information Model — CIM for utility
  applications)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON), RFC 9457 (Problem Details)
- OASIS MQTT 5.0, OASIS AMQP 1.0
- W3C XML Schema Definition 1.1, W3C SPARQL 1.1 (semantic
  integration)

---

## §1 Scope

This PHASE document defines the persistent shapes for the records
that an integration operator exchanges across a multi-system
infrastructure deployment: federation registration, system
descriptor, protocol-binding record, semantic-mapping record,
namespace mapping, message-flow record, replay-window record,
audit-trail record, change-control record, decommissioning, and
archival. It is intended for use by:

- Utility integration architects who connect SCADA, EMS, GIS,
  BMS, and AMI systems.
- Public-works integration teams who connect bridge-management,
  pavement-management, and CMMS systems to a common asset spine.
- Smart-city integration teams who connect transit, energy,
  water, and emergency-response systems to a common operations
  picture.
- Long-term archives that hold integration records beyond the
  operating life of the contributing systems.

Out of scope are the data formats native to each contributing
system; those are governed by the respective vendor-specific
or sector-specific standards. This standard governs the *bridge*
between systems, not the contents of any single system.

## §2 Federation Identifier

```
federationId       : string (uuidv7)
operatorOrgId      : string (institutional identifier)
federationName     : string (operator-controlled, e.g.
                     "Seoul-Metro-OCC-EMS-AssetSpine")
federationScope    : enum  ("single-utility" | "multi-utility" |
                     "smart-city-operations" |
                     "transport-operations" |
                     "national-grid" | "user-defined")
created            : string (ISO 8601)
governanceCharterContentAddress : string (URI of the federation's
                     governance charter — naming authority,
                     change-control board, dispute resolution)
```

The federation identifier is opaque and is generated when the
federation is established. Identifier reuse is forbidden — a
federation that is wound down retains its identifier permanently
for citation purposes.

## §3 Contributing-System Descriptor

```
system:
  systemId        : string (uuidv7)
  federationId    : string (uuidv7)
  vendorRef       : string (institutional identifier)
  productName     : string
  productVersion  : string (Semantic Versioning 2.0.0)
  systemRole      : enum  ("scada" | "ems" | "dms" | "ami" |
                     "bms" | "cmms" | "eam" | "gis" | "bim-cde" |
                     "historian" | "ticketing" | "weather-feed" |
                     "user-defined")
  protocolBindings : array of string (protocol-binding record
                     identifiers — see §4)
  certificationRefs : array of string (vendor SOC 2 reports,
                     ISO/IEC 27001 certificate references, IEC
                     62443 certifications, sector-specific
                     accreditation references)
```

System descriptors record the contributing system's role and
certification posture so that integration architects can resolve
the trust level of each end of every cross-system message flow.

## §4 Protocol-Binding Record

```
protocolBinding:
  bindingId       : string (uuidv7)
  systemId        : string (uuidv7)
  protocolFamily  : enum  ("opc-ua" | "iec-61850" |
                     "iec-60870-5-104" | "dnp3" |
                     "modbus-tcp" | "bacnet-ip" |
                     "mqtt-5" | "amqp-1.0" | "kafka" |
                     "rest" | "grpc" | "soap" | "ftp-batch" |
                     "user-defined")
  endpointUri     : string (the resolvable endpoint at which the
                     contributing system publishes or consumes
                     this binding)
  authenticationProfile : string (mutual-TLS profile reference,
                     SASL profile, IEC 62351 profile reference)
  messageSchemaContentAddress : string (URI of the schema —
                     OPC UA NodeSet2.xml, IEC 61850 SCL ICD,
                     OpenAPI document, AsyncAPI document, etc.)
  conformanceClass : string (vendor-published conformance class
                     for the endpoint, e.g. OPC UA Embedded UA
                     Server profile, IEC 61850 Edition 2.1 Logical
                     Node set)
```

Protocol-binding records describe the *wire-level* contract that
the contributing system honours; they are the foundation against
which the integration broker verifies inter-system messages.

## §5 Semantic-Mapping Record

```
semanticMapping:
  mappingId       : string (uuidv7)
  federationId    : string (uuidv7)
  sourceSystemId  : string (uuidv7)
  targetSystemId  : string (uuidv7)
  sourceModelRef  : string (model reference in the source system —
                     e.g. OPC UA NodeId path, IEC 61850 Logical
                     Node identifier, CIM 16 class instance)
  targetModelRef  : string (model reference in the target system)
  mappingMethod   : enum  ("direct" | "scaled" | "computed" |
                     "lookup-table" | "rule-engine" |
                     "ml-classifier")
  mappingExpression : string (operator-published expression in
                     the federation's mapping DSL, or the
                     content-address of a SPARQL CONSTRUCT
                     query)
  validatedBy     : string (institutional identifier of the
                     integration architect or working group that
                     validated this mapping)
  validatedAt     : string (ISO 8601)
```

Semantic-mapping records bind cross-system data elements at the
*meaning* level rather than the wire level. A mapping is the
provenance for any integrated value the federation publishes
downstream.

## §6 Namespace Mapping

The federation maintains a namespace mapping that records the
authoritative naming for cross-system entities (substations,
feeders, pump stations, gates, buildings, transit stops, asset
identifiers from the asset-management system).

```
namespaceMapping:
  namespaceMappingId : string (uuidv7)
  federationId    : string (uuidv7)
  authoritativeSystemId : string (uuidv7)
  participantSystemMappings : array of object (systemId,
                     local-identifier expression)
  resolverEndpoint : string (URI for resolving a local
                     identifier in any participant to the
                     authoritative identifier)
```

Namespace mapping is critical: a federation that lacks a
canonical naming authority degenerates into pairwise mappings
that fragment the integration fabric over time.

## §7 Message-Flow Record

```
messageFlow:
  flowId          : string (uuidv7)
  federationId    : string (uuidv7)
  publisherBindingId : string (uuidv7)
  subscriberBindingIds : array of string
  semanticMappings : array of string (mappingId references)
  qualityOfService : object
    ordering      : enum ("strict" | "per-source" | "best-effort")
    durability    : enum ("at-most-once" | "at-least-once" |
                       "exactly-once")
    latencyBudgetMs : number
  flowOwner       : string (institutional identifier of the
                     integration architect who owns this flow)
  governingChangeProcess : string (reference to the change-control
                     procedure that governs amendments to this
                     flow — see §10)
```

Message-flow records bind a publisher to its subscribers under
explicit quality-of-service constraints. The QoS object is the
contract that the integration broker honours; deviations emit
priority-1 events on the federation's event stream.

## §8 Replay Window Record

Federations that consume historian archives or vendor-provided
event replays maintain replay-window records:

```
replayWindow:
  windowId        : string (uuidv7)
  federationId    : string (uuidv7)
  flowRefs        : array of string (flowId references)
  windowStart     : string (ISO 8601 / RFC 3339)
  windowEnd       : string (ISO 8601 / RFC 3339)
  replayPolicy    : enum  ("incident-investigation" |
                     "model-revalidation" |
                     "archive-restore" | "training-data-set")
  replayInitiatedBy : string (institutional identifier)
  approvalsContentAddress : string (URI of the approval pack
                     authorising the replay)
```

Replay windows are governed because replay traffic on production
flows can mask current real-time events; the federation's change-
control board records the approvals that authorised any replay
that touched a production flow.

## §9 Audit-Trail Record

```
auditTrail:
  auditId         : string (uuidv7)
  federationId    : string (uuidv7)
  flowRef         : string (flowId)
  occurredAt      : string (ISO 8601 / RFC 3339)
  actor           : string (institutional identifier)
  action          : enum ("flow-created" | "flow-modified" |
                       "flow-suspended" | "flow-restored" |
                       "mapping-validated" | "mapping-deprecated" |
                       "replay-initiated" | "replay-completed")
  changeContentAddress : string (URI of the change record — JSON
                       Patch RFC 6902 against the prior state)
```

Audit-trail records are immutable and form the evidence base for
post-incident review and regulator-driven audits.

## §10 Change-Control Record

```
changeControl:
  changeId        : string (uuidv7)
  federationId    : string (uuidv7)
  changeType      : enum  ("schema-revision" |
                     "protocol-binding-revision" |
                     "semantic-mapping-revision" |
                     "namespace-revision" |
                     "system-onboarding" |
                     "system-decommissioning")
  affectedRefs    : array of string (record identifiers affected)
  proposedAt      : string (ISO 8601)
  approvedAt      : string (ISO 8601, nullable until approved)
  approvers       : array of string (institutional identifiers
                     of the change-control board members who
                     approved the change)
  rolloutWindow   : object
    plannedStart  : string (ISO 8601)
    plannedEnd    : string (ISO 8601)
  rollbackPlanContentAddress : string (URI of the rollback plan
                     that the operator commits to execute on
                     failure)
```

## §11 Decommissioning

```
decommissioning:
  decommissioningId : string (uuidv7)
  federationId    : string (uuidv7)
  systemRef       : string (uuidv7)
  reason          : enum ("vendor-end-of-life" |
                       "consolidation" | "regulatory-divestiture" |
                       "incident-driven-removal")
  decommissionDate : string (ISO 8601)
  archivalDepositRef : string
```

## §12 Conformance

Implementations claiming PHASE-1 conformance emit each record
type for every contributing system and message flow under their
operation, hold the federation governance charter and the change-
control register at the content-addresses recorded above, and
honour identifier-permanence and immutability rules in §2 / §9.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-infrastructure-integration
- **Last Updated:** 2026-04-28
