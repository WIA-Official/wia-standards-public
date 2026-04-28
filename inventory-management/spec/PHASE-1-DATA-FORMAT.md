# WIA-inventory-management PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-inventory-management
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-inventory-management. The standard covers persistent
record shapes for inventory operations across multi-site
warehouse, retail, and manufacturing environments: item
master records, lot and serial controlled inventory,
location-controlled stock balances, receiving and put-away,
order picking and shipping, cycle-count and physical-
inventory reconciliations, and the lifecycle of returned
goods. The format is consumed by Warehouse Management
Systems (WMS), Enterprise Resource Planning (ERP) modules,
Transportation Management Systems (TMS), 3PL provider
integrations, and the EDI / API channels through which
trading partners exchange inventory state.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO 4217 (currency codes)
- ISO 3166-1 / 3166-2 (country and subdivision codes)
- ISO/IEC 11578 (UUID)
- ISO 6346 (intermodal container coding)
- ISO 28219 (item identification — specifications for
  identification using bar code symbols)
- ISO/IEC 15962 (RFID for item management — data protocol —
  data encoding rules and logical memory functions)
- ISO/IEC 19987 (EPCglobal Electronic Product Code
  Information Services — EPCIS)
- ISO/IEC 19988 (EPCglobal Core Business Vocabulary — CBV)
- ISO/IEC 27001:2022 (information security management)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- GS1 General Specifications (cited normatively for AI
  application identifiers and the SSCC / GTIN / GLN /
  GRAI / GIAI identifier families)
- GS1 EPCIS 2.0 / CBV 2.0 (canonical event vocabulary for
  inventory events)
- GS1 Digital Link 1.4 (URI representation of GS1 keys)
- UN/CEFACT recommendation 20 (codes for units of measure
  used in international trade)

---

## §1 Scope

This PHASE defines persistent record shapes for the artefacts
an inventory-management programme manages. Implementations
covered include:

- WMS deployments at distribution centres, retail back-of-
  store stockrooms, and manufacturing materials departments.
- ERP inventory-module integrations (Oracle, SAP, Microsoft
  Dynamics, Infor, NetSuite, Odoo, custom).
- 3PL provider gateways for inbound, outbound, and
  cross-dock operations.
- Trading-partner EDI and API channels for advance shipment
  notifications, purchase-order acknowledgements, and
  inventory-available-to-promise feeds.
- Cycle-count and physical-inventory governance platforms.

Demand-forecasting, replenishment-planning, and pricing-
optimisation systems are out of scope; this PHASE addresses
the records of inventory state and movement, not the upstream
demand engines.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
programmeOperator    : string (institutional identifier of
                         the operator)
programmeRegistered  : string (ISO 8601 / RFC 3339)
operatingSites       : array of string (per-site GS1 GLN
                         identifiers; the operator's facilities
                         and 3PL nodes that participate in
                         the programme's inventory)
inventoryClasses     : array of enum ("retail-finished-goods" |
                         "wholesale-distribution" |
                         "manufacturing-raw-materials" |
                         "manufacturing-work-in-progress" |
                         "manufacturing-finished-goods" |
                         "spare-parts" |
                         "consumables-mro" |
                         "controlled-substances" |
                         "perishable-cold-chain" |
                         "hazardous-materials")
programmeStatus      : enum ("draft" | "operating" |
                         "frozen" | "archived")
```

## §3 Item Master Record

```
itemMaster:
  itemId             : string (uuidv7)
  programmeId        : string (uuidv7)
  primaryGtin        : string (GS1 GTIN-14, mandatory for
                         retail and wholesale items)
  alternateIdentifiers : array of object (per-identifier
                         scheme and value; e.g. operator-
                         internal SKU, manufacturer part
                         number, supplier item number,
                         UNSPSC classification code)
  description        : string (UTF-8; localised descriptions
                         held in the localisation registry)
  unitOfMeasure      : string (UN/CEFACT recommendation 20
                         common code; e.g. "EA" each, "KGM"
                         kilogram, "MTR" metre, "L" litre)
  packagingHierarchy : object (each / inner / case / pallet
                         conversion factors and per-level
                         GS1 identifiers — GTIN at each level
                         where applicable)
  itemControl        : enum ("none" | "lot-controlled" |
                         "serial-controlled" |
                         "lot-and-serial-controlled" |
                         "expiration-controlled" |
                         "controlled-substance")
  hazardClassification : enum ("non-hazardous" |
                         "un-class-1-explosive" |
                         "un-class-2-gas" |
                         "un-class-3-flammable-liquid" |
                         "un-class-4-flammable-solid" |
                         "un-class-5-oxidizer" |
                         "un-class-6-toxic-infectious" |
                         "un-class-7-radioactive" |
                         "un-class-8-corrosive" |
                         "un-class-9-misc")
  coldChainProfile   : enum ("ambient" | "chilled-2-8c" |
                         "frozen-minus-18c" |
                         "ultra-cold-minus-70c" |
                         "user-defined")
```

## §4 Location Record

```
location:
  locationId         : string (uuidv7)
  siteRef            : string (GS1 GLN; per §2)
  locationKind       : enum ("receiving-dock" |
                         "putaway-staging" |
                         "active-pick-face" |
                         "reserve-pallet-rack" |
                         "case-flow" |
                         "very-narrow-aisle" |
                         "bulk-floor" |
                         "cold-room" |
                         "freezer" |
                         "controlled-substance-cage" |
                         "haz-mat-room" |
                         "shipping-staging" |
                         "yard-trailer-spot" |
                         "user-defined")
  locationCode       : string (operator's bin / slot
                         identifier; e.g. "A-12-03-B")
  capacityProfile    : object (volume / weight / SKU-mix
                         constraints that the WMS honours
                         during put-away assignment)
  controlledClassification : enum (matches itemControl;
                         locations that hold controlled-
                         substance items carry the matching
                         classification so that put-away and
                         picking workflows respect the
                         classification)
```

## §5 Stock Balance Record

The stock balance is the on-hand-at-location quantity for an
item, optionally further qualified by lot and serial.

```
stockBalance:
  balanceId          : string (uuidv7)
  itemRef            : string (item UUID)
  locationRef        : string (location UUID)
  capturedAt         : string (ISO 8601 / RFC 3339)
  onHandQuantity     : number (in itemMaster's
                         unitOfMeasure)
  allocatedQuantity  : number (allocated to in-flight orders)
  inboundQuantity    : number (expected from in-transit
                         receipts)
  lotRef             : string (lot UUID; absent for non-lot-
                         controlled items)
  serialRefs         : array of string (serial UUIDs; absent
                         for non-serial-controlled items)
  expirationDate     : string (ISO 8601 date; required for
                         expiration-controlled and lot-
                         controlled items)
```

## §6 Lot and Serial Records

```
lot:
  lotId              : string (uuidv7)
  itemRef            : string (item UUID)
  lotCode            : string (manufacturer or supplier lot
                         number; e.g. "L240312-A1")
  manufacturedAt     : string (ISO 8601 date)
  expirationDate     : string (ISO 8601 date; required for
                         expiration-controlled items)
  manufacturerRef    : string (GS1 GLN of the manufacturer)
  countryOfOrigin    : string (ISO 3166-1 alpha-2)
  qaReleaseRef       : string (URI of the lot's QA release
                         certificate; required for regulated
                         products)

serial:
  serialId           : string (uuidv7)
  itemRef            : string (item UUID)
  lotRef             : string (lot UUID; required when item
                         is lot-and-serial-controlled)
  serialNumber       : string (manufacturer or operator-
                         assigned serial number)
  warrantyStartAt    : string (ISO 8601 date)
  warrantyEndAt      : string (ISO 8601 date)
```

## §7 Inventory Movement Event

Movement events follow GS1 EPCIS 2.0 ObjectEvent / AggregationEvent
/ TransactionEvent / TransformationEvent semantics so that
WIA-native events interoperate with the broader supply-chain
ecosystem.

```
movementEvent:
  eventId            : string (uuidv7)
  programmeId        : string (uuidv7)
  eventTime          : string (ISO 8601 / RFC 3339)
  recordTime         : string (ISO 8601 / RFC 3339)
  epcisEventKind     : enum ("ObjectEvent" |
                         "AggregationEvent" |
                         "TransactionEvent" |
                         "TransformationEvent" |
                         "AssociationEvent")
  bizStep            : string (CBV 2.0 vocabulary; e.g.
                         "receiving", "shipping", "picking",
                         "packing", "stocking", "void_shipping")
  disposition        : string (CBV 2.0 vocabulary; e.g.
                         "in_progress", "in_transit",
                         "available_for_sale",
                         "non_sellable_other",
                         "destroyed")
  readPointRef       : string (GS1 GLN of the read point —
                         dock door, pick face, packing station)
  bizLocationRef     : string (GS1 GLN of the business location)
  itemEpcs           : array of string (GS1 EPC URI per
                         affected item; e.g. SGTIN, SSCC,
                         GIAI)
  parentEpc          : string (parent EPC for aggregation
                         events; e.g. SSCC for case-on-
                         pallet)
  bizTransactionRefs : array of object (purchase-order, ASN,
                         despatch-advice, return-merchandise-
                         authorisation references)
  sourceDestination  : array of object (source / destination
                         party identifiers per CBV 2.0)
```

## §8 Receiving and Shipping Record

```
receiving:
  receivingId        : string (uuidv7)
  asnRef             : string (advance shipment notification
                         reference)
  carrierRef         : string (carrier identifier)
  trailerSealNumber  : string
  receivedAt         : string (ISO 8601)
  putawayCompletedAt : string (ISO 8601; absent until put-
                         away done)
  discrepancyRecords : array of object (per-line under-,
                         over-, or damaged-shipment
                         discrepancies)

shipping:
  shippingId         : string (uuidv7)
  orderRef           : string (operator's outbound-order
                         reference)
  carrierRef         : string
  trackingNumber     : string
  shippedAt          : string (ISO 8601)
  packagedSscc       : array of string (GS1 SSCC for each
                         shipped pallet or case)
  proofOfDeliveryRef : string (URI of the proof-of-delivery
                         artefact; absent until delivered)
```

## §9 Cycle-Count and Physical-Inventory Record

```
inventoryCount:
  countId            : string (uuidv7)
  programmeId        : string (uuidv7)
  countKind          : enum ("cycle-count-abc" |
                         "cycle-count-random" |
                         "spot-count-discrepancy-investigation"
                         | "wall-to-wall-physical")
  countedAt          : string (ISO 8601)
  counterRef         : string (opaque associate token; clinical
                         identity in operator HR)
  scopedLocationRefs : array of string (location UUIDs counted)
  countResults       : array of object (per-item per-location
                         counted quantity vs system quantity
                         and computed variance)
  approvalRef        : string (URI of the count adjustment
                         approval record; required to post
                         variance adjustments)
```

## §10 Recall Record

```
recall:
  recallId           : string (uuidv7)
  programmeId        : string (uuidv7)
  initiatedAt        : string (ISO 8601)
  recallKind         : enum ("manufacturer-initiated" |
                         "regulator-mandated" |
                         "operator-initiated-quality-hold")
  recallClassification : enum ("class-i-imminent-health-risk" |
                         "class-ii-potential-health-risk" |
                         "class-iii-non-health-related")
  affectedLots       : array of string (lot UUIDs)
  affectedSerials    : array of string (serial UUIDs;
                         optional — recalls are typically
                         lot-level, serial-level recalls
                         exist for high-value or
                         individually-traceable items)
  rootCauseRef       : string (URI of the root-cause
                         narrative)
  customerCommunicationRef : string (URI of the customer
                         communication template)
  regulatorNotificationRef : string (URI of the regulator
                         notification artefact)
  closedAt           : string (ISO 8601; absent until closed)
```

## §11 Conformance

Implementations claiming PHASE-1 conformance emit each of the
records defined above for every operating site and honour the
GS1 EPCIS 2.0 / CBV 2.0 vocabulary in §7.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-inventory-management
- **Last Updated:** 2026-04-28
