# WIA-inventory-management PHASE 4 — INTEGRATION Specification

**Standard:** WIA-inventory-management
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an inventory-management programme
integrates with the systems that surround it: ERP suites
(SAP, Oracle, Microsoft Dynamics, NetSuite, Infor, Odoo,
custom); 3PL provider gateways; trading-partner EDI / API
channels (EDIFACT, X12, AS2, peppol); transportation
management systems; cold-chain monitoring services;
controlled-substance regulators (US DEA, EU national
authorities, KFDA); GS1 member organisations; e-commerce
storefronts; and long-term archives.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 8601 (date and time)
- GS1 EPCIS 2.0 / CBV 2.0
- GS1 Digital Link 1.4
- UN/EDIFACT D.18B (and successor versions)
- ANSI ASC X12 EDI specifications
- W3C Verifiable Credentials Data Model 2.0 (optional)

---

## §1 ERP Suite Integration

ERP integration carries the per-vendor adapter identifier
(SAP IDOC mapping, Oracle Fusion Inventory adapter,
Microsoft Dynamics WMS adapter, NetSuite native, Infor SCE
adapter, Odoo connector, custom-built integration), the
per-record-class master-data direction (operator-as-master
vs ERP-as-master), and the per-event reconciliation cadence.

ERP-as-master records (typically item master, supplier
master, customer master) flow downward to the WMS; WMS-as-
master records (typically stock balance, receiving,
shipping, cycle-count results) flow upward to the ERP for
financial-inventory reconciliation.

## §2 3PL Provider Gateway Integration

3PL provider integration carries the 3PL operator's
identifier, the per-warehouse adapter manifest, the per-
event reconciliation cadence, and the dispute-resolution
path when 3PL-side movement events disagree with the
operator's master record. Multi-3PL operators (omni-channel
retailers using regional 3PLs) maintain per-3PL adapter
mappings to a unified WIA event vocabulary.

## §3 Trading-Partner EDI / API Channel Integration

Trading partners exchange inventory state through EDIFACT
(D.18B and successors), X12, peppol, or trading-partner-
specific JSON / XML APIs. Common transactions:

- DESADV (advance shipment notification);
- ORDERS (purchase order);
- ORDRSP (purchase order acknowledgement);
- INVRPT (inventory report);
- RECADV (receiving advice);
- RETANN (return announcement).

Integration carries each partner's identifier, the per-
transaction template, and the per-channel certification
(AS2 partner certificates, peppol participant identifiers,
X12 GS-segment identifiers).

## §4 Transportation Management System Integration

TMS integration consumes shipping records (PHASE-1 §8) for
load planning, carrier tendering, and tracking. Integration
carries the TMS's identifier, the per-shipment data
contract, and the proof-of-delivery (POD) ingestion
endpoint that the TMS uses to attach POD images and
electronic signatures back to the shipping record.

## §5 Cold-Chain Monitoring Service Integration

Cold-chain monitoring services (Sensitech, ELPRO, Berlinger,
operator-internal IoT loggers) emit temperature trip data
that the operator's WMS attaches to the per-shipment cold-
chain integrity record. Integration carries the monitoring
service's identifier, the per-logger device-id allocation,
and the excursion-event notification endpoint that triggers
the operator's quality-deviation workflow.

## §6 Controlled-Substance Regulator Integration

Controlled-substance regulators (US DEA via ARCOS,
EU national competent authorities, KFDA in Korea,
PMDA-equivalent in Japan, equivalent authorities elsewhere)
consume the operator's reconciliation reports and discrepancy
notifications. Integration carries the regulator's
identifier, the per-licence-class submission template, and
the discrepancy-notification SLA that the operator commits
to.

## §7 GS1 Member Organisation Integration

GS1 member organisations (GS1 US, GS1 UK, GS1 Korea, GS1
Japan, GS1 Germany, equivalent national MOs) consume the
operator's prefix-assignment and registration updates.
Integration carries the MO's identifier, the per-prefix
assignment record, and the operator's renewal cadence.
Operators that participate in the GS1 Global Data Synchronization
Network (GDSN) emit item-master synchronisation messages
through the GDSN data pool.

## §8 E-Commerce Storefront Integration

E-commerce storefronts (Shopify, Magento, BigCommerce,
WooCommerce, custom marketplaces) consume real-time stock
balances for the available-to-promise display on product
detail pages. Integration carries the storefront's
identifier, the per-storefront product catalogue mapping,
and the inventory-availability cache TTL that the storefront
honours so that stock-out scenarios do not propagate to
shopper carts.

## §9 Evidence Package Format

```
evidence/
  manifest.json              — package manifest (signed)
  programme.json             — programme record
  items/                     — item master records
  locations/                 — location records
  lots-and-serials/          — lot and serial records
  movement-events/           — EPCIS event log for the cited
                                interval
  receiving-and-shipping/    — receiving and shipping records
  inventory-counts/          — count records and approvals
  audit/                     — API audit log excerpts
```

The package is content-addressable; the manifest is signed
by the operator's HTTP-message-signature key (RFC 9421) and
counter-signed by the operator's quality manager when the
package supports a regulator submission.

## §10 Manifest and Signatures

Verification tools recompute file digests, compare to the
manifest, and reject the package on mismatch with type
`urn:wia:inventory-management:evidence-mismatch`.

## §11 well-known URI Discovery

A conformant operator exposes a discovery document at
`/.well-known/wia-inventory-management` that links to the
API root, the GS1 prefix-assignment summary, the operator's
quality dossier, the per-jurisdiction regulator licences,
and the catalogue of operating sites.

## §12 Long-Term Archive Integration

Operators designate a long-term archive that holds movement
events, lot release certificates, and controlled-substance
reconciliation reports beyond the operator's primary
retention horizon. Quarterly deposits round-trip content-
addresses; on programme wind-down, remaining records
transfer to the archive with content-addresses preserved.

## §13 Verifiable-Credential Re-Issuance (optional)

Operators that wish to expose attestations (GS1 prefix
assignment, ISO 9001 conformance, ISO 22000 conformance,
ISO/IEC 27001 certification, GDP / cGMP regulator
certifications) to consumers of W3C Verifiable Credentials
MAY re-issue the attestations as Verifiable Credentials
under the Data Model 2.0 specification. Re-issuance is
optional; the canonical record remains the JSON evidence-
package manifest.

## §14 Streaming Heartbeat

SSE subscribers receive a heartbeat every 30 seconds with
`Last-Event-ID` resume support. Subscribers that disconnect
during long receiving / shipping windows resume from the
last seen event identifier without losing visibility of
priority-1 events (cold-chain excursions, controlled-
substance discrepancies, hazmat segregation violations).

## §15 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with
prior-minor clients. Major revisions go through a
deprecation window of at least one full GS1 EPCIS / CBV
release cycle so that trading-partner integrations have
time to migrate.

## §16 Cross-Standard Linkage

Operators that consume adjacent WIA standards (cold-chain
monitoring, dangerous-goods compliance, supplier-quality-
management) emit cross-standard linkage records that name
the consuming standard and the version under which the
linkage is claimed.

## §17 Public Catalogue (Wholesale)

Wholesale operators that publish a public available-to-
promise catalogue (B2B-facing) emit a JSON Feed listing
items with their evidence-package manifest digests, the
public available quantity, and the operator's per-region
fulfilment expectations.

## §18 Reverse-Logistics Integration

Returns operations integrate with the operator's reverse-
logistics 3PL through the same gateway as forward 3PL §2,
with returns-specific event vocabulary
(`bizStep=returning`, `disposition=returned_for_credit`,
`disposition=destroyed`). Integration carries the returns
3PL's identifier and the per-class refurbishment SOP.

## §19 Customs and Trade Compliance Integration

Cross-border operators integrate with customs broker systems
for import / export declarations (US ACE, EU Trader Portal,
KCS UNI-PASS in Korea, JCS in Japan, equivalent systems
elsewhere). Integration carries the broker's identifier, the
per-shipment harmonized-tariff-schedule line items, and the
per-jurisdiction trade-agreement preference claim where
applicable.

## §20 Audit-Reviewer Workflow Integration

External auditors (GxP auditors for regulated pharmaceutical
and food operators, ISO 9001 / 22000 / 27001 surveillance
auditors, internal-audit functions) consume audit-trail
exports through dedicated client certificates. The export
carries the API audit logs, the EPCIS event log for the
audit window, the cycle-count and physical-inventory
records, and the controlled-substance reconciliation reports.

## §21 Reader Tooling for Operations Floor

Operators publish supplementary reader tools (real-time
dashboards for the warehouse floor, mobile picker apps,
zone-level cycle-count companion apps, recall trace-graph
visualisers for QA teams) alongside the canonical evidence
package. Reader tools are non-normative; the canonical
record remains the JSON evidence-package manifest.

## §22 Conformance and Sunset

A programme conformant with PHASE-4 has integrated
successfully with the operator's ERP, at least one 3PL
provider (where applicable), at least one trading-partner
channel, the operator's TMS (where applicable), the cold-
chain monitoring service (for cold-chain operators), the
controlled-substance regulator (for controlled-substance
operators), and at least one long-term archive, and has
published at least one externally citable evidence package.

Sunsetting an integration is announced via the well-known
discovery document at least 90 calendar days before
removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-inventory-management
- **Last Updated:** 2026-04-28
