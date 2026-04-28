# WIA-AUTO-016 — Phase 4: Integration

> Smart-logistics canonical Phase 4: ecosystem integration (EDI + webhooks + carrier adapters).

# WIA-AUTO-016: Smart Logistics Specification v1.0

> **Standard ID:** WIA-AUTO-016
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive & Logistics Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Warehouse Automation](#2-warehouse-automation)
3. [Fleet Management](#3-fleet-management)
4. [Route Optimization Algorithms](#4-route-optimization-algorithms)
5. [Real-time Tracking](#5-real-time-tracking)
6. [Last-mile Delivery](#6-last-mile-delivery)
7. [Inventory Management](#7-inventory-management)
8. [Data Formats](#8-data-formats)
9. [API Interface](#9-api-interface)
10. [Integration Protocols](#10-integration-protocols)
11. [References](#11-references)

---


## 10. Integration Protocols

### 10.1 EDI (Electronic Data Interchange)

#### 10.1.1 Common EDI Transactions

**EDI 940 (Warehouse Shipping Order)**:
```
ISA*00*          *00*          *ZZ*SENDER         *ZZ*RECEIVER       *250126*1000*U*00401*000000001*0*P*>
GS*SW*SENDER*RECEIVER*20250126*1000*1*X*004010
ST*940*0001
W05*N*WH-SF-001*ORD-123456*20250126
N1*ST*CUSTOMER NAME
N3*456 RESIDENTIAL ST
N4*OAKLAND*CA*94601
W01*1*PROD-123*2*EA
W01*2*PROD-456*1*EA
SE*8*0001
GE*1*1
IEA*1*000000001
```

**EDI 856 (Advanced Ship Notice)**:
```
ISA*00*          *00*          *ZZ*SENDER         *ZZ*RECEIVER       *250126*1030*U*00401*000000002*0*P*>
GS*SH*SENDER*RECEIVER*20250126*1030*2*X*004010
ST*856*0002
BSN*00*WIA-SHIP-20250126-001*20250126*1030
HL*1**S
TD5****GROUND
REF*BM*1Z999AA10123456784
DTM*011*20250127
N1*ST*JOHN DOE
N3*456 RESIDENTIAL ST
N4*OAKLAND*CA*94601
HL*2*1*O
PRF*ORD-123456
HL*3*2*I
LIN**SK*PROD-123
SN1**2*EA
SE*16*0002
GE*1*2
IEA*1*000000002
```

### 10.2 API Integration Patterns

#### 10.2.1 Webhook Notifications

```
POST https://customer.example.com/webhook/shipment
Content-Type: application/json
X-WIA-Signature: sha256=abcdef123456...

{
  "event": "shipment.delivered",
  "timestamp": "2025-01-27T16:05:00Z",
  "data": {
    "shipmentId": "WIA-SHIP-20250126-001",
    "trackingNumber": "1Z999AA10123456784",
    "status": "delivered",
    "deliveryTime": "2025-01-27T16:05:00Z",
    "signedBy": "John Doe",
    "pod": {
      "signature": "https://api.wia.com/pod/signatures/xxx",
      "photo": "https://api.wia.com/pod/photos/xxx"
    }
  }
}
```

**Signature Verification**:
```python
import hmac
import hashlib

def verify_webhook(payload, signature, secret):
    expected = hmac.new(
        secret.encode(),
        payload.encode(),
        hashlib.sha256
    ).hexdigest()

    return hmac.compare_digest(f"sha256={expected}", signature)
```

#### 10.2.2 Rate Limiting

```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 995
X-RateLimit-Reset: 1643299200

Algorithm: Token Bucket
- Bucket capacity: 1000 requests
- Refill rate: 1000 tokens per hour
- Burst allowed: Up to bucket capacity
```

### 10.3 Third-Party Carrier Integration

#### 10.3.1 Carrier API Mapping

**Unified Tracking Interface**:
```python
class CarrierAdapter:
    def get_tracking(self, tracking_number):
        # Map to carrier-specific API
        carrier = identify_carrier(tracking_number)

        if carrier == 'UPS':
            return self.ups_tracking(tracking_number)
        elif carrier == 'FedEx':
            return self.fedex_tracking(tracking_number)
        elif carrier == 'USPS':
            return self.usps_tracking(tracking_number)

        # Normalize response to WIA format
        return normalize_tracking_data(raw_data)

    def normalize_tracking_data(self, raw_data):
        # Convert to standard WIA format
        return {
            'trackingNumber': raw_data.get('tracking_id'),
            'status': map_status(raw_data.get('status')),
            'location': parse_location(raw_data.get('location')),
            'estimatedDelivery': parse_date(raw_data.get('eta')),
            'events': [normalize_event(e) for e in raw_data.get('events', [])]
        }
```

**Status Code Mapping**:
```python
STATUS_MAP = {
    # UPS codes
    'I': 'label_created',
    'P': 'picked_up',
    'M': 'in_transit',
    'X': 'out_for_delivery',
    'D': 'delivered',

    # FedEx codes
    'OC': 'label_created',
    'PU': 'picked_up',
    'IT': 'in_transit',
    'OFD': 'out_for_delivery',
    'DL': 'delivered',

    # USPS codes
    'Pre-Shipment': 'label_created',
    'Accepted': 'picked_up',
    'In Transit': 'in_transit',
    'Out for Delivery': 'out_for_delivery',
    'Delivered': 'delivered'
}
```

---




---

## A.1 EDI integration (940 / 856 / 850)

Smart-logistics integrates with EDI 940 (Warehouse Shipping Order), EDI 856 (Advanced Ship Notice), and EDI 850 (Purchase Order) via an adapter container. The adapter normalizes the EDI fields to the WIA shipment / route / warehouse-operation envelopes (Phase 1 §A.4) so consumers can ignore the EDI wire format. Outbound direction reverses the mapping. ANSI X12 4010 is the floor; partners that require 5010 or higher receive a configuration override.

## A.2 Webhook envelope and signature

Webhook deliveries carry an `X-WIA-Signature: sha256=...` header computed as `HMAC-SHA256(secret, raw_body)`. Receivers MUST verify the signature using a constant-time compare; receivers MUST also dedupe by `deliveryId` to defend against retries. Retry policy is 3 attempts with exponential backoff (1s, 4s, 16s). After the third failure the delivery enters the dead-letter queue and operations is paged.

## A.3 Carrier adapter (UPS / FedEx / USPS)

The carrier-adapter layer maps third-party tracking APIs onto the WIA shipment envelope. UPS, FedEx, and USPS adapters ship in the reference container; additional carriers register via the adapter registry. Status-code mapping is published at `https://wiastandards.com/smart-logistics/status-map.json` and is versioned. The adapter handles backoff on 429s and quotes the carrier's own SLA in the response header.

## A.4 TMS / WMS bridges

The TMS bridge maps onto Oracle TMS, SAP TM, and BluJay; the WMS bridge maps onto Manhattan WMS, SAP EWM, and Blue Yonder WMS. Bridge envelopes carry a `tenantId`, the source-system record id, and a checksum; replay attempts are detected and rejected. Bridges are unidirectional unless explicitly configured otherwise.

## A.5 Compliance cross-walk

| Concern              | Standard / Regulation                                        |
|----------------------|--------------------------------------------------------------|
| Quality management   | ISO 9001:2015                                                |
| Environmental        | ISO 14001:2015                                               |
| Occupational safety  | ISO 45001:2018                                               |
| Supply chain identifiers | GS1 GTIN, GS1 SSCC                                       |
| Carrier safety (US)  | FMCSA hours-of-service, DOT carrier registration             |
| Air-quality (CA)     | CARB advanced clean fleet rule                               |
| Privacy (EU)         | EU GDPR + national transpositions                            |
| Privacy (US)         | CCPA / CPRA + state equivalents                              |

This cross-walk is informative; implementers obtain authoritative copies from the issuing body and maintain their own attestation evidence.

## A.6 Reference container and CLI

The reference container at `wia/smart-logistics-host:1.0.0` exposes every Phase 2 endpoint with mock data and feeds the conformance suite. The companion CLI at `cli/smart-logistics.sh` ships sample envelope generators for shipments, routes, pick lists, and tracking events. WIA Standards composition: WIA-INTENT for workload intent, WIA-OMNI-API for credential storage, WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for federation handshake.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/smart-logistics/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-smart-logistics-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/smart-logistics-host:1.0.0` ships every smart-logistics envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/smart-logistics.sh` ships sample envelope generators with no
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
ecosystem. Smart-logistics deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
