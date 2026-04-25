# WIA-AGRI-014: Agricultural Supply Chain Standard
## Phase 2 — API Interface Specification

**Version:** 1.1.0
**Status:** ✅ Deep-Published
**Last Updated:** 2026-04-25
**Conforms to:** OpenAPI 3.1.0 (RFC 8259, RFC 7231), GS1 EPCIS 2.0 + CBV 1.2.2, HL7 FHIR R5 ProductShelfLife / SubstanceDefinition, Apache Iceberg v2 table format, ISO 22005:2007, EU 2023/1115 (EUDR), CFR Title 21 §1.1305 (FSMA Rule 204)

---

## 1. Scope and conformance

This Phase-2 specification defines the wire-level **HTTP/JSON contract** between Producers (farms, cooperatives), Aggregators (processors, sorting houses), Carriers (cold-chain logistics), Retailers, and Authorities (FDA, EFSA, MFDS, USDA APHIS) participating in a WIA-AGRI-014 traceability network. Two equally normative façades are defined:

1. **REST + OpenAPI 3.1** — synchronous CRUD over JSON resources (`/shipments`, `/cold-chain/...`, `/provenance/...`, `/quality-control/...`).
2. **GS1 EPCIS 2.0 REST API** — event-sourced façade emitting `ObjectEvent`, `AggregationEvent`, `TransformationEvent`, `TransactionEvent`, `AssociationEvent` records keyed by SGTIN/SSCC/GLN.

Both façades MUST surface the same underlying state. An implementation claiming `WIA-AGRI-014:2026 Gold` SHALL pass the conformance suite at https://conformance.wia-supply-chain.org against both façades.

### 1.1 Authority and base URLs

```
Production : https://api.wia-supply-chain.org/v1
EPCIS 2.0  : https://api.wia-supply-chain.org/epcis/v2
Sandbox    : https://staging-api.wia-supply-chain.org/v1
EU EUDR DDS: https://api.wia-supply-chain.org/eudr/v1
KR 농산물이력: https://api.wia-supply-chain.org/mafra/v1
```

### 1.2 Authentication

Three methods MUST be supported:

| Method                | Use case                      | RFC          |
|-----------------------|-------------------------------|--------------|
| OAuth 2.1 + PKCE      | Operator app, retailer portal | RFC 9700 BCP |
| mTLS (X.509)          | Carrier IoT gateway           | RFC 8446     |
| HTTP Message Signature| EPCIS event ingestion         | RFC 9421     |

```http
Authorization: Bearer eyJhbGciOiJFZERTQSIs...
Signature-Input: epcis=("@method" "@target-uri" "content-digest");created=1714032000;keyid="farm-kr-001";alg="ed25519"
Signature: epcis=:NN5BFJ7H...:
```

---

## 2. OpenAPI 3.1 contract (excerpt)

```yaml
openapi: 3.1.0
info:
  title: WIA Agricultural Supply Chain API
  version: 1.1.0
  license: { name: MIT, url: https://opensource.org/licenses/MIT }
servers:
  - { url: https://api.wia-supply-chain.org/v1 }
components:
  securitySchemes:
    bearerAuth: { type: http, scheme: bearer, bearerFormat: JWT }
  schemas:
    Shipment:
      $id: https://schemas.wia-supply-chain.org/Shipment.json
      type: object
      required: [shipmentId, productInfo, origin, destination, status]
      properties:
        shipmentId:    { type: string, pattern: '^SHIP-[0-9A-Z]{4,32}$' }
        sscc:          { type: string, pattern: '^[0-9]{18}$', description: 'GS1 SSCC (Application Identifier 00)' }
        productInfo:
          type: object
          required: [gtin, productName, quantity, unit, harvestDate]
          properties:
            gtin:        { type: string, pattern: '^[0-9]{14}$', description: 'GS1 GTIN-14' }
            productName: { type: string, minLength: 1, maxLength: 256 }
            quantity:    { type: number, exclusiveMinimum: 0 }
            unit:        { type: string, enum: [kg, g, lb, oz, L, mL, count, case, pallet] }
            harvestDate: { type: string, format: date }
            lotNumber:   { type: string, pattern: '^[0-9A-Z\-]{1,20}$' }
            organic:     { type: boolean }
            certifications:
              type: array
              items: { type: string, enum: [USDA_ORGANIC, EU_ORGANIC, GAP, GLOBALG_A_P, GFSI, JAS, KOREA_FRIENDLY] }
        origin:        { $ref: '#/components/schemas/Location' }
        destination:   { $ref: '#/components/schemas/Location' }
        coldChainRequired: { type: boolean, default: false }
        targetTemperature:
          type: object
          properties:
            min: { type: number }
            max: { type: number }
            unit: { type: string, enum: [C, F], default: C }
        status: { type: string, enum: [PENDING, AT_ORIGIN, IN_TRANSIT, AT_HUB, OUT_FOR_DELIVERY, DELIVERED, REJECTED, RECALLED] }
    Location:
      type: object
      required: [gln, latitude, longitude]
      properties:
        gln:       { type: string, pattern: '^[0-9]{13}$', description: 'GS1 GLN' }
        farmId:    { type: string }
        latitude:  { type: number, minimum: -90, maximum: 90 }
        longitude: { type: number, minimum: -180, maximum: 180 }
        altitudeM: { type: number }
        addr:      { type: string }
paths:
  /shipments:
    post:
      summary: Create shipment
      operationId: createShipment
      security: [{ bearerAuth: [] }]
      requestBody:
        required: true
        content:
          application/json:
            schema: { $ref: '#/components/schemas/Shipment' }
      responses:
        '201':
          description: Created
          headers:
            Location: { schema: { type: string, format: uri } }
            ETag:     { schema: { type: string } }
          content:
            application/json:
              schema: { $ref: '#/components/schemas/Shipment' }
        '409': { description: Conflict — shipmentId exists }
        '422': { description: Schema validation failed (RFC 9457 problem+json) }
```

The full contract is published at `https://schemas.wia-supply-chain.org/openapi-1.1.0.yaml` and is served with `application/vnd.oai.openapi+json;version=3.1` content negotiation.

### 2.1 Problem details (RFC 9457)

All error responses MUST conform to RFC 9457:

```http
HTTP/1.1 422 Unprocessable Entity
Content-Type: application/problem+json

{
  "type": "https://errors.wia-supply-chain.org/cold-chain-violation",
  "title": "Temperature out of target range",
  "status": 422,
  "detail": "Target 2-8°C, observed 11.4°C at 2026-04-25T14:31:08Z",
  "instance": "/shipments/SHIP-2026-001",
  "violations": [
    { "field": "currentTemperature", "code": "ABOVE_MAX", "value": 11.4, "max": 8 }
  ]
}
```

---

## 3. Shipment endpoints

### 3.1 `POST /shipments`

```bash
curl -X POST https://api.wia-supply-chain.org/v1/shipments \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -H "Idempotency-Key: 4f1d3c0e-7b2c-4a98-9c7e-2c0e9b8a4a01" \
  -d '{
    "shipmentId": "SHIP-2026-0421",
    "sscc": "356123450000000017",
    "productInfo": {
      "gtin": "08801234567896",
      "productName": "Jeju Organic Hallabong",
      "quantity": 480, "unit": "kg",
      "harvestDate": "2026-04-22",
      "lotNumber": "JJ-HBG-2026-W17",
      "organic": true,
      "certifications": ["EU_ORGANIC", "KOREA_FRIENDLY"]
    },
    "origin":      {"gln":"8801234500000","farmId":"FARM-KR-JJ-001","latitude":33.4996,"longitude":126.5312},
    "destination": {"gln":"8801234500113","facilityId":"RETAIL-SEL-021","latitude":37.5665,"longitude":126.9780},
    "coldChainRequired": true,
    "targetTemperature": {"min": 2, "max": 8, "unit": "C"}
  }'
```

`Idempotency-Key` (RFC draft-ietf-httpapi-idempotency-key) prevents duplicates on retried POSTs. Servers MUST cache the response for at least 24 h.

Response `201 Created` includes the resource representation, `Location` header, and `ETag` (strong validator over the canonical JSON). Subsequent updates MUST send `If-Match: <etag>` to enable optimistic concurrency control (RFC 9110 §13.1.1).

### 3.2 `GET /shipments/{id}` and conditional reads

Clients SHOULD send `If-None-Match` to leverage 304 Not Modified caching:

```http
GET /shipments/SHIP-2026-0421 HTTP/1.1
If-None-Match: "W/\"f3a2e1c\""
```

### 3.3 `PATCH /shipments/{id}` (RFC 7396 merge-patch)

```bash
curl -X PATCH https://api.wia-supply-chain.org/v1/shipments/SHIP-2026-0421 \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/merge-patch+json" \
  -H "If-Match: \"f3a2e1c\"" \
  -d '{ "status": "DELIVERED", "deliveredAt": "2026-04-23T17:42:11Z" }'
```

---

## 4. GS1 EPCIS 2.0 façade

### 4.1 Capture endpoint

EPCIS 2.0 (GS1 standard ratified Aug 2022) defines a JSON-LD wire format. `POST /epcis/v2/capture` accepts an `EPCISDocument`:

```json
{
  "@context": ["https://gs1.github.io/EPCIS/epcis-context.jsonld",
               { "wia": "https://schemas.wia-supply-chain.org/cbv/" }],
  "type": "EPCISDocument",
  "schemaVersion": "2.0",
  "creationDate": "2026-04-22T08:00:00.000Z",
  "epcisBody": {
    "eventList": [{
      "type": "ObjectEvent",
      "eventTime": "2026-04-22T08:00:00.000Z",
      "eventTimeZoneOffset": "+09:00",
      "epcList": [
        "urn:epc:id:sgtin:880123.4567896.JJHBG2026W17",
        "urn:epc:id:sscc:880123.40000000017"
      ],
      "action": "ADD",
      "bizStep": "https://ref.gs1.org/cbv/BizStep-commissioning",
      "disposition": "https://ref.gs1.org/cbv/Disp-active",
      "readPoint":  { "id": "urn:epc:id:sgln:880123.45000.0" },
      "bizLocation":{ "id": "urn:epc:id:sgln:880123.45000.0" },
      "wia:harvestMetadata": {
        "weather": "Sunny, 22°C",
        "harvester": "FARM-KR-JJ-001/CREW-A",
        "soilMoisture": 0.34
      }
    }]
  }
}
```

### 4.2 Query endpoint

```bash
curl -G https://api.wia-supply-chain.org/epcis/v2/queries/SimpleEventQuery/events \
  -H "Authorization: Bearer $TOKEN" \
  --data-urlencode "MATCH_anyEPC=urn:epc:id:sgtin:880123.4567896.*" \
  --data-urlencode "GE_eventTime=2026-04-01T00:00:00Z" \
  --data-urlencode "EQ_bizStep=https://ref.gs1.org/cbv/BizStep-shipping"
```

Returns a `QueryResults` JSON-LD document. Subscribers can register polling/webhook subscriptions per CBV 1.2.2 §8.

### 4.3 Event-type → use-case mapping

| EPCIS event type     | WIA-AGRI-014 use case                                   |
|----------------------|---------------------------------------------------------|
| `ObjectEvent`        | Harvest, packing, inspection, retail receipt            |
| `AggregationEvent`   | Pallet build (cases → pallet SSCC), pallet break-down   |
| `TransformationEvent`| Cleaning + grading + repacking (input lot → output lot) |
| `TransactionEvent`   | PO, despatch advice (DESADV), invoice link             |
| `AssociationEvent`   | Sensor → pallet binding (cold-chain logger lifecycle)   |

---

## 5. Cold-chain monitoring API

### 5.1 Sensor ingest

```bash
curl -X POST https://api.wia-supply-chain.org/v1/cold-chain/data \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "shipmentId":"SHIP-2026-0421",
    "deviceId":"BLE-LOG-77AA","deviceModel":"Sensitech TempTale Ultra",
    "timestamp":"2026-04-22T14:31:08Z",
    "temperatureC":4.2,"humidityPct":74,
    "gps":{"lat":36.34,"lon":127.40,"hdop":1.2},
    "battery":{"voltageMv":3015,"percent":78}
  }'
```

The server appends to a time-partitioned **Apache Iceberg v2** table (`agri.cold_chain.readings`) with a hidden partition transform `hours(timestamp)`, enabling sub-second range scans for the last 30 d window. The Iceberg snapshot id is returned in the `X-Snapshot-Id` response header so downstream analytics can do snapshot-isolated reads:

```http
HTTP/1.1 201 Created
X-Snapshot-Id: 7841029384712340987
ETag: "snap-7841029384712340987"
```

### 5.2 Aggregation query

```bash
curl -G https://api.wia-supply-chain.org/v1/cold-chain/history/SHIP-2026-0421 \
  -H "Authorization: Bearer $TOKEN" \
  --data-urlencode "interval=PT5M" \
  --data-urlencode "agg=avg,min,max,p95"
```

Response time-series is **CF-1.10 compliant** when negotiated as `application/vnd.unidata.netcdf+json`, allowing Iris/xarray ingestion for cold-chain ML model training.

### 5.3 Threshold alerts (CAP 1.2)

When a violation is detected, the server emits a CAP 1.2 (OASIS Common Alerting Protocol) message to subscribers:

```xml
<alert xmlns="urn:oasis:names:tc:emergency:cap:1.2">
  <identifier>WIA-AGRI-COLD-2026-04-22-0001</identifier>
  <sender>cold-chain@wia-supply-chain.org</sender>
  <sent>2026-04-22T14:33:00+09:00</sent>
  <status>Actual</status><msgType>Alert</msgType>
  <scope>Restricted</scope>
  <info>
    <category>Health</category><event>Cold-Chain Excursion</event>
    <urgency>Immediate</urgency><severity>Severe</severity><certainty>Observed</certainty>
    <eventCode><valueName>WIA-AGRI-014</valueName><value>COLD_CHAIN_VIOLATION</value></eventCode>
    <description>SHIP-2026-0421 exceeded 8°C target for 7 minutes</description>
  </info>
</alert>
```

---

## 6. Provenance and shelf-life APIs

### 6.1 Provenance event (HL7 FHIR R5 alignment)

`POST /provenance/events` accepts an FHIR R5 `Provenance` resource that **WIA-AGRI-014 profiles** with a Meat/Produce extension:

```json
{
  "resourceType": "Provenance",
  "meta": { "profile": ["https://wia-supply-chain.org/StructureDefinition/AgriProvenance"] },
  "target": [{ "reference": "Substance/JJ-HBG-2026-W17" }],
  "occurredDateTime": "2026-04-22T08:00:00+09:00",
  "recorded": "2026-04-22T08:00:03Z",
  "activity": {
    "coding": [{ "system": "https://wia-supply-chain.org/cs/agri-stage", "code": "harvest" }]
  },
  "agent": [{ "type": { "coding":[{"system":"http://terminology.hl7.org/CodeSystem/provenance-participant-type","code":"author"}]},
              "who": { "identifier": { "system":"https://wia-supply-chain.org/farm","value":"FARM-KR-JJ-001"}}}],
  "signature":[{
    "type":[{"system":"urn:iso-astm:E1762-95:2013","code":"1.2.840.10065.1.12.1.1"}],
    "when":"2026-04-22T08:00:03Z",
    "who":{"reference":"Organization/farm-kr-jj-001"},
    "sigFormat":"application/jose","data":"eyJhbGciOiJFZERTQSIs..."
  }]
}
```

### 6.2 Shelf-life publication

Per FHIR R5 `ProductShelfLife` resource attached to `SubstanceDefinition`:

```bash
curl -X PUT https://api.wia-supply-chain.org/v1/products/JJ-HBG-2026-W17/shelf-life \
  -H "Content-Type: application/fhir+json" \
  -d '{
    "resourceType":"ProductShelfLife",
    "type":{"coding":[{"system":"http://hl7.org/fhir/CodeSystem/product-shelf-life","code":"shelfLifeAfterPacking"}]},
    "periodDuration":{"value":18,"unit":"d","system":"http://unitsofmeasure.org","code":"d"},
    "specialPrecautionsForStorage":[{"text":"Maintain 2-8°C, 75-85% RH"}]
  }'
```

### 6.3 Verify (zero-knowledge friendly)

`GET /provenance/verify/{lot}?proof=zk-snark` returns a Groth16 proof bundle when the producer wants to publish provenance without disclosing supplier prices or buyer identity, while still proving organic certification, country of origin, and absence of disallowed pesticides.

---

## 7. Quality control + lab integration

```bash
curl -X POST https://api.wia-supply-chain.org/v1/quality-control/checks \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "shipmentId":"SHIP-2026-0421","stage":"PRE_SHIPMENT",
    "inspector":{"name":"이검수","license":"KCA-QC-2026-0177"},
    "tests":[
      {"loinc":"5778-6","label":"Pesticide residue panel","result":"NEGATIVE","method":"LC-MS/MS"},
      {"loinc":"42832-7","label":"Brix","value":11.8,"unit":"%","method":"Refractometer ATAGO PR-101α"}
    ],
    "haccp":{"ccp":"CCP-2","limitCelsius":7,"observed":4.2,"compliant":true},
    "passed":true,"grade":"GRADE_A"
  }'
```

LOINC codes (https://loinc.org) align lab payloads with hospital EHRs for outbreak investigations.

---

## 8. EUDR Due-Diligence Statement (EU 2023/1115)

For deforestation-linked commodities (palm, soy, beef, cocoa, coffee, rubber, wood) the API exposes:

```http
POST /eudr/v1/dds
Content-Type: application/json

{
  "operatorEori":"KR1234567890123",
  "commodity":"0901.21",        // CN code: roasted coffee
  "geolocations":[
    {"plot":"PLT-001","polygon":"POLYGON((-79.51 -1.23, -79.50 -1.23, -79.50 -1.22, -79.51 -1.22, -79.51 -1.23))","plotAreaHa":3.2}
  ],
  "harvestPeriod":{"from":"2026-03-01","to":"2026-04-15"},
  "deforestationRiskAssessment":{"basis":"GLAD-S2 alert overlay","risk":"NEGLIGIBLE"},
  "supplierStatement": {"@id":"did:wia:farm:co-aoa-014","signature":"eyJhbGciOiJFZERT..."}
}
```

Returns a **DDS reference number** that downstream EU operators reference in their own statements (cascade), satisfying EUDR Article 4(8).

---

## 9. Korean MAFRA / KCA integration

The 농산물 생산이력 (production history) and 농산물품질관리법 (Quality Control Act) require notification to **eaT** (e-Agricultural Trade) and **농산물이력추적관리시스템** (KCA). The shim:

```http
POST /mafra/v1/traceability/register
Content-Type: application/json

{
  "이력번호":"080123456789012",   // 13-digit ID issued by 농관원
  "생산자식별":"FARM-KR-JJ-001",
  "품목":"한라봉","수확일":"2026-04-22",
  "재배방법":"유기재배","친환경인증번호":"OG-9912-2026-0033"
}
```

Returns the upstream `MAFRA receipt id`, which is appended to the EPCIS event as `wia:mafraReceipt`.

---

## 10. Webhooks and async notifications

Webhooks use **CloudEvents 1.0** + **HTTP Message Signatures** (RFC 9421):

```http
POST /webhooks/your-endpoint HTTP/1.1
Host: hooks.example.com
Content-Type: application/cloudevents+json
Signature-Input: sig1=("@method" "@target-uri" "content-digest");created=1714032000;keyid="wia-supply-chain";alg="ed25519"

{
  "specversion":"1.0",
  "id":"evt-7b9...","source":"https://api.wia-supply-chain.org/v1",
  "type":"org.wia.supply-chain.cold_chain.violation",
  "subject":"SHIP-2026-0421","time":"2026-04-22T14:33:00Z",
  "datacontenttype":"application/json",
  "data":{"oldStatus":"NORMAL","newStatus":"VIOLATION","temperatureC":11.4,"durationSec":420}
}
```

Subscriber MAY return `204 No Content` on success or `410 Gone` to deregister.

---

## 11. Rate limits and SLOs

| Tier        | Sustained TPS | Burst | Cold-chain ingest | EPCIS capture |
|-------------|--------------:|------:|------------------:|---------------:|
| Standard    |           100 |   200 |             1 000 |             50 |
| Premium     |         1 000 | 2 000 |            10 000 |            500 |
| Enterprise  |        10 000 | 30 000|           100 000 |          5 000 |

`X-RateLimit-*` headers (RFC draft-ietf-httpapi-ratelimit-headers) on every response.

**Phase-2 SLO:** P95 read latency ≤ 80 ms intra-region, P95 write ≤ 250 ms with Iceberg snapshot commit, error budget 0.1 % monthly.

---

**Cross-reference:** [Phase 1: Data Format](PHASE-1-DATA-FORMAT.md) defines the JSON Schemas embedded above. [Phase 3: Protocol](PHASE-3-PROTOCOL.md) covers MQTT 5 / CoAP transport for the IoT side. [Phase 4: Integration](PHASE-4-INTEGRATION.md) closes the loop with ERP, WMS, POS, and authority systems.

---

**弘益人間 · Benefit All Humanity**
*WIA — World Industry Association · © 2026 MIT License*
