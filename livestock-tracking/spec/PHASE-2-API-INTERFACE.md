# WIA Livestock Tracking — Phase 2 API Interface Specification

**Standard:** WIA-AGRI-009
**Version:** 1.1.0
**Status:** ✅ Deep-Published
**Last updated:** 2026-04-25
**Conforms to:** OpenAPI 3.1.0, ICAR Animal Data Exchange (ADE) v1.7.4, ISO 11784/11785, ISO 14223, ISO 17532, GS1 EPCIS 2.0, HL7 FHIR R5 (animal-as-patient profile), EU Reg 2016/429 (Animal Health Law) + 2019/2035, USDA APHIS NAIS / AINS, AAA NLIS (Australia), Defra BCMS (UK), Korea 가축전염병예방법 + 축산물이력관리에관한법률, RFC 9457, RFC 9421, RFC 9449.

---

## 1. Scope and conformance

This specification defines the wire-level HTTP/JSON contract between farms, veterinary systems, slaughterhouses, retailers, and competent authorities (USDA APHIS, EU TRACES NT, MAFRA, Defra, AAA Integrity Systems) that participate in a WIA-AGRI-009 traceability network. Two equally normative façades are required:

1. **REST + OpenAPI 3.1** — synchronous CRUD over JSON resources (`/animals`, `/locations`, `/health`, `/events`, `/traceability`).
2. **GS1 EPCIS 2.0** — event-sourced façade (`ObjectEvent`, `AggregationEvent`, `TransactionEvent`, `TransformationEvent`) for cross-organisation interoperability.

A `WIA-AGRI-009:2026 Gold` implementation passes the suite at <code>https://conformance.livestock.wia.com</code> against both façades.

### 1.1 Base URLs

```
Production : https://api.livestock.wia.com/v1
Sandbox    : https://sandbox.livestock.wia.com/v1
EPCIS 2.0  : https://api.livestock.wia.com/epcis/v2
ICAR ADE   : https://api.livestock.wia.com/icar/ade/v1.7
EU TRACES  : https://api.livestock.wia.com/eu/traces/v1
KR MAFRA   : https://api.livestock.wia.com/mafra/v1
```

### 1.2 Authentication

| Method                  | RFC      | Use case                                  |
|-------------------------|----------|-------------------------------------------|
| OAuth 2.1 + PKCE + DPoP | RFC 9700, 9449 | Operator portal, vet desktop apps   |
| mTLS X.509              | RFC 8446 | Collar gateway, abattoir scanner          |
| HTTP Message Signature  | RFC 9421 | Cross-org EPCIS, regulator submission     |

```http
Authorization: DPoP eyJhbGciOiJFZERTQSIs...
DPoP: eyJ0eXAiOiJkcG9wK2p3dCIs...
Signature-Input: sig=("@method" "@target-uri" "content-digest");
                 created=1714032000;keyid="farm-kr-001";alg="ed25519"
```

---

## 2. OpenAPI 3.1 contract (excerpt)

```yaml
openapi: 3.1.0
info: { title: WIA Livestock Tracking API, version: 1.1.0 }
servers: [{ url: https://api.livestock.wia.com/v1 }]
components:
  securitySchemes:
    bearerAuth: { type: http, scheme: bearer, bearerFormat: JWT }
  schemas:
    Animal:
      $id: https://schemas.livestock.wia.com/Animal.json
      type: object
      required: [animalId, rfidTag, species, birthDate, holding]
      properties:
        animalId:    { type: string, pattern: '^[A-Z]{3}-[0-9A-Z]{2}-[0-9]{9,12}$' }
        rfidTag:     { type: string, pattern: '^[0-9]{3}-[0-9]{12}$',
                       description: 'ISO 11784 — country.national-id' }
        eid:         { type: string, pattern: '^[0-9]{15}$',
                       description: 'EU electronic identifier (15-digit)' }
        species:
          type: string
          enum: [CATTLE, SHEEP, GOAT, PIG, HORSE, DEER, BUFFALO, CAMELID, POULTRY, BEE_HIVE]
        breed:       { type: string }
        sex:         { type: string, enum: [FEMALE, MALE, CASTRATED_MALE, UNKNOWN] }
        birthDate:   { type: string, format: date }
        damId:       { type: string, description: 'Mother animal ID' }
        sireId:      { type: string, description: 'Father animal ID' }
        holding:
          type: object
          required: [holdingId, gln]
          properties:
            holdingId: { type: string, pattern: '^[A-Z]{3}-[0-9]{2}-[A-Z0-9]{6,12}$' }
            gln:       { type: string, pattern: '^[0-9]{13}$',
                         description: 'GS1 Global Location Number for the premises' }
        certifications:
          type: array
          items:
            type: string
            enum: [ORGANIC, GAP, GLOBALG_A_P, ANIMAL_WELFARE_RSPCA, KOREA_HACCP, EU_ORGANIC]
paths:
  /animals:
    post:
      operationId: registerAnimal
      security: [{ bearerAuth: [] }]
      parameters:
        - { in: header, name: Idempotency-Key, schema: { type: string }, required: true }
      requestBody:
        required: true
        content: { application/json: { schema: { $ref: '#/components/schemas/Animal' } } }
      responses:
        '201':
          headers:
            Location: { schema: { type: string, format: uri } }
            ETag:     { schema: { type: string } }
          content: { application/json: { schema: { $ref: '#/components/schemas/Animal' } } }
        '409': { description: Conflict — RFID/EID already registered }
        '422': { description: Validation failure (problem+json) }
```

---

## 3. Animal lifecycle endpoints

### 3.1 Register an animal

```bash
curl -X POST https://api.livestock.wia.com/v1/animals \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -H "Idempotency-Key: register-KOR-01-123456789" \
  -d '{
    "animalId":"KOR-01-123456789",
    "rfidTag":"410-000123456789",
    "eid":"410123456789012",
    "species":"CATTLE","breed":"HANWOO","sex":"FEMALE",
    "birthDate":"2026-01-15",
    "damId":"KOR-01-100200300",
    "holding":{"holdingId":"KOR-01-J-CHE-022","gln":"8801234500000"},
    "certifications":["KOREA_HACCP"]
  }'
```

Response `201 Created` includes the canonical resource, `Location`, strong `ETag`. The same call automatically:

- Pushes an EPCIS `ObjectEvent action=ADD bizStep=commissioning`.
- Mirrors to **MAFRA 축산물이력제** via `POST /mafra/v1/livestock/register` with the issued 12-digit 이력번호.
- For EU-registered holdings, mirrors to **TRACES NT** with the EID.

### 3.2 Read with conditional GET

```http
GET /v1/animals/KOR-01-123456789
If-None-Match: "v3:f3a2e1c"
```

Returns `200 OK` with full body or `304 Not Modified` (RFC 7234).

### 3.3 Patch (concurrency-safe)

```http
PATCH /v1/animals/KOR-01-123456789
Content-Type: application/merge-patch+json
If-Match: "v3:f3a2e1c"

{ "currentWeightKg": 510.4, "bodyConditionScore": 3.5 }
```

### 3.4 List with filters

```http
GET /v1/animals?holdingId=KOR-01-J-CHE-022&status=ACTIVE&sortBy=birthDate&page=1&limit=50
```

---

## 4. Location and movement

### 4.1 Submit location

```bash
curl -X POST https://api.livestock.wia.com/v1/animals/KOR-01-123456789/location \
  -H "Content-Type: application/json" \
  -d '{
    "lat":33.4996,"lng":126.5312,"accuracyM":2.5,
    "deviceId":"DEV-COLLAR-7AA","deviceModel":"Allflex SenseHub",
    "timestamp":"2026-04-25T14:22:00+09:00",
    "activity":"GRAZING",
    "battery":{"voltageMv":3015,"percent":78}
  }'
```

The server appends to a time-partitioned Iceberg v2 table (`livestock.locations`); the snapshot id is returned in `X-Snapshot-Id`.

### 4.2 Movement event (between holdings)

Movement is an EPCIS `ObjectEvent action=OBSERVE bizStep=arriving` plus a regulatory notification:

```http
POST /v1/animals/KOR-01-123456789/movements
{
  "from":{"holdingId":"KOR-01-J-CHE-022"},
  "to":{"holdingId":"KOR-02-G-ANS-018"},
  "departureAt":"2026-04-25T08:00:00+09:00",
  "arrivalAt":"2026-04-25T13:30:00+09:00",
  "transportVehicle":{"plate":"31허8841","glnCarrier":"8804567000003"},
  "regulatoryFlags":["INTRA_KR","HACCP_LIVESTOCK_CARRIER"]
}
```

For EU intra-community movement the regulator endpoint is `POST /eu/traces/v1/movements` (TRACES NT).

---

## 5. Health, vaccination, and welfare

### 5.1 Vaccination (LOINC + ATC-vet aligned)

```bash
curl -X POST https://api.livestock.wia.com/v1/animals/KOR-01-123456789/health \
  -H "Content-Type: application/json" \
  -d '{
    "type":"VACCINATION",
    "date":"2026-04-25",
    "veterinarian":{"licenseNumber":"KVMA-2026-0177","name":"이수의"},
    "vaccine":{"loinc":"30958-7","label":"Foot-and-mouth disease vaccine, type O+A",
               "manufacturer":"Merial","batch":"FMD-2026-Q1-A33","doseMl":2.0,
               "atcvet":"QI02AA01"},
    "route":"INTRAMUSCULAR",
    "withdrawal":{"meatDays":21,"milkDays":0}
  }'
```

### 5.2 Disease event

```bash
curl -X POST https://api.livestock.wia.com/v1/animals/KOR-01-123456789/health \
  -d '{
    "type":"DIAGNOSIS",
    "date":"2026-04-25",
    "diagnosis":{
      "icd10vet":"A50.0","label":"Foot-and-mouth disease, suspected",
      "snomedct":"419218000",
      "severity":"MODERATE","status":"SUSPECTED"
    },
    "notification":{"oie":"OIE-WAHIS-2026-KR-FMD-0001","domestic":"MAFRA-2026-04-25-0033"}
  }'
```

Notifiable diseases per OIE-WOAH terrestrial code automatically trigger `notification` to the competent authority (MAFRA in Korea, EFSA + national CA in the EU, USDA APHIS in the US).

---

## 6. ICAR ADE 1.7 façade

The International Committee for Animal Recording (ICAR) publishes the Animal Data Exchange standard (ADE) for genealogy, productivity, and lactation data exchange.

```bash
curl -G https://api.livestock.wia.com/icar/ade/v1.7/animals/KOR-01-123456789/lactation \
  --data-urlencode "from=2026-01-01" --data-urlencode "to=2026-04-30"
```

Response is an ICAR ADE JSON document conforming to <code>https://github.com/adewg/ICAR/blob/master/Resources/Documents/IcarLactationResource.md</code> — directly consumable by Lely, DeLaval, and CRV systems.

---

## 7. EPCIS 2.0 façade

```http
POST /epcis/v2/capture
Content-Type: application/json+ld

{
  "@context":["https://gs1.github.io/EPCIS/epcis-context.jsonld"],
  "type":"EPCISDocument","schemaVersion":"2.0",
  "creationDate":"2026-04-25T14:22:00Z",
  "epcisBody":{"eventList":[{
    "type":"ObjectEvent","eventTime":"2026-04-25T14:22:00Z",
    "eventTimeZoneOffset":"+09:00",
    "epcList":["urn:icar:animal:kor.01.123456789"],
    "action":"OBSERVE",
    "bizStep":"https://ref.gs1.org/cbv/BizStep-arriving",
    "disposition":"https://ref.gs1.org/cbv/Disp-active",
    "readPoint":{"id":"urn:epc:id:sgln:880456.70000.0"},
    "wia:transportVehicle":"31허8841"
  }]}
}
```

---

## 8. Traceability

```bash
curl https://api.livestock.wia.com/v1/animals/KOR-01-123456789/traceability \
  -H "Authorization: Bearer $TOKEN"
```

```json
{
  "animalId":"KOR-01-123456789",
  "trail":[
    {"event":"BIRTH","at":"2026-01-15","holding":"KOR-01-J-CHE-022"},
    {"event":"VACCINATION-FMD","at":"2026-04-25","veterinarian":"KVMA-2026-0177"},
    {"event":"MOVEMENT","at":"2026-04-25","from":"KOR-01-J-CHE-022","to":"KOR-02-G-ANS-018"}
  ],
  "blockchain":{
    "network":"WIA-CHAIN-PoA","anchorTx":"0xab12...","bsc":"0xcd34...",
    "merkleRoot":"sha256:2d48...","verified":true
  }
}
```

For consumers, `GET /verify/{eid}` returns a public-friendly page with farm, vaccination summary, and a QR-code-resolvable signed credential.

---

## 9. Webhooks (CloudEvents 1.0 + RFC 9421)

```http
POST /webhooks/your-endpoint
Content-Type: application/cloudevents+json
Signature-Input: sig1=("@method" "@target-uri" "content-digest");
                 created=1714032000;keyid="wia-livestock";alg="ed25519"

{"specversion":"1.0","id":"evt-7b9","source":"https://api.livestock.wia.com/v1",
 "type":"com.wia.livestock.geofence.exit","subject":"KOR-01-123456789",
 "time":"2026-04-25T14:22:00Z","datacontenttype":"application/json",
 "data":{"holdingId":"KOR-01-J-CHE-022","lastInside":"2026-04-25T14:18:00Z"}}
```

Required event types: `geofence.exit`, `health.alert`, `disease.notifiable`, `movement.arrived`, `vaccination.due`, `withdrawal.expired`.

---

## 10. Errors — RFC 9457 problem+json

```http
HTTP/1.1 422 Unprocessable Entity
Content-Type: application/problem+json

{
  "type":"https://errors.livestock.wia.com/withdrawal-violation",
  "title":"Slaughter inside withdrawal period",
  "status":422,
  "detail":"FMD vaccine administered 2026-04-25 — meat withdrawal until 2026-05-16",
  "instance":"/v1/animals/KOR-01-123456789/slaughter",
  "violations":[{"check":"meatWithdrawalDays","required":21,"observed":3}]
}
```

---

## 11. Rate limits and SLOs

| Tier        | Sustained TPS | Locations/s | P95 read | P95 write |
|-------------|--------------:|------------:|---------:|----------:|
| Standard    |           100 |       2 000 |    80 ms |    250 ms |
| Premium     |         1 000 |      20 000 |    50 ms |    180 ms |
| Enterprise  |        10 000 |     200 000 |    30 ms |    120 ms |

`X-RateLimit-*` headers per draft-ietf-httpapi-ratelimit-headers. P95 SLO 30/50/80 ms read; error budget 0.1% monthly.

---

**弘益人間 — Benefit All Humanity**
*WIA-AGRI-009 · © 2026 MIT License*
