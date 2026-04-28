# WIA-blockchain-intro PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-blockchain-intro
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the API contract that a
blockchain-intro operator (learning-platform, pilot
deployment, intro-service-provider, educational
institution) exposes for the records defined in
PHASE-1. The contract is consumed by the developer /
student audience, the operator's compliance-and-
risk function, the supervisory authority's
examination tooling, and the W3C VC / DID
ecosystem partners (issuer / holder / verifier
trio).

References (CITATION-POLICY ALLOW only):

- W3C VC 2.0 + DID Core 1.0 + VC API 0.7 (work-
  in-progress)
- IETF RFC 9110 (HTTP Semantics), RFC 9111 (HTTP
  Caching), RFC 9457 (Problem Details), RFC 6901 /
  6902 (JSON Pointer / Patch), RFC 8288 (Web
  Linking), RFC 8259 (JSON), RFC 9421 (HTTP Message
  Signatures)
- IETF RFC 7515 / 7516 / 7517 / 7519 / 7638 / 9162
- IETF RFC 8949 (CBOR), RFC 8152 (COSE), RFC 9381
  (VRF)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022, ISO/TC 307 deliverables
- W3C Trace Context

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by
the operator. Versioning uses `/v1/` path segments.
The OpenAPI 3.1 document at `/v1/openapi.json` is
canonical.

The operator exposes:

- The HTTPS / JSON RESTful surface for the
  programme, platform, identity, contract, wallet,
  and custody records.
- The W3C VC API surface for credential issuance,
  presentation, and verification.
- The on-chain RPC adapter surface that proxies
  to the underlying DLT platform (Ethereum JSON-RPC
  for EVM-class networks; per-platform RPC for
  others).

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-blockchain-intro",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":              "/v1/programmes",
    "dltPlatforms":            "/v1/dlt-platforms",
    "consensusGovernance":     "/v1/consensus-governance",
    "cryptoPrimitives":        "/v1/crypto-primitives",
    "identities":              "/v1/identities",
    "verifiableCredentials":   "/v1/vc",
    "smartContracts":          "/v1/smart-contracts",
    "wallets":                 "/v1/wallets",
    "custodyEvents":           "/v1/custody-events",
    "regulatoryClassifications": "/v1/regulatory-classifications",
    "rpcAdapter":              "/rpc/",
    "examination":             "/v1/examination",
    "openapi":                 "/v1/openapi.json"
  }
}
```

## §3 W3C VC API Surface

```
POST   /v1/vc/issue                  (issue a VC
                                      from an
                                      issuer DID)
POST   /v1/vc/verify                 (verify a VC
                                      against the
                                      issuer's
                                      DID
                                      document)
POST   /v1/vc/present                (create a
                                      Verifiable
                                      Presentation)
POST   /v1/vc/verify-presentation
GET    /v1/vc/{vcId}/status          (consult the
                                      VC Bitstring
                                      Status List
                                      entry)
PATCH  /v1/vc/{vcId}/revoke
```

## §4 DID Resolution Surface

```
GET    /1.0/identifiers/{did}         (W3C DID
                                       resolver
                                       per W3C DID
                                       Resolution
                                       1.0)
GET    /v1/identities                 (operator-
                                       managed DIDs)
POST   /v1/identities                 (create a
                                       new managed
                                       DID)
```

## §5 Smart-Contract Endpoints

```
GET    /v1/smart-contracts
GET    /v1/smart-contracts/{contractId}
POST   /v1/smart-contracts            (register a
                                       deployed
                                       contract)
GET    /v1/smart-contracts/{contractId}/audit-reports
PATCH  /v1/smart-contracts/{contractId}/upgrade
       (record a contract upgrade event for the
        upgrade-pattern declared in PHASE-1 §7)
```

## §6 Wallet and Custody Endpoints

```
GET    /v1/wallets
POST   /v1/wallets                   (register a
                                      wallet)
GET    /v1/wallets/{walletId}
GET    /v1/custody-events?address={address}&from={iso}&to={iso}
POST   /v1/custody-events            (record a
                                      transaction
                                      custody event
                                      observed
                                      on-chain)
```

## §7 Regulatory-Classification Endpoints

```
GET    /v1/regulatory-classifications
POST   /v1/regulatory-classifications
GET    /v1/regulatory-classifications/{classificationId}
GET    /v1/regulatory-classifications/per-token/{tokenAddress}
```

## §8 RPC Adapter Surface

For EVM-class networks the RPC adapter exposes the
Ethereum JSON-RPC method set:

```
POST   /rpc/eth/{networkChainId}    (Ethereum JSON-
                                     RPC — eth_call,
                                     eth_sendRawTransaction,
                                     eth_getTransactionReceipt,
                                     eth_getLogs,
                                     eth_blockNumber,
                                     eth_getBalance,
                                     eth_chainId,
                                     eth_estimateGas)
```

For non-EVM networks the per-platform RPC method
set is exposed (Bitcoin Core JSON-RPC, Cosmos SDK
gRPC + REST, Polkadot JSON-RPC, Hyperledger Fabric
gRPC, Corda RPC).

## §9 Examination Endpoints

```
GET    /v1/examination/programmes
GET    /v1/examination/regulatory-classifications
GET    /v1/examination/custody-events
GET    /v1/examination/wallets
GET    /v1/examination/audit-events
```

The examination scope is read-only and bound to
the authority's identity (US SEC + CFTC + FinCEN +
state regulators; EU EBA + ESMA + Member-State
NCAs; KR FSC + FSS + FIU).

## §10 Authentication, HTTP Status, and Caching

Bearer tokens conform to OAuth 2.1; per-surface
audiences distinguish learner, operator, and
examination scopes. Standard HTTP status codes
apply. The RPC adapter forwards the underlying
node's responses transparently.

## §11 Webhook and Event Surface

Lifecycle events:

- `vc.issued`, `vc.verified`, `vc.revoked`
- `did.created`, `did.deactivated`
- `smart-contract.deployed`,
  `smart-contract.upgraded`,
  `smart-contract.audit-completed`
- `custody-event.observed`
- `regulatory-classification.assigned`,
  `regulatory-classification.amended`

Webhook signatures use HTTP Message Signatures
(RFC 9421).

## §12 Bulk-Export Surface

```
POST   /v1/bulk-export
GET    /v1/bulk-export/{exportId}/manifest
GET    /v1/examination/audit-events.csv
```

## §13 Travel-Rule Messaging Surface

For VASP operators:

```
POST   /v1/travel-rule/outbound-message    (originate
                                            an IVMS
                                            101 +
                                            Travel-
                                            Rule
                                            payload)
POST   /v1/travel-rule/inbound-message     (receive
                                            an IVMS
                                            101 +
                                            Travel-
                                            Rule
                                            payload)
GET    /v1/travel-rule/counterparty-vasps  (the
                                            registry
                                            of
                                            approved
                                            VASPs)
```

The Travel-Rule messages follow IVMS 101 in the
canonical wire format; technical-stack adoption
(TRP / TRISA / Sygna Bridge / OpenVASP) is declared
in the operator's PHASE-1 §2 governing-frameworks
list.

## §14 Educational and Assessment Surface

For educational operators:

```
GET    /portal/v1/curriculum
GET    /portal/v1/courses
POST   /portal/v1/assessments/submit
GET    /portal/v1/credentials/me            (student's
                                             VC
                                             wallet)
POST   /portal/v1/credentials/issue         (course-
                                             completion
                                             VC
                                             issuance)
```

The assessment-and-credential endpoints integrate
the Open Badges 3.0 + W3C VC 2.0 ecosystem; the
student's earned credentials are portable across
institutions through the W3C DID / VC stack.

## §15 Examination Surface

```
GET    /v1/examination/programmes
GET    /v1/examination/regulatory-classifications
GET    /v1/examination/custody-events.csv
GET    /v1/examination/travel-rule-messages
GET    /v1/examination/wallet-attestations
GET    /v1/examination/audit-events.csv
```

The examination scope serves the supervisory
authority's data calls — US SEC / CFTC / FinCEN +
state regulators; EU EBA / ESMA / Member-State NCA;
KR FSC / FSS / FIU. CSV downloads support the
authority's trend analysis.

## §16 Conformance

Implementations claiming PHASE-2 conformance publish
the OpenAPI document, expose the W3C VC API +
DID Resolution surfaces, expose the supervisory
examination surface, expose the RPC adapter for the
DLT platform, and propagate trace-context across
the issuance / verification / on-chain interaction
chain.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-blockchain-intro
- **Last Updated:** 2026-04-28
