# WIA-open-banking PHASE 2 — API Interface Specification

**Standard:** WIA-open-banking
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surface an ASPSP exposes for TPP
registration, consent setup, account-information access, payment
initiation, funds confirmation, SCA invocation, and notification
delivery. The shape is FAPI 2.0-secured OAuth 2.1 + OIDC for the
authorisation plane, with HTTP/JSON resources for the data plane.

References (CITATION-POLICY ALLOW only):
- OpenID Foundation FAPI 2.0 (Baseline + Advanced Profile)
- OpenID Connect Core 1.0
- IETF OAuth 2.0 (RFC 6749), OAuth 2.1 draft, RFC 7636 (PKCE),
  RFC 9101 (JAR), RFC 9126 (PAR), RFC 8705 (mTLS, certificate-bound tokens)
- IETF RFC 9457 (Problem Details)
- OBIE Read/Write API Specification v3.x — endpoint shapes for UK
- Berlin Group NextGenPSD2 — endpoint shapes for EU
- KFSC MyData Standard API — endpoint shapes for KR
- ISO 20022 — payment-initiation message profiles

---

## §1 TPP registration

PSD2 deployments use Dynamic Client Registration (DCR) with TPP
software statements per OBIE / Berlin Group:

```
POST /register HTTP/1.1
Content-Type: application/jwt

<software statement JWT signed by the OB Directory>
```

The boundary verifies the software statement signature against the
OB Directory's published JWKS (or the equivalent national directory
for KR/JP/AU), validates the TPP's licence status with the
issuing regulator, and on success issues OAuth client credentials.
Subsequent re-registration on licence change uses the same endpoint
with an updated software statement.

## §2 Consent setup (PAR + RAR)

Consent setup uses Pushed Authorization Requests (PAR, RFC 9126)
plus Rich Authorization Requests (RAR) to convey the consent's
detailed scope:

```
POST /authorize/par HTTP/1.1
Content-Type: application/x-www-form-urlencoded

client_id=<tpp>&request=<JAR-signed-JWT>
```

The JAR JWT carries:

```json
{
  "iss": "<tppId>",
  "aud": "<aspsp-authorize-base>",
  "client_id": "<tpp>",
  "redirect_uri": "<allowlisted>",
  "code_challenge": "<PKCE>",
  "code_challenge_method": "S256",
  "authorization_details": [{
    "type": "payment_initiation",
    "instructedAmount": {"currency": "GBP", "amount": "100.00"},
    "creditorAccount": {...},
    "creditorName": "..."
  }]
}
```

PAR returns a `request_uri` that the TPP redirects the user to. The
user authenticates via SCA (PHASE 3 §1) and the boundary creates the
consent record (PHASE 1 §3).

## §3 Token exchange

Tokens follow OAuth 2.0 authorisation_code with PKCE + mTLS-bound:

```
POST /token HTTP/1.1
Content-Type: application/x-www-form-urlencoded
Cert-Subject: <TPP mTLS cert subject>

grant_type=authorization_code&code=<code>&code_verifier=<PKCE>&client_id=<tpp>
```

Issued access tokens are mTLS-bound per RFC 8705; the TPP MUST
present the same client certificate on subsequent API calls. A token
presented over a different mTLS session is rejected with
`urn:wia:ob:problem:token-not-bound`.

## §4 Account information

```
GET /accounts HTTP/1.1
Authorization: Bearer <mtls-bound-token>
x-fapi-interaction-id: <UUID>
x-fapi-auth-date: <RFC 7231 date>
x-fapi-customer-ip-address: <IP>
```

FAPI 2.0 mandatory headers (`x-fapi-interaction-id`,
`x-fapi-auth-date`, `x-fapi-customer-ip-address`,
`x-fapi-customer-last-logged-time`) are included on every call so
the boundary can correlate TPP-side activity with the ASPSP's audit
chain. Resource endpoints follow the OBIE / Berlin Group / KFSC
shape per the deployment's regime.

## §5 Payment initiation

```
POST /payment-initiations HTTP/1.1
Authorization: Bearer <mtls-bound-token>
x-fapi-interaction-id: <UUID>
x-jws-signature: <JWS detached over body>
Content-Type: application/json

{
  "Data": {
    "ConsentId": "<consentId>",
    "Initiation": {
      "InstructionIdentification": "...",
      "EndToEndIdentification": "E2E-91a7",
      "InstructedAmount": {"Amount": "100.00", "Currency": "GBP"},
      "DebtorAccount": {...},
      "CreditorAccount": {...},
      "CreditorAgent": {...},
      "RemittanceInformation": {"Reference": "..."}
    }
  },
  "Risk": {
    "PaymentContextCode": "EcommerceGoods",
    "MerchantCategoryCode": "5411",
    "MerchantCustomerIdentification": "...",
    "DeliveryAddress": {...}
  }
}
```

The boundary validates the consent reference, the JWS detached
signature, applies the deployment's fraud / sanctions screening,
forwards to clearing (WIA-payment-system), and returns the
resulting UETR back to the PISP.

## §6 Funds confirmation

```
POST /funds-confirmations HTTP/1.1
{
  "Data": {
    "ConsentId": "<cofConsentId>",
    "Reference": "...",
    "InstructedAmount": {"Amount": "100.00", "Currency": "GBP"}
  }
}
```

Response carries `FundsAvailableResult: { FundsAvailable: true|false }`.
PSD2 RTS Article 36(1)(c) limits this to a yes/no answer; the
ASPSP MUST NOT return the actual balance.

## §7 SCA invocation (decoupled and embedded)

For decoupled SCA, the ASPSP starts an authentication flow on a
separate channel (mobile banking app push notification, hardware
token):

```
POST /authorize/decoupled-sca HTTP/1.1
{
  "consentRef": "...",
  "scaChannel": "mobile-app-push"
}
```

The boundary returns a poll URI; the TPP polls until the SCA
completes or the user declines. For embedded SCA (less common in
PSD2 / FAPI 2.0 because of redirect-based flows being preferred),
the SCA UI is rendered inline with the consent setup.

## §8 Notification webhook

ASPSPs deliver asynchronous notifications to the TPP's registered
webhook:

```
POST <tpp-webhook-uri> HTTP/1.1
x-jws-signature: <JWS detached>
Content-Type: application/json

{
  "notificationId": "...",
  "notificationType": "consent-revoked",
  "subjectRef": "<consentId>",
  "notificationTimestamp": "..."
}
```

The TPP MUST acknowledge with `200 OK`. Unacknowledged notifications
retry per the deployment's retry budget; persistent failure
suspends notification delivery and surfaces an incident.

## §9 Errors and warnings

| URI                                              | Status | Meaning                                      |
|--------------------------------------------------|-------:|----------------------------------------------|
| `urn:wia:ob:problem:tpp-licence-suspended`       | 403    | TPP licence is suspended/revoked              |
| `urn:wia:ob:problem:consent-not-authorised`      | 403    | consent was never SCA-authorised              |
| `urn:wia:ob:problem:consent-expired`             | 403    | consent past expirationDateTime               |
| `urn:wia:ob:problem:permission-not-granted`      | 403    | call requires a permission outside consent    |
| `urn:wia:ob:problem:token-not-bound`             | 403    | bearer token presented on different mTLS session |
| `urn:wia:ob:problem:sca-required`                | 401    | call requires fresh SCA evidence              |
| `urn:wia:ob:problem:fapi-headers-missing`        | 400    | x-fapi-* headers not provided                 |
| `urn:wia:ob:problem:jws-signature-invalid`       | 400    | x-jws-signature header verification failed    |
| `urn:wia:ob:problem:audit-unavailable`           | 503    | audit chain write failed                      |

Warnings (200-OK with content caveats) use `Warning:` headers per
RFC 7234 §5.5 with codes namespaced under `wia-ob-`.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Capability advertisement (informative)

```
GET /.well-known/openid-configuration HTTP/1.1
Accept: application/json
```

Standard OpenID Connect discovery document, augmented with FAPI 2.0
indicators:

```json
{
  "issuer": "https://aspsp.bank-uk.example",
  "authorization_endpoint": "...",
  "token_endpoint": "...",
  "pushed_authorization_request_endpoint": "...",
  "request_object_signing_alg_values_supported": ["PS256", "ES256"],
  "token_endpoint_auth_methods_supported": ["tls_client_auth", "private_key_jwt"],
  "tls_client_certificate_bound_access_tokens": true,
  "require_pushed_authorization_requests": true,
  "code_challenge_methods_supported": ["S256"],
  "fapi_metadata": {
    "fapi_profile": "fapi_2_advanced",
    "regime": "UK-OBIE",
    "obDirectoryEntry": "...",
    "scopes_supported": ["accounts", "payments", "fundsconfirmations"]
  }
}
```

## Annex B — Pagination and rate limiting (informative)

Account-information list and transaction-history queries paginate
per OBIE / Berlin Group / KFSC profile (page size typically ≤ 500).
Per-token rate limits default to OBIE's recommendations: 4 calls
per second for AIS, 1 call per second for PIS. Rate-limit refusals
carry `urn:wia:ob:problem:rate-limited`.

## Annex C — Worked OAuth/OIDC + PAR sequence (informative)

1. TPP submits a PAR request (PHASE 2 §2) with a JAR-signed JWT
   containing the consent's authorization_details
2. ASPSP returns a `request_uri` valid for ≤ 90 seconds
3. TPP redirects the user-agent to the authorize endpoint with
   `request_uri`
4. ASPSP performs SCA per PHASE 3 §1
5. ASPSP redirects back to the TPP's redirect_uri with an
   authorization_code
6. TPP exchanges the code for a mTLS-bound access token (PHASE 2 §3)
7. TPP calls the resource endpoints with the access token plus the
   FAPI 2.0 mandatory headers

## Annex D — Worked AIS read with permission filtering (informative)

```
GET /accounts/{AccountId}/transactions?fromBookingDateTime=2025-10-27T00:00:00Z&toBookingDateTime=2026-04-27T23:59:59Z HTTP/1.1
Authorization: Bearer <mtls-bound-token>
x-fapi-interaction-id: 6b7d2c1e-...
x-fapi-auth-date: Mon, 27 Apr 2026 09:15:00 GMT
x-fapi-customer-ip-address: 203.0.113.42
```

Response (filtered to `ReadTransactionsDebits` permission only —
credits omitted because the consent did not include
`ReadTransactionsCredits`):

```json
{
  "Data": {
    "Transaction": [
      {"AccountId": "...", "TransactionId": "...", "CreditDebitIndicator": "Debit", "Status": "Booked", "BookingDateTime": "...", "Amount": {"Amount": "5.50", "Currency": "GBP"}}
    ]
  },
  "Links": {"Self": "..."},
  "Meta": {"TotalPages": 1, "FirstAvailableDateTime": "2025-10-27T00:00:00Z", "LastAvailableDateTime": "2026-04-27T23:59:59Z"}
}
```

The boundary's permission gate filters credits silently; the
response does not indicate that credits exist or are filtered.

## Annex E — Worked PIS sequence (informative)

1. TPP submits PAR with payment-initiation `authorization_details`
2. ASPSP returns request_uri; TPP redirects user
3. User authenticates (SCA + dynamic linking)
4. ASPSP redirects back with authorization_code
5. TPP exchanges code for mTLS-bound token
6. TPP submits POST /payment-initiations with the consent reference
   and JWS-detached body signature
7. Boundary validates, applies fraud + sanctions screening, forwards
   to clearing
8. Clearing returns UETR; boundary records and returns to TPP
9. Settlement events (accepted-for-settlement, settled, returned)
   propagate via webhook notifications back to the TPP

## Annex F — Versioning

OBIE 3.x and Berlin Group profiles bump independently from this PHASE; the deployment policy maps each PHASE version to the regime profile version it honours. Deprecation enters a 12-month sunset window with migration notes recorded in the audit chain.

## Annex G — OBIE event-notification API (informative)

OBIE specifies an event-notification API for asynchronous events
(consent revocation, payment-status updates, financial-data
changes). The boundary delivers events as Server-Sent Events on a
TPP-registered subscription, with each event carrying:

- `event-type` — closed enum per OBIE event-notification spec
- `consent-ref` or `transaction-ref` — the subject of the event
- `event-timestamp` — RFC 3339 with offset
- `signature` — JWS detached over the event body

Event delivery follows at-least-once semantics; duplicates are
filtered on the TPP side via the event-id. Persistent delivery
failure suspends the TPP's subscription and surfaces an incident.
