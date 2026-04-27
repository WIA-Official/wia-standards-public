# WIA-SEC-013 — Phase 3: Cross-Platform Interoperability

> Secure-enclave Phase 3 covers interoperability between TEE
> implementations: Intel SGX, AMD SEV-SNP, ARM TrustZone, AWS
> Nitro Enclaves, and Apple Secure Enclave. Cross-platform
> attestation and remote attestation flows are specified here.

## Phase 3: Cross-Platform Interoperability

### 3.1 Unified TEE API

```typescript
// Platform-agnostic TEE interface
interface TrustedExecutionEnvironment {
  // Platform detection
  getPlatform(): TEEPlatform; // SGX, TrustZone, SEV, Nitro

  // Enclave/TA lifecycle
  createEnclave(config: EnclaveConfig): Promise<EnclaveHandle>;
  destroyEnclave(handle: EnclaveHandle): Promise<void>;

  // Attestation
  generateAttestation(
    handle: EnclaveHandle,
    data: Uint8Array
  ): Promise<AttestationQuote>;

  verifyAttestation(
    quote: AttestationQuote
  ): Promise<AttestationResult>;

  // Sealed storage
  sealData(
    handle: EnclaveHandle,
    data: Uint8Array,
    policy: SealPolicy
  ): Promise<SealedBlob>;

  unsealData(
    handle: EnclaveHandle,
    blob: SealedBlob
  ): Promise<Uint8Array>;

  // Secure communication
  establishSecureChannel(
    handle: EnclaveHandle,
    remote: EnclaveIdentity
  ): Promise<SecureChannel>;
}

// Platform-specific implementations
class IntelSGXTEE implements TrustedExecutionEnvironment { }
class ARMTrustZoneTEE implements TrustedExecutionEnvironment { }
class AMDSevTEE implements TrustedExecutionEnvironment { }
class AWSNitroTEE implements TrustedExecutionEnvironment { }
```

### 3.2 Cross-Platform Attestation

```rust
/// Universal attestation format
pub struct UniversalAttestation {
    /// Platform type
    pub platform: TEEPlatform,

    /// Platform-specific quote
    pub quote: Vec<u8>,

    /// Enclave/TA measurement
    pub measurement: [u8; 32],

    /// Certificate chain
    pub certificates: Vec<Certificate>,

    /// Timestamp
    pub timestamp: u64,

    /// Signature
    pub signature: Signature,
}

pub enum TEEPlatform {
    IntelSGX,
    ARMTrustZone,
    AMDSEV,
    AWSNitro,
    AzureConfidentialVM,
}

/// Verify attestation across platforms
pub fn verify_universal_attestation(
    attestation: &UniversalAttestation,
) -> Result<bool> {
    match attestation.platform {
        TEEPlatform::IntelSGX => verify_sgx_quote(&attestation.quote),
        TEEPlatform::ARMTrustZone => verify_trustzone_attestation(&attestation.quote),
        TEEPlatform::AMDSEV => verify_sev_report(&attestation.quote),
        TEEPlatform::AWSNitro => verify_nitro_attestation(&attestation.quote),
        _ => Err(Error::UnsupportedPlatform),
    }
}
```

### 3.3 Cloud TEE Services

**AWS Nitro Enclaves:**

```python
import aws_nitro_enclaves as nitro

# Create Nitro Enclave
enclave = nitro.create_enclave(
    enclave_image='myapp.eif',
    cpu_count=2,
    memory_mb=512
)

# Get attestation document
attestation = nitro.get_attestation_document(
    enclave_id=enclave.id,
    user_data=b'challenge_nonce'
)

# Verify with AWS KMS
kms_result = aws_kms.verify_attestation(attestation)
```

**Azure Confidential Computing:**

```csharp
// Azure Confidential VM with SEV-SNP
using Microsoft.Azure.ConfidentialComputing;

var attestation = await AttestationClient.GetAttestationAsync(
    new AttestationRequest {
        Runtime = RuntimeType.SEV_SNP,
        Nonce = GenerateNonce()
    }
);

// Verify with Microsoft Azure Attestation
var result = await maaClient.VerifyAsync(attestation);
```

---

## 3.4 Cross-platform attestation envelope

Different TEE families emit different native attestation evidence:
SGX produces a Quote signed by a Quoting Enclave; AMD SEV-SNP
produces an Attestation Report signed by the AMD Secure Processor;
TDX produces a Quote signed via Intel DCAP; ARM TrustZone produces
a Realm attestation; AWS Nitro produces a COSE_Sign1-wrapped
attestation document.

Phase 3 specifies a unified envelope that carries the native
evidence verbatim plus a vendor-neutral header. The header gives a
verifier enough information to dispatch to the correct platform-
specific verification path without parsing the native blob.

```json
{
  "wia_secure_enclave_version": "1.0.0",
  "type": "attestation_envelope",
  "tee_family": "tdx",
  "claims": {
    "MRTD": "0x...",
    "user_data": "..."
  },
  "evidence": "<base64 native blob>",
  "issued_at": "2026-04-27T03:14:33Z"
}
```

The verifier parses the header, dispatches to the family-specific
verifier, and emits a verdict envelope carrying the verifier's
identity, the trust-anchor chain consulted, and the verdict
(`ok` / `fail` / `unknown`). The verdict envelope is signed and
auditable downstream.

## 3.5 Trust anchors and rotation

Every TEE family has a root trust anchor maintained by the platform
vendor. The standard requires every conformant verifier to:

1. Fetch trust anchors over an authenticated channel from a vendor
   endpoint.
2. Refresh trust anchors on a documented cadence (≤ 30 days).
3. Cache trust anchors signed by the vendor for the cache lifetime.
4. Refuse attestations whose chain root is not present in the
   active trust anchor set.

The most common failure mode here is stale collateral: a verifier
that has not refreshed its DCAP collateral for several months will
start refusing valid attestations after the collateral expiry.
Conformant verifiers MUST refresh ahead of expiry and emit a
warning envelope when ≤ 7 days from expiry.

## 3.6 Cross-cloud workload migration

A workload migrating between cloud providers (e.g. AWS Nitro
Enclave to GCP Confidential VM) re-attests on arrival. The new
attestation is verified against the new provider's trust anchors;
the workload's sealing key is re-derived from the new attestation
identity and a tenant-supplied key-wrapping key. Sealed data from
the prior provider is unsealed using the prior identity's key,
re-sealed with the new identity's key, and the prior identity's
key is destroyed.

The migration itself emits a chain of envelopes (workload exit on
prior provider, attestation on new provider, key-rotation event)
so an auditor can verify the migration was clean and that no
sealed data lingered with the prior provider's identity.

## 3.7 Multi-vendor verification policies

Production policy engines (e.g. Microsoft Azure Attestation) accept
multi-vendor evidence and emit a unified verdict. The standard's
verdict envelope carries a `policy_id` and `policy_version` so a
consumer of the verdict can verify which policy was applied and
whether the policy itself has been updated. Policy updates that
tighten requirements (e.g. raising the minimum SGX SVN level) are
deployed via a deprecation window: the verifier emits a warning
envelope for some weeks before refusing attestations that satisfy
only the older policy.

## 3.8 Replay defence on attestation envelopes

Each attestation envelope carries a 96-bit nonce and an RFC 3339
timestamp. Verifiers reject envelopes with skew greater than
±300 seconds and reject envelopes whose `(quote_id, nonce)` tuple
has been seen within the last 600 seconds. The seen-nonce cache is
persistent across verifier restarts so a power cycle does not
re-open the window for a previously-blocked replay.

## 3.9 Federation of attestation verifiers

Production deployments rarely run a single verifier. Cross-cloud
workloads need verifiers in each cloud, and audit-grade workloads
need an independent third-party verifier in addition to the
cloud's first-party verifier. Phase 3 specifies a federation
envelope that lets a workload submit one attestation and receive
verdicts from multiple verifiers without re-signing the evidence.

```json
{
  "wia_secure_enclave_version": "1.0.0",
  "type": "verification_request",
  "request_id": "req_01HXY...",
  "evidence": "<base64 native blob>",
  "tee_family": "tdx",
  "verifiers": [
    "did:wia:verifier:azure-attestation",
    "did:wia:verifier:wia-third-party-eu",
    "did:wia:verifier:tenant-self-host"
  ],
  "issued_at": "RFC 3339",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

The aggregator service routes the request to each named verifier,
collects the signed verdicts, and returns the bundle to the
requester. The requester decides on its policy: unanimous
agreement is required for high-trust workloads, while
operational workloads accept a configurable quorum.

## 3.10 Audit log for attestation outcomes

Every conformant verifier MUST emit an audit envelope for every
verdict issued. The envelope carries `request_id`, `verdict`
(`ok` / `fail` / `unknown`), `policy_id` and `policy_version`
applied, `trust_anchor_chain` consulted, `verifier_id` (DID),
`issued_at` (RFC 3339), and the Ed25519 signature over the
canonical JSON form.

Audit envelopes are written to an append-only log replicated
across at least two storage backends and exposed via a federated
query endpoint so an auditor reconstructing an incident months
later can verify which verdicts were issued at which time
without trusting the verifier's current state.

## 3.11 Compatibility with existing TEE attestation services

The Phase 3 envelope is intentionally a thin wrapper around
existing native evidence. Conformant implementations MAY continue
to use the native verification flow (Intel IAS / DCAP, AMD KDS,
Microsoft Azure Attestation, AWS Nitro attestation document
verifier) and adopt the envelope only at the outer edge. This
keeps migration cost low: existing verifier libraries do not need
to be rewritten, only wrapped.

---

