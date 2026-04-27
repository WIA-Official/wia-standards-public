# WIA-SEC-013 — Phase 2: Advanced Features

> Secure-enclave Phase 2 covers the advanced operational features
> built on top of the Phase 1 core: confidential containers,
> machine-learning inference inside an enclave, and the
> orchestration patterns required to run these workloads at
> scale on Kubernetes-class platforms.


## Phase 2: Advanced Features

### 2.1 Multi-Party Computation (MPC) in TEE

Combine TEEs with cryptographic MPC for enhanced privacy:

```rust
/// Secure multi-party computation protocol
pub struct SecureMPC {
    parties: Vec<PartyInfo>,
    enclave_id: EnclaveId,
}

impl SecureMPC {
    /// Each party contributes encrypted input
    pub async fn add_party_input(
        &mut self,
        party_id: u32,
        encrypted_input: &[u8],
    ) -> Result<()> {
        // Decrypt and verify inside enclave
        let input = decrypt_in_enclave(encrypted_input)?;

        // Store for computation
        self.store_party_input(party_id, input)?;
        Ok(())
    }

    /// Compute result without revealing individual inputs
    pub async fn compute_aggregate(&self) -> Result<Vec<u8>> {
        // All computation happens inside enclave
        let result = compute_inside_enclave(&self.parties)?;

        // Encrypt result for output
        let encrypted = encrypt_result(result)?;
        Ok(encrypted)
    }
}
```

**Use Cases:**
- Privacy-preserving machine learning
- Secure voting systems
- Confidential data analytics
- Federated learning

### 2.2 Confidential Containers

Run entire containerized workloads in TEE:

```yaml
# Kubernetes Pod with SGX enclave
apiVersion: v1
kind: Pod
metadata:
  name: confidential-workload
spec:
  runtimeClassName: kata-qemu-sev
  containers:
  - name: secure-app
    image: myapp:latest
    resources:
      limits:
        sgx.intel.com/epc: "512Mi"
    volumeMounts:
    - name: sealed-storage
      mountPath: /sealed
  volumes:
  - name: sealed-storage
    hostPath:
      path: /var/sealed-data
```

**Benefits:**
- Lift-and-shift existing applications
- Protect entire container runtime
- Encrypted memory and storage
- Remote attestation for containers

### 2.3 Secure Enclaves for AI/ML

```python
# PyTorch model inference in SGX enclave
import secure_torch

# Load model inside enclave
model = secure_torch.load_model_enclave('model.pth')

# Inference without exposing data
def secure_inference(encrypted_input):
    # Decrypt inside enclave
    input_data = secure_torch.decrypt(encrypted_input)

    # Run inference in enclave
    with secure_torch.enclave():
        output = model(input_data)

    # Encrypt result
    return secure_torch.encrypt(output)
```

**Applications:**
- Medical image analysis
- Financial fraud detection
- Biometric authentication
- Sensitive NLP tasks

### 2.4 Blockchain Integration

```solidity
// Ethereum smart contract with SGX verification
contract SecureOracle {
    struct AttestationProof {
        bytes32 mrenclave;
        bytes quote;
        bytes iasCert;
    }

    mapping(bytes32 => AttestationProof) public attestations;

    function submitDataWithAttestation(
        bytes memory data,
        AttestationProof memory proof
    ) public {
        // Verify SGX attestation on-chain
        require(verifyAttestation(proof), "Invalid attestation");

        // Store data with proof
        bytes32 hash = keccak256(data);
        attestations[hash] = proof;
    }

    function verifyAttestation(
        AttestationProof memory proof
    ) internal view returns (bool) {
        // Verify IAS signature
        // Check MRENCLAVE matches expected value
        // Validate quote structure
        return true; // Simplified
    }
}
```

---

## 2.4 Confidential containers — operational guidance

A conformant deployment of confidential containers (Kata Containers
in TDX mode, Confidential Containers, AMD SEV-SNP CVMs) MUST satisfy
five operational requirements:

1. **Image attestation.** Every container image is verified at launch
   against a signed image-digest envelope. Unsigned or unverified
   images are refused; the runtime emits a `cc_image_refused` event
   carrying the digest and the verification failure reason.
2. **Secret injection.** Secrets enter the container only via a
   sealed channel from the orchestrator's KMS to the enclave.
   Secrets are never written to a container layer or any host-visible
   filesystem.
3. **Memory budget.** The enclave's encrypted memory (EPC for SGX,
   private memory for SEV-SNP and TDX) is a finite resource. The
   runtime tracks per-tenant memory consumption and refuses
   over-budget allocations rather than silently swapping to host
   memory (which would defeat the confidentiality guarantee).
4. **Crash dumps.** A crash inside the enclave produces a sealed
   crash report rather than a host-readable core file. The report
   is readable only by the tenant's debugging team holding the
   matching sealing key.
5. **Side-channel resistance.** The runtime applies vendor-published
   side-channel mitigations (LFENCE on Intel, equivalent on AMD/ARM)
   and reports the applied mitigations as part of the launch
   attestation so a downstream verifier can refuse pods running on
   unmitigated hardware.

## 2.5 ML inference inside an enclave

Running ML inference inside an enclave changes the threat model:
the model owner trusts the enclave to compute over inputs without
revealing them, and the input owner trusts the enclave to compute
over the model without revealing it. Both trust assumptions rest on
the attestation chain.

The standard's ML-inference profile requires:

- **Quantised model weights.** Full-precision FP32 weights rarely
  fit in encrypted memory; the profile mandates INT8 or INT4
  quantisation with a documented accuracy delta vs. FP32 on the
  model's eval set.
- **Streaming inputs.** Large inputs (medical imagery, full-document
  text) stream into the enclave in encrypted chunks rather than
  loading wholesale into encrypted memory.
- **Output redaction.** Inference outputs that risk leaking input
  content (model gradients, attention maps) are redacted before
  exiting the enclave unless the tenant explicitly opts in.

## 2.6 Orchestration patterns at scale

Production deployments running confidential workloads on Kubernetes
typically separate the cluster into trust zones: a regular zone for
non-confidential services, and one or more confidential zones each
running a single tenant's workloads. The standard does not mandate
this topology; it mandates only that any cross-zone data flow carry
explicit attestation evidence so an auditor can verify which trust
boundary a given byte crossed.

A scheduler-side `tee_required` taint and toleration pair pin
confidential workloads to confidential nodes; nodes lacking valid
attestation are tainted out of the scheduling pool until they
re-attest. The lifecycle observer emits envelopes for every node-
level attestation pass / fail so a fleet's attestation health is
auditable from a central log without granting the central log access
to the workload data.

## 2.7 Failure modes and operator responses

The most common Phase 2 failure modes:

| Failure | Cause | Operator response |
|---------|-------|-------------------|
| Image attestation fails | Image unsigned or signing key revoked | Refuse launch; alert image-publisher |
| EPC exhausted | Tenant exceeded encrypted-memory quota | Surface clear quota-exceeded error to tenant; do NOT swap |
| Side-channel mitigation absent | Bare-metal not patched | Mark node unhealthy; reschedule workloads |
| Quote signature stale | IAS / DCAP collateral expired | Re-fetch collateral on a documented refresh cadence |
| Sealed-key access from wrong identity | Wrong enclave or wrong tenant | Refuse decryption; log with envelope for forensics |

Each failure has a documented event class so dashboards can group by
operator-actionable categories rather than per-vendor SDK quirks.

## 2.8 Capacity planning notes

Encrypted memory is the dominant capacity constraint on Phase 2
deployments. On Intel SGX1 the EPC ceiling per socket is roughly
128 MiB; SGX2 raised this substantially but the fleet still mixes
both generations. AMD SEV-SNP and Intel TDX expose memory in the
hundreds of GiB range per socket but allocate at VM-launch time,
not on demand, so over-provisioning is wasteful and
under-provisioning aborts the launch.

The standard recommends operators publish a per-tenant memory
budget envelope at onboarding so capacity is allocated from a
declared pool rather than discovered when a workload fails to
launch. The envelope carries:

```json
{
  "tenant_id": "did:wia:tenant:...",
  "epc_mib_reserved": 0,
  "snp_private_gib_reserved": 0,
  "tdx_private_gib_reserved": 0,
  "node_pool": "confidential-zone-A",
  "issued_at": "RFC 3339"
}
```

A capacity-planner service consumes these envelopes and refuses
new tenants whose declared budget exceeds the remaining headroom
on the requested node pool.

---

