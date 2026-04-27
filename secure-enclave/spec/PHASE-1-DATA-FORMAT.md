# WIA-SEC-013: Secure Enclave - Phase 1 Core Specification

**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2025-12-25

---

## 1. Introduction

### 1.1 Purpose

This specification defines international standards for **Secure Enclave** technologies, including Trusted Execution Environments (TEE), Intel SGX, ARM TrustZone, and secure processing architectures. The standard enables hardware-based isolation for protecting sensitive data and code from unauthorized access, even from privileged software.

### 1.2 Philosophy

**弘益人間 (Hongik Ingan)** - "Benefit All Humanity"

This standard embodies the principle of protecting human privacy and security through robust hardware-based isolation, ensuring that sensitive computations remain confidential and tamper-proof.

### 1.3 Scope

This Phase 1 Core specification covers:
- Trusted Execution Environment (TEE) architecture
- Intel SGX enclave specifications
- ARM TrustZone secure world architecture
- Remote attestation protocols
- Sealed storage mechanisms
- Memory encryption

---

## 2. Architecture Overview

### 2.1 Trusted Execution Environment (TEE)

A TEE provides an isolated execution environment with hardware-enforced security guarantees:

```
┌─────────────────────────────────────┐
│         Normal World                │
│  ┌──────────────────────────────┐  │
│  │   Operating System           │  │
│  │   Applications               │  │
│  └──────────────────────────────┘  │
└─────────────────────────────────────┘
              ↕ (Hardware Isolation)
┌─────────────────────────────────────┐
│         Secure World                │
│  ┌──────────────────────────────┐  │
│  │   Trusted OS                 │  │
│  │   Secure Applications        │  │
│  │   Cryptographic Keys         │  │
│  └──────────────────────────────┘  │
└─────────────────────────────────────┘
```

**Key Properties:**
- Hardware-enforced isolation
- Cryptographic memory protection
- Secure boot and attestation
- Protected storage
- Minimal Trusted Computing Base (TCB)

### 2.2 Intel SGX Architecture

Intel Software Guard Extensions (SGX) enables applications to create secure **enclaves**:

```
┌────────────────────────────────────────┐
│         Untrusted Application          │
│  ┌──────────────────────────────────┐ │
│  │  ┌──────────────────────┐        │ │
│  │  │   Secure Enclave     │        │ │
│  │  │   (Protected)        │        │ │
│  │  │  - Encrypted Memory  │        │ │
│  │  │  - Sealed Storage    │        │ │
│  │  │  - Attestation       │        │ │
│  │  └──────────────────────┘        │ │
│  │                                   │ │
│  │   Untrusted Code                 │ │
│  └──────────────────────────────────┘ │
└────────────────────────────────────────┘
```

**SGX Features:**
- Memory Encryption Engine (MEE)
- Enclave Page Cache (EPC)
- MRENCLAVE measurement
- Local and remote attestation
- Sealed storage with CPU-derived keys

### 2.3 ARM TrustZone Architecture

ARM TrustZone partitions hardware and software resources:

```
┌──────────────────┬──────────────────┐
│   Normal World   │   Secure World   │
├──────────────────┼──────────────────┤
│   Rich OS        │   Trusted OS     │
│   (Linux/Android)│   (OP-TEE/Trusty)│
├──────────────────┼──────────────────┤
│   NS Memory      │   S Memory       │
│   NS Peripherals │   S Peripherals  │
├──────────────────┼──────────────────┤
│        ARM Processor (TrustZone)    │
└──────────────────┴──────────────────┘
```

**TrustZone Features:**
- Hardware-enforced world separation
- Secure Monitor Call (SMC) interface
- Trusted Applications (TAs)
- Secure storage
- Hardware crypto acceleration

---

## 3. Data Structures

### 3.1 Enclave Identity

```rust
/// Enclave measurement and identity
pub struct EnclaveIdentity {
    /// SHA-256 hash of enclave code and data
    pub mr_enclave: [u8; 32],

    /// SHA-256 hash of enclave signer public key
    pub mr_signer: [u8; 32],

    /// Product ID assigned by enclave author
    pub isv_prod_id: u16,

    /// Security version number
    pub isv_svn: u16,

    /// Enclave attributes (debug, mode64bit, etc.)
    pub attributes: u64,
}
```

### 3.2 Attestation Report

```rust
/// SGX Attestation Report
pub struct AttestationReport {
    /// Report version
    pub version: u16,

    /// Signature type (EPID/DCAP)
    pub sign_type: u16,

    /// EPID group ID
    pub epid_group_id: [u8; 4],

    /// Quoting Enclave security version
    pub qe_svn: u16,

    /// Provisioning Certification Enclave SVN
    pub pce_svn: u16,

    /// Basename for linkability
    pub basename: [u8; 32],

    /// Custom data from enclave (64 bytes)
    pub report_data: [u8; 64],

    /// CPU security version
    pub cpu_svn: [u8; 16],

    /// Enclave identity
    pub identity: EnclaveIdentity,

    /// Signature over the report
    pub signature: Vec<u8>,
}
```

### 3.3 Sealed Data

```rust
/// Sealed data structure for persistent storage
pub struct SealedData {
    /// Key derivation policy (MRENCLAVE/MRSIGNER)
    pub key_policy: KeyPolicy,

    /// Initialization vector for encryption
    pub iv: [u8; 12],

    /// Encrypted payload
    pub ciphertext: Vec<u8>,

    /// Authentication tag (GCM)
    pub mac: [u8; 16],

    /// Additional authenticated data
    pub aad: Vec<u8>,
}

pub enum KeyPolicy {
    /// Key bound to enclave measurement
    MRENCLAVE,

    /// Key bound to enclave signer
    MRSIGNER,
}
```

---

## 4. Core Operations

### 4.1 Enclave Creation

**Intel SGX:**

```c
sgx_status_t sgx_create_enclave(
    const char *file_name,           // Enclave binary path
    const int debug,                 // Debug mode flag
    sgx_launch_token_t *token,       // Launch token
    int *updated,                    // Token update flag
    sgx_enclave_id_t *eid,          // Output: enclave ID
    sgx_misc_attribute_t *attribute  // Output: attributes
);
```

**ARM TrustZone:**

```c
TEEC_Result TEEC_OpenSession(
    TEEC_Context *context,           // TEE context
    TEEC_Session *session,           // Output: session
    const TEEC_UUID *destination,    // TA UUID
    uint32_t connectionMethod,       // Connection method
    const void *connectionData,      // Connection parameters
    TEEC_Operation *operation,       // Session parameters
    uint32_t *returnOrigin          // Error origin
);
```

### 4.2 Remote Attestation

```rust
/// Generate attestation quote
pub async fn generate_attestation_quote(
    enclave_id: EnclaveId,
    report_data: &[u8; 64],
) -> Result<AttestationQuote> {
    // 1. Create enclave report with custom data
    let report = create_enclave_report(enclave_id, report_data)?;

    // 2. Get quoting enclave to sign the report
    let quote = get_quote(report)?;

    // 3. Return signed quote for verification
    Ok(quote)
}

/// Verify attestation quote
pub async fn verify_attestation_quote(
    quote: &AttestationQuote,
) -> Result<AttestationResult> {
    // 1. Verify quote signature
    verify_quote_signature(quote)?;

    // 2. Check enclave identity (MRENCLAVE/MRSIGNER)
    verify_enclave_identity(&quote.report.identity)?;

    // 3. Verify with Intel Attestation Service (IAS)
    let ias_report = verify_with_ias(quote).await?;

    Ok(ias_report)
}
```

### 4.3 Sealed Storage

```rust
/// Seal data for persistent storage
pub fn seal_data(
    data: &[u8],
    key_policy: KeyPolicy,
) -> Result<SealedData> {
    // 1. Derive sealing key from CPU
    let seal_key = derive_seal_key(key_policy)?;

    // 2. Generate random IV
    let iv = generate_random_iv();

    // 3. Encrypt with AES-GCM
    let (ciphertext, mac) = aes_gcm_encrypt(data, &seal_key, &iv)?;

    Ok(SealedData {
        key_policy,
        iv,
        ciphertext,
        mac,
        aad: vec![],
    })
}

/// Unseal data from storage
pub fn unseal_data(sealed: &SealedData) -> Result<Vec<u8>> {
    // 1. Derive same sealing key
    let seal_key = derive_seal_key(sealed.key_policy)?;

    // 2. Decrypt and verify MAC
    let plaintext = aes_gcm_decrypt(
        &sealed.ciphertext,
        &seal_key,
        &sealed.iv,
        &sealed.mac,
    )?;

    Ok(plaintext)
}
```

---

## 5. Memory Encryption

### 5.1 Memory Encryption Engine (MEE)

Intel SGX uses MEE to encrypt enclave memory:

```
┌──────────────────────────────────┐
│    Enclave Memory (Plaintext)   │
└──────────────┬───────────────────┘
               │
               ▼
       ┌──────────────┐
       │     MEE      │
       │  AES-128-CTR │
       │  + Integrity │
       └──────────────┘
               │
               ▼
┌──────────────────────────────────┐
│   Physical Memory (Encrypted)   │
└──────────────────────────────────┘
```

**Encryption Process:**

1. **Counter Mode:** Each cache line has a counter
2. **AES-128-CTR:** Encrypt with line-specific counter
3. **Integrity Tree:** Merkle tree for tamper detection
4. **MAC:** Galois Message Authentication Code

### 5.2 Page Table Protection

```
Virtual Address → Page Table → EPC → MEE → DRAM
                     ▲
                     │
              Protected by CPU
```

---

## 6. Security Guarantees

### 6.1 Confidentiality

- **Memory Encryption:** All enclave memory encrypted
- **Register Protection:** Registers cleared on world switch
- **DMA Protection:** No DMA access to secure memory

### 6.2 Integrity

- **Code Measurement:** MRENCLAVE verifies code integrity
- **Memory Integrity:** Merkle tree detects tampering
- **Replay Protection:** Counters prevent rollback

### 6.3 Attestation

- **Local Attestation:** Enclaves on same platform
- **Remote Attestation:** Third-party verification via IAS/DCAP
- **Freshness:** Nonces prevent replay attacks

---

## 7. Use Cases

### 7.1 Secure Key Management

```rust
// Store encryption keys in enclave
pub struct SecureKeyStore {
    master_key: [u8; 32],
}

impl SecureKeyStore {
    pub fn derive_key(&self, context: &[u8]) -> [u8; 32] {
        // Key derivation happens inside enclave
        hkdf_expand(&self.master_key, context)
    }
}
```

### 7.2 Confidential Computing

```rust
// Process sensitive data in enclave
pub async fn process_medical_records(
    encrypted_records: &[u8],
) -> Result<Statistics> {
    // 1. Decrypt inside enclave
    let records = decrypt_in_enclave(encrypted_records)?;

    // 2. Compute statistics without exposing raw data
    let stats = compute_statistics(&records);

    // 3. Return only aggregated results
    Ok(stats)
}
```

### 7.3 Secure Biometric Storage

```rust
// Store biometric templates in sealed storage
pub fn store_biometric_template(
    user_id: &str,
    template: &BiometricTemplate,
) -> Result<()> {
    let data = serialize_template(template);
    let sealed = seal_data(&data, KeyPolicy::MRENCLAVE)?;
    persist_sealed_data(user_id, sealed)?;
    Ok(())
}
```

---

## 8. Implementation Requirements

### 8.1 Hardware Requirements

**Intel SGX:**
- Intel CPU with SGX support (6th gen Core or later)
- BIOS with SGX enabled
- SGX driver installed
- Sufficient EPC memory

**ARM TrustZone:**
- ARM Cortex-A processor with TrustZone
- Trusted OS (OP-TEE, Trusty, QSEE)
- Secure boot enabled

### 8.2 Software Requirements

- Enclave/TA development SDK
- Attestation client library
- Cryptographic primitives (AES, SHA-256, ECDSA)
- Secure random number generator

---

## 9. Compliance

### 9.1 Standards Conformance

- **TCG:** Trusted Computing Group specifications
- **GlobalPlatform:** TEE specifications
- **FIPS 140-3:** Cryptographic module validation
- **Common Criteria:** EAL4+ certification

### 9.2 Best Practices

1. **Minimize TCB:** Keep enclave code small and auditable
2. **Side-Channel Protection:** Implement constant-time operations
3. **Attestation:** Always verify enclave identity before trust
4. **Key Management:** Use hardware-derived keys
5. **Updates:** Support secure enclave updates with version rollback protection

---

## 10. References

1. Intel SGX Developer Reference: https://software.intel.com/sgx
2. ARM TrustZone Technology: https://developer.arm.com/trustzone
3. GlobalPlatform TEE Specifications: https://globalplatform.org/
4. IETF RATS Working Group: https://datatracker.ietf.org/wg/rats/

---

**Published by:**
World Certification Industry Association (WIA)

**License:** CC BY-SA 4.0

**弘益人間 · Benefit All Humanity**
