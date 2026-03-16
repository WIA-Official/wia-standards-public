# WIA-SEC-013: Secure Enclave - Glossary

**Version:** 1.0.0
**Last Updated:** 2025-12-25

---

## A

**AAD (Additional Authenticated Data)**
Extra data included in authentication but not encrypted. Used in AEAD modes like AES-GCM to authenticate metadata.

**AES-GCM (Advanced Encryption Standard - Galois/Counter Mode)**
Authenticated encryption algorithm providing both confidentiality and integrity. Used by SGX for memory encryption.

**AMD SEV (Secure Encrypted Virtualization)**
AMD's technology for encrypting virtual machine memory to protect against hypervisor attacks.

**ARM TrustZone**
Hardware security technology in ARM processors that creates a secure world isolated from the normal world.

**Attestation**
Process of proving that an enclave is genuine and running expected code. Can be local (same platform) or remote (third party).

**AWS Nitro Enclaves**
Amazon's isolated compute environment for processing sensitive data in EC2 instances.

---

## B

**Basename**
Value used in EPID signatures to control linkability between attestation quotes.

---

## C

**Confidential Computing**
Computing paradigm that protects data in use through hardware-based trusted execution environments.

**CPU SVN (Security Version Number)**
Version number tracking security patches applied to CPU microcode.

---

## D

**DCAP (Data Center Attestation Primitives)**
Intel's scalable attestation model for SGX that doesn't require connection to Intel's servers.

**DMA (Direct Memory Access)**
Hardware capability to access memory without CPU involvement. TEEs protect against DMA attacks.

---

## E

**ECALL (Enclave Call)**
Call from untrusted application into SGX enclave. Crosses trust boundary.

**EPC (Enclave Page Cache)**
Special memory region for storing encrypted enclave pages in Intel SGX.

**EPID (Enhanced Privacy ID)**
Group signature scheme used by Intel SGX for anonymous remote attestation.

---

## F

**FIPS 140-3**
Federal Information Processing Standard for cryptographic module security.

**FHE (Fully Homomorphic Encryption)**
Encryption allowing computation on ciphertext without decryption.

---

## G

**GlobalPlatform**
Industry consortium defining standards for TEEs and secure elements.

**GMAC (Galois Message Authentication Code)**
Authentication algorithm used with AES-GCM.

---

## H

**HKDF (HMAC-based Key Derivation Function)**
Cryptographic key derivation function based on HMAC.

**HSM (Hardware Security Module)**
Dedicated crypto processor for secure key management.

---

## I

**IAS (Intel Attestation Service)**
Intel's service for verifying SGX attestation quotes.

**ISV (Independent Software Vendor)**
Developer creating enclave applications.

**ISV Product ID**
Identifier for enclave software assigned by developer.

**ISV SVN (Security Version Number)**
Version number for enclave software to track security updates.

---

## K

**Key Derivation**
Process of generating cryptographic keys from a master secret.

**Key Policy**
Rule determining how sealing keys are derived (MRENCLAVE or MRSIGNER).

**KSS (Key Separation and Sharing)**
SGX feature for additional key isolation and management.

---

## M

**MAC (Message Authentication Code)**
Cryptographic checksum proving message authenticity and integrity.

**MEE (Memory Encryption Engine)**
Hardware component in Intel SGX encrypting enclave memory.

**MRENCLAVE**
SHA-256 hash of enclave code, data, and layout. Unique identifier for enclave build.

**MRSIGNER**
SHA-256 hash of public key used to sign the enclave.

---

## N

**Nitro Enclaves**
See AWS Nitro Enclaves.

**Nonce**
Random number used once to ensure freshness in cryptographic protocols.

---

## O

**OCALL (Outside Call)**
Call from SGX enclave to untrusted application code.

**OP-TEE (Open Portable Trusted Execution Environment)**
Open-source TEE for ARM TrustZone.

---

## P

**PCE (Provisioning Certification Enclave)**
Intel enclave responsible for platform provisioning.

**PQC (Post-Quantum Cryptography)**
Cryptographic algorithms resistant to quantum computer attacks.

---

## Q

**QE (Quoting Enclave)**
Intel enclave that signs attestation reports to create quotes.

**Quote**
Signed attestation report from SGX enclave, verifiable by third parties.

---

## R

**RATS (Remote ATtestation procedureS)**
IETF working group defining remote attestation standards.

**Remote Attestation**
Proving enclave authenticity to a remote verifier via cryptographic evidence.

---

## S

**Sealed Storage**
Encrypted persistent storage where data can only be unsealed by the same enclave.

**Secure Boot**
Boot process verifying all software before execution.

**Secure World**
Isolated execution environment in ARM TrustZone where trusted code runs.

**SEV (Secure Encrypted Virtualization)**
See AMD SEV.

**SGX (Software Guard Extensions)**
Intel's instruction set extension for creating secure enclaves.

**Side-Channel Attack**
Attack exploiting information leaked through timing, power consumption, or electromagnetic radiation.

**SMC (Secure Monitor Call)**
ARM instruction to switch from normal world to secure world.

**Spectre/Meltdown**
CPU vulnerabilities exploiting speculative execution. TEEs implement mitigations.

---

## T

**TA (Trusted Application)**
Application running in ARM TrustZone secure world.

**TCB (Trusted Computing Base)**
Minimal set of software/hardware components critical for security.

**TCG (Trusted Computing Group)**
Industry consortium developing trusted computing standards.

**TEE (Trusted Execution Environment)**
Isolated execution environment with hardware-backed security guarantees.

**TPM (Trusted Platform Module)**
Hardware chip providing cryptographic functions and secure storage.

**TrustZone**
See ARM TrustZone.

---

## U

**UUID (Universally Unique Identifier)**
128-bit identifier for TrustZone Trusted Applications.

---

## V

**Verifiable Credential**
W3C standard for cryptographically verifiable digital credentials.

---

## W

**WebAuthn**
W3C standard for passwordless authentication using public key cryptography.

---

## Z

**Zero Trust**
Security model requiring verification for every access request.

**zk-SNARK (Zero-Knowledge Succinct Non-Interactive Argument of Knowledge)**
Cryptographic proof allowing verification without revealing underlying data.

---

## Acronyms Quick Reference

| Acronym | Full Name |
|---------|-----------|
| AAD | Additional Authenticated Data |
| AES | Advanced Encryption Standard |
| AMD | Advanced Micro Devices |
| ARM | Advanced RISC Machines |
| AWS | Amazon Web Services |
| CPU | Central Processing Unit |
| DCAP | Data Center Attestation Primitives |
| DMA | Direct Memory Access |
| ECALL | Enclave Call |
| EPC | Enclave Page Cache |
| EPID | Enhanced Privacy ID |
| FHE | Fully Homomorphic Encryption |
| FIPS | Federal Information Processing Standards |
| GCM | Galois/Counter Mode |
| GMAC | Galois Message Authentication Code |
| HKDF | HMAC-based Key Derivation Function |
| HSM | Hardware Security Module |
| IAS | Intel Attestation Service |
| IETF | Internet Engineering Task Force |
| ISV | Independent Software Vendor |
| KSS | Key Separation and Sharing |
| MAC | Message Authentication Code |
| MEE | Memory Encryption Engine |
| OCALL | Outside Call |
| OP-TEE | Open Portable Trusted Execution Environment |
| PCE | Provisioning Certification Enclave |
| PQC | Post-Quantum Cryptography |
| QE | Quoting Enclave |
| RATS | Remote ATtestation procedureS |
| SEV | Secure Encrypted Virtualization |
| SGX | Software Guard Extensions |
| SMC | Secure Monitor Call |
| SVN | Security Version Number |
| TA | Trusted Application |
| TCB | Trusted Computing Base |
| TCG | Trusted Computing Group |
| TEE | Trusted Execution Environment |
| TPM | Trusted Platform Module |
| UUID | Universally Unique Identifier |
| VM | Virtual Machine |
| W3C | World Wide Web Consortium |

---

## Common TEE Technologies Comparison

| Feature | Intel SGX | ARM TrustZone | AMD SEV | AWS Nitro |
|---------|-----------|---------------|---------|-----------|
| **Isolation Level** | Process-level | System-level | VM-level | VM-level |
| **Memory Encryption** | Yes (MEE) | Optional | Yes (SME) | Yes |
| **Attestation** | Local + Remote | Varies | Remote | Remote |
| **Ecosystem** | x86 servers/clients | Mobile/embedded/IoT | x86 servers | AWS cloud |
| **Key Use Cases** | Cloud computing | Mobile payments | VM isolation | Confidential computing |

---

## Security Guarantees Overview

### Confidentiality
- **Memory Encryption**: All data encrypted in DRAM
- **Register Protection**: Cleared on context switch
- **Cache Protection**: Encrypted cache lines
- **Storage Encryption**: Sealed data at rest

### Integrity
- **Code Measurement**: MRENCLAVE/MRSIGNER
- **Memory Integrity**: Merkle tree verification
- **Replay Protection**: Version counters
- **Attestation**: Cryptographic proof

### Availability
- **Denial of Service**: Limited protection
- **Resource Management**: OS controls allocation
- **Side Channels**: Ongoing research/mitigation

---

## Related Standards

### WIA Standards
- **WIA-SEC-007**: Biometric Authentication
- **WIA-SEC-008**: Multi-Factor Authentication
- **WIA-SEC-010**: Identity Management
- **WIA-SEC-011**: Blockchain Security
- **WIA-SEC-012**: Zero Trust Architecture

### External Standards
- **ISO/IEC 19790**: Security requirements for cryptographic modules
- **ISO/IEC 15408**: Common Criteria for IT Security Evaluation
- **FIPS 140-3**: Cryptographic Module Validation Program
- **GlobalPlatform TEE**: TEE specifications
- **TCG TPM**: Trusted Platform Module specifications

---

## Further Reading

### Technical Documentation
1. Intel SGX Developer Guide: https://software.intel.com/sgx-sdk
2. ARM TrustZone Developer Portal: https://developer.arm.com/ip-products/security-ip/trustzone
3. AMD SEV API Specification: https://developer.amd.com/sev/
4. GlobalPlatform TEE Specifications: https://globalplatform.org/specs-library/

### Academic Papers
1. "Intel SGX Explained" by Costan & Devadas (2016)
2. "ARM TrustZone: Integrating Building Blocks for Mobile Security" (2013)
3. "Sanctum: Minimal Hardware Extensions for Strong Software Isolation" (2016)

### Industry Resources
1. Confidential Computing Consortium: https://confidentialcomputing.io/
2. IETF RATS Working Group: https://datatracker.ietf.org/wg/rats/
3. TCG Specifications: https://trustedcomputinggroup.org/

---

**Published by:**
World Certification Industry Association (WIA)

**License:** CC BY-SA 4.0

**弘益人間 · Benefit All Humanity**
