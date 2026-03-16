# WIA-SEC-014: Hardware Security Module - Glossary

**Version:** 1.0
**Last Updated:** 2025-12-25

---

## A

**AES (Advanced Encryption Standard)**
Symmetric block cipher standardized by NIST (FIPS 197). Supports key sizes of 128, 192, and 256 bits. Used for data encryption, key wrapping, and message authentication.

**Algorithm**
Mathematical procedure for encryption, decryption, signing, or hashing. Must be approved by FIPS 140-2/140-3 for use in certified HSMs.

**API (Application Programming Interface)**
Set of functions and protocols for interacting with the HSM. WIA-SEC-014 supports PKCS#11 and REST APIs.

**Asymmetric Cryptography**
Cryptographic system using two mathematically related keys: public key (for encryption/verification) and private key (for decryption/signing). Examples: RSA, ECC.

**Attestation**
Process where HSM proves its identity and security properties using a cryptographically signed statement.

**Audit Log**
Tamper-evident record of all security-relevant events occurring within the HSM. Required for compliance and forensics.

**Authentication**
Process of verifying the identity of a user or system before granting access to HSM resources.

**Authorization**
Process of determining what operations an authenticated user is permitted to perform.

---

## B

**BB84**
Quantum key distribution protocol invented by Bennett and Brassard in 1984. Enables information-theoretically secure key exchange using quantum properties.

**Backup**
Process of creating secure copies of cryptographic keys for disaster recovery. Keys should be wrapped before export.

**Block Cipher**
Encryption algorithm that operates on fixed-size blocks of data (e.g., AES with 128-bit blocks).

**BLS12-381**
Pairing-friendly elliptic curve used in zero-knowledge proofs and blockchain applications.

---

## C

**CA (Certificate Authority)**
Trusted entity that issues digital certificates. HSMs protect the CA's signing keys.

**CCM (Counter with CBC-MAC)**
Authenticated encryption mode combining CTR mode encryption with CBC-MAC authentication.

**Certificate**
Digital document binding a public key to an identity, signed by a Certificate Authority.

**CKA (Cryptoki Attribute)**
PKCS#11 attribute defining properties of cryptographic objects. Examples: CKA_LABEL, CKA_SENSITIVE.

**CKM (Cryptoki Mechanism)**
PKCS#11 mechanism identifier specifying the cryptographic algorithm. Examples: CKM_RSA_PKCS, CKM_AES_GCM.

**CKR (Cryptoki Return)**
PKCS#11 return code indicating success or error. Example: CKR_OK (0x00000000).

**CMAC (Cipher-based Message Authentication Code)**
Message authentication code based on block cipher (e.g., AES-CMAC).

**Common Criteria**
International standard (ISO/IEC 15408) for computer security certification. HSMs often target EAL4+ assurance level.

**Crypto Officer (CO)**
HSM role responsible for key management and cryptographic policy configuration.

**Cryptographic Boundary**
Physical or logical perimeter within which cryptographic operations occur and keys are protected.

**CTR (Counter Mode)**
Block cipher mode of operation that converts a block cipher into a stream cipher using a counter.

---

## D

**DER (Distinguished Encoding Rules)**
Binary encoding format for ASN.1 data structures, used for certificates and keys.

**Dilithium**
Post-quantum digital signature algorithm selected by NIST (FIPS 204). Based on lattice cryptography.

**DKG (Distributed Key Generation)**
Protocol allowing multiple parties to jointly generate a key without any single party knowing the complete key.

**DRBG (Deterministic Random Bit Generator)**
Cryptographically secure pseudorandom number generator. NIST SP 800-90A specifies approved DRBGs.

---

## E

**EAL (Evaluation Assurance Level)**
Security evaluation level in Common Criteria, ranging from EAL1 (lowest) to EAL7 (highest). HSMs typically target EAL4+.

**ECC (Elliptic Curve Cryptography)**
Public-key cryptography based on elliptic curves. Provides equivalent security to RSA with smaller key sizes.

**ECDH (Elliptic Curve Diffie-Hellman)**
Key agreement protocol using elliptic curves. Allows two parties to establish a shared secret.

**ECDSA (Elliptic Curve Digital Signature Algorithm)**
Digital signature algorithm using elliptic curves. Standardized in FIPS 186-4.

**EdDSA (Edwards-curve Digital Signature Algorithm)**
Modern signature algorithm using twisted Edwards curves (e.g., Ed25519, Ed448).

**Entropy**
Measure of randomness. High-quality entropy is essential for secure key generation.

---

## F

**Federated HSM**
Network of HSMs from different organizations that coordinate for multi-party operations.

**FHE (Fully Homomorphic Encryption)**
Encryption scheme allowing arbitrary computations on encrypted data without decryption.

**FIPS 140-2/140-3**
Federal Information Processing Standard specifying security requirements for cryptographic modules.

**FIPS Mode**
Operating mode where only FIPS-approved algorithms and key sizes are permitted.

**Firmware**
Software embedded in HSM hardware. Must be cryptographically signed and verified.

---

## G

**GCM (Galois/Counter Mode)**
Authenticated encryption mode providing both confidentiality and authenticity. Widely used with AES.

**GROTH16**
Efficient zero-knowledge proof system commonly used in blockchain applications.

---

## H

**HKDF (HMAC-based Key Derivation Function)**
Key derivation function based on HMAC, standardized in RFC 5869.

**HMAC (Hash-based Message Authentication Code)**
Message authentication code using a cryptographic hash function. Examples: HMAC-SHA256.

**HSM (Hardware Security Module)**
Tamper-resistant hardware device that generates, stores, and protects cryptographic keys.

**Hybrid Cryptography**
Combining classical and post-quantum cryptographic algorithms for defense against future quantum computers.

---

## I

**Initialization**
Process of setting up an HSM for first use, including setting SO PIN and generating master keys.

**IV (Initialization Vector)**
Random value used with certain encryption modes (e.g., CBC, GCM) to ensure identical plaintexts produce different ciphertexts.

---

## J

**JWT (JSON Web Token)**
Compact token format for securely transmitting information between parties. Can be signed using HSM-protected keys.

---

## K

**KDF (Key Derivation Function)**
Function that derives one or more keys from a master key or password. Examples: HKDF, PBKDF2.

**KEM (Key Encapsulation Mechanism)**
Public-key encryption scheme specifically designed for encrypting symmetric keys. Kyber is a post-quantum KEM.

**Key Ceremony**
Formal, audited process for generating and initializing high-value cryptographic keys (e.g., CA root keys).

**Key Handle**
Identifier or reference to a key object stored in the HSM. Used in API calls instead of the actual key material.

**Key Wrapping**
Encrypting a cryptographic key with another key (KEK) for secure storage or transmission.

**KMIP (Key Management Interoperability Protocol)**
Standard protocol (OASIS) for enterprise key management across different systems.

**Kyber**
Post-quantum key encapsulation mechanism selected by NIST (FIPS 203). Based on lattice cryptography.

---

## L

**Lattice Cryptography**
Post-quantum cryptographic approach based on hard mathematical problems in lattices. Basis for Kyber and Dilithium.

**Login**
Authentication process to establish an HSM session with specific user privileges.

---

## M

**M-of-N**
Threshold scheme requiring M out of N key shares to reconstruct a key. Used for secure key recovery.

**MAC (Message Authentication Code)**
Short piece of information used to authenticate a message. Examples: HMAC, CMAC.

**Master Key**
High-level key used to encrypt or derive other keys. Also called KEK (Key Encryption Key).

**Mechanism**
In PKCS#11, specification of a cryptographic algorithm and its parameters.

**MEK (Master Encryption Key)**
Master key used for encrypting data or other keys.

**MGF (Mask Generation Function)**
Function used in RSA-OAEP and RSA-PSS. Typically MGF1 with a hash function.

**MSK (Master Signing Key)**
Master key used for digital signatures.

**Multi-Tenancy**
Architecture allowing multiple isolated tenants (organizations) to share the same HSM hardware.

**MWK (Master Wrapping Key)**
Master key used specifically for wrapping (encrypting) other keys for export.

---

## N

**Nonce**
Number used once. Random or counter value used in cryptographic operations to prevent replay attacks.

**NIST (National Institute of Standards and Technology)**
US government agency that publishes cryptographic standards including FIPS and Special Publications.

---

## O

**OAEP (Optimal Asymmetric Encryption Padding)**
Padding scheme for RSA encryption (RSA-OAEP). More secure than PKCS#1 v1.5 padding.

**Object**
In PKCS#11, a data item stored in the HSM such as a key, certificate, or arbitrary data.

**Object Handle**
Unique identifier for a PKCS#11 object within a session.

---

## P

**P-256, P-384, P-521**
NIST-standardized elliptic curves. Numbers indicate approximate bit security levels.

**PBKDF2 (Password-Based Key Derivation Function 2)**
Function for deriving keys from passwords using salting and iteration.

**PIN (Personal Identification Number)**
Secret value used for user authentication to the HSM.

**PKCS (Public-Key Cryptography Standards)**
Series of standards published by RSA Labs. PKCS#11 defines the Cryptoki API for HSMs.

**PKCS#1**
Standard for RSA cryptography, including signature and encryption schemes.

**PKCS#11 (Cryptoki)**
API standard for communicating with cryptographic tokens and HSMs.

**Post-Quantum Cryptography (PQC)**
Cryptographic algorithms resistant to attacks by quantum computers. Examples: Kyber, Dilithium, SPHINCS+.

**Private Key**
Secret key in asymmetric cryptography. Used for decryption and signature generation. Must never leave HSM.

**PSS (Probabilistic Signature Scheme)**
RSA signature scheme with stronger security properties than PKCS#1 v1.5.

**Public Key**
Non-secret key in asymmetric cryptography. Used for encryption and signature verification. Can be freely distributed.

---

## Q

**QKD (Quantum Key Distribution)**
Method of secure key distribution using quantum mechanical properties. Provides information-theoretic security.

**Quantum Computer**
Computer using quantum mechanics to perform computations. Threatens classical cryptography (RSA, ECC).

---

## R

**RBAC (Role-Based Access Control)**
Access control model assigning permissions based on user roles (e.g., SO, CO, User).

**Recovery**
Process of restoring keys from backup, typically requiring M-of-N shares.

**Revocation**
Process of marking a key or certificate as no longer valid.

**RNG (Random Number Generator)**
Device or algorithm that generates random numbers. HSMs use hardware TRNGs.

**Root Key**
Top-level key in a key hierarchy. Compromise of root key compromises all derived keys.

**RSA**
Public-key cryptography algorithm based on factoring large numbers. Named after Rivest, Shamir, Adleman.

---

## S

**Salt**
Random data added to inputs before hashing or key derivation to prevent dictionary attacks.

**secp256k1**
Elliptic curve used in Bitcoin and Ethereum. Not a NIST curve.

**Security Officer (SO)**
HSM role responsible for initialization, user management, and security policy.

**Sensitive**
PKCS#11 attribute (CKA_SENSITIVE) indicating a key value cannot be read from the HSM.

**Session**
Logical connection between an application and the HSM. Required for performing operations.

**SHA (Secure Hash Algorithm)**
Family of cryptographic hash functions. Current standard is SHA-2 (SHA-256, SHA-384, SHA-512).

**SHA-3**
Cryptographic hash function family based on Keccak algorithm. Alternative to SHA-2.

**Shamir's Secret Sharing**
Algorithm for splitting a secret into N shares such that M shares are needed to reconstruct it.

**Signature**
Cryptographic value proving that data was signed by the holder of a private key.

**Slot**
Physical or logical HSM device. PKCS#11 organizes HSMs as slots containing tokens.

**SO (Security Officer)**
See Security Officer.

**SPHINCS+**
Stateless hash-based post-quantum signature algorithm selected by NIST (FIPS 205).

**Symmetric Cryptography**
Cryptography using the same key for encryption and decryption. Examples: AES, 3DES.

---

## T

**Tamper Evidence**
Design features that make it obvious when physical tampering has occurred.

**Tamper Resistance**
Design features that make physical tampering difficult.

**Tamper Response**
Actions taken when tampering is detected (e.g., key zeroization, alarm).

**TBS (To Be Signed)**
Data structure that will be signed, typically in X.509 certificate generation.

**Threshold Cryptography**
Cryptographic protocols requiring M-of-N participants to perform operations.

**TLS (Transport Layer Security)**
Protocol for secure network communication. HSMs can protect TLS private keys.

**Token**
PKCS#11 term for a logical view of a cryptographic device within a slot.

**TRNG (True Random Number Generator)**
Hardware device generating true random numbers from physical phenomena.

---

## U

**Unwrap**
Decrypting a wrapped (encrypted) key and importing it into the HSM.

**User**
Standard HSM role for performing cryptographic operations with authorized keys.

---

## V

**VC (Verifiable Credential)**
Tamper-evident credential in W3C standard format. Can be signed using HSM-protected keys.

**Verification**
Process of checking a digital signature's validity using the corresponding public key.

**Verify**
PKCS#11 operation for signature verification.

---

## W

**Wrap**
Encrypting a key with another key (KEK) for secure export or backup.

**Wrapping Key**
Key used specifically for wrapping (encrypting) other keys. Also called KEK.

---

## X

**X.509**
ITU standard for public key certificates. Used in TLS, S/MIME, code signing.

---

## Z

**Zeroization**
Secure deletion of cryptographic keys by overwriting memory with zeros or random data.

**Zero-Knowledge Proof (ZKP)**
Cryptographic protocol proving knowledge of information without revealing the information itself.

**zk-SNARK (Zero-Knowledge Succinct Non-Interactive Argument of Knowledge)**
Compact zero-knowledge proof that can be verified quickly. Used in privacy-preserving blockchains.

---

## Acronyms Quick Reference

| Acronym | Full Name |
|---------|-----------|
| AES | Advanced Encryption Standard |
| API | Application Programming Interface |
| CA | Certificate Authority |
| CCM | Counter with CBC-MAC |
| CKA | Cryptoki Attribute |
| CKM | Cryptoki Mechanism |
| CKR | Cryptoki Return |
| CMAC | Cipher-based Message Authentication Code |
| CO | Crypto Officer |
| DER | Distinguished Encoding Rules |
| DKG | Distributed Key Generation |
| DRBG | Deterministic Random Bit Generator |
| EAL | Evaluation Assurance Level |
| ECC | Elliptic Curve Cryptography |
| ECDH | Elliptic Curve Diffie-Hellman |
| ECDSA | Elliptic Curve Digital Signature Algorithm |
| EdDSA | Edwards-curve Digital Signature Algorithm |
| FHE | Fully Homomorphic Encryption |
| FIPS | Federal Information Processing Standard |
| GCM | Galois/Counter Mode |
| HKDF | HMAC-based Key Derivation Function |
| HMAC | Hash-based Message Authentication Code |
| HSM | Hardware Security Module |
| IV | Initialization Vector |
| JWT | JSON Web Token |
| KDF | Key Derivation Function |
| KEK | Key Encryption Key |
| KEM | Key Encapsulation Mechanism |
| KMIP | Key Management Interoperability Protocol |
| MAC | Message Authentication Code |
| MEK | Master Encryption Key |
| MGF | Mask Generation Function |
| MSK | Master Signing Key |
| MWK | Master Wrapping Key |
| NIST | National Institute of Standards and Technology |
| OAEP | Optimal Asymmetric Encryption Padding |
| PBKDF2 | Password-Based Key Derivation Function 2 |
| PIN | Personal Identification Number |
| PKCS | Public-Key Cryptography Standards |
| PQC | Post-Quantum Cryptography |
| PSS | Probabilistic Signature Scheme |
| QKD | Quantum Key Distribution |
| RBAC | Role-Based Access Control |
| RNG | Random Number Generator |
| RSA | Rivest-Shamir-Adleman |
| SHA | Secure Hash Algorithm |
| SO | Security Officer |
| TBS | To Be Signed |
| TLS | Transport Layer Security |
| TRNG | True Random Number Generator |
| VC | Verifiable Credential |
| ZKP | Zero-Knowledge Proof |
| zk-SNARK | Zero-Knowledge Succinct Non-Interactive Argument of Knowledge |

---

## Key Size Recommendations

| Algorithm | Minimum (Legacy) | Recommended | High Security |
|-----------|-----------------|-------------|---------------|
| RSA | 2048 bits | 3072 bits | 4096 bits |
| ECC | 256 bits (P-256) | 384 bits (P-384) | 521 bits (P-521) |
| AES | 128 bits | 256 bits | 256 bits |
| SHA | SHA-256 | SHA-384 | SHA-512 |

**Note:** As of 2025, SHA-1 and RSA-1024 are deprecated and should not be used.

---

## Standards References

| Standard | Title | Organization |
|----------|-------|--------------|
| FIPS 140-2 | Security Requirements for Cryptographic Modules | NIST |
| FIPS 140-3 | Security Requirements for Cryptographic Modules (2019) | NIST |
| FIPS 186-4 | Digital Signature Standard (DSS) | NIST |
| FIPS 197 | Advanced Encryption Standard (AES) | NIST |
| FIPS 198-1 | The Keyed-Hash Message Authentication Code (HMAC) | NIST |
| FIPS 202 | SHA-3 Standard | NIST |
| FIPS 203 | Module-Lattice-Based Key-Encapsulation (ML-KEM / Kyber) | NIST |
| FIPS 204 | Module-Lattice-Based Digital Signature (ML-DSA / Dilithium) | NIST |
| FIPS 205 | Stateless Hash-Based Digital Signature (SLH-DSA / SPHINCS+) | NIST |
| ISO 15408 | Common Criteria for Information Technology Security Evaluation | ISO/IEC |
| PKCS#11 v2.40 | Cryptographic Token Interface Standard | OASIS |
| RFC 3394 | Advanced Encryption Standard (AES) Key Wrap Algorithm | IETF |
| RFC 5869 | HMAC-based Extract-and-Expand Key Derivation Function (HKDF) | IETF |
| SP 800-57 | Recommendation for Key Management | NIST |
| SP 800-90A | Recommendation for Random Number Generation Using DRBGs | NIST |
| X.509 | Public Key Infrastructure Certificate Format | ITU-T |

---

**Document Control:**
- Version: 1.0
- Status: Official Standard
- Maintained by: WIA Security Working Group

**弘益人間 (Benefit All Humanity)**

© 2025 WIA (World Certification Industry Association)
