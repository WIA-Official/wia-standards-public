# WIA-SEC-011: Data Encryption Standard
## GLOSSARY

---

## A

**AAD (Additional Authenticated Data)**
- Data that is authenticated but not encrypted in AEAD schemes
- Provides integrity protection without confidentiality
- Example: Header information in network packets

**AEAD (Authenticated Encryption with Associated Data)**
- Encryption mode providing both confidentiality and authenticity
- Combines encryption and MAC in a single operation
- Examples: AES-GCM, ChaCha20-Poly1305

**AES (Advanced Encryption Standard)**
- Symmetric block cipher standardized by NIST in 2001
- Supports key sizes: 128, 192, and 256 bits
- Block size: 128 bits
- Replaces DES as the standard encryption algorithm

**Algorithm**
- Step-by-step procedure for solving a problem
- In cryptography: mathematical formula for encryption/decryption
- Must be publicly known (Kerckhoffs's principle)

**Argon2**
- Modern password hashing algorithm
- Winner of Password Hashing Competition (2015)
- Resistant to GPU and ASIC attacks
- Variants: Argon2d, Argon2i, Argon2id (recommended)

**Asymmetric Encryption**
- Uses different keys for encryption and decryption
- Public key encrypts, private key decrypts
- Also called public-key cryptography
- Examples: RSA, ECC

**Authentication Tag**
- Cryptographic checksum in AEAD schemes
- Verifies data integrity and authenticity
- Typical size: 128 bits (16 bytes)
- Must be validated in constant time

---

## B

**Base64**
- Encoding scheme converting binary data to ASCII text
- Uses 64 characters: A-Z, a-z, 0-9, +, /
- Common in email, JSON, XML
- Increases size by ~33%

**Block Cipher**
- Encrypts fixed-size blocks of data
- Common block size: 128 bits
- Requires padding for data not matching block size
- Examples: AES, DES, 3DES

**BLAKE2**
- Fast cryptographic hash function
- Faster than SHA-3, SHA-2, and MD5
- Variants: BLAKE2b (64-bit), BLAKE2s (32-bit)
- Secure and widely used

---

## C

**CBC (Cipher Block Chaining)**
- Block cipher mode of operation
- Each block depends on previous blocks
- Requires IV for first block
- ⚠️ Not authenticated - use with HMAC or prefer GCM

**Certificate**
- Digital document binding public key to identity
- Issued by Certificate Authority (CA)
- Used in TLS/SSL, code signing
- Format: X.509

**ChaCha20**
- Stream cipher designed by Daniel J. Bernstein
- Fast in software (no hardware requirements)
- Often paired with Poly1305 MAC
- Used in TLS 1.3, mobile apps

**Cipher**
- Algorithm for encryption and decryption
- Two types: block ciphers and stream ciphers
- Modern ciphers: AES, ChaCha20

**Ciphertext**
- Encrypted data
- Appears random and unintelligible
- Can be safely transmitted over insecure channels
- Opposite of plaintext

**Confidentiality**
- Property ensuring data is accessible only to authorized parties
- Achieved through encryption
- One of the three pillars of information security (CIA triad)

**CSRNG (Cryptographically Secure Random Number Generator)**
- Random number generator suitable for cryptography
- Must be unpredictable and unbiased
- Examples: /dev/urandom (Linux), crypto.randomBytes (Node.js)

**CTR (Counter Mode)**
- Block cipher mode that creates a stream cipher
- Can be parallelized (fast)
- Requires unique nonce for each encryption
- Often used with authentication (CTR+HMAC or GCM)

---

## D

**DEK (Data Encryption Key)**
- Key used to encrypt actual data
- In envelope encryption, DEK is encrypted by KEK
- Should be unique per data object
- Typically AES-256 key (32 bytes)

**DES (Data Encryption Standard)**
- ⚠️ Obsolete symmetric cipher (1977-2001)
- 56-bit key (too small, vulnerable to brute force)
- Replaced by AES
- Still found in legacy systems

**Deterministic Encryption**
- Same plaintext always produces same ciphertext
- Allows searching encrypted data
- ⚠️ Less secure than randomized encryption
- Use only when searchability is required

**Diffie-Hellman (DH)**
- Key exchange protocol (1976)
- Allows two parties to establish shared secret over insecure channel
- Foundation of modern key exchange
- Used in TLS, VPNs

**Digital Signature**
- Cryptographic proof of authenticity and integrity
- Created using private key
- Verified using public key
- Provides non-repudiation

---

## E

**ECC (Elliptic Curve Cryptography)**
- Asymmetric cryptography based on elliptic curves
- Smaller keys than RSA for equivalent security
- Faster computation
- Common curves: P-256, P-384, P-521

**ECDH (Elliptic Curve Diffie-Hellman)**
- Key exchange using elliptic curves
- More efficient than traditional DH
- Used in TLS 1.3, Signal Protocol
- Perfect Forward Secrecy (PFS)

**ECDSA (Elliptic Curve Digital Signature Algorithm)**
- Digital signature algorithm using ECC
- Smaller signatures than RSA
- Used in Bitcoin, Ethereum
- Fast verification

**Encryption**
- Process of converting plaintext to ciphertext
- Requires algorithm and key
- Protects confidentiality
- Should be combined with authentication

**Entropy**
- Measure of randomness
- Critical for key generation
- Insufficient entropy = weak keys
- Sources: hardware RNG, OS entropy pool

**Envelope Encryption**
- Encrypting data with DEK, then encrypting DEK with KEK
- Adds security layer
- Enables key rotation without re-encrypting data
- Used by cloud providers (AWS, Azure, GCP)

---

## F

**FIPS 140-2/140-3**
- US government standard for cryptographic modules
- Four security levels (1-4)
- Required for government systems
- Certifies hardware and software implementations

**Forward Secrecy (Perfect Forward Secrecy)**
- Property ensuring past sessions remain secure even if long-term keys compromised
- Requires ephemeral (temporary) keys
- Implemented using ECDHE or DHE
- Critical for messaging apps, TLS

---

## G

**GCM (Galois/Counter Mode)**
- AEAD mode combining CTR encryption and GMAC authentication
- Fast and secure
- Parallelizable (hardware acceleration)
- Recommended mode for AES

**GMAC (Galois Message Authentication Code)**
- Authentication component of GCM
- Based on universal hashing
- Fast in hardware
- Provides authenticity and integrity

---

## H

**Hash Function**
- One-way function mapping arbitrary data to fixed-size output
- Properties: deterministic, fast, avalanche effect, collision-resistant
- Examples: SHA-256, SHA-3, BLAKE2
- Uses: integrity checking, password hashing, digital signatures

**HKDF (HMAC-based Key Derivation Function)**
- Derives keys from input key material
- Two-step: extract then expand
- Based on HMAC
- Recommended by NIST (SP 800-56C)

**HMAC (Hash-based Message Authentication Code)**
- MAC using cryptographic hash function
- Provides authenticity and integrity
- Requires shared secret key
- Example: HMAC-SHA256

**Homomorphic Encryption**
- Allows computations on encrypted data
- Result decrypts to same value as if computed on plaintext
- Types: Partially Homomorphic (PHE), Fully Homomorphic (FHE)
- Use cases: cloud computing, privacy-preserving analytics

**HSM (Hardware Security Module)**
- Tamper-resistant hardware device for key management
- Stores keys, performs crypto operations
- FIPS 140-2 Level 2+ certified
- Used in banks, CAs, enterprises

---

## I

**Integrity**
- Property ensuring data has not been modified
- Achieved through MACs, digital signatures, hashes
- One of the three pillars of information security (CIA triad)

**IV (Initialization Vector)**
- Random value used to initialize encryption
- Must be unique for each encryption with same key
- Can be public (transmitted with ciphertext)
- Critical for security of many cipher modes

---

## J

**JSON Web Encryption (JWE)**
- Standard for encrypting JSON data
- Compact, URL-safe format
- Supports multiple algorithms
- Used in OAuth 2.0, OpenID Connect

**JSON Web Token (JWT)**
- Compact, URL-safe token format
- Can be signed (JWS) or encrypted (JWE)
- Used for authentication and authorization
- Format: header.payload.signature

---

## K

**KEK (Key Encryption Key)**
- Key used to encrypt other keys
- In envelope encryption, encrypts DEK
- Often stored in HSM or KMS
- Typically RSA or AES key

**Kerckhoffs's Principle**
- "A cryptosystem should be secure even if everything about the system, except the key, is public knowledge"
- Foundation of modern cryptography
- Algorithm can be open source
- Security depends only on key secrecy

**Key**
- Secret parameter used in cryptographic algorithm
- Must be random and unpredictable
- Size determines security level
- Must be protected at all costs

**Key Derivation Function (KDF)**
- Function deriving keys from secret value
- Examples: HKDF, PBKDF2, Argon2
- Used for password-based encryption
- Adds computational cost to slow brute-force

**Key Exchange**
- Process of securely sharing keys between parties
- Methods: Diffie-Hellman, RSA key transport
- Critical for establishing secure communication
- Should provide forward secrecy

**Key Management**
- Lifecycle management of cryptographic keys
- Includes: generation, distribution, storage, rotation, revocation
- Most critical aspect of cryptography
- Often weakest link in security

**Key Rotation**
- Process of replacing old keys with new keys
- Recommended schedule: 90-365 days
- Re-encrypts data with new key
- Limits impact of key compromise

**Key Size**
- Length of cryptographic key in bits
- Determines security level
- Recommendations: AES-256 (256-bit), RSA-4096 (4096-bit), ECC P-384 (384-bit)

**KMS (Key Management Service)**
- Cloud service for managing encryption keys
- Centralized key storage and access control
- Automated rotation and logging
- Examples: AWS KMS, Azure Key Vault, Google Cloud KMS

---

## L

**Lattice-Based Cryptography**
- Post-quantum cryptography based on lattice problems
- Resistant to quantum attacks
- Examples: CRYSTALS-Kyber, CRYSTALS-Dilithium
- NIST selected for standardization

---

## M

**MAC (Message Authentication Code)**
- Cryptographic checksum verifying authenticity and integrity
- Requires shared secret key
- Examples: HMAC, GMAC, Poly1305
- Different from hash (requires key)

**Master Key**
- Top-level key in key hierarchy
- Used to derive or encrypt other keys
- Must be protected with highest security
- Often stored in HSM

**MGF (Mask Generation Function)**
- Function used in RSA-OAEP padding
- Typically MGF1 with SHA-256
- Expands variable-length input to desired length

**Mode of Operation**
- Method of applying block cipher to data larger than block size
- Secure modes: GCM, CCM
- ⚠️ Insecure without authentication: ECB, CBC (alone)
- Choice affects security and performance

---

## N

**NIST (National Institute of Standards and Technology)**
- US agency developing cryptographic standards
- Published AES, SHA-3, post-quantum algorithms
- FIPS publications widely adopted globally

**Non-repudiation**
- Property preventing denial of action
- Achieved through digital signatures
- Proof of origin and integrity
- Important for legal and financial transactions

**Nonce (Number Used Once)**
- Value that must be unique for each encryption
- Can be public
- Critical for security of CTR, GCM modes
- Reuse with same key catastrophic

---

## O

**OAEP (Optimal Asymmetric Encryption Padding)**
- Padding scheme for RSA encryption
- Adds randomness and prevents attacks
- Standard: PKCS#1 v2.0+
- Recommended over PKCS#1 v1.5

**OpenSSL**
- Open-source cryptography library
- Implements SSL/TLS, cryptographic algorithms
- Used by many applications and languages
- Command-line tools and C library

---

## P

**Padding**
- Adding data to reach required block size
- Schemes: PKCS#7, OAEP, PSS
- Must be handled carefully to prevent attacks
- Not needed in stream ciphers or CTR mode

**PBKDF2 (Password-Based Key Derivation Function 2)**
- KDF for deriving keys from passwords
- Uses many iterations to slow brute-force
- Minimum 100,000 iterations recommended
- Superseded by Argon2 but still widely used

**PEM (Privacy-Enhanced Mail)**
- File format for cryptographic keys and certificates
- Base64-encoded with headers
- Example: "-----BEGIN CERTIFICATE-----"
- Common in TLS/SSL

**Perfect Forward Secrecy (PFS)**
- See Forward Secrecy

**PKCS (Public-Key Cryptography Standards)**
- Set of standards for cryptography
- Maintained by RSA Security LLC
- Examples: PKCS#1 (RSA), PKCS#8 (private keys), PKCS#11 (HSM interface)

**Plaintext**
- Unencrypted data
- Input to encryption algorithm
- Output from decryption algorithm
- Opposite of ciphertext

**Poly1305**
- Fast MAC designed by Daniel J. Bernstein
- Often paired with ChaCha20
- 128-bit security
- Used in TLS 1.3, mobile apps

**Post-Quantum Cryptography (PQC)**
- Algorithms resistant to quantum computer attacks
- NIST selected: Kyber, Dilithium, SPHINCS+
- Needed as quantum computers advance
- Hybrid approach recommended (classical + PQC)

**Private Key**
- Secret key in asymmetric cryptography
- Used for decryption and signing
- Must be kept confidential
- Compromise = total security failure

**Public Key**
- Non-secret key in asymmetric cryptography
- Used for encryption and signature verification
- Can be freely distributed
- Mathematically related to private key

**Public Key Infrastructure (PKI)**
- Framework for managing digital certificates
- Includes CAs, certificates, key management
- Used for TLS/SSL, email encryption, code signing

---

## Q

**Quantum Computing**
- Computing using quantum mechanics
- Threatens current asymmetric cryptography
- Shor's algorithm breaks RSA, ECC
- AES-256 still secure (with doubled key size conceptually)

**Quantum Key Distribution (QKD)**
- Key exchange using quantum mechanics
- Theoretically unbreakable
- Detects eavesdropping
- Limited to short distances currently

---

## R

**Random Number Generator (RNG)**
- System generating random numbers
- Cryptography requires CSRNG
- Poor RNG = weak keys
- Sources: hardware noise, OS entropy pool

**Replay Attack**
- Attacker retransmits valid data
- Prevented by: timestamps, nonces, sequence numbers
- Common in authentication systems

**RSA (Rivest-Shamir-Adleman)**
- First practical public-key cryptosystem (1977)
- Based on factoring large numbers
- Key sizes: 2048-bit (minimum), 4096-bit (recommended)
- ⚠️ Vulnerable to quantum computers

---

## S

**Salt**
- Random data added to password before hashing
- Prevents rainbow table attacks
- Should be unique per password
- Stored alongside hash

**Secure Enclave**
- Isolated processor for cryptographic operations
- Found in Apple devices, some Android phones
- Protects keys from main OS
- Similar to HSM but integrated in device

**SHA (Secure Hash Algorithm)**
- Family of cryptographic hash functions
- SHA-256, SHA-384, SHA-512 (SHA-2 family)
- SHA-3 (Keccak) - latest standard
- ⚠️ SHA-1 deprecated (collision attacks)

**Shamir's Secret Sharing**
- Splits secret into shares
- Requires threshold of shares to reconstruct
- Example: 3-of-5 (need any 3 of 5 shares)
- Used in key management, multi-party computation

**Side-Channel Attack**
- Attacks based on implementation, not algorithm
- Types: timing, power analysis, electromagnetic
- Countermeasures: constant-time operations, masking
- Harder to prevent than algorithmic attacks

**Signature**
- See Digital Signature

**Stream Cipher**
- Encrypts data one bit/byte at a time
- Examples: ChaCha20, RC4 (⚠️ broken)
- Faster than block ciphers
- Requires unique key or nonce

**Symmetric Encryption**
- Uses same key for encryption and decryption
- Fast and efficient
- Key distribution problem
- Examples: AES, ChaCha20

---

## T

**TEE (Trusted Execution Environment)**
- Secure area in processor
- Isolated from main OS
- Examples: Intel SGX, AMD SEV, ARM TrustZone
- Used for confidential computing

**Threshold Cryptography**
- Distributes cryptographic operations among multiple parties
- Requires threshold to complete operation
- No single point of failure
- Used in blockchain, secure storage

**TLS (Transport Layer Security)**
- Protocol for secure communication over networks
- Successor to SSL
- Current version: TLS 1.3
- Used in HTTPS, email, VPNs

**Token**
- Representation of authentication or authorization
- Can be encrypted (JWE) or signed (JWS)
- Examples: JWT, SAML token
- Stateless alternative to sessions

---

## V

**Verifiable Credential (VC)**
- Digital credential with cryptographic proof
- W3C standard
- Self-sovereign identity
- Use cases: diplomas, licenses, medical records

---

## W

**W3C (World Wide Web Consortium)**
- International standards organization
- Published VC, DID standards
- Web Crypto API specification

**Web Crypto API**
- JavaScript API for cryptographic operations
- Available in browsers
- Provides: hash, sign, encrypt, key generation
- Secure but limited compared to native libraries

---

## X

**X.509**
- Standard format for digital certificates
- Used in TLS/SSL, code signing
- Contains: public key, identity, CA signature
- Version 3 is current standard

**XTS (XEX-based Tweaked-codebook mode)**
- Block cipher mode for disk encryption
- Used in BitLocker, FileVault
- Designed for random-access storage
- Example: AES-256-XTS

---

## Z

**Zero-Knowledge Proof (ZKP)**
- Proves knowledge without revealing information
- Types: interactive, non-interactive (zk-SNARKs, zk-STARKs)
- Use cases: privacy-preserving authentication, blockchain
- Cutting-edge cryptography

**zk-SNARK (Zero-Knowledge Succinct Non-Interactive Argument of Knowledge)**
- Type of zero-knowledge proof
- Succinct: small proof size
- Non-interactive: no back-and-forth
- Used in Zcash, other privacy coins

---

## Acronyms Quick Reference

| Acronym | Full Name |
|---------|-----------|
| AAD | Additional Authenticated Data |
| AEAD | Authenticated Encryption with Associated Data |
| AES | Advanced Encryption Standard |
| CA | Certificate Authority |
| CBC | Cipher Block Chaining |
| CTR | Counter Mode |
| DEK | Data Encryption Key |
| DES | Data Encryption Standard |
| DH | Diffie-Hellman |
| ECC | Elliptic Curve Cryptography |
| ECDH | Elliptic Curve Diffie-Hellman |
| ECDSA | Elliptic Curve Digital Signature Algorithm |
| FIPS | Federal Information Processing Standards |
| GCM | Galois/Counter Mode |
| GMAC | Galois Message Authentication Code |
| HKDF | HMAC-based Key Derivation Function |
| HMAC | Hash-based Message Authentication Code |
| HSM | Hardware Security Module |
| IV | Initialization Vector |
| JWE | JSON Web Encryption |
| JWS | JSON Web Signature |
| JWT | JSON Web Token |
| KDF | Key Derivation Function |
| KEK | Key Encryption Key |
| KMS | Key Management Service |
| MAC | Message Authentication Code |
| MGF | Mask Generation Function |
| NIST | National Institute of Standards and Technology |
| OAEP | Optimal Asymmetric Encryption Padding |
| PBKDF2 | Password-Based Key Derivation Function 2 |
| PEM | Privacy-Enhanced Mail |
| PFS | Perfect Forward Secrecy |
| PKI | Public Key Infrastructure |
| PKCS | Public-Key Cryptography Standards |
| PQC | Post-Quantum Cryptography |
| QKD | Quantum Key Distribution |
| RNG | Random Number Generator |
| RSA | Rivest-Shamir-Adleman |
| SHA | Secure Hash Algorithm |
| TEE | Trusted Execution Environment |
| TLS | Transport Layer Security |
| VC | Verifiable Credential |
| W3C | World Wide Web Consortium |
| XTS | XEX-based Tweaked-codebook mode |
| ZKP | Zero-Knowledge Proof |

---

**弘益人間 (홍익인간)** - Benefit All Humanity

© 2025 SmileStory Inc. / WIA
