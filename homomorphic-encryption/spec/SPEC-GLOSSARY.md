# WIA-SEC-012: Homomorphic Encryption Standard
## GLOSSARY OF TERMS

**Standard ID:** WIA-SEC-012
**Version:** 1.0.0
**Last Updated:** 2025-12-25

---

## A

**Additive Homomorphism**
A property where encryption of the sum equals the product of encryptions:
Enc(a + b) = Enc(a) × Enc(b)

**Approximate Arithmetic**
Computations that produce results within a certain precision, as used in CKKS scheme for real/complex numbers.

**Asymmetric Encryption**
Encryption using different keys for encryption (public key) and decryption (secret key).

**Authenticated Encryption**
Encryption that provides both confidentiality and authenticity verification.

---

## B

**Batching**
The technique of encoding multiple values into a single plaintext/ciphertext, enabling SIMD-style parallel operations.

**BFV (Brakerski-Fan-Vercauteren)**
A fully homomorphic encryption scheme supporting exact integer arithmetic.

**BGV (Brakerski-Gentry-Vaikuntanathan)**
An FHE scheme similar to BFV, predecessor to the BFV scheme.

**Bootstrapping**
The process of refreshing a ciphertext to reduce its noise, enabling unlimited computation depth in FHE.

**BKZ (Block Korkine-Zolotarev)**
A lattice reduction algorithm used in analyzing the security of lattice-based cryptography.

---

## C

**CKKS (Cheon-Kim-Kim-Song)**
A fully homomorphic encryption scheme optimized for approximate arithmetic on real/complex numbers.

**Ciphertext**
Data in its encrypted form, produced by applying an encryption algorithm to plaintext.

**Circuit**
A representation of computation as a series of addition and multiplication operations.

**Circuit Privacy**
Property ensuring that ciphertexts don't leak information about the computation performed on them.

**Coefficient Modulus**
Parameter q in RLWE-based schemes, determining ciphertext modulus and noise budget.

**CryptoNets**
Neural network architecture designed for efficient evaluation on encrypted data.

---

## D

**Decryption**
The process of converting ciphertext back to plaintext using a secret key.

**DGK (Damgård-Geisler-Krøigaard)**
A protocol for secure comparison of encrypted values.

**Discrete Gaussian Distribution**
A probability distribution commonly used for sampling error/noise in lattice cryptography.

**Distributed Key Generation**
Protocol for generating cryptographic keys across multiple parties without a trusted dealer.

---

## E

**ElGamal Encryption**
A public-key cryptosystem with multiplicative homomorphic properties.

**Encryption**
The process of converting plaintext to ciphertext using an encryption algorithm and key.

**Error Distribution**
The probability distribution from which noise is sampled in RLWE-based encryption.

**Evaluation Key**
Generic term for keys enabling homomorphic operations (relinearization keys, Galois keys, etc.).

---

## F

**Federated Learning**
Machine learning approach where models are trained across decentralized data sources.

**FHE (Fully Homomorphic Encryption)**
Encryption supporting arbitrary computation (both addition and multiplication) on encrypted data.

**FHEW**
A fully homomorphic encryption scheme optimized for fast bootstrapping.

**FPGA (Field-Programmable Gate Array)**
Hardware that can be configured for custom HE acceleration.

---

## G

**Galois Automorphism**
Mathematical operation enabling rotation of encrypted vectors in RLWE schemes.

**Galois Keys**
Evaluation keys enabling rotation operations on encrypted data.

**Garbled Circuits**
Cryptographic protocol for secure function evaluation, sometimes combined with HE.

**GPU Acceleration**
Using graphics processing units to speed up polynomial operations in HE.

---

## H

**Hardware Security Module (HSM)**
Physical device providing secure key storage and cryptographic operations.

**Homomorphic Encryption (HE)**
Encryption allowing computation on encrypted data without decryption.

**Homomorphism**
Mathematical property where operations on encrypted values correspond to operations on plaintexts.

---

## I

**Ideal Lattice**
Special type of lattice with algebraic structure, used in efficient FHE constructions.

**Integer Arithmetic**
Exact computation on whole numbers, supported by schemes like BFV.

---

## K

**Key Derivation Function (KDF)**
Algorithm for deriving cryptographic keys from a master secret.

**Key Pair**
Combination of public key (for encryption) and secret key (for decryption).

**Key Switching**
Technique for changing the secret key associated with a ciphertext, used in relinearization.

---

## L

**Lattice-Based Cryptography**
Cryptographic constructions based on the hardness of lattice problems.

**Learning With Errors (LWE)**
Cryptographic problem involving finding a secret from noisy linear equations.

**Leveled FHE**
FHE scheme supporting bounded computation depth without bootstrapping.

**LoLa (Low Latency)**
Optimized CNN architecture for fast encrypted inference.

---

## M

**Modulus**
The integer modulus used in polynomial arithmetic (coefficient modulus q or plaintext modulus t).

**Modulus Switching**
Technique for reducing ciphertext modulus to manage noise growth.

**MPC (Multi-Party Computation)**
Cryptographic protocol enabling joint computation while keeping inputs private.

**Multiplicative Depth**
The maximum number of consecutive multiplication operations in a circuit.

**Multiplicative Homomorphism**
Property where encryption of product equals product of encryptions:
Enc(a × b) = Enc(a) × Enc(b)

---

## N

**NIST (National Institute of Standards and Technology)**
U.S. agency providing cryptographic standards and recommendations.

**Noise**
Random error added during encryption for security, grows with computation.

**Noise Budget**
Amount of noise a ciphertext can tolerate before decryption fails.

**NTT (Number Theoretic Transform)**
Fast algorithm for polynomial multiplication, analogous to FFT.

---

## P

**Paillier Cryptosystem**
Partially homomorphic encryption scheme supporting additive operations.

**Partial Homomorphic Encryption (PHE)**
Encryption supporting only specific operations (addition OR multiplication, not both).

**PHI (Protected Health Information)**
Sensitive healthcare data requiring encryption under HIPAA regulations.

**Plaintext**
Original, unencrypted data.

**Plaintext Modulus**
Parameter t in BFV, determining the message space size.

**Polynomial Encoding**
Method of representing data as polynomials for encryption.

**Polynomial Modulus**
The polynomial xⁿ + 1 defining the ring structure in RLWE schemes.

**Polynomial Modulus Degree**
Parameter n, determining the polynomial ring dimension and security level.

**Post-Quantum Cryptography**
Cryptographic schemes believed secure against quantum computers.

**PPML (Privacy-Preserving Machine Learning)**
Machine learning on encrypted data without exposing training data or queries.

**Private Key**
See Secret Key.

**Public Key**
Key used for encryption, can be freely distributed.

---

## Q

**Quantum Resistance**
Property of being secure against attacks by quantum computers.

---

## R

**RBAC (Role-Based Access Control)**
Access control model based on user roles.

**ReLU (Rectified Linear Unit)**
Activation function in neural networks: f(x) = max(0, x).

**Relinearization**
Process of reducing ciphertext size after multiplication using relinearization keys.

**Relinearization Keys**
Evaluation keys enabling size reduction after homomorphic multiplication.

**Rescaling**
CKKS operation that reduces scale and manages fixed-point precision.

**Ring**
Algebraic structure R = ℤ[x]/(xⁿ + 1) used in RLWE-based schemes.

**RLWE (Ring Learning With Errors)**
Variant of LWE over polynomial rings, basis for efficient FHE schemes.

**Root of Unity**
Complex number ω where ωⁿ = 1, used in NTT operations.

**Rotation**
Homomorphic operation that cyclically shifts elements in an encrypted vector.

**RSA**
Public-key cryptosystem with multiplicative homomorphic properties.

---

## S

**Scale**
Parameter in CKKS determining fixed-point precision for approximate arithmetic.

**Secret Key**
Key used for decryption, must be kept confidential.

**Secret Sharing**
Technique for distributing a secret across multiple parties.

**Security Level**
Estimated computational cost (in bits) of breaking the encryption (e.g., 128-bit, 256-bit).

**Semantic Security**
Property that ciphertext reveals no information about plaintext beyond its length.

**Shamir Secret Sharing**
Method for threshold secret sharing using polynomial interpolation.

**SIMD (Single Instruction Multiple Data)**
Parallel computation model, enabled by batching in HE.

**Slot**
Individual position in a batched plaintext/ciphertext containing one value.

**SMPC (Secure Multi-Party Computation)**
See MPC.

**Standard Deviation**
Parameter σ determining the width of the error distribution.

**Symmetric Encryption**
Encryption using the same key for encryption and decryption.

---

## T

**TFHE (Fast Fully Homomorphic Encryption over the Torus)**
FHE scheme with very fast bootstrapping, suitable for binary operations.

**Threshold Cryptography**
Cryptographic schemes requiring t-out-of-n parties to perform operations.

**Threshold Decryption**
Decryption requiring cooperation of multiple key share holders.

**Throughput**
Number of operations completed per unit time (e.g., encryptions per second).

---

## V

**Verifiable Computation**
Property enabling verification that computation was performed correctly.

**Verifiable Credential (VC)**
Digital credential with cryptographic proof of authenticity.

---

## W

**WIA (World Certification Industry Association)**
Organization developing global standards including WIA-SEC-012.

---

## Z

**Zero-Knowledge Proof (ZKP)**
Cryptographic proof that a statement is true without revealing why it's true.

**ℤ (Integers)**
The ring of integers.

**ℤ[x]**
Ring of polynomials with integer coefficients.

**ℤ_q**
Ring of integers modulo q.

---

## Symbols and Notation

**≈**
Approximately equal (used in CKKS for approximate results)

**⊙**
Component-wise multiplication

**Enc(m)**
Encryption of message m

**Dec(ct)**
Decryption of ciphertext ct

**||·||**
Norm (magnitude) of a vector or polynomial

**←**
Sampled from (e.g., e ← χ means "e is sampled from distribution χ")

**mod q**
Modulo operation with modulus q

**⌊x⌉**
Rounding to nearest integer

**[·]**
Vector or array notation

**{·}**
Set notation

**O(·)**
Big-O notation for computational complexity

---

## Abbreviations

**AES** - Advanced Encryption Standard
**API** - Application Programming Interface
**BFV** - Brakerski-Fan-Vercauteren
**CKKS** - Cheon-Kim-Kim-Song
**CPU** - Central Processing Unit
**DPA** - Differential Power Analysis
**FHE** - Fully Homomorphic Encryption
**GDPR** - General Data Protection Regulation
**GPU** - Graphics Processing Unit
**HE** - Homomorphic Encryption
**HIPAA** - Health Insurance Portability and Accountability Act
**HSM** - Hardware Security Module
**KDF** - Key Derivation Function
**LWE** - Learning With Errors
**ML** - Machine Learning
**MPC** - Multi-Party Computation
**NIST** - National Institute of Standards and Technology
**NTT** - Number Theoretic Transform
**PCI DSS** - Payment Card Industry Data Security Standard
**PHE** - Partial Homomorphic Encryption
**PHI** - Protected Health Information
**PKI** - Public Key Infrastructure
**PPML** - Privacy-Preserving Machine Learning
**RBAC** - Role-Based Access Control
**ReLU** - Rectified Linear Unit
**RLWE** - Ring Learning With Errors
**RSA** - Rivest-Shamir-Adleman
**SIMD** - Single Instruction Multiple Data
**SMPC** - Secure Multi-Party Computation
**TFHE** - Fast Fully Homomorphic Encryption over the Torus
**TLS** - Transport Layer Security
**VC** - Verifiable Credential
**WIA** - World Certification Industry Association
**ZKP** - Zero-Knowledge Proof

---

## Common Parameter Names

**n** - Polynomial modulus degree
**q** - Coefficient modulus
**t** - Plaintext modulus (BFV)
**σ** (sigma) - Standard deviation of error distribution
**Δ** (delta) - Scale parameter (CKKS)
**λ** (lambda) - Security parameter
**pk** - Public key
**sk** - Secret key
**rlk** - Relinearization keys
**gk** - Galois keys
**ct** - Ciphertext
**pt** - Plaintext
**m** - Message

---

## Mathematical Structures

**Ring R**
R = ℤ[x]/(xⁿ + 1) - The polynomial ring used in RLWE

**Ring R_q**
R_q = R/qR - The ring R reduced modulo q

**Error Distribution χ**
Typically discrete Gaussian distribution χ = DG(σ²)

**Message Space**
R_t for BFV, ℂⁿ/² for CKKS

---

## Operation Notation

**Enc(m, pk)** - Encrypt message m with public key pk
**Dec(ct, sk)** - Decrypt ciphertext ct with secret key sk
**Add(ct₁, ct₂)** - Homomorphic addition
**Mult(ct₁, ct₂)** - Homomorphic multiplication
**Relin(ct, rlk)** - Relinearization
**Rot(ct, k, gk)** - Rotation by k positions
**Bootstrap(ct)** - Refresh ciphertext noise budget

---

## Related Standards

**WIA-SEC-001** - Foundation Security Framework
**WIA-SEC-005** - Zero-Knowledge Proofs
**WIA-SEC-008** - Blockchain Security
**WIA-IDENTITY-002** - Decentralized Identity
**ISO/IEC 18033** - Encryption Algorithms
**ISO/IEC 19790** - Security Requirements for Cryptographic Modules
**NIST SP 800-57** - Key Management Recommendations
**FIPS 140-3** - Cryptographic Module Validation

---

## Resources for Further Learning

**Books:**
- "A Graduate Course in Applied Cryptography" by Boneh and Shoup
- "Fully Homomorphic Encryption for Mathematicians" by Rohloff and Cousins

**Courses:**
- Stanford CS 355: Topics in Cryptography
- MIT 6.875: Foundations of Cryptography

**Communities:**
- Homomorphic Encryption Standardization Consortium
- WIA Standards Community
- Cryptography Stack Exchange

**Libraries:**
- Microsoft SEAL
- PALISADE
- HElib
- TFHE
- OpenFHE

---

**Note on Terminology:**

This glossary uses standard cryptographic notation and terminology from the academic literature. When implementing WIA-SEC-012, developers should use these terms consistently to ensure interoperability and clear communication.

For terms specific to particular implementations or programming languages, refer to the respective SDK documentation.

---

弘益人間 (홍익인간) · Benefit All Humanity

© 2025 SmileStory Inc. / WIA

**End of Glossary**
