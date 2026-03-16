# WIA-SEC-023: Privacy Preservation - GLOSSARY

**Standard ID:** WIA-SEC-023
**Version:** 1.0.0
**Category:** Security (SEC)

---

## A

**Anonymization**
The process of removing or modifying personally identifiable information from datasets so that individuals cannot be readily identified. Includes techniques like generalization, suppression, and perturbation.

**Adversary**
An entity attempting to breach privacy by re-identifying individuals, inferring sensitive information, or otherwise violating privacy guarantees.

**Attribute Disclosure**
A privacy breach where an attacker learns sensitive information about an individual without necessarily identifying them.

---

## B

**Background Knowledge Attack**
An attack where an adversary uses external information combined with released data to compromise privacy, even when the data satisfies formal privacy guarantees.

**BBS+ Signatures**
A cryptographic signature scheme that enables selective disclosure of attributes in verifiable credentials while maintaining unforgeable signatures.

**Bloom Filter**
A space-efficient probabilistic data structure used for privacy-preserving record linkage that tests whether an element is a member of a set.

**Budget (Privacy Budget)**
In differential privacy, the cumulative epsilon (ε) consumed by queries. Once exhausted, no more queries can be answered without compromising privacy.

---

## C

**CCPA (California Consumer Privacy Act)**
California state law that provides privacy rights and consumer protection for residents of California, USA.

**Composition**
In differential privacy, the effect of multiple queries on privacy guarantees. Sequential composition adds epsilon values; parallel composition takes the maximum.

**Credential (Verifiable Credential)**
A tamper-evident statement made by an issuer about a subject, following W3C standards, optionally enhanced with privacy-preserving features.

---

## D

**Data Clean Room**
A secure environment where multiple parties can collaborate on data analysis without exposing raw data to each other, using privacy-preserving techniques.

**Data Minimization**
The principle of collecting and processing only the minimum amount of personal data necessary for a specified purpose.

**De-identification**
The process of removing identifying information from data. Less formal than anonymization; may be reversible.

**Delta (δ)**
In (ε, δ)-differential privacy, the probability that the privacy guarantee fails. Typically set to be very small (e.g., 1/n²).

**Differential Privacy (DP)**
A mathematical framework providing provable privacy guarantees by adding controlled noise to query results, ensuring individual records don't significantly affect outputs.

**DID (Decentralized Identifier)**
A globally unique identifier that does not require a centralized registration authority, often used with verifiable credentials.

**Direct Identifier**
An attribute that uniquely identifies an individual (e.g., name, SSN, email). Must be removed or encrypted in anonymized datasets.

**Discernibility Metric (DM)**
A measure of information loss in k-anonymity, calculated as the sum of squared equivalence class sizes.

---

## E

**Earth Mover's Distance (EMD)**
A distance measure between probability distributions used in t-closeness to compare sensitive attribute distributions.

**Epsilon (ε)**
The privacy budget parameter in differential privacy. Lower values provide stronger privacy but less accuracy. Common values: 0.1 to 10.

**Equivalence Class**
In k-anonymity, a group of records with identical quasi-identifier values. Each class must contain at least k records.

**Exponential Mechanism**
A differential privacy mechanism for selecting from a discrete set of outputs based on a quality function, with selection probability proportional to exp(ε × quality).

---

## F

**Federated Learning**
A machine learning approach where models are trained across decentralized devices holding local data, without exchanging raw data.

**Fully Homomorphic Encryption (FHE)**
Encryption scheme allowing arbitrary computations on encrypted data without decryption. Enables complete privacy-preserving computation.

---

## G

**GDPR (General Data Protection Regulation)**
European Union regulation on data protection and privacy, applicable to processing of personal data of EU residents.

**Generalization**
A data anonymization technique that replaces specific values with broader categories (e.g., age 30 → age range 30-39).

**Global Sensitivity**
The maximum change in a query's output when a single record is added or removed from the dataset. Critical for differential privacy noise calibration.

**Gaussian Mechanism**
A differential privacy mechanism adding Gaussian noise to query results, used for (ε, δ)-differential privacy.

---

## H

**HIPAA (Health Insurance Portability and Accountability Act)**
US federal law establishing data privacy and security provisions for safeguarding medical information.

**Homogeneity Attack**
An attack on k-anonymous data where all records in an equivalence class have the same sensitive value, enabling attribute disclosure.

**Homomorphic Encryption (HE)**
Encryption allowing computation on ciphertexts, generating encrypted results that decrypt to the same result as operations on plaintexts.

---

## I

**Identity Disclosure**
A privacy breach where an attacker successfully links a record to a specific individual's identity.

**Information Loss**
The reduction in data utility resulting from privacy-preserving transformations. Measured by metrics like NCP or entropy loss.

**Inference Attack**
An attack where an adversary deduces sensitive information about individuals through statistical analysis or correlation with other data.

---

## K

**k-Anonymity**
A privacy property where each record is indistinguishable from at least k-1 other records with respect to quasi-identifiers.

**Kullback-Leibler Divergence**
A measure of difference between two probability distributions, sometimes used as an alternative distance measure in privacy metrics.

---

## L

**l-Diversity**
An enhancement of k-anonymity requiring each equivalence class to have at least l well-represented distinct values for sensitive attributes.

**Laplace Distribution**
A probability distribution with PDF proportional to exp(-|x|/b), used for adding noise in differential privacy.

**Laplace Mechanism**
A differential privacy mechanism adding noise from the Laplace distribution, providing ε-differential privacy.

**Linkage Attack**
An attack where an adversary links anonymized records to external datasets containing identifying information.

**Local Differential Privacy (LDP)**
A variant of differential privacy where noise is added by data owners before sharing, providing stronger privacy at cost of utility.

---

## M

**Membership Inference Attack**
An attack attempting to determine whether a specific individual's data was included in a training dataset or analysis.

**Microdata**
Data containing information about individual entities (people, households, businesses), requiring privacy protection.

**Mondrian Algorithm**
A popular k-anonymity algorithm using recursive multidimensional partitioning to generalize quasi-identifiers.

---

## N

**Noise Addition**
A privacy technique adding random values to data or query results to prevent exact inference about individuals.

**Normalized Certainty Penalty (NCP)**
A metric for measuring information loss in anonymized data, calculated as the ratio of generalized range to total domain range.

---

## O

**Obfuscation**
Deliberately obscuring information to prevent understanding, used in various privacy-preserving techniques.

---

## P

**Paillier Cryptosystem**
An additive homomorphic encryption scheme where E(m₁) × E(m₂) = E(m₁ + m₂), useful for privacy-preserving computation.

**Personal Data**
Any information relating to an identified or identifiable natural person. Subject to privacy regulations.

**Personally Identifiable Information (PII)**
Information that can be used to distinguish or trace an individual's identity, alone or combined with other information.

**Privacy-Enhancing Technologies (PETs)**
Technologies designed to protect privacy by minimizing personal data use, maximizing data security, or empowering individuals.

**Privacy by Design**
An approach embedding privacy into technology design from the outset, rather than as an afterthought.

**Pseudonymization**
Replacing identifying fields with pseudonyms (artificial identifiers). Unlike anonymization, pseudonymization is reversible.

---

## Q

**Quasi-Identifier**
An attribute that, while not uniquely identifying alone, can identify individuals when combined with other quasi-identifiers.

**Query**
A request for information from a database or dataset. In differential privacy, each query consumes privacy budget.

---

## R

**Re-identification**
Successfully linking anonymized data back to specific individuals, breaching privacy.

**Re-identification Risk**
The probability that an individual can be uniquely identified in a dataset, measured by metrics like prosecutor risk or journalist risk.

**Reconstruction Attack**
An attack attempting to reconstruct original sensitive data from anonymized or perturbed versions.

---

## S

**Safe Harbor Method**
A HIPAA de-identification approach specifying 18 identifiers to remove from protected health information.

**SEAL (Simple Encrypted Arithmetic Library)**
Microsoft's open-source homomorphic encryption library supporting BFV and CKKS schemes.

**Secret Sharing**
A cryptographic method dividing a secret into shares such that a threshold number of shares can reconstruct the secret, but fewer reveal nothing.

**Secure Multi-Party Computation (SMPC)**
Cryptographic protocols enabling parties to jointly compute functions over their inputs while keeping inputs private.

**Selective Disclosure**
The ability to reveal only specific attributes from a credential while keeping others hidden, often using BBS+ signatures.

**Sensitivity (Global Sensitivity)**
See Global Sensitivity.

**Suppression**
A data anonymization technique that removes or replaces values with special markers (e.g., *) to achieve privacy.

**Synthetic Data**
Artificially generated data that mimics the statistical properties of real data while containing no actual personal information.

---

## T

**t-Closeness**
An enhancement of l-diversity requiring the distribution of sensitive attributes in each equivalence class to be close to the overall distribution.

**Trusted Execution Environment (TEE)**
Hardware-isolated secure area providing confidentiality and integrity guarantees for code and data processing.

---

## U

**Utility**
The usefulness of data for its intended purpose. Privacy protection often reduces utility; the goal is optimal tradeoff.

---

## V

**Verifiable Credential (VC)**
A tamper-evident credential following W3C standards, containing claims about a subject that can be cryptographically verified.

**Verifiable Presentation**
Data derived from one or more verifiable credentials, presented to a verifier for verification.

---

## Z

**Zero-Knowledge Proof (ZKP)**
A cryptographic method proving knowledge of information without revealing the information itself.

**zk-SNARK (Zero-Knowledge Succinct Non-Interactive Argument of Knowledge)**
A form of zero-knowledge proof that is succinct (small proof size) and non-interactive (no back-and-forth communication).

---

## Mathematical Notation

**ε (Epsilon)**
Privacy budget parameter in differential privacy

**δ (Delta)**
Failure probability in (ε, δ)-differential privacy

**k**
Anonymity parameter in k-anonymity (minimum equivalence class size)

**l**
Diversity parameter in l-diversity (minimum distinct sensitive values)

**t**
Closeness parameter in t-closeness (maximum distribution distance)

**Δf (Delta f)**
Global sensitivity of function f

**Pr[·]**
Probability of an event

**E(·)**
Encryption function or mathematical expectation (context-dependent)

**D₁, D₂**
Neighboring datasets (differ in one record)

**M(D)**
Output of mechanism M on dataset D

**⊕, ⊗**
Homomorphic addition and multiplication operations

---

## Acronyms

**AI** - Artificial Intelligence
**API** - Application Programming Interface
**BFV** - Brakerski-Fan-Vercauteren (homomorphic encryption scheme)
**BGV** - Brakerski-Gentry-Vaikuntanathan (homomorphic encryption scheme)
**CCPA** - California Consumer Privacy Act
**CKKS** - Cheon-Kim-Kim-Song (homomorphic encryption scheme)
**DCR** - Decisional Composite Residuosity
**DID** - Decentralized Identifier
**DM** - Discernibility Metric
**DP** - Differential Privacy
**DP-SGD** - Differentially Private Stochastic Gradient Descent
**DPIA** - Data Protection Impact Assessment
**EMD** - Earth Mover's Distance
**FHE** - Fully Homomorphic Encryption
**GDPR** - General Data Protection Regulation
**GMW** - Goldreich-Micali-Wigderson (SMPC protocol)
**HE** - Homomorphic Encryption
**HIPAA** - Health Insurance Portability and Accountability Act
**JSON** - JavaScript Object Notation
**LDP** - Local Differential Privacy
**ML** - Machine Learning
**NCP** - Normalized Certainty Penalty
**PETs** - Privacy-Enhancing Technologies
**PII** - Personally Identifiable Information
**PIPEDA** - Personal Information Protection and Electronic Documents Act
**RLWE** - Ring Learning With Errors
**SDK** - Software Development Kit
**SEAL** - Simple Encrypted Arithmetic Library
**SMPC** - Secure Multi-Party Computation
**TEE** - Trusted Execution Environment
**VC** - Verifiable Credential
**VP** - Verifiable Presentation
**W3C** - World Wide Web Consortium
**WIA** - World Certification Industry Association
**ZKP** - Zero-Knowledge Proof
**zk-SNARK** - Zero-Knowledge Succinct Non-Interactive Argument of Knowledge

---

## Privacy Frameworks by Region

**Europe**
- GDPR (General Data Protection Regulation)
- ePrivacy Directive

**United States**
- CCPA (California Consumer Privacy Act)
- CPRA (California Privacy Rights Act)
- HIPAA (Health Insurance Portability and Accountability Act)
- COPPA (Children's Online Privacy Protection Act)
- GLBA (Gramm-Leach-Bliley Act)

**Canada**
- PIPEDA (Personal Information Protection and Electronic Documents Act)

**Asia-Pacific**
- APPI (Japan - Act on Protection of Personal Information)
- PDPA (Singapore - Personal Data Protection Act)
- Privacy Act 1988 (Australia)

**Latin America**
- LGPD (Brazil - Lei Geral de Proteção de Dados)

---

## Common Privacy Levels

**By Epsilon (ε):**
```
ε ≤ 0.1    → Very High Privacy
0.1 < ε ≤ 1.0  → High Privacy
1.0 < ε ≤ 5.0  → Medium Privacy
ε > 5.0    → Low Privacy
```

**By k-Anonymity:**
```
k ≥ 10     → High Privacy
5 ≤ k < 10 → Medium Privacy
3 ≤ k < 5  → Low Privacy
k < 3      → Insufficient Privacy
```

**By Use Case:**
```
Medical Records: ε ≤ 0.5, k ≥ 10
Financial Data:  ε ≤ 1.0, k ≥ 5
General Analytics: ε ≤ 5.0, k ≥ 3
Public Statistics: ε ≤ 10.0, k ≥ 2
```

---

## Related WIA Standards

**WIA-SEC Family:**
- WIA-SEC-001: Encryption Standards
- WIA-SEC-002: Authentication Framework
- WIA-SEC-003: Authorization System
- WIA-SEC-023: Privacy Preservation (this standard)

**WIA-INTENT:**
- Intent-based privacy controls

**WIA-OMNI-API:**
- Privacy-aware API gateway

**WIA-SOCIAL:**
- Privacy-preserving social networking

---

## References for Further Reading

**Books:**
- "The Algorithmic Foundations of Differential Privacy" by Dwork & Roth
- "Privacy-Preserving Data Mining" by Aggarwal & Yu
- "A Pragmatic Introduction to Secure Multi-Party Computation" by Evans et al.

**Online Resources:**
- OpenDP Project: https://opendp.org
- NIST Privacy Framework: https://www.nist.gov/privacy-framework
- W3C Verifiable Credentials: https://www.w3.org/TR/vc-data-model/

**Courses:**
- Harvard CS208: Applied Privacy for Data Science
- Coursera: Privacy-Preserving Technologies
- edX: Data Privacy Fundamentals

---

**弘益人間 (홍익인간) - Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
World Certification Industry Association
