# WIA-blockchain-intro PHASE 3 — PROTOCOL Specification

**Standard:** WIA-blockchain-intro
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern a
blockchain-intro operator: the ISO/TC 307 reference-
architecture discipline; the W3C VC / DID identity
discipline; the smart-contract security and audit
discipline (OWASP Smart Contract Top 10 + SWC
registry + EEA Common Standards); the wallet-and-
key-management discipline; the regulatory-
classification discipline (EU MiCA + US Howey-test
+ KR Virtual Asset User Protection Act); the AML /
KYC + Travel Rule discipline (FATF Rec 16 + 31 CFR
+ EU AMLR); the privacy-and-data-protection
discipline (ISO/TR 23244 + GDPR Art 17 erasure
challenge for immutable ledgers); the post-quantum
migration discipline (NIST FIPS 203 / 204 / 205);
the curriculum-integrity discipline for educational
operators; and the supervisory cooperation
discipline.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015, ISO/IEC 27001:2022
- ISO/TC 307 — ISO 22739:2024 + ISO 23257:2022 +
  ISO 23455:2019 + ISO/TS 23635 + ISO/TR 23244 +
  ISO/TR 23576 + ISO/TS 23258 + ISO/TR 3242
- ITU-T FG DLT
- W3C VC 2.0 + DID Core 1.0 + Linked Data Proofs +
  VC Bitstring Status List + Data Integrity 1.0
- IETF RFC 9381 (VRF), RFC 7515-7519 (JOSE / JWT),
  RFC 9162 (Certificate Transparency), RFC 5905
  (NTPv4), RFC 9421 (HTTP Message Signatures), RFC
  9457 (Problem Details), RFC 8949 (CBOR), RFC 8152
  (COSE)
- NIST IR 8202 + IR 8301 + FIPS 186-5 + FIPS 203 +
  FIPS 204 + FIPS 205
- OWASP Smart Contract Top 10
- SWC (Smart Contract Weakness Classification)
  registry
- Enterprise Ethereum Alliance (EEA) Mainnet Working
  Group Common Standards
- ConsenSys Diligence Smart Contract Best Practices
- US SEC Framework for "Investment Contract"
  Analysis of Digital Assets (April 2019)
- US CFTC LabCFTC primer + virtual currency
  derivatives oversight
- US FinCEN BSA + 31 CFR 1010.100(ff)(5) MSB +
  FinCEN guidance FIN-2019-G001 (Travel Rule for
  CVCs)
- FATF Recommendations 15 + 16 (the Travel Rule
  for Virtual Asset Service Providers)
- EU MiCA (Reg (EU) 2023/1114) Articles 16-58
  (asset-referenced tokens) + 59-66 (e-money
  tokens) + 67-89 (CASP) + 90-92 (transparency)
- EU AMLR (Reg (EU) 2024/1153) extension to crypto-
  asset service providers
- KR 가상자산 이용자 보호 등에 관한 법률 (in force
  2024-07-19) + 특정금융정보법 + 자본시장법

---

## §1 ISO/TC 307 Reference-Architecture Discipline

The ISO/TC 307 reference-architecture discipline:

- ISO 22739:2024 vocabulary applied uniformly across
  the operator's documentation.
- ISO 23257:2022 reference architecture maps the
  operator's deployment to the standardised layers
  (cryptographic services, distributed-ledger,
  smart contracts, identity-and-access management,
  application services).
- ISO/TS 23635 governance baseline for the
  operator's network operations.
- ISO/TR 23244 privacy considerations integrated
  with the operator's PIA / DPIA.
- ISO/TR 23576 digital-asset-custodian security
  baseline integrated with the operator's wallet
  discipline.

## §2 W3C VC / DID Identity Discipline

The VC / DID discipline:

- DID method selection — did:web for institutional
  identifiers, did:key / did:jwk for ephemeral
  identities, did:ion / did:cheqd for DID-on-DLT
  use cases, did:pkh for blockchain-account-bound
  identifiers.
- VC issuance — the issuer publishes the credential
  schema, issues VCs with cryptographic proof
  (Data Integrity 1.0 / JSON Web Signature 2020 /
  Ed25519 Signature 2020 / BBS Signature 2023).
- Verifiable Presentation — the holder selectively
  discloses VC claims to the verifier per the
  presentation-request.
- Status checking — the verifier consults the VC
  Bitstring Status List for revocation status
  before accepting a credential.

## §3 Smart-Contract Security and Audit Discipline

The smart-contract security discipline:

- OWASP Smart Contract Top 10 covering reentrancy,
  arithmetic overflow / underflow, oracle
  manipulation, access-control bypass, denial-of-
  service, front-running, randomness predictability,
  signature replay, off-by-one errors, gas
  optimisation hazards.
- SWC registry mapping each weakness to a unique
  identifier referenced in audit reports.
- EEA Mainnet Working Group Common Standards for
  EVM-class enterprise deployments.
- Independent security audit by at least two firms
  for production-class contracts handling real
  value.
- Formal verification (K-framework, Dafny, Coq) for
  high-stakes contracts.
- Bug bounty programme for contracts in production.

## §4 Wallet-and-Key-Management Discipline

The wallet discipline:

- Custodial wallets — the operator's HSM under
  FIPS 140-3 (Level 3 minimum for production-value
  custody).
- Non-custodial wallets — user-controlled private
  keys; the operator's role is education only.
- MPC wallets — threshold-cryptography schemes
  (e.g., GG18, CMP, FROST) replacing single-key
  signing.
- Smart-contract wallets — ERC-4337 account
  abstraction with social recovery, session keys,
  gas sponsorship.
- Insurance — custodial-wallet operators carry
  cyber-and-crime insurance against the published
  loss scenarios.

## §5 Regulatory-Classification Discipline

For each token / digital-asset issued or supported:

- US SEC Howey-Test analysis — the SEC Framework
  for "Investment Contract" Analysis of Digital
  Assets (April 2019) walks through the four
  prongs.
- US CFTC commodity classification (Bitcoin and
  Ether classified as commodities).
- US FinCEN MSB obligation — wallet operators and
  exchanges that meet 31 CFR 1010.100(ff)(5)
  register and comply with BSA.
- EU MiCA classification — asset-referenced token
  (ART), e-money token (EMT), or other crypto-
  asset; per-classification disclosures and
  authorisation regime.
- KR 가상자산이용자보호법 — KR Virtual Asset User
  Protection Act covers user-protection,
  segregation of customer assets, cold-storage
  ratios, and listing-and-delisting transparency.
- KR 특정금융정보법 — KR-jurisdiction VASP
  registration and Travel-Rule compliance.

## §6 AML / KYC and Travel-Rule Discipline

For VASPs (Virtual Asset Service Providers):

- FATF Rec 15 covering VASPs.
- FATF Rec 16 Travel Rule — originator and
  beneficiary information travels with each
  transfer above the de-minimis threshold (USD /
  EUR 1,000).
- US FinCEN Travel Rule under 31 CFR 1010.410(e)
  + 1010.410(f) extended to CVCs by FinCEN
  guidance FIN-2019-G001.
- EU AMLR Reg (EU) 2024/1153 extending the AML
  regime to crypto-asset service providers; the
  EU Wire Transfer Regulation (Reg (EU) 2023/1113)
  Travel Rule applies to crypto.
- KR 특정금융정보법 Travel-Rule applied through
  bilateral integration between KR-licensed VASPs.
- Travel-Rule technical solutions — IVMS 101 +
  TRP / TRISA / OpenVASP / Sygna Bridge.

## §7 Privacy and GDPR Erasure-Challenge Discipline

For immutable-ledger privacy:

- ISO/TR 23244 privacy considerations.
- Personal-data minimisation — store hashes /
  pseudonyms on-chain; reference off-chain
  encrypted data store.
- GDPR Article 17 right-to-erasure challenge —
  immutable ledger conflicts with erasure;
  mitigation through pseudonymisation, encryption-
  with-key-discard, or off-chain storage.
- Right of access (Art 15) and rectification (Art
  16) addressed through off-chain operator records.

## §8 Post-Quantum Migration Discipline

The post-quantum-cryptography migration discipline:

- NIST FIPS 203 (ML-KEM) + FIPS 204 (ML-DSA) + FIPS
  205 (SLH-DSA) post-quantum primitives.
- Crypto-agility — the operator's signature and
  key-encapsulation algorithms are versioned so
  migration is possible without protocol breaks.
- Hash-based signatures (XMSS / LMS) for one-shot
  signing where appropriate.
- Hybrid signing (classical + post-quantum) during
  the transition window.

## §9 Educational-Curriculum Integrity Discipline

For educational operators:

- Curriculum content references the cited
  references (ISO 22739 vocabulary, NIST IR 8202
  / 8301 introduction, W3C VC / DID, OWASP / SWC
  for security).
- Hands-on labs use test-network deployments only
  (Sepolia / Holesky / Polygon Amoy / private
  test-chains) — no real-value tokens for student
  exercises.
- Assessment integrity — student submissions are
  attributable via signed VC issuance, not by
  username only.

## §10 Identity, Time, and Audit Discipline

NTPv4 stratum-2 or better is the operator's clock
baseline. Audit-events are emitted for every VC
issuance / verification / revocation, contract
deployment / upgrade / audit completion, custody-
event recording, regulatory-classification update,
and educational-assessment submission.

## §11 Layer-2 and Rollup Discipline

For operators integrating Layer-2 / rollup
networks:

- Optimistic rollups (Arbitrum, Optimism, Base) —
  fraud-proof window, sequencer decentralisation
  status, withdrawal challenge period.
- ZK rollups (zkSync, StarkNet, Linea, Scroll) —
  validity-proof system, prover decentralisation,
  data-availability layer.
- Validium / volition — data-availability committee
  governance.
- L2 cross-chain bridge audits per SWC + EEA Best
  Practices.
- Sequencer-decentralisation roadmap participation.

## §12 Stablecoin and Payment-Token Discipline

For operators issuing or supporting stablecoins:

- Reserve-backing attestation per the operator's
  published auditor cadence (typically monthly or
  quarterly).
- Reserve composition disclosure (USD cash + cash-
  equivalent + Treasury bills + repo).
- Smart-contract-controlled mint / burn restricted
  to authorised parties.
- Per-jurisdictional licensing — EU MiCA Title
  III (asset-referenced) + Title IV (e-money
  token); US state money-transmitter licensing;
  KR 가상자산이용자보호법 stablecoin reserve
  requirements when in force.

## §13 Cross-Chain Bridge Risk Discipline

For operators integrating cross-chain bridges:

- Bridge architecture review — lock-and-mint vs.
  burn-and-mint vs. liquidity-pool design.
- Validator-set decentralisation — multi-party
  computation, threshold signing, trusted-execution-
  environment attestation.
- Bridge audit cadence — quarterly minimum given
  historical bridge-exploit incidents (Ronin,
  Wormhole, Nomad, Multichain post-mortems).
- Insurance integration — Nexus Mutual / Sherlock /
  InsurAce coverage for bridge exposure.
- Withdrawal-pause governance — emergency-pause
  authority and accountability.

## §14 Conformance

Implementations claiming PHASE-3 conformance enforce
the discipline at every relevant decision point,
satisfy the ISO/TC 307 reference-architecture
baseline, exercise the smart-contract security and
audit discipline before promoting any contract to
production, satisfy the regulatory-classification
discipline for the operating jurisdiction, exercise
the AML / KYC / Travel-Rule discipline where the
operator is a VASP, and exercise the post-quantum
migration discipline.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-blockchain-intro
- **Last Updated:** 2026-04-28
