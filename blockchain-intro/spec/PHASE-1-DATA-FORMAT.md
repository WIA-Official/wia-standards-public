# WIA-blockchain-intro PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-blockchain-intro
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-blockchain-intro. The standard covers
persistent record shapes for the lifecycle of a
blockchain / distributed-ledger-technology (DLT)
introduction or learning programme — the DLT
platform identification record; the consensus-and-
governance record; the cryptographic-primitive
record; the W3C VC / DID identity-and-credential
record; the smart-contract definition and audit
record; the wallet and key-management record; the
chain-of-custody record for tokens, assets, and
digital twins; the regulatory-classification record
(EU MiCA + the operating jurisdiction's virtual-
asset law); the privacy-and-data-protection record;
and the supervisory-and-curriculum correspondence
record. WIA-blockchain-intro is targeted at the
introduction / on-ramp / educational phase — it
provides the reference data shape suitable for a
learning-platform or pilot deployment, while the
deeper trade and exchange disciplines remain in
specialised standards. Records are consumed by the
learning-platform operator, the developer / student
audience, the operator's compliance-and-risk function,
the supervisory authority for the operating
jurisdiction (US SEC + CFTC + FinCEN + state
regulators; EU EBA + ESMA + Member-State NCAs; KR
FSC + FSS + MOEF + KISA), and external auditors.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID) and IETF RFC 4122 (UUID URN)
- ISO/IEC 27001:2022 (information security management)
- IETF RFC 8259 (JSON), RFC 9457 (Problem Details)
- ISO/TC 307 (Blockchain and distributed ledger
  technologies) deliverables — ISO 22739:2024
  (Vocabulary), ISO 23257:2022 (Reference
  architecture), ISO 23455:2019 (Overview of and
  interactions between smart contracts in
  blockchain and distributed ledger technology
  systems), ISO/TS 23635 (Guidelines for governance),
  ISO/TR 23244 (Privacy and personally identifiable
  information protection considerations), ISO/TR
  23576 (Security of digital asset custodians),
  ISO/TS 23258 (Taxonomy and ontology), ISO/TR
  3242 (Use cases — interoperability)
- ITU-T FG DLT — Focus Group on Application of
  Distributed Ledger Technology deliverables
  (TR-DLT-Reqs, TR-DLT-Plat, TR-DLT-Reg-Frmwk,
  TR-DLT-Use-Cases)
- W3C Verifiable Credentials Data Model 2.0
- W3C Decentralized Identifiers (DIDs) v1.0
- W3C Linked Data Proofs + Data Integrity 1.0
- W3C VC JSON Schema 2023 + VC Status List 2021 +
  VC Bitstring Status List + VC Revocation List
  2020
- IETF RFC 9381 (Verifiable Random Function, VRF)
- IETF RFC 7515 / 7516 / 7517 / 7519 / 7638 / 9162
  (JOSE / JWT / JWK / Certificate Transparency)
- IETF RFC 8949 (CBOR), RFC 8152 (COSE)
- NIST IR 8202 (Blockchain Technology Overview)
- NIST IR 8301 (Blockchain Networks: Token Design
  and Management Overview)
- NIST FIPS 186-5 (Digital Signature Standard
  including ECDSA + EdDSA)
- NIST FIPS 203 / 204 / 205 (Module-Lattice-Based
  Key-Encapsulation, Module-Lattice-Based Digital
  Signature, Stateless Hash-Based Digital Signature
  — post-quantum cryptography)
- Ethereum ERC standards reference set (ERC-20
  fungible token, ERC-721 non-fungible token,
  ERC-1155 multi-token, ERC-4626 tokenized vault,
  ERC-1271 smart-contract signature validation,
  ERC-2771 meta-transactions, ERC-4337 account
  abstraction)
- EIP standards reference set (EIP-155 chain-id,
  EIP-712 typed-data signing, EIP-1559 fee market,
  EIP-2930 access lists, EIP-4844 blob transactions
  proto-danksharding)
- EU MiCA (Regulation (EU) 2023/1114) for crypto-
  asset operators + Reg (EU) 2024/859 (DLT Pilot
  Regime amendment)
- EU Reg (EU) 1153/2024 (anti-money-laundering
  package extension to crypto-asset service
  providers)
- US SEC No-Action Letter and Investor Bulletin
  references for digital-asset securities; SEC
  Framework for "Investment Contract" Analysis of
  Digital Assets (April 2019)
- US CFTC oversight of virtual-currency derivatives
  + Bitcoin and Ether classified as commodities
- US FinCEN BSA + 31 CFR 1010.100(ff)(5) money
  services business
- KR 가상자산 이용자 보호 등에 관한 법률 (the
  KR Virtual Asset User Protection Act, in force
  2024-07-19) + KR 특정금융정보법 + KR 자본시장법
  + KR FSS 가상자산 거래소 검사 + KR FIU
- ISO 17442 LEI for institutional identifiers

---

## §1 Scope

This PHASE defines persistent shapes for the
artefacts a blockchain-intro operator (a learning-
platform operator, a pilot-deployment operator, a
service-provider exposing introductory blockchain
features, an educational institution providing
blockchain coursework) maintains:

- The DLT platform identification record.
- The consensus-and-governance record.
- The cryptographic-primitive record.
- The W3C VC / DID identity record.
- The smart-contract definition and audit record.
- The wallet and key-management record.
- The token / asset / digital-twin chain-of-custody
  record.
- The regulatory-classification record.
- The privacy-and-data-protection record.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name)
operatorRole         : enum ("learning-platform" |
                       "pilot-deployment" |
                       "intro-service-provider" |
                       "educational-institution" |
                       "research-consortium" |
                       "user-defined")
operatorJurisdiction : array of string (ISO 3166-1)
operatorLei          : string (ISO 17442)
governingFrameworks  : array of enum ("ISO-22739-
                       2024" | "ISO-23257-2022" |
                       "ISO-23455-2019" |
                       "ISO-TS-23635" |
                       "ISO-TR-23244" |
                       "ISO-TR-23576" |
                       "ITU-T-FG-DLT" |
                       "W3C-VC-2-0" | "W3C-DID-1-0"
                       | "RFC-9381-VRF" |
                       "RFC-7515-7516-7517-7519-
                       JOSE-JWT" | "RFC-9162-CT" |
                       "NIST-IR-8202" |
                       "NIST-IR-8301" |
                       "NIST-FIPS-186-5" |
                       "NIST-FIPS-203-204-205-PQ" |
                       "ERC-20-721-1155-4626-1271-
                       2771-4337" |
                       "EIP-155-712-1559-2930-4844"
                       | "EU-MICA-2023-1114" |
                       "EU-MICA-PILOT-2024-859" |
                       "EU-AMLR-2024-1153" |
                       "US-SEC-DIGITAL-ASSET-FRMWK
                       -2019" |
                       "US-CFTC-VIRTUAL-CURRENCY" |
                       "US-FINCEN-31-CFR-1010-MSB"
                       | "KR-가상자산이용자보호법" |
                       "KR-특정금융정보법" |
                       "user-defined")
introductionPurpose  : enum ("educational-only" |
                       "pilot-with-permissioned-
                       network" | "intro-service-
                       with-test-tokens" |
                       "intro-service-with-real-
                       value-tokens" |
                       "research-prototype" |
                       "user-defined")
programmeStatus      : enum ("design" | "operating"
                       | "limited-rollout" |
                       "wind-down" | "archived")
```

## §3 DLT Platform Identification Record

```
dltPlatform:
  platformId         : string (uuidv7)
  platformName       : string
  platformKind       : enum ("public-permissionless
                       -bitcoin" | "public-
                       permissionless-ethereum" |
                       "public-permissionless-
                       polkadot" | "public-
                       permissionless-cosmos" |
                       "permissioned-hyperledger-
                       fabric" | "permissioned-
                       hyperledger-besu" |
                       "permissioned-corda" |
                       "permissioned-quorum" |
                       "consortium-other" |
                       "private-test-only" |
                       "user-defined")
  networkChainId     : string (the chain identifier
                       per EIP-155 for EVM-class
                       networks; the equivalent
                       per-platform identifier
                       elsewhere)
  blockTimeAverage   : object (target average
                       block-time; ms)
  finality           : enum ("probabilistic-bitcoin"
                       | "probabilistic-ethereum-
                       pre-merge" | "casper-ffg-
                       finality-ethereum-post-merge"
                       | "byzantine-fault-tolerant
                       -bft" | "instant-finality" |
                       "user-defined")
```

## §4 Consensus-and-Governance Record

```
consensusRecord:
  consensusKind      : enum ("proof-of-work" |
                       "proof-of-stake" | "delegated
                       -proof-of-stake" |
                       "proof-of-authority" |
                       "practical-byzantine-fault-
                       tolerant-pbft" |
                       "raft" | "tendermint-bft" |
                       "casper-ffg" | "user-defined")
  validatorSetSize   : integer (the cardinality of
                       the validator / miner set;
                       absent for fully-permissionless
                       networks)
  byzantineFaultThreshold : object (the BFT
                       threshold — typically 1/3 for
                       BFT-class consensus)

governanceRecord:
  governanceModel    : enum ("on-chain-token-
                       weighted-voting" |
                       "off-chain-improvement-
                       proposal" | "consortium-
                       multisig" | "foundation-
                       managed" | "decentralized-
                       autonomous-organization-dao"
                       | "hybrid" | "user-defined")
  upgradeMechanism   : enum ("hard-fork" | "soft-
                       fork" | "smart-contract-
                       upgrade-pattern" |
                       "proxy-upgrade-eip-1822-1967"
                       | "user-defined")
  improvementProposalRegistryRef : string (URI of
                       the EIP / BIP / cosmos-IP /
                       per-network improvement-
                       proposal registry)
```

## §5 Cryptographic-Primitive Record

```
cryptoPrimitives:
  recordId           : string (uuidv7)
  hashAlgorithms     : array of enum ("sha-256" |
                       "sha-3-256" | "keccak-256" |
                       "blake2b" | "blake3" |
                       "user-defined")
  signatureAlgorithms : array of enum ("ecdsa-secp
                       -256k1" | "ecdsa-p-256" |
                       "ed25519" | "bls12-381" |
                       "schnorr-bip-340" |
                       "fips-186-5-eddsa" |
                       "ml-dsa-fips-204-pq" |
                       "user-defined")
  zkProofSystems     : array of enum ("groth16" |
                       "plonk" | "halo2" | "stark"
                       | "bulletproofs" |
                       "marlin" | "user-defined")
                       (zero-knowledge proof
                       systems used by the
                       network's privacy / scaling
                       layers)
  pqMigrationStatus  : enum ("not-pq-aware" |
                       "pq-readiness-evaluated" |
                       "pq-co-deployed" |
                       "pq-only" | "user-defined")
                       (post-quantum migration
                       posture per NIST FIPS 203 /
                       204 / 205)
```

## §6 W3C VC / DID Identity Record

```
identityRecord:
  identityId         : string (uuidv7)
  didMethod          : enum ("did-web" | "did-key"
                       | "did-jwk" | "did-ion" |
                       "did-pkh" | "did-cheqd" |
                       "user-defined")
  did                : string (the W3C DID URI per
                       RFC 3986)
  didDocumentRef     : string (URI of the DID
                       document conformant to W3C
                       DID Core 1.0)

verifiableCredential:
  vcId               : string (uuidv7)
  issuerDid          : string
  subjectDid         : string
  credentialType     : array of string (W3C VC type
                       URIs)
  proofKind          : enum ("data-integrity-proof
                       -2023" | "ed25519-signature
                       -2020" | "bbs-signature-2023"
                       | "json-web-signature-2020"
                       | "ecdsa-secp256k1-signature
                       -2019" | "user-defined")
  validityWindow     : object (issuance / expiration
                       per ISO 8601)
  statusListRef      : string (URI of the W3C VC
                       Bitstring Status List entry
                       for revocation)
```

## §7 Smart-Contract Definition and Audit Record

```
smartContract:
  contractId         : string (uuidv7)
  platformRef        : string (PHASE-1 §3)
  contractAddress    : string (the on-chain contract
                       address)
  bytecodeDigest     : string (cryptographic digest
                       of the deployed bytecode)
  sourceLanguage     : enum ("solidity" | "vyper" |
                       "ink-rust" | "rust-cosmwasm"
                       | "move-aptos" | "move-sui"
                       | "go-fabric-chaincode" |
                       "kotlin-corda-flow" |
                       "user-defined")
  sourceVersion      : string (the language /
                       compiler version)
  ercStandardsImplemented : array of enum ("erc-20"
                       | "erc-721" | "erc-1155" |
                       "erc-4626" | "erc-1271" |
                       "erc-2771-meta-tx" |
                       "erc-4337-aa" | "user-defined")
  formallyVerified   : boolean
  formalSpecRef      : string (URI of the formal
                       specification — Hoare logic,
                       K-framework, Dafny, Coq;
                       absent if not formally
                       verified)
  auditReportRefs    : array of string (URIs of the
                       independent security-audit
                       reports)
  upgradePattern     : enum ("non-upgradeable" |
                       "transparent-proxy-eip-1967"
                       | "uups-eip-1822" |
                       "diamond-eip-2535" |
                       "user-defined")
```

## §8 Wallet and Key-Management Record

```
walletRecord:
  walletId           : string (uuidv7)
  walletKind         : enum ("custodial-exchange-
                       hosted" | "custodial-platform
                       -hosted" | "non-custodial-
                       hot-software" | "non-
                       custodial-hardware-secure-
                       element" | "non-custodial-
                       multi-party-computation-mpc"
                       | "smart-contract-wallet-
                       erc-4337" | "multisig" |
                       "user-defined")
  keyDerivationPath  : string (BIP-32 / BIP-44
                       derivation path; absent for
                       contract wallets)
  hsmReference       : string (the HSM identifier
                       under FIPS 140-3 for
                       institutional custody)
  insurancePolicyRef : string (URI of the wallet's
                       insurance policy; absent for
                       non-custodial)
```

## §9 Token / Asset Chain-of-Custody Record

```
custodyEvent:
  eventId            : string (uuidv7)
  tokenAddress       : string (the on-chain token
                       contract)
  tokenStandard      : enum ("erc-20" | "erc-721" |
                       "erc-1155" | "fungible-other"
                       | "non-fungible-other" |
                       "user-defined")
  fromAddress        : string
  toAddress          : string
  amount             : object (UoM-quantified —
                       fungible quantity or NFT
                       tokenId)
  txHash             : string
  blockNumber        : integer
  blockTimestamp     : string (ISO 8601)
  custodyKind        : enum ("user-to-user" |
                       "user-to-contract" |
                       "contract-to-user" |
                       "contract-to-contract" |
                       "issuance-mint" | "burn" |
                       "user-defined")
```

## §10 Regulatory-Classification Record

```
regulatoryClassification:
  classificationId   : string (uuidv7)
  tokenAddressOrName : string
  classificationKind : enum ("eu-mica-asset-
                       referenced-token-art" |
                       "eu-mica-e-money-token-emt"
                       | "eu-mica-other-crypto-
                       asset" | "us-sec-investment
                       -contract-howey-test" |
                       "us-cftc-commodity" |
                       "us-fincen-cvc-msb" |
                       "kr-가상자산-가상자산이용자
                       보호법" | "kr-증권성-자본
                       시장법" | "user-defined")
  regulatoryAuthority : string (the operating
                       authority — EBA / ESMA /
                       Member-State NCA for EU; SEC
                       / CFTC / FinCEN for US; FSC
                       / FSS for KR)
  decisionRef        : string (URI of the regulator's
                       decision document or the
                       operator's published legal
                       analysis)
```

## §11 Conformance

Implementations claiming PHASE-1 conformance maintain
the records defined above for the operator's
introductory blockchain programme, satisfy the
operating jurisdiction's regulatory-classification
discipline, and preserve the records under the
applicable retention horizon (US SEC 17a-4 where
the operator is a regulated entity; EU MiCA Art 70
records-and-record-keeping; KR 가상자산이용자보호법
보존 의무).

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-blockchain-intro
- **Last Updated:** 2026-04-28
