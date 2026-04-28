# WIA-blockchain-intro PHASE 4 — INTEGRATION Specification

**Standard:** WIA-blockchain-intro
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a blockchain-intro operator
integrates with the systems that surround the
introduction / educational / pilot lifecycle: the
upstream DLT platform (mainnet, test-network, or
private deployment); the W3C VC / DID ecosystem
issuer / holder / verifier trio; the security-audit
firm ecosystem; the regulatory authority for the
operating jurisdiction; the on-ramp / off-ramp
fiat exchange; the Travel-Rule data-exchange network
(IVMS 101 / TRP / TRISA / Sygna); the open-source
project ecosystem (Ethereum, Hyperledger, Cosmos,
Polkadot foundations); the educational accreditation
body for educational operators; the external auditor
and ISO/IEC 27001 certification body; and the long-
term archive that preserves DLT-related operator
records past the active retention horizon.

References (CITATION-POLICY ALLOW only):

- ISO/TC 307 deliverables
- W3C VC 2.0 + DID Core 1.0
- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022, ISO/IEC 17021-1:2015, ISO/IEC
  17065:2012
- ISO 8601, ISO 17442 LEI
- NIST IR 8202 + IR 8301 + FIPS 203 / 204 / 205
- W3C Verifiable Credentials Data Model 2.0
- IVMS 101 (interVASP Messaging Standard 101)
- TRP / TRISA / OpenVASP / Sygna Bridge Travel-
  Rule technical solutions
- US SEC Framework for "Investment Contract"
  Analysis of Digital Assets (April 2019) + 17a-4
- US CFTC virtual currency derivatives oversight
- US FinCEN BSA + 31 CFR + FinCEN guidance
- EU MiCA Reg (EU) 2023/1114 + Reg (EU) 2024/859
  + Reg (EU) 2024/1153 AML extension + Reg (EU)
  2023/1113 Wire Transfer
- KR 가상자산이용자보호법 + 특정금융정보법 + KR FSC
  + FSS + FIU
- Ethereum Foundation governance + ERC / EIP process
- Hyperledger Foundation + LF Decentralized Trust
- Cosmos Hub governance + ICS / IBC
- Polkadot Treasury + OpenGov

---

## §1 Upstream DLT Platform Integration

The operator's DLT platform integration:

- Mainnet integration — for production-class
  deployments the operator runs full nodes (or
  uses an institutional RPC provider) with the
  operator's published reliability and latency
  SLOs.
- Test-network integration — for educational and
  pilot deployments the operator uses Sepolia,
  Holesky, Goerli (deprecated), Polygon Amoy,
  Cosmos Theta-Testnet, Polkadot Westend, etc.
- Private-network integration — for permissioned
  deployments the operator runs the network nodes
  under the consortium agreement.
- Per-platform improvement-proposal participation
  (EIP / BIP / cosmos-IP / etc.) for upstream
  protocol evolution.

## §2 W3C VC / DID Ecosystem Integration

The W3C VC / DID ecosystem integration:

- Issuer integration — credential schemas published
  to the operator's catalogue.
- Holder integration — wallet apps (smart wallets
  + non-custodial wallets) that store VCs.
- Verifier integration — third-party services
  consuming VCs for access decisions.
- Trust list — the operator's federated trust list
  publishes the issuers the operator's verifiers
  trust.

## §3 Security-Audit Firm Integration

For smart-contract operators:

- Bilateral engagement contracts with security-
  audit firms for pre-deployment audits.
- Per-audit findings tracked through the operator's
  remediation cycle.
- Bug-bounty programme operated through Immunefi /
  HackerOne / operator-internal channel.

## §4 Regulatory-Authority Integration

For US-jurisdiction operators:

- US SEC for digital-asset-securities
  classification, registration, and reporting.
- US CFTC for virtual-currency derivatives
  oversight.
- US FinCEN for MSB registration, BSA AML
  programme, and Travel-Rule compliance.
- US state regulators (NYDFS BitLicense,
  state-by-state money-transmission licences).

For EU-jurisdiction operators:

- EU EBA for stablecoin-issuer authorisation
  (asset-referenced and e-money tokens under
  MiCA).
- EU ESMA for crypto-asset trading-platform
  oversight under MiCA.
- EU Member-State NCA for CASP authorisation.
- EU AMLA (once operational) for AML supervision.

For KR-jurisdiction operators:

- KR FSC + FSS for VASP supervision under 가상
  자산이용자보호법.
- KR FIU for AML / Travel-Rule supervision under
  특정금융정보법.
- KR 한국인터넷진흥원 (KISA) for cybersecurity
  oversight under 정보통신망법.

## §5 Fiat On-Ramp / Off-Ramp Integration

For operators integrating fiat-crypto rails:

- Bank-partnership for fiat ACH / SEPA / wire
  rails.
- Card-acquirer partnership for card-funded
  on-ramps.
- Tier-1 exchange partnership for liquidity at
  off-ramps.
- Per-jurisdictional bank-account segregation
  under the user-protection regime.

## §6 Travel-Rule Network Integration

For VASP operators:

- IVMS 101 message format adoption.
- Bilateral or network membership in TRP / TRISA
  / OpenVASP / Sygna Bridge.
- Counterparty-VASP discovery via published
  registries.
- Per-corridor compliance review for the operator's
  approved-counterparty list.

## §7 Open-Source Foundation Integration

For operators contributing to or depending on open-
source projects:

- Ethereum Foundation — EIP authorship, client-
  team bilateral engagement.
- Hyperledger Foundation + LF Decentralized Trust
  — TSC participation, project-maintenance
  engagement.
- Cosmos Hub governance — IBC / ICS specification
  participation.
- Polkadot Treasury + OpenGov — governance
  proposal participation.

## §8 Educational Accreditation Integration

For educational operators:

- Curriculum review by the relevant accreditation
  body (regional / national accreditation per
  jurisdiction).
- Course-completion VC issuance using the
  Open Badges 3.0 + W3C VC 2.0 framework.
- Lifelong-learning credential portability via
  the operator's W3C DID / VC stack.

## §9 External Audit and ISMS Certification

The operator's ISMS is certified against ISO/IEC
27001:2022 with the scope explicitly extending to
the wallet, VC issuance, RPC adapter, and audit-log
endpoints. The certification body operates under
ISO/IEC 17021-1; the conformity-assessment body for
WIA-blockchain-intro operates under ISO/IEC 17065.
For digital-asset-custodian operators ISO/TR 23576-
aligned audit is exercised on the operator's
published cadence.

## §10 Long-Term Archival Integration

Records governed by the operator's retention horizons
(US SEC 17a-4 retention where the operator is a
regulated entity; EU MiCA Article 70 records-and-
record-keeping; KR 가상자산이용자보호법 보존 의무;
operator's internal retention) are migrated to the
long-term archive at the close of the active
retention window. The archive preserves the DLT-
platform identification snapshots, the VC / DID
records, the smart-contract audit reports, the
custody-event ledger, the regulatory-classification
records, the Travel-Rule message records, and the
audit-event trail.

## §11 Cross-Chain and Interoperability Integration

For operators with cross-chain interactions:

- IBC (Inter-Blockchain Communication) for Cosmos-
  ecosystem cross-chain.
- LayerZero / Hyperlane / Wormhole / Axelar for
  EVM-and-non-EVM cross-chain messaging.
- ISO/TR 3242 use-cases for interoperability —
  reference for the operator's interoperability
  design choices.
- Cross-chain bridge audits — given historical
  bridge exploits, audit cadence is more
  aggressive (quarterly minimum).

## §12 Climate-and-Sustainability Integration

For operators integrating sustainability disclosure:

- Per-network energy-consumption reporting (post-
  Merge Ethereum vs. Bitcoin; the orders-of-
  magnitude difference between PoS and PoW networks
  is a published comparison).
- Crypto Climate Accord / EEA Climate Working Group
  references for industry-wide reporting.
- Operator's WIA-esg-finance disclosure record
  integrates the DLT-sector attribution.

## §13 Real-World-Asset Tokenisation Integration

For operators bridging real-world assets to DLT:

- Tokenisation framework — the operator publishes
  the tokenised-asset class (treasuries, real-
  estate, equities, art) and the wrapper-contract
  audit reference.
- Custodian integration — the off-chain custodian
  holds the underlying asset; on-chain token
  represents the redemption claim.
- Oracle integration — Chainlink Proof of Reserves
  / Pyth Network / RedStone for off-chain
  attestation verifiability.
- Per-jurisdictional securities-classification
  review (US SEC + EU MiCA + KR 자본시장법).

## §14 Decentralised-Identity Federation Integration

For institutional adoption of decentralised
identity:

- EBSI (European Blockchain Services Infrastructure)
  for EU-citizen credential issuance.
- KR 분산신원 (DID) 시범사업 + KR PIPC guidance
  for KR-jurisdiction.
- US Federal IdM and state-level DLT-based
  driver's-licence pilots.
- ISO/IEC 18013-5 mDL (mobile driver's licence)
  cross-domain reference.

## §15 Conformance

Implementations claiming PHASE-4 conformance maintain
the DLT platform, W3C VC / DID ecosystem, audit-firm,
regulatory-authority, fiat-on-ramp, Travel-Rule, and
foundation integrations, hold the ISO/IEC 27001
certification + (for digital-asset-custodian
operators) ISO/TR 23576-aligned audit, exercise the
post-quantum-migration discipline cross-referenced
from PHASE-3, and operate the long-term archival
integration described above.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-blockchain-intro
- **Last Updated:** 2026-04-28
