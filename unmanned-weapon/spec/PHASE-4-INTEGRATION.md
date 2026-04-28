# WIA-DEF-001 — Phase 4: Integration

> Unmanned-weapon canonical Phase 4: ecosystem integration (IHL + Geneva + UN CCW + LAWS + airworthiness + SOTIF + export).

# WIA-DEF-001: Unmanned Weapon Specification v1.0

> **Standard ID:** WIA-DEF-001
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Defense & Security Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Weapon Classification](#2-weapon-classification)
3. [Autonomy Levels](#3-autonomy-levels)
4. [Targeting Systems](#4-targeting-systems)
5. [Engagement Rules](#5-engagement-rules)
6. [Swarm Coordination](#6-swarm-coordination)
7. [Safety Protocols](#7-safety-protocols)
8. [Ethical Framework](#8-ethical-framework)
9. [Implementation Guidelines](#9-implementation-guidelines)
10. [References](#10-references)

---


## 8. Ethical Framework

### 8.1 International Humanitarian Law (IHL) Compliance

All systems must comply with:
1. **Geneva Conventions**: Protection of civilians and wounded
2. **Hague Conventions**: Limits on weaponry and methods
3. **CCW Protocol**: Prohibitions on specific weapon types

### 8.2 Principles of Warfare

#### 8.2.1 Distinction

Systems must distinguish between:
- Combatants and civilians
- Military and civilian objects
- Active threats and surrendering forces

**Implementation**:
```
if target_classification == "Civilian":
    prohibit_engagement()
elif target_classification == "Uncertain":
    escalate_to_human()
elif target_classification == "Combatant":
    verify_active_threat()
    proceed_if_authorized()
```

#### 8.2.2 Proportionality

Response must be proportional to threat:
```
if collateral_damage > military_advantage:
    prohibit_engagement()
    seek_alternative_method()
```

#### 8.2.3 Necessity

Use minimum force necessary:
```
weapon_options = [non_lethal, disabling, lethal]
for weapon in weapon_options:
    if weapon.can_neutralize(target):
        return weapon
```

#### 8.2.4 Humanity

Minimize suffering:
```
if target_incapacitated:
    cease_engagement()
    provide_aid_if_possible()
```

### 8.3 Accountability Framework

#### 8.3.1 Chain of Responsibility

1. **Manufacturer**: Technical compliance with standards
2. **Programmer**: Algorithm ethical compliance
3. **Commander**: Deployment authorization and ROE
4. **Operator**: Execution oversight (if HITL)
5. **System**: Autonomous decisions (logged and reviewable)

#### 8.3.2 Audit Trail Requirements

Complete logging of:
```
- All sensor inputs (stored 5 years minimum)
- Decision-making processes (explainable AI)
- Human interventions and overrides
- System malfunctions and errors
- Engagement outcomes and BDA
```

#### 8.3.3 Review and Investigation

Mandatory investigation if:
- Civilian casualties occur
- Friendly fire incident
- Unauthorized engagement
- System malfunction during operation
- Significant deviation from predicted behavior

### 8.4 Prohibited Actions

Absolutely forbidden:
1. Targeting civilians intentionally
2. Indiscriminate weapons use
3. Torture or cruel treatment
4. Perfidy (false surrender, misuse of protected symbols)
5. Attacks on medical facilities, personnel, or transports
6. Use against persons hors de combat (wounded, shipwrecked, surrendering)

### 8.5 Ethical AI Decision-Making

#### 8.5.1 Explainability Requirement

All autonomous decisions must be explainable:
```python
def explain_decision(decision_id):
    return {
        "decision": decision_type,
        "inputs": sensor_data_summary,
        "reasoning": step_by_step_logic,
        "alternatives_considered": other_options,
        "confidence": decision_confidence,
        "human_override": was_overridden
    }
```

#### 8.5.2 Bias Mitigation

Regular testing for algorithmic bias:
- Geographic bias (different regions)
- Demographic bias (racial, ethnic, gender)
- Temporal bias (day/night, weather)
- Context bias (urban vs. rural)

**Requirement**: Bias variance < 5% across all categories

#### 8.5.3 Value Alignment

System values prioritization:
```
1. Protect human life (friendly > neutral > hostile)
2. Minimize suffering
3. Complete mission objectives
4. Preserve system (self-preservation lowest priority)
```

---



## 10. References

### 10.1 International Law

1. Geneva Conventions (1949)
2. Additional Protocols I & II (1977)
3. Hague Conventions (1899, 1907)
4. UN Charter (1945)
5. Convention on Certain Conventional Weapons (CCW)

### 10.2 Technical Standards

1. NATO STANAG 4586: Standard Interfaces of UAV Control System (UCS)
2. ISO 21384: Unmanned Aircraft Systems
3. IEEE P7001: Transparency of Autonomous Systems
4. IEEE P7000: Model Process for Addressing Ethical Concerns
5. MIL-STD-1553: Digital Time Division Command/Response Multiplex Data Bus

### 10.3 Ethical Guidelines

1. ICRC Guidelines on Autonomous Weapon Systems
2. UN Convention on Certain Conventional Weapons (CCW)
3. IEEE Global Initiative on Ethics of Autonomous and Intelligent Systems
4. Future of Life Institute: Autonomous Weapons Open Letter
5. Campaign to Stop Killer Robots Principles

### 10.4 WIA Standards

- WIA-INTENT: Intent-based control interfaces
- WIA-OMNI-API: Universal API gateway
- WIA-AI: Artificial Intelligence ethics frameworks
- WIA-SOCIAL: Coordination and communication protocols

---

## Appendix A: Example Calculations

### A.1 Engagement Score Calculation

```
Given:
- Threat_Level: 0.92
- Target_Certainty: 0.96
- Proportionality: 9.5 (scale 0-10)
- IFF_Clear: 1 (true)
- ROE_Compliance: 1 (true)

Normalized Proportionality = 9.5 / 10 = 0.95

Engagement_Score = 0.92 × 0.96 × 0.95 × 1 × 1
                 = 0.839

Decision:
- Level 2: Human approval required
- Level 3: Below threshold (0.95), escalate to human
- Level 4: Below threshold (0.98), escalate to human

Action: Escalate to human operator for authorization
```

### A.2 Swarm Task Allocation

```
Scenario: 5 UAVs, 3 targets

Distance matrix (km):
        T1    T2    T3
UAV1   [2.5,  4.0,  6.2]
UAV2   [3.1,  2.8,  5.5]
UAV3   [5.0,  3.5,  2.0]
UAV4   [1.8,  5.2,  7.1]
UAV5   [4.5,  1.5,  3.8]

Optimal assignment (Hungarian algorithm):
- UAV1 → T1 (2.5 km)
- UAV2 → T2 (2.8 km)
- UAV3 → T3 (2.0 km)
- UAV4 → Reserve
- UAV5 → Reserve

Total distance: 7.3 km
```

### A.3 Collateral Damage Estimation

```
Scenario: Precision strike in urban area

Weapon: Small missile, 10kg warhead
Lethal radius: 15 meters
Injury radius: 50 meters

Civilian population estimate:
- 0-15m: 0 persons (evacuated)
- 15-50m: 12 persons estimated

P_casualty at 20m: 0.8
P_casualty at 30m: 0.4
P_casualty at 40m: 0.1

Estimated civilians: 4 @ 20m, 5 @ 30m, 3 @ 40m

CCE = (4 × 0.8) + (5 × 0.4) + (3 × 0.1)
    = 3.2 + 2.0 + 0.3
    = 5.5 expected casualties

Result: CCE = 5.5 persons (exceeds acceptable threshold)
Decision: ENGAGEMENT PROHIBITED
Alternative: Precision sniper system or delay until evacuation
```

---

**弘익人間 (홍익인간) · Benefit All Humanity**

*WIA-DEF-001 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*



---

## A.1 International-law cross-walk

| Concern                       | Standard                                  |
|-------------------------------|-------------------------------------------|
| IHL — distinction             | Additional Protocol I, Article 48         |
| IHL — proportionality         | Additional Protocol I, Article 51 (5)(b)  |
| IHL — precaution              | Additional Protocol I, Article 57         |
| Geneva Conventions            | Geneva I-IV (1949) + AP I-III             |
| CCW protocols                 | UN CCW 1980 + Protocols I-V               |
| Lethal autonomous weapons (LAWS) framework | UN GGE on LAWS (Geneva)      |
| Treaty of Ottawa              | Anti-Personnel Mine Ban Convention 1997   |
| Cluster Munitions Convention  | CCM 2008                                  |
| Outer Space Treaty            | UN Outer Space Treaty 1967                |
| Convention on Cyber Crime     | Budapest Convention 2001                  |

This cross-walk is informative; implementers obtain authoritative copies from the issuing body.

## A.2 Ethical framework integration

The ethics envelope at Phase 1 §A.4 cross-references three authoritative frameworks: the just-war tradition's jus in bello principles (distinction, proportionality, precaution) reflected in IHL; the IEEE Ethically Aligned Design v2 (autonomy with appropriate human oversight); and the operator's national/coalition-defence ethics policy (e.g., DoD Directive 3000.09, NATO AAP-06 doctrine). The envelope captures the ethics-board outcome (where the operator's process requires one), the dissent register, and the conditional-approval terms (such as required additional kill-switch redundancy or restricted geographic envelopes).

## A.3 Airworthiness and regulatory integration

Airworthiness integration follows the platform-domain authority: FAA 14 CFR Part 107 (US small UAS commercial); ASTM F38 standards (UAS design and operation); EASA SC-VTOL / Cat A/B/C UAS frameworks; KOCA UAS Type Certification; equivalents in JCAB / CAAC / DGCA. Maritime platforms follow IMO MSC.428(98) cyber-risk management plus the relevant flag-state regulations. Ground platforms follow ISO 13482 (personal-care robots, applied analogously) and the operator's vehicle-safety regulatory framework.

## A.4 Safety-of-the-intended-functionality (SOTIF) integration

SOTIF integration per ISO 21448 captures the unknown-unsafe and known-unsafe envelopes for autonomous functionality. The integration envelope at Phase 1 §A.5 carries the SOTIF analysis outcome, the residual-risk acceptance, the post-deployment monitoring plan, and the trigger conditions for re-running the SOTIF analysis (e.g., after a software update that touches the targeting decoder; after a sensor stack swap; after a doctrinal change to the ROE). SOTIF integrates with the platform's functional-safety envelope per IEC 61508 / ISO 26262 (automotive) / DO-178C (airborne software) as applicable.

## A.5 Future directions

Active research tracks: explainable-AI for engagement-decision support so operators can audit the reasoning rather than just the outcome; verifiable-compute proofs for autonomy-level certification; cross-domain heterogeneous swarms with consistent ROE enforcement; counter-swarm defensive systems with strict no-strike envelopes against civilian platforms; signature-management for stealthy ISR with adversary-aware obfuscation. **The standard explicitly prohibits operationalising fully-autonomous lethal engagement (Level 5) and tracks all Level 4 deployments with mandatory human-in-the-loop for the engagement decision** in line with the UN GGE on LAWS guiding principles. The standard's roadmap envelope (`POST /standards/v1/proposals`) tracks active proposals through the WIA Committee voting process per Phase 4 §Z.4.

## A.6 Reference list

- Additional Protocol I to the Geneva Conventions (1977) — Articles 48, 51, 57
- Geneva Conventions of 1949 (I, II, III, IV) plus AP I, II, III
- UN Convention on Certain Conventional Weapons 1980 + Protocols I-V
- UN GGE on Lethal Autonomous Weapons Systems (Geneva) — guiding principles
- IEEE Ethically Aligned Design v2 (2019)
- ISO 21384-1/-2/-3 — UAS general requirements, product systems, operations
- IEC 63402 — UGV functional safety (industrial mobile robots)
- ISO 21448 — SOTIF (Safety of the Intended Functionality)
- IEC 61508 — functional safety for E/E/PE safety-related systems
- DO-178C / DO-254 — airborne software / hardware certification
- IEC 60825-1 — laser-product safety
- DoD Directive 3000.09 — Autonomy in Weapon Systems
- ASTM F38 series — UAS standards
- FAA 14 CFR Part 107 — Small Unmanned Aircraft Rule
- IETF RFC 9325 — TLS recommendations
- NIST FIPS 203/204/205 — post-quantum cryptography (ML-KEM / ML-DSA / SLH-DSA)

## A.7 Export-control integration

Export-control integration captures the platform's classification under the relevant export-control regime: US ITAR USML Category VIII (aircraft) / Category XII (sensors) / Category XIII (auxiliary equipment); US EAR ECCN 9A012 / 9D004; EU Common Military List (CML) ML10 / ML11 / ML15; the Wassenaar Arrangement Munitions List ML10/ML15; the MTCR Annex Items 1-19. The export envelope carries the licensing authority's reference, the destination-country envelope, the end-use-monitoring envelope, and the post-shipment-verification schedule. Re-export requires a fresh authorisation and is logged in the platform's chain-of-custody record.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/unmanned-weapon/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-unmanned-weapon-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/unmanned-weapon-host:1.0.0` ships every unmanned-weapon envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/unmanned-weapon.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Unmanned-weapon deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
