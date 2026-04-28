# WIA-QUA-019 — Phase 3: Protocol

> Validation-protocol layer: the wire-level discipline by which a synthesis run claims room-temperature-superconductor status and downstream consumers can audit the claim.

## 7. Validation Protocols

### 7.1 Multi-Method Confirmation

**Required Measurements:**

**Tier 1 (Essential):**
1. **Zero Resistance:**
   - Four-point probe R(T)
   - R(T<Tc) / R(T>Tc) < 10⁻⁶
   - ΔTc < 5K

2. **Meissner Effect:**
   - SQUID magnetometry or VSM
   - χ < -0.9
   - Both ZFC and FC

3. **Reproducibility:**
   - ≥3 samples from same batch
   - Same Tc ± 2K
   - Same transition width

**Tier 2 (Highly Recommended):**
4. **Critical Current:**
   - Transport measurement
   - Jc > 10⁴ A/m² minimum

5. **Specific Heat Jump:**
   - C(T) anomaly at Tc
   - ΔC / γTc ratio measured

6. **Structural Characterization:**
   - XRD phase identification
   - Single phase or identify impurities

**Tier 3 (Confirmatory):**
7. **Isotope Effect:**
   - Δ(Tc) with isotope substitution
   - For hydrides: H → D substitution

8. **Gap Spectroscopy:**
   - STM, ARPES, or tunneling
   - Measure superconducting gap Δ

9. **Independent Verification:**
   - ≥3 independent groups
   - Different techniques
   - Peer-reviewed publication

### 7.2 Red Flags and Artifacts

**Warning Signs (Not Superconductivity):**

1. **Resistance Drop Without Zero:**
   - R decreases but doesn't reach < 10⁻⁶ R_normal
   - Likely: Metal-insulator transition, contact improvement

2. **Weak Diamagnetic Signal:**
   - χ > -0.1 (e.g., χ = -0.01)
   - Likely: Intrinsic diamagnetism, ferromagnetic impurity

3. **No ZFC/FC Difference:**
   - Both curves identical
   - Likely: Not superconducting, just diamagnetic

4. **Tc Depends on Measurement:**
   - Different Tc from resistance vs magnetization
   - Likely: Artifact or impurity phase

5. **Non-Reproducible:**
   - Sample-to-sample variation >10K in Tc
   - Likely: Contamination, metastable phases

6. **No Critical Current:**
   - Zero resistance but Jc = 0
   - Likely: Artifact or filamentary paths

**Common Artifacts:**
- **Percolation paths**: Conducting filaments, not bulk superconductivity
- **Impurity phases**: Small fraction of superconducting phase gives signals
- **Magnetic impurities**: Can mimic diamagnetism
- **Contact effects**: Apparent resistance drop at contacts

### 7.3 Reporting Standards

**Required Information in Publication:**

**Sample Details:**
- Synthesis method (exact protocol)
- Starting materials (purity, source)
- Sample dimensions and mass
- Expected composition and actual (if measured)
- Crystallographic phase (XRD)

**Measurement Conditions:**
- All instrument details
- Measurement parameters (current, field, etc.)
- Temperature calibration method
- Multiple techniques used

**Data Presentation:**
- Raw data shown (not just processed)
- Multiple samples (show reproducibility)
- Error bars and uncertainties
- Negative results disclosed

**Validation:**
- All validation criteria addressed
- Explanation of any missing measurements
- Comparison to known materials
- Independent confirmation status

---


## 9. Safety and Handling

### 9.1 High-Pressure Safety

**Diamond Anvil Cell Hazards:**
1. **Catastrophic Failure:**
   - Diamond breakage at high pressure
   - Explosive release of pressure
   - Projectile hazards

**Mitigation:**
- Protective enclosures (blast shields)
- Remote operation when possible
- Gradual pressure changes
- Regular diamond inspection

2. **Laser Heating Hazards:**
   - High-power laser (10-100 W)
   - Eye damage risk (Class 4 laser)
   - Skin burns

**Mitigation:**
- Laser safety goggles (OD 7+ at wavelength)
- Interlocks on enclosure
- Training and certification
- Beam dumps and blocks

### 9.2 Chemical Hazards

**LK-99 Synthesis:**
- **Lead compounds**: Toxic, teratogenic
- **Copper phosphide**: Toxic, flammable
- **Sulfates**: Respiratory irritant

**Mitigation:**
- Fume hood for all synthesis
- Protective equipment (gloves, coat, goggles)
- Proper waste disposal (heavy metal protocols)
- Minimize dust generation

### 9.3 High-Temperature Furnaces

**Hazards:**
- Burns from hot materials
- Thermal shock cracking
- Atmosphere control (inert gases)

**Mitigation:**
- Heat-resistant gloves
- Face shields
- Slow heating/cooling rates
- Gas monitors for leaks

---



## A.1 Validation protocol

A claim of room-temperature-superconductivity follows the validation
protocol:

1. Lab A publishes signed synthesis + characterisation envelopes
2. Lab B (independent, agreed-upon trust list) replicates the
   synthesis and publishes its own signed envelopes
3. Both labs publish a joint comparison envelope with documented
   agreement bounds
4. Independent reviewers audit the chain via the published audit log
5. The community accepts or rejects the claim based on the
   documented protocol outcome

This protocol exists because room-temperature-superconductivity
claims have historically been controversial; the standards
discipline forces the evidence chain into a verifiable shape from
day one.

## A.2 Replay defence and audit

Standard 96-bit nonce + 300-second skew window + 600-second
seen-nonce cache. Audit envelopes are written to an append-only
log replicated across at least two storage backends with retention
sized to the regulatory and academic-record requirements (typically
50 years for foundational physics records).


## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/room-temp-superconductor/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-room-temp-superconductor-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/room-temp-superconductor-host:1.0.0` ships every Phase 2 endpoint with mock
data for integrators to test against. The companion CLI at
`cli/room-temp-superconductor.sh` ships sample envelope generators with no dependencies
beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, signature, and
audit machinery rather than maintaining N parallel implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up reference
container; run conformance suite against it; replace mock backend
with real backend one endpoint at a time; wire audit log
replication; onboard a single trusted peer for federation; expand
to multiple peers; promote to production with warning-envelope
subscription.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the standards four-Phase architecture: Phase
1 envelopes are the wire-format contract; Phase 2 surfaces them
through HTTPS; Phase 3 wraps them in protocol exchanges that cross
trust boundaries; Phase 4 integrates with the broader ecosystem.

弘益人間 — Benefit All Humanity.


## A.3 Cross-laboratory replication discipline

The replication discipline is the load-bearing part of the
validation protocol. It comprises:

1. Lab A publishes its synthesis recipe, characterisation data, and
   raw measurement traces with full hash commitments.
2. The community proposes Lab B (independent, agreed-upon trust list).
3. Lab B attempts replication using only the published artefacts;
   Lab A may answer questions but may not provide additional unpublished
   information that would shortcut the test.
4. Lab B publishes its own envelopes regardless of replication outcome.
5. Both labs publish a joint comparison envelope documenting agreement
   bounds; the bounds appear in the audit log.

The discipline is rigorous because room-temperature-superconductor
claims have a public-credibility-cost that retracted physics papers
cannot recover. Standardising the protocol up-front protects honest
scientists from being entangled with bad-actor claims.

## A.4 Closing protocol note

The standard does not adjudicate which materials are or are not
room-temperature superconductors. It documents the wire format by
which any claim is published, replicated, and audited. Materials
science makes the determination; this standard makes the
determination process auditable.
