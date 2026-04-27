# WIA-photonic-chip PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-photonic-chip
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for WIA-photonic-chip.
The standard covers integrated photonic circuits fabricated on silicon,
silicon-nitride, indium-phosphide, lithium-niobate, and emerging photonic
platforms. The format captures the photonic-circuit netlist exchange,
process-design-kit (PDK) references, photonic-component characterisation
(modal indices, propagation losses, phase-shifter responses, coupler
splitting ratios), wafer-level test data, packaging metadata, and
post-fabrication measurement evidence used to certify a deployed
photonic chip against its design.

References (CITATION-POLICY ALLOW only):

- ISO 10110 (optics and photonics — preparation of drawings)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 11578 (UUID)
- ISO 8601 (date and time)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- IEC 62496-2 (optical circuit boards — interface)
- IEC 60825-1 (safety of laser products — equipment classification)
- W3C XML Schema Definition 1.1 (legacy GDS interchange envelope)
- IEEE Std P3120 (cited only for terminology where applicable)

---

## §1 Scope

This PHASE document defines the persistent shapes for the records that
flow across the life of a photonic chip: schematic capture, layout
output, process kit binding, fabrication run, wafer-level test, die
selection, packaging, system-level test, and field operation
telemetry. It is intended for use by:

- Photonic design houses that exchange schematics and layouts with
  fabrication foundries.
- Foundries that publish process design kits (PDKs) and consume
  customer layouts for tape-out.
- Wafer-test laboratories that produce per-die test reports.
- Packaging facilities that bind chips into modules.
- System integrators that deploy photonic modules into transceivers,
  spectrometers, LiDAR, sensing front-ends, or photonic compute
  fabrics.
- Reference laboratories that re-measure deployed modules and certify
  their conformance to the design.

Free-space optical assemblies, fibre-only systems, and discrete laser
diodes that do not enter a photonic IC are out of scope; they are
covered by adjacent WIA standards.

## §2 Design Identifier

```
designId          : string (uuidv7)
designCreatedAt   : string (ISO 8601 / RFC 3339)
designAuthor      : string (institutional author identifier)
platform          : enum  ("silicon" | "silicon-nitride" |
                       "indium-phosphide" | "lithium-niobate-thin-film" |
                       "iii-v-on-si" | "polymer" | "diamond")
operatingBand:
  centralWavelengthNm : number
  fractionalBandwidth  : number (dimensionless)
  polarization         : enum ("te" | "tm" | "polarization-diverse")
designStatus      : enum ("draft" | "tape-out" | "fabricated" |
                       "characterised" | "deployed" | "deprecated")
```

## §3 Process Design Kit (PDK) Reference

Every design pins the PDK against which it was created. PDKs are
versioned and a re-build of the design against a new PDK version
emits a derivative design rather than overwriting the original.

```
pdkReference:
  pdkId           : string (foundry-assigned PDK identifier)
  pdkVersion      : string (Semantic Versioning 2.0.0)
  cellLibraryRef  : string (content-addressed URI of the cell library)
  drcDeckRef      : string (URI of the design-rule deck)
  lvsDeckRef      : string (URI of the layout-versus-schematic deck)
  modelLibraryRef : string (URI of the compact-model library that
                       drives schematic-level simulation)
```

## §4 Schematic and Component Records

```
schematic:
  designId        : string (uuidv7)
  artefactRef     : string (content-addressed URI of the schematic file;
                       SPICE-photonic / OpenIPKISS-equivalent format)
  components      : array of ComponentInstance

ComponentInstance:
  refdes          : string (component reference designator,
                       e.g. "MZI1", "DC4", "PD2")
  cellName        : string (PDK cell name, e.g. "wg_strip_te1550")
  parameters      : object (cell-specific parameters: lengths, gaps,
                       grating periods, ring radii, etc.)
  ports           : array of string (port labels)
```

## §5 Layout Record

```
layout:
  designId        : string (uuidv7)
  artefactRef     : string (content-addressed URI of the GDSII or OASIS
                       file)
  reticleSiteId   : string (foundry reticle-slot identifier; assigned
                       at tape-out)
  drcResultRef    : string (URI of the DRC report)
  lvsResultRef    : string (URI of the LVS report)
  estimatedArea   : object (length-unit × length-unit, in micrometres)
```

DRC and LVS results MUST accompany every layout; layouts whose decks
have not run cleanly may not be tape-out-ready and the foundry MUST
refuse acceptance with type
`urn:wia:photonic-chip:drc-lvs-incomplete`.

## §6 Fabrication Run Record

```
fabricationRun:
  runId           : string (uuidv7)
  designId        : string (uuidv7)
  foundryId       : string (foundry institutional identifier)
  startedAt       : string (ISO 8601)
  completedAt     : string (ISO 8601)
  waferLot        : string (foundry wafer-lot identifier)
  reticleSiteId   : string (references §5)
  processSplit    : string (recipe code for the process variant)
  yieldEstimate   : YieldEstimate (per-die count by category)
```

Fabrication-run records are emitted by the foundry into the operating
programme's API; the foundry retains the canonical run information in
its own systems. The programme persists what it needs for downstream
testing and citation.

## §7 Wafer-Level Test Record

```
waferTest:
  waferTestId     : string (uuidv7)
  runId           : string (uuidv7)
  laboratoryId    : string (ISO/IEC 17025-accredited laboratory ID)
  startedAt       : string (ISO 8601)
  completedAt     : string (ISO 8601)
  testRecipeRef   : string (content-addressed URI of the test recipe)
  perDieResults   : array of DieResult

DieResult:
  dieId           : string
  reticleX        : integer
  reticleY        : integer
  pass            : boolean
  failureCategory : enum ("cleave-edge" | "alignment" |
                       "metal-defect" | "waveguide-loss-out-of-spec" |
                       "ring-resonance-out-of-spec" |
                       "phase-shifter-out-of-spec" | "other")
  measurements    : array of ComponentMeasurement
```

## §8 Component Measurement Record

```
componentMeasurement:
  measurementId   : string (uuidv7)
  dieId           : string
  refdes          : string (matches §4 reference designator)
  metric          : enum ("propagation-loss" | "modal-index" |
                       "splitting-ratio" | "extinction-ratio" |
                       "fsr" | "q-factor" | "phase-shifter-vpi" |
                       "responsivity" | "dark-current")
  unit            : string (metric-specific units)
  value           : number
  uncertainty     : Uncertainty (type-A and type-B per JCGM 100)
  conditions      : object (temperature, wavelength sweep, drive
                       current, etc.)
```

Programmes that publish externally cited per-component metrics MUST
emit the per-die measurement records that support the population
statistics; aggregate-only emission is insufficient for citation.

## §9 Packaging Record

```
package:
  packageId       : string (uuidv7)
  dieId           : string
  packageType     : enum ("co-packaged-optics" | "fibre-attached" |
                       "free-space-coupling" | "wafer-level-fanout" |
                       "ceramic-leadframe")
  thermalInterface: enum ("epoxy" | "indium" | "metal-bonded" |
                       "tim-paste")
  fibreAttachLoss : number (dB; absent for non-fibre packages)
  laserSafetyClass: enum ("1" | "1M" | "2" | "2M" | "3R" | "3B" | "4")
                       (per IEC 60825-1)
  packageRef      : string (content-addressed URI of the package
                       drawing per ISO 10110 or the equivalent)
```

## §10 Field Telemetry Record

Deployed modules emit telemetry suitable for monitoring drift and
end-of-life. The telemetry record carries periodic summary
observations rather than raw waveforms.

```
telemetry:
  telemetryId     : string (uuidv7)
  packageId       : string (uuidv7)
  capturedAt      : string (ISO 8601)
  samples         : array of TelemetrySample

TelemetrySample:
  metric          : string (e.g. "thermal-shift-pm-per-K",
                       "tx-power-dbm", "rx-current-ua",
                       "wavelength-locker-error")
  value           : number
  unit            : string
```

## §11 Wavelength Plan and Channel Assignment

Modules that operate on wavelength-division-multiplexed channels carry
a wavelength-plan record that names the standard channel grid the
module targets and the per-channel acceptance window.

```
wavelengthPlan:
  planId          : string (uuidv7)
  designId        : string (uuidv7)
  gridFamily      : enum ("dwdm-50ghz" | "dwdm-100ghz" | "dwdm-200ghz" |
                       "cwdm-20nm" | "lan-wdm" | "custom")
  channelMap      : array of ChannelAssignment

ChannelAssignment:
  channelIndex    : integer
  centreWavelengthNm : number
  acceptanceWindowGhz : number (full-width acceptance window)
  associatedRefdes: string (which schematic component services this
                       channel; e.g. "AWG1.out[3]")
```

Wavelength-plan records are exchanged with downstream optical-network
designers so that the deployed module's wavelength assignments can be
verified against the network plan; mismatches are surfaced as a
Problem-Details response of type
`urn:wia:photonic-chip:wavelength-plan-mismatch`.

## §12 Phase-Shifter Calibration Record

Thermal and electro-optic phase shifters drift over time and require
periodic re-calibration. The calibration record carries the
shifter-by-shifter response curve at deployment and at each
re-calibration event.

```
phaseShifterCalibration:
  calibrationId   : string (uuidv7)
  packageId       : string (uuidv7)
  performedAt     : string (ISO 8601 / RFC 3339)
  shifters        : array of PhaseShifterPoint

PhaseShifterPoint:
  refdes          : string
  controlMode     : enum ("thermal" | "carrier-injection" |
                       "carrier-depletion" | "electro-optic-pockels" |
                       "mems")
  controlSweep    : array of number (drive levels at which response
                       was sampled)
  phaseResponseRad: array of number (measured phase per drive level)
  hysteresisRad   : number (worst-case forward-vs-reverse hysteresis)
```

## §13 Loss-Budget Record

Photonic systems are dimensioned by loss budgets that aggregate per-
component contributions to total optical-path loss. The loss-budget
record captures the budgeted contribution and the measured
contribution side-by-side so that downstream consumers can identify
which component is responsible when a deployed module under-performs
its budget.

```
lossBudget:
  budgetId        : string (uuidv7)
  designId        : string (uuidv7)
  pathRefdes      : string (start-end refdes pair, e.g.
                       "GC1-out -> AWG1.in")
  budgetedDb      : array of LossContribution (per-component)
  measuredDb      : array of LossContribution (per-component, when
                       measurements are available)
  totalBudgetedDb : number
  totalMeasuredDb : number
  marginDb        : number (totalBudgetedDb - totalMeasuredDb)

LossContribution:
  refdes          : string
  contributionDb  : number
  conditions      : object (wavelength, polarization, temperature)
```

Loss-budget records are emitted at design time (`measuredDb` empty)
and updated as wafer-test and system-level measurements arrive.
Modules whose measured loss exceeds the budgeted loss MUST be flagged
in the public catalogue so that downstream integrators can plan
re-budgeting or component substitution.

## §14 Reliability Test Record

Long-term reliability is established through accelerated-aging
campaigns (high-temperature operating life, high-temperature humidity
storage, temperature-cycling, mechanical shock, and modality-specific
photonic tests). The reliability record captures the test conditions
and the outcome.

```
reliability:
  reliabilityId   : string (uuidv7)
  designId        : string (uuidv7)
  packageId       : string (uuidv7; absent for unpackaged-die tests)
  testFamily      : enum ("hotl" | "hths" | "tc" | "shock" |
                       "vibration" | "mechanical-pull-test" |
                       "fibre-pull-test" | "laser-burn-in")
  conditions      : object (test-specific: temperature, humidity,
                       cycle count, shock g-force, fibre-pull force)
  population      : integer
  passCount       : integer
  failCount       : integer
  observations    : string (free text describing failure modes)
```

## §15 Polarization Diversity Record

Modules that handle polarization-diverse signals (coherent receivers,
polarization-multiplexed transmitters, polarization-insensitive
sensors) carry a polarization-diversity record that describes the
splitter, rotator, and combiner topology used to handle both
polarization eigenstates. The record names the polarization-handling
strategy ("PSR-then-process", "polarization-insensitive design",
"polarization-tracking-feedback") and the per-eigenstate insertion
loss and crosstalk metrics observed during characterisation.

## §16 Conformance

Implementations claiming PHASE-1 conformance emit each of the records
defined above for every fabricated chip and honour the
content-addressing rules in §3, §5, §7, and §8.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-photonic-chip
- **Last Updated:** 2026-04-27
