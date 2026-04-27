# WIA-neuromorphic-chip PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-neuromorphic-chip
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-neuromorphic-chip. The standard covers spiking-neural-network
hardware accelerators that implement biologically-inspired event-driven
computation through silicon neurons, plastic synapses, and
address-event packet networks. The format captures the network
topology and parameters, the neuron and synapse model parameters, the
mapping from network to physical hardware, the address-event traffic,
the on-chip plasticity events, the per-die characterisation evidence,
and the operational telemetry that supports reproducibility and
external citation of neuromorphic-compute results.

References (CITATION-POLICY ALLOW only):

- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 11578 (UUID)
- ISO 8601 (date and time)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- IEEE Std 754-2019 (floating-point arithmetic; cited only for
  numerical-format definitions where parameters are stored in
  non-fixed-point representations)
- W3C XML Schema Definition 1.1 (legacy NIDM-style provenance import)

---

## §1 Scope

This PHASE document defines the persistent shapes for the records that
flow across the life of a neuromorphic deployment: design-time
network description, mapping to physical chip resources, on-chip
operation traffic, learning and plasticity events, characterisation
evidence, and field telemetry. It is intended for use by:

- Neuromorphic SDK developers that compile networks for deployment.
- Hardware vendors that publish device descriptions and consume
  compiled networks.
- Reference laboratories that characterise neuron and synapse
  responses against the published model parameters.
- System integrators that deploy neuromorphic accelerators in
  always-on sensing, robotics, low-power inference, and on-device
  learning contexts.
- Citation tools that resolve a published neuromorphic-compute
  result to its underlying records.

Conventional clocked digital accelerators that do not implement
event-driven spiking computation are out of scope; they are covered
by adjacent WIA standards.

## §2 Network Identifier

```
networkId         : string (uuidv7)
networkCreatedAt  : string (ISO 8601 / RFC 3339)
networkAuthor     : string (institutional author identifier)
networkPurpose    : enum  ("inference-only" | "online-learning" |
                       "sensorimotor-loop" | "research" |
                       "always-on-sensing")
neuronModel       : enum  ("lif" | "lif-with-refractory" | "adex" |
                       "izhikevich" | "hh-reduced" | "custom")
synapseModel      : enum  ("static" | "stdp" | "triplet-stdp" |
                       "voltage-dependent" | "neuromodulated" |
                       "biologically-inspired-other")
networkStatus     : enum ("draft" | "compiled" | "deployed" |
                       "deprecated")
```

A network's `neuronModel` and `synapseModel` define the contract that
the underlying hardware must support; networks compiled for a chip
that does not support the model return a Problem-Details (RFC 9457)
response of type
`urn:wia:neuromorphic-chip:model-unsupported` at compile time.

## §3 Network Description

```
networkDescription:
  networkId       : string (uuidv7)
  artefactRef     : string (content-addressed URI of the network
                       description; common formats are NIR, NeuroML2,
                       or a vendor-neutral graph format)
  populations     : array of NeuronPopulation
  projections     : array of SynapseProjection
  inputSources    : array of InputSource
  outputProbes    : array of OutputProbe

NeuronPopulation:
  populationId    : string
  size            : integer
  parameters      : object (model-specific: vReset, vThresh, tauM,
                       refractoryMs, ...)

SynapseProjection:
  projectionId    : string
  preId           : string (population identifier)
  postId          : string
  connectivity    : enum ("dense" | "sparse-fixed-fan-in" |
                       "sparse-fixed-fan-out" | "from-list")
  weightInit      : object (initialisation distribution parameters)
  plasticity      : object (synapse-model-specific parameters)

InputSource:
  sourceId        : string
  modality        : enum ("dvs-event-camera" | "spiking-cochlea" |
                       "tactile-event" | "rate-code-injection" |
                       "poisson-noise" | "user-defined")
  encodingRef     : string (encoding-recipe content-address)

OutputProbe:
  probeId         : string
  populationId    : string
  metric          : enum ("spike-times" | "membrane-voltage" |
                       "weight-snapshot" | "rate-window")
```

## §4 Hardware Description

```
hardware:
  hardwareId      : string (uuidv7)
  vendorClassRef  : string (URI of the vendor's published class
                       description; vendor-specific identifiers are
                       opaque to this standard)
  cores           : integer (number of physical cores)
  neuronsPerCore  : integer
  synapsesPerCore : integer
  fanInLimit      : integer (per-neuron upper bound)
  fanOutLimit     : integer
  weightBitsTotal : integer
  weightBitsTrained: integer (some platforms reserve part of the
                       weight word for plasticity state)
  modelSupport    : array of enum (neuron and synapse models the
                       hardware natively supports)
  numericalNotes  : object (e.g. dynamic-range constraints, IEEE 754
                       partial support)
```

Hardware description records are emitted by the vendor and consumed
by the SDK at compile time; they are signed by the vendor's release
key so that the SDK refuses to compile against a tampered description.

## §5 Mapping Record

The compiler emits a mapping that places populations onto cores,
projections onto routing fabric, and per-neuron parameters into
on-chip memory. The mapping is deterministic given the network, the
hardware, and the compiler version.

```
mapping:
  mappingId       : string (uuidv7)
  networkId       : string (uuidv7)
  hardwareId      : string (uuidv7)
  compilerVersion : string (Semantic Versioning 2.0.0)
  perCoreLoad     : array of CoreAssignment
  routingPlan     : array of RoutingEntry
  utilisation:
    neuronsUsed   : integer
    synapsesUsed  : integer
    routingHopsP95: integer
  fallbackEvents  : array of FallbackEvent (cases where the compiler
                       relaxed a network-side preference because the
                       hardware could not satisfy it)
```

Mappings whose `fallbackEvents` are non-empty are flagged in the
public catalogue so that downstream consumers know when the deployed
network differs from the original specification.

## §6 Address-Event Traffic Record

The hardware publishes a stream of address-events (AER spikes) during
operation. The traffic record carries either the full event stream
(for short experiments) or summary statistics over windows (for
long-running deployments).

```
aerStream:
  streamId        : string (uuidv7)
  mappingId       : string (uuidv7)
  startedAt       : string (ISO 8601 / RFC 3339)
  durationS       : number
  encoding        : enum ("aedat4" | "binary-flat" | "csv" |
                       "summary-windowed")
  artefactRef     : string (content-addressed URI of the stream archive)
  windowedSummary : array of WindowSummary (present iff encoding =
                       "summary-windowed")

WindowSummary:
  windowStart     : string (ISO 8601 / RFC 3339)
  windowDurationMs: integer
  perPopulationRateHz : object (rate per population identifier)
```

## §7 Plasticity Event Record

Networks that learn online emit plasticity events that update synaptic
weights. The plasticity-event record carries the trigger conditions
and the resulting weight change so that the trajectory of the network
can be reconstructed during audit.

```
plasticityEvent:
  eventId         : string (uuidv7)
  streamId        : string (uuidv7)
  occurredAt      : string (ISO 8601 / RFC 3339)
  projectionId    : string
  rule            : enum ("stdp" | "triplet-stdp" | "neuromodulated" |
                       "supervised-bp-through-time" | "vendor-rule")
  preTraceValue   : number
  postTraceValue  : number
  deltaWeight     : number
  modulator       : number (present for neuromodulated rules)
```

## §8 Characterisation Record

Per-die characterisation captures the deviation of the physical
hardware from its published model. Neurons in analogue or
mixed-signal implementations show device mismatch; synapses with
emerging memory technologies show variability and drift. The
characterisation record captures the measured deviation and the
calibration applied to mitigate it.

```
characterisation:
  characterisationId : string (uuidv7)
  hardwareId      : string (uuidv7)
  laboratoryId    : string (ISO/IEC 17025-accredited laboratory ID)
  performedAt     : string (ISO 8601)
  perCoreNeuronStats : array of NeuronStats
  perCoreSynapseStats: array of SynapseStats
  calibrationApplied : boolean
  calibrationArtefactRef : string (content-addressed URI of the
                       calibration table)
```

## §9 Field Telemetry Record

Deployed accelerators emit field telemetry covering operational
metrics: average and peak event rates, on-chip thermal observations,
power consumption per inference, error-correction rates on routing
links, and watchdog events. Telemetry is summary-only; raw AER traffic
is not appended to telemetry to avoid bandwidth explosion.

## §10 Workload Definition Record

Externally cited deployments report performance and energy under named
workloads. The workload-definition record carries the input stimulus,
the expected output behaviour, and the metrics under which the
deployment is evaluated.

```
workloadDefinition:
  workloadId      : string (uuidv7)
  artefactRef     : string (content-addressed URI of the workload bundle)
  domain          : enum ("event-camera-classification" |
                       "spiking-cochlea-keyword-spotting" |
                       "tactile-anomaly-detection" |
                       "always-on-wakeword" |
                       "robotics-sensorimotor-loop" |
                       "research-benchmark-suite" |
                       "user-defined")
  inputs          : array of WorkloadInput
  metrics         : array of WorkloadMetric

WorkloadInput:
  modality        : enum (matches PHASE-1 §3 inputSources.modality)
  dataDescription : object (timing, format, source dataset)
  preprocessing   : string (encoder recipe identifier)

WorkloadMetric:
  metric          : enum ("classification-accuracy" |
                       "false-positive-rate" |
                       "energy-per-inference-uj" |
                       "latency-to-first-spike-ms" |
                       "average-event-rate-hz" |
                       "weight-update-count")
  unit            : string
  expectedRangeLo : number (optional lower bound)
  expectedRangeHi : number (optional upper bound)
```

Workload definitions are content-addressed so that consumers can
re-execute the same workload against alternate hardware or alternate
mappings.

## §11 Energy Account Record

```
energyAccount:
  accountId       : string (uuidv7)
  mappingId       : string (uuidv7)
  workloadId      : string (uuidv7)
  measuredAt      : string (ISO 8601)
  perInferenceMicroJ  : number
  perSpikeNanoJ       : number
  idlePowerWatts      : number
  measurementMethod   : enum ("on-chip-current-shunt" |
                       "off-chip-power-monitor" |
                       "gpib-supply-meter")
  uncertainty         : Uncertainty (type-A and type-B per JCGM 100)
```

Energy claims that lack a corresponding energy-account record return
`409 Conflict` from the API at evidence-package time with type
`urn:wia:neuromorphic-chip:energy-claim-unsupported`.

## §12 Encoder and Decoder Recipes

Networks consume input through encoders that translate analog or
digital data into spike trains and emit output through decoders
that translate spike trains into actionable values. Encoder and
decoder recipes are content-addressed records that describe the
conversion function, the parameter range, and the canonical test
vectors that any implementation MUST pass to claim conformance.

```
encoderRecipe:
  recipeId        : string (uuidv7)
  family          : enum ("rate-coding" | "temporal-coding" |
                       "phase-coding" | "rank-order-coding" |
                       "delta-modulation" | "user-defined")
  parameterSchema : string (JSON Schema 2020-12 URI)
  testVectorRef   : string (content-addressed URI of canonical
                       input/output pairs)
  versionedAs     : string (Semantic Versioning 2.0.0)

decoderRecipe:
  recipeId        : string (uuidv7)
  family          : enum ("rate-decode" | "first-spike-decode" |
                       "winner-take-all" | "spike-time-population" |
                       "user-defined")
  parameterSchema : string (JSON Schema 2020-12 URI)
  testVectorRef   : string (content-addressed URI)
  versionedAs     : string
```

## §13 Provenance Record

Every record described in this PHASE carries a provenance entry that
names the entity that produced the record (vendor, SDK developer,
laboratory, integrator), the time of production, the cryptographic
signature, and the parent records that the record derives from. The
provenance record is the audit anchor that lets downstream consumers
trace a deployed mapping back to the network description, the
hardware description, the encoder recipes, and the workload that
together justified the deployment.

## §14 Conformance

Implementations claiming PHASE-1 conformance emit each of the records
defined above for every deployed network, honour the immutability and
content-addressing rules in §3-§8, and fail validation cleanly with
Problem Details (RFC 9457) responses when an export is requested for
a deployment that contains incomplete records.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-neuromorphic-chip
- **Last Updated:** 2026-04-27
