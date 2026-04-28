# WIA-ai-chip PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-ai-chip
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-ai-chip. The standard covers persistent record shapes
for AI accelerator silicon — chip identity and capability
registration, MLPerf-aligned benchmark publication, ONNX
model artefact compilation lineage, runtime telemetry
(per-inference latency, per-batch throughput, per-tenant
SLO compliance), thermal and power telemetry, firmware
and microcode revisions, security disclosures, and
end-of-life decommissioning. The format is consumed by
chip vendors, accelerator runtime stacks (CUDA, ROCm,
oneAPI, OpenAI Triton, TVM, vendor-specific compilers),
data-centre operators, hyperscale cloud providers, and
the regulators that supervise high-performance compute
deployment.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID)
- ISO/IEC 24029-1:2021 (assessment of robustness of neural
  networks — overview)
- ISO/IEC 24029-2:2023 (methodology for the use of formal
  methods)
- ISO/IEC 27001:2022 (information security management)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- MLCommons MLPerf Inference v4 (canonical benchmark suite
  for AI inference accelerators)
- MLCommons MLPerf Training (companion training-side
  benchmark)
- ONNX (Open Neural Network Exchange) for portable model
  artefacts
- Apache TVM (open-source ML compiler stack reference)
- OpenAI Triton (open-source language for accelerator
  kernel programming, cited as a representative kernel-
  level reference)
- PCI-SIG PCIe 6.0 base specification (host-attach
  electrical / link / protocol layer)
- CXL Consortium CXL 3.0 (Compute Express Link;
  cache-coherent host-accelerator memory protocol)
- NVLink fabric reference (proprietary high-bandwidth
  inter-chip fabric for accelerators that adopt it; cited
  as the operator records the fabric in use)
- UEFI Forum UEFI 2.10 + HII (Human Interface
  Infrastructure for accelerator BMC and firmware UI)
- IEEE 754-2019 (floating-point arithmetic; relevant for
  chip numerics conformance)
- OCP (Open Compute Project) Open Accelerator Module
  specification

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts an
AI-chip operator manages. Implementations covered include:

- Discrete AI-accelerator chip designs (GPU, TPU, NPU,
  ASIC, FPGA-as-accelerator).
- Accelerator-module form-factor implementations (PCIe
  add-in card, OCP OAM module, SXM module, custom
  socketed module).
- Vendor runtime stacks (CUDA, ROCm, oneAPI, vendor-
  proprietary stacks) and open compiler stacks (TVM,
  OpenXLA, Triton).
- ML model deployment platforms that consume the
  chip's runtime stack.
- Hyperscale cloud accelerator-fleet operators.

Pre-silicon design and verification (RTL development,
simulation, synthesis, place-and-route) is governed by
adjacent WIA standards and is out of scope here.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
programmeOperator    : string (institutional identifier of
                         the chip vendor, runtime vendor,
                         or accelerator-fleet operator)
programmeRegistered  : string (ISO 8601 / RFC 3339)
operationalRole      : enum ("silicon-vendor" |
                         "accelerator-module-vendor" |
                         "runtime-stack-vendor" |
                         "compiler-toolchain-vendor" |
                         "fleet-operator-hyperscale" |
                         "fleet-operator-enterprise" |
                         "research-deployment")
jurisdictionScope    : array of string (ISO 3166-1)
exportControlBinding : string (operator's per-jurisdiction
                         export-control attestation
                         reference; e.g. US BIS Entity
                         List screening, EU dual-use
                         regulation 2021/821, KR Strategic
                         Items export control)
programmeStatus      : enum ("design" | "tape-out" |
                         "engineering-sampling" |
                         "production" |
                         "deprecation-announced" |
                         "end-of-life" | "archived")
```

## §3 Chip Record

```
chip:
  chipId             : string (uuidv7)
  programmeId        : string (uuidv7)
  partNumber         : string (vendor part number)
  silicon Revision   : string (silicon stepping
                         identifier)
  fabricationProcess : string (e.g. "TSMC N4P", "Samsung
                         3nm GAA"; declared per the vendor's
                         disclosure policy)
  computeArchitecture: enum ("gpu-simt" | "gpu-rdna" |
                         "gpu-cdna" | "tpu-systolic" |
                         "npu-vliw" | "asic-domain-specific"
                         | "fpga-overlay" | "user-defined")
  precisionSupport   : array of enum ("fp64-ieee-754" |
                         "fp32-ieee-754" | "tf32-rounded" |
                         "fp16-ieee-754" | "bf16-google"
                         | "fp8-e5m2" | "fp8-e4m3" |
                         "int8-symmetric" |
                         "int4-quantised" |
                         "binary-neural-net")
  hostInterface      : enum ("pcie-5.0" | "pcie-6.0" |
                         "cxl-3.0" | "nvlink-fabric" |
                         "ocp-oam" | "sxm-module" |
                         "user-defined")
  registeredAt       : string (ISO 8601)
  endOfLifeDate      : string (ISO 8601; absent until
                         announced)
```

## §4 MLPerf Benchmark Result Record

```
mlperfResult:
  resultId           : string (uuidv7)
  chipRef            : string (chip UUID)
  benchmarkSuite     : enum ("MLPerf-Inference-v4-Datacenter"
                         | "MLPerf-Inference-v4-Edge" |
                         "MLPerf-Inference-v4-Mobile" |
                         "MLPerf-Training-v4" |
                         "MLPerf-Power-Inference-v4" |
                         "MLPerf-Tiny" |
                         "user-defined")
  workloadName       : string (per-suite workload name as
                         published by MLCommons; e.g.
                         "resnet50-v1.5", "bert-99",
                         "stable-diffusion-xl")
  scenario           : enum ("offline" | "server" |
                         "single-stream" | "multi-stream")
  submittedAt        : string (ISO 8601)
  resultArtefactRef  : string (URI of the MLCommons
                         results submission archive)
  publicationStatus  : enum ("submitted-pending" |
                         "verified-published" |
                         "withdrawn-by-submitter")
  submitterRef       : string (MLCommons submitter
                         identifier)
```

## §5 Compilation Lineage Record

```
compilationLineage:
  lineageId          : string (uuidv7)
  programmeId        : string (uuidv7)
  inputArtefactRef   : string (URI of the input model
                         artefact — typically ONNX, but
                         may be PyTorch state dict, JAX
                         pytree, TensorFlow SavedModel)
  inputDigest        : string (SHA-256)
  compilerToolchain  : enum ("tvm-relay-tir" |
                         "openxla-stablehlo" |
                         "triton-language" |
                         "vendor-cuda-graph" |
                         "vendor-rocm-graph" |
                         "vendor-oneapi-sycl" |
                         "vendor-proprietary" |
                         "user-defined")
  compilerVersion    : string
  compiledArtefactRef: string (URI of the compiled
                         binary)
  compiledDigest     : string (SHA-256)
  targetChipRef      : string (chip UUID the compilation
                         targets)
  optimizationProfile : enum ("inference-low-latency" |
                         "inference-high-throughput" |
                         "training-data-parallel" |
                         "training-tensor-parallel" |
                         "training-pipeline-parallel" |
                         "user-defined")
```

## §6 Runtime Telemetry Record

```
runtimeTelemetry:
  telemetryId        : string (uuidv7)
  chipRef            : string (chip UUID)
  capturedAt         : string (ISO 8601)
  intervalDurationS  : integer
  inferencesCompleted : integer (per-interval count)
  averageLatencyMs   : number
  p99LatencyMs       : number
  throughputInferencesPerS : number
  utilisationCoresPercent : number
  utilisationMemoryPercent : number
  hbmBandwidthGbPerS : number (high-bandwidth memory
                         bandwidth observed)
  thermalCoreCelsius : number (peak observed)
  thermalHbmCelsius  : number (peak observed)
  powerWatts         : number (average observed; per the
                         operator's chosen telemetry
                         endpoint, e.g. NVML, ROCm SMI,
                         BMC-side IPMI)
  faultCount         : integer (per-interval ECC and
                         system-fault count)
```

## §7 Firmware / Microcode Revision Record

```
firmwareRevision:
  firmwareId         : string (uuidv7)
  chipRef            : string (chip UUID)
  appliedAt          : string (ISO 8601)
  firmwareKind       : enum ("system-bmc" |
                         "accelerator-rom" |
                         "accelerator-microcode" |
                         "uefi-firmware" |
                         "vendor-runtime-driver")
  fromVersion        : string
  toVersion          : string
  firmwareArtefactRef: string (URI of the signed firmware
                         artefact; signing per the vendor's
                         firmware-signing policy)
  rollbackPossible   : boolean
```

## §8 Security Disclosure Record

```
securityDisclosure:
  disclosureId       : string (uuidv7)
  programmeId        : string (uuidv7)
  publishedAt        : string (ISO 8601)
  cveRef             : string (CVE identifier where one is
                         assigned)
  affectedSiliconRevisions : array of string (silicon
                         steppings affected)
  severityCvssV4     : number (CVSS v4 base score)
  exploitabilityClass : enum ("local-attacker-required" |
                         "remote-attacker-network" |
                         "physical-access-required" |
                         "side-channel-only" |
                         "supply-chain")
  remediationFirmwareRef : string (URI of the firmware
                         revision that remediates the
                         vulnerability)
```

## §9 Inter-Chip Fabric Topology Record

Multi-accelerator deployments form fabric topologies
(NVLink mesh, Infinity Fabric, OCP UALink, custom
fabrics) that affect collective-communication
performance.

```
fabricTopology:
  topologyId         : string (uuidv7)
  programmeId        : string (uuidv7)
  capturedAt         : string (ISO 8601)
  fabricKind         : enum ("nvlink" | "infinity-fabric"
                         | "ualink" | "cxl-3-cluster" |
                         "pcie-switch-fabric" |
                         "custom-rdma" |
                         "user-defined")
  chipMembers        : array of string (chip UUIDs in the
                         fabric)
  perLinkBandwidthGbPerS : number (per-link bandwidth)
  topologyShape      : enum ("all-to-all" | "ring" |
                         "tree-cascade" | "torus" |
                         "user-defined")
  hopCountMaxBetweenAny : integer
```

## §10 Per-Tenant SLO Compliance Record

```
sloCompliance:
  recordId           : string (uuidv7)
  chipRef            : string (chip UUID)
  tenantTokenRef     : string (opaque tenant token; clinical
                         tenant identity in the operator's
                         IDP)
  intervalStart      : string (ISO 8601)
  intervalEnd        : string (ISO 8601)
  slaLatencyMs       : number (per-tenant SLA target)
  observedP99LatencyMs : number (observed p99 over the
                         interval)
  slaThroughputInferencesPerS : number (per-tenant SLA
                         target)
  observedThroughputInferencesPerS : number
  slaCompliancePct   : number (interval-level compliance
                         percentage)
  breachEventCount   : integer (number of SLA breach events
                         in the interval)
```

## §12 Supply-Chain Provenance Record

```
supplyChainProvenance:
  recordId           : string (uuidv7)
  chipRef            : string (chip UUID)
  capturedAt         : string (ISO 8601)
  waferLotRef        : string (foundry-issued wafer-lot
                         identifier)
  packageAssemblyRef : string (advanced-packaging facility
                         identifier; relevant for COWOS /
                         FOWLP / 2.5D / 3D packaging)
  burnInTestRef      : string (URI of the burn-in test
                         report)
  serialisedFingerprintRef : string (URI of the chip's
                         physically-unclonable-function
                         fingerprint capture, where the
                         chip provides one)
  custodyChain       : array of object (per-handoff
                         counterparty + handoff-time +
                         shipping-document reference)
```

## §13 Microcode Update Detail Record

```
microcodeUpdate:
  updateId           : string (uuidv7)
  firmwareRevisionRef: string (PHASE-1 §7 firmware UUID
                         that this microcode is part of)
  performanceDeltaRef : string (URI of the performance-
                         delta report against the
                         operator's workload portfolio)
  cvesAddressed      : array of string (CVE identifiers
                         the microcode addresses; absent
                         where the update is purely
                         performance / capability)
  rolloutStagingPlanRef : string (URI of the staged
                         rollout plan honouring the
                         customer-notification SLA)
```

## §15 Side-Channel Evaluation Record

```
sideChannelEval:
  evalId             : string (uuidv7)
  chipRef            : string (chip UUID)
  evaluatedAt        : string (ISO 8601)
  threatVector       : enum ("timing-cache" |
                         "power-analysis-spa-dpa" |
                         "electromagnetic-emanation" |
                         "memory-readback" |
                         "speculative-execution-leak" |
                         "user-defined")
  evaluatorRef       : string (per ETSI ISG SAI / per
                         operator's chosen evaluator)
  resultClass        : enum ("no-leakage-detected" |
                         "leakage-within-mitigated-bound"
                         | "leakage-actionable" |
                         "inconclusive")
  remediationFirmwareRef : string (URI of the firmware
                         revision that mitigates; absent
                         where mitigation is purely
                         operational)
```

## §16 Conformance

Implementations claiming PHASE-1 conformance emit each of
the records defined above for every operating chip and
honour the MLCommons MLPerf Inference v4 result format
in §4.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-ai-chip
- **Last Updated:** 2026-04-28
