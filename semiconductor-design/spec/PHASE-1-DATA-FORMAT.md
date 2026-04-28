# WIA-SEMI-DESIGN-001 — Phase 1: DATA-FORMAT

> Semiconductor Design canonical Phase 1 specification per the WIA Standards four-Phase architecture.

> Domain: 반도체 설계 — 반도체 회로 설계 · IP 코어 · DRC/LVS · 검증 · 테이프아웃.

## A.1 Scope

This Phase covers the canonical data-format layer of the WIA-SEMI-DESIGN-001 standard. It composes with the Phase 2 document for cross-Phase integration and inherits cross-standard behaviour from the wider WIA Standards family per the README.md scope statement.

## A.2 Normative references

- IEEE 1801-2024 (UPF Power Intent)
- IEEE 1685-2022 (IP-XACT)
- Accellera SystemVerilog 2017 + UVM 1.2
- IEC 61508-3 (Functional safety)
- ISO 26262-11 (Hardware in road vehicles)
- IEEE P2851 (Functional Safety Reference)
- ITU-T X.1054 (Information security gov)
- IEC 62443-4-1 (Secure product development)
- OASIS SBOM CycloneDX 1.6

## # WIA-SEMI-001: Semiconductor Design Standard - Technical Specification v1.0

**Status**: Draft
**Version**: 1.0.0
**Date**: 2025-01-15
**Authors**: WIA Standards Committee

---


## 1. Introduction

### 1.1 Purpose

The purpose of WIA-SEMI-001 is to standardize semiconductor design flows, enabling:
- **Interoperability**: Seamless integration between different EDA tools
- **Portability**: Designs that can be migrated across tool chains
- **Reusability**: IP cores and design components that work across projects
- **Quality**: Consistent design practices leading to better results

### 1.2 Background

Modern semiconductor design involves complex interactions between multiple tools, languages, and methodologies. The lack of standardization leads to:
- Vendor lock-in
- Manual translation between formats
- Inconsistent design quality
- Difficult IP reuse

WIA-SEMI-001 addresses these challenges through open, well-defined standards.

### 1.3 Design Principles

- **Openness**: Specifications are publicly available
- **Vendor Neutrality**: No preference for specific tool vendors
- **Backward Compatibility**: Support for existing industry formats
- **Extensibility**: Ability to add new features without breaking existing implementations

---


## 5. RTL Design Standards

### 5.1 Coding Style

#### 5.1.1 General Guidelines

All RTL code SHALL follow these principles:

1. **Readability**: Code should be self-documenting
2. **Maintainability**: Easy to modify and extend
3. **Synthesizability**: Translatable to hardware
4. **Reusability**: Parameterized and modular

#### 5.1.2 Naming Conventions

```verilog
// Module names: lowercase with underscores
module fifo_sync_ram (...);

// Parameters: UPPERCASE with underscores
parameter DATA_WIDTH = 32;
parameter DEPTH = 16;

// Signals:
//   - Inputs: lowercase with underscores
//   - Outputs: lowercase with underscores
//   - Internal: lowercase with underscores
//   - Active-low: suffix _n
input  wire clk;
input  wire rst_n;
output wire valid;

// Constants: UPPERCASE
localparam IDLE_STATE = 2'b00;
```

#### 5.1.3 File Organization

```
rtl/
├── top.sv              # Top-level module
├── core/
│   ├── cpu.sv
│   ├── alu.sv
│   └── regfile.sv
├── memory/
│   ├── cache.sv
│   └── sram.sv
└── peripherals/
    ├── uart.sv
    └── spi.sv
```

### 5.2 RTL Constructs

#### 5.2.1 Always Blocks

**Combinational Logic**:
```verilog
// Use always_comb (SystemVerilog) or always @(*)
always_comb begin
  sum = a + b;
  carry = (a & b) | (b & cin) | (a & cin);
end
```

**Sequential Logic**:
```verilog
// Use always_ff (SystemVerilog) or always @(posedge clk)
always_ff @(posedge clk or negedge rst_n) begin
  if (!rst_n)
    q <= '0;
  else
    q <= d;
end
```

#### 5.2.2 Reset Strategy

All registers SHALL have explicit reset logic:

```verilog
// Asynchronous reset (preferred for FPGAs)
always_ff @(posedge clk or negedge rst_n) begin
  if (!rst_n)
    state <= IDLE;
  else
    state <= next_state;
end

// Synchronous reset (preferred for ASICs)
always_ff @(posedge clk) begin
  if (!rst_n)
    state <= IDLE;
  else
    state <= next_state;
end
```

#### 5.2.3 Clock Domain Crossing

CDC SHALL use recognized safe techniques:

```systemverilog
// Two-flop synchronizer
always_ff @(posedge clk_dst) begin
  sync_ff1 <= signal_src;
  sync_ff2 <= sync_ff1;
end
assign signal_dst = sync_ff2;

// For control signals only, not buses!
```

### 5.3 IP-XACT Integration

Design modules SHALL provide IP-XACT metadata:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<ipxact:component>
  <ipxact:vendor>wia-official.org</ipxact:vendor>
  <ipxact:library>standard_cells</ipxact:library>
  <ipxact:name>fifo_sync</ipxact:name>
  <ipxact:version>1.0.0</ipxact:version>
  <ipxact:busInterfaces>
    <ipxact:busInterface>
      <ipxact:name>clk</ipxact:name>
      <ipxact:busType>clock</ipxact:busType>
    </ipxact:busInterface>
  </ipxact:busInterfaces>
  <ipxact:model>
    <ipxact:views>
      <ipxact:view>
        <ipxact:name>rtl</ipxact:name>
        <ipxact:fileSetRef>rtlFiles</ipxact:fileSetRef>
      </ipxact:view>
    </ipxact:views>
  </ipxact:model>
</ipxact:component>
```

---


## 9. API Specifications

### 9.1 Design Database API

```typescript
interface DesignDatabase {
  // Design creation
  createDesign(name: string, config: DesignConfig): Design;

  // Module operations
  addModule(design: Design, module: Module): void;
  getModule(design: Design, name: string): Module;

  // Hierarchy navigation
  getTopModule(design: Design): Module;
  getSubmodules(module: Module): Module[];

  // Signal queries
  getSignals(module: Module, filter?: SignalFilter): Signal[];

  // Netlist operations
  exportNetlist(design: Design, format: 'verilog' | 'vhdl'): string;
}
```

### 9.2 Simulation API

```typescript
interface Simulation {
  // Setup
  compile(files: string[], options: CompileOptions): void;
  elaborate(top: string): void;

  // Execution
  run(cycles: number): SimulationResult;
  runUntil(condition: string): SimulationResult;

  // Debug
  setBreakpoint(time: number | string): void;
  dumpWaveform(file: string, signals?: string[]): void;

  // Coverage
  getCoverage(): CoverageReport;
}
```

### 9.3 Synthesis API

```typescript
interface Synthesis {
  // Configuration
  setTechnology(library: string): void;
  setConstraints(sdc: string): void;

  // Optimization
  setGoal(goal: 'area' | 'power' | 'performance' | 'balanced'): void;

  // Execution
  synthesize(): SynthesisResult;

  // Reporting
  getTimingReport(): TimingReport;
  getPowerReport(): PowerReport;
  getAreaReport(): AreaReport;
}
```

---

---

## Z.1 Audit transport and observability hooks (Phase 1)

Every Phase 1 envelope SHOULD emit a structured log line at the
host's audit transport: timestamp per RFC 3339, host identifier,
tenant identifier, envelope class, envelope identifier, operation
outcome, and a W3C Trace Context `traceparent` propagated end-to-end
so a single operation can be reconstructed across hosts. Phase 2
surfaces this trace identifier as the `X-WIA-Trace-Id` response
header. Phase 3 protocol exchanges propagate the trace identifier
inside the exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (Splunk, Elastic, Sumo Logic,
Wazuh, Microsoft Sentinel) per OpenTelemetry semantic conventions,
with `wia.standard.slug` = `semiconductor-design` and `wia.standard.phase` =
`1` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 1)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 Sec 5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations. The composition also lets the operator's SIEM
correlate per-tenant audit records across multiple standards
without per-standard schema-mapping work.

## Z.3 Capabilities discovery and SemVer (Phase 1)

Hosts SHOULD publish a capabilities document at
`/.well-known/wia-semiconductor-design-capabilities` enumerating per-endpoint
optionality. Clients MUST treat unsupported capabilities as absent
rather than as an error condition; a client that needs a capability
the host does not advertise MUST surface a clear configuration
error rather than silently degrade. Hosts moving from one minor
version to the next MUST publish the change in the host's release
notes with the per-capability migration window per IETF RFC 8594
(Sunset header) + RFC 9745 (Deprecation header) + RFC 9651
(Structured Field Values) so machine consumers can plan migration
without waiting for human-channel notification.

## Z.4 Privacy envelope per per-jurisdiction law (Phase 1)

Phase 1 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California
CPRA per Cal. Civ. Code Sec 1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Subject-rights endpoints (access, rectification,
erasure, portability, restriction, objection) compose with
WIA-OMNI-API Sec 5 subject-rights surface and need not be
re-implemented per-standard.

## Z.5 DR / continuity envelope per ISO 22301 (Phase 1)

Hosts running this Phase MUST publish a continuity-of-operations
envelope per ISO 22301:2019 + ISO/IEC 27031 + NIST SP 800-34 Rev 1
covering: per-host RTO (Recovery Time Objective) per the operator's
business-impact analysis; per-host RPO (Recovery Point Objective)
tied to the host's audit-stream replication policy; per-host
backup envelope (per-region cross-replicated immutable backup with
the per-tier retention envelope per the per-jurisdiction record-
retention policy); per-host failover-rehearsal envelope (typically
quarterly per the operator's BC/DR program); per-host vendor-exit
envelope so the operator can migrate the host to an alternate
implementation without losing audit-trail continuity.

## Z.6 Supply-chain envelope per SLSA (Phase 1)

Every host implementation MUST publish a software-bill-of-materials
(SBOM) per SPDX 2.3 / 3.0 (per ISO/IEC 5962 + Linux Foundation SPDX)
or CycloneDX 1.6 (per OWASP Foundation). The SBOM enumerates every
direct + transitive dependency with the per-component name +
version + licence + supplier + per-component hash + per-component
PURL (Package URL per package-url spec) + per-component CPE
(Common Platform Enumeration per NIST). Supply-chain attestation
follows in-toto per CNCF in-toto + SLSA (Supply-chain Levels for
Software Artifacts) per OpenSSF SLSA Framework — typically targeting
SLSA Level 3 for hosted production deployments.

弘益人間 — Benefit All Humanity.

---

## Z.1 Audit transport and observability hooks (Phase 1 (variant 1))

Every Phase 1 envelope SHOULD emit a structured log line at the
host's audit transport: timestamp per RFC 3339, host identifier,
tenant identifier, envelope class, envelope identifier, operation
outcome, and a W3C Trace Context `traceparent` propagated end-to-end
so a single operation can be reconstructed across hosts. Phase 2
surfaces this trace identifier as the `X-WIA-Trace-Id` response
header. Phase 3 protocol exchanges propagate the trace identifier
inside the exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (Splunk, Elastic, Sumo Logic,
Wazuh, Microsoft Sentinel) per OpenTelemetry semantic conventions,
with `wia.standard.slug` = `semiconductor-design` and `wia.standard.phase` =
`1` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 1 (variant 1))

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 Sec 5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations. The composition also lets the operator's SIEM
correlate per-tenant audit records across multiple standards
without per-standard schema-mapping work.

## Z.3 Capabilities discovery and SemVer (Phase 1 (variant 1))

Hosts SHOULD publish a capabilities document at
`/.well-known/wia-semiconductor-design-capabilities` enumerating per-endpoint
optionality. Clients MUST treat unsupported capabilities as absent
rather than as an error condition; a client that needs a capability
the host does not advertise MUST surface a clear configuration
error rather than silently degrade. Hosts moving from one minor
version to the next MUST publish the change in the host's release
notes with the per-capability migration window per IETF RFC 8594
(Sunset header) + RFC 9745 (Deprecation header) + RFC 9651
(Structured Field Values) so machine consumers can plan migration
without waiting for human-channel notification.

## Z.4 Privacy envelope per per-jurisdiction law (Phase 1 (variant 1))

Phase 1 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California
CPRA per Cal. Civ. Code Sec 1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Subject-rights endpoints (access, rectification,
erasure, portability, restriction, objection) compose with
WIA-OMNI-API Sec 5 subject-rights surface and need not be
re-implemented per-standard.

## Z.5 DR / continuity envelope per ISO 22301 (Phase 1 (variant 1))

Hosts running this Phase MUST publish a continuity-of-operations
envelope per ISO 22301:2019 + ISO/IEC 27031 + NIST SP 800-34 Rev 1
covering: per-host RTO (Recovery Time Objective) per the operator's
business-impact analysis; per-host RPO (Recovery Point Objective)
tied to the host's audit-stream replication policy; per-host
backup envelope (per-region cross-replicated immutable backup with
the per-tier retention envelope per the per-jurisdiction record-
retention policy); per-host failover-rehearsal envelope (typically
quarterly per the operator's BC/DR program); per-host vendor-exit
envelope so the operator can migrate the host to an alternate
implementation without losing audit-trail continuity.

## Z.6 Supply-chain envelope per SLSA (Phase 1 (variant 1))

Every host implementation MUST publish a software-bill-of-materials
(SBOM) per SPDX 2.3 / 3.0 (per ISO/IEC 5962 + Linux Foundation SPDX)
or CycloneDX 1.6 (per OWASP Foundation). The SBOM enumerates every
direct + transitive dependency with the per-component name +
version + licence + supplier + per-component hash + per-component
PURL (Package URL per package-url spec) + per-component CPE
(Common Platform Enumeration per NIST). Supply-chain attestation
follows in-toto per CNCF in-toto + SLSA (Supply-chain Levels for
Software Artifacts) per OpenSSF SLSA Framework — typically targeting
SLSA Level 3 for hosted production deployments.

弘益人間 — Benefit All Humanity.
