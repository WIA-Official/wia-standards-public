# WIA-SEMI-DESIGN-001 — Phase 3: PROTOCOL

> Semiconductor Design canonical Phase 3 specification per the WIA Standards four-Phase architecture.

> Domain: 반도체 설계 — 반도체 회로 설계 · IP 코어 · DRC/LVS · 검증 · 테이프아웃.

## A.1 Scope

This Phase covers the canonical protocol layer of the WIA-SEMI-DESIGN-001 standard. It composes with the Phase 4 document for cross-Phase integration and inherits cross-standard behaviour from the wider WIA Standards family per the README.md scope statement.

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

## Abstract

This document specifies the WIA-SEMI-001 standard for semiconductor design, providing a comprehensive framework for RTL design, verification, synthesis, and physical implementation. The standard aims to enable tool interoperability, design reuse, and collaboration across the global semiconductor ecosystem.

---


## 3. Normative References

The following standards are referenced in this specification:

- **IEEE 1364-2005**: Verilog Hardware Description Language
- **IEEE 1076-2019**: VHDL Language Reference Manual
- **IEEE 1800-2017**: SystemVerilog Unified Hardware Design, Specification, and Verification Language
- **IEEE 1801-2018**: UPF (Unified Power Format)
- **Accellera UVM 1.2**: Universal Verification Methodology
- **Liberty NCX**: Liberty Timing Model Format
- **LEF/DEF**: Library Exchange Format / Design Exchange Format
- **GDSII**: Graphic Database System II
- **OASIS**: Open Artwork System Interchange Standard

---


## 7. Synthesis Standards

### 7.1 Constraint Format

Timing constraints SHALL use SDC format:

```tcl
# Clock definition
create_clock -name clk -period 10.0 [get_ports clk]

# Input/Output delays
set_input_delay -clock clk -max 3.0 [all_inputs]
set_output_delay -clock clk -max 2.0 [all_outputs]

# False paths
set_false_path -from [get_ports rst_n]

# Multi-cycle paths
set_multicycle_path 2 -setup -from [get_pins reg_a/Q] -to [get_pins reg_b/D]
```

### 7.2 Power Intent

Power domains SHALL be specified using UPF:

```tcl
# Create power domains
create_power_domain PD_TOP -include_scope
create_power_domain PD_CORE -elements {u_core}

# Define supply nets
create_supply_net VDD -domain PD_TOP
create_supply_net VSS -domain PD_TOP

# Connect supplies
connect_supply_net VDD -ports VDD
connect_supply_net VSS -ports VSS

# Power states
add_power_state PD_CORE \
  -state ON {-supply_expr {VDD == 1.0}} \
  -state OFF {-supply_expr {VDD == 0.0}}
```

### 7.3 Optimization Goals

Synthesis tools SHALL support optimization for:

- **Area**: Minimize gate count
- **Power**: Minimize dynamic + static power
- **Performance**: Maximize frequency
- **Balanced**: Trade-off between all three

---


## Appendix A: Example Designs

### A.1 Simple Counter

```verilog
module counter #(
  parameter WIDTH = 8
)(
  input  wire             clk,
  input  wire             rst_n,
  input  wire             enable,
  output reg [WIDTH-1:0]  count
);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      count <= {WIDTH{1'b0}};
    else if (enable)
      count <= count + 1'b1;
  end

endmodule
```

### A.2 FIFO

See full example in `examples/fifo_sync.sv`

---

---

## Z.1 Audit transport and observability hooks (Phase 3)

Every Phase 3 envelope SHOULD emit a structured log line at the
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
`3` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 3)

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

## Z.3 Capabilities discovery and SemVer (Phase 3)

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

## Z.4 Privacy envelope per per-jurisdiction law (Phase 3)

Phase 3 envelopes that carry personal data MUST honour the
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

## Z.5 DR / continuity envelope per ISO 22301 (Phase 3)

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

## Z.6 Supply-chain envelope per SLSA (Phase 3)

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

## Z.1 Audit transport and observability hooks (Phase 3 (variant 1))

Every Phase 3 envelope SHOULD emit a structured log line at the
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
`3` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 3 (variant 1))

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

## Z.3 Capabilities discovery and SemVer (Phase 3 (variant 1))

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

## Z.4 Privacy envelope per per-jurisdiction law (Phase 3 (variant 1))

Phase 3 envelopes that carry personal data MUST honour the
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

## Z.5 DR / continuity envelope per ISO 22301 (Phase 3 (variant 1))

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

## Z.6 Supply-chain envelope per SLSA (Phase 3 (variant 1))

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
