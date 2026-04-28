# WIA-SEMI-DESIGN-001 — Phase 2: API

> Semiconductor Design canonical Phase 2 specification per the WIA Standards four-Phase architecture.

> Domain: 반도체 설계 — 반도체 회로 설계 · IP 코어 · DRC/LVS · 검증 · 테이프아웃.

## A.1 Scope

This Phase covers the canonical api layer of the WIA-SEMI-DESIGN-001 standard. It composes with the Phase 3 document for cross-Phase integration and inherits cross-standard behaviour from the wider WIA Standards family per the README.md scope statement.

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

## 弘益人間 (홍익인간) · Benefit All Humanity

---


## 2. Scope

This standard covers:

### 2.1 In Scope

- RTL coding guidelines (Verilog, VHDL, SystemVerilog)
- Verification methodologies (directed, random, UVM, formal)
- Synthesis constraints and optimization
- Physical design data formats
- API specifications for tool integration
- Design data exchange formats

### 2.2 Out of Scope

- Specific tool implementations
- Process technology details (foundry-specific)
- Analog/mixed-signal design (covered by WIA-AMS-001)
- Software development for embedded processors

---


## 6. Verification Standards

### 6.1 Verification Methodology

Designs SHALL be verified using at least one of:

1. **Directed Testing**: Manual test vectors
2. **Constrained-Random**: UVM/SystemVerilog
3. **Formal Verification**: Property checking
4. **Emulation**: FPGA-based verification

### 6.2 UVM Compliance

UVM testbenches SHALL follow UVM 1.2 standard:

```systemverilog
class my_test extends uvm_test;
  `uvm_component_utils(my_test)

  my_env env;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    env = my_env::type_id::create("env", this);
  endfunction

  virtual task run_phase(uvm_phase phase);
    phase.raise_objection(this);
    // Test execution
    phase.drop_objection(this);
  endtask
endclass
```

### 6.3 Coverage Requirements

Minimum coverage targets:

| Coverage Type | Minimum Target |
|---------------|----------------|
| Code Coverage | 95% |
| Functional Coverage | 90% |
| Toggle Coverage | 85% |
| FSM Coverage | 100% |
| Assertion Coverage | 100% |

### 6.4 Assertion-Based Verification

Critical properties SHALL be specified using SVA:

```systemverilog
// Property: Valid must remain stable when ready is low
property valid_stable;
  @(posedge clk) disable iff (!rst_n)
  (!ready && valid) |=> ($stable(valid) && $stable(data));
endproperty

assert_valid_stable: assert property (valid_stable)
  else $error("Valid signal not stable!");
```

---


## 10. Conformance

### 10.1 Conformance Levels

**Level 1 (Basic)**:
- RTL syntax checking
- Basic simulation
- Single-module designs

**Level 2 (Standard)**:
- Multi-module SoC
- UVM verification
- Synthesis to gates

**Level 3 (Advanced)**:
- Complete tape-out flow
- Multi-voltage design
- Advanced DFT

### 10.2 Certification Process

1. Submit design for review
2. Run conformance test suite
3. Review results with WIA committee
4. Receive certification badge

### 10.3 Test Suite

Reference implementations and test cases available at:
```
https://github.com/WIA-Official/wia-semi-001-tests
```

---

---

## Z.1 Audit transport and observability hooks (Phase 2)

Every Phase 2 envelope SHOULD emit a structured log line at the
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
`2` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 2)

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

## Z.3 Capabilities discovery and SemVer (Phase 2)

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

## Z.4 Privacy envelope per per-jurisdiction law (Phase 2)

Phase 2 envelopes that carry personal data MUST honour the
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

## Z.5 DR / continuity envelope per ISO 22301 (Phase 2)

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

## Z.6 Supply-chain envelope per SLSA (Phase 2)

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

## Z.1 Audit transport and observability hooks (Phase 2 (variant 1))

Every Phase 2 envelope SHOULD emit a structured log line at the
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
`2` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 2 (variant 1))

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

## Z.3 Capabilities discovery and SemVer (Phase 2 (variant 1))

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

## Z.4 Privacy envelope per per-jurisdiction law (Phase 2 (variant 1))

Phase 2 envelopes that carry personal data MUST honour the
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

## Z.5 DR / continuity envelope per ISO 22301 (Phase 2 (variant 1))

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

## Z.6 Supply-chain envelope per SLSA (Phase 2 (variant 1))

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
