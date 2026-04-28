# WIA-COMP-007 — Phase 1: Data Format

> Virtualization canonical Phase 1: hypervisor + VM + disk-image + virtual-NIC + vCPU-topology + memory-management envelopes.

# WIA-COMP-007: Virtualization Specification v1.0

> **Standard ID:** WIA-COMP-007
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active

---


## 1. Introduction

This specification defines standards for virtualization technology including hypervisors, virtual machines, resource management, and VM lifecycle operations.

### 1.1 Philosophy

**弘익人間 (Benefit All Humanity)** - Virtualization democratizes computing resources, making powerful infrastructure accessible to all.

---



## 2. Hypervisor Architecture

### 2.1 Type-1 (Bare-Metal)

```
Hardware → Hypervisor → VM 1, VM 2, VM 3
```

**Characteristics:**
- Direct hardware access
- Better performance
- Production environments
- Examples: VMware ESXi, Xen, KVM, Hyper-V

### 2.2 Type-2 (Hosted)

```
Hardware → Host OS → Hypervisor → VM 1, VM 2
```

**Characteristics:**
- Runs on host OS
- Easier setup
- Development/testing
- Examples: VirtualBox, VMware Workstation

---



## 3. CPU Virtualization

### 3.1 Hardware-Assisted Virtualization

**Intel VT-x:**
- VMX (Virtual Machine Extensions)
- EPT (Extended Page Tables)
- VPID (Virtual Processor IDs)

**AMD-V:**
- SVM (Secure Virtual Machine)
- NPT/RVI (Nested Page Tables)
- ASID (Address Space IDs)

### 3.2 CPU Allocation

```yaml
vcpu_config:
  count: 4
  shares: 2048  # Relative weight
  reservation: 2000  # MHz guaranteed
  limit: 8000  # MHz maximum
  affinity: [0, 1, 2, 3]  # CPU pinning
```

---




---

## A.1 Hypervisor-record envelope

The Phase 1 envelope groups hypervisors by architectural class
(Type-1 bare-metal — KVM per Linux Kernel, VMware ESXi per VMware
vSphere doc, Microsoft Hyper-V per Microsoft Server doc, Xen per
Xen Project, IBM PowerVM per IBM Power Systems doc, Solaris LDoms
per Oracle Solaris doc; Type-2 hosted — VirtualBox per Oracle
VirtualBox doc, VMware Workstation per VMware doc, Parallels
Desktop per Parallels doc, QEMU user-mode per QEMU doc) with the
canonical fields: hypervisor identifier, version, host-CPU
extension envelope (Intel VT-x / VT-d / VT-c per Intel SDM Vol 3C;
AMD-V / AMD-Vi / SVM per AMD Manual Vol 2; ARMv8.1+ Virtualization
Host Extensions per ARM ARM DDI 0487; RISC-V Hypervisor Extension
per RISC-V Privileged Spec), peripheral-passthrough capability
envelope (PCIe ATS + PRI + PASID per PCIe 4.0 + 5.0; SR-IOV per
PCI-SIG SR-IOV spec), and the audit envelope tied to the
operating tenant.

## A.2 Virtual-machine record envelope

A virtual-machine record envelope MUST list: VM identifier (UUID v7
per RFC 9562), tenant + project envelope, vCPU descriptor (count
+ topology + pinning + reservation + limit + shares per the
operator's QoS policy), memory descriptor (size + reservation +
limit + shares + NUMA-node affinity per the operator's NUMA-policy
+ huge-pages envelope per Linux MM Documentation), virtual-disk
descriptor (per-disk format per §A.3 + size + IOPS-cap +
bandwidth-cap), virtual-NIC descriptor (per-NIC virtio / SR-IOV /
emulation envelope per §A.4 + bandwidth + IOPS shaping), guest-OS
descriptor (per-guest paravirtualisation envelope; balloon-driver
state; vTPM state per TCG TPM 2.0 Library Spec; secure-boot
envelope per UEFI 2.10), and the lifecycle-envelope (creation +
power-state + checkpoint + migration + retirement events).

## A.3 Disk-image-format catalogue

Disk-image formats span: raw — flat binary, fastest, no features;
QCOW2 v3 per QEMU spec — copy-on-write, snapshots, compression,
encryption per AES-XTS, lazy-allocation, internal/external
snapshots; VMDK per VMware spec — descriptor + extent layout,
streamOptimized for OVF, sparse / flat / preallocated variants;
VHD per Microsoft spec — fixed / dynamic / differencing variants,
512B sector size; VHDX per Microsoft spec — fixed / dynamic
variants, 4K sector size, log-based metadata journal, 64TB max
size; VDI per Oracle VirtualBox spec — fixed / dynamic / immutable;
HDD per Parallels spec; per-format conversion envelope per
qemu-img convert + per-format snapshot-chain envelope. The
operator's disk-image-format-of-record SHOULD prefer QCOW2 / VHDX
for production with compression + encryption + thin-provisioning;
the raw format is reserved for the ultra-low-latency case.

## A.4 Virtual-NIC envelope

Virtual-NIC envelopes catalogue: emulated devices (e1000e per
Intel 82574L; e1000 per Intel 82540EM; rtl8139 per Realtek;
pcnet32 per AMD Lance) — wide guest support, lower performance;
paravirtualised devices (virtio-net per OASIS Virtual I/O Device
1.2 + 1.3 specification — high performance, requires guest virtio
driver; vmxnet3 per VMware paravirt envelope; netvsc per Hyper-V
synthetic-NIC envelope) — best general-purpose performance;
direct-assignment devices (SR-IOV VF per PCI-SIG SR-IOV; PCIe
passthrough per Intel VT-d / AMD-Vi / ARM SMMU; macvtap per
Linux kernel macvtap envelope) — near-native performance, limited
sharing; smartNIC + DPU envelope per Open Compute Project DPU spec
+ NVIDIA BlueField + AMD Pensando + Intel IPU.

## A.5 vCPU-topology envelope

vCPU-topology envelopes carry: socket / core / thread layout per
the guest's expected ACPI MADT + SRAT; NUMA topology envelope
per the host NUMA layout with per-domain memory-affinity envelope;
CPU-feature envelope (model + family + stepping + per-feature flag
list per CPUID leaves) with the host-passthrough vs custom-model
vs minimum-baseline policy; turbo / power-state envelope per ACPI
P-states + C-states with the per-VM gating envelope; and the
real-time envelope (vCPU-pinning to dedicated host pCPU + isolated
host pCPU + IRQ-affinity envelope + RT-priority envelope) for
workloads that cannot tolerate per-vCPU jitter.

## A.6 Memory-management envelope

Memory-management envelopes carry: shadow-page-table envelope (pre-
hardware-assist; deprecated for production; preserved for older
processors); two-dimensional paging envelope (Intel EPT per VT-x;
AMD NPT per AMD-V; ARM Stage-2 per ARMv8 Virtualization; RISC-V
G-stage per H-extension); ballooning envelope per virtio-balloon
per OASIS spec — guest-cooperative reclaim; KSM (Kernel Same-page
Merging) envelope per Linux MM Documentation — host-cooperative
deduplication; transparent huge pages envelope per Linux MM doc;
hugetlbfs envelope; persistent-memory (PMEM) envelope per ACPI 6.2
NFIT + per the SNIA NVM Programming Model; CXL Type-3 memory
envelope per CXL 3.0 spec for memory-pooled deployments.

## A.7 Boot-and-firmware envelope

Boot-and-firmware envelopes carry: per-VM firmware-class envelope
(SeaBIOS for legacy-BIOS guests per QEMU SeaBIOS; OVMF / EDK2-based
UEFI per TianoCore EDK2 + UEFI 2.10; Hyper-V firmware per Microsoft
Hypervisor Firmware Spec); per-VM Secure-Boot envelope (per UEFI
2.10 §32 + the operator's Secure-Boot signing-key envelope); per-VM
TPM envelope per TCG TPM 2.0 Library Spec with the per-VM vTPM
state-database envelope (swtpm-backed + libtpms-backed reference
implementations + vendor-specific vTPM such as VMware vTPM + Hyper-V
vTPM); per-VM measured-boot envelope per TCG D-RTM with the per-VM
event-log retained inside the vTPM PCR-extension trail; per-VM
console envelope (VNC + SPICE + serial-console + virtio-console
per OASIS Virtual I/O Device spec). The boot-and-firmware envelope
ties into Phase 4 §A.3 attestation envelope so a verifier can match
the as-booted measurement chain against the per-tenant policy.

## A.8 Quota and tenant-isolation envelope

Quota and tenant-isolation envelopes carry: per-tenant compute
quota (vCPU-hour budget + accelerator-hour budget per the operator's
billing policy); per-tenant memory quota (peak-allocated memory +
peak-resident memory per the operator's overcommit policy); per-
tenant storage quota (capacity per tier + IOPS-rate cap + bandwidth
cap); per-tenant network quota (per-vNIC bandwidth cap + per-tenant
aggregate bandwidth cap + per-tenant connection-rate cap); per-
tenant API-rate quota; per-tenant security-isolation envelope (per-
tenant VLAN + per-tenant VXLAN + per-tenant virtual-router with
the operator's micro-segmentation policy); and the cross-tenant
audit-isolation envelope ensuring one tenant's audit trail cannot
be mined by another tenant absent operator authorisation.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/virtualization/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-virtualization-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/virtualization-host:1.0.0` ships every virtualization envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/virtualization.sh` ships sample envelope generators with no
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
ecosystem. Virtualization deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

## Z.6 Logging and observability hooks

Every Phase 1 envelope SHOULD emit a structured log line at the
host's audit transport: ISO 8601 UTC timestamp per RFC 3339, host
identifier, tenant identifier, envelope class, envelope identifier,
operation outcome, and an opaque trace identifier propagated end-
to-end per W3C Trace Context (`traceparent` header) so a single
operation can be reconstructed across hosts. Phase 2 surfaces this
trace identifier as the `X-WIA-Trace-Id` response header. Phase 3
protocol exchanges propagate the trace identifier inside the
exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (e.g., Splunk, Elastic, Sumo
Logic, Wazuh, Microsoft Sentinel) per OpenTelemetry semantic
conventions, with `wia.standard.slug` and `wia.standard.phase` as
required attributes.

## Z.7 Versioning, deprecation, and capability discovery

Within the 1.x line, hosts MAY publish a capabilities document at
`/.well-known/wia-virtualization-capabilities` that enumerates which
optional fields, optional endpoints, and optional protocol exchanges
the host implements. Clients MUST treat unsupported capabilities
as absent rather than as an error condition; a client that needs
a capability the host does not advertise MUST surface a clear
configuration error rather than silently degrade. Hosts moving
from one minor version to the next MUST publish the change in
the host's release notes with the per-capability migration window
per IETF RFC 8594 (Sunset header) + RFC 9745 (Deprecation header)
+ RFC 9651 (Structured Field Values) so machine consumers can
plan migration without waiting for human-channel notification.

## Z.8 Privacy and data-minimisation envelope

Phase 1 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California CPRA
per Cal. Civ. Code §1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Where the envelope ships across jurisdictional borders,
the operator's per-jurisdiction transfer envelope (SCC for EU, UK
IDTA, APEC CBPR, ASEAN MCC) MUST be referenced inside the audit
record. Subject-rights endpoints (access, rectification, erasure,
portability, restriction, objection) compose with WIA-OMNI-API per
its §5 subject-rights surface and need not be re-implemented
per-standard.

弘益人間 — Benefit All Humanity.
