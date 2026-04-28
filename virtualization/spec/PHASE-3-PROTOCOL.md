# WIA-COMP-007 — Phase 3: Protocol

> Virtualization canonical Phase 3: protocols (memory-virt + live-migration + HA + storage-virt + resource-management + snapshot).

# WIA-COMP-007: Virtualization Specification v1.0

> **Standard ID:** WIA-COMP-007
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active

---


## 4. Memory Virtualization

### 4.1 Memory Management Techniques

1. **Shadow Page Tables**: Pre-hardware assist
2. **Extended/Nested Page Tables**: Hardware-assisted
3. **Memory Ballooning**: Dynamic memory reclamation
4. **Page Sharing**: KSM (Kernel Same-page Merging)
5. **Memory Overcommit**: Allocate more than physical

### 4.2 Memory Configuration

```yaml
memory:
  size: 16Gi
  reservation: 8Gi  # Guaranteed
  limit: 32Gi  # Maximum (overcommit)
  shares: 163840  # Relative weight
  numa_node: 0  # NUMA awareness
```

---



## 8. Live Migration

### 8.1 Migration Process

1. **Pre-copy**: Iteratively copy memory pages
2. **Stop-and-copy**: Pause VM, copy remaining state
3. **Resume**: Start VM on target host

### 8.2 Requirements

- Shared storage or storage migration
- Compatible CPU features
- Network connectivity between hosts
- Sufficient resources on target

---



## 9. High Availability

### 9.1 HA Features

- **Automatic Failover**: Restart VMs on another host
- **VM Monitoring**: Heartbeat checks
- **Host Monitoring**: Cluster health
- **Admission Control**: Reserve resources for failover

---




---

## A.1 Memory-virtualization protocol

Memory-virtualization protocols cover: two-dimensional paging
exchange (guest-physical-to-host-physical translation per Intel
EPT, AMD NPT, ARM Stage-2, RISC-V G-stage with the per-walk TLB
envelope per the operator's CPU model); shadow-page-table fallback
envelope for unsupported processors (deprecated; preserved only
for legacy guests); ballooning protocol per virtio-balloon per
OASIS — guest-driver requests pages from guest-OS, hypervisor
reclaims; KSM protocol per Linux MM Documentation — periodic
host-side scan, per-page CRC + content compare + COW merge;
transparent huge pages protocol — host kernel khugepaged scans +
collapses; persistent-memory protocol per SNIA NVM Programming
Model — direct guest mapping bypassing page-cache for low-latency
PMEM access.

## A.2 Live-migration protocol

Live-migration protocols cover: pre-copy phase per Clark + Fraser
+ Hand + Hansen + Jul + Limpach + Pratt + Warfield 2005 — iterative
dirty-page tracking + transmission with per-
iteration convergence check; stop-and-copy phase — VM paused on
source, remaining dirty pages + CPU-state + device-state
transferred; resume phase — VM started on target, CPU + device
state restored, network-overlay continuity envelope ensures L2/L3
continuity per the operator's overlay-network-of-record; post-
copy fallback per Hines + Gopalan 2009 — VM resumed early, on-
demand page-fault transfer; hybrid pre-copy + post-copy envelope
per the operator's policy. The protocol covers the security
envelope (TLS 1.3 per RFC 8446 mandated for cross-host;
authentication per mTLS with per-host certificate rooted in the
operator's PKI).

## A.3 High-availability protocol

High-availability protocols cover: heartbeat envelope (per-cluster
heartbeat at typically 1s cadence with N-of-M voting per the
operator's quorum policy); host-failure detection envelope
(missed-heartbeat + IPMI-fence + STONITH envelope per the
operator's fencing policy per Pacemaker / Corosync / VMware HA /
Hyper-V Failover-Cluster); VM-restart envelope on host failure
(per-VM restart-priority + per-VM admission-control envelope so
the cluster reserves spare capacity); per-VM monitoring envelope
(VM heartbeat via guest-agent, application heartbeat via per-app
hook; restart-on-VM-fault envelope); and the split-brain
prevention envelope (quorum-disk + arbitration-witness + STONITH
envelope per the operator's policy).

## A.4 Storage-virtualization protocol

Storage-virtualization protocols cover: per-VM disk-attachment
protocol (virtio-blk / virtio-scsi per OASIS Virtual I/O Device
spec; emulated AHCI per Intel AHCI 1.3.1; emulated NVMe per NVM
Express Spec; SCSI per T10 SCSI-3); thin-provisioning protocol
(allocation-on-demand + UNMAP per T10 SBC-3 + DISCARD per Linux
block-layer); snapshot protocol (per-format COW chain per QCOW2
backing-file + VMDK redo-log + VHDX log; QCOW2 internal-snapshot
vs external-snapshot envelope per QEMU spec); live-storage-
migration protocol (per-block live-mirror per qemu-block-mirror
+ VMware Storage vMotion per VMware doc); and the data-integrity
envelope per T10-PI per SCSI + per-format checksum envelope.

## A.5 Resource-management protocol

Resource-management protocols cover: vCPU-scheduling envelope
(host-kernel CFS for KVM + ESXi proportional-share per VMware
ESXi scheduler doc + Hyper-V root-scheduler per Microsoft doc);
per-VM weight + reservation + limit envelope per the operator's
QoS policy with the per-tenant fairness envelope; memory-overcommit
envelope (active vs swapped vs ballooned vs deduplicated +
operator's overcommit-ratio envelope with per-tenant cap); I/O-QoS
envelope (per-disk IOPS / bandwidth shaping per blk-throttle per
Linux block-layer; per-VM IOPS-budget envelope; per-tenant
fair-queueing envelope per the operator's policy); network-QoS
envelope (per-vNIC bandwidth + DSCP per RFC 2474 + 802.1p per IEEE
802.1Q + flow-rate-limiting per the operator's policy).

## A.6 Snapshot-and-checkpoint protocol

Snapshot-and-checkpoint protocols cover: per-VM live-snapshot
protocol (memory + disk consistent snapshot per QEMU savevm + per
VMware live-snapshot envelope; quiesced via guest-agent for
crash-consistent vs application-consistent envelope); per-VM
external-snapshot protocol (block-device-level point-in-time
snapshot per LVM thin-pool / Ceph RBD snapshot / ZFS snapshot per
each storage-stack envelope); checkpoint-restart envelope per the
operator's per-VM checkpoint policy; per-VM clone envelope (full-
clone + linked-clone per the operator's COW envelope); and the
backup-integration envelope per the per-vendor changed-block-
tracking API (CBT per VMware + RCT per Hyper-V + virDomainBlock
per libvirt + Ceph RBD diff per Ceph doc).

## A.7 Container-coexistence protocol

Container-coexistence protocols cover the operator's choice of
container runtime alongside or inside virtualisation: KubeVirt
per CNCF KubeVirt project — containers + VMs co-resident on the
same Kubernetes cluster with a shared scheduler; Kata Containers
per Kata project — per-pod lightweight VM (Cloud Hypervisor or
Firecracker) wrapping the container runtime for VM-grade isolation;
Firecracker per AWS Firecracker project — micro-VM optimised for
serverless functions with sub-second boot envelope; gVisor per
Google open-source — user-space kernel-emulation alternative to
hardware-virtualisation; Nabla containers per IBM research — Linux
unikernel envelope. The protocol envelope ensures that the operator's
multi-tenant isolation budget is met by the per-runtime defence-
in-depth envelope (hypervisor isolation + namespace + cgroup +
seccomp-bpf + AppArmor / SELinux per the operator's policy).


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
