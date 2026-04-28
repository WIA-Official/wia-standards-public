# WIA-COMP-007 — Phase 2: API Interface

> Virtualization canonical Phase 2: API surface (hypervisors + VMs + lifecycle + migration + storage + networks + telemetry).

# WIA-COMP-007: Virtualization Specification v1.0

> **Standard ID:** WIA-COMP-007
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active

---


## 5. I/O Virtualization

### 5.1 Device Emulation

- Full hardware emulation
- Compatibility with any guest
- Performance overhead

### 5.2 Paravirtualization

- Guest-aware drivers (virtio)
- Better performance
- Requires guest OS support

### 5.3 Direct Device Assignment

- SR-IOV (Single Root I/O Virtualization)
- PCIe passthrough
- Near-native performance
- Limited sharing

---



## 6. Storage Virtualization

### 6.1 Disk Image Formats

| Format | Description | Features |
|--------|-------------|----------|
| RAW | Flat binary | Fast, no features |
| QCOW2 | QEMU Copy-On-Write | Snapshots, compression, encryption |
| VMDK | VMware | Wide compatibility |
| VHD/VHDX | Microsoft | Hyper-V format |

### 6.2 Storage Features

- **Thin Provisioning**: Allocate on demand
- **Snapshots**: Point-in-time state
- **Linked Clones**: Share base disk
- **Live Storage Migration**: Move disks while running

---



## 7. Network Virtualization

### 7.1 Virtual Networking

```
VM1    VM2    VM3
 |      |      |
 +------+------+
        |
   Virtual Switch
        |
   Physical NIC
```

### 7.2 Network Modes

- **Bridged**: VM on same network as host
- **NAT**: VM behind host NAT
- **Host-only**: VMs communicate with host only
- **Internal**: VMs communicate with each other

---




---

## A.1 Endpoint reference

```http
POST /virtualization/v1/hypervisors                # register hypervisor
GET  /virtualization/v1/hypervisors/{id}           # fetch hypervisor record
POST /virtualization/v1/vms                        # create VM
GET  /virtualization/v1/vms/{id}                   # fetch VM record
POST /virtualization/v1/vms/{id}/lifecycle         # power/reset/snapshot
POST /virtualization/v1/vms/{id}/migrate           # initiate live migration
GET  /virtualization/v1/disks/{id}                 # fetch virtual disk
POST /virtualization/v1/networks                   # virtual network ops
WS   /virtualization/v1/state/stream               # state stream
GET  /virtualization/v1/audit/{id}                 # audit trail
```

Every endpoint follows the discovery convention at
`/.well-known/wia-virtualization`. VM-creation and lifecycle
endpoints require the requester's tenant-administrator credential
plus the per-project resource-quota envelope; cross-tenant
operations require platform-operator authorisation per the
operator's RBAC policy.

## A.2 Hypervisor-registration API

`POST /hypervisors` accepts the Phase 1 §A.1 envelope. The endpoint
validates the host-CPU extension envelope against the host's CPUID
+ /proc/cpuinfo + per-platform attestation envelope, materialises
the per-host capability record into the operator's CMDB, registers
the hypervisor with the operator's compute-cluster pool, and emits
the per-host capability + capacity events. Hypervisor-state
transitions through `provisioning`, `available`, `maintenance`,
`evacuating`, `retired`; state transitions emit audit events.

## A.3 VM-lifecycle API

`POST /vms` accepts the Phase 1 §A.2 envelope. The endpoint
selects a target hypervisor per the operator's scheduler envelope
(Nova scheduler per OpenStack; vSphere DRS; Hyper-V cluster-shared-
volume placement; per-tenant affinity / anti-affinity rules),
allocates the requested resources against the project quota,
provisions the per-VM virtual disks per §A.3 envelope, attaches
the per-VM virtual networks per §A.4 envelope, and emits the
power-on event. Subsequent lifecycle operations (power-off, reset,
snapshot-create, snapshot-revert, snapshot-delete) emit per-event
audit records with the operator-credential trace.

## A.4 Live-migration API

`POST /vms/{id}/migrate` accepts a migration-request envelope:
target-hypervisor identifier, migration-mode envelope (pre-copy
default per Clark + Fraser + Hand + Hansen + Jul + Limpach + Pratt
+ Warfield 2005; post-copy per Hines + Gopalan 2009;
hybrid pre-copy + post-copy per the operator's policy), migration-
network envelope (dedicated migration vSwitch per the operator's
network-segregation policy; encrypted per AES-GCM-256 per FIPS
197 + RFC 5288), bandwidth-budget envelope, and the maximum-
downtime envelope per the operator's SLA (typical 50ms-1s
acceptable). The endpoint enforces compatibility (per-CPU
compatibility-flag set; storage-shared or storage-migrated per
the operator's policy; network-overlay continuity envelope) and
emits the per-stage migration events.

## A.5 Storage-management API

`GET /disks/{id}` returns the virtual-disk record per Phase 1 §A.3:
format, size, thin-provisioning state, snapshot-chain envelope,
encryption-state envelope per LUKS2 per dm-crypt or per QCOW2
internal AES-XTS, IOPS-cap + bandwidth-cap state per the operator's
storage-QoS, and the per-disk attachment envelope (which VM(s)
attach this disk in shared-disk configurations). Disk-conversion
operations (raw <-> QCOW2 <-> VMDK <-> VHDX) follow the per-format
conversion envelope; thin-clone operations follow the operator's
COW envelope.

## A.6 Network-management API

`POST /networks` accepts a virtual-network envelope: network-mode
(bridged / NAT / host-only / internal / overlay per VXLAN per RFC
7348 / Geneve per RFC 8926 / NVGRE per RFC 7637), IPAM envelope
(per-tenant IP pool + DHCP + DNS envelope per the operator's
network-policy), per-port security-group envelope (ingress/egress
ACL per the operator's micro-segmentation policy), and the
overlay-encapsulation envelope where applicable. The endpoint
materialises the per-network OpenFlow / OVN / NSX-T / per-vendor
overlay records and emits the per-network audit events.

## A.7 Telemetry WebSocket

The state-stream WebSocket multiplexes per-VM events: vCPU-state
+ vCPU-utilisation, memory-pressure + balloon-state + KSM state,
per-disk IOPS + latency, per-NIC bandwidth + packet-rate + drop
envelope, lifecycle events (boot / shutdown / pause / migrate /
crash / panic), guest-agent events (per-OS guest-tools /
qemu-guest-agent / VMware Tools / Hyper-V Integration Services
heartbeat), and the host-side health envelope. Subscribers can
filter by VM-id, tenant-id, host-id, and event-class. Rate limits:
2000 req/h authenticated; 20000 req/h trusted-partner. WebSocket
subscriptions are bounded at 200 simultaneous per credential.

## A.8 Audit endpoint envelope

`GET /audit/{id}` returns the immutable audit trail: hypervisor
registration, every VM-create and VM-delete event, every lifecycle-
operation event, every snapshot-create / snapshot-revert event,
every live-migration event with the source + target hypervisor
envelope, every disk-attach / disk-detach event, every network-
attach / network-detach event, every credential change, every
quota-budget breach event with the operator-acknowledgement
envelope, and every retirement event. The audit-trail integrity is
anchored into a Merkle tree per-tenant; the root is committed to
the operator's archival record per the tenant's retention envelope
(typical 7 years for regulated workloads per SOC 2 + ISO 27001 +
the operator's per-jurisdiction retention envelope).


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
