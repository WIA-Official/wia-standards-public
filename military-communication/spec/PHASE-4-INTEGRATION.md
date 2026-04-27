# WIA-military-communication PHASE 4 — Integration Specification

**Standard:** WIA-military-communication
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how a deployment integrates the data, APIs,
and protocols from PHASEs 1–3 with the operational picture: terminal
fleet management, command-and-control consoles, gateway / bridging
infrastructure, spectrum-management offices, EMCON enforcement,
ALE coordination, and coalition exchange. It is non-prescriptive
about specific vendor products; it specifies the integration
*contracts* a deployment must satisfy.

References (CITATION-POLICY ALLOW only):
- STANAG 4406 — Military Message Handling System
- STANAG 5066 — bandwidth-constrained tactical data exchange
- STANAG 4774 / 4778 — Confidentiality and Information labelling
- MIL-STD-2045-47001D — Variable Message Format
- ITU-R Recommendations for spectrum coordination
- WIA-medical-data-privacy / WIA-nbc-defense — for cross-domain
  integration when communications carry medical or NBC content

---

## §1 Terminal fleet registry

The deployment maintains a registry of every fielded terminal:

- `terminalRef` — endpoint URN
- vendor, model, serial number
- supported waveforms and bands
- firmware version, last firmware update
- crypto-suite identifier(s) loaded
- TLS client certificate fingerprint and expiry
- operator (the unit owning the terminal)
- operating-area policy (geographic and EMCON scope)

A terminal not in the registry, or with expired certificate or
expired firmware revision, is refused at PHASE 2 §1 message
origination. Registry changes are themselves auditable.

## §2 C2 integration

The C2 system consumes the operational picture by subscribing to:

- New `precedence: flash`/`immediate` messages addressed to the
  command's monitoring endpoints
- Link-state changes (degraded, lost, restored)
- EMCON posture changes
- Spectrum-conflict alerts

The C2 system displays this on its operational picture; messages
can be acknowledged or referenced through the boundary's API. The
C2 system does not write back over the boundary; new messages flow
through PHASE 2 §1 from the originating principal, not from the
C2 console's edit path. This preserves a single canonical record
per message.

## §3 Gateway / bridging infrastructure

Gateways translate between waveforms (Link 16 ↔ Link 22, MMHS ↔
SMS over satellite, etc.). The integration contract:

- Each gateway is itself an addressable endpoint with a
  `wia_role: gateway` token
- Bridge events (PHASE 1 §8) are emitted at the gateway and
  recorded in the audit chain
- Lossy bridges require a recorded `translationPolicy:
  "lossy-acknowledged"` signed by a release authority before the
  bridge event commits
- Cross-coalition bridges are gated by the federation manifest;
  unfederated coalition bridging is refused

A gateway that fails to record its bridge events is treated as
a non-conformant gateway and its traffic is suspended pending
remediation.

## §4 Spectrum-management offices

Spectrum allocation requests (PHASE 2 §4) are coordinated with
spectrum-management offices in the operating area:

- Host-nation coordination — for forward-deployed units, the host
  nation's frequency-management body must approve operating bands
- ITU-R Region adherence — broadcast and shared-use bands obey the
  operating area's ITU-R Region rules
- Mission-priority routing — multiple competing requests in the
  same band/area are resolved by the priority class defined in
  PHASE 1 §6

The boundary surfaces upcoming allocation expirations to the
spectrum-management office at least 14 days in advance so renewals
proceed without operational impact.

## §5 EMCON enforcement

EMCON postures (PHASE 2 §7) propagate from the issuing authority
to all affected terminals in the operating area:

- The boundary blocks message origination for affected endpoints
  unless a flash-precedence override accompanied by release-authority
  signature is present
- Override events are themselves audited and reviewed within 24
  hours of the override
- Terminals confirm EMCON acknowledgement so the issuing authority
  knows posture is in effect

EMCON expiry is automatic at the recorded end timestamp; manual
extension issues a new posture record.

## §6 ALE coordination

For HF Automatic Link Establishment networks:

- Terminals report scan results (PHASE 2 §8) to the boundary
- The boundary aggregates scan results and surfaces a current
  reachable-peer view for each terminal
- The reachable-peer view drives automatic call-setup decisions
  in higher-layer applications
- Channel stress (overuse, jamming) surfaces from aggregated scans
  and is reported to the spectrum-management office for
  reassignment

ALE network health is itself a metric; degraded ALE health prompts
operational review of the deployment's HF posture.

## §7 Cross-domain integration

Communications often carry classified content from other domains.
The boundary integrates with adjacent WIA standards:

- **NBC events** — when a `validated` NBC event is to be reported
  to a coalition partner, the message body references the NBC
  event ID and the boundary applies the NBC-defence release-
  authority handshake in addition to the milcomms handshake
- **Medical evacuation coordination** — casualty handoffs use the
  pseudonymous subjectRef from WIA-medical-data-privacy; the
  message body references the medical record without exposing
  direct identifiers on the wire
- **Imaging hand-off** — pre-operative imaging shared with a
  forward surgical team flows via the milcomms system; the
  imaging metadata is referenced and the bulk transfer follows
  WIA-medical-imaging PHASE 2 §7 bulk export

Cross-domain references in messages are validated end-to-end:
the receiving boundary verifies that the referenced records exist,
are still active, and are accessible to the receiving organisation
under the relevant federation manifest.

## §8 Quarterly compliance report

The boundary emits a quarterly compliance report covering:

- Total messages originated by precedence and classification
- Releasability mismatches detected and refused
- EMCON postures issued, overrides invoked, override review backlog
- Spectrum allocations created, conflicts resolved, expirations
- Bridge events with their translation policies (lossless / lossy)
- Cross-coalition releases by federation peer
- Federation manifest health (active, expiring, expired)
- Terminal-fleet certificate health (current, expiring, expired)
- Audit-chain integrity check results

The report is signed and is itself in scope for the audit chain so
report tampering would surface in the chain.

## §9 Acceptance criteria

A deployment claims conformance when:

1. Every fielded terminal is in the registry with current
   certificate and current firmware
2. Every message in the past quarter has a matching audit chain
   entry with verifiable inclusion proof
3. Every cross-coalition release has both release-authority
   signatures on file
4. EMCON override review backlog is zero over the prior 30 days
5. Federation manifests for all listed peers are current
6. Spectrum allocations have no unresolved conflicts beyond the
   resolution window declared in the deployment policy
7. ALE health is within operational thresholds for the deployment
8. Quarterly compliance report has no integrity-check failures

A deployment failing any of these reports the gap in its compliance
package rather than concealing it.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Operational SLAs (informative)

| Concern                                          | Default SLA              |
|--------------------------------------------------|--------------------------|
| Message origination p95 added latency            | ≤ 100 ms                 |
| Tactical narrowband round-trip                   | ≤ 60 s                   |
| Spectrum-allocation conflict resolution          | ≤ 60 s                   |
| EMCON propagation to all affected terminals      | ≤ 5 minutes              |
| Federation manifest expiry alert lead time       | ≥ 30 days                |
| Catalogue refresh cycle                          | ≥ daily                  |
| Audit chain entry available after operation      | ≤ 10 s                   |

Tighter SLAs are negotiable per deployment; loosening them requires
operational-command sign-off.

## Annex B — Decommissioning (informative)

When a deployment is decommissioned:

1. Outstanding messages are either delivered, returned, or recorded
   as undeliverable with structured reasons
2. Active spectrum allocations are released back to the coordinating
   authorities
3. Outstanding EMCON postures expire or transfer to a successor
   deployment
4. Final daily root is sealed and the chain exported to the
   receiving custodian

The decommissioning manifest is itself an audit event in the final
chain root, signed by both outgoing and incoming custodians.

## Annex C — Acceptance checklist (informative)

A new deployment claims conformance after a checklist sign-off:

- [ ] Doctrine declared and matches the issuing authority on file
- [ ] KA published JWKS with current keys; cross-coalition manifests
      signed and current
- [ ] Terminal fleet registry populated with current certificates
      and current firmware
- [ ] Spectrum-management interface bound and tested with the
      coordinating authority
- [ ] EMCON propagation tested in dry-run
- [ ] Cross-domain references validated end-to-end with adjacent
      WIA standards' boundaries
- [ ] Audit chain initialised with a signed genesis root
- [ ] Disaster-recovery test completed with audit-chain
      reconstruction
- [ ] Quarterly compliance report scheduled and signed

A deployment failing any item logs the gap in its compliance
package and tracks remediation publicly.

## Annex D — Common pitfalls (informative)

Field experience surfaces a small set of recurring integration
pitfalls. They are not normative requirements, but deployments
SHOULD avoid them:

- Catalogue drift — issuing authorities change endpoint URNs
  without notice; the boundary's cache must refresh frequently
  enough to catch this
- Time skew on tactical terminals after long air-gap operation —
  GPS-disciplined PPS clocks recover quickly; software-only clocks
  may need manual resync
- Releasability flip after coalition reorganisation — federation
  manifests must be re-signed promptly when a coalition adds or
  removes a partner
- Bridge classification ratchet — repeated bridges of the same
  message can accumulate caveats; the boundary refuses bridges
  whose accumulated caveats exceed a per-policy threshold

## Annex E — Decommissioning (informative)

When a deployment is decommissioned, the final daily root is sealed,
outstanding messages are delivered or returned, active spectrum
allocations are released back to coordination authorities, and the
chain is exported to the receiving custodian. The decommissioning
manifest is itself an audit event in the final chain root, signed
by both outgoing and incoming custodians so coalition partners can
trace continuity.

## Annex F — Coalition exchange pitfall summary (informative)

Recurring lessons learned across coalition exercises:

- **Manifest renewal cadence** — federations renewed 30 days
  before expiry have the smoothest flow; renewals at ≤ 7 days
  have observable disruption to operational traffic
- **Releasability label drift** — when a coalition adds a new
  partner, every existing endpoint's releasability set may need
  review; deployments SHOULD schedule a quarterly releasability
  audit
- **EMCON propagation gap** — a posture issued at the C2 level
  has measurable propagation delay to fielded terminals; the
  deployment's network-management layer SHOULD verify terminal
  acknowledgement before the posture is reported as "in effect"
- **Bridge accumulation** — a message bridged across multiple
  waveforms may accumulate caveats; the boundary refuses bridges
  whose caveat count exceeds the per-policy threshold (typically
  3) without explicit release-authority sign-off
