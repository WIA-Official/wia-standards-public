# WIA-military-communication PHASE 1 — Data Format Specification

**Standard:** WIA-military-communication
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for military communications:
addressing, message envelopes, classification handling, waveform metadata,
spectrum-allocation records, link-quality observations, and the cross-
references that bind these to the broader operational picture. The shape
interoperates with allied-nation military-communication standards so
that a multi-national network deployment does not require parallel data
models.

References (CITATION-POLICY ALLOW only):
- STANAG 4406 — Military Message Handling System (MMHS) message format
- STANAG 5066 — bandwidth-constrained tactical data exchange (HF radio sub-network access)
- STANAG 4677 — Dismounted Soldier Reference Architecture
- MIL-STD-188-110D — interoperability and performance standards for HF radio modems
- MIL-STD-188-141C — interoperability and performance standards for ALE / HF radios
- Link 16 / MIDS / J-series messages (STANAG 5516, MIL-STD-6016)
- Link 22 (STANAG 5522) — Joint Range Extension over BLOS
- VMF (Variable Message Format, MIL-STD-2045-47001D)
- ITU-R Recommendations (frequency allocation, channel plans)
- IETF RFC 8259 (JSON), RFC 7515 (JWS), RFC 9162 (Certificate Transparency 2.0 pattern)

---

## §1 Scope

This PHASE applies to systems that originate, relay, or consume
military communication payloads on tactical and strategic networks:
single-channel ground/airborne radios (SINCGARS), HF Automatic Link
Establishment (ALE) networks, satellite TT&C and beyond-line-of-sight
(BLOS) links, tactical data links (Link 16, Link 22), and MMHS
message exchange.

The standard is allied-doctrine-aware: an implementation MUST declare
which national or coalition doctrine its records originate under so
that downstream consumers can apply the correct interpretation of
classification, release, and handling caveats. In scope: addressing
(URN-form), envelope shape, classification metadata, waveform
metadata, link-quality observations, and spectrum allocation. Out
of scope: encryption-key generation (handled by national key
distribution centres) and physical-layer modulation (governed by
the cited mil-std documents).

## §2 Addressing

Endpoints are identified as URNs of form
`urn:wia:milcomms:ep:<scope>:<authority>:<id>` where:

- `scope` is one of `unit`, `platform`, `terminal`, `app`
- `authority` is the issuing national or coalition catalogue
- `id` is the catalogue's identifier for the endpoint

Examples:

- `urn:wia:milcomms:ep:platform:nato-ramcis:f-15k-tail-37`
- `urn:wia:milcomms:ep:unit:rok-army:roa-2-bde-3-bn`
- `urn:wia:milcomms:ep:terminal:us-spawar:an-prc-117g-sn-91a7`

The catalogue is maintained by the issuing authority and is itself
auditable; an endpoint not in a recognised catalogue is rejected
at addressing-time.

## §3 Message envelope (MMHS-aligned)

Each message carries an envelope with these mandatory fields:

| Field             | Source / Binding                                         |
|-------------------|----------------------------------------------------------|
| `msgId`           | URN of form `urn:wia:milcomms:msg:<originator>:<seq>`    |
| `originator`      | endpoint URN of the sending principal                    |
| `addressees[]`    | each entry: endpoint URN, role (`To`, `Cc`, `Info`)      |
| `precedence`      | one of `flash`, `immediate`, `priority`, `routine`,      |
|                   | `deferred` per STANAG 4406                               |
| `dateTime`        | RFC 3339 with offset; UTC SHOULD be used on transit      |
| `classification`  | structured marking per §4 below                          |
| `releaseAuthority`| URN of the release authority that approved transmission  |
| `subject`         | free-text, with classification rolled into the marking   |
| `body`            | encoded payload (text, binary blob, MMHS body parts)     |
| `references[]`    | other `msgId` values this message refers to              |

The envelope is signed using JWS (RFC 7515) by the originating
principal. The detached signature is held alongside the payload so
that the payload is canonical for downstream processing.

## §4 Classification marking

Classification follows a structured marking compatible with NATO
Common Format (STANAG 4774 / 4778) and US Information Security
Marking Standard:

- `level` — one of `unclassified`, `restricted`, `confidential`,
  `secret`, `top-secret` (NATO basic levels) plus equivalents for
  KR (대외비, 비밀-Ⅲ, 비밀-Ⅱ, 비밀-Ⅰ, Ⅰ급비밀)
- `releasability[]` — coalition releasability tags (e.g., NATO,
  KR-US, FVEY, NOFORN)
- `caveats[]` — handling caveats (e.g., COMINT, ORCON, IMCON,
  NOCONTRACT)
- `formerlyRestrictedData` — boolean
- `declassifyOn` — date or event reference (e.g., source-derived
  declassification rule)

A marking without `level` and at least one releasability tag is
rejected by the boundary; markings inconsistent with the source
catalogue's authority are rejected at canonicalisation.

## §5 Waveform metadata

Each transmission carries metadata describing the radio waveform
used:

- `waveformId` — URN naming the waveform (e.g.,
  `urn:wia:milcomms:wf:link-16-ja`,
  `urn:wia:milcomms:wf:milstd-188-110d-9600bps`)
- `band` — frequency band (HF, VHF, UHF, L, S, X, Ku, Ka, Q/V)
- `centerFrequency` — Hz, with ITU channel reference where
  applicable
- `bandwidth` — Hz
- `modulation` — coded modulation per the waveform spec
- `forwardErrorCorrection` — coded FEC scheme
- `cryptoSuite` — cryptographic suite identifier (no key material
  in this field)
- `interleaving` — interleaver depth and type, where applicable

Waveform metadata is read-only post-transmission; corrections issue
a new record with a `corrigenda` reference to the prior. The metadata
is the basis for compatibility checks at receiving terminals.

## §6 Spectrum allocation record

Spectrum use is recorded with:

- `allocationId` — URN
- `frequencyRange` — start and end in Hz
- `geoArea` — polygon of WGS-84 coordinates within which the
  allocation is valid
- `period` — start and end timestamps (RFC 3339 with offset)
- `users[]` — endpoint URNs authorised under this allocation
- `coordinationAuthority` — URN of the spectrum-management body
  that issued the allocation
- `priority` — allocation priority class (mission-critical,
  operational, training, exercise)

Allocations follow ITU-R Region rules for the originating country
plus any host-nation coordination (e.g., a forward-deployed unit
in a partner nation's spectrum). Conflicts (overlapping allocations
in the same geo area and period) are detected by the boundary at
allocation-creation time and surfaced for resolution.

## §7 Link-quality observation

Operational radio terminals emit link-quality observations:

- `observationId` — URN
- `terminalRef` — endpoint URN
- `peerRef` — endpoint URN of the other end of the link (or
  `broadcast` for one-to-many)
- `timestamp` — RFC 3339 with offset
- `snr` — signal-to-noise ratio (dB) with measurement uncertainty
- `ber` — bit-error rate (dimensionless)
- `frameErrorRate` — link-layer frame error rate
- `connectionState` — one of `acquiring`, `linked`, `degraded`,
  `lost`
- `routePath[]` — relay nodes traversed (for store-and-forward
  networks)

Observations feed routing decisions and operator displays.
Aggregations of observations form the link's health record;
extended degradation triggers an alert per the deployment's
network-management policy.

## §8 Cross-link interoperability records

For cross-link operations (a Link 16 message handed off to a
SATCOM-relayed Link 22 voice channel for example), the boundary
records the bridge event:

- The originating waveform and message ID
- The bridging gateway URN
- The destination waveform and resulting message ID
- The translation policy used (lossless mapping, lossy mapping,
  format-converted summary)
- The classification check applied at the bridge

The bridge event ties the two messages into a single audit chain
so that auditors can verify that a coalition partner saw the
intended classified content at the intended classification level.

## §9 Allied report shapes (informative)

This PHASE preserves report shapes used in allied operations:

| Shape                | Trigger                                                     |
|----------------------|-------------------------------------------------------------|
| Communications status | per-link health summary at intervals                         |
| Spectrum-coordination request | new allocation request to a coordination authority    |
| EMCON status          | radio-silence posture for an operating area                  |
| Outage report         | unplanned link or terminal outage                            |
| Restoration report    | resumption of service after an outage                        |
| HF ALE channel scan   | reachable peer set from an ALE scan                          |

A deployment that needs to emit allied-format reports renders them
on demand from the underlying records; the canonical record remains
in the WIA shape, which is more general and lossless.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Worked example: MMHS message envelope (informative)

A fully populated message envelope ready for STANAG 4406 derivation:

```json
{
  "msgId": "urn:wia:milcomms:msg:awacs-tail-12:20260427-091500-0001",
  "originator": "urn:wia:milcomms:ep:platform:nato-ramcis:awacs-tail-12",
  "addressees": [
    {"to": "urn:wia:milcomms:ep:unit:rok-army:roa-2-bde-3-bn", "role": "To"},
    {"to": "urn:wia:milcomms:ep:unit:us-army:1id", "role": "Cc"}
  ],
  "precedence": "immediate",
  "dateTime": "2026-04-27T09:15:00+09:00",
  "classification": {
    "level": "secret",
    "releasability": ["NATO", "ROK"],
    "caveats": ["NOFORN"],
    "declassifyOn": "+25Y"
  },
  "releaseAuthority": "urn:wia:org:nato.cao.j2",
  "subject": "FRAGO 27-04 — see attached",
  "body": {"contentType": "text/plain; charset=utf-8", "value": "..."},
  "references": []
}
```

The envelope is signed by the originator's signing key, detached
JWS held alongside. The receiving boundary verifies the signature
against the originator's KA JWKS before delivering to the addressees.

## Annex B — Conformance disclosure

Implementations declare per-section conformance in their published
capability document. Sections marked `partial` reference the
deployment policy explaining the gap; sections marked `excluded`
carry a justification citing the controlling doctrine's allowance.
A deployment that is `partial` or `excluded` on §3 (Message envelope),
§4 (Classification), §6 (Spectrum allocation), or §7 (Link-quality)
is non-conformant overall.

## Annex C — Cross-domain references (informative)

The following table summarises how military-communication records
reference content from other domains and which gates apply on each
side of the boundary:

| Source domain                  | Use site in milcomms record       | Gate applied                                          |
|--------------------------------|-----------------------------------|-------------------------------------------------------|
| WIA-medical-data-privacy       | message body referencing patient  | medical-side consent + milcomms-side releasability    |
| WIA-medical-imaging            | reference to imaging study UID    | medical-side consent + milcomms-side releasability    |
| WIA-nbc-defense                | reference to validated NBC event  | NBC-side release authority + milcomms-side releasability |
| WIA-public-safety              | reference to civil-emergency event| civil-emergency policy + milcomms-side releasability  |

The boundary verifies the cross-domain reference exists at the
referenced standard's boundary before delivering the milcomms
message, so that downstream readers do not see dangling references.

## Annex D — Cryptographic suite identifiers (informative)

The `cryptoSuite` field in the waveform metadata (PHASE 1 §5) is
a URN. Suite identifiers are issued by the deployment's national
key authority and are documented in a separate cryptographic
registry. Examples (illustrative only):

- `urn:wia:milcomms:crypto:nato-ssia-aes256-2024`
- `urn:wia:milcomms:crypto:rok-mil-aes256-kcmvp-2025`
- `urn:wia:milcomms:crypto:us-suiteB-cnsa2`

The registry is itself classified at a higher handling caveat than
operational traffic; this PHASE only references registry IDs and
never embeds suite-internal parameters into the canonical record.

## Annex E — Versioning and deprecation (informative)

Versioning follows Semantic Versioning 2.0.0. Major bumps require
≥ 90 days overlap with the prior major version on every operationally-
fielded reference implementation; deprecated versions enter a 12-
month sunset window with migration notes recorded in the audit chain
so coalition partners know which version their counterpart honours.

## Annex F — Conformance levels (informative)

| Level     | Scope                                                      |
|-----------|------------------------------------------------------------|
| Surface   | data formats accepted; self-attested                        |
| Verified  | annual third-party audit                                    |
| Anchored  | continuous evidence package per audit chain transparency    |

Implementations declare their level in the capability advertisement.
Coalition operations typically require Verified or Anchored from
all parties.
