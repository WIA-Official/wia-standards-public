# WIA-CODE-001 — Orbit Anchor 2D Code · Specification (draft)

Version 0.4 · reference implementation lives in `../reference/`. All constants below are the
values used by that code; the spec and code are kept in sync.

## 1. Coordinate system & grids

- Unit = **module**. A code is an N×N module grid. Standard grids: **S = 64, M = 96, L = 128**.
- Rendered pixel size = `(N + 2·quiet) · cellPx`, with a `quiet` zone (default 4 modules) of
  white around the code. `cellPx` = module size in pixels (renderer parameter; the decoder
  recovers it automatically).

## 2. Fixed structure (localization anchors)

All positions in modules; the code center is `c = N/2`.

### 2.1 Core (central bullseye) — at `(c, c)`
Concentric rings, inner→outer (ink = black):

| radius (modules) | fill |
|---|---|
| ≤ 1.5 | black disk (brand/identity slot) |
| 1.5 – 3.5 | white ring |
| 3.5 – 5.5 | black ring |
| 5.5 – 6.5 | white separator |

Reserved core radius = 6.5 modules.

### 2.2 Satellites (four corners) — inset `d = 4.5` modules from each corner
- Positions: TL `(d,d)`, TR `(N−d,d)`, BR `(N−d,N−d)`, BL `(d,N−d)`.
- Radius = 2.5 modules (diameter 5). Three are solid **black disks**; **TL is a donut**
  (white hole radius 1.0) — the **north star** that fixes rotation (breaks the 4-fold symmetry).

### 2.3 Format orbit
24 dots on a ring of radius 8.5 modules about the core (dot radius 0.55). Reserved for format
information (e.g. grid, level count) in future versions; not required by the current decoder,
which discovers grid and level count by trial decode.

### 2.4 Reserved zones
Core (≤ 7.1), the orbit ring band, and each satellite (≤ 3.1) are reserved — no data cells there.

### 2.5 Silhouettes — anchor placement outside the square case
The code boundary is a **data mask**, not a localization feature. A renderer MAY clip the data
field to any silhouette; `insideShape(...)` in `geometry.js` implements `square`, `round`,
`heart`, `clover`, `star`, `hex` and `boomerang`.

For every silhouette other than `square`, the four satellites move from the corner insets of
§2.2 onto a ring of radius `Rs = N/2 − satRimMargin` (default `satRimMargin = 3.5`) at
**12 / 3 / 6 / 9 o'clock**; TL (12 o'clock) remains the donut north star. This keeps all five
anchors inside every supported silhouette. The core (§2.1) and orbit (§2.3) are unchanged.

A decoder MUST NOT use the boundary to identify or orient a code. Narrower silhouettes simply
have fewer data cells (§3), so capacity falls with silhouette area.

### 2.6 Visual signature (normative identity)
A mark is a WIA Code **if and only if** it carries, at its center, the structure of §2.1–§2.3:
a concentric bullseye core, four satellite anchors of which exactly one is a donut, and the
24-dot format orbit. These three together are the **visual signature** of the format.

The outer silhouette (§2.5), the choice of `bitsPerCell`, the presence of a color layer (§3.2),
and any brand artwork placed in the core's central disk are all free and carry no identity.
Conformant documentation, tooling and user interfaces SHOULD identify a code by its visual
signature and MUST NOT name a code after its silhouette (e.g. "heart code").

A deployment MAY overlay additional marks on the data field — for example a small standard QR
code acting as an onboarding bridge for cameras that do not know this format yet. Such overlays
are deployment concerns outside this specification; they do not change the code's identity, and
an image is not a QR code merely because one is present.

## 3. Data cells

- **Data cells** = every non-reserved module, enumerated in raster order (row-major). This
  ordering is the encode/decode contract (`dataCells(layout)` in `geometry.js`).
- Each data cell carries a symbol of **`bitsPerCell` bits** (1, 2, or 3), rendered as a round
  dot whose **gray level** encodes the symbol. Recommended dot radius: **0.50 modules**
  (SHOULD; moved from 0.42 in v0.2). Rationale: raising fill ratio 55% → 79% pushed the
  low-resolution decode breakpoint from 1.50 to 1.25 px/module with no change in lock success
  (measured across grid L, 10 silhouettes × 3 payloads — see reference implementation history).
  This is a recommendation, not a decode-contract requirement: the decoder samples each cell by
  its **center** (±0.22 modules) and never measures dot radius, so it is radius-agnostic by
  construction. A code rendered at 0.42 (or any other radius in a similar range) remains fully
  conformant and decodes identically; encoders SHOULD use 0.50 for new output to get the
  low-resolution benefit, but existing 0.42 output and any renderer that has not yet updated
  are unaffected and require no re-issue.
- Cell counts (reference): S = 3728, M = 8848, L = 16016.

### 3.1 Grayscale levels & Gray coding
- `nlev = 2^bitsPerCell` evenly spaced luminance levels: level `rank` → gray
  `round(255·(1 − rank/(nlev−1)))` (rank 0 = white 255 … rank max = black 0).
- Symbol → luminance rank uses a **Gray code** (`rank = v ^ (v>>1)`), so a misread to an
  adjacent luminance level costs exactly one bit — recoverable by RS. 1-bit is the classic
  black/white case (symbol 1 = black dot).

### 3.2 Color (hue) layer — optional, additive
A second independent payload may ride in **chroma** (YCbCr, Rec.601). Each hue-carrying dot keeps
its luminance Y equal to its grayscale level and places its color on a fixed-radius ring in the
Cb–Cr plane. Because Rec.601 luma coefficients cancel the chroma contribution exactly, `toGray`
recovers the same luma symbol (±1 LSB) — the grayscale layer and localization are unchanged, and a
grayscale-only decode yields a complete payload (graceful degradation, colorblind-safe).
- **Default K=2** on the blue–yellow (Cb) axis: symbol 0 = +ρ (blue), 1 = −ρ (yellow), ρ≈40. K=4
  adds the red–green (Cr) axis for a 2nd bit (not colorblind-safe; opt-in).
- **Hue rides only mid-luma cells** (rank ∉ {0, nlev−1}); which cells are eligible is derived from
  the decoded luma (no side channel). Eligible fraction: 50% at 2-luma, 75% at 3-luma.
- **Separate RS plane**: the hue payload is its own `frame → RS(50%) → scramble → permutation`
  over the ordered eligible cells — so it never corrupts the luma payload.
- **Decode**: white/black anchors give a per-channel white-balance affine; a per-region neutral
  field is estimated from the (known-neutral) hue-ineligible cells; each eligible cell's corrected
  chroma is classified to the nearest ring point. Chroma collapsed below a threshold → erasure.

## 4. Data encoding pipeline

1. **Frame**: `[0x57 'W', 0x01 version, lenHi, lenLo]` + payload (UTF-8) + **CRC-16** (2 bytes).
2. **Capacity plan**: `rawBytes = floor(cells · bitsPerCell / 8)`; Reed–Solomon block plan over
   `rawBytes` at an ECC ratio of **25% (1-bit) / 35% (2-bit) / 50% (3-bit)** — more levels are
   noisier, so more parity. The 2-bit tier moved from 50%→35% after measurement showed the extra
   parity bought no real robustness (degradation failures come from contrast collapse, not from
   exhausting the error budget), for roughly +30% capacity at that tier. Decoders SHOULD try both
   35% and 50% for 2-bit content and keep whichever parse reports fewer corrected errors, to stay
   compatible with codes encoded before this change. Frame must fit the plan's data capacity.
3. **RS + interleave**: systematic Reed–Solomon (GF(256)) with block interleaving.
4. **Whitening**: XOR the codeword stream with a deterministic LCG mask (`scramble`). Required —
   without it, RS zero-padding leaves most cells at one level and breaks level calibration.
5. **Position permutation**: assign each codeword unit to a spatially scattered cell via a
   deterministic coprime-stride permutation. A byte is 4–8 adjacent cells; under perspective the
   compressed far side has clustered errors that would concentrate in those bytes. Scattering
   makes each byte's cells come from different regions, so byte-error rate ≈ cell-error rate.
6. **Map to cells**: pack the permuted, whitened codeword bits MSB-first, `bitsPerCell` bits per
   cell (in `dataCells` order after permutation); convert each symbol to its Gray-coded gray level.

Decoding reverses exactly: sample → classify to symbols → unpack bits → un-whiten → de-interleave
→ RS decode → CRC-check the frame → payload.

### 4.1 Frame version 2 — segment-mode payload (doc v0.4, 2026-09-05)

Version 1 frames carry raw UTF-8. Version 2 keeps the **same envelope, RS plan, whitening and
permutation** and only changes what sits between the header and the CRC:

```
[0x57 'W', 0x02, lenHi, lenLo] + segment bitstream (len bytes, MSB-first) + CRC-16
bitstream = flags(2b, reserved = 0)
          + { mode(3b) + count(12b) + body }*   ← one or more segments
          + END(mode 0) + zero padding to a byte boundary
```

| mode | name | body | count means |
|---|---|---|---|
| 0 | END | — | — |
| 1 | N numeric `0-9` | 10 bits per 3 digits; remainder 1 digit → 4 bits, 2 digits → 7 bits | digits |
| 2 | A url45 | 11 bits per 2 chars; odd tail → 6 bits. Table (45): `a-z 0-9 - . _ : / ? & =` and space | chars |
| 3 | H Hangul | alphabet = 11,172 syllables U+AC00–D7A3 followed by ASCII 0–127 (11,300 symbols); 27 bits per 2 symbols; odd tail → 14 bits | symbols |
| 4 | B bytes | 8 bits per byte (UTF-8) | **bytes** |
| 5 | C7 ASCII | 7 bits per char (0–127) | chars |
| 6 | S64 base64url | 6 bits per char, table `A-Z a-z 0-9 - _` | chars |
| 7 | reserved | — | decoders MUST reject the frame |

- `count` is 12 bits (max 4,095) for every mode; a longer run is split into consecutive segments.
- Encoders choose segmentation freely (the reference encoder uses a cost-based dynamic program
  and emits version 1 when that is not longer). **Encoders MAY still emit version 1.**
- Decoders MUST accept both versions. A version-1-only decoder sees `version ≠ 1` and fails
  cleanly (no payload is produced) — no misdecode is possible because the CRC still covers the
  segment bitstream.
- Capacity (grid L, square, booster on, 1,450 B plan): digits 1,450 → **3,474**
  (ISO/IEC 18004 v27-Q: 1,933 → 1.80×), url45 text → 2,106, Hangul syllables 483 → 857,
  arbitrary UTF-8 bytes unchanged.
- Test vectors: `conformance-vectors/clean_square_{num,alnum,hangul}.png` and their `blur_r2`
  variants are version-2 frames; `verify.js` must decode all of them.

## 5. Localization (decode front-end)

1. **Detect** the core and satellites with a Fast Radial Symmetry Transform (dark radial-symmetry
   voting; noise averages over the ring circumference). Auto-scale: the core's outer ring radius /
   5.5 gives `cellPx` with no prior knowledge of code size.
2. **Assign** core + 4 satellites by geometric consistency (the core sits at the satellites'
   centroid; angular spread must surround it). Refine centers by dark/bright centroid.
3. **North star**: the satellite with the brightest (white-holed) center is TL → fixes rotation.
4. **Homography** (module→image), least-squares over the 5 anchors, two-stage refined.
5. **Perspective recovery (steep angles)**: when satellites are lost, fit two concentric core
   ellipses and recover the vanishing line + true center in closed form from the conic pencil,
   rectify the image, re-detect, and compose the homography. Reprojection residual ≈ 0.03 modules.

## 6. Level classification (decode back-end)

- Sample each data cell's gray through the homography (small disk average).
- Establish the luminance range from the **data-cell distribution** (robust low/high percentiles),
  not fixed thresholds — small dots do not reach the anchors' extremes under blur, but blur is an
  affine mix with the white background, so even level spacing is preserved within the observed
  range. Known anchors (core/satellites = black, donut hole/quiet zone = white) provide a
  contrast/orientation check.
- Classify each cell to the nearest of `nlev` levels; invert the Gray code to symbols.
- **Mode discovery**: try `bitsPerCell ∈ {1,2,3}`; the CRC/RS success gates which is correct.

## 7. Conformance & versioning

- A conformant encoder MUST reproduce the frame format (§4.1), RS/whitening/mapping (§4.2–4.5),
  and the fixed structure (§2). A conformant decoder MUST localize per §5 and decode per §6.
- Format version byte = `0x01` (raw UTF-8) or `0x02` (segment modes, §4.1). Future versions may add the format orbit payload (§2.3), a color
  layer, and additional grids; decoders should trial-decode and reject on CRC failure.

## 8. Non-goals / honesty

- This draft carries no data-codec innovation of its own beyond layout; the RS/CRC math is the
  reused, proven `wiacode-v1-codec.js`. The novel contribution is §5 (orbit-anchor localization +
  conic perspective recovery) and §6 (anchor-calibrated multi-level classification).
- **Only 1-bit (`bpc:1`) is usable in practice; conforming encoders MUST emit `bpc:1`.**
  Multi-level (2–3 bit) is defined by this specification but is **not camera-decodable**: the
  level-classification error floor (~14.7%) exceeds the ECC budget (~6.7%), so such a code locks
  on cleanly and then never decodes. The multi-level clauses remain in this document to define
  the format, not to recommend it. The same applies to the hue layer, which presupposes `bpc≥2`.
