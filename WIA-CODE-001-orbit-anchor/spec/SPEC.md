# WIA-CODE-001 — Orbit Anchor 2D Code · Specification (draft)

Version 0.2 · reference implementation lives in `../reference/`. All constants below are the
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

## 3. Data cells

- **Data cells** = every non-reserved module, enumerated in raster order (row-major). This
  ordering is the encode/decode contract (`dataCells(layout)` in `geometry.js`).
- Each data cell carries a symbol of **`bitsPerCell` bits** (1, 2, or 3), rendered as a round
  dot (radius 0.42 modules) whose **gray level** encodes the symbol.
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
   `rawBytes` at an ECC ratio of **25% (1-bit) / 50% (2-bit, robust) / 50% (3-bit)** — more levels
   are noisier, so more parity. A 35% 2-bit tier trades robustness for ×1.7 capacity. Frame must
   fit the plan's data capacity.
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
- Format version byte = `0x01`. Future versions may add the format orbit payload (§2.3), a color
  layer, and additional grids; decoders should trial-decode and reject on CRC failure.

## 8. Non-goals / honesty

- This draft carries no data-codec innovation of its own beyond layout; the RS/CRC math is the
  reused, proven `wiacode-v1-codec.js`. The novel contribution is §5 (orbit-anchor localization +
  conic perspective recovery) and §6 (anchor-calibrated multi-level classification).
- Multi-level capacity is conditional on scan quality (pixels-per-module); use 1-bit high-ECC for
  harsh/disaster conditions and 2–3 bit for close, high-contrast scans (books, screens).
