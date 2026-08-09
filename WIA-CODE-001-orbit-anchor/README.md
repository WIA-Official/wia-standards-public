# WIA-CODE-001 — Orbit Anchor 2D Code

**[English](README.md) · [日本語](README.ja.md) · [简体中文](README.zh-CN.md) · [繁體中文](README.zh-TW.md)**

> An open, perspective-robust, high-capacity 2D machine-readable code — a QR alternative
> designed for the AI-camera / AR-glasses era and for **offline** use where it matters most:
> books for children without internet, and disaster-relief information readable in bad conditions.
>
> Status: **working reference prototype** (real-phone localization confirmed). This is a
> living specification + reference implementation, not yet a frozen standard.

弘益人間 — *benefit all humankind.* A code that anyone can read, with no app install and no
network, is a choice for everyone.

---

## Why another 2D code?

QR is the incumbent by ubiquity, not by performance. It is standardized as black/white only
(ISO/IEC 18004) and its square finder patterns are read by straight-line scanning, which
degrades under perspective tilt, motion, and low light — exactly the conditions of glasses,
moving cameras, and field/disaster use. WIA-CODE reads by **circles and centers** instead of
straight lines, which buys three structural advantages: rotation invariance, closed-form
perspective recovery from a single bullseye, and integral (noise-averaging) detection.

## What it is

- **Orbit anchor localization** — a central concentric-circle *bullseye* (core) + four corner
  *satellites* (one donut "north star" for rotation) + a format orbit ring. Detected by a Fast
  Radial Symmetry Transform, refined to a sub-pixel homography, with **conic-pencil perspective
  recovery** from the core alone when satellites are compressed by steep viewing angles.
- **Data layer** — payload → frame(header+CRC16) → Reed–Solomon error correction → data cells.
  A standalone Reed–Solomon/CRC codec (`rs.js`) — no external engine.
- **Multi-level capacity** — data cells carry 1, 2, or 3 bits each (2/4/8 grayscale levels),
  multiplying capacity. QR is frozen to 1 bit (black/white); this is WIA-CODE's structural
  capacity lead. Multi-level is only readable because localization is sub-pixel accurate.

## Measured results (reproducible — run `examples/benchmark.js`)

All numbers below are produced by the reference code in this folder; they are not cited claims.

**Capacity is a dial** — more grayscale levels = more bytes, fewer levels = more robustness.
Net payload bytes per code (default ECC per tier):

| grid | 1-bit (25% ECC) | 2-bit robust (50% ECC) | 2-bit max-cap (35% ECC) | 3-bit (50% ECC) |
|---|---|---|---|---|
| S (3728 cells) | 344 | 458 (×1.3) | 598 (×1.7) | 690 |
| M (8848 cells) | 824 | 1099 | 1432 (×1.7) | 1646 |
| L (16016 cells) | 1492 | 1994 (×1.3) | 2590 (×1.7) | 2994 |

QR is frozen to 1 bit (black/white) by ISO/IEC 18004 and cannot follow.

**Robustness vs QR** (same payload, same physical size ~440px, identical degradations; QR
decoded by jsQR, WIA by the reference engine — see `BENCHMARK.md`). Overall decode-rate
(excl. no-degradation): **QR 75% · WIA-1bit 85% · WIA-2bit 80%**.

- **WIA wins perspective outright** — yaw 0–50° all decode; QR fails past 40°. This is the AR-
  glasses / held-at-an-angle regime.
- **WIA wins noise** (σ 30–60 where QR fails) — the low-light / disaster-field regime.
- **WIA-2bit beats QR on both axes at once**: ×1.3 capacity *and* 80% vs 75% robustness.
- Honest losses: both WIA modes lose **blur** (WIA-S has smaller modules at matched size), and
  WIA-2bit loses **extreme low-resolution (0.28× downscale)** — grayscale levels need pixels, so
  a shrunk/faraway code should use 1-bit. This 1-bit↔multi-level split is the capacity/robustness
  dial, chosen per code by conditions (books & screens → 2–3 bit; disaster & distance → 1-bit).

### Color (hue) layer — additive capacity, luma stays intact

A second, independent payload rides in **color** on top of the grayscale one. The trick (YCbCr):
the dot's luminance Y equals the grayscale level, so `toGray` recovers the exact same luma symbol
— **a grayscale scan, a B/W print, or a colorblind viewer loses zero of the base payload.** The
hue carries *extra* bits in the chroma. Default is **K=2 on the blue–yellow (Cb) axis** — the most
JPEG- and print-robust, and fully colorblind-safe. Measured (`examples/test-color.js`):

| mode | S | M | L | vs 2-bit luma |
|---|---|---|---|---|
| 2-luma (no hue) | 458 | 1099 | 1994 | 1.00× |
| 2-luma + 1-hue | 567 | 1369 | 2488 | **×1.25** |
| 2-luma + 2-hue | 683 | 1644 | 2988 | ×1.50 |
| **3-luma + 2-hue** | 1029 | 2459 | **4474** | **×2.25** |

- **Survives JPEG 4:2:0 chroma subsampling** at normal scan resolution (measured, M/L grids,
  including + blur) — the main hazard for any color code.
- **Honest boundary**: hue needs pixels — it is a close-range / screenshot / M–L-grid feature and
  degrades first at distance/low-res. When it fails, the grayscale base still decodes fully
  (graceful degradation). The two hue planes are separately error-corrected (RS 50%).

## How to use

```bash
# Node — data round-trip (encode → render → degrade → detect → decode)
node examples/test-codec.js         # 1-bit
node examples/test-multilevel.js    # 2/3-bit grayscale capacity
node examples/benchmark.js --md     # regenerate BENCHMARK.md

# Browser — install-free camera scanner (open over HTTPS, grant camera)
#   examples/scan.html + examples/wiascan-core.js  (self-contained, no CDN)
```

The scanner engine exposes:
`WiaScan.detectAuto(imageData)` → `{ ok, text, hue, bitsPerCell, grid, shape, cellPx, corners, residPx, … }`
(auto scale, grid, **shape**, level, and color detection), `WiaScan.generate(opts)` (payload → code
PNG), and `WiaScan.renderTestCode(...)`. A browser **generator** (`examples/generate.html`) and
**scanner** (`examples/scan.html`) are included.

## Shapes & content — things QR structurally cannot do

- **Shapes**: `square` · **`round`** · **`heart`** (any silhouette is a data mask; detection is
  radial so it is shape-independent — QR's square finders make it impossible for QR to be round).
  The scanner auto-detects the shape. Round costs ~24–28% capacity vs square at equal bounding box,
  but *wins* on round objects (chips, caps, badges) where a square must inscribe in the circle.
- **Content types / "config-to-code"**: text, URL, and structured **SSH / API / MCP-bootstrap**
  payloads (compact JSON) — a full server/API/agent config carried offline in one code. This is
  WIA's machine-to-machine wedge: a high-capacity, offline, RF-silent,
  perspective-robust mark for AI-agent, robotics, and air-gapped provisioning.
- **Video-embeddable**: because detection tolerates perspective, rotation, and motion far better
  than QR, a WIA code composited into an animated/3D video is readable in practice — not every
  frame decodes under extreme combined motion, but a live scanner only needs one good frame, and
  WIA catches many more of them than QR (use 1-bit for maximum motion robustness).

## Reference implementation (`reference/`)

Pure JavaScript, runs in Node and browsers, no external dependencies.

| file | role |
|---|---|
| `geometry.js` | orbit geometry spec + renderer + data-cell layout |
| `frst.js` | Fast Radial Symmetry detector + sub-pixel/centroid refinement |
| `conic.js` | concentric-circle conic-pencil perspective recovery |
| `locate.js` | anchor localization → module→image homography (`locateRobust`) |
| `codec.js` | data layer: frame + RS + multi-level cells + calibrated classification |
| `rs.js` | standalone Reed–Solomon (GF256) + CRC-16 + block plan (no external engine) |
| `degrade.js` | instrumented degradations (used only by the benchmark/tests) |
| `jsQR.js` | third-party QR decoder, used only for the head-to-head benchmark |

## Honest prior-art & IP note

Concentric-circle machine-readable codes have decades of prior art (MaxiCode 1992, ShotCode,
TopCode, and academic circular fiducials); a key concentric-ring code patent (US 7,621,459)
expired in 2025 and is public domain. WIA-CODE claims novelty **only** in its specific
orbit-anchor detection method (FRST localization + conic-pencil perspective recovery + anchor-
calibrated multi-level sampling), not in "a code made of circles." This project is released as
open source with no patent filed; a formal freedom-to-operate review is recommended before any
commercial claim.

## License

The reference implementation is released under this repository's license (MIT). "WIA Code" may
be retained as a trademark while the code stays open — the QR pattern for an open standard with a
protected name.

## Roadmap

- Perspective robustness for multi-level (sample cells in the rectified frame).
- Optional color (hue) layer carrying extra bits on top of a grayscale-separable base.
- Reference decoders for more platforms (WebAssembly, native) and an engagement with the
  Khronos OpenXR marker-tracking extension as a candidate marker type.
