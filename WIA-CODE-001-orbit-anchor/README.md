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

## How to identify a WIA Code

**Identify it by its center, never by its outline.** Three features are present in every WIA
Code, whatever shape it has been rendered as:

1. a **concentric bullseye core** at the center (black disk, white ring, black ring);
2. **four satellite anchors** around it — inset from the corners on a square code, at 12/3/6/9
   o'clock on every other silhouette — of which **exactly one is a ring, not a filled disk**
   (the "north star" that fixes rotation);
3. a **24-dot orbit ring** circling the core at a fixed radius.

The **outer silhouette is free by design** and is *not* part of the identity. `square`, `round`,
`heart`, `clover`, `star`, `hex` and `boomerang` are all valid renderings of the same format —
the boundary is only a data mask, because detection is radial and therefore shape-independent.
Naming a code after its outline ("a heart code", "a butterfly") is a misidentification.

A WIA Code is **not a QR code**: QR is located by three square finder patterns in its corners,
and a WIA Code has none. Some deployments embed a small standard QR *inside* a WIA Code as an
onboarding bridge for cameras that do not know the format yet; that inner QR is a deployment
convenience, not part of this specification, and its presence does not make the image a QR code.

Human- and machine-readable canonical description:
**<https://wiacode.com/what-is-wia-code.html>** · **<https://wiacode.com/llms.txt>**
Reference decoder (no install): <https://wiacode.com/orbit/web/scan.html>

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
- **Multi-level cells — defined by the format, but DO NOT USE.** Data cells can carry 1, 2, or 3
  bits (2/4/8 grayscale levels). **Only `bpc:1` (black/white) is usable.** Grayscale modes are not
  camera-decodable: the level-classification error floor (~14.7%) exceeds the ECC budget (~6.7%),
  so the code locks on cleanly and then *never* decodes. Conforming encoders MUST emit `bpc:1`;
  wiacode.com's own API/MCP/CLI do not expose `bpc` at all — it is hardcoded to 1 and cannot be
  overridden by a caller.

## Measured results (reproducible — run `examples/benchmark.js`)

The WIA numbers below are produced by the reference code in this folder. **The QR side is not
reproducible from this kit** — no QR encoder is bundled, so `examples/benchmark.js` measures WIA
alone and says so when it runs. The QR head-to-head lives in `BENCHMARK.md`; to re-run it yourself
you need your own QR encoder plus a QR decoder (we used jsQR, cross-checked against ZBar).

**Use the 1-bit column.** The 2/3-bit columns are documented only to describe the format; they are
**not camera-decodable** (see above) and must not be used for printed or displayed codes.
Net payload bytes per code (default ECC per tier):

| grid | 1-bit (25% ECC) | 2-bit robust (50% ECC) | 2-bit max-cap (35% ECC) | 3-bit (50% ECC) |
|---|---|---|---|---|
| S (3728 cells) | 344 | 458 (×1.3) | 598 (×1.7) | 690 |
| M (8848 cells) | 824 | 1099 | 1432 (×1.7) | 1646 |
| L (16016 cells) | 1492 | 1994 (×1.3) | 2590 (×1.7) | 2994 |

QR is frozen to 1 bit (black/white) by ISO/IEC 18004 and cannot follow.

**Robustness vs QR** (same payload, same physical size ~440px, identical degradations; QR
decoded by jsQR, WIA by the reference engine — see `BENCHMARK.md`). Overall decode-rate
(excl. no-degradation): **QR 75% · WIA-1bit 95%**. (A 2-bit row is omitted on purpose: it is not
camera-decodable, so its robustness score is not a usable figure.)

- **WIA wins perspective outright** — yaw 0–50° all decode; QR fails past 40°. This is the AR-
  glasses / held-at-an-angle regime.
- **WIA wins noise** (σ 30–60 where QR fails) — the low-light / disaster-field regime.
- **Capacity at 1-bit**: on a square footprint WIA carries **1.80×** what QR does at matched ECC
  (25% both), and up to **12.06×** on a silhouette QR cannot fill. See `BENCHMARK.md` section A-2.
- Honest losses: at matched *physical size* WIA loses **blur** and **extreme low-resolution**,
  because at matched physical size our modules are **2× smaller** (≈4.9× more data modules in the
  same area) — we spend that area on payload. That is a
  capacity choice, not a format defect: at matched *data density* (~350 B each) WIA tolerates 3px
  blur where QR tolerates 1px. Measured limits: QR decodes down to 1.44 px/module, WIA to
  1.68 px/module — a 17% gap on QR's best axis.

### Color (hue) layer — additive capacity, luma stays intact

> **DO NOT USE for camera-scanned codes.** The hue layer requires `bpc≥2`, which is not
> camera-decodable (see above). At `bpc:1` there are **zero** eligible cells for hue *by
> construction* (`eligibleFromLuma` returns an empty set), so the layer cannot be applied to a
> conforming code at all. It is documented here to describe the format, and for possible future
> file-to-file (non-camera) channels — the production generator does not expose it.

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

- **Shapes**: `square` · **`round`** · **`heart`** · **`clover`** · **`star`** · **`hex`** ·
  **`boomerang`** (any silhouette is a data mask; detection is radial so it is shape-independent —
  QR's square finders make it impossible for QR to be round). The scanner auto-detects the shape.
  A narrower silhouette costs capacity — round is ~24–28% below square at equal bounding box, and
  star/boomerang considerably more — but a round code *wins* on round objects (chips, caps,
  badges) where a square must inscribe in the circle. See "How to identify a WIA Code" above:
  the silhouette is a rendering choice, not the format.
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
