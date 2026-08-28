# WIA Code vs QR — Capacity + Robustness Benchmark

_Reproducible: `node benchmark.js`. Same payload (`https://wiacode.com/x`), same physical size, identical degradations (`degrade.js`). QR decoded by jsQR; WIA by the reference engine (`WiaScan.detectAuto`, auto scale/grid/level)._

## A. Net capacity (bytes per code)

_**Use the 1-bit column; do not use `bpc≥2`.** The multiplier column compares WIA against **its own 1-bit mode**, not against QR. Multi-level is **not camera-decodable** — the level-classification error floor (~14.7%) exceeds the ECC budget (~6.7%), so such a code locks on and then never decodes. The 2/3-bit columns describe the format; they are not an operating recommendation, and their robustness rows below are not usable figures. See the QR head-to-head in section A-2._

| grid | 1-bit | 2-bit (4-level) | 3-bit (8-level) | multiplier |
|---|---|---|---|---|
| S (3728 cells) | 344 | 598 | 690 | 2-bit ×1.7, 3-bit ×2.0 |
| M (8848 cells) | 824 | 1432 | 1646 | 2-bit ×1.7, 3-bit ×2.0 |
| L (16016 cells) | 1492 | 2590 | 2994 | 2-bit ×1.7, 3-bit ×2.0 |

## A-2. QR head-to-head — capacity at the same footprint

_Same total footprint (136 cells), same cell/module pitch, matched error correction (WIA RS 25% ↔ QR ECC-Q 25%). A square label is fully printable so QR gets the whole footprint; on a non-square silhouette QR is confined to the largest square that fits inside the shape (computed by DP over the silhouette bitmask, not estimated)._

| silhouette | WIA (booster incl.) | QR usable (cells) | QR version | QR bytes (Q) | WIA/QR |
|---|---|---|---|---|---|
| heart | 854 | 78 | v13 | 241 | 3.54× |
| rose | 586 | 75 | v12 | 203 | 2.89× |
| round | 1093 | 90 | v16 | 322 | 3.39× |
| clover | 521 | 42 | v4 | 46 | 11.33× |
| star | 386 | 50 | v6 | 74 | 5.22× |
| boomerang | 386 | 40 | v3 | 32 | 12.06× |
| hex | 861 | 78 | v13 | 241 | 3.57× |
| square | 1450 | 136 | v27 | 805 | 1.80× |

### Legacy silhouettes (generation retired — decoding still required)

_These shapes can no longer be produced by the generator, but codes in them are already in the field (printed credentials, a distributed WIA Stream video). A conforming decoder must still read them; the conformance kit marks their vectors `legacy-decode-only`._

| silhouette | WIA (booster incl.) | QR usable (cells) | QR version | QR bytes (Q) | WIA/QR |
|---|---|---|---|---|---|
| bubble | 1098 | 108 | v20 | 482 | 2.28× |

WIA carries more in every silhouette, including the square. The honest counterpoint is in section B: **QR is more blur-tolerant** at matched physical size, because our cells are smaller.

## B. Robustness head-to-head (same physical size ~440px)

### 원근(yaw)°

| | 0 | 10 | 20 | 30 | 40 | 50 |
|---|---|---|---|---|---|---|
| QR | ✅ | ✅ | ✅ | ✅ | ❌ | ❌ |
| WIA-1bit | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| WIA-2bit | ✅ | ✅ | ✅ | ✅ | ✅ | ❌ |

### 회전(roll)°

| | 0 | 10 | 20 | 30 | 45 |
|---|---|---|---|---|---|
| QR | ✅ | ✅ | ✅ | ✅ | ✅ |
| WIA-1bit | ✅ | ✅ | ✅ | ✅ | ✅ |
| WIA-2bit | ✅ | ✅ | ✅ | ✅ | ✅ |

### 블러(px)

| | 0 | 1 | 2 | 3 | 4 |
|---|---|---|---|---|---|
| QR | ✅ | ✅ | ✅ | ✅ | ✅ |
| WIA-1bit | ✅ | ✅ | ✅ | ✅ | ❌ |
| WIA-2bit | ✅ | ✅ | ❌ | ❌ | ❌ |

### 저해상도(축소)

| | 1 | 0.6 | 0.45 | 0.35 | 0.28 |
|---|---|---|---|---|---|
| QR | ✅ | ✅ | ✅ | ✅ | ✅ |
| WIA-1bit | ✅ | ✅ | ✅ | ✅ | ✅ |
| WIA-2bit | ✅ | ✅ | ✅ | ✅ | ❌ |

### 노이즈(σ)

| | 0 | 15 | 30 | 45 | 60 |
|---|---|---|---|---|---|
| QR | ✅ | ✅ | ❌ | ❌ | ❌ |
| WIA-1bit | ✅ | ✅ | ✅ | ✅ | ✅ |
| WIA-2bit | ✅ | ✅ | ✅ | ✅ | ✅ |

**Overall decode rate (excl. no-degradation): QR 75% · WIA-1bit 95% · WIA-2bit 75%**

WIA wins the perspective + noise regime (the AR-glasses / disaster-field condition) and, at `bpc:1` — the only camera-decodable setting — carries **1.80× on a square and up to 12.06× on a silhouette** (section A-2); QR wins blur (WIA-S has smaller modules at matched physical size). Reported honestly, including WIA's losses.

_Decoder note: WIA is decoded by our own reference engine, QR by jsQR. At this geometry that is fair — an independent decoder (ZBar) scores QR 71.4% here, close to jsQR's 76.2%. The two diverge sharply at high QR versions (v27: jsQR 17.6%, ZBar 52.9%), so **any comparison above ~v20 must report the ZBar figure**, not jsQR's._
