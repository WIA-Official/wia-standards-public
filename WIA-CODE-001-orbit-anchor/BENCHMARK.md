# WIA Code vs QR — Capacity + Robustness Benchmark

_Reproducible: `node benchmark.js`. Same payload (`https://wiacode.com/x`), same physical size, identical degradations (`degrade.js`). QR decoded by jsQR; WIA by the reference engine (`WiaScan.detectAuto`, auto scale/grid/level)._

## A. Net capacity (bytes per code)

WIA multiplies capacity with per-cell grayscale levels; QR is locked to 1-bit black/white by ISO/IEC 18004.

| grid | 1-bit | 2-bit (4-level) | 3-bit (8-level) | multiplier |
|---|---|---|---|---|
| S (3728 cells) | 344 | 458 | 690 | 2-bit ×1.3, 3-bit ×2.0 |
| M (8848 cells) | 824 | 1099 | 1646 | 2-bit ×1.3, 3-bit ×2.0 |
| L (16016 cells) | 1492 | 1994 | 2994 | 2-bit ×1.3, 3-bit ×2.0 |

## B. Robustness head-to-head (same physical size ~440px)

### 원근(yaw)°

| | 0 | 10 | 20 | 30 | 40 | 50 |
|---|---|---|---|---|---|---|
| QR | ✅ | ✅ | ✅ | ✅ | ❌ | ❌ |
| WIA-1bit | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| WIA-2bit | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |

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
| WIA-1bit | ✅ | ✅ | ❌ | ❌ | ❌ |
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

**Overall decode rate (excl. no-degradation): QR 75% · WIA-1bit 85% · WIA-2bit 80%**

WIA wins the perspective + noise regime (the AR-glasses / disaster-field condition) and carries 1.7–3× the data; QR wins blur (WIA-S has smaller modules at matched physical size). Reported honestly, including WIA's losses.
