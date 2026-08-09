'use strict';
/*
 * ============================================================================
 *  WIA Code v2 — 계측된 열화 라이브러리
 * ============================================================================
 *  v1/tools/robustness-harness.js 의 열화 수식(회전·원근기울기·블러·저해상도·
 *  노이즈)을 바이트 단위로 계승한다. 차이는 단 하나:
 *    각 열화가 {img, fwd} 를 반환한다. fwd(x,y) = 원본픽셀 → 열화픽셀 정변환.
 *  이 정변환으로 "정답 앵커 중심이 열화 후 어디에 있는지"를 알 수 있고,
 *  검출 중심과의 재투영 오차(모듈 단위)를 잴 수 있다. v1 하네스는 이진
 *  디코드 성공만 재서 이 값을 낼 수 없었다 — 그게 이 파일의 존재 이유다.
 * ============================================================================
 */

// ── 시드 RNG (하네스와 동일 LCG/Box-Muller) ──────────────────────────────
function makeGauss(seed) {
  let s = (seed >>> 0) || 0x9e3779b9;
  const rnd = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 0x100000000; };
  return (sigma) => { const u = Math.max(1e-9, rnd()), v = rnd();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v) * sigma; };
}

function clone(img) { return { data: new Uint8ClampedArray(img.data), width: img.width, height: img.height }; }
function blank(w, h) { const d = new Uint8ClampedArray(w * h * 4).fill(255); return { data: d, width: w, height: h }; }
const IDENTITY = (x, y) => [x, y];

// 바이리니어 샘플 (경계 밖 = 흰색)
function sample(img, x, y) {
  const { data, width: w, height: h } = img;
  if (x < 0 || y < 0 || x > w - 1 || y > h - 1) return [255, 255, 255];
  const x0 = Math.floor(x), y0 = Math.floor(y), x1 = Math.min(x0 + 1, w - 1), y1 = Math.min(y0 + 1, h - 1);
  const fx = x - x0, fy = y - y0, out = [0, 0, 0];
  for (let c = 0; c < 3; c++) {
    const p00 = data[(y0 * w + x0) * 4 + c], p10 = data[(y0 * w + x1) * 4 + c];
    const p01 = data[(y1 * w + x0) * 4 + c], p11 = data[(y1 * w + x1) * 4 + c];
    out[c] = (p00 * (1 - fx) + p10 * fx) * (1 - fy) + (p01 * (1 - fx) + p11 * fx) * fy;
  }
  return out;
}

// ── 호모그래피 (8×8 가우스 소거) ─────────────────────────────────────────
function solve8(A, b) {
  const n = 8;
  for (let i = 0; i < n; i++) {
    let piv = i; for (let r = i + 1; r < n; r++) if (Math.abs(A[r][i]) > Math.abs(A[piv][i])) piv = r;
    [A[i], A[piv]] = [A[piv], A[i]]; [b[i], b[piv]] = [b[piv], b[i]];
    const d = A[i][i] || 1e-12;
    for (let c = i; c < n; c++) A[i][c] /= d; b[i] /= d;
    for (let r = 0; r < n; r++) { if (r === i) continue; const f = A[r][i]; for (let c = i; c < n; c++) A[r][c] -= f * A[i][c]; b[r] -= f * b[i]; }
  }
  return b;
}
// H mapping A[i] → B[i] (평면 4점 대응)
function homography(A4, B4) {
  const M = [], v = [];
  for (let i = 0; i < 4; i++) {
    const [X, Y] = A4[i], [x, y] = B4[i];
    M.push([X, Y, 1, 0, 0, 0, -X * x, -Y * x]); v.push(x);
    M.push([0, 0, 0, X, Y, 1, -X * y, -Y * y]); v.push(y);
  }
  const h = solve8(M, v); return [h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], 1];
}
function applyH(H, X, Y) { const d = H[6] * X + H[7] * Y + H[8]; return [(H[0] * X + H[1] * Y + H[2]) / d, (H[3] * X + H[4] * Y + H[5]) / d]; }

// ── 4-코너 워프 (원근) ────────────────────────────────────────────────────
//   dstCorners = 출력에서 코드 네 모서리가 놓일 위치. 캔버스 = 원본 크기.
function warpQuad(img, dstCorners) {
  const w = img.width, h = img.height;
  const srcCorners = [[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]];
  const Hout2src = homography(dstCorners, srcCorners);   // 렌더용: out→src
  const Hsrc2out = homography(srcCorners, dstCorners);   // 정변환: src→out
  const out = blank(w, h);
  for (let Y = 0; Y < h; Y++) for (let X = 0; X < w; X++) {
    const [sx, sy] = applyH(Hout2src, X, Y);
    const px = sample(img, sx, sy);
    const o = (Y * w + X) * 4; out.data[o] = px[0]; out.data[o + 1] = px[1]; out.data[o + 2] = px[2]; out.data[o + 3] = 255;
  }
  return { img: out, fwd: (x, y) => applyH(Hsrc2out, x, y), width: w, height: h };
}

// ── 원근 기울기(yaw) — 오른쪽 변을 뒤로 ──────────────────────────────────
function tilt(img, deg) {
  if (deg === 0) return { img: clone(img), fwd: IDENTITY, width: img.width, height: img.height };
  const w = img.width, h = img.height, s = Math.sin(deg * Math.PI / 180);
  const shrink = 1 - 0.9 * Math.abs(s), dx = 0.5 * Math.abs(s) * w;
  const top = h * (1 - shrink) / 2, bot = h - top;
  const dst = [[0, 0], [w - 1 - dx, top], [w - 1 - dx, bot], [0, h - 1]];
  return warpQuad(img, dst);
}

// ── 면내 회전(roll) — 캔버스 확장 ────────────────────────────────────────
function rotate(img, deg) {
  if (deg === 0) return { img: clone(img), fwd: IDENTITY, width: img.width, height: img.height };
  const w = img.width, h = img.height, a = deg * Math.PI / 180;
  const ca = Math.cos(-a), sa = Math.sin(-a);                 // 역변환용
  const W2 = Math.ceil(w * Math.abs(Math.cos(a)) + h * Math.abs(Math.sin(a))) + 8;
  const H2 = Math.ceil(w * Math.abs(Math.sin(a)) + h * Math.abs(Math.cos(a))) + 8;
  const cx = (w - 1) / 2, cy = (h - 1) / 2, CX = (W2 - 1) / 2, CY = (H2 - 1) / 2;
  const out = blank(W2, H2);
  for (let Y = 0; Y < H2; Y++) for (let X = 0; X < W2; X++) {
    const dx = X - CX, dy = Y - CY;
    const px = sample(img, cx + dx * ca - dy * sa, cy + dx * sa + dy * ca);
    const o = (Y * W2 + X) * 4; out.data[o] = px[0]; out.data[o + 1] = px[1]; out.data[o + 2] = px[2]; out.data[o + 3] = 255;
  }
  // 정변환: src(x,y) → out.  [X-CX;Y-CY] = R(a)[x-cx;y-cy]
  const cA = Math.cos(a), sA = Math.sin(a);
  const fwd = (x, y) => { const dx = x - cx, dy = y - cy; return [CX + dx * cA - dy * sA, CY + dx * sA + dy * cA]; };
  return { img: out, fwd, width: W2, height: H2 };
}

// ── 블러 (분리형 박스 × passes, 가우시안 근사) — 기하 불변 ────────────────
function boxBlur(img, radius, passes) {
  if (radius <= 0) return { img: clone(img), fwd: IDENTITY, width: img.width, height: img.height };
  let cur = clone(img); const w = img.width, h = img.height;
  for (let p = 0; p < (passes || 3); p++) {
    const tmp = blank(w, h);
    for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
      let r = 0, g = 0, b = 0, n = 0;
      for (let k = -radius; k <= radius; k++) { const xx = x + k; if (xx < 0 || xx >= w) continue; const o = (y * w + xx) * 4; r += cur.data[o]; g += cur.data[o + 1]; b += cur.data[o + 2]; n++; }
      const o = (y * w + x) * 4; tmp.data[o] = r / n; tmp.data[o + 1] = g / n; tmp.data[o + 2] = b / n; tmp.data[o + 3] = 255;
    }
    const tmp2 = blank(w, h);
    for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
      let r = 0, g = 0, b = 0, n = 0;
      for (let k = -radius; k <= radius; k++) { const yy = y + k; if (yy < 0 || yy >= h) continue; const o = (yy * w + x) * 4; r += tmp.data[o]; g += tmp.data[o + 1]; b += tmp.data[o + 2]; n++; }
      const o = (y * w + x) * 4; tmp2.data[o] = r / n; tmp2.data[o + 1] = g / n; tmp2.data[o + 2] = b / n; tmp2.data[o + 3] = 255;
    }
    cur = tmp2;
  }
  return { img: cur, fwd: IDENTITY, width: w, height: h };
}

// ── 저해상도 (축소→복원) — 기하 불변 ─────────────────────────────────────
function resample(img, factor) {
  if (factor >= 0.999) return { img: clone(img), fwd: IDENTITY, width: img.width, height: img.height };
  const w = img.width, h = img.height;
  const sw = Math.max(8, Math.round(w * factor)), sh = Math.max(8, Math.round(h * factor));
  const small = blank(sw, sh);
  for (let y = 0; y < sh; y++) for (let x = 0; x < sw; x++) {
    const px = sample(img, x / (sw - 1) * (w - 1), y / (sh - 1) * (h - 1));
    const o = (y * sw + x) * 4; small.data[o] = px[0]; small.data[o + 1] = px[1]; small.data[o + 2] = px[2]; small.data[o + 3] = 255;
  }
  const out = blank(w, h);
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    const px = sample(small, x / (w - 1) * (sw - 1), y / (h - 1) * (sh - 1));
    const o = (y * w + x) * 4; out.data[o] = px[0]; out.data[o + 1] = px[1]; out.data[o + 2] = px[2]; out.data[o + 3] = 255;
  }
  return { img: out, fwd: IDENTITY, width: w, height: h };
}

// ── 가우시안 노이즈 — 기하 불변 ──────────────────────────────────────────
function noise(img, sigma, seed) {
  if (sigma <= 0) return { img: clone(img), fwd: IDENTITY, width: img.width, height: img.height };
  const g = makeGauss(seed || 12345), out = clone(img);
  for (let i = 0; i < out.data.length; i += 4) {
    const n = g(sigma);
    out.data[i] = Math.max(0, Math.min(255, out.data[i] + n));
    out.data[i + 1] = Math.max(0, Math.min(255, out.data[i + 1] + n));
    out.data[i + 2] = Math.max(0, Math.min(255, out.data[i + 2] + n));
  }
  return { img: out, fwd: IDENTITY, width: img.width, height: img.height };
}

// ── 크로마 서브샘플(JPEG 4:2:0 근사) — 색압축의 벽 실측용 ─────────────────
//   YCbCr 변환 → Cb/Cr 을 blk×blk 박스평균(Y 는 유지) → RGB. blk=2 가 4:2:0.
function chromaSubsample(img, blk) {
  blk = blk || 2; const w = img.width, h = img.height, d = img.data, out = clone(img);
  const Y = new Float32Array(w * h), Cb = new Float32Array(w * h), Cr = new Float32Array(w * h);
  for (let i = 0, j = 0; i < d.length; i += 4, j++) {
    const R = d[i], G = d[i + 1], B = d[i + 2];
    Y[j] = 0.299 * R + 0.587 * G + 0.114 * B;
    Cb[j] = -0.168736 * R - 0.331264 * G + 0.5 * B;
    Cr[j] = 0.5 * R - 0.418688 * G - 0.081312 * B;
  }
  for (let by = 0; by < h; by += blk) for (let bx = 0; bx < w; bx += blk) {
    let sb = 0, sr = 0, n = 0;
    for (let y = by; y < Math.min(h, by + blk); y++) for (let x = bx; x < Math.min(w, bx + blk); x++) { sb += Cb[y * w + x]; sr += Cr[y * w + x]; n++; }
    const mb = sb / n, mr = sr / n;
    for (let y = by; y < Math.min(h, by + blk); y++) for (let x = bx; x < Math.min(w, bx + blk); x++) { Cb[y * w + x] = mb; Cr[y * w + x] = mr; }
  }
  for (let j = 0, i = 0; j < w * h; j++, i += 4) {
    const y = Y[j], cb = Cb[j], cr = Cr[j];
    out.data[i] = Math.max(0, Math.min(255, y + 1.402 * cr));
    out.data[i + 1] = Math.max(0, Math.min(255, y - 0.344136 * cb - 0.714136 * cr));
    out.data[i + 2] = Math.max(0, Math.min(255, y + 1.772 * cb));
  }
  return { img: out, fwd: IDENTITY, width: w, height: h };
}

module.exports = { tilt, rotate, boxBlur, resample, noise, clone, blank, sample, homography, applyH, chromaSubsample };
