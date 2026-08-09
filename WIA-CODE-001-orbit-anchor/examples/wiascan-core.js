/* WIA Code Scanner Engine — 자동생성(build-core.js). 편집 금지. */
(function(){
"use strict";
var __mods={};
function require(name){var k=name.replace(/^.*\//,"").replace(/\.js$/,"");if(!(k in __mods))throw new Error("mod?"+k);return __mods[k];}
function __def(name,fn){var module={exports:{}};fn(module,module.exports);__mods[name]=module.exports;}

__def("degrade", function(module, exports){
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

});

__def("frst", function(module, exports){
'use strict';
/*
 * ============================================================================
 *  WIA Code v2 — Fast Radial Symmetry Transform (FRST) 검출기
 * ============================================================================
 *  Fable 5 설계의 심장. QR의 스캔라인 런-길이(점 검출)를 버리고, 방사대칭
 *  적분 검출로 코어/위성 중심을 찾는다.
 *
 *  원리(Loy & Zelinsky 2003 변형):
 *    - 어두운 대칭 중심(코어 중심·위성 원판은 검정)은, 경계 그레이디언트가
 *      바깥(어두움→밝음)을 향하므로, 각 에지픽셀에서 반대방향으로 반경 n 떨어진
 *      점(p - n·ĝ)에 투표하면 중심에 표가 쌓인다.
 *    - 여러 반경 n에 걸쳐 누적 → 불스아이(다중 링)에 표가 압도적으로 몰린다.
 *    - 원주 O(2πr)개 픽셀이 한 점에 투표 → 가우시안 노이즈가 √N로 평균화.
 *      (QR 런-길이의 얇은 점 검출과 근본적으로 다른 노이즈 마진)
 *
 *  입력은 그레이스케일. 출력은 대칭맵 S와 극대점(후보 중심).
 * ============================================================================
 */

// RGBA → 그레이 (Float32, 0..255)
function toGray(img) {
  const { data, width: w, height: h } = img;
  const g = new Float32Array(w * h);
  for (let i = 0, j = 0; i < data.length; i += 4, j++) g[j] = 0.299 * data[i] + 0.587 * data[i + 1] + 0.114 * data[i + 2];
  return { g, w, h };
}

// Sobel 그레이디언트
function sobel(gray) {
  const { g, w, h } = gray;
  const gx = new Float32Array(w * h), gy = new Float32Array(w * h), mag = new Float32Array(w * h);
  let maxMag = 1e-6;
  for (let y = 1; y < h - 1; y++) for (let x = 1; x < w - 1; x++) {
    const i = y * w + x;
    const a = g[i - w - 1], b = g[i - w], c = g[i - w + 1];
    const d = g[i - 1],                 f = g[i + 1];
    const p = g[i + w - 1], q = g[i + w], r = g[i + w + 1];
    const sx = (c + 2 * f + r) - (a + 2 * d + p);
    const sy = (p + 2 * q + r) - (a + 2 * b + c);
    gx[i] = sx; gy[i] = sy; const m = Math.hypot(sx, sy); mag[i] = m; if (m > maxMag) maxMag = m;
  }
  return { gx, gy, mag, maxMag, w, h };
}

// 3×3 박스블러 × passes (대칭맵 평활)
function smooth(buf, w, h, passes) {
  let cur = buf;
  for (let p = 0; p < (passes || 1); p++) {
    const out = new Float32Array(w * h);
    for (let y = 1; y < h - 1; y++) for (let x = 1; x < w - 1; x++) {
      const i = y * w + x;
      out[i] = (cur[i] + cur[i - 1] + cur[i + 1] + cur[i - w] + cur[i + w] +
                cur[i - w - 1] + cur[i - w + 1] + cur[i + w - 1] + cur[i + w + 1]) / 9;
    }
    cur = out;
  }
  return cur;
}

/*
 * frst(gray, radii, opts) → { S, w, h }
 *   radii  : 투표 반경 배열(px). 코어 링 반경 + 위성 반경을 커버하도록.
 *   opts.gradFrac : 그레이디언트 임계 (maxMag 대비, 기본 0.15)
 *   opts.kappa    : 정규화 스케일 (기본 9.9)
 *   opts.alpha    : 방사 강도 지수 (기본 2)
 *   opts.polarity : 'dark'(기본)=어두운 중심만. 'both'=양극성.
 */
function frst(grayObj, radii, opts) {
  opts = opts || {};
  const { g, w, h } = grayObj;
  const sob = sobel(grayObj);
  const thr = (opts.gradFrac != null ? opts.gradFrac : 0.15) * sob.maxMag;
  const kappa = opts.kappa || 9.9, alpha = opts.alpha != null ? opts.alpha : 2;
  const dark = (opts.polarity || 'dark') !== 'both';
  const S = new Float32Array(w * h);

  for (const n of radii) {
    const O = new Float32Array(w * h);   // 방향(카운트) 누적
    const M = new Float32Array(w * h);   // 크기 누적
    for (let y = 1; y < h - 1; y++) for (let x = 1; x < w - 1; x++) {
      const i = y * w + x, m = sob.mag[i];
      if (m < thr) continue;
      const ux = sob.gx[i] / m, uy = sob.gy[i] / m;
      // 어두운 중심 → 반대방향(-n) 에 투표
      const nx = Math.round(x - n * ux), ny = Math.round(y - n * uy);
      if (nx >= 0 && nx < w && ny >= 0 && ny < h) { const j = ny * w + nx; O[j] += 1; M[j] += m; }
      if (!dark) { // 양극성: 순방향(+n)에도 음의 투표
        const px = Math.round(x + n * ux), py = Math.round(y + n * uy);
        if (px >= 0 && px < w && py >= 0 && py < h) { const j = py * w + px; O[j] -= 1; M[j] -= m; }
      }
    }
    // 정규화 + 방사 강도. 반경 n에 비례한 약한 평활(투표 산포 흡수).
    const blurPass = Math.max(1, Math.round(n / 6));
    const Ob = smooth(O, w, h, blurPass), Mb = smooth(M, w, h, blurPass);
    for (let idx = 0; idx < w * h; idx++) {
      let o = Ob[idx]; if (dark && o < 0) o = 0; else o = Math.abs(o);
      const on = Math.min(o, kappa) / kappa;
      S[idx] += Math.pow(on, alpha) * (Mb[idx] > 0 ? Mb[idx] : 0);
    }
  }
  // 최종 평활
  return { S: smooth(S, w, h, 2), w, h };
}

// 극대점 추출 (비최대억제). win=억제 반경(px). 반환: [{x,y,score}] 내림차순.
function peaks(Sobj, opts) {
  opts = opts || {};
  const { S, w, h } = Sobj;
  let max = 1e-9; for (let i = 0; i < S.length; i++) if (S[i] > max) max = S[i];
  const thr = (opts.thrFrac != null ? opts.thrFrac : 0.12) * max;
  const win = opts.win || 8, topK = opts.topK || 24;
  const cand = [];
  for (let y = win; y < h - win; y++) for (let x = win; x < w - win; x++) {
    const v = S[y * w + x]; if (v < thr) continue;
    let isMax = true;
    for (let dy = -win; dy <= win && isMax; dy++) for (let dx = -win; dx <= win; dx++) {
      if (dx === 0 && dy === 0) continue;
      if (S[(y + dy) * w + (x + dx)] > v) { isMax = false; break; }
    }
    if (isMax) cand.push({ x, y, score: v });
  }
  cand.sort((a, b) => b.score - a.score);
  // 근접 중복 제거 + 포물선 서브픽셀 정밀화(far쪽 압축 영역에서 정수 양자화 오차 제거)
  const out = [];
  for (const c of cand) {
    if (out.some(o => Math.hypot(o.x - c.x, o.y - c.y) < win)) continue;
    const i = c.y * w + c.x;
    const sx = subpix(S[i - 1], S[i], S[i + 1]);
    const sy = subpix(S[i - w], S[i], S[i + w]);
    out.push({ x: c.x + sx, y: c.y + sy, score: c.score });
    if (out.length >= topK) break;
  }
  return out;
}
// 1D 포물선 정점 오프셋 (-0.5..0.5)
function subpix(a, b, c) {
  const d = a - 2 * b + c;
  if (Math.abs(d) < 1e-9) return 0;
  const o = 0.5 * (a - c) / d;
  return o > 0.5 ? 0.5 : o < -0.5 ? -0.5 : o;
}

// 방사 강도 프로파일: 중심에서 바깥으로 R까지 그레이 평균(각도 평균). 링 구조 판정용.
function radialProfile(grayObj, cx, cy, R, steps) {
  const { g, w, h } = grayObj; steps = steps || Math.max(8, Math.round(R));
  const prof = new Float32Array(steps);
  for (let s = 0; s < steps; s++) {
    const r = (s + 0.5) / steps * R; let sum = 0, n = 0;
    const na = Math.max(8, Math.round(2 * Math.PI * r));
    for (let a = 0; a < na; a++) {
      const th = 2 * Math.PI * a / na, x = cx + r * Math.cos(th), y = cy + r * Math.sin(th);
      if (x < 0 || y < 0 || x >= w - 1 || y >= h - 1) continue;
      const x0 = Math.floor(x), y0 = Math.floor(y), fx = x - x0, fy = y - y0, i = y0 * w + x0;
      sum += (g[i] * (1 - fx) + g[i + 1] * fx) * (1 - fy) + (g[i + w] * (1 - fx) + g[i + w + 1] * fx) * fy; n++;
    }
    prof[s] = n ? sum / n : 255;
  }
  return prof;
}

/*
 * 암부 무게중심 정밀화: (cx,cy) 주위 반경 R 창에서, 배경보다 어두운 픽셀의
 * 무게중심을 구한다. 투영된 균일 원판(위성)·대칭 구조(코어)의 밝기 무게중심은
 * 아핀 불변으로 참 투영중심과 일치 → 원근에서 원형-FRST 무게중심 편향을 제거.
 * 2패스: ①창 평균 ②(평균-그레이)를 가중치로 무게중심.
 */
function refineCentroid(grayObj, cx, cy, R, bright) {
  const { g, w, h } = grayObj;
  const x0 = Math.max(1, Math.floor(cx - R)), x1 = Math.min(w - 2, Math.ceil(cx + R));
  const y0 = Math.max(1, Math.floor(cy - R)), y1 = Math.min(h - 2, Math.ceil(cy + R));
  if (x1 <= x0 || y1 <= y0) return { x: cx, y: cy, ok: false };
  let sum = 0, n = 0;
  for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) {
    if (Math.hypot(x - cx, y - cy) > R) continue;
    sum += g[y * w + x]; n++;
  }
  if (!n) return { x: cx, y: cy, ok: false };
  const mean = sum / n;
  let wsum = 0, sx = 0, sy = 0;
  for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) {
    if (Math.hypot(x - cx, y - cy) > R) continue;
    // bright=true → 밝을수록 큰 가중(도넛 흰 구멍), 기본 → 어두울수록(원판/링)
    const ww = bright ? (g[y * w + x] - mean) : (mean - g[y * w + x]);
    if (ww <= 0) continue;
    wsum += ww; sx += ww * x; sy += ww * y;
  }
  if (wsum <= 0) return { x: cx, y: cy, ok: false };
  return { x: sx / wsum, y: sy / wsum, ok: true };
}

module.exports = { toGray, sobel, frst, peaks, radialProfile, refineCentroid };

});

__def("conic", function(module, exports){
'use strict';
/*
 * ============================================================================
 *  WIA Code v2 — 코어 동심원(conic-pencil) 원근복원
 * ============================================================================
 *  설계문서 §1.2-2 구현. 급격한 원근(yaw≥30°)에서 코너 위성이 타원으로 압축·
 *  소실해 `no-core-surround`로 죽던 tail을, 코어 하나만으로 연다.
 *
 *  원리(고전 결과, MaxiCode·동심원 포즈추정 계보):
 *    - 세계의 원(circle)은 사영변환에서 타원(conic)으로 간다. 코어는 동심원
 *      불스아이 → 이미지에선 "동심 타원 2개"(반경 1.5모듈 원판edge, 5.5모듈
 *      바깥edge)로 관측된다. 코어는 원근에서도 반석(0.05모듈)이라 항상 잡힌다.
 *    - 두 이미지 타원 C1,C2 의 conic-pencil C1-λC2 에는 rank-1 퇴화원뿔이 하나
 *      있고, 그것이 = (소실선)². 즉 코어만으로 평면의 소실선을 닫힌형으로 얻는다.
 *    - 코어 참중심(원의 중심 이미지) = 소실선의 극(pole) = adj(C)·l.
 *      (타원의 무게중심이 아니다 — 원근에서 그 둘은 다르다.)
 *    - 소실선+원뿔 하나로 이미지를 similarity(회전·스케일·평행이동)만 남기고
 *      정면화(metric rectify)한다. 회전/스케일 잔여 자유도는 정면화된 그림에서
 *      위성(이제 원형 복원)·북극성으로 확정 → 기존 locate() 재사용.
 *
 *  이 파일은 순수 기하(픽셀→픽셀). 데이터 코덱과 무관.
 * ============================================================================
 */

// ── 선형대수 (작은 밀집행렬) ───────────────────────────────────────────────

// 대칭 n×n 고유분해 (cyclic Jacobi). 반환 { val:[..], vec:[[..]..] (열=고유벡터) }.
function jacobiEig(Ain, n) {
  const A = Ain.map(r => Float64Array.from(r));
  const V = Array.from({ length: n }, (_, i) => { const r = new Float64Array(n); r[i] = 1; return r; });
  for (let sweep = 0; sweep < 100; sweep++) {
    // off-diagonal 크기
    let off = 0;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) off += A[p][q] * A[p][q];
    if (off < 1e-24) break;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) {
      const apq = A[p][q];
      if (Math.abs(apq) < 1e-300) continue;
      const app = A[p][p], aqq = A[q][q];
      const phi = 0.5 * Math.atan2(2 * apq, aqq - app);
      const c = Math.cos(phi), s = Math.sin(phi);
      for (let k = 0; k < n; k++) {
        const akp = A[k][p], akq = A[k][q];
        A[k][p] = c * akp - s * akq; A[k][q] = s * akp + c * akq;
      }
      for (let k = 0; k < n; k++) {
        const apk = A[p][k], aqk = A[q][k];
        A[p][k] = c * apk - s * aqk; A[q][k] = s * apk + c * aqk;
      }
      for (let k = 0; k < n; k++) {
        const vkp = V[k][p], vkq = V[k][q];
        V[k][p] = c * vkp - s * vkq; V[k][q] = s * vkp + c * vkq;
      }
    }
  }
  const val = new Array(n), vec = Array.from({ length: n }, () => new Float64Array(n));
  for (let i = 0; i < n; i++) { val[i] = A[i][i]; for (let k = 0; k < n; k++) vec[k][i] = V[k][i]; }
  return { val, vec };
}

// 3×3 역행렬
function inv3(m) {
  const [a, b, c, d, e, f, g, h, i] = [m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2]];
  const A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
  const det = a * A + b * B + c * C;
  if (Math.abs(det) < 1e-300) return null;
  const id = 1 / det;
  return [
    [A * id, (c * h - b * i) * id, (b * f - c * e) * id],
    [B * id, (a * i - c * g) * id, (c * d - a * f) * id],
    [C * id, (b * g - a * h) * id, (a * e - b * d) * id],
  ];
}

// 3×3 수반행렬(adjugate) = det·inv. pole 계산에 det 무관하게 쓸 수 있어 안전.
function adj3(m) {
  const a = m[0][0], b = m[0][1], c = m[0][2], d = m[1][0], e = m[1][1], f = m[1][2], g = m[2][0], h = m[2][1], i = m[2][2];
  return [
    [e * i - f * h, c * h - b * i, b * f - c * e],
    [f * g - d * i, a * i - c * g, c * d - a * f],
    [d * h - e * g, b * g - a * h, a * e - b * d],
  ];
}
function mul3(A, B) {
  const C = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) { let s = 0; for (let k = 0; k < 3; k++) s += A[i][k] * B[k][j]; C[i][j] = s; }
  return C;
}
function matVec3(M, v) { return [M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2], M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2], M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2]]; }
function transpose3(M) { return [[M[0][0], M[1][0], M[2][0]], [M[0][1], M[1][1], M[2][1]], [M[0][2], M[1][2], M[2][2]]]; }
function frob(M) { let s = 0; for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) s += M[i][j] * M[i][j]; return Math.sqrt(s); }
function scale3(M, k) { return M.map(r => r.map(v => v * k)); }
function sub3(A, B) { return A.map((r, i) => r.map((v, j) => v - B[i][j])); }

// ── 코어 링 에지점 추출 ────────────────────────────────────────────────────
// 중심(cx,cy) 주위로 방사 스캔. 각 각도에서 바깥으로 나가며 그레이가
//   disk(검)→흰링→검링→흰 세퍼레이터. 두 개의 dark→light 상승에지(r≈1.5, ≈5.5
//   모듈)를 서브픽셀로 잡는다. 스케일 무지 상태에서 "첫 두 상승에지"라는 순서로 견고.
function extractCoreRings(grayObj, cx, cy, rMaxPx, nAng) {
  const { g, w, h } = grayObj;
  nAng = nAng || 180;
  const sampAt = (x, y) => {
    if (x < 0 || y < 0 || x >= w - 1 || y >= h - 1) return null;
    const x0 = Math.floor(x), y0 = Math.floor(y), fx = x - x0, fy = y - y0, i = y0 * w + x0;
    return (g[i] * (1 - fx) + g[i + 1] * fx) * (1 - fy) + (g[i + w] * (1 - fx) + g[i + w + 1] * fx) * fy;
  };
  const step = 0.5;                       // 방사 샘플 간격(px)
  const inner = [], outer = [];
  for (let a = 0; a < nAng; a++) {
    const th = 2 * Math.PI * a / nAng, ct = Math.cos(th), st = Math.sin(th);
    // 프로파일 샘플
    const rs = [], vs = [];
    for (let r = 1; r <= rMaxPx; r += step) { const v = sampAt(cx + r * ct, cy + r * st); if (v == null) break; rs.push(r); vs.push(v); }
    if (vs.length < 8) continue;
    // 상승에지 후보: dGray/dr 국소최대(+). 중심이 어두운지 확인(disk).
    const rises = [];                       // {r, mag}
    for (let k = 2; k < vs.length - 2; k++) {
      const gm1 = vs[k + 1] - vs[k - 1];    // 중심차분(상승=+)
      if (gm1 <= 0) continue;
      // 국소최대
      const gprev = vs[k] - vs[k - 2], gnext = vs[k + 2] - vs[k];
      if (gm1 >= gprev && gm1 >= gnext && gm1 > 40) {
        // 서브픽셀: 그레이디언트 3점 포물선 정점
        const gm = vs[k] - vs[k - 2], gp = vs[k + 2] - vs[k], gc = gm1;
        const den = (gm - 2 * gc + gp);
        let off = Math.abs(den) > 1e-9 ? 0.5 * (gm - gp) / den : 0;
        off = Math.max(-1, Math.min(1, off));
        rises.push({ r: rs[k] + off * step, mag: gc });
      }
    }
    if (rises.length < 2) continue;
    // 첫 두 상승에지 = 원판경계(1.5), 검은링 바깥경계(5.5).
    inner.push([cx + rises[0].r * ct, cy + rises[0].r * st]);
    outer.push([cx + rises[1].r * ct, cy + rises[1].r * st]);
  }
  return { inner, outer };
}

// ── 대수적 타원(원뿔) 적합 (Hartley 정규화 + 최소SV) ───────────────────────
// 점들 → conic 대칭행렬 C: [A,B/2,D/2; B/2,C,E/2; D/2,E/2,F] with A x²+B xy+C y²+D x+E y+F.
function fitConic(pts) {
  const n = pts.length;
  if (n < 6) return null;
  // Hartley 정규화 (수치안정)
  let mx = 0, my = 0; for (const p of pts) { mx += p[0]; my += p[1]; } mx /= n; my /= n;
  let s = 0; for (const p of pts) s += Math.hypot(p[0] - mx, p[1] - my); s /= n;
  const sc = s > 1e-9 ? Math.SQRT2 / s : 1;
  const T = [[sc, 0, -sc * mx], [0, sc, -sc * my], [0, 0, 1]];
  // 설계행렬 D (n×6), 정규화 좌표
  const rows = [];
  for (const p of pts) { const x = sc * (p[0] - mx), y = sc * (p[1] - my); rows.push([x * x, x * y, y * y, x, y, 1]); }
  // 6×6 스캐터
  const S = Array.from({ length: 6 }, () => new Float64Array(6));
  for (const r of rows) for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) S[i][j] += r[i] * r[j];
  const { val, vec } = jacobiEig(S, 6);
  // 최소 고유값의 고유벡터 = 계수
  let mi = 0; for (let i = 1; i < 6; i++) if (val[i] < val[mi]) mi = i;
  const c = vec.map(row => row[mi]);       // [A,B,C,D,E,F] (정규화좌표)
  // 타원 판정 (B²-4AC<0)
  const disc = c[1] * c[1] - 4 * c[0] * c[2];
  // 정규화좌표 conic 행렬
  const Cn = [[c[0], c[1] / 2, c[3] / 2], [c[1] / 2, c[2], c[4] / 2], [c[3] / 2, c[4] / 2, c[5]]];
  // 역정규화: C = Tᵀ Cn T
  const C = mul3(transpose3(T), mul3(Cn, T));
  return { C: scale3(C, 1 / (frob(C) || 1)), isEllipse: disc < 0, n };
}

// 3×3 행렬식
function det3(m) {
  return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
       - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
       + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}
// 실근 3차방정식 aλ³+bλ²+cλ+d=0. 실근 전부 반환(근접 이중근 포함).
//   한 실근을 Cardano/Newton으로 얻고 2차로 deflation → 나머지 두 실근 복구.
function solveQuadR(a, b, c) {
  if (Math.abs(a) < 1e-14) return Math.abs(b) < 1e-14 ? [] : [-c / b];
  const D = b * b - 4 * a * c; if (D < -1e-9 * (b * b + 1)) return [];
  const s = Math.sqrt(Math.max(0, D)); return [(-b + s) / (2 * a), (-b - s) / (2 * a)];
}
function solveCubic(a, b, c, d) {
  if (Math.abs(a) < 1e-14) return solveQuadR(b, c, d);
  const B = b / a, C = c / a, D = d / a;
  const p = C - B * B / 3, q = 2 * B * B * B / 27 - B * C / 3 + D;
  const disc = q * q / 4 + p * p * p / 27;
  let r1;
  if (disc > 0) {
    const s = Math.sqrt(disc);
    r1 = Math.cbrt(-q / 2 + s) + Math.cbrt(-q / 2 - s) - B / 3;
  } else {
    const r = Math.sqrt(Math.max(1e-30, -p * p * p / 27));
    const phi = Math.acos(Math.max(-1, Math.min(1, -q / 2 / r)));
    r1 = 2 * Math.cbrt(r) * Math.cos(phi / 3) - B / 3;
  }
  // Newton 한두 번(정밀화)
  for (let it = 0; it < 3; it++) {
    const f = ((r1 + B) * r1 + C) * r1 + D, fp = (3 * r1 + 2 * B) * r1 + C;
    if (Math.abs(fp) < 1e-14) break; r1 -= f / fp;
  }
  // deflation: λ³+Bλ²+Cλ+D = (λ-r1)(λ²+e1λ+e0)
  const e1 = B + r1, e0 = C + r1 * e1;
  const Dq = e1 * e1 - 4 * e0;
  // Dq<0 = fit 잡음으로 이중근이 복소화 → 실수부(-e1/2)를 이중근으로.
  const rest = Dq >= 0 ? [(-e1 + Math.sqrt(Dq)) / 2, (-e1 - Math.sqrt(Dq)) / 2] : [-e1 / 2, -e1 / 2];
  return [r1, ...rest].filter(isFinite);
}
// 4점 샘플로 det(C1-λC2)(=λ의 3차) 계수 복원
function pencilCubicCoeffs(C1, C2) {
  const xs = [-1, 0, 1, 2], ys = xs.map(l => det3(sub3(C1, scale3(C2, l))));
  // Vandermonde 4×4 해
  const M = xs.map(l => [l * l * l, l * l, l, 1]).map(r => Float64Array.from(r));
  const v = Float64Array.from(ys);
  for (let i = 0; i < 4; i++) {
    let p = i; for (let r = i + 1; r < 4; r++) if (Math.abs(M[r][i]) > Math.abs(M[p][i])) p = r;
    [M[i], M[p]] = [M[p], M[i]]; [v[i], v[p]] = [v[p], v[i]];
    const dg = M[i][i] || 1e-30;
    for (let r = 0; r < 4; r++) { if (r === i) continue; const f = M[r][i] / dg; for (let c = i; c < 4; c++) M[r][c] -= f * M[i][c]; v[r] -= f * v[i]; }
  }
  return [v[0] / M[0][0], v[1] / M[1][1], v[2] / M[2][2], v[3] / M[3][3]];
}

/*
 * 두 동심원 이미지(C1=바깥원, C2=안쪽원) → 소실선 l + 참중심.
 * 이론: 동심원 conic-pencil C1-λC2 의 det=0 근 3개 =
 *   · 이중근 λ=1  → M=rank1=(소실선)²           → l = M 의 지배고유벡터
 *   · 단일근 λ=r1²/r2² → M=rank2=중심 지나는 두 직선 → 중심 = M 의 널공간
 * 원근 왜곡을 완전히 견디는 닫힌형 해. (무게중심 정밀화가 원근서 편향되던 문제 근절)
 * 반환 { ok, l:[a,b,c], center:[x,y], single, lamVan }.
 */
function recoverFromConcentric(C1, C2) {
  const coef = pencilCubicCoeffs(C1, C2);
  // 계수 정규화(정규화 conic이라 절대크기가 그리드마다 미세). 상대스케일로 풀어야 함.
  const mx = Math.max(...coef.map(Math.abs)) || 1;
  const roots = solveCubic(coef[0] / mx, coef[1] / mx, coef[2] / mx, coef[3] / mx);
  if (roots.length < 3) return { ok: false, reason: 'roots<3', roots };
  roots.sort((a, b) => a - b);
  // 단일근 = 나머지 둘과 가장 멀리 떨어진 근. 이중근 = 나머지 둘(평균).
  let idx = 0, bestGap = -1;
  for (let i = 0; i < roots.length; i++) {
    const gap = Math.min(...roots.filter((_, j) => j !== i).map(o => Math.abs(o - roots[i])));
    if (gap > bestGap) { bestGap = gap; idx = i; }
  }
  const single = roots[idx];
  const pair = roots.filter((_, j) => j !== idx);
  const lamVan = pair.reduce((s, v) => s + v, 0) / pair.length;

  // 중심 = 단일근 M 의 널공간(최소 |고유값|의 고유벡터). 원근서도 0.02모듈 정밀.
  const Ms = sub3(C1, scale3(C2, single)), es = jacobiEig(Ms, 3);
  let ni = 0; for (let i = 1; i < 3; i++) if (Math.abs(es.val[i]) < Math.abs(es.val[ni])) ni = i;
  const cn = [es.vec[0][ni], es.vec[1][ni], es.vec[2][ni]];
  if (Math.abs(cn[2]) < 1e-12) return { ok: false, reason: 'center-at-inf' };
  const center = [cn[0] / cn[2], cn[1] / cn[2]];

  // 소실선 = 참중심의 (바깥 conic 기준) 극선 polar = C1·center.
  //   원의 중심의 극선은 무한선 → 이미지에선 소실선. 정밀 중심을 그대로 쓰므로
  //   이중근 고유벡터(잡음 민감)보다 견고(각도오차 <2°).
  let l = matVec3(C1, [center[0], center[1], 1]);
  // max-abs 정규화(무한선 근처=저yaw 에서 hypot(l0,l1)→0 로 발산하는 것 방지).
  const ln = Math.max(Math.abs(l[0]), Math.abs(l[1]), Math.abs(l[2])) || 1; l = [l[0] / ln, l[1] / ln, l[2] / ln];

  return { ok: true, l, center, single, lamVan, ringRatio: Math.sqrt(Math.abs(single)) };
}

// ── metric rectify 호모그래피 (이미지→정면, similarity만 잔여) ─────────────
// 입력: 소실선 l, 코어 원뿔 C(원의 이미지), 참중심(이미지) O, 목표스케일/중심.
// 반환: Himg2rect (3×3). 코어 원이 반경 targetR, 중심 targetC 인 원이 되도록.
function rectifyHomography(l, C, O, targetR, targetC) {
  // 1) 사영교정 P: 소실선 → 무한선 [0,0,1].
  const P = [[1, 0, 0], [0, 1, 0], [l[0], l[1], l[2]]];
  // P 하의 코어 원뿔: Ca = P^{-T} C P^{-1}
  const Pinv = inv3(P); if (!Pinv) return null;
  const Ca = mul3(transpose3(Pinv), mul3(C, Pinv));
  // 2) Ca(아핀평면상 타원)의 2×2 이차부 Q → 원으로 만드는 아핀 A.
  const Q = [[Ca[0][0], Ca[0][1]], [Ca[0][1], Ca[1][1]]];
  const eg = jacobiEig(Q, 2);
  // Q 를 정부호로(원뿔 부호 정규화). 두 고유값 동부호여야 타원.
  let e0 = eg.val[0], e1 = eg.val[1];
  if (e0 * e1 <= 0) return null;
  if (e0 < 0) { e0 = -e0; e1 = -e1; }
  // A_shape = D^{1/2} Vᵀ (Q 를 등방화). V=eg.vec (열=고유벡터)
  const V = eg.vec;
  const sq0 = Math.sqrt(e0), sq1 = Math.sqrt(e1);
  const Ashape = [
    [sq0 * V[0][0], sq0 * V[1][0]],
    [sq1 * V[0][1], sq1 * V[1][1]],
  ];
  // 반사 제거: det<0 이면 한 축 뒤집어 방향보존(원은 회전대칭이라 회전만 남기고
  //   반사는 locate 의 시계방향 라벨을 깨므로 반드시 제거).
  if (Ashape[0][0] * Ashape[1][1] - Ashape[0][1] * Ashape[1][0] < 0) { Ashape[1][0] = -Ashape[1][0]; Ashape[1][1] = -Ashape[1][1]; }
  // 아핀 전체 = Ashape 적용한 P (3×3). 아직 중심/스케일/회전 미정.
  const Aaff = [[Ashape[0][0], Ashape[0][1], 0], [Ashape[1][0], Ashape[1][1], 0], [0, 0, 1]];
  let H = mul3(Aaff, P);                       // 이미지→(등방)정면. 코어=원(임의 반경/중심).
  // 3) 정규화: 코어중심 O 가 targetC 로, 코어 반경이 targetR 로.
  const oc = matVec3(H, [O[0], O[1], 1]); const ocx = oc[0] / oc[2], ocy = oc[1] / oc[2];
  // 현재 코어 반경: 원뿔 C 를 H 로 보낸 원의 반경 측정 — 원 위 한 점이 필요.
  // O 에서 소실선 반대방향으로 원의 한 점 대신, C 로부터 반경 추정:
  //   변환후 원뿔 Cr = H^{-T} C H^{-1}; 중심 (ocx,ocy), 반경 = sqrt(-Fc/Ac) (정규원).
  const Hinv = inv3(H); if (!Hinv) return null;
  const Cr = mul3(transpose3(Hinv), mul3(C, Hinv));
  // 정규화 원뿔: A(x²+y²) + D x + E y + F, 중심(-D/2A,-E/2A), 반경²=(D²+E²)/(4A²)-F/A
  const Ac = (Cr[0][0] + Cr[1][1]) / 2;
  let curR = Math.sqrt(Math.max(1e-9, (Cr[0][2] * Cr[0][2] + Cr[1][2] * Cr[1][2]) / (Ac * Ac) - Cr[2][2] / Ac));
  const k = targetR / (curR || 1);
  // 스케일 k + 평행이동으로 중심 맞춤
  const Snorm = [[k, 0, targetC[0] - k * ocx], [0, k, targetC[1] - k * ocy], [0, 0, 1]];
  H = mul3(Snorm, H);
  return H;
}

module.exports = {
  jacobiEig, inv3, adj3, mul3, matVec3, transpose3, frob, scale3, sub3, det3, solveCubic,
  extractCoreRings, fitConic, recoverFromConcentric, rectifyHomography,
};

});

__def("geometry", function(module, exports){
'use strict';
/*
 * ============================================================================
 *  WIA Code v2 — 오르빗(Orbit) 앵커 기하 스펙 + 렌더러
 * ============================================================================
 *  Fable 5 설계 결론 구현. QR의 7×7 파인더를 대체하는 "독자적 localization
 *  primitive"의 기하만(데이터 코덱 제외) 정의·렌더한다. 이 파일 하나로
 *  반나절 검증에 필요한 "정답 앵커 좌표"까지 산출한다.
 *
 *  좌표계: 모듈(module) 단위. 코드 격자는 N×N 모듈(S=64/M=96/L=128).
 *          픽셀 = (module + quiet) × cellPx.
 *
 *  앵커 5개 (검출·정렬의 기준점):
 *    - core     : 정중앙 동심원 불스아이(행성). 링 주기 2모듈(QR 최소특징의 2배).
 *    - sat TL/TR/BR/BL : 네 코너 위성. TL만 도넛("북극성", 흰 중심)→회전 확정.
 *  그 외 포맷 궤도(24 도트)·데이터 도트(성좌 질감)는 시각 충실도 + 검출기
 *  스트레스(클러터)용으로 렌더하되, 앵커가 아니다.
 * ============================================================================
 */

// ── 기하 상수 (모듈 단위) ────────────────────────────────────────────────
const SPEC = {
  // 코어 동심원: [바깥반경, 강도] 경계. 안→밖으로 검/흰/검/흰(세퍼레이터).
  coreRings: [
    { r: 1.5, ink: true },   // 중심 검정 원판 (브랜드 슬롯)
    { r: 3.5, ink: false },  // 흰 링 (폭 2)
    { r: 5.5, ink: true },   // 검정 링 (폭 2)
    { r: 6.5, ink: false },  // 흰 세퍼레이터 (폭 1)
  ],
  coreOuter: 6.5,            // 코어 예약 반경
  satRadius: 2.5,            // 위성 반경(지름 5모듈)
  satInner: 1.0,            // 북극성 도넛 흰 중심 반경
  satInset: 4.5,            // 코너에서 위성 중심까지(=모듈 4.5)
  orbitRadius: 8.5,          // 포맷 궤도 반경
  orbitDots: 24,             // 궤도 도트 수
  orbitDotR: 0.55,           // 궤도 도트 반경
  dataDotR: 0.42,            // 데이터 도트 반경(라운드 채움)
};

const GRID_CELLS = { S: 64, M: 96, L: 128 };

// 스펙 오버라이드 병합 (Fable5 레버 실험용: satRadius/satInset/satInner 등)
function mergeSpec(over) {
  const s = Object.assign({}, SPEC, over || {});
  s.coreRings = SPEC.coreRings; // 코어 링은 고정
  return s;
}

// ── 시드 RNG (재현성) ─────────────────────────────────────────────────────
function makeRng(seed) {
  let s = (seed >>> 0) || 0x9e3779b9;
  return () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 0x100000000; };
}

// ── 앵커/궤도 좌표 산출 ───────────────────────────────────────────────────
//   shape: 'square'(기본, 위성=4코너) | 'round'(위성=링 12/3/6/9시, 데이터판=원판).
//   matcher(locate)는 "코어를 둘러싼 4위성 + 북극성"만 보므로 두 모양 모두 같은 코드로 검출됨.
function layout(grid, spec, shape) {
  spec = spec || SPEC; shape = shape || 'square';
  const N = GRID_CELLS[grid];
  const c = N / 2;                 // 코어 중심(모듈)
  let anchors;
  if (shape !== 'square') {
    const Rs = N / 2 - (spec.satRimMargin || 3.5);   // 위성 링 반경(rim 안쪽) — 원/하트 공용
    const ANG = { TL: -Math.PI / 2, TR: 0, BR: Math.PI / 2, BL: Math.PI };  // 12/3/6/9시(시계방향)
    anchors = [{ name: 'core', type: 'bullseye', mx: c, my: c }];
    for (const k of ['TL', 'TR', 'BR', 'BL'])
      anchors.push({ name: k, type: k === 'TL' ? 'donut' : 'disk', mx: c + Rs * Math.cos(ANG[k]), my: c + Rs * Math.sin(ANG[k]) });
  } else {
    const d = spec.satInset;       // 코너 인셋
    anchors = [
      { name: 'core', type: 'bullseye', mx: c,     my: c     },
      { name: 'TL',   type: 'donut',    mx: d,     my: d     }, // 북극성
      { name: 'TR',   type: 'disk',     mx: N - d, my: d     },
      { name: 'BR',   type: 'disk',     mx: N - d, my: N - d },
      { name: 'BL',   type: 'disk',     mx: d,     my: N - d },
    ];
  }
  // 포맷 궤도 도트 (코어 기준). 상단(-90°)부터 시계방향.
  const orbit = [];
  for (let i = 0; i < spec.orbitDots; i++) {
    const a = -Math.PI / 2 + (2 * Math.PI * i) / spec.orbitDots;
    orbit.push({ mx: c + spec.orbitRadius * Math.cos(a), my: c + spec.orbitRadius * Math.sin(a), on: (i * 7 + 3) % 5 < 3 });
  }
  const L = { N, coreCenter: { mx: c, my: c }, anchors, orbit, spec, shape };
  if (shape !== 'square') {
    L.Rdata = N / 2 - 0.8;         // 데이터 원판 반경(콰이엇 고리 안쪽)
    const TL = anchors[1];
    L.calibPoints = {              // 흑/백 캘리브레이션 기준점(모양 무관하게 명시)
      black: [[c, c]].concat(anchors.filter(a => a.type === 'disk').map(a => [a.mx, a.my])),
      white: [[TL.mx, TL.my], [c + 2.5, c]].concat([45, 135, 225, 315].map(deg => {
        const a = deg * Math.PI / 180, r = N / 2 + 1.5; return [c + r * Math.cos(a), c + r * Math.sin(a)];
      })),
    };
  }
  return L;
}

// ── 모양 마스크: 데이터 판의 실루엣 (검출 원리와 무관 — 앵커는 그대로) ────────
//   'round'=원판, 'heart'=하트, 그 외=전부(사각). 정규화 좌표(중심0, 반경 Rdata=1).
function insideShape(mx, my, L) {
  const c = L.coreCenter.mx, R = L.Rdata || (L.N / 2 - 0.8);
  const nx = (mx - c) / R, ny = (my - c) / R;
  if (L.shape === 'round') return nx * nx + ny * ny <= 1;
  if (L.shape === 'heart') {
    // 고전 하트 곡선 (x²+y²-1)³ - x²y³ ≤ 0. 이미지 y는 아래로 증가 → 뒤집고 스케일.
    const x = nx / 0.95, y = -ny / 0.95 + 0.35;
    const t = x * x + y * y - 1;
    return t * t * t - x * x * y * y * y <= 0;
  }
  return true;
}

// ── 예약 판정 (앵커/궤도 영역엔 데이터 도트 금지) ─────────────────────────
function reservedAt(mx, my, L) {
  const { coreCenter, anchors, orbit, spec } = L; const S = spec || SPEC;
  const dCore = Math.hypot(mx - coreCenter.mx, my - coreCenter.my);
  if (L.shape && L.shape !== 'square' && !insideShape(mx, my, L)) return true;   // 모양 밖 = 데이터 없음
  if (dCore <= S.coreOuter + 0.6) return true;
  if (Math.abs(dCore - S.orbitRadius) <= S.orbitDotR + 0.4) return true;
  for (let i = 1; i < anchors.length; i++) {
    if (Math.hypot(mx - anchors[i].mx, my - anchors[i].my) <= S.satRadius + 0.6) return true;
  }
  return false;
}

// ── 데이터 셀 순서 열거 (예약 밖 셀, 래스터 순서) — 인코드/디코드 공용 계약 ──
//   render(opts.bits) 배치 순서와 반드시 동일해야 한다(디코더가 같은 순서로 읽음).
function dataCells(L) {
  const N = L.N, out = [];
  for (let cy = 0; cy < N; cy++) for (let cx = 0; cx < N; cx++) {
    if (!reservedAt(cx + 0.5, cy + 0.5, L)) out.push([cx, cy]);
  }
  return out;
}

// ── 한 점(모듈좌표)의 잉크 여부: true=검정 ────────────────────────────────
function inkAt(mx, my, L, cellBit) {
  const { coreCenter, anchors, orbit, spec } = L; const S = spec || SPEC;

  // 코어 동심원
  const dCore = Math.hypot(mx - coreCenter.mx, my - coreCenter.my);
  if (dCore <= S.coreOuter) {
    for (const ring of S.coreRings) if (dCore <= ring.r) return ring.ink;
    return false;
  }
  // 포맷 궤도 도트
  if (Math.abs(dCore - S.orbitRadius) <= S.orbitDotR + 0.4) {
    for (const o of orbit) if (o.on && Math.hypot(mx - o.mx, my - o.my) <= S.orbitDotR) return true;
  }
  // 위성
  for (let i = 1; i < anchors.length; i++) {
    const a = anchors[i];
    const ds = Math.hypot(mx - a.mx, my - a.my);
    if (ds <= S.satRadius) {
      if (a.type === 'donut') return ds >= S.satInner; // 도넛: 흰 중심
      return true;                                     // 원판
    }
  }
  // 데이터 도트 (라운드 채움): 예약 밖 + 셀 비트 on
  if (cellBit) {
    const cx = Math.floor(mx) + 0.5, cy = Math.floor(my) + 0.5;
    if (Math.hypot(mx - cx, my - cy) <= S.dataDotR) return true;
  }
  return false;
}

// ── 렌더: 오르빗 코드 이미지(RGBA) + 정답 앵커 픽셀좌표 ──────────────────
//   opts: { grid:'S', cellPx:6, quiet:4, seed:1, ss:2(슈퍼샘플), data:true }
function render(opts) {
  const grid = opts.grid || 'S';
  const cellPx = opts.cellPx || 6;
  const quiet = opts.quiet != null ? opts.quiet : 4;
  const ss = opts.ss || 2;
  const withData = opts.data !== false;
  const spec = mergeSpec(opts.spec);
  const L = layout(grid, spec, opts.shape);
  const N = L.N;

  // 데이터 셀 → "도트 그레이" 맵 cellG (N*N, -1=도트없음). 다단계 지원:
  //   opts.cellGray(데이터셀 순서 0~255 그레이) = 다단계 payload(도트를 그 그레이로 채움),
  //   opts.bits = 1비트(1→검정도트 gray0, 0→도트없음) — 기존과 픽셀 동일,
  //   없으면 랜덤 검정도트(검출 클러터). 앵커(코어·궤도·위성)는 항상 검정(inkAt).
  const cellG = new Int16Array(N * N).fill(-1);
  // 컬러(hue) 레이어: opts.cellChroma[i]=[Cb',Cr'](데이터셀 순서, 0,0=무채색=오늘과 동일).
  //   밝기 Y=cellGray 는 그대로 → toGray 가 색을 무시(루마 레이어 무손상). 색은 Cb/Cr 에만.
  let cellCb = null, cellCr = null;
  if (withData && opts.cellChroma) {
    cellCb = new Int16Array(N * N); cellCr = new Int16Array(N * N);
    const dc = dataCells(L);
    for (let i = 0; i < dc.length; i++) { const c = dc[i], ch = opts.cellChroma[i]; if (ch) { cellCb[c[1] * N + c[0]] = ch[0]; cellCr[c[1] * N + c[0]] = ch[1]; } }
  }
  if (withData) {
    if (opts.cellGray) {
      const dc = dataCells(L);
      for (let i = 0; i < dc.length; i++) { const c = dc[i]; cellG[c[1] * N + c[0]] = opts.cellGray[i]; }
    } else if (opts.bits) {
      const dc = dataCells(L);
      for (let i = 0; i < dc.length; i++) { const c = dc[i]; if (opts.bits[i]) cellG[c[1] * N + c[0]] = 0; }
    } else {
      const rng = makeRng(opts.seed || 1);
      for (let cy = 0; cy < N; cy++) for (let cx = 0; cx < N; cx++) {
        if (!reservedAt(cx + 0.5, cy + 0.5, L) && rng() < 0.5) cellG[cy * N + cx] = 0;
      }
    }
  }
  const dataDotR = spec.dataDotR;

  // Y,Cb,Cr(0..255) → RGB (Rec.601). 컬러셀 렌더용. 무채색(Cb=Cr=128)이면 R=G=B=Y.
  const ycc2rgb = (Y, Cb, Cr) => {
    const cb = Cb - 128, cr = Cr - 128;
    return [Math.max(0, Math.min(255, Y + 1.402 * cr)),
            Math.max(0, Math.min(255, Y - 0.344136 * cb - 0.714136 * cr)),
            Math.max(0, Math.min(255, Y + 1.772 * cb))];
  };
  const dim = (N + 2 * quiet) * cellPx;
  const data = new Uint8ClampedArray(dim * dim * 4).fill(255);
  const inv = 1 / ss, base = (inv - 1) / 2 * inv; // 슈퍼샘플 오프셋
  const ss2 = ss * ss;
  for (let py = 0; py < dim; py++) {
    for (let px = 0; px < dim; px++) {
      let accR = 0, accG = 0, accB = 0;              // 서브샘플 RGB 누적
      for (let sy = 0; sy < ss; sy++) for (let sx = 0; sx < ss; sx++) {
        const fx = px + inv * sx + base, fy = py + inv * sy + base;
        const mx = fx / cellPx - quiet, my = fy / cellPx - quiet;
        if (inkAt(mx, my, L, 0)) continue;           // 앵커 잉크 = 검정(0,0,0)
        let R = 255, G = 255, B = 255;               // 기본 배경 = 흰색
        const cbx = Math.floor(mx), cby = Math.floor(my);
        if (cbx >= 0 && cbx < N && cby >= 0 && cby < N) {
          const idx = cby * N + cbx, gv = cellG[idx];
          if (gv >= 0 && Math.hypot(mx - (cbx + 0.5), my - (cby + 0.5)) <= dataDotR) {
            if (cellCb && (cellCb[idx] || cellCr[idx])) { const c = ycc2rgb(gv, 128 + cellCb[idx], 128 + cellCr[idx]); R = c[0]; G = c[1]; B = c[2]; }
            else { R = G = B = gv; }
          }
        }
        accR += R; accG += G; accB += B;
      }
      const o = (py * dim + px) * 4;
      data[o] = Math.round(accR / ss2); data[o + 1] = Math.round(accG / ss2); data[o + 2] = Math.round(accB / ss2); data[o + 3] = 255;
    }
  }

  // 정답 앵커 픽셀좌표 (모듈중심 → 이미지 픽셀)
  const toPx = (m) => (m + quiet) * cellPx;
  const anchors = L.anchors.map(a => ({
    name: a.name, type: a.type,
    x: toPx(a.mx), y: toPx(a.my),
    mx: a.mx, my: a.my,
  }));

  return { img: { data, width: dim, height: dim }, width: dim, height: dim,
           grid, N, cellPx, quiet, anchors, toPx, spec };
}

module.exports = { SPEC, GRID_CELLS, mergeSpec, layout, render, makeRng, dataCells, reservedAt };

});

__def("locate", function(module, exports){
'use strict';
/*
 * ============================================================================
 *  WIA Code v2 — 오르빗 앵커 정렬(localization) 매처
 * ============================================================================
 *  FRST 극대점 → 5개 앵커(코어 + 위성4) 배정 + 회전 라벨(북극성) + 모듈→이미지
 *  호모그래피. 이 파이프가 성공하면 하류(v1 코덱)는 검증된 셀 샘플링·RS·CRC를
 *  그대로 쓴다. 즉 이 파일이 "QR 파인더 목발"을 대체하는 유일한 신규 지점.
 *
 *  검출기는 이미지 외 정보를 쓰지 않는다(스케일/회전/코너위치 모름). 오직:
 *    1) 코어 = 불스아이 링 프로파일이 가장 뚜렷한 극대점
 *    2) 위성 = 코어 주위 4사분면에 하나씩, 링 프로파일 통과한 강한 극대점
 *    3) 북극성 = 위성 중 중심이 밝은(도넛) 하나 → TL 고정 → 회전 확정
 * ============================================================================
 */

const { radialProfile, refineCentroid } = require('./frst.js');
const { homography, applyH, sample } = require('./degrade.js');
const CN = require('./conic.js');

// 바이리니어 그레이 샘플
function grayAt(G, x, y) {
  const { g, w, h } = G;
  if (x < 0 || y < 0 || x >= w - 1 || y >= h - 1) return 255;
  const x0 = Math.floor(x), y0 = Math.floor(y), fx = x - x0, fy = y - y0, i = y0 * w + x0;
  return (g[i] * (1 - fx) + g[i + 1] * fx) * (1 - fy) + (g[i + w] * (1 - fx) + g[i + w + 1] * fx) * fy;
}

// 프로파일이 "밝은 바깥 배경 + 어두운 중심"인지(위성·코어 공통) + 반전 횟수.
function ringScore(prof) {
  // 정규화
  let mn = Infinity, mx = -Infinity;
  for (const v of prof) { if (v < mn) mn = v; if (v > mx) mx = v; }
  const rng = Math.max(1, mx - mn);
  const norm = prof.map(v => (v - mn) / rng);
  // 중심(첫 20%)은 어두워야, 바깥(마지막 20%)은 밝아야
  const k = norm.length;
  let cin = 0, nin = 0, cout = 0, nout = 0;
  for (let i = 0; i < k; i++) {
    if (i < k * 0.22) { cin += norm[i]; nin++; }
    if (i > k * 0.78) { cout += norm[i]; nout++; }
  }
  const centerDark = (cout / nout) - (cin / nin);   // >0 = 중심 어두움
  // 링 반전 횟수(불스아이일수록 큼)
  let rev = 0; const mid = 0.5;
  let above = norm[0] > mid;
  for (let i = 1; i < k; i++) { const a = norm[i] > mid; if (a !== above) { rev++; above = a; } }
  return { centerDark, rev, contrast: rng };
}

// 코어 후보 반경 추정: 프로파일 첫 상승 에지(어두운 중심 끝) × 경험비 → 모듈px.
// 실패해도 위성 탐색은 기하 일관성(호모그래피 잔차)로 커버하므로 대략치면 충분.
function estimateModulePx(grayObj, cx, cy) {
  const prof = radialProfile(grayObj, cx, cy, 60, 60);
  const rs = ringScore(prof);
  // 코어 바깥 검정 링(반경 ~5.5모듈)까지의 물리 반경을 프로파일 마지막 어두운 골로 추정
  // 간단화: 대칭맵이 이미 중심을 줬으므로 여기선 스케일만 러프 추정
  return { profile: prof, ring: rs };
}

// 두 벡터 각도(코어 기준) 사분면
function quadrant(dx, dy) { // 이미지 좌표(y 아래로 증가)
  if (dx >= 0 && dy < 0) return 0;   // 우상
  if (dx >= 0 && dy >= 0) return 1;  // 우하
  if (dx < 0 && dy >= 0) return 2;   // 좌하
  return 3;                          // 좌상
}

/*
 * locate(grayObj, peaksList, layout) → 결과
 *   layout = geometry.layout(grid) 의 { N, coreCenter, anchors } (모듈 좌표 정답 구조)
 *            ※ 스케일/회전 아님 — "모듈 격자 상의 앵커 배치"라는 포맷 상수일 뿐.
 *   반환: { ok, core:{x,y}, corners:{TL,TR,BR,BL}, northStar, Hmod2img, residPx }
 */
function locate(grayObj, peaksList, layout) {
  if (!peaksList || peaksList.length < 5) return { ok: false, reason: 'peaks<5', found: peaksList ? peaksList.length : 0 };

  // 1+2) 코어 + 위성4를 기하 불변 성질로 동시 선택:
  //   · 코어 후보 h 주위 4사분면에 각각 위성이 하나씩 있어야(원근·회전 불변)
  //   · 코어는 4위성의 무게중심에 위치(dCen≈0) — 코어가 아닌 후보(예: 코너 도넛)는
  //     나머지가 한쪽으로 몰려 사분면이 비거나 dCen이 큼 → 자동 배제.
  //   링 프로파일 기반 코어선별이 원근에서 도넛을 코어로 오인하던 문제를 근본 해결.
  const MINSEP = 12;
  const topH = peaksList.slice(0, 8);
  let core = null, sats = null, bestDCen = Infinity;
  for (const h of topH) {
    // h 를 코어로 가정. 주위 극대점 중 4개가 h 를 "둘러싸는가"를 사분면 비닝 없이
    // 각도 분산으로 판정(회전 불변 — roll 45°에서 사분면 경계 축퇴 문제 제거).
    const cand = peaksList.filter(p => Math.hypot(p.x - h.x, p.y - h.y) > MINSEP)
                          .sort((a, b) => b.score - a.score);
    const pick = [];
    for (const p of cand) { if (pick.some(s => Math.hypot(s.x - p.x, s.y - p.y) < MINSEP)) continue; pick.push(p); if (pick.length === 6) break; }
    if (pick.length < 4) continue;
    // 상위 후보들 중, h 를 가장 잘 둘러싸는(무게중심 근접 + 각도 고른) 4개 선택.
    // 상위 4개를 기본으로 하되 각도 분산 검사.
    const four = pick.slice(0, 4);
    const cx = four.reduce((s, p) => s + p.x, 0) / 4, cy = four.reduce((s, p) => s + p.y, 0) / 4;
    const meanD = four.reduce((s, p) => s + Math.hypot(p.x - h.x, p.y - h.y), 0) / 4;
    const dCen = Math.hypot(h.x - cx, h.y - cy) / (meanD || 1);
    // 각도 최대 간극: 4개가 한쪽에 몰리면 큰 간극 → 둘러싸지 못함
    const angs = four.map(p => Math.atan2(p.y - h.y, p.x - h.x)).sort((a, b) => a - b);
    let maxGap = 2 * Math.PI - (angs[3] - angs[0]);
    for (let i = 1; i < 4; i++) maxGap = Math.max(maxGap, angs[i] - angs[i - 1]);
    if (maxGap > 2.7) continue;                          // >155° 간극 = 둘러싸지 못함
    if (dCen < bestDCen) { bestDCen = dCen; core = { x: h.x, y: h.y }; sats = four.map(p => ({ x: p.x, y: p.y, score: p.score })); }
  }
  if (!core || !sats) return { ok: false, reason: 'no-core-surround' };
  if (bestDCen > 0.35) return { ok: false, reason: 'core-offcenter', dCen: +bestDCen.toFixed(2) };

  const spec = layout.spec || {};
  const satR = spec.satRadius || 2.5;
  const c2c = Math.hypot(layout.coreCenter.mx - layout.anchors[1].mx, layout.coreCenter.my - layout.anchors[1].my);
  const meanSatD = sats.reduce((s, p) => s + Math.hypot(p.x - core.x, p.y - core.y), 0) / 4;
  const satPx = Math.max(3, meanSatD * satR / (c2c || 1));   // 위성 반경(px) 러프추정

  // 3) 위성 중심 정밀화(라벨 무관). FRST 극대점은 도넛의 흰 중심이 아니라 링 위에
  //    찍힐 수 있음 → 먼저 암부 무게중심으로 참 중심(대칭중심=흰 구멍)으로 이동.
  //    원판은 무게중심=원판중심. 이후 밝기로 도넛 식별이 신뢰 가능해진다.
  const satRef = sats.map(s => {
    const r = refineCentroid(grayObj, s.x, s.y, satPx * 1.1);
    return (r.ok && Math.hypot(r.x - s.x, r.y - s.y) < satPx) ? { x: r.x, y: r.y } : { x: s.x, y: s.y };
  });

  // 4) 북극성(도넛): 정밀화된 중심의 그레이가 가장 밝은 위성. 도넛=흰 중심(≈255),
  //    원판=검정 중심(≈0). 정밀화 후엔 far 압축 원판(회색)도 도넛보다 어두워 안정.
  //    이 신호가 4겹 회전대칭을 깨므로 여기서 틀리면 배정이 90° 돈다.
  let donutIdx = 0, brightest = -Infinity;
  for (let i = 0; i < 4; i++) { const v = grayAt(grayObj, satRef[i].x, satRef[i].y); if (v > brightest) { brightest = v; donutIdx = i; } }

  // 5) 각도순(시계방향, 이미지 y-down) 정렬 → 도넛을 TL 에 맞춰 순환 배치 = 회전 확정.
  const orderCW = ['TL', 'TR', 'BR', 'BL'];
  const withAng = satRef.map((s, i) => ({ x: s.x, y: s.y, i, ang: Math.atan2(s.y - core.y, s.x - core.x) }));
  withAng.sort((a, b) => a.ang - b.ang);
  const donutPos = withAng.findIndex(s => s.i === donutIdx);
  const cornR = {};
  for (let i = 0; i < 4; i++) cornR[orderCW[i]] = withAng[(donutPos + i) % 4];

  // 6) 코어 정밀화(중앙 원판만 — 바깥 링은 원근에서 무게중심 편향) + 호모그래피.
  const corePx = Math.max(2, meanSatD * 1.5 / (c2c || 1));
  const cr = refineCentroid(grayObj, core.x, core.y, corePx * 1.1);
  const coreR = (cr.ok && Math.hypot(cr.x - core.x, cr.y - core.y) < corePx) ? { x: cr.x, y: cr.y } : { x: core.x, y: core.y };

  const A = orderCW.map(k => { const a = layout.anchors.find(x => x.name === k); return [a.mx, a.my]; });
  const B = orderCW.map(k => [cornR[k].x, cornR[k].y]);
  A.push([layout.coreCenter.mx, layout.coreCenter.my]); B.push([coreR.x, coreR.y]);
  const Hseed = homographyLS(A, B);
  if (!Hseed) return { ok: false, reason: 'homography-fail' };

  // 7) 2단계 정밀화: seed H의 앵커별 국소 스케일로 창을 재조정. 균일창(1단계)은
  //    도넛 식별엔 충분하나 압축 far 위성엔 과대 → 클러터 혼입. 앵커별 창이 정밀도↑.
  function localMod(mx, my) { const a = applyH(Hseed, mx, my), b = applyH(Hseed, mx + 1, my), c = applyH(Hseed, mx, my + 1);
    return (Math.hypot(b[0] - a[0], b[1] - a[1]) + Math.hypot(c[0] - a[0], c[1] - a[1])) / 2; }
  const satInner = spec.satInner || 1.0;
  function fine(pt, mx, my, rMod, bright) {
    const R = Math.max(2, rMod * localMod(mx, my) * 1.05);
    const r = refineCentroid(grayObj, pt.x, pt.y, R, bright);
    return (r.ok && Math.hypot(r.x - pt.x, r.y - pt.y) < R * 0.6) ? { x: r.x, y: r.y } : { x: pt.x, y: pt.y };
  }
  const coreF = fine(coreR, layout.coreCenter.mx, layout.coreCenter.my, 1.5);
  const cornF = {};
  // TL=도넛: 흰 구멍(solid 밝은 blob)을 밝기중심으로 → 링 암부중심보다 정밀.
  //          창은 구멍 반경(satInner)에 맞춤. 나머지 원판은 암부중심.
  cornF.TL = fine(cornR.TL, layout.anchors.find(a => a.name === 'TL').mx, layout.anchors.find(a => a.name === 'TL').my, satInner * 1.3, true);
  for (const k of ['TR', 'BR', 'BL']) { const a = layout.anchors.find(x => x.name === k); cornF[k] = fine(cornR[k], a.mx, a.my, satR); }

  const A2 = orderCW.map(k => { const a = layout.anchors.find(x => x.name === k); return [a.mx, a.my]; });
  const B2 = orderCW.map(k => [cornF[k].x, cornF[k].y]);
  A2.push([layout.coreCenter.mx, layout.coreCenter.my]); B2.push([coreF.x, coreF.y]);
  const H = homographyLS(A2, B2) || Hseed;
  let resid = 0;
  for (let i = 0; i < A2.length; i++) { const [px, py] = applyH(H, A2[i][0], A2[i][1]); resid += Math.hypot(px - B2[i][0], py - B2[i][1]); }
  resid /= A2.length;

  return { ok: true, core: coreF,
           corners: { TL: cornF.TL, TR: cornF.TR, BR: cornF.BR, BL: cornF.BL },
           northStar: 'TL', donutGray: +brightest.toFixed(0),
           Hmod2img: H, residPx: +resid.toFixed(2), dCen: +bestDCen.toFixed(3) };
}

// 최소자승 호모그래피 (n≥4 대응, DLT 정규방정식). A[i]→B[i].
function homographyLS(A, B) {
  const n = A.length;
  if (n === 4) return homography(A, B);
  // 8미지수 정규방정식 (2n×8)
  const rows = [], rhs = [];
  for (let i = 0; i < n; i++) {
    const [X, Y] = A[i], [x, y] = B[i];
    rows.push([X, Y, 1, 0, 0, 0, -X * x, -Y * x]); rhs.push(x);
    rows.push([0, 0, 0, X, Y, 1, -X * y, -Y * y]); rhs.push(y);
  }
  // 정규방정식 M^T M h = M^T r  (8×8)
  const MtM = Array.from({ length: 8 }, () => new Float64Array(8));
  const Mtr = new Float64Array(8);
  for (let k = 0; k < rows.length; k++) {
    const row = rows[k], rv = rhs[k];
    for (let a = 0; a < 8; a++) { Mtr[a] += row[a] * rv; for (let b = 0; b < 8; b++) MtM[a][b] += row[a] * row[b]; }
  }
  const h = solveN(MtM, Mtr, 8); if (!h) return null;
  return [h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], 1];
}

// 일반 n×n 가우스 소거
function solveN(A, b, n) {
  const M = A.map(r => Array.from(r)), v = Array.from(b);
  for (let i = 0; i < n; i++) {
    let piv = i; for (let r = i + 1; r < n; r++) if (Math.abs(M[r][i]) > Math.abs(M[piv][i])) piv = r;
    if (Math.abs(M[piv][i]) < 1e-12) return null;
    [M[i], M[piv]] = [M[piv], M[i]]; [v[i], v[piv]] = [v[piv], v[i]];
    const d = M[i][i];
    for (let c = i; c < n; c++) M[i][c] /= d; v[i] /= d;
    for (let r = 0; r < n; r++) { if (r === i) continue; const f = M[r][i]; for (let c = i; c < n; c++) M[r][c] -= f * M[i][c]; v[r] -= f * v[i]; }
  }
  return v;
}

// ── 코어 시드 선택 (surround 실패 시): 링 프로파일이 가장 불스아이다운 피크 ──
//   코어는 원근서도 최강 방사대칭 → 상위 피크 중 반전 최다·중심 최암.
function pickCoreSeed(grayObj, peaksList, rProbe) {
  // 코어(불스아이)는 FRST 응답이 압도적 최강(다중 링이 한 점에 중첩투표, 실측 ~10×)
  //   이면서 작은 반경서 다중반전(rev≥2)을 낸다. score×(1+rev) 로 확실히 분리.
  let best = null, bestS = -Infinity;
  for (const p of peaksList.slice(0, 10)) {
    const prof = radialProfile(grayObj, p.x, p.y, rProbe, Math.max(12, Math.round(rProbe)));
    const rs = ringScore(prof);
    const s = (p.score || 1) * (1 + rs.rev) * (rs.centerDark > 0 ? 1 : 0.3);
    if (s > bestS) { bestS = s; best = p; }
  }
  return best;
}

// H(3×3)로 이미지 워프(역샘플). 반환 RGBA {data,width,height}.
function warpImage(img, Himg2rect, dim) {
  const Hinv = CN.inv3(Himg2rect); if (!Hinv) return null;
  const out = { data: new Uint8ClampedArray(dim * dim * 4).fill(255), width: dim, height: dim };
  for (let Y = 0; Y < dim; Y++) for (let X = 0; X < dim; X++) {
    const p = CN.matVec3(Hinv, [X, Y, 1]); const px = sample(img, p[0] / p[2], p[1] / p[2]);
    const o = (Y * dim + X) * 4; out.data[o] = px[0]; out.data[o + 1] = px[1]; out.data[o + 2] = px[2]; out.data[o + 3] = 255;
  }
  return out;
}

/*
 * locateRobust — orbit 매처 우선, 실패(주로 급격 원근의 no-core-surround)시
 *   코어 동심원 conic-pencil 로 정면화(rectify)→재검출→호모그래피 합성.
 *   img      : 원본(열화된) RGBA. 정면화 워프용.
 *   grayObj  : img 의 그레이. peaksList : 그 위 FRST 극대점.
 *   opts.redetect(rectImg) → { gray, peaks } : 정면화 이미지 재검출(하네스 파라미터 재사용).
 *   opts.cellPx : 모듈 픽셀(정면화 목표 스케일).
 *   반환은 locate 와 동일한 필드 + method:'orbit'|'conic'. 좌표는 항상 원본 이미지 프레임.
 */
function locateRobust(img, grayObj, peaksList, layout, opts) {
  opts = opts || {};
  const cellPx = opts.cellPx || 6;
  const primary = locate(grayObj, peaksList, layout);
  const passPx = opts.passPx != null ? opts.passPx : 0.45 * cellPx;
  if (primary.ok && primary.residPx <= passPx) return Object.assign({ method: 'orbit' }, primary);

  // ── conic 폴백 ──
  // 코어 판별 probe 는 작게(코어 내부 링만): 크면 이웃 위성이 프로파일에 섞여
  //   코어의 다중반전 이점이 희석돼 위성을 코어로 오인한다.
  const rProbe = 3.5 * cellPx;
  const seed = pickCoreSeed(grayObj, peaksList, rProbe);
  if (!seed) return primary.ok ? Object.assign({ method: 'orbit' }, primary) : primary;

  const rings = CN.extractCoreRings(grayObj, seed.x, seed.y, 0.18 * Math.min(grayObj.w, grayObj.h), 180);
  const cOut = CN.fitConic(rings.outer), cIn = CN.fitConic(rings.inner);
  if (!cOut || !cIn) return primary;
  const rec = CN.recoverFromConcentric(cOut.C, cIn.C);
  // ringRatio 는 바깥/안쪽 반경비(코어 설계=5.5/1.5≈3.67). 크게 벗어나면 링 오추출 → 폴백 포기.
  if (!rec.ok || rec.ringRatio < 2.6 || rec.ringRatio > 4.6) return primary;

  const targetR = 5.5 * cellPx;
  // 정면화 캔버스 크기 = 코어→가장 먼 앵커 거리(사각·원형 모양 무관).
  const maxDist = Math.max.apply(null, layout.anchors.slice(1).map(a => Math.hypot(a.mx - layout.coreCenter.mx, a.my - layout.coreCenter.my))) * cellPx;
  const dim = 2 * Math.ceil(maxDist + 4 * cellPx);
  const H = CN.rectifyHomography(rec.l, cOut.C, rec.center, targetR, [dim / 2, dim / 2]);
  if (!H) return primary;
  const rimg = warpImage(img, H, dim);
  if (!rimg || !opts.redetect) return primary;
  const rd = opts.redetect(rimg);
  const rres = locate(rd.gray, rd.peaks, layout);
  if (!rres.ok) return primary.ok ? Object.assign({ method: 'orbit' }, primary) : Object.assign({ method: 'conic-fail', reason: 'rect-' + rres.reason }, rres.ok ? {} : { ok: false });

  // 정면좌표 앵커 → 원본 이미지 좌표(Hinv)
  const Hinv = CN.inv3(H);
  const back = (pt) => { const q = CN.matVec3(Hinv, [pt.x, pt.y, 1]); return { x: q[0] / q[2], y: q[1] / q[2] }; };
  const cornersImg = { TL: back(rres.corners.TL), TR: back(rres.corners.TR), BR: back(rres.corners.BR), BL: back(rres.corners.BL) };
  const coreImg = back(rres.core);

  // 원본 프레임 최종 호모그래피 + 재투영오차
  const order = ['TL', 'TR', 'BR', 'BL'];
  const A = order.map(k => { const a = layout.anchors.find(x => x.name === k); return [a.mx, a.my]; });
  const B = order.map(k => [cornersImg[k].x, cornersImg[k].y]);
  A.push([layout.coreCenter.mx, layout.coreCenter.my]); B.push([coreImg.x, coreImg.y]);
  const Hfin = homographyLS(A, B);
  let resid = 0; if (Hfin) { for (let i = 0; i < A.length; i++) { const [px, py] = applyH(Hfin, A[i][0], A[i][1]); resid += Math.hypot(px - B[i][0], py - B[i][1]); } resid /= A.length; }

  return { ok: true, method: 'conic', core: coreImg, corners: cornersImg,
           northStar: rres.northStar, Hmod2img: Hfin, residPx: +resid.toFixed(2),
           conicCenterErr: null, ringRatio: +rec.ringRatio.toFixed(2) };
}

module.exports = { locate, locateRobust, homographyLS, pickCoreSeed };

});

__def("wiacode", function(module, exports){
'use strict';
/*
 * ============================================================================
 *  WIA Code v1 엔진 — 단일 파일 · 의존성 0 · 브라우저 + Node 공용
 * ============================================================================
 *
 *  We're Code (위아코드) — "인터넷 없는 곳의 사람을 살리는 코드"
 *
 *  단일 진실 원천: cqm:/var/www/wiacode/v0-core/CORE_DESIGN_v0.md (WIA3 / v0 스펙)
 *
 *  이 파일은 v0/lib 의 검증된 모듈들(W1-1 인코더 · W1-2 디코더 · W1-3 트로이)을
 *  하나의 무의존 엔진으로 통합한 "차세대 코드"다. 기존 v0 구현의 핵심 단절
 *  ──pngjs + CommonJS require 의존으로 정작 브라우저 PWA 에서 못 돌던 문제──를
 *  제거했다. 같은 알고리즘이 이제 브라우저 <canvas> 와 Node 양쪽에서 동일하게
 *  동작하며, 빌드 스텝/인터넷 없이 한 파일로 인코딩·디코딩 풀 라운드트립이 된다.
 *
 *  v0 대비 전진 (차세대):
 *    - 무의존 통합: PNG/Canvas 추상화(ImageData-like RGBA)로 브라우저·Node 공용
 *    - §5.3 컬러 모드(4단계 그레이, variant 0x01) 구현 ─ v0.1이 미룬 용량 2배 채널
 *    - 디코더가 흑백/컬러 · 4 ECC비율 · 7 오프셋을 자동 전수 시도(CRC가 게이트)
 *    - 셀 매트릭스 ↔ 래스터(ImageData) 분리로 인쇄/카메라/표시 파이프라인 단일화
 *
 *  설계 원칙(스펙 §5.4) 보존:
 *    - 흑백 기본. 컬러는 WiaBookGate / 의료(medical) / 암호화에서 강제 거부.
 *
 *  매직: "WIA3" (0x57 0x49 0x41 0x33) · 16바이트 고정 헤더 · RS GF(256) 0x11D
 * ============================================================================
 */

(function (root, factory) {
  const mod = factory();
  if (typeof module !== 'undefined' && module.exports) module.exports = mod;     // Node / CommonJS
  if (typeof window !== 'undefined') window.WIACode = mod;                       // 브라우저 글로벌
  if (typeof globalThis !== 'undefined') globalThis.WIACode = globalThis.WIACode || mod;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {

  // ──────────────────────────────────────────────────────────────────────
  // 0. 상수 (스펙 §1.5, §3, §4, §5)
  // ──────────────────────────────────────────────────────────────────────
  const MAGIC = [0x57, 0x49, 0x41, 0x33];          // "WIA3"
  const VERSION_FLAT = 0x00;                       // v0 코어 (블록 순차 직렬화)
  const VERSION = 0x01;                            // v0 + 코드워드 인터리브 (§6.1.3) — 인코더 기본
  const HEADER_LEN = 16;
  const CELL = 12;                                 // px / 셀 (300dpi ≈ 1mm)

  const MODES = { static: 0x01, dynamic: 0x02, wiabook: 0x03 };
  const MODE_NAMES = { 0x01: 'static', 0x02: 'dynamic', 0x03: 'wiabook' };

  // §4.2 / §4.9 — S/M/L 그리드 + 핀더 간 모듈 거리(자동 판별용)
  const GRIDS = {
    S: { name: 'S', cells: 64,  finderModuleDistance: 57,  canvas: 64  * CELL },
    M: { name: 'M', cells: 96,  finderModuleDistance: 89,  canvas: 96  * CELL },
    L: { name: 'L', cells: 128, finderModuleDistance: 121, canvas: 128 * CELL },
  };

  const FINDER_SIZE = 7;       // 7×7 모듈
  const FINDER_RESERVED = 8;   // 7×7 + 1 separator
  const ALIGNMENT_SIZE = 5;    // L 전용 5×5

  // §3.2 Flags bit6-7 → ECC 비율
  const ECC_BITS = { '25%': 0b00, '30%': 0b01, '35%': 0b10, '50%': 0b11 };
  const ECC_FROM_BITS = ['25%', '30%', '35%', '50%'];
  const ECC_RATIOS = { '25%': 0.25, '30%': 0.30, '35%': 0.35, '50%': 0.50 };

  const DEFAULT_ECC = { static: '25%', dynamic: '25%', wiabook: '35%' };

  // §5.3 컬러 모드 — variant 0x01: 4단계 그레이(2비트/셀). Reserved 바이트에 기록.
  const COLOR_GRAY4 = 0x01;
  const CHANNEL_LEVELS = [40, 120, 190, 240];      // 인덱스 0..3 → 픽셀 강도
  // 분류 경계(중간값): 80 / 155 / 215

  const STATIC_DATA_TYPES = { text: 0x01, vcard: 0x02, wifi: 0x03, medical: 0x10 };
  const STATIC_DATA_NAMES = { 0x01: 'text', 0x02: 'vcard', 0x03: 'wifi', 0x10: 'medical' };
  const STATIC_ENCODINGS  = { 'utf-8': 0x00, base64: 0x01, binary: 0x02 };
  const STATIC_ENC_NAMES  = { 0x00: 'utf-8', 0x01: 'base64', 0x02: 'binary' };
  const HOST_HINT = { none: 0x00, url: 0x01, local: 0x02 };

  const OFFSETS_7 = [[0,0],[1,0],[-1,0],[0,1],[0,-1],[1,1],[-1,-1]]; // 서브픽셀 보정

  // ──────────────────────────────────────────────────────────────────────
  // 1. 텍스트 인코딩 헬퍼 (TextEncoder/Decoder 는 Node18+/모든 브라우저 내장)
  // ──────────────────────────────────────────────────────────────────────
  const _enc = new TextEncoder();
  const _dec = new TextDecoder('utf-8');
  const utf8 = (s) => _enc.encode(s);
  const fromUtf8 = (b) => _dec.decode(b);

  function b64decode(str) {
    if (typeof atob === 'function') {
      const bin = atob(str); const out = new Uint8Array(bin.length);
      for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
      return out;
    }
    return Uint8Array.from(Buffer.from(str, 'base64'));
  }
  function b64encode(bytes) {
    if (typeof btoa === 'function') {
      let bin = ''; for (let i = 0; i < bytes.length; i++) bin += String.fromCharCode(bytes[i]);
      return btoa(bin);
    }
    return Buffer.from(bytes).toString('base64');
  }
  function toHex(b) { return Array.from(b).map(x => x.toString(16).padStart(2, '0')).join(''); }

  // ──────────────────────────────────────────────────────────────────────
  // 2. CRC (crc.js 포팅) — CRC16/CCITT-FALSE, CRC32/IEEE
  // ──────────────────────────────────────────────────────────────────────
  function crc32(buf) {
    let c = 0xFFFFFFFF;
    for (let i = 0; i < buf.length; i++) {
      c ^= buf[i];
      for (let j = 0; j < 8; j++) c = (c >>> 1) ^ ((c & 1) ? 0xEDB88320 : 0);
    }
    return (c ^ 0xFFFFFFFF) >>> 0;
  }
  function crc16(buf) {
    let crc = 0xFFFF;
    for (let i = 0; i < buf.length; i++) {
      crc ^= (buf[i] << 8);
      for (let j = 0; j < 8; j++) crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) & 0xFFFF : (crc << 1) & 0xFFFF;
    }
    return crc & 0xFFFF;
  }

  // ──────────────────────────────────────────────────────────────────────
  // 3. Reed-Solomon GF(256), 0x11D, α=0x02 (decode-ecc.js 포팅: encode+decode)
  // ──────────────────────────────────────────────────────────────────────
  const GF_EXP = new Uint8Array(512);
  const GF_LOG = new Uint8Array(256);
  (function initGF() {
    let x = 1;
    for (let i = 0; i < 255; i++) { GF_EXP[i] = x; GF_LOG[x] = i; x <<= 1; if (x & 0x100) x ^= 0x11D; }
    for (let i = 255; i < 512; i++) GF_EXP[i] = GF_EXP[i - 255];
  })();
  const gfMul = (a, b) => (a === 0 || b === 0) ? 0 : GF_EXP[GF_LOG[a] + GF_LOG[b]];
  const gfDiv = (a, b) => { if (b === 0) throw new Error('GF div0'); return a === 0 ? 0 : GF_EXP[(GF_LOG[a] + 255 - GF_LOG[b]) % 255]; };
  const gfInv = (a) => { if (a === 0) throw new Error('GF inv0'); return GF_EXP[255 - GF_LOG[a]]; };

  function polyMul(p, q) {
    const r = new Uint8Array(p.length + q.length - 1);
    for (let i = 0; i < p.length; i++) { if (!p[i]) continue; for (let j = 0; j < q.length; j++) r[i + j] ^= gfMul(p[i], q[j]); }
    return r;
  }
  function polyScale(p, x) { const r = new Uint8Array(p.length); for (let i = 0; i < p.length; i++) r[i] = gfMul(p[i], x); return r; }
  function polyAdd(p, q) {
    const r = new Uint8Array(Math.max(p.length, q.length));
    for (let i = 0; i < p.length; i++) r[i + r.length - p.length] = p[i];
    for (let i = 0; i < q.length; i++) r[i + r.length - q.length] ^= q[i];
    return r;
  }
  function polyEval(p, x) { let y = p[0]; for (let i = 1; i < p.length; i++) y = gfMul(y, x) ^ p[i]; return y; }
  function rsGenPoly(nsym) { let g = Uint8Array.of(1); for (let i = 0; i < nsym; i++) g = polyMul(g, Uint8Array.of(1, GF_EXP[i])); return g; }

  function rsEncodeBlock(data, nsym) {
    if (nsym <= 0) return Uint8Array.from(data);
    const gen = rsGenPoly(nsym);
    const buf = new Uint8Array(data.length + nsym);
    buf.set(data, 0);
    for (let i = 0; i < data.length; i++) {
      const coef = buf[i];
      if (coef !== 0) for (let j = 0; j < gen.length; j++) buf[i + j] ^= gfMul(gen[j], coef);
    }
    return buf.subarray(data.length); // parity
  }

  function calcSynd(msg, nsym) { const s = new Uint8Array(nsym + 1); for (let i = 0; i < nsym; i++) s[i + 1] = polyEval(msg, GF_EXP[i]); return s; }
  function syndNonZero(s) { for (let i = 1; i < s.length; i++) if (s[i]) return true; return false; }

  function findErrLocator(synd, nsym) {
    let errLoc = Uint8Array.of(1), oldLoc = Uint8Array.of(1);
    const shift = synd.length - 1 - nsym;
    for (let i = 0; i < nsym; i++) {
      const K = i + shift;
      let delta = synd[K + 1];
      for (let j = 1; j < errLoc.length; j++) delta ^= gfMul(errLoc[errLoc.length - 1 - j], synd[K + 1 - j]);
      oldLoc = Uint8Array.from([...oldLoc, 0]);
      if (delta !== 0) {
        if (oldLoc.length > errLoc.length) { const nl = polyScale(oldLoc, delta); oldLoc = polyScale(errLoc, gfInv(delta)); errLoc = nl; }
        errLoc = polyAdd(errLoc, polyScale(oldLoc, delta));
      }
    }
    let start = 0; while (start < errLoc.length - 1 && errLoc[start] === 0) start++;
    errLoc = errLoc.subarray(start);
    const errs = errLoc.length - 1;
    if (errs * 2 > nsym) throw new Error('too many errors (' + errs + ' > ' + Math.floor(nsym / 2) + ')');
    return errLoc;
  }
  function findErrPositions(errLoc, msgLen) {
    const errs = errLoc.length - 1, positions = [];
    for (let i = 0; i < msgLen; i++) if (polyEval(errLoc, GF_EXP[(255 - i) % 255]) === 0) positions.push(msgLen - 1 - i);
    if (positions.length !== errs) throw new Error('err pos mismatch');
    return positions;
  }
  function correctErrors(msg, synd, positions) {
    const msgLen = msg.length;
    const coefPos = positions.map(p => msgLen - 1 - p);
    let lam = Uint8Array.of(1);
    for (const c of coefPos) lam = polyMul(lam, Uint8Array.of(GF_EXP[c], 1));
    const sc = new Uint8Array(synd.length - 1);
    for (let i = 0; i < sc.length; i++) sc[i] = synd[synd.length - 1 - i];
    const omegaFull = polyMul(sc, lam);
    const omega = omegaFull.subarray(omegaFull.length - sc.length);
    const prime = []; const deg = lam.length - 1;
    for (let i = 0; i < deg; i++) { const k = deg - i; prime.push(k % 2 === 1 ? lam[i] : 0); }
    const lamP = Uint8Array.from(prime);
    for (let i = 0; i < positions.length; i++) {
      const Xi = GF_EXP[coefPos[i]], XiInv = gfInv(Xi);
      const num = polyEval(omega, XiInv), den = polyEval(lamP, XiInv);
      if (den === 0) throw new Error('Forney div0');
      msg[positions[i]] ^= gfMul(Xi, gfDiv(num, den));
    }
  }
  function rsDecodeBlock(codeword, nsym) {
    const msg = new Uint8Array(codeword);
    const synd = calcSynd(msg, nsym);
    if (!syndNonZero(synd)) return { data: msg.subarray(0, msg.length - nsym), errors: 0 };
    const errLoc = findErrLocator(synd, nsym);
    const positions = findErrPositions(errLoc, msg.length);
    correctErrors(msg, synd, positions);
    if (syndNonZero(calcSynd(msg, nsym))) throw new Error('post-correction syndromes nonzero');
    return { data: msg.subarray(0, msg.length - nsym), errors: positions.length };
  }

  // 블록 분할 (ecc.js / decode-ecc.js 동일 알고리즘) — 인코더·디코더 정합 ground truth
  function planBlocks(rawBytes, ratioStr) {
    const ratio = ECC_RATIOS[ratioStr];
    if (ratio === undefined) throw new Error('Unknown ECC ratio: ' + ratioStr);
    const b = Math.max(1, Math.ceil(rawBytes / 255));
    const ceilN = Math.ceil(rawBytes / b), floorN = Math.floor(rawBytes / b);
    const ceilCount = rawBytes - floorN * b;
    const blocks = []; let totalK = 0, totalN = 0;
    for (let i = 0; i < b; i++) {
      const n = i < ceilCount ? ceilN : floorN;
      const nsym = Math.round(n * ratio), k = n - nsym;
      if (k <= 0) throw new Error('Block too small for ECC ' + ratioStr);
      blocks.push({ n, k, nsym }); totalK += k; totalN += n;
    }
    return { blocks, totalK, totalN };
  }
  function rsEncodeAll(data, plan) {
    if (data.length !== plan.totalK) throw new Error('encodeAll len ' + data.length + ' != totalK ' + plan.totalK);
    const dataParts = [], parityParts = []; let off = 0;
    for (const blk of plan.blocks) {
      const slice = data.subarray(off, off + blk.k); off += blk.k;
      dataParts.push(slice); parityParts.push(rsEncodeBlock(slice, blk.nsym));
    }
    const out = new Uint8Array(plan.totalN); let p = 0;
    for (const part of dataParts) { out.set(part, p); p += part.length; }
    for (const part of parityParts) { out.set(part, p); p += part.length; }
    return out;
  }
  // §6.1.3 — Version 0x01 코드워드 인터리브. 블록 순차(0x00) ↔ 라운드로빈 순서 맵.
  // order[j] = 인터리브 스트림의 j번째 바이트가 블록순차 스트림에서 갖는 위치.
  function interleaveOrder(plan) {
    const order = [];
    const dataStart = [], parStart = []; let off = 0;
    for (const b of plan.blocks) { dataStart.push(off); off += b.k; }
    for (const b of plan.blocks) { parStart.push(off); off += b.nsym; }
    let maxK = 0, maxS = 0;
    for (const b of plan.blocks) { if (b.k > maxK) maxK = b.k; if (b.nsym > maxS) maxS = b.nsym; }
    for (let i = 0; i < maxK; i++) for (let k = 0; k < plan.blocks.length; k++) if (i < plan.blocks[k].k) order.push(dataStart[k] + i);
    for (let i = 0; i < maxS; i++) for (let k = 0; k < plan.blocks.length; k++) if (i < plan.blocks[k].nsym) order.push(parStart[k] + i);
    return order;
  }
  function interleaveBytes(seq, plan) {
    const order = interleaveOrder(plan), out = new Uint8Array(seq.length);
    for (let j = 0; j < order.length; j++) out[j] = seq[order[j]];
    return out;
  }
  function deinterleaveBytes(raw, plan) {
    const order = interleaveOrder(plan), out = new Uint8Array(raw.length);
    for (let j = 0; j < order.length; j++) out[order[j]] = raw[j];
    return out;
  }
  function rsDecodeAll(codeword, plan) {
    if (codeword.length < plan.totalN) throw new Error('codeword too short');
    const dataParts = [], parityParts = []; let off = 0;
    for (const blk of plan.blocks) { dataParts.push(codeword.subarray(off, off + blk.k)); off += blk.k; }
    for (const blk of plan.blocks) { parityParts.push(codeword.subarray(off, off + blk.nsym)); off += blk.nsym; }
    const out = new Uint8Array(plan.totalK); let outOff = 0, totalErrors = 0;
    for (let i = 0; i < plan.blocks.length; i++) {
      const blk = plan.blocks[i];
      const cw = new Uint8Array(blk.k + blk.nsym);
      cw.set(dataParts[i], 0); cw.set(parityParts[i], blk.k);
      const r = rsDecodeBlock(cw, blk.nsym);
      out.set(r.data, outOff); outOff += blk.k; totalErrors += r.errors;
    }
    return { data: out, errors: totalErrors };
  }

  // ──────────────────────────────────────────────────────────────────────
  // 4. 그리드 기하 (grid.js + cell-layout.js 통합) — 행우선 [row,col]
  // ──────────────────────────────────────────────────────────────────────
  function gridDef(grade) { const d = GRIDS[grade]; if (!d) throw new Error("grid must be 'S'|'M'|'L', got " + grade); return d; }

  function isFinderCell(grade, r, c) {
    const G = gridDef(grade).cells, F = FINDER_RESERVED;
    if (r < F && c < F) return true;            // TL
    if (r < F && c >= G - F) return true;       // TR
    if (r >= G - F && c < F) return true;       // BL
    return false;
  }
  function isAlignmentCell(grade, r, c) {
    const G = gridDef(grade).cells;
    if (G !== 128) return false;
    return r >= G - 9 && r <= G - 5 && c >= G - 9 && c <= G - 5; // 5×5 우하 안쪽
  }
  function isReserved(grade, r, c) { return isFinderCell(grade, r, c) || isAlignmentCell(grade, r, c); }

  function dataCellOrder(grade) {
    const G = gridDef(grade).cells, cells = [];
    for (let r = 0; r < G; r++) for (let c = 0; c < G; c++) if (!isReserved(grade, r, c)) cells.push([r, c]);
    return cells;
  }
  function dataCellCount(grade) { return dataCellOrder(grade).length; }

  // 핀더 패턴 + separator (검정=픽셀0, 흰=픽셀255) — 컬러 모드에서도 핀더는 순흑백 유지
  function drawFinders(levels, grade) {
    const G = gridDef(grade).cells;
    const pos = [{ r0: 0, c0: 0 }, { r0: 0, c0: G - FINDER_SIZE }, { r0: G - FINDER_SIZE, c0: 0 }];
    for (const p of pos) {
      for (let dr = 0; dr < FINDER_SIZE; dr++) for (let dc = 0; dc < FINDER_SIZE; dc++) {
        let black = false;
        if (dr === 0 || dr === 6 || dc === 0 || dc === 6) black = true;
        else if (dr >= 2 && dr <= 4 && dc >= 2 && dc <= 4) black = true;
        levels[(p.r0 + dr) * G + (p.c0 + dc)] = black ? 0 : 255;
      }
    }
    for (let i = 0; i < FINDER_RESERVED; i++) {        // separator(흰)
      levels[7 * G + i] = 255; levels[i * G + 7] = 255;
      levels[7 * G + (G - FINDER_RESERVED + i)] = 255; levels[i * G + (G - FINDER_RESERVED)] = 255;
      levels[(G - FINDER_RESERVED) * G + i] = 255; levels[(G - FINDER_RESERVED + i) * G + 7] = 255;
    }
  }
  function drawAlignment(levels, grade) {
    const G = gridDef(grade).cells;
    if (G !== 128) return;
    const r0 = G - 9, r1 = G - 5, c0 = G - 9, c1 = G - 5, mid = (r0 + r1) / 2;
    for (let r = r0; r <= r1; r++) for (let c = c0; c <= c1; c++) {
      const edge = (r === r0 || r === r1 || c === c0 || c === c1);
      const center = (r === mid && c === mid);
      levels[r * G + c] = (edge || center) ? 0 : 255;
    }
  }

  // ──────────────────────────────────────────────────────────────────────
  // 5. 헤더 (header.js / decode-header.js 포팅)
  // ──────────────────────────────────────────────────────────────────────
  function buildFlags(flags, eccStr, colorOn) {
    let f = 0;
    if (flags.trojan) f |= 1 << 0;
    if (flags.braille) f |= 1 << 1;
    if (flags.distributed) f |= 1 << 2;
    if (colorOn) f |= 1 << 3;
    if (flags.compressed) f |= 1 << 4;
    if (flags.encrypted) f |= 1 << 5;
    f |= (ECC_BITS[eccStr] & 0b11) << 6;
    return f & 0xFF;
  }
  function serializeHeader(mode, flags, eccStr, payload, colorVariant, version) {
    const colorOn = !!colorVariant;
    const h = new Uint8Array(HEADER_LEN);
    h[0] = MAGIC[0]; h[1] = MAGIC[1]; h[2] = MAGIC[2]; h[3] = MAGIC[3];
    h[4] = version === undefined ? VERSION : version;
    h[5] = MODES[mode];
    h[6] = buildFlags(flags, eccStr, colorOn);
    h[7] = colorOn ? colorVariant : 0x00;             // §5.3 Reserved = 컬러 variant
    h[8] = (payload.length >> 8) & 0xFF; h[9] = payload.length & 0xFF;
    const pc = crc32(payload);
    h[10] = (pc >>> 24) & 0xFF; h[11] = (pc >>> 16) & 0xFF; h[12] = (pc >>> 8) & 0xFF; h[13] = pc & 0xFF;
    const hc = crc16(h.subarray(0, 14));
    h[14] = (hc >>> 8) & 0xFF; h[15] = hc & 0xFF;
    return { header: h, headerCrc16: hc, payloadCrc32: pc, flagsByte: h[6] };
  }
  function parseHeader(buf) {
    if (!buf || buf.length < HEADER_LEN) return { ok: false, error: 'header too short' };
    if (!(buf[0] === 0x57 && buf[1] === 0x49 && buf[2] === 0x41)) return { ok: false, error: 'magic mismatch (not WIA)' };
    if (buf[3] !== 0x33) {
      if (buf[3] === 0x31 || buf[3] === 0x32) return { ok: false, error: 'WIA1/WIA2 deprecated — v0 rejects' };
      return { ok: false, error: 'magic mismatch (not WIA3)' };
    }
    const version = buf[4], mode = buf[5], flagsB = buf[6], reserved = buf[7];
    const payloadLen = (buf[8] << 8) | buf[9];
    const payloadCrc32 = ((buf[10] << 24) | (buf[11] << 16) | (buf[12] << 8) | buf[13]) >>> 0;
    const headerCrc16 = (buf[14] << 8) | buf[15];
    if (crc16(buf.subarray(0, 14)) !== headerCrc16) return { ok: false, error: 'header CRC16 mismatch' };
    if (version !== VERSION_FLAT && version !== VERSION) return { ok: false, error: 'unsupported version' };
    if (!MODE_NAMES[mode]) return { ok: false, error: 'unknown mode 0x' + mode.toString(16) };
    const flags = {
      trojan: !!(flagsB & 0x01), braille: !!(flagsB & 0x02), distributed: !!(flagsB & 0x04),
      color: !!(flagsB & 0x08), compressed: !!(flagsB & 0x10), encrypted: !!(flagsB & 0x20),
      eccBits: (flagsB >> 6) & 0x03, raw: flagsB,
    };
    return {
      ok: true,
      header: {
        magic: 'WIA3', version, mode, modeName: MODE_NAMES[mode], flags, reserved,
        colorVariant: flags.color ? reserved : 0,
        payloadLen, payloadCrc32, headerCrc16, eccRatio: ECC_FROM_BITS[flags.eccBits],
      },
    };
  }

  // ──────────────────────────────────────────────────────────────────────
  // 6. 페이로드 직렬화/파싱 (payload-*.js / decode-payload-*.js 포팅)
  // ──────────────────────────────────────────────────────────────────────
  function serializeStatic(p) {
    if (!p || typeof p !== 'object') throw new Error('static payload must be object');
    let typeByte = typeof p.data_type === 'number' ? (p.data_type & 0xFF) : STATIC_DATA_TYPES[p.data_type];
    if (typeByte === undefined) throw new Error('unknown static data_type: ' + p.data_type);
    const encName = p.encoding || 'utf-8';
    let encByte = typeof encName === 'number' ? (encName & 0xFF) : STATIC_ENCODINGS[encName];
    if (encByte === undefined) throw new Error('unknown static encoding: ' + encName);
    let dataBytes;
    if (p.data instanceof Uint8Array) dataBytes = p.data;
    else if (typeof p.data === 'string') dataBytes = encByte === STATIC_ENCODINGS.base64 ? b64decode(p.data) : utf8(p.data);
    else throw new Error('static data must be string|Uint8Array');
    const out = new Uint8Array(2 + dataBytes.length);
    out[0] = typeByte; out[1] = encByte; out.set(dataBytes, 2);
    return out;
  }
  function parseStatic(payload) {
    if (payload.length < 2) return { ok: false, error: 'static payload too short' };
    const tId = payload[0], eId = payload[1], data = payload.subarray(2);
    let decoded;
    if (eId === 0x00) decoded = fromUtf8(data);
    else if (eId === 0x01) decoded = b64encode(data);
    else decoded = data;
    return { ok: true, data: {
      data_type: STATIC_DATA_NAMES[tId] || ('type_0x' + tId.toString(16)), data_type_id: tId,
      encoding: STATIC_ENC_NAMES[eId] || ('enc_0x' + eId.toString(16)), encoding_id: eId,
      data: decoded, raw: Uint8Array.from(data),
    }};
  }
  function serializeDynamic(p) {
    if (!p || typeof p.url !== 'string' || !p.url) throw new Error('dynamic payload.url required');
    const u = utf8(p.url);
    if (u.length > 255) throw new Error('dynamic URL too long (' + u.length + ' > 255)');
    const out = new Uint8Array(1 + u.length); out[0] = u.length; out.set(u, 1); return out;
  }
  function parseDynamic(payload) {
    if (payload.length < 1) return { ok: false, error: 'dynamic too short' };
    const len = payload[0];
    if (payload.length < 1 + len) return { ok: false, error: 'dynamic url len mismatch' };
    return { ok: true, data: { url: fromUtf8(payload.subarray(1, 1 + len)), url_length: len } };
  }
  function _uuidBytes(s) {
    if (s instanceof Uint8Array) { if (s.length !== 16) throw new Error('uuid 16B'); return s; }
    const hex = String(s).replace(/-/g, '').toLowerCase();
    if (!/^[0-9a-f]{32}$/.test(hex)) throw new Error('invalid UUID: ' + s);
    const o = new Uint8Array(16); for (let i = 0; i < 16; i++) o[i] = parseInt(hex.substr(i * 2, 2), 16); return o;
  }
  function _hash8Bytes(s) {
    if (s instanceof Uint8Array) { if (s.length !== 8) throw new Error('hash 8B'); return s; }
    const hex = String(s).toLowerCase();
    if (!/^[0-9a-f]{16}$/.test(hex)) throw new Error('hash_sha256_8b must be 16 hex');
    const o = new Uint8Array(8); for (let i = 0; i < 8; i++) o[i] = parseInt(hex.substr(i * 2, 2), 16); return o;
  }
  function serializeWiaBook(p) {
    if (!p || typeof p !== 'object') throw new Error('wiabook payload must be object');
    if (!Number.isInteger(p.chapter_index) || p.chapter_index < 0 || p.chapter_index > 0xFFFF)
      throw new Error('chapter_index must be 0..65535');
    const uuid = _uuidBytes(p.uuid), hash = _hash8Bytes(p.hash_sha256_8b);
    let hintType = HOST_HINT.none, hintBytes = new Uint8Array(0);
    if (p.host_hint && p.host_hint.type && p.host_hint.type !== 'none') {
      hintType = p.host_hint.type === 'url' ? HOST_HINT.url : p.host_hint.type === 'local' ? HOST_HINT.local : null;
      if (hintType === null) throw new Error('unknown host_hint.type');
      if (typeof p.host_hint.value !== 'string' || !p.host_hint.value) throw new Error('host_hint.value required');
      hintBytes = utf8(p.host_hint.value);
      if (hintBytes.length > 255) throw new Error('host_hint too long');
    }
    const out = new Uint8Array(28 + hintBytes.length);
    out.set(uuid, 0);
    out[16] = (p.chapter_index >> 8) & 0xFF; out[17] = p.chapter_index & 0xFF;
    out.set(hash, 18); out[26] = hintType; out[27] = hintBytes.length;
    if (hintBytes.length) out.set(hintBytes, 28);
    return out;
  }
  function parseWiaBook(payload) {
    if (payload.length < 28) return { ok: false, error: 'wiabook too short' };
    const h = toHex(payload.subarray(0, 16));
    const uuid = `${h.slice(0,8)}-${h.slice(8,12)}-${h.slice(12,16)}-${h.slice(16,20)}-${h.slice(20,32)}`;
    const chapter = (payload[16] << 8) | payload[17];
    const hash = toHex(payload.subarray(18, 26));
    const hintType = payload[26], hintLen = payload[27];
    let hostHint = null;
    if (hintType !== HOST_HINT.none) {
      if (payload.length < 28 + hintLen) return { ok: false, error: 'host_hint truncated' };
      hostHint = { type: hintType === HOST_HINT.url ? 'url' : 'local', value: fromUtf8(payload.subarray(28, 28 + hintLen)) };
    }
    return { ok: true, data: { uuid, chapter_index: chapter, hash_sha256_8b: hash, host_hint: hostHint } };
  }

  // ──────────────────────────────────────────────────────────────────────
  // 7. 인코더 — {mode,grid,payload,flags} → 셀 매트릭스(levels) + meta
  //    bitsPerCell: 흑백=1, 컬러(gray4)=2
  // ──────────────────────────────────────────────────────────────────────
  function encode(input) {
    const t0 = Date.now();
    const flags = Object.assign({ trojan: false, braille: false, distributed: false, compressed: false, encrypted: false }, input.flags || {});
    const wantColor = !!(input.flags && input.flags.color);

    if (!MODES[input.mode]) throw new Error("mode must be 'static'|'dynamic'|'wiabook'");
    gridDef(input.grid);

    // §5.4 컬러 금지 룰
    let colorVariant = 0;
    if (wantColor) {
      if (input.mode === 'wiabook') throw new Error('§5.4: WiaBookGate 는 컬러 금지');
      if (input.mode === 'static' && isMedical(input.payload)) throw new Error('§5.4: 의료(medical) 데이터는 컬러 금지');
      if (flags.encrypted) throw new Error('§5.4: 암호화 페이로드는 컬러 금지');
      colorVariant = COLOR_GRAY4;
    }
    const bitsPerCell = colorVariant ? 2 : 1;

    // 트로이 호환 (§7) — M/L 그리드 강제, 컬러 불가, ECC 그리드별 최소값 상향
    const wantTrojan = !!flags.trojan;
    if (wantTrojan) {
      if (input.grid === 'S') throw new Error('§4.7: 트로이는 M 또는 L 그리드 필요 (가운데 QR 면적)');
      if (colorVariant) throw new Error('트로이는 흑백 전용 (가운데 QR = QR 스캐너 호환)');
    }
    const TROJAN_MIN_ECC = { M: '50%', L: '35%' };

    // ECC 비율 결정
    let eccStr = (input.flags && input.flags.ecc) || DEFAULT_ECC[input.mode];
    if (!(input.flags && input.flags.ecc) && input.mode === 'static' && isMedical(input.payload)) eccStr = '50%';
    if (wantTrojan) {
      const minE = TROJAN_MIN_ECC[input.grid];
      if (ECC_FROM_BITS.indexOf(eccStr) < ECC_FROM_BITS.indexOf(minE)) eccStr = minE;  // overlay corruption 흡수
    }
    if (ECC_BITS[eccStr] === undefined) throw new Error('Invalid ECC ratio ' + eccStr);

    // 페이로드 + 헤더
    let payload;
    if (input.mode === 'static') payload = serializeStatic(input.payload);
    else if (input.mode === 'dynamic') payload = serializeDynamic(input.payload);
    else payload = serializeWiaBook(input.payload);
    // §6.1.3 직렬화 버전 — 기본 0x01(인터리브), flags.version 명시 시 0x00 허용
    const fmtVersion = (input.flags && input.flags.version !== undefined) ? input.flags.version : VERSION;
    if (fmtVersion !== VERSION_FLAT && fmtVersion !== VERSION) throw new Error('Invalid format version 0x' + fmtVersion.toString(16));
    const { header, headerCrc16, payloadCrc32, flagsByte } = serializeHeader(input.mode, flags, eccStr, payload, colorVariant, fmtVersion);

    const hpp = new Uint8Array(header.length + payload.length);
    hpp.set(header, 0); hpp.set(payload, header.length);

    // 용량 + RS 계획
    const cells = gridDef(input.grid).cells;
    const order = dataCellOrder(input.grid);
    const rawBytes = Math.floor((order.length * bitsPerCell) / 8);
    const plan = planBlocks(rawBytes, eccStr);
    if (hpp.length > plan.totalK) {
      throw new Error(`Data too large for grid ${input.grid} ${colorVariant ? '(color) ' : ''}ECC ${eccStr}: ` +
                      `${hpp.length}B > capacity ${plan.totalK}B`);
    }

    const dataIn = new Uint8Array(plan.totalK); dataIn.set(hpp, 0);
    let codewords = rsEncodeAll(dataIn, plan);
    if (fmtVersion === VERSION) codewords = interleaveBytes(codewords, plan);   // §6.1.3

    // 셀 강도 매트릭스 (255=흰 기본)
    const levels = new Uint8Array(cells * cells).fill(255);
    drawFinders(levels, input.grid);
    drawAlignment(levels, input.grid);

    // 코드워드 비트 → 데이터 셀 (MSB-first, bitsPerCell 씩 묶음)
    const totalBits = codewords.length * 8;
    const cellsNeeded = Math.ceil(totalBits / bitsPerCell);
    const getBit = (i) => i < totalBits ? ((codewords[i >> 3] >>> (7 - (i & 7))) & 1) : 0;
    let bitIdx = 0;
    for (let ci = 0; ci < cellsNeeded && ci < order.length; ci++) {
      let v = 0;
      for (let b = 0; b < bitsPerCell; b++) v = (v << 1) | getBit(bitIdx++);
      const [r, c] = order[ci];
      levels[r * cells + c] = cellValueToPixel(v, colorVariant);
    }
    // 남은 셀: 결정적 LCG 패딩
    let seed = (payloadCrc32 ^ 0xA5A5A5A5) >>> 0;
    for (let ci = cellsNeeded; ci < order.length; ci++) {
      seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0;
      const v = colorVariant ? ((seed >>> 16) & 0b11) : ((seed >>> 16) & 1);
      const [r, c] = order[ci];
      levels[r * cells + c] = cellValueToPixel(v, colorVariant);
    }

    // 트로이 가운데 QR 합성 (데이터 셀 위에 덮어쓰기 → corruption 은 ECC 가 흡수)
    let trojanMeta = null;
    if (wantTrojan) {
      const ecl = (input.flags && input.flags.qrEcl) || 'M';
      const url = trojanUrl(payloadCrc32);
      const qr = overlayTrojan(levels, cells, url, ecl);
      trojanMeta = { trojan: true, short_id: shortIdFromCrc32(payloadCrc32), trojan_url: url, qr_version: QR_VERSION, qr_ecl: qr.ecl, qr_mask: qr.mask };
    }

    return {
      matrix: { cells, levels, color: !!colorVariant, colorVariant, bitsPerCell, grid: input.grid, trojan: !!wantTrojan },
      meta: Object.assign({
        grid: input.grid, mode: input.mode, color: !!colorVariant, color_variant: colorVariant,
        bits_per_cell: bitsPerCell, canvas_size: cells * CELL,
        payload_length: payload.length, payload_crc32: payloadCrc32, header_crc16: headerCrc16,
        flags_byte: flagsByte, ecc_used: eccStr,
        capacity_bytes: plan.totalK, used_bytes: hpp.length,
        data_cells: order.length, rs_blocks: plan.blocks.length, rs_total_codewords: plan.totalN,
        encode_time_ms: Date.now() - t0,
      }, trojanMeta || { trojan: false }),
    };
  }
  function isMedical(p) {
    if (!p) return false;
    return typeof p.data_type === 'number' ? p.data_type === STATIC_DATA_TYPES.medical : p.data_type === 'medical';
  }
  function cellValueToPixel(v, colorVariant) {
    if (colorVariant) return CHANNEL_LEVELS[v & 0b11];   // 4단계 그레이
    return (v & 1) ? 0 : 255;                            // 흑백: 1=검정
  }

  // ──────────────────────────────────────────────────────────────────────
  // 8. 래스터화 — 셀 매트릭스 → RGBA ImageData-like (브라우저·Node 공용)
  //    quietZone: 외곽 흰 여백(모듈), 기본 4 (인쇄/카메라 권장). scale: 셀 픽셀 배율
  // ──────────────────────────────────────────────────────────────────────
  // 렌더러. opts.style='quantum' — 🧬 퀀텀 스킨 (wia-quantum-code 유산의 WIA3 이식):
  //  디코더 호환 유지 룰 ① 배경·콰이엇존 순백(극성·핀더 런 구조) ② 핀더·정렬
  //  패턴은 표준 정사각(스캔라인 검출 계약) ③ 데이터 셀만 원형 도트, 어두운
  //  레벨(≤60)만 딥 인디고 틴트(루마 ≈ 유지 — Otsu·§5.3 그레이 임계 마진 보존).
  //  도트 중심부는 solid 라 sampleCellGrays(반경 cellPx/4 평균)가 원본과 동일 판독.
  function toImageData(matrix, opts) {
    opts = opts || {};
    const quiet = opts.quietZone === undefined ? 4 : opts.quietZone;
    const cellPx = (opts.cellPx || CELL) * (opts.scale || 1);
    const cells = matrix.cells;
    const W = (cells + quiet * 2) * cellPx;
    const data = new Uint8ClampedArray(W * W * 4).fill(255); // 흰 배경
    const grade = ({ 64: 'S', 96: 'M', 128: 'L' })[cells] || null;
    const quantum = opts.style === 'quantum' && grade !== null;
    // 트로이 가운데 QR 영역은 외부 일반 QR 스캐너 계약 — 도트화 금지, 정사각 유지
    const tr = (quantum && matrix.trojan) ? trojanRect(cells) : null;
    for (let r = 0; r < cells; r++) {
      for (let c = 0; c < cells; c++) {
        const g = matrix.levels[r * cells + c];
        if (g === 255) continue;
        const x0 = (c + quiet) * cellPx, y0 = (r + quiet) * cellPx;
        const inTrojan = tr && r >= tr.f0 && r <= tr.f1 && c >= tr.f0 && c <= tr.f1;
        if (quantum && !inTrojan && !isReserved(grade, r, c)) {
          // 뉴럴 도트 (안티앨리어스 원)
          const cxd = x0 + cellPx / 2 - 0.5, cyd = y0 + cellPx / 2 - 0.5, rad = cellPx * 0.46;
          let R = g, Gc = g, B = g;
          if (g <= 60) { R = 28 + (g >> 2); Gc = 22 + (g >> 2); B = Math.min(255, 96 + (g >> 1)); }
          for (let dy = 0; dy < cellPx; dy++) {
            let off = ((y0 + dy) * W + x0) * 4;
            for (let dx = 0; dx < cellPx; dx++) {
              const dxp = x0 + dx - cxd, dyp = y0 + dy - cyd;
              const cov = Math.max(0, Math.min(1, rad + 0.5 - Math.sqrt(dxp * dxp + dyp * dyp)));
              if (cov > 0) {
                data[off] = 255 + (R - 255) * cov;
                data[off + 1] = 255 + (Gc - 255) * cov;
                data[off + 2] = 255 + (B - 255) * cov;
              }
              off += 4;
            }
          }
        } else {
          for (let dy = 0; dy < cellPx; dy++) {
            let off = ((y0 + dy) * W + x0) * 4;
            for (let dx = 0; dx < cellPx; dx++) { data[off] = g; data[off + 1] = g; data[off + 2] = g; data[off + 3] = 255; off += 4; }
          }
        }
      }
    }
    return { data, width: W, height: W };
  }

  // 브라우저: <canvas> 에 직접 렌더
  function renderToCanvas(matrix, canvas, opts) {
    const img = toImageData(matrix, opts);
    canvas.width = img.width; canvas.height = img.height;
    const ctx = canvas.getContext('2d');
    const id = ctx.createImageData(img.width, img.height);
    id.data.set(img.data);
    ctx.putImageData(id, 0, 0);
    return canvas;
  }

  // 편의: encode + 캔버스 렌더 한 번에
  function encodeToCanvas(input, canvas, renderOpts) {
    const res = encode(input);
    renderToCanvas(res.matrix, canvas, renderOpts);
    return res;
  }

  // ──────────────────────────────────────────────────────────────────────
  // 9. 이미지 처리 (decode-otsu.js 포팅)
  // ──────────────────────────────────────────────────────────────────────
  function toGray(img) {
    const { data } = img, out = new Uint8Array(img.width * img.height);
    for (let i = 0, j = 0; i < data.length; i += 4, j++) out[j] = (data[i] * 77 + data[i + 1] * 150 + data[i + 2] * 29) >> 8;
    return out;
  }
  function otsu(gray) {
    const hist = new Uint32Array(256);
    for (let i = 0; i < gray.length; i++) hist[gray[i]]++;
    const total = gray.length; let sum = 0;
    for (let i = 0; i < 256; i++) sum += i * hist[i];
    let sumB = 0, wB = 0, maxVar = -1, thr = 128;
    for (let t = 0; t < 256; t++) {
      wB += hist[t]; if (wB === 0) continue;
      const wF = total - wB; if (wF === 0) break;
      sumB += t * hist[t];
      const mB = sumB / wB, mF = (sum - sumB) / wF, v = wB * wF * (mB - mF) * (mB - mF);
      if (v > maxVar) { maxVar = v; thr = t; }
    }
    return thr < 40 ? 40 : thr > 180 ? 180 : thr;
  }
  function binarize(gray, thr) { const out = new Uint8Array(gray.length); for (let i = 0; i < gray.length; i++) out[i] = gray[i] < thr ? 1 : 0; return out; }
  // 3×3 가우시안 근사(분리형 [1,2,1]) — 노이즈 평균화 재시도 패스용. 그레이 1채널.
  function blurGray(gray, w, h) {
    const tmp = new Uint8Array(gray.length), out = new Uint8Array(gray.length);
    for (let y = 0; y < h; y++) {
      const o = y * w;
      tmp[o] = gray[o]; tmp[o + w - 1] = gray[o + w - 1];
      for (let x = 1; x < w - 1; x++) tmp[o + x] = (gray[o + x - 1] + 2 * gray[o + x] + gray[o + x + 1]) >> 2;
    }
    for (let x = 0; x < w; x++) {
      out[x] = tmp[x]; out[(h - 1) * w + x] = tmp[(h - 1) * w + x];
      for (let y = 1; y < h - 1; y++) out[y * w + x] = (tmp[(y - 1) * w + x] + 2 * tmp[y * w + x] + tmp[(y + 1) * w + x]) >> 2;
    }
    return out;
  }

  // ── 핀더 검출 (decode-finder.js 포팅) ──────────────────────────────────
  function findHorizontal(binary, width, height, step) {
    const cands = [];
    for (let y = 0; y < height; y += step) {
      const rowOff = y * width;
      let runs = [0, 0, 0, 0, 0], runIdx = 0, cur = binary[rowOff], runStart = 0;
      if (cur === 0) runIdx = -1;
      for (let x = 1; x <= width; x++) {
        const v = (x < width) ? binary[rowOff + x] : (1 - cur);
        if (v !== cur) {
          const runLen = x - runStart;
          if (runIdx >= 0 && runIdx < 5) {
            runs[runIdx] = runLen;
            if (runIdx === 4) {
              const total = runs[0] + runs[1] + runs[2] + runs[3] + runs[4], mod = total / 7;
              if (mod >= 0.8) {
                const ok = Math.abs(runs[0] - mod) < mod * 0.5 && Math.abs(runs[1] - mod) < mod * 0.5 &&
                           Math.abs(runs[2] - 3 * mod) < mod * 1.0 && Math.abs(runs[3] - mod) < mod * 0.5 &&
                           Math.abs(runs[4] - mod) < mod * 0.5;
                if (ok) cands.push({ x: x - runs[4] - runs[3] - runs[2] / 2, y, mod, total });
              }
              runs = [runs[1], runs[2], runs[3], runs[4], 0]; runIdx = 4;
            } else runIdx++;
          } else if (runIdx === -1 && cur === 0) runIdx = 0;
          runStart = x; cur = v;
        }
      }
    }
    return cands;
  }
  function verifyVertical(binary, width, height, cx, cy, expMod) {
    const xi = Math.round(cx); if (xi < 0 || xi >= width) return null;
    const yi = Math.round(cy); if (binary[yi * width + xi] !== 1) return null;
    let topBlack = yi; while (topBlack > 0 && binary[(topBlack - 1) * width + xi] === 1) topBlack--;
    let topWhiteEnd = topBlack - 1; while (topWhiteEnd > 0 && binary[(topWhiteEnd - 1) * width + xi] === 0) topWhiteEnd--;
    let topOuterEnd = topWhiteEnd - 1; while (topOuterEnd > 0 && binary[(topOuterEnd - 1) * width + xi] === 1) topOuterEnd--;
    let botBlack = yi; while (botBlack < height - 1 && binary[(botBlack + 1) * width + xi] === 1) botBlack++;
    let botWhiteEnd = botBlack + 1; while (botWhiteEnd < height - 1 && binary[(botWhiteEnd + 1) * width + xi] === 0) botWhiteEnd++;
    let botOuterEnd = botWhiteEnd + 1; while (botOuterEnd < height - 1 && binary[(botOuterEnd + 1) * width + xi] === 1) botOuterEnd++;
    const r2 = botBlack - topBlack + 1, r1 = topBlack - topWhiteEnd, r0 = topWhiteEnd - topOuterEnd,
          r3 = botWhiteEnd - botBlack, r4 = botOuterEnd - botWhiteEnd;
    if (r0 < 1 || r1 < 1 || r3 < 1 || r4 < 1 || r2 < 1) return null;
    const mod = (r0 + r1 + r2 + r3 + r4) / 7; if (mod < 0.8) return null;
    if (Math.abs(r0 - mod) > mod * 0.6 || Math.abs(r1 - mod) > mod * 0.6 || Math.abs(r2 - 3 * mod) > mod * 1.0 ||
        Math.abs(r3 - mod) > mod * 0.6 || Math.abs(r4 - mod) > mod * 0.6) return null;
    if (expMod && (mod < expMod * 0.6 || mod > expMod * 1.6)) return null;
    return { x: cx, y: (topBlack + botBlack) / 2, mod, size: 7 * mod };
  }
  function nms(fs) {
    const out = [];
    for (const f of fs) { let dup = false; for (const o of out) if (Math.hypot(f.x - o.x, f.y - o.y) < (f.size + o.size) / 4) { dup = true; break; } if (!dup) out.push(f); }
    return out;
  }
  // 핀더 구조 품질 — 중심에서 8방향 레이가 core(흑)→gap(백)→ring(흑) 런 구조를
  // 보이는지 셈. 동심 정사각형은 어느 방향 레이든 이 순서를 지나므로 회전 불변.
  // 밀집 데이터 영역의 우연한 1:1:3:1:1 히트(가짜 핀더)는 대부분 전방향을 못 채운다.
  function finderRayQuality(binary, width, height, f) {
    const dirs = [[1, 0], [-1, 0], [0, 1], [0, -1], [0.7071, 0.7071], [0.7071, -0.7071], [-0.7071, 0.7071], [-0.7071, -0.7071]];
    const step = Math.max(0.5, f.mod / 4), maxT = 6 * f.mod;
    let good = 0;
    for (const [dx, dy] of dirs) {
      let phase = 0, runStart = 0, ok = false;   // 0=core흑 1=gap백 2=ring흑
      for (let t = 0; t <= maxT; t += step) {
        const x = Math.round(f.x + dx * t), y = Math.round(f.y + dy * t);
        const v = (x >= 0 && x < width && y >= 0 && y < height) ? binary[y * width + x] : 0;
        if (phase === 0 && v === 0) {
          const L = t - runStart;
          if (L < 0.8 * f.mod || L > 3.4 * f.mod) break;      // core 반폭 1~1.5·√2 mod
          phase = 1; runStart = t;
        } else if (phase === 1 && v === 1) {
          const L = t - runStart;
          if (L < 0.3 * f.mod || L > 2.4 * f.mod) break;      // gap 1~1.5·√2 mod
          phase = 2; ok = true; break;
        }
      }
      if (ok) good++;
    }
    return good;                                              // 최대 8
  }
  function findFinders(binary, width, height) {
    const step = Math.max(1, Math.floor(Math.min(width, height) / 200));
    const horiz = findHorizontal(binary, width, height, step), verified = [];
    for (const c of horiz) { const v = verifyVertical(binary, width, height, c.x, c.y, c.mod); if (v) verified.push(v); }
    const out = nms(verified);
    // 품질 내림차순 정렬 — 후보 상한(slice) 시 진짜 핀더가 밀려나지 않게.
    // (밀집 페이로드의 L 그리드에서 가짜 후보 15+개가 스캔순서 앞을 차지해
    //  진짜 BL 이 탈락하던 잠복 버그의 수정)
    for (const f of out) f.q = finderRayQuality(binary, width, height, f);
    out.sort((a, b) => b.q - a.q);
    return out;
  }
  function identifyTLTRBL(a, b, c) {
    const dAB = Math.hypot(a.x - b.x, a.y - b.y), dAC = Math.hypot(a.x - c.x, a.y - c.y), dBC = Math.hypot(b.x - c.x, b.y - c.y);
    let tl, tr, bl;
    if (dAB >= dAC && dAB >= dBC) { tl = c; const cr = (a.x - c.x) * (b.y - c.y) - (a.y - c.y) * (b.x - c.x); if (cr > 0) { tr = a; bl = b; } else { tr = b; bl = a; } }
    else if (dAC >= dAB && dAC >= dBC) { tl = b; const cr = (a.x - b.x) * (c.y - b.y) - (a.y - b.y) * (c.x - b.x); if (cr > 0) { tr = a; bl = c; } else { tr = c; bl = a; } }
    else { tl = a; const cr = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); if (cr > 0) { tr = b; bl = c; } else { tr = c; bl = b; } }
    return { tl, tr, bl };
  }
  function scoreTriple(tl, tr, bl) {
    const dTLTR = Math.hypot(tr.x - tl.x, tr.y - tl.y), dTLBL = Math.hypot(bl.x - tl.x, bl.y - tl.y);
    const lenDiff = Math.abs(dTLTR - dTLBL);
    const dot = (tr.x - tl.x) * (bl.x - tl.x) + (tr.y - tl.y) * (bl.y - tl.y);
    const angleDev = Math.abs(dot) / (dTLTR * dTLBL + 1e-9);
    const modVar = Math.abs(tl.mod - tr.mod) + Math.abs(tr.mod - bl.mod) + Math.abs(bl.mod - tl.mod);
    const dTRBL = Math.hypot(tr.x - bl.x, tr.y - bl.y);
    return { baseScore: 1 / (1 + lenDiff / 5 + angleDev * 50 + modVar), triangleSize: (dTLTR + dTLBL + dTRBL) / 3 };
  }
  function sortFindersRanked(finders, k) {
    if (finders.length < 3) return [];
    if (finders.length === 3) return [identifyTLTRBL(finders[0], finders[1], finders[2])];
    const cands = finders.slice(0, Math.min(finders.length, 16)), all = [];
    for (let i = 0; i < cands.length; i++) for (let j = i + 1; j < cands.length; j++) for (let m = j + 1; m < cands.length; m++) {
      const tri = identifyTLTRBL(cands[i], cands[j], cands[m]); const s = scoreTriple(tri.tl, tri.tr, tri.bl);
      all.push({ triple: tri, baseScore: s.baseScore, triangleSize: s.triangleSize });
    }
    if (!all.length) return [];
    all.sort((a, b) => b.baseScore - a.baseScore);
    const top = all.filter(x => x.baseScore >= all[0].baseScore * 0.5);
    top.sort((a, b) => b.triangleSize - a.triangleSize);
    return top.slice(0, k || 2).map(x => x.triple);
  }
  function sortFinders(finders) { const r = sortFindersRanked(finders, 1); return r.length ? r[0] : null; }
  // 후보 삼중조 열거 — scoreTriple 의 등변·직각 가정은 강한 원근에서 압축된
  // 진짜 삼중조를 걸러버린다(yaw 40° 실측). 비공선(roundness)과 크기만으로
  // 남기고, 참/거짓 판정은 핀더 패턴 적합도 + CRC 게이트에 넘긴다.
  function plausibleTriples(finders, cap) {
    if (finders.length < 3) return [];
    if (finders.length === 3) return [identifyTLTRBL(finders[0], finders[1], finders[2])];
    const cands = finders.slice(0, Math.min(finders.length, 12)), scored = [];
    for (let i = 0; i < cands.length; i++) for (let j = i + 1; j < cands.length; j++) for (let m = j + 1; m < cands.length; m++) {
      const a = cands[i], b = cands[j], c = cands[m];
      const area2 = Math.abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
      const per = Math.hypot(b.x - a.x, b.y - a.y) + Math.hypot(c.x - a.x, c.y - a.y) + Math.hypot(c.x - b.x, c.y - b.y);
      if (per <= 0 || area2 / (per * per) < 0.01) continue;   // 공선/퇴화 제거
      scored.push({ tri: identifyTLTRBL(a, b, c), size: area2 });
    }
    scored.sort((x, y) => y.size - x.size);
    return scored.slice(0, cap || 8).map(s => s.tri);
  }
  // 최장변 규칙(identifyTLTRBL)은 강한 원근에서 변 길이가 왜곡돼 진짜 TR/BL을
  // TL 로 오인한다(yaw 40°+ 실측). 세 점 각각을 TL 로 두는 라벨링 3종을 만들어
  // 핀더 패턴 적합도 + CRC 게이트가 판정하게 한다. [0]은 기존 기본 라벨링.
  function labelingsOf(tri) {
    const pts = [tri.tl, tri.tr, tri.bl], out = [];
    for (let i = 0; i < 3; i++) {
      const tl = pts[i], p1 = pts[(i + 1) % 3], p2 = pts[(i + 2) % 3];
      const cr = (p1.x - tl.x) * (p2.y - tl.y) - (p1.y - tl.y) * (p2.x - tl.x);
      out.push(cr > 0 ? { tl, tr: p1, bl: p2 } : { tl, tr: p2, bl: p1 });
    }
    return out;
  }
  // 그리드 판별 — 회전 보정 + 후보 랭킹.
  // verifyVertical 의 mod 는 수직 런 기반이라 roll θ 에서 1/cosθ 과대 측정
  // → modDist 가 cosθ 로 축소되어 고정 허용오차를 벗어나거나(기준선 회전 붕괴의
  // 실병목) 인접 그리드로 앨리어싱된다(L rot45°→M). 원시/cos보정 두 해석 중
  // 가까운 쪽으로 채점하고, 허용오차 내 후보를 전부 순서대로 반환한다(CRC 게이트).
  function detectGridCandidates(sorted) {
    const { tl, tr, bl } = sorted;
    const dxTR = Math.hypot(tr.x - tl.x, tr.y - tl.y), dxBL = Math.hypot(bl.x - tl.x, bl.y - tl.y);
    const mod = (tl.mod + tr.mod + bl.mod) / 3; if (mod <= 0 || tl.mod <= 0 || tr.mod <= 0 || bl.mod <= 0) return [];
    // 팔(arm)별 추정: 원근에서 한쪽 팔만 압축될 때 전체 평균 mod 로 나누면
    // modDist 가 비뚤어져 정답 라벨링이 기각된다. 팔 양끝 mod 평균은 팔 위의
    // 평균 스케일에 1차 근사로 일치한다.
    const modDist = (dxTR / ((tl.mod + tr.mod) / 2) + dxBL / ((tl.mod + bl.mod) / 2)) / 2;
    const cosR = Math.max(0.5, Math.abs(Math.cos(Math.atan2(tr.y - tl.y, tr.x - tl.x))));
    const scored = [];
    for (const k of ['S', 'M', 'L']) {
      const fmd = GRIDS[k].finderModuleDistance;
      const d = Math.min(Math.abs(modDist - fmd), Math.abs(modDist / cosR - fmd));
      scored.push({ k, d });
    }
    scored.sort((a, b) => a.d - b.d);
    const within = scored.filter(s => s.d <= 6);
    const picked = within.length ? within : (scored[0].d <= 12 ? [scored[0]] : []);
    return picked.map(s => ({ grade: s.k, spec: GRIDS[s.k], moduleDistance: modDist, mod }));
  }
  function detectGrid(sorted) { const c = detectGridCandidates(sorted); return c.length ? c[0] : null; }

  // ── 호모그래피 (decode-homography.js 포팅) ─────────────────────────────
  function computeHomography(dst, src) {
    const A = [];
    for (let i = 0; i < 4; i++) {
      const [x, y] = dst[i], [u, v] = src[i];
      A.push([x, y, 1, 0, 0, 0, -u * x, -u * y, u]);
      A.push([0, 0, 0, x, y, 1, -v * x, -v * y, v]);
    }
    for (let col = 0; col < 8; col++) {
      let piv = col, mx = Math.abs(A[col][col]);
      for (let r = col + 1; r < 8; r++) { const a = Math.abs(A[r][col]); if (a > mx) { mx = a; piv = r; } }
      if (mx < 1e-10) return null;
      if (piv !== col) { const t = A[col]; A[col] = A[piv]; A[piv] = t; }
      for (let r = col + 1; r < 8; r++) { const f = A[r][col] / A[col][col]; for (let c = col; c < 9; c++) A[r][c] -= f * A[col][c]; }
    }
    const h = new Array(8);
    for (let r = 7; r >= 0; r--) { let s = A[r][8]; for (let c = r + 1; c < 8; c++) s -= A[r][c] * h[c]; h[r] = s / A[r][r]; }
    return [h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], 1.0];
  }
  // 정준 좌표 → 소스 투영 (H: dst→src)
  function projectH(H, x, y) {
    const w = H[6] * x + H[7] * y + H[8];
    if (Math.abs(w) < 1e-10) return null;
    return [(H[0] * x + H[1] * y + H[2]) / w, (H[3] * x + H[4] * y + H[5]) / w];
  }
  function sampleGrayAt(gray, w, h, x, y) {
    const xi = Math.round(x), yi = Math.round(y);
    if (xi < 0 || xi >= w || yi < 0 || yi >= h) return 255;
    return gray[yi * w + xi];
  }
  // 바이리니어 샘플 — 최근접은 1px 계단이라 원근 미세 최적화의 목적함수가
  // 평탄역(plateau)을 만든다. 경계 밖 = 255(백).
  function sampleGrayBilinear(gray, w, h, x, y) {
    if (x < 0 || y < 0 || x > w - 1 || y > h - 1) return 255;
    const x0 = Math.floor(x), y0 = Math.floor(y), x1 = Math.min(x0 + 1, w - 1), y1 = Math.min(y0 + 1, h - 1);
    const fx = x - x0, fy = y - y0;
    const p00 = gray[y0 * w + x0], p10 = gray[y0 * w + x1], p01 = gray[y1 * w + x0], p11 = gray[y1 * w + x1];
    return (p00 * (1 - fx) + p10 * fx) * (1 - fy) + (p01 * (1 - fx) + p11 * fx) * fy;
  }
  // 주어진 H(dst→src)로 소스 회색조를 SIZE×SIZE 정준 이미지로 워핑
  function warpImageWithH(gray, srcW, srcH, H, SIZE) {
    const out = new Uint8Array(SIZE * SIZE);
    for (let y = 0; y < SIZE; y++) for (let x = 0; x < SIZE; x++) {
      const w = H[6] * x + H[7] * y + H[8];
      if (Math.abs(w) < 1e-10) { out[y * SIZE + x] = 255; continue; }
      const sx = Math.round((H[0] * x + H[1] * y + H[2]) / w), sy = Math.round((H[3] * x + H[4] * y + H[5]) / w);
      out[y * SIZE + x] = (sx >= 0 && sx < srcW && sy >= 0 && sy < srcH) ? gray[sy * srcW + sx] : 255;
    }
    return out;
  }
  function warpToCanonical(img, gray, sorted, cells) {
    const SIZE = cells * CELL, halfF = 3.5 * CELL;
    const { tl, tr, bl } = sorted;
    // 4번째 점(우하)은 평행사변형 가정으로 합성 — 아핀 근사(원근 미보정). §D.2에서 정렬패턴으로 보강.
    const brx = tr.x + bl.x - tl.x, bry = tr.y + bl.y - tl.y;
    const src = [[tl.x, tl.y], [tr.x, tr.y], [bl.x, bl.y], [brx, bry]];
    const dst = [[halfF, halfF], [SIZE - halfF, halfF], [halfF, SIZE - halfF], [SIZE - halfF, SIZE - halfF]];
    const H = computeHomography(dst, src); if (!H) return null;
    const out = warpImageWithH(gray, img.width, img.height, H, SIZE);
    return { warped: out, size: SIZE, H: H };
  }
  // ── §D.2: L 그리드 정렬 패턴 기반 원근 미세 보정 ───────────────────────
  // 1차 H(평행사변형 합성 우하)는 큰 각도/페이지 휨에서 먼 쪽 셀이 어긋난다.
  // 우하 안쪽 정렬 패턴(중심 셀 = G-7)의 실측 중심을 4번째 대응점으로 삼아
  // 진짜 원근 호모그래피를 재계산. 성공 시 refined H(dst→src), 실패 시 null(→ coarse 폴백).
  function refineHomographyWithAlignment(gray, srcW, srcH, sorted, cells, H) {
    if (cells !== 128) return null;                       // 정렬 패턴은 L 그리드만 존재
    const SIZE = cells * CELL, halfF = 3.5 * CELL;
    const ax = (cells - 7 + 0.5) * CELL, ay = ax;         // 정렬 중심 셀 = G-7 의 정준 픽셀
    const { tl, tr, bl } = sorted;
    const spacing = (Math.hypot(tr.x - tl.x, tr.y - tl.y) + Math.hypot(bl.x - tl.x, bl.y - tl.y)) / 2;
    const modSrc = spacing / (cells - 7);                 // 소스 1모듈 픽셀 크기 (핀더 중심간 = cells-7 모듈)
    if (!(modSrc > 0)) return null;
    const pred = projectH(H, ax, ay); if (!pred) return null;
    let px = pred[0], py = pred[1];
    // 다크 무게중심으로 실측 중심 수렴 (창 재중심 3회 반복으로 절단 편향 완화)
    const win = 2.5 * modSrc, step = Math.max(1, modSrc / 2);
    for (let iter = 0; iter < 3; iter++) {
      let sw = 0, swx = 0, swy = 0;
      for (let dy = -win; dy <= win; dy += step) for (let dx = -win; dx <= win; dx += step) {
        const g = sampleGrayAt(gray, srcW, srcH, px + dx, py + dy);
        const wgt = Math.max(0, 255 - g);                 // 어두울수록 가중
        sw += wgt; swx += wgt * (px + dx); swy += wgt * (py + dy);
      }
      if (sw <= 0) return null;
      px = swx / sw; py = swy / sw;
    }
    // 정렬 패턴 서명 검증: 중심 어둡고 · 링(±2모듈) 어둡고 · 갭(±1모듈) 상대적으로 밝음
    const dark = (x, y) => sampleGrayAt(gray, srcW, srcH, x, y) < 128;
    const centerDark = dark(px, py);
    const ring = [[2, 0], [-2, 0], [0, 2], [0, -2]].filter(m => dark(px + m[0] * modSrc, py + m[1] * modSrc)).length >= 3;
    const gapLight = [[1, 0], [-1, 0], [0, 1], [0, -1]].filter(m => !dark(px + m[0] * modSrc, py + m[1] * modSrc)).length >= 2;
    if (!(centerDark && ring && gapLight)) return null;
    if (Math.hypot(px - pred[0], py - pred[1]) > 4 * modSrc) return null;  // 비상식적 이동 = 오검출
    const dst4 = [[halfF, halfF], [SIZE - halfF, halfF], [halfF, SIZE - halfF], [ax, ay]];
    const src4 = [[tl.x, tl.y], [tr.x, tr.y], [bl.x, bl.y], [px, py]];
    return computeHomography(dst4, src4);
  }
  // ── Phase 1: 원근 복원 (§9.2 [3] 보강 · 포맷 무변경) ────────────────────
  // 핀더 3점만으로는 사영 성분이 결정되지 않아(아핀) yaw 10°부터 BR 사분면
  // 샘플링이 어긋난다. 사영 성분 (g,h)를 핀더별 실측 모듈크기 비대칭에서
  // 폐형 추정하고, 핀더 7×7 기대 패턴 적합도를 목적함수로 좌표하강(quirc
  // jiggle 방식) 최적화한다. (g,h) 고정 시 아핀 성분은 3중심 대응으로 유일.
  function canonicalFinderFrame(cells) {
    const SIZE = cells * CELL, t = 3.5 * CELL;
    return { SIZE, t, D: SIZE - t };
  }
  function solveHWithPerspective(sorted, cells, g, h) {
    const { t, D } = canonicalFinderFrame(cells);
    const wTL = g * t + h * t + 1, wTR = g * D + h * t + 1, wBL = g * t + h * D + 1;
    if (wTL < 0.05 || wTR < 0.05 || wBL < 0.05) return null;   // 뒤집힘/극단 기각
    const U1 = sorted.tl.x * wTL, U2 = sorted.tr.x * wTR, U3 = sorted.bl.x * wBL;
    const V1 = sorted.tl.y * wTL, V2 = sorted.tr.y * wTR, V3 = sorted.bl.y * wBL;
    const inv = 1 / (D - t);
    const a = (U2 - U1) * inv, b = (U3 - U1) * inv, c = U1 - (a + b) * t;
    const d = (V2 - V1) * inv, e = (V3 - V1) * inv, f = V1 - (d + e) * t;
    return [a, b, c, d, e, f, g, h, 1];
  }
  // 핀더 중심 서브픽셀 재정밀화 — 스캔라인 런 중점은 원근 foreshortening 에서
  // 편향된다(중심 오차가 아핀 성분을 거쳐 원거리 코너로 증폭). 3×3 core 의
  // 다크 무게중심으로 수렴. 실패/과이동 시 원본 유지.
  function refineFinderCenter(gray, srcW, srcH, f) {
    let px = f.x, py = f.y;
    // 원형 창 반경 1.6·mod — 정사각 창은 45° 회전에서 대각 모서리가
    // (mod 과대측정 1.41배와 겹쳐) 바깥 링을 침범해 무게중심을 끌어당긴다.
    const win = 1.6 * f.mod, win2 = win * win, step = Math.max(0.5, f.mod / 4);
    for (let iter = 0; iter < 3; iter++) {
      let sw = 0, swx = 0, swy = 0;
      for (let dy = -win; dy <= win; dy += step) for (let dx = -win; dx <= win; dx += step) {
        if (dx * dx + dy * dy > win2) continue;
        const g = sampleGrayBilinear(gray, srcW, srcH, px + dx, py + dy);
        const wgt = Math.max(0, 255 - g);
        sw += wgt; swx += wgt * (px + dx); swy += wgt * (py + dy);
      }
      if (sw <= 0) return f;
      px = swx / sw; py = swy / sw;
    }
    if (Math.hypot(px - f.x, py - f.y) > 1.5 * f.mod) return f;
    return { x: px, y: py, mod: f.mod, size: f.size };
  }
  // 핀더 기대 패턴 적합도: 중심 기준 체비쇼프 거리 0,1=흑(core) · 2=백(gap) ·
  // 3=흑(outer ring) · 4=백(separator/quiet). 3핀더 × 9×9 모듈 샘플.
  function finderPatternFitness(gray, srcW, srcH, H, cells) {
    const { t, D } = canonicalFinderFrame(cells);
    let score = 0;
    for (const [cx, cy] of [[t, t], [D, t], [t, D]]) {
      for (let my = -4; my <= 4; my++) for (let mx = -4; mx <= 4; mx++) {
        const m = Math.max(Math.abs(mx), Math.abs(my));
        const p = projectH(H, cx + mx * CELL, cy + my * CELL);
        if (!p) return -Infinity;
        const gv = sampleGrayBilinear(gray, srcW, srcH, p[0], p[1]);
        score += (m <= 1 || m === 3) ? (255 - gv) : gv;
      }
    }
    return score;
  }
  // 수직 런 기반 mod 는 국소 스케일 ∝ w^-k (k는 축·왜곡 방향 의존, 1~1.5).
  // 지수별 폐형 해를 전부 시드로 만들고 적합도로 고른다.
  function estimatePerspectiveSeeds(sorted, cells) {
    const { t, D } = canonicalFinderFrame(cells);
    const seeds = [[0, 0]];
    const mTL = sorted.tl.mod, mTR = sorted.tr.mod, mBL = sorted.bl.mod;
    if (mTL > 0 && mTR > 0 && mBL > 0) {
      for (const k of [1, 1.5]) {
        const rTR = Math.pow(mTL / mTR, 1 / k);    // = wTR/wTL
        const rBL = Math.pow(mTL / mBL, 1 / k);    // = wBL/wTL
        // w 비율 두 식을 (g,h)에 대해 선형화: g(D−r·t) + h·t(1−r) = r−1 (TR행)
        const a1 = D - rTR * t, b1 = t * (1 - rTR), c1 = rTR - 1;
        const a2 = t * (1 - rBL), b2 = D - rBL * t, c2 = rBL - 1;
        const det = a1 * b2 - a2 * b1;
        if (Math.abs(det) > 1e-12) {
          const g = (c1 * b2 - c2 * b1) / det, h = (a1 * c2 - a2 * c1) / det;
          if (isFinite(g) && isFinite(h)) seeds.push([g, h]);
        }
      }
    }
    return seeds;
  }
  function estimatePerspectiveH(gray, srcW, srcH, sorted, cells, quick) {
    const { SIZE } = canonicalFinderFrame(cells);
    let g = 0, h = 0, bestFit = -Infinity;
    for (const [sg, sh] of estimatePerspectiveSeeds(sorted, cells)) {
      const H = solveHWithPerspective(sorted, cells, sg, sh);
      if (!H) continue;
      const fit = finderPatternFitness(gray, srcW, srcH, H, cells);
      if (fit > bestFit) { bestFit = fit; g = sg; h = sh; }
    }
    if (bestFit === -Infinity) return null;
    if (quick) {                                   // 시드만 (해석 후보 랭킹용)
      const H = solveHWithPerspective(sorted, cells, g, h);
      return H ? { H, fit: bestFit } : null;
    }
    let delta = 0.35 / SIZE;                       // w 전폭 변화 ~0.35 부터 반감
    for (let pass = 0; pass < 8; pass++) {
      for (const [dg, dh] of [[delta, 0], [-delta, 0], [0, delta], [0, -delta]]) {
        for (let step = 0; step < 16; step++) {
          const H = solveHWithPerspective(sorted, cells, g + dg, h + dh);
          const fit = H ? finderPatternFitness(gray, srcW, srcH, H, cells) : -Infinity;
          if (fit > bestFit) { bestFit = fit; g += dg; h += dh; } else break;
        }
      }
      delta /= 2;
    }
    const H = solveHWithPerspective(sorted, cells, g, h);
    return H ? { H, fit: bestFit } : null;
  }
  // ── 외곽 4변 실측 → 4코너 재구성 (원근 복원의 최종 정밀화) ──────────────
  // 핀더 패턴 적합도의 지렛대는 핀더 반경 4모듈뿐이라 원거리 코너에서
  // 서브셀 정밀도가 원리적으로 안 나온다. 핀더 링은 코드 테두리 위에 있으므로
  // H 가 정확한 "핀더 근방"에서 외곽 에지를 서브픽셀 실측하고, 4변 직선의
  // 교점으로 진짜 4코너를 복원해 8-DOF H 를 다시 푼다(오라클과 동일 구성).
  function scanEdgePoint(gray, srcW, srcH, H, bx, by, ix, iy) {
    const S0 = 0.75 * CELL, S1 = -1.25 * CELL, STEP = 0.125 * CELL;
    const samples = []; let mn = 255, mx = 0;
    for (let s = S0; s >= S1; s -= STEP) {
      const p = projectH(H, bx + ix * s, by + iy * s); if (!p) return null;
      const g = sampleGrayBilinear(gray, srcW, srcH, p[0], p[1]);
      samples.push([s, g]); if (g < mn) mn = g; if (g > mx) mx = g;
    }
    if (mx - mn < 40) return null;                    // 대비 없음 = 에지 아님
    const thr = (mn + mx) / 2;
    for (let i = 1; i < samples.length; i++) {
      const s1 = samples[i - 1], s2 = samples[i];
      if (s1[1] <= thr && s2[1] > thr) {              // 안(흑)→밖(백) 첫 교차
        const f = (thr - s1[1]) / (s2[1] - s1[1]), s = s1[0] + f * (s2[0] - s1[0]);
        return projectH(H, bx + ix * s, by + iy * s);
      }
    }
    return null;
  }
  function fitLine(pts) {
    const n = pts.length; let mx = 0, my = 0;
    for (const p of pts) { mx += p[0]; my += p[1]; } mx /= n; my /= n;
    let sxx = 0, sxy = 0, syy = 0;
    for (const p of pts) { const dx = p[0] - mx, dy = p[1] - my; sxx += dx * dx; sxy += dx * dy; syy += dy * dy; }
    const L = (sxx + syy) / 2 + Math.sqrt(((sxx - syy) / 2) * ((sxx - syy) / 2) + sxy * sxy);
    let dx, dy;
    if (Math.abs(sxy) > 1e-9) { dx = L - syy; dy = sxy; }
    else if (sxx >= syy) { dx = 1; dy = 0; } else { dx = 0; dy = 1; }
    const len = Math.hypot(dx, dy) || 1;
    return { p: [mx, my], d: [dx / len, dy / len] };
  }
  function intersectLines(a, b) {
    const det = a.d[0] * b.d[1] - a.d[1] * b.d[0];
    if (Math.abs(det) < 1e-9) return null;
    const t = ((b.p[0] - a.p[0]) * b.d[1] - (b.p[1] - a.p[1]) * b.d[0]) / det;
    return [a.p[0] + t * a.d[0], a.p[1] + t * a.d[1]];
  }
  function refineHomographyWithEdges(gray, srcW, srcH, sorted, cells, H) {
    const SIZE = cells * CELL;
    const top = [], right = [], bottom = [], left = [];
    for (let k = 0; k < 7; k++) {
      const o = (k + 0.5) * CELL;
      let p = scanEdgePoint(gray, srcW, srcH, H, o, 0, 0, 1); if (p) top.push(p);         // TL 위변
      p = scanEdgePoint(gray, srcW, srcH, H, SIZE - o, 0, 0, 1); if (p) top.push(p);      // TR 위변
      p = scanEdgePoint(gray, srcW, srcH, H, 0, o, 1, 0); if (p) left.push(p);            // TL 좌변
      p = scanEdgePoint(gray, srcW, srcH, H, 0, SIZE - o, 1, 0); if (p) left.push(p);     // BL 좌변
      p = scanEdgePoint(gray, srcW, srcH, H, SIZE, o, -1, 0); if (p) right.push(p);       // TR 우변
      p = scanEdgePoint(gray, srcW, srcH, H, o, SIZE, 0, -1); if (p) bottom.push(p);      // BL 아래변
    }
    if (top.length < 8 || left.length < 8 || right.length < 5 || bottom.length < 5) return null;
    const cTL = intersectLines(fitLine(top), fitLine(left));
    const cTR = intersectLines(fitLine(top), fitLine(right));
    const cBL = intersectLines(fitLine(bottom), fitLine(left));
    const cBR = intersectLines(fitLine(bottom), fitLine(right));
    if (!cTL || !cTR || !cBL || !cBR) return null;
    // 코너가 현 H 예측에서 비상식적으로 벗어나면 오검출로 기각
    const modMax = Math.max(sorted.tl.mod, sorted.tr.mod, sorted.bl.mod);
    for (const [c, X, Y] of [[cTL, 0, 0], [cTR, SIZE, 0], [cBL, 0, SIZE], [cBR, SIZE, SIZE]]) {
      const pred = projectH(H, X, Y); if (!pred) return null;
      if (Math.hypot(c[0] - pred[0], c[1] - pred[1]) > 8 * modMax) return null;
    }
    return computeHomography([[0, 0], [SIZE, 0], [0, SIZE], [SIZE, SIZE]], [cTL, cTR, cBL, cBR]);
  }
  // 셀 평균 강도 샘플링 (분류는 호출측: 흑백=threshold, 컬러=4단계 nearest)
  function sampleCellGrays(warped, size, cells, ox, oy) {
    const cellPx = size / cells, out = new Uint8Array(cells * cells), r = Math.max(1, Math.floor(cellPx / 4));
    for (let cy = 0; cy < cells; cy++) {
      const yc = Math.round((cy + 0.5) * cellPx) + oy;
      for (let cx = 0; cx < cells; cx++) {
        const xc = Math.round((cx + 0.5) * cellPx) + ox;
        let sum = 0, cnt = 0;
        for (let dy = -r; dy <= r; dy++) { const y = yc + dy; if (y < 0 || y >= size) continue;
          for (let dx = -r; dx <= r; dx++) { const x = xc + dx; if (x < 0 || x >= size) continue; sum += warped[y * size + x]; cnt++; } }
        out[cy * cells + cx] = cnt ? Math.round(sum / cnt) : 255;
      }
    }
    return out;
  }
  function nearestLevelIndex(g) {
    let bi = 0, bd = Infinity;
    for (let i = 0; i < 4; i++) { const d = Math.abs(g - CHANNEL_LEVELS[i]); if (d < bd) { bd = d; bi = i; } }
    return bi;
  }
  // 데이터 셀 강도 → 바이트 (bitsPerCell: 1=흑백, 2=컬러). thr: 흑백 임계값.
  function extractBytes(cellGrays, grade, bitsPerCell, thr) {
    const order = dataCellOrder(grade), cells = gridDef(grade).cells;
    const totalBits = order.length * bitsPerCell, byteLen = Math.floor(totalBits / 8);
    const out = new Uint8Array(byteLen);
    let bitPos = 0;
    for (let i = 0; i < order.length; i++) {
      const [r, c] = order[i], g = cellGrays[r * cells + c];
      let v;
      if (bitsPerCell === 1) v = g < thr ? 1 : 0;
      else v = nearestLevelIndex(g);                  // 0..3 (2비트)
      for (let b = bitsPerCell - 1; b >= 0; b--) {
        const bit = (v >> b) & 1, idx = bitPos++;
        if (idx >= byteLen * 8) break;
        out[idx >> 3] |= bit << (7 - (idx & 7));
      }
    }
    return out;
  }

  // ──────────────────────────────────────────────────────────────────────
  // 10. 디코더 — RGBA ImageData-like → DecodeResult (스펙 §9.2 13단계)
  //     흑백/컬러 · 4 ECC비율 · 7 오프셋 전수 시도(CRC가 정합 게이트).
  //     트로이 QR 폴백 훅(decodeQR 주입 시) — 기본은 미연결.
  // ──────────────────────────────────────────────────────────────────────
  function decode(img, opts) {
    opts = opts || {};
    const t0 = Date.now();
    const fail = (errors, extra) => Object.assign({
      success: false, mode: 'error', data: null, reliability: 0, source: 'on-device',
      errors: Array.isArray(errors) ? errors : [errors],
      meta: { grid: null, color: false, trojan: false, ecc_used: null, decode_time_ms: Date.now() - t0 },
    }, extra || {});

    if (!img || !img.data || !img.width) return fail('invalid image input');
    const gray = toGray(img);
    // 1차: 원본 그레이 → 실패 시 2차: 3×3 가우시안 블러(노이즈 평균화) 재시도.
    // 추가 시도는 RS+CRC 3중 게이트라 무회귀 (레거시 폴백 사다리의 1순위 항목 포팅).
    // opts.fast: 실시간 카메라 프레임용 — 해석 후보 상한 축소 + 블러 재시도 생략.
    // (손떨림으로 프레임마다 조건이 바뀌므로, 프레임당 얕게 × 여러 프레임이
    //  한 프레임 깊게보다 체감 히트율이 좋다. 주기적 full 프레임은 호출측이 섞음.)
    const a1 = _decodeAttempt(img, gray, t0, opts);
    if (a1.result) return a1.result;
    if (opts.fast) return tryQRFallback(gray, img, t0, a1.errors, opts, fail);
    const a2 = _decodeAttempt(img, blurGray(gray, img.width, img.height), t0, opts);
    if (a2.result) return a2.result;
    const allErrors = a1.errors.concat(a2.errors.filter(e => a1.errors.indexOf(e) < 0).map(e => 'blur-retry: ' + e));
    return tryQRFallback(gray, img, t0, allErrors, opts, fail);
  }

  // 한 그레이 이미지에 대한 전체 해석 탐색 시도. 성공 시 {result}, 실패 시 {errors}.
  function _decodeAttempt(img, gray, t0, opts) {
    const binary = binarize(gray, otsu(gray));
    const found = findFinders(binary, img.width, img.height);
    if (found.length < 3) return { errors: ['finder count < 3 (' + found.length + ')'] };
    const finders = found.map(f => refineFinderCenter(gray, img.width, img.height, f));
    const triples = plausibleTriples(finders, 8);
    if (!triples.length) return { errors: ['finder sort failed'] };

    // 해석 후보 = (비퇴화 삼중조 ≤8) × (TL 라벨링 3) × (그리드 후보) 를 핀더
    // 패턴 적합도(시드만, 지글 생략)로 랭킹. 기하 규칙이 강한 원근/가짜
    // 핀더에서 고른 오답을 적합도가 뒤로 밀고, 최종 판정은 RS+CRC 3중
    // 게이트가 한다(무회귀). 상위 후보만 전체 원근 최적화+디코드를 받는다.
    const interps = [];
    for (const tri of triples) {
      for (const lab of labelingsOf(tri)) {
        const gds = detectGridCandidates(lab);
        if (!gds.length) continue;
        const pe = estimatePerspectiveH(gray, img.width, img.height, lab, gds[0].spec.cells, true);
        for (let gi = 0; gi < gds.length; gi++) {
          interps.push({ lab, gd: gds[gi], fit: pe ? pe.fit : -Infinity });
        }
      }
    }
    interps.sort((a, b) => b.fit - a.fit);

    let lastError = null;
    for (const it of interps.slice(0, (opts && opts.fast) ? 2 : 6)) {
      const lab = it.lab, gd = it.gd, cells = gd.spec.cells;
      const coarse = warpToCanonical(img, gray, lab, cells);
      if (!coarse) continue;
      // 후보 워프 순서: 원근 추정 H(+정렬패턴 보정) → coarse 아핀 경로(§D.2 보정
      // 포함) 폴백.
      const candidates = [];
      const pe = estimatePerspectiveH(gray, img.width, img.height, lab, cells);
      if (pe) {
        // 외곽 4변 실측 4코너 H — 가장 정밀. 측정→H 개선의 2회 반복.
        let edgeH = refineHomographyWithEdges(gray, img.width, img.height, lab, cells, pe.H);
        if (edgeH) {
          const e2 = refineHomographyWithEdges(gray, img.width, img.height, lab, cells, edgeH);
          if (e2) edgeH = e2;
          candidates.push(warpImageWithH(gray, img.width, img.height, edgeH, coarse.size));
        }
        const refP = refineHomographyWithAlignment(gray, img.width, img.height, lab, cells, pe.H);
        if (refP) candidates.push(warpImageWithH(gray, img.width, img.height, refP, coarse.size));
        candidates.push(warpImageWithH(gray, img.width, img.height, pe.H, coarse.size));
      }
      const refC = refineHomographyWithAlignment(gray, img.width, img.height, lab, cells, coarse.H);
      if (refC) candidates.push(warpImageWithH(gray, img.width, img.height, refC, coarse.size));
      candidates.push(coarse.warped);

      for (const warped of candidates) {
        const r = _decodeWarped(warped, coarse.size, gd, t0);
        if (r && r.success) return { result: r };
        if (r && r.lastError) lastError = r.lastError;
      }
    }
    return { errors: ['all offsets × ratios × channels failed', lastError ? 'last: ' + lastError : null].filter(Boolean) };
  }

  // 한 워프 이미지에 대해 7오프셋 × 2채널 × 4ECC 전수 시도 (CRC 정합 게이트).
  function _decodeWarped(warped, size, gd, t0) {
    const cells = gd.spec.cells;
    const warpThr = otsu(warped);
    let lastError = null;
    for (const [ox, oy] of OFFSETS_7) {
      const grays = sampleCellGrays(warped, size, cells, ox, oy);
      for (const bitsPerCell of [1, 2]) {                       // 흑백 우선, 그다음 컬러
        const dataBytes = extractBytes(grays, gd.grade, bitsPerCell, warpThr);
        const rawBytes = Math.floor((dataCellCount(gd.grade) * bitsPerCell) / 8);
        for (const ratio of ECC_FROM_BITS) {
          let plan;
          try { plan = planBlocks(rawBytes, ratio); } catch (e) { lastError = e.message; continue; }
          if (dataBytes.length < plan.totalN) continue;
          const rawStream = dataBytes.subarray(0, plan.totalN);
          // §6.1.3 두 직렬화 시도 (0x01 역인터리브 우선). 헤더 version 일치가 게이트.
          for (const ver of [VERSION, VERSION_FLAT]) {
            try {
              const seq = ver === VERSION ? deinterleaveBytes(rawStream, plan) : rawStream;
              const dec = rsDecodeAll(seq, plan);
              const ph = parseHeader(dec.data.subarray(0, HEADER_LEN));
              if (!ph.ok) { lastError = ph.error; continue; }
              const H = ph.header;
              if (H.version !== ver) { lastError = 'version/serialization mismatch'; continue; }
              if (H.eccRatio !== ratio) { lastError = 'ratio mismatch'; continue; }
              // 컬러 일관성: 헤더 color 플래그 ↔ 시도한 bitsPerCell
              if ((H.flags.color ? 2 : 1) !== bitsPerCell) { lastError = 'color/bitsPerCell mismatch'; continue; }
              const payload = dec.data.subarray(HEADER_LEN, HEADER_LEN + H.payloadLen);
              if (crc32(payload) !== H.payloadCrc32) { lastError = 'payload CRC32 mismatch'; continue; }

              let parsed;
              if (H.mode === 0x01) parsed = parseStatic(payload);
              else if (H.mode === 0x02) parsed = parseDynamic(payload);
              else parsed = parseWiaBook(payload);
              if (!parsed.ok) { lastError = parsed.error; continue; }

              return {
                success: true, mode: H.modeName, data: parsed.data,
                reliability: dec.errors === 0 ? 1.0 : Math.max(0.5, 1.0 - dec.errors * 0.05),
                source: 'on-device', errors: [],
                meta: {
                  grid: gd.grade, color: H.flags.color, color_variant: H.colorVariant, trojan: H.flags.trojan,
                  ecc_used: H.eccRatio, ecc_errors_corrected: dec.errors, decode_time_ms: Date.now() - t0,
                  header: {
                    version: H.version, flags: H.flags, payload_length: H.payloadLen,
                    payload_crc32: '0x' + H.payloadCrc32.toString(16), header_crc16: '0x' + H.headerCrc16.toString(16),
                  },
                },
              };
            } catch (e) { lastError = e.message; continue; }
          }
        }
      }
    }
    return { success: false, lastError: lastError };
  }

  // 트로이 QR 폴백 — opts.decodeQR(grayU8, width, height) 주입 시 사용 (jsQR 등).
  function tryQRFallback(gray, img, t0, errors, opts, fail) {
    if (opts && typeof opts.decodeQR === 'function') {
      try {
        const text = opts.decodeQR(gray, img.width, img.height);
        if (text) return {
          success: true, mode: 'qr_fallback', data: { url: text, qr_text: text }, reliability: 0.8,
          source: 'on-device', errors,
          meta: { grid: null, color: false, trojan: true, ecc_used: null, decode_time_ms: Date.now() - t0 },
        };
      } catch (e) { errors = [...errors, 'qr fallback error: ' + e.message]; }
    }
    return fail(errors);
  }

  // ──────────────────────────────────────────────────────────────────────
  // 10.4 화면 스트리밍 (부록 §C.1 · data_type 0x70) — 프레임 캐러셀 송수신.
  //      단일 코드의 인쇄 밀도 한계를 시간축으로 우회. 순서 무관·유실 내성(반복 순환).
  // ──────────────────────────────────────────────────────────────────────
  const STREAM_TYPE = 0x70;
  function streamChunkCapacity(grid, ecc) {
    const raw = Math.floor(dataCellCount(grid) / 8);
    const plan = planBlocks(raw, ecc);
    return plan.totalK - HEADER_LEN - 2 - 13;        // 헤더16 + static(type/enc)2 + 조각헤더13
  }
  function encodeStreamFrames(content, opts) {
    opts = opts || {};
    const grid = opts.grid || 'M', ecc = opts.ecc || '25%';
    const isText = typeof content === 'string';
    const bytes = isText ? utf8(content) : content;
    if (!(bytes instanceof Uint8Array) || !bytes.length) throw new Error('stream content must be non-empty string|Uint8Array');
    const cap = streamChunkCapacity(grid, ecc);
    const total = Math.ceil(bytes.length / cap);
    if (total > 65535) throw new Error('content too large for stream (' + total + ' chunks)');
    const sid = crc32(bytes);
    const frames = [];
    for (let i = 0; i < total; i++) {
      const part = bytes.subarray(i * cap, Math.min(bytes.length, (i + 1) * cap));
      const d = new Uint8Array(13 + part.length);
      d[0] = (sid >>> 24) & 255; d[1] = (sid >>> 16) & 255; d[2] = (sid >>> 8) & 255; d[3] = sid & 255;
      d[4] = (i >> 8) & 255; d[5] = i & 255;
      d[6] = (total >> 8) & 255; d[7] = total & 255;
      d[8] = (bytes.length >>> 24) & 255; d[9] = (bytes.length >>> 16) & 255; d[10] = (bytes.length >>> 8) & 255; d[11] = bytes.length & 255;
      d[12] = isText ? 0x01 : 0x02;
      d.set(part, 13);
      frames.push(encode({ mode: 'static', grid, payload: { data_type: STREAM_TYPE, encoding: 'binary', data: d }, flags: { ecc } }));
    }
    return { frames, stream_id: sid, chunk_total: total, total_bytes: bytes.length, chunk_bytes: cap };
  }
  function parseStreamChunk(staticData) {
    if (!staticData || staticData.data_type_id !== STREAM_TYPE) return null;
    const d = staticData.raw;
    if (!d || d.length < 13) return null;
    const sid = ((d[0] << 24) | (d[1] << 16) | (d[2] << 8) | d[3]) >>> 0;
    const index = (d[4] << 8) | d[5], total = (d[6] << 8) | d[7];
    const totalBytes = ((d[8] << 24) | (d[9] << 16) | (d[10] << 8) | d[11]) >>> 0;
    if (!total || index >= total) return null;
    return { stream_id: sid, index, total, total_bytes: totalBytes, content_type: d[12], bytes: d.subarray(13) };
  }
  // 수신 조립기 — decode 결과를 add() 로 먹이면 진행률/완성 통지. 완성 시 CRC32 무결성 확정.
  function createStreamRx() {
    let sid = null, chunks = null, got = 0, meta = null;
    return {
      add(decResult) {
        if (!decResult || !decResult.success || decResult.mode !== 'static') return null;
        const ch = parseStreamChunk(decResult.data);
        if (!ch) return null;
        if (sid !== ch.stream_id) { sid = ch.stream_id; chunks = new Array(ch.total).fill(null); got = 0; meta = ch; }
        if (!chunks[ch.index]) { chunks[ch.index] = ch.bytes; got++; }
        const done = got === meta.total;
        let content = null, ok = false;
        if (done) {
          const out = new Uint8Array(meta.total_bytes); let off = 0;
          for (const c of chunks) { const n = Math.min(c.length, meta.total_bytes - off); out.set(c.subarray(0, n), off); off += n; }
          ok = off === meta.total_bytes && crc32(out) === sid;
          content = ok ? (meta.content_type === 0x01 ? fromUtf8(out) : out) : null;
        }
        return { stream_id: sid, received: got, total: meta.total, total_bytes: meta.total_bytes, content_type: meta.content_type, done, ok, content };
      },
      reset() { sid = null; chunks = null; got = 0; meta = null; },
    };
  }

  // ──────────────────────────────────────────────────────────────────────
  // 10.5 QR Version 3 인코더 (트로이 가운데 QR · 무의존 · GF(256) RS 재사용)
  //      스펙 §7 — 콜드스타트: 일반 QR 스캐너가 WIA Code 를 인식해 안내 페이지로.
  //      nayuki 표준 알고리즘 기반. 29×29 모듈, 바이트 모드, V3 무 version-info.
  // ──────────────────────────────────────────────────────────────────────
  const QR_SIZE = 29, QR_VERSION = 3;
  // V3 EC 테이블 (ISO/IEC 18004)
  const QR_V3 = {
    L: { ec: 15, blocks: [{ count: 1, k: 55 }] },
    M: { ec: 26, blocks: [{ count: 1, k: 44 }] },
    Q: { ec: 18, blocks: [{ count: 2, k: 17 }] },
    H: { ec: 22, blocks: [{ count: 2, k: 13 }] },
  };
  const QR_FORMAT_ECL = { L: 1, M: 0, Q: 3, H: 2 };
  // §7.3 short_id — Crockford base32(상위30비트)
  const CROCKFORD = '0123456789ABCDEFGHJKMNPQRSTVWXYZ';
  function shortIdFromCrc32(crc) {
    const u = crc >>> 0, top30 = u >>> 2; let out = '';
    for (let i = 5; i >= 0; i--) out += CROCKFORD[(top30 >>> (i * 5)) & 0x1F];
    return out;
  }
  function trojanUrl(crc) { return 'https://wiacode.com/r/' + shortIdFromCrc32(crc); }

  function qrDataCodewords(text, ecl) {
    const spec = QR_V3[ecl]; if (!spec) throw new Error('QR ecl must be L/M/Q/H');
    const bytes = utf8(text);
    const totalK = spec.blocks.reduce((s, b) => s + b.count * b.k, 0);
    const bits = [];
    const push = (val, len) => { for (let i = len - 1; i >= 0; i--) bits.push((val >> i) & 1); };
    push(0b0100, 4); push(bytes.length, 8);               // 바이트 모드 + 길이(8b, V1-9)
    if (4 + 8 + bytes.length * 8 > totalK * 8) throw new Error(`QR V3 ${ecl} 용량 초과 (${bytes.length}B)`);
    for (const b of bytes) push(b, 8);
    const cap = totalK * 8;
    for (let i = 0, t = Math.min(4, cap - bits.length); i < t; i++) bits.push(0);   // 종단자
    while (bits.length % 8) bits.push(0);
    const cw = []; for (let i = 0; i < bits.length; i += 8) { let v = 0; for (let j = 0; j < 8; j++) v = (v << 1) | bits[i + j]; cw.push(v); }
    const pad = [0xEC, 0x11]; let pi = 0; while (cw.length < totalK) cw.push(pad[pi++ & 1]);
    // 블록 분할 + RS + 인터리브
    const dBlocks = [], eBlocks = []; let off = 0;
    for (const grp of spec.blocks) for (let b = 0; b < grp.count; b++) {
      const blk = cw.slice(off, off + grp.k); off += grp.k;
      dBlocks.push(blk); eBlocks.push(Array.from(rsEncodeBlock(Uint8Array.from(blk), spec.ec)));
    }
    const res = [];
    const maxK = Math.max(...dBlocks.map(b => b.length));
    for (let i = 0; i < maxK; i++) for (const b of dBlocks) if (i < b.length) res.push(b[i]);
    const maxE = Math.max(...eBlocks.map(b => b.length));
    for (let i = 0; i < maxE; i++) for (const b of eBlocks) if (i < b.length) res.push(b[i]);
    const out = []; for (const c of res) for (let i = 7; i >= 0; i--) out.push((c >> i) & 1);
    return out; // V3 remainder bits = 0
  }

  function buildQRv3(text, ecl) {
    ecl = ecl || 'M';
    const dataBits = qrDataCodewords(text, ecl);
    const N = QR_SIZE;
    const mods = Array.from({ length: N }, () => new Array(N).fill(false));
    const fn = Array.from({ length: N }, () => new Array(N).fill(false));
    const setFn = (r, c, v) => { mods[r][c] = v; fn[r][c] = true; };
    // 핀더 + separator
    function finder(r0, c0) {
      for (let dr = -1; dr <= 7; dr++) for (let dc = -1; dc <= 7; dc++) {
        const r = r0 + dr, c = c0 + dc; if (r < 0 || r >= N || c < 0 || c >= N) continue;
        let dark = false;
        if (dr >= 0 && dr <= 6 && dc >= 0 && dc <= 6) {
          if (dr === 0 || dr === 6 || dc === 0 || dc === 6) dark = true;
          else if (dr >= 2 && dr <= 4 && dc >= 2 && dc <= 4) dark = true;
        }
        setFn(r, c, dark);
      }
    }
    finder(0, 0); finder(0, N - 7); finder(N - 7, 0);
    // 타이밍 패턴
    for (let i = 8; i < N - 8; i++) { setFn(6, i, i % 2 === 0); setFn(i, 6, i % 2 === 0); }
    // 정렬 패턴 (V3: 중심 (22,22))
    (function (cr, cc) {
      for (let dr = -2; dr <= 2; dr++) for (let dc = -2; dc <= 2; dc++) {
        const ring = Math.max(Math.abs(dr), Math.abs(dc));
        setFn(cr + dr, cc + dc, ring !== 1);
      }
    })(N - 7, N - 7);
    // dark 모듈 (4V+9, 8) = (21,8)
    setFn(N - 8, 8, true);
    // 포맷 영역 예약 (값은 마스크 후)
    for (let i = 0; i <= 8; i++) { if (i !== 6) { fn[8][i] = true; fn[i][8] = true; } }
    for (let i = 0; i < 8; i++) { fn[8][N - 1 - i] = true; fn[N - 1 - i][8] = true; }

    // 데이터 배치 함수 (마스크별로 새 행렬에)
    function place(mask) {
      const m = mods.map(row => row.slice());
      let bi = 0, dir = -1;
      for (let right = N - 1; right >= 1; right -= 2) {
        if (right === 6) right = 5;
        for (let vert = 0; vert < N; vert++) {
          for (let j = 0; j < 2; j++) {
            const col = right - j;
            const upward = ((right + 1) & 2) === 0;
            const row = upward ? N - 1 - vert : vert;
            if (fn[row][col]) continue;
            let dark = bi < dataBits.length ? !!dataBits[bi++] : false;
            // 마스크 적용
            let inv;
            switch (mask) {
              case 0: inv = (row + col) % 2 === 0; break;
              case 1: inv = row % 2 === 0; break;
              case 2: inv = col % 3 === 0; break;
              case 3: inv = (row + col) % 3 === 0; break;
              case 4: inv = (Math.floor(row / 2) + Math.floor(col / 3)) % 2 === 0; break;
              case 5: inv = (row * col) % 2 + (row * col) % 3 === 0; break;
              case 6: inv = ((row * col) % 2 + (row * col) % 3) % 2 === 0; break;
              default: inv = ((row + col) % 2 + (row * col) % 3) % 2 === 0; break;
            }
            m[row][col] = inv ? !dark : dark;
          }
        }
      }
      return m;
    }
    function drawFormat(m, mask) {
      // nayuki 좌표는 (x=col, y=row): setFunctionModule(x,y) → modules[y][x].
      const data = (QR_FORMAT_ECL[ecl] << 3) | mask;
      let rem = data; for (let i = 0; i < 10; i++) rem = (rem << 1) ^ (((rem >> 9) & 1) * 0x537);
      const bits = ((data << 10) | rem) ^ 0x5412;
      const get = (i) => !!((bits >>> i) & 1);
      // 첫 복사본 (TL 핀더 주변)
      for (let i = 0; i <= 5; i++) m[i][8] = get(i);
      m[7][8] = get(6); m[8][8] = get(7); m[8][7] = get(8);
      for (let i = 9; i < 15; i++) m[8][14 - i] = get(i);
      // 둘째 복사본 (TR/BL)
      for (let i = 0; i < 8; i++) m[8][N - 1 - i] = get(i);
      for (let i = 8; i < 15; i++) m[N - 15 + i][8] = get(i);
      m[N - 8][8] = true; // 항상 dark
    }
    function penalty(m) {
      let p = 0;
      // 규칙1: 같은색 5+ 연속 (행·열)
      for (let r = 0; r < N; r++) for (const ax of [0, 1]) {
        let run = 1, prev = ax ? m[0][r] : m[r][0];
        for (let i = 1; i < N; i++) { const v = ax ? m[i][r] : m[r][i]; if (v === prev) { run++; if (run === 5) p += 3; else if (run > 5) p++; } else { run = 1; prev = v; } }
      }
      // 규칙2: 2×2 동색
      for (let r = 0; r < N - 1; r++) for (let c = 0; c < N - 1; c++) { const v = m[r][c]; if (v === m[r][c + 1] && v === m[r + 1][c] && v === m[r + 1][c + 1]) p += 3; }
      // 규칙3: 1:1:3:1:1 + 4 light 패턴
      const pat = [true, false, true, true, true, false, true];
      for (let r = 0; r < N; r++) for (let c = 0; c < N; c++) for (const ax of [0, 1]) {
        const at = (i) => { if (ax) { const rr = r + i; return (rr >= 0 && rr < N) ? m[rr][c] : null; } const cc = c + i; return (cc >= 0 && cc < N) ? m[r][cc] : null; };
        let ok = true; for (let i = 0; i < 7; i++) if (at(i) !== pat[i]) { ok = false; break; }
        if (!ok) continue;
        let l1 = true; for (let i = -4; i < 0; i++) { const v = at(i); if (v !== false && v !== null) { l1 = false; break; } }
        let l2 = true; for (let i = 7; i < 11; i++) { const v = at(i); if (v !== false && v !== null) { l2 = false; break; } }
        if (l1 || l2) p += 40;
      }
      // 규칙4: 어두운 비율 편차
      let dark = 0; for (let r = 0; r < N; r++) for (let c = 0; c < N; c++) if (m[r][c]) dark++;
      const pct = dark * 100 / (N * N);
      p += Math.floor(Math.abs(pct - 50) / 5) * 10;
      return p;
    }
    // 8 마스크 중 최소 페널티 선택
    let best = null, bestP = Infinity, bestMask = 0;
    for (let mask = 0; mask < 8; mask++) {
      const m = place(mask); drawFormat(m, mask);
      const p = penalty(m);
      if (p < bestP) { bestP = p; best = m; bestMask = mask; }
    }
    // 1=dark Uint8Array 반환
    const data = new Uint8Array(N * N);
    for (let r = 0; r < N; r++) for (let c = 0; c < N; c++) data[r * N + c] = best[r][c] ? 1 : 0;
    return { size: N, data, mask: bestMask, ecl };
  }

  // 트로이 합성: levels(셀강도)의 가운데에 QR(29×29) + frame(31×31 흰 separator) 덮어쓰기
  function trojanRect(G) {
    const q0 = Math.floor(G / 2) - Math.floor(QR_SIZE / 2);   // §7.2: G/2 - 14
    return { q0, q1: q0 + QR_SIZE - 1, f0: q0 - 1, f1: q0 + QR_SIZE };
  }
  function overlayTrojan(levels, G, url, ecl) {
    const qr = buildQRv3(url, ecl), r = trojanRect(G);
    for (let cr = r.f0; cr <= r.f1; cr++) for (let cc = r.f0; cc <= r.f1; cc++) levels[cr * G + cc] = 255; // separator
    for (let i = 0; i < QR_SIZE; i++) for (let j = 0; j < QR_SIZE; j++)
      levels[(r.q0 + i) * G + (r.q0 + j)] = qr.data[i * QR_SIZE + j] ? 0 : 255;
    return qr;
  }

  // ──────────────────────────────────────────────────────────────────────
  // 11. 공개 API
  // ──────────────────────────────────────────────────────────────────────
  return {
    // 메타
    VERSION: 'v1', SPEC: 'WIA3/v0', MAGIC, CELL, GRIDS, CHANNEL_LEVELS,
    STATIC_DATA_TYPES, STATIC_ENCODINGS,
    // 인코드
    encode, toImageData, renderToCanvas, encodeToCanvas,
    // 디코드
    decode,
    // 화면 스트리밍 (§C.1)
    encodeStreamFrames, parseStreamChunk, createStreamRx, streamChunkCapacity,
    // 용량 조회 (UI용)
    capacity(grid, ecc, color) {
      const bpc = color ? 2 : 1;
      const raw = Math.floor((dataCellCount(grid) * bpc) / 8);
      const plan = planBlocks(raw, ecc || DEFAULT_ECC.static);
      return { grid, ecc: ecc || DEFAULT_ECC.static, color: !!color, data_cells: dataCellCount(grid),
               capacity_bytes: plan.totalK, payload_bytes: plan.totalK - HEADER_LEN, rs_blocks: plan.blocks.length };
    },
    // 트로이 (§7)
    buildQRv3, shortIdFromCrc32, trojanUrl,
    // 저수준 (테스트/확장용)
    _internal: {
      crc16, crc32, planBlocks, rsEncodeAll, rsDecodeAll, parseHeader, serializeHeader,
      dataCellOrder, dataCellCount, findFinders, sortFinders, sortFindersRanked, plausibleTriples, labelingsOf,
      detectGrid, detectGridCandidates, otsu, toGray, binarize,
      buildQRv3, overlayTrojan,
      computeHomography, warpToCanonical, warpImageWithH, refineHomographyWithAlignment, _decodeWarped,
      solveHWithPerspective, finderPatternFitness, estimatePerspectiveH, refineFinderCenter, sampleGrayBilinear,
      refineHomographyWithEdges, interleaveBytes, deinterleaveBytes, blurGray,
    },
  };
});

});

__def("codec", function(module, exports){
'use strict';
/*
 * ============================================================================
 *  WIA Code v2 — 오르빗 데이터 계층 (payload ↔ 데이터셀 비트)
 * ============================================================================
 *  설계 전제("코덱은 기하와 분리") 실행: v1 의 검증된 바이트 코덱(Reed-Solomon
 *  오류정정·인터리브·CRC)을 _internal 로 그대로 계승하고, 픽셀 배치만 오르빗
 *  레이아웃(geometry.dataCells 순서)으로 바꾼다. QR 파인더 목발 없이 데이터를
 *  싣고 읽는다.
 *
 *  프레임: [0x57 'W', 0x01 ver, lenHi, lenLo] + payload(UTF-8) + crc16(2)
 *  이후 RS(ECC) + 인터리브 → 데이터셀 비트(MSB-first) → geometry.render(opts.bits)
 * ============================================================================
 */
const V1 = require('../v1/wiacode.js');
const I = V1._internal;
const GEO = require('./geometry.js');
const { applyH } = require('./degrade.js');

const ECC = '25%';                 // 오류정정 비율(카메라 비트오류 흡수)
const MAGIC = 0x57, VER = 0x01;

// 결정적 화이트닝(스크램블): RS가 systematic이라 zero-padding→대부분 0바이트→대부분 흰 셀.
//   그러면 다단계 분류의 퍼센타일(lo/hi)이 skew돼 검정/중간단계를 놓친다. XOR 마스크로
//   코드워드를 균등분포화 → 셀 레벨 균등 → 캘리브레이션 견고 + 버스트오류 분산. XOR=자기역.
function scramble(bytes) {
  let s = 0x9e3779b9 >>> 0;
  const out = new Uint8Array(bytes.length);
  for (let i = 0; i < bytes.length; i++) { s = (Math.imul(s, 1664525) + 1013904223) >>> 0; out[i] = bytes[i] ^ ((s >>> 16) & 0xff); }
  return out;
}

function toBytes(str) {
  if (typeof TextEncoder !== 'undefined') return new TextEncoder().encode(str);
  return Uint8Array.from(Buffer.from(str, 'utf8'));
}
function fromBytes(u8) {
  if (typeof TextDecoder !== 'undefined') return new TextDecoder().decode(u8);
  return Buffer.from(u8).toString('utf8');
}

// ── 프레임(헤더+길이+CRC) ─────────────────────────────────────────────────
function frameEncode(payload) {
  const len = payload.length;
  const body = new Uint8Array(4 + len);
  body[0] = MAGIC; body[1] = VER; body[2] = (len >>> 8) & 255; body[3] = len & 255;
  body.set(payload, 4);
  const crc = I.crc16(body) & 0xffff;
  const out = new Uint8Array(body.length + 2);
  out.set(body, 0); out[body.length] = (crc >>> 8) & 255; out[body.length + 1] = crc & 255;
  return out;
}
function frameDecode(bytes) {
  if (bytes.length < 6 || bytes[0] !== MAGIC || bytes[1] !== VER) return null;
  const len = (bytes[2] << 8) | bytes[3];
  if (4 + len + 2 > bytes.length) return null;
  const body = bytes.subarray(0, 4 + len);
  const crc = I.crc16(body) & 0xffff;
  const got = (bytes[4 + len] << 8) | bytes[4 + len + 1];
  if (crc !== got) return null;
  return body.subarray(4, 4 + len);
}

// nCells(데이터셀 수) → RS 계획(인코드/디코드 공용, 결정적)
function planFor(nCells) { return I.planBlocks(Math.floor(nCells / 8), ECC); }

// payload 문자열 → 데이터셀 비트배열(길이 nCells). capacity 초과 시 예외.
function encodeToBits(text, nCells) {
  const frame = frameEncode(toBytes(text));
  const plan = planFor(nCells);
  if (frame.length > plan.totalK) throw new Error('용량초과: ' + frame.length + 'B > ' + plan.totalK + 'B');
  const dataIn = new Uint8Array(plan.totalK); dataIn.set(frame, 0);   // 나머지 0패딩
  let cw = I.rsEncodeAll(dataIn, plan);
  cw = scramble(I.interleaveBytes(cw, plan));
  const bits = new Uint8Array(nCells);
  const nb = Math.min(cw.length * 8, nCells), pos = buildPerm(nCells);
  for (let k = 0; k < nb; k++) bits[pos[k]] = (cw[k >> 3] >>> (7 - (k & 7))) & 1;   // 위치 산포
  return { bits: bits, plan: plan, usedBytes: frame.length, capBytes: plan.totalK };
}

// 데이터셀 비트배열 → payload 문자열(오류정정 포함). 실패 시 {ok:false}.
function decodeFromBits(bits, nCells) {
  const plan = planFor(nCells);
  const totalN = plan.totalN;
  const cw = new Uint8Array(totalN);
  const nb = Math.min(totalN * 8, bits.length), pos = buildPerm(bits.length);
  for (let k = 0; k < nb; k++) if (bits[pos[k]]) cw[k >> 3] |= (1 << (7 - (k & 7)));   // 위치 산포 역
  const seq = I.deinterleaveBytes(scramble(cw), plan);
  let res;
  try { res = I.rsDecodeAll(seq, plan); }
  catch (e) { return { ok: false, reason: 'rs-uncorrectable' }; }
  const payload = frameDecode(res.data);
  if (!payload) return { ok: false, reason: 'bad-frame/crc', errors: res.errors };
  return { ok: true, text: fromBytes(payload), errors: res.errors };
}

// ── 이미지에서 데이터셀 비트 읽기 (locate 의 Hmod2img 사용) ─────────────────
//   각 데이터셀 중심(모듈) → 이미지(H) → 작은 원판 평균 그레이 → Otsu 임계로 0/1.
function grayAt(G, x, y) {
  const g = G.g, w = G.w, h = G.h;
  if (x < 0 || y < 0 || x >= w - 1 || y >= h - 1) return 255;
  const x0 = Math.floor(x), y0 = Math.floor(y), fx = x - x0, fy = y - y0, i = y0 * w + x0;
  return (g[i] * (1 - fx) + g[i + 1] * fx) * (1 - fy) + (g[i + w] * (1 - fx) + g[i + w + 1] * fx) * fy;
}
function sampleCellGrays(grayObj, H, layout, cellPx) {
  const dc = GEO.dataCells(layout);
  const grays = new Float32Array(dc.length);
  // 셀 내부 작은 원판(반경 ~0.3모듈) 다중샘플 평균 — 블러/저해상도 강건.
  const offs = [[0, 0], [0.22, 0], [-0.22, 0], [0, 0.22], [0, -0.22]];
  for (let i = 0; i < dc.length; i++) {
    const mx = dc[i][0] + 0.5, my = dc[i][1] + 0.5;
    let s = 0, n = 0;
    for (const o of offs) { const p = applyH(H, mx + o[0], my + o[1]); s += grayAt(grayObj, p[0], p[1]); n++; }
    grays[i] = s / n;
  }
  return { grays: grays, cells: dc };
}
// Otsu 임계 (0~255 히스토그램)
function otsu(grays) {
  const hist = new Float64Array(256);
  for (const v of grays) hist[Math.max(0, Math.min(255, Math.round(v)))]++;
  const total = grays.length; let sum = 0; for (let t = 0; t < 256; t++) sum += t * hist[t];
  let sumB = 0, wB = 0, maxVar = -1, tLo = 128, tHi = 128;
  for (let t = 0; t < 256; t++) {
    wB += hist[t]; if (wB === 0) continue; const wF = total - wB; if (wF === 0) break;
    sumB += t * hist[t];
    const mB = sumB / wB, mF = (sum - sumB) / wF, v = wB * wF * (mB - mF) * (mB - mF);
    // 완전 양극단(깨끗한 렌더: 그레이 0/255 만, 중간 빈 다수)에선 분산이 넓은 평탄부에서
    //   동일 최대 → 그 평탄부의 '중앙'을 임계로. maxVar 갱신=평탄부 시작, 동률=끝 연장.
    if (v > maxVar * (1 + 1e-9)) { maxVar = v; tLo = t; tHi = t; }
    else if (v >= maxVar * (1 - 1e-9)) { tHi = t; }
  }
  return Math.round((tLo + tHi) / 2);
}

// ── 다단계(그레이) 데이터 셀 (Fable5 설계) ─────────────────────────────────
//   셀당 bpc 비트 = 2^bpc 그레이 단계. QR(흑백 1비트)을 용량에서 넘는 구조적 레버.
//   ·팔레트: 밝기 rank 0=흰(255) … max=검(0), 등간격. levelGray(rank).
//   ·그레이코드: 인접 밝기단계 오독 = 1비트 오류(RS 정정) → symbol=grayEnc, 역=grayDec.
//   ·캘리브레이션: 코드의 알려진 앵커(코어·위성=검, 도넛구멍·흰링·콰이엇=흰)로 매프레임
//     흑점/백점 역산 → 조명 무관. ·모드판별: RS/CRC 가 통과하는 bpc 를 trial-decode.
const PAL_ECC = { 1: '25%', 2: '50%', 3: '50%' };     // 단계 많을수록 노이즈↑ → ECC 상향(원근 far쪽 강건)
function nlevOf(bpc) { return 1 << bpc; }
function gcd(a, b) { while (b) { const t = a % b; a = b; b = t; } return a; }
// 셀 위치 산포 permutation. 바이트=인접 4셀이라 원근 far쪽 오류가 바이트로 뭉쳐(5%셀→20%바이트)
//   ECC 초과. 공간 인접 셀을 비트스트림서 멀리 흩으면 각 바이트의 셀이 서로 다른 구역 → 바이트
//   오류≈셀오류. pos[k]=k번째 데이터유닛의 공간위치(큰 coprime 승수 = 결정적 선형 permutation).
const _permCache = {};
function buildPerm(n) {
  if (_permCache[n]) return _permCache[n];
  let A = Math.floor(n * 0.6180339887) | 1;      // 황금비 근처 홀수
  while (gcd(A, n) !== 1) A += 2;
  const pos = new Int32Array(n);
  for (let k = 0; k < n; k++) pos[k] = (k * A) % n;
  return (_permCache[n] = pos);
}
function levelGray(rank, nlev) { return Math.round(255 * (1 - rank / (nlev - 1))); } // rank0=흰..max=검
function grayEnc(v) { return v ^ (v >> 1); }          // symbol → 밝기 rank (binary→Gray)
function grayDec(r) { let v = 0; for (; r > 0; r >>= 1) v ^= r; return v; }

// payload → 데이터셀 심볼(0..2^bpc-1) + 렌더용 그레이(cellGray). bpc=1 은 기존 흑백과 동일.
function encodeToCells(text, nCells, bpc, ecc) {
  bpc = bpc || 1;
  const frame = frameEncode(toBytes(text));
  const rawBytes = Math.floor(nCells * bpc / 8);
  const plan = I.planBlocks(rawBytes, ecc || PAL_ECC[bpc] || '25%');
  if (frame.length > plan.totalK) throw new Error('용량초과: ' + frame.length + 'B > ' + plan.totalK + 'B');
  const dataIn = new Uint8Array(plan.totalK); dataIn.set(frame, 0);
  const cw = scramble(I.interleaveBytes(I.rsEncodeAll(dataIn, plan), plan));
  const totalBits = cw.length * 8, getBit = (i) => i < totalBits ? (cw[i >> 3] >>> (7 - (i & 7))) & 1 : 0;
  const nlev = nlevOf(bpc), symbols = new Uint8Array(nCells), cellGray = new Uint8ClampedArray(nCells);
  const pos = buildPerm(nCells);                 // k번째 데이터유닛 → 산포된 공간위치 pos[k]
  let bit = 0;
  for (let k = 0; k < nCells; k++) {
    let v = 0; for (let b = 0; b < bpc; b++) v = (v << 1) | getBit(bit++);
    const p = pos[k]; symbols[p] = v; cellGray[p] = levelGray(grayEnc(v), nlev);
  }
  return { cells: symbols, cellGray: cellGray, plan: plan, usedBytes: frame.length, capBytes: plan.totalK, bitsPerCell: bpc };
}

// 알려진 앵커로 흑점/백점 캘리브레이션(전역 M1). 반환 {b0,w0}.
function calibPointsOf(layout) {
  if (layout.calibPoints) return layout.calibPoints;       // 모양이 명시(주로 round)
  const N = layout.N, c = layout.coreCenter.mx, d = (layout.spec.satInset || 4.5);
  const black = [[c, c]]; for (const a of layout.anchors) if (a.type === 'disk') black.push([a.mx, a.my]);
  const white = [[d, d], [c + 2.5, c], [-2, -2], [N + 2, -2], [N + 2, N + 2], [-2, N + 2]];
  return { black, white };
}
function calibrate(grayObj, H, layout) {
  const smp = (mx, my) => { const p = applyH(H, mx, my); return grayAt(grayObj, p[0], p[1]); };
  const cp = calibPointsOf(layout);
  const mean = (arr) => { let s = 0; for (const p of arr) s += smp(p[0], p[1]); return s / arr.length; };
  return { b0: mean(cp.black), w0: mean(cp.white) };
}

// 퍼센타일 lo/hi.
function pct(arr, p) { const s = Float32Array.from(arr).sort(); return s[Math.max(0, Math.min(s.length - 1, Math.floor(p * s.length)))]; }

// 셀 그레이 → 심볼. 흑/백 범위를 "공간 구역별(K×K)" 데이터 분포에서 잡는다(Fable M2).
//   원근서 far쪽이 압축·어두워져 전역 lo/hi 하나론 그쪽 오분류(far 33%). 구역별 lo/hi 로
//   그쪽만의 범위 → far 오류 33%→11%. 도트분포 퍼센타일은 blur/조명서도 균등간격 보존(affine).
//   scramble 로 각 구역도 레벨 균등이라 구역 퍼센타일이 견고. cells=데이터셀 좌표, layout 필요.
function classifyCells(grays, cells, bpc, layout) {
  const nlev = nlevOf(bpc), N = layout.N, K = 6;
  const gLo = pct(grays, 0.02), gHi = pct(grays, 0.98);
  const blocks = Array.from({ length: K * K }, () => []);
  const bidx = (i) => (Math.min(K - 1, Math.floor(cells[i][1] / N * K)) * K + Math.min(K - 1, Math.floor(cells[i][0] / N * K)));
  for (let i = 0; i < grays.length; i++) blocks[bidx(i)].push(grays[i]);
  const lh = blocks.map(a => a.length < 8 ? null : [pct(a, 0.02), pct(a, 0.98)]);
  const sym = new Uint8Array(grays.length);
  for (let i = 0; i < grays.length; i++) {
    const x = lh[bidx(i)] || [gLo, gHi];
    const lo = x[0], span = (x[1] - x[0]) || 1;
    let t = (grays[i] - lo) / span; t = t < 0 ? 0 : t > 1 ? 1 : t;
    sym[i] = grayDec(Math.round((1 - t) * (nlev - 1)));
  }
  return { sym, lo: gLo, hi: gHi, span: gHi - gLo };
}

// 심볼(bpc비트/셀) → 바이트 → RS 디코드 → payload.
function decodeFromCells(symbols, nCells, bpc, ecc) {
  const plan = I.planBlocks(Math.floor(nCells * bpc / 8), ecc || PAL_ECC[bpc] || '25%');
  const totalN = plan.totalN, cw = new Uint8Array(totalN), nb = totalN * 8;
  const pos = buildPerm(nCells);                 // 위치 산포 역변환: 데이터유닛 k = symbols[pos[k]]
  let bit = 0;
  for (let k = 0; k < nCells && bit < nb; k++) {
    const v = symbols[pos[k]];
    for (let b = bpc - 1; b >= 0 && bit < nb; b--) { if ((v >> b) & 1) cw[bit >> 3] |= (1 << (7 - (bit & 7))); bit++; }
  }
  const seq = I.deinterleaveBytes(scramble(cw), plan);
  let res; try { res = I.rsDecodeAll(seq, plan); } catch (e) { return { ok: false, reason: 'rs-uncorrectable' }; }
  const payload = frameDecode(res.data);
  if (!payload) return { ok: false, reason: 'bad-frame/crc', errors: res.errors };
  return { ok: true, text: fromBytes(payload), errors: res.errors };
}

// ── 컬러(hue) 레이어 (Fable5 설계, luma×hue 곱코드) ────────────────────────
//   밝기 Y=luma 그대로(toGray 무손상), 색(hue)만 Cb/Cr 에 별도 RS 평면으로. K=2=파랑/노랑축(Cb).
//   hue 는 "중간밝기 셀"만 탑승(검/흰은 색 헤드룸 없음). eligible 은 디코드된 루마로 결정.
const HUE_ECC = '50%', HUE_RHO = 72;   // 채도. 실폰 카메라가 채도를 깎으므로 진하게(gamut 헤드룸 내).
function rgbAtC(img, x, y) {           // 바이리니어 RGB 샘플
  const w = img.width, h = img.height, d = img.data;
  if (x < 0 || y < 0 || x >= w - 1 || y >= h - 1) return [255, 255, 255];
  const x0 = Math.floor(x), y0 = Math.floor(y), fx = x - x0, fy = y - y0, i = (y0 * w + x0) * 4, j = i + w * 4;
  const s = (a, b) => (d[a] * (1 - fx) + d[a + 4] * fx) * (1 - fy) + (d[b] * (1 - fx) + d[b + 4] * fx) * fy;
  return [s(i, j), s(i + 1, j + 1), s(i + 2, j + 2)];
}
function rgb2cbcr(R, G, B) { return [-0.168736 * R - 0.331264 * G + 0.5 * B, 0.5 * R - 0.418688 * G - 0.081312 * B]; }
function hueChroma(sym, hueBits) {
  if (hueBits === 1) return sym === 0 ? [HUE_RHO, 0] : [-HUE_RHO, 0];    // 0=파랑(+Cb) 1=노랑(-Cb)
  const pts = [[HUE_RHO, 0], [-HUE_RHO, 0], [0, HUE_RHO], [0, -HUE_RHO]];  // K=4: +Cr=빨강 -Cr=초록(색맹 비안전)
  return pts[sym & 3];
}
// 루마 심볼(셀별) → hue 자격 셀 인덱스(밝기 rank 극단 제외).
function eligibleFromLuma(lumaSymbols, bpc) {
  const nlev = nlevOf(bpc), out = [];
  for (let i = 0; i < lumaSymbols.length; i++) { const rank = grayEnc(lumaSymbols[i]); if (rank > 0 && rank < nlev - 1) out.push(i); }
  return out;
}
// 각 데이터셀 중심 RGB 평균(타이트 디스크 — 4:2:0 색압축이 셀 안에 머물게).
function sampleCellColor(rgbaImg, H, layout) {
  const dc = GEO.dataCells(layout), rgb = new Array(dc.length);
  const offs = [[0, 0], [0.12, 0], [-0.12, 0], [0, 0.12], [0, -0.12]];
  for (let i = 0; i < dc.length; i++) {
    const mx = dc[i][0] + 0.5, my = dc[i][1] + 0.5; let R = 0, G = 0, B = 0;
    for (const o of offs) { const p = applyH(H, mx + o[0], my + o[1]), c = rgbAtC(rgbaImg, p[0], p[1]); R += c[0]; G += c[1]; B += c[2]; }
    rgb[i] = [R / offs.length, G / offs.length, B / offs.length];
  }
  return { rgb: rgb, cells: dc };
}
// 흑/백 앵커 RGB → 채널별 화이트밸런스 아핀 {Ck[3], Cw[3]}.
function calibrateColor(rgbaImg, H, layout) {
  const N = layout.N, c = layout.coreCenter.mx, d = (layout.spec.satInset || 4.5);
  const smp = (mx, my) => { const p = applyH(H, mx, my); return rgbAtC(rgbaImg, p[0], p[1]); };
  const blk = [[c, c]]; for (const a of layout.anchors) if (a.type === 'disk') blk.push([a.mx, a.my]);
  const wht = [[d, d], [c + 2.5, c], [-2, -2], [N + 2, -2], [N + 2, N + 2], [-2, N + 2]];
  const mean = (arr) => { const s = [0, 0, 0]; for (const p of arr) { const rc = smp(p[0], p[1]); s[0] += rc[0]; s[1] += rc[1]; s[2] += rc[2]; } return s.map(x => x / arr.length); };
  return { Ck: mean(blk), Cw: mean(wht) };
}
// eligible 셀 색 → hue 심볼(화이트밸런스 + 구역별 무채색 기준 + Cb 부호/축 최근접).
function classifyHue(rgbAll, eligibleIdx, cells, lumaSymbols, bpc, cc, layout, hueBits) {
  const N = layout.N, K = 6, nlev = nlevOf(bpc);
  const gain = [0, 1, 2].map(k => 255 / ((cc.Cw[k] - cc.Ck[k]) || 1));
  const cb = new Float32Array(rgbAll.length), cr = new Float32Array(rgbAll.length);
  for (let i = 0; i < rgbAll.length; i++) {
    const R = (rgbAll[i][0] - cc.Ck[0]) * gain[0], G = (rgbAll[i][1] - cc.Ck[1]) * gain[1], B = (rgbAll[i][2] - cc.Ck[2]) * gain[2];
    const v = rgb2cbcr(R, G, B); cb[i] = v[0]; cr[i] = v[1];
  }
  // 구역별 무채색 기준 = 그 구역의 hue-ineligible(극단밝기) 셀 CbCr 평균.
  const bidx = (i) => (Math.min(K - 1, (cells[i][1] / N * K) | 0) * K + Math.min(K - 1, (cells[i][0] / N * K) | 0));
  const nCb = new Float64Array(K * K), nCr = new Float64Array(K * K), nn = new Float64Array(K * K);
  for (let i = 0; i < cells.length; i++) { const rank = grayEnc(lumaSymbols[i]); if (rank === 0 || rank === nlev - 1) { const b = bidx(i); nCb[b] += cb[i]; nCr[b] += cr[i]; nn[b]++; } }
  const gCb = nCb.reduce((a, b) => a + b, 0) / (nn.reduce((a, b) => a + b, 0) || 1), gCr = nCr.reduce((a, b) => a + b, 0) / (nn.reduce((a, b) => a + b, 0) || 1);
  const sym = new Uint8Array(eligibleIdx.length);
  for (let e = 0; e < eligibleIdx.length; e++) {
    const i = eligibleIdx[e], b = bidx(i);
    const bx = nn[b] >= 4 ? nCb[b] / nn[b] : gCb, by = nn[b] >= 4 ? nCr[b] / nn[b] : gCr;
    const x = cb[i] - bx, y = cr[i] - by;
    if (hueBits === 1) sym[e] = x >= 0 ? 0 : 1;               // 파랑/노랑
    else { const pts = [[1, 0], [-1, 0], [0, 1], [0, -1]]; let best = 0, bd = -1e9; for (let k = 0; k < 4; k++) { const dot = x * pts[k][0] + y * pts[k][1]; if (dot > bd) { bd = dot; best = k; } } sym[e] = best; }
  }
  return sym;
}
// 컬러 인코드: 루마 payload + hue payload → {cellGray, cellChroma}.
function encodeColor(lumaText, hueText, nCells, bpc, hueBits) {
  const lu = encodeToCells(lumaText, nCells, bpc);
  const elig = eligibleFromLuma(lu.cells, bpc);
  const hu = encodeToCells(hueText, elig.length, hueBits, HUE_ECC);
  const cellChroma = new Array(nCells).fill(null);
  for (let e = 0; e < elig.length; e++) cellChroma[elig[e]] = hueChroma(hu.cells[e], hueBits);
  return { cellGray: lu.cellGray, cellChroma: cellChroma, bitsPerCell: bpc, hueBits: hueBits,
           eligible: elig.length, lumaCapBytes: lu.capBytes - 6, hueCapBytes: hu.capBytes - 6 };
}

// 종합: grayObj + locate 결과(res.Hmod2img) → 디코드. 앵커 캘리브레이션 + bpc trial-decode.
//   opts.bpcTry: 시도할 bpc 목록(기본 [1,2,3]). opts.hue=true + rgbaImg → hue 평면도 디코드.
function readCode(grayObj, res, layout, cellPx, opts) {
  if (!res || !res.ok || !res.Hmod2img) return { ok: false, reason: 'no-lock' };
  const s = sampleCellGrays(grayObj, res.Hmod2img, layout, cellPx);
  let tries = (opts && opts.bpcTry) || [1, 2, 3];
  for (const bpc of tries) {
    const cl = classifyCells(s.grays, s.cells, bpc, layout);
    if (cl.hi - cl.lo < 25) continue;                    // 대비붕괴 → 이 모드 스킵
    const out = decodeFromCells(cl.sym, s.cells.length, bpc);
    if (out.ok) {
      out.bitsPerCell = bpc; out.range = { lo: Math.round(cl.lo), hi: Math.round(cl.hi) };
      // ── hue 평면(있으면) — 루마 성공 후에만. 실패해도 루마 결과는 그대로 반환(우아한 열화). ──
      if (opts && opts.hue && opts.rgbaImg) {
        try {
          const hueBits = opts.hueBits || 1;
          const canon = encodeToCells(out.text, s.cells.length, bpc);   // 오류정정된 정규 루마심볼
          const elig = eligibleFromLuma(canon.cells, bpc);
          const col = sampleCellColor(opts.rgbaImg, res.Hmod2img, layout);
          const cc = calibrateColor(opts.rgbaImg, res.Hmod2img, layout);
          const hsym = classifyHue(col.rgb, elig, col.cells, canon.cells, bpc, cc, layout, hueBits);
          const hout = decodeFromCells(hsym, elig.length, hueBits, HUE_ECC);
          out.hue = hout.ok ? { text: hout.text, hueBits: hueBits, errors: hout.errors } : false;
        } catch (e) { out.hue = false; }
      }
      return out;
    }
  }
  return { ok: false, reason: 'all-modes-failed' };
}

module.exports = {
  ECC, encodeToBits, decodeFromBits, frameEncode, frameDecode, planFor, sampleCellGrays, otsu, readCode,
  encodeToCells, decodeFromCells, calibrate, classifyCells, levelGray, grayEnc, grayDec, PAL_ECC,
  encodeColor, eligibleFromLuma, sampleCellColor, calibrateColor, classifyHue, hueChroma, HUE_ECC, HUE_RHO,
};

});


// ===== WiaScan 실시간 API =====
var FR = require('./frst'), CN = require('./conic'), GEO = require('./geometry'), LOC = require('./locate'), DEG = require('./degrade');
var toGray = FR.toGray, frst = FR.frst, peaks = FR.peaks;
// 배율추정 전용 경량 디노이즈: 강한 노이즈가 코어 링에지를 가짜로 만들어 cellPx 를
//   과소추정하는 것 방지. 정밀 locate 는 원본 그레이(노이즈 강건) 사용.
function _denoiseGray(imageData){ var b = DEG.boxBlur({ data:imageData.data, width:imageData.width, height:imageData.height }, 1, 1); return toGray(b.img); }
var layout = GEO.layout, SPEC = GEO.SPEC, render = GEO.render;
var pickCoreSeed = LOC.pickCoreSeed, locateRobust = LOC.locateRobust;
var extractCoreRings = CN.extractCoreRings;
var CODEC = require('codec'), dataCells = GEO.dataCells;
function _now(){ return (typeof performance!=='undefined'&&performance.now)?performance.now():Date.now(); }

/*
 * detectAuto(imageData) — 코드의 픽셀 크기(cellPx)·격자(S/M/L)를 모르는 실사 프레임에서
 *   코어 불스아이로 배율을 역산하고 위치확정까지 한 번에.
 *   1) 스케일 무지 상태로 넓은 반경 FRST → 코어 후보 → pickCoreSeed
 *   2) 코어 링에지 추출(스케일 불변) → 바깥원 반경/5.5 = cellPx
 *   3) 추정 cellPx 로 정식 FRST → locateRobust(orbit+conic 폴백) → 격자 3종 시도, 최소 잔차 채택
 */
function detectAuto(imageData){
  var t0 = _now();
  var W = imageData.width, H = imageData.height, mn = Math.min(W,H);
  var gray = toGray(imageData);
  var grayD = _denoiseGray(imageData);         // 배율추정 전용(경량 디노이즈)
  // 1) 스케일 무지 coarse 코어탐색
  var coarse = [3,5,7,10,14,19,25,33].filter(function(r){ return r < 0.25*mn; });
  if (coarse.length < 2) coarse = [3,5,7,10];
  var Sc = frst(grayD, coarse, { gradFrac:0.10, alpha:2 });
  var pkC = peaks(Sc, { win: Math.max(6, Math.round(0.02*mn)), topK:24, thrFrac:0.05 });
  if (!pkC.length) return { ok:false, reason:'no-core', ms:Math.round(_now()-t0) };
  var seed = pickCoreSeed(grayD, pkC, 0.05*mn);
  if (!seed) return { ok:false, reason:'no-core-seed', ms:Math.round(_now()-t0) };
  // 2) 코어 링에지 → cellPx (디노이즈 그레이)
  var rings = extractCoreRings(grayD, seed.x, seed.y, 0.18*mn, 180);
  if (rings.outer.length < 20) return { ok:false, reason:'weak-core', ms:Math.round(_now()-t0) };
  var ro = rings.outer.map(function(p){ return Math.hypot(p[0]-seed.x, p[1]-seed.y); }).sort(function(a,b){ return a-b; });
  var outerMed = ro[ro.length>>1];
  var cellPx = outerMed / 5.5;                 // 코어 바깥원 = 5.5 모듈
  if (!(cellPx > 1.2 && cellPx < 60)) return { ok:false, reason:'bad-scale', cellPx:cellPx, ms:Math.round(_now()-t0) };
  // 3) 정식 검출 + 격자 시도
  var radii = [1.5,2.5,3.5,4.5,5.5].map(function(r){ return r*cellPx; });
  var fo = { gradFrac:0.10, alpha:2 }, po = { win: Math.max(4, Math.round(1.6*cellPx)), topK:30, thrFrac:0.03 };
  var S2 = frst(gray, radii, fo);
  var pk2 = peaks(S2, po);
  var redetect = function(im){ var g = toGray(im); var s = frst(g, radii, fo); return { gray:g, peaks:peaks(s, po) }; };
  // 격자 S/M/L × 앵커모양(사각-코너 / 링) 위치확정 → 후보(잔차 오름차순).
  //   round·heart 는 앵커가 동일(링) → locate 는 'round'로 1번, readCode 는 모양별.
  var cands = [];
  ['S','M','L'].forEach(function(grid){
    ['square','round'].forEach(function(sh){
      var lay = layout(grid, SPEC, sh);
      var res = locateRobust(imageData, gray, pk2, lay, { cellPx:cellPx, redetect:redetect });
      if (res.ok) cands.push({ grid:grid, anchorShape:sh, lay:lay, res:res });
    });
  });
  if (!cands.length) return { ok:false, reason:'no-lock', cellPx:+cellPx.toFixed(2), ms:Math.round(_now()-t0) };
  cands.sort(function(a,b){ return a.res.residPx - b.res.residPx; });
  // 격자·모양 확정 = "실제로 CRC 디코드되는" 조합. 링 앵커면 round·heart 둘 다 시도.
  var best = cands[0], decoded = null, decShape = 'square';
  for (var i = 0; i < cands.length && !decoded; i++) {
    var shapes = cands[i].anchorShape === 'square' ? ['square'] : ['round', 'heart'];
    for (var j = 0; j < shapes.length; j++) {
      var lay = shapes[j] === cands[i].anchorShape ? cands[i].lay : layout(cands[i].grid, SPEC, shapes[j]);
      var dec = CODEC.readCode(gray, cands[i].res, lay, cellPx, { hue: true, hueBits: 1, rgbaImg: imageData });
      if (dec && dec.ok) { decoded = dec; best = cands[i]; decShape = shapes[j]; break; }
    }
  }
  var r = best.res;
  return { ok:true, method:r.method, core:r.core, corners:r.corners,
           northStar:r.northStar||'TL', residPx:+(r.residPx||0).toFixed(2),
           cellPx:+cellPx.toFixed(2), grid:best.grid, shape: decoded ? decShape : null,
           decoded: !!decoded, text: decoded ? decoded.text : null, errors: decoded ? decoded.errors : null,
           bitsPerCell: decoded ? decoded.bitsPerCell : null,
           hue: (decoded && decoded.hue) ? decoded.hue.text : null,
           ms:Math.round(_now()-t0) };
}

// renderTestCode — 스캔할 샘플 코드 PNG(dataURL) + 실제 payload 삽입. 브라우저 canvas 필요.
//   hueText 주면 컬러(2luma+1hue) 코드로 렌더 → 스캔 시 루마+색 payload 둘 다 표시.
function renderTestCode(grid, cellPx, text, hueText){
  grid = grid || 'S'; cellPx = cellPx || 6;
  if (text == null) text = '홍익인간 · WIA CODE — 설치 없이 읽히는 차세대 코드';
  if (hueText == null) hueText = '🎨 색에 담긴 추가 데이터 (hue layer)';
  var opts = { grid:grid, cellPx:cellPx, quiet:4, seed:7, ss:2, data:true };
  try {
    var L = layout(grid, SPEC), nC = dataCells(L).length;
    var col = CODEC.encodeColor(text, hueText, nC, 2, 1);   // 2luma + 1hue 컬러코드
    opts.cellGray = col.cellGray; opts.cellChroma = col.cellChroma;
  } catch (e) {
    try { opts.bits = CODEC.encodeToBits(text, dataCells(layout(grid, SPEC)).length).bits; } catch (e2) {}
  }
  var r = render(opts);
  // DOM 캔버스 우선(동기 toDataURL 있음). OffscreenCanvas 는 toDataURL 이 없어 제외.
  if (typeof document === 'undefined')
    return { dataURL:null, width:r.width, height:r.height, text:text, raw:r.img };
  var cv = document.createElement('canvas'); cv.width = r.width; cv.height = r.height;
  var ctx = cv.getContext('2d');
  var id = ctx.createImageData(r.width, r.height); id.data.set(r.img.data); ctx.putImageData(id, 0, 0);
  return { dataURL: cv.toDataURL('image/png'), width:r.width, height:r.height, text:text };
}

// generate(opts) — 임의 payload → WIA 코드 PNG. opts:{text, hueText?, grid, cellPx, bpc}.
//   hueText 있으면 컬러(2luma+1hue), 없으면 흑백/그레이(bpc). 반환 {dataURL,width,height,bytes,...}.
function generate(opts){
  opts = opts || {};
  var grid = opts.grid || 'M', cellPx = opts.cellPx || 8, text = opts.text || '', bpc = opts.bpc || 2, shape = opts.shape || 'square';
  var L = layout(grid, SPEC, shape), nC = dataCells(L).length;
  var ropts = { grid:grid, shape:shape, cellPx:cellPx, quiet:4, seed:7, ss:2, data:true }, bytes = 0, capBytes = 0;
  try {
    if (opts.hueText) { var col = CODEC.encodeColor(text, opts.hueText, nC, bpc, 1); ropts.cellGray = col.cellGray; ropts.cellChroma = col.cellChroma; bytes = text.length + opts.hueText.length; capBytes = col.lumaCapBytes + col.hueCapBytes; }
    else if (bpc === 1) { var e1 = CODEC.encodeToBits(text, nC); ropts.bits = e1.bits; bytes = text.length; capBytes = e1.capBytes - 6; }
    else { var e = CODEC.encodeToCells(text, nC, bpc); ropts.cellGray = e.cellGray; bytes = text.length; capBytes = e.capBytes - 6; }
  } catch (err) { return { error: String((err && err.message) || err) }; }
  var r = render(ropts);
  if (typeof document === 'undefined') return { dataURL:null, width:r.width, height:r.height, bytes:bytes, capBytes:capBytes };
  var cv = document.createElement('canvas'); cv.width = r.width; cv.height = r.height;
  var ctx = cv.getContext('2d'); var id = ctx.createImageData(r.width, r.height); id.data.set(r.img.data); ctx.putImageData(id, 0, 0);
  return { dataURL: cv.toDataURL('image/png'), width:r.width, height:r.height, grid:grid, shape:shape, bpc:bpc, color:!!opts.hueText, bytes:bytes, capBytes:capBytes };
}

var G = (typeof window!=='undefined') ? window : globalThis;
G.WiaScan = { ready: Promise.resolve(true), detectAuto: detectAuto, renderTestCode: renderTestCode, generate: generate, _version:'proto-3-gen' };

})();
