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
function extractCoreRings(grayObj, cx, cy, rMaxPx, nAng, gradFrac) {
  const { g, w, h } = grayObj;
  nAng = nAng || 180;
  gradFrac = gradFrac != null ? gradFrac : 0.10;   // 대비-상대 에지임계(과거 절대 40 대체)
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
    // 대비-상대 임계: 블러로 절대 그래디언트가 낮아져도 이 프로파일 자신의 대비 대비로 판정.
    //   (과거 절대 40 은 블러 2px에서 코어 링을 20개 못 찾아 weak-core 로 컷 → 배율추정 실패)
    let vmin = Infinity, vmax = -Infinity;
    for (let i = 0; i < vs.length; i++) { if (vs[i] < vmin) vmin = vs[i]; if (vs[i] > vmax) vmax = vs[i]; }
    const gThr = Math.max(6, gradFrac * (vmax - vmin));
    // 상승에지 후보: dGray/dr 국소최대(+). 중심이 어두운지 확인(disk).
    const rises = [];                       // {r, mag}
    for (let k = 2; k < vs.length - 2; k++) {
      const gm1 = vs[k + 1] - vs[k - 1];    // 중심차분(상승=+)
      if (gm1 <= 0) continue;
      // 국소최대
      const gprev = vs[k] - vs[k - 2], gnext = vs[k + 2] - vs[k];
      if (gm1 >= gprev && gm1 >= gnext && gm1 > gThr) {
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

// ── 각도평균 방사 프로파일 (극단블러용 배율추정 기반) ──────────────────────
// extractCoreRings 는 "각 각도마다 상승에지 2개"를 요구한다. 블러 σ가 링 폭(2모듈)에
// 근접하면 안쪽 흰 링이 먼저 소멸(MTF≈0) → 각도별로 에지가 1개만 남아 링 수집이
// 0개로 붕괴한다(실측: box r6·3pass 에서 180각 중 0~1개). 그러나 코어 전체(반경 5.5모듈
// 암부 블롭)의 바깥 경계는 훨씬 저주파라 훨씬 오래 산다. 각도평균(180각)은 SNR을 √180배
// 올려 그 경계를 블러 r10 까지 안정적으로 남긴다 — 각도별 국소 그래디언트 판정이 죽는
// 지점에서도. 그래서 "에지 개수 세기" 대신 "평균 프로파일의 마지막 상승 반값교차".
function meanRadialProfile(grayObj, cx, cy, rMaxPx, step) {
  const { g, w, h } = grayObj;
  step = step || 0.5;
  const sampAt = (x, y) => {
    if (x < 0 || y < 0 || x >= w - 1 || y >= h - 1) return null;
    const x0 = Math.floor(x), y0 = Math.floor(y), fx = x - x0, fy = y - y0, i = y0 * w + x0;
    return (g[i] * (1 - fx) + g[i + 1] * fx) * (1 - fy) + (g[i + w] * (1 - fx) + g[i + w + 1] * fx) * fy;
  };
  const nR = Math.max(2, Math.floor(rMaxPx / step) + 1);
  const P = new Float64Array(nR);
  let used = 0;
  for (let k = 0; k < nR; k++) {
    const r = k * step;
    const nA = Math.max(16, Math.min(240, Math.round(2 * Math.PI * r)));
    let s = 0, n = 0;
    for (let a = 0; a < nA; a++) {
      const th = 2 * Math.PI * a / nA;
      const v = sampAt(cx + r * Math.cos(th), cy + r * Math.sin(th));
      if (v != null) { s += v; n++; }
    }
    if (n < nA * 0.6) break;                 // 화면 밖으로 절반 이상 나가면 절단
    P[k] = s / n; used = k + 1;
  }
  return { P: P.subarray(0, used), step, n: used };
}

/*
 * estimateCoreScale — 사전지식 없이 코어 불스아이 바깥경계(5.5모듈)를 각도평균
 *   프로파일에서 찾아 cellPx 를 역산. extractCoreRings(각도별 에지수집)가 극단블러에서
 *   붕괴한 뒤의 폴백 겸, 코어/위성 판별자(위성 블롭반경 2.5모듈 vs 코어 5.5모듈).
 *   반환 { ok, cellPx, rOuter, contrast, centerDark }.
 */
function estimateCoreScale(grayObj, cx, cy, rMaxPx, opts) {
  opts = opts || {};
  const prof = meanRadialProfile(grayObj, cx, cy, rMaxPx, opts.step || 0.5);
  const P = prof.P, n = prof.n, step = prof.step;
  if (n < 8) return { ok: false, reason: 'short-profile' };
  const srt = Array.from(P).sort((a, b) => a - b);
  const vmin = srt[0], vhi = srt[Math.floor(0.9 * (srt.length - 1))];
  const contrast = vhi - vmin;
  if (contrast < (opts.minContrast != null ? opts.minContrast : 12)) return { ok: false, reason: 'flat', contrast };
  const thr = vmin + 0.5 * contrast;
  // 마지막(가장 바깥) 상승 반값교차 = 코어 암부 블롭의 바깥 경계.
  //   블러가 안쪽 흰 링을 지워도 이 교차는 남는다. 서브샘플 선형보간.
  let rOuter = NaN;
  for (let k = 1; k < n; k++) {
    if (P[k - 1] < thr && P[k] >= thr) {
      const t = (thr - P[k - 1]) / ((P[k] - P[k - 1]) || 1);
      rOuter = (k - 1 + t) * step;
    }
  }
  if (!isFinite(rOuter) || rOuter < step * 2) return { ok: false, reason: 'no-crossing', contrast };
  const cellPx = rOuter / 5.5;                    // 코어 바깥 링 = 5.5 모듈
  // 중심이 실제로 어두운가(밝은 blob 오검출 배제)
  const centerDark = (P[0] < thr);
  return { ok: centerDark && cellPx > 1.2 && cellPx < 60, cellPx, rOuter, contrast, centerDark, profile: P };
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
  extractCoreRings, meanRadialProfile, estimateCoreScale, fitConic, recoverFromConcentric, rectifyHomography,
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

// ── 하트 실루엣: 고전 매끈한 파라메트릭 곡선(원+쐐기 이어붙이기 아님) ─────────
//   x=sin³t, y=13cos t−5cos2t−2cos3t−cos4t (전 구간 C∞ 매끈, 이음매 없음).
//   이전 원(로브)+선형쐐기 결합은 두 곡선이 만나는 지점에서 값은 같아도 기울기가
//   달라(원 쪽 기울기 -1.4대 쐐기 쪽 -0.5) 옆구리에 살짝 꺾인 자국(홀쭉한 자국)이 남았음 —
//   단일 매끈 곡선으로 바꿔 그 이음매 자체를 없앰. 720점 샘플 폴리곤 + point-in-polygon.
//   앵커 위치엔 무관(검출 원리와 분리, insideShape는 데이터 셀 마스크에만 관여).
var HEART_POLY = (function () {
  var N = 720, pts = [];
  for (var i = 0; i < N; i++) {
    var t = (i / N) * 2 * Math.PI;
    var x = Math.pow(Math.sin(t), 3);
    var yRaw = 13 * Math.cos(t) - 5 * Math.cos(2 * t) - 2 * Math.cos(3 * t) - Math.cos(4 * t);
    var y = -0.205 - 0.07088 * yRaw; // yRaw∈[-17,11.92] → ny∈[1.0,-1.05] (뾰족점 아래, 로브 위)
    pts.push([x, y]);
  }
  return pts;
})();
function pointInHeart(nx, ny) {
  var inside = false;
  for (var i = 0, j = HEART_POLY.length - 1; i < HEART_POLY.length; j = i++) {
    var xi = HEART_POLY[i][0], yi = HEART_POLY[i][1], xj = HEART_POLY[j][0], yj = HEART_POLY[j][1];
    if (((yi > ny) !== (yj > ny)) && (nx < (xj - xi) * (ny - yi) / (yj - yi) + xi)) inside = !inside;
  }
  return inside;
}

// ── 모양 마스크: 데이터 판의 실루엣 (검출 원리와 무관 — 앵커는 그대로) ────────
//   'round'=원판, 'heart'=하트, 그 외=전부(사각). 정규화 좌표(중심0, 반경 Rdata=1).
function insideShape(mx, my, L) {
  const c = L.coreCenter.mx, R = L.Rdata || (L.N / 2 - 0.8);
  const nx = (mx - c) / R, ny = (my - c) / R;
  if (L.shape === 'round') return nx * nx + ny * ny <= 1;
  if (L.shape === 'heart') return pointInHeart(nx, ny);
  // clover/star/hex: 4개 위성이 항상 상/우/하/좌(0°,90°,180°,270°) 고정이므로, 이 세 모양의
  // 4중 대칭축도 같은 각도에 맞춰(|cos(2θ)|류) 위성이 잎/꼭짓점 자리에 정확히 올라가게 한다.
  if (L.shape === 'clover') {
    // 잎 4개 = 카디널 방향으로 띄운 원 4개의 합집합(둥근 잎 모양) + 중심 허브(연결 보장).
    // lobeR=0.50(잎이 서로 붙어 하나의 사각 꽃봉오리처럼 보임, 실사용자 지적)에서 0.33으로 줄여
    // 잎 사이에 뚜렷한 잘록함을 만듦 — 허브가 여전히 4개 잎을 중앙에서 이어준다.
    const lobeD = 0.55, lobeR = 0.33, hubR = 0.30;
    if ((nx * nx + ny * ny) <= hubR * hubR) return true;
    const centers = [[0, -lobeD], [lobeD, 0], [0, lobeD], [-lobeD, 0]];
    for (const [cx, cy] of centers) {
      const dx = nx - cx, dy = ny - cy;
      if (dx * dx + dy * dy <= lobeR * lobeR) return true;
    }
    return false;
  }
  if (L.shape === 'boomerang') {
    // 옛 'star'(4꼭짓점) — 진짜 5각별을 새로 넣으면서 이 모양은 이름 그대로 부메랑으로 개명.
    // 4개 꼭짓점(위성 자리) + 4개 안쪽 오목점을 잇는 실제 폴리곤(대칭은 유지, 이름만 정정).
    const Ro = 1.0, Ri = 0.24;
    const th = Math.atan2(ny, nx);
    const sect = Math.PI / 2, half = sect / 2;
    let a = ((th + half) % sect + sect) % sect - half;
    const phi = Math.abs(a);
    const x1 = Ro, y1 = 0, x2 = Ri * Math.cos(half), y2 = Ri * Math.sin(half);
    const dx = x2 - x1, dy = y2 - y1;
    const rr = (x1 * dy - y1 * dx) / (Math.cos(phi) * dy - Math.sin(phi) * dx);
    return (nx * nx + ny * ny) <= rr * rr;
  }
  if (L.shape === 'star') {
    // 진짜 5각별(꼭짓점 5개, 오각 대칭) — 위성 4개는 4중 대칭이라 별의 5개 꼭짓점과는 애초에
    // 안 맞는다(5와 4는 공약수가 1). 그래도 안전한 이유: 위성은 오목점(반경 Ri=0.42) 근방에
    // 걸려도 이 반경이 위성 예약반경(Rs, 대략 0.14)보다 훨씬 커서 어느 회전에서도 위성이
    // 별 몸통 밖으로 삐져나오지 않는다(reservedAt이 앵커존을 모양과 무관하게 먼저 예약).
    // rot=90°: 꼭짓점 하나가 정확히 하단 위성과 정렬 — 육안으로 "위가 넓고 아래가 뾰족한" 표준
    // 별 실루엣이 나오도록 고른 값(실측 비교 후 결정, Ri 0.24/0.5도 시험했으나 0.42가 가장 또렷).
    const Ro = 1.0, Ri = 0.42, m = 5, sect = 2 * Math.PI / m, half = sect / 2, rot = Math.PI / 2;
    const th = Math.atan2(ny, nx) - rot;
    let a = ((th + half) % sect + sect) % sect - half;
    const phi = Math.abs(a);
    const x1 = Ro, y1 = 0, x2 = Ri * Math.cos(half), y2 = Ri * Math.sin(half);
    const dx = x2 - x1, dy = y2 - y1;
    const rr = (x1 * dy - y1 * dx) / (Math.cos(phi) * dy - Math.sin(phi) * dx);
    return (nx * nx + ny * ny) <= rr * rr;
  }
  if (L.shape === 'hex') {
    const th = Math.atan2(ny, nx), m = 6, ro = 0.98;
    const sect = 2 * Math.PI / m;
    const a = ((th % sect) + sect) % sect;
    const rr = ro * Math.cos(Math.PI / m) / Math.cos(a - Math.PI / m);
    return (nx * nx + ny * ny) <= rr * rr;
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

  const vf = verifyLayout(grayObj, H, layout);   // residPx 는 grid 판별불가 → 독립 검증신호 동봉
  return { ok: true, core: coreF,
           corners: { TL: cornF.TL, TR: cornF.TR, BR: cornF.BR, BL: cornF.BL },
           northStar: 'TL', donutGray: +brightest.toFixed(0),
           Hmod2img: H, residPx: +resid.toFixed(2), dCen: +bestDCen.toFixed(3),
           modPx: vf.modPx, orbitMatch: vf.orbitMatch, orbitContrast: vf.orbitContrast };
}

/*
 * verifyLayout — 잠금(H)의 grid/cellPx 검증 신호 2종 (2026-08-10).
 *   residPx(5점 자기잔차)는 grid 판별력이 0 이다: S/M/L(및 사각↔링) 앵커 배치가
 *   전부 "중심+대칭4점" 닮음꼴이라, 8자유도 호모그래피가 스케일·회전을 통째로
 *   흡수해 어떤 layout 가정에도 같은 물리 4점에 거의 정확히 들어맞는다(실측:
 *   클린 합성서도 오답 grid residPx 0.00~0.25). → 앵커 밖의 독립 증거로 판별:
 *   · modPx      : H가 함의하는 코어 국소 모듈크기(px). 코어 링 실측 cellPx와
 *                  대조하면 스케일 흡수를 잡는다(오답 grid는 ≥36% 어긋남).
 *   · orbitMatch : 포맷 궤도 24도트 on/off 패턴의 재투영 일치율(0~1). 스케일이
 *                  거의 같은 근사합동 쌍(M-square↔L-round, 45° 회전 차)까지 판별.
 *                  실측: 정답 1.000, 오답 0.33~0.63 (실사진·노이즈σ45·블러r2·
 *                  0.5축소·yaw30 전부). 대비<30 이면 정보없음 → 0.5 반환.
 */
function verifyLayout(grayObj, H, layout) {
  const mx = layout.coreCenter.mx, my = layout.coreCenter.my;
  const a = applyH(H, mx, my), b = applyH(H, mx + 1, my), c = applyH(H, mx, my + 1);
  const modPx = (Math.hypot(b[0] - a[0], b[1] - a[1]) + Math.hypot(c[0] - a[0], c[1] - a[1])) / 2;
  let orbitMatch = 0.5, orbitContrast = 0;
  if (layout.orbit && layout.orbit.length) {
    const vals = layout.orbit.map(d => { const p = applyH(H, d.mx, d.my); return grayAt(grayObj, p[0], p[1]); });
    let mn = Infinity, mxv = -Infinity;
    for (const v of vals) { if (v < mn) mn = v; if (v > mxv) mxv = v; }
    orbitContrast = mxv - mn;
    if (orbitContrast >= 30) {
      const t = (mn + mxv) / 2;
      let m = 0;
      for (let i = 0; i < vals.length; i++) if ((vals[i] < t) === layout.orbit[i].on) m++;
      orbitMatch = m / vals.length;
    }
  }
  return { modPx: +modPx.toFixed(2), orbitMatch: +orbitMatch.toFixed(3), orbitContrast: Math.round(orbitContrast) };
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

  const vf = Hfin ? verifyLayout(grayObj, Hfin, layout) : { modPx: 0, orbitMatch: 0.5, orbitContrast: 0 };
  return { ok: true, method: 'conic', core: coreImg, corners: cornersImg,
           northStar: rres.northStar, Hmod2img: Hfin, residPx: +resid.toFixed(2),
           conicCenterErr: null, ringRatio: +rec.ringRatio.toFixed(2),
           modPx: vf.modPx, orbitMatch: vf.orbitMatch, orbitContrast: vf.orbitContrast };
}

module.exports = { locate, locateRobust, homographyLS, pickCoreSeed, verifyLayout };

});

__def("rs", function(module, exports){
'use strict';
/*
 * ============================================================================
 *  WIA Code — 순수 Reed–Solomon (GF(256)) + CRC-16 + 블록 플랜
 * ============================================================================
 *  독립 구현(외부 엔진 의존 0). 표준 알고리즘(GF(256) 0x11d, α=2; 다항식은
 *  최고차 우선; Berlekamp–Massey + Chien + Forney). codec.js 인터페이스 제공:
 *  crc16·planBlocks·rsEncodeAll·rsDecodeAll·interleaveBytes·deinterleaveBytes.
 * ============================================================================
 */
const EXP = new Uint8Array(512), LOG = new Uint8Array(256);
(function () { let x = 1; for (let i = 0; i < 255; i++) { EXP[i] = x; LOG[x] = i; x <<= 1; if (x & 0x100) x ^= 0x11d; } for (let i = 255; i < 512; i++) EXP[i] = EXP[i - 255]; })();
function mul(a, b) { return (a === 0 || b === 0) ? 0 : EXP[LOG[a] + LOG[b]]; }
function div(a, b) { if (a === 0) return 0; return EXP[(LOG[a] + 255 - LOG[b]) % 255]; }
function inv(a) { return EXP[255 - LOG[a]]; }
function pw(a, n) { return a === 0 ? 0 : EXP[(LOG[a] * ((n % 255) + 255)) % 255]; }

// 다항식(최고차 우선)
function pScale(p, s) { const r = new Array(p.length); for (let i = 0; i < p.length; i++) r[i] = mul(p[i], s); return r; }
function pAdd(a, b) { const r = new Array(Math.max(a.length, b.length)).fill(0); for (let i = 0; i < a.length; i++) r[i + r.length - a.length] = a[i]; for (let i = 0; i < b.length; i++) r[i + r.length - b.length] ^= b[i]; return r; }
function pMul(a, b) { const r = new Array(a.length + b.length - 1).fill(0); for (let i = 0; i < a.length; i++) for (let j = 0; j < b.length; j++) r[i + j] ^= mul(a[i], b[j]); return r; }
function pEval(p, x) { let y = p[0]; for (let i = 1; i < p.length; i++) y = mul(y, x) ^ p[i]; return y; }

const GENC = {};
function genPoly(nsym) {
  if (GENC[nsym]) return GENC[nsym];
  let g = [1]; for (let i = 0; i < nsym; i++) g = pMul(g, [1, EXP[i]]);
  return (GENC[nsym] = g);
}
function encodeBlock(msg, nsym) {                        // → parity(nsym)
  const g = genPoly(nsym), out = new Uint8Array(msg.length + nsym); out.set(msg, 0);
  for (let i = 0; i < msg.length; i++) { const c = out[i]; if (c !== 0) for (let j = 1; j < g.length; j++) out[i + j] ^= mul(g[j], c); }
  return out.subarray(msg.length);
}
function pDiv(dividend, divisor) {                        // → 나머지(remainder)
  const out = dividend.slice();
  for (let i = 0; i < dividend.length - (divisor.length - 1); i++) {
    const c = out[i]; if (c !== 0) for (let j = 1; j < divisor.length; j++) if (divisor[j] !== 0) out[i + j] ^= mul(divisor[j], c);
  }
  return out.slice(-(divisor.length - 1));
}
// 신드롬: [0, S0, S1, …, S_{nsym-1}] (선행 0 = reedsolo 규약)
function syndromes(msg, nsym) { const s = [0]; for (let i = 0; i < nsym; i++) s.push(pEval(msg, EXP[i])); return s; }
function anyNZ(a) { for (const v of a) if (v) return true; return false; }
function errLocator(synd, nsym) {                          // Berlekamp–Massey (선행0 synd)
  let errLoc = [1], oldLoc = [1];
  const shift = synd.length - nsym;
  for (let i = 0; i < nsym; i++) {
    const K = i + shift;
    let delta = synd[K];
    for (let j = 1; j < errLoc.length; j++) delta ^= mul(errLoc[errLoc.length - 1 - j], synd[K - j]);
    oldLoc = oldLoc.concat([0]);
    if (delta !== 0) {
      if (oldLoc.length > errLoc.length) { const nl = pScale(oldLoc, delta); oldLoc = pScale(errLoc, inv(delta)); errLoc = nl; }
      errLoc = pAdd(errLoc, pScale(oldLoc, delta));
    }
  }
  while (errLoc.length && errLoc[0] === 0) errLoc.shift();
  return errLoc;
}
function findErrors(errLoc, n) {                           // Chien
  const nerr = errLoc.length - 1, pos = [];
  for (let i = 0; i < n; i++) if (pEval(errLoc, EXP[(255 - i) % 255]) === 0) pos.push(n - 1 - i);
  if (pos.length !== nerr) throw new Error('rs: too many errors');
  return pos;
}
function correctErrata(msg, synd, pos) {                   // Forney (reedsolo 충실)
  const coefPos = pos.map(p => msg.length - 1 - p);
  let eLoc = [1]; for (const i of coefPos) eLoc = pMul(eLoc, pAdd([1], [pw(2, i), 0]));
  const sRev = synd.slice().reverse();
  const errEvalRev = pDiv(pMul(sRev, eLoc), [1].concat(new Array(eLoc.length).fill(0)));
  const errEval = errEvalRev.slice().reverse();
  const X = coefPos.map(cp => pw(2, cp - 0));
  for (let i = 0; i < X.length; i++) {
    const Xi = X[i], Xinv = inv(Xi);
    let prime = 1;
    for (let j = 0; j < X.length; j++) if (j !== i) prime = mul(prime, 1 ^ mul(Xinv, X[j]));
    let y = pEval(errEval.slice().reverse(), Xinv);
    y = mul(Xi, y);
    if (prime === 0) throw new Error('rs: forney zero');
    msg[pos[i]] ^= div(y, prime);
  }
}
function decodeBlock(cw, nsym) {
  const msg = new Uint8Array(cw), s = syndromes(msg, nsym);
  if (!anyNZ(s)) return { data: msg.subarray(0, msg.length - nsym), errors: 0 };
  const eloc = errLocator(s, nsym), pos = findErrors(eloc, msg.length);
  correctErrata(msg, s, pos);
  if (anyNZ(syndromes(msg, nsym))) throw new Error('rs: uncorrectable');
  return { data: msg.subarray(0, msg.length - nsym), errors: pos.length };
}

// ── 블록 플랜 + 전체 + 인터리브 (codec 인터페이스) ──────────────────────────
const ECC = { '25%': 0.25, '30%': 0.30, '35%': 0.35, '50%': 0.50 };
function planBlocks(rawBytes, ratioStr) {
  const ratio = ECC[ratioStr]; if (ratio === undefined) throw new Error('ECC?' + ratioStr);
  const b = Math.max(1, Math.ceil(rawBytes / 255));
  const ceilN = Math.ceil(rawBytes / b), floorN = Math.floor(rawBytes / b), cc = rawBytes - floorN * b;
  const blocks = []; let totalK = 0, totalN = 0;
  for (let i = 0; i < b; i++) { const n = i < cc ? ceilN : floorN, nsym = Math.round(n * ratio), k = n - nsym; if (k <= 0) throw new Error('블록 과소'); blocks.push({ n, k, nsym }); totalK += k; totalN += n; }
  return { blocks, totalK, totalN };
}
function rsEncodeAll(data, plan) {
  const dp = [], pp = []; let off = 0;
  for (const b of plan.blocks) { const s = data.subarray(off, off + b.k); off += b.k; dp.push(s); pp.push(encodeBlock(s, b.nsym)); }
  const out = new Uint8Array(plan.totalN); let p = 0;
  for (const s of dp) { out.set(s, p); p += s.length; } for (const s of pp) { out.set(s, p); p += s.length; }
  return out;
}
function rsDecodeAll(cw, plan) {
  const dp = [], pp = []; let off = 0;
  for (const b of plan.blocks) { dp.push(cw.subarray(off, off + b.k)); off += b.k; }
  for (const b of plan.blocks) { pp.push(cw.subarray(off, off + b.nsym)); off += b.nsym; }
  const out = new Uint8Array(plan.totalK); let oo = 0, errs = 0;
  for (let i = 0; i < plan.blocks.length; i++) { const b = plan.blocks[i], blk = new Uint8Array(b.k + b.nsym); blk.set(dp[i], 0); blk.set(pp[i], b.k); const r = decodeBlock(blk, b.nsym); out.set(r.data, oo); oo += b.k; errs += r.errors; }
  return { data: out, errors: errs };
}
function ilOrder(plan) {
  const o = [], ds = [], ps = []; let off = 0;
  for (const b of plan.blocks) { ds.push(off); off += b.k; } for (const b of plan.blocks) { ps.push(off); off += b.nsym; }
  let mK = 0, mS = 0; for (const b of plan.blocks) { if (b.k > mK) mK = b.k; if (b.nsym > mS) mS = b.nsym; }
  for (let i = 0; i < mK; i++) for (let k = 0; k < plan.blocks.length; k++) if (i < plan.blocks[k].k) o.push(ds[k] + i);
  for (let i = 0; i < mS; i++) for (let k = 0; k < plan.blocks.length; k++) if (i < plan.blocks[k].nsym) o.push(ps[k] + i);
  return o;
}
function interleaveBytes(seq, plan) { const o = ilOrder(plan), out = new Uint8Array(seq.length); for (let j = 0; j < o.length; j++) out[j] = seq[o[j]]; return out; }
function deinterleaveBytes(raw, plan) { const o = ilOrder(plan), out = new Uint8Array(raw.length); for (let j = 0; j < o.length; j++) out[o[j]] = raw[j]; return out; }
function crc16(buf) { let c = 0xffff; for (let i = 0; i < buf.length; i++) { c ^= buf[i] << 8; for (let k = 0; k < 8; k++) c = (c & 0x8000) ? ((c << 1) ^ 0x1021) & 0xffff : (c << 1) & 0xffff; } return c & 0xffff; }

module.exports = { crc16, planBlocks, rsEncodeAll, rsDecodeAll, interleaveBytes, deinterleaveBytes, encodeBlock, decodeBlock };

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
const I = require('./rs.js');            // 순수 표준 RS/CRC (v1 엔진 의존 제거)
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

// 원시 바이트 왕복(텍스트 아님) — encodeToCells/decodeFromCells과 완전히 동일한 파이프라인이지만
// toBytes(text)/fromBytes(payload)의 UTF-8 변환을 건너뛴다(임의 바이너리는 유효한 UTF-8이 아닐 수
// 있어 TextEncoder/Decoder를 거치면 손실·팽창됨 — 이미지 등 순수 바이트 페이로드 데모용, 2026-08-10 추가).
function encodeBytesToCells(bytes, nCells, bpc, ecc) {
  bpc = bpc || 1;
  const frame = frameEncode(bytes);
  const rawBytes = Math.floor(nCells * bpc / 8);
  const plan = I.planBlocks(rawBytes, ecc || PAL_ECC[bpc] || '25%');
  if (frame.length > plan.totalK) throw new Error('용량초과: ' + frame.length + 'B > ' + plan.totalK + 'B');
  const dataIn = new Uint8Array(plan.totalK); dataIn.set(frame, 0);
  const cw = scramble(I.interleaveBytes(I.rsEncodeAll(dataIn, plan), plan));
  const totalBits = cw.length * 8, getBit = (i) => i < totalBits ? (cw[i >> 3] >>> (7 - (i & 7))) & 1 : 0;
  const nlev = nlevOf(bpc), symbols = new Uint8Array(nCells), cellGray = new Uint8ClampedArray(nCells);
  const pos = buildPerm(nCells);
  let bit = 0;
  for (let k = 0; k < nCells; k++) {
    let v = 0; for (let b = 0; b < bpc; b++) v = (v << 1) | getBit(bit++);
    const p = pos[k]; symbols[p] = v; cellGray[p] = levelGray(grayEnc(v), nlev);
  }
  return { cells: symbols, cellGray: cellGray, plan: plan, usedBytes: frame.length, capBytes: plan.totalK, bitsPerCell: bpc };
}
function decodeBytesFromCells(symbols, nCells, bpc, ecc) {
  const plan = I.planBlocks(Math.floor(nCells * bpc / 8), ecc || PAL_ECC[bpc] || '25%');
  const totalN = plan.totalN, cw = new Uint8Array(totalN), nb = totalN * 8;
  const pos = buildPerm(nCells);
  let bit = 0;
  for (let k = 0; k < nCells && bit < nb; k++) {
    const v = symbols[pos[k]];
    for (let b = bpc - 1; b >= 0 && bit < nb; b--) { if ((v >> b) & 1) cw[bit >> 3] |= (1 << (7 - (bit & 7))); bit++; }
  }
  const seq = I.deinterleaveBytes(scramble(cw), plan);
  let res; try { res = I.rsDecodeAll(seq, plan); } catch (e) { return { ok: false, reason: 'rs-uncorrectable' }; }
  const payload = frameDecode(res.data);
  if (!payload) return { ok: false, reason: 'bad-frame/crc', errors: res.errors };
  return { ok: true, bytes: payload, errors: res.errors };
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
// 원시 바이트 버전(이미지 데모용) — 루마 채널에 lumaBytes, 색 채널에 hueBytes를 각자 독립된
// 바이트 스트림으로 싣는다(같은 이미지의 앞/뒷부분을 나눠 담는 방식 — 픽셀 단위 색변환 아님).
function encodeColorBytes(lumaBytes, hueBytes, nCells, bpc, hueBits) {
  const lu = encodeBytesToCells(lumaBytes, nCells, bpc);
  const elig = eligibleFromLuma(lu.cells, bpc);
  const hu = encodeBytesToCells(hueBytes, elig.length, hueBits, HUE_ECC);
  const cellChroma = new Array(nCells).fill(null);
  for (let e = 0; e < elig.length; e++) cellChroma[elig[e]] = hueChroma(hu.cells[e], hueBits);
  return { cellGray: lu.cellGray, cellChroma: cellChroma, bitsPerCell: bpc, hueBits: hueBits,
           eligible: elig.length, lumaCapBytes: lu.capBytes - 6, hueCapBytes: hu.capBytes - 6 };
}

// 종합: grayObj + locate 결과(res.Hmod2img) → 디코드. 앵커 캘리브레이션 + bpc trial-decode.
//   opts.bpcTry: 시도할 bpc 목록(기본 [1,2,3]). opts.hue=true + rgbaImg → hue 평면도 디코드.
// ── PSF-ISI 디코더 (블러 강건, Fable5 설계) ────────────────────────────────
//   블러는 랜덤이 아니라 결정적 선형채널(저역통과)이다. 셀 독립판독은 이웃 도트 번짐
//   (ISI=심볼간간섭)을 노이즈로 취급해 블러 2px부터 죽는다. 여기선 전체 모듈 그리드를
//   샘플해 가우시안 PSF로 역합성곱(Landweber 최소자승)한 뒤 기존 classify+decode 재사용.
//   PSF σ 는 코어피팅 대신 스윕 + CRC 게이트(trial-decode 철학) — 통과하는 σ 만 채택.
function sampleModuleGrid(grayObj, H, N) {
  const grid = new Float32Array(N * N);
  for (let my = 0; my < N; my++) for (let mx = 0; mx < N; mx++) {
    const p = applyH(H, mx + 0.5, my + 0.5); grid[my * N + mx] = grayAt(grayObj, p[0], p[1]);
  }
  return grid;
}
function gaussKernel1D(sigma) {
  const rad = Math.max(1, Math.ceil(3 * sigma)), k = new Float64Array(2 * rad + 1); let s = 0;
  for (let d = -rad; d <= rad; d++) { const v = Math.exp(-d * d / (2 * sigma * sigma)); k[d + rad] = v; s += v; }
  for (let i = 0; i < k.length; i++) k[i] /= s; return { k, rad };
}
function blurSep(src, N, K) {                       // 분리형 대칭 가우시안(경계 반사) — K=Kᵀ
  const k = K.k, rad = K.rad, tmp = new Float64Array(N * N), out = new Float64Array(N * N);
  for (let y = 0; y < N; y++) for (let x = 0; x < N; x++) { let s = 0; for (let d = -rad; d <= rad; d++) { let xx = x + d; if (xx < 0) xx = -xx; if (xx >= N) xx = 2 * N - 2 - xx; s += src[y * N + xx] * k[d + rad]; } tmp[y * N + x] = s; }
  for (let y = 0; y < N; y++) for (let x = 0; x < N; x++) { let s = 0; for (let d = -rad; d <= rad; d++) { let yy = y + d; if (yy < 0) yy = -yy; if (yy >= N) yy = 2 * N - 2 - yy; s += tmp[yy * N + x] * k[d + rad]; } out[y * N + x] = s; }
  return out;
}
function deconvGauss(obs, N, sigma, iters) {        // Landweber: T ← clamp(T + Kᵀ(O − K T))
  const K = gaussKernel1D(sigma), T = Float64Array.from(obs);
  for (let it = 0; it < iters; it++) {
    const KT = blurSep(T, N, K), resid = new Float64Array(N * N);
    for (let i = 0; i < N * N; i++) resid[i] = obs[i] - KT[i];
    const KtR = blurSep(resid, N, K);
    for (let i = 0; i < N * N; i++) { let v = T[i] + KtR[i]; T[i] = v < 0 ? 0 : v > 255 ? 255 : v; }
  }
  return T;
}
// 표준 셀판독이 블러로 실패했을 때 재시도(1비트). 통과 σ 반환, 실패 시 null.
function decodePSF(grayObj, H, layout, cells) {
  const N = layout.N, obs = sampleModuleGrid(grayObj, H, N), dc = new Float32Array(cells.length);
  for (const sig of [0.5, 0.75]) {
    const dec = deconvGauss(obs, N, sig, 45);
    for (let i = 0; i < cells.length; i++) dc[i] = dec[cells[i][1] * N + cells[i][0]];
    const cl = classifyCells(dc, cells, 1, layout);
    if (cl.hi - cl.lo < 25) continue;
    const out = decodeFromCells(cl.sym, cells.length, 1);
    if (out.ok) { out.psfSigma = sig; return out; }
  }
  return null;
}

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
  // 표준 셀판독 실패(주로 블러) → PSF-ISI 재시도(1비트). 우아한 열화: 안 되면 원래대로 실패.
  if (!(opts && opts.psf === false)) {
    const p = decodePSF(grayObj, res.Hmod2img, layout, s.cells);
    if (p && p.ok) { p.bitsPerCell = 1; return p; }
  }
  return { ok: false, reason: 'all-modes-failed' };
}

module.exports = {
  ECC, encodeToBits, decodeFromBits, frameEncode, frameDecode, planFor, sampleCellGrays, otsu, readCode,
  encodeToCells, decodeFromCells, calibrate, classifyCells, levelGray, grayEnc, grayDec, PAL_ECC,
  encodeColor, eligibleFromLuma, sampleCellColor, calibrateColor, classifyHue, hueChroma, HUE_ECC, HUE_RHO,
  encodeBytesToCells, decodeBytesFromCells, encodeColorBytes,
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
var extractCoreRings = CN.extractCoreRings, estimateCoreScale = CN.estimateCoreScale;
var CODEC = require('codec'), dataCells = GEO.dataCells;
function _now(){ return (typeof performance!=='undefined'&&performance.now)?performance.now():Date.now(); }

/*
 * detectAuto(imageData) — 코드의 픽셀 크기(cellPx)·격자(S/M/L)를 모르는 실사 프레임에서
 *   코어 불스아이로 배율을 역산하고 위치확정까지 한 번에.
 *   1) 스케일 무지 상태로 넓은 반경 FRST → 코어 후보 → pickCoreSeed
 *   2) 코어 링에지 추출(스케일 불변) → 바깥원 반경/5.5 = cellPx
 *   3) 추정 cellPx 로 정식 FRST → locateRobust(orbit+conic 폴백) → 격자 3종 시도, 최소 잔차 채택
 */
function detectAuto(imageData, dopts){
  var t0 = _now();
  var usePsf = !(dopts && dopts.psf === false);   // 라이브 스캐너는 프레임별로 끄고 주기적으로만 켬
  var deep = !(dopts && dopts.deep === false);    // 극단블러 배율추정 폴백(실패 프레임만 비용 발생)
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
  // 2) 배율추정 — 시도목록(seed,cellPx) 구성.
  //   2a) 빠른 경로: 코어 링에지(각도별 상승에지 2개) 중앙값. 정상~중블러에서 최선(무변경).
  //   2b) deep 폴백: 각도평균 프로파일의 마지막 반값교차(estimateCoreScale). 링에지가
  //       죽는 극단블러(box r6+)에서도 코어 바깥경계가 남아 배율을 준다. 동시에
  //       블롭반경(코어 5.5모듈 vs 위성 2.5모듈)이 "어느 피크가 코어인가"를 가려
  //       pickCoreSeed 가 위성을 고르는 r7+ 도 함께 복구한다.
  var attempts = [], failReason = null;
  var rings = extractCoreRings(grayD, seed.x, seed.y, 0.18*mn, 180);
  if (rings.outer.length >= 20) {
    var ro = rings.outer.map(function(p){ return Math.hypot(p[0]-seed.x, p[1]-seed.y); }).sort(function(a,b){ return a-b; });
    var c0 = ro[ro.length>>1] / 5.5;           // 코어 바깥원 = 5.5 모듈
    if (c0 > 1.2 && c0 < 60) attempts.push({ x:seed.x, y:seed.y, cellPx:c0, src:'rings' });
    else failReason = 'bad-scale';
  } else failReason = 'weak-core';
  if (deep) {
    var pool = [seed], pi;
    for (pi = 0; pi < pkC.length && pool.length < 9; pi++) {
      var pk = pkC[pi], dup = false;
      for (var pj = 0; pj < pool.length; pj++) if (Math.hypot(pool[pj].x-pk.x, pool[pj].y-pk.y) < 3) { dup = true; break; }
      if (!dup) pool.push(pk);
    }
    var ests = [], maxCt = 0;
    for (pi = 0; pi < pool.length; pi++) {
      var e = estimateCoreScale(grayD, pool[pi].x, pool[pi].y, 0.18*mn);
      if (!e.ok) continue;
      if (e.contrast > maxCt) maxCt = e.contrast;
      ests.push({ x:pool[pi].x, y:pool[pi].y, cellPx:e.cellPx, rOuter:e.rOuter, contrast:e.contrast, src:'profile' });
    }
    // 저대비 유령블롭 제거 후, 블롭반경 큰 순(코어가 위성보다 2배 이상 크다) 상위 3개.
    ests = ests.filter(function(a){ return a.contrast >= Math.max(12, 0.25*maxCt); })
               .sort(function(a,b){ return b.rOuter - a.rOuter; }).slice(0,3);
    for (pi = 0; pi < ests.length; pi++) {
      var dup2 = false;
      for (var aj = 0; aj < attempts.length; aj++) {
        var A = attempts[aj];
        if (Math.hypot(A.x-ests[pi].x, A.y-ests[pi].y) < 3 && Math.abs(A.cellPx-ests[pi].cellPx)/A.cellPx < 0.08) { dup2 = true; break; }
      }
      if (!dup2) attempts.push(ests[pi]);
    }
  }
  if (!attempts.length) return { ok:false, reason: failReason || 'weak-core', ms:Math.round(_now()-t0) };
  // 3) 정식 검출 + 격자 시도 (시도목록 순서대로).
  //   "잠금 인정" 기준 = ① CRC 해독 성공(=증명) 또는 ② 기하 잔차가 타이트(resid ≤ TIGHT·cellPx).
  //   실측(20시드×블러/노이즈/원근/저해상도): 진짜 잠금 resid/cellPx ≤0.17, 가짜 잠금 ≥0.71 로
  //   깨끗이 갈린다 → 0.5 게이트. 느슨한 잠금은 버리고 다음 배율후보로 계속(오검출 억제).
  var TIGHT = 0.5;
  var chosen = null, cands = [], cellPx = attempts[0].cellPx, tried = 0;
  var best = null, decoded = null, decShape = 'square';
  for (var ai = 0; ai < attempts.length && !decoded; ai++) {
    cellPx = attempts[ai].cellPx; tried++;
    var radii = [1.5,2.5,3.5,4.5,5.5].map(function(r){ return r*cellPx; });
    var fo = { gradFrac:0.10, alpha:2 }, po = { win: Math.max(4, Math.round(1.6*cellPx)), topK:30, thrFrac:0.03 };
    var S2 = frst(gray, radii, fo);
    var pk2 = peaks(S2, po);
    var redetect = (function(radii, fo, po){ return function(im){ var g = toGray(im); var s = frst(g, radii, fo); return { gray:g, peaks:peaks(s, po) }; }; })(radii, fo, po);
    // 격자 S/M/L × 앵커모양(사각-코너 / 링) 위치확정 → 후보(잔차 오름차순).
    //   round·heart 는 앵커가 동일(링) → locate 는 'round'로 1번, readCode 는 모양별.
    cands = [];
    ['S','M','L'].forEach(function(grid){
      ['square','round'].forEach(function(sh){
        var lay = layout(grid, SPEC, sh);
        var res = locateRobust(imageData, gray, pk2, lay, { cellPx:cellPx, redetect:redetect });
        if (res.ok) cands.push({ grid:grid, anchorShape:sh, lay:lay, res:res });
      });
    });
    if (!cands.length) continue;
    // ★격자/모양 판별 (2026-08-10 수정): residPx 는 판별력이 0 이다 — S/M/L(사각↔링)
    //   앵커 배치가 전부 닮음꼴이라 호모그래피가 스케일·회전을 흡수, 오답 layout 도
    //   같은 물리 4점에 잔차 0.0x 로 들어맞는다(클린 합성서도 동일, 실측 확인).
    //   → 앵커 밖 독립 증거로 순위 결정:
    //   · orbitMatch(궤도 24도트 패턴 일치율): 정답 1.0 / 오답 0.33~0.63 — 주신호.
    //   · scaleErr = |ln(modPx/cellPx)|: H-함의 모듈px vs 코어링 실측 cellPx —
    //     오답 grid 는 ≥0.31, 근사합동 쌍(M-square↔L-round)만 못 가름(궤도가 가름).
    //   · residPx 는 미세 타이브레이커로만.
    cands.forEach(function(c){
      c.scaleErr = c.res.modPx ? Math.abs(Math.log(c.res.modPx / cellPx)) : 1;
      var om = (typeof c.res.orbitMatch === 'number') ? c.res.orbitMatch : 0.5;
      c.score = (1 - om) * 2 + Math.min(c.scaleErr, 1) + c.res.residPx / (10 * cellPx);
    });
    cands.sort(function(a,b){ return a.score - b.score; });
    // Pass 1: 빠른 표준 셀판독(PSF 끔) — 모든 후보. 정상/약열화는 여기서 끝(빠름).
    for (var i = 0; i < cands.length && !decoded; i++) {
      var shapes = cands[i].anchorShape === 'square' ? ['square'] : ['round', 'heart'];
      for (var j = 0; j < shapes.length; j++) {
        var lay = shapes[j] === cands[i].anchorShape ? cands[i].lay : layout(cands[i].grid, SPEC, shapes[j]);
        var dec = CODEC.readCode(gray, cands[i].res, lay, cellPx, { hue: true, hueBits: 1, rgbaImg: imageData, psf: false });
        if (dec && dec.ok) { decoded = dec; best = cands[i]; decShape = shapes[j]; chosen = { cands:cands, cellPx:cellPx }; break; }
      }
    }
    if (decoded) break;
    if (!chosen) chosen = { cands:cands, cellPx:cellPx };       // 첫 잠금(느슨할 수 있음) 보관
    // 조기종료는 "스케일도 맞는" 타이트 잠금만: residPx 는 오답 cellPx 가정에도 0.0x
    //   로 나와(자기잔차) 잘못된 배율후보에서 탐색을 끊던 구멍을 scaleErr 로 막음.
    if (cands[0].res.residPx <= TIGHT * cellPx && cands[0].scaleErr <= 0.25) { chosen = { cands:cands, cellPx:cellPx }; break; }
  }
  if (!chosen) return { ok:false, reason:'no-lock', cellPx:+cellPx.toFixed(2), tried:tried, ms:Math.round(_now()-t0) };
  cands = chosen.cands; cellPx = chosen.cellPx;
  // Pass 2: 전부 실패(주로 블러) → PSF-ISI 폴백. 잔차 최소가 정답격자가 아닐 수 있어
  //   (블러 하 오답격자가 spurious 저잔차) → CRC가 격자를 판별하도록 후보 순회, 성공시 조기종료.
  if (!decoded && usePsf) {
    for (var i2 = 0; i2 < cands.length && !decoded; i2++) {
      var sh2 = cands[i2].anchorShape === 'square' ? ['square'] : ['round', 'heart'];
      for (var j2 = 0; j2 < sh2.length; j2++) {
        var lay2 = sh2[j2] === cands[i2].anchorShape ? cands[i2].lay : layout(cands[i2].grid, SPEC, sh2[j2]);
        var dec2 = CODEC.readCode(gray, cands[i2].res, lay2, cellPx, { hue: true, hueBits: 1, rgbaImg: imageData, psf: true });
        if (dec2 && dec2.ok) { decoded = dec2; best = cands[i2]; decShape = sh2[j2]; break; }
      }
    }
  }
  if (!best) best = cands[0];
  // 해독으로 증명되지 않은 느슨한 잠금은 거절(오검출 차단).
  //   스케일 불일치(>0.5 ≈ 65% 어긋남)도 미해독 상태에선 유령 잠금으로 취급.
  if (!decoded && (best.res.residPx > TIGHT * cellPx || best.scaleErr > 0.5))
    return { ok:false, reason:'loose-lock', residPx:+best.res.residPx.toFixed(2), scaleErr:+(best.scaleErr||0).toFixed(3), cellPx:+cellPx.toFixed(2), tried:tried, ms:Math.round(_now()-t0) };
  var r = best.res;
  return { ok:true, method:r.method, core:r.core, corners:r.corners,
           northStar:r.northStar||'TL', residPx:+(r.residPx||0).toFixed(2),
           modPx: r.modPx || null, orbitMatch: (typeof r.orbitMatch === 'number') ? r.orbitMatch : null,
           scaleErr: +(best.scaleErr || 0).toFixed(3),
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
