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
