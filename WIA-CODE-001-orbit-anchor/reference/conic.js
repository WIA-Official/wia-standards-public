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
