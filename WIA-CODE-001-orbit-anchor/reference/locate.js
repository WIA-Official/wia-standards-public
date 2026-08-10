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
