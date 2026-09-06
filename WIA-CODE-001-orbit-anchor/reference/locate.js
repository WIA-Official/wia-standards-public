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

/* ★2026-09-01 — 불스아이 템플릿 적합도 (0.05..~1). 배율을 후보 **자신의** 첫 상승 에지에서
 *   유도한다(r1 ≈ 1.5c). 그러면 코어는 1.5c 어둠 / 3.5c 밝음 / 5.5c 어둠 / 바깥 밝음이
 *   제자리에 오고, 위성 원판(첫 에지 2.5c)이나 도넛(중심이 밝아 첫 에지 없음)은 이 밴드가
 *   맞지 않는다. 프레임 비율 하나(0.05·mn)에 묶인 탐침은 배율에 따라 코어 서명을 깼고
 *   (heart/M: rev 2·centerDark 0), "여러 R 중 최대"는 위성이 유리한 R 을 고르게 했다. */
function bullseyeFit(grayObj, x, y) {
  const mn = Math.min(grayObj.w, grayObj.h), R = 0.2 * mn, steps = Math.max(40, Math.round(R));
  const prof = radialProfile(grayObj, x, y, R, steps), k = prof.length, st = R / k;
  let mn2 = Infinity, mx2 = -Infinity; for (let i = 0; i < k; i++) { if (prof[i] < mn2) mn2 = prof[i]; if (prof[i] > mx2) mx2 = prof[i]; }
  const rng = Math.max(1, mx2 - mn2);
  const nz = i => (prof[Math.min(k - 1, Math.max(0, i))] - mn2) / rng;
  if (nz(0) > 0.5) return 0.05;                         // 중심이 밝다(도넛) → 불스아이 아님
  let i1 = -1; for (let i = 1; i < k; i++) if (nz(i) > 0.5) { i1 = i; break; }
  if (i1 < 2) return 0.05;                              // 첫 에지가 없거나 너무 가깝다
  const c = ((i1 + 0.5) * st) / 1.5;                    // 모듈 px 추정
  if (6.5 * c > R) return 0.05;                         // 탐침이 바깥 밝음 밴드까지 못 미친다
  const band = (a, b) => { let s = 0, n = 0; for (let i = Math.round(a * c / st); i <= Math.round(b * c / st); i++) { s += nz(i); n++; } return n ? s / n : 0.5; };
  const D0 = band(0.2, 1.1), L1 = band(1.9, 3.1), D2 = band(3.9, 5.1), L3 = band(5.9, 6.5);
  const fit = (L1 - D0 + L1 - D2 + L3 - D2) / 3;      // 이상적 1, 원판 ≈ 0.3 이하, 잡음 ≈ 0
  return Math.max(0.05, fit);
}

// ── 코어 시드 선택 (surround 실패 시): 불스아이 템플릿에 가장 잘 맞는 피크 ──
//   ★2026-09-01 개정. 이전 식 score×(1+rev)×(centerDark>0?1:0.3) 은 두 가지로 깨졌다
//   (SVG_CORESEED_HANDOFF_2026-09-01.md 실측):
//   ① rProbe(0.05·프레임)가 배율에 안 맞으면 코어 서명이 사라진다 — heart/M 640px 에선
//      탐침이 코어 바깥 링(5.5모듈=33.8px) 안에서 끝나 rev 2·centerDark 0 → 코어(FRST 15,792,
//      위성의 2.2배)가 도넛에 졌다. ② 탐침이 충분한 cellPx=10(생성기 기본 1360px)에선 위성
//      반경 25px 가 coarse 투표 반경 25 와 겹쳐 위성 원시점수가 코어를 넘고, 코어는 rev 배수로만
//      ×1.02~1.06 여유로 이겼다 → 벡터 SVG 래스터의 앨리어싱 한 번에 뒤집혀 weak-core.
//   "코어는 압도적"이라는 옛 주석은 밀착(8/30) 이후 사실이 아니다. 66종×3배율 198건 최소여유:
//   옛 식 ×1.02 → 이 식 ×2.16(bird@10). bench-detect 회귀 0 · HUG_ON 왕복 500/500 · arena/yaw30 통과.
//   (rProbe 인자는 호환을 위해 남겨 두지만 쓰지 않는다 — 배율은 후보 자신이 말한다.)
function pickCoreSeed(grayObj, peaksList, rProbe) {
  let best = null, bestS = -Infinity;
  for (const p of peaksList.slice(0, 10)) {
    const s = (p.score || 1) * bullseyeFit(grayObj, p.x, p.y);
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
/* fitSimilarity — (X,Y) → (u,v) 를 similarity 4자유도로 최소제곱 적합 (2026-08-30 Q4)
 *   u = aX − bY + tx ,  v = bX + aY + ty       (a,b 가 배율·회전을 함께 담는다)
 *   점 n 개면 식이 2n 개, 미지수는 4개 → **n ≥ 3 이면 과결정**이라 잔차가 의미를 갖는다.
 *   (자유 호모그래피는 미지수 8개라 4점에서 잔차가 항상 0 — 어제 locate3 가 죽은 이유다.) */
function fitSimilarity(A, B) {
  var M = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]], r = [0,0,0,0];
  for (var i = 0; i < A.length; i++) {
    var X = A[i][0], Y = A[i][1], u = B[i][0], v = B[i][1];
    var rows = [[X, -Y, 1, 0, u], [Y, X, 0, 1, v]];
    for (var k = 0; k < 2; k++) {
      var row = rows[k];
      for (var a = 0; a < 4; a++) { r[a] += row[a] * row[4]; for (var b = 0; b < 4; b++) M[a][b] += row[a] * row[b]; }
    }
  }
  var G = M.map(function (row, i2) { return row.concat([r[i2]]); });
  for (var p = 0; p < 4; p++) {
    var piv = p;
    for (var q = p + 1; q < 4; q++) if (Math.abs(G[q][p]) > Math.abs(G[piv][p])) piv = q;
    var t = G[p]; G[p] = G[piv]; G[piv] = t;
    var d = G[p][p]; if (Math.abs(d) < 1e-12) return null;
    for (var c = p; c < 5; c++) G[p][c] /= d;
    for (var q2 = 0; q2 < 4; q2++) { if (q2 === p) continue; var f = G[q2][p]; for (var c2 = p; c2 < 5; c2++) G[q2][c2] -= f * G[p][c2]; }
  }
  return { a: G[0][4], b: G[1][4], tx: G[2][4], ty: G[3][4] };
}

/*
 * locateSim3 — 정면화 프레임에서 **위성 3개**로 확정한다 (2026-08-30 Q4)
 *   conic 정면화가 사영+아핀 4자유도를 이미 고정했으므로 잔여는 similarity 4자유도다.
 *   코어 + 위성 3 = 8식/4미지수 = **과결정** → 잔차 게이트가 살아난다.
 *   ★prep(Q1-B 의 공유 정면화)을 그대로 쓴다 — 워프를 새로 하지 않는다(추가 비용 0).
 *   ★도넛(12시)이 가려지면 포기한다 — 회전을 정할 근거가 없다.
 */
function locateSim3(grayObj, peaksList, layout, prep, cellPx) {
  if (!prep || !prep.gray || !prep.peaks) return { ok: false, reason: 'no-prep' };
  var spec = layout.spec || {};
  var satR = spec.satRadius || 2.5;
  var cx = prep.dim / 2, cy = prep.dim / 2;      // 정면화가 코어를 캔버스 중앙에 놓는다
  // 정면화 프레임에서 1 모듈 = cellPx 픽셀 (rectifyHomography 가 코어 바깥링 5.5모듈을
  //   5.5*cellPx 로 맞추므로). 그래서 아래 거리 비교가 **모듈 단위**로 성립한다.
  var Rs = Math.hypot(layout.anchors[1].mx - layout.coreCenter.mx,
                      layout.anchors[1].my - layout.coreCenter.my);
  // ★C3(2026-09-04): 밀착 핀은 마름모(rA≠rB — hug-on 79종 중 56종이 12% 넘게 다르다)라
  //   반경 하나로 거르면 위성 절반이 창 밖이다(whale/L 13.75 vs 21.75 → 위성 0개 → sats<3, 실측).
  //   레이아웃의 위성 4개 거리를 전부 모아(5% 안이면 하나로) 어느 하나에라도 15% 안이면 받는다.
  //   원형 배치는 Rlist 가 원소 1개라 예전과 완전히 같다.
  var Rlist = [];
  ['TL','TR','BR','BL'].forEach(function(nm){ var a = layout.anchors.find(function(x){ return x.name === nm; }); if (!a) return;
    var d = Math.hypot(a.mx - layout.coreCenter.mx, a.my - layout.coreCenter.my);
    for (var ri = 0; ri < Rlist.length; ri++) if (Math.abs(Rlist[ri] - d) <= 0.05 * Rlist[ri]) return;
    Rlist.push(d); });
  if (!Rlist.length) Rlist = [Rs];

  // 코어 정밀화(정면화 프레임)
  var cr = refineCentroid(prep.gray, cx, cy, 1.5 * cellPx * 1.1);
  var coreR = (cr.ok && Math.hypot(cr.x - cx, cr.y - cy) < 2 * cellPx) ? { x: cr.x, y: cr.y } : { x: cx, y: cy };

  // 위성 후보 — ★**정밀화는 원본 프레임에서** 한다 (2026-08-30 실측으로 고침).
  //   처음엔 정면화 이미지(prep.gray)에서 정밀화했더니 잔차 1.17px 였다(cellPx 4.7 의 25%).
  //   정면화본은 워프로 리샘플된 이미지라 가장자리가 뭉개져 무게중심이 부정확하다.
  //   원본에서 정밀화한 뒤 정면화 좌표로 사상하면 원본의 선명도를 그대로 쓴다.
  var toRect = function (pt) { var q = CN.matVec3(prep.H, [pt.x, pt.y, 1]); return { x: q[0] / q[2], y: q[1] / q[2] }; };
  var sats = [];
  var src = (peaksList && peaksList.length) ? peaksList : prep.peaks;
  var srcIsImg = !!(peaksList && peaksList.length);
  for (var i = 0; i < src.length; i++) {
    var p0 = src[i];
    // 정면화 좌표로 옮겨 거리(모듈)를 잰다 — 정면화 프레임은 metric 이다.
    var pr = srcIsImg ? toRect(p0) : p0;
    var dMod = Math.hypot(pr.x - coreR.x, pr.y - coreR.y) / cellPx;
    var okR = false; for (var rj = 0; rj < Rlist.length; rj++) if (Math.abs(dMod - Rlist[rj]) <= Rlist[rj] * 0.15) { okR = true; break; }
    if (!okR) continue;
    var fxi, fyi;
    if (srcIsImg) {
      var rf = refineCentroid(grayObj, p0.x, p0.y, satR * cellPx * 1.1);   // ★원본에서 정밀화
      fxi = rf.ok ? rf.x : p0.x; fyi = rf.ok ? rf.y : p0.y;
    } else { fxi = p0.x; fyi = p0.y; }
    var pf = srcIsImg ? toRect({ x: fxi, y: fyi }) : { x: fxi, y: fyi };
    var dup = false;
    for (var j = 0; j < sats.length; j++) if (Math.hypot(sats[j].x - pf.x, sats[j].y - pf.y) < 3 * cellPx) { dup = true; break; }
    if (dup) continue;
    sats.push({ x: pf.x, y: pf.y,
                bright: srcIsImg ? grayAt(grayObj, fxi, fyi) : grayAt(prep.gray, fxi, fyi),
                score: p0.score });
  }
  if (sats.length < 3) return { ok: false, reason: 'sim3-sats<3', found: sats.length };
  sats.sort(function (a, b) { return b.score - a.score; });
  var use = sats.slice(0, 4);

  // 도넛(북극성) = 중심이 가장 밝은 것. 원판과의 차가 작으면 도넛이 가려진 것으로 보고 포기.
  var di = 0, bm = -Infinity;
  for (var k2 = 0; k2 < use.length; k2++) if (use[k2].bright > bm) { bm = use[k2].bright; di = k2; }
  var others = [];
  for (var k3 = 0; k3 < use.length; k3++) if (k3 !== di) others.push(use[k3].bright);
  // ★도넛이 안 보이는 경우 — 포기하지 않는다 (2026-08-30 Fable 2차).
  //   위성 3개가 90° 간격으로 보이면 **빈 슬롯 자체가 회전**이다:
  //   셋이 전부 암부(원판)면 가려진 것이 도넛이고, **빈 자리가 TL** 이다.
  //   안전판: 정확히 3개이고 최대 밝기가 128 미만(= 전부 어둡다)일 때만 추론한다.
  //   밝은 가짜 피크가 슬롯을 채우면 지금처럼 CRC 가 거른다(현행 대비 악화 없음).
  var donutInferred = false;
  if (others.length && bm - Math.max.apply(null, others) < 40) {
    // ★C3(2026-09-04): 전부 어두운데 후보가 4개면 하나는 가짜(데이터 도트 뭉치·궤도점)다. 그 가짜가 낀 채로
    //   '정확히 3개'를 요구하면 도넛 가림이 통째로 죽는다(whale TL 실측). 점수 상위 3개만 남기고 추론한다 —
    //   가짜가 진짜보다 높으면 슬롯 충돌·적합 잔차·CRC 가 거른다(현행 대비 악화 없음).
    if (bm < 128 && use.length >= 3) { use = use.slice(0, 3); donutInferred = true; }
    else return { ok: false, reason: 'sim3-donut-missing' };
  }

  // 각도 라벨 — 정면화 프레임은 metric 이라 90° 간격으로 떨어진다.
  var names = ['TL', 'TR', 'BR', 'BL'];
  var labeled = {};
  if (donutInferred) {
    // 셋 다 원판이다 → 빈 슬롯이 도넛(TL). 임의의 하나를 기준 0 으로 잡고 슬롯을 채운 뒤,
    // 비어 있는 슬롯이 TL 이 되도록 전체를 회전시킨다.
    var base = use[0];
    var a0 = Math.atan2(base.y - coreR.y, base.x - coreR.x);
    var slots = [null, null, null, null];
    for (var mi = 0; mi < use.length; mi++) {
      var rl = (Math.atan2(use[mi].y - coreR.y, use[mi].x - coreR.x) - a0) / (Math.PI / 2);
      var sl = ((Math.round(rl) % 4) + 4) % 4;
      if (slots[sl]) return { ok: false, reason: 'sim3-slot-collide' };
      slots[sl] = use[mi];
    }
    var empty = slots.indexOf(null);
    if (empty < 0) return { ok: false, reason: 'sim3-no-empty' };
    for (var si = 0; si < 4; si++) {
      if (!slots[si]) continue;
      labeled[names[(si - empty + 4) % 4]] = slots[si];   // 빈 자리를 TL(0)로 맞춰 회전
    }
  } else {
    var donut = use[di];
    var ang0 = Math.atan2(donut.y - coreR.y, donut.x - coreR.x);
    labeled.TL = donut;
    for (var m = 0; m < use.length; m++) {
      if (use[m] === donut) continue;
      var rel = (Math.atan2(use[m].y - coreR.y, use[m].x - coreR.x) - ang0) / (Math.PI / 2);
      var slot = ((Math.round(rel) % 4) + 4) % 4;
      if (slot >= 1 && slot <= 3 && !labeled[names[slot]]) labeled[names[slot]] = use[m];
    }
  }
  var found = Object.keys(labeled);
  if (found.length < 3) return { ok: false, reason: 'sim3-slots<3', found: found.length };

  // similarity 적합: 모듈좌표 → 정면화 프레임
  var A = [[layout.coreCenter.mx, layout.coreCenter.my]], B = [[coreR.x, coreR.y]];
  for (var f1 = 0; f1 < found.length; f1++) {
    var an = layout.anchors.find(function (x) { return x.name === found[f1]; });
    A.push([an.mx, an.my]); B.push([labeled[found[f1]].x, labeled[found[f1]].y]);
  }
  var sim = fitSimilarity(A, B);
  if (!sim) return { ok: false, reason: 'sim3-fit' };

  // ★2단계 정밀화 — `locate()` 의 fine() 과 같은 원리 (2026-08-30).
  //   1단계는 FRST 극대점을 균일 창으로 정밀화한 것이라 오차가 남는다(실측 resid 1.19px,
  //   cellPx 4.7 의 25% — 셀 표본을 어긋나게 하기에 충분해서 "락만" 으로 끝났다).
  //   적합된 sim 이 각 앵커의 **예상 위치**를 주므로, 그 자리에서 앵커에 맞는 창으로
  //   다시 정밀화한다: 도넛은 **밝은 중심**(흰 구멍, satInner 창), 원판은 **암부 중심**(satR 창).
  //   ★정밀화는 원본 프레임에서 한다 — 정면화본은 리샘플이라 가장자리가 뭉개진다.
  var satInner = spec.satInner || 1.0;
  var toImg = function (pt) { var q = CN.matVec3(prep.Hinv, [pt.x, pt.y, 1]); return { x: q[0] / q[2], y: q[1] / q[2] }; };
  var imgPts = null;      // ★원본 프레임 좌표를 보관한다 — 아래 내삽 재적합이 이걸 쓴다
  for (var pass = 0; pass < 2; pass++) {
    var B2 = [], IP = [];
    for (var a2 = 0; a2 < A.length; a2++) {
      var ex = sim.a * A[a2][0] - sim.b * A[a2][1] + sim.tx;
      var ey = sim.b * A[a2][0] + sim.a * A[a2][1] + sim.ty;
      var isCore = (a2 === 0);
      var isDonut = (!isCore && found[a2 - 1] === 'TL' && !donutInferred);
      var rMod = isCore ? 1.5 : (isDonut ? satInner * 1.3 : satR);
      var pi2 = toImg({ x: ex, y: ey });
      var rf2 = refineCentroid(grayObj, pi2.x, pi2.y, Math.max(2, rMod * cellPx * 1.05), isDonut);
      if (rf2.ok && Math.hypot(rf2.x - pi2.x, rf2.y - pi2.y) < rMod * cellPx * 0.6) {
        var pr2 = toRect({ x: rf2.x, y: rf2.y });
        B2.push([pr2.x, pr2.y]); IP.push([rf2.x, rf2.y]);
      } else { B2.push([ex, ey]); IP.push([pi2.x, pi2.y]); }
    }
    var sim2 = fitSimilarity(A, B2);
    if (!sim2) break;
    sim = sim2; B = B2; imgPts = IP;
  }

  // ★배율 검증 — 정면화 프레임의 1모듈은 cellPx 픽셀이므로 similarity 배율은 cellPx 여야 한다.
  //   벗어나면 위성이 아닌 것을 주웠다는 뜻이다(잔차만으로는 못 거르던 자리를 여기서 막는다).
  var scale = Math.hypot(sim.a, sim.b);
  if (!(scale > 0) || Math.abs(Math.log(scale / cellPx)) > 0.12) {
    return { ok: false, reason: 'sim3-scale', scale: +scale.toFixed(2), cellPx: +cellPx.toFixed(2) };
  }

  // 과결정 잔차 — 여기서는 **의미가 있다**(8식/4미지수).
  var resid = 0;
  for (var r2 = 0; r2 < A.length; r2++) {
    var ux = sim.a * A[r2][0] - sim.b * A[r2][1] + sim.tx;
    var uy = sim.b * A[r2][0] + sim.a * A[r2][1] + sim.ty;
    resid += Math.hypot(ux - B[r2][0], uy - B[r2][1]);
  }
  resid /= A.length;

  // 정면화 프레임 → 원본 이미지 프레임으로 합성
  // ★★행렬 형식이 두 벌이다 (2026-08-30 실측으로 잡음):
  //   `homographyLS` 는 **평탄 9배열** `[h0..h8]` 을 돌려주고 `applyH` 도 그 형식을 받는다.
  //   `CN.mul3`·`CN.matVec3` 는 **중첩 3×3** `[[..],[..],[..]]` 을 쓴다.
  //   섞으면 `applyH` 가 조용히 `[null,null]` 을 낸다 — 던지지 않는다.
  //   실제 피해: 기하는 완벽했는데(코어 오차 0.2px, 회전 0.0°) `verifyLayout` 의
  //   modPx 가 NaN 이 되어 scaleErr 가 NaN → loose-lock 으로 버려졌다.
  //   여기서는 mul3 로 곱한 뒤 **평탄화해서** 내보낸다.
  var Hsim = [[sim.a, -sim.b, sim.tx], [sim.b, sim.a, sim.ty], [0, 0, 1]];
  var Hn = CN.mul3(prep.Hinv, Hsim);
  if (!Hn) return { ok: false, reason: 'sim3-compose' };
  var H = [Hn[0][0], Hn[0][1], Hn[0][2], Hn[1][0], Hn[1][1], Hn[1][2], Hn[2][0], Hn[2][1], Hn[2][2]];
  if (H[8]) { for (var hi = 0; hi < 9; hi++) H[hi] /= H[8]; }   // h8=1 로 정규화(다른 경로와 동일)

  var back = function (pt) { var q = CN.matVec3(prep.Hinv, [pt.x, pt.y, 1]); return { x: q[0] / q[2], y: q[1] / q[2] }; };
  var corners = {};
  for (var c3 = 0; c3 < 4; c3++) {
    var nm = names[c3];
    if (labeled[nm]) { corners[nm] = back(labeled[nm]); continue; }
    var an2 = layout.anchors.find(function (x) { return x.name === nm; });   // 빠진 위성은 되짚어 채운다
    var ex = sim.a * an2.mx - sim.b * an2.my + sim.tx, ey = sim.b * an2.mx + sim.a * an2.my + sim.ty;
    corners[nm] = back({ x: ex, y: ey }); corners[nm].inferred = true;
  }
  // ★★내삽 재적합 (2026-08-30 Fable 2차) — **이것이 잔차 1.12 → 0.1 을 만든다.**
  //   위 sim-H 는 코어(5.5모듈)에서 위성(60.5모듈)으로 **외삽**한 결과라
  //   코어 링 중심차 0.025px 가 131배로 자라 쌍극 오차를 만든다.
  //   여기서는 **원본 프레임에서** 코어 + 위성3 + 궤도 on-도트로 8DOF 를 다시 적합한다.
  //   궤도(8.5모듈)와 위성(60.5모듈) 사이가 되므로 데이터판이 **내삽 구간**에 들어온다.
  //   ★homographyLS 는 평탄 9배열을 바로 돌려준다 — mul3 합성·평탄화가 필요 없다.
  var Horb = null, residOrb = null;
  if (imgPts && imgPts.length >= 4 && layout.orbit && layout.orbit.length) {
    var oa = [], ob = [];
    for (var q1 = 0; q1 < A.length; q1++) { oa.push([A[q1][0], A[q1][1]]); ob.push([imgPts[q1][0], imgPts[q1][1]]); }
    var dots = 0;
    for (var d1 = 0; d1 < layout.orbit.length; d1++) {
      var od = layout.orbit[d1];
      if (!od.on) continue;                                  // 켜진 도트(암부)만 무게중심이 성립한다
      var pe = applyH(H, od.mx, od.my);
      var win = Math.max(1.5, 0.9 * cellPx);
      var rd2 = refineCentroid(grayObj, pe[0], pe[1], win);
      if (!rd2.ok) continue;
      if (Math.hypot(rd2.x - pe[0], rd2.y - pe[1]) > 0.7 * win) continue;   // 예측에서 너무 멀면 딴 것
      oa.push([od.mx, od.my]); ob.push([rd2.x, rd2.y]); dots++;
    }
    if (dots >= 6) {                                          // 도트가 적으면 신뢰하지 않는다
      var Hf = homographyLS(oa, ob);
      if (Hf) {
        var ro2 = 0;
        for (var q2 = 0; q2 < A.length; q2++) {               // 잔차는 코어+위성에서만(도트는 작아 편향)
          var pq = applyH(Hf, oa[q2][0], oa[q2][1]);
          ro2 += Math.hypot(pq[0] - ob[q2][0], pq[1] - ob[q2][1]);
        }
        ro2 /= A.length;
        if (ro2 <= 0.35 * cellPx) { Horb = Hf; residOrb = ro2; }
      }
    }
  }

  var Huse = Horb || H, residUse = (residOrb != null) ? residOrb : resid;
  var vf = verifyLayout(grayObj, Huse, layout);
  return { ok: true, method: Horb ? 'sim3orb' : 'sim3', core: back(coreR), corners: corners,
           northStar: 'TL', Hmod2img: Huse, residPx: +residUse.toFixed(2), dCen: 0,
           // ★orb-H 가 실패해도 sim-H 로는 읽히는 경우가 있다(블러가 쌍극을 뭉갠다) — 보조로 실어 보낸다.
           altH: Horb ? H : null,
           sats3: found.length, donutInferred: donutInferred,
           missing: names.filter(function (n) { return !labeled[n]; })[0] || null,
           modPx: vf.modPx, orbitMatch: vf.orbitMatch, orbitContrast: vf.orbitContrast };
}

/* ★refitWithOrbit — 궤도 24도트로 **내삽 재적합** (2026-08-30, 밀착 전용)
 *
 *   `locate` 의 4점 호모그래피는 코어(5.5)와 위성으로 만든 **외삽**이다.
 *   위성이 멀면(60.5) 문제가 없지만, **밀착하면 위성이 안으로 들어와** 외삽 배율이 커진다:
 *     foot/L 짧은 축 16모듈 → 격자 가장자리(64)까지 **4배 외삽**
 *   그러면 작은 각도 오차가 4배로 자라 데이터 표본이 셀을 벗어난다.
 *   실측: foot/L 을 **정확히 원본 크기(816px)** 로 주면 locate 는 resid 0.13 으로 성공하는데
 *   readCode 가 실패한다(815·817px 는 성공 — 딱 정수배에서만 표본이 셀 경계에 앉는다).
 *
 *   여기서는 궤도(8.5모듈) on-도트를 추가로 실측해 **코어+위성4+도트**로 8DOF 를 다시 적합한다.
 *   궤도가 안쪽에 있으니 데이터판이 **내삽 구간**에 들어와 오차가 안 자란다.
 *   `locateSim3` 안에 있던 같은 기법을 4위성 경로에서도 쓸 수 있게 꺼낸 것이다.
 *
 *   ★안전: 도트가 6개 미만이거나 재적합 잔차가 나빠지면 **원래 H 를 그대로 돌려준다.**
 *     즉 이 함수는 결과를 나쁘게 만들지 않는다. */
function refitWithOrbit(grayObj, res, layout, cellPx) {
  if (!res || !res.ok || !res.Hmod2img || !layout || !layout.orbit || !layout.orbit.length) return res;
  if (!res.core || !res.corners) return res;
  const H = res.Hmod2img;
  const oa = [], ob = [];
  const c = layout.coreCenter;
  oa.push([c.mx, c.my]); ob.push([res.core.x, res.core.y]);
  for (const a of layout.anchors) {
    if (a.name === 'core') continue;
    const p = res.corners[a.name];
    if (!p) continue;
    oa.push([a.mx, a.my]); ob.push([p.x !== undefined ? p.x : p[0], p.y !== undefined ? p.y : p[1]]);
  }
  if (oa.length < 4) return res;
  const nAnchor = oa.length;
  let dots = 0;
  for (const od of layout.orbit) {
    if (!od.on) continue;
    const pe = applyH(H, od.mx, od.my);
    const win = Math.max(1.5, 0.9 * cellPx);
    const rd = refineCentroid(grayObj, pe[0], pe[1], win);
    if (!rd.ok) continue;
    if (Math.hypot(rd.x - pe[0], rd.y - pe[1]) > 0.7 * win) continue;
    oa.push([od.mx, od.my]); ob.push([rd.x, rd.y]); dots++;
  }
  if (dots < 6) return res;
  const Hf = homographyLS(oa, ob);
  if (!Hf) return res;
  let ro = 0;
  for (let i = 0; i < nAnchor; i++) {
    const pq = applyH(Hf, oa[i][0], oa[i][1]);
    ro += Math.hypot(pq[0] - ob[i][0], pq[1] - ob[i][1]);
  }
  ro /= nAnchor;
  if (ro > 0.35 * cellPx) return res;
  const vf = verifyLayout(grayObj, Hf, layout);
  const out = {};
  for (const k in res) out[k] = res[k];
  out.Hmod2img = Hf; out.residPx = +ro.toFixed(2);
  out.altH = H;                       // ★원래 H 도 실어 보낸다 — 재적합이 나쁠 때 호출부가 되돌린다
  out.method = (res.method || 'locate') + 'orb';
  out.modPx = vf.modPx; out.orbitMatch = vf.orbitMatch; out.orbitContrast = vf.orbitContrast;
  return out;
}

/*
 * prepareConic — conic 정면화의 **layout 무관 부분**을 한 번만 한다 (2026-08-30 Q1-B)
 *   반환한 `prep` 을 여러 layout 이 공유하면 warp·재-FRST 를 6회 → 1회로 줄인다.
 *   실측 근거: 실패 프레임에서 detectAuto 가 locateRobust 를 최대 18회 부르고,
 *   그때마다 이 전부(링추출·conic적합·워프 약600²·FRST전체)를 다시 했다.
 *   layout 이 관여하는 건 캔버스 크기(dim)뿐이라 **가장 큰 dim 으로 한 번**이면 된다.
 *   opts.redetect 가 없으면 준비하지 않는다(호출부가 재검출을 못 하면 의미가 없다).
 */
function prepareConic(img, grayObj, peaksList, cellPx, maxDistMod, opts) {
  opts = opts || {};
  if (!opts.redetect) return null;
  const rProbe = 3.5 * cellPx;
  // ★씨앗 힌트(2026-08-31) — 호출자가 코어 좌표를 이미 알면 그걸 쓴다.
  //   원근(yaw)에서 코어의 5.5모듈 링이 타원이 되면 원형 FRST 투표가 흩어져
  //   pickCoreSeed 가 **위성을 코어로 고른다**(실측: 잘못 고른 씨앗의 코어 거리가
  //   그 실루엣의 밀착 핀 반경과 정확히 일치했다). 힌트가 없으면 기존 경로 그대로.
  const seed = (opts && opts.seed && isFinite(opts.seed.x) && isFinite(opts.seed.y))
             ? opts.seed : pickCoreSeed(grayObj, peaksList, rProbe);
  if (!seed) return null;
  const rings = CN.extractCoreRings(grayObj, seed.x, seed.y, 0.18 * Math.min(grayObj.w, grayObj.h), 180);
  const cOut = CN.fitConic(rings.outer), cIn = CN.fitConic(rings.inner);
  if (!cOut || !cIn) return null;
  const rec = CN.recoverFromConcentric(cOut.C, cIn.C);
  if (!rec.ok || rec.ringRatio < 2.6 || rec.ringRatio > 4.6) return null;
  const dim = 2 * Math.ceil(maxDistMod * cellPx + 4 * cellPx);
  const H = CN.rectifyHomography(rec.l, cOut.C, rec.center, 5.5 * cellPx, [dim / 2, dim / 2]);
  if (!H) return null;
  const rimg = warpImage(img, H, dim);
  if (!rimg) return null;
  const rd = opts.redetect(rimg);
  if (!rd || !rd.gray) return null;
  const Hinv = CN.inv3(H);
  if (!Hinv) return null;
  return { H: H, Hinv: Hinv, dim: dim, gray: rd.gray, peaks: rd.peaks, ringRatio: rec.ringRatio };
}

/*
 * locateRobustShared — 준비된 conic(prep)을 써서 한 layout 을 확정한다 (2026-08-30 Q1-B)
 *   prep 이 null 이면 **기존 locateRobust 를 그대로 부른다**(동작 동일, 안전판).
 *   prep 이 있으면 primary 실패 시 warp 없이 정면화 이미지에서 바로 locate 한다.
 */
function locateRobustShared(img, grayObj, peaksList, layout, opts, prep) {
  opts = opts || {};
  const cellPx = opts.cellPx || 6;
  const primary = locate(grayObj, peaksList, layout);
  const passPx = opts.passPx != null ? opts.passPx : 0.45 * cellPx;
  if (primary.ok && primary.residPx <= passPx) return Object.assign({ method: 'orbit' }, primary);
  if (!prep) return locateRobust(img, grayObj, peaksList, layout, opts);

  const rres = locate(prep.gray, prep.peaks, layout);
  // ★2026-08-30 Q4 — 위성 3개 경로(similarity 과결정). 정면화 프레임에서는 잔여 자유도가
  //   similarity 4개뿐이라 **위성 3개로도 과결정**이다(코어 포함 8식/4미지수).
  //   워프는 이미 prep 에 있으므로 추가 비용이 없다.
  // ★★게이트를 `!rres.ok` 로 잡았다가 **블러에서 한 번도 안 불렸다**(2026-08-30 실측).
  //   블러가 생기면 극대점이 늘어(4→11개) locate 가 **엉터리 4점으로 "성공"**한다 —
  //   ok:true 에 잔차만 14~15px 인 상태다. 그러면 위 조건이 거짓이라 sim3 를 건너뛴다.
  //   (어제 locate3 에서 똑같은 실수를 했다: "실패했을 때만"이 아니라 "잘 안 됐을 때"여야 한다.)
  //   → 4위성 결과가 **깨끗하지 않으면** 항상 시도하고, 잔차가 더 좋은 쪽을 쓴다.
  if (!rres.ok || rres.residPx > passPx) {
    const t3 = locateSim3(grayObj, peaksList, layout, prep, cellPx);
    if (t3.ok && (!rres.ok || t3.residPx < rres.residPx)) return t3;
  }
  if (!rres.ok) {
    return primary.ok ? Object.assign({ method: 'orbit' }, primary)
                      : Object.assign({ method: 'conic-fail', reason: 'rect-' + rres.reason }, { ok: false });
  }
  const back = (pt) => { const q = CN.matVec3(prep.Hinv, [pt.x, pt.y, 1]); return { x: q[0] / q[2], y: q[1] / q[2] }; };
  const cornersImg = { TL: back(rres.corners.TL), TR: back(rres.corners.TR),
                       BR: back(rres.corners.BR), BL: back(rres.corners.BL) };
  const coreImg = back(rres.core);
  const order = ['TL', 'TR', 'BR', 'BL'];
  const A = order.map(k => { const a = layout.anchors.find(x => x.name === k); return [a.mx, a.my]; });
  const B = order.map(k => [cornersImg[k].x, cornersImg[k].y]);
  A.push([layout.coreCenter.mx, layout.coreCenter.my]); B.push([coreImg.x, coreImg.y]);
  const Hfin = homographyLS(A, B);
  let resid = 0;
  if (Hfin) { for (let i = 0; i < A.length; i++) { const p = applyH(Hfin, A[i][0], A[i][1]); resid += Math.hypot(p[0] - B[i][0], p[1] - B[i][1]); } resid /= A.length; }
  const vf = Hfin ? verifyLayout(grayObj, Hfin, layout) : { modPx: 0, orbitMatch: 0.5, orbitContrast: 0 };
  return { ok: true, method: 'conic', core: coreImg, corners: cornersImg,
           northStar: rres.northStar, Hmod2img: Hfin, residPx: +resid.toFixed(2),
           conicCenterErr: null, ringRatio: +prep.ringRatio.toFixed(2),
           modPx: vf.modPx, orbitMatch: vf.orbitMatch, orbitContrast: vf.orbitContrast };
}

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

module.exports = { locate, locateRobust, locateRobustShared, locateSim3, refitWithOrbit, prepareConic, fitSimilarity, homographyLS, pickCoreSeed, bullseyeFit, verifyLayout };
