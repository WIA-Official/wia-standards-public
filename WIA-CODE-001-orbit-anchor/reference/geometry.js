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
    const lobeD = 0.55, lobeR = 0.50, hubR = 0.30;
    if ((nx * nx + ny * ny) <= hubR * hubR) return true;
    const centers = [[0, -lobeD], [lobeD, 0], [0, lobeD], [-lobeD, 0]];
    for (const [cx, cy] of centers) {
      const dx = nx - cx, dy = ny - cy;
      if (dx * dx + dy * dy <= lobeR * lobeR) return true;
    }
    return false;
  }
  if (L.shape === 'star') {
    // 4개 꼭짓점(위성 자리) + 4개 안쪽 오목점 — 꼭짓점/오목점을 직선으로 잇는 실제 별 폴리곤.
    const Ro = 1.0, Ri = 0.24; // Ri=0.42은 오목점이 너무 얕아 별이 아니라 마름모로 보였음(실사용자 지적) — 0.24로 깊게
    const th = Math.atan2(ny, nx);
    const sect = Math.PI / 2, half = sect / 2;
    let a = ((th + half) % sect + sect) % sect - half; // -half..+half, 0=꼭짓점 방향
    const phi = Math.abs(a);
    // 꼭짓점(phi=0, r=Ro) → 오목점(phi=half, r=Ri)을 잇는 직선과 각도 phi 광선의 교점 반지름.
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
