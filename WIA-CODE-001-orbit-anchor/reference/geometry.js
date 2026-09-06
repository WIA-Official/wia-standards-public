'use strict';
/* ★밀착 반경 핀(2026-08-30) — 정본은 shape-registry.js 다.
 *   번들(build-core.js)이 shape-registry 를 geometry 보다 **먼저** 등록하므로 안전하다.
 *   없어도 동작한다(그때는 solver 가 계산한다) — 그래서 실패를 삼킨다. */
let REG = null; try { REG = require('./shape-registry.js'); } catch (e) { REG = null; }
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
  satHalo: 1.0,             // ★밀착 모드: 위성 바깥 흰 고리 두께(모듈). 데이터 속에서 위성을 떼어낸다
                            //   1.6 은 과잉이었다 — 0.6 에서도 위치오차 0.06~0.08모듈로 편향이 없다(실측).
                            //   1.0 이면 1.6 대비 용량 +1.0%(ear)·+1.8%(foot) 를 회복한다.
  hugFanDeg: 20,            // ★밀착 탐색 부채꼴 반각(도) — **탈출구로만** 쓴다(P2 정책, 아래 참조).
                            //   35° 로 "가장 먼 잉크"를 쫓았더니 반경 시그니처 충돌이 7→59 쌍으로 늘고
                            //   16종이 90° 슬롯 배정에서 경계에 걸렸다(ear 44°/45°). 최대반경 추구가 틀렸다.
                            //   ±20° 면 45° 슬롯 경계까지 25° 여유가 남는다.
  hugFloor: 14,             // ★밀착 반경 바닥(모듈). 궤도 8.5 + 도트 0.9 + 고리 + 위성 2.5 + 여유.
                            //   축방향 탐색은 love TL=4.6 처럼 **코어·궤도와 겹치는** 반경을 조용히 만들었다.
  satInset: 4.5,            // 코너에서 위성 중심까지(=모듈 4.5)
  orbitRadius: 8.5,          // 포맷 궤도 반경
  orbitDots: 24,             // 궤도 도트 수
  orbitDotR: 0.55,           // 궤도 도트 반경
  /* 데이터 도트 반경(라운드 채움). ★2026-09-02 (C7/P5) 0.42 → 0.50, 채움률 55% → 79%.
   *   근거: vs-QR 감사 — 저해상도 격차의 진범은 서브픽셀 정합이 아니라 **채움률**이다
   *   (`~/wiacode-vs-qr/RESEARCH.md` §3-3, `exp_fable/fill2.js`·`ber.js`).
   *   실측(라이브 경로 generate→detectAuto, 격자 L, 형상 10종 × 문구 3개):
   *     r 0.42  전건 해독 1.50 px/mod 까지 · r 0.46 = 0.42 와 동일(중간값 무의미) · r 0.50  1.25 px/mod 까지
   *   락 성공률은 세 값 모두 전 구간 100%.
   * ★첫 착수(Opus)는 foot/L 이 깨져 되돌렸다(hugon 493/496, 811·816·817·819·823·1000px 실패).
   *   Fable 이 셀 단위로 추적한 결과 도트 굵기가 아니라 **밀착 마름모 종횡비(foot 3.2)가 만든
   *   사영 H 의 가짜 원근**이 원인이었다 — 먼 셀에서 2.5~5px 오차, 0.42 는 도트 사이 흰 여백이
   *   그걸 흡수했고 0.50 은 도트가 맞닿아 못 흡수했을 뿐. build-core.js `_hugAttempt` 의
   *   **아핀 대안**(앵커 5점 자기투영)으로 근치 → foot 9/9. 그 뒤에 이 값을 올렸다.
   * ★청구항 1·4 에 0.42 가 기재돼 있다 — 바꾸는 것은 막히지 않는다(특허는 우리 구현을 얼리지
   *   않는다). `~/patent-wiacode/IMPROVEMENTS_AFTER_FILING.md` I2 에 기록해 후속출원에 얹는다. */
  dataDotR: 0.50,
};

const GRID_CELLS = { S: 64, M: 96, L: 128 };

// ── 다리QR 존 (2026-08-16, Fable5 설계) ────────────────────────────────────
//   ★포맷 상수 — 한 번 배포되면 절대 변경 금지. 바꾸면 이미 인쇄·저장된 다리QR
//   코드가 전부 해독 불능이 된다(스캐너는 이 값으로 "예약된 자리"를 재구성함).
//   모듈 단위(cellPx·quiet 무관)로 동결 — 픽셀 좌표는 이 값에서 항상 유도만 한다
//   (지금까지 픽셀 공식이 cellPx마다 값이 달라 디코더가 셀 집합을 재현 못 하던
//   문제를 근본적으로 없앰). 존 예약은 grid L에서만 정의(다리QR은 L 전용).
const BRIDGE = {
  grid: 'L',
  qrModules: 25,     // bridge-qr.js의 SIZE와 반드시 일치(런타임 가드로 검증)
  qMod: 0.7,          // 오르빗 모듈 per QR모듈 — 기존 cellPx10·qFrac0.15 실측 배치와 동일 밀도
  padQrMod: 0.35,     // 흰 quiet zone(모듈) — ★2026-09-02 1.4→0.35 (아래 §다리QR 배치 개정)
  topMy: 77.5,        // 표에 없는 형상의 기본 QR 상단 y(모듈) — 코어 바로 아래
  marginMod: 0.5,     // 예약 안전여백(데이터 도트 반경 + 샘플 오프셋 커버)
                      //   ★C7 로 도트 반경이 0.42→0.50 이 됐다. 여백 0.35 인 형상에서는
                      //     도트 잉크가 QR 흰 패드 경계에 딱 닿는다(0.85-0.50=0.35). 겹치지는
                      //     않지만 여유가 0 이라, 여기를 더 줄이려면 배터리부터 다시 돌릴 것.
};
/* ── 다리QR 배치 개정 (2026-09-02, 오너 착안) ──────────────────────────────
 *   문제: QR 자리가 **모든 형상에서 같았다**(가로중앙·topMy 77.5). 그 자리에 몸통이
 *   없는 실루엣(폐·부메랑·보청기·강아지…)은 QR 이 흰 배경 위에 홀로 떠서, 형상보다
 *   QR 이 먼저 눈에 들어왔다. 우리 목적은 반대다 — **형상을 각인시키고 QR 은 코드
 *   안에 희석시킨다.**
 *
 *   왜 옮겨도 되는가: QR 은 자기 파인더 패턴으로 스스로를 찾으므로 **일반 QR 리더는
 *   위치와 무관하게 읽는다.** 우리 디코더는 이미 형상 후보를 순회하며 형상마다
 *   layout() 을 부르므로(build-core.js Pass1/Pass2), 형상별 자리는 그 순회 안에서
 *   그대로 재구성된다 — **해독 비용 증가 0.**
 *
 *   실측(66종 전수, `~/wiacode-perf/` 도구): 지금 100% 가 아닌 37종 중 24종이 100%,
 *   12종이 크게 개선, 뇌만 87.0→90.1 로 제자리. 못 고치는 형상은 0종.
 *
 *   값 = **inner(QR 자체) 좌상단**의 모듈 좌표. pad 를 바꿔도 흔들리지 않도록 inner 를
 *   저장한다. 후보는 ①코어·궤도·위성 예약과 **겹치지 않고**(밀착/비밀착 두 배치 모두)
 *   ②실루엣 안에 든 비율 최대 ③동점이면 주변 도트 밀도(묻힘)가 높은 쪽으로 골랐다.
 *   ★재생성: `node ~/wiacode-perf/solve-bridge-pos.js`
 *
 *   ★표에 없는 형상은 예전 자리 그대로다(개선 5%p 미만이라 **잘 되는 걸 안 건드린다**).
 *   ★옛 발행분 호환: 자리와 pad 가 바뀌면 데이터 셀 집합이 달라져 옛 코드가 안 읽힌다.
 *   그래서 디코더는 `bridgeLegacy:true`(옛 자리 + 옛 pad 1.4)를 **한 번 더 시도**한다.
 *   그 시도는 표에 있는 형상에서만 추가되므로 나머지 32종은 비용이 그대로다. */
const BRIDGE_LEGACY = { padQrMod: 1.4, topMy: 77.5 };   // 2026-09-02 이전 발행분
/* ★여백 축소에서 제외하는 형상 (2026-09-03) — 아래 bridgeRect 주석 참조.
 *   square 는 WIA Stream 의 캐리어이고 그 프레임 계약(capBytes 1450·blockSize 1059)이
 *   동결돼 있다. 여백을 줄이면 용량이 올라가 계약이 어긋난다. 계약 개정과 함께가 아니면
 *   여기서 빼지 말 것. */
const BRIDGE_PAD_FROZEN = { square: true };
const BRIDGE_POS = {
  boomer:       [34.9, 49.9],   // 부메랑 0.0% → 94.8% (팔이 얇아 100%는 불가)
  lungs:        [80.9, 39.9],   // 폐 0.0% → 95.2%
  turtle:       [80.9, 78.9],   // 거북이 5.8% → 99.6%
  dog:          [54.9, 12.9],   // 강아지 7.6% → 100%
  hearingaid:   [31.9, 83.9],   // 보청기 18.2% → 100%
  smile:        [54.9, 22.9],   // 웃음 21.1% → 100%
  bone:         [34.9, 56.9],   // 뼈 22.7% → 100%
  embrace:      [54.9, 34.9],   // 포옹 44.2% → 100%
  whale:        [28.9, 52.9],   // 고래 49.8% → 81.8%
  gimbap:       [95.9, 45.9],   // 김밥 51.9% → 96.1%
  family:      [100.9, 48.9],   // 가족 52.1% → 95.2%
  piano:        [46.9, 26.9],   // 건반 52.3% → 93.2%
  butterfly:    [28.9, 38.9],   // 나비 55.0% → 95.7%
  sadness:      [54.9, 23.9],   // 슬픔 55.4% → 100%
  car:          [74.9, 62.9],   // 자동차 59.1% → 100%
  ear:          [42.9, 29.9],   // 귀 65.3% → 90.5%
  earth:        [77.9, 71.9],   // 지구 75.2% → 100%
  rose:         [48.9, 94.9],   // 장미 75.6% → 82.6%
  fish:         [45.9, 31.9],   // 물고기 76.2% → 100%
  grandpiano:   [34.9, 65.9],   // 그랜드피아노 77.3% → 100%
  dolhareubang: [30.9, 79.9],   // 돌하르방 78.3% → 95.5%
  deer:         [54.9, 96.9],   // 사슴 78.5% → 100%
  tree:         [55.9, 19.9],   // 나무 81.4% → 100%
  elevator:     [41.9, 97.9],   // 엘리베이터 81.8% → 100%
  love:         [23.9, 52.9],   // 사랑 84.7% → 100%
  arena:        [18.9, 67.9],   // 경기장 86.0% → 100%
  connection:   [54.9, 34.9],   // 연결 86.4% → 98.1%
  book:         [54.9, 90.9],   // 책 86.6% → 100%
  tooth:        [35.9, 69.9],   // 치아 86.8% → 100%
  rabbit:       [54.9, 87.9],   // 토끼 87.2% → 100%
  prayer:       [66.9, 76.9],   // 기도 88.8% → 99.4%
  lonely:       [64.9, 84.9],   // 외로움 92.8% → 100%
  eye:          [53.9, 74.9],   // 눈 93.2% → 99.0%
  oksign:       [47.9, 85.9],   // 오케이 94.0% → 100%
};
/* grid의 다리QR 존 사각형(모듈 좌표). grid L이 아니면 null.
 *   shape  — 형상별 자리표(BRIDGE_POS)를 쓴다. 없으면 예전 기본 자리.
 *   legacy — true 면 2026-09-02 이전 배치(가로중앙·topMy 77.5·pad 1.4)로 되돌린다. */
function bridgeRect(N, shape, legacy) {
  if (N !== GRID_CELLS[BRIDGE.grid]) return null;
  const c = N / 2, q = BRIDGE.qMod * BRIDGE.qrModules;
  const pos = (!legacy && shape) ? BRIDGE_POS[shape] : null;
  const inner = { mx0: pos ? pos[0] : c - q / 2,
                  my0: pos ? pos[1] : (legacy ? BRIDGE_LEGACY.topMy : BRIDGE.topMy), w: q, h: q };
  /* ★흰 여백 축소(1.4→0.35) — 2026-09-02 엔 **자리를 옮긴 34종에만** 걸었다.
   *   전 형상에 걸었다가 hugon-battery 가 foot/L 을 잡았기 때문이다(496 → 494). 원인은
   *   여백이 아니라 **그림이 달라진 것**이었다 — 여백이 줄면 데이터 셀이 늘고, foot/L 은
   *   "정확히 원본 크기(816px)에서 격자를 오판한다"는 칼날 위 사례라 그 미세한 변화에
   *   격자가 넘어가 해독이 죽었다. 그때 이 주석이 조건을 적어 뒀다: **"전 형상으로 넓히려면
   *   foot/L 격자 오판을 먼저 근치할 것."**
   *
   *   ★★2026-09-03 — 그 조건이 충족돼 넓힌다(오너 지시). C7(밀착 아핀 대안 + 원거리격자
   *   재적합, 09-02)이 격자 오판을 근치했고, 재현으로 확인했다: foot/L 을 정확히 816px 로
   *   줘도 격자를 L 로 맞힌다. 하위호환은 build-core.js 변형 2(옛 pad 1.4 재시도)가 받는다 —
   *   그쪽 게이트도 "옛 배치와 다른 형상 전부"로 같이 넓혔다(09-02 ① 사고의 정확한 처방).
   *
   *   ★square 만 예외다. Stream 프레임 계약(`capBytes 1450`·`blockSize 1059`,
   *   PILOT_B0_CORE_2026-08-23 §3)이 **square/L 에 동결**돼 있고, 여백을 줄이면 데이터 셀이
   *   늘어 capBytes 가 올라간다 → `stream-lab.html` 의 `d.capBytes !== CAP_BYTES` 가드가
   *   프레임을 거부한다. 계약을 고치는 건 별건이라 square 는 옛 여백을 그대로 둔다.
   *   부수 효과로 square 는 옛 배치와 완전히 같아져 변형 2 가 자동으로 꺼진다(해독 비용 0). */
  const p = (legacy || BRIDGE_PAD_FROZEN[shape]) ? BRIDGE_LEGACY.padQrMod : BRIDGE.padQrMod, m = BRIDGE.marginMod;
  const padded = { mx0: inner.mx0 - p, my0: inner.my0 - p, mx1: inner.mx0 + q + p, my1: inner.my0 + q + p };
  return { inner, padded, reserve: { mx0: padded.mx0 - m, my0: padded.my0 - m, mx1: padded.mx1 + m, my1: padded.my1 + m } };
}

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


/* ── 밀착(마름모) 풀이 ─────────────────────────────────────────────────────
 *   §J 참조. 축 A(12·6시)와 축 B(3·9시) 각각에서, **위성 원판+흰 고리가 채운 윤곽 안에
 *   완전히 들어가는 가장 큰 계단**을 고른다. 두 축은 독립이다 — 그래서 마름모가 된다.
 *   판정은 **채운 윤곽**(바깥에서 flood fill 해 닿지 않는 곳)이다: 안쪽 흰 무늬
 *   (뇌주름·경위선·장미의 로그나선 골)에 걸려 멀쩡한 형상이 탈락하던 것을 막는다. */
const _hugCache = new Map();
function _filledOutline(L, N) {
  const SS = 2, W = N * SS, ink = new Uint8Array(W * W);
  for (let y = 0; y < W; y++) for (let x = 0; x < W; x++)
    if (insideShape((x + 0.5) / SS, (y + 0.5) / SS, L)) ink[y * W + x] = 1;
  const out = new Uint8Array(W * W), st = [];
  for (let x = 0; x < W; x++) { st.push(x); st.push(x + (W - 1) * W); }
  for (let y = 0; y < W; y++) { st.push(y * W); st.push(y * W + W - 1); }
  while (st.length) {
    const i = st.pop();
    if (out[i] || ink[i]) continue;
    out[i] = 1;
    const x = i % W, y = (i - x) / W;
    if (x + 1 < W) st.push(i + 1);
    if (x > 0) st.push(i - 1);
    if (y + 1 < W) st.push(i + W);
    if (y > 0) st.push(i - W);
  }
  return { out, W, SS };
}
function solveRhombus(L, N, c, Rs, tiers, spec, bridgeLegacy) {
  const F = _filledOutline(L, N);
  const SR = (spec.satRadius || 2.5) + (spec.satHalo || 1.0);
  /* ★부스터 QR 예약 사각은 금지 구역이다 (2026-09-01).
   *   밀착은 위성을 코어 쪽으로 당기는데, 짧은 축이 ~25모듈 아래로 내려오면 그 자리가
   *   코어 바로 아래 QR 존(topMy 77.5, 예약 53.35~74.65 × 75.6~96.9)과 **겹친다.**
   *   생성기는 QR 을 항상 얹으므로(bridge:true) 흰 패드가 위성을 통째로 지워 **라이브에서
   *   락이 안 된다** — arena·whale·mouth(원판 전체), like·prayer·car·dove·connection(일부),
   *   실측 8종. hugon-battery 는 QR 을 안 얹고 재서 8일 동안 못 잡았다(이제 얹는다).
   *   실루엣 잉크만 보던 판정에 QR 예약 사각을 "잉크 아님"으로 더한다 → φ·반경 탐색이
   *   저절로 그 자리를 피한다. 격자 L 에만 존이 있다(bridgeRect 는 그 외 null). */
  //   ★2026-09-02: 자리가 형상별이 됐으므로 이 금지구역도 **그 형상의 자리**여야 한다.
  const _br = bridgeRect(N, L.shape, bridgeLegacy), _bz = _br && _br.reserve;
  const inF = (mx, my) => {
    if (_bz && mx >= _bz.mx0 && mx <= _bz.mx1 && my >= _bz.my0 && my <= _bz.my1) return false;
    const x = Math.floor(mx * F.SS), y = Math.floor(my * F.SS);
    if (x < 0 || y < 0 || x >= F.W || y >= F.W) return false;
    return !F.out[y * F.W + x];
  };
  const cov = (ang, R) => {
    let ok = 0, tot = 0;
    for (let dy = -SR; dy <= SR; dy += 0.5) for (let dx = -SR; dx <= SR; dx += 0.5) {
      if (dx * dx + dy * dy > SR * SR) continue;
      tot++;
      if (inF(c + R * Math.cos(ang) + dx, c + R * Math.sin(ang) + dy)) ok++;
    }
    return tot ? ok / tot : 0;
  };
  /* ★바닥 — 위성이 **궤도링을 침범하면 안 된다.**
   *   궤도 8.5 + 궤도점 0.9 + 위성 2.5 + 흰고리 1.0 + 여유 1.0 = 13.9. */
  const floor = (spec.orbitRadius || 8.5) + (spec.orbitDotR || 0.9)
              + (spec.satRadius || 2.5) + (spec.satHalo || 1.0) + 1.0;

  /* ★연속 밴드-톱 (2026-08-30, Fable 검토 뒤 계단 폐기) ────────────────────
   *   전에는 Rs×0.8^k 계단에서 골랐다. 계단을 버린 이유는 **커버리지가 비단조**라서다:
   *   hope 의 3·9시 통과대역은 R 52~54.5 의 **폭 3모듈짜리 밴드**인데 0.8 계단
   *   (60.5 → 48.4)이 그 위아래로 건너뛰어 통째로 놓친다. 계단을 없애니
   *   hope 짧은축 24.8 → 54.3 (**+119%**), star +20~24%, family +16~19%, 중앙값 ~13%.
   *   0.75/0.8/0.85 어느 고정비도 지배하지 못한다(0.85는 hope를 살리고 rose를 죽인다).
   *
   *   ★연속값이어도 정합은 자동이다 — 생성과 해독이 **같은 결정적 solver** 를 쓴다.
   *   ★다만 레이아웃 반경 허용오차가 **0.5~1.0%** 라(실측: brain/L 은 0.5%에서 해독 사망)
   *     solver 드리프트에 극도로 민감하다 → 확정값은 shape-registry 에 **핀으로 박는다**
   *     (아래 REG.hugPin). 핀이 있으면 이 탐색을 아예 안 돈다.
   *   ★밴드-톱은 95% 경계에 정확히 앉으므로 **0.5모듈 물러선다**(BACKOFF) — 경계에
   *     걸터앉으면 마스크 샘플링이 조금만 달라져도 넘어간다. */
  const STEP = 0.25, BACKOFF = 0.5;
  const scan = (a1, a2) => {
    for (let R = tiers[0]; R >= floor; R -= STEP) {
      if (cov(a1, R) >= 0.95 && cov(a2, R) >= 0.95) {
        // 경계에서 물러선다 — 물러선 자리도 통과해야 채택한다(밴드가 얇으면 그대로 둔다).
        const b = R - BACKOFF;
        if (b >= floor && cov(a1, b) >= 0.95 && cov(a2, b) >= 0.95) return b;
        return R;
      }
    }
    return null;
  };
  const bestTier = scan;
  /* ★φ(위성 십자의 회전)도 함께 찾는다 — 2026-08-30, 오너 착안.
   *   전에는 형상을 돌려서 위성 자리에 잉크를 갖다 댔다. 그러면 **사람이 기울어진 나비를 본다.**
   *   그럴 필요가 없다: 스캐너는 코어+위성으로 호모그래피를 잡으므로 **회전을 흡수한다.**
   *   그러니 **형상은 똑바로 두고 위성 십자만 φ 만큼 돌린다.**
   *   사람은 똑바른 나비를 보고, 위성은 잉크 위에 앉고, 코드 외곽은 정사각 그대로다.
   *   (butterfly: 형상을 47° 눕히던 것 → 형상 똑바로 + 위성 33° 로 47.0/55.3)
   *   ★φ 는 프레임에서 관측되지 않는다(전역 회전과 구별 불가) — 그래서 **핀에 박아** 두고,
   *     같은 (rA,rB) 를 쓰는 형상끼리는 readCode 의 CRC 가 가른다. 후보가 안 늘어난다. */
  const A0 = -Math.PI / 2;
  let best = null;
  for (let d = 0; d < 90; d += 1) {
    const ph = d * Math.PI / 180;
    const rA = bestTier(A0 + ph, A0 + ph + Math.PI);
    const rB = bestTier(A0 + ph + Math.PI / 2, A0 + ph + 3 * Math.PI / 2);
    if (rA == null || rB == null) continue;
    if (!best || rA + rB > best.rA + best.rB) best = { rA, rB, phi: d };
  }
  if (!best) return null;                                      // 밀착 불가 — 기본 배치로
  return best;
}

// ── 앵커/궤도 좌표 산출 ───────────────────────────────────────────────────
//   shape: 'square'(기본, 위성=4코너) | 'round'(위성=링 12/3/6/9시, 데이터판=원판).
//   matcher(locate)는 "코어를 둘러싼 4위성 + 북극성"만 보므로 두 모양 모두 같은 코드로 검출됨.
function layout(grid, spec, shape, opts) {
  spec = spec || SPEC; shape = shape || 'square';
  const N = GRID_CELLS[grid];
  const c = N / 2;                 // 코어 중심(모듈)
  let anchors, hugOn = false;
  if (shape !== 'square') {
    const Rs = N / 2 - (spec.satRimMargin || 3.5);   // 위성 링 반경(rim 안쪽) — 원/하트 공용
    let ANG = { TL: -Math.PI / 2, TR: 0, BR: Math.PI / 2, BL: Math.PI };  // 12/3/6/9시(시계방향)
    anchors = [{ name: 'core', type: 'bullseye', mx: c, my: c }];
    /* ★밀착 = **마름모** (2026-08-30, 오너 착안 — 사각에서 나온 원리) ─────────────
     *   사각의 위성 4개는 자기 꼭짓점 그 자체(r 84.1, ±45°)이고 8개월째 잘 읽힌다.
     *   거기서 나온 규칙: **긴 축 2개는 살리고 짧은 축 2개만 코어 쪽으로 당긴다.**
     *
     *   앵커 반경 = [R긴, R짧, R긴, R짧] — 마주 보는 짝이 같다.
     *   ① 점대칭이 정확히 유지 → 네 앵커 무게중심 = 코어 (dCen ≡ 0).
     *      방향별 4반경(이전 ±20° 부채꼴 정책)은 이 성질이 깨져 similarity 적합이 나빴다.
     *   ② 계단 = Rs × 0.8^k → 비율이 **정확히 1.25^m**(1.00 1.25 1.56 1.95 2.44 3.05).
     *      반경 측정오차 2.8% 대비 **9배 여유**. 이전 4벡터 시그니처는 최소 분리 2.1% 였다.
     *   ③ 앵커 각도는 **안 돌린다**(항상 12/3/6/9시). 형상 쪽을 미리 돌려서 굽는다 —
     *      각도를 앵커에 두면 φ 가 잠금 후보에 들어가 후보가 폭발한다.
     *   ④ 원(60.5,60.5)·사각(84.1,84.1)이 이 규칙의 특수해라 별도 분기가 필요 없다.
     *
     *   ★판정은 **채운 윤곽**으로 한다. 잉크 기준으로 재면 뇌주름·경위선·장미의 로그나선 골
     *     같은 **안쪽 흰 무늬**에 위성이 걸려 brain 36%·earth 21% 로 떨어진다(실측).
     *     사람이 프레임에 넣는 건 바깥 윤곽이므로 바깥에서 flood fill 한 채움이 기준이다.
     *   ★결과는 캐시한다 — layout() 은 스캔 1회에 수십 번 불린다. */
    const hug = !!(opts && opts.hug);
    let hugR = null;
    if (hug) {
      const tiers = [Rs];      // 연속 탐색의 **상한**만 쓴다(계단은 2026-08-30 폐기)
      const key = (opts && opts.customMask) ? null : (grid + '|' + shape);
      // ★핀 우선 — 레지스트리에 확정값이 있으면 탐색하지 않는다.
      //   ①solver 를 나중에 고쳐도 이미 발행된 코드가 안 깨진다(허용오차 0.5%)
      //   ②첫 스캔의 solver 비용(~1.2s)이 사라진다
      let got = null;
      if (key && REG && REG.hugPin) {
        const pin = REG.hugPin(shape, grid);
        if (pin) got = { rA: pin[0], rB: pin[1], phi: pin[2] || 0 };
      }
      if (!got) got = key ? _hugCache.get(key) : null;
      if (!got) {
        const tmpL = { N, coreCenter: { mx: c, my: c }, spec, shape, Rdata: N / 2 - 0.8 };
        if (opts && opts.customMask) tmpL.customMask = opts.customMask;
        got = solveRhombus(tmpL, N, c, Rs, tiers, spec, opts && opts.bridgeLegacy);
        if (key) _hugCache.set(key, got);
      }
      if (got) {
        hugR = { TL: got.rA, BR: got.rA, TR: got.rB, BL: got.rB };
        if (got.phi) {                       // 위성 십자만 돌린다 — 형상·데이터격자는 그대로
          const ph = got.phi * Math.PI / 180;
          for (const k of ['TL', 'TR', 'BR', 'BL']) ANG[k] += ph;
        }
      }
    }
    /* ★밀착이 성립하지 않으면 **플래그까지 끈다.**
     *   앵커만 기본으로 돌리고 L.hug 를 켜 두면 흰 고리(satHalo)만 그려져
     *   reservedAt/dataCells 가 비밀착 레이아웃과 어긋난다 — 생성은 되는데 해독이 안 된다.
     *   실측: rose 가 격자 M/S 에서 밀착 불가라 이 상태가 됐고 4/4 전부 실패했다. */
    hugOn = hug && hugR != null;
    for (const k of ['TL', 'TR', 'BR', 'BL']) {
      const R = hugR ? hugR[k] : Rs;
      anchors.push({ name: k, type: k === 'TL' ? 'donut' : 'disk', mx: c + R * Math.cos(ANG[k]), my: c + R * Math.sin(ANG[k]) });
    }
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
  const L = { N, coreCenter: { mx: c, my: c }, anchors, orbit, spec, shape, hug: hugOn, noGrooves: !!(opts && opts.noGrooves) };
  L.bridgeRect = (opts && opts.bridge) ? bridgeRect(N, shape, opts.bridgeLegacy) : null;
  // 'custom' 실루엣 마스크 전달(insideShape 참고). 다른 shape에선 무시된다.
  if (opts && opts.customMask) L.customMask = opts.customMask;
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

// ── 말풍선(bubble) 실루엣: 모서리 둥근 사각형 몸통(모서리 반경=몸통의 12%) + 좌하단
//   꼬리 삼각형 ─────────────────────────────────────────────────────────
//   ★2026-08-18 오너 확정(B안): 카디널 위성 앵커(정규화 반경 0.9573, grid L 기준)를
//   몸통 안에 "완전히" 담으려 하면(BUBBLE_H≈0.957) 그리드 하드 한계(≈1.005)까지 남는
//   여유가 0.05 안팎뿐이라 꼬리가 육안으로 거의 안 보인다(실측 확인). 대신 기존
//   clover 실루엣의 기존 전례(잎 4개, lobeD+lobeR=0.88 — 위성이 잎 밖 0.0773만큼
//   떠서 이미 라이브 중)와 "같은 급"으로 앵커 이탈을 맞춘다: BUBBLE_H=0.88 →
//   이탈량 0.9573-0.88=0.0773 (clover와 소수점 4자리까지 동일). 이 여유(0.125)로
//   좌하단에 실제로 알아볼 수 있는 삼각형 꼬리를 낸다. 위성은 clover와 동일한 원리로
//   몸통 밖 여백에 그대로 그려진다(inkAt은 insideShape와 무관 — 위치·판독 불변).
//   다리QR 존(코어 바로 아래 중앙 좁은 띠, ny≈0.18~0.52)과는 안 겹침(중앙 vs 좌하단).
var BUBBLE_H = 0.88, BUBBLE_R = BUBBLE_H * 0.12;
var BUBBLE_TAIL = [[-0.793, 0.878], [-0.878, 0.793], [-0.965, 0.965]];
function pointInRoundedSquare(nx, ny, H, r) {
  var qx = Math.abs(nx) - (H - r), qy = Math.abs(ny) - (H - r);
  var ax = Math.max(qx, 0), ay = Math.max(qy, 0);
  return Math.sqrt(ax * ax + ay * ay) + Math.min(Math.max(qx, qy), 0) - r <= 0;
}
function pointInTriangle(px, py, tri) {
  var sign = function (x1, y1, x2, y2, x3, y3) { return (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3); };
  var d1 = sign(px, py, tri[0][0], tri[0][1], tri[1][0], tri[1][1]);
  var d2 = sign(px, py, tri[1][0], tri[1][1], tri[2][0], tri[2][1]);
  var d3 = sign(px, py, tri[2][0], tri[2][1], tri[0][0], tri[0][1]);
  var hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0), hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);
  return !(hasNeg && hasPos);
}
function pointInBubble(nx, ny) {
  if (pointInRoundedSquare(nx, ny, BUBBLE_H, BUBBLE_R)) return true;
  return pointInTriangle(nx, ny, BUBBLE_TAIL);
}

// ── 말풍선 v2(chat) — 타원 몸통 + 좌하단 뿔 꼬리 ────────────────────────────
//   ★2026-08-27 신설. 기존 `bubble`(모서리 둥근 사각 + 얇은 꼬리)은 실물이 "꼬리 붙은
//   사각형"으로 읽혀 브랜드 형상으로 쓰기 어렵다는 오너 지적에 따라, **말풍선으로 즉시
//   읽히는** 형태를 새 키로 추가한다.
//   ★★`bubble` 은 절대 고치지 않는다 — 실루엣을 바꾸면 데이터 셀 집합·순서가 바뀌어
//   **이미 발행된 bubble 코드와 적합성 벡터 3종이 해독 불능**이 된다(생성 중단 ≠ 삭제).
//   그래서 `bubble` 은 레거시 해독용으로 그대로 두고, 생성 대상만 `chat` 으로 옮긴다.
//   기하: 가로로 넓은 타원(rx/ry = 1.23, 카카오톡 말풍선 비율과 같은 자릿수) + 좌하단에서
//   왼쪽 아래로 뻗는 삼각 꼬리. 꼬리 밑변 두 점은 타원 **안쪽**에 두어 이음매가 벌어지지
//   않게 했다(§7-29 교훈 1: 매끈하게 이어붙이면 특징이 죽는다 → 밑변은 붙이되 끝은 명백히
//   밖으로 내보낸다). 돌출량 = 몸통 세로 반경의 약 29%(bubble 꼬리 0.125·lid 0.17보다 크지만
//   말풍선은 꼬리가 정체성이라 의도적으로 키움).
//   ★위성 이탈: 북/남 위성(정규화 0.9573)이 타원 밖으로 0.177 뜬다 — heart/clover/star 가
//   이미 같은 성질을 갖고 있고 판독에는 무관하다(reservedAt 이 실루엣과 별개로 예약).
var CHAT_RX = 0.96, CHAT_RY = 0.78;
var CHAT_TAIL = [[-0.04, 0.778], [-0.40, 0.712], [-0.56, 1.01]];
function pointInChat(nx, ny) {
  if ((nx * nx) / (CHAT_RX * CHAT_RX) + (ny * ny) / (CHAT_RY * CHAT_RY) <= 1) return true;
  return pointInTriangle(nx, ny, CHAT_TAIL);
}

// ══════════════════════════════════════════════════════════════════════════
// ── 산업 실루엣 팩 파일럿 3종 (2026-08-26, 조사+파일럿 지시) ────────────────
//   ★★정품 미노출 — generate.html 드롭다운엔 추가하지 않음. WiaScan.generate()에
//   shape 이름을 직접 넘겨야만 접근되는 파일럿 전용 실루엣(갤러리 후보 보고용).
//   전부 "본체를 실제 반경(1.0)보다 살짝 줄이고 그 여유로 특징을 낸다" 원칙
//   (bubble의 BUBBLE_H=0.88 전례와 동일) — 그리드 하드 한계(≈1.0127, round의
//   ny∈[-1,1] 경계로 이미 room이 거의 없음)를 넘기지 않기 위함.
// ══════════════════════════════════════════════════════════════════════════

// ── 밥뚜껑(lid): 원판 본체 + 상단 손잡이 꼭지 ───────────────────────────────
//   꼭지 중심(0,-0.92)은 북극성 앵커(TL, ny=-0.957 축)와 같은 축 위에 둬서
//   "손잡이 자리 = 북극성 자리"로 자연스럽게 겹치게 한다(clover/bubble이 위성을
//   잎/꼬리 자리에 맞춘 것과 같은 원리). 몸통 반경 0.88은 bubble과 동일 여유값.
var LID_R = 0.88, LID_KNOB_CY = -0.92, LID_KNOB_R = 0.13;
function pointInLid(nx, ny) {
  if (nx * nx + ny * ny <= LID_R * LID_R) return true;
  const dx = nx, dy = ny - LID_KNOB_CY;
  return dx * dx + dy * dy <= LID_KNOB_R * LID_KNOB_R;
}

// ── 물고기(fish, 옛 "flatfish/넙치" — 2026-08-26 정식 개명): 타원 몸통 +
//   등/배지느러미 + 동쪽 제비꼬리(포크) ─────────────────────────────────────
//   ★개명 이유: 종 특정(넙치=flatfish/flounder 해부학) 실루엣이 아니라 "물고기"로
//   범용 인식되도록 설계됐다 — 이름을 실제 형태에 맞춤(오너 재가, 2026-08-26).
//   내부 식별자·키만 변경, 기하 자체는 무변경(pointInFish 왕복검증 그대로 재사용).
//   ★v1(타원+얕은 삼각패치)은 실루엣이 그냥 눌린 원으로 보여 기각(2026-08-26).
//   ★v2(타원+포크 꼬리만)는 꼬리는 살았지만 위성이 뜨는 상/하 축이 그냥 빈
//   허공에 뜬 점으로 보여 재검토(형 지시로 2차 다듬기, 2026-08-26). v3: §7-29에
//   적어둔 "위성이 뜨는 자리에 특징을 배치" 원칙을 여기도 적용 — 북/남 위성
//   축으로 뻗는 지느러미 삼각형을 몸통에 붙여 그 이탈을 등/배지느러미로
//   흡수한다(서쪽 위성은 그대로 눈처럼 남겨둠 — 그건 이미 잘 읽혔다).
//   포크는 FORK_START를 앞당기고 FORK_GAP을 키워 축소본에서도 더 또렷하게.
//   ★v4(2026-08-26 오너 재검수 불합격 교정): v3 는 소형(256px)에서 **형태가 성립하지
//   않았고** 용량 336B 로 전 실루엣 최저, 중심어긋남 5.8% 로 최악이었다. 원인 3가지:
//     ① 몸통이 작아 데이터가 안 들어감(밀도 부족 → 소형에서 흩어져 보임)
//     ② 질량이 왼쪽(몸통)에 쏠렸는데 꼬리만 오른쪽으로 뻗어 **무게중심이 밀림**
//     ③ 꼬리 포크가 가늘어 축소하면 사라짐
//   교정: 몸통을 크게 키우고(0.64→0.74) 중심을 오른쪽으로 당겨 균형을 맞추며,
//   꼬리를 **짧고 두껍게** 바꿔 256px 에서도 갈라짐이 남게 했다.
//   ★중심 정합은 **부호를 재서** 맞춘다(눈대중 금지). v4-a 는 x=-2.20·y=-2.51모듈로
//   왼쪽·위로 쏠렸다 — x 는 과교정이었고, y 는 **부스터 QR 예약이 아래쪽 질량을
//   빼가서** 생긴 구조적 쏠림이다. 둘 다 몸통 중심 이동으로 상쇄한다.
var FISH_CX = -0.065;                 // 몸통 중심 x — 꼬리 무게 상쇄
var FISH_CY = 0.045;                  // 몸통 중심 y — 부스터 예약(아래) 상쇄
var FISH_RX = 0.74, FISH_RY = 0.60;
var FISH_TAPER_X0 = 0.30, FISH_TAIL_TIP = 0.97;
var FISH_FORK_START = 0.34, FISH_FORK_GAP = 0.15;
var FISH_DORSAL = [[-0.22, -FISH_RY + 0.06], [0.02, -FISH_RY + 0.02], [-0.10, -0.82]];
var FISH_VENTRAL = [[-0.20, FISH_RY - 0.06], [0.00, FISH_RY - 0.02], [-0.10, 0.80]];
function pointInFish(nx, ny) {
  var bx = nx - FISH_CX, byy = ny - FISH_CY;
  // 몸통(타원) + 등/배지느러미
  if (bx <= FISH_TAPER_X0) {
    if ((bx * bx) / (FISH_RX * FISH_RX) + (byy * byy) / (FISH_RY * FISH_RY) <= 1) return true;
    return pointInTriangle(nx, ny, FISH_DORSAL) || pointInTriangle(nx, ny, FISH_VENTRAL);
  }
  // 꼬리 — 짧고 두껍게, 끝만 갈라진다
  if (bx > FISH_TAIL_TIP) return false;
  var halfAtX0 = FISH_RY * Math.sqrt(Math.max(0, 1 - (FISH_TAPER_X0 * FISH_TAPER_X0) / (FISH_RX * FISH_RX)));
  var frac = (bx - FISH_TAPER_X0) / (FISH_TAIL_TIP - FISH_TAPER_X0);
  var totalHalf = halfAtX0 * (1 - 0.45 * frac);        // 덜 좁아진다(두꺼운 꼬리)
  if (frac < FISH_FORK_START) return Math.abs(byy) <= totalHalf;
  var forkT = (frac - FISH_FORK_START) / (1 - FISH_FORK_START);
  var gapNow = FISH_FORK_GAP * forkT;
  var lobe = totalHalf - gapNow;
  if (lobe <= 0) return false;
  return Math.abs(byy) > gapNow && Math.abs(byy) <= gapNow + lobe;
}

// ── 피아노 건반(piano): 둥근 사각 몸통 + 하단 빗살(콤) 절개 ─────────────────
//   상단 KEYS_COMB_Y0까지는 통짜(건반 뒤판), 그 아래는 KEYS_TEETH개의 빗살로
//   나눠 각 주기의 KEYS_TOOTH_FRAC만 남기고 나머지를 파낸다(건반 사이 홈).
//   실루엣만으로 "건반열"임을 알아보게 하는 것이 목적 — 실제 흑백건반 배치는
//   재현하지 않는다(캡션 없이 연상만 목표, §질문의 품질기준).
//   ★v2(참조 실물 반영, 2026-08-26): v1은 "아래로 뻗은 빗살"이라 건반이 아니라
//   빗·성벽으로 보였다. 진짜 건반은 **위에서 내려다본 판**이다 —
//   ①전체가 사각 판 ②흰건반 경계가 **가는 세로 틈** ③검은건반이 **위에서 꽂힌 막대**,
//   그것도 2개-3개 묶음(옥타브 문법). 흑백 2단계뿐이라 검은건반은 **비움**으로 표현한다
//   (도트밭 위의 흰 막대 = 최대 대비, 형태가 가장 잘 읽힌다).
//   ★v3(2026-08-26 오너 재검수 불합격 교정): v2 는 소형(256px)에서 **세로 줄무늬**로만
//   읽혔고 불스아이가 줄무늬 사이에 끼어 "건반 끝으로 쏠린" 인상을 줬다. 세 가지 교정:
//     ① 흰건반 14→**7개**(1옥타브) — 소형에서 건반 하나가 2배 넓어진다(최대 효과)
//     ② **중앙 가로띠를 통짜로** — 검은건반은 위쪽에서만, 흰건반 홈은 아래쪽에서만
//        파서 중앙에 잉크 띠를 남긴다. 불스아이가 덩어리 위에 앉는다(쏠림 해소)
//        + 실제 건반을 위에서 본 모습과도 같다(검은건반 위, 흰건반 홈 아래)
//     ③ 검은건반을 넓고 깊게 — 2개·3개 묶음(옥타브 문법)이 소형에서도 읽히게
//   ★v3-b 재교정: v3-a 는 여전히 통짜 줄무늬였다. 원인 — **검은건반과 흰건반 홈이
//     같은 x(경계)** 라 위아래 비움이 한 줄로 이어져 중앙 띠가 안 보였다.
//     건반은 **검은건반의 2-3 묶음**으로 알아보는 것이므로 흰건반 홈을 거의 없애고
//     (0.016→0.007) 검은건반을 크게(0.058→0.082) 키웠다.
//   ⚠️남는 한계: 진짜 검은건반은 **어둡지만** 우리는 비움(밝음)뿐이라 흑백이 뒤집힌다.
//     셀을 강제로 채우면 데이터가 깨지므로 이건 원리적으로 못 고친다.
var KEYS_H = 0.90, KEYS_R = 0.05;
var KEYS_WHITE = 7;                     // 흰건반 수(1옥타브) — 소형 가독성 최우선
var KEYS_GAP = 0.007;                   // 흰건반 경계 홈 — 아주 가늘게(아래 주석)
var KEYS_BLACK_W = 0.082;               // 검은건반 반폭 — 크게(소형 가독성의 핵심)
//   ★검은건반은 **건반 전체 길이를 쓰지 않는다** — 실제 피아노도 뒤쪽에 뒷판(rail)이
//   남는다. 위를 KEYS_BLACK_TOP 에서 시작해 그 위는 통짜로 두면 ①더 정확하고
//   ②위쪽 질량이 보존돼 무게중심이 아래로 쏠리지 않는다(v1.2-b).
var KEYS_BLACK_TOP = -0.62;             // 검은건반 시작(이 위는 뒷판=통짜)
var KEYS_BLACK_Y = -0.05;               // 검은건반이 내려오는 하한(중앙 위에서 멈춤)
var KEYS_BAND_LO = -0.05, KEYS_BAND_HI = 0.10;   // ★중앙 통짜 띠(불스아이 안착)
function pointInPianoKeys(nx, ny) {
  if (!pointInRoundedSquare(nx, ny, KEYS_H, KEYS_R)) return false;
  // ★중앙 가로띠는 무조건 잉크 — 불스아이·오르빗이 덩어리 위에 앉는다
  if (ny >= KEYS_BAND_LO && ny <= KEYS_BAND_HI) return true;
  var span = 2 * KEYS_H, w = span / KEYS_WHITE;
  var u = (nx + KEYS_H) / w;
  var k = Math.floor(u);
  if (ny > KEYS_BAND_HI) {
    // 아래쪽: 흰건반 경계 홈
    var edge = Math.round(u);
    if (edge > 0 && edge < KEYS_WHITE && Math.abs(u - edge) * w < KEYS_GAP) return false;
    return true;
  }
  // 위쪽 뒷판(rail) — 통짜로 남긴다
  if (ny < KEYS_BLACK_TOP) return true;
  // 검은건반(비움) — 흰건반 **경계에 중앙 대칭**으로 놓는다.
  //   ★버그 수정(2026-08-26, 오너 지적): 예전엔 k=floor(u) 로 잡고 |u-k| 를 재서
  //   경계의 **오른쪽 절반만** 비웠다. 그래서 검은건반이 ①폭이 의도의 절반이고
  //   ②한쪽으로 치우쳐, 가운데 두 개가 서로 붙어 보였다.
  //   지금은 가장 가까운 경계(round)를 기준으로 양쪽 대칭으로 비운다 → 폭이 균일하다.
  var edge = Math.round(u);
  if (edge > 0 && edge < KEYS_WHITE) {
    var inOct = ((edge % 7) + 7) % 7;
    if (inOct === 1 || inOct === 2 || inOct === 4 || inOct === 5 || inOct === 6) {
      if (Math.abs(u - edge) * w < KEYS_BLACK_W) return false;
    }
  }
  return true;
}

// ── 산업-4. rose (장미) ★lid 대체 간판 — 수제 정본 ────────────────────────
//   오너 참조의 **미학 방향만** 따른다(그 이미지는 비데이터 일러스트다). 여기서는
//   불스아이 1개·부스터 포함·실데이터로 우리 문법으로 재창조한다.
//   기법 = **내부 백선(void)** — 규격 v1.1/v1.2 의 속살형 조항 그대로:
//     ① 물결 외곽(꽃잎 끝) ② 로그나선 골을 비워 꽃잎을 가른다 ③ 중앙은 잉크.
//   ★골 폭을 **반경에 반비례**하게 잡는 것이 핵심이다 — 고정 위상폭으로 하면
//   중심으로 갈수록 골이 얇아져 3셀 미만이 되고(v1.1 위반) 나선이 뭉갠다.
//   ★v2 튜닝(256px 소형 게이트 통과용): 나선 바퀴수를 2.0→2.8 로 늘리고(b↓)
//   외곽 물결을 깊게(A↑), 골을 살짝 넓혀(3.8셀) 축소본에서도 나선이 읽히게 했다.
var ROSE_LOBES = 9;             // 외곽 물결(꽃잎 끝) 개수
var ROSE_R0 = 0.85, ROSE_LOBE_A = 0.080;
var ROSE_B = 0.060;             // 로그나선 성장률 — 중심~가장자리 약 2.8바퀴
var ROSE_GROOVE_HALF = 0.030;   // 골 반폭(정규화) ≈ 3.8셀
var ROSE_CORE = 0.30;           // 이 반경 안은 통짜 잉크(중앙 30% 존 규칙)
function pointInRose(nx, ny) {
  // ★180° 회전(오너 확정 2026-08-27) — 회전을 **기하에 구워 넣는다.**
  //   §1-1: 방향 정본은 엔진 출력 그대로이고, 렌더·표시 계층에서 임의 회전 금지다.
  //   그래서 CSS/캔버스 transform 이 아니라 여기서 좌표를 뒤집는다(= θ+π).
  nx = -nx; ny = -ny;
  var r = Math.sqrt(nx * nx + ny * ny);
  var th = Math.atan2(ny, nx);
  // ① 물결 외곽
  var R = ROSE_R0 + ROSE_LOBE_A * Math.cos(ROSE_LOBES * th);
  if (r > R) return false;
  // ③ 중앙은 잉크 — 불스아이가 점 구름 속에 앉는다
  if (r <= ROSE_CORE) return true;
  // ② 로그나선 골(void). 위상 f 의 골 반폭을 반경에 반비례로 잡아 **골의 실폭을 일정하게**
  var g = ROSE_GROOVE_HALF / (r * 2 * Math.PI * ROSE_B);
  if (g > 0.34) g = 0.34;                         // 안쪽에서 골이 꽃잎을 삼키지 않도록
  var phase = (Math.log(r) / ROSE_B - th) / (2 * Math.PI);
  var f = phase - Math.floor(phase);
  var d = f < 0.5 ? f : 1 - f;
  if (d < g) return false;
  return true;
}

// ── H-1. ear (귀) ★인체 팩 최우선 ─────────────────────────────────────────
//   외이(pinna) 윤곽: 바깥 타원(helix) − 안쪽 절개(concha 그늘) + 이수(귓불).
//   서 위성 = 이주(tragus) 자리, 북 위성 = helix 상단 너머.
//   ★v2(2026-08-26 자체 재검토): v1(타원−얇은 초승달)은 그냥 도넛/"0"으로 읽혀 기각.
//   교훈 — 귀의 연상성은 **오목한 그릇(concha)이 앞쪽으로 열린 C자**에서 온다.
//   그 void 는 얇으면 안 되고 **굵어야** 형태가 읽힌다(hex 대비 실루엣 면적 손해를 감수).
//   구성: 비대칭 달걀(위 넓고 아래로 좁아짐) + 귓불 + 앞(동쪽)으로 열린 굵은 C 절개 +
//   이주(tragus) 돌기. 서 위성 = helix 뒤쪽 능선, 북 위성 = helix 상단 너머(§11 원칙 2).
function pointInEar(nx, ny) {
  // ── 외곽: 위가 넓고 아래로 갈수록 좁아지는 달걀(귀 윤곽)
  var t = (ny + 0.92) / 1.84;                        // 0(위)~1(아래)
  var halfW = 0.62 * (1 - 0.42 * Math.pow(Math.max(0, t - 0.35) / 0.65, 1.7));
  var cxShift = -0.10 + 0.16 * t;                     // 아래로 갈수록 살짝 앞으로
  var inOuter = (ny >= -0.92 && ny <= 0.66) && Math.abs(nx - cxShift) <= halfW;
  // 위쪽 둥근 마감
  if (ny < -0.62) {
    var tx = (nx - (-0.06)) / 0.60, ty = (ny + 0.62) / 0.32;
    inOuter = (tx * tx + ty * ty) <= 1;
  }
  // ── 귓불: 아래로 늘어진 둥근 덩어리(외곽과 이어짐)
  var lx = (nx - 0.06) / 0.30, ly = (ny - 0.70) / 0.26;
  var inLobe = (lx * lx + ly * ly) <= 1;
  if (!inOuter && !inLobe) return false;

  if (inLobe && !inOuter) return true;                // 귓불은 통짜

  // ── ★v3(참조 실물 반영): 귀는 **2겹**이다 — 바깥 이륜(helix) 테두리와
  //   안쪽 대이륜(antihelix) 능선. v2는 C 한 겹뿐이라 평평했다.
  //   타원 반경 er 로 층을 나누고, 각 층의 틈을 **앞(동쪽)만 남기고** 파낸다.
  var ex2 = (nx + 0.02) / 0.60, ey2 = (ny + 0.02) / 0.86;
  var er = Math.sqrt(ex2 * ex2 + ey2 * ey2);
  var th = Math.atan2(ey2, ex2);                      // 0 = 동(앞)

  // 1) 이륜 틈 — 테두리 한 겹을 띄운다(뒤쪽만 비움 → 앞은 붙어 있어 통짜로 이어짐)
  if (er > 0.62 && er < 0.80 && Math.abs(th) > 0.78) return false;
  // 2) 이갑개(concha) — 중앙의 굵은 C, 앞으로 열림
  if (er > 0.17 && er < 0.47 && Math.abs(th) > 0.62) return false;

  // ── 이주(tragus): 앞쪽 입구를 살짝 가리는 작은 돌기(항상 솔리드)
  var gx = (nx - 0.34) / 0.13, gy = (ny - 0.10) / 0.17;
  if (gx * gx + gy * gy <= 1) return true;
  return true;
}

// ── H-2. palm (손바닥 + 다섯 손가락) ──────────────────────────────────────
//   손바닥 둥근 사각 + 손가락 4개(위) + 엄지(서). 가운뎃손가락 끝이 북 위성에 닿는다.
//   ★v2(2026-08-26 자체 재검토): v1 은 손가락이 짧고 손바닥에 묻혀 그냥 덩어리로 읽혔다.
//   교훈 — 손의 연상성은 **길이와 사이 틈**에서 온다. 손가락을 위성 축까지 길게 뽑고
//   (가운뎃손가락 끝이 북 위성 자리, §11 원칙 2) 손바닥을 줄여 틈을 확보한다.
//   ★v3(참조 실물 반영): v2는 손가락 4개가 같은 길이·같은 굵기라 "빗" 처럼 보였다.
//   진짜 손의 인상은 **길이 서열**(중지>검지≈약지>새끼)과 **새끼손가락이 낮게 붙는 것**,
//   그리고 **엄지가 손바닥 옆에서 비스듬히 갈라지는 것**에서 온다. 셋 다 반영.
//   중지 끝 = 북 위성 축, 엄지 끝 = 서 위성 방향(§11 원칙 2).
var PALM_FINGERS = [
  { x: -0.30, tip: -0.70, w: 0.086 },   // 검지
  { x: -0.08, tip: -0.90, w: 0.090 },   // 중지 — 가장 길다(북 위성 축)
  { x:  0.14, tip: -0.75, w: 0.086 },   // 약지
  { x:  0.34, tip: -0.47, w: 0.078 },   // 새끼 — 짧고 낮게
];
// 선분까지의 거리(캡슐 판정용) — 엄지처럼 비스듬한 부재에 쓴다.
function segDist2(px, py, ax, ay, bx, by) {
  var vx = bx - ax, vy = by - ay, wx = px - ax, wy = py - ay;
  var t = (wx * vx + wy * vy) / (vx * vx + vy * vy);
  t = t < 0 ? 0 : (t > 1 ? 1 : t);
  var dx = wx - t * vx, dy = wy - t * vy;
  return dx * dx + dy * dy;
}
function pointInPalm(nx, ny) {
  // 손바닥(넓게 — 참조처럼 손목까지 내려온다)
  if (pointInRoundedSquare(nx - 0.00, ny - 0.34, 0.46, 0.15)) return true;
  // 손가락 4개(세로 캡슐, 끝 둥글게)
  for (var i = 0; i < PALM_FINGERS.length; i++) {
    var f = PALM_FINGERS[i];
    var capY = f.tip + f.w;
    if (nx > f.x - f.w && nx < f.x + f.w && ny >= capY && ny < 0.02) return true;
    var dx = nx - f.x, dy = ny - capY;
    if (dx * dx + dy * dy <= f.w * f.w) return true;
  }
  // 엄지: 손바닥 왼쪽 옆구리에서 서-북서로 갈라진다(캡슐)
  if (segDist2(nx, ny, -0.36, 0.30, -0.84, -0.04) <= 0.105 * 0.105) return true;
  return false;
}

// ── H-3. footprint (발바닥 — 신생아 발도장) ───────────────────────────────
//   발바닥 본체(앞볼 넓고 뒤꿈치 좁은 타원 2개 결합) + 발가락 5개(위, 분리).
//   ★v2(참조 실물 반영): v1은 발가락 5개가 비슷한 크기라 "점 다섯"으로 보였다.
//   발도장의 인상은 ①**엄지가 확연히 크고** ②나머지가 **사선으로 작아지며**
//   ③아치가 **안쪽 한쪽만** 파이는 것에서 온다. 셋 다 반영.
var FOOT_TOES = [
  { x: -0.30, y: -0.76, rx: 0.165, ry: 0.185 },   // 엄지 — 확연히 크다
  { x:  0.01, y: -0.82, rx: 0.103, ry: 0.100 },
  { x:  0.22, y: -0.75, rx: 0.090, ry: 0.088 },
  { x:  0.39, y: -0.64, rx: 0.077, ry: 0.076 },
  { x:  0.52, y: -0.50, rx: 0.066, ry: 0.066 },   // 새끼 — 가장 작고 가장 낮다
];
function pointInFootprint(nx, ny) {
  // 앞볼(넓고 둥글다)
  var bx = nx - 0.06, by = ny + 0.30;
  if ((bx * bx) / (0.44 * 0.44) + (by * by) / (0.31 * 0.31) <= 1) return true;
  // 아치 — 중심을 **바깥쪽(오른쪽)으로 밀어** 안쪽 모서리만 오목하게 판다
  var ax = nx - 0.17, ay = ny - 0.16;
  if ((ax * ax) / (0.25 * 0.25) + (ay * ay) / (0.44 * 0.44) <= 1) return true;
  // 뒤꿈치(앞볼보다 좁고 둥글다)
  var hx = nx - 0.05, hy = ny - 0.62;
  if ((hx * hx) / (0.30 * 0.30) + (hy * hy) / (0.31 * 0.31) <= 1) return true;
  // 발가락 5개(본체와 떨어져 있는 게 실제 발도장 문법)
  for (var i = 0; i < FOOT_TOES.length; i++) {
    var t = FOOT_TOES[i], dx = nx - t.x, dy = ny - t.y;
    if ((dx * dx) / (t.rx * t.rx) + (dy * dy) / (t.ry * t.ry) <= 1) return true;
  }
  return false;
}

// ── H-4. brain (뇌 반구 + 주름) ───────────────────────────────────────────
//   위가 부푼 타원 + 좌우를 가르는 세로 틈(대뇌 종렬) + 물결 주름 절개 + 아래 뇌간.
function pointInBrain(nx, ny) {
  // 뇌간(아래로 짧게)
  if (Math.abs(nx - 0.06) <= 0.11 && ny > 0.62 && ny < 0.92) return true;
  var ex = nx / 0.94, ey = (ny + 0.06) / 0.80;
  if (ex * ex + ey * ey > 1) return false;
  // 대뇌 종렬(가운데 세로 틈) — 위쪽 절반만
  if (Math.abs(nx - 0.02) < 0.035 && ny < 0.30) return false;
  // 주름: 사인파 등고선을 따라 얇게 파낸다
  var g1 = Math.sin(nx * 6.1 + ny * 2.3);
  var g2 = Math.sin(ny * 7.4 - nx * 1.9);
  if (Math.abs(g1) < 0.13 && ny < 0.52) return false;
  if (Math.abs(g2) < 0.11) return false;
  return true;
}

// ── 모양 마스크: 데이터 판의 실루엣 (검출 원리와 무관 — 앵커는 그대로) ────────
//   'round'=원판, 'heart'=하트, 그 외=전부(사각). 정규화 좌표(중심0, 반경 Rdata=1).
// ★§7-36 방어 ② 보조표 — 실루엣 → 기하 함수. 없는 것은 null 로 남아 호출되지 않는다.
//   함수 선언은 호이스팅되므로 이 리터럴이 평가될 때 이미 존재한다.
var SHAPE_POINT_FN = {
  'ear':       typeof pointInEar       === 'function' ? pointInEar       : null,
  'palm':      typeof pointInPalm      === 'function' ? pointInPalm      : null,
  'footprint': typeof pointInFootprint === 'function' ? pointInFootprint : null,
  'brain':     typeof pointInBrain     === 'function' ? pointInBrain     : null,
};

// ── 비트맵 실루엣 (2026-08-28) ─────────────────────────────────────────────
//   기존 실루엣은 전부 수식이다(pointInRose 는 cos/sin). 그런데 귀·손·발·뇌처럼
//   **사람이 알아보는** 형상은 수식으로 못 쓴다 — 특히 형상을 살리는 **흰 통로**가
//   자유곡선이라 더 그렇다. 그래서 128×128 비트맵을 base64 로 구워 넣는다.
//   격자 L 과 같은 해상도라 L 에서 정확하고, S/M 은 정규화 좌표로 자연스럽게 내려간다.
//   한 형상당 약 2.7KB. 마스크 생성: ~/wiacode-shape-lab/mask-to-geometry.js
//
//   ★★흰 통로가 형상을 만든다 — 통짜로 메우면 "비슷한 덩어리"가 되고, 골이 있어야
//     "아, 귀네"가 된다. 장미의 `② 로그나선 골(void)`과 같은 원리다.
//     마스크에 구멍을 뚫으면 그 칸은 데이터 칸에서 빠져 흰색으로 남는다.
function decodeMask(n, b64) {
  var raw = typeof Buffer !== 'undefined' ? Buffer.from(b64, 'base64')
          : (function () { var s = atob(b64), a = new Uint8Array(s.length);
              for (var i = 0; i < s.length; i++) a[i] = s.charCodeAt(i); return a; })();
  var bits = new Uint8Array(n * n);
  for (var i = 0; i < n * n; i++) bits[i] = (raw[i >> 3] >> (7 - (i & 7))) & 1;
  return { n: n, bits: bits };
}
/** 정규화 좌표 → 비트맵 조회. 마스크 밖은 false(=데이터 없음). */
function inMask(M, nx, ny) {
  var ix = Math.floor((nx + 1) * 0.5 * M.n), iy = Math.floor((ny + 1) * 0.5 * M.n);
  if (ix < 0 || iy < 0 || ix >= M.n || iy >= M.n) return false;
  return !!M.bits[iy * M.n + ix];
}
// ── ear — 비트맵 실루엣(128×128, 잉크 25.8%)
//   원본 마스크: ear-v6.png
var MASK_EAR = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAgAAAAAAAAAAAAAAAAAAA//+AAAAAAAAAAAAAAAAAA///4AAAAAAAAAAAAAAAAA////gAAAAAAAA' +
  'AAAAAAAAf///+AAAAAAAAAAAAAAAA/////8AAAAAAAAAAAAAACf//v//gAAAAAAAAAAAAAB//PAf/+AAAAAAAAAAAAAA//xg' +
  'C//wAAAAAAAAAAAAA//YAAEP+AAAAAAAAAAAAAf/CAAAD/4AAAAAAAAAAAAH/wgCUA/+AAAAAAAAAAAAD/kIf/8GfwAAAAAA' +
  'AAAAAA/wD///zH+AAAAAAAAAAAAf+A////wfwAAAAAAAAAAAf+AP///+D+AAAAAAAAAAAH/Af////gfwAAAAAAAAAAD/gP//' +
  '//8D8AAAAAAAAAAA/4P/////AfgAAAAAAAAAAP8H/////+D8AAAAAAAAAAH8D//////gfAAAAAAAAAAB/A///8P/+D4AAAAA' +
  'AAAAA/wf//8Af/g+AAAAAAAAAAP8P///AD/8PwAAAAAAAAAD8D///AAP/j8AAAAAAAAAB/B///wAD/8fAAAAAAAAAAfwf//+' +
  'IAf/H4AAAAAAAAAH0P///zAD/4+AAAAAAAAAB4D////wAP+HgAAAAAAAAA+Af////AB/h4AAAAAAAAAHwH////wAf8PAAAAA' +
  'AAAAB8D////+AP/DwAAAAAAAAAfB//+f/gD/wcAAAAAAAAAPwf/wAJsAf+HAAAAAAAAABgH/8AABAD/h4AAAAAAAAAYB//gA' +
  'AAA/4fAAAAAAAAAGAf//AAAAP+HwAAAAAAAABgH///wAAH/x4AAAAAAAAA4B////4AA/+OAAAAAAAAAGAf////AAP/BwAAAA' +
  'AAAABwD////4AB/wcAAAAAAAAA+A/////wAf8PAAAAAAAAAHwP////8AH/BwAAAAAAAAB8B/////gB/wcAAAAAAAAAPgf///' +
  '/4AP8HAAAAAAAAAD4D/////gD/BwAAAAAAAAAfA/4///4Af4cAAAAAAAAAHwH8BBH/AH8PAAAAAAAAAB8A/AAA/wD/DwAAAA' +
  'AAAAAPAH4AAD8A/w8AAAAAAAAAB4A/8AA/gf8PAAAAAAAAAAfgH/AAf4D/DwAAAAAAAAAD4A/4AH+A/48AAAAAAAAAA+AP+H' +
  'j/gP8PAAAAAAAAAAPwD/3//4H/HwAAAAAAAAAB+Af///+B/x4AAAAAAAAAAHwH//+fgP4fAAAAAAAAAAA4D///n4D+HgAAAA' +
  'AAAAAAOA////+B/hwAAAAAAAAAABgf////gf4cAAAAAAAAAAAMH////4H+HAAAAAAAAAAABB///x+B/BwAAAAAAAAAAAQP//' +
  '8fgfwcAAAAAAAAAAACD/j8/wH4HAAAAAAAAAAAAgf4AP8D+DwAAAAAAAAAAAYD/AB/A/g4AAAAAAAAAAAHA/wA/gP4eAAAAA' +
  'AAAAAABwP8Af4D8fgAAAAAAAAAAAcD/9/8B/H4AAAAAAAAAAAHA////Afx+AAAAAAAAAAAB4P///gH//AAAAAAAAAAAAcD//' +
  '/wB8/wAAAAAAAAAAAHh///4D/H4AAAAAAAAAAADw///4A/j8AAAAAAAAAAAA4P//4Afw/AAAAAAAAAAAAOH//gAP8PwAAAAA' +
  'AAAAAAHj//AAH+D4AAAAAAAAAAAB4//gAH/B+AAAAAAAAAAAAff/wAD/wfAAAAAAAAAAAAPv/4AB/wPwAAAAAAAAAAADj/8A' +
  'Af8f4AAAAAAAAAAAA4f+AAf+H8AAAAAAAAAAAAOH/gAP/B/AAAAAAAAAAAADh/wPH/w/gAAAAAAAAAAAA4P4X//4PwAAAAAA' +
  'AAAAAAPD8P//+H4AAAAAAAAAAAADwPD///h8AAAAAAAAAAAAB8AB///wfAAAAAAAAAAAAAPAB///4PgAAAAAAAAAAAAH4A//' +
  '/4HwAAAAAAAAAAAAB/H///8D4AAAAAAAAAAAAAf////+B8AAAAAAAAAAAAAH/////A+AAAAAAAAAAAAAB/5///wfgAAAAAAA' +
  'AAAAAAf+f//8PwAAAAAAAAAAAAAH/n//8H4AAAAAAAAAAAAAB/////P+AAAAAAAAAAAAAAf///+P/AAAAAAAAAAAAAAH/n//' +
  'j/gAAAAAAAAAAAAAB/5//9/4AAAAAAAAAAAAAAf+P///8AAAAAAAAAAAAAAH/D///+AAAAAAAAAAAAAAA/x////gAAAAAAAA' +
  'AAAAAAP/////wAAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAAf////8AAAAAAAAAAAAAAAH//AH+AAAAAAAAAAAAAAAA//wB' +
  '/gAAAAAAAAAAAAAAAP/8If4AAAAAAAAAAAAAAAD//j/8AAAAAAAAAAAAAAAAf/4/+AAAAAAAAAAAAAAAAH////AAAAAAAAAA' +
  'AAAAAAA////gAAAAAAAAAAAAAAAAD///wAAAAAAAAAAAAAAAAAf//4AAAAAAAAAAAAAAAAAB//4AAAAAAAAAAAAAAAAAAH/8' +
  'AAAAAAAAAAAAAAAAAAAPyAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── hand — 비트맵 실루엣(128×128, 잉크 33.1%)
//   원본 마스크: ns1.png
var MASK_HAND = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAQAAAAAAAAAAAAAAAAAAAAeAAAAAAAAAAAAAAAAAAAAP4AAAAAAAAAAAAAAAAAAAH+AAAAAAAAA' +
  'AAAAAAAAAAB/wAAAAAAAAAAAAAAAAAAA/8AAAAAAAAAAAAAAAAAAAP/AAAAAAAAAAAAAAAAAAAD/4APAAAAAAAAAAAAAAAAA' +
  '/+AP4AAAAAAAAAAAAAAAAP/gD/AAAAAAAAAAAAAAAAD/4B/wAAAAAAAAAAAAAfAA/+Af8AAAAAAAAAAAAAP4AP/gH/AAAAAA' +
  'AAAAAAAH+AD/4B/wAAAAAAAAAAAAB/wA/+Af8AAAAAAAAAAAAAf8AP/gH/AAAAAAAAAAAAAH/AD/wB/wAAAAAAAAAAAAB/wA' +
  '/+Af8AAAAAAAAAAAAAf8AP/gP/AAAAAAAAAAAAAH/gD/4D/wAAAAAAAAAAAAB/4A/+Af8AAAAAAAAAAAAAf+AP/gP/AAAAAA' +
  'AAAAAAAH/gD/4D/wAAAAAAAAAAAAB/4A/+A/8AAAAAAAAAAAAAf+AP/gP/AAAAAAAAAAAAAH/wD/4D/wABAAAAAAAAAAA/8A' +
  '/+A/8AB8AAAAAAAAAAP/AP/gf/AA/gAAAAAAAAAD/wD/4H/wAf4AAAAAAAAAA/+A/+B/8AH+AAAAAAAAAAP/gP/gf/AB/gAA' +
  'AAAAAAAD/4D/8H/wAf4AAAAAAAAAA/+A//B/4AP+AAAAAAAAAAP/wP/wf+AD/gAAAAAAAAAB/8D/8P/gA/wAAAAAAAAAAf/A' +
  '//D/4Af8AAAAAAAAAAH/4P/w/+AH/AAAAAAAAAAB/+D/8P/gB/wAAAAAAAAAAf/g//H/4A/4AAAAAAAAAAH/8P/x/8AP+AAA' +
  'AAAAAAAA//D/8f/AH/gAAAAAAAAAAP/wf/H/wB/4AAAAAAAAAAD/8H/x/8Af8AAAAAAAAAAA//B/8f/AP/AAAAAAAAAAAP/w' +
  'f/H/wD/wAAAAAAAAAAB/+H/x/4B/4AAAAAAAAAAAf/h/8f+Af+AAAAAAAAAAAH/4f/H/gP/gAAAAAAAAAAB//P/7/4D/wAAA' +
  'AAAAAAAAP/////+B/8AAAAAAAAAAAD//////gf/AAAAAAAAAAAA//////4H/gAAAAAAAAAAAP/////+D/4AAAAAAAAAAAD//' +
  '////w/+AAAAAAB/AAAA//////+f/AAAAAAA/8AAAP////////wAAAAAAf/wAAD////////4AAAAAAH//AAA////////+AAAA' +
  'AAB//4AAP////////gAAAAAAP/+AAD////////wAAAAAAB//wAA////////8AAAAAAAP/+AAP////////AAAAAAAB//gAB//' +
  '//////wAAAAAAAP/8AA////////8AAAAAAAB//gAP////////AAAAAAAAf/8AD////////wAAAAAAAD//AA////////4AAAA' +
  'AAAA//4Af///////+AAAAAAAAH//AH////////gAAAAAAAB//wD////////4AAAAAAAAP/+A////////+AAAAAAAAB//wP//' +
  '//////gAAAAAAAAf/8H////////4AAAAAAAAD//h////////+AAAAAAAAAf///////////gAAAAAAAAH///////////4AAAA' +
  'AAAAA///////////+AAAAAAAAAP///////////gAAAAAAAAB///////////4AAAAAAAAAf//////////+AAAAAAAAAD/////' +
  '//////AAAAAAAAAA///////////wAAAAAAAAAP//////////8AAAAAAAAAB///////////AAAAAAAAAAf//////////wAAAA' +
  'AAAAAD//////////8AAAAAAAAAA///////////AAAAAAAAAAP//////////wAAAAAAAAAB//////////4AAAAAAAAAAP////' +
  '/////+AAAAAAAAAAB//////////gAAAAAAAAAAP/////////4AAAAAAAAAAB/////////8AAAAAAAAAAAP/////////AAAAA' +
  'AAAAAAD/////////wAAAAAAAAAAAf////////4AAAAAAAAAAAD////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAD///' +
  '/////wAAAAAAAAAAAAf///////8AAAAAAAAAAAAD///////+AAAAAAAAAAAAAf///////gAAAAAAAAAAAAD///////wAAAAA' +
  'AAAAAAAAf//////8AAAAAAAAAAAAAD//////+AAAAAAAAAAAAAAP//////gAAAAAAAAAAAAAB//////wAAAAAAAAAAAAAAH/' +
  '////4AAAAAAAAAAAAAAB/////8AAAAAAAAAAAAAAAP/////AAAAAAAAAAAAAAAD/////wAAAAAAAAAAAAAAA/////8AAAAAA' +
  'AAAAAAAAAP/////AAAAAAAAAAAAAAAD/////wAAAAAAAAAAAAAAA/////8AAAAAAAAAAAAAAAP/////AAAAAAAAAAAAAAAA/' +
  'AAD+AAAAAAAAAAAAAAAAAAAACAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── foot — 비트맵 실루엣(128×128, 잉크 29.9%)
//   원본 마스크: foot.png
var MASK_FOOT = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAD4AAAAAAAAAAAAAAAAAAAD/gAAAAAAAAAAAAAAAAAAD/8AAAAAAAAAAAAAAAAAAA//gAAAAAAAAAAAA' +
  'AAAAAAf/8BwAAAAAAAAAAAAAAAAP//g/AAAAAAAAAAAAAAAAD//4f4AAAAAAAAAAAAAAAB//+P/AAAAAAAAAAAAAAAAf//3/' +
  'wAAAAAAAAAAAAAAAH//9/+PgAAAAAAAAAAAAAB/////n8AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAH//////4AAAAAAAA' +
  'AAAAAB//////+AAAAAAAAAAAAAAf//////zwAAAAAAAAAAAAH///////+AAAAAAAAAAAAA////////wAAAAAAAAAAAAP////' +
  '///8AAAAAAAAAAAAB////////AAAAAAAAAAAAAf///////wAAAAAAAAAAAAD///////8AAAAAAAAAAAAAf///////OAAAAAA' +
  'AAAAAAH////////wAAAAAAAAAAAB////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAH////////4AAAAAAAAAAAD////' +
  '////+AAAAAAAAAAAA/////////gAAAAAAAAAAAf////////4AAAAAAAAAAAH////////8AAAAAAAAAAAD/////////AAAAAA' +
  'AAAAAA/////////gAAAAAAAAAAAP////////4AAAAAAAAAAAD////////8AAAAAAAAAAAB/////////AAAAAAAAAAAAf////' +
  '////wAAAAAAAAAAAH////////8AAAAAAAAAAAB/////////gAAAAAAAAAAAf////////4AAAAAAAAAAAH////////+AAAAAA' +
  'AAAAAB/////////gAAAAAAAAAAAP////////4AAAAAAAAAAAD////////+AAAAAAAAAAAA/////////gAAAAAAAAAAAP////' +
  '////4AAAAAAAAAAAB////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAD////////4AAAAAAAAAAAA////////+AAAAAA' +
  'AAAAAAH////////gAAAAAAAAAAAB////////wAAAAAAAAAAAAP///////8AAAAAAAAAAAAD////////AAAAAAAAAAAAAf///' +
  '////wAAAAAAAAAAAAD///////4AAAAAAAAAAAAA///////+AAAAAAAAAAAAAH///////gAAAAAAAAAAAAB///////wAAAAAA' +
  'AAAAAAAP//////8AAAAAAAAAAAAAD//////+AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAA///' +
  '///8AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAD//////gAAAAAAAAAAAAAA//////4AAAAAAAAAAAAAAH/////+AAAAAAA' +
  'AAAAAAAB//////AAAAAAAAAAAAAAAf/////wAAAAAAAAAAAAAAH/////8AAAAAAAAAAAAAAB/////+AAAAAAAAAAAAAAAf//' +
  '///gAAAAAAAAAAAAAAH/////4AAAAAAAAAAAAAAB/////+AAAAAAAAAAAAAAAf/////gAAAAAAAAAAAAAAH/////4AAAAAAA' +
  'AAAAAAAD/////+AAAAAAAAAAAAAAA//////AAAAAAAAAAAAAAAP/////wAAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAB///' +
  '///AAAAAAAAAAAAAAAf/////wAAAAAAAAAAAAAAH/////8AAAAAAAAAAAAAAD//////gAAAAAAAAAAAAAA//////4AAAAAAA' +
  'AAAAAAAP/////+AAAAAAAAAAAAAAH//////gAAAAAAAAAAAAAB//////4AAAAAAAAAAAAAAf/////+AAAAAAAAAAAAAAP///' +
  '///gAAAAAAAAAAAAAD//////4AAAAAAAAAAAAAA//////+AAAAAAAAAAAAAAP//////gAAAAAAAAAAAAAD//////4AAAAAAA' +
  'AAAAAAA//////+AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAf///' +
  '///gAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAA//////+AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAD//////wAAAAAAA' +
  'AAAAAAA//////8AAAAAAAAAAAAAAP/////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAD///' +
  '//8AAAAAAAAAAAAAAA//////AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAB/////4AAAAAAAAAAAAAAAP////8AAAAAAAA' +
  'AAAAAAAB////+AAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAB////gAAAAAAAAAAAAAAAAH///gAAAAAAAAAAAAAAAAAf/' +
  '/gAAAAAAAAAAAAAAAAAA//AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── brain — 비트맵 실루엣(128×128, 잉크 47.7%)
//   원본 마스크: brain-v5.png
var MASK_BRAIN = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAD/AAD/AAAAAAAAAAAAAAAH/8AD/2AAAAAAAAAAAAAAH//gB//wAAAAAAAAAAAAAD//8A///AAAAAAA' +
  'AAAAAAP///gf///AAAAAAAAAAAAH///8H///4AAAAAAAAAAAH/P//B//x/gAAAAAAAAAAB/j/////8f4AAAAAAAAAAA/g///' +
  '///B/AAAAAAAAAAAPw//////8P4AAAAAAAAAAP8P/j/8f/D/AAAAAAAAAAP/D/4f+D/w/8AAAAAAAAAD/x/+H/g/+P/AAAAA' +
  'AAAAA/+f/h/4f/n/4AAAAAAAAA/P////+P//8/AAAAAAAAAfx////////+H4AAAAAAAAP4cH/w///8Dz/AAAAAAAAH/PB/8P' +
  '///A//4AAAAAAAD//gf/D///wH//AAAAAAAA//4f/x/8f/g//wAAAAAAAP/8P/4//D/8H/8AAAAAAAD/+D/8P/w//A//AAAA' +
  'AAAB//B//D/8P/4P/8AAAAAAAf/x//x//D7/D//AAAAAAAPn8P4/f/74fw//4AAAAAAH4+D+H///+D8H/+AAAAAAD+fg/B//' +
  '//g/B//wAAAAAA/j4fw////4f4f/+AAAAAAfh8H8P///+H+B8fgAAAAAHweD/j3//7h/4eD8AAAAAB8Hg//gf/4f//HgOAAA' +
  'AAAfAwPP4D/8H//gADwAAAAAPwODx+Af+B//4AA8AAAAAH/jg8f+D/B//+EH/gAAAAB//8H//w/w//+Bz/8AAAAA///g//8P' +
  '8P//h//3AAAAAP//4Mf/h/D+YAf/4wAAAAD///ADH8//+AAf/+OAAAAB//zwAAf//+AADz/jgAAAAf/4eMAD///gAB4/48AA' +
  'AAP/+H/gA///4Af8P//AAAAH////8f//4/+H/D//4AAAB////+H/H+D/g/x//+AAAAfz///B/x/g/4H////gAAAP8f//Af8f' +
  '+P+A////8AAAD/D//gf+H/h/8H4///AAAA/4fj4P8B/4H/gcPn/wAAAP/D4+P/Af+A/8HD4/8AAAD/w+MP/x//+P/gx+P/AA' +
  'AA/+f/D/wf//3/8H/3/wAAAP///wf8f///P/D///8AAAH/5/8HvH///h3g/+f/gAAB/4D/BgA///wAcP8B/8AAAf4Af4AAP/' +
  '/8AAD+AH/AAAH8AD+AAA//8AAA+AA/gAAB+BgfwH/H/+P+AfAcH8AAAfg+D8B/w//j/wHwPh/AAAP4fw/x/8fn4/8D8P8P4A' +
  'AD+P+H8f//w///j/H/D+AAB/H/z////4H//////4/gAAfx//////+BH//////H4AAH8///////gB//////w+AAB///+P/D//' +
  '8P4/8f/8PgAA+Pj/h/w///D8H+H/Hh8AAPj4/4P4P//9/h/A/x4fAAD4+D8AYD////4HAPwfHwAA//gfAAB/////AAD8H/8A' +
  'AH//H+AA/////4AH/H/+AAB+P7/wA//////AD/z8fgAAfj/3+Af/////8B/v/D4AAP4f4/wf//////gfw/g/AAB+P8H+H///' +
  '///4P4P8fgAAP//D/wf/////+P/D//4AAH//4/8Dj///8MD/w/8eAAD//+P/wQ////AD/+P/HwAA///if8CP///wA/+D/x8A' +
  'AH//4D/xj///8A//A//+AAB//+A//////////gP//gAAP8/4f/////////8P8f4AAD+H+P////////w/H+H+AAA/h/j//+P/' +
  '/+f8Hx/h/AAAH+P9/H/B//+D/A//w/gAAB/Bf4B/wH//gf4B/4P4AAAPwD8Af8A//4P/AP8D+AAAD4A/A//8P////8D/AfAA' +
  'AA+DfwP8P3///j/Af4HwAAAPg/4H/D/4P/w/8H/h8AAADw/+H/w/8B/+H/g/8PAAAAcP/D/4P/Af/g/8H/DgAAADj/g/8H/4' +
  'H/8P/gHx4AAAA8/AP/A//B/+D/4B8eAAAAP/gD/wH/wf+A/8AfHAAAAB/8wcfA/8H/gfODP/wAAAAf/+HH4D/D/YPhh//4AA' +
  'AAD//gw/gf//j/wQf/8AAAAAf/8MP8D//4/8MP/+AAAAAB///n/h//+P/nn//AAAAAAf////8f//j/////gAAAAAD///7///' +
  '///n///wAAAAAAf//wP////xwP//4AAAAAAD//8B////8cD//8AAAAAAAf//AP////GA//+AAAAAAAD//+D/////g///AAAA' +
  'AAAAP//h/////4f//AAAAAAAAAf/4f////+D//AAAAAAAAAD/8H/////g//gAAAAAAAAA//D/////8P/wAAAAAAAAAH/////' +
  '//z//4AAAAAAAAAB/////n/4//+AAAAAAAAAAP////w/+P//AAAAAAAAAAA////8P////gAAAAAAAAAAP///+B////wAAAAA' +
  'AAAAAB////gf/z/4AAAAAAAAAAAH///wD/8f4AAAAAAAAAAAAf//4Af/H4AAAAAAAAAAAAB//8AD//8AAAAAAAAAAAAAD/8A' +
  'AP/gAAAAAAAAAAAAAAH+AAB/gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── eye — 비트맵 실루엣(128×128, 잉크 25.0%)
//   원본 마스크: eye.png
var MASK_EYE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAMAAAAAAAAAAAAAAAeAAAACAAAAAAAAAAAAAB///8AABgAAAAAAAAAAAAH////8AA4AQAAA' +
  'AAAAAAAP/////4AcAMAAAAAAAAAAf//////wPADAAAAAAAAAAf///////ngBgAAAAAAAAAf////////wBwBAAAAAAAAf////' +
  '////8A8AQAAAAAAAf/////////g+AIAAAAAAAP/////////+/AGAAAAAAAP///////////gDAAAAAAAH///////////wDwAA' +
  'AAAAH///////////8B4AAAAAAD////////////h8AQAAAAB/////////////+AMAAAAA/////////////+AOAAAAAf//////' +
  '///////AHAAAAAP/////////////wPgAAAAH///////////////wAAAAD///////////////wAAAAB///////////////wAA' +
  'AAA////H/////4////4AYAAAf///B/////+B///+AcAAAP///A//////wP////+AAAH///AP/////8A////+AAAD///AH///' +
  '///gD///8AAAA///gB//////4Af//+AAAAf//gA//4////AB///gAAAP//wAP/8H///wAP//8AAAH//4AD//B///8AA///AA' +
  'AB//8AA//wf///gAH//4AAA//8AAf/8H///4AA//+AAAf/+AAH//////+AAH//wAAH//AAB///////gAA//+AAD//gAAf///' +
  '///4AAH//gAB//wAAH//////+AAA//8AAf/4AAB///////gAAP//AAP/8AAAf//////4AAD//4AD/+AAAH//////+AAB//+A' +
  'B//gAAB///////gAA//+AAf/8AAAP//////wAAP//AAA//gAAD//////8AAH/+AAAH/8AAA///////AAD/+AAAA//wAAH///' +
  '///gAB/+AAAAA/+AAB//////4AA//AAAAAB/wAAP/////+AAf/AAAAAAH/AAD//////AAP/gAAAAAAf4AAf/////gAP/gAAA' +
  'AAAB/gAD/////4AH/wAAAAAAAH8AA/////8AH/wAAAAAAAA/wAH////+AD/4AAAAAAAAD/AA/////AD/4AAAAAAAAAP8AD//' +
  '//gD/4AAAAAAAAAB/wAf///gD/8AAAAAAAAAAH/gB///wH/8AAAAAAAAAAAf/AP//8P/+AAAAAAAAAAAD//D/////+AAAAAA' +
  'AAAAAAP///////+AAAAAAAAAAAAA///////+AAAAAAAAAAAAAD//////+AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAAP//' +
  '//4AAAAAAAAAAAAAAAAf///wAAAAAAAAAAAAAAAAAP/+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── nose — 비트맵 실루엣(128×128, 잉크 42.4%)
//   원본 마스크: nose.png
var MASK_NOSE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAP/wAAAAAAAAAAAAAAAAAAf//gAAAAAAAAAAAAAAAAA////AAAAAAAAAAAAAAAAAf///4AAAAAAAA' +
  'AAAAAAAAf////gAAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAA///' +
  '///AAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAD//////wAAAAAAAAAAAAAA//////8AAAAAAA' +
  'AAAAAAAP//////AAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAH///' +
  '///4AAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAD///////AAAAAAA' +
  'AAAAAAA///////wAAAAAAAAAAAAAP//////8AAAAAAAAAAAAAD///////AAAAAAAAAAAAAA///////wAAAAAAAAAAAAAP///' +
  '///8AAAAAAAAAAAAAD///////AAAAAAAAAAAAAA///////wAAAAAAAAAAAAAf//////8AAAAAAAAAAAAAH///////AAAAAAA' +
  'AAAAAAB///////4AAAAAAAAAAAAAf//////+AAAAAAAAAAAAAH///////gAAAAAAAAAAAAB///////4AAAAAAAAAAAAAf///' +
  '///+AAAAAAAAAAAAAH///////gAAAAAAAAAAAAB///////4AAAAAAAAAAAAAf//////+AAAAAAAAAAAAAP///////wAAAAAA' +
  'AAAAAAD///////8AAAAAAAAAAAAA////////AAAAAAAAAAAAAP///////wAAAAAAAAAAAAD///////8AAAAAAAAAAAAA////' +
  '////AAAAAAAAAAAAAf///////wAAAAAAAAAAAAH///////+AAAAAAAAAAAAB////////gAAAAAAAAAAAAf///////4AAAAAA' +
  'AAAAAAH///////+AAAAAAAAAAAAD////////wAAAAAAAAAAAA////////8AAAAAAAAAAAAP////////AAAAAAAAAAAAH////' +
  '////4AAAAAAAAAAAB////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAP////////8AAAAAAAAAAAD/////////AAAAAA' +
  'AAAAAB/////////4AAAAAAAAAAAf////////+AAAAAAAAAAAP/////////wAAAAAAAAAAH/////////+AAAAAAAAAAD/////' +
  '/////wAAAAAAAAAB//////////+AAAAAAAAAA///////////wAAAAAAAAAf///////////AAAAAAAAAf///////////4AAAA' +
  'AAAAf////////////gAAAAAAAP////////////8AAAAAAAP/////////////wAAAAAAH/////////////+AAAAAAD///////' +
  '///////wAAAAAB//////////////+AAAAAA///////////////wAAAAAf//////////////+AAAAAH///////////////wAA' +
  'AAD///////////////8AAAAA////////////////gAAAAf///////////////4AAAAH////////////////AAAAD////////' +
  '////////wAAAA////////////////8AAAAP////////////////gAAAH////////////////4AAAB////////////////+AA' +
  'AAf////////////////gAAAH//+D////////wf//4AAAB//+AP///////wB//+AAAAf//AA///////wAP//gAAAH//gAD///' +
  '///wAB//4AAAB//4AAf/////4AAP/+AAAAf/8AAD/////8AAD//gAAAH//AAAf/////AAA//4AAAB//wAAH/////gAAP/+AA' +
  'AAf/8AAB/////4AAD//gAAAD//gAAf////+AAA//wAAAA//4AAH/////gAAf/8AAAAP//AAB/////4AAP//AAAAB//4AAf//' +
  '//+AAH//gAAAAf//AAP/////wAD//4AAAAD//8AH/////+AD//8AAAAAf//wB//////wD///AAAAAH///3///////v///gAA' +
  'AAA///////////////wAAAAAH//////////////4AAAAAA//////////////8AAAAAAD/////////////+AAAAAAAf//////' +
  '//////+AAAAAAAB////////////+AAAAAAAAH///////////+AAAAAAAAAH//////////4AAAAAAAAAAAAf/////4AAAAAAA' +
  'AAAAAAAB/////4AAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAA////8AAAAAAAAAAAAAAAAD///8AAAAAAAAAAAAAAAAAP/' +
  '/8AAAAAAAAAAAAAAAAAAP/wAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── butterfly — 비트맵 실루엣(128×128, 잉크 46.0%)
//   원본 마스크: butterfly.png
var MASK_BUTTERFLY = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAPwAAAAAAAAAAAAAAAPwA' +
  'AP/wAAAAAAAAAAAAAA//AAH//AAAAAAAAAAAAAA//4AD//+AAAAAAAAAAAAB///AB+P/4AAAAAAAAAAAB//H4AfA//AAAAAA' +
  'AAAAAA//A+AHwD/8AAAAAAAAAAA//gPgB8Af/gABAAAAAIAAf/gD4AfgD/+AA8AAAAPAAf/4B+AD+Af/wAfgAAAH4AP/8B/A' +
  'Af4H3+AH8AAAD+AH++B/gAH/g4f4A/gAAB/AH+HB/4AA/+GB/AH8AAA/gB+Bx/+AAPHxgH4AfAAAPgB+Ac+HAADgf8A/AD4A' +
  'AHwA/AP+BwAA4D/gH4AOAABwAfgH/AcAAPAf+A/ADwAA8APwH/gPAAD4D/4H4AeAAeAH4H/wHwAA/x//g/ADgAHAB8H/+P8A' +
  'AP///+DwAYADgA+H////AAD////weAHAA4AfD////wAA8H+/+DwAwAMAPh/9/g8AAPAfH/geAOAHAHwf+PgPAABwDx/+HwBg' +
  'BgD4f/jwDgAAeAf//w+AYAYB8P/94B4AAH4H///HgDAMAeH//+B+AAB/////58AwDAPn/////gAAf//////gEAwH//////4A' +
  'AH5/////8BgYD/////5+AAD4B/////AYGA/////gDwAA8AH////4CBAf////gA8AAPAB/////AwQP////4APAAD4A/////wM' +
  'MD/////AHwAA/j//j//+BCB///n//H8AAH///4f//wQgf//h////AAB/8P+A//8EIP//Af8P/gAAf8D/wP//gkH//wP/A/4A' +
  'AH8B//3//4PB//8//4D+AAA+A//////H4//////AfAAAPgf/////x+P/////4DwAADwP/////+////////A8AAAcD///////' +
  '///////wPAAAHg//////////////8HgAAB4H/////////////+B4AAAfB//////////////g+AAAH8f/////////////4/gA' +
  'AB/////////////////4AAAP////////////////+AAAD/////////////////AAAA/j/////////////8fwAAAH4///////' +
  '///////H4AAAA+f/////////////5+AAAAH////////////////AAAAB////////////////gAAAAH///H///////j///wAA' +
  'AAA//gB//4/x//4Af/wAAAAAD4AAf/+H4f//AAHwAAAAAAAAB///j/H//+AAAAAAAAAAAf//////////gAAAAAAAAA//////' +
  '//////AAAAAAAAH/////////////gAAAAAAP//////////////AAAAAAf//////////////+AAAAAP///////////////wAA' +
  'AAH/////////////+f+AAAAD/x/////n5/////j/wAAAB/8f////5+P////4/+AAAAf+P/z//8fj//8//H/gAAAH/j/4///H' +
  '4///H/x/4AAAB/w//f//x+P//z/8P+AAAAP8P////8fj/////D/gAAAD/B///8/H4fP///g/wAAAA/4B///Pg8Hz//+Af8AA' +
  'AAf+Af//z4fh8///gH/gAAAH/wH///+H4f///4D/4AAAD//D////h+H////D//AAAA//w+f//4fh////w//wAAAf/8fn//+H' +
  '4f//5+P/+AAAH/eHw///B+D//8fj3/gAAB/Dh8P//wfg///H4cf4AAAfg4fgH/8DwP/4B+HB+AAADweH4B//A8D/+AfhwPAA' +
  'AA8Ph/A//wPA//wH4fDwAAAHH4f4//8H4P//H+H44AAAA/+D////A8D////D/8AAAAP/wP///wPA////A//AAAAD+cAf//8D' +
  'wP//+AOfwAAAA+HgB///A8D//+AHj8AAAAPh8Af//gPA///gD4fAAAADw/wD+P4BgH8/wD+DwAAAB8P/A/j+AYB/H8D/w+AA' +
  'AAfHn4H5/gGAf7+B+ePgAAAH/x/A//4AAH//A/D/4AAAB/4f4A/8AAA/8Afw/+AAAAP+H/AP/AAAP/AP+H/AAAAB/B/4H/gA' +
  'AD/4H/g/gAAAAPw8f//4AAAf//48PwAAAAD8OH//+AAAH//+PD4AAAAAfHh///AAAA///h4+AAAAAHzw8//gAAAH/88fPgAA' +
  'AAA/8OP/4AAAB//HD/wAAAAAP/Dj/8AAAAP/xw/8AAAAAD/x4/+AAAAB/8eP/AAAAAB/8+P/AAAAAP/Hz/4AAAAAf//n/gAA' +
  'AAB/5//+AAAAAH////gAAAAAH////gAAAAB////AAAAAAAP///4AAAAAf+H/gAAAAAAB/w/+AAAAAH/APwAAAAAAAPwD/gAA' +
  'AAB/gAAAAAAAAAAAAf4AAAAAPwAAAAAAAAAAAAD8AAAAABwAAAAAAAAAAAAAOAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── earth — 비트맵 실루엣(128×128, 잉크 49.4%)
//   원본 마스크: earth.png
var MASK_EARTH = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAf///4AAAAAAAAAAAAAAAAf////gAAAAAAAAAAAAAAA//////AAAAAAAAAAAAAAB//////+AAAAAAA' +
  'AAAAAAB///////4AAAAAAAAAAAAB////////gAAAAAAAAAAAB//n/////+AAAAAAAAAAAB/+B//////4AAAAAAAAAAA/+A//' +
  '/////gAAAAAAAAAA//AY//////8AAAAAAAAAA//wAA/D///DwAAAAAAAAAf/AAAGAD+fwOAAAAAAAAAP/AAAAAAAB+AwAAAA' +
  'AAAAPwgAAAAAAAP4HAAAAAAAAHAAAAAAAAAD+A4AAAAAAADgAAAAAAAAB/wDAAAAAAABwAAAAAAAAAf8A4AAAAAAA4AAAAAA' +
  'AAAD/APAAAAAAAcAAAAAAAAAA/wH4AAAAAAOAAAAAAAAAQf/D/AAAAAAHAAAAAAAAA8f///4AAAAABgAAAAAAAAf/////AAA' +
  'AAAwAAAAAAAB//////wAAAAAYAAAAAAAH/3////+AAAAAOAAAAAAAD/4/////wAAAADAAAAAAAB/8P////+AAAABgAAAAAAA' +
  '//D/////gAAAAwAAAAAAAP/5/////8AAAAMAAAAAAAB////////gAAAGAAAAAAAAH///////4AAADgAAAAAAAB////////AA' +
  'AA/AAAAAAAAf///////wAAAfgAAAAAAAH///////+AAAHwAAAAAAAD////////gAAD8AAAAAAAA////////8AAA/AAAAAAAA' +
  'f////////AAAfgAAAAAAAP////////4AAH4AAAAAAAD////////+AAB8AAAAAAAB/////////gAA5AAAAAAAH/////////8A' +
  'AMAAAAAAAB//////////AADAAAAAABg//////////wABgAAAAAA+//////////+AAYAAAAAAf///////////gAGAAAAAAD//' +
  '/////////4AB4AAAAAA////////////AA+AAAAAAP///////////wAfgAAAAAD///////////+AHMAAAAAAf///////////g' +
  'BzgAAAAAH///////////4Ac4AAAAAB///////////+AGDgAAAAAf///////////gBg8AAAAAH///////////4AYPAAAAAB//' +
  '/////////+AGDwAAAAA////////////gBg+AAAAAf///////////4AYfwBAAAP///////////+AGH8B4AAH////////////g' +
  'Bj/A+ACf////////////4AZ/wPgB/////////////+AH/8H8A//////////////gB//D/AH/////////////4Af/x/8B////' +
  '/////////+AH/+f/AP/////////////gB//n/4B/////////////4Af////gf////////////+AH////8H/////////////g' +
  'B/////j/////////////4Af//////////////////+AH///////////////////gB///////////////////4AP/////////' +
  '/////////8AB//////3////////////AAf/////w////////////gAH/////4P///////////4AB/////4D///////////+A' +
  'AP////+A////////////AAD/////wf///////////wAA/////8H//7////////8AAH////////+H///////+AAB/////////' +
  'gP///////gAAP////////8D///////4AAD/////////A///////8AAA/////////8////////AAAH/////////////////gA' +
  'AA/////////////////4AAAP///////x////////8AAAB///////4P////////AAAAf//////gD5///////gAAAD//////wA' +
  'cP//////wAAAA//////4AAD//////8AAAAH/////8AAAf/////+AAAAA/////+AAAD//////AAAAAP////8AAAAf/////wAA' +
  'AAB////+AAAAD/////4AAAAAP////gAAAAf////8AAAAAB////4AAAAD////+AAAAAAP///+AAAAA/////AAAAAAB////wAA' +
  'AAP////wAAAAAAP///+AAAAH////4AAAAAAB////gBAAB////8AAAAAAAP///8H+AA////+AAAAAAAB////f/8Af///+AAAA' +
  'AAAAP//////AH////AAAAAAAAB//////4D////gAAAAAAAAP//////B////wAAAAAAAAA///////////4AAAAAAAAAH/////' +
  '/////4AAAAAAAAAAf/////////8AAAAAAAAAAD/////////8AAAAAAAAAAAP////////8AAAAAAAAAAAA////////8AAAAAA' +
  'AAAAAAD///////8AAAAAAAAAAAAAP//////8AAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAA/////4AAAAAAAAAAAAAAAB//' +
  '//gAAAAAAAAAAAAAAAAP///wAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── piano — 비트맵 실루엣(128×128, 잉크 35.6%)
//   원본 마스크: piano.png
var MASK_PIANO = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAH////////////////8AAAH/////////////////wA' +
  'AH/////////////////+AAD//////////////////wAB//////////////////+AAf//////////////////wAP/////////' +
  '/////////8AD///////////////////AB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB/gD/8B//A/8D/8H/8Af4AfwA//AP/wP+A//A//AD+AH8AP/wD/4D/gP/gP/wA/g' +
  'B/AD/8A/+A/4D/4D/8AP4AfwA//AP/gP+A/+A//AD+AH8AP/wD/4D/gP/gP/wA/gB/AD/8A/+A/4D/4D/8AP4AfwA//AP/gP' +
  '+A/+A//AD+AH8AP/wD/4D/gP/gP/wA/gB/AD/8A/+A/4D/4D/8AP4AfwA//AP/gP+A/+A//AD+AH8AP/wD/4D/gP/gP/wA/g' +
  'B/AD/8A/+A/4D/4D/8AP4AfwA//AP/gP+A/+A//AD+AH8AP/wD/4D/gP/gP/wA/gB/AD/8A/+A/4D/4D/8AP4AfwA//AP/gP' +
  '+A/+A//AD+AH8AP/wD/4D/gP/gP/wA/gB/AD/8A/+A/4D/4D/8AP4AfwA//AP/gP+A/+A//AD+AH8AP/wD/4D/gP/gP/wA/g' +
  'B/AD/8A/+A/4D/4D/8AP4AfwA//AP/gP+A/+A//AD+AH8AP/wD/4D/gP/gP/wA/gB/AD/4A/+A/4D/4D/4AP4AfwAf+AH/AP' +
  '+Af8Af8AD+AH8AB+AA/AD/gB+AB+AA/gB/AAPgAPgA/4APgAfAAP4AfwADwAD4AP+ADwADwAD+AH8AA8AA+AD/gA8AA8AA/g' +
  'B/AAPAAPgA/4APAAPAAP4AfwADwAD4AP+ADwADwAD+AH8AA8AA+AD/gA8AA8AA/gB/AAPAAPgA/4APAAPAAP4AfwADwAD4AP' +
  '+ADwADwAD+AH8AA8AA+AD/gA8AA8AA/gB/AAPAAPgA/4APAAPAAP4AfwADwAD4AP+ADwADwAD+AH8AA8AA+AD/gA8AA8AA/g' +
  'B/AAPAAPgA/4APAAPAAP4AfwADwAD4AP+ADwADwAD+AH8AA8AA+AD/gA8AA8AA/gB/AAPAAPgA/4APAAPAAP4AfwADwAD4AP' +
  '+ADwADwAD+AH8AA8AA+AD/gA8AA8AA/gB/AAPAAPgA/4APAAPAAP4AfwADwAD4AP+ADwADwAD+AH8AA8AA+AD/gA8AA8AA/g' +
  'B/AAPAAPgA/4APAAPAAP4AfwADwAD4AP+ADwADwAD+AH+AB+AA+AD/wA+AB8AA/gB/wAfwAfwB/8AfwA/gAf4Af/////////' +
  '/////////+AD///////////////////AA///////////////////wAH//////////////////4AB//////////////////+A' +
  'AP//////////////////AAB//////////////////gAAH/////////////////wAAAf////////////////gAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── dove — 비트맵 실루엣(128×128, 잉크 35.3%)
//   원본 마스크: dove.png
var MASK_DOVE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAABgAAAAAAAAAAAAAAAAAHA' +
  'A8AAAAAAAAAAAAAAAAADwAPAAAAAAAAAAAAAAAAAA8AD4AAAAAAAAAAAAAAAAAfAB/AAAAAAAAAAAAAAAAAP4AfwAAAAAAAA' +
  'AAAAAAAAD+AH+AAAAAAAAAAAAAAAAB/gB/wAAAAAAAAAAAAAAAA/4Af+AAAAAAAAAAAAAAAAf+AH/wAAAAAAAAAAAAAAAP/g' +
  'B/+AAAAAAAAAAAAAAAH/4Af/wAAAAAAAAAAAAAAD/+AH//AAAAAAAAAAAAAAD//gB//4AAAAAAAAAAAAAB//4Af//gAAAAAA' +
  'AAAAAAB//+AH//8AAAAAAAAAAAAA///gB///wAAAAAAAAAAAA///4Af//+AAAAAAAAAAAAf//+AD///4AAAAAAAAAAAf///A' +
  'A////gAAAAAAAAAAf///wAP///8AAAAAAAAAAP///8AD////wAAAAAAAAAP////AA/////AAAAAAAAAP////wAP////4AAAA' +
  'AAAAH////4AB/////AAAA8AAAD////+AAf////8AAA/wAAD/////gAH/////gAAf+AAB/////4AA/////8AAP/wAA/////8A' +
  'AP/////gAD/8AAf/////AAD/////8AA//AAP/////wAAf/////gAf/wAH/////4AAH/////8AH/8AD/////+AAB//////AB/' +
  '/AA//////gAAP/////4Af/wAf/////wAAD/////+AH/8AP/////8AAAf/////wB//AD/////+AAAH/////+Af/wA//////gA' +
  'AB//////gH/+Af/////4AAAP/////4B//gH/////8AAAD//////Af/4D//////AAAAf/////wP/+A//////gAAAD/////8D/' +
  '/wP/////4AAAA//////h//+H/////8AAAAH/////4///h/////+AAAAB/////8f//8P/////gAAAAP////+H///h/////wAA' +
  'AAD/////j///8f////8AAAAAf////x////j////+AAAAAD////4////8f////AAAAAAf///+P////H////wAAAAAH////n//' +
  '//5////4AAAAAA////x////+P///8AAAAAAH///8/////z///+AAAAAAA////P////8////AAAAAAAH///j/////H///wAAA' +
  'AAAB///4/////x///4AAAAAAAP//+f////8f//8AAAAAAAB///n/////n//+AAAAAAAAP//5/////5///AAAAAAAAB//+f//' +
  '//+f//gAAAAAAAAP//n/////n//wAAAAAAAAA//5/////5//4AAAAAAAAAH/+f////+f/4AAAAAAAAAA//n/////P/8AAAAA' +
  'AAAAAH/9/////z/+AAAAAAAAAAAf/P////8/+AAAAAAAAAAAD/z/////P/AAAAAAAAAAAAf+/////3/gAAAAAAAAAAAB/v//' +
  '//5/gAAAAAAAAAAAAH5////+fgAAAAAAAAAAAAAff////vwAAAAAAAAAAAAABz////zwAAAAAAAAAAAAAAG////9gAAAAAAA' +
  'AAAAAAAAH///+AAAAAAAAAAAAAAAAB////gAAAAAAAAAAAAAAAAf///4AAAAAAAAAAAAAAAAH///+AAAAAAAAAAAAAAAAB//' +
  '//gAAAAAAAAAAAAAAAAf///4AAAAAAAAAAAAAAAAH///+AAAAAAAAAAAAAAAAD////wAAAAAAAAAAAAAAAA////8AAAAAAAA' +
  'AAAAAAAAf////gAAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAD///' +
  '///wAAAAAAAAAAAAAB//////+AAAAAAAAAAAAAA///////4AAAAAAAAAAAAA////////AAAAAAAAAAAAA////////8AAAAAA' +
  'AAAAAAf////////gAAAAAAAAAAAH////////8AAAAAAAAAAAB////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAD////' +
  '////wAAAAAAAAAAAA////////4AAAAAAAAAAAAH///////+AAAAAAAAAAAAAf//////+AAAAAAAAAAAAAD///////AAAAAAA' +
  'AAAAAAAP//////AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAP/////4AAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAAf//' +
  '//+AAAAAAAAAAAAAAAD/////AAAAAAAAAAAAAAAAf////gAAAAAAAAAAAAAAAB////wAAAAAAAAAAAAAAAAH///gAAAAAAAA' +
  'AAAAAAAAAD/8AAAAAAAAAAAAAAAAAAAP+AAAAAAAAAAAAAAAAAAAB+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── love — 비트맵 실루엣(128×128, 잉크 51.1%)
//   원본 마스크: love.png
var MASK_LOVE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAP/AAAAAB/8AAAAAAAAAAAP/+AAAAB//wAAAAA' +
  'AAAAAP//wAAAB///AAAAAAAAAAP///AAAA///4AAAAAAAAAD///4AAAf///AAAAAAAAAB////AAAP///4AAAAAAAAA////wA' +
  'AH////AAAAAAAAAf///+AAB////4AAAAAAAAH////wAA////+AAAAAAAAD////8AAP////wAAAAAAAA/////AAH////8AAAA' +
  'AAAAP////4AB/////AAAAAAAAH////+AAf////wAAAAAAAB/////gAH////+AAAAAAAAf////4AB/////gAAAAAAAH////+A' +
  'Af////4AAAAAAAB/////gAH////+AAAAAAAAf////4AB/////gAAAAAAAH////+AAf////wAAAAAAAA/////gAH////8AAAA' +
  'AAAAP////wAB/////AAAAAAAAD////8AAP////wAAAAAAAAf////AAD////4AAAAAAAAH////gAAf///+AAAAAAAAA////wA' +
  'AH////AAAAAAAAAH///8AAA////gAAAAAAAAA///+AAAH///wAAAAAAAAAP///AAAA///4AAAAAAAAAA///AAAAH//8AAAAA' +
  'AAADwD//gAAAAf/8A8AAAAAAH/AP/gAAAAB/8A/4AAAAAH/4AMAAAAAAAwAf/gAAAAH//gAAAAAAAAAAf/+AAAAD//+AAAAA' +
  'AAAAAf//wAAAB///4AAAAAAAAAf//+AAAA////wAAAAAAAA////wAAAf/////+AAAAf/////+AAAP//////+AAB///////wA' +
  'AH///////4AB///////+AAB////////gB////////gAA////////8A////////8AAP////////gf////////AAH////////8' +
  'P////////4AB/////////n////////+AA/////////5/////////wAP////////gB////////8AD////////gAD////////A' +
  'A////////gAAf///////wAf///////wAAB///////+AH///////4B/AP///////gB///////8D/8B///////4Af//////+B/' +
  '/wP//////+AH///////A//+D///////gB///////wf//wf//////4Af//////4P//+H//////+AH//////+H///g///////g' +
  'B///////B///8P//////4Af//////w////B//////+AH//////8P///4f//////gB///////D///+H//////4AP//////w//' +
  '//h//////8AD//////8f///4f//////AA///////D///+H//////wAP//////w////h//////8AD//////8P///4f//////A' +
  'Af//////D///8H//////gAH//////wf///D//////4AB//////+H///g//////+AAP//////g///4f//////AAD//////wH/' +
  '/8D//////wAA//////wA//+AP/////8AAH/////4AH//AB/////+AAB/////8Ag//BAH/////gAAf////+B+B/A+B/////4A' +
  'AD/////A/wAA/4P////8AAA/////gf/AAf/B/////AAAH////4P/8A//4f////gAAB////8H/////+D////4AAAP////D///' +
  '///w////8AAAD////w//////8H////AAAAf///4P//////h////gAAAD///+H//////4f///wAAAA////h//////+H///8AA' +
  'AAH///4f//////x///+AAAAB///+H//////8f///gAAAAP///h///////H///wAAAAB///4f//////x///4AAAAAP//+H///' +
  '///4f//8AAAAAD///h//////+H///AAAAAAf//4P//////h///gAAAAAD///D//////wf//wAAAAAAf//w//////8P//4AAA' +
  'AAAD//8H//////D//8AAAAAAAf//h//////g///AAAAAAAH//4P/////4f//gAAAAAAAf//D/////8H//gAAAAAAAD//wf//' +
  '///D//wAAAAAAAA//+H/////g//8AAAAAAAAD//g/////wf/+AAAAAAAAAf/8H////8P/+AAAAAAAAAD//B////+D//AAAAA' +
  'AAAAAf/4P////B//gAAAAAAAAAD//B////wf/wAAAAAAAAAAP/4f///4P/wAAAAAAAAAAB/+D///8H/4AAAAAAAAAAAH/wf/' +
  '/+D/4AAAAAAAAAAAA/+D///B/8AAAAAAAAAAAAD/wf//gf8AAAAAAAAAAAAAP+D//4P8AAAAAAAAAAAAAA/wf/8H+AAAAAAA' +
  'AAAAAAAD8D/8D8AAAAAAAAAAAAAAAPgf+B8AAAAAAAAAAAAAAAAeD/A4AAAAAAAAAAAAAAAAAAPgAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── hope — 비트맵 실루엣(128×128, 잉크 44.4%)
//   원본 마스크: hope.png
var MASK_HOPE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAQAAAAAAAAAAAAAAAIAAAAMAAAAAAAAAAAAAAADAAAAHAAAAAAAAAAAAAAAAQAAABgAAAAAAAAAAAAAAAGAA' +
  'AA4AAAAAAAAAAAAAAABwAAAOAAAAAAAAAAAAAAAAcAAAHgAAAAAAAAAAAAAAAHgAAD4AAAAAAAAAAAAAAAB4AAA+AAAAAAAA' +
  'AAAAAAAAfAAAfgAAAAAAAAAAAAAAAHwAAH4AAAAAAAAAAAAAAAB+AAB+AAAAAAAAAAAAAAAAfgAA/gAAAAAAAAAAAAAAAH8A' +
  'AP4AAAAAAAAAAAAAAAB/AAD+AAAAAAAP8AAAAAAAfwAB/gAAAAAAf/4AAAAAAH+AAf4AAAAAAf//AAAAAAB/gAH/AAAAAAP/' +
  '/8AAAAAA/4AB/wAAAAAH///gAAAAAP+AAf8AAAAAD///8AAAAAD/gAP/gAAAAB////gAAAAB/4AD/4AAAAA////8AAAAAf+A' +
  'A//AAAAAP////AAAAAP/wAP/wAAAAH////4AAAAD/4AD/+AAAAB////+AAAAB/+AAf/gAAAA/////gAAAAf/gAH/8AAAAP//' +
  '//8AAAAP/4AB//gAAAD/////AAAAH/+AAf/4AAAA/////wAAAB//gAH//AAAAf////8AAAA//4AA//4AAAH/////AAAAf/8A' +
  'AP//AAAB/////wAAAP//AAD//4AAAP////8AAAH//wAAf//AAAD/////AAAD//4AAH//4AAA/////wAAB//+AAB///gAAP//' +
  '//8AAB///AAAP//8AAB////+AAA///wAAB///gAAf////gAAf//4AAAf//+AAH////wAAf//+AAAD///4AA////8AAf///AA' +
  'AA////AAH///+AAP///wAAAH///8AA////AAP///4AAAA////wAH///gAP///8AAAAH///+AA///wAP///+AAAAA////4AH/' +
  '/4AH////AAAAAP////gAf/4AH////wAAAAB////+AB/4AH////4AAAAAP////4AAAAH////8AAAAAB/////AAAAD////+AAA' +
  'AQAP////8AAAD/////AAgAOAB/////gAAB/////gAcADwAH////+AAB/////gAPAA8AA/////wAA/////wAHwAPgAH////+A' +
  'Af////4AB8AD8AA/////wAP////8AA/AA/gAH////+AH////+AAfwAP8AA/////wD/////AAP8AD/wAD////+B/////gAP/A' +
  'A/+AAf////w/////wAH/wAP/wAD////+f////wAD/8AD/+AAf/////////4AB//AA//4AD/////////8AB//wAP//AAf////' +
  '////+AA//8AD//4AD/////////AAf//AA///AAf////////wAf//wAP//8AD////////4AP//8AD///gA////////8AH///A' +
  'A///8AH///////+AD///gAH///gA////////AD///4AB///+AH///////wB///+AAf///wB///////4A////gAH///+AP///' +
  '///8Af///wAA////wB///////AP///8AAP///+Af//////gH////AAD////wD//////4D////gAAf///+A//////8B////4A' +
  'AH////wH//////A////+AAA////8B//////gf////AAAP////gP/////4H////wAAB////8D/////8D////4AAAf////A///' +
  '///B////+AAAD////4H/////wf////AAAA/////B/////4P////wAAAH////wf////+D////4AAAB////+D/////h////8AA' +
  'AAP////g/////wf////AAAAB////8P////8P////gAAAAf////D/////D////4AAAAD////wf////h////8AAAAAf///+H//' +
  '//4f///+AAAAAD////h////+H////AAAAAAf///4f////D////gAAAAAD////H////w////wAAAAAA////x////8P///4AAA' +
  'AAAD///8f///+H///8AAAAAAAf///H////h///+AAAAAAAD///x////4f///AAAAAAAAf//+f///8P///gAAAAAAAD///n//' +
  '//D///wAAAAAAAAf//5////x///4AAAAAAAAB//+f///4f//4AAAAAAAAAP//n///+P//8AAAAAAAAAB//5////D//+AAAAA' +
  'AAAAAH/+f///x//+AAAAAAAAAAA//n///4f/+AAAAAAAAAAAD/5///+P//AAAAAAAAAAAAf8f///D//AAAAAAAAAAAAD/H//' +
  '/h//AAAAAAAAAAAAAfz///4f/AAAAAAAAAAAAAD8///8P/AAAAAAAAAAAAAAfP//+H/AAAAAAAAAAAAAADz///B/AAAAAAAA' +
  'AAAAAAA5///g+AAAAAAAAAAAAAAAGf//wcAAAAAAAAAAAAAAABn//wAAAAAAAAAAAAAAAAAD//4AAAAAAAAAAAAAAAAAA//w' +
  'AAAAAAAAAAAAAAAAAAP/gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── connection — 비트맵 실루엣(128×128, 잉크 30.9%)
//   원본 마스크: connection.png
var MASK_CONNECTION = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAH///8A' +
  'AAf//4AAAAAAAAAf////8AB////4AAAAAAAA//////wD/////wAAAAAAA///////3//////AAAAAAA//////////////8AAA' +
  'AAAf//////////////wAAAAAf//////////////+AAAAAP///////////////wAAAAH///////////////+AAAAD////////' +
  '////////wAAAB////////////////+AAAA/////////////////wAAAf////////////////+AAAH/////////////////gA' +
  'AD///4AP///////////8AAB///gAAf/////AAB///gAA///gAAD/////gAAH//8AB///gAAA/////4AAAf//4Af//wAAAH//' +
  '//8AAAD//+AH//4AAAB/////AAAAf//gB//8AAAAf////wAAAD//4Af//AAAAH////8AAAA//+AH//gAAAD/////AAAAH//g' +
  'B//4AAAA/////4AAAB//4Af/8AAAAf////+AAAAP/+AH//AAAAH/////gAAAD//gB//wAAAB/////8AAAAf/4Af/4AAAAf//' +
  '///AAAAH/+AH/+AAAAP/////wAAAB//gB//gAAAD/////8AAAAf/4Af/4AAAA//////AAAAH/+AH/+AAAAP/////wAAAB//g' +
  'B//gAAAD/////8AAAAf/4Af/4AAAA//////AAAAH/+AH/+AAAAP/////wAAAB//gB//gAAAD/////8AAAAf/4Af/4AAAA///' +
  '///AAAAH/+AH/+AAAAH/////wAAAB//gB//gAAAB/////4AAAAf/4Af/8AAAAf////+AAAAP/+AH//AAAAH/////gAAAD//g' +
  'B//4AAAB/////4AAAB//4Af/+AAAAP////8AAAAf/+AH//wAAAD/////AAAAP//gB//8AAAA/////gAAAD//4Af//gAAAP//' +
  '//4AAAB//+AH//8AAAD////+AAAA///gB///gAAA/////gAAAf//4AD//8AAAf////8AAAP//4AAf//wAAH/////gAAP//4A' +
  'AD///AAH/////8AAP//8AAAf/////////////////AAAH/////////////////gAAA/////////////////wAAAH////////' +
  '////////8AAAA////////////////+AAAAH////////////////AAAAA////////////////gAAAAH///////////////wAA' +
  'AAA///////////////wAAAAAH//////////////4AAAAAAf/////////////8AAAAAAB//////3//////8AAAAAAAH/////g' +
  'H/////8AAAAAAAAP////AAP////4AAAAAAAAAD//wAAAP///gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── family — 비트맵 실루엣(128×128, 잉크 40.3%)
//   원본 마스크: family.png
var MASK_FAMILY = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAD8AAAAA/AAAAAAAAAAAAAD/wAAAA/8AAAAAAAAAAAAB/+AAAAf/gAAAAAAAAAAAA//wAAAP/8AAAAAA' +
  'AAAAAAf/+AAAH//gAAAAAAAAAAAH//wAAB//8AAAAAAAAAAAD//8AAA///AAAAAAAAAAAA///AAAP//wAAAAAAAAAAAP//4A' +
  'AD//8AAAAAAAAAAAD//+AAA///gAAAAAAAAAAA///gAAP//wAAAAAAAAAAAP//wAAD//8AAAAAAAAAAAD//8AAA///AAAAAA' +
  'AAAAAAf//AAAH//wAAAAAAAAGAAH//gAAB//4AAYAAAAAH8AA//wAAAP/8AA/wAAAAH/gAP/8AAAD//AAf+AAAAB/8AD//AA' +
  'AA//wAP/wAAAA//gA//4AAAf/8AD/8AAAAP/4Af//gAAf//gB//AAAAH/+Af//+AAf//+Af/4AAAB//gP///4Af///wH/+AA' +
  'AAP/4H////gf////B//gAAAD/+D//////////4f/wAAAA//7/////////////8AAAAH////////////////AAAAB////////' +
  '////////gAAAAf///////////////8AAAAP////////////////gAAAP////////////////8AAAH///////H/h///////gA' +
  'AD///////gGAP//////8AAB///////wAAD///////gAA///////8AAA///z///8AAP///h///gAAP//4f///gAH///wP//4A' +
  'AD//8D///4AB///4D//+AAB///Af///AA///8A///gAAf//wD///wAP///AP//4AAH//8A///8AD///wD//+AAB///gP///A' +
  'B///8A///wAAf//4D///4Af///AP//8AAH//+A///+AH///wH///AAD///gP///gB///8B///wAA///4D///4Af///Af//8A' +
  'AP//+A///+AH///wH///AAD///gP///gB///8B///wAA///4D///4Af///Af//+AAP//+A///+AH///wH///gAH///gP///g' +
  'B///8B///4AB///4D///4Af///Af//+AAf//+A///+AH///wH///gAH///wP///gA///8D///4AB///8D///wAP///A///+A' +
  'Af///A///8AD///wP///gAH///wH///AA///8D///4AB///8B///wAH///A////AAf///Af//8AB///gP///wAH///wH//+A' +
  'Af//4D///8AB///8B///gAD//+A////AA////Af//wAA///gf/j/wAP/D/4H//8AAH//4H/wf8AD/w/+A///AAB//8B/8H/A' +
  'A/4P/gP//gAAP//Af/B/wAP+B/4D//4AAD//wH/wf8AD/gf+A//8AAAf/4B/4D/AA/4H/wH//AAAH/4Af+A/wAP8B/8Af/gA' +
  'AA/+AP/gP8AD/Af/AD/wAAAH/gD/4D/AA/wD/wA/8AAAB/4A/+A/4AP8A/8Af+AAAAP/AP/AP+AD/AP/AP/AAAAB/4H/wD/g' +
  'A/wD/4H/gAAAAP/B/8Af4AP8A/+D/4AAAAD////AH+AH/AH///8AAAAAf///gB/gB/gB///+AAAAAD///4Af4Af4Af///AAA' +
  'AAAf//+AH+AH+AH///gAAAAAD///gB/gB/gB///wAAAAAAf//4Af4Af4Af//4AAAAAAD//+AH+AH+AH//8AAAAAAAf//wB/w' +
  'B/gD//+AAAAAAAD///g/8Af8D///AAAAAAAAf///f/AP/3///gAAAAAAAD/////wD/////wAAAAAAAAP/////D/////4AAAA' +
  'AAAAB///////////4AAAAAAAAAP//////////8AAAAAAAAAB//////////+AAAAAAAAAAH/////////+AAAAAAAAAAA/////' +
  '/////AAAAAAAAAAAD/////////AAAAAAAAAAAAf////////gAAAAAAAAAAAB////////gAAAAAAAAAAAAP///////wAAAAAA' +
  'AAAAAAAf//////wAAAAAAAAAAAAAB//////wAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAAf////gAAAAAAAAAAAAAAAAf/' +
  '/+AAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── smile — 비트맵 실루엣(128×128, 잉크 46.5%)
//   원본 마스크: smile.png
var MASK_SMILE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA/' +
  '/gAAAAAAAAAAAAAAAAA////+AAAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAf//////+AAAAAAAAAAAAAf///////4AAAAAA' +
  'AAAAAA/////////wAAAAAAAAAAA//////////gAAAAAAAAAA//////////8AAAAAAAAAA///////////wAAAAAAAAAf/////' +
  '/////+AAAAAAAAAP///////////wAAAAAAAAP////////////AAAAAAAAH////////////4AAAAAAAD/////////////AAAA' +
  'AAAA/////////////wAAAAAAAf////////////+AAAAAAAP/////////////wAAAAAAH/////////////+AAAAAAB///////' +
  '///////gAAAAAA//////////////8AAAAAAP//////////////gAAAAAH//////////////4AAAAAB///////////////AAA' +
  'AAA///////////////wAAAAAP//////////////8AAAAAH///////////////gAAAAB///////////////4AAAAAf///////' +
  '///////+AAAAAP///////////////wAAAAD///wH/////+A///8AAAAA///wAf////+AD///AAAAAP//wAB/////AAP//4AA' +
  'AAD//4AAP////AAB//+AAAAB//8AAB////wAAP//gAAAAf//AAAf///4AAB//4AAAAH//gAAD///8AAAf/+AAAAD//4AAAf/' +
  '//AAAD//wAAAB//8AAAH///gAAA//+AAAB///AAAB///4AAAP//4AAA///wD8Af//+APwB///AAAf//8D/4H///gf/Af//4A' +
  'AP///B//h///4P/8P///AAD//////////////////4AB//////////////////+AA///////////////////wAP/////////' +
  '/////////8AD///////////////////AB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gA///////////////////wAP//////////////////8AD///////////////////A' +
  'Af//////////////////gAD////z////////z////4AA////4P///////wf///8AAH///8A///////4D///+AAA////AD///' +
  '///4A////AAAD///wAP/////wAP///AAAAP//8AAf////wAD///gAAAB///gAAf///AAA///wAAAAf//4AAAH/wAAAf//4AA' +
  'AAH///AAAAAAAAAH//+AAAAB///wAAAAAAAAD///gAAAAP//+AAAAAAAAA///4AAAAD///gAAAAAAAAf//8AAAAA///8AAAA' +
  'AAAAP///AAAAAH///gAAAAAAAD///wAAAAB///8AAAAAAAB///4AAAAAf///AAAAAAAA///+AAAAAD///8AAAAAAAf///AAA' +
  'AAA////AAAAAAAP///wAAAAAH///4AAAAAAH///4AAAAAB////gAAAAAD///+AAAAAAP///8AAAAAD////AAAAAAD////wAA' +
  'AAD////wAAAAAAf////AAAAB////4AAAAAAD////8AAAB////8AAAAAAA/////4AAD/////AAAAAAAH/////8Af/////gAAA' +
  'AAAA/////////////wAAAAAAAH////////////4AAAAAAAA////////////8AAAAAAAAH///////////+AAAAAAAAA//////' +
  '//////AAAAAAAAAH///////////gAAAAAAAAAf//////////gAAAAAAAAAD//////////wAAAAAAAAAAP/////////wAAAAA' +
  'AAAAAA/////////4AAAAAAAAAAAD////////4AAAAAAAAAAAAP///////4AAAAAAAAAAAAA///////wAAAAAAAAAAAAAA///' +
  '///gAAAAAAAAAAAAAAB////+AAAAAAAAAAAAAAAAA///wAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── sadness — 비트맵 실루엣(128×128, 잉크 61.3%)
//   원본 마스크: sadness.png
var MASK_SADNESS = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAABgAAAAAAAAAAAAAAAAAB/////AAAAAAAAAAAAAAAP//////gAAAAAAAAAAAAD////' +
  '////wAAAAAAAAAAAP/////////wAAAAAAAAAAf//////////wAAAAAAAAA////////////AAAAAAAAA////////////8AAAA' +
  'AAAA/////////////wAAAAAAA//////////////AAAAAAA//////////////8AAAAAAf//////////////gAAAAAP///////' +
  '///////+AAAAAP///////////////wAAAAH///////////////+AAAAD////////////////wAAAB////////////////+AA' +
  'AAf////////////////wAAAP////////////////8AAAH/////////////////gAAD/////////////////8AAA/////////' +
  '/////////AAAf/////////////////4AAH/////////////////+AAD//////////////////wAA//////////////////8A' +
  'AP//////////////////AAH//////////////////4AB//////////////////+AAf//////////////////gAP/////////' +
  '/////////8AD///////////////////AA///////////////////wAP//////////////////8AD///////////////////A' +
  'A///////////////////wAf//////////////////+AH///////////////////gB///////h///4f//////4Af//////wf/' +
  '/+D//////+AH//////4H///gf//////gB//////8B///4D//////4Af/////8Af//+AP/////+AH/////+AP///wA//////g' +
  'B///w/8AD///8AD/4f//4Af//4AAAB////gAAAH//+AH//+AAAAf///8AAAB///gB///gAAAP////AAAAf//4Af//8AAAH//' +
  '//4AAAP//+AH///gAAD/////AAAH///gB///8AAB/////8AAD///4Af///wAB//////gAB///+AH////AD///////AD////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////AB///////////////////wAP//////////////////8AD///////////////////A' +
  'A///////////////////wAP//////////////////8AD///////////////////AA/////////gf////////wAP///////+A' +
  'Af///////8AD///////8AAA///////+AAf//////8AAAB///////gAH//////8AAAAP//////4AB//////8AAAAA//////+A' +
  'Af/////+AAAAAH//////gAH//////AAAAAA//////4AA//////gAAAAAH/////8AAP/////wAAAAAA//////AAD/////4AAD' +
  'wAAH/////wAA/////8AAf/4AA/////8AAH/////AA///wAH////+AAB/////gA////AB/////gAAf////wA////8AP////4A' +
  'AH////8Af////gD////+AAA/////Af////+Af////AAAP////gP/////wH////wAAD////4H/////+B////4AAAf///+D///' +
  '///wf///+AAAH////g//////+H////gAAA////8f//////z////wAAAP////////////////8AAAB////////////////+AA' +
  'AAf////////////////gAAAD////////////////wAAAA////////////////8AAAAH///////////////+AAAAA////////' +
  '////////AAAAAP///////////////wAAAAB///////////////4AAAAAP//////////////8AAAAAB//////////////+AAA' +
  'AAAP//////////////AAAAAAB//////////////gAAAAAAH/////////////wAAAAAAA/////////////wAAAAAAAD//////' +
  '//////4AAAAAAAAP///////////wAAAAAAAAA///////////wAAAAAAAAAB//////////AAAAAAAAAAAAf///////4AAAAAA' +
  'AAAAAAAAD///8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── friendship — 비트맵 실루엣(128×128, 잉크 53.6%)
//   원본 마스크: friendship.png
var MASK_FRIENDSHIP = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAD/gAA' +
  'AAD/gAAAAAAAAAAAH//AAAAH//AAAAAAAAAAAH//8AAAH//8AAAAAAAAAAD///gAAD///gAAAAAAAAAB///+AAD///8AAAAA' +
  'AAAAA////wAB////gAAAAAAAAAf///+AAf///8AAAAAAAAAP////gAP////gAAAAAAAAH////8AH////8AAAAAAAAB/////g' +
  'D/////AAAAAAAAA/////4A/////4AAAAAAAAP////+AP////+AAAAAAAAD/////wH/////gAAAAAAAB/////8B/////8AAAA' +
  'AAAAf/////Af/////AAAAAAAAH/////4P/////wAAAAAAAB/////+D/////8AAAAAAAAf/////g//////gAAAAAAAH/////4' +
  'P/////4AAAAAAAB/////+D/////8AAAAAAAAf/////g//////AAAAAAAAH/////wH/////wAAAAAAAB/////8B/////8AAAA' +
  'AAAAP/////Af////+AAAAAAAAD/////gD/////gAAAAAAAA/////4A/////4AAAAAAAAH////8AH////8AAAAAAAAA/////A' +
  'B/////AAAAAAAAAP////gAP////gAAAAAAAAB////wAB////wAAAAAAAAAP///4AAP///4AAAAAAAAAB///8AAB///8AAAAA' +
  'AAAAAH//+AAAP//+AAAAAAAAAAA//+AAAA//+A8AAAAAAAH+D/+AAAAD/+D/8AAAAAAP/8D8AAAAAH8D//wAAAAAH//wAAAA' +
  'B/gAH//+AAAAAH///wAAD////////4AAAAD///////////////+AAAAB////////////////wAAAAf///////////////+AA' +
  'AAP////////////////gAAAD////////////////4AAAB/////////////////AAAAf////////////////wAAAH////////' +
  '////////8AAAB/////////////////AAAAf////////////////wAAAP////////////////8AAAH/////////////////gA' +
  'AB/////////////////8AAA//////////////////AAAP/////////////////4AAH/////////////////+AAD/////////' +
  '/////////wAA//////////////////8AAP//////////////////AAH//////////////////4AB//////////////////+A' +
  'Af//////////////////gAP//////////////////8AD///////////////////AA//4/////////////5//wAP/8H//////' +
  '//////8P/8AH/+B////////////+B//gB//gP////////////Af/4Af/4D////////////wH/+AH/+Af///////////4A//g' +
  'B//AH///////////+AP/4Af/wB////////////gD/+AH/8Af///////////4A//gB//AD///////////8AP/4Af/wA//////' +
  '//////AD/+AH/8AP///////////wA//gB//gD///////////8AP/4Af/4A////////////AD/+AH/+AP///////////wB//g' +
  'B//gD///////////8Af/4Af/8A////////////AP/+AH//AP///////////wD//gA//4D///////////8B//wAP/+A//////' +
  '//////Af/8AD//wP///////////wP//AAf/+D///////////8H//wAH//w////////////D//4AA///f///////////7//+A' +
  'AP//////////////////AAB//////////////////wAAf/////////////////4AAD/////////////////8AAAf////////' +
  '////////+AAAD/////////////////AAAAf////////////////wAAAD////////////////4AAAAf///////////////4AA' +
  'AAD///////////////8AAAAAf//////////////+AAAAAB///////////////AAAAAAP//////////////AAAAAAA///////' +
  '///////AAAAAAAB/////////////AAAAAAAAH///////////+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── embrace — 비트맵 실루엣(128×128, 잉크 47.4%)
//   원본 마스크: embrace.png
var MASK_EMBRACE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAA/wAAAAAAAAAAAAAAAAAAB//gAAAAAAAAAAAAAAAAAB//+AAAAAAAAAAAAAAAAAB///4AAAAAAAAAAA' +
  'AAAAAA////AAAAAAAAAAAAAAAAA////4AADwAAAAAAAAAAAAf////AAH/wAAAAAAAAAAAH////4AP//gAAAAAAAAAAD/////' +
  'AP//+AAAAAAAAAAB/////4H///wAAAAAAAAAAf/////////+AAAAAAAAAAP//////////wAAAAAAAAAD//////////+AAAAA' +
  'AAAAB///////////wAAAAAAAAAf//////////8AAAAAAAAAH///////////gAAAAAAAAB///////////4AAAAAAAAAf/////' +
  '//////AAAAAAAAAH///////////wAAAAAAAAB///////////8AAAAAAAAAf///////////gAAAAAAAAH///////////4AAAA' +
  'AAAAB///////////+AAAAAAAAAf///////////gAAAAAAAAD///////////4AAAAAAAAA///////////+AAAAAAAAAP/////' +
  '//////gAAAAAAAAD///////////wAAAAAAAAA///////////8AAAAAAAAAP///////////AAAAAAAAAD///////////gAAAA' +
  'AAAAB///////////4AAAAAAAAA///////////+AAAAAAAAAf///////////gAAAAAAAAf///////////4AAAAAAAAP//////' +
  '/////+AAAAAAAAH////////////wAAAAAAAD////////////8AAAAAAAB/////////////gAAAAAAA/////////////+AAAA' +
  'AAAP/////////////4AAAAAAH//////////////AAAAAAB//////////////4AAAAAAf//////////////AAAAAAP///////' +
  '///////4AAAAAD//////////////+AAAAAA///////////////wAAAAAf//////////////8AAAAAH///////////////gAA' +
  'AAB///////////////4AAAAAf///////////////AAAAAH/8P////////////wAAAAB/8A////////////8AAAAAf+AD////' +
  '///4f///AAAAAH/gAf//////wB///wAAAAB/wAA//////gAP//+AAAAAf8AAD/////gAB///gAAAAH/AAAD///8AAAf//4AA' +
  'AAB/wAAAA//gAAAH//+AAAAAf8AAAAAAAAAAB///gAAAAH/gAAAAAAAAAAf//wAAAAB/4AAAAAAAAAAH//8AAAAAf/AAAAAA' +
  'AAAAB///AAAAAH/4AAAAAAAAAA///wAAAAD//AAAAAAAAAAf//8AAAAA//8AAAAAAAAAf///AAAAAP//gAAAAAAAAP///wAA' +
  'AAH//+AAAAAAAAP///+AAAAB///4AAAAAAAP////gAAAAf///gAAAAAAP////4AAAAH///+AAAAAAf/////AAAAD////8AAA' +
  'AAf/////wAAAA/////4AAAB//////8AAAAP/////+AAH///4f//AAAAD////////////4B//wAAAA////////////8AP/8AA' +
  'AAP///////////8AB//AAAAD///////////+AAf/wAAAA////////////AAH/8AAAAP////B//////AAA//AAAAD////AD//' +
  '///AAAf/wAAAA////gAP////AAAH/8AAAAP///4AAP//+AAAB//AAAAD///8AAAH/wAAAA//wAAAA////AAAAAAAAAAP/8AA' +
  'AAP///wAAAAAAAAAH//AAAAD///8AAAAAAAAAB//wAAAA////gAAAAAAAAA//8AAAAP///4AAAAAAAAAf//AAAAB////AAAA' +
  'AAAAAP//wAAAAf///4AAAAAAAAP//4AAAAH////gAAAAAAAH//+AAAAB////8AAAAAAAH///gAAAAP////4AAAAAAH///4AA' +
  'AAD/////gAAAAAH///8AAAAA//////AAAAAH////AAAAAH//////AAAAP////wAAAAB///////wAD/////4AAAAAf///////' +
  '///////+AAAAAD///////////////gAAAAA///////////////wAAAAAH//////////////8AAAAAB//////////////+AAA' +
  'AAAP//////////////gAAAAAD//////////////wAAAAAAf/////////////4AAAAAAD/////////////+AAAAAAA///////' +
  '///////AAAAAAAH/////////////gAAAAAAA/////////////wAAAAAAAH////////////4AAAAAAAA////////////8AAAA' +
  'AAAAH///////////+AAAAAAAAAf///////////AAAAAAAAAB///////////AAAAAAAAAAH//////////AAAAAAAAAAAf////' +
  '////+AAAAAAAAAAAAD///////AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── boomer — 비트맵 실루엣(128×128, 잉크 19.9%)
//   원본 마스크: boomer.png
var MASK_BOOMER = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA//AAAAAAAAAAAAAAAAAAA//8AAAAAAAAA' +
  'AAAAAAAAA///wAAAAAAAAAAAAAAAAA////AAAAAAAAAAAAAAAAA////4AAAAAAAAAAAAAAAAf////gAAAAAAAAAAAAAAAP//' +
  '//8AAAAAAAAAAAAAAAP/////wAAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAD//////wAAAAAAAAAAAAAB//////+AAAAAAA' +
  'AAAAAAB///////4AAAAAAAAAAAAA////////AAAAAAAAAAAAAf///////4AAAAAAAAAAAAP////////AAAAAAAAAAAAH////' +
  '////4AAAAAAAAAAAD/////////AAAAAAAAAAAD/////////8AAAAAAAAAAB//////////gAAAAAAAAAA//////////8AAAAA' +
  'AAAAAf//////////gAAAAAAAAAP//////////8AAAAAAAAAH///////////gAAAAAAAAD///////////8AAAAAAAAD//////' +
  '//////wAAAAAAAB////////////+AAAAAAAA/////////////wAAAAAAAf////////////+AAAAAAAP/////////////wAAA' +
  'AAAH/////////////+AAAAAAD//////wD//////wAAAAAB//////gAH/////+AAAAAA//////gAAf/////wAAAAA//////gA' +
  'AB//////AAAAAf/////gAAAH/////4AAAAP/////gAAAAf/////AAAAH/////wAAAAD/////4AAAD/////4AAAAAf/////AA' +
  'AB/////4AAAAAB/////wAAAf////8AAAAAAP////+AAAP////8AAAAAAA/////wAAH////+AAAAAAAH////+AAD/////AAAA' +
  'AAAA/////wAA/////gAAAAAAAH////8AAf////gAAAAAAAAf////gAP////wAAAAAAAAD////4AD////4AAAAAAAAAf////A' +
  'A////8AAAAAAAAAD////wAf///8AAAAAAAAAAP///8AH///+AAAAAAAAAAB////gB////AAAAAAAAAAAP///4Af///gAAAAA' +
  'AAAAAB///+AH///wAAAAAAAAAAAP///gB///wAAAAAAAAAAAA///4Af//4AAAAAAAAAAAAH//+AH//8AAAAAAAAAAAAA///A' +
  'A//+AAAAAAAAAAAAAH//wAH/+AAAAAAAAAAAAAAf/4AA//AAAAAAAAAAAAAAD/8AAH/AAAAAAAAAAAAAAAP+AAAMAAAAAAAA' +
  'AAAAAAAAMAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── grandpiano — 비트맵 실루엣(128×128, 잉크 41.3%)
//   원본 마스크: grandpiano.png
var MASK_GRANDPIANO = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAP/4AAAAAAAAA' +
  'AAAAAAAAAH//8AAAAAAAAAAAAAAAAAD///4AAAAAAAAAAAAAAAAA////AAAAAAAAAAAAAAAAAf///8AAAAAAAAAAAAAAAAH/' +
  '///gAAAAAAAAAAAAAAAB////4AAAAAAAAAAAAAAAA/////AAAAAAAAAAAAAAAAP////wAAAAAAAAAAAAAAAH////+AAAAAAA' +
  'AAAAAAAAB/////gAAAAAAAAAAAAAAA/////4AAAAAAAAAAAAAAAP/////AAAAAAAAAAAAAAAH/////wAAAAAAAAAAAAAAB//' +
  '///8AAAAAAAAAAAAAAA//////AAAAAAAAAAAAAAAP/////wAAAAAAAAAAAAAAH/////8AAAAAAAAAAAAAAB//////AAAAAAA' +
  'AAAAAAAA//////wAAAAAAAAAAAAAAP/////+AAAAAAAAAAAAAAH//////gAAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAf//' +
  '////wAAAAAAAAAAAAAP///////AAAAAAAAAAAAAD///////8AAAAAAAAAAAAB////////4AAAAAAAAAAAAf////////8AAAA' +
  'AAAAAAAP/////////8AAAAAAAAAAD//////////4AAAAAAAAAB///////////4AAAAAAAAAf///////////gAAAAAAAAP///' +
  '////////+AAAAAAAAD////////////wAAAAAAAB////////////+AAAAAAAAf////////////wAAAAAAAP////////////8A' +
  'AAAAAAD/////////////gAAAAAAB/////////////4AAAAAAAf/////////////AAAAAAAP/////////////wAAAAAAD////' +
  '/////////8AAAAAAA//////////////AAAAAAAf/////////////wAAAAAAH/////////////8AAAAAAD//////////////A' +
  'AAAAAA//////////////wAAAAAAf/////////////8AAAAAAH/////////////+AAAAAAD//////////////gAAAAAB/////' +
  '/////////wAAAAAP//////////////8AAAAAf//////////////+AAAAAH///////////////AAAAAB///////////////gA' +
  'AAAAf//////////////4AAAAAP//////////////4AAAAAD//////////////8AAAAAA///////////////AAAAAAf//////' +
  '////////wAAAH//////////////////AAD//////////////////4AA//////////////////+AAP//////////////////g' +
  'AB//////////////////4AAP/////////////////+AAD//////////////////gAA//////////////////4AAP////////' +
  '/////////+AAH//////////////////gAf//////////////////4AP//////////////////+AH///////////////////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////AB//////////+AAAAAD/8AAf//////////gAAAAAf+AAD//////////wAAAAAH/AA' +
  'AAP/4AAB///wAAAAAA/wAAAB/8AAAP//4AAAAAAP8AAAAH/AAAAfj8AAAAAAD/AAAAB/gAAAD4eAAAAAAA/wAAAAf4AAAA+H' +
  'gAAAAAAP8AAAAH+AAAAPh4AAAAAAD/AAAAB/gAAAD4eAAAAAAA/wAAAAf4AAAAeHgAAAAAAP8AAAAH+AAAAHh4AAAAAAD/AA' +
  'AAB/gAAAB4eAAAAAAA/wAAAAf4AAAAeHgAAAAAAP8AAAAH+AAAAHhwAAAAAAD/AAAAA/gAAAB4cAAAAAAA/wAAAAP4AAAAeH' +
  'AAAAAAAP4AAAAD+AAAAHhwAAAAAAD+AAAAA/gAAAB4cAAAAAAAfgAAAAP4AAAAeHAAAAAAAH4AAAAD8AAAAHhwAAAAAAB+AA' +
  'AAA/AAAAB4cAAAAAAAfgAAAAPwAAAAeHAAAAAAAH4AAAAD8AAAAHhwAAAAAAB+AAAAA/AAAAB4eAAAAAAAfgAAAAP4AAAA/f' +
  'gAAAAAAP8AAAAH+AAAAf/+AAAAAAD/AAAAB/gAAAP//gAAAAAA/wAAAAPwAAAD//4AAAAAAH4AAAAB8AAAA//+AAAAAAB+AA' +
  'AAAeAAAAP//gAAAAAAPAAAAAHgAAAD//4AAAAAADwAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── heartorgan — 비트맵 실루엣(128×128, 잉크 40.0%)
//   원본 마스크: heartorgan.png
var MASK_HEARTORGAN = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAA/8AAAAAAAAAAAAAAAAAAB//wAAAAAAAAAAAAAAAAAB//8DgAAAAAAAAAAAAAAAAf//B/AAAAAAA' +
  'AAAAAAAAAH//wf8AAAAAAAAAAAAAB+A//8P/gAAAAAAAAAAAAB/wP//D/4AAAAAAAAAAAAB/8B//h/+AAAAAAAAAAAAB//Af' +
  '/4f/AAAAAAAAAAAAA//4H/+H/wAAAAAAAAAAAAf/+B//z/4AAAAAAAAAAAAH//wf///+AAAAAAAAAAAAB//8D////AAAAAAA' +
  'AAAAAAf//A////wAAAAAAAAAAAAD//4P///8AAAAAAAAAAAAAf/+H////AAAAAAAAAAAAAD//x////wAAAAAAAAAAAAA//4f' +
  '///8AAAAAAAAAAAAAP/+P////gAAAAAAAAAAAAB//H////8AAAAAAAAAAAAAf/j/////gAAAAAAAAAAAAD/x/////8AAAAAA' +
  'AAAAA/w/8f/////AAAAAAAAAAA//P+P/////4AAAAAAAAAAf/z/n/////+AAAAAAAAAAP/8fx//////gAAAAAAAAAH//H8//' +
  '////4AAAAAAAAAB//x+P/////+AAAAAAAAAAP/8fj/////8B8AAAAAAAAD//H5/////8H/wAAAAAAAA//x+f//8B8P/+AAAA' +
  'AAAAH/8fH//8AEH//gAAAAAAAB//Hx//+AAH//8AAAAAAAAP/58///AAD///AAAAAAAAD/+fP//wAD///wAAAAAAAA//jz//' +
  '8AB///8AAAAAAAAP/8Y//+AA////AAAAAAAAD//gP//gAf///gAAAAAAAA//+D//4AH///4AAAAAAAAP//w//+AD///4AAAA' +
  'AAAAD//+P//gB///wAAAAAAAAA///x//4Af//gAAAAAAAAAP//8f/+AP//wAAAAAAAAAD//+H//wD//4wAAAAAAAAA///j//' +
  '/h//8eAAAAAAAAAP//w//////+PwAAAAAAAAH//wf//////H+AAAAAAAAB//4f//////z/gAAAAAAAA//8P//////4/8AAAA' +
  'AAAAP/+H//////+P/AAAAAAAAD//H///////j/4AAAAAAAB//x///////4f+AAAAAAAAf/4////////D/gAAAAAAAH/8f///' +
  '////8P4AAAAAAAB//P////////h+AAAAAAAAf/j////////+PgAAAAAAAP/5/////////x4AAAAAAAD/8f////////+OAAAA' +
  'AAAA//P/////////xgAAAAAAAP/z/////////+IAAAAAAAD/4//////////wAAAAAAAA//f/////////+AAAAAAAAP//////' +
  '//////gAAAAAAAB////////////8AAAAAAAAf////////////AAAAAAAAH////////////4AAAAAAAB////////////+AAAA' +
  'AAAAP////////////gAAAAAAAD////////////4AAAAAAAA/////////////AAAAAAAAH////////////wAAAAAAAB//////' +
  '//////8AAAAAAAAP////////////AAAAAAAAB////////////wAAAAAAAAf///////////8AAAAAAAAD////////////AAAA' +
  'AAAAA////////////wAAAAAAAAH///////////8AAAAAAAAB////////////AAAAAAAAAf///////////gAAAAAAAAH/////' +
  '//////4AAAAAAAAA///////////+AAAAAAAAAP///////////gAAAAAAAAD///////////4AAAAAAAAAf//////////8AAAA' +
  'AAAAAH///////////AAAAAAAAAA///////////wAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAAAAAAAP////' +
  '//////gAAAAAAAAAD//////////wAAAAAAAAAAf/////////8AAAAAAAAAAD/////////+AAAAAAAAAAA//////////gAAAA' +
  'AAAAAAH/////////wAAAAAAAAAAA/////////4AAAAAAAAAAAH////////+AAAAAAAAAAAA/////////AAAAAAAAAAAAH///' +
  '/////wAAAAAAAAAAAA////////4AAAAAAAAAAAAH///////8AAAAAAAAAAAAA////////AAAAAAAAAAAAAH///////gAAAAA' +
  'AAAAAAAA///////wAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAA//////+AAAAAAAAAAAAAAD//////AAAAAAAAAAAAAAA//' +
  '////gAAAAAAAAAAAAAAD/////wAAAAAAAAAAAAAAAf////4AAAAAAAAAAAAAAAD////8AAAAAAAAAAAAAAAAf///+AAAAAAA' +
  'AAAAAAAAAD////AAAAAAAAAAAAAAAAAP///gAAAAAAAAAAAAAAAAB///wAAAAAAAAAAAAAAAAAP//4AAAAAAAAAAAAAAAAAA' +
  '//8AAAAAAAAAAAAAAAAAAB/4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── chat — 비트맵 실루엣(128×128, 잉크 55.3%)
//   원본 마스크: chat.png
var MASK_CHAT = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAD///+AAAAAAAAAAAAAAAAf/////AAAAAAAAAAAAAAA//////+AAAAAAAAAAAAAD///////8AAAAAAAAAAAAH////' +
  '////4AAAAAAAAAAAH/////////wAAAAAAAAAAH//////////AAAAAAAAAAH//////////8AAAAAAAAAH///////////gAAAA' +
  'AAAAH///////////+AAAAAAAAD////////////4AAAAAAAD/////////////AAAAAAAB/////////////4AAAAAAA///////' +
  '///////gAAAAAAf/////////////8AAAAAAP//////////////gAAAAAP//////////////8AAAAAH///////////////gAA' +
  'AAB///////////////8AAAAA////////////////gAAAAf///////////////8AAAAP////////////////AAAAH////////' +
  '////////4AAAB/////////////////AAAA/////////////////4AAAf////////////////+AAAH/////////////////wA' +
  'AD/////////////////8AAB//////////////////gAAf/////////////////4AAP//////////////////AAD/////////' +
  '/////////wAA//////////////////+AAf//////////////////gAH//////////////////4AB///////////////////A' +
  'A///////////////////wAP//////////////////8AD///////////////////gA///////////////////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AD///////////////////g' +
  'A///////////////////4AP//////////////////8AD///////////////////AAf//////////////////wAH/////////' +
  '/////////4AB//////////////////+AAP//////////////////gAD//////////////////wAA//////////////////8A' +
  'AH/////////////////+AAB//////////////////gAAP/////////////////wAAD/////////////////8AAAf////////' +
  '////////+AAAD/////////////////gAAA/////////////////wAAAH////////////////4AAAA////////////////+AA' +
  'AAH////////////////AAAAA////////////////gAAAAP///////////////wAAAAB///////////////4AAAAAP///////' +
  '///////8AAAAAB//////////////+AAAAAAH//////////////AAAAAAA//////////////gAAAAAAH/////////////gAAA' +
  'AAAA/////////////wAAAAAAAD////////////4AAAAAAAAP///////////4AAAAAAAAB///////////4AAAAAAAAAH/////' +
  '/////8AAAAAAAAAA//////////8AAAAAAAAAAP/////////8AAAAAAAAAAD/////////8AAAAAAAAAAA/////////4AAAAAA' +
  'AAAAAf////////wAAAAAAAAAAAH////////gAAAAAAAAAAAB///////8AAAAAAAAAAAAA///+P//AAAAAAAAAAAAAAP//+AA' +
  'AAAAAAAAAAAAAAAD///AAAAAAAAAAAAAAAAAA///gAAAAAAAAAAAAAAAAAP//gAAAAAAAAAAAAAAAAAH//wAAAAAAAAAAAAA' +
  'AAAAB//wAAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAAD/8AAAAAAAAAAAAAAAAAAA/+AAAA' +
  'AAAAAAAAAAAAAAAP+AAAAAAAAAAAAAAAAAAAH/AAAAAAAAAAAAAAAAAAAB/gAAAAAAAAAAAAAAAAAAAfgAAAAAAAAAAAAAAA' +
  'AAAAHwAAAAAAAAAAAAAAAAAAABwAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── book — 비트맵 실루엣(128×128, 잉크 42.1%)
//   원본 마스크: book.png
var MASK_BOOK = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAD//AAAAAAAAAAAAD//AAAH///wAAAAAAAAAA///+AAD////4AAAA' +
  'AAAAH////4AD/////4AAAAAAAf/////AA//////wAAAAAAf/////wAf//////AAAAAA//////+AH//////+AAAAB///////g' +
  'B///////4AAAB///////4Af///////wAAB///////+AH////////AAB////////gB////////4AB////////4Af////////g' +
  'B////////+AH////////+B/////////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4Af//wD//////////wD//+AH//4AD/////////AAf//gB//8AAH///////+AAD//4Af/+AAAP///' +
  '///+AAAf/+AH//gAAA//////8AAAH//gB//wAAAD/////8AAAA//4Af/8AAAAf////+AAAAP/+AH//AAAAB////+AAAAD//g' +
  'B//wAAAAH////AAAAA//4Af/8AAAAA////AAAAAP/+AH//AAAAAH///gAAAAD//gB//wAAAAA///wAAAAA//4Af/8AAAAAH/' +
  '/4AAAAAP/+AH//AAAAAB//+AAAAAD//gB//wAAAAAP//AAAAAA//4Af/8AAAAAD//wAAAAAP/+AH//AAAAAA//8AAAAAD//g' +
  'B//wAAAAAP//AAAAAA//4Af/8AAAAAD//wAAAAAP/+AH//AAAAAA//8AAAAAD//gB//wAAAAAP//AAAAAA//4Af/8AAAAAD/' +
  '/wAAAAAP/+AH//AAAAAA//8AAAAAD//gB//wAAAAAP//AAAAAA//4Af/8AAAAAD//wAAAAAP/+AH//AAAAAA//8AAAAAD//g' +
  'B//wAAAAAP//AAAAAA//4Af/8AAAAAD//wAAAAAP/+AH//AAAAAA//8AAAAAD//gB//wAAAAAP//AAAAAA//4Af/8AAAAAD/' +
  '/wAAAAAP/+AH//AAAAAA//8AAAAAD//gB//wAAAAAP//AAAAAA//4Af/8AAAAAD//wAAAAAP/+AH//AAAAAA//8AAAAAD//g' +
  'B//wAAAAAP//AAAAAA//4Af/8AAAAAD//wAAAAAP/+AH//AAAAAA//8AAAAAD//gB//wAAAAAP//AAAAAA//4Af/8AAAAAD/' +
  '/wAAAAAP/+AH//AAAAAA//8AAAAAD//gB//wAAAAAP//AAAAAA//4Af/8AAAAAD//wAAAAAP/+AH//AAAAAA//8AAAAAD//g' +
  'B//wAAAAAP//AAAAAA//4Af/8AAAAAD//wAAAAAP/+AH//AAAAAA//8AAAAAD//gB//wAAAAAP//AAAAAA//4Af/8AAAAAD/' +
  '/wAAAAAP/+AH//AAAAAA//8AAAAAD//gB//wAAAAAP//AAAAAA//4Af/8AAAAAD//wAAAAAP/+AH//gAAAAA//8AAAAAH//g' +
  'B//4AAAAAP//AAAAAB//4Af//AAAAAD//wAAAAA//+AH//4AAAAA//+AAAAAf//gB///gAAAAP//gAAAAf//4Af///4AAAD/' +
  '/4AAAH///+AH////8AAB//+AAA/////gB/////4AAf//wAB/////4Af/////wAP//+AD/////+AH//////gP///4H//////g' +
  'B///////f///////////4Af//////////////////+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4AP//////////////////8AB//////////////////+AAH/////////////////+AAAAAD//////' +
  '//////wAAAAAAAAAf/////////4AAAAAAAAAAAf////////gAAAAAAAAAAAAf//////+AAAAAAAAAAAAAA//////8AAAAAAA' +
  'AAAAAAAD/////8AAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAA////8AAAAAAAAAAAAAAAAD///8AAAAAAAAAAAAAAAAAf/' +
  '/+AAAAAAAAAAAAAAAAAB///AAAAAAAAAAAAAAAAAAP//AAAAAAAAAAAAAAAAAAB//gAAAAAAAAAAAAAAAAAAH/gAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── kiosk — 비트맵 실루엣(128×128, 잉크 36.0%)
//   원본 마스크: kiosk.png
var MASK_KIOSK = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAD/////////AAAAAAAAAAAB/////////4AAAAAAAAAAA//////////AAAAAAAAAAAP/////////wAAAAA' +
  'AAAAAH/////////+AAAAAAAAAAB//////////gAAAAAAAAAAf/////////4AAAAAAAAAAH/////////+AAAAAAAAAAB/wAAA' +
  'AAAD/gAAAAAAAAAAf4AAAAAAAf4AAAAAAAAAAH+AAAAAAAH+AAAAAAAAAAB/gAAAAAAB/gAAAAAAAAAAf4AAAAAAAf4AAAAA' +
  'AAAAAH+AAAAAAAD+AAAAAAAAAAB/gAAAAAAA/wAAAAAAAAAAf4AAAAAAAP8AAAAAAAAAAH+AAAAAAAD/AAAAAAAAAAB/gAAA' +
  'AAAA/wAAAAAAAAAAfwAAAAAAAP8AAAAAAAAAAP8AAAAAAAD/AAAAAAAAAAD/AAAAAAAA/wAAAAAAAAAA/wAAAAAAAP8AAAAA' +
  'AAAAAP8AAAAAAAD/AAAAAAAAAAD/AAAAAAAA/wAAAAAAAAAA/wAAAAAAAP8AAAAAAAAAAP8AAAAAAAD/AAAAAAAAAAD/AAAA' +
  'AAAA/wAAAAAAAAAA/wAAAAAAAP8AAAAAAAAAAP8AAAAAAAD/AAAAAAAAAAD/AAAAAAAA/wAAAAAAAAAA/wAAAAAAAP8AAAAA' +
  'AAAAAP8AAAAAAAB/AAAAAAAAAAD+AAAAAAAAfwAAAAAAAAAB/gAAAAAAAH+AAAAAAAAAAf4AAAAAAAB/gAAAAAAAAAH+AAAA' +
  'AAAAf4AAAAAAAAAB/gAAAAAAAH+AAAAAAAAAAf4AAAAAAAB/gAAAAAAAAAH+AAAAAAAAf4AAAAAAAAAB/gAAAAAAAH+AAAAA' +
  'AAAAAf4AAAAAAAB/gAAAAAAAAAH+AAAAAAAAf4AAAAAAAAAB/gAAAAAAAH+AAAAAAAAAAf4AAAAAAAB/gAAAAAAAAAH/AAAA' +
  'AAAAf4AAAAAAAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAA' +
  'AAAAAf//////////gAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAH/////' +
  '/////4AAAAAAAAAB//////8AAf+AAAAAAAAAAf/////+AAH/gAAAAAAAAAH//////gAA/4AAAAAAAAAB/4AAf/4AAf+AAAAA' +
  'AAAAAf+AAD//AAP/gAAAAAAAAAH/gAA//////4AAAAAAAAAB/4AAP/////+AAAAAAAAAAf/AAH//////gAAAAAAAAAH/////' +
  '/7/3/4AAAAAAAAAB//////4AAf+AAAAAAAAAAf/////+AAH/gAAAAAAAAAH//////gAA/4AAAAAAAAAB//////4AAf+AAAAA' +
  'AAAAAf//////AAH/gAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAH/////' +
  '/////4AAAAAAAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAA' +
  'AAAAAf//////////gAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAH/////' +
  '/////4AAAAAAAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAA' +
  'AAAAAf//////////gAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAH/////' +
  '/////4AAAAAAAAAB///////////AAAAAAAAAA///////////wAAAAAAAAAP//////////8AAAAAAAAAD///////////gAAAA' +
  'AAAAB///////////4AAAAAAAAAf//////////+AAAAAAAAAH///////////wAAAAAAAAD///////////8AAAAAAAAA//////' +
  '//////AAAAAAAAAP///////////4AAAAAAAAH///////////+AAAAAAAAB////////////gAAAAAAAA////////////8AAAA' +
  'AAAAP////////////AAAAAAAAD////////////wAAAAAAAB////////////+AAAAAAAAf////////////gAAAAAAAH//////' +
  '//////8AAAAAAAD/////////////AAAAAAAA/////////////wAAAAAAAP////////////8AAAAAAAD/////////////AAAA' +
  'AAAA/////////////wAAAAAAAP////////////8AAAAAAAD/////////////AAAAAAAA/////////////wAAAAAAAH//////' +
  '//////4AAAAAAAA////////////8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── gimbap — 비트맵 실루엣(128×128, 잉크 40.2%)
//   원본 마스크: gimbap.png
var MASK_GIMBAP = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAD/wAAAAAAAAAAAAAAAAAAf//4AAAAAAAAAAAAAAA' +
  'AA////wAAAAAAAAAAAAAAAB/////AAAAAAAAAAAAAAAB/////8AAAAAAAAAAAAAAB//////wAAAAAAAAAAAAAA//////+AAA' +
  'AAAAAAAAAAAP+H////wAAAAAAAAAAAAB//+H////AAAAAAAAAAAAD///8f///4AAAAAAAAAAAH////x////AAAAAAAAAAAH/' +
  '////H///4AAAAAAAAAAH/////8f///AAAAAAAAAAH//////z///4AAAAAAAAAH//////+f//+AAAAAAAAAH///////x///wA' +
  'AAAAAAAAAH/////+P//8AAAAAAAAAAAD/////5///gAAAAAAAP//gD/////P//8AAAAAAA////gP////5///AAAAAAB/////' +
  'A/////P//wAAAAAD/////+D////z//+AAAAAD//////4P///+f//gAAAAD///////g////z//4AAAAB///////8H///+f//A' +
  'AAAB////////w////n//wAAAA/////////H///8//8AAAAf////////4////v//AAAAf///+D////H///5//4AAAP///8AH/' +
  '//4////f/+AAAH///8AAf///H///z//gAAD///+AAD///4///+//4AAB////AAAf///H///n/+AAAf///gAAD///4///5//g' +
  'AAP///4AAA////P///f/4AAH///+AAAP///5///3/+AAD////gAAD///+P//8//gAA//wf8AAA////z///v/4AAf/wD/AAAf' +
  '8P/+f//7/+AAP/4Af4AAH+A//j//+//gAD/8AH/AAH/AD/8///n/wAB/+AA/8AD/wAf/H//9/8AAf/AAP/+f/4AH/5///f/A' +
  'AH/gAD////+AA/+f//3/wAD/4AA/////gAH/z//9/4AA/8AAP////4AB/8///f+AAf/AAD////+AAf/H//3/gAH/wAB/////' +
  'wAD/5////wAB/8AAf////8AA/+f///8AAf+AAP/////AAP/z////AAP/gAD/+AP/4AD/8////gAD/4AB/+AA/+AA//P///wA' +
  'A//AA/+AAD/wAP/z///8AAP/wAP/AAAf+AD/+f//+AAH/8AH/wAAH/wB//n///gAB//gH/4AAA/+Af/5///wAAf/8H/+AAAP' +
  '/4f/+f//4AAH/////AAAB/////n//8AAB/////wAAAf////5//+AAAf////8AAAH////+f//AAAH////+AAAB/////n//gAA' +
  'B/////gAAAP////5//wAAAf////4AAAH////+f/4AAAH/////AAAB/////n/4AAAB//4f/wAAAf/w//5/8AAAAf/4B/8AAAH' +
  '/wH/+f/AAAAH/+AH/gAAD/wA//n/gAAAB//AA/4AAA/4AH/5/4AAAAP/wAH/AAAf8AB/+f8AAAAD/4AB/4AAP/AAf/v+AAAA' +
  'A/+AAP/AAH/gAH/7/gAAAAP/gAD/4AD/4AB/8/wAAAAD/4AAf/AD/8AAf/P4AAAAAf+AAH//v//AAH/z+AAAAAH/wAB/////' +
  'wAB/9/AAAAAB/8AAP////4AA/+fgAAAAAP/AAD////+AAP/nwAAAAAD/4AA/////gAD/74AAAAAA/+AAP////4AB/88AAAAA' +
  'AH/wAD////+AA//OAAAAAAB/+AB/8AP/wAP/ngAAAAAAP/gAf8AA/8AH/5wAAAAAAD/8AH+AAD/AD/84AAAAAAAf/wD/AAA/' +
  '4D//MAAAAAAAH//D/gAAH/j//gAAAAAAAA////4AAA////4AAAAAAAAH///8AAAP///8AAAAAAAAB////AAAD///+AAAAAAA' +
  'AAP///wAAA////gAAAAAAAAB///8AAAP///wAAAAAAAAAP///gAAD///4AAAAAAAAAB///4AAB///8AAAAAAAAAAP///AAA/' +
  '//+AAAAAAAAAAB///4AAf///AAAAAAAAAAAP///gAP///gAAAAAAAAAAB///+Af///wAAAAAAAAAAAP////////4AAAAAAAA' +
  'AAAA////////4AAAAAAAAAAAAH///////8AAAAAAAAAAAAAf//////8AAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAH/////' +
  '+AAAAAAAAAAAAAAA/////+AAAAAAAAAAAAAAAB////8AAAAAAAAAAAAAAAAD///4AAAAAAAAAAAAAAAAAB//gAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── elevator — 비트맵 실루엣(128×128, 잉크 45.6%)
//   원본 마스크: elevator.png
var MASK_ELEVATOR = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAf///////4AAAAAAAAAAAAf////////gAAAAAAAAAAAP////////8AAAAAAAAAAAD/////////gAAAAA' +
  'AAAAAB/////////4AAAAAAAAAAAf/////////AAAAAAAAAAAH/////////wAAAAAAAAAAB/////////8AAAAAAAAAAAf/8//' +
  '//n//AAAAAAAAAAAH/+H///w//wAAAAAAAAAAB//A///4P/8AAAAAAAAAAAf/gH//+D//AAAAAAAAAAAH/wA///g//wAAAAA' +
  'AAAAA//4AH//4P//wAAAAAAAAD//8AA//+D///wAAAAAAAB//+AAH//g////AAAAAAAB///AAA//4P///4AAAAAAAf//gAAP' +
  '/+D////AAAAAAAP//4GGD//g////wAAAAAAH///Dhw/4YOH//+AAAAAAB////4f/8CDB///gAAAAAA////+H//AAgf//8AAA' +
  'AAAP////h//4AAH///AAAAAAD////4f//AAD///wAAAAAA////+H//4AB///8AAAAAAP////h///AA////AAAAAAD////4f/' +
  '/4Af///wAAAAAA////+H///AP///8AAAAAAP////h///4H////AAAAAAD////4f///D////wAAAAAA//////////////8AAA' +
  'AAAP//////////////AAAAAAD//////////////wAAAAAA//////////////8AAAAAAP//////////////AAAAAAD///////' +
  '///////wAAAAAA//////////////8AAAAAAP/4AAAAAAAAAB//AAAAAAD/8AAAAAAAAAAP/wAAAAAA//AAAAAAAAAAD/8AAA' +
  'AAAP/wAAAAAAAAAA//AAAAAAD/8AAAAAAAAAAP/wAAAAAA//AAAAAAAAAAD/8AAAAAAP/wAAAAAAAAAA//AAAAAAD/8AAAAA' +
  'AAAAAP/wAAAAAA//AAAAAAAEAAD/8AAAAAAP/wAB/AAAP4AA//AAAAAAD/8AA/8AAH/AAP/wAAAAAA//AAf/gAD/4AD/8AAA' +
  'AAAP/wAP/4AB//AA//AAAAAAD/8AD//AAf/wAP/wAAAAAA//AA//wAP/8AD/8AAAAAAP/wAP/8AD//gA//AAAAAAD/8AD//A' +
  'A//wAP/wAAAAAA//AA//wAH/8AD/8AAAAAAP/wAP/4AB//AA//AAAAAAD/8AB/+AAf/gAP/wAAAAAA//AAf/AAD/4AD/8AAA' +
  'AAAP/wAD/gAAf8AA//AAAAAAD/8AAPgAAB8AAP/wAAAAAA//AAD4AAAfAAD/8AAAAAAP/wAP/8AD//gA//AAAAAAD/8AP//g' +
  'B//8AP/wAAAAAA//AH//+A///gD/8AAAAAAP/wD///gf//8A//AAAAAAD/8A///8H///AP/wAAAAAA//AP///D///4D/8AAA' +
  'AAAP/wH///w///+A//AAAAAAD/8B///8P///gP/wAAAAAA//Af///D///4D/8AAAAAAP/wH///w///+A//AAAAAAD/8B///8' +
  'P///gP/wAAAAAA//Af///D///4D/8AAAAAAP/wH///w///+A//AAAAAAD/8B///8P///gP/wAAAAAA//Af///D///4D/8AAA' +
  'AAAP/wH///w///+A//AAAAAAD/8B///8P///gP/wAAAAAA//Af///D///4D/8AAAAAAP/wH///w///+A//AAAAAAD/8B///8' +
  'P///gP/wAAAAAA//Af///D///4D/8AAAAAAP/wH///w///+A//AAAAAAD/8B///8P///gP/wAAAAAA//Af///D///4D/8AAA' +
  'AAAP/wH///w///+A//AAAAAAD/8B///8P///gP/wAAAAAA//Af///D///4D/8AAAAAAP/wH///w///+A//AAAAAAD/8B///8' +
  'P///gP/wAAAAAA//Af///D///4D/8AAAAAAP/wH///w///+A//AAAAAAD/8B///8P///gP/wAAAAAA//Af///D///4D/8AAA' +
  'AAAP/wH///w///+A//AAAAAAD/8B///8P///gP/wAAAAAA//Af///D///4D/8AAAAAAP/wH///w///+A//AAAAAAD/8B///8' +
  'P///gP/wAAAAAA//Af///D///4D/8AAAAAAP/wH///w///+A//AAAAAAD/8B///8P///gP/wAAAAAA//Af///D///4D/8AAA' +
  'AAAP/wH///w///+A//AAAAAAD/8B///8P///gP/wAAAAAA//gf///D///4H/8AAAAAAP//////////////AAAAAAD///////' +
  '///////wAAAAAA//////////////8AAAAAAP//////////////AAAAAAD//////////////wAAAAAAf/////////////4AAA' +
  'AAAH/////////////+AAAAAAA//////////////AAAAAAAH/////////////gAAAAAAA/////////////wAAAAAAAH//////' +
  '//////4AAAAAAAAP///////////4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── manse — 비트맵 실루엣(128×128, 잉크 35.8%)
//   원본 마스크: manse.png
var MASK_MANSE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAH/wAAAAAAAAA' +
  'AAAAAAAAAf//gAAAAAAAAAAAAAAAAAf//+AAAAAAAAAAAAAAAAAf///4AAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAH//' +
  '//8AAAAAAAAAAAAAAAD/////gAAAAAAAAAAAAAAB/////8AAAAAAAAAAAAAAA//////AAAAAAAAAAAAAAAf/////4AAAAAAA' +
  'AAAAAAAP//////AAAAAAAAAAAAAAD//////4AAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAf//////wAAAAAAAAeAAAAP///' +
  '///8AAAAfAAAf4AAAD///////AAAAf8AAf/gAAA///////4AAAf/gAP/8AAAf//////+AAAP/8AD//AAAH///////gAAD//A' +
  'B//4AAB///////4AAB//4Af/+AAAf///////AAAf/+AH//gAAH///////wAAH//gB//8AAB///////8AAD//4Af//AAAf///' +
  '////AAA//+AH//8AAH///////wAA///gB///wAB///////8AA///4AP//8AAf///////AAP//8AD///AAH///////wAD///A' +
  'A///wAB///////8AA///wAP//8AAf/8AAP/+AAP//8AB///AAH/8AAA//gAD//+AAf//wAA//AAAP/4AA///gAD//+AAP/wA' +
  'AD/8AAf//wAA///wAD/+AAA//AAP//8AAP//8AA//wAAf/wAD///AAB///gAH/+AAP/4AB///gAAf//8AA//wAH/+AA///4A' +
  'AD///gAP/+AH//AAf//8AAA///4AB//4H//wAH///AAAH///AAP/////4AD///gAAB///4AD/////8AB///4AAAP///AAf//' +
  '//+AA///8AAAB///4AD/////AAf///AAAAf///AAP////gAP///gAAAD///8AB////gAP///wAAAA////gAP///wAH///8AA' +
  'AAH///8AA///4AD///+AAAAA////wAH//4AD////AAAAAH////AB//+AD////wAAAAB////8Af//gD////4AAAAAP////4P/' +
  '/8H////8AAAAAB//////////////+AAAAAAP//////////////AAAAAAD//////////////wAAAAAAf/////////////4AAA' +
  'AAAD/////////////8AAAAAAAf////////////+AAAAAAAD/////////////AAAAAAAAf////////////gAAAAAAAD//////' +
  '//////wAAAAAAAAf///////////4AAAAAAAAD///////////8AAAAAAAAAf//////////+AAAAAAAAAD///////////gAAAA' +
  'AAAAAf//////////gAAAAAAAAAD//////////wAAAAAAAAAAf/////////4AAAAAAAAAAH/////////+AAAAAAAAAAA/////' +
  '/////AAAAAAAAAAAH/////////wAAAAAAAAAAB/////////4AAAAAAAAAAAP////////+AAAAAAAAAAAD/////////AAAAAA' +
  'AAAAAA/////////wAAAAAAAAAAAP////////8AAAAAAAAAAAD/////////AAAAAAAAAAAAf////////gAAAAAAAAAAAH////' +
  '////4AAAAAAAAAAAB////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAH////////4AAAAAAAAAAAB////////+AAAAAA' +
  'AAAAAAf////////gAAAAAAAAAAAH////////4AAAAAAAAAAAB////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAH////' +
  '////4AAAAAAAAAAAB////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAH////////4AAAAAAAAAAAB////////+AAAAAA' +
  'AAAAAAH///////+AAAAAAAAAAAAAH//////4AAAAAAAAAAAAAAAf///4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── lonely — 비트맵 실루엣(128×128, 잉크 48.0%)
//   원본 마스크: lonely.png
var MASK_LONELY = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAA//AAAAAAAAAAAAAAAAAAB///AAAAAAAAAAAAAAAAAD///8AAAAAAAAAAAAAAAAD////wAAAAAAAA' +
  'AAAAAAAB////+AAAAAAAAAAAAAAAB/////4AAAAAAAAAAAAAAA//////AAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAP///' +
  '///AAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAD///////AAAAAAAAAAAAAB///////4AAAAAAAAAAAAAf//////+AAAAAAA' +
  'AAAAAAP///////wAAAAAAAAAAAAH///////+AAAAAAAAAAAAB////////gAAAAAAAAAAAA////////4AAAAAAAAAAAAP////' +
  '////AAAAAAAAAAAAH////////wAAAAAAAAAAAB////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAH////////4AAAAAA' +
  'AAAAAD////////+AAAAAAAAAAAA/////////wAAAAAAAAAAAP////////8AAAAAAAAAAAD/////////AAAAAAAAAAAA/////' +
  '////wAAAAAAAAAAAP////////8AAAAAAAAAAAD/////////AAAAAAAAAAAA/////////wAAAAAAAAAAAP////////8AAAAAA' +
  'AAAAAD/////////AAAAAAAAAAAA/////////wAAAAAAAAAAAH////////4AAAAAAAAAAAB////////+AAAAAAAAAAAAf////' +
  '////gAAAAAAAAAAAD////////wAAAAAAAAAAAA////////8AAAAAAAAAAAAP////////AAAAAAAAAAAAB///xx///gAAAAAA' +
  'AAAAAAf//wcH//4AAAAAAAAAAAAD//4Pg//8AAAAAAAAAAAAAf/4P/H//AAAAAAAAAAAAAH/8P/4//gAAAAAAAAAAAAA//H/' +
  '/H/wAAAAAAAAAAAAAH/7//5/4AAAAAAAAAAAAD4////+P8P4AAAAAAAAAAD+H////j/H/wAAAAAAAAAD/w////8/D//AAAAA' +
  'AAAAB/+D/////h//4AAAAAAAAB//4H////w///AAAAAAAAA///B///8wf//8AAAAAAAAf//4f//+AP///gAAAAAAAP//+P//' +
  '/gP///4AAAAAAAH///////4P////AAAAAAAB///////8P////4AAAAAAA///////+H/////AAAAAAAP///////j/////wAAA' +
  'AAAH/////////////8AAAAAAB//////////////gAAAAAA///////+f8f///4AAAAAAP///////H/D////AAAAAAD///////' +
  '7/g////wAAAAAA/////////4H///8AAAAAAP//////8f8B////AAAAAAD///////v/Af///wAAAAAA/////////wH///8AAA' +
  'AAAP//////x/8B////AAAAAAD//////8//Af///wAAAAAA///////v/4P///8AAAAAAP//////////////AAAAAAD//////z' +
  '///////wAAAAAA//////4///////8AAAAAAP/////+P///////AAAAAAD//////////////wAAAAAA//////n///////8AAA' +
  'AAAP/////x////////AAAAAAD/////8f///////wAAAAAA/////+P///////8AAAAAAP/////j////////AAAAAAD/////x/' +
  '///////wAAAAAA/////4////////8AAAAAAP////+P////////AAAAAAD/////H////////wAAAAAA/////j////////8AAA' +
  'AAAP////w/////////AAAAAAD////8f////////wAAAAAA////+P////////8AAAAAAP////D/////////AAAAAAB////x//' +
  '///////wAAAAAAf///4/////////8AAAAAAD///8f/////////AAAAAAA///+H/////////wAAAAAAH///j/////////8AAA' +
  'AAAA///x//////////AAAAAAAH//4//////////wAAAAAAA//8f/////////8AAAAAAAD/8P//////////AAAAAAAAD8H///' +
  '///////wAAAAAABgeH//////////8AAAAAAAeH////////////AAAAAAAH/////////////wAAAAAAB/////////////8AAA' +
  'AAAAf/////////////AAAAAAAH/////////////wAAAAAAB/////////////8AAAAAAAf/////////////AAAAAAAH//////' +
  '///////wAAAAAAB/////////////8AAAAAAAf/////////////AAAAAAAH/////////////wAAAAAAB/////////////8AAA' +
  'AAAAf/////////////AAAAAAAH/////////////wAAAAAAA/////////////4AAAAAAAP////////////+AAAAAAAB//////' +
  '///////AAAAAAAAH////////////AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── together — 비트맵 실루엣(128×128, 잉크 59.7%)
//   원본 마스크: together.png
var MASK_TOGETHER = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAHgAAAAAHgAAAAAAAAAAAAf/wAAAAf/gAAAAAAAAAAA///AA' +
  'AA///AAAAAAAAAAAf//8AAAf//4AAAAAAAAAAf///gAAf///gAAAAAAAAAP///8AAP///8AAAAAAAAAH////gAH////gAAAA' +
  'AAAAD////8AD////8AAAAAAAAA/////gB/////AAAAAAAAAf////4Af////4AAAAAAAAP/////AP/////AAAAAAAAD/////4' +
  'H/////wAAAAAAAB////////////+AAAAAAAAf////////////gAAAAAAAH////////////4AAAAAAAD/////////////AAAA' +
  'AAAA/////////////wAAAAAAAP////////////8AAAAAAAD/////////////AAAAAAAA/////////////wAAAAAAAP//////' +
  '//////8AAAAAAAD/////////////AAAAAAAA/////////////wAAAAAAAP////////////8AAAAAAAB////////////+AAAA' +
  'AAAAf////////////gAAAAAAAH////////////4AAAAAAAA////////////8AAAAAAAAP////////////AAAAAAAAD//////' +
  '//////wAAAAAAAAf///////////4AAAAAAAAP////////////AAAAAAAAD////////////wAAAAAAAA////////////8AAAA' +
  'AAAAf////////////gAAAAAAAP////////////8AAAAAAAf/////////////4AAAAAA///////////////gAAAAAf///////' +
  '///////+AAAAAP///////////////wAAAAH///////////////+AAAAD////////////////wAAAB////////////////+AA' +
  'AA/////////////////wAAAP////////////////8AAAH/////////////////gAAB/////////////////4AAA/////////' +
  '/////////AAAP/////////////////wAAH/////////////////+AAH//////////////////4AH///////////////////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gAP//////////////////AAB/////////' +
  '/////////gAAP/////////////////wAAD/////////////////8AAAf////////////////+AAAD/////////////////AA' +
  'AA/////////////////wAAAP////////////////8AAAD/////////////////AAAA/////////////////wAAAP////////' +
  '////////8AAAD/////////////////AAAB/////////////////4AAAf////////////////+AAAH/////////////////gA' +
  'AB/////////////////4AAAf////////////////+AAAH/////////////////gAAB/////////////////4AAAf////////' +
  '////////+AAAH/////////////////gAAB/////////////////4AAAf////////////////+AAAH/////////////////gA' +
  'AB/////////////////4AAAf////////////////+AAAH/////////////////gAAB/////////////////4AAAf////////' +
  '////////+AAAH/////////////////gAAB/////////////////4AAAf////////////////+AAAH/////////////////gA' +
  'AB/////////////////4AAAf////////////////+AAAH/////////////////gAAB/////////////////4AAAf////////' +
  '////////+AAAH/////////////////gAAB/////////////////4AAAP////////////////8AAAD/////////////////AA' +
  'AA/////////////////wAAAH////////////////4AAAB////////////////+AAAAP////////////////AAAAB////////' +
  '////////gAAAAP///////////////wAAAAB///////////////4AAAAAH//////////////4AAAAAAH/////////////wAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── comfort — 비트맵 실루엣(128×128, 잉크 58.6%)
//   원본 마스크: comfort.png
var MASK_COMFORT = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAB/wAAAAAAAAAAAAAAAAAAD//AAAAHAAAAAAAAAAAAAD//8AAA//AAAAAAAAAAAAD///wAA//+AAAAAA' +
  'AAAAAB///+AA///wAAAAAAAAAAA////wAf///AAAAAAAAAAAf///+AP///4AAAAAAAAAAP////wH////AAAAAAAAAAH////+' +
  'D////4AAAAAAAAAB/////h////+AAAAAAAAAA/////8/////wAAAAAAAAAP//////////+AAAAAAAAAH///////////gAAAA' +
  'AAAAB///////////4AAAAAAAAAf///////////AAAAAAAAAH///////////wAAAAAAAAB///////////8AAAAAAAAAf/////' +
  '//////AAAAAAAAAH///////////wAAAAAAAAB///////////8AAAAAAAAAf///////////AAAAAAAAAH///////////wAAAA' +
  'AAAAB///////////8AAAAAAAAAf///////////AAAAAAAAAD///////////gAAAAAAAAA///////////4AAAAAAAAAH/////' +
  '/////+AAAAAAAAAB///////////AAAAAAAAAAP//////////wAAAAAAAAAD//////////4AAAAAAAAAA//////////8AAAAA' +
  'AAAAB///////////AAAAAAAAAD///////////gAAAAAAAAB///////////8AAAAAAAAA////////////wAAAAAAAAf//////' +
  '//////AAAAAAAAP////////////4AAAAAAAD/////////////AAAAAAAB/////////////4AAAAAAAf/////////////AAAA' +
  'AAAH/////////////4AAAAAAB/////////////+AAAAAAAf/////////////wAAAAAAH/////////////8AAAAAAB///////' +
  '///////gAAAAAAP/////////////4AAAAAAH/////////////+AAAAAAD//////////////gAAAAAA//////////////4AAA' +
  'AAAP/////////////+AAAAAAH//////////////AAAAAAB//////////////wAAAAAAf/////////////8AAAAAAH///////' +
  '///////gAAAAAB//////////////8AAAAAAf//////////////AAAAAAH//////////////wAAAAAB//////////////8AAA' +
  'AAAf//////////////AAAAAAH//////////////wAAAAAA//////////////4AAAAAAP//////////////AAAAAAH///////' +
  '///////wAAAAAB//////////////+AAAAAA///////////////gAAAAAP//////////////4AAAAAD//////////////+AAA' +
  'AAA///////////////wAAAAAH//////////////8AAAAAB///////////////AAAAAAf//////////////wAAAAAP///////' +
  '///////8AAAAAD///////////////AAAAAA///////////////wAAAAAP//////////////8AAAAAD///////////////gAA' +
  'AAB///////////////4AAAAAf//////////////+AAAAAH///////////////gAAAAB///////////////4AAAAAf///////' +
  '///////+AAAAAH///////////////gAAAAB///////////////4AAAAAf//////////////+AAAAAH///////////////gAA' +
  'AAB///////////////4AAAAAf//////////////+AAAAAH///////////////gAAAAB///////////////4AAAAAf///////' +
  '///////+AAAAAH///////////////gAAAAB///////////////4AAAAAf//////////////+AAAAAH///////////////gAA' +
  'AAB///////////////4AAAAAf//////////////+AAAAAH///////////////gAAAAB///////////////4AAAAAf///////' +
  '///////+AAAAAH///////////////gAAAAB///////////////4AAAAAf//////////////+AAAAAH///////////////gAA' +
  'AAB///////////////4AAAAAf//////////////+AAAAAH///////////////gAAAAB///////////////4AAAAAf///////' +
  '///////+AAAAAH///////////////gAAAAB///////////////4AAAAAP//////////////8AAAAAD//////+P///////AAA' +
  'AAA///////j///////wAAAAAH//////4f//////4AAAAAB//////8H//////8AAAAAAP/////+A//////+AAAAAAB//////A' +
  'D//////AAAAAAAD/////AAP/////AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── dolhareubang — 비트맵 실루엣(128×128, 잉크 38.9%)
//   원본 마스크: dolhareubang.png
var MASK_DOLHAREUBANG = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAH/gAAAAAAAAAAAAAAAAAAP//AAAAAAAAAAAAAAAAAAP//8AAAAAAAAAAAAAAAAAH///gAAAAAAAA' +
  'AAAAAAAAD///+AAAAAAAAAAAAAAAAB////wAAAAAAAAAAAAAAAA////8AAAAAAAAAAAAAAAAf////gAAAAAAAAAAAAAAAP//' +
  '//8AAAAAAAAAAAAAAAD/////AAAAAAAAAAAAAAAB/////4AAAAAAAAAAAAAAAf////+AAAAAAAAAAAAAAAH/////gAAAAAAA' +
  'AAAAAAAD/////8AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAB//////+AAAAAAAAAAAAAB///////4AAAAAAAAAAAAA////' +
  '////AAAAAAAAAAAAAf///////4AAAAAAAAAAAAP////////AAAAAAAAAAAAH////////4AAAAAAAAAAAB////////+AAAAAA' +
  'AAAAAAf////////gAAAAAAAAAAAH////////4AAAAAAAAAAAB////////+AAAAAAAAAAAAf/+AAAD//gAAAAAAAAAAAH/wP/' +
  '/8D/4AAAAAAAAAAAA/h////+H+AAAAAAAAAAAAPj/////8fAAAAAAAAAAAABz//////xgAAAAAAAAAAAAB//////+AAAAAAA' +
  'AAAAABg///////wYAAAAAAAAAAB+P//////8fgAAAAAAAAAAfj/Af/8D/n4AAAAAAAAAAP5/gD/8AP5/AAAAAAAAAAD+fwA/' +
  '/AB+fwAAAAAAAAAA/n4AH/gAfn8AAAAAAAAAAP5+HB/8OH9/AAAAAAAAAAD+/v////9/fwAAAAAAAAAA/v///////38AAAAA' +
  'AAAAAP7///////8/gAAAAAAAAAD+////////P4AAAAAAAAAA/v///////z+AAAAAAAAAAPz///////8/AAAAAAAAAAD8//8f' +
  '+P//PwAAAAAAAAAA/P/+D/B//78AAAAAAAAAAH3//g/wf/++AAAAAAAAAAA5//4P8H//nAAAAAAAAAAAAf/+D/B//4AAAAAA' +
  'AAAAAAH//7/8//+AAAAAAAAAAAAB////////wAAAAAAAAAAAB////////+AAAAAAAAAAAA/////////wAAAAAAAAAAAf////' +
  '////+AAAAAAAAAAAP/+H///h//wAAAAAAAAAAH//gH/+Af/+AAAAAAAAAAB//4AAAAH//wAAAAAAAAAA///AAAAD//8AAAAA' +
  'AAAAAP//4AAAB///gAAAAAAAAAH///AAAA///4AAAAAAAAAB///8AAAf///AAAAAAAAAA////wAAf///wAAAAAAAAAP////g' +
  'B////8AAAAAAAAAH///////////gAAAAAAAAB///////////4AAAAAAAAAf//////////+AAAAAAAAAP///////////wAAAA' +
  'AAAAD///////////8AAAAAAAAA////////////AAAAAAAAAf+AH/////gB/4AAAAAAAAH/AAf////gAP+AAAAAAAAB/wAD//' +
  '//wAB/gAAAAAAAA//+AP///wB//8AAAAAAAAP//8B///4D///AAAAAAAAD///wP//8D///wAAAAAAAA////D///D///8AAAA' +
  'AAAAP///4f//h////gAAAAAAAH///+D//4f///4AAAAAAAB////w//8P///+AAAAAAAAf///+P//H////gAAAAAAAH////h/' +
  '/x////4AAAAAAAB////8f/4f///+AAAAAAAAf////H/+P////gAAAAAAAH////x//j////4AAAAAAAB////8f/4////+AAAA' +
  'AAAAP////H/+P////AAAAAAAAD////x//j////wAAAAAAAA////8f/4////8AAAAAAAAH////H/+P///+AAAAAAAAB////x/' +
  '/j////gAAAAAAAAP///4f/8f///wAAAAAAAAB///+P//H///4AAAAAAAAAH///D//4///4AAAAAAAAAAf//h//+H//4AAAAA' +
  'AAAABg//g///wf/wYAAAAAAAAA+A/Af//+A/AfAAAAAAAAAP8AAP///4AA/wAAAAAAAAD/8Af////gD/8AAAAAAAAA//////' +
  '//////gAAAAAAAAf///////////4AAAAAAAAH///////////+AAAAAAAAB////////////gAAAAAAAAf///////////4AAAA' +
  'AAAAH///////////+AAAAAAAAB////////////gAAAAAAAAf///////////4AAAAAAAAH///////////+AAAAAAAAB//////' +
  '//////gAAAAAAAAf///////////4AAAAAAAAD///////////8AAAAAAAAA////////////AAAAAAAAAP///////////wAAAA' +
  'AAAAB///////////4AAAAAAAAAP//////////8AAAAAAAAAA//////////+AAAAAAAAAAB/////////8AAAAAAAAAAAB////' +
  '////gAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── tree — 비트맵 실루엣(128×128, 잉크 42.5%)
//   원본 마스크: tree.png
var MASK_TREE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAAf/4AAAAAAAAA' +
  'AAAAAAAAAf//gAAAAAAAAAAAAAAAAAP//8AAAAAAAAAAAAAAAAAH///gAAAAAAAAAAAAAAAAB///8AAAAAAAAAAAAAAAAA//' +
  '//AAAAAAAAAAAAAAAAD/////AAAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAA//////+AAAAAAA' +
  'AAAAAAAf//////gAAAAAAAAAAAAAH//////8AAAAAAAAAAADgD///////AHgAAAAAAAAP/A///////wP/AAAAAAAAP/8P///' +
  '///8P/8AAAAAAAH//j///////H//wAAAAAAD//8///////z//8AAAAAAB///H//////4///gAAAAAA///5//////+f//8AAA' +
  'AAAP//+P//////H///AAAAAAD///7//////3///wAAAAAA//////////////8AAAAAAP//////////////AAAAAAD///////' +
  '///////wAAAAAA//////////////8AAAAAA///////////////wAAAAA////////////////AAAAAf///////////////4AA' +
  'AAP///+B///////////AAAAD///+AH//////////wAAAA////AA//////////+AAAAf///gAH/////v////gAAAH///wAA//' +
  '//8AH///4AAAB///8AAP///+AA///+AAAAf//+AAB///+AAH///gAAAH///gAAP///AAA///4AAAA///wAAB///wAAP//+AA' +
  'AAP//8AAAP//4AAB///AAAAD///AAAB//8AAAf//wAAAAf//wAAAP//AAAH//4AAAAD//8AAAB//gAAB//8AAAAAf//gAAAf' +
  '/4AAAf/+AAAAAAf/4AAAD/+AAAP/4AAAAAAAf+AAAA//gAAD/gAAAAAAAD/wAAAP/4AAA/wAAAAAAPwf+AAAD/+AAAf4PwAA' +
  'AAP/H/wAAA//gAAH+P/AAAAP/8/+AAAf/4AAD/v/8AAAH////wAAH//AAB////gAAD/////AAD//wAB////8AAA/////8AB/' +
  '/+AA/////AAAf/////4B///4A/////4AAH/////////////////+AAB//////////////////gAAf/////////////////4A' +
  'AP//////////////////AAH//////////////////4AD/////4//////4f/////AA/////+D/////4P/////wAf/////4P//' +
  '//4H/////+AH//////A////4D//////gB//////wD///8B//////4Af/////+Af//+Af/////+AH//////wH///AP//////g' +
  'B///////A///wP//////4AP//////+P//4P//////8AD///////7//+P///////AAf//////////////////gAD/////////' +
  '/////////wAAP/////////////////wAAAf///8P/////8D////gAAAD///+A/////+Af///wAAAAf///AD////+AH///4AA' +
  'AAH///wAf////AA///+AAAAA///4AD//h/gAH///AAAAAH//4AAf/wf4AA///gAAAAAf/8AAH/4D8AAD//gAAAAAA/wAAA/8' +
  'A/AAAH/AAAAAAAAAAAAP/APgAAAAAAAAAAAAAAAAB/wD4AAAAAAAAAAAAAAAAAf8A+AAAAAAAAAAAAAAAAAH/APAAAAAAAAA' +
  'AAAAAAAAB/wHwAAAAAAAAAAAAAAAAAf8B8AAAAAAAAAAAAAAAAAH/AfAAAAAAAAAAAAAAAAAB/4PwAAAAAAAAAAAAAAAAAf/' +
  'H+AAAAAAAAAAAAAAAAAH///gAAAAAAAAAAAAAAAAB///4AAAAAAAAAAAAAAAAA///+AAAAAAAAAAAAAAAAAP///gAAAAAAAA' +
  'AAAAAAAAH///8AAAAAAAAAAAAAAAAB////AAAAAAAAAAAAAAAAA////4AAAAAAAAAAAAAAAAf////AAAAAAAAAAAAAAAAP//' +
  '//4AAAAAAAAAAAAAAAP/////AAAAAAAAAAAAAAAP/////8AAAAAAAAAAAAAAP//////4AAAAAAAAAAAAB////////8AAAAAA' +
  'AAAAAD/////////8AAAAAAAAAAB//7////3//wAAAAAAAAAA//4////4f/+AAAAAAAAAA//4P////B//wAAAAAAAAB//4P//' +
  '//8P//AAAAAAAAH//AP/////wP//AAAAAAAH/+AP//////AP/+AAAAAAD/+Af//////+A//wAAAAAA/+Af///////4D/8AAA' +
  'AAAH8Af///Af///gH+AAAAAAAAAH//8AAf//4AAAAAAAAAAAB//wAAAf/+AAAAAAAAAAAAP+AAAAAD/AAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── cup — 비트맵 실루엣(128×128, 잉크 42.0%)
//   원본 마스크: cup.png
var MASK_CUP = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAADgAAAAAAAAAAAAAAAAAAAA+AAAAAAAAAAAAAAAAAAAAPwAAAAAAAAAAAAAAAAAAAB+AAAAAAAAAA' +
  'AAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAH+AAAAAAAAAAAAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAAD' +
  '/gAAAAAAAAAAAAAAAAAAA/4AAAAAAAAAAAAAAAAAAAP+AAAAAAAAAAAAAAAAAAAD/wAAAAAAAAAAAAAAAAAAB/8AAAAAAAAA' +
  'AAAAAAPAAAf/AAAAAAAAAAAAAAAB8AAH/wAAAAAAAAAAAAAAAfgAD/+AAAAAAAAAAAAAAAD8AB//h4AAAAAAAAAAAAAA/gA/' +
  '/gPgAAAAAAAAAAAAAP4Af/4D8AAAAAAAAAAAAAD/Af/+AfgAAAAAAAAAAAAA/wP//AH4AAAAAAAAAAAAAP/H//wB/AAAAAAA' +
  'AAAAAAD////4AfwAAAAAAAAAAAAA////8AH+AAAAAAAAAAAAAf///+AB/gAAAAAAAAAAAAH////AAf4AAAAAAAAAAAAD////' +
  'wAP+AAAAAAAAAAAAB////4AH/gAAAAAAAAAAAAf///8AD/4AAAAAAAAAAAAP////AB/+AAAAAAAAAAAAH////gB//AAAAAAA' +
  'AAAAAB////4Af/gAAAAAAAAAAAA////+AP/wAAAAAAAAAAAAP////gH/4AAAAAAAAAAAAD////8B/8AAAAAAAAAAAAA/////' +
  'A/+AAAAAAAAAAAAAP////8f/gAAAAAAAAAAAAB///////wAAAAAAAAAAAAAf//////8AAAAAAAAAAAAAD///////AAAAAAAA' +
  'AAAAAAf//////4AAAAAAAAAAAAAH//////+AAAAAAAAAAAAAB///////gAAAAAAAAAAAAAf//////8AAAAAAAAAAAAAP////' +
  '///AAAAAAAAAAAAAH///////+AAAAAAAAAAAAP////////+AAAAAAAAAAAf/////////8AAAAAAAAAA///////////wAAAAA' +
  'AAAA////AAAA////AAAAAAAAAf/+AAAAAAB//4AAAAAAAAP/+AAAAAAAP//AAAAAAAAH//AAAAAAAB//4AAAAAAAB//wAAAA' +
  'AAAf/+AAAAAAAA//8AAAAAAAH//wAAAAAAAP//AAAAAAAB///PwAAAAAD//wAAAAAAAf////wAAAAA//+AAAAAAAP/////AA' +
  'AAAP//+AAAAAA//////8AAAAD////8AAB////////gAAAA////////////////8AAAAP////////////////gAAAD///////' +
  '/////////4AAAA//////////////gf/AAAAP/////////////gB/wAAAD/////////////wAP8AAAA/////////////4AD/g' +
  'AAAP////////////8AAf4AAAD/////////////AAH+AAAA/////////////wAB/gAAAH////////////8AAf4AAAB///////' +
  '//////AAH+AAAAf////////////gAB/gAAAH////////////4AAf4AAAB////////////+AAH+AAAAf////////////gAB/A' +
  'AAAD////////////wAA/wAAAA////////////8AAP8AAAAP////////////AAH+AAAAD////////////wAD/gAAAAf//////' +
  '/////8AB/4AAAAH////////////gA/8AAAAB////////////8A/+AAAAAP////////////4//AAAAAD///////////////wA' +
  'AAAAf//////////////4AAAAAH//////////////4AAAAAB//////////////8AAAAAA//////////////8AAAAAAP//////' +
  '///////+AAAAAAP//////////////gAAAAAf//////////////4AAAAA///////////////8AAAAB////////////////gAA' +
  'AB////////////////4AAAB//wH/////////gP//AAAA//gA/////////wAf/8AAA//gAH////////wAB//gAAP/wAA/////' +
  '///8AAP/8AAH/4AAH///////+AAB//AAB/+AAA////////AAAf/wAAf/gAAP///////wAAH/8AAH/4AAD///////8AAB//AA' +
  'B//AAA///////+AAA//wAAP/4AAH///////gAAP/4AAB//gAB///////4AAP/8AAAf/+AA////////AAf/+AAAB//8AP////' +
  '///wA//+AAAAP//8H////////D///AAAAA////////////////AAAAAH///////////////gAAAAAf//////////////AAAA' +
  'AAA/////////////+AAAAAAAB////////////8AAAAAAAAH///////////4AAAAAAAAAH//////////AAAAAAAAAAAB/////' +
  '///gAAAAAAAAAAAAAD////+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── car — 비트맵 실루엣(128×128, 잉크 44.5%)
//   원본 마스크: car.png
var MASK_CAR = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAP//////AAAAAAAAAAAAAB////////gAAAAAAAAAAAB////////+AAAAAAAAAAAB/////////4AAAAAAAAAAB/////' +
  '/////AAAAAAAAAAA//////////8AAAAAAAAAAf//////////AAAAAAAAAAH//////////4AAAAAAAAAD//AAAAAAD//AAAAA' +
  'AAAAB/8AAAAAAAD/4AAAAAAAAA/8AAAAAAAAP+AAAAAAAAAP+AAAAAAAAB/wAAAAAAAAH/AAAAAAAAAP8AAAAAAAAB/gAAAA' +
  'AAAAD/gAAAAAAAA/4AAAAAAAAAf8AAAAAAAAP8AAAAAAAAAD/AAAAAAAAH/AAAAAAAAAA/4AAAAAAAD/gAAAAAAAAAH+AAAA' +
  'AAAA/4AAAAAAAAAB/wAAAAAAAf8AAAAAAAAAAP+AAAAAAAP/AAAAAAAAAAD/wAAAAA///wAAAAAAAAAA///wAAA///8AAAAA' +
  'AAAAAP///AAAf///AAAAAAAAAAD///4AAH///4AAAAAAAAAB////AAD//////////////////wAA//////////////////8A' +
  'AP//////////////////AAD//////////////////wAAf/////////////////8AAH/////////////////+AAA/////////' +
  '/////////gAAP/////////////////wAAD/////////////////8AAA//////////////////AAAP/////////////////wA' +
  'AD/////////////////+AAB//////////////////wAA//////////////////8AAf//////////////////gAH/////////' +
  '/////////4AD///////////////////AA///////////////////wAP//////////////////8AD///////////////////g' +
  'B/4A/////////////4B/4Af+AB////////////wAP+AH/AAH///////////wAD/gB/wAA///////////wAA/4Af8AAH/////' +
  '/////8AAP+AH/AAB//////////+AAD/gB/wAAf//////////gAA/4Af+AAH//////////4AAP+AH/gAB//////////+AAH/g' +
  'B/+AA///////////wAD/4Af/+Af//////////+Af/+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB/////gAAAAAAAAP////4Af////gAAAAAAAAA////+AH////wAAAAAAAAAH////g' +
  'B////4AAAAAAAAAB////4Af///+AAAAAAAAAAP///+AH////gAAAAAAAAAD////gB////4AAAAAAAAAB////4Af////AAAAA' +
  'AAAAAf///+AH////4AAAAAAAAAP////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gB///////////////////4AP///4AAAAA' +
  'AAAAAD///+AD///4AAAAAAAAAAAf///AA///+AAAAAAAAAAAD///wAH///AAAAAAAAAAAA///4AA///wAAAAAAAAAAAH//8A' +
  'AH//4AAAAAAAAAAAA//+AAAf/4AAAAAAAAAAAAH/+AAAA/wAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── dog — 비트맵 실루엣(128×128, 잉크 47.0%)
//   원본 마스크: dog.png
var MASK_DOG = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAA//8AAAAAAAAAAAAAAAAAH///+AAAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAP/////wAAAAAAA' +
  'AAAAAAAP//////AAAAAAAAAAAAAAP//////8AAAAAAAAAAAAD/////////AAAAAAAAAAAH/////////+AAAAAAAAAAH/////' +
  '/////4AAAAAAAAAH///////////gAAAAAAAAD///////////+AAAAAAAAD////////////wAAAAAAAB////////////+AAAA' +
  'AAAB/////////////4AAAAAAA//////////////AAAAAAAf/////////////4AAAAAAP//////////////AAAAAAH///////' +
  '///////4AAAAAD///////////////AAAAAB///////////////4AAAAAf//////////////+AAAAAP///////////////wAA' +
  'AAH///////////////+AAAAB////////////////wAAAA////////////////8AAAAP////8/////x/////gAAAH////8D//' +
  '//wP////4AAAB////+Af///8B////+AAAA/////gH///+AP////wAAAP////wA////gD////8AAAD////8AP///wA/////AA' +
  'AA/////AD///8AP////4AAAP////wA////AD////+AAAD////8AP///4A/////gAAA/////AH///+AP////4AAAP////4B//' +
  '//gH////+AAAD/////A////8B/////gAAA/////4f////g/////wAAAP////////////////8AAAD///////+B////////AA' +
  'AAf//////4AB///////gAAAH//////wAAD//////4AAAB//////wAAAP/////+AAAAP/////4AGAB//////AAAAD/////8Af' +
  '+AP/////wAAAAf////+AP/4B/////4AAAAD/////AH/+AP////8AAAAAf////gB//gB////+AAAAAD/H//wAf/4AP//j/AAA' +
  'AAAHh//8AH/+AD//4PAAAAAAAAP//AA//AAf/+AAAAAAAAAD//gAH/gAH//AAAAAAAAAA//4AA/wAB//wAAAAAAAAAH/+AAH' +
  '4AAf/4AAAAAAAAAB//gAA8AAH/+AAAAAAAAAAP/4AAPAAB//AAAAAAAAAAB/+AAD4AAf/gAAAAAAAAAAf/gAD/AAH/4AAAAA' +
  'AAAAAD/8AH//AD/8AAAAAAAAAAA//AD//wA//AAAAAAAAAAAf/4AfD8Af/4AAAAAAAAAAP//AAAAAH//AAAAAAAAAAD//4AA' +
  'AAD//4AAAAAAAAAB///AAAAD//+AAAAAAAAAAf//4AAAB///wAAAAAAAAAP///gAAB///8AAAAAAAAAD////AAD////gAAAA' +
  'AAAAB/////9/////4AAAAAAAAAf//////////+AAAAAAAAAP///////////wAAAAAAAAD//8P////D//8AAAAAAAAA//+Af/' +
  '/+Af//AAAAAAAAAP//AAf/8AD//wAAAAAAAAH//wAAAAAA//+AAAAAAAAB//4AAAAAAH//gAAAAAAAAf/+AAAAAAB//4AAAA' +
  'AAAAH//gAAAAAAf/+AAAAAAAAD//4AAAAAAH//wAAAAAAAB//+AAAAAAB///AAAAAAAA///gAAAAAAf//4AAAAAAAf//4AAA' +
  'AAAH///AAAAAAAP//+AAAAAAB///wAAAAAAH///wAAAAAA///+AAAAAAB///8AAAAAAP///gAAAAAA////gAAAAAH///8AAA' +
  'AAAP///8AAAAAD////AAAAAAD////gAAAAB////wAAAAAB////8AAAAA////+AAAAAAf////gAAAAf////gAAAAAH////+AA' +
  'AAP////4AAAAAB/////wAAAH////+AAAAAAf////+AAAD/////gAAAAAH/////gAAB/////4AAAAAA/////8AAA/////8AAA' +
  'AAAP/////gAAP/////AAAAAAD/////4AAH/////wAAAAAA/////+AAB/////8AAAAAAH/////wAA//////AAAAAAB/////8A' +
  'AP/////gAAAAAAf/////AAD/////4AAAAAAH/////wAA/////+AAAAAAB/////8AAP/////wAAAAAB//////gAH/////+AAA' +
  'AAAf/////4AB//////gAAAAAP/////+AAf/////8AAAAAD///////////////AAAAAA///////////////wAAAAAP///////' +
  '///////+AAAAAD///////////////gAAAAA///////////////wAAAAAP//////////////8AAAAAD///////////////AAA' +
  'AAAf//////////////gAAAAAD/////+AAP/////wAAAAAAf/////AAD/////4AAAAAAAP4///gAAf//x/AAAAAAAAAAH//wA' +
  'AD//4AAAAAAAAAAAAP/wAAAH/4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── cat — 비트맵 실루엣(128×128, 잉크 43.9%)
//   원본 마스크: cat.png
var MASK_CAT = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAABgAAAAAAAAAPAAAAAAAAAB/AAAAAAAAAf8AAAAAAAAA/8AAAAAAAAf/AAAAAAAAAf/wAAAAAAAf/4AAAA' +
  'AAAAH//AAAAAAAP/+AAAAAAAAB//4AAAAAAH//gAAAAAAAAf//AAAAAAD//8AAAAAAAAP//4AAAAAD///AAAAAAAAD///gAA' +
  'AAB///wAAAAAAAA///4AAAAA///8AAAAAAAAP///gAAAAf///AAAAAAAAD///4H//4P///wAAAAAAAA////////////8AAAA' +
  'AAAAP////////////AAAAAAAAD////////////wAAAAAAAAf///////////8AAAAAAAAH///////////+AAAAAAAAB//////' +
  '//////gAAAAAAAAf///////////4AAAAAAAAH///////////+AAAAAAAAB////////////gAAAAAAAAP///////////wAAAA' +
  'AAAAD///////////8AAAAAAAAA////////////AAAAAAAAAH///////////wAAAAAAAAB///////////4AAAAAAAAAf/////' +
  '/////+AAAAAAAAAH///////////gAAAAAAAAB///////////8AAAAAAAAA////////////AAAAAAAAAf///////////4AAAA' +
  'AAAAH///////////+AAAAAAAAD//4P////8D//wAAAAAAAA//4A////8Af/8AAAAAAAAP/8AH////AD//gAAAAAAAH/+AA//' +
  '//gAf/4AAAAAAAB//gAP///wAD/+AAAAAAAA//wAB///8AA//wAAAAAAAP/8AAf//+AAP/8AAAAAAAD//AAH///gAB//AAAA' +
  'AAAA//gAB///4AAf/wAAAAAAAP/4AAf//+AAH/8AAAAAAAD/+AAD///gAB//AAAAAAAB//gAB///4AAf/4AAAAAAAf/4AAf/' +
  '/+AAH/+AAAAAAAH/+AAH///gAB//gAAAAAAB//wAB///4AAf/4AAAAAAAP/8AAf///AAP/8AAAAAAAD//AAP///wAD//AAAA' +
  'AAAA//4AD///8AB//wAAAAAAAP//AB////gAf/8AAAAAAAD//wA////8AP//AAAAAAAA///Af////gH//wAAAAAAAH//////' +
  '//////4AAAAAAAB////////////+AAAAAAAAP////////////gAAAAAAAD////////////wAAAAAAAAf///////////4AAAA' +
  'AAAAH///////////+AAAAAAAAA////////////AAAAAAAAAH///////////gAAAAAAAAB///////////4AAAAAAAAAH/////' +
  '/////8AAAAAAAAAA//////////+AAAAAAAAAAH/////////+AAAAAAAAAAA//////////AAAAAAAAAAAD/////////AAAAAA' +
  'AAAAAAf////////gAAAAAAAAAAAH////////4AAAAAAAAAAAD/////////AAAAAAAAAAAA/////////4AAAAAAAAAAAf////' +
  '////+AAAAAAAAAAAP/////////wAAAAAAAAAAD/////////8AAAAAAAAAAB//////////gAAAAAAAAAAf/////////4AAAAA' +
  'AAAAAH//////////AAAAAAAAAAD//////////wAAAAAAAAAA//////////8AAAAAAAAAAP//////////AAAAAAAAAAH/////' +
  '/////4AAAAAAAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAA' +
  'AAAAA///////////wAAAAAAAAAP//////////8AAAAAAAAAH///////////gAAAAAAAAB///////////8AAAAAAAAA//////' +
  '//////AAAAAAAAAP///////////wAAAAAAAAH///////////+AAAAAAAAB////////////gAAAAAAAAf///////////4AAAA' +
  'AAAAH///////////+AAAAAAAAB////////////gAAAAAAAAf///////////4AAAAAAAAH///////////+AAAAAAAAB//////' +
  '//////gAAAAAAAAf///////////4AAAAAAAAH///////////+AAAAAAAAB////////////gAAAAAAAAP///////////wAAAA' +
  'AAAAD///////////8AAAAAAAAAf//////////+AAAAAAAAAH///////////gAAAAAAAAA///////////wAAAAAAAAAH/////' +
  '/////4AAAAAAAAAAf/////////4AAAAAAAAAAD////v////8AAAAAAAAAAA////4H////AAAAAAAAAAAP///8A////wAAAAA' +
  'AAAAAB////AP///4AAAAAAAAAAAf///wB///+AAAAAAAAAAAD///wAP///AAAAAAAAAAAAf//8AB///gAAAAAAAAAAAB//8A' +
  'AP//wAAAAAAAAAAAAD/wAAAP/AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── fish — 비트맵 실루엣(128×128, 잉크 43.9%)
//   원본 마스크: fish.png
var MASK_FISH = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAHAAAAAAAAAAAAAAAAAAAA/+AAAAAAAAAAAAAAAAAAA//wAAAAAAAAAAAAAAAAAA//+AAAAAAAAAAAAAAAAAA//' +
  '/gAAAAAAAAAAAAAAAAAf//4AAAAAAAAAAAAAAAAAP//+AAAAAAAAAAAAAAAAAP///gAAAAAAAAAAAAAAAAH///4AAAAAAAAA' +
  'AAAAAAAD///+AAAAAAAAAAAAAAAAA////gAAAAAAAAAAAAAAAAf///wAAAAAAAAAAAAAAAAP///8AAAAAAAAAAAAAAAAH///' +
  '+AAAAAAAAAAAAAAAAD////gAAAAAH/gAAAAAAAA////4AAAAAP/+AAAAAAAAf///+AAAAAf//4AAAAAAAP////gAAAAP//+A' +
  'AAAAAAD////4AAAAP///wAAAAAAA/////AAAAH///8AAAAAAAf////wAAAH////gAAAAAAH////+AAAD////4AAAAAAD////' +
  '/wAAB////+AAAAAAA/////8AAA/////gAAAAAAP/////gAAf////4AAAAAAD/////4AAP////+AAAAAAB/////+AAH/////g' +
  'AAAAAB//////gAD/////wAAAAAB//////4AA/////8AAAAAB//////+AAf/////AAAAAB///////AAP/////gAAAAB//////' +
  '/gAD/////wAAAAA///////4AB/////8AAAAAf//////+AAf////+AAAAAf///////gAP/////gAAAAP///////4AD/////4A' +
  'AAAH////////AB//////AAAAD////////4Af/////wAAAA/////////AP/////8AAAAf////////4D//////gAAAP///////' +
  '//B//////4AAAD/////////wf/////+AAAB/////////////////gAAA/////////////////4AAAP/B//////////////+A' +
  'AAH/wH//////////////AAAB/8A//////////////wAAA//AH/8H//////////4AAAP/4B//Af/////////+AAAD//AP/gD/' +
  '/////////AAAB//4D/8Af/////////gAAA///A//AD/////////wAAAf//wH/wAf////////4AAAH//8B/+AD////////4AA' +
  'AB///Af/gA////////8AAAA///wP/4AH///////+AAAAP//8D/+AB////////AAAAD///A//gAP///////wAAAD///gP/8AD' +
  '///////+AAAD///wH//AAf///////wAAA///4B//wAH////////AAAf//+A//8AB////////4AAH///Af//AAf////////AA' +
  'B///4P//wAH////////4AAP//+H//4AB/////////AAD//////+AAf////////4AA///////gAH////////+AAP//////4AB' +
  '/////////wAH//////8AAf////////8AB///////AAH//x//////gAf//////gAD//4P/////4AD//////4AA//+D/////+A' +
  'Af/////8AAf//Af/////gAD//////AAH//wH/////4AAD/////gAD//4A/////+AAAH////4AB//8AP/////gAAA////8AAf' +
  '//AD/////wAAAH///+AAf//gAf////8AAAA////gAP//wAH/////AAAAD///4AP//wAA/////gAAAAf///AP//4AAP////4A' +
  'AAAD///8f//+AAB////+AAAAAP///////gAAf////gAAAAD///////4AAD////4AAAAA////////AAAf///+AAAAAP//////' +
  '/8AAD////wAAAAD////////gAA////8AAAAA////////8AAH////gAAAAf////////gAA////4AAAAH//+Af///4AAH///+A' +
  'AAAB///AH////AAAf///gAAAAP//wA////wAAD///4AAAAD//8AP///+AAAf//+AAAAA///AB////gAAB///AAAAAP//wAf/' +
  '//4AAAD//gAAAAB//8AD///+AAAAP/gAAAAAf//AA////gAAAAAAAAAAAH//gAH///4AAAAAAAAAAAA//4AA///+AAAAAAAA' +
  'AAAAH/8AAH///gAAAAAAAAAAAA//AAA///4AAAAAAAAAAAAH/wAAH//+AAAAAAAAAAAAA/4AAA///gAAAAAAAAAAAAH+AAAH' +
  '//8AAAAAAAAAAAAAfgAAA///AAAAAAAAAAAAAAAAAAD//wAAAAAAAAAAAAAAAAAAf/8AAAAAAAAAAAAAAAAAAB/+AAAAAAAA' +
  'AAAAAAAAAAAH/AAAAAAAAAAAAAAAAAAAAHAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── bear — 비트맵 실루엣(128×128, 잉크 50.5%)
//   원본 마스크: bear.png
var MASK_BEAR = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAfwAAAAAP4AAAAAAAAAAAAf+AAAAAP/AAAAAAAAAAAAP/4AAAAH/8AAAAAAAAAAAH//Af/4D//gAAAAA' +
  'AAAAAD//x///5//4AAAAAAAAAAA//////////AAAAAAAAAAAP/////////wAAAAAAAAAAH/////////+AAAAAAAAAAB/////' +
  '/////gAAAAAAAAAAf/////////4AAAAAAAAAAH/////////+AAAAAAAAAAB//////////gAAAAAAAAAAP/////////wAAAAA' +
  'AAAAAD/////////8AAAAAAAAAAA//////////AAAAAAAAAAAH//3//////gAAAAAAAAAAA//4///x//wAAAAAAAAAAAP/8H/' +
  '/4P/8AAAAAAAAAAAD//B//+D//AAAAAAAAAAAA//wf//g//wAAAAAAAAAAAf/8H//4P/+AAAAAAAAAAAH//B//+D//gAAAAA' +
  'AAAAAB//w///w//4AAAAAAAAAAA//+P//8f//AAAAAAAAAAAP///+B////wAAAAAAAAAAD///wAA///8AAAAAAAAAAA///wA' +
  'AD///AAAAAAAAAAAP//wD/Af//wAAAAAAAAAAD//4D/8B//8AAAAAAAAAAA//+A//Af//gAAAAAAAAAAP//AP/wD//4AAAAA' +
  'AAAAAD//wD/8A//8AAAAAAAAAAA//8A//AP//AAAAAAAAAAAP//AH/gD//wAAAAAAAAAAD//wA/wA//8AAAAAAAAAAA//8AH' +
  '4AP//AAAAAAAAAAAP//AAcAD//wAAAAAAAAAAB//4AAAB//4AAAAAAAAAAAf/+AAAAf/+AAAAAAAAAAAH//4AAAf//gAAAAA' +
  'AAAAAA///wAA///wAAAAAAAAAAAP////////8AAAAAAAAAAAB////////+AAAAAAAAAAAA/////////wAAAAAAAAAAA/////' +
  '/////AAAAAAAAAAA//////////8AAAAAAAAAAf//////////gAAAAAAAAAf//////////+AAAAAAAAAP///////////wAAAA' +
  'AAAAH///////////+AAAAAAAAH////////////4AAAAAAAD/////////////AAAAAAAB/////////////4AAAAAAA///////' +
  '///////AAAAAAAf/////////////4AAAAAAP//////////////AAAAAAD//////////////wAAAAAB//////////////+AAA' +
  'AAA///////////////wAAAAAf//////////////+AAAAAH///////////////gAAAAD///////////////8AAAAA////////' +
  '////////gAAAAf///////////////4AAAAP////////////////AAAAD////////////////wAAAB///8////////8///+AA' +
  'AAf//+H///////+H///gAAAH///B////////g///4AAAD///wf///////4P///AAAA///4H///////+B///wAAAP//+B////' +
  '////gf//8AAAD///Af///////4D///AAAB///wH///////+A///wAAAf//4B////////gP//+AAAH//+Af///////4B///gA' +
  'AB///gP////////Af//4AAAf//4D////////wH//+AAAH//+A////////8A///gAAB///AP////////AP//wAAAf//wH////' +
  '////4D//8AAAD///B////////+D///AAAA///8f////////j///wAAAP////////////////4AAAB////////////////+AA' +
  'AAf////////////////AAAAH////////////////4AAAB////////////////+AAAA/////////////////wAAAP/g//////' +
  '//////h/8AAAH/gD///////////AH/gAAB/wAf//////////gA/4AAAf8AB//////////gAP+AAAP+AAf/////////4AB/wA' +
  'AD/gAD/////////8AAf8AAA/4AAf////////+AAH/AAAP8AAH/////////gAB/wAAD/AAA/////////wAAf8AAA/4AAP////' +
  '////8AAH/AAAP+AAB/////////AAB/wAAD/gAAf////////gAAf4AAAf4AAH////////4AAH+AAAH/AAB////////+AAD/gA' +
  'AB/wAAf////////gAA/4AAAP+AAH////////4AAf8AAAD/gAB////////+AAH/AAAAf8AA/////////wAD/wAAAH/gAP////' +
  '////8AB/4AAAA/8AH/////////gA/8AAAAH/wD/////////8Af/AAAAB//B//////////wf/gAAAAP///////////////wAA' +
  'AAB///////////////4AAAAAP///+f////+P///8AAAAAA///+AP///wB///+AAAAAAH///AAAAAAAP//+AAAAAAAf//AAAA' +
  'AAAA//+AAAAAAAA/8AAAAAAAAA/8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── rabbit — 비트맵 실루엣(128×128, 잉크 36.7%)
//   원본 마스크: rabbit.png
var MASK_RABBIT = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAPgAAAAAPwAAAAAAAAAAAAP+AAAAAP/AAAAAAAAAAAAH/4AAAAH/4AAAAAAAAAAAD//AAAAD//AAAAAA' +
  'AAAAAA//4AAAB//wAAAAAAAAAAAf//AAAA//+AAAAAAAAAAAH//wAAAf//gAAAAAAAAAAD//+AAAH//4AAAAAAAAAAA///gA' +
  'AD//+AAAAAAAAAAAP//8AAA///wAAAAAAAAAAD///AAAP//8AAAAAAAAAAA///4AAH///AAAAAAAAAAAP//+AAB///wAAAAA' +
  'AAAAAD///gAA///8AAAAAAAAAAA///8AAP///AAAAAAAAAAAP///AAD///gAAAAAAAAAAD///wAB///4AAAAAAAAAAA///+A' +
  'Af//+AAAAAAAAAAAH///gAH///gAAAAAAAAAAB///4AB///4AAAAAAAAAAAf///AA///8AAAAAAAAAAAH///wAP///AAAAAA' +
  'AAAAAA///8AD///wAAAAAAAAAAAP///AA///8AAAAAAAAAAAD///wAP//+AAAAAAAAAAAAf//+AH///gAAAAAAAAAAAH///g' +
  'B///4AAAAAAAAAAAB///4Af//8AAAAAAAAAAAAP//+AH///AAAAAAAAAAAAD///gB///wAAAAAAAAAAAA///4A///4AAAAAA' +
  'AAAAAAH///AP//+AAAAAAAAAAAAB///wD///AAAAAAAAAAAAAP//8A///wAAAAAAAAAAAAD///AP//4AAAAAAAAAAAAAf//w' +
  'D//+AAAAAAAAAAAAAH//8A///AAAAAAAAAAAAAA///////wAAAAAAAAAAAAAP//////4AAAAAAAAAAAAAB//////+AAAAAAA' +
  'AAAAAAAf//////AAAAAAAAAAAAAAD//////wAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB///' +
  '///gAAAAAAAAAAAAAA//////8AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAP//////8AAAAAAAAAAAAAH///////gAAAAAA' +
  'AAAAAAD///////8AAAAAAAAAAAAA////////AAAAAAAAAAAAAf///////4AAAAAAAAAAAAH///////+AAAAAAAAAAAAD////' +
  '////gAAAAAAAAAAAA//f///7/8AAAAAAAAAAAAP/B///4P/AAAAAAAAAAAAH/wf//+D/wAAAAAAAAAAAB/4D///Af+AAAAAA' +
  'AAAAAAf+A///wH/gAAAAAAAAAAAH/gP//8B/4AAAAAAAAAAAB/4D///Af+AAAAAAAAAAAAf+A///wH/gAAAAAAAAAAAH/gP/' +
  '/8B/4AAAAAAAAAAAB/4D///Af+AAAAAAAAAAAAf/B///4P/gAAAAAAAAAAAH/wf4H+D/4AAAAAAAAAAAB/+PwAPx/8AAAAAA' +
  'AAAAAAP//4AB///AAAAAAAAAAAAD//+AAf//wAAAAAAAAAAAA///gAH//8AAAAAAAAAAAAf//4AB///gAAAAAAAAAAAP//+A' +
  'Af//4AAAAAAAAAAAD///wAP///AAAAAAAAAAAB///8AD///4AAAAAAAAAAA////gB////AAAAAAAAAAAP///8A////wAAAAA' +
  'AAAAAH////gf///+AAAAAAAAAAB//////////gAAAAAAAAAA//////////8AAAAAAAAAAP//////////AAAAAAAAAAH/////' +
  '/////4AAAAAAAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAH//////////4AAAAAAAAAD///////////AAAAA' +
  'AAAAA///////////wAAAAAAAAAP//////////8AAAAAAAAAD///////////AAAAAAAAAA///////////wAAAAAAAAAP/////' +
  '/////8AAAAAAAAAD///////////AAAAAAAAAA///////////wAAAAAAAAAP//////////8AAAAAAAAAB//////////+AAAAA' +
  'AAAAAf//////////gAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAP/////' +
  '/////8AAAAAAAAAP///////////wAAAAAAAAH///////////+AAAAAAAAD////////////wAAAAAAAB////////////+AAAA' +
  'AAAAf////////////gAAAAAAAH////////////4AAAAAAAD/////////////AAAAAAAA/////////////wAAAAAAAP//////' +
  '//////8AAAAAAAD/////////////AAAAAAAAf////////////gAAAAAAAH////////////4AAAAAAAB////////////+AAAA' +
  'AAAAP////////////AAAAAAAAB////////////gAAAAAAAAP///////////wAAAAAAAAB///////////4AAAAAAAAAH//wf/' +
  '/+D//4AAAAAAAAAAP/gAf/4AH/wAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── turtle — 비트맵 실루엣(128×128, 잉크 40.9%)
//   원본 마스크: turtle.png
var MASK_TURTLE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAD/AAAAAAAAAAAAAAAAAAAD/8AAAAAAAAAAAAAAAAAAB//gAAAAAAAAAAAAAAAAAA//8AAAAAAAAA' +
  'AAAAAAAAAf//gAAAAAAAAAAAAAAAAAH//4AAAAAAAAAAAAAAAAAD///AAAAAAAAAAAAAAAAAA///wAAAAAAAAAAAAAAAAAf/' +
  '/+AAAAAAAAAAAAAAAAAH///gAAAAAAAAAAAAAAAAB///4AAAAAAAAAAD/AAAAAf//+AAAAA/wAAAH//AAAAH///gAAAD//gA' +
  'AD//+AAAB///4AAAH//8AAD///4AAAf//+AAAH///wAB////gAAH///gAAH///+AAf///+AAB///4AAD////gAP////wAAf/' +
  '/+AAD////8AD////+AAH///gAB/////AA/////4AA///wAB/////wAP/////AAP//8AA/////8AB/////4AH///gAf/////A' +
  'Af/////AP////AP/////gAD/////4P////8H/////wAA//////P/////7/////8AAD/////////////////+AAAf////////' +
  '////////+AAAD/////////////////AAAAP////////////////AAAAA////////////////AAAAAD///////////////AAA' +
  'AAAf////wH////////gAAAAAB////gAf///////gAAAAAAH///gAD///////wAAAAAAA///wAAf//////wAAAAAAAH//4AAD' +
  '//////4AAAAAAAAf/4AAA//////4AAAAAAAAD/8AAAH/////8AAAAAAAAA//AAAB//////AAAAAAAAAH/gAAAf/////gAAAA' +
  'AAAAD/wAAAH/////8AAAAAAAAA/4AAAB//////gAAAAAAAAf8AAAA//////4AAAAAAAAH/AAAAP//4D/+AAAAAAAAD/gAAAD' +
  '//4AP/wAAAAAAAA/4AAAB//+AB/8AAAAAAAAf8AAAAf//AAH/gAAAAAAAH/AAAAH//wAB/4AAAAAAAB/gAAAD//4AAP/AAAA' +
  'AAAA/4AAAB//+AAB/wAAAAAAAP+AAAAf//gAAf8AAAAAAAD/AAAAP//4AAD/AAAAAAAB/wAAAD//+AAA/4AAAAAAAf8AAAB/' +
  '//gAAH+AAAAAAAH/AAAA///8AAB/gAAAAAAB/wAAAf///AAAf4AAAAAAAf8AAAP///wAAH/AAAAAAAH/AAAH///+AAB/wAAA' +
  'AAAB/wAAD////gAAf8AAAAAAA/8AAD////4AAH/AAAAAAAP/gAB/////AAB/wAAAAAAD/4AB/////wAAf8AAAAAAA//AB///' +
  '//+AAP/AAAAAAAP/8B//////wAD/wAAAAAAD/////////8AB/8AAAAAAAf/////////gA//AAAAAAAH/////////+Af/wAAA' +
  'AAAB//////////4f/4AAAAAAAf////////////+AAAAAAAH//////z//////gAAAAAAB//////AH/////4AAAAAAAP/////g' +
  'Af////8AAAAAAAD/////gAD/////AAAAAAAA/////wAAf////wAAAAAAAP////4AAD////8AAAAAAAB////8AAA////+AAAA' +
  'AAAAf////AAAH////gAAAAAAAH////gAAA////4AAAAAAAA////wAAAP///8AAAAAAAAP///4AAAB////AAAAAAAAB///+AA' +
  'AAf///gAAAAAAAAf///AAAAD///4AAAAAAAAD///wAAAA///8AAAAAAAAA///4AAAAH///AAAAAAAAAf//+AAAAB///4AAAA' +
  'AAAAP///gAAAAf///AAAAAAAAP///wAAAAH///8AAAAAAAH///8AAAAB////gAAAAAAD////AAAAAf///8AAAAAAB////wAA' +
  'AAH////gAAAAAA////+AAAAB////8AAAAAAf////gAAAAf////gAAAAAP////8AAAAP////8AAAAAH/////AAAAH/////gAA' +
  'AAD/////4AAAD/////8AAAAA//////gAAB//////AAAAAf/////8AAA//////4AAAAP//////8AD///////AAAAD////////' +
  '////////wAAAB////5//////+f///+AAAAf///8P//////D////gAAAP////A//////A////8AAAD////gD/////gH////AA' +
  'AA////wAf////gA////4AAAf///4AB////gAH///+AAAH///8AAD///AAA////gAAB///+AAAH/+AAAH///4AAAf///AAAAD' +
  'wAAAA///+AAAH///gAAAAAAAAAH///gAAB///wAAAAAAAAAA///4AAAf//4AAAAAAAAAAH//+AAAH//8AAAAAAAAAAA///gA' +
  'AB//8AAAAAAAAAAAD//4AAAf/+AAAAAAAAAAAAf/+AAAD//AAAAAAAAAAAAB//AAAA//AAAAAAAAAAAAAP/wAAAH/AAAAAAA' +
  'AAAAAAA/4AAAA/AAAAAAAAAAAAAAB8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── whale — 비트맵 실루엣(128×128, 잉크 18.8%)
//   원본 마스크: whale.png
var MASK_WHALE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA+AAAAAAAAAAAAAAAAAAAA/' +
  'gAAAAAAAAAAAAAAAAAAAf4AAAAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAAH/AAAAAAAAAAAAAAAAAAAD/wAAAAAAAAAA' +
  'AAAAAAAAB/8AAAAAAAAAAAAAAAAAAAf+AAAAAAAAAAAAAAAAAAAP/gAAAAAAAAAAAAAAAAAAH/4AAAADAAAAAAAAAAAAAB/+' +
  'AAAAAwAAAAAAAAAAAAA//gAAAAeAAAAAAAAAAAAAP/4AAAAH4AAAAAAAAAAAAH/+AAAAB/gAAAAAAAAAAAB//gAAAAf+AAAA' +
  'AAAAAAAA//4AAAAH/wAAYAAAAAAAAf/+AAAAB//AB+AAAAAAAH///wAAAAf/5//gAAAAAB////+AAAAD////4AAAAAH/////' +
  '8AAAA////8AAAAAP//////4AAAH////AAAAAf///////gAAB////wAAAAf////////AAAP///4AAAAf////////8AAB///8A' +
  'AAAf/////////wAAP//+AAAAf//////////AAB///AAAAP//////////8AAP//gAAAH///////////wAD//AAAAD////////' +
  '////AB//AAAAB////////////8A//AAAAA//+A////////////wAAAAf/8AB///////////4AAAAP/8AAP//////////+AAA' +
  'AH/+AAD///////////AAAAB//AAB///////////wAAAA//gAA///////////4AAAAP/wAA///////////+AAAAD/8AA/////' +
  '///////AAAAB//wB////////////gAAAA///////////8f///4AAAAf//////////gAP//8AAAAH//////////AAAf/+AAAA' +
  'B//////////AAAD//AAAAAf/+AH/////AAAAf/gAAAAD/8AAP////gAAAH/wAAAAAfAAAAf///gAAAB/4AAAAAHwAAAB///w' +
  'AAAA/8AAAAAA8AAAAP//wAAAAf+AAAAAADwAAAD//4AAAAf+AAAAAAAfAAAAf/+AAAA//AAAAAAAD8AAAH//wAAA//AAAAAA' +
  'AAH+AAB//+AAH//AAAAAAAAAP/AAP//gA//+AAAAAAAAAAP//3//////4AAAAAAAAAAAP////////AAAAAAAAAAAAAA/////' +
  '/AAAAAAAAAAAAAAAAA///AAAAAAAAAAAAAAAAAAP//wAAAAAAAAAAAAAAAAAB//+AAAAAAAAAAAAAAAAAAP//gAAAAAAAAAA' +
  'AAAAAAAB//8AAAAAAAAAAAAAAAAAAP//gAAAAAAAAAAAAAAAAAB//4AAAAAAAAAAAAAAAAAAP//AAAAAAAAAAAAAAAAAAB//' +
  '4AAAAAAAAAAAAAAAAAAH//AAAAAAAAAAAAAAAAAAAf/wAAAAAAAAAAAAAAAAAAB/4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── horse — 비트맵 실루엣(128×128, 잉크 41.7%)
//   원본 마스크: horse.png
var MASK_HORSE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAADAAAAAAAAAAAAAAAAAAAABwAAAAAAAAAAAAAAAAAAAA8AAAAAAAAAAAAAAAAAAAAfgAMAAAAAA' +
  'AAAAAAAAAAAP4AHAAAAAAAAAAAAAAAAAD+AB4AAAAAAAAAAAAAAAAB/gA+AAAAAAAAAAAAAAAAA/8APgAAAAAAAAAAAAAAAA' +
  'P/AH4AAAAAAAAAAAAAAAAD/wD+AAAAAAAAAAAAAAAAB/8B/gAAAAAAAAAAAAAAAAf+A/4AAAAAAAAAAAAAAAAH/wf8AAAAAA' +
  'AAAAAAAAAAD////AAAAAAAAAAAAAAAf/////wAAAAAAAAAAAAAA//////8AAAAAAAAAAAAAD//////+AAAAAAAAAAAAAD///' +
  '////gAAAAAAAAAAAAB///////wAAAAAAAAAAAAB///////8AAAAAAAAAAAAA////////gAAAAAAAAAAAAf///////8AAAAAA' +
  'AAAAAAAP///////gAAAAAAAAAAAAH///////8AAAAAAAAAAAAH////////gAAAAAAAAAAAH////////4AAAAAAAAAAAH////' +
  '/////AAAAAAAAAAAH/////////4AAAAAAAAAAD/////////+AAAAAAAAAAB//////////wAAAAAAAAAA//////////8AAAAA' +
  'AAAAAf//////////AAAAAAAAAAP//////////wAAAAAAAAAH//////////8AAAAAAAAAD///8///////AAAAAAAAAB///+f/' +
  '/////wAAAAAAAAAf///n//////+AAAAAAAAAP///z///////gAAAAAAAAH///4///////8AAAAAAAAB///+f///////gAAAA' +
  'AAAAf///H///////4AAAAAAAAPn//j////////AAAAAAAADj//4////////wAAAAAAAAx//8P///////+AAAAAAAAA///D//' +
  '//////gAAAAAAAAf//h////////8AAAAAAAAP//4f////////AAAAAAAAD//8H////////4AAAAAAAB///B////////+AAAA' +
  'AAAA///wf////////wAAAAAAAf//4P////////+AAAAAAAH//+D/////////gAAAAAAD///g/////////8AAAAAAA///wP//' +
  '///////gAAAAAAf//8D/////////4AAAAAAH///A//////////AAAAAAB///wP///5/////wAAAAAA///4D///+D////8AAA' +
  'AAAP//+A////gP////gAAAAAD///gP///8A////4AAAAAA///4D////AD///8AAAAAAf//+Af///4Af///AAAAAAH///AH//' +
  '/+AD///wAAAAAB///wB////wAf//8AAAAAAfv/8Af///8AD//+AAAAAAHz//AH////gA///gAAAAAB8//wA////8AP//4AAA' +
  'AAAOf/8AP////AB//8AAAAAADn//AD////4Af//AAAAAAAx//wAf////AD//gAAAAAAM//8AH////4Af/wAAAAAAAP/+AA//' +
  '//+AB/gAAAAAAAD//gAP////wAAAAAAAAAAA//4AD////+AAAAAAAAAAAP/+AAf////wAAAAAAAAAAH//gAD////+AAAAAAA' +
  'AAAB//4AA/////wAAAAAAAAAAf/8AAH////+AAAAAAAAAAH//AAA/////wAAAAAAAAAB//wP/v////+AAAAAAAAAAf/4P///' +
  '////wAAAAAAAAAH/+P///////+AAAAAAAAAA//H////////wAAAAAAAAAP/n////////+AAAAAAAAAD/7/////////gAAAAA' +
  'AAAA///////////8AAAAAAAAAP///////////gAAAAAAAAB///////////4AAAAAAAAAf3//////////AAAAAAAAAH5/////' +
  '/////4AAAAAAAAA+//////////+AAAAAAAAAPP//////////wAAAAAAAAB3//////////8AAAAAAAAAZ///////////gAAAA' +
  'AAAAAf//////////4AAAAAAAAAP//////////+AAAAAAAAAD///////////wAAAAAAAAA///////////8AAAAAAAAAf/////' +
  '//////AAAAAAAAAH///////////4AAAAAAAAB///////////+AAAAAAAAAf///////////gAAAAAAAAH///////////4AAAA' +
  'AAAAB///////////+AAAAAAAAAf///////////gAAAAAAAAH///////////4AAAAAAAAB///////////+AAAAAAAAAf/////' +
  '//////gAAAAAAAAH///////////4AAAAAAAAB///////////+AAAAAAAAAP///////////AAAAAAAAAD///////////wAAAA' +
  'AAAAAP//////////gAAAAAAAAAAf/////////AAAAAAAAAAAA/////////AAAAAAAAAAAAB///////8AAAAAAAAAAAAAA///' +
  '///gAAAAAAAAAAAAAAAH///wAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── bird — 비트맵 실루엣(128×128, 잉크 36.1%)
//   원본 마스크: bird.png
var MASK_BIRD = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAB8AAAAAAAAAAAAAAAAAAAB/4AAAAAAAAAAAAAAAAAAA//AAAAAA' +
  'AAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAH//AAAAAAAAAAAAAAAAAAB//4AAAAAAAAAAAAAAAAAA///AAAAAAAAAAAAAAAA' +
  'AAP//8AAAAAAAAAAAAAAAAAD///gAAAAAAAAAAAAAAAAA///8AAAAAAAAAAAAAAAAAP///gAAAAAAAAAAAAAAAAD///8AAAA' +
  'AAAAAAAAAAAAA////wAAAAAAAAAAAAAAAAP///+AAAAAAAAAAAAAAAAD////4AAAAAAAAAAAAAAAAf////AAAAAAAAAAAAAA' +
  'AAH////8AAAAAAAAAAAOAAAB/////gAAAAAAAAAAPgAAAf////+AAAAAAAAAAH4AAAD/////wAAAAAAAAAD+AAAA//////AA' +
  'AAAAAAAA/gAAAH/////4AAAAAAAAAf4AAAB//////gAAAAAAAwH+AAAAP/////8AAAAAAAeD/AAAAB//////gAAAAAAH5/wA' +
  'AAAf/////8AAAAAAB//4AAAAH//////gAAAAAA//8AAAAD//////8AAAAAAP//AAAAH///////AAAAAAD//wAAAH///////4' +
  'AAAAAA//+AAAD///////+AAAAAAH//8AAA////////wAAAAAB///4AAf///////8AAAP8Af//+AAH////////gAAf/wD///A' +
  'AB////////4AAP/+Af//wAAf///////+AAH//wD//4AAH////////wAD//8A//4AAB////////8AB///gH/4AAAf////////' +
  'AA///4D/8AAAD////////wAf///A//AAAA////////8AH///4f/wAAAP////////AD////v/8AAAD////////4B///////gA' +
  'AAf///////+Af//////8AAAD////////gP//////+AAAA////////8H///////AAAAH////////D/////j/AAAAB////////' +
  '//////wAAAAAAP/////////////4AAAAAAB/////////////+AAAAAAAP/////////////AAAAAAAB/////////////wAAAA' +
  'AAAf////////////8AAAAAAAB/////////////AAAAAAAAP///////////jgAAAAAAAB///////////wYAAAAAAAAP//////' +
  '////8AAAAAAAAAB///////////AAAAAAAAAAH//////////wAAAAAAAAAA//////////8AAAAAAAAAAB//////////AAAAAA' +
  'AAAAAP/////////wAAAAAAAAAAAf////////8AAAAAAAAAAAD/////////AAAAAAAAAAAA/////////wAAAAAAAAAAAH////' +
  '////4AAAAAAAAAAAD////////+AAAAAAAAAAAB/////////gAAAAAABwAAD/////////4AAAAAAB//Af/////////+AAAAAA' +
  'A//////////////AAAAAAAf/////////////wAAAAAAH/////////////8AAAAAAB/////////////+AAAAAAAf/////////' +
  '////gAAAAAAH/////////////wAAAAAAB/////////////4AAAAAAAP////////////+AAAAAAAB/////////////AAAAAAA' +
  'Af////////////gAAAAAAAB////////////wAAAAAAAAf///////////4AAAAAAAAH///////////8AAAAAAAAB/////////' +
  '//+AAAAAAAAA////////////AAAAAAAAAP///////////AAAAAAAAAD///////////gAAAAAAAAA///////////AAAAAAAAA' +
  'AP/////wP///AAAAAAAAAAD/////4AH/8AAAAAAAAAAA/////8AAAAAAAAAAAAAAAH////+AAAAAAAAAAAAAAAA/////gAAA' +
  'AAAAAAAAAAAAH////wAAAAAAAAAAAAAAAAf///4AAAAAAAAAAAAAAAAB///+AAAAAAAAAAAAAAAAAf///AAAAAAAAAAAAAAA' +
  'AAD///gAAAAAAAAAAAAAAAAA///4AAAAAAAAAAAAAAAAAP//8AAAAAAAAAAAAAAAAAD///AAAAAAAAAAAAAAAAAAf//gAAAA' +
  'AAAAAAAAAAAAAD//wAAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAD/8AAAAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── deer — 비트맵 실루엣(128×128, 잉크 31.5%)
//   원본 마스크: deer.png
var MASK_DEER = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAD4AAAAAAAAAAAfAAAAAAAD/AAAAAAAAAAAP8AAAAAAB/4AAAAAAAAAAH/gAAAAAA//AAAAAAAAAAD/8AAA' +
  'AAAf/4AAAAAAAAAA//gAAAAAP/+AAAAAAAAAAf/8AAAAAD//gAAAAAAAAAH//AAAAAB//4AAAAAAAAAA//4AAAAAf/8AAAAA' +
  'AAAAAP/+AAAAAH//AAAAAAAAAAD//wAAAAD//gAAAAAAAAAAf/8AAAAA//4AAAAAAAAAAH//AAAAAP/8AAAAAAAAAAA//wAA' +
  'AAD//AAAAAAAAAAAP/8AAAAA//wAAAAAAAAAAD//AAAAAP/8AAAAAAAAAAA//wAAAAD//AAAAAAAAAAAP/8AAAAA//wAAAAA' +
  'AAAAAD//AAAAAP/+AAAAAAAAAAB//wAAAAD//wAAAAAAAAAA//8AAAAA///AAAAAAAAAA///AAAAAP///AAAAAAAAD///wAA' +
  'AAD///8AAAAAAAD///8AAAAA////wAAAAAAD////AAAAAH///+AAAAAAB////wAAAAB////wAAAAAA////4AAAAAf///+AAA' +
  'AAAf///+AAAAAD////wAAAAAP////AAAAAA////8AAAAAD////wAAAAAH////gAAAAB////4AAAAAA////4AAAAAf///+AAA' +
  'AAAP////AAAAAH////AAAAAAB////wAAAAD////gAAAAAAP///8AAAAA////wAAAAAAA////gAAAAf///wAAAAAAAH///4AA' +
  'AAH///4AAAAAAAAf///AAAAD///4AAAAAAAAB///wAAAA///4AAAAAAAAAH//+AAAAf//4AAAAAAAAAAP//wAAAP//wAAAAA' +
  'AAAAAA//+AAAD//wAAAAAAAAA/gD//wAAD//wA/AAAAAAA//gP/+AAB//4H/8AAAAAAf//B///////4P//gAAAAAD//8P///' +
  '///8P//wAAAAAA///x//////+P//8AAAAAAP//+P//////H///AAAAAAD///5//////n///wAAAAAAf///P/////z///4AAA' +
  'AAAH/////////////+AAAAAAA//////////////gAAAAAAP/////////////wAAAAAAB/////////////4AAAAAAAP//////' +
  '//////+AAAAAAAD/////////////AAAAAAAAf////////////gAAAAAAAD////////////wAAAAAAAAf///////////4AAAA' +
  'AAAAB///////////4AAAAAAAAAH//////////4AAAAAAAAAAP/////////wAAAAAAAAAAAB///////4AAAAAAAAAAAAAfh//' +
  '//h+AAAAAAAAAAAAAD4P///wfgAAAAAAAAAAAAA+B///4HwAAAAAAAAAAAAAPwP//+D8AAAAAAAAAAAAAD+D///B/AAAAAAA' +
  'AAAAAAAfw///w/gAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAA//////+AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAB///' +
  '///gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////4AAAAAAA' +
  'AAAAAAAH/////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB//g' +
  'B//gAAAAAAAAAAAAAAf/wAP/4AAAAAAAAAAAAAAH/8AD/+AAAAAAAAAAAAAAB//AA//gAAAAAAAAAAAAAAf/wAP/4AAAAAAA' +
  'AAAAAAAP/8AD//AAAAAAAAAAAAAAD//gB//wAAAAAAAAAAAAAA//4Af/8AAAAAAAAAAAAAAP//AP//gAAAAAAAAAAAAAH//4' +
  'H//4AAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAP//////8AAAAAAAAAAAAAD///////AAAAAAA' +
  'AAAAAAA///////4AAAAAAAAAAAAAf//////+AAAAAAAAAAAAAH///////gAAAAAAAAAAAAB///////4AAAAAAAAAAAAA////' +
  '////AAAAAAAAAAAAAP///////wAAAAAAAAAAAAD///////8AAAAAAAAAAAAA////////AAAAAAAAAAAAAP///////wAAAAAA' +
  'AAAAAAD///////8AAAAAAAAAAAAA////////AAAAAAAAAAAAAH///////gAAAAAAAAAAAAA///////wAAAAAAAAAAAAAH///' +
  '///4AAAAAAAAAAAAAA//////8AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAAf/////AAAAAAAAAAAAAAAD/////AAAAAAAA' +
  'AAAAAAAAf////gAAAAAAAAAAAAAAAB////gAAAAAAAAAAAAAAAAH///gAAAAAAAAAAAAAAAAAf//gAAAAAAAAAAAAAAAAAA/' +
  '/gAAAAAAAAAAAAAAAAAAB+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── hearingaid — 비트맵 실루엣(128×128, 잉크 46.0%)
//   원본 마스크: hearingaid.png
var MASK_HEARINGAID = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAf//wAAAAAAAAAAAAAAAAB////wAAAAAAAAAAAAAAAD/////gAAAAAAAAAAAAAAH/////+AAAAAAA' +
  'AAAAAAAH//////4AAAAAAAAAAAAAP///////gAAAAAAAAAAAAH///////+AAAAAAAAAAAAH////////4AAAAAAAAAAAH////' +
  '/////AAAAAAAAAAAD/////////4AAAAAAAAAAD//////////gAAAAAAAAAB//////////8AAAAAAAAAA///////////gAAAA' +
  'AAAAAf//////////8AAAAAAAAAf///////////gAAAAAAAAP///////////4AAAAAAAAH////////////AAAAAAAAD//////' +
  '//////4AAAAAAAB/////////////AAAAAAAA///////wAH///wAAAAAAAP//////wAAf//+AAAAAAAH//////wAAB///gAAA' +
  'AAAD//////4AAAH//8AAAAAAB//////4AAAA///AAAAAAA//////8AAAAH//4AAAAAAP/////+AAAAA//+AAAAAAH//////A' +
  'AAAAH//wAAAAAD//////wAAAAB//8AAAAAA//////4AAAAAP//AAAAAAf/////+AAAAAB//4AAAAAP//////AAAAAAf/+AAA' +
  'AAD//////gAAAAAH//gAAAAB//////4AAAAAA//8AAAAAf/////+AAAAAAP//AAAAAP//////AAAAAAB//wAAAAD//////wA' +
  'AAAAAf/8AAAAB//////8AAAAAAH//gAAAAf//////AAAAAAA//4AAAAP//////gAAAAAAP/+AAAAD//////4AAAAAAD//gAA' +
  'AA//////+AAAAAAA//4AAAAf//////gAAAAAAP/+AAAAH//////4AAAAAAD//gAAAD//////+AAAAAAAf/4AAAA///////gA' +
  'AAAAAH/+AAAAP//////4AAAAAAB//gAAAH//////+AAAAAAAf/4AAAB///////gAAAAAAH/+AAAAf//////4AAAAAAB//gAA' +
  'AH//////+AAAAAAAf/4AAAD///////gAAAAAAH/+AAAA///////4AAAAAAB//gAAAP//////+AAAAAAAf/4AAAD///////gA' +
  'AAAAAP/+AAAB///////4AAAAAAD//gAAAf//////+AAAAAAA//wAAAH///////gAAAAAAP/8AAAB///////4AAAAAAD//AAA' +
  'Af//////+AAAAAAA//wAAAH///////wAAAAAAP/8AAAD///////8AAAAAAD//AAAA////////AAAAAAB//wAAAP///////wA' +
  'AAAAAf/+AAAD///////8AAAAAAP//gAAA////////AAAAAAH//8AAAP///////wAAAAAB///gAAD///////+AAAAAA///4AA' +
  'A////////gAAAAAP///AAAP///////4AAAAAD///wAAD///////+AAAAAB///8AAA////////gAAAAAf///AAAP///////4A' +
  'AAAAP///wAAD///////+AAAAAH///8AAA////////wAAAAD////AAAP///////8AAAAD////wAAD////////AAAAB////+AA' +
  'A////////wAAAA/////gAAP///////8AAAAf////8AAD////////gAAAH/////gAA////////4AAAD/////+AAP///////+A' +
  'AAB//////wAD////////gAAAf/////8AA////////4AAAH//////gAP///////+AAAD//////4AB////////wAAA///////A' +
  'Af///////8AAAP//////wAH////////AAAD//////8AB////////wAAA///////AAf///////8AAAP//////wAH////////A' +
  'AAD//////8AA////////wAAA///////AAP///////8AAAP//////wAD////////AAAD//////8AA////////wAAA//////+A' +
  'AP///////8AAAH//////gAB////////AAAB//////wAAf///////wAAAf/////8AAH///////8AAAD/////+AAB////////A' +
  'AAAf/////AAAP///////wAAAH/////gAAD///////8AAAA/////gAAA////////AAAAH////gAAAH///////wAAAA////gAA' +
  'AB///////8AAAAH///gAAAAf//////+AAAAAf//AAAAAD///////gAAAAA/8AAAAAA///////4AAAAAAAAAAAAAH//////8A' +
  'AAAAAAAAAAAAB///////AAAAAAAAAAAAAAP//////gAAAAAAAAAAAAAB//////wAAAAAAAAAAAAAAf/////4AAAAAAAAAAAA' +
  'AAB/////8AAAAAAAAAAAAAAAP////+AAAAAAAAAAAAAAAB/////AAAAAAAAAAAAAAAAH///+AAAAAAAAAAAAAAAAAP//+AAA' +
  'AAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── tooth — 비트맵 실루엣(128×128, 잉크 56.4%)
//   원본 마스크: tooth.png
var MASK_TOOTH = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAB/+AAAAAAA//AAAAAAAAAD//8AAAAAB//+AAAAAAAAH///4AAAAD///4AAAAAAAD////gAAAD////gAAA' +
  'AAAD////+AAAD////8AAAAAAB/////4AAD/////wAAAAAA//////gAD/////+AAAAAAf//////AH//////wAAAAAP///////' +
  '///////+AAAAAH///////////////gAAAAD///////////////8AAAAB////////////////gAAAAf///////////////4AA' +
  'AAP////////////////AAAAD////////////////wAAAB////////////////+AAAAf////////////////gAAAH////////' +
  '////////8AAAD/////////////////AAAA/////////////////wAAAP////////////////+AAAH/////////////////gA' +
  'AB/////////////////4AAAf////////////////+AAAH/////////////////gAAB/////////////////4AAAf////////' +
  '////////+AAAP/////////////////wAAD/////////////////8AAA//////////////////AAAP/////////////////wA' +
  'AB/////////////////4AAAf////////////////+AAAH/////////////////gAAB/////////////////4AAAf////////' +
  '////////+AAAH/////////////////gAAA/////////////////4AAAP////////////////8AAAD/////////////////AA' +
  'AA/////////////////wAAAH////////////////4AAAB////////////////+AAAAP////////////////gAAAD////////' +
  '////////wAAAA////////////////8AAAAH///////////////+AAAAB////////////////gAAAAP///////////////wAA' +
  'AAB///////////////8AAAAAf//////////////+AAAAAD///////////////AAAAAA///////////////wAAAAAH///////' +
  '///////4AAAAAA//////////////+AAAAAAP//////////////AAAAAAB//////////////gAAAAAAf/////////////4AAA' +
  'AAAD/////////////8AAAAAAA//////////////AAAAAAAH/////////////wAAAAAAB/////////////4AAAAAAAf//////' +
  '//////+AAAAAAAH/////////////gAAAAAAA/////////////4AAAAAAAP////////////8AAAAAAAD/////////////AAAA' +
  'AAAA/////////////wAAAAAAAP////////////8AAAAAAAD/////////////AAAAAAAA/////////////wAAAAAAAP//////' +
  '//////8AAAAAAAD/////////////AAAAAAAAf////////////wAAAAAAAH////////////4AAAAAAAB////////////+AAAA' +
  'AAAAf////////////gAAAAAAAH////////////4AAAAAAAB////////////+AAAAAAAAf////////////gAAAAAAAH//////' +
  '//////4AAAAAAAB////////////+AAAAAAAAf////////////gAAAAAAAH////////////4AAAAAAAB////////////+AAAA' +
  'AAAAP////////////gAAAAAAAD/////+f/////4AAAAAAAA/////+B/////+AAAAAAAAP/////AP/////AAAAAAAAD/////g' +
  'B/////wAAAAAAAA/////wAP////8AAAAAAAAP////8AD/////AAAAAAAAB////+AAf////gAAAAAAAAf////gAH////4AAAA' +
  'AAAAH////wAA////+AAAAAAAAB////8AAP////gAAAAAAAAP////AAD////wAAAAAAAAD////gAAf///8AAAAAAAAA////4A' +
  'AH////AAAAAAAAAP///+AAB////wAAAAAAAAB////gAAf///4AAAAAAAAAf///wAAD///+AAAAAAAAAD///8AAA////AAAAA' +
  'AAAAA////AAAP///wAAAAAAAAAP///wAAB///8AAAAAAAAAB///8AAAf//+AAAAAAAAAAf//+AAAH///gAAAAAAAAAD///gA' +
  'AB///wAAAAAAAAAA///4AAAf//8AAAAAAAAAAH//8AAAD//+AAAAAAAAAAB///AAAA///gAAAAAAAAAAP//wAAAH//wAAAAA' +
  'AAAAAB//4AAAB//4AAAAAAAAAAAP/+AAAAf/+AAAAAAAAAAAB//AAAAD//AAAAAAAAAAAAP/gAAAAf/AAAAAAAAAAAAB/wAA' +
  'AAD/gAAAAAAAAAAAAH4AAAAAfgAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── mouth — 비트맵 실루엣(128×128, 잉크 34.1%)
//   원본 마스크: mouth.png
var MASK_MOUTH = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAD/wAAAP/AAAAAAAAAAAAAH//gAAf/+AAAAAAAAAAAAH//+A' +
  'Af//4AAAAAAAAAAAH///wAP///gAAAAAAAAAAD////AH///8AAAAAAAAAAB////4H////gAAAAAAAAAB//////////+AAAAA' +
  'AAAAA///////////wAAAAAAAAAf//////////+AAAAAAAAAP///////////wAAAAAAAAH///////////+AAAAAAAAD//////' +
  '//////wAAAAAAAA////////////8AAAAAAAAf////////////gAAAAAAAP////////////8AAAAAAAH/////////////gAAA' +
  'AAAD/////////////8AAAAAAB//////////////gAAAAAA//////////////8AAAAAAf//////////////gAAAAAP///////' +
  '///////8AAAAAH///////////////gAAAAD///////////////8AAAAB////////////////gAAAA////////////////8AA' +
  'AAf////////////////gAAAP////////////////8AAAP/////////////////wAAH/////////////////+AAH/////////' +
  '/////////4AH///////////////////gB///////////////////4AH//////////////////4AA//////////////////8A' +
  'AH/////////////////+AAA//////////////////AAAH/////////////////wAAA/////////////////4AAAP////////' +
  '////////8AAAB////////////////+AAAAP////////////////AAAAB////////////////gAAAAf///////////////4AA' +
  'AAD///////////////8AAAAAf//////////////+AAAAAD///////////////AAAAAAf//////////////wAAAAAH///////' +
  '///////4AAAAAA//////////////8AAAAAAH/////////////+AAAAAAA//////////////AAAAAAAH/////////////wAAA' +
  'AAAA/////////////4AAAAAAAH////////////8AAAAAAAA////////////+AAAAAAAAH////////////AAAAAAAAA//////' +
  '//////gAAAAAAAAH///////////gAAAAAAAAA///////////wAAAAAAAAAH//////////4AAAAAAAAAAf/////////8AAAAA' +
  'AAAAAD/////////8AAAAAAAAAAAP////////+AAAAAAAAAAAB////////+AAAAAAAAAAAAH///////+AAAAAAAAAAAAAf///' +
  '///+AAAAAAAAAAAAAA//////+AAAAAAAAAAAAAAB/////8AAAAAAAAAAAAAAAD////wAAAAAAAAAAAAAAAAB//+AAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── lungs — 비트맵 실루엣(128×128, 잉크 43.1%)
//   원본 마스크: lungs.png
var MASK_LUNGS = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAD/AAAAAAAAAAAAAAAAAAAD/8AAAAAAAAAAAAAAAAAAB//AAAAAAAAAAAAAAAAAAAf/4AAAAAAAAA' +
  'AAAAAAAAAH/+AAAAAAAAAAAAAAAAAAB//gAAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAH/+AAAAAAAAAAAAAAAAAAB/' +
  '/gAAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAH/+AAAAAAAAAAAAAAAAAAB//gAAAAAAAAAAAAAAAAAAf/4AAAAAAAAA' +
  'AAAAAAAAAH/+AAAAAAAAAAAAAAAAAAB//gAAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAH/+AAAAAAAAAAAAAAAAAAB/' +
  '/gAAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAH/+AAAAAAAAAAAAAAAAAAB//gAAAAAAAAAAAAAAAAAAf/4AAAAAAAAA' +
  'AAAAAAAAAH/+AAAAAAAAAAAAAAAAgAB//gABAAAAAAAAAAAAH/gAf/4AH/gAAAAAAAAAAH/8AH/+AH/+AAAAAAAAAAH//wB/' +
  '/gD//4AAAAAAAAAH//+Af/4B///gAAAAAAAAD///wH/+A///8AAAAAAAAB///8B//gP///gAAAAAAAA////gf/4H///8AAAA' +
  'AAAAf///4H/+B////gAAAAAAAP///+B//g////8AAAAAAAH////wf/4P////gAAAAAAD////8H/+D////8AAAAAAB/////B/' +
  '/h/////gAAAAAA/////4f/4f////8AAAAAAP////+H/+H/////AAAAAAH/////h//h/////4AAAAAD/////4f/4f/////AAA' +
  'AAA/////+H/+H/////wAAAAAf/////h//h/////+AAAAAP/////4f/4f/////gAAAAD/////+H/+H/////8AAAAB//////h/' +
  '/h//////AAAAAf/////wf/4f/////4AAAAP/////8P/+D/////+AAAAD//////D//w//////wAAAB//////h//8H/////8AA' +
  'AAf/////wf//h//////gAAAH/////8P//8P/////4AAAD/+H//+H///h///h//AAAA//Af//D///8P//gP/wAAAf/gD//h//' +
  '//h//wB/+AAAH/4Af/w////8P/4Af/gAAB/8AD/4f////h/8AH/4AAA//AAf4P////8H+AB//AAAP/4ABwH/////geAAf/wA' +
  'AD/+AAAD/////4AAAH/8AAA//wAAA//4P//AAAD//gAAf/8AAAf/8A//4AAA//4AAH//gAAP/+AH//AAAf/+AAD//+AAD//A' +
  'A//4AAf//gAA///4AB//gAH/+AAf//8AAP////g//wAA//wf////AAD////4P/4AAH/8H////wAB////8H/8AAA//h////8A' +
  'Af////B/+AAAH/4P////gAH////g//gAAB//B////4AB////wP/wAAAP/wP///+AAf///4H/8AAAD/8B////gAP///8B/+AA' +
  'AAf/gP///4AD///+Af/gAAAH/4B////AA////AH/4AAAB/+AP///wAP///AD/+AAAAf/wA///8AD///AA//gAAAP/8AH///A' +
  'A///gAP/8AAAD//AAf//wAP//wAD//AAAA//wAD//8AH//4AB//wAAAP/+AAf//AB//8AAf/8AAAH//gAD//wAf/+AAH//AA' +
  'AB//4AAf/+AH//gAB//4AAAf/+AAH//gB//wAA//+AAAH//wAA//4Af/8AAP//gAAB//8AAP/+AH//AAH//4AAAf//gAD//g' +
  'B//gAB//+AAAH//4AA//4Af/4AA///gAAB///AAP/+AH/+AAP//4AAAf//wAD//gB//gAH//+AAAH//+AA//4Af/8AD///gA' +
  'AB///wAP/+AH//AA///4AAAf//8AD//gB//4A///+AAAH///wB//4Af/+Af///gAAB///+Af/+AH//4f///wAAAf///4f//g' +
  'B///////8AAAD///////4Af///////AAAA///////+AH///////wAAAP///////gB///////4AAAB///////4Af//////+AA' +
  'AAf//////+AH///////AAAAH///////AB///////wAAAA///////wAP//////4AAAAH//////8AD//////+AAAAB///////A' +
  'A///////AAAAAP//////wAP//////gAAAAB//////8AD//////wAAAAAP/////+AAf/////4AAAAAB//////gAH/////8AAA' +
  'AAAP/////4AB/////8AAAAAAA/////+AAf////8AAAAAAAD/////AAD////wAAAAAAAAH////wAA////gAAAAAAAAAH///8A' +
  'AH///AAAAAAAAAAAP//+AAA///AAAAAAAAAAAA///AAAP//AAAAAAAAAAAAD//wAAB//AAAAAAAAAAAAAP/4AAAP/AAAAAAA' +
  'AAAAAAA/4AAAAeAAAAAAAAAAAAAAB4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── bone — 비트맵 실루엣(128×128, 잉크 40.8%)
//   원본 마스크: bone.png
var MASK_BONE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAD/4AAAAAAAAAAD/4AAAAAH//gAAAAAAAAAH//wAA' +
  'AAH//+AAAAAAAAAH///AAAAH///4AAAAAAAAH///4AAAD////AAAAAAAAD////AAAB////4AAAAAAAB////8AAA/////AAAA' +
  'AAAA/////AAAf////4AAAAAAAf////4AAP/////AAAAAAAP/////AAD/////4AAAAAAD/////4AB/////+AAAAAAB/////+A' +
  'Af/////wAAAAAAf/////gAP/////8AAAAAAP/////8AD//////gAAAAAD//////AA//////4AAAAAB//////wAf/////+AAA' +
  'AAAf/////+AH//////wAAAAAP//////gB//////8AAAAAD//////4Af//////gAAAAB//////+AH//////8AAAAA///////g' +
  'B///////gAAAAf//////4Af//////+AAAAf//////+AH///////////////////gA///////////////////4AP/////////' +
  '/////////8AD///////////////////AAf//////////////////wAH//////////////////4AA//////////////////+A' +
  'AP//////////////////AAB//////////////////gAAP/////////////////wAAB/////////////////4AAAP////////' +
  '////////8AAAB////////////////+AAAAf////////////////gAAAD////////////////wAAAA////////////////8AA' +
  'AAH////////////////AAAAB////////////////wAAAA////////////////8AAAAP////////////////AAAAH////////' +
  '////////4AAAD/////////////////AAAB/////////////////4AAA//////////////////AAAf/////////////////4A' +
  'AH//////////////////AAD//////////////////wAB//////////////////+AAf//////////////////gAP/////////' +
  '/////////8AD///////////////////AA///////////////////wAf//////////////////+AH///////////////////g' +
  'B///////4AAAB///////4Af//////4AAAAH//////+AH//////8AAAAAf//////gB//////+AAAAAD//////4Af//////AAA' +
  'AAA//////+AH//////wAAAAAH//////gB//////4AAAAAB//////4AP/////+AAAAAAf/////8AD//////gAAAAAD//////A' +
  'A//////wAAAAAA//////wAH/////8AAAAAAH/////4AB/////+AAAAAAB/////+AAP/////gAAAAAAP/////AAD/////wAAA' +
  'AAAD/////wAAf////4AAAAAAAf////4AAD////8AAAAAAAD////8AAAf///+AAAAAAAAf///+AAAD////AAAAAAAAD////AA' +
  'AAf///gAAAAAAAAf///gAAAB///wAAAAAAAAB///wAAAAH//wAAAAAAAAAP//wAAAAAf/gAAAAAAAAAAf/gAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── arena — 비트맵 실루엣(128×128, 잉크 31.1%)
//   원본 마스크: arena.png
var MASK_ARENA = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAADAAAAAAAOAAAAAAAAAAAAA4AAA' +
  'AAADgAAAAAAAAAAAAOAAAAAAA4AAAAAAAAAAAAHgAAAAAAeAAAAAAAAAAAAB8AAAAAAHwAAAAAAAAAAAA/gAAAAAD+AAAAAA' +
  'AAAAAAf4AAAAAB/gAAAAAAAAHAAP/AAAAAA/8AAYAAAAABwAGfYAAAAAZ5gAOAAAAAAcADHjgAAAAceMADgAAAAAHABh4cAA' +
  'AAOHhgA4AAAAAB4AweDwAAAPB4MAfAAAAAA/AYHx/////4eBwPwAAAAAP4eB////////wOH8AAAAAH/v3/////////n3/gAA' +
  'AAB+f////////////j4AAAAA/n////////////4/AAAAAP5////////////+fwAAAAH///////////////+AAAABP///////' +
  '///////4gAAAAx//////////////+MAAAAYf//////////////hgAAAGH//////////////4YAAADB//////////////+DAA' +
  'AAg///////////////wQAAAY/////+AAAAf/////GAAAHf////gAAAAAH////7gAAD////+AAAAAAAH////8AAB////+AAAA' +
  'AAAAf////AAAf///+AAAAAAAAB////4AAP////AAAAAAAAAP////AAD////gAAAAAAAAB////wAB////4AAAAAAAAAf///+A' +
  'Af////AAAAAAAAAP////gAP////4AAAAAAAAH////8AH/////gAAAAAAAH/////gB//////wAAAAAA//////4Af//////+AA' +
  'AAf//////+AD///////////////////AA///////////////////wAP//////////////////8AB//////////////////+A' +
  'Af//////////////////gAD//////////////////wAAf/////////////////4AAH/////////////////8AAA/////////' +
  '/////////AAAH/////////////////gAAB/////////////////4AAAP////////////////8AAAD/////////////////AA' +
  'AA/////////////////wAAAP////////////////8AAAD/////////////////AAAA/////////////////wAAAP////////' +
  '////////8AAAH//////8Pnw///////gAAA///////D58P//////wAAAP//////w//D//////8AAAA//////8f/4//////8AA' +
  'AAH//////H/+P/////+AAAAAf/////x//j/////+AAAAAA/////8f/4/////8AAAAAAB/////H/+P////4AAAAAAAD////x/' +
  '/j////wAAAAAAAAB///8//8///+AAAAAAAAAAA/////////wAAAAAAAAAAAAD//////wAAAAAAAAAAAAAAAAP/wAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── pestbug — 비트맵 실루엣(128×128, 잉크 42.4%)
//   원본 마스크: pestbug.png
var MASK_PESTBUG = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAwAAAAAAAYAAAAAAAAAAAA/AAAAAAAfAAAAAAAAAAAAP8AAAAAAf4AAAAAAAAAAAD/gAAAAAf8AAAAAA' +
  'AAAAAA/+AAAAAP/AAAAAAAAAIAAD/wAAAAH/AAAAAAAAAHAAAP+AAAAH/gAAAAAAAAD4AAB/wAAAD/gAAAAAAAAB/AAAH+AA' +
  'AB/gAAAAAAAAA/4AAA/wAAAfwAAAAAAAAAf/AAAH8AAAP4AAAAAAAAAP/4AAA/gAAH8AAAAAAAAAH//AAAH8AAD+AAAAAAAA' +
  'AD//4AAA/gAA/AAAAAAAAAB///AAAH4AAfgAAAAAAAAA///4AAA/AAH4AAAAAAAAAP///AAAP4ED8AAAAAAAAAB///4AAB//' +
  '//AAAAAAAAAAP///AAAP///gAAAAAAAAAB///4AAD///4AAAAAAAAAAP///AAA///+AAAAAAAAAAB///4AAP///gAAAAAAAA' +
  'AAP///AAH///8AAAAAAAAAAB///4AB////AAAAAAAAAAAP///AA////4AAAAAAAAAAB///4Af////AAAAAAAAAAAP///AH//' +
  '//wAAAYAAAAAAB///4D////+AAAPAAAAAAAP///A/////gAAH4AAAAAAB///4P////4AAB+AAAAAAAP///H////+AAAfgAAA' +
  'AAAB/////////gAAPwAAAAAAAP////////4AAD8AAAAAAAB////////+AAA/AAAAAAAAP////////gAAPwAAAAAAAB//////' +
  '//4AAH8AAAAAAAAP///////+AAB+AAAAAAAAB////////wAAfgAAAAAAHAP///////+AAH4AAAAAAD4B////////wAB+AAAA' +
  'AAB+AP///////+AB/AAAAAAAfwB////////wA/wAAAAAAD8AP///////+B/8AAAAAAA/AB////////x/+AAAAAAAP4AP////' +
  '//////AAAAAAAB+AB//////////gAAAAAAAfgAP/////////gAAAAAAAH8AB/////////gAAAAAAAA/AAf////////gAAAAA' +
  'AAAP+AH////////gAAAAAAAAD/4B////////4AAAAAAAAAf/x////////+AAAAAAAAAH///////////gAAAAAAAAA///////' +
  '////4AAAAAAAAAD//////////+AAAAAAAAAAH//////////wAAAAAAAAAAP/////////8AAAAAAAAAAAf/////////AAAAAA' +
  'AAAAAD/////////4AAAAAAAAAAA//////////AAAAAAAAAAAP///////////wAAAAAAAAD////////////AAAAAAAAB/////' +
  '///////4AAAAAAAA/////////////AAAAAAAA/////////////wAAAAAH//////////////+AAAAAD////////////4A/wAA' +
  'AAB////////////+AH+AAAAA/////////////AA/wAAAAf////////////wAH+AAAAP////////////8AA/wAAAH/AD/////' +
  '/////AAH+AAAD/gA//////////wAA/wAAB/wAH/////////8AAH8AAA/4AB/////////8AAB/gAAf4AAf////////+AAAP4A' +
  'AP8AAH/////////gAAB8AAD+AAB/////////4AAAOAAA/AAAf/////////AAAAAAAPgAAH/////////4AAAAAABwAAB/////' +
  '/////AAAAAAAAAAAf/////////4AAAAAAAAAAP//////////AAAAAAAAAAP//////////4AAAAAAAAAH///////////AAAAA' +
  'AAAAH///////////4AAAAAAAAH////////////AAAAAAAAD////////////4AAAAAAAB/////////////gAAAAAAA/8f////' +
  '//////4AAAAAAAP8D///////////AAAAAAAD+A///////////4AAAAAAA/gP///////////AAAAAAAPwB///////////4AAA' +
  'AAAD8Af///////////AAAAAAA/AH///////////4AAAAAAPwA////////////AAAAAAD8AP///////////4AAAAAA/AB////' +
  '////////AAAAAAfwAf///////h///4AAAAAH8AD///////wP///AAAAAB/AA///////8B///4AAAAAfwAH//////+AP///AA' +
  'AAAH4AA///////gB///4AAAAB+AAP//////wAP///AAAAAfgAB//////4AB///4AAAAH4AAP/////8AAP///AAAAB+AAD///' +
  '///AAB///wAAAAfgAAf/////gAAP//4AAAAH4AAD/////wAAB//8AAAAD+AAAf////4AAAP/+AAAAA/gAAD////8AAAB//gA' +
  'AAAfwAAAf///+AAAAP/wAAAAH4AAAD////AAAAB/4AAAAA+AAAAP///AAAAAP8AAAAAHAAAAA///gAAAAB+AAAAAAAAAAAD/' +
  '/gAAAAAOAAAAAAAAAAAAP/AAAAAABAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── finger — 비트맵 실루엣(128×128, 잉크 27.4%)
//   원본 마스크: finger.png
var MASK_FINGER = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAB+AAAAAAAAAAAAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAf+AAAAAAAAAAAAAAAAAAAP/gAAAAAAA' +
  'AAAAAAAAAAAD/8AAAAAAAAAAAAAAAAAAA//AAAAAAAAAAAAAAAAAAAP/wAAAAAAAAAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAA' +
  'B//AAAAAAAAAAAAAAAAAAAf/wAAAAAAAAAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAAB//AAAAAAAAAAAAAAAAAAAf/wAAAAAAA' +
  'AAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAAB//AAAAAAAAAAAAAAAAAAAf/wAAAAAAAAAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAA' +
  'B//AAAAAAAAAAAAAAAAAAAf/wAAAAAAAAAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAAB//AAAAAAAAAAAAAAAAAAAf/wAAAAAAA' +
  'AAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAAB//AAAAAAAAAAAAAAAAAAAf/wAAAAAAAAAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAA' +
  'B//AAAAAAAAAAAAAAAAAAAf/wAAAAAAAAAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAAB//AAAAAAAAAAAAAAAAAAAf/wAAAAAAA' +
  'AAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAAB//AAAAAAAAAAAAAAAAAAAf/wAAAAAAAAAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAA' +
  'B//AAAAAAAAAAAAAAAAAAgf/wAAAAAAAAAAAAAAAAB/H/8AAAAAAAAAAAAAAAAB/5//AAAAAAAAAAAAAAAHgf///wAAAAAAA' +
  'AAAAAAAD+P///8AAAAAAAAAAAAAAB/z////AAAAAAAAAAAAAAA//////wAAAAAAAAAAAAB4f/////8AAAAAAAAAAAAB/H///' +
  '///AAAAAAAAAAAAA////////wAAAAAAAAAAAAf///////8AAAAAAAAAAAAH////////AAAAAAAAAAAAB////////wAAAAAAA' +
  'AAAAAf///////8AAAAAAAAAAAAH////////AAAAAAAAAAAAB////////wAAAAAAAAAAAAf///////8D/AAAAAAAAAAH/////' +
  '///B/4AAAAAAAAAB////////w//AAAAAAAAAAf///////+//wAAAAAAAAAH//////////8AAAAAAAAAB///////////AAAAA' +
  'AAAAAf//////////wAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAAAAAAAf//////////AAAAAAAAAAH/////' +
  '/////gAAAAAAAAAB//////////wAAAAAAAAAAf/////////4AAAAAAAAAAH/////////+AAAAAAAAAAB//////////AAAAAA' +
  'AAAAA//////////wAAAAAAAAAAP/////////4AAAAAAAAAAD/////////+AAAAAAAAAAA//////////AAAAAAAAAAAP/////' +
  '////wAAAAAAAAAAD/////////8AAAAAAAAAAA/////////+AAAAAAAAAAAP/////////gAAAAAAAAAAD/////////4AAAAAA' +
  'AAAAA/////////+AAAAAAAAAAAP/////////gAAAAAAAAAAD/////////4AAAAAAAAAAA/////////8AAAAAAAAAAAH/////' +
  '////AAAAAAAAAAAB/////////wAAAAAAAAAAAf////////4AAAAAAAAAAAH////////+AAAAAAAAAAAB/////////gAAAAAA' +
  'AAAAAf////////wAAAAAAAAAAAD////////8AAAAAAAAAAAA////////+AAAAAAAAAAAAP////////gAAAAAAAAAAAB/////' +
  '///wAAAAAAAAAAAAf///////4AAAAAAAAAAAAH///////+AAAAAAAAAAAAA////////AAAAAAAAAAAAAP///////gAAAAAAA' +
  'AAAAAB///////4AAAAAAAAAAAAAf//////8AAAAAAAAAAAAAD//////+AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAH////' +
  '//wAAAAAAAAAAAAAA//////8AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAD//////gAAAAAAAAAAAAAA//////4AAAAAAAA' +
  'AAAAAAH/////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB////' +
  '//gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////4AAAAAAAA' +
  'AAAAAAH/////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAP/////wAAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAAP///' +
  '/+AAAAAAAAAAAAAAAA////+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── like — 비트맵 실루엣(128×128, 잉크 44.4%)
//   원본 마스크: like.png
var MASK_LIKE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAB+AAAAAAAAAAAAAAAAAAAA/4AAAAAAAAAAAAAAAAAAAf/AAAAAAAAAAAAAAAAAAAH/4AAAAAAAA' +
  'AAAAAAAAAAB//AAAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAH/+AAAAAAAAAAAAAAAAAAB//wAAAAAAAAAAAAAAAAAA' +
  'f/8AAAAAAAAAAAAAAAAAAH//gAAAAAAAAAAAAAAAAAB//4AAAAAAAAAAAAAAAAAAf//AAAAAAAAAAAAAAAAAAH//wAAAAAAA' +
  'AAAAAAAAAAD//8AAAAAAAAAAAAAAAAAA///AAAAAAAAAAAAAAAAAAP//4AAAAAAAAAAAAAAAAAD//+AAAAAAAAAAAAAAAAAA' +
  '///gAAAAAAAAAAAAAAAAAP//4AAAAAAAAAAAAAAAAAH//+AAAAAAAAAAAAAAAAAB///gAAAAAAAAAAAAAAAAAf//4AAAAAAA' +
  'AAAAAAAAAAP//+AAAAAAAAAAAAAAAAAD///gAAAAAAAAAAAAAAAAB///4AAAAAAAAAAAAAAAAAf//+AAAAAAAAAAAAAAAAAP' +
  '///AAAAAAAAAAAAAAAAAD///wAAAAAAAAAAAAAAAAB///8AAAAAAAAAAAAAAAAAf///AAAAAAAAAAAAAAAAAP///gAAAAAAA' +
  'AAAAAAAAAH///4AAAAAAAAAAAAAAAAB///+AAAAAAAAAAAAAAAAA////AAAAAAAAAAAAAAAAAf///wAAAAAAAAAAAAAAAAP/' +
  '//8AAAAAAAAAAAAAAAAD///+AAAAAAAAAAAAAAAAB////gAAAAAAAAAAAAAAAA////4AAAAAAAAAAAAAAAAf///8AAAAAAAA' +
  'AAAAAAAAH////AAAAAAAAAAAAAAAAD////wAAAAAAAAAAAAAAAB////8AAAAAAAAAAAAAAAA/////AAAAAAAAAAAAAAAAP//' +
  '//wAAAAAAAAAAAAAAAH////8AAAAAAAAAAAAAAAD/////AAAAAAAAAAAAAAAA/////4AAAAAAAAAAAAAAAf/////AAAAAAAA' +
  'AAAAAAAP//////////4AAAAAAAAAD///////////AAAAAAAAAB///////////4AAAAAAAAAf///////////AAAAAAAAAP///' +
  '////////wAAAAAAAAH///////////+AAAAAAAAB////////////gAAAAf//A////////////4AAAAf//8f///////////+AA' +
  'AAP////////////////gAAAD////////////////wAAAB////////////////8AAAAf///////////////+AAAAH////////' +
  '////////AAAAB////////////////AAAAAf///////////////gAAAAH///////////////4AAAAB////////////////AAA' +
  'AAf///////////////8AAAAH////////////////gAAAB////////////////8AAAAf////////////////gAAAH////////' +
  '////////4AAAB////////////////+AAAAf////////////////gAAAH////////////////4AAAB////////////////+AA' +
  'AAf////////////////AAAAH////////////////wAAAB////////////////4AAAAf///////////////8AAAAH////////' +
  '///////8AAAAB///////////////+AAAAAf///////////////AAAAAH///////////////4AAAAB////////////////gAA' +
  'AAf///////////////8AAAAH////////////////gAAAB////////////////4AAAAf///////////////+AAAAH////////' +
  '////////gAAAB////////////////4AAAAf///////////////+AAAAH////////////////gAAAB////////////////4AA' +
  'AAf///////////////8AAAAH///////////////+AAAAB////////////////AAAAAf///////////////AAAAAH////////' +
  '//////+AAAAAB///////////////gAAAAAf//////////////4AAAAAH///////////////gAAAAB///////////////8AAA' +
  'AAf///////////////AAAAAH///////////////wAAAAB///////////////+AAAAAf///////////////gAAAAH////////' +
  '///////4AAAAB///////////////+AAAAAP///////////////AAAAAD///////////////wAAAAAf//8D//////////4AAA' +
  'AAB//8AH/////////8AAAAAAAAAAAf////////+AAAAAAAAAAAA/////x//gAAAAAAAAAAAAD////4AAAAAAAAAAAAAAAAH/' +
  '//8AAAAAAAAAAAAAAAAAD//wAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── victory — 비트맵 실루엣(128×128, 잉크 29.8%)
//   원본 마스크: victory.png
var MASK_VICTORY = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAPgAAAAAAAAAAAAAH4AAAAP8AAAAAAAAAAAAAD/gAAAH/gAAAAAAAAAAAAB/4AAAB/8AAAAAAA' +
  'AAAAAA//AAAA//AAAAAAAAAAAAAP/wAAAP/wAAAAAAAAAAAAD/+AAAD/+AAAAAAAAAAAAA//gAAB//AAAAAAAAAAAAAP/4AA' +
  'Af/wAAAAAAAAAAAAD/+AAAH/8AAAAAAAAAAAAA//wAAB//AAAAAAAAAAAAAP/8AAA//gAAAAAAAAAAAAD//AAAP/4AAAAAAA' +
  'AAAAAAf/4AAD/+AAAAAAAAAAAAAH/+AAB//gAAAAAAAAAAAAB//gAAf/4AAAAAAAAAAAAAf/4AAH/8AAAAAAAAAAAAAD//AA' +
  'B//AAAAAAAAAAAAAA//wAA//wAAAAAAAAAAAAAP/8AAP/8AAAAAAAAAAAAAD//AAD/+AAAAAAAAAAAAAAf/4AB//gAAAAAAA' +
  'AAAAAAH/+AAf/4AAAAAAAAAAAAAB//gAH/+AAAAAAAAAAAAAAf/4AB//AAAAAAAAAAAAAAH//AA//wAAAAAAAAAAAAAA//wA' +
  'P/8AAAAAAAAAAAAAAP/8AD//AAAAAAAAAAAAAAD//AA//gAAAAAAAAAAAAAA//4Af/4AAAAAAAAAAAAAAH/+AH/+AAAAAAAA' +
  'AAAAAAB//gB//AAAAAAAAAAAAAAAf/4A//wAAAAAAAAAAAAAAH//AP/8AAAAAAAAAAAAAAA//wD//AAAAAAAAAAAAAAAP/8B' +
  '//gAAAAAAAAAAAAAAD//gf/4AAAAAAAAAAAAAAA//8P/+AAAAAAAAAAAAAAAP/////gAAAAAAAAAAAAAAB/////wAAAAAAAA' +
  'AAAAAAAf////+AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAB/////4AAAAAAAAAAAAAAAP/////gAAAAAAAAAAAAAAD///' +
  '///gAAAAAAAAAAAAAA//////+AAAAAAAAAAAAAAP//////wAAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAf//////gAAAAAA' +
  'AAAAAAAH//////4AAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAf//////wAAAAAAAAAAAAAH//////+AAAAAAAAAAAAAB///' +
  '////8AAAAAAAAAAAAAf///////wAAAAAAAAAAAAH///////+AAAAAAAAAAAAB////////wAAAAAAAAAAAA////////8AAAAA' +
  'AAAAAAAf////////AAAAAAAAAAAAf////////wAAAAAAAAAAAf////////8AAAAAAAAAAAP/////////AAAAAAAAAAAH////' +
  '/////gAAAAAAAAAAD/////////4AAAAAAAAAAB/////////8AAAAAAAAAAA//////////AAAAAAAAAAAP/////////gAAAAA' +
  'AAAAAH/////////4AAAAAAAAAAB/////////+AAAAAAAAAAA//////////wAAAAAAAAAAP/////////8AAAAAAAAAAD/////' +
  '/////AAAAAAAAAAA//////////wAAAAAAAAAAP/////////8AAAAAAAAAAD//////////AAAAAAAAAAA//////////wAAAAA' +
  'AAAAAP/////////8AAAAAAAAAAD//////////AAAAAAAAAAA//////////gAAAAAAAAAAH/////////4AAAAAAAAAAB/////' +
  '////+AAAAAAAAAAAf/////////gAAAAAAAAAAH/////////4AAAAAAAAAAA/////////+AAAAAAAAAAAP/////////AAAAAA' +
  'AAAAAD/////////wAAAAAAAAAAAf////////8AAAAAAAAAAAH////////+AAAAAAAAAAAA/////////gAAAAAAAAAAAP////' +
  '////4AAAAAAAAAAAB////////8AAAAAAAAAAAAf////////AAAAAAAAAAAAD////////gAAAAAAAAAAAA////////4AAAAAA' +
  'AAAAAAH///////8AAAAAAAAAAAAA////////AAAAAAAAAAAAAP///////gAAAAAAAAAAAAB///////4AAAAAAAAAAAAAf///' +
  '///8AAAAAAAAAAAAAD///////AAAAAAAAAAAAAA///////gAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAB//////8AAAAAAA' +
  'AAAAAAAf//////AAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAP/////+AAAAAAAAAAAAAAD///' +
  '///gAAAAAAAAAAAAAA//////4AAAAAAAAAAAAAAP/////+AAAAAAAAAAAAAAD//////gAAAAAAAAAAAAAA//////4AAAAAAA' +
  'AAAAAAAH/////+AAAAAAAAAAAAAAB//////AAAAAAAAAAAAAAAP/////wAAAAAAAAAAAAAAB/////wAAAAAAAAAAAAAAAD//' +
  '//wAAAAAAAAAAAAAAAAB//+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── oksign — 비트맵 실루엣(128×128, 잉크 31.5%)
//   원본 마스크: oksign.png
var MASK_OKSIGN = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAA+AAAAAAAAAAAAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAP+AAAAAAAAAAAAAAAAA8AD/gAAAAAAAA' +
  'AAAAAAAAfwB/8AAAAAAAAAAAAAAAAP8Af/AAAAAAAAAAAAAAAAD/gP/wAAAAAAAAAAAAAAAB/4D/8AAAAAAAAAAAAAAAAf/B' +
  '/+AAAAAAAAAAAAAAAAH////gAAAAAAAAAAAAAAAB////4AAAAAAAAAAAAAAAA////+AAAAAAAAAAAAAAAAP////AAAAAAAAA' +
  'AAAAAAAD////wAAAAAAAAAAAAAAAA////4AAAAAAAAAAAAAAAAf///+AAAAAAAAAAAAAAAAH////AAAAAAAAAAAAAAAAB///' +
  '/wAAAAAAAAAAAAAB8A////8AAAAAAAAAAAAAA/gP///+AAAAAAAAAAAAAAf4H////gAAAAAAAAAAAAAH/B////wAAAAAAAAA' +
  'AAAAB//////8AAAAAAAAAAAAAA//////+AAAAAAAAAAAAAAP//////gAAAAAAAAAAAAAD//////4AAAAAAAAAAAAAA//////' +
  '8AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAD//////gAAAAAAAAAAAAAA//////4AAAAAAAAAAAAAAP/////8AAAAAAAAAA' +
  'AAAAD//////AAAAAAAAAAAAAAA//////wAAAAAAAAAAAAAAP/////4AAAAAAAAAAAAAAD//////AAAAAAAAAAAAAAA//////' +
  'wAAAAAAAAAAAAAAP/////+D4AAAAAAAAAAAAD////////gAAAAAAAAAAAA////////+AAAAAAAAAAAAP////////4AAAAAAA' +
  'AAAAD/////////gAAAAAAAAAAA/////////8AAAAAAAAAAAP/////////wAAAAAAAAAAD/////////+AAAAAAAAAAA//////' +
  '////wAAAAAAAAAAP/////////+AAAAAAAAAAD//////////wAAAAAAAAAA//////////+AAAAAAAAAAP//////////gAAAAA' +
  'AAAAB//////////8AAAAAAAAAAf//////4P//AAAAAAAAAAH//////4Af/4AAAAAAAAAB//////8AB/+AAAAAAAAAAf/////' +
  '+AAP/gAAAAAAAAAH//////AAB/8AAAAAAAAAB//////AAAP/AAAAAAAAAAf/////wAAB/4AAAAAAAAAH/////4AAAH/gAAAA' +
  'AAAAB/////8AAAB/4AAAAAAAAAf/////AAAAf+AAAAAAAAAH/////wAAAH/gAAAAAAAAB/////4AAAB/4AAAAAAAAAf////+' +
  'AAAA/+AAAAAAAAAH/////gAAAP/gAAAAAAAAB/////4AAAD/4AAAAAAAAAf////+AAAB/+AAAAAAAAAH/////wAAAf/AAAAA' +
  'AAAAB/////8AAAH/wAAAAAAAAAf/////AAAB/8AAAAAAAAAH/////4AAA//AAAAAAAAAB//////AAAf/wAAAAAAAAAf/////' +
  '4AAP/8AAAAAAAAAD//////AAH//AAAAAAAAAA//////4AD//wAAAAAAAAAP//////AD//8AAAAAAAAAD//////8B///AAAAA' +
  'AAAAA///////////gAAAAAAAAAP//////////4AAAAAAAAAD//////////8AAAAAAAAAA//////////+AAAAAAAAAAP/////' +
  '/////gAAAAAAAAAD//////////wAAAAAAAAAA//////////4AAAAAAAAAAP/////////8AAAAAAAAAAB/////////+AAAAAA' +
  'AAAAAf/////////gAAAAAAAAAAH/////////wAAAAAAAAAAB/////////4AAAAAAAAAAAP////////8AAAAAAAAAAAD/////' +
  '///+AAAAAAAAAAAA/////////gAAAAAAAAAAAH////////wAAAAAAAAAAAB////////4AAAAAAAAAAAAf///////8AAAAAAA' +
  'AAAAAD///////+AAAAAAAAAAAAA////////AAAAAAAAAAAAAP///////gAAAAAAAAAAAAB///////wAAAAAAAAAAAAAf////' +
  '//4AAAAAAAAAAAAAH//////8AAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAD//////gAAAAAAAA' +
  'AAAAAA//////wAAAAAAAAAAAAAAP/////4AAAAAAAAAAAAAAD/////+AAAAAAAAAAAAAAA//////AAAAAAAAAAAAAAAP////' +
  '/gAAAAAAAAAAAAAAD/////4AAAAAAAAAAAAAAA/////+AAAAAAAAAAAAAAAP/////gAAAAAAAAAAAAAAD/////4AAAAAAAAA' +
  'AAAAAAf////8AAAAAAAAAAAAAAAH/////AAAAAAAAAAAAAAAB/////wAAAAAAAAAAAAAAAP////4AAAAAAAAAAAAAAAA////' +
  '8AAAAAAAAAAAAAAAAA///wAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── lovesign — 비트맵 실루엣(128×128, 잉크 33.0%)
//   원본 마스크: lovesign.png
var MASK_LOVESIGN = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAHgAAAAAAAAAAAAAAAAAAAH+AAAAAAAAAAAAAAAAAAAD/wAAAAAAAAAAAAAAAAAAB/8AAAAAAAAAAAA' +
  'AAAAAAAf/gAAAAAAAAAAAAAAAAAAP/4AAAAAAAAAAAAAAAAAAD/+AAAAAAAAAAAAAAAAAAA//wAAAAAAAAAAAAAAAAAAP/8A' +
  'AAAAAAAAAAAAAAAAAD//AAAAAAAAAAAAAAAAAAA//wAAAAAAAAAAAAAAAAAAP/8AAAAAAAAAAAAAAAAAAD//AAAAAAAAAAAA' +
  'AAAAAAAf/wAAAAAAAAAAAAAAAAAAH/+AAAAAAAPwAAAAAAAAAB//gAAAAAAH+AAAAAAAAAAf/4AAAAAAD/wAAAAAAAAAH/+A' +
  'AAAAAA/8AAAAAAAAAB//gAAAAAAf/gAAAAAAAAAf/4AAAAAAH/4AAAAAAAAAH/+AAAAAAB/+AAAAAAAAAA//wAAAAAAf/gAA' +
  'AAAAAAAP/8AAAAAAP/wAAAAAAAAAD//AAAAAAD/8AAAAAAAAAA//wAAAAAA//AAAAAAAAAAP/8AAAAAAP/wAAAAAAAAAD//A' +
  'AAAAAH/8AAAAAAAAAA//wAAAAAB/+AAAAAAAAAAH/+AAAAAAf/gAAAAAAAAAB//gAAAAAP/4AAAAAAAAAAf/4AAAAAD/+AAA' +
  'AAAAAAAH/+AAAAAA//gAAAAAAAAAB//gAAAAAP/wAAAAAAAAAAf/4AAAAAH/8AAAAAAAAAAH/+AAAAAB//AAAAAAAAAAB//w' +
  'AAAAAf/wAAAAAAAAAAP/8AAAAAP/8AAAAAAAAAAD//g+AAAD/+AAAAAAAAAAA////4D8A//gAAAAAAAAAAP////B/wf/4AAA' +
  'AAAAAAAD/////////+AAAAAAAAAAA//////////gAAAAAAAAAAP/////////wAAAAAAAAAAB/////////8AAAAAAAAAAAf//' +
  '///////AAAAAAAAAAAH/////////wAAAAAAAAAAB/////////4AAAAAAAAAAAf////////+AAAAAAAAAAAH/////////gAAA' +
  'AAAAAAAB/////////4AAAAAAAAAAAP////////+AAAAAAAAAAAD/////////AAAAAAAAAAAA/////////wAAAAAAAAAAAP//' +
  '//////8AAAAAAAP8AAD/////////AAAAAAAP/4AA/////////wAAAAAAH//AAP////////8AAAAAAD//8AD/////////AAAA' +
  'AAA///wA/////////wAAAAAAf//+AP////////8AAAAAAH///wD////////+AAAAAAA///+B/////////gAAAAAAH///w///' +
  '//////4AAAAAAA/////////////+AAAAAAAD/////////////gAAAAAAAP////////////4AAAAAAAB////////////+AAAA' +
  'AAAAH////////////gAAAAAAAA////////////4AAAAAAAAH///////////+AAAAAAAAA////////////gAAAAAAAAP/////' +
  '//////4AAAAAAAAB///////////+AAAAAAAAAP///////////gAAAAAAAAB///////////4AAAAAAAAAf//////////8AAAA' +
  'AAAAAD///////////AAAAAAAAAA///////////wAAAAAAAAAH//////////8AAAAAAAAAB///////////AAAAAAAAAAP////' +
  '//////wAAAAAAAAAB//////////8AAAAAAAAAAf/////////+AAAAAAAAAAD//////////gAAAAAAAAAAf/////////4AAAA' +
  'AAAAAAH/////////+AAAAAAAAAAA//////////AAAAAAAAAAAH/////////wAAAAAAAAAAA/////////8AAAAAAAAAAAH///' +
  '/////+AAAAAAAAAAAB/////////gAAAAAAAAAAAP////////4AAAAAAAAAAAA////////8AAAAAAAAAAAAH////////AAAAA' +
  'AAAAAAAB////////wAAAAAAAAAAAAP///////4AAAAAAAAAAAAB///////+AAAAAAAAAAAAAP///////AAAAAAAAAAAAAB//' +
  '/////wAAAAAAAAAAAAAf//////4AAAAAAAAAAAAAD//////+AAAAAAAAAAAAAA///////gAAAAAAAAAAAAAH//////wAAAAA' +
  'AAAAAAAAB//////8AAAAAAAAAAAAAAf//////AAAAAAAAAAAAAAH//////gAAAAAAAAAAAAAB//////4AAAAAAAAAAAAAAP/' +
  '////+AAAAAAAAAAAAAAD//////gAAAAAAAAAAAAAA//////4AAAAAAAAAAAAAAP/////+AAAAAAAAAAAAAAD//////gAAAAA' +
  'AAAAAAAAA//////4AAAAAAAAAAAAAAH/////8AAAAAAAAAAAAAAB/////+AAAAAAAAAAAAAAAH/////AAAAAAAAAAAAAAAAf' +
  '///+AAAAAAAAAAAAAAAAAf//4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── prayer — 비트맵 실루엣(128×128, 잉크 27.6%)
//   원본 마스크: prayer.png
var MASK_PRAYER = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAeB4AAAAAAAAAAAAAAAAAAP4/AAAAAAAAAAAAAAAAAAD+fwAAAAAAAAAAAAAAAAAA/n8AAAAAAAAA' +
  'AAAAAAAAAP5/AAAAAAAAAAAAAAAAAAH+f4AAAAAAAAAAAAAAAAAB/n+AAAAAAAAAAAAAAAAAAf5/gAAAAAAAAAAAAAAAAAH+' +
  'f4AAAAAAAAAAAAAAAAAB/n+AAAAAAAAAAAAAAAAAAf5/gAAAAAAAAAAAAAAAAAP+f8AAAAAAAAAAAAAAAAAD/n/AAAAAAAAA' +
  'AAAAAAAAA/5/wAAAAAAAAAAAAAAAAAP+f8AAAAAAAAAAAAAAAAAD/n/AAAAAAAAAAAAAAAAAB/5/wAAAAAAAAAAAAAAAAAf+' +
  'f+AAAAAAAAAAAAAAAAAH/n/gAAAAAAAAAAAAAAAAB/5/4AAAAAAAAAAAAAAAAAf+f+AAAAAAAAAAAAAAAAAH/n/gAAAAAAAA' +
  'AAAAAAAAD/5/8AAAAAAAAAAAAAAAAA/+f/AAAAAAAAAAAAAAAAAP/n/wAAAAAAAAAAAAAAAAD/5/8AAAAAAAAAAAAAAAAA/+' +
  'f/AAAAAAAAAAAAAAAAAf/n/wAAAAAAAAAAAAAAAAH/5/+AAAAAAAAAAAAAAAAB/+f/gAAAAAAAAAAAAAAAAf/n/4AAAAAAAA' +
  'AAAAAAAAH/5/+AAAAAAAAAAAAAAAAD/+f/wAAAAAAAAAAAAAAAA//n/8AAAAAAAAAAAAAAAAP/5//AAAAAAAAAAAAAAAAD/+' +
  'f/wAAAAAAAAAAAAAAAA//n/+AAAAAAAAAAAAAAAAf/5//gAAAAAAAAAAAAAAAH/+f/4AAAAAAAAAAAAAAAD//n/+AAAAAAAA' +
  'AAAAAAAA/4JB/wAAAAAAAAAAAAAAAP84PP8AAAAAAAAAAAAAAAH/fH7/gAAAAAAAAAAAAAAB/v5/f4AAAAAAAAAAAAAAA/7+' +
  'f3/AAAAAAAAAAAAAAAP+/n9/wAAAAAAAAAAAAAAD/f5/v8AAAAAAAAAAAAAAB/3+f7/gAAAAAAAAAAAAAAf9/n+/4AAAAAAA' +
  'AAAAAAAH/f5/v/AAAAAAAAAAAAAAD//+f//wAAAAAAAAAAAAAA/7/n/f8AAAAAAAAAAAAAAP//5///AAAAAAAAAAAAAAD/v+' +
  'f9/wAAAAAAAAAAAAAA/7/n/f8AAAAAAAAAAAAAAf//5/3/gAAAAAAAAAAAAAH/f+f+/4AAAAAAAAAAAAAB/3/n/v+AAAAAAA' +
  'AAAAAAAf9/5/7/gAAAAAAAAAAAAAH/f+f+/4AAAAAAAAAAAAAB///n/v+AAAAAAAAAAAAAAf//5///gAAAAAAAAAAAAAH//+' +
  'f//4AAAAAAAAAAAAAB///n//+AAAAAAAAAAAAAAf//5///gAAAAAAAAAAAAAH//+f//8AAAAAAAAAAAAAD///n///AAAAAAA' +
  'AAAAAAA///5///wAAAAAAAAAAAAAP//+f//8AAAAAAAAAAAAAD///n///AAAAAAAAAAAAAA///5///4AAAAAAAAAAAAAf//+' +
  'f//+AAAAAAAAAAAAAH///n///gAAAAAAAAAAAAB///5///4AAAAAAAAAAAAAf//+f///AAAAAAAAAAAAAP///n///wAAAAAA' +
  'AAAAAAD///5///8AAAAAAAAAAAAB///+f///gAAAAAAAAAAAAf///n///8AAAAAAAAAAAAP///5////AAAAAAAAAAAAH///+' +
  'f///4AAAAAAAAAAAD////n////AAAAAAAAAAAB////5////4AAAAAAAAAAA////+f////AAAAAAAAAAAf////n////4AAAAA' +
  'AAAAAf////5/////gAAAAAAAAAP////+f////8AAAAAAAAAH/////n/////gAAAAAAAAH/////5/////+AAAAAAAAD/////8' +
  'P/////wAAAAAAAB//////D//////AAAAAAAB//////w//////4AAAAAAB//////4H//////gAAAAAA//////+B//////8AAA' +
  'AAA///////AP//////wAAAAAf//////wD//////+AAAAAH//////4Af//////gAAAAD//////8AD//////4AAAAAf/////+A' +
  'Af/////+AAAAAH//////AAD//////gAAAAA//////gAAf/////wAAAAAH/////wAAD/////4AAAAAB/////wAAAP////+AAA' +
  'AAAP////4AAAB/////AAAAAAB////8AAAAP////gAAAAAAP///+AAAAB////wAAAAAAB////AAAAAP///8AAAAAAAP///gAA' +
  'AAB///+AAAAAAAD///wAAAAAP///AAAAAAAAf//4AAAAAB///gAAAAAAAD//8AAAAAAP//wAAAAAAAAf/+AAAAAAB//4AAAA' +
  'AAAAD//AAAAAAAP/8AAAAAAAAAf/gAAAAAAB/+AAAAAAAAAD/wAAAAAAAP/AAAAAAAAAAf4AAAAAAAB/gAAAAAAAAAD8AAAA' +
  'AAAAPwAAAAAAAAAAeAAAAAAAAB4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── fist — 비트맵 실루엣(128×128, 잉크 57.2%)
//   원본 마스크: fist.png
var MASK_FIST = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAf4AAAAAAAAAAAAAAAA/gA//gAAAAAAAAAAAAAAB//Af/8AAAAAAAAAAAAAAA//4P//gB8AAAAA' +
  'AAAAAwAf//H//8D/8AAAAAAAAB/wP//5///B//wAAAAAAAB/+D//////4//+AAAAAAAB//w//////+f//wAAAAAAA///////' +
  '//////8AAAAAAAf/////////////gAAAAAAH/////////////4AAAAAAD/////////////+AAAAAAA//////////////wAAA' +
  'AAAP/////////////8AAAAAAD//////////////AAAAAAB//////////////wAAAAAAP/////////////4AAAAAAD///////' +
  '//////+AAAAAAA//////////////gAAAAAAP/////////////4AAAAAAD/////////////+AAAAAAA//////////////gAAA' +
  'AAAP/////////////4AAAAAAD/////////////+AAAAAAA//////////////gAAAAAAP/////////////4AAAAAAD///////' +
  '//////+AAAAAAA//////////////gAAAAAAP/////////////4AAAAAAD/////////////+AAAAAAB//////////////gAAA' +
  'AAA//////////////4AAAAAAf/////////////+AAAAAAH//////////////gAAAAAB//////////////4AAAAAA////////' +
  '//////+AAAAAAP//////////////AAAAAAD//////////////wAAAAAA//////////////+AAAAAAf//////////////4AAA' +
  'AAH///////////////gAAAAB///////////////8AAAAAf///////////////AAAAAH///////////////4AAAAB////////' +
  '////////AAAAAf///////////////wAAAAP///////////////+AAAAD////////////////gAAAA////////////////4AA' +
  'AAP///////////////+AAAAD////////////////wAAAA////////////////8AAAAP////////////////AAAAD////////' +
  '////////wAAAA////////////////8AAAAP////////////////AAAAB////////////////wAAAAf///////////////4AA' +
  'AAH///////////////+AAAAB////////////////gAAAAf///////////////4AAAAH///////////////+AAAAB////////' +
  '////////AAAAAf///////////////wAAAAD///////////////8AAAAA////////////////AAAAAP///////////////gAA' +
  'AAD///////////////4AAAAAf//////////////8AAAAAH///////////////AAAAAB///////////////wAAAAAP///////' +
  '///////4AAAAAD//////////////+AAAAAA///////////////AAAAAAH//////////////wAAAAAB//////////////4AAA' +
  'AAAP/////////////8AAAAAAD//////////////AAAAAAAf/////////////gAAAAAAH/////////////wAAAAAAA///////' +
  '//////8AAAAAAAH////////////+AAAAAAAB/////////////AAAAAAAAP////////////gAAAAAAAD////////////4AAAA' +
  'AAAAf///////////8AAAAAAAAD///////////+AAAAAAAAA////////////gAAAAAAAAH///////////wAAAAAAAAB//////' +
  '/////4AAAAAAAAAP//////////+AAAAAAAAAD///////////gAAAAAAAAAf//////////wAAAAAAAAAH//////////8AAAAA' +
  'AAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAAAAAAAP/////' +
  '/////gAAAAAAAAAD//////////4AAAAAAAAAA//////////+AAAAAAAAAAP//////////gAAAAAAAAAD//////////4AAAAA' +
  'AAAAA//////////+AAAAAAAAAAP//////////gAAAAAAAAAD//////////4AAAAAAAAAA//////////8AAAAAAAAAAP/////' +
  '/////AAAAAAAAAAD//////////wAAAAAAAAAAf/////////4AAAAAAAAAAH/////////+AAAAAAAAAAA//////////AAAAAA' +
  'AAAAAH/////////gAAAAAAAAAAAf////////wAAAAAAAAAAAB////////wAAAAAAAAAAAAH///////gAAAAAAAAAAAAAD///' +
  '//8AAAAAAAAAAAAAAAA////AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

/* ── 내부 백선(골) — 실루엣 안쪽을 분절하는 문법 (2026-09-02, B4) ─────────────
 *   장미(`pointInRose` ②)가 로그나선 골로 꽃잎을 갈랐던 그 기법을 **일반화**한 것이다.
 *   왜 필요한가: B1 인지 게이트 실측(`~/wiacode-shape-lab/recog/BASELINE_2026-09-01.json`)에서
 *   손 계열은 **세운 손가락은 이미 갈라져 있는데 접힌 손가락(주먹·손바닥 덩어리)이 통짜**라
 *   판정자가 "hand"·"shield"·"bear"·"rabbit"·"kidney" 로 읽었다. 덩어리에 접힌 마디선을
 *   그어 주면 제스처가 읽힌다 — 그림을 다시 그리지 않고(=B2-b ⛔ 우회) 코드로 해결한다.
 *
 *   ★설계 제약 세 가지 (지키지 않으면 다른 게 깨진다)
 *   ① **골은 실루엣 경계에 닿지 않는다.** 밀착 풀이의 `_filledOutline` 은 바깥에서 flood fill
 *      하므로 안쪽 백선은 무시하지만, 골이 경계까지 열리면 채움이 새어 들어와 **윤곽이 달라지고
 *      기존 HUG_PINS 가 무효가 된다.** 그래서 끝을 경계에서 띄운다(닫힌 골).
 *   ② **폭은 3셀 이상.** 규격 v1.1 속살형 조항. 격자 L 에서 1셀 = 2/128 = 0.015625(정규화)
 *      이므로 반폭 0.024 ≈ 3.1셀. 이보다 얇으면 축소본에서 뭉개져 아무 효과가 없다.
 *   ③ **중앙 예약 근처를 피한다.** 코어·궤도는 골 위에 덮여 그려지지만, 데이터 셀을 잃는
 *      자리를 굳이 겹칠 이유가 없다.
 *   ★골은 데이터 셀을 먹는다 — 넣거나 고치면 용량표(`shape-registry.js` qrTable)를 다시 뽑을 것. */
var GROOVE_HW = 0.024;          // 골 반폭(정규화) ≈ 3.1셀
var SHAPE_GROOVES = {
  /* fist(주먹) — B1 실측 최악(카탈로그 1/2 "shield", 디테일 1/2 "bear"). 위 경계에 주먹뼈
   *   융기 4개는 있는데 아래가 통짜라 방패로 읽힌다. 융기 사이 골(x≈-0.40 / -0.03 / +0.35,
   *   경계 프로파일 실측)에서 아래로 손가락 분할선을 긋고, 접힌 마디선 1개와 엄지선 1개를 더한다. */
  fist: [
    { pts: [[-0.40, -0.72], [-0.42, -0.30], [-0.44, -0.02]] },   // 손가락 분할 ①
    { pts: [[-0.03, -0.76], [-0.04, -0.32], [-0.05, -0.02]] },   // 손가락 분할 ②
    { pts: [[ 0.35, -0.76], [ 0.37, -0.32], [ 0.39, -0.02]] },   // 손가락 분할 ③
    { pts: [[-0.46,  0.06], [ 0.00,  0.10], [ 0.42,  0.04]] },   // 접힌 마디선(가로)
    { pts: [[-0.48,  0.34], [-0.05,  0.44], [ 0.30,  0.40]] },   // 엄지가 앞을 가로지르는 선
    /* ★오너 지시(2026-09-03) — 오른쪽 위 기둥이 엄지인데 구분이 없어 손가락과 붙어 보였다.
     *   그 자리에 얕은 아크를 그어 엄지 마디를 낸다(실측: 그 구간 잉크는 x 0.30~0.64). */
    { pts: [[ 0.34, -0.46], [ 0.48, -0.53], [ 0.61, -0.45]] },   // 엄지 마디 아크
  ],
  /* lovesign(사랑해요) — 디테일 판정 0/2, 둘 다 그냥 "hand". 검지·새끼는 이미 갈라져 있고
   *   그 사이 **접힌 중지·약지**가 손바닥 덩어리에 묻혀 있다. 둘을 가르고 마디선을 얹는다. */
  lovesign: [
    { pts: [[ 0.00, -0.20], [ 0.01,  0.02], [ 0.02,  0.16]] },   // 접힌 중지/약지 사이
    { pts: [[ 0.22, -0.20], [ 0.23,  0.02], [ 0.24,  0.16]] },   // 접힌 약지/새끼 사이
    { pts: [[-0.28,  0.26], [ 0.10,  0.32], [ 0.44,  0.24]] },   // 접힌 마디선(가로)
  ],
  /* victory(브이) — 디테일 판정에서 "rabbit". V 만 보이고 접힌 약지·새끼가 안 읽힌다.
   *   덩어리 오른쪽(접힌 손가락 쪽)에 분할선을, 아래에 마디선을 넣는다. */
  victory: [
    { pts: [[ 0.24, -0.02], [ 0.26,  0.14], [ 0.27,  0.30]] },   // 접힌 약지/새끼 사이
    /* ★x 0.50 은 실루엣 밖이었다(골 커버리지 0% 로 검출) — 잉크는 x≈0.44 까지다. */
    { pts: [[ 0.38,  0.02], [ 0.39,  0.16], [ 0.40,  0.28]] },   // 새끼 쪽
    { pts: [[-0.26,  0.44], [ 0.10,  0.50], [ 0.40,  0.42]] },   // 접힌 마디선(가로)
  ],
  /* oksign(오케이) — 디테일 판정에서 "kidney". O 는 읽히는데 손이 안 읽힌다.
   *   O 근처는 건드리지 않고(정체성), 손바닥 덩어리에만 마디선을 넣는다. */
  oksign: [
    /* ★첫 안은 x -0.62 / 마디선 좌단 -0.68 이 실루엣 밖이었다(커버리지 0%·76% 로 검출).
     *   실측 잉크 범위는 x -0.50~-0.10 이다. 눈으로 보지 말고 커버리지로 확인할 것. */
    { pts: [[-0.42, -0.28], [-0.41, -0.06], [-0.40,  0.12]] },   // 접힌 손가락 분할 ①
    { pts: [[-0.26, -0.28], [-0.25, -0.06], [-0.24,  0.12]] },   // 접힌 손가락 분할 ②
    { pts: [[-0.44,  0.28], [-0.28,  0.32], [-0.14,  0.30]] },   // 접힌 마디선(가로)
  ],
  /* finger(검지) — 디테일 판정에서 "hand". 세운 검지 말고는 구분이 없다.
   *   접힌 중지·약지·새끼를 가르고 마디선을 얹는다. */
  finger: [
    { pts: [[-0.22, -0.06], [-0.21,  0.10], [-0.20,  0.24]] },   // 접힌 손가락 분할 ①
    { pts: [[-0.04, -0.06], [-0.03,  0.10], [-0.02,  0.24]] },   // 접힌 손가락 분할 ②
    { pts: [[-0.30,  0.36], [ 0.04,  0.42], [ 0.30,  0.36]] },   // 접힌 마디선(가로)
  ],
};
function _segDist2(px, py, ax, ay, bx, by) {
  var vx = bx - ax, vy = by - ay, wx = px - ax, wy = py - ay;
  var L2 = vx * vx + vy * vy;
  var t = L2 > 0 ? (wx * vx + wy * vy) / L2 : 0;
  if (t < 0) t = 0; else if (t > 1) t = 1;
  var dx = px - (ax + t * vx), dy = py - (ay + t * vy);
  return dx * dx + dy * dy;
}
function inGroove(list, nx, ny) {
  for (var i = 0; i < list.length; i++) {
    var g = list[i], pts = g.pts, hw = g.hw || GROOVE_HW, hw2 = hw * hw;
    for (var j = 0; j + 1 < pts.length; j++)
      if (_segDist2(nx, ny, pts[j][0], pts[j][1], pts[j + 1][0], pts[j + 1][1]) < hw2) return true;
  }
  return false;
}
/* 실루엣 판정 = 본체(아래) − 내부 골. 골이 없는 형상은 픽셀까지 예전 그대로다. */
// ── badge — 비트맵 실루엣(128×128, 잉크 56.4%)
//   원본 마스크: badge.png
var MASK_BADGE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAACAA/AAAAAAAAAAAAAAAAAPwAf+AAAAAAAAAAAAAAAAf+AH/4AAAAAAAAAAAAAAAf/g' +
  'B//wAAAAAAAAAAAAAA//4Af//gAAAAAAAAAAAAB//+AH///AAAAAAAAAAAAD///gA///+AAAAAAAAAAAH///wAP///8AAAAA' +
  'AAAAAP///8AD////wAAAAAAAAAf////AA/////wAAAAAAAA/////wAH/////AAAAAAAA/////4AB/////+AAP/wAB/////+A' +
  'Af/////8Af//gD//////gAH//////wP//8D//////4AA///////f//////////8AAP//////////////////AAD/////////' +
  '/////////wAA//////////////////8AAH//////////////////AAB//////////////////wAAf/////////////////4A' +
  'AH/////////////////+AAB//////////////////gAAf/////////////////4AAD/////////////////8AAA/////////' +
  '/////////AAAP/////////////////wAAD/////////////////8AAB//////////////////gAAf/////////////////4A' +
  'AH/////////////////+AAB//////////////////gAAf/////////////////8AAP//////////////////AAD/////////' +
  '/////////wAA//////////////////8AAP//////////////////AAH//////////////////4AB//////////////////+A' +
  'Af//////////////////gAH//////////////////4AD///////////////////AA///////////////////wAP/////////' +
  '/////////8AD///////////////////AB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gA///////////////////wAB/////////' +
  '/////////gAAH/////////////////gAAAP////////////////AAAAA////////////////AAAAAD///////////////AAA' +
  'AAAH/////////////+AAAAAAAf////////////+AAAAAAAA////////////8AAAAAAAAB///////////4AAAAAAAAAP/////' +
  '/////8AAAAAAAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAA' +
  'AAAAAf//////////gAAAAAAAAAH//////////8AAAAAAAAAD///////////AAAAAAAAAA///////////wAAAAAAAAAf/////' +
  '/////+AAAAAAAAAH///////////gAAAAAAAAB///////////4AAAAAAAAAf//////////+AAAAAAAAAP///////////wAAAA' +
  'AAAAD///////////8AAAAAAAAA////////////AAAAAAAAAP///////////wAAAAAAAAD///////////8AAAAAAAAA//////' +
  '//////AAAAAAAAAP///////////wAAAAAAAAD///////////8AAAAAAAAA////////////AAAAAAAAAP///////////wAAAA' +
  'AAAAD///////////8AAAAAAAAA////////////AAAAAAAAAP///////////wAAAAAAAAB///////////8AAAAAAAAAf/////' +
  '/////+AAAAAAAAAH///////////gAAAAAAAAB///////////4AAAAAAAAAP//////////8AAAAAAAAAD///////////AAAAA' +
  'AAAAA///////////wAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAAAAAAAP//////////AAAAAAAAAAD/////' +
  '/////wAAAAAAAAAAf/////////4AAAAAAAAAAH/////////+AAAAAAAAAAA//////////AAAAAAAAAAAH/////////gAAAAA' +
  'AAAAAA/////////4AAAAAAAAAAAP////////8AAAAAAAAAAAB////////+AAAAAAAAAAAAP////////AAAAAAAAAAAAB////' +
  '////gAAAAAAAAAAAAP///////wAAAAAAAAAAAAB///////4AAAAAAAAAAAAAH//////8AAAAAAAAAAAAAA//////8AAAAAAA' +
  'AAAAAAAD/////+AAAAAAAAAAAAAAAf////+AAAAAAAAAAAAAAAA////+AAAAAAAAAAAAAAAAD///8AAAAAAAAAAAAAAAAAH/' +
  '/8AAAAAAAAAAAAAAAAAAB+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── waterdrop — 비트맵 실루엣(128×128, 잉크 42.2%)
//   원본 마스크: waterdrop.png
var MASK_WATERDROP = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAA8AAAAAAAAAAAAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAAH/gAAAAAAAAA' +
  'AAAAAAAAAB/4AAAAAAAAAAAAAAAAAAAf/AAAAAAAAAAAAAAAAAAAP/wAAAAAAAAAAAAAAAAAAD/8AAAAAAAAAAAAAAAAAAB/' +
  '/gAAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAP//AAAAAAAAAAAAAAAAAAD//wAAAAAAAAAAAAAAAAAB//+AAAAAAAAA' +
  'AAAAAAAAAf//gAAAAAAAAAAAAAAAAAP//8AAAAAAAAAAAAAAAAAD///AAAAAAAAAAAAAAAAAB///4AAAAAAAAAAAAAAAAA//' +
  '//AAAAAAAAAAAAAAAAAP///wAAAAAAAAAAAAAAAAH///+AAAAAAAAAAAAAAAAD////wAAAAAAAAAAAAAAAA////+AAAAAAAA' +
  'AAAAAAAAf////gAAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAA///' +
  '///AAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAD///////AAAAAAA' +
  'AAAAAAB///////4AAAAAAAAAAAAAf///////AAAAAAAAAAAAAf///////4AAAAAAAAAAAAH////////AAAAAAAAAAAAD////' +
  '////wAAAAAAAAAAAB////////+AAAAAAAAAAAA/////////wAAAAAAAAAAAf////////+AAAAAAAAAAAP/////////wAAAAA' +
  'AAAAAH/////////+AAAAAAAAAAB//////////gAAAAAAAAAA//////////8AAAAAAAAAAf//////////gAAAAAAAAAP/////' +
  '/////8AAAAAAAAAD///////////AAAAAAAAAB///////////4AAAAAAAAA////////////AAAAAAAAAP///////////4AAAA' +
  'AAAAH///////////+AAAAAAAAD////////////wAAAAAAAA////////////8AAAAAAAAf////////////gAAAAAAAH//////' +
  '//////4AAAAAAAD/////////////AAAAAAAA/////////////wAAAAAAAf////////////+AAAAAAAH/////////////gAAA' +
  'AAAB/////////////4AAAAAAA//////////////AAAAAAAP/////////////wAAAAAAD/////////////+AAAAAAB///////' +
  '///////gAAAAAAf/////////////4AAAAAAH/////////////+AAAAAAB//////////////gAAAAAA//////////////8AAA' +
  'AAAP//////////////AAAAAAD//////////////wAAAAAA//////////////8AAAAAAP//////////////AAAAAAD///////' +
  '///////wAAAAAA//////////////8AAAAAAP//////////////AAAAAAD//////////////wAAAAAA//////////////8AAA' +
  'AAAP//////////////AAAAAAD//////////////wAAAAAA//////////////8AAAAAAP//////////////AAAAAAD///////' +
  '///////wAAAAAA//////////////8AAAAAAH/////////////+AAAAAAB//////////////gAAAAAAf/////////////4AAA' +
  'AAAH/////////////+AAAAAAA//////////////AAAAAAAP/////////////wAAAAAAD/////////////8AAAAAAAf//////' +
  '//////+AAAAAAAH/////////////gAAAAAAA/////////////4AAAAAAAP////////////8AAAAAAAD/////////////AAAA' +
  'AAAAf////////////gAAAAAAAD////////////4AAAAAAAA////////////8AAAAAAAAH///////////+AAAAAAAAB//////' +
  '//////gAAAAAAAAP///////////wAAAAAAAAB///////////4AAAAAAAAAP//////////+AAAAAAAAAB///////////AAAAA' +
  'AAAAAf//////////gAAAAAAAAAD//////////wAAAAAAAAAAf/////////4AAAAAAAAAAB/////////8AAAAAAAAAAAP////' +
  '////+AAAAAAAAAAAB/////////AAAAAAAAAAAAP////////AAAAAAAAAAAAA////////gAAAAAAAAAAAAH///////gAAAAAA' +
  'AAAAAAAf//////wAAAAAAAAAAAAAB//////wAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAAf////gAAAAAAAAAAAAAAAA//' +
  '//AAAAAAAAAAAAAAAAAA//8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── crown — 비트맵 실루엣(128×128, 잉크 39.1%)
//   원본 마스크: crown.png
var MASK_CROWN = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAEAAAAAAQAAAAAEAAAAAAg' +
  'BgAAAAAGAAAAADgAAAAAIAYAAAAADgAAAAA4AAAAAGAHAAAAAA8AAAAAeAAAAADgB4AAAAAPAAAAAHwAAAAA4AOAAAAAH4AA' +
  'AAB8AAAAAeADwAAAAB+AAAAA/gAAAAPAA+AAAAA/wAAAAP4AAAADwAPgAAAAP8AAAAH/AAAAB8AD8AAAAH/gAAAB/wAAAA/A' +
  'A/gAAAB/4AAAA/8AAAAPwAP4AAAA//AAAAP/gAAAH8AB/AAAAP/wAAAH/4AAAD/AAf4AAAH/8AAAB//AAAA/wAH+AAAB//gA' +
  'AA//wAAAf8AB/wAAAf/4AAAP/8AAAP/AAf+AAAP//AAAH//gAAD/gAH/gAAD//wAAB//4AAB/4AB/8AAB//+AAA///AAA/+A' +
  'Af/gAAf//gAAP//wAAP/gAH/4AAP//8AAH//+AAH/4AB//AAD///AAB///gAB/+AAP/4AA///4AA///8AA//gAD/+AAf//+A' +
  'AP///AAf/4AA//wAH///wAD///wAH/8AAP/+AD///8AB///+AD//AAD//gB////gAf///wB//wAA//8Af///4AP///+A//8A' +
  'AP//wf////AH////////AAB////////4D////////wAAf/////////////////8AAH//////////////////AAB/////////' +
  '/////////wAAf/////////////////8AAH/////////////////+AAB//////////////////gAAf/////////////////4A' +
  'AH/////////////////+AAB//////////////////gAAP/////////////////4AAD/////////////////+AAA/////////' +
  '/////////gAAP/////////////////wAAD/////////////////8AAA//////////////////AAAP/////////////////wA' +
  'AD/////////////////8AAA//////////////////AAAP/////////////////wAAD/////////////////8AAA/////////' +
  '/////////AAAP/////////////////wAAD/////////////////8AAA//////////////////AAAP/////////////////wA' +
  'AD/////////////////8AAA//////////////////AAAP/////////////////wAAD/////////////////8AAA/////////' +
  '/////////AAAP/////////////////wAAD/////////////////8AAA//////////////////AAAP/////////////////wA' +
  'AD/////////////////8AAA//////////////////AAAP/////////////////wAAD/////////////////8AAA/////////' +
  '/////////AAAP/////////////////wAAD/////////////////8AAA//////////////////AAAP/////////////////wA' +
  'AB/////////////////4AAAP////////////////+AAAB////////////////+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── bag — 비트맵 실루엣(128×128, 잉크 42.3%)
//   원본 마스크: bag.png
var MASK_BAG = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAH/wAAAAAAAAAAAAAAAAAAf//gAAAAAAAAAAAAAAAAA////AAAAAAAAAAAAAAAAA////8AAAAAAAA' +
  'AAAAAAAA/////wAAAAAAAAAAAAAAAf////+AAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAH///' +
  '///4AAAAAAAAAAAAAD///////AAAAAAAAAAAAAB///////4AAAAAAAAAAAAA///gA///AAAAAAAAAAAAAP//AAD//wAAAAAA' +
  'AAAAAAH//AAAP/+AAAAAAAAAAAAD//AAAA//wAAAAAAAAAAAA//gAAAH/8AAAAAAAAAAAAf/wAAAA//gAAAAAAAAAAAH/4AA' +
  'AAH/4AAAAAAAAAAAD/8AAAAA//AAAAAAAAAAAA/+AAAAAH/wAAAAAAAAAAAf/gAAAAB/+AAAAAAAAAAAH/wAAAAAP/gAAAAA' +
  'AAAAAB/8AAAAAB/4AAAAAAAAAAA/+AAAAAAf/AAAAAAAAAAAP/gAAAAAH/wAAAAAAAAAAD/4AAAAAB/8AAAAAAAAAAA/+AAA' +
  'AAAf/AAAAAAAAAAAf/gAAAAAH/4AAAAAAAAAAH/4AAAAAB/+AAAAAAAAAAB//AAAAAA//gAAAAAAAAAAf/wAAAAAP/4AAAAA' +
  'AAAAAP/+AAAAAH//AAAAAAAAAAH//wAAAAD//4AAAAAAAAAD///AAAAD///AAAAAAAAAB///////////4AAAAAAAAB//////' +
  '//////gAAAAAAAA////////////+AAAAAAAA/////////////wAAAAAAAf////////////+AAAAAAAH/////////////wAAA' +
  'AAAD/////////////8AAAAAAA//////////////gAAAAAAf/////////////4AAAAAAH/////////////+AAAAAAB///////' +
  '///////gAAAAAA//////////////8AAAAAAP//////////////AAAAAAD//////////////wAAAAAA//////////////8AAA' +
  'AAAf//////////////gAAAAAH//////////////4AAAAAB//////////////+AAAAAAf//////////////wAAAAAP///////' +
  '///////8AAAAAD///////////////AAAAAA///////////////wAAAAAf//////////////+AAAAAH///////////////gAA' +
  'AAB///////////////4AAAAAf//////////////+AAAAAP///////////////wAAAAD///////////////8AAAAA////////' +
  '////////AAAAAP///////////////wAAAAH///////////////+AAAAB////////////////gAAAAf///////////////4AA' +
  'AAH///////////////+AAAAD////////////////wAAAA////////////////8AAAAP//wAAAAAAAAAA///AAAAD//gAAAAA' +
  'AAAAAB//wAAAB//wAAAAAAAAAAAP/+AAAAf/8AAAAAAAAAAAD//gAAAH/+AAAAAAAAAAAAf/4AAAB//gAAAAAAAAAAAH//AA' +
  'AA//4AAAAAAAAAAAB//wAAAP/8AAAAAAAAAAAAP/8AAAD//AAAAAAAAAAAAD//AAAA//wAAAAAAAAAAAA//4AAAf/+AAAAAA' +
  'AAAAAAP/+AAAH//gAAAAAAAAAAAH//gAAB//4AAAAAAAAAAAB//4AAA//+AAAAAAAAAAAAf//AAAP//gAAAAAAAAAAAH//wA' +
  'AD//8AAAAAAAAAAAB//8AAA///AAAAAAAAAAAA///AAAP//wAAAAAAAAAAAP//4AAD//+AAAAAAAAAAAH//+AAA///gAAAAA' +
  'AAAAAB///gAAf//8AAAAAAAAAAA///4AAH///AAAAAAAAAAAP//+AAB///4AAAAAAAAAAH///gAAf///AAAAAAAAAAD///4A' +
  'AD///wAAAAAAAAAA///+AAA///+AAAAAAAAAAf///gAAP///wAAAAAAAAAP///wAAD////AAAAAAAAAP///8AAA////4AAAA' +
  'AAAAH////AAAP////gAAAAAAAH////wAAB////+AAAAAAAH////4AAAf////4AAAAAAH////+AAAD/////////////////AA' +
  'AA/////////////////wAAAH////////////////4AAAB////////////////+AAAAP////////////////AAAAB////////' +
  '////////wAAAAf///////////////4AAAAD///////////////8AAAAAf//////////////+AAAAAD///////////////AAA' +
  'AAAf//////////////gAAAAAB//////////////wAAAAAAP/////////////wAAAAAAA/////////////wAAAAAAAD//////' +
  '//////wAAAAAAAAH///////////gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── watch — 비트맵 실루엣(128×128, 잉크 31.6%)
//   원본 마스크: watch.png
var MASK_WATCH = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAB//////8AAAAAAA' +
  'AAAAAAAf//////AAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAf//////AAAAAAAAAAAAAAH///' +
  '///wAAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAf//////AAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAB//////8AAAAAAA' +
  'AAAAAAAf//////AAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAf//////AAAAAAAAAAAAAAH///' +
  '///wAAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAf//////AAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAB//////8AAAAAAA' +
  'AAAAAAAf//////gAAAAAAAAAAAAAP//////8AAAAAAAAAAAAAH///////gAAAAAAAAAAAAB///////4AAAAAAAAAAAAA////' +
  '///+AAAAAAAAAAAAAP///////wAAAAAAAAAAAAD///////8AAAAAAAAAAAAA////////AAAAAAAAAAAAAf///////4AAAAAA' +
  'AAAAAAH//4AB//+AAAAAAAAAAAAD//wAAD//wAAAAAAAAAAAB//wAAAP/+AAAAAAAAAAAA//wAAAA//wAAAAAAAAAAAf/wAA' +
  'AAD/+AAAAAAAAAAAH/4AAAAAf/gAAAAAAAAAAD/8AAAAAD/8AAAAAAAAAAB/+AAP8AAf/gAAAAAAAAAAf/AA//4AD/4AAAAA' +
  'AAAAAP/gA///wAf/AAAAAAAAAAD/wAf//+AD/wAAAAAAAAAB/4Af///4Af+AAAAAAAAAAf+AP////AH/gAAAAAAAAAP/AH//' +
  '//4A/8AAAAAAAAAD/wD/////AP/AAAAAAAAAA/4B/////4B/wAAAAAAAAAf+Af////+Af+AAAAAAAAAH/AP/////wD/gAAAA' +
  'AAAAB/wH/////8A/4AAAAAAAAA/4B//////gH/AAAAAAAAAP+Af/////4B/wAAAAAAAAD/gP/////+Af8AAAAAAAAA/4D///' +
  '///wH/AAAAAAAAAP+A//////8B/wAAAAAAAAD/AP//////AP8AAAAAAAAA/wH//////wD/AAAAAAAAAP8B//////8A/wAAAA' +
  'AAAAD/Af//////AP+AAAAAAAAA/wH//////wD/AAAAAAAAAP8B//////8A/wAAAAAAAAD/AP//////AP8AAAAAAAAA/4D///' +
  '///wD/AAAAAAAAAP+A//////8B/wAAAAAAAAD/gP//////Af8AAAAAAAAA/4B//////gH/AAAAAAAAAP+Af/////4B/wAAAA' +
  'AAAAB/wH/////+A/4AAAAAAAAAf8A//////AP+AAAAAAAAAH/AP/////gH/gAAAAAAAAA/4B/////4B/4AAAAAAAAAP+AP//' +
  '//8A/8AAAAAAAAAD/wB////+AP/AAAAAAAAAAf8AP////AH/gAAAAAAAAAH/gB////gB/4AAAAAAAAAB/8AP///wA/8AAAAA' +
  'AAAAAP/gA///wAf/AAAAAAAAAAB/4AH//4AP/gAAAAAAAAAAf/AAP/wAH/4AAAAAAAAAAD/4AAAAAD/8AAAAAAAAAAAf/gAA' +
  'AAB/+AAAAAAAAAAAH/8AAAAA//gAAAAAAAAAAA//gAAAA//wAAAAAAAAAAAH/+AAAAf/4AAAAAAAAAAAA//4AAAf/8AAAAAA' +
  'AAAAAAH//wAA//+AAAAAAAAAAAAB///4H///gAAAAAAAAAAAAP///////wAAAAAAAAAAAAD///////8AAAAAAAAAAAAA////' +
  '////AAAAAAAAAAAAAP///////wAAAAAAAAAAAAB///////4AAAAAAAAAAAAAf//////+AAAAAAAAAAAAAH///////gAAAAAA' +
  'AAAAAAA///////wAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAf//////AAAAAAAAAAAAAAH///' +
  '///wAAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAf//////AAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAB//////8AAAAAAA' +
  'AAAAAAAf//////AAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAf//////AAAAAAAAAAAAAAH///' +
  '///wAAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAf//////AAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAB//////8AAAAAAA' +
  'AAAAAAAf//////AAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAf//////AAAAAAAAAAAAAAD///' +
  '///wAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── ring — 비트맵 실루엣(128×128, 잉크 25.9%)
//   원본 마스크: ring.png
var MASK_RING = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAB/////AAAAAAAAAAAAAAAA/////wAAAAAAAAAAAAAAAf////+AAAAAAAAAAAAAAAP/////wAAAAAAA' +
  'AAAAAAAD/////+AAAAAAAAAAAAAAB//gB//gAAAAAAAAAAAAAA/n4Afj8AAAAAAAAAAAAAAPw+AD4/gAAAAAAAAAAAAAH4PA' +
  'A+H8AAAAAAAAAAAAAD+HwAPg/AAAAAAAAAAAAAB/h///8f4AAAAAAAAAAAAAf//////+AAAAAAAAAAAAAH///////gAAAAAA' +
  'AAAAAAA///////4AAAAAAAAAAAAAP//////8AAAAAAAAAAAAAB//4Af/+AAAAAAAAAAAAAAf/8AD//gAAAAAAAAAAAAAD//A' +
  'A//wAAAAAAAAAAAAAA//wAP/8AAAAAAAAAAAAAAH/+AH/+AAAAAAAAAAAAAAA//wD//gAAAAAAAAAAAAAAP/8A//wAAAAAAA' +
  'AAAAAAAB//gf/8AAAAAAAAAAAAAAAf/8H/+AAAAAAAAAAAAAAAH//D//gAAAAAAAAAAAAAAH//4//+AAAAAAAAAAAAAAH///' +
  '///4AAAAAAAAAAAAAD///////gAAAAAAAAAAAAD///////+AAAAAAAAAAAAD////////wAAAAAAAAAAAB/////////AAAAAA' +
  'AAAAAB/////////4AAAAAAAAAAA//////////AAAAAAAAAAAf/////////4AAAAAAAAAAP//////////gAAAAAAAAAH/////' +
  '/////8AAAAAAAAAD////AAD////gAAAAAAAAB///+AAAD///8AAAAAAAAA///+AAAAP///AAAAAAAAAf//8AAAAA///4AAAA' +
  'AAAAP//8AAAAAD///AAAAAAAAH//+AAAAAAf//4AAAAAAAB//+AAAAAAB///AAAAAAAA///AAAAAAAP//wAAAAAAAf//gAAA' +
  'AAAB//+AAAAAAAH//wAAAAAAAP//gAAAAAAD//4AAAAAAAB//8AAAAAAA//8AAAAAAAAP//gAAAAAAf/+AAAAAAAAB//4AAA' +
  'AAAP//AAAAAAAAAP//AAAAAAD//wAAAAAAAAD//wAAAAAA//4AAAAAAAAAf/8AAAAAAf/8AAAAAAAAAD//gAAAAAH//AAAAA' +
  'AAAAA//4AAAAAB//gAAAAAAAAAH/+AAAAAA//4AAAAAAAAAB//wAAAAAP/8AAAAAAAAAAP/8AAAAAD//AAAAAAAAAAD//AAA' +
  'AAB//gAAAAAAAAAAf/4AAAAAf/4AAAAAAAAAAH/+AAAAAH/+AAAAAAAAAAB//gAAAAB//gAAAAAAAAAAP/4AAAAAf/wAAAAA' +
  'AAAAAD/+AAAAAP/8AAAAAAAAAAA//wAAAAD//AAAAAAAAAAAP/8AAAAA//wAAAAAAAAAAD//AAAAAP/8AAAAAAAAAAA//wAA' +
  'AAD//AAAAAAAAAAAH/8AAAAA//wAAAAAAAAAAB//AAAAAP/8AAAAAAAAAAAf/wAAAAD//AAAAAAAAAAAH/8AAAAA//wAAAAA' +
  'AAAAAB//AAAAAP/8AAAAAAAAAAAf/wAAAAD//AAAAAAAAAAAP/8AAAAA//wAAAAAAAAAAD//AAAAAP/8AAAAAAAAAAA//wAA' +
  'AAD//AAAAAAAAAAAP/8AAAAAf/wAAAAAAAAAAD/+AAAAAH/+AAAAAAAAAAA//gAAAAB//gAAAAAAAAAAf/4AAAAAf/4AAAAA' +
  'AAAAAH/+AAAAAH//AAAAAAAAAAB//gAAAAA//wAAAAAAAAAA//wAAAAAP/8AAAAAAAAAAP/8AAAAAD//gAAAAAAAAAH//AAA' +
  'AAAf/4AAAAAAAAAB//gAAAAAH//AAAAAAAAAA//4AAAAAB//wAAAAAAAAAP/+AAAAAAP/+AAAAAAAAAH//AAAAAAD//wAAAA' +
  'AAAAB//wAAAAAAf/8AAAAAAAAA//8AAAAAAH//gAAAAAAAAf/+AAAAAAA//8AAAAAAAAP//gAAAAAAP//gAAAAAAAH//wAAA' +
  'AAAB//8AAAAAAAD//4AAAAAAAf//gAAAAAAB//+AAAAAAAD//8AAAAAAA///AAAAAAAAf//gAAAAAAf//gAAAAAAAH//+AAA' +
  'AAAP//4AAAAAAAA///wAAAAAP//8AAAAAAAAH///AAAAAP//+AAAAAAAAA///8AAAAP///AAAAAAAAAH///wAAAP///gAAAA' +
  'AAAAA////gAAf///wAAAAAAAAAH////8P////4AAAAAAAAAA//////////8AAAAAAAAAAH/////////+AAAAAAAAAAA/////' +
  '/////AAAAAAAAAAAH/////////gAAAAAAAAAAAf////////gAAAAAAAAAAAB////////wAAAAAAAAAAAAP///////wAAAAAA' +
  'AAAAAAA///////4AAAAAAAAAAAAAD//////4AAAAAAAAAAAAAAP/////4AAAAAAAAAAAAAAAf////wAAAAAAAAAAAAAAAA//' +
  '//AAAAAAAAAAAAAAAAAAf/8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── perfume — 비트맵 실루엣(128×128, 잉크 36.6%)
//   원본 마스크: perfume.png
var MASK_PERFUME = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAf//////+AAAAAAAAAAAAAP///////wAAAAAA' +
  'AAAAAAH///////+AAAAAAAAAAAAB////////wAAAAAAAAAAAA////////8AAAAAAAAAAAAP////////AAAAAAAAAAAAH////' +
  '////4AAAAAAAAAAAB////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAH////////4AAAAAAAAAAAB////////+AAAAAA' +
  'AAAAAAf////////gAAAAAAAAAAAH////////4AAAAAAAAAAAB////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAH////' +
  '////4AAAAAAAAAAAB////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAH////////4AAAAAAAAAAAB////////+AAAAAA' +
  'AAAAAAf////////gAAAAAAAAAAAD////////4AAAAAAAAAAAA////////8AAAAAAAAAAAAP////////AAAAAAAAAAAAB////' +
  '////gAAAAAAAAAAAAP///////wAAAAAAAAAAAAB///////8AAAAAAAAAAAAAf//////+AAAAAAAAAAAAAH///////gAAAAAA' +
  'AAAAAAB///////4AAAAAAAAAAAAAf//////+AAAAAAAAAAAAAH///////gAAAAAAAAAAAAB///////4AAAAAAAAAAAAAf///' +
  '///+AAAAAAAAAAAAAH///////gAAAAAAAAAAAAD///////8AAAAAAAAAAAAA////////AAAAAAAAAAAAAf///////4AAAAAA' +
  'AAAAAAf////////gAAAAAAAAAAA//////////AAAAAAAAAAH///////////gAAAAAAAAH///////////+AAAAAAAAH//////' +
  '//////4AAAAAAAH/////////////gAAAAAAD/////////////8AAAAAAD//////////////wAAAAAB//////////////+AAA' +
  'AAA///////////////wAAAAAf//////////////+AAAAAH///////////////gAAAAD///////////////8AAAAB////////' +
  '////////gAAAAf///gAAAAAAAH///4AAAAP//+AAAAAAAAAH///AAAAD//+AAAAAAAAAAf//wAAAA///AAAAAAAAAAD//+AA' +
  'AAf//gAAAAAAAAAAf//gAAAH//wAAAAAAAAAAD//4AAAB//4AAAAAAAAAAAf/+AAAA//+AAAAAAAAAAAD//wAAAP//AAAAAA' +
  'AAAAAA//8AAAD//wAAAAAAAAAAAP//AAAA//8AAAAAAAAAAAD//wAAAP/+AAAAAAAAAAAAf/8AAAD//gAAAAAAAAAAAH//AA' +
  'AA//4AAAAAAAAAAAB//wAAAP/+AAAAAAAAAAAAf/8AAAD//gAAAAAAAAAAAH//AAAA//4AAAAAAAAAAAB//wAAAP/+AAAAAA' +
  'AAAAAAf/8AAAD//gAAAAAAAAAAAH//AAAA//4AAAAAAAAAAAB//wAAAP/+AAAAAAAAAAAAf/8AAAD//gAAAAAAAAAAAH//AA' +
  'AA//4AAAAAAAAAAAB//wAAAP/+AAAAAAAAAAAAf/8AAAD//gAAAAAAAAAAAH//AAAA//4AAAAAAAAAAAB//wAAAP/+AAAAAA' +
  'AAAAAAf/8AAAD//gAAAAAAAAAAAH//AAAA//4AAAAAAAAAAAB//wAAAP/+AAAAAAAAAAAAf/8AAAD//gAAAAAAAAAAAH//AA' +
  'AA//4AAAAAAAAAAAB//wAAAP/+AAAAAAAAAAAAf/8AAAD//gAAAAAAAAAAAH//AAAA//4AAAAAAAAAAAB//wAAAP/+AAAAAA' +
  'AAAAAAf/8AAAD//gAAAAAAAAAAAH//AAAA//4AAAAAAAAAAAB//wAAAP/+AAAAAAAAAAAAf/8AAAD//gAAAAAAAAAAAH//AA' +
  'AA//8AAAAAAAAAAAD//wAAAP//AAAAAAAAAAAA//8AAAD//wAAAAAAAAAAAP//AAAA//+AAAAAAAAAAAH//wAAAH//wAAAAA' +
  'AAAAAB//4AAAB//+AAAAAAAAAAA//+AAAAf//gAAAAAAAAAAf//gAAAH//+AAAAAAAAAAf//4AAAA///4AAAAAAAAAf//8AA' +
  'AAP///gAAAAAAAAf///AAAAB////////////////gAAAAf///////////////4AAAAD///////////////8AAAAA////////' +
  '////////AAAAAH///////////////gAAAAA///////////////wAAAAAH//////////////4AAAAAA//////////////8AAA' +
  'AAAH/////////////+AAAAAAA//////////////AAAAAAAD/////////////AAAAAAAAP////////////AAAAAAAAAf/////' +
  '/////+AAAAAAAAAAf/////////4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── helmet — 비트맵 실루엣(128×128, 잉크 50.3%)
//   원본 마스크: helmet.png
var MASK_HELMET = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA/wAAAAAAAAAAAAAAAAAAH//4AAAAAAAAAAAAAAAAAD///AAAAAAAAA' +
  'AAAAAAAAA///4AAAAAAAAAAAAAAAAAf//+AAAAAAAAAAAAAAAAAP///wAAAAAAAAAAAAAAAAH///+AAAAAAAAAAAAAAAAf//' +
  '//+AAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAf//////+AAAAAAAAAAAAAf///////4AAAAAA' +
  'AAAAAAH////////AAAAAAAAAAAAD////////wAAAAAAAAAAAB////////+AAAAAAAAAAAB/////////4AAAAAAAAAAA/////' +
  '/////gAAAAAAAAAA//////////8AAAAAAAAAAf//////////gAAAAAAAAAP//////////8AAAAAAAAAH///////////wAAAA' +
  'AAAAH///////////+AAAAAAAAD////////////wAAAAAAAA////////////+AAAAAAAAf////////////wAAAAAAAP//////' +
  '//////+AAAAAAAH/////////////gAAAAAAD/////////////8AAAAAAB//////////////gAAAAAAf/////////////8AAA' +
  'AAAP//////////////AAAAAAH//////////////4AAAAAB//////////////+AAAAAA///////////////wAAAAAP///////' +
  '///////8AAAAAH///////////////gAAAAB///////////////4AAAAA////////////////AAAAAP///////////////wAA' +
  'AAD///////////////+AAAAB////////////////gAAAAf///////////////4AAAAP////////////////AAAAD////////' +
  '////////wAAAA////////////////8AAAAP////////////////AAAAH////////////////4AAAB////////////////+AA' +
  'AAf////////////////gAAAH////////////////4AAAB////////////////+AAAAf////////////////wAAAH////////' +
  '////////8AAAB/////////////////AAAA/////////////////wAAAP////////////////8AAAD/////////////////AA' +
  'AA/////////////////wAAAP////////////////8AAAD/////////////////AAAB/////////////////4AAAf////////' +
  '////////+AAAP/////////////////wAAH/////////////////+AAD//////////////////wAA//////////////////8A' +
  'AP//////////////////AAD//////////////////wAB//////////////////+AAf//////////////////gAP/////////' +
  '/////////8AH///////////////////gB///////////////////4Af//////////////////+AD///////////////////A' +
  'A///////////////////wAH//////////////////4AAf/////////////////4AAD/////////////////8AAAf////////' +
  '////////+AAAD/////////////////AAAAf////////////////gAAAD////////////////wAAAAP///////////////4AA' +
  'AAB///////////////4AAAAAP//////////////8AAAAAB//////////////+AAAAAAP//////////////AAAAAAA///////' +
  '///////AAAAAAAH/////////////gAAAAAAAf////////////wAAAAAAAD////////////wAAAAAAAAP///////////wAAAA' +
  'AAAAA///////////wAAAAAAAAAA//////////AAAAAAAAAAAB////////+AAAAAAAAAAAAAf//////gAAAAAAAAAAAAAAAP/' +
  '/8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── fan — 비트맵 실루엣(128×128, 잉크 37.6%)
//   원본 마스크: fan.png
var MASK_FAN = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAP/wAAAAAAAAAAAAAAAAAD///8AAAAAAAAAAAAAAAAH////4AAAAAAAAAAAAAAAH/////gAAAAAAA' +
  'AAAAAAAD/////8AAAAAAAAAAAAAAA//////gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB///' +
  '///gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAA//////AAAAAAAAAAAAAAAP/////wAAAAAAA' +
  'AAAAAAAD/////8AAAAAAAAAAAAAAA/////+AAAAAAAAAAAAAAAP/////gAAAAAAAAAAAAAAD/////4AAAAAAAAAAAAAAAf//' +
  '//8AAAAAAAAAAAAAAAH/////AAAAAAAAAAAAAAAB/////wAAAAAAAAAAAAAAAf////4AAAAAAAAAAAAAAAD////+AAAAAAAA' +
  'AAAAAAAA/////gAAAAAAAAAAAAAAAP////wAAAAAAAAAAAAAAAB////8AAAAAAAAAAAAAAAAf////AAAAAAAAAAAAAAAAH//' +
  '//gAAAAAAAAAAAAAAAA////4AAAAAAAAAAAAAAAAP///+AAAAAAAAAAAAAAAAD////AAAAAAAAAAAAAAAAAf///wAAAAAAAA' +
  'AAAAAAAAH///8AAAAAAAAAAAAAAAAB///+AAAAAAAAAAAAAAAAAP///gAAAAAAAAAAAAAAAAD///4AAAAAAAAAAAAAAAAAf/' +
  '/8AAAAAAAAAAAAAAAAAH///AAAAAAAAAAAAAAAAAA///wAAAAAAAAAAGAAAAAAP//8AAAAAf/AAAH+AAAAAB//+AAAAB//8A' +
  'AD/8AAAAAf//gAAAH///AAB//wAAAAD//4AAAP///4AAf//gAAAA//+AAAf///+AAP//+AAAA///wAAf////wAD///8AAAf/' +
  '/+AAf////8AB////4AAP///4A//////AAf////wAH////A//////wAH/////AD////wf/////8AB//////B////+f//////g' +
  'A///////////////////4AP//////////////////+AD///////////////////gA///////////////////4AP/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////wAf//////////////////8AH///////////////////A' +
  'B////////////z//////wAP//////n////8D/////8AD//////g////+AH////+AA//////gP////AAP////gAP/////gB//' +
  '//gAA////4AD/////gAP///wAAB///8AAf////AAA///4AAAH///AAH////AAAH//4AAAAP//wAA///+AAAB//+AAAAA//4A' +
  'AP//4AAAAf//gAAAAB/8AAB//gAAAAH//8AAAAAH+AAAHwAAAAAB///gAAAAAHAAAAAAAAAAA///4AAAAAAAAAAAAAAAAAP/' +
  '//AAAAAAAAAAAAAAAAAD///wAAAAAAAAAAAAAAAAA///+AAAAAAAAAAAAAAAAAf///gAAAAAAAAAAAAAAAAH///8AAAAAAAA' +
  'AAAAAAAAB////AAAAAAAAAAAAAAAAA////4AAAAAAAAAAAAAAAAP///+AAAAAAAAAAAAAAAAD////gAAAAAAAAAAAAAAAB//' +
  '//8AAAAAAAAAAAAAAAAf////AAAAAAAAAAAAAAAAH////wAAAAAAAAAAAAAAAD////+AAAAAAAAAAAAAAAA/////gAAAAAAA' +
  'AAAAAAAAP////4AAAAAAAAAAAAAAAH/////AAAAAAAAAAAAAAAB/////wAAAAAAAAAAAAAAAf////8AAAAAAAAAAAAAAAP//' +
  '///AAAAAAAAAAAAAAAD/////4AAAAAAAAAAAAAAB/////+AAAAAAAAAAAAAAAf/////gAAAAAAAAAAAAAAH/////4AAAAAAA' +
  'AAAAAAAD/////+AAAAAAAAAAAAAAA//////gAAAAAAAAAAAAAAP/////8AAAAAAAAAAAAAAH//////AAAAAAAAAAAAAAB///' +
  '///wAAAAAAAAAAAAAAf/////8AAAAAAAAAAAAAAH//////AAAAAAAAAAAAAAB//////wAAAAAAAAAAAAAAf/////8AAAAAAA' +
  'AAAAAAAD/////+AAAAAAAAAAAAAAAf/////gAAAAAAAAAAAAAAB/////gAAAAAAAAAAAAAAAH////wAAAAAAAAAAAAAAAAP/' +
  '//gAAAAAAAAAAAAAAAAAD/4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── goggles — 비트맵 실루엣(128×128, 잉크 16.8%)
//   원본 마스크: goggles.png
var MASK_GOGGLES = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAB/////4AAAAAAAAAAAAAD////' +
  '////4AAAAAAAAAAA//////////+AAAAAAAAAH///////////+AAAAAAAAP////////////+AAAAAAA//////////////8AAA' +
  'AAAf//////////////gAAAAAf//////////////+AAAAAP///////////////wAAAAH///////////////+AAAAB////+AAA' +
  'AAAP////wAAAA///4AAAAAAAAAf//8AAAAP//AAAAAAAAAAAH//gAAAD/4AAAAAAAAAAAAH/4AAAB/4AAAAAAAAAAAAAP+AA' +
  'AAf4AAAAAAAAAAAAAB/gAAAH8AAAAAAAAAAAAAAP8AAAD/AAAAAAAAAAAAAAB/AAAB/gAAAAAAAAAAAAAAf8AAP/4AAAAAAA' +
  'AAAAAAAH/+AH/+AAAAAAAAAAAAAAA//gB//gAAAAAAAAAAAAAAP/4Af/4AAAAAAAAAAAAAAD/+AH/+AAAAAAAAAAAAAAA//g' +
  'B//gAAAAAAAAAAAAAAP/4Af/4AAAAAAAAAAAAAAD/+AH/+AAAAAAAAAAAAAAA//gB//gAAAAAAAAAAAAAAP/4Af/4AAAAAAA' +
  'AAAAAAAD/+AH/+AAAAAAAAAAAAAAA//gB//gAAAAAAAAAAAAAAP/4Af/4AAAAAAAAAAAAAAD/+AH/+AAAAAAAAAAAAAAA//g' +
  'B//gAAAAAAAAAAAAAAP/4Af/4AAAAAAAAAAAAAAD/+AH/+AAAAAAA8AAAAAAA//gB//gAAAAAA/4AAAAAAP/4Af/4AAAAAAf' +
  '/AAAAAAD/+AH/+AAAAAAP/4AAAAAA//gB//gAAAAAH//AAAAAAP/4Af/4AAAAAD//wAAAAAD/+AD/+AAAAAA//+AAAAAB//A' +
  'AA/gAAAAAf//gAAAAAfwAAAP4AAAAAH//8AAAAAH8AAAB+AAAAAD/D/AAAAAB/AAAAfgAAAAA/gf4AAAAAfgAAAH8AAAAAf4' +
  'D+AAAAAH4AAAB/AAAAAH8A/wAAAAD+AAAAf4AAAAD/AH8AAAAA/gAAAD+AAAAA/gB/AAAAAf4AAAA/wAAAAf4AP4AAAAP8AA' +
  'AAP+AAAAH8AD+AAAAD/AAAAB/wAAAB/AAfwAAAD/wAAAAf/AAAA/wAH8AAAB/4AAAAD/4AAAP4AB/gAAB/+AAAAA//gAAH+A' +
  'AP4AAB//AAAAAH/+AAD/AAD/AAB//gAAAAA//8AA/wAAf4AD//wAAAAAH//8A/4AAH/AP//4AAAAAA/////+AAA/////8AAA' +
  'AAAH/////AAAP////+AAAAAAAf////wAAB/////AAAAAAAD////4AAAf////AAAAAAAAP///8AAAD////AAAAAAAAA////AA' +
  'AAf///AAAAAAAAAB///gAAAD///AAAAAAAAAAH//gAAAAf/+AAAAAAAAAAAH/wAAAAB/4AAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── harness — 비트맵 실루엣(128×128, 잉크 29.6%)
//   원본 마스크: harness.png
var MASK_HARNESS = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAA/AAAAAAH8AAAAAAAAAAAH/4AAAAAB/+AAAAAAAAAAD//AAAAAA//gAAAAAAAAAA//wAAAAAf/4AAAAA' +
  'AAAAAP/+AAAAAH/+AAAAAAAAAAD//wAAAAD//gAAAAAAAAAA//8AAAAB//4AAAAAAAAAAP//gAAAAf/+AAAAAAAAAAD//8AA' +
  'AAP//gAAAAAAAAAA///AAAAH//4AAAAAAAAAAP//4AAAB//+AAAAAAAAAAD//+AAAA///gAAAAAAAAAA///wAAAf//4AAAAA' +
  'AAAAAP//+AAAH//+AAAAAAAAAAD///gAAD///gAAAAAAAAAAf//8AAB///4AAAAAAAAAAH///gAAf//+AAAAAAAAAAB///4A' +
  'AP///gAAAAAAAAAAf+f/AAH/7/wAAAAAAAAAAH/3/4AB/8/8AAAAAAAAAAB/8/+AA/+P/AAAAAAAAAAAf/H/wAf/j/wAAAAA' +
  'AAAAAD/x/+AH/x/8AAAAAAAAAAA/8P/gD/4f/AAAAAAAAAAAP/B/8A/+H/gAAAAAAAAAAD/wf/gf/B/4AAAAAAAAAAA/+D//' +
  '//wf+AAAAAAAAAAAH/g////4H/gAAAAAAAAAAB/4H///+D/4AAAAAAAAAAAf+A////A/8AAAAAAAAAAAH/wP///gP/AAAAAA' +
  'AAAAAA/8B///4D/wAAAAAAAAAAAP/A/gB/B/8AAAAAAAAAAAD/wPwAPwf+AAAAAAAAAAAA/+D8AD8H/gAAAAAAAAAAAH/g/A' +
  'A/B/4AAAAAAAAAAAB/4PwAPw/+AAAAAAAAAAAAf+D8AD8P/AAAAAAAAAAAAH/w/AA/D/wAAAAAAAAAAAA/8PwAPw/8AAAAAA' +
  'AAAAAAP/D8AD8f+AAAAAAAAAAAAD/4/AA/H/gAAAAAAAAAAAAf+P4Afx/4AAAAAAAAAAAAH/n///+f+AAAAAAAAAAAAB/7//' +
  '//n/AAAAAAAAAAAAAf9////8/wAAAAAAAAAAAAD+f////n4AAAAAAAAAAAAA/P////8+AAAAAAAAAAAAAP3/////vgAAAAAA' +
  'AAAAAAB5/////5wAAAAAAAAAAAAAc//////MAAAAAAAAAAAAACf/n/n/7AAAAAAAAAAAAAAv/x/5//AAAAAAAAAAAAAAD/4f' +
  '+P/wAAAAAAAAAAAAAB/+H/h/+AAAAAAAAAAAAAA//B/4P/wAAAAAAAAAAAAAf/gf+D/+AAAAAAAAAAAAAH/wH/gf/gAAAAAA' +
  'AAAAAAD/4B/4D/8AAAAAAAAAAAAB/+Af+Af/gAAAAAAAAAAAAf/AH/gH/4AAAAAAAAAAAAP/gB/4A//AAAAAAAAAAAAH/4Af' +
  '+AH/4AAAAAAAAAAAB/8AH/gA//AAAAAAAAAAAA/+AB/4AP/wAAAAAAAAAAAf/gAf+AB/+AAAAAAAAAAAH/wAH/gAP/wAAAAA' +
  'AAAAAD/4AAAAAB/8AAAAAAAAAAA/8AAAAAAf/AAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAA////v//f///wAAAAAAAAAP////g' +
  'B3///8AAAAAAAAAD////7/d////AAAAAAAAAA///////f///wAAAAAAAAAP//////3///8AAAAAAAAAD//////9////AAAAA' +
  'AAAAA///////f///wAAAAAAAAAP////v93///8AAAAAAAAAD////4Ad////AAAAAAAAAAf///v/+f///wAAAAAAAAAH/gAB/' +
  '/gAB/4AAAAAAAAAB/4AAP/gAAf+AAAAAAAAAA/8AAB/4AAD/wAAAAAAAAAP/AAAf+AAA/8AAAAAAAAAD/gAAH/gAAP/AAAAA' +
  'AAAAB/4AAB/4AAB/4AAAAAAAAAf+AAAf+AAAf+AAAAAAAAAP/AAAH/gAAD/wAAAAAAAAD/wAAB/4AAA/8AAAAAAAAA/4AAAf' +
  '+AAAP/AAAAAAAAAf+AAAH/gAAB/4AAAAAAAAH/gAAB/4AAAf+AAAAAAAAD/8AAAf+AAAP/wAAAAAAAA//wAAH/gAAf/8AAAA' +
  'AAAAP//gAB/4AAf//gAAAAAAAH//+AAf+AAf//4AAAAAAAB///4AH/gAf//+AAAAAAAAf///gB/4Af///wAAAAAAAP///+Af' +
  '+Af///8AAAAAAAD////wH/gP////AAAAAAAA/z///B/4P//8/wAAAAAAAP4H//4f+H//4H8AAAAAAAD+Af//H/j//4B/AAAA' +
  'AAAA/AA//9/5//4APwAAAAAAAH4AD//////wAH4AAAAAAAB/AAP/////wAB+AAAAAAAAfwAA//AP/4AA/gAAAAAAAD/AAH/w' +
  'D/4AA/wAAAAAAAA/4AAf8A/4AAf8AAAAAAAAH/wAD/AP8AA/+AAAAAAAAA//gAfwD/AB//AAAAAAAAAP////8A/////wAAAA' +
  'AAAAA/////AP////4AAAAAAAAAH////wD////4AAAAAAAAAAf///4Af///8AAAAAAAAAAD///+AH///8AAAAAAAAAAAH//+A' +
  'Af//4AAAAAAAAAAAAH/8AAA//gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── extinguisher — 비트맵 실루엣(128×128, 잉크 36.8%)
//   원본 마스크: extinguisher.png
var MASK_EXTINGUISHER = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAA//+AAAAAAAAAAAAAAAAAB///wAAAAAAAAAAAAAAAAD///8AAAA' +
  'AAAAAAAAAAAAH///+AAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAf///+AAAAAAAAAAAAAAAA////8AAAAAAAAAAAAAAAB' +
  '////wAAAAAAAAAAAAAAAA////gAAAAAAAAAAAAAAAAf///AAAAAAAAAAAAAAAAAH//+AAAAAAAAAAAAAAAAAB//+AAAAAAAA' +
  'AAAAAAAAAAf//gAAAAAAAAAAAAAAAAAH///AAAAAAAAAAAAAAAAP5////AAAAAAAAAAAAAAP//f///+AAAAAAAAAAAAA///3' +
  '////8AAAAAAAAAAAA///9/////4AAAAAAAAAAA////f/////4AAAAAAAAAAf///3//////wAAAAAAAAAP///9//4///+AAAA' +
  'AAAAAP////f/+B///wAAAAAAAAH////3//gB//8AAAAAAAAD//wH///8AD//AAAAAAAAA//gAA///gAH/wAAAAAAAAf/gAAP' +
  '//4AAP8AAAAAAAAP/gAAD//+AAAfAAAAAAAAD/wAAA///gAAAAAAAAAAAB/4AAA///8AAAAAAAAAAAAf8AAA////wAAAAAAA' +
  'AAAAP+AAA/////AAAAAAAAAAAD/gAA/////8AAAAAAAAAAB/wAAf/////gAAAAAAAAAAf8AAP/////8AAAAAAAAAAH+AAH//' +
  '////gAAAAAAAAAB/gAD//////8AAAAAAAAAA/4AA///////gAAAAAAAAAP8AAf//////4AAAAAAAAAD/AAP///////AAAAAA' +
  'AAAA/wAD///////wAAAAAAAAAP8AA///////+AAAAAAAAAD/AAf///////gAAAAAAAAA/wAH///////4AAAAAAAAAP8AB///' +
  '////+AAAAAAAAAD/AAf///////wAAAAAAAAA/wAP///////8AAAAAAAAAP8AD////////AAAAAAAAAD/AA////////wAAAAA' +
  'AAAA/wAP///////8AAAAAAAAAP+AD////////AAAAAAAAAD/gA////////wAAAAAAAAAf4AP///////8AAAAAAAAAH+AD///' +
  '/////AAAAAAAAAB/wA////////wAAAAAAAAAf8AP///////8AAAAAAAAAD/AD////////AAAAAAAAAA/4A////////wAAAAA' +
  'AAAAP+AP///////8AAAAAAAAAB/gD////////AAAAAAAAAAf8A////////wAAAAAAAAAH/AP///////8AAAAAAAAAA/wD///' +
  '/////AAAAAAAAAAP8A////////wAAAAAAAAAD/gP///////8AAAAAAAAAA/4D////////AAAAAAAAAAH+A////////wAAAAA' +
  'AAAAB/gP///////8AAAAAAAAAAf4D////////AAAAAAAAAAH/A////////wAAAAAAAAAB/wP///////8AAAAAAAAAAf8D///' +
  '/////AAAAAAAAAAH/A////////wAAAAAAAAAB/wP///////8AAAAAAAAAAf8D////////AAAAAAAAAAH/A////////wAAAAA' +
  'AAAAB/wP///////8AAAAAAAAAAf8D////////AAAAAAAAAAD/A////////wAAAAAAAAAA/wP///////8AAAAAAAAAAf+D///' +
  '/////AAAAAAAAAAH/g////////wAAAAAAAAAB/4P///////8AAAAAAAAAAf+D////////AAAAAAAAAAH/g////////wAAAAA' +
  'AAAAB/4P///////8AAAAAAAAAAf8D////////AAAAAAAAAAH/g////////wAAAAAAAAAD/4P///////8AAAAAAAAAA/+D///' +
  '/////AAAAAAAAAAP/g////////wAAAAAAAAAD/8P///////8AAAAAAAAAB//D////////AAAAAAAAAAf/w////////wAAAAA' +
  'AAAAH/8P///////8AAAAAAAAAB//D////////AAAAAAAAAAf/w////////wAAAAAAAAAH/8P///////8AAAAAAAAAB//D///' +
  '/////AAAAAAAAAAf/wAAAAAAAAAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAAD//h///////+AAAAAAAAAA//4////////wAAAAA' +
  'AAAAP/+P///////8AAAAAAAAAD//j////////AAAAAAAAAA//4////////wAAAAAAAAAP/+P///////8AAAAAAAAAD//h///' +
  '/////AAAAAAAAAA//4f///////wAAAAAAAAAf/+H///////4AAAAAAAAAH//h///////+AAAAAAAAAB//4P///////gAAAAA' +
  'AAAAP/+D///////wAAAAAAAAAAAAAf//////4AAAAAAAAAAAAAD//////+AAAAAAAAAAAAAAf//////AAAAAAAAAAAAAAD//' +
  '////AAAAAAAAAAAAAAAP/////AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── safetycone — 비트맵 실루엣(128×128, 잉크 36.2%)
//   원본 마스크: safetycone.png
var MASK_SAFETYCONE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAD+AAAAAAAAAAAAAAAAAAAD/4AAAAAAAAAAAAAAAAAAB//AAAAAAAAAAAAAAAAAAAf/4AAAAAAAAA' +
  'AAAAAAAAAP/+AAAAAAAAAAAAAAAAAAD//wAAAAAAAAAAAAAAAAAA//8AAAAAAAAAAAAAAAAAAf//AAAAAAAAAAAAAAAAAAH/' +
  '/wAAAAAAAAAAAAAAAAAB//+AAAAAAAAAAAAAAAAAA///gAAAAAAAAAAAAAAAAAP//4AAAAAAAAAAAAAAAAAD///AAAAAAAAA' +
  'AAAAAAAAA///wAAAAAAAAAAAAAAAAAf//8AAAAAAAAAAAAAAAAAH///AAAAAAAAAAAAAAAAAB///4AAAAAAAAAAAAAAAAAf/' +
  '/+AAAAAAAAAAAAAAAAAP///gAAAAAAAAAAAAAAAAD///8AAAAAAAAAAAAAAAAA////AAAAAAAAAAAAAAAAAf///wAAAAAAAA' +
  'AAAAAAAAH///8AAAAAAAAAAAAAAAAB////gAAAAAAAAAAAAAAAAf///4AAAAAAAAAAAAAAAAP///+AAAAAAAAAAAAAAAAD//' +
  '//wAAAAAAAAAAAAAAAA////8AAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAH////wAAAAAAAAAAAAAAAB////+AAAAAAAA' +
  'AAAAAAAAf////gAAAAAAAAAAAAAAAH////4AAAAAAAAAAAAAAAD/////AAAAAAAAAAAAAAAA/////wAAAAAAAAAAAAAAAP//' +
  '//8AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAB/////4AAAAAAAAAAAAAAAf////+AAAAAAAAAAAAAAAH/////gAAAAAAA' +
  'AAAAAAAD/////8AAAAAAAAAAAAAAA//////AAAAAAAAAAAAAAAP/////wAAAAAAAAAAAAAAD/////+AAAAAAAAAAAAAAB///' +
  '///gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB//////wAAAAAAAAAAAAAA//////8AAAAAAA' +
  'AAAAAAAP//////AAAAAAAAAAAAAAD//////wAAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAH///' +
  '///4AAAAAAAAAAAAAB///////AAAAAAAAAAAAAA///////wAAAAAAAAAAAAAP//////8AAAAAAAAAAAAAD///////AAAAAAA' +
  'AAAAAAA///////4AAAAAAAAAAAAAf//////+AAAAAAAAAAAAAH///////gAAAAAAAAAAAAB///////8AAAAAAAAAAAAA////' +
  '////AAAAAAAAAAAAAP///////wAAAAAAAAAAAAD///////8AAAAAAAAAAAAA////////gAAAAAAAAAAAAP///////4AAAAAA' +
  'AAAAAAH///////+AAAAAAAAAAAAB////////gAAAAAAAAAAAAf///////8AAAAAAAAAAAAP////////AAAAAAAAAAAAD////' +
  '////wAAAAAAAAAAAA////////+AAAAAAAAAAAAP////////gAAAAAAAAAAAH////////4AAAAAAAAAAAB////////+AAAAAA' +
  'AAAAAAf////////wAAAAAAAAAAAH////////8AAAAAAAAAAAD/////////gAAAAAAAAAAB/////////8AAAAAAAAAAA/////' +
  '/////gAAAAAAAAAD///////////AAAAAAAAAD///////////8AAAAAAAAB////////////gAAAAAAAAf///////////4AAAA' +
  'AAAAP////////////AAAAAAAAD////////////wAAAAAAAA////////////+AAAAAAAAf////////////gAAAAAAAH//////' +
  '//////4AAAAAAAB////////////+AAAAAAAA/////////////wAAAAAAAP////////////8AAAAAAAD/////////////AAAA' +
  'AAAB/////////////4AAAAAAAf////////////+AAAAAAAH/////////////wAAAAAAD/////////////8AAAAAAA///////' +
  '///////AAAAAAAP/////////////wAAAAAAH/////////////+AAAAAAB//////////////gAAAAAAf/////////////4AAA' +
  'AAAP//////////////AAAAAAD//////////////wAAAAAA//////////////+AAAAAAf//////////////gAAAAAH///////' +
  '///////4AAAAAB///////////////AAAAAA///////////////wAAAAAP//////////////8AAAAAD///////////////AAA' +
  'AAA///////////////wAAAAAH//////////////8AAAAAB//////////////+AAAAAAP//////////////gAAAAAD///////' +
  '///////wAAAAAAP/////////////wAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── gloves — 비트맵 실루엣(128×128, 잉크 57.0%)
//   원본 마스크: gloves.png
var MASK_GLOVES = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAD4AAAAAAAAAAAAAAAAAAAD/AAAAAAAAAAAAAAAAAAAB/4AgAAAAAAAAAAAAAAAAAf/B/AAAAAAAAAAAA' +
  'AAAAAP/w/4AAAAAAAAAAAAAAAAD/8f+AAAAAAAAAAAAAAAAA////wAAAADgAAAAAAAAB8P///8AAAAD+AAAAAAAAB/j////A' +
  'AAAB/wAAAAAAAA//////wAA/A/+AAAAAAAAP/////+AAf4P/gAAAAAAAH//////gAP/H/4AAAAAAAB//////4AH/5/+AAAAA' +
  'AAAf/////+AB////gAAAAAAAH//////gAf///4PAAAAAAA//////4AH////P4AAAAAAP/////+AB//////AAAAAAD//////g' +
  'Af/////4AAAAHw//////8AP/////+AAAAH/f//////AD//////wAAAD////////wA//////8AAAA////////8AP//////AAA' +
  'AP////////AD//////gAAAH////////wA//////4AAAB////////8Af/////+AAAAP////////gH//////gwAAD////////4' +
  'B//////4/AAA////////+Af///////4AAP////////gH////////AAD////////4B////////4AAf///////+A////////+A' +
  'AH////////wP////////gAB////////8D////////4AAf////////A////////+AAD////////wP////////AAA////////8' +
  'D////////wAAP////////h////////8AAD////////4f////////AAAf///////+H////////gAAH////////h////////4A' +
  'AB////////4f///////+AAAf///////+P////////AAAH////////j////////wAAA/////////////////8AAAP///////8' +
  'f////////AAAD////////D////////wAAA////////w////////4AAAH///////8P///////+AAAB////////H////////gA' +
  'AAf///////5////////4AAAH////////////////8AAAB/////////////////AAAAP////////////////wAAAD////////' +
  '////////8AAAA////////////////+AAAAP////////////////gAAAD////////////////4AAAAf///////////////+AA' +
  'AAH////////////////AAAAB////////////////wAAAAf///////////////8AAAAD////////////////AAAAA////////' +
  '////////gAAAAP///////////////4AAAAD///////////////+AAAAAf///////////////gAAAAH///////////////wAA' +
  'AAB///////////////8AAAAAP///////////////AAAAAD///////////////wAAAAAf//////////////4AAAAAH///////' +
  '///////+AAAAAA///////////////gAAAAAP//////////////wAAAAAB//////////////8AAAAAAf//////////////AAA' +
  'AAAD//////////////gAAAAAAf/////////////4AAAAAAH/////////////8AAAAAAA//////////////AAAAAAAP//////' +
  '///////gAAAAAAB/////////////4AAAAAAAf////////////8AAAAAAAH/////////////AAAAAAAA/////////////gAAA' +
  'AAAAP////////////wAAAAAAAD////////////8AAAAAAAA////////////+AAAAAAAAP////////////gAAAAAAAH//////' +
  '//////wAAAAAAAB////////////8AAAAAAAAf////////////AAAAAAAAH////////////wAAAAAAAB////////////4AAAA' +
  'AAAAP///////////+AAAAAAAAD////////////wAAAAAAAA////////////+AAAAAAAAP////////////gAAAAAAAD//////' +
  '//////4AAAAAAAAf///////////+AAAAAAAAH////////////AAAAAAAAB////////////wAAAAAAAAf///////////8AAAA' +
  'AAAAH////////////AAAAAAAAB////////////wAAAAAAAAP///////////8AAAAAAAAD///wP//////+AAAAAAAAA//+AD/' +
  '//////gAAAAAAAAD/AAAf//////4AAAAAAAAAAAAAH//////+AAAAAAAAAAAAAA///////gAAAAAAAAAAAAAH//////4AAAA' +
  'AAAAAAAAAAf/////8AAAAAAAAAAAAAAAP/////AAAAAAAAAAAAAAAAH////wAAAAAAAAAAAAAAAAD///8AAAAAAAAAAAAAAA' +
  'AAAf/+AAAAAAAAAAAAAAAAAAAH/AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── dustmask — 비트맵 실루엣(128×128, 잉크 32.1%)
//   원본 마스크: dustmask.png
var MASK_DUSTMASK = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAPAAAAAAAAAAAAAAAAAAAAf+AAAAAAAAAAAAAAAAAAAP/4AAAAAAAAAAAAAAAAAAP//AAAAAAAAAAAAAAAAAAP/' +
  '/8AAAAAAAAAAAAAAAAAH///gAAAAAAAAAAAAAAAAH///+AAAAAAAAAAAAAAAAD////wAAAAAAAAAAAAAAAB/////AAAAAAAA' +
  'AAAAAAAB//w//4AAAAAAAAAAAAAAA//wD//AAAAAAAAAAAAAAA//wAP/8AAAAAAAAAAAAAAf/4PA//gAAAAAAAAAAAAAP/4H' +
  '4H/8AAAAAAAAAAAAAP/4H/gf/wAAAAAAAAAAAAH/8D/8D/+AAAAAAAAAAAAD/+D//wf/wAAAAAAAAAAAB/+B//+B/+AAAAAA' +
  'AAAAAA//B/gf4P/wAAAAAAAAAAAf/g/gB/B/+AAAAAAAAAAAP/gfwAP8H/wAAAAAAAAAAH/wfwPA/g/+AAAAAAAAAAD/4P4H' +
  '4H8H/wAAAAAAAAAB/8H8H/g/w/+AAAAAAAAAAf+H8D/+D+H/gAAAAAAAAAP/j+D//wfx/8AAAAAAAAAD/9/B//+D+//gAAAA' +
  'AAAAB///B///4P//4AAAAAAAAAf//g////B///AAAAAAAAAP//wf///4P//wAAAAAAB8D//4P////h//8D8AAAAH/9//8P//' +
  '//8P//v/4AAAH/////n/////n/////gAAD/////////////////8AAB//////////////////gAA//////////////////8A' +
  'AP//////////////////AAH+Af////////////+Af4AB/AA////////////8AD+AA/gAP////////////AAfwAPwAD//////' +
  '//////wAD8AD8AA////////////8AA/gB/AAP////////////AAP4AfgAD////////////wAB+AH4AA////////////8AAfg' +
  'B+AAH////////////AAH4AfgAB////////////gAB+AH4AAf///////////4AAfgB+AAH///////////+AAH4AfwAB//////' +
  '//////gAD+AD8AAP///////////4AA/AA/gAD///////////8AAfwAP8AA////////////AAP8AB/gAP///////////wAH+A' +
  'Af+AB///////////4AH/gAD/4Af//////////+AH/wAAf/8P///////////w//4AAD/////////////////8AAAf////////' +
  '////////+AAAB////////////////+AAAAH///////////////+AAAAAf//////////////+AAAAAAf/////////////4AAA' +
  'AAAAAP//////////AAAAAAAAAAB//////////gAAAAAAAAAAf/////////4AAAAAAAAAAD/////////8AAAAAAAAAAAf////' +
  '////+AAAAAAAAAAAH/////////gAAAAAAAAAAA/////////wAAAAAAAAAAAH////////4AAAAAAAAAAAA////////+AAAAAA' +
  'AAAAAAP////////AAAAAAAAAAAAB////////gAAAAAAAAAAAAP///////wAAAAAAAAAAAAB//gAAf/4AAAAAAAAAAAAAP/4A' +
  'AH/+AAAAAAAAAAAAAB//AAD/+AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAB//////wAAAAAAAAAAAAAAP/////wAAAAAAA' +
  'AAAAAAAB/////4AAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAB////+AAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAA//' +
  '//gAAAAAAAAAAAAAAAAH///gAAAAAAAAAAAAAAAAA///wAAAAAAAAAAAAAAAAAD//4AAAAAAAAAAAAAAAAAAf/4AAAAAAAAA' +
  'AAAAAAAAAB/4AAAAAAAAAAAAAAAAAAADwAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── haetae — 비트맵 실루엣(128×128, 잉크 48.4%)
//   원본 마스크: haetae.png
var MASK_HAETAE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAfAAAAAAAAAAAAAAAAAAAAP4AAAAAAAAAAAAAAAAAAAB/AAAAAAAAAAAAAAAAAAAAfwAQAAAAAAAAAAA' +
  'AAAAAAH+AOAAAAAAAAAAAAAAAAAB/gHwAAAAAAAAAAAAAAAAA/4B8AAAAAAAAAAAAAAAAA/8AfgAAAAAAAAAAAAAAAD//AH4' +
  'AAAAAAAAAAAAAAAD//wD+AAAAAAAAAAAAAAAB//+D/gAAAAAAAAAAAAAAA/////wAAAAAAAAAAAAAAAP////8AAAAAAAAAAA' +
  'AAAAH////+AAAAAAAAAAAAAAAD/////AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAH/////wAAAAAAAAAAAAAAD/////8A' +
  'AAAAAAAAAAAAAB/////+AAAAAAAAAAAAAAA//////gAAAAAAAAAMAAAAf/////8AAAAAAAAAHAAAAP//////AAAAAAAAAD4A' +
  'AAD//////wAAAAAAAAB+AAAA//////8AAAAAAAAB/gAA4///////gAAAAAAAD/4AAf///////4AAAAAAAf/+AAP////////A' +
  'AAAAAAf//gAD////////wAAAAAAf//4AA////////8AAAAAAf//+AAP////////AAAAAAP///gAD////////wAAAAAH///wA' +
  'Af///////8AAAAAB///8AAH////////AAAAAA///+AAB////////wAAAAAP///gAAP///////+AAAAAH///4AAD////////w' +
  'AAAAB////AAAf///////+AAAAAf///wAAD////////gAAAAH///+AAAf///////4AAAAD////wAAH///////+AAAAB////+A' +
  'AB////////gAAAA/////gAAf///////4AAAAP////4AAH///////8AAAAH/////AAA////////AAAAB/////wAAP///////g' +
  'AAAAP////8AAB///////4AAAAD/////AAAP//////+AAAAA/////wAAB///////gAAAAP////8AAAH//////4AAAAB/////A' +
  'AAAf/////+AAAAAP////wAAAD//////gAAAAD////8AAAAf/////wAAAAAf////AAAAH/////8AAAAAH////wAAAD/////+A' +
  'AAAAB////4AAAA//////AAAAAA////+AAAAf/////gAAAP//////gAAAP/////4AAB///////4AAAD/////+AAD///////8A' +
  'AAB//////gAf////////AAAAf/////+P/////////gAAAP////////////////wAAAD////////////////8AAAB////////' +
  '////////+AAAAf////////////////AAAAH////////////////gAAAD////////////////gAAAA////////////////wAA' +
  'AAP///////////////wAAAAD///////////////4AAAAA///////////////8AAAAAP///////////////AAAAAD////////' +
  '///////wAAAAA///////////////8AAAAAP///////////////AAAAAD///////////////wAAAAA///////////////8AAA' +
  'AAH///////////////AAAAAB///////////////wAAAAAf//////////////8AAAAAD///////////////AAAAAA////////' +
  '///////wAAAAAH//////////////8AAAAAA//////////////+AAAAAAP//////////////gAAAAAB//////////////4AAA' +
  'AAAP/////////////+BAAAAAB//////////////wYAAAAAf/////////////8OAAAAAH///////////////wAAAAB///////' +
  '////////8AAAAAf//////+////////AAAAAP//////8D///////4AAAAD//////wAf//////8AAAAA/////AAAD///////AA' +
  'AAAP////AAAA///////wAAAAD////wAAAH//wP//8AAAAB////8AAAA//8A//+AAAAAf////AAAAP/+AD//gAAAAH////gAA' +
  'AD//gAf/4AAAAB////4AAAA//4AD/+AAAAAf///8AAAAP/8AA//gAAAAH///+AAAAD//AAH/4AAAAD////AAAAB//gAB/+AA' +
  'AAA////AAAAAf/4AAf/wAAAAP///gAAAAP/8AAH/8AAAAH///wAAAAD/+AAB//AAAAB///4AAAAB//gAAf/wAAAB///+AAAA' +
  'A//wAAH/8AAAD////AAAAD//4AAB//AAAB////gAAAD//+AAD//4AAA////4AAAB///AAD//+AAAf///8AAAAf//wAB///gA' +
  'AH////AAAAP//4AA///4AAB////wAAAD//+AAP//+AAAf///8AAAA///gAD///AAAH////AAAAP//4AA///wAAA////wAAAB' +
  '//8AAP//8AAAAP//4AAAAAAAAAB//+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── safetymanager — 비트맵 실루엣(128×128, 잉크 35.8%)
//   원본 마스크: safetymanager.png
var MASK_SAFETYMANAGER = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAD/gAAAAAAAAAAAAAAAAAAB/4AAAAAAAAAAAAAAAAAAA//AAAAAAAAAAAAAAAAAAAP/wAAAAAAAAA' +
  'AAAAAAAAAz/8wAAAAAAAAAAAAAAAAA+//fAAAAAAAAAAAAAAAAAfv/34AAAAAAAAAAAAAAAAf7/9/gAAAAAAAAAAAAAAAP+/' +
  '/f8AAAAAAAAAAAAAAAH/v/3/AAAAAAAAAAAAAAAB/7/9/4AAAAAAAAAAAAAAA////f/AAAAAAAAAAAAAAAf///3/wAAAAAAA' +
  'AAAAAAAH///9/+AAAAAAAAAAAAAAB////f/gAAAAAAAAAAAAAA//////8AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAD///' +
  '///wAAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAB//////+AAAAAAA' +
  'AAAAAAB///////wAAAAAAAAAAAAAf//////+AAAAAAAAAAAAAH///////gAAAAAAAAAAAAB///////4AAAAAAAAAAAAAf///' +
  '///8AAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAAH////8AAAAAAAAAAAAAAA+rf/7Z+AAAAAAA' +
  'AAAAAAAf/3/+//gAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAB+Af/8B+AAAAAAAAAAAAAAfgAAAAfgAAAAAAAAAAAAAH4AA' +
  'AAH4AAAAAAAAAAAAAB4AAAAAeAAAAAAAAAAAAAAeAAAAAHgAAAAAAAAAAAAAHgAAAAB4AAAAAAAAAAAAAA4AAAAAcAAAAAAA' +
  'AAAAAAAOAAAAAHAAAAAAAAAAAAAAB4AAAAHgAAAAAAAAAAAAAAeAAAAB4AAAAAAAAAAAAAABwAAAAYAAAAAAAAAAAAAAAcAA' +
  'AAOAAAAAAAAAAAAAAADAAAADgAAAAAAAAAAAAAAA4AAABwAAAAAAAAAAAAAAAOAAAAcAAAAAAAAAAAAAAABwAAAOAAAAAAAA' +
  'AAAAAAAAeAAADgAAAAAAAAAAAAAAADgAABwAAAAAAAAAAAAAAAAcAAA8AAAAAAAAAAAAAAAAHgAAeAAAAAAAAAAAAAAAAA8A' +
  'APAAAAAAAAAAAAAAAAAHwAHwAAAAAAAAAAAAAAAAM/AH2AAAAAAAAAAAAAAAAHj//7wAAAAAAAAAAAAAAAB8f/5+AAAAAAAA' +
  'AAAAAAAA/w/4/wAAAAAAAAAAAAAAB/+AAf9gAAAAAAAAAAAAAD//wAP//AAAAAAAAAAAAAH///AP/v+AAAAAAAAAAAAP/3/4' +
  'H/7/8AAAAAAAAAAAf/+//b/9//4AAAAAAAAAAf//v///////gAAAAAAAAAf//9///////+AAAAAAAAAP///f///////wAAAA' +
  'AAAAH///75/7////+AAAAAAAAD///++f+f////wAAAAAAAB////3z/Pv///+AAAAAAAAf///9+/3L////wAAAAAAAP////n3' +
  '55////8AAAAAAAD/////9+//////gAAAAAAB/////////////4AAAAAAAf////////////+AAAAAAAH/////////////wAAA' +
  'AAAD//////////3//8AAAAAAA//////////4f//AAAAAAAP/////////4x//wAAAAAAD/////////4+H/+AAAAAAB///////' +
  '//g/8P/gAAAAAAf////////5//7/4AAAAAAH////////+f/+/+AAAAAAB/////////n//v/gAAAAAAf////////5//7/4AAA' +
  'AAAH////////+f/e/+AAAAAAB/////////n/Pv/gAAAAAAf////////5/z7/8AAAAAAP/////////cz+//AAAAAAD///////' +
  '//3h/v/wAAAAAA/////////98/z/8AAAAAAP/////////v/9//AAAAAAD/////////7/+f/wAAAAAA/////////+f/v/8AAA' +
  'AAAP/////////z/3//AAAAAAD/////////+f5//wAAAAAB//////////z8//+AAAAAAf/////////+c///gAAAAAH///////' +
  '///4f//4AAAAAB//////////////+AAAAAAf//////////////gAAAAAH//////////////4AAAAAB//////////////+AAA' +
  'AAAf//////////////wAAAAAH//////////////8AAAAAB///////////////AAAAAA///////////////wAAAAAP///////' +
  '///////8AAAAAB///////////////AAAAAAf//////////////gAAAAAD//////////////wAAAAAAH/////////////wAAA' +
  'AAAAP////////////AAAAAAAAAf//////////+AAAAAAAAAAf/////////4AAAAAAAAAAAP////////AAAAAAAAAAAAAD///' +
  '///4AAAAAAAAAAAAAAAP///4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── healthmanager — 비트맵 실루엣(128×128, 잉크 34.2%)
//   원본 마스크: healthmanager.png
var MASK_HEALTHMANAGER = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAD/gAAAAAAAAAAAAAAAAAAB/8AAAAAAAAAAAAAAAAAAAf/AAAAAAAAAAAAAAAAAAAH/wAAAAAAAAA' +
  'AAAAAAAABx/88AAAAAAAAAAAAAAAAB8f/PgAAAAAAAAAAAAAAAA/H/z+AAAAAAAAAAAAAAAAfx/8/gAAAAAAAAAAAAAAAP8f' +
  '/P8AAAAAAAAAAAAAAAH/n/z/gAAAAAAAAAAAAAAB/5/8/8AAAAAAAAAAAAAAA/+f/P/AAAAAAAAAAAAAAAP/n/z/4AAAAAAA' +
  'AAAAAAAH/5/9/+AAAAAAAAAAAAAAB/+f/f/wAAAAAAAAAAAAAA//n/3/8AAAAAAAAAAAAAAP/5/9//AAAAAAAAAAAAAAD/+f' +
  '/f/wAAAAAAAAAAAAAA//////+AAAAAAAAAAAAAAP//////gAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAB//////+AAAAAAA' +
  'AAAAAAA///////wAAAAAAAAAAAAAf//////+AAAAAAAAAAAAAH///////wAAAAAAAAAAAAB///////4AAAAAAAAAAAAAf///' +
  '///+AAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAD/////+AAAAAAAAAAAAAAC//////wAAAAAAAAAAAAAB//////+AAAAAAA' +
  'AAAAAAAf//////gAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAB+A//8B+AAAAAAAAAAAAAAfgAAAAfgAAAAAAAAAAAAAHgAA' +
  'AAB4AAAAAAAAAAAAABwAAAAAOAAAAAAAAAAAAAAeAAAAADgAAAAAAAAAAAAADgAAAAB4AAAAAAAAAAAAAA4AAAAAcAAAAAAA' +
  'AAAAAAAPAAAAAPAAAAAAAAAAAAAAB4AAAAHgAAAAAAAAAAAAAAHAAAABwAAAAAAAAAAAAAABwAAAA4AAAAAAAAAAAAAAAMAA' +
  'AAOAAAAAAAAAAAAAAADgAAAHAAAAAAAAAAAAAAAA4AAABwAAAAAAAAAAAAAAAHAAAAYAAAAAAAAAAAAAAABwAAAOAAAAAAAA' +
  'AAAAAAAAOAAAHAAAAAAAAAAAAAAAABwAABwAAAAAAAAAAAAAAAAeAAA4AAAAAAAAAAAAAAAADwAAcAAAAAAAAAAAAAAAAEeA' +
  'AOIAAAAAAAAAAAAAAADjwAPnAAAAAAAAAAAAAAAB8fAPj4AAAAAAAAAAAAAAAfh//x/AAAAAAAAAAAAAAAP8P/w/wAAAAAAA' +
  'AAAAAAAf/wfgf/gAAAAAAAAAAAAAf/+AAf/+AAAAAAAAAAAAA///wAP/38AAAAAAAAAAAB/9/+AH/7/4AAAAAAAAAAD//f/w' +
  'D/+//wAAAAAAAAAD//7/8A/////AAAAAAAAAD//+//w//3//8AAAAAAAAB///n/8P/5///gAAAAAAAA///9//n/+///8AAAA' +
  'AAAAf///vv5//P///gAAAAAAAP///73///3///8AAAAAAAD////f///5////AAAAAAAB////z///+////4AAAAAAAf//////' +
  '//////+AAAAAAAH/////////////wAAAAAAD/////////////8AAAAAAA//////////////AAAAAAAP/////////////wAAA' +
  'AAAD/////////////+AAAAAAA//////wD//////gAAAAAAP/////4Af/////4AAAAAAH/////+AH/////+AAAAAAB//////g' +
  'B//////gAAAAAAf/////4Af/////4AAAAAAH/////+AH/////+AAAAAAB//////gB//////gAAAAAAf/////4Af/////4AAA' +
  'AAAH/////+AH/////+AAAAAAB////+AAAAP////wAAAAAA/////AAAAD////8AAAAAAP////wAAAA/////AAAAAAD////8AA' +
  'AAP////wAAAAAA/////AAAAD////8AAAAAAP////wAAAA/////AAAAAAD////8AAAAP////wAAAAAA/////AAAAD////8AAA' +
  'AAAP////wAAAA/////AAAAAAD////+AAAAP////wAAAAAA//////4Af/////8AAAAAAP/////+AH//////gAAAAAH//////g' +
  'B//////4AAAAAB//////4Af/////+AAAAAAf/////+AH//////gAAAAAH//////gB//////4AAAAAB//////4Af/////+AAA' +
  'AAAf/////+AH//////gAAAAAH//////wB//////4AAAAAB//////////////+AAAAAAf//////////////wAAAAAH///////' +
  '///////4AAAAAB//////////////+AAAAAAP//////////////gAAAAAB//////////////wAAAAAAP/////////////wAAA' +
  'AAAAf////////////gAAAAAAAA////////////AAAAAAAAAA//////////8AAAAAAAAAAA/////////wAAAAAAAAAAAAP///' +
  '///8AAAAAAAAAAAAAAAf///4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── safetyboot — 비트맵 실루엣(128×128, 잉크 38.8%)
//   원본 마스크: safetyboot.png
var MASK_SAFETYBOOT = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAB/4AAAAAAAAAAAAAAAAAAD//AAAAAAAAAAAAAAAAAAB//4' +
  'AAAAAAAAAAAAAAAAAA//+AAAAAAAAAAAAAAAAAAP//gAAAAAAAAAAAAAAAAAD//4AAAAAAAAAAAAAAAAAA//+AAAAAAAAAAA' +
  'AAAAAA////wAAAAAAAAAAAAAD//////8AAAAAAAAAAAAAf///////AAAAAAAAAAAAAP///////wAAAAAAAAAAAAH///////8' +
  'AAAAAAAAAAAAB////////AAAAAAAAAAAAAf///////wAAAAAAAAAAAAH///////8AAAAAAAAAAAAB////////gAAAAAAAAAA' +
  'AAf///////4AAAAAAAAAAAAH///////+AAAAAAAAAAAAB////////gAAAAAAAAAAAAf///////4AAAAAAAAAAAAH///////+' +
  'AAAAAAAAAAAAB////////wAAAAAAAAAAAAf///////8AAAAAAAAAAAAH////////AAAAAAAAAAAAA////////wAAAAAAAAAA' +
  'AAP///////+AAAAAAAAAAAAB////////gAAAAAAAAAAAAf///////4AAAAAAAAAAAAP///////+AAAAAAAAAAAAD////////' +
  'wAAAAAAAAAAAA////////8AAAAAAAAAAAAP////////gAAAAAAAAAAAD////////4AAAAAAAAAAAA////////+AAAAAAAAAA' +
  'AAf////////wAAAAAAAAAAAH////////8AAAAAAAAAAAB/////////AAAAAAAAAAAA/////////8AAAAAAAAAAAP////////' +
  '/AAAAAAAAAAAD/////////4AAAAAAAAAAA//////////AAAAAAAAAAAP/////////4AAAAAAAAAAH//////////AAAAAAAAA' +
  'AB//////////4AAAAAAAAAA//////////+AAAAAAAAAAP//////////4AAAAAAAAAD///////////AAAAAAAAAB/////////' +
  '//8AAAAAAAAAf///////////gAAAAAAAAH///////////+AAAAAAAAB////////////4AAAAAAAA/////////////gAAAAAA' +
  'AP/////////////gP/AAAAD/////////////////AAAA/////////////////4AAAP/////////////////gAAH/////////' +
  '////////8AAB//////////////////gAAf/////////////////4AAH//////////////////AAB//////////////////wA' +
  'Af/////////////////+AAH//////////////////gAB//////////////////4AAf/////////////////+AAH/////////' +
  '/////////gAB//////////////////8AAf//////////////////AAP//////////////////4AH///////////////////A' +
  'B///////////////////wAf//////////////////8AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////8AH//////////////////+A' +
  'B///////////////////AAf/////gf///////////AAH/////gAf//////////AAB/////4AAf////////4AAAf////8AAB/' +
  '///////+AAAD/////AAAP///////8AAAAf5/7/wAAD//////8IAAAAB+P8f8AAAfz/n/n/AAAAAAAAAAAAAAAIf4/4wAAAAA' +
  'AAAAAAAAAAAAAGAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── safetyvest — 비트맵 실루엣(128×128, 잉크 61.2%)
//   원본 마스크: safetyvest.png
var MASK_SAFETYVEST = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAP8AAAAA/4AAAAAAAAAAAAf/4AAAA//gAAAAAAAAAAA///4AAB///AAAAAAAAAAD///////////gAAAA' +
  'AAAAH///////////+AAAAAAAAD////////////wAAAAAAAA////////////8AAAAAAAAP////////////AAAAAAAAD//////' +
  '//////wAAAAAAAA////////////8AAAAAAAAP////////////AAAAAAAAD////////////wAAAAAAAA////////////8AAAA' +
  'AAAAP////////////AAAAAAAAD////////////wAAAAAAAAf///////////4AAAAAAAAH///////////+AAAAAAAAB//////' +
  '//////gAAAAAAAAf///////////4AAAAAAAAH///////////+AAAAAAAAB////////////gAAAAAAAAf///////////4AAAA' +
  'AAAAH///////////+AAAAAAAAB////////////gAAAAAAAAf///////////4AAAAAAAAD///////////8AAAAAAAAA//////' +
  '//////AAAAAAAAAP///////////wAAAAAAAAD///////////8AAAAAAAAA////////////AAAAAAAAAP///////////wAAAA' +
  'AAAAD///////////8AAAAAAAAB////////////gAAAAAAAAf///////////4AAAAAAAAH///////////+AAAAAAAAB//////' +
  '//////gAAAAAAAAf///////////4AAAAAAAAH///////////+AAAAAAAAB////////////gAAAAAAAA////////////8AAAA' +
  'AAAAP////////////AAAAAAAAD////////////wAAAAAAAA////////////+AAAAAAAAf////////////gAAAAAAAH//////' +
  '//////4AAAAAAAD/////////////AAAAAAAA/////////////4AAAAAAAf////////////+AAAAAAAP/////////////wAAA' +
  'AAAH/////////////+AAAAAAD//////////////wAAAAAB///////////////AAAAAB///////////////4AAAAA////////' +
  '////////AAAAAP///////////////wAAAAD///////////////8AAAAA////////////////AAAAAP///////////////wAA' +
  'AAD///////////////8AAAAA////////////////AAAAAP///////////////wAAAAD///////////////8AAAAA////////' +
  '////////AAAAAP///////////////wAAAAD///////////////8AAAAA////////////////AAAAAP///////////////wAA' +
  'AAD///////////////8AAAAA////////////////AAAAAP///////////////wAAAAD///////////////8AAAAA////////' +
  '////////AAAAAP///////////////wAAAAB///////////////4AAAAAf//////////////+AAAAAH///////////////gAA' +
  'AAB///////////////4AAAAAf//////////////+AAAAAH///////////////gAAAAD///////////////8AAAAA////////' +
  '////////AAAAAP///////////////wAAAAD///////////////8AAAAA////////////////AAAAAP///////////////wAA' +
  'AAD///////////////8AAAAA////////////////AAAAAP///////////////wAAAAD///////////////8AAAAA////////' +
  '////////AAAAAH///////////////gAAAAB///////////////4AAAAAf//////////////+AAAAAH///////////////gAA' +
  'AAB///////////////4AAAAAf//////////////+AAAAAP///////////////wAAAAD///////////////8AAAAA////////' +
  '////////AAAAAP///////////////wAAAAD///////////////8AAAAA////////////////AAAAAP///////////////wAA' +
  'AAD///////////////8AAAAA////////////////AAAAAP///////////////wAAAAD///////////////8AAAAA////////' +
  '////////AAAAAP///////////////wAAAAD///////////////8AAAAA////////////////AAAAAP///////////////wAA' +
  'AAB///////////////4AAAAAP//////////////8AAAAAD///////n///////AAAAAAP//////gf//////AAAAAAA//////w' +
  'D//////AAAAAAAA/////wAP////8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── elephant — 비트맵 실루엣(128×128, 잉크 54.2%)
//   원본 마스크: elephant.png
var MASK_ELEPHANT = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAA//8AAAAAAAAAAAAAAAAAB///4AAAAAAAAAAAB//AAD////wAA//gAAAAAB//8AD/////AA//+AAA' +
  'AAD///4D/////8B///8AAAAB////h//////h////gAAAA////////////////8AAAA/////////////////wAAAP////////' +
  '////////+AAAH/////////////////gAAD/////////////////8AAB//////////////////gAAf/////////////////8A' +
  'AP//////////////////AAH//////////////////4AB//////////////////+AAf//////////////////wAP/////////' +
  '/////////8AD///////////////////AA///////////////////wAf//////////////////+AH///////////////////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'A///////////////////wAP//////////////////8AD///////////////////AAf//////////////////wAH/////////' +
  '/////////4AB//////////////////+AAP//////////////////AAD//////////////////wAAf/////////////////4A' +
  'AH/////////////////+AAA//////////////////AAAP/////////////////wAAB/////////////////4AAAf////////' +
  '////////+AAAD/////////////////AAAA/////////////////wAAAH////////////////4AAAB////////////////+AA' +
  'AAP////////////////AAAAD////////////////wAAAAf///////////////4AAAAD///////////////8AAAAA////////' +
  '////////AAAAAH///////////////gAAAAA///////////////wAAAAAH//////////////4AAAAAA//////////////8AAA' +
  'AAAH/////////////+AAAAAAAf////////////+AAAAAAAB////////////+AAAAAAAAP////////////AAAAAAAAD//////' +
  '//////wAAAAAAAA////////////8AAAAAAAAf////////////gAAAAAAAH////////////4AAAAAAAB////////////+AAAA' +
  'AAAAf////////////wAAAAAAAH////////////8AAAAAAAD/////////////AAAAAAAA/////////////wAAAAAAAP///8//' +
  '//P///8AAAAAAAD///+P///x////AAAAAAAB////j///8P///4AAAAAAAf///wf//+D///+AAAAAAAH///8H///g////gAAA' +
  'AAAB////B///4P///4AAAAAAAf///wf//+D///+AAAAAAAH///8H///g////gAAAAAAB////B///4P///4AAAAAAAf///wf/' +
  '/+D///+AAAAAAAH///8H///g////gAAAAAAB////B///4P///4AAAAAAAf///wf//+D///+AAAAAAAH///8H///g////gAAA' +
  'AAAB////B///4P///4AAAAAAAf///wP//8D///+AAAAAAAH///8D///A////gAAAAAAB////A///wP///4AAAAAAAf///wP/' +
  '/8D///+AAAAAAAH///8D///A////gAAAAAAB///+A///wH///4AAAAAAAP///gP//8B///8AAAAAAAD///4D///Af///AAAA' +
  'AAAAf//8A///wD///gAAAAAAAD//+AP//8Af//wAAAAAAAAf//AB//+AD//4AAAAAAAAAAAAAP//AAAAAAAAAAAAAAAAAAB/' +
  '/gAAAAAAAAAAAAAAAAAAH/gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── owl — 비트맵 실루엣(128×128, 잉크 30.4%)
//   원본 마스크: owl.png
var MASK_OWL = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAeAAAAH/gAAAB4AAAAAAAAH8AAA///wAAD+AAAAAAAAB/wAD////wAD/gAAAAAAAAf/AD/////AD/4AAAA' +
  'AAAAH/8H/////+D/+AAAAAAAAB////////////gAAAAAAAAf///////////4AAAAAAAAD///////////8AAAAAAAAA//////' +
  '//////AAAAAAAAAP///////////wAAAAAAAAB///////////8AAAAAAAAAf//////////+AAAAAAAAAH///////////gAAAA' +
  'AAAAA///////////wAAAAAAAAAP//////////8AAAAAAAAAD///////////AAAAAAAAAA//gH///+Af/wAAAAAAAAAf/AAP/' +
  '/8AA/+AAAAAAAAAH/AAA//8AAD/gAAAAAAAAD/gAAD/8AAAf8AAAAAAAAA/gAAAf+AAAB/AAAAAAAAAfwAAAD/AAAAP4AAAA' +
  'AAAAH8AAAAfgAAAD+AAAAAAAAD+AAAADwAAAAfwAAAAAAAA/AAAAAYAAAAD8AAAAAAAAPwAAAAGAAAAA/AAAAAAAAD4AAAAA' +
  'AAAAAHwAAAAAAAB+AAAAAAAAAAB+AAAAAAAAfgAA4AAABwAAfgAAAAAAAH4AA/gAAB/AAD4AAAAAAAB8AAf+AAA/4AA+AAAA' +
  'AAAAfAAP/gAAf/AAPgAAAAAAAHwAD/8AAP/wAD4AAAAAAAB8AA//AAD/+AA+AAAAAAAAfAAf/wAA//gAPgAAAAAAAHwAH/8A' +
  'AP/4AD4AAAAAAAB8AB//gAD/+AA+AAAAAAAAfAAP/4AB//AAPgAAAAAAAH4AD/+AAf/wAD4AAAAAAAB+AAf/wYP/4AB+AAAA' +
  'AAAAfgAD/////8AAfgAAAAAAAH8AAAB//gAAAP4AAAAAAAB/AAAAP/wAAAD+AAAAAAAA/4AAAB/8AAAB/wAAAAAAAf+AAAAf' +
  '/AAAAf+AAAAAAAH/wAAAH/gAAAP/wAAAAAAD/+AAAB/4AAAH/8AAAAAAB//wAAAf+AAAD//gAAAAAAf/+AAAH/gAAB//4AAA' +
  'AAAP//wAAB/4AAA///AAAAAAD//+AAA//AAAf//wAAAAAB///4AAP/4AAf//+AAAAAAf///wAP//AA////gAAAAAH////w//' +
  '7//////8AAAAAD//////gYH//////AAAAAA//////wAAf/////wAAAAAP/////gAAB/////+AAAAAD/////gAAAH/////gAA' +
  'AAB////AAAAAAAP///4AAAAAf///AAAAAAAA///+AAAAAH///gAAAAAAAH///gAAAAB///4AAAAAAAB///4AAAAAf//+AAAA' +
  'AAAAf//+AAAAAH///AAAAAAAAD///gAAAAB///wAAAAAAAA///4AAAAA///8AAAAAAAAP///AAAAAH///AAAAAAAAD///gAA' +
  'AAB///wAAAAAAAA///4AAAAAf//4AAAAAAAAH//+AAAAAH//+AAAAAAAAB///gAAAAB///gAAAAAAAAf//4AAAAAf//4AAAA' +
  'AAAAH//+AAAAAH//+AAAAAAAAB///gAAAAB///gAAAAAAAAf//4AAAAAP//4AAAAAAAAH//+AAAAAD//+AAAAAAAAB///AAA' +
  'AAA///wAAAAAAAA///wAAAAAP//8AAAAAAAAP//8AAAAAB///AAAAAAAAD//+AAAAAAf//wAAAAAAAA///gAAAAAH//8AAAA' +
  'AAAAP//4AAAAAA///gAAAAAAAH//8AAAAAAP//4AAAAAAAB///AAAAAAD//+AAAAAAAAf//wAAAAAAf//wAAAAAAAH//4AAA' +
  'AAAH//8AAAAAAAD//+AAAAAAA///AAAAAAAA///AAAAAAAP//4AAAAAAAf//wAAAAAAB//+AAAAAAAH//8AAAAAAAf//wAAA' +
  'AAAD//+AAAAAAAD//8AAAAAAA///AAAAAAAA///gAAAAAAf//wAAAAAAAH//4AAAAAAH//4AAAAAAAB///AAAAAAD//+AAAA' +
  'AAAAP//4AAAAAB///AAAAAAAAB//+AAAAAAf//wAAAAAAAAf//wAAAAAP//4AAAAAAAAD//+AAAAAH//8AAAAAAAAAf//wAA' +
  'AAD///AAAAAAAAAH//+AAAAB///gAAAAAAAAA///wAAAA///wAAAAAAAAAH//+AAAAf//4AAAAAAAAAA///wAAAP//8AAAAA' +
  'AAAAAH//+AAAH//+AAAAAAAAAAB///wAAD///gAAAAAAAAAAP///AAB///wAAAAAAAAAAB///4AB///4AAAAAAAAAAAP///g' +
  'B///8AAAAAAAAAAAB///8A///+AAAAAAAAAAAAH///5///+AAAAAAAAAAAAA////////AAAAAAAAAAAAAH///////gAAAAAA' +
  'AAAAAAAf//////gAAAAAAAAAAAAAD//////wAAAAAAAAAAAAAAP/////wAAAAAAAAAAAAAAAf////wAAAAAAAAAAAAAAAB//' +
  '//gAAAAAAAAAAAAAAAAA//8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── crab — 비트맵 실루엣(128×128, 잉크 34.2%)
//   원본 마스크: crab.png
var MASK_CRAB = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAB+AAAAAAB+AAAAAA' +
  'AAAAAH/+AAAAAH/+AAAAAAAAAAP//wAAAAD//8AAAAAAAAAP//8AAAAA///wAAAAAAAAP///AAAAAP///AAAAAAAAH//+AAA' +
  'AAAf//4AAAAAAAD//8AAAAAAA///AAAAAAAD//8AAAAAAAD//8AAAAAAB//8AEAAAAIAH//gAAAAAA//8AHAAAADgA//8AAA' +
  'AAAP/+ADwAAAA8AH//AAAAAAH//AD8AAAAPwA//4AAAAAD//gB/AAAAD+AH//AAAAAA//4B/wAAAA/4B//wAAAAAf/8D/4AA' +
  'AAH/wP/+AAAAAH//D/+AAAAB//D//gAAAAD//z//AAAAAP/8//8AAAAA/////gAAAAB/////AAAAAP////4AAAAAf////wAA' +
  'AAD////8AAAAAD////8AAAAB////+AAAAAAf////gAAAAf////AAAAAAD////4AAAAH////gAAAAAAf///+AAAAB////wAAA' +
  'AAAD////gAAAAf///4AAAAAAAf///4AAAAD///8AAAAAAAD///8AAAAA///8AD4AAHwAP///AAAAAH//8AD/gAH/AA///gAA' +
  'AAB//8AA44ABxwAD//4AAAAB//8AAcHAA4OAAP//gAAAAf/4AAHBwAODgAAf/4AAAAH/8AABwMADA4AAD/+AAAAD//AAAcDg' +
  'BwOAAA//wAAAA//4AAHA4AcDgAAf/8AAAAP/+AABwcADg4AAH//AAAAB//wAAOHAA4cAAD//gAAAAf/8AAB/z/v+AAA//4AA' +
  'AAD//AAAP////AAAH/8AAAAA//+ABx////jgAP//AAAAAH//wA//////8AP//gAAAAA///Af//////gP//wAAAAAH//4f///' +
  '///+H//4AAAAAA///P///////z//8AAAAAAH//3//////////+AAAAAAAP////////////8AAAAAAAD/////////////AAAA' +
  'AAAAf////////////gAAAAAAAD////////////wAAAAAAAwP///////////wMAAAAAP/w///////////w//AAAAP//D/////' +
  '/////w//8AAAH//8P/////////w///gAAD///z/////////8f//8AAB//////////////////gAA/5//////////////+f8A' +
  'Af8P//////////////D/gAP+A//////////////Af8AD/AB////////////+AD/AB/gAD///////////8AAfwAfwAAHP////' +
  '////88AAD+AH4A//5/////////f/8AfgB8Af//////////////gD4AeAf//////////////+AeAHgf///////////////4Hg' +
  'BwP///4///////x////A4AIH///+P//////8f///4EAAD/P/////////////z/AAAB/gAP//////////AAf4AAAf4AP/////' +
  '/////8AH+AAAP8AH///////////gA/wAAH/AB//+f////n//4AP+AAB/gA///n////5///AB/gAA/wA//+f/////5//8AP8A' +
  'AP4Af/4P+f//n/B//gB/AAD8AP8AP/gDwB/8AP8APwAA/AH/AH/wAAAP/gD/gD8AAPgB/gD/4AAAB/8Af4AfAAB4A/4A/8AA' +
  'AAP/AH/AHgAAeAP+AP+AAAAB/wB/wB4AAHAH/AH+AAAAAH+AP+AOAAAwB/wD8AAAAAAPwD/gDAAAAAf4A/gAAAAAH8Af4AAA' +
  'AAAH8AP4AAAAAB/AD+AAAAAAB/AD+AAAAAAfwA/gAAAAAAfgA/gAAAAAH8AH4AAAAAAH4AP8AAAAAD/AB+AAAAAAB+AD/AAA' +
  'AAA/wAfgAAAAAAPgA/4AAAAAP8AHwAAAAAAD4AH/AAAAAP+AB8AAAAAAAOAB/4AAAAH/gAcAAAAAAADgAP/AAAAD/wAHAAAA' +
  'AAAAAAB/wAAAA/4AAAAAAAAAAAAAH8AAAAP4AAAAAAAAAAAAAAHAAAADgAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── octopus — 비트맵 실루엣(128×128, 잉크 42.4%)
//   원본 마스크: octopus.png
var MASK_OCTOPUS = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAH/+AAAAAAAAAAAAAAAAAAP//8AAAAAAAAAAAAAAAAAP///4AAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAH//' +
  '//8AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAA//////8AAAAAAA' +
  'AAAAAAAP//////gAAAAAAAAAAAAAH//////8AAAAAAAAAAAAAD///////AAAAAAAAAAAAAA///////4AAAAAAAAAAAAAf///' +
  '///+AAAAAAAAAAAAAH///////wAAAAAAAAAAAAD///////8AAAAAAAAAAAAA////////AAAAAAAAAAAAAP///////4AAAAAA' +
  'AAAAAAH///////+AAAAAAAAAAAAB////////gAAAAAAAAAAAAf///////4AAAAAAAAAAAAH///////+AAAAAAAAAAAAB////' +
  '////gAAAAAAAAD/AAf///////8AD/AAAAAD/8AH////////AD/8AAAAB//gB////////wB//gAAAA//4Af///////4Af/8AA' +
  'AAf//AH///////+AP//gAAAH//wB////////gD//4AAAD+P8Af///////4A/x/AAAA+B/gH///////+AP4HwAAAPgP4B////' +
  '////gH+B+AAAHwD+AP///////wB/gPgAAB8A/gD///////8Af4D4AAAfAf4A////////AH+A+AAAHwH+AP///////wB/gPgA' +
  'AB8B/gB///////4Af8D4AAAfB/8Af//////+AP/g+AAAH9//AD///////gD/+/gAAA///4A///////wB///wAAAP//+AP///' +
  '///8Af//8AAAD///wD///////AP///AAAAf//+A/gf//gfwH///gAAAH///wP4D//wH8D///4AAAA////D8Af/8A/D///8AA' +
  'AAH///9/AH/+AP7///+AAAAA/////wA//AB/////AAAAAH////8AP/wAf////gAAAAA/////AD/8AH////wAAAAAH////wAf' +
  '+AB////4AAAAAAf///8AH/gAf///4AAAAAAB////AB/4AP///8AAAAAAAH///wAf+AD///8AAAAAAAAf//+AH/wB///4AAAA' +
  'AAEAAf//gD/8Af//gADAAAAf8AAP/8A//AP/+AAH+AAAf/4AB//gf/4H/+AAP/4AAP//gAf/+P//n//gAf//AAH///AP////' +
  '////8A///4AB///////////////////AA///////////////////wAP//////////////////8AH+H///////////////h/g' +
  'B+Af//////////////gH4AfgD//////////////wB+AH4Af/////////////4AfgB+AH/////////////+AH4AfgB///////' +
  '///////gB+AH8Af/////////////4A/gA/gP//8f//////h///AfwAP+H//8D//////4H//4f8AB////8B//////+A////+A' +
  'AP///+Af//////gD////AAB///+AP//////8Af///gAAP//+AH///////gB///wAAA//+AH///////+AH//wAAAA/4AH////' +
  '////4AH/AAAAAAAAH//+P/x///gAAAAAAAAAAP///D/8P///AAAAAAAAAAf///g//B///+AAAAAAAAAf///4P/wf///4AAAA' +
  'AAAAP///8D/8D////AAAAAAAAH////A//A////4AAAAAAAD////wP/wH////gAAAAAAB////4H/+B////4AAAAAAA////+B/' +
  '/gf////AAAAAAAP////Af/8D////4AAAAAAH+A//wP//A//wH+AAAAAAB/AH/8D//4P/8A/gAAAAAAfgB/+B//+B/+AH4AAA' +
  'AAAP4Af/gf//wf/gA/AAAAAAD+AH/wP//8D/4APwAAAAAAfgD/8D///Af/AD8AAAAAAH4B/+B///4H/4B+AAAAAAB/A//Af/' +
  '/+A//A/gAAAAAAf///gH+B/gH///4AAAAAAD///wB/AP4A///8AAAAAAAf//4AfwB/AH///AAAAAAAH//8AP4AfwA///gAAA' +
  'AAAAf/+AD+AH8AD//wAAAAAAAD/+AAfgB/AAf/wAAAAAAAAH4AAH8AfgAAfgAAAAAAAAAAAAB/AP4AAAAAAAAAAAAAAAAAf4' +
  'H+AAAAAAAAAAAAAAAAAD/j/AAAAAAAAAAAAAAAAAA///wAAAAAAAAAAAAAAAAAH//4AAAAAAAAAAAAAAAAAA//8AAAAAAAAA' +
  'AAAAAAAAAH/+AAAAAAAAAAAAAAAAAAAf+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── snail — 비트맵 실루엣(128×128, 잉크 57.2%)
//   원본 마스크: snail.png
var MASK_SNAIL = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAB/8AAAAAAAAAAAAAAAAAAP///AAAAAAAAAAAAAAAAA////+AAAAAAAAAA' +
  'AAAAAB/////8AAAAAAAAAAAAAAB//////wAAAAAAAAAAAAAB//////+AAAAAAAAAAAAAB///////4AAAAAAAAAAAAA//////' +
  '//AAAAAAAAAAAAA////////8AAAAAAAAAAAAf////////gAAAAAAAAAAAP////////8AAAAAAAAAAAP/////////gAAAAAAA' +
  'AAAH/////////8AAAAAAAAAAD//////////gAAAAAAAAAB//////////8AAAAAAAAAA///////////gAAAAAAAAAf///////' +
  '///8AAAAAAAAAP///n///////AAAAAAAAAD///AH//////4AAAAAAAAB///AAf//////AAAAAAAAA///gAB//////wAAAAAA' +
  'AAP//4AAH/////+AAAAAAAAH//8AAA//////gAAA+AAAD///AAAH/////8AAB/4AAA///wAAA//////AAA//gAAf//8AAAH/' +
  '////wAAf/8AAH///AAAB/////+AAP//AAD///4AAAP/////gAD//4AB///+AAAB/////8AB//+AAf///wAAAf/////AAf//g' +
  'AH////AAAD/////4AP//4AD////8AAA//////AH//+AA/////gAAP/////4D///gAP////8AAB//////3///4AH/////gAAf' +
  '/////////8AB/////4AAH/////////+AAf/////AAB//////////AAP/////wAAf/////////gAD/////+AAD/////////gA' +
  'A//////gAA/////////gAAP/////4AAP////////wAAH/////+AAD////////4AAB//////gAA////////8AAAf/////4AAP' +
  '////////AAAH/////+AAD////////wAAB//////gAA////////4AAAf/////4AAP///////+AAAH/////+AAH////////AAA' +
  'B//////gAB////////wAAAf/////wAAf///////8AAAH/////8AAH////////AAAB//////AAB////////wAAAf/////wAAf' +
  '///////8AAAH/////4AAP////////AAAB/////+AAD////////wAAAf/////AAA////////+AAAH/////wAAf////////wAA' +
  'A/////4AAH////////8AAAP////+AAD/////////gAAD/////AAA/////////8AAA/////gAAf/////////AAAH////4AAH/' +
  '////////4AAB////8AAD/////////+AAAP///+AAA//////////gAAD////gAAf/////////8AAA////wAAP//////////AA' +
  'AP///8AAH//////////wAAB////AAB//////////+AAAf///wAA///////////gAAD///+AAf//////////4AAA////gAP//' +
  '////////+AAAH///8AP///////////gAAB////gH///////////4AAAP///+H///////////+AAAB/////////////////AA' +
  'AAf////////////////wAAAD////////////////8AAAAf///////////////+AAAAH////////////////gAAAA////////' +
  '////////wAAAAH///////////////8AAAAA///////////////+AAAAAH///////////////AAAAAA///////////////gAA' +
  'AAAP//////////////4AAAAAB//////////////8AAAAAAf//////////////AAAAAAH//////////////wAAAAAB///////' +
  '///////8AAAAAAf//////////////AAAAAAH//////////////wAAAAAD//////////////+AAAAAB///////////////gAA' +
  'AAA///////////////8AAAAAf///////////////gAAAAf///////////////8AAAAP////////////////wAAAH////////' +
  '////////+AAAD/////////////////wAAB/////////////////+AAAf/////////////////gAAH/////////////////4A' +
  'AB/////////////////+AAAf/////////////////gAAD/////////////////wAAAP////////////////wAAAA////////' +
  '////////wAAAAA///////////////gAAAAAAA///////////4AAAAAAAAAAP/////////AAAAAAAAAAAAf///////+AAAAAA' +
  'AAAAAAB///////+AAAAAAAAAAAAAH//////+AAAAAAAAAAAAAAP/////8AAAAAAAAAAAAAAAf////wAAAAAAAAAAAAAAAAH/' +
  '/8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── bee — 비트맵 실루엣(128×128, 잉크 29.3%)
//   원본 마스크: bee.png
var MASK_BEE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAOAAAAAABwAAAAAAAAAAAAP4AAAAAB/AAAAAAAAAAAAH/AAA' +
  'AAA/4AAAAAAAAAAAD/4AAAAAf/AAAAAAAAAAAA//AAAAAP/wAAAAAAAAAAAP/4AAAAH/8AAAAAAAAAAAD//gAAAH//AAAAAA' +
  'AAAAAA//8AAAD//wAAAAAAAAAAAP//gAAB//8AAAAAAAAAAAB//8AAA//+AAAAAAAAAAAAPw/AAAPw/AAAAAAAAAAAAAwD4A' +
  'AHwDAAAAAAAAAAAAAAAfAAD4AAAAAAAAAAAAAAAAHwAA+AAAAAAAAAAAAAAAAA+AAfAAAAAAAAAAAAAAAAAHwAPgAAAAAAAA' +
  'AAAAAAAAB///4AAAAAAAAAAAAAAAAAf//+AAAAAAAAAAAAAAAAAH///gAAAAAAAAAAAAAAAAB///4AAAAAAAAAAAAAAAAB//' +
  '//gAAAAAAAAAAAAAAAAf///8AAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAH////4AAAAAAAAAB/8AAAD/////AAAA/+AA' +
  'AB//4AAA/////wAAB//4AAA///gAAf////+AAB///AAA///+AAH/////gAB///8AAP+P/4AD/////8AB//H/AAH8AP/AA///' +
  '///AA/8AP4AD8AA/8AP/////wA/8AA/AA+AAD/gD/////8Af8AAHwAPgAAP8A//////AP8AAB8AHwAAB/gP/////wH+AAAPg' +
  'B8AAAP8H/////+D/AAAD4AfAAAA/h//////h/AAAA+AHwAAAH/////////gAAAPgB8AAAA/////////wAAAD4AfAAAAH////' +
  '////4AAAA+AH4AAAA////////8AAAAfgB+AAAAP////////AAAAH4AfwAAAB////////gAAAD+AD+AAAAP///////wAAAB/A' +
  'A/wAAAB///////4AAAA/wAP/AAAAP//////+AAAA/8AD/8AAAD///////AAAA//AAf/4AAAf/B/4P/gAAB//gAH//4AAH/wA' +
  'AD/4AAH//4AA///+AD/8AAA//AB///8AAP8AP/5//AAAP/5//AD/AAB+AAf///4AAH///+AAfgAAPgAA////AAD///8AAHwA' +
  'AD8AAD///4AB///8AAD8AAAfAAAH///wD///4AAA+AAAD4AAA////////8AAAfAAAAfAAAP////////AAAPgAAAD4AAD////' +
  '////wAAHwAAAAfgAB////////+AAH4AAAAD+AB/////////4AH8AAAAAP+P//////////8f8AAAAAB//////////////+AAA' +
  'AAAH/////////////+AAAAAAAP////////////8AAAAAAAAfx////////+P4AAAAAAAAAAP////////AAAAAAAAAAAAD////' +
  '////wAAAAAAAAAAAA////////8AAAAAAAAAAAAf////////gAAAAAAAAAAAP////////8AAAAAAAAAAAD/+P///x//AAAAAA' +
  'AAAAAB//AP//AP/4AAAAAAAAAAA//gAAAAB//AAAAAAAAAAAPz4AAAAAf/wAAAAAAAAAAH4eAAAAAHh+AAAAAAAAAAB8HgAA' +
  'AAB4PgAAAAAAAAAA/B8AAAAA+D8AAAAAAAAAAfwfAAAAAPg/gAAAAAAAAAP8H8AAAAP4P8AAAAAAAAAH/B/wAAAP+D/gAAAA' +
  'AAAAD/4f/wAA//x/8AAAAAAAAA////////////AAAAAAAAAP///////////wAAAAAAAAD///////////8AAAAAAAAA//////' +
  '//////AAAAAAAAAH///////////gAAAAAAAAAef////////ngAAAAAAAAAAD/+Af+Af/wAAAAAAAAAAAA//AAAAD/8AAAAAA' +
  'AAAAAAP/wAAAA//AAAAAAAAAAAAD9+AAAAfPwAAAAAAAAAAAA+HgAAAHh8AAAAAAAAAAAAfB8AAAD4PgAAAAAAAAAAAHwPwA' +
  'AD8D4AAAAAAAAAAAD+B/AAD+B/AAAAAAAAAAAB/gP/AP/Af4AAAAAAAAAAA/4D////wH/AAAAAAAAAAAf/Af///4D/4AAAAA' +
  'AAAAAH/wD///8A/+AAAAAAAAAAB/8Af//+AP/gAAAAAAAAAAf+AD///AB/4AAAAAAAAAAH/gAf//gAf+AAAAAAAAAAB/wAD/' +
  '/wAD/gAAAAAAAAAAPwAAf/wAAPwAAAAAAAAAAAwAAB/4AAAwAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAAAYAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── frog — 비트맵 실루엣(128×128, 잉크 45.3%)
//   원본 마스크: frog.png
var MASK_FROG = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAP/AAAAAP/AAAAAAAAAAAAP/8AAAAP/8AAAAAAAAAAAH//wAAAP//wAAAAA' +
  'AAAAAD//+AAAH//8AAAAAAAAAAB///wAAD///gAAAAAAAAAA/B//AAD/+D8AAAAAAAAAAPgH/////+AfAAAAAAAAAAHwA///' +
  '///AD4AAAAAAAAAB8AH/////gAeAAAAAAAAAAeAA/////wAHgAAAAAAAAAHgAP////8AB4AAAAAAAAAB4AB/////AAOAAAAA' +
  'AAAAAcAAf////gADgAAAAAAAAAPAAH////4AA8AAAAAAAAADwAB/////AAPAAAAAAAAAA8AA/////wADwAAAAAAAAAPgAf//' +
  '//+AA8AAAAAAAAAD4AH/////gAfAAAAAAAAAA+AH/////+AHwAAAAAAAAAfwD//////wD+AAAAAAAAAH/H///////j/gAAAA' +
  'AAAAD///////////8AAAAAAAAB////////////gAAAAAAAA////////////8AAAAAAAAP////////////AAAAAAAAH//////' +
  '//////4AAAAAAAB////////////+AAAAAAAA/////////////wAAAAAAAP////////////8AAAAAAAD/////////////gAAA' +
  'AAAA/////////////4AAAAAAAf////////////+AAAAAAAH/////////////gAAAAAAB/////////////4AAAAAAAf//////' +
  '//////+AAAAAAAD/////////////gAAAAAAA/////////////wAAAAAAAP////////////8AAAAAAAD/////////////AAAA' +
  'AAAAf////////////gAAAAAAAH////////////4AAAAAA/A////////////8D8AAAB/+f////////////n/4AAA/////////' +
  '/////////AAAf/////////////////4AAP//////////////////AAH//////////////////4AB//////////////////+A' +
  'A///////////////////wAP//////////////////8AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4Af//////////////////+AD///////////////////AA///////////////////wAP/////////' +
  '/////////8AD///////////////////AAf//////////////////gAH//////////////////4AA///////+AAB///////8A' +
  'AH//////wAAAA///////AAB//////gAAAAB//////gAAP/////gAAAAAH/////wAAD/////gAAAAAAf////8AAAf////wAAA' +
  'AAAD////+AAAD////4AAAAAAAf////AAAAf///8AAAAAAAD////wAAAD////AAAAAAAA////4AAAA////wAAAAAAAP///8AA' +
  'AAH///8AAAAAAAD///+AAAAA////AAAAAAAA////AAAAAH///wAAAAAAAP///gAAAAA///8AAAAAAAD///wAAAAAH///gAAA' +
  'AAAB///4AAAAAA///4AAAAAAAf//8AAAAAAH///AAAAAAAP//+AAAAAAB///wAAAAAAD///gAAAAAB///+AAAAAAB///+AAA' +
  'AAD////wAAAAAA////8AAAAD////+AAAAAAf////wAAAD/////wAAAAAP/////AAAB/////+AAAAAH/////4AAA//////wAA' +
  'AAD//////AAAP/////+AAAAB//////wAAH//////wAAAA//////+AAB///////AAAA///////gAAf//////+AAB///////4A' +
  'AD/////////////////8AAA//////////////////AAAH/////////////////gAAAP////////////////AAAAAB/gP/5//' +
  '//j/8B/wAAAAAAAAAHAAAAAADwAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── swan — 비트맵 실루엣(128×128, 잉크 43.1%)
//   원본 마스크: swan.png
var MASK_SWAN = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAP/AAAAAAAAAAAAAAAAAAAf/+AAAAAAAAAAAAAAAAAAf//4AAAAAAAAAAAAAAAAAP///AAAAAAAAA' +
  'AAAAAAAAH///4AAAAAAAAAAAAAAAAH////gAAAAAAAAAAAAAAAB////4AAAAAAAAAAAAAAAA/////AAAAAAAAAAAAAAAAf//' +
  '//4AAAAAAAAAAAAAAAH////+AAAAAAAAAAAAAAAD/////wAAAAAAAAAAAAAAA/////+AAAAAAAAAAAAAAAf/////gAAAAAAA' +
  'AAAAAAAH/////4AAAAAAAAAAAAAAB//////AAAAAAAAAAAAAAAf/////wAAAAAAAAAAAAAAP/////8AAAAAAAAAAAAAAB///' +
  '///AAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////4AAAAAAA' +
  'AAAAAAAD/////+AAAAAAAAAAAAAAA//////gAAAAAAAAAAAAAAP/////4AAAAAAAAAAAAAAD/////+AAAAAAAAAAAAAAA///' +
  '///gAAAAAAAAAAAAAAP/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB//////AAAAAAAAAAAAAAAf/4f//wAAAAAAA' +
  'AAAAAAAH/8D//8AAAAAAAAAAAAAAD/+A///AAAAAAAAAAAAAAA//AP//gAAAAAAAAAAAAAAP/gD//4AAAAAAAAAAAAAAH/wB' +
  '//+AAAAAAAAAAAAAAB/8A///AAAAAAAAAAAAAAA/+AP//wAAAAAAAAAAAAAAP/AH//8AAAAAAAAAAAAAAD/gD//+AAAAAAAA' +
  'AAAAAAA/gB///gAAAAAAAAAAAAAAPwAf//wAAAAAAAAAAAAAAAgAP//8AAAAAAAAAAAAAAAAAH//+AAAAAAAAAAAAAgAAAD/' +
  '//gAADAAAAAAAAA/gAAB///wAAP8AAAAAAAAf+AAA///8AAP/gAAAAAAAP/4AAP//+AAP/8AAAAAAAH//AAH///gAH//gAAA' +
  'AAAD//4AD///wAH//8AAAAAAA///AB///8AD///AAAAAAAf//8Af///AB///4AAAAAAP///AP///gA////AAAAAAD///4H//' +
  '/4Af///wAAAAAB////B///+AP///+AAAAAAf///4////gH////gAAAAAH////////4D////8AAAAAD////////+A/////AAA' +
  'AAA/////////gf////wAAAAAP////////8P////+AAAAAH///////////////gAAAAB///////////////4AAAAAf///////' +
  '///////+AAAAAH///////////////gAAAAD///////////////8AAAAA////////////////AAAAAP///////////////wAA' +
  'AAD///////////////8AAAAA////////////////AAAAAP///////////////wAAAAD///////////////8AAAAA////////' +
  '////////AAAAAP///////////////wAAAAD///////////////8AAAAA////////////////AAAAAP///////////////wAA' +
  'AAD///////////////8AAAAA////////////////AAAAAP///////////////wAAAAD///////////////8AAAAAf///////' +
  '////////AAAAAH///////////////gAAAAB///////////////4AAAAAf//////////////+AAAAAH///////////////gAA' +
  'AAA///////////////wAAAAAP//////////////8AAAAAD///////////////AAAAAAf//////////////gAAAAAH///////' +
  '///////4AAAAAA//////////////8AAAAAAP//////////////AAAAAAB//////////////gAAAAAAf/////////////4AAA' +
  'AAAD/////////////+AAAAAAA//////////////AAAAAAAH/////////////gAAAAAAB/////////////4AAAAAAAP//////' +
  '//////8AAAAAAAB////////////+AAAAAAAAP////////////AAAAAAAAB////////////wAAAAAAAAP///////////4AAAA' +
  'AAAAD///////////8AAAAAAAAAP//////////+AAAAAAAAAB///////////AAAAAAAAAAP//////////gAAAAAAAAAB/////' +
  '/////gAAAAAAAAAAP/////////wAAAAAAAAAAA/////////4AAAAAAAAAAAH////////4AAAAAAAAAAAAf///////8AAAAAA' +
  'AAAAAAB///////8AAAAAAAAAAAAAH//////8AAAAAAAAAAAAAAf/////8AAAAAAAAAAAAAAA/////4AAAAAAAAAAAAAAAA//' +
  '//gAAAAAAAAAAAAAAAAA//+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── umbrella — 비트맵 실루엣(128×128, 잉크 29.6%)
//   원본 마스크: umbrella.png
var MASK_UMBRELLA = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAA8AAAAAAAAAAAAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAAD/AAAAAAAAAA' +
  'AAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAP+AAAAAAAAAAAAAAAAAAB//+AAAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAf//' +
  '///AAAAAAAAAAAAAAA//////8AAAAAAAAAAAAAB///5///4AAAAAAAAAAAAB///8P///gAAAAAAAAAAAB////B///+AAAAAA' +
  'AAAAAB////gP///4AAAAAAAAAAA////wD////gAAAAAAAAAA////8Af///+AAAAAAAAAA////+AH////wAAAAAAAAAf////g' +
  'A////+AAAAAAAAAP////wAP////4AAAAAAAAH////8AB/////AAAAAAAAD/////AAf////4AAAAAAAD/////gAH/////AAAA' +
  'AAAB/////4AB/////4AAAAAAA///g/+AAP/A///AAAAAAAf//AP/gAD/wB//4AAAAAAP//AD/4AA/8AP//AAAAAAD//gA/+A' +
  'AP/AA//4AAAAAB//gAP/gAD/4AH//AAAAAA//wAD/4AA/+AAf/4AAAAAf/wAB/+AAP/gAD/+AAAAAH/4AAf/gAD/8AAf/wAA' +
  'AAD/8AAP/4AA//AAD/+AAAAB/+AAD/+AAP/wAAf/gAAAAf/AAB//gAD/+AAD/8AAAAP/wAAf/4AB//gAAf/AAAAH/4AAH/+A' +
  'Af/8AAD/4AAAB/8AAD//w+H//AAA//AAAA/+AAB//9////4AAH/wAAAP/gAAf///////AAA/+AAAH/wAAP///////wAAP/gA' +
  'AB/8AAH///////+AAB/4AAA/+AAD////////wAAf/AAAP/AAB////////+AAD/wAAD/wAA/////////wAA/8AAB/4AAf////' +
  '////+AAH/gAAf+AAP/////////wAB/4AAH/gAP//////////AAP+AAD/wAH//////////4AD/wAA/8AH///////////gA/8A' +
  'AP/AH///////////+AH/AAD/gP////////////8B/4AB/4f/////////////4f+AAf//////////////////gAH/////////' +
  '/////////4AB//////////////////+AAf//////////////////wAP//////////////////8AD///////////////////A' +
  'A///////////////////wAP//////////////////8AD///////////////////AA///////////////////wAP/////////' +
  '/////////8AD8B/////w//8P////+A/AA8AH/gH/wB/4A/+Af+ADwAOAA/gAfwAP8AD+AB/AAMADAAHwAD4AB/AAfAAHgABA' +
  'AAAA4AAcAAfwADgAAwAAAAAAAEAACAAH8AAQAAMAAAAAAAAAAAAAB/AAAAAAAAAAAAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAH' +
  '8AAAAAAAAAAAAAAAAAAAB/AAAAAAAAAAAAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAH8AAAAAAAAAAAAAAAAAAAB/AAAAAAAAAA' +
  'AAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAH8AAAAAAAAAAAAAAAAAAAB/AAAAAAAAAAAAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAH' +
  '8AAAAAAAAAAAAAAAAAAAB/AAAAAAAAAAAAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAH8AAAAAAAAAAAAAAAAAAAB/AAAAAAAAAA' +
  'AAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAH8AAAAAAAAAAAAAAAAAAAB/AAAAAAAAAAAAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAH' +
  '8AAAAAAAAAAAAAAAAAAAB/AAAAAAAAAAAAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAH8AAAAAAAAAAAAAAAAAAAB/AAAAAAAAAA' +
  'AAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAH8AAAAAAAAAAAAAAAAAAAB/AAAAAAAAAAAAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAP' +
  '8AAAAAAAAAAAAAAAAAAAD/AAAAAAAAAAAAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAAD/AeAAAAAAAA' +
  'AAAAAAAAAA/wPwAAAAAAAAAAAAAAAAAP8D+AAAAAAAAAAAAAAAAAD/B/gAAAAAAAAAAAAAAAAA/wf4AAAAAAAAAAAAAAAAAP' +
  '8H+AAAAAAAAAAAAAAAAAD/B/gAAAAAAAAAAAAAAAAA/4f4AAAAAAAAAAAAAAAAAP/P+AAAAAAAAAAAAAAAAAB///AAAAAAAA' +
  'AAAAAAAAAAf//wAAAAAAAAAAAAAAAAAD//4AAAAAAAAAAAAAAAAAA//+AAAAAAAAAAAAAAAAAAH//AAAAAAAAAAAAAAAAAAA' +
  'f/AAAAAAAAAAAAAAAAAAAB/AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── key — 비트맵 실루엣(128×128, 잉크 20.6%)
//   원본 마스크: key.png
var MASK_KEY = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAD/AAAAAAAAAAAAAAAAAAAH/+AAAAAAAAAAAAAAAAAAH//4AAAAAAAAAAAAAAAAAD///AAAAAAAAA' +
  'AAAAAAAAB///4AAAAAAAAAAAAAAAAA////AAAAAAAAAAAAAAAAAf///4AAAAAAAAAAAAAAAAP/w//AAAAAAAAAAAAAAAAD/w' +
  'D/4AAAAAAAAAAAAAAAB/4Af+AAAAAAAAAAAAAAAA/8AD/4AAAAAAAAAAAAAAD//AA//wAAAAAAAAAAAAAH//wAP//gAAAAAA' +
  'AAAAAAH//8AD//+AAAAAAAAAAAAH///AA///4AAAAAAAAAAAD///wAP///gAAAAAAAAAAB///8AD///8AAAAAAAAAAB////g' +
  'B////gAAAAAAAAAAf///8A////8AAAAAAAAAAP//////////gAAAAAAAAAH/8A////AP/4AAAAAAAAAD/8AD//+AA//AAAAA' +
  'AAAAA/+AAP//AAD/4AAAAAAAAAf+AAB//gAAf+AAAAAAAAAH/gAAP/wAAD/gAAAAAAAAD/wAAD/4AAA/8AAAAAAAAA/4AAAf' +
  '+AAAH/AAAAAAAAAP+AAAH/AAAB/wAAAAAAAAH/AAAA/wAAAP+AAAAAAAAB/wAAAP8AAAD/gAAAAAAAAf8AAAD/AAAA/4AAAA' +
  'AAAAH/AAAA/gAAAP+AAAAAAAAB/wAAAP4AAAD/gAAAAAAAAf8AAAD+AAAA/4AAAAAAAAH/AAAA/wAAAP+AAAAAAAAB/wAAAP' +
  '8AAAD/gAAAAAAAAf8AAAD/AAAA/4AAAAAAAAD/gAAB/wAAAf8AAAAAAAAA/4AAAf+AAAH/AAAAAAAAAP/AAAP/wAAD/wAAAA' +
  'AAAAB/4AAH/8AAB/4AAAAAAAAAf+AAB//gAA/+AAAAAAAAAD/4AA//8AAf/AAAAAAAAAA//AA/gfwAP/wAAAAAAAAAH/8A/w' +
  'D/AP/4AAAAAAAAAA////4Af///8AAAAAAAAAAH///8AD///+AAAAAAAAAAA////AA////AAAAAAAAAAAH///wAP///gAAAAA' +
  'AAAAAA///8AD///wAAAAAAAAAAAD///AB///wAAAAAAAAAAAAP//4Af//wAAAAAAAAAAAAAf//AP//AAAAAAAAAAAAAAAP/4' +
  'P/4AAAAAAAAAAAAAAAA////8AAAAAAAAAAAAAAAAP///+AAAAAAAAAAAAAAAAB////AAAAAAAAAAAAAAAAAH///AAAAAAAAA' +
  'AAAAAAAAA///gAAAAAAAAAAAAAAAAAf//8AAAAAAAAAAAAAAAAAf///wAAAAAAAAAAAAAAAAH///+AAAAAAAAAAAAAAAAD//' +
  '//gAAAAAAAAAAAAAAAA////4AAAAAAAAAAAAAAAAP///+AAAAAAAAAAAAAAAAB////gAAAAAAAAAAAAAAAAP///wAAAAAAAA' +
  'AAAAAAAAA///gAAAAAAAAAAAAAAAAAH//wAAAAAAAAAAAAAAAAAB//8AAAAAAAAAAAAAAAAAAf//AAAAAAAAAAAAAAAAAAH/' +
  '/wAAAAAAAAAAAAAAAAAB//8AAAAAAAAAAAAAAAAAAf//AAAAAAAAAAAAAAAAAAH//wAAAAAAAAAAAAAAAAAB//8AAAAAAAAA' +
  'AAAAAAAAAf//AAAAAAAAAAAAAAAAAAH//wAAAAAAAAAAAAAAAAAB//8AAAAAAAAAAAAAAAAAAf//AAAAAAAAAAAAAAAAAAH/' +
  '/wAAAAAAAAAAAAAAAAAB//8AAAAAAAAAAAAAAAAAAf//AAAAAAAAAAAAAAAAAAH//wAAAAAAAAAAAAAAAAAB//8AAAAAAAAA' +
  'AAAAAAAAAf//AAAAAAAAAAAAAAAAAAP//8AAAAAAAAAAAAAAAAAP///gAAAAAAAAAAAAAAAAD///8AAAAAAAAAAAAAAAAB//' +
  '////8AAAAAAAAAAAAAAP//////gAAAAAAAAAAAAAD//////4AAAAAAAAAAAAAAP/////+AAAAAAAAAAAAAAB//////gAAAAA' +
  'AAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB/////+AAAAAAAAAAAAAAAf///+AAAAAAAAAAAAAAAAH/' +
  '///gAAAAAAAAAAAAAAAB////4AAAAAAAAAAAAAAAAf///+AAAAAAAAAAAAAAAAH////gAAAAAAAAAAAAAAAB/////+AAAAAA' +
  'AAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/' +
  '////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH//4AAAAAAAAAAAAAAAAAB//+AAAAAAAAA' +
  'AAAAAAAAAf//AAAAAAAAAAAAAAAAAAD//wAAAAAAAAAAAAAAAAAA//4AAAAAAAAAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAAA/' +
  '+AAAAAAAAAAAAAAAAAAAD+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── lightbulb — 비트맵 실루엣(128×128, 잉크 30.6%)
//   원본 마스크: lightbulb.png
var MASK_LIGHTBULB = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAP/wAAAAAAAAAAAAAAAAAAf//wAAAAAAAAAAAAAAAAA////gAAAAAAAAAAAAAAAB////+AAAAAAAA' +
  'AAAAAAAB/////4AAAAAAAAAAAAAAA//////gAAAAAAAAAAAAAA//////8AAAAAAAAAAAAAAf//////wAAAAAAAAAAAAAf///' +
  '///+AAAAAAAAAAAAAP/wf////wAAAAAAAAAAAAH/wD////+AAAAAAAAAAAAD/wAf////wAAAAAAAAAAAB/4AD////+AAAAAA' +
  'AAAAAAf8AAf////wAAAAAAAAAAAP8AAH////8AAAAAAAAAAAH+AAB/////gAAAAAAAAAAD/AAAf////8AAAAAAAAAAA/gAAH' +
  '/////gAAAAAAAAAAf4AAD/////4AAAAAAAAAAH8AAA//////AAAAAAAAAAD+AAA//////wAAAAAAAAAA/gAAP/////+AAAAA' +
  'AAAAAfwAAH//////gAAAAAAAAAH8AAD//////4AAAAAAAAAD+AAB///////AAAAAAAAAA/gAB///////wAAAAAAAAAP4AA//' +
  '/////8AAAAAAAAAD8AAP///////gAAAAAAAAB/AAH///////4AAAAAAAAAfwAD///////+AAAAAAAAAH8AB////////gAAAA' +
  'AAAAB/AAf///////4AAAAAAAAAf4AP////////AAAAAAAAAH+AH////////wAAAAAAAAB/wD////////8AAAAAAAAAf/B///' +
  '//////AAAAAAAAAH///////////wAAAAAAAAB///////////8AAAAAAAAAf///////////AAAAAAAAAH///////////gAAAA' +
  'AAAAB///////////4AAAAAAAAAf//////////+AAAAAAAAAH///////////gAAAAAAAAA///////////4AAAAAAAAAP/////' +
  '/////8AAAAAAAAAD///////////AAAAAAAAAA///////////wAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAA' +
  'AAAAAP//////////gAAAAAAAAAD//////////wAAAAAAAAAA//////////4AAAAAAAAAAH/////////+AAAAAAAAAAA/////' +
  '/////AAAAAAAAAAAP/////////wAAAAAAAAAAB/////////4AAAAAAAAAAAP////////8AAAAAAAAAAAD////////+AAAAAA' +
  'AAAAAAf////////AAAAAAAAAAAAD////////wAAAAAAAAAAAAf///////4AAAAAAAAAAAAD///////8AAAAAAAAAAAAAf///' +
  '///+AAAAAAAAAAAAAD///////AAAAAAAAAAAAAA///////wAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAA//////8AAAAAAA' +
  'AAAAAAAP//////AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAA///' +
  '///AAAAAAAAAAAAAAAP/////wAAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAAf/////AAAAAAAAAAAAAAAH/////wAAAAAAA' +
  'AAAAAAAB/////8AAAAAAAAAAAAAAAf/////AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAB/////4AAAAAAAAAAAAAAAf//' +
  '//+AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAA/////wAAAAAAAAAAAAAAAP////4AAAAAAAAAAAAAAAB////+AAAAAAAA' +
  'AAAAAAAAf////gAAAAAAAAAAAAAAAH////4AAAAAAAAAAAAAAAB////+AAAAAAAAAAAAAAAAf////gAAAAAAAAAAAAAAAP//' +
  '//8AAAAAAAAAAAAAAAD/////AAAAAAAAAAAAAAAA/////wAAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAD/////AAAAAAAA' +
  'AAAAAAAA/////wAAAAAAAAAAAAAAAH////8AAAAAAAAAAAAAAAD/////AAAAAAAAAAAAAAAA/////wAAAAAAAAAAAAAAAP//' +
  '//8AAAAAAAAAAAAAAAD/////AAAAAAAAAAAAAAAA/////wAAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAD/////AAAAAAAA' +
  'AAAAAAAA/////wAAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAD/////AAAAAAAAAAAAAAAA/////wAAAAAAAAAAAAAAAH//' +
  '//8AAAAAAAAAAAAAAAB////+AAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAA////AAAAAAAAAAAAAAAAAD///gAAAAAAAA' +
  'AAAAAAAAAf//wAAAAAAAAAAAAAAAAAH//4AAAAAAAAAAAAAAAAAA//8AAAAAAAAAAAAAAAAAAH/+AAAAAAAAAAAAAAAAAAA/' +
  '/AAAAAAAAAAAAAAAAAAAD/AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── scissors — 비트맵 실루엣(128×128, 잉크 31.5%)
//   원본 마스크: scissors.png
var MASK_SCISSORS = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAD4AAAAAAAAAAAAAfAAAAAB/gAAAAAAAAAAAAf8AAAAB/8AAAAAAAAAAAAP/gAAAAf/gAAAAAAAAAAAH/8AA' +
  'AAP/8AAAAAAAAAAAD//AAAAD//gAAAAAAAAAAB//4AAAA//8AAAAAAAAAAA//+AAAAf//gAAAAAAAAAAf//gAAAD//8AAAAA' +
  'AAAAAH//4AAAA///gAAAAAAAAAD//+AAAAP//8AAAAAAAAAB///gAAAD///AAAAAAAAAA///wAAAAf//4AAAAAAAAAf//8AA' +
  'AAH///AAAAAAAAAP//+AAAAB///4AAAAAAAAH///gAAAAP///AAAAAAAAD///wAAAAD///4AAAAAAAA///8AAAAAf///AAAA' +
  'AAAAf//+AAAAAH///4AAAAAAAP///gAAAAA///+AAAAAAAH///wAAAAAP///wAAAAAAD///8AAAAAB///+AAAAAAB///+AAA' +
  'AAAf///wAAAAAA////gAAAAAD///+AAAAAAf///wAAAAAAf///wAAAAAH///8AAAAAAH///+AAAAAD///+AAAAAAA////gAA' +
  'AAB////AAAAAAAH///8AAAAA////wAAAAAAB////gAAAAf///4AAAAAAAP///8AAAAH///8AAAAAAAB////gAAAD///+AAAA' +
  'AAAAf///8AAAB////gAAAAAAAD////AAAA////wAAAAAAAAf///4AAAf///4AAAAAAAAD////AAAP///8AAAAAAAAAf///4A' +
  'AH////AAAAAAAAAH////AAB////gAAAAAAAAA////wAA////wAAAAAAAAAH///+AAf///4AAAAAAAAAA////wAP///8AAAAA' +
  'AAAAAH///+AH///+AAAAAAAAAAA////wB////AAAAAAAAAAAP///8A////gAAAAAAAAAAB////gf///wAAAAAAAAAAAP///8' +
  'P///4AAAAAAAAAAAB////j///8AAAAAAAAAAAAP///9///+AAAAAAAAAAAAB////////AAAAAAAAAAAAAP///////gAAAAAA' +
  'AAAAAAB///////wAAAAAAAAAAAAAP//////4AAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAA///' +
  '///AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAA/////wAAAAAAAAAAAAAAAH////4AAAAAAAAAAAAAAAA////8AAAAAAAA' +
  'AAAAAAAAH///+AAAAAAAAAAAAAAAAB////gAAAAAAAAAAAAAAAAf///4AAAAAAAAAAAAAAAAH///+AAAAAAAAAAAAAAAAA//' +
  '//gAAAAAAAAAAAAAAAAP///4AAAAAAAAAAAAAAAAH////AAAAAAAAAAAAAAAAD////4AAAAAAAAAAAAAAAB/////AAAAAAAA' +
  'AAAAAAAA/////4AAAAAAAAAAAAAAAf////+AAAAAAAAAAAAAAAP/////wAAAAAAAAAAAAAAH/////+AAAAAAAAAAAf+AD///' +
  '///wAf/AAAAAAA//+B//////+A//+AAAAAA///4///gf//5///4AAAAA///////wD///////gAAAAf//////4Af//////8AA' +
  'AAf//////+AD///////gAAAP///////AAf//////8AAAH///////gAD///////gAAD//gP///wAA////Af/8AAA//gAf//4A' +
  'AH///AA//gAAf/gAB//8AAA///AAH/4AAP/gAAP/+AAAH//AAAf/AAD/wAAB//gAAA//gAAD/wAB/4AAAP/wAAAP/wAAAf+A' +
  'Af+AAAD/4AAAB/8AAAH/gAH/AAAAf+AAAAP+AAAA/8AD/gAAAH/AAAAD/gAAAH/AA/4AAAA/wAAAA/wAAAB/wAP8AAAAP8AA' +
  'AAP8AAAAP+AH/AAAAD/AAAAD/AAAAD/gB/wAAAAf4AAAA/wAAAA/4Af8AAAAH+AAAAP4AAAAP+AH+AAAAB/gAAAH+AAAAD/g' +
  'B/gAAAAf4AAAB/gAAAA/4Af4AAAAH+AAAAf4AAAAP+AH+AAAAB/gAAAD/AAAAD/gB/gAAAAfwAAAA/wAAAA/4Af8AAAAP8AA' +
  'AAP8AAAAP+AH/AAAAD/AAAAD/AAAAD/gB/wAAAA/wAAAA/wAAAA/4AP8AAAAf4AAAAH+AAAAP8AD/gAAAH+AAAAB/gAAAH/A' +
  'A/4AAAD/gAAAAf8AAAB/wAH/AAAA/wAAAAD/gAAA/4AB/wAAAf8AAAAA/4AAAf+AAP+AAAP+AAAAAH/AAAH/AAD/wAAH/AAA' +
  'AAA/4AAD/wAAf/AAH/wAAAAAP/gAD/4AAH/8AH/4AAAAAB/+AD/8AAA//wH/8AAAAAAP/8D/+AAAH////+AAAAAAB/////AA' +
  'AA/////AAAAAAAP////gAAAH////AAAAAAAB////wAAAAf///gAAAAAAAH///4AAAAD///gAAAAAAAAf//4AAAAAH//gAAAA' +
  'AAAAB//4AAAAAAH+AAAAAAAAAAB/gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── hammer — 비트맵 실루엣(128×128, 잉크 30.4%)
//   원본 마스크: hammer.png
var MASK_HAMMER = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAPwAAAAAAAAAAAAAAAAAAAf+AAAAAAAAAAAAAAAAAAA//wAAAAAAAAAAAAAAAAAB//+AAAAAAAAA' +
  'AAAAAAAAD///wAAAAAAAAAAAAAAAAH////AAAAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAH///////gAAAAAAAAAAAAB///' +
  '/////AAAAAAAAAAAAA////////+AAAAAAAAAAAAP////////4AAAAAAAAAAAD/////////gAAAAAAAAAAA/////////+AAAA' +
  'AAAAAAAf/////////wAAAAAAAAAAH/////////+AAAAAAAAAAD//////////wAAAAAAAHwD//////////+AAAAAAAP/3////' +
  '///////wAAAAAAf/////////////+AAAAAAf//////////////wAAAAAf//////////////+AAAAA////////////////wAA' +
  'AAf///////////////+AAAAH////////////////gAAAD////////////8Af/8AAAA////////////4AA//AAAAH////////' +
  '///8AAB/4AAAB///////////+AAAH+AAAAf///////////gAAAfgAAAD///////////wAAAD4AAAA///////////8AAAAMAA' +
  'AAH///////////AAAAAAAAAB///////////wAAAAAAAAAP//////////8AAAAAAAAAD///////////AAAAAAAAAA////////' +
  '///gAAAAAAAAAH//////////4AAAAAAAAAB//////////8AAAAAAAAAAf//////////AAAAAAAAAAD//////////wAAAAAAA' +
  'AAA//////////8AAAAAAAAAAH//////////AAAAAAAAAAB//////////wAAAAAAAAAAf////Af///+AAAAAAAAAAD////AD/' +
  '///gAAAAAAAAAA////gAP///8AAAAAAAAAAH///wAAf///AAAAAAAAAAB///8AAD///4AAAAAAAAAAf//+AAAf//+AAAAAAA' +
  'AAAD///AAAD///wAAAAAAAAAA///AAAAf//8AAAAAAAAAAP/+AAAAH///gAAAAAAAAAB/+AAAAB///4AAAAAAAAAAf8AAAAA' +
  'P//+AAAAAAAAAAD4AAAAAD///wAAAAAAAAAAAAAAAAA///8AAAAAAAAAAAAAAAAAH///gAAAAAAAAAAAAAAAAB///4AAAAAA' +
  'AAAAAAAAAAAf///AAAAAAAAAAAAAAAAAD///4AAAAAAAAAAAAAAAAA///+AAAAAAAAAAAAAAAAAP///wAAAAAAAAAAAAAAAA' +
  'D///8AAAAAAAAAAAAAAAAAf///gAAAAAAAAAAAAAAAAH///4AAAAAAAAAAAAAAAAB////AAAAAAAAAAAAAAAAAf///wAAAAA' +
  'AAAAAAAAAAAD///+AAAAAAAAAAAAAAAAA////gAAAAAAAAAAAAAAAAP///8AAAAAAAAAAAAAAAAD////AAAAAAAAAAAAAAAA' +
  'Af///4AAAAAAAAAAAAAAAAH///+AAAAAAAAAAAAAAAAB////wAAAAAAAAAAAAAAAAf///8AAAAAAAAAAAAAAAAH////gAAAA' +
  'AAAAAAAAAAAA////8AAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAD////4AAAAAAAAAAAAAAAA////+AAAAAAAAAAAAAAA' +
  'AP////wAAAAAAAAAAAAAAAB////8AAAAAAAAAAAAAAAAf////AAAAAAAAAAAAAAAAH////4AAAAAAAAAAAAAAAB////+AAAA' +
  'AAAAAAAAAAAAP////wAAAAAAAAAAAAAAAD////8AAAAAAAAAAAAAAAA/////gAAAAAAAAAAAAAAAP////4AAAAAAAAAAAAAA' +
  'AB/////AAAAAAAAAAAAAAAAf////4AAAAAAAAAAAAAAAH////+AAAAAAAAAAAAAAAA/////wAAAAAAAAAAAAAAAP////8AAA' +
  'AAAAAAAAAAAAD/////gAAAAAAAAAAAAAAAf////4AAAAAAAAAAAAAAAH/////AAAAAAAAAAAAAAAB/////wAAAAAAAAAAAAA' +
  'AAf////+AAAAAAAAAAAAAAAD/////gAAAAAAAAAAAAAAA/////8AAAAAAAAAAAAAAAP/////AAAAAAAAAAAAAAAB/////4AA' +
  'AAAAAAAAAAAAAf////+AAAAAAAAAAAAAAAH/////AAAAAAAAAAAAAAAB/////wAAAAAAAAAAAAAAAP////8AAAAAAAAAAAAA' +
  'AAD////+AAAAAAAAAAAAAAAA/////gAAAAAAAAAAAAAAAH////wAAAAAAAAAAAAAAAB////4AAAAAAAAAAAAAAAAf///8AAA' +
  'AAAAAAAAAAAAAH///8AAAAAAAAAAAAAAAAA///8AAAAAAAAAAAAAAAAAP//8AAAAAAAAAAAAAAAAAB//4AAAAAAAAAAAAAAA' +
  'AAAP/wAAAAAAAAAAAAAAAAAAAfgAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── glasses — 비트맵 실루엣(128×128, 잉크 15.4%)
//   원본 마스크: glasses.png
var MASK_GLASSES = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAB///+AAAAAAB///+AAAAAf////+AAAAAD/////gAAA//////8AA' +
  'AAH//////AAB///////wAAAP//////+AA////////AAAP///////wAf///////8AAP///////+AH///////////////////g' +
  'B////AP////////gH///4Af//wAAB//////wAAB//+AH//wAAAH/////wAAAH//gB//wAAAAf////wAAAA//4Af/4AAAAD//' +
  '//wAAAAH/+AH/8AAAAAf///4AAAAA//gB//AAAAAD///+AAAAAH/4Af/gAAAAA////AAAAAB/+AH/4AAAAAH///wAAAAAP/g' +
  'A/8AAAAAB///4AAAAAD/wAH/AAAAAAf//+AAAAAA/4AA/wAAAAAD///gAAAAAP8AAH8AAAAAA///wAAAAAD+AAB/AAAAAAP/' +
  '/8AAAAAA/gAAfwAAAAAD///AAAAAAP4AAH8AAAAAA///wAAAAAD+AAB/AAAAAAP//8AAAAAA/gAAfwAAAAAD///AAAAAAP4A' +
  'AH8AAAAAA///wAAAAAD+AAB/AAAAAAP//8AAAAAA/gAAPwAAAAAD///AAAAAAPwAAD8AAAAAA///4AAAAAD8AAA/gAAAAAf/' +
  '/+AAAAAA/AAAP4AAAAAH///gAAAAAfwAAD+AAAAAB///4AAAAAH8AAAfgAAAAAf///AAAAAB+AAAH4AAAAAP///wAAAAAfgA' +
  'AB/AAAAAD/AP8AAAAAP4AAAPwAAAAB/gB/gAAAAD8AAAD+AAAAAfwAP4AAAAB/AAAA/gAAAAP8AD/AAAAAfwAAAH8AAAAD+A' +
  'AfwAAAAP4AAAB/gAAAB/gAH+AAAAH+AAAAP4AAAA/wAA/wAAAB/AAAAD/gAAAf8AAP+AAAA/wAAAAf8AAAP+AAB/wAAA/4AA' +
  'AAD/gAAH/AAAP+AAAf8AAAAA//AAP/gAAB/8AAf/AAAAAH//B//4AAAf/8D//gAAAAA/////8AAAD/////wAAAAAH////8AA' +
  'AAP////4AAAAAAf///+AAAAB////4AAAAAAD///+AAAAAH///8AAAAAAAH//+AAAAAAf//8AAAAAAAAf/+AAAAAAB//4AAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── envelope — 비트맵 실루엣(128×128, 잉크 42.1%)
//   원본 마스크: envelope.png
var MASK_ENVELOPE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAf//////////////////+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AH////AAAAAAAAAAB////g' +
  'B///8AAAAAAAAAAAD///4Af//+AAAAAAAAAAAAf//+AH///AAAAAAAAAAAAD///gB///gAAAAAAAAAAAAf//4Af//4AAAAAA' +
  'AAAAAAH//+AH//+AAAAAAAAAAAAA///gB///gAAAAAAAAAAAAP//4Af//4AAAAAAAAAAAAD//+AH//+AAAAAAAAAAAAA///g' +
  'B///gAAAAAAAAAAAAf//4Af//8AAAAAAAAAAAAH//+AH///AAAAAAAAAAAAD///gB///8AAAAAAAAAAAB///4Af///AAAAAA' +
  'AAAAAA///+AH///8AAAAAAAAAAAf///gB////AAAAAAAAAAAP///4Af///8AAAAAAAAAAH///+AH////gAAAAAAAAAD////g' +
  'B////8AAAAAAAAAB////4Af////gAAAAAAAAB////+AH////8AAAAAAAAA/////gB/////gAAAAAAAAf////4Af////8AAAA' +
  'AAAAP////+AH/////gAAAAAAAH/////gB/////8AAAAAAAD/////4Af/////gAAAAAAB/////+AH/////8AAAAAAA//////g' +
  'B//////gAAAAAAf/////4Af/////8AD//wAP/////+AH//////gD///AH//////gB//////+D///8H//////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4Af/////4P////8H/////+AH/////4A////8Af/////gB/////8AH///+AD/////4Af////+AB//' +
  '//AAf////+AH/////AAP///gAD/////gB/////gAA///wAAf////4Af////wAAAAAAAAD////+AH////4AAAAAAAAAP////g' +
  'B////4AAAAAAAAAB////4Af///8AAAAAAAAAAP///+AH///+AAAAAAAAAAB////gB////AAAAAAAAAAAP///4Af///gAAAAA' +
  'AAAAAB///+AH///wAAAAAAAAAAAP///gB///4AAAAAAAAAAAB///4Af//8AAAAAAAAAAAAP//+AH//+AAAAAAAAAAAAB///g' +
  'B///AAAAAAAAAAAAAP//4Af//wAAAAAAAAAAAAD//+AH//8AAAAAAAAAAAAAf//gB///AAAAAAAAAAAAAH//4Af//gAAAAAA' +
  'AAAAAAB//+AH//8AAAAAAAAAAAAAf//gB///AAAAAAAAAAAAAP//4Af//wAAAAAAAAAAAAD//+AH//+AAAAAAAAAAAAB///g' +
  'B///wAAAAAAAAAAAA///4Af///AAAAAAAAAAAAf//+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── gift — 비트맵 실루엣(128×128, 잉크 50.7%)
//   원본 마스크: gift.png
var MASK_GIFT = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAD8AAAAAAB/gAAAAAAAAAAH/4AAAAAB/+AAAAAAAAAAH//gAAAAB//4AAAAAAAAAD//8AAAAA///gAAAA' +
  'AAAAB///gAAAAf//8AAAAAAAAA///8AAAAP///gAAAAAAAAf///gAAAH///4AAAAAAAAP///8AAAD////AAAAAAAAH////gA' +
  'AB////4AAAAAAAB////8AAA////+AAAAAAAA/wH//gAAP/+A/wAAAAAAAP4AP/4AAH/8AH8AAAAAAAD+AB//AAD/+AA/AAAA' +
  'AAAB/AAH/wAA/+AAP4AAAAAAAfwAA/+AAf/AAD+AAAAAAAH+AAH/gAH/gAA/gAAAAAAB/gAA/8AD/wAAf4AAAAAAAf4AAP/A' +
  'A/8AAH+AAAAAAAH/AAB/4Af+AAD/gAAAAAAB/4AAP+AH/AAB/4AAAAAAAf/AAB/wD/gAA/+AAAAAAAD/4AAf+9/4AAf/AAAA' +
  'AAAA//AAD///8AAP/wAAAAAAAP/8AAf//+AAH/8AAAAAAAB//gAH///gAH/+AAAAAAAAP/+AA///wAH//AAAAAAAAB//8AP/' +
  '/8AH//gAAAAAAAAP//wB///AP//wAAAAAAAAB///w///wf//4AAAAAAAAAP//////////8AAAAAAAAAA//////////8AAAAA' +
  'AAAAAB/////////8AAAAAAAAAAAP////////8AAAAAAAAAAAH/////////gAAAAAAAAAAD/////////8AAAAAAAP//////P/' +
  '/8//////8AAAH//////B//+D//////gAAD//////gf//gP/////8AAA//////gD//wB//////AAAP/////wA//8AH/////wA' +
  'AD/////wAf//gA/////8AAA/////4AH//8AD/////AAAP////4AD4AfAAf////wAAD////4AB8AD4AB////8AAA////8AA/A' +
  'A/AAP////AAAP///+AAPgAHwAB////wAAD////gAH4AA+AAf///8AAA////+AD+AAPwAf////AAAP////4A/gAD8Af////wA' +
  'AD/////Af4AA/gP////8AAA/////4P+AAP8H/////AAAP////+D/gAD/B/////wAAD/////x/4AA/4/////8AAA/////+/+A' +
  'AP/P/////AAAP///////gAD///////wAAD///////4AA///////8AAAf//////+AAP//////+AAAD///////gAD///////AA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAB//////8A' +
  'AP//////gAAAA///////gAH//////8AAAAP//////4AB///////AAAAD//////+AAf//////wAAAA///////gAH//////8AA' +
  'AAP//////4AB///////AAAAD//////+AAf//////wAAAA///////gAH//////8AAAAP//////4AB///////AAAAD//////+A' +
  'Af//////wAAAA///////gAH//////8AAAAP//////4AB///////AAAAD//////+AAf//////wAAAA///////gAH//////8AA' +
  'AAP//////4AB///////AAAAD//////+AAf//////wAAAA///////gAH//////8AAAAP//////4AB///////AAAAD//////+A' +
  'Af//////wAAAA///////gAH//////8AAAAP//////4AB///////AAAAD//////+AAf//////wAAAA///////gAH//////8AA' +
  'AAP//////4AB///////AAAAD//////+AAf//////wAAAA///////gAH//////8AAAAP//////4AB///////AAAAD//////+A' +
  'Af//////wAAAA///////gAH//////8AAAAP//////4AB///////AAAAD//////+AAf//////wAAAA///////gAH//////8AA' +
  'AAP//////4AB///////AAAAD//////+AAf//////wAAAA///////gAH//////8AAAAP//////4AB///////AAAAD//////+A' +
  'Af//////wAAAA///////gAH//////8AAAAP//////4AB///////AAAAD//////+AAf//////wAAAA///////gAH//////8AA' +
  'AAP//////4AB///////AAAAD//////+AAf//////wAAAA///////gAH//////8AAAAP//////4AB///////AAAAD//////+A' +
  'Af//////wAAAA///////gAH//////8AAAAP//////4AB///////AAAAD//////+AAf//////wAAAA///////gAH//////8AA' +
  'AAP//////4AB///////AAAAB//////+AAf//////gAAAAf//////gAH//////4AAAAD//////4AB//////8AAAAAf/////+A' +
  'Af/////+AAAAAD//////gAH//////AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── apple — 비트맵 실루엣(128×128, 잉크 56.2%)
//   원본 마스크: apple.png
var MASK_APPLE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAB//4AAAAAAAAAAAAAAAAAH///8AAAAAAAAAAAAAAAAH////AAAAAAAAAAAAAAAAP////wAA' +
  'AAAAAAAAAAAAAH////8AAAAAAAAAAADwAAH////+AAAAAAAAAAAD+AAH/////gAAAAAAAAAAB/gAD/////4AAAAAAAAAAA/8' +
  'AB/////8AAAAAAAAAAAf/AA//////AAAAAAAAAAAH/4Af/////gAAAAAAAAAAA/+AH/////4AAAAAAAAAAAH/wD/////8AAA' +
  'AAAAAAAAA/8B/////+AAAAAAAAAAAAH/g//////gAAAAAAAAAAAB/4P/////wAAAAAAAAAAAAP+H/////4AAAAAAAAAAAAB/' +
  'x/////8AAAAAAAAAAAAAf//////+AAAAAAAAAAAAAD///////AAAAAAAAAAAAAA///////gAAAAAAAAAAAAAH//////gAAAA' +
  'AAAAAAAAAB//////gAAAAAAAAAAAAAAf/////gAAAAAAAAAAAAAAD/////wAAAAAAAAAAAAAAA/////+AAAAAAAAAAA/+AAP' +
  '/////gAAAAAAAAAB//8AD//////AAAAAAAAAD///4A//////+AAAAAAAAD////gP//////4AAAAAAAB/////D///////AAAA' +
  'AAAB/////////////4AAAAAAA//////////////AAAAAAAf/////////////8AAAAAAP//////////////AAAAAAH///////' +
  '///////4AAAAAD///////////////AAAAAA///////////////4AAAAAf//////////////+AAAAAH///////////////wAA' +
  'AAD///////////////8AAAAB////////////////gAAAAf///////////////4AAAAH////////////////AAAAD////////' +
  '////////wAAAA////////////////+AAAAf////////////////gAAAH////////////////4AAAB/////////////////AA' +
  'AAf////////////////wAAAP////////////////8AAAD/////////////////AAAA/////////////////wAAAP////////' +
  '////////+AAAD/////////////////gAAB/////////////////4AAAf////////////////+AAAH/////////////////gA' +
  'AB/////////////////4AAAf////////////////+AAAH/////////////////gAAB/////////////////4AAAf////////' +
  '////////+AAAH/////////////////gAAB/////////////////4AAAf////////////////+AAAD/////////////////gA' +
  'AA/////////////////wAAAP////////////////8AAAD/////////////////AAAA/////////////////wAAAP////////' +
  '////////8AAAB/////////////////AAAAf////////////////gAAAH////////////////4AAAB////////////////+AA' +
  'AAP////////////////gAAAD////////////////wAAAA////////////////8AAAAP////////////////AAAAB////////' +
  '////////wAAAAf///////////////4AAAAH///////////////+AAAAA////////////////gAAAAP///////////////wAA' +
  'AAD///////////////8AAAAAf///////////////AAAAAH///////////////gAAAAA///////////////4AAAAAP///////' +
  '///////8AAAAAB///////////////AAAAAAf//////////////gAAAAAD//////////////4AAAAAA//////////////8AAA' +
  'AAAH//////////////AAAAAAB//////////////gAAAAAAP/////////////4AAAAAAB/////////////8AAAAAAAf//////' +
  '//////+AAAAAAAD/////////////gAAAAAAAf////////////wAAAAAAAH////////////4AAAAAAAA////////////8AAAA' +
  'AAAAH////////////AAAAAAAAA////////////gAAAAAAAAP///////////wAAAAAAAAB///////////4AAAAAAAAAP/////' +
  '/////8AAAAAAAAAB//////////+AAAAAAAAAAP//////////AAAAAAAAAAA//////////gAAAAAAAAAAH/////////wAAAAA' +
  'AAAAAA/////////wAAAAAAAAAAAH////////4AAAAAAAAAAAAf///////4AAAAAAAAAAAAB///////8AAAAAAAAAAAAAH///' +
  '///4AAAAAAAAAAAAAAP/wAP/4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── icecream — 비트맵 실루엣(128×128, 잉크 24.1%)
//   원본 마스크: icecream.png
var MASK_ICECREAM = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAB///wAAAAAAAAAAAAAAAAB////gAAAAAAAAAAAAAAAD////+AAAAAAAA' +
  'AAAAAAAD/////4AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAB/wP///8AAAAAAAAAAAAAA/wB////gAAAAAAAAAAAAAfwAf' +
  '///8AAAAAAAAAAAAAPwAP////gAAAAAAAAAAAAH4AH////8AAAAAAAAAAAAD4AD/////gAAAAAAAAAAAB8AB/////8AAAAAA' +
  'AAAAAA+AA//////gAAAAAAAAAAAPAA//////4AAAAAAAAAAAHwAf//////AAAAAAAAAAAD4AP//////4AAAAAAAAAAA8AH//' +
  '////+AAAAAAAAAAAfAD/////x/wAAAAAAAAAAHgA/////4P8AAAAAAAAAAB4Af////+B/AAAAAAAAAAA8AP/////wP4AAAAA' +
  'AAAAAPAD/////8D+AAAAAAAAAADwB//////AfgAAAAAAAAAA8A//////4H8AAAAAAAAAAfAP//////A/AAAAAAAAAAHwH///' +
  '///wPwAAAAAAAAAB8D//////+H8AAAAAAAAAAfA///////7/AAAAAAAAAAH4/////////wAAAAAAAAAB//////////8AAAAA' +
  'AAAAAf//////////gAAAAAAAAAP//////////8AAAAAAAAAH///////////gAAAAAAAAD///////////8AAAAAAAAB//////' +
  '//////gAAAAAAAA////////////4AAAAAAAAP////////////AAAAAAAAD////////////wAAAAAAAB////////////8AAAA' +
  'AAAAff///////////AAAAAAAAH3///////////4AAAAAAAB8//Af///////+AAAAAAAAPH/AB////////AAAAAAAADweAAP/' +
  '//////wAAAAAAAA8AAAB//////88AAAAAAAAHAAAAP//4B/+OAAAAAAAAB4AAwB//wAH/DgAAAAAAAAPAB+AP/wAAGBwAAAA' +
  'AAAAB4B/wAfwAAAA8AAAAAAAAAPz/8AAAAAAAeAAAAAAAAAB///AAAAIAAOAAAAAAAAAAP//4AAAfwAHAAAAAAAAAAB///AA' +
  'Af/ABwAAAAAAAAAAf//8AAf/4A4AAAAAAAAAAD///8A///A8AAAAAAAAAAAf/////////AAAAAAAAAAAD4P///////AAAAAA' +
  'AAAAAAeAf////h/gAAAAAAAAAAAHwH////wDwAAAAAAAAAAAA8B/gAP+B8AAAAAAAAAAAAPg/8AD/weAAAAAAAAAAAAD8f/g' +
  'B/+PgAAAAAAAAAAAAf/z8A/P34AAAAAAAAAAAAH/4fgfh/8AAAAAAAAAAAAB/8D8PwP/AAAAAAAAAAAAAP+Afn4B/gAAAAAA' +
  'AAAAAAD/AD/8AP4AAAAAAAAAAAAAfgAf+AD+AAAAAAAAAAAAAH4AD/AA/AAAAAAAAAAAAAB/AA/gAPwAAAAAAAAAAAAAP4AP' +
  '8AH4AAAAAAAAAAAAAD+AH/gD+AAAAAAAAAAAAAA/wD/8B/gAAAAAAAAAAAAAH+B8fgfwAAAAAAAAAAAAAB/w+D8P8AAAAAAA' +
  'AAAAAAAP8fAfn+AAAAAAAAAAAAAAD//gD//gAAAAAAAAAAAAAA//wAf/4AAAAAAAAAAAAAAHv4AD+8AAAAAAAAAAAAAAB5+A' +
  'A/PAAAAAAAAAAAAAAAf/gAP/gAAAAAAAAAAAAAAD/8AH/4AAAAAAAAAAAAAAA//gD/+AAAAAAAAAAAAAAAH58B8fAAAAAAAA' +
  'AAAAAAAB8Pg+HwAAAAAAAAAAAAAAAeB8fA4AAAAAAAAAAAAAAADwP/geAAAAAAAAAAAAAAAA8B/wHgAAAAAAAAAAAAAAAPgP' +
  '4DwAAAAAAAAAAAAAAAB4D+A8AAAAAAAAAAAAAAAAeA/gOAAAAAAAAAAAAAAAADwf8HgAAAAAAAAAAAAAAAA8P/hwAAAAAAAA' +
  'AAAAAAAAPnx88AAAAAAAAAAAAAAAAB/4P/AAAAAAAAAAAAAAAAAf8B/gAAAAAAAAAAAAAAAAH+AP4AAAAAAAAAAAAAAAAA/A' +
  'B8AAAAAAAAAAAAAAAAAPgAfAAAAAAAAAAAAAAAAAB4AHgAAAAAAAAAAAAAAAAAfAD4AAAAAAAAAAAAAAAAAH4A+AAAAAAAAA' +
  'AAAAAAAAA+AfAAAAAAAAAAAAAAAAAAPwPwAAAAAAAAAAAAAAAAAB+D4AAAAAAAAAAAAAAAAAAfh+AAAAAAAAAAAAAAAAAAH8' +
  '/AAAAAAAAAAAAAAAAAAA//wAAAAAAAAAAAAAAAAAAP/8AAAAAAAAAAAAAAAAAAB/+AAAAAAAAAAAAAAAAAAAf/gAAAAAAAAA' +
  'AAAAAAAAAH/wAAAAAAAAAAAAAAAAAAA/8AAAAAAAAAAAAAAAAAAAP+AAAAAAAAAAAAAAAAAAAB/gAAAAAAAAAAAAAAAAAAAP' +
  'wAAAAAAAAAAAAAAAAAAAB4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── cupcake — 비트맵 실루엣(128×128, 잉크 52.2%)
//   원본 마스크: cupcake.png
var MASK_CUPCAKE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAcAAAAAAAAAAAAAAAAAAAAHwAAAAAAAAAAAAAAAAAAAB/AAAAAAAAAAAAAAAAAAAAf8AAAAAAAAA' +
  'AAAAAAAAAAP/wAAAAAAAAAAAAAAAAAAD//AAAAAAAAAAAAAAAAAAB//4AAAAAAAAAAAAAAAAAA///AAAAAAAAAAAAAAAAAAf' +
  '//8AAAAAAAAAAAAAAAAAP///gAAAAAAAAAAAAAAAAH///8AAAAAAAAAAAAAAAAH////gAAAAAAAAAAAAAAAP////8AAAAAAA' +
  'AAAAAAAAf/////AAAAAAAAAAAAAAA//////4AAAAAAAAAAAAAA//////+AAAAAAAAAAAAAA///////wAAAAAAAAAAAAA////' +
  '///8AAAAAAAAAAAAA////////gAAAAAAAAAAAAf////////AAAAAAAAAAAAf////////+AAAAAAAAAAAP/////////wAAAAA' +
  'AAAAAH//////////AAAAAAAAAAD//////////4AAAAAAAAAA///////////AAAAAAAAAAf//////////4AAAAAAAAAP/////' +
  '//////AAAAAAAAAD///////////4AAAAAAAAB////////////AAAAAAAAAf///////////wAAAAAAAAH///////////+AAAA' +
  'AAAAD////////////gAAAAAAAA////////////4AAAAAAAAP////////////AAAAAAAAD////////////wAAAAAAAD//////' +
  '//////8AAAAAAAD/////////////AAAAAAAD/////////////4AAAAAAB//////////////AAAAAAA//////////////8AAA' +
  'AAAf//////////////gAAAAAf//////////////8AAAAAH///////////////gAAAAD///////////////8AAAAB////////' +
  '////////gAAAAf///////////////8AAAAP////////////////AAAAD////////////////4AAAB////////////////+AA' +
  'AAf////////////////wAAAP////////////////8AAAD/////////////////AAAA/////////////////4AAAP////////' +
  '////////+AAAD/////////////////gAAA/////////////////4AAAf////////////////+AAAH/////////////////gA' +
  'AA/////////////////4AAAP////////////////+AAAD/////////////////AAAA/////////////////wAAAP////////' +
  '////////8AAAB////////////////+AAAAf////////////////gAAAH////////////////4AAAA////////////////8AA' +
  'AAH///////////////+AAAAB////////////////gAAAAP///////////////wAAAAA///////////////4AAAAAH///////' +
  '///////4AAAAAAf/////////////4AAAAAAH/////////////+AAAAAAA//////////////AAAAAAAP/////////////wAAA' +
  'AAAD/////////////8AAAAAAAf/////////////AAAAAAAH/////////////gAAAAAAB/////////////4AAAAAAAf//////' +
  '//////+AAAAAAAD/////////////AAAAAAAA/////////////wAAAAAAAP////////////8AAAAAAAD/////////////AAAA' +
  'AAAAf////////////gAAAAAAAH////////////4AAAAAAAB////////////+AAAAAAAAP////////////AAAAAAAAD//////' +
  '//////wAAAAAAAA////////////8AAAAAAAAP////////////AAAAAAAAB////////////gAAAAAAAAf///////////4AAAA' +
  'AAAAH///////////+AAAAAAAAA////////////gAAAAAAAAP///////////wAAAAAAAAD///////////8AAAAAAAAA//////' +
  '//////AAAAAAAAAH///////////gAAAAAAAAB///////////4AAAAAAAAAf//////////+AAAAAAAAAD///////////gAAAA' +
  'AAAAA///////////wAAAAAAAAAP//////////8AAAAAAAAAD///////////AAAAAAAAAAf//////////gAAAAAAAAAH/////' +
  '/////4AAAAAAAAAB//////////+AAAAAAAAAAP//////////gAAAAAAAAAD//////////wAAAAAAAAAA//////////8AAAAA' +
  'AAAAAP//////////AAAAAAAAAAB//////////gAAAAAAAAAAf/////////4AAAAAAAAAAD/////////+AAAAAAAAAAAf////' +
  '////+AAAAAAAAAAAB////////+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── donut — 비트맵 실루엣(128×128, 잉크 55.9%)
//   원본 마스크: donut.png
var MASK_DONUT = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAH/gAAAAAAAAA' +
  'AAAAAAAAH///+AAAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAf/////8AAAAAAAAAAAAAA///////wAAAAAAAAAAAAA////' +
  '////gAAAAAAAAAAAB////////+AAAAAAAAAAAB/////////4AAAAAAAAAAA//////////AAAAAAAAAAA//////////8AAAAA' +
  'AAAAAf//////////wAAAAAAAAAf//////////+AAAAAAAAAP///////////wAAAAAAAAP////////////AAAAAAAAH//////' +
  '//////4AAAAAAAD/////////////AAAAAAAB/////////////4AAAAAAA//////////////AAAAAAAf/////////////4AAA' +
  'AAAP//////////////AAAAAAH//////////////4AAAAAD///////////////AAAAAB///////////////4AAAAAf///////' +
  '//gH///+AAAAAP////////+AAH///wAAAAH////////8AAAf//+AAAAD////////8AAAB///wAAAA////////+AAAAP//8AA' +
  'AAf////////AAAAB///gAAAH////////gAAAAP//4AAAD////////wAAAAB///AAAB////////4AAAAAP//4AAAf///////8' +
  'AAAAAB//+AAAP////////AAAAAAf//wAAD////////gAAAAAD//8AAB////////4AAAAAA///gAAf///////8AAAAAAH//4A' +
  'AH////////AAAAAAB//+AAD////////wAAAAAAf//wAA////////8AAAAAAH//8AAP////////AAAAAAB///AAH////////w' +
  'AAAAAAf//4AB////////8AAAAAAH//+AAf////////AAAAAAB///gAP////////wAAAAAAf//4AD////////8AAAAAAH///A' +
  'A/////////gAAAAAD///wAP////////4AAAAAA///8AH////////+AAAAAAf///AB/////////wAAAAAH///4Af////////8' +
  'AAAAAD///+AH/////////gAAAAB////gB/////////8AAAAAf///4Af/////////gAAAAP///+AH/////////8AAAAP////g' +
  'B//////////gAAAH////4Af/////////+AAAH////+AH//////////4AAH/////gB///////////wAP/////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////wAf//////////////////8AD///////////////////AA///////////////////wAP/////////' +
  '/////////8AD//////////////////+AAf//////////////////gAH//////////////////4AB//////////////////8A' +
  'AP//////////////////AAD//////////////////wAA//////////////////4AAH/////////////////+AAB/////////' +
  '/////////gAAf/////////////////wAAD/////////////////8AAA/////////////////+AAAH/////////////////gA' +
  'AA/////////////////wAAAP////////////////8AAAB////////////////+AAAAP////////////////AAAAD////////' +
  '////////wAAAAf///////////////4AAAAD///////////////8AAAAA///////////////+AAAAAH///////////////gAA' +
  'AAA///////////////wAAAAAH//////////////4AAAAAA//////////////8AAAAAAH/////////////+AAAAAAA///////' +
  '///////AAAAAAAH/////////////gAAAAAAA/////////////wAAAAAAAH////////////wAAAAAAAA////////////4AAAA' +
  'AAAAD///////////8AAAAAAAAAf//////////8AAAAAAAAAB//////////+AAAAAAAAAAP/////////+AAAAAAAAAAA/////' +
  '////+AAAAAAAAAAAD////////+AAAAAAAAAAAAP///////+AAAAAAAAAAAAA///////+AAAAAAAAAAAAAA//////8AAAAAAA' +
  'AAAAAAAD/////4AAAAAAAAAAAAAAAB////gAAAAAAAAAAAAAAAAAH/gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── airplane — 비트맵 실루엣(128×128, 잉크 24.3%)
//   원본 마스크: airplane.png
var MASK_AIRPLANE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAA8AAAAAAAAAAAAAAAAAAAAfgAAAAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAAH/gAAAAAAAAA' +
  'AAAAAAAAAD/8AAAAAAAAAAAAAAAAAAA//AAAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAH/+AAAAAAAAAAAAAAAAAAB/' +
  '/gAAAAAAAAAAAAAAAAAA//8AAAAAAAAAAAAAAAAAAP//AAAAAAAAAAAAAAAAAAD//wAAAAAAAAAAAAAAAAAA//+AAAAAAAAA' +
  'AAAAAAAAAf//gAAAAAAAAAAAAAAAAAH//4AAAAAAAAAAAAAAAAAB//+AAAAAAAAAAAAAAAAAAf//gAAAAAAAAAAAAAAAAAH/' +
  '/4AAAAAAAAAAAAAAAAAB//+AAAAAAAAAAAAAAAAAAf//wAAAAAAAAAAAAAAAAAH//8AAAAAAAAAAAAAAAAAB///AAAAAAAAA' +
  'AAAAAAAAAf//wAAAAAAAAAAAAAAAAAH//8AAAAAAAAAAAAAAAAAB///AAAAAAAAAAAAAAAAAAf//wAAAAAAAAAAAAAAAAAH/' +
  '/8AAAAAAAAAAAAAAAAAB///AAAAAAAAAAAAAAAAAAf//wAAAAAAAAAAAAAAAAAH//8AAAAAAAAAAAAAAAAAB///AAAAAAAAA' +
  'AAAAAAAAAf//wAAAAAAAAAAAAAAAAAH//8AAAAAAAAAAAAAAAAAB///AAAAAAAAAAAAAAAAAAf//wAAAAAAAAAAAAAAAAAH/' +
  '/8AAAAAAAAAAAAAAAAAB///AAAAAAAAAAAAAAAAAAf//wAAAAAAAAAAAAAAAAAH//8AAAAAAAAAAAAAAAAAD///AAAAAAAAA' +
  'AAAAAAAAA///4AAAAAAAAAAAAAAAAA////AAAAAAAAAAAAAAAAAf///8AAAAAAAAAAAAAAAAf////wAAAAAAAAAAAAAAAf//' +
  '//+AAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAf//////+AAAAAAAAAAAAAP///////4AAAAAA' +
  'AAAAAAf/j///8f/gAAAAAAAAAAAP/gP//+B/+AAAAAAAAAAAP/gD///AH/4AAAAAAAAAAP/wAf//wA//gAAAAAAAAAP/4AH/' +
  '/4AH/8AAAAAAAAAP/+AB//+AB//wAAAAAAAAH//gAf//gAf//AAAAAAAAH//4AH//4AH//8AAAAAAAH//+AB//+AB///wAAA' +
  'AAAH///gAf//gAf//+AAAAAAH///4AH//4AH///4AAAAAD///+AB//+AB////gAAAAH////gAf//wAf///+AAAAD////4AH/' +
  '/8AH////4AAAD////+AD///AB/////gAAD/////wB///8A/////8AAB/////+D////wf/////wAA//////////////////8A' +
  'AP//////////////////AAD//////////////////wAA//////////////////8AAP//////B///4H//////AAD/////gAP/' +
  '/8AB/////wAA////8AAB//+AAA////8AAP///AAAAf//gAAAP///AAD//4AAAAH//4AAAAH//wAA/+AAAAAB//+AAAAAB/8A' +
  'APAAAAAAAP//gAAAAAAPAAAAAAAAAAD//4AAAAAAAAAAAAAAAAAA//8AAAAAAAAAAAAAAAAAAP//AAAAAAAAAAAAAAAAAAD/' +
  '/wAAAAAAAAAAAAAAAAAA//8AAAAAAAAAAAAAAAAAAP//AAAAAAAAAAAAAAAAAAD//wAAAAAAAAAAAAAAAAAA//8AAAAAAAAA' +
  'AAAAAAAAAP//AAAAAAAAAAAAAAAAAAD//wAAAAAAAAAAAAAAAAAA//8AAAAAAAAAAAAAAAAAAH//AAAAAAAAAAAAAAAAAAB/' +
  '/wAAAAAAAAAAAAAAAAAAf/8AAAAAAAAAAAAAAAAAAP//AAAAAAAAAAAAAAAAAAD//wAAAAAAAAAAAAAAAAAB//+AAAAAAAAA' +
  'AAAAAAAAA///wAAAAAAAAAAAAAAAAAf//+AAAAAAAAAAAAAAAAAf///4AAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAP//' +
  '//8AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAD//////wAAAAAAAAAAAAAB///////AAAAAAA' +
  'AAAAAAB///////4AAAAAAAAAAAAA////////gAAAAAAAAAAAAf///////4AAAAAAAAAAAAH////////AAAAAAAAAAAAB////' +
  '////wAAAAAAAAAAAAf///////8AAAAAAAAAAAAH////////AAAAAAAAAAAAB//x//j//wAAAAAAAAAAAAf/gH/wH/8AAAAAA' +
  'AAAAAAH+AA/4AH/AAAAAAAAAAAAB8AAP8AAPgAAAAAAAAAAAAAAAD/AAAAAAAAAAAAAAAAAAAAfgAAAAAAAAAAAAAAAAAAAD' +
  'wAAAAAAAAAAAAAAAAAAAAYAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── sailboat — 비트맵 실루엣(128×128, 잉크 35.2%)
//   원본 마스크: sailboat.png
var MASK_SAILBOAT = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAA/AAAAAAAAAAAAAAAAAAAAf4AAAAAAAAAAAAAAAAAAAP/AAAAAAAAAAAAAAAAAAAH/wAAAAAAAAA' +
  'AAAAAAAAAB/+AAAAAAAAAAAAAAAAAAAf/gAAAAAAAAAAAAAAAAAAH/4AAAAAAAAAAAAAAAAAAB/+AAAAAAAAAAAAAAAAAAAf' +
  '/gAAAAAAAAAAAAAAAAAAH/4AAAAAAAAAAAAAAAAAAD/+AAAAAAAAAAAAAAAAAAA//gAAAAAAAAAAAAAAAAAAf/8AAAAAAAAA' +
  'AAAAAAAAAP//gAAAAAAAAAAAAAAAAAH//8AAAAAAAAAAAAAAAAAH///gAAAAAAAAAAAAAAAAD///8AAAAAAAAAAAAAAAAB//' +
  '//wAAAAAAAAAAAAAAAA////+AAAAAAAAAAAAAAAA/////4AAAAAAAAAAAAAAAf/////AAAAAAAAAAAAAAAP/////4AAAAAAA' +
  'AAAAAAAH//////AAAAAAAAAAAAAAD//////4AAAAAAAAAAAAAB///////AAAAAAAAAAAAAA///////4AAAAAAAAAAAAAf///' +
  '////AAAAAAAAAAAAAP///////4AAAAAAAAAAAAH////////AAAAAAAAAAAAD////////4AAAAAAAAAAAA//h//////AAAAAA' +
  'AAAAAAf/gH/////4AAAAAAAAAAAP/wA/////+AAAAAAAAAAAH/4AH/////wAAAAAAAAAAB/8AB/////+AAAAAAAAAAA/+AAP' +
  '/////wAAAAAAAAAAf/gAD/////+AAAAAAAAAAP/wAA///w//gAAAAAAAAAD/4AAP//wD/8AAAAAAAAAB/8AAD//4Af/gAAAA' +
  'AAAAA//AAA//8AD/4AAAAAAAAAP/gAAP//AAf/AAAAAAAAAH/wAAD//wAH/wAAAAAAAAD/4AAA//4AA/+AAAAAAAAA/+AAAP' +
  '/+AAH/wAAAAAAAAf/AAAD//gAB/8AAAAAAAAH/wAAA//4AAP/gAAAAAAAD/4AAAP/+AAB/4AAAAAAAA/8AAAD//gAAf/AAAA' +
  'AAAAf/AAAA//4AAD/wAAAAAAAH/gAAAP/+AAA/8AAAAAAAD/4AAAD//gAAH/gAAAAAAA/8AAAA//4AAB/4AAAAAAAP/AAAAP' +
  '/+AAAP/AAAAAAAH/gAAAD//gAAD/wAAAAAAB/4AAAA//4AAA/8AAAAAAA/8AAAAP/+AAAH/gAAAAAAP/AAAAD//gAAB/4AAA' +
  'AAAD/gAAAA//4AAAP+AAAAAAB/4AAAAP/+AAAD/gAAAAAAf+AAAAD//gAAA/8AAAAAAH/AAAAA//4AAAH/AAAAAAD/wAAAAP' +
  '/+AAAB/wAAAAAA/8AAAAD//gAAAf8AAAAAAP+AAAAA//4AAAH/AAAAAAD/gAAAAP/+AAAB/4AAAAAB/4AAAAD//gAAAf+AAA' +
  'AAAf+AAAAA//4AAAD/gAAAAAH/gAAAAP/+AAAA/4AAAAAB/4AAAAH//wAAAP+AAAAAAf+AAAAB//8AAAH/gAAAAAH/gAAAA/' +
  '//AAAB/4AAAAAB/8AAAAf//4AAAf+AAAAAA//gAAA////AAAH/gAAAAAP/+A//////+AAD/4AAAAAD///////////AB//AAA' +
  'AAA////////////3//gAAAAAP//////////////4AAAAAD//////////////+AAAAAAf//////////////gAAAAAH///////' +
  '///////4AAAAAB//////////////+AAAAAAf//////////////gAAAAAH//////////////4AAAAAB//////////////+AAA' +
  'AAAf//////////////gAAAAAD//////////////wAAAAAA//////////////8AAAAAAP//////////////AAAAAAD///////' +
  '///////wAAAAAAf/////////////4AAAAAAH/////////////+AAAAAAB//////////////gAAAAAAP/////////////4AAA' +
  'AAAD/////////////8AAAAAAA//////////////AAAAAAAH/////////////gAAAAAAB/////////////4AAAAAAAP//////' +
  '//////+AAAAAAAD/////////////AAAAAAAAf////////////gAAAAAAAD////////////4AAAAAAAA////////////8AAAA' +
  'AAAAH///////////+AAAAAAAAA////////////gAAAAAAAAP///////////wAAAAAAAAB///////////4AAAAAAAAAP/////' +
  '/////8AAAAAAAAAB//////////+AAAAAAAAAAP//////////AAAAAAAAAAB//////////gAAAAAAAAAAH/////////wAAAAA' +
  'AAAAAA/////////wAAAAAAAAAAAD////////wAAAAAAAAAAAAH///////wAAAAAAAAAAAAAP//////gAAAAAAAAAAAAAAf//' +
  '//+AAAAAAAAAAAAAAAAD///gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── rocket — 비트맵 실루엣(128×128, 잉크 25.6%)
//   원본 마스크: rocket.png
var MASK_ROCKET = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAA8AAAAAAAAAAAAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAf+AAAAAAAAAAAAAAAAAAAP/wAAAAAAAAA' +
  'AAAAAAAAAH/+AAAAAAAAAAAAAAAAAAD//wAAAAAAAAAAAAAAAAAB//+AAAAAAAAAAAAAAAAAA///wAAAAAAAAAAAAAAAAAf/' +
  '/+AAAAAAAAAAAAAAAAAH///gAAAAAAAAAAAAAAAAD///8AAAAAAAAAAAAAAAAB////gAAAAAAAAAAAAAAAAf///4AAAAAAAA' +
  'AAAAAAAAP////AAAAAAAAAAAAAAAAD////wAAAAAAAAAAAAAAAB////8AAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAB8AAAHwAAAAAAAAAAAAAAAf////+AAAAAAAAAAAAAAAH/////gAAAAAAA' +
  'AAAAAAAD/////8AAAAAAAAAAAAAAA//////AAAAAAAAAAAAAAAf/////wAAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB///' +
  '///gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAD//wD//wAAAAAAAAAAAAAA//wAP/8AAAAAAA' +
  'AAAAAAAf/wAA//gAAAAAAAAAAAAAH/4AAH/4AAAAAAAAAAAAAB/8AAA/+AAAAAAAAAAAAAAf+AAAH/gAAAAAAAAAAAAAP/gA' +
  'AB/4AAAAAAAAAAAAAD/wAAAP/AAAAAAAAAAAAAA/8AAAD/wAAAAAAAAAAAAAP+AAAAf8AAAAAAAAAAAAAD/gAAAH/AAAAAAA' +
  'AAAAAAA/4AAAB/4AAAAAAAAAAAAAf+AAAAf+AAAAAAAAAAAAAH/gAAAH/gAAAAAAAAAAAAB/4AAAB/4AAAAAAAAAAAAAf+AA' +
  'AAf+AAAAAAAAAAAAAH/gAAAH/gAAAAAAAAAAAAB/8AAAD/4AAAAAAAAAAAAAf/AAAA/+AAAAAAAAAAAAAP/wAAAf/gAAAAAA' +
  'AAAAAAD/+AAAH/4AAAAAAAAAAAAA//wAAD/+AAAAAAAAAAAAAP/+AAB//gAAAAAAAAAAAAD//wAA//8AAAAAAAAAAAAA//+A' +
  'A///AAAAAAAAAAAAAP//4Af//wAAAAAAAAAAAAD///////8AAAAAAAAAAAAA///////+AAAAAAAAAAAAAP///////gAAAAAA' +
  'AAAAAAD///////8AAAAAAAAAAAAD////////wAAAAAAAAAAAD////////+AAAAAAAAAAAB/////////4AAAAAAAAAAA/////' +
  '/////AAAAAAAAAAA//////////4AAAAAAAAAAP9///////7/AAAAAAAAAAP+P//////8f4AAAAAAAAAH/D///////D/AAAAA' +
  'AAAAB/g///////wf4AAAAAAAAA/wP//////8H/AAAAAAAAAf8D//////+A/4AAAAAAAAP+Af//////gP+AAAAAAAAD/gH///' +
  '///4B/wAAAAAAAB/4B//////8Af+AAAAAAAA/8AP//////AD/gAAAAAAAP/AD//////wA/8AAAAAAAD/gAf/////4AP/AAAA' +
  'AAAB/4AH/////+AB/4AAAAAAAf+AB//////AAf+AAAAAAAP/gAP/////wAH/wAAAAAAD/4AD/////4AB/8AAAAAAA/+AA///' +
  '///AAf/AAAAAAAf/AAf/////4AH/4AAAAAAH/wAf//////gB/+AAAAAAB/8Af//////8Af/gAAAAAAf/AP///////wH/4AAA' +
  'AAAP/wH/h///4f+B/+AAAAAAD/+D/g////B/wf/gAAAAAA//h/gP///wH+H/8AAAAAAP/4/gD///+Afx//AAAAAAD/+fwB//' +
  '//gD+f/wAAAAAA//v4Af///4Afn/4AAAAAAP//4AH///+AD//+AAAAAAB//8AB////gAf//gAAAAAAf/+AAf///4AB//4AAA' +
  'AAAH//AAH///+AAP/+AAAAAAB//gAB////gAB//gAAAAAAP/wAAf///4AAP/wAAAAAAD/4AAH///+AAB/8AAAAAAAf4AAA//' +
  '//AAAH+AAAAAAAD8AAAP///wAAA/AAAAAAAAAAAAD///8AAAAAAAAAAAAAAAAA///+AAAAAAAAAAAAAAAAAH///gAAAAAAAA' +
  'AAAAAAAAB///4AAAAAAAAAAAAAAAAAf//8AAAAAAAAAAAAAAAAAD///AAAAAAAAAAAAAAAAAA///gAAAAAAAAAAAAAAAAAH/' +
  '/4AAAAAAAAAAAAAAAAAB//8AAAAAAAAAAAAAAAAAAP//AAAAAAAAAAAAAAAAAAD//gAAAAAAAAAAAAAAAAAAf/4AAAAAAAAA' +
  'AAAAAAAAAH/8AAAAAAAAAAAAAAAAAAA/+AAAAAAAAAAAAAAAAAAAH/gAAAAAAAAAAAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAH' +
  '4AAAAAAAAAAAAAAAAAAAA8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── crescent — 비트맵 실루엣(128×128, 잉크 33.3%)
//   원본 마스크: crescent.png
var MASK_CRESCENT = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAA//8AAAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAA//////AAAAAAAAAAAAAAA//////8AAAAAA' +
  'AAAAAAAB///////4AAAAAAAAAAAAB////////gAAAAAAAAAAAB////////+AAAAAAAAAAAB/////////4AAAAAAAAAAB////' +
  '//////gAAAAAAAAAA//////////8AAAAAAAAAA///////////gAAAAAAAAAf//////////+AAAAAAAAAP///////////wAAA' +
  'AAAAAP///////////+AAAAAAAAH////////4AAAgAAAAAAAD////////gAAAAAAAAAAAB////////gAAAAAAAAAAAA//////' +
  '//AAAAAAAAAAAAAf///////AAAAAAAAAAAAAP///////AAAAAAAAAAAAAH///////AAAAAAAAAAAAAB///////gAAAAAAAAA' +
  'AAAA///////gAAAAAAAAAAAAAf//////wAAAAAAAAAAAAAP//////4AAAAAAAAAAAAAH//////8AAAAAAAAAAAAAB//////8' +
  'AAAAAAAAAAAAAA//////+AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAD//////4AAAAAAAAAAA' +
  'AAA//////8AAAAAAAAAAAAAAf/////+AAAAAAAAAAAAAAH//////gAAAAAAAAAAAAAD//////wAAAAAAAAAAAAAA//////4A' +
  'AAAAAAAAAAAAAP/////+AAAAAAAAAAAAAAH//////AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAA//////4AAAAAAAAAAAA' +
  'AAP/////8AAAAAAAAAAAAAAD//////AAAAAAAAAAAAAAA//////wAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AA' +
  'AAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////wAAAAAAAAAAAAAAP/////8AAAAAAAAAAAAAAD//////AAAAAAAAAAAAA' +
  'AA//////wAAAAAAAAAAAAAAP/////4AAAAAAAAAAAAAAD/////+AAAAAAAAAAAAAAA//////gAAAAAAAAAAAAAAf/////4AA' +
  'AAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAA' +
  'AB//////gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////4AA' +
  'AAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////8AAAAAAAAAAAAAAH//////AAAAAAAAAAAAA' +
  'AA//////wAAAAAAAAAAAAAAP/////8AAAAAAAAAAAAAAD//////AAAAAAAAAAAAAAA//////4AAAAAAAAAAAAAAP/////+AA' +
  'AAAAAAAAAAAAB//////wAAAAAAAAAAAAAAf/////8AAAAAAAAAAAAAAH//////AAAAAAAAAAAAAAB//////4AAAAAAAAAAAA' +
  'AAf/////+AAAAAAAAAAAAAAD//////wAAAAAAAAAAAAAA//////+AAAAAAAAAAAAAAP//////gAAAAAAAAAAAAAB//////8A' +
  'AAAAAAAAAAAAAf//////AAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAA///////AAAAAAAAAAAAAAP//////4AAAAAAAAAAA' +
  'AAB///////AAAAAAAAAAAAAAf//////4AAAAAAAAAAAAAD///////AAAAAAAAAAAAAA///////4AAAAAAAAAAAAAH///////' +
  'AAAAAAAAAAAAAA///////4AAAAAAAAgAAAAP///////AAAAAAAAwAAAAB///////8AAAAAAA8AAAAAf///////gAAAAAAeAA' +
  'AAAD///////+AAAAAAfAAAAAAf///////4AAAAAfwAAAAAH////////gAAAA/4AAAAAA/////////gAAB/8AAAAAAH//////' +
  '///AAH/+AAAAAAA//////////////AAAAAAAH/////////////gAAAAAAA/////////////wAAAAAAAH////////////4AAA' +
  'AAAAA////////////8AAAAAAAAH///////////+AAAAAAAAA////////////AAAAAAAAAH///////////AAAAAAAAAAf////' +
  '//////gAAAAAAAAAD//////////wAAAAAAAAAAP/////////wAAAAAAAAAAB/////////wAAAAAAAAAAAH////////4AAAAA' +
  'AAAAAAAf///////4AAAAAAAAAAAAB///////wAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAAP/////gAAAAAAAAAAAAAAAP/' +
  '///AAAAAAAAAAAAAAAAAf//4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── snowflake — 비트맵 실루엣(128×128, 잉크 29.5%)
//   원본 마스크: snowflake.png
var MASK_SNOWFLAKE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAAD/AAAAAAAAAA' +
  'AAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAcD/A4AAAAAAAAAAAAAAAAPg/wfAAAAAAAAAAAAAAAAH8P' +
  '8P4AAAAAAAAAAAAAAAD/n/n/AAAAAAAAAAAAAAAA/////wAAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAB////+AAAAAAAA' +
  'AAAAAAAAP////AAAAAAAAAAAAAAAAB////gAAAAAAAAAAAAA/gAP///gAH8AAAAAAAAAAf4AB///wAB/gAAAAAAAAcH+AAH/' +
  '/4AAf4OAAAAAAAPh/gAA//8AAH+HwAAAAAAH8f4AAH/+AAB/j+AAAAAAD/v+AAB//gAAf9/wAAAAAA///gAAf/4AAH//8AAA' +
  'AAAP//4AAHgeAAB///AAAAAAD//+AABwDgAAf//wAAAAAAf//gAAcA4AAH//4AAAAAAD//4AAGAGAAB//8AAAAAAAf/+AABg' +
  'BgAAf/+AAAAAAAD//gAAYAYAAH//AAAAAAAB//8AAHAOAAD//4AAAAAAP///gABwDgAB///8AAAAAH///8AAeB4AA////gAA' +
  'AAB////gAH/+AAf///4AAAAAf//4cAB//gAMH//+AAAAAH//8BgAf/4AGA///gAAAAB//+AMAH/+ADAH//4AAAAAf//gDgB/' +
  '/gBwB//+AAAAAAAD4A8Af/4A8AfAAAAAAAAAAeAPgH/+AfAHgAAAAAAAAADgD8D//wPwBwAAAAAAAAAAYB/j///H+A4AAAAA' +
  'AAAAADA///w///wMAAAAAAAAAAAf///4H///+AAAAAAAAAAAD///+B////AAAAAAAAAAAAf///gf///gAAAAAAAAAAAD/+f4' +
  'H8f/gAAAAAAABgAAAf/B+B+D/wAAAGAAAA8AAAD/gP//Af4AAADwAAAfgAAAf4D//wH8AAAB+AAAP8AAAD+A//8B+AAAA/wA' +
  'AH/gAAAfgfgfgfgAAAf+AAA/8AAAH8HgB4P4AAAP/AAAH/gAAD//gAH//AAAH/wAAA/8AAA//wAA//wAAD/4AAAP/gAAf/4A' +
  'AH/+AAB/8AAAB//////8AAA//////+AAAAf//////AAAP//////gAAP//+B//PgAAB8//gf//8AH///AP/A4AAAcD/wD///g' +
  'B///wB/gOAAAHAf4A///4Af//8Af4BgAABgH+AP//+AH///AH+AYAAAYB/gB///gB///wB/gOAAAGAf4A///4Af//8A/8DgA' +
  'ABwP/AP//+AD///gP/h4AAAeH/wH///AAAf/8P///AAAP///D//gAAAH//////wAAD//////4AAAB/5AHP/+AAB//gAB/+AA' +
  'AA/8AAA//gAAf/wAAD/wAAA/+AAAP/8AAP/8AAAf+AAAf/AAAD+DwAPB+AAAD/wAAH/gAAAfgeAHgfgAAAf+AAA/wAAAHwH+' +
  'f4D4AAAD/AAAH4AAAD8B//+A/AAAAfgAAA8AAAB/Af5/gP4AAADwAAAGAAAA/4H4H4H/AAAAYAAAAAAAAf/D+B/D/4AAAAAA' +
  'AAAAAAP///AP///AAAAAAAAAAAAH///wD///4AAAAAAAAAAAD///8A////AAAAAAAAAAABg///gf//gYAAAAAAAAAAAwH+f8' +
  'P+fwDAAAAAAAAAAAYA/B//+D8AYAAAAAAAAAAOAPgH/+AfAHAAAAAAAAAAHgDwB//gDwB4AAAAAAAD//4A4Af/4AcAf//AAA' +
  'AAB//+AMAH/+ADAP//4AAAAAf//wGAB//gAYD//+AAAAAH//+DAAf/4ADD///gAAAAB////gAHgeAAf///4AAAAAf///wABw' +
  'DgAD///+AAAAAD///4AAcA4AAf///AAAAAAB//8AAGAGAAD//4AAAAAAAf//AABgBgAA//8AAAAAAAH//wAAYAYAAP//gAAA' +
  'AAAD//8AAHAOAAD//8AAAAAAB///AABwDgAA///gAAAAAA///wAAeD4AAP//8AAAAAAP//8AAH/+AAD///AAAAAAD///AAB/' +
  '/gAA///wAAAAAAfx/wAAf/4AAP+f4AAAAAAD4f8AAP//AAD/D8AAAAAAAcH/AAP//8AA/wOAAAAAAACA/wAH///gAP8BAAAA' +
  'AAAAAP4AD///8AB/AAAAAAAAAAB8AB////gAAAAAAAAAAAAAAAA////8AAAAAAAAAAAAAAAA/////gAAAAAAAAAAAAAAAP//' +
  '//8AAAAAAAAAAAAAAAD/3/v/AAAAAAAAAAAAAAAAfw/w/wAAAAAAAAAAAAAAAD4P8H4AAAAAAAAAAAAAAAAcD/A4AAAAAAAA' +
  'AAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAAD/AAAAAAAAAAAAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAP' +
  '8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

function insideShape(mx, my, L) {
  if (!insideShapeBase(mx, my, L)) return false;
  /* ★noGrooves = **랩 비교 전용**이다. 이걸로 만든 코드는 데이터 셀 집합이 지금 디코더와
   *   달라 **읽히지 않는다**(실측: 골 5종 전부 "해독 실패"). 발행 경로에 절대 쓰지 말 것.
   *   반대로 **골 도입 전에 이미 발행된 코드**는 디코더의 변형 3 이 이 경로로 읽어 준다. */
  var gl = L.noGrooves ? null : SHAPE_GROOVES[L.shape];
  if (!gl) return true;
  var c = L.coreCenter.mx, R = L.Rdata || (L.N / 2 - 0.8);
  return !inGroove(gl, (mx - c) / R, (my - c) / R);
}
function insideShapeBase(mx, my, L) {
  const c = L.coreCenter.mx, R = L.Rdata || (L.N / 2 - 0.8);
  const nx = (mx - c) / R, ny = (my - c) / R;
  // ── 'custom' — 사용자 이미지에서 뽑은 임의 마스크 (2026-08-26, "내 모양 만들기") ──
  //   L.customMask = { n, bits } : n×n 비트맵을 정규화 좌표 [-1,1]²에 매핑한 것.
  //   ★★디코더 호환성 경고: 이 마스크는 스캐너가 알 방법이 없다 — `codec.readCode`가
  //   `dataCells(layout)`로 셀 집합·순서를 재구성하는데, 그 순서가 마스크에 의존하기
  //   때문이다(§B 실측). 따라서 'custom'은 **마스크를 아는 쪽**(생성기 자신의 셀프테스트,
  //   또는 마스크를 함께 전달받은 디코더)에서만 해독된다. 일반 스캐너용 코드를 만들 때
  //   쓰면 안 된다 — 호출부가 반드시 이 제약을 처리할 것.
  if (L.shape === 'custom') {
    const M = L.customMask;
    if (!M || !M.n || !M.bits) return true;          // 마스크 없으면 사각(=제약 없음)
    const ix = Math.floor((nx + 1) * 0.5 * M.n);
    const iy = Math.floor((ny + 1) * 0.5 * M.n);
    if (ix < 0 || iy < 0 || ix >= M.n || iy >= M.n) return false;
    return !!M.bits[iy * M.n + ix];
  }
  if (L.shape === 'piano') return inMask(MASK_PIANO, nx, ny);
  if (L.shape === 'butterfly') return inMask(MASK_BUTTERFLY, nx, ny);
  if (L.shape === 'earth') return inMask(MASK_EARTH, nx, ny);
  if (L.shape === 'eye') return inMask(MASK_EYE, nx, ny);
  if (L.shape === 'nose') return inMask(MASK_NOSE, nx, ny);
  if (L.shape === 'ear') return inMask(MASK_EAR, nx, ny);
  if (L.shape === 'hand') return inMask(MASK_HAND, nx, ny);
  if (L.shape === 'foot') return inMask(MASK_FOOT, nx, ny);
  if (L.shape === 'brain') return inMask(MASK_BRAIN, nx, ny);
  if (L.shape === 'dove') return inMask(MASK_DOVE, nx, ny);
  if (L.shape === 'love') return inMask(MASK_LOVE, nx, ny);
  if (L.shape === 'hope') return inMask(MASK_HOPE, nx, ny);
  if (L.shape === 'connection') return inMask(MASK_CONNECTION, nx, ny);
  if (L.shape === 'family') return inMask(MASK_FAMILY, nx, ny);
  if (L.shape === 'smile') return inMask(MASK_SMILE, nx, ny);
  if (L.shape === 'sadness') return inMask(MASK_SADNESS, nx, ny);
  if (L.shape === 'friendship') return inMask(MASK_FRIENDSHIP, nx, ny);
  if (L.shape === 'embrace') return inMask(MASK_EMBRACE, nx, ny);
  if (L.shape === 'boomer') return inMask(MASK_BOOMER, nx, ny);
  if (L.shape === 'grandpiano') return inMask(MASK_GRANDPIANO, nx, ny);
  if (L.shape === 'heartorgan') return inMask(MASK_HEARTORGAN, nx, ny);
  if (L.shape === 'chat') return inMask(MASK_CHAT, nx, ny);
  if (L.shape === 'book') return inMask(MASK_BOOK, nx, ny);
  if (L.shape === 'kiosk') return inMask(MASK_KIOSK, nx, ny);
  if (L.shape === 'gimbap') return inMask(MASK_GIMBAP, nx, ny);
  if (L.shape === 'elevator') return inMask(MASK_ELEVATOR, nx, ny);
  if (L.shape === 'manse') return inMask(MASK_MANSE, nx, ny);
  if (L.shape === 'lonely') return inMask(MASK_LONELY, nx, ny);
  if (L.shape === 'together') return inMask(MASK_TOGETHER, nx, ny);
  if (L.shape === 'comfort') return inMask(MASK_COMFORT, nx, ny);
  if (L.shape === 'dolhareubang') return inMask(MASK_DOLHAREUBANG, nx, ny);
  if (L.shape === 'tree') return inMask(MASK_TREE, nx, ny);
  if (L.shape === 'cup') return inMask(MASK_CUP, nx, ny);
  if (L.shape === 'car') return inMask(MASK_CAR, nx, ny);
  if (L.shape === 'dog') return inMask(MASK_DOG, nx, ny);
  if (L.shape === 'cat') return inMask(MASK_CAT, nx, ny);
  if (L.shape === 'fish') return inMask(MASK_FISH, nx, ny);
  if (L.shape === 'bear') return inMask(MASK_BEAR, nx, ny);
  if (L.shape === 'rabbit') return inMask(MASK_RABBIT, nx, ny);
  if (L.shape === 'turtle') return inMask(MASK_TURTLE, nx, ny);
  if (L.shape === 'whale') return inMask(MASK_WHALE, nx, ny);
  if (L.shape === 'horse') return inMask(MASK_HORSE, nx, ny);
  if (L.shape === 'bird') return inMask(MASK_BIRD, nx, ny);
  if (L.shape === 'deer') return inMask(MASK_DEER, nx, ny);
  if (L.shape === 'hearingaid') return inMask(MASK_HEARINGAID, nx, ny);
  if (L.shape === 'tooth') return inMask(MASK_TOOTH, nx, ny);
  if (L.shape === 'mouth') return inMask(MASK_MOUTH, nx, ny);
  if (L.shape === 'lungs') return inMask(MASK_LUNGS, nx, ny);
  if (L.shape === 'bone') return inMask(MASK_BONE, nx, ny);
  if (L.shape === 'arena') return inMask(MASK_ARENA, nx, ny);
  if (L.shape === 'pestbug') return inMask(MASK_PESTBUG, nx, ny);
  if (L.shape === 'finger') return inMask(MASK_FINGER, nx, ny);
  if (L.shape === 'like') return inMask(MASK_LIKE, nx, ny);
  if (L.shape === 'victory') return inMask(MASK_VICTORY, nx, ny);
  if (L.shape === 'oksign') return inMask(MASK_OKSIGN, nx, ny);
  if (L.shape === 'lovesign') return inMask(MASK_LOVESIGN, nx, ny);
  if (L.shape === 'prayer') return inMask(MASK_PRAYER, nx, ny);
  if (L.shape === 'fist') return inMask(MASK_FIST, nx, ny);
  if (L.shape === 'badge') return inMask(MASK_BADGE, nx, ny);
  if (L.shape === 'waterdrop') return inMask(MASK_WATERDROP, nx, ny);
  if (L.shape === 'crown') return inMask(MASK_CROWN, nx, ny);
  if (L.shape === 'bag') return inMask(MASK_BAG, nx, ny);
  if (L.shape === 'watch') return inMask(MASK_WATCH, nx, ny);
  if (L.shape === 'ring') return inMask(MASK_RING, nx, ny);
  if (L.shape === 'perfume') return inMask(MASK_PERFUME, nx, ny);
  if (L.shape === 'fan') return inMask(MASK_FAN, nx, ny);
  if (L.shape === 'helmet') return inMask(MASK_HELMET, nx, ny);
  if (L.shape === 'goggles') return inMask(MASK_GOGGLES, nx, ny);
  if (L.shape === 'harness') return inMask(MASK_HARNESS, nx, ny);
  if (L.shape === 'extinguisher') return inMask(MASK_EXTINGUISHER, nx, ny);
  if (L.shape === 'safetycone') return inMask(MASK_SAFETYCONE, nx, ny);
  if (L.shape === 'gloves') return inMask(MASK_GLOVES, nx, ny);
  if (L.shape === 'dustmask') return inMask(MASK_DUSTMASK, nx, ny);
  if (L.shape === 'haetae') return inMask(MASK_HAETAE, nx, ny);
  if (L.shape === 'safetymanager') return inMask(MASK_SAFETYMANAGER, nx, ny);
  if (L.shape === 'healthmanager') return inMask(MASK_HEALTHMANAGER, nx, ny);
  if (L.shape === 'safetyboot') return inMask(MASK_SAFETYBOOT, nx, ny);
  if (L.shape === 'safetyvest') return inMask(MASK_SAFETYVEST, nx, ny);
  if (L.shape === 'elephant') return inMask(MASK_ELEPHANT, nx, ny);
  if (L.shape === 'owl') return inMask(MASK_OWL, nx, ny);
  if (L.shape === 'crab') return inMask(MASK_CRAB, nx, ny);
  if (L.shape === 'octopus') return inMask(MASK_OCTOPUS, nx, ny);
  if (L.shape === 'snail') return inMask(MASK_SNAIL, nx, ny);
  if (L.shape === 'bee') return inMask(MASK_BEE, nx, ny);
  if (L.shape === 'frog') return inMask(MASK_FROG, nx, ny);
  if (L.shape === 'swan') return inMask(MASK_SWAN, nx, ny);
  if (L.shape === 'umbrella') return inMask(MASK_UMBRELLA, nx, ny);
  if (L.shape === 'key') return inMask(MASK_KEY, nx, ny);
  if (L.shape === 'lightbulb') return inMask(MASK_LIGHTBULB, nx, ny);
  if (L.shape === 'scissors') return inMask(MASK_SCISSORS, nx, ny);
  if (L.shape === 'hammer') return inMask(MASK_HAMMER, nx, ny);
  if (L.shape === 'glasses') return inMask(MASK_GLASSES, nx, ny);
  if (L.shape === 'envelope') return inMask(MASK_ENVELOPE, nx, ny);
  if (L.shape === 'gift') return inMask(MASK_GIFT, nx, ny);
  if (L.shape === 'apple') return inMask(MASK_APPLE, nx, ny);
  if (L.shape === 'icecream') return inMask(MASK_ICECREAM, nx, ny);
  if (L.shape === 'cupcake') return inMask(MASK_CUPCAKE, nx, ny);
  if (L.shape === 'donut') return inMask(MASK_DONUT, nx, ny);
  if (L.shape === 'airplane') return inMask(MASK_AIRPLANE, nx, ny);
  if (L.shape === 'sailboat') return inMask(MASK_SAILBOAT, nx, ny);
  if (L.shape === 'rocket') return inMask(MASK_ROCKET, nx, ny);
  if (L.shape === 'crescent') return inMask(MASK_CRESCENT, nx, ny);
  if (L.shape === 'snowflake') return inMask(MASK_SNOWFLAKE, nx, ny);
  if (L.shape === 'round') return nx * nx + ny * ny <= 1;
  if (L.shape === 'heart') return pointInHeart(nx, ny);
  if (L.shape === 'bubble') return pointInBubble(nx, ny);
  if (L.shape === 'chat') return pointInChat(nx, ny);
  if (L.shape === 'lid') return pointInLid(nx, ny);
  if (L.shape === 'rose') return pointInRose(nx, ny);
  if (L.shape === 'fish') return pointInFish(nx, ny);
  if (L.shape === 'piano') return pointInPianoKeys(nx, ny);
  // ★2026-08-27 §7-36 유령 로스터 사건 — 3겹 방어 ②.
  //   AI 팩 5종(pixel-crab·spark·knot6·bot·twinkle)이 **기하 함수 없이** 여기와 로스터에
  //   등재돼 있었다. 정의 없는 식별자를 호출해 ReferenceError 가 나면 "못 읽음"이 아니라
  //   **해독 자체가 중단**된다 — 스캐너 프레임 루프가 죽고 진단도 가려진다.
  //   그 5종은 뿌리에서 제거했고(방어 ①), 여기서는 **미래에 같은 실수가 나도** 크래시가
  //   아니라 깨끗한 false 로 떨어지게 한다. typeof 는 선언되지 않은 식별자에도 안전하다.
  //   (인체 팩 4종은 기하가 실존해 그대로 유지 — 파일럿이라 생성기에는 미노출.)
  if (SHAPE_POINT_FN.hasOwnProperty(L.shape)) {
    var _sfn = SHAPE_POINT_FN[L.shape];
    return _sfn ? _sfn(nx, ny) : false;
  }
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
    // ★2026-08-16(Fable 5 자문): Ri=0.24는 허리가 너무 좁아 다리QR 박스(코어 아래, 캔버스 폭의
    // 15%)가 실루엣 밖으로 삐져나왔다(실측: 264px QR박스 vs 134~238px 허리 폭). 다리QR을 줄이면
    // 실제 스캔이 안 되는 게 이미 확인된 사실이라 축소는 불가 — 대신 허리를 넓혀 QR을 완전히
    // 감싸도록 Ri를 키움. 독자 기하 재구현으로 grid L 기준 "박스가 완전히 안에 들어가는 최소값"을
    // 이진탐색: 0.39. Fable 5가 별도로 유도한 값(0.43)과 근접해 서로 검증됨 — 두 grid 크기(S/M)
    // 차이·라운딩 여유까지 감안해 0.44로 적용(5각별의 Ri=0.42와도 톤이 맞음). 이름은 "boomerang"
    // 그대로 유지(형이 "살짝 바뀌어도 된다"고 했지만 부메랑이라는 정체성 자체는 여전히 유효한
    // 폭넓은 4꼭짓점 모양이라 새 이름 없이 파라미터만 조정) — 실측 검증 통과 후 라이브 반영.
    const Ro = 1.0, Ri = 0.44;
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
// ── 반경 비교 가속 (2026-08-29) ─────────────────────────────────────────────
// inkAt/reservedAt 은 픽셀마다 6번 넘게 불린다(격자 L·cellPx4·ss2 = 3,200만 회).
// Math.hypot 은 오버플로 안전 알고리즘이라 곱셈 몇 번보다 훨씬 비싸다.
//
// ★그런데 hypot 을 제곱비교로 **그냥** 바꾸면 픽셀이 달라질 수 있다.
//   반경이 2.5(위성)처럼 딱 떨어지는 값이라 3-4-5 직각삼각형 같은 **정확한 경계점**이
//   실제로 생긴다 — 그 점에서 두 계산이 마지막 자리에서 갈리면 도트 하나가 바뀐다.
//   그래서 확실한 안/밖만 제곱으로 끝내고, 경계 극소수(상대오차 1e-7 띠)만 hypot 으로
//   되받는다. 결과는 **비트 동일**이고 hypot 호출은 사실상 0 이 된다.
function within(dx, dy, r) {
  const q = dx * dx + dy * dy, r2 = r * r;
  if (q < r2 * 0.9999999) return true;
  if (q > r2 * 1.0000001) return false;
  return Math.hypot(dx, dy) <= r;   // 경계 — 원래 계산 그대로
}

function reservedAt(mx, my, L) {
  const { coreCenter, anchors, orbit, spec } = L; const S = spec || SPEC;
  if (L.shape && L.shape !== 'square' && !insideShape(mx, my, L)) return true;   // 모양 밖 = 데이터 없음
  const cdx = mx - coreCenter.mx, cdy = my - coreCenter.my;
  const cq = cdx * cdx + cdy * cdy;
  const RO2 = S.orbitRadius + S.orbitDotR + 0.4, RC2 = S.coreOuter + 0.6;
  const ROUT2 = (RO2 > RC2 ? RO2 : RC2);
  if (cq <= ROUT2 * ROUT2 * 1.0000001) {
    const dCore = Math.hypot(cdx, cdy);
    if (dCore <= S.coreOuter + 0.6) return true;
    if (Math.abs(dCore - S.orbitRadius) <= S.orbitDotR + 0.4) return true;
  }
  const SR2 = S.satRadius + 0.6 + ((L.hug && S.satHalo) ? S.satHalo : 0);
  for (let i = 1; i < anchors.length; i++) {
    const dx = mx - anchors[i].mx; if (dx > SR2 || dx < -SR2) continue;
    const dy = my - anchors[i].my; if (dy > SR2 || dy < -SR2) continue;
    if (within(dx, dy, SR2)) return true;
  }
  if (L.bridgeRect) { const r = L.bridgeRect.reserve;
    if (mx >= r.mx0 && mx < r.mx1 && my >= r.my0 && my < r.my1) return true; }
  return false;
}

// ── 데이터 셀 순서 열거 (예약 밖 셀, 래스터 순서) — 인코드/디코드 공용 계약 ──
//   render(opts.bits) 배치 순서와 반드시 동일해야 한다(디코더가 같은 순서로 읽음).
/* ★C10-b(2026-09-04) — dataCells 예약마스크 메모이즈.
 *   dataCells 는 순수 함수다 — 같은 레이아웃이면 같은 셀 목록이 나온다. 그런데 해독기의
 *   순회(pass1/pass2 = 후보 x 84형상 x 부스터변형, 밀착 = 후보 x 변형)는 매 호출마다
 *   layout() 을 새로 만들어 넘기므로 **같은 계산을 프레임당 수백~수천 번 반복**한다.
 *   실측(격자 L, N=128): dataCells 66.4ms 중 reservedAt 16,384회가 거의 전부이고,
 *   예약마스크만 있으면 셀 목록 재구성은 0.30ms — **221배**.
 *   ★자르는 게 아니라 같은 답을 다시 구하지 않는 것이라 **원리적으로 무손실**이다
 *     (순서·집합 그대로. 캐시가 비어도 예전 경로 그대로 돈다).
 *   ★저장하는 건 배열이 아니라 **비트마스크**다 — 셀 목록을 통째로 캐시하면 격자 L 한 건이
 *     ~0.6MB 라 폰에서 못 쓴다. 마스크는 N^2 비트 = 2KB.
 *   키 = reservedAt/insideShape 가 실제로 읽는 필드 전부:
 *     shape · N · hug(satHalo) · noGrooves · Rdata · coreCenter · anchors · bridgeRect.reserve
 *     + reservedAt 이 읽는 SPEC 값(satRadius·satHalo·coreOuter·orbitRadius·orbitDotR).
 *     ★SPEC 은 지금 코드베이스 어디서도 변형되지 않지만(전수 확인), 키가 '읽는 필드 전부'라고
 *       적어 놓고 일부를 빼면 그 주석이 다음 사람을 속인다. 상수 5개는 문자열 한 번이면 된다.
 *   customMask(임의 비트맵)와 SPEC 이 아닌 스펙은 키를 만들 수 없어 캐시하지 않는다.
 *   반환 배열은 읽기 전용으로만 쓰인다(codec.sampleCellGrays/sampleCellColor · render ·
 *   locate 격자재적합 — 전수 확인, 변형하는 소비처 없음). */
const _DC_CACHE = new Map(), _DC_MAX = 2048;
//   SPEC 은 위 가드(L.spec !== SPEC 이면 캐시 안 함)로 하나에 고정되므로 한 번만 만든다.
const _DC_SPEC_K = '|' + SPEC.satRadius + ',' + (SPEC.satHalo || 0) + ',' + SPEC.coreOuter
                 + ',' + SPEC.orbitRadius + ',' + SPEC.orbitDotR;
function _dcKey(L) {
  if (!L || L.shape === 'custom' || L.customMask) return null;
  if ((L.spec || SPEC) !== SPEC) return null;
  let k = L.shape + '|' + L.N + '|' + (L.hug ? 1 : 0) + '|' + (L.noGrooves ? 1 : 0)
        + '|' + (L.Rdata || 0) + '|' + L.coreCenter.mx + ',' + L.coreCenter.my;
  const A = L.anchors || [];
  for (let i = 0; i < A.length; i++) k += '|' + A[i].mx + ',' + A[i].my;
  k += _DC_SPEC_K;
  const r = L.bridgeRect && L.bridgeRect.reserve;
  k += '|' + (r ? r.mx0 + ',' + r.my0 + ',' + r.mx1 + ',' + r.my1 : '-');
  return k;
}
function dataCells(L) {
  const N = L.N, key = _dcKey(L);
  if (key === null) {                       // 캐시 불가(custom 등) — 예전 경로 그대로
    const out0 = [];
    for (let cy = 0; cy < N; cy++) for (let cx = 0; cx < N; cx++) {
      if (!reservedAt(cx + 0.5, cy + 0.5, L)) out0.push([cx, cy]);
    }
    return out0;
  }
  let m = _DC_CACHE.get(key);
  if (!m) {
    m = new Uint8Array((N * N + 7) >> 3);
    for (let cy = 0; cy < N; cy++) for (let cx = 0; cx < N; cx++) {
      if (!reservedAt(cx + 0.5, cy + 0.5, L)) { const i = cy * N + cx; m[i >> 3] |= (1 << (i & 7)); }
    }
    if (_DC_CACHE.size >= _DC_MAX) _DC_CACHE.clear();
    _DC_CACHE.set(key, m);
  }
  const out = [];
  for (let cy = 0; cy < N; cy++) for (let cx = 0; cx < N; cx++) {
    const i = cy * N + cx; if (m[i >> 3] & (1 << (i & 7))) out.push([cx, cy]);
  }
  return out;
}

// ── 한 점(모듈좌표)의 잉크 여부: true=검정 ────────────────────────────────
function inkAt(mx, my, L, cellBit) {
  const { coreCenter, anchors, orbit, spec } = L; const S = spec || SPEC;

  // ★코어+궤도를 한 번에 걸러낸다 — 둘 다 코어 중심 기준이라 바깥 원 하나로 끝난다.
  //   이미지의 98%는 여기서 곱셈 두 번에 빠져나간다(예전엔 매번 hypot 6회).
  const cdx = mx - coreCenter.mx, cdy = my - coreCenter.my;
  const cq = cdx * cdx + cdy * cdy;
  const RO = S.orbitRadius + S.orbitDotR + 0.4, RC = S.coreOuter;
  const ROUT = (RO > RC ? RO : RC);
  if (cq <= ROUT * ROUT * 1.0000001) {
    // 코어 동심원
    const dCore = Math.hypot(cdx, cdy);
    if (dCore <= S.coreOuter) {
      for (const ring of S.coreRings) if (dCore <= ring.r) return ring.ink;
      return false;
    }
    // 포맷 궤도 도트
    if (Math.abs(dCore - S.orbitRadius) <= S.orbitDotR + 0.4) {
      const od = S.orbitDotR;
      for (const o of orbit) {
        if (!o.on) continue;
        const dx = mx - o.mx; if (dx > od || dx < -od) continue;   // 정확한 기각(|dx|>r ⇒ 거리>r)
        const dy = my - o.my; if (dy > od || dy < -od) continue;
        if (within(dx, dy, od)) return true;
      }
    }
  }
  // 위성 — 사각 경계로 먼저 쳐낸다
  // ★밀착 모드에서는 위성이 **데이터 한복판**에 앉으므로 바깥에 흰 고리를 두른다.
  //   고리가 없으면 데이터 점과 붙어 위성 원판의 경계가 사라지고 무게중심이 편향된다.
  const SR = S.satRadius;
  const HALO = (L.hug && S.satHalo) ? S.satHalo : 0;
  const SRH = SR + HALO;
  for (let i = 1; i < anchors.length; i++) {
    const a = anchors[i];
    const dx = mx - a.mx; if (dx > SRH || dx < -SRH) continue;
    const dy = my - a.my; if (dy > SRH || dy < -SRH) continue;
    const ds = Math.hypot(dx, dy);
    if (ds <= SR) {
      if (a.type === 'donut') return ds >= S.satInner; // 도넛: 흰 중심
      return true;                                     // 원판
    }
    if (ds <= SRH) return false;                       // 고리 = 강제 흰색
  }
  // 데이터 도트 (라운드 채움): 예약 밖 + 셀 비트 on
  if (cellBit) {
    const cx = Math.floor(mx) + 0.5, cy = Math.floor(my) + 0.5;
    if (within(mx - cx, my - cy, S.dataDotR)) return true;
  }
  return false;
}

// ── 렌더: 오르빗 코드 이미지(RGBA) + 정답 앵커 픽셀좌표 ──────────────────
//   opts: { grid:'S', cellPx:6, quiet:4, seed:1, ss:2(슈퍼샘플), data:true }
/* ── 외곽 스트로크(인지 보강) ─────────────────────────────────────────────────
 *   실루엣 **밖 여백**에 흰 해자(gap 모듈)를 띄우고 그 바깥으로 w 모듈 두께의 검정 선을 두른다.
 *   데이터 셀·예약·셀 순서를 하나도 건드리지 않는다 → 포맷 무변경, 스캐너 수정 0.
 *
 *   ★왜 필요한가(B1 인지 게이트 실측): 코드를 크게 그릴수록 데이터 도트가 **개별 점으로 분해**되어
 *     실루엣이 "면"이 아니라 "점 구름"으로 보이고 가장자리 윤곽이 헤진다. 인지율이 오히려 떨어진다
 *     (catalog 80% → detail 71%, 두 모델 같은 방향). 외곽선을 명시적으로 그어야 면으로 읽힌다.
 *
 *   ★왜 해자(gap)가 필수인가(outline-lab probe 실측): 스트로크를 실루엣에 **밀착**시키면
 *     데이터 셀을 한 칸도 안 건드렸는데도 해독이 죽는다(pestbug·oksign). 적응형 이진화의 지역
 *     평균이 내려가 경계의 안티앨리어싱된 도트가 흰색으로 뒤집히기 때문. 1모듈 띄우면 산다.
 *
 *   ★왜 형상별 핀인가: 더 두껍거나 더 멀면 오목한 자리(고래 꼬리·지느러미 사이)를 메워
 *     새 검정 덩어리를 만든다 — 형상마다 (gap,w) 가 다르다. 전역 상수 불가.
 *
 *   ★QR 존은 비운다: 실루엣이 QR 자리를 벗어나는 형상에서 스트로크가 QR 을 가로질러 죽인다.
 */
/* ── B2-d 무지개 외곽선 (2026-09-04, 오너 제안) ─────────────────────────────
 *  wiabook.com 화제작 카드의 linear-gradient(135deg, …) 를 외곽 스트로크에 입힌다.
 *
 *  ★"밝기는 그대로 두고 색만" 은 **물리적으로 불가능**하다 — 검정 스트로크는 Y=0 이고
 *    Y=0 인 색은 검정 하나뿐이다(Y=0.299R+0.587G+0.114B). 그래서 대신 **밝기 상한**을 둔다:
 *    색상(hue)은 그대로 두고 Y 가 maxY 를 넘는 색만 눌러 내린다. 결과적으로 스트로크 전체가
 *    어두운 띠로 남아 적응형 이진화에서 여전히 "검정"으로 떨어진다.
 *    참고: 원본 그라디언트의 Y 는 노랑 225.9 ~ 보라 69.7 로 3배 넘게 벌어져 있어, 상한을 안 두면
 *    노랑·주황 구간이 회색변환에서 **흰색으로 뒤집혀 외곽선이 끊긴다**(스트로크의 존재 이유인
 *    인지율이 오히려 나빠진다).
 *  ★스트로크는 데이터 셀 밖(해자 바깥 띠)에만 칠해지므로 페이로드에 닿지 않는다. 그래도
 *    B2-c 가 닫은 결정(기울여 찍으면 3/15 종이 실루엣 경계와 겹쳐 깨진다)은 **색과 무관한
 *    기하 문제**라 무지개판도 똑같이 깨진다 — 그래서 이것도 **옵트인**이다. 기본값 켬 금지.
 */
const RAINBOW_STOPS = [
  [0xff,0x00,0x00],[0xff,0x88,0x00],[0xff,0xff,0x00],[0x00,0xcc,0x00],
  [0x00,0x88,0xff],[0x88,0x00,0xff],[0xff,0x00,0xff],[0xff,0x00,0x00]
];
const RAINBOW_MAX_Y = 96;          // 기본 밝기 상한. 낮출수록 차분·안전, 올릴수록 선명.
function _rainbowRGB(t, maxY) {
  const n = RAINBOW_STOPS.length - 1;
  let u = t - Math.floor(t);                      // 0~1 로 순환
  const f = u * n, i = Math.min(n - 1, Math.floor(f)), k = f - i;
  const a = RAINBOW_STOPS[i], b = RAINBOW_STOPS[i + 1];
  let R = a[0] + (b[0] - a[0]) * k, G = a[1] + (b[1] - a[1]) * k, B = a[2] + (b[2] - a[2]) * k;
  const Y = 0.299 * R + 0.587 * G + 0.114 * B;
  const cap = maxY != null ? maxY : RAINBOW_MAX_Y;
  if (Y > cap && Y > 0) { const q = cap / Y; R *= q; G *= q; B *= q; }   // 색상 유지, 밝기만 하강
  return [Math.round(R), Math.round(G), Math.round(B)];
}

function _dilateMask(src, W, H, r) {
  if (r <= 0) return src;
  const tmp = new Uint8Array(W * H), out = new Uint8Array(W * H);
  for (let y = 0; y < H; y++) { let run = 0; const row = y * W;
    for (let x = 0; x < W + r; x++) { if (x < W && src[row + x]) run = 2 * r + 1;
      if (run > 0 && x - r >= 0 && x - r < W) tmp[row + x - r] = 1; if (run > 0) run--; } }
  for (let x = 0; x < W; x++) { let run = 0;
    for (let y = 0; y < H + r; y++) { if (y < H && tmp[y * W + x]) run = 2 * r + 1;
      if (run > 0 && y - r >= 0 && y - r < H) out[(y - r) * W + x] = 1; if (run > 0) run--; } }
  return out;
}
function strokeOutline(data, dim, L, cellPx, quiet, gap, w, rainbow) {
  if (!(w > 0)) return 0;
  const N = L.N;
  const inside = new Uint8Array(dim * dim);
  for (let y = 0; y < dim; y++) for (let x = 0; x < dim; x++) {
    const mx = (x + 0.5) / cellPx - quiet, my = (y + 0.5) / cellPx - quiet;
    if (mx < 0 || my < 0 || mx >= N || my >= N) continue;
    if (insideShape(mx, my, L)) inside[y * dim + x] = 1;
  }
  const rGap = Math.round(gap * cellPx), rAll = Math.round((gap + w) * cellPx);
  const moat = rGap > 0 ? _dilateMask(inside, dim, dim, rGap) : inside;
  const band = _dilateMask(inside, dim, dim, rAll);
  // QR 예약 사각형(모듈) → 픽셀. 스트로크가 QR 을 가로지르지 않게 비운다.
  const br = L.bridgeRect || null;
  let qx0 = -1, qy0 = -1, qx1 = -1, qy1 = -1;
  if (br && br.reserve) {
    qx0 = (br.reserve.mx0 + quiet) * cellPx; qy0 = (br.reserve.my0 + quiet) * cellPx;
    qx1 = (br.reserve.mx1 + quiet) * cellPx; qy1 = (br.reserve.my1 + quiet) * cellPx;
  }
  let painted = 0;
  for (let y = 0; y < dim; y++) for (let x = 0; x < dim; x++) {
    const i = y * dim + x;
    if (!band[i] || moat[i]) continue;
    if (qx0 >= 0 && x >= qx0 && x <= qx1 && y >= qy0 && y <= qy1) continue;   // QR 존 비움
    const o = i * 4;
    if (rainbow) {
      // CSS linear-gradient(135deg) 와 같은 축: 화면좌표에서 오른쪽아래 방향 → t ∝ (x+y).
      const c = _rainbowRGB((x + y) / (2 * dim), rainbow.maxY);
      data[o] = c[0]; data[o + 1] = c[1]; data[o + 2] = c[2];
    } else { data[o] = 0; data[o + 1] = 0; data[o + 2] = 0; }
    data[o + 3] = 255; painted++;
  }
  return painted;
}

function render(opts) {
  const grid = opts.grid || 'S';
  const cellPx = opts.cellPx || 6;
  const quiet = opts.quiet != null ? opts.quiet : 4;
  const ss = opts.ss || 2;
  const withData = opts.data !== false;
  const spec = mergeSpec(opts.spec);
  const L = layout(grid, spec, opts.shape, { bridge: opts.bridge, bridgeLegacy: opts.bridgeLegacy, customMask: opts.customMask, hug: opts.hug, noGrooves: opts.noGrooves });
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

  // ── 앵커 근접 맵 (2026-08-29) ────────────────────────────────────────────
  // 앵커(코어·궤도·위성)가 차지하는 넓이는 그림의 **1.7%** 뿐인데, 예전엔 나머지 98%의
  // 픽셀도 서브샘플마다 inkAt() 을 불러 전수검사를 받았다. 모듈 한 칸 단위로 "여기 앵커가
  // 닿을 수 있나"를 미리 구워두고, 닿을 수 없는 칸은 호출 자체를 건너뛴다.
  //   · 판정은 **원판의 사각 경계**로 넉넉하게 — 실제보다 넓게 잡히는 건 안전한 쪽이다.
  //   · 한 칸 부풀린다(픽셀 하나가 모듈 두 칸에 걸칠 수 있으므로).
  //   · inkAt 은 여기서 cellBit=0 으로만 불리므로 데이터 도트는 이 맵과 무관하다.
  const AW = N + 2 * quiet;
  const AMASK = new Uint8Array(AW * AW);
  {
    const S2 = L.spec || spec;
    const rc = Math.max(S2.coreOuter, S2.orbitRadius + S2.orbitDotR + 0.4);
    const discs = [[L.coreCenter.mx, L.coreCenter.my, rc]];
    for (let i = 1; i < L.anchors.length; i++) discs.push([L.anchors[i].mx, L.anchors[i].my, S2.satRadius]);
    for (const [dx0, dy0, r] of discs) {
      const x0 = Math.floor(dx0 - r) - 1, x1 = Math.ceil(dx0 + r) + 1;
      const y0 = Math.floor(dy0 - r) - 1, y1 = Math.ceil(dy0 + r) + 1;
      for (let iy = y0; iy <= y1; iy++) {
        const ay = iy + quiet; if (ay < 0 || ay >= AW) continue;
        for (let ix = x0; ix <= x1; ix++) {
          const ax = ix + quiet; if (ax < 0 || ax >= AW) continue;
          AMASK[ay * AW + ax] = 1;
        }
      }
    }
  }

  // ── 좌표표 (2026-08-29) ─────────────────────────────────────────────────
  // 서브샘플 좌표는 **픽셀 좌표에만** 의존한다(x·y 공식이 같다) — 그런데 예전엔
  // 서브샘플마다 다시 계산했다: 나눗셈 2회 × 4서브샘플 × 168만 픽셀.
  // 축 하나 분량만 미리 굽고 x·y 양쪽이 같이 쓴다.
  //   ★식을 **한 글자도 바꾸지 않고** 그대로 옮겨 담는다 — 부동소수 결과가 달라지면
  //     도트 경계에서 픽셀이 흔들린다(그래서 1/cellPx 곱셈으로 바꾸지 않았다).
  const TM = new Float64Array(dim * ss);   // 모듈 좌표
  const TCB = new Int32Array(dim * ss);    // 그 좌표가 속한 셀
  const TD = new Float64Array(dim * ss);   // 셀 중심으로부터의 변위
  const TDQ = new Float64Array(dim * ss);  // 그 변위의 제곱
  const TA = new Int32Array(dim);          // 픽셀 → 모듈칸(quiet 포함)
  for (let c = 0; c < dim; c++) {
    TA[c] = Math.floor(c / cellPx);
    for (let k = 0; k < ss; k++) {
      const f = c + inv * k + base;
      const m = f / cellPx - quiet;
      const cb = Math.floor(m);
      const d = m - (cb + 0.5);
      const i = c * ss + k;
      TM[i] = m; TCB[i] = cb; TD[i] = d; TDQ[i] = d * d;
    }
  }
  const dotR2 = dataDotR * dataDotR;

  for (let py = 0; py < dim; py++) {
    const arow = TA[py];                             // 픽셀 → 모듈칸(quiet 포함 좌표)
    const arowOff = (arow >= 0 && arow < AW) ? arow * AW : -1;
    for (let px = 0; px < dim; px++) {
      const acol = TA[px];
      const nearAnchor = arowOff >= 0 && acol < AW && AMASK[arowOff + acol] === 1;
      let accR = 0, accG = 0, accB = 0;              // 서브샘플 RGB 누적
      for (let sy = 0; sy < ss; sy++) {
        const iy = py * ss + sy;
        const my = TM[iy], cby = TCB[iy], dy = TD[iy], dyq = TDQ[iy];
        const rowIn = cby >= 0 && cby < N, rowBase = cby * N;
        for (let sx = 0; sx < ss; sx++) {
          const ix = px * ss + sx;
          const mx = TM[ix];
          if (nearAnchor && inkAt(mx, my, L, 0)) continue;   // 앵커 잉크 = 검정(0,0,0)
          let R = 255, G = 255, B = 255;             // 기본 배경 = 흰색
          const cbx = TCB[ix];
          if (rowIn && cbx >= 0 && cbx < N) {
            const idx = rowBase + cbx, gv = cellG[idx];
            if (gv >= 0) {
              // within(dx,dy,dataDotR) 을 표에서 꺼낸 값으로 편 것 — 연산 순서까지 동일하다
              const q = TDQ[ix] + dyq;
              let hit;
              if (q < dotR2 * 0.9999999) hit = true;
              else if (q > dotR2 * 1.0000001) hit = false;
              else hit = Math.hypot(TD[ix], dy) <= dataDotR;
              if (hit) {
                if (cellCb && (cellCb[idx] || cellCr[idx])) { const c = ycc2rgb(gv, 128 + cellCb[idx], 128 + cellCr[idx]); R = c[0]; G = c[1]; B = c[2]; }
                else { R = G = B = gv; }
              }
            }
          }
          accR += R; accG += G; accB += B;
        }
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

  // 외곽 스트로크 — **옵션이 있을 때만**. 없으면 위 픽셀 루프 결과가 그대로 나간다(픽셀 불변).
  if (opts.outline && opts.outline.w > 0) {
    strokeOutline(data, dim, L, cellPx, quiet,
      opts.outline.gap != null ? opts.outline.gap : 1, opts.outline.w,
      opts.outline.rainbow || null);
  }
  return { img: { data, width: dim, height: dim }, width: dim, height: dim,
           grid, N, cellPx, quiet, anchors, toPx, spec };
}

module.exports = { SPEC, GRID_CELLS, mergeSpec, layout, render, makeRng, dataCells, reservedAt, BRIDGE, BRIDGE_POS, BRIDGE_LEGACY, SHAPE_GROOVES, bridgeRect, strokeOutline, RAINBOW_STOPS, RAINBOW_MAX_Y, _rainbowRGB,
  // ★§7-36: 로스터 자가시험(test-codec.js)이 실루엣 판정을 직접 호출하려면 필요하다.
  insideShape };
