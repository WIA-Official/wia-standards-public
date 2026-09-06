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
 *          VER=2 는 payload 자리에 **세그먼트 비트열**(숫자·URL·한글 압축) — 아래 P3 절.
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
  if (bytes.length < 6 || bytes[0] !== MAGIC) return null;
  const ver = bytes[1];
  if (ver !== VER && ver !== VER2) return null;      // 모르는 버전 → 깨끗이 실패(오염 아님)
  const len = (bytes[2] << 8) | bytes[3];
  if (4 + len + 2 > bytes.length) return null;
  const body = bytes.subarray(0, 4 + len);
  const crc = I.crc16(body) & 0xffff;
  const got = (bytes[4 + len] << 8) | bytes[4 + len + 1];
  if (crc !== got) return null;
  if (ver === VER) return body.subarray(4, 4 + len);
  // VER=2 — 세그먼트 비트열을 풀어 UTF-8 바이트로 돌려준다(호출부 인터페이스 무변경).
  const text = segDecode(body.subarray(4, 4 + len));
  if (text === null) return null;
  return toBytes(text);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  세그먼트 모드 코덱 (프레임 VER=2) — P3 / MASTER LINE F7
 * ══════════════════════════════════════════════════════════════════════════
 *  VER=1 은 payload 를 UTF-8 바이트열로만 싣는다. 숫자·URL·한글은 그 표현이
 *  낭비다(숫자 1자리 8비트, 한글 1음절 24비트). VER=2 는 Han Xin/QR 식으로
 *  **내용물 종류별 비트폭**을 쓰고, 어떤 종류로 자를지는 DP 로 최적 분할한다.
 *
 *  프레임: [0x57 'W', 0x02 ver, lenHi, lenLo] + 세그먼트비트열(len 바이트) + crc16(2)
 *          — 껍데기는 VER=1 과 완전히 같다(RS·인터리브·산포·CRC 무변경).
 *
 *  세그먼트 비트열(MSB-first):
 *      flags   2b   예약(0 고정). ★P8 마스크 시드 2비트의 자리 — 지금 디코더는 무시한다.
 *      [ mode 3b + count 12b + body ] * n
 *      mode 0(END) 로 끝내고 바이트 경계까지 0 으로 채운다.
 *
 *  모드:
 *    0 END   종결
 *    1 N     숫자 '0'-'9'          3자리→10b, 나머지 1자리→4b / 2자리→7b
 *    2 A     소문자 URL 45자표      2자→11b, 홀수 끝 1자→6b
 *    3 H     한글음절 11172 + ASCII 128 = 11300  2자→27b(13.5b/자), 홀수 끝→14b
 *    4 B     UTF-8 바이트          8b/바이트 (count 는 **바이트 수**)
 *    5 C7    ASCII 0x00-0x7F       7b/자
 *    6 S64   base64url 64자표      6b/자
 *    7 —     예약(만나면 해독 실패)
 *
 *  ★설계가 원안(PROPOSALS_FABLE §P3)에서 벗어난 곳은 이 파일 하단 주석과
 *    `test-codec-seg.js` 의 측정표에 근거와 함께 적혀 있다. 요약:
 *    ① count 는 전 모드 **12비트 통일**(원안의 A:11b·H:10b 는 최대용량에서 넘친다)
 *    ② N 은 count 가 있으므로 Han Xin 종결자를 쓰지 않는다(중복) — QR 식 4/7b 잔여
 *    ③ H 는 14b 단일 대신 **27b/2자(13.5b)** + 알파벳에 ASCII 포함
 *       (ASCII 를 빼면 띄어쓰기마다 모드가 끊겨 한글 산문에서 이득이 1.05배로 죽는다)
 *    ④ C7·S64 는 원안에 없던 추가 모드(자격증 페이로드 1.05→1.19배)
 * ══════════════════════════════════════════════════════════════════════════ */
const VER2 = 0x02;
const SEG_END = 0, SEG_N = 1, SEG_A = 2, SEG_H = 3, SEG_B = 4, SEG_C7 = 5, SEG_S64 = 6;
const SEG_MODE_BITS = 3, SEG_CNT_BITS = 12, SEG_MAX_COUNT = (1 << SEG_CNT_BITS) - 1;
const SEG_HDR_BITS = SEG_MODE_BITS + SEG_CNT_BITS;          // 15
const SEG_FLAG_BITS = 2;
// A 표 45자 = a-z(26) + 0-9(10) + 구두점 9. 45² = 2025 ≤ 2048 이라 2자가 11비트에 든다.
//   ★구두점 9칸의 선정은 실측(test-codec-seg.js §A): URL 필수 8개(- . _ : / ? & =) +
//     9번째는 **공백**. 공백은 URL 에 안 나오므로 URL 이득을 1도 깎지 않으면서
//     소문자 산문을 0.99배(손해)에서 1.44배로 바꾼다. 탈락: ~ # % (우리 페이로드 빈도 낮음).
const A_TABLE = 'abcdefghijklmnopqrstuvwxyz0123456789-._:/?&= ';
const S64_TABLE = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_';
const HAN_BASE = 0xAC00, HAN_CNT = 11172, H_ASCII = 128, H_CNT = HAN_CNT + H_ASCII;  // 11300
// 11300² = 127,689,999 < 2²⁷ — 두 자가 27비트에 든다(13.5b/자). 14b 단일보다 3.6% 낫다.
const _A_IDX = (() => { const m = new Map(); for (let i = 0; i < A_TABLE.length; i++) m.set(A_TABLE.codePointAt(i), i); return m; })();
const _S_IDX = (() => { const m = new Map(); for (let i = 0; i < S64_TABLE.length; i++) m.set(S64_TABLE.codePointAt(i), i); return m; })();

function _bitWriter() {
  const out = []; let acc = 0, n = 0;
  return {
    put(v, nb) { for (let i = nb - 1; i >= 0; i--) { acc = ((acc << 1) | ((v >>> i) & 1)) & 255; if (++n === 8) { out.push(acc); acc = 0; n = 0; } } },
    bits() { return out.length * 8 + n; },
    finish() { if (n) { out.push((acc << (8 - n)) & 255); acc = 0; n = 0; } return Uint8Array.from(out); },
  };
}
function _bitReader(u8) {
  let p = 0; const total = u8.length * 8;
  return {
    left() { return total - p; },
    get(nb) { let v = 0; for (let i = 0; i < nb; i++) { v = v * 2 + ((u8[p >> 3] >>> (7 - (p & 7))) & 1); p++; } return v; },
    restZero() { while (p < total) { if ((u8[p >> 3] >>> (7 - (p & 7))) & 1) return false; p++; } return true; },
  };
}
function _u8len(cp) { return cp < 0x80 ? 1 : cp < 0x800 ? 2 : cp < 0x10000 ? 3 : 4; }
function _hIdx(cp) { return (cp >= HAN_BASE && cp < HAN_BASE + HAN_CNT) ? cp - HAN_BASE : (cp < 0x80 ? HAN_CNT + cp : -1); }
function _hCp(ix) { return ix < HAN_CNT ? HAN_BASE + ix : ix - HAN_CNT; }

// 모드 서술자. six = 문자당 비용(1/6비트 단위) — 3·2 자 묶음의 최소공배수가 6이라 정수화된다.
const SEG_MODES = [
  { id: SEG_N,   six: 20,   ok: cp => cp >= 48 && cp <= 57 },                       // 10b/3자
  { id: SEG_A,   six: 33,   ok: cp => _A_IDX.has(cp) },                             // 11b/2자
  { id: SEG_H,   six: 81,   ok: cp => _hIdx(cp) >= 0 },                             // 27b/2자
  { id: SEG_S64, six: 36,   ok: cp => _S_IDX.has(cp) },                             // 6b/자
  { id: SEG_C7,  six: 42,   ok: cp => cp < 0x80 },                                  // 7b/자
  { id: SEG_B,   six: null, ok: () => true },                                       // 8b/바이트
];
const _SEG_HDR_SIX = SEG_HDR_BITS * 6;

/* 문자열 → 세그먼트 목록. Han Xin `hx_define_modes` 와 같은 DP(비용 1/6비트).
 *   묶음 경계(3자·2자)의 반올림은 무시한다 — QR/Han Xin 도 같다. 오차 상한은
 *   세그먼트당 N 6b · A 5b · H 13b 이고, 그 대가로 O(L·6) 로 끝난다. */
function segPlan(text) {
  const cp = [];
  for (const ch of text) cp.push(ch.codePointAt(0));
  const L = cp.length;
  if (!L) return [];
  const K = SEG_MODES.length, INF = Infinity;
  const w = (m, c) => m.six === null ? 48 * _u8len(c) : m.six;
  let prev = new Array(K).fill(_SEG_HDR_SIX);
  const from = new Int8Array(L * K);
  for (let i = 0; i < L; i++) {
    let bi = -1, bv = INF;
    for (let k = 0; k < K; k++) if (prev[k] < bv) { bv = prev[k]; bi = k; }
    const cur = new Array(K).fill(INF);
    for (let k = 0; k < K; k++) {
      const M = SEG_MODES[k];
      if (!M.ok(cp[i])) continue;
      const stay = prev[k], sw = bv + _SEG_HDR_SIX;
      if (stay <= sw) { cur[k] = stay + w(M, cp[i]); from[i * K + k] = k; }
      else { cur[k] = sw + w(M, cp[i]); from[i * K + k] = bi; }
    }
    prev = cur;
  }
  let bi = -1, bv = INF;
  for (let k = 0; k < K; k++) if (prev[k] < bv) { bv = prev[k]; bi = k; }
  const pick = new Int8Array(L);
  for (let i = L - 1; i >= 0; i--) { pick[i] = bi; bi = from[i * K + bi]; }
  // 같은 모드 연속을 하나의 세그먼트로 묶고, count 상한(4095)에서 자른다.
  const segs = [];
  let s = 0;
  for (let i = 1; i <= L; i++) {
    if (i === L || pick[i] !== pick[s]) { segs.push({ mode: SEG_MODES[pick[s]].id, from: s, to: i, cps: cp.slice(s, i) }); s = i; }
  }
  const out = [];
  for (const g of segs) {
    if (g.mode === SEG_B) {                       // count 가 바이트 수라 바이트로 자른다
      let acc = 0, st = 0;
      for (let i = 0; i < g.cps.length; i++) {
        const n = _u8len(g.cps[i]);
        if (acc + n > SEG_MAX_COUNT) { out.push({ mode: SEG_B, cps: g.cps.slice(st, i) }); st = i; acc = 0; }
        acc += n;
      }
      out.push({ mode: SEG_B, cps: g.cps.slice(st) });
    } else {
      const step = g.mode === SEG_N ? 4095 - (4095 % 3) : SEG_MAX_COUNT - (SEG_MAX_COUNT % 2);  // 묶음 경계에서 자른다
      for (let i = 0; i < g.cps.length; i += step) out.push({ mode: g.mode, cps: g.cps.slice(i, i + step) });
    }
  }
  return out;
}

/* 세그먼트 목록 → 비트열 바이트. */
function segEncode(text) {
  const segs = segPlan(text), bw = _bitWriter();
  bw.put(0, SEG_FLAG_BITS);                       // flags 예약 = 0
  for (const g of segs) {
    const c = g.cps, n = c.length;
    if (g.mode === SEG_B) {
      const by = toBytes(String.fromCodePoint.apply(String, c));
      bw.put(SEG_B, SEG_MODE_BITS); bw.put(by.length, SEG_CNT_BITS);
      for (let i = 0; i < by.length; i++) bw.put(by[i], 8);
      continue;
    }
    bw.put(g.mode, SEG_MODE_BITS); bw.put(n, SEG_CNT_BITS);
    if (g.mode === SEG_N) {
      let i = 0;
      for (; i + 3 <= n; i += 3) bw.put((c[i] - 48) * 100 + (c[i + 1] - 48) * 10 + (c[i + 2] - 48), 10);
      const r = n - i;
      if (r === 1) bw.put(c[i] - 48, 4);
      else if (r === 2) bw.put((c[i] - 48) * 10 + (c[i + 1] - 48), 7);
    } else if (g.mode === SEG_A) {
      let i = 0;
      for (; i + 2 <= n; i += 2) bw.put(_A_IDX.get(c[i]) * 45 + _A_IDX.get(c[i + 1]), 11);
      if (i < n) bw.put(_A_IDX.get(c[i]), 6);
    } else if (g.mode === SEG_H) {
      let i = 0;
      for (; i + 2 <= n; i += 2) bw.put(_hIdx(c[i]) * H_CNT + _hIdx(c[i + 1]), 27);
      if (i < n) bw.put(_hIdx(c[i]), 14);
    } else if (g.mode === SEG_C7) {
      for (let i = 0; i < n; i++) bw.put(c[i], 7);
    } else if (g.mode === SEG_S64) {
      for (let i = 0; i < n; i++) bw.put(_S_IDX.get(c[i]), 6);
    } else throw new Error('세그먼트 모드 미상: ' + g.mode);
  }
  bw.put(SEG_END, SEG_MODE_BITS);
  return bw.finish();
}

/* 비트열 바이트 → 문자열. 규격 위반이면 **null**(엉뚱한 문자열을 내지 않는다). */
function segDecode(u8) {
  const br = _bitReader(u8);
  if (br.left() < SEG_FLAG_BITS) return null;
  br.get(SEG_FLAG_BITS);                          // flags — 예약, 지금은 무시(전방호환)
  const parts = [];
  for (;;) {
    if (br.left() < SEG_MODE_BITS) return br.restZero() ? parts.join('') : null;
    const mode = br.get(SEG_MODE_BITS);
    if (mode === SEG_END) return br.restZero() ? parts.join('') : null;
    if (mode > SEG_S64) return null;              // 예약 모드 → 깨끗이 실패
    if (br.left() < SEG_CNT_BITS) return null;
    const n = br.get(SEG_CNT_BITS);
    let need;
    if (mode === SEG_N) { const f = Math.floor(n / 3), r = n % 3; need = f * 10 + (r === 1 ? 4 : r === 2 ? 7 : 0); }
    else if (mode === SEG_A) need = (n >> 1) * 11 + (n & 1 ? 6 : 0);
    else if (mode === SEG_H) need = (n >> 1) * 27 + (n & 1 ? 14 : 0);
    else if (mode === SEG_B) need = n * 8;
    else if (mode === SEG_C7) need = n * 7;
    else need = n * 6;
    if (br.left() < need) return null;
    if (mode === SEG_N) {
      let i = 0, s = '';
      for (; i + 3 <= n; i += 3) { const v = br.get(10); if (v > 999) return null; s += String(v).padStart(3, '0'); }
      const r = n - i;
      if (r === 1) { const v = br.get(4); if (v > 9) return null; s += String(v); }
      else if (r === 2) { const v = br.get(7); if (v > 99) return null; s += String(v).padStart(2, '0'); }
      parts.push(s);
    } else if (mode === SEG_A) {
      let i = 0, s = '';
      for (; i + 2 <= n; i += 2) { const v = br.get(11); if (v >= 45 * 45) return null; s += A_TABLE[(v / 45) | 0] + A_TABLE[v % 45]; }
      if (i < n) { const v = br.get(6); if (v >= 45) return null; s += A_TABLE[v]; }
      parts.push(s);
    } else if (mode === SEG_H) {
      let i = 0, s = '';
      for (; i + 2 <= n; i += 2) { const v = br.get(27); if (v >= H_CNT * H_CNT) return null; s += String.fromCodePoint(_hCp(Math.floor(v / H_CNT)), _hCp(v % H_CNT)); }
      if (i < n) { const v = br.get(14); if (v >= H_CNT) return null; s += String.fromCodePoint(_hCp(v)); }
      parts.push(s);
    } else if (mode === SEG_B) {
      const by = new Uint8Array(n);
      for (let i = 0; i < n; i++) by[i] = br.get(8);
      parts.push(fromBytes(by));
    } else if (mode === SEG_C7) {
      let s = '';
      for (let i = 0; i < n; i++) s += String.fromCharCode(br.get(7));
      parts.push(s);
    } else {
      let s = '';
      for (let i = 0; i < n; i++) s += S64_TABLE[br.get(6)];
      parts.push(s);
    }
  }
}

/* 세그먼트 프레임(VER=2). 껍데기·CRC 는 VER=1 과 동일. */
function frameEncodeSeg(text) {
  const seg = segEncode(text);
  if (seg.length > 0xffff) throw new Error('세그먼트 길이초과: ' + seg.length + 'B');
  const body = new Uint8Array(4 + seg.length);
  body[0] = MAGIC; body[1] = VER2; body[2] = (seg.length >>> 8) & 255; body[3] = seg.length & 255;
  body.set(seg, 4);
  const crc = I.crc16(body) & 0xffff;
  const out = new Uint8Array(body.length + 2);
  out.set(body, 0); out[body.length] = (crc >>> 8) & 255; out[body.length + 1] = crc & 255;
  return out;
}

/* 인코더가 쓸 프레임을 고른다.
 *   ver=1(기본)  : 현행 그대로 — **옛 스캐너가 읽는다**
 *   ver=2        : 무조건 세그먼트
 *   ver='auto'   : 더 짧은 쪽(동률이면 1) — 바이트 콘텐츠는 자동으로 VER=1 이라 불변
 *   ★기본값을 'auto' 로 넘기는 것이 곧 "표준 채택" 이다. 스캐너 배포가 끝난 뒤
 *     한 줄로 바꾼다(그전에 바꾸면 옛 스캐너가 숫자·한글 코드를 못 읽는다). */
function pickFrame(text, opts) {
  // ★2026-09-05 P3 ON(오너 결정): 기본 'auto' — v2 가 더 짧을 때만 v2, 아니면 v1 바이트동일.
  //   v1 로 고정하려면 opts.ver=1. 스캐너·킷 엔진·API 는 같은 codec 을 쓰므로 함께 바뀐다.
  const ver = (opts && opts.ver) || 'auto';
  if (ver === 2) return frameEncodeSeg(text);
  if (ver !== 'auto') return frameEncode(toBytes(text));
  const f1 = frameEncode(toBytes(text));
  let f2 = null;
  try { f2 = frameEncodeSeg(text); } catch (e) { f2 = null; }
  return (f2 && f2.length < f1.length) ? f2 : f1;
}

// nCells(데이터셀 수) → RS 계획(인코드/디코드 공용, 결정적)
function planFor(nCells) { return I.planBlocks(Math.floor(nCells / 8), ECC); }

// payload 문자열 → 데이터셀 비트배열(길이 nCells). capacity 초과 시 예외.
function encodeToBits(text, nCells, opts) {
  const frame = pickFrame(text, opts);
  const plan = planFor(nCells);
  if (frame.length > plan.totalK) throw new Error('용량초과: ' + frame.length + 'B > ' + plan.totalK + 'B');
  const dataIn = new Uint8Array(plan.totalK); dataIn.set(frame, 0);   // 나머지 0패딩
  let cw = I.rsEncodeAll(dataIn, plan);
  cw = scramble(I.interleaveBytes(cw, plan));
  const bits = new Uint8Array(nCells);
  const nb = Math.min(cw.length * 8, nCells), pos = buildPerm(nCells);
  for (let k = 0; k < nb; k++) bits[pos[k]] = (cw[k >> 3] >>> (7 - (k & 7))) & 1;   // 위치 산포
  return { bits: bits, plan: plan, usedBytes: frame.length, capBytes: plan.totalK, ver: frame[1] };
}

/* ★2026-08-30 B단계 — 오염이 **확실한 셀**들을 RS 소거 위치로 옮긴다.
 *   RS 는 위치를 모르면 예산의 절반을 찾는 데 쓴다(오류 nsym/2 vs 소거 nsym).
 *   위성 하나가 가려진 것을 locateSim3 가 이미 알므로, 그 주변 셀을 여기로 넘기면
 *   같은 패리티로 **2배**를 복구한다(실측: 블록당 27 → 55 바이트).
 *   ★buildPerm 이 인접 셀을 비트스트림에서 흩어 놓으므로 **그 산포를 그대로 따라가야** 한다 —
 *     셀 번호를 바이트 번호로 그냥 쓰면 엉뚱한 자리를 소거로 표시한다.
 *   반환: 블록 인덱스별 소거 바이트 위치 배열. */
function cellsToErasures(badCells, nCells, plan, bpc) {
  bpc = bpc || 1;
  if (!badCells || !badCells.length) return null;
  const bad = new Uint8Array(nCells);
  for (const c of badCells) if (c >= 0 && c < nCells) bad[c] = 1;
  const pos = buildPerm(nCells);
  const totalN = plan.totalN, nb = totalN * 8;
  // 셀 → 비트 → 코드워드 바이트. **decodeFromCells 와 완전히 같은 순서**로 돈다
  //   (bpc 가 1이 아니면 셀 하나가 여러 비트를 낸다).
  const cwBad = new Uint8Array(totalN);
  let bit = 0;
  for (let k = 0; k < nCells && bit < nb; k++) {
    const dirty = bad[pos[k]];
    for (let b = bpc - 1; b >= 0 && bit < nb; b--) { if (dirty) cwBad[bit >> 3] = 1; bit++; }
  }
  // scramble 은 값만 XOR 하므로 위치가 안 바뀐다. deinterleave 는 위치를 바꾼다.
  const seqBad = I.deinterleaveBytes(cwBad, plan);
  // 블록별로 나눈다 — rsDecodeAll 과 **같은 순서**(데이터 전부 → 패리티 전부)
  const out = [];
  let off = 0;
  const dOff = [], pOff = [];
  for (const b of plan.blocks) { dOff.push(off); off += b.k; }
  for (const b of plan.blocks) { pOff.push(off); off += b.nsym; }
  for (let i = 0; i < plan.blocks.length; i++) {
    const b = plan.blocks[i], e = [];
    for (let j = 0; j < b.k; j++) if (seqBad[dOff[i] + j]) e.push(j);
    for (let j = 0; j < b.nsym; j++) if (seqBad[pOff[i] + j]) e.push(b.k + j);
    out.push(e);
  }
  return out;
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
const PAL_ECC = { 1: '25%', 2: '35%', 3: '50%' };     // 단계 많을수록 노이즈↑ → ECC 상향(원근 far쪽 강건)
// ★bpc2 는 2026-08-16 에 50%→35% (다리QR 존 예약으로 예산 낭비가 사라진 뒤). 순용량 +30~31%.
//   열화 스윕 실측(하트/사각/원형, 다리 on·off): 노이즈·블러 파단점은 50%와 완전 동일 —
//   그 구간의 실패는 예비분 고갈이 아니라 흑백 대비붕괴라 ECC 를 올려도 못 살린다. 유일한 손실은
//   극단 원근(50°→48°, 2°). 반대로 50% 는 정정능력 절반을 놀리고 용량만 30% 깎고 있었다.
// 구형 호환: 그 이전 bpc2 코드는 50% 로 인코딩됐다 → 디코더는 두 비율을 모두 시도한다.
//   ★"먼저 성공한 쪽"이 아니라 "errors 가 적은 쪽"을 채택해야 한다. payload 가 짧으면 셀 대부분이
//   패딩이라 틀린 비율로도 RS 가 억지로 복원해내는데(실측 76 errors 소모), 그대로 채택하면
//   정정예산을 태운 채 성공한 것이라 실제 노이즈가 얹히면 그때 터진다.
const ECC_TRY = { 1: ['25%'], 2: ['35%', '50%'], 3: ['50%'] };
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
function encodeToCells(text, nCells, bpc, ecc, opts) {
  bpc = bpc || 1;
  const frame = pickFrame(text, opts);
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
  return { cells: symbols, cellGray: cellGray, plan: plan, usedBytes: frame.length, capBytes: plan.totalK, bitsPerCell: bpc, ver: frame[1] };
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
function decodeFromCells(symbols, nCells, bpc, ecc, badCells) {
  const plan = I.planBlocks(Math.floor(nCells * bpc / 8), ecc || PAL_ECC[bpc] || '25%');
  const totalN = plan.totalN, cw = new Uint8Array(totalN), nb = totalN * 8;
  const pos = buildPerm(nCells);                 // 위치 산포 역변환: 데이터유닛 k = symbols[pos[k]]
  let bit = 0;
  for (let k = 0; k < nCells && bit < nb; k++) {
    const v = symbols[pos[k]];
    for (let b = bpc - 1; b >= 0 && bit < nb; b--) { if ((v >> b) & 1) cw[bit >> 3] |= (1 << (7 - (bit & 7))); bit++; }
  }
  const seq = I.deinterleaveBytes(scramble(cw), plan);
  let res = null;
  // ★소거 우선 — 오염이 확실한 셀을 알면 예산이 2배가 된다(블록당 nsym/2 → nsym).
  //   실패하면 조용히 옛 경로로 떨어진다(소거 위치가 틀렸을 수도 있으므로).
  const eras = badCells && badCells.length ? cellsToErasures(badCells, nCells, plan, bpc) : null;
  if (eras) { try { res = rsDecodeAllErased(seq, plan, eras); } catch (e) { res = null; } }
  if (!res) { try { res = I.rsDecodeAll(seq, plan); } catch (e) { return { ok: false, reason: 'rs-uncorrectable' }; } }
  const payload = frameDecode(res.data);
  if (!payload) return { ok: false, reason: 'bad-frame/crc', errors: res.errors };
  return { ok: true, text: fromBytes(payload), errors: res.errors, erased: res.erasures || 0 };
}

/* 블록별 소거 위치를 받아 복호한다. rsDecodeAll 과 **같은 블록 순서**를 쓴다. */
function rsDecodeAllErased(cw, plan, erasByBlock) {
  const dp = [], pp = []; let off = 0;
  for (const b of plan.blocks) { dp.push(cw.subarray(off, off + b.k)); off += b.k; }
  for (const b of plan.blocks) { pp.push(cw.subarray(off, off + b.nsym)); off += b.nsym; }
  const out = new Uint8Array(plan.totalK);
  let oo = 0, errs = 0, ers = 0;
  for (let i = 0; i < plan.blocks.length; i++) {
    const b = plan.blocks[i], blk = new Uint8Array(b.k + b.nsym);
    blk.set(dp[i], 0); blk.set(pp[i], b.k);
    const r = I.decodeBlock(blk, b.nsym, erasByBlock[i]);
    out.set(r.data, oo); oo += b.k; errs += r.errors; ers += (r.erasures || 0);
  }
  return { data: out, errors: errs, erasures: ers };
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
/* ★2026-08-30 Q1-C — 역합성곱 결과 재사용 캐시.
 *   프로파일러 실측: 실패 프레임 22초 중 **14.4초(66%)가 deconvGauss** 였다.
 *   원인은 알고리즘이 아니라 **같은 계산의 반복**이다 — decodePSF 의 `obs` 와 `dec` 는
 *   (grayObj, H, N, sigma) 로만 정해지고 **모양과 무관**한데, detectAuto 의 pass2 는
 *   같은 후보로 26모양 × 2변형을 순회하며 매번 처음부터 다시 계산했다(최대 104회).
 *   모양은 `dec[cells[i]]` 로 **어느 셀을 읽을지**만 정한다.
 *   호출이 연속이라 **크기 1 캐시**로 충분하다(같은 후보의 52회가 연달아 온다). */
var _psfCache = null;   // { gray, H, N, obs, decs:{sigma: Float64Array} }
function _sameH(a, b) {
  if (a === b) return true;
  if (!a || !b || a.length !== b.length) return false;
  for (var i = 0; i < a.length; i++) if (a[i] !== b[i]) return false;
  return true;
}
function _psfGet(grayObj, H, N, sigma) {
  if (_psfCache && _psfCache.gray === grayObj && _psfCache.N === N && _sameH(_psfCache.H, H)) {
    if (_psfCache.decs[sigma]) return _psfCache.decs[sigma];
  } else {
    _psfCache = { gray: grayObj, H: H, N: N, obs: sampleModuleGrid(grayObj, H, N), decs: {} };
  }
  var d = deconvGauss(_psfCache.obs, N, sigma, 45);
  _psfCache.decs[sigma] = d;
  return d;
}

// 표준 셀판독이 블러로 실패했을 때 재시도(1비트). 통과 σ 반환, 실패 시 null.
function decodePSF(grayObj, H, layout, cells) {
  const N = layout.N, dc = new Float32Array(cells.length);
  for (const sig of [0.5, 0.75]) {
    const dec = _psfGet(grayObj, H, N, sig);
    for (let i = 0; i < cells.length; i++) dc[i] = dec[cells[i][1] * N + cells[i][0]];
    const cl = classifyCells(dc, cells, 1, layout);
    if (cl.hi - cl.lo < 25) continue;
    const out = decodeFromCells(cl.sym, cells.length, 1);
    if (out.ok) { out.psfSigma = sig; return out; }
  }
  return null;
}

/* ★2026-08-30 — RS 소거(erasure) 배관은 **남겨 두되, 부르는 곳은 없다.**
 *   `rs.js decodeBlock(cw,nsym,erasures)` + `cellsToErasures` + `decodeFromCells(...,badCells)`
 *   까지는 검증돼 있고(합성 시험: 오류만 250셀 벽 → 소거 350셀, 약 1.4배), 소거를 안 주면
 *   동작이 한 글자도 안 바뀐다. 그러나 **오염 자리를 짚는 층은 전부 걷어냈다** —
 *   회색지대 추정·앵커 근처 원판·구역 대비붕괴 셋 다 실측에서 경계를 한 칸도 못 옮겼다.
 *   왜 안 되는지는 `~/wiacode-perf/SCANNER_AIM_FINDINGS_2026-08-29.md` §I 에 있다.
 *   요약: 실제 광학 손상은 소거가 이기는 구간(오염 250~350셀)에 **거의 안 떨어진다** —
 *   작으면 평범한 RS 가 이미 살리고, 크면 소거 예산(513셀)도 넘긴다(실측 786셀).
 *   다시 쓰려면 `decodeFromCells(sym, n, bpc, ecc, badCells)` 에 **확실한** 자리를 주면 된다. */

function readCode(grayObj, res, layout, cellPx, opts) {
  if (!res || !res.ok || !res.Hmod2img) return { ok: false, reason: 'no-lock' };
  const s = sampleCellGrays(grayObj, res.Hmod2img, layout, cellPx);
  let tries = (opts && opts.bpcTry) || [1, 2, 3];
  for (const bpc of tries) {
    const cl = classifyCells(s.grays, s.cells, bpc, layout);
    if (cl.hi - cl.lo < 25) continue;                    // 대비붕괴 → 이 모드 스킵
    // ECC 비율 trial(신형35%/구형50%) — 오류가 적은 쪽 채택. 추가비용 실측 3.2ms.
    let out = null, outEcc = null;
    for (const ecc of (ECC_TRY[bpc] || [PAL_ECC[bpc]])) {
      const r = decodeFromCells(cl.sym, s.cells.length, bpc, ecc);
      if (r.ok && (!out || r.errors < out.errors)) { out = r; outEcc = ecc; }
    }
    if (out) {
      out.bitsPerCell = bpc; out.ecc = outEcc; out.range = { lo: Math.round(cl.lo), hi: Math.round(cl.hi) };
      // ── hue 평면(있으면) — 루마 성공 후에만. 실패해도 루마 결과는 그대로 반환(우아한 열화). ──
      if (opts && opts.hue && opts.rgbaImg) {
        try {
          const hueBits = opts.hueBits || 1;
          // ★루마와 반드시 같은 ECC 로 재인코딩해야 한다(outEcc 누락 시 컬러 코드만 조용히 깨짐).
          const canon = encodeToCells(out.text, s.cells.length, bpc, outEcc);   // 오류정정된 정규 루마심볼
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
  encodeToCells, decodeFromCells, calibrate, classifyCells, levelGray, grayEnc, grayDec, PAL_ECC, ECC_TRY,
  encodeColor, eligibleFromLuma, sampleCellColor, calibrateColor, classifyHue, hueChroma, HUE_ECC, HUE_RHO,
  encodeBytesToCells, decodeBytesFromCells, encodeColorBytes,
  // ── P3 세그먼트 모드(VER=2) ──
  VER, VER2, MAGIC, frameEncodeSeg, pickFrame, segEncode, segDecode, segPlan,
  A_TABLE, S64_TABLE, SEG_MODES, SEG_END, SEG_N, SEG_A, SEG_H, SEG_B, SEG_C7, SEG_S64,
  SEG_CNT_BITS, SEG_MAX_COUNT, SEG_HDR_BITS, H_CNT, HAN_CNT,
};
