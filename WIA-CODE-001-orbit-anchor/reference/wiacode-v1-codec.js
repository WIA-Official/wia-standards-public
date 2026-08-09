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
