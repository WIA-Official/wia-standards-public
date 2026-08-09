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
