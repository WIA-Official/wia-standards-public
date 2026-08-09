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
