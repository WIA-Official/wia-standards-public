#!/usr/bin/env node
'use strict';
// 컬러(hue) 레이어 왕복 + 우아한 열화 + 용량. luma+hue 인코드 → 컬러렌더 → locate → readCode(hue).
const GEO = require('../reference/geometry.js');
const DEG = require('../reference/degrade.js');
const FR = require('../reference/frst.js');
const LOC = require('../reference/locate.js');
const COD = require('../reference/codec.js');

const CELLPX = 6;
const LUMA = '홍익인간 · WIA CODE luma 레이어';
const HUE = '색으로 추가된 hue 페이로드!';

function locate(imgObj, L, cellPx) {
  const gray = FR.toGray(imgObj);
  const radii = [1.5, 2.5, 3.5, 4.5, 5.5].map(r => r * cellPx);
  const fo = { gradFrac: 0.10, alpha: 2 }, po = { win: Math.round(1.6 * cellPx), topK: 30, thrFrac: 0.03 };
  const S = FR.frst(gray, radii, fo), pk = FR.peaks(S, po);
  const rd = (im) => { const g = FR.toGray(im); const s = FR.frst(g, radii, fo); return { gray: g, peaks: FR.peaks(s, po) }; };
  return { gray, res: LOC.locateRobust(imgObj, gray, pk, L, { cellPx, redetect: rd }) };
}

// 용량표
console.log('컬러 용량표 (순바이트, luma@50% + hue@50%):');
for (const g of ['S', 'M', 'L']) {
  const L = GEO.layout(g, GEO.SPEC), n = GEO.dataCells(L).length;
  const c1 = COD.encodeColor('x', 'x', n, 2, 1), c2 = COD.encodeColor('x', 'x', n, 2, 2), c3 = COD.encodeColor('x', 'x', n, 3, 2);
  const base = c1.lumaCapBytes;
  console.log(`  ${g}: 2L(${base}) · 2L+1H(${c1.lumaCapBytes + c1.hueCapBytes}, ×${((c1.lumaCapBytes + c1.hueCapBytes) / base).toFixed(2)}) · 2L+2H(${c2.lumaCapBytes + c2.hueCapBytes}, ×${((c2.lumaCapBytes + c2.hueCapBytes) / base).toFixed(2)}) · 3L+2H(${c3.lumaCapBytes + c3.hueCapBytes}, ×${((c3.lumaCapBytes + c3.hueCapBytes) / base).toFixed(2)})`);
}

console.log('\n왕복 + 우아한 열화:');
let pass = 0, tot = 0;
function trial(grid, hueBits, label, degFn) {
  const L = GEO.layout(grid, GEO.SPEC), n = GEO.dataCells(L).length;
  const enc = COD.encodeColor(LUMA, HUE, n, 2, hueBits);
  const rimg = GEO.render({ grid, cellPx: CELLPX, quiet: 4, ss: 2, data: true, cellGray: enc.cellGray, cellChroma: enc.cellChroma });
  const d = degFn(rimg.img);
  const { gray, res } = locate(d.img, L, CELLPX);
  tot++;
  if (!res.ok) { console.log(`  ✗ ${grid} ${hueBits}H ${label}: 위치확정 실패`); return; }
  const out = COD.readCode(gray, res, L, CELLPX, { hue: true, hueBits, rgbaImg: d.img });
  const lumaOk = out.ok && out.text === LUMA;
  const hueOk = out.hue && out.hue.text === HUE;
  if (lumaOk && hueOk) pass++;
  console.log(`  ${lumaOk && hueOk ? '✅' : (lumaOk ? '◐' : '❌')} ${grid} ${hueBits}H ${label}: luma ${lumaOk ? 'OK' : '✗'} · hue ${out.hue ? (hueOk ? 'OK errs' + out.hue.errors : '✗내용') : '✗디코드'}`);
}
for (const g of ['M', 'L']) {
  trial(g, 1, '무열화', img => DEG.noise(img, 0, 7));
  trial(g, 1, 'blur1', img => DEG.boxBlur(img, 1, 3));
  trial(g, 1, 'noise15', img => DEG.noise(img, 15, 7));
}
// 우아한 열화: 컬러 코드를 흑백(그레이)로만 읽어도 루마는 나와야
console.log('\n우아한 열화(흑백 스캔이어도 루마 복원):');
{
  const L = GEO.layout('M', GEO.SPEC), n = GEO.dataCells(L).length;
  const enc = COD.encodeColor(LUMA, HUE, n, 2, 1);
  const rimg = GEO.render({ grid: 'M', cellPx: CELLPX, quiet: 4, ss: 2, data: true, cellGray: enc.cellGray, cellChroma: enc.cellChroma });
  // toGray 로 흑백화 후 R=G=B (색 제거) → 그레이만으로 디코드
  const g = FR.toGray(rimg.img); const bw = { data: new Uint8ClampedArray(rimg.img.data.length), width: rimg.img.width, height: rimg.img.height };
  for (let i = 0; i < g.g.length; i++) { const v = g.g[i]; bw.data[i * 4] = bw.data[i * 4 + 1] = bw.data[i * 4 + 2] = v; bw.data[i * 4 + 3] = 255; }
  const { res } = locate(bw, L, CELLPX);
  const out = COD.readCode(FR.toGray(bw), res, L, CELLPX);
  console.log(`  흑백만 디코드: luma ${out.ok && out.text === LUMA ? '✅ OK' : '❌'} (hue 없이도 완전한 payload)`);
}
console.log(`\n컬러 왕복 합격 ${pass}/${tot}`);
