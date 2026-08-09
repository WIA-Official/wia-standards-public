#!/usr/bin/env node
'use strict';
// 다단계 그레이 셀 왕복 + 용량 + 열화 견딤. encodeToCells → render(cellGray) → 열화 → readCode(trial).
const GEO = require('../reference/geometry.js');
const DEG = require('../reference/degrade.js');
const FR = require('../reference/frst.js');
const LOC = require('../reference/locate.js');
const COD = require('../reference/codec.js');

const CELLPX = 6;
const PAY = '홍익인간 · WIA CODE 다단계 · 인터넷 없는 곳의 사람을 살리는 코드';

function detect(imgObj, layout, cellPx) {
  const gray = FR.toGray(imgObj);
  const radii = [1.5, 2.5, 3.5, 4.5, 5.5].map(r => r * cellPx);
  const fo = { gradFrac: 0.10, alpha: 2 }, po = { win: Math.round(1.6 * cellPx), topK: 30, thrFrac: 0.03 };
  const S = FR.frst(gray, radii, fo), pk = FR.peaks(S, po);
  const redetect = (im) => { const g = FR.toGray(im); const s = FR.frst(g, radii, fo); return { gray: g, peaks: FR.peaks(s, po) }; };
  return { gray, res: LOC.locateRobust(imgObj, gray, pk, layout, { cellPx, redetect }) };
}

console.log('다단계 용량 표 (payload=' + JSON.stringify(PAY.slice(0, 16) + '…') + ')');
console.log('grid │ 1비트  2비트  3비트  (셀당 비트 → 순용량 바이트)');
for (const g of ['S', 'M', 'L']) {
  const L = GEO.layout(g, GEO.SPEC), nCells = GEO.dataCells(L).length;
  const caps = [1, 2, 3].map(bpc => COD.encodeToCells('x', nCells, bpc).capBytes - 6);
  console.log(` ${g}   │ ${String(caps[0]).padStart(5)} ${String(caps[1]).padStart(6)} ${String(caps[2]).padStart(6)}   (셀 ${nCells})`);
}

console.log('\n왕복 + 열화 견딤 (✅=원문복원, 괄호=검출된 bpc):');
let pass = 0, tot = 0;
function trial(grid, bpc, label, degFn) {
  const L = GEO.layout(grid, GEO.SPEC), nCells = GEO.dataCells(L).length;
  const enc = COD.encodeToCells(PAY, nCells, bpc);
  const base = GEO.render({ grid, cellPx: CELLPX, quiet: 4, ss: 2, data: true, cellGray: enc.cellGray });
  const d = degFn(base.img);
  const { gray, res } = detect(d.img, L, CELLPX);
  tot++;
  if (!res.ok) { console.log(`  ✗ ${grid} ${bpc}bpc ${label}: 위치확정 실패`); return; }
  const out = COD.readCode(gray, res, L, CELLPX);
  const ok = out.ok && out.text === PAY && out.bitsPerCell === bpc;
  if (ok) pass++;
  console.log(`  ${ok ? '✅' : '❌'} ${grid} ${bpc}bpc ${label}: ${out.ok ? 'OK(bpc' + out.bitsPerCell + ' errs' + out.errors + ')' : '실패(' + out.reason + ')'}`);
}
for (const g of ['S', 'M', 'L']) {
  for (const bpc of [2, 3]) {
    trial(g, bpc, '무열화', img => DEG.noise(img, 0, 7));
    trial(g, bpc, 'yaw30', img => DEG.tilt(img, 30));
    trial(g, bpc, 'blur1', img => DEG.boxBlur(img, 1, 3));
    trial(g, bpc, 'noise20', img => DEG.noise(img, 20, 7));
  }
}
console.log(`\n다단계 왕복 합격 ${pass}/${tot}`);
