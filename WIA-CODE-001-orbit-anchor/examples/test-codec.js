#!/usr/bin/env node
'use strict';
// 오르빗 데이터 왕복검증: payload → 렌더 → 열화 → 우리 위치확정 → 셀샘플 → 디코드 → 원문 일치?
const GEO = require('../reference/geometry.js');
const DEG = require('../reference/degrade.js');
const FR = require('../reference/frst.js');
const LOC = require('../reference/locate.js');
const COD = require('../reference/codec.js');

const CELLPX = 6;
const PAYLOAD = '홍익인간 · WIA CODE — 인터넷 없는 곳의 사람을 살리는 코드 · O RH+';

function detect(imgObj, layout, cellPx) {
  const gray = FR.toGray(imgObj);
  const radii = [1.5, 2.5, 3.5, 4.5, 5.5].map(r => r * cellPx);
  const fo = { gradFrac: 0.10, alpha: 2 }, po = { win: Math.round(1.6 * cellPx), topK: 30, thrFrac: 0.03 };
  const S = FR.frst(gray, radii, fo);
  const pk = FR.peaks(S, po);
  const redetect = (im) => { const g = FR.toGray(im); const s = FR.frst(g, radii, fo); return { gray: g, peaks: FR.peaks(s, po) }; };
  const res = LOC.locateRobust(imgObj, gray, pk, layout, { cellPx, redetect });
  return { gray, res };
}

let pass = 0, tot = 0;
function trial(grid, label, degFn) {
  const L = GEO.layout(grid, GEO.SPEC);
  const nCells = GEO.dataCells(L).length;
  const enc = COD.encodeToBits(PAYLOAD, nCells);
  const base = GEO.render({ grid, cellPx: CELLPX, quiet: 4, ss: 2, data: true, bits: enc.bits });
  const d = degFn(base.img);
  const { gray, res } = detect(d.img, L, CELLPX);
  tot++;
  if (!res.ok) { console.log(`  ✗ ${grid} ${label}: 위치확정 실패(${res.reason})`); return; }
  const out = COD.readCode(gray, res, L, CELLPX);
  const ok = out.ok && out.text === PAYLOAD;
  if (ok) pass++;
  const short = out.ok ? (out.text.slice(0, 14) + (out.text.length > 14 ? '…' : '')) : '';
  console.log(`  ${ok ? '✅' : '❌'} ${grid} ${label}: ${out.ok ? 'decode OK errs=' + out.errors + ' "' + short + '"' : '디코드실패(' + out.reason + ')'}` +
              `  [method ${res.method}, cap ${enc.capBytes}B/${enc.usedBytes}B, cells ${nCells}]`);
}

console.log('오르빗 데이터 왕복검증 — payload:', JSON.stringify(PAYLOAD.slice(0, 30) + '…'));
for (const g of ['S', 'M', 'L']) {
  console.log(` [${g}] 데이터셀 ${GEO.dataCells(GEO.layout(g, GEO.SPEC)).length}개`);
  trial(g, '무열화',      img => DEG.noise(img, 0, 7));
  trial(g, 'yaw40',      img => DEG.tilt(img, 40));
  trial(g, 'noise σ40',  img => DEG.noise(img, 40, 7));
  trial(g, 'rot30',      img => DEG.rotate(img, 30));
  trial(g, 'blur2',      img => DEG.boxBlur(img, 2, 3));
}
console.log(`\n합격 ${pass}/${tot}`);
process.exit(pass === tot ? 0 : 1);
