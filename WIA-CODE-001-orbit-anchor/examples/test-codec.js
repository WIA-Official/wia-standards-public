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
  // ★2026-09-05 P3: 이 파일은 **맨몸 locator(locateRobust 직접) + RS 파이프라인** 시험이다.
  //   코덱 기본값이 'auto' 가 되면서 이 문장은 v2(81B) 로 실리는데, 그 비트 패턴 하나가 폴백 없는
  //   맨몸 locator 에서 M/blur2 의 rect-core-offcenter 를 건드린다(제품 경로 detectAuto 는 4/4 해독,
  //   다른 한글 5건은 v2 로도 통과). 여기선 v1 로 고정해 locator 시험을 유지하고, v2 는 아래
  //   production 왕복(trialV2)으로 따로 본다. 세그먼트 코덱 자체는 test-codec-seg.js(51건).
  const enc = COD.encodeToBits(PAYLOAD, nCells, { ver: 1 });
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

// v2(세그먼트) 프레임을 **제품 경로**(detectAuto: 힌트·conic·sim3·hug 폴백 포함)로 왕복 — P3 커버
function trialV2(grid, label, degFn) {
  const { createCanvas } = require('canvas');
  global.window = global; global.document = { createElement: () => createCanvas(1, 1) };
  require('../examples/wiascan-core.js'); const WS = globalThis.WiaScan;
  const L = GEO.layout(grid, GEO.SPEC); const nCells = GEO.dataCells(L).length;
  const enc = COD.encodeToBits(PAYLOAD, nCells, { ver: 2 });
  const base = GEO.render({ grid, cellPx: CELLPX, quiet: 4, ss: 2, data: true, bits: enc.bits });
  const d = degFn(base.img).img;
  const r = WS.detectAuto({ data: new Uint8ClampedArray(d.data), width: d.width, height: d.height }, { psf: true, deep: true });
  const ok = !!(r && r.decoded && r.text === PAYLOAD); tot++; if (ok) pass++;
  console.log(`  ${ok ? '✅' : '❌'} ${grid} ${label} [v2·detectAuto]: ${ok ? 'decode OK' : '실패'}  [cap ${enc.capBytes}B/${enc.usedBytes}B]`);
}

console.log('오르빗 데이터 왕복검증 — payload:', JSON.stringify(PAYLOAD.slice(0, 30) + '…'));
for (const g of ['S', 'M', 'L']) {
  console.log(` [${g}] 데이터셀 ${GEO.dataCells(GEO.layout(g, GEO.SPEC)).length}개`);
  trial(g, '무열화',      img => DEG.noise(img, 0, 7));
  trial(g, 'yaw40',      img => DEG.tilt(img, 40));
  trial(g, 'noise σ40',  img => DEG.noise(img, 40, 7));
  trial(g, 'rot30',      img => DEG.rotate(img, 30));
  trial(g, 'blur2',      img => DEG.boxBlur(img, 2, 3));
  trialV2(g, 'blur2',    img => DEG.boxBlur(img, 2, 3));
  trialV2(g, 'yaw40',    img => DEG.tilt(img, 40));
}
// ─────────────────────────────────────────────────────────────────────────────
// ★2026-08-27 §7-36 유령 로스터 사건 — 3겹 방어 ③(재발 자동 검출).
//   NON_SQUARE_SHAPES 에 이름만 올라가고 기하 함수가 없는 실루엣이 섞여 있으면
//   디코더가 그걸 시도하다 ReferenceError 로 **해독을 중단**한다("못 읽음"이 아니다).
//   그 5종(pixel-crab·spark·twinkle·knot6·bot)이 실제로 그렇게 들어가 있었고,
//   증상은 엉뚱하게 "fish·piano 가 안 읽힌다"로 나타났다 — 원인과 증상이 멀다.
//   그래서 로스터 전 항목이 실제로 **판정 가능한지**를 여기서 단정한다.
console.log('\n [로스터] NON_SQUARE_SHAPES 자가시험 — 전 항목의 기하가 실존하는가');
{
  const L0 = GEO.layout('M', GEO.SPEC);
  // 로스터 정본은 build-core.js 의 var 선언 하나다(브라우저 전역이라 require 로 못 꺼낸다).
  // 소스에서 직접 읽어야 "누가 이름만 추가했다"를 그대로 잡는다.
  // (kit copy) the roster declaration is inside the bundled engine; the build script is not shipped.
  const _p = require('path').join(__dirname, 'wiascan-core.js');
  const src = require('fs').existsSync(_p) ? require('fs').readFileSync(_p, 'utf8') : '';
  const m = src.match(/var NON_SQUARE_SHAPES = \[([\s\S]*?)\];/);
  const roster = m ? m[1].split('\n').filter(l => !l.trim().startsWith('//'))
                        .join(' ').match(/'([^']+)'/g).map(x => x.replace(/'/g, '')) : [];
  const missing = [];
  for (const sh of (roster || [])) {
    tot++;
    let ok = false, why = '';
    try {
      // ★단정은 "boolean 을 돌려주는가"로는 부족하다 — 처리되지 않은 실루엣은 기본 분기로
      //   떨어져 **항상 true** 를 돌려주고 그대로 통과해 버린다(실측: 가짜 이름 주입 시 ✅).
      //   실루엣의 본질은 "어떤 점은 안에, 어떤 점은 밖에"다. 격자로 훑어 둘 다 있는지 본다.
      //   · 전부 안 → 그 이름은 사실 처리되지 않는다(사각과 구별 불가).
      //   · 전부 밖 → 기하가 깨졌다(방어 ②의 null 폴백에 걸린 경우 포함).
      const L = { ...L0, shape: sh };
      const c = L.coreCenter.mx, R = L.Rdata || (L.N / 2 - 0.8);
      let inN = 0, outN = 0;
      for (let i = -10; i <= 10; i++) {
        for (let j = -10; j <= 10; j++) {
          const r = GEO.insideShape(c + (i / 10) * R * 0.98, c + (j / 10) * R * 0.98, L);
          if (typeof r !== 'boolean') { why = 'boolean 이 아님: ' + typeof r; inN = outN = -1; break; }
          r ? inN++ : outN++;
        }
        if (inN < 0) break;
      }
      if (inN < 0) { /* why 설정됨 */ }
      else if (outN === 0) why = '경계가 없다 — 이 이름은 실제로 처리되지 않는다(사각과 동일)';
      else if (inN === 0) why = '전부 바깥 — 기하가 깨졌거나 함수가 없다';
      else ok = true;
    } catch (e) {
      why = String(e && e.message || e).slice(0, 70);
    }
    if (ok) pass++; else missing.push(sh + ' — ' + why);
    console.log(`  ${ok ? '✅' : '❌'} ${sh}${ok ? '' : ' : ' + why}`);
  }
  if (missing.length) {
    console.log('  ★로스터에 기하 없는 실루엣이 있다 — 등재 조건 = 기하 함수 실존 + 오너 게이트 통과');
  }
}

console.log(`\n합격 ${pass}/${tot}`);
process.exit(pass === tot ? 0 : 1);
