'use strict';
/*
 * ============================================================================
 *  WIA Code — 스캐너 "다리 QR" 오버레이  (★배포 전용 — GitHub 공개 패키지에 넣지 말 것)
 * ============================================================================
 *  ★★ 제거 조건 (형 확정 2026-08-17): 다리QR은 WIA Code 리더 "1억 다운로드" 도달 +
 *     형 직접 선언 전까지 절대 제거 금지. 어떤 세션도 임의로 끄지 말 것. 1억 = 카메라
 *     제조사·OS가 네이티브 탑재를 검토할 임계 규모 — 그 전까지 다리QR이 유일한 온보딩 경로.
 *  목적: WIA 코드 데이터영역에 작은 표준 QR(고정 URL=scan.html)을 겹쳐,
 *        기본 카메라 사용자가 그 QR로 우리 스캐너에 도달하게 하는 "다리".
 *        WIA 본체는 RS 오류정정이 QR 자리를 흡수 → 우리 스캐너는 여전히 전체 해독.
 *        네이티브 채택 전까지의 온보딩 브리지. (QR 인코더 미포함 — 고정 URL을
 *        1회 인코딩한 매트릭스 상수만 박음. buildQRv3 등 v1 함수 의존 0.)
 *  전제: WIA 격자 L + 2비트(50% ECC) 등 오류정정 여유가 큰 설정에서만 켤 것.
 *  ★2026-08-15: wiago.link(외부 단축링크 서비스) 경유를 없앴다 — 다리 QR이 우리 걸 홍보하면서
 *    정작 우리 도메인이 아닌 곳을 거쳐가는 게 앞뒤가 안 맞는다는 지적(형)으로 wiacode.com 자체
 *    스캐너 URL을 직접 인코딩하도록 교체. 매트릭스는 이제 우리 자체 QR 인코더(wia-soom의
 *    qr-encoder.ts, byte mode EC-M, RS까지 전부 자체구현)로 생성 — 외부 서비스 의존 0.
 *    로고 컷아웃(브랜드 모드)은 짧은 URL+EC-H 조합에만 안전했는데 새 URL이 더 길어 EC-M으로는
 *    안전 마진이 없어 우선 제거 — EC-H 지원을 인코더에 추가하면 복원 가능(별도 작업).
 *  ★2026-08-15(같은 날, 후속): 로고 컷아웃 복원. wia-soom 쪽에서 동일 URL·동일 매트릭스로
 *    jsQR 왕복 검증한 결과, 센터 컷아웃은 폭의 26%까지도 디코딩 성공 — "새 URL이 길어 EC-M
 *    마진이 없다"는 위 판단은 근거가 부족했다(실측 안 하고 추정만 했음). 18%(SAFE_LOGO_FRAC,
 *    아래)로 여유있게 설정하고 다시 켠다. 로고는 이 QR 박스 "안에서" 잘라내는 것뿐이라 박스
 *    바깥 크기(qdim)는 그대로 — 그러니 generate.js가 실측으로 정한 qpx(캔버스의 0.08 —
 *    흑백/bpc1/25%ECC에서 0.15·0.11은 실패, 0.08부터 성공, 2026-08-10 검증) 즉 WIA 코드 자체
 *    RS 예산 문제와 완전히 독립: 그 예산은 "패치가 차지하는 면적"으로 정해지고 로고는 그 안에서만
 *    바뀌니 면적 불변. 회귀 검증: heart/round/square × grid S/M/L × bpc 1(최악값)에서 로고 켠 채로
 *    WiaScan.detectAuto()가 원문을 정확히 복원 — WIA 코드 자체 디코딩에 영향 없음 확인.
 *  ★2026-08-16: 페이로드 URL을 초단축(wiacode.com/s, 21자)으로 바꿔 QR 버전 3(29모듈)→
 *    버전 2(25모듈)로 축소 — 실사용 중 "카메라를 15cm까지 가까이 대야 겨우 읽힘" 리포트가
 *    있었고, 원인이 다리QR이 WIA 코드 자체 RS 오류정정 예산을 38~62%(모양별)까지 먹어치우는
 *    것으로 실측 확인됨(면적=(qdim+2·pad)²이 그대로 orbit 코드 셀 위에 찍히고, 그 아래 깔린
 *    orbit 데이터는 전부 RS로 복원해야 하는 손실로 취급됨 — 다리QR 안에 뭘 그리는지와 무관,
 *    오직 박스 "면적"만이 비용을 결정한다). 모듈 29→25로 줄면 면적이 625/841≈74%로 줄어
 *    orbit RS 소모가 그만큼 감소한다. 형이 제안한 "가운데 WIA 로고 제거"는 시도하지 않음 —
 *    로고는 이미 박스 안에서만 잘라내는 것이라(바로 위 2026-08-15 항목) qdim 자체를 안 줄이므로
 *    RS 예산에 영향이 없다는 게 코드로 이미 실증돼 있었음(로고 있고/없고 면적 불변). 실제로
 *    비용을 줄이는 유일한 손잡이는 "박스 물리 면적" = URL 길이(QR 버전) 뿐이라 이 경로로 감.
 *    /s는 Apache RedirectMatch(001-wiacode-api-ssl.conf)로 scan.html에 301 리다이렉트.
 *  ★2026-08-16(같은 날, 후속 — 진짜 원인): 위 /s(확장자 없음)로 배포하자마자 형이 "스캔은 되는데
 *    엉뚱한 화면에서 안 넘어간다"고 리포트 — 스크린샷을 보니 scan.html이 아니라 wiacode.com
 *    "루트 SPA"(React/Vite, /var/www/wiacode/index.html + sw.js, PWA scope=/)의 캐시된 스플래시가
 *    떴다. 원인: 그 루트 SW의 네비게이션 폴백이 `.html`로 안 끝나는 경로는 전부 자기 앱의 캐시된
 *    index.html로 가로챈다(denylist가 /^\/r(\/|$)/ 등 몇 개 접두어 + /\.html$/ 뿐이라 /s는 안
 *    걸림) — 그 폰에 이 루트 SW가 이미 설치돼 있으면 /s로의 이동이 우리 Apache 리다이렉트까지
 *    가지도 못하고 그 SW가 통째로 가로챔(Puppeteer로 루트 방문→SW active 확인→/s 이동 재현해
 *    직접 확인함, 최종 URL이 /s에 그대로 머물고 루트 SPA 내용이 렌더됨). 고쳐야 할 건 다리QR이
 *    아니라 "이 경로가 그 SW의 denylist를 안전하게 통과하게" 만드는 것 — 그래서 /s → /s.html로
 *    한 글자 늘림(정확히 26자, V2=25모듈 그대로 유지, RS 절감 효과 그대로).
 * ============================================================================
 */
(function (root) {
  // ★2026-08-16(후속): /s.html — 루트 SW denylist(/\.html$/) 통과용, 모듈수는 25로 동일.
  var URL = 'https://wiacode.com/s.html';
  var SIZE = 25;
  // 위 URL을 wia-soom의 자체 QR 인코더(qr-encoder.ts)로 1회 인코딩해 비트팩+base64로 고정.
  var B64 = '/n8/wUXQboBrt0Jl26ty7BM5B/qq/gBNAKoniSA0MHvvBPML8CW+z1hMtyYKOs67/xKqnvwAVMb/gWtwR5G7qc/F0ETy6iwjBLua/uXRgA==';

  var SAFE_LOGO_FRAC = 0.18; // jsQR round-trip tested up to 26% before decode fails; 18% keeps margin.

  function b64bytes(b64) {
    if (typeof Buffer !== 'undefined') return new Uint8Array(Buffer.from(b64, 'base64'));
    var bin = atob(b64), a = new Uint8Array(bin.length);
    for (var i = 0; i < bin.length; i++) a[i] = bin.charCodeAt(i);
    return a;
  }
  // 매트릭스: mod(x,y) → dark(1)/light(0).
  function matrix() {
    var bytes = b64bytes(B64), m = new Uint8Array(SIZE * SIZE);
    for (var i = 0; i < SIZE * SIZE; i++) m[i] = (bytes[i >> 3] >> (7 - (i & 7))) & 1;
    return { m: m, size: SIZE };
  }

  /* WIA 렌더 이미지(정사각, 중앙정렬) 위에 다리 QR을 그린다(픽셀 직접 조작).
   * img: {data:Uint8ClampedArray, width, height}
   * opts: { qpx=QR 모듈당 px(기본 6 — 호출자가 실측해서 넘기는 값을 그대로 신뢰, 이 함수는
   *         스스로 키우지 않음, 소수 허용), place='below-core'|'center'|'br', coreRadiusPx=코어
   *         반경(px, 겹침 회피용), logo=false로 로고 컷아웃 끄기(기본 켜짐),
   *         ★2026-08-16: at={x0,y0}(px) — 있으면 place 계산을 건너뛰고 이 좌표에 바로 스탬프
   *         (다리QR 존 예약 도입 — geometry.js가 정한 자리를 generate.js가 픽셀로 변환해 넘김,
   *         padPx도 함께 오면 그 여백을 씀) }
   * 반환: {ox, oy, qdim, logoCx, logoCy, logoHalf} (그린 위치/크기, 로고 중심/반폭 — logoHalf=0이면 로고 없음) */
  function overlay(img, opts) {
    opts = opts || {};
    var W = img.width, D = img.data;
    /* ★2026-09-01 (C1 direct-link): opts.matrix 로 다른 25x25 매트릭스를 주입할 수 있다.
     *   안 주면 지금까지와 똑같이 고정 URL 매트릭스를 쓴다 — fixed-scanner 출력은
     *   **픽셀 단위로 동일**하다(렌더 지문 2238건으로 확인). 크기가 25 가 아니면 무시하고
     *   고정 매트릭스로 되돌아간다: 예약된 자리와 안 맞는 매트릭스를 그리면 WIA 본체의
     *   RS 예산을 넘겨 코드 전체가 해독 불가가 되기 때문이다. */
    var mm = (opts.matrix && opts.matrix.size === SIZE && opts.matrix.m) ? opts.matrix : matrix();
    var m = mm.m, sz = mm.size;
    var qpx = opts.qpx || 6, qdim = sz * qpx, pad = (opts.padPx != null) ? opts.padPx : qpx * 2; // 흰 여백(quiet zone)
    var ox, oy;
    if (opts.at) {
      ox = Math.round(opts.at.x0); oy = Math.round(opts.at.y0);
    } else {
      var cx = W / 2, cy = W / 2;
      var core = opts.coreRadiusPx || Math.round(W * 0.09); // 코어(불스아이) 대략 반경
      var place = opts.place || 'below-core';
      var mgn = Math.round(W * 0.03); // 코너 여백
      if (place === 'center') { ox = Math.round(cx - qdim / 2); oy = Math.round(cy - qdim / 2); }
      else if (place === 'br') { ox = W - qdim - mgn; oy = W - qdim - mgn; }
      else if (place === 'bl') { ox = mgn; oy = W - qdim - mgn; }
      else if (place === 'tr') { ox = W - qdim - mgn; oy = mgn; }
      else if (place === 'tl') { ox = mgn; oy = mgn; }
      else if (place === 'above-core') { // 가로 중앙, 코어 바로 위
        ox = Math.round(cx - qdim / 2);
        oy = Math.round(cy - core - pad - qdim);
        if (oy < pad) oy = pad;
      }
      else { // below-core: 가로 중앙, 세로는 코어 바로 아래
        ox = Math.round(cx - qdim / 2);
        oy = Math.round(cy + core + pad);
        if (oy + qdim + pad > W) oy = Math.round(cy - qdim / 2);
      }
    }
    function set(px, py, v) {
      if (px < 0 || py < 0 || px >= W || py >= img.height) return;
      var o = (py * W + px) * 4; D[o] = D[o + 1] = D[o + 2] = v; D[o + 3] = 255;
    }
    var padR = Math.round(pad);
    for (var y = -padR; y < qdim + padR; y++) for (var x = -padR; x < qdim + padR; x++) set(ox + x, oy + y, 255);
    // ★2026-08-16: qpx가 소수여도 모듈 경계가 딱 맞물리게, 매 모듈을 "누적거리 반올림" 구간으로 그린다
    //   (고정폭 xx<qpx 루프는 소수 qpx에서 모듈 사이 틈/겹침이 생김 — cellPx=10일 때만 우연히 qpx가
    //   정수(7)라 안 드러났을 뿐, 다른 cellPx에서 재현되는 잠재버그라 미리 막음).
    for (var my = 0; my < sz; my++) for (var mx = 0; mx < sz; mx++) {
      if (!m[my * sz + mx]) continue;
      var x0 = ox + Math.round(mx * qpx), x1 = ox + Math.round((mx + 1) * qpx);
      var y0 = oy + Math.round(my * qpx), y1 = oy + Math.round((my + 1) * qpx);
      for (var yy = y0; yy < y1; yy++) for (var xx = x0; xx < x1; xx++) set(xx, yy, 0);
    }
    // Logo cutout — carved out of the box's own already-allocated footprint, so it changes
    // nothing about how much of the WIA code's cells the patch covers (that's fixed by qdim,
    // which this function never enlarges on its own — see header note).
    var logoHalf = 0, logoCx = ox + qdim / 2, logoCy = oy + qdim / 2;
    if (opts.logo !== false) {
      logoHalf = Math.round(qdim * SAFE_LOGO_FRAC / 2);
      var lx0 = Math.round(logoCx - logoHalf), ly0 = Math.round(logoCy - logoHalf);
      for (var ly = 0; ly < logoHalf * 2; ly++) for (var lx = 0; lx < logoHalf * 2; lx++) set(lx0 + lx, ly0 + ly, 255);
    }
    /* matrix 를 함께 돌려준다 — svg-vector.js 가 PNG 와 **같은** 매트릭스를 그려야 한다.
     * (예전엔 거기서 WiaBridge.matrix() 를 다시 불러 고정 매트릭스를 그렸다. 동적 매트릭스가
     *  들어오면 PNG 와 SVG 가 서로 다른 QR 을 담게 되는 자리였다.) */
    return { ox: ox, oy: oy, qdim: qdim, url: (opts.url || URL), matrix: mm,
             logoCx: logoCx, logoCy: logoCy, logoHalf: logoHalf };
  }

  var API = { overlay: overlay, matrix: matrix, SIZE: SIZE, URL: URL };
  if (typeof module !== 'undefined' && module.exports) module.exports = API;
  root.WiaBridge = API;
})(typeof globalThis !== 'undefined' ? globalThis : this);
