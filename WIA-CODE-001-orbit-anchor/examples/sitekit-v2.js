/*!
 * WIA Code 공유 헤더/네비 — v2 DRAFT (리뷰용, 아직 어떤 라이브 페이지에도 연결 안 됨)
 * ---------------------------------------------------------------------------
 * 대상 3페이지: scan.html · generate.html · generate-lab.html
 *
 * 기존 sitekit.js(19줄) 대비 개선점
 *   1) generate-lab.html을 네비에 추가 (고아 페이지 문제 해결) + generate.html과
 *      "정확히" 구분되는 활성탭 표시 (기존 indexOf 부분일치 버그로 두 페이지가
 *      동시에 "here"로 표시되던 것을 exact-match로 수정).
 *   2) scan.html처럼 마운트 지점(#kit)이 아예 없는 페이지에서도 동작 —
 *      #kit/#wia-header/[data-wck-mount]가 있으면 그 안에 인라인 바(막대형 네비)를
 *      렌더링하고, 없으면 화면 구석에 떠 있는 작은 FAB(원형 버튼)+펼침 패널로
 *      자동 전환된다. scan.html의 HTML은 건드리지 않고도 붙일 수 있게 하기 위함.
 *   3) 모바일 반응형: 인라인 바는 640px 이하에서 햄버거로 접힘, FAB는 항상 접힘형.
 *   4) 완전히 자체 스코프 CSS(.wck- 접두사 전용 클래스만 사용, 태그 셀렉터 없음) —
 *      각 페이지 <style> 블록의 .kit/.kit a 등 기존 규칙과 충돌하지 않는다.
 *      유일하게 손대는 기존 DOM은 마운트 컨테이너 자신(#kit 등)의 inline style —
 *      그 요소의 레거시 padding/overflow-x/white-space만 리셋해서 새 바가 정상
 *      레이아웃되게 한다 (페이지 <style> 블록 자체는 절대 수정하지 않음).
 *   5) 멱등 마운트 가드 — 같은 페이지에 스크립트가 두 번 실려도 중복 삽입 안 됨.
 *   6) 다크테마 통일: #0b0d10 배경 · #00e6a0 accent (scan.html 쪽 값으로 통일).
 *      generate.html/generate-lab.html 자체 버튼(#10b981) 등 다른 요소 색은
 *      건드리지 않음 — 이 컴포넌트 자신의 색만 통일.
 *
 * 통합 방법(참고용 — 이 커밋에서는 실행하지 않음)
 *   - generate.html / generate-lab.html: <script src="sitekit.js">를
 *     sitekit-v2-draft.js로 교체만 하면 끝 (#kit가 이미 있어 그대로 인라인 바로 마운트).
 *   - scan.html: <script src="scan.js"> 다음 줄에 <script src="sitekit-v2-draft.js">
 *     한 줄만 추가하면 끝 (#kit가 없으므로 자동으로 FAB 모드).
 *
 * 알려진 미해결 사항(리뷰 대상)
 *   - GITHUB_URL 값은 실제 공개 저장소 URL을 확정 후 교체 필요(현재 placeholder).
 *   - scan.html에 FAB를 붙일 때 기본 코너(bottom-left)가 카메라 UI(.controls,
 *     #openUrlBtn, engineBadge)와 실제 겹치는지는 실기기 스크린샷으로 확인 안 됨 —
 *     겹치면 `window.WIA_SITEKIT_CONFIG = { corner: 'tr' | 'tl' | 'br' | 'bl' }`를
 *     스크립트 로드 "전"에 지정해 조정 가능하게만 해뒀음.
 *   - generate.js/generate-lab.js의 grid override 표시 안 됨 / bpc 라벨 불일치
 *     버그는 이 파일의 책임 범위 밖(해당 스크립트 쪽 수정 필요).
 */
(function () {
  'use strict';

  // ---- 멱등 가드 -----------------------------------------------------------
  if (window.__WCK_V2_MOUNTED__) return;
  if (document.getElementById('wck-style') || document.getElementById('wck-fab-wrap')) return;
  window.__WCK_V2_MOUNTED__ = true;

  // ---- 설정 ------------------------------------------------------------
  var HOME_URL = 'https://wiacode.com';
  var GITHUB_URL = 'https://github.com/WIA-Official/wia-standards-public/tree/main/WIA-CODE-001-orbit-anchor';

  var USER_CFG = window.WIA_SITEKIT_CONFIG || {};
  // basePath: /orbit/web/ 페이지들에선 상대경로("generate.html")로 충분하지만, 다른 경로(루트
  // wiacode.com/ 등)에 이 스크립트를 붙일 땐 목적지가 어긋난다 — 그럴 땐 config로 접두사를 지정.
  var BASE = USER_CFG.basePath || '';

  // generate-lab.html(다리 QR 실험용 내부 개발 도구, 라이브 기능 아님)은 의도적으로 여기서 뺐다 —
  // 방문자용 공개 네비게이션에 실험실 페이지가 노출될 이유가 없다. 필요하면 URL로 직접 접근.
  var NAV = [
    { key: 'generate',     href: BASE + 'generate.html',     label: '생성기',     icon: '⚡', i18n: 'nav.menu' },
    { key: 'scan',         href: BASE + 'scan.html',         label: '스캐너',     icon: '📷', i18n: 'nav.scan' }
  ];

  // ---- CSS (전부 .wck- 접두사 — 페이지 자체 스타일과 충돌 없음) -------------
  var CSS = ''
    + '.wck-scope,.wck-scope *{box-sizing:border-box;}'
    + '.wck-scope{'
    +   '--wck-bg:#0b0d10;--wck-panel:#12151a;--wck-panel-2:#181c22;--wck-border:#242a31;'
    +   '--wck-accent:#00e6a0;--wck-accent-ink:#06261c;--wck-text:#eef2f5;--wck-text-dim:#9aa4ad;'
    +   'font-family:-apple-system,BlinkMacSystemFont,"Segoe UI","Apple SD Gothic Neo","Malgun Gothic",system-ui,sans-serif;'
    +   'font-size:13px;color:var(--wck-text);line-height:1.3;'
    + '}'
    // brand / logo mark (no external image dependency)
    + '.wck-brand{display:flex;align-items:center;gap:8px;text-decoration:none;color:var(--wck-text);flex:0 0 auto;}'
    + '.wck-mark{width:22px;height:22px;border-radius:7px;background:var(--wck-accent);color:var(--wck-accent-ink);'
    +   'font-weight:800;font-size:10.5px;display:flex;align-items:center;justify-content:center;letter-spacing:-0.5px;flex:0 0 auto;}'
    + '.wck-brand-text{font-weight:800;font-size:13px;white-space:nowrap;}'
    // link chips (shared by both bar mode and panel mode)
    + '.wck-link{display:inline-flex;align-items:center;gap:5px;color:var(--wck-text-dim);text-decoration:none;'
    +   'font-weight:600;font-size:12.5px;padding:7px 9px;border-radius:8px;white-space:nowrap;}'
    + 'a.wck-link:hover{color:var(--wck-text);background:var(--wck-panel-2);}'
    + '.wck-link.wck-here{color:var(--wck-accent);background:rgba(0,230,160,0.12);}'
    + '.wck-tag{font-size:9px;font-weight:800;background:var(--wck-accent);color:var(--wck-accent-ink);'
    +   'border-radius:999px;padding:1px 5px;letter-spacing:0.3px;margin-left:2px;}'
    + '.wck-gh{color:var(--wck-text-dim);text-decoration:none;font-weight:600;font-size:12.5px;'
    +   'padding:7px 9px;border:1px solid var(--wck-border);border-radius:8px;white-space:nowrap;display:inline-flex;align-items:center;gap:5px;}'
    + '.wck-gh:hover{color:var(--wck-text);border-color:#3a4451;}'
    + '.wck-lang{background:none;color:var(--wck-text-dim);border:1px solid var(--wck-border);border-radius:8px;'
    +   'width:34px;height:34px;font-size:15px;cursor:pointer;flex:0 0 auto;display:inline-flex;align-items:center;justify-content:center;}'
    + '.wck-lang:hover{color:var(--wck-text);border-color:#3a4451;}'
    // ---- mode A: in-flow bar (mounted inside #kit etc.) ----
    + '.wck-bar{display:flex;align-items:center;gap:8px;background:var(--wck-panel);'
    +   'border-bottom:1px solid var(--wck-border);padding:8px 12px;position:relative;min-height:40px;}'
    + '.wck-links{display:flex;align-items:center;gap:4px;margin-left:auto;flex-wrap:wrap;}'
    + '.wck-burger{display:none;background:none;border:1px solid var(--wck-border);color:var(--wck-text);'
    +   'border-radius:8px;width:34px;height:34px;font-size:15px;margin-left:auto;cursor:pointer;flex:0 0 auto;}'
    + '@media (max-width:640px){'
    +   '.wck-links{display:none;position:absolute;top:100%;left:0;right:0;flex-direction:column;align-items:stretch;'
    +     'gap:2px;background:var(--wck-panel);border-bottom:1px solid var(--wck-border);padding:8px;z-index:50;'
    +     'box-shadow:0 8px 20px rgba(0,0,0,.35);}'
    +   '.wck-links.wck-open{display:flex;}'
    +   '.wck-links .wck-link,.wck-links .wck-gh{width:100%;}'
    +   '.wck-burger{display:flex;align-items:center;justify-content:center;}'
    + '}'
    // ---- mode B: floating corner FAB + panel (no mount point found) ----
    // 2026-08-10 실기기 스크린샷 확인: 12px 기본 오프셋은 scan.html의 상단바("ENGINE" 배지)·
    // 하단 컨트롤바와 겹침(둘 다 일반 문서흐름 요소라 position:fixed FAB가 그 위에 얹힘).
    // 대부분의 페이지 상단바(~48px)·하단 컨트롤바(~76px)를 넉넉히 피하도록 기본값을 올림 —
    // 페이지별로 더 정확한 값이 필요하면 WIA_SITEKIT_CONFIG.offset으로 덮어쓸 것.
    + '.wck-fab-wrap{position:fixed;z-index:99999;}'
    + '.wck-fab-wrap.wck-bl{left:12px;bottom:calc(84px + env(safe-area-inset-bottom));}'
    + '.wck-fab-wrap.wck-br{right:12px;bottom:calc(84px + env(safe-area-inset-bottom));}'
    + '.wck-fab-wrap.wck-tl{left:12px;top:calc(58px + env(safe-area-inset-top));}'
    + '.wck-fab-wrap.wck-tr{right:12px;top:calc(58px + env(safe-area-inset-top));}'
    + '.wck-fab{width:38px;height:38px;border-radius:12px;background:var(--wck-panel);border:1px solid var(--wck-border);'
    +   'color:var(--wck-accent);display:flex;align-items:center;justify-content:center;font-weight:800;'
    +   'font-size:11px;cursor:pointer;box-shadow:0 4px 14px rgba(0,0,0,.4);}'
    + '.wck-fab:active{transform:scale(.95);}'
    + '.wck-panel{position:absolute;min-width:190px;background:var(--wck-panel);border:1px solid var(--wck-border);'
    +   'border-radius:12px;padding:6px;box-shadow:0 10px 30px rgba(0,0,0,.5);display:none;flex-direction:column;gap:2px;}'
    + '.wck-panel.wck-open{display:flex;}'
    + '.wck-bl .wck-panel,.wck-tl .wck-panel{left:0;}'
    + '.wck-br .wck-panel,.wck-tr .wck-panel{right:0;}'
    + '.wck-bl .wck-panel,.wck-br .wck-panel{bottom:calc(100% + 8px);}'
    + '.wck-tl .wck-panel,.wck-tr .wck-panel{top:calc(100% + 8px);}'
    + '.wck-panel .wck-link,.wck-panel .wck-gh{width:100%;}'
    // ---- footer (bar 모드 페이지에만 — scan.html처럼 앱형 단일화면 UX엔 안 어울려서 제외) ----
    // wiamotion site-kit(.fv2/.foot-top/.foot-cols)의 정보구조(브랜드+컬럼형 링크+하단바)를
    // wck 자체 색 토큰으로 재현 — 팔레트는 안 베끼고 레이아웃만 이식. 2026-08-14.
    // ★2026-08-14 폭 버그 수정: 바탕색이 있는 .wck-footer 자체를 max-width로 좁히면 넓은 화면에서
    // 배경이 가운데 좁은 띠로 잘려 "만들다 만 것"처럼 보인다(형 실측 지적). .wck-footer는 항상
    // 뷰포트 풀블리드로 두고, 안쪽 콘텐츠(.wck-foot-inner)만 max-width로 가운데 정렬한다.
    + '.wck-footer{background:var(--wck-panel);border-top:1px solid var(--wck-border);'
    +   'padding:32px 16px calc(22px + env(safe-area-inset-bottom));margin-top:32px;text-align:left;}'
    + '.wck-foot-inner{max-width:1400px;margin:0 auto;}'
    + '.wck-foot-top{display:flex;flex-wrap:wrap;gap:32px;justify-content:space-between;padding-bottom:24px;}'
    + '.wck-foot-brand{max-width:280px;}'
    + '.wck-foot-brand .wck-brand{margin-bottom:8px;}'
    + '.wck-foot-brand p{margin:0;color:var(--wck-text-dim);font-size:12.5px;line-height:1.6;}'
    + '.wck-foot-cols{display:flex;flex-wrap:wrap;gap:28px;flex:1;justify-content:flex-end;}'
    + '.wck-foot-col{min-width:140px;}'
    + '.wck-foot-col h5{margin:0 0 10px;font-size:10.5px;font-weight:800;letter-spacing:.05em;'
    +   'text-transform:uppercase;color:var(--wck-text-dim);}'
    + '.wck-foot-col a{display:block;color:var(--wck-text-dim);font-size:12.5px;text-decoration:none;'
    +   'margin-bottom:9px;transition:.15s;}'
    + '.wck-foot-col a:hover{color:var(--wck-text);}'
    + '.wck-foot-bottom{display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;'
    +   'gap:10px;padding-top:20px;border-top:1px solid var(--wck-border);}'
    + '.wck-footer-copy{color:#5c646d;font-size:11px;}'
    + '.wck-footer-badge img{display:block;}'
    + '@media(max-width:640px){.wck-foot-top{flex-direction:column;gap:22px;}'
    +   '.wck-foot-cols{justify-content:flex-start;gap:20px;}'
    +   '.wck-foot-bottom{flex-direction:column;align-items:flex-start;}}';

  function injectStyle() {
    var s = document.createElement('style');
    s.id = 'wck-style';
    s.textContent = CSS;
    document.head.appendChild(s);
  }

  // ---- helpers -----------------------------------------------------------
  function currentKey() {
    var seg = (location.pathname.split('/').pop() || '').toLowerCase();
    seg = seg.replace(/\.html?$/, '');
    return seg || 'home';
  }

  function linksHTML(here) {
    var html = '';
    NAV.forEach(function (item) {
      var tag = item.tag ? ' <span class="wck-tag">' + item.tag + '</span>' : '';
      var label = '<span data-i18n="' + item.i18n + '">' + item.label + '</span>';
      if (item.key === here) {
        html += '<span class="wck-link wck-here" aria-current="page">' + item.icon + ' ' + label + tag + '</span>';
      } else {
        html += '<a class="wck-link" href="' + item.href + '">' + item.icon + ' ' + label + tag + '</a>';
      }
    });
    html += '<a class="wck-gh" href="' + GITHUB_URL + '" target="_blank" rel="noopener noreferrer">GitHub ↗</a>';
    html += '<button type="button" class="wck-lang" onclick="if(window.openLanguageModal)openLanguageModal()" aria-label="Language / 언어">🌐</button>';
    return html;
  }

  function closeOnOutsideClickAndEsc(root, toggle, closeFn) {
    document.addEventListener('click', function (e) {
      if (!root.contains(e.target)) closeFn();
    });
    document.addEventListener('keydown', function (e) {
      if (e.key === 'Escape') { closeFn(); toggle.focus(); }
    });
  }

  // ---- mode A: mount inline bar into an existing container (#kit 등) -----
  function mountBar(container) {
    // 레거시 .kit{padding/overflow-x/white-space...} 규칙이 새 바 레이아웃을
    // 깨지 않도록, 마운트 컨테이너 자신의 inline style만 리셋한다.
    // (페이지 <style> 블록은 전혀 건드리지 않음 — 그 규칙은 그대로 남되 여기서 override)
    container.style.cssText = 'display:block;padding:0;margin:0;overflow:visible;white-space:normal;background:transparent;border:0;';
    container.classList.add('wck-scope');

    var here = currentKey();
    container.innerHTML =
      '<div class="wck-bar">' +
        '<a class="wck-brand" href="' + HOME_URL + '"><span class="wck-mark">WC</span><span class="wck-brand-text">WIA Code</span></a>' +
        '<button type="button" class="wck-burger" id="wck-burger" aria-expanded="false" aria-label="메뉴 열기">☰</button>' +
        '<nav class="wck-links" id="wck-links" aria-label="WIA Code 페이지 이동">' + linksHTML(here) + '</nav>' +
      '</div>';

    var burger = container.querySelector('#wck-burger');
    var links = container.querySelector('#wck-links');
    function close() { links.classList.remove('wck-open'); burger.setAttribute('aria-expanded', 'false'); }
    burger.addEventListener('click', function (e) {
      e.stopPropagation();
      var open = links.classList.toggle('wck-open');
      burger.setAttribute('aria-expanded', open ? 'true' : 'false');
    });
    closeOnOutsideClickAndEsc(container, burger, close);
    window.addEventListener('resize', function () {
      if (window.innerWidth > 640) close();
    });
  }

  // ---- footer column helper (제품/리소스/법적고지 공용) ----
  function footerColumnHTML(title, items) {
    var links = items.map(function (it) {
      var ext = it.ext ? ' target="_blank" rel="noopener"' : '';
      return '<a href="' + it.href + '"' + ext + '>' + it.label + '</a>';
    }).join('');
    return '<div class="wck-foot-col"><h5>' + title + '</h5>' + links + '</div>';
  }

  // ---- footer — appended to body end on bar-mode (content) pages only ----
  function mountFooter() {
    if (document.getElementById('wck-footer')) return;
    var footer = document.createElement('footer');
    footer.id = 'wck-footer';
    footer.className = 'wck-scope wck-footer';

    var productLinks = NAV.map(function (n) {
      return { href: n.href, label: n.icon + ' ' + n.label + (n.tag ? ' ' + n.tag : '') };
    });
    var resourceLinks = [
      { href: GITHUB_URL, label: 'GitHub ↗', ext: true },
      { href: '/orbit/BENCHMARK.md', label: '벤치마크', ext: true }
    ];
    var legalLinks = [
      { href: '/privacy.html', label: '개인정보처리방침' },
      { href: '/terms.html', label: '이용약관' }
    ];
    // 2026-08-14: "WIA 패밀리"(퀀텀 DNA 코드·wia.tools QR 생성기·WIA Go) 컬럼 완전 삭제 — 형이
    // 실사이트를 직접 보고 "이건 전혀 우리 WIA Code에 도움이 안 되는거잖아" 지적. 특히 wia.tools의
    // "일반 QR 생성기"는 이 사이트의 핵심 메시지("QR 너머, 우리들의 코드")와 정면으로 충돌해서
    // 푸터 격하로는 부족했음 — 링크 자체를 없앤다(홈 본문에서도 이미 제거된 상태, 어디에도 없음).

    footer.innerHTML =
      '<div class="wck-foot-inner">' +
        '<div class="wck-foot-top">' +
          '<div class="wck-foot-brand">' +
            '<a class="wck-brand" href="' + HOME_URL + '"><span class="wck-mark">WC</span><span class="wck-brand-text">WIA Code</span></a>' +
            '<p>인터넷 없는 곳의 사람을 살리는 코드 &middot; 홍익인간</p>' +
          '</div>' +
          '<div class="wck-foot-cols">' +
            footerColumnHTML('제품', productLinks) +
            footerColumnHTML('리소스', resourceLinks) +
            footerColumnHTML('법적고지', legalLinks) +
          '</div>' +
        '</div>' +
        '<div class="wck-foot-bottom">' +
          '<div class="wck-footer-badge"><a href="https://a11y.wiabook.com/verify/?id=result-1786682008221-5bce4a" target="_blank" rel="noopener noreferrer" title="WIA A11Y accessibility self-audit result — click to verify"><img src="https://a11y.wiabook.com/badges/wia-premium.png" alt="WIA A11Y self-audit — A grade (verify)" width="56" height="56" loading="lazy"></a></div>' +
          '<div class="wck-footer-copy">&copy; ' + new Date().getFullYear() + ' WIA Code &middot; MIT License</div>' +
        '</div>' +
      '</div>';
    // ★body에 직접 붙인다 — #root 안에 넣으면 안 됨(2026-08-10 발견): React 같은 SPA는 #root를
    // 통째로 자기가 소유한다고 가정해서, 다음 렌더(예: hashchange) 때 자기가 안 만든 자식(이 푸터)을
    // 지워버림. body 레벨 배치가 밀리는 원인(#root height:100% 고정이라 실제 콘텐츠가 넘쳐도 박스가
    // 뷰포트 높이에서 끝남)은 앱 쪽 CSS를 min-height로 바꿔 근본 수정할 것 — 여기서 우회하지 않는다.
    document.body.appendChild(footer);
  }

  // ---- mode B: no mount point found → floating corner FAB + panel --------
  function mountFab() {
    var corner = USER_CFG.corner || 'tr'; // 'tl' | 'tr' | 'bl' | 'br' — scan.html 스크린샷으로 확인됨(2026-08-10)
    var here = currentKey();
    var wrap = document.createElement('div');
    wrap.id = 'wck-fab-wrap';
    wrap.className = 'wck-scope wck-fab-wrap wck-' + corner;
    wrap.innerHTML =
      '<div class="wck-panel" id="wck-panel" role="menu">' +
        '<a class="wck-link" href="' + HOME_URL + '">🏠 WIA Code <span data-i18n="nav.home">홈</span></a>' +
        linksHTML(here) +
      '</div>' +
      '<button type="button" class="wck-fab" id="wck-fab" aria-expanded="false" aria-label="WIA Code 메뉴 열기">WC</button>';
    document.body.appendChild(wrap);

    var fab = wrap.querySelector('#wck-fab');
    var panel = wrap.querySelector('#wck-panel');
    function close() { panel.classList.remove('wck-open'); fab.setAttribute('aria-expanded', 'false'); }
    fab.addEventListener('click', function (e) {
      e.stopPropagation();
      var open = panel.classList.toggle('wck-open');
      fab.setAttribute('aria-expanded', open ? 'true' : 'false');
    });
    closeOnOutsideClickAndEsc(wrap, fab, close);
  }

  // ---- language modal — 127개 언어, 네비 라벨(생성기/생성기 LAB/스캐너/홈)만 번역 -----
  // wia-lang-bridge.js가 이미 감시하는 storageKey('wia-nav-lang')를 그대로 써서, 다른 WIA
  // 패밀리 사이트에서 고른 언어가 여기서도 그대로 적용되고 그 반대도 성립한다(교차 동기화 무료).
  function mountLanguageModal() {
    if (window.openLanguageModal) return; // 이미 로드됨(중복 삽입 방지)
    window.WIA_MODAL_CONFIG = window.WIA_MODAL_CONFIG ||
      { i18nBasePath: '/i18n/', languageCount: 127, defaultLang: 'ko', storageKey: 'wia-nav-lang' };
    var s = document.createElement('script');
    s.src = BASE + 'language-modal.js';
    document.body.appendChild(s);
  }

  // ---- init ----------------------------------------------------------------
  function init() {
    injectStyle();
    // footerOnly: 페이지가 이미 자체 헤더/네비를 갖고 있을 때(예: wiacode.com 루트의 React 앱) —
    // 중복 헤더 없이 하단 푸터(공통 브랜드·링크)만 붙인다.
    if (USER_CFG.footerOnly) { mountFooter(); return; }
    var mountSel = USER_CFG.mount || '#kit, #wia-header, [data-wck-mount]';
    var container = document.querySelector(mountSel);
    if (container) {
      mountBar(container);
      // noFooter: 페이지가 이미 자체 <footer>(contentinfo 랜드마크)를 갖고 있을 때(예: wiacode.com
      // 루트의 React 앱 — App.tsx의 <footer className="app-footer">)만 지정 — 안 지정하면 기존과
      // 동일하게 bar 모드(콘텐츠 페이지)에 공통 푸터를 붙인다. scan.html의 FAB 모드는 앱형 UX라 제외.
      // (a11y: 2026-08-14 axe landmark-no-duplicate-contentinfo/landmark-unique 수정 — 루트 페이지가
      // 이 공통 푸터 + 자체 app-footer 두 개의 contentinfo 랜드마크를 동시에 갖고 있었음)
      if (!USER_CFG.noFooter) mountFooter();
    } else {
      mountFab();
    }
    mountLanguageModal();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
