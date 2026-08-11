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
  // basePath: /v2-orbit/web/ 페이지들에선 상대경로("generate.html")로 충분하지만, 다른 경로(루트
  // wiacode.com/ 등)에 이 스크립트를 붙일 땐 목적지가 어긋난다 — 그럴 땐 config로 접두사를 지정.
  var BASE = USER_CFG.basePath || '';

  var NAV = [
    { key: 'generate',     href: BASE + 'generate.html',     label: '생성기',     icon: '⚡' },
    { key: 'generate-lab', href: BASE + 'generate-lab.html', label: '생성기', icon: '🧪', tag: 'LAB' },
    { key: 'scan',         href: BASE + 'scan.html',         label: '스캐너',     icon: '📷' }
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
    + '.wck-footer{background:var(--wck-panel);border-top:1px solid var(--wck-border);'
    +   'padding:22px 16px calc(22px + env(safe-area-inset-bottom));margin-top:32px;'
    +   'display:flex;flex-direction:column;align-items:center;gap:10px;text-align:center;}'
    + '.wck-footer-links{display:flex;flex-wrap:wrap;justify-content:center;gap:4px;}'
    + '.wck-footer-tagline{color:var(--wck-text-dim);font-size:12px;}'
    + '.wck-footer-copy{color:#5c646d;font-size:11px;}';

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
      if (item.key === here) {
        html += '<span class="wck-link wck-here" aria-current="page">' + item.icon + ' ' + item.label + tag + '</span>';
      } else {
        html += '<a class="wck-link" href="' + item.href + '">' + item.icon + ' ' + item.label + tag + '</a>';
      }
    });
    html += '<a class="wck-gh" href="' + GITHUB_URL + '" target="_blank" rel="noopener noreferrer">GitHub ↗</a>';
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

  // ---- footer — appended to body end on bar-mode (content) pages only ----
  function mountFooter() {
    if (document.getElementById('wck-footer')) return;
    var here = currentKey();
    var footer = document.createElement('footer');
    footer.id = 'wck-footer';
    footer.className = 'wck-scope wck-footer';
    footer.innerHTML =
      '<div class="wck-footer-links">' + linksHTML(here) + '</div>' +
      '<div class="wck-footer-tagline">인터넷 없는 곳의 사람을 살리는 코드 &middot; 홍익인간</div>' +
      '<div class="wck-footer-copy">&copy; ' + new Date().getFullYear() + ' WIA Code &middot; MIT License</div>';
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
        '<a class="wck-link" href="' + HOME_URL + '">🏠 WIA Code 홈</a>' +
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
      mountFooter(); // bar 모드(콘텐츠 페이지)에만 — scan.html의 FAB 모드는 앱형 UX라 제외
    } else {
      mountFab();
    }
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
