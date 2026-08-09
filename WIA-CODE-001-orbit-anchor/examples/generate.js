(function () {
  'use strict';
  var $ = function (id) { return document.getElementById(id); };
  var state = { ctype: 'text', shape: 'square', grid: 'M', bpc: 2, mcpT: 'stdio' };

  function seg(id, key, cast, after) {
    var wrap = $(id); if (!wrap) return;
    wrap.addEventListener('click', function (e) {
      var b = e.target.closest('button'); if (!b) return;
      Array.prototype.forEach.call(wrap.children, function (c) { c.classList.toggle('on', c === b); });
      state[key] = cast ? cast(b.getAttribute('data-v')) : b.getAttribute('data-v');
      if (after) after();
    });
  }
  seg('ctype', 'ctype', null, function () {
    Array.prototype.forEach.call(document.querySelectorAll('.fields'), function (f) {
      f.classList.toggle('on', f.getAttribute('data-t') === state.ctype);
    });
  });
  seg('shape', 'shape'); seg('grid', 'grid');
  seg('bpc', 'bpc', function (v) { return parseInt(v, 10); });
  seg('mcp_transport', 'mcpT');
  $('colorOn').addEventListener('change', function () { $('hueWrap').classList.toggle('on', this.checked); });

  var val = function (id) { var e = $(id); return e ? (e.value || '').trim() : ''; };

  // 콘텐츠 타입 → payload 문자열 (구조적 타입은 컴팩트 JSON = "설정을 코드로").
  function buildPayload() {
    switch (state.ctype) {
      case 'url': return val('f_url');
      case 'ssh': {
        var o = { t: 'ssh', host: val('ssh_host'), port: val('ssh_port') || '22', user: val('ssh_user') };
        var k = val('ssh_key'); if (k) o.key = k;
        return JSON.stringify(o);
      }
      case 'api': {
        var o = { t: 'api', name: val('api_name'), url: val('api_url'), key: val('api_key') };
        var h = val('api_headers'); if (h) { try { o.headers = JSON.parse(h); } catch (e) { o.headers = h; } }
        return JSON.stringify(o);
      }
      case 'mcp': {
        var o = { t: 'mcp', name: val('mcp_name'), transport: state.mcpT, endpoint: val('mcp_ep') };
        var tk = val('mcp_token'); if (tk) o.token = tk;
        var hs = val('mcp_hash'); if (hs) o.sha256 = hs;
        return JSON.stringify(o);
      }
      default: return val('f_text');
    }
  }

  function ready(cb) {
    if (window.WiaScan) { $('badge').textContent = 'ENGINE: real'; cb(); }
    else { $('badge').textContent = 'ENGINE 로딩…'; setTimeout(function () { ready(cb); }, 150); }
  }

  function run() {
    var out = $('out');
    var text = buildPayload();
    if (!text) { out.innerHTML = '<div class="err">내용을 입력하세요.</div>'; return; }
    var opts = { text: text, grid: state.grid, bpc: state.bpc, shape: state.shape, cellPx: 10 };
    if ($('colorOn').checked) opts.hueText = val('hueText') || ' ';
    var r;
    try { r = window.WiaScan.generate(opts); } catch (e) { r = { error: String(e) }; }
    if (!r || r.error) { out.innerHTML = '<div class="err">생성 실패: ' + (r && r.error ? r.error : '?') + '</div>'; return; }
    if (r.bytes > r.capBytes) { out.innerHTML = '<div class="err">내용이 너무 깁니다 (' + r.bytes + '자 > 용량 ' + r.capBytes + 'B). 격자를 L로 키우거나 색을 켜세요.</div>'; return; }
    var name = 'wia-' + state.ctype + '-' + r.grid + (r.shape !== 'square' ? '-' + r.shape : '') + (r.color ? '-color' : '') + '.png';
    var kind = { text: '텍스트', url: 'URL', ssh: 'SSH 접속설정', api: 'API 설정', mcp: 'MCP 부트스트랩' }[state.ctype];
    out.innerHTML =
      '<img alt="WIA Code" src="' + r.dataURL + '">' +
      '<div class="meta"><b>' + kind + '</b> · ' + r.width + '×' + r.height + 'px · ' + r.grid + '/' + (r.shape) + ' · ' + (r.color ? '컬러' : state.bpc + '단계') +
      '<br>' + r.bytes + '자 담김 / 용량 약 ' + r.capBytes + '바이트</div>' +
      '<a class="dl" download="' + name + '" href="' + r.dataURL + '">⬇ PNG 다운로드</a>' +
      '<div class="meta" style="margin-top:12px">화면/인쇄 후 <a class="scan" href="scan.html">스캐너</a>로 읽으면 이 설정이 그대로 나옵니다 (오프라인).</div>';
  }

  ready(function () { $('gen').addEventListener('click', run); run(); });
})();
