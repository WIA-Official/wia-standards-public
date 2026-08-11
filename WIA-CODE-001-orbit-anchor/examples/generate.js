(function () {
  'use strict';
  var $ = function (id) { return document.getElementById(id); };
  var state = { ctype: 'text', shape: 'square', grid: 'M', bpc: 2, mcpT: 'stdio', wifiSec: 'WPA' };

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
    run();
  });
  // 옵션 바꾸면 즉시 재생성(라이브 미리보기). run 은 아래에서 함수선언(호이스팅)됨.
  var regen = function () { run(); };
  seg('shape', 'shape', null, regen); seg('grid', 'grid', null, regen);
  seg('bpc', 'bpc', function (v) { return parseInt(v, 10); }, regen);
  seg('mcp_transport', 'mcpT', null, regen);
  seg('wifi_sec', 'wifiSec', null, regen);
  $('colorOn').addEventListener('change', function () { $('hueWrap').classList.toggle('on', this.checked); regen(); });
  $('wifi_hidden').addEventListener('change', regen);
  $('printLabelOn').addEventListener('change', regen);
  // 텍스트 입력은 디바운스 재생성
  var dbt; var deb = function () { clearTimeout(dbt); dbt = setTimeout(regen, 350); };
  Array.prototype.forEach.call(document.querySelectorAll('#ctype ~ * input[type="text"], textarea'), function (el) { el.addEventListener('input', deb); });

  var val = function (id) { var e = $(id); return e ? (e.value || '').trim() : ''; };

  // "YYYY-MM-DD HH:MM" → iCalendar DTSTART/DTEND용 "YYYYMMDDTHHMMSS"(로컬시간, 지정 안 되면 빈 문자열).
  function icsDate(s) {
    var m = /^(\d{4})-(\d{2})-(\d{2})[ T](\d{2}):(\d{2})/.exec(s || '');
    if (!m) return '';
    return m[1] + m[2] + m[3] + 'T' + m[4] + m[5] + '00';
  }
  // vCard/iCalendar는 필드 안의 개행·쉼표·세미콜론을 이스케이프해야 표준 파서가 안 깨짐(RFC 6350/5545).
  function icsEsc(s) { return (s || '').replace(/\\/g, '\\\\').replace(/\n/g, '\\n').replace(/,/g, '\\,').replace(/;/g, '\\;'); }

  // 콘텐츠 타입 → payload 문자열 (구조적 타입은 표준 URI/포맷 — 스캔한 폰이 그대로 처리 가능).
  function buildPayload() {
    switch (state.ctype) {
      case 'url': return val('f_url');
      case 'wifi': {
        var sec = state.wifiSec, ssid = val('wifi_ssid'), pw = val('wifi_pw');
        var s = 'WIFI:T:' + (sec === 'nopass' ? 'nopass' : sec) + ';S:' + icsEsc(ssid) + ';';
        if (sec !== 'nopass') s += 'P:' + icsEsc(pw) + ';';
        if ($('wifi_hidden').checked) s += 'H:true;';
        return s + ';';
      }
      case 'vcard': {
        var lines = ['BEGIN:VCARD', 'VERSION:3.0', 'N:;' + icsEsc(val('vc_name')) + ';;;', 'FN:' + icsEsc(val('vc_name'))];
        var org = val('vc_org'); if (org) lines.push('ORG:' + icsEsc(org));
        var tel = val('vc_tel'); if (tel) lines.push('TEL:' + icsEsc(tel));
        var em = val('vc_email'); if (em) lines.push('EMAIL:' + icsEsc(em));
        lines.push('END:VCARD');
        return lines.join('\n');
      }
      case 'email': {
        var q = []; var subj = val('em_subject'), body = val('em_body');
        if (subj) q.push('subject=' + encodeURIComponent(subj));
        if (body) q.push('body=' + encodeURIComponent(body));
        return 'mailto:' + val('em_to') + (q.length ? '?' + q.join('&') : '');
      }
      case 'phone': return 'tel:' + val('ph_num');
      case 'sms': {
        var body2 = val('sms_body');
        return 'sms:' + val('sms_num') + (body2 ? '?body=' + encodeURIComponent(body2) : '');
      }
      case 'event': {
        var el = ['BEGIN:VCALENDAR', 'VERSION:2.0', 'BEGIN:VEVENT', 'SUMMARY:' + icsEsc(val('ev_title'))];
        var ds = icsDate(val('ev_start')); if (ds) el.push('DTSTART:' + ds);
        var de = icsDate(val('ev_end')); if (de) el.push('DTEND:' + de);
        var loc = val('ev_loc'); if (loc) el.push('LOCATION:' + icsEsc(loc));
        var desc = val('ev_desc'); if (desc) el.push('DESCRIPTION:' + icsEsc(desc));
        el.push('END:VEVENT', 'END:VCALENDAR');
        return el.join('\n');
      }
      case 'geo': {
        var lat = val('geo_lat'), lon = val('geo_lon'), lbl = val('geo_label');
        return 'geo:' + lat + ',' + lon + (lbl ? '?q=' + lat + ',' + lon + '(' + encodeURIComponent(lbl) + ')' : '');
      }
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

  // 인쇄용 안내 문구(비동기): 화면에서만 보이는 도움말은 인쇄해서 나눠주면 안 따라간다 —
  // 그래서 원할 때만 이미지 자체 아래에 흰 띠+문구를 실제로 그려 넣는다(다운로드 이미지에 포함).
  function withPrintLabel(dataURL, cb) {
    var im = new Image();
    im.onload = function () {
      var bandH = Math.round(im.width * 0.09);
      var cv = document.createElement('canvas'); cv.width = im.width; cv.height = im.height + bandH;
      var cx = cv.getContext('2d');
      cx.fillStyle = '#ffffff'; cx.fillRect(0, 0, cv.width, cv.height);
      cx.drawImage(im, 0, 0);
      var msg = '📷 WIA Code 스캐너로 스캔하세요 · wiacode.com';
      cx.fillStyle = '#0b0f14';
      var fontPx = Math.max(14, Math.round(bandH * 0.34));
      cx.font = fontPx + 'px -apple-system, system-ui, "Apple SD Gothic Neo", sans-serif';
      cx.textAlign = 'center'; cx.textBaseline = 'middle';
      // 캔버스 폭에 안 맞으면 글자 크기를 줄여서 한 줄에 맞춘다(다국어 폭 차이 대응).
      while (cx.measureText(msg).width > cv.width * 0.94 && fontPx > 10) {
        fontPx -= 1; cx.font = fontPx + 'px -apple-system, system-ui, "Apple SD Gothic Neo", sans-serif';
      }
      cx.fillText(msg, cv.width / 2, im.height + bandH / 2);
      cb(cv.toDataURL('image/png'));
    };
    im.onerror = function () { cb(dataURL); };
    im.src = dataURL;
  }

  var BPC_LABEL = { 1: '흑백', 2: '4단계', 3: '8단계' };

  function run() {
    var out = $('out');
    var text = buildPayload();
    if (!text) { out.innerHTML = '<div class="err">내용을 입력하세요.</div>'; return; }
    var grid = state.grid, bpc = state.bpc;
    var opts = { text: text, grid: grid, bpc: bpc, shape: state.shape, cellPx: 10 };
    if ($('colorOn').checked) opts.hueText = val('hueText') || ' ';
    var r;
    try { r = window.WiaScan.generate(opts); } catch (e) { r = { error: String(e) }; }
    if (!r || r.error) { out.innerHTML = '<div class="err">생성 실패: ' + (r && r.error ? r.error : '?') + '</div>'; return; }
    if (r.bytes > r.capBytes) { out.innerHTML = '<div class="err">내용이 너무 깁니다 (' + r.bytes + '자 > 용량 ' + r.capBytes + 'B). 격자를 L로 키우거나 색을 켜세요.</div>'; return; }
    var name = 'wiacode-' + state.ctype + '-' + r.grid + (r.shape !== 'square' ? '-' + r.shape : '') + (r.color ? '-color' : '') + '.png';
    var kind = { text: '텍스트', url: 'URL', wifi: 'WiFi', vcard: '연락처', email: '이메일', phone: '전화',
      sms: 'SMS', event: '일정', geo: '위치', ssh: 'SSH 접속설정', api: 'API 설정', mcp: 'MCP 부트스트랩' }[state.ctype];
    var render = function (dataURL) {
      out.innerHTML =
        '<img alt="WIA Code" src="' + dataURL + '">' +
        '<div class="meta"><b>' + kind + '</b> · ' + r.width + '×' + r.height + 'px · ' + r.grid + '/' + (r.shape) + ' · ' + (r.color ? '컬러' : (BPC_LABEL[bpc] || bpc + '단계')) +
        '<br>' + r.bytes + '자 담김 / 용량 약 ' + r.capBytes + '바이트</div>' +
        '<a class="dl" download="' + name + '" href="' + dataURL + '">⬇ PNG 다운로드</a>' +
        '<div class="meta" style="margin-top:12px">' +
        '화면/인쇄 후 <a class="scan" href="scan.html">스캐너</a>로 읽으면 이 설정이 그대로 나옵니다 (오프라인).' +
        '</div>';
    };
    var printLabelOn = $('printLabelOn') && $('printLabelOn').checked;
    if (printLabelOn) withPrintLabel(r.dataURL, render); else render(r.dataURL);
  }

  ready(function () { $('gen').addEventListener('click', run); run(); });
})();
