(function () {
  'use strict';
  var $ = function (id) { return document.getElementById(id); };
  var state = { cat: 'daily', ctype: 'text', shape: 'heart', grid: 'M', bpc: 2, mcpT: 'stdio', wifiSec: 'WPA',
    cryptoCoin: 'btc', ftMode: 'video', sipSec: 'sip', frame: 'none',
    morphOn: false, morphSel: [] };

  // BIP-21 계열 코인 — 스킴 이름만 다르고 파라미터(amount/label/message)는 전부 동일하다.
  // (Litecoin·Dogecoin 코어 모두 BIP-21을 그대로 채택했음 — dogecoin/doc/bips.md, litecoin.com/learning-center/uri-schemes)
  var COINS = {
    btc: { scheme: 'bitcoin', unit: 'BTC', ph: 'bc1q…', name: '비트코인' },
    ltc: { scheme: 'litecoin', unit: 'LTC', ph: 'ltc1… / L…', name: '라이트코인' },
    doge: { scheme: 'dogecoin', unit: 'DOGE', ph: 'D…', name: '도지코인' }
  };

  /* ============================================================
     타입 IA — 쓰임새 4그룹 (design-concepts 시안의 분류를 그대로 실배선).
     각 튜플 = [ctype값(=기존 payload 빌더/.fields data-t 와 동일), 아이콘, 표시명]
     ============================================================ */
  var GROUPS = [
    { id: 'daily', icon: '✦', name: '일상 · 공유', desc: '가장 자주 쓰는 것들 — 스캔하면 폰이 바로 알아듣는 표준 포맷.',
      types: [['text', '📝', '텍스트'], ['url', '🔗', 'URL'], ['wifi', '📶', 'WiFi'], ['geo', '📍', '위치'], ['event', '📅', '일정'], ['todo', '✅', '할 일']] },
    { id: 'contact', icon: '💬', name: '연락 · 소통', desc: '사람과 사람을 잇는 코드 — 명함·매장·행사장에서.',
      types: [['vcard', '👤', '연락처'], ['phone', '📞', '전화'], ['sms', '💬', 'SMS'], ['email', '📧', '이메일'], ['whatsapp', '💚', 'WhatsApp'], ['facetime', '🎥', 'FaceTime'], ['xmpp', '🗨️', 'XMPP'], ['sip', '☎️', 'SIP']] },
    { id: 'pay', icon: '🛡', name: '결제 · 인증', desc: '돈과 신뢰가 걸린 것들 — 표준 URI 그대로, 지갑·은행·인증 앱이 읽습니다.',
      types: [['crypto', '₿', '암호화폐'], ['sepa', '🏦', 'SEPA 이체'], ['otp', '🔐', '2단계 인증'], ['pgpkey', '🔏', 'PGP 공개키']] },
    { id: 'dev', icon: '⌘', name: '개발자 · AI', desc: 'WIA의 고용량이 빛나는 곳 — 설정·키·매니페스트를 코드 하나에.',
      types: [['ssh', '🔑', 'SSH 접속'], ['sshpubkey', '🗝️', 'SSH 공개키'], ['api', '🔌', 'API'], ['mcp', '🧩', 'MCP']] }
  ];
  var TYPE_LABEL = {}; var TYPE_ICON = {};
  GROUPS.forEach(function (g) { g.types.forEach(function (t) { TYPE_ICON[t[0]] = t[1]; TYPE_LABEL[t[0]] = t[2]; }); });
  // crypto/geo 는 하위 선택(코인)에 따라 더 구체적인 라벨을 run()에서 별도로 씀 — 목록용 기본 라벨은 위 표.

  var SHAPES = [['heart', '하트', ''], ['round', '원형', ''], ['clover', '네잎클로버', 'NEW'], ['star', '별', 'NEW'], ['boomerang', '부메랑', 'NEW'], ['hex', '육각', 'NEW'], ['square', '사각', '']];
  var SHAPE_KO = { square: '사각', round: '원형', heart: '하트', clover: '네잎클로버', star: '별', boomerang: '부메랑', hex: '육각' };

  function seg(id, key, cast, after) {
    var wrap = $(id); if (!wrap) return;
    wrap.addEventListener('click', function (e) {
      var b = e.target.closest('button'); if (!b) return;
      Array.prototype.forEach.call(wrap.children, function (c) { c.classList.toggle('on', c === b); });
      state[key] = cast ? cast(b.getAttribute('data-v')) : b.getAttribute('data-v');
      if (after) after();
    });
  }
  function setSegOn(id, v) {
    var wrap = $(id); if (!wrap) return;
    Array.prototype.forEach.call(wrap.children, function (c) { c.classList.toggle('on', c.getAttribute('data-v') === String(v)); });
  }

  // 옵션 바꾸면 즉시 재생성(라이브 미리보기). run 은 아래에서 함수선언(호이스팅)됨.
  var regen = function () { run(); };
  seg('grid', 'grid', null, regen);
  seg('bpc', 'bpc', function (v) { return parseInt(v, 10); }, regen);
  seg('mcp_transport', 'mcpT', null, regen);
  seg('wifi_sec', 'wifiSec', null, regen);
  // 이더리움만 표준이 아예 다르다(EIP-681: value=wei) — BTC/LTC/DOGE는 BIP-21 한 벌로 공유한다.
  seg('crypto_coin', 'cryptoCoin', null, function () {
    var panel = state.cryptoCoin === 'eth' ? 'eth' : 'bip21';
    Array.prototype.forEach.call(document.querySelectorAll('.coinfields'), function (f) {
      f.classList.toggle('on', f.getAttribute('data-c') === panel);
    });
    var ci = COINS[state.cryptoCoin];
    if (ci) {
      $('c_addr').placeholder = ci.ph;
      $('c_amount_lbl').textContent = '금액 ' + ci.unit + ' (선택)';
      $('c_hint').innerHTML = 'BIP-21 표준 (<b>' + ci.scheme + ':주소?amount=…</b>). 금액은 <b>' + ci.unit + ' 단위 소수</b>로 적은 그대로 들어갑니다.';
    }
    run();
  });
  seg('ft_mode', 'ftMode', null, regen);
  seg('sip_sec', 'sipSec', null, regen);
  seg('frame', 'frame', null, regen);
  $('colorOn').addEventListener('change', function () { $('hueWrap').classList.toggle('on', this.checked); regen(); });
  $('bridgeOn').addEventListener('change', regen);
  $('wifi_hidden').addEventListener('change', regen);
  $('styleOn').addEventListener('change', function () { $('styleWrap').classList.toggle('on', this.checked); regen(); });
  $('styleGradOn').addEventListener('change', function () { $('styleFg2Wrap').style.display = this.checked ? 'block' : 'none'; regen(); });
  $('styleEyeOn').addEventListener('change', function () { $('styleEyeWrap').style.display = this.checked ? 'block' : 'none'; regen(); });
  var dbc; var debC = function () { clearTimeout(dbc); dbc = setTimeout(regen, 150); };
  ['styleFg', 'styleBg', 'styleFg2', 'styleEye'].forEach(function (id) { $(id).addEventListener('input', debC); });
  $('logoOn').addEventListener('change', function () { $('logoWrap').style.display = this.checked ? 'block' : 'none'; regen(); });
  $('logoFile').addEventListener('change', function () { loadLogoFile(this.files && this.files[0]); });
  $('logoSize').addEventListener('input', debC);
  // 텍스트 입력은 디바운스 재생성 — content-panel 안 어떤 input/textarea든(동적으로 .fields가 갈려도) 위임으로 잡는다.
  var dbt; var deb = function () { clearTimeout(dbt); dbt = setTimeout(regen, 350); };
  $('contentPanel').addEventListener('input', function (e) {
    var t = e.target;
    if (t && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA')) {
      if (state.ctype === 'geo') { /* no-op: pin code 연동 없음(시안 목업 기능은 이식하지 않음) */ }
      deb();
    }
  });

  var val = function (id) { var e = $(id); return e ? (e.value || '').trim() : ''; };

  // "YYYY-MM-DD HH:MM" → iCalendar DTSTART/DTEND용 "YYYYMMDDTHHMMSS"(로컬시간, 지정 안 되면 빈 문자열).
  function icsDate(s) {
    var m = /^(\d{4})-(\d{2})-(\d{2})[ T](\d{2}):(\d{2})/.exec(s || '');
    if (!m) return '';
    return m[1] + m[2] + m[3] + 'T' + m[4] + m[5] + '00';
  }
  // vCard/iCalendar는 필드 안의 개행·쉼표·세미콜론을 이스케이프해야 표준 파서가 안 깨짐(RFC 6350/5545).
  function icsEsc(s) { return (s || '').replace(/\\/g, '\\\\').replace(/\n/g, '\\n').replace(/,/g, '\\,').replace(/;/g, '\\;'); }

  // ETH(사람이 쓰는 소수) → wei(EIP-681 `value`가 요구하는 원자단위 정수 문자열).
  // ★부동소수 곱셈(Number(s) * 1e18) 금지 — 0.1처럼 2진수로 딱 안 떨어지는 값에서 오차가 나
  // 지갑이 엉뚱한 금액을 제안한다(0.1 → 100000000000000001…). 그래서 문자열 자리이동으로만 계산한다.
  function ethToWei(s) {
    s = (s || '').trim();
    if (!s || s === '.' || !/^\d*\.?\d*$/.test(s)) return '';
    var p = s.split('.');
    var frac = (p[1] || '').slice(0, 18);
    while (frac.length < 18) frac += '0';
    return ((p[0] || '') + frac).replace(/^0+/, '');
  }

  // EUR 금액 → EPC069-12가 요구하는 "소수점 둘째 자리까지" 문자열(EUR12.50). 부동소수 대신 문자열 계산.
  function eurAmt(s) {
    s = (s || '').replace(/[\s,]/g, '');
    if (!s || s === '.' || !/^\d*\.?\d*$/.test(s)) return '';
    var p = s.split('.');
    var ip = (p[0] || '').replace(/^0+/, '') || '0';
    var fp = ((p[1] || '') + '00').slice(0, 2);
    if (ip === '0' && fp === '00') return '';
    return ip + '.' + fp;
  }

  // 콘텐츠 타입 → payload 문자열 (구조적 타입은 표준 URI/포맷 — 스캔한 폰이 그대로 처리 가능).
  // ★ 기존 live generate.js 의 buildPayload() 를 한 글자도 바꾸지 않고 그대로 재사용한다.
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
      case 'crypto': {
        if (state.cryptoCoin === 'eth') {
          var ea = val('eth_addr'); if (!ea) return '';
          var ch = val('eth_chain');
          var es = 'ethereum:' + ea + (ch ? '@' + ch : '');
          var wei = ethToWei(val('eth_amount'));
          return es + (wei ? '?value=' + wei : '');
        }
        var ci = COINS[state.cryptoCoin] || COINS.btc;
        var ca = val('c_addr'); if (!ca) return '';
        var cq = [];
        var cam = val('c_amount'); if (cam) cq.push('amount=' + encodeURIComponent(cam));
        var cl = val('c_label'); if (cl) cq.push('label=' + encodeURIComponent(cl));
        var cm = val('c_msg'); if (cm) cq.push('message=' + encodeURIComponent(cm));
        return ci.scheme + ':' + ca + (cq.length ? '?' + cq.join('&') : '');
      }
      case 'sepa': {
        var sn = val('sepa_name'), sib = val('sepa_iban').replace(/\s/g, '').toUpperCase();
        if (!sn || !sib) return '';
        var sbic = val('sepa_bic').replace(/\s/g, '').toUpperCase();
        var samt = eurAmt(val('sepa_amount'));
        var L = ['BCD', '002', '1', 'SCT', sbic, sn, sib, samt ? 'EUR' + samt : '', '', '', val('sepa_ref')];
        while (L.length && L[L.length - 1] === '') L.pop();
        return L.join('\n');
      }
      case 'todo': {
        var tt = val('td_title'); if (!tt) return '';
        var tl = ['BEGIN:VCALENDAR', 'VERSION:2.0', 'BEGIN:VTODO', 'SUMMARY:' + icsEsc(tt)];
        var due = icsDate(val('td_due')); if (due) tl.push('DUE:' + due);
        var tdd = val('td_desc'); if (tdd) tl.push('DESCRIPTION:' + icsEsc(tdd));
        tl.push('END:VTODO', 'END:VCALENDAR');
        return tl.join('\n');
      }
      case 'pgpkey':
        return val('pgp_key');
      case 'sshpubkey':
        return val('sshpub_key');
      case 'facetime': {
        var ft = val('ft_target'); if (!ft) return '';
        if (ft.indexOf('@') < 0) ft = ft.replace(/[^\d+]/g, '');
        if (!ft) return '';
        return (state.ftMode === 'audio' ? 'facetime-audio://' : 'facetime://') + ft;
      }
      case 'xmpp': {
        var jid = val('xm_jid').replace(/\s/g, ''); if (!jid) return '';
        var xb = val('xm_body');
        return 'xmpp:' + jid + (xb ? '?message;body=' + encodeURIComponent(xb) : '');
      }
      case 'sip': {
        var sa = val('sip_addr').replace(/\s/g, ''); if (!sa) return '';
        if (/^sips?:/i.test(sa)) return sa;
        return (state.sipSec === 'sips' ? 'sips:' : 'sip:') + sa;
      }
      case 'otp': {
        var iss = val('otp_issuer'), acct = val('otp_account');
        var sec = val('otp_secret').replace(/\s+/g, '').toUpperCase();
        if (!sec || !acct) return '';
        var lab = iss ? iss + ':' + acct : acct;
        var os = 'otpauth://totp/' + encodeURIComponent(lab) + '?secret=' + encodeURIComponent(sec);
        if (iss) os += '&issuer=' + encodeURIComponent(iss);
        return os + '&algorithm=SHA1&digits=' + (val('otp_digits') || '6') + '&period=' + (val('otp_period') || '30');
      }
      case 'whatsapp': {
        var wn = val('wa_num').replace(/\D/g, ''); if (!wn) return '';
        var wm = val('wa_msg');
        return 'https://wa.me/' + wn + (wm ? '?text=' + encodeURIComponent(wm) : '');
      }
      case 'ssh': {
        var o = { t: 'ssh', host: val('ssh_host'), port: val('ssh_port') || '22', user: val('ssh_user') };
        var k = val('ssh_key'); if (k) o.key = k;
        return JSON.stringify(o);
      }
      case 'api': {
        var o2 = { t: 'api', name: val('api_name'), url: val('api_url'), key: val('api_key') };
        var h = val('api_headers'); if (h) { try { o2.headers = JSON.parse(h); } catch (e) { o2.headers = h; } }
        return JSON.stringify(o2);
      }
      case 'mcp': {
        var o3 = { t: 'mcp', name: val('mcp_name'), transport: state.mcpT, endpoint: val('mcp_ep') };
        var tk = val('mcp_token'); if (tk) o3.token = tk;
        var hs = val('mcp_hash'); if (hs) o3.sha256 = hs;
        return JSON.stringify(o3);
      }
      default: return val('f_text');
    }
  }

  function ready(cb) {
    if (window.WiaScan) { $('badge').textContent = 'ENGINE: real'; cb(); }
    else { $('badge').textContent = 'ENGINE 로딩…'; setTimeout(function () { ready(cb); }, 150); }
  }

  // WIA 브랜드 로고 프리로드(QR 중앙 삽입용)
  var wiaLogo = new Image(); wiaLogo.src = window.WIA_LOGO_URI || 'wia-logo.png';

  // 사용자 업로드 로고(파일 선택 시 비동기 디코드 후 보관, run()이 재호출되며 반영)
  var userLogo = null;
  function loadLogoFile(file) {
    if (!file) { userLogo = null; regen(); return; }
    var reader = new FileReader();
    reader.onload = function (e) {
      var img = new Image();
      img.onload = function () { userLogo = img; regen(); };
      img.onerror = function () { userLogo = null; };
      img.src = e.target.result;
    };
    reader.readAsDataURL(file);
  }

  // 사용자 로고 삽입(비동기): 다리QR과 같은 중앙 자리를 쓰므로 다리QR이 켜져 있으면 건너뜀.
  function withLogo(dataURL, cb) {
    var on = $('logoOn') && $('logoOn').checked, bridge = $('bridgeOn') && $('bridgeOn').checked;
    if (!on || bridge || !userLogo || !userLogo.naturalWidth || !window.WiaLogoFrame) { cb(dataURL); return; }
    var im = new Image();
    im.onload = function () {
      var cv = document.createElement('canvas'); cv.width = im.width; cv.height = im.height;
      cv.getContext('2d').drawImage(im, 0, 0);
      window.WiaLogoFrame.embedLogo(cv, userLogo, (parseInt($('logoSize').value, 10) || 15) / 100);
      cb(cv.toDataURL('image/png'));
    };
    im.onerror = function () { cb(dataURL); };
    im.src = dataURL;
  }

  // 다리 QR 오버레이(비동기): dataURL → canvas → WiaBridge.overlay(브랜드) → 로고 → 새 dataURL
  function withBridge(dataURL, cb) {
    if (!window.WiaBridge) { cb(dataURL); return; }
    var im = new Image();
    im.onload = function () {
      var cv = document.createElement('canvas'); cv.width = im.width; cv.height = im.height;
      var cx = cv.getContext('2d'); cx.drawImage(im, 0, 0);
      var id = cx.getImageData(0, 0, cv.width, cv.height);
      var qpx = Math.max(3, Math.floor(cv.width * 0.08 / window.WiaBridge.SIZE));
      var pos = window.WiaBridge.overlay(id, { place: 'below-core', qpx: qpx });
      cx.putImageData(id, 0, 0);
      var finish = function () {
        if (pos.logoHalf && wiaLogo.naturalWidth) {
          var side = pos.logoHalf * 2 * 0.94;
          cx.drawImage(wiaLogo, pos.logoCx - side / 2, pos.logoCy - side / 2, side, side);
        }
        cb(cv.toDataURL('image/png'));
      };
      if (wiaLogo.complete && wiaLogo.naturalWidth) finish();
      else { wiaLogo.onload = finish; wiaLogo.onerror = finish; }
    };
    im.onerror = function () { cb(dataURL); };
    im.src = dataURL;
  }

  // 프레임 템플릿(비동기): 코드 이미지 자체는 안 건드리고 바깥에 액자+안내문구를 둘러 그린다.
  function withFrame(dataURL, frameKey, bridge, cb) {
    if (frameKey === 'none' || !window.WiaLogoFrame) { cb(dataURL); return; }
    var im = new Image();
    im.onload = function () {
      var cv = document.createElement('canvas'); cv.width = im.width; cv.height = im.height;
      cv.getContext('2d').drawImage(im, 0, 0);
      var label = bridge ? '📷 아무 카메라로 비추세요 · SCAN ME' : '📷 WIA Code 스캐너로 스캔 · wiacode.com';
      var out = window.WiaLogoFrame.wrapFrame(cv, frameKey, { label: label });
      cb(out.toDataURL('image/png'));
    };
    im.onerror = function () { cb(dataURL); };
    im.src = dataURL;
  }

  // 다리QR이 격자를 L로 강제할 때 격자 세그먼트 버튼도 같이 L로 표시.
  var lastSyncedGrid = null;
  function syncGridUI(actualGrid) {
    if (actualGrid === lastSyncedGrid) return;
    lastSyncedGrid = actualGrid;
    setSegOn('grid', actualGrid);
  }

  var BPC_LABEL = { 1: '흑백', 2: '4단계', 3: '8단계' };

  /* ============================================================
     STEP 1 — 카테고리 탭 + 타입 그리드 (기존 payload 빌더가 읽는
     .fields[data-t] 는 손대지 않고, 그 위 선택 UI만 재구성한다)
     ============================================================ */
  function selectType(v) {
    state.ctype = v;
    var g = GROUPS.filter(function (gr) { return gr.types.some(function (t) { return t[0] === v; }); })[0];
    if (g) state.cat = g.id;
    buildTabs(); buildTypes();
    Array.prototype.forEach.call(document.querySelectorAll('.fields'), function (f) {
      f.classList.toggle('on', f.getAttribute('data-t') === state.ctype);
    });
    updateContentHead();
  }
  function updateContentHead() {
    var ic = $('cpIcon'), nm = $('cpName');
    if (ic) ic.textContent = TYPE_ICON[state.ctype] || '❓';
    if (nm) nm.textContent = TYPE_LABEL[state.ctype] || state.ctype;
  }
  function buildTabs() {
    var wrap = $('catTabs'); if (!wrap) return;
    wrap.innerHTML = '';
    GROUPS.forEach(function (g) {
      var b = document.createElement('button');
      b.type = 'button';
      b.classList.toggle('on', g.id === state.cat);
      b.innerHTML = '<span>' + g.icon + '</span> ' + g.name + ' <span class="cnt">' + g.types.length + '</span>';
      b.addEventListener('click', function () { state.cat = g.id; buildTabs(); buildTypes(); });
      wrap.appendChild(b);
    });
    var cur = GROUPS.filter(function (g) { return g.id === state.cat; })[0];
    $('catDesc').textContent = cur ? cur.desc : '';
  }
  function buildTypes() {
    var wrap = $('ctype'); if (!wrap) return;
    wrap.innerHTML = '';
    var g = GROUPS.filter(function (gr) { return gr.id === state.cat; })[0];
    if (!g) return;
    g.types.forEach(function (t) {
      var v = t[0], ic = t[1], nm = t[2];
      var b = document.createElement('button');
      b.type = 'button';
      b.className = 'type-card' + (v === state.ctype ? ' on' : '');
      b.innerHTML = '<span class="ic">' + ic + '</span><span class="nm">' + nm + '</span>';
      b.addEventListener('click', function () { selectType(v); run(); });
      wrap.appendChild(b);
    });
  }

  /* ============================================================
     STEP 3 — 실루엣 갤러리. 썸네일은 진짜 window.WiaScan.generate()
     결과(작은 S 격자, 고정 데모문구) — 시안의 가짜 drawOrbitCode() 는
     이식하지 않는다. 클릭 시: 일반모드=모양 선택, 모핑모드=다중선택.
     ============================================================ */
  var SHAPE_THUMB_CACHE = {};
  function renderShapeThumbs() {
    SHAPES.forEach(function (t) {
      var v = t[0];
      if (SHAPE_THUMB_CACHE[v] === undefined) {
        var du = null;
        try {
          var r = window.WiaScan && window.WiaScan.generate({ text: 'WIA CODE', grid: 'S', bpc: 1, shape: v, cellPx: 3 });
          du = (r && !r.error) ? r.dataURL : null;
        } catch (e) { du = null; }
        SHAPE_THUMB_CACHE[v] = du;
      }
      var img = document.querySelector('img[data-shape-thumb="' + v + '"]');
      if (img && SHAPE_THUMB_CACHE[v]) img.src = SHAPE_THUMB_CACHE[v];
    });
  }
  function buildShapeRow() {
    var wrap = $('shape'); if (!wrap) return;
    wrap.innerHTML = '';
    SHAPES.forEach(function (t) {
      var v = t[0], nm = t[1], tag = t[2];
      var isPrimary = v === state.shape;
      var selIx = state.morphSel.indexOf(v);
      var b = document.createElement('button');
      b.type = 'button';
      b.className = 'shape-card' + (isPrimary ? ' on' : '') + (state.morphOn && (isPrimary || selIx >= 0) ? ' sel' : '');
      var ord = !state.morphOn ? '' : isPrimary ? '<span class="ord">스캔</span>' : (selIx >= 0 ? '<span class="ord">' + (selIx + 1) + '</span>' : '');
      b.innerHTML = ord + (tag ? '<span class="tag">' + tag + '</span>' : '') +
        '<img data-shape-thumb="' + v + '" width="76" height="76" alt="' + nm + ' 모양 WIA 코드 실제 미리보기" style="width:76px;height:76px;object-fit:contain;background:#fff;border-radius:8px;">' +
        '<span class="nm">' + nm + '</span>';
      b.addEventListener('click', function () {
        if (state.morphOn) {
          if (isPrimary) return; // 현재 실루엣은 항상 영상 마지막 스캔 프레임으로 포함됨
          var ix = state.morphSel.indexOf(v);
          if (ix >= 0) state.morphSel.splice(ix, 1); else state.morphSel.push(v);
          buildShapeRow(); syncMorphUI();
          return;
        }
        state.shape = v;
        buildShapeRow();
        run();
      });
      wrap.appendChild(b);
    });
    renderShapeThumbs();
  }

  /* ============================================================
     실루엣 모핑 쇼케이스 — 진짜 real 배선.
     wiascan-core.js 의 공개 API(window.WiaScan.generate)는 이름있는
     실루엣(square/round/heart/clover/star/hex)만 받고, insideShape()의
     각도 경계를 두 모양 사이로 연속 보간하는 훅은 노출돼 있지 않다
     (geometry.js 확인 완료 — layout()/render() 어디에도 커스텀 경계
     오버라이드 파라미터가 없음, insideShape()는 L.shape 문자열로 분기).
     core를 수정하지 않는 것이 이번 작업 범위이므로, 시안이 스스로도
     허용한 대체안을 그대로 택한다:
       - 시퀀스의 각 모양 = 그 모양으로 실제 인코딩된 진짜 WIA 코드
         (real window.WiaScan.generate() 호출, 매 프레임 재인코딩 아님)
       - 두 실제 키프레임 사이의 "전환" 구간만 픽셀 크로스페이드로 근사
       - 영상 마지막 정지 구간 = 현재 state.shape 의 실제 키프레임 그대로
     ============================================================ */
  var MORPH_HOLD = 850, MORPH_TRANS = 900;
  function easeMorph(t) { return t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2; }
  function morphSeqNow() {
    var seq = state.morphSel.filter(function (s) { return s !== state.shape; });
    seq.push(state.shape);
    return seq;
  }
  function buildMorphKeyframes(seq, cellPx, cb) {
    var text = buildPayload();
    var hueOpt = ($('colorOn').checked ? (val('hueText') || ' ') : null);
    var imgs = {}, remaining = seq.length;
    if (!remaining) { cb(imgs); return; }
    seq.forEach(function (s) {
      var o = { text: text, grid: state.grid, bpc: state.bpc, shape: s, cellPx: cellPx };
      if (hueOpt) o.hueText = hueOpt;
      var r;
      try { r = window.WiaScan.generate(o); } catch (e) { r = null; }
      if (!r || r.error || !r.dataURL) { remaining--; if (remaining === 0) cb(imgs); return; }
      var im = new Image();
      im.onload = function () { imgs[s] = im; remaining--; if (remaining === 0) cb(imgs); };
      im.onerror = function () { remaining--; if (remaining === 0) cb(imgs); };
      im.src = r.dataURL;
    });
  }
  function drawCrossfade(canvas, imgA, imgB, e, bg) {
    if (!imgA) return;
    var w = imgA.naturalWidth || imgA.width, h = imgA.naturalHeight || imgA.height;
    if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
    var ctx = canvas.getContext('2d');
    if (bg) { ctx.fillStyle = bg; ctx.fillRect(0, 0, w, h); } else ctx.clearRect(0, 0, w, h);
    ctx.globalAlpha = 1; ctx.drawImage(imgA, 0, 0, w, h);
    if (imgB && e > 0) { ctx.globalAlpha = e; ctx.drawImage(imgB, 0, 0, w, h); ctx.globalAlpha = 1; }
  }
  var morphRaf = 0, morphToken = 0;
  function startMorphPreview() {
    cancelAnimationFrame(morphRaf); morphRaf = 0;
    var seq = morphSeqNow();
    if (seq.length < 2) return;
    var token = ++morphToken;
    buildMorphKeyframes(seq, 6, function (imgs) {
      if (token !== morphToken || !state.morphOn) return;
      var canvas = $('morphCanvas');
      $('codeImg').style.display = 'none';
      $('previewEmpty').style.display = 'none';
      canvas.style.display = 'block';
      var t0 = performance.now();
      var seg = MORPH_HOLD + MORPH_TRANS, total = seq.length * seg;
      (function tick(now) {
        if (token !== morphToken || !state.morphOn) { morphRaf = 0; return; }
        var m = (now - t0) % total;
        var i = Math.floor(m / seg), local = m - i * seg, j = (i + 1) % seq.length;
        var e = local < MORPH_HOLD ? 0 : easeMorph((local - MORPH_HOLD) / MORPH_TRANS);
        drawCrossfade(canvas, imgs[seq[i]], imgs[seq[j]], e, '#ffffff');
        morphRaf = requestAnimationFrame(tick);
      })(t0);
    });
  }
  function stopMorphPreview() {
    morphToken++; cancelAnimationFrame(morphRaf); morphRaf = 0;
    var canvas = $('morphCanvas'); if (canvas) canvas.style.display = 'none';
  }
  function syncMorphUI() {
    document.body.classList.toggle('morph-mode', state.morphOn);
    var bar = $('morphBar');
    bar.classList.toggle('show', state.morphOn);
    if (!state.morphOn) { stopMorphPreview(); if (!$('previewErr') || $('previewErr').style.display === 'none') $('codeImg').style.display = lastGoodDataURL ? 'block' : 'none'; return; }
    var seq = morphSeqNow();
    var lab = $('morphSeqLab'), exp = $('morphExport');
    if (seq.length < 2) {
      lab.innerHTML = '실루엣 카드를 눌러 함께 흐를 모양을 <b>2개 이상</b> 고르세요.';
      exp.disabled = true;
      stopMorphPreview();
    } else {
      lab.innerHTML = seq.map(function (s, i) {
        return i === seq.length - 1 ? ('<b>' + SHAPE_KO[s] + '</b> (스캔 프레임)') : SHAPE_KO[s];
      }).join(' → ');
      exp.disabled = false;
      startMorphPreview();
    }
  }
  $('morphOn').addEventListener('change', function () {
    state.morphOn = this.checked;
    buildShapeRow(); syncMorphUI();
  });

  var morphRecording = false;
  function exportMorphVideo(btn) {
    if (morphRecording) return;
    var seq = morphSeqNow();
    if (seq.length < 2) return;
    var candidates = ['video/webm;codecs=vp9', 'video/webm;codecs=vp8', 'video/webm', 'video/mp4'];
    var mime = null;
    for (var i = 0; i < candidates.length; i++) {
      if (window.MediaRecorder && MediaRecorder.isTypeSupported(candidates[i])) { mime = candidates[i]; break; }
    }
    if (!mime) { alert('이 브라우저는 캔버스 영상 녹화(MediaRecorder)를 지원하지 않습니다.'); return; }
    morphRecording = true;
    var orig = btn.textContent;
    btn.disabled = true;
    buildMorphKeyframes(seq, 9, function (imgs) {
      var missing = seq.filter(function (s) { return !imgs[s]; });
      if (missing.length) {
        alert('영상용 코드 생성 실패: ' + missing.join(', '));
        morphRecording = false; btn.disabled = false; btn.textContent = orig;
        return;
      }
      var rc = document.createElement('canvas');
      rc.width = imgs[seq[0]].naturalWidth; rc.height = imgs[seq[0]].naturalHeight;
      var stream, rec;
      try {
        stream = rc.captureStream(30);
        rec = new MediaRecorder(stream, { mimeType: mime, videoBitsPerSecond: 6e6 });
      } catch (e) {
        alert('영상 녹화 초기화 실패: ' + e);
        morphRecording = false; btn.disabled = false; btn.textContent = orig;
        return;
      }
      var chunks = [];
      rec.ondataavailable = function (ev) { if (ev.data && ev.data.size) chunks.push(ev.data); };
      var stopped = new Promise(function (res) { rec.onstop = res; });
      rec.start(200);
      var seg = MORPH_HOLD + MORPH_TRANS, main = (seq.length - 1) * seg, total = main + 1800;
      var t0 = performance.now();
      (function tick(now) {
        var t = now - t0, i, j, e;
        if (t >= main) { i = j = seq.length - 1; e = 0; }
        else {
          i = Math.floor(t / seg);
          var local = t - i * seg; j = i + 1;
          e = local < MORPH_HOLD ? 0 : easeMorph((local - MORPH_HOLD) / MORPH_TRANS);
        }
        drawCrossfade(rc, imgs[seq[i]], imgs[seq[j]], e, '#ffffff');
        btn.textContent = '● 녹화 중… ' + Math.min(100, Math.round(t / total * 100)) + '%';
        if (t >= total) {
          rec.stop();
          stopped.then(function () {
            var ext = mime.indexOf('mp4') === 0 || mime.indexOf('video/mp4') === 0 ? 'mp4' : 'webm';
            var blob = new Blob(chunks, { type: mime.split(';')[0] });
            var a = document.createElement('a');
            a.href = URL.createObjectURL(blob);
            a.download = 'wiacode-morph-' + seq.join('-') + '.' + ext;
            document.body.appendChild(a); a.click(); a.remove();
            setTimeout(function () { URL.revokeObjectURL(a.href); }, 5000);
            morphRecording = false; btn.disabled = false; btn.textContent = orig;
          });
        } else {
          requestAnimationFrame(tick);
        }
      })(t0);
    });
  }
  $('morphExport').addEventListener('click', function (e) { exportMorphVideo(e.currentTarget); });

  /* ============================================================
     용량 게이지 — WIA 막대는 지금 설정으로 실제 window.WiaScan.generate()가
     반환한 real capBytes/bytes. 내용이 용량을 넘겨 generate()가 에러를
     반환할 때도, 같은 grid/bpc/shape/hueText로 최소 텍스트를 넣어 real
     capBytes만 다시 얻는 "탐침 호출"을 한다(용량 숫자 자체는 언제나 real).
     QR 등가 막대만 orbit/BENCHMARK.md 실측표를 인용한 정적 참고치 —
     WIA는 실루엣이 사각이 아니면 실제로 그 표보다 여유가 더/덜 있을 수
     있어 실루엣별 real 수치와 나란히 두어 비교 맥락만 제공한다.
     ============================================================ */
  var QR_CAPS = { S: [344, 458, 690], M: [824, 1099, 1646], L: [1492, 1994, 2994] };
  var MODE_KO = { 1: '흑백 · 1bit/셀', 2: '4단계 · 2bit/셀', 3: '8단계 · 3bit/셀' };
  function probeCap(grid, bpc, shape, hueText) {
    var o = { text: ' ', grid: grid, bpc: bpc, shape: shape, cellPx: 4 };
    if (hueText) o.hueText = hueText;
    try {
      var p = window.WiaScan.generate(o);
      return (p && !p.error && p.capBytes != null) ? p.capBytes : null;
    } catch (e) { return null; }
  }
  function findBestFit(used, shape, hueText) {
    var grids = ['S', 'M', 'L'], bpcs = [1, 2, 3], best = null, bestCap = Infinity;
    for (var gi = 0; gi < grids.length; gi++) for (var bi = 0; bi < bpcs.length; bi++) {
      var c = probeCap(grids[gi], bpcs[bi], shape, hueText);
      if (c != null && c >= used && c < bestCap) { bestCap = c; best = { grid: grids[gi], bpc: bpcs[bi], cap: c }; }
    }
    return best;
  }
  function updateCapacityGauge(r, text, hueText) {
    var grid = state.grid, bpc = state.bpc;
    var qrCap = QR_CAPS[grid][0];
    var wiaCap, used;
    if (r && !r.error && r.capBytes != null) { wiaCap = r.capBytes; used = r.bytes; }
    else {
      wiaCap = probeCap(grid, bpc, state.shape, hueText);
      used = (text || '').length + (hueText ? hueText.length : 0);
    }
    var capMultEl = $('capMult');
    if (wiaCap == null) capMultEl.textContent = '—';
    else capMultEl.textContent = bpc === 1 ? 'QR과 동급' : ('같은 크기 ×' + (wiaCap / qrCap).toFixed(1));
    $('wiaModeLab').textContent = MODE_KO[bpc] || '';
    $('qrCapLab').textContent = used + ' / ' + qrCap + ' B';
    $('wiaCapLab').textContent = used + ' / ' + (wiaCap == null ? '?' : wiaCap) + ' B';
    $('qrFill').style.width = Math.min(100, used / qrCap * 100) + '%';
    $('wiaFill').style.width = wiaCap ? (Math.min(100, used / wiaCap * 100) + '%') : '0%';
    $('qrTrack').classList.toggle('over', used > qrCap);
    $('wiaTrack').classList.toggle('over', wiaCap != null && used > wiaCap);

    var note = $('capNote');
    if (wiaCap != null && used > wiaCap) {
      var best = findBestFit(used, state.shape, hueText);
      if (best) {
        note.innerHTML = '지금 설정엔 다 안 담깁니다 — <a id="capUp">격자 ' + best.grid + ' · ' + MODE_KO[best.bpc].split(' ')[0] + '로 올리기 (실측 ' + best.cap + ' B)</a>';
        var a = document.getElementById('capUp');
        if (a) a.onclick = function () {
          state.grid = best.grid; state.bpc = best.bpc;
          setSegOn('grid', best.grid); setSegOn('bpc', best.bpc);
          run();
        };
      } else {
        note.textContent = '이 실루엣 · 최대 설정(L·8단계)으로도 다 담기지 않습니다 — 내용을 줄이거나 다른 실루엣을 선택하세요.';
      }
    } else if (used > qrCap) {
      note.innerHTML = '🔥 같은 크기 QR은 여기서 끝 — WIA에는 아직 <b>' + (wiaCap - used) + ' B 여유</b>가 있습니다.';
    } else if (bpc === 1) {
      note.textContent = '흑백 모드는 QR과 같은 밀도 — 4단계·8단계를 켜면 같은 크기에 더 담깁니다.';
    } else if (wiaCap != null) {
      note.innerHTML = '같은 물리 크기에서 QR보다 <b>+' + (wiaCap - qrCap) + ' B</b> 더 담깁니다.';
    } else {
      note.textContent = '';
    }
  }

  // 무거운 예시 페이로드 — 실제 필드에 채워 넣어 게이지가 진짜로 차오르게(합성 더미 텍스트, 진짜 PGP 키 아님).
  function mulberry32(a) { return function () { a |= 0; a = a + 0x6D2B79F5 | 0; var t = Math.imul(a ^ a >>> 15, 1 | a); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; }; }
  var enc = window.TextEncoder ? new TextEncoder() : null;
  function bytesOf(s) { return enc ? enc.encode(s).length : s.length; }
  function armorSample(targetBytes) {
    var head = '-----BEGIN PGP PUBLIC KEY BLOCK-----\n\n', tail = '\n-----END PGP PUBLIC KEY BLOCK-----';
    var chars = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/';
    var rng = mulberry32(42), body = '';
    while (bytesOf(head + body + tail) < targetBytes) {
      var line = ''; for (var i = 0; i < 64; i++) line += chars[(rng() * 64) | 0];
      body += line + '\n';
    }
    return head + body + tail;
  }
  var SAMPLES = {
    wifi: { ctype: 'wifi', fill: function () { $('wifi_ssid').value = 'WIA-Guest-5F-Conference'; $('wifi_pw').value = 'correct-horse-battery-staple-2026!'; } },
    pgpEcc: { ctype: 'pgpkey', fill: function () { $('pgp_key').value = armorSample(980); } },
    pgpRsa: { ctype: 'pgpkey', fill: function () { $('pgp_key').value = armorSample(1750); } }
  };
  Array.prototype.forEach.call(document.querySelectorAll('.cb-samples button'), function (btn) {
    btn.addEventListener('click', function () {
      var s = SAMPLES[btn.getAttribute('data-s')];
      if (!s) return;
      selectType(s.ctype);
      s.fill();
      run();
    });
  });

  /* ============================================================
     테마 토글 — data-theme 스위칭 + 저장. 실제 생성 PNG는 항상 흰바탕/
     검정 잉크(엔진 자체가 그렇게 렌더 — opts.style 은 core generate()가
     현재 읽지 않음, 기존 live 페이지와 동일한 실제 동작) 라 테마는 UI
     크롬(패널/글자/배경)에만 영향을 준다.
     ============================================================ */
  function themeIsDark() { return document.documentElement.getAttribute('data-theme') !== 'light'; }
  function applyTheme(t) {
    document.documentElement.setAttribute('data-theme', t);
    var knob = $('themeKnob'); if (knob) knob.textContent = t === 'dark' ? '☾' : '☀';
    try { localStorage.setItem('wia-theme', t); } catch (e) {}
  }
  $('themeToggle').addEventListener('click', function () { applyTheme(themeIsDark() ? 'light' : 'dark'); });
  (function initTheme() {
    var q = new URLSearchParams(window.location.search);
    var t = q.get('theme');
    if (!t) { try { t = localStorage.getItem('wia-theme'); } catch (e) {} }
    applyTheme(t === 'light' ? 'light' : 'dark');
  })();

  /* ============================================================
     메인 생성 — 실제 payload → 실제 window.WiaScan.generate() →
     (다리QR|로고) → 프레임, 전부 기존 live 파이프라인 그대로.
     ============================================================ */
  var lastGoodDataURL = null;
  function clearPreview(msg) {
    $('previewErr').textContent = msg || '';
    $('previewErr').style.display = msg ? 'block' : 'none';
    $('previewEmpty').style.display = msg ? 'none' : 'block';
    $('codeImg').style.display = 'none';
    stopMorphPreview();
    $('dlRow').style.display = 'none';
    $('metaBox').textContent = '';
    $('scanNote').textContent = '';
    lastGoodDataURL = null;
  }
  function run() {
    var text = buildPayload();
    var bridge = $('bridgeOn') && $('bridgeOn').checked;
    var grid = state.grid, bpc = state.bpc;
    if (bridge) grid = 'L';
    syncGridUI(grid);

    var hueOpt = $('colorOn').checked ? (val('hueText') || ' ') : null;

    if (!text) { clearPreview(); updateCapacityGauge(null, '', hueOpt); return; }

    var opts = { text: text, grid: grid, bpc: bpc, shape: state.shape, cellPx: 10 };
    if (hueOpt) opts.hueText = hueOpt;
    if ($('styleOn').checked && !$('colorOn').checked) {
      opts.style = { fg: val('styleFg') || '#000000', bg: val('styleBg') || '#ffffff' };
      if ($('styleGradOn').checked) opts.style.fg2 = val('styleFg2') || opts.style.fg;
      if ($('styleEyeOn').checked) opts.style.eye = val('styleEye') || opts.style.fg;
    }

    var r;
    try { r = window.WiaScan.generate(opts); } catch (e) { r = { error: String(e) }; }

    updateCapacityGauge(r, text, hueOpt);

    if (!r || r.error) { clearPreview('생성 실패: ' + (r && r.error ? r.error : '?')); return; }
    if (r.bytes > r.capBytes) { clearPreview('내용이 너무 깁니다 (' + r.bytes + '자 > 용량 ' + r.capBytes + 'B). 격자를 키우거나 색을 켜세요.'); return; }

    var name = 'wiacode-' + state.ctype + '-' + r.grid + (r.shape !== 'square' ? '-' + r.shape : '') + (r.color ? '-color' : '') + (bridge ? '-bridge' : '') + '.png';

    var renderFinal = function (dataURL) {
      lastGoodDataURL = dataURL;
      $('previewErr').style.display = 'none';
      $('previewEmpty').style.display = 'none';
      if (!state.morphOn || morphSeqNow().length < 2) {
        $('codeImg').src = dataURL;
        $('codeImg').style.display = 'block';
        $('morphCanvas').style.display = 'none';
      }
      $('dlRow').style.display = 'flex';
      var dl = $('dlPng'); dl.href = dataURL; dl.setAttribute('download', name);
      $('metaBox').innerHTML =
        '<b>포맷</b> orbit · <b>격자</b> ' + r.grid + ' · <b>실루엣</b> ' + (SHAPE_KO[r.shape] || r.shape) + '<br>' +
        '<b>밝기</b> ' + (r.color ? '컬러(hue)' : (BPC_LABEL[bpc] || bpc + '단계')) + ' · <b>용량</b> ' + r.bytes + ' / ' + r.capBytes + ' bytes' + (bridge ? ' · 🔗다리QR' : '');
      $('scanNote').innerHTML = bridge
        ? '기본 카메라로 중앙 QR을 비추면 <a class="scan" href="scan.html">스캐너</a>로 이동합니다. 스캐너로 코드 전체를 읽으면 이 내용이 그대로 나옵니다 (오프라인).'
        : '화면/인쇄 후 <a class="scan" href="scan.html">스캐너</a>로 읽으면 이 설정이 그대로 나옵니다 (오프라인).';

      if (state.morphOn && morphSeqNow().length >= 2) startMorphPreview();
    };
    var afterCenter = function (du) { withFrame(du, state.frame, bridge, renderFinal); };
    if (bridge) withBridge(r.dataURL, afterCenter); else withLogo(r.dataURL, afterCenter);
  }

  // 쿼리스트링 프리필 — 다른 WIA 서비스에서 "WIA Code로 내보내기"로 딥링크할 때 씀.
  var PREFILL_FIELD = { url: 'f_url', text: 'f_text', phone: 'ph_num' };
  function applyPrefillFromQuery() {
    var qs = new URLSearchParams(window.location.search);
    var ctype = qs.get('ctype'), value = qs.get('value');
    if (!ctype || !PREFILL_FIELD[ctype]) return;
    selectType(ctype); // 카테고리가 달라도 정확히 그 타입으로 전환됨(버튼 querySelector 방식보다 견고)
    if (value != null) {
      var el = $(PREFILL_FIELD[ctype]);
      if (el) { el.value = value; }
    }
  }

  $('gen').addEventListener('click', run);

  // 초기 빌드
  buildTabs(); buildTypes(); buildShapeRow(); updateContentHead();
  ready(function () { applyPrefillFromQuery(); run(); });
})();
