(function () {
  'use strict';
  var $ = function (id) { return document.getElementById(id); };
  var state = { ctype: 'text', shape: 'square', grid: 'M', bpc: 2, mcpT: 'stdio', wifiSec: 'WPA',
    cryptoCoin: 'btc', ftMode: 'video', sipSec: 'sip', frame: 'none' };

  // BIP-21 계열 코인 — 스킴 이름만 다르고 파라미터(amount/label/message)는 전부 동일하다.
  // (Litecoin·Dogecoin 코어 모두 BIP-21을 그대로 채택했음 — dogecoin/doc/bips.md, litecoin.com/learning-center/uri-schemes)
  var COINS = {
    btc: { scheme: 'bitcoin', unit: 'BTC', ph: 'bc1q…', name: '비트코인' },
    ltc: { scheme: 'litecoin', unit: 'LTC', ph: 'ltc1… / L…', name: '라이트코인' },
    doge: { scheme: 'dogecoin', unit: 'DOGE', ph: 'D…', name: '도지코인' }
  };

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
  // 이더리움만 표준이 아예 다르다(EIP-681: value=wei) — BTC/LTC/DOGE는 BIP-21 한 벌로 공유한다.
  // 그래서 하위 입력칸은 'bip21' / 'eth' 두 벌만 두고 코인에 따라 갈아끼운다.
  seg('crypto_coin', 'cryptoCoin', null, function () {
    var panel = state.cryptoCoin === 'eth' ? 'eth' : 'bip21';
    Array.prototype.forEach.call(document.querySelectorAll('.coinfields'), function (f) {
      f.classList.toggle('on', f.getAttribute('data-c') === panel);
    });
    var ci = COINS[state.cryptoCoin];
    if (ci) { // 코인마다 주소 생김새·단위가 달라 안내를 같이 갈아끼움(엉뚱한 체인 주소 붙여넣기 방지)
      $('c_addr').placeholder = ci.ph;
      $('c_amount_lbl').textContent = '금액 ' + ci.unit + ' (선택)';
      $('c_hint').innerHTML = 'BIP-21 표준 (<b>' + ci.scheme + ':주소?amount=…</b>). 금액은 <b>' + ci.unit + ' 단위 소수</b>로 적은 그대로 들어갑니다.';
    }
    run();
  });
  seg('ft_mode', 'ftMode', null, regen);
  seg('sip_sec', 'sipSec', null, regen);
  $('colorOn').addEventListener('change', function () { $('hueWrap').classList.toggle('on', this.checked); regen(); });
  $('bridgeOn').addEventListener('change', regen);
  $('wifi_hidden').addEventListener('change', regen);
  seg('frame', 'frame', null, regen);
  // 색상 커스텀 — 토글 4개(켜기/그라디언트/눈색상 별도) + 색상피커 4개.
  $('styleOn').addEventListener('change', function () { $('styleWrap').classList.toggle('on', this.checked); regen(); });
  $('styleGradOn').addEventListener('change', function () { $('styleFg2Wrap').style.display = this.checked ? 'block' : 'none'; regen(); });
  $('styleEyeOn').addEventListener('change', function () { $('styleEyeWrap').style.display = this.checked ? 'block' : 'none'; regen(); });
  var dbc; var debC = function () { clearTimeout(dbc); dbc = setTimeout(regen, 150); };
  ['styleFg', 'styleBg', 'styleFg2', 'styleEye'].forEach(function (id) { $(id).addEventListener('input', debC); });
  // 로고 — 파일은 즉시(비동기 로드 후 run이 다시 그림), 크기 슬라이더는 디바운스.
  $('logoOn').addEventListener('change', function () { $('logoWrap').style.display = this.checked ? 'block' : 'none'; regen(); });
  $('logoFile').addEventListener('change', function () { loadLogoFile(this.files && this.files[0]); });
  $('logoSize').addEventListener('input', debC);
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

  // ETH(사람이 쓰는 소수) → wei(EIP-681 `value`가 요구하는 원자단위 정수 문자열).
  // ★부동소수 곱셈(Number(s) * 1e18) 금지 — 0.1처럼 2진수로 딱 안 떨어지는 값에서 오차가 나
  // 지갑이 엉뚱한 금액을 제안한다(0.1 → 100000000000000001…). 그래서 문자열 자리이동으로만 계산한다.
  // 소수점 아래 18자리(=wei)보다 잘게는 쪼갤 수 없으므로 초과분은 버린다.
  function ethToWei(s) {
    s = (s || '').trim();
    if (!s || s === '.' || !/^\d*\.?\d*$/.test(s)) return '';
    var p = s.split('.');
    var frac = (p[1] || '').slice(0, 18);
    while (frac.length < 18) frac += '0';
    return ((p[0] || '') + frac).replace(/^0+/, ''); // 전부 0이면 '' → 호출부가 value= 자체를 생략
  }

  // EUR 금액 → EPC069-12가 요구하는 "소수점 둘째 자리까지" 문자열(EUR12.50). 여기서도 부동소수는
  // 안 쓴다(12.345 * 100 같은 계산이 은행 이체액을 어긋나게 만들 수 있음). 셋째 자리 아래는 버림 —
  // 반올림하면 사용자가 적은 것보다 더 청구될 수 있으므로 항상 낮은 쪽으로.
  function eurAmt(s) {
    s = (s || '').replace(/[\s,]/g, '');
    if (!s || s === '.' || !/^\d*\.?\d*$/.test(s)) return '';
    var p = s.split('.');
    var ip = (p[0] || '').replace(/^0+/, '') || '0';
    var fp = ((p[1] || '') + '00').slice(0, 2);
    if (ip === '0' && fp === '00') return ''; // 0원 이체는 없음 → 금액 줄 자체를 생략
    return ip + '.' + fp;
  }

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
      case 'crypto': {
        if (state.cryptoCoin === 'eth') {
          // EIP-681: ethereum:<주소>[@<chainId>][?value=<wei>]  (value의 단위는 wei — ETH 아님)
          var ea = val('eth_addr'); if (!ea) return '';
          var ch = val('eth_chain');
          var es = 'ethereum:' + ea + (ch ? '@' + ch : '');
          var wei = ethToWei(val('eth_amount'));
          return es + (wei ? '?value=' + wei : '');
        }
        // BIP-21: <스킴>:<주소>[?amount=<코인단위 소수>&label=…&message=…]
        // amount는 코인 단위 소수라 사용자가 친 문자열을 ★그대로★ 넘긴다(반올림/재포맷하면 금액이 바뀐다).
        var ci = COINS[state.cryptoCoin] || COINS.btc;
        var ca = val('c_addr'); if (!ca) return '';
        var cq = [];
        var cam = val('c_amount'); if (cam) cq.push('amount=' + encodeURIComponent(cam));
        var cl = val('c_label'); if (cl) cq.push('label=' + encodeURIComponent(cl));
        var cm = val('c_msg'); if (cm) cq.push('message=' + encodeURIComponent(cm));
        return ci.scheme + ':' + ca + (cq.length ? '?' + cq.join('&') : '');
      }
      case 'sepa': {
        // EPC069-12 v3.1 (GiroCode) — ★줄 순서가 곧 필드다. 값이 없어도 자리를 비워 둬야
        // 뒤 필드가 앞으로 밀리지 않는다. 8번=금액, 9번=Purpose, 10번=정형 참조, 11번=자유 적요.
        // (여기서 9·10번을 빼먹고 적요를 9번에 넣는 실수가 흔한데, 그러면 은행이 적요를 Purpose로 읽는다.)
        var sn = val('sepa_name'), sib = val('sepa_iban').replace(/\s/g, '').toUpperCase();
        if (!sn || !sib) return '';
        var sbic = val('sepa_bic').replace(/\s/g, '').toUpperCase();
        var samt = eurAmt(val('sepa_amount'));
        var L = ['BCD', '002', '1', 'SCT', sbic, sn, sib, samt ? 'EUR' + samt : '', '', '', val('sepa_ref')];
        while (L.length && L[L.length - 1] === '') L.pop(); // 뒤쪽 빈 줄만 생략 가능(총 331바이트 제한)
        return L.join('\n');
      }
      case 'todo': {
        // RFC 5545 VTODO. 위 'event'(VEVENT)와 같은 최소 프로파일을 의도적으로 유지한다.
        var tt = val('td_title'); if (!tt) return '';
        var tl = ['BEGIN:VCALENDAR', 'VERSION:2.0', 'BEGIN:VTODO', 'SUMMARY:' + icsEsc(tt)];
        var due = icsDate(val('td_due')); if (due) tl.push('DUE:' + due);
        var tdd = val('td_desc'); if (tdd) tl.push('DESCRIPTION:' + icsEsc(tdd));
        tl.push('END:VTODO', 'END:VCALENDAR');
        return tl.join('\n');
      }
      case 'pgpkey':
        // RFC 4880 ASCII Armor 블록을 ★한 글자도 손대지 않고★ 그대로 담는다 — 줄바꿈 하나만
        // 어긋나도 CRC24 검사가 깨져서 상대가 키를 못 읽는다. 이스케이프·재정렬 절대 금지.
        return val('pgp_key');
      case 'sshpubkey':
        // authorized_keys 한 줄 형식(`ssh-ed25519 AAAA… comment`). 이것도 그대로 통과시킨다.
        return val('sshpub_key');
      case 'facetime': {
        // Apple 등록 스킴: facetime://<번호 또는 이메일>, 음성전용은 facetime-audio://
        var ft = val('ft_target'); if (!ft) return '';
        if (ft.indexOf('@') < 0) ft = ft.replace(/[^\d+]/g, ''); // 번호면 공백·하이픈 정리, 이메일은 그대로
        if (!ft) return '';
        return (state.ftMode === 'audio' ? 'facetime-audio://' : 'facetime://') + ft;
      }
      case 'xmpp': {
        // RFC 5122: xmpp:<JID>[?<querytype>[;key=value]] — ★구분자가 '&'가 아니라 ';'다.
        var jid = val('xm_jid').replace(/\s/g, ''); if (!jid) return '';
        var xb = val('xm_body');
        return 'xmpp:' + jid + (xb ? '?message;body=' + encodeURIComponent(xb) : '');
      }
      case 'sip': {
        // RFC 3261: sip:user@host (sips: 는 TLS 강제). 이미 스킴이 붙어 있으면 덧붙이지 않는다.
        var sa = val('sip_addr').replace(/\s/g, ''); if (!sa) return '';
        if (/^sips?:/i.test(sa)) return sa;
        return (state.sipSec === 'sips' ? 'sips:' : 'sip:') + sa;
      }
      case 'otp': {
        // otpauth:// (Google Key-Uri-Format = 사실상 표준, IETF draft-linuxgemini-otpauth-uri-00로 문서화 중).
        // 라벨의 "발급자:계정" 접두사와 issuer 쿼리 파라미터를 ★둘 다★ 넣는다 — 구버전 앱은 접두사를,
        // 신버전은 파라미터를 보기 때문에 둘 다 있어야 어느 인증 앱에서든 안전하게 등록된다.
        var iss = val('otp_issuer'), acct = val('otp_account');
        var sec = val('otp_secret').replace(/\s+/g, '').toUpperCase(); // 공백만 털고 검증/변환은 안 함
        if (!sec || !acct) return '';
        var lab = iss ? iss + ':' + acct : acct;
        var os = 'otpauth://totp/' + encodeURIComponent(lab) + '?secret=' + encodeURIComponent(sec);
        if (iss) os += '&issuer=' + encodeURIComponent(iss);
        return os + '&algorithm=SHA1&digits=' + (val('otp_digits') || '6') + '&period=' + (val('otp_period') || '30');
      }
      case 'whatsapp': {
        // wa.me 공식 형식: 숫자만 허용(+, 공백, 하이픈이 섞이면 링크가 깨진다) → 비숫자 전부 제거.
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
      // qsize 0.08(예전 기본 0.15) — 2026-08-10 실측: 다리QR이 흑백(bpc1, ECC 25%)의 여유를 다 먹어
      // 해독 실패로 이어졌음(0.15·0.11 실패, 0.08부터 성공). bpc를 억지로 올리는 대신 QR을 줄여서
      // "다리 켜면 무조건 흑백 포기"를 없앰 — 흑백이 인쇄+카메라 실사용에 더 강건하다는 오늘 결론과 합치.
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

  // 프레임 템플릿(비동기): 코드 이미지 자체는 안 건드리고 바깥에 액자+안내문구를 둘러 그린다
  // (2026-08-14, 예전 printLabelOn 토글을 흡수 — "없음"이 그 기본 꺼짐 상태와 동일).
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

  // 다리QR이 격자를 L로 강제할 때 격자 세그먼트 버튼도 같이 L로 표시(2026-08-10 발견: 이전엔 버튼은
  // 사용자가 마지막으로 누른 값 그대로인데 실제 생성은 L로 나가 화면-실제가 어긋났었음).
  var lastSyncedGrid = null;
  function syncGridUI(actualGrid) {
    if (actualGrid === lastSyncedGrid) return;
    lastSyncedGrid = actualGrid;
    var wrap = $('grid'); if (!wrap) return;
    Array.prototype.forEach.call(wrap.children, function (c) {
      c.classList.toggle('on', c.getAttribute('data-v') === actualGrid);
    });
  }

  var BPC_LABEL = { 1: '흑백', 2: '4단계', 3: '8단계' };

  function run() {
    var out = $('out');
    var text = buildPayload();
    if (!text) { out.innerHTML = '<div class="err">내용을 입력하세요.</div>'; return; }
    var bridge = $('bridgeOn') && $('bridgeOn').checked;
    var grid = state.grid, bpc = state.bpc;
    // 다리ON시 격자는 L로(다리QR이 먹는 셀 비율을 낮춤) — bpc는 더 이상 강제 상향 안 함(2026-08-10:
    // 위 qsize 축소와 짝지어 흑백도 안전하게 버팀, 흑백이 오히려 인쇄+카메라에 더 강건함).
    if (bridge) grid = 'L';
    syncGridUI(grid);
    var opts = { text: text, grid: grid, bpc: bpc, shape: state.shape, cellPx: 10 };
    if ($('colorOn').checked) opts.hueText = val('hueText') || ' ';
    // 색상 커스텀은 색(hue) 데이터 레이어와 같은 채널(크로마)을 써서 동시 사용 불가 — hue가 우선.
    if ($('styleOn').checked && !$('colorOn').checked) {
      opts.style = { fg: val('styleFg') || '#000000', bg: val('styleBg') || '#ffffff' };
      if ($('styleGradOn').checked) opts.style.fg2 = val('styleFg2') || opts.style.fg;
      if ($('styleEyeOn').checked) opts.style.eye = val('styleEye') || opts.style.fg;
    }
    var r;
    try { r = window.WiaScan.generate(opts); } catch (e) { r = { error: String(e) }; }
    if (!r || r.error) { out.innerHTML = '<div class="err">생성 실패: ' + (r && r.error ? r.error : '?') + '</div>'; return; }
    if (r.bytes > r.capBytes) { out.innerHTML = '<div class="err">내용이 너무 깁니다 (' + r.bytes + '자 > 용량 ' + r.capBytes + 'B). 격자를 L로 키우거나 색을 켜세요.</div>'; return; }
    // wiacode-{타입}-{격자}[-모양][-color][-bridge].png — 예전엔 접두사가 "wia-"라 이 서버의
    // 다른 잡다한 wia-* 산출물들과 뒤섞였음. 브랜드가 분명하게 드러나도록 정리.
    var name = 'wiacode-' + state.ctype + '-' + r.grid + (r.shape !== 'square' ? '-' + r.shape : '') + (r.color ? '-color' : '') + (bridge ? '-bridge' : '') + '.png';
    var kind = { text: '텍스트', url: 'URL', wifi: 'WiFi', vcard: '연락처', email: '이메일', phone: '전화',
      sms: 'SMS', event: '일정', geo: '위치',
      crypto: (state.cryptoCoin === 'eth' ? '이더리움 결제' : (COINS[state.cryptoCoin] || COINS.btc).name + ' 결제'),
      otp: '2단계 인증 키', whatsapp: 'WhatsApp', sepa: 'SEPA 계좌이체', todo: '할 일',
      pgpkey: 'PGP 공개키', sshpubkey: 'SSH 공개키', facetime: 'FaceTime', xmpp: 'XMPP', sip: 'SIP',
      ssh: 'SSH 접속설정', api: 'API 설정', mcp: 'MCP 부트스트랩' }[state.ctype];
    var render = function (dataURL) {
      out.innerHTML =
        '<img alt="WIA Code" src="' + dataURL + '">' +
        '<div class="meta"><b>' + kind + '</b> · ' + r.width + '×' + r.height + 'px · ' + r.grid + '/' + (r.shape) + ' · ' + (r.color ? '컬러' : (BPC_LABEL[bpc] || bpc + '단계')) + (bridge ? ' · 🔗다리QR' : '') +
        '<br>' + r.bytes + '자 담김 / 용량 약 ' + r.capBytes + '바이트</div>' +
        '<a class="dl" download="' + name + '" href="' + dataURL + '">⬇ PNG 다운로드</a>' +
        '<div class="meta" style="margin-top:12px">' +
        (bridge
          ? '기본 카메라로 중앙 QR을 비추면 <a class="scan" href="scan.html">스캐너</a>로 이동합니다. 스캐너로 코드 전체를 읽으면 이 내용이 그대로 나옵니다 (오프라인).'
          : '화면/인쇄 후 <a class="scan" href="scan.html">스캐너</a>로 읽으면 이 설정이 그대로 나옵니다 (오프라인).') +
        '</div>';
    };
    // 중앙 오버레이(다리QR 또는 사용자 로고, 상호배타) → 프레임 액자 → 최종 렌더.
    var afterCenter = function (du) { withFrame(du, state.frame, bridge, render); };
    if (bridge) withBridge(r.dataURL, afterCenter); else withLogo(r.dataURL, afterCenter);
  }

  // 쿼리스트링 프리필(2026-08-14) — 다른 WIA 서비스(예: go.wiacode.com 단축링크)에서
  // "WIA Code로 내보내기"로 딥링크할 때 씀. 지원: ?ctype=<타입>&value=<그 타입의 주 필드값>.
  // 타입별 "주 필드"만 지원(전부는 아님) — 필요해지면 여기 표에 추가.
  var PREFILL_FIELD = { url: 'f_url', text: 'f_text', phone: 'ph_num' };
  function applyPrefillFromQuery() {
    var qs = new URLSearchParams(window.location.search);
    var ctype = qs.get('ctype'), value = qs.get('value');
    if (!ctype || !PREFILL_FIELD[ctype]) return;
    var typeBtn = document.querySelector('#ctype button[data-v="' + ctype + '"]');
    if (typeBtn) typeBtn.click(); // 기존 seg() 핸들러가 state.ctype 갱신 + .fields 토글 + run() 까지 처리
    if (value != null) {
      var el = $(PREFILL_FIELD[ctype]);
      if (el) { el.value = value; }
    }
  }
  ready(function () { $('gen').addEventListener('click', run); applyPrefillFromQuery(); run(); });
})();
