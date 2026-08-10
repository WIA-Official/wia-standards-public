// scan.js — WIA Code Scanner UI (camera + overlay)
// Talks only to the window.WiaScan engine contract. No frameworks, no CDN deps.
(function () {
  'use strict';

  var els = {
    engineBadge: document.getElementById('engineBadge'),
    startScreen: document.getElementById('startScreen'),
    cameraScreen: document.getElementById('cameraScreen'),
    testScreen: document.getElementById('testScreen'),
    errorScreen: document.getElementById('errorScreen'),
    startBtn: document.getElementById('startBtn'),
    startMsg: document.getElementById('startMsg'),
    video: document.getElementById('video'),
    overlay: document.getElementById('overlay'),
    videoWrap: document.getElementById('videoWrap'),
    hudFps: document.getElementById('hudFps'),
    hudMs: document.getElementById('hudMs'),
    hint: document.getElementById('hint'),
    stopBtn: document.getElementById('stopBtn'),
    testToggleBtn: document.getElementById('testToggleBtn'),
    testImg: document.getElementById('testImg'),
    closeTestBtn: document.getElementById('closeTestBtn'),
    gridBtns: Array.prototype.slice.call(document.querySelectorAll('.grid-btn')),
    errorMsg: document.getElementById('errorMsg'),
    retryBtn: document.getElementById('retryBtn'),
    installBanner: document.getElementById('installBanner'),
    installBannerText: document.getElementById('installBannerText'),
    installBtn: document.getElementById('installBtn'),
    installDismissBtn: document.getElementById('installDismissBtn'),
    openUrlBtn: document.getElementById('openUrlBtn')
  };

  // 480 was too aggressive a downscale: locate still succeeds well below this, but per-cell
  // decode needs materially more effective resolution than locate does. Real-world capture
  // (heart/round shapes at a normal scanning distance)
  // was landing right below the decode floor — lock found, payload never read. Verified with
  // degrade.js resample sweeps: effective cellPx~2.2px locks but never decodes; ~2.8px decodes
  // reliably. 720 buys back that margin for a typical "doesn't quite fill the frame" photo.
  var PROC_MAX_W = 720;          // processed frame width cap (px)
  var DETECT_INTERVAL_MS = 1000 / 8; // throttle detection to ~8fps

  var stream = null;
  var procCanvas = null;
  var procCtx = null;
  var overlayCtx = null;
  var rafId = null;
  var testMode = false;
  var currentGrid = 'S';
  var engineIsReal = false;
  var firstDecodeDone = false;
  var deferredInstallPrompt = null;
  var pendingOpenUrl = null;

  // ---------------------------------------------------------------------
  // Screen helpers
  // ---------------------------------------------------------------------
  function showScreen(name) {
    ['startScreen', 'cameraScreen', 'testScreen', 'errorScreen'].forEach(function (id) {
      els[id].classList.toggle('hidden', id !== name);
    });
  }

  function showError(msg) {
    stopLoop();
    els.errorMsg.textContent = msg;
    showScreen('errorScreen');
  }

  // ---------------------------------------------------------------------
  // Engine loading: prefer real wiascan-core.js (loaded via <script> tag
  // in scan.html); if window.WiaScan isn't defined shortly after, fall
  // back to the local dev stub so the UI still works end to end.
  // ---------------------------------------------------------------------
  function loadScript(src) {
    return new Promise(function (resolve, reject) {
      var s = document.createElement('script');
      s.src = src;
      s.onload = function () { resolve(); };
      s.onerror = function () { reject(new Error('load fail: ' + src)); };
      document.body.appendChild(s);
    });
  }

  function updateEngineBadge() {
    els.engineBadge.textContent = engineIsReal ? 'ENGINE: real' : 'ENGINE: stub';
    els.engineBadge.classList.toggle('badge-real', engineIsReal);
    els.engineBadge.classList.toggle('badge-stub', !engineIsReal);
  }

  function ensureEngine() {
    return Promise.resolve()
      .then(function () {
        if (window.WiaScan) return;
        // give wiascan-core.js a brief window in case it defines the
        // global asynchronously (e.g. via a module or deferred init).
        return new Promise(function (r) { setTimeout(r, 150); });
      })
      .then(function () {
        if (window.WiaScan) {
          engineIsReal = true;
          return;
        }
        engineIsReal = false;
        return loadScript('wiascan-stub.js').catch(function (e) {
          console.error('스텁 엔진 로드 실패', e);
        });
      })
      .then(function () {
        updateEngineBadge();
        if (window.WiaScan && window.WiaScan.ready) {
          els.startMsg.textContent = '엔진 로딩 중…';
          return Promise.resolve(window.WiaScan.ready).catch(function (e) {
            console.error('엔진 초기화 오류', e);
          });
        }
      })
      .then(function () {
        els.startMsg.textContent = '';
      });
  }

  // ---------------------------------------------------------------------
  // Camera lifecycle
  // ---------------------------------------------------------------------
  function startCamera() {
    els.startMsg.textContent = '';

    if (!window.isSecureContext) {
      showError('보안 연결(HTTPS)이 필요합니다. https 주소로 다시 접속해주세요.');
      return;
    }
    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
      showError('이 브라우저는 카메라 기능을 지원하지 않습니다.');
      return;
    }

    navigator.mediaDevices.getUserMedia({
      video: { facingMode: 'environment' },
      audio: false
    }).then(function (s) {
      stream = s;
      els.video.srcObject = stream;
      return new Promise(function (resolve) {
        if (els.video.readyState >= 1) { resolve(); return; }
        els.video.onloadedmetadata = function () { resolve(); };
      });
    }).then(function () {
      return els.video.play().catch(function () { /* muted+playsinline should allow autoplay */ });
    }).then(function () {
      // Show the camera screen BEFORE measuring layout for the overlay
      // canvas — while #cameraScreen is still display:none (the .hidden
      // class), videoWrap.getBoundingClientRect() reports 0x0 and the
      // overlay ends up permanently sized to nothing.
      showScreen('cameraScreen');
      setupProcessingCanvas();
      setupOverlayCanvas();
      startLoop();
    }).catch(function (err) {
      var name = err && err.name;
      if (name === 'NotAllowedError' || name === 'PermissionDeniedError') {
        showError('카메라 접근 권한이 필요합니다. 브라우저 설정에서 카메라 권한을 허용해주세요.');
      } else if (name === 'NotFoundError' || name === 'DevicesNotFoundError') {
        showError('사용 가능한 카메라를 찾을 수 없습니다.');
      } else if (name === 'NotReadableError') {
        showError('카메라를 다른 앱이 사용 중입니다. 다른 앱을 종료한 뒤 다시 시도해주세요.');
      } else {
        showError('카메라를 시작할 수 없습니다: ' + (err && err.message ? err.message : '알 수 없는 오류'));
      }
    });
  }

  function stopCamera() {
    stopLoop();
    pendingOpenUrl = null;
    els.openUrlBtn.classList.add('hidden');
    if (stream) {
      stream.getTracks().forEach(function (t) { t.stop(); });
      stream = null;
    }
    els.video.srcObject = null;
    testMode = false;
    showScreen('startScreen');
  }

  // ---------------------------------------------------------------------
  // Canvas setup
  // ---------------------------------------------------------------------
  function setupProcessingCanvas() {
    var vw = els.video.videoWidth || 640;
    var vh = els.video.videoHeight || 480;
    var scale = Math.min(1, PROC_MAX_W / vw);
    var pw = Math.max(1, Math.round(vw * scale));
    var ph = Math.max(1, Math.round(vh * scale));
    procCanvas = document.createElement('canvas');
    procCanvas.width = pw;
    procCanvas.height = ph;
    procCtx = procCanvas.getContext('2d', { willReadFrequently: true });
  }

  function setupOverlayCanvas() {
    overlayCtx = els.overlay.getContext('2d');
    resizeOverlay();
    window.addEventListener('resize', resizeOverlay);
    window.addEventListener('orientationchange', resizeOverlay);
  }

  function resizeOverlay() {
    if (!overlayCtx) return;
    var rect = els.videoWrap.getBoundingClientRect();
    var dpr = window.devicePixelRatio || 1;
    els.overlay.width = Math.max(1, Math.round(rect.width * dpr));
    els.overlay.height = Math.max(1, Math.round(rect.height * dpr));
    els.overlay.style.width = rect.width + 'px';
    els.overlay.style.height = rect.height + 'px';
    overlayCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  // Maps a point from the (downscaled) processing-canvas pixel space to
  // on-screen CSS pixel space, accounting for the video's object-fit:cover
  // scaling inside its container.
  function computeVideoDisplayRect() {
    var containerW = els.videoWrap.clientWidth;
    var containerH = els.videoWrap.clientHeight;
    var vw = els.video.videoWidth || containerW;
    var vh = els.video.videoHeight || containerH;
    var scale = Math.max(containerW / vw, containerH / vh); // cover
    var dispW = vw * scale;
    var dispH = vh * scale;
    return {
      scale: scale,
      offsetX: (containerW - dispW) / 2,
      offsetY: (containerH - dispH) / 2,
      vw: vw,
      vh: vh
    };
  }

  function mapPoint(px, py, rect) {
    var procScale = procCanvas.width / rect.vw;
    var nativeX = px / procScale;
    var nativeY = py / procScale;
    return {
      x: rect.offsetX + nativeX * rect.scale,
      y: rect.offsetY + nativeY * rect.scale
    };
  }

  // ---------------------------------------------------------------------
  // Main loop — throttled to ~8fps for detection; overlay redraw follows.
  // ---------------------------------------------------------------------
  function startLoop() {
    var lastDetectTime = 0;
    var frames = 0;
    var fpsWindowStart = performance.now();
    var noDecodeStreak = 0;         // 코드는 보이는데 해독 안 되는 연속 프레임(블러 의심)
    var noLockStreak = 0;           // 아예 못 잡는 연속 프레임(극단블러/배율추정 실패 의심)

    function tick(now) {
      rafId = requestAnimationFrame(tick);
      if (testMode) return;
      if (now - lastDetectTime < DETECT_INTERVAL_MS) return;
      lastDetectTime = now;

      procCtx.drawImage(els.video, 0, 0, procCanvas.width, procCanvas.height);

      var imageData;
      try {
        imageData = procCtx.getImageData(0, 0, procCanvas.width, procCanvas.height);
      } catch (e) {
        console.error('getImageData 실패', e);
        return;
      }

      var t0 = performance.now();
      var result;
      try {
        // 매 프레임은 빠른 표준 판독(PSF·deep 폴백 둘 다 끔) — 프리뷰 안 얼림.
        result = window.WiaScan.detectAuto(imageData, { psf: false, deep: false });
        // 코드는 보이는데(locate O) 해독 안 됨 = 블러 의심 → 4프레임에 1번만 무거운 PSF-ISI 1회.
        if (result && result.ok && !result.decoded) {
          noDecodeStreak++; noLockStreak = 0;
          if (noDecodeStreak % 4 === 0) {
            var pr = window.WiaScan.detectAuto(imageData, { psf: true, deep: false });
            if (pr && pr.decoded) result = pr;
          }
        } else if (!result || !result.ok) {
          // 아예 못 잡음 = 극단 디포커스로 코어 링에지가 죽어 배율추정 실패 가능 →
          //   4프레임에 1번만 deep(각도평균 프로파일 배율추정) 재시도. 실패프레임 비용 ≤4배라 스로틀.
          noDecodeStreak = 0; noLockStreak++;
          if (noLockStreak % 4 === 0) {
            var dr = window.WiaScan.detectAuto(imageData, { psf: false, deep: true });
            if (dr && dr.ok) result = dr;
          }
        } else {
          noDecodeStreak = 0; noLockStreak = 0;
        }
      } catch (e) {
        console.error('detectAuto 오류', e);
        result = { ok: false, reason: 'engine-error', ms: 0 };
      }
      var wallMs = performance.now() - t0;

      renderOverlay(result);
      updateHud(result, wallMs);
      updateUrlButton(result);
      updateProximityHint(result, noDecodeStreak);

      frames++;
      if (now - fpsWindowStart >= 1000) {
        els.hudFps.textContent = 'FPS ' + frames;
        frames = 0;
        fpsWindowStart = now;
      }
    }
    rafId = requestAnimationFrame(tick);
  }

  function stopLoop() {
    if (rafId) cancelAnimationFrame(rafId);
    rafId = null;
  }

  function updateHud(result, wallMs) {
    var ms = (result && typeof result.ms === 'number') ? result.ms : wallMs;
    els.hudMs.textContent = ms.toFixed(1) + ' ms';
  }

  // 잡히긴 했는데(locate ok) 계속 해독이 안 되는 경우 — 대개 원인은 "화면을 덜 채워서" 실효
  // 해상도가 낮은 것. 원인 모른 채 계속 안 되는 채로 두지 않고 바로 알려준다.
  var DEFAULT_HINT_TEXT = els.hint.textContent;
  var CLOSER_HINT_TEXT = '해독이 안 돼요 — 코드에 더 가까이, 화면을 꽉 채워보세요';
  var CLOSER_STREAK_THRESHOLD = 10; // ~1.2s @ 8fps 목표 — 너무 빨리 뜨면 정상 프레임에도 깜빡임

  function updateProximityHint(result, streak) {
    if (result && result.ok && !result.decoded && streak >= CLOSER_STREAK_THRESHOLD) {
      els.hint.textContent = CLOSER_HINT_TEXT;
      els.hint.classList.remove('hidden');
    } else if (result && result.ok) {
      els.hint.textContent = DEFAULT_HINT_TEXT; // 해독 성공 — renderOverlay가 이미 숨김
    } else {
      els.hint.textContent = DEFAULT_HINT_TEXT; // 락 자체가 없음 — renderOverlay 기본 문구로 복귀
    }
  }

  // 해독된 payload가 http(s) URL이면 자동으로 그 페이지로 이동한다. 새 탭이 아니라 "현재 탭
  // 자체를 이동"시키는 게 핵심 — 이동이 시작되는 순간 이 페이지의 카메라 루프도 같이 죽으므로,
  // 매 프레임 반복 판정에도 실제 네비게이션은 정확히 한 번만 실행된다(navigatedAway 가드는
  // 이동 시작~언로드 사이의 짧은 틈에 대비한 이중 안전장치). 새 탭 방식(window.open)은 이
  // 탭이 안 죽어 스캔이 계속 돌아 여러 탭이 반복해서 열릴 위험이 있어 폐기.
  var navigatedAway = false;
  function updateUrlButton(result) {
    var isUrl = !!(result && result.decoded && result.text && /^https?:\/\//i.test(result.text));
    if (!isUrl || navigatedAway) return;
    pendingOpenUrl = result.text;
    els.openUrlBtn.textContent = '🔗 이동 중… ' + pendingOpenUrl;
    els.openUrlBtn.classList.remove('hidden'); // 이동 직전 짧은 순간 목적지를 보여줌(깜빡 이동 방지)
    navigatedAway = true;
    setTimeout(function () { location.href = pendingOpenUrl; }, 400);
  }

  els.openUrlBtn.addEventListener('click', function () {
    if (pendingOpenUrl) location.href = pendingOpenUrl; // 지연 400ms 기다리지 않고 바로 이동
  });

  function renderOverlay(result) {
    var dpr = window.devicePixelRatio || 1;
    var w = els.overlay.width / dpr;
    var h = els.overlay.height / dpr;
    overlayCtx.clearRect(0, 0, w, h);

    if (!result || !result.ok) {
      els.hint.classList.remove('hidden');
      return;
    }
    els.hint.classList.add('hidden');

    var rect = computeVideoDisplayRect();
    var TL = mapPoint(result.corners.TL.x, result.corners.TL.y, rect);
    var TR = mapPoint(result.corners.TR.x, result.corners.TR.y, rect);
    var BR = mapPoint(result.corners.BR.x, result.corners.BR.y, rect);
    var BL = mapPoint(result.corners.BL.x, result.corners.BL.y, rect);
    var core = mapPoint(result.core.x, result.core.y, rect);

    // quad fill + stroke
    overlayCtx.beginPath();
    overlayCtx.moveTo(TL.x, TL.y);
    overlayCtx.lineTo(TR.x, TR.y);
    overlayCtx.lineTo(BR.x, BR.y);
    overlayCtx.lineTo(BL.x, BL.y);
    overlayCtx.closePath();
    overlayCtx.fillStyle = 'rgba(0, 230, 160, 0.18)';
    overlayCtx.fill();
    overlayCtx.lineWidth = 2.5;
    overlayCtx.strokeStyle = '#00e6a0';
    overlayCtx.stroke();

    // corner dots (northStar gets a ring)
    var corners = { TL: TL, TR: TR, BR: BR, BL: BL };
    Object.keys(corners).forEach(function (key) {
      var p = corners[key];
      overlayCtx.beginPath();
      overlayCtx.arc(p.x, p.y, 5, 0, Math.PI * 2);
      overlayCtx.fillStyle = '#00e6a0';
      overlayCtx.fill();
      if (key === result.northStar) {
        overlayCtx.beginPath();
        overlayCtx.arc(p.x, p.y, 11, 0, Math.PI * 2);
        overlayCtx.lineWidth = 2;
        overlayCtx.strokeStyle = '#ffd400';
        overlayCtx.stroke();
      }
    });

    // core dot
    overlayCtx.beginPath();
    overlayCtx.arc(core.x, core.y, 4, 0, Math.PI * 2);
    overlayCtx.fillStyle = '#ff5f5f';
    overlayCtx.fill();

    // label: method / grid / cellPx / residPx
    var label = (result.method || '?') + ' · ' + (result.grid || '?') +
      ' · cell=' + (typeof result.cellPx === 'number' ? result.cellPx.toFixed(1) : '?') +
      ' · resid=' + (typeof result.residPx === 'number' ? result.residPx.toFixed(2) : '?');
    overlayCtx.font = '12px -apple-system, system-ui, sans-serif';
    var metrics = overlayCtx.measureText(label);
    var padX = 6, padY = 4;
    var boxW = metrics.width + padX * 2;
    var boxH = 12 + padY * 2;
    var lx = TL.x;
    var ly = TL.y - boxH - 6;
    if (ly < 4) ly = TL.y + 6;
    lx = Math.max(4, Math.min(w - boxW - 4, lx));
    overlayCtx.fillStyle = 'rgba(0,0,0,0.6)';
    overlayCtx.fillRect(lx, ly, boxW, boxH);
    overlayCtx.fillStyle = '#eafff5';
    overlayCtx.fillText(label, lx + padX, ly + padY + 10);

    // 해독된 내용(payload) — 잡기 너머 "읽기". CRC 검증 통과분만 큰 초록 배너로.
    if (result.decoded && result.text) {
      if (!firstDecodeDone) {
        firstDecodeDone = true;
        maybeShowInstallBanner();
      }
      var txt = '📖 ' + result.text;
      overlayCtx.font = 'bold 16px -apple-system, system-ui, sans-serif';
      var maxW = w - 24;
      // 폭 초과 시 말줄임
      while (overlayCtx.measureText(txt).width > maxW && txt.length > 4) txt = txt.slice(0, -2);
      if (txt !== ('📖 ' + result.text)) txt += '…';
      var tw = overlayCtx.measureText(txt).width;
      var bw = tw + 20, bh = 30;
      var bx = Math.max(8, (w - bw) / 2);
      var by = Math.min(h - bh - 12, BL.y + 12);
      if (by < ly + boxH + 8) by = ly + boxH + 8;
      overlayCtx.fillStyle = 'rgba(16,185,129,0.92)';
      roundRect(overlayCtx, bx, by, bw, bh, 8); overlayCtx.fill();
      overlayCtx.fillStyle = '#04140d';
      overlayCtx.fillText(txt, bx + 10, by + 20);

      // 컬러(hue) 레이어 payload — 있으면 그 아래 보라색 배너로.
      if (result.hue) {
        var htxt = '🎨 ' + result.hue;
        while (overlayCtx.measureText(htxt).width > maxW && htxt.length > 4) htxt = htxt.slice(0, -2);
        if (htxt !== ('🎨 ' + result.hue)) htxt += '…';
        var hbw = overlayCtx.measureText(htxt).width + 20, hby = Math.min(h - bh - 12, by + bh + 6);
        var hbx = Math.max(8, (w - hbw) / 2);
        overlayCtx.fillStyle = 'rgba(139,92,246,0.92)';
        roundRect(overlayCtx, hbx, hby, hbw, bh, 8); overlayCtx.fill();
        overlayCtx.fillStyle = '#fff';
        overlayCtx.fillText(htxt, hbx + 10, hby + 20);
      }
    }
  }
  function roundRect(ctx, x, y, ww, hh, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y); ctx.arcTo(x + ww, y, x + ww, y + hh, r);
    ctx.arcTo(x + ww, y + hh, x, y + hh, r); ctx.arcTo(x, y + hh, x, y, r);
    ctx.arcTo(x, y, x + ww, y, r); ctx.closePath();
  }

  // ---------------------------------------------------------------------
  // PWA install (scan-only — never touches wiacode.com root PWA)
  // ---------------------------------------------------------------------
  var INSTALL_DISMISS_KEY = 'wiaScanInstallDismissed';

  function registerServiceWorker() {
    if (!window.isSecureContext || !('serviceWorker' in navigator)) return;
    navigator.serviceWorker.register('scan-sw.js', { scope: './' }).catch(function () {
      /* offline install unavailable — camera scanning still works fully online */
    });
  }

  function isStandaloneDisplay() {
    return (window.matchMedia && window.matchMedia('(display-mode: standalone)').matches) ||
      window.navigator.standalone === true;
  }

  window.addEventListener('beforeinstallprompt', function (e) {
    e.preventDefault();
    deferredInstallPrompt = e;
  });

  window.addEventListener('appinstalled', function () {
    deferredInstallPrompt = null;
    els.installBanner.classList.add('hidden');
  });

  // 첫 성공적 해독 이후에만 뜬다 — 처음 온 사람에게 다짜고짜 설치부터 들이밀지 않는다.
  function maybeShowInstallBanner() {
    if (isStandaloneDisplay()) return;
    if (localStorage.getItem(INSTALL_DISMISS_KEY) === '1') return;
    var isIOS = /iphone|ipad|ipod/i.test(navigator.userAgent) && !window.MSStream;
    if (deferredInstallPrompt) {
      els.installBannerText.textContent = '홈 화면에 추가하면 브라우저 없이 바로 카메라로 열려요.';
      els.installBtn.classList.remove('hidden');
    } else if (isIOS) {
      els.installBannerText.textContent = 'Safari 공유 버튼 → "홈 화면에 추가"를 누르면 설치돼요.';
      els.installBtn.classList.add('hidden');
    } else {
      return; // Android/기타 브라우저가 아직 설치 가능 신호를 안 줬으면 조용히 넘어간다
    }
    els.installBanner.classList.remove('hidden');
  }

  els.installBtn.addEventListener('click', function () {
    els.installBanner.classList.add('hidden');
    if (!deferredInstallPrompt) return;
    deferredInstallPrompt.prompt();
    deferredInstallPrompt.userChoice.finally(function () { deferredInstallPrompt = null; });
  });

  els.installDismissBtn.addEventListener('click', function () {
    els.installBanner.classList.add('hidden');
    localStorage.setItem(INSTALL_DISMISS_KEY, '1');
  });

  // ---------------------------------------------------------------------
  // Test-code mode
  // ---------------------------------------------------------------------
  function renderTestCode(grid) {
    if (!window.WiaScan || typeof window.WiaScan.renderTestCode !== 'function') return;
    try {
      var res = window.WiaScan.renderTestCode(grid, 6);
      if (res && typeof res.dataURL === 'string' && res.dataURL.indexOf('data:') === 0) {
        els.testImg.src = res.dataURL;
        els.testImg.alt = '테스트 코드';
      } else {
        // Engine returned no usable image (e.g. dataURL missing/null) —
        // don't feed a bad value into <img src>, show a clear placeholder.
        els.testImg.removeAttribute('src');
        els.testImg.alt = '테스트 코드를 생성하지 못했습니다 (엔진 응답에 dataURL 없음)';
      }
    } catch (e) {
      console.error('테스트 코드 렌더 실패', e);
      els.testImg.removeAttribute('src');
      els.testImg.alt = '테스트 코드 렌더 오류';
    }
  }

  els.testToggleBtn.addEventListener('click', function () {
    testMode = true;
    renderTestCode(currentGrid);
    showScreen('testScreen');
  });

  els.closeTestBtn.addEventListener('click', function () {
    testMode = false;
    showScreen('cameraScreen');
  });

  els.gridBtns.forEach(function (btn) {
    btn.addEventListener('click', function () {
      currentGrid = btn.getAttribute('data-grid');
      els.gridBtns.forEach(function (b) { b.classList.toggle('active', b === btn); });
      renderTestCode(currentGrid);
    });
  });

  // ---------------------------------------------------------------------
  // Wiring
  // ---------------------------------------------------------------------
  els.startBtn.addEventListener('click', startCamera);
  els.retryBtn.addEventListener('click', startCamera);
  els.stopBtn.addEventListener('click', stopCamera);

  // 재방문 자동 시작: 카메라 권한이 이미 'granted'면(= 전에 스캔해본 사람) "카메라 시작"
  // 탭을 건너뛰고 곧바로 카메라를 켠다. 처음 온 사람은 'prompt'라 버튼이 그대로 뜬다.
  // (iOS Safari는 Permissions API 카메라 미지원 → 안전하게 버튼 유지)
  function maybeAutoStart() {
    if (!window.isSecureContext) return;
    if (!(navigator.permissions && navigator.permissions.query)) return;
    navigator.permissions.query({ name: 'camera' }).then(function (st) {
      if (st.state === 'granted') startCamera();
    }).catch(function () { /* 미지원 → 버튼 유지 */ });
  }

  document.addEventListener('DOMContentLoaded', function () {
    els.startBtn.disabled = true;
    registerServiceWorker();
    ensureEngine().then(function () {
      els.startBtn.disabled = false;
      maybeAutoStart();
    });
  });
})();
