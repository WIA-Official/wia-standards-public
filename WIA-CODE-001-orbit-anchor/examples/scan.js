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
    retryBtn: document.getElementById('retryBtn')
  };

  var PROC_MAX_W = 480;          // processed frame width cap (px)
  var DETECT_INTERVAL_MS = 1000 / 8; // throttle detection to ~8fps

  var stream = null;
  var procCanvas = null;
  var procCtx = null;
  var overlayCtx = null;
  var rafId = null;
  var testMode = false;
  var currentGrid = 'S';
  var engineIsReal = false;

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
        result = window.WiaScan.detectAuto(imageData);
      } catch (e) {
        console.error('detectAuto 오류', e);
        result = { ok: false, reason: 'engine-error', ms: 0 };
      }
      var wallMs = performance.now() - t0;

      renderOverlay(result);
      updateHud(result, wallMs);

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

  document.addEventListener('DOMContentLoaded', function () {
    els.startBtn.disabled = true;
    ensureEngine().then(function () {
      els.startBtn.disabled = false;
    });
  });
})();
