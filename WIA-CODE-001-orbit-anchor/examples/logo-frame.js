'use strict';
/*
 * ============================================================================
 *  WIA Code — 로고 삽입 + 프레임 템플릿 (2026-08-14)
 * ============================================================================
 *  둘 다 순수 캔버스 합성이라 코덱 픽셀(디코딩 대상)에는 손을 안 댄다:
 *    - embedLogo: bridge-qr.js가 이미 실사용 중인 "중앙 ~20% 반경은 RS 오류정정이
 *      흡수한다"는 검증된 관례를 그대로 재사용(같은 자릿수), 다리QR과는 동시 사용 불가.
 *    - wrapFrame: WIA 코드 이미지 자체는 그대로 두고 바깥에 더 큰 캔버스로 감싸서
 *      액자+안내문구만 추가 — 디코딩 영역 밖이라 위험 0.
 * ============================================================================
 */
(function (root) {
  function embedLogo(canvas, imgEl, sizeFrac) {
    sizeFrac = Math.max(0.08, Math.min(0.20, sizeFrac || 0.15));
    var ctx = canvas.getContext('2d');
    var W = canvas.width, cx = W / 2, cy = canvas.height / 2, r = W * sizeFrac;
    ctx.save();
    ctx.beginPath(); ctx.arc(cx, cy, r * 1.15, 0, Math.PI * 2); ctx.fillStyle = '#fff'; ctx.fill();
    ctx.restore();
    ctx.save();
    ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2); ctx.clip();
    var iw = imgEl.naturalWidth || imgEl.width, ih = imgEl.naturalHeight || imgEl.height;
    var scale = Math.max((r * 2) / iw, (r * 2) / ih);
    var dw = iw * scale, dh = ih * scale;
    ctx.drawImage(imgEl, cx - dw / 2, cy - dh / 2, dw, dh);
    ctx.restore();
    return { cx: cx, cy: cy, r: r };
  }

  var FRAMES = {
    plain: { label: '스캔하세요 · SCAN ME', round: 0, dash: null },
    rounded: { label: '스캔하세요 · SCAN ME', round: 28, dash: null },
    dashed: { label: '스캔하세요 · SCAN ME', round: 14, dash: [10, 8] },
  };

  // srcCanvas(원본 코드) → 더 큰 새 캔버스에 액자+문구 얹어서 반환. frameKey 없으면 srcCanvas 그대로.
  function wrapFrame(srcCanvas, frameKey, opts) {
    var f = FRAMES[frameKey];
    if (!f) return srcCanvas;
    opts = opts || {};
    var pad = 28, labelH = 64, border = 6;
    var accent = opts.accent || '#10b981';
    var W = srcCanvas.width + pad * 2, H = srcCanvas.height + pad * 2 + labelH;
    var out = document.createElement('canvas'); out.width = W; out.height = H;
    var ctx = out.getContext('2d');
    ctx.fillStyle = '#ffffff'; ctx.fillRect(0, 0, W, H);
    // 액자 테두리
    ctx.save();
    ctx.lineWidth = border; ctx.strokeStyle = accent;
    if (f.dash) ctx.setLineDash(f.dash);
    roundRect(ctx, border / 2, border / 2, W - border, H - border, f.round);
    ctx.stroke();
    ctx.restore();
    // 코드 이미지
    ctx.drawImage(srcCanvas, pad, pad);
    // 하단 라벨
    ctx.fillStyle = accent;
    ctx.font = '700 26px -apple-system, "Apple SD Gothic Neo", system-ui, sans-serif';
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(opts.label || f.label, W / 2, pad + srcCanvas.height + labelH / 2 + pad / 2);
    return out;
  }

  function roundRect(ctx, x, y, w, h, r) {
    if (!r) { ctx.strokeRect(x, y, w, h); return; }
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
  }

  var API = { embedLogo: embedLogo, wrapFrame: wrapFrame, FRAMES: FRAMES };
  if (typeof module !== 'undefined' && module.exports) module.exports = API;
  root.WiaLogoFrame = API;
})(typeof globalThis !== 'undefined' ? globalThis : this);
