/* WIA 생태계 공유 헤더(경량). #kit 에 렌더. wiamotion assets/site-kit.js 의 축소 이식 — 추후 통합. */
(function () {
  'use strict';
  var LINKS = [
    { href: 'https://wiacode.com', label: 'WIA Code', brand: true },
    { href: 'scan.html', label: '스캐너', match: 'scan' },
    { href: 'generate.html', label: '생성기', match: 'generate' },
    { href: 'https://app.wiasoom.com/studio', label: 'SOOM Studio' },
    { href: 'https://wiamotion.ai/studio.html', label: 'Motion Studio' },
    { href: 'https://wiamusic.smilestory.ai/', label: 'Music' },
  ];
  var kit = document.getElementById('kit');
  if (!kit) return;
  var here = (location.pathname.split('/').pop() || '').replace('.html', '');
  kit.innerHTML = LINKS.map(function (l) {
    var cls = (l.brand ? 'brand' : '') + (l.match && here.indexOf(l.match) >= 0 ? ' here' : '');
    return '<a href="' + l.href + '"' + (cls ? ' class="' + cls.trim() + '"' : '') + '>' + l.label + '</a>';
  }).join('');
})();
