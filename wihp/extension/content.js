/**
 * WIHP Chrome Extension - Content Script
 * Injects WIHP functionality into web pages
 */

// Load WIHP Engine
const script = document.createElement('script');
script.src = chrome.runtime.getURL('wihp-engine.js');
document.head.appendChild(script);

// Create tooltip element
const tooltip = document.createElement('div');
tooltip.id = 'wihp-tooltip';
tooltip.style.cssText = `
  position: fixed;
  background: linear-gradient(135deg, #1e40af 0%, #059669 100%);
  color: white;
  padding: 12px 16px;
  border-radius: 12px;
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
  font-size: 14px;
  z-index: 999999;
  display: none;
  box-shadow: 0 4px 20px rgba(0,0,0,0.3);
  max-width: 300px;
`;
document.body.appendChild(tooltip);

// Handle text selection
document.addEventListener('mouseup', (e) => {
  const selection = window.getSelection().toString().trim();

  if (selection && selection.length > 0 && selection.length < 100) {
    // Check if it's not Korean already
    if (!/[가-힣]/.test(selection)) {
      showTooltip(selection, e.pageX, e.pageY);
    }
  } else {
    tooltip.style.display = 'none';
  }
});

// Hide tooltip on scroll or click elsewhere
document.addEventListener('scroll', () => {
  tooltip.style.display = 'none';
});

document.addEventListener('mousedown', (e) => {
  if (e.target !== tooltip && !tooltip.contains(e.target)) {
    tooltip.style.display = 'none';
  }
});

// Show tooltip with conversion
function showTooltip(text, x, y) {
  // Wait for engine to load
  if (typeof WIHPEngine === 'undefined') {
    setTimeout(() => showTooltip(text, x, y), 100);
    return;
  }

  const hangul = WIHPEngine.convert(text);

  tooltip.innerHTML = `
    <div style="font-size: 11px; opacity: 0.8; margin-bottom: 4px;">WIHP</div>
    <div style="font-size: 24px; font-weight: bold; margin-bottom: 8px;">${hangul}</div>
    <div style="font-size: 12px; opacity: 0.7;">"${text}"</div>
    <button id="wihp-copy" style="
      margin-top: 8px;
      padding: 6px 12px;
      background: #fbbf24;
      border: none;
      border-radius: 6px;
      font-size: 12px;
      cursor: pointer;
      color: #1f2937;
      font-weight: bold;
    ">📋 Copy</button>
  `;

  // Position tooltip
  const viewportWidth = window.innerWidth;
  const viewportHeight = window.innerHeight;

  let posX = x + 10;
  let posY = y + 10;

  // Adjust if too close to edges
  if (posX + 300 > viewportWidth) posX = viewportWidth - 310;
  if (posY + 150 > viewportHeight) posY = viewportHeight - 160;

  tooltip.style.left = posX + 'px';
  tooltip.style.top = posY + 'px';
  tooltip.style.display = 'block';

  // Copy button handler
  document.getElementById('wihp-copy').onclick = () => {
    navigator.clipboard.writeText(hangul).then(() => {
      document.getElementById('wihp-copy').textContent = '✓ Copied!';
      setTimeout(() => {
        tooltip.style.display = 'none';
      }, 1000);
    });
  };
}

// Listen for messages from background
chrome.runtime.onMessage.addListener((request, sender, sendResponse) => {
  if (request.action === 'convert' && request.text) {
    const selection = window.getSelection();
    if (selection.rangeCount > 0) {
      const range = selection.getRangeAt(0);
      const rect = range.getBoundingClientRect();
      showTooltip(request.text, rect.left + window.scrollX, rect.bottom + window.scrollY);
    }
  }
});

console.log('WIHP Content Script loaded');
