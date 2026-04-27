/**
 * WIA Screen Reader - Chrome Extension Content Script
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

// Result overlay element
let resultOverlay = null;

// Initialize content script
function init() {
  // Listen for messages from background script
  chrome.runtime.onMessage.addListener((request, sender, sendResponse) => {
    switch (request.action) {
      case 'showResult':
        showResultOverlay(request.type, request.original, request.result);
        break;
      case 'convert-wihp':
        handleConvertWIHP();
        break;
      case 'convert-braille':
        handleConvertBraille();
        break;
      case 'speak-selection':
        handleSpeak();
        break;
    }
  });

  // Add keyboard listener
  document.addEventListener('keydown', handleKeyboard);
}

// Handle keyboard shortcuts
function handleKeyboard(e) {
  if (e.altKey && e.key === 'w') {
    handleConvertWIHP();
  } else if (e.altKey && e.key === 'b') {
    handleConvertBraille();
  } else if (e.altKey && e.key === 's') {
    handleSpeak();
  }
}

// Get selected text
function getSelectedText() {
  return window.getSelection().toString().trim();
}

// Convert selection to WIHP
function handleConvertWIHP() {
  const text = getSelectedText();
  if (!text) return;

  chrome.runtime.sendMessage({ action: 'getWIHP', text }, (response) => {
    if (response && response.wihp) {
      showResultOverlay('WIHP', text, response.wihp);
    }
  });
}

// Convert selection to Braille
function handleConvertBraille() {
  const text = getSelectedText();
  if (!text) return;

  chrome.runtime.sendMessage({ action: 'getBraille', text }, (response) => {
    if (response && response.braille) {
      showResultOverlay('Braille', text, response.braille);
    }
  });
}

// Speak selection
function handleSpeak() {
  const text = getSelectedText();
  if (!text) return;

  chrome.runtime.sendMessage({ action: 'speak', text });
}

// Show result overlay
function showResultOverlay(type, original, result) {
  // Remove existing overlay
  if (resultOverlay) {
    resultOverlay.remove();
  }

  // Create overlay
  resultOverlay = document.createElement('div');
  resultOverlay.id = 'wia-result-overlay';
  resultOverlay.innerHTML = `
    <div class="wia-overlay-header">
      <span class="wia-logo">🔊 WIA Screen Reader</span>
      <button class="wia-close" aria-label="Close">&times;</button>
    </div>
    <div class="wia-overlay-content">
      <div class="wia-type">${type}</div>
      <div class="wia-original">${original}</div>
      <div class="wia-arrow">↓</div>
      <div class="wia-result">${result}</div>
    </div>
    <div class="wia-overlay-footer">
      <button class="wia-copy">Copy</button>
      <button class="wia-speak">🔊 Speak</button>
    </div>
  `;

  // Style the overlay
  resultOverlay.style.cssText = `
    position: fixed;
    top: 20px;
    right: 20px;
    width: 300px;
    background: #1e293b;
    border: 1px solid #334155;
    border-radius: 12px;
    box-shadow: 0 20px 40px rgba(0,0,0,0.4);
    z-index: 999999;
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
    color: #f8fafc;
    overflow: hidden;
  `;

  document.body.appendChild(resultOverlay);

  // Add event listeners
  resultOverlay.querySelector('.wia-close').addEventListener('click', () => {
    resultOverlay.remove();
    resultOverlay = null;
  });

  resultOverlay.querySelector('.wia-copy').addEventListener('click', () => {
    navigator.clipboard.writeText(result);
    resultOverlay.querySelector('.wia-copy').textContent = 'Copied!';
  });

  resultOverlay.querySelector('.wia-speak').addEventListener('click', () => {
    chrome.runtime.sendMessage({ action: 'speak', text: result });
  });

  // Auto-hide after 10 seconds
  setTimeout(() => {
    if (resultOverlay) {
      resultOverlay.style.opacity = '0';
      resultOverlay.style.transition = 'opacity 0.5s';
      setTimeout(() => {
        if (resultOverlay) {
          resultOverlay.remove();
          resultOverlay = null;
        }
      }, 500);
    }
  }, 10000);
}

// Initialize
init();
