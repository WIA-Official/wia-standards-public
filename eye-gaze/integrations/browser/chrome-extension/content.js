/**
 * WIA Eye Gaze Browser Extension - Content Script
 *
 * Handles gaze visualization and interaction on web pages.
 *
 * 弘益人間 - 널리 인간을 이롭게
 */

// State
let settings = {
  dwellTime: 800,
  showCursor: true,
  cursorSize: 40,
  cursorColor: '#00ff00',
  dwellEnabled: true,
  scrollEnabled: true,
};

let gazeCursor = null;
let dwellProgress = null;
let currentGaze = { x: 0, y: 0 };
let dwellTarget = null;
let dwellStartTime = 0;
let dwellTimer = null;
let lastHoveredElement = null;

// Initialize
function init() {
  createGazeCursor();
  loadSettings();
  setupMessageListener();
  setupARIATargets();
}

// Create gaze cursor overlay
function createGazeCursor() {
  // Remove existing cursor if any
  const existing = document.getElementById('wia-gaze-cursor');
  if (existing) existing.remove();

  // Create cursor container
  gazeCursor = document.createElement('div');
  gazeCursor.id = 'wia-gaze-cursor';
  gazeCursor.className = 'wia-gaze-cursor';
  gazeCursor.style.cssText = `
    position: fixed;
    pointer-events: none;
    z-index: 2147483647;
    width: ${settings.cursorSize}px;
    height: ${settings.cursorSize}px;
    border-radius: 50%;
    border: 3px solid ${settings.cursorColor};
    background: ${settings.cursorColor}33;
    transform: translate(-50%, -50%);
    transition: width 0.1s, height 0.1s;
    display: ${settings.showCursor ? 'block' : 'none'};
  `;

  // Create dwell progress indicator
  dwellProgress = document.createElement('div');
  dwellProgress.className = 'wia-dwell-progress';
  dwellProgress.style.cssText = `
    position: absolute;
    top: -4px;
    left: -4px;
    right: -4px;
    bottom: -4px;
    border-radius: 50%;
    border: 4px solid transparent;
    border-top-color: ${settings.cursorColor};
    transform: rotate(0deg);
  `;
  gazeCursor.appendChild(dwellProgress);

  document.body.appendChild(gazeCursor);
}

// Load settings from storage
function loadSettings() {
  chrome.runtime.sendMessage({ type: 'getSettings' }, (response) => {
    if (response && response.settings) {
      settings = { ...settings, ...response.settings };
      updateCursorStyle();
    }
  });
}

// Setup message listener
function setupMessageListener() {
  chrome.runtime.onMessage.addListener((message, sender, sendResponse) => {
    switch (message.type) {
      case 'gazePoint':
        handleGazePoint(message.point);
        break;

      case 'gazeEvent':
        handleGazeEvent(message.event);
        break;

      case 'settingsUpdated':
        settings = { ...settings, ...message.settings };
        updateCursorStyle();
        break;
    }
  });
}

// Handle incoming gaze point
function handleGazePoint(point) {
  if (!point.valid) return;

  // Convert normalized coordinates to screen coordinates
  const x = point.x * window.innerWidth;
  const y = point.y * window.innerHeight;

  currentGaze = { x, y };

  // Update cursor position
  if (gazeCursor && settings.showCursor) {
    gazeCursor.style.left = `${x}px`;
    gazeCursor.style.top = `${y}px`;
    gazeCursor.style.display = 'block';
  }

  // Find element under gaze
  const element = document.elementFromPoint(x, y);

  // Handle hover state
  if (element !== lastHoveredElement) {
    // Remove highlight from previous element
    if (lastHoveredElement) {
      lastHoveredElement.classList.remove('wia-gaze-hover');
    }

    // Add highlight to new element
    if (element && isInteractiveElement(element)) {
      element.classList.add('wia-gaze-hover');
      announceElement(element);
    }

    lastHoveredElement = element;
  }

  // Handle dwell selection
  if (settings.dwellEnabled && element && isInteractiveElement(element)) {
    handleDwell(element, x, y);
  }

  // Handle scroll zones
  if (settings.scrollEnabled) {
    handleScrollZone(x, y);
  }
}

// Handle dwell selection
function handleDwell(element, x, y) {
  if (element === dwellTarget) {
    // Continue dwelling on same target
    const elapsed = Date.now() - dwellStartTime;
    const progress = Math.min(elapsed / settings.dwellTime, 1);

    // Update progress indicator
    if (dwellProgress) {
      dwellProgress.style.transform = `rotate(${progress * 360}deg)`;
      dwellProgress.style.borderTopColor = progress >= 1 ? '#ff0' : settings.cursorColor;
    }

    // Dwell complete - trigger click
    if (progress >= 1) {
      triggerClick(element);
      resetDwell();
    }
  } else {
    // New target - start dwell
    dwellTarget = element;
    dwellStartTime = Date.now();

    if (dwellProgress) {
      dwellProgress.style.transform = 'rotate(0deg)';
      dwellProgress.style.borderTopColor = settings.cursorColor;
    }
  }
}

// Reset dwell state
function resetDwell() {
  dwellTarget = null;
  dwellStartTime = 0;
  if (dwellProgress) {
    dwellProgress.style.transform = 'rotate(0deg)';
  }
}

// Trigger click on element
function triggerClick(element) {
  // Visual feedback
  element.classList.add('wia-gaze-clicked');
  setTimeout(() => element.classList.remove('wia-gaze-clicked'), 200);

  // Create and dispatch click event
  const clickEvent = new MouseEvent('click', {
    bubbles: true,
    cancelable: true,
    view: window,
    clientX: currentGaze.x,
    clientY: currentGaze.y,
  });

  element.dispatchEvent(clickEvent);

  // For links, we might need to trigger navigation
  if (element.tagName === 'A' && element.href) {
    // Let the click event handle it
  }

  // For inputs, focus them
  if (element.tagName === 'INPUT' || element.tagName === 'TEXTAREA') {
    element.focus();
  }
}

// Handle scroll zones at edges
function handleScrollZone(x, y) {
  const scrollMargin = 50;
  const scrollSpeed = 10;

  // Top edge - scroll up
  if (y < scrollMargin) {
    window.scrollBy(0, -scrollSpeed);
  }
  // Bottom edge - scroll down
  else if (y > window.innerHeight - scrollMargin) {
    window.scrollBy(0, scrollSpeed);
  }

  // Left edge - scroll left
  if (x < scrollMargin) {
    window.scrollBy(-scrollSpeed, 0);
  }
  // Right edge - scroll right
  else if (x > window.innerWidth - scrollMargin) {
    window.scrollBy(scrollSpeed, 0);
  }
}

// Handle gaze events
function handleGazeEvent(event) {
  switch (event.eventType) {
    case 'DWELL_COMPLETE':
      // Already handled in handleDwell
      break;

    case 'BLINK':
      // Could trigger action on blink
      break;
  }
}

// Check if element is interactive
function isInteractiveElement(element) {
  if (!element) return false;

  const tagName = element.tagName.toUpperCase();
  const interactiveTags = ['A', 'BUTTON', 'INPUT', 'SELECT', 'TEXTAREA', 'LABEL'];

  if (interactiveTags.includes(tagName)) return true;

  // Check role attribute
  const role = element.getAttribute('role');
  const interactiveRoles = ['button', 'link', 'checkbox', 'radio', 'tab', 'menuitem', 'option'];
  if (role && interactiveRoles.includes(role)) return true;

  // Check tabindex
  if (element.hasAttribute('tabindex') && element.tabIndex >= 0) return true;

  // Check onclick handler
  if (element.onclick || element.hasAttribute('onclick')) return true;

  // Check for click event listeners (limited detection)
  if (element.classList.contains('clickable') || element.classList.contains('btn')) return true;

  return false;
}

// Setup ARIA targets for accessibility
function setupARIATargets() {
  // Find all focusable/interactive elements
  const targets = document.querySelectorAll(`
    a[href],
    button,
    input:not([type="hidden"]),
    select,
    textarea,
    [role="button"],
    [role="link"],
    [role="checkbox"],
    [role="radio"],
    [role="tab"],
    [tabindex]:not([tabindex="-1"])
  `);

  return Array.from(targets).map(el => ({
    element: el,
    label: getElementLabel(el),
    bounds: el.getBoundingClientRect(),
  }));
}

// Get accessible label for element
function getElementLabel(element) {
  // aria-label
  if (element.getAttribute('aria-label')) {
    return element.getAttribute('aria-label');
  }

  // aria-labelledby
  const labelledBy = element.getAttribute('aria-labelledby');
  if (labelledBy) {
    const labelEl = document.getElementById(labelledBy);
    if (labelEl) return labelEl.textContent;
  }

  // Input labels
  if (element.id) {
    const label = document.querySelector(`label[for="${element.id}"]`);
    if (label) return label.textContent;
  }

  // Text content
  if (element.textContent) {
    return element.textContent.trim().substring(0, 50);
  }

  // Alt text for images
  if (element.alt) {
    return element.alt;
  }

  // Title attribute
  if (element.title) {
    return element.title;
  }

  return element.tagName.toLowerCase();
}

// Announce element (for screen reader users)
function announceElement(element) {
  // Could integrate with screen reader or provide audio feedback
  const label = getElementLabel(element);
  console.log(`[WIA Gaze] Hovering: ${label}`);
}

// Update cursor style based on settings
function updateCursorStyle() {
  if (!gazeCursor) return;

  gazeCursor.style.width = `${settings.cursorSize}px`;
  gazeCursor.style.height = `${settings.cursorSize}px`;
  gazeCursor.style.borderColor = settings.cursorColor;
  gazeCursor.style.background = `${settings.cursorColor}33`;
  gazeCursor.style.display = settings.showCursor ? 'block' : 'none';

  if (dwellProgress) {
    dwellProgress.style.borderTopColor = settings.cursorColor;
  }
}

// Detect gaze targets on page (for WebGaze API)
function detectGazeTargets() {
  return setupARIATargets().map(t => t.element);
}

// Show/hide gaze cursor
function showGazeCursor(style = {}) {
  settings = { ...settings, ...style, showCursor: true };
  updateCursorStyle();
}

function hideGazeCursor() {
  settings.showCursor = false;
  updateCursorStyle();
}

// Enable/disable dwell selection
function enableDwellSelection(options = {}) {
  settings = { ...settings, ...options, dwellEnabled: true };
}

function disableDwellSelection() {
  settings.dwellEnabled = false;
  resetDwell();
}

// Get ARIA targets
function getARIATargets() {
  return setupARIATargets();
}

// Expose WebGaze API to page
const webGazeAPI = {
  detectGazeTargets,
  showGazeCursor,
  hideGazeCursor,
  enableDwellSelection,
  disableDwellSelection,
  getARIATargets,
};

// Make API available to page scripts
window.WiaGaze = webGazeAPI;

// Initialize on load
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', init);
} else {
  init();
}

// Inject CSS for hover/click effects
const style = document.createElement('style');
style.textContent = `
  .wia-gaze-hover {
    outline: 2px solid ${settings.cursorColor} !important;
    outline-offset: 2px !important;
  }

  .wia-gaze-clicked {
    transform: scale(0.95);
    transition: transform 0.1s;
  }
`;
document.head.appendChild(style);
