/**
 * WIA Eye Gaze Browser Extension - Background Service Worker
 *
 * Manages WebSocket connection to eye tracker and coordinates with content scripts.
 *
 * 弘益人間 - 널리 인간을 이롭게
 */

// State
let wsConnection = null;
let isConnected = false;
let isTracking = false;
let settings = {
  serverUrl: 'ws://localhost:8765/wia-eye-gaze/v1/stream',
  dwellTime: 800,
  showCursor: true,
  cursorSize: 40,
  cursorColor: '#00ff00',
  dwellEnabled: true,
  scrollEnabled: true,
  autoConnect: false,
};

// Load settings on startup
chrome.storage.sync.get(settings, (stored) => {
  settings = { ...settings, ...stored };
  if (settings.autoConnect) {
    connect();
  }
});

// Message handling from popup and content scripts
chrome.runtime.onMessage.addListener((message, sender, sendResponse) => {
  switch (message.type) {
    case 'connect':
      connect().then(() => sendResponse({ success: true }));
      return true;

    case 'disconnect':
      disconnect();
      sendResponse({ success: true });
      break;

    case 'start':
      startTracking();
      sendResponse({ success: true });
      break;

    case 'stop':
      stopTracking();
      sendResponse({ success: true });
      break;

    case 'getStatus':
      sendResponse({
        isConnected,
        isTracking,
        settings,
      });
      break;

    case 'updateSettings':
      settings = { ...settings, ...message.settings };
      chrome.storage.sync.set(settings);
      broadcastToContentScripts({ type: 'settingsUpdated', settings });
      sendResponse({ success: true });
      break;

    case 'getSettings':
      sendResponse({ settings });
      break;

    default:
      sendResponse({ error: 'Unknown message type' });
  }
});

// Connect to eye tracker server
async function connect() {
  if (wsConnection) {
    disconnect();
  }

  return new Promise((resolve, reject) => {
    try {
      wsConnection = new WebSocket(settings.serverUrl, ['wia-eye-gaze-v1']);

      wsConnection.onopen = () => {
        isConnected = true;
        console.log('Connected to eye tracker server');
        updateBadge('connected');
        resolve();
      };

      wsConnection.onclose = () => {
        isConnected = false;
        isTracking = false;
        console.log('Disconnected from eye tracker server');
        updateBadge('disconnected');
      };

      wsConnection.onerror = (error) => {
        console.error('WebSocket error:', error);
        updateBadge('error');
        reject(error);
      };

      wsConnection.onmessage = (event) => {
        handleServerMessage(event.data);
      };
    } catch (error) {
      reject(error);
    }
  });
}

// Disconnect from server
function disconnect() {
  if (wsConnection) {
    wsConnection.close();
    wsConnection = null;
  }
  isConnected = false;
  isTracking = false;
  updateBadge('disconnected');
}

// Start tracking
function startTracking() {
  if (!wsConnection || wsConnection.readyState !== WebSocket.OPEN) {
    console.error('Not connected to server');
    return;
  }

  wsConnection.send(JSON.stringify({
    type: 'control',
    timestamp: Date.now(),
    action: 'start',
  }));

  isTracking = true;
  updateBadge('tracking');
}

// Stop tracking
function stopTracking() {
  if (!wsConnection || wsConnection.readyState !== WebSocket.OPEN) {
    return;
  }

  wsConnection.send(JSON.stringify({
    type: 'control',
    timestamp: Date.now(),
    action: 'stop',
  }));

  isTracking = false;
  updateBadge('connected');
}

// Handle messages from server
function handleServerMessage(data) {
  try {
    const message = JSON.parse(data);

    switch (message.type) {
      case 'gaze_data':
        // Forward gaze data to active tab's content script
        if (message.points && message.points.length > 0) {
          const lastPoint = message.points[message.points.length - 1];
          broadcastToContentScripts({
            type: 'gazePoint',
            point: {
              x: lastPoint.x,
              y: lastPoint.y,
              confidence: lastPoint.c,
              valid: lastPoint.v,
              timestamp: lastPoint.t,
            },
          });
        }
        break;

      case 'gaze_event':
        // Forward events to content script
        broadcastToContentScripts({
          type: 'gazeEvent',
          event: message.event,
        });
        break;

      case 'status':
        // Update connection status
        console.log('Server status:', message.status);
        break;

      case 'error':
        console.error('Server error:', message.error);
        break;
    }
  } catch (error) {
    console.error('Error parsing server message:', error);
  }
}

// Broadcast message to all content scripts
async function broadcastToContentScripts(message) {
  const tabs = await chrome.tabs.query({});
  for (const tab of tabs) {
    if (tab.id) {
      chrome.tabs.sendMessage(tab.id, message).catch(() => {
        // Tab might not have content script loaded
      });
    }
  }
}

// Update extension badge
function updateBadge(status) {
  const badges = {
    disconnected: { text: '', color: '#888888' },
    connected: { text: 'ON', color: '#2ecc71' },
    tracking: { text: '👁', color: '#00d4aa' },
    error: { text: '!', color: '#e74c3c' },
  };

  const badge = badges[status] || badges.disconnected;
  chrome.action.setBadgeText({ text: badge.text });
  chrome.action.setBadgeBackgroundColor({ color: badge.color });
}

// Handle keyboard shortcuts
chrome.commands.onCommand.addListener((command) => {
  switch (command) {
    case 'toggle-gaze':
      settings.showCursor = !settings.showCursor;
      chrome.storage.sync.set(settings);
      broadcastToContentScripts({ type: 'settingsUpdated', settings });
      break;

    case 'toggle-dwell':
      settings.dwellEnabled = !settings.dwellEnabled;
      chrome.storage.sync.set(settings);
      broadcastToContentScripts({ type: 'settingsUpdated', settings });
      break;
  }
});

// Initialize badge
updateBadge('disconnected');
