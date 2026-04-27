/**
 * WIA Screen Reader - Chrome Extension Background Script
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

// WIHP Word Map
const WIHP_MAP = {
  'hello': '헬로우',
  'world': '월드',
  'the': '더',
  'is': '이즈',
  'and': '앤드',
  'for': '포',
  'with': '위드',
  'you': '유',
  'we': '위',
  'good': '굿',
  'thank': '땡크',
  'please': '플리즈',
  'welcome': '웰컴'
};

// Braille Map
const BRAILLE_MAP = {
  'a': '⠁', 'b': '⠃', 'c': '⠉', 'd': '⠙', 'e': '⠑',
  'f': '⠋', 'g': '⠛', 'h': '⠓', 'i': '⠊', 'j': '⠚',
  'k': '⠅', 'l': '⠇', 'm': '⠍', 'n': '⠝', 'o': '⠕',
  'p': '⠏', 'q': '⠟', 'r': '⠗', 's': '⠎', 't': '⠞',
  'u': '⠥', 'v': '⠧', 'w': '⠺', 'x': '⠭', 'y': '⠽', 'z': '⠵',
  ' ': ' ', '.': '⠲', ',': '⠂', '!': '⠖', '?': '⠦'
};

// Initialize extension
chrome.runtime.onInstalled.addListener(() => {
  // Create context menu
  chrome.contextMenus.create({
    id: 'wia-convert-wihp',
    title: 'Convert to WIHP',
    contexts: ['selection']
  });

  chrome.contextMenus.create({
    id: 'wia-convert-braille',
    title: 'Convert to Braille',
    contexts: ['selection']
  });

  chrome.contextMenus.create({
    id: 'wia-speak',
    title: 'Speak Selection',
    contexts: ['selection']
  });

  // Initialize storage
  chrome.storage.sync.set({
    language: 'en',
    ttsRate: 1.0,
    ttsPitch: 1.0,
    useWIHP: true
  });
});

// Handle context menu clicks
chrome.contextMenus.onClicked.addListener((info, tab) => {
  const text = info.selectionText;

  switch (info.menuItemId) {
    case 'wia-convert-wihp':
      convertToWIHP(text, tab);
      break;
    case 'wia-convert-braille':
      convertToBraille(text, tab);
      break;
    case 'wia-speak':
      speakText(text);
      break;
  }
});

// Handle keyboard commands
chrome.commands.onCommand.addListener((command) => {
  chrome.tabs.query({ active: true, currentWindow: true }, (tabs) => {
    chrome.tabs.sendMessage(tabs[0].id, { action: command });
  });
});

// Handle messages from content script
chrome.runtime.onMessage.addListener((request, sender, sendResponse) => {
  switch (request.action) {
    case 'getWIHP':
      sendResponse({ wihp: convertTextToWIHP(request.text) });
      break;
    case 'getBraille':
      sendResponse({ braille: convertTextToBraille(request.text) });
      break;
    case 'speak':
      speakText(request.text);
      sendResponse({ success: true });
      break;
    case 'getSettings':
      chrome.storage.sync.get(null, (settings) => {
        sendResponse(settings);
      });
      return true; // Keep channel open for async response
  }
});

// Convert text to WIHP
function convertTextToWIHP(text) {
  return text.toLowerCase().split(' ').map(word => {
    const clean = word.replace(/[^a-z]/g, '');
    return WIHP_MAP[clean] || clean;
  }).join(' ');
}

// Convert text to Braille
function convertTextToBraille(text) {
  return text.toLowerCase().split('').map(char => {
    return BRAILLE_MAP[char] || char;
  }).join('');
}

// Send WIHP result to content script
function convertToWIHP(text, tab) {
  const wihp = convertTextToWIHP(text);
  chrome.tabs.sendMessage(tab.id, {
    action: 'showResult',
    type: 'WIHP',
    original: text,
    result: wihp
  });
}

// Send Braille result to content script
function convertToBraille(text, tab) {
  const braille = convertTextToBraille(text);
  chrome.tabs.sendMessage(tab.id, {
    action: 'showResult',
    type: 'Braille',
    original: text,
    result: braille
  });
}

// Speak text using TTS
function speakText(text) {
  chrome.storage.sync.get(['ttsRate', 'ttsPitch', 'useWIHP'], (settings) => {
    const textToSpeak = settings.useWIHP ? convertTextToWIHP(text) : text;

    chrome.tts.speak(textToSpeak, {
      rate: settings.ttsRate || 1.0,
      pitch: settings.ttsPitch || 1.0
    });
  });
}
