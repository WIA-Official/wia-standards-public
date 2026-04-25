/**
 * WIHP Chrome Extension - Background Service Worker
 */

// Create context menu on install
chrome.runtime.onInstalled.addListener(() => {
  chrome.contextMenus.create({
    id: 'wihp-convert',
    title: 'Convert to WIHP Hangul',
    contexts: ['selection']
  });

  console.log('WIHP Extension installed');
});

// Handle context menu click
chrome.contextMenus.onClicked.addListener((info, tab) => {
  if (info.menuItemId === 'wihp-convert' && info.selectionText) {
    chrome.tabs.sendMessage(tab.id, {
      action: 'convert',
      text: info.selectionText
    });
  }
});

// Handle messages from content script
chrome.runtime.onMessage.addListener((request, sender, sendResponse) => {
  if (request.action === 'getConversion') {
    // Could add server-side conversion here
    sendResponse({ success: true });
  }
  return true;
});
