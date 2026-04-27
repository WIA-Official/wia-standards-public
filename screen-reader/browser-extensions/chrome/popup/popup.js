/**
 * WIA Screen Reader - Popup Script
 */

// Load settings
document.addEventListener('DOMContentLoaded', () => {
    chrome.storage.sync.get(null, (settings) => {
        if (settings.language) {
            document.getElementById('language').value = settings.language;
        }
        if (settings.ttsRate) {
            document.getElementById('tts-rate').value = settings.ttsRate;
            document.getElementById('rate-value').textContent = settings.ttsRate;
        }
        if (settings.ttsPitch) {
            document.getElementById('tts-pitch').value = settings.ttsPitch;
            document.getElementById('pitch-value').textContent = settings.ttsPitch;
        }
        if (settings.useWIHP !== undefined) {
            document.getElementById('use-wihp').checked = settings.useWIHP;
        }
    });
});

// Save settings on change
document.getElementById('language').addEventListener('change', (e) => {
    chrome.storage.sync.set({ language: e.target.value });
});

document.getElementById('tts-rate').addEventListener('input', (e) => {
    const value = parseFloat(e.target.value);
    document.getElementById('rate-value').textContent = value.toFixed(1);
    chrome.storage.sync.set({ ttsRate: value });
});

document.getElementById('tts-pitch').addEventListener('input', (e) => {
    const value = parseFloat(e.target.value);
    document.getElementById('pitch-value').textContent = value.toFixed(1);
    chrome.storage.sync.set({ ttsPitch: value });
});

document.getElementById('use-wihp').addEventListener('change', (e) => {
    chrome.storage.sync.set({ useWIHP: e.target.checked });
});

// Quick action buttons
document.getElementById('btn-wihp').addEventListener('click', () => {
    sendCommand('convert-wihp');
});

document.getElementById('btn-braille').addEventListener('click', () => {
    sendCommand('convert-braille');
});

document.getElementById('btn-speak').addEventListener('click', () => {
    sendCommand('speak-selection');
});

function sendCommand(action) {
    chrome.tabs.query({ active: true, currentWindow: true }, (tabs) => {
        chrome.tabs.sendMessage(tabs[0].id, { action });
    });
}
