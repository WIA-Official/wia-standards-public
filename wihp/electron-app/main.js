/**
 * WIHP Desktop - Electron Main Process
 * Universal Hangul Phonology Converter
 *
 * 홍익인간 (弘益人間) - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

const { app, BrowserWindow, Tray, Menu, globalShortcut, clipboard, ipcMain, nativeImage, Notification } = require('electron');
const path = require('path');

let mainWindow;
let tray;
let isQuitting = false;

// WIHP Engine (embedded)
const WIHPEngine = require('./wihp-engine.js');

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 480,
    height: 640,
    minWidth: 400,
    minHeight: 500,
    frame: true,
    transparent: false,
    resizable: true,
    icon: path.join(__dirname, 'assets', 'icon.png'),
    webPreferences: {
      nodeIntegration: true,
      contextIsolation: false,
    },
    titleBarStyle: 'hiddenInset',
    backgroundColor: '#1e40af',
  });

  mainWindow.loadFile('index.html');

  // Hide window instead of closing (minimize to tray)
  mainWindow.on('close', (event) => {
    if (!isQuitting) {
      event.preventDefault();
      mainWindow.hide();
      return false;
    }
    return true;
  });

  mainWindow.on('closed', () => {
    mainWindow = null;
  });
}

function createTray() {
  // Create tray icon
  const iconPath = path.join(__dirname, 'assets', 'tray-icon.png');
  let trayIcon;

  try {
    trayIcon = nativeImage.createFromPath(iconPath);
    if (trayIcon.isEmpty()) {
      // Create a simple icon if file doesn't exist
      trayIcon = nativeImage.createEmpty();
    }
    trayIcon = trayIcon.resize({ width: 16, height: 16 });
  } catch (e) {
    trayIcon = nativeImage.createEmpty();
  }

  tray = new Tray(trayIcon);

  const contextMenu = Menu.buildFromTemplate([
    {
      label: 'WIHP Desktop',
      enabled: false,
      icon: trayIcon
    },
    { type: 'separator' },
    {
      label: '창 열기 (Show Window)',
      click: () => {
        mainWindow.show();
        mainWindow.focus();
      }
    },
    {
      label: '클립보드 변환 (Convert Clipboard)',
      accelerator: 'CmdOrCtrl+Shift+W',
      click: convertClipboard
    },
    { type: 'separator' },
    {
      label: '도움말 (Help)',
      click: () => {
        require('electron').shell.openExternal('https://wiastandards.com/wihp/');
      }
    },
    { type: 'separator' },
    {
      label: '종료 (Quit)',
      accelerator: 'CmdOrCtrl+Q',
      click: () => {
        isQuitting = true;
        app.quit();
      }
    }
  ]);

  tray.setToolTip('WIHP Desktop - 한글 음성학');
  tray.setContextMenu(contextMenu);

  tray.on('click', () => {
    if (mainWindow.isVisible()) {
      mainWindow.hide();
    } else {
      mainWindow.show();
      mainWindow.focus();
    }
  });

  tray.on('double-click', () => {
    mainWindow.show();
    mainWindow.focus();
  });
}

function convertClipboard() {
  const text = clipboard.readText();
  if (text && text.trim()) {
    const hangul = WIHPEngine.convert(text.trim());
    clipboard.writeText(hangul);

    // Show notification
    if (Notification.isSupported()) {
      new Notification({
        title: 'WIHP 변환 완료',
        body: `${text.substring(0, 20)}${text.length > 20 ? '...' : ''} → ${hangul}`,
        silent: true
      }).show();
    }

    // Send to renderer
    if (mainWindow) {
      mainWindow.webContents.send('clipboard-converted', { original: text, hangul });
    }
  }
}

function registerShortcuts() {
  // Global hotkey: Ctrl+Shift+W (or Cmd+Shift+W on Mac)
  globalShortcut.register('CommandOrControl+Shift+W', convertClipboard);

  // Alternative: Ctrl+Shift+H for Hangul
  globalShortcut.register('CommandOrControl+Shift+H', () => {
    mainWindow.show();
    mainWindow.focus();
  });
}

// IPC handlers
ipcMain.handle('convert', (event, text) => {
  return WIHPEngine.convert(text);
});

ipcMain.handle('to-braille', (event, hangul) => {
  return WIHPEngine.toBraille(hangul);
});

ipcMain.handle('get-clipboard', () => {
  return clipboard.readText();
});

ipcMain.handle('set-clipboard', (event, text) => {
  clipboard.writeText(text);
  return true;
});

// App lifecycle
app.whenReady().then(() => {
  createWindow();
  createTray();
  registerShortcuts();

  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) {
      createWindow();
    } else {
      mainWindow.show();
    }
  });
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    // Keep running in tray on Windows/Linux
  }
});

app.on('before-quit', () => {
  isQuitting = true;
});

app.on('will-quit', () => {
  globalShortcut.unregisterAll();
});

// Handle second instance
const gotTheLock = app.requestSingleInstanceLock();
if (!gotTheLock) {
  app.quit();
} else {
  app.on('second-instance', () => {
    if (mainWindow) {
      if (mainWindow.isMinimized()) mainWindow.restore();
      mainWindow.show();
      mainWindow.focus();
    }
  });
}
