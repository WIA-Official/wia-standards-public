/**
 * WIA Braille Display Standard - SDK Implementation
 * @packageDocumentation
 * @module wia-braille-display
 */

import { EventEmitter } from 'events';
import * as types from './types';

export * from './types';

export class WIABrailleDisplay extends EventEmitter {
  private config: types.BrailleDisplayConfig;
  private displays: Map<string, types.DisplayInfo> = new Map();
  private states: Map<string, types.DisplayState> = new Map();
  private translations: Map<string, string> = new Map();

  constructor(config: types.BrailleDisplayConfig) {
    super();
    this.config = config;
    this.initializeTranslations();
  }

  private initializeTranslations(): void {
    const basicTable = 'abcdefghijklmnopqrstuvwxyz0123456789';
    const brailleUnicode = '⠁⠃⠉⠙⠑⠋⠛⠓⠊⠚⠅⠇⠍⠝⠕⠏⠟⠗⠎⠞⠥⠧⠺⠭⠽⠵⠚⠁⠃⠉⠙⠑⠋⠛⠓⠊';
    for (let i = 0; i < basicTable.length; i++) {
      this.translations.set(basicTable[i], brailleUnicode[i] || '⠿');
    }
  }

  async connect(deviceId: string, connection: types.ConnectionType): Promise<types.DisplayInfo> {
    const display: types.DisplayInfo = {
      id: deviceId,
      name: 'Braille Display',
      manufacturer: 'WIA',
      model: 'BD-40',
      type: types.DisplayType.SingleLine,
      cellCount: 40,
      rowCount: 1,
      dotCount: 8,
      hasCursor: true,
      hasButtons: true,
      buttonCount: 8,
      connection,
      firmwareVersion: '1.0.0'
    };

    this.displays.set(deviceId, display);
    this.states.set(deviceId, {
      deviceId,
      connected: true,
      cells: Array(40).fill({ dots: Array(8).fill(false) }),
      cursorPosition: 0,
      activeRow: 0,
      buttonStates: Array(8).fill(false),
      lastUpdate: new Date()
    });

    this.emit('connected', display);
    return display;
  }

  async disconnect(deviceId: string): Promise<void> {
    this.displays.delete(deviceId);
    const state = this.states.get(deviceId);
    if (state) state.connected = false;
    this.emit('disconnected', deviceId);
  }

  getDisplayInfo(deviceId: string): types.DisplayInfo | undefined {
    return this.displays.get(deviceId);
  }

  getDisplayState(deviceId: string): types.DisplayState | undefined {
    return this.states.get(deviceId);
  }

  translateToBraille(content: types.TextContent): types.TranslationResult {
    const cells: types.BrailleCell[] = [];
    const mappings: types.CharacterMapping[] = [];
    let brailleStr = '';

    for (let i = 0; i < content.text.length; i++) {
      const char = content.text[i].toLowerCase();
      const brailleChar = this.translations.get(char) || '⠀';
      brailleStr += brailleChar;

      const dots = this.unicodeToDots(brailleChar);
      cells.push({ dots, character: char, unicode: brailleChar.charCodeAt(0) });
      mappings.push({ textIndex: i, brailleIndex: cells.length - 1, textLength: 1, brailleLength: 1 });
    }

    return { brailleCells: cells, brailleString: brailleStr, characterMapping: mappings };
  }

  private unicodeToDots(brailleChar: string): boolean[] {
    const code = brailleChar.charCodeAt(0) - 0x2800;
    return [
      (code & 0x01) !== 0, (code & 0x02) !== 0, (code & 0x04) !== 0, (code & 0x08) !== 0,
      (code & 0x10) !== 0, (code & 0x20) !== 0, (code & 0x40) !== 0, (code & 0x80) !== 0
    ];
  }

  async displayText(deviceId: string, content: types.TextContent): Promise<void> {
    const state = this.states.get(deviceId);
    const display = this.displays.get(deviceId);
    if (!state || !display) throw new Error('Device not found');

    const translation = this.translateToBraille(content);
    state.cells = translation.brailleCells.slice(0, display.cellCount);
    state.cursorPosition = content.cursorPosition || 0;
    state.lastUpdate = new Date();

    this.emit('display-updated', { deviceId, cells: state.cells });
  }

  async displayGraphic(deviceId: string, graphic: types.TactileGraphic): Promise<void> {
    const display = this.displays.get(deviceId);
    if (!display || display.type !== types.DisplayType.TactileGraphics) {
      throw new Error('Display does not support tactile graphics');
    }

    this.emit('graphic-displayed', { deviceId, graphic });
  }

  async setCursor(deviceId: string, position: number): Promise<void> {
    const state = this.states.get(deviceId);
    if (!state) throw new Error('Device not found');

    state.cursorPosition = position;
    this.emit('cursor-moved', { deviceId, position });
  }

  async scroll(deviceId: string, direction: 'left' | 'right', amount: number = 1): Promise<void> {
    const state = this.states.get(deviceId);
    if (!state) throw new Error('Device not found');

    const delta = direction === 'left' ? -amount : amount;
    state.cursorPosition = Math.max(0, state.cursorPosition + delta);
    this.emit('scrolled', { deviceId, direction, newPosition: state.cursorPosition });
  }

  processInput(event: types.InputEvent): void {
    this.emit('input', event);

    switch (event.type) {
      case 'button':
        this.emit('button', event.data as types.ButtonEvent);
        break;
      case 'cursor':
        this.emit('cursor-route', event.data as types.CursorEvent);
        break;
      case 'braille_input':
        const input = event.data as types.BrailleInputEvent;
        this.emit('braille-input', input);
        break;
    }
  }

  checkCompliance(targetLevel: types.CertificationLevel): types.ComplianceReport {
    const tests: types.TestResult[] = [];

    tests.push({
      testName: 'Configuration Validation',
      passed: this.config.defaultGrade !== undefined && this.config.defaultCode !== undefined,
      notes: 'Default grade and code must be defined'
    });

    tests.push({
      testName: 'Translation Support',
      passed: this.translations.size > 0,
      notes: 'Braille translation table must be loaded'
    });

    if (targetLevel !== types.CertificationLevel.Bronze) {
      tests.push({
        testName: 'Audio Feedback',
        passed: this.config.audioFeedback === true,
        notes: 'Audio feedback required for Silver/Gold'
      });
    }

    const passed = tests.every(t => t.passed);

    return {
      standard: 'WIA-BRAILLE-DISPLAY',
      testDate: new Date().toISOString(),
      config: this.config,
      targetLevel,
      tests,
      passed,
      achievedLevel: passed ? targetLevel : undefined
    };
  }
}

export default { WIABrailleDisplay };
