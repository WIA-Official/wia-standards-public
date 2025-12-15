/**
 * WIA BCI Mock Output Adapters
 *
 * Mock implementations for testing and development.
 */

import { BaseOutputAdapter } from './IOutputAdapter';
import {
  OutputType,
  OutputContent,
  OutputOptions,
  ISPCode,
  Avatar,
  BrailleOutput,
  BrailleDisplay,
} from './types';

/**
 * Mock Sign Language Adapter
 */
export class MockSignLanguageAdapter extends BaseOutputAdapter {
  readonly type: OutputType = 'sign_language';
  readonly name = 'Mock Sign Language';

  private avatars: Avatar[] = [
    { id: 'default', name: 'Default Avatar', style: 'simple' },
    { id: 'realistic', name: 'Realistic Avatar', style: 'realistic' },
    { id: 'cartoon', name: 'Cartoon Avatar', style: 'cartoon' },
  ];
  private currentAvatar: Avatar = this.avatars[0];
  private speed = 1.0;

  async initialize(_options?: OutputOptions): Promise<void> {
    this.ready = true;
    this.emit('ready', {});
  }

  async output(content: OutputContent): Promise<void> {
    if (content.type === 'text' && content.text) {
      const codes = await this.textToISP(content.text);
      await this.playSequence(codes);
    }
  }

  async textToISP(text: string): Promise<ISPCode[]> {
    // Mock ISP conversion: generate placeholder codes
    const words = text.split(/\s+/);
    return words.map((word, i) => ({
      code: `HS${String(i % 10).padStart(2, '0')}-LC${String((i + 1) % 10).padStart(2, '0')}-MV${String((i + 2) % 10).padStart(2, '0')}`,
      meaning: word,
      duration: Math.max(500, word.length * 100),
    }));
  }

  async playGesture(code: ISPCode): Promise<void> {
    this.emit('start', {});
    // Simulate gesture duration
    await this.delay(code.duration ?? 500);
    this.emit('end', {});
  }

  async playSequence(codes: ISPCode[]): Promise<void> {
    for (const code of codes) {
      await this.playGesture(code);
    }
  }

  getAvatars(): Avatar[] {
    return this.avatars;
  }

  setAvatar(avatarId: string): void {
    const avatar = this.avatars.find((a) => a.id === avatarId);
    if (avatar) {
      this.currentAvatar = avatar;
    }
  }

  getAvatar(): Avatar {
    return this.currentAvatar;
  }

  setSpeed(speed: number): void {
    this.speed = Math.max(0.5, Math.min(2, speed));
  }

  private delay(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms / this.speed));
  }
}

/**
 * Mock Braille Adapter
 */
export class MockBrailleAdapter extends BaseOutputAdapter {
  readonly type: OutputType = 'braille';
  readonly name = 'Mock Braille';

  private displays: BrailleDisplay[] = [
    {
      id: 'mock-40',
      name: 'Mock Braille 40',
      manufacturer: 'WIA',
      cells: 40,
      rows: 1,
      connected: true,
      battery: 100,
    },
  ];
  private currentDisplay: BrailleDisplay = this.displays[0];
  private grade: 1 | 2 = 2;

  async initialize(_options?: OutputOptions): Promise<void> {
    this.ready = true;
    this.emit('ready', {});
  }

  async output(content: OutputContent): Promise<void> {
    if (content.type === 'text' && content.text) {
      const braille = await this.textToBraille(content.text);
      await this.sendToDisplay(braille);
    }
  }

  async textToIPA(text: string, _language?: string): Promise<string> {
    // Mock IPA conversion (very simplified)
    const ipaMap: Record<string, string> = {
      a: 'ɑ', e: 'e', i: 'i', o: 'o', u: 'u',
      안: 'an', 녕: 'njʌŋ', 하: 'ha', 세: 'se', 요: 'jo',
    };

    let ipa = '';
    for (const char of text) {
      ipa += ipaMap[char.toLowerCase()] ?? char;
    }
    return `/${ipa}/`;
  }

  async textToBraille(text: string, language?: string): Promise<BrailleOutput> {
    const ipa = await this.textToIPA(text, language);

    // Mock braille conversion (simplified Unicode braille)
    const brailleMap: Record<string, string> = {
      a: '⠁', b: '⠃', c: '⠉', d: '⠙', e: '⠑',
      f: '⠋', g: '⠛', h: '⠓', i: '⠊', j: '⠚',
      k: '⠅', l: '⠇', m: '⠍', n: '⠝', o: '⠕',
      p: '⠏', q: '⠟', r: '⠗', s: '⠎', t: '⠞',
      u: '⠥', v: '⠧', w: '⠺', x: '⠭', y: '⠽', z: '⠵',
      ' ': '⠀',
      안: '⠁⠝', 녕: '⠝⠚', 하: '⠓⠁', 세: '⠎⠑', 요: '⠚⠕',
    };

    let braille = '';
    const unicode: string[] = [];

    for (const char of text) {
      const bc = brailleMap[char.toLowerCase()] ?? '⠿';
      braille += bc;
      for (const c of bc) {
        unicode.push(`U+${c.charCodeAt(0).toString(16).toUpperCase().padStart(4, '0')}`);
      }
    }

    return {
      original: text,
      ipa,
      braille,
      unicode,
      cells: braille.length,
      grade: this.grade,
    };
  }

  async sendToDisplay(braille: BrailleOutput): Promise<void> {
    this.emit('start', {});
    // Simulate display update delay
    await new Promise((resolve) => setTimeout(resolve, 100));
    console.log(`[Braille Display] ${braille.braille}`);
    this.emit('end', {});
  }

  async getDisplays(): Promise<BrailleDisplay[]> {
    return this.displays;
  }

  setDisplay(displayId: string): void {
    const display = this.displays.find((d) => d.id === displayId);
    if (display) {
      this.currentDisplay = display;
    }
  }

  getDisplay(): BrailleDisplay {
    return this.currentDisplay;
  }

  setGrade(grade: 1 | 2): void {
    this.grade = grade;
  }
}

/**
 * Mock Output Adapter (generic)
 */
export class MockOutputAdapter extends BaseOutputAdapter {
  readonly type: OutputType;
  readonly name: string;

  private outputLog: OutputContent[] = [];

  constructor(type: OutputType = 'custom', name = 'Mock Output') {
    super();
    this.type = type;
    this.name = name;
  }

  async initialize(_options?: OutputOptions): Promise<void> {
    this.ready = true;
    this.emit('ready', {});
  }

  async output(content: OutputContent): Promise<void> {
    this.emit('start', { content });
    this.outputLog.push(content);
    // Simulate processing
    await new Promise((resolve) => setTimeout(resolve, 50));
    this.emit('end', { content });
  }

  getOutputLog(): OutputContent[] {
    return [...this.outputLog];
  }

  clearLog(): void {
    this.outputLog = [];
  }
}
