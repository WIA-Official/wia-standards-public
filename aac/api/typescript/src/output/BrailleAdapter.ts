/**
 * WIA AAC Braille Adapter
 * Phase 4: WIA Ecosystem Integration
 *
 * WIA Braille integration for braille output
 */

import { BaseOutputAdapter } from './IOutputAdapter';
import {
  OutputOptions,
  BrailleOutput,
  BrailleDisplay,
  OutputError,
  OutputErrorCode
} from './types';

/**
 * Braille adapter interface
 */
export interface IBrailleAdapter {
  /**
   * Convert text to IPA
   * @param text - Text to convert
   */
  textToIPA(text: string): Promise<string>;

  /**
   * Convert text to braille
   * @param text - Text to convert
   */
  textToBraille(text: string): Promise<BrailleOutput>;

  /**
   * Send braille to display
   * @param braille - Braille output to send
   */
  sendToDisplay(braille: BrailleOutput): Promise<void>;

  /**
   * Get connected displays
   */
  getConnectedDisplays(): Promise<BrailleDisplay[]>;

  /**
   * Set active display
   * @param displayId - Display ID
   */
  setDisplay(displayId: string): void;
}

/**
 * Mock Braille Adapter
 * Uses WIA Braille system (IPA-based)
 */
export class MockBrailleAdapter extends BaseOutputAdapter implements IBrailleAdapter {
  readonly type = 'braille' as const;
  readonly name = 'MockBraille';

  private ipaMap: Map<string, string> = new Map();
  private brailleMap: Map<string, string> = new Map();
  private selectedDisplayId: string | null = null;
  private mockDisplays: BrailleDisplay[] = [];

  async initialize(): Promise<void> {
    this.loadIPAMap();
    this.loadBrailleMap();
    this.initMockDisplays();
    this._state = 'idle';
    this.emit('start', { message: 'Braille adapter initialized' });
  }

  /**
   * Initialize mock braille displays
   */
  private initMockDisplays(): void {
    this.mockDisplays = [
      { id: 'mock-40', name: 'Mock Braille Display 40', cells: 40, connected: true },
      { id: 'mock-80', name: 'Mock Braille Display 80', cells: 80, connected: false }
    ];
    this.selectedDisplayId = 'mock-40';
  }

  /**
   * Load simple IPA mapping for common words
   * In production, this would use a proper G2P (Grapheme-to-Phoneme) library
   */
  private loadIPAMap(): void {
    // Korean words
    this.ipaMap.set('안녕', '/annjʌŋ/');
    this.ipaMap.set('안녕하세요', '/annjʌŋhasejo/');
    this.ipaMap.set('감사', '/kamsa/');
    this.ipaMap.set('감사합니다', '/kamsahamnida/');
    this.ipaMap.set('사랑', '/saraŋ/');
    this.ipaMap.set('사랑해', '/saraŋhɛ/');
    this.ipaMap.set('네', '/ne/');
    this.ipaMap.set('아니오', '/anio/');
    this.ipaMap.set('도움', '/toum/');
    this.ipaMap.set('미안', '/mian/');
    this.ipaMap.set('좋아', '/tʃoa/');

    // English words
    this.ipaMap.set('hello', '/həˈloʊ/');
    this.ipaMap.set('thanks', '/θæŋks/');
    this.ipaMap.set('love', '/lʌv/');
    this.ipaMap.set('help', '/hɛlp/');
    this.ipaMap.set('yes', '/jɛs/');
    this.ipaMap.set('no', '/noʊ/');
  }

  /**
   * Load IPA to Braille mapping (WIA Braille system - 8-dot)
   * Based on IPA phonemes to braille cells
   */
  private loadBrailleMap(): void {
    // Vowels
    this.brailleMap.set('a', '⠁');   // U+2801
    this.brailleMap.set('e', '⠑');   // U+2811
    this.brailleMap.set('i', '⠊');   // U+280A
    this.brailleMap.set('o', '⠕');   // U+2815
    this.brailleMap.set('u', '⠥');   // U+2825
    this.brailleMap.set('ə', '⠢');   // U+2822 (schwa)
    this.brailleMap.set('ʌ', '⠪');   // U+282A
    this.brailleMap.set('ɛ', '⠫');   // U+282B
    this.brailleMap.set('ɔ', '⠬');   // U+282C
    this.brailleMap.set('æ', '⠭');   // U+282D
    this.brailleMap.set('ʊ', '⠮');   // U+282E

    // Consonants
    this.brailleMap.set('p', '⠏');   // U+280F
    this.brailleMap.set('b', '⠃');   // U+2803
    this.brailleMap.set('t', '⠞');   // U+281E
    this.brailleMap.set('d', '⠙');   // U+2819
    this.brailleMap.set('k', '⠅');   // U+2805
    this.brailleMap.set('g', '⠛');   // U+281B
    this.brailleMap.set('m', '⠍');   // U+280D
    this.brailleMap.set('n', '⠝');   // U+281D
    this.brailleMap.set('ŋ', '⠻');   // U+283B (eng)
    this.brailleMap.set('f', '⠋');   // U+280B
    this.brailleMap.set('v', '⠧');   // U+2827
    this.brailleMap.set('s', '⠎');   // U+280E
    this.brailleMap.set('z', '⠵');   // U+2835
    this.brailleMap.set('h', '⠓');   // U+2813
    this.brailleMap.set('l', '⠇');   // U+2807
    this.brailleMap.set('r', '⠗');   // U+2817
    this.brailleMap.set('w', '⠺');   // U+283A
    this.brailleMap.set('j', '⠚');   // U+281A
    this.brailleMap.set('ʃ', '⠱');   // U+2831 (sh)
    this.brailleMap.set('ʒ', '⠴');   // U+2834 (zh)
    this.brailleMap.set('tʃ', '⠹');  // U+2839 (ch) - combined
    this.brailleMap.set('dʒ', '⠚');  // U+281A (j as in "judge")
    this.brailleMap.set('θ', '⠹');   // U+2839 (th voiceless)
    this.brailleMap.set('ð', '⠮');   // U+282E (th voiced)

    // Diacritics/Markers
    this.brailleMap.set('ˈ', '⠄');   // U+2804 (primary stress)
    this.brailleMap.set('ˌ', '⠠');   // U+2820 (secondary stress)
    this.brailleMap.set('ː', '⠒');   // U+2812 (long)

    // Space
    this.brailleMap.set(' ', '⠀');   // U+2800 (blank)
  }

  /**
   * Get braille character for an IPA character
   */
  private getBrailleChar(ipaChar: string): string {
    return this.brailleMap.get(ipaChar) || '⠿'; // U+283F for unknown
  }

  async textToIPA(text: string): Promise<string> {
    const words = text.toLowerCase().split(/\s+/);
    const ipaWords: string[] = [];

    for (const word of words) {
      const ipa = this.ipaMap.get(word);
      if (ipa) {
        ipaWords.push(ipa);
      } else {
        // Simple fallback: just wrap in slashes
        ipaWords.push(`/${word}/`);
      }
    }

    return ipaWords.join(' ');
  }

  async textToBraille(text: string): Promise<BrailleOutput> {
    const ipa = await this.textToIPA(text);

    // Remove IPA delimiters and process
    const cleanIPA = ipa.replace(/[\/\[\]]/g, '');
    const chars = this.tokenizeIPA(cleanIPA);

    let braille = '';
    const unicode: string[] = [];
    const dots: number[] = [];

    for (const char of chars) {
      const brailleChar = this.getBrailleChar(char);
      braille += brailleChar;

      const codePoint = brailleChar.charCodeAt(0);
      unicode.push(`U+${codePoint.toString(16).toUpperCase().padStart(4, '0')}`);
      dots.push(codePoint - 0x2800);
    }

    return {
      text,
      ipa,
      braille,
      unicode,
      dots
    };
  }

  /**
   * Tokenize IPA string into individual phonemes
   */
  private tokenizeIPA(ipa: string): string[] {
    const tokens: string[] = [];
    let i = 0;

    while (i < ipa.length) {
      // Check for two-character combinations first
      if (i < ipa.length - 1) {
        const twoChar = ipa.substring(i, i + 2);
        if (this.brailleMap.has(twoChar)) {
          tokens.push(twoChar);
          i += 2;
          continue;
        }
      }

      // Single character
      const char = ipa[i];
      tokens.push(char);
      i++;
    }

    return tokens;
  }

  async sendToDisplay(braille: BrailleOutput): Promise<void> {
    if (!this.selectedDisplayId) {
      throw new OutputError(
        OutputErrorCode.DEVICE_NOT_CONNECTED,
        'No braille display selected',
        true
      );
    }

    const display = this.mockDisplays.find(d => d.id === this.selectedDisplayId);
    if (!display || !display.connected) {
      throw new OutputError(
        OutputErrorCode.DEVICE_NOT_CONNECTED,
        'Braille display not connected',
        true
      );
    }

    this._state = 'outputting';
    this.emit('start', {
      displayId: this.selectedDisplayId,
      braille: braille.braille
    });

    // Simulate sending to display
    // In production, this would use BrlAPI or similar

    // Display duration based on braille length
    const readingTime = Math.max(braille.braille.length * 100, 500);
    await new Promise(resolve => setTimeout(resolve, readingTime));

    this._state = 'idle';
    this.emit('end', {
      displayId: this.selectedDisplayId,
      braille: braille.braille
    });
  }

  async output(text: string, options?: OutputOptions): Promise<void> {
    const brailleOutput = await this.textToBraille(text);
    await this.sendToDisplay(brailleOutput);
  }

  async getConnectedDisplays(): Promise<BrailleDisplay[]> {
    return this.mockDisplays.filter(d => d.connected);
  }

  setDisplay(displayId: string): void {
    const display = this.mockDisplays.find(d => d.id === displayId);
    if (!display) {
      throw new OutputError(
        OutputErrorCode.DEVICE_NOT_CONNECTED,
        `Display not found: ${displayId}`,
        true
      );
    }
    this.selectedDisplayId = displayId;
  }

  stop(): void {
    this._state = 'idle';
  }

  isAvailable(): boolean {
    return this.mockDisplays.some(d => d.connected);
  }

  async dispose(): Promise<void> {
    this.ipaMap.clear();
    this.brailleMap.clear();
    this.handlers.clear();
  }

  /**
   * Add custom IPA mapping
   */
  addIPAMapping(word: string, ipa: string): void {
    this.ipaMap.set(word.toLowerCase(), ipa);
  }

  /**
   * Add custom braille mapping
   */
  addBrailleMapping(ipa: string, braille: string): void {
    this.brailleMap.set(ipa, braille);
  }

  /**
   * Simulate connecting a display
   */
  connectDisplay(displayId: string): void {
    const display = this.mockDisplays.find(d => d.id === displayId);
    if (display) {
      display.connected = true;
    }
  }

  /**
   * Simulate disconnecting a display
   */
  disconnectDisplay(displayId: string): void {
    const display = this.mockDisplays.find(d => d.id === displayId);
    if (display) {
      display.connected = false;
    }
  }
}
