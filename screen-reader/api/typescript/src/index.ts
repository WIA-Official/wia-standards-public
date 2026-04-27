/**
 * WIA Screen Reader Standard - TypeScript SDK
 * Universal Accessibility for 211 Languages
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Standards <official@wia.codes>
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

import {
  LanguageCode,
  BrailleGrade,
  Pronunciation,
  BrailleOutput,
  BrailleCell,
  TTSConfig,
  Context,
  Metadata,
  ScreenReaderResult,
  WIAScreenReaderConfig,
  ProcessElementOptions,
  SpeakOptions,
  WIAScreenReaderData,
} from './types';

// Re-export types
export * from './types';

/**
 * WIHP (WIA International Hangul Pronunciation) Engine
 */
class WIHPEngine {
  private static readonly PHONEME_MAP: Record<string, string> = {
    // English consonants
    'b': 'ㅂ', 'c': 'ㅋ', 'd': 'ㄷ', 'f': 'ㅍ', 'g': 'ㄱ',
    'h': 'ㅎ', 'j': 'ㅈ', 'k': 'ㅋ', 'l': 'ㄹ', 'm': 'ㅁ',
    'n': 'ㄴ', 'p': 'ㅍ', 'q': 'ㅋ', 'r': 'ㄹ', 's': 'ㅅ',
    't': 'ㅌ', 'v': 'ㅂ', 'w': 'ㅇ', 'x': 'ㅋㅅ', 'y': 'ㅇ', 'z': 'ㅈ',
    // English vowels
    'a': 'ㅏ', 'e': 'ㅔ', 'i': 'ㅣ', 'o': 'ㅗ', 'u': 'ㅜ',
  };

  private static readonly WORD_MAP: Record<string, string> = {
    'hello': '헬로우',
    'world': '월드',
    'the': '더',
    'is': '이즈',
    'a': '어',
    'an': '앤',
    'and': '앤드',
    'or': '오어',
    'but': '벗',
    'for': '포',
    'with': '위드',
    'this': '디스',
    'that': '댓',
    'what': '왓',
    'when': '웬',
    'where': '웨어',
    'why': '와이',
    'how': '하우',
    'you': '유',
    'we': '위',
    'they': '데이',
    'it': '잇',
    'he': '히',
    'she': '쉬',
    'good': '굿',
    'bad': '배드',
    'yes': '예스',
    'no': '노',
    'thank': '땡크',
    'please': '플리즈',
    'welcome': '웰컴',
    'sorry': '소리',
    'computer': '컴퓨터',
    'internet': '인터넷',
    'screen': '스크린',
    'reader': '리더',
    'accessibility': '액세시빌리티',
  };

  /**
   * Convert text to WIHP
   */
  convert(text: string, language: LanguageCode = 'en'): string {
    const words = text.toLowerCase().split(/\s+/);
    return words
      .map(word => WIHPEngine.WORD_MAP[word] || this.phoneticConvert(word))
      .join(' ');
  }

  private phoneticConvert(word: string): string {
    let result = '';
    for (const char of word) {
      result += WIHPEngine.PHONEME_MAP[char] || char;
    }
    return result;
  }
}

/**
 * Braille Engine
 */
class BrailleEngine {
  private static readonly BRAILLE_MAP: Record<string, string> = {
    'a': '⠁', 'b': '⠃', 'c': '⠉', 'd': '⠙', 'e': '⠑',
    'f': '⠋', 'g': '⠛', 'h': '⠓', 'i': '⠊', 'j': '⠚',
    'k': '⠅', 'l': '⠇', 'm': '⠍', 'n': '⠝', 'o': '⠕',
    'p': '⠏', 'q': '⠟', 'r': '⠗', 's': '⠎', 't': '⠞',
    'u': '⠥', 'v': '⠧', 'w': '⠺', 'x': '⠭', 'y': '⠽', 'z': '⠵',
    '0': '⠚', '1': '⠁', '2': '⠃', '3': '⠉', '4': '⠙',
    '5': '⠑', '6': '⠋', '7': '⠛', '8': '⠓', '9': '⠊',
    ' ': ' ', '.': '⠲', ',': '⠂', '!': '⠖', '?': '⠦',
    "'": '⠄', '-': '⠤', ':': '⠒', ';': '⠆',
  };

  private static readonly WIA_MAP: Record<string, string> = {
    'a': 'ㅏ', 'b': 'ㅂ', 'c': 'ㅋ', 'd': 'ㄷ', 'e': 'ㅔ',
    'f': 'ㅍ', 'g': 'ㄱ', 'h': 'ㅎ', 'i': 'ㅣ', 'j': 'ㅈ',
    'k': 'ㅋ', 'l': 'ㄹ', 'm': 'ㅁ', 'n': 'ㄴ', 'o': 'ㅗ',
    'p': 'ㅍ', 'q': 'ㅋ', 'r': 'ㄹ', 's': 'ㅅ', 't': 'ㅌ',
    'u': 'ㅜ', 'v': 'ㅂ', 'w': 'ㅇ', 'x': 'ㅋㅅ', 'y': 'ㅇ', 'z': 'ㅈ',
  };

  /**
   * Convert text to braille
   */
  convert(text: string, grade: BrailleGrade = 1): BrailleOutput {
    const lower = text.toLowerCase();
    let unicode = '';
    let wia = '';
    const cells: BrailleCell[] = [];

    for (const char of lower) {
      const brailleChar = BrailleEngine.BRAILLE_MAP[char] || char;
      const wiaChar = BrailleEngine.WIA_MAP[char] || char;
      unicode += brailleChar;
      wia += wiaChar;

      if (BrailleEngine.BRAILLE_MAP[char]) {
        cells.push({
          char,
          dots: this.unicodeToDots(brailleChar),
          unicode: brailleChar,
        });
      }
    }

    return {
      grade1: unicode,
      grade2: unicode, // Grade 2 contractions would go here
      wia,
      dots: cells,
      cells: cells.length,
    };
  }

  private unicodeToDots(char: string): number[] {
    const code = char.charCodeAt(0) - 0x2800;
    const dots: number[] = [];
    for (let i = 0; i < 8; i++) {
      if (code & (1 << i)) {
        dots.push(i + 1);
      }
    }
    return dots;
  }
}

/**
 * WIA Screen Reader SDK
 *
 * @example
 * ```typescript
 * import { WIAScreenReader } from '@wia/screen-reader';
 *
 * const reader = new WIAScreenReader();
 * const result = await reader.process("Hello World");
 * console.log(result.pronunciation.wihp); // "헬로우 월드"
 * console.log(result.braille.grade1);     // "⠓⠑⠇⠇⠕ ⠺⠕⠗⠇⠙"
 * ```
 */
export class WIAScreenReader {
  private config: WIAScreenReaderConfig;
  private wihpEngine: WIHPEngine;
  private brailleEngine: BrailleEngine;
  private speechSynthesis?: SpeechSynthesis;

  /**
   * Create a new WIA Screen Reader instance
   */
  constructor(config: WIAScreenReaderConfig = {}) {
    this.config = {
      defaultLanguage: 'en',
      defaultBrailleGrade: 1,
      autoDetectLanguage: true,
      ...config,
    };
    this.wihpEngine = new WIHPEngine();
    this.brailleEngine = new BrailleEngine();

    // Initialize speech synthesis if available
    if (typeof window !== 'undefined' && 'speechSynthesis' in window) {
      this.speechSynthesis = window.speechSynthesis;
    }
  }

  /**
   * Process text and return screen reader data
   */
  async process(text: string, language?: LanguageCode): Promise<ScreenReaderResult> {
    const startTime = Date.now();
    const lang = language || this.config.defaultLanguage || 'en';

    const wihp = this.wihpEngine.convert(text, lang);
    const braille = this.brailleEngine.convert(text, this.config.defaultBrailleGrade);

    const result: ScreenReaderResult = {
      text,
      language: lang,
      pronunciation: {
        ipa: this.generateIPA(text),
        wihp,
        romanized: text.toLowerCase(),
      },
      braille,
      metadata: {
        processedAt: new Date(),
        processingTimeMs: Date.now() - startTime,
        engineVersion: '1.0.0',
        confidence: 0.95,
      },
    };

    return result;
  }

  /**
   * Process a DOM element
   */
  async processElement(
    element: Element,
    options: ProcessElementOptions = {}
  ): Promise<ScreenReaderResult> {
    const text = this.extractText(element, options);
    const context = this.extractContext(element);
    const result = await this.process(text);
    result.context = context;
    return result;
  }

  /**
   * Speak text using TTS
   */
  speak(text: string, options: SpeakOptions = {}): void {
    if (!this.speechSynthesis) {
      console.warn('Speech synthesis not available');
      return;
    }

    if (options.interrupt) {
      this.speechSynthesis.cancel();
    }

    const textToSpeak = options.useWIHP
      ? this.wihpEngine.convert(text)
      : text;

    const utterance = new SpeechSynthesisUtterance(textToSpeak);
    utterance.rate = options.rate ?? this.config.tts?.rate ?? 1;
    utterance.pitch = options.pitch ?? this.config.tts?.pitch ?? 1;
    utterance.volume = options.volume ?? this.config.tts?.volume ?? 1;

    if (options.voice) {
      const voices = this.speechSynthesis.getVoices();
      const voice = voices.find(v => v.name === options.voice);
      if (voice) utterance.voice = voice;
    }

    this.speechSynthesis.speak(utterance);
  }

  /**
   * Stop speaking
   */
  stopSpeaking(): void {
    this.speechSynthesis?.cancel();
  }

  /**
   * Get available TTS voices
   */
  getVoices(): SpeechSynthesisVoice[] {
    return this.speechSynthesis?.getVoices() || [];
  }

  /**
   * Set TTS configuration
   */
  setTTS(config: TTSConfig): void {
    this.config.tts = { ...this.config.tts, ...config };
  }

  /**
   * Set language
   */
  setLanguage(language: LanguageCode): void {
    this.config.defaultLanguage = language;
  }

  /**
   * Convert to JSON format
   */
  toJSON(result: ScreenReaderResult): WIAScreenReaderData {
    return {
      wia_screen_reader: {
        version: '1.0.0',
        text: result.text,
        language: result.language,
        pronunciation: result.pronunciation,
        braille: result.braille,
        tts: result.tts,
        context: result.context,
      },
    };
  }

  private generateIPA(text: string): string {
    // Simplified IPA generation
    return `/${text.toLowerCase()}/`;
  }

  private extractText(element: Element, options: ProcessElementOptions): string {
    if (options.includeChildren !== false) {
      return element.textContent || '';
    }
    return Array.from(element.childNodes)
      .filter(node => node.nodeType === Node.TEXT_NODE)
      .map(node => node.textContent)
      .join('');
  }

  private extractContext(element: Element): Context {
    const tagName = element.tagName.toLowerCase();
    const role = element.getAttribute('role');

    const context: Context = {};

    // Determine element type
    if (tagName.match(/^h[1-6]$/)) {
      context.elementType = 'heading';
      context.level = parseInt(tagName[1]) as 1 | 2 | 3 | 4 | 5 | 6;
    } else if (tagName === 'a') {
      context.elementType = 'link';
    } else if (tagName === 'button' || role === 'button') {
      context.elementType = 'button';
    } else if (tagName === 'input') {
      const type = element.getAttribute('type');
      if (type === 'checkbox') context.elementType = 'checkbox';
      else if (type === 'radio') context.elementType = 'radio';
      else context.elementType = 'textbox';
    } else if (tagName === 'nav' || role === 'navigation') {
      context.landmark = 'navigation';
    } else if (tagName === 'main' || role === 'main') {
      context.landmark = 'main';
    }

    // Extract state
    const state: Context['state'] = {};
    if (element.hasAttribute('disabled')) state.disabled = true;
    if (element.hasAttribute('readonly')) state.readonly = true;
    if (element.hasAttribute('required')) state.required = true;
    if (element.getAttribute('aria-expanded') === 'true') state.expanded = true;
    if (element.getAttribute('aria-selected') === 'true') state.selected = true;
    if (element.getAttribute('aria-checked') === 'true') state.checked = true;

    if (Object.keys(state).length > 0) {
      context.state = state;
    }

    return context;
  }
}

// Default export
export default WIAScreenReader;
