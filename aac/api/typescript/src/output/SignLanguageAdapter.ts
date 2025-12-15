/**
 * WIA AAC Sign Language Adapter
 * Phase 4: WIA Ecosystem Integration
 *
 * ISP/WIA Talk integration for sign language output
 */

import { BaseOutputAdapter } from './IOutputAdapter';
import {
  OutputOptions,
  ISPCode,
  OutputError,
  OutputErrorCode
} from './types';

/**
 * Sign language adapter interface
 */
export interface ISignLanguageAdapter {
  /**
   * Convert text to ISP codes
   * @param text - Text to convert
   */
  textToISP(text: string): Promise<ISPCode[]>;

  /**
   * Play a single gesture
   * @param ispCode - ISP code to play
   */
  playGesture(ispCode: ISPCode): Promise<void>;

  /**
   * Play a sequence of gestures
   * @param ispCodes - ISP codes to play
   */
  playSequence(ispCodes: ISPCode[]): Promise<void>;

  /**
   * Set avatar
   * @param avatarId - Avatar ID
   */
  setAvatar(avatarId: string): void;

  /**
   * Set playback speed
   * @param speed - Speed (0.5 ~ 2.0)
   */
  setSpeed(speed: number): void;
}

/**
 * ISP dictionary entry
 */
interface ISPDictionaryEntry {
  word: string;
  code: string;
  duration: number;
}

/**
 * Mock Sign Language Adapter
 * Uses ISP code system for sign language representation
 */
export class MockSignLanguageAdapter extends BaseOutputAdapter implements ISignLanguageAdapter {
  readonly type = 'sign_language' as const;
  readonly name = 'MockSignLanguage';

  private ispDictionary: Map<string, ISPDictionaryEntry> = new Map();
  private speed: number = 1.0;
  private avatarId: string = 'default';
  private isPlaying: boolean = false;

  async initialize(): Promise<void> {
    this.loadDefaultDictionary();
    this._state = 'idle';
    this.emit('start', { message: 'Sign language adapter initialized' });
  }

  /**
   * Load default ISP dictionary
   * Based on WIA Talk 93 core gestures
   */
  private loadDefaultDictionary(): void {
    const defaultEntries: ISPDictionaryEntry[] = [
      // Greetings
      { word: '안녕', code: 'HS01-LC01-MV01-OR01-NM01', duration: 1000 },
      { word: '안녕하세요', code: 'HS01-LC01-MV01-OR01-NM01', duration: 1200 },
      { word: 'hello', code: 'HS01-LC01-MV01-OR01-NM01', duration: 1000 },
      { word: 'hi', code: 'HS01-LC01-MV01-OR01-NM01', duration: 800 },

      // Thanks
      { word: '감사', code: 'HS02-LC07-MV02-OR02-NM02', duration: 1000 },
      { word: '감사합니다', code: 'HS02-LC07-MV02-OR02-NM02', duration: 1200 },
      { word: 'thank', code: 'HS02-LC07-MV02-OR02-NM02', duration: 1000 },
      { word: 'thanks', code: 'HS02-LC07-MV02-OR02-NM02', duration: 1000 },

      // Yes/No
      { word: '예', code: 'HS03-LC01-MV03-OR01-NM03', duration: 600 },
      { word: '네', code: 'HS03-LC01-MV03-OR01-NM03', duration: 600 },
      { word: 'yes', code: 'HS03-LC01-MV03-OR01-NM03', duration: 600 },
      { word: '아니오', code: 'HS04-LC01-MV04-OR01-NM04', duration: 800 },
      { word: '아니', code: 'HS04-LC01-MV04-OR01-NM04', duration: 600 },
      { word: 'no', code: 'HS04-LC01-MV04-OR01-NM04', duration: 600 },

      // Love
      { word: '사랑', code: 'HS09-LC07-MV10-OR02-NM01', duration: 1200 },
      { word: '사랑해', code: 'HS09-LC07-MV10-OR02-NM01', duration: 1200 },
      { word: 'love', code: 'HS09-LC07-MV10-OR02-NM01', duration: 1000 },

      // Help
      { word: '도움', code: 'HS05-LC08-MV07-OR03-NM05', duration: 1000 },
      { word: '도와주세요', code: 'HS05-LC08-MV07-OR03-NM05', duration: 1200 },
      { word: 'help', code: 'HS05-LC08-MV07-OR03-NM05', duration: 800 },

      // Basic words
      { word: '나', code: 'HS06-LC07-MV10-OR01-NM01', duration: 600 },
      { word: '너', code: 'HS07-LC02-MV10-OR02-NM01', duration: 600 },
      { word: 'I', code: 'HS06-LC07-MV10-OR01-NM01', duration: 500 },
      { word: 'you', code: 'HS07-LC02-MV10-OR02-NM01', duration: 500 },

      // Actions
      { word: '먹다', code: 'HS08-LC03-MV05-OR01-NM01', duration: 800 },
      { word: '마시다', code: 'HS08-LC03-MV06-OR01-NM01', duration: 800 },
      { word: '가다', code: 'HS10-LC12-MV08-OR03-NM01', duration: 800 },
      { word: '오다', code: 'HS10-LC12-MV09-OR04-NM01', duration: 800 },

      // Questions
      { word: '뭐', code: 'HS11-LC04-MV11-OR01-NM06', duration: 700 },
      { word: '왜', code: 'HS12-LC04-MV11-OR01-NM06', duration: 700 },
      { word: '어디', code: 'HS13-LC04-MV12-OR01-NM06', duration: 800 },
      { word: '언제', code: 'HS14-LC04-MV12-OR01-NM06', duration: 800 },

      // Emotions
      { word: '좋아', code: 'HS15-LC07-MV10-OR02-NM07', duration: 800 },
      { word: '싫어', code: 'HS16-LC07-MV10-OR02-NM08', duration: 800 },
      { word: '기쁘다', code: 'HS15-LC07-MV10-OR02-NM09', duration: 900 },
      { word: '슬프다', code: 'HS16-LC07-MV10-OR02-NM10', duration: 900 },

      // Common phrases
      { word: '미안', code: 'HS17-LC07-MV13-OR01-NM11', duration: 1000 },
      { word: '미안합니다', code: 'HS17-LC07-MV13-OR01-NM11', duration: 1200 },
      { word: 'sorry', code: 'HS17-LC07-MV13-OR01-NM11', duration: 800 },

      // Understanding
      { word: '알겠어', code: 'HS18-LC07-MV14-OR01-NM12', duration: 900 },
      { word: '이해했어', code: 'HS18-LC07-MV15-OR06-NM23', duration: 1000 },
      { word: 'understand', code: 'HS18-LC07-MV15-OR06-NM23', duration: 900 }
    ];

    for (const entry of defaultEntries) {
      this.ispDictionary.set(entry.word.toLowerCase(), entry);
    }
  }

  /**
   * Parse ISP code into components
   */
  private parseISPCode(code: string): ISPCode['components'] | undefined {
    const match = code.match(/^(HS\d+)-(LC\d+)-(MV\d+)-(OR\d+)-(NM\d+)$/);
    if (!match) return undefined;

    return {
      handshape: match[1],
      location: match[2],
      movement: match[3],
      orientation: match[4],
      nonManual: match[5]
    };
  }

  async textToISP(text: string): Promise<ISPCode[]> {
    // Split text into words
    const words = text.toLowerCase().split(/\s+/).filter(w => w.length > 0);
    const codes: ISPCode[] = [];

    for (const word of words) {
      const entry = this.ispDictionary.get(word);
      if (entry) {
        codes.push({
          code: entry.code,
          meaning: entry.word,
          duration: entry.duration,
          components: this.parseISPCode(entry.code)
        });
      } else {
        // Unknown word - use default gesture with fingerspelling placeholder
        codes.push({
          code: 'HS00-LC00-MV00-OR00-NM00',
          meaning: word,
          duration: word.length * 200 + 400, // Fingerspelling time
          components: this.parseISPCode('HS00-LC00-MV00-OR00-NM00')
        });
      }
    }

    return codes;
  }

  async playGesture(ispCode: ISPCode): Promise<void> {
    if (this._state === 'outputting') {
      throw new OutputError(
        OutputErrorCode.ADAPTER_BUSY,
        'Sign language adapter is busy',
        true
      );
    }

    this._state = 'outputting';
    this.isPlaying = true;

    const duration = (ispCode.duration || 1000) / this.speed;

    this.emit('start', {
      code: ispCode.code,
      meaning: ispCode.meaning,
      avatarId: this.avatarId
    });

    // Simulate gesture playback
    await new Promise<void>((resolve, reject) => {
      const timer = setTimeout(() => {
        if (this.isPlaying) {
          this._state = 'idle';
          this.isPlaying = false;
          this.emit('end', { code: ispCode.code });
          resolve();
        }
      }, duration);

      // Allow cancellation
      if (!this.isPlaying) {
        clearTimeout(timer);
        reject(new OutputError(
          OutputErrorCode.OUTPUT_CANCELLED,
          'Gesture cancelled',
          true
        ));
      }
    });
  }

  async playSequence(ispCodes: ISPCode[]): Promise<void> {
    for (let i = 0; i < ispCodes.length; i++) {
      if (!this.isPlaying && this._state !== 'idle') {
        break; // Stopped
      }

      const code = ispCodes[i];
      this.emit('progress', {
        current: i + 1,
        total: ispCodes.length,
        code: code.code
      });

      await this.playGesture(code);

      // Small pause between gestures
      if (i < ispCodes.length - 1) {
        await new Promise(resolve => setTimeout(resolve, 100 / this.speed));
      }
    }
  }

  async output(text: string, options?: OutputOptions): Promise<void> {
    if (options?.speed) {
      this.speed = options.speed;
    }

    const codes = await this.textToISP(text);

    if (codes.length === 0) {
      return;
    }

    await this.playSequence(codes);
  }

  setAvatar(avatarId: string): void {
    this.avatarId = avatarId;
  }

  setSpeed(speed: number): void {
    this.speed = Math.max(0.5, Math.min(2.0, speed));
  }

  stop(): void {
    this.isPlaying = false;
    this._state = 'idle';
  }

  isAvailable(): boolean {
    return true;
  }

  async dispose(): Promise<void> {
    this.stop();
    this.ispDictionary.clear();
    this.handlers.clear();
  }

  /**
   * Add custom ISP mapping
   */
  addMapping(word: string, code: string, duration: number = 1000): void {
    this.ispDictionary.set(word.toLowerCase(), {
      word,
      code,
      duration
    });
  }

  /**
   * Get all mappings
   */
  getMappings(): Map<string, ISPDictionaryEntry> {
    return new Map(this.ispDictionary);
  }
}
