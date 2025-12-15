/**
 * WIA BCI TTS Adapter
 *
 * Web Speech API based Text-to-Speech adapter.
 */

import { BaseOutputAdapter } from './IOutputAdapter';
import {
  OutputType,
  OutputContent,
  OutputOptions,
  Voice,
  TTSOptions,
  OutputError,
  OutputErrorCode,
} from './types';

/**
 * TTS Adapter interface
 */
export interface ITTSAdapter {
  getVoices(): Promise<Voice[]>;
  setVoice(voiceId: string): void;
  getVoice(): Voice | undefined;
  speak(text: string, options?: TTSOptions): Promise<void>;
  pause(): void;
  resume(): void;
  stop(): void;
  isSpeaking(): boolean;
  isPaused(): boolean;
  setRate(rate: number): void;
  setPitch(pitch: number): void;
  setVolume(volume: number): void;
}

/**
 * Web Speech API TTS Adapter
 */
export class WebSpeechTTSAdapter extends BaseOutputAdapter implements ITTSAdapter {
  readonly type: OutputType = 'tts';
  readonly name = 'Web Speech TTS';

  private synth: SpeechSynthesis | null = null;
  private voices: SpeechSynthesisVoice[] = [];
  private currentVoice: SpeechSynthesisVoice | null = null;
  private rate = 1.0;
  private pitch = 1.0;
  private volume = 1.0;
  private language = 'ko-KR';

  async initialize(options?: OutputOptions): Promise<void> {
    // Check if Web Speech API is available
    if (typeof window === 'undefined' || !window.speechSynthesis) {
      this.available = false;
      throw new OutputError(
        OutputErrorCode.TTS_NOT_SUPPORTED,
        'Web Speech API not supported',
        'tts'
      );
    }

    this.synth = window.speechSynthesis;

    if (options?.language) {
      this.language = options.language as string;
    }

    // Load voices
    await this.loadVoices();

    // Set default voice for language
    this.selectDefaultVoice();

    this.ready = true;
    this.emit('ready', {});
  }

  private async loadVoices(): Promise<void> {
    return new Promise((resolve) => {
      const loadVoicesInternal = () => {
        this.voices = this.synth!.getVoices();
        if (this.voices.length > 0) {
          resolve();
        }
      };

      loadVoicesInternal();

      if (this.voices.length === 0) {
        this.synth!.onvoiceschanged = () => {
          loadVoicesInternal();
          resolve();
        };
        // Timeout fallback
        setTimeout(resolve, 1000);
      }
    });
  }

  private selectDefaultVoice(): void {
    // Try to find voice matching language
    const langVoice = this.voices.find((v) =>
      v.lang.toLowerCase().startsWith(this.language.split('-')[0])
    );
    if (langVoice) {
      this.currentVoice = langVoice;
      return;
    }

    // Use default voice
    const defaultVoice = this.voices.find((v) => v.default);
    if (defaultVoice) {
      this.currentVoice = defaultVoice;
      return;
    }

    // Use first available
    if (this.voices.length > 0) {
      this.currentVoice = this.voices[0];
    }
  }

  async getVoices(): Promise<Voice[]> {
    return this.voices.map((v) => ({
      id: v.voiceURI,
      name: v.name,
      language: v.lang,
      localService: v.localService,
      default: v.default,
    }));
  }

  setVoice(voiceId: string): void {
    const voice = this.voices.find((v) => v.voiceURI === voiceId);
    if (voice) {
      this.currentVoice = voice;
    } else {
      throw new OutputError(
        OutputErrorCode.TTS_VOICE_NOT_FOUND,
        `Voice not found: ${voiceId}`,
        'tts'
      );
    }
  }

  getVoice(): Voice | undefined {
    if (!this.currentVoice) return undefined;
    return {
      id: this.currentVoice.voiceURI,
      name: this.currentVoice.name,
      language: this.currentVoice.lang,
      localService: this.currentVoice.localService,
      default: this.currentVoice.default,
    };
  }

  async speak(text: string, options?: TTSOptions): Promise<void> {
    if (!this.synth) {
      throw new OutputError(
        OutputErrorCode.TTS_NOT_SUPPORTED,
        'TTS not initialized',
        'tts'
      );
    }

    // Cancel any ongoing speech
    this.synth.cancel();

    const utterance = new SpeechSynthesisUtterance(text);

    // Apply options
    if (options?.voice) {
      const voice = this.voices.find((v) => v.voiceURI === options.voice);
      if (voice) utterance.voice = voice;
    } else if (this.currentVoice) {
      utterance.voice = this.currentVoice;
    }

    utterance.rate = options?.rate ?? this.rate;
    utterance.pitch = options?.pitch ?? this.pitch;
    utterance.volume = options?.volume ?? this.volume;

    if (options?.language) {
      utterance.lang = options.language;
    }

    return new Promise((resolve, reject) => {
      utterance.onstart = () => {
        this.emit('start', {});
      };

      utterance.onend = () => {
        this.emit('end', {});
        resolve();
      };

      utterance.onerror = (event) => {
        const error = new OutputError(
          OutputErrorCode.TTS_SYNTHESIS_FAILED,
          `Speech synthesis failed: ${event.error}`,
          'tts'
        );
        this.emit('error', { error });
        reject(error);
      };

      this.synth!.speak(utterance);
    });
  }

  pause(): void {
    this.synth?.pause();
  }

  resume(): void {
    this.synth?.resume();
  }

  stop(): void {
    this.synth?.cancel();
  }

  isSpeaking(): boolean {
    return this.synth?.speaking ?? false;
  }

  isPaused(): boolean {
    return this.synth?.paused ?? false;
  }

  setRate(rate: number): void {
    this.rate = Math.max(0.1, Math.min(10, rate));
  }

  setPitch(pitch: number): void {
    this.pitch = Math.max(0, Math.min(2, pitch));
  }

  setVolume(volume: number): void {
    this.volume = Math.max(0, Math.min(1, volume));
  }

  async output(content: OutputContent): Promise<void> {
    if (content.type === 'text' && content.text) {
      await this.speak(content.text);
    } else if (content.type === 'classification' && content.classification) {
      // Speak classification result
      const text = `${content.classification.className}, 확신도 ${Math.round(content.classification.confidence * 100)}퍼센트`;
      await this.speak(text);
    }
  }

  async dispose(): Promise<void> {
    this.stop();
    this.synth = null;
    await super.dispose();
  }
}
