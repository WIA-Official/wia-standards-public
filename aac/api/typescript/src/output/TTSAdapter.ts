/**
 * WIA AAC TTS Adapter
 * Phase 4: WIA Ecosystem Integration
 *
 * Text-to-Speech adapter using Web Speech API
 */

import { BaseOutputAdapter } from './IOutputAdapter';
import {
  OutputOptions,
  OutputState,
  Voice,
  OutputError,
  OutputErrorCode
} from './types';

/**
 * TTS adapter interface
 */
export interface ITTSAdapter {
  /**
   * Get available voices
   */
  getVoices(): Promise<Voice[]>;

  /**
   * Set voice
   * @param voiceId - Voice ID
   */
  setVoice(voiceId: string): void;

  /**
   * Pause output
   */
  pause(): void;

  /**
   * Resume output
   */
  resume(): void;
}

/**
 * Web Speech API TTS Adapter
 */
export class WebSpeechTTSAdapter extends BaseOutputAdapter implements ITTSAdapter {
  readonly type = 'tts' as const;
  readonly name = 'WebSpeechTTS';

  private synth: SpeechSynthesis | null = null;
  private currentUtterance: SpeechSynthesisUtterance | null = null;
  private selectedVoice: SpeechSynthesisVoice | null = null;
  private defaultOptions: OutputOptions = {
    speed: 1.0,
    volume: 1.0
  };

  async initialize(options?: OutputOptions): Promise<void> {
    if (typeof window === 'undefined' || !window.speechSynthesis) {
      throw new OutputError(
        OutputErrorCode.NOT_AVAILABLE,
        'Web Speech API not available',
        false
      );
    }

    this.synth = window.speechSynthesis;
    if (options) {
      this.defaultOptions = { ...this.defaultOptions, ...options };
    }

    // Wait for voices to load
    await this.waitForVoices();

    this._state = 'idle';
    this.emit('start', { message: 'TTS adapter initialized' });
  }

  private async waitForVoices(): Promise<void> {
    return new Promise((resolve) => {
      const voices = this.synth!.getVoices();
      if (voices.length > 0) {
        resolve();
        return;
      }

      this.synth!.onvoiceschanged = () => {
        resolve();
      };

      // Timeout fallback
      setTimeout(resolve, 1000);
    });
  }

  async getVoices(): Promise<Voice[]> {
    if (!this.synth) {
      throw new OutputError(
        OutputErrorCode.ADAPTER_NOT_READY,
        'TTS adapter not initialized',
        true
      );
    }

    const voices = this.synth.getVoices();
    return voices.map(v => ({
      id: v.voiceURI,
      name: v.name,
      language: v.lang,
      local: v.localService
    }));
  }

  setVoice(voiceId: string): void {
    if (!this.synth) {
      throw new OutputError(
        OutputErrorCode.ADAPTER_NOT_READY,
        'TTS adapter not initialized',
        true
      );
    }

    const voices = this.synth.getVoices();
    const voice = voices.find(v => v.voiceURI === voiceId);
    if (voice) {
      this.selectedVoice = voice;
    }
  }

  async output(text: string, options?: OutputOptions): Promise<void> {
    if (!this.synth) {
      throw new OutputError(
        OutputErrorCode.ADAPTER_NOT_READY,
        'TTS adapter not initialized',
        true
      );
    }

    if (this._state === 'outputting') {
      this.stop();
    }

    const mergedOptions = { ...this.defaultOptions, ...options };

    return new Promise((resolve, reject) => {
      const utterance = new SpeechSynthesisUtterance(text);

      if (mergedOptions.language) {
        utterance.lang = mergedOptions.language;
      }
      if (mergedOptions.speed !== undefined) {
        utterance.rate = mergedOptions.speed;
      }
      if (mergedOptions.volume !== undefined) {
        utterance.volume = mergedOptions.volume;
      }
      if (this.selectedVoice) {
        utterance.voice = this.selectedVoice;
      }

      utterance.onstart = () => {
        this._state = 'outputting';
        this.emit('start', { text });
      };

      utterance.onend = () => {
        this._state = 'idle';
        this.emit('end', { text });
        resolve();
      };

      utterance.onerror = (event) => {
        this._state = 'error';
        const error = new OutputError(
          OutputErrorCode.OUTPUT_FAILED,
          `TTS error: ${event.error}`,
          true,
          event
        );
        this.emit('error', error);
        reject(error);
      };

      utterance.onpause = () => {
        this._state = 'paused';
        this.emit('pause');
      };

      utterance.onresume = () => {
        this._state = 'outputting';
        this.emit('resume');
      };

      this.currentUtterance = utterance;
      this.synth!.speak(utterance);
    });
  }

  stop(): void {
    if (this.synth) {
      this.synth.cancel();
      this._state = 'idle';
      this.currentUtterance = null;
    }
  }

  pause(): void {
    if (this.synth && this._state === 'outputting') {
      this.synth.pause();
      this._state = 'paused';
    }
  }

  resume(): void {
    if (this.synth && this._state === 'paused') {
      this.synth.resume();
      this._state = 'outputting';
    }
  }

  isAvailable(): boolean {
    return typeof window !== 'undefined' && !!window.speechSynthesis;
  }

  async dispose(): Promise<void> {
    this.stop();
    this.synth = null;
    this.selectedVoice = null;
    this.handlers.clear();
  }
}

/**
 * Mock TTS Adapter for testing
 */
export class MockTTSAdapter extends BaseOutputAdapter implements ITTSAdapter {
  readonly type = 'tts' as const;
  readonly name = 'MockTTS';

  private mockVoices: Voice[] = [
    { id: 'mock-ko', name: 'Mock Korean', language: 'ko-KR', gender: 'neutral', local: true },
    { id: 'mock-en', name: 'Mock English', language: 'en-US', gender: 'neutral', local: true }
  ];

  private selectedVoiceId: string | null = null;

  async initialize(): Promise<void> {
    this._state = 'idle';
    this.emit('start', { message: 'Mock TTS adapter initialized' });
  }

  async getVoices(): Promise<Voice[]> {
    return this.mockVoices;
  }

  setVoice(voiceId: string): void {
    this.selectedVoiceId = voiceId;
  }

  async output(text: string, options?: OutputOptions): Promise<void> {
    this._state = 'outputting';
    this.emit('start', { text });

    // Simulate TTS duration based on text length
    const duration = Math.max(text.length * 50, 500);
    const speed = options?.speed || 1.0;

    await new Promise(resolve => setTimeout(resolve, duration / speed));

    this._state = 'idle';
    this.emit('end', { text });
  }

  stop(): void {
    this._state = 'idle';
  }

  pause(): void {
    if (this._state === 'outputting') {
      this._state = 'paused';
      this.emit('pause');
    }
  }

  resume(): void {
    if (this._state === 'paused') {
      this._state = 'outputting';
      this.emit('resume');
    }
  }

  isAvailable(): boolean {
    return true;
  }

  async dispose(): Promise<void> {
    this.handlers.clear();
  }
}
