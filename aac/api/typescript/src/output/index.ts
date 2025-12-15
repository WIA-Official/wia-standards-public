/**
 * WIA AAC Output Module
 * Phase 4: WIA Ecosystem Integration
 *
 * Provides output adapters for TTS, Sign Language, and Braille
 */

// Types
export * from './types';

// Interfaces
export { IOutputAdapter, BaseOutputAdapter } from './IOutputAdapter';

// TTS Adapters
export {
  ITTSAdapter,
  WebSpeechTTSAdapter,
  MockTTSAdapter
} from './TTSAdapter';

// Sign Language Adapters
export {
  ISignLanguageAdapter,
  MockSignLanguageAdapter
} from './SignLanguageAdapter';

// Braille Adapters
export {
  IBrailleAdapter,
  MockBrailleAdapter
} from './BrailleAdapter';

// Output Manager
export {
  IOutputManager,
  OutputManager,
  OutputManagerEventType,
  OutputManagerEventHandler
} from './OutputManager';
