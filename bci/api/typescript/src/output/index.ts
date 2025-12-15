/**
 * WIA BCI Output Module
 *
 * Phase 4: Ecosystem Integration
 */

// Types
export * from './types';

// Interface
export { IOutputAdapter, BaseOutputAdapter } from './IOutputAdapter';

// TTS Adapter
export { WebSpeechTTSAdapter, ITTSAdapter } from './TTSAdapter';

// Mock Adapters
export {
  MockSignLanguageAdapter,
  MockBrailleAdapter,
  MockOutputAdapter,
} from './MockOutputAdapter';

// Neurofeedback Adapter
export {
  CanvasNeurofeedbackAdapter,
  INeurofeedbackAdapter,
} from './NeurofeedbackAdapter';

// Output Manager
export { OutputManager } from './OutputManager';
