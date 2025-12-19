/**
 * WIA Emotion AI SDK for TypeScript
 *
 * Affective Computing / Emotion Recognition Standards
 *
 * @example
 * ```typescript
 * import { EmotionClient } from '@wia/emotion-ai';
 *
 * const client = new EmotionClient({ apiKey: 'your-api-key' });
 * const result = await client.analyzeFacialImage('photo.jpg');
 * console.log(result.emotions[0].category);
 * ```
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

export { EmotionClient, EmotionClientConfig } from './client';
export { EmotionStream, StreamConfig, StreamEventType } from './stream';
export {
    EmotionEvent,
    Emotion,
    EmotionCategory,
    ActionUnit,
    DimensionalModel,
    Modality,
    ModalityType,
    AnalysisResult
} from './models';
