/**
 * WIA Emotion AI - Basic TypeScript Example
 *
 * This example demonstrates basic usage of the WIA Emotion AI SDK.
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

import { EmotionClient, EmotionCategory } from '@wia/emotion-ai';

// Initialize client
const client = new EmotionClient({ apiKey: 'your-api-key' });

// Example 1: Analyze facial expression in an image
async function analyzeImage(): Promise<void> {
    const result = await client.analyzeFacialImage('photo.jpg', {
        detectActionUnits: true,
        culturalContext: 'ko-KR'
    });

    console.log(`Request ID: ${result.requestId}`);
    console.log(`Processing time: ${result.processingTimeMs}ms`);

    for (const emotion of result.emotions) {
        console.log(`  ${emotion.category}: ${emotion.intensity.toFixed(2)} (confidence: ${emotion.confidence.toFixed(2)})`);
    }

    if (result.dimensional) {
        console.log(`Valence: ${result.dimensional.valence.toFixed(2)}`);
        console.log(`Arousal: ${result.dimensional.arousal.toFixed(2)}`);
    }
}

// Example 2: Analyze voice emotion
async function analyzeVoice(): Promise<void> {
    const result = await client.analyzeVoice('speech.wav', {
        language: 'ko-KR',
        extractProsody: true
    });

    console.log(`Dominant emotion: ${result.emotions[0].category}`);
    console.log(`Segments analyzed: ${result.segments?.length || 0}`);
}

// Example 3: Analyze text sentiment
async function analyzeText(): Promise<void> {
    const result = await client.analyzeText(
        '오늘 정말 좋은 하루였어요!',
        { language: 'ko', detectSarcasm: true }
    );

    console.log(`Emotion: ${result.emotions[0].category}`);
    if (result.dimensional) {
        console.log(`Valence: ${result.dimensional.valence.toFixed(2)}`);
    }
}

// Example 4: Multimodal analysis
async function analyzeMultimodal(): Promise<void> {
    const result = await client.analyzeMultimodal(
        {
            facial: { image: 'photo.jpg' },
            voice: { audio: 'speech.wav', format: 'wav' },
            text: { content: "I'm very happy today!" }
        },
        {
            fusionStrategy: 'late_fusion',
            modalityWeights: {
                facial: 0.4,
                voice: 0.35,
                text: 0.25
            }
        }
    );

    console.log(`Fused emotion: ${result.emotions[0].category}`);
}

// Example 5: Real-time streaming
async function streamEmotions(): Promise<void> {
    const stream = client.createStream({
        modalities: ['facial', 'voice'],
        frameRate: 15
    });

    stream.on('emotion', (event) => {
        const emotion = event.emotions[0];
        console.log(`Emotion: ${emotion.category} (${emotion.intensity.toFixed(2)})`);
    });

    stream.on('aggregate', (summary) => {
        console.log(`Dominant: ${summary.aggregate.dominantEmotion}`);
        console.log(`Engagement: ${summary.aggregate.engagementScore.toFixed(2)}`);
    });

    stream.on('error', (error) => {
        console.error('Stream error:', error);
    });

    await stream.start();

    // In real app: stream.sendFrame(videoFrame);
    // await new Promise(resolve => setTimeout(resolve, 60000));

    await stream.stop();
}

// Main
async function main(): Promise<void> {
    console.log('=== Image Analysis ===');
    // await analyzeImage();

    console.log('\n=== Voice Analysis ===');
    // await analyzeVoice();

    console.log('\n=== Text Analysis ===');
    // await analyzeText();

    console.log('\n=== Multimodal Analysis ===');
    // await analyzeMultimodal();

    console.log('\n=== Streaming ===');
    // await streamEmotions();

    console.log('\nNote: Uncomment the function calls and provide valid API key and files.');
}

main().catch(console.error);
