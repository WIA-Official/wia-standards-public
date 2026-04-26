# Chapter 5: Detection Protocols

## AI Content Detection Methodologies

This chapter details the technical approaches for detecting AI-generated content across different media types, including deepfake detection, synthetic image analysis, and text generation identification.

---

## Detection Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Multi-Modal Detection Pipeline                    │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  Input Content                                                       │
│       │                                                              │
│       ▼                                                              │
│  ┌─────────────────┐                                                 │
│  │ Content Router  │──── Image ────▶ Image Detection Pipeline       │
│  │                 │──── Video ────▶ Video Detection Pipeline       │
│  │                 │──── Audio ────▶ Audio Detection Pipeline       │
│  │                 │──── Text  ────▶ Text Detection Pipeline        │
│  └─────────────────┘                                                 │
│                                                                      │
│  Detection Pipelines:                                                │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │ Image Pipeline                                               │    │
│  │ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐        │    │
│  │ │Frequency │ │CNN-based │ │CLIP      │ │Artifact  │        │    │
│  │ │Analysis  │ │Detector  │ │Embedding │ │Detection │        │    │
│  │ └──────────┘ └──────────┘ └──────────┘ └──────────┘        │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                                                                      │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │ Video Pipeline                                               │    │
│  │ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐        │    │
│  │ │Face      │ │Temporal  │ │Lip-Sync  │ │Physio-   │        │    │
│  │ │Forensics │ │Coherence │ │Analysis  │ │logical   │        │    │
│  │ └──────────┘ └──────────┘ └──────────┘ └──────────┘        │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                                                                      │
│       │                                                              │
│       ▼                                                              │
│  ┌─────────────────┐                                                 │
│  │   Aggregator    │                                                 │
│  │ (Ensemble Vote) │                                                 │
│  └─────────────────┘                                                 │
│       │                                                              │
│       ▼                                                              │
│  Detection Result                                                    │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Image Detection Models

### CNN-Based Detection

```typescript
// WIA Content AI - Image Detection Models
// Deep learning approaches for synthetic image detection

import * as tf from '@tensorflow/tfjs-node';

/**
 * CNN-based Synthetic Image Detector
 * EfficientNet backbone with custom classification head
 */
class CNNImageDetector {
  private model: tf.LayersModel;
  private preprocessor: ImagePreprocessor;
  private config: CNNDetectorConfig;

  constructor(config: CNNDetectorConfig) {
    this.config = config;
    this.preprocessor = new ImagePreprocessor(config.inputSize);
  }

  /**
   * Load pre-trained detection model
   */
  async loadModel(modelPath: string): Promise<void> {
    this.model = await tf.loadLayersModel(modelPath);
    console.log("Model loaded:", this.model.summary());
  }

  /**
   * Detect synthetic image
   */
  async detect(imageBuffer: Buffer): Promise<ImageDetectionResult> {
    // Preprocess image
    const tensor = await this.preprocessor.process(imageBuffer);

    // Run inference
    const prediction = this.model.predict(tensor) as tf.Tensor;
    const scores = await prediction.data();

    // Get class probabilities
    const syntheticScore = scores[1];  // Assuming binary [real, synthetic]
    const realScore = scores[0];

    // Generate activation map for explainability
    const activationMap = await this.generateActivationMap(tensor);

    // Cleanup tensors
    tensor.dispose();
    prediction.dispose();

    return {
      modelId: "cnn-efficientnet-v2",
      syntheticProbability: syntheticScore,
      realProbability: realScore,
      prediction: syntheticScore > this.config.threshold ? "SYNTHETIC" : "REAL",
      confidence: Math.max(syntheticScore, realScore),
      activationMap,
      processingTime: 0  // Set by caller
    };
  }

  /**
   * Batch detection for multiple images
   */
  async detectBatch(imageBuffers: Buffer[]): Promise<ImageDetectionResult[]> {
    const tensors = await Promise.all(
      imageBuffers.map(buf => this.preprocessor.process(buf))
    );

    const batchTensor = tf.stack(tensors);
    const predictions = this.model.predict(batchTensor) as tf.Tensor;
    const allScores = await predictions.array() as number[][];

    // Cleanup
    tensors.forEach(t => t.dispose());
    batchTensor.dispose();
    predictions.dispose();

    return allScores.map(scores => ({
      modelId: "cnn-efficientnet-v2",
      syntheticProbability: scores[1],
      realProbability: scores[0],
      prediction: scores[1] > this.config.threshold ? "SYNTHETIC" : "REAL",
      confidence: Math.max(scores[0], scores[1])
    }));
  }

  /**
   * Generate Grad-CAM activation map
   */
  private async generateActivationMap(inputTensor: tf.Tensor): Promise<number[][]> {
    // Implementation of Grad-CAM for explainability
    const lastConvLayer = this.model.getLayer('top_conv');
    const classifierLayer = this.model.getLayer('predictions');

    // Create gradient model
    const gradModel = tf.model({
      inputs: this.model.inputs,
      outputs: [lastConvLayer.output, classifierLayer.output]
    });

    // Compute gradients
    const [convOutputs, predictions] = gradModel.predict(inputTensor) as tf.Tensor[];

    // Generate heatmap
    const heatmap = await this.computeHeatmap(convOutputs, predictions);

    convOutputs.dispose();
    predictions.dispose();

    return heatmap;
  }

  private async computeHeatmap(
    convOutputs: tf.Tensor,
    predictions: tf.Tensor
  ): Promise<number[][]> {
    // Grad-CAM computation
    return [];
  }
}

interface CNNDetectorConfig {
  inputSize: [number, number];
  threshold: number;
  modelVersion: string;
}

interface ImageDetectionResult {
  modelId: string;
  syntheticProbability: number;
  realProbability: number;
  prediction: "SYNTHETIC" | "REAL";
  confidence: number;
  activationMap?: number[][];
  processingTime?: number;
}

class ImagePreprocessor {
  private size: [number, number];

  constructor(size: [number, number]) {
    this.size = size;
  }

  async process(buffer: Buffer): Promise<tf.Tensor> {
    const decoded = tf.node.decodeImage(buffer, 3);
    const resized = tf.image.resizeBilinear(decoded, this.size);
    const normalized = resized.div(255.0);
    decoded.dispose();
    resized.dispose();
    return normalized.expandDims(0);
  }
}
```

### Frequency Domain Analysis

```typescript
/**
 * Frequency Domain Detector
 * Analyzes DCT and FFT artifacts in synthetic images
 */
class FrequencyDomainDetector {
  private fftSize: number;
  private spectralAnalyzer: SpectralAnalyzer;

  constructor(config: FrequencyConfig) {
    this.fftSize = config.fftSize;
    this.spectralAnalyzer = new SpectralAnalyzer();
  }

  /**
   * Detect based on frequency characteristics
   */
  async detect(imageBuffer: Buffer): Promise<FrequencyDetectionResult> {
    // Convert to grayscale
    const grayscale = await this.toGrayscale(imageBuffer);

    // Compute 2D FFT
    const fftResult = await this.compute2DFFT(grayscale);

    // Analyze spectrum
    const spectralFeatures = this.analyzeSpectrum(fftResult);

    // Check for GAN artifacts
    const ganArtifacts = this.detectGANArtifacts(fftResult);

    // Check for diffusion model patterns
    const diffusionPatterns = this.detectDiffusionPatterns(fftResult);

    // Compute anomaly score
    const anomalyScore = this.computeAnomalyScore(
      spectralFeatures,
      ganArtifacts,
      diffusionPatterns
    );

    return {
      modelId: "frequency-domain-v1",
      anomalyScore,
      spectralFeatures,
      ganArtifactScore: ganArtifacts.score,
      diffusionScore: diffusionPatterns.score,
      prediction: anomalyScore > 0.5 ? "SYNTHETIC" : "REAL",
      confidence: Math.abs(anomalyScore - 0.5) * 2 + 0.5
    };
  }

  /**
   * Analyze frequency spectrum for anomalies
   */
  private analyzeSpectrum(fft: ComplexArray): SpectralFeatures {
    const magnitude = this.computeMagnitude(fft);
    const azimuthalAverage = this.computeAzimuthalAverage(magnitude);

    return {
      // High-frequency energy ratio
      highFreqRatio: this.computeHighFreqRatio(magnitude),

      // Spectral centroid
      spectralCentroid: this.computeSpectralCentroid(magnitude),

      // Spectral flatness (indicates uniformity)
      spectralFlatness: this.computeSpectralFlatness(magnitude),

      // Azimuthal consistency (GANs often show radial patterns)
      azimuthalConsistency: this.computeAzimuthalConsistency(azimuthalAverage),

      // Peak detection
      spectralPeaks: this.detectSpectralPeaks(magnitude),

      // DCT grid artifacts (JPEG vs synthetic)
      dctGridScore: this.analyzeDCTGrid(magnitude)
    };
  }

  /**
   * Detect GAN-specific frequency artifacts
   */
  private detectGANArtifacts(fft: ComplexArray): GANArtifactAnalysis {
    // GANs often produce characteristic grid patterns in frequency domain
    const gridPattern = this.detectGridPattern(fft);

    // Check for spectral peaks at regular intervals
    const regularPeaks = this.detectRegularPeaks(fft);

    // Analyze mid-frequency anomalies
    const midFreqAnomalies = this.analyzeMidFrequencies(fft);

    return {
      score: (gridPattern.score + regularPeaks.score + midFreqAnomalies.score) / 3,
      gridPatternDetected: gridPattern.detected,
      peakLocations: regularPeaks.locations,
      midFreqDeviation: midFreqAnomalies.deviation
    };
  }

  /**
   * Detect diffusion model patterns
   */
  private detectDiffusionPatterns(fft: ComplexArray): DiffusionPatternAnalysis {
    // Diffusion models have different frequency signatures than GANs
    const noisePattern = this.analyzeNoisePattern(fft);
    const highFreqRolloff = this.analyzeHighFreqRolloff(fft);

    return {
      score: (noisePattern.score + highFreqRolloff.score) / 2,
      noiseCharacteristics: noisePattern,
      rolloffProfile: highFreqRolloff
    };
  }

  private async toGrayscale(buffer: Buffer): Promise<number[][]> {
    return [];
  }

  private async compute2DFFT(image: number[][]): Promise<ComplexArray> {
    return { real: [], imag: [] };
  }

  private computeMagnitude(fft: ComplexArray): number[][] {
    return [];
  }

  private computeAzimuthalAverage(magnitude: number[][]): number[] {
    return [];
  }

  private computeHighFreqRatio(magnitude: number[][]): number {
    return 0;
  }

  private computeSpectralCentroid(magnitude: number[][]): number {
    return 0;
  }

  private computeSpectralFlatness(magnitude: number[][]): number {
    return 0;
  }

  private computeAzimuthalConsistency(azimuthal: number[]): number {
    return 0;
  }

  private detectSpectralPeaks(magnitude: number[][]): SpectralPeak[] {
    return [];
  }

  private analyzeDCTGrid(magnitude: number[][]): number {
    return 0;
  }

  private detectGridPattern(fft: ComplexArray): { score: number; detected: boolean } {
    return { score: 0, detected: false };
  }

  private detectRegularPeaks(fft: ComplexArray): { score: number; locations: number[][] } {
    return { score: 0, locations: [] };
  }

  private analyzeMidFrequencies(fft: ComplexArray): { score: number; deviation: number } {
    return { score: 0, deviation: 0 };
  }

  private analyzeNoisePattern(fft: ComplexArray): { score: number } {
    return { score: 0 };
  }

  private analyzeHighFreqRolloff(fft: ComplexArray): { score: number } {
    return { score: 0 };
  }

  private computeAnomalyScore(
    spectral: SpectralFeatures,
    gan: GANArtifactAnalysis,
    diffusion: DiffusionPatternAnalysis
  ): number {
    return 0;
  }
}

interface FrequencyConfig {
  fftSize: number;
}

interface ComplexArray {
  real: number[][];
  imag: number[][];
}

interface SpectralFeatures {
  highFreqRatio: number;
  spectralCentroid: number;
  spectralFlatness: number;
  azimuthalConsistency: number;
  spectralPeaks: SpectralPeak[];
  dctGridScore: number;
}

interface SpectralPeak {
  frequency: [number, number];
  magnitude: number;
}

interface GANArtifactAnalysis {
  score: number;
  gridPatternDetected: boolean;
  peakLocations: number[][];
  midFreqDeviation: number;
}

interface DiffusionPatternAnalysis {
  score: number;
  noiseCharacteristics: any;
  rolloffProfile: any;
}

interface FrequencyDetectionResult {
  modelId: string;
  anomalyScore: number;
  spectralFeatures: SpectralFeatures;
  ganArtifactScore: number;
  diffusionScore: number;
  prediction: "SYNTHETIC" | "REAL";
  confidence: number;
}

class SpectralAnalyzer {}
```

---

## Video Detection (Deepfakes)

```typescript
/**
 * Deepfake Video Detector
 * Multi-modal analysis for manipulated video detection
 */
class DeepfakeDetector {
  private faceDetector: FaceDetector;
  private temporalAnalyzer: TemporalConsistencyAnalyzer;
  private lipSyncAnalyzer: LipSyncAnalyzer;
  private physiologicalAnalyzer: PhysiologicalSignalAnalyzer;
  private audioVisualAnalyzer: AudioVisualSyncAnalyzer;

  constructor(config: DeepfakeDetectorConfig) {
    this.faceDetector = new FaceDetector(config.face);
    this.temporalAnalyzer = new TemporalConsistencyAnalyzer(config.temporal);
    this.lipSyncAnalyzer = new LipSyncAnalyzer(config.lipSync);
    this.physiologicalAnalyzer = new PhysiologicalSignalAnalyzer(config.physiological);
    this.audioVisualAnalyzer = new AudioVisualSyncAnalyzer(config.audioVisual);
  }

  /**
   * Analyze video for deepfake manipulation
   */
  async analyzeVideo(videoPath: string): Promise<DeepfakeAnalysisResult> {
    // Extract frames and audio
    const { frames, audio, metadata } = await this.extractMedia(videoPath);

    // Detect faces in frames
    const faceDetections = await this.detectFaces(frames);

    // Run analysis pipelines in parallel
    const [
      temporalResult,
      lipSyncResult,
      physiologicalResult,
      audioVisualResult
    ] = await Promise.all([
      this.analyzeTemporalConsistency(frames, faceDetections),
      this.analyzeLipSync(frames, audio, faceDetections),
      this.analyzePhysiologicalSignals(frames, faceDetections),
      this.analyzeAudioVisualSync(frames, audio, faceDetections)
    ]);

    // Aggregate results
    const aggregatedScore = this.aggregateScores({
      temporal: temporalResult,
      lipSync: lipSyncResult,
      physiological: physiologicalResult,
      audioVisual: audioVisualResult
    });

    // Identify manipulation regions
    const manipulatedRegions = this.identifyManipulatedRegions(
      faceDetections,
      temporalResult,
      lipSyncResult
    );

    return {
      isDeepfake: aggregatedScore > 0.5,
      confidence: aggregatedScore,
      analysisResults: {
        temporal: temporalResult,
        lipSync: lipSyncResult,
        physiological: physiologicalResult,
        audioVisual: audioVisualResult
      },
      manipulatedRegions,
      frameAnalysis: this.summarizeFrameAnalysis(faceDetections),
      metadata: {
        duration: metadata.duration,
        frameCount: frames.length,
        resolution: metadata.resolution
      }
    };
  }

  /**
   * Analyze temporal consistency across frames
   */
  private async analyzeTemporalConsistency(
    frames: VideoFrame[],
    faces: FaceDetection[][]
  ): Promise<TemporalAnalysisResult> {
    const results: FrameConsistencyScore[] = [];

    for (let i = 1; i < frames.length; i++) {
      const prevFace = faces[i - 1][0];
      const currFace = faces[i][0];

      if (!prevFace || !currFace) continue;

      // Landmark consistency
      const landmarkConsistency = this.computeLandmarkConsistency(
        prevFace.landmarks,
        currFace.landmarks
      );

      // Texture consistency
      const textureConsistency = await this.computeTextureConsistency(
        frames[i - 1],
        frames[i],
        prevFace,
        currFace
      );

      // Boundary consistency
      const boundaryConsistency = this.computeBoundaryConsistency(
        prevFace,
        currFace
      );

      results.push({
        frameIndex: i,
        landmarkScore: landmarkConsistency,
        textureScore: textureConsistency,
        boundaryScore: boundaryConsistency,
        overallScore: (landmarkConsistency + textureConsistency + boundaryConsistency) / 3
      });
    }

    // Compute statistics
    const scores = results.map(r => r.overallScore);
    const avgScore = scores.reduce((a, b) => a + b, 0) / scores.length;
    const variance = scores.reduce((sum, s) => sum + Math.pow(s - avgScore, 2), 0) / scores.length;

    return {
      averageConsistency: avgScore,
      consistencyVariance: variance,
      anomalousFrames: results.filter(r => r.overallScore < 0.5),
      prediction: avgScore < 0.7 ? "MANIPULATED" : "AUTHENTIC",
      confidence: Math.abs(avgScore - 0.5) * 2
    };
  }

  /**
   * Analyze lip-audio synchronization
   */
  private async analyzeLipSync(
    frames: VideoFrame[],
    audio: AudioData,
    faces: FaceDetection[][]
  ): Promise<LipSyncAnalysisResult> {
    // Extract lip regions from frames
    const lipRegions = faces.map(frameFaces =>
      frameFaces[0]?.landmarks?.mouth || null
    );

    // Extract audio features (MFCC, visemes)
    const audioFeatures = await this.extractAudioFeatures(audio);

    // Compute expected lip movements from audio
    const expectedLipMovements = this.computeExpectedLipMovements(audioFeatures);

    // Compare with actual lip movements
    const actualLipMovements = this.extractLipMovements(lipRegions);

    // Compute synchronization score
    const syncScores: number[] = [];
    for (let i = 0; i < Math.min(expectedLipMovements.length, actualLipMovements.length); i++) {
      const score = this.computeSyncScore(expectedLipMovements[i], actualLipMovements[i]);
      syncScores.push(score);
    }

    const avgSyncScore = syncScores.reduce((a, b) => a + b, 0) / syncScores.length;

    // Detect async segments
    const asyncSegments = this.detectAsyncSegments(syncScores, frames.length);

    return {
      averageSyncScore: avgSyncScore,
      syncTimeline: syncScores,
      asyncSegments,
      prediction: avgSyncScore < 0.6 ? "MANIPULATED" : "AUTHENTIC",
      confidence: Math.abs(avgSyncScore - 0.5) * 2
    };
  }

  /**
   * Analyze physiological signals (blinking, pulse)
   */
  private async analyzePhysiologicalSignals(
    frames: VideoFrame[],
    faces: FaceDetection[][]
  ): Promise<PhysiologicalAnalysisResult> {
    // Extract eye regions
    const eyeRegions = faces.map(frameFaces => ({
      leftEye: frameFaces[0]?.landmarks?.leftEye,
      rightEye: frameFaces[0]?.landmarks?.rightEye
    }));

    // Detect blinks
    const blinkPattern = this.detectBlinkPattern(eyeRegions);

    // Analyze blink rate (normal: 15-20 per minute)
    const blinkRate = this.computeBlinkRate(blinkPattern, frames.length);

    // Check blink naturalness
    const blinkNaturalness = this.assessBlinkNaturalness(blinkPattern);

    // Extract remote PPG signal (pulse from skin color changes)
    const ppgSignal = await this.extractRemotePPG(frames, faces);
    const ppgQuality = this.assessPPGQuality(ppgSignal);

    return {
      blinkRate,
      blinkNaturalness,
      ppgSignalQuality: ppgQuality,
      prediction: (blinkNaturalness < 0.5 || ppgQuality < 0.3) ? "MANIPULATED" : "AUTHENTIC",
      confidence: (blinkNaturalness + ppgQuality) / 2
    };
  }

  // Helper methods (stubs)
  private async extractMedia(path: string): Promise<{ frames: VideoFrame[]; audio: AudioData; metadata: any }> {
    return { frames: [], audio: {} as AudioData, metadata: {} };
  }

  private async detectFaces(frames: VideoFrame[]): Promise<FaceDetection[][]> {
    return [];
  }

  private computeLandmarkConsistency(prev: any, curr: any): number {
    return 0;
  }

  private async computeTextureConsistency(prev: VideoFrame, curr: VideoFrame, pf: FaceDetection, cf: FaceDetection): Promise<number> {
    return 0;
  }

  private computeBoundaryConsistency(prev: FaceDetection, curr: FaceDetection): number {
    return 0;
  }

  private async extractAudioFeatures(audio: AudioData): Promise<any[]> {
    return [];
  }

  private computeExpectedLipMovements(features: any[]): any[] {
    return [];
  }

  private extractLipMovements(regions: any[]): any[] {
    return [];
  }

  private computeSyncScore(expected: any, actual: any): number {
    return 0;
  }

  private detectAsyncSegments(scores: number[], frameCount: number): TimeSegment[] {
    return [];
  }

  private detectBlinkPattern(eyes: any[]): BlinkEvent[] {
    return [];
  }

  private computeBlinkRate(blinks: BlinkEvent[], frameCount: number): number {
    return 0;
  }

  private assessBlinkNaturalness(blinks: BlinkEvent[]): number {
    return 0;
  }

  private async extractRemotePPG(frames: VideoFrame[], faces: FaceDetection[][]): Promise<number[]> {
    return [];
  }

  private assessPPGQuality(signal: number[]): number {
    return 0;
  }

  private async analyzeAudioVisualSync(frames: VideoFrame[], audio: AudioData, faces: FaceDetection[][]): Promise<any> {
    return {};
  }

  private aggregateScores(results: any): number {
    return 0;
  }

  private identifyManipulatedRegions(faces: FaceDetection[][], temporal: any, lipSync: any): ManipulatedRegion[] {
    return [];
  }

  private summarizeFrameAnalysis(faces: FaceDetection[][]): any {
    return {};
  }
}

// Types
interface VideoFrame {
  index: number;
  timestamp: number;
  data: Buffer;
}

interface AudioData {
  samples: Float32Array;
  sampleRate: number;
  channels: number;
}

interface FaceDetection {
  boundingBox: { x: number; y: number; width: number; height: number };
  confidence: number;
  landmarks: FaceLandmarks;
}

interface FaceLandmarks {
  leftEye: number[];
  rightEye: number[];
  nose: number[];
  mouth: number[];
  jawline: number[];
}

interface DeepfakeAnalysisResult {
  isDeepfake: boolean;
  confidence: number;
  analysisResults: {
    temporal: TemporalAnalysisResult;
    lipSync: LipSyncAnalysisResult;
    physiological: PhysiologicalAnalysisResult;
    audioVisual: any;
  };
  manipulatedRegions: ManipulatedRegion[];
  frameAnalysis: any;
  metadata: any;
}

interface TemporalAnalysisResult {
  averageConsistency: number;
  consistencyVariance: number;
  anomalousFrames: FrameConsistencyScore[];
  prediction: string;
  confidence: number;
}

interface FrameConsistencyScore {
  frameIndex: number;
  landmarkScore: number;
  textureScore: number;
  boundaryScore: number;
  overallScore: number;
}

interface LipSyncAnalysisResult {
  averageSyncScore: number;
  syncTimeline: number[];
  asyncSegments: TimeSegment[];
  prediction: string;
  confidence: number;
}

interface TimeSegment {
  start: number;
  end: number;
  score: number;
}

interface PhysiologicalAnalysisResult {
  blinkRate: number;
  blinkNaturalness: number;
  ppgSignalQuality: number;
  prediction: string;
  confidence: number;
}

interface BlinkEvent {
  frameStart: number;
  frameEnd: number;
  duration: number;
}

interface ManipulatedRegion {
  type: string;
  frames: [number, number];
  confidence: number;
  description: string;
}

interface DeepfakeDetectorConfig {
  face: any;
  temporal: any;
  lipSync: any;
  physiological: any;
  audioVisual: any;
}

class FaceDetector {
  constructor(config: any) {}
}

class TemporalConsistencyAnalyzer {
  constructor(config: any) {}
}

class LipSyncAnalyzer {
  constructor(config: any) {}
}

class PhysiologicalSignalAnalyzer {
  constructor(config: any) {}
}

class AudioVisualSyncAnalyzer {
  constructor(config: any) {}
}
```

---

## Text Detection

```typescript
/**
 * AI Text Detection
 * Detection of LLM-generated text content
 */
class AITextDetector {
  private perplexityModel: PerplexityAnalyzer;
  private stylisticAnalyzer: StylisticAnalyzer;
  private zeroShotDetector: ZeroShotDetector;

  constructor(config: TextDetectorConfig) {
    this.perplexityModel = new PerplexityAnalyzer(config.perplexity);
    this.stylisticAnalyzer = new StylisticAnalyzer(config.stylistic);
    this.zeroShotDetector = new ZeroShotDetector(config.zeroShot);
  }

  /**
   * Detect AI-generated text
   */
  async detect(text: string): Promise<TextDetectionResult> {
    // Perplexity-based detection
    const perplexityResult = await this.perplexityModel.analyze(text);

    // Stylistic analysis
    const stylisticResult = await this.stylisticAnalyzer.analyze(text);

    // Zero-shot classification
    const zeroShotResult = await this.zeroShotDetector.classify(text);

    // Aggregate scores
    const aggregatedScore = this.aggregateScores(
      perplexityResult,
      stylisticResult,
      zeroShotResult
    );

    return {
      isAIGenerated: aggregatedScore > 0.5,
      confidence: aggregatedScore,
      perplexityScore: perplexityResult.score,
      burstinessScore: perplexityResult.burstiness,
      stylisticFeatures: stylisticResult.features,
      sentenceAnalysis: this.analyzeSentences(text),
      prediction: aggregatedScore > 0.5 ? "AI_GENERATED" : "HUMAN_WRITTEN"
    };
  }

  private analyzeSentences(text: string): SentenceAnalysis[] {
    const sentences = text.split(/[.!?]+/).filter(s => s.trim());

    return sentences.map((sentence, index) => ({
      index,
      length: sentence.length,
      perplexity: 0,  // Computed per sentence
      aiProbability: 0
    }));
  }

  private aggregateScores(
    perplexity: PerplexityResult,
    stylistic: StylisticResult,
    zeroShot: ZeroShotResult
  ): number {
    return (
      perplexity.score * 0.4 +
      stylistic.score * 0.3 +
      zeroShot.score * 0.3
    );
  }
}

interface TextDetectorConfig {
  perplexity: any;
  stylistic: any;
  zeroShot: any;
}

interface TextDetectionResult {
  isAIGenerated: boolean;
  confidence: number;
  perplexityScore: number;
  burstinessScore: number;
  stylisticFeatures: any;
  sentenceAnalysis: SentenceAnalysis[];
  prediction: string;
}

interface SentenceAnalysis {
  index: number;
  length: number;
  perplexity: number;
  aiProbability: number;
}

interface PerplexityResult {
  score: number;
  burstiness: number;
}

interface StylisticResult {
  score: number;
  features: any;
}

interface ZeroShotResult {
  score: number;
}

class PerplexityAnalyzer {
  constructor(config: any) {}
  async analyze(text: string): Promise<PerplexityResult> {
    return { score: 0, burstiness: 0 };
  }
}

class StylisticAnalyzer {
  constructor(config: any) {}
  async analyze(text: string): Promise<StylisticResult> {
    return { score: 0, features: {} };
  }
}

class ZeroShotDetector {
  constructor(config: any) {}
  async classify(text: string): Promise<ZeroShotResult> {
    return { score: 0 };
  }
}
```

---

## Summary

| Detection Type | Key Techniques | Accuracy Range |
|---------------|----------------|----------------|
| **Image** | CNN, Frequency analysis, CLIP | 85-95% |
| **Video** | Temporal, Lip-sync, Physiological | 80-92% |
| **Audio** | Spectral, Voice biometrics | 82-94% |
| **Text** | Perplexity, Stylistic, Zero-shot | 70-85% |

---

**Next Chapter:** [Chapter 6: Integration](./06-integration.md) - Platform and tool integration patterns.

---

© 2025 World Industry Association (WIA). All rights reserved.
弘益人間 (홍익인간) · Benefit All Humanity
