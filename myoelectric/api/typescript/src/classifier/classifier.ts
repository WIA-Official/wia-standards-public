/**
 * WIA Myoelectric Gesture Classifier
 * @module classifier/classifier
 */

import { Observable, Subject, BehaviorSubject } from 'rxjs';
import { map, filter, bufferCount, scan } from 'rxjs/operators';
import {
  Gesture,
  FeatureVector,
  ClassificationResult,
  TrainingDataset,
  TrainingResult,
  LabeledSample,
  ClassifierConfig,
  ClassificationAlgorithm,
  ModelInfo,
  AlternativeClassification,
} from './types';

/**
 * Abstract base class for gesture classifiers
 */
export abstract class GestureClassifier {
  protected config: ClassifierConfig;
  protected modelInfo: ModelInfo | null = null;
  protected isLoaded: boolean = false;

  constructor(config: Partial<ClassifierConfig> = {}) {
    this.config = {
      algorithm: config.algorithm ?? ClassificationAlgorithm.LDA,
      confidenceThreshold: config.confidenceThreshold ?? 0.7,
      majorityVotingFrames: config.majorityVotingFrames ?? 3,
      minGestureDurationMs: config.minGestureDurationMs ?? 100,
    };
  }

  /**
   * Load a trained model from file
   */
  abstract loadModel(path: string): Promise<void>;

  /**
   * Save the current model to file
   */
  abstract saveModel(path: string): Promise<void>;

  /**
   * Classify a single feature vector
   */
  abstract classify(features: FeatureVector): ClassificationResult;

  /**
   * Train the classifier on a dataset
   */
  abstract train(dataset: TrainingDataset): Promise<TrainingResult>;

  /**
   * Fine-tune the model with additional samples
   */
  abstract finetune(samples: LabeledSample[]): Promise<void>;

  /**
   * Get model information
   */
  getModelInfo(): ModelInfo | null {
    return this.modelInfo;
  }

  /**
   * Check if model is loaded
   */
  isModelLoaded(): boolean {
    return this.isLoaded;
  }

  /**
   * Classify a stream of feature vectors with majority voting
   */
  classifyStream(
    stream: Observable<FeatureVector>
  ): Observable<ClassificationResult> {
    return stream.pipe(
      map((features) => this.classify(features)),
      bufferCount(this.config.majorityVotingFrames, 1),
      map((results) => this.majorityVote(results)),
      filter((result) => result.confidence >= this.config.confidenceThreshold)
    );
  }

  /**
   * Apply majority voting to a buffer of results
   */
  protected majorityVote(results: ClassificationResult[]): ClassificationResult {
    const voteCounts = new Map<Gesture, number>();
    const confidenceSums = new Map<Gesture, number>();

    for (const result of results) {
      const currentCount = voteCounts.get(result.gesture) ?? 0;
      const currentConf = confidenceSums.get(result.gesture) ?? 0;
      voteCounts.set(result.gesture, currentCount + 1);
      confidenceSums.set(result.gesture, currentConf + result.confidence);
    }

    let maxVotes = 0;
    let winningGesture = Gesture.REST;

    for (const [gesture, votes] of voteCounts) {
      if (votes > maxVotes) {
        maxVotes = votes;
        winningGesture = gesture;
      }
    }

    const avgConfidence =
      (confidenceSums.get(winningGesture) ?? 0) / maxVotes;
    const avgLatency =
      results.reduce((sum, r) => sum + r.latencyMs, 0) / results.length;

    // Build alternatives from other gestures
    const alternatives: AlternativeClassification[] = [];
    for (const [gesture, votes] of voteCounts) {
      if (gesture !== winningGesture) {
        alternatives.push({
          gesture,
          confidence: (confidenceSums.get(gesture) ?? 0) / votes,
        });
      }
    }

    alternatives.sort((a, b) => b.confidence - a.confidence);

    return {
      gesture: winningGesture,
      confidence: avgConfidence,
      alternatives,
      latencyMs: avgLatency,
      timestamp: results[results.length - 1].timestamp,
    };
  }
}

/**
 * Linear Discriminant Analysis classifier
 */
export class LDAClassifier extends GestureClassifier {
  private weights: number[][] = [];
  private means: number[][] = [];
  private classes: Gesture[] = [];
  private grandMean: number[] = [];

  constructor(config: Partial<ClassifierConfig> = {}) {
    super({ ...config, algorithm: ClassificationAlgorithm.LDA });
  }

  async loadModel(path: string): Promise<void> {
    // In a real implementation, load from file
    // For now, just mark as loaded
    console.log(`Loading LDA model from ${path}`);
    this.isLoaded = true;
  }

  async saveModel(path: string): Promise<void> {
    if (!this.isLoaded) {
      throw new Error('No model to save');
    }
    console.log(`Saving LDA model to ${path}`);
  }

  classify(features: FeatureVector): ClassificationResult {
    const startTime = performance.now();

    if (!this.isLoaded || this.weights.length === 0) {
      return this.defaultResult(startTime);
    }

    const x = features.features;
    const scores: number[] = [];

    // Compute discriminant scores for each class
    for (let i = 0; i < this.classes.length; i++) {
      let score = 0;
      for (let j = 0; j < x.length; j++) {
        score += this.weights[i][j] * (x[j] - this.grandMean[j]);
      }
      scores.push(score);
    }

    // Softmax to get probabilities
    const maxScore = Math.max(...scores);
    const expScores = scores.map((s) => Math.exp(s - maxScore));
    const sumExp = expScores.reduce((a, b) => a + b, 0);
    const probabilities = expScores.map((e) => e / sumExp);

    // Find best class
    let maxProb = 0;
    let bestIdx = 0;
    for (let i = 0; i < probabilities.length; i++) {
      if (probabilities[i] > maxProb) {
        maxProb = probabilities[i];
        bestIdx = i;
      }
    }

    // Build alternatives
    const alternatives: AlternativeClassification[] = [];
    for (let i = 0; i < this.classes.length; i++) {
      if (i !== bestIdx) {
        alternatives.push({
          gesture: this.classes[i],
          confidence: probabilities[i],
        });
      }
    }
    alternatives.sort((a, b) => b.confidence - a.confidence);

    return {
      gesture: this.classes[bestIdx],
      confidence: maxProb,
      alternatives,
      latencyMs: performance.now() - startTime,
      timestamp: features.timestamp,
    };
  }

  async train(dataset: TrainingDataset): Promise<TrainingResult> {
    const startTime = Date.now();

    // Group samples by class
    const classSamples = new Map<Gesture, number[][]>();
    for (const sample of dataset.samples) {
      const existing = classSamples.get(sample.label) ?? [];
      existing.push(sample.features.features);
      classSamples.set(sample.label, existing);
    }

    this.classes = Array.from(classSamples.keys());
    const numFeatures = dataset.samples[0].features.features.length;

    // Compute class means
    this.means = [];
    for (const gesture of this.classes) {
      const samples = classSamples.get(gesture)!;
      const mean = new Array(numFeatures).fill(0);
      for (const sample of samples) {
        for (let i = 0; i < numFeatures; i++) {
          mean[i] += sample[i];
        }
      }
      for (let i = 0; i < numFeatures; i++) {
        mean[i] /= samples.length;
      }
      this.means.push(mean);
    }

    // Compute grand mean
    this.grandMean = new Array(numFeatures).fill(0);
    for (const mean of this.means) {
      for (let i = 0; i < numFeatures; i++) {
        this.grandMean[i] += mean[i];
      }
    }
    for (let i = 0; i < numFeatures; i++) {
      this.grandMean[i] /= this.classes.length;
    }

    // Simplified LDA: use class means as weights
    this.weights = this.means.map((mean) =>
      mean.map((v, i) => v - this.grandMean[i])
    );

    this.isLoaded = true;

    // Compute training accuracy
    let correct = 0;
    for (const sample of dataset.samples) {
      const result = this.classify(sample.features);
      if (result.gesture === sample.label) {
        correct++;
      }
    }

    const accuracy = correct / dataset.samples.length;

    return {
      success: true,
      accuracy,
      gestureMetrics: new Map(),
      confusionMatrix: [],
      trainingDurationSec: (Date.now() - startTime) / 1000,
      modelSizeBytes: JSON.stringify(this.weights).length,
    };
  }

  async finetune(samples: LabeledSample[]): Promise<void> {
    // Simple fine-tuning: update means with new samples
    for (const sample of samples) {
      const classIdx = this.classes.indexOf(sample.label);
      if (classIdx >= 0) {
        const x = sample.features.features;
        for (let i = 0; i < x.length; i++) {
          this.means[classIdx][i] =
            0.9 * this.means[classIdx][i] + 0.1 * x[i];
        }
      }
    }

    // Update weights
    this.weights = this.means.map((mean) =>
      mean.map((v, i) => v - this.grandMean[i])
    );
  }

  private defaultResult(startTime: number): ClassificationResult {
    return {
      gesture: Gesture.REST,
      confidence: 0,
      alternatives: [],
      latencyMs: performance.now() - startTime,
      timestamp: Date.now(),
    };
  }
}

/**
 * K-Nearest Neighbors classifier
 */
export class KNNClassifier extends GestureClassifier {
  private k: number = 5;
  private trainingSamples: LabeledSample[] = [];

  constructor(config: Partial<ClassifierConfig> = {}, k: number = 5) {
    super({ ...config, algorithm: ClassificationAlgorithm.KNN });
    this.k = k;
  }

  async loadModel(path: string): Promise<void> {
    console.log(`Loading KNN model from ${path}`);
    this.isLoaded = true;
  }

  async saveModel(path: string): Promise<void> {
    console.log(`Saving KNN model to ${path}`);
  }

  classify(features: FeatureVector): ClassificationResult {
    const startTime = performance.now();

    if (this.trainingSamples.length === 0) {
      return {
        gesture: Gesture.REST,
        confidence: 0,
        alternatives: [],
        latencyMs: performance.now() - startTime,
        timestamp: features.timestamp,
      };
    }

    // Compute distances to all training samples
    const distances: { distance: number; label: Gesture }[] = [];
    for (const sample of this.trainingSamples) {
      const dist = this.euclideanDistance(
        features.features,
        sample.features.features
      );
      distances.push({ distance: dist, label: sample.label });
    }

    // Sort by distance and take k nearest
    distances.sort((a, b) => a.distance - b.distance);
    const nearest = distances.slice(0, this.k);

    // Vote among k nearest
    const votes = new Map<Gesture, number>();
    for (const neighbor of nearest) {
      const current = votes.get(neighbor.label) ?? 0;
      votes.set(neighbor.label, current + 1);
    }

    // Find winner
    let maxVotes = 0;
    let winner = Gesture.REST;
    for (const [gesture, count] of votes) {
      if (count > maxVotes) {
        maxVotes = count;
        winner = gesture;
      }
    }

    const confidence = maxVotes / this.k;

    // Build alternatives
    const alternatives: AlternativeClassification[] = [];
    for (const [gesture, count] of votes) {
      if (gesture !== winner) {
        alternatives.push({ gesture, confidence: count / this.k });
      }
    }
    alternatives.sort((a, b) => b.confidence - a.confidence);

    return {
      gesture: winner,
      confidence,
      alternatives,
      latencyMs: performance.now() - startTime,
      timestamp: features.timestamp,
    };
  }

  async train(dataset: TrainingDataset): Promise<TrainingResult> {
    const startTime = Date.now();
    this.trainingSamples = [...dataset.samples];
    this.isLoaded = true;

    // Compute accuracy using leave-one-out
    let correct = 0;
    for (let i = 0; i < dataset.samples.length; i++) {
      const test = dataset.samples[i];
      const tempSamples = this.trainingSamples;
      this.trainingSamples = [
        ...dataset.samples.slice(0, i),
        ...dataset.samples.slice(i + 1),
      ];
      const result = this.classify(test.features);
      if (result.gesture === test.label) {
        correct++;
      }
      this.trainingSamples = tempSamples;
    }

    return {
      success: true,
      accuracy: correct / dataset.samples.length,
      gestureMetrics: new Map(),
      confusionMatrix: [],
      trainingDurationSec: (Date.now() - startTime) / 1000,
      modelSizeBytes: JSON.stringify(this.trainingSamples).length,
    };
  }

  async finetune(samples: LabeledSample[]): Promise<void> {
    this.trainingSamples.push(...samples);
  }

  private euclideanDistance(a: number[], b: number[]): number {
    let sum = 0;
    for (let i = 0; i < a.length; i++) {
      const diff = a[i] - b[i];
      sum += diff * diff;
    }
    return Math.sqrt(sum);
  }
}

/**
 * Factory function to create a classifier
 */
export function createClassifier(
  algorithm: ClassificationAlgorithm,
  config: Partial<ClassifierConfig> = {}
): GestureClassifier {
  switch (algorithm) {
    case ClassificationAlgorithm.LDA:
      return new LDAClassifier(config);
    case ClassificationAlgorithm.KNN:
      return new KNNClassifier(config);
    default:
      throw new Error(`Unsupported algorithm: ${algorithm}`);
  }
}
