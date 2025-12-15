//! Gesture Classification Module
//!
//! This module provides gesture classification for EMG-based
//! myoelectric prosthetic control.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::time::Instant;

/// Supported hand gestures
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Gesture {
    Rest,
    HandOpen,
    HandClose,
    WristFlexion,
    WristExtension,
    WristPronation,
    WristSupination,
    PinchThumbIndex,
    PinchThumbMiddle,
    PointIndex,
    ThumbsUp,
}

impl Gesture {
    /// Get all basic gestures for simple prosthetic control
    pub fn basic_gestures() -> Vec<Gesture> {
        vec![
            Gesture::Rest,
            Gesture::HandOpen,
            Gesture::HandClose,
            Gesture::WristFlexion,
            Gesture::WristExtension,
        ]
    }

    /// Get all supported gestures
    pub fn all_gestures() -> Vec<Gesture> {
        vec![
            Gesture::Rest,
            Gesture::HandOpen,
            Gesture::HandClose,
            Gesture::WristFlexion,
            Gesture::WristExtension,
            Gesture::WristPronation,
            Gesture::WristSupination,
            Gesture::PinchThumbIndex,
            Gesture::PinchThumbMiddle,
            Gesture::PointIndex,
            Gesture::ThumbsUp,
        ]
    }

    /// Convert to string representation
    pub fn as_str(&self) -> &'static str {
        match self {
            Gesture::Rest => "rest",
            Gesture::HandOpen => "hand_open",
            Gesture::HandClose => "hand_close",
            Gesture::WristFlexion => "wrist_flexion",
            Gesture::WristExtension => "wrist_extension",
            Gesture::WristPronation => "wrist_pronation",
            Gesture::WristSupination => "wrist_supination",
            Gesture::PinchThumbIndex => "pinch_thumb_index",
            Gesture::PinchThumbMiddle => "pinch_thumb_middle",
            Gesture::PointIndex => "point_index",
            Gesture::ThumbsUp => "thumbs_up",
        }
    }
}

/// Feature vector for classification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeatureVector {
    /// Timestamp in milliseconds
    pub timestamp: u64,
    /// Number of channels
    pub channel_count: usize,
    /// Feature names
    pub feature_names: Vec<String>,
    /// Feature values
    pub features: Vec<f32>,
}

impl FeatureVector {
    /// Create a new feature vector
    pub fn new(timestamp: u64, features: Vec<f32>, channel_count: usize) -> Self {
        let feature_names = Self::generate_feature_names(channel_count);
        Self {
            timestamp,
            channel_count,
            feature_names,
            features,
        }
    }

    fn generate_feature_names(channels: usize) -> Vec<String> {
        let mut names = Vec::new();
        for ch in 0..channels {
            names.push(format!("ch{}_mav", ch));
            names.push(format!("ch{}_rms", ch));
            names.push(format!("ch{}_wl", ch));
            names.push(format!("ch{}_zc", ch));
            names.push(format!("ch{}_ssc", ch));
        }
        names
    }
}

/// Classification result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ClassificationResult {
    /// Primary gesture
    pub gesture: Gesture,
    /// Confidence score (0.0 - 1.0)
    pub confidence: f32,
    /// Alternative classifications
    pub alternatives: Vec<(Gesture, f32)>,
    /// Classification latency in milliseconds
    pub latency_ms: f32,
    /// Timestamp
    pub timestamp: u64,
}

/// Classifier configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ClassifierConfig {
    /// Confidence threshold for valid classification
    pub confidence_threshold: f32,
    /// Number of frames for majority voting
    pub majority_voting_frames: usize,
    /// Minimum gesture duration in milliseconds
    pub min_gesture_duration_ms: u64,
}

impl Default for ClassifierConfig {
    fn default() -> Self {
        Self {
            confidence_threshold: 0.7,
            majority_voting_frames: 3,
            min_gesture_duration_ms: 100,
        }
    }
}

/// Linear Discriminant Analysis classifier
pub struct LDAClassifier {
    _config: ClassifierConfig,
    weights: Vec<Vec<f32>>,
    grand_mean: Vec<f32>,
    classes: Vec<Gesture>,
    is_trained: bool,
}

impl LDAClassifier {
    /// Create a new LDA classifier
    pub fn new(config: ClassifierConfig) -> Self {
        Self {
            _config: config,
            weights: Vec::new(),
            grand_mean: Vec::new(),
            classes: Vec::new(),
            is_trained: false,
        }
    }

    /// Train the classifier
    pub fn train(
        &mut self,
        samples: &[Vec<f32>],
        labels: &[usize],
        gestures: Vec<Gesture>,
    ) -> Result<TrainingResult, ClassifierError> {
        if samples.is_empty() {
            return Err(ClassifierError::EmptyTrainingSet);
        }

        let start = Instant::now();
        self.classes = gestures;
        let num_features = samples[0].len();
        let num_classes = self.classes.len();

        // Group samples by class
        let mut class_samples: Vec<Vec<&Vec<f32>>> = vec![Vec::new(); num_classes];
        for (sample, &label) in samples.iter().zip(labels.iter()) {
            if label < num_classes {
                class_samples[label].push(sample);
            }
        }

        // Compute class means
        let mut class_means: Vec<Vec<f32>> = Vec::new();
        for class_idx in 0..num_classes {
            let samples_for_class = &class_samples[class_idx];
            if samples_for_class.is_empty() {
                return Err(ClassifierError::EmptyClass(class_idx));
            }

            let mut mean = vec![0.0; num_features];
            for sample in samples_for_class {
                for (i, &val) in sample.iter().enumerate() {
                    mean[i] += val;
                }
            }
            for val in &mut mean {
                *val /= samples_for_class.len() as f32;
            }
            class_means.push(mean);
        }

        // Compute grand mean
        self.grand_mean = vec![0.0; num_features];
        for mean in &class_means {
            for (i, &val) in mean.iter().enumerate() {
                self.grand_mean[i] += val;
            }
        }
        for val in &mut self.grand_mean {
            *val /= num_classes as f32;
        }

        // Compute weights (simplified LDA)
        self.weights = class_means
            .iter()
            .map(|mean| {
                mean.iter()
                    .zip(self.grand_mean.iter())
                    .map(|(m, gm)| m - gm)
                    .collect()
            })
            .collect();

        self.is_trained = true;

        // Compute accuracy
        let mut correct = 0;
        for (sample, &label) in samples.iter().zip(labels.iter()) {
            let features = FeatureVector::new(0, sample.clone(), 4);
            let result = self.classify(&features);
            let predicted = self
                .classes
                .iter()
                .position(|g| *g == result.gesture)
                .unwrap_or(0);
            if predicted == label {
                correct += 1;
            }
        }

        let accuracy = correct as f32 / samples.len() as f32;

        Ok(TrainingResult {
            accuracy,
            training_duration_ms: start.elapsed().as_millis() as f32,
        })
    }

    /// Classify a feature vector
    pub fn classify(&self, features: &FeatureVector) -> ClassificationResult {
        let start = Instant::now();

        if !self.is_trained || self.weights.is_empty() {
            return ClassificationResult {
                gesture: Gesture::Rest,
                confidence: 0.0,
                alternatives: Vec::new(),
                latency_ms: start.elapsed().as_secs_f32() * 1000.0,
                timestamp: features.timestamp,
            };
        }

        // Compute discriminant scores
        let mut scores: Vec<f32> = Vec::new();
        for weights in &self.weights {
            let score: f32 = features
                .features
                .iter()
                .zip(self.grand_mean.iter())
                .zip(weights.iter())
                .map(|((&f, &gm), &w)| w * (f - gm))
                .sum();
            scores.push(score);
        }

        // Softmax
        let max_score = scores.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
        let exp_scores: Vec<f32> = scores.iter().map(|&s| (s - max_score).exp()).collect();
        let sum_exp: f32 = exp_scores.iter().sum();
        let probabilities: Vec<f32> = exp_scores.iter().map(|&e| e / sum_exp).collect();

        // Find best class
        let (best_idx, &max_prob) = probabilities
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .unwrap();

        // Build alternatives
        let mut alternatives: Vec<(Gesture, f32)> = self
            .classes
            .iter()
            .zip(probabilities.iter())
            .filter(|(_, &p)| p != max_prob)
            .map(|(&g, &p)| (g, p))
            .collect();
        alternatives.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
        alternatives.truncate(3);

        ClassificationResult {
            gesture: self.classes[best_idx],
            confidence: max_prob,
            alternatives,
            latency_ms: start.elapsed().as_secs_f32() * 1000.0,
            timestamp: features.timestamp,
        }
    }

    /// Check if classifier is trained
    pub fn is_trained(&self) -> bool {
        self.is_trained
    }

    /// Get supported gestures
    pub fn get_classes(&self) -> &[Gesture] {
        &self.classes
    }
}

/// Training result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrainingResult {
    pub accuracy: f32,
    pub training_duration_ms: f32,
}

/// Classifier errors
#[derive(Debug)]
pub enum ClassifierError {
    EmptyTrainingSet,
    EmptyClass(usize),
    NotTrained,
    InvalidFeatures,
}

impl std::fmt::Display for ClassifierError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ClassifierError::EmptyTrainingSet => write!(f, "Empty training set"),
            ClassifierError::EmptyClass(idx) => write!(f, "Empty class: {}", idx),
            ClassifierError::NotTrained => write!(f, "Classifier not trained"),
            ClassifierError::InvalidFeatures => write!(f, "Invalid features"),
        }
    }
}

impl std::error::Error for ClassifierError {}

/// Majority voting buffer for temporal smoothing
pub struct MajorityVotingBuffer {
    buffer: Vec<ClassificationResult>,
    size: usize,
}

impl MajorityVotingBuffer {
    /// Create a new majority voting buffer
    pub fn new(size: usize) -> Self {
        Self {
            buffer: Vec::with_capacity(size),
            size,
        }
    }

    /// Add a result and get the majority vote
    pub fn add(&mut self, result: ClassificationResult) -> Option<ClassificationResult> {
        self.buffer.push(result);

        if self.buffer.len() > self.size {
            self.buffer.remove(0);
        }

        if self.buffer.len() < self.size {
            return None;
        }

        // Count votes
        let mut votes: HashMap<Gesture, (usize, f32)> = HashMap::new();
        for r in &self.buffer {
            let entry = votes.entry(r.gesture).or_insert((0, 0.0));
            entry.0 += 1;
            entry.1 += r.confidence;
        }

        // Find winner
        let (winner, (count, conf_sum)) = votes
            .iter()
            .max_by_key(|(_, (count, _))| *count)
            .unwrap();

        let avg_confidence = conf_sum / *count as f32;
        let avg_latency: f32 =
            self.buffer.iter().map(|r| r.latency_ms).sum::<f32>() / self.buffer.len() as f32;

        // Build alternatives
        let mut alternatives: Vec<(Gesture, f32)> = votes
            .iter()
            .filter(|(g, _)| *g != winner)
            .map(|(g, (c, s))| (*g, s / *c as f32))
            .collect();
        alternatives.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

        Some(ClassificationResult {
            gesture: *winner,
            confidence: avg_confidence,
            alternatives,
            latency_ms: avg_latency,
            timestamp: self.buffer.last().unwrap().timestamp,
        })
    }

    /// Clear the buffer
    pub fn clear(&mut self) {
        self.buffer.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gesture_basic() {
        let gestures = Gesture::basic_gestures();
        assert_eq!(gestures.len(), 5);
        assert!(gestures.contains(&Gesture::Rest));
    }

    #[test]
    fn test_feature_vector() {
        let features = vec![0.1, 0.2, 0.3, 0.4, 0.5];
        let fv = FeatureVector::new(1234, features.clone(), 1);
        assert_eq!(fv.features, features);
        assert_eq!(fv.channel_count, 1);
    }

    #[test]
    fn test_lda_classifier() {
        let mut classifier = LDAClassifier::new(ClassifierConfig::default());

        // Create simple training data
        let samples = vec![
            vec![0.1, 0.1, 0.1, 0.1],
            vec![0.2, 0.1, 0.1, 0.1],
            vec![0.9, 0.9, 0.9, 0.9],
            vec![0.8, 0.9, 0.9, 0.9],
        ];
        let labels = vec![0, 0, 1, 1];
        let gestures = vec![Gesture::Rest, Gesture::HandClose];

        let result = classifier.train(&samples, &labels, gestures);
        assert!(result.is_ok());
        assert!(classifier.is_trained());
    }

    #[test]
    fn test_majority_voting() {
        let mut buffer = MajorityVotingBuffer::new(3);

        let r1 = ClassificationResult {
            gesture: Gesture::Rest,
            confidence: 0.8,
            alternatives: vec![],
            latency_ms: 1.0,
            timestamp: 1,
        };

        // First two shouldn't return anything
        assert!(buffer.add(r1.clone()).is_none());
        assert!(buffer.add(r1.clone()).is_none());

        // Third should return majority
        let result = buffer.add(r1);
        assert!(result.is_some());
        assert_eq!(result.unwrap().gesture, Gesture::Rest);
    }
}
