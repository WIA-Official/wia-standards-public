//! Real-time Processing Module
//!
//! This module provides real-time EMG processing and gesture
//! classification for prosthetic control.

use std::collections::VecDeque;
use std::time::Instant;

use crate::classifier::{
    ClassificationResult, FeatureVector, Gesture, LDAClassifier,
    MajorityVotingBuffer,
};
use crate::signal::features::{
    mean_absolute_value, root_mean_square, slope_sign_changes, waveform_length, zero_crossings,
};

/// Real-time processing configuration
#[derive(Debug, Clone)]
pub struct RealtimeConfig {
    /// Sample rate in Hz
    pub sample_rate: u32,
    /// Window size in milliseconds
    pub window_size_ms: u32,
    /// Window overlap percentage (0-99)
    pub overlap_percent: f32,
    /// Number of EMG channels
    pub channel_count: usize,
    /// Enable majority voting
    pub enable_voting: bool,
    /// Voting window size
    pub voting_window: usize,
}

impl Default for RealtimeConfig {
    fn default() -> Self {
        Self {
            sample_rate: 1000,
            window_size_ms: 200,
            overlap_percent: 50.0,
            channel_count: 4,
            enable_voting: true,
            voting_window: 3,
        }
    }
}

/// EMG sample from hardware
#[derive(Debug, Clone)]
pub struct EMGSample {
    /// Timestamp in microseconds
    pub timestamp_us: u64,
    /// Channel values
    pub channels: Vec<f32>,
}

/// Real-time EMG processor
pub struct RealtimeProcessor {
    config: RealtimeConfig,
    classifier: LDAClassifier,
    sample_buffer: VecDeque<EMGSample>,
    voting_buffer: MajorityVotingBuffer,
    window_samples: usize,
    hop_samples: usize,
    samples_since_last_window: usize,
    last_result: Option<ClassificationResult>,
}

impl RealtimeProcessor {
    /// Create a new real-time processor
    pub fn new(config: RealtimeConfig, classifier: LDAClassifier) -> Self {
        let window_samples =
            (config.window_size_ms as f32 * config.sample_rate as f32 / 1000.0) as usize;
        let hop_samples =
            (window_samples as f32 * (1.0 - config.overlap_percent / 100.0)) as usize;

        Self {
            voting_buffer: MajorityVotingBuffer::new(config.voting_window),
            config,
            classifier,
            sample_buffer: VecDeque::with_capacity(window_samples),
            window_samples,
            hop_samples,
            samples_since_last_window: 0,
            last_result: None,
        }
    }

    /// Process a single EMG sample
    ///
    /// Returns a classification result when enough samples have been collected
    pub fn process_sample(&mut self, sample: EMGSample) -> Option<ClassificationResult> {
        self.sample_buffer.push_back(sample);
        self.samples_since_last_window += 1;

        // Remove old samples to maintain window size
        while self.sample_buffer.len() > self.window_samples {
            self.sample_buffer.pop_front();
        }

        // Check if we have enough samples and hop interval has passed
        if self.sample_buffer.len() >= self.window_samples
            && self.samples_since_last_window >= self.hop_samples
        {
            self.samples_since_last_window = 0;
            let features = self.extract_features();
            let result = self.classifier.classify(&features);

            if self.config.enable_voting {
                if let Some(voted_result) = self.voting_buffer.add(result) {
                    self.last_result = Some(voted_result.clone());
                    return Some(voted_result);
                }
            } else {
                self.last_result = Some(result.clone());
                return Some(result);
            }
        }

        None
    }

    /// Process multiple samples
    pub fn process_samples(&mut self, samples: &[EMGSample]) -> Vec<ClassificationResult> {
        samples
            .iter()
            .filter_map(|s| self.process_sample(s.clone()))
            .collect()
    }

    /// Extract features from the current window
    fn extract_features(&self) -> FeatureVector {
        let window: Vec<&EMGSample> = self.sample_buffer.iter().collect();
        let timestamp = window.last().map(|s| s.timestamp_us / 1000).unwrap_or(0);

        let mut features = Vec::new();

        for ch in 0..self.config.channel_count {
            let channel_data: Vec<f32> = window.iter().map(|s| s.channels[ch]).collect();

            // MAV
            features.push(mean_absolute_value(&channel_data));

            // RMS
            features.push(root_mean_square(&channel_data));

            // WL
            features.push(waveform_length(&channel_data));

            // ZC
            features.push(zero_crossings(&channel_data, 0.01) as f32);

            // SSC
            features.push(slope_sign_changes(&channel_data, 0.0001) as f32);
        }

        FeatureVector::new(timestamp, features, self.config.channel_count)
    }

    /// Get the last classification result
    pub fn get_last_result(&self) -> Option<&ClassificationResult> {
        self.last_result.as_ref()
    }

    /// Reset the processor state
    pub fn reset(&mut self) {
        self.sample_buffer.clear();
        self.voting_buffer.clear();
        self.samples_since_last_window = 0;
        self.last_result = None;
    }

    /// Get current configuration
    pub fn config(&self) -> &RealtimeConfig {
        &self.config
    }
}

/// Gesture event for prosthetic control
#[derive(Debug, Clone)]
pub enum GestureEvent {
    /// Gesture started
    Start { gesture: Gesture, timestamp: u64 },
    /// Gesture ended
    End {
        gesture: Gesture,
        timestamp: u64,
        duration_ms: u64,
    },
    /// Gesture changed
    Change {
        from: Gesture,
        to: Gesture,
        timestamp: u64,
    },
}

/// Gesture event detector
pub struct GestureEventDetector {
    last_gesture: Gesture,
    gesture_start_time: u64,
    min_duration_ms: u64,
}

impl GestureEventDetector {
    /// Create a new gesture event detector
    pub fn new(min_duration_ms: u64) -> Self {
        Self {
            last_gesture: Gesture::Rest,
            gesture_start_time: 0,
            min_duration_ms,
        }
    }

    /// Process a classification result and detect events
    pub fn process(&mut self, result: &ClassificationResult) -> Option<GestureEvent> {
        if result.gesture != self.last_gesture {
            let duration = result.timestamp.saturating_sub(self.gesture_start_time);

            // Only emit event if previous gesture was long enough
            let event = if self.last_gesture != Gesture::Rest && duration >= self.min_duration_ms {
                Some(GestureEvent::Change {
                    from: self.last_gesture,
                    to: result.gesture,
                    timestamp: result.timestamp,
                })
            } else if result.gesture != Gesture::Rest {
                Some(GestureEvent::Start {
                    gesture: result.gesture,
                    timestamp: result.timestamp,
                })
            } else {
                Some(GestureEvent::End {
                    gesture: self.last_gesture,
                    timestamp: result.timestamp,
                    duration_ms: duration,
                })
            };

            self.last_gesture = result.gesture;
            self.gesture_start_time = result.timestamp;

            event
        } else {
            None
        }
    }

    /// Get current gesture
    pub fn current_gesture(&self) -> Gesture {
        self.last_gesture
    }

    /// Reset detector state
    pub fn reset(&mut self) {
        self.last_gesture = Gesture::Rest;
        self.gesture_start_time = 0;
    }
}

/// Performance metrics for real-time processing
#[derive(Debug, Clone, Default)]
pub struct PerformanceMetrics {
    /// Average classification latency in milliseconds
    pub avg_latency_ms: f32,
    /// Maximum classification latency
    pub max_latency_ms: f32,
    /// Samples processed per second
    pub samples_per_second: f32,
    /// Classifications per second
    pub classifications_per_second: f32,
    /// Total samples processed
    pub total_samples: u64,
    /// Total classifications made
    pub total_classifications: u64,
}

/// Performance monitor for real-time processing
pub struct PerformanceMonitor {
    start_time: Instant,
    total_samples: u64,
    total_classifications: u64,
    latencies: VecDeque<f32>,
    max_latencies: usize,
}

impl PerformanceMonitor {
    /// Create a new performance monitor
    pub fn new(max_latencies: usize) -> Self {
        Self {
            start_time: Instant::now(),
            total_samples: 0,
            total_classifications: 0,
            latencies: VecDeque::with_capacity(max_latencies),
            max_latencies,
        }
    }

    /// Record a sample processed
    pub fn record_sample(&mut self) {
        self.total_samples += 1;
    }

    /// Record a classification with latency
    pub fn record_classification(&mut self, latency_ms: f32) {
        self.total_classifications += 1;
        self.latencies.push_back(latency_ms);
        if self.latencies.len() > self.max_latencies {
            self.latencies.pop_front();
        }
    }

    /// Get current performance metrics
    pub fn get_metrics(&self) -> PerformanceMetrics {
        let elapsed = self.start_time.elapsed().as_secs_f32();

        let avg_latency = if self.latencies.is_empty() {
            0.0
        } else {
            self.latencies.iter().sum::<f32>() / self.latencies.len() as f32
        };

        let max_latency = self
            .latencies
            .iter()
            .cloned()
            .fold(0.0, f32::max);

        PerformanceMetrics {
            avg_latency_ms: avg_latency,
            max_latency_ms: max_latency,
            samples_per_second: self.total_samples as f32 / elapsed,
            classifications_per_second: self.total_classifications as f32 / elapsed,
            total_samples: self.total_samples,
            total_classifications: self.total_classifications,
        }
    }

    /// Reset the monitor
    pub fn reset(&mut self) {
        self.start_time = Instant::now();
        self.total_samples = 0;
        self.total_classifications = 0;
        self.latencies.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_sample(timestamp: u64, value: f32) -> EMGSample {
        EMGSample {
            timestamp_us: timestamp * 1000,
            channels: vec![value; 4],
        }
    }

    #[test]
    fn test_realtime_config_default() {
        let config = RealtimeConfig::default();
        assert_eq!(config.sample_rate, 1000);
        assert_eq!(config.window_size_ms, 200);
    }

    #[test]
    fn test_gesture_event_detector() {
        let mut detector = GestureEventDetector::new(100);

        // Initial state
        assert_eq!(detector.current_gesture(), Gesture::Rest);

        // Process a non-rest gesture
        let result = ClassificationResult {
            gesture: Gesture::HandClose,
            confidence: 0.9,
            alternatives: vec![],
            latency_ms: 1.0,
            timestamp: 1000,
        };

        let event = detector.process(&result);
        assert!(event.is_some());

        if let Some(GestureEvent::Start { gesture, .. }) = event {
            assert_eq!(gesture, Gesture::HandClose);
        }
    }

    #[test]
    fn test_performance_monitor() {
        let mut monitor = PerformanceMonitor::new(100);

        for _ in 0..100 {
            monitor.record_sample();
        }

        for i in 0..10 {
            monitor.record_classification(i as f32);
        }

        let metrics = monitor.get_metrics();
        assert_eq!(metrics.total_samples, 100);
        assert_eq!(metrics.total_classifications, 10);
    }
}
