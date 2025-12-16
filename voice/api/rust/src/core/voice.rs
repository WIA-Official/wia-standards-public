//! Voice processing utilities

use crate::error::{AudioError, Result, VoiceSignError};
use crate::types::*;

/// Audio processor for preprocessing audio input
pub struct AudioProcessor {
    target_sample_rate: u32,
}

impl AudioProcessor {
    /// Create a new audio processor
    pub fn new() -> Self {
        Self {
            target_sample_rate: 16000,
        }
    }

    /// Create with custom target sample rate
    pub fn with_sample_rate(sample_rate: u32) -> Self {
        Self {
            target_sample_rate: sample_rate,
        }
    }

    /// Validate audio input
    pub fn validate(&self, audio: &AudioInput) -> Result<()> {
        // Check for empty audio
        if let Some(data) = &audio.data {
            if data.is_empty() {
                return Err(VoiceSignError::Audio(AudioError::EmptyAudio));
            }
        }

        // Validate sample rate
        let valid_rates = [8000, 16000, 22050, 44100, 48000];
        if !valid_rates.contains(&audio.sample_rate) {
            return Err(VoiceSignError::Audio(AudioError::InvalidSampleRate(
                audio.sample_rate,
            )));
        }

        // Validate duration
        if audio.duration_ms > 3600000.0 {
            return Err(VoiceSignError::Audio(AudioError::DurationTooLong(
                audio.duration_ms,
            )));
        }

        Ok(())
    }

    /// Get the number of samples from duration
    pub fn samples_from_duration(&self, duration_ms: f64, sample_rate: u32) -> usize {
        ((duration_ms / 1000.0) * sample_rate as f64) as usize
    }

    /// Calculate duration from sample count
    pub fn duration_from_samples(&self, samples: usize, sample_rate: u32) -> f64 {
        (samples as f64 / sample_rate as f64) * 1000.0
    }

    /// Target sample rate for processing
    pub fn target_sample_rate(&self) -> u32 {
        self.target_sample_rate
    }
}

impl Default for AudioProcessor {
    fn default() -> Self {
        Self::new()
    }
}

/// Audio chunk for streaming processing
#[derive(Debug, Clone)]
pub struct AudioChunk {
    /// Chunk sequence number
    pub sequence: u64,

    /// Audio data
    pub data: Vec<u8>,

    /// Sample rate
    pub sample_rate: u32,

    /// Is this the final chunk?
    pub is_final: bool,
}

/// Streaming audio buffer
pub struct StreamingBuffer {
    chunks: Vec<AudioChunk>,
    total_samples: usize,
    sample_rate: u32,
}

impl StreamingBuffer {
    /// Create a new streaming buffer
    pub fn new(sample_rate: u32) -> Self {
        Self {
            chunks: Vec::new(),
            total_samples: 0,
            sample_rate,
        }
    }

    /// Add a chunk to the buffer
    pub fn push(&mut self, chunk: AudioChunk) {
        let samples = chunk.data.len() / 2; // Assuming 16-bit audio
        self.total_samples += samples;
        self.chunks.push(chunk);
    }

    /// Get total duration in milliseconds
    pub fn duration_ms(&self) -> f64 {
        (self.total_samples as f64 / self.sample_rate as f64) * 1000.0
    }

    /// Check if buffer has enough data for processing
    pub fn has_enough_data(&self, min_duration_ms: f64) -> bool {
        self.duration_ms() >= min_duration_ms
    }

    /// Clear the buffer
    pub fn clear(&mut self) {
        self.chunks.clear();
        self.total_samples = 0;
    }

    /// Get chunk count
    pub fn chunk_count(&self) -> usize {
        self.chunks.len()
    }
}

/// Voice activity detection result
#[derive(Debug, Clone)]
pub struct VadResult {
    /// Is speech detected?
    pub is_speech: bool,

    /// Confidence score (0.0 - 1.0)
    pub confidence: f32,

    /// Start time in milliseconds
    pub start_ms: f64,

    /// End time in milliseconds
    pub end_ms: f64,
}

/// Simple voice activity detector
pub struct VoiceActivityDetector {
    /// Energy threshold for speech detection
    energy_threshold: f32,

    /// Minimum speech duration in ms
    min_speech_ms: f64,
}

impl VoiceActivityDetector {
    /// Create a new VAD
    pub fn new() -> Self {
        Self {
            energy_threshold: 0.01,
            min_speech_ms: 100.0,
        }
    }

    /// Create with custom threshold
    pub fn with_threshold(threshold: f32) -> Self {
        Self {
            energy_threshold: threshold,
            min_speech_ms: 100.0,
        }
    }

    /// Detect voice activity in audio
    /// Note: This is a placeholder implementation
    pub fn detect(&self, _audio: &AudioInput) -> Vec<VadResult> {
        // In a real implementation, this would analyze the audio data
        // For now, return a placeholder result
        vec![VadResult {
            is_speech: true,
            confidence: 0.9,
            start_ms: 0.0,
            end_ms: 1000.0,
        }]
    }

    /// Check if audio contains speech
    pub fn has_speech(&self, audio: &AudioInput) -> bool {
        let results = self.detect(audio);
        results.iter().any(|r| r.is_speech && r.confidence > 0.5)
    }
}

impl Default for VoiceActivityDetector {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::Utc;

    fn create_test_audio() -> AudioInput {
        AudioInput {
            audio_id: "test".to_string(),
            timestamp: Utc::now(),
            format: AudioFormat::Wav,
            sample_rate: 16000,
            channels: 1,
            bit_depth: 16,
            duration_ms: 1000.0,
            source: None,
            language_hint: None,
            data: Some(vec![0u8; 32000]),
            data_url: None,
        }
    }

    #[test]
    fn test_audio_processor_validation() {
        let processor = AudioProcessor::new();
        let audio = create_test_audio();
        assert!(processor.validate(&audio).is_ok());
    }

    #[test]
    fn test_audio_processor_invalid_sample_rate() {
        let processor = AudioProcessor::new();
        let mut audio = create_test_audio();
        audio.sample_rate = 12345;
        assert!(processor.validate(&audio).is_err());
    }

    #[test]
    fn test_streaming_buffer() {
        let mut buffer = StreamingBuffer::new(16000);

        buffer.push(AudioChunk {
            sequence: 0,
            data: vec![0u8; 3200], // 100ms at 16kHz, 16-bit
            sample_rate: 16000,
            is_final: false,
        });

        assert_eq!(buffer.chunk_count(), 1);
        assert!(buffer.duration_ms() > 0.0);
    }

    #[test]
    fn test_vad() {
        let vad = VoiceActivityDetector::new();
        let audio = create_test_audio();
        assert!(vad.has_speech(&audio));
    }
}
