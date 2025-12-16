//! Streaming example for WIA Voice-Sign API
//!
//! This example demonstrates how to use the streaming capabilities
//! of the Voice-Sign API for real-time translation.
//!
//! Run with: cargo run --example streaming --features websocket

use std::time::Duration;
use tokio::time::sleep;
use wia_voice_sign::*;
use wia_voice_sign::core::{AudioChunk, StreamingBuffer};

/// Simulated audio stream producer
struct AudioProducer {
    sample_rate: u32,
    chunk_duration_ms: u64,
    total_chunks: usize,
}

impl AudioProducer {
    fn new(sample_rate: u32, chunk_duration_ms: u64, total_chunks: usize) -> Self {
        Self {
            sample_rate,
            chunk_duration_ms,
            total_chunks,
        }
    }

    async fn produce_chunks(&self) -> Vec<AudioChunk> {
        let mut chunks = Vec::new();
        let samples_per_chunk = (self.sample_rate as u64 * self.chunk_duration_ms / 1000) as usize;
        let bytes_per_chunk = samples_per_chunk * 2; // 16-bit audio

        for i in 0..self.total_chunks {
            // Simulate real-time audio capture delay
            sleep(Duration::from_millis(self.chunk_duration_ms / 2)).await;

            chunks.push(AudioChunk {
                sequence: i as u64,
                data: vec![0u8; bytes_per_chunk],
                sample_rate: self.sample_rate,
                is_final: i == self.total_chunks - 1,
            });

            println!("  Produced chunk {} of {}", i + 1, self.total_chunks);
        }

        chunks
    }
}

/// Streaming translation processor
struct StreamingProcessor {
    pipeline: TranslationPipeline,
    buffer: StreamingBuffer,
    min_buffer_ms: f64,
    target_language: SignLanguageCode,
}

impl StreamingProcessor {
    fn new(target_language: SignLanguageCode) -> Self {
        Self {
            pipeline: create_pipeline(target_language),
            buffer: StreamingBuffer::new(16000),
            min_buffer_ms: 500.0, // Process every 500ms
            target_language,
        }
    }

    async fn process_chunk(&mut self, chunk: AudioChunk) -> Option<SignGlossSequence> {
        let is_final = chunk.is_final;
        self.buffer.push(chunk);

        // Process if we have enough data or if this is the final chunk
        if self.buffer.has_enough_data(self.min_buffer_ms) || is_final {
            // In a real implementation, we would:
            // 1. Extract audio data from buffer
            // 2. Send to ASR for transcription
            // 3. Translate to sign gloss
            // 4. Clear processed data from buffer

            // For this example, we'll simulate the process
            let request = TranslationRequest {
                request_id: format!("stream-{}", uuid::Uuid::new_v4()),
                audio: None,
                text: Some(TextInput {
                    text: "Hello".to_string(), // Simulated transcription
                    language: "en".to_string(),
                }),
                target_language: self.target_language,
                output: OutputPreferences {
                    include_transcript: false,
                    include_gloss: true,
                    include_notation: false,
                    include_pose: false,
                    include_render: false,
                    render_settings: None,
                },
                options: None,
            };

            let response = self.pipeline.process(request).await;
            self.buffer.clear();

            return response.gloss;
        }

        None
    }
}

#[tokio::main]
async fn main() {
    println!("WIA Voice-Sign API - Streaming Example");
    println!("======================================\n");

    // Configuration
    let sample_rate = 16000u32;
    let chunk_duration_ms = 200u64; // 200ms chunks
    let total_chunks = 10;
    let target_language = SignLanguageCode::Asl;

    println!("Configuration:");
    println!("  - Sample rate: {} Hz", sample_rate);
    println!("  - Chunk duration: {} ms", chunk_duration_ms);
    println!("  - Total chunks: {}", total_chunks);
    println!(
        "  - Total duration: {} ms",
        chunk_duration_ms * total_chunks as u64
    );
    println!("  - Target language: {}", target_language);

    // Create producer and processor
    let producer = AudioProducer::new(sample_rate, chunk_duration_ms, total_chunks);
    let mut processor = StreamingProcessor::new(target_language);

    println!("\nStarting streaming translation...\n");

    // Produce and process chunks
    let chunks = producer.produce_chunks().await;

    println!("\nProcessing chunks...\n");

    let mut gloss_count = 0;
    for chunk in chunks {
        if let Some(gloss) = processor.process_chunk(chunk).await {
            gloss_count += 1;
            println!("  Received gloss sequence {}:", gloss_count);
            for g in &gloss.glosses {
                println!("    - {} ({:?})", g.gloss, g.sign_type);
            }
        }
    }

    println!("\nStreaming completed!");
    println!("  - Total gloss sequences: {}", gloss_count);

    // Demonstrate streaming buffer usage
    println!("\n\nStreaming Buffer Demo");
    println!("---------------------");

    let mut buffer = StreamingBuffer::new(16000);

    // Add some chunks
    for i in 0..5 {
        buffer.push(AudioChunk {
            sequence: i,
            data: vec![0u8; 3200], // 100ms at 16kHz, 16-bit
            sample_rate: 16000,
            is_final: i == 4,
        });
        println!(
            "  Added chunk {}, buffer duration: {:.1}ms",
            i,
            buffer.duration_ms()
        );
    }

    println!("\nBuffer status:");
    println!("  - Chunk count: {}", buffer.chunk_count());
    println!("  - Duration: {:.1}ms", buffer.duration_ms());
    println!(
        "  - Has enough data (500ms): {}",
        buffer.has_enough_data(500.0)
    );

    println!("\nStreaming example completed!");
}
