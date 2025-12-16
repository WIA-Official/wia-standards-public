//! Translation pipeline for voice-to-sign conversion

use async_trait::async_trait;
use chrono::Utc;
use uuid::Uuid;

use crate::error::{Result, TranscriptionError, TranslationError, VoiceSignError};
use crate::types::*;

/// Trait for speech recognition (ASR) engines
#[async_trait]
pub trait AsrEngine: Send + Sync {
    /// Transcribe audio to text
    async fn transcribe(&self, audio: &AudioInput) -> Result<TranscriptionResult>;

    /// Get supported languages
    fn supported_languages(&self) -> Vec<String>;

    /// Get engine name
    fn name(&self) -> &str;
}

/// Trait for text-to-sign translation engines
#[async_trait]
pub trait TranslationEngine: Send + Sync {
    /// Translate text to sign gloss sequence
    async fn translate(
        &self,
        transcript: &TranscriptionResult,
        target_language: SignLanguageCode,
    ) -> Result<SignGlossSequence>;

    /// Get supported sign languages
    fn supported_sign_languages(&self) -> Vec<SignLanguageCode>;

    /// Get engine name
    fn name(&self) -> &str;
}

/// Trait for pose generation from gloss
#[async_trait]
pub trait PoseGenerator: Send + Sync {
    /// Generate pose sequence from gloss
    async fn generate(
        &self,
        gloss: &SignGlossSequence,
        frame_rate: u32,
    ) -> Result<SignPoseSequence>;

    /// Get supported skeleton standards
    fn supported_skeletons(&self) -> Vec<SkeletonStandard>;

    /// Get generator name
    fn name(&self) -> &str;
}

/// Trait for rendering avatar animation
#[async_trait]
pub trait Renderer: Send + Sync {
    /// Render pose sequence to output
    async fn render(
        &self,
        pose: &SignPoseSequence,
        settings: &RenderSettings,
    ) -> Result<RenderOutput>;

    /// Get available avatars
    fn available_avatars(&self) -> Vec<AvatarInfo>;

    /// Get renderer name
    fn name(&self) -> &str;
}

/// Main translation pipeline
pub struct TranslationPipeline {
    asr: Box<dyn AsrEngine>,
    translator: Box<dyn TranslationEngine>,
    pose_generator: Box<dyn PoseGenerator>,
    renderer: Option<Box<dyn Renderer>>,
}

impl TranslationPipeline {
    /// Create a new translation pipeline
    pub fn new(
        asr: Box<dyn AsrEngine>,
        translator: Box<dyn TranslationEngine>,
        pose_generator: Box<dyn PoseGenerator>,
    ) -> Self {
        Self {
            asr,
            translator,
            pose_generator,
            renderer: None,
        }
    }

    /// Add a renderer to the pipeline
    pub fn with_renderer(mut self, renderer: Box<dyn Renderer>) -> Self {
        self.renderer = Some(renderer);
        self
    }

    /// Process a translation request
    pub async fn process(&self, request: TranslationRequest) -> TranslationResponse {
        let start_time = std::time::Instant::now();
        let response_id = Uuid::new_v4().to_string();

        let mut transcript = None;
        let mut gloss = None;
        let mut notation = None;
        let mut pose = None;
        let mut render = None;
        let mut asr_time = None;
        let mut translation_time = None;
        let mut pose_time = None;
        let mut render_time = None;

        // Step 1: Transcription (if audio input)
        let transcript_result = if let Some(audio) = &request.audio {
            let asr_start = std::time::Instant::now();
            match self.asr.transcribe(audio).await {
                Ok(t) => {
                    asr_time = Some(asr_start.elapsed().as_secs_f64() * 1000.0);
                    if request.output.include_transcript {
                        transcript = Some(t.clone());
                    }
                    Ok(t)
                }
                Err(e) => Err(e),
            }
        } else if let Some(text) = &request.text {
            // Create a synthetic transcription from text input
            Ok(TranscriptionResult {
                audio_id: "text-input".to_string(),
                transcription_id: Uuid::new_v4().to_string(),
                timestamp: Utc::now(),
                language: text.language.clone(),
                confidence: 1.0,
                text: text.text.clone(),
                words: text
                    .text
                    .split_whitespace()
                    .enumerate()
                    .map(|(i, w)| TranscribedWord {
                        word: w.to_string(),
                        start_ms: i as f64 * 200.0,
                        end_ms: (i as f64 + 1.0) * 200.0,
                        confidence: 1.0,
                        pos: None,
                        lemma: None,
                    })
                    .collect(),
                alternatives: None,
                model: "text-input".to_string(),
                processing_time_ms: 0.0,
            })
        } else {
            Err(VoiceSignError::Validation(
                crate::error::ValidationError::NoInput,
            ))
        };

        // Handle transcription error
        let transcript_data = match transcript_result {
            Ok(t) => t,
            Err(e) => {
                return TranslationResponse {
                    request_id: request.request_id,
                    response_id,
                    timestamp: Utc::now(),
                    status: ResponseStatus::Error,
                    error: Some(e.to_error_info()),
                    transcript: None,
                    gloss: None,
                    notation: None,
                    pose: None,
                    render: None,
                    processing: ProcessingSummary {
                        total_time_ms: start_time.elapsed().as_secs_f64() * 1000.0,
                        asr_time_ms: asr_time,
                        translation_time_ms: None,
                        pose_generation_time_ms: None,
                        render_time_ms: None,
                    },
                };
            }
        };

        // Step 2: Translation to sign gloss
        let trans_start = std::time::Instant::now();
        let gloss_result = self
            .translator
            .translate(&transcript_data, request.target_language)
            .await;
        translation_time = Some(trans_start.elapsed().as_secs_f64() * 1000.0);

        let gloss_data = match gloss_result {
            Ok(g) => {
                if request.output.include_gloss {
                    gloss = Some(g.clone());
                }
                if request.output.include_notation {
                    notation = Some(self.generate_notation(&g));
                }
                g
            }
            Err(e) => {
                return TranslationResponse {
                    request_id: request.request_id,
                    response_id,
                    timestamp: Utc::now(),
                    status: ResponseStatus::Partial,
                    error: Some(e.to_error_info()),
                    transcript,
                    gloss: None,
                    notation: None,
                    pose: None,
                    render: None,
                    processing: ProcessingSummary {
                        total_time_ms: start_time.elapsed().as_secs_f64() * 1000.0,
                        asr_time_ms: asr_time,
                        translation_time_ms: translation_time,
                        pose_generation_time_ms: None,
                        render_time_ms: None,
                    },
                };
            }
        };

        // Step 3: Pose generation (if requested)
        if request.output.include_pose || request.output.include_render {
            let frame_rate = request
                .output
                .render_settings
                .as_ref()
                .map(|s| s.frame_rate)
                .unwrap_or(30);

            let pose_start = std::time::Instant::now();
            match self.pose_generator.generate(&gloss_data, frame_rate).await {
                Ok(p) => {
                    pose_time = Some(pose_start.elapsed().as_secs_f64() * 1000.0);
                    if request.output.include_pose {
                        pose = Some(p.clone());
                    }

                    // Step 4: Rendering (if requested and renderer available)
                    if request.output.include_render {
                        if let Some(renderer) = &self.renderer {
                            if let Some(settings) = &request.output.render_settings {
                                let render_start = std::time::Instant::now();
                                match renderer.render(&p, settings).await {
                                    Ok(r) => {
                                        render_time =
                                            Some(render_start.elapsed().as_secs_f64() * 1000.0);
                                        render = Some(r);
                                    }
                                    Err(e) => {
                                        // Non-fatal: continue with partial response
                                        log::warn!("Rendering failed: {}", e);
                                    }
                                }
                            }
                        }
                    }
                }
                Err(e) => {
                    log::warn!("Pose generation failed: {}", e);
                }
            }
        }

        TranslationResponse {
            request_id: request.request_id,
            response_id,
            timestamp: Utc::now(),
            status: ResponseStatus::Success,
            error: None,
            transcript,
            gloss,
            notation,
            pose,
            render,
            processing: ProcessingSummary {
                total_time_ms: start_time.elapsed().as_secs_f64() * 1000.0,
                asr_time_ms: asr_time,
                translation_time_ms: translation_time,
                pose_generation_time_ms: pose_time,
                render_time_ms: render_time,
            },
        }
    }

    /// Generate notation from gloss sequence
    fn generate_notation(&self, gloss: &SignGlossSequence) -> Vec<SignNotation> {
        gloss
            .glosses
            .iter()
            .map(|g| SignNotation {
                gloss_id: gloss.gloss_id.clone(),
                gloss: g.gloss.clone(),
                notation_type: NotationType::Hamnosys,
                manual: ManualComponent {
                    dominant: HandConfiguration {
                        handshape: "flat".to_string(),
                        thumb: Some(ThumbPosition::Out),
                        palm_orientation: Orientation {
                            direction: Direction::Forward,
                            rotation_deg: None,
                        },
                        finger_direction: Orientation {
                            direction: Direction::Up,
                            rotation_deg: None,
                        },
                        location: BodyLocation {
                            region: BodyRegion::NeutralSpace,
                            specific: None,
                            offset: None,
                        },
                        contact: None,
                    },
                    non_dominant: None,
                    hand_relationship: None,
                    movement: MovementComponent {
                        path: MovementPath {
                            path_type: PathType::None,
                            direction: None,
                            size: PathSize::Medium,
                            arc_direction: None,
                            waypoints: None,
                        },
                        dynamics: MovementDynamics {
                            speed: Speed::Normal,
                            tension: Tension::Normal,
                            acceleration: None,
                        },
                        repetition: None,
                    },
                },
                non_manual: None,
            })
            .collect()
    }
}

/// Request validator
pub struct RequestValidator;

impl RequestValidator {
    /// Validate a translation request
    pub fn validate(request: &TranslationRequest) -> Result<()> {
        use crate::error::ValidationError;

        // Check input
        if request.audio.is_none() && request.text.is_none() {
            return Err(VoiceSignError::Validation(ValidationError::NoInput));
        }

        if request.audio.is_some() && request.text.is_some() {
            return Err(VoiceSignError::Validation(ValidationError::ConflictingInput));
        }

        // Validate audio if present
        if let Some(audio) = &request.audio {
            Self::validate_audio(audio)?;
        }

        // Validate render settings if present
        if let Some(settings) = &request.output.render_settings {
            Self::validate_render_settings(settings)?;
        }

        Ok(())
    }

    fn validate_audio(audio: &AudioInput) -> Result<()> {
        use crate::error::{AudioError, ValidationError};

        // Check sample rate
        let valid_rates = [8000, 16000, 22050, 44100, 48000];
        if !valid_rates.contains(&audio.sample_rate) {
            return Err(VoiceSignError::Audio(AudioError::InvalidSampleRate(
                audio.sample_rate,
            )));
        }

        // Check duration
        if audio.duration_ms > 3600000.0 {
            return Err(VoiceSignError::Audio(AudioError::DurationTooLong(
                audio.duration_ms,
            )));
        }

        // Check data
        if audio.data.is_none() && audio.data_url.is_none() {
            return Err(VoiceSignError::Validation(ValidationError::MissingField(
                "audio.data or audio.data_url".to_string(),
            )));
        }

        Ok(())
    }

    fn validate_render_settings(settings: &RenderSettings) -> Result<()> {
        use crate::error::RenderingError;

        // Check resolution
        if settings.resolution.width < 320
            || settings.resolution.width > 3840
            || settings.resolution.height < 240
            || settings.resolution.height > 2160
        {
            return Err(VoiceSignError::Rendering(RenderingError::InvalidResolution(
                settings.resolution.width,
                settings.resolution.height,
            )));
        }

        // Check frame rate
        let valid_rates = [24, 25, 30, 50, 60];
        if !valid_rates.contains(&settings.frame_rate) {
            return Err(VoiceSignError::PoseGeneration(
                crate::error::PoseGenerationError::InvalidFrameRate(settings.frame_rate),
            ));
        }

        Ok(())
    }
}
