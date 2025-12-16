//! Simulator adapters for testing and development

use async_trait::async_trait;
use chrono::Utc;
use uuid::Uuid;

use crate::core::{AsrEngine, GlossDatabase, PoseGenerator, Renderer, TranslationEngine};
use crate::error::{Result, TranscriptionError, TranslationError, VoiceSignError};
use crate::types::*;

/// Simulated ASR engine for testing
pub struct SimulatorAsr {
    delay_ms: u64,
    supported_languages: Vec<String>,
}

impl SimulatorAsr {
    /// Create a new simulator ASR
    pub fn new() -> Self {
        Self {
            delay_ms: 100,
            supported_languages: vec![
                "en".to_string(),
                "ko".to_string(),
                "ja".to_string(),
                "zh".to_string(),
            ],
        }
    }

    /// Set simulated processing delay
    pub fn with_delay(mut self, delay_ms: u64) -> Self {
        self.delay_ms = delay_ms;
        self
    }
}

impl Default for SimulatorAsr {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl AsrEngine for SimulatorAsr {
    async fn transcribe(&self, audio: &AudioInput) -> Result<TranscriptionResult> {
        // Simulate processing delay
        tokio::time::sleep(tokio::time::Duration::from_millis(self.delay_ms)).await;

        // Check language hint
        let language = audio.language_hint.clone().unwrap_or_else(|| "en".to_string());

        if !self.supported_languages.contains(&language) {
            return Err(VoiceSignError::Transcription(
                TranscriptionError::UnsupportedLanguage(language),
            ));
        }

        // Generate simulated transcription
        let text = match language.as_str() {
            "ko" => "안녕하세요, 반갑습니다.".to_string(),
            "ja" => "こんにちは、よろしくお願いします。".to_string(),
            "zh" => "你好，很高兴认识你。".to_string(),
            _ => "Hello, how are you?".to_string(),
        };

        let words: Vec<TranscribedWord> = text
            .split_whitespace()
            .enumerate()
            .map(|(i, word)| TranscribedWord {
                word: word.to_string(),
                start_ms: i as f64 * 300.0,
                end_ms: (i as f64 + 1.0) * 300.0 - 50.0,
                confidence: 0.95,
                pos: None,
                lemma: None,
            })
            .collect();

        Ok(TranscriptionResult {
            audio_id: audio.audio_id.clone(),
            transcription_id: Uuid::new_v4().to_string(),
            timestamp: Utc::now(),
            language,
            confidence: 0.95,
            text,
            words,
            alternatives: None,
            model: "simulator-asr-v1".to_string(),
            processing_time_ms: self.delay_ms as f64,
        })
    }

    fn supported_languages(&self) -> Vec<String> {
        self.supported_languages.clone()
    }

    fn name(&self) -> &str {
        "SimulatorASR"
    }
}

/// Simulated translation engine
pub struct SimulatorTranslator {
    delay_ms: u64,
    gloss_db: GlossDatabase,
}

impl SimulatorTranslator {
    /// Create a new simulator translator
    pub fn new(language: SignLanguageCode) -> Self {
        Self {
            delay_ms: 50,
            gloss_db: GlossDatabase::new(language),
        }
    }

    /// Set simulated processing delay
    pub fn with_delay(mut self, delay_ms: u64) -> Self {
        self.delay_ms = delay_ms;
        self
    }
}

#[async_trait]
impl TranslationEngine for SimulatorTranslator {
    async fn translate(
        &self,
        transcript: &TranscriptionResult,
        target_language: SignLanguageCode,
    ) -> Result<SignGlossSequence> {
        // Simulate processing delay
        tokio::time::sleep(tokio::time::Duration::from_millis(self.delay_ms)).await;

        let mut glosses = Vec::new();
        let mut current_time = 0.0;

        for word in &transcript.words {
            let entries = self.gloss_db.lookup_word(&word.word);

            if let Some(entry) = entries.first() {
                let duration = entry.default_duration_ms;
                glosses.push(SignGloss {
                    gloss: entry.gloss.clone(),
                    start_ms: current_time,
                    end_ms: current_time + duration,
                    duration_ms: duration,
                    sign_type: entry.sign_type,
                    hamnosys: entry.hamnosys.clone(),
                    sigml: entry.sigml.clone(),
                    signwriting: entry.signwriting.clone(),
                    modifiers: None,
                    spatial_ref: None,
                    confidence: 0.9,
                });
                current_time += duration + 100.0; // Add gap between signs
            } else {
                // Fingerspell unknown words
                for (i, c) in word.word.chars().enumerate() {
                    let duration = 200.0;
                    glosses.push(SignGloss {
                        gloss: format!("FS:{}", c.to_uppercase()),
                        start_ms: current_time,
                        end_ms: current_time + duration,
                        duration_ms: duration,
                        sign_type: SignType::Fingerspell,
                        hamnosys: None,
                        sigml: None,
                        signwriting: None,
                        modifiers: None,
                        spatial_ref: None,
                        confidence: 0.85,
                    });
                    current_time += duration + 50.0;
                }
            }
        }

        // Determine sentence type
        let sentence_type = if transcript.text.ends_with('?') {
            SentenceType::Interrogative
        } else if transcript.text.ends_with('!') {
            SentenceType::Exclamatory
        } else {
            SentenceType::Declarative
        };

        Ok(SignGlossSequence {
            transcription_id: transcript.transcription_id.clone(),
            gloss_id: Uuid::new_v4().to_string(),
            timestamp: Utc::now(),
            sign_language: target_language,
            glosses,
            sentence_type,
            confidence: 0.88,
            translation_model: "simulator-translator-v1".to_string(),
            processing_time_ms: self.delay_ms as f64,
        })
    }

    fn supported_sign_languages(&self) -> Vec<SignLanguageCode> {
        vec![
            SignLanguageCode::Asl,
            SignLanguageCode::Bsl,
            SignLanguageCode::Ksl,
        ]
    }

    fn name(&self) -> &str {
        "SimulatorTranslator"
    }
}

/// Simulated pose generator
pub struct SimulatorPoseGenerator {
    delay_ms: u64,
    skeleton_standard: SkeletonStandard,
}

impl SimulatorPoseGenerator {
    /// Create a new simulator pose generator
    pub fn new() -> Self {
        Self {
            delay_ms: 50,
            skeleton_standard: SkeletonStandard::MediapipeHolistic,
        }
    }

    /// Set skeleton standard
    pub fn with_skeleton(mut self, standard: SkeletonStandard) -> Self {
        self.skeleton_standard = standard;
        self
    }

    /// Generate default skeleton joints
    fn create_skeleton(&self) -> SkeletonDefinition {
        let joints = vec![
            JointDefinition { id: 0, name: "root".to_string(), parent_id: None },
            JointDefinition { id: 1, name: "spine".to_string(), parent_id: Some(0) },
            JointDefinition { id: 2, name: "chest".to_string(), parent_id: Some(1) },
            JointDefinition { id: 3, name: "neck".to_string(), parent_id: Some(2) },
            JointDefinition { id: 4, name: "head".to_string(), parent_id: Some(3) },
            JointDefinition { id: 5, name: "left_shoulder".to_string(), parent_id: Some(2) },
            JointDefinition { id: 6, name: "left_elbow".to_string(), parent_id: Some(5) },
            JointDefinition { id: 7, name: "left_wrist".to_string(), parent_id: Some(6) },
            JointDefinition { id: 8, name: "right_shoulder".to_string(), parent_id: Some(2) },
            JointDefinition { id: 9, name: "right_elbow".to_string(), parent_id: Some(8) },
            JointDefinition { id: 10, name: "right_wrist".to_string(), parent_id: Some(9) },
        ];

        SkeletonDefinition {
            standard: self.skeleton_standard,
            joints,
        }
    }

    /// Generate a pose frame
    fn generate_frame(&self, index: u32, timestamp_ms: f64, _gloss: &SignGloss) -> PoseFrame {
        // Generate basic pose with some animation
        let t = (timestamp_ms / 1000.0) as f32;
        let wave = (t * 2.0 * std::f32::consts::PI).sin() * 0.1;

        let joints = vec![
            JointPose { joint_id: 0, position: Vector3::new(0.0, 0.0, 0.0), rotation: Quaternion::identity(), confidence: Some(1.0) },
            JointPose { joint_id: 1, position: Vector3::new(0.0, 0.3, 0.0), rotation: Quaternion::identity(), confidence: Some(1.0) },
            JointPose { joint_id: 2, position: Vector3::new(0.0, 0.6, 0.0), rotation: Quaternion::identity(), confidence: Some(1.0) },
            JointPose { joint_id: 3, position: Vector3::new(0.0, 0.8, 0.0), rotation: Quaternion::identity(), confidence: Some(1.0) },
            JointPose { joint_id: 4, position: Vector3::new(0.0, 1.0, 0.0), rotation: Quaternion::identity(), confidence: Some(1.0) },
            JointPose { joint_id: 5, position: Vector3::new(-0.2, 0.6, 0.0), rotation: Quaternion::identity(), confidence: Some(1.0) },
            JointPose { joint_id: 6, position: Vector3::new(-0.4, 0.4 + wave, 0.0), rotation: Quaternion::identity(), confidence: Some(1.0) },
            JointPose { joint_id: 7, position: Vector3::new(-0.5, 0.3 + wave, 0.1), rotation: Quaternion::identity(), confidence: Some(1.0) },
            JointPose { joint_id: 8, position: Vector3::new(0.2, 0.6, 0.0), rotation: Quaternion::identity(), confidence: Some(1.0) },
            JointPose { joint_id: 9, position: Vector3::new(0.4, 0.4 - wave, 0.0), rotation: Quaternion::identity(), confidence: Some(1.0) },
            JointPose { joint_id: 10, position: Vector3::new(0.5, 0.3 - wave, 0.1), rotation: Quaternion::identity(), confidence: Some(1.0) },
        ];

        PoseFrame {
            frame_index: index,
            timestamp_ms,
            joints,
            left_hand: Some(self.generate_hand_pose()),
            right_hand: Some(self.generate_hand_pose()),
            face: None,
        }
    }

    /// Generate a hand pose
    fn generate_hand_pose(&self) -> HandPose {
        let landmark_names = [
            HandLandmarkName::Wrist,
            HandLandmarkName::ThumbCmc, HandLandmarkName::ThumbMcp, HandLandmarkName::ThumbIp, HandLandmarkName::ThumbTip,
            HandLandmarkName::IndexMcp, HandLandmarkName::IndexPip, HandLandmarkName::IndexDip, HandLandmarkName::IndexTip,
            HandLandmarkName::MiddleMcp, HandLandmarkName::MiddlePip, HandLandmarkName::MiddleDip, HandLandmarkName::MiddleTip,
            HandLandmarkName::RingMcp, HandLandmarkName::RingPip, HandLandmarkName::RingDip, HandLandmarkName::RingTip,
            HandLandmarkName::PinkyMcp, HandLandmarkName::PinkyPip, HandLandmarkName::PinkyDip, HandLandmarkName::PinkyTip,
        ];

        let landmarks = landmark_names
            .iter()
            .enumerate()
            .map(|(i, name)| HandLandmark {
                id: i as u8,
                name: *name,
                position: Vector3::new(
                    (i % 5) as f32 * 0.02 - 0.04,
                    (i / 5) as f32 * 0.02,
                    0.0,
                ),
                confidence: Some(0.95),
            })
            .collect();

        HandPose { landmarks }
    }
}

impl Default for SimulatorPoseGenerator {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl PoseGenerator for SimulatorPoseGenerator {
    async fn generate(
        &self,
        gloss: &SignGlossSequence,
        frame_rate: u32,
    ) -> Result<SignPoseSequence> {
        // Simulate processing delay
        tokio::time::sleep(tokio::time::Duration::from_millis(self.delay_ms)).await;

        // Calculate total duration
        let total_duration_ms = gloss
            .glosses
            .last()
            .map(|g| g.end_ms)
            .unwrap_or(1000.0);

        let frame_duration_ms = 1000.0 / frame_rate as f64;
        let total_frames = (total_duration_ms / frame_duration_ms).ceil() as u32;

        let skeleton = self.create_skeleton();

        // Generate frames
        let mut frames = Vec::with_capacity(total_frames as usize);
        let mut current_gloss_idx = 0;

        for i in 0..total_frames {
            let timestamp_ms = i as f64 * frame_duration_ms;

            // Find current gloss
            while current_gloss_idx < gloss.glosses.len()
                && gloss.glosses[current_gloss_idx].end_ms < timestamp_ms
            {
                current_gloss_idx += 1;
            }

            let current_gloss = gloss.glosses.get(current_gloss_idx);
            let frame = if let Some(g) = current_gloss {
                self.generate_frame(i, timestamp_ms, g)
            } else {
                // Generate neutral pose
                self.generate_frame(i, timestamp_ms, &gloss.glosses[0])
            };

            frames.push(frame);
        }

        Ok(SignPoseSequence {
            gloss_id: gloss.gloss_id.clone(),
            pose_sequence_id: Uuid::new_v4().to_string(),
            timestamp: Utc::now(),
            frame_rate,
            total_frames,
            duration_ms: total_duration_ms,
            skeleton,
            frames,
            blend_shapes: None,
        })
    }

    fn supported_skeletons(&self) -> Vec<SkeletonStandard> {
        vec![
            SkeletonStandard::MediapipeHolistic,
            SkeletonStandard::WiaSignSkeleton,
        ]
    }

    fn name(&self) -> &str {
        "SimulatorPoseGenerator"
    }
}

/// Simulated renderer
pub struct SimulatorRenderer {
    delay_ms: u64,
    avatars: Vec<AvatarInfo>,
}

impl SimulatorRenderer {
    /// Create a new simulator renderer
    pub fn new() -> Self {
        Self {
            delay_ms: 100,
            avatars: vec![
                AvatarInfo {
                    avatar_id: "avatar-001".to_string(),
                    name: "Alex".to_string(),
                    style: AvatarStyle::Realistic,
                },
                AvatarInfo {
                    avatar_id: "avatar-002".to_string(),
                    name: "Sam".to_string(),
                    style: AvatarStyle::Stylized,
                },
                AvatarInfo {
                    avatar_id: "avatar-003".to_string(),
                    name: "Pat".to_string(),
                    style: AvatarStyle::Cartoon,
                },
            ],
        }
    }
}

impl Default for SimulatorRenderer {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Renderer for SimulatorRenderer {
    async fn render(
        &self,
        pose: &SignPoseSequence,
        settings: &RenderSettings,
    ) -> Result<RenderOutput> {
        // Simulate processing delay
        tokio::time::sleep(tokio::time::Duration::from_millis(self.delay_ms)).await;

        let avatar = if let Some(avatar_id) = &settings.avatar_id {
            self.avatars
                .iter()
                .find(|a| &a.avatar_id == avatar_id)
                .cloned()
        } else {
            self.avatars.first().cloned()
        };

        // Simulate file size (rough estimate)
        let file_size = (pose.total_frames as u64)
            * (settings.resolution.width as u64)
            * (settings.resolution.height as u64)
            / 100;

        Ok(RenderOutput {
            pose_sequence_id: pose.pose_sequence_id.clone(),
            render_id: Uuid::new_v4().to_string(),
            timestamp: Utc::now(),
            format: OutputFormat {
                output_type: settings.format,
                codec: Some("h264".to_string()),
                container: Some("mp4".to_string()),
            },
            resolution: settings.resolution,
            frame_rate: settings.frame_rate,
            duration_ms: pose.duration_ms,
            file_size_bytes: Some(file_size),
            data_url: Some(format!(
                "https://api.wia.org/renders/{}",
                Uuid::new_v4()
            )),
            avatar,
            quality_score: Some(0.92),
        })
    }

    fn available_avatars(&self) -> Vec<AvatarInfo> {
        self.avatars.clone()
    }

    fn name(&self) -> &str {
        "SimulatorRenderer"
    }
}

/// Create a default translation pipeline with simulators
pub fn create_simulator_pipeline(target_language: SignLanguageCode) -> crate::core::TranslationPipeline {
    crate::core::TranslationPipeline::new(
        Box::new(SimulatorAsr::new()),
        Box::new(SimulatorTranslator::new(target_language)),
        Box::new(SimulatorPoseGenerator::new()),
    )
    .with_renderer(Box::new(SimulatorRenderer::new()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_simulator_asr() {
        let asr = SimulatorAsr::new();
        let audio = AudioInput {
            audio_id: "test".to_string(),
            timestamp: Utc::now(),
            format: AudioFormat::Wav,
            sample_rate: 16000,
            channels: 1,
            bit_depth: 16,
            duration_ms: 1000.0,
            source: None,
            language_hint: Some("en".to_string()),
            data: Some(vec![0u8; 32000]),
            data_url: None,
        };

        let result = asr.transcribe(&audio).await;
        assert!(result.is_ok());

        let transcript = result.unwrap();
        assert!(!transcript.text.is_empty());
        assert!(!transcript.words.is_empty());
    }

    #[tokio::test]
    async fn test_simulator_translator() {
        let translator = SimulatorTranslator::new(SignLanguageCode::Asl);
        let transcript = TranscriptionResult {
            audio_id: "test".to_string(),
            transcription_id: "t1".to_string(),
            timestamp: Utc::now(),
            language: "en".to_string(),
            confidence: 0.95,
            text: "Hello, how are you?".to_string(),
            words: vec![
                TranscribedWord {
                    word: "Hello".to_string(),
                    start_ms: 0.0,
                    end_ms: 300.0,
                    confidence: 0.95,
                    pos: None,
                    lemma: None,
                },
            ],
            alternatives: None,
            model: "test".to_string(),
            processing_time_ms: 100.0,
        };

        let result = translator.translate(&transcript, SignLanguageCode::Asl).await;
        assert!(result.is_ok());

        let gloss = result.unwrap();
        assert!(!gloss.glosses.is_empty());
    }

    #[tokio::test]
    async fn test_simulator_pose_generator() {
        let generator = SimulatorPoseGenerator::new();
        let gloss = SignGlossSequence {
            transcription_id: "t1".to_string(),
            gloss_id: "g1".to_string(),
            timestamp: Utc::now(),
            sign_language: SignLanguageCode::Asl,
            glosses: vec![SignGloss {
                gloss: "HELLO".to_string(),
                start_ms: 0.0,
                end_ms: 500.0,
                duration_ms: 500.0,
                sign_type: SignType::Lexical,
                hamnosys: None,
                sigml: None,
                signwriting: None,
                modifiers: None,
                spatial_ref: None,
                confidence: 0.9,
            }],
            sentence_type: SentenceType::Declarative,
            confidence: 0.9,
            translation_model: "test".to_string(),
            processing_time_ms: 50.0,
        };

        let result = generator.generate(&gloss, 30).await;
        assert!(result.is_ok());

        let pose = result.unwrap();
        assert!(pose.total_frames > 0);
        assert!(!pose.frames.is_empty());
    }

    #[tokio::test]
    async fn test_full_pipeline() {
        let pipeline = create_simulator_pipeline(SignLanguageCode::Asl);

        let request = TranslationRequest {
            request_id: "req-001".to_string(),
            audio: None,
            text: Some(TextInput {
                text: "Hello".to_string(),
                language: "en".to_string(),
            }),
            target_language: SignLanguageCode::Asl,
            output: OutputPreferences {
                include_transcript: true,
                include_gloss: true,
                include_notation: false,
                include_pose: true,
                include_render: false,
                render_settings: None,
            },
            options: None,
        };

        let response = pipeline.process(request).await;
        assert_eq!(response.status, ResponseStatus::Success);
        assert!(response.gloss.is_some());
        assert!(response.pose.is_some());
    }
}
