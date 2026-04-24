//! CI Signal Enhancement Module
//!
//! Phase 3: 옥타브 정보를 CI 신호에 인코딩하는 프로토콜

use serde::{Deserialize, Serialize};
use std::f32::consts::PI;
use crate::{CI_ELECTRODE_COUNT, COCHLEAR_CENTER_FREQUENCIES};

/// Enhancement 설정
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnhancementConfig {
    /// Temporal Modulation 활성화
    pub temporal_modulation: bool,
    /// Electrode Pattern Enhancement 활성화
    pub electrode_pattern: bool,
    /// Harmonic Enhancement 활성화
    pub harmonic_enhancement: bool,
    /// 변조 깊이 (0.0 - 1.0)
    pub modulation_depth: f32,
    /// 옥타브별 변조 주파수 (Hz)
    pub octave_modulation_rates: [f32; 10],
}

impl Default for EnhancementConfig {
    fn default() -> Self {
        // 옥타브별 고유 변조 주파수 (구분 가능한 주파수 대역)
        let octave_modulation_rates = [
            4.0,   // Octave 0 (C0)
            5.0,   // Octave 1
            6.5,   // Octave 2
            8.0,   // Octave 3
            10.0,  // Octave 4 (middle C)
            12.5,  // Octave 5
            16.0,  // Octave 6
            20.0,  // Octave 7
            25.0,  // Octave 8
            32.0,  // Octave 9
        ];

        Self {
            temporal_modulation: true,
            electrode_pattern: true,
            harmonic_enhancement: true,
            modulation_depth: 0.3,
            octave_modulation_rates,
        }
    }
}

/// Enhanced CI 신호
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnhancedCISignal {
    /// 원본 전극 활성화
    pub original_electrodes: Vec<ElectrodeEnhancement>,
    /// 옥타브 정보
    pub octave_info: OctaveEncodingInfo,
    /// Enhancement 메타데이터
    pub metadata: EnhancementMetadata,
}

/// 전극 Enhancement 데이터
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ElectrodeEnhancement {
    /// 전극 ID (1-22)
    pub electrode_id: u8,
    /// 원본 진폭
    pub original_amplitude: f32,
    /// Enhanced 진폭
    pub enhanced_amplitude: f32,
    /// 시간적 변조 패턴
    pub temporal_modulation: Option<TemporalModulation>,
}

/// 시간적 변조 파라미터
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TemporalModulation {
    /// 변조 주파수 (Hz)
    pub frequency: f32,
    /// 변조 깊이 (0.0 - 1.0)
    pub depth: f32,
    /// 위상 (radians)
    pub phase: f32,
}

/// 옥타브 인코딩 정보
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OctaveEncodingInfo {
    /// 검출된 옥타브
    pub octave: u8,
    /// 기본 주파수
    pub fundamental_frequency: f32,
    /// 인코딩 방법
    pub encoding_methods: Vec<String>,
    /// 인코딩 신뢰도
    pub encoding_confidence: f32,
}

/// Enhancement 메타데이터
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnhancementMetadata {
    /// 처리 시간 (ms)
    pub processing_time_ms: f32,
    /// 사용된 설정
    pub config_used: String,
    /// 버전
    pub version: String,
}

/// CI 신호 Enhancer
pub struct CIEnhancer {
    config: EnhancementConfig,
    sample_rate: u32,
}

impl CIEnhancer {
    pub fn new(config: EnhancementConfig, sample_rate: u32) -> Self {
        Self { config, sample_rate }
    }

    pub fn with_default(sample_rate: u32) -> Self {
        Self::new(EnhancementConfig::default(), sample_rate)
    }

    /// 옥타브 정보를 CI 신호에 인코딩
    pub fn enhance(
        &self,
        electrode_amplitudes: &[f32],
        octave: u8,
        fundamental_frequency: f32,
        frame_index: usize,
    ) -> EnhancedCISignal {
        let start_time = std::time::Instant::now();

        let mut enhanced_electrodes = Vec::new();
        let mut encoding_methods = Vec::new();

        for (i, &amplitude) in electrode_amplitudes.iter().enumerate() {
            let electrode_id = (i + 1) as u8;
            let mut enhanced_amplitude = amplitude;
            let mut temporal_mod = None;

            // Method 1: Temporal Modulation
            if self.config.temporal_modulation && amplitude > 0.05 {
                let mod_freq = self.config.octave_modulation_rates
                    .get(octave as usize)
                    .copied()
                    .unwrap_or(10.0);

                let time = frame_index as f32 / self.sample_rate as f32;
                let modulation = 1.0 + self.config.modulation_depth
                    * (2.0 * PI * mod_freq * time).sin();

                enhanced_amplitude *= modulation;

                temporal_mod = Some(TemporalModulation {
                    frequency: mod_freq,
                    depth: self.config.modulation_depth,
                    phase: (2.0 * PI * mod_freq * time) % (2.0 * PI),
                });

                if !encoding_methods.contains(&"temporal_modulation".to_string()) {
                    encoding_methods.push("temporal_modulation".to_string());
                }
            }

            // Method 2: Electrode Pattern Enhancement
            if self.config.electrode_pattern {
                let pattern_boost = self.octave_electrode_pattern(octave, electrode_id);
                enhanced_amplitude *= pattern_boost;

                if pattern_boost != 1.0 && !encoding_methods.contains(&"electrode_pattern".to_string()) {
                    encoding_methods.push("electrode_pattern".to_string());
                }
            }

            // Method 3: Harmonic Enhancement
            if self.config.harmonic_enhancement {
                let harmonic_boost = self.harmonic_emphasis(fundamental_frequency, electrode_id);
                enhanced_amplitude *= harmonic_boost;

                if harmonic_boost != 1.0 && !encoding_methods.contains(&"harmonic_enhancement".to_string()) {
                    encoding_methods.push("harmonic_enhancement".to_string());
                }
            }

            enhanced_electrodes.push(ElectrodeEnhancement {
                electrode_id,
                original_amplitude: amplitude,
                enhanced_amplitude: enhanced_amplitude.min(1.0),
                temporal_modulation: temporal_mod,
            });
        }

        let processing_time = start_time.elapsed().as_secs_f32() * 1000.0;

        EnhancedCISignal {
            original_electrodes: enhanced_electrodes,
            octave_info: OctaveEncodingInfo {
                octave,
                fundamental_frequency,
                encoding_methods,
                encoding_confidence: 0.9,
            },
            metadata: EnhancementMetadata {
                processing_time_ms: processing_time,
                config_used: format!("{:?}", self.config),
                version: "0.1.0".to_string(),
            },
        }
    }

    /// 옥타브별 전극 패턴 부스트
    fn octave_electrode_pattern(&self, octave: u8, electrode_id: u8) -> f32 {
        // 낮은 옥타브: 낮은 전극 강조
        // 높은 옥타브: 높은 전극 강조
        let electrode_idx = electrode_id as f32;
        let center_electrode = match octave {
            0..=2 => 3.0,   // 저음역: 전극 1-5 강조
            3..=4 => 8.0,   // 중음역: 전극 6-12 강조
            5..=6 => 15.0,  // 고음역: 전극 13-18 강조
            _ => 20.0,      // 초고음역: 전극 19-22 강조
        };

        // 가우시안 분포로 부스트
        let sigma = 4.0;
        let distance = (electrode_idx - center_electrode).abs();
        let boost = (-distance * distance / (2.0 * sigma * sigma)).exp();

        // 최소 1.0 (감쇄 없음), 최대 1.3 (30% 부스트)
        1.0 + boost * 0.3
    }

    /// 하모닉 관계 강조
    fn harmonic_emphasis(&self, fundamental: f32, electrode_id: u8) -> f32 {
        let electrode_freq = COCHLEAR_CENTER_FREQUENCIES
            .get((electrode_id as usize).saturating_sub(1))
            .copied()
            .unwrap_or(1000.0);

        // 기본음 또는 하모닉에 해당하는지 확인
        for h in 1..=8 {
            let harmonic_freq = fundamental * h as f32;
            let ratio = electrode_freq / harmonic_freq;

            // 20% 이내면 하모닉으로 판단
            if (ratio - 1.0).abs() < 0.2 {
                // 기본음이 가장 강조, 하모닉은 점차 감소
                return 1.0 + 0.3 / h as f32;
            }
        }

        1.0 // 비하모닉은 변경 없음
    }

    /// 실시간 스트림용 Enhancement
    pub fn enhance_stream(
        &self,
        electrode_stream: &[Vec<f32>],
        octave: u8,
        fundamental_frequency: f32,
    ) -> Vec<EnhancedCISignal> {
        electrode_stream
            .iter()
            .enumerate()
            .map(|(i, electrodes)| {
                self.enhance(electrodes, octave, fundamental_frequency, i)
            })
            .collect()
    }
}

/// CI Vocoder Simulator
///
/// Enhanced 신호가 실제로 어떻게 들리는지 시뮬레이션
pub struct CIVocoderSimulator {
    sample_rate: u32,
    num_channels: usize,
    /// 전극별 밴드패스 필터 중심 주파수
    center_frequencies: Vec<f32>,
    /// 전극별 대역폭
    bandwidths: Vec<f32>,
}

impl CIVocoderSimulator {
    pub fn new(sample_rate: u32) -> Self {
        let num_channels = CI_ELECTRODE_COUNT;
        let center_frequencies: Vec<f32> = COCHLEAR_CENTER_FREQUENCIES.to_vec();

        // ERB 기반 대역폭
        let bandwidths: Vec<f32> = center_frequencies
            .iter()
            .map(|&cf| 24.7 * (4.37 * cf / 1000.0 + 1.0))
            .collect();

        Self {
            sample_rate,
            num_channels,
            center_frequencies,
            bandwidths,
        }
    }

    /// CI 처리 시뮬레이션 (원본 오디오 → CI 사용자가 듣는 소리)
    pub fn simulate_ci_hearing(&self, audio: &[f32]) -> Vec<f32> {
        let mut output = vec![0.0f32; audio.len()];

        for ch in 0..self.num_channels {
            let cf = self.center_frequencies[ch];
            let bw = self.bandwidths[ch];

            // 밴드패스 필터링
            let filtered = self.bandpass_filter(audio, cf, bw);

            // 엔벨로프 추출 (정류 + 로우패스)
            let envelope = self.extract_envelope(&filtered);

            // 노이즈 캐리어로 재합성
            let carrier = self.generate_noise_carrier(audio.len(), cf, bw);

            for i in 0..audio.len() {
                output[i] += envelope[i] * carrier[i] / self.num_channels as f32;
            }
        }

        output
    }

    /// CI 처리 시뮬레이션 with Enhancement
    pub fn simulate_enhanced_ci_hearing(
        &self,
        audio: &[f32],
        enhanced_signal: &EnhancedCISignal,
    ) -> Vec<f32> {
        let mut output = vec![0.0f32; audio.len()];

        for electrode in &enhanced_signal.original_electrodes {
            let ch = (electrode.electrode_id as usize).saturating_sub(1);
            if ch >= self.num_channels {
                continue;
            }

            let cf = self.center_frequencies[ch];
            let bw = self.bandwidths[ch];

            // 밴드패스 필터링
            let filtered = self.bandpass_filter(audio, cf, bw);

            // 엔벨로프 추출
            let mut envelope = self.extract_envelope(&filtered);

            // Enhancement 적용
            if let Some(ref tm) = electrode.temporal_modulation {
                for (i, env) in envelope.iter_mut().enumerate() {
                    let t = i as f32 / self.sample_rate as f32;
                    let modulation = 1.0 + tm.depth * (2.0 * PI * tm.frequency * t + tm.phase).sin();
                    *env *= modulation;
                }
            }

            // Enhanced 진폭 적용
            let amp_ratio = electrode.enhanced_amplitude / electrode.original_amplitude.max(0.001);

            // 사인파 캐리어로 재합성 (더 명확한 피치)
            let carrier = self.generate_tone_carrier(audio.len(), cf);

            for i in 0..audio.len() {
                output[i] += envelope[i] * carrier[i] * amp_ratio / self.num_channels as f32;
            }
        }

        output
    }

    /// 간단한 밴드패스 필터 (2차 IIR)
    fn bandpass_filter(&self, input: &[f32], center_freq: f32, bandwidth: f32) -> Vec<f32> {
        let omega = 2.0 * PI * center_freq / self.sample_rate as f32;
        let bw = 2.0 * PI * bandwidth / self.sample_rate as f32;

        let alpha = bw.sin() / 2.0;
        let cos_omega = omega.cos();

        let b0 = alpha;
        let b1 = 0.0;
        let b2 = -alpha;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * cos_omega;
        let a2 = 1.0 - alpha;

        let mut output = vec![0.0f32; input.len()];
        let mut x1 = 0.0f32;
        let mut x2 = 0.0f32;
        let mut y1 = 0.0f32;
        let mut y2 = 0.0f32;

        for (i, &x0) in input.iter().enumerate() {
            let y0 = (b0 * x0 + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2) / a0;
            output[i] = y0;

            x2 = x1;
            x1 = x0;
            y2 = y1;
            y1 = y0;
        }

        output
    }

    /// 엔벨로프 추출 (전파 정류 + 로우패스)
    fn extract_envelope(&self, input: &[f32]) -> Vec<f32> {
        // 전파 정류
        let rectified: Vec<f32> = input.iter().map(|&x| x.abs()).collect();

        // 로우패스 필터 (400 Hz cutoff)
        let cutoff = 400.0;
        let rc = 1.0 / (2.0 * PI * cutoff);
        let dt = 1.0 / self.sample_rate as f32;
        let alpha = dt / (rc + dt);

        let mut output = vec![0.0f32; input.len()];
        output[0] = rectified[0];

        for i in 1..input.len() {
            output[i] = output[i - 1] + alpha * (rectified[i] - output[i - 1]);
        }

        output
    }

    /// 노이즈 캐리어 생성 (기존 CI 시뮬레이션)
    fn generate_noise_carrier(&self, length: usize, center_freq: f32, bandwidth: f32) -> Vec<f32> {
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};

        // 의사 난수 생성
        let noise: Vec<f32> = (0..length)
            .map(|i| {
                let mut hasher = DefaultHasher::new();
                (i as u64 ^ (center_freq as u64)).hash(&mut hasher);
                let h = hasher.finish();
                (h as f32 / u64::MAX as f32) * 2.0 - 1.0
            })
            .collect();

        // 밴드패스 필터링
        self.bandpass_filter(&noise, center_freq, bandwidth)
    }

    /// 사인파 캐리어 생성 (Enhanced CI - 더 명확한 피치)
    fn generate_tone_carrier(&self, length: usize, frequency: f32) -> Vec<f32> {
        (0..length)
            .map(|i| {
                let t = i as f32 / self.sample_rate as f32;
                (2.0 * PI * frequency * t).sin()
            })
            .collect()
    }
}

/// A/B 비교 테스트용 결과
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ABComparisonResult {
    /// 원본 CI 처리 오디오
    pub original_ci: Vec<f32>,
    /// Enhanced CI 처리 오디오
    pub enhanced_ci: Vec<f32>,
    /// 옥타브 명확도 점수 (시뮬레이션)
    pub octave_clarity_score: f32,
    /// 처리 정보
    pub info: String,
}

impl CIVocoderSimulator {
    /// A/B 비교 생성
    pub fn create_ab_comparison(
        &self,
        audio: &[f32],
        enhanced_signal: &EnhancedCISignal,
    ) -> ABComparisonResult {
        let original_ci = self.simulate_ci_hearing(audio);
        let enhanced_ci = self.simulate_enhanced_ci_hearing(audio, enhanced_signal);

        // 옥타브 명확도 점수 계산 (피치 변동성 기반)
        let original_variance = self.calculate_pitch_variance(&original_ci);
        let enhanced_variance = self.calculate_pitch_variance(&enhanced_ci);

        let clarity_improvement = if original_variance > 0.0 {
            (original_variance - enhanced_variance) / original_variance
        } else {
            0.0
        };

        ABComparisonResult {
            original_ci,
            enhanced_ci,
            octave_clarity_score: (0.5 + clarity_improvement * 0.5).max(0.0).min(1.0),
            info: format!(
                "Octave: {}, Fundamental: {:.1} Hz, Enhancement methods: {:?}",
                enhanced_signal.octave_info.octave,
                enhanced_signal.octave_info.fundamental_frequency,
                enhanced_signal.octave_info.encoding_methods
            ),
        }
    }

    fn calculate_pitch_variance(&self, audio: &[f32]) -> f32 {
        // 간단한 제로 크로싱 기반 피치 변동성
        let mut zero_crossings = Vec::new();

        for i in 1..audio.len() {
            if (audio[i] >= 0.0) != (audio[i - 1] >= 0.0) {
                zero_crossings.push(i);
            }
        }

        if zero_crossings.len() < 3 {
            return 0.0;
        }

        let periods: Vec<f32> = zero_crossings
            .windows(2)
            .map(|w| (w[1] - w[0]) as f32)
            .collect();

        let mean = periods.iter().sum::<f32>() / periods.len() as f32;
        let variance = periods.iter().map(|&p| (p - mean).powi(2)).sum::<f32>() / periods.len() as f32;

        variance
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_enhancement() {
        let enhancer = CIEnhancer::with_default(44100);

        // 22개 전극의 테스트 진폭
        let electrodes: Vec<f32> = (0..22).map(|i| (i as f32 / 22.0) * 0.5 + 0.1).collect();

        let result = enhancer.enhance(&electrodes, 4, 440.0, 0);

        assert_eq!(result.original_electrodes.len(), 22);
        assert_eq!(result.octave_info.octave, 4);
        assert!(result.octave_info.encoding_methods.len() > 0);
    }

    #[test]
    fn test_vocoder_simulator() {
        let simulator = CIVocoderSimulator::new(44100);

        // 1초 440Hz 사인파
        let audio: Vec<f32> = (0..44100)
            .map(|i| (2.0 * PI * 440.0 * i as f32 / 44100.0).sin())
            .collect();

        let ci_output = simulator.simulate_ci_hearing(&audio);

        assert_eq!(ci_output.len(), audio.len());
        // 출력이 있어야 함
        assert!(ci_output.iter().any(|&x| x.abs() > 0.001));
    }

    #[test]
    fn test_ab_comparison() {
        let enhancer = CIEnhancer::with_default(44100);
        let simulator = CIVocoderSimulator::new(44100);

        // 테스트 오디오
        let audio: Vec<f32> = (0..44100)
            .map(|i| (2.0 * PI * 440.0 * i as f32 / 44100.0).sin())
            .collect();

        // Enhanced 신호 생성
        let electrodes: Vec<f32> = vec![0.5; 22];
        let enhanced = enhancer.enhance(&electrodes, 4, 440.0, 0);

        // A/B 비교
        let comparison = simulator.create_ab_comparison(&audio, &enhanced);

        assert!(!comparison.original_ci.is_empty());
        assert!(!comparison.enhanced_ci.is_empty());
        assert!(comparison.octave_clarity_score >= 0.0);
        assert!(comparison.octave_clarity_score <= 1.0);
    }
}
