//! Octave Detection Module - OctaveYIN Algorithm
//!
//! Phase 2: 정확한 옥타브 결정을 위한 피치 검출 알고리즘

use std::f32::consts::PI;
use crate::{frequency_to_octave, frequency_to_note};

/// YIN 알고리즘 파라미터
pub struct YINConfig {
    /// 샘플레이트
    pub sample_rate: u32,
    /// YIN 임계값 (0.1 ~ 0.2 권장)
    pub threshold: f32,
    /// 최소 검출 주파수 (Hz)
    pub min_frequency: f32,
    /// 최대 검출 주파수 (Hz)
    pub max_frequency: f32,
}

impl Default for YINConfig {
    fn default() -> Self {
        Self {
            sample_rate: 44100,
            threshold: 0.15,
            min_frequency: 50.0,   // A1 근처
            max_frequency: 2000.0, // C7 근처
        }
    }
}

/// 옥타브 검출 결과
#[derive(Debug, Clone)]
pub struct OctaveResult {
    /// 검출된 기본 주파수 (Hz)
    pub fundamental_frequency: f32,
    /// 옥타브 번호 (0-9)
    pub octave: u8,
    /// 음표 이름 (C, C#, D, ...)
    pub note: String,
    /// 신뢰도 (0.0 - 1.0)
    pub confidence: f32,
    /// 하모닉 분석 결과
    pub harmonics: Vec<HarmonicInfo>,
    /// 옥타브 결정 방법
    pub determination_method: OctaveDeterminationMethod,
}

/// 하모닉 정보
#[derive(Debug, Clone)]
pub struct HarmonicInfo {
    /// 하모닉 번호 (1 = 기본음)
    pub harmonic_number: u8,
    /// 주파수 (Hz)
    pub frequency: f32,
    /// 상대적 진폭 (기본음 대비)
    pub relative_amplitude: f32,
}

/// 옥타브 결정 방법
#[derive(Debug, Clone, PartialEq)]
pub enum OctaveDeterminationMethod {
    /// YIN 직접 검출
    YINDirect,
    /// 하모닉 분석 보정
    HarmonicAnalysis,
    /// Subharmonic 검증
    SubharmonicVerification,
}

/// OctaveYIN - 옥타브 정확도를 보장하는 피치 검출기
pub struct OctaveYIN {
    config: YINConfig,
    /// YIN 버퍼
    yin_buffer: Vec<f32>,
}

impl OctaveYIN {
    pub fn new(config: YINConfig) -> Self {
        Self {
            config,
            yin_buffer: Vec::new(),
        }
    }

    pub fn with_default() -> Self {
        Self::new(YINConfig::default())
    }

    /// 오디오 프레임에서 피치 및 옥타브 검출
    pub fn detect(&mut self, frame: &[f32]) -> Option<OctaveResult> {
        // Step 1: YIN 알고리즘으로 초기 피치 추정
        let initial_pitch = self.yin_pitch(frame)?;

        // Step 2: 하모닉 분석
        let harmonics = self.analyze_harmonics(frame, initial_pitch);

        // Step 3: 옥타브 검증 및 보정
        let (verified_pitch, method) = self.verify_octave(frame, initial_pitch, &harmonics);

        // Step 4: 신뢰도 계산
        let confidence = self.calculate_confidence(&harmonics);

        Some(OctaveResult {
            fundamental_frequency: verified_pitch,
            octave: frequency_to_octave(verified_pitch),
            note: frequency_to_note(verified_pitch).to_string(),
            confidence,
            harmonics,
            determination_method: method,
        })
    }

    /// YIN 피치 검출 알고리즘
    fn yin_pitch(&mut self, frame: &[f32]) -> Option<f32> {
        let n = frame.len();
        let tau_max = (self.config.sample_rate as f32 / self.config.min_frequency) as usize;
        let tau_min = (self.config.sample_rate as f32 / self.config.max_frequency) as usize;

        if n < tau_max * 2 {
            return None;
        }

        // Step 1: 차이 함수 계산
        self.yin_buffer.clear();
        self.yin_buffer.resize(tau_max, 0.0);

        for tau in 1..tau_max {
            let mut sum = 0.0;
            for i in 0..(n - tau_max) {
                let diff = frame[i] - frame[i + tau];
                sum += diff * diff;
            }
            self.yin_buffer[tau] = sum;
        }

        // Step 2: 누적 평균 정규화 차이 함수 (CMNDF)
        self.yin_buffer[0] = 1.0;
        let mut running_sum = 0.0;

        for tau in 1..tau_max {
            running_sum += self.yin_buffer[tau];
            if running_sum > 0.0 {
                self.yin_buffer[tau] *= tau as f32 / running_sum;
            }
        }

        // Step 3: 절대 임계값 기반 피치 검출
        let mut best_tau = tau_min;

        for tau in tau_min..tau_max {
            if self.yin_buffer[tau] < self.config.threshold {
                // 국소 최솟값 찾기
                let mut local_tau = tau;
                while local_tau + 1 < tau_max && self.yin_buffer[local_tau + 1] < self.yin_buffer[local_tau] {
                    local_tau += 1;
                }
                best_tau = local_tau;
                break;
            }
        }

        // Step 4: 포물선 보간으로 정밀도 향상
        let refined_tau = self.parabolic_interpolation(best_tau);

        if refined_tau > 0.0 {
            Some(self.config.sample_rate as f32 / refined_tau)
        } else {
            None
        }
    }

    /// 포물선 보간
    fn parabolic_interpolation(&self, tau: usize) -> f32 {
        if tau == 0 || tau >= self.yin_buffer.len() - 1 {
            return tau as f32;
        }

        let s0 = self.yin_buffer[tau - 1];
        let s1 = self.yin_buffer[tau];
        let s2 = self.yin_buffer[tau + 1];

        let adjustment = (s2 - s0) / (2.0 * (2.0 * s1 - s2 - s0));

        if adjustment.is_finite() {
            tau as f32 + adjustment
        } else {
            tau as f32
        }
    }

    /// 하모닉 분석
    fn analyze_harmonics(&self, frame: &[f32], fundamental: f32) -> Vec<HarmonicInfo> {
        use rustfft::{FftPlanner, num_complex::Complex};

        let n = frame.len();
        let mut planner = FftPlanner::new();
        let fft = planner.plan_fft_forward(n);

        // Hann 윈도우 적용
        let windowed: Vec<Complex<f32>> = frame
            .iter()
            .enumerate()
            .map(|(i, &s)| {
                let w = 0.5 * (1.0 - (2.0 * PI * i as f32 / n as f32).cos());
                Complex::new(s * w, 0.0)
            })
            .collect();

        let mut buffer = windowed;
        fft.process(&mut buffer);

        // 크기 스펙트럼
        let spectrum: Vec<f32> = buffer[..n / 2]
            .iter()
            .map(|c| (c.re * c.re + c.im * c.im).sqrt())
            .collect();

        let freq_resolution = self.config.sample_rate as f32 / n as f32;

        // 기본음 진폭
        let fundamental_bin = (fundamental / freq_resolution) as usize;
        let fundamental_amp = if fundamental_bin < spectrum.len() {
            self.peak_amplitude(&spectrum, fundamental_bin)
        } else {
            1.0
        };

        // 하모닉 검출 (최대 8개)
        let mut harmonics = Vec::new();

        for h in 1..=8 {
            let harmonic_freq = fundamental * h as f32;
            let harmonic_bin = (harmonic_freq / freq_resolution) as usize;

            if harmonic_bin >= spectrum.len() {
                break;
            }

            let amp = self.peak_amplitude(&spectrum, harmonic_bin);
            let relative_amp = if fundamental_amp > 0.0 {
                amp / fundamental_amp
            } else {
                0.0
            };

            harmonics.push(HarmonicInfo {
                harmonic_number: h,
                frequency: harmonic_freq,
                relative_amplitude: relative_amp,
            });
        }

        harmonics
    }

    /// 피크 주변 진폭
    fn peak_amplitude(&self, spectrum: &[f32], bin: usize) -> f32 {
        let search_range = 3;
        let start = bin.saturating_sub(search_range);
        let end = (bin + search_range + 1).min(spectrum.len());

        spectrum[start..end]
            .iter()
            .cloned()
            .fold(0.0f32, f32::max)
    }

    /// 옥타브 검증 - 핵심 알고리즘
    fn verify_octave(
        &self,
        frame: &[f32],
        initial_pitch: f32,
        harmonics: &[HarmonicInfo],
    ) -> (f32, OctaveDeterminationMethod) {
        // 방법 1: 하모닉 패턴 분석
        if let Some(corrected) = self.harmonic_pattern_check(initial_pitch, harmonics) {
            return (corrected, OctaveDeterminationMethod::HarmonicAnalysis);
        }

        // 방법 2: 서브하모닉 검증
        if let Some(corrected) = self.subharmonic_check(frame, initial_pitch) {
            return (corrected, OctaveDeterminationMethod::SubharmonicVerification);
        }

        // 보정 불필요
        (initial_pitch, OctaveDeterminationMethod::YINDirect)
    }

    /// 하모닉 패턴으로 옥타브 오류 검출
    fn harmonic_pattern_check(&self, pitch: f32, harmonics: &[HarmonicInfo]) -> Option<f32> {
        if harmonics.len() < 3 {
            return None;
        }

        // 옥타브 오류 징후: 짝수 하모닉만 강함
        // (실제 기본음이 한 옥타브 아래일 가능성)
        let even_harmonics: Vec<_> = harmonics
            .iter()
            .filter(|h| h.harmonic_number % 2 == 0)
            .collect();

        let odd_harmonics: Vec<_> = harmonics
            .iter()
            .filter(|h| h.harmonic_number % 2 == 1)
            .collect();

        if even_harmonics.is_empty() || odd_harmonics.is_empty() {
            return None;
        }

        let even_avg: f32 = even_harmonics.iter().map(|h| h.relative_amplitude).sum::<f32>()
            / even_harmonics.len() as f32;
        let odd_avg: f32 = odd_harmonics.iter().map(|h| h.relative_amplitude).sum::<f32>()
            / odd_harmonics.len() as f32;

        // 짝수 하모닉이 훨씬 강하면 → 한 옥타브 낮추기
        if even_avg > odd_avg * 2.0 && pitch > 100.0 {
            return Some(pitch / 2.0);
        }

        // 홀수 하모닉만 강하면 → 정상
        None
    }

    /// 서브하모닉 존재 확인
    fn subharmonic_check(&self, frame: &[f32], pitch: f32) -> Option<f32> {
        use rustfft::{FftPlanner, num_complex::Complex};

        let n = frame.len();
        let mut planner = FftPlanner::new();
        let fft = planner.plan_fft_forward(n);

        let windowed: Vec<Complex<f32>> = frame
            .iter()
            .enumerate()
            .map(|(i, &s)| {
                let w = 0.5 * (1.0 - (2.0 * PI * i as f32 / n as f32).cos());
                Complex::new(s * w, 0.0)
            })
            .collect();

        let mut buffer = windowed;
        fft.process(&mut buffer);

        let spectrum: Vec<f32> = buffer[..n / 2]
            .iter()
            .map(|c| (c.re * c.re + c.im * c.im).sqrt())
            .collect();

        let freq_resolution = self.config.sample_rate as f32 / n as f32;

        // 서브하모닉 (pitch/2) 확인
        let subharmonic_freq = pitch / 2.0;
        if subharmonic_freq < self.config.min_frequency {
            return None;
        }

        let sub_bin = (subharmonic_freq / freq_resolution) as usize;
        let pitch_bin = (pitch / freq_resolution) as usize;

        if sub_bin >= spectrum.len() || pitch_bin >= spectrum.len() {
            return None;
        }

        let sub_amp = self.peak_amplitude(&spectrum, sub_bin);
        let pitch_amp = self.peak_amplitude(&spectrum, pitch_bin);

        // 서브하모닉이 유의미하게 존재하면 → 한 옥타브 낮추기
        if sub_amp > pitch_amp * 0.3 {
            return Some(subharmonic_freq);
        }

        None
    }

    /// 신뢰도 계산
    fn calculate_confidence(&self, harmonics: &[HarmonicInfo]) -> f32 {
        if harmonics.is_empty() {
            return 0.0;
        }

        // 하모닉 일관성 기반 신뢰도
        let mut consistency_score = 0.0;
        let mut count = 0;

        for h in harmonics.iter().take(5) {
            if h.relative_amplitude > 0.1 {
                consistency_score += 1.0;
            }
            count += 1;
        }

        let harmonic_confidence = consistency_score / count as f32;

        // YIN 품질 기반 보정
        let yin_min = self.yin_buffer.iter().cloned().fold(f32::MAX, f32::min);
        let yin_confidence = 1.0 - yin_min.min(1.0);

        // 가중 평균
        (harmonic_confidence * 0.6 + yin_confidence * 0.4).min(1.0).max(0.0)
    }
}

/// 실시간 스트리밍용 옥타브 검출기
pub struct StreamingOctaveDetector {
    yin: OctaveYIN,
    buffer: Vec<f32>,
    hop_size: usize,
    frame_size: usize,
    /// 결과 스무딩을 위한 히스토리
    history: Vec<OctaveResult>,
    history_size: usize,
}

impl StreamingOctaveDetector {
    pub fn new(sample_rate: u32, frame_size: usize, hop_size: usize) -> Self {
        let config = YINConfig {
            sample_rate,
            ..Default::default()
        };

        Self {
            yin: OctaveYIN::new(config),
            buffer: Vec::with_capacity(frame_size * 2),
            hop_size,
            frame_size,
            history: Vec::new(),
            history_size: 5,
        }
    }

    /// 오디오 샘플 추가 및 검출
    pub fn process(&mut self, samples: &[f32]) -> Vec<OctaveResult> {
        self.buffer.extend_from_slice(samples);

        let mut results = Vec::new();

        while self.buffer.len() >= self.frame_size {
            let frame: Vec<f32> = self.buffer[..self.frame_size].to_vec();

            if let Some(result) = self.yin.detect(&frame) {
                // 히스토리 기반 스무딩
                let smoothed = self.smooth_result(result);
                results.push(smoothed);
            }

            // Hop
            self.buffer.drain(..self.hop_size);
        }

        results
    }

    /// 결과 스무딩 (점프 방지)
    fn smooth_result(&mut self, result: OctaveResult) -> OctaveResult {
        self.history.push(result.clone());

        if self.history.len() > self.history_size {
            self.history.remove(0);
        }

        if self.history.len() < 3 {
            return result;
        }

        // 옥타브 투표
        let mut octave_votes = [0u8; 10];
        for r in &self.history {
            if (r.octave as usize) < 10 {
                octave_votes[r.octave as usize] += 1;
            }
        }

        let majority_octave = octave_votes
            .iter()
            .enumerate()
            .max_by_key(|(_, &v)| v)
            .map(|(o, _)| o as u8)
            .unwrap_or(result.octave);

        // 현재 옥타브가 다수결과 다르면 보정
        if result.octave != majority_octave && result.confidence < 0.8 {
            let octave_diff = result.octave as i8 - majority_octave as i8;
            let correction_factor = 2.0f32.powi(octave_diff as i32);

            OctaveResult {
                fundamental_frequency: result.fundamental_frequency / correction_factor,
                octave: majority_octave,
                note: result.note,
                confidence: result.confidence * 0.9,
                harmonics: result.harmonics,
                determination_method: OctaveDeterminationMethod::HarmonicAnalysis,
            }
        } else {
            result
        }
    }

    /// 버퍼 클리어
    pub fn reset(&mut self) {
        self.buffer.clear();
        self.history.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    fn generate_sine(frequency: f32, sample_rate: u32, duration_samples: usize) -> Vec<f32> {
        (0..duration_samples)
            .map(|i| (2.0 * PI * frequency * i as f32 / sample_rate as f32).sin())
            .collect()
    }

    fn generate_with_harmonics(
        fundamental: f32,
        harmonics: &[(u8, f32)], // (harmonic_number, amplitude)
        sample_rate: u32,
        duration_samples: usize,
    ) -> Vec<f32> {
        let mut signal = vec![0.0f32; duration_samples];

        // 기본음
        for i in 0..duration_samples {
            signal[i] += (2.0 * PI * fundamental * i as f32 / sample_rate as f32).sin();
        }

        // 하모닉
        for (h, amp) in harmonics {
            let freq = fundamental * *h as f32;
            for i in 0..duration_samples {
                signal[i] += amp * (2.0 * PI * freq * i as f32 / sample_rate as f32).sin();
            }
        }

        signal
    }

    #[test]
    fn test_pure_tone_detection() {
        let sample_rate = 44100;
        let mut yin = OctaveYIN::with_default();

        // A4 = 440 Hz
        let signal = generate_sine(440.0, sample_rate, 4096);
        let result = yin.detect(&signal);

        assert!(result.is_some());
        let result = result.unwrap();

        // 주파수 오차 5% 이내
        assert!((result.fundamental_frequency - 440.0).abs() < 22.0);
        assert_eq!(result.octave, 4);
        assert_eq!(result.note, "A");
    }

    #[test]
    fn test_octave_accuracy() {
        let sample_rate = 44100;
        let mut yin = OctaveYIN::with_default();

        // 같은 음, 다른 옥타브 테스트
        let test_cases = [
            (220.0, 3, "A"),   // A3
            (440.0, 4, "A"),   // A4
            (880.0, 5, "A"),   // A5
            (261.63, 4, "C"),  // C4 (middle C)
            (523.25, 5, "C"),  // C5
        ];

        for (freq, expected_octave, expected_note) in test_cases {
            let signal = generate_sine(freq, sample_rate, 4096);
            let result = yin.detect(&signal).unwrap();

            assert_eq!(
                result.octave, expected_octave,
                "Octave mismatch for {} Hz: expected {}, got {}",
                freq, expected_octave, result.octave
            );
        }
    }

    #[test]
    fn test_harmonic_rich_signal() {
        let sample_rate = 44100;
        let mut yin = OctaveYIN::with_default();

        // 하모닉이 풍부한 신호 (기타/피아노 같은)
        let fundamental = 220.0; // A3
        let harmonics = [
            (2, 0.8),  // 2nd harmonic
            (3, 0.6),  // 3rd harmonic
            (4, 0.4),  // 4th harmonic
            (5, 0.3),  // 5th harmonic
        ];

        let signal = generate_with_harmonics(fundamental, &harmonics, sample_rate, 4096);
        let result = yin.detect(&signal).unwrap();

        // 기본음을 정확히 찾아야 함 (하모닉에 속지 않고)
        assert!((result.fundamental_frequency - 220.0).abs() < 15.0);
        assert_eq!(result.octave, 3);
    }

    #[test]
    fn test_streaming_detector() {
        let sample_rate = 44100;
        let frame_size = 2048;
        let hop_size = 512;

        let mut detector = StreamingOctaveDetector::new(sample_rate, frame_size, hop_size);

        // 440 Hz 신호 생성
        let signal = generate_sine(440.0, sample_rate, sample_rate as usize); // 1초

        // 청크로 처리
        let chunk_size = 1024;
        let mut all_results = Vec::new();

        for chunk in signal.chunks(chunk_size) {
            let results = detector.process(chunk);
            all_results.extend(results);
        }

        // 충분한 결과가 나와야 함
        assert!(!all_results.is_empty());

        // 대부분 A4로 검출되어야 함
        let a4_count = all_results.iter().filter(|r| r.octave == 4).count();
        assert!(a4_count > all_results.len() / 2);
    }
}
