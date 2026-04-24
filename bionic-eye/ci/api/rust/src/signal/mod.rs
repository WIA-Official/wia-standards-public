//! CI Signal Analysis Module
//!
//! Phase 1: CI 신호 형식 분석 및 처리

use serde::{Deserialize, Serialize};
use crate::{CI_ELECTRODE_COUNT, COCHLEAR_CENTER_FREQUENCIES};

/// CI 신호 데이터
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CISignal {
    /// 버전
    pub version: String,
    /// 타입
    pub signal_type: String,
    /// 디바이스 정보
    pub device: DeviceInfo,
    /// 타임스탬프 (ms)
    pub timestamp: u64,
    /// 샘플레이트
    pub sample_rate: u32,
    /// 전극 활성화 데이터
    pub electrodes: Vec<ElectrodeActivation>,
    /// 분석 결과
    pub analysis: Option<SignalAnalysis>,
}

/// 디바이스 정보
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceInfo {
    pub manufacturer: String,
    pub model: String,
    pub strategy: String,
    pub channels: u8,
}

/// 전극 활성화 데이터
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ElectrodeActivation {
    /// 전극 ID (1-22)
    pub id: u8,
    /// 주파수 범위
    pub frequency: FrequencyRange,
    /// 진폭 (0.0 - 1.0)
    pub amplitude: f32,
    /// 펄스율 (Hz)
    pub pulse_rate: u32,
    /// 펄스폭 (μs)
    pub pulse_width: u32,
}

/// 주파수 범위
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FrequencyRange {
    pub min: f32,
    pub max: f32,
    pub center: f32,
}

/// 신호 분석 결과
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignalAnalysis {
    /// 지배 주파수 (Hz)
    pub dominant_frequency: f32,
    /// 추정 피치
    pub estimated_pitch: String,
    /// 옥타브
    pub octave: u8,
    /// 신뢰도 (0.0 - 1.0)
    pub confidence: f32,
}

/// 주파수-전극 매핑기
pub struct FrequencyMapper {
    /// 전극별 중심 주파수
    center_frequencies: [f32; CI_ELECTRODE_COUNT],
    /// 전극별 주파수 범위 (min, max)
    frequency_ranges: [(f32, f32); CI_ELECTRODE_COUNT],
}

impl FrequencyMapper {
    /// Cochlear 기본 매핑으로 생성
    pub fn new_cochlear() -> Self {
        let mut frequency_ranges = [(0.0f32, 0.0f32); CI_ELECTRODE_COUNT];

        for i in 0..CI_ELECTRODE_COUNT {
            let center = COCHLEAR_CENTER_FREQUENCIES[i];
            // 대략적인 대역폭 계산 (ERB 기반)
            let bandwidth = center * 0.25;
            frequency_ranges[i] = (center - bandwidth / 2.0, center + bandwidth / 2.0);
        }

        Self {
            center_frequencies: COCHLEAR_CENTER_FREQUENCIES,
            frequency_ranges,
        }
    }

    /// 주파수에 해당하는 전극 ID 반환 (1-22)
    pub fn frequency_to_electrode(&self, frequency: f32) -> Option<u8> {
        for (i, (min, max)) in self.frequency_ranges.iter().enumerate() {
            if frequency >= *min && frequency <= *max {
                return Some((i + 1) as u8);
            }
        }

        // 범위 밖이면 가장 가까운 전극
        if frequency < self.frequency_ranges[0].0 {
            return Some(1);
        }
        if frequency > self.frequency_ranges[CI_ELECTRODE_COUNT - 1].1 {
            return Some(CI_ELECTRODE_COUNT as u8);
        }

        None
    }

    /// 전극 ID에 해당하는 중심 주파수 반환
    pub fn electrode_to_frequency(&self, electrode_id: u8) -> Option<f32> {
        let idx = electrode_id.saturating_sub(1) as usize;
        if idx < CI_ELECTRODE_COUNT {
            Some(self.center_frequencies[idx])
        } else {
            None
        }
    }

    /// 전극 ID에 해당하는 주파수 범위 반환
    pub fn electrode_to_range(&self, electrode_id: u8) -> Option<FrequencyRange> {
        let idx = electrode_id.saturating_sub(1) as usize;
        if idx < CI_ELECTRODE_COUNT {
            let (min, max) = self.frequency_ranges[idx];
            Some(FrequencyRange {
                min,
                max,
                center: self.center_frequencies[idx],
            })
        } else {
            None
        }
    }

    /// 하모닉에 해당하는 전극들 반환
    pub fn harmonic_electrodes(&self, fundamental: f32, harmonic_count: u8) -> Vec<u8> {
        let mut electrodes = Vec::new();

        for h in 1..=harmonic_count {
            let freq = fundamental * h as f32;
            if let Some(electrode) = self.frequency_to_electrode(freq) {
                if !electrodes.contains(&electrode) {
                    electrodes.push(electrode);
                }
            }
        }

        electrodes
    }
}

/// 신호 분석기
pub struct SignalAnalyzer {
    sample_rate: u32,
    mapper: FrequencyMapper,
}

impl SignalAnalyzer {
    pub fn new(sample_rate: u32) -> Self {
        Self {
            sample_rate,
            mapper: FrequencyMapper::new_cochlear(),
        }
    }

    /// 오디오 프레임에서 스펙트럼 분석
    pub fn analyze_spectrum(&self, frame: &[f32]) -> Vec<f32> {
        use rustfft::{FftPlanner, num_complex::Complex};

        let n = frame.len();
        let mut planner = FftPlanner::new();
        let fft = planner.plan_fft_forward(n);

        // 윈도우 함수 적용 (Hann)
        let windowed: Vec<Complex<f32>> = frame
            .iter()
            .enumerate()
            .map(|(i, &sample)| {
                let window = 0.5 * (1.0 - (2.0 * std::f32::consts::PI * i as f32 / n as f32).cos());
                Complex::new(sample * window, 0.0)
            })
            .collect();

        let mut buffer = windowed;
        fft.process(&mut buffer);

        // 크기 스펙트럼 반환 (절반만 - Nyquist)
        buffer[..n / 2]
            .iter()
            .map(|c| (c.re * c.re + c.im * c.im).sqrt())
            .collect()
    }

    /// 스펙트럼에서 지배 주파수 찾기
    pub fn find_dominant_frequency(&self, spectrum: &[f32]) -> Option<f32> {
        if spectrum.is_empty() {
            return None;
        }

        let (max_idx, _) = spectrum
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())?;

        let freq = max_idx as f32 * self.sample_rate as f32 / (spectrum.len() * 2) as f32;
        Some(freq)
    }

    /// 전극 활성화 패턴 분석
    pub fn analyze_electrode_pattern(&self, frame: &[f32]) -> Vec<ElectrodeActivation> {
        let spectrum = self.analyze_spectrum(frame);
        let freq_resolution = self.sample_rate as f32 / (spectrum.len() * 2) as f32;

        let mut activations = Vec::new();

        // 각 전극에 해당하는 에너지 계산
        for electrode_id in 1..=CI_ELECTRODE_COUNT as u8 {
            if let Some(range) = self.mapper.electrode_to_range(electrode_id) {
                let start_bin = (range.min / freq_resolution) as usize;
                let end_bin = ((range.max / freq_resolution) as usize).min(spectrum.len());

                if start_bin < end_bin && end_bin <= spectrum.len() {
                    let _energy: f32 = spectrum[start_bin..end_bin].iter().sum();
                    let max_energy = spectrum[start_bin..end_bin]
                        .iter()
                        .cloned()
                        .fold(0.0f32, f32::max);

                    // 정규화된 진폭
                    let amplitude = (max_energy / spectrum.iter().cloned().fold(0.0f32, f32::max))
                        .min(1.0)
                        .max(0.0);

                    if amplitude > 0.05 {
                        activations.push(ElectrodeActivation {
                            id: electrode_id,
                            frequency: range,
                            amplitude,
                            pulse_rate: 900,
                            pulse_width: 25,
                        });
                    }
                }
            }
        }

        activations
    }
}

impl Default for FrequencyMapper {
    fn default() -> Self {
        Self::new_cochlear()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frequency_mapper() {
        let mapper = FrequencyMapper::new_cochlear();

        // A4 (440Hz) → 전극 7 근처
        let electrode = mapper.frequency_to_electrode(440.0);
        assert!(electrode.is_some());

        // 전극 1 → 저주파
        let freq = mapper.electrode_to_frequency(1);
        assert_eq!(freq, Some(250.0));
    }

    #[test]
    fn test_harmonic_electrodes() {
        let mapper = FrequencyMapper::new_cochlear();

        // A4 (440Hz)의 하모닉 → 440, 880, 1320, 1760...
        let electrodes = mapper.harmonic_electrodes(440.0, 4);
        assert!(!electrodes.is_empty());
    }
}
