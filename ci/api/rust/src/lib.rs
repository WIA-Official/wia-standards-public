//! # WIA CI Octave Enhancement
//!
//! 인공와우(Cochlear Implant) 사용자를 위한 옥타브 인핸스먼트 라이브러리
//!
//! ## 철학
//!
//! **홍익인간 (弘益人間)** - 널리 인간을 이롭게 하라
//!
//! ## 주요 기능
//!
//! - 실시간 옥타브 검출 (OctaveYIN 알고리즘)
//! - CI 신호 인핸스먼트 (Temporal Modulation)
//! - REST/WebSocket API
//!
//! ## Example
//!
//! ```rust
//! use wia_ci::{OctaveDetector, OctaveResult};
//!
//! let detector = OctaveDetector::new(16000); // 16kHz sample rate
//! let audio_frame: Vec<f32> = vec![/* audio samples */];
//!
//! if let Some(result) = detector.detect(&audio_frame) {
//!     println!("F0: {} Hz, Octave: {}, Note: {}",
//!         result.f0, result.octave, result.note);
//! }
//! ```

pub mod signal;
pub mod octave;
pub mod enhancement;
pub mod server;

// Re-exports
pub use signal::{CISignal, ElectrodeActivation, FrequencyMapper, SignalAnalyzer};
pub use octave::{OctaveYIN, OctaveResult, YINConfig, StreamingOctaveDetector};
pub use enhancement::{CIEnhancer, EnhancedCISignal, EnhancementConfig, CIVocoderSimulator};
pub use server::{create_app, run_server, ServerConfig};

/// CI 표준 전극 수
pub const CI_ELECTRODE_COUNT: usize = 22;

/// 기본 샘플레이트 (Hz)
pub const DEFAULT_SAMPLE_RATE: u32 = 16000;

/// Cochlear 22-electrode 중심 주파수 (Hz)
/// Electrode 22 (apex, low freq) → Electrode 1 (base, high freq)
pub const COCHLEAR_CENTER_FREQUENCIES: [f32; 22] = [
    250.0, 375.0, 500.0, 625.0, 750.0, 875.0, 1000.0, 1125.0,
    1250.0, 1438.0, 1688.0, 1938.0, 2188.0, 2500.0, 2875.0,
    3313.0, 3813.0, 4375.0, 5000.0, 5688.0, 6500.0, 7438.0,
];

/// 옥타브별 변조 주파수 매핑 (Hz)
/// 저옥타브 = 저변조주파수
pub fn octave_to_modulation_frequency(octave: u8) -> f32 {
    match octave {
        0..=2 => 55.0,
        3 => 110.0,
        4 => 110.0,
        5 => 220.0,
        6 => 220.0,
        7..=8 => 440.0,
        _ => 110.0,
    }
}

/// 주파수를 옥타브로 변환
pub fn frequency_to_octave(frequency: f32) -> u8 {
    if frequency <= 0.0 {
        return 0;
    }
    // A4 = 440Hz = octave 4
    // 옥타브는 주파수가 2배가 될 때마다 1 증가
    let a4 = 440.0;
    let octave = 4.0 + (frequency / a4).log2();
    octave.round().max(0.0).min(8.0) as u8
}

/// 주파수를 음이름으로 변환
pub fn frequency_to_note(frequency: f32) -> &'static str {
    if frequency <= 0.0 {
        return "?";
    }

    // A4 = 440Hz 기준
    let a4 = 440.0;
    let semitones = 12.0 * (frequency / a4).log2();
    let note_index = ((semitones.round() as i32 % 12) + 12) % 12;

    const NOTES: [&str; 12] = [
        "A", "A#", "B", "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#"
    ];

    NOTES[note_index as usize]
}

/// 주파수를 cents로 변환 (가장 가까운 반음 기준 -50 ~ +50)
pub fn frequency_to_cents(frequency: f32) -> i32 {
    if frequency <= 0.0 {
        return 0;
    }

    let a4 = 440.0;
    let semitones = 12.0 * (frequency / a4).log2();
    let cents = ((semitones - semitones.round()) * 100.0).round() as i32;
    cents.max(-50).min(50)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frequency_to_octave() {
        assert_eq!(frequency_to_octave(440.0), 4); // A4
        assert_eq!(frequency_to_octave(880.0), 5); // A5
        assert_eq!(frequency_to_octave(220.0), 3); // A3
        assert_eq!(frequency_to_octave(261.63), 4); // C4 (middle C)
    }

    #[test]
    fn test_frequency_to_note() {
        assert_eq!(frequency_to_note(440.0), "A");
        assert_eq!(frequency_to_note(261.63), "C");
        assert_eq!(frequency_to_note(329.63), "E");
    }

    #[test]
    fn test_octave_to_modulation_frequency() {
        assert_eq!(octave_to_modulation_frequency(2), 55.0);
        assert_eq!(octave_to_modulation_frequency(4), 110.0);
        assert_eq!(octave_to_modulation_frequency(6), 220.0);
    }
}
