//! WIA WASM Bindings
//!
//! 브라우저에서 WIA 표준 실행
//!
//! ```javascript
//! import init, { WIA } from '@anthropic/wia-wasm';
//!
//! await init();
//! const wia = new WIA();
//! const score = await wia.getA11yScore('https://example.com');
//! console.log(score);
//! ```
//!
//! 홍익인간 (弘益人間)

use wasm_bindgen::prelude::*;
use serde::{Deserialize, Serialize};

/// WIA WASM 인스턴스
#[wasm_bindgen]
pub struct WIA {
    config: WIAConfig,
}

#[derive(Clone, Serialize, Deserialize)]
struct WIAConfig {
    api_endpoint: String,
    language: String,
}

#[wasm_bindgen]
impl WIA {
    /// 새 인스턴스 생성
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self {
            config: WIAConfig {
                api_endpoint: "https://api.wia.live".to_string(),
                language: "ko".to_string(),
            },
        }
    }

    /// 언어 설정
    #[wasm_bindgen]
    pub fn set_language(&mut self, lang: &str) {
        self.config.language = lang.to_string();
    }

    /// 접근성 점수 (a11y.wiabooks.store 연동)
    #[wasm_bindgen]
    pub async fn get_a11y_score(&self, url: &str) -> Result<JsValue, JsValue> {
        let api_url = format!(
            "https://a11y.wiabooks.store/api/score?url={}&lang={}",
            url, self.config.language
        );

        // Fetch API 사용
        let window = web_sys::window().unwrap();
        let resp = JsFuture::from(window.fetch_with_str(&api_url)).await?;
        let resp: web_sys::Response = resp.dyn_into()?;
        let json = JsFuture::from(resp.json()?).await?;

        Ok(json)
    }

    /// 버전 정보
    #[wasm_bindgen]
    pub fn version() -> String {
        "1.0.0".to_string()
    }

    /// 철학
    #[wasm_bindgen]
    pub fn philosophy() -> String {
        "홍익인간 (弘益人間) - Benefit All Humanity".to_string()
    }
}

// ============================================
// Voice 모듈
// ============================================

#[wasm_bindgen]
pub mod voice {
    use super::*;

    /// 음성 의도 파싱 (브라우저용)
    #[wasm_bindgen]
    pub fn parse_intent(text: &str) -> JsValue {
        let result = serde_json::json!({
            "text": text,
            "intent": detect_intent(text),
            "confidence": 0.95
        });
        JsValue::from_str(&result.to_string())
    }

    fn detect_intent(text: &str) -> &str {
        if text.contains("음악") || text.contains("노래") {
            "music.play"
        } else if text.contains("볼륨") {
            "volume.adjust"
        } else if text.contains("집") || text.contains("가") {
            "navigation.home"
        } else {
            "unknown"
        }
    }
}

// ============================================
// CI 모듈
// ============================================

#[wasm_bindgen]
pub mod ci {
    use super::*;

    /// 옥타브 감지 (WebAudio API용)
    #[wasm_bindgen]
    pub fn detect_octave(frequency: f32) -> JsValue {
        // A4 = 440Hz 기준
        let a4 = 440.0f32;
        let a0 = 27.5f32;

        let octave = (frequency / a0).log2().floor() as i32;
        let note_num = ((12.0 * (frequency / a0).log2()) % 12.0).round() as i32;

        let notes = ["A", "A#", "B", "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#"];
        let note = notes[note_num as usize % 12];

        let result = serde_json::json!({
            "frequency": frequency,
            "octave": octave,
            "note": note,
            "isMusic": true
        });
        JsValue::from_str(&result.to_string())
    }
}

// ============================================
// A11Y 모듈 (a11y.wiabooks.store 연동)
// ============================================

#[wasm_bindgen]
pub mod a11y {
    use super::*;

    /// 211개 언어 목록
    #[wasm_bindgen]
    pub fn supported_languages() -> JsValue {
        let languages = vec![
            "ko", "en", "ja", "zh", "es", "fr", "de", "it", "pt", "ru",
            "ar", "hi", "bn", "pa", "jv", "vi", "th", "tr", "fa", "ur",
            // ... 211개
        ];
        JsValue::from_str(&serde_json::to_string(&languages).unwrap())
    }

    /// 접근성 체크 (로컬)
    #[wasm_bindgen]
    pub fn check_element(element_html: &str) -> JsValue {
        let mut issues: Vec<String> = vec![];

        // 이미지 alt 체크
        if element_html.contains("<img") && !element_html.contains("alt=") {
            issues.push("Missing alt attribute on image".to_string());
        }

        // 색상 대비 (간단 체크)
        if element_html.contains("color:") && !element_html.contains("background") {
            issues.push("Check color contrast ratio".to_string());
        }

        let result = serde_json::json!({
            "passed": issues.is_empty(),
            "issues": issues,
            "checkedAt": chrono::Utc::now().to_rfc3339()
        });
        JsValue::from_str(&result.to_string())
    }
}

// JavaScript에서 사용:
// ```javascript
// import init, { WIA, voice, ci, a11y } from '@anthropic/wia-wasm';
//
// await init();
//
// // 음성 의도 파싱
// const intent = voice.parse_intent("음악 틀어줘");
//
// // 옥타브 감지
// const octave = ci.detect_octave(440.0);  // A4
//
// // 접근성 체크
// const check = a11y.check_element('<img src="test.jpg">');
//
// // 전체 API
// const wia = new WIA();
// const score = await wia.get_a11y_score('https://example.com');
// ```
