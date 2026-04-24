//! CI Enhancement API Server
//!
//! Phase 4: REST & WebSocket API for real-time CI signal enhancement

use axum::{
    extract::{State, WebSocketUpgrade, ws::{Message, WebSocket}},
    response::{IntoResponse, Json},
    routing::{get, post},
    Router,
};
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use tokio::sync::RwLock;

use crate::octave::{OctaveYIN, OctaveResult, YINConfig, StreamingOctaveDetector};
use crate::enhancement::{CIEnhancer, CIVocoderSimulator};
use crate::signal::SignalAnalyzer;

/// API 서버 상태
pub struct AppState {
    /// 옥타브 검출기
    octave_detector: RwLock<OctaveYIN>,
    /// CI Enhancer
    enhancer: CIEnhancer,
    /// Vocoder Simulator
    simulator: CIVocoderSimulator,
    /// Signal Analyzer
    analyzer: SignalAnalyzer,
    /// 설정
    config: ServerConfig,
}

/// 서버 설정
#[derive(Debug, Clone)]
pub struct ServerConfig {
    pub host: String,
    pub port: u16,
    pub sample_rate: u32,
}

impl Default for ServerConfig {
    fn default() -> Self {
        Self {
            host: "0.0.0.0".to_string(),
            port: 3000,
            sample_rate: 44100,
        }
    }
}

// ==================== Request/Response Types ====================

/// 피치 검출 요청
#[derive(Debug, Deserialize)]
pub struct PitchDetectionRequest {
    /// 오디오 샘플 (f32 배열)
    pub samples: Vec<f32>,
    /// 샘플레이트 (기본: 44100)
    pub sample_rate: Option<u32>,
}

/// 피치 검출 응답
#[derive(Debug, Serialize)]
pub struct PitchDetectionResponse {
    pub success: bool,
    pub result: Option<OctaveResultDTO>,
    pub error: Option<String>,
}

/// Octave Result DTO
#[derive(Debug, Serialize)]
pub struct OctaveResultDTO {
    pub fundamental_frequency: f32,
    pub octave: u8,
    pub note: String,
    pub confidence: f32,
    pub harmonics: Vec<HarmonicDTO>,
}

#[derive(Debug, Serialize)]
pub struct HarmonicDTO {
    pub harmonic_number: u8,
    pub frequency: f32,
    pub relative_amplitude: f32,
}

impl From<OctaveResult> for OctaveResultDTO {
    fn from(r: OctaveResult) -> Self {
        Self {
            fundamental_frequency: r.fundamental_frequency,
            octave: r.octave,
            note: r.note,
            confidence: r.confidence,
            harmonics: r.harmonics.into_iter().map(|h| HarmonicDTO {
                harmonic_number: h.harmonic_number,
                frequency: h.frequency,
                relative_amplitude: h.relative_amplitude,
            }).collect(),
        }
    }
}

/// Enhancement 요청
#[derive(Debug, Deserialize)]
pub struct EnhancementRequest {
    /// 전극 진폭 (22개)
    pub electrode_amplitudes: Vec<f32>,
    /// 옥타브 (검출된 값 또는 지정)
    pub octave: u8,
    /// 기본 주파수
    pub fundamental_frequency: f32,
    /// 프레임 인덱스 (실시간용)
    pub frame_index: Option<usize>,
}

/// Enhancement 응답
#[derive(Debug, Serialize)]
pub struct EnhancementResponse {
    pub success: bool,
    pub result: Option<EnhancedSignalDTO>,
    pub error: Option<String>,
}

#[derive(Debug, Serialize)]
pub struct EnhancedSignalDTO {
    pub electrodes: Vec<ElectrodeDTO>,
    pub octave_info: OctaveInfoDTO,
    pub processing_time_ms: f32,
}

#[derive(Debug, Serialize)]
pub struct ElectrodeDTO {
    pub electrode_id: u8,
    pub original_amplitude: f32,
    pub enhanced_amplitude: f32,
    pub modulation_frequency: Option<f32>,
}

#[derive(Debug, Serialize)]
pub struct OctaveInfoDTO {
    pub octave: u8,
    pub fundamental_frequency: f32,
    pub encoding_methods: Vec<String>,
}

/// 전체 처리 요청 (오디오 → Enhanced CI)
#[derive(Debug, Deserialize)]
pub struct FullProcessRequest {
    /// 오디오 샘플
    pub samples: Vec<f32>,
    /// 샘플레이트
    pub sample_rate: Option<u32>,
    /// A/B 비교 포함
    pub include_ab_comparison: Option<bool>,
}

/// 전체 처리 응답
#[derive(Debug, Serialize)]
pub struct FullProcessResponse {
    pub success: bool,
    pub pitch: Option<OctaveResultDTO>,
    pub enhancement: Option<EnhancedSignalDTO>,
    pub ab_comparison: Option<ABComparisonDTO>,
    pub error: Option<String>,
}

#[derive(Debug, Serialize)]
pub struct ABComparisonDTO {
    pub original_ci_samples: Vec<f32>,
    pub enhanced_ci_samples: Vec<f32>,
    pub octave_clarity_score: f32,
}

/// 서버 상태 정보
#[derive(Debug, Serialize)]
pub struct ServerInfo {
    pub name: String,
    pub version: String,
    pub sample_rate: u32,
    pub endpoints: Vec<EndpointInfo>,
}

#[derive(Debug, Serialize)]
pub struct EndpointInfo {
    pub method: String,
    pub path: String,
    pub description: String,
}

// ==================== WebSocket Messages ====================

/// WebSocket 입력 메시지
#[derive(Debug, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum WSInputMessage {
    #[serde(rename = "audio_frame")]
    AudioFrame { samples: Vec<f32> },
    #[serde(rename = "config")]
    Config { sample_rate: Option<u32>, threshold: Option<f32> },
    #[serde(rename = "reset")]
    Reset,
}

/// WebSocket 출력 메시지
#[derive(Debug, Serialize)]
#[serde(tag = "type")]
pub enum WSOutputMessage {
    #[serde(rename = "pitch_result")]
    PitchResult { result: OctaveResultDTO },
    #[serde(rename = "enhancement_result")]
    EnhancementResult { result: EnhancedSignalDTO },
    #[serde(rename = "error")]
    Error { message: String },
    #[serde(rename = "status")]
    Status { message: String },
}

// ==================== API Handlers ====================

/// GET / - 서버 정보
async fn get_info(State(state): State<Arc<AppState>>) -> Json<ServerInfo> {
    Json(ServerInfo {
        name: "WIA CI Octave Enhancement API".to_string(),
        version: "0.1.0".to_string(),
        sample_rate: state.config.sample_rate,
        endpoints: vec![
            EndpointInfo {
                method: "GET".to_string(),
                path: "/".to_string(),
                description: "Server info".to_string(),
            },
            EndpointInfo {
                method: "POST".to_string(),
                path: "/api/v1/pitch".to_string(),
                description: "Detect pitch and octave from audio".to_string(),
            },
            EndpointInfo {
                method: "POST".to_string(),
                path: "/api/v1/enhance".to_string(),
                description: "Enhance CI signal with octave encoding".to_string(),
            },
            EndpointInfo {
                method: "POST".to_string(),
                path: "/api/v1/process".to_string(),
                description: "Full pipeline: audio → pitch → enhancement".to_string(),
            },
            EndpointInfo {
                method: "GET".to_string(),
                path: "/api/v1/ws".to_string(),
                description: "WebSocket for real-time processing".to_string(),
            },
        ],
    })
}

/// GET /health - 헬스체크
async fn health_check() -> &'static str {
    "OK"
}

/// POST /api/v1/pitch - 피치 검출
async fn detect_pitch(
    State(state): State<Arc<AppState>>,
    Json(req): Json<PitchDetectionRequest>,
) -> Json<PitchDetectionResponse> {
    if req.samples.is_empty() {
        return Json(PitchDetectionResponse {
            success: false,
            result: None,
            error: Some("No audio samples provided".to_string()),
        });
    }

    let mut detector = state.octave_detector.write().await;

    match detector.detect(&req.samples) {
        Some(result) => Json(PitchDetectionResponse {
            success: true,
            result: Some(result.into()),
            error: None,
        }),
        None => Json(PitchDetectionResponse {
            success: false,
            result: None,
            error: Some("Could not detect pitch".to_string()),
        }),
    }
}

/// POST /api/v1/enhance - CI Enhancement
async fn enhance_signal(
    State(state): State<Arc<AppState>>,
    Json(req): Json<EnhancementRequest>,
) -> Json<EnhancementResponse> {
    if req.electrode_amplitudes.len() != 22 {
        return Json(EnhancementResponse {
            success: false,
            result: None,
            error: Some("electrode_amplitudes must have exactly 22 elements".to_string()),
        });
    }

    let frame_index = req.frame_index.unwrap_or(0);
    let enhanced = state.enhancer.enhance(
        &req.electrode_amplitudes,
        req.octave,
        req.fundamental_frequency,
        frame_index,
    );

    Json(EnhancementResponse {
        success: true,
        result: Some(EnhancedSignalDTO {
            electrodes: enhanced.original_electrodes.iter().map(|e| ElectrodeDTO {
                electrode_id: e.electrode_id,
                original_amplitude: e.original_amplitude,
                enhanced_amplitude: e.enhanced_amplitude,
                modulation_frequency: e.temporal_modulation.as_ref().map(|m| m.frequency),
            }).collect(),
            octave_info: OctaveInfoDTO {
                octave: enhanced.octave_info.octave,
                fundamental_frequency: enhanced.octave_info.fundamental_frequency,
                encoding_methods: enhanced.octave_info.encoding_methods,
            },
            processing_time_ms: enhanced.metadata.processing_time_ms,
        }),
        error: None,
    })
}

/// POST /api/v1/process - 전체 파이프라인
async fn full_process(
    State(state): State<Arc<AppState>>,
    Json(req): Json<FullProcessRequest>,
) -> Json<FullProcessResponse> {
    if req.samples.is_empty() {
        return Json(FullProcessResponse {
            success: false,
            pitch: None,
            enhancement: None,
            ab_comparison: None,
            error: Some("No audio samples provided".to_string()),
        });
    }

    // Step 1: 피치 검출
    let mut detector = state.octave_detector.write().await;
    let pitch_result = match detector.detect(&req.samples) {
        Some(r) => r,
        None => {
            return Json(FullProcessResponse {
                success: false,
                pitch: None,
                enhancement: None,
                ab_comparison: None,
                error: Some("Could not detect pitch".to_string()),
            });
        }
    };

    // Step 2: 전극 활성화 분석
    let electrode_activations = state.analyzer.analyze_electrode_pattern(&req.samples);
    let electrode_amplitudes: Vec<f32> = (1..=22)
        .map(|id| {
            electrode_activations
                .iter()
                .find(|a| a.id == id)
                .map(|a| a.amplitude)
                .unwrap_or(0.0)
        })
        .collect();

    // Step 3: Enhancement
    let enhanced = state.enhancer.enhance(
        &electrode_amplitudes,
        pitch_result.octave,
        pitch_result.fundamental_frequency,
        0,
    );

    // Step 4: A/B 비교 (선택적)
    let ab_comparison = if req.include_ab_comparison.unwrap_or(false) {
        let comparison = state.simulator.create_ab_comparison(&req.samples, &enhanced);
        Some(ABComparisonDTO {
            original_ci_samples: comparison.original_ci,
            enhanced_ci_samples: comparison.enhanced_ci,
            octave_clarity_score: comparison.octave_clarity_score,
        })
    } else {
        None
    };

    Json(FullProcessResponse {
        success: true,
        pitch: Some(pitch_result.into()),
        enhancement: Some(EnhancedSignalDTO {
            electrodes: enhanced.original_electrodes.iter().map(|e| ElectrodeDTO {
                electrode_id: e.electrode_id,
                original_amplitude: e.original_amplitude,
                enhanced_amplitude: e.enhanced_amplitude,
                modulation_frequency: e.temporal_modulation.as_ref().map(|m| m.frequency),
            }).collect(),
            octave_info: OctaveInfoDTO {
                octave: enhanced.octave_info.octave,
                fundamental_frequency: enhanced.octave_info.fundamental_frequency,
                encoding_methods: enhanced.octave_info.encoding_methods,
            },
            processing_time_ms: enhanced.metadata.processing_time_ms,
        }),
        ab_comparison,
        error: None,
    })
}

/// WebSocket 핸들러
async fn websocket_handler(
    ws: WebSocketUpgrade,
    State(state): State<Arc<AppState>>,
) -> impl IntoResponse {
    ws.on_upgrade(move |socket| handle_websocket(socket, state))
}

async fn handle_websocket(mut socket: WebSocket, state: Arc<AppState>) {
    // 스트리밍 검출기 생성
    let mut streaming_detector = StreamingOctaveDetector::new(
        state.config.sample_rate,
        2048,
        512,
    );

    // 연결 확인 메시지
    let status_msg = WSOutputMessage::Status {
        message: "Connected to WIA CI Enhancement WebSocket".to_string(),
    };
    if let Ok(json) = serde_json::to_string(&status_msg) {
        let _ = socket.send(Message::Text(json)).await;
    }

    while let Some(msg) = socket.recv().await {
        let msg = match msg {
            Ok(Message::Text(text)) => text,
            Ok(Message::Binary(data)) => {
                // 바이너리는 f32 샘플로 해석
                if data.len() % 4 != 0 {
                    continue;
                }
                let samples: Vec<f32> = data
                    .chunks_exact(4)
                    .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
                    .collect();

                let input = WSInputMessage::AudioFrame { samples };
                serde_json::to_string(&input).unwrap_or_default()
            }
            Ok(Message::Close(_)) => break,
            _ => continue,
        };

        let input: WSInputMessage = match serde_json::from_str(&msg) {
            Ok(m) => m,
            Err(e) => {
                let err = WSOutputMessage::Error {
                    message: format!("Invalid message: {}", e),
                };
                if let Ok(json) = serde_json::to_string(&err) {
                    let _ = socket.send(Message::Text(json)).await;
                }
                continue;
            }
        };

        match input {
            WSInputMessage::AudioFrame { samples } => {
                // 실시간 처리
                let results = streaming_detector.process(&samples);

                for result in results {
                    // 피치 결과
                    let pitch_msg = WSOutputMessage::PitchResult {
                        result: result.clone().into(),
                    };
                    if let Ok(json) = serde_json::to_string(&pitch_msg) {
                        let _ = socket.send(Message::Text(json)).await;
                    }

                    // Enhancement (간략화)
                    let electrode_amplitudes: Vec<f32> = vec![0.3; 22]; // 실제로는 분석 필요
                    let enhanced = state.enhancer.enhance(
                        &electrode_amplitudes,
                        result.octave,
                        result.fundamental_frequency,
                        0,
                    );

                    let enhance_msg = WSOutputMessage::EnhancementResult {
                        result: EnhancedSignalDTO {
                            electrodes: enhanced.original_electrodes.iter().map(|e| ElectrodeDTO {
                                electrode_id: e.electrode_id,
                                original_amplitude: e.original_amplitude,
                                enhanced_amplitude: e.enhanced_amplitude,
                                modulation_frequency: e.temporal_modulation.as_ref().map(|m| m.frequency),
                            }).collect(),
                            octave_info: OctaveInfoDTO {
                                octave: enhanced.octave_info.octave,
                                fundamental_frequency: enhanced.octave_info.fundamental_frequency,
                                encoding_methods: enhanced.octave_info.encoding_methods,
                            },
                            processing_time_ms: enhanced.metadata.processing_time_ms,
                        },
                    };
                    if let Ok(json) = serde_json::to_string(&enhance_msg) {
                        let _ = socket.send(Message::Text(json)).await;
                    }
                }
            }
            WSInputMessage::Config { sample_rate, threshold } => {
                // 설정 업데이트는 재연결 필요
                let status = WSOutputMessage::Status {
                    message: format!(
                        "Config received: sample_rate={:?}, threshold={:?}. Reconnect to apply.",
                        sample_rate, threshold
                    ),
                };
                if let Ok(json) = serde_json::to_string(&status) {
                    let _ = socket.send(Message::Text(json)).await;
                }
            }
            WSInputMessage::Reset => {
                streaming_detector.reset();
                let status = WSOutputMessage::Status {
                    message: "Detector reset".to_string(),
                };
                if let Ok(json) = serde_json::to_string(&status) {
                    let _ = socket.send(Message::Text(json)).await;
                }
            }
        }
    }
}

// ==================== Server Builder ====================

/// API 서버 생성
pub fn create_app(config: ServerConfig) -> Router {
    let sample_rate = config.sample_rate;

    let state = Arc::new(AppState {
        octave_detector: RwLock::new(OctaveYIN::new(YINConfig {
            sample_rate,
            ..Default::default()
        })),
        enhancer: CIEnhancer::with_default(sample_rate),
        simulator: CIVocoderSimulator::new(sample_rate),
        analyzer: SignalAnalyzer::new(sample_rate),
        config,
    });

    Router::new()
        .route("/", get(get_info))
        .route("/health", get(health_check))
        .route("/api/v1/pitch", post(detect_pitch))
        .route("/api/v1/enhance", post(enhance_signal))
        .route("/api/v1/process", post(full_process))
        .route("/api/v1/ws", get(websocket_handler))
        .with_state(state)
}

/// 서버 실행
pub async fn run_server(config: ServerConfig) -> Result<(), Box<dyn std::error::Error>> {
    let addr = format!("{}:{}", config.host, config.port);
    let app = create_app(config);

    println!("🦻 WIA CI Octave Enhancement API Server");
    println!("   Version: 0.1.0");
    println!("   Listening on: http://{}", addr);
    println!();
    println!("Endpoints:");
    println!("   GET  /           - Server info");
    println!("   GET  /health     - Health check");
    println!("   POST /api/v1/pitch   - Pitch detection");
    println!("   POST /api/v1/enhance - CI enhancement");
    println!("   POST /api/v1/process - Full pipeline");
    println!("   WS   /api/v1/ws      - Real-time WebSocket");
    println!();
    println!("홍익인간 🤟 - 널리 인간을 이롭게 하라");

    let listener = tokio::net::TcpListener::bind(&addr).await?;
    axum::serve(listener, app).await?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use axum::body::Body;
    use axum::http::{Request, StatusCode};
    use tower::ServiceExt;

    #[tokio::test]
    async fn test_server_info() {
        let app = create_app(ServerConfig::default());

        let response = app
            .oneshot(Request::builder().uri("/").body(Body::empty()).unwrap())
            .await
            .unwrap();

        assert_eq!(response.status(), StatusCode::OK);
    }

    #[tokio::test]
    async fn test_health_check() {
        let app = create_app(ServerConfig::default());

        let response = app
            .oneshot(Request::builder().uri("/health").body(Body::empty()).unwrap())
            .await
            .unwrap();

        assert_eq!(response.status(), StatusCode::OK);
    }
}
