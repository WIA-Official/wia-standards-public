//! WIA Refugee Credential API Server
//!
//! REST API and WebSocket server for refugee credential management.
//!
//! 홍익인간 (弘益人間) - Benefit All Humanity
//! "국가가 무너져도, 사람의 가치는 무너지지 않습니다."

use axum::{
    extract::{Path, Query, State, WebSocketUpgrade},
    http::{header, Method, StatusCode},
    response::{IntoResponse, Response},
    routing::{get, post},
    Json, Router,
};
use axum::extract::ws::{Message, WebSocket};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use tower_http::cors::{Any, CorsLayer};
use tower_http::trace::TraceLayer;
use uuid::Uuid;

use crate::core::{
    AssessmentEvaluator, ConfidenceCalculator, CredentialManager, DocumentVerifier, PeerMatcher,
};
use crate::types::*;

/// Application state shared across handlers
pub struct AppState {
    pub credential_manager: RwLock<CredentialManager>,
    pub confidence_calculator: ConfidenceCalculator,
    pub peer_matcher: PeerMatcher,
    pub assessment_evaluator: AssessmentEvaluator,
    pub document_verifier: DocumentVerifier,
}

impl AppState {
    pub fn new() -> Self {
        Self {
            credential_manager: RwLock::new(CredentialManager::new()),
            confidence_calculator: ConfidenceCalculator::new(),
            peer_matcher: PeerMatcher::new(),
            assessment_evaluator: AssessmentEvaluator::new(),
            document_verifier: DocumentVerifier::new(),
        }
    }
}

impl Default for AppState {
    fn default() -> Self {
        Self::new()
    }
}

/// API response wrapper
#[derive(Debug, Serialize)]
pub struct ApiResponse<T: Serialize> {
    pub success: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data: Option<T>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub message: Option<String>,
}

impl<T: Serialize> ApiResponse<T> {
    pub fn success(data: T) -> Self {
        Self {
            success: true,
            data: Some(data),
            error: None,
            message: None,
        }
    }

    pub fn success_with_message(data: T, message: &str) -> Self {
        Self {
            success: true,
            data: Some(data),
            error: None,
            message: Some(message.to_string()),
        }
    }
}

impl ApiResponse<()> {
    pub fn error(error: &str) -> Self {
        Self {
            success: false,
            data: None,
            error: Some(error.to_string()),
            message: None,
        }
    }
}

/// Health check response
#[derive(Debug, Serialize)]
pub struct HealthResponse {
    pub status: String,
    pub service: String,
    pub version: String,
    pub philosophy: String,
    pub supported_languages: Vec<String>,
}

/// Version response
#[derive(Debug, Serialize)]
pub struct VersionResponse {
    pub version: String,
    pub standard: String,
    pub api_version: String,
    pub supported_languages: Vec<String>,
    pub verification_levels: Vec<VerificationLevelInfo>,
}

#[derive(Debug, Serialize)]
pub struct VerificationLevelInfo {
    pub level: u8,
    pub name: String,
    pub description: String,
    pub base_confidence: f64,
}

/// Query parameters for credential search
#[derive(Debug, Deserialize)]
pub struct CredentialSearchQuery {
    pub nationality: Option<String>,
    pub education_level: Option<String>,
    pub field: Option<String>,
    pub min_verification_level: Option<u8>,
}

/// Query parameters for peer search
#[derive(Debug, Deserialize)]
pub struct PeerSearchQuery {
    pub limit: Option<usize>,
}

/// Create credential request
#[derive(Debug, Deserialize)]
pub struct CreateCredentialRequest {
    pub holder: HolderIdentity,
    #[serde(default)]
    pub education: Vec<EducationRecord>,
    #[serde(default)]
    pub competencies: Vec<CompetencyRecord>,
    #[serde(default)]
    pub languages: Vec<LanguageAbility>,
    #[serde(default)]
    pub work_experience: Vec<WorkExperience>,
}

/// Peer verification request
#[derive(Debug, Deserialize)]
pub struct PeerVerificationRequest {
    pub credential_id: Uuid,
    pub peer_did: String,
    pub verification_type: String,
    pub shared_context: String,
    pub confidence_score: f64,
    #[serde(default)]
    pub notes: Option<String>,
}

/// Assessment submission request
#[derive(Debug, Deserialize)]
pub struct AssessmentSubmissionRequest {
    pub credential_id: Uuid,
    pub domain: CompetencyDomain,
    pub assessment_type: String,
    pub raw_score: f64,
    pub max_score: f64,
    pub assessor_did: String,
}

/// Institution verification request
#[derive(Debug, Deserialize)]
pub struct InstitutionVerifyRequest {
    pub credential_id: Uuid,
    pub institution_did: String,
    pub verification_type: String,
    pub purpose: String,
    pub fields_requested: Vec<String>,
}

/// Confidence calculation result
#[derive(Debug, Serialize)]
pub struct ConfidenceResult {
    pub overall_confidence: f64,
    pub verification_level: u8,
    pub breakdown: ConfidenceBreakdown,
}

#[derive(Debug, Serialize)]
pub struct ConfidenceBreakdown {
    pub base_confidence: f64,
    pub peer_bonus: f64,
    pub assessment_bonus: f64,
    pub document_bonus: f64,
}

/// Peer match result
#[derive(Debug, Serialize)]
pub struct PeerMatchResult {
    pub potential_peers: Vec<PeerInfo>,
    pub match_count: usize,
}

#[derive(Debug, Serialize)]
pub struct PeerInfo {
    pub peer_did: String,
    pub shared_context: String,
    pub relevance_score: f64,
}

/// Start the API server
pub async fn start_server(host: &str, port: u16) -> anyhow::Result<()> {
    let state = Arc::new(AppState::new());

    let cors = CorsLayer::new()
        .allow_origin(Any)
        .allow_methods([Method::GET, Method::POST, Method::PUT, Method::DELETE])
        .allow_headers([header::CONTENT_TYPE, header::AUTHORIZATION]);

    let app = Router::new()
        // Health and info
        .route("/health", get(health_check))
        .route("/version", get(version_info))
        // Credential endpoints
        .route("/api/v1/credentials", post(create_credential))
        .route("/api/v1/credentials", get(list_credentials))
        .route("/api/v1/credentials/:id", get(get_credential))
        .route("/api/v1/credentials/:id", put(update_credential))
        // Verification endpoints
        .route(
            "/api/v1/credentials/:id/confidence",
            get(calculate_confidence),
        )
        .route("/api/v1/credentials/:id/peers/find", get(find_peers))
        .route("/api/v1/credentials/:id/peers/verify", post(peer_verify))
        .route(
            "/api/v1/credentials/:id/assessment",
            post(submit_assessment),
        )
        // Institution verification
        .route("/api/v1/verify", post(institution_verify))
        // WebSocket
        .route("/ws", get(ws_handler))
        // Middleware
        .layer(cors)
        .layer(TraceLayer::new_for_http())
        .with_state(state);

    let addr = format!("{}:{}", host, port);
    let listener = tokio::net::TcpListener::bind(&addr).await?;

    tracing::info!("🌍 WIA Refugee Credential API Server");
    tracing::info!("📍 Listening on http://{}", addr);
    tracing::info!("🔒 Verification levels: 1-4");
    tracing::info!("🌐 Languages: ar, uk, fa, ps, ti, fr, en, es, de, tr, ko");
    tracing::info!("💫 홍익인간 (弘益人間) - Benefit All Humanity");

    axum::serve(listener, app).await?;
    Ok(())
}

/// Health check endpoint
async fn health_check() -> Json<ApiResponse<HealthResponse>> {
    Json(ApiResponse::success(HealthResponse {
        status: "healthy".to_string(),
        service: "WIA Refugee Credential API".to_string(),
        version: crate::VERSION.to_string(),
        philosophy: crate::PHILOSOPHY.to_string(),
        supported_languages: crate::SUPPORTED_LANGUAGES
            .iter()
            .map(|s| s.to_string())
            .collect(),
    }))
}

/// Version info endpoint
async fn version_info() -> Json<ApiResponse<VersionResponse>> {
    Json(ApiResponse::success(VersionResponse {
        version: crate::VERSION.to_string(),
        standard: "WIA-REFUGEE-CREDENTIAL".to_string(),
        api_version: "v1".to_string(),
        supported_languages: crate::SUPPORTED_LANGUAGES
            .iter()
            .map(|s| s.to_string())
            .collect(),
        verification_levels: vec![
            VerificationLevelInfo {
                level: 1,
                name: "Self-Declaration".to_string(),
                description: "Holder's own claims, unverified".to_string(),
                base_confidence: 0.30,
            },
            VerificationLevelInfo {
                level: 2,
                name: "Peer Verification".to_string(),
                description: "Verified by peers from same institution/profession".to_string(),
                base_confidence: 0.60,
            },
            VerificationLevelInfo {
                level: 3,
                name: "Assessment Verified".to_string(),
                description: "Passed competency assessment".to_string(),
                base_confidence: 0.80,
            },
            VerificationLevelInfo {
                level: 4,
                name: "Document Verified".to_string(),
                description: "Original documents verified by institution".to_string(),
                base_confidence: 0.95,
            },
        ],
    }))
}

/// Create a new credential
async fn create_credential(
    State(state): State<Arc<AppState>>,
    Json(request): Json<CreateCredentialRequest>,
) -> Result<Json<ApiResponse<RefugeeCredential>>, (StatusCode, Json<ApiResponse<()>>)> {
    // Validate holder identity
    if request.holder.display_name.is_empty() {
        return Err((
            StatusCode::BAD_REQUEST,
            Json(ApiResponse::error("Display name is required")),
        ));
    }

    // Generate credential
    let now = chrono::Utc::now();
    let credential_id = Uuid::now_v7();
    let holder_did = crate::generate_did();

    let credential = RefugeeCredential {
        id: credential_id,
        context: vec![
            "https://www.w3.org/2018/credentials/v1".to_string(),
            "https://wia.org/contexts/refugee-credential/v1".to_string(),
        ],
        credential_type: vec![
            "VerifiableCredential".to_string(),
            "WIARefugeeCredential".to_string(),
        ],
        issuer: crate::generate_did(),
        issuance_date: now,
        expiration_date: None,
        holder: HolderIdentity {
            did: Some(holder_did),
            ..request.holder
        },
        education: request.education,
        competencies: request.competencies,
        languages: request.languages,
        work_experience: request.work_experience,
        verification: VerificationInfo {
            level: VerificationLevel::SelfDeclared,
            overall_confidence: 0.30,
            last_verification_date: now,
            peer_verifications: vec![],
            assessment_results: vec![],
            document_verifications: vec![],
        },
        evidence: vec![],
        metadata: CredentialMetadata {
            version: crate::VERSION.to_string(),
            created_at: now,
            updated_at: now,
            language: "en".to_string(),
            privacy_level: PrivacyLevel::Standard,
        },
        signature: None,
    };

    // Store credential
    let mut manager = state.credential_manager.write().await;
    manager.create(credential.clone());

    Ok(Json(ApiResponse::success_with_message(
        credential,
        "Credential created successfully. Verification level: 1 (Self-Declared)",
    )))
}

/// List credentials with optional filters
async fn list_credentials(
    State(state): State<Arc<AppState>>,
    Query(query): Query<CredentialSearchQuery>,
) -> Json<ApiResponse<Vec<RefugeeCredential>>> {
    let manager = state.credential_manager.read().await;

    let mut credentials: Vec<RefugeeCredential> = manager.list();

    // Apply filters
    if let Some(ref nationality) = query.nationality {
        credentials.retain(|c| c.holder.nationality_claimed == *nationality);
    }

    if let Some(ref education_level) = query.education_level {
        credentials.retain(|c| {
            c.education.iter().any(|e| {
                format!("{:?}", e.level).to_lowercase() == education_level.to_lowercase()
            })
        });
    }

    if let Some(ref field) = query.field {
        credentials.retain(|c| {
            c.education
                .iter()
                .any(|e| e.field_of_study.to_lowercase().contains(&field.to_lowercase()))
        });
    }

    if let Some(min_level) = query.min_verification_level {
        credentials.retain(|c| {
            let level = match c.verification.level {
                VerificationLevel::SelfDeclared => 1,
                VerificationLevel::PeerVerified => 2,
                VerificationLevel::AssessmentVerified => 3,
                VerificationLevel::DocumentVerified => 4,
            };
            level >= min_level
        });
    }

    Json(ApiResponse::success(credentials))
}

/// Get credential by ID
async fn get_credential(
    State(state): State<Arc<AppState>>,
    Path(id): Path<Uuid>,
) -> Result<Json<ApiResponse<RefugeeCredential>>, (StatusCode, Json<ApiResponse<()>>)> {
    let manager = state.credential_manager.read().await;

    match manager.get(&id) {
        Some(credential) => Ok(Json(ApiResponse::success(credential.clone()))),
        None => Err((
            StatusCode::NOT_FOUND,
            Json(ApiResponse::error("Credential not found")),
        )),
    }
}

/// Update credential
async fn update_credential(
    State(state): State<Arc<AppState>>,
    Path(id): Path<Uuid>,
    Json(updates): Json<serde_json::Value>,
) -> Result<Json<ApiResponse<RefugeeCredential>>, (StatusCode, Json<ApiResponse<()>>)> {
    let mut manager = state.credential_manager.write().await;

    let credential = match manager.get(&id) {
        Some(c) => c.clone(),
        None => {
            return Err((
                StatusCode::NOT_FOUND,
                Json(ApiResponse::error("Credential not found")),
            ))
        }
    };

    // Apply updates (simplified - in production would be more sophisticated)
    let mut updated = credential;
    updated.metadata.updated_at = chrono::Utc::now();

    // Update education if provided
    if let Some(education) = updates.get("education") {
        if let Ok(edu) = serde_json::from_value(education.clone()) {
            updated.education = edu;
        }
    }

    // Update competencies if provided
    if let Some(competencies) = updates.get("competencies") {
        if let Ok(comp) = serde_json::from_value(competencies.clone()) {
            updated.competencies = comp;
        }
    }

    // Update languages if provided
    if let Some(languages) = updates.get("languages") {
        if let Ok(lang) = serde_json::from_value(languages.clone()) {
            updated.languages = lang;
        }
    }

    manager.update(updated.clone());

    Ok(Json(ApiResponse::success(updated)))
}

/// Calculate confidence score
async fn calculate_confidence(
    State(state): State<Arc<AppState>>,
    Path(id): Path<Uuid>,
) -> Result<Json<ApiResponse<ConfidenceResult>>, (StatusCode, Json<ApiResponse<()>>)> {
    let manager = state.credential_manager.read().await;

    let credential = match manager.get(&id) {
        Some(c) => c.clone(),
        None => {
            return Err((
                StatusCode::NOT_FOUND,
                Json(ApiResponse::error("Credential not found")),
            ))
        }
    };

    let (confidence, level) = state.confidence_calculator.calculate(&credential);

    // Calculate breakdown
    let base = crate::get_base_confidence(level);
    let peer_count = credential.verification.peer_verifications.len();
    let assessment_count = credential.verification.assessment_results.len();
    let doc_count = credential.verification.document_verifications.len();

    let peer_bonus = (peer_count as f64 * 0.05).min(0.15);
    let assessment_bonus = (assessment_count as f64 * 0.03).min(0.10);
    let doc_bonus = (doc_count as f64 * 0.02).min(0.05);

    let level_num = match level {
        VerificationLevel::SelfDeclared => 1,
        VerificationLevel::PeerVerified => 2,
        VerificationLevel::AssessmentVerified => 3,
        VerificationLevel::DocumentVerified => 4,
    };

    Ok(Json(ApiResponse::success(ConfidenceResult {
        overall_confidence: confidence,
        verification_level: level_num,
        breakdown: ConfidenceBreakdown {
            base_confidence: base,
            peer_bonus,
            assessment_bonus,
            document_bonus: doc_bonus,
        },
    })))
}

/// Find peer verifiers
async fn find_peers(
    State(state): State<Arc<AppState>>,
    Path(id): Path<Uuid>,
    Query(query): Query<PeerSearchQuery>,
) -> Result<Json<ApiResponse<PeerMatchResult>>, (StatusCode, Json<ApiResponse<()>>)> {
    let manager = state.credential_manager.read().await;

    let credential = match manager.get(&id) {
        Some(c) => c.clone(),
        None => {
            return Err((
                StatusCode::NOT_FOUND,
                Json(ApiResponse::error("Credential not found")),
            ))
        }
    };

    // Get all other credentials as potential peers
    let all_credentials: Vec<_> = manager
        .list()
        .into_iter()
        .filter(|c| c.id != id)
        .collect();

    let matches = state.peer_matcher.find_matches(&credential, &all_credentials);

    let limit = query.limit.unwrap_or(10);
    let peers: Vec<PeerInfo> = matches
        .into_iter()
        .take(limit)
        .map(|(peer_cred, context, score)| PeerInfo {
            peer_did: peer_cred
                .holder
                .did
                .unwrap_or_else(|| "unknown".to_string()),
            shared_context: context,
            relevance_score: score,
        })
        .collect();

    let count = peers.len();

    Ok(Json(ApiResponse::success(PeerMatchResult {
        potential_peers: peers,
        match_count: count,
    })))
}

/// Submit peer verification
async fn peer_verify(
    State(state): State<Arc<AppState>>,
    Path(id): Path<Uuid>,
    Json(request): Json<PeerVerificationRequest>,
) -> Result<Json<ApiResponse<RefugeeCredential>>, (StatusCode, Json<ApiResponse<()>>)> {
    let mut manager = state.credential_manager.write().await;

    let mut credential = match manager.get(&id) {
        Some(c) => c.clone(),
        None => {
            return Err((
                StatusCode::NOT_FOUND,
                Json(ApiResponse::error("Credential not found")),
            ))
        }
    };

    // Add peer verification
    let peer_verification = PeerVerification {
        peer_did: request.peer_did,
        verification_date: chrono::Utc::now(),
        verification_type: request.verification_type,
        shared_context: request.shared_context,
        confidence_score: request.confidence_score.clamp(0.0, 1.0),
        peer_credential_id: Some(request.credential_id),
    };

    credential
        .verification
        .peer_verifications
        .push(peer_verification);

    // Recalculate confidence
    let (new_confidence, new_level) = state.confidence_calculator.calculate(&credential);
    credential.verification.overall_confidence = new_confidence;
    credential.verification.level = new_level;
    credential.verification.last_verification_date = chrono::Utc::now();
    credential.metadata.updated_at = chrono::Utc::now();

    manager.update(credential.clone());

    Ok(Json(ApiResponse::success_with_message(
        credential,
        "Peer verification added successfully",
    )))
}

/// Submit assessment result
async fn submit_assessment(
    State(state): State<Arc<AppState>>,
    Path(id): Path<Uuid>,
    Json(request): Json<AssessmentSubmissionRequest>,
) -> Result<Json<ApiResponse<RefugeeCredential>>, (StatusCode, Json<ApiResponse<()>>)> {
    let mut manager = state.credential_manager.write().await;

    let mut credential = match manager.get(&id) {
        Some(c) => c.clone(),
        None => {
            return Err((
                StatusCode::NOT_FOUND,
                Json(ApiResponse::error("Credential not found")),
            ))
        }
    };

    // Evaluate assessment
    let normalized_score = request.raw_score / request.max_score;
    let passed = normalized_score >= 0.70; // 70% passing threshold

    let assessment_result = AssessmentResult {
        assessment_id: Uuid::now_v7(),
        assessment_date: chrono::Utc::now(),
        domain: request.domain,
        assessment_type: request.assessment_type,
        raw_score: request.raw_score,
        normalized_score,
        passed,
        assessor_did: request.assessor_did,
        certificate_url: None,
    };

    credential
        .verification
        .assessment_results
        .push(assessment_result);

    // Recalculate confidence
    let (new_confidence, new_level) = state.confidence_calculator.calculate(&credential);
    credential.verification.overall_confidence = new_confidence;
    credential.verification.level = new_level;
    credential.verification.last_verification_date = chrono::Utc::now();
    credential.metadata.updated_at = chrono::Utc::now();

    manager.update(credential.clone());

    let message = if passed {
        "Assessment passed! Credential verification level updated."
    } else {
        "Assessment recorded but did not meet passing threshold (70%)."
    };

    Ok(Json(ApiResponse::success_with_message(credential, message)))
}

/// Institution verification endpoint
async fn institution_verify(
    State(state): State<Arc<AppState>>,
    Json(request): Json<InstitutionVerifyRequest>,
) -> Result<Json<ApiResponse<InstitutionVerificationResponse>>, (StatusCode, Json<ApiResponse<()>>)>
{
    let manager = state.credential_manager.read().await;

    let credential = match manager.get(&request.credential_id) {
        Some(c) => c.clone(),
        None => {
            return Err((
                StatusCode::NOT_FOUND,
                Json(ApiResponse::error("Credential not found")),
            ))
        }
    };

    // Build response based on requested fields
    let mut response_data = HashMap::new();

    for field in &request.fields_requested {
        match field.as_str() {
            "education_summary" => {
                let summary: Vec<String> = credential
                    .education
                    .iter()
                    .map(|e| format!("{:?} in {}", e.level, e.field_of_study))
                    .collect();
                response_data.insert(
                    "education_summary".to_string(),
                    serde_json::json!(summary),
                );
            }
            "verification_level" => {
                let level = match credential.verification.level {
                    VerificationLevel::SelfDeclared => 1,
                    VerificationLevel::PeerVerified => 2,
                    VerificationLevel::AssessmentVerified => 3,
                    VerificationLevel::DocumentVerified => 4,
                };
                response_data
                    .insert("verification_level".to_string(), serde_json::json!(level));
            }
            "confidence_score" => {
                response_data.insert(
                    "confidence_score".to_string(),
                    serde_json::json!(credential.verification.overall_confidence),
                );
            }
            "languages" => {
                let langs: Vec<String> = credential
                    .languages
                    .iter()
                    .map(|l| format!("{} ({:?})", l.language_code, l.proficiency))
                    .collect();
                response_data.insert("languages".to_string(), serde_json::json!(langs));
            }
            "competencies" => {
                let comps: Vec<String> = credential
                    .competencies
                    .iter()
                    .map(|c| format!("{} ({:?})", c.name, c.proficiency))
                    .collect();
                response_data.insert("competencies".to_string(), serde_json::json!(comps));
            }
            _ => {}
        }
    }

    Ok(Json(ApiResponse::success(InstitutionVerificationResponse {
        credential_id: credential.id,
        verification_timestamp: chrono::Utc::now(),
        verifier_did: request.institution_did,
        verification_type: request.verification_type,
        purpose: request.purpose,
        data: response_data,
        attestation: generate_attestation(&credential),
    })))
}

#[derive(Debug, Serialize)]
pub struct InstitutionVerificationResponse {
    pub credential_id: Uuid,
    pub verification_timestamp: chrono::DateTime<chrono::Utc>,
    pub verifier_did: String,
    pub verification_type: String,
    pub purpose: String,
    pub data: HashMap<String, serde_json::Value>,
    pub attestation: String,
}

fn generate_attestation(credential: &RefugeeCredential) -> String {
    // In production, this would be a proper cryptographic signature
    format!(
        "WIA-ATTESTATION:{}:{}:{}",
        credential.id,
        credential.verification.overall_confidence,
        chrono::Utc::now().timestamp()
    )
}

/// WebSocket handler for real-time updates
async fn ws_handler(
    ws: WebSocketUpgrade,
    State(_state): State<Arc<AppState>>,
) -> Response {
    ws.on_upgrade(handle_socket)
}

async fn handle_socket(mut socket: WebSocket) {
    // Send welcome message
    let welcome = serde_json::json!({
        "type": "welcome",
        "message": "Connected to WIA Refugee Credential WebSocket",
        "philosophy": "국가가 무너져도, 사람의 가치는 무너지지 않습니다",
        "supported_events": [
            "credential.created",
            "credential.updated",
            "verification.peer_added",
            "verification.assessment_completed",
            "verification.level_changed"
        ]
    });

    if socket
        .send(Message::Text(welcome.to_string().into()))
        .await
        .is_err()
    {
        return;
    }

    // Handle incoming messages
    while let Some(msg) = socket.recv().await {
        match msg {
            Ok(Message::Text(text)) => {
                // Parse and handle subscription requests
                if let Ok(request) = serde_json::from_str::<serde_json::Value>(&text) {
                    if request.get("type") == Some(&serde_json::json!("subscribe")) {
                        let response = serde_json::json!({
                            "type": "subscribed",
                            "credential_id": request.get("credential_id"),
                            "message": "Subscribed to credential updates"
                        });
                        if socket
                            .send(Message::Text(response.to_string().into()))
                            .await
                            .is_err()
                        {
                            break;
                        }
                    } else if request.get("type") == Some(&serde_json::json!("ping")) {
                        let response = serde_json::json!({
                            "type": "pong",
                            "timestamp": chrono::Utc::now().to_rfc3339()
                        });
                        if socket
                            .send(Message::Text(response.to_string().into()))
                            .await
                            .is_err()
                        {
                            break;
                        }
                    }
                }
            }
            Ok(Message::Ping(data)) => {
                if socket.send(Message::Pong(data)).await.is_err() {
                    break;
                }
            }
            Ok(Message::Close(_)) => break,
            Err(_) => break,
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_api_response_success() {
        let response = ApiResponse::success("test data");
        assert!(response.success);
        assert_eq!(response.data, Some("test data"));
        assert!(response.error.is_none());
    }

    #[test]
    fn test_api_response_error() {
        let response: ApiResponse<()> = ApiResponse::error("test error");
        assert!(!response.success);
        assert!(response.data.is_none());
        assert_eq!(response.error, Some("test error".to_string()));
    }

    #[test]
    fn test_app_state_creation() {
        let state = AppState::new();
        // Just verify it creates without panic
        assert!(true);
    }
}
