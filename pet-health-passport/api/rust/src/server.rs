//! WIA Pet Health Passport - HTTP/WebSocket Server
//!
//! Phase 3: Communication Protocol Implementation
//! REST API + WebSocket for real-time updates
//!
//! 홍익인간 (弘益人間) - Benefit All Humanity & Animals

use axum::{
    extract::{Path, Query, State, WebSocketUpgrade},
    http::StatusCode,
    response::{IntoResponse, Response},
    routing::{get, post, put, delete},
    Json, Router,
};
use axum::extract::ws::{Message, WebSocket};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::{broadcast, RwLock};
use tracing::{info, error, warn};
use uuid::Uuid;

use crate::types::*;
use crate::core::*;

// ============================================================================
// Server State
// ============================================================================

/// Shared application state
pub struct AppState {
    /// Passport manager for CRUD operations
    pub passport_manager: RwLock<PassportManager>,
    /// Quarantine eligibility checker
    pub quarantine_checker: QuarantineChecker,
    /// Vaccination scheduler
    pub vaccination_scheduler: VaccinationScheduler,
    /// Health risk assessor
    pub health_risk_assessor: HealthRiskAssessor,
    /// Emergency info extractor
    pub emergency_extractor: EmergencyExtractor,
    /// Document verifier
    pub document_verifier: DocumentVerifier,
    /// Broadcast channel for real-time updates
    pub broadcast_tx: broadcast::Sender<WebSocketEvent>,
}

impl AppState {
    pub fn new() -> Self {
        let (broadcast_tx, _) = broadcast::channel(1000);

        Self {
            passport_manager: RwLock::new(PassportManager::new()),
            quarantine_checker: QuarantineChecker::new(),
            vaccination_scheduler: VaccinationScheduler::new(),
            health_risk_assessor: HealthRiskAssessor::new(),
            emergency_extractor: EmergencyExtractor::new(),
            document_verifier: DocumentVerifier::new(),
            broadcast_tx,
        }
    }
}

impl Default for AppState {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// WebSocket Events
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", content = "data")]
pub enum WebSocketEvent {
    /// Passport created
    PassportCreated { passport_id: String },
    /// Passport updated
    PassportUpdated { passport_id: String },
    /// Vaccination added
    VaccinationAdded { passport_id: String, vaccine: String },
    /// Quarantine status changed
    QuarantineStatusChanged { passport_id: String, eligible: bool },
    /// Health alert
    HealthAlert { passport_id: String, message: String },
}

// ============================================================================
// API Request/Response Types
// ============================================================================

#[derive(Debug, Deserialize)]
pub struct CreatePassportRequest {
    pub pet: PetIdentity,
    pub microchip: Option<MicrochipInfo>,
    pub guardian: GuardianInfo,
}

#[derive(Debug, Serialize)]
pub struct CreatePassportResponse {
    pub passport_id: String,
    pub created_at: String,
}

#[derive(Debug, Deserialize)]
pub struct QuarantineCheckRequest {
    pub passport_id: String,
    pub destination_country: String,
}

#[derive(Debug, Deserialize)]
pub struct AddVaccinationRequest {
    pub vaccination: VaccinationRecord,
}

#[derive(Debug, Deserialize)]
pub struct QueryParams {
    pub species: Option<String>,
    pub country: Option<String>,
    pub limit: Option<usize>,
    pub offset: Option<usize>,
}

#[derive(Debug, Serialize)]
pub struct ApiResponse<T> {
    pub success: bool,
    pub data: Option<T>,
    pub error: Option<ApiError>,
}

#[derive(Debug, Serialize)]
pub struct ApiError {
    pub code: String,
    pub message: String,
}

impl<T: Serialize> ApiResponse<T> {
    pub fn success(data: T) -> Self {
        Self {
            success: true,
            data: Some(data),
            error: None,
        }
    }

    pub fn error(code: &str, message: &str) -> ApiResponse<()> {
        ApiResponse {
            success: false,
            data: None,
            error: Some(ApiError {
                code: code.to_string(),
                message: message.to_string(),
            }),
        }
    }
}

// ============================================================================
// Router Configuration
// ============================================================================

/// Create the API router
pub fn create_router(state: Arc<AppState>) -> Router {
    Router::new()
        // Health check
        .route("/health", get(health_check))
        .route("/version", get(version))

        // Passport CRUD
        .route("/api/v1/passports", post(create_passport))
        .route("/api/v1/passports", get(list_passports))
        .route("/api/v1/passports/:id", get(get_passport))
        .route("/api/v1/passports/:id", put(update_passport))
        .route("/api/v1/passports/:id", delete(delete_passport))

        // Vaccinations
        .route("/api/v1/passports/:id/vaccinations", post(add_vaccination))
        .route("/api/v1/passports/:id/vaccinations", get(get_vaccinations))
        .route("/api/v1/passports/:id/vaccinations/schedule", get(get_vaccination_schedule))

        // Quarantine
        .route("/api/v1/quarantine/check", post(check_quarantine))
        .route("/api/v1/quarantine/requirements/:country", get(get_country_requirements))

        // Health
        .route("/api/v1/passports/:id/health/risk", get(assess_health_risk))

        // Emergency
        .route("/api/v1/emergency/:id", get(get_emergency_info))
        .route("/api/v1/emergency/qr/:code", get(get_emergency_by_qr))

        // Verification
        .route("/api/v1/verify", post(verify_document))

        // WebSocket
        .route("/ws", get(websocket_handler))

        .with_state(state)
}

// ============================================================================
// Health & Version Endpoints
// ============================================================================

async fn health_check() -> impl IntoResponse {
    Json(serde_json::json!({
        "status": "healthy",
        "service": "wia-pet-passport",
        "timestamp": chrono::Utc::now().to_rfc3339()
    }))
}

async fn version() -> impl IntoResponse {
    Json(serde_json::json!({
        "version": crate::VERSION,
        "standard": "WIA-PET-HEALTH-PASSPORT",
        "phase": "1.0.0",
        "microchip_standard": crate::MICROCHIP_STANDARD
    }))
}

// ============================================================================
// Passport CRUD Endpoints
// ============================================================================

/// POST /api/v1/passports - Create new passport
async fn create_passport(
    State(state): State<Arc<AppState>>,
    Json(request): Json<CreatePassportRequest>,
) -> impl IntoResponse {
    let mut manager = state.passport_manager.write().await;

    // Create new passport
    let passport = PetHealthPassport {
        id: Uuid::now_v7().to_string(),
        version: crate::VERSION.to_string(),
        created_at: chrono::Utc::now(),
        updated_at: chrono::Utc::now(),
        pet: request.pet,
        microchip: request.microchip,
        guardian: request.guardian,
        vaccinations: Vec::new(),
        surgeries: Vec::new(),
        diagnostics: Vec::new(),
        allergies: Vec::new(),
        chronic_conditions: Vec::new(),
        medications: Vec::new(),
        genetic_profile: None,
        travel_history: Vec::new(),
    };

    let passport_id = passport.id.clone();

    match manager.create(passport) {
        Ok(_) => {
            info!("Created passport: {}", passport_id);

            // Broadcast event
            let _ = state.broadcast_tx.send(WebSocketEvent::PassportCreated {
                passport_id: passport_id.clone(),
            });

            (
                StatusCode::CREATED,
                Json(ApiResponse::success(CreatePassportResponse {
                    passport_id,
                    created_at: chrono::Utc::now().to_rfc3339(),
                }))
            )
        }
        Err(e) => {
            error!("Failed to create passport: {:?}", e);
            (
                StatusCode::INTERNAL_SERVER_ERROR,
                Json(ApiResponse::success(CreatePassportResponse {
                    passport_id: String::new(),
                    created_at: String::new(),
                }))
            )
        }
    }
}

/// GET /api/v1/passports - List all passports
async fn list_passports(
    State(state): State<Arc<AppState>>,
    Query(params): Query<QueryParams>,
) -> impl IntoResponse {
    let manager = state.passport_manager.read().await;
    let passports = manager.list();

    // Apply filters
    let filtered: Vec<_> = passports
        .iter()
        .filter(|p| {
            if let Some(ref species) = params.species {
                let species_str = format!("{:?}", p.pet.species).to_lowercase();
                if !species_str.contains(&species.to_lowercase()) {
                    return false;
                }
            }
            true
        })
        .skip(params.offset.unwrap_or(0))
        .take(params.limit.unwrap_or(100))
        .cloned()
        .collect();

    Json(ApiResponse::success(filtered))
}

/// GET /api/v1/passports/:id - Get passport by ID
async fn get_passport(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
) -> Response {
    let manager = state.passport_manager.read().await;

    match manager.get(&id) {
        Some(passport) => Json(ApiResponse::success(passport.clone())).into_response(),
        None => (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("NOT_FOUND", "Passport not found"))
        ).into_response(),
    }
}

/// PUT /api/v1/passports/:id - Update passport
async fn update_passport(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
    Json(passport): Json<PetHealthPassport>,
) -> Response {
    let mut manager = state.passport_manager.write().await;

    if manager.get(&id).is_none() {
        return (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("NOT_FOUND", "Passport not found"))
        ).into_response();
    }

    match manager.update(passport) {
        Ok(_) => {
            let _ = state.broadcast_tx.send(WebSocketEvent::PassportUpdated {
                passport_id: id.clone(),
            });
            Json(ApiResponse::success(serde_json::json!({"updated": true}))).into_response()
        }
        Err(e) => (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(ApiResponse::<()>::error("UPDATE_FAILED", &format!("{:?}", e)))
        ).into_response(),
    }
}

/// DELETE /api/v1/passports/:id - Delete passport
async fn delete_passport(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
) -> Response {
    let mut manager = state.passport_manager.write().await;

    match manager.delete(&id) {
        Ok(_) => Json(ApiResponse::success(serde_json::json!({"deleted": true}))).into_response(),
        Err(e) => (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("NOT_FOUND", &format!("{:?}", e)))
        ).into_response(),
    }
}

// ============================================================================
// Vaccination Endpoints
// ============================================================================

/// POST /api/v1/passports/:id/vaccinations - Add vaccination
async fn add_vaccination(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
    Json(request): Json<AddVaccinationRequest>,
) -> Response {
    let mut manager = state.passport_manager.write().await;

    match manager.add_vaccination(&id, request.vaccination.clone()) {
        Ok(_) => {
            let vaccine_name = format!("{:?}", request.vaccination.disease);
            let _ = state.broadcast_tx.send(WebSocketEvent::VaccinationAdded {
                passport_id: id.clone(),
                vaccine: vaccine_name,
            });
            (
                StatusCode::CREATED,
                Json(ApiResponse::success(serde_json::json!({"added": true})))
            ).into_response()
        }
        Err(e) => (
            StatusCode::BAD_REQUEST,
            Json(ApiResponse::<()>::error("ADD_FAILED", &format!("{:?}", e)))
        ).into_response(),
    }
}

/// GET /api/v1/passports/:id/vaccinations - Get all vaccinations
async fn get_vaccinations(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
) -> Response {
    let manager = state.passport_manager.read().await;

    match manager.get(&id) {
        Some(passport) => Json(ApiResponse::success(&passport.vaccinations)).into_response(),
        None => (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("NOT_FOUND", "Passport not found"))
        ).into_response(),
    }
}

/// GET /api/v1/passports/:id/vaccinations/schedule - Get vaccination schedule
async fn get_vaccination_schedule(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
) -> Response {
    let manager = state.passport_manager.read().await;

    match manager.get(&id) {
        Some(passport) => {
            let schedule = state.vaccination_scheduler.calculate_schedule(passport);
            Json(ApiResponse::success(schedule)).into_response()
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("NOT_FOUND", "Passport not found"))
        ).into_response(),
    }
}

// ============================================================================
// Quarantine Endpoints
// ============================================================================

/// POST /api/v1/quarantine/check - Check quarantine eligibility
async fn check_quarantine(
    State(state): State<Arc<AppState>>,
    Json(request): Json<QuarantineCheckRequest>,
) -> Response {
    let manager = state.passport_manager.read().await;

    match manager.get(&request.passport_id) {
        Some(passport) => {
            let result = state.quarantine_checker.check_eligibility(
                passport,
                &request.destination_country
            );

            let _ = state.broadcast_tx.send(WebSocketEvent::QuarantineStatusChanged {
                passport_id: request.passport_id.clone(),
                eligible: result.eligible,
            });

            Json(ApiResponse::success(result)).into_response()
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("NOT_FOUND", "Passport not found"))
        ).into_response(),
    }
}

/// GET /api/v1/quarantine/requirements/:country - Get country requirements
async fn get_country_requirements(
    Path(country): Path<String>,
) -> Response {
    // Return country-specific requirements
    let requirements = match country.to_uppercase().as_str() {
        "AU" => serde_json::json!({
            "country": "AU",
            "name": "Australia",
            "quarantine_days": 10,
            "required_vaccines": ["Rabies"],
            "rabies_titer_required": true,
            "microchip_required": true,
            "import_permit_required": true,
            "approved_countries": ["US", "UK", "JP", "NZ", "SG"]
        }),
        "JP" => serde_json::json!({
            "country": "JP",
            "name": "Japan",
            "quarantine_days": 0,
            "required_vaccines": ["Rabies"],
            "rabies_titer_required": true,
            "microchip_required": true,
            "waiting_period_days": 180
        }),
        "EU" | "DE" | "FR" | "IT" | "ES" => serde_json::json!({
            "country": country.to_uppercase(),
            "name": "European Union",
            "quarantine_days": 0,
            "required_vaccines": ["Rabies"],
            "rabies_titer_required": false,
            "microchip_required": true,
            "eu_pet_passport_accepted": true
        }),
        "US" => serde_json::json!({
            "country": "US",
            "name": "United States",
            "quarantine_days": 0,
            "required_vaccines": ["Rabies"],
            "rabies_titer_required": false,
            "microchip_required": false,
            "health_certificate_required": true
        }),
        "KR" => serde_json::json!({
            "country": "KR",
            "name": "South Korea",
            "quarantine_days": 0,
            "required_vaccines": ["Rabies"],
            "rabies_titer_required": true,
            "microchip_required": true
        }),
        _ => serde_json::json!({
            "country": country.to_uppercase(),
            "name": "Unknown",
            "message": "Requirements not available. Please check with local authorities."
        }),
    };

    Json(ApiResponse::success(requirements)).into_response()
}

// ============================================================================
// Health Endpoints
// ============================================================================

/// GET /api/v1/passports/:id/health/risk - Assess health risk
async fn assess_health_risk(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
) -> Response {
    let manager = state.passport_manager.read().await;

    match manager.get(&id) {
        Some(passport) => {
            let risk = state.health_risk_assessor.assess(passport);

            // Send alert if high risk
            if risk.overall_score > 0.7 {
                let _ = state.broadcast_tx.send(WebSocketEvent::HealthAlert {
                    passport_id: id.clone(),
                    message: format!("High health risk detected: {:.0}%", risk.overall_score * 100.0),
                });
            }

            Json(ApiResponse::success(risk)).into_response()
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("NOT_FOUND", "Passport not found"))
        ).into_response(),
    }
}

// ============================================================================
// Emergency Endpoints
// ============================================================================

/// GET /api/v1/emergency/:id - Get emergency info by passport ID
async fn get_emergency_info(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
) -> Response {
    let manager = state.passport_manager.read().await;

    match manager.get(&id) {
        Some(passport) => {
            let info = state.emergency_extractor.extract(passport);
            Json(ApiResponse::success(info)).into_response()
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("NOT_FOUND", "Passport not found"))
        ).into_response(),
    }
}

/// GET /api/v1/emergency/qr/:code - Get emergency info by QR code
async fn get_emergency_by_qr(
    State(state): State<Arc<AppState>>,
    Path(code): Path<String>,
) -> Response {
    // QR code format: WIAPET:<passport_id>
    let passport_id = if code.starts_with("WIAPET:") {
        code.strip_prefix("WIAPET:").unwrap_or(&code)
    } else {
        &code
    };

    let manager = state.passport_manager.read().await;

    match manager.get(passport_id) {
        Some(passport) => {
            let info = state.emergency_extractor.extract(passport);
            Json(ApiResponse::success(info)).into_response()
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("NOT_FOUND", "Invalid QR code or passport not found"))
        ).into_response(),
    }
}

// ============================================================================
// Verification Endpoint
// ============================================================================

#[derive(Debug, Deserialize)]
pub struct VerifyRequest {
    pub passport_id: String,
    pub signature: String,
    pub public_key: String,
}

/// POST /api/v1/verify - Verify document signature
async fn verify_document(
    State(state): State<Arc<AppState>>,
    Json(request): Json<VerifyRequest>,
) -> Response {
    let manager = state.passport_manager.read().await;

    match manager.get(&request.passport_id) {
        Some(passport) => {
            let result = state.document_verifier.verify(
                passport,
                &request.signature,
                &request.public_key
            );
            Json(ApiResponse::success(result)).into_response()
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("NOT_FOUND", "Passport not found"))
        ).into_response(),
    }
}

// ============================================================================
// WebSocket Handler
// ============================================================================

async fn websocket_handler(
    ws: WebSocketUpgrade,
    State(state): State<Arc<AppState>>,
) -> impl IntoResponse {
    ws.on_upgrade(|socket| handle_websocket(socket, state))
}

async fn handle_websocket(mut socket: WebSocket, state: Arc<AppState>) {
    let mut rx = state.broadcast_tx.subscribe();

    info!("WebSocket client connected");

    // Send welcome message
    let welcome = serde_json::json!({
        "type": "connected",
        "message": "Connected to WIA Pet Health Passport real-time updates",
        "version": crate::VERSION
    });

    if socket.send(Message::Text(welcome.to_string().into())).await.is_err() {
        return;
    }

    loop {
        tokio::select! {
            // Receive from broadcast channel
            result = rx.recv() => {
                match result {
                    Ok(event) => {
                        let json = serde_json::to_string(&event).unwrap_or_default();
                        if socket.send(Message::Text(json.into())).await.is_err() {
                            break;
                        }
                    }
                    Err(broadcast::error::RecvError::Lagged(n)) => {
                        warn!("WebSocket client lagged by {} messages", n);
                    }
                    Err(_) => break,
                }
            }

            // Receive from client
            msg = socket.recv() => {
                match msg {
                    Some(Ok(Message::Text(text))) => {
                        // Handle client messages (subscribe to specific passport, etc.)
                        info!("Received from client: {}", text);
                    }
                    Some(Ok(Message::Ping(data))) => {
                        if socket.send(Message::Pong(data)).await.is_err() {
                            break;
                        }
                    }
                    Some(Ok(Message::Close(_))) | None => break,
                    _ => {}
                }
            }
        }
    }

    info!("WebSocket client disconnected");
}

// ============================================================================
// Server Entry Point
// ============================================================================

/// Start the HTTP server
pub async fn start_server(host: &str, port: u16) -> anyhow::Result<()> {
    let state = Arc::new(AppState::new());
    let app = create_router(state);

    let addr = format!("{}:{}", host, port);
    info!("Starting WIA Pet Health Passport server on {}", addr);
    info!("홍익인간 (弘益人間) - Benefit All Humanity & Animals");

    let listener = tokio::net::TcpListener::bind(&addr).await?;
    axum::serve(listener, app).await?;

    Ok(())
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use axum::body::Body;
    use axum::http::Request;
    use tower::ServiceExt;

    #[tokio::test]
    async fn test_health_check() {
        let state = Arc::new(AppState::new());
        let app = create_router(state);

        let response = app
            .oneshot(Request::builder().uri("/health").body(Body::empty()).unwrap())
            .await
            .unwrap();

        assert_eq!(response.status(), StatusCode::OK);
    }

    #[tokio::test]
    async fn test_version() {
        let state = Arc::new(AppState::new());
        let app = create_router(state);

        let response = app
            .oneshot(Request::builder().uri("/version").body(Body::empty()).unwrap())
            .await
            .unwrap();

        assert_eq!(response.status(), StatusCode::OK);
    }
}
