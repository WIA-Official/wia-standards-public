//! WIA Battery Passport - HTTP/WebSocket Server
//!
//! Phase 3: REST API and real-time updates
//!
//! 홍익인간 (弘益人間) - Benefit All Humanity

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

use crate::types::*;
use crate::core::*;

// ============================================================================
// Server State
// ============================================================================

/// Shared application state
pub struct AppState {
    pub passport_manager: RwLock<PassportManager>,
    pub soh_calculator: SohCalculator,
    pub rul_predictor: RulPredictor,
    pub carbon_calculator: CarbonCalculator,
    pub second_life_assessor: SecondLifeAssessor,
    pub supply_chain_verifier: SupplyChainVerifier,
    pub recycling_calculator: RecyclingCalculator,
    pub broadcast_tx: broadcast::Sender<WebSocketEvent>,
}

impl AppState {
    pub fn new() -> Self {
        let (broadcast_tx, _) = broadcast::channel(1000);

        Self {
            passport_manager: RwLock::new(PassportManager::new()),
            soh_calculator: SohCalculator::new(),
            rul_predictor: RulPredictor::new(),
            carbon_calculator: CarbonCalculator::new(),
            second_life_assessor: SecondLifeAssessor::new(),
            supply_chain_verifier: SupplyChainVerifier::new(),
            recycling_calculator: RecyclingCalculator::new(),
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
    PassportCreated { passport_id: String },
    PassportUpdated { passport_id: String },
    HealthUpdated { passport_id: String, soh: f64 },
    SohThreshold { passport_id: String, soh: f64, threshold: f64 },
    LifecycleEvent { passport_id: String, event_type: String },
    Alert { passport_id: String, severity: String, message: String },
}

// ============================================================================
// API Types
// ============================================================================

#[derive(Debug, Deserialize)]
pub struct QueryParams {
    pub chemistry: Option<String>,
    pub manufacturer: Option<String>,
    pub page: Option<usize>,
    pub limit: Option<usize>,
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
        Self { success: true, data: Some(data), error: None }
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

#[derive(Debug, Deserialize)]
pub struct HealthUpdateRequest {
    pub state_of_health_percent: f64,
    pub state_of_charge_percent: f64,
    pub current_capacity_ah: Option<f64>,
    pub internal_resistance_mohm: Option<f64>,
    pub total_energy_throughput_kwh: Option<f64>,
}

#[derive(Debug, Deserialize)]
pub struct RulQueryParams {
    pub usage_profile: Option<String>,
}

#[derive(Debug, Deserialize)]
pub struct RecyclingRecordRequest {
    pub materials_recovered: HashMap<String, f64>,
    pub process_type: String,
}

// ============================================================================
// Router
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

        // QR lookup
        .route("/api/v1/passports/qr/:qr_code", get(get_passport_by_qr))

        // Health data
        .route("/api/v1/passports/:id/health", get(get_health))
        .route("/api/v1/passports/:id/health", post(update_health))

        // SOH calculation
        .route("/api/v1/passports/:id/soh/calculate", get(calculate_soh))

        // RUL prediction
        .route("/api/v1/passports/:id/rul/predict", get(predict_rul))

        // Carbon footprint
        .route("/api/v1/passports/:id/carbon", get(get_carbon))
        .route("/api/v1/passports/:id/carbon/calculate", post(calculate_carbon))

        // Second-life assessment
        .route("/api/v1/passports/:id/second-life/assess", get(assess_second_life))

        // Supply chain
        .route("/api/v1/passports/:id/supply-chain/verify", post(verify_supply_chain))

        // Recycling
        .route("/api/v1/passports/:id/recycling", get(get_recycling_info))
        .route("/api/v1/passports/:id/recycling/record", post(record_recycling))

        // Lifecycle
        .route("/api/v1/passports/:id/lifecycle", get(get_lifecycle))
        .route("/api/v1/passports/:id/lifecycle", post(add_lifecycle_event))

        // WebSocket
        .route("/ws", get(websocket_handler))

        .with_state(state)
}

// ============================================================================
// Health & Version
// ============================================================================

async fn health_check() -> impl IntoResponse {
    Json(serde_json::json!({
        "status": "healthy",
        "service": "wia-battery-passport",
        "timestamp": chrono::Utc::now().to_rfc3339()
    }))
}

async fn version() -> impl IntoResponse {
    Json(serde_json::json!({
        "version": crate::VERSION,
        "standard": "WIA-BATTERY-PASSPORT",
        "eu_regulation": crate::EU_REGULATION,
        "supported_chemistries": crate::SUPPORTED_CHEMISTRIES
    }))
}

// ============================================================================
// Passport CRUD
// ============================================================================

async fn create_passport(
    State(state): State<Arc<AppState>>,
    Json(passport): Json<BatteryPassport>,
) -> impl IntoResponse {
    let mut manager = state.passport_manager.write().await;
    let passport_id = passport.id.clone();

    match manager.create(passport) {
        Ok(_) => {
            info!("Created passport: {}", passport_id);
            let _ = state.broadcast_tx.send(WebSocketEvent::PassportCreated {
                passport_id: passport_id.clone(),
            });
            (StatusCode::CREATED, Json(ApiResponse::success(serde_json::json!({
                "passport_id": passport_id,
                "created_at": chrono::Utc::now().to_rfc3339()
            }))))
        }
        Err(e) => {
            error!("Failed to create passport: {:?}", e);
            (StatusCode::BAD_REQUEST, Json(ApiResponse::success(serde_json::json!({
                "error": format!("{:?}", e)
            }))))
        }
    }
}

async fn list_passports(
    State(state): State<Arc<AppState>>,
    Query(params): Query<QueryParams>,
) -> impl IntoResponse {
    let manager = state.passport_manager.read().await;
    let passports: Vec<_> = manager.list()
        .into_iter()
        .filter(|p| {
            if let Some(ref chem) = params.chemistry {
                let chem_str = format!("{:?}", p.identity.chemistry).to_lowercase();
                if !chem_str.contains(&chem.to_lowercase()) {
                    return false;
                }
            }
            if let Some(ref mfr) = params.manufacturer {
                if !p.manufacturer.name.to_lowercase().contains(&mfr.to_lowercase()) {
                    return false;
                }
            }
            true
        })
        .skip(params.page.unwrap_or(0) * params.limit.unwrap_or(50))
        .take(params.limit.unwrap_or(50))
        .cloned()
        .collect();

    Json(ApiResponse::success(passports))
}

async fn get_passport(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
) -> Response {
    let manager = state.passport_manager.read().await;

    match manager.get(&id) {
        Some(passport) => Json(ApiResponse::success(passport.clone())).into_response(),
        None => (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("PASSPORT_NOT_FOUND", "Passport not found"))
        ).into_response(),
    }
}

async fn get_passport_by_qr(
    State(state): State<Arc<AppState>>,
    Path(qr_code): Path<String>,
) -> Response {
    let manager = state.passport_manager.read().await;

    match manager.get_by_qr(&qr_code) {
        Some(passport) => Json(ApiResponse::success(passport.clone())).into_response(),
        None => (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("INVALID_QR_CODE", "QR code not found"))
        ).into_response(),
    }
}

async fn update_passport(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
    Json(passport): Json<BatteryPassport>,
) -> Response {
    let mut manager = state.passport_manager.write().await;

    match manager.update(passport) {
        Ok(_) => {
            let _ = state.broadcast_tx.send(WebSocketEvent::PassportUpdated {
                passport_id: id.clone(),
            });
            Json(ApiResponse::success(serde_json::json!({"updated": true}))).into_response()
        }
        Err(e) => (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("UPDATE_FAILED", &format!("{:?}", e)))
        ).into_response(),
    }
}

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
// Health Endpoints
// ============================================================================

async fn get_health(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
) -> Response {
    let manager = state.passport_manager.read().await;

    match manager.get(&id) {
        Some(passport) => Json(ApiResponse::success(&passport.health)).into_response(),
        None => (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("NOT_FOUND", "Passport not found"))
        ).into_response(),
    }
}

async fn update_health(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
    Json(request): Json<HealthUpdateRequest>,
) -> Response {
    let mut manager = state.passport_manager.write().await;

    let passport = match manager.get(&id) {
        Some(p) => p.clone(),
        None => return (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("NOT_FOUND", "Passport not found"))
        ).into_response(),
    };

    let mut health = passport.health.clone();
    health.state_of_health_percent = request.state_of_health_percent;
    health.state_of_charge_percent = request.state_of_charge_percent;
    health.last_bms_sync = Some(chrono::Utc::now());
    health.data_source = HealthDataSource::Bms;

    if let Some(cap) = request.current_capacity_ah {
        health.current_capacity_ah = cap;
    }
    if let Some(res) = request.internal_resistance_mohm {
        health.internal_resistance_mohm = res;
    }
    if let Some(throughput) = request.total_energy_throughput_kwh {
        health.total_energy_throughput_kwh = throughput;
    }

    // Check SOH threshold
    if request.state_of_health_percent < 80.0 && passport.health.state_of_health_percent >= 80.0 {
        let _ = state.broadcast_tx.send(WebSocketEvent::SohThreshold {
            passport_id: id.clone(),
            soh: request.state_of_health_percent,
            threshold: 80.0,
        });
    }

    match manager.update_health(&id, health) {
        Ok(_) => {
            let _ = state.broadcast_tx.send(WebSocketEvent::HealthUpdated {
                passport_id: id.clone(),
                soh: request.state_of_health_percent,
            });
            (StatusCode::CREATED, Json(ApiResponse::success(serde_json::json!({"updated": true})))).into_response()
        }
        Err(e) => (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(ApiResponse::<()>::error("UPDATE_FAILED", &format!("{:?}", e)))
        ).into_response(),
    }
}

// ============================================================================
// SOH & RUL Endpoints
// ============================================================================

async fn calculate_soh(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
) -> Response {
    let manager = state.passport_manager.read().await;

    match manager.get(&id) {
        Some(passport) => {
            let result = state.soh_calculator.calculate(passport);
            Json(ApiResponse::success(result)).into_response()
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("NOT_FOUND", "Passport not found"))
        ).into_response(),
    }
}

async fn predict_rul(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
    Query(params): Query<RulQueryParams>,
) -> Response {
    let manager = state.passport_manager.read().await;

    match manager.get(&id) {
        Some(passport) => {
            let profile = match params.usage_profile.as_deref() {
                Some("ev_heavy") => UsageProfile {
                    avg_operating_temp_c: 30.0,
                    annual_cycles: 500,
                    avg_depth_of_discharge: 0.85,
                },
                Some("stationary") => UsageProfile {
                    avg_operating_temp_c: 22.0,
                    annual_cycles: 250,
                    avg_depth_of_discharge: 0.6,
                },
                _ => UsageProfile::default(),
            };

            let result = state.rul_predictor.predict(passport, &profile);
            Json(ApiResponse::success(result)).into_response()
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("NOT_FOUND", "Passport not found"))
        ).into_response(),
    }
}

// ============================================================================
// Carbon Endpoints
// ============================================================================

async fn get_carbon(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
) -> Response {
    let manager = state.passport_manager.read().await;

    match manager.get(&id) {
        Some(passport) => Json(ApiResponse::success(&passport.carbon_footprint)).into_response(),
        None => (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("NOT_FOUND", "Passport not found"))
        ).into_response(),
    }
}

async fn calculate_carbon(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
) -> Response {
    let manager = state.passport_manager.read().await;

    match manager.get(&id) {
        Some(passport) => {
            let result = state.carbon_calculator.calculate(passport);
            Json(ApiResponse::success(result)).into_response()
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("NOT_FOUND", "Passport not found"))
        ).into_response(),
    }
}

// ============================================================================
// Second-Life & Supply Chain
// ============================================================================

async fn assess_second_life(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
) -> Response {
    let manager = state.passport_manager.read().await;

    match manager.get(&id) {
        Some(passport) => {
            let result = state.second_life_assessor.assess(passport);
            Json(ApiResponse::success(result)).into_response()
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("NOT_FOUND", "Passport not found"))
        ).into_response(),
    }
}

async fn verify_supply_chain(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
) -> Response {
    let manager = state.passport_manager.read().await;

    match manager.get(&id) {
        Some(passport) => {
            let result = state.supply_chain_verifier.verify(passport);
            Json(ApiResponse::success(result)).into_response()
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("NOT_FOUND", "Passport not found"))
        ).into_response(),
    }
}

// ============================================================================
// Recycling Endpoints
// ============================================================================

async fn get_recycling_info(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
) -> Response {
    let manager = state.passport_manager.read().await;

    match manager.get(&id) {
        Some(passport) => Json(ApiResponse::success(&passport.recycling)).into_response(),
        None => (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("NOT_FOUND", "Passport not found"))
        ).into_response(),
    }
}

async fn record_recycling(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
    Json(request): Json<RecyclingRecordRequest>,
) -> Response {
    let manager = state.passport_manager.read().await;

    match manager.get(&id) {
        Some(passport) => {
            let result = state.recycling_calculator.calculate(passport, &request.materials_recovered);
            Json(ApiResponse::success(result)).into_response()
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("NOT_FOUND", "Passport not found"))
        ).into_response(),
    }
}

// ============================================================================
// Lifecycle Endpoints
// ============================================================================

async fn get_lifecycle(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
) -> Response {
    let manager = state.passport_manager.read().await;

    match manager.get(&id) {
        Some(passport) => Json(ApiResponse::success(&passport.lifecycle)).into_response(),
        None => (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("NOT_FOUND", "Passport not found"))
        ).into_response(),
    }
}

async fn add_lifecycle_event(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
    Json(event): Json<LifecycleEvent>,
) -> Response {
    let mut manager = state.passport_manager.write().await;
    let event_type = format!("{:?}", event.event_type);

    match manager.add_lifecycle_event(&id, event) {
        Ok(_) => {
            let _ = state.broadcast_tx.send(WebSocketEvent::LifecycleEvent {
                passport_id: id.clone(),
                event_type,
            });
            (StatusCode::CREATED, Json(ApiResponse::success(serde_json::json!({"added": true})))).into_response()
        }
        Err(e) => (
            StatusCode::NOT_FOUND,
            Json(ApiResponse::<()>::error("NOT_FOUND", &format!("{:?}", e)))
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

    let welcome = serde_json::json!({
        "type": "connected",
        "message": "Connected to WIA Battery Passport real-time updates",
        "version": crate::VERSION
    });

    if socket.send(Message::Text(welcome.to_string().into())).await.is_err() {
        return;
    }

    loop {
        tokio::select! {
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

            msg = socket.recv() => {
                match msg {
                    Some(Ok(Message::Text(text))) => {
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
    info!("Starting WIA Battery Passport server on {}", addr);
    info!("EU Battery Regulation: {}", crate::EU_REGULATION);
    info!("홍익인간 (弘益人間) - Benefit All Humanity");

    let listener = tokio::net::TcpListener::bind(&addr).await?;
    axum::serve(listener, app).await?;

    Ok(())
}
