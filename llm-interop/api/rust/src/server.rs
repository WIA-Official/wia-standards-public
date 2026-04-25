//! WIA-LLM-INTEROP REST API Server
//!
//! HTTP/WebSocket server for AI interoperability.
//!
//! 홍익인간 (弘益人間) - Benefit All Humanity

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

use crate::core::*;
use crate::types::*;

/// Application state
pub struct AppState {
    pub capability_registry: RwLock<CapabilityRegistry>,
    pub federation_manager: RwLock<FederationManager>,
    pub task_manager: RwLock<TaskManager>,
    pub consensus_engine: RwLock<ConsensusEngine>,
}

impl AppState {
    pub fn new() -> Self {
        Self {
            capability_registry: RwLock::new(CapabilityRegistry::new()),
            federation_manager: RwLock::new(FederationManager::new()),
            task_manager: RwLock::new(TaskManager::new()),
            consensus_engine: RwLock::new(ConsensusEngine::new()),
        }
    }
}

impl Default for AppState {
    fn default() -> Self {
        Self::new()
    }
}

/// API Response wrapper
#[derive(Debug, Serialize)]
pub struct ApiResponse<T: Serialize> {
    pub success: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data: Option<T>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<ApiError>,
}

#[derive(Debug, Serialize)]
pub struct ApiError {
    pub code: String,
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<serde_json::Value>,
}

impl<T: Serialize> ApiResponse<T> {
    pub fn success(data: T) -> Self {
        Self {
            success: true,
            data: Some(data),
            error: None,
        }
    }
}

impl ApiResponse<()> {
    pub fn error(code: &str, message: &str) -> Self {
        Self {
            success: false,
            data: None,
            error: Some(ApiError {
                code: code.to_string(),
                message: message.to_string(),
                details: None,
            }),
        }
    }
}

/// Health response
#[derive(Debug, Serialize)]
pub struct HealthResponse {
    pub status: String,
    pub version: String,
    pub wia_version: String,
    pub philosophy: String,
    pub capabilities: ServiceCapabilities,
}

#[derive(Debug, Serialize)]
pub struct ServiceCapabilities {
    pub capability_registry: bool,
    pub message_routing: bool,
    pub federation: bool,
    pub consensus: bool,
    pub websocket: bool,
}

/// Version response
#[derive(Debug, Serialize)]
pub struct VersionResponse {
    pub version: String,
    pub wia_version: String,
    pub api_version: String,
    pub standard: String,
    pub components: Vec<String>,
}

/// Capability query params
#[derive(Debug, Deserialize)]
pub struct CapabilityQueryParams {
    pub domain: Option<String>,
    pub min_level: Option<u8>,
    pub language: Option<String>,
    pub limit: Option<usize>,
}

/// Create federation request
#[derive(Debug, Deserialize)]
pub struct CreateFederationRequest {
    pub name: String,
    pub topology: FederationTopology,
    #[serde(default)]
    pub description: Option<String>,
}

/// Join federation request
#[derive(Debug, Deserialize)]
pub struct JoinFederationRequest {
    pub model_id: String,
    pub capability_uri: String,
    pub role: FederationRole,
    #[serde(default)]
    pub domains: Vec<DomainCode>,
    pub endpoint: String,
}

/// Submit task request
#[derive(Debug, Deserialize)]
pub struct SubmitTaskRequest {
    pub query: String,
    #[serde(default)]
    pub description: Option<String>,
    #[serde(default)]
    pub timeout_seconds: Option<u32>,
}

/// Start consensus request
#[derive(Debug, Deserialize)]
pub struct StartConsensusRequest {
    pub question: ConsensusQuestion,
    #[serde(default = "default_timeout")]
    pub timeout_seconds: u32,
}

fn default_timeout() -> u32 {
    30
}

/// Vote request
#[derive(Debug, Deserialize)]
pub struct VoteRequest {
    pub voter_id: String,
    pub vote: serde_json::Value,
    pub confidence: f64,
    #[serde(default)]
    pub reasoning: Option<String>,
}

/// Message request
#[derive(Debug, Deserialize)]
pub struct SendMessageRequest {
    pub to: MessageTarget,
    pub payload: QueryPayload,
    #[serde(default)]
    pub priority: Option<Priority>,
}

/// Start the API server
pub async fn start_server(host: &str, port: u16) -> anyhow::Result<()> {
    let state = Arc::new(AppState::new());

    let cors = CorsLayer::new()
        .allow_origin(Any)
        .allow_methods([Method::GET, Method::POST, Method::PUT, Method::DELETE])
        .allow_headers([header::CONTENT_TYPE, header::AUTHORIZATION]);

    let app = Router::new()
        // Health & Info
        .route("/health", get(health_check))
        .route("/version", get(version_info))
        // Well-known discovery
        .route("/.well-known/wia-llm-interop", get(well_known))
        // Capability endpoints
        .route("/wia/llm-interop/v1/capability", get(list_capabilities))
        .route("/wia/llm-interop/v1/capability", post(register_capability))
        .route("/wia/llm-interop/v1/capability/:model_id", get(get_capability))
        .route("/wia/llm-interop/v1/capability/query", post(query_capabilities))
        // Message endpoints
        .route("/wia/llm-interop/v1/message", post(send_message))
        // Federation endpoints
        .route("/wia/llm-interop/v1/federation", get(list_federations))
        .route("/wia/llm-interop/v1/federation", post(create_federation))
        .route("/wia/llm-interop/v1/federation/:federation_id", get(get_federation))
        .route("/wia/llm-interop/v1/federation/:federation_id/join", post(join_federation))
        .route("/wia/llm-interop/v1/federation/:federation_id/leave/:member_id", post(leave_federation))
        // Task endpoints
        .route("/wia/llm-interop/v1/federation/:federation_id/task", post(submit_task))
        .route("/wia/llm-interop/v1/federation/:federation_id/task/:task_id", get(get_task))
        // Consensus endpoints
        .route("/wia/llm-interop/v1/federation/:federation_id/consensus", post(start_consensus))
        .route("/wia/llm-interop/v1/federation/:federation_id/consensus/:consensus_id/vote", post(submit_vote))
        .route("/wia/llm-interop/v1/federation/:federation_id/consensus/:consensus_id", get(get_consensus))
        // WebSocket
        .route("/wia/llm-interop/v1/ws", get(ws_handler))
        // Middleware
        .layer(cors)
        .layer(TraceLayer::new_for_http())
        .with_state(state);

    let addr = format!("{}:{}", host, port);
    let listener = tokio::net::TcpListener::bind(&addr).await?;

    tracing::info!("🌐 WIA-LLM-INTEROP API Server");
    tracing::info!("📍 Listening on http://{}", addr);
    tracing::info!("📋 API: /wia/llm-interop/v1/");
    tracing::info!("🔌 WebSocket: /wia/llm-interop/v1/ws");
    tracing::info!("💫 홍익인간 (弘益人間) - Benefit All Humanity");

    axum::serve(listener, app).await?;
    Ok(())
}

/// Health check
async fn health_check() -> Json<ApiResponse<HealthResponse>> {
    Json(ApiResponse::success(HealthResponse {
        status: "healthy".to_string(),
        version: crate::VERSION.to_string(),
        wia_version: crate::WIA_VERSION.to_string(),
        philosophy: crate::PHILOSOPHY.to_string(),
        capabilities: ServiceCapabilities {
            capability_registry: true,
            message_routing: true,
            federation: true,
            consensus: true,
            websocket: true,
        },
    }))
}

/// Version info
async fn version_info() -> Json<ApiResponse<VersionResponse>> {
    Json(ApiResponse::success(VersionResponse {
        version: crate::VERSION.to_string(),
        wia_version: crate::WIA_VERSION.to_string(),
        api_version: "v1".to_string(),
        standard: "WIA-LLM-INTEROP".to_string(),
        components: vec![
            "WIA-LLM-CAPABILITY".to_string(),
            "WIA-LLM-MESSAGE".to_string(),
            "WIA-LLM-FEDERATION".to_string(),
        ],
    }))
}

/// Well-known discovery endpoint
async fn well_known() -> Json<serde_json::Value> {
    Json(serde_json::json!({
        "wia_llm_interop_version": crate::WIA_VERSION,
        "endpoints": {
            "capability": "/wia/llm-interop/v1/capability",
            "message": "/wia/llm-interop/v1/message",
            "federation": "/wia/llm-interop/v1/federation",
            "websocket": "/wia/llm-interop/v1/ws"
        },
        "supported_auth": ["api_key", "did_auth"],
        "documentation": "https://wia.org/standards/llm-interop"
    }))
}

/// List capabilities
async fn list_capabilities(
    State(state): State<Arc<AppState>>,
    Query(params): Query<CapabilityQueryParams>,
) -> Json<ApiResponse<Vec<CapabilityDocument>>> {
    let registry = state.capability_registry.read().await;
    let mut capabilities: Vec<CapabilityDocument> = registry.list().into_iter().cloned().collect();

    // Apply filters
    if let Some(domain) = params.domain {
        capabilities.retain(|c| {
            c.domains.iter().any(|d| format!("{:?}", d.domain).to_lowercase() == domain.to_lowercase())
        });
    }

    if let Some(min_level) = params.min_level {
        capabilities.retain(|c| c.level.0 >= min_level);
    }

    if let Some(ref lang) = params.language {
        capabilities.retain(|c| c.languages.iter().any(|l| &l.code == lang));
    }

    if let Some(limit) = params.limit {
        capabilities.truncate(limit);
    }

    Json(ApiResponse::success(capabilities))
}

/// Register capability
async fn register_capability(
    State(state): State<Arc<AppState>>,
    Json(capability): Json<CapabilityDocument>,
) -> Result<Json<ApiResponse<CapabilityDocument>>, (StatusCode, Json<ApiResponse<()>>)> {
    let mut registry = state.capability_registry.write().await;
    registry.register(capability.clone());

    Ok(Json(ApiResponse::success(capability)))
}

/// Get capability by model ID
async fn get_capability(
    State(state): State<Arc<AppState>>,
    Path(model_id): Path<String>,
) -> Result<Json<ApiResponse<CapabilityDocument>>, (StatusCode, Json<ApiResponse<()>>)> {
    let registry = state.capability_registry.read().await;

    // URL decode the model_id (DIDs contain colons)
    let decoded_id = urlencoding::decode(&model_id)
        .map(|s| s.into_owned())
        .unwrap_or(model_id);

    match registry.get(&decoded_id) {
        Some(cap) => Ok(Json(ApiResponse::success(cap.clone()))),
        None => Err((
            StatusCode::NOT_FOUND,
            Json(ApiResponse::error("E100", "Capability not found")),
        )),
    }
}

/// Query capabilities
async fn query_capabilities(
    State(state): State<Arc<AppState>>,
    Json(query): Json<CapabilityQuery>,
) -> Json<ApiResponse<Vec<CapabilityMatch>>> {
    let registry = state.capability_registry.read().await;
    let matches = registry.query(&query);

    Json(ApiResponse::success(matches))
}

/// Send message (mock - would route to actual AI)
async fn send_message(
    State(_state): State<Arc<AppState>>,
    Json(request): Json<SendMessageRequest>,
) -> Result<Json<ApiResponse<MessageEnvelope>>, (StatusCode, Json<ApiResponse<()>>)> {
    let now = chrono::Utc::now();

    // Create response message (mock)
    let response = MessageEnvelope {
        wia_version: crate::WIA_VERSION.to_string(),
        document_type: "message".to_string(),
        message_id: Uuid::now_v7(),
        timestamp: now,
        trace_id: Uuid::now_v7(),
        parent_message_id: None,
        sequence_number: None,
        from: MessageParticipant {
            model_id: request.to.model_id.clone().unwrap_or_else(|| "server".to_string()),
            capability_uri: None,
            instance_id: None,
        },
        to: MessageTarget {
            model_id: Some("client".to_string()),
            capability_query: None,
            broadcast: None,
        },
        message_type: MessageType::Response,
        priority: request.priority.unwrap_or(Priority::Normal),
        ttl_seconds: None,
        payload: MessagePayload::Response(ResponsePayload {
            content: format!("Received query: {}", request.payload.content),
            confidence: 0.95,
            reasoning: None,
            sources: vec![],
            suggestions: vec![],
            delegations: vec![],
            usage: UsageStats {
                input_tokens: request.payload.content.len() as u32,
                output_tokens: 50,
                total_tokens: request.payload.content.len() as u32 + 50,
                processing_time_ms: 100,
                model_id: "server".to_string(),
            },
        }),
        metadata: None,
        signature: None,
    };

    Ok(Json(ApiResponse::success(response)))
}

/// List federations
async fn list_federations(
    State(state): State<Arc<AppState>>,
) -> Json<ApiResponse<Vec<FederationDocument>>> {
    let manager = state.federation_manager.read().await;
    let federations: Vec<FederationDocument> = manager.list().into_iter().cloned().collect();

    Json(ApiResponse::success(federations))
}

/// Create federation
async fn create_federation(
    State(state): State<Arc<AppState>>,
    Json(request): Json<CreateFederationRequest>,
) -> Json<ApiResponse<FederationDocument>> {
    let mut manager = state.federation_manager.write().await;

    let config = FederationConfig {
        task_config: TaskConfig {
            auto_decompose: true,
            max_subtasks: 10,
            max_parallel_tasks: 5,
            max_retries: 3,
            retry_delay_ms: 1000,
        },
        consensus_config: ConsensusConfig {
            algorithm: ConsensusAlgorithm::Majority,
            quorum_percentage: 0.5,
            voting_timeout_seconds: 30,
            tie_breaker: TieBreaker::Orchestrator,
        },
        fault_tolerance: FaultToleranceConfig {
            circuit_breaker_enabled: true,
            failure_threshold: 3,
            success_threshold: 2,
            timeout_seconds: 10,
            failover_enabled: true,
            min_members_for_operation: 1,
        },
        timeouts: TimeoutConfig {
            handshake_timeout_seconds: 10,
            task_timeout_seconds: 300,
            response_timeout_seconds: 60,
            heartbeat_interval_seconds: 30,
            member_timeout_seconds: 120,
        },
    };

    let federation = manager.create(&request.name, request.topology, config);

    Json(ApiResponse::success(federation))
}

/// Get federation
async fn get_federation(
    State(state): State<Arc<AppState>>,
    Path(federation_id): Path<Uuid>,
) -> Result<Json<ApiResponse<FederationDocument>>, (StatusCode, Json<ApiResponse<()>>)> {
    let manager = state.federation_manager.read().await;

    match manager.get(&federation_id) {
        Some(fed) => Ok(Json(ApiResponse::success(fed.clone()))),
        None => Err((
            StatusCode::NOT_FOUND,
            Json(ApiResponse::error("E600", "Federation not found")),
        )),
    }
}

/// Join federation
async fn join_federation(
    State(state): State<Arc<AppState>>,
    Path(federation_id): Path<Uuid>,
    Json(request): Json<JoinFederationRequest>,
) -> Result<Json<ApiResponse<FederationMember>>, (StatusCode, Json<ApiResponse<()>>)> {
    let mut manager = state.federation_manager.write().await;

    match manager.add_member(
        &federation_id,
        &request.model_id,
        &request.capability_uri,
        request.role,
        request.domains,
        &request.endpoint,
    ) {
        Ok(member) => Ok(Json(ApiResponse::success(member))),
        Err(e) => Err((
            StatusCode::BAD_REQUEST,
            Json(ApiResponse::error("E600", &e.to_string())),
        )),
    }
}

/// Leave federation
async fn leave_federation(
    State(state): State<Arc<AppState>>,
    Path((federation_id, member_id)): Path<(Uuid, Uuid)>,
) -> Result<Json<ApiResponse<()>>, (StatusCode, Json<ApiResponse<()>>)> {
    let mut manager = state.federation_manager.write().await;

    match manager.remove_member(&federation_id, &member_id) {
        Ok(()) => Ok(Json(ApiResponse {
            success: true,
            data: Some(()),
            error: None,
        })),
        Err(e) => Err((
            StatusCode::BAD_REQUEST,
            Json(ApiResponse::error("E600", &e.to_string())),
        )),
    }
}

/// Submit task
async fn submit_task(
    State(state): State<Arc<AppState>>,
    Path(federation_id): Path<Uuid>,
    Json(request): Json<SubmitTaskRequest>,
) -> Result<Json<ApiResponse<TaskDocument>>, (StatusCode, Json<ApiResponse<()>>)> {
    // Verify federation exists
    {
        let manager = state.federation_manager.read().await;
        if manager.get(&federation_id).is_none() {
            return Err((
                StatusCode::NOT_FOUND,
                Json(ApiResponse::error("E600", "Federation not found")),
            ));
        }
    }

    let mut task_manager = state.task_manager.write().await;

    let deadline = request.timeout_seconds.map(|s| {
        chrono::Utc::now() + chrono::Duration::seconds(s as i64)
    });

    let description = request.description.unwrap_or_else(|| request.query.clone());
    let task = task_manager.create_task(federation_id, &request.query, &description, deadline);

    Ok(Json(ApiResponse::success(task)))
}

/// Get task
async fn get_task(
    State(state): State<Arc<AppState>>,
    Path((_federation_id, task_id)): Path<(Uuid, Uuid)>,
) -> Result<Json<ApiResponse<TaskDocument>>, (StatusCode, Json<ApiResponse<()>>)> {
    let task_manager = state.task_manager.read().await;

    match task_manager.get(&task_id) {
        Some(task) => Ok(Json(ApiResponse::success(task.clone()))),
        None => Err((
            StatusCode::NOT_FOUND,
            Json(ApiResponse::error("E601", "Task not found")),
        )),
    }
}

/// Start consensus
async fn start_consensus(
    State(state): State<Arc<AppState>>,
    Path(federation_id): Path<Uuid>,
    Json(request): Json<StartConsensusRequest>,
) -> Result<Json<ApiResponse<ConsensusSession>>, (StatusCode, Json<ApiResponse<()>>)> {
    // Get federation config
    let config = {
        let manager = state.federation_manager.read().await;
        match manager.get(&federation_id) {
            Some(fed) => fed.config.consensus_config.clone(),
            None => {
                return Err((
                    StatusCode::NOT_FOUND,
                    Json(ApiResponse::error("E600", "Federation not found")),
                ));
            }
        }
    };

    let mut engine = state.consensus_engine.write().await;
    let session = engine.start_session(federation_id, request.question, config, request.timeout_seconds);

    Ok(Json(ApiResponse::success(session)))
}

/// Submit vote
async fn submit_vote(
    State(state): State<Arc<AppState>>,
    Path((_federation_id, consensus_id)): Path<(Uuid, Uuid)>,
    Json(request): Json<VoteRequest>,
) -> Result<Json<ApiResponse<()>>, (StatusCode, Json<ApiResponse<()>>)> {
    let mut engine = state.consensus_engine.write().await;

    match engine.vote(
        &consensus_id,
        &request.voter_id,
        request.vote,
        request.confidence,
        request.reasoning,
    ) {
        Ok(()) => Ok(Json(ApiResponse {
            success: true,
            data: Some(()),
            error: None,
        })),
        Err(e) => Err((
            StatusCode::BAD_REQUEST,
            Json(ApiResponse::error("E602", &e.to_string())),
        )),
    }
}

/// Get consensus status
async fn get_consensus(
    State(state): State<Arc<AppState>>,
    Path((_federation_id, consensus_id)): Path<(Uuid, Uuid)>,
) -> Result<Json<ApiResponse<ConsensusSession>>, (StatusCode, Json<ApiResponse<()>>)> {
    let engine = state.consensus_engine.read().await;

    match engine.get(&consensus_id) {
        Some(session) => Ok(Json(ApiResponse::success(session.clone()))),
        None => Err((
            StatusCode::NOT_FOUND,
            Json(ApiResponse::error("E602", "Consensus session not found")),
        )),
    }
}

/// WebSocket handler
async fn ws_handler(
    ws: WebSocketUpgrade,
    State(_state): State<Arc<AppState>>,
) -> Response {
    ws.on_upgrade(handle_socket)
}

async fn handle_socket(mut socket: WebSocket) {
    // Send welcome
    let welcome = serde_json::json!({
        "type": "welcome",
        "wia_version": crate::WIA_VERSION,
        "server_id": "wia-llm-interop-server",
        "capabilities": ["capability", "message", "federation", "consensus"],
        "philosophy": crate::PHILOSOPHY
    });

    if socket.send(Message::Text(welcome.to_string().into())).await.is_err() {
        return;
    }

    // Message loop
    while let Some(msg) = socket.recv().await {
        match msg {
            Ok(Message::Text(text)) => {
                if let Ok(request) = serde_json::from_str::<serde_json::Value>(&text) {
                    let response = handle_ws_message(request).await;
                    if socket.send(Message::Text(response.to_string().into())).await.is_err() {
                        break;
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

async fn handle_ws_message(request: serde_json::Value) -> serde_json::Value {
    let msg_type = request.get("type").and_then(|t| t.as_str()).unwrap_or("");

    match msg_type {
        "ping" => serde_json::json!({
            "type": "pong",
            "timestamp": chrono::Utc::now().to_rfc3339()
        }),
        "capability_request" => serde_json::json!({
            "type": "capability_response",
            "message": "Use REST API for capability queries"
        }),
        "subscribe" => serde_json::json!({
            "type": "subscribed",
            "events": request.get("events"),
            "message": "Subscribed to events"
        }),
        _ => serde_json::json!({
            "type": "error",
            "code": "E200",
            "message": format!("Unknown message type: {}", msg_type)
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_api_response_success() {
        let response = ApiResponse::success("test");
        assert!(response.success);
        assert_eq!(response.data, Some("test"));
    }

    #[test]
    fn test_api_response_error() {
        let response: ApiResponse<()> = ApiResponse::error("E500", "Test error");
        assert!(!response.success);
        assert!(response.error.is_some());
    }
}
