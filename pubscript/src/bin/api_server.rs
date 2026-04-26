//! WIA PubScript REST API Server
//!
//! Provides a REST API for braille conversion with 56-language support.
//!
//! ## Usage
//!
//! ```bash
//! cargo run --bin api_server --features api
//! ```
//!
//! ## Endpoints
//!
//! - `POST /api/convert` - Convert text to braille with auto-detection
//! - `POST /api/convert/:lang` - Convert text to braille with specific language
//! - `GET /api/languages` - List all supported languages
//! - `GET /health` - Health check
//!
//! ## Example
//!
//! ```bash
//! # Auto-detect language
//! curl -X POST http://localhost:3000/api/convert \
//!   -H "Content-Type: application/json" \
//!   -d '{"text": "안녕하세요"}'
//!
//! # Specific language (French)
//! curl -X POST http://localhost:3000/api/convert/fr \
//!   -H "Content-Type: application/json" \
//!   -d '{"text": "bonjour"}'
//!
//! # List languages
//! curl http://localhost:3000/api/languages
//! ```

#[cfg(feature = "api")]
use axum::{
    extract::Path,
    http::{StatusCode, Method},
    response::{IntoResponse, Json},
    routing::{get, post},
    Router,
};

#[cfg(feature = "api")]
use serde::{Deserialize, Serialize};

#[cfg(feature = "api")]
use tower_http::cors::{Any, CorsLayer};

#[cfg(feature = "api")]
use wia_pubscript::braille::{text_to_braille, text_to_braille_with_language, Language, LibLouisTranslator};

#[cfg(feature = "api")]
#[derive(Deserialize)]
struct ConvertRequest {
    text: String,
    #[serde(default)]
    format: Option<String>,
}

#[cfg(feature = "api")]
#[derive(Serialize)]
struct ConvertResponse {
    success: bool,
    braille: Option<String>,
    error: Option<String>,
    language: Option<String>,
}

#[cfg(feature = "api")]
#[derive(Serialize)]
struct LanguageInfo {
    code: String,
    name: String,
    table: String,
}

#[cfg(feature = "api")]
#[derive(Serialize)]
struct LanguagesResponse {
    count: usize,
    languages: Vec<LanguageInfo>,
}

#[cfg(feature = "api")]
#[derive(Serialize)]
struct HealthResponse {
    status: String,
    version: String,
    languages_supported: usize,
}

/// Convert text to braille with auto-detection
#[cfg(feature = "api")]
async fn convert_auto(
    Json(payload): Json<ConvertRequest>,
) -> impl IntoResponse {
    let braille = text_to_braille(&payload.text);

    Json(ConvertResponse {
        success: true,
        braille: Some(braille),
        error: None,
        language: Some("auto-detect".to_string()),
    })
}

/// Convert text to braille with specific language
#[cfg(feature = "api")]
async fn convert_with_lang(
    Path(lang_code): Path<String>,
    Json(payload): Json<ConvertRequest>,
) -> impl IntoResponse {
    let lang = match Language::from_code(&lang_code) {
        Some(l) => l,
        None => {
            return (
                StatusCode::BAD_REQUEST,
                Json(ConvertResponse {
                    success: false,
                    braille: None,
                    error: Some(format!("Unknown language code: '{}'", lang_code)),
                    language: None,
                }),
            );
        }
    };

    match text_to_braille_with_language(&payload.text, lang) {
        Ok(braille) => (
            StatusCode::OK,
            Json(ConvertResponse {
                success: true,
                braille: Some(braille),
                error: None,
                language: Some(lang.name().to_string()),
            }),
        ),
        Err(e) => (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(ConvertResponse {
                success: false,
                braille: None,
                error: Some(e),
                language: Some(lang.name().to_string()),
            }),
        ),
    }
}

/// List all supported languages
#[cfg(feature = "api")]
async fn list_languages() -> impl IntoResponse {
    let translator = LibLouisTranslator::new();
    let languages = translator
        .list_languages()
        .into_iter()
        .map(|(code, name, table)| LanguageInfo { code, name, table })
        .collect::<Vec<_>>();

    Json(LanguagesResponse {
        count: languages.len(),
        languages,
    })
}

/// Health check endpoint
#[cfg(feature = "api")]
async fn health() -> impl IntoResponse {
    Json(HealthResponse {
        status: "healthy".to_string(),
        version: env!("CARGO_PKG_VERSION").to_string(),
        languages_supported: 56,
    })
}

#[cfg(feature = "api")]
#[tokio::main]
async fn main() {
    // Enable CORS for all origins
    let cors = CorsLayer::new()
        .allow_origin(Any)
        .allow_methods([Method::GET, Method::POST])
        .allow_headers(Any);

    // Build our application with routes
    let app = Router::new()
        .route("/health", get(health))
        .route("/api/convert", post(convert_auto))
        .route("/api/convert/:lang", post(convert_with_lang))
        .route("/api/languages", get(list_languages))
        .layer(cors);

    // Run server
    let listener = tokio::net::TcpListener::bind("0.0.0.0:3000")
        .await
        .unwrap();

    println!("🌟 WIA PubScript API Server");
    println!("📡 Listening on http://0.0.0.0:3000");
    println!("🌍 56 languages supported");
    println!();
    println!("Endpoints:");
    println!("  POST /api/convert          - Auto-detect language");
    println!("  POST /api/convert/:lang    - Specific language (e.g., /api/convert/fr)");
    println!("  GET  /api/languages        - List all languages");
    println!("  GET  /health               - Health check");
    println!();
    println!("Example:");
    println!(r#"  curl -X POST http://localhost:3000/api/convert -H "Content-Type: application/json" -d '{{"text": "안녕하세요"}}'"#);

    axum::serve(listener, app).await.unwrap();
}

#[cfg(not(feature = "api"))]
fn main() {
    eprintln!("Error: API server requires the 'api' feature.");
    eprintln!("Build with: cargo run --bin api_server --features api");
    std::process::exit(1);
}
