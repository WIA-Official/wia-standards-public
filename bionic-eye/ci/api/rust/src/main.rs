//! WIA CI Octave Enhancement - API Server
//!
//! 인공와우 사용자를 위한 옥타브 인핸스먼트 API 서버
//!
//! ## 사용법
//!
//! ```bash
//! # 기본 실행 (포트 3000)
//! wia-ci
//!
//! # 포트 지정
//! wia-ci --port 8080
//!
//! # 샘플레이트 지정
//! wia-ci --sample-rate 44100
//! ```

use std::env;
use wia_ci::{run_server, ServerConfig};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 환경변수 또는 명령줄 인자에서 설정 읽기
    let args: Vec<String> = env::args().collect();

    let mut config = ServerConfig::default();

    // 간단한 인자 파싱
    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--port" | "-p" => {
                if i + 1 < args.len() {
                    config.port = args[i + 1].parse().unwrap_or(3000);
                    i += 1;
                }
            }
            "--host" | "-h" => {
                if i + 1 < args.len() {
                    config.host = args[i + 1].clone();
                    i += 1;
                }
            }
            "--sample-rate" | "-s" => {
                if i + 1 < args.len() {
                    config.sample_rate = args[i + 1].parse().unwrap_or(44100);
                    i += 1;
                }
            }
            "--help" => {
                print_help();
                return Ok(());
            }
            _ => {}
        }
        i += 1;
    }

    // 환경변수 오버라이드
    if let Ok(port) = env::var("WIA_CI_PORT") {
        config.port = port.parse().unwrap_or(config.port);
    }
    if let Ok(host) = env::var("WIA_CI_HOST") {
        config.host = host;
    }
    if let Ok(sr) = env::var("WIA_CI_SAMPLE_RATE") {
        config.sample_rate = sr.parse().unwrap_or(config.sample_rate);
    }

    // 서버 실행
    run_server(config).await
}

fn print_help() {
    println!(r#"
WIA CI Octave Enhancement API Server
=====================================

인공와우(Cochlear Implant) 사용자를 위한 옥타브 인핸스먼트 API

Usage:
    wia-ci [OPTIONS]

Options:
    -p, --port <PORT>           서버 포트 (기본: 3000)
    -h, --host <HOST>           서버 호스트 (기본: 0.0.0.0)
    -s, --sample-rate <RATE>    샘플레이트 (기본: 44100)
    --help                      도움말 표시

Environment Variables:
    WIA_CI_PORT                 서버 포트
    WIA_CI_HOST                 서버 호스트
    WIA_CI_SAMPLE_RATE          샘플레이트

Examples:
    wia-ci                      # 기본 설정으로 실행
    wia-ci -p 8080              # 포트 8080으로 실행
    wia-ci -s 16000             # 16kHz 샘플레이트로 실행

API Endpoints:
    GET  /                      서버 정보
    GET  /health                헬스체크
    POST /api/v1/pitch          피치/옥타브 검출
    POST /api/v1/enhance        CI 신호 인핸스먼트
    POST /api/v1/process        전체 파이프라인
    WS   /api/v1/ws             실시간 WebSocket

Philosophy:
    홍익인간 (弘益人間) - 널리 인간을 이롭게 하라

License:
    MIT License - Copyright (c) 2025 SmileStory Inc. / WIA
"#);
}
