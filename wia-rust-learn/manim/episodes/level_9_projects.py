"""
WIA-RUST-LEARN Level 9: 실전 프로젝트 (Real-World Projects)
==========================================================

EP 9-01: CLI 앱 - minigrep 만들기 (25분)
EP 9-02: 웹 API - Axum으로 REST API 구축 (28분)
EP 9-03: 시스템 도구 - 파일 감시자 만들기 (22분)

실행 방법:
    manim -pql level_9_projects.py EP_9_01_CLI
    manim -pql level_9_projects.py EP_9_02_WebAPI
    manim -pql level_9_projects.py EP_9_03_SystemTool

전체 렌더링 (고화질):
    manim -pqh level_9_projects.py --all

🎓 이 레벨을 완료하면 WIA-RUST-LEARN 전체 과정 수료!
   Level 0-5 Core → Level 6-8 Expert → Level 9 Master
"""

from manim import *
import numpy as np

# WIA-RUST-LEARN 테마 색상
RUST_ORANGE = "#F74C00"
MEMORY_BLUE = "#4A90D9"
STACK_GREEN = "#2ECC71"
HEAP_PURPLE = "#9B59B6"
ERROR_RED = "#E74C3C"
SUCCESS_GREEN = "#27AE60"
WARNING_YELLOW = "#F39C12"
CLI_CYAN = "#00BCD4"
WEB_ORANGE = "#FF9800"
SYSTEM_TEAL = "#009688"
DARK_BG = "#1a1a2e"


class EP_9_01_CLI(Scene):
    """
    EP 9-01: CLI 앱 - minigrep 만들기

    학습 목표:
    1. 커맨드 라인 인자 처리
    2. 파일 읽기와 에러 처리
    3. 환경 변수 활용
    4. TDD로 개발하기
    """

    def construct(self):
        self.camera.background_color = DARK_BG

        self.section_intro()
        self.wait(0.5)
        self.clear()

        self.section_project_overview()
        self.wait(0.5)
        self.clear()

        self.section_args_parsing()
        self.wait(0.5)
        self.clear()

        self.section_file_handling()
        self.wait(0.5)
        self.clear()

        self.section_search_logic()
        self.wait(0.5)
        self.clear()

        self.section_env_variables()
        self.wait(0.5)
        self.clear()

        self.section_refactoring()
        self.wait(0.5)
        self.clear()

        self.section_outro()

    def section_intro(self):
        title = Text("EP 9-01", font_size=32, color=GRAY).to_edge(UP)
        main_title = Text("CLI App: minigrep", font_size=56, color=RUST_ORANGE)
        subtitle = Text("커맨드 라인 도구 만들기", font_size=28, color=GRAY)

        main_title.next_to(title, DOWN, buff=0.8)
        subtitle.next_to(main_title, DOWN, buff=0.3)

        self.play(Write(title), Write(main_title))
        self.play(FadeIn(subtitle))
        self.wait(1)

        # 목표
        goal = VGroup(
            Text("만들 것:", font_size=24, color=CLI_CYAN),
            Code(
                code='''$ minigrep rust poem.txt
Are you safe, fast, and productive?
Rust says yes to all three!''',
                language="bash",
                font_size=16,
            ).scale(0.8),
            Text("grep처럼 파일에서 패턴 검색!", font_size=18, color=GRAY),
        ).arrange(DOWN, buff=0.3)
        goal.next_to(subtitle, DOWN, buff=0.8)

        self.play(FadeIn(goal, shift=UP))
        self.wait(2)

    def section_project_overview(self):
        title = Text("프로젝트 구조", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        structure = Code(
            code='''minigrep/
├── Cargo.toml
├── src/
│   ├── main.rs    # 진입점, 인자 처리
│   └── lib.rs     # 핵심 로직
└── tests/
    └── integration_test.rs''',
            language="text",
            font_size=16,
        ).scale(0.8)
        structure.next_to(title, DOWN, buff=0.3).to_edge(LEFT, buff=1)

        self.play(Create(structure))
        self.wait(0.5)

        # 관심사 분리
        separation = VGroup(
            Text("관심사 분리:", font_size=20, color=SUCCESS_GREEN),
            VGroup(
                Text("main.rs → CLI 인터페이스", font_size=16, color=GRAY),
                Text("lib.rs → 비즈니스 로직", font_size=16, color=GRAY),
                Text("테스트 가능한 구조!", font_size=16, color=WHITE),
            ).arrange(DOWN, aligned_edge=LEFT, buff=0.1),
        ).arrange(DOWN, buff=0.2)
        separation.to_edge(RIGHT, buff=1)

        self.play(FadeIn(separation, shift=LEFT))
        self.wait(2)

    def section_args_parsing(self):
        title = Text("커맨드 라인 인자 처리", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''// src/main.rs
use std::env;
use std::process;

use minigrep::Config;

fn main() {
    let args: Vec<String> = env::args().collect();

    let config = Config::build(&args).unwrap_or_else(|err| {
        eprintln!("인자 파싱 에러: {err}");
        process::exit(1);
    });

    if let Err(e) = minigrep::run(config) {
        eprintln!("실행 에러: {e}");
        process::exit(1);
    }
}

// src/lib.rs
pub struct Config {
    pub query: String,
    pub file_path: String,
}

impl Config {
    pub fn build(args: &[String]) -> Result<Config, &'static str> {
        if args.len() < 3 {
            return Err("인자가 부족합니다");
        }
        Ok(Config {
            query: args[1].clone(),
            file_path: args[2].clone(),
        })
    }
}''',
            language="rust",
            font_size=10,
            background="rectangle",
            background_stroke_color=CLI_CYAN,
        ).scale(0.7)
        code.next_to(title, DOWN, buff=0.2)

        self.play(Create(code))
        self.wait(2)

    def section_file_handling(self):
        title = Text("파일 읽기와 에러 처리", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''use std::fs;
use std::error::Error;

pub fn run(config: Config) -> Result<(), Box<dyn Error>> {
    // ? 연산자로 에러 전파
    let contents = fs::read_to_string(&config.file_path)?;

    for line in search(&config.query, &contents) {
        println!("{line}");
    }

    Ok(())
}

// Box<dyn Error> = 모든 에러 타입을 담을 수 있는 트레이트 객체''',
            language="rust",
            font_size=14,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.75)
        code.next_to(title, DOWN, buff=0.3)

        self.play(Create(code))
        self.wait(1)

        # 에러 처리 흐름
        flow = VGroup(
            Text("에러 처리 흐름:", font_size=18, color=WARNING_YELLOW),
            Text("fs::read_to_string? → 실패시 조기 반환", font_size=14, color=GRAY),
            Text("main에서 eprintln!으로 사용자에게 표시", font_size=14, color=GRAY),
        ).arrange(DOWN, buff=0.1)
        flow.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(flow, shift=UP))
        self.wait(2)

    def section_search_logic(self):
        title = Text("검색 로직 (TDD)", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''// 테스트 먼저 작성!
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_result() {
        let query = "duct";
        let contents = "\\
Rust:
safe, fast, productive.
Pick three.";

        assert_eq!(vec!["safe, fast, productive."], search(query, contents));
    }
}

// 테스트를 통과하는 구현
pub fn search<'a>(query: &str, contents: &'a str) -> Vec<&'a str> {
    contents
        .lines()
        .filter(|line| line.contains(query))
        .collect()
}''',
            language="rust",
            font_size=12,
            background="rectangle",
            background_stroke_color=SUCCESS_GREEN,
        ).scale(0.7)
        code.next_to(title, DOWN, buff=0.3)

        self.play(Create(code))
        self.wait(1)

        # TDD 사이클
        tdd = VGroup(
            Text("TDD 사이클:", font_size=18, color=SUCCESS_GREEN),
            Text("1. Red - 실패하는 테스트 작성", font_size=14, color=ERROR_RED),
            Text("2. Green - 테스트 통과하는 최소 구현", font_size=14, color=SUCCESS_GREEN),
            Text("3. Refactor - 코드 개선", font_size=14, color=MEMORY_BLUE),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.1)
        tdd.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(tdd, shift=UP))
        self.wait(2)

    def section_env_variables(self):
        title = Text("환경 변수로 기능 추가", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''// 대소문자 구분 없는 검색 옵션
pub struct Config {
    pub query: String,
    pub file_path: String,
    pub ignore_case: bool,
}

impl Config {
    pub fn build(args: &[String]) -> Result<Config, &'static str> {
        // 환경 변수 확인
        let ignore_case = env::var("IGNORE_CASE").is_ok();

        Ok(Config {
            query: args[1].clone(),
            file_path: args[2].clone(),
            ignore_case,
        })
    }
}

// 사용
// $ IGNORE_CASE=1 minigrep rust poem.txt''',
            language="rust",
            font_size=13,
            background="rectangle",
            background_stroke_color=CLI_CYAN,
        ).scale(0.7)
        code.next_to(title, DOWN, buff=0.3)

        self.play(Create(code))
        self.wait(2)

    def section_refactoring(self):
        title = Text("최종 리팩터링", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # 개선 포인트
        improvements = VGroup(
            self.create_improvement("clap 크레이트", "전문적인 CLI 인자 파싱"),
            self.create_improvement("색상 출력", "colored 크레이트로 하이라이팅"),
            self.create_improvement("병렬 검색", "rayon으로 대용량 파일 처리"),
            self.create_improvement("정규식 지원", "regex 크레이트 통합"),
        ).arrange(DOWN, buff=0.3, aligned_edge=LEFT)
        improvements.next_to(title, DOWN, buff=0.5)

        for imp in improvements:
            self.play(FadeIn(imp, shift=RIGHT), run_time=0.4)

        self.wait(1)

        # 개선된 Cargo.toml
        cargo = Code(
            code='''[dependencies]
clap = { version = "4", features = ["derive"] }
colored = "2"
rayon = "1.8"
regex = "1"''',
            language="toml",
            font_size=14,
        ).scale(0.8)
        cargo.to_edge(DOWN, buff=0.5)

        self.play(Create(cargo))
        self.wait(2)

    def create_improvement(self, name, desc):
        return VGroup(
            Text("✓", font_size=20, color=SUCCESS_GREEN),
            Text(name, font_size=18, color=CLI_CYAN),
            Text(f"- {desc}", font_size=14, color=GRAY),
        ).arrange(RIGHT, buff=0.3)

    def section_outro(self):
        title = Text("EP 9-01 완료!", font_size=44, color=RUST_ORANGE)

        summary = VGroup(
            Text("✓ CLI 인자 처리 (env::args)", font_size=24, color=CLI_CYAN),
            Text("✓ 파일 I/O (fs::read_to_string)", font_size=24, color=SUCCESS_GREEN),
            Text("✓ 에러 처리 (Result, ?)", font_size=24, color=WARNING_YELLOW),
            Text("✓ TDD 개발 방법론", font_size=24, color=MEMORY_BLUE),
        ).arrange(DOWN, buff=0.4)

        VGroup(title, summary).arrange(DOWN, buff=0.8)

        self.play(Write(title))
        for line in summary:
            self.play(FadeIn(line, shift=UP), run_time=0.4)

        self.wait(1)

        next_ep = Text("다음: EP 9-02 웹 API 구축", font_size=28, color=GRAY)
        next_ep.to_edge(DOWN, buff=1)
        self.play(FadeIn(next_ep))
        self.wait(2)


class EP_9_02_WebAPI(Scene):
    """
    EP 9-02: 웹 API - Axum으로 REST API 구축

    학습 목표:
    1. Axum 웹 프레임워크 사용
    2. REST API 엔드포인트 구현
    3. 비동기 핸들러 작성
    4. JSON 직렬화/역직렬화

    ⚡ 책에서 다루는 멀티스레드 서버보다 더 현대적!
    """

    def construct(self):
        self.camera.background_color = DARK_BG

        self.section_intro()
        self.wait(0.5)
        self.clear()

        self.section_axum_basics()
        self.wait(0.5)
        self.clear()

        self.section_handlers()
        self.wait(0.5)
        self.clear()

        self.section_json_serde()
        self.wait(0.5)
        self.clear()

        self.section_state_management()
        self.wait(0.5)
        self.clear()

        self.section_full_api()
        self.wait(0.5)
        self.clear()

        self.section_outro()

    def section_intro(self):
        title = Text("EP 9-02", font_size=32, color=GRAY).to_edge(UP)
        main_title = Text("Web API with Axum", font_size=56, color=RUST_ORANGE)
        subtitle = Text("현대적인 REST API 구축", font_size=28, color=GRAY)

        main_title.next_to(title, DOWN, buff=0.8)
        subtitle.next_to(main_title, DOWN, buff=0.3)

        self.play(Write(title), Write(main_title))
        self.play(FadeIn(subtitle))
        self.wait(1)

        # vs 책
        comparison = VGroup(
            Text("책: std::net으로 직접 구현 (교육용)", font_size=18, color=GRAY),
            Text("우리: Axum 프레임워크 (실무용)", font_size=18, color=WEB_ORANGE),
            Text("→ 진짜 프로덕션에서 쓰는 방식!", font_size=16, color=SUCCESS_GREEN),
        ).arrange(DOWN, buff=0.2)
        comparison.next_to(subtitle, DOWN, buff=0.8)

        self.play(FadeIn(comparison, shift=UP))
        self.wait(2)

    def section_axum_basics(self):
        title = Text("Axum 시작하기", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''// Cargo.toml
[dependencies]
axum = "0.7"
tokio = { version = "1", features = ["full"] }
serde = { version = "1", features = ["derive"] }
serde_json = "1"

// src/main.rs
use axum::{routing::get, Router};

#[tokio::main]
async fn main() {
    let app = Router::new()
        .route("/", get(|| async { "Hello, World!" }))
        .route("/health", get(health_check));

    let listener = tokio::net::TcpListener::bind("0.0.0.0:3000")
        .await
        .unwrap();

    println!("🚀 Server running on http://localhost:3000");
    axum::serve(listener, app).await.unwrap();
}

async fn health_check() -> &'static str {
    "OK"
}''',
            language="rust",
            font_size=11,
            background="rectangle",
            background_stroke_color=WEB_ORANGE,
        ).scale(0.68)
        code.next_to(title, DOWN, buff=0.2)

        self.play(Create(code))
        self.wait(2)

    def section_handlers(self):
        title = Text("핸들러 작성하기", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''use axum::{
    extract::{Path, Query},
    http::StatusCode,
    response::Json,
};
use serde::{Deserialize, Serialize};

// 경로 매개변수
// GET /users/123
async fn get_user(Path(user_id): Path<u64>) -> String {
    format!("User ID: {}", user_id)
}

// 쿼리 매개변수
// GET /search?q=rust&limit=10
#[derive(Deserialize)]
struct SearchQuery {
    q: String,
    limit: Option<usize>,
}

async fn search(Query(params): Query<SearchQuery>) -> String {
    format!("Searching: {} (limit: {:?})", params.q, params.limit)
}

// JSON 응답
#[derive(Serialize)]
struct User { id: u64, name: String }

async fn get_user_json(Path(id): Path<u64>) -> Json<User> {
    Json(User { id, name: "Alice".into() })
}''',
            language="rust",
            font_size=10,
            background="rectangle",
            background_stroke_color=WEB_ORANGE,
        ).scale(0.68)
        code.next_to(title, DOWN, buff=0.2)

        self.play(Create(code))
        self.wait(2)

    def section_json_serde(self):
        title = Text("JSON 처리 (Serde)", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''use axum::Json;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
struct CreateUser {
    name: String,
    email: String,
}

#[derive(Serialize)]
struct UserResponse {
    id: u64,
    name: String,
    email: String,
    created_at: String,
}

// POST /users
// Body: {"name": "Bob", "email": "bob@example.com"}
async fn create_user(Json(payload): Json<CreateUser>) -> Json<UserResponse> {
    Json(UserResponse {
        id: 1,  // 실제로는 DB에서 생성
        name: payload.name,
        email: payload.email,
        created_at: "2024-01-01".into(),
    })
}

// 라우터에 등록
let app = Router::new()
    .route("/users", post(create_user));''',
            language="rust",
            font_size=11,
            background="rectangle",
            background_stroke_color=WEB_ORANGE,
        ).scale(0.68)
        code.next_to(title, DOWN, buff=0.2)

        self.play(Create(code))
        self.wait(2)

    def section_state_management(self):
        title = Text("상태 관리 (State)", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''use std::sync::Arc;
use tokio::sync::RwLock;
use axum::extract::State;

// 공유 상태 정의
type SharedState = Arc<RwLock<AppState>>;

struct AppState {
    users: Vec<User>,
}

// 핸들러에서 상태 접근
async fn list_users(State(state): State<SharedState>) -> Json<Vec<User>> {
    let state = state.read().await;
    Json(state.users.clone())
}

async fn add_user(
    State(state): State<SharedState>,
    Json(new_user): Json<CreateUser>,
) -> StatusCode {
    let mut state = state.write().await;
    state.users.push(User { /* ... */ });
    StatusCode::CREATED
}

// 라우터에 상태 주입
let shared_state = Arc::new(RwLock::new(AppState { users: vec![] }));
let app = Router::new()
    .route("/users", get(list_users).post(add_user))
    .with_state(shared_state);''',
            language="rust",
            font_size=10,
            background="rectangle",
            background_stroke_color=WEB_ORANGE,
        ).scale(0.65)
        code.next_to(title, DOWN, buff=0.2)

        self.play(Create(code))
        self.wait(2)

    def section_full_api(self):
        title = Text("완성된 REST API", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # API 엔드포인트 목록
        endpoints = VGroup(
            self.create_endpoint("GET", "/users", "모든 사용자 조회"),
            self.create_endpoint("GET", "/users/:id", "특정 사용자 조회"),
            self.create_endpoint("POST", "/users", "사용자 생성"),
            self.create_endpoint("PUT", "/users/:id", "사용자 수정"),
            self.create_endpoint("DELETE", "/users/:id", "사용자 삭제"),
        ).arrange(DOWN, buff=0.25, aligned_edge=LEFT)
        endpoints.next_to(title, DOWN, buff=0.4).to_edge(LEFT, buff=1)

        for ep in endpoints:
            self.play(FadeIn(ep, shift=RIGHT), run_time=0.3)

        self.wait(0.5)

        # 라우터 코드
        router = Code(
            code='''let app = Router::new()
    .route("/users", get(list_users)
                     .post(create_user))
    .route("/users/:id", get(get_user)
                         .put(update_user)
                         .delete(delete_user))
    .with_state(shared_state);''',
            language="rust",
            font_size=12,
            background="rectangle",
            background_stroke_color=SUCCESS_GREEN,
        ).scale(0.75)
        router.to_edge(RIGHT, buff=0.5)

        self.play(Create(router))
        self.wait(2)

    def create_endpoint(self, method, path, desc):
        method_colors = {
            "GET": SUCCESS_GREEN,
            "POST": MEMORY_BLUE,
            "PUT": WARNING_YELLOW,
            "DELETE": ERROR_RED,
        }
        return VGroup(
            Text(method, font_size=16, color=method_colors.get(method, WHITE)),
            Text(path, font_size=14, color=WHITE),
            Text(f"- {desc}", font_size=12, color=GRAY),
        ).arrange(RIGHT, buff=0.3)

    def section_outro(self):
        title = Text("EP 9-02 완료!", font_size=44, color=RUST_ORANGE)

        summary = VGroup(
            Text("✓ Axum 웹 프레임워크", font_size=24, color=WEB_ORANGE),
            Text("✓ 비동기 핸들러 (async/await)", font_size=24, color=MEMORY_BLUE),
            Text("✓ Serde로 JSON 직렬화", font_size=24, color=SUCCESS_GREEN),
            Text("✓ Arc<RwLock>으로 상태 관리", font_size=24, color=HEAP_PURPLE),
        ).arrange(DOWN, buff=0.4)

        VGroup(title, summary).arrange(DOWN, buff=0.8)

        self.play(Write(title))
        for line in summary:
            self.play(FadeIn(line, shift=UP), run_time=0.4)

        self.wait(1)

        next_ep = Text("다음: EP 9-03 시스템 도구", font_size=28, color=GRAY)
        next_ep.to_edge(DOWN, buff=1)
        self.play(FadeIn(next_ep))
        self.wait(2)


class EP_9_03_SystemTool(Scene):
    """
    EP 9-03: 시스템 도구 - 파일 감시자 만들기

    학습 목표:
    1. 파일 시스템 이벤트 감시
    2. 크로스 플랫폼 개발
    3. 시그널 처리
    4. 실시간 로깅

    🔧 시스템 프로그래밍의 정수!
    """

    def construct(self):
        self.camera.background_color = DARK_BG

        self.section_intro()
        self.wait(0.5)
        self.clear()

        self.section_notify_crate()
        self.wait(0.5)
        self.clear()

        self.section_event_handling()
        self.wait(0.5)
        self.clear()

        self.section_signal_handling()
        self.wait(0.5)
        self.clear()

        self.section_full_watcher()
        self.wait(0.5)
        self.clear()

        self.section_series_finale()

    def section_intro(self):
        title = Text("EP 9-03", font_size=32, color=GRAY).to_edge(UP)
        main_title = Text("System Tool", font_size=56, color=RUST_ORANGE)
        subtitle = Text("파일 감시자 만들기", font_size=28, color=GRAY)

        main_title.next_to(title, DOWN, buff=0.8)
        subtitle.next_to(main_title, DOWN, buff=0.3)

        self.play(Write(title), Write(main_title))
        self.play(FadeIn(subtitle))
        self.wait(1)

        # 목표
        goal = VGroup(
            Text("만들 것:", font_size=24, color=SYSTEM_TEAL),
            Code(
                code='''$ file-watcher ./src
[2024-01-01 12:00:00] Watching ./src
[2024-01-01 12:00:05] MODIFY: ./src/main.rs
[2024-01-01 12:00:10] CREATE: ./src/utils.rs
[2024-01-01 12:00:15] DELETE: ./src/old.rs
^C Gracefully shutting down...''',
                language="text",
                font_size=14,
            ).scale(0.8),
        ).arrange(DOWN, buff=0.3)
        goal.next_to(subtitle, DOWN, buff=0.8)

        self.play(FadeIn(goal, shift=UP))
        self.wait(2)

    def section_notify_crate(self):
        title = Text("notify 크레이트", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''// Cargo.toml
[dependencies]
notify = "6"
tokio = { version = "1", features = ["full", "signal"] }
chrono = "0.4"
colored = "2"

// 기본 구조
use notify::{Config, RecommendedWatcher, RecursiveMode, Watcher};
use std::sync::mpsc::channel;
use std::path::Path;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (tx, rx) = channel();

    // 파일 시스템 감시자 생성
    let mut watcher = RecommendedWatcher::new(tx, Config::default())?;

    // 경로 감시 시작
    watcher.watch(Path::new("./src"), RecursiveMode::Recursive)?;

    // 이벤트 수신
    for event in rx {
        println!("{:?}", event);
    }

    Ok(())
}''',
            language="rust",
            font_size=11,
            background="rectangle",
            background_stroke_color=SYSTEM_TEAL,
        ).scale(0.68)
        code.next_to(title, DOWN, buff=0.2)

        self.play(Create(code))
        self.wait(2)

    def section_event_handling(self):
        title = Text("이벤트 처리하기", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''use notify::event::{CreateKind, ModifyKind, RemoveKind};
use notify::EventKind;
use chrono::Local;
use colored::*;

fn handle_event(event: notify::Event) {
    let timestamp = Local::now().format("%Y-%m-%d %H:%M:%S");

    let (action, color) = match event.kind {
        EventKind::Create(CreateKind::File) => ("CREATE", "green"),
        EventKind::Modify(ModifyKind::Data(_)) => ("MODIFY", "yellow"),
        EventKind::Remove(RemoveKind::File) => ("DELETE", "red"),
        EventKind::Access(_) => ("ACCESS", "blue"),
        _ => return,  // 기타 이벤트 무시
    };

    for path in event.paths {
        println!(
            "[{}] {}: {}",
            timestamp,
            action.color(color),
            path.display()
        );
    }
}''',
            language="rust",
            font_size=12,
            background="rectangle",
            background_stroke_color=SYSTEM_TEAL,
        ).scale(0.7)
        code.next_to(title, DOWN, buff=0.3)

        self.play(Create(code))
        self.wait(2)

    def section_signal_handling(self):
        title = Text("우아한 종료 (Graceful Shutdown)", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''use tokio::signal;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 감시 시작
    let (tx, rx) = channel();
    let mut watcher = RecommendedWatcher::new(tx, Config::default())?;
    watcher.watch(Path::new("./src"), RecursiveMode::Recursive)?;

    println!("🔍 Watching ./src (Press Ctrl+C to stop)");

    // 이벤트 처리 태스크
    let event_task = tokio::spawn(async move {
        for event in rx {
            if let Ok(e) = event {
                handle_event(e);
            }
        }
    });

    // Ctrl+C 시그널 대기
    signal::ctrl_c().await?;
    println!("\\n👋 Gracefully shutting down...");

    // 정리
    event_task.abort();
    Ok(())
}''',
            language="rust",
            font_size=11,
            background="rectangle",
            background_stroke_color=SYSTEM_TEAL,
        ).scale(0.68)
        code.next_to(title, DOWN, buff=0.2)

        self.play(Create(code))
        self.wait(2)

    def section_full_watcher(self):
        title = Text("확장 아이디어", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        ideas = VGroup(
            self.create_idea("자동 빌드", "cargo build 트리거", "🔨"),
            self.create_idea("테스트 실행", "파일 변경 시 cargo test", "🧪"),
            self.create_idea("핫 리로드", "서버 자동 재시작", "🔥"),
            self.create_idea("동기화", "원격 서버로 rsync", "☁️"),
            self.create_idea("알림", "데스크톱 노티피케이션", "🔔"),
        ).arrange(DOWN, buff=0.3, aligned_edge=LEFT)
        ideas.next_to(title, DOWN, buff=0.5)

        for idea in ideas:
            self.play(FadeIn(idea, shift=RIGHT), run_time=0.35)

        self.wait(1)

        # 실제 도구 언급
        real_tools = VGroup(
            Text("실제 Rust로 만든 도구들:", font_size=18, color=SYSTEM_TEAL),
            Text("• watchexec - 범용 파일 감시자", font_size=14, color=GRAY),
            Text("• cargo-watch - Rust 프로젝트 전용", font_size=14, color=GRAY),
            Text("• bacon - 터미널 UI로 빌드/테스트", font_size=14, color=GRAY),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.1)
        real_tools.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(real_tools, shift=UP))
        self.wait(2)

    def create_idea(self, name, desc, emoji):
        return VGroup(
            Text(emoji, font_size=24),
            Text(name, font_size=18, color=SYSTEM_TEAL),
            Text(f"- {desc}", font_size=14, color=GRAY),
        ).arrange(RIGHT, buff=0.3)

    def section_series_finale(self):
        # EP 9-03 정리
        title = Text("EP 9-03 완료!", font_size=44, color=RUST_ORANGE)

        summary = VGroup(
            Text("✓ 파일 시스템 이벤트 감시", font_size=24, color=SYSTEM_TEAL),
            Text("✓ 크로스 플랫폼 (notify)", font_size=24, color=SUCCESS_GREEN),
            Text("✓ 시그널 처리 (Ctrl+C)", font_size=24, color=WARNING_YELLOW),
            Text("✓ 비동기 이벤트 루프", font_size=24, color=MEMORY_BLUE),
        ).arrange(DOWN, buff=0.4)

        VGroup(title, summary).arrange(DOWN, buff=0.8)

        self.play(Write(title))
        for line in summary:
            self.play(FadeIn(line, shift=UP), run_time=0.4)

        self.wait(2)
        self.clear()

        # ═══════════════════════════════════════════════════
        # 🎉 전체 시리즈 완료!!!
        # ═══════════════════════════════════════════════════

        congrats = Text("🎉 축하합니다! 🎉", font_size=64, color=RUST_ORANGE)
        self.play(FadeIn(congrats, scale=1.5))
        self.wait(1)
        self.play(congrats.animate.to_edge(UP, buff=0.5))

        complete = Text("WIA-RUST-LEARN 전체 과정 수료!", font_size=36, color=WHITE)
        complete.next_to(congrats, DOWN, buff=0.5)
        self.play(Write(complete))
        self.wait(0.5)

        # 전체 통계
        stats = VGroup(
            Text("📊 당신이 완료한 것:", font_size=28, color=GRAY),
            VGroup(
                Text("28편의 에피소드", font_size=20, color=WHITE),
                Text("9개의 레벨", font_size=20, color=WHITE),
                Text("10,000+ 줄의 Manim 코드", font_size=20, color=WHITE),
                Text("3개의 실전 프로젝트", font_size=20, color=WHITE),
            ).arrange(DOWN, buff=0.15),
        ).arrange(DOWN, buff=0.3)
        stats.next_to(complete, DOWN, buff=0.5)

        self.play(FadeIn(stats, shift=UP))
        self.wait(1)

        # 인증 등급
        cert = VGroup(
            Text("🏆 인증 등급:", font_size=24, color=WARNING_YELLOW),
            VGroup(
                Text("Core (Level 0-5) ✅", font_size=16, color=SUCCESS_GREEN),
                Text("Expert (Level 6-8) ✅", font_size=16, color=MEMORY_BLUE),
                Text("Master (Level 9) ✅", font_size=16, color=HEAP_PURPLE),
            ).arrange(RIGHT, buff=0.5),
        ).arrange(DOWN, buff=0.2)
        cert.next_to(stats, DOWN, buff=0.3)

        self.play(FadeIn(cert, shift=UP))
        self.wait(2)

        self.clear()

        # 최종 메시지
        final_message = VGroup(
            Text("\"Rust는 어렵지 않다.\"", font_size=40, color=RUST_ORANGE),
            Text("\"다를 뿐이다.\"", font_size=40, color=RUST_ORANGE),
            Text("", font_size=20),
            Text("NRT: Non-Repeatable Transfer", font_size=24, color=WARNING_YELLOW),
            Text("소유권은 NFT처럼 유일하고 이전 가능!", font_size=18, color=GRAY),
        ).arrange(DOWN, buff=0.3)

        self.play(FadeIn(final_message, scale=0.8))
        self.wait(2)

        self.clear()

        # 엔딩 크레딧
        credits = VGroup(
            Text("WIA-RUST-LEARN", font_size=56, color=RUST_ORANGE),
            Text("28편 시리즈 완료", font_size=28, color=WHITE),
            Text("", font_size=20),
            Text("WIA", font_size=24, color=GRAY),
            Text("World Certification Industry Association", font_size=16, color=GRAY),
            Text("", font_size=20),
            Text("弘益人間 (홍익인간)", font_size=32, color=SUCCESS_GREEN),
            Text("널리 인간을 이롭게 하라", font_size=20, color=GRAY),
            Text("", font_size=20),
            Text("© 2025 SmileStory Inc.", font_size=14, color=GRAY),
        ).arrange(DOWN, buff=0.25)

        self.play(FadeIn(credits, shift=UP, run_time=2))
        self.wait(3)


# 렌더링 헬퍼
if __name__ == "__main__":
    print("""
╔═══════════════════════════════════════════════════════════════╗
║          WIA-RUST-LEARN Level 9: 실전 프로젝트                ║
╠═══════════════════════════════════════════════════════════════╣
║                                                               ║
║  EP 9-01: CLI App - minigrep                                  ║
║    manim -pql level_9_projects.py EP_9_01_CLI                 ║
║                                                               ║
║  EP 9-02: Web API - Axum                                      ║
║    manim -pql level_9_projects.py EP_9_02_WebAPI              ║
║                                                               ║
║  EP 9-03: System Tool - File Watcher                          ║
║    manim -pql level_9_projects.py EP_9_03_SystemTool          ║
║                                                               ║
║  전체 렌더링:                                                  ║
║    manim -pqh level_9_projects.py --all                       ║
║                                                               ║
╠═══════════════════════════════════════════════════════════════╣
║                                                               ║
║  🎓 이 레벨을 완료하면 WIA-RUST-LEARN Master!                 ║
║                                                               ║
║  Level 0-5: Core Certificate                                  ║
║  Level 6-8: Expert Certificate                                ║
║  Level 9:   Master Certificate                                ║
║                                                               ║
║  弘益人間 - 널리 인간을 이롭게 하라                           ║
║                                                               ║
╚═══════════════════════════════════════════════════════════════╝
    """)
