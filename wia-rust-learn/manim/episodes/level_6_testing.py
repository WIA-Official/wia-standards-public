"""
WIA-RUST-LEARN Level 6: 테스팅 & 도구 (Testing & Tools)
=======================================================

EP 6-01: Testing - 자동화 테스트 작성하기 (20분)
EP 6-02: Cargo Advanced - 고급 Cargo & crates.io (18분)

실행 방법:
    manim -pql level_6_testing.py EP_6_01_Testing
    manim -pql level_6_testing.py EP_6_02_CargoAdvanced

전체 렌더링 (고화질):
    manim -pqh level_6_testing.py --all
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
TEST_CYAN = "#00BCD4"
CARGO_BROWN = "#8D6E63"
DARK_BG = "#1a1a2e"


class EP_6_01_Testing(Scene):
    """
    EP 6-01: Testing - 자동화 테스트 작성하기

    학습 목표:
    1. #[test] 속성으로 테스트 함수 작성
    2. assert! 매크로 시리즈 활용
    3. 유닛 테스트 vs 통합 테스트
    4. 테스트 조직화와 실행
    """

    def construct(self):
        self.camera.background_color = DARK_BG

        self.section_intro()
        self.wait(0.5)
        self.clear()

        self.section_test_basics()
        self.wait(0.5)
        self.clear()

        self.section_assert_macros()
        self.wait(0.5)
        self.clear()

        self.section_test_organization()
        self.wait(0.5)
        self.clear()

        self.section_test_attributes()
        self.wait(0.5)
        self.clear()

        self.section_integration_tests()
        self.wait(0.5)
        self.clear()

        self.section_outro()

    def section_intro(self):
        title = Text("EP 6-01", font_size=32, color=GRAY).to_edge(UP)
        main_title = Text("Testing", font_size=56, color=RUST_ORANGE)
        subtitle = Text("자동화 테스트 작성하기", font_size=28, color=GRAY)

        main_title.next_to(title, DOWN, buff=0.8)
        subtitle.next_to(main_title, DOWN, buff=0.3)

        self.play(Write(title), Write(main_title))
        self.play(FadeIn(subtitle))
        self.wait(1)

        # 왜 테스트?
        why_test = VGroup(
            Text("왜 테스트를 작성하는가?", font_size=28, color=TEST_CYAN),
            VGroup(
                Text("• 버그를 조기에 발견", font_size=20, color=GRAY),
                Text("• 리팩터링에 자신감", font_size=20, color=GRAY),
                Text("• 문서화 역할", font_size=20, color=GRAY),
                Text("• 설계 개선 유도", font_size=20, color=GRAY),
            ).arrange(DOWN, aligned_edge=LEFT, buff=0.15),
        ).arrange(DOWN, buff=0.4)
        why_test.next_to(subtitle, DOWN, buff=0.8)

        self.play(FadeIn(why_test, shift=UP))
        self.wait(1)

        # Rust 테스트의 특징
        rust_test = VGroup(
            Text("Rust 테스트의 특징", font_size=24, color=RUST_ORANGE),
            Text("• 언어에 내장된 테스트 프레임워크", font_size=18, color=WHITE),
            Text("• cargo test 한 줄로 실행", font_size=18, color=WHITE),
        ).arrange(DOWN, buff=0.2)
        rust_test.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(rust_test, shift=UP))
        self.wait(2)

    def section_test_basics(self):
        title = Text("테스트 기초", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''// src/lib.rs
pub fn add(a: i32, b: i32) -> i32 {
    a + b
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_add() {
        let result = add(2, 3);
        assert_eq!(result, 5);
    }

    #[test]
    fn test_add_negative() {
        assert_eq!(add(-1, 1), 0);
    }
}''',
            language="rust",
            font_size=14,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.75)
        code.next_to(title, DOWN, buff=0.3).to_edge(LEFT, buff=0.5)

        self.play(Create(code))
        self.wait(1)

        # 핵심 요소 설명
        elements = VGroup(
            self.create_element_box("#[cfg(test)]", "테스트 빌드에서만 컴파일", TEST_CYAN),
            self.create_element_box("#[test]", "이 함수가 테스트임을 표시", SUCCESS_GREEN),
            self.create_element_box("assert_eq!", "같은지 검사", WARNING_YELLOW),
        ).arrange(DOWN, buff=0.3)
        elements.to_edge(RIGHT, buff=0.5)

        for elem in elements:
            self.play(FadeIn(elem, shift=LEFT), run_time=0.4)

        self.wait(1)

        # 실행 명령
        cmd = Code(
            code="$ cargo test",
            language="bash",
            font_size=20,
            background="rectangle",
            background_stroke_color=SUCCESS_GREEN,
        ).scale(0.8)
        cmd.to_edge(DOWN, buff=0.5)

        self.play(Create(cmd))
        self.wait(2)

    def create_element_box(self, code_text, desc, color):
        box = RoundedRectangle(
            width=4, height=0.9, corner_radius=0.1,
            fill_color=color, fill_opacity=0.1,
            stroke_color=color
        )
        content = VGroup(
            Text(code_text, font_size=16, color=color),
            Text(desc, font_size=12, color=GRAY),
        ).arrange(DOWN, buff=0.1)
        content.move_to(box)
        return VGroup(box, content)

    def section_assert_macros(self):
        title = Text("assert! 매크로 시리즈", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        macros = VGroup(
            self.create_macro_row("assert!(expr)", "expr이 true인지 확인", "assert!(result > 0);"),
            self.create_macro_row("assert_eq!(a, b)", "a == b 인지 확인", "assert_eq!(add(2,2), 4);"),
            self.create_macro_row("assert_ne!(a, b)", "a != b 인지 확인", "assert_ne!(result, 0);"),
        ).arrange(DOWN, buff=0.4)
        macros.next_to(title, DOWN, buff=0.5)

        for macro in macros:
            self.play(FadeIn(macro, shift=UP), run_time=0.5)

        self.wait(1)

        # 커스텀 메시지
        custom = Code(
            code='''#[test]
fn test_with_message() {
    let value = 42;
    assert!(
        value > 100,
        "값 {}은(는) 100보다 커야 합니다!", value
    );
}

// 실패 시 출력:
// assertion failed: 값 42은(는) 100보다 커야 합니다!''',
            language="rust",
            font_size=14,
            background="rectangle",
        ).scale(0.75)
        custom.to_edge(DOWN, buff=0.5)

        self.play(Create(custom))
        self.wait(2)

    def create_macro_row(self, name, desc, example):
        return VGroup(
            Text(name, font_size=20, color=TEST_CYAN),
            Text(desc, font_size=14, color=GRAY),
            Text(example, font_size=12, color=WHITE),
        ).arrange(DOWN, buff=0.1)

    def section_test_organization(self):
        title = Text("유닛 테스트 구조", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # 파일 구조
        structure = Code(
            code='''src/
├── lib.rs       # 라이브러리 코드 + 유닛 테스트
├── main.rs      # 바이너리 진입점
└── utils.rs     # 모듈 + 해당 모듈 테스트

// src/utils.rs
pub fn helper() -> String {
    "helper".to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_helper() {
        assert_eq!(helper(), "helper");
    }
}''',
            language="rust",
            font_size=14,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.7)
        code.next_to(title, DOWN, buff=0.3)

        self.play(Create(structure))
        self.wait(1)

        # 장점
        benefit = VGroup(
            Text("✓ 비공개 함수도 테스트 가능!", font_size=20, color=SUCCESS_GREEN),
            Text("(같은 모듈 내에서는 private 접근 가능)", font_size=14, color=GRAY),
        ).arrange(DOWN, buff=0.1)
        benefit.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(benefit, shift=UP))
        self.wait(2)

    def section_test_attributes(self):
        title = Text("테스트 속성들", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''#[test]
#[ignore]  // 기본적으로 건너뜀
fn expensive_test() {
    // 시간이 오래 걸리는 테스트
}

#[test]
#[should_panic]  // panic!이 발생해야 통과
fn test_panic() {
    panic!("이 패닉이 발생해야 함!");
}

#[test]
#[should_panic(expected = "특정 메시지")]
fn test_specific_panic() {
    panic!("특정 메시지가 포함되어야 함");
}

#[test]
fn test_result() -> Result<(), String> {
    if 2 + 2 == 4 {
        Ok(())
    } else {
        Err("수학이 고장났습니다!".to_string())
    }
}''',
            language="rust",
            font_size=13,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.7)
        code.next_to(title, DOWN, buff=0.3)

        self.play(Create(code))
        self.wait(1)

        # 실행 옵션
        options = VGroup(
            Text("실행 옵션:", font_size=20, color=TEST_CYAN),
            Code(
                code='''cargo test                    # 모든 테스트
cargo test test_name          # 특정 테스트만
cargo test -- --ignored       # ignore된 테스트만
cargo test -- --test-threads=1  # 순차 실행''',
                language="bash",
                font_size=12,
            ).scale(0.8),
        ).arrange(DOWN, buff=0.2)
        options.to_edge(DOWN, buff=0.3)

        self.play(FadeIn(options, shift=UP))
        self.wait(2)

    def section_integration_tests(self):
        title = Text("통합 테스트", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # 구조
        structure = Code(
            code='''my_project/
├── Cargo.toml
├── src/
│   └── lib.rs        # 라이브러리 코드
└── tests/            # 통합 테스트 디렉토리
    ├── common/
    │   └── mod.rs    # 공유 헬퍼
    └── integration_test.rs''',
            language="text",
            font_size=16,
        ).scale(0.8)
        structure.next_to(title, DOWN, buff=0.3).to_edge(LEFT, buff=0.5)

        self.play(Create(structure))
        self.wait(0.5)

        # 통합 테스트 코드
        test_code = Code(
            code='''// tests/integration_test.rs
use my_project;  // 외부 크레이트처럼 사용

mod common;  // 공유 헬퍼 가져오기

#[test]
fn test_full_flow() {
    common::setup();

    let result = my_project::public_api();
    assert!(result.is_ok());
}''',
            language="rust",
            font_size=14,
            background="rectangle",
            background_stroke_color=TEST_CYAN,
        ).scale(0.75)
        test_code.to_edge(RIGHT, buff=0.5)

        self.play(Create(test_code))
        self.wait(1)

        # 차이점
        diff = VGroup(
            VGroup(
                Text("유닛 테스트", font_size=18, color=SUCCESS_GREEN),
                Text("• 같은 파일에 위치", font_size=12, color=GRAY),
                Text("• private 접근 가능", font_size=12, color=GRAY),
            ).arrange(DOWN, buff=0.1),
            VGroup(
                Text("통합 테스트", font_size=18, color=TEST_CYAN),
                Text("• tests/ 디렉토리", font_size=12, color=GRAY),
                Text("• public API만 테스트", font_size=12, color=GRAY),
            ).arrange(DOWN, buff=0.1),
        ).arrange(RIGHT, buff=1.5)
        diff.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(diff, shift=UP))
        self.wait(2)

    def section_outro(self):
        title = Text("EP 6-01 정리", font_size=44, color=RUST_ORANGE)

        summary = VGroup(
            Text("#[test] = 테스트 함수 표시", font_size=24, color=TEST_CYAN),
            Text("assert!/eq!/ne! = 검증 매크로", font_size=24, color=SUCCESS_GREEN),
            Text("#[cfg(test)] = 테스트 전용 코드", font_size=24, color=WARNING_YELLOW),
            Text("tests/ = 통합 테스트 디렉토리", font_size=24, color=MEMORY_BLUE),
        ).arrange(DOWN, buff=0.4)

        VGroup(title, summary).arrange(DOWN, buff=0.8)

        self.play(Write(title))
        for line in summary:
            self.play(FadeIn(line, shift=UP), run_time=0.4)

        self.wait(1)

        next_ep = Text("다음: EP 6-02 Cargo Advanced", font_size=28, color=GRAY)
        next_ep.to_edge(DOWN, buff=1)
        self.play(FadeIn(next_ep))
        self.wait(2)


class EP_6_02_CargoAdvanced(Scene):
    """
    EP 6-02: Cargo Advanced - 고급 Cargo & crates.io

    학습 목표:
    1. 릴리스 프로필 설정
    2. 문서화 주석 작성
    3. crates.io 배포
    4. Cargo 작업 공간
    """

    def construct(self):
        self.camera.background_color = DARK_BG

        self.section_intro()
        self.wait(0.5)
        self.clear()

        self.section_release_profiles()
        self.wait(0.5)
        self.clear()

        self.section_documentation()
        self.wait(0.5)
        self.clear()

        self.section_publishing()
        self.wait(0.5)
        self.clear()

        self.section_workspaces()
        self.wait(0.5)
        self.clear()

        self.section_outro()

    def section_intro(self):
        title = Text("EP 6-02", font_size=32, color=GRAY).to_edge(UP)
        main_title = Text("Cargo Advanced", font_size=56, color=RUST_ORANGE)
        subtitle = Text("프로 개발자의 Cargo 활용법", font_size=28, color=GRAY)

        main_title.next_to(title, DOWN, buff=0.8)
        subtitle.next_to(main_title, DOWN, buff=0.3)

        self.play(Write(title), Write(main_title))
        self.play(FadeIn(subtitle))
        self.wait(1)

        # Cargo의 힘
        powers = VGroup(
            Text("Cargo = 빌드 + 패키지 + 워크플로우", font_size=24, color=CARGO_BROWN),
            VGroup(
                Text("📦 의존성 관리", font_size=18, color=WHITE),
                Text("🔧 빌드 설정", font_size=18, color=WHITE),
                Text("📚 문서 생성", font_size=18, color=WHITE),
                Text("🚀 배포", font_size=18, color=WHITE),
            ).arrange(RIGHT, buff=0.8),
        ).arrange(DOWN, buff=0.5)
        powers.next_to(subtitle, DOWN, buff=0.8)

        self.play(FadeIn(powers, shift=UP))
        self.wait(2)

    def section_release_profiles(self):
        title = Text("릴리스 프로필", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # 프로필 비교
        profiles = VGroup(
            self.create_profile_box("dev", "개발용", ["최적화 없음 (opt-level=0)", "빠른 컴파일", "디버그 심볼 포함"], STACK_GREEN),
            self.create_profile_box("release", "배포용", ["최적화 최대 (opt-level=3)", "느린 컴파일", "작은 바이너리"], HEAP_PURPLE),
        ).arrange(RIGHT, buff=0.8)
        profiles.next_to(title, DOWN, buff=0.5)

        self.play(FadeIn(profiles, shift=UP))
        self.wait(1)

        # Cargo.toml 설정
        config = Code(
            code='''# Cargo.toml
[profile.dev]
opt-level = 0      # 최적화 레벨 (0-3)
debug = true       # 디버그 정보

[profile.release]
opt-level = 3
lto = true         # 링크 타임 최적화
codegen-units = 1  # 단일 코드젠 (더 느리지만 더 최적화)

# 커스텀 프로필
[profile.bench]
inherits = "release"
debug = true''',
            language="toml",
            font_size=14,
            background="rectangle",
            background_stroke_color=CARGO_BROWN,
        ).scale(0.7)
        config.to_edge(DOWN, buff=0.3)

        self.play(Create(config))
        self.wait(2)

    def create_profile_box(self, name, desc, features, color):
        box = RoundedRectangle(
            width=4.5, height=2.5, corner_radius=0.2,
            fill_color=color, fill_opacity=0.1,
            stroke_color=color
        )
        content = VGroup(
            Text(name, font_size=24, color=color),
            Text(desc, font_size=14, color=GRAY),
            VGroup(*[Text(f"• {f}", font_size=11, color=WHITE) for f in features]).arrange(DOWN, aligned_edge=LEFT, buff=0.05),
        ).arrange(DOWN, buff=0.2)
        content.move_to(box)
        return VGroup(box, content)

    def section_documentation(self):
        title = Text("문서화 주석", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''//! 크레이트 레벨 문서 (lib.rs 상단)
//!
//! # My Crate
//! 이 크레이트는 놀라운 기능을 제공합니다.

/// 두 숫자를 더합니다.
///
/// # Examples
///
/// ```
/// let result = my_crate::add(2, 3);
/// assert_eq!(result, 5);
/// ```
///
/// # Panics
/// 오버플로우 시 패닉합니다.
pub fn add(a: i32, b: i32) -> i32 {
    a + b
}''',
            language="rust",
            font_size=13,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.7)
        code.next_to(title, DOWN, buff=0.3).to_edge(LEFT, buff=0.3)

        self.play(Create(code))
        self.wait(1)

        # cargo doc
        doc_cmd = VGroup(
            Code(
                code="$ cargo doc --open",
                language="bash",
                font_size=18,
            ).scale(0.8),
            Text("→ 아름다운 HTML 문서 생성!", font_size=16, color=SUCCESS_GREEN),
        ).arrange(DOWN, buff=0.2)
        doc_cmd.to_edge(RIGHT, buff=0.5)

        self.play(Create(doc_cmd))
        self.wait(1)

        # 문서 테스트
        doctest = VGroup(
            Text("📝 문서의 코드 예제도 테스트됨!", font_size=18, color=TEST_CYAN),
            Text("cargo test가 /// ``` 블록도 실행", font_size=14, color=GRAY),
        ).arrange(DOWN, buff=0.1)
        doctest.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(doctest, shift=UP))
        self.wait(2)

    def section_publishing(self):
        title = Text("crates.io 배포", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # 단계
        steps = VGroup(
            self.create_step("1", "계정 설정", "cargo login <api-token>"),
            self.create_step("2", "메타데이터", "Cargo.toml에 필수 정보 추가"),
            self.create_step("3", "배포", "cargo publish"),
        ).arrange(DOWN, buff=0.4)
        steps.next_to(title, DOWN, buff=0.4).to_edge(LEFT, buff=0.5)

        for step in steps:
            self.play(FadeIn(step, shift=RIGHT), run_time=0.4)

        self.wait(0.5)

        # Cargo.toml
        toml = Code(
            code='''[package]
name = "my-awesome-crate"
version = "0.1.0"
edition = "2021"
description = "An awesome crate"
license = "MIT OR Apache-2.0"
repository = "https://github.com/..."
keywords = ["rust", "awesome"]
categories = ["development-tools"]

[dependencies]
# ...''',
            language="toml",
            font_size=13,
            background="rectangle",
            background_stroke_color=CARGO_BROWN,
        ).scale(0.7)
        toml.to_edge(RIGHT, buff=0.5)

        self.play(Create(toml))
        self.wait(1)

        # 주의사항
        warning = VGroup(
            Text("⚠️ 배포된 버전은 삭제 불가!", font_size=18, color=WARNING_YELLOW),
            Text("cargo yank로 사용 비권장만 가능", font_size=14, color=GRAY),
        ).arrange(DOWN, buff=0.1)
        warning.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(warning, shift=UP))
        self.wait(2)

    def create_step(self, num, title_text, desc):
        circle = Circle(radius=0.3, fill_color=CARGO_BROWN, fill_opacity=0.5)
        num_text = Text(num, font_size=20, color=WHITE)
        num_text.move_to(circle)

        content = VGroup(
            Text(title_text, font_size=18, color=WHITE),
            Text(desc, font_size=12, color=GRAY),
        ).arrange(DOWN, buff=0.05, aligned_edge=LEFT)
        content.next_to(circle, RIGHT, buff=0.3)

        return VGroup(circle, num_text, content)

    def section_workspaces(self):
        title = Text("Cargo 작업 공간", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # 구조
        structure = Code(
            code='''my_workspace/
├── Cargo.toml       # 작업 공간 루트
├── Cargo.lock       # 공유 락 파일
├── target/          # 공유 빌드 디렉토리
├── app/
│   ├── Cargo.toml
│   └── src/main.rs
├── core/
│   ├── Cargo.toml
│   └── src/lib.rs
└── utils/
    ├── Cargo.toml
    └── src/lib.rs''',
            language="text",
            font_size=14,
        ).scale(0.75)
        structure.next_to(title, DOWN, buff=0.3).to_edge(LEFT, buff=0.5)

        self.play(Create(structure))
        self.wait(0.5)

        # 루트 Cargo.toml
        root_toml = Code(
            code='''# 루트 Cargo.toml
[workspace]
members = [
    "app",
    "core",
    "utils",
]

# 공유 의존성
[workspace.dependencies]
serde = "1.0"
tokio = { version = "1", features = ["full"] }''',
            language="toml",
            font_size=14,
            background="rectangle",
            background_stroke_color=CARGO_BROWN,
        ).scale(0.7)
        root_toml.to_edge(RIGHT, buff=0.5)

        self.play(Create(root_toml))
        self.wait(1)

        # 장점
        benefits = VGroup(
            Text("작업 공간의 장점:", font_size=18, color=SUCCESS_GREEN),
            Text("• 의존성 버전 통일", font_size=14, color=GRAY),
            Text("• 빌드 캐시 공유", font_size=14, color=GRAY),
            Text("• 한 번에 전체 빌드/테스트", font_size=14, color=GRAY),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.1)
        benefits.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(benefits, shift=UP))
        self.wait(2)

    def section_outro(self):
        title = Text("EP 6-02 정리", font_size=44, color=RUST_ORANGE)

        summary = VGroup(
            Text("[profile.*] = 빌드 최적화 설정", font_size=24, color=STACK_GREEN),
            Text("/// = 문서화 주석 → cargo doc", font_size=24, color=MEMORY_BLUE),
            Text("cargo publish = crates.io 배포", font_size=24, color=HEAP_PURPLE),
            Text("[workspace] = 모노레포 관리", font_size=24, color=CARGO_BROWN),
        ).arrange(DOWN, buff=0.4)

        VGroup(title, summary).arrange(DOWN, buff=0.8)

        self.play(Write(title))
        for line in summary:
            self.play(FadeIn(line, shift=UP), run_time=0.4)

        self.wait(1)

        # Level 6 완료
        complete = Text("🎉 Level 6: 테스팅 & 도구 완료!", font_size=32, color=SUCCESS_GREEN)
        next_level = Text("다음: Level 7 - 고급 패턴", font_size=24, color=GRAY)

        VGroup(complete, next_level).arrange(DOWN, buff=0.3).to_edge(DOWN, buff=0.8)

        self.play(FadeIn(complete, scale=1.2))
        self.play(FadeIn(next_level))
        self.wait(2)


# 렌더링 헬퍼
if __name__ == "__main__":
    print("""
╔═══════════════════════════════════════════════════════════════╗
║          WIA-RUST-LEARN Level 6: 테스팅 & 도구                ║
╠═══════════════════════════════════════════════════════════════╣
║                                                               ║
║  EP 6-01: Testing                                             ║
║    manim -pql level_6_testing.py EP_6_01_Testing              ║
║                                                               ║
║  EP 6-02: Cargo Advanced                                      ║
║    manim -pql level_6_testing.py EP_6_02_CargoAdvanced        ║
║                                                               ║
║  전체 렌더링:                                                  ║
║    manim -pqh level_6_testing.py --all                        ║
║                                                               ║
╚═══════════════════════════════════════════════════════════════╝
    """)
