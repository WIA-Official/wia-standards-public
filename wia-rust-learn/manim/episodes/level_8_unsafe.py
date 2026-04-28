"""
WIA-RUST-LEARN Level 8: Unsafe & 매크로 (Unsafe & Macros)
=========================================================

EP 8-01: Unsafe Rust - 안전 경계 넘기 (22분)
EP 8-02: Macros - 메타프로그래밍의 세계 (25분)

실행 방법:
    manim -pql level_8_unsafe.py EP_8_01_UnsafeRust
    manim -pql level_8_unsafe.py EP_8_02_Macros

전체 렌더링 (고화질):
    manim -pqh level_8_unsafe.py --all

⚠️ 이 레벨은 고급 주제입니다. Level 0-5를 완료한 후 도전하세요!
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
UNSAFE_RED = "#B71C1C"
MACRO_PURPLE = "#7B1FA2"
DARK_BG = "#1a1a2e"


class EP_8_01_UnsafeRust(Scene):
    """
    EP 8-01: Unsafe Rust - 안전 경계 넘기

    학습 목표:
    1. Unsafe의 5가지 슈퍼파워
    2. 원시 포인터 다루기
    3. 안전하지 않은 함수 호출
    4. FFI (Foreign Function Interface)
    """

    def construct(self):
        self.camera.background_color = DARK_BG

        self.section_intro()
        self.wait(0.5)
        self.clear()

        self.section_five_superpowers()
        self.wait(0.5)
        self.clear()

        self.section_raw_pointers()
        self.wait(0.5)
        self.clear()

        self.section_unsafe_functions()
        self.wait(0.5)
        self.clear()

        self.section_ffi()
        self.wait(0.5)
        self.clear()

        self.section_safe_abstraction()
        self.wait(0.5)
        self.clear()

        self.section_outro()

    def section_intro(self):
        title = Text("EP 8-01", font_size=32, color=GRAY).to_edge(UP)
        main_title = Text("Unsafe Rust", font_size=56, color=UNSAFE_RED)
        subtitle = Text("안전 경계 넘기", font_size=28, color=GRAY)

        main_title.next_to(title, DOWN, buff=0.8)
        subtitle.next_to(main_title, DOWN, buff=0.3)

        self.play(Write(title), Write(main_title))
        self.play(FadeIn(subtitle))
        self.wait(1)

        # Unsafe의 오해
        misconception = VGroup(
            Text("⚠️ Unsafe ≠ 위험한 코드", font_size=24, color=WARNING_YELLOW),
            Text("Unsafe = \"컴파일러야, 내가 책임질게\"", font_size=20, color=WHITE),
        ).arrange(DOWN, buff=0.3)
        misconception.next_to(subtitle, DOWN, buff=0.8)

        self.play(FadeIn(misconception, shift=UP))
        self.wait(1)

        # 왜 필요한가
        why_need = VGroup(
            Text("왜 필요한가?", font_size=22, color=RUST_ORANGE),
            VGroup(
                Text("• 하드웨어와 직접 상호작용", font_size=16, color=GRAY),
                Text("• C 라이브러리 호출 (FFI)", font_size=16, color=GRAY),
                Text("• 성능 최적화", font_size=16, color=GRAY),
                Text("• 자료구조 구현 (LinkedList 등)", font_size=16, color=GRAY),
            ).arrange(DOWN, aligned_edge=LEFT, buff=0.1),
        ).arrange(DOWN, buff=0.3)
        why_need.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(why_need, shift=UP))
        self.wait(2)

    def section_five_superpowers(self):
        title = Text("Unsafe의 5가지 슈퍼파워", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        powers = VGroup(
            self.create_power_item("1", "원시 포인터 역참조", "*const T, *mut T"),
            self.create_power_item("2", "안전하지 않은 함수 호출", "unsafe fn"),
            self.create_power_item("3", "가변 정적 변수 접근", "static mut"),
            self.create_power_item("4", "안전하지 않은 트레이트 구현", "unsafe trait"),
            self.create_power_item("5", "유니언 필드 접근", "union"),
        ).arrange(DOWN, buff=0.35, aligned_edge=LEFT)
        powers.next_to(title, DOWN, buff=0.5)

        for power in powers:
            self.play(FadeIn(power, shift=RIGHT), run_time=0.4)

        self.wait(1)

        # 중요한 점
        important = VGroup(
            Text("⚡ 중요:", font_size=20, color=WARNING_YELLOW),
            Text("unsafe 블록 안에서도", font_size=16, color=WHITE),
            Text("대여 검사기는 여전히 작동!", font_size=16, color=WHITE),
        ).arrange(DOWN, buff=0.1)
        important.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(important, shift=UP))
        self.wait(2)

    def create_power_item(self, num, title_text, example):
        circle = Circle(radius=0.25, fill_color=UNSAFE_RED, fill_opacity=0.5)
        num_text = Text(num, font_size=16, color=WHITE)
        num_text.move_to(circle)

        content = VGroup(
            Text(title_text, font_size=18, color=WHITE),
            Text(example, font_size=12, color=GRAY),
        ).arrange(RIGHT, buff=0.5)

        return VGroup(circle, num_text, content).arrange(RIGHT, buff=0.3)

    def section_raw_pointers(self):
        title = Text("원시 포인터", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''let mut num = 5;

// 원시 포인터 생성 (안전!)
let r1 = &num as *const i32;      // 불변 원시 포인터
let r2 = &mut num as *mut i32;    // 가변 원시 포인터

// 역참조는 unsafe!
unsafe {
    println!("r1: {}", *r1);
    *r2 = 10;
    println!("r2: {}", *r2);
}

// 참조자와의 차이
// • null 가능
// • 대여 규칙 무시
// • 자동 정리 없음
// • 유효성 보장 없음''',
            language="rust",
            font_size=14,
            background="rectangle",
            background_stroke_color=UNSAFE_RED,
        ).scale(0.7)
        code.next_to(title, DOWN, buff=0.3).to_edge(LEFT, buff=0.5)

        self.play(Create(code))
        self.wait(1)

        # 비교 다이어그램
        comparison = VGroup(
            VGroup(
                Text("참조자 (&T)", font_size=18, color=SUCCESS_GREEN),
                Text("• 항상 유효", font_size=12, color=GRAY),
                Text("• 대여 규칙 적용", font_size=12, color=GRAY),
                Text("• null 불가", font_size=12, color=GRAY),
            ).arrange(DOWN, buff=0.1, aligned_edge=LEFT),
            VGroup(
                Text("원시 포인터 (*T)", font_size=18, color=UNSAFE_RED),
                Text("• 유효성 미보장", font_size=12, color=GRAY),
                Text("• 대여 규칙 무시", font_size=12, color=GRAY),
                Text("• null 가능", font_size=12, color=GRAY),
            ).arrange(DOWN, buff=0.1, aligned_edge=LEFT),
        ).arrange(RIGHT, buff=1)
        comparison.to_edge(RIGHT, buff=0.5)

        self.play(FadeIn(comparison, shift=LEFT))
        self.wait(2)

    def section_unsafe_functions(self):
        title = Text("안전하지 않은 함수", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''// unsafe 함수 정의
unsafe fn dangerous() {
    // 이 함수를 호출하는 것 자체가 unsafe
}

// 호출
unsafe {
    dangerous();
}

// 실제 예: slice를 두 개로 분할
fn split_at_mut(values: &mut [i32], mid: usize)
    -> (&mut [i32], &mut [i32])
{
    let len = values.len();
    let ptr = values.as_mut_ptr();

    assert!(mid <= len);

    unsafe {
        (
            std::slice::from_raw_parts_mut(ptr, mid),
            std::slice::from_raw_parts_mut(ptr.add(mid), len - mid),
        )
    }
}''',
            language="rust",
            font_size=12,
            background="rectangle",
            background_stroke_color=UNSAFE_RED,
        ).scale(0.7)
        code.next_to(title, DOWN, buff=0.3)

        self.play(Create(code))
        self.wait(1)

        # 핵심
        key = VGroup(
            Text("안전한 추상화의 예!", font_size=20, color=SUCCESS_GREEN),
            Text("내부는 unsafe, 외부 API는 safe", font_size=16, color=GRAY),
        ).arrange(DOWN, buff=0.1)
        key.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(key, shift=UP))
        self.wait(2)

    def section_ffi(self):
        title = Text("FFI: 외부 함수 인터페이스", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''// C 함수 호출하기
extern "C" {
    fn abs(input: i32) -> i32;
    fn strlen(s: *const std::ffi::c_char) -> usize;
}

fn main() {
    unsafe {
        println!("abs(-3) = {}", abs(-3));
    }
}

// Rust 함수를 C에서 호출 가능하게
#[no_mangle]
pub extern "C" fn rust_function(x: i32) -> i32 {
    x * 2
}

// C에서 호출:
// int result = rust_function(21);  // 42''',
            language="rust",
            font_size=13,
            background="rectangle",
            background_stroke_color=UNSAFE_RED,
        ).scale(0.7)
        code.next_to(title, DOWN, buff=0.3)

        self.play(Create(code))
        self.wait(1)

        # FFI 사용 사례
        use_cases = VGroup(
            Text("FFI 사용 사례:", font_size=18, color=MEMORY_BLUE),
            VGroup(
                Text("• OpenSSL, SQLite 등 C 라이브러리", font_size=14, color=GRAY),
                Text("• Python, Node.js 확장 모듈", font_size=14, color=GRAY),
                Text("• 게임 엔진 통합", font_size=14, color=GRAY),
            ).arrange(DOWN, aligned_edge=LEFT, buff=0.1),
        ).arrange(DOWN, buff=0.2)
        use_cases.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(use_cases, shift=UP))
        self.wait(2)

    def section_safe_abstraction(self):
        title = Text("안전한 추상화 원칙", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # 원칙
        principles = VGroup(
            VGroup(
                Text("1. Unsafe를 최소화", font_size=20, color=SUCCESS_GREEN),
                Text("가능한 한 작은 블록으로", font_size=14, color=GRAY),
            ).arrange(DOWN, buff=0.1),
            VGroup(
                Text("2. Safe API 제공", font_size=20, color=SUCCESS_GREEN),
                Text("외부에는 안전한 인터페이스만 노출", font_size=14, color=GRAY),
            ).arrange(DOWN, buff=0.1),
            VGroup(
                Text("3. 불변조건 문서화", font_size=20, color=SUCCESS_GREEN),
                Text("어떤 가정이 필요한지 명시", font_size=14, color=GRAY),
            ).arrange(DOWN, buff=0.1),
            VGroup(
                Text("4. 철저한 테스트", font_size=20, color=SUCCESS_GREEN),
                Text("Miri, Valgrind 등 도구 활용", font_size=14, color=GRAY),
            ).arrange(DOWN, buff=0.1),
        ).arrange(DOWN, buff=0.4, aligned_edge=LEFT)
        principles.next_to(title, DOWN, buff=0.5)

        for p in principles:
            self.play(FadeIn(p, shift=RIGHT), run_time=0.4)

        self.wait(1)

        # 결론
        conclusion = VGroup(
            Text("\"Unsafe is a promise, not a disclaimer\"", font_size=22, color=RUST_ORANGE),
            Text("- Rust 커뮤니티", font_size=16, color=GRAY),
        ).arrange(DOWN, buff=0.2)
        conclusion.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(conclusion, shift=UP))
        self.wait(2)

    def section_outro(self):
        title = Text("EP 8-01 정리", font_size=44, color=RUST_ORANGE)

        summary = VGroup(
            Text("unsafe = 5가지 슈퍼파워 해금", font_size=24, color=UNSAFE_RED),
            Text("*const/*mut = 원시 포인터", font_size=24, color=MEMORY_BLUE),
            Text("extern \"C\" = FFI 연결", font_size=24, color=HEAP_PURPLE),
            Text("안전한 추상화로 감싸기!", font_size=24, color=SUCCESS_GREEN),
        ).arrange(DOWN, buff=0.4)

        VGroup(title, summary).arrange(DOWN, buff=0.8)

        self.play(Write(title))
        for line in summary:
            self.play(FadeIn(line, shift=UP), run_time=0.4)

        self.wait(1)

        next_ep = Text("다음: EP 8-02 Macros", font_size=28, color=GRAY)
        next_ep.to_edge(DOWN, buff=1)
        self.play(FadeIn(next_ep))
        self.wait(2)


class EP_8_02_Macros(Scene):
    """
    EP 8-02: Macros - 메타프로그래밍의 세계

    학습 목표:
    1. 선언적 매크로 (macro_rules!)
    2. 절차적 매크로 (proc-macro)
    3. derive 매크로 만들기
    4. 매크로 vs 함수
    """

    def construct(self):
        self.camera.background_color = DARK_BG

        self.section_intro()
        self.wait(0.5)
        self.clear()

        self.section_macro_basics()
        self.wait(0.5)
        self.clear()

        self.section_declarative_macros()
        self.wait(0.5)
        self.clear()

        self.section_procedural_macros()
        self.wait(0.5)
        self.clear()

        self.section_derive_macro()
        self.wait(0.5)
        self.clear()

        self.section_macro_tips()
        self.wait(0.5)
        self.clear()

        self.section_outro()

    def section_intro(self):
        title = Text("EP 8-02", font_size=32, color=GRAY).to_edge(UP)
        main_title = Text("Macros", font_size=56, color=RUST_ORANGE)
        subtitle = Text("메타프로그래밍의 세계", font_size=28, color=GRAY)

        main_title.next_to(title, DOWN, buff=0.8)
        subtitle.next_to(main_title, DOWN, buff=0.3)

        self.play(Write(title), Write(main_title))
        self.play(FadeIn(subtitle))
        self.wait(1)

        # 매크로란?
        what_is = VGroup(
            Text("매크로 = 코드를 생성하는 코드", font_size=24, color=MACRO_PURPLE),
            Text("컴파일 타임에 코드 확장", font_size=18, color=GRAY),
        ).arrange(DOWN, buff=0.3)
        what_is.next_to(subtitle, DOWN, buff=0.8)

        self.play(FadeIn(what_is, shift=UP))
        self.wait(1)

        # 익숙한 매크로들
        familiar = VGroup(
            Text("이미 써본 매크로들:", font_size=20, color=RUST_ORANGE),
            VGroup(
                Text("• println!(\"Hello\")", font_size=16, color=WHITE),
                Text("• vec![1, 2, 3]", font_size=16, color=WHITE),
                Text("• #[derive(Debug)]", font_size=16, color=WHITE),
                Text("• panic!(\"Error\")", font_size=16, color=WHITE),
            ).arrange(DOWN, aligned_edge=LEFT, buff=0.1),
        ).arrange(DOWN, buff=0.2)
        familiar.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(familiar, shift=UP))
        self.wait(2)

    def section_macro_basics(self):
        title = Text("매크로 vs 함수", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        comparison = VGroup(
            VGroup(
                Text("함수", font_size=24, color=STACK_GREEN),
                VGroup(
                    Text("• 런타임에 호출", font_size=14, color=GRAY),
                    Text("• 고정된 매개변수", font_size=14, color=GRAY),
                    Text("• 타입 시스템 적용", font_size=14, color=GRAY),
                    Text("• 디버깅 쉬움", font_size=14, color=GRAY),
                ).arrange(DOWN, aligned_edge=LEFT, buff=0.1),
            ).arrange(DOWN, buff=0.2),
            VGroup(
                Text("매크로", font_size=24, color=MACRO_PURPLE),
                VGroup(
                    Text("• 컴파일 타임에 확장", font_size=14, color=GRAY),
                    Text("• 가변 인자 가능", font_size=14, color=GRAY),
                    Text("• 새로운 문법 가능", font_size=14, color=GRAY),
                    Text("• 디버깅 어려움", font_size=14, color=GRAY),
                ).arrange(DOWN, aligned_edge=LEFT, buff=0.1),
            ).arrange(DOWN, buff=0.2),
        ).arrange(RIGHT, buff=2)
        comparison.next_to(title, DOWN, buff=0.5)

        self.play(FadeIn(comparison))
        self.wait(1)

        # 언제 매크로?
        when = VGroup(
            Text("매크로가 필요한 경우:", font_size=18, color=WARNING_YELLOW),
            Text("• 가변 개수 인자 (println!)", font_size=14, color=GRAY),
            Text("• 반복 코드 생성 (impl 여러 타입)", font_size=14, color=GRAY),
            Text("• DSL 구현", font_size=14, color=GRAY),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.1)
        when.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(when, shift=UP))
        self.wait(2)

    def section_declarative_macros(self):
        title = Text("선언적 매크로: macro_rules!", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''// vec! 매크로 간단 버전
macro_rules! my_vec {
    // 패턴 => 확장
    ( $( $x:expr ),* ) => {
        {
            let mut temp_vec = Vec::new();
            $(
                temp_vec.push($x);
            )*
            temp_vec
        }
    };
}

// 사용
let v = my_vec![1, 2, 3];

// 확장 결과:
// {
//     let mut temp_vec = Vec::new();
//     temp_vec.push(1);
//     temp_vec.push(2);
//     temp_vec.push(3);
//     temp_vec
// }''',
            language="rust",
            font_size=12,
            background="rectangle",
            background_stroke_color=MACRO_PURPLE,
        ).scale(0.7)
        code.next_to(title, DOWN, buff=0.3)

        self.play(Create(code))
        self.wait(1)

        # 패턴 문법
        syntax = VGroup(
            Text("패턴 문법:", font_size=16, color=MACRO_PURPLE),
            Text("$x:expr = 표현식", font_size=12, color=GRAY),
            Text("$( ... ),* = 0개 이상 반복", font_size=12, color=GRAY),
            Text("$( ... ),+ = 1개 이상 반복", font_size=12, color=GRAY),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.1)
        syntax.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(syntax, shift=UP))
        self.wait(2)

    def section_procedural_macros(self):
        title = Text("절차적 매크로", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # 3가지 종류
        types = VGroup(
            self.create_macro_type("derive", "#[derive(MyTrait)]", "트레이트 자동 구현"),
            self.create_macro_type("attribute", "#[my_attr]", "아이템에 속성 추가"),
            self.create_macro_type("function-like", "my_macro!(...)", "함수처럼 호출"),
        ).arrange(DOWN, buff=0.4)
        types.next_to(title, DOWN, buff=0.5)

        for t in types:
            self.play(FadeIn(t, shift=RIGHT), run_time=0.4)

        self.wait(1)

        # proc-macro 크레이트
        note = VGroup(
            Text("절차적 매크로는 별도 크레이트 필요:", font_size=16, color=WARNING_YELLOW),
            Code(
                code='''# Cargo.toml
[lib]
proc-macro = true

[dependencies]
quote = "1"
syn = "2"''',
                language="toml",
                font_size=12,
            ).scale(0.8),
        ).arrange(DOWN, buff=0.2)
        note.to_edge(DOWN, buff=0.3)

        self.play(FadeIn(note, shift=UP))
        self.wait(2)

    def create_macro_type(self, name, example, desc):
        box_rect = RoundedRectangle(
            width=8, height=0.9, corner_radius=0.1,
            fill_color=MACRO_PURPLE, fill_opacity=0.1,
            stroke_color=MACRO_PURPLE
        )
        content = VGroup(
            Text(name, font_size=18, color=MACRO_PURPLE),
            Text(example, font_size=14, color=WHITE),
            Text(desc, font_size=12, color=GRAY),
        ).arrange(RIGHT, buff=0.5)
        content.move_to(box_rect)
        return VGroup(box_rect, content)

    def section_derive_macro(self):
        title = Text("derive 매크로 만들기", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''// my_macro/src/lib.rs
use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, DeriveInput};

#[proc_macro_derive(HelloMacro)]
pub fn hello_macro_derive(input: TokenStream) -> TokenStream {
    let ast = parse_macro_input!(input as DeriveInput);
    let name = &ast.ident;

    let gen = quote! {
        impl HelloMacro for #name {
            fn hello_macro() {
                println!("Hello from {}!", stringify!(#name));
            }
        }
    };

    gen.into()
}

// 사용
#[derive(HelloMacro)]
struct Pancakes;

Pancakes::hello_macro();  // "Hello from Pancakes!"''',
            language="rust",
            font_size=11,
            background="rectangle",
            background_stroke_color=MACRO_PURPLE,
        ).scale(0.68)
        code.next_to(title, DOWN, buff=0.2)

        self.play(Create(code))
        self.wait(2)

    def section_macro_tips(self):
        title = Text("매크로 작성 팁", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        tips = VGroup(
            VGroup(
                Text("1. cargo expand로 확장 결과 확인", font_size=18, color=SUCCESS_GREEN),
                Code(code="cargo expand", language="bash", font_size=12).scale(0.8),
            ).arrange(DOWN, buff=0.1),
            VGroup(
                Text("2. 가능하면 함수를 먼저 고려", font_size=18, color=SUCCESS_GREEN),
                Text("매크로는 마지막 수단", font_size=14, color=GRAY),
            ).arrange(DOWN, buff=0.1),
            VGroup(
                Text("3. 좋은 에러 메시지 제공", font_size=18, color=SUCCESS_GREEN),
                Text("compile_error! 매크로 활용", font_size=14, color=GRAY),
            ).arrange(DOWN, buff=0.1),
            VGroup(
                Text("4. 위생성(hygiene) 이해하기", font_size=18, color=SUCCESS_GREEN),
                Text("매크로 내 변수는 외부와 충돌 안 함", font_size=14, color=GRAY),
            ).arrange(DOWN, buff=0.1),
        ).arrange(DOWN, buff=0.35, aligned_edge=LEFT)
        tips.next_to(title, DOWN, buff=0.4)

        for tip in tips:
            self.play(FadeIn(tip, shift=RIGHT), run_time=0.4)

        self.wait(2)

    def section_outro(self):
        title = Text("EP 8-02 정리", font_size=44, color=RUST_ORANGE)

        summary = VGroup(
            Text("macro_rules! = 선언적 매크로", font_size=24, color=MACRO_PURPLE),
            Text("proc_macro = 절차적 매크로", font_size=24, color=HEAP_PURPLE),
            Text("#[derive] = 자동 구현 매크로", font_size=24, color=SUCCESS_GREEN),
            Text("quote! + syn = 절차적 매크로 도구", font_size=24, color=MEMORY_BLUE),
        ).arrange(DOWN, buff=0.4)

        VGroup(title, summary).arrange(DOWN, buff=0.8)

        self.play(Write(title))
        for line in summary:
            self.play(FadeIn(line, shift=UP), run_time=0.4)

        self.wait(1)

        # Level 8 완료
        complete = Text("🎉 Level 8: Unsafe & 매크로 완료!", font_size=32, color=SUCCESS_GREEN)
        next_level = Text("다음: Level 9 - 실전 프로젝트", font_size=24, color=GRAY)

        VGroup(complete, next_level).arrange(DOWN, buff=0.3).to_edge(DOWN, buff=0.8)

        self.play(FadeIn(complete, scale=1.2))
        self.play(FadeIn(next_level))
        self.wait(2)


# 렌더링 헬퍼
if __name__ == "__main__":
    print("""
╔═══════════════════════════════════════════════════════════════╗
║          WIA-RUST-LEARN Level 8: Unsafe & 매크로              ║
╠═══════════════════════════════════════════════════════════════╣
║                                                               ║
║  EP 8-01: Unsafe Rust                                         ║
║    manim -pql level_8_unsafe.py EP_8_01_UnsafeRust            ║
║                                                               ║
║  EP 8-02: Macros                                              ║
║    manim -pql level_8_unsafe.py EP_8_02_Macros                ║
║                                                               ║
║  전체 렌더링:                                                  ║
║    manim -pqh level_8_unsafe.py --all                         ║
║                                                               ║
║  ⚠️ 고급 주제입니다. Level 0-5 완료 후 도전하세요!            ║
║                                                               ║
╚═══════════════════════════════════════════════════════════════╝
    """)
