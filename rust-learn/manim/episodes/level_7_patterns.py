"""
WIA-RUST-LEARN Level 7: 고급 패턴 (Advanced Patterns)
=====================================================

EP 7-01: Pattern Matching Deep Dive - 패턴의 모든 것 (22분)
EP 7-02: OOP in Rust - Rust 방식의 객체지향 (20분)

실행 방법:
    manim -pql level_7_patterns.py EP_7_01_PatternDeep
    manim -pql level_7_patterns.py EP_7_02_OOPRust

전체 렌더링 (고화질):
    manim -pqh level_7_patterns.py --all
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
PATTERN_PINK = "#E91E63"
OOP_INDIGO = "#3F51B5"
DARK_BG = "#1a1a2e"


class EP_7_01_PatternDeep(Scene):
    """
    EP 7-01: Pattern Matching Deep Dive - 패턴의 모든 것

    학습 목표:
    1. 패턴이 사용되는 모든 위치
    2. 반박 가능 vs 반박 불가능 패턴
    3. 모든 패턴 문법
    4. 매치 가드와 @ 바인딩
    """

    def construct(self):
        self.camera.background_color = DARK_BG

        self.section_intro()
        self.wait(0.5)
        self.clear()

        self.section_pattern_places()
        self.wait(0.5)
        self.clear()

        self.section_refutability()
        self.wait(0.5)
        self.clear()

        self.section_pattern_syntax()
        self.wait(0.5)
        self.clear()

        self.section_destructuring()
        self.wait(0.5)
        self.clear()

        self.section_advanced_patterns()
        self.wait(0.5)
        self.clear()

        self.section_outro()

    def section_intro(self):
        title = Text("EP 7-01", font_size=32, color=GRAY).to_edge(UP)
        main_title = Text("Pattern Matching", font_size=56, color=RUST_ORANGE)
        subtitle = Text("Deep Dive - 패턴의 모든 것", font_size=28, color=GRAY)

        main_title.next_to(title, DOWN, buff=0.8)
        subtitle.next_to(main_title, DOWN, buff=0.3)

        self.play(Write(title), Write(main_title))
        self.play(FadeIn(subtitle))
        self.wait(1)

        # 패턴이란?
        what_is = VGroup(
            Text("패턴 = 데이터 구조를 분해하는 템플릿", font_size=24, color=PATTERN_PINK),
            VGroup(
                Text("• 값의 구조와 매칭", font_size=18, color=GRAY),
                Text("• 동시에 값 추출 (바인딩)", font_size=18, color=GRAY),
                Text("• 컴파일 타임 완전성 검사", font_size=18, color=GRAY),
            ).arrange(DOWN, aligned_edge=LEFT, buff=0.15),
        ).arrange(DOWN, buff=0.4)
        what_is.next_to(subtitle, DOWN, buff=0.8)

        self.play(FadeIn(what_is, shift=UP))
        self.wait(2)

    def section_pattern_places(self):
        title = Text("패턴이 사용되는 곳", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # 6가지 위치
        places = VGroup(
            self.create_place_item("match 갈래", "match x { 패턴 => ... }", PATTERN_PINK),
            self.create_place_item("if let", "if let Some(x) = option", SUCCESS_GREEN),
            self.create_place_item("while let", "while let Some(x) = iter.next()", MEMORY_BLUE),
            self.create_place_item("for 루프", "for (i, val) in enumerate()", WARNING_YELLOW),
            self.create_place_item("let 구문", "let (x, y) = tuple;", HEAP_PURPLE),
            self.create_place_item("함수 매개변수", "fn foo((x, y): (i32, i32))", STACK_GREEN),
        ).arrange(DOWN, buff=0.25, aligned_edge=LEFT)
        places.next_to(title, DOWN, buff=0.4)

        for place in places:
            self.play(FadeIn(place, shift=RIGHT), run_time=0.35)

        self.wait(1)

        # 핵심
        key = Text("패턴은 Rust 어디에나 있다!", font_size=24, color=RUST_ORANGE)
        key.to_edge(DOWN, buff=0.5)
        self.play(Write(key))
        self.wait(2)

    def create_place_item(self, name, example, color):
        dot = Circle(radius=0.1, fill_color=color, fill_opacity=1, stroke_width=0)
        content = VGroup(
            Text(name, font_size=18, color=color),
            Text(example, font_size=14, color=GRAY),
        ).arrange(RIGHT, buff=0.5)
        return VGroup(dot, content).arrange(RIGHT, buff=0.3)

    def section_refutability(self):
        title = Text("반박 가능성 (Refutability)", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # 두 종류
        types = VGroup(
            VGroup(
                Text("반박 불가능 (Irrefutable)", font_size=24, color=SUCCESS_GREEN),
                Text("항상 매칭됨", font_size=16, color=GRAY),
                Code(
                    code='''let x = 5;           // 항상 성공
let (a, b) = (1, 2); // 항상 성공
fn foo(x: i32) { }   // 항상 성공''',
                    language="rust",
                    font_size=12,
                ).scale(0.8),
            ).arrange(DOWN, buff=0.2),
            VGroup(
                Text("반박 가능 (Refutable)", font_size=24, color=WARNING_YELLOW),
                Text("실패할 수 있음", font_size=16, color=GRAY),
                Code(
                    code='''if let Some(x) = opt { } // None이면 실패
match result {
    Ok(v) => {},    // Err면 실패
    Err(e) => {},
}''',
                    language="rust",
                    font_size=12,
                ).scale(0.8),
            ).arrange(DOWN, buff=0.2),
        ).arrange(RIGHT, buff=1)
        types.next_to(title, DOWN, buff=0.5)

        self.play(FadeIn(types))
        self.wait(1)

        # 규칙
        rules = VGroup(
            Text("규칙:", font_size=20, color=RUST_ORANGE),
            Text("• let, for, 함수 매개변수 → 반박 불가능만", font_size=16, color=GRAY),
            Text("• if let, while let → 반박 가능 허용", font_size=16, color=GRAY),
            Text("• match → 모든 케이스 커버해야 함", font_size=16, color=GRAY),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.15)
        rules.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(rules, shift=UP))
        self.wait(2)

    def section_pattern_syntax(self):
        title = Text("패턴 문법 총정리", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        patterns = Code(
            code='''// 리터럴
match x { 1 => "one", 2 => "two", _ => "other" }

// 명명된 변수
let x = 5;  // x에 5 바인딩

// 다중 패턴 (|)
match x { 1 | 2 | 3 => "small", _ => "big" }

// 범위 (..=)
match x { 1..=5 => "1-5", 6..=10 => "6-10", _ => "other" }

// 와일드카드 (_)
let (first, _, third) = (1, 2, 3);  // 2 무시

// 나머지 패턴 (..)
let (first, .., last) = (1, 2, 3, 4, 5);''',
            language="rust",
            font_size=13,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.7)
        code.next_to(title, DOWN, buff=0.3)

        self.play(Create(patterns))
        self.wait(2)

    def section_destructuring(self):
        title = Text("구조체/열거형 해체", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''struct Point { x: i32, y: i32 }
enum Message { Move { x: i32, y: i32 }, Write(String), Quit }

// 구조체 해체
let p = Point { x: 10, y: 20 };
let Point { x, y } = p;           // x=10, y=20
let Point { x: a, y: b } = p;     // 이름 변경: a=10, b=20
let Point { x, .. } = p;          // y 무시

// 열거형 해체
match msg {
    Message::Move { x, y } => println!("Move to {},{}", x, y),
    Message::Write(text) => println!("Text: {}", text),
    Message::Quit => println!("Quit"),
}

// 중첩 해체
let ((a, b), Point { x, y }) = ((1, 2), Point { x: 3, y: 4 });''',
            language="rust",
            font_size=13,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.68)
        code.next_to(title, DOWN, buff=0.3)

        self.play(Create(code))
        self.wait(2)

    def section_advanced_patterns(self):
        title = Text("고급 패턴: 가드 & 바인딩", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''// 매치 가드 (if 조건)
match num {
    x if x < 0 => println!("음수: {}", x),
    x if x > 100 => println!("큰 수: {}", x),
    x => println!("보통: {}", x),
}

// 다중 패턴과 가드
match x {
    4 | 5 | 6 if condition => println!("조건 충족"),
    _ => println!("기타"),
}

// @ 바인딩 (값 테스트 + 변수 바인딩)
match msg {
    Message::Hello { id: id_var @ 3..=7 } => {
        println!("id {} (3-7 범위)", id_var)
    }
    Message::Hello { id: 10..=12 } => {
        println!("10-12 범위 (값 접근 불가)")
    }
    Message::Hello { id } => println!("기타 id: {}", id),
}''',
            language="rust",
            font_size=13,
            background="rectangle",
            background_stroke_color=PATTERN_PINK,
        ).scale(0.68)
        code.next_to(title, DOWN, buff=0.3)

        self.play(Create(code))
        self.wait(1)

        # @ 설명
        at_explain = VGroup(
            Text("@ = ", font_size=24, color=PATTERN_PINK),
            Text("값 테스트와 동시에 변수에 바인딩", font_size=18, color=WHITE),
        ).arrange(RIGHT, buff=0.2)
        at_explain.to_edge(DOWN, buff=0.5)

        self.play(Write(at_explain))
        self.wait(2)

    def section_outro(self):
        title = Text("EP 7-01 정리", font_size=44, color=RUST_ORANGE)

        summary = VGroup(
            Text("패턴은 6군데에서 사용", font_size=24, color=PATTERN_PINK),
            Text("반박 가능/불가능 구분 중요", font_size=24, color=SUCCESS_GREEN),
            Text("| 다중, ..= 범위, .. 나머지", font_size=24, color=MEMORY_BLUE),
            Text("if 가드, @ 바인딩 = 강력함", font_size=24, color=WARNING_YELLOW),
        ).arrange(DOWN, buff=0.4)

        VGroup(title, summary).arrange(DOWN, buff=0.8)

        self.play(Write(title))
        for line in summary:
            self.play(FadeIn(line, shift=UP), run_time=0.4)

        self.wait(1)

        next_ep = Text("다음: EP 7-02 OOP in Rust", font_size=28, color=GRAY)
        next_ep.to_edge(DOWN, buff=1)
        self.play(FadeIn(next_ep))
        self.wait(2)


class EP_7_02_OOPRust(Scene):
    """
    EP 7-02: OOP in Rust - Rust 방식의 객체지향

    학습 목표:
    1. Rust에서의 캡슐화
    2. 트레이트 객체로 다형성
    3. 상태 패턴 구현
    4. OOP vs Rust 철학
    """

    def construct(self):
        self.camera.background_color = DARK_BG

        self.section_intro()
        self.wait(0.5)
        self.clear()

        self.section_encapsulation()
        self.wait(0.5)
        self.clear()

        self.section_trait_objects()
        self.wait(0.5)
        self.clear()

        self.section_state_pattern()
        self.wait(0.5)
        self.clear()

        self.section_rust_way()
        self.wait(0.5)
        self.clear()

        self.section_outro()

    def section_intro(self):
        title = Text("EP 7-02", font_size=32, color=GRAY).to_edge(UP)
        main_title = Text("OOP in Rust", font_size=56, color=RUST_ORANGE)
        subtitle = Text("Rust 방식의 객체지향", font_size=28, color=GRAY)

        main_title.next_to(title, DOWN, buff=0.8)
        subtitle.next_to(main_title, DOWN, buff=0.3)

        self.play(Write(title), Write(main_title))
        self.play(FadeIn(subtitle))
        self.wait(1)

        # OOP 3대 요소
        oop_elements = VGroup(
            Text("OOP의 3대 요소", font_size=28, color=OOP_INDIGO),
            VGroup(
                self.create_oop_item("캡슐화", "✅", "pub/private으로 구현"),
                self.create_oop_item("상속", "❌", "트레이트로 대체"),
                self.create_oop_item("다형성", "✅", "트레이트 객체로 구현"),
            ).arrange(DOWN, buff=0.3, aligned_edge=LEFT),
        ).arrange(DOWN, buff=0.5)
        oop_elements.next_to(subtitle, DOWN, buff=0.8)

        self.play(FadeIn(oop_elements, shift=UP))
        self.wait(2)

    def create_oop_item(self, name, support, rust_way):
        return VGroup(
            Text(support, font_size=24),
            Text(name, font_size=20, color=WHITE),
            Text(f"→ {rust_way}", font_size=14, color=GRAY),
        ).arrange(RIGHT, buff=0.3)

    def section_encapsulation(self):
        title = Text("캡슐화: pub과 private", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''pub struct AveragedCollection {
    list: Vec<i32>,     // private: 외부 접근 불가
    average: f64,       // private
}

impl AveragedCollection {
    pub fn new() -> Self {
        Self { list: vec![], average: 0.0 }
    }

    pub fn add(&mut self, value: i32) {
        self.list.push(value);
        self.update_average();
    }

    pub fn average(&self) -> f64 {
        self.average
    }

    fn update_average(&mut self) {  // private 메서드
        let sum: i32 = self.list.iter().sum();
        self.average = sum as f64 / self.list.len() as f64;
    }
}''',
            language="rust",
            font_size=12,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.7)
        code.next_to(title, DOWN, buff=0.3)

        self.play(Create(code))
        self.wait(1)

        # 핵심
        key = VGroup(
            Text("내부 구현 숨기고, 공개 API만 노출", font_size=20, color=SUCCESS_GREEN),
            Text("= 캡슐화 완벽 지원!", font_size=18, color=GRAY),
        ).arrange(DOWN, buff=0.1)
        key.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(key, shift=UP))
        self.wait(2)

    def section_trait_objects(self):
        title = Text("트레이트 객체: 다형성", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''pub trait Draw {
    fn draw(&self);
}

pub struct Screen {
    // dyn Draw = 런타임에 타입 결정 (동적 디스패치)
    pub components: Vec<Box<dyn Draw>>,
}

impl Screen {
    pub fn run(&self) {
        for component in self.components.iter() {
            component.draw();  // 어떤 타입이든 draw() 호출
        }
    }
}

// 다양한 구현
struct Button { label: String }
struct TextField { text: String }

impl Draw for Button { fn draw(&self) { /* ... */ } }
impl Draw for TextField { fn draw(&self) { /* ... */ } }

// 사용: 서로 다른 타입을 하나의 Vec에!
let screen = Screen {
    components: vec![
        Box::new(Button { label: "OK".into() }),
        Box::new(TextField { text: "Hello".into() }),
    ],
};''',
            language="rust",
            font_size=11,
            background="rectangle",
            background_stroke_color=OOP_INDIGO,
        ).scale(0.68)
        code.next_to(title, DOWN, buff=0.2)

        self.play(Create(code))
        self.wait(2)

    def section_state_pattern(self):
        title = Text("상태 패턴 구현", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # 상태 다이어그램
        states = VGroup(
            self.create_state_node("Draft", GRAY),
            self.create_state_node("PendingReview", WARNING_YELLOW),
            self.create_state_node("Published", SUCCESS_GREEN),
        ).arrange(RIGHT, buff=1.5)
        states.next_to(title, DOWN, buff=0.5)

        arrows = VGroup(
            Arrow(states[0].get_right(), states[1].get_left(), color=WHITE, buff=0.1),
            Arrow(states[1].get_right(), states[2].get_left(), color=WHITE, buff=0.1),
        )

        labels = VGroup(
            Text("request_review()", font_size=10, color=GRAY).next_to(arrows[0], UP, buff=0.05),
            Text("approve()", font_size=10, color=GRAY).next_to(arrows[1], UP, buff=0.05),
        )

        self.play(Create(states), Create(arrows), Write(labels))
        self.wait(1)

        # 코드
        code = Code(
            code='''trait State {
    fn request_review(self: Box<Self>) -> Box<dyn State>;
    fn approve(self: Box<Self>) -> Box<dyn State>;
    fn content<'a>(&self, _post: &'a Post) -> &'a str { "" }
}

struct Draft;
impl State for Draft {
    fn request_review(self: Box<Self>) -> Box<dyn State> {
        Box::new(PendingReview)
    }
    fn approve(self: Box<Self>) -> Box<dyn State> { self }
}

struct PendingReview;
impl State for PendingReview {
    fn request_review(self: Box<Self>) -> Box<dyn State> { self }
    fn approve(self: Box<Self>) -> Box<dyn State> {
        Box::new(Published)
    }
}''',
            language="rust",
            font_size=10,
            background="rectangle",
        ).scale(0.7)
        code.to_edge(DOWN, buff=0.3)

        self.play(Create(code))
        self.wait(2)

    def create_state_node(self, name, color):
        circle = Circle(radius=0.5, fill_color=color, fill_opacity=0.3, stroke_color=color)
        label = Text(name, font_size=12, color=WHITE)
        label.move_to(circle)
        return VGroup(circle, label)

    def section_rust_way(self):
        title = Text("Rust의 철학: 상속 없이 조합", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # 상속 vs 조합
        comparison = VGroup(
            VGroup(
                Text("전통 OOP (상속)", font_size=22, color=ERROR_RED),
                Code(
                    code='''class Animal { ... }
class Dog extends Animal { ... }
class Cat extends Animal { ... }
// 다이아몬드 문제, 깊은 계층''',
                    language="java",
                    font_size=11,
                ).scale(0.8),
            ).arrange(DOWN, buff=0.2),
            VGroup(
                Text("Rust (조합)", font_size=22, color=SUCCESS_GREEN),
                Code(
                    code='''trait Speak { fn speak(&self); }
trait Walk { fn walk(&self); }

struct Dog;
impl Speak for Dog { ... }
impl Walk for Dog { ... }
// 필요한 트레이트만 구현!''',
                    language="rust",
                    font_size=11,
                ).scale(0.8),
            ).arrange(DOWN, buff=0.2),
        ).arrange(RIGHT, buff=0.8)
        comparison.next_to(title, DOWN, buff=0.4)

        self.play(FadeIn(comparison))
        self.wait(1)

        # 핵심 메시지
        message = VGroup(
            Text("\"상속보다 조합을 선호하라\"", font_size=24, color=RUST_ORANGE),
            Text("- GoF Design Patterns (1994)", font_size=16, color=GRAY),
            Text("Rust는 이 원칙을 언어 레벨에서 강제!", font_size=18, color=WHITE),
        ).arrange(DOWN, buff=0.2)
        message.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(message, shift=UP))
        self.wait(2)

    def section_outro(self):
        title = Text("EP 7-02 정리", font_size=44, color=RUST_ORANGE)

        summary = VGroup(
            Text("캡슐화 = pub/private", font_size=24, color=SUCCESS_GREEN),
            Text("다형성 = dyn Trait (동적 디스패치)", font_size=24, color=OOP_INDIGO),
            Text("상속 없음 = 트레이트 조합으로 대체", font_size=24, color=WARNING_YELLOW),
            Text("상태 패턴도 트레이트로 구현 가능", font_size=24, color=MEMORY_BLUE),
        ).arrange(DOWN, buff=0.4)

        VGroup(title, summary).arrange(DOWN, buff=0.8)

        self.play(Write(title))
        for line in summary:
            self.play(FadeIn(line, shift=UP), run_time=0.4)

        self.wait(1)

        # Level 7 완료
        complete = Text("🎉 Level 7: 고급 패턴 완료!", font_size=32, color=SUCCESS_GREEN)
        next_level = Text("다음: Level 8 - Unsafe & 매크로", font_size=24, color=GRAY)

        VGroup(complete, next_level).arrange(DOWN, buff=0.3).to_edge(DOWN, buff=0.8)

        self.play(FadeIn(complete, scale=1.2))
        self.play(FadeIn(next_level))
        self.wait(2)


# 렌더링 헬퍼
if __name__ == "__main__":
    print("""
╔═══════════════════════════════════════════════════════════════╗
║          WIA-RUST-LEARN Level 7: 고급 패턴                    ║
╠═══════════════════════════════════════════════════════════════╣
║                                                               ║
║  EP 7-01: Pattern Matching Deep Dive                          ║
║    manim -pql level_7_patterns.py EP_7_01_PatternDeep         ║
║                                                               ║
║  EP 7-02: OOP in Rust                                         ║
║    manim -pql level_7_patterns.py EP_7_02_OOPRust             ║
║                                                               ║
║  전체 렌더링:                                                  ║
║    manim -pqh level_7_patterns.py --all                       ║
║                                                               ║
╚═══════════════════════════════════════════════════════════════╝
    """)
