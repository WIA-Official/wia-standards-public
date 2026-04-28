"""
WIA-RUST-LEARN Level 4: 고급 기능 (Advanced)
============================================

EP 4-01: Generics & Traits (제네릭과 트레이트) - 22분
EP 4-02: Closures (클로저) - 18분
EP 4-03: Smart Pointers (스마트 포인터) - 20분

실행 방법:
    manim -pql level_4_advanced.py EP_4_01_GenericsTrajts
    manim -pql level_4_advanced.py EP_4_02_Closures
    manim -pql level_4_advanced.py EP_4_03_SmartPointers

전체 렌더링 (고화질):
    manim -pqh level_4_advanced.py --all
"""

from manim import *
import numpy as np

# WIA-RUST-LEARN 테마 색상
RUST_ORANGE = "#F74C00"
RUST_BROWN = "#8B4513"
MEMORY_BLUE = "#4A90D9"
STACK_GREEN = "#2ECC71"
HEAP_PURPLE = "#9B59B6"
ERROR_RED = "#E74C3C"
SUCCESS_GREEN = "#27AE60"
WARNING_YELLOW = "#F39C12"
TRAIT_CYAN = "#00BCD4"
DARK_BG = "#1a1a2e"


class EP_4_01_GenericsTraits(Scene):
    """
    EP 4-01: Generics & Traits - 다형성의 Rust 방식

    학습 목표:
    1. 제네릭으로 코드 재사용
    2. 트레이트로 공통 동작 정의
    3. 트레이트 바운드로 제약 조건
    4. 제네릭 vs 동적 디스패치
    """

    def construct(self):
        self.camera.background_color = DARK_BG

        self.section_intro()
        self.wait(0.5)
        self.clear()

        self.section_generics_basics()
        self.wait(0.5)
        self.clear()

        self.section_trait_definition()
        self.wait(0.5)
        self.clear()

        self.section_trait_bounds()
        self.wait(0.5)
        self.clear()

        self.section_monomorphization()
        self.wait(0.5)
        self.clear()

        self.section_dyn_trait()
        self.wait(0.5)
        self.clear()

        self.section_outro()

    def section_intro(self):
        title = Text("EP 4-01", font_size=32, color=GRAY).to_edge(UP)
        main_title = Text("Generics & Traits", font_size=56, color=RUST_ORANGE)
        subtitle = Text("다형성의 Rust 방식", font_size=28, color=GRAY)

        main_title.next_to(title, DOWN, buff=0.8)
        subtitle.next_to(main_title, DOWN, buff=0.3)

        self.play(Write(title), Write(main_title))
        self.play(FadeIn(subtitle))
        self.wait(1)

        # 핵심 개념
        concepts = VGroup(
            VGroup(
                Text("Generics", font_size=28, color=MEMORY_BLUE),
                Text("타입을 매개변수화", font_size=18, color=GRAY),
                Text("Vec<T>, Option<T>", font_size=16, color=WHITE),
            ).arrange(DOWN, buff=0.1),
            Text("+", font_size=36, color=WHITE),
            VGroup(
                Text("Traits", font_size=28, color=TRAIT_CYAN),
                Text("공통 동작 정의", font_size=18, color=GRAY),
                Text("Display, Clone, Iterator", font_size=16, color=WHITE),
            ).arrange(DOWN, buff=0.1),
            Text("=", font_size=36, color=WHITE),
            VGroup(
                Text("Zero-cost Abstraction", font_size=28, color=RUST_ORANGE),
                Text("런타임 비용 없이 추상화", font_size=18, color=GRAY),
            ).arrange(DOWN, buff=0.1),
        ).arrange(RIGHT, buff=0.5)
        concepts.next_to(subtitle, DOWN, buff=1)

        self.play(FadeIn(concepts, shift=UP))
        self.wait(2)

    def section_generics_basics(self):
        title = Text("제네릭: 타입 매개변수", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # 문제 상황
        problem = Code(
            code='''// 중복 코드 문제
fn largest_i32(list: &[i32]) -> i32 { ... }
fn largest_f64(list: &[f64]) -> f64 { ... }
fn largest_char(list: &[char]) -> char { ... }''',
            language="rust",
            font_size=16,
            background="rectangle",
        ).scale(0.8)
        problem.next_to(title, DOWN, buff=0.5)

        problem_label = Text("❌ 타입마다 함수 작성?", font_size=20, color=ERROR_RED)
        problem_label.next_to(problem, LEFT, buff=0.3)

        self.play(Create(problem), Write(problem_label))
        self.wait(1)

        # 해결책
        solution = Code(
            code='''// 제네릭으로 해결!
fn largest<T: PartialOrd>(list: &[T]) -> &T {
    let mut largest = &list[0];
    for item in list {
        if item > largest {
            largest = item;
        }
    }
    largest
}

// 모든 타입에 사용 가능
largest(&[1, 2, 3]);       // i32
largest(&[1.0, 2.0, 3.0]); // f64
largest(&['a', 'b', 'c']); // char''',
            language="rust",
            font_size=14,
            background="rectangle",
            background_stroke_color=SUCCESS_GREEN,
        ).scale(0.75)
        solution.next_to(problem, DOWN, buff=0.5)

        self.play(Create(solution))
        self.wait(1)

        # T 설명
        t_explain = VGroup(
            Text("<T>", font_size=32, color=MEMORY_BLUE),
            Text("= 어떤 타입이든 올 수 있음", font_size=18, color=WHITE),
        ).arrange(RIGHT, buff=0.3)
        t_explain.to_edge(DOWN, buff=0.5)

        self.play(Write(t_explain))
        self.wait(2)

    def section_trait_definition(self):
        title = Text("트레이트: 공통 동작 정의", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''// 트레이트 정의
trait Summary {
    fn summarize(&self) -> String;

    // 기본 구현 제공 가능
    fn summarize_author(&self) -> String {
        String::from("(unknown)")
    }
}

// 구조체에 구현
struct Article { title: String, content: String }

impl Summary for Article {
    fn summarize(&self) -> String {
        format!("{}: {}", self.title, &self.content[..50])
    }
}

// 사용
let article = Article { ... };
println!("{}", article.summarize());''',
            language="rust",
            font_size=14,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.7)
        code.to_edge(LEFT, buff=0.3)

        self.play(Create(code))
        self.wait(1)

        # 트레이트 다이어그램
        diagram = VGroup()

        # Summary 트레이트
        trait_box = RoundedRectangle(
            width=3, height=1.5, corner_radius=0.2,
            fill_color=TRAIT_CYAN, fill_opacity=0.2,
            stroke_color=TRAIT_CYAN
        )
        trait_label = Text("Summary", font_size=20, color=TRAIT_CYAN)
        trait_methods = VGroup(
            Text("summarize()", font_size=12, color=WHITE),
            Text("summarize_author()", font_size=12, color=GRAY),
        ).arrange(DOWN, buff=0.1)
        trait_content = VGroup(trait_label, trait_methods).arrange(DOWN, buff=0.2)
        trait_content.move_to(trait_box)

        trait_group = VGroup(trait_box, trait_content)

        # 구현하는 타입들
        article_box = RoundedRectangle(
            width=2, height=0.8, corner_radius=0.1,
            fill_color=STACK_GREEN, fill_opacity=0.2,
        )
        article_label = Text("Article", font_size=14, color=STACK_GREEN)
        article_label.move_to(article_box)

        tweet_box = RoundedRectangle(
            width=2, height=0.8, corner_radius=0.1,
            fill_color=MEMORY_BLUE, fill_opacity=0.2,
        )
        tweet_label = Text("Tweet", font_size=14, color=MEMORY_BLUE)
        tweet_label.move_to(tweet_box)

        article_g = VGroup(article_box, article_label)
        tweet_g = VGroup(tweet_box, tweet_label)

        impls = VGroup(article_g, tweet_g).arrange(RIGHT, buff=0.5)

        trait_group.to_edge(RIGHT, buff=1)
        impls.next_to(trait_group, DOWN, buff=1)

        # 화살표
        arrow1 = Arrow(article_g.get_top(), trait_group.get_bottom(), color=GRAY, buff=0.1)
        arrow2 = Arrow(tweet_g.get_top(), trait_group.get_bottom(), color=GRAY, buff=0.1)

        impl_label = Text("impl Summary for", font_size=12, color=GRAY)
        impl_label.next_to(arrow1, LEFT, buff=0.1)

        self.play(
            Create(trait_group),
            Create(impls),
            Create(arrow1), Create(arrow2),
            Write(impl_label)
        )
        self.wait(2)

    def section_trait_bounds(self):
        title = Text("트레이트 바운드", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''// 방법 1: 트레이트 바운드 문법
fn notify<T: Summary>(item: &T) {
    println!("속보! {}", item.summarize());
}

// 방법 2: impl Trait 문법 (간단)
fn notify(item: &impl Summary) {
    println!("속보! {}", item.summarize());
}

// 여러 트레이트 요구
fn process<T: Summary + Display>(item: &T) { ... }

// where 절 (복잡한 경우)
fn complex<T, U>(t: &T, u: &U) -> i32
where
    T: Display + Clone,
    U: Clone + Debug,
{ ... }''',
            language="rust",
            font_size=15,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.7)
        code.next_to(title, DOWN, buff=0.4)

        self.play(Create(code))
        self.wait(1)

        # 시각화
        constraint = VGroup(
            Text("T: Summary", font_size=28, color=TRAIT_CYAN),
            Text("=", font_size=24, color=WHITE),
            Text("T는 Summary를 구현해야 함", font_size=20, color=GRAY),
        ).arrange(RIGHT, buff=0.3)
        constraint.to_edge(DOWN, buff=0.8)

        self.play(Write(constraint))
        self.wait(2)

    def section_monomorphization(self):
        title = Text("단형화 (Monomorphization)", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # 컴파일 전
        before = VGroup(
            Text("소스 코드", font_size=20, color=GRAY),
            Code(
                code='''fn print<T: Display>(val: T) {
    println!("{}", val);
}

print(5);       // i32
print("hello"); // &str''',
                language="rust",
                font_size=14,
            ).scale(0.7),
        ).arrange(DOWN, buff=0.2)

        # 컴파일 후
        after = VGroup(
            Text("컴파일된 코드", font_size=20, color=GRAY),
            Code(
                code='''fn print_i32(val: i32) {
    println!("{}", val);
}

fn print_str(val: &str) {
    println!("{}", val);
}

print_i32(5);
print_str("hello");''',
                language="rust",
                font_size=14,
            ).scale(0.7),
        ).arrange(DOWN, buff=0.2)

        before.to_edge(LEFT, buff=1)
        after.to_edge(RIGHT, buff=1)

        compile_arrow = Arrow(before.get_right(), after.get_left(), color=RUST_ORANGE)
        compile_label = Text("컴파일", font_size=18, color=RUST_ORANGE)
        compile_label.next_to(compile_arrow, UP, buff=0.1)

        self.play(Create(before))
        self.wait(0.5)
        self.play(Create(compile_arrow), Write(compile_label))
        self.play(Create(after))
        self.wait(1)

        # Zero-cost
        zero_cost = VGroup(
            Text("Zero-cost Abstraction", font_size=28, color=SUCCESS_GREEN),
            Text("제네릭 사용해도 런타임 비용 없음!", font_size=18, color=GRAY),
            Text("(각 타입별로 최적화된 코드 생성)", font_size=14, color=GRAY),
        ).arrange(DOWN, buff=0.2)
        zero_cost.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(zero_cost, shift=UP))
        self.wait(2)

    def section_dyn_trait(self):
        title = Text("동적 디스패치: dyn Trait", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # 비교
        comparison = VGroup(
            VGroup(
                Text("Static Dispatch", font_size=24, color=STACK_GREEN),
                Text("impl Trait / <T: Trait>", font_size=16, color=WHITE),
                Text("• 컴파일 타임 결정", font_size=14, color=GRAY),
                Text("• 인라인 가능", font_size=14, color=GRAY),
                Text("• 바이너리 크기 증가", font_size=14, color=GRAY),
            ).arrange(DOWN, buff=0.15),
            VGroup(
                Text("Dynamic Dispatch", font_size=24, color=HEAP_PURPLE),
                Text("dyn Trait", font_size=16, color=WHITE),
                Text("• 런타임 결정", font_size=14, color=GRAY),
                Text("• vtable 사용", font_size=14, color=GRAY),
                Text("• 바이너리 크기 작음", font_size=14, color=GRAY),
            ).arrange(DOWN, buff=0.15),
        ).arrange(RIGHT, buff=1.5)
        comparison.next_to(title, DOWN, buff=0.5)

        self.play(FadeIn(comparison))
        self.wait(1)

        # dyn 예시
        code = Code(
            code='''// 다른 타입을 같은 컬렉션에
fn get_drawables() -> Vec<Box<dyn Draw>> {
    vec![
        Box::new(Circle { radius: 1.0 }),
        Box::new(Rectangle { w: 2.0, h: 3.0 }),
        Box::new(Triangle { ... }),
    ]
}

// 런타임에 적절한 메서드 호출
for drawable in get_drawables() {
    drawable.draw();  // vtable 통해 호출
}''',
            language="rust",
            font_size=14,
            background="rectangle",
            background_stroke_color=HEAP_PURPLE,
        ).scale(0.75)
        code.to_edge(DOWN, buff=0.5)

        self.play(Create(code))
        self.wait(2)

    def section_outro(self):
        title = Text("EP 4-01 정리", font_size=44, color=RUST_ORANGE)

        summary = VGroup(
            Text("Generics = 타입 매개변수화", font_size=24, color=MEMORY_BLUE),
            Text("Trait = 공통 동작 정의", font_size=24, color=TRAIT_CYAN),
            Text("Bound = 제네릭에 제약 조건", font_size=24, color=WARNING_YELLOW),
            Text("Monomorphization = Zero-cost", font_size=24, color=SUCCESS_GREEN),
        ).arrange(DOWN, buff=0.4)

        VGroup(title, summary).arrange(DOWN, buff=0.8)

        self.play(Write(title))
        for line in summary:
            self.play(FadeIn(line, shift=UP), run_time=0.4)

        self.wait(1)

        next_ep = Text("다음: EP 4-02 Closures", font_size=28, color=GRAY)
        next_ep.to_edge(DOWN, buff=1)
        self.play(FadeIn(next_ep))
        self.wait(2)


class EP_4_02_Closures(Scene):
    """
    EP 4-02: Closures - 환경을 캡처하는 익명 함수

    학습 목표:
    1. 클로저 문법 이해
    2. 환경 캡처 방식 (Fn, FnMut, FnOnce)
    3. move 키워드 사용
    4. 이터레이터와 함께 사용
    """

    def construct(self):
        self.camera.background_color = DARK_BG

        self.section_intro()
        self.wait(0.5)
        self.clear()

        self.section_syntax()
        self.wait(0.5)
        self.clear()

        self.section_capture_modes()
        self.wait(0.5)
        self.clear()

        self.section_move_closure()
        self.wait(0.5)
        self.clear()

        self.section_with_iterators()
        self.wait(0.5)
        self.clear()

        self.section_outro()

    def section_intro(self):
        title = Text("EP 4-02", font_size=32, color=GRAY).to_edge(UP)
        main_title = Text("Closures", font_size=56, color=RUST_ORANGE)
        subtitle = Text("환경을 캡처하는 익명 함수", font_size=28, color=GRAY)

        main_title.next_to(title, DOWN, buff=0.8)
        subtitle.next_to(main_title, DOWN, buff=0.3)

        self.play(Write(title), Write(main_title))
        self.play(FadeIn(subtitle))
        self.wait(1)

        # 클로저 = 익명 함수 + 환경 캡처
        formula = VGroup(
            Text("Closure", font_size=32, color=RUST_ORANGE),
            Text("=", font_size=28, color=WHITE),
            Text("익명 함수", font_size=24, color=MEMORY_BLUE),
            Text("+", font_size=28, color=WHITE),
            Text("환경 캡처", font_size=24, color=HEAP_PURPLE),
        ).arrange(RIGHT, buff=0.3)
        formula.next_to(subtitle, DOWN, buff=1)

        self.play(Write(formula))
        self.wait(2)

    def section_syntax(self):
        title = Text("클로저 문법", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''// 기본 문법
let add = |a, b| a + b;
let result = add(1, 2);  // 3

// 타입 명시
let add: fn(i32, i32) -> i32 = |a, b| a + b;

// 여러 줄
let complex = |x| {
    let doubled = x * 2;
    doubled + 1
};

// 환경 캡처
let factor = 10;
let multiply = |x| x * factor;  // factor 캡처!
println!("{}", multiply(5));     // 50''',
            language="rust",
            font_size=16,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.75)
        code.next_to(title, DOWN, buff=0.4)

        self.play(Create(code))
        self.wait(1)

        # 함수 vs 클로저
        comparison = VGroup(
            VGroup(
                Text("fn", font_size=24, color=GRAY),
                Text("fn add(a: i32, b: i32) -> i32", font_size=14, color=WHITE),
            ).arrange(DOWN, buff=0.1),
            VGroup(
                Text("closure", font_size=24, color=RUST_ORANGE),
                Text("|a, b| a + b", font_size=14, color=WHITE),
            ).arrange(DOWN, buff=0.1),
        ).arrange(RIGHT, buff=2)
        comparison.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(comparison, shift=UP))
        self.wait(2)

    def section_capture_modes(self):
        title = Text("세 가지 캡처 방식", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # 세 가지 트레이트
        traits = VGroup(
            self.create_trait_box("Fn", "불변 참조", "&self", STACK_GREEN,
                                  ["읽기만 함", "여러 번 호출 가능"]),
            self.create_trait_box("FnMut", "가변 참조", "&mut self", WARNING_YELLOW,
                                  ["수정 가능", "여러 번 호출 가능"]),
            self.create_trait_box("FnOnce", "소유권 이전", "self", ERROR_RED,
                                  ["소비함", "한 번만 호출 가능"]),
        ).arrange(RIGHT, buff=0.5)
        traits.next_to(title, DOWN, buff=0.5)

        for trait in traits:
            self.play(FadeIn(trait, shift=UP), run_time=0.5)

        self.wait(1)

        # 예시 코드
        code = Code(
            code='''let s = String::from("hello");

// Fn: 읽기만
let print_s = || println!("{}", s);
print_s(); print_s();  // ✓ 여러 번 가능

// FnMut: 수정
let mut count = 0;
let mut inc = || count += 1;
inc(); inc();  // count = 2

// FnOnce: 소비
let consume = || drop(s);
consume();
// consume(); // ✗ 두 번째 호출 불가!''',
            language="rust",
            font_size=14,
            background="rectangle",
        ).scale(0.7)
        code.to_edge(DOWN, buff=0.3)

        self.play(Create(code))
        self.wait(2)

    def create_trait_box(self, name, capture, self_type, color, features):
        box_rect = RoundedRectangle(
            width=3.5, height=2.5, corner_radius=0.2,
            fill_color=color, fill_opacity=0.1,
            stroke_color=color
        )

        content = VGroup(
            Text(name, font_size=28, color=color),
            Text(capture, font_size=16, color=WHITE),
            Text(self_type, font_size=14, color=GRAY),
            VGroup(*[Text(f"• {f}", font_size=12, color=GRAY) for f in features]).arrange(DOWN, aligned_edge=LEFT, buff=0.05),
        ).arrange(DOWN, buff=0.15)
        content.move_to(box_rect)

        return VGroup(box_rect, content)

    def section_move_closure(self):
        title = Text("move 클로저", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # 문제 상황
        problem = Code(
            code='''// 문제: 스레드에 데이터 전달
let data = vec![1, 2, 3];

std::thread::spawn(|| {
    println!("{:?}", data);  // ✗ 빌려올 수 없음!
});
// data가 먼저 drop될 수 있음''',
            language="rust",
            font_size=16,
            background="rectangle",
        ).scale(0.8)
        problem.next_to(title, DOWN, buff=0.4)

        problem_label = Text("❌ 컴파일 에러!", font_size=20, color=ERROR_RED)
        problem_label.next_to(problem, RIGHT, buff=0.3)

        self.play(Create(problem), Write(problem_label))
        self.wait(1)

        # 해결책
        solution = Code(
            code='''// 해결: move로 소유권 이전
let data = vec![1, 2, 3];

std::thread::spawn(move || {
    println!("{:?}", data);  // ✓ 소유권 이전됨
});
// data는 더 이상 사용 불가''',
            language="rust",
            font_size=16,
            background="rectangle",
            background_stroke_color=SUCCESS_GREEN,
        ).scale(0.8)
        solution.next_to(problem, DOWN, buff=0.5)

        solution_label = Text("✓ move 키워드 추가", font_size=20, color=SUCCESS_GREEN)
        solution_label.next_to(solution, RIGHT, buff=0.3)

        self.play(Create(solution), Write(solution_label))
        self.wait(1)

        # 소유권 이동 시각화
        viz = VGroup(
            VGroup(
                Circle(radius=0.3, fill_color=STACK_GREEN, fill_opacity=0.5),
                Text("data", font_size=14, color=WHITE),
            ),
            Arrow(ORIGIN, RIGHT * 2, color=RUST_ORANGE),
            VGroup(
                Circle(radius=0.3, fill_color=HEAP_PURPLE, fill_opacity=0.5),
                Text("closure", font_size=14, color=WHITE),
            ),
        ).arrange(RIGHT, buff=0.3)

        move_label = Text("move", font_size=18, color=RUST_ORANGE)
        move_label.next_to(viz[1], UP, buff=0.1)

        viz_group = VGroup(viz, move_label)
        viz_group.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(viz_group, shift=UP))
        self.wait(2)

    def section_with_iterators(self):
        title = Text("이터레이터와 클로저", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''let numbers = vec![1, 2, 3, 4, 5];

// map: 변환
let doubled: Vec<_> = numbers.iter()
    .map(|x| x * 2)
    .collect();  // [2, 4, 6, 8, 10]

// filter: 필터링
let evens: Vec<_> = numbers.iter()
    .filter(|x| *x % 2 == 0)
    .collect();  // [2, 4]

// 체이닝
let result: i32 = numbers.iter()
    .filter(|x| *x > 2)      // [3, 4, 5]
    .map(|x| x * 10)         // [30, 40, 50]
    .sum();                  // 120

// fold: 누적
let sum = numbers.iter()
    .fold(0, |acc, x| acc + x);  // 15''',
            language="rust",
            font_size=14,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.7)
        code.next_to(title, DOWN, buff=0.3)

        self.play(Create(code))
        self.wait(1)

        # 파이프라인 시각화
        pipeline = VGroup(
            self.create_stage("[1,2,3,4,5]", STACK_GREEN),
            Arrow(ORIGIN, RIGHT * 0.8, color=WHITE),
            self.create_stage("filter", WARNING_YELLOW),
            Arrow(ORIGIN, RIGHT * 0.8, color=WHITE),
            self.create_stage("map", MEMORY_BLUE),
            Arrow(ORIGIN, RIGHT * 0.8, color=WHITE),
            self.create_stage("sum", HEAP_PURPLE),
            Arrow(ORIGIN, RIGHT * 0.8, color=WHITE),
            self.create_stage("120", SUCCESS_GREEN),
        ).arrange(RIGHT, buff=0.1)
        pipeline.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(pipeline, shift=UP))
        self.wait(2)

    def create_stage(self, text, color):
        box = RoundedRectangle(
            width=1.2, height=0.6, corner_radius=0.1,
            fill_color=color, fill_opacity=0.3,
            stroke_color=color
        )
        label = Text(text, font_size=12, color=WHITE)
        label.move_to(box)
        return VGroup(box, label)

    def section_outro(self):
        title = Text("EP 4-02 정리", font_size=44, color=RUST_ORANGE)

        summary = VGroup(
            Text("|args| expr = 클로저 문법", font_size=24, color=MEMORY_BLUE),
            Text("Fn/FnMut/FnOnce = 캡처 방식", font_size=24, color=TRAIT_CYAN),
            Text("move = 소유권 강제 이전", font_size=24, color=WARNING_YELLOW),
            Text("이터레이터 + 클로저 = 강력!", font_size=24, color=SUCCESS_GREEN),
        ).arrange(DOWN, buff=0.4)

        VGroup(title, summary).arrange(DOWN, buff=0.8)

        self.play(Write(title))
        for line in summary:
            self.play(FadeIn(line, shift=UP), run_time=0.4)

        self.wait(1)

        next_ep = Text("다음: EP 4-03 Smart Pointers", font_size=28, color=GRAY)
        next_ep.to_edge(DOWN, buff=1)
        self.play(FadeIn(next_ep))
        self.wait(2)


class EP_4_03_SmartPointers(Scene):
    """
    EP 4-03: Smart Pointers - 소유권을 넘어서

    학습 목표:
    1. Box<T>: 힙 할당
    2. Rc<T>: 참조 카운팅
    3. RefCell<T>: 내부 가변성
    4. 약한 참조와 순환 방지
    """

    def construct(self):
        self.camera.background_color = DARK_BG

        self.section_intro()
        self.wait(0.5)
        self.clear()

        self.section_box()
        self.wait(0.5)
        self.clear()

        self.section_rc()
        self.wait(0.5)
        self.clear()

        self.section_refcell()
        self.wait(0.5)
        self.clear()

        self.section_combined()
        self.wait(0.5)
        self.clear()

        self.section_outro()

    def section_intro(self):
        title = Text("EP 4-03", font_size=32, color=GRAY).to_edge(UP)
        main_title = Text("Smart Pointers", font_size=56, color=RUST_ORANGE)
        subtitle = Text("소유권을 넘어서", font_size=28, color=GRAY)

        main_title.next_to(title, DOWN, buff=0.8)
        subtitle.next_to(main_title, DOWN, buff=0.3)

        self.play(Write(title), Write(main_title))
        self.play(FadeIn(subtitle))
        self.wait(1)

        # 스마트 포인터란?
        definition = VGroup(
            Text("스마트 포인터 = 포인터 + 메타데이터 + 자동 관리", font_size=22, color=WHITE),
        )
        definition.next_to(subtitle, DOWN, buff=0.8)

        self.play(Write(definition))
        self.wait(1)

        # 주요 스마트 포인터
        pointers = VGroup(
            VGroup(
                Text("Box<T>", font_size=28, color=STACK_GREEN),
                Text("힙 할당", font_size=16, color=GRAY),
            ).arrange(DOWN, buff=0.1),
            VGroup(
                Text("Rc<T>", font_size=28, color=MEMORY_BLUE),
                Text("참조 카운팅", font_size=16, color=GRAY),
            ).arrange(DOWN, buff=0.1),
            VGroup(
                Text("RefCell<T>", font_size=28, color=WARNING_YELLOW),
                Text("내부 가변성", font_size=16, color=GRAY),
            ).arrange(DOWN, buff=0.1),
            VGroup(
                Text("Arc<T>", font_size=28, color=HEAP_PURPLE),
                Text("스레드 안전 Rc", font_size=16, color=GRAY),
            ).arrange(DOWN, buff=0.1),
        ).arrange(RIGHT, buff=1)
        pointers.next_to(definition, DOWN, buff=0.8)

        self.play(FadeIn(pointers, shift=UP))
        self.wait(2)

    def section_box(self):
        title = Text("Box<T>: 힙 할당", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''// 기본 사용
let b = Box::new(5);
println!("b = {}", b);  // 자동 역참조

// 재귀 타입에 필수
enum List {
    Cons(i32, Box<List>),
    Nil,
}

let list = Cons(1, Box::new(Cons(2, Box::new(Nil))));

// 큰 데이터 힙으로 이동
let big_array = Box::new([0u8; 1_000_000]);''',
            language="rust",
            font_size=16,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.75)
        code.next_to(title, DOWN, buff=0.4).to_edge(LEFT, buff=0.5)

        self.play(Create(code))
        self.wait(1)

        # Stack / Heap 시각화
        stack = self.create_memory_region("Stack", STACK_GREEN, [
            ("b (ptr)", "→"),
        ])
        heap = self.create_memory_region("Heap", HEAP_PURPLE, [
            ("5", "i32"),
        ])

        stack.to_edge(RIGHT, buff=2).shift(UP)
        heap.next_to(stack, DOWN, buff=0.5)

        arrow = Arrow(
            stack.get_center() + DOWN * 0.3,
            heap.get_center() + UP * 0.3,
            color=MEMORY_BLUE
        )

        self.play(Create(stack), Create(heap), Create(arrow))
        self.wait(2)

    def create_memory_region(self, label, color, items):
        box = RoundedRectangle(
            width=2.5, height=len(items) * 0.6 + 0.4,
            corner_radius=0.1,
            fill_color=color, fill_opacity=0.1,
            stroke_color=color
        )
        title = Text(label, font_size=16, color=color)
        title.next_to(box, UP, buff=0.1)

        cells = VGroup(*[
            VGroup(
                Text(name, font_size=12, color=WHITE),
                Text(val, font_size=12, color=GRAY),
            ).arrange(RIGHT, buff=0.3)
            for name, val in items
        ]).arrange(DOWN, buff=0.1)
        cells.move_to(box)

        return VGroup(box, title, cells)

    def section_rc(self):
        title = Text("Rc<T>: 참조 카운팅", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''use std::rc::Rc;

let data = Rc::new(vec![1, 2, 3]);
println!("count: {}", Rc::strong_count(&data)); // 1

let a = Rc::clone(&data);  // 카운트 증가
let b = Rc::clone(&data);
println!("count: {}", Rc::strong_count(&data)); // 3

drop(a);  // 카운트 감소
println!("count: {}", Rc::strong_count(&data)); // 2

// data와 b가 같은 Vec을 공유!''',
            language="rust",
            font_size=16,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.7)
        code.next_to(title, DOWN, buff=0.3).to_edge(LEFT, buff=0.5)

        self.play(Create(code))
        self.wait(1)

        # 참조 카운팅 시각화
        data_box = RoundedRectangle(
            width=2, height=1.5, corner_radius=0.1,
            fill_color=HEAP_PURPLE, fill_opacity=0.3,
            stroke_color=HEAP_PURPLE
        )
        data_label = VGroup(
            Text("[1, 2, 3]", font_size=16, color=WHITE),
            Text("count: 3", font_size=14, color=MEMORY_BLUE),
        ).arrange(DOWN, buff=0.1)
        data_label.move_to(data_box)

        refs = VGroup(
            Circle(radius=0.3, fill_color=STACK_GREEN, fill_opacity=0.5),
            Circle(radius=0.3, fill_color=STACK_GREEN, fill_opacity=0.5),
            Circle(radius=0.3, fill_color=STACK_GREEN, fill_opacity=0.5),
        ).arrange(DOWN, buff=0.3)

        ref_labels = VGroup(
            Text("data", font_size=12, color=WHITE),
            Text("a", font_size=12, color=WHITE),
            Text("b", font_size=12, color=WHITE),
        )
        for ref, label in zip(refs, ref_labels):
            label.move_to(ref)

        data_group = VGroup(data_box, data_label)
        refs_group = VGroup(refs, ref_labels)

        data_group.to_edge(RIGHT, buff=1.5)
        refs_group.next_to(data_group, LEFT, buff=1)

        arrows = VGroup(*[
            Arrow(refs[i].get_right(), data_box.get_left(), color=MEMORY_BLUE, buff=0.1)
            for i in range(3)
        ])

        self.play(Create(data_group), Create(refs_group), Create(arrows))
        self.wait(2)

    def section_refcell(self):
        title = Text("RefCell<T>: 내부 가변성", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # 문제 상황
        problem = VGroup(
            Text("문제:", font_size=20, color=ERROR_RED),
            Text("불변 참조에서 내부 값을 수정하고 싶다면?", font_size=18, color=GRAY),
        ).arrange(RIGHT, buff=0.3)
        problem.next_to(title, DOWN, buff=0.4)

        self.play(Write(problem))
        self.wait(0.5)

        code = Code(
            code='''use std::cell::RefCell;

let data = RefCell::new(5);

// 불변 대여 (런타임 검사)
let borrowed = data.borrow();
println!("{}", borrowed);  // 5
drop(borrowed);

// 가변 대여 (런타임 검사)
*data.borrow_mut() += 1;
println!("{}", data.borrow());  // 6

// ⚠️ 런타임 패닉 가능!
let a = data.borrow();
let b = data.borrow_mut();  // 💥 panic!''',
            language="rust",
            font_size=14,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.7)
        code.next_to(problem, DOWN, buff=0.3)

        self.play(Create(code))
        self.wait(1)

        # 컴파일 vs 런타임 검사
        comparison = VGroup(
            VGroup(
                Text("&T / &mut T", font_size=20, color=STACK_GREEN),
                Text("컴파일 타임 검사", font_size=14, color=GRAY),
            ).arrange(DOWN, buff=0.1),
            VGroup(
                Text("RefCell<T>", font_size=20, color=WARNING_YELLOW),
                Text("런타임 검사", font_size=14, color=GRAY),
            ).arrange(DOWN, buff=0.1),
        ).arrange(RIGHT, buff=2)
        comparison.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(comparison, shift=UP))
        self.wait(2)

    def section_combined(self):
        title = Text("Rc<RefCell<T>>: 공유 + 가변", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''use std::rc::Rc;
use std::cell::RefCell;

// 여러 소유자가 공유하면서 수정도 가능
let shared = Rc::new(RefCell::new(vec![1, 2, 3]));

let a = Rc::clone(&shared);
let b = Rc::clone(&shared);

// a를 통해 수정
a.borrow_mut().push(4);

// b에서도 변경 확인
println!("{:?}", b.borrow());  // [1, 2, 3, 4]''',
            language="rust",
            font_size=16,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.75)
        code.next_to(title, DOWN, buff=0.4)

        self.play(Create(code))
        self.wait(1)

        # 주의사항
        warning = VGroup(
            Text("⚠️ 주의", font_size=24, color=WARNING_YELLOW),
            Text("• 순환 참조 → 메모리 누수", font_size=16, color=GRAY),
            Text("• Weak<T>로 해결", font_size=16, color=GRAY),
            Text("• 스레드 간 공유: Arc<Mutex<T>>", font_size=16, color=GRAY),
        ).arrange(DOWN, buff=0.2, aligned_edge=LEFT)
        warning.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(warning, shift=UP))
        self.wait(2)

    def section_outro(self):
        title = Text("EP 4-03 정리", font_size=44, color=RUST_ORANGE)

        summary = VGroup(
            Text("Box<T> = 힙 할당, 단일 소유", font_size=24, color=STACK_GREEN),
            Text("Rc<T> = 참조 카운팅, 공유 소유", font_size=24, color=MEMORY_BLUE),
            Text("RefCell<T> = 내부 가변성", font_size=24, color=WARNING_YELLOW),
            Text("Rc<RefCell<T>> = 공유 + 가변", font_size=24, color=HEAP_PURPLE),
        ).arrange(DOWN, buff=0.4)

        VGroup(title, summary).arrange(DOWN, buff=0.8)

        self.play(Write(title))
        for line in summary:
            self.play(FadeIn(line, shift=UP), run_time=0.4)

        self.wait(1)

        # Level 4 완료
        complete = Text("🎉 Level 4: 고급 기능 완료!", font_size=32, color=SUCCESS_GREEN)
        next_level = Text("다음: Level 5 - 동시성", font_size=24, color=GRAY)

        VGroup(complete, next_level).arrange(DOWN, buff=0.3).to_edge(DOWN, buff=0.8)

        self.play(FadeIn(complete, scale=1.2))
        self.play(FadeIn(next_level))
        self.wait(2)


# 렌더링 헬퍼
if __name__ == "__main__":
    print("""
╔═══════════════════════════════════════════════════════════════╗
║          WIA-RUST-LEARN Level 4: 고급 기능                    ║
╠═══════════════════════════════════════════════════════════════╣
║                                                               ║
║  EP 4-01: Generics & Traits                                   ║
║    manim -pql level_4_advanced.py EP_4_01_GenericsTraits      ║
║                                                               ║
║  EP 4-02: Closures                                            ║
║    manim -pql level_4_advanced.py EP_4_02_Closures            ║
║                                                               ║
║  EP 4-03: Smart Pointers                                      ║
║    manim -pql level_4_advanced.py EP_4_03_SmartPointers       ║
║                                                               ║
║  전체 렌더링:                                                  ║
║    manim -pqh level_4_advanced.py --all                       ║
║                                                               ║
╚═══════════════════════════════════════════════════════════════╝
    """)
