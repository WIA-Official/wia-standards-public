"""
WIA-RUST-LEARN: Level 1 - Rust Basics
=====================================
EP 1-01: Variables & Types (22분)
EP 1-02: Functions & Control Flow (25분)
EP 1-03: String vs &str (20분)
EP 1-04: Collections & Iterators (23분)

Run: manim -pqh level_1_basics.py EP_1_01_Variables
"""

from manim import *

# Colors
RUST_ORANGE = "#f74c00"
STACK_BLUE = "#3498db"
HEAP_PURPLE = "#9b59b6"
VALID_GREEN = "#00d26a"
INVALID_RED = "#ff4757"
TEXT_WHITE = "#e6e6e6"
CODE_BG = "#1e1e1e"


class EP_1_01_Variables(Scene):
    """EP 1-01: Variables & Types - Immutability by Default"""

    def construct(self):
        self.intro()
        self.section_immutability()
        self.section_types()
        self.section_shadowing()
        self.outro()

    def intro(self):
        title = Text("EP 1-01: Variables & Types", font_size=48, color=RUST_ORANGE)
        subtitle = Text("Immutability by Default", font_size=32, color=TEXT_WHITE, opacity=0.7)
        subtitle.next_to(title, DOWN, buff=0.5)

        self.play(Write(title), FadeIn(subtitle))
        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_immutability(self):
        title = Text("불변성이 기본!", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        # Immutable example
        code1 = self.create_code_block(
            'let x = 5;\nx = 6;  // ERROR!',
            error=True
        )
        code1.next_to(title, DOWN, buff=0.8)
        code1.shift(LEFT * 3)

        error_text = Text("cannot assign twice\nto immutable variable",
                         font_size=18, color=INVALID_RED)
        error_text.next_to(code1, DOWN, buff=0.3)

        # Mutable example
        code2 = self.create_code_block(
            'let mut x = 5;\nx = 6;  // OK!',
            success=True
        )
        code2.next_to(title, DOWN, buff=0.8)
        code2.shift(RIGHT * 3)

        ok_text = Text("mut 키워드로\n가변 선언", font_size=18, color=VALID_GREEN)
        ok_text.next_to(code2, DOWN, buff=0.3)

        self.play(Create(code1), Create(code2))
        self.play(Write(error_text), Write(ok_text))

        # Why immutable by default?
        why = Text("왜 불변이 기본인가?", font_size=32, color=RUST_ORANGE)
        why.to_edge(DOWN, buff=2)

        reasons = VGroup(
            Text("• 버그 감소", font_size=22),
            Text("• 동시성에서 안전", font_size=22),
            Text("• 컴파일러 최적화", font_size=22),
        ).arrange(RIGHT, buff=1)
        reasons.next_to(why, DOWN, buff=0.3)

        self.play(Write(why))
        self.play(Write(reasons))
        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def create_code_block(self, code, error=False, success=False):
        border_color = INVALID_RED if error else VALID_GREEN if success else TEXT_WHITE
        box = Rectangle(width=6, height=2.5, color=border_color, fill_color=CODE_BG, fill_opacity=0.9)
        text = Text(code, font_size=20, font="monospace", color=TEXT_WHITE)
        text.move_to(box)
        return VGroup(box, text)

    def section_types(self):
        title = Text("기본 타입", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        types_table = VGroup(
            self.create_type_row("i32, i64", "정수 (signed)", "-2^31 ~ 2^31-1"),
            self.create_type_row("u32, u64", "정수 (unsigned)", "0 ~ 2^32-1"),
            self.create_type_row("f32, f64", "부동소수점", "IEEE 754"),
            self.create_type_row("bool", "불리언", "true / false"),
            self.create_type_row("char", "유니코드 문자", "'a', '한', '🦀'"),
            self.create_type_row("()", "유닛 타입", "값 없음"),
        ).arrange(DOWN, buff=0.3)
        types_table.next_to(title, DOWN, buff=0.8)

        for row in types_table:
            self.play(FadeIn(row), run_time=0.4)

        # Type inference
        inference = Text("Rust는 대부분 타입을 추론합니다!", font_size=28, color=VALID_GREEN)
        inference.to_edge(DOWN, buff=1)
        self.play(Write(inference))

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def create_type_row(self, type_name, desc, example):
        return VGroup(
            Text(type_name, font_size=22, color=RUST_ORANGE, font="monospace"),
            Text(desc, font_size=20),
            Text(example, font_size=18, color=TEXT_WHITE, opacity=0.7),
        ).arrange(RIGHT, buff=1)

    def section_shadowing(self):
        title = Text("Shadowing", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        code = '''let x = 5;
let x = x + 1;      // x = 6 (새로운 변수)
let x = x * 2;      // x = 12

// 타입도 변경 가능!
let spaces = "   ";
let spaces = spaces.len();  // 3 (i32)'''

        code_block = Text(code, font_size=22, font="monospace", color=TEXT_WHITE)
        code_block.next_to(title, DOWN, buff=0.8)
        self.play(Write(code_block), run_time=2)

        note = Text("mut와 다름: 매번 새로운 변수 생성", font_size=24, color=STACK_BLUE)
        note.to_edge(DOWN, buff=1)
        self.play(Write(note))

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def outro(self):
        summary = VGroup(
            Text("✓ 변수는 기본적으로 불변", font_size=28),
            Text("✓ mut로 가변 선언", font_size=28),
            Text("✓ 강력한 타입 시스템 + 타입 추론", font_size=28),
            Text("✓ Shadowing으로 유연한 재선언", font_size=28),
        ).arrange(DOWN, buff=0.4, aligned_edge=LEFT)

        self.play(Write(summary))

        next_ep = Text("다음: EP 1-02 Functions & Control Flow", font_size=24, color=RUST_ORANGE)
        next_ep.to_edge(DOWN, buff=1)
        self.play(FadeIn(next_ep))
        self.wait(2)


class EP_1_02_Functions(Scene):
    """EP 1-02: Functions & Control Flow"""

    def construct(self):
        self.intro()
        self.section_functions()
        self.section_expressions()
        self.section_control_flow()
        self.outro()

    def intro(self):
        title = Text("EP 1-02: Functions & Control Flow", font_size=44, color=RUST_ORANGE)
        self.play(Write(title))
        self.wait(1)
        self.play(FadeOut(title))

    def section_functions(self):
        title = Text("함수 정의", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        code = '''fn add(a: i32, b: i32) -> i32 {
    a + b   // 세미콜론 없음 = 반환값
}

fn greet(name: &str) {
    println!("Hello, {}!", name);
}

fn main() {
    let sum = add(3, 5);    // 8
    greet("Rust");          // Hello, Rust!
}'''

        code_text = Text(code, font_size=20, font="monospace", color=TEXT_WHITE)
        code_text.next_to(title, DOWN, buff=0.5)
        self.play(Write(code_text), run_time=2)

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_expressions(self):
        title = Text("표현식 vs 문장", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        # Key insight
        insight = Text("Rust에서 거의 모든 것은 표현식!", font_size=32, color=VALID_GREEN)
        insight.next_to(title, DOWN, buff=0.5)
        self.play(Write(insight))

        code = '''// if도 표현식!
let number = if condition { 5 } else { 6 };

// 블록도 표현식!
let y = {
    let x = 3;
    x + 1   // 세미콜론 없음 = 이 값이 반환
};  // y = 4'''

        code_text = Text(code, font_size=20, font="monospace", color=TEXT_WHITE)
        code_text.next_to(insight, DOWN, buff=0.5)
        self.play(Write(code_text), run_time=2)

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_control_flow(self):
        title = Text("제어 흐름", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        # Loop types
        loops = VGroup(
            Text("loop { ... }  // 무한 루프", font_size=22, font="monospace"),
            Text("while cond { ... }", font_size=22, font="monospace"),
            Text("for item in iter { ... }", font_size=22, font="monospace"),
        ).arrange(DOWN, buff=0.4, aligned_edge=LEFT)
        loops.next_to(title, DOWN, buff=0.8)

        self.play(Write(loops))

        # For loop example
        for_example = '''for i in 0..5 {     // 0, 1, 2, 3, 4
    println!("{}", i);
}

for ch in "hello".chars() {
    println!("{}", ch);  // h, e, l, l, o
}'''

        for_text = Text(for_example, font_size=18, font="monospace", color=TEXT_WHITE)
        for_text.next_to(loops, DOWN, buff=0.5)
        self.play(Write(for_text), run_time=1.5)

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def outro(self):
        summary = Text("함수와 제어 흐름 마스터!", font_size=48, color=VALID_GREEN)
        next_ep = Text("다음: EP 1-03 String vs &str", font_size=28, color=RUST_ORANGE)
        next_ep.next_to(summary, DOWN, buff=1)

        self.play(Write(summary), FadeIn(next_ep))
        self.wait(2)


class EP_1_03_Strings(Scene):
    """EP 1-03: String vs &str - Your First Ownership Preview"""

    def construct(self):
        self.intro()
        self.section_two_types()
        self.section_memory()
        self.section_conversion()
        self.outro()

    def intro(self):
        title = Text("EP 1-03: String vs &str", font_size=48, color=RUST_ORANGE)
        subtitle = Text("Your First Ownership Preview", font_size=28, opacity=0.7)
        subtitle.next_to(title, DOWN, buff=0.5)

        self.play(Write(title), FadeIn(subtitle))
        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_two_types(self):
        title = Text("왜 두 가지 문자열 타입?", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        # String
        string_box = Rectangle(width=5, height=3, color=HEAP_PURPLE, fill_opacity=0.2)
        string_box.shift(LEFT * 3)
        string_title = Text("String", font_size=32, color=HEAP_PURPLE)
        string_title.next_to(string_box, UP)
        string_desc = VGroup(
            Text("• 힙에 저장", font_size=20),
            Text("• 소유권 있음", font_size=20),
            Text("• 크기 변경 가능", font_size=20),
            Text("• UTF-8", font_size=20),
        ).arrange(DOWN, buff=0.2, aligned_edge=LEFT)
        string_desc.move_to(string_box)

        # &str
        str_box = Rectangle(width=5, height=3, color=STACK_BLUE, fill_opacity=0.2)
        str_box.shift(RIGHT * 3)
        str_title = Text("&str", font_size=32, color=STACK_BLUE)
        str_title.next_to(str_box, UP)
        str_desc = VGroup(
            Text("• 참조 (슬라이스)", font_size=20),
            Text("• 빌림 (borrow)", font_size=20),
            Text("• 불변", font_size=20),
            Text("• UTF-8", font_size=20),
        ).arrange(DOWN, buff=0.2, aligned_edge=LEFT)
        str_desc.move_to(str_box)

        self.play(
            Create(string_box), Write(string_title), Write(string_desc),
            Create(str_box), Write(str_title), Write(str_desc)
        )

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_memory(self):
        title = Text("메모리 구조", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        # Stack
        stack = Rectangle(width=4, height=4, color=STACK_BLUE)
        stack.shift(LEFT * 3 + DOWN * 0.5)
        stack_label = Text("STACK", font_size=24, color=STACK_BLUE)
        stack_label.next_to(stack, UP)

        # Heap
        heap = Rectangle(width=4, height=4, color=HEAP_PURPLE)
        heap.shift(RIGHT * 3 + DOWN * 0.5)
        heap_label = Text("HEAP", font_size=24, color=HEAP_PURPLE)
        heap_label.next_to(heap, UP)

        self.play(Create(stack), Write(stack_label), Create(heap), Write(heap_label))

        # String on stack (pointer)
        s_ptr = Rectangle(width=3, height=0.8, color=HEAP_PURPLE, fill_opacity=0.3)
        s_ptr.move_to(stack.get_center() + UP)
        s_text = Text('s: String', font_size=16, color=HEAP_PURPLE)
        s_text.move_to(s_ptr)

        # String data on heap
        s_data = Rectangle(width=3, height=0.8, color=HEAP_PURPLE, fill_opacity=0.3)
        s_data.move_to(heap.get_center() + UP)
        s_data_text = Text('"hello"', font_size=16)
        s_data_text.move_to(s_data)

        # Arrow
        arrow = Arrow(s_ptr.get_right(), s_data.get_left(), color=HEAP_PURPLE, buff=0.1)

        self.play(Create(s_ptr), Write(s_text), Create(s_data), Write(s_data_text), Create(arrow))

        # &str on stack
        str_ref = Rectangle(width=3, height=0.8, color=STACK_BLUE, fill_opacity=0.3)
        str_ref.move_to(stack.get_center() + DOWN)
        str_text = Text('r: &str', font_size=16, color=STACK_BLUE)
        str_text.move_to(str_ref)

        # &str points to same data
        arrow2 = DashedLine(str_ref.get_right(), s_data.get_left() + DOWN * 0.2, color=STACK_BLUE)

        borrow_label = Text("borrows", font_size=14, color=STACK_BLUE)
        borrow_label.next_to(arrow2, DOWN, buff=0.1)

        self.play(Create(str_ref), Write(str_text), Create(arrow2), Write(borrow_label))

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_conversion(self):
        title = Text("상호 변환", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        code = '''// &str → String
let s: String = "hello".to_string();
let s: String = String::from("hello");

// String → &str
let s: String = String::from("hello");
let r: &str = &s;           // 자동 역참조
let r: &str = s.as_str();   // 명시적'''

        code_text = Text(code, font_size=20, font="monospace", color=TEXT_WHITE)
        code_text.next_to(title, DOWN, buff=0.5)
        self.play(Write(code_text), run_time=2)

        tip = Text("💡 함수 매개변수는 &str를 받는 것이 유연!", font_size=24, color=VALID_GREEN)
        tip.to_edge(DOWN, buff=1)
        self.play(Write(tip))

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def outro(self):
        preview = Text("소유권 시스템의 첫 맛보기!", font_size=40, color=RUST_ORANGE)
        next_ep = Text("다음: EP 1-04 Collections & Iterators", font_size=24, opacity=0.7)
        next_ep.next_to(preview, DOWN, buff=1)

        self.play(Write(preview), FadeIn(next_ep))
        self.wait(2)


class EP_1_04_Collections(Scene):
    """EP 1-04: Collections - Vec, HashMap, and Iterators"""

    def construct(self):
        self.intro()
        self.section_vec()
        self.section_hashmap()
        self.section_iterators()
        self.outro()

    def intro(self):
        title = Text("EP 1-04: Collections", font_size=48, color=RUST_ORANGE)
        subtitle = Text("Vec, HashMap, and Iterators", font_size=28, opacity=0.7)
        subtitle.next_to(title, DOWN, buff=0.5)

        self.play(Write(title), FadeIn(subtitle))
        self.wait(1)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_vec(self):
        title = Text("Vec<T> - 동적 배열", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        code = '''let mut v: Vec<i32> = Vec::new();
v.push(1);
v.push(2);
v.push(3);

// 매크로로 생성
let v = vec![1, 2, 3];

// 접근
let first = v[0];           // 패닉 가능
let first = v.get(0);       // Option<&i32>'''

        code_text = Text(code, font_size=20, font="monospace", color=TEXT_WHITE)
        code_text.next_to(title, DOWN, buff=0.5)
        self.play(Write(code_text), run_time=2)

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_hashmap(self):
        title = Text("HashMap<K, V>", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        code = '''use std::collections::HashMap;

let mut scores = HashMap::new();
scores.insert("Blue", 10);
scores.insert("Yellow", 50);

// 조회
let score = scores.get("Blue");  // Some(&10)

// 없으면 삽입
scores.entry("Red").or_insert(25);'''

        code_text = Text(code, font_size=20, font="monospace", color=TEXT_WHITE)
        code_text.next_to(title, DOWN, buff=0.5)
        self.play(Write(code_text), run_time=2)

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_iterators(self):
        title = Text("Iterator 체이닝", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        code = '''let numbers = vec![1, 2, 3, 4, 5];

let result: Vec<i32> = numbers
    .iter()             // 이터레이터 생성
    .map(|x| x * 2)     // 각 요소 2배
    .filter(|x| *x > 4) // 4 초과만
    .collect();         // Vec로 수집

// result = [6, 8, 10]'''

        code_text = Text(code, font_size=20, font="monospace", color=TEXT_WHITE)
        code_text.next_to(title, DOWN, buff=0.5)
        self.play(Write(code_text), run_time=2)

        tip = Text("💡 이터레이터는 게으르게(lazy) 평가됩니다", font_size=24, color=VALID_GREEN)
        tip.to_edge(DOWN, buff=1)
        self.play(Write(tip))

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def outro(self):
        done = Text("Level 1 완료!", font_size=56, color=VALID_GREEN)

        next_level = Text("다음: Level 2 - Ownership (핵심!)", font_size=32, color=RUST_ORANGE)
        next_level.next_to(done, DOWN, buff=1)

        self.play(Write(done), FadeIn(next_level))
        self.wait(3)
