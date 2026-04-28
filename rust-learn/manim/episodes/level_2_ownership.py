"""
WIA-RUST-LEARN: Level 2 - Ownership (The Heart of Rust)
========================================================
EP 2-01: Ownership Rules (30분)
EP 2-02: Move, Copy, Clone (28분)
EP 2-03: References & Borrowing (32분)
EP 2-04: Lifetimes (30분)

Run: manim -pqh level_2_ownership.py EP_2_01_OwnershipRules
"""

from manim import *

# Colors
RUST_ORANGE = "#f74c00"
STACK_BLUE = "#3498db"
HEAP_PURPLE = "#9b59b6"
VALID_GREEN = "#00d26a"
INVALID_RED = "#ff4757"
TEXT_WHITE = "#e6e6e6"


class EP_2_01_OwnershipRules(Scene):
    """EP 2-01: Ownership Rules - The Three Laws"""

    def construct(self):
        self.intro()
        self.section_three_rules()
        self.section_stack_heap()
        self.section_ownership_demo()
        self.section_nrt_concept()
        self.outro()

    def intro(self):
        # NRT Concept Introduction
        nrt = Text("NRT: Non-Repeatable Transfer", font_size=40, color=RUST_ORANGE)
        meaning = Text("소유권은 NFT처럼 단 하나만 존재한다", font_size=28, color=TEXT_WHITE)
        meaning.next_to(nrt, DOWN, buff=0.5)

        self.play(Write(nrt))
        self.play(FadeIn(meaning))
        self.wait(2)

        title = Text("EP 2-01: Ownership Rules", font_size=48, color=RUST_ORANGE)
        title.move_to(nrt.get_center())

        self.play(Transform(nrt, title), FadeOut(meaning))
        self.wait(1)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_three_rules(self):
        title = Text("소유권의 3가지 규칙", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        rules = VGroup(
            VGroup(
                Text("1", font_size=48, color=RUST_ORANGE),
                Text("각 값은 하나의 소유자(owner)만 가진다", font_size=28),
            ).arrange(RIGHT, buff=0.5),
            VGroup(
                Text("2", font_size=48, color=RUST_ORANGE),
                Text("한 번에 하나의 소유자만 존재할 수 있다", font_size=28),
            ).arrange(RIGHT, buff=0.5),
            VGroup(
                Text("3", font_size=48, color=RUST_ORANGE),
                Text("소유자가 스코프를 벗어나면 값이 해제된다", font_size=28),
            ).arrange(RIGHT, buff=0.5),
        ).arrange(DOWN, buff=0.8, aligned_edge=LEFT)
        rules.next_to(title, DOWN, buff=1)

        for rule in rules:
            self.play(Write(rule), run_time=1)
            self.wait(0.5)

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_stack_heap(self):
        title = Text("Stack vs Heap", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        # Stack
        stack_rect = Rectangle(width=5, height=4, color=STACK_BLUE)
        stack_rect.shift(LEFT * 3)
        stack_label = Text("STACK", font_size=28, color=STACK_BLUE)
        stack_label.next_to(stack_rect, UP)

        stack_props = VGroup(
            Text("• 고정 크기", font_size=20),
            Text("• 빠른 할당/해제", font_size=20),
            Text("• LIFO 순서", font_size=20),
            Text("• i32, f64, bool...", font_size=20),
        ).arrange(DOWN, buff=0.2, aligned_edge=LEFT)
        stack_props.move_to(stack_rect)

        # Heap
        heap_rect = Rectangle(width=5, height=4, color=HEAP_PURPLE)
        heap_rect.shift(RIGHT * 3)
        heap_label = Text("HEAP", font_size=28, color=HEAP_PURPLE)
        heap_label.next_to(heap_rect, UP)

        heap_props = VGroup(
            Text("• 동적 크기", font_size=20),
            Text("• 느린 할당", font_size=20),
            Text("• 포인터로 접근", font_size=20),
            Text("• String, Vec...", font_size=20),
        ).arrange(DOWN, buff=0.2, aligned_edge=LEFT)
        heap_props.move_to(heap_rect)

        self.play(
            Create(stack_rect), Write(stack_label), Write(stack_props),
            Create(heap_rect), Write(heap_label), Write(heap_props)
        )

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_ownership_demo(self):
        title = Text("소유권 이동 시각화", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        # Memory layout
        stack = Rectangle(width=4, height=5, color=STACK_BLUE)
        stack.shift(LEFT * 3 + DOWN * 0.5)
        stack_label = Text("STACK", font_size=20, color=STACK_BLUE)
        stack_label.next_to(stack, UP)

        heap = Rectangle(width=4, height=5, color=HEAP_PURPLE)
        heap.shift(RIGHT * 3 + DOWN * 0.5)
        heap_label = Text("HEAP", font_size=20, color=HEAP_PURPLE)
        heap_label.next_to(heap, UP)

        self.play(Create(stack), Write(stack_label), Create(heap), Write(heap_label))

        # s1 = String::from("hello")
        code1 = Text('let s1 = String::from("hello");', font_size=18,
                    font="monospace", color=VALID_GREEN)
        code1.to_edge(DOWN)
        self.play(Write(code1))

        s1_box = Rectangle(width=3, height=0.7, color=VALID_GREEN, fill_opacity=0.3)
        s1_box.move_to(stack.get_center() + UP * 1.5)
        s1_text = Text("s1", font_size=18, color=VALID_GREEN)
        s1_text.move_to(s1_box)

        hello_box = Rectangle(width=3, height=0.7, color=HEAP_PURPLE, fill_opacity=0.3)
        hello_box.move_to(heap.get_center() + UP * 1.5)
        hello_text = Text('"hello"', font_size=18)
        hello_text.move_to(hello_box)

        arrow1 = Arrow(s1_box.get_right(), hello_box.get_left(), color=VALID_GREEN, buff=0.1)

        self.play(Create(s1_box), Write(s1_text), Create(hello_box), Write(hello_text), Create(arrow1))
        self.wait(1)

        # s2 = s1 (MOVE!)
        self.play(FadeOut(code1))
        code2 = Text('let s2 = s1;  // MOVE!', font_size=18, font="monospace", color=RUST_ORANGE)
        code2.to_edge(DOWN)
        self.play(Write(code2))

        s2_box = Rectangle(width=3, height=0.7, color=VALID_GREEN, fill_opacity=0.3)
        s2_box.move_to(stack.get_center())
        s2_text = Text("s2", font_size=18, color=VALID_GREEN)
        s2_text.move_to(s2_box)

        arrow2 = Arrow(s2_box.get_right(), hello_box.get_left(), color=VALID_GREEN, buff=0.1)

        # Invalidate s1
        self.play(
            Create(s2_box), Write(s2_text), Create(arrow2),
            s1_box.animate.set_color(INVALID_RED).set_fill(INVALID_RED, opacity=0.2),
            s1_text.animate.set_color(INVALID_RED),
            FadeOut(arrow1)
        )

        cross = Cross(VGroup(s1_box, s1_text), color=INVALID_RED)
        self.play(Create(cross))

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_nrt_concept(self):
        title = Text("NRT: Non-Repeatable Transfer", font_size=44, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        comparison = VGroup(
            VGroup(
                Text("NFT", font_size=36, color=STACK_BLUE),
                Text("디지털 자산의 유일한 소유권", font_size=22),
            ).arrange(DOWN, buff=0.3),
            Text("=", font_size=48, color=RUST_ORANGE),
            VGroup(
                Text("Rust Ownership", font_size=36, color=HEAP_PURPLE),
                Text("메모리 데이터의 유일한 소유권", font_size=22),
            ).arrange(DOWN, buff=0.3),
        ).arrange(RIGHT, buff=1)
        comparison.next_to(title, DOWN, buff=1)

        self.play(Write(comparison), run_time=2)

        key_point = Text(
            "한 번에 하나의 소유자만!\n복제가 아닌 이동(transfer)!",
            font_size=28, color=VALID_GREEN
        )
        key_point.next_to(comparison, DOWN, buff=1)
        self.play(Write(key_point))

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def outro(self):
        summary = Text("소유권 = 컴파일 타임 메모리 안전", font_size=40, color=RUST_ORANGE)
        next_ep = Text("다음: EP 2-02 Move, Copy, Clone", font_size=28, opacity=0.7)
        next_ep.next_to(summary, DOWN, buff=1)

        self.play(Write(summary), FadeIn(next_ep))
        self.wait(2)


class EP_2_02_MoveCopyClone(Scene):
    """EP 2-02: Move, Copy, Clone - Data Transfer Semantics"""

    def construct(self):
        self.intro()
        self.section_move()
        self.section_copy()
        self.section_clone()
        self.section_comparison()
        self.outro()

    def intro(self):
        title = Text("EP 2-02: Move, Copy, Clone", font_size=48, color=RUST_ORANGE)
        self.play(Write(title))
        self.wait(1)
        self.play(FadeOut(title))

    def section_move(self):
        title = Text("Move (기본 동작)", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        code = '''let s1 = String::from("hello");
let s2 = s1;    // s1의 소유권이 s2로 이동

// s1은 더 이상 사용 불가!
println!("{}", s1);  // ERROR!'''

        code_text = Text(code, font_size=22, font="monospace", color=TEXT_WHITE)
        code_text.next_to(title, DOWN, buff=0.5)
        self.play(Write(code_text), run_time=1.5)

        explanation = VGroup(
            Text("• Heap 데이터는 복사되지 않음", font_size=22),
            Text("• 포인터만 이동", font_size=22),
            Text("• 효율적! O(1)", font_size=22),
        ).arrange(DOWN, buff=0.2, aligned_edge=LEFT)
        explanation.next_to(code_text, DOWN, buff=0.5)

        self.play(Write(explanation))
        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_copy(self):
        title = Text("Copy (스택 데이터)", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        code = '''let x = 5;
let y = x;    // x가 복사됨 (Copy trait)

println!("{}, {}", x, y);  // 둘 다 OK!'''

        code_text = Text(code, font_size=22, font="monospace", color=TEXT_WHITE)
        code_text.next_to(title, DOWN, buff=0.5)
        self.play(Write(code_text))

        copy_types = VGroup(
            Text("Copy를 구현하는 타입:", font_size=24, color=VALID_GREEN),
            Text("i32, u32, f64, bool, char", font_size=22, font="monospace"),
            Text("(i32, i32) - Copy 요소만 있는 튜플", font_size=22, font="monospace"),
            Text("[i32; 3] - Copy 요소 배열", font_size=22, font="monospace"),
        ).arrange(DOWN, buff=0.3, aligned_edge=LEFT)
        copy_types.next_to(code_text, DOWN, buff=0.5)

        self.play(Write(copy_types))
        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_clone(self):
        title = Text("Clone (명시적 깊은 복사)", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        code = '''let s1 = String::from("hello");
let s2 = s1.clone();    // 명시적 깊은 복사

println!("{}, {}", s1, s2);  // 둘 다 OK!'''

        code_text = Text(code, font_size=22, font="monospace", color=TEXT_WHITE)
        code_text.next_to(title, DOWN, buff=0.5)
        self.play(Write(code_text))

        # Visual: Two separate heap allocations
        heap1 = Rectangle(width=2.5, height=1, color=HEAP_PURPLE, fill_opacity=0.3)
        heap1.shift(LEFT * 2 + DOWN * 1.5)
        text1 = Text('s1: "hello"', font_size=18)
        text1.move_to(heap1)

        heap2 = Rectangle(width=2.5, height=1, color=HEAP_PURPLE, fill_opacity=0.3)
        heap2.shift(RIGHT * 2 + DOWN * 1.5)
        text2 = Text('s2: "hello"', font_size=18)
        text2.move_to(heap2)

        arrow = Arrow(heap1.get_right(), heap2.get_left(), color=VALID_GREEN, buff=0.2)
        arrow_label = Text("clone()", font_size=16, color=VALID_GREEN)
        arrow_label.next_to(arrow, UP, buff=0.1)

        self.play(Create(heap1), Write(text1))
        self.play(Create(arrow), Write(arrow_label))
        self.play(Create(heap2), Write(text2))

        note = Text("⚠️ 비용 발생: Heap 할당 + 데이터 복사", font_size=22, color=RUST_ORANGE)
        note.to_edge(DOWN, buff=0.5)
        self.play(Write(note))

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_comparison(self):
        title = Text("Move vs Copy vs Clone", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        table = VGroup(
            VGroup(
                Text("Move", font_size=28, color=INVALID_RED),
                Text("포인터 이동", font_size=20),
                Text("원본 무효화", font_size=20),
                Text("O(1)", font_size=20),
            ).arrange(DOWN, buff=0.2),
            VGroup(
                Text("Copy", font_size=28, color=STACK_BLUE),
                Text("비트 복사", font_size=20),
                Text("둘 다 유효", font_size=20),
                Text("O(1)", font_size=20),
            ).arrange(DOWN, buff=0.2),
            VGroup(
                Text("Clone", font_size=28, color=HEAP_PURPLE),
                Text("깊은 복사", font_size=20),
                Text("둘 다 유효", font_size=20),
                Text("O(n)", font_size=20),
            ).arrange(DOWN, buff=0.2),
        ).arrange(RIGHT, buff=1.5)
        table.next_to(title, DOWN, buff=1)

        self.play(Write(table), run_time=2)
        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def outro(self):
        summary = Text("데이터 이동의 3가지 방식 마스터!", font_size=40, color=VALID_GREEN)
        next_ep = Text("다음: EP 2-03 References & Borrowing", font_size=28, opacity=0.7)
        next_ep.next_to(summary, DOWN, buff=1)

        self.play(Write(summary), FadeIn(next_ep))
        self.wait(2)


class EP_2_03_Borrowing(Scene):
    """EP 2-03: References & Borrowing"""

    def construct(self):
        self.intro()
        self.section_immutable_ref()
        self.section_mutable_ref()
        self.section_rules()
        self.outro()

    def intro(self):
        title = Text("EP 2-03: References & Borrowing", font_size=44, color=RUST_ORANGE)
        subtitle = Text("소유권 없이 접근하기", font_size=28, opacity=0.7)
        subtitle.next_to(title, DOWN, buff=0.5)

        self.play(Write(title), FadeIn(subtitle))
        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_immutable_ref(self):
        title = Text("불변 참조 (&T)", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        code = '''let s1 = String::from("hello");
let r1 = &s1;    // 불변 참조
let r2 = &s1;    // 여러 개 OK!

println!("{}, {}, {}", s1, r1, r2);  // 모두 OK!'''

        code_text = Text(code, font_size=22, font="monospace", color=TEXT_WHITE)
        code_text.next_to(title, DOWN, buff=0.5)
        self.play(Write(code_text), run_time=1.5)

        # Visual
        owner = Rectangle(width=3, height=1, color=VALID_GREEN, fill_opacity=0.3)
        owner.shift(UP * 0 + LEFT * 3)
        owner_text = Text("s1 (owner)", font_size=18, color=VALID_GREEN)
        owner_text.move_to(owner)

        ref1 = Rectangle(width=2, height=0.8, color=STACK_BLUE, fill_opacity=0.3)
        ref1.shift(RIGHT * 2 + UP * 0.5)
        ref1_text = Text("r1: &s1", font_size=16, color=STACK_BLUE)
        ref1_text.move_to(ref1)

        ref2 = Rectangle(width=2, height=0.8, color=STACK_BLUE, fill_opacity=0.3)
        ref2.shift(RIGHT * 2 + DOWN * 0.5)
        ref2_text = Text("r2: &s1", font_size=16, color=STACK_BLUE)
        ref2_text.move_to(ref2)

        arrow1 = Arrow(ref1.get_left(), owner.get_right(), color=STACK_BLUE, buff=0.1)
        arrow2 = Arrow(ref2.get_left(), owner.get_right(), color=STACK_BLUE, buff=0.1)

        all_refs = VGroup(owner, owner_text, ref1, ref1_text, ref2, ref2_text, arrow1, arrow2)
        all_refs.shift(DOWN * 1.5)

        self.play(Create(owner), Write(owner_text))
        self.play(Create(ref1), Write(ref1_text), Create(arrow1))
        self.play(Create(ref2), Write(ref2_text), Create(arrow2))

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_mutable_ref(self):
        title = Text("가변 참조 (&mut T)", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        code = '''let mut s = String::from("hello");
let r = &mut s;     // 가변 참조

r.push_str(", world");
println!("{}", r);  // "hello, world"'''

        code_text = Text(code, font_size=22, font="monospace", color=TEXT_WHITE)
        code_text.next_to(title, DOWN, buff=0.5)
        self.play(Write(code_text), run_time=1.5)

        warning = Text("⚠️ 가변 참조는 동시에 하나만!", font_size=28, color=INVALID_RED)
        warning.next_to(code_text, DOWN, buff=0.5)
        self.play(Write(warning))

        error_code = '''let mut s = String::from("hello");
let r1 = &mut s;
let r2 = &mut s;    // ERROR!'''

        error_text = Text(error_code, font_size=20, font="monospace", color=INVALID_RED)
        error_text.next_to(warning, DOWN, buff=0.5)
        self.play(Write(error_text))

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_rules(self):
        title = Text("빌림 규칙", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        rules = VGroup(
            Text("1. 여러 개의 불변 참조 OR", font_size=28, color=STACK_BLUE),
            Text("2. 하나의 가변 참조", font_size=28, color=HEAP_PURPLE),
            Text("(둘 다는 불가!)", font_size=24, color=INVALID_RED),
        ).arrange(DOWN, buff=0.5)
        rules.next_to(title, DOWN, buff=1)

        self.play(Write(rules), run_time=2)

        why = Text("왜? → 데이터 레이스 방지!", font_size=32, color=VALID_GREEN)
        why.next_to(rules, DOWN, buff=1)
        self.play(Write(why))

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def outro(self):
        summary = Text("빌림 = 소유권 없이 안전하게 접근", font_size=40, color=RUST_ORANGE)
        next_ep = Text("다음: EP 2-04 Lifetimes", font_size=28, opacity=0.7)
        next_ep.next_to(summary, DOWN, buff=1)

        self.play(Write(summary), FadeIn(next_ep))
        self.wait(2)


class EP_2_04_Lifetimes(Scene):
    """EP 2-04: Lifetimes - The Compiler's Memory Guard"""

    def construct(self):
        self.intro()
        self.section_what_is_lifetime()
        self.section_syntax()
        self.section_in_functions()
        self.section_in_structs()
        self.outro()

    def intro(self):
        title = Text("EP 2-04: Lifetimes", font_size=48, color=RUST_ORANGE)
        subtitle = Text("참조의 유효 범위를 명시하다", font_size=28, opacity=0.7)
        subtitle.next_to(title, DOWN, buff=0.5)

        self.play(Write(title), FadeIn(subtitle))
        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_what_is_lifetime(self):
        title = Text("라이프타임이란?", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        definition = Text(
            "참조가 유효한 범위(scope)를\n컴파일러에게 알려주는 주석",
            font_size=28, color=TEXT_WHITE
        )
        definition.next_to(title, DOWN, buff=0.5)
        self.play(Write(definition))

        # Dangling reference example
        problem = Text("문제: 댕글링 참조 방지", font_size=28, color=INVALID_RED)
        problem.next_to(definition, DOWN, buff=0.5)
        self.play(Write(problem))

        code = '''{
    let r;              // 'a 시작
    {
        let x = 5;      // 'b 시작
        r = &x;         // r은 x를 참조
    }                   // 'b 끝, x 해제!
    println!("{}", r);  // ERROR: r은 댕글링!
}'''

        code_text = Text(code, font_size=18, font="monospace", color=TEXT_WHITE)
        code_text.next_to(problem, DOWN, buff=0.3)
        self.play(Write(code_text), run_time=2)

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_syntax(self):
        title = Text("라이프타임 문법", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        syntax = VGroup(
            Text("'a  ← 라이프타임 파라미터", font_size=24, font="monospace"),
            Text("&'a T  ← 'a 동안 유효한 불변 참조", font_size=24, font="monospace"),
            Text("&'a mut T  ← 'a 동안 유효한 가변 참조", font_size=24, font="monospace"),
        ).arrange(DOWN, buff=0.4, aligned_edge=LEFT)
        syntax.next_to(title, DOWN, buff=0.8)

        for s in syntax:
            self.play(Write(s), run_time=0.6)

        note = Text("💡 'a는 관례적 이름. 'b, 'c, 'lifetime 등 사용 가능",
                   font_size=22, color=VALID_GREEN)
        note.next_to(syntax, DOWN, buff=0.5)
        self.play(Write(note))

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_in_functions(self):
        title = Text("함수에서 라이프타임", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        code = '''// 두 문자열 중 긴 것을 반환
fn longest<'a>(x: &'a str, y: &'a str) -> &'a str {
    if x.len() > y.len() { x } else { y }
}

fn main() {
    let s1 = String::from("long string");
    let s2 = String::from("short");
    let result = longest(&s1, &s2);
    println!("{}", result);  // "long string"
}'''

        code_text = Text(code, font_size=18, font="monospace", color=TEXT_WHITE)
        code_text.next_to(title, DOWN, buff=0.5)
        self.play(Write(code_text), run_time=2)

        explanation = Text("'a = x와 y 중 더 짧은 라이프타임", font_size=24, color=VALID_GREEN)
        explanation.to_edge(DOWN, buff=0.5)
        self.play(Write(explanation))

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_in_structs(self):
        title = Text("구조체에서 라이프타임", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        code = '''struct ImportantExcerpt<'a> {
    part: &'a str,  // 구조체가 참조를 보유
}

fn main() {
    let novel = String::from("Call me Ishmael...");
    let first = novel.split('.').next().unwrap();
    let i = ImportantExcerpt { part: first };
}  // novel이 i보다 오래 살아야 함!'''

        code_text = Text(code, font_size=18, font="monospace", color=TEXT_WHITE)
        code_text.next_to(title, DOWN, buff=0.5)
        self.play(Write(code_text), run_time=2)

        static_note = Text("'static: 프로그램 전체 기간 유효", font_size=24, color=RUST_ORANGE)
        static_note.to_edge(DOWN, buff=0.5)
        self.play(Write(static_note))

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def outro(self):
        done = Text("Level 2 완료!", font_size=56, color=VALID_GREEN)

        mastered = VGroup(
            Text("✓ 소유권 3가지 규칙", font_size=24),
            Text("✓ Move, Copy, Clone", font_size=24),
            Text("✓ References & Borrowing", font_size=24),
            Text("✓ Lifetimes", font_size=24),
        ).arrange(DOWN, buff=0.3, aligned_edge=LEFT)
        mastered.next_to(done, DOWN, buff=0.5)

        nrt = Text("NRT: 당신은 이제 소유권 마스터!", font_size=28, color=RUST_ORANGE)
        nrt.next_to(mastered, DOWN, buff=0.5)

        next_level = Text("다음: Level 3 - Data Structures", font_size=24, opacity=0.7)
        next_level.next_to(nrt, DOWN, buff=0.5)

        self.play(Write(done), FadeIn(mastered), FadeIn(nrt), FadeIn(next_level))
        self.wait(3)
