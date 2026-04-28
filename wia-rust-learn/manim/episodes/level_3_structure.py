"""
WIA-RUST-LEARN Level 3: 구조화 (Structure)
===========================================

EP 3-01: Structs & Enums (구조체와 열거형) - 20분
EP 3-02: Error Handling (에러 처리) - 18분
EP 3-03: Modules & Crates (모듈과 크레이트) - 15분

실행 방법:
    manim -pql level_3_structure.py EP_3_01_StructsEnums
    manim -pql level_3_structure.py EP_3_02_ErrorHandling
    manim -pql level_3_structure.py EP_3_03_ModulesCrates

전체 렌더링 (고화질):
    manim -pqh level_3_structure.py --all
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
DARK_BG = "#1a1a2e"


class EP_3_01_StructsEnums(Scene):
    """
    EP 3-01: Structs & Enums - 데이터를 구조화하는 두 가지 방법

    학습 목표:
    1. struct로 관련 데이터 그룹화
    2. enum으로 상태/변형 표현
    3. impl로 메서드 정의
    4. match로 패턴 매칭
    """

    def construct(self):
        self.camera.background_color = DARK_BG

        # 인트로
        self.section_intro()
        self.wait(0.5)
        self.clear()

        # 섹션 1: Struct 기초
        self.section_struct_basics()
        self.wait(0.5)
        self.clear()

        # 섹션 2: Struct 메모리 레이아웃
        self.section_struct_memory()
        self.wait(0.5)
        self.clear()

        # 섹션 3: Enum 기초
        self.section_enum_basics()
        self.wait(0.5)
        self.clear()

        # 섹션 4: Option & Result
        self.section_option_result()
        self.wait(0.5)
        self.clear()

        # 섹션 5: impl 블록
        self.section_impl_block()
        self.wait(0.5)
        self.clear()

        # 섹션 6: 패턴 매칭
        self.section_pattern_matching()
        self.wait(0.5)
        self.clear()

        # 아웃트로
        self.section_outro()

    def section_intro(self):
        title = Text("EP 3-01", font_size=32, color=GRAY).to_edge(UP)
        main_title = Text("Structs & Enums", font_size=56, color=RUST_ORANGE)
        subtitle = Text("데이터를 구조화하는 두 가지 방법", font_size=28, color=GRAY)

        main_title.next_to(title, DOWN, buff=0.8)
        subtitle.next_to(main_title, DOWN, buff=0.3)

        self.play(Write(title))
        self.play(Write(main_title))
        self.play(FadeIn(subtitle))
        self.wait(1)

        # 핵심 질문
        question = Text("관련된 데이터를 어떻게 묶을까?", font_size=36, color=WHITE)
        question.next_to(subtitle, DOWN, buff=1)

        self.play(Write(question))
        self.wait(1)

        # Struct vs Enum
        comparison = VGroup(
            VGroup(
                Text("Struct", font_size=32, color=STACK_GREEN),
                Text("여러 필드를 AND로 결합", font_size=20, color=GRAY),
                Text("User { name, age, email }", font_size=18, color=WHITE),
            ).arrange(DOWN, buff=0.2),
            VGroup(
                Text("Enum", font_size=32, color=HEAP_PURPLE),
                Text("여러 변형 중 OR로 선택", font_size=20, color=GRAY),
                Text("Status::Active | Inactive", font_size=18, color=WHITE),
            ).arrange(DOWN, buff=0.2),
        ).arrange(RIGHT, buff=2)
        comparison.next_to(question, DOWN, buff=0.8)

        self.play(FadeIn(comparison, shift=UP))
        self.wait(2)

    def section_struct_basics(self):
        title = Text("Struct: 데이터 그룹화", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # 코드 예시
        code = Code(
            code='''struct User {
    name: String,
    age: u32,
    email: String,
    active: bool,
}

let user = User {
    name: String::from("Alice"),
    age: 30,
    email: String::from("alice@example.com"),
    active: true,
};''',
            language="rust",
            font_size=20,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.8)
        code.next_to(title, DOWN, buff=0.5).to_edge(LEFT, buff=0.5)

        self.play(Create(code))
        self.wait(1)

        # 시각화
        struct_box = VGroup()

        # User 구조체 박스
        user_rect = RoundedRectangle(
            width=4, height=3.5, corner_radius=0.2,
            fill_color=STACK_GREEN, fill_opacity=0.2,
            stroke_color=STACK_GREEN
        )
        user_label = Text("User", font_size=24, color=STACK_GREEN)
        user_label.next_to(user_rect, UP, buff=0.1)

        # 필드들
        fields = VGroup(
            self.create_field("name", '"Alice"', STRING_COLOR := "#E67E22"),
            self.create_field("age", "30", "#3498DB"),
            self.create_field("email", '"alice@..."', STRING_COLOR),
            self.create_field("active", "true", SUCCESS_GREEN),
        ).arrange(DOWN, buff=0.15)
        fields.move_to(user_rect)

        struct_box.add(user_rect, user_label, fields)
        struct_box.to_edge(RIGHT, buff=1)

        self.play(Create(user_rect), Write(user_label))
        for field in fields:
            self.play(FadeIn(field, shift=LEFT), run_time=0.3)

        self.wait(1)

        # 필드 접근
        access_code = Text("user.name → \"Alice\"", font_size=24, color=WHITE)
        access_code.next_to(struct_box, DOWN, buff=0.5)

        arrow = Arrow(
            access_code.get_left() + LEFT * 0.3,
            fields[0].get_right() + RIGHT * 0.2,
            color=WARNING_YELLOW
        )

        self.play(Write(access_code))
        self.play(Create(arrow), fields[0].animate.set_opacity(1))
        self.wait(1)

    def create_field(self, name, value, color):
        field = VGroup(
            Text(f"{name}:", font_size=16, color=GRAY),
            Text(value, font_size=16, color=color),
        ).arrange(RIGHT, buff=0.3)
        return field

    def section_struct_memory(self):
        title = Text("Struct 메모리 레이아웃", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # Stack과 Heap
        stack_label = Text("Stack", font_size=24, color=STACK_GREEN)
        heap_label = Text("Heap", font_size=24, color=HEAP_PURPLE)

        # Stack 영역
        stack = VGroup()
        stack_rect = Rectangle(width=4, height=4, fill_color=STACK_GREEN, fill_opacity=0.1)
        stack_label.next_to(stack_rect, UP)

        # User 구조체의 스택 레이아웃
        stack_fields = VGroup(
            self.create_memory_cell("name (ptr, len, cap)", "24 bytes"),
            self.create_memory_cell("age (u32)", "4 bytes"),
            self.create_memory_cell("email (ptr, len, cap)", "24 bytes"),
            self.create_memory_cell("active (bool)", "1 byte"),
        ).arrange(DOWN, buff=0.1)
        stack_fields.move_to(stack_rect)

        stack.add(stack_rect, stack_label, stack_fields)
        stack.to_edge(LEFT, buff=1)

        # Heap 영역
        heap = VGroup()
        heap_rect = Rectangle(width=3.5, height=2.5, fill_color=HEAP_PURPLE, fill_opacity=0.1)
        heap_label.next_to(heap_rect, UP)

        heap_data = VGroup(
            self.create_heap_block("Alice", 5),
            self.create_heap_block("alice@example.com", 17),
        ).arrange(DOWN, buff=0.3)
        heap_data.move_to(heap_rect)

        heap.add(heap_rect, heap_label, heap_data)
        heap.to_edge(RIGHT, buff=1)

        self.play(Create(stack), Create(heap))
        self.wait(0.5)

        # 포인터 화살표
        arrow1 = Arrow(
            stack_fields[0].get_right(),
            heap_data[0].get_left(),
            color=MEMORY_BLUE, buff=0.1
        )
        arrow2 = Arrow(
            stack_fields[2].get_right(),
            heap_data[1].get_left(),
            color=MEMORY_BLUE, buff=0.1
        )

        self.play(Create(arrow1), Create(arrow2))
        self.wait(1)

        # 총 크기
        size_note = VGroup(
            Text("Stack: 53+ bytes (정렬로 더 커질 수 있음)", font_size=18, color=GRAY),
            Text("Heap: 동적 할당 (실제 문자열 데이터)", font_size=18, color=GRAY),
        ).arrange(DOWN, buff=0.2)
        size_note.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(size_note))
        self.wait(2)

    def create_memory_cell(self, label, size):
        cell = VGroup(
            Rectangle(width=3.5, height=0.5, fill_color=STACK_GREEN, fill_opacity=0.3),
            Text(label, font_size=12, color=WHITE),
            Text(size, font_size=10, color=GRAY).shift(RIGHT * 1.2),
        )
        cell[1].move_to(cell[0]).shift(LEFT * 0.3)
        cell[2].next_to(cell[0], RIGHT, buff=0.1)
        return cell

    def create_heap_block(self, text, size):
        block = VGroup(
            Rectangle(
                width=2.5, height=0.6,
                fill_color=HEAP_PURPLE, fill_opacity=0.3,
                stroke_color=HEAP_PURPLE
            ),
            Text(f'"{text}"', font_size=14, color=WHITE),
        )
        block[1].move_to(block[0])
        return block

    def section_enum_basics(self):
        title = Text("Enum: 변형 중 하나 선택", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # 코드 예시
        code = Code(
            code='''enum Status {
    Active,
    Inactive,
    Pending { since: u64 },
    Banned(String),
}

let status = Status::Active;
let banned = Status::Banned(String::from("spam"));''',
            language="rust",
            font_size=20,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.8)
        code.next_to(title, DOWN, buff=0.5).to_edge(LEFT, buff=0.5)

        self.play(Create(code))
        self.wait(1)

        # Enum 시각화 - 상태 머신처럼
        states = VGroup()

        # 각 상태를 원으로 표현
        active = Circle(radius=0.5, fill_color=SUCCESS_GREEN, fill_opacity=0.5)
        active_label = Text("Active", font_size=14, color=WHITE)
        active_label.move_to(active)

        inactive = Circle(radius=0.5, fill_color=GRAY, fill_opacity=0.5)
        inactive_label = Text("Inactive", font_size=14, color=WHITE)
        inactive_label.move_to(inactive)

        pending = Circle(radius=0.6, fill_color=WARNING_YELLOW, fill_opacity=0.5)
        pending_label = VGroup(
            Text("Pending", font_size=12, color=WHITE),
            Text("{since}", font_size=10, color=GRAY),
        ).arrange(DOWN, buff=0.05)
        pending_label.move_to(pending)

        banned = Circle(radius=0.6, fill_color=ERROR_RED, fill_opacity=0.5)
        banned_label = VGroup(
            Text("Banned", font_size=12, color=WHITE),
            Text("(String)", font_size=10, color=GRAY),
        ).arrange(DOWN, buff=0.05)
        banned_label.move_to(banned)

        active_g = VGroup(active, active_label)
        inactive_g = VGroup(inactive, inactive_label)
        pending_g = VGroup(pending, pending_label)
        banned_g = VGroup(banned, banned_label)

        states = VGroup(active_g, inactive_g, pending_g, banned_g)
        states.arrange_in_grid(rows=2, cols=2, buff=0.8)
        states.to_edge(RIGHT, buff=1)

        # 중앙에 Status 레이블
        status_label = Text("Status", font_size=28, color=HEAP_PURPLE)
        status_label.next_to(states, UP, buff=0.3)

        self.play(Write(status_label))
        for state in states:
            self.play(FadeIn(state, scale=0.5), run_time=0.4)

        self.wait(1)

        # 핵심 포인트
        key_point = VGroup(
            Text("✓ 한 번에 하나의 변형만 가능", font_size=18, color=SUCCESS_GREEN),
            Text("✓ 각 변형은 다른 데이터를 가질 수 있음", font_size=18, color=SUCCESS_GREEN),
            Text("✓ 컴파일러가 모든 케이스 처리 보장", font_size=18, color=SUCCESS_GREEN),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.2)
        key_point.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(key_point, shift=UP))
        self.wait(2)

    def section_option_result(self):
        title = Text("표준 Enum: Option & Result", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # Option
        option_box = VGroup(
            Text("Option<T>", font_size=28, color=MEMORY_BLUE),
            Code(
                code='''enum Option<T> {
    Some(T),
    None,
}''',
                language="rust",
                font_size=18,
                background="rectangle",
            ).scale(0.7),
            Text("값이 있거나 없거나", font_size=16, color=GRAY),
        ).arrange(DOWN, buff=0.3)

        # Result
        result_box = VGroup(
            Text("Result<T, E>", font_size=28, color=HEAP_PURPLE),
            Code(
                code='''enum Result<T, E> {
    Ok(T),
    Err(E),
}''',
                language="rust",
                font_size=18,
                background="rectangle",
            ).scale(0.7),
            Text("성공하거나 실패하거나", font_size=16, color=GRAY),
        ).arrange(DOWN, buff=0.3)

        boxes = VGroup(option_box, result_box).arrange(RIGHT, buff=1.5)
        boxes.next_to(title, DOWN, buff=0.8)

        self.play(FadeIn(option_box, shift=RIGHT))
        self.wait(0.5)
        self.play(FadeIn(result_box, shift=LEFT))
        self.wait(1)

        # 사용 예시
        example = Code(
            code='''// Option: null 대신 사용
fn find_user(id: u32) -> Option<User> {
    // Some(user) 또는 None 반환
}

// Result: 예외 대신 사용
fn read_file(path: &str) -> Result<String, Error> {
    // Ok(content) 또는 Err(error) 반환
}''',
            language="rust",
            font_size=16,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.8)
        example.to_edge(DOWN, buff=0.5)

        self.play(Create(example))
        self.wait(1)

        # No null, No exceptions
        badge = VGroup(
            Text("No NULL", font_size=24, color=ERROR_RED),
            Text("No Exceptions", font_size=24, color=ERROR_RED),
        ).arrange(RIGHT, buff=1)
        badge.next_to(boxes, DOWN, buff=0.3)

        cross1 = Cross(badge[0], stroke_color=SUCCESS_GREEN, stroke_width=3)
        cross2 = Cross(badge[1], stroke_color=SUCCESS_GREEN, stroke_width=3)

        self.play(Write(badge))
        self.play(Create(cross1), Create(cross2))
        self.wait(2)

    def section_impl_block(self):
        title = Text("impl: 메서드 정의", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''struct Rectangle {
    width: u32,
    height: u32,
}

impl Rectangle {
    // 연관 함수 (생성자)
    fn new(width: u32, height: u32) -> Self {
        Rectangle { width, height }
    }

    // 메서드 (&self 사용)
    fn area(&self) -> u32 {
        self.width * self.height
    }

    // 가변 메서드 (&mut self)
    fn resize(&mut self, scale: u32) {
        self.width *= scale;
        self.height *= scale;
    }
}''',
            language="rust",
            font_size=16,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.75)
        code.to_edge(LEFT, buff=0.3)

        self.play(Create(code))
        self.wait(1)

        # self 종류 설명
        self_types = VGroup(
            VGroup(
                Text("Self", font_size=20, color=RUST_ORANGE),
                Text("타입 자체", font_size=14, color=GRAY),
            ).arrange(DOWN, buff=0.1),
            VGroup(
                Text("&self", font_size=20, color=STACK_GREEN),
                Text("불변 참조 (읽기)", font_size=14, color=GRAY),
            ).arrange(DOWN, buff=0.1),
            VGroup(
                Text("&mut self", font_size=20, color=WARNING_YELLOW),
                Text("가변 참조 (수정)", font_size=14, color=GRAY),
            ).arrange(DOWN, buff=0.1),
            VGroup(
                Text("self", font_size=20, color=ERROR_RED),
                Text("소유권 이전 (소비)", font_size=14, color=GRAY),
            ).arrange(DOWN, buff=0.1),
        ).arrange(DOWN, buff=0.4, aligned_edge=LEFT)
        self_types.to_edge(RIGHT, buff=0.8)

        for st in self_types:
            self.play(FadeIn(st, shift=LEFT), run_time=0.4)

        self.wait(2)

    def section_pattern_matching(self):
        title = Text("match: 패턴 매칭", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''enum Message {
    Quit,
    Move { x: i32, y: i32 },
    Write(String),
    Color(u8, u8, u8),
}

fn handle(msg: Message) {
    match msg {
        Message::Quit => println!("종료"),
        Message::Move { x, y } => {
            println!("이동: ({}, {})", x, y)
        }
        Message::Write(text) => println!("{}", text),
        Message::Color(r, g, b) => {
            println!("RGB: {}, {}, {}", r, g, b)
        }
    }
}''',
            language="rust",
            font_size=16,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.7)
        code.to_edge(LEFT, buff=0.3)

        self.play(Create(code))
        self.wait(1)

        # 패턴 매칭 장점
        benefits = VGroup(
            Text("✓ 모든 케이스 처리 강제", font_size=20, color=SUCCESS_GREEN),
            Text("✓ 값 추출과 동시에 분기", font_size=20, color=SUCCESS_GREEN),
            Text("✓ 컴파일 타임 완전성 검사", font_size=20, color=SUCCESS_GREEN),
        ).arrange(DOWN, buff=0.3, aligned_edge=LEFT)
        benefits.to_edge(RIGHT, buff=0.8)

        self.play(FadeIn(benefits, shift=LEFT))
        self.wait(1)

        # if let 소개
        if_let = Code(
            code='''// 하나의 케이스만 관심있을 때
if let Some(value) = option {
    println!("값: {}", value);
}

// while let도 가능
while let Some(item) = stack.pop() {
    process(item);
}''',
            language="rust",
            font_size=14,
            background="rectangle",
        ).scale(0.7)
        if_let.next_to(benefits, DOWN, buff=0.5)

        self.play(Create(if_let))
        self.wait(2)

    def section_outro(self):
        title = Text("EP 3-01 정리", font_size=44, color=RUST_ORANGE)

        summary = VGroup(
            Text("Struct = AND (모든 필드 필요)", font_size=24, color=STACK_GREEN),
            Text("Enum = OR (변형 중 하나)", font_size=24, color=HEAP_PURPLE),
            Text("impl = 메서드 정의", font_size=24, color=MEMORY_BLUE),
            Text("match = 패턴 매칭", font_size=24, color=WARNING_YELLOW),
        ).arrange(DOWN, buff=0.4)

        VGroup(title, summary).arrange(DOWN, buff=0.8)

        self.play(Write(title))
        for line in summary:
            self.play(FadeIn(line, shift=UP), run_time=0.4)

        self.wait(1)

        next_ep = Text("다음: EP 3-02 Error Handling", font_size=28, color=GRAY)
        next_ep.to_edge(DOWN, buff=1)
        self.play(FadeIn(next_ep))
        self.wait(2)


class EP_3_02_ErrorHandling(Scene):
    """
    EP 3-02: Error Handling - Rust의 에러 처리 철학

    학습 목표:
    1. Result<T, E>로 복구 가능한 에러 처리
    2. panic!으로 복구 불가능한 에러 처리
    3. ? 연산자로 에러 전파
    4. 커스텀 에러 타입 정의
    """

    def construct(self):
        self.camera.background_color = DARK_BG

        self.section_intro()
        self.wait(0.5)
        self.clear()

        self.section_two_kinds_of_errors()
        self.wait(0.5)
        self.clear()

        self.section_result_usage()
        self.wait(0.5)
        self.clear()

        self.section_question_mark()
        self.wait(0.5)
        self.clear()

        self.section_custom_errors()
        self.wait(0.5)
        self.clear()

        self.section_outro()

    def section_intro(self):
        title = Text("EP 3-02", font_size=32, color=GRAY).to_edge(UP)
        main_title = Text("Error Handling", font_size=56, color=RUST_ORANGE)
        subtitle = Text("예외 없이 안전하게", font_size=28, color=GRAY)

        main_title.next_to(title, DOWN, buff=0.8)
        subtitle.next_to(main_title, DOWN, buff=0.3)

        self.play(Write(title), Write(main_title))
        self.play(FadeIn(subtitle))
        self.wait(1)

        # 다른 언어와 비교
        comparison = VGroup(
            VGroup(
                Text("다른 언어", font_size=24, color=GRAY),
                Text("try { ... } catch { ... }", font_size=18, color=WHITE),
                Text("숨겨진 제어 흐름", font_size=14, color=ERROR_RED),
            ).arrange(DOWN, buff=0.2),
            Text("→", font_size=36, color=WHITE),
            VGroup(
                Text("Rust", font_size=24, color=RUST_ORANGE),
                Text("Result<T, E>", font_size=18, color=WHITE),
                Text("명시적인 에러 타입", font_size=14, color=SUCCESS_GREEN),
            ).arrange(DOWN, buff=0.2),
        ).arrange(RIGHT, buff=1)
        comparison.next_to(subtitle, DOWN, buff=1)

        self.play(FadeIn(comparison, shift=UP))
        self.wait(2)

    def section_two_kinds_of_errors(self):
        title = Text("두 종류의 에러", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # Recoverable vs Unrecoverable
        recoverable = VGroup(
            Text("복구 가능", font_size=28, color=SUCCESS_GREEN),
            Text("Result<T, E>", font_size=22, color=WHITE),
            VGroup(
                Text("• 파일을 찾을 수 없음", font_size=16, color=GRAY),
                Text("• 네트워크 연결 실패", font_size=16, color=GRAY),
                Text("• 잘못된 사용자 입력", font_size=16, color=GRAY),
            ).arrange(DOWN, aligned_edge=LEFT, buff=0.1),
        ).arrange(DOWN, buff=0.4)

        unrecoverable = VGroup(
            Text("복구 불가능", font_size=28, color=ERROR_RED),
            Text("panic!", font_size=22, color=WHITE),
            VGroup(
                Text("• 배열 인덱스 초과", font_size=16, color=GRAY),
                Text("• 논리적 버그", font_size=16, color=GRAY),
                Text("• 불변 조건 위반", font_size=16, color=GRAY),
            ).arrange(DOWN, aligned_edge=LEFT, buff=0.1),
        ).arrange(DOWN, buff=0.4)

        # 박스로 감싸기
        rec_box = SurroundingRectangle(recoverable, color=SUCCESS_GREEN, buff=0.3)
        unrec_box = SurroundingRectangle(unrecoverable, color=ERROR_RED, buff=0.3)

        left_group = VGroup(recoverable, rec_box)
        right_group = VGroup(unrecoverable, unrec_box)

        groups = VGroup(left_group, right_group).arrange(RIGHT, buff=1.5)
        groups.next_to(title, DOWN, buff=0.8)

        self.play(Create(rec_box), FadeIn(recoverable))
        self.wait(0.5)
        self.play(Create(unrec_box), FadeIn(unrecoverable))
        self.wait(1)

        # 핵심 메시지
        message = Text(
            "대부분의 경우 Result를 사용하세요!",
            font_size=24,
            color=WARNING_YELLOW
        )
        message.to_edge(DOWN, buff=0.8)

        self.play(Write(message))
        self.wait(2)

    def section_result_usage(self):
        title = Text("Result<T, E> 사용법", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''use std::fs::File;

fn main() {
    let file_result = File::open("hello.txt");

    let file = match file_result {
        Ok(f) => f,
        Err(error) => {
            println!("파일 열기 실패: {}", error);
            return;
        }
    };

    // 또는 간단하게
    let file = File::open("hello.txt").unwrap();      // 실패시 panic
    let file = File::open("hello.txt").expect("msg"); // 메시지와 함께 panic
    let file = File::open("hello.txt").unwrap_or(default); // 기본값
}''',
            language="rust",
            font_size=16,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.7)
        code.next_to(title, DOWN, buff=0.5)

        self.play(Create(code))
        self.wait(1)

        # 흐름도
        flow = VGroup()

        result_node = RoundedRectangle(width=2, height=0.8, corner_radius=0.1)
        result_node.set_fill(MEMORY_BLUE, opacity=0.3)
        result_label = Text("Result", font_size=18, color=WHITE)
        result_label.move_to(result_node)

        ok_node = RoundedRectangle(width=1.5, height=0.6, corner_radius=0.1)
        ok_node.set_fill(SUCCESS_GREEN, opacity=0.3)
        ok_label = Text("Ok(T)", font_size=14, color=WHITE)
        ok_label.move_to(ok_node)

        err_node = RoundedRectangle(width=1.5, height=0.6, corner_radius=0.1)
        err_node.set_fill(ERROR_RED, opacity=0.3)
        err_label = Text("Err(E)", font_size=14, color=WHITE)
        err_label.move_to(err_node)

        result_g = VGroup(result_node, result_label)
        ok_g = VGroup(ok_node, ok_label)
        err_g = VGroup(err_node, err_label)

        result_g.to_edge(DOWN, buff=1.5)
        ok_g.next_to(result_g, DOWN + LEFT, buff=0.5)
        err_g.next_to(result_g, DOWN + RIGHT, buff=0.5)

        arrow_ok = Arrow(result_g.get_bottom(), ok_g.get_top(), color=SUCCESS_GREEN, buff=0.1)
        arrow_err = Arrow(result_g.get_bottom(), err_g.get_top(), color=ERROR_RED, buff=0.1)

        self.play(
            Create(result_g),
            Create(ok_g),
            Create(err_g),
            Create(arrow_ok),
            Create(arrow_err),
        )
        self.wait(2)

    def section_question_mark(self):
        title = Text("? 연산자: 에러 전파", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # Before
        before = Code(
            code='''fn read_username() -> Result<String, io::Error> {
    let file = match File::open("username.txt") {
        Ok(f) => f,
        Err(e) => return Err(e),
    };

    let mut contents = String::new();
    match file.read_to_string(&mut contents) {
        Ok(_) => Ok(contents),
        Err(e) => Err(e),
    }
}''',
            language="rust",
            font_size=14,
            background="rectangle",
        ).scale(0.7)

        # After
        after = Code(
            code='''fn read_username() -> Result<String, io::Error> {
    let mut file = File::open("username.txt")?;
    let mut contents = String::new();
    file.read_to_string(&mut contents)?;
    Ok(contents)
}

// 더 간단하게
fn read_username() -> Result<String, io::Error> {
    std::fs::read_to_string("username.txt")
}''',
            language="rust",
            font_size=14,
            background="rectangle",
            background_stroke_color=SUCCESS_GREEN,
        ).scale(0.7)

        before_label = Text("Before", font_size=20, color=ERROR_RED)
        after_label = Text("After", font_size=20, color=SUCCESS_GREEN)

        before_g = VGroup(before_label, before).arrange(DOWN, buff=0.2)
        after_g = VGroup(after_label, after).arrange(DOWN, buff=0.2)

        groups = VGroup(before_g, after_g).arrange(RIGHT, buff=0.5)
        groups.next_to(title, DOWN, buff=0.5)

        self.play(Create(before_g))
        self.wait(1)

        arrow = Arrow(before_g.get_right(), after_g.get_left(), color=WARNING_YELLOW)
        self.play(Create(arrow), Create(after_g))
        self.wait(1)

        # ? 설명
        explanation = VGroup(
            Text("? = ", font_size=24, color=RUST_ORANGE),
            Text("Ok면 값 추출, Err면 조기 반환", font_size=20, color=WHITE),
        ).arrange(RIGHT, buff=0.3)
        explanation.to_edge(DOWN, buff=0.5)

        self.play(Write(explanation))
        self.wait(2)

    def section_custom_errors(self):
        title = Text("커스텀 에러 타입", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''use thiserror::Error;

#[derive(Error, Debug)]
pub enum AppError {
    #[error("사용자를 찾을 수 없습니다: {0}")]
    UserNotFound(String),

    #[error("권한이 없습니다")]
    Unauthorized,

    #[error("IO 에러: {0}")]
    Io(#[from] std::io::Error),

    #[error("데이터베이스 에러: {0}")]
    Database(#[from] sqlx::Error),
}

fn find_user(id: &str) -> Result<User, AppError> {
    // UserNotFound 또는 다른 에러 반환 가능
}''',
            language="rust",
            font_size=15,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.7)
        code.next_to(title, DOWN, buff=0.4)

        self.play(Create(code))
        self.wait(1)

        # 장점
        benefits = VGroup(
            Text("✓ 타입 안전한 에러 처리", font_size=18, color=SUCCESS_GREEN),
            Text("✓ 자동 변환 (#[from])", font_size=18, color=SUCCESS_GREEN),
            Text("✓ 사용자 친화적 메시지", font_size=18, color=SUCCESS_GREEN),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.2)
        benefits.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(benefits, shift=UP))
        self.wait(2)

    def section_outro(self):
        title = Text("EP 3-02 정리", font_size=44, color=RUST_ORANGE)

        summary = VGroup(
            Text("Result<T, E> = 복구 가능한 에러", font_size=24, color=SUCCESS_GREEN),
            Text("panic! = 복구 불가능한 에러", font_size=24, color=ERROR_RED),
            Text("? = 에러 전파 연산자", font_size=24, color=WARNING_YELLOW),
            Text("예외 없이 명시적으로!", font_size=24, color=MEMORY_BLUE),
        ).arrange(DOWN, buff=0.4)

        VGroup(title, summary).arrange(DOWN, buff=0.8)

        self.play(Write(title))
        for line in summary:
            self.play(FadeIn(line, shift=UP), run_time=0.4)

        self.wait(1)

        next_ep = Text("다음: EP 3-03 Modules & Crates", font_size=28, color=GRAY)
        next_ep.to_edge(DOWN, buff=1)
        self.play(FadeIn(next_ep))
        self.wait(2)


class EP_3_03_ModulesCrates(Scene):
    """
    EP 3-03: Modules & Crates - 코드 구조화와 재사용

    학습 목표:
    1. mod로 모듈 정의
    2. pub로 가시성 제어
    3. use로 경로 단축
    4. Cargo.toml로 의존성 관리
    """

    def construct(self):
        self.camera.background_color = DARK_BG

        self.section_intro()
        self.wait(0.5)
        self.clear()

        self.section_module_basics()
        self.wait(0.5)
        self.clear()

        self.section_visibility()
        self.wait(0.5)
        self.clear()

        self.section_use_keyword()
        self.wait(0.5)
        self.clear()

        self.section_cargo_deps()
        self.wait(0.5)
        self.clear()

        self.section_outro()

    def section_intro(self):
        title = Text("EP 3-03", font_size=32, color=GRAY).to_edge(UP)
        main_title = Text("Modules & Crates", font_size=56, color=RUST_ORANGE)
        subtitle = Text("코드 구조화와 재사용", font_size=28, color=GRAY)

        main_title.next_to(title, DOWN, buff=0.8)
        subtitle.next_to(main_title, DOWN, buff=0.3)

        self.play(Write(title), Write(main_title))
        self.play(FadeIn(subtitle))
        self.wait(1)

        # 계층 구조
        hierarchy = VGroup(
            VGroup(
                Text("Crate", font_size=28, color=RUST_ORANGE),
                Text("(컴파일 단위)", font_size=16, color=GRAY),
            ).arrange(DOWN, buff=0.1),
            Text("↓", font_size=32, color=WHITE),
            VGroup(
                Text("Module", font_size=28, color=HEAP_PURPLE),
                Text("(코드 그룹)", font_size=16, color=GRAY),
            ).arrange(DOWN, buff=0.1),
            Text("↓", font_size=32, color=WHITE),
            VGroup(
                Text("Item", font_size=28, color=STACK_GREEN),
                Text("(함수, 구조체 등)", font_size=16, color=GRAY),
            ).arrange(DOWN, buff=0.1),
        ).arrange(DOWN, buff=0.3)
        hierarchy.next_to(subtitle, DOWN, buff=0.8)

        self.play(FadeIn(hierarchy, shift=UP))
        self.wait(2)

    def section_module_basics(self):
        title = Text("모듈 정의하기", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # 파일 구조
        file_structure = VGroup(
            Text("프로젝트 구조", font_size=24, color=MEMORY_BLUE),
            Code(
                code='''my_project/
├── Cargo.toml
└── src/
    ├── main.rs      # 바이너리 크레이트 루트
    ├── lib.rs       # 라이브러리 크레이트 루트
    └── models/
        ├── mod.rs   # 모듈 선언
        ├── user.rs
        └── post.rs''',
                language="text",
                font_size=14,
            ).scale(0.8),
        ).arrange(DOWN, buff=0.3)
        file_structure.to_edge(LEFT, buff=0.5)

        self.play(Create(file_structure))
        self.wait(1)

        # 코드로 보기
        code = Code(
            code='''// src/lib.rs
mod models;  // models 모듈 선언

// src/models/mod.rs
mod user;    // user 서브모듈
mod post;    // post 서브모듈

pub use user::User;  // 재내보내기

// src/models/user.rs
pub struct User {
    pub name: String,
    age: u32,  // private
}''',
            language="rust",
            font_size=15,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.75)
        code.to_edge(RIGHT, buff=0.5)

        self.play(Create(code))
        self.wait(2)

    def section_visibility(self):
        title = Text("pub: 가시성 제어", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # 가시성 레벨들
        levels = VGroup(
            self.create_visibility_item("(없음)", "현재 모듈 내에서만", ERROR_RED),
            self.create_visibility_item("pub", "외부 어디서나", SUCCESS_GREEN),
            self.create_visibility_item("pub(crate)", "현재 크레이트 내에서만", WARNING_YELLOW),
            self.create_visibility_item("pub(super)", "부모 모듈에서", MEMORY_BLUE),
            self.create_visibility_item("pub(in path)", "지정된 경로에서", HEAP_PURPLE),
        ).arrange(DOWN, buff=0.4, aligned_edge=LEFT)
        levels.next_to(title, DOWN, buff=0.6).to_edge(LEFT, buff=1)

        for level in levels:
            self.play(FadeIn(level, shift=RIGHT), run_time=0.4)

        self.wait(1)

        # 예시 코드
        code = Code(
            code='''mod outer {
    pub struct Public;     // ✓ 외부 접근 가능
    struct Private;        // ✗ 모듈 내부만

    pub(crate) fn crate_only() {}

    mod inner {
        pub(super) fn parent_only() {}
    }
}

// 외부에서
use outer::Public;  // ✓
// use outer::Private; // ✗ 컴파일 에러!''',
            language="rust",
            font_size=14,
            background="rectangle",
        ).scale(0.75)
        code.to_edge(RIGHT, buff=0.5)

        self.play(Create(code))
        self.wait(2)

    def create_visibility_item(self, keyword, desc, color):
        return VGroup(
            Text(keyword, font_size=20, color=color),
            Text(f"→ {desc}", font_size=16, color=GRAY),
        ).arrange(RIGHT, buff=0.5)

    def section_use_keyword(self):
        title = Text("use: 경로 단축", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''// 전체 경로
std::collections::HashMap::new();

// use로 가져오기
use std::collections::HashMap;
HashMap::new();

// 여러 항목 가져오기
use std::collections::{HashMap, HashSet};

// 전부 가져오기 (권장하지 않음)
use std::collections::*;

// 별칭
use std::collections::HashMap as Map;

// 재내보내기
pub use crate::models::User;''',
            language="rust",
            font_size=16,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.75)
        code.next_to(title, DOWN, buff=0.5)

        self.play(Create(code))
        self.wait(1)

        # 관용적 use
        idiom = VGroup(
            Text("관용적 use 패턴", font_size=22, color=WARNING_YELLOW),
            Text("• 함수: 부모 모듈까지만 (use module;)", font_size=16, color=GRAY),
            Text("• 구조체/열거형: 전체 경로 (use Type;)", font_size=16, color=GRAY),
        ).arrange(DOWN, buff=0.2, aligned_edge=LEFT)
        idiom.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(idiom, shift=UP))
        self.wait(2)

    def section_cargo_deps(self):
        title = Text("Cargo: 의존성 관리", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        cargo_toml = Code(
            code='''[package]
name = "my_project"
version = "0.1.0"
edition = "2021"

[dependencies]
serde = { version = "1.0", features = ["derive"] }
tokio = { version = "1", features = ["full"] }
reqwest = "0.11"

[dev-dependencies]
criterion = "0.5"''',
            language="toml",
            font_size=16,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.8)
        cargo_toml.to_edge(LEFT, buff=0.5)

        self.play(Create(cargo_toml))
        self.wait(1)

        # Cargo 명령어
        commands = VGroup(
            Text("주요 명령어", font_size=24, color=MEMORY_BLUE),
            VGroup(
                Text("cargo add serde", font_size=16, color=WHITE),
                Text("→ 의존성 추가", font_size=14, color=GRAY),
            ).arrange(RIGHT, buff=0.5),
            VGroup(
                Text("cargo update", font_size=16, color=WHITE),
                Text("→ 의존성 업데이트", font_size=14, color=GRAY),
            ).arrange(RIGHT, buff=0.5),
            VGroup(
                Text("cargo tree", font_size=16, color=WHITE),
                Text("→ 의존성 트리 보기", font_size=14, color=GRAY),
            ).arrange(RIGHT, buff=0.5),
        ).arrange(DOWN, buff=0.3, aligned_edge=LEFT)
        commands.to_edge(RIGHT, buff=0.5)

        self.play(FadeIn(commands, shift=LEFT))
        self.wait(1)

        # crates.io 언급
        crates_io = VGroup(
            Text("📦 crates.io", font_size=24, color=RUST_ORANGE),
            Text("Rust 공식 패키지 저장소", font_size=16, color=GRAY),
            Text("140,000+ 크레이트!", font_size=18, color=SUCCESS_GREEN),
        ).arrange(DOWN, buff=0.2)
        crates_io.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(crates_io, shift=UP))
        self.wait(2)

    def section_outro(self):
        title = Text("EP 3-03 정리", font_size=44, color=RUST_ORANGE)

        summary = VGroup(
            Text("mod = 모듈 정의", font_size=24, color=HEAP_PURPLE),
            Text("pub = 가시성 제어", font_size=24, color=SUCCESS_GREEN),
            Text("use = 경로 단축", font_size=24, color=MEMORY_BLUE),
            Text("Cargo = 의존성 관리", font_size=24, color=RUST_ORANGE),
        ).arrange(DOWN, buff=0.4)

        VGroup(title, summary).arrange(DOWN, buff=0.8)

        self.play(Write(title))
        for line in summary:
            self.play(FadeIn(line, shift=UP), run_time=0.4)

        self.wait(1)

        # Level 3 완료
        complete = Text("🎉 Level 3: 구조화 완료!", font_size=32, color=SUCCESS_GREEN)
        next_level = Text("다음: Level 4 - 고급 기능", font_size=24, color=GRAY)

        VGroup(complete, next_level).arrange(DOWN, buff=0.3).to_edge(DOWN, buff=0.8)

        self.play(FadeIn(complete, scale=1.2))
        self.play(FadeIn(next_level))
        self.wait(2)


# 렌더링 헬퍼
if __name__ == "__main__":
    print("""
╔═══════════════════════════════════════════════════════════════╗
║          WIA-RUST-LEARN Level 3: 구조화                       ║
╠═══════════════════════════════════════════════════════════════╣
║                                                               ║
║  EP 3-01: Structs & Enums                                     ║
║    manim -pql level_3_structure.py EP_3_01_StructsEnums       ║
║                                                               ║
║  EP 3-02: Error Handling                                      ║
║    manim -pql level_3_structure.py EP_3_02_ErrorHandling      ║
║                                                               ║
║  EP 3-03: Modules & Crates                                    ║
║    manim -pql level_3_structure.py EP_3_03_ModulesCrates      ║
║                                                               ║
║  전체 렌더링:                                                  ║
║    manim -pqh level_3_structure.py --all                      ║
║                                                               ║
╚═══════════════════════════════════════════════════════════════╝
    """)
