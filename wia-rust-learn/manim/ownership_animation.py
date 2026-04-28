"""
WIA-RUST-LEARN: Ownership Animation with Manim
==============================================

Install: pip install manim
Run: manim -pql ownership_animation.py OwnershipMove

Scenes:
1. OwnershipMove - Move 시맨틱스 시각화
2. OwnershipBorrow - Borrowing 시각화
3. OwnershipLifetime - Lifetime 시각화
4. FullOwnershipDemo - 전체 데모
"""

from manim import *

# WIA Color Scheme
RUST_ORANGE = "#f74c00"
STACK_BLUE = "#3498db"
HEAP_PURPLE = "#9b59b6"
VALID_GREEN = "#00d26a"
INVALID_RED = "#ff4757"
TEXT_WHITE = "#e6e6e6"


class OwnershipIntro(Scene):
    """Episode 0: Rust Ownership 소개"""

    def construct(self):
        # Title
        title = Text("Rust Ownership", font_size=72, color=RUST_ORANGE)
        subtitle = Text("메모리를 보는 새로운 관점", font_size=36, color=TEXT_WHITE)
        subtitle.next_to(title, DOWN)

        self.play(Write(title), run_time=1.5)
        self.play(FadeIn(subtitle, shift=UP))
        self.wait(2)

        # The Three Rules
        self.play(FadeOut(title), FadeOut(subtitle))

        rules_title = Text("소유권의 3가지 규칙", font_size=48, color=RUST_ORANGE)
        rules_title.to_edge(UP)
        self.play(Write(rules_title))

        rules = VGroup(
            Text("1. 각 값은 하나의 소유자(owner)만 가진다", font_size=32),
            Text("2. 한 번에 하나의 소유자만 존재한다", font_size=32),
            Text("3. 소유자가 스코프를 벗어나면 값이 해제된다", font_size=32),
        ).arrange(DOWN, buff=0.5, aligned_edge=LEFT)
        rules.next_to(rules_title, DOWN, buff=1)

        for rule in rules:
            self.play(Write(rule), run_time=1)
            self.wait(0.5)

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])


class OwnershipMove(Scene):
    """Episode 1: Move Semantics 시각화"""

    def construct(self):
        # Title
        title = Text("Move Semantics", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        # Create Stack and Heap sections
        stack_rect = Rectangle(width=4, height=5, color=STACK_BLUE)
        stack_rect.to_edge(LEFT, buff=1)
        stack_label = Text("STACK", font_size=24, color=STACK_BLUE)
        stack_label.next_to(stack_rect, UP)

        heap_rect = Rectangle(width=4, height=5, color=HEAP_PURPLE)
        heap_rect.to_edge(RIGHT, buff=1)
        heap_label = Text("HEAP", font_size=24, color=HEAP_PURPLE)
        heap_label.next_to(heap_rect, UP)

        self.play(
            Create(stack_rect), Write(stack_label),
            Create(heap_rect), Write(heap_label)
        )

        # Step 1: Create s1
        code1 = Code(
            code='let s1 = String::from("hello");',
            language="rust",
            font_size=24,
            background="window"
        ).scale(0.7)
        code1.to_edge(DOWN)
        self.play(Create(code1))

        # s1 on stack
        s1_box = Rectangle(width=3, height=0.8, color=VALID_GREEN, fill_opacity=0.3)
        s1_box.move_to(stack_rect.get_center() + UP * 1.5)
        s1_text = Text("s1", font_size=20, color=VALID_GREEN)
        s1_text.move_to(s1_box)
        s1_group = VGroup(s1_box, s1_text)

        # "hello" on heap
        hello_box = Rectangle(width=3, height=0.8, color=HEAP_PURPLE, fill_opacity=0.3)
        hello_box.move_to(heap_rect.get_center() + UP * 1.5)
        hello_text = Text('"hello"', font_size=20, color=TEXT_WHITE)
        hello_text.move_to(hello_box)
        hello_group = VGroup(hello_box, hello_text)

        # Arrow from s1 to hello
        arrow1 = Arrow(
            s1_box.get_right(), hello_box.get_left(),
            color=VALID_GREEN, buff=0.1
        )

        self.play(Create(s1_group), Create(hello_group), Create(arrow1))
        self.wait(1)

        # Step 2: Move to s2
        self.play(FadeOut(code1))
        code2 = Code(
            code='let s2 = s1;  // s1 moves to s2',
            language="rust",
            font_size=24,
            background="window"
        ).scale(0.7)
        code2.to_edge(DOWN)
        self.play(Create(code2))

        # s2 on stack
        s2_box = Rectangle(width=3, height=0.8, color=VALID_GREEN, fill_opacity=0.3)
        s2_box.move_to(stack_rect.get_center() + DOWN * 0.5)
        s2_text = Text("s2", font_size=20, color=VALID_GREEN)
        s2_text.move_to(s2_box)
        s2_group = VGroup(s2_box, s2_text)

        # New arrow from s2 to hello
        arrow2 = Arrow(
            s2_box.get_right(), hello_box.get_left(),
            color=VALID_GREEN, buff=0.1
        )

        # Invalidate s1
        self.play(
            Create(s2_group),
            Create(arrow2),
            s1_box.animate.set_color(INVALID_RED).set_fill(INVALID_RED, opacity=0.2),
            s1_text.animate.set_color(INVALID_RED),
            FadeOut(arrow1)
        )

        # Cross out s1
        cross = Cross(s1_group, color=INVALID_RED)
        self.play(Create(cross))

        # Add "MOVED" label
        moved_label = Text("MOVED", font_size=16, color=INVALID_RED)
        moved_label.next_to(s1_box, RIGHT, buff=0.2)
        self.play(Write(moved_label))

        self.wait(1)

        # Step 3: Try to use s1 - ERROR!
        self.play(FadeOut(code2))
        code3 = Code(
            code='println!("{}", s1);  // ERROR!',
            language="rust",
            font_size=24,
            background="window"
        ).scale(0.7)
        code3.to_edge(DOWN)
        self.play(Create(code3))

        error_box = Rectangle(width=8, height=1, color=INVALID_RED, fill_opacity=0.2)
        error_box.next_to(code3, UP, buff=0.3)
        error_text = Text("error: value borrowed after move", font_size=18, color=INVALID_RED)
        error_text.move_to(error_box)

        self.play(Create(error_box), Write(error_text))
        self.wait(2)

        # Clean up
        self.play(*[FadeOut(mob) for mob in self.mobjects])


class OwnershipBorrow(Scene):
    """Episode 2: Borrowing 시각화"""

    def construct(self):
        # Title
        title = Text("References & Borrowing", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        # Create memory sections
        stack_rect = Rectangle(width=4, height=5, color=STACK_BLUE)
        stack_rect.to_edge(LEFT, buff=1)
        stack_label = Text("STACK", font_size=24, color=STACK_BLUE)
        stack_label.next_to(stack_rect, UP)

        heap_rect = Rectangle(width=4, height=5, color=HEAP_PURPLE)
        heap_rect.to_edge(RIGHT, buff=1)
        heap_label = Text("HEAP", font_size=24, color=HEAP_PURPLE)
        heap_label.next_to(heap_rect, UP)

        self.play(
            Create(stack_rect), Write(stack_label),
            Create(heap_rect), Write(heap_label)
        )

        # Create s1 (owner)
        s1_box = Rectangle(width=3, height=0.8, color=VALID_GREEN, fill_opacity=0.3)
        s1_box.move_to(stack_rect.get_center() + UP * 1.5)
        s1_text = Text("s1 (owner)", font_size=18, color=VALID_GREEN)
        s1_text.move_to(s1_box)

        hello_box = Rectangle(width=3, height=0.8, color=HEAP_PURPLE, fill_opacity=0.3)
        hello_box.move_to(heap_rect.get_center() + UP * 1)
        hello_text = Text('"hello"', font_size=20, color=TEXT_WHITE)
        hello_text.move_to(hello_box)

        arrow_owner = Arrow(s1_box.get_right(), hello_box.get_left(), color=VALID_GREEN, buff=0.1)

        self.play(Create(s1_box), Write(s1_text), Create(hello_box), Write(hello_text), Create(arrow_owner))

        code1 = Code(
            code='let s1 = String::from("hello");',
            language="rust",
            font_size=24,
            background="window"
        ).scale(0.7)
        code1.to_edge(DOWN)
        self.play(Create(code1))
        self.wait(1)

        # Create reference r1
        self.play(FadeOut(code1))
        code2 = Code(
            code='let r1 = &s1;  // immutable borrow',
            language="rust",
            font_size=24,
            background="window"
        ).scale(0.7)
        code2.to_edge(DOWN)
        self.play(Create(code2))

        r1_box = Rectangle(width=3, height=0.8, color=STACK_BLUE, fill_opacity=0.3)
        r1_box.move_to(stack_rect.get_center())
        r1_text = Text("r1 (&s1)", font_size=18, color=STACK_BLUE)
        r1_text.move_to(r1_box)

        # Dashed arrow (borrow, not own)
        arrow_r1 = DashedLine(
            r1_box.get_right(), s1_box.get_right() + RIGHT * 0.5,
            color=STACK_BLUE
        )
        arrow_r1_tip = Triangle(fill_color=STACK_BLUE, fill_opacity=1).scale(0.1)
        arrow_r1_tip.next_to(arrow_r1.get_end(), RIGHT, buff=0)

        self.play(Create(r1_box), Write(r1_text))

        # Arrow from r1 to s1
        borrow_arrow = CurvedArrow(
            r1_box.get_top(), s1_box.get_bottom(),
            color=STACK_BLUE, angle=-TAU/4
        )
        borrow_label = Text("borrows", font_size=14, color=STACK_BLUE)
        borrow_label.next_to(borrow_arrow, LEFT, buff=0.1)

        self.play(Create(borrow_arrow), Write(borrow_label))
        self.wait(1)

        # Multiple immutable borrows OK
        self.play(FadeOut(code2))
        code3 = Code(
            code='let r2 = &s1;  // multiple immutable borrows OK!',
            language="rust",
            font_size=24,
            background="window"
        ).scale(0.7)
        code3.to_edge(DOWN)
        self.play(Create(code3))

        r2_box = Rectangle(width=3, height=0.8, color=STACK_BLUE, fill_opacity=0.3)
        r2_box.move_to(stack_rect.get_center() + DOWN * 1.5)
        r2_text = Text("r2 (&s1)", font_size=18, color=STACK_BLUE)
        r2_text.move_to(r2_box)

        borrow_arrow2 = CurvedArrow(
            r2_box.get_top(), s1_box.get_bottom(),
            color=STACK_BLUE, angle=-TAU/6
        )

        self.play(Create(r2_box), Write(r2_text), Create(borrow_arrow2))

        # Show "OK" indicator
        ok_text = Text("Multiple &T OK!", font_size=24, color=VALID_GREEN)
        ok_text.move_to(ORIGIN)
        self.play(Write(ok_text))
        self.wait(2)

        self.play(*[FadeOut(mob) for mob in self.mobjects])


class OwnershipCopy(Scene):
    """Episode 3: Copy vs Clone"""

    def construct(self):
        title = Text("Copy vs Clone", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        # Left side: Copy (integers)
        copy_title = Text("Copy (Stack only)", font_size=28, color=STACK_BLUE)
        copy_title.move_to(LEFT * 3.5 + UP * 2)

        copy_code = Code(
            code='let x = 5;\nlet y = x;  // Copy!\nprintln!("{}, {}", x, y);  // Both work!',
            language="rust",
            font_size=18,
            background="window"
        ).scale(0.8)
        copy_code.move_to(LEFT * 3.5)

        # Right side: Clone (heap)
        clone_title = Text("Clone (Deep copy)", font_size=28, color=HEAP_PURPLE)
        clone_title.move_to(RIGHT * 3.5 + UP * 2)

        clone_code = Code(
            code='let s1 = String::from("hi");\nlet s2 = s1.clone();  // Explicit!\nprintln!("{}, {}", s1, s2);  // Both work!',
            language="rust",
            font_size=18,
            background="window"
        ).scale(0.8)
        clone_code.move_to(RIGHT * 3.5)

        self.play(Write(copy_title), Write(clone_title))
        self.play(Create(copy_code), Create(clone_code))

        # Visualize Copy
        x_box = Rectangle(width=1.5, height=0.6, color=STACK_BLUE, fill_opacity=0.3)
        x_box.move_to(LEFT * 3.5 + DOWN * 2)
        x_text = Text("x: 5", font_size=16)
        x_text.move_to(x_box)

        y_box = Rectangle(width=1.5, height=0.6, color=STACK_BLUE, fill_opacity=0.3)
        y_box.next_to(x_box, RIGHT, buff=0.5)
        y_text = Text("y: 5", font_size=16)
        y_text.move_to(y_box)

        copy_arrow = Arrow(x_box.get_right(), y_box.get_left(), color=VALID_GREEN, buff=0.1)
        copy_label = Text("bit copy", font_size=12, color=VALID_GREEN)
        copy_label.next_to(copy_arrow, UP, buff=0.1)

        self.play(Create(x_box), Write(x_text))
        self.play(Create(copy_arrow), Write(copy_label), Create(y_box), Write(y_text))

        # Visualize Clone
        s1_stack = Rectangle(width=1.5, height=0.6, color=STACK_BLUE, fill_opacity=0.3)
        s1_stack.move_to(RIGHT * 2.5 + DOWN * 1.8)
        s1_label = Text("s1", font_size=14)
        s1_label.move_to(s1_stack)

        s1_heap = Rectangle(width=1.2, height=0.5, color=HEAP_PURPLE, fill_opacity=0.3)
        s1_heap.move_to(RIGHT * 2.5 + DOWN * 2.8)
        s1_data = Text('"hi"', font_size=12)
        s1_data.move_to(s1_heap)

        s2_stack = Rectangle(width=1.5, height=0.6, color=STACK_BLUE, fill_opacity=0.3)
        s2_stack.move_to(RIGHT * 4.5 + DOWN * 1.8)
        s2_label = Text("s2", font_size=14)
        s2_label.move_to(s2_stack)

        s2_heap = Rectangle(width=1.2, height=0.5, color=HEAP_PURPLE, fill_opacity=0.3)
        s2_heap.move_to(RIGHT * 4.5 + DOWN * 2.8)
        s2_data = Text('"hi"', font_size=12)
        s2_data.move_to(s2_heap)

        self.play(
            Create(s1_stack), Write(s1_label),
            Create(s1_heap), Write(s1_data)
        )

        clone_arrow = Arrow(s1_heap.get_right(), s2_heap.get_left(), color=HEAP_PURPLE, buff=0.1)
        clone_label2 = Text("deep copy", font_size=12, color=HEAP_PURPLE)
        clone_label2.next_to(clone_arrow, UP, buff=0.1)

        self.play(
            Create(clone_arrow), Write(clone_label2),
            Create(s2_stack), Write(s2_label),
            Create(s2_heap), Write(s2_data)
        )

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])


class FullOwnershipDemo(Scene):
    """Complete ownership demonstration"""

    def construct(self):
        # Opening
        logo = Text("WIA-RUST-LEARN", font_size=64, color=RUST_ORANGE)
        tagline = Text("Rust isn't hard - it's just different", font_size=28)
        tagline.next_to(logo, DOWN)

        self.play(Write(logo), run_time=2)
        self.play(FadeIn(tagline))
        self.wait(2)

        self.play(FadeOut(logo), FadeOut(tagline))

        # NRT Concept (user's idea!)
        nrt_title = Text("NRT: Non-Repeatable Transfer", font_size=48, color=RUST_ORANGE)
        nrt_subtitle = Text("소유권은 NFT처럼 단 하나만 존재한다", font_size=28)
        nrt_subtitle.next_to(nrt_title, DOWN)

        self.play(Write(nrt_title))
        self.play(FadeIn(nrt_subtitle))
        self.wait(2)

        # Show the analogy
        analogy = VGroup(
            Text("NFT: 디지털 자산의 유일한 소유권", font_size=24),
            Text("Rust: 메모리 데이터의 유일한 소유권", font_size=24, color=RUST_ORANGE),
        ).arrange(DOWN, buff=0.5)
        analogy.next_to(nrt_subtitle, DOWN, buff=1)

        self.play(Write(analogy[0]))
        self.wait(0.5)
        self.play(Write(analogy[1]))
        self.wait(2)

        # Ending
        self.play(*[FadeOut(mob) for mob in self.mobjects])

        ending = Text("弘益人間 - Benefit All Humanity", font_size=36, color=RUST_ORANGE)
        self.play(Write(ending))
        self.wait(2)


# Run instructions
if __name__ == "__main__":
    print("""
    =============================================
    WIA-RUST-LEARN Manim Animations
    =============================================

    Install Manim:
        pip install manim

    Render individual scenes:
        manim -pql ownership_animation.py OwnershipIntro
        manim -pql ownership_animation.py OwnershipMove
        manim -pql ownership_animation.py OwnershipBorrow
        manim -pql ownership_animation.py OwnershipCopy
        manim -pql ownership_animation.py FullOwnershipDemo

    Render all scenes in high quality:
        manim -pqh ownership_animation.py

    Options:
        -p : Preview (play after rendering)
        -ql : Low quality (480p, faster)
        -qm : Medium quality (720p)
        -qh : High quality (1080p)
        -qk : 4K quality

    Output: media/videos/ownership_animation/
    =============================================
    """)
