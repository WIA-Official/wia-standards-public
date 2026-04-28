"""
WIA-RUST-LEARN: Level 0 - Setup & Introduction
===============================================
EP 0-01: Why Rust? (15분)
EP 0-02: Installing Rust (12분)
EP 0-03: Cargo Deep Dive (18분)

Run: manim -pqh level_0_setup.py EP_0_01_WhyRust
"""

from manim import *

# WIA Brand Colors
RUST_ORANGE = "#f74c00"
RUST_DARK = "#1a1a2e"
STACK_BLUE = "#3498db"
HEAP_PURPLE = "#9b59b6"
VALID_GREEN = "#00d26a"
INVALID_RED = "#ff4757"
TEXT_WHITE = "#e6e6e6"
CODE_BG = "#1e1e1e"


class WIAIntro(Scene):
    """Common intro for all episodes"""

    def create_intro(self, episode_num, title, subtitle):
        # WIA Logo
        logo = Text("WIA-RUST-LEARN", font_size=48, color=RUST_ORANGE)
        logo.to_edge(UP, buff=1)

        # Episode number
        ep_badge = VGroup(
            Rectangle(width=3, height=1, color=RUST_ORANGE, fill_opacity=0.8),
            Text(f"EP {episode_num}", font_size=32, color=TEXT_WHITE)
        )
        ep_badge[1].move_to(ep_badge[0])
        ep_badge.next_to(logo, DOWN, buff=0.5)

        # Title
        title_text = Text(title, font_size=56, color=TEXT_WHITE)
        title_text.next_to(ep_badge, DOWN, buff=0.8)

        # Subtitle
        sub_text = Text(subtitle, font_size=28, color=TEXT_WHITE, opacity=0.7)
        sub_text.next_to(title_text, DOWN, buff=0.3)

        self.play(Write(logo), run_time=1)
        self.play(FadeIn(ep_badge, scale=0.8))
        self.play(Write(title_text), run_time=1.5)
        self.play(FadeIn(sub_text))
        self.wait(2)

        self.play(*[FadeOut(mob) for mob in self.mobjects])


class EP_0_01_WhyRust(Scene):
    """EP 0-01: Why Rust? - Safety Without Sacrifice (15분)"""

    def construct(self):
        # Intro
        self.intro()

        # Section 1: The Problem with C/C++
        self.section_c_problems()

        # Section 2: The Problem with GC Languages
        self.section_gc_problems()

        # Section 3: Rust's Solution
        self.section_rust_solution()

        # Section 4: Zero-Cost Abstractions
        self.section_zero_cost()

        # Outro
        self.outro()

    def intro(self):
        logo = Text("WIA-RUST-LEARN", font_size=48, color=RUST_ORANGE)
        logo.to_edge(UP, buff=1)

        ep_text = Text("EP 0-01", font_size=36, color=RUST_ORANGE)
        ep_text.next_to(logo, DOWN, buff=0.5)

        title = Text("Why Rust?", font_size=64, color=TEXT_WHITE)
        title.next_to(ep_text, DOWN, buff=0.5)

        subtitle = Text("Safety Without Sacrifice", font_size=32, color=TEXT_WHITE, opacity=0.7)
        subtitle.next_to(title, DOWN, buff=0.3)

        self.play(Write(logo))
        self.play(FadeIn(ep_text))
        self.play(Write(title), run_time=1.5)
        self.play(FadeIn(subtitle))
        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_c_problems(self):
        title = Text("C/C++의 문제점", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        # Memory bugs
        problems = VGroup(
            self.create_problem_card("Use After Free", "해제된 메모리 접근"),
            self.create_problem_card("Double Free", "같은 메모리 두 번 해제"),
            self.create_problem_card("Buffer Overflow", "배열 범위 초과"),
            self.create_problem_card("Null Pointer", "널 포인터 역참조"),
        ).arrange_in_grid(rows=2, cols=2, buff=0.5)
        problems.next_to(title, DOWN, buff=1)

        for problem in problems:
            self.play(FadeIn(problem, scale=0.9), run_time=0.5)

        # Statistics
        stat_text = Text("Microsoft: 보안 취약점의 70%가 메모리 안전 문제",
                        font_size=24, color=INVALID_RED)
        stat_text.to_edge(DOWN, buff=1)
        self.play(Write(stat_text))

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def create_problem_card(self, title, desc):
        card = VGroup(
            Rectangle(width=4, height=2, color=INVALID_RED, fill_opacity=0.2),
            Text(title, font_size=24, color=INVALID_RED),
            Text(desc, font_size=18, color=TEXT_WHITE, opacity=0.7),
        )
        card[1].move_to(card[0].get_center() + UP * 0.3)
        card[2].move_to(card[0].get_center() + DOWN * 0.3)
        return card

    def section_gc_problems(self):
        title = Text("GC 언어의 트레이드오프", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        # Languages with GC
        gc_langs = VGroup(
            Text("Java", font_size=32, color=STACK_BLUE),
            Text("Python", font_size=32, color=STACK_BLUE),
            Text("Go", font_size=32, color=STACK_BLUE),
            Text("JavaScript", font_size=32, color=STACK_BLUE),
        ).arrange(RIGHT, buff=1)
        gc_langs.next_to(title, DOWN, buff=1)

        self.play(Write(gc_langs))

        # Trade-offs
        tradeoffs = VGroup(
            Text("✓ 메모리 안전", font_size=28, color=VALID_GREEN),
            Text("✗ 런타임 오버헤드", font_size=28, color=INVALID_RED),
            Text("✗ 예측 불가능한 GC 일시정지", font_size=28, color=INVALID_RED),
            Text("✗ 메모리 사용량 증가", font_size=28, color=INVALID_RED),
        ).arrange(DOWN, buff=0.3, aligned_edge=LEFT)
        tradeoffs.next_to(gc_langs, DOWN, buff=1)

        for t in tradeoffs:
            self.play(Write(t), run_time=0.5)

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_rust_solution(self):
        title = Text("Rust의 해결책", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        # The magic
        ownership = Text("소유권 시스템", font_size=56, color=RUST_ORANGE)
        ownership.next_to(title, DOWN, buff=1)

        self.play(Write(ownership), run_time=1.5)

        # Benefits
        benefits = VGroup(
            Text("✓ 컴파일 타임에 메모리 안전 보장", font_size=28, color=VALID_GREEN),
            Text("✓ GC 없음 - 제로 런타임 오버헤드", font_size=28, color=VALID_GREEN),
            Text("✓ 데이터 레이스 컴파일 타임 방지", font_size=28, color=VALID_GREEN),
            Text("✓ C/C++ 수준의 성능", font_size=28, color=VALID_GREEN),
        ).arrange(DOWN, buff=0.4, aligned_edge=LEFT)
        benefits.next_to(ownership, DOWN, buff=1)

        for b in benefits:
            self.play(Write(b), run_time=0.6)

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_zero_cost(self):
        title = Text("Zero-Cost Abstractions", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        quote = Text(
            '"What you don\'t use, you don\'t pay for.\n'
            'What you do use, you couldn\'t hand code any better."',
            font_size=24, color=TEXT_WHITE, opacity=0.8
        )
        quote.next_to(title, DOWN, buff=0.5)

        author = Text("- Bjarne Stroustrup (C++ 창시자)", font_size=20, color=TEXT_WHITE, opacity=0.5)
        author.next_to(quote, DOWN, buff=0.3)

        self.play(Write(quote), run_time=2)
        self.play(FadeIn(author))

        # Rust applies this
        rust_applies = Text("Rust는 이 원칙을 메모리 안전에도 적용",
                           font_size=32, color=RUST_ORANGE)
        rust_applies.next_to(author, DOWN, buff=1)

        self.play(Write(rust_applies))
        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def outro(self):
        # Key takeaway
        takeaway = Text("Rust isn't hard - it's just different",
                       font_size=48, color=RUST_ORANGE)

        subtitle = Text("어렵지 않다. 다를 뿐이다.", font_size=32, color=TEXT_WHITE)
        subtitle.next_to(takeaway, DOWN, buff=0.5)

        self.play(Write(takeaway), run_time=1.5)
        self.play(FadeIn(subtitle))
        self.wait(2)

        # Next episode
        next_ep = Text("다음: EP 0-02 Installing Rust", font_size=28, color=TEXT_WHITE, opacity=0.7)
        next_ep.to_edge(DOWN, buff=1)
        self.play(FadeIn(next_ep))
        self.wait(2)


class EP_0_02_InstallingRust(Scene):
    """EP 0-02: Installing Rust (12분)"""

    def construct(self):
        self.intro()
        self.section_rustup()
        self.section_toolchain()
        self.section_hello_world()
        self.section_vscode()
        self.outro()

    def intro(self):
        logo = Text("WIA-RUST-LEARN", font_size=48, color=RUST_ORANGE)
        logo.to_edge(UP, buff=1)

        ep_text = Text("EP 0-02", font_size=36, color=RUST_ORANGE)
        title = Text("Installing Rust", font_size=64, color=TEXT_WHITE)
        subtitle = Text("rustup, cargo, and Your First Program", font_size=28, opacity=0.7)

        VGroup(ep_text, title, subtitle).arrange(DOWN, buff=0.4).next_to(logo, DOWN, buff=0.8)

        self.play(Write(logo), FadeIn(ep_text), Write(title), FadeIn(subtitle))
        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_rustup(self):
        title = Text("rustup 설치", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        # Command
        cmd_box = Rectangle(width=12, height=1.5, color=CODE_BG, fill_opacity=0.9)
        cmd_box.next_to(title, DOWN, buff=1)

        cmd_text = Text("curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh",
                       font_size=20, font="monospace", color=VALID_GREEN)
        cmd_text.move_to(cmd_box)

        self.play(Create(cmd_box), Write(cmd_text))

        # What it installs
        installs = VGroup(
            Text("• rustc - Rust 컴파일러", font_size=24),
            Text("• cargo - 패키지 매니저 & 빌드 툴", font_size=24),
            Text("• rustfmt - 코드 포매터", font_size=24),
            Text("• clippy - 린터", font_size=24),
        ).arrange(DOWN, buff=0.3, aligned_edge=LEFT)
        installs.next_to(cmd_box, DOWN, buff=1)

        for item in installs:
            self.play(Write(item), run_time=0.5)

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_toolchain(self):
        title = Text("Toolchain 확인", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        commands = VGroup(
            self.create_command("rustc --version", "rustc 1.75.0 (82e1608df 2023-12-21)"),
            self.create_command("cargo --version", "cargo 1.75.0 (1d8b05cdd 2023-11-20)"),
            self.create_command("rustup show", "stable-x86_64-apple-darwin (default)"),
        ).arrange(DOWN, buff=0.8)
        commands.next_to(title, DOWN, buff=1)

        for cmd in commands:
            self.play(FadeIn(cmd), run_time=0.8)

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def create_command(self, cmd, output):
        group = VGroup(
            Text(f"$ {cmd}", font_size=22, font="monospace", color=VALID_GREEN),
            Text(output, font_size=20, font="monospace", color=TEXT_WHITE, opacity=0.7),
        ).arrange(DOWN, buff=0.2, aligned_edge=LEFT)
        return group

    def section_hello_world(self):
        title = Text("Hello, World!", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        # Create project
        cmd1 = Text("$ cargo new hello_rust", font_size=24, font="monospace", color=VALID_GREEN)
        cmd1.next_to(title, DOWN, buff=0.8)
        self.play(Write(cmd1))

        # Code
        code_box = Rectangle(width=10, height=4, color=CODE_BG, fill_opacity=0.9)
        code_box.next_to(cmd1, DOWN, buff=0.5)

        code = Text(
            'fn main() {\n'
            '    println!("Hello, World!");\n'
            '}',
            font_size=24, font="monospace", color=TEXT_WHITE
        )
        code.move_to(code_box)

        self.play(Create(code_box), Write(code))

        # Run
        cmd2 = Text("$ cargo run", font_size=24, font="monospace", color=VALID_GREEN)
        cmd2.next_to(code_box, DOWN, buff=0.5)

        output = Text("Hello, World!", font_size=28, color=RUST_ORANGE)
        output.next_to(cmd2, DOWN, buff=0.3)

        self.play(Write(cmd2))
        self.play(Write(output))

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_vscode(self):
        title = Text("VS Code 설정", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        extensions = VGroup(
            self.create_extension("rust-analyzer", "공식 Rust 언어 서버"),
            self.create_extension("CodeLLDB", "디버거"),
            self.create_extension("Even Better TOML", "Cargo.toml 지원"),
        ).arrange(DOWN, buff=0.5)
        extensions.next_to(title, DOWN, buff=1)

        for ext in extensions:
            self.play(FadeIn(ext), run_time=0.6)

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def create_extension(self, name, desc):
        return VGroup(
            Rectangle(width=8, height=1.2, color=STACK_BLUE, fill_opacity=0.2),
            Text(name, font_size=28, color=STACK_BLUE),
            Text(desc, font_size=18, color=TEXT_WHITE, opacity=0.7),
        ).arrange(DOWN, buff=0.1)

    def outro(self):
        done = Text("✓ Rust 개발 환경 완료!", font_size=48, color=VALID_GREEN)
        next_ep = Text("다음: EP 0-03 Cargo Deep Dive", font_size=28, opacity=0.7)
        next_ep.next_to(done, DOWN, buff=1)

        self.play(Write(done))
        self.play(FadeIn(next_ep))
        self.wait(2)


class EP_0_03_CargoDeepDive(Scene):
    """EP 0-03: Cargo Deep Dive (18분)"""

    def construct(self):
        self.intro()
        self.section_cargo_toml()
        self.section_dependencies()
        self.section_commands()
        self.section_project_structure()
        self.outro()

    def intro(self):
        logo = Text("WIA-RUST-LEARN", font_size=48, color=RUST_ORANGE)
        ep_text = Text("EP 0-03", font_size=36, color=RUST_ORANGE)
        title = Text("Cargo Deep Dive", font_size=64, color=TEXT_WHITE)
        subtitle = Text("Building, Testing, Publishing", font_size=28, opacity=0.7)

        VGroup(logo, ep_text, title, subtitle).arrange(DOWN, buff=0.4)

        self.play(Write(logo), FadeIn(ep_text), Write(title), FadeIn(subtitle))
        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_cargo_toml(self):
        title = Text("Cargo.toml 구조", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        toml_code = '''[package]
name = "my_project"
version = "0.1.0"
edition = "2021"

[dependencies]
serde = "1.0"
tokio = { version = "1", features = ["full"] }

[dev-dependencies]
criterion = "0.5"'''

        code_text = Text(toml_code, font_size=20, font="monospace", color=TEXT_WHITE)
        code_text.next_to(title, DOWN, buff=0.5)

        self.play(Write(code_text), run_time=2)

        # Annotations
        annotations = VGroup(
            Text("← 프로젝트 메타데이터", font_size=16, color=RUST_ORANGE),
            Text("← 런타임 의존성", font_size=16, color=VALID_GREEN),
            Text("← 테스트 전용 의존성", font_size=16, color=STACK_BLUE),
        )

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_dependencies(self):
        title = Text("의존성 관리", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        # crates.io
        crates_logo = Text("crates.io", font_size=40, color=RUST_ORANGE)
        crates_logo.next_to(title, DOWN, buff=0.8)
        desc = Text("Rust 공식 패키지 레지스트리", font_size=24, opacity=0.7)
        desc.next_to(crates_logo, DOWN, buff=0.3)

        self.play(Write(crates_logo), FadeIn(desc))

        # Popular crates
        crates = VGroup(
            Text("serde - 직렬화/역직렬화", font_size=22),
            Text("tokio - 비동기 런타임", font_size=22),
            Text("reqwest - HTTP 클라이언트", font_size=22),
            Text("clap - CLI 파싱", font_size=22),
        ).arrange(DOWN, buff=0.3, aligned_edge=LEFT)
        crates.next_to(desc, DOWN, buff=0.8)

        for crate in crates:
            self.play(Write(crate), run_time=0.4)

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def section_commands(self):
        title = Text("주요 Cargo 명령어", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        commands = VGroup(
            self.create_cmd_row("cargo build", "컴파일"),
            self.create_cmd_row("cargo run", "컴파일 + 실행"),
            self.create_cmd_row("cargo test", "테스트 실행"),
            self.create_cmd_row("cargo doc --open", "문서 생성"),
            self.create_cmd_row("cargo clippy", "린트 검사"),
            self.create_cmd_row("cargo fmt", "코드 포매팅"),
        ).arrange(DOWN, buff=0.3)
        commands.next_to(title, DOWN, buff=0.8)

        for cmd in commands:
            self.play(FadeIn(cmd), run_time=0.4)

        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def create_cmd_row(self, cmd, desc):
        return VGroup(
            Text(cmd, font_size=24, font="monospace", color=VALID_GREEN),
            Text(f"  →  {desc}", font_size=22, color=TEXT_WHITE, opacity=0.7),
        ).arrange(RIGHT, buff=0.5)

    def section_project_structure(self):
        title = Text("프로젝트 구조", font_size=48, color=RUST_ORANGE)
        title.to_edge(UP)
        self.play(Write(title))

        structure = '''my_project/
├── Cargo.toml
├── Cargo.lock
├── src/
│   ├── main.rs      # 바이너리 진입점
│   └── lib.rs       # 라이브러리 진입점
├── tests/           # 통합 테스트
├── benches/         # 벤치마크
└── examples/        # 예제 코드'''

        struct_text = Text(structure, font_size=20, font="monospace", color=TEXT_WHITE)
        struct_text.next_to(title, DOWN, buff=0.5)

        self.play(Write(struct_text), run_time=2)
        self.wait(2)
        self.play(*[FadeOut(mob) for mob in self.mobjects])

    def outro(self):
        done = Text("Level 0 완료!", font_size=56, color=VALID_GREEN)

        summary = VGroup(
            Text("✓ Rust가 왜 특별한지 이해", font_size=24),
            Text("✓ 개발 환경 설정 완료", font_size=24),
            Text("✓ Cargo로 프로젝트 관리", font_size=24),
        ).arrange(DOWN, buff=0.3, aligned_edge=LEFT)
        summary.next_to(done, DOWN, buff=1)

        next_level = Text("다음: Level 1 - Rust Basics", font_size=28, color=RUST_ORANGE)
        next_level.next_to(summary, DOWN, buff=1)

        self.play(Write(done))
        self.play(FadeIn(summary))
        self.play(FadeIn(next_level))
        self.wait(3)
