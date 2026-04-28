"""
WIA-RUST-LEARN Level 5: 동시성 (Concurrency)
============================================

EP 5-01: Threads & Channels (스레드와 채널) - 22분
EP 5-02: Async/Await (비동기 프로그래밍) - 20분

실행 방법:
    manim -pql level_5_concurrency.py EP_5_01_ThreadsChannels
    manim -pql level_5_concurrency.py EP_5_02_AsyncAwait

전체 렌더링 (고화질):
    manim -pqh level_5_concurrency.py --all

🎉 축하합니다! 전체 19편 시리즈 완료!
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
THREAD_TEAL = "#009688"
ASYNC_PINK = "#E91E63"
DARK_BG = "#1a1a2e"


class EP_5_01_ThreadsChannels(Scene):
    """
    EP 5-01: Threads & Channels - 공포 없는 동시성

    학습 목표:
    1. std::thread로 스레드 생성
    2. move 클로저로 데이터 전달
    3. Arc<Mutex<T>>로 공유 상태
    4. mpsc 채널로 메시지 전달
    """

    def construct(self):
        self.camera.background_color = DARK_BG

        self.section_intro()
        self.wait(0.5)
        self.clear()

        self.section_thread_basics()
        self.wait(0.5)
        self.clear()

        self.section_move_and_arc()
        self.wait(0.5)
        self.clear()

        self.section_mutex()
        self.wait(0.5)
        self.clear()

        self.section_channels()
        self.wait(0.5)
        self.clear()

        self.section_fearless_concurrency()
        self.wait(0.5)
        self.clear()

        self.section_outro()

    def section_intro(self):
        title = Text("EP 5-01", font_size=32, color=GRAY).to_edge(UP)
        main_title = Text("Threads & Channels", font_size=56, color=RUST_ORANGE)
        subtitle = Text("공포 없는 동시성 (Fearless Concurrency)", font_size=28, color=GRAY)

        main_title.next_to(title, DOWN, buff=0.8)
        subtitle.next_to(main_title, DOWN, buff=0.3)

        self.play(Write(title), Write(main_title))
        self.play(FadeIn(subtitle))
        self.wait(1)

        # 동시성의 어려움
        problems = VGroup(
            Text("동시성 프로그래밍의 악몽들:", font_size=24, color=ERROR_RED),
            VGroup(
                Text("• Data Race (데이터 경쟁)", font_size=18, color=GRAY),
                Text("• Deadlock (교착 상태)", font_size=18, color=GRAY),
                Text("• Race Condition (경쟁 상태)", font_size=18, color=GRAY),
            ).arrange(DOWN, aligned_edge=LEFT, buff=0.1),
        ).arrange(DOWN, buff=0.3)
        problems.next_to(subtitle, DOWN, buff=0.8)

        self.play(FadeIn(problems, shift=UP))
        self.wait(1)

        # Rust의 해결책
        solution = VGroup(
            Text("Rust의 답:", font_size=24, color=SUCCESS_GREEN),
            Text("소유권 시스템이 컴파일 타임에 방지!", font_size=20, color=WHITE),
        ).arrange(DOWN, buff=0.2)
        solution.next_to(problems, DOWN, buff=0.5)

        self.play(FadeIn(solution, scale=1.1))
        self.wait(2)

    def section_thread_basics(self):
        title = Text("스레드 생성하기", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''use std::thread;
use std::time::Duration;

fn main() {
    // 스레드 생성
    let handle = thread::spawn(|| {
        for i in 1..5 {
            println!("스레드: {}", i);
            thread::sleep(Duration::from_millis(100));
        }
    });

    // 메인 스레드 작업
    for i in 1..3 {
        println!("메인: {}", i);
        thread::sleep(Duration::from_millis(100));
    }

    // 스레드 종료 대기
    handle.join().unwrap();
}''',
            language="rust",
            font_size=14,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.7)
        code.next_to(title, DOWN, buff=0.3).to_edge(LEFT, buff=0.5)

        self.play(Create(code))
        self.wait(1)

        # 스레드 시각화
        main_thread = self.create_thread_viz("Main", STACK_GREEN)
        spawned_thread = self.create_thread_viz("Spawned", THREAD_TEAL)

        threads = VGroup(main_thread, spawned_thread).arrange(DOWN, buff=0.5)
        threads.to_edge(RIGHT, buff=0.8)

        self.play(Create(threads))
        self.wait(1)

        # join 설명
        join_note = VGroup(
            Text("join()", font_size=20, color=WARNING_YELLOW),
            Text("= 스레드 종료까지 대기", font_size=16, color=GRAY),
        ).arrange(RIGHT, buff=0.3)
        join_note.to_edge(DOWN, buff=0.5)

        self.play(Write(join_note))
        self.wait(2)

    def create_thread_viz(self, name, color):
        rect = RoundedRectangle(
            width=3, height=0.8, corner_radius=0.1,
            fill_color=color, fill_opacity=0.3,
            stroke_color=color
        )
        label = Text(name, font_size=18, color=color)
        label.move_to(rect)

        # 실행 상태 표시
        dots = VGroup(*[
            Circle(radius=0.1, fill_color=WHITE, fill_opacity=0.5)
            for _ in range(5)
        ]).arrange(RIGHT, buff=0.1)
        dots.next_to(rect, RIGHT, buff=0.2)

        return VGroup(rect, label, dots)

    def section_move_and_arc(self):
        title = Text("데이터 공유: move와 Arc", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # move 클로저
        move_code = Code(
            code='''// move: 소유권 이전
let data = vec![1, 2, 3];

let handle = thread::spawn(move || {
    println!("{:?}", data);  // data 소유권 이전됨
});

// println!("{:?}", data);  // ✗ 이미 이동됨!''',
            language="rust",
            font_size=14,
            background="rectangle",
        ).scale(0.75)
        move_code.next_to(title, DOWN, buff=0.3).to_edge(LEFT, buff=0.5)

        self.play(Create(move_code))
        self.wait(1)

        # Arc
        arc_code = Code(
            code='''use std::sync::Arc;

// Arc: 스레드 안전한 공유
let data = Arc::new(vec![1, 2, 3]);

let handles: Vec<_> = (0..3).map(|i| {
    let data = Arc::clone(&data);  // 참조 카운트 증가
    thread::spawn(move || {
        println!("스레드 {}: {:?}", i, data);
    })
}).collect();

for h in handles { h.join().unwrap(); }''',
            language="rust",
            font_size=14,
            background="rectangle",
            background_stroke_color=SUCCESS_GREEN,
        ).scale(0.75)
        arc_code.next_to(move_code, DOWN, buff=0.3).to_edge(LEFT, buff=0.5)

        self.play(Create(arc_code))
        self.wait(1)

        # Rc vs Arc
        comparison = VGroup(
            VGroup(
                Text("Rc<T>", font_size=22, color=MEMORY_BLUE),
                Text("단일 스레드", font_size=14, color=GRAY),
            ).arrange(DOWN, buff=0.1),
            Text("vs", font_size=18, color=WHITE),
            VGroup(
                Text("Arc<T>", font_size=22, color=THREAD_TEAL),
                Text("멀티 스레드", font_size=14, color=GRAY),
                Text("(Atomic Reference Count)", font_size=12, color=GRAY),
            ).arrange(DOWN, buff=0.1),
        ).arrange(RIGHT, buff=0.8)
        comparison.to_edge(RIGHT, buff=0.5).shift(UP * 0.5)

        self.play(FadeIn(comparison, shift=LEFT))
        self.wait(2)

    def section_mutex(self):
        title = Text("Mutex<T>: 상호 배제", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''use std::sync::{Arc, Mutex};

let counter = Arc::new(Mutex::new(0));
let mut handles = vec![];

for _ in 0..10 {
    let counter = Arc::clone(&counter);
    let handle = thread::spawn(move || {
        // 락 획득 (다른 스레드 차단)
        let mut num = counter.lock().unwrap();
        *num += 1;
        // 스코프 끝 → 자동 락 해제
    });
    handles.push(handle);
}

for h in handles { h.join().unwrap(); }
println!("Result: {}", *counter.lock().unwrap()); // 10''',
            language="rust",
            font_size=14,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.7)
        code.next_to(title, DOWN, buff=0.3).to_edge(LEFT, buff=0.5)

        self.play(Create(code))
        self.wait(1)

        # Mutex 시각화
        mutex_box = RoundedRectangle(
            width=2, height=1.5, corner_radius=0.2,
            fill_color=WARNING_YELLOW, fill_opacity=0.2,
            stroke_color=WARNING_YELLOW
        )
        mutex_label = VGroup(
            Text("Mutex", font_size=18, color=WARNING_YELLOW),
            Text("counter: 0", font_size=14, color=WHITE),
        ).arrange(DOWN, buff=0.1)
        mutex_label.move_to(mutex_box)

        mutex_viz = VGroup(mutex_box, mutex_label)
        mutex_viz.to_edge(RIGHT, buff=1).shift(UP * 0.5)

        # 스레드들
        threads = VGroup(*[
            Circle(radius=0.3, fill_color=THREAD_TEAL, fill_opacity=0.5)
            for _ in range(3)
        ]).arrange(DOWN, buff=0.3)
        threads.next_to(mutex_viz, LEFT, buff=1)

        thread_labels = VGroup(
            Text("T1", font_size=12, color=WHITE),
            Text("T2", font_size=12, color=WHITE),
            Text("T3", font_size=12, color=WHITE),
        )
        for t, l in zip(threads, thread_labels):
            l.move_to(t)

        # 락 표시 (첫 번째 스레드만 연결)
        lock_arrow = Arrow(
            threads[0].get_right(), mutex_box.get_left(),
            color=SUCCESS_GREEN, buff=0.1
        )
        lock_label = Text("🔒 lock", font_size=12, color=SUCCESS_GREEN)
        lock_label.next_to(lock_arrow, UP, buff=0.05)

        # 대기 중
        wait_label = Text("⏳ waiting...", font_size=12, color=GRAY)
        wait_label.next_to(threads[1], RIGHT, buff=0.2)

        self.play(
            Create(mutex_viz),
            Create(threads),
            FadeIn(thread_labels),
        )
        self.wait(0.5)
        self.play(Create(lock_arrow), Write(lock_label), Write(wait_label))
        self.wait(2)

    def section_channels(self):
        title = Text("mpsc 채널: 메시지 전달", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''use std::sync::mpsc;

// 채널 생성 (tx: 송신, rx: 수신)
let (tx, rx) = mpsc::channel();

// 여러 송신자
let tx2 = tx.clone();

thread::spawn(move || {
    tx.send("from thread 1").unwrap();
});

thread::spawn(move || {
    tx2.send("from thread 2").unwrap();
});

// 메시지 수신
for received in rx {
    println!("Got: {}", received);
}''',
            language="rust",
            font_size=14,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.7)
        code.next_to(title, DOWN, buff=0.3).to_edge(LEFT, buff=0.5)

        self.play(Create(code))
        self.wait(1)

        # 채널 시각화
        # Sender들
        senders = VGroup(
            self.create_endpoint("T1", THREAD_TEAL),
            self.create_endpoint("T2", THREAD_TEAL),
        ).arrange(DOWN, buff=0.5)

        # 채널
        channel = Rectangle(
            width=2.5, height=0.6,
            fill_color=MEMORY_BLUE, fill_opacity=0.2,
            stroke_color=MEMORY_BLUE
        )
        channel_label = Text("channel", font_size=14, color=MEMORY_BLUE)
        channel_label.next_to(channel, UP, buff=0.1)

        # 메시지들
        messages = VGroup(
            self.create_message("msg1"),
            self.create_message("msg2"),
        ).arrange(RIGHT, buff=0.2)
        messages.move_to(channel)

        channel_group = VGroup(channel, channel_label, messages)

        # Receiver
        receiver = self.create_endpoint("Main", SUCCESS_GREEN)

        # 배치
        senders.to_edge(RIGHT, buff=2.5)
        channel_group.next_to(senders, RIGHT, buff=0.5)
        receiver.next_to(channel_group, RIGHT, buff=0.5)

        # 화살표
        arrows_send = VGroup(*[
            Arrow(s.get_right(), channel.get_left(), color=THREAD_TEAL, buff=0.1)
            for s in senders
        ])
        arrow_recv = Arrow(
            channel.get_right(), receiver.get_left(),
            color=SUCCESS_GREEN, buff=0.1
        )

        viz = VGroup(senders, channel_group, receiver, arrows_send, arrow_recv)
        viz.shift(UP * 0.5)

        self.play(Create(viz))
        self.wait(1)

        # mpsc 설명
        mpsc_note = VGroup(
            Text("mpsc = ", font_size=20, color=RUST_ORANGE),
            Text("Multiple Producer, Single Consumer", font_size=18, color=WHITE),
        ).arrange(RIGHT, buff=0.2)
        mpsc_note.to_edge(DOWN, buff=0.5)

        self.play(Write(mpsc_note))
        self.wait(2)

    def create_endpoint(self, name, color):
        circle = Circle(radius=0.4, fill_color=color, fill_opacity=0.3, stroke_color=color)
        label = Text(name, font_size=14, color=WHITE)
        label.move_to(circle)
        return VGroup(circle, label)

    def create_message(self, text):
        rect = RoundedRectangle(
            width=0.8, height=0.4, corner_radius=0.05,
            fill_color=WARNING_YELLOW, fill_opacity=0.5
        )
        label = Text(text, font_size=10, color=WHITE)
        label.move_to(rect)
        return VGroup(rect, label)

    def section_fearless_concurrency(self):
        title = Text("공포 없는 동시성", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # Rust가 방지하는 것들
        prevented = VGroup(
            Text("컴파일러가 방지하는 버그들:", font_size=24, color=SUCCESS_GREEN),
            VGroup(
                self.create_check_item("Data Race", "소유권 + Send/Sync 트레이트"),
                self.create_check_item("Use After Free", "수명 검사"),
                self.create_check_item("Double Free", "단일 소유권"),
            ).arrange(DOWN, buff=0.3, aligned_edge=LEFT),
        ).arrange(DOWN, buff=0.5)
        prevented.next_to(title, DOWN, buff=0.5)

        self.play(FadeIn(prevented, shift=UP))
        self.wait(1)

        # Send와 Sync
        traits = VGroup(
            VGroup(
                Text("Send", font_size=24, color=THREAD_TEAL),
                Text("스레드 간 소유권 이전 가능", font_size=16, color=GRAY),
            ).arrange(DOWN, buff=0.1),
            VGroup(
                Text("Sync", font_size=24, color=MEMORY_BLUE),
                Text("스레드 간 참조 공유 가능", font_size=16, color=GRAY),
            ).arrange(DOWN, buff=0.1),
        ).arrange(RIGHT, buff=1.5)
        traits.to_edge(DOWN, buff=0.8)

        self.play(FadeIn(traits, shift=UP))
        self.wait(2)

    def create_check_item(self, problem, solution):
        return VGroup(
            Text("✗", font_size=18, color=ERROR_RED),
            Text(problem, font_size=16, color=WHITE),
            Text("→", font_size=14, color=GRAY),
            Text(solution, font_size=14, color=SUCCESS_GREEN),
        ).arrange(RIGHT, buff=0.2)

    def section_outro(self):
        title = Text("EP 5-01 정리", font_size=44, color=RUST_ORANGE)

        summary = VGroup(
            Text("thread::spawn = 스레드 생성", font_size=24, color=THREAD_TEAL),
            Text("Arc<T> = 스레드 안전 공유", font_size=24, color=MEMORY_BLUE),
            Text("Mutex<T> = 상호 배제 락", font_size=24, color=WARNING_YELLOW),
            Text("mpsc = 메시지 전달 채널", font_size=24, color=HEAP_PURPLE),
        ).arrange(DOWN, buff=0.4)

        VGroup(title, summary).arrange(DOWN, buff=0.8)

        self.play(Write(title))
        for line in summary:
            self.play(FadeIn(line, shift=UP), run_time=0.4)

        self.wait(1)

        next_ep = Text("다음: EP 5-02 Async/Await (마지막!)", font_size=28, color=GRAY)
        next_ep.to_edge(DOWN, buff=1)
        self.play(FadeIn(next_ep))
        self.wait(2)


class EP_5_02_AsyncAwait(Scene):
    """
    EP 5-02: Async/Await - 효율적인 비동기 프로그래밍

    학습 목표:
    1. async/await 문법
    2. Future 트레이트
    3. tokio 런타임
    4. 동시 작업 실행

    🎉 시리즈 마지막 에피소드!
    """

    def construct(self):
        self.camera.background_color = DARK_BG

        self.section_intro()
        self.wait(0.5)
        self.clear()

        self.section_sync_vs_async()
        self.wait(0.5)
        self.clear()

        self.section_async_syntax()
        self.wait(0.5)
        self.clear()

        self.section_future()
        self.wait(0.5)
        self.clear()

        self.section_tokio()
        self.wait(0.5)
        self.clear()

        self.section_concurrent_tasks()
        self.wait(0.5)
        self.clear()

        self.section_series_finale()

    def section_intro(self):
        title = Text("EP 5-02", font_size=32, color=GRAY).to_edge(UP)
        main_title = Text("Async/Await", font_size=56, color=RUST_ORANGE)
        subtitle = Text("효율적인 비동기 프로그래밍", font_size=28, color=GRAY)

        main_title.next_to(title, DOWN, buff=0.8)
        subtitle.next_to(main_title, DOWN, buff=0.3)

        self.play(Write(title), Write(main_title))
        self.play(FadeIn(subtitle))
        self.wait(1)

        # 스레드 vs Async
        comparison = VGroup(
            VGroup(
                Text("스레드", font_size=24, color=THREAD_TEAL),
                Text("• OS 스레드 사용", font_size=16, color=GRAY),
                Text("• 메모리 오버헤드", font_size=16, color=GRAY),
                Text("• 병렬 실행", font_size=16, color=GRAY),
            ).arrange(DOWN, buff=0.15, aligned_edge=LEFT),
            VGroup(
                Text("Async", font_size=24, color=ASYNC_PINK),
                Text("• 단일 스레드 가능", font_size=16, color=GRAY),
                Text("• 저비용 태스크", font_size=16, color=GRAY),
                Text("• 동시 실행 (I/O)", font_size=16, color=GRAY),
            ).arrange(DOWN, buff=0.15, aligned_edge=LEFT),
        ).arrange(RIGHT, buff=2)
        comparison.next_to(subtitle, DOWN, buff=0.8)

        self.play(FadeIn(comparison, shift=UP))
        self.wait(2)

    def section_sync_vs_async(self):
        title = Text("동기 vs 비동기", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # 동기 (순차)
        sync_label = Text("동기 (Sync)", font_size=20, color=GRAY)
        sync_bars = VGroup(
            self.create_task_bar("Task 1", 2, STACK_GREEN),
            self.create_task_bar("Wait", 1.5, GRAY, dashed=True),
            self.create_task_bar("Task 2", 1.5, MEMORY_BLUE),
            self.create_task_bar("Wait", 1, GRAY, dashed=True),
            self.create_task_bar("Task 3", 1, HEAP_PURPLE),
        ).arrange(RIGHT, buff=0)

        sync_group = VGroup(sync_label, sync_bars).arrange(DOWN, buff=0.3)

        # 비동기 (겹침)
        async_label = Text("비동기 (Async)", font_size=20, color=ASYNC_PINK)

        # 시각화: 태스크들이 겹쳐서 실행
        async_timeline = Rectangle(width=6, height=1.5, fill_opacity=0)

        task1 = self.create_task_bar("Task 1", 2, STACK_GREEN)
        task2 = self.create_task_bar("Task 2", 1.5, MEMORY_BLUE)
        task3 = self.create_task_bar("Task 3", 1, HEAP_PURPLE)

        task1.move_to(async_timeline).shift(LEFT * 1.5 + UP * 0.4)
        task2.move_to(async_timeline).shift(LEFT * 0.5)
        task3.move_to(async_timeline).shift(RIGHT * 1 + DOWN * 0.4)

        async_tasks = VGroup(task1, task2, task3)
        async_group = VGroup(async_label, async_timeline, async_tasks).arrange(DOWN, buff=0.3)

        # 배치
        sync_group.next_to(title, DOWN, buff=0.5)
        async_group.next_to(sync_group, DOWN, buff=0.5)

        self.play(Create(sync_group))
        self.wait(0.5)

        # 총 시간 표시
        sync_time = Text("총: 7 단위", font_size=16, color=ERROR_RED)
        sync_time.next_to(sync_bars, RIGHT, buff=0.3)

        self.play(Write(sync_time))
        self.wait(0.5)

        self.play(Create(async_group))

        async_time = Text("총: ~3 단위!", font_size=16, color=SUCCESS_GREEN)
        async_time.next_to(async_tasks, RIGHT, buff=0.3)

        self.play(Write(async_time))
        self.wait(2)

    def create_task_bar(self, name, width, color, dashed=False):
        rect = Rectangle(
            width=width, height=0.4,
            fill_color=color, fill_opacity=0.5 if not dashed else 0.2,
            stroke_color=color
        )
        if dashed:
            rect.set_stroke(style="dashed")
        label = Text(name, font_size=10, color=WHITE)
        label.move_to(rect)
        return VGroup(rect, label)

    def section_async_syntax(self):
        title = Text("async/await 문법", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''// async 함수 정의
async fn fetch_data(url: &str) -> Result<String, Error> {
    // 비동기 HTTP 요청
    let response = reqwest::get(url).await?;
    let text = response.text().await?;
    Ok(text)
}

// async 블록
let future = async {
    println!("시작!");
    let data = fetch_data("https://api.example.com").await?;
    println!("완료: {}", data);
    Ok::<_, Error>(())
};

// Future는 실행하기 전까지 아무것도 안함!
// 런타임이 필요함''',
            language="rust",
            font_size=14,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.7)
        code.next_to(title, DOWN, buff=0.3)

        self.play(Create(code))
        self.wait(1)

        # await 설명
        await_note = VGroup(
            Text(".await", font_size=24, color=ASYNC_PINK),
            Text("= Future가 완료될 때까지 대기", font_size=18, color=WHITE),
            Text("(다른 태스크가 실행될 수 있음)", font_size=14, color=GRAY),
        ).arrange(RIGHT, buff=0.3)
        await_note.to_edge(DOWN, buff=0.5)

        self.play(Write(await_note))
        self.wait(2)

    def section_future(self):
        title = Text("Future 트레이트", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        # Future 정의
        code = Code(
            code='''trait Future {
    type Output;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>)
        -> Poll<Self::Output>;
}

enum Poll<T> {
    Ready(T),    // 완료! 결과 반환
    Pending,     // 아직... 나중에 다시
}''',
            language="rust",
            font_size=16,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.75)
        code.next_to(title, DOWN, buff=0.4)

        self.play(Create(code))
        self.wait(1)

        # 상태 머신
        states = VGroup(
            self.create_state("Created", GRAY),
            Arrow(ORIGIN, RIGHT * 1.5, color=WHITE),
            self.create_state("Pending", WARNING_YELLOW),
            Arrow(ORIGIN, RIGHT * 1.5, color=WHITE),
            self.create_state("Ready", SUCCESS_GREEN),
        ).arrange(RIGHT, buff=0.2)
        states.to_edge(DOWN, buff=1)

        loop_arrow = CurvedArrow(
            states[2].get_top() + UP * 0.2,
            states[2].get_top() + UP * 0.2 + LEFT * 0.5,
            color=WARNING_YELLOW, angle=-PI
        )
        loop_label = Text("poll again", font_size=10, color=WARNING_YELLOW)
        loop_label.next_to(loop_arrow, UP, buff=0.1)

        self.play(Create(states))
        self.play(Create(loop_arrow), Write(loop_label))
        self.wait(2)

    def create_state(self, name, color):
        circle = Circle(radius=0.5, fill_color=color, fill_opacity=0.3, stroke_color=color)
        label = Text(name, font_size=14, color=WHITE)
        label.move_to(circle)
        return VGroup(circle, label)

    def section_tokio(self):
        title = Text("tokio: 비동기 런타임", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''// Cargo.toml
[dependencies]
tokio = { version = "1", features = ["full"] }

// main.rs
#[tokio::main]
async fn main() {
    // 이제 async 코드 실행 가능!
    let result = fetch_data("https://api.example.com").await;
    println!("{:?}", result);
}

// 또는 수동으로
fn main() {
    let rt = tokio::runtime::Runtime::new().unwrap();
    rt.block_on(async {
        // 비동기 코드
    });
}''',
            language="rust",
            font_size=14,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.7)
        code.next_to(title, DOWN, buff=0.3)

        self.play(Create(code))
        self.wait(1)

        # tokio 특징
        features = VGroup(
            Text("tokio 특징:", font_size=20, color=ASYNC_PINK),
            VGroup(
                Text("• 멀티 스레드 런타임", font_size=16, color=GRAY),
                Text("• 타이머, I/O, 네트워크", font_size=16, color=GRAY),
                Text("• 동기화 프리미티브", font_size=16, color=GRAY),
            ).arrange(DOWN, aligned_edge=LEFT, buff=0.1),
        ).arrange(DOWN, buff=0.2, aligned_edge=LEFT)
        features.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(features, shift=UP))
        self.wait(2)

    def section_concurrent_tasks(self):
        title = Text("동시 태스크 실행", font_size=40, color=RUST_ORANGE).to_edge(UP)
        self.play(Write(title))

        code = Code(
            code='''use tokio::join;
use tokio::select;

#[tokio::main]
async fn main() {
    // join!: 모두 완료될 때까지 대기
    let (a, b, c) = join!(
        fetch_user(),
        fetch_posts(),
        fetch_comments(),
    );

    // select!: 먼저 완료되는 것 선택
    select! {
        result = fetch_fast() => println!("fast: {:?}", result),
        result = fetch_slow() => println!("slow: {:?}", result),
    }

    // spawn: 백그라운드 태스크
    let handle = tokio::spawn(async {
        expensive_computation().await
    });
    // 다른 작업...
    let result = handle.await?;
}''',
            language="rust",
            font_size=13,
            background="rectangle",
            background_stroke_color=RUST_ORANGE,
        ).scale(0.68)
        code.next_to(title, DOWN, buff=0.3)

        self.play(Create(code))
        self.wait(1)

        # 패턴 비교
        patterns = VGroup(
            self.create_pattern_box("join!", "모두 기다림", SUCCESS_GREEN),
            self.create_pattern_box("select!", "하나만 기다림", WARNING_YELLOW),
            self.create_pattern_box("spawn", "백그라운드 실행", ASYNC_PINK),
        ).arrange(RIGHT, buff=0.5)
        patterns.to_edge(DOWN, buff=0.5)

        self.play(FadeIn(patterns, shift=UP))
        self.wait(2)

    def create_pattern_box(self, name, desc, color):
        box = RoundedRectangle(
            width=2.5, height=1, corner_radius=0.1,
            fill_color=color, fill_opacity=0.2,
            stroke_color=color
        )
        content = VGroup(
            Text(name, font_size=18, color=color),
            Text(desc, font_size=12, color=GRAY),
        ).arrange(DOWN, buff=0.1)
        content.move_to(box)
        return VGroup(box, content)

    def section_series_finale(self):
        # 에피소드 정리
        title = Text("EP 5-02 정리", font_size=44, color=RUST_ORANGE)

        summary = VGroup(
            Text("async fn = 비동기 함수", font_size=24, color=ASYNC_PINK),
            Text(".await = Future 완료 대기", font_size=24, color=WARNING_YELLOW),
            Text("tokio = 비동기 런타임", font_size=24, color=THREAD_TEAL),
            Text("join!/select!/spawn = 동시 실행 패턴", font_size=24, color=MEMORY_BLUE),
        ).arrange(DOWN, buff=0.4)

        VGroup(title, summary).arrange(DOWN, buff=0.8)

        self.play(Write(title))
        for line in summary:
            self.play(FadeIn(line, shift=UP), run_time=0.4)

        self.wait(2)
        self.clear()

        # ===== 시리즈 완료 축하 =====
        congrats = Text("🎉 축하합니다! 🎉", font_size=56, color=RUST_ORANGE)
        self.play(FadeIn(congrats, scale=1.5))
        self.wait(1)
        self.play(congrats.animate.to_edge(UP))

        complete_title = Text("WIA-RUST-LEARN 전체 과정 완료!", font_size=36, color=WHITE)
        complete_title.next_to(congrats, DOWN, buff=0.5)
        self.play(Write(complete_title))
        self.wait(0.5)

        # 전체 여정 요약
        journey = VGroup(
            Text("당신의 여정:", font_size=28, color=GRAY),
            VGroup(
                Text("Level 0: 시작하기 (3편)", font_size=18, color=STACK_GREEN),
                Text("Level 1: 기초 (4편)", font_size=18, color=MEMORY_BLUE),
                Text("Level 2: 소유권 (4편) - NRT!", font_size=18, color=HEAP_PURPLE),
                Text("Level 3: 구조화 (3편)", font_size=18, color=WARNING_YELLOW),
                Text("Level 4: 고급 기능 (3편)", font_size=18, color=RUST_ORANGE),
                Text("Level 5: 동시성 (2편)", font_size=18, color=ASYNC_PINK),
            ).arrange(DOWN, buff=0.15, aligned_edge=LEFT),
        ).arrange(DOWN, buff=0.3)
        journey.next_to(complete_title, DOWN, buff=0.5)

        self.play(FadeIn(journey, shift=UP))
        self.wait(1)

        # 핵심 메시지
        message = VGroup(
            Text("\"Rust는 어렵지 않다.\"", font_size=32, color=RUST_ORANGE),
            Text("\"다를 뿐이다.\"", font_size=32, color=RUST_ORANGE),
        ).arrange(DOWN, buff=0.2)
        message.to_edge(DOWN, buff=1)

        self.play(Write(message))
        self.wait(1)

        # NRT 언급
        nrt_badge = VGroup(
            Text("🔗 NRT: Non-Repeatable Transfer", font_size=20, color=WARNING_YELLOW),
            Text("소유권은 NFT처럼 유일하고 이전 가능!", font_size=16, color=GRAY),
        ).arrange(DOWN, buff=0.1)
        nrt_badge.next_to(journey, RIGHT, buff=0.5)

        self.play(FadeIn(nrt_badge, shift=LEFT))
        self.wait(2)

        # 페이드 아웃
        self.play(*[FadeOut(mob) for mob in self.mobjects])

        # 최종 메시지
        final = VGroup(
            Text("WIA-RUST-LEARN", font_size=48, color=RUST_ORANGE),
            Text("19편 시리즈 완료", font_size=28, color=WHITE),
            Text("", font_size=20, color=GRAY),
            Text("弘益人間 (홍익인간)", font_size=24, color=SUCCESS_GREEN),
            Text("널리 인간을 이롭게 하라", font_size=18, color=GRAY),
        ).arrange(DOWN, buff=0.4)

        self.play(FadeIn(final, scale=0.8))
        self.wait(3)


# 렌더링 헬퍼
if __name__ == "__main__":
    print("""
╔═══════════════════════════════════════════════════════════════╗
║          WIA-RUST-LEARN Level 5: 동시성                       ║
╠═══════════════════════════════════════════════════════════════╣
║                                                               ║
║  EP 5-01: Threads & Channels                                  ║
║    manim -pql level_5_concurrency.py EP_5_01_ThreadsChannels  ║
║                                                               ║
║  EP 5-02: Async/Await                                         ║
║    manim -pql level_5_concurrency.py EP_5_02_AsyncAwait       ║
║                                                               ║
║  전체 렌더링:                                                  ║
║    manim -pqh level_5_concurrency.py --all                    ║
║                                                               ║
╠═══════════════════════════════════════════════════════════════╣
║                                                               ║
║  🎉 축하합니다! 전체 19편 시리즈가 완료되었습니다!             ║
║                                                               ║
║  "Rust는 어렵지 않다. 다를 뿐이다."                            ║
║                                                               ║
║  弘益人間 (홍익인간) - 널리 인간을 이롭게 하라                 ║
║                                                               ║
╚═══════════════════════════════════════════════════════════════╝
    """)
