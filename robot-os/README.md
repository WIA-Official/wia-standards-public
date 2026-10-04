# WIA-ROB-020 — Robot Operating System

> 로봇 운영체제 표준 — 로봇 운영체제의 감독·실행 계약: 배타적 모션 권한·관절 속도 명령·로컬 실행 펜싱·단조 시간·정지 확인·텔레메트리·관리 API·적합성 평가

## Scope

WIA-ROB-020 v1.0 defines an interoperable supervisory and execution contract
for a robot operating system. It covers:

- Runtime and component discovery.
- Configuration of controlled joints, components, and coordinate frames.
- Dependency validation and runtime lifecycle management.
- Exclusive, expiring motion authority.
- A complete joint-velocity command contract.
- Local actuator adapters, feedback acquisition, and execution fencing.
- Monotonic timing, watchdogs, and bounded command validity.
- Joint-state, frame-transform, and component-health telemetry.
- Fault reporting, stop initiation, and stop confirmation.
- Authentication, authorization, auditability, and privacy controls.
- REST management interfaces, bounded event replay, and conformance assessment.

Out of scope for v1.0: kernel implementation and language bindings;
navigation, planning, perception, and inverse kinematics; position-trajectory,
force, impedance, and whole-body control; mechanical design and
application-specific risk assessment; emergency-stop circuit design or
functional-safety certification; guarantees of collision avoidance or
human-safe contact; automatic transfer of active motion control between
redundant runtimes; and a universal actuator fieldbus. Conformance does not
establish compatibility with software named ROS or ROS 2.

## Conformance levels

| Level | Name | Adds |
|---|---|---|
| 1 | Basic | Complete runtime, configuration, local execution, command, safety-boundary, security, and telemetry contracts |
| 2 | Standard | Event replay, integrity-protected audit records, measured timing qualification |
| 3 | Advanced | Overload isolation and crash-recovery qualification |

## Normative references

- RFC 2119, RFC 8174 (requirement keywords)
- RFC 8259 (JSON), RFC 8446 (TLS 1.3), RFC 9110 (HTTP Semantics)
- JSON Schema Draft 2020-12

## Files

- `spec/WIA-ROB-020-v1.0.md` — the full specification (v1.0)
- `ebook/ko/`, `ebook/en/` — the 8-chapter companion guide in Korean and English

## License and publisher

© SmileStory Inc. · WIA Standards
