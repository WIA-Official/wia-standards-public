# WIA Quantum - Phase 1 Research Report

**Quantum Computing Standards 기술 조사 보고서**

**Version**: 1.0.0
**Date**: 2025-01
**Author**: WIA Quantum Working Group

---

## 1. Executive Summary

양자 컴퓨팅은 2025년 현재 상용화 전환기에 진입했습니다. IBM, Google, Microsoft 등 주요 기업들이 오류 정정 양자 컴퓨터(Fault-Tolerant Quantum Computer) 개발에 박차를 가하고 있으며, NIST의 포스트-양자 암호화 표준이 확정되어 산업 전반의 마이그레이션이 시작되었습니다.

WIA Quantum 표준은 다음 영역을 통합합니다:
- **양자 회로 (Quantum Circuits)**: 게이트 기반 양자 연산
- **양자 암호화 (Quantum Cryptography)**: PQC 및 QKD
- **양자 센서 (Quantum Sensors)**: 원자시계, 자력계
- **양자 네트워크 (Quantum Networks)**: 얽힘 분배

---

## 2. Quantum Computing Hardware

### 2.1 주요 기업 및 로드맵

| 기업 | 기술 | 2025 현황 | 목표 |
|------|------|----------|------|
| **IBM** | 초전도 큐비트 | Kookaburra 1,386 큐비트 | 2029 Starling (200 논리큐비트) |
| **Google** | 초전도 큐비트 | Willow 105 큐비트 | 2029 오류정정 컴퓨터 |
| **PsiQuantum** | 광자 | 실리콘 포토닉스 | 2027 백만 큐비트 |
| **Xanadu** | 광자 (CV) | Borealis 216 큐비트 | 2029 양자 데이터센터 |
| **IonQ** | 이온 트랩 | 상용화 | 네트워크형 양자 컴퓨터 |

### 2.2 하드웨어 유형

```
1. 초전도 큐비트 (Superconducting)
   - IBM, Google, Rigetti
   - 극저온 (15mK) 필요
   - 높은 게이트 속도

2. 광자 큐비트 (Photonic)
   - PsiQuantum, Xanadu, ORCA
   - 실온 동작 가능
   - 네트워킹에 유리

3. 이온 트랩 (Trapped Ion)
   - IonQ, Quantinuum
   - 높은 큐비트 연결성
   - 긴 결어긋남 시간

4. 중성 원자 (Neutral Atom)
   - QuEra, Pasqal
   - 대규모 확장성
   - 재배열 가능

5. 위상 큐비트 (Topological)
   - Microsoft
   - 내재적 오류 내성
   - 연구 단계
```

### 2.3 IBM 2025 주요 발표

- **Nighthawk 프로세서**: 120 큐비트, 218개 튜너블 커플러
- **Kookaburra**: 1,386 큐비트 멀티칩, 3개 연결 시 4,158 큐비트
- **Loon 프로세서**: 오류 정정 핵심 컴포넌트 실험용
- **Starling (2029)**: 200 논리 큐비트, 1억 게이트 회로

### 2.4 Google Willow 칩

- 105개 초전도 큐비트
- **임계치 이하 달성**: 큐비트 수 증가 시 오류율 지수적 감소
- 벤치마크: 고전 슈퍼컴퓨터 10^25년 → 5분

---

## 3. Quantum Error Correction (QEC)

### 3.1 표면 코드 (Surface Code)

```
Surface Code 메모리 성과 (2025):
- 거리-7 코드: 101 큐비트
- 논리 오류율: 0.143% ± 0.003% per cycle
- 최고 물리 큐비트 대비 수명 2.4배 향상
- 실시간 디코딩: 63μs 지연
```

### 3.2 QEC 접근 방식

| 코드 유형 | 특징 | 용도 |
|----------|------|------|
| **Surface Code** | 국소 오류 내성 | 연산 |
| **qLDPC Code** | 높은 부호율 | 메모리 |
| **Color Code** | Magic State 효율적 | T-Gate |
| **HGP Code** | 계층적 구조 | 대규모 시스템 |

### 3.3 QEC25 컨퍼런스

2025년 Yale Quantum Institute 주최, 7번째 국제 양자 오류 정정 컨퍼런스.
주요 주제: 양자 제어, 오류 정정, 내결함성.

---

## 4. Quantum Circuit Standards

### 4.1 OpenQASM 3

**Open Quantum Assembly Language**는 양자 회로를 기술하는 업계 표준 중간 표현(IR)입니다.

```qasm
// OpenQASM 3.0 예제
OPENQASM 3.0;
include "stdgates.inc";

qubit[2] q;
bit[2] c;

h q[0];
cx q[0], q[1];
c = measure q;
```

### 4.2 OpenQASM 3 특징

| 기능 | 설명 |
|------|------|
| **동적 회로** | 중간 측정, 조건부 연산 |
| **타이밍 제어** | delay, 물리 큐비트 지정 |
| **펄스 제어** | 캘리브레이션, 특성화 |
| **게이트 수정자** | 제어, 역연산, 거듭제곱 |
| **서브루틴** | 재사용 가능 회로 블록 |

### 4.3 OpenQASM TSC (기술 운영 위원회)

- **참여사**: IBM, Microsoft, AWS 등
- **저장소**: github.com/openqasm/openqasm
- **현재 버전**: 3.0

---

## 5. Post-Quantum Cryptography (PQC)

### 5.1 NIST PQC 표준 (2024-2025)

| 알고리즘 | 유형 | 용도 | 상태 |
|----------|------|------|------|
| **ML-KEM** | 격자 기반 | 키 캡슐화 | FIPS 표준 |
| **ML-DSA** | 격자 기반 | 디지털 서명 | FIPS 표준 |
| **SLH-DSA** | 해시 기반 | 디지털 서명 | FIPS 표준 |
| **HQC** | 코드 기반 | 키 캡슐화 (백업) | 2027 예정 |

### 5.2 마이그레이션 일정

- **NIST IR 8547**: 2035년까지 PQC 전환 로드맵
- **Harvest Now, Decrypt Later**: 현재 시급한 위협
- **권고**: 즉시 마이그레이션 시작

### 5.3 HQC 선정 (2025.03)

- ML-KEM의 백업 알고리즘으로 선정
- 격자 기반과 다른 수학적 기반 (코드 기반)
- 최종 표준: 2027년 예정

---

## 6. Quantum Key Distribution (QKD)

### 6.1 배포 현황

| 네트워크 | 위치 | 기술 |
|----------|------|------|
| **GothamQ** | 뉴욕 | Qunnect 얽힘 기반 |
| **Bearlinq** | 베를린 | Deutsche Telekom + Qunnect |
| **CERN** | 스위스 | 연구용 양자 네트워크 |

### 6.2 기술 성과

```
- 99% 이상 피델리티
- 24/7 운영
- 기존 광섬유 활용
- 극저온 불필요 (Qunnect)
- 325시간 연속 QKD 운영 (50km)
- 100km 확장 링크 시연
```

### 6.3 상하이 자오퉁 대학 성과

- 18명 사용자 다자간 양자 네트워크
- 2개 독립 네트워크 융합
- 가장 복잡한 다자간 양자 네트워크

---

## 7. Quantum Sensors

### 7.1 센서 유형

| 센서 | 기술 | 응용 |
|------|------|------|
| **원자시계** | 마이크로파/광학 | GPS, 전력망 동기화 |
| **원자 자력계** | 알칼리 증기 | 생체자기, 지자기 |
| **NV 다이아몬드** | 결함 기반 | 나노스케일 자기장 |
| **원자 간섭계** | 중력계 | 탐사, 항법 |

### 7.2 2025 주요 배치

- **Royal Navy**: Infleqtion Tiqker 양자 광학 원자시계 시험 (XV Excalibur)
- **MAGNAV**: 자기 이상 기반 항법
- **GRAVNAV**: 중력 이상 기반 항법

### 7.3 시장 전망

```
- 원자시계: 2025년 양자센서 시장의 33.2%
- CAGR: 15.0% (2025-2035)
- 응용: 에너지, 국방, 의료
```

---

## 8. Quantum Networking

### 8.1 Cisco 양자 네트워크 칩

- **Quantum Network Entanglement Chip**
- 초당 2억 얽힘 쌍 생성
- 채널당 100만 고충실도 얽힘 쌍
- Cisco Quantum Labs (Santa Monica) 설립

### 8.2 기술 이정표

| 성과 | 내용 |
|------|------|
| **12시간 안정 얽힘** | 캠퍼스 규모 자동 분배 |
| **네트워크 융합** | 18 사용자 독립 네트워크 통합 |
| **다중화** | 복수 경로 동시 활용 |
| **브리징** | 중복성으로 안정성 확보 |

### 8.3 양자 인터넷 로드맵

```
Level 0: 신뢰 노드 네트워크 (현재)
Level 1: QKD 네트워크
Level 2: 얽힘 분배 네트워크 ← 2025 진입
Level 3: 양자 메모리 네트워크
Level 4: 완전한 양자 인터넷
```

---

## 9. WIA Quantum 표준 범위

### 9.1 데이터 형식 표준화 대상

```
1. 양자 회로 (Quantum Circuit)
   - 게이트, 측정, 조건부 연산
   - OpenQASM 3 호환
   - 시각화용 메타데이터

2. 양자 작업 (Quantum Job)
   - 실행 요청, 결과
   - 백엔드 정보
   - 샷 수, 에러 완화

3. 양자 상태 (Quantum State)
   - 상태 벡터, 밀도 행렬
   - 토모그래피 데이터

4. 양자 암호화 (Quantum Crypto)
   - PQC 키, 서명
   - QKD 세션, 키 자료

5. 양자 센서 (Quantum Sensor)
   - 시계, 자력계, 중력계 데이터
   - 캘리브레이션 정보

6. 양자 네트워크 (Quantum Network)
   - 노드, 링크, 얽힘 자원
   - 라우팅 정보
```

### 9.2 상호운용성 목표

| 호환 대상 | 방식 |
|----------|------|
| **OpenQASM 3** | 회로 표현 |
| **Qiskit** | Python SDK |
| **Cirq** | Google SDK |
| **PennyLane** | 하이브리드 ML |
| **NIST PQC** | 암호화 알고리즘 |

---

## 10. References

### Quantum Computing Hardware
- [IBM Quantum Roadmap 2025](https://www.ibm.com/quantum/blog/ibm-quantum-roadmap-2025)
- [IBM Quantum Blog - Fault Tolerant Path](https://www.ibm.com/quantum/blog/large-scale-ftqc)
- [Quantum Computing Industry Trends 2025](https://www.spinquanta.com/news-detail/quantum-computing-industry-trends-2025-breakthrough-milestones-commercial-transition)

### Quantum Error Correction
- [Quantum Error Correction Below Threshold - Nature](https://www.nature.com/articles/s41586-024-08449-y)
- [QEC25 Conference - Yale](https://qec25.yalepages.org/)

### OpenQASM
- [OpenQASM GitHub](https://github.com/openqasm/openqasm)
- [OpenQASM 3 - ACM](https://dl.acm.org/doi/10.1145/3505636)
- [IBM Quantum Documentation - OpenQASM](https://quantum.cloud.ibm.com/docs/en/guides/introduction-to-qasm)

### Post-Quantum Cryptography
- [NIST PQC Standards](https://www.nist.gov/news-events/news/2024/08/nist-releases-first-3-finalized-post-quantum-encryption-standards)
- [NIST IR 8547 - PQC Transition](https://csrc.nist.gov/pubs/ir/8547/ipd)
- [NIST Selects HQC](https://www.nist.gov/news-events/news/2025/03/nist-selects-hqc-fifth-algorithm-post-quantum-encryption)

### Quantum Networking
- [Qunnect - Entanglement Networks](https://www.qunnect.inc/posts/2025-11-25)
- [Cisco Quantum Labs](https://blogs.cisco.com/news/quantum-networking-how-cisco-is-accelerating-practical-quantum-computing)
- [Shanghai Network Fusion - Phys.org](https://phys.org/news/2025-11-independent-quantum-networks-successfully-fused.html)

### Photonic Quantum Computing
- [PsiQuantum Overview](https://eureka.patsnap.com/blog/psiquantum-quantum-computing/)
- [Xanadu SPAC Merger](https://www.webpronews.com/xanadus-3-6-billion-quantum-leap-photonic-pioneer-bets-big-on-spac-path-to-nasdaq/)

### Quantum Sensors
- [IDTechEx - Quantum Sensors Market](https://www.idtechex.com/en/research-report/quantum-sensors-market-2025/1063)

---

**Document Version**: 1.0.0
**Last Updated**: 2025-01

---

弘益人間 - *Benefit All Humanity*
