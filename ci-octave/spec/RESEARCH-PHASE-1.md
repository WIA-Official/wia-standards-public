# Phase 1 사전 조사 결과
## CI Signal Processing Research

**Date**: 2025-12-14
**Sources**: PubMed, IEEE, Nature, ASHA

---

## 1. 인공와우 기본 원리

### 전극 배열 (Electrode Array)

Cochlear 22-electrode 중심 주파수 (electrode 22 → 1):
```
250, 375, 500, 625, 750, 875, 1000, 1125, 1250, 1438,
1688, 1938, 2188, 2500, 2875, 3313, 3813, 4375, 5000,
5688, 6500, 7438 Hz
```

- 총 22개 전극
- 주파수 배치: 대수적(logarithmic) 분포
- 저주파(apex) → 고주파(base)

### Tonotopic Mapping

달팽이관은 tonotopic 구조:
- 기저부(base): 고주파 감지
- 첨단부(apex): 저주파 감지

**문제**: CI 기본 주파수 할당표(FAT)는 개인 달팽이관 구조를 무시
- **Frequency-to-Place Mismatch**: 최대 1옥타브 이상 불일치 가능
- 2024 연구: 고강도 자극(>80 dB SPL) 시 최대 158° (약 1옥타브) 기저부 이동

Sources:
- [Electrocochleography-Based Tonotopic Map](https://pmc.ncbi.nlm.nih.gov/articles/PMC11493529/)
- [Intensity-Driven Shifts in Tonotopic Coding](https://www.medrxiv.org/content/10.1101/2025.05.05.25327002v1.full)

---

## 2. 신호 처리 전략

### CIS (Continuous Interleaved Sampling)

- **원리**: 각 채널에서 envelope 추출 → 고정 펄스율로 전달
- **장점**: 어음 명료도 우수
- **단점**: TFS(Temporal Fine Structure) 완전 손실

### ACE (Advanced Combination Encoder)

- **원리**: CIS 기반 + n-of-m 전략 (22채널 중 8-12개만 활성화)
- **Cochlear** 기본 전략
- **특징**: 에너지 가장 큰 채널만 선택

### FSP (Fine Structure Processing)

- **개발**: MED-EL (2008)
- **원리**: 저주파 채널에서 위상 정보 일부 보존
- **효과**:
  - 음악 음질 개선 (특히 베이스)
  - 12-24개월 후 소음 속 어음 인식 향상
- **한계**: 모든 사용자에게 효과적이지 않음

Sources:
- [Comparison of FSP and CIS strategies](https://pubmed.ncbi.nlm.nih.gov/21190508/)
- [Long-term improvement with FSP](https://pubmed.ncbi.nlm.nih.gov/24685836/)

---

## 3. Temporal Fine Structure (TFS) 손실 분석

### TFS란?

복잡한 소리 = Envelope + TFS
- **Envelope (E)**: 느리게 변하는 진폭 변화
- **TFS**: 빠른 위상 변화 (개별 파형 주기)

### CI에서 TFS 손실 원인

```
정상 청력:
├── 유모세포 → 개별 파형에 phase locking
├── TFS 정보 보존
└── 정확한 피치 인식

CI:
├── Envelope만 추출
├── 고정 펄스율로 전달
├── TFS 정보 제거
└── 피치 인식 저하
```

### 연구 결과

1. **피치 인식 저하**
   - 대부분의 성인 CI 사용자에서 저주파 피치 인식 비정상
   - 정상 청력 대비 현저히 낮은 TFS 처리 능력

2. **어음 인식 영향**
   - FM 신호(TFS 대용) 추가 시 **71 percentage points** 향상
   - 배경 소음 속 어음 인식에 TFS 필수

3. **바이모달 효과**
   - 반대측 보청기 사용 시 TFS 활용 가능
   - 현저한 바이모달 효과 확인

Sources:
- [Temporal Fine Structure Processing in CI](https://pubmed.ncbi.nlm.nih.gov/29194080/)
- [Role of TFS in Pitch and Speech](https://pmc.ncbi.nlm.nih.gov/articles/PMC2580810/)

---

## 4. 옥타브/피치 손실 정량화

### 피치 인식 오류

| 테스트 | 정상 청력 | CI 사용자 | 차이 |
|-------|----------|----------|------|
| 순음 피치 차별 | ~1% | ~10-20% | 10-20x |
| 옥타브 인식 | >99% | ~50-70% | 30-50% 오류 |
| 멜로디 인식 | >95% | ~40-60% | 35-55% 손실 |

### 음악 청취 문제

1. **옥타브 혼동**: 440Hz vs 880Hz 구분 어려움
2. **화음 인식 불가**: 하모닉 관계 파악 어려움
3. **멜로디 추적 실패**: 음정 변화 감지 어려움

### 성조 언어 문제

- 중국어 성조 인식률: 50-70% (정상 청력 95%+)
- 태국어, 베트남어 등 동일 문제

---

## 5. 기존 개선 시도

### Fine Structure Processing (FSP)

- **효과**: 베이스 주파수 지각 개선
- **한계**:
  - 저주파(1-4채널)에만 적용
  - 옥타브 해결에 불충분
  - 개인차 큼

### Current Steering

- **원리**: 인접 전극 동시 자극으로 가상 채널 생성
- **효과**: 22 → 최대 90+ 가상 채널
- **한계**: 피치 인식 개선 미미

### Temporal Limits Encoder (TLE)

- **원리**: 청신경 시간적 피치 한계 내에서 TFS 인코딩
- **상태**: 연구 단계

Sources:
- [Pitch Perception with TLE](https://pubmed.ncbi.nlm.nih.gov/36044501/)
- [Temporal Pitch Perception Multi-Channel](https://link.springer.com/article/10.1007/s10162-025-00983-4)

---

## 6. 결론 및 설계 방향

### 핵심 문제

1. **TFS 손실**: 현재 CI는 envelope만 전달
2. **옥타브 모호성**: 주파수 정보만으로 옥타브 결정 불가
3. **Place-Rate 불일치**: 22채널로 연속 주파수 표현 한계

### 우리의 접근법 (WIA CI)

```
기존 CI:
  Audio → Envelope 추출 → 전극 자극
          (TFS 버림)

WIA CI:
  Audio → Envelope 추출 → 전극 자극
       ↘
        Octave 검출 → 추가 인코딩
        (하모닉 분석)   (Temporal Modulation)
```

### 설계 원칙

1. **어음 명료도 유지**: 기존 envelope 처리 방해 금지
2. **옥타브 정보 추가**: Temporal Modulation으로 인코딩
3. **실시간 처리**: <50ms 지연
4. **호환성**: 기존 CI 하드웨어와 호환

---

## 7. 참고문헌

1. [Electrocochleography-Based Tonotopic Map II](https://pmc.ncbi.nlm.nih.gov/articles/PMC11493529/) - PMC, 2024
2. [Temporal Fine Structure Processing in CI](https://pubmed.ncbi.nlm.nih.gov/29194080/) - PubMed, 2018
3. [Role of TFS in Pitch and Speech](https://pmc.ncbi.nlm.nih.gov/articles/PMC2580810/) - PMC
4. [Comparison of FSP and CIS strategies](https://pubmed.ncbi.nlm.nih.gov/21190508/) - PubMed, 2011
5. [Musical Sound Quality FSP vs HDCIS](https://pubmed.ncbi.nlm.nih.gov/25906173/) - PubMed, 2015
6. [Pitch Perception with TLE](https://pubmed.ncbi.nlm.nih.gov/36044501/) - IEEE, 2022
7. [Anatomy-based electrode spacing](https://www.nature.com/articles/s41598-024-53070-8) - Nature, 2024

---

**Document ID**: WIA-CI-RESEARCH-001
**Version**: 1.0.0
**Last Updated**: 2025-12-14
**Copyright**: © 2025 SmileStory Inc. / WIA - MIT License
