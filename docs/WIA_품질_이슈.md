# WIA 표준 품질 이슈 추적

> **Last Updated:** 2025-12-26
> **Created by:** Claude PM
> **Purpose:** 서버 배포 전 품질 검사에서 발견된 문제 추적

---

## 품질 기준 (모든 표준 필수)

### 필수 파일 구조
```
{표준명}/
├── index.html              ← 랜딩페이지 (필수)
├── simulator/
│   └── index.html          ← 시뮬레이터 (필수, 99개 언어 지원)
├── ebook/
│   ├── en/
│   │   ├── index.html      ← 목차
│   │   └── chapter-01~08.html  ← 8개 챕터 (각 15KB 이상)
│   └── ko/
│       ├── index.html      ← 목차 (한글)
│       └── chapter-01~08.html  ← 8개 챕터 (각 15KB 이상, 실제 한글!)
└── spec/
    ├── PHASE-1-DATA-FORMAT.md    ← 최소 5KB
    ├── PHASE-2-API.md            ← 최소 5KB
    ├── PHASE-3-PROTOCOL.md       ← 최소 5KB
    └── PHASE-4-INTEGRATION.md    ← 최소 5KB
```

### 품질 체크리스트
- [ ] Landing page (index.html) 존재
- [ ] Simulator 존재 + 99개 언어 드롭다운
- [ ] EN ebook 9개 파일 (각 15KB 이상)
- [ ] KO ebook 9개 파일 (각 15KB 이상, 실제 한글!)
- [ ] Spec 4개 파일 (각 5KB 이상)
- [ ] Landing의 Ebook 링크 = `https://wiabook.com/tag/wia-{표준명}/`

---

## TYPE A: Ebook 완전 누락 (23개)

> **상태:** 미해결
> **작업:** Landing, Simulator, Ebook EN/KO 전체, Spec 4개 전부 생성

| # | 표준명 | 상태 |
|---|--------|------|
| 1 | 3d-printing-construction | ⬜ |
| 2 | access-control-system | ⬜ |
| 3 | biodiversity-index | ⬜ |
| 4 | building-energy-management | ⬜ |
| 5 | deep-sea-exploration | ⬜ |
| 6 | drought-monitoring | ⬜ |
| 7 | e-waste-management | ⬜ |
| 8 | ecosystem-monitoring | ⬜ |
| 9 | elevator-system | ⬜ |
| 10 | environmental-sensor | ⬜ |
| 11 | fire-safety-system | ⬜ |
| 12 | flood-prediction | ⬜ |
| 13 | forest-fire-detection | ⬜ |
| 14 | hvac-system | ⬜ |
| 15 | indoor-air-quality | ⬜ |
| 16 | light-pollution | ⬜ |
| 17 | mineral-mining | ⬜ |
| 18 | noise-pollution | ⬜ |
| 19 | oil-gas-drilling | ⬜ |
| 20 | radioactive-waste | ⬜ |
| 21 | rare-earth-mining | ⬜ |
| 22 | recycling | ⬜ |
| 23 | resource-depletion | ⬜ |
| 24 | seabed-resource | ⬜ |

---

## TYPE B: Ebook 부분완성 (35개)

> **상태:** 미해결
> **작업:** Ebook EN 8개 챕터 추가, KO 8개 챕터 추가 (index만 있는 상태)

| # | 표준명 | 상태 |
|---|--------|------|
| 1 | access-control | ⬜ |
| 2 | bim | ⬜ |
| 3 | biometric-auth | ⬜ |
| 4 | blockchain-security | ⬜ |
| 5 | climate-change-mitigation | ⬜ |
| 6 | construction-robot-city | ⬜ |
| 7 | coral-reef-restoration | ⬜ |
| 8 | cyber-weapon-defense | ⬜ |
| 9 | cybersecurity | ⬜ |
| 10 | data-encryption | ⬜ |
| 11 | ddos-protection | ⬜ |
| 12 | desertification-prevention | ⬜ |
| 13 | direct-air-capture | ⬜ |
| 14 | gdpr-compliance | ⬜ |
| 15 | glacier-preservation | ⬜ |
| 16 | green-infrastructure | ⬜ |
| 17 | greenhouse-gas-monitoring | ⬜ |
| 18 | hardware-security-module | ⬜ |
| 19 | homomorphic-encryption | ⬜ |
| 20 | identity-management | ⬜ |
| 21 | intrusion-detection | ⬜ |
| 22 | mangrove-restoration | ⬜ |
| 23 | multi-factor-auth | ⬜ |
| 24 | ocean-acidification | ⬜ |
| 25 | penetration-testing | ⬜ |
| 26 | permafrost-protection | ⬜ |
| 27 | plastic-alternative | ⬜ |
| 28 | polar-region-protection | ⬜ |
| 29 | privacy-preservation | ⬜ |
| 30 | rainforest-conservation | ⬜ |
| 31 | ransomware-protection | ⬜ |
| 32 | sea-level-rise | ⬜ |
| 33 | seawater-desalination | ⬜ |
| 34 | secure-enclave | ⬜ |
| 35 | zero-trust | ⬜ |

---

## TYPE C: KO 번역 누락 (4개)

> **상태:** 미해결
> **작업:** KO ebook chapter 파일들 한글로 완전 번역

| # | 표준명 | EN | KO | 필요 작업 | 상태 |
|---|--------|----|----|----------|------|
| 1 | nlp-standard | 9개 | 2개 | KO 7개 추가 | ⬜ |
| 2 | pet-nutrition | 9개 | 2개 | KO 7개 추가 | ⬜ |
| 3 | prosthetic-control | 9개 | 1개 | KO 8개 + Spec 4개 | ⬜ |
| 4 | reinforcement-learning | 8개 | 1개 | EN 1개 + KO 8개 | ⬜ |

---

## TYPE D: Simulator 누락 (14개)

> **상태:** 미해결
> **작업:** simulator/index.html 생성 (99개 언어 지원)

| # | 표준명 | 상태 |
|---|--------|------|
| 1 | ai-diagnosis | ⬜ |
| 2 | clinical-decision-support | ⬜ |
| 3 | digital-pathology | ⬜ |
| 4 | elderly-care-device | ⬜ |
| 5 | emergency-medical-data | ⬜ |
| 6 | hospital-info-system | ⬜ |
| 7 | medical-alert-system | ⬜ |
| 8 | medical-data-privacy | ⬜ |
| 9 | medical-imaging | ⬜ |
| 10 | medical-iot | ⬜ |
| 11 | medication-adherence | ⬜ |
| 12 | mental-health-monitoring | ⬜ |
| 13 | ozone-layer-protection | ⬜ |
| 14 | remote-patient-monitoring | ⬜ |

---

## TYPE E: 거의 완성 - 1개 챕터 누락 (7개)

> **상태:** 미해결
> **작업:** EN/KO 각각 chapter-08 추가

| # | 표준명 | 상태 |
|---|--------|------|
| 1 | carbon-capture | ⬜ |
| 2 | distributed-energy | ⬜ |
| 3 | energy-cloud | ⬜ |
| 4 | fusion-energy | ⬜ |
| 5 | generative-ai | ⬜ |
| 6 | nuclear-energy | ⬜ |
| 7 | recommendation-ai | ⬜ |

---

## TYPE F: Spec 부족 (15개)

> **상태:** 미해결
> **작업:** PHASE-1~4.md 파일 4개로 맞추기

| # | 표준명 | 현재 | 필요 | 상태 |
|---|--------|------|------|------|
| 1 | climate-refugee | 0개 | 4개 | ⬜ |
| 2 | prosthetic-control | 0개 | 4개 | ⬜ |
| 3 | ai-diagnosis | 2개 | 2개 추가 | ⬜ |
| 4 | clinical-decision-support | 2개 | 2개 추가 | ⬜ |
| 5 | medical-data-privacy | 2개 | 2개 추가 | ⬜ |
| 6 | medical-imaging | 2개 | 2개 추가 | ⬜ |
| 7 | medical-iot | 2개 | 2개 추가 | ⬜ |
| 8 | mental-health-monitoring | 2개 | 2개 추가 | ⬜ |
| 9 | elderly-care-device | 1개 | 3개 추가 | ⬜ |
| 10 | emergency-medical-data | 1개 | 3개 추가 | ⬜ |
| 11 | hospital-info-system | 1개 | 3개 추가 | ⬜ |
| 12 | medical-alert-system | 1개 | 3개 추가 | ⬜ |
| 13 | medication-adherence | 1개 | 3개 추가 | ⬜ |
| 14 | ozone-layer-protection | 1개 | 3개 추가 | ⬜ |
| 15 | remote-patient-monitoring | 1개 | 3개 추가 | ⬜ |

---

## TYPE G: Landing 누락 (7개)

> **상태:** 미해결
> **작업:** index.html (랜딩페이지) 생성

| # | 표준명 | 상태 |
|---|--------|------|
| 1 | ai-assistant | ⬜ |
| 2 | ai-embodiment-ethics | ⬜ |
| 3 | ai-human-coexistence | ⬜ |
| 4 | ai-motor-control | ⬜ |
| 5 | ai-robot-interface | ⬜ |
| 6 | ai-safety-physical | ⬜ |
| 7 | ai-sensor-fusion | ⬜ |

---

## TYPE H: Spec 누락 (3개)

> **상태:** 미해결
> **작업:** PHASE-1~4.md 4개 생성

| # | 표준명 | 상태 |
|---|--------|------|
| 1 | emotion-ai | ⬜ |
| 2 | llm-interop | ⬜ |
| 3 | refugee-credential | ⬜ |

---

## 제외 항목 (건드리지 말 것!)

- `ai-survival-2026` - 전자책 전용
- `wihp` - 전략 문서

---

## 세션 분리 권장

| 세션 | 작업 | 예상 부하 |
|------|------|----------|
| 1 | TYPE A 전반 (12개) | Heavy |
| 2 | TYPE A 후반 (12개) | Heavy |
| 3 | TYPE B 전반 (17개) | Medium |
| 4 | TYPE B 후반 (18개) | Medium |
| 5 | TYPE C + D (18개) | Light |
| 6 | TYPE E + F + G + H (32개) | Light |

---

## 템플릿 참조

- **Simulator:** `/aac/simulator/index.html`
- **Ebook:** `/emotion-ai/ebook/en/chapter-01.html`
- **Spec:** `/battery-passport/spec/PHASE-1-DATA-FORMAT.md`
- **Landing:** `/emotion-ai/index.html`

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*© 2025 WIA - World Certification Industry Association*
