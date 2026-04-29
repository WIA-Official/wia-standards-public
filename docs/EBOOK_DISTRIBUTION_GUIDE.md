# WIA Ebook 분산 작업 가이드

**생성일:** 2025-12-27
**총 작업량:** 346개 표준, 5,749개 파일
**목표:** 각 chapter 15KB 이상의 고품질 교육 콘텐츠

---

## 🎯 품질 기준

### 파일 크기
- **최소:** 15KB (약 15,000 bytes)
- **권장:** 25-50KB per chapter
- **절대 금지:** 빈 파일, 의미 없는 패딩

### 콘텐츠 요구사항

| 요소 | 필수 | 설명 |
|------|------|------|
| 표준 관련 기술 내용 | ✓ | spec/ 파일 참조하여 매칭 |
| 테이블 (2개 이상) | ✓ | 비교, 데이터, 스펙 정리 |
| 코드/다이어그램 | ✓ | 기술 설명 보조 |
| Chapter Summary | ✓ | Key Takeaways 5개 이상 |
| Review Questions | ✓ | 6개 이상 |
| 네비게이션 링크 | ✓ | 이전/다음 chapter |

### CSS 스타일 (다크 테마)
```css
:root {
    --bg: #0f172a;
    --surface: #1e293b;
    --primary: [카테고리별 색상];
    --text: #f1f5f9;
    --text-dim: #94a3b8;
    --border: #334155;
}
body {
    font-family: Georgia, 'Times New Roman', serif;
    line-height: 1.8;
}
```

### 카테고리별 Primary Color
| 카테고리 | 색상 코드 | 테마 |
|----------|-----------|------|
| TIME | #06B6D4 | cyan |
| CONTACT | #8B5CF6 | purple |
| OCEAN | #0EA5E9 | sky blue |
| ART | #F59E0B | amber |
| HERITAGE | #D97706 | orange |
| LANG | #10B981 | emerald |
| MENTAL | #EC4899 | pink |
| CHILD | #F97316 | orange |
| SENIOR | #6366F1 | indigo |
| LEGAL | #64748B | slate |
| IND | #EF4444 | red |
| 기타 | #8B5CF6 | purple |

---

## 📚 완성 샘플 참조

**위치:** `standards/fire-safety-system/ebook/en/chapter-01.html`

### 샘플 구조
```
1. Chapter Title (h1)
2. Introduction paragraph (역사/배경)
3. Evolution section (발전 과정)
   - Table 1: 시대별 발전
4. Components section (구성 요소)
   - 상세 설명 (ul/li)
   - Callout boxes
5. Technical section
   - Table 2: 기술 비교
   - Code blocks
6. Integration section
7. Standards/Regulatory section
   - Table 3: 관련 표준
8. Challenges section
9. Summary box
   - Key Takeaways (ol)
10. Review Questions box
11. Looking Ahead callout
12. Navigation links
```

---

## 🔗 Spec ↔ Ebook 매칭 규칙

각 표준의 `spec/` 폴더를 **반드시** 읽고 ebook 콘텐츠를 생성해야 합니다.

### Chapter 매핑
| Chapter | Spec 파일 | 주제 |
|---------|-----------|------|
| chapter-01 | PHASE-1-DATA-FORMAT.md | 소개, 데이터 형식 |
| chapter-02 | PHASE-1 심화 | 현재 과제, 문제점 |
| chapter-03 | PHASE-2-API-INTERFACE.md | API, 인터페이스 |
| chapter-04 | PHASE-2 심화 | 구현 가이드 |
| chapter-05 | PHASE-3-PROTOCOL.md | 프로토콜, 통신 |
| chapter-06 | PHASE-3 심화 | 보안, 인증 |
| chapter-07 | PHASE-4-INTEGRATION.md | 통합, 배포 |
| chapter-08 | 종합 | 미래 전망, 결론 |

### Spec 읽기 명령
```bash
# 해당 표준의 spec 파일 확인
cat standards/WIA-XXX-YYY/spec/*.md
```

---

## 📋 분산 작업 할당

### Session 1: TIME + CONTACT + OCEAN (55개)
```
WIA-TIME-001 ~ WIA-TIME-035 (35개)
WIA-CONTACT-001 ~ WIA-CONTACT-010 (10개)
WIA-OCEAN-001 ~ WIA-OCEAN-010 (10개)
```
**테마:** 시간, 우주, 해양 탐사

### Session 2: ART + HERITAGE + LANG (32개)
```
WIA-ART-001 ~ WIA-ART-012 (12개)
WIA-HERITAGE-001 ~ WIA-HERITAGE-010 (10개)
WIA-LANG-001 ~ WIA-LANG-010 (10개)
```
**테마:** 예술, 문화유산, 언어

### Session 3: MENTAL + CHILD + SENIOR (37개)
```
WIA-MENTAL-001 ~ WIA-MENTAL-015 (15개)
WIA-CHILD-001 ~ WIA-CHILD-012 (12개)
WIA-SENIOR-001 ~ WIA-SENIOR-010 (10개)
```
**테마:** 정신건강, 아동, 노인

### Session 4: LEGAL + IND + ROB (23개)
```
WIA-LEGAL-001 ~ WIA-LEGAL-010 (10개)
WIA-IND-001 ~ WIA-IND-011 (11개)
WIA-ROB-001 ~ WIA-ROB-002 (2개)
```
**테마:** 법률, 산업, 로봇

### Session 5: Technology Misc A (70개)
```
3d-image-sensor, 3d-printing-construction, 5g-6g-spectrum, 6g-communication
adas, additive-manufacturing, anti-gravity, api-gateway
artificial-organ, augmentation-ethics, augmentation-safety, autonomous-ship
autonomous-weapon-ethics, battery-management-system, beauty-tech, bio-banking
bio-ethics, bio-integration, bio-manufacturing, bio-safety, biodiversity-index
bioinformatics, biomarker-data, bionic-ear, bionic-limb, biopharma, biosensor
building-energy-management, cdn, cellular-therapy, ci-cd, circular-economy
clinical-trial-data, cloud-computing, code-quality, cognitive-enhancement
connected-car, container-technology, cosmetics-data, crispr-protocol
cyber-defense, cyber-weapon, cybernetic-implant, dark-energy-research
dark-matter-detection, data-center, deep-sea-exploration, delivery-drone
devops, digital-factory, dimension-portal, distributed-computing
drought-monitoring, drug-discovery, e-waste-management, ecosystem-monitoring
edge-computing, electric-vehicle, electronic-warfare, elevator-system
embedded-software, environmental-sensor, ev-charging, event-management
fashion-tech, fire-safety-system, fitness-tracking, fleet-management
food-delivery, food-tech, ftl-communication, gene-therapy
```

### Session 6: Technology Misc B (59개)
```
genome-sequencing, holography, hongik-impact-metric, hotel-tech
human-augmentation, human-machine-interface, hydrogen-vehicle, hyperloop
hypersonic-weapon, industrial-iot, intelligent-transportation
inventory-management, iot-m2m, laser-weapon, lidar-sensor
longevity-gene, low-code-platform, low-power-network, maas, maglev-train
manufacturing-automation, memory-enhancement, memory-semiconductor
mesh-network, microbiome, microservices, military-ai, military-communication
military-drone, military-encryption, military-robot, military-satellite
missile-defense, multiverse-interface, nbc-defense, network-protocol
network-security, network-slicing, neural-enhancement, next-gen-data-storage
nuclear-defense, open-source, operating-system, optical-communication
parallel-processing, personalized-cosmetics, photonic-chip, photonics
physical-enhancement, plasma-technology, power-semiconductor
predictive-maintenance, protein-structure, quality-control
```

### Session 7: Remaining (70개)
```
quantum-algorithm, quantum-communication, quantum-machine-learning
quantum-network, quantum-sensor, quantum-simulation, railway-system
real-time-communication, real-time-os, reconnaissance-satellite
regenerative-medicine, restaurant-tech, retail-tech, ride-sharing
room-temp-superconductor, satellite-internet, scientific-instrument
security-robot, semiconductor-equipment, sensory-enhancement
serverless-architecture, service-robot, smart-gym, smart-kitchen
smart-logistics, smart-parking, smart-store, smart-textile
software-documentation, software-license, software-testing
solid-state-battery, space-surveillance, sports-analytics, sports-tech
stealth-technology, supercomputing, superconducting, supply-chain
synthetic-bio-registry, synthetic-biology, system-semiconductor
teleportation-protocol, ticketing-system, tissue-engineering
tourism-data, traffic-management, transhumanism-protocol, travel-tech
underwater-weapon, universal-consent, universal-data-exchange
universal-error-handling, universal-identity, universal-metadata
universal-protocol, universal-timestamp, unmanned-weapon
v2x, v2x-communication, vehicle-cybersecurity, vehicle-infotainment
vehicle-lightweight-material, vehicle-safety, vehicle-semiconductor
vehicle-to-grid, virtualization, vpn-protocol, wearable-fashion
wireless-charging, wireless-power-transfer, wormhole-navigation
```

---

## 💻 작업 명령어

### 1. 할당된 표준 목록 확인
```bash
# Session 1 예시
ls -d standards/WIA-TIME-* standards/WIA-CONTACT-* standards/WIA-OCEAN-*
```

### 2. Spec 파일 읽기
```bash
cat standards/WIA-TIME-001/spec/*.md
```

### 3. Ebook 파일 생성
각 chapter HTML 파일을 생성할 때:
1. 해당 표준의 spec 파일 내용 확인
2. 카테고리 색상 적용
3. 관련 기술 콘텐츠 작성
4. 15KB 이상 확인

### 4. 품질 검증
```bash
# 파일 크기 확인
find standards/WIA-TIME-*/ebook -name "*.html" -exec wc -c {} \; | awk '{if($1 < 15000) print "FAIL:", $2}'

# 인코딩 확인
file standards/WIA-TIME-001/ebook/en/chapter-01.html
# 예상 결과: HTML document, Unicode text, UTF-8 text
```

---

## ⚠️ 주의사항

1. **인코딩:** 반드시 UTF-8 사용 (弘益人間, 한국어 등 정상 표시)
2. **콘텐츠:** 해당 표준과 관련된 실제 기술 내용 (소설 금지)
3. **크기:** 패딩/반복 금지, 실제 콘텐츠로 15KB 달성
4. **CSS:** 다크 테마 + 카테고리 색상
5. **커밋:** 각 session별 별도 브랜치에서 작업 후 main 머지

---

## 📁 Ebook 폴더 구조

```
standards/WIA-XXX-YYY/
└── ebook/
    ├── en/
    │   ├── index.html      (목차)
    │   ├── chapter-01.html
    │   ├── chapter-02.html
    │   ├── ...
    │   └── chapter-08.html
    └── ko/
        ├── index.html      (목차)
        ├── chapter-01.html
        ├── chapter-02.html
        ├── ...
        └── chapter-08.html
```

---

## ✅ 완료 체크리스트

각 Session 완료 후:
- [ ] 모든 chapter 파일 15KB 이상
- [ ] UTF-8 인코딩 확인
- [ ] Spec 내용과 매칭 확인
- [ ] 테이블, Summary, Questions 포함
- [ ] 네비게이션 링크 동작
- [ ] git commit & push
- [ ] PR 생성 & main 머지

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*© 2025 WIA - World Certification Industry Association*
