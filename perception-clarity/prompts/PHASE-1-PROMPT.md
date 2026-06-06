# Phase 1: Data Format Standard
## Claude Code 작업 프롬프트

---

**Standard**: WIA Perception Clarity  
**Phase**: 1 of 4  
**목표**: 피지컬 AI 센서 인식 명료도(PCI·상태·보고) 데이터 형식 표준화

---

## 🎯 목표

피지컬 AI 에이전트(차량·로봇·드론·AMR)의 광학/인식 센서가 "지금 얼마나 잘 보이는가"를 측정·상태화·보고하기 위한 데이터 형식 정의. PCI(0–100)·상태 enum·오염 분류·보고 메시지 스키마를 표준화하며, 세척 하드웨어는 표준화하지 않는다.

---

## 📋 웹서치 키워드

```
sensor contamination, lens soiling autonomous, lidar window cleaning, camera occlusion detection, VDA5050, MTF degradation
```

---

## 🔄 작업 순서

```
1. 웹서치로 기술 조사
   - 현재 산업 현황
   - 기존 표준/프로토콜
   - 주요 기업/제품
   - 기술 트렌드

2. /perception-clarity/spec/RESEARCH-PHASE-1.md 작성

3. 데이터 형식 설계
   - PCI 데이터 모델 (occlusion·distance·MTF 축)
   - 상태 enum / 오염 분류
   - JSON Schema 정의
   - 필수/선택 필드

4. /perception-clarity/spec/PHASE-1-DATA-FORMAT.md 작성

5. /perception-clarity/spec/schemas/*.schema.json 생성
```

---

## 📁 산출물

```
/perception-clarity/spec/RESEARCH-PHASE-1.md
/perception-clarity/spec/PHASE-1-DATA-FORMAT.md
/perception-clarity/spec/schemas/*.schema.json
```

---

## ✅ 완료 체크리스트

```
□ 웹서치로 기술 조사 완료
□ RESEARCH-PHASE-1.md 작성
□ 데이터 형식 설계
□ PHASE-1-DATA-FORMAT.md 작성
□ JSON Schema 생성
□ README 업데이트 (Phase 1 완료)
```

---

弘益人間 🤟
