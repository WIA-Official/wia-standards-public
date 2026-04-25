# 🚨 WIA Talk 전체 재설계 프롬프트

## ⚠️ 심각한 문제 발견

현재 WIA Talk 배치에 다음 문제가 있습니다:

1. **번호 불연속**: 배치 01-06은 1-10, 배치 07부터 61 시작
2. **언어 중복**: 여러 언어가 다른 배치에 중복 존재
3. **Braille과 싱크 불일치**: 배치 07-08이 완전히 다른 언어

---

## 🎯 해결 방법: Braille 마스터 기준 전체 재작성

**마스터 목록 파일**: `/var/www/wia.live/docs/MASTER-LANGUAGE-LIST.md`

이 파일을 **반드시 먼저 읽고** 작업하세요!

---

## 📋 작업 순서

### 1단계: 기존 파일 백업
```bash
cd /var/www/wia.live/docs/TALK_LANGUAGES/
mkdir -p backup-old
mv batch-07-talk-detailed.md backup-old/
mv batch-08-talk-detailed.md backup-old/
# 09-10도 있으면 백업
```

### 2단계: 마스터 목록 확인
```bash
cat /var/www/wia.live/docs/MASTER-LANGUAGE-LIST.md
```

### 3단계: 배치 07-14 새로 작성

**Braille 마스터 기준으로 정확히 동일한 언어, 동일한 순서로!**

| 배치 | 언어 번호 | 저장 파일 |
|------|----------|----------|
| 07 | 60-80 | batch-07-talk-detailed.md |
| 08 | 81-98 | batch-08-talk-detailed.md |
| 09 | 99-114 | batch-09-talk-detailed.md |
| 10 | 115-130 | batch-10-talk-detailed.md |
| 11 | 131-143 | batch-11-talk-detailed.md |
| 12 | 144-152 | batch-12-talk-detailed.md |
| 13 | 153-162 | batch-13-talk-detailed.md |
| 14 | 163-180 | batch-14-talk-detailed.md |

---

## 📝 각 언어 작성 형식

```markdown
# [전체번호]. [언어명] ([원어명]) [ISO코드]

## 언어 개요
- 사용자 수: 
- 사용 지역: 
- 어족: 

## 자음 체계 → WIA Talk 매핑

| 자음 | IPA | 손모양 | 위치 | 움직임 |
|-----|-----|--------|-----|--------|
| ... | [.] | HS## | LC## | MV## |

## 모음 체계 → WIA Talk 매핑

| 모음 | IPA | 손모양 | 위치 | 움직임 |
|-----|-----|--------|-----|--------|
| ... | [.] | HS## | LC## | MV## |

## 예시 단어

| 단어 | 의미 | IPA | 제스처 시퀀스 |
|-----|------|-----|--------------|
| 안녕 | hello | [annjʌŋ] | HS05-LC03-MV01 → ... |
```

---

## 🔴 중요: 기존 배치 07-08의 언어 처리

**현재 Talk 배치 07에 있는 언어들 (잘못된 위치):**
- 아이슬란드어, 웨일스어, 바스크어 등 → **배치 09**에 있어야 함

**현재 Talk 배치 08에 있는 언어들 (잘못된 위치):**
- 라틴어, 에스페란토, 클링온어 등 → **배치 15 (인공어)** 별도 생성

### 인공어/고전어 배치 15 생성 (선택)
```markdown
# 배치 15: 인공어 및 고전어

181. Latin (Latina) [la]
182. Esperanto [eo]
183. Interlingua [ia]
184. Ido [io]
185. Lojban [jbo]
186. Toki Pona [tok]
187. Volapük [vo]
188. Klingon [tlh]
189. Sanskrit (संस्कृतम्) [sa]
190. Classical Greek [grc]
```

---

## ✅ 완료 체크리스트

| 배치 | Braille | Talk | 상태 |
|------|---------|------|------|
| 01-06 | ✅ | ✅ | 유지 (싱크 맞음) |
| 07 | ✅ 60-80 | ⏳ | **재작성 필요** |
| 08 | ✅ 81-98 | ⏳ | **재작성 필요** |
| 09 | ✅ 99-114 | ⏳ | 새로 생성 |
| 10 | ✅ 115-130 | ⏳ | 새로 생성 |
| 11 | ✅ 131-143 | ⏳ | 새로 생성 |
| 12 | ✅ 144-152 | ⏳ | 새로 생성 |
| 13 | ✅ 153-162 | ⏳ | 새로 생성 |
| 14 | ✅ 163-180 | ⏳ | 새로 생성 |
| 15 | - | ⏳ | 인공어 (선택) |

---

## 🎯 핵심 원칙

**ALL FIVE ARE EQUAL. NO DEFAULT EXISTS.**

```
한글 ↔ IPA ↔ 점자 ↔ 제스처 ↔ 음성
         ↑
      중심축(허브)
```

**Braille과 Talk은 반드시 동일한 언어 목록!**

---

## 저장 경로

```
/var/www/wia.live/docs/TALK_LANGUAGES/batch-##-talk-detailed.md
```

---

*생성일: 2025-12-12*
*세계인증산업협회(WIA) 연삼흠 박사*
