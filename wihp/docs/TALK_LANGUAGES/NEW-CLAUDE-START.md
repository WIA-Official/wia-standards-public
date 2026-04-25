# 🚀 WIA Talk 전체 작성 프롬프트 (새 클로드용)

## 📌 당신의 역할

WIA Talk 211개 언어 교재를 작성합니다.
**WIA Braille 마스터 목록 기준으로 정확히 싱크를 맞춰야 합니다.**

---

## 📂 먼저 읽어야 할 파일

```bash
# 1. 기본 교재 (정음) - WIA Talk 시스템 이해
cat /var/www/wia.live/docs/WIA-Talk-Jeongeum.md

# 2. 마스터 언어 목록 - 180개 언어 전체 리스트
cat /var/www/wia.live/docs/MASTER-LANGUAGE-LIST.md

# 3. Braille 배치 참조 - 각 언어별 IPA/음소 정보
ls /var/www/wia.live/docs/BRAILLE_LANGUAGES/
```

---

## 🎯 핵심 원칙

**ALL FIVE ARE EQUAL. NO DEFAULT EXISTS.**

```
한글 ↔ IPA ↔ 점자 ↔ 제스처 ↔ 음성
         ↑
      중심축(허브)
```

---

## 📋 작성할 배치 (총 14개)

| 배치 | 언어 번호 | 언어 수 | 파일명 |
|------|----------|---------|--------|
| 01 | 1-10 | 10 | batch-01-talk-detailed.md |
| 02 | 11-20 | 10 | batch-02-talk-detailed.md |
| 03 | 21-30 | 10 | batch-03-talk-detailed.md |
| 04 | 31-40 | 10 | batch-04-talk-detailed.md |
| 05 | 41-49 | 9 | batch-05-talk-detailed.md |
| 06 | 50-59 | 10 | batch-06-talk-detailed.md |
| 07 | 60-80 | 21 | batch-07-talk-detailed.md |
| 08 | 81-98 | 18 | batch-08-talk-detailed.md |
| 09 | 99-114 | 16 | batch-09-talk-detailed.md |
| 10 | 115-130 | 16 | batch-10-talk-detailed.md |
| 11 | 131-143 | 13 | batch-11-talk-detailed.md |
| 12 | 144-152 | 9 | batch-12-talk-detailed.md |
| 13 | 153-162 | 10 | batch-13-talk-detailed.md |
| 14 | 163-180 | 18 | batch-14-talk-detailed.md |

---

## 📝 각 언어 작성 형식

```markdown
# [전체번호]. [언어명] ([원어명]) [ISO코드]

## 언어 개요
- 사용자 수: 
- 사용 지역: 
- 어족: 
- 문자 체계:

## 음소 체계

### 자음 (##개) → WIA Talk 매핑

| 자음 | IPA | 손모양 | 위치 | 움직임 | 설명 |
|-----|-----|--------|-----|--------|------|
| ㄱ | [k/g] | HS03 | LC05 | MV01 | 주먹-목-정지 |
| ... | ... | ... | ... | ... | ... |

### 모음 (##개) → WIA Talk 매핑

| 모음 | IPA | 손모양 | 위치 | 움직임 | 설명 |
|-----|-----|--------|-----|--------|------|
| ㅏ | [a] | HS01 | LC06 | MV02 | 펼침-가슴-아래 |
| ... | ... | ... | ... | ... | ... |

## 성조/강세 (해당시)

| 성조 | 표기 | 움직임 | 비수지 신호 |
|-----|-----|--------|------------|

## 예시 단어

| 단어 | 의미 | IPA | 제스처 시퀀스 |
|-----|------|-----|--------------|
| 안녕 | hello | [annjʌŋ] | HS05-LC03-MV01 → HS02-LC06-MV02 → ... |

## WIA Talk 특이사항
- 해당 언어의 고유한 발음/제스처 매핑 참고사항
```

---

## 🔧 WIA Talk 제스처 코드 참조

### 손모양 (HS: Hand Shape) - 18종
| 코드 | 이름 | 설명 |
|-----|------|------|
| HS01 | 펼친손 | 손가락 5개 펼침 |
| HS02 | 주먹 | 손가락 모두 접음 |
| HS03 | 검지 | 검지만 펼침 |
| HS04 | V자 | 검지+중지 펼침 |
| HS05 | 엄지척 | 엄지만 펼침 |
| ... | ... | ... |

### 위치 (LC: Location) - 12영역
| 코드 | 이름 | 설명 |
|-----|------|------|
| LC01 | 이마 | 머리 위쪽 |
| LC02 | 눈 | 눈 높이 |
| LC03 | 코 | 코 높이 |
| LC04 | 입 | 입 높이 |
| LC05 | 목 | 목 부위 |
| LC06 | 가슴 | 가슴 중앙 |
| ... | ... | ... |

### 움직임 (MV: Movement) - 15종
| 코드 | 이름 | 설명 |
|-----|------|------|
| MV01 | 정지 | 움직임 없음 |
| MV02 | 아래로 | 수직 하강 |
| MV03 | 위로 | 수직 상승 |
| MV04 | 좌로 | 수평 왼쪽 |
| MV05 | 우로 | 수평 오른쪽 |
| MV06 | 원형 | 원 그리기 |
| ... | ... | ... |

### 방향 (OR: Orientation) - 6방향
| 코드 | 이름 |
|-----|------|
| OR01 | 손바닥 위 |
| OR02 | 손바닥 아래 |
| OR03 | 손바닥 앞 |
| OR04 | 손바닥 뒤 |
| OR05 | 손바닥 좌 |
| OR06 | 손바닥 우 |

### 비수지 신호 (표정/입모양)
- FE01-FE15: 표정 (15종)
- MM01-MM08: 입모양 (8종)

---

## 📂 저장 경로

```
/var/www/wia.live/docs/TALK_LANGUAGES/batch-##-talk-detailed.md
```

---

## ⚠️ 중요 사항

1. **Braille과 정확히 동일한 언어, 동일한 순서**
2. **번호는 전체 연속 (1-180)**
3. **각 언어의 IPA 정보는 Braille 파일 참조**
4. **기존 파일 덮어쓰기 OK** (잘못된 내용임)

---

## 🚀 시작하기

배치 01부터 시작:

```bash
# Braille 배치 01 참조
cat /var/www/wia.live/docs/BRAILLE_LANGUAGES/batch-01-braille-detailed.md
```

그리고 Talk 배치 01 작성:

```bash
vi /var/www/wia.live/docs/TALK_LANGUAGES/batch-01-talk-detailed.md
```

---

*세계인증산업협회(WIA) 연삼흠 박사*
*2025-12-12*
