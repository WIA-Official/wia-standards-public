# 🚨 WIA Talk 재작성 - 엄격한 지시사항

## ⛔ 현재 문제점 (당신이 만든 파일)

| 문제 | 설명 |
|------|------|
| ❌ Braille과 언어 순서 불일치 | Braille 배치 02는 es-AR, fr-CA... 인데 Talk는 it, pt... |
| ❌ 번호 체계 잘못됨 | Braille은 배치별 1-10, Talk은 전체 연속 |
| ❌ 배치 04-14 내용 부실 | 테이블 요약만 있고 상세 매핑 없음 |
| ❌ 총 분량 부족 | Braille 20,849줄 vs Talk 2,405줄 (11%) |

---

## 🎯 반드시 지켜야 할 규칙

### 규칙 1: Braille 파일과 100% 동일한 언어, 동일한 순서

**각 배치 작성 전에 반드시 해당 Braille 파일을 먼저 읽으세요!**

```bash
# 예: 배치 02 작성 전
cat /var/www/wia.live/docs/BRAILLE_LANGUAGES/batch-02-braille-detailed.md
```

Braille 배치 02의 언어:
```
# 1. Español Rioplatense (Argentine Spanish) [es-AR]
# 2. Français Canadien (Canadian French) [fr-CA]
# 3. Italiano (Italian) [it]
# 4. Português Europeu (European Portuguese) [pt]
# 5. Português Brasileiro (Brazilian Portuguese) [pt-BR]
# 6. العربية (Arabic - Modern Standard) [ar]
# 7. हिन्दी (Hindi) [hi]
# 8. বাংলা (Bengali/Bangla) [bn]
# 9. ਪੰਜਾਬੀ (Punjabi) [pa]
# 10. తెలుగు (Telugu) [te]
```

**Talk 배치 02도 정확히 이 순서로!**

---

### 규칙 2: 번호 체계 - 배치별 1부터 시작

```markdown
# Batch 02

# 1. Español Rioplatense [es-AR]  ← 1번
# 2. Français Canadien [fr-CA]     ← 2번
...
# 10. తెలుగు [te]                  ← 10번
```

**❌ 잘못된 예**: `# 11. Italiano` (전체 연속 번호)
**✅ 올바른 예**: `# 1. Español Rioplatense` (배치별 1부터)

---

### 규칙 3: 각 언어별 상세 매핑 필수

**테이블 요약만 하면 안 됩니다!**

각 언어마다 반드시 포함할 내용:

```markdown
# 1. [언어명] ([원어명]) [ISO코드]

## 1.1 언어 개요
- 사용자 수: 약 X명
- 사용 지역: 
- 어족: 
- 문자 체계:

## 1.2 자음 → WIA Talk 매핑 (##개)

| 자음 | IPA | 손모양 | 위치 | 움직임 | 방향 | 설명 |
|-----|-----|--------|-----|--------|-----|------|
| b | [b] | HS02 | LC04 | MV01 | OR03 | 주먹-입-정지-앞 |
| d | [d] | HS03 | LC03 | MV02 | OR03 | 검지-코-아래-앞 |
| ... | ... | ... | ... | ... | ... | ... |

## 1.3 모음 → WIA Talk 매핑 (##개)

| 모음 | IPA | 손모양 | 위치 | 움직임 | 설명 |
|-----|-----|--------|-----|--------|------|
| a | [a] | HS01 | LC06 | MV01 | 펼침-가슴-정지 |
| e | [e] | HS01 | LC05 | MV01 | 펼침-목-정지 |
| ... | ... | ... | ... | ... | ... |

## 1.4 성조/강세 (해당 언어만)

## 1.5 예시 단어

| 단어 | 의미 | IPA | 제스처 시퀀스 |
|-----|------|-----|--------------|
| hola | 안녕 | [ola] | HS01-LC04-MV01 → HS01-LC06-MV01 |

## 1.6 특이사항
- 해당 언어의 고유한 음소/제스처 참고사항
```

---

### 규칙 4: 분량 기준

| 배치 | 최소 줄 수 | 참고 (Braille) |
|------|-----------|----------------|
| 01 | 800줄 이상 | 3,740줄 |
| 02 | 800줄 이상 | 2,115줄 |
| 03 | 800줄 이상 | 1,420줄 |
| 04 | 800줄 이상 | 1,389줄 |
| 05 | 800줄 이상 | 1,249줄 |
| 06 | 800줄 이상 | 1,077줄 |
| 07 | 1,000줄 이상 | 2,024줄 |
| 08 | 800줄 이상 | 597줄 |
| 09-14 | 각 600줄 이상 | 평균 1,000줄 |

**총 목표: 최소 10,000줄 이상**

---

## 📋 배치별 Braille 언어 목록 (정확히 따라하세요)

### 배치 01 (10개) ✅ 이미 완료 - 유지
1. English [en]
2. 한국어 [ko]
3. 日本語 [ja]
4. 普通话 [zh-CN]
5. 國語 [zh-TW]
6. 粵語 [zh-HK]
7. Español [es]
8. Español Mexicano [es-MX]
9. Français [fr]
10. Deutsch [de]

### 배치 02 (10개) ❌ 재작성 필요
1. Español Rioplatense [es-AR]
2. Français Canadien [fr-CA]
3. Italiano [it]
4. Português Europeu [pt]
5. Português Brasileiro [pt-BR]
6. العربية (Arabic) [ar]
7. हिन्दी (Hindi) [hi]
8. বাংলা (Bengali) [bn]
9. ਪੰਜਾਬੀ (Punjabi) [pa]
10. తెలుగు (Telugu) [te]

### 배치 03 (10개) ❌ 재작성 필요
1. தமிழ் (Tamil) [ta]
2. मराठी (Marathi) [mr]
3. ગુજરાતી (Gujarati) [gu]
4. ಕನ್ನಡ (Kannada) [kn]
5. മലയാളം (Malayalam) [ml]
6. اردو (Urdu) [ur]
7. ภาษาไทย (Thai) [th]
8. Tiếng Việt (Vietnamese) [vi]
9. Bahasa Indonesia [id]
10. Bahasa Melayu [ms]

### 배치 04 (10개) ❌ 재작성 필요
1. Tagalog/Filipino [tl]
2. 조선어 (Korean-DPRK) [ko-KP]
3. 日本語 かな専用 [ja-kana]
4. فارسی (Persian) [fa]
5. Türkçe (Turkish) [tr]
6. Polski (Polish) [pl]
7. Українська (Ukrainian) [uk]
8. Русский (Russian) [ru]
9. Čeština (Czech) [cs]
10. Română (Romanian) [ro]

### 배치 05 (9개) ❌ 재작성 필요
1. Nederlands (Dutch) [nl]
2. Svenska (Swedish) [sv]
3. Dansk (Danish) [da]
4. Norsk (Norwegian) [no]
5. Suomi (Finnish) [fi]
6. Ελληνικά (Greek) [el]
7. Magyar (Hungarian) [hu]
8. Български (Bulgarian) [bg]
9. Slovenčina (Slovak) [sk]

### 배치 06 (10개) ❌ 재작성 필요
1. Slovenščina (Slovenian) [sl]
2. Српски (Serbian) [sr]
3. Македонски (Macedonian) [mk]
4. Shqip (Albanian) [sq]
5. Latviešu (Latvian) [lv]
6. Lietuvių (Lithuanian) [lt]
7. Eesti (Estonian) [et]
8. ქართული (Georgian) [ka]
9. Հայdelays (Armenian) [hy]
10. Azərbaycan (Azerbaijani) [az]

### 배치 07-14: Braille 파일 직접 참조
```bash
cat /var/www/wia.live/docs/BRAILLE_LANGUAGES/batch-07-braille-detailed.md
cat /var/www/wia.live/docs/BRAILLE_LANGUAGES/batch-08-braille-detailed.md
# ... 14까지
```

---

## 🔧 WIA Talk 제스처 코드 (암기하세요)

### 손모양 (HS) - 18종
| 코드 | 이름 | IPA 대응 예시 |
|-----|------|--------------|
| HS01 | 펼친손 (5) | 모음, 활음 |
| HS02 | 주먹 (S) | 폐쇄음 [p,t,k,b,d,g] |
| HS03 | 검지 (1) | 치경음 [t,d,n,s,z] |
| HS04 | V자 (V) | 치음 [θ,ð] |
| HS05 | 엄지척 (A) | 후음 [h,ʔ] |
| HS06 | 새끼손가락 (I) | 고모음 [i,u] |
| HS07 | 집게 (C) | 파찰음 [tʃ,dʒ] |
| HS08 | OK (F) | 순치음 [f,v] |
| HS09 | 전화기 (Y) | 비음 [m,n,ŋ] |
| HS10 | 총 (L) | 설측음 [l] |
| HS11 | ILY (ILY) | 이중모음 |
| HS12 | W자 (W) | 원순활음 [w] |
| HS13 | 중지 (M) | 저모음 [a,æ] |
| HS14 | 약지 (R) | 권설음 [ɹ,ɾ] |
| HS15 | 손날 (B) | 양순음 [p,b,m] |
| HS16 | 손등 (K) | 연구개음 [k,g,ŋ] |
| HS17 | 손바닥 (P) | 순음 일반 |
| HS18 | 손가락모음 (O) | 원순모음 [o,u] |

### 위치 (LC) - 12영역
| 코드 | 위치 | 조음점 대응 |
|-----|------|------------|
| LC01 | 이마 | - |
| LC02 | 눈 | - |
| LC03 | 코 | 비강 (비음) |
| LC04 | 입 | 양순, 순치 |
| LC05 | 턱 | 치경, 치음 |
| LC06 | 목/가슴 | 성문, 인두 |
| LC07 | 어깨 | - |
| LC08 | 팔꿈치 | - |
| LC09 | 손목 | - |
| LC10 | 중립공간 | 기본 위치 |
| LC11 | 좌측공간 | - |
| LC12 | 우측공간 | - |

### 움직임 (MV) - 15종
| 코드 | 움직임 | 음성 대응 |
|-----|--------|----------|
| MV01 | 정지 | 단모음, 폐쇄 |
| MV02 | 아래로 | 하강조, 종성 |
| MV03 | 위로 | 상승조, 초성 |
| MV04 | 좌로 | - |
| MV05 | 우로 | - |
| MV06 | 원형 | 전동음 [r] |
| MV07 | 진동 | 전동음 [ʀ] |
| MV08 | 앞으로 | 방출 |
| MV09 | 뒤로 | 흡착 |
| MV10 | 튕김 | 탄설음 [ɾ] |
| MV11 | 물결 | 장모음 |
| MV12 | 지그재그 | 마찰음 연속 |
| MV13 | 꺾임 | 이중모음 |
| MV14 | 두번터치 | 겹자음 |
| MV15 | 유지 | 장자음 |

---

## 📂 작업 순서

### 1단계: 기존 파일 백업
```bash
cd /var/www/wia.live/docs/TALK_LANGUAGES/
mkdir -p backup-20241212-v2
mv batch-02-talk-detailed.md backup-20241212-v2/
mv batch-03-talk-detailed.md backup-20241212-v2/
mv batch-04-talk-detailed.md backup-20241212-v2/
mv batch-05-talk-detailed.md backup-20241212-v2/
mv batch-06-talk-detailed.md backup-20241212-v2/
mv batch-07-talk-detailed.md backup-20241212-v2/
mv batch-08-talk-detailed.md backup-20241212-v2/
mv batch-09-talk-detailed.md backup-20241212-v2/
mv batch-10-talk-detailed.md backup-20241212-v2/
mv batch-11-talk-detailed.md backup-20241212-v2/
mv batch-12-talk-detailed.md backup-20241212-v2/
mv batch-13-talk-detailed.md backup-20241212-v2/
mv batch-14-talk-detailed.md backup-20241212-v2/
```

### 2단계: 배치 02부터 순서대로 작성

**각 배치 작성 전 반드시:**
```bash
cat /var/www/wia.live/docs/BRAILLE_LANGUAGES/batch-02-braille-detailed.md | head -100
```

### 3단계: 작성 후 검증
```bash
wc -l /var/www/wia.live/docs/TALK_LANGUAGES/batch-02-talk-detailed.md
# 800줄 이상이어야 함!
```

---

## ✅ 체크리스트 (각 배치마다 확인)

- [ ] Braille 파일 먼저 읽었는가?
- [ ] 언어 순서가 Braille과 100% 동일한가?
- [ ] 번호가 배치별 1부터 시작하는가?
- [ ] 각 언어마다 상세 매핑 (자음/모음 테이블)이 있는가?
- [ ] 예시 단어가 포함되어 있는가?
- [ ] 최소 줄 수를 충족하는가?

---

## ⚠️ 경고

**다시 요약만 하면 안 됩니다!**
**Braille 순서를 무시하면 안 됩니다!**
**배치 01 스타일로 상세하게 작성하세요!**

---

*세계인증산업협회(WIA) 연삼흠 박사*
*2025-12-12*
