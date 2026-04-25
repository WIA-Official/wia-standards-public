# 🚨 WIA Talk 전체 재작성 프롬프트

## ⚠️ 심각한 문제: Talk 배치 01-14 전부 Braille과 불일치

| 배치 | Braille | Talk | 상태 |
|------|---------|------|------|
| 01 | ✅ | ✅ | 동일 (유지) |
| 02 | es-AR, fr-CA, ar, hi... | pt, it, nl, ru... | ❌ 재작성 |
| 03 | ta, mr, gu, kn... | ar, he, fa, tr... | ❌ 재작성 |
| 04 | tl, ko-KP, fa, tr... | sw, ms, tl, ta... | ❌ 재작성 |
| 05 | nl, sv, da, el... | sv, no, da, et... | ❌ 재작성 |
| 06 | sl, sr, lv, lt... | bg, mk, sr, kk... | ❌ 재작성 |
| 07-14 | ✅ 있음 | ❌ 잘못됨/없음 | ❌ 재작성 |

---

## 🎯 작업 지시

### 0단계: 기존 파일 전체 백업
```bash
cd /var/www/wia.live/docs/TALK_LANGUAGES/
mkdir -p backup-$(date +%Y%m%d)
cp batch-*.md backup-$(date +%Y%m%d)/
```

### 1단계: 마스터 목록 확인
```bash
cat /var/www/wia.live/docs/MASTER-LANGUAGE-LIST.md
```

### 2단계: 배치 02-14 재작성 (배치 01은 유지)

**Braille 마스터 기준으로 정확히 동일한 언어, 동일한 순서!**

---

## 📋 Braille 마스터 기준 전체 목록

### 배치 01 (10개) ✅ 유지
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

### 배치 02 (10개) ❌ 재작성
11. Español Rioplatense (Argentine Spanish) [es-AR]
12. Français Canadien [fr-CA]
13. Italiano [it]
14. Português Europeu [pt]
15. Português Brasileiro [pt-BR]
16. العربية (Arabic) [ar]
17. हिन्दी (Hindi) [hi]
18. বাংলা (Bengali) [bn]
19. ਪੰਜਾਬੀ (Punjabi) [pa]
20. తెలుగు (Telugu) [te]

### 배치 03 (10개) ❌ 재작성
21. தமிழ் (Tamil) [ta]
22. मराठी (Marathi) [mr]
23. ગુજરાતી (Gujarati) [gu]
24. ಕನ್ನಡ (Kannada) [kn]
25. മലയാളം (Malayalam) [ml]
26. اردو (Urdu) [ur]
27. ภาษาไทย (Thai) [th]
28. Tiếng Việt (Vietnamese) [vi]
29. Bahasa Indonesia [id]
30. Bahasa Melayu [ms]

### 배치 04 (10개) ❌ 재작성
31. Tagalog/Filipino [tl]
32. 조선어 (Korean-DPRK) [ko-KP]
33. 日本語 かな専用 [ja-kana]
34. فارسی (Persian) [fa]
35. Türkçe (Turkish) [tr]
36. Polski (Polish) [pl]
37. Українська (Ukrainian) [uk]
38. Русский (Russian) [ru]
39. Čeština (Czech) [cs]
40. Română (Romanian) [ro]

### 배치 05 (9개) ❌ 재작성
41. Nederlands (Dutch) [nl]
42. Svenska (Swedish) [sv]
43. Dansk (Danish) [da]
44. Norsk (Norwegian) [no]
45. Suomi (Finnish) [fi]
46. Ελληνικά (Greek) [el]
47. Magyar (Hungarian) [hu]
48. Български (Bulgarian) [bg]
49. Slovenčina (Slovak) [sk]

### 배치 06 (10개) ❌ 재작성
50. Slovenščina (Slovenian) [sl]
51. Српски (Serbian) [sr]
52. Македонски (Macedonian) [mk]
53. Shqip (Albanian) [sq]
54. Latviešu (Latvian) [lv]
55. Lietuvių (Lithuanian) [lt]
56. Eesti (Estonian) [et]
57. ქართული (Georgian) [ka]
58. Հայերdelays (Armenian) [hy]
59. Azərbaycan (Azerbaijani) [az]

### 배치 07 (21개) ❌ 재작성
60. Filipino [fil]
61. עברית (Hebrew) [he]
62. Hrvatski (Croatian) [hr]
63. Oʻzbek (Uzbek) [uz]
64. Қазақ (Kazakh) [kk]
65. Кыргызча (Kyrgyz) [ky]
66. Тоҷикӣ (Tajik) [tg]
67. Монгол (Mongolian) [mn]
68. ភាសាខ្មែរ (Khmer) [km]
69. ພາສາລາວ (Lao) [lo]
70. မြန်မာဘာသာ (Burmese) [my]
71. සිංහල (Sinhala) [si]
72. नेपाली (Nepali) [ne]
73. Kiswahili (Swahili) [sw]
74. አማርኛ (Amharic) [am]
75. Hausa [ha]
76. Igbo [ig]
77. Yorùbá [yo]
78. isiZulu [zu]
79. isiXhosa [xh]
80. Afrikaans [af]

### 배치 08 (18개) ❌ 새로 작성
81-98: 아프리카 언어 (Tigrinya, Oromo, Somali, Malagasy...)

### 배치 09 (16개) ❌ 새로 작성
99-114: 유럽 소수 언어 (Catalan, Basque, Welsh, Irish...)

### 배치 10 (16개) ❌ 새로 작성
115-130: 유럽 지역 언어 (Asturian, Romansh, Yiddish...)

### 배치 11 (13개) ❌ 새로 작성
131-143: 슬라브 소수 언어 (Belarusian, Rusyn, Sorbian...)

### 배치 12 (9개) ❌ 새로 작성
144-152: 우랄 언어 (Komi, Mari, Udmurt...)

### 배치 13 (10개) ❌ 새로 작성
153-162: 튀르크 언어 (Tuvan, Yakut, Chuvash...)

### 배치 14 (18개) ❌ 새로 작성
163-180: 토착/태평양 언어 (Ainu, Inuktitut, Hawaiian, Maori...)

---

## 📝 각 언어 작성 형식

```markdown
# [전체번호]. [언어명] ([원어명]) [ISO코드]

## 언어 개요
- 사용자 수: 
- 사용 지역: 
- 어족: 

## 자음 → WIA Talk 매핑

| 자음 | IPA | 손모양 | 위치 | 움직임 |
|-----|-----|--------|-----|--------|
| ... | [.] | HS## | LC## | MV## |

## 모음 → WIA Talk 매핑

| 모음 | IPA | 손모양 | 위치 | 움직임 |
|-----|-----|--------|-----|--------|
| ... | [.] | HS## | LC## | MV## |

## 예시 단어

| 단어 | 의미 | IPA | 제스처 시퀀스 |
|-----|------|-----|--------------|
```

---

## ✅ 작업 순서

1. ✅ 배치 01 유지
2. ⏳ 배치 02 재작성 → batch-02-talk-detailed.md
3. ⏳ 배치 03 재작성 → batch-03-talk-detailed.md
4. ⏳ 배치 04 재작성 → batch-04-talk-detailed.md
5. ⏳ 배치 05 재작성 → batch-05-talk-detailed.md
6. ⏳ 배치 06 재작성 → batch-06-talk-detailed.md
7. ⏳ 배치 07 재작성 → batch-07-talk-detailed.md
8. ⏳ 배치 08 새로 작성 → batch-08-talk-detailed.md
9. ⏳ 배치 09 새로 작성 → batch-09-talk-detailed.md
10. ⏳ 배치 10 새로 작성 → batch-10-talk-detailed.md
11. ⏳ 배치 11 새로 작성 → batch-11-talk-detailed.md
12. ⏳ 배치 12 새로 작성 → batch-12-talk-detailed.md
13. ⏳ 배치 13 새로 작성 → batch-13-talk-detailed.md
14. ⏳ 배치 14 새로 작성 → batch-14-talk-detailed.md

---

## 🎯 핵심 원칙

**ALL FIVE ARE EQUAL. NO DEFAULT EXISTS.**
**Braille = Talk 동일한 언어, 동일한 순서!**

---

## 저장 경로

```
/var/www/wia.live/docs/TALK_LANGUAGES/batch-##-talk-detailed.md
```

---

*생성일: 2025-12-12*
*세계인증산업협회(WIA) 연삼흠 박사*
