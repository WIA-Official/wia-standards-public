# WIHP Tools

WIA International Hangul Phonology (WIHP) 도구 모음

## 📦 설치

```bash
# Python 3.8+ 필요
cd /path/to/WIHP/tools

# 의존성 없음 - 순수 Python 표준 라이브러리만 사용
```

## 🛠️ 도구 목록

### 1. verify_ipa.py - IPA 검증 도구

WIHP 매핑 파일의 IPA 정확성을 자동으로 검증합니다.

**기능:**
- IPA 기호 유효성 검사 (Unicode IPA 차트 기준)
- WIHP 코드 형식 검증 (PL##-MN##-AR##, VL##-VH##-VR##)
- 누락된 매핑 탐지
- 한글 전사 존재 여부 확인
- JSON 보고서 생성

**사용법:**
```bash
# 모든 티어 검증
python verify_ipa.py

# 특정 티어만 검증
python verify_ipa.py --tier 1

# 상세 출력
python verify_ipa.py --verbose

# JSON 보고서 생성
python verify_ipa.py --report
```

**출력 예시:**
```
📊 SUMMARY
   Total files validated: 67
   Valid files: 67
   Total mappings: 2,450
   Total sentences: 3,350
   Errors: 0
   Warnings: 12

📁 TIER1
   ✅ korean-kor.md
   ✅ japanese-jpn.md
   ✅ chinese-cmn.md
```

---

### 2. generate_flashcards.py - 플래시카드 생성기

WIHP 매핑 파일에서 학습용 플래시카드를 생성합니다.

**출력 형식:**
- **Anki**: `.txt` (탭 구분, Anki 가져오기용)
- **Quizlet**: `.txt` (Quizlet 가져오기용)
- **CSV**: 표준 CSV 형식
- **JSON**: 커스텀 앱용 구조화된 JSON
- **HTML**: 인터랙티브 웹 플래시카드

**카드 유형:**
- 음소 카드: IPA → 한글
- 문장 카드: 원문 → 번역 + IPA + 한글
- 어휘 카드: 단어 + 의미

**사용법:**
```bash
# 모든 형식으로 내보내기
python generate_flashcards.py

# Anki 형식만
python generate_flashcards.py --format anki

# 특정 티어만
python generate_flashcards.py --tier 1

# HTML 인터랙티브 카드
python generate_flashcards.py --format html
```

**HTML 플래시카드 기능:**
- 키보드 단축키 (←→ 이동, 스페이스 뒤집기)
- 언어별/유형별 필터링
- 셔플 기능
- 진행률 표시

---

### 3. compare_pronunciation.py - 발음 비교 도구

여러 언어에서 동일한 개념이 어떻게 발음되는지 비교합니다.

**비교 카테고리:**
- `greetings`: 인사말 (안녕, 감사합니다, 등)
- `numbers`: 숫자 1-10
- `colors`: 색깔 (흰색, 검정, 빨강, 등)
- `family`: 가족 호칭 (어머니, 아버지, 등)
- `days`: 요일

**사용법:**
```bash
# 인사말 비교
python compare_pronunciation.py --category greetings

# 숫자 비교 (특정 언어만)
python compare_pronunciation.py --category numbers --languages "Korean,Japanese,Chinese"

# 모든 카테고리, HTML 출력
python compare_pronunciation.py --category all --format html
```

**출력 예시 (Markdown):**
```markdown
| Concept | Korean | Japanese | Chinese |
|---------|--------|----------|---------|
| **hello** | 안녕하세요<br>/annjʌŋhaseyo/<br>안녕하세요 | こんにちは<br>/konnitɕiwa/<br>콘니치와 | 你好<br>/ni˧˥xau˧˩˦/<br>니하오 |
```

---

### 4. phoneme_contrast.py - 음운 대조 도구

언어 간 음운 체계를 분석하고 비교합니다.

**기능:**
- 음소 인벤토리 추출 (자음/모음)
- 두 언어 간 대조 분석
- 공유/고유 음소 식별
- 유형론적 특징 매트릭스
- 특수 음운 특징 탐지 (클릭음, 내파음, 방출음 등)

**탐지 가능한 특수 특징:**
- clicks (클릭음)
- implosives (내파음)
- ejectives (방출음)
- retroflexes (권설음)
- pharyngeals (인두음)
- uvulars (목젖음)
- vowel_nasalization (비모음)
- vowel_harmony (모음조화)
- tones (성조)

**사용법:**
```bash
# 전체 요약 생성
python phoneme_contrast.py --summary

# 두 언어 비교
python phoneme_contrast.py --languages "Korean,Japanese"

# 특정 특징을 가진 언어 찾기
python phoneme_contrast.py --feature clicks

# 유형론 매트릭스 (HTML)
python phoneme_contrast.py --typology --format html
```

**비교 출력 예시:**
```markdown
# Phoneme Contrast: Korean vs Japanese

## Consonants
| Category | Count |
|----------|-------|
| Korean total | 19 |
| Japanese total | 15 |
| Shared | 12 |

### Shared Consonants
/p/, /t/, /k/, /m/, /n/, /s/, /h/...

### Unique to Korean
/pʰ/, /tʰ/, /kʰ/, /p͈/, /t͈/, /k͈/...
```

---

## 📁 출력 디렉토리

모든 도구의 기본 출력 경로:
```
/WIHP/tools/output/
├── validation_report.json
├── wihp_flashcards.txt      # Anki
├── wihp_quizlet.txt         # Quizlet
├── wihp_flashcards.csv
├── wihp_flashcards.json
├── wihp_flashcards.html     # Interactive
├── comparison_greetings.md
├── comparison_greetings.html
├── phoneme_summary.md
├── typology_matrix.html
└── contrast_Korean_Japanese.md
```

---

## 🔧 공통 옵션

모든 도구에서 사용 가능한 옵션:

| 옵션 | 설명 |
|------|------|
| `--root PATH` | WIHP 루트 디렉토리 지정 |
| `--output PATH` | 출력 디렉토리 지정 |
| `--tier N` | 특정 티어만 처리 (1-4) |
| `--format FMT` | 출력 형식 (markdown, html, json, csv) |
| `--verbose, -v` | 상세 출력 |
| `--help, -h` | 도움말 표시 |

---

## 📊 사용 예시

### 전체 검증 및 보고서 생성
```bash
cd /path/to/WIHP/tools

# 1. IPA 검증
python verify_ipa.py --report

# 2. 플래시카드 생성
python generate_flashcards.py --format all

# 3. 언어 비교 테이블
python compare_pronunciation.py --category all --format html

# 4. 음운 분석
python phoneme_contrast.py --summary
python phoneme_contrast.py --typology --format html
```

### 특정 언어 학습 자료 생성
```bash
# 한국어-일본어 비교 자료
python compare_pronunciation.py --languages "Korean,Japanese" --format html
python phoneme_contrast.py --languages "Korean,Japanese"

# 특정 언어 플래시카드
python generate_flashcards.py --language Korean --format anki
```

### CI/CD 통합
```bash
# 검증 실패 시 exit code 1 반환
python verify_ipa.py
if [ $? -ne 0 ]; then
    echo "Validation failed!"
    exit 1
fi
```

---

## 🌐 WIHP 코드 체계

### 자음 코드 (PL##-MN##-AR##)
- **PL**: 조음 위치 (Place)
  - PL01: 양순음, PL02: 순치음, PL03: 치음
  - PL04: 치경음, PL05: 권설음, PL06: 경구개음
  - PL07: 연구개음, PL08: 구개수음, PL09: 성문음
- **MN**: 조음 방식 (Manner)
  - MN01: 파열음, MN02: 비음, MN03: 마찰음
  - MN04: 파찰음, MN05: 접근음, MN06: 설측음
- **AR**: 발성 유형 (Airstream)
  - AR01: 무성, AR02: 유기, AR03: 유성
  - AR04: 기식, AR05: 내파, AR06: 방출

### 모음 코드 (VL##-VH##-VR##)
- **VL**: 전설/중설/후설 (Vowel Location)
- **VH**: 고/중/저 (Vowel Height)
- **VR**: 원순/비원순 (Vowel Rounding)

---

## 📝 라이선스

MIT License - WIA Official

---

## 🤝 기여

버그 리포트 및 기능 제안:
- GitHub Issues: https://github.com/WIA-Official/WIHP/issues
