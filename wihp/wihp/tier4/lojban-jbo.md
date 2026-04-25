# WIHP: Lojban - jbo

**어족**: Constructed > Logical Language
**문자**: Latin alphabet (26 letters)
**화자 수**: 약 1,500명 (유창자 수백 명)
**난이도**: ★★★★☆ (논리적이지만 복잡함)

---

## 1. 음운 체계

### 1.1 자음 (Consonants) - 17개

| 한글 표기 | IPA | WIHP 코드 | 로즈반 철자 | 예시 |
|-----------|-----|-----------|-------------|------|
| ㅂ | /b/ | PL02-MN01-AR01 | b | **b**roda [broda] 뭔가 1 |
| ㅍ | /p/ | PL02-MN01-AR00 | p | **p**lise [plise] 사과 |
| ㄷ | /d/ | PL04-MN01-AR01 | d | **d**unda [dunda] 주다 |
| ㅌ | /t/ | PL04-MN01-AR00 | t | **t**avla [tavla] 말하다 |
| ㄱ | /ɡ/ | PL07-MN01-AR01 | g | **g**leki [ɡleki] 행복한 |
| ㅋ | /k/ | PL07-MN01-AR00 | k | **k**arce [kartʃe] 차 |
| ㅁ | /m/ | PL02-MN02-AR01 | m | **m**latu [mlatu] 고양이 |
| ㄴ | /n/ | PL04-MN02-AR01 | n | **n**anmu [nanmu] 남자 |
| ㄹ | /r/ | PL04-MN05-AR01 | r | **r**onte [ronte] 녹다 |
| ㄹ(측음) | /l/ | PL04-MN04-AR01 | l | **l**ojban [loʒban] 로즈반 |
| ㅂ(마) | /v/ | PL01-MN06-AR01 | v | **v**oksa [voksa] 목소리 |
| ㅍ(마) | /f/ | PL01-MN06-AR00 | f | **f**inti [finti] 발명하다 |
| ㅅ | /s/ | PL04-MN06-AR00 | s | **s**abji [sabdʒi] 현명한 |
| ㅈ | /z/ | PL04-MN06-AR01 | z | **z**ekri [zekri] 범죄 |
| ㅅ(후) | /ʃ/ | PL05-MN06-AR00 | c | **c**izra [ʃizra] 이상한 |
| ㅈ(후) | /ʒ/ | PL05-MN06-AR01 | j | **j**dini [ʒdini] 종교 |
| ㅎ | /x~h/ | PL07-MN06-AR00 | x | **x**rula [xrula] 꽃 |

### 1.2 모음 (Vowels) - 6개

| 한글 표기 | IPA | WIHP 코드 | 로즈반 철자 | 예시 |
|-----------|-----|-----------|-------------|------|
| 아 | /a/ | V01-L01-R00 | a | d**a** [da] 이것 |
| 애 | /ɛ~e/ | V03-L02-R00 | e | t**e** [te] 저것 |
| 이 | /i/ | V01-L03-R00 | i | d**i** [di] 지금 |
| 오 | /o/ | V05-L02-R01 | o | l**o** [lo] 정관사 |
| 우 | /u/ | V07-L03-R01 | u | t**u** [tu] 너 |
| 으 | /ə~ɨ/ | V04-L02-R00 | y | b**y** [bə] 모음 분리자 |

### 1.3 이중모음 (Diphthongs)
| 한글 표기 | IPA | 로즈반 철자 | 예시 |
|-----------|-----|-------------|------|
| 아이 | /ai/ | ai | **ai** [ai] 애정 |
| 에이 | /ei/ | ei | **ei** [ei] 이 |
| 오이 | /oi/ | oi | **oi** [oi] 저기 |
| 아우 | /au/ | au | **au** [au] 오! |

---

## 2. 설계 철학 (Design Philosophy)

### 2.1 논리 언어의 원칙
```
로즈반은 술어 논리(predicate logic)를 기반으로 합니다:

1. 문법적 모호성 제거
   - 모든 문장은 단 하나의 해석만 가능
   - 파싱이 알고리즘적으로 가능

2. 문화적 중립성
   - 6개 주요 언어에서 균등하게 어휘 차용
   (중국어, 영어, 힌디어, 스페인어, 러시아어, 아랍어)

3. 표현력
   - 자연 언어로 표현 가능한 모든 것을 표현
   - 논리적 관계를 명시적으로 표현
```

### 2.2 Gismu (기본 단어 1350개)
```
모든 gismu는 5글자 (CVCCV 또는 CCVCV):
  broda, mlatu, nanmu, tavla...

6개 언어에서 점수를 계산하여 선정:
  - 음성적 유사성
  - 화자 수 가중치
  - 최대 문화적 중립성
```

### 2.3 selbri와 sumti (술어와 논항)
```
로즈반 문장의 핵심 구조:

selbri (술어): 관계를 나타냄
  tavla = x1은 x2에게 x3에 대해 x4 언어로 말한다

sumti (논항): 자리를 채우는 것들
  mi tavla do ti la .lojban.
  (나는 너에게 이것에 대해 로즈반으로 말한다)

모든 selbri는 논리적 장소 구조를 가짐
```

---

## 3. 기본 표현 (50문장)

### 인사 (Greetings)
| # | 로즈반 | IPA | 한글 발음 | 의미 |
|---|--------|-----|-----------|------|
| 1 | coi | [tʃoi] | 초이 | 안녕하세요 |
| 2 | coi do | [tʃoi do] | 초이 도 | 안녕하세요 (당신에게) |
| 3 | co'o | [tʃoʔo] | 초오 | 안녕 (작별) |
| 4 | ki'e | [kiʔe] | 키에 | 감사합니다 |
| 5 | .oi | [oi] | 오이 | 오! (불만) |
| 6 | .uu | [uu] | 우우 | 불쌍해 |
| 7 | .ui | [ui] | 우이 | 야호! (행복) |
| 8 | doi | [doi] | 도이 | 이봐 (호칭) |
| 9 | ju'i | [dʒuʔi] | 주이 | 여기요! (주의 끌기) |
| 10 | pe'u | [peʔu] | 페우 | 제발 |

### 기본 회화 (Basic Conversation)
| # | 로즈반 | IPA | 한글 발음 | 의미 |
|---|--------|-----|-----------|------|
| 11 | go'i | [ɡoʔi] | 고이 | 네 (동의) |
| 12 | na go'i | [na ɡoʔi] | 나 고이 | 아니오 |
| 13 | ie | [ie] | 이에 | 동의합니다 |
| 14 | ie nai | [ie nai] | 이에 나이 | 반대합니다 |
| 15 | xu do gleki | [xu do ɡleki] | 슈 도 글레키 | 행복하세요? |
| 16 | mi gleki | [mi ɡleki] | 미 글레키 | 행복해요 |
| 17 | mi na jimpe | [mi na dʒimpe] | 미 나 짐페 | 이해 못 해요 |
| 18 | mi jimpe | [mi dʒimpe] | 미 짐페 | 이해합니다 |
| 19 | xu do cusku bau la .gliban. | [xu do tʃusku bau la ɡliban] | 슈 도 추스쿠 바우 라 글리반 | 영어 하세요? |
| 20 | mi tavla fi la .lojban. | [mi tavla fi la loʒban] | 미 타블라 피 라 로즈반 | 로즈반으로 말합니다 |

### 질문과 대답 (Questions & Answers)
| # | 로즈반 | IPA | 한글 발음 | 의미 |
|---|--------|-----|-----------|------|
| 21 | ti mo | [ti mo] | 티 모 | 이것이 무엇입니까? |
| 22 | ma | [ma] | 마 | 무엇? 누구? |
| 23 | ma noi prenu cu zvati | [ma noi prenu tʃu zvati] | 마 노이 프레누 추 즈바티 | 누가 있나요? |
| 24 | fi'i | [fiʔi] | 피이 | 어디? |
| 25 | ca ma | [tʃa ma] | 차 마 | 언제? |
| 26 | ri'a ma | [riʔa ma] | 리아 마 | 왜? |
| 27 | ta poi zdani cu jdika | [ta poi zdani tʃu dʒdika] | 타 포이 즈다니 추 즈디카 | 저 집이 어느 방향이에요? |
| 28 | ti djica ma | [ti dʒitʃa ma] | 티 즈차 마 | 이것은 무엇을 원하나요? |
| 29 | xu do kakne lo nu sidju mi | [xu do kakne lo nu sidʒu mi] | 슈 도 카크네 로 누 시즈주 미 | 도와주실 수 있어요? |
| 30 | ma cmene do | [ma tʃmene do] | 마 츰네 도 | 성함이 무엇입니까? |

### 일상 표현 (Daily Expressions)
| # | 로즈반 | IPA | 한글 발음 | 의미 |
|---|--------|-----|-----------|------|
| 31 | mi cmene zo... | [mi tʃmene zo] | 미 츰네 조... | 제 이름은...입니다 |
| 32 | mi se zdani la .xuguos. | [mi se zdani la xuɡuos] | 미 세 즈다니 라 후구오스 | 저는 한국에 삽니다 |
| 33 | mi xagji | [mi xadʒi] | 미 하지 | 배고파요 |
| 34 | mi taske | [mi taske] | 미 타스케 | 목 말라요 |
| 35 | mi tatpi | [mi tatpi] | 미 탓피 | 피곤해요 |
| 36 | .e'o denpa | [eʔo denpa] | 에오 덴파 | 기다려 주세요 |
| 37 | .e'o klama ti | [eʔo klama ti] | 에오 클라마 티 | 이리 오세요 |
| 38 | je'u | [dʒeʔu] | 제우 | 정말로 |
| 39 | na je'u | [na dʒeʔu] | 나 제우 | 정말 아님 |
| 40 | .ei mi cliva | [ei mi tʃliva] | 에이 미 츨리바 | 가야겠어요 |

### 식당/쇼핑 (Restaurant/Shopping)
| # | 로즈반 | IPA | 한글 발음 | 의미 |
|---|--------|-----|-----------|------|
| 41 | .e'o do bevri le vi liste | [eʔo do bevri le vi liste] | 에오 도 베브리 레 비 리스테 | 메뉴 주세요 |
| 42 | mi djica lo nu citka | [mi dʒitʃa lo nu tʃitka] | 미 즈차 로 누 치트카 | 먹고 싶어요 |
| 43 | .e'o jdima | [eʔo dʒdima] | 에오 즈디마 | 계산서 주세요 |
| 44 | ti vrude | [ti vrude] | 티 브루데 | 맛있어요 |
| 45 | ki'e do | [kiʔe do] | 키에 도 | 감사합니다 |
| 46 | .e'o pa birje | [eʔo pa birdʒe] | 에오 파 비르제 | 맥주 주세요 |
| 47 | mi djica lo ckafi | [mi dʒitʃa lo tʃkafi] | 미 즈차 로 츠카피 | 커피 주세요 |
| 48 | ti jdika | [ti dʒdika] | 티 즈디카 | 너무 비싸요 |
| 49 | xu da jdima | [xu da dʒdima] | 슈 다 즈디마 | 할인 있나요? |
| 50 | .e'o do bevri ti | [eʔo do bevri ti] | 에오 도 베브리 티 | 포장해 주세요 |

---

## 4. 숫자 (Numbers)

### 4.1 기본 숫자 (십진법)
```
로즈반은 십진법을 사용하며,
숫자는 PA 클래스 단어입니다.
```

| 숫자 | 로즈반 | IPA | 한글 표기 |
|------|--------|-----|-----------|
| 0 | no | [no] | 노 |
| 1 | pa | [pa] | 파 |
| 2 | re | [re] | 레 |
| 3 | ci | [tʃi] | 치 |
| 4 | vo | [vo] | 보 |
| 5 | mu | [mu] | 무 |
| 6 | xa | [xa] | 하 |
| 7 | ze | [ze] | 제 |
| 8 | bi | [bi] | 비 |
| 9 | so | [so] | 소 |
| 10 | pano | [pano] | 파노 |
| 11 | papa | [papa] | 파파 |
| 20 | reno | [reno] | 레노 |
| 100 | pa ki'o ki'o | [pa kiʔo kiʔo] | 파 키오 키오 |

### 4.2 특수 숫자 표현
```
로즈반은 다양한 진법을 지원:

pa pi re (1.2 - 십진 소수)
pa fi'u re (1/2 - 분수)
re sy'e ci (2³ - 지수)

ju'u = 십육진법
pi'e = 연결 (전화번호 등)
```

---

## 5. 요일 (Days of the Week)

```
로즈반은 표준 요일 체계를 사용하지 않고,
논리적으로 표현합니다.
```

| 한국어 | 로즈반 | IPA | 한글 표기 | 의미 |
|--------|--------|-----|-----------|------|
| 월요일 | pavdijy | [pavdidʒə] | 파브디지 | 1번 요일 |
| 화요일 | reldijy | [reldidʒə] | 렐디지 | 2번 요일 |
| 수요일 | cibdijy | [tʃibdidʒə] | 칩디지 | 3번 요일 |
| 목요일 | vondijy | [vondidʒə] | 본디지 | 4번 요일 |
| 금요일 | mumdijy | [mumdidʒə] | 뭄디지 | 5번 요일 |
| 토요일 | xavdijy | [xavdidʒə] | 하브디지 | 6번 요일 |
| 일요일 | zeldijy | [zeldidʒə] | 젤디지 | 7번 요일 |

**또는 전통 이름 (lujvo):**
| 한국어 | 로즈반 | 의미 |
|--------|--------|------|
| 월요일 | nondei | 달의 날 |
| 일요일 | soldei | 태양의 날 |

---

## 6. 색깔 (Colors)

```
로즈반 색깔은 gismu(기본 단어):
```

| 한국어 | 로즈반 | IPA | 한글 표기 | 정의 |
|--------|--------|-----|-----------|------|
| 빨간색 | xunre | [xunre] | 훈레 | x1은 빨갛다 |
| 주황색 | narju | [nardʒu] | 나르주 | x1은 주황색이다 |
| 노란색 | pelxu | [pelxu] | 펠후 | x1은 노랗다 |
| 초록색 | crino | [tʃrino] | 츠리노 | x1은 초록색이다 |
| 파란색 | blanu | [blanu] | 블라누 | x1은 파랗다 |
| 하얀색 | blabi | [blabi] | 블라비 | x1은 하얗다 |
| 검은색 | xekri | [xekri] | 헤크리 | x1은 검다 |
| 회색 | grusi | [ɡrusi] | 그루시 | x1은 회색이다 |
| 갈색 | bunre | [bunre] | 분레 | x1은 갈색이다 |
| 보라색 | zirpu | [zirpu] | 지르푸 | x1은 보라색이다 |

**사용 예:**
```
le xunre zdani = 빨간 집
(lo를 사용하면 일반적, le는 특정)
```

---

## 7. 가족 (Family Terms)

| 한국어 | 로즈반 | IPA | 한글 표기 | 장소 구조 |
|--------|--------|-----|-----------|-----------|
| 아버지 | patfu | [patfu] | 파트푸 | x1은 x2의 아버지 |
| 어머니 | mamta | [mamta] | 맘타 | x1은 x2의 어머니 |
| 아들 | bersa | [bersa] | 베르사 | x1은 x2의 아들 |
| 딸 | tixnu | [tixnu] | 틱스누 | x1은 x2의 딸 |
| 형제 | bruna | [bruna] | 브루나 | x1은 x2의 형제 |
| 자매 | mensi | [mensi] | 멘시 | x1은 x2의 자매 |
| 할아버지 | patfu be le patfu | [patfu be le patfu] | 파트푸 베 레 파트푸 | 아버지의 아버지 |
| 할머니 | mamta be le patfu | [mamta be le patfu] | 맘타 베 레 파트푸 | 아버지의 어머니 |
| 배우자 | speni | [speni] | 스페니 | x1은 x2의 배우자 |
| 자녀 | verba | [verba] | 베르바 | x1은 아이 |

---

## 8. 문법 (Grammar)

### 8.1 bridi 구조 (술어 구조)
```
기본 구조:
  [sumti] selbri [sumti] [sumti]...

예:
  mi tavla do ti
  (나는 너에게 이것에 대해 말한다)

  mi = x1 (나)
  tavla = selbri (말하다)
  do = x2 (너)
  ti = x3 (이것)
```

### 8.2 FA 태그 (장소 태그)
```
fa, fe, fi, fo, fu = 장소 1, 2, 3, 4, 5

순서를 바꿀 수 있음:
  mi tavla do ti
  = fi ti fe do fa mi tavla
  (어순 자유롭게 재배치)
```

### 8.3 시제 체계 (Tense)
```
ba = 미래
ca = 현재
pu = 과거

예:
  mi pu klama le zarci
  (나는 시장에 갔다)

  mi ba citka
  (나는 먹을 것이다)
```

### 8.4 논리 연결사
```
.a = 또는 (or)
.e = 그리고 (and)
.o = 배타적 or (xor)
.u = 관계없음

na = 부정
ja = 만약...하면

예:
  mi citka je pinxe
  (나는 먹고 마신다)
```

### 8.5 cmavo (문법 단어)
```
로즈반의 핵심:

lo = 정관사 (논리적)
le = 정관사 (화자 의도)
la = 고유명사 마커
zo = 다음 단어를 인용
lu...li'u = 텍스트 인용
```

---

## 9. 철학적 개념 (Philosophical Concepts)

### 9.1 사피어-워프 가설 실험
| 로즈반 | 의미 |
|--------|------|
| bangu spofu | 언어 결함 (자연 언어의 모호성) |
| logji bangu | 논리 언어 |
| jufra tcaci | 문장 진리값 |
| cusku stodi | 명확한 표현 |
| kulnu midju | 문화적 중립성 |

### 9.2 로즈반의 목표
```
1. 논리적 사고 촉진
   - 모호성 제거로 명확한 사고
   - 논리적 관계를 명시적으로 표현

2. 사피어-워프 가설 검증
   - 언어가 사고를 형성하는가?
   - 논리 언어가 논리적 사고를 촉진하는가?

3. 인간-컴퓨터 통신
   - 파싱 가능한 구조
   - 번역 프로그램 제작 가능

4. 문화적 중립성
   - 어떤 문화도 우선하지 않음
   - 6개 주요 언어에서 균등 차용
```

### 9.3 attitudinals (태도 표시자)
```
로즈반의 독특한 특징:

.ui = 행복
.uu = 연민
.oi = 불만
.ie = 동의
.ia = 믿음
.au = 욕망

문장 어디든 삽입 가능:
  mi .ui tavla do
  (나는 [행복하게] 너에게 말한다)
```

---

## 10. 문화 표현 (Cultural Expressions)

### 10.1 로즈반 커뮤니티 용어
| 로즈반 | 의미 |
|--------|------|
| lojbo | 로즈반 사용자/로즈반의 |
| la .lojban. | 로즈반 (고유명사) |
| jbobau | 로즈반 언어로 |
| vlaste | 사전 |
| guskant | 이야기, 소설 |
| jufra | 문장 |
| selsku | 의미된 것 |

### 10.2 Logfest (로즈반 축제)
```
연례 모임:
- 로즈반으로만 대화
- 언어 발전 논의
- 새로운 lujvo (합성어) 제안
- Loglandia (가상 국가 개념)
```

### 10.3 유명한 문장
```
lo jbobau cu cafne
(로즈반은 명확하다)

mi nelci lo nu tavla bau la .lojban.
(나는 로즈반으로 말하는 것을 좋아한다)

ko kurji la .lojban.
(로즈반을 돌봐라!)
```

### 10.4 문학과 창작
```
로즈반으로 번역된 작품:
- 이상한 나라의 앨리스
- 어린 왕자
- 다양한 시

원창작:
- la .teris. poi jbopre (로즈반 SF 소설)
- 로즈반 노래들
- 로즈반 하이쿠 실험
```

---

## 11. 실전 예문 (Complex Sentences)

### 11.1 논리 구조 예시
```
mi djica lo nu do klama le zarci
(나는 [너가 시장에 가는 것]을 원한다)

분석:
  mi = x1 (나)
  djica = 원하다
  lo nu = 추상화 (사건)
  do = 너
  klama = 가다
  le zarci = 그 시장
```

### 11.2 복잡한 관계
```
le prenu poi mi tavla ke'a cu gleki
(내가 말하는 그 사람은 행복하다)

poi = 관계절 시작
ke'a = 선행사 (되돌아가는 대명사)
```

---

## 참고 문헌 (References)

1. Cowan, John Woldemar (2016). *The Complete Lojban Language*
2. La .krtisfranks. (2020). *Lojban Reference Grammar*
3. Lojban.org - Official Lojban Community
4. La Sutysisku - Lojban Dictionary
5. Brown, James Cooke (1989). *Loglan 1: A Logical Language* (Loglan의 선구)

---

---

弘益人間 · Benefit All Humanity
WIA - World Certification Industry Association
© 2025 MIT License
