# WIA Braille 211개 언어 프로젝트
## 남은 작업 가이드 + 경험자 노하우

**철학**: 홍익인간 (弘益人間)  
**원칙**: ALL FIVE ARE EQUAL. NO DEFAULT EXISTS.  
**작성일**: 2024-12-12  
**작성자**: Claude (AI Assistant)

---

# 📊 진행 현황

## ✅ 완료된 작업 (98개 언어)

| 파일 | 언어 수 | 용량 | 내용 |
|------|--------|------|------|
| batch-01 | 10개 | 150KB | en, ko, ja, zh-CN, zh-TW, zh-HK, es, es-MX, fr, de |
| batch-02 | 10개 | 84KB | es-AR, fr-CA, it, pt, pt-BR, ar, hi, bn, pa, te |
| batch-03 | 10개 | 59KB | ta, mr, gu, kn, ml, ur, th, vi, id, ms |
| batch-04 | 10개 | 49KB | tl, ko-KP, ja-kana, fa, tr, pl, uk, ru, cs, ro |
| batch-05 | 10개 | 42KB | nl, sv, da, no, fi, el, hu, bg, sk |
| batch-06 | 10개 | 37KB | sl, sr, mk, sq, lv, lt, et, ka, hy, az |
| batch-07 | 21개 | 71KB | fil, he, hr, uz, kk, ky, tg, mn, km, lo, my, si, ne, sw, am, ha, ig, yo, zu, xh, af |
| batch-08 | 18개 | 19KB | ti, om, so, mg, rw, ny, sn, st, tn, ts, ss, ve, nr, nso, wo, ff, ln, kg |

**총계**: 98개 언어, 525KB

---

# 📋 남은 작업: 원래 프롬프트 Batch 7-21

## ⚠️ 중요 안내
- 아래는 **원래 프롬프트에서 지정된 Batch 7-21**입니다
- 위에서 완료된 batch-07, batch-08 파일과는 **별개**입니다
- 새 파일명: `batch-09-braille-detailed.md`부터 시작 권장
- 또는 원래 프롬프트 번호 유지: `original-batch-07-braille-detailed.md`

---

## 🔵 Batch 7 (20개) - 서유럽 소수언어 + 인도 추가
| # | 코드 | 언어명 | 문자 | 난이도 | 비고 |
|---|------|--------|------|--------|------|
| 1 | gu | ગુજરાતી (Gujarati) | 구자라트 문자 | ⭐⭐⭐ | 이미 batch-03에 있음 - 스킵 |
| 2 | kn | ಕನ್ನಡ (Kannada) | 칸나다 문자 | ⭐⭐⭐ | 이미 batch-03에 있음 - 스킵 |
| 3 | ml | മലയാളം (Malayalam) | 말라얄람 문자 | ⭐⭐⭐ | 이미 batch-03에 있음 - 스킵 |
| 4 | ta-SL | தமிழ் (Tamil - Sri Lanka) | 타밀 문자 | ⭐⭐ | 스리랑카 타밀 변종 |
| 5 | nl | Nederlands (Dutch) | 라틴 | ⭐ | 이미 batch-05에 있음 - 스킵 |
| 6 | ca | Català (Catalan) | 라틴 | ⭐ | 신규 |
| 7 | eu | Euskara (Basque) | 라틴 | ⭐ | 신규 |
| 8 | gl | Galego (Galician) | 라틴 | ⭐ | 신규 |
| 9 | cy | Cymraeg (Welsh) | 라틴 | ⭐ | 신규 |
| 10 | ga | Gaeilge (Irish) | 라틴 | ⭐ | 신규 |
| 11 | mt | Malti (Maltese) | 라틴 | ⭐⭐ | 아랍어 영향, 신규 |
| 12 | is | Íslenska (Icelandic) | 라틴 | ⭐ | 신규 |
| 13 | fo | Føroyskt (Faroese) | 라틴 | ⭐ | 신규 |
| 14 | lb | Lëtzebuergesch (Luxembourgish) | 라틴 | ⭐ | 신규 |
| 15 | fy | Frysk (West Frisian) | 라틴 | ⭐ | 신규 |
| 16 | gd | Gàidhlig (Scottish Gaelic) | 라틴 | ⭐ | 신규 |
| 17 | br | Brezhoneg (Breton) | 라틴 | ⭐ | 신규 |
| 18 | co | Corsu (Corsican) | 라틴 | ⭐ | 신규 |
| 19 | oc | Occitan | 라틴 | ⭐ | 신규 |
| 20 | sc | Sardu (Sardinian) | 라틴 | ⭐ | 신규 |

**실제 신규 작업**: 16개 (중복 4개 제외)

---

## 🔵 Batch 8 (20개) - 로망스어 + 게르만어 방언
| # | 코드 | 언어명 | 문자 | 난이도 |
|---|------|--------|------|--------|
| 1 | ast | Asturianu (Asturian) | 라틴 | ⭐ |
| 2 | an | Aragonés (Aragonese) | 라틴 | ⭐ |
| 3 | rm | Rumantsch (Romansh) | 라틴 | ⭐ |
| 4 | wa | Walon (Walloon) | 라틴 | ⭐ |
| 5 | li | Limburgs (Limburgish) | 라틴 | ⭐ |
| 6 | zea | Zeêuws (Zeelandic) | 라틴 | ⭐ |
| 7 | nds | Plattdüütsch (Low German) | 라틴 | ⭐ |
| 8 | gsw | Schwiizerdütsch (Swiss German) | 라틴 | ⭐ |
| 9 | bar | Boarisch (Bavarian) | 라틴 | ⭐ |
| 10 | pfl | Pfälzisch (Palatinate German) | 라틴 | ⭐ |
| 11 | ksh | Kölsch (Colognian) | 라틴 | ⭐ |
| 12 | sli | Schläsch (Silesian German) | 라틴 | ⭐ |
| 13 | yi | ייִדיש (Yiddish) | 히브리 문자 | ⭐⭐⭐ |
| 14 | frr | Frasch (North Frisian) | 라틴 | ⭐ |
| 15 | stq | Seeltersk (Saterland Frisian) | 라틴 | ⭐ |
| 16 | pdc | Deitsch (Pennsylvania Dutch) | 라틴 | ⭐ |
| 17 | hrx | Hunsrik (Riograndenser Hunsrückisch) | 라틴 | ⭐ |
| 18 | vmf | Mainfränkisch (East Franconian) | 라틴 | ⭐ |
| 19 | swg | Schwäbisch (Swabian) | 라틴 | ⭐ |
| 20 | wae | Walser (Walliser German) | 라틴 | ⭐ |

---

## 🔵 Batch 9 (20개) - 슬라브어 방언 + 발틱
| # | 코드 | 언어명 | 문자 | 난이도 |
|---|------|--------|------|--------|
| 1 | be | Беларуская (Belarusian) | 키릴 | ⭐⭐ |
| 2 | rue | Русиньскый (Rusyn) | 키릴 | ⭐⭐ |
| 3 | szl | Ślōnsko godka (Silesian) | 라틴 | ⭐ |
| 4 | csb | Kaszëbsczi (Kashubian) | 라틴 | ⭐ |
| 5 | hsb | Hornjoserbšćina (Upper Sorbian) | 라틴 | ⭐ |
| 6 | dsb | Dolnoserbšćina (Lower Sorbian) | 라틴 | ⭐ |
| 7 | bs | Bosanski (Bosnian) | 라틴/키릴 | ⭐ |
| 8 | cnr | Crnogorski (Montenegrin) | 라틴/키릴 | ⭐ |
| 9 | ltg | Latgaļu (Latgalian) | 라틴 | ⭐ |
| 10 | sgs | Žemaitiu (Samogitian) | 라틴 | ⭐ |
| 11 | prg | Prūsiskan (Old Prussian - revived) | 라틴 | ⭐⭐ |
| 12 | cu | Словѣньскъ (Church Slavonic) | 키릴 | ⭐⭐⭐ |
| 13 | orv | Древнерусский (Old East Slavic) | 키릴 | ⭐⭐⭐ |
| 14 | sla | Proto-Slavic (reconstructed) | 라틴/키릴 | ⭐⭐⭐ |
| 15 | bat | Proto-Baltic (reconstructed) | 라틴 | ⭐⭐ |
| 16 | sem | Proto-Semitic (reconstructed) | 특수 | ⭐⭐⭐ |
| 17 | ine | Proto-Indo-European (reconstructed) | 특수 | ⭐⭐⭐ |
| 18 | wym | Wymysorys (Vilamovian) | 라틴 | ⭐ |
| 19 | zlw | Proto-West Slavic | 라틴 | ⭐⭐ |
| 20 | zls | Proto-South Slavic | 라틴/키릴 | ⭐⭐ |

---

## 🔵 Batch 10 (20개) - 핀우그릭어 + 시베리아
| # | 코드 | 언어명 | 문자 | 난이도 |
|---|------|--------|------|--------|
| 1 | se | Davvisámegiella (Northern Sami) | 라틴 | ⭐⭐ |
| 2 | smn | Anarâškielâ (Inari Sami) | 라틴 | ⭐⭐ |
| 3 | sms | Nuõrttsääʹmǩiõll (Skolt Sami) | 라틴 | ⭐⭐ |
| 4 | sma | Åarjelsaemien (Southern Sami) | 라틴 | ⭐⭐ |
| 5 | smj | Julevsámegiella (Lule Sami) | 라틴 | ⭐⭐ |
| 6 | sjd | Кӣллт са̄мь (Kildin Sami) | 키릴 | ⭐⭐ |
| 7 | krl | Karjala (Karelian) | 라틴/키릴 | ⭐⭐ |
| 8 | olo | Livvi (Livvi-Karelian) | 라틴 | ⭐ |
| 9 | vep | Vepsän kel' (Veps) | 라틴 | ⭐ |
| 10 | liv | Līvõ kēļ (Livonian) | 라틴 | ⭐⭐ |
| 11 | myv | Эрзянь кель (Erzya) | 키릴 | ⭐⭐ |
| 12 | mdf | Мокшень кяль (Moksha) | 키릴 | ⭐⭐ |
| 13 | mhr | Олык марий (Meadow Mari) | 키릴 | ⭐⭐ |
| 14 | mrj | Кырык мары (Hill Mari) | 키릴 | ⭐⭐ |
| 15 | udm | Удмурт кыл (Udmurt) | 키릴 | ⭐⭐ |
| 16 | koi | Коми-пермяк (Komi-Permyak) | 키릴 | ⭐⭐ |
| 17 | kpv | Коми кыв (Komi-Zyrian) | 키릴 | ⭐⭐ |
| 18 | mns | Маньси (Mansi) | 키릴 | ⭐⭐ |
| 19 | kca | Ханты (Khanty) | 키릴 | ⭐⭐ |
| 20 | nio | Нганасан (Nganasan) | 키릴 | ⭐⭐ |

---

## 🔵 Batch 11 (20개) - 튀르크어 + 몽골어
| # | 코드 | 언어명 | 문자 | 난이도 |
|---|------|--------|------|--------|
| 1 | tk | Türkmen (Turkmen) | 라틴 | ⭐ |
| 2 | tt | Татар (Tatar) | 키릴/라틴 | ⭐⭐ |
| 3 | ba | Башҡорт (Bashkir) | 키릴 | ⭐⭐ |
| 4 | cv | Чӑваш (Chuvash) | 키릴 | ⭐⭐ |
| 5 | sah | Саха тыла (Yakut/Sakha) | 키릴 | ⭐⭐ |
| 6 | tyv | Тыва дыл (Tuvan) | 키릴 | ⭐⭐ |
| 7 | alt | Алтай тил (Altai) | 키릴 | ⭐⭐ |
| 8 | kjh | Хакас тілі (Khakas) | 키릴 | ⭐⭐ |
| 9 | cjs | Шор тили (Shor) | 키릴 | ⭐⭐ |
| 10 | kim | Тофа дыл (Tofa) | 키릴 | ⭐⭐ |
| 11 | dlg | Долган тыла (Dolgan) | 키릴 | ⭐⭐ |
| 12 | gag | Gagauz (Gagauz) | 라틴 | ⭐ |
| 13 | crh | Qırımtatar (Crimean Tatar) | 라틴/키릴 | ⭐⭐ |
| 14 | nog | Ногай (Nogai) | 키릴 | ⭐⭐ |
| 15 | kum | Къумукъ (Kumyk) | 키릴 | ⭐⭐ |
| 16 | krc | Къарачай-малкъар (Karachay-Balkar) | 키릴 | ⭐⭐ |
| 17 | xal | Хальмг келн (Kalmyk) | 키릴 | ⭐⭐ |
| 18 | bua | Буряад хэлэн (Buryat) | 키릴 | ⭐⭐ |
| 19 | xwo | Written Oirat (Clear Script) | 토드 문자 | ⭐⭐⭐⭐ |
| 20 | cmg | Classical Mongolian | 몽골 문자 | ⭐⭐⭐⭐ |

---

## 🔵 Batch 12 (20개) - 캅카스어
| # | 코드 | 언어명 | 문자 | 난이도 |
|---|------|--------|------|--------|
| 1 | ce | Нохчийн мотт (Chechen) | 키릴 | ⭐⭐ |
| 2 | inh | ГӀалгӀай мотт (Ingush) | 키릴 | ⭐⭐ |
| 3 | av | Авар мацӏ (Avar) | 키릴 | ⭐⭐ |
| 4 | lez | Лезги чӏал (Lezgian) | 키릴 | ⭐⭐ |
| 5 | dar | Дарган мез (Dargwa) | 키릴 | ⭐⭐ |
| 6 | lbe | Лакку маз (Lak) | 키릴 | ⭐⭐ |
| 7 | tab | Табасаран чӏал (Tabasaran) | 키릴 | ⭐⭐ |
| 8 | aqc | Арчиб (Archi) | 키릴 | ⭐⭐⭐ |
| 9 | tkr | Цахур (Tsakhur) | 키릴 | ⭐⭐ |
| 10 | rut | Рутул (Rutul) | 키릴 | ⭐⭐ |
| 11 | agx | Агъул (Aghul) | 키릴 | ⭐⭐ |
| 12 | udi | Удин (Udi) | 라틴/키릴 | ⭐⭐ |
| 13 | kbd | Адыгэбзэ (Kabardian) | 키릴 | ⭐⭐ |
| 14 | ady | Адыгабзэ (Adyghe) | 키릴 | ⭐⭐ |
| 15 | abq | Абаза (Abaza) | 키릴 | ⭐⭐ |
| 16 | ab | Аҧсуа (Abkhaz) | 키릴 | ⭐⭐⭐ |
| 17 | os | Ирон æвзаг (Ossetic) | 키릴 | ⭐⭐ |
| 18 | xmf | მარგალური (Mingrelian) | 조지아 문자 | ⭐⭐⭐ |
| 19 | sva | ლუშნუ ნინ (Svan) | 조지아 문자 | ⭐⭐⭐ |
| 20 | lzz | ლაზური (Laz) | 조지아/라틴 | ⭐⭐⭐ |

---

## 🔵 Batch 13 (20개) - 중동 + 아랍 방언
| # | 코드 | 언어명 | 문자 | 난이도 |
|---|------|--------|------|--------|
| 1 | ar-EG | العربية المصرية (Egyptian Arabic) | 아랍 | ⭐⭐ |
| 2 | ar-MA | الدارجة (Moroccan Arabic) | 아랍 | ⭐⭐ |
| 3 | ar-LB | اللبنانية (Lebanese Arabic) | 아랍 | ⭐⭐ |
| 4 | ar-IQ | العراقية (Iraqi Arabic) | 아랍 | ⭐⭐ |
| 5 | ar-SA | الخليجية (Gulf Arabic) | 아랍 | ⭐⭐ |
| 6 | ar-YE | اليمنية (Yemeni Arabic) | 아랍 | ⭐⭐ |
| 7 | ar-SD | السودانية (Sudanese Arabic) | 아랍 | ⭐⭐ |
| 8 | apc | الشامي (Levantine Arabic) | 아랍 | ⭐⭐ |
| 9 | acm | عراقي (Mesopotamian Arabic) | 아랍 | ⭐⭐ |
| 10 | ary | الدارجة المغربية (Moroccan) | 아랍 | ⭐⭐ |
| 11 | arz | مصرى (Egyptian) | 아랍 | ⭐⭐ |
| 12 | aeb | تونسي (Tunisian) | 아랍 | ⭐⭐ |
| 13 | arq | الدارجة الجزائرية (Algerian) | 아랍 | ⭐⭐ |
| 14 | ku | Kurdî (Kurdish - Kurmanji) | 라틴 | ⭐ |
| 15 | ckb | سۆرانی (Central Kurdish/Sorani) | 아랍 | ⭐⭐ |
| 16 | sdh | کوردی خوارین (Southern Kurdish) | 아랍 | ⭐⭐ |
| 17 | ps | پښتو (Pashto) | 아랍 | ⭐⭐⭐ |
| 18 | bal | بلوچی (Balochi) | 아랍 | ⭐⭐ |
| 19 | haz | آزرگی (Hazaragi) | 아랍 | ⭐⭐ |
| 20 | prs | دری (Dari) | 아랍 | ⭐⭐ |

---

## 🔵 Batch 14 (20개) - 인도 추가
| # | 코드 | 언어명 | 문자 | 난이도 |
|---|------|--------|------|--------|
| 1 | as | অসমীয়া (Assamese) | 벵골 문자 | ⭐⭐ |
| 2 | or | ଓଡ଼ିଆ (Odia) | 오디아 문자 | ⭐⭐⭐ |
| 3 | mai | मैथिली (Maithili) | 데바나가리 | ⭐⭐ |
| 4 | sat | ᱥᱟᱱᱛᱟᱲᱤ (Santali) | 올 치키 문자 | ⭐⭐⭐ |
| 5 | ks | کٲشُر (Kashmiri) | 아랍/데바나가리 | ⭐⭐⭐ |
| 6 | sd | سنڌي (Sindhi) | 아랍 문자 | ⭐⭐ |
| 7 | doi | डोगरी (Dogri) | 데바나가리 | ⭐⭐ |
| 8 | kok | कोंकणी (Konkani) | 데바나가리 | ⭐⭐ |
| 9 | mni | মৈতৈলোন্ (Manipuri/Meitei) | 메이테이 문자 | ⭐⭐⭐ |
| 10 | bo | བོད་སྐད་ (Tibetan) | 티베트 문자 | ⭐⭐⭐⭐ |
| 11 | dz | རྫོང་ཁ (Dzongkha) | 티베트 문자 | ⭐⭐⭐⭐ |
| 12 | new | नेपाल भाषा (Newari) | 데바나가리 | ⭐⭐ |
| 13 | bho | भोजपुरी (Bhojpuri) | 데바나가리 | ⭐⭐ |
| 14 | hne | छत्तीसगढ़ी (Chhattisgarhi) | 데바나가리 | ⭐⭐ |
| 15 | raj | राजस्थानी (Rajasthani) | 데바나가리 | ⭐⭐ |
| 16 | gbm | गढ़वाली (Garhwali) | 데바나가리 | ⭐⭐ |
| 17 | kfy | कुमाऊँनी (Kumaoni) | 데바나가리 | ⭐⭐ |
| 18 | brx | बर' (Bodo) | 데바나가리 | ⭐⭐ |
| 19 | lus | Mizo ṭawng (Mizo) | 라틴 | ⭐ |
| 20 | kha | Ka Ktien Khasi (Khasi) | 라틴 | ⭐ |

---

## 🔵 Batch 15 (20개) - 동남아 + 필리핀
| # | 코드 | 언어명 | 문자 | 난이도 |
|---|------|--------|------|--------|
| 1 | ceb | Cebuano | 라틴 | ⭐ |
| 2 | ilo | Ilocano | 라틴 | ⭐ |
| 3 | hil | Hiligaynon | 라틴 | ⭐ |
| 4 | war | Winaray (Waray) | 라틴 | ⭐ |
| 5 | pag | Pangasinan | 라틴 | ⭐ |
| 6 | bcl | Bikol | 라틴 | ⭐ |
| 7 | pam | Kapampangan | 라틴 | ⭐ |
| 8 | tsg | Tausug | 라틴 | ⭐ |
| 9 | mdh | Maguindanaon | 라틴 | ⭐ |
| 10 | mrw | Maranao | 라틴 | ⭐ |
| 11 | jv | ꦧꦱꦗꦮ (Javanese) | 자바 문자/라틴 | ⭐⭐⭐ |
| 12 | su | ᮘᮞ ᮞᮥᮔ᮪ᮓ (Sundanese) | 순다 문자/라틴 | ⭐⭐⭐ |
| 13 | min | Minangkabau | 라틴 | ⭐ |
| 14 | ace | Acèh (Acehnese) | 라틴 | ⭐ |
| 15 | ban | ᬅᬓ᭄ᬱᬭᬩᬮᬶ (Balinese) | 발리 문자 | ⭐⭐⭐⭐ |
| 16 | bew | Betawi | 라틴 | ⭐ |
| 17 | mad | Madhurâ (Madurese) | 라틴 | ⭐ |
| 18 | bjn | Banjar | 라틴 | ⭐ |
| 19 | bug | ᨅᨔ ᨕᨘᨁᨗ (Buginese) | 룬타라 문자 | ⭐⭐⭐ |
| 20 | mak | Makassar | 라틴 | ⭐ |

---

## 🔵 Batch 16 (20개) - 태평양 + 아메리카 원주민
| # | 코드 | 언어명 | 문자 | 난이도 |
|---|------|--------|------|--------|
| 1 | mi | Te Reo Māori | 라틴 | ⭐ |
| 2 | haw | ʻŌlelo Hawaiʻi | 라틴 | ⭐ |
| 3 | sm | Gagana Samoa | 라틴 | ⭐ |
| 4 | to | Lea faka-Tonga | 라틴 | ⭐ |
| 5 | fj | Na Vosa Vakaviti (Fijian) | 라틴 | ⭐ |
| 6 | ty | Reo Tahiti | 라틴 | ⭐ |
| 7 | rap | Vananga Rapanui (Easter Island) | 라틴 | ⭐ |
| 8 | qu | Runasimi (Quechua) | 라틴 | ⭐ |
| 9 | ay | Aymar aru (Aymara) | 라틴 | ⭐ |
| 10 | gn | Avañe'ẽ (Guarani) | 라틴 | ⭐ |
| 11 | nah | Nāhuatl | 라틴 | ⭐ |
| 12 | yua | Màaya t'àan (Yucatec Maya) | 라틴 | ⭐ |
| 13 | iu | ᐃᓄᒃᑎᑐᑦ (Inuktitut) | 캐나다 음절문자 | ⭐⭐⭐⭐ |
| 14 | cr | ᓀᐦᐃᔭᐍᐏᐣ (Cree) | 캐나다 음절문자 | ⭐⭐⭐⭐ |
| 15 | chr | ᏣᎳᎩ (Cherokee) | 체로키 음절문자 | ⭐⭐⭐⭐ |
| 16 | nv | Diné bizaad (Navajo) | 라틴 | ⭐⭐ |
| 17 | oj | Anishinaabemowin (Ojibwe) | 라틴/음절문자 | ⭐⭐ |
| 18 | cic | Chahta (Choctaw) | 라틴 | ⭐ |
| 19 | mus | Mvskoke (Creek/Muskogee) | 라틴 | ⭐ |
| 20 | lkt | Lakȟótiyapi (Lakota) | 라틴 | ⭐⭐ |

---

## 🔵 Batch 17 (20개) - 아프리카 추가
| # | 코드 | 언어명 | 문자 | 난이도 |
|---|------|--------|------|--------|
| 1 | rn | Ikirundi (Kirundi) | 라틴 | ⭐ |
| 2 | lg | Luganda | 라틴 | ⭐ |
| 3 | nyn | Runyankore | 라틴 | ⭐ |
| 4 | luo | Dholuo | 라틴 | ⭐ |
| 5 | kam | Kikamba | 라틴 | ⭐ |
| 6 | ki | Gĩkũyũ | 라틴 | ⭐ |
| 7 | mer | Kĩmĩrũ | 라틴 | ⭐ |
| 8 | bem | Ichibemba | 라틴 | ⭐ |
| 9 | loz | Silozi | 라틴 | ⭐ |
| 10 | toi | Chitonga | 라틴 | ⭐ |
| 11 | tw | Twi (Akan) | 라틴 | ⭐ |
| 12 | ee | Eʋegbe (Ewe) | 라틴 | ⭐ |
| 13 | ak | Akan | 라틴 | ⭐ |
| 14 | gaa | Ga | 라틴 | ⭐ |
| 15 | ber | ⵜⴰⵎⴰⵣⵉⵖⵜ (Tamazight) | 티피나그 | ⭐⭐⭐ |
| 16 | kab | Taqbaylit (Kabyle) | 티피나그/라틴 | ⭐⭐ |
| 17 | shi | ⵜⴰⵛⵍⵃⵉⵜ (Tashelhit) | 티피나그 | ⭐⭐⭐ |
| 18 | tzm | ⵜⴰⵎⴰⵣⵉⵖⵜ (Central Atlas Tamazight) | 티피나그 | ⭐⭐⭐ |
| 19 | rif | Tarifit (Riffian) | 티피나그/라틴 | ⭐⭐ |
| 20 | zgh | ⵜⴰⵎⴰⵣⵉⵖⵜ (Standard Moroccan Tamazight) | 티피나그 | ⭐⭐⭐ |

---

## 🔵 Batch 18 (20개) - 서아프리카 + 나이지리아
| # | 코드 | 언어명 | 문자 | 난이도 |
|---|------|--------|------|--------|
| 1 | bm | Bamanankan (Bambara) | 라틴/응코 | ⭐⭐ |
| 2 | dyu | Julakan (Dyula) | 라틴 | ⭐ |
| 3 | mnk | Mandinka | 라틴 | ⭐ |
| 4 | snk | Soninke | 라틴 | ⭐ |
| 5 | mos | Mòoré (Mossi) | 라틴 | ⭐ |
| 6 | kbp | Kabiyè | 라틴 | ⭐ |
| 7 | fon | Fɔ̀ngbè (Fon) | 라틴 | ⭐ |
| 8 | pcm | Naijá (Nigerian Pidgin) | 라틴 | ⭐ |
| 9 | fuv | Fulfulde (Nigerian Fulani) | 라틴 | ⭐ |
| 10 | bin | Ẹ̀dó (Edo/Bini) | 라틴 | ⭐ |
| 11 | ibb | Ibibio | 라틴 | ⭐ |
| 12 | efi | Efik | 라틴 | ⭐ |
| 13 | tiv | Tiv | 라틴 | ⭐ |
| 14 | idu | Idoma | 라틴 | ⭐ |
| 15 | nup | Nupe | 라틴 | ⭐ |
| 16 | urh | Urhobo | 라틴 | ⭐ |
| 17 | ijs | Ijaw (Izon) | 라틴 | ⭐ |
| 18 | ann | Obolo | 라틴 | ⭐ |
| 19 | kri | Krio (Sierra Leone) | 라틴 | ⭐ |
| 20 | tem | Temne | 라틴 | ⭐ |

---

## 🔵 Batch 19 (20개) - 크리올 + 피진
| # | 코드 | 언어명 | 문자 | 난이도 |
|---|------|--------|------|--------|
| 1 | ht | Kreyòl ayisyen (Haitian Creole) | 라틴 | ⭐ |
| 2 | pap | Papiamentu | 라틴 | ⭐ |
| 3 | srn | Sranantongo | 라틴 | ⭐ |
| 4 | gcr | Kriyòl gwiyanè | 라틴 | ⭐ |
| 5 | lou | Kréyol la Lwizyàn | 라틴 | ⭐ |
| 6 | jam | Patwa (Jamaican) | 라틴 | ⭐ |
| 7 | tpi | Tok Pisin | 라틴 | ⭐ |
| 8 | bi | Bislama | 라틴 | ⭐ |
| 9 | crs | Seselwa (Seychellois) | 라틴 | ⭐ |
| 10 | mfe | Kreol Morisien | 라틴 | ⭐ |
| 11 | rcf | Réunion Creole | 라틴 | ⭐ |
| 12 | cpf | Rodriguan Creole | 라틴 | ⭐ |
| 13 | dcr | Negerhollands (Virgin Islands Creole) | 라틴 | ⭐ |
| 14 | skw | Saramaccan | 라틴 | ⭐ |
| 15 | djk | Aukan (Ndyuka) | 라틴 | ⭐ |
| 16 | bzj | Belize Kriol | 라틴 | ⭐ |
| 17 | icr | Islander Creole (San Andrés) | 라틴 | ⭐ |
| 18 | vic | Virgin Islands Creole English | 라틴 | ⭐ |
| 19 | aig | Antiguan Creole | 라틴 | ⭐ |
| 20 | bah | Bahamian Creole | 라틴 | ⭐ |

---

## 🔵 Batch 20 (20개) - 인공어 + 역사어
| # | 코드 | 언어명 | 문자 | 난이도 |
|---|------|--------|------|--------|
| 1 | eo | Esperanto | 라틴 | ⭐ |
| 2 | ia | Interlingua | 라틴 | ⭐ |
| 3 | vo | Volapük | 라틴 | ⭐ |
| 4 | io | Ido | 라틴 | ⭐ |
| 5 | jbo | la .lojban. | 라틴 | ⭐⭐ |
| 6 | nov | Novial | 라틴 | ⭐ |
| 7 | lfn | Lingua Franca Nova | 라틴 | ⭐ |
| 8 | la | Latina (Latin) | 라틴 | ⭐ |
| 9 | grc | Ἑλληνική (Ancient Greek) | 그리스 | ⭐⭐⭐ |
| 10 | sa | संस्कृतम् (Sanskrit) | 데바나가리 | ⭐⭐⭐ |
| 11 | pi | पालि (Pali) | 데바나가리/타이 | ⭐⭐⭐ |
| 12 | ae | 𐬀𐬎𐬭𐬀𐬨𐬀𐬰𐬛𐬁 (Avestan) | 아베스타 문자 | ⭐⭐⭐⭐ |
| 13 | peo | 𐎠𐎼𐎹 (Old Persian) | 설형문자 | ⭐⭐⭐⭐ |
| 14 | got | 𐌲𐌿𐍄𐌹𐍃𐌺 (Gothic) | 고딕 문자 | ⭐⭐⭐⭐ |
| 15 | non | Norrœnt (Old Norse) | 룬 문자/라틴 | ⭐⭐⭐ |
| 16 | ang | Englisc (Old English) | 룬 문자/라틴 | ⭐⭐ |
| 17 | sga | Goídelc (Old Irish) | 오감 문자/라틴 | ⭐⭐⭐ |
| 18 | egy | Hieroglyphic Egyptian | 상형문자 | ⭐⭐⭐⭐⭐ |
| 19 | akk | 𒀀𒀝𒅗𒁺𒌑 (Akkadian) | 설형문자 | ⭐⭐⭐⭐⭐ |
| 20 | sux | 𒅴𒂠 (Sumerian) | 설형문자 | ⭐⭐⭐⭐⭐ |

---

## 🔵 Batch 21 (13개) - 수어 + 기타
| # | 코드 | 언어명 | 문자 | 난이도 |
|---|------|--------|------|--------|
| 1 | ase | American Sign Language | SignWriting | ⭐⭐⭐⭐ |
| 2 | bfi | British Sign Language | SignWriting | ⭐⭐⭐⭐ |
| 3 | fsl | Langue des Signes Française | SignWriting | ⭐⭐⭐⭐ |
| 4 | gsg | Deutsche Gebärdensprache | SignWriting | ⭐⭐⭐⭐ |
| 5 | kvk | 한국 수어 (Korean Sign Language) | SignWriting | ⭐⭐⭐⭐ |
| 6 | jsl | 日本手話 (Japanese Sign Language) | SignWriting | ⭐⭐⭐⭐ |
| 7 | csl | 中国手语 (Chinese Sign Language) | SignWriting | ⭐⭐⭐⭐ |
| 8 | ins | International Sign | SignWriting | ⭐⭐⭐⭐ |
| 9 | psp | Philippine Sign Language | SignWriting | ⭐⭐⭐⭐ |
| 10 | bvl | Bolivian Sign Language | SignWriting | ⭐⭐⭐⭐ |
| 11 | bzs | Brazilian Sign Language (Libras) | SignWriting | ⭐⭐⭐⭐ |
| 12 | mfs | Mexican Sign Language | SignWriting | ⭐⭐⭐⭐ |
| 13 | asf | Australian Sign Language (Auslan) | SignWriting | ⭐⭐⭐⭐ |

---

# 🛠️ 작업 노하우 (경험자 권고사항)

## 1. 필수 포함 항목 체크리스트

```
각 언어 문서에 반드시 포함:

□ 언어 개요 (어족, 문자, 화자 수, 지역, 음운 특징)
□ 자음 체계 테이블
  - 필수 컬럼: 문자 | 로마자 | IPA | WIA Braille | Unicode | 조음 위치 | 조음 방법 | 예시
□ 모음 체계 테이블
  - 필수 컬럼: 문자 | 로마자 | IPA | WIA Braille | Unicode | 예시
□ 특수 음운 (성조, 클릭, 방출음 등 해당 시)
□ 기본 인사 5개 (원문 + 로마자 + IPA + WIA Braille + 의미)
□ 숫자 1-10 (원문 + WIA Braille)
□ 기본 문장 5개
□ 예시 단어 각 음소당 1개 이상
```

---

## 2. Unicode 코드포인트 필수화

```markdown
⚠️ 경험에서 얻은 교훈:
Unicode 없이 작업하면 나중에 2배 시간이 듭니다!

반드시 처음부터 포함:
| ก | k | /k/ | ⠅ | U+0E01 | 연구개 | 무성 파열음 |
| あ | a | /a/ | ⠁ | U+3042 | - | 모음 |
| ᄀ | g | /k~g/ | ⠛ | U+1100 | 연구개 | 파열음 |
```

---

## 3. 난이도별 배치 전략

```markdown
### ⭐ 쉬움 (라틴 문자): 배치당 20개 가능
- 서유럽어, 크리올어, 필리핀어군, 태평양어, 반투어

### ⭐⭐ 보통 (키릴/확장 라틴): 배치당 15개
- 슬라브어, 튀르크어, 핀우그릭어

### ⭐⭐⭐ 어려움 (고유 문자): 배치당 8-10개
- 아랍 문자, 인도계 문자, 그으즈 문자, 조지아 문자, 티피나그

### ⭐⭐⭐⭐ 매우 어려움: 배치당 5개 이하
- 티베트 문자, 자바/발리 문자, 음절문자 (이누크티툿, 체로키, 크리)
- 클릭 자음 언어, 수어

### ⭐⭐⭐⭐⭐ 극한 난이도: 배치당 2-3개
- 설형문자 (수메르, 아카드, 고대 페르시아)
- 상형문자 (이집트)
```

---

## 4. 특수 언어 처리 가이드

### 4.1 클릭 자음 언어 (zu, xh, ss, nr 등)
```
클릭 자음 3종류 × 3변이 = 9개 이상:
- 치간 클릭 /ǀ/ (c): 무성, 유성(gc), 비음(nc)
- 경구개 클릭 /ǃ/ (q): 무성, 유성(gq), 비음(nq)
- 측면 클릭 /ǁ/ (x): 무성, 유성(gx), 비음(nx)
```

### 4.2 성조 언어 (zh, th, vi, yo 등)
```
성조 표기법 통일:
- 숫자: ma1 ma2 ma3 ma4 ma5
- 기호: mā má mǎ mà ma
- WIA Braille 성조 부호 매핑 필수
```

### 4.3 아브기다 (인도계, 그으즈)
```
자음+모음 결합 체계:
- 기본 자음 형태 (inherent vowel)
- 모음 부호 (마트라) 결합 규칙
- 결합 예시 테이블 포함
```

### 4.4 수어
```
SignWriting 또는 Stokoe 표기법:
- 손 모양 (handshape)
- 위치 (location)
- 움직임 (movement)
- 손바닥 방향 (palm orientation)
- 비수지 신호 (non-manual signals)
```

---

## 5. 작업 효율 팁

### 5.1 중간 저장 필수!
```bash
# 매 5개 언어마다 확인
wc -l batch-XX-braille-detailed.md
grep -c "U+[0-9A-F]" batch-XX-braille-detailed.md

# 백업 생성
cp batch-XX.md batch-XX.md.bak
```

### 5.2 sed 명령 주의
```
⚠️ 경험에서 얻은 교훈:
sed로 대규모 수정 시 내용이 날아갈 수 있음!

1. 반드시 백업 먼저
2. 범위 지정 정확히: /시작패턴/,/끝패턴/
3. 특수문자 이스케이프: / → \/, ' → '\''
4. 수정 후 라인 수 확인
```

### 5.3 유사 언어 그룹핑
```
같이 작업하면 효율적:
- 슬라브어: ru, uk, be, bg, sr, hr, mk, cs, sk, pl, sl
- 로망스어: es, fr, it, pt, ro, ca, gl, oc
- 게르만어: de, nl, sv, da, no, is, gsw, yi
- 드라비다어: ta, te, kn, ml
- 반투어: sw, zu, xh, rw, ny, sn, st, tn, lg, ki
- 튀르크어: tr, az, uz, kk, ky, tk, tt, ba
```

---

## 6. 참고 자료

### IPA
- https://www.internationalphoneticassociation.org/content/ipa-chart
- https://en.wikipedia.org/wiki/International_Phonetic_Alphabet

### Unicode
- https://unicode.org/charts/
- https://www.compart.com/en/unicode/

### 언어별 정보
- Ethnologue: https://www.ethnologue.com/
- Omniglot: https://www.omniglot.com/
- WALS: https://wals.info/
- Glottolog: https://glottolog.org/

---

# 📅 권장 작업 일정

| 배치 | 언어 수 | 평균 난이도 | 예상 시간 |
|------|--------|------------|----------|
| Batch 7 | 16개 (실제) | ⭐ | 1.5시간 |
| Batch 8 | 20개 | ⭐ | 1.5시간 |
| Batch 9 | 20개 | ⭐⭐ | 2시간 |
| Batch 10 | 20개 | ⭐⭐ | 2시간 |
| Batch 11 | 20개 | ⭐⭐ | 2시간 |
| Batch 12 | 20개 | ⭐⭐ | 2시간 |
| Batch 13 | 20개 | ⭐⭐ | 2시간 |
| Batch 14 | 20개 | ⭐⭐⭐ | 3시간 |
| Batch 15 | 20개 | ⭐⭐ | 2시간 |
| Batch 16 | 20개 | ⭐⭐⭐ | 3시간 |
| Batch 17 | 20개 | ⭐⭐ | 2시간 |
| Batch 18 | 20개 | ⭐ | 1.5시간 |
| Batch 19 | 20개 | ⭐ | 1.5시간 |
| Batch 20 | 20개 | ⭐⭐⭐⭐ | 4시간 |
| Batch 21 | 13개 | ⭐⭐⭐⭐ | 3시간 |

**총 예상**: 약 32-35시간 (15개 배치, 293개 언어 항목)
**실제 신규 언어**: 약 113개 (기존 98개 + 113개 = 211개)

---

# ✅ 다음 채팅 시작 프롬프트 예시

```
WIA Braille 프로젝트 계속합니다.

참조 파일: /var/www/wia.live/docs/BRAILLE_LANGUAGES/WIA-BRAILLE-REMAINING-GUIDE.md

오늘 작업: 원래 프롬프트 Batch 7 (16개 언어)
- ta-SL, ca, eu, gl, cy, ga, mt, is, fo, lb, fy, gd, br, co, oc, sc

필수 포함:
- Unicode 코드포인트 (U+XXXX)
- 조음 위치/방법
- 기본 인사 5개, 숫자 1-10, 기본 문장 5개

파일명: batch-09-braille-detailed.md (또는 original-batch-07)
저장 위치: /var/www/wia.live/docs/BRAILLE_LANGUAGES/
```

---

© 2024 WIA (World Certification Industry Association)
**홍익인간 (弘益人間)** - 널리 인간을 이롭게 하라
