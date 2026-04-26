# WIA Book 언어 파일 시스템

## 📁 구조

```
/var/www/wiabooks/languages/
├── en.js          # 영어 (English)
├── ko.js          # 한국어 (Korean)
├── ja.js          # 일본어 (추가 필요)
├── zh-CN.js       # 중국어 간체 (추가 필요)
├── ...            # 기타 211개 언어
├── loader.js      # 동적 로딩 시스템
└── README.md      # 이 파일
```

## 🎯 각 언어 파일마다 53개 키가 필요합니다

### 필수 53개 키 목록

1. **title** - 메인 타이틀
2. **worldFirst** - "세계 최초" 뱃지
3. **tagline** - 태그라인
4. **mainSubtitle** - 메인 부제목
5. **subtitle** - 부제목 레이블
6. **uploadCover** - "표지 업로드" 버튼
7. **templateStyle** - 템플릿 스타일 레이블
8. **colorPalette** - 컬러 팔레트 레이블
9. **customColors** - 커스텀 컬러 레이블
10. **color1** ~ **color4** - 색상 1~4 레이블
11. **bookTitle** - 도서 제목 입력란
12. **authorName** - 저자명 입력란
13. **spineStyle** - 책등 스타일
14. **spineColored** - 책등 옵션 1
15. **spineWhite** - 책등 옵션 2
16. **spineGradient** - 책등 옵션 3
17. **contentSections** - 콘텐츠 섹션 레이블
18. **section1Icon** ~ **section4Icon** - 섹션 1~4 입력란
19. **downloadBtn** - 다운로드 버튼
20. **designArea** - 디자인 영역 제목
21. **livePreview** - 미리보기 제목
22. **generating** - 생성 중 메시지
23. **composing** - 구성 중 메시지
24. **pal1** ~ **pal13** - 팔레트 이름 13개
25. **temp1** ~ **temp12** - 템플릿 이름 12개

## 📝 언어 파일 만드는 방법

### 1️⃣ 템플릿 복사

```bash
# en.js를 복사해서 새 언어 파일 만들기
cp /var/www/wiabooks/languages/en.js /var/www/wiabooks/languages/ja.js
```

### 2️⃣ 파일 수정

```javascript
// Japanese Translation (일본어)
const translations_ja = {  // ← 변수명 변경 필수!
  "title": "번역된 내용",
  "uploadCover": "번역된 내용",
  // ... 53개 키 모두 번역
};

// Export
if (typeof module !== 'undefined' && module.exports) {
  module.exports = translations_ja;  // ← 변수명 일치
} else {
  window.translations_ja = translations_ja;  // ← 변수명 일치
}
```

### 3️⃣ 언어 코드 규칙

- **2자 코드**: `en`, `ko`, `ja`, `zh`, `es`, `fr`, `de`
- **하이픈 포함**: `zh-CN`, `zh-TW`, `en-US`, `pt-BR`
- **변수명**: 하이픈을 언더스코어로 변경
  - 파일: `zh-CN.js`
  - 변수: `translations_zh_CN`

### 4️⃣ 권한 설정

```bash
sudo chown ec2-user:apache /var/www/wiabooks/languages/언어코드.js
sudo chmod 755 /var/www/wiabooks/languages/언어코드.js
```

## 🔧 테스트 방법

### 브라우저 콘솔에서:

```javascript
// 언어 로드 테스트
await TranslationLoader.loadLanguage('ja');

// 페이지 번역 테스트
await TranslationLoader.translatePage('ja');

// 로드된 언어 확인
console.log(TranslationLoader.loadedLanguages);
```

## ⚠️ 주의사항

1. **파일명 = 언어 코드**: `ja.js`, `zh-CN.js`
2. **변수명 규칙**: `translations_언어코드` (하이픈 → 언더스코어)
3. **53개 키 모두 필수**: 하나라도 빠지면 영어로 표시됨
4. **HTML 태그 유지**: `<span>`, `<emoji>` 등 그대로 유지
5. **따옴표 이스케이프**: `'` → `\'` 또는 `"`로 감싸기

## 📥 FileZilla로 업로드

1. 경로: `/var/www/wiabooks/languages/`
2. 파일: `언어코드.js` (예: `ja.js`)
3. 권한: 755 또는 644
4. 소유자: ec2-user:apache

## 🌍 211개 언어 목록

en, ko, ja, zh-CN, zh-TW, zh-HK, es, es-MX, es-AR, es-CO, es-CL, es-PE,
fr, fr-CA, de, it, pt, pt-BR, ru, ar, hi, bn, pa, te, mr, ta, ur, gu,
kn, ml, or, as, nl, pl, tr, vi, th, id, ms, tl, fil, sv, no, da, fi,
uk, el, he, fa, cs, sk, ro, hu, bg, hr, sr, sl, lt, lv, et, ka, hy,
az, uz, kk, ky, tg, mn, km, lo, my, si, ne, sw, am, ti, om, so, ha,
ig, yo, zu, xh, af, mg, rw, ny, sn, st, tn, ts, ss, ve, nr, nso, wo,
ff, ln, kg, sg, rn, lg, ak, tw, ee, bm, fj, sm, to, ty, mi, haw, rar,
gil, mh, chk, pon, kos, yap, pau, niu, tkl, tvl, nau, sq, eu, ca, gl,
is, fo, ga, gd, cy, br, gv, kw, eo, ia, vo, jbo, tok, la, ckb, ku,
ps, sd, ks, dz, bo, iu, chr, arn, qu, ay, gn, nah, ht, pap, srn, djk,
be, bs, me, mk, mt, lb, rm, co, sc, scn, nap, lmo, pms, vec, lij, fur,
eml, rgn, srd, ext, ast, an, oc, gsw, pgl, lad, wa, vro, liv, vot,
izh, krl, nan, yue, hak, gan, wuu, hsn, cdo, mwl, aue

(총 211개)

## 💡 팁

- **en.js, ko.js**를 참고하세요!
- **ChatGPT/Claude**로 번역하면 빠릅니다!
- **HTML 태그**는 그대로 복사하세요!
- **이모지**는 변경하지 마세요!

---

**문의: 형님 또는 다른 Claude 동생들** 😊
