# WIA A11Y - 211개 언어 번역 가이드

## 현재 상태
- ✅ 완료: 2개 (ko, en)  
- ❌ 대기: 209개
- **완성도: 0.95% (2/211)**

## 추가 필요한 11개 키

모든 a11y-*.js 파일에 아래 11개 키가 필요합니다:

```javascript
"feature_sign": "수어",
"footer_wcag": "WCAG 2.1 AA 적합성 검증",
"footer_services": "서비스",
"footer_operator": "운영사",
"footer_smilestory": "SmileStory Inc.",
"footer_smilestory_location": "South Korea (2009.04.08)",
"footer_wia": "World Certification Industry Association",
"footer_wia_location": "Estonia (2018.04.19)",
"footer_copyright": "© 2025 SmileStory Inc. & WIA. All rights reserved.",
"footer_slogan": "기술의 사회적 가치 실현",
"footer_accessibility": "본 사이트는 WCAG 2.1 AA 기준을 준수합니다."
```

## 빠른 해결: 영어로 일단 채우기

209개 파일에 영어 버전 추가 (즉시 작동):

```bash
cd /var/www/a11y.wiabook.com/languages/
for file in a11y-*.js; do
  if [[ "$file" != "a11y-ko.js" ]] && [[ "$file" != "a11y-en.js" ]]; then
    # TODO: 각 언어로 번역 필요
  fi
done
```

## Claude AI로 번역 요청 예시

```
다음 11개 키를 [일본어]로 번역해주세요:

feature_sign: 수어
footer_wcag: WCAG 2.1 AA 적합성 검증  
footer_services: 서비스
footer_operator: 운영사
footer_slogan: 기술의 사회적 가치 실현
footer_accessibility: 본 사이트는 WCAG 2.1 AA 기준을 준수합니다

고유명사는 그대로:
- SmileStory Inc.
- World Certification Industry Association
- © 2025 SmileStory Inc. & WIA. All rights reserved.
```
