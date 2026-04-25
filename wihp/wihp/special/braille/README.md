# WIHP-Braille Integration

## 개요

WIHP(WIA International Hangul Phonology)와 WIA-Braille 프로젝트의 통합 매핑입니다.
IPA → 한글 → 점자의 삼중 연결을 제공합니다.

## 통합 체계

```
음성 언어 → IPA → WIHP 한글 → WIA-Braille 점자
English "hello" → /həˈloʊ/ → 헬로 → ⠚⠢⠇⠕
```

## 연결 프로젝트

- **WIHP**: IPA → Hangul 음성학 매핑
- **WIA-Braille**: Hangul → Braille 점자 변환
- **Integration**: IPA → Hangul → Braille 완전 체인

## 파일 구조

```
braille/
├── README.md                    # 이 문서
├── braille-wihp-mapping.md      # 통합 매핑 표
└── (wia-braille 프로젝트 연동)
```

## 활용 사례

1. **다국어 점자 학습**: 외국어 발음을 한글 점자로 학습
2. **시각장애인 언어 교육**: IPA 기호를 촉각으로 학습
3. **점자 언어 사전**: 다국어 단어를 점자로 표기

## 관련 프로젝트

- WIA-Braille: `/home/user/wia-braille`
- WIHP: `/home/user/WIHP`
