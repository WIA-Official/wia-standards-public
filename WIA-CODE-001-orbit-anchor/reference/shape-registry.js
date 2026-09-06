/* ============================================================================
 *  WIA Code — 실루엣 정본 레지스트리 (2026-08-28 신설)
 *
 *  ★★여기가 실루엣의 **유일한 정본**이다. 새 실루엣은 이 파일에만 추가한다.
 *
 *  ── 왜 만들었나 ──────────────────────────────────────────────────────────
 *  전에는 같은 목록이 **네 군데에 복사**돼 있었다:
 *    ① 생성기 로스터        orbit/web/generate.js      SHAPES
 *    ② 해독 트라이얼 목록    orbit/web/build-core.js    NON_SQUARE_SHAPES
 *    ③ API 생성 목록        wiacode-api-node/.../wia-render.js  SHAPES
 *    ④ 집계 화이트리스트     wiacode-api-node/.../milestone.js  SHAPES
 *  하나만 빠뜨려도 조용히 깨지는데, 증상이 매번 다른 곳에서 나왔다:
 *    · ②에서 빠지면 → **생성은 되는데 디코드가 영원히 안 된다**
 *      (2026-08-18 실사고: clover/boomerang/star/hex 4종이 그랬다)
 *    · ①에만 있고 게이트가 꺼져 있으면 → **유령 로스터**
 *      (2026-08-28 실사고: 만들 수 없는 piano 에 용량표 값이 들어갔다)
 *    · ④에서 빠지면 → 집계가 조용히 'unknown' 으로 떨어진다
 *  그래서 목록을 하나로 모으고, 나머지는 전부 여기서 **파생**시킨다.
 *
 *  ── 새 실루엣 추가 절차 ──────────────────────────────────────────────────
 *   1. ~/wiacode-shape-lab/check-mask.js 로 심사 (9·6·4·3 px/칸 전 크기 해독 필수)
 *   2. geometry.js 에 기하 함수 또는 마스크 추가 + insideShape 분기
 *   3. **이 파일에 한 줄 추가**
 *   4. cd orbit/web && node build-core.js      (엔진 재생성)
 *   5. node ~/wiacode-shape-lab/verify-registry.js   (4곳 정합 확인)
 *   6. scan-sw.js 의 CACHE_NAME 을 올린다 (안 올리면 설치 사용자는 옛 엔진)
 *   7. API 재기동:  pm2 restart wia-api-node
 *
 *  ── status 의 뜻 ────────────────────────────────────────────────────────
 *   'current'            생성·해독 둘 다 된다. 생성기·API 에 노출된다.
 *   'legacy-decode-only' **생성은 중단**했지만 현장에 발행분이 있어 해독은 살린다.
 *                        생성기·API 에는 안 나오고, 해독 트라이얼과 집계에만 남는다.
 *   ※ 생성도 해독도 안 할 실루엣은 **여기서 지운다**(기하 함수는 남겨도 된다 —
 *     이 목록에 없으면 어차피 도달하지 못한다).
 *
 *  ── ★확장 한계 (2026-08-28 실측) ─────────────────────────────────────────
 *  카탈로그가 커질 때 어디서 막히는지 미리 재 뒀다. 추측이 아니라 실측이다.
 *    · 마스크형 1종 = geometry.js 에 **2.9 KB** (128×128 base64 2732자)
 *    · 수식형 1종  = 약 0.5~1.5 KB
 *    · 스캔 트라이얼 비용 = **측정 한계 이하** (목록 뒤쪽 모양이 앞쪽보다 오히려 빨랐다)
 *      → 개수가 스캔 속도를 늦추지는 않는다. 병목은 **엔진 파일 크기**다.
 *
 *      실루엣  14종 → 엔진 159 KB   (현재)
 *              50종 → 엔진 263 KB
 *             100종 → 엔진 408 KB
 *             300종 → 엔진 987 KB
 *            1000종 → 엔진 3.0 MB   ← PWA 프리캐시로 감당 못 한다
 *
 *  ★즉 이 구조는 **100종 남짓까지** 편하다. 그 이상으로 갈 거면 구조를 바꿔야 한다:
 *    (a) 궤도 링 24도트에 **실루엣 ID** 를 싣는다. 지금 그 24비트는 고정 패턴이라
 *        정보를 전혀 안 싣는다(실측 확인: 모양·격자·내용 무관하게 동일).
 *        ID 가 있으면 디코더가 마스크를 **필요할 때만** 받아 쓸 수 있다.
 *    (b) 마스크를 엔진에서 떼어 **지연 로드**한다(코어 8종만 번들).
 *  둘 다 포맷/디코더 변경이라 적합성 킷 개정과 묶어야 한다. 지금은 필요 없다.
 * ========================================================================== */
(function (root, factory) {
  var api = factory();
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.WiaShapes = api;
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  // geom: 'formula' = 수식(cos/sin 등) · 'mask' = 128×128 비트맵
  //   마스크형은 형상을 살리는 **흰 통로**가 자유곡선이라 수식으로 못 쓰는 것들이다
  //   (귀의 이륜 골, 뇌의 고랑, 손발의 손가락 사이). 장미의 로그나선 골과 같은 원리.
  var SHAPES = [
    { id: 'heart',     ko: '하트',       en: 'Heart',     geom: 'formula', status: 'current' },
    { id: 'rose',      ko: '장미',       en: 'Rose',      geom: 'formula', status: 'current' },


    { id: 'round',     ko: '원형',       en: 'Circle',    geom: 'formula', status: 'current' },
    { id: 'clover',    ko: '네잎클로버', en: 'Clover',    geom: 'formula', status: 'current' },
    { id: 'star',      ko: '별',         en: 'Star',      geom: 'formula', status: 'current' },
    /* ★2026-08-30 이름 정정 — 이 도형은 **부메랑이 아니라 4각 별**이다(오너 지적).
     *   id 는 `boomerang` 그대로 둔다: id 는 코드에 저장되지 않는 **내부 이름**이고,
     *   바꾸면 119개 언어 사본의 홍보 문구·이미지까지 따라 고쳐야 하는데 얻는 게 없다.
     *   사람이 보는 건 아래 ko/en 이다. 진짜 부메랑은 별도 실루엣으로 들어간다. */
    { id: 'boomerang', ko: '반짝임',     en: 'Sparkle',   geom: 'formula', status: 'current' },
    { id: 'hex',       ko: '육각',       en: 'Hexagon',   geom: 'formula', status: 'current' },
    { id: 'square',    ko: '사각',       en: 'Square',    geom: 'formula', status: 'current' },
    // ★2026-08-28 편입 — 신체 6종. 전부 심사 통과(9·6·4·3 px/칸 해독 확인).
    { id: 'eye',       ko: '눈',         en: 'Eye',       geom: 'mask',    status: 'current' , badge: 'NEW' },
    { id: 'nose',      ko: '코',         en: 'Nose',      geom: 'mask',    status: 'current' , badge: 'NEW' },
    { id: 'ear',       ko: '귀',         en: 'Ear',       geom: 'mask',    status: 'current' , badge: 'NEW' },
    { id: 'hand',      ko: '손',         en: 'Hand',      geom: 'mask',    status: 'current' , badge: 'NEW' },
    { id: 'foot',      ko: '발',         en: 'Foot',      geom: 'mask',    status: 'current' , badge: 'NEW' },
    { id: 'brain',     ko: '뇌',         en: 'Brain',     geom: 'mask',    status: 'current' , badge: 'NEW' },
    // ★2026-08-29 — 카탈로그 첫 줄(Eye·Hand·Brain·Heart·Rose·Butterfly·Earth) 완성용.
    { id: 'butterfly', ko: '나비',       en: 'Butterfly', geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'earth',     ko: '지구',       en: 'Earth',     geom: 'mask',    status: 'current', badge: 'NEW' },
    // ★2026-08-29 재도전 성공 — 2026-08-26 불합격(소형 해독 실패) 후 기하로 재제작.
    //   실패 원인은 흰 구분선이 1~3%(1.3~3.7칸)로 데이터 점과 구분이 안 된 것.
    //   성공값: 구분선 4칸 + 검은건반 홈 13칸(대비 3.3배). 두께가 아니라 **대비**가 관건이었다.
    { id: 'piano',     ko: '건반',       en: 'Keyboard',     geom: 'mask',    status: 'current', badge: 'NEW' },
    // 생성 중단분 — 현장 발행물이 있어 해독만 살린다.
    { id: 'bubble',    ko: '말풍선',     en: 'Bubble',    geom: 'formula', status: 'legacy-decode-only' },
    { id: 'dove',                       ko: '평화',             en: 'Peace',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'love',                       ko: '사랑',             en: 'Love',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'hope',                       ko: '희망',             en: 'Hope',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'connection',                       ko: '연결',             en: 'Connection',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'family',                       ko: '가족',             en: 'Family',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'smile',                       ko: '웃음',             en: 'Smile',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'sadness',                       ko: '슬픔',             en: 'Sadness',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'friendship',                       ko: '우정',             en: 'Friendship',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'embrace',                       ko: '포옹',             en: 'Embrace',             geom: 'mask',    status: 'current', badge: 'NEW' },
    // ★2026-09-03 은퇴(오너 지시) — 밀착도 안 되고(V자 열린 아래쪽에 짝 없음, HUG_ON 주석 참고)
    //   오너가 "여전히 튄다"고 판단, 4방향 대칭인 `fan`(선풍기 날개)으로 대체했다.
    //   말풍선(bubble→chat)과 같은 방식: 생성만 중단하고 기존 발행분 해독은 그대로 살린다.
    { id: 'boomer',                       ko: '부메랑',             en: 'Boomerang',             geom: 'mask',    status: 'legacy-decode-only' },
    { id: 'grandpiano',                       ko: '그랜드피아노',             en: 'GrandPiano',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'heartorgan',                       ko: '심장',             en: 'Anatomical Heart',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'chat',                       ko: '말풍선',             en: 'Chat',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'book',                       ko: '책',             en: 'Book',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'kiosk',                       ko: '키오스크',             en: 'Kiosk',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'gimbap',                       ko: '김밥',             en: 'Gimbap',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'elevator',                       ko: '엘리베이터',             en: 'Elevator',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'manse',                       ko: '만세',             en: 'Hooray',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'lonely',                       ko: '외로움',             en: 'Loneliness',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'together',                       ko: '함께',             en: 'Together',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'comfort',                       ko: '위로',             en: 'Comfort',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'dolhareubang',                       ko: '돌하르방',             en: 'Dolhareubang',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'tree',                       ko: '나무',             en: 'Tree',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'cup',                       ko: '컵',             en: 'Cup',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'car',                       ko: '자동차',             en: 'Car',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'dog',                       ko: '강아지',             en: 'Dog',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'cat',                       ko: '고양이',             en: 'Cat',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'fish',                       ko: '물고기',             en: 'Fish',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'bear',                       ko: '곰',             en: 'Bear',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'rabbit',                       ko: '토끼',             en: 'Rabbit',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'turtle',                       ko: '거북이',             en: 'Turtle',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'whale',                       ko: '고래',             en: 'Whale',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'horse',                       ko: '말',             en: 'Horse',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'bird',                       ko: '새',             en: 'Bird',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'deer',                       ko: '사슴',             en: 'Deer',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'hearingaid',                       ko: '보청기',             en: 'Hearing Aid',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'tooth',                       ko: '치아',             en: 'Tooth',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'mouth',                       ko: '입',             en: 'Mouth',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'lungs',                       ko: '폐',             en: 'Lungs',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'bone',                       ko: '뼈',             en: 'Bone',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'arena',                       ko: '경기장',             en: 'Arena',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'pestbug',                       ko: '해충 · 방역',             en: 'Pest Control',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'finger',                       ko: '검지',             en: 'Finger',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'like',                       ko: '좋아요',             en: 'Like',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'victory',                       ko: '브이',             en: 'Victory',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'oksign',                       ko: '오케이',             en: 'OK Sign',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'lovesign',                       ko: '사랑해요',             en: 'Love Sign',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'prayer',                       ko: '기도',             en: 'Prayer',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'fist',                       ko: '주먹',             en: 'Fist',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'badge',                       ko: '배지',             en: 'Badge',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'waterdrop',                       ko: '물방울',             en: 'Waterdrop',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'crown',                       ko: '왕관',             en: 'Crown',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'bag',                       ko: '가방',             en: 'Bag',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'watch',                       ko: '시계',             en: 'Watch',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'ring',                       ko: '반지',             en: 'Ring',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'perfume',                       ko: '향수병',             en: 'Perfume',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'fan',                       ko: '선풍기 날개',             en: 'Fan Blade',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'helmet',                       ko: '안전모',             en: 'Hard Hat',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'goggles',                       ko: '보안경',             en: 'Safety Goggles',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'harness',                       ko: '안전대',             en: 'Safety Harness',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'extinguisher',                       ko: '소화기',             en: 'Fire Extinguisher',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'safetycone',                       ko: '안전콘',             en: 'Safety Cone',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'gloves',                       ko: '안전장갑',             en: 'Safety Gloves',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'dustmask',                       ko: '방진마스크',             en: 'Dust Mask',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'haetae',                       ko: '해치',             en: 'Haetae',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'safetyboot',                       ko: '안전화',             en: 'Safety Boot',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'safetyvest',                       ko: '안전조끼',             en: 'Safety Vest',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'elephant',                       ko: '코끼리',             en: 'Elephant',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'owl',                       ko: '부엉이',             en: 'Owl',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'crab',                       ko: '게',             en: 'Crab',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'octopus',                       ko: '문어',             en: 'Octopus',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'snail',                       ko: '달팽이',             en: 'Snail',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'bee',                       ko: '벌',             en: 'Bee',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'frog',                       ko: '개구리',             en: 'Frog',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'swan',                       ko: '백조',             en: 'Swan',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'umbrella',                       ko: '우산',             en: 'Umbrella',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'key',                       ko: '열쇠',             en: 'Key',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'lightbulb',                       ko: '전구',             en: 'Light Bulb',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'scissors',                       ko: '가위',             en: 'Scissors',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'hammer',                       ko: '망치',             en: 'Hammer',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'glasses',                       ko: '안경',             en: 'Glasses',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'envelope',                       ko: '편지봉투',             en: 'Envelope',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'gift',                       ko: '선물상자',             en: 'Gift Box',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'apple',                       ko: '사과',             en: 'Apple',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'icecream',                       ko: '아이스크림',             en: 'Ice Cream',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'cupcake',                       ko: '컵케이크',             en: 'Cupcake',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'donut',                       ko: '도넛',             en: 'Donut',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'airplane',                       ko: '비행기',             en: 'Airplane',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'sailboat',                       ko: '돛단배',             en: 'Sailboat',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'rocket',                       ko: '로켓',             en: 'Rocket',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'crescent',                       ko: '초승달',             en: 'Crescent Moon',             geom: 'mask',    status: 'current', badge: 'NEW' },
    { id: 'snowflake',                       ko: '눈송이',             en: 'Snowflake',             geom: 'mask',    status: 'current', badge: 'NEW' },
  ];


  // ── 같은 발자국에 들어가는 QR 용량 (생성기 용량 게이지의 "QR 등가" 막대) ──
  //   ★★2026-08-28 실사고: 이 표가 generate.js 에 따로 있었고, 새 실루엣 6종을 넣을 때
  //     **여기만 빠뜨렸다.** qrCapFor() 가 null 을 돌려주면서 화면에 "92 / null B" 와
  //     가짜 OVERFLOW 가 떴다. 등록 지점을 다섯 곳 통합해 놓고 여섯 번째를 놓친 것이다.
  //     → 표를 정본으로 옮긴다. 실루엣을 추가하면 여기 한 줄도 같이 채워야 하고,
  //       verify-registry.js 가 빠진 실루엣을 잡아낸다.
  //
  //   산출 규칙(benchmark.js §C 와 동일): 같은 발자국(격자 N + 콰이엇 4×2) · 같은 피치 ·
  //     양쪽 오류정정 25%(우리 RS ↔ QR ECC-Q). 정사각은 QR 이 발자국 전체를 쓰고,
  //     비정사각은 **실루엣 안 최대 내접 정사각**까지만 쓴다(QR 은 사각형 밖으로 못 나간다).
  //     0 = 그 크기·실루엣에는 QR 을 v1 조차 못 넣는다.
  //   재계산: ~/wiacode-qr-audit/scripts/qrcaps_table.js
  var QR_SAME_FOOTPRINT = {
    S: { piano: 32, heart: 32, rose: 32, round: 60, clover: 0, star: 0, boomerang: 0, hex: 32, square: 177, eye: 0, nose: 0, ear: 0, hand: 0, foot: 0, brain: 32, bubble: 86 , butterfly: 20, earth: 46, dove: 11, love: 20, hope: 60, connection: 20, family: 20, smile: 32, sadness: 46, friendship: 20, embrace: 32, boomer: 0, grandpiano: 32, heartorgan: 11, chat: 32, book: 60, kiosk: 20, gimbap: 11, elevator: 46, manse: 11, lonely: 20, together: 32, comfort: 32, dolhareubang: 11, tree: 20, cup: 20, car: 32, dog: 11, cat: 11, fish: 20, bear: 20, rabbit: 0, turtle: 32, whale: 0, horse: 0, bird: 0, deer: 11, hearingaid: 46, tooth: 32, mouth: 11, lungs: 46, bone: 32, arena: 11, pestbug: 11, finger: 0, like: 11, victory: 0, oksign: 0, lovesign: 0, prayer: 0, fist: 32, badge: 20, waterdrop: 11, crown: 32, bag: 32, watch: 0, ring: 20, perfume: 32, fan: 0, helmet: 20, goggles: 20, harness: 0, extinguisher: 11, safetycone: 0, gloves: 32, dustmask: 0, haetae: 46, safetyboot: 0, safetyvest: 32, elephant: 32, owl: 32, crab: 32, octopus: 20, snail: 46, bee: 0, frog: 32, swan: 11, umbrella: 0, key: 0, lightbulb: 0, scissors: 0, hammer: 0, glasses: 0, envelope: 60, gift: 11, apple: 32, icecream: 0, cupcake: 20, donut: 46, airplane: 0, sailboat: 20, rocket: 0, crescent: 0, snowflake: 11 },
    M: { piano: 108, heart: 108, rose: 108, round: 151, clover: 11, star: 32, boomerang: 11, hex: 108, square: 442, eye: 32, nose: 46, ear: 32, hand: 32, foot: 11, brain: 108, bubble: 258 , butterfly: 46, earth: 130, dove: 60, love: 86, hope: 151, connection: 74, family: 86, smile: 108, sadness: 151, friendship: 74, embrace: 86, boomer: 32, grandpiano: 86, heartorgan: 60, chat: 86, book: 177, kiosk: 74, gimbap: 60, elevator: 151, manse: 46, lonely: 74, together: 86, comfort: 108, dolhareubang: 60, tree: 74, cup: 74, car: 108, dog: 60, cat: 60, fish: 86, bear: 86, rabbit: 32, turtle: 86, whale: 11, horse: 46, bird: 46, deer: 46, hearingaid: 130, tooth: 108, mouth: 46, lungs: 130, bone: 108, arena: 46, pestbug: 46, finger: 20, like: 60, victory: 11, oksign: 32, lovesign: 46, prayer: 20, fist: 108, badge: 74, waterdrop: 60, crown: 108, bag: 108, watch: 32, ring: 74, perfume: 86, fan: 11, helmet: 86, goggles: 60, harness: 46, extinguisher: 60, safetycone: 32, gloves: 108, dustmask: 46, haetae: 130, safetyboot: 20, safetyvest: 108, elephant: 108, owl: 86, crab: 108, octopus: 60, snail: 130, bee: 32, frog: 86, swan: 60, umbrella: 46, key: 11, lightbulb: 32, scissors: 20, hammer: 11, glasses: 32, envelope: 151, gift: 46, apple: 86, icecream: 32, cupcake: 86, donut: 108, airplane: 0, sailboat: 74, rocket: 20, crescent: 0, snowflake: 32 },
    L: { piano: 241, heart: 241, rose: 203, round: 322, clover: 46, star: 74, boomerang: 32, hex: 241, square: 805, eye: 74, nose: 86, ear: 74, hand: 74, foot: 46, brain: 241, bubble: 482 , butterfly: 86, earth: 258, dove: 130, love: 177, hope: 292, connection: 151, family: 177, smile: 203, sadness: 292, friendship: 151, embrace: 203, boomer: 60, grandpiano: 203, heartorgan: 108, chat: 203, book: 322, kiosk: 151, gimbap: 130, elevator: 292, manse: 108, lonely: 151, together: 203, comfort: 241, dolhareubang: 108, tree: 151, cup: 151, car: 203, dog: 108, cat: 130, fish: 177, bear: 177, rabbit: 74, turtle: 177, whale: 46, horse: 86, bird: 86, deer: 86, hearingaid: 258, tooth: 241, mouth: 108, lungs: 258, bone: 241, arena: 108, pestbug: 108, finger: 60, like: 130, victory: 46, oksign: 74, lovesign: 86, prayer: 60, fist: 241, badge: 151, waterdrop: 130, crown: 203, bag: 241, watch: 74, ring: 151, perfume: 177, fan: 46, helmet: 177, goggles: 130, harness: 60, extinguisher: 130, safetycone: 74, gloves: 203, dustmask: 86, haetae: 258, safetyboot: 46, safetyvest: 203, elephant: 203, owl: 203, crab: 203, octopus: 108, snail: 292, bee: 74, frog: 203, swan: 130, umbrella: 86, key: 32, lightbulb: 74, scissors: 60, hammer: 32, glasses: 74, envelope: 292, gift: 86, apple: 203, icecream: 74, cupcake: 177, donut: 241, airplane: 20, sailboat: 151, rocket: 60, crescent: 20, snowflake: 74 },
  };
  /** 같은 발자국 QR 용량(바이트). 모르는 실루엣이면 null — 비교 문구를 띄우지 않는다. */
  function qrCapFor(grid, shape) {
    var g = QR_SAME_FOOTPRINT[grid]; if (!g) return null;
    var v = g[shape];
    return (v === undefined) ? null : v;
  }

  var byId = {};
  for (var i = 0; i < SHAPES.length; i++) byId[SHAPES[i].id] = SHAPES[i];

  function ids(list) { var r = []; for (var i = 0; i < list.length; i++) r.push(list[i].id); return r; }
  /* ── 생성기 첫 화면 노출 순서 (2026-09-01 신설) ────────────────────────────
   * 생성기는 66종을 다 굽지 않고 상위 N종만 만든다(실측 3797ms → 460ms, 8.3배).
   * 그 N종을 고르는 순서가 이 표다. 낮을수록 먼저.
   *
   * ★★집계 기반 자동 순위는 **아직 켜지 않았다.** milestone.db 의 daily 테이블에
   *   shape 차원이 있지만(2026-08-27 추가) 지금 상태가 이렇다:
   *     generate 220건 중 **180건이 unknown**(구클라이언트) → 유효 표본 40건
   *     pestbug 18 · ear 14 · 나머지 8종 각 1
   *   이대로 노출하면 **1위 해충 · 2위 귀**가 뜬다. 서비스 첫인상이 그렇게 정해지면 안 된다.
   * ★자동 전환 조건(둘 다 만족할 때만):
   *     ① 유효 표본(unknown 제외 generate 합계) >= 500
   *     ② 1위 실루엣 비중 < 60%   ← 한 종이 쏠리면 나머지가 안 보인다
   *   그 전까지는 아래 큐레이션 순서를 쓴다. 조건을 다시 조사하지 않아도 되게 여기 적어 둔다.
   * ★하트 카운트(♥ 수치) 표시도 같은 조건에 묶는다 — 지금 숫자는 1~18 이라
   *   그대로 보이면 서비스가 한산해 보인다. */
  var SHAPE_RANK = {
    // ★오너 확정(2026-09-01). 기준: "네모가 아니어도 읽힌다"를 첫 화면에서 한눈에
    //   보여주는 순서. 문화 실루엣(돌하르방·김밥·만세)을 앞에 두는 안은 뺐다 —
    //   B1 인지 측정에서 문화 6종의 인지율이 21% 라 처음 보는 사람이 "이게 뭐지"가 된다.
    //   바꾸려면 이 표만 갈아 끼우면 된다(다른 코드 무수정).
    heart: 1, round: 2, star: 3, clover: 4, butterfly: 5, dog: 6, tree: 7, hex: 8
  };

  function filter(fn) { var r = []; for (var i = 0; i < SHAPES.length; i++) if (fn(SHAPES[i])) r.push(SHAPES[i]); return r; }

  /** 생성 가능한 실루엣(생성기·API 노출). */
  function current() { return filter(function (s) { return s.status === 'current'; }); }
  /** 첫 화면 노출 순서(낮을수록 먼저). 표에 없으면 9999 — 정의부 SHAPE_RANK 주석 참조.
   *  ★항목 객체에 rank 를 심지 않는다(레지스트리 자체를 변형하지 않기 위함). */
  function rank(id) { var v = SHAPE_RANK[id]; return (v == null) ? 9999 : v; }
  /** 해독해야 하는 실루엣(생성 가능 + 현장 발행분). */
  function decodable() { return filter(function (s) { return s.status === 'current' || s.status === 'legacy-decode-only'; }); }
  /** 디코더 트라이얼 목록 — square 는 앵커 배치가 달라 별도 경로라 제외한다. */
  function nonSquareDecodable() { return ids(decodable()).filter(function (id) { return id !== 'square'; }); }
  /** 집계 화이트리스트 — 해독 대상 + 'custom'(사용자 마스크 보고). */
  function countable() { return ids(decodable()).concat(['custom']); }
  /** 언어별 이름 사전 { id: '이름' } */
  function names(lang) {
    var o = {}, L = decodable();
    for (var i = 0; i < L.length; i++) o[L[i].id] = L[i][lang] || L[i].id;
    return o;
  }

  /* ★밀착을 **실제로 켠** 실루엣 (2026-08-30).
   *   핀(HUG_PINS)은 **전 실루엣**에 박는다 — 해독기가 빨리 후보를 만들려면 필요하다.
   *   그러나 **생성**은 여기 있는 것만 밀착으로 굽는다. 하나씩 늘려 가며 실기기로 확인한다.
   *   ★비어 있으면 아무것도 안 바뀐다(옛 배치 그대로) — 되돌리기가 이 줄 하나다.
   *   ★★이 블록은 HUG_PINS **앞**에 둔다. pin-hug.js 가 HUG_PINS 부터 hugPin 주석까지를
   *     통째로 갈아끼우므로, 뒤에 두면 핀을 다시 박을 때 **조용히 지워진다**(2026-08-30 실사고). */
  /* 2026-08-30: dove 를 뺀 **전부**를 켰다.
   *   근거(실측): 기본 배치에서 위성 덮임이 25종 중 23종이 **50% 미만**이다
   *   (clover·eye·hand·brain·butterfly·smile·embrace 는 0% — 완전히 흰 공간).
   *   밀착하면 전부 95~100% 로 들어온다. 프레임 반경도 rose −70%, star −46%, hand −38%.
   *   ★섞어 두면 "어떤 건 되고 어떤 건 안 되네"를 사용자가 겪는다 — 규칙은 하나여야 한다.
   *   ★dove 는 원래 불가였다(깃털 틈이 바깥까지 이어짐). 오너가 **몸통을 굵게** 다시 그려
   *     2026-08-30 통과했다(33.3/25.0, 이득 2.42배, φ 0°).
   *
   *   ★★rose 만 뺐다 — **두 축이 모두 바닥**(15.75/15.75)이라 사방으로 4.1배 외삽이 되고,
   *     작게 찍으면 궤도 재적합도 못 버틴다. 실측: 화면점유 한계가 **28% → 40% 로 나빠졌다.**
   *     밀착이 주는 것(프레이밍)보다 잃는 것(스캔 거리)이 크다.
   *     ★판정 규칙: **긴 축이 25모듈 미만이면 켜지 마라.** 25종 중 rose 하나가 걸린다
   *       (다음은 star 31 — 여유 있다). 한 축만 작은 건 괜찮다(foot 16/51 은 51 축이 버텨 준다).
   *     rose 를 살리려면 로그나선 골을 얕게 다시 그려야 한다. */
  var HUG_ON = {
    // 2026-09-05 편입 배치 — induct '✅ 밀착 적격' 판정분(정책: 실기기 확인 없이 바로 켠다)
    owl: true, crab: true, octopus: true, bee: true, frog: true, swan: true, lightbulb: true, glasses: true, apple: true, icecream: true, airplane: true, sailboat: true, rocket: true,
    heart: true, round: true, clover: true, star: true, boomerang: true,
    hex: true, eye: true, nose: true, ear: true, hand: true, foot: true,
    brain: true, butterfly: true, earth: true, piano: true, love: true, hope: true,
    connection: true, family: true, smile: true, sadness: true, friendship: true, embrace: true,
    dove: true, grandpiano: true,
    heartorgan: true, chat: true, book: true, kiosk: true, gimbap: true,
    /* ★엘리베이터는 이득 1.12배로 문턱(1.25) 아래지만 **φ 44°** 라 켠다 —
       안 켜면 3·9시 위성이 형상 밖에 앉는다(실측, 2026-08-31). 조건은 이득 또는 φ 다. */
    elevator: true,
    manse: true, lonely: true, together: true, comfort: true, dolhareubang: true,
    tree: true, cup: true, car: true,
    dog: true, cat: true, fish: true, bear: true, rabbit: true, turtle: true,
    whale: true, horse: true, bird: true, deer: true,
    hearingaid: true, tooth: true, mouth: true, lungs: true,
    finger: true, like: true, victory: true, oksign: true, lovesign: true, prayer: true, fist: true,
    bone: true, arena: true, pestbug: true,
    /* ★boomer(진짜 부메랑)는 뺐다 — V자가 열린 **아래쪽 9/24 방향에 잉크가 없어서**
     *   마주 보는 짝을 못 만든다. 부메랑은 원래 그렇게 생긴 물건이고, 억지로 살을 붙이면
     *   부메랑이 아니게 된다. 밀착만 안 켜고 코드는 그대로 잘 읽힌다. */
    /* ★2026-09-03 (오너 판단) — "실기기로 하나씩 확인 후 켠다"는 원칙을 완화했다.
     *   지금까지 편입된 전 실루엣 중 밀착을 못 켠 예외는 boomer 단 하나(위 주석)뿐이었고,
     *   그 사유(마주 보는 짝 부재)는 induct.js의 "④ 밀착 적격" 검사가 이미 자동으로 잡아낸다
     *   (실측: 62/63 — induct.js 통과가 실기기 결과와 사실상 100% 일치). 그러니 이 검사를
     *   통과한(이득 배율이 나온) 실루엣은 실기기 확인 없이 바로 켠다. 아래 7종은 이번 배치
     *   (badge~perfume) 전부 induct.js "✅ 밀착 적격"(이득 1.29~2.02배)로 통과했다.
     *   자세한 인수인계: SILHOUETTE_INDUCT_HANDOFF.md §10. */
    badge: true, waterdrop: true, crown: true, bag: true, watch: true, ring: true, perfume: true,
    // ★2026-09-03 — helmet 은 위 정책 그대로: induct.js "✅ 밀착 적격"(이득 1.34배) 통과.
    helmet: true,
    /* ★2026-09-03 (오너 지시) — fan 은 induct.js 자체 문턱(이득 ≥1.25배)에는 못 미친다
     *   (실측 1.10배, hug-check.js "밀착 불필요 — 이미 꽉 찬 형상"). 그런데 이 문턱은
     *   "밀착을 켤 만한 값어치가 있는가"를 묻는 것이지 "밀착이 필요한가"를 묻는 게 아니다 —
     *   fan 은 날개 몸통이 위성 방향(0/90/180/270°)으로 반경 55모듈까지만 뻗는데 밀착을 끄면
     *   위성이 기본 반경 60.5모듈에 앉아 **날개 끝보다 바깥에 떠 보인다**(오너 지적, 실측
     *   렌더로 확인). 핀 [55,55,1°]로 켜면 위성이 정확히 날개 끝단에 들어맞는다. */
    fan: true,
    /* ★2026-09-03 — 12 산업·안전 배치 5종. 위 2026-09-03 완화 정책 그대로:
     *   induct.js "✅ 밀착 적격" 통과분만 켠다(이득 1.26~3.56배, 실기기 확인 없이).
     *   같은 배치의 safetyboot 은 부적격이라 뺐다.
     *   ★관리자 2종(safetymanager·healthmanager)은 2026-09-04 오너 판정으로 **로스터에서
     *     제거**했다 — 얼굴이 빈 흰 구멍이라 코드로 구우면 형상이 안 살았다("영 이상하다").
     *     편입 4시간 만이고 대외 공지·생성 기록이 0 이라 legacy-decode-only 가 아니라
     *     완전 제거로 갔다(이 파일 머리말 status 규칙: "생성도 해독도 안 할 실루엣은 지운다").
     *     기하 마스크는 geometry.js 에 남아 있으나 이 목록에 없어 도달하지 못한다. */
    extinguisher: true, safetycone: true, gloves: true, dustmask: true, haetae: true,
    /* ★2026-09-03 — goggles·harness 는 편입 당시 "✅ 밀착 적격"(1.77배·2.40배) 판정을 받고도
     *   HUG_ON 에 안 올라와 있었다. 그 배치 세션이 API 오류로 죽으면서 핀·HUG_ON 단계를 못
     *   밟았고, 뒤이은 세션은 "의도인지 누락인지 모르겠다"며 판단을 넘겼다(옳은 처신 — 핀은
     *   그때 같이 채워졌다). 위 완화 정책(적격이면 실기기 확인 없이 켬)에 그대로 해당한다. */
    goggles: true, harness: true,
    /* ★2026-09-04 — safetyvest(안전조끼). 판독(C축)은 처음부터 깨끗했고 밀착도 적격(1.52배)이라
     *   위 완화 정책 그대로 켠다. 편입 자체는 `⚠ 사람 판단`이라 --force 로 갔다: 외곽이
     *   comfort(위로)와 95% 겹치지만 **뜻이 다르므로 카탈로그에 둘 다 둔다**(오너 판정,
     *   우정·포옹을 78~83% 겹침에도 넣은 2026-08-29 선례와 같은 판단). */
    safetyvest: true,
  };

  /** 이 실루엣을 밀착으로 **생성**할 것인가 */
  function hugOn(id) { return !!HUG_ON[id]; }

  /* ── 밀착(마름모) 반경 핀 ───────────────────────────────────────────────
   *  [rA, rB] = [12·6시 반경, 3·9시 반경] (모듈). null = 그 격자에서 밀착 불가.
   *
   *  ★왜 핀으로 박나 (2026-08-30, Fable 실측)
   *    레이아웃 반경 허용오차가 **0.5~1.0%** 다 — brain/L(rB 48.4)은 0.5% 어긋나면
   *    해독이 죽고, ear/L(rB 19.8)은 1.5%에서 죽는다. 절대오차 ~0.2모듈이 한계다.
   *    반면 반경은 마스크에서 계산되므로 **마스크를 재편입하거나 샘플링을 바꾸면
   *    조용히 드리프트**한다. 그러면 이미 발행된 코드가 안 읽힌다.
   *    → 편입 시점의 값을 여기 박아 두고, geometry.js 는 이 값을 **먼저** 본다.
   *    부수효과: 첫 스캔의 solver 비용(~1.2초)이 사라진다.
   *
   *  ★값은 손으로 쓰지 말 것. `node ~/wiacode-shape-lab/pin-hug.js --write` 가 만든다.
   *  ★핀이 없으면 geometry.js 가 그때그때 계산한다 — 동작은 하지만 위 드리프트에 노출된다.
   *    `verify-registry.js` 가 누락을 잡는다. */
  var HUG_PINS = {
    'airplane': { S: [23.75, 15.5, 0], M: [38.75, 26.5, 0], L: [53.75, 37.5, 0] },
    'apple': { S: [26.5, 21.5, 25], M: [41.5, 34.5, 22], L: [57, 47.25, 22] },
    'arena': { S: [15.25, 15.25, 44], M: [25.5, 25, 42], L: [35.25, 34.5, 40] },
    'badge': { S: [20, 20, 45], M: [32.5, 31.75, 44], L: [45.75, 42.5, 41] },
    'bag': { S: [25.25, 19.75, 0], M: [39.75, 31.25, 0], L: [54.75, 43.25, 0] },
    'bear': { S: [23.75, 18.75, 0], M: [37.75, 30.25, 0], L: [51.75, 41.75, 0] },
    'bee': { S: [14.5, 21.5, 1], M: [24, 34.75, 0], L: [51.5, 25.5, 28] },
    'bird': { S: [24.75, 14, 63], M: [39.75, 22.5, 62], L: [60, 30.5, 58] },
    'bone': { S: [23, 18.75, 46], M: [36.75, 36, 46], L: [51.5, 48.5, 47] },
    'book': { S: [20.5, 28, 32], M: [33.25, 44, 34], L: [46.25, 59.75, 36] },
    'boomerang': { S: [23, 23, 1], M: [38.75, 38.75, 0], L: [54.75, 54.75, 0] },
    'brain': { S: [23, 26.5, 73], M: [36.25, 41.5, 72], L: [49.75, 56.75, 72] },
    'butterfly': { S: [28, 28, 45], M: [44, 44, 45], L: [60, 60, 45] },
    'car': { S: [21, 21, 42], M: [33.5, 34, 46], L: [37.5, 55.25, 21] },
    'cat': { S: [24.75, 15.25, 35], M: [38.75, 26.75, 38], L: [42.25, 49.25, 48] },
    'chat': { S: [24.75, 20, 32], M: [39.5, 32.5, 32], L: [54, 44.25, 32] },
    'clover': { S: [24, 24, 1], M: [38, 38, 1], L: [52, 52, 1] },
    'comfort': { S: [26.75, 19.5, 26], M: [42.25, 31, 26], L: [57.5, 43.25, 26] },
    'connection': { S: [23.5, 14, 68], M: [40.5, 20.5, 77], L: [33.75, 49, 31] },
    'crab': { S: null, M: null, L: [20, 53.5, 43] },
    'crown': { S: [14, 14, 48], M: [21.75, 24.5, 56], L: [30, 34.5, 60] },
    'cup': { S: [14.5, 17.5, 85], M: [36.5, 29, 3], L: [52.5, 40, 1] },
    'cupcake': { S: [24, 22.5, 2], M: [39, 35, 3], L: [54, 48.5, 2] },
    'deer': { S: null, M: [16.5, 14.5, 41], L: [25, 20.5, 39] },
    'dog': { S: [26.5, 25, 43], M: [41.75, 39.75, 43], L: [57.25, 54.75, 43] },
    'dolhareubang': { S: null, M: [20.5, 40, 89], L: [28.5, 55, 89] },
    'donut': { S: [24.25, 25.75, 9], M: [38, 40, 1], L: [52, 55, 1] },
    'dove': { S: null, M: [20, 25.5, 1], L: null },
    'dustmask': { S: [19, 25.5, 1], M: [30.5, 40, 1], L: [42, 55, 1] },
    'ear': { S: [22, 13.75, 25], M: [36.75, 21.75, 21], L: [51.25, 29.5, 19] },
    'earth': { S: [26, 26, 77], M: [40.75, 40.75, 11], L: [56, 56, 11] },
    'elephant': { S: [25.25, 22, 41], M: [39.75, 35.75, 42], L: [54.25, 50.75, 43] },
    'elevator': { S: [25, 25, 44], M: [39.75, 39.75, 44], L: [54.75, 54.25, 44] },
    'embrace': { S: [19, 27.25, 69], M: [31, 42, 72], L: [42.75, 58, 71] },
    'envelope': { S: [25, 27.75, 42], M: [39.75, 44, 42], L: [56, 60, 43] },
    'extinguisher': { S: null, M: [14.25, 32.25, 54], L: [53.75, 17, 16] },
    'eye': { S: [14, 14, 45], M: [35, 19.5, 86], L: [36.75, 29.25, 62] },
    'family': { S: [23, 21.25, 69], M: [33.25, 37, 19], L: [46.25, 51.5, 18] },
    'fan': { S: [25.75, 25.75, 82], M: [40.75, 40, 79], L: [55, 55, 1] },
    'finger': { S: null, M: [41, 18.25, 13], L: [56.25, 25.5, 13] },
    'fish': { S: [24.25, 22.25, 79], M: [39.25, 35.25, 78], L: [54.25, 48, 77] },
    'fist': { S: [20.25, 26, 74], M: [32, 41, 74], L: [55.5, 44.25, 8] },
    'foot': { S: null, M: [19, 13.75, 34], L: [16, 51, 87] },
    'friendship': { S: [22.5, 21.5, 44], M: [35.5, 36.25, 45], L: [48, 50.5, 46] },
    'frog': { S: [17.5, 24.75, 0], M: [37.25, 30.25, 31], L: [39, 53.75, 0] },
    'gift': { S: [26.75, 22.5, 21], M: [43.25, 36.25, 23], L: [59.75, 50, 23] },
    'gimbap': { S: [26.75, 15.5, 57], M: [42, 26.25, 59], L: [57.25, 36.25, 58] },
    'glasses': { S: null, M: [22, 22, 45], L: [31.25, 31.75, 44] },
    'gloves': { S: [18.75, 27.75, 64], M: [30, 44, 64], L: [41.25, 60, 63] },
    'goggles': { S: [16.25, 20.5, 34], M: [24.75, 34, 30], L: [34.25, 46.75, 29] },
    'grandpiano': { S: null, M: [37, 22, 3], L: [51, 35.5, 1] },
    'haetae': { S: [22.25, 22.25, 70], M: [35, 35, 68], L: [48.5, 48, 68] },
    'hand': { S: null, M: null, L: [37.5, 14.25, 39] },
    'harness': { S: null, M: [31.75, 19.75, 36], L: [47.75, 25.25, 34] },
    'hearingaid': { S: [25.75, 24.25, 22], M: [40.5, 37.75, 26], L: [55, 51.75, 22] },
    'heart': { S: [17, 21, 1], M: [31.75, 30.75, 8], L: [44.25, 42.75, 7] },
    'heartorgan': { S: [21.25, 15.75, 11], M: [25.25, 38.25, 72], L: [34.75, 52.75, 71] },
    'helmet': { S: [20.5, 21.5, 1], M: [32.5, 34, 1], L: [45, 47, 1] },
    'hex': { S: [26.5, 23.25, 31], M: [42, 36.75, 31], L: [57.75, 50.25, 30] },
    'hope': { S: [23.25, 14, 67], M: [29.5, 39.25, 0], L: [41, 54, 1] },
    'horse': { S: [21.75, 17, 9], M: [37.75, 27.75, 8], L: [53.5, 38.25, 8] },
    'icecream': { S: null, M: [39.5, 16, 1], L: [55, 22, 1] },
    'kiosk': { S: [27.75, 14, 24], M: [44, 23.25, 26], L: [60, 32.5, 27] },
    'lightbulb': { S: null, M: [40, 15, 1], L: [55, 21.5, 1] },
    'like': { S: null, M: [40.5, 14, 5], L: [54.5, 20, 5] },
    'lonely': { S: [25.25, 17.25, 0], M: [40, 28, 1], L: [55, 38.5, 1] },
    'love': { S: null, M: [37.5, 35.75, 27], L: [52.25, 52.75, 68] },
    'lovesign': { S: [14.5, 26, 74], M: [23.75, 41.25, 75], L: [33.5, 56.5, 76] },
    'lungs': { S: [15, 13.75, 44], M: [29.75, 22.5, 41], L: [41.75, 32.25, 43] },
    'manse': { S: [19.5, 19.5, 1], M: [30.75, 31.25, 0], L: [43, 44, 1] },
    'mouth': { S: [13.75, 18.75, 21], M: [18.5, 35.5, 2], L: [32.5, 39.75, 25] },
    'nose': { S: null, M: [39.75, 17.75, 0], L: [55, 25, 1] },
    'octopus': { S: null, M: [29.25, 34.25, 81], L: [44.25, 45.25, 79] },
    'oksign': { S: [23, 14, 4], M: [40.25, 22.5, 7], L: [55.25, 31, 6] },
    'owl': { S: [25.5, 19.5, 1], M: [40, 31, 1], L: [43, 55, 87] },
    'perfume': { S: [25.5, 21.5, 1], M: [40, 34, 1], L: [47, 55, 89] },
    'pestbug': { S: null, M: [16.25, 44, 45], L: [23, 60, 42] },
    'piano': { S: [18.75, 28, 34], M: [31, 44, 35], L: [43.25, 59.75, 36] },
    'prayer': { S: null, M: [13.75, 15.5, 56], L: [37.25, 17.25, 13] },
    'rabbit': { S: null, M: [43, 15.5, 22], L: [59.25, 21.5, 22] },
    'ring': { S: [25.5, 19.5, 1], M: [40, 31, 1], L: [54.75, 42.75, 0] },
    'rocket': { S: null, M: [39.5, 14.5, 1], L: [55, 20, 1] },
    'rose': { S: null, M: null, L: [15.75, 15.75, 11] },
    'round': { S: [28, 28, 11], M: [44, 44, 9], L: [60, 60, 8] },
    'sadness': { S: [26.25, 27.5, 40], M: [42.5, 41.75, 47], L: [58.5, 56.5, 48] },
    'safetycone': { S: null, M: [19.75, 13.75, 25], L: [55, 18.5, 1] },
    'safetyvest': { S: [28, 17.75, 32], M: [44, 29.25, 33], L: [60, 39.75, 34] },
    'sailboat': { S: [24.5, 17.5, 0], M: [39.75, 28, 0], L: [39, 55, 89] },
    'scissors': { S: [27.75, 27, 45], M: [44, 44, 45], L: [60, 60, 44] },
    'smile': { S: [21.5, 25.5, 1], M: [34, 40, 1], L: [47, 55, 1] },
    'snail': { S: [22, 25.75, 63], M: [35.5, 40.25, 68], L: [49.25, 55, 69] },
    'snowflake': { S: [22.75, 23, 42], M: [37.25, 36.75, 0], L: [53.75, 54.75, 0] },
    'star': { S: null, M: [17, 21.75, 0], L: [24.25, 30, 35] },
    'swan': { S: [25.25, 19.25, 0], M: [40, 30.5, 1], L: [42.5, 55, 89] },
    'together': { S: [24.25, 21.5, 40], M: [37.5, 36.25, 44], L: [51, 51, 45] },
    'tooth': { S: [17.25, 27.5, 68], M: [43, 28, 22], L: [58.5, 38.75, 24] },
    'tree': { S: [22.75, 24.25, 0], M: [37, 36.5, 86], L: [51, 50, 85] },
    'turtle': { S: [28, 28, 45], M: [44, 44, 45], L: [60, 60, 44] },
    'umbrella': { S: null, M: [36.25, 39.25, 0], L: [54, 53, 89] },
    'victory': { S: null, M: [15, 32.75, 68], L: [55, 16.25, 12] },
    'watch': { S: [26.25, 14.25, 14], M: [41.25, 23.75, 15], L: [57, 32.75, 16] },
    'waterdrop': { S: [24.5, 16.75, 0], M: [39.75, 26.75, 0], L: [55, 37.5, 1] },
    'whale': { S: null, M: [30, 14, 68], L: [13.75, 21.75, 51] },
  };

  /** 밀착 반경 핀 조회 — [rA, rB, φ] 또는 null */
  function hugPin(id, grid) {
    var e = HUG_PINS[id];
    if (!e) return null;
    var v = e[grid];
    // [rA, rB] 또는 [rA, rB, φ] — φ 는 나중에 붙었으므로 2 도 3 도 받는다.
    return (v && v.length >= 2) ? v : null;
  }

  /* ── 외곽 스트로크 핀 (MASTER LINE B2, 2026-09-01) ─────────────────────────
   *   { id: [gap, w] } — 모듈 단위. 실루엣 밖으로 gap 만큼 흰 해자를 띄우고 w 두께로 두른다.
   *   ★여기 없는 형상은 스트로크를 **제공하지 않는다**(hug off 와 같은 정책) — 후보 조합이
   *     전부 해독을 깨뜨렸거나 아직 안 쟀다는 뜻이다. 임의로 전역 기본값을 쓰지 말 것.
   *   ★값은 `~/wiacode-shape-lab/outline-lab/pin-battery.js` 가 정면·카메라흉내 × cellPx 6/10
   *     네 조건에서 전부 해독(O)되는 조합만 골라 채운다. 손으로 추측해 넣지 말 것.
   *   ★대상 선정 근거는 B1 인지 게이트(`~/wiacode-shape-lab/recog/`) — 4표 전원 적중한 23종은
   *     넣지 않는다(고칠 게 없는데 그림만 바뀐다).
   */
  var OUTLINE_PINS = {
    // B1 인지 게이트에서 4표 중 0~2표만 적중한 형상 (2026-09-01 배터리 확정, 전부 OOOO)
    hearingaid: [1, 1], dove: [1, 1], clover: [1, 1], whale: [1, 1], lovesign: [1, 1],
    turtle: [1, 1], bone: [1, 1], piano: [1, 1], nose: [1, 1], cat: [1, 1],
    grandpiano: [1, 1], fist: [1, 1], pestbug: [1, 1], fish: [1, 1], horse: [1, 1],
    // B2-e 인지 게이트(2026-09-05)에서 4표 중 0~2표만 적중한 신규 배치 6종. 전부 OOOO.
    //   ★스트로크가 실제로 뒤집은 건 **helmet·harness 둘**이다(재측정 실측):
    //     helmet  1/4 → 3/4  (hat·shell·bear → helmet·helmet·helmet)
    //     harness 0/4 → 2/4  (rabbit·rabbit·scissors·bunny → harness·harness·overalls·bowtie)
    //   나머지 badge·fan·dustmask·safetycone 은 이득이 판정 잡음 안이다 —
    //   기존 15종 때와 같은 처리(핀은 두고 B2-b 재제작 후보로 표시).
    // ★`goggles` 는 뺐다 — 후보 6개(1/1·1/1.5·1.5/1·1/2·1.5/1.5·2/1)가 **전부** c6 카메라흉내에서
    //   락만 잡히고 해독이 죽는다(기준선은 OOOO). 인지 후보 1순위였는데 스트로크를 못 쓴다.
    // ★`perfume` 은 뺐다 — 두 가지가 겹쳤다: ①인지가 **나빠졌다**(1/4 → 0/4, 네 판정자 전부
    //   용기류 jar·bottle → 금속부품류 speaker·buckle·padlock·lock 으로 이동) ②무지개 배터리에서
    //   **yaw30 만 락(OOOLOOOL)** — 핀 배터리는 yaw30 을 안 재서 못 잡았다(B2-c 의 piano·fish·whale 과 같은 계열).
    helmet: [1, 1], dustmask: [1, 1],
    badge: [1, 1], fan: [1, 1], harness: [1, 1], safetycone: [1, 1]
    // ★boomerang 은 뺐다 — 표시 이름이 "반짝임/Sparkle" 이고 판정자들이 그대로 맞혔다.
    //   (id 가 boomerang 이라 처음엔 실패로 셌던 채점기 버그. id≠표시이름을 잊지 말 것.)
  };
  function outlinePin(id) {
    var v = OUTLINE_PINS[id];
    return (v && v.length === 2) ? { gap: v[0], w: v[1] } : null;
  }

  /* ── B2-d 무지개 외곽선의 형상별 밝기 상한 (2026-09-04) ──────────────────────
   *  무지개 스트로크는 기하(gap,w)가 검정과 **완전히 같고 색만 다르다**. 그런데도 해독이
   *  달라질 수 있다 — 검정은 Y=0, 무지개는 Y 70~96 이라 **적응형 이진화의 지역 평균**이
   *  경계에서 달라지고, 안티앨리어싱된 도트가 뒤집힌다. (해자(gap)가 필요한 이유와 같은
   *  메커니즘인데 방향이 반대다.)
   *
   *  ★실측(`~/wiacode-rainbow/rainbow-battery.js`, clean·blur·noise·yaw30 × cellPx 6/10):
   *    15종 중 **동일 12 · 무지개가 더 좋음 2(lovesign·piano) · 나쁨 1(whale)**.
   *    whale 만 maxY 스윕에서 절벽이 가팔랐다 — Y24 는 검정과 동일(OOOOOOOO)인데
   *    **Y40 부터 깨진다**(OLOOLLOO). 그래서 whale 만 24 로 내린다(거의 검정에 가까운 색조).
   *
   *  ★여기 없는 형상은 기본 상한(geometry.js RAINBOW_MAX_Y = 96)을 쓴다.
   *    ★★OUTLINE_PINS 에 형상을 새로 추가하면 **무지개 배터리도 같이 돌릴 것** —
   *      검정에서 통과했다고 무지개가 통과한다는 보장이 없다(whale 이 그 반례다). */
  var RAINBOW_MAX_Y = { whale: 24 };
  function rainbowMaxY(id) {
    var v = RAINBOW_MAX_Y[id];
    return (typeof v === 'number') ? v : null;
  }

  return {
    ALL: SHAPES,
    OUTLINE_PINS: OUTLINE_PINS,
    outlinePin: outlinePin,
    RAINBOW_MAX_Y: RAINBOW_MAX_Y,
    rainbowMaxY: rainbowMaxY,
    HUG_PINS: HUG_PINS,
    hugPin: hugPin,
    HUG_ON: HUG_ON,
    hugOn: hugOn,
    byId: byId,
    ids: function () { return ids(SHAPES); },
    current: current,
    rank: rank,
    currentIds: function () { return ids(current()); },
    decodable: decodable,
    decodableIds: function () { return ids(decodable()); },
    nonSquareDecodable: nonSquareDecodable,
    countable: countable,
    names: names,
    isCurrent: function (id) { return !!byId[id] && byId[id].status === 'current'; },
    isDecodable: function (id) { return !!byId[id]; },
    qrCapFor: qrCapFor,
    qrTable: QR_SAME_FOOTPRINT,
  };
});
