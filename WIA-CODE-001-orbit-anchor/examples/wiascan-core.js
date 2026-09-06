/* WIA Code Scanner Engine — 자동생성(build-core.js). 편집 금지. */
(function(){
"use strict";
var __mods={};
function require(name){var k=name.replace(/^.*\//,"").replace(/\.js$/,"");if(!(k in __mods))throw new Error("mod?"+k);return __mods[k];}
function __def(name,fn){var module={exports:{}};fn(module,module.exports);__mods[name]=module.exports;}

__def("shape-registry", function(module, exports){
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

});

__def("degrade", function(module, exports){
'use strict';
/*
 * ============================================================================
 *  WIA Code v2 — 계측된 열화 라이브러리
 * ============================================================================
 *  v1/tools/robustness-harness.js 의 열화 수식(회전·원근기울기·블러·저해상도·
 *  노이즈)을 바이트 단위로 계승한다. 차이는 단 하나:
 *    각 열화가 {img, fwd} 를 반환한다. fwd(x,y) = 원본픽셀 → 열화픽셀 정변환.
 *  이 정변환으로 "정답 앵커 중심이 열화 후 어디에 있는지"를 알 수 있고,
 *  검출 중심과의 재투영 오차(모듈 단위)를 잴 수 있다. v1 하네스는 이진
 *  디코드 성공만 재서 이 값을 낼 수 없었다 — 그게 이 파일의 존재 이유다.
 * ============================================================================
 */

// ── 시드 RNG (하네스와 동일 LCG/Box-Muller) ──────────────────────────────
function makeGauss(seed) {
  let s = (seed >>> 0) || 0x9e3779b9;
  const rnd = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 0x100000000; };
  return (sigma) => { const u = Math.max(1e-9, rnd()), v = rnd();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v) * sigma; };
}

function clone(img) { return { data: new Uint8ClampedArray(img.data), width: img.width, height: img.height }; }
function blank(w, h) { const d = new Uint8ClampedArray(w * h * 4).fill(255); return { data: d, width: w, height: h }; }
const IDENTITY = (x, y) => [x, y];

// 바이리니어 샘플 (경계 밖 = 흰색)
function sample(img, x, y) {
  const { data, width: w, height: h } = img;
  if (x < 0 || y < 0 || x > w - 1 || y > h - 1) return [255, 255, 255];
  const x0 = Math.floor(x), y0 = Math.floor(y), x1 = Math.min(x0 + 1, w - 1), y1 = Math.min(y0 + 1, h - 1);
  const fx = x - x0, fy = y - y0, out = [0, 0, 0];
  for (let c = 0; c < 3; c++) {
    const p00 = data[(y0 * w + x0) * 4 + c], p10 = data[(y0 * w + x1) * 4 + c];
    const p01 = data[(y1 * w + x0) * 4 + c], p11 = data[(y1 * w + x1) * 4 + c];
    out[c] = (p00 * (1 - fx) + p10 * fx) * (1 - fy) + (p01 * (1 - fx) + p11 * fx) * fy;
  }
  return out;
}

// ── 호모그래피 (8×8 가우스 소거) ─────────────────────────────────────────
function solve8(A, b) {
  const n = 8;
  for (let i = 0; i < n; i++) {
    let piv = i; for (let r = i + 1; r < n; r++) if (Math.abs(A[r][i]) > Math.abs(A[piv][i])) piv = r;
    [A[i], A[piv]] = [A[piv], A[i]]; [b[i], b[piv]] = [b[piv], b[i]];
    const d = A[i][i] || 1e-12;
    for (let c = i; c < n; c++) A[i][c] /= d; b[i] /= d;
    for (let r = 0; r < n; r++) { if (r === i) continue; const f = A[r][i]; for (let c = i; c < n; c++) A[r][c] -= f * A[i][c]; b[r] -= f * b[i]; }
  }
  return b;
}
// H mapping A[i] → B[i] (평면 4점 대응)
function homography(A4, B4) {
  const M = [], v = [];
  for (let i = 0; i < 4; i++) {
    const [X, Y] = A4[i], [x, y] = B4[i];
    M.push([X, Y, 1, 0, 0, 0, -X * x, -Y * x]); v.push(x);
    M.push([0, 0, 0, X, Y, 1, -X * y, -Y * y]); v.push(y);
  }
  const h = solve8(M, v); return [h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], 1];
}
function applyH(H, X, Y) { const d = H[6] * X + H[7] * Y + H[8]; return [(H[0] * X + H[1] * Y + H[2]) / d, (H[3] * X + H[4] * Y + H[5]) / d]; }

// ── 4-코너 워프 (원근) ────────────────────────────────────────────────────
//   dstCorners = 출력에서 코드 네 모서리가 놓일 위치. 캔버스 = 원본 크기.
function warpQuad(img, dstCorners) {
  const w = img.width, h = img.height;
  const srcCorners = [[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]];
  const Hout2src = homography(dstCorners, srcCorners);   // 렌더용: out→src
  const Hsrc2out = homography(srcCorners, dstCorners);   // 정변환: src→out
  const out = blank(w, h);
  for (let Y = 0; Y < h; Y++) for (let X = 0; X < w; X++) {
    const [sx, sy] = applyH(Hout2src, X, Y);
    const px = sample(img, sx, sy);
    const o = (Y * w + X) * 4; out.data[o] = px[0]; out.data[o + 1] = px[1]; out.data[o + 2] = px[2]; out.data[o + 3] = 255;
  }
  return { img: out, fwd: (x, y) => applyH(Hsrc2out, x, y), width: w, height: h };
}

// ── 원근 기울기(yaw) — 오른쪽 변을 뒤로 ──────────────────────────────────
function tilt(img, deg) {
  if (deg === 0) return { img: clone(img), fwd: IDENTITY, width: img.width, height: img.height };
  const w = img.width, h = img.height, s = Math.sin(deg * Math.PI / 180);
  const shrink = 1 - 0.9 * Math.abs(s), dx = 0.5 * Math.abs(s) * w;
  const top = h * (1 - shrink) / 2, bot = h - top;
  const dst = [[0, 0], [w - 1 - dx, top], [w - 1 - dx, bot], [0, h - 1]];
  return warpQuad(img, dst);
}

// ── 면내 회전(roll) — 캔버스 확장 ────────────────────────────────────────
function rotate(img, deg) {
  if (deg === 0) return { img: clone(img), fwd: IDENTITY, width: img.width, height: img.height };
  const w = img.width, h = img.height, a = deg * Math.PI / 180;
  const ca = Math.cos(-a), sa = Math.sin(-a);                 // 역변환용
  const W2 = Math.ceil(w * Math.abs(Math.cos(a)) + h * Math.abs(Math.sin(a))) + 8;
  const H2 = Math.ceil(w * Math.abs(Math.sin(a)) + h * Math.abs(Math.cos(a))) + 8;
  const cx = (w - 1) / 2, cy = (h - 1) / 2, CX = (W2 - 1) / 2, CY = (H2 - 1) / 2;
  const out = blank(W2, H2);
  for (let Y = 0; Y < H2; Y++) for (let X = 0; X < W2; X++) {
    const dx = X - CX, dy = Y - CY;
    const px = sample(img, cx + dx * ca - dy * sa, cy + dx * sa + dy * ca);
    const o = (Y * W2 + X) * 4; out.data[o] = px[0]; out.data[o + 1] = px[1]; out.data[o + 2] = px[2]; out.data[o + 3] = 255;
  }
  // 정변환: src(x,y) → out.  [X-CX;Y-CY] = R(a)[x-cx;y-cy]
  const cA = Math.cos(a), sA = Math.sin(a);
  const fwd = (x, y) => { const dx = x - cx, dy = y - cy; return [CX + dx * cA - dy * sA, CY + dx * sA + dy * cA]; };
  return { img: out, fwd, width: W2, height: H2 };
}

// ── 블러 (분리형 박스 × passes, 가우시안 근사) — 기하 불변 ────────────────
/* ★2026-08-30 — **정수 반경만 유효하다.** 소수를 넣으면 조용히 엉뚱한 화소를 읽는다.
 *   `for (let k = -radius; k <= radius; k++)` 에서 k 가 소수가 되고,
 *   세로 패스의 `o = (yy*w + x)*4` 에서 yy 가 소수면 오프셋이 **반 줄**이 된다 —
 *   w=640 이면 k=-0.5 가 "0.5줄 위"가 아니라 **같은 줄에서 320화소 왼쪽**을 읽는다.
 *   실측 피해: 내가 blur 1.5 로 시험하고 "전 실루엣이 blur1.5 에서 실패한다"는
 *   **없는 절벽을 보고했다**(제대로 된 가우시안으로 재니 시그마 0.58 모듈까지 해독된다).
 *   조용히 망가지는 게 가장 나쁘므로 반올림해서 **정의된 동작**으로 만든다.
 *   ★소수 시그마가 필요하면 boxBlur 가 아니라 진짜 가우시안을 쓸 것. */
function boxBlur(img, radius, passes) {
  radius = Math.round(radius);
  if (radius <= 0) return { img: clone(img), fwd: IDENTITY, width: img.width, height: img.height };
  let cur = clone(img); const w = img.width, h = img.height;
  for (let p = 0; p < (passes || 3); p++) {
    const tmp = blank(w, h);
    for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
      let r = 0, g = 0, b = 0, n = 0;
      for (let k = -radius; k <= radius; k++) { const xx = x + k; if (xx < 0 || xx >= w) continue; const o = (y * w + xx) * 4; r += cur.data[o]; g += cur.data[o + 1]; b += cur.data[o + 2]; n++; }
      const o = (y * w + x) * 4; tmp.data[o] = r / n; tmp.data[o + 1] = g / n; tmp.data[o + 2] = b / n; tmp.data[o + 3] = 255;
    }
    const tmp2 = blank(w, h);
    for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
      let r = 0, g = 0, b = 0, n = 0;
      for (let k = -radius; k <= radius; k++) { const yy = y + k; if (yy < 0 || yy >= h) continue; const o = (yy * w + x) * 4; r += tmp.data[o]; g += tmp.data[o + 1]; b += tmp.data[o + 2]; n++; }
      const o = (y * w + x) * 4; tmp2.data[o] = r / n; tmp2.data[o + 1] = g / n; tmp2.data[o + 2] = b / n; tmp2.data[o + 3] = 255;
    }
    cur = tmp2;
  }
  return { img: cur, fwd: IDENTITY, width: w, height: h };
}

// ── 저해상도 (축소→복원) — 기하 불변 ─────────────────────────────────────
function resample(img, factor) {
  if (factor >= 0.999) return { img: clone(img), fwd: IDENTITY, width: img.width, height: img.height };
  const w = img.width, h = img.height;
  const sw = Math.max(8, Math.round(w * factor)), sh = Math.max(8, Math.round(h * factor));
  const small = blank(sw, sh);
  for (let y = 0; y < sh; y++) for (let x = 0; x < sw; x++) {
    const px = sample(img, x / (sw - 1) * (w - 1), y / (sh - 1) * (h - 1));
    const o = (y * sw + x) * 4; small.data[o] = px[0]; small.data[o + 1] = px[1]; small.data[o + 2] = px[2]; small.data[o + 3] = 255;
  }
  const out = blank(w, h);
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    const px = sample(small, x / (w - 1) * (sw - 1), y / (h - 1) * (sh - 1));
    const o = (y * w + x) * 4; out.data[o] = px[0]; out.data[o + 1] = px[1]; out.data[o + 2] = px[2]; out.data[o + 3] = 255;
  }
  return { img: out, fwd: IDENTITY, width: w, height: h };
}

// ── 가우시안 노이즈 — 기하 불변 ──────────────────────────────────────────
function noise(img, sigma, seed) {
  if (sigma <= 0) return { img: clone(img), fwd: IDENTITY, width: img.width, height: img.height };
  const g = makeGauss(seed || 12345), out = clone(img);
  for (let i = 0; i < out.data.length; i += 4) {
    const n = g(sigma);
    out.data[i] = Math.max(0, Math.min(255, out.data[i] + n));
    out.data[i + 1] = Math.max(0, Math.min(255, out.data[i + 1] + n));
    out.data[i + 2] = Math.max(0, Math.min(255, out.data[i + 2] + n));
  }
  return { img: out, fwd: IDENTITY, width: img.width, height: img.height };
}

// ── 크로마 서브샘플(JPEG 4:2:0 근사) — 색압축의 벽 실측용 ─────────────────
//   YCbCr 변환 → Cb/Cr 을 blk×blk 박스평균(Y 는 유지) → RGB. blk=2 가 4:2:0.
function chromaSubsample(img, blk) {
  blk = blk || 2; const w = img.width, h = img.height, d = img.data, out = clone(img);
  const Y = new Float32Array(w * h), Cb = new Float32Array(w * h), Cr = new Float32Array(w * h);
  for (let i = 0, j = 0; i < d.length; i += 4, j++) {
    const R = d[i], G = d[i + 1], B = d[i + 2];
    Y[j] = 0.299 * R + 0.587 * G + 0.114 * B;
    Cb[j] = -0.168736 * R - 0.331264 * G + 0.5 * B;
    Cr[j] = 0.5 * R - 0.418688 * G - 0.081312 * B;
  }
  for (let by = 0; by < h; by += blk) for (let bx = 0; bx < w; bx += blk) {
    let sb = 0, sr = 0, n = 0;
    for (let y = by; y < Math.min(h, by + blk); y++) for (let x = bx; x < Math.min(w, bx + blk); x++) { sb += Cb[y * w + x]; sr += Cr[y * w + x]; n++; }
    const mb = sb / n, mr = sr / n;
    for (let y = by; y < Math.min(h, by + blk); y++) for (let x = bx; x < Math.min(w, bx + blk); x++) { Cb[y * w + x] = mb; Cr[y * w + x] = mr; }
  }
  for (let j = 0, i = 0; j < w * h; j++, i += 4) {
    const y = Y[j], cb = Cb[j], cr = Cr[j];
    out.data[i] = Math.max(0, Math.min(255, y + 1.402 * cr));
    out.data[i + 1] = Math.max(0, Math.min(255, y - 0.344136 * cb - 0.714136 * cr));
    out.data[i + 2] = Math.max(0, Math.min(255, y + 1.772 * cb));
  }
  return { img: out, fwd: IDENTITY, width: w, height: h };
}

module.exports = { tilt, rotate, boxBlur, resample, noise, clone, blank, sample, homography, applyH, chromaSubsample };

});

__def("frst", function(module, exports){
'use strict';
/*
 * ============================================================================
 *  WIA Code v2 — Fast Radial Symmetry Transform (FRST) 검출기
 * ============================================================================
 *  Fable 5 설계의 심장. QR의 스캔라인 런-길이(점 검출)를 버리고, 방사대칭
 *  적분 검출로 코어/위성 중심을 찾는다.
 *
 *  원리(Loy & Zelinsky 2003 변형):
 *    - 어두운 대칭 중심(코어 중심·위성 원판은 검정)은, 경계 그레이디언트가
 *      바깥(어두움→밝음)을 향하므로, 각 에지픽셀에서 반대방향으로 반경 n 떨어진
 *      점(p - n·ĝ)에 투표하면 중심에 표가 쌓인다.
 *    - 여러 반경 n에 걸쳐 누적 → 불스아이(다중 링)에 표가 압도적으로 몰린다.
 *    - 원주 O(2πr)개 픽셀이 한 점에 투표 → 가우시안 노이즈가 √N로 평균화.
 *      (QR 런-길이의 얇은 점 검출과 근본적으로 다른 노이즈 마진)
 *
 *  입력은 그레이스케일. 출력은 대칭맵 S와 극대점(후보 중심).
 * ============================================================================
 */

// RGBA → 그레이 (Float32, 0..255)
function toGray(img) {
  const { data, width: w, height: h } = img;
  const g = new Float32Array(w * h);
  for (let i = 0, j = 0; i < data.length; i += 4, j++) g[j] = 0.299 * data[i] + 0.587 * data[i + 1] + 0.114 * data[i + 2];
  return { g, w, h };
}

// Sobel 그레이디언트
function sobel(gray) {
  const { g, w, h } = gray;
  const gx = new Float32Array(w * h), gy = new Float32Array(w * h), mag = new Float32Array(w * h);
  let maxMag = 1e-6;
  for (let y = 1; y < h - 1; y++) for (let x = 1; x < w - 1; x++) {
    const i = y * w + x;
    const a = g[i - w - 1], b = g[i - w], c = g[i - w + 1];
    const d = g[i - 1],                 f = g[i + 1];
    const p = g[i + w - 1], q = g[i + w], r = g[i + w + 1];
    const sx = (c + 2 * f + r) - (a + 2 * d + p);
    const sy = (p + 2 * q + r) - (a + 2 * b + c);
    gx[i] = sx; gy[i] = sy; const m = Math.hypot(sx, sy); mag[i] = m; if (m > maxMag) maxMag = m;
  }
  return { gx, gy, mag, maxMag, w, h };
}

// 3×3 박스블러 × passes (대칭맵 평활)
function smooth(buf, w, h, passes) {
  let cur = buf;
  for (let p = 0; p < (passes || 1); p++) {
    const out = new Float32Array(w * h);
    for (let y = 1; y < h - 1; y++) for (let x = 1; x < w - 1; x++) {
      const i = y * w + x;
      out[i] = (cur[i] + cur[i - 1] + cur[i + 1] + cur[i - w] + cur[i + w] +
                cur[i - w - 1] + cur[i - w + 1] + cur[i + w - 1] + cur[i + w + 1]) / 9;
    }
    cur = out;
  }
  return cur;
}

/*
 * frst(gray, radii, opts) → { S, w, h }
 *   radii  : 투표 반경 배열(px). 코어 링 반경 + 위성 반경을 커버하도록.
 *   opts.gradFrac : 그레이디언트 임계 (maxMag 대비, 기본 0.15)
 *   opts.kappa    : 정규화 스케일 (기본 9.9)
 *   opts.alpha    : 방사 강도 지수 (기본 2)
 *   opts.polarity : 'dark'(기본)=어두운 중심만. 'both'=양극성.
 */
function frst(grayObj, radii, opts) {
  opts = opts || {};
  const { g, w, h } = grayObj;
  const sob = sobel(grayObj);
  const thr = (opts.gradFrac != null ? opts.gradFrac : 0.15) * sob.maxMag;
  const kappa = opts.kappa || 9.9, alpha = opts.alpha != null ? opts.alpha : 2;
  const dark = (opts.polarity || 'dark') !== 'both';
  const S = new Float32Array(w * h);

  for (const n of radii) {
    const O = new Float32Array(w * h);   // 방향(카운트) 누적
    const M = new Float32Array(w * h);   // 크기 누적
    for (let y = 1; y < h - 1; y++) for (let x = 1; x < w - 1; x++) {
      const i = y * w + x, m = sob.mag[i];
      if (m < thr) continue;
      const ux = sob.gx[i] / m, uy = sob.gy[i] / m;
      // 어두운 중심 → 반대방향(-n) 에 투표
      const nx = Math.round(x - n * ux), ny = Math.round(y - n * uy);
      if (nx >= 0 && nx < w && ny >= 0 && ny < h) { const j = ny * w + nx; O[j] += 1; M[j] += m; }
      if (!dark) { // 양극성: 순방향(+n)에도 음의 투표
        const px = Math.round(x + n * ux), py = Math.round(y + n * uy);
        if (px >= 0 && px < w && py >= 0 && py < h) { const j = py * w + px; O[j] -= 1; M[j] -= m; }
      }
    }
    // 정규화 + 방사 강도. 반경 n에 비례한 약한 평활(투표 산포 흡수).
    const blurPass = Math.max(1, Math.round(n / 6));
    const Ob = smooth(O, w, h, blurPass), Mb = smooth(M, w, h, blurPass);
    for (let idx = 0; idx < w * h; idx++) {
      let o = Ob[idx]; if (dark && o < 0) o = 0; else o = Math.abs(o);
      const on = Math.min(o, kappa) / kappa;
      S[idx] += Math.pow(on, alpha) * (Mb[idx] > 0 ? Mb[idx] : 0);
    }
  }
  // 최종 평활
  return { S: smooth(S, w, h, 2), w, h };
}

// 극대점 추출 (비최대억제). win=억제 반경(px). 반환: [{x,y,score}] 내림차순.
function peaks(Sobj, opts) {
  opts = opts || {};
  const { S, w, h } = Sobj;
  let max = 1e-9; for (let i = 0; i < S.length; i++) if (S[i] > max) max = S[i];
  const thr = (opts.thrFrac != null ? opts.thrFrac : 0.12) * max;
  const win = opts.win || 8, topK = opts.topK || 24;
  const cand = [];
  for (let y = win; y < h - win; y++) for (let x = win; x < w - win; x++) {
    const v = S[y * w + x]; if (v < thr) continue;
    let isMax = true;
    for (let dy = -win; dy <= win && isMax; dy++) for (let dx = -win; dx <= win; dx++) {
      if (dx === 0 && dy === 0) continue;
      if (S[(y + dy) * w + (x + dx)] > v) { isMax = false; break; }
    }
    if (isMax) cand.push({ x, y, score: v });
  }
  cand.sort((a, b) => b.score - a.score);
  // 근접 중복 제거 + 포물선 서브픽셀 정밀화(far쪽 압축 영역에서 정수 양자화 오차 제거)
  const out = [];
  for (const c of cand) {
    if (out.some(o => Math.hypot(o.x - c.x, o.y - c.y) < win)) continue;
    const i = c.y * w + c.x;
    const sx = subpix(S[i - 1], S[i], S[i + 1]);
    const sy = subpix(S[i - w], S[i], S[i + w]);
    out.push({ x: c.x + sx, y: c.y + sy, score: c.score });
    if (out.length >= topK) break;
  }
  return out;
}
// 1D 포물선 정점 오프셋 (-0.5..0.5)
function subpix(a, b, c) {
  const d = a - 2 * b + c;
  if (Math.abs(d) < 1e-9) return 0;
  const o = 0.5 * (a - c) / d;
  return o > 0.5 ? 0.5 : o < -0.5 ? -0.5 : o;
}

// 방사 강도 프로파일: 중심에서 바깥으로 R까지 그레이 평균(각도 평균). 링 구조 판정용.
function radialProfile(grayObj, cx, cy, R, steps) {
  const { g, w, h } = grayObj; steps = steps || Math.max(8, Math.round(R));
  const prof = new Float32Array(steps);
  for (let s = 0; s < steps; s++) {
    const r = (s + 0.5) / steps * R; let sum = 0, n = 0;
    const na = Math.max(8, Math.round(2 * Math.PI * r));
    for (let a = 0; a < na; a++) {
      const th = 2 * Math.PI * a / na, x = cx + r * Math.cos(th), y = cy + r * Math.sin(th);
      if (x < 0 || y < 0 || x >= w - 1 || y >= h - 1) continue;
      const x0 = Math.floor(x), y0 = Math.floor(y), fx = x - x0, fy = y - y0, i = y0 * w + x0;
      sum += (g[i] * (1 - fx) + g[i + 1] * fx) * (1 - fy) + (g[i + w] * (1 - fx) + g[i + w + 1] * fx) * fy; n++;
    }
    prof[s] = n ? sum / n : 255;
  }
  return prof;
}

/*
 * 암부 무게중심 정밀화: (cx,cy) 주위 반경 R 창에서, 배경보다 어두운 픽셀의
 * 무게중심을 구한다. 투영된 균일 원판(위성)·대칭 구조(코어)의 밝기 무게중심은
 * 아핀 불변으로 참 투영중심과 일치 → 원근에서 원형-FRST 무게중심 편향을 제거.
 * 2패스: ①창 평균 ②(평균-그레이)를 가중치로 무게중심.
 */
function refineCentroid(grayObj, cx, cy, R, bright) {
  const { g, w, h } = grayObj;
  const x0 = Math.max(1, Math.floor(cx - R)), x1 = Math.min(w - 2, Math.ceil(cx + R));
  const y0 = Math.max(1, Math.floor(cy - R)), y1 = Math.min(h - 2, Math.ceil(cy + R));
  if (x1 <= x0 || y1 <= y0) return { x: cx, y: cy, ok: false };
  let sum = 0, n = 0;
  for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) {
    if (Math.hypot(x - cx, y - cy) > R) continue;
    sum += g[y * w + x]; n++;
  }
  if (!n) return { x: cx, y: cy, ok: false };
  const mean = sum / n;
  let wsum = 0, sx = 0, sy = 0;
  for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) {
    if (Math.hypot(x - cx, y - cy) > R) continue;
    // bright=true → 밝을수록 큰 가중(도넛 흰 구멍), 기본 → 어두울수록(원판/링)
    const ww = bright ? (g[y * w + x] - mean) : (mean - g[y * w + x]);
    if (ww <= 0) continue;
    wsum += ww; sx += ww * x; sy += ww * y;
  }
  if (wsum <= 0) return { x: cx, y: cy, ok: false };
  return { x: sx / wsum, y: sy / wsum, ok: true };
}

module.exports = { toGray, sobel, frst, peaks, radialProfile, refineCentroid };

});

__def("conic", function(module, exports){
'use strict';
/*
 * ============================================================================
 *  WIA Code v2 — 코어 동심원(conic-pencil) 원근복원
 * ============================================================================
 *  설계문서 §1.2-2 구현. 급격한 원근(yaw≥30°)에서 코너 위성이 타원으로 압축·
 *  소실해 `no-core-surround`로 죽던 tail을, 코어 하나만으로 연다.
 *
 *  원리(고전 결과, MaxiCode·동심원 포즈추정 계보):
 *    - 세계의 원(circle)은 사영변환에서 타원(conic)으로 간다. 코어는 동심원
 *      불스아이 → 이미지에선 "동심 타원 2개"(반경 1.5모듈 원판edge, 5.5모듈
 *      바깥edge)로 관측된다. 코어는 원근에서도 반석(0.05모듈)이라 항상 잡힌다.
 *    - 두 이미지 타원 C1,C2 의 conic-pencil C1-λC2 에는 rank-1 퇴화원뿔이 하나
 *      있고, 그것이 = (소실선)². 즉 코어만으로 평면의 소실선을 닫힌형으로 얻는다.
 *    - 코어 참중심(원의 중심 이미지) = 소실선의 극(pole) = adj(C)·l.
 *      (타원의 무게중심이 아니다 — 원근에서 그 둘은 다르다.)
 *    - 소실선+원뿔 하나로 이미지를 similarity(회전·스케일·평행이동)만 남기고
 *      정면화(metric rectify)한다. 회전/스케일 잔여 자유도는 정면화된 그림에서
 *      위성(이제 원형 복원)·북극성으로 확정 → 기존 locate() 재사용.
 *
 *  이 파일은 순수 기하(픽셀→픽셀). 데이터 코덱과 무관.
 * ============================================================================
 */

// ── 선형대수 (작은 밀집행렬) ───────────────────────────────────────────────

// 대칭 n×n 고유분해 (cyclic Jacobi). 반환 { val:[..], vec:[[..]..] (열=고유벡터) }.
function jacobiEig(Ain, n) {
  const A = Ain.map(r => Float64Array.from(r));
  const V = Array.from({ length: n }, (_, i) => { const r = new Float64Array(n); r[i] = 1; return r; });
  for (let sweep = 0; sweep < 100; sweep++) {
    // off-diagonal 크기
    let off = 0;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) off += A[p][q] * A[p][q];
    if (off < 1e-24) break;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) {
      const apq = A[p][q];
      if (Math.abs(apq) < 1e-300) continue;
      const app = A[p][p], aqq = A[q][q];
      const phi = 0.5 * Math.atan2(2 * apq, aqq - app);
      const c = Math.cos(phi), s = Math.sin(phi);
      for (let k = 0; k < n; k++) {
        const akp = A[k][p], akq = A[k][q];
        A[k][p] = c * akp - s * akq; A[k][q] = s * akp + c * akq;
      }
      for (let k = 0; k < n; k++) {
        const apk = A[p][k], aqk = A[q][k];
        A[p][k] = c * apk - s * aqk; A[q][k] = s * apk + c * aqk;
      }
      for (let k = 0; k < n; k++) {
        const vkp = V[k][p], vkq = V[k][q];
        V[k][p] = c * vkp - s * vkq; V[k][q] = s * vkp + c * vkq;
      }
    }
  }
  const val = new Array(n), vec = Array.from({ length: n }, () => new Float64Array(n));
  for (let i = 0; i < n; i++) { val[i] = A[i][i]; for (let k = 0; k < n; k++) vec[k][i] = V[k][i]; }
  return { val, vec };
}

// 3×3 역행렬
function inv3(m) {
  const [a, b, c, d, e, f, g, h, i] = [m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2]];
  const A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
  const det = a * A + b * B + c * C;
  if (Math.abs(det) < 1e-300) return null;
  const id = 1 / det;
  return [
    [A * id, (c * h - b * i) * id, (b * f - c * e) * id],
    [B * id, (a * i - c * g) * id, (c * d - a * f) * id],
    [C * id, (b * g - a * h) * id, (a * e - b * d) * id],
  ];
}

// 3×3 수반행렬(adjugate) = det·inv. pole 계산에 det 무관하게 쓸 수 있어 안전.
function adj3(m) {
  const a = m[0][0], b = m[0][1], c = m[0][2], d = m[1][0], e = m[1][1], f = m[1][2], g = m[2][0], h = m[2][1], i = m[2][2];
  return [
    [e * i - f * h, c * h - b * i, b * f - c * e],
    [f * g - d * i, a * i - c * g, c * d - a * f],
    [d * h - e * g, b * g - a * h, a * e - b * d],
  ];
}
function mul3(A, B) {
  const C = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) { let s = 0; for (let k = 0; k < 3; k++) s += A[i][k] * B[k][j]; C[i][j] = s; }
  return C;
}
function matVec3(M, v) { return [M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2], M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2], M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2]]; }
function transpose3(M) { return [[M[0][0], M[1][0], M[2][0]], [M[0][1], M[1][1], M[2][1]], [M[0][2], M[1][2], M[2][2]]]; }
function frob(M) { let s = 0; for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) s += M[i][j] * M[i][j]; return Math.sqrt(s); }
function scale3(M, k) { return M.map(r => r.map(v => v * k)); }
function sub3(A, B) { return A.map((r, i) => r.map((v, j) => v - B[i][j])); }

// ── 코어 링 에지점 추출 ────────────────────────────────────────────────────
// 중심(cx,cy) 주위로 방사 스캔. 각 각도에서 바깥으로 나가며 그레이가
//   disk(검)→흰링→검링→흰 세퍼레이터. 두 개의 dark→light 상승에지(r≈1.5, ≈5.5
//   모듈)를 서브픽셀로 잡는다. 스케일 무지 상태에서 "첫 두 상승에지"라는 순서로 견고.
function extractCoreRings(grayObj, cx, cy, rMaxPx, nAng, gradFrac) {
  const { g, w, h } = grayObj;
  nAng = nAng || 180;
  gradFrac = gradFrac != null ? gradFrac : 0.10;   // 대비-상대 에지임계(과거 절대 40 대체)
  const sampAt = (x, y) => {
    if (x < 0 || y < 0 || x >= w - 1 || y >= h - 1) return null;
    const x0 = Math.floor(x), y0 = Math.floor(y), fx = x - x0, fy = y - y0, i = y0 * w + x0;
    return (g[i] * (1 - fx) + g[i + 1] * fx) * (1 - fy) + (g[i + w] * (1 - fx) + g[i + w + 1] * fx) * fy;
  };
  const step = 0.5;                       // 방사 샘플 간격(px)
  const inner = [], outer = [];
  for (let a = 0; a < nAng; a++) {
    const th = 2 * Math.PI * a / nAng, ct = Math.cos(th), st = Math.sin(th);
    // 프로파일 샘플. r=0.5 부터 — 과거 r=1 시작 + 국소최대 조건(k±2)이 만든
    //   "r<2.25px 검출 사각지대"가 yaw≥44° 압축방향에서 디스크 에지(1.5모듈≈2px)를
    //   통째로 놓쳐 첫 에지가 5.5링으로 밀리던 원인이었다(2026-08-28 실측).
    const rs = [], vs = [];
    for (let r = 0.5; r <= rMaxPx; r += step) { const v = sampAt(cx + r * ct, cy + r * st); if (v == null) break; rs.push(r); vs.push(v); }
    if (vs.length < 8) continue;
    let vmin = Infinity, vmax = -Infinity;
    for (let i = 0; i < vs.length; i++) { if (vs[i] < vmin) vmin = vs[i]; if (vs[i] > vmax) vmax = vs[i]; }
    const contrast = vmax - vmin;
    if (contrast < 12) continue;               // 평탄 프로파일 스킵
    // ── 슈미트 트리거(히스테리시스) 상승 반값교차 ─────────────────────────
    //   과거 "그래디언트 국소최대 + 첫 두 개" 방식의 두 파괴 모드를 원리적으로 봉쇄:
    //   ① 스미어된 에지 하나가 국소최대 2개로 쪼개져 (inner,outer)가 같은 에지에
    //      찍히던 이중검출 — 교차 사이에 lo 이하 하강이 필수라 불가능.
    //   ② 위 사각지대 누락 — 교차는 r=0.5부터 판정된다.
    //   대비-상대 임계(gradFrac 도입 취지)는 lo/hi 가 프로파일 자기 대비로 정해져 유지.
    const lo = vmin + 0.30 * contrast, hi = vmin + 0.70 * contrast, mid = vmin + 0.5 * contrast;
    // 시작점이 암부가 아니면 이 ray 는 코어 디스크 밖에서 출발한 것 —
    //   "첫 두 상승" 의미가 깨져 5.5링을 1.5링으로 오인한다. 버리는 게 정답.
    //   (시드가 코어 중심에서 1모듈 이상 벗어난 고yaw 에서 실제로 발생)
    if (vs[0] > mid) continue;
    let state = 0;                          // 0=암부, 1=명부
    const crossings = [];
    for (let k = 1; k < vs.length && crossings.length < 2; k++) {
      if (state === 0 && vs[k] >= hi) {
        // 직전 mid 상향교차를 서브픽셀(선형보간)로
        let j = k - 1; while (j > 0 && vs[j] >= mid) j--;
        const t = (mid - vs[j]) / ((vs[j + 1] - vs[j]) || 1);
        crossings.push(rs[j] + Math.max(0, Math.min(1, t)) * step);
        state = 1;
      } else if (state === 1 && vs[k] <= lo) state = 0;
    }
    if (crossings.length < 2) continue;
    // 첫 두 상승교차 = 원판경계(1.5모듈), 검은링 바깥경계(5.5모듈).
    inner.push([cx + crossings[0] * ct, cy + crossings[0] * st]);
    outer.push([cx + crossings[1] * ct, cy + crossings[1] * st]);
  }
  return { inner, outer };
}

// ── 각도평균 방사 프로파일 (극단블러용 배율추정 기반) ──────────────────────
// extractCoreRings 는 "각 각도마다 상승에지 2개"를 요구한다. 블러 σ가 링 폭(2모듈)에
// 근접하면 안쪽 흰 링이 먼저 소멸(MTF≈0) → 각도별로 에지가 1개만 남아 링 수집이
// 0개로 붕괴한다(실측: box r6·3pass 에서 180각 중 0~1개). 그러나 코어 전체(반경 5.5모듈
// 암부 블롭)의 바깥 경계는 훨씬 저주파라 훨씬 오래 산다. 각도평균(180각)은 SNR을 √180배
// 올려 그 경계를 블러 r10 까지 안정적으로 남긴다 — 각도별 국소 그래디언트 판정이 죽는
// 지점에서도. 그래서 "에지 개수 세기" 대신 "평균 프로파일의 마지막 상승 반값교차".
function meanRadialProfile(grayObj, cx, cy, rMaxPx, step) {
  const { g, w, h } = grayObj;
  step = step || 0.5;
  const sampAt = (x, y) => {
    if (x < 0 || y < 0 || x >= w - 1 || y >= h - 1) return null;
    const x0 = Math.floor(x), y0 = Math.floor(y), fx = x - x0, fy = y - y0, i = y0 * w + x0;
    return (g[i] * (1 - fx) + g[i + 1] * fx) * (1 - fy) + (g[i + w] * (1 - fx) + g[i + w + 1] * fx) * fy;
  };
  const nR = Math.max(2, Math.floor(rMaxPx / step) + 1);
  const P = new Float64Array(nR);
  let used = 0;
  for (let k = 0; k < nR; k++) {
    const r = k * step;
    const nA = Math.max(16, Math.min(240, Math.round(2 * Math.PI * r)));
    let s = 0, n = 0;
    for (let a = 0; a < nA; a++) {
      const th = 2 * Math.PI * a / nA;
      const v = sampAt(cx + r * Math.cos(th), cy + r * Math.sin(th));
      if (v != null) { s += v; n++; }
    }
    if (n < nA * 0.6) break;                 // 화면 밖으로 절반 이상 나가면 절단
    P[k] = s / n; used = k + 1;
  }
  return { P: P.subarray(0, used), step, n: used };
}

/*
 * estimateCoreScale — 사전지식 없이 코어 불스아이 바깥경계(5.5모듈)를 각도평균
 *   프로파일에서 찾아 cellPx 를 역산. extractCoreRings(각도별 에지수집)가 극단블러에서
 *   붕괴한 뒤의 폴백 겸, 코어/위성 판별자(위성 블롭반경 2.5모듈 vs 코어 5.5모듈).
 *   반환 { ok, cellPx, rOuter, contrast, centerDark }.
 */
function estimateCoreScale(grayObj, cx, cy, rMaxPx, opts) {
  opts = opts || {};
  const prof = meanRadialProfile(grayObj, cx, cy, rMaxPx, opts.step || 0.5);
  const P = prof.P, n = prof.n, step = prof.step;
  if (n < 8) return { ok: false, reason: 'short-profile' };
  const srt = Array.from(P).sort((a, b) => a - b);
  const vmin = srt[0], vhi = srt[Math.floor(0.9 * (srt.length - 1))];
  const contrast = vhi - vmin;
  if (contrast < (opts.minContrast != null ? opts.minContrast : 12)) return { ok: false, reason: 'flat', contrast };
  const thr = vmin + 0.5 * contrast;
  // 마지막(가장 바깥) 상승 반값교차 = 코어 암부 블롭의 바깥 경계.
  //   블러가 안쪽 흰 링을 지워도 이 교차는 남는다. 서브샘플 선형보간.
  let rOuter = NaN;
  for (let k = 1; k < n; k++) {
    if (P[k - 1] < thr && P[k] >= thr) {
      const t = (thr - P[k - 1]) / ((P[k] - P[k - 1]) || 1);
      rOuter = (k - 1 + t) * step;
    }
  }
  if (!isFinite(rOuter) || rOuter < step * 2) return { ok: false, reason: 'no-crossing', contrast };
  const cellPx = rOuter / 5.5;                    // 코어 바깥 링 = 5.5 모듈
  // 중심이 실제로 어두운가(밝은 blob 오검출 배제)
  const centerDark = (P[0] < thr);
  return { ok: centerDark && cellPx > 1.2 && cellPx < 60, cellPx, rOuter, contrast, centerDark, profile: P };
}

// ── 대수적 타원(원뿔) 적합 (Hartley 정규화 + 최소SV) ───────────────────────
// 점들 → conic 대칭행렬 C: [A,B/2,D/2; B/2,C,E/2; D/2,E/2,F] with A x²+B xy+C y²+D x+E y+F.
function fitConic(pts) {
  const n = pts.length;
  if (n < 6) return null;
  // Hartley 정규화 (수치안정)
  let mx = 0, my = 0; for (const p of pts) { mx += p[0]; my += p[1]; } mx /= n; my /= n;
  let s = 0; for (const p of pts) s += Math.hypot(p[0] - mx, p[1] - my); s /= n;
  const sc = s > 1e-9 ? Math.SQRT2 / s : 1;
  const T = [[sc, 0, -sc * mx], [0, sc, -sc * my], [0, 0, 1]];
  // 설계행렬 D (n×6), 정규화 좌표
  const rows = [];
  for (const p of pts) { const x = sc * (p[0] - mx), y = sc * (p[1] - my); rows.push([x * x, x * y, y * y, x, y, 1]); }
  // 6×6 스캐터
  const S = Array.from({ length: 6 }, () => new Float64Array(6));
  for (const r of rows) for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) S[i][j] += r[i] * r[j];
  const { val, vec } = jacobiEig(S, 6);
  // 최소 고유값의 고유벡터 = 계수
  let mi = 0; for (let i = 1; i < 6; i++) if (val[i] < val[mi]) mi = i;
  const c = vec.map(row => row[mi]);       // [A,B,C,D,E,F] (정규화좌표)
  // 타원 판정 (B²-4AC<0)
  const disc = c[1] * c[1] - 4 * c[0] * c[2];
  // 정규화좌표 conic 행렬
  const Cn = [[c[0], c[1] / 2, c[3] / 2], [c[1] / 2, c[2], c[4] / 2], [c[3] / 2, c[4] / 2, c[5]]];
  // 역정규화: C = Tᵀ Cn T
  const C = mul3(transpose3(T), mul3(Cn, T));
  return { C: scale3(C, 1 / (frob(C) || 1)), isEllipse: disc < 0, n };
}

// 3×3 행렬식
function det3(m) {
  return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
       - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
       + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}
// 실근 3차방정식 aλ³+bλ²+cλ+d=0. 실근 전부 반환(근접 이중근 포함).
//   한 실근을 Cardano/Newton으로 얻고 2차로 deflation → 나머지 두 실근 복구.
function solveQuadR(a, b, c) {
  if (Math.abs(a) < 1e-14) return Math.abs(b) < 1e-14 ? [] : [-c / b];
  const D = b * b - 4 * a * c; if (D < -1e-9 * (b * b + 1)) return [];
  const s = Math.sqrt(Math.max(0, D)); return [(-b + s) / (2 * a), (-b - s) / (2 * a)];
}
function solveCubic(a, b, c, d) {
  if (Math.abs(a) < 1e-14) return solveQuadR(b, c, d);
  const B = b / a, C = c / a, D = d / a;
  const p = C - B * B / 3, q = 2 * B * B * B / 27 - B * C / 3 + D;
  const disc = q * q / 4 + p * p * p / 27;
  let r1;
  if (disc > 0) {
    const s = Math.sqrt(disc);
    r1 = Math.cbrt(-q / 2 + s) + Math.cbrt(-q / 2 - s) - B / 3;
  } else {
    const r = Math.sqrt(Math.max(1e-30, -p * p * p / 27));
    const phi = Math.acos(Math.max(-1, Math.min(1, -q / 2 / r)));
    r1 = 2 * Math.cbrt(r) * Math.cos(phi / 3) - B / 3;
  }
  // Newton 한두 번(정밀화)
  for (let it = 0; it < 3; it++) {
    const f = ((r1 + B) * r1 + C) * r1 + D, fp = (3 * r1 + 2 * B) * r1 + C;
    if (Math.abs(fp) < 1e-14) break; r1 -= f / fp;
  }
  // deflation: λ³+Bλ²+Cλ+D = (λ-r1)(λ²+e1λ+e0)
  const e1 = B + r1, e0 = C + r1 * e1;
  const Dq = e1 * e1 - 4 * e0;
  // Dq<0 = fit 잡음으로 이중근이 복소화 → 실수부(-e1/2)를 이중근으로.
  const rest = Dq >= 0 ? [(-e1 + Math.sqrt(Dq)) / 2, (-e1 - Math.sqrt(Dq)) / 2] : [-e1 / 2, -e1 / 2];
  return [r1, ...rest].filter(isFinite);
}
// 4점 샘플로 det(C1-λC2)(=λ의 3차) 계수 복원
function pencilCubicCoeffs(C1, C2) {
  const xs = [-1, 0, 1, 2], ys = xs.map(l => det3(sub3(C1, scale3(C2, l))));
  // Vandermonde 4×4 해
  const M = xs.map(l => [l * l * l, l * l, l, 1]).map(r => Float64Array.from(r));
  const v = Float64Array.from(ys);
  for (let i = 0; i < 4; i++) {
    let p = i; for (let r = i + 1; r < 4; r++) if (Math.abs(M[r][i]) > Math.abs(M[p][i])) p = r;
    [M[i], M[p]] = [M[p], M[i]]; [v[i], v[p]] = [v[p], v[i]];
    const dg = M[i][i] || 1e-30;
    for (let r = 0; r < 4; r++) { if (r === i) continue; const f = M[r][i] / dg; for (let c = i; c < 4; c++) M[r][c] -= f * M[i][c]; v[r] -= f * v[i]; }
  }
  return [v[0] / M[0][0], v[1] / M[1][1], v[2] / M[2][2], v[3] / M[3][3]];
}

/*
 * 두 동심원 이미지(C1=바깥원, C2=안쪽원) → 소실선 l + 참중심.
 * 이론: 동심원 conic-pencil C1-λC2 의 det=0 근 3개 =
 *   · 이중근 λ=1  → M=rank1=(소실선)²           → l = M 의 지배고유벡터
 *   · 단일근 λ=r1²/r2² → M=rank2=중심 지나는 두 직선 → 중심 = M 의 널공간
 * 원근 왜곡을 완전히 견디는 닫힌형 해. (무게중심 정밀화가 원근서 편향되던 문제 근절)
 * 반환 { ok, l:[a,b,c], center:[x,y], single, lamVan }.
 */
function recoverFromConcentric(C1, C2) {
  const coef = pencilCubicCoeffs(C1, C2);
  // 계수 정규화(정규화 conic이라 절대크기가 그리드마다 미세). 상대스케일로 풀어야 함.
  const mx = Math.max(...coef.map(Math.abs)) || 1;
  const roots = solveCubic(coef[0] / mx, coef[1] / mx, coef[2] / mx, coef[3] / mx);
  if (roots.length < 3) return { ok: false, reason: 'roots<3', roots };
  roots.sort((a, b) => a - b);
  // 단일근 = 나머지 둘과 가장 멀리 떨어진 근. 이중근 = 나머지 둘(평균).
  let idx = 0, bestGap = -1;
  for (let i = 0; i < roots.length; i++) {
    const gap = Math.min(...roots.filter((_, j) => j !== i).map(o => Math.abs(o - roots[i])));
    if (gap > bestGap) { bestGap = gap; idx = i; }
  }
  const single = roots[idx];
  const pair = roots.filter((_, j) => j !== idx);
  const lamVan = pair.reduce((s, v) => s + v, 0) / pair.length;

  // 중심 = 단일근 M 의 널공간(최소 |고유값|의 고유벡터). 원근서도 0.02모듈 정밀.
  const Ms = sub3(C1, scale3(C2, single)), es = jacobiEig(Ms, 3);
  let ni = 0; for (let i = 1; i < 3; i++) if (Math.abs(es.val[i]) < Math.abs(es.val[ni])) ni = i;
  const cn = [es.vec[0][ni], es.vec[1][ni], es.vec[2][ni]];
  if (Math.abs(cn[2]) < 1e-12) return { ok: false, reason: 'center-at-inf' };
  const center = [cn[0] / cn[2], cn[1] / cn[2]];

  // 소실선 = 참중심의 (바깥 conic 기준) 극선 polar = C1·center.
  //   원의 중심의 극선은 무한선 → 이미지에선 소실선. 정밀 중심을 그대로 쓰므로
  //   이중근 고유벡터(잡음 민감)보다 견고(각도오차 <2°).
  let l = matVec3(C1, [center[0], center[1], 1]);
  // max-abs 정규화(무한선 근처=저yaw 에서 hypot(l0,l1)→0 로 발산하는 것 방지).
  const ln = Math.max(Math.abs(l[0]), Math.abs(l[1]), Math.abs(l[2])) || 1; l = [l[0] / ln, l[1] / ln, l[2] / ln];

  return { ok: true, l, center, single, lamVan, ringRatio: Math.sqrt(Math.abs(single)) };
}

// ── metric rectify 호모그래피 (이미지→정면, similarity만 잔여) ─────────────
// 입력: 소실선 l, 코어 원뿔 C(원의 이미지), 참중심(이미지) O, 목표스케일/중심.
// 반환: Himg2rect (3×3). 코어 원이 반경 targetR, 중심 targetC 인 원이 되도록.
function rectifyHomography(l, C, O, targetR, targetC) {
  // 1) 사영교정 P: 소실선 → 무한선 [0,0,1].
  const P = [[1, 0, 0], [0, 1, 0], [l[0], l[1], l[2]]];
  // P 하의 코어 원뿔: Ca = P^{-T} C P^{-1}
  const Pinv = inv3(P); if (!Pinv) return null;
  const Ca = mul3(transpose3(Pinv), mul3(C, Pinv));
  // 2) Ca(아핀평면상 타원)의 2×2 이차부 Q → 원으로 만드는 아핀 A.
  const Q = [[Ca[0][0], Ca[0][1]], [Ca[0][1], Ca[1][1]]];
  const eg = jacobiEig(Q, 2);
  // Q 를 정부호로(원뿔 부호 정규화). 두 고유값 동부호여야 타원.
  let e0 = eg.val[0], e1 = eg.val[1];
  if (e0 * e1 <= 0) return null;
  if (e0 < 0) { e0 = -e0; e1 = -e1; }
  // A_shape = D^{1/2} Vᵀ (Q 를 등방화). V=eg.vec (열=고유벡터)
  const V = eg.vec;
  const sq0 = Math.sqrt(e0), sq1 = Math.sqrt(e1);
  const Ashape = [
    [sq0 * V[0][0], sq0 * V[1][0]],
    [sq1 * V[0][1], sq1 * V[1][1]],
  ];
  // 반사 제거: det<0 이면 한 축 뒤집어 방향보존(원은 회전대칭이라 회전만 남기고
  //   반사는 locate 의 시계방향 라벨을 깨므로 반드시 제거).
  if (Ashape[0][0] * Ashape[1][1] - Ashape[0][1] * Ashape[1][0] < 0) { Ashape[1][0] = -Ashape[1][0]; Ashape[1][1] = -Ashape[1][1]; }
  // 아핀 전체 = Ashape 적용한 P (3×3). 아직 중심/스케일/회전 미정.
  const Aaff = [[Ashape[0][0], Ashape[0][1], 0], [Ashape[1][0], Ashape[1][1], 0], [0, 0, 1]];
  let H = mul3(Aaff, P);                       // 이미지→(등방)정면. 코어=원(임의 반경/중심).
  // 3) 정규화: 코어중심 O 가 targetC 로, 코어 반경이 targetR 로.
  const oc = matVec3(H, [O[0], O[1], 1]); const ocx = oc[0] / oc[2], ocy = oc[1] / oc[2];
  // 현재 코어 반경: 원뿔 C 를 H 로 보낸 원의 반경 측정 — 원 위 한 점이 필요.
  // O 에서 소실선 반대방향으로 원의 한 점 대신, C 로부터 반경 추정:
  //   변환후 원뿔 Cr = H^{-T} C H^{-1}; 중심 (ocx,ocy), 반경 = sqrt(-Fc/Ac) (정규원).
  const Hinv = inv3(H); if (!Hinv) return null;
  const Cr = mul3(transpose3(Hinv), mul3(C, Hinv));
  // 정규화 원뿔: A(x²+y²) + D x + E y + F, 중심(-D/2A,-E/2A), 반경²=(D²+E²)/(4A²)-F/A
  const Ac = (Cr[0][0] + Cr[1][1]) / 2;
  let curR = Math.sqrt(Math.max(1e-9, (Cr[0][2] * Cr[0][2] + Cr[1][2] * Cr[1][2]) / (Ac * Ac) - Cr[2][2] / Ac));
  const k = targetR / (curR || 1);
  // 스케일 k + 평행이동으로 중심 맞춤
  const Snorm = [[k, 0, targetC[0] - k * ocx], [0, k, targetC[1] - k * ocy], [0, 0, 1]];
  H = mul3(Snorm, H);
  return H;
}

module.exports = {
  jacobiEig, inv3, adj3, mul3, matVec3, transpose3, frob, scale3, sub3, det3, solveCubic,
  extractCoreRings, meanRadialProfile, estimateCoreScale, fitConic, recoverFromConcentric, rectifyHomography,
};

});

__def("geometry", function(module, exports){
'use strict';
/* ★밀착 반경 핀(2026-08-30) — 정본은 shape-registry.js 다.
 *   번들(build-core.js)이 shape-registry 를 geometry 보다 **먼저** 등록하므로 안전하다.
 *   없어도 동작한다(그때는 solver 가 계산한다) — 그래서 실패를 삼킨다. */
let REG = null; try { REG = require('./shape-registry.js'); } catch (e) { REG = null; }
/*
 * ============================================================================
 *  WIA Code v2 — 오르빗(Orbit) 앵커 기하 스펙 + 렌더러
 * ============================================================================
 *  Fable 5 설계 결론 구현. QR의 7×7 파인더를 대체하는 "독자적 localization
 *  primitive"의 기하만(데이터 코덱 제외) 정의·렌더한다. 이 파일 하나로
 *  반나절 검증에 필요한 "정답 앵커 좌표"까지 산출한다.
 *
 *  좌표계: 모듈(module) 단위. 코드 격자는 N×N 모듈(S=64/M=96/L=128).
 *          픽셀 = (module + quiet) × cellPx.
 *
 *  앵커 5개 (검출·정렬의 기준점):
 *    - core     : 정중앙 동심원 불스아이(행성). 링 주기 2모듈(QR 최소특징의 2배).
 *    - sat TL/TR/BR/BL : 네 코너 위성. TL만 도넛("북극성", 흰 중심)→회전 확정.
 *  그 외 포맷 궤도(24 도트)·데이터 도트(성좌 질감)는 시각 충실도 + 검출기
 *  스트레스(클러터)용으로 렌더하되, 앵커가 아니다.
 * ============================================================================
 */

// ── 기하 상수 (모듈 단위) ────────────────────────────────────────────────
const SPEC = {
  // 코어 동심원: [바깥반경, 강도] 경계. 안→밖으로 검/흰/검/흰(세퍼레이터).
  coreRings: [
    { r: 1.5, ink: true },   // 중심 검정 원판 (브랜드 슬롯)
    { r: 3.5, ink: false },  // 흰 링 (폭 2)
    { r: 5.5, ink: true },   // 검정 링 (폭 2)
    { r: 6.5, ink: false },  // 흰 세퍼레이터 (폭 1)
  ],
  coreOuter: 6.5,            // 코어 예약 반경
  satRadius: 2.5,            // 위성 반경(지름 5모듈)
  satInner: 1.0,            // 북극성 도넛 흰 중심 반경
  satHalo: 1.0,             // ★밀착 모드: 위성 바깥 흰 고리 두께(모듈). 데이터 속에서 위성을 떼어낸다
                            //   1.6 은 과잉이었다 — 0.6 에서도 위치오차 0.06~0.08모듈로 편향이 없다(실측).
                            //   1.0 이면 1.6 대비 용량 +1.0%(ear)·+1.8%(foot) 를 회복한다.
  hugFanDeg: 20,            // ★밀착 탐색 부채꼴 반각(도) — **탈출구로만** 쓴다(P2 정책, 아래 참조).
                            //   35° 로 "가장 먼 잉크"를 쫓았더니 반경 시그니처 충돌이 7→59 쌍으로 늘고
                            //   16종이 90° 슬롯 배정에서 경계에 걸렸다(ear 44°/45°). 최대반경 추구가 틀렸다.
                            //   ±20° 면 45° 슬롯 경계까지 25° 여유가 남는다.
  hugFloor: 14,             // ★밀착 반경 바닥(모듈). 궤도 8.5 + 도트 0.9 + 고리 + 위성 2.5 + 여유.
                            //   축방향 탐색은 love TL=4.6 처럼 **코어·궤도와 겹치는** 반경을 조용히 만들었다.
  satInset: 4.5,            // 코너에서 위성 중심까지(=모듈 4.5)
  orbitRadius: 8.5,          // 포맷 궤도 반경
  orbitDots: 24,             // 궤도 도트 수
  orbitDotR: 0.55,           // 궤도 도트 반경
  /* 데이터 도트 반경(라운드 채움). ★2026-09-02 (C7/P5) 0.42 → 0.50, 채움률 55% → 79%.
   *   근거: vs-QR 감사 — 저해상도 격차의 진범은 서브픽셀 정합이 아니라 **채움률**이다
   *   (`~/wiacode-vs-qr/RESEARCH.md` §3-3, `exp_fable/fill2.js`·`ber.js`).
   *   실측(라이브 경로 generate→detectAuto, 격자 L, 형상 10종 × 문구 3개):
   *     r 0.42  전건 해독 1.50 px/mod 까지 · r 0.46 = 0.42 와 동일(중간값 무의미) · r 0.50  1.25 px/mod 까지
   *   락 성공률은 세 값 모두 전 구간 100%.
   * ★첫 착수(Opus)는 foot/L 이 깨져 되돌렸다(hugon 493/496, 811·816·817·819·823·1000px 실패).
   *   Fable 이 셀 단위로 추적한 결과 도트 굵기가 아니라 **밀착 마름모 종횡비(foot 3.2)가 만든
   *   사영 H 의 가짜 원근**이 원인이었다 — 먼 셀에서 2.5~5px 오차, 0.42 는 도트 사이 흰 여백이
   *   그걸 흡수했고 0.50 은 도트가 맞닿아 못 흡수했을 뿐. build-core.js `_hugAttempt` 의
   *   **아핀 대안**(앵커 5점 자기투영)으로 근치 → foot 9/9. 그 뒤에 이 값을 올렸다.
   * ★청구항 1·4 에 0.42 가 기재돼 있다 — 바꾸는 것은 막히지 않는다(특허는 우리 구현을 얼리지
   *   않는다). `~/patent-wiacode/IMPROVEMENTS_AFTER_FILING.md` I2 에 기록해 후속출원에 얹는다. */
  dataDotR: 0.50,
};

const GRID_CELLS = { S: 64, M: 96, L: 128 };

// ── 다리QR 존 (2026-08-16, Fable5 설계) ────────────────────────────────────
//   ★포맷 상수 — 한 번 배포되면 절대 변경 금지. 바꾸면 이미 인쇄·저장된 다리QR
//   코드가 전부 해독 불능이 된다(스캐너는 이 값으로 "예약된 자리"를 재구성함).
//   모듈 단위(cellPx·quiet 무관)로 동결 — 픽셀 좌표는 이 값에서 항상 유도만 한다
//   (지금까지 픽셀 공식이 cellPx마다 값이 달라 디코더가 셀 집합을 재현 못 하던
//   문제를 근본적으로 없앰). 존 예약은 grid L에서만 정의(다리QR은 L 전용).
const BRIDGE = {
  grid: 'L',
  qrModules: 25,     // bridge-qr.js의 SIZE와 반드시 일치(런타임 가드로 검증)
  qMod: 0.7,          // 오르빗 모듈 per QR모듈 — 기존 cellPx10·qFrac0.15 실측 배치와 동일 밀도
  padQrMod: 0.35,     // 흰 quiet zone(모듈) — ★2026-09-02 1.4→0.35 (아래 §다리QR 배치 개정)
  topMy: 77.5,        // 표에 없는 형상의 기본 QR 상단 y(모듈) — 코어 바로 아래
  marginMod: 0.5,     // 예약 안전여백(데이터 도트 반경 + 샘플 오프셋 커버)
                      //   ★C7 로 도트 반경이 0.42→0.50 이 됐다. 여백 0.35 인 형상에서는
                      //     도트 잉크가 QR 흰 패드 경계에 딱 닿는다(0.85-0.50=0.35). 겹치지는
                      //     않지만 여유가 0 이라, 여기를 더 줄이려면 배터리부터 다시 돌릴 것.
};
/* ── 다리QR 배치 개정 (2026-09-02, 오너 착안) ──────────────────────────────
 *   문제: QR 자리가 **모든 형상에서 같았다**(가로중앙·topMy 77.5). 그 자리에 몸통이
 *   없는 실루엣(폐·부메랑·보청기·강아지…)은 QR 이 흰 배경 위에 홀로 떠서, 형상보다
 *   QR 이 먼저 눈에 들어왔다. 우리 목적은 반대다 — **형상을 각인시키고 QR 은 코드
 *   안에 희석시킨다.**
 *
 *   왜 옮겨도 되는가: QR 은 자기 파인더 패턴으로 스스로를 찾으므로 **일반 QR 리더는
 *   위치와 무관하게 읽는다.** 우리 디코더는 이미 형상 후보를 순회하며 형상마다
 *   layout() 을 부르므로(build-core.js Pass1/Pass2), 형상별 자리는 그 순회 안에서
 *   그대로 재구성된다 — **해독 비용 증가 0.**
 *
 *   실측(66종 전수, `~/wiacode-perf/` 도구): 지금 100% 가 아닌 37종 중 24종이 100%,
 *   12종이 크게 개선, 뇌만 87.0→90.1 로 제자리. 못 고치는 형상은 0종.
 *
 *   값 = **inner(QR 자체) 좌상단**의 모듈 좌표. pad 를 바꿔도 흔들리지 않도록 inner 를
 *   저장한다. 후보는 ①코어·궤도·위성 예약과 **겹치지 않고**(밀착/비밀착 두 배치 모두)
 *   ②실루엣 안에 든 비율 최대 ③동점이면 주변 도트 밀도(묻힘)가 높은 쪽으로 골랐다.
 *   ★재생성: `node ~/wiacode-perf/solve-bridge-pos.js`
 *
 *   ★표에 없는 형상은 예전 자리 그대로다(개선 5%p 미만이라 **잘 되는 걸 안 건드린다**).
 *   ★옛 발행분 호환: 자리와 pad 가 바뀌면 데이터 셀 집합이 달라져 옛 코드가 안 읽힌다.
 *   그래서 디코더는 `bridgeLegacy:true`(옛 자리 + 옛 pad 1.4)를 **한 번 더 시도**한다.
 *   그 시도는 표에 있는 형상에서만 추가되므로 나머지 32종은 비용이 그대로다. */
const BRIDGE_LEGACY = { padQrMod: 1.4, topMy: 77.5 };   // 2026-09-02 이전 발행분
/* ★여백 축소에서 제외하는 형상 (2026-09-03) — 아래 bridgeRect 주석 참조.
 *   square 는 WIA Stream 의 캐리어이고 그 프레임 계약(capBytes 1450·blockSize 1059)이
 *   동결돼 있다. 여백을 줄이면 용량이 올라가 계약이 어긋난다. 계약 개정과 함께가 아니면
 *   여기서 빼지 말 것. */
const BRIDGE_PAD_FROZEN = { square: true };
const BRIDGE_POS = {
  boomer:       [34.9, 49.9],   // 부메랑 0.0% → 94.8% (팔이 얇아 100%는 불가)
  lungs:        [80.9, 39.9],   // 폐 0.0% → 95.2%
  turtle:       [80.9, 78.9],   // 거북이 5.8% → 99.6%
  dog:          [54.9, 12.9],   // 강아지 7.6% → 100%
  hearingaid:   [31.9, 83.9],   // 보청기 18.2% → 100%
  smile:        [54.9, 22.9],   // 웃음 21.1% → 100%
  bone:         [34.9, 56.9],   // 뼈 22.7% → 100%
  embrace:      [54.9, 34.9],   // 포옹 44.2% → 100%
  whale:        [28.9, 52.9],   // 고래 49.8% → 81.8%
  gimbap:       [95.9, 45.9],   // 김밥 51.9% → 96.1%
  family:      [100.9, 48.9],   // 가족 52.1% → 95.2%
  piano:        [46.9, 26.9],   // 건반 52.3% → 93.2%
  butterfly:    [28.9, 38.9],   // 나비 55.0% → 95.7%
  sadness:      [54.9, 23.9],   // 슬픔 55.4% → 100%
  car:          [74.9, 62.9],   // 자동차 59.1% → 100%
  ear:          [42.9, 29.9],   // 귀 65.3% → 90.5%
  earth:        [77.9, 71.9],   // 지구 75.2% → 100%
  rose:         [48.9, 94.9],   // 장미 75.6% → 82.6%
  fish:         [45.9, 31.9],   // 물고기 76.2% → 100%
  grandpiano:   [34.9, 65.9],   // 그랜드피아노 77.3% → 100%
  dolhareubang: [30.9, 79.9],   // 돌하르방 78.3% → 95.5%
  deer:         [54.9, 96.9],   // 사슴 78.5% → 100%
  tree:         [55.9, 19.9],   // 나무 81.4% → 100%
  elevator:     [41.9, 97.9],   // 엘리베이터 81.8% → 100%
  love:         [23.9, 52.9],   // 사랑 84.7% → 100%
  arena:        [18.9, 67.9],   // 경기장 86.0% → 100%
  connection:   [54.9, 34.9],   // 연결 86.4% → 98.1%
  book:         [54.9, 90.9],   // 책 86.6% → 100%
  tooth:        [35.9, 69.9],   // 치아 86.8% → 100%
  rabbit:       [54.9, 87.9],   // 토끼 87.2% → 100%
  prayer:       [66.9, 76.9],   // 기도 88.8% → 99.4%
  lonely:       [64.9, 84.9],   // 외로움 92.8% → 100%
  eye:          [53.9, 74.9],   // 눈 93.2% → 99.0%
  oksign:       [47.9, 85.9],   // 오케이 94.0% → 100%
};
/* grid의 다리QR 존 사각형(모듈 좌표). grid L이 아니면 null.
 *   shape  — 형상별 자리표(BRIDGE_POS)를 쓴다. 없으면 예전 기본 자리.
 *   legacy — true 면 2026-09-02 이전 배치(가로중앙·topMy 77.5·pad 1.4)로 되돌린다. */
function bridgeRect(N, shape, legacy) {
  if (N !== GRID_CELLS[BRIDGE.grid]) return null;
  const c = N / 2, q = BRIDGE.qMod * BRIDGE.qrModules;
  const pos = (!legacy && shape) ? BRIDGE_POS[shape] : null;
  const inner = { mx0: pos ? pos[0] : c - q / 2,
                  my0: pos ? pos[1] : (legacy ? BRIDGE_LEGACY.topMy : BRIDGE.topMy), w: q, h: q };
  /* ★흰 여백 축소(1.4→0.35) — 2026-09-02 엔 **자리를 옮긴 34종에만** 걸었다.
   *   전 형상에 걸었다가 hugon-battery 가 foot/L 을 잡았기 때문이다(496 → 494). 원인은
   *   여백이 아니라 **그림이 달라진 것**이었다 — 여백이 줄면 데이터 셀이 늘고, foot/L 은
   *   "정확히 원본 크기(816px)에서 격자를 오판한다"는 칼날 위 사례라 그 미세한 변화에
   *   격자가 넘어가 해독이 죽었다. 그때 이 주석이 조건을 적어 뒀다: **"전 형상으로 넓히려면
   *   foot/L 격자 오판을 먼저 근치할 것."**
   *
   *   ★★2026-09-03 — 그 조건이 충족돼 넓힌다(오너 지시). C7(밀착 아핀 대안 + 원거리격자
   *   재적합, 09-02)이 격자 오판을 근치했고, 재현으로 확인했다: foot/L 을 정확히 816px 로
   *   줘도 격자를 L 로 맞힌다. 하위호환은 build-core.js 변형 2(옛 pad 1.4 재시도)가 받는다 —
   *   그쪽 게이트도 "옛 배치와 다른 형상 전부"로 같이 넓혔다(09-02 ① 사고의 정확한 처방).
   *
   *   ★square 만 예외다. Stream 프레임 계약(`capBytes 1450`·`blockSize 1059`,
   *   PILOT_B0_CORE_2026-08-23 §3)이 **square/L 에 동결**돼 있고, 여백을 줄이면 데이터 셀이
   *   늘어 capBytes 가 올라간다 → `stream-lab.html` 의 `d.capBytes !== CAP_BYTES` 가드가
   *   프레임을 거부한다. 계약을 고치는 건 별건이라 square 는 옛 여백을 그대로 둔다.
   *   부수 효과로 square 는 옛 배치와 완전히 같아져 변형 2 가 자동으로 꺼진다(해독 비용 0). */
  const p = (legacy || BRIDGE_PAD_FROZEN[shape]) ? BRIDGE_LEGACY.padQrMod : BRIDGE.padQrMod, m = BRIDGE.marginMod;
  const padded = { mx0: inner.mx0 - p, my0: inner.my0 - p, mx1: inner.mx0 + q + p, my1: inner.my0 + q + p };
  return { inner, padded, reserve: { mx0: padded.mx0 - m, my0: padded.my0 - m, mx1: padded.mx1 + m, my1: padded.my1 + m } };
}

// 스펙 오버라이드 병합 (Fable5 레버 실험용: satRadius/satInset/satInner 등)
function mergeSpec(over) {
  const s = Object.assign({}, SPEC, over || {});
  s.coreRings = SPEC.coreRings; // 코어 링은 고정
  return s;
}

// ── 시드 RNG (재현성) ─────────────────────────────────────────────────────
function makeRng(seed) {
  let s = (seed >>> 0) || 0x9e3779b9;
  return () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 0x100000000; };
}


/* ── 밀착(마름모) 풀이 ─────────────────────────────────────────────────────
 *   §J 참조. 축 A(12·6시)와 축 B(3·9시) 각각에서, **위성 원판+흰 고리가 채운 윤곽 안에
 *   완전히 들어가는 가장 큰 계단**을 고른다. 두 축은 독립이다 — 그래서 마름모가 된다.
 *   판정은 **채운 윤곽**(바깥에서 flood fill 해 닿지 않는 곳)이다: 안쪽 흰 무늬
 *   (뇌주름·경위선·장미의 로그나선 골)에 걸려 멀쩡한 형상이 탈락하던 것을 막는다. */
const _hugCache = new Map();
function _filledOutline(L, N) {
  const SS = 2, W = N * SS, ink = new Uint8Array(W * W);
  for (let y = 0; y < W; y++) for (let x = 0; x < W; x++)
    if (insideShape((x + 0.5) / SS, (y + 0.5) / SS, L)) ink[y * W + x] = 1;
  const out = new Uint8Array(W * W), st = [];
  for (let x = 0; x < W; x++) { st.push(x); st.push(x + (W - 1) * W); }
  for (let y = 0; y < W; y++) { st.push(y * W); st.push(y * W + W - 1); }
  while (st.length) {
    const i = st.pop();
    if (out[i] || ink[i]) continue;
    out[i] = 1;
    const x = i % W, y = (i - x) / W;
    if (x + 1 < W) st.push(i + 1);
    if (x > 0) st.push(i - 1);
    if (y + 1 < W) st.push(i + W);
    if (y > 0) st.push(i - W);
  }
  return { out, W, SS };
}
function solveRhombus(L, N, c, Rs, tiers, spec, bridgeLegacy) {
  const F = _filledOutline(L, N);
  const SR = (spec.satRadius || 2.5) + (spec.satHalo || 1.0);
  /* ★부스터 QR 예약 사각은 금지 구역이다 (2026-09-01).
   *   밀착은 위성을 코어 쪽으로 당기는데, 짧은 축이 ~25모듈 아래로 내려오면 그 자리가
   *   코어 바로 아래 QR 존(topMy 77.5, 예약 53.35~74.65 × 75.6~96.9)과 **겹친다.**
   *   생성기는 QR 을 항상 얹으므로(bridge:true) 흰 패드가 위성을 통째로 지워 **라이브에서
   *   락이 안 된다** — arena·whale·mouth(원판 전체), like·prayer·car·dove·connection(일부),
   *   실측 8종. hugon-battery 는 QR 을 안 얹고 재서 8일 동안 못 잡았다(이제 얹는다).
   *   실루엣 잉크만 보던 판정에 QR 예약 사각을 "잉크 아님"으로 더한다 → φ·반경 탐색이
   *   저절로 그 자리를 피한다. 격자 L 에만 존이 있다(bridgeRect 는 그 외 null). */
  //   ★2026-09-02: 자리가 형상별이 됐으므로 이 금지구역도 **그 형상의 자리**여야 한다.
  const _br = bridgeRect(N, L.shape, bridgeLegacy), _bz = _br && _br.reserve;
  const inF = (mx, my) => {
    if (_bz && mx >= _bz.mx0 && mx <= _bz.mx1 && my >= _bz.my0 && my <= _bz.my1) return false;
    const x = Math.floor(mx * F.SS), y = Math.floor(my * F.SS);
    if (x < 0 || y < 0 || x >= F.W || y >= F.W) return false;
    return !F.out[y * F.W + x];
  };
  const cov = (ang, R) => {
    let ok = 0, tot = 0;
    for (let dy = -SR; dy <= SR; dy += 0.5) for (let dx = -SR; dx <= SR; dx += 0.5) {
      if (dx * dx + dy * dy > SR * SR) continue;
      tot++;
      if (inF(c + R * Math.cos(ang) + dx, c + R * Math.sin(ang) + dy)) ok++;
    }
    return tot ? ok / tot : 0;
  };
  /* ★바닥 — 위성이 **궤도링을 침범하면 안 된다.**
   *   궤도 8.5 + 궤도점 0.9 + 위성 2.5 + 흰고리 1.0 + 여유 1.0 = 13.9. */
  const floor = (spec.orbitRadius || 8.5) + (spec.orbitDotR || 0.9)
              + (spec.satRadius || 2.5) + (spec.satHalo || 1.0) + 1.0;

  /* ★연속 밴드-톱 (2026-08-30, Fable 검토 뒤 계단 폐기) ────────────────────
   *   전에는 Rs×0.8^k 계단에서 골랐다. 계단을 버린 이유는 **커버리지가 비단조**라서다:
   *   hope 의 3·9시 통과대역은 R 52~54.5 의 **폭 3모듈짜리 밴드**인데 0.8 계단
   *   (60.5 → 48.4)이 그 위아래로 건너뛰어 통째로 놓친다. 계단을 없애니
   *   hope 짧은축 24.8 → 54.3 (**+119%**), star +20~24%, family +16~19%, 중앙값 ~13%.
   *   0.75/0.8/0.85 어느 고정비도 지배하지 못한다(0.85는 hope를 살리고 rose를 죽인다).
   *
   *   ★연속값이어도 정합은 자동이다 — 생성과 해독이 **같은 결정적 solver** 를 쓴다.
   *   ★다만 레이아웃 반경 허용오차가 **0.5~1.0%** 라(실측: brain/L 은 0.5%에서 해독 사망)
   *     solver 드리프트에 극도로 민감하다 → 확정값은 shape-registry 에 **핀으로 박는다**
   *     (아래 REG.hugPin). 핀이 있으면 이 탐색을 아예 안 돈다.
   *   ★밴드-톱은 95% 경계에 정확히 앉으므로 **0.5모듈 물러선다**(BACKOFF) — 경계에
   *     걸터앉으면 마스크 샘플링이 조금만 달라져도 넘어간다. */
  const STEP = 0.25, BACKOFF = 0.5;
  const scan = (a1, a2) => {
    for (let R = tiers[0]; R >= floor; R -= STEP) {
      if (cov(a1, R) >= 0.95 && cov(a2, R) >= 0.95) {
        // 경계에서 물러선다 — 물러선 자리도 통과해야 채택한다(밴드가 얇으면 그대로 둔다).
        const b = R - BACKOFF;
        if (b >= floor && cov(a1, b) >= 0.95 && cov(a2, b) >= 0.95) return b;
        return R;
      }
    }
    return null;
  };
  const bestTier = scan;
  /* ★φ(위성 십자의 회전)도 함께 찾는다 — 2026-08-30, 오너 착안.
   *   전에는 형상을 돌려서 위성 자리에 잉크를 갖다 댔다. 그러면 **사람이 기울어진 나비를 본다.**
   *   그럴 필요가 없다: 스캐너는 코어+위성으로 호모그래피를 잡으므로 **회전을 흡수한다.**
   *   그러니 **형상은 똑바로 두고 위성 십자만 φ 만큼 돌린다.**
   *   사람은 똑바른 나비를 보고, 위성은 잉크 위에 앉고, 코드 외곽은 정사각 그대로다.
   *   (butterfly: 형상을 47° 눕히던 것 → 형상 똑바로 + 위성 33° 로 47.0/55.3)
   *   ★φ 는 프레임에서 관측되지 않는다(전역 회전과 구별 불가) — 그래서 **핀에 박아** 두고,
   *     같은 (rA,rB) 를 쓰는 형상끼리는 readCode 의 CRC 가 가른다. 후보가 안 늘어난다. */
  const A0 = -Math.PI / 2;
  let best = null;
  for (let d = 0; d < 90; d += 1) {
    const ph = d * Math.PI / 180;
    const rA = bestTier(A0 + ph, A0 + ph + Math.PI);
    const rB = bestTier(A0 + ph + Math.PI / 2, A0 + ph + 3 * Math.PI / 2);
    if (rA == null || rB == null) continue;
    if (!best || rA + rB > best.rA + best.rB) best = { rA, rB, phi: d };
  }
  if (!best) return null;                                      // 밀착 불가 — 기본 배치로
  return best;
}

// ── 앵커/궤도 좌표 산출 ───────────────────────────────────────────────────
//   shape: 'square'(기본, 위성=4코너) | 'round'(위성=링 12/3/6/9시, 데이터판=원판).
//   matcher(locate)는 "코어를 둘러싼 4위성 + 북극성"만 보므로 두 모양 모두 같은 코드로 검출됨.
function layout(grid, spec, shape, opts) {
  spec = spec || SPEC; shape = shape || 'square';
  const N = GRID_CELLS[grid];
  const c = N / 2;                 // 코어 중심(모듈)
  let anchors, hugOn = false;
  if (shape !== 'square') {
    const Rs = N / 2 - (spec.satRimMargin || 3.5);   // 위성 링 반경(rim 안쪽) — 원/하트 공용
    let ANG = { TL: -Math.PI / 2, TR: 0, BR: Math.PI / 2, BL: Math.PI };  // 12/3/6/9시(시계방향)
    anchors = [{ name: 'core', type: 'bullseye', mx: c, my: c }];
    /* ★밀착 = **마름모** (2026-08-30, 오너 착안 — 사각에서 나온 원리) ─────────────
     *   사각의 위성 4개는 자기 꼭짓점 그 자체(r 84.1, ±45°)이고 8개월째 잘 읽힌다.
     *   거기서 나온 규칙: **긴 축 2개는 살리고 짧은 축 2개만 코어 쪽으로 당긴다.**
     *
     *   앵커 반경 = [R긴, R짧, R긴, R짧] — 마주 보는 짝이 같다.
     *   ① 점대칭이 정확히 유지 → 네 앵커 무게중심 = 코어 (dCen ≡ 0).
     *      방향별 4반경(이전 ±20° 부채꼴 정책)은 이 성질이 깨져 similarity 적합이 나빴다.
     *   ② 계단 = Rs × 0.8^k → 비율이 **정확히 1.25^m**(1.00 1.25 1.56 1.95 2.44 3.05).
     *      반경 측정오차 2.8% 대비 **9배 여유**. 이전 4벡터 시그니처는 최소 분리 2.1% 였다.
     *   ③ 앵커 각도는 **안 돌린다**(항상 12/3/6/9시). 형상 쪽을 미리 돌려서 굽는다 —
     *      각도를 앵커에 두면 φ 가 잠금 후보에 들어가 후보가 폭발한다.
     *   ④ 원(60.5,60.5)·사각(84.1,84.1)이 이 규칙의 특수해라 별도 분기가 필요 없다.
     *
     *   ★판정은 **채운 윤곽**으로 한다. 잉크 기준으로 재면 뇌주름·경위선·장미의 로그나선 골
     *     같은 **안쪽 흰 무늬**에 위성이 걸려 brain 36%·earth 21% 로 떨어진다(실측).
     *     사람이 프레임에 넣는 건 바깥 윤곽이므로 바깥에서 flood fill 한 채움이 기준이다.
     *   ★결과는 캐시한다 — layout() 은 스캔 1회에 수십 번 불린다. */
    const hug = !!(opts && opts.hug);
    let hugR = null;
    if (hug) {
      const tiers = [Rs];      // 연속 탐색의 **상한**만 쓴다(계단은 2026-08-30 폐기)
      const key = (opts && opts.customMask) ? null : (grid + '|' + shape);
      // ★핀 우선 — 레지스트리에 확정값이 있으면 탐색하지 않는다.
      //   ①solver 를 나중에 고쳐도 이미 발행된 코드가 안 깨진다(허용오차 0.5%)
      //   ②첫 스캔의 solver 비용(~1.2s)이 사라진다
      let got = null;
      if (key && REG && REG.hugPin) {
        const pin = REG.hugPin(shape, grid);
        if (pin) got = { rA: pin[0], rB: pin[1], phi: pin[2] || 0 };
      }
      if (!got) got = key ? _hugCache.get(key) : null;
      if (!got) {
        const tmpL = { N, coreCenter: { mx: c, my: c }, spec, shape, Rdata: N / 2 - 0.8 };
        if (opts && opts.customMask) tmpL.customMask = opts.customMask;
        got = solveRhombus(tmpL, N, c, Rs, tiers, spec, opts && opts.bridgeLegacy);
        if (key) _hugCache.set(key, got);
      }
      if (got) {
        hugR = { TL: got.rA, BR: got.rA, TR: got.rB, BL: got.rB };
        if (got.phi) {                       // 위성 십자만 돌린다 — 형상·데이터격자는 그대로
          const ph = got.phi * Math.PI / 180;
          for (const k of ['TL', 'TR', 'BR', 'BL']) ANG[k] += ph;
        }
      }
    }
    /* ★밀착이 성립하지 않으면 **플래그까지 끈다.**
     *   앵커만 기본으로 돌리고 L.hug 를 켜 두면 흰 고리(satHalo)만 그려져
     *   reservedAt/dataCells 가 비밀착 레이아웃과 어긋난다 — 생성은 되는데 해독이 안 된다.
     *   실측: rose 가 격자 M/S 에서 밀착 불가라 이 상태가 됐고 4/4 전부 실패했다. */
    hugOn = hug && hugR != null;
    for (const k of ['TL', 'TR', 'BR', 'BL']) {
      const R = hugR ? hugR[k] : Rs;
      anchors.push({ name: k, type: k === 'TL' ? 'donut' : 'disk', mx: c + R * Math.cos(ANG[k]), my: c + R * Math.sin(ANG[k]) });
    }
  } else {
    const d = spec.satInset;       // 코너 인셋
    anchors = [
      { name: 'core', type: 'bullseye', mx: c,     my: c     },
      { name: 'TL',   type: 'donut',    mx: d,     my: d     }, // 북극성
      { name: 'TR',   type: 'disk',     mx: N - d, my: d     },
      { name: 'BR',   type: 'disk',     mx: N - d, my: N - d },
      { name: 'BL',   type: 'disk',     mx: d,     my: N - d },
    ];
  }
  // 포맷 궤도 도트 (코어 기준). 상단(-90°)부터 시계방향.
  const orbit = [];
  for (let i = 0; i < spec.orbitDots; i++) {
    const a = -Math.PI / 2 + (2 * Math.PI * i) / spec.orbitDots;
    orbit.push({ mx: c + spec.orbitRadius * Math.cos(a), my: c + spec.orbitRadius * Math.sin(a), on: (i * 7 + 3) % 5 < 3 });
  }
  const L = { N, coreCenter: { mx: c, my: c }, anchors, orbit, spec, shape, hug: hugOn, noGrooves: !!(opts && opts.noGrooves) };
  L.bridgeRect = (opts && opts.bridge) ? bridgeRect(N, shape, opts.bridgeLegacy) : null;
  // 'custom' 실루엣 마스크 전달(insideShape 참고). 다른 shape에선 무시된다.
  if (opts && opts.customMask) L.customMask = opts.customMask;
  if (shape !== 'square') {
    L.Rdata = N / 2 - 0.8;         // 데이터 원판 반경(콰이엇 고리 안쪽)
    const TL = anchors[1];
    L.calibPoints = {              // 흑/백 캘리브레이션 기준점(모양 무관하게 명시)
      black: [[c, c]].concat(anchors.filter(a => a.type === 'disk').map(a => [a.mx, a.my])),
      white: [[TL.mx, TL.my], [c + 2.5, c]].concat([45, 135, 225, 315].map(deg => {
        const a = deg * Math.PI / 180, r = N / 2 + 1.5; return [c + r * Math.cos(a), c + r * Math.sin(a)];
      })),
    };
  }
  return L;
}

// ── 하트 실루엣: 고전 매끈한 파라메트릭 곡선(원+쐐기 이어붙이기 아님) ─────────
//   x=sin³t, y=13cos t−5cos2t−2cos3t−cos4t (전 구간 C∞ 매끈, 이음매 없음).
//   이전 원(로브)+선형쐐기 결합은 두 곡선이 만나는 지점에서 값은 같아도 기울기가
//   달라(원 쪽 기울기 -1.4대 쐐기 쪽 -0.5) 옆구리에 살짝 꺾인 자국(홀쭉한 자국)이 남았음 —
//   단일 매끈 곡선으로 바꿔 그 이음매 자체를 없앰. 720점 샘플 폴리곤 + point-in-polygon.
//   앵커 위치엔 무관(검출 원리와 분리, insideShape는 데이터 셀 마스크에만 관여).
var HEART_POLY = (function () {
  var N = 720, pts = [];
  for (var i = 0; i < N; i++) {
    var t = (i / N) * 2 * Math.PI;
    var x = Math.pow(Math.sin(t), 3);
    var yRaw = 13 * Math.cos(t) - 5 * Math.cos(2 * t) - 2 * Math.cos(3 * t) - Math.cos(4 * t);
    var y = -0.205 - 0.07088 * yRaw; // yRaw∈[-17,11.92] → ny∈[1.0,-1.05] (뾰족점 아래, 로브 위)
    pts.push([x, y]);
  }
  return pts;
})();
function pointInHeart(nx, ny) {
  var inside = false;
  for (var i = 0, j = HEART_POLY.length - 1; i < HEART_POLY.length; j = i++) {
    var xi = HEART_POLY[i][0], yi = HEART_POLY[i][1], xj = HEART_POLY[j][0], yj = HEART_POLY[j][1];
    if (((yi > ny) !== (yj > ny)) && (nx < (xj - xi) * (ny - yi) / (yj - yi) + xi)) inside = !inside;
  }
  return inside;
}

// ── 말풍선(bubble) 실루엣: 모서리 둥근 사각형 몸통(모서리 반경=몸통의 12%) + 좌하단
//   꼬리 삼각형 ─────────────────────────────────────────────────────────
//   ★2026-08-18 오너 확정(B안): 카디널 위성 앵커(정규화 반경 0.9573, grid L 기준)를
//   몸통 안에 "완전히" 담으려 하면(BUBBLE_H≈0.957) 그리드 하드 한계(≈1.005)까지 남는
//   여유가 0.05 안팎뿐이라 꼬리가 육안으로 거의 안 보인다(실측 확인). 대신 기존
//   clover 실루엣의 기존 전례(잎 4개, lobeD+lobeR=0.88 — 위성이 잎 밖 0.0773만큼
//   떠서 이미 라이브 중)와 "같은 급"으로 앵커 이탈을 맞춘다: BUBBLE_H=0.88 →
//   이탈량 0.9573-0.88=0.0773 (clover와 소수점 4자리까지 동일). 이 여유(0.125)로
//   좌하단에 실제로 알아볼 수 있는 삼각형 꼬리를 낸다. 위성은 clover와 동일한 원리로
//   몸통 밖 여백에 그대로 그려진다(inkAt은 insideShape와 무관 — 위치·판독 불변).
//   다리QR 존(코어 바로 아래 중앙 좁은 띠, ny≈0.18~0.52)과는 안 겹침(중앙 vs 좌하단).
var BUBBLE_H = 0.88, BUBBLE_R = BUBBLE_H * 0.12;
var BUBBLE_TAIL = [[-0.793, 0.878], [-0.878, 0.793], [-0.965, 0.965]];
function pointInRoundedSquare(nx, ny, H, r) {
  var qx = Math.abs(nx) - (H - r), qy = Math.abs(ny) - (H - r);
  var ax = Math.max(qx, 0), ay = Math.max(qy, 0);
  return Math.sqrt(ax * ax + ay * ay) + Math.min(Math.max(qx, qy), 0) - r <= 0;
}
function pointInTriangle(px, py, tri) {
  var sign = function (x1, y1, x2, y2, x3, y3) { return (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3); };
  var d1 = sign(px, py, tri[0][0], tri[0][1], tri[1][0], tri[1][1]);
  var d2 = sign(px, py, tri[1][0], tri[1][1], tri[2][0], tri[2][1]);
  var d3 = sign(px, py, tri[2][0], tri[2][1], tri[0][0], tri[0][1]);
  var hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0), hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);
  return !(hasNeg && hasPos);
}
function pointInBubble(nx, ny) {
  if (pointInRoundedSquare(nx, ny, BUBBLE_H, BUBBLE_R)) return true;
  return pointInTriangle(nx, ny, BUBBLE_TAIL);
}

// ── 말풍선 v2(chat) — 타원 몸통 + 좌하단 뿔 꼬리 ────────────────────────────
//   ★2026-08-27 신설. 기존 `bubble`(모서리 둥근 사각 + 얇은 꼬리)은 실물이 "꼬리 붙은
//   사각형"으로 읽혀 브랜드 형상으로 쓰기 어렵다는 오너 지적에 따라, **말풍선으로 즉시
//   읽히는** 형태를 새 키로 추가한다.
//   ★★`bubble` 은 절대 고치지 않는다 — 실루엣을 바꾸면 데이터 셀 집합·순서가 바뀌어
//   **이미 발행된 bubble 코드와 적합성 벡터 3종이 해독 불능**이 된다(생성 중단 ≠ 삭제).
//   그래서 `bubble` 은 레거시 해독용으로 그대로 두고, 생성 대상만 `chat` 으로 옮긴다.
//   기하: 가로로 넓은 타원(rx/ry = 1.23, 카카오톡 말풍선 비율과 같은 자릿수) + 좌하단에서
//   왼쪽 아래로 뻗는 삼각 꼬리. 꼬리 밑변 두 점은 타원 **안쪽**에 두어 이음매가 벌어지지
//   않게 했다(§7-29 교훈 1: 매끈하게 이어붙이면 특징이 죽는다 → 밑변은 붙이되 끝은 명백히
//   밖으로 내보낸다). 돌출량 = 몸통 세로 반경의 약 29%(bubble 꼬리 0.125·lid 0.17보다 크지만
//   말풍선은 꼬리가 정체성이라 의도적으로 키움).
//   ★위성 이탈: 북/남 위성(정규화 0.9573)이 타원 밖으로 0.177 뜬다 — heart/clover/star 가
//   이미 같은 성질을 갖고 있고 판독에는 무관하다(reservedAt 이 실루엣과 별개로 예약).
var CHAT_RX = 0.96, CHAT_RY = 0.78;
var CHAT_TAIL = [[-0.04, 0.778], [-0.40, 0.712], [-0.56, 1.01]];
function pointInChat(nx, ny) {
  if ((nx * nx) / (CHAT_RX * CHAT_RX) + (ny * ny) / (CHAT_RY * CHAT_RY) <= 1) return true;
  return pointInTriangle(nx, ny, CHAT_TAIL);
}

// ══════════════════════════════════════════════════════════════════════════
// ── 산업 실루엣 팩 파일럿 3종 (2026-08-26, 조사+파일럿 지시) ────────────────
//   ★★정품 미노출 — generate.html 드롭다운엔 추가하지 않음. WiaScan.generate()에
//   shape 이름을 직접 넘겨야만 접근되는 파일럿 전용 실루엣(갤러리 후보 보고용).
//   전부 "본체를 실제 반경(1.0)보다 살짝 줄이고 그 여유로 특징을 낸다" 원칙
//   (bubble의 BUBBLE_H=0.88 전례와 동일) — 그리드 하드 한계(≈1.0127, round의
//   ny∈[-1,1] 경계로 이미 room이 거의 없음)를 넘기지 않기 위함.
// ══════════════════════════════════════════════════════════════════════════

// ── 밥뚜껑(lid): 원판 본체 + 상단 손잡이 꼭지 ───────────────────────────────
//   꼭지 중심(0,-0.92)은 북극성 앵커(TL, ny=-0.957 축)와 같은 축 위에 둬서
//   "손잡이 자리 = 북극성 자리"로 자연스럽게 겹치게 한다(clover/bubble이 위성을
//   잎/꼬리 자리에 맞춘 것과 같은 원리). 몸통 반경 0.88은 bubble과 동일 여유값.
var LID_R = 0.88, LID_KNOB_CY = -0.92, LID_KNOB_R = 0.13;
function pointInLid(nx, ny) {
  if (nx * nx + ny * ny <= LID_R * LID_R) return true;
  const dx = nx, dy = ny - LID_KNOB_CY;
  return dx * dx + dy * dy <= LID_KNOB_R * LID_KNOB_R;
}

// ── 물고기(fish, 옛 "flatfish/넙치" — 2026-08-26 정식 개명): 타원 몸통 +
//   등/배지느러미 + 동쪽 제비꼬리(포크) ─────────────────────────────────────
//   ★개명 이유: 종 특정(넙치=flatfish/flounder 해부학) 실루엣이 아니라 "물고기"로
//   범용 인식되도록 설계됐다 — 이름을 실제 형태에 맞춤(오너 재가, 2026-08-26).
//   내부 식별자·키만 변경, 기하 자체는 무변경(pointInFish 왕복검증 그대로 재사용).
//   ★v1(타원+얕은 삼각패치)은 실루엣이 그냥 눌린 원으로 보여 기각(2026-08-26).
//   ★v2(타원+포크 꼬리만)는 꼬리는 살았지만 위성이 뜨는 상/하 축이 그냥 빈
//   허공에 뜬 점으로 보여 재검토(형 지시로 2차 다듬기, 2026-08-26). v3: §7-29에
//   적어둔 "위성이 뜨는 자리에 특징을 배치" 원칙을 여기도 적용 — 북/남 위성
//   축으로 뻗는 지느러미 삼각형을 몸통에 붙여 그 이탈을 등/배지느러미로
//   흡수한다(서쪽 위성은 그대로 눈처럼 남겨둠 — 그건 이미 잘 읽혔다).
//   포크는 FORK_START를 앞당기고 FORK_GAP을 키워 축소본에서도 더 또렷하게.
//   ★v4(2026-08-26 오너 재검수 불합격 교정): v3 는 소형(256px)에서 **형태가 성립하지
//   않았고** 용량 336B 로 전 실루엣 최저, 중심어긋남 5.8% 로 최악이었다. 원인 3가지:
//     ① 몸통이 작아 데이터가 안 들어감(밀도 부족 → 소형에서 흩어져 보임)
//     ② 질량이 왼쪽(몸통)에 쏠렸는데 꼬리만 오른쪽으로 뻗어 **무게중심이 밀림**
//     ③ 꼬리 포크가 가늘어 축소하면 사라짐
//   교정: 몸통을 크게 키우고(0.64→0.74) 중심을 오른쪽으로 당겨 균형을 맞추며,
//   꼬리를 **짧고 두껍게** 바꿔 256px 에서도 갈라짐이 남게 했다.
//   ★중심 정합은 **부호를 재서** 맞춘다(눈대중 금지). v4-a 는 x=-2.20·y=-2.51모듈로
//   왼쪽·위로 쏠렸다 — x 는 과교정이었고, y 는 **부스터 QR 예약이 아래쪽 질량을
//   빼가서** 생긴 구조적 쏠림이다. 둘 다 몸통 중심 이동으로 상쇄한다.
var FISH_CX = -0.065;                 // 몸통 중심 x — 꼬리 무게 상쇄
var FISH_CY = 0.045;                  // 몸통 중심 y — 부스터 예약(아래) 상쇄
var FISH_RX = 0.74, FISH_RY = 0.60;
var FISH_TAPER_X0 = 0.30, FISH_TAIL_TIP = 0.97;
var FISH_FORK_START = 0.34, FISH_FORK_GAP = 0.15;
var FISH_DORSAL = [[-0.22, -FISH_RY + 0.06], [0.02, -FISH_RY + 0.02], [-0.10, -0.82]];
var FISH_VENTRAL = [[-0.20, FISH_RY - 0.06], [0.00, FISH_RY - 0.02], [-0.10, 0.80]];
function pointInFish(nx, ny) {
  var bx = nx - FISH_CX, byy = ny - FISH_CY;
  // 몸통(타원) + 등/배지느러미
  if (bx <= FISH_TAPER_X0) {
    if ((bx * bx) / (FISH_RX * FISH_RX) + (byy * byy) / (FISH_RY * FISH_RY) <= 1) return true;
    return pointInTriangle(nx, ny, FISH_DORSAL) || pointInTriangle(nx, ny, FISH_VENTRAL);
  }
  // 꼬리 — 짧고 두껍게, 끝만 갈라진다
  if (bx > FISH_TAIL_TIP) return false;
  var halfAtX0 = FISH_RY * Math.sqrt(Math.max(0, 1 - (FISH_TAPER_X0 * FISH_TAPER_X0) / (FISH_RX * FISH_RX)));
  var frac = (bx - FISH_TAPER_X0) / (FISH_TAIL_TIP - FISH_TAPER_X0);
  var totalHalf = halfAtX0 * (1 - 0.45 * frac);        // 덜 좁아진다(두꺼운 꼬리)
  if (frac < FISH_FORK_START) return Math.abs(byy) <= totalHalf;
  var forkT = (frac - FISH_FORK_START) / (1 - FISH_FORK_START);
  var gapNow = FISH_FORK_GAP * forkT;
  var lobe = totalHalf - gapNow;
  if (lobe <= 0) return false;
  return Math.abs(byy) > gapNow && Math.abs(byy) <= gapNow + lobe;
}

// ── 피아노 건반(piano): 둥근 사각 몸통 + 하단 빗살(콤) 절개 ─────────────────
//   상단 KEYS_COMB_Y0까지는 통짜(건반 뒤판), 그 아래는 KEYS_TEETH개의 빗살로
//   나눠 각 주기의 KEYS_TOOTH_FRAC만 남기고 나머지를 파낸다(건반 사이 홈).
//   실루엣만으로 "건반열"임을 알아보게 하는 것이 목적 — 실제 흑백건반 배치는
//   재현하지 않는다(캡션 없이 연상만 목표, §질문의 품질기준).
//   ★v2(참조 실물 반영, 2026-08-26): v1은 "아래로 뻗은 빗살"이라 건반이 아니라
//   빗·성벽으로 보였다. 진짜 건반은 **위에서 내려다본 판**이다 —
//   ①전체가 사각 판 ②흰건반 경계가 **가는 세로 틈** ③검은건반이 **위에서 꽂힌 막대**,
//   그것도 2개-3개 묶음(옥타브 문법). 흑백 2단계뿐이라 검은건반은 **비움**으로 표현한다
//   (도트밭 위의 흰 막대 = 최대 대비, 형태가 가장 잘 읽힌다).
//   ★v3(2026-08-26 오너 재검수 불합격 교정): v2 는 소형(256px)에서 **세로 줄무늬**로만
//   읽혔고 불스아이가 줄무늬 사이에 끼어 "건반 끝으로 쏠린" 인상을 줬다. 세 가지 교정:
//     ① 흰건반 14→**7개**(1옥타브) — 소형에서 건반 하나가 2배 넓어진다(최대 효과)
//     ② **중앙 가로띠를 통짜로** — 검은건반은 위쪽에서만, 흰건반 홈은 아래쪽에서만
//        파서 중앙에 잉크 띠를 남긴다. 불스아이가 덩어리 위에 앉는다(쏠림 해소)
//        + 실제 건반을 위에서 본 모습과도 같다(검은건반 위, 흰건반 홈 아래)
//     ③ 검은건반을 넓고 깊게 — 2개·3개 묶음(옥타브 문법)이 소형에서도 읽히게
//   ★v3-b 재교정: v3-a 는 여전히 통짜 줄무늬였다. 원인 — **검은건반과 흰건반 홈이
//     같은 x(경계)** 라 위아래 비움이 한 줄로 이어져 중앙 띠가 안 보였다.
//     건반은 **검은건반의 2-3 묶음**으로 알아보는 것이므로 흰건반 홈을 거의 없애고
//     (0.016→0.007) 검은건반을 크게(0.058→0.082) 키웠다.
//   ⚠️남는 한계: 진짜 검은건반은 **어둡지만** 우리는 비움(밝음)뿐이라 흑백이 뒤집힌다.
//     셀을 강제로 채우면 데이터가 깨지므로 이건 원리적으로 못 고친다.
var KEYS_H = 0.90, KEYS_R = 0.05;
var KEYS_WHITE = 7;                     // 흰건반 수(1옥타브) — 소형 가독성 최우선
var KEYS_GAP = 0.007;                   // 흰건반 경계 홈 — 아주 가늘게(아래 주석)
var KEYS_BLACK_W = 0.082;               // 검은건반 반폭 — 크게(소형 가독성의 핵심)
//   ★검은건반은 **건반 전체 길이를 쓰지 않는다** — 실제 피아노도 뒤쪽에 뒷판(rail)이
//   남는다. 위를 KEYS_BLACK_TOP 에서 시작해 그 위는 통짜로 두면 ①더 정확하고
//   ②위쪽 질량이 보존돼 무게중심이 아래로 쏠리지 않는다(v1.2-b).
var KEYS_BLACK_TOP = -0.62;             // 검은건반 시작(이 위는 뒷판=통짜)
var KEYS_BLACK_Y = -0.05;               // 검은건반이 내려오는 하한(중앙 위에서 멈춤)
var KEYS_BAND_LO = -0.05, KEYS_BAND_HI = 0.10;   // ★중앙 통짜 띠(불스아이 안착)
function pointInPianoKeys(nx, ny) {
  if (!pointInRoundedSquare(nx, ny, KEYS_H, KEYS_R)) return false;
  // ★중앙 가로띠는 무조건 잉크 — 불스아이·오르빗이 덩어리 위에 앉는다
  if (ny >= KEYS_BAND_LO && ny <= KEYS_BAND_HI) return true;
  var span = 2 * KEYS_H, w = span / KEYS_WHITE;
  var u = (nx + KEYS_H) / w;
  var k = Math.floor(u);
  if (ny > KEYS_BAND_HI) {
    // 아래쪽: 흰건반 경계 홈
    var edge = Math.round(u);
    if (edge > 0 && edge < KEYS_WHITE && Math.abs(u - edge) * w < KEYS_GAP) return false;
    return true;
  }
  // 위쪽 뒷판(rail) — 통짜로 남긴다
  if (ny < KEYS_BLACK_TOP) return true;
  // 검은건반(비움) — 흰건반 **경계에 중앙 대칭**으로 놓는다.
  //   ★버그 수정(2026-08-26, 오너 지적): 예전엔 k=floor(u) 로 잡고 |u-k| 를 재서
  //   경계의 **오른쪽 절반만** 비웠다. 그래서 검은건반이 ①폭이 의도의 절반이고
  //   ②한쪽으로 치우쳐, 가운데 두 개가 서로 붙어 보였다.
  //   지금은 가장 가까운 경계(round)를 기준으로 양쪽 대칭으로 비운다 → 폭이 균일하다.
  var edge = Math.round(u);
  if (edge > 0 && edge < KEYS_WHITE) {
    var inOct = ((edge % 7) + 7) % 7;
    if (inOct === 1 || inOct === 2 || inOct === 4 || inOct === 5 || inOct === 6) {
      if (Math.abs(u - edge) * w < KEYS_BLACK_W) return false;
    }
  }
  return true;
}

// ── 산업-4. rose (장미) ★lid 대체 간판 — 수제 정본 ────────────────────────
//   오너 참조의 **미학 방향만** 따른다(그 이미지는 비데이터 일러스트다). 여기서는
//   불스아이 1개·부스터 포함·실데이터로 우리 문법으로 재창조한다.
//   기법 = **내부 백선(void)** — 규격 v1.1/v1.2 의 속살형 조항 그대로:
//     ① 물결 외곽(꽃잎 끝) ② 로그나선 골을 비워 꽃잎을 가른다 ③ 중앙은 잉크.
//   ★골 폭을 **반경에 반비례**하게 잡는 것이 핵심이다 — 고정 위상폭으로 하면
//   중심으로 갈수록 골이 얇아져 3셀 미만이 되고(v1.1 위반) 나선이 뭉갠다.
//   ★v2 튜닝(256px 소형 게이트 통과용): 나선 바퀴수를 2.0→2.8 로 늘리고(b↓)
//   외곽 물결을 깊게(A↑), 골을 살짝 넓혀(3.8셀) 축소본에서도 나선이 읽히게 했다.
var ROSE_LOBES = 9;             // 외곽 물결(꽃잎 끝) 개수
var ROSE_R0 = 0.85, ROSE_LOBE_A = 0.080;
var ROSE_B = 0.060;             // 로그나선 성장률 — 중심~가장자리 약 2.8바퀴
var ROSE_GROOVE_HALF = 0.030;   // 골 반폭(정규화) ≈ 3.8셀
var ROSE_CORE = 0.30;           // 이 반경 안은 통짜 잉크(중앙 30% 존 규칙)
function pointInRose(nx, ny) {
  // ★180° 회전(오너 확정 2026-08-27) — 회전을 **기하에 구워 넣는다.**
  //   §1-1: 방향 정본은 엔진 출력 그대로이고, 렌더·표시 계층에서 임의 회전 금지다.
  //   그래서 CSS/캔버스 transform 이 아니라 여기서 좌표를 뒤집는다(= θ+π).
  nx = -nx; ny = -ny;
  var r = Math.sqrt(nx * nx + ny * ny);
  var th = Math.atan2(ny, nx);
  // ① 물결 외곽
  var R = ROSE_R0 + ROSE_LOBE_A * Math.cos(ROSE_LOBES * th);
  if (r > R) return false;
  // ③ 중앙은 잉크 — 불스아이가 점 구름 속에 앉는다
  if (r <= ROSE_CORE) return true;
  // ② 로그나선 골(void). 위상 f 의 골 반폭을 반경에 반비례로 잡아 **골의 실폭을 일정하게**
  var g = ROSE_GROOVE_HALF / (r * 2 * Math.PI * ROSE_B);
  if (g > 0.34) g = 0.34;                         // 안쪽에서 골이 꽃잎을 삼키지 않도록
  var phase = (Math.log(r) / ROSE_B - th) / (2 * Math.PI);
  var f = phase - Math.floor(phase);
  var d = f < 0.5 ? f : 1 - f;
  if (d < g) return false;
  return true;
}

// ── H-1. ear (귀) ★인체 팩 최우선 ─────────────────────────────────────────
//   외이(pinna) 윤곽: 바깥 타원(helix) − 안쪽 절개(concha 그늘) + 이수(귓불).
//   서 위성 = 이주(tragus) 자리, 북 위성 = helix 상단 너머.
//   ★v2(2026-08-26 자체 재검토): v1(타원−얇은 초승달)은 그냥 도넛/"0"으로 읽혀 기각.
//   교훈 — 귀의 연상성은 **오목한 그릇(concha)이 앞쪽으로 열린 C자**에서 온다.
//   그 void 는 얇으면 안 되고 **굵어야** 형태가 읽힌다(hex 대비 실루엣 면적 손해를 감수).
//   구성: 비대칭 달걀(위 넓고 아래로 좁아짐) + 귓불 + 앞(동쪽)으로 열린 굵은 C 절개 +
//   이주(tragus) 돌기. 서 위성 = helix 뒤쪽 능선, 북 위성 = helix 상단 너머(§11 원칙 2).
function pointInEar(nx, ny) {
  // ── 외곽: 위가 넓고 아래로 갈수록 좁아지는 달걀(귀 윤곽)
  var t = (ny + 0.92) / 1.84;                        // 0(위)~1(아래)
  var halfW = 0.62 * (1 - 0.42 * Math.pow(Math.max(0, t - 0.35) / 0.65, 1.7));
  var cxShift = -0.10 + 0.16 * t;                     // 아래로 갈수록 살짝 앞으로
  var inOuter = (ny >= -0.92 && ny <= 0.66) && Math.abs(nx - cxShift) <= halfW;
  // 위쪽 둥근 마감
  if (ny < -0.62) {
    var tx = (nx - (-0.06)) / 0.60, ty = (ny + 0.62) / 0.32;
    inOuter = (tx * tx + ty * ty) <= 1;
  }
  // ── 귓불: 아래로 늘어진 둥근 덩어리(외곽과 이어짐)
  var lx = (nx - 0.06) / 0.30, ly = (ny - 0.70) / 0.26;
  var inLobe = (lx * lx + ly * ly) <= 1;
  if (!inOuter && !inLobe) return false;

  if (inLobe && !inOuter) return true;                // 귓불은 통짜

  // ── ★v3(참조 실물 반영): 귀는 **2겹**이다 — 바깥 이륜(helix) 테두리와
  //   안쪽 대이륜(antihelix) 능선. v2는 C 한 겹뿐이라 평평했다.
  //   타원 반경 er 로 층을 나누고, 각 층의 틈을 **앞(동쪽)만 남기고** 파낸다.
  var ex2 = (nx + 0.02) / 0.60, ey2 = (ny + 0.02) / 0.86;
  var er = Math.sqrt(ex2 * ex2 + ey2 * ey2);
  var th = Math.atan2(ey2, ex2);                      // 0 = 동(앞)

  // 1) 이륜 틈 — 테두리 한 겹을 띄운다(뒤쪽만 비움 → 앞은 붙어 있어 통짜로 이어짐)
  if (er > 0.62 && er < 0.80 && Math.abs(th) > 0.78) return false;
  // 2) 이갑개(concha) — 중앙의 굵은 C, 앞으로 열림
  if (er > 0.17 && er < 0.47 && Math.abs(th) > 0.62) return false;

  // ── 이주(tragus): 앞쪽 입구를 살짝 가리는 작은 돌기(항상 솔리드)
  var gx = (nx - 0.34) / 0.13, gy = (ny - 0.10) / 0.17;
  if (gx * gx + gy * gy <= 1) return true;
  return true;
}

// ── H-2. palm (손바닥 + 다섯 손가락) ──────────────────────────────────────
//   손바닥 둥근 사각 + 손가락 4개(위) + 엄지(서). 가운뎃손가락 끝이 북 위성에 닿는다.
//   ★v2(2026-08-26 자체 재검토): v1 은 손가락이 짧고 손바닥에 묻혀 그냥 덩어리로 읽혔다.
//   교훈 — 손의 연상성은 **길이와 사이 틈**에서 온다. 손가락을 위성 축까지 길게 뽑고
//   (가운뎃손가락 끝이 북 위성 자리, §11 원칙 2) 손바닥을 줄여 틈을 확보한다.
//   ★v3(참조 실물 반영): v2는 손가락 4개가 같은 길이·같은 굵기라 "빗" 처럼 보였다.
//   진짜 손의 인상은 **길이 서열**(중지>검지≈약지>새끼)과 **새끼손가락이 낮게 붙는 것**,
//   그리고 **엄지가 손바닥 옆에서 비스듬히 갈라지는 것**에서 온다. 셋 다 반영.
//   중지 끝 = 북 위성 축, 엄지 끝 = 서 위성 방향(§11 원칙 2).
var PALM_FINGERS = [
  { x: -0.30, tip: -0.70, w: 0.086 },   // 검지
  { x: -0.08, tip: -0.90, w: 0.090 },   // 중지 — 가장 길다(북 위성 축)
  { x:  0.14, tip: -0.75, w: 0.086 },   // 약지
  { x:  0.34, tip: -0.47, w: 0.078 },   // 새끼 — 짧고 낮게
];
// 선분까지의 거리(캡슐 판정용) — 엄지처럼 비스듬한 부재에 쓴다.
function segDist2(px, py, ax, ay, bx, by) {
  var vx = bx - ax, vy = by - ay, wx = px - ax, wy = py - ay;
  var t = (wx * vx + wy * vy) / (vx * vx + vy * vy);
  t = t < 0 ? 0 : (t > 1 ? 1 : t);
  var dx = wx - t * vx, dy = wy - t * vy;
  return dx * dx + dy * dy;
}
function pointInPalm(nx, ny) {
  // 손바닥(넓게 — 참조처럼 손목까지 내려온다)
  if (pointInRoundedSquare(nx - 0.00, ny - 0.34, 0.46, 0.15)) return true;
  // 손가락 4개(세로 캡슐, 끝 둥글게)
  for (var i = 0; i < PALM_FINGERS.length; i++) {
    var f = PALM_FINGERS[i];
    var capY = f.tip + f.w;
    if (nx > f.x - f.w && nx < f.x + f.w && ny >= capY && ny < 0.02) return true;
    var dx = nx - f.x, dy = ny - capY;
    if (dx * dx + dy * dy <= f.w * f.w) return true;
  }
  // 엄지: 손바닥 왼쪽 옆구리에서 서-북서로 갈라진다(캡슐)
  if (segDist2(nx, ny, -0.36, 0.30, -0.84, -0.04) <= 0.105 * 0.105) return true;
  return false;
}

// ── H-3. footprint (발바닥 — 신생아 발도장) ───────────────────────────────
//   발바닥 본체(앞볼 넓고 뒤꿈치 좁은 타원 2개 결합) + 발가락 5개(위, 분리).
//   ★v2(참조 실물 반영): v1은 발가락 5개가 비슷한 크기라 "점 다섯"으로 보였다.
//   발도장의 인상은 ①**엄지가 확연히 크고** ②나머지가 **사선으로 작아지며**
//   ③아치가 **안쪽 한쪽만** 파이는 것에서 온다. 셋 다 반영.
var FOOT_TOES = [
  { x: -0.30, y: -0.76, rx: 0.165, ry: 0.185 },   // 엄지 — 확연히 크다
  { x:  0.01, y: -0.82, rx: 0.103, ry: 0.100 },
  { x:  0.22, y: -0.75, rx: 0.090, ry: 0.088 },
  { x:  0.39, y: -0.64, rx: 0.077, ry: 0.076 },
  { x:  0.52, y: -0.50, rx: 0.066, ry: 0.066 },   // 새끼 — 가장 작고 가장 낮다
];
function pointInFootprint(nx, ny) {
  // 앞볼(넓고 둥글다)
  var bx = nx - 0.06, by = ny + 0.30;
  if ((bx * bx) / (0.44 * 0.44) + (by * by) / (0.31 * 0.31) <= 1) return true;
  // 아치 — 중심을 **바깥쪽(오른쪽)으로 밀어** 안쪽 모서리만 오목하게 판다
  var ax = nx - 0.17, ay = ny - 0.16;
  if ((ax * ax) / (0.25 * 0.25) + (ay * ay) / (0.44 * 0.44) <= 1) return true;
  // 뒤꿈치(앞볼보다 좁고 둥글다)
  var hx = nx - 0.05, hy = ny - 0.62;
  if ((hx * hx) / (0.30 * 0.30) + (hy * hy) / (0.31 * 0.31) <= 1) return true;
  // 발가락 5개(본체와 떨어져 있는 게 실제 발도장 문법)
  for (var i = 0; i < FOOT_TOES.length; i++) {
    var t = FOOT_TOES[i], dx = nx - t.x, dy = ny - t.y;
    if ((dx * dx) / (t.rx * t.rx) + (dy * dy) / (t.ry * t.ry) <= 1) return true;
  }
  return false;
}

// ── H-4. brain (뇌 반구 + 주름) ───────────────────────────────────────────
//   위가 부푼 타원 + 좌우를 가르는 세로 틈(대뇌 종렬) + 물결 주름 절개 + 아래 뇌간.
function pointInBrain(nx, ny) {
  // 뇌간(아래로 짧게)
  if (Math.abs(nx - 0.06) <= 0.11 && ny > 0.62 && ny < 0.92) return true;
  var ex = nx / 0.94, ey = (ny + 0.06) / 0.80;
  if (ex * ex + ey * ey > 1) return false;
  // 대뇌 종렬(가운데 세로 틈) — 위쪽 절반만
  if (Math.abs(nx - 0.02) < 0.035 && ny < 0.30) return false;
  // 주름: 사인파 등고선을 따라 얇게 파낸다
  var g1 = Math.sin(nx * 6.1 + ny * 2.3);
  var g2 = Math.sin(ny * 7.4 - nx * 1.9);
  if (Math.abs(g1) < 0.13 && ny < 0.52) return false;
  if (Math.abs(g2) < 0.11) return false;
  return true;
}

// ── 모양 마스크: 데이터 판의 실루엣 (검출 원리와 무관 — 앵커는 그대로) ────────
//   'round'=원판, 'heart'=하트, 그 외=전부(사각). 정규화 좌표(중심0, 반경 Rdata=1).
// ★§7-36 방어 ② 보조표 — 실루엣 → 기하 함수. 없는 것은 null 로 남아 호출되지 않는다.
//   함수 선언은 호이스팅되므로 이 리터럴이 평가될 때 이미 존재한다.
var SHAPE_POINT_FN = {
  'ear':       typeof pointInEar       === 'function' ? pointInEar       : null,
  'palm':      typeof pointInPalm      === 'function' ? pointInPalm      : null,
  'footprint': typeof pointInFootprint === 'function' ? pointInFootprint : null,
  'brain':     typeof pointInBrain     === 'function' ? pointInBrain     : null,
};

// ── 비트맵 실루엣 (2026-08-28) ─────────────────────────────────────────────
//   기존 실루엣은 전부 수식이다(pointInRose 는 cos/sin). 그런데 귀·손·발·뇌처럼
//   **사람이 알아보는** 형상은 수식으로 못 쓴다 — 특히 형상을 살리는 **흰 통로**가
//   자유곡선이라 더 그렇다. 그래서 128×128 비트맵을 base64 로 구워 넣는다.
//   격자 L 과 같은 해상도라 L 에서 정확하고, S/M 은 정규화 좌표로 자연스럽게 내려간다.
//   한 형상당 약 2.7KB. 마스크 생성: ~/wiacode-shape-lab/mask-to-geometry.js
//
//   ★★흰 통로가 형상을 만든다 — 통짜로 메우면 "비슷한 덩어리"가 되고, 골이 있어야
//     "아, 귀네"가 된다. 장미의 `② 로그나선 골(void)`과 같은 원리다.
//     마스크에 구멍을 뚫으면 그 칸은 데이터 칸에서 빠져 흰색으로 남는다.
function decodeMask(n, b64) {
  var raw = typeof Buffer !== 'undefined' ? Buffer.from(b64, 'base64')
          : (function () { var s = atob(b64), a = new Uint8Array(s.length);
              for (var i = 0; i < s.length; i++) a[i] = s.charCodeAt(i); return a; })();
  var bits = new Uint8Array(n * n);
  for (var i = 0; i < n * n; i++) bits[i] = (raw[i >> 3] >> (7 - (i & 7))) & 1;
  return { n: n, bits: bits };
}
/** 정규화 좌표 → 비트맵 조회. 마스크 밖은 false(=데이터 없음). */
function inMask(M, nx, ny) {
  var ix = Math.floor((nx + 1) * 0.5 * M.n), iy = Math.floor((ny + 1) * 0.5 * M.n);
  if (ix < 0 || iy < 0 || ix >= M.n || iy >= M.n) return false;
  return !!M.bits[iy * M.n + ix];
}
// ── ear — 비트맵 실루엣(128×128, 잉크 25.8%)
//   원본 마스크: ear-v6.png
var MASK_EAR = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAgAAAAAAAAAAAAAAAAAAA//+AAAAAAAAAAAAAAAAAA///4AAAAAAAAAAAAAAAAA////gAAAAAAAA' +
  'AAAAAAAAf///+AAAAAAAAAAAAAAAA/////8AAAAAAAAAAAAAACf//v//gAAAAAAAAAAAAAB//PAf/+AAAAAAAAAAAAAA//xg' +
  'C//wAAAAAAAAAAAAA//YAAEP+AAAAAAAAAAAAAf/CAAAD/4AAAAAAAAAAAAH/wgCUA/+AAAAAAAAAAAAD/kIf/8GfwAAAAAA' +
  'AAAAAA/wD///zH+AAAAAAAAAAAAf+A////wfwAAAAAAAAAAAf+AP///+D+AAAAAAAAAAAH/Af////gfwAAAAAAAAAAD/gP//' +
  '//8D8AAAAAAAAAAA/4P/////AfgAAAAAAAAAAP8H/////+D8AAAAAAAAAAH8D//////gfAAAAAAAAAAB/A///8P/+D4AAAAA' +
  'AAAAA/wf//8Af/g+AAAAAAAAAAP8P///AD/8PwAAAAAAAAAD8D///AAP/j8AAAAAAAAAB/B///wAD/8fAAAAAAAAAAfwf//+' +
  'IAf/H4AAAAAAAAAH0P///zAD/4+AAAAAAAAAB4D////wAP+HgAAAAAAAAA+Af////AB/h4AAAAAAAAAHwH////wAf8PAAAAA' +
  'AAAAB8D////+AP/DwAAAAAAAAAfB//+f/gD/wcAAAAAAAAAPwf/wAJsAf+HAAAAAAAAABgH/8AABAD/h4AAAAAAAAAYB//gA' +
  'AAA/4fAAAAAAAAAGAf//AAAAP+HwAAAAAAAABgH///wAAH/x4AAAAAAAAA4B////4AA/+OAAAAAAAAAGAf////AAP/BwAAAA' +
  'AAAABwD////4AB/wcAAAAAAAAA+A/////wAf8PAAAAAAAAAHwP////8AH/BwAAAAAAAAB8B/////gB/wcAAAAAAAAAPgf///' +
  '/4AP8HAAAAAAAAAD4D/////gD/BwAAAAAAAAAfA/4///4Af4cAAAAAAAAAHwH8BBH/AH8PAAAAAAAAAB8A/AAA/wD/DwAAAA' +
  'AAAAAPAH4AAD8A/w8AAAAAAAAAB4A/8AA/gf8PAAAAAAAAAAfgH/AAf4D/DwAAAAAAAAAD4A/4AH+A/48AAAAAAAAAA+AP+H' +
  'j/gP8PAAAAAAAAAAPwD/3//4H/HwAAAAAAAAAB+Af///+B/x4AAAAAAAAAAHwH//+fgP4fAAAAAAAAAAA4D///n4D+HgAAAA' +
  'AAAAAAOA////+B/hwAAAAAAAAAABgf////gf4cAAAAAAAAAAAMH////4H+HAAAAAAAAAAABB///x+B/BwAAAAAAAAAAAQP//' +
  '8fgfwcAAAAAAAAAAACD/j8/wH4HAAAAAAAAAAAAgf4AP8D+DwAAAAAAAAAAAYD/AB/A/g4AAAAAAAAAAAHA/wA/gP4eAAAAA' +
  'AAAAAABwP8Af4D8fgAAAAAAAAAAAcD/9/8B/H4AAAAAAAAAAAHA////Afx+AAAAAAAAAAAB4P///gH//AAAAAAAAAAAAcD//' +
  '/wB8/wAAAAAAAAAAAHh///4D/H4AAAAAAAAAAADw///4A/j8AAAAAAAAAAAA4P//4Afw/AAAAAAAAAAAAOH//gAP8PwAAAAA' +
  'AAAAAAHj//AAH+D4AAAAAAAAAAAB4//gAH/B+AAAAAAAAAAAAff/wAD/wfAAAAAAAAAAAAPv/4AB/wPwAAAAAAAAAAADj/8A' +
  'Af8f4AAAAAAAAAAAA4f+AAf+H8AAAAAAAAAAAAOH/gAP/B/AAAAAAAAAAAADh/wPH/w/gAAAAAAAAAAAA4P4X//4PwAAAAAA' +
  'AAAAAAPD8P//+H4AAAAAAAAAAAADwPD///h8AAAAAAAAAAAAB8AB///wfAAAAAAAAAAAAAPAB///4PgAAAAAAAAAAAAH4A//' +
  '/4HwAAAAAAAAAAAAB/H///8D4AAAAAAAAAAAAAf////+B8AAAAAAAAAAAAAH/////A+AAAAAAAAAAAAAB/5///wfgAAAAAAA' +
  'AAAAAAf+f//8PwAAAAAAAAAAAAAH/n//8H4AAAAAAAAAAAAAB/////P+AAAAAAAAAAAAAAf///+P/AAAAAAAAAAAAAAH/n//' +
  'j/gAAAAAAAAAAAAAB/5//9/4AAAAAAAAAAAAAAf+P///8AAAAAAAAAAAAAAH/D///+AAAAAAAAAAAAAAA/x////gAAAAAAAA' +
  'AAAAAAP/////wAAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAAf////8AAAAAAAAAAAAAAAH//AH+AAAAAAAAAAAAAAAA//wB' +
  '/gAAAAAAAAAAAAAAAP/8If4AAAAAAAAAAAAAAAD//j/8AAAAAAAAAAAAAAAAf/4/+AAAAAAAAAAAAAAAAH////AAAAAAAAAA' +
  'AAAAAAA////gAAAAAAAAAAAAAAAAD///wAAAAAAAAAAAAAAAAAf//4AAAAAAAAAAAAAAAAAB//4AAAAAAAAAAAAAAAAAAH/8' +
  'AAAAAAAAAAAAAAAAAAAPyAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── hand — 비트맵 실루엣(128×128, 잉크 33.1%)
//   원본 마스크: ns1.png
var MASK_HAND = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAQAAAAAAAAAAAAAAAAAAAAeAAAAAAAAAAAAAAAAAAAAP4AAAAAAAAAAAAAAAAAAAH+AAAAAAAAA' +
  'AAAAAAAAAAB/wAAAAAAAAAAAAAAAAAAA/8AAAAAAAAAAAAAAAAAAAP/AAAAAAAAAAAAAAAAAAAD/4APAAAAAAAAAAAAAAAAA' +
  '/+AP4AAAAAAAAAAAAAAAAP/gD/AAAAAAAAAAAAAAAAD/4B/wAAAAAAAAAAAAAfAA/+Af8AAAAAAAAAAAAAP4AP/gH/AAAAAA' +
  'AAAAAAAH+AD/4B/wAAAAAAAAAAAAB/wA/+Af8AAAAAAAAAAAAAf8AP/gH/AAAAAAAAAAAAAH/AD/wB/wAAAAAAAAAAAAB/wA' +
  '/+Af8AAAAAAAAAAAAAf8AP/gP/AAAAAAAAAAAAAH/gD/4D/wAAAAAAAAAAAAB/4A/+Af8AAAAAAAAAAAAAf+AP/gP/AAAAAA' +
  'AAAAAAAH/gD/4D/wAAAAAAAAAAAAB/4A/+A/8AAAAAAAAAAAAAf+AP/gP/AAAAAAAAAAAAAH/wD/4D/wABAAAAAAAAAAA/8A' +
  '/+A/8AB8AAAAAAAAAAP/AP/gf/AA/gAAAAAAAAAD/wD/4H/wAf4AAAAAAAAAA/+A/+B/8AH+AAAAAAAAAAP/gP/gf/AB/gAA' +
  'AAAAAAAD/4D/8H/wAf4AAAAAAAAAA/+A//B/4AP+AAAAAAAAAAP/wP/wf+AD/gAAAAAAAAAB/8D/8P/gA/wAAAAAAAAAAf/A' +
  '//D/4Af8AAAAAAAAAAH/4P/w/+AH/AAAAAAAAAAB/+D/8P/gB/wAAAAAAAAAAf/g//H/4A/4AAAAAAAAAAH/8P/x/8AP+AAA' +
  'AAAAAAAA//D/8f/AH/gAAAAAAAAAAP/wf/H/wB/4AAAAAAAAAAD/8H/x/8Af8AAAAAAAAAAA//B/8f/AP/AAAAAAAAAAAP/w' +
  'f/H/wD/wAAAAAAAAAAB/+H/x/4B/4AAAAAAAAAAAf/h/8f+Af+AAAAAAAAAAAH/4f/H/gP/gAAAAAAAAAAB//P/7/4D/wAAA' +
  'AAAAAAAAP/////+B/8AAAAAAAAAAAD//////gf/AAAAAAAAAAAA//////4H/gAAAAAAAAAAAP/////+D/4AAAAAAAAAAAD//' +
  '////w/+AAAAAAB/AAAA//////+f/AAAAAAA/8AAAP////////wAAAAAAf/wAAD////////4AAAAAAH//AAA////////+AAAA' +
  'AAB//4AAP////////gAAAAAAP/+AAD////////wAAAAAAB//wAA////////8AAAAAAAP/+AAP////////AAAAAAAB//gAB//' +
  '//////wAAAAAAAP/8AA////////8AAAAAAAB//gAP////////AAAAAAAAf/8AD////////wAAAAAAAD//AA////////4AAAA' +
  'AAAA//4Af///////+AAAAAAAAH//AH////////gAAAAAAAB//wD////////4AAAAAAAAP/+A////////+AAAAAAAAB//wP//' +
  '//////gAAAAAAAAf/8H////////4AAAAAAAAD//h////////+AAAAAAAAAf///////////gAAAAAAAAH///////////4AAAA' +
  'AAAAA///////////+AAAAAAAAAP///////////gAAAAAAAAB///////////4AAAAAAAAAf//////////+AAAAAAAAAD/////' +
  '//////AAAAAAAAAA///////////wAAAAAAAAAP//////////8AAAAAAAAAB///////////AAAAAAAAAAf//////////wAAAA' +
  'AAAAAD//////////8AAAAAAAAAA///////////AAAAAAAAAAP//////////wAAAAAAAAAB//////////4AAAAAAAAAAP////' +
  '/////+AAAAAAAAAAB//////////gAAAAAAAAAAP/////////4AAAAAAAAAAB/////////8AAAAAAAAAAAP/////////AAAAA' +
  'AAAAAAD/////////wAAAAAAAAAAAf////////4AAAAAAAAAAAD////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAD///' +
  '/////wAAAAAAAAAAAAf///////8AAAAAAAAAAAAD///////+AAAAAAAAAAAAAf///////gAAAAAAAAAAAAD///////wAAAAA' +
  'AAAAAAAAf//////8AAAAAAAAAAAAAD//////+AAAAAAAAAAAAAAP//////gAAAAAAAAAAAAAB//////wAAAAAAAAAAAAAAH/' +
  '////4AAAAAAAAAAAAAAB/////8AAAAAAAAAAAAAAAP/////AAAAAAAAAAAAAAAD/////wAAAAAAAAAAAAAAA/////8AAAAAA' +
  'AAAAAAAAAP/////AAAAAAAAAAAAAAAD/////wAAAAAAAAAAAAAAA/////8AAAAAAAAAAAAAAAP/////AAAAAAAAAAAAAAAA/' +
  'AAD+AAAAAAAAAAAAAAAAAAAACAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── foot — 비트맵 실루엣(128×128, 잉크 29.9%)
//   원본 마스크: foot.png
var MASK_FOOT = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAD4AAAAAAAAAAAAAAAAAAAD/gAAAAAAAAAAAAAAAAAAD/8AAAAAAAAAAAAAAAAAAA//gAAAAAAAAAAAA' +
  'AAAAAAf/8BwAAAAAAAAAAAAAAAAP//g/AAAAAAAAAAAAAAAAD//4f4AAAAAAAAAAAAAAAB//+P/AAAAAAAAAAAAAAAAf//3/' +
  'wAAAAAAAAAAAAAAAH//9/+PgAAAAAAAAAAAAAB/////n8AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAH//////4AAAAAAAA' +
  'AAAAAB//////+AAAAAAAAAAAAAAf//////zwAAAAAAAAAAAAH///////+AAAAAAAAAAAAA////////wAAAAAAAAAAAAP////' +
  '///8AAAAAAAAAAAAB////////AAAAAAAAAAAAAf///////wAAAAAAAAAAAAD///////8AAAAAAAAAAAAAf///////OAAAAAA' +
  'AAAAAAH////////wAAAAAAAAAAAB////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAH////////4AAAAAAAAAAAD////' +
  '////+AAAAAAAAAAAA/////////gAAAAAAAAAAAf////////4AAAAAAAAAAAH////////8AAAAAAAAAAAD/////////AAAAAA' +
  'AAAAAA/////////gAAAAAAAAAAAP////////4AAAAAAAAAAAD////////8AAAAAAAAAAAB/////////AAAAAAAAAAAAf////' +
  '////wAAAAAAAAAAAH////////8AAAAAAAAAAAB/////////gAAAAAAAAAAAf////////4AAAAAAAAAAAH////////+AAAAAA' +
  'AAAAAB/////////gAAAAAAAAAAAP////////4AAAAAAAAAAAD////////+AAAAAAAAAAAA/////////gAAAAAAAAAAAP////' +
  '////4AAAAAAAAAAAB////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAD////////4AAAAAAAAAAAA////////+AAAAAA' +
  'AAAAAAH////////gAAAAAAAAAAAB////////wAAAAAAAAAAAAP///////8AAAAAAAAAAAAD////////AAAAAAAAAAAAAf///' +
  '////wAAAAAAAAAAAAD///////4AAAAAAAAAAAAA///////+AAAAAAAAAAAAAH///////gAAAAAAAAAAAAB///////wAAAAAA' +
  'AAAAAAAP//////8AAAAAAAAAAAAAD//////+AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAA///' +
  '///8AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAD//////gAAAAAAAAAAAAAA//////4AAAAAAAAAAAAAAH/////+AAAAAAA' +
  'AAAAAAAB//////AAAAAAAAAAAAAAAf/////wAAAAAAAAAAAAAAH/////8AAAAAAAAAAAAAAB/////+AAAAAAAAAAAAAAAf//' +
  '///gAAAAAAAAAAAAAAH/////4AAAAAAAAAAAAAAB/////+AAAAAAAAAAAAAAAf/////gAAAAAAAAAAAAAAH/////4AAAAAAA' +
  'AAAAAAAD/////+AAAAAAAAAAAAAAA//////AAAAAAAAAAAAAAAP/////wAAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAB///' +
  '///AAAAAAAAAAAAAAAf/////wAAAAAAAAAAAAAAH/////8AAAAAAAAAAAAAAD//////gAAAAAAAAAAAAAA//////4AAAAAAA' +
  'AAAAAAAP/////+AAAAAAAAAAAAAAH//////gAAAAAAAAAAAAAB//////4AAAAAAAAAAAAAAf/////+AAAAAAAAAAAAAAP///' +
  '///gAAAAAAAAAAAAAD//////4AAAAAAAAAAAAAA//////+AAAAAAAAAAAAAAP//////gAAAAAAAAAAAAAD//////4AAAAAAA' +
  'AAAAAAA//////+AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAf///' +
  '///gAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAA//////+AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAD//////wAAAAAAA' +
  'AAAAAAA//////8AAAAAAAAAAAAAAP/////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAD///' +
  '//8AAAAAAAAAAAAAAA//////AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAB/////4AAAAAAAAAAAAAAAP////8AAAAAAAA' +
  'AAAAAAAB////+AAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAB////gAAAAAAAAAAAAAAAAH///gAAAAAAAAAAAAAAAAAf/' +
  '/gAAAAAAAAAAAAAAAAAA//AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── brain — 비트맵 실루엣(128×128, 잉크 47.7%)
//   원본 마스크: brain-v5.png
var MASK_BRAIN = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAD/AAD/AAAAAAAAAAAAAAAH/8AD/2AAAAAAAAAAAAAAH//gB//wAAAAAAAAAAAAAD//8A///AAAAAAA' +
  'AAAAAAP///gf///AAAAAAAAAAAAH///8H///4AAAAAAAAAAAH/P//B//x/gAAAAAAAAAAB/j/////8f4AAAAAAAAAAA/g///' +
  '///B/AAAAAAAAAAAPw//////8P4AAAAAAAAAAP8P/j/8f/D/AAAAAAAAAAP/D/4f+D/w/8AAAAAAAAAD/x/+H/g/+P/AAAAA' +
  'AAAAA/+f/h/4f/n/4AAAAAAAAA/P////+P//8/AAAAAAAAAfx////////+H4AAAAAAAAP4cH/w///8Dz/AAAAAAAAH/PB/8P' +
  '///A//4AAAAAAAD//gf/D///wH//AAAAAAAA//4f/x/8f/g//wAAAAAAAP/8P/4//D/8H/8AAAAAAAD/+D/8P/w//A//AAAA' +
  'AAAB//B//D/8P/4P/8AAAAAAAf/x//x//D7/D//AAAAAAAPn8P4/f/74fw//4AAAAAAH4+D+H///+D8H/+AAAAAAD+fg/B//' +
  '//g/B//wAAAAAA/j4fw////4f4f/+AAAAAAfh8H8P///+H+B8fgAAAAAHweD/j3//7h/4eD8AAAAAB8Hg//gf/4f//HgOAAA' +
  'AAAfAwPP4D/8H//gADwAAAAAPwODx+Af+B//4AA8AAAAAH/jg8f+D/B//+EH/gAAAAB//8H//w/w//+Bz/8AAAAA///g//8P' +
  '8P//h//3AAAAAP//4Mf/h/D+YAf/4wAAAAD///ADH8//+AAf/+OAAAAB//zwAAf//+AADz/jgAAAAf/4eMAD///gAB4/48AA' +
  'AAP/+H/gA///4Af8P//AAAAH////8f//4/+H/D//4AAAB////+H/H+D/g/x//+AAAAfz///B/x/g/4H////gAAAP8f//Af8f' +
  '+P+A////8AAAD/D//gf+H/h/8H4///AAAA/4fj4P8B/4H/gcPn/wAAAP/D4+P/Af+A/8HD4/8AAAD/w+MP/x//+P/gx+P/AA' +
  'AA/+f/D/wf//3/8H/3/wAAAP///wf8f///P/D///8AAAH/5/8HvH///h3g/+f/gAAB/4D/BgA///wAcP8B/8AAAf4Af4AAP/' +
  '/8AAD+AH/AAAH8AD+AAA//8AAA+AA/gAAB+BgfwH/H/+P+AfAcH8AAAfg+D8B/w//j/wHwPh/AAAP4fw/x/8fn4/8D8P8P4A' +
  'AD+P+H8f//w///j/H/D+AAB/H/z////4H//////4/gAAfx//////+BH//////H4AAH8///////gB//////w+AAB///+P/D//' +
  '8P4/8f/8PgAA+Pj/h/w///D8H+H/Hh8AAPj4/4P4P//9/h/A/x4fAAD4+D8AYD////4HAPwfHwAA//gfAAB/////AAD8H/8A' +
  'AH//H+AA/////4AH/H/+AAB+P7/wA//////AD/z8fgAAfj/3+Af/////8B/v/D4AAP4f4/wf//////gfw/g/AAB+P8H+H///' +
  '///4P4P8fgAAP//D/wf/////+P/D//4AAH//4/8Dj///8MD/w/8eAAD//+P/wQ////AD/+P/HwAA///if8CP///wA/+D/x8A' +
  'AH//4D/xj///8A//A//+AAB//+A//////////gP//gAAP8/4f/////////8P8f4AAD+H+P////////w/H+H+AAA/h/j//+P/' +
  '/+f8Hx/h/AAAH+P9/H/B//+D/A//w/gAAB/Bf4B/wH//gf4B/4P4AAAPwD8Af8A//4P/AP8D+AAAD4A/A//8P////8D/AfAA' +
  'AA+DfwP8P3///j/Af4HwAAAPg/4H/D/4P/w/8H/h8AAADw/+H/w/8B/+H/g/8PAAAAcP/D/4P/Af/g/8H/DgAAADj/g/8H/4' +
  'H/8P/gHx4AAAA8/AP/A//B/+D/4B8eAAAAP/gD/wH/wf+A/8AfHAAAAB/8wcfA/8H/gfODP/wAAAAf/+HH4D/D/YPhh//4AA' +
  'AAD//gw/gf//j/wQf/8AAAAAf/8MP8D//4/8MP/+AAAAAB///n/h//+P/nn//AAAAAAf////8f//j/////gAAAAAD///7///' +
  '///n///wAAAAAAf//wP////xwP//4AAAAAAD//8B////8cD//8AAAAAAAf//AP////GA//+AAAAAAAD//+D/////g///AAAA' +
  'AAAAP//h/////4f//AAAAAAAAAf/4f////+D//AAAAAAAAAD/8H/////g//gAAAAAAAAA//D/////8P/wAAAAAAAAAH/////' +
  '//z//4AAAAAAAAAB/////n/4//+AAAAAAAAAAP////w/+P//AAAAAAAAAAA////8P////gAAAAAAAAAAP///+B////wAAAAA' +
  'AAAAAB////gf/z/4AAAAAAAAAAAH///wD/8f4AAAAAAAAAAAAf//4Af/H4AAAAAAAAAAAAB//8AD//8AAAAAAAAAAAAAD/8A' +
  'AP/gAAAAAAAAAAAAAAH+AAB/gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── eye — 비트맵 실루엣(128×128, 잉크 25.0%)
//   원본 마스크: eye.png
var MASK_EYE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAMAAAAAAAAAAAAAAAeAAAACAAAAAAAAAAAAAB///8AABgAAAAAAAAAAAAH////8AA4AQAAA' +
  'AAAAAAAP/////4AcAMAAAAAAAAAAf//////wPADAAAAAAAAAAf///////ngBgAAAAAAAAAf////////wBwBAAAAAAAAf////' +
  '////8A8AQAAAAAAAf/////////g+AIAAAAAAAP/////////+/AGAAAAAAAP///////////gDAAAAAAAH///////////wDwAA' +
  'AAAAH///////////8B4AAAAAAD////////////h8AQAAAAB/////////////+AMAAAAA/////////////+AOAAAAAf//////' +
  '///////AHAAAAAP/////////////wPgAAAAH///////////////wAAAAD///////////////wAAAAB///////////////wAA' +
  'AAA////H/////4////4AYAAAf///B/////+B///+AcAAAP///A//////wP////+AAAH///AP/////8A////+AAAD///AH///' +
  '///gD///8AAAA///gB//////4Af//+AAAAf//gA//4////AB///gAAAP//wAP/8H///wAP//8AAAH//4AD//B///8AA///AA' +
  'AB//8AA//wf///gAH//4AAA//8AAf/8H///4AA//+AAAf/+AAH//////+AAH//wAAH//AAB///////gAA//+AAD//gAAf///' +
  '///4AAH//gAB//wAAH//////+AAA//8AAf/4AAB///////gAAP//AAP/8AAAf//////4AAD//4AD/+AAAH//////+AAB//+A' +
  'B//gAAB///////gAA//+AAf/8AAAP//////wAAP//AAA//gAAD//////8AAH/+AAAH/8AAA///////AAD/+AAAA//wAAH///' +
  '///gAB/+AAAAA/+AAB//////4AA//AAAAAB/wAAP/////+AAf/AAAAAAH/AAD//////AAP/gAAAAAAf4AAf/////gAP/gAAA' +
  'AAAB/gAD/////4AH/wAAAAAAAH8AA/////8AH/wAAAAAAAA/wAH////+AD/4AAAAAAAAD/AA/////AD/4AAAAAAAAAP8AD//' +
  '//gD/4AAAAAAAAAB/wAf///gD/8AAAAAAAAAAH/gB///wH/8AAAAAAAAAAAf/AP//8P/+AAAAAAAAAAAD//D/////+AAAAAA' +
  'AAAAAAP///////+AAAAAAAAAAAAA///////+AAAAAAAAAAAAAD//////+AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAAP//' +
  '//4AAAAAAAAAAAAAAAAf///wAAAAAAAAAAAAAAAAAP/+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── nose — 비트맵 실루엣(128×128, 잉크 42.4%)
//   원본 마스크: nose.png
var MASK_NOSE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAP/wAAAAAAAAAAAAAAAAAAf//gAAAAAAAAAAAAAAAAA////AAAAAAAAAAAAAAAAAf///4AAAAAAAA' +
  'AAAAAAAAf////gAAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAA///' +
  '///AAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAD//////wAAAAAAAAAAAAAA//////8AAAAAAA' +
  'AAAAAAAP//////AAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAH///' +
  '///4AAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAD///////AAAAAAA' +
  'AAAAAAA///////wAAAAAAAAAAAAAP//////8AAAAAAAAAAAAAD///////AAAAAAAAAAAAAA///////wAAAAAAAAAAAAAP///' +
  '///8AAAAAAAAAAAAAD///////AAAAAAAAAAAAAA///////wAAAAAAAAAAAAAf//////8AAAAAAAAAAAAAH///////AAAAAAA' +
  'AAAAAAB///////4AAAAAAAAAAAAAf//////+AAAAAAAAAAAAAH///////gAAAAAAAAAAAAB///////4AAAAAAAAAAAAAf///' +
  '///+AAAAAAAAAAAAAH///////gAAAAAAAAAAAAB///////4AAAAAAAAAAAAAf//////+AAAAAAAAAAAAAP///////wAAAAAA' +
  'AAAAAAD///////8AAAAAAAAAAAAA////////AAAAAAAAAAAAAP///////wAAAAAAAAAAAAD///////8AAAAAAAAAAAAA////' +
  '////AAAAAAAAAAAAAf///////wAAAAAAAAAAAAH///////+AAAAAAAAAAAAB////////gAAAAAAAAAAAAf///////4AAAAAA' +
  'AAAAAAH///////+AAAAAAAAAAAAD////////wAAAAAAAAAAAA////////8AAAAAAAAAAAAP////////AAAAAAAAAAAAH////' +
  '////4AAAAAAAAAAAB////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAP////////8AAAAAAAAAAAD/////////AAAAAA' +
  'AAAAAB/////////4AAAAAAAAAAAf////////+AAAAAAAAAAAP/////////wAAAAAAAAAAH/////////+AAAAAAAAAAD/////' +
  '/////wAAAAAAAAAB//////////+AAAAAAAAAA///////////wAAAAAAAAAf///////////AAAAAAAAAf///////////4AAAA' +
  'AAAAf////////////gAAAAAAAP////////////8AAAAAAAP/////////////wAAAAAAH/////////////+AAAAAAD///////' +
  '///////wAAAAAB//////////////+AAAAAA///////////////wAAAAAf//////////////+AAAAAH///////////////wAA' +
  'AAD///////////////8AAAAA////////////////gAAAAf///////////////4AAAAH////////////////AAAAD////////' +
  '////////wAAAA////////////////8AAAAP////////////////gAAAH////////////////4AAAB////////////////+AA' +
  'AAf////////////////gAAAH//+D////////wf//4AAAB//+AP///////wB//+AAAAf//AA///////wAP//gAAAH//gAD///' +
  '///wAB//4AAAB//4AAf/////4AAP/+AAAAf/8AAD/////8AAD//gAAAH//AAAf/////AAA//4AAAB//wAAH/////gAAP/+AA' +
  'AAf/8AAB/////4AAD//gAAAD//gAAf////+AAA//wAAAA//4AAH/////gAAf/8AAAAP//AAB/////4AAP//AAAAB//4AAf//' +
  '//+AAH//gAAAAf//AAP/////wAD//4AAAAD//8AH/////+AD//8AAAAAf//wB//////wD///AAAAAH///3///////v///gAA' +
  'AAA///////////////wAAAAAH//////////////4AAAAAA//////////////8AAAAAAD/////////////+AAAAAAAf//////' +
  '//////+AAAAAAAB////////////+AAAAAAAAH///////////+AAAAAAAAAH//////////4AAAAAAAAAAAAf/////4AAAAAAA' +
  'AAAAAAAB/////4AAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAA////8AAAAAAAAAAAAAAAAD///8AAAAAAAAAAAAAAAAAP/' +
  '/8AAAAAAAAAAAAAAAAAAP/wAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── butterfly — 비트맵 실루엣(128×128, 잉크 46.0%)
//   원본 마스크: butterfly.png
var MASK_BUTTERFLY = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAPwAAAAAAAAAAAAAAAPwA' +
  'AP/wAAAAAAAAAAAAAA//AAH//AAAAAAAAAAAAAA//4AD//+AAAAAAAAAAAAB///AB+P/4AAAAAAAAAAAB//H4AfA//AAAAAA' +
  'AAAAAA//A+AHwD/8AAAAAAAAAAA//gPgB8Af/gABAAAAAIAAf/gD4AfgD/+AA8AAAAPAAf/4B+AD+Af/wAfgAAAH4AP/8B/A' +
  'Af4H3+AH8AAAD+AH++B/gAH/g4f4A/gAAB/AH+HB/4AA/+GB/AH8AAA/gB+Bx/+AAPHxgH4AfAAAPgB+Ac+HAADgf8A/AD4A' +
  'AHwA/AP+BwAA4D/gH4AOAABwAfgH/AcAAPAf+A/ADwAA8APwH/gPAAD4D/4H4AeAAeAH4H/wHwAA/x//g/ADgAHAB8H/+P8A' +
  'AP///+DwAYADgA+H////AAD////weAHAA4AfD////wAA8H+/+DwAwAMAPh/9/g8AAPAfH/geAOAHAHwf+PgPAABwDx/+HwBg' +
  'BgD4f/jwDgAAeAf//w+AYAYB8P/94B4AAH4H///HgDAMAeH//+B+AAB/////58AwDAPn/////gAAf//////gEAwH//////4A' +
  'AH5/////8BgYD/////5+AAD4B/////AYGA/////gDwAA8AH////4CBAf////gA8AAPAB/////AwQP////4APAAD4A/////wM' +
  'MD/////AHwAA/j//j//+BCB///n//H8AAH///4f//wQgf//h////AAB/8P+A//8EIP//Af8P/gAAf8D/wP//gkH//wP/A/4A' +
  'AH8B//3//4PB//8//4D+AAA+A//////H4//////AfAAAPgf/////x+P/////4DwAADwP/////+////////A8AAAcD///////' +
  '///////wPAAAHg//////////////8HgAAB4H/////////////+B4AAAfB//////////////g+AAAH8f/////////////4/gA' +
  'AB/////////////////4AAAP////////////////+AAAD/////////////////AAAA/j/////////////8fwAAAH4///////' +
  '///////H4AAAA+f/////////////5+AAAAH////////////////AAAAB////////////////gAAAAH///H///////j///wAA' +
  'AAA//gB//4/x//4Af/wAAAAAD4AAf/+H4f//AAHwAAAAAAAAB///j/H//+AAAAAAAAAAAf//////////gAAAAAAAAA//////' +
  '//////AAAAAAAAH/////////////gAAAAAAP//////////////AAAAAAf//////////////+AAAAAP///////////////wAA' +
  'AAH/////////////+f+AAAAD/x/////n5/////j/wAAAB/8f////5+P////4/+AAAAf+P/z//8fj//8//H/gAAAH/j/4///H' +
  '4///H/x/4AAAB/w//f//x+P//z/8P+AAAAP8P////8fj/////D/gAAAD/B///8/H4fP///g/wAAAA/4B///Pg8Hz//+Af8AA' +
  'AAf+Af//z4fh8///gH/gAAAH/wH///+H4f///4D/4AAAD//D////h+H////D//AAAA//w+f//4fh////w//wAAAf/8fn//+H' +
  '4f//5+P/+AAAH/eHw///B+D//8fj3/gAAB/Dh8P//wfg///H4cf4AAAfg4fgH/8DwP/4B+HB+AAADweH4B//A8D/+AfhwPAA' +
  'AA8Ph/A//wPA//wH4fDwAAAHH4f4//8H4P//H+H44AAAA/+D////A8D////D/8AAAAP/wP///wPA////A//AAAAD+cAf//8D' +
  'wP//+AOfwAAAA+HgB///A8D//+AHj8AAAAPh8Af//gPA///gD4fAAAADw/wD+P4BgH8/wD+DwAAAB8P/A/j+AYB/H8D/w+AA' +
  'AAfHn4H5/gGAf7+B+ePgAAAH/x/A//4AAH//A/D/4AAAB/4f4A/8AAA/8Afw/+AAAAP+H/AP/AAAP/AP+H/AAAAB/B/4H/gA' +
  'AD/4H/g/gAAAAPw8f//4AAAf//48PwAAAAD8OH//+AAAH//+PD4AAAAAfHh///AAAA///h4+AAAAAHzw8//gAAAH/88fPgAA' +
  'AAA/8OP/4AAAB//HD/wAAAAAP/Dj/8AAAAP/xw/8AAAAAD/x4/+AAAAB/8eP/AAAAAB/8+P/AAAAAP/Hz/4AAAAAf//n/gAA' +
  'AAB/5//+AAAAAH////gAAAAAH////gAAAAB////AAAAAAAP///4AAAAAf+H/gAAAAAAB/w/+AAAAAH/APwAAAAAAAPwD/gAA' +
  'AAB/gAAAAAAAAAAAAf4AAAAAPwAAAAAAAAAAAAD8AAAAABwAAAAAAAAAAAAAOAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── earth — 비트맵 실루엣(128×128, 잉크 49.4%)
//   원본 마스크: earth.png
var MASK_EARTH = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAf///4AAAAAAAAAAAAAAAAf////gAAAAAAAAAAAAAAA//////AAAAAAAAAAAAAAB//////+AAAAAAA' +
  'AAAAAAB///////4AAAAAAAAAAAAB////////gAAAAAAAAAAAB//n/////+AAAAAAAAAAAB/+B//////4AAAAAAAAAAA/+A//' +
  '/////gAAAAAAAAAA//AY//////8AAAAAAAAAA//wAA/D///DwAAAAAAAAAf/AAAGAD+fwOAAAAAAAAAP/AAAAAAAB+AwAAAA' +
  'AAAAPwgAAAAAAAP4HAAAAAAAAHAAAAAAAAAD+A4AAAAAAADgAAAAAAAAB/wDAAAAAAABwAAAAAAAAAf8A4AAAAAAA4AAAAAA' +
  'AAAD/APAAAAAAAcAAAAAAAAAA/wH4AAAAAAOAAAAAAAAAQf/D/AAAAAAHAAAAAAAAA8f///4AAAAABgAAAAAAAAf/////AAA' +
  'AAAwAAAAAAAB//////wAAAAAYAAAAAAAH/3////+AAAAAOAAAAAAAD/4/////wAAAADAAAAAAAB/8P////+AAAABgAAAAAAA' +
  '//D/////gAAAAwAAAAAAAP/5/////8AAAAMAAAAAAAB////////gAAAGAAAAAAAAH///////4AAADgAAAAAAAB////////AA' +
  'AA/AAAAAAAAf///////wAAAfgAAAAAAAH///////+AAAHwAAAAAAAD////////gAAD8AAAAAAAA////////8AAA/AAAAAAAA' +
  'f////////AAAfgAAAAAAAP////////4AAH4AAAAAAAD////////+AAB8AAAAAAAB/////////gAA5AAAAAAAH/////////8A' +
  'AMAAAAAAAB//////////AADAAAAAABg//////////wABgAAAAAA+//////////+AAYAAAAAAf///////////gAGAAAAAAD//' +
  '/////////4AB4AAAAAA////////////AA+AAAAAAP///////////wAfgAAAAAD///////////+AHMAAAAAAf///////////g' +
  'BzgAAAAAH///////////4Ac4AAAAAB///////////+AGDgAAAAAf///////////gBg8AAAAAH///////////4AYPAAAAAB//' +
  '/////////+AGDwAAAAA////////////gBg+AAAAAf///////////4AYfwBAAAP///////////+AGH8B4AAH////////////g' +
  'Bj/A+ACf////////////4AZ/wPgB/////////////+AH/8H8A//////////////gB//D/AH/////////////4Af/x/8B////' +
  '/////////+AH/+f/AP/////////////gB//n/4B/////////////4Af////gf////////////+AH////8H/////////////g' +
  'B/////j/////////////4Af//////////////////+AH///////////////////gB///////////////////4AP/////////' +
  '/////////8AB//////3////////////AAf/////w////////////gAH/////4P///////////4AB/////4D///////////+A' +
  'AP////+A////////////AAD/////wf///////////wAA/////8H//7////////8AAH////////+H///////+AAB/////////' +
  'gP///////gAAP////////8D///////4AAD/////////A///////8AAA/////////8////////AAAH/////////////////gA' +
  'AA/////////////////4AAAP///////x////////8AAAB///////4P////////AAAAf//////gD5///////gAAAD//////wA' +
  'cP//////wAAAA//////4AAD//////8AAAAH/////8AAAf/////+AAAAA/////+AAAD//////AAAAAP////8AAAAf/////wAA' +
  'AAB////+AAAAD/////4AAAAAP////gAAAAf////8AAAAAB////4AAAAD////+AAAAAAP///+AAAAA/////AAAAAAB////wAA' +
  'AAP////wAAAAAAP///+AAAAH////4AAAAAAB////gBAAB////8AAAAAAAP///8H+AA////+AAAAAAAB////f/8Af///+AAAA' +
  'AAAAP//////AH////AAAAAAAAB//////4D////gAAAAAAAAP//////B////wAAAAAAAAA///////////4AAAAAAAAAH/////' +
  '/////4AAAAAAAAAAf/////////8AAAAAAAAAAD/////////8AAAAAAAAAAAP////////8AAAAAAAAAAAA////////8AAAAAA' +
  'AAAAAAD///////8AAAAAAAAAAAAAP//////8AAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAA/////4AAAAAAAAAAAAAAAB//' +
  '//gAAAAAAAAAAAAAAAAP///wAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── piano — 비트맵 실루엣(128×128, 잉크 35.6%)
//   원본 마스크: piano.png
var MASK_PIANO = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAH////////////////8AAAH/////////////////wA' +
  'AH/////////////////+AAD//////////////////wAB//////////////////+AAf//////////////////wAP/////////' +
  '/////////8AD///////////////////AB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB/gD/8B//A/8D/8H/8Af4AfwA//AP/wP+A//A//AD+AH8AP/wD/4D/gP/gP/wA/g' +
  'B/AD/8A/+A/4D/4D/8AP4AfwA//AP/gP+A/+A//AD+AH8AP/wD/4D/gP/gP/wA/gB/AD/8A/+A/4D/4D/8AP4AfwA//AP/gP' +
  '+A/+A//AD+AH8AP/wD/4D/gP/gP/wA/gB/AD/8A/+A/4D/4D/8AP4AfwA//AP/gP+A/+A//AD+AH8AP/wD/4D/gP/gP/wA/g' +
  'B/AD/8A/+A/4D/4D/8AP4AfwA//AP/gP+A/+A//AD+AH8AP/wD/4D/gP/gP/wA/gB/AD/8A/+A/4D/4D/8AP4AfwA//AP/gP' +
  '+A/+A//AD+AH8AP/wD/4D/gP/gP/wA/gB/AD/8A/+A/4D/4D/8AP4AfwA//AP/gP+A/+A//AD+AH8AP/wD/4D/gP/gP/wA/g' +
  'B/AD/8A/+A/4D/4D/8AP4AfwA//AP/gP+A/+A//AD+AH8AP/wD/4D/gP/gP/wA/gB/AD/4A/+A/4D/4D/4AP4AfwAf+AH/AP' +
  '+Af8Af8AD+AH8AB+AA/AD/gB+AB+AA/gB/AAPgAPgA/4APgAfAAP4AfwADwAD4AP+ADwADwAD+AH8AA8AA+AD/gA8AA8AA/g' +
  'B/AAPAAPgA/4APAAPAAP4AfwADwAD4AP+ADwADwAD+AH8AA8AA+AD/gA8AA8AA/gB/AAPAAPgA/4APAAPAAP4AfwADwAD4AP' +
  '+ADwADwAD+AH8AA8AA+AD/gA8AA8AA/gB/AAPAAPgA/4APAAPAAP4AfwADwAD4AP+ADwADwAD+AH8AA8AA+AD/gA8AA8AA/g' +
  'B/AAPAAPgA/4APAAPAAP4AfwADwAD4AP+ADwADwAD+AH8AA8AA+AD/gA8AA8AA/gB/AAPAAPgA/4APAAPAAP4AfwADwAD4AP' +
  '+ADwADwAD+AH8AA8AA+AD/gA8AA8AA/gB/AAPAAPgA/4APAAPAAP4AfwADwAD4AP+ADwADwAD+AH8AA8AA+AD/gA8AA8AA/g' +
  'B/AAPAAPgA/4APAAPAAP4AfwADwAD4AP+ADwADwAD+AH+AB+AA+AD/wA+AB8AA/gB/wAfwAfwB/8AfwA/gAf4Af/////////' +
  '/////////+AD///////////////////AA///////////////////wAH//////////////////4AB//////////////////+A' +
  'AP//////////////////AAB//////////////////gAAH/////////////////wAAAf////////////////gAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── dove — 비트맵 실루엣(128×128, 잉크 35.3%)
//   원본 마스크: dove.png
var MASK_DOVE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAABgAAAAAAAAAAAAAAAAAHA' +
  'A8AAAAAAAAAAAAAAAAADwAPAAAAAAAAAAAAAAAAAA8AD4AAAAAAAAAAAAAAAAAfAB/AAAAAAAAAAAAAAAAAP4AfwAAAAAAAA' +
  'AAAAAAAAD+AH+AAAAAAAAAAAAAAAAB/gB/wAAAAAAAAAAAAAAAA/4Af+AAAAAAAAAAAAAAAAf+AH/wAAAAAAAAAAAAAAAP/g' +
  'B/+AAAAAAAAAAAAAAAH/4Af/wAAAAAAAAAAAAAAD/+AH//AAAAAAAAAAAAAAD//gB//4AAAAAAAAAAAAAB//4Af//gAAAAAA' +
  'AAAAAAB//+AH//8AAAAAAAAAAAAA///gB///wAAAAAAAAAAAA///4Af//+AAAAAAAAAAAAf//+AD///4AAAAAAAAAAAf///A' +
  'A////gAAAAAAAAAAf///wAP///8AAAAAAAAAAP///8AD////wAAAAAAAAAP////AA/////AAAAAAAAAP////wAP////4AAAA' +
  'AAAAH////4AB/////AAAA8AAAD////+AAf////8AAA/wAAD/////gAH/////gAAf+AAB/////4AA/////8AAP/wAA/////8A' +
  'AP/////gAD/8AAf/////AAD/////8AA//AAP/////wAAf/////gAf/wAH/////4AAH/////8AH/8AD/////+AAB//////AB/' +
  '/AA//////gAAP/////4Af/wAf/////wAAD/////+AH/8AP/////8AAAf/////wB//AD/////+AAAH/////+Af/wA//////gA' +
  'AB//////gH/+Af/////4AAAP/////4B//gH/////8AAAD//////Af/4D//////AAAAf/////wP/+A//////gAAAD/////8D/' +
  '/wP/////4AAAA//////h//+H/////8AAAAH/////4///h/////+AAAAB/////8f//8P/////gAAAAP////+H///h/////wAA' +
  'AAD/////j///8f////8AAAAAf////x////j////+AAAAAD////4////8f////AAAAAAf///+P////H////wAAAAAH////n//' +
  '//5////4AAAAAA////x////+P///8AAAAAAH///8/////z///+AAAAAAA////P////8////AAAAAAAH///j/////H///wAAA' +
  'AAAB///4/////x///4AAAAAAAP//+f////8f//8AAAAAAAB///n/////n//+AAAAAAAAP//5/////5///AAAAAAAAB//+f//' +
  '//+f//gAAAAAAAAP//n/////n//wAAAAAAAAA//5/////5//4AAAAAAAAAH/+f////+f/4AAAAAAAAAA//n/////P/8AAAAA' +
  'AAAAAH/9/////z/+AAAAAAAAAAAf/P////8/+AAAAAAAAAAAD/z/////P/AAAAAAAAAAAAf+/////3/gAAAAAAAAAAAB/v//' +
  '//5/gAAAAAAAAAAAAH5////+fgAAAAAAAAAAAAAff////vwAAAAAAAAAAAAABz////zwAAAAAAAAAAAAAAG////9gAAAAAAA' +
  'AAAAAAAAH///+AAAAAAAAAAAAAAAAB////gAAAAAAAAAAAAAAAAf///4AAAAAAAAAAAAAAAAH///+AAAAAAAAAAAAAAAAB//' +
  '//gAAAAAAAAAAAAAAAAf///4AAAAAAAAAAAAAAAAH///+AAAAAAAAAAAAAAAAD////wAAAAAAAAAAAAAAAA////8AAAAAAAA' +
  'AAAAAAAAf////gAAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAD///' +
  '///wAAAAAAAAAAAAAB//////+AAAAAAAAAAAAAA///////4AAAAAAAAAAAAA////////AAAAAAAAAAAAA////////8AAAAAA' +
  'AAAAAAf////////gAAAAAAAAAAAH////////8AAAAAAAAAAAB////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAD////' +
  '////wAAAAAAAAAAAA////////4AAAAAAAAAAAAH///////+AAAAAAAAAAAAAf//////+AAAAAAAAAAAAAD///////AAAAAAA' +
  'AAAAAAAP//////AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAP/////4AAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAAf//' +
  '//+AAAAAAAAAAAAAAAD/////AAAAAAAAAAAAAAAAf////gAAAAAAAAAAAAAAAB////wAAAAAAAAAAAAAAAAH///gAAAAAAAA' +
  'AAAAAAAAAD/8AAAAAAAAAAAAAAAAAAAP+AAAAAAAAAAAAAAAAAAAB+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── love — 비트맵 실루엣(128×128, 잉크 51.1%)
//   원본 마스크: love.png
var MASK_LOVE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAP/AAAAAB/8AAAAAAAAAAAP/+AAAAB//wAAAAA' +
  'AAAAAP//wAAAB///AAAAAAAAAAP///AAAA///4AAAAAAAAAD///4AAAf///AAAAAAAAAB////AAAP///4AAAAAAAAA////wA' +
  'AH////AAAAAAAAAf///+AAB////4AAAAAAAAH////wAA////+AAAAAAAAD////8AAP////wAAAAAAAA/////AAH////8AAAA' +
  'AAAAP////4AB/////AAAAAAAAH////+AAf////wAAAAAAAB/////gAH////+AAAAAAAAf////4AB/////gAAAAAAAH////+A' +
  'Af////4AAAAAAAB/////gAH////+AAAAAAAAf////4AB/////gAAAAAAAH////+AAf////wAAAAAAAA/////gAH////8AAAA' +
  'AAAAP////wAB/////AAAAAAAAD////8AAP////wAAAAAAAAf////AAD////4AAAAAAAAH////gAAf///+AAAAAAAAA////wA' +
  'AH////AAAAAAAAAH///8AAA////gAAAAAAAAA///+AAAH///wAAAAAAAAAP///AAAA///4AAAAAAAAAA///AAAAH//8AAAAA' +
  'AAADwD//gAAAAf/8A8AAAAAAH/AP/gAAAAB/8A/4AAAAAH/4AMAAAAAAAwAf/gAAAAH//gAAAAAAAAAAf/+AAAAD//+AAAAA' +
  'AAAAAf//wAAAB///4AAAAAAAAAf//+AAAA////wAAAAAAAA////wAAAf/////+AAAAf/////+AAAP//////+AAB///////wA' +
  'AH///////4AB///////+AAB////////gB////////gAA////////8A////////8AAP////////gf////////AAH////////8' +
  'P////////4AB/////////n////////+AA/////////5/////////wAP////////gB////////8AD////////gAD////////A' +
  'A////////gAAf///////wAf///////wAAB///////+AH///////4B/AP///////gB///////8D/8B///////4Af//////+B/' +
  '/wP//////+AH///////A//+D///////gB///////wf//wf//////4Af//////4P//+H//////+AH//////+H///g///////g' +
  'B///////B///8P//////4Af//////w////B//////+AH//////8P///4f//////gB///////D///+H//////4AP//////w//' +
  '//h//////8AD//////8f///4f//////AA///////D///+H//////wAP//////w////h//////8AD//////8P///4f//////A' +
  'Af//////D///8H//////gAH//////wf///D//////4AB//////+H///g//////+AAP//////g///4f//////AAD//////wH/' +
  '/8D//////wAA//////wA//+AP/////8AAH/////4AH//AB/////+AAB/////8Ag//BAH/////gAAf////+B+B/A+B/////4A' +
  'AD/////A/wAA/4P////8AAA/////gf/AAf/B/////AAAH////4P/8A//4f////gAAB////8H/////+D////4AAAP////D///' +
  '///w////8AAAD////w//////8H////AAAAf///4P//////h////gAAAD///+H//////4f///wAAAA////h//////+H///8AA' +
  'AAH///4f//////x///+AAAAB///+H//////8f///gAAAAP///h///////H///wAAAAB///4f//////x///4AAAAAP//+H///' +
  '///4f//8AAAAAD///h//////+H///AAAAAAf//4P//////h///gAAAAAD///D//////wf//wAAAAAAf//w//////8P//4AAA' +
  'AAAD//8H//////D//8AAAAAAAf//h//////g///AAAAAAAH//4P/////4f//gAAAAAAAf//D/////8H//gAAAAAAAD//wf//' +
  '///D//wAAAAAAAA//+H/////g//8AAAAAAAAD//g/////wf/+AAAAAAAAAf/8H////8P/+AAAAAAAAAD//B////+D//AAAAA' +
  'AAAAAf/4P////B//gAAAAAAAAAD//B////wf/wAAAAAAAAAAP/4f///4P/wAAAAAAAAAAB/+D///8H/4AAAAAAAAAAAH/wf/' +
  '/+D/4AAAAAAAAAAAA/+D///B/8AAAAAAAAAAAAD/wf//gf8AAAAAAAAAAAAAP+D//4P8AAAAAAAAAAAAAA/wf/8H+AAAAAAA' +
  'AAAAAAAD8D/8D8AAAAAAAAAAAAAAAPgf+B8AAAAAAAAAAAAAAAAeD/A4AAAAAAAAAAAAAAAAAAPgAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── hope — 비트맵 실루엣(128×128, 잉크 44.4%)
//   원본 마스크: hope.png
var MASK_HOPE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAQAAAAAAAAAAAAAAAIAAAAMAAAAAAAAAAAAAAADAAAAHAAAAAAAAAAAAAAAAQAAABgAAAAAAAAAAAAAAAGAA' +
  'AA4AAAAAAAAAAAAAAABwAAAOAAAAAAAAAAAAAAAAcAAAHgAAAAAAAAAAAAAAAHgAAD4AAAAAAAAAAAAAAAB4AAA+AAAAAAAA' +
  'AAAAAAAAfAAAfgAAAAAAAAAAAAAAAHwAAH4AAAAAAAAAAAAAAAB+AAB+AAAAAAAAAAAAAAAAfgAA/gAAAAAAAAAAAAAAAH8A' +
  'AP4AAAAAAAAAAAAAAAB/AAD+AAAAAAAP8AAAAAAAfwAB/gAAAAAAf/4AAAAAAH+AAf4AAAAAAf//AAAAAAB/gAH/AAAAAAP/' +
  '/8AAAAAA/4AB/wAAAAAH///gAAAAAP+AAf8AAAAAD///8AAAAAD/gAP/gAAAAB////gAAAAB/4AD/4AAAAA////8AAAAAf+A' +
  'A//AAAAAP////AAAAAP/wAP/wAAAAH////4AAAAD/4AD/+AAAAB////+AAAAB/+AAf/gAAAA/////gAAAAf/gAH/8AAAAP//' +
  '//8AAAAP/4AB//gAAAD/////AAAAH/+AAf/4AAAA/////wAAAB//gAH//AAAAf////8AAAA//4AA//4AAAH/////AAAAf/8A' +
  'AP//AAAB/////wAAAP//AAD//4AAAP////8AAAH//wAAf//AAAD/////AAAD//4AAH//4AAA/////wAAB//+AAB///gAAP//' +
  '//8AAB///AAAP//8AAB////+AAA///wAAB///gAAf////gAAf//4AAAf//+AAH////wAAf//+AAAD///4AA////8AAf///AA' +
  'AA////AAH///+AAP///wAAAH///8AA////AAP///4AAAA////wAH///gAP///8AAAAH///+AA///wAP///+AAAAA////4AH/' +
  '/4AH////AAAAAP////gAf/4AH////wAAAAB////+AB/4AH////4AAAAAP////4AAAAH////8AAAAAB/////AAAAD////+AAA' +
  'AQAP////8AAAD/////AAgAOAB/////gAAB/////gAcADwAH////+AAB/////gAPAA8AA/////wAA/////wAHwAPgAH////+A' +
  'Af////4AB8AD8AA/////wAP////8AA/AA/gAH////+AH////+AAfwAP8AA/////wD/////AAP8AD/wAD////+B/////gAP/A' +
  'A/+AAf////w/////wAH/wAP/wAD////+f////wAD/8AD/+AAf/////////4AB//AA//4AD/////////8AB//wAP//AAf////' +
  '////+AA//8AD//4AD/////////AAf//AA///AAf////////wAf//wAP//8AD////////4AP//8AD///gA////////8AH///A' +
  'A///8AH///////+AD///gAH///gA////////AD///4AB///+AH///////wB///+AAf///wB///////4A////gAH///+AP///' +
  '///8Af///wAA////wB///////AP///8AAP///+Af//////gH////AAD////wD//////4D////gAAf///+A//////8B////4A' +
  'AH////wH//////A////+AAA////8B//////gf////AAAP////gP/////4H////wAAB////8D/////8D////4AAAf////A///' +
  '///B////+AAAD////4H/////wf////AAAA/////B/////4P////wAAAH////wf////+D////4AAAB////+D/////h////8AA' +
  'AAP////g/////wf////AAAAB////8P////8P////gAAAAf////D/////D////4AAAAD////wf////h////8AAAAAf///+H//' +
  '//4f///+AAAAAD////h////+H////AAAAAAf///4f////D////gAAAAAD////H////w////wAAAAAA////x////8P///4AAA' +
  'AAAD///8f///+H///8AAAAAAAf///H////h///+AAAAAAAD///x////4f///AAAAAAAAf//+f///8P///gAAAAAAAD///n//' +
  '//D///wAAAAAAAAf//5////x///4AAAAAAAAB//+f///4f//4AAAAAAAAAP//n///+P//8AAAAAAAAAB//5////D//+AAAAA' +
  'AAAAAH/+f///x//+AAAAAAAAAAA//n///4f/+AAAAAAAAAAAD/5///+P//AAAAAAAAAAAAf8f///D//AAAAAAAAAAAAD/H//' +
  '/h//AAAAAAAAAAAAAfz///4f/AAAAAAAAAAAAAD8///8P/AAAAAAAAAAAAAAfP//+H/AAAAAAAAAAAAAADz///B/AAAAAAAA' +
  'AAAAAAA5///g+AAAAAAAAAAAAAAAGf//wcAAAAAAAAAAAAAAABn//wAAAAAAAAAAAAAAAAAD//4AAAAAAAAAAAAAAAAAA//w' +
  'AAAAAAAAAAAAAAAAAAP/gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── connection — 비트맵 실루엣(128×128, 잉크 30.9%)
//   원본 마스크: connection.png
var MASK_CONNECTION = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAH///8A' +
  'AAf//4AAAAAAAAAf////8AB////4AAAAAAAA//////wD/////wAAAAAAA///////3//////AAAAAAA//////////////8AAA' +
  'AAAf//////////////wAAAAAf//////////////+AAAAAP///////////////wAAAAH///////////////+AAAAD////////' +
  '////////wAAAB////////////////+AAAA/////////////////wAAAf////////////////+AAAH/////////////////gA' +
  'AD///4AP///////////8AAB///gAAf/////AAB///gAA///gAAD/////gAAH//8AB///gAAA/////4AAAf//4Af//wAAAH//' +
  '//8AAAD//+AH//4AAAB/////AAAAf//gB//8AAAAf////wAAAD//4Af//AAAAH////8AAAA//+AH//gAAAD/////AAAAH//g' +
  'B//4AAAA/////4AAAB//4Af/8AAAAf////+AAAAP/+AH//AAAAH/////gAAAD//gB//wAAAB/////8AAAAf/4Af/4AAAAf//' +
  '///AAAAH/+AH/+AAAAP/////wAAAB//gB//gAAAD/////8AAAAf/4Af/4AAAA//////AAAAH/+AH/+AAAAP/////wAAAB//g' +
  'B//gAAAD/////8AAAAf/4Af/4AAAA//////AAAAH/+AH/+AAAAP/////wAAAB//gB//gAAAD/////8AAAAf/4Af/4AAAA///' +
  '///AAAAH/+AH/+AAAAH/////wAAAB//gB//gAAAB/////4AAAAf/4Af/8AAAAf////+AAAAP/+AH//AAAAH/////gAAAD//g' +
  'B//4AAAB/////4AAAB//4Af/+AAAAP////8AAAAf/+AH//wAAAD/////AAAAP//gB//8AAAA/////gAAAD//4Af//gAAAP//' +
  '//4AAAB//+AH//8AAAD////+AAAA///gB///gAAA/////gAAAf//4AD//8AAAf////8AAAP//4AAf//wAAH/////gAAP//4A' +
  'AD///AAH/////8AAP//8AAAf/////////////////AAAH/////////////////gAAA/////////////////wAAAH////////' +
  '////////8AAAA////////////////+AAAAH////////////////AAAAA////////////////gAAAAH///////////////wAA' +
  'AAA///////////////wAAAAAH//////////////4AAAAAAf/////////////8AAAAAAB//////3//////8AAAAAAAH/////g' +
  'H/////8AAAAAAAAP////AAP////4AAAAAAAAAD//wAAAP///gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── family — 비트맵 실루엣(128×128, 잉크 40.3%)
//   원본 마스크: family.png
var MASK_FAMILY = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAD8AAAAA/AAAAAAAAAAAAAD/wAAAA/8AAAAAAAAAAAAB/+AAAAf/gAAAAAAAAAAAA//wAAAP/8AAAAAA' +
  'AAAAAAf/+AAAH//gAAAAAAAAAAAH//wAAB//8AAAAAAAAAAAD//8AAA///AAAAAAAAAAAA///AAAP//wAAAAAAAAAAAP//4A' +
  'AD//8AAAAAAAAAAAD//+AAA///gAAAAAAAAAAA///gAAP//wAAAAAAAAAAAP//wAAD//8AAAAAAAAAAAD//8AAA///AAAAAA' +
  'AAAAAAf//AAAH//wAAAAAAAAGAAH//gAAB//4AAYAAAAAH8AA//wAAAP/8AA/wAAAAH/gAP/8AAAD//AAf+AAAAB/8AD//AA' +
  'AA//wAP/wAAAA//gA//4AAAf/8AD/8AAAAP/4Af//gAAf//gB//AAAAH/+Af//+AAf//+Af/4AAAB//gP///4Af///wH/+AA' +
  'AAP/4H////gf////B//gAAAD/+D//////////4f/wAAAA//7/////////////8AAAAH////////////////AAAAB////////' +
  '////////gAAAAf///////////////8AAAAP////////////////gAAAP////////////////8AAAH///////H/h///////gA' +
  'AD///////gGAP//////8AAB///////wAAD///////gAA///////8AAA///z///8AAP///h///gAAP//4f///gAH///wP//4A' +
  'AD//8D///4AB///4D//+AAB///Af///AA///8A///gAAf//wD///wAP///AP//4AAH//8A///8AD///wD//+AAB///gP///A' +
  'B///8A///wAAf//4D///4Af///AP//8AAH//+A///+AH///wH///AAD///gP///gB///8B///wAA///4D///4Af///Af//8A' +
  'AP//+A///+AH///wH///AAD///gP///gB///8B///wAA///4D///4Af///Af//+AAP//+A///+AH///wH///gAH///gP///g' +
  'B///8B///4AB///4D///4Af///Af//+AAf//+A///+AH///wH///gAH///wP///gA///8D///4AB///8D///wAP///A///+A' +
  'Af///A///8AD///wP///gAH///wH///AA///8D///4AB///8B///wAH///A////AAf///Af//8AB///gP///wAH///wH//+A' +
  'Af//4D///8AB///8B///gAD//+A////AA////Af//wAA///gf/j/wAP/D/4H//8AAH//4H/wf8AD/w/+A///AAB//8B/8H/A' +
  'A/4P/gP//gAAP//Af/B/wAP+B/4D//4AAD//wH/wf8AD/gf+A//8AAAf/4B/4D/AA/4H/wH//AAAH/4Af+A/wAP8B/8Af/gA' +
  'AA/+AP/gP8AD/Af/AD/wAAAH/gD/4D/AA/wD/wA/8AAAB/4A/+A/4AP8A/8Af+AAAAP/AP/AP+AD/AP/AP/AAAAB/4H/wD/g' +
  'A/wD/4H/gAAAAP/B/8Af4AP8A/+D/4AAAAD////AH+AH/AH///8AAAAAf///gB/gB/gB///+AAAAAD///4Af4Af4Af///AAA' +
  'AAAf//+AH+AH+AH///gAAAAAD///gB/gB/gB///wAAAAAAf//4Af4Af4Af//4AAAAAAD//+AH+AH+AH//8AAAAAAAf//wB/w' +
  'B/gD//+AAAAAAAD///g/8Af8D///AAAAAAAAf///f/AP/3///gAAAAAAAD/////wD/////wAAAAAAAAP/////D/////4AAAA' +
  'AAAAB///////////4AAAAAAAAAP//////////8AAAAAAAAAB//////////+AAAAAAAAAAH/////////+AAAAAAAAAAA/////' +
  '/////AAAAAAAAAAAD/////////AAAAAAAAAAAAf////////gAAAAAAAAAAAB////////gAAAAAAAAAAAAP///////wAAAAAA' +
  'AAAAAAAf//////wAAAAAAAAAAAAAB//////wAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAAf////gAAAAAAAAAAAAAAAAf/' +
  '/+AAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── smile — 비트맵 실루엣(128×128, 잉크 46.5%)
//   원본 마스크: smile.png
var MASK_SMILE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA/' +
  '/gAAAAAAAAAAAAAAAAA////+AAAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAf//////+AAAAAAAAAAAAAf///////4AAAAAA' +
  'AAAAAA/////////wAAAAAAAAAAA//////////gAAAAAAAAAA//////////8AAAAAAAAAA///////////wAAAAAAAAAf/////' +
  '/////+AAAAAAAAAP///////////wAAAAAAAAP////////////AAAAAAAAH////////////4AAAAAAAD/////////////AAAA' +
  'AAAA/////////////wAAAAAAAf////////////+AAAAAAAP/////////////wAAAAAAH/////////////+AAAAAAB///////' +
  '///////gAAAAAA//////////////8AAAAAAP//////////////gAAAAAH//////////////4AAAAAB///////////////AAA' +
  'AAA///////////////wAAAAAP//////////////8AAAAAH///////////////gAAAAB///////////////4AAAAAf///////' +
  '///////+AAAAAP///////////////wAAAAD///wH/////+A///8AAAAA///wAf////+AD///AAAAAP//wAB/////AAP//4AA' +
  'AAD//4AAP////AAB//+AAAAB//8AAB////wAAP//gAAAAf//AAAf///4AAB//4AAAAH//gAAD///8AAAf/+AAAAD//4AAAf/' +
  '//AAAD//wAAAB//8AAAH///gAAA//+AAAB///AAAB///4AAAP//4AAA///wD8Af//+APwB///AAAf//8D/4H///gf/Af//4A' +
  'AP///B//h///4P/8P///AAD//////////////////4AB//////////////////+AA///////////////////wAP/////////' +
  '/////////8AD///////////////////AB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gA///////////////////wAP//////////////////8AD///////////////////A' +
  'Af//////////////////gAD////z////////z////4AA////4P///////wf///8AAH///8A///////4D///+AAA////AD///' +
  '///4A////AAAD///wAP/////wAP///AAAAP//8AAf////wAD///gAAAB///gAAf///AAA///wAAAAf//4AAAH/wAAAf//4AA' +
  'AAH///AAAAAAAAAH//+AAAAB///wAAAAAAAAD///gAAAAP//+AAAAAAAAA///4AAAAD///gAAAAAAAAf//8AAAAA///8AAAA' +
  'AAAAP///AAAAAH///gAAAAAAAD///wAAAAB///8AAAAAAAB///4AAAAAf///AAAAAAAA///+AAAAAD///8AAAAAAAf///AAA' +
  'AAA////AAAAAAAP///wAAAAAH///4AAAAAAH///4AAAAAB////gAAAAAD///+AAAAAAP///8AAAAAD////AAAAAAD////wAA' +
  'AAD////wAAAAAAf////AAAAB////4AAAAAAD////8AAAB////8AAAAAAA/////4AAD/////AAAAAAAH/////8Af/////gAAA' +
  'AAAA/////////////wAAAAAAAH////////////4AAAAAAAA////////////8AAAAAAAAH///////////+AAAAAAAAA//////' +
  '//////AAAAAAAAAH///////////gAAAAAAAAAf//////////gAAAAAAAAAD//////////wAAAAAAAAAAP/////////wAAAAA' +
  'AAAAAA/////////4AAAAAAAAAAAD////////4AAAAAAAAAAAAP///////4AAAAAAAAAAAAA///////wAAAAAAAAAAAAAA///' +
  '///gAAAAAAAAAAAAAAB////+AAAAAAAAAAAAAAAAA///wAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── sadness — 비트맵 실루엣(128×128, 잉크 61.3%)
//   원본 마스크: sadness.png
var MASK_SADNESS = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAABgAAAAAAAAAAAAAAAAAB/////AAAAAAAAAAAAAAAP//////gAAAAAAAAAAAAD////' +
  '////wAAAAAAAAAAAP/////////wAAAAAAAAAAf//////////wAAAAAAAAA////////////AAAAAAAAA////////////8AAAA' +
  'AAAA/////////////wAAAAAAA//////////////AAAAAAA//////////////8AAAAAAf//////////////gAAAAAP///////' +
  '///////+AAAAAP///////////////wAAAAH///////////////+AAAAD////////////////wAAAB////////////////+AA' +
  'AAf////////////////wAAAP////////////////8AAAH/////////////////gAAD/////////////////8AAA/////////' +
  '/////////AAAf/////////////////4AAH/////////////////+AAD//////////////////wAA//////////////////8A' +
  'AP//////////////////AAH//////////////////4AB//////////////////+AAf//////////////////gAP/////////' +
  '/////////8AD///////////////////AA///////////////////wAP//////////////////8AD///////////////////A' +
  'A///////////////////wAf//////////////////+AH///////////////////gB///////h///4f//////4Af//////wf/' +
  '/+D//////+AH//////4H///gf//////gB//////8B///4D//////4Af/////8Af//+AP/////+AH/////+AP///wA//////g' +
  'B///w/8AD///8AD/4f//4Af//4AAAB////gAAAH//+AH//+AAAAf///8AAAB///gB///gAAAP////AAAAf//4Af//8AAAH//' +
  '//4AAAP//+AH///gAAD/////AAAH///gB///8AAB/////8AAD///4Af///wAB//////gAB///+AH////AD///////AD////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////AB///////////////////wAP//////////////////8AD///////////////////A' +
  'A///////////////////wAP//////////////////8AD///////////////////AA/////////gf////////wAP///////+A' +
  'Af///////8AD///////8AAA///////+AAf//////8AAAB///////gAH//////8AAAAP//////4AB//////8AAAAA//////+A' +
  'Af/////+AAAAAH//////gAH//////AAAAAA//////4AA//////gAAAAAH/////8AAP/////wAAAAAA//////AAD/////4AAD' +
  'wAAH/////wAA/////8AAf/4AA/////8AAH/////AA///wAH////+AAB/////gA////AB/////gAAf////wA////8AP////4A' +
  'AH////8Af////gD////+AAA/////Af////+Af////AAAP////gP/////wH////wAAD////4H/////+B////4AAAf///+D///' +
  '///wf///+AAAH////g//////+H////gAAA////8f//////z////wAAAP////////////////8AAAB////////////////+AA' +
  'AAf////////////////gAAAD////////////////wAAAA////////////////8AAAAH///////////////+AAAAA////////' +
  '////////AAAAAP///////////////wAAAAB///////////////4AAAAAP//////////////8AAAAAB//////////////+AAA' +
  'AAAP//////////////AAAAAAB//////////////gAAAAAAH/////////////wAAAAAAA/////////////wAAAAAAAD//////' +
  '//////4AAAAAAAAP///////////wAAAAAAAAA///////////wAAAAAAAAAB//////////AAAAAAAAAAAAf///////4AAAAAA' +
  'AAAAAAAAD///8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── friendship — 비트맵 실루엣(128×128, 잉크 53.6%)
//   원본 마스크: friendship.png
var MASK_FRIENDSHIP = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAD/gAA' +
  'AAD/gAAAAAAAAAAAH//AAAAH//AAAAAAAAAAAH//8AAAH//8AAAAAAAAAAD///gAAD///gAAAAAAAAAB///+AAD///8AAAAA' +
  'AAAAA////wAB////gAAAAAAAAAf///+AAf///8AAAAAAAAAP////gAP////gAAAAAAAAH////8AH////8AAAAAAAAB/////g' +
  'D/////AAAAAAAAA/////4A/////4AAAAAAAAP////+AP////+AAAAAAAAD/////wH/////gAAAAAAAB/////8B/////8AAAA' +
  'AAAAf/////Af/////AAAAAAAAH/////4P/////wAAAAAAAB/////+D/////8AAAAAAAAf/////g//////gAAAAAAAH/////4' +
  'P/////4AAAAAAAB/////+D/////8AAAAAAAAf/////g//////AAAAAAAAH/////wH/////wAAAAAAAB/////8B/////8AAAA' +
  'AAAAP/////Af////+AAAAAAAAD/////gD/////gAAAAAAAA/////4A/////4AAAAAAAAH////8AH////8AAAAAAAAA/////A' +
  'B/////AAAAAAAAAP////gAP////gAAAAAAAAB////wAB////wAAAAAAAAAP///4AAP///4AAAAAAAAAB///8AAB///8AAAAA' +
  'AAAAAH//+AAAP//+AAAAAAAAAAA//+AAAA//+A8AAAAAAAH+D/+AAAAD/+D/8AAAAAAP/8D8AAAAAH8D//wAAAAAH//wAAAA' +
  'B/gAH//+AAAAAH///wAAD////////4AAAAD///////////////+AAAAB////////////////wAAAAf///////////////+AA' +
  'AAP////////////////gAAAD////////////////4AAAB/////////////////AAAAf////////////////wAAAH////////' +
  '////////8AAAB/////////////////AAAAf////////////////wAAAP////////////////8AAAH/////////////////gA' +
  'AB/////////////////8AAA//////////////////AAAP/////////////////4AAH/////////////////+AAD/////////' +
  '/////////wAA//////////////////8AAP//////////////////AAH//////////////////4AB//////////////////+A' +
  'Af//////////////////gAP//////////////////8AD///////////////////AA//4/////////////5//wAP/8H//////' +
  '//////8P/8AH/+B////////////+B//gB//gP////////////Af/4Af/4D////////////wH/+AH/+Af///////////4A//g' +
  'B//AH///////////+AP/4Af/wB////////////gD/+AH/8Af///////////4A//gB//AD///////////8AP/4Af/wA//////' +
  '//////AD/+AH/8AP///////////wA//gB//gD///////////8AP/4Af/4A////////////AD/+AH/+AP///////////wB//g' +
  'B//gD///////////8Af/4Af/8A////////////AP/+AH//AP///////////wD//gA//4D///////////8B//wAP/+A//////' +
  '//////Af/8AD//wP///////////wP//AAf/+D///////////8H//wAH//w////////////D//4AA///f///////////7//+A' +
  'AP//////////////////AAB//////////////////wAAf/////////////////4AAD/////////////////8AAAf////////' +
  '////////+AAAD/////////////////AAAAf////////////////wAAAD////////////////4AAAAf///////////////4AA' +
  'AAD///////////////8AAAAAf//////////////+AAAAAB///////////////AAAAAAP//////////////AAAAAAA///////' +
  '///////AAAAAAAB/////////////AAAAAAAAH///////////+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── embrace — 비트맵 실루엣(128×128, 잉크 47.4%)
//   원본 마스크: embrace.png
var MASK_EMBRACE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAA/wAAAAAAAAAAAAAAAAAAB//gAAAAAAAAAAAAAAAAAB//+AAAAAAAAAAAAAAAAAB///4AAAAAAAAAAA' +
  'AAAAAA////AAAAAAAAAAAAAAAAA////4AADwAAAAAAAAAAAAf////AAH/wAAAAAAAAAAAH////4AP//gAAAAAAAAAAD/////' +
  'AP//+AAAAAAAAAAB/////4H///wAAAAAAAAAAf/////////+AAAAAAAAAAP//////////wAAAAAAAAAD//////////+AAAAA' +
  'AAAAB///////////wAAAAAAAAAf//////////8AAAAAAAAAH///////////gAAAAAAAAB///////////4AAAAAAAAAf/////' +
  '//////AAAAAAAAAH///////////wAAAAAAAAB///////////8AAAAAAAAAf///////////gAAAAAAAAH///////////4AAAA' +
  'AAAAB///////////+AAAAAAAAAf///////////gAAAAAAAAD///////////4AAAAAAAAA///////////+AAAAAAAAAP/////' +
  '//////gAAAAAAAAD///////////wAAAAAAAAA///////////8AAAAAAAAAP///////////AAAAAAAAAD///////////gAAAA' +
  'AAAAB///////////4AAAAAAAAA///////////+AAAAAAAAAf///////////gAAAAAAAAf///////////4AAAAAAAAP//////' +
  '/////+AAAAAAAAH////////////wAAAAAAAD////////////8AAAAAAAB/////////////gAAAAAAA/////////////+AAAA' +
  'AAAP/////////////4AAAAAAH//////////////AAAAAAB//////////////4AAAAAAf//////////////AAAAAAP///////' +
  '///////4AAAAAD//////////////+AAAAAA///////////////wAAAAAf//////////////8AAAAAH///////////////gAA' +
  'AAB///////////////4AAAAAf///////////////AAAAAH/8P////////////wAAAAB/8A////////////8AAAAAf+AD////' +
  '///4f///AAAAAH/gAf//////wB///wAAAAB/wAA//////gAP//+AAAAAf8AAD/////gAB///gAAAAH/AAAD///8AAAf//4AA' +
  'AAB/wAAAA//gAAAH//+AAAAAf8AAAAAAAAAAB///gAAAAH/gAAAAAAAAAAf//wAAAAB/4AAAAAAAAAAH//8AAAAAf/AAAAAA' +
  'AAAAB///AAAAAH/4AAAAAAAAAA///wAAAAD//AAAAAAAAAAf//8AAAAA//8AAAAAAAAAf///AAAAAP//gAAAAAAAAP///wAA' +
  'AAH//+AAAAAAAAP///+AAAAB///4AAAAAAAP////gAAAAf///gAAAAAAP////4AAAAH///+AAAAAAf/////AAAAD////8AAA' +
  'AAf/////wAAAA/////4AAAB//////8AAAAP/////+AAH///4f//AAAAD////////////4B//wAAAA////////////8AP/8AA' +
  'AAP///////////8AB//AAAAD///////////+AAf/wAAAA////////////AAH/8AAAAP////B//////AAA//AAAAD////AD//' +
  '///AAAf/wAAAA////gAP////AAAH/8AAAAP///4AAP//+AAAB//AAAAD///8AAAH/wAAAA//wAAAA////AAAAAAAAAAP/8AA' +
  'AAP///wAAAAAAAAAH//AAAAD///8AAAAAAAAAB//wAAAA////gAAAAAAAAA//8AAAAP///4AAAAAAAAAf//AAAAB////AAAA' +
  'AAAAAP//wAAAAf///4AAAAAAAAP//4AAAAH////gAAAAAAAH//+AAAAB////8AAAAAAAH///gAAAAP////4AAAAAAH///4AA' +
  'AAD/////gAAAAAH///8AAAAA//////AAAAAH////AAAAAH//////AAAAP////wAAAAB///////wAD/////4AAAAAf///////' +
  '///////+AAAAAD///////////////gAAAAA///////////////wAAAAAH//////////////8AAAAAB//////////////+AAA' +
  'AAAP//////////////gAAAAAD//////////////wAAAAAAf/////////////4AAAAAAD/////////////+AAAAAAA///////' +
  '///////AAAAAAAH/////////////gAAAAAAA/////////////wAAAAAAAH////////////4AAAAAAAA////////////8AAAA' +
  'AAAAH///////////+AAAAAAAAAf///////////AAAAAAAAAB///////////AAAAAAAAAAH//////////AAAAAAAAAAAf////' +
  '////+AAAAAAAAAAAAD///////AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── boomer — 비트맵 실루엣(128×128, 잉크 19.9%)
//   원본 마스크: boomer.png
var MASK_BOOMER = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA//AAAAAAAAAAAAAAAAAAA//8AAAAAAAAA' +
  'AAAAAAAAA///wAAAAAAAAAAAAAAAAA////AAAAAAAAAAAAAAAAA////4AAAAAAAAAAAAAAAAf////gAAAAAAAAAAAAAAAP//' +
  '//8AAAAAAAAAAAAAAAP/////wAAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAD//////wAAAAAAAAAAAAAB//////+AAAAAAA' +
  'AAAAAAB///////4AAAAAAAAAAAAA////////AAAAAAAAAAAAAf///////4AAAAAAAAAAAAP////////AAAAAAAAAAAAH////' +
  '////4AAAAAAAAAAAD/////////AAAAAAAAAAAD/////////8AAAAAAAAAAB//////////gAAAAAAAAAA//////////8AAAAA' +
  'AAAAAf//////////gAAAAAAAAAP//////////8AAAAAAAAAH///////////gAAAAAAAAD///////////8AAAAAAAAD//////' +
  '//////wAAAAAAAB////////////+AAAAAAAA/////////////wAAAAAAAf////////////+AAAAAAAP/////////////wAAA' +
  'AAAH/////////////+AAAAAAD//////wD//////wAAAAAB//////gAH/////+AAAAAA//////gAAf/////wAAAAA//////gA' +
  'AB//////AAAAAf/////gAAAH/////4AAAAP/////gAAAAf/////AAAAH/////wAAAAD/////4AAAD/////4AAAAAf/////AA' +
  'AB/////4AAAAAB/////wAAAf////8AAAAAAP////+AAAP////8AAAAAAA/////wAAH////+AAAAAAAH////+AAD/////AAAA' +
  'AAAA/////wAA/////gAAAAAAAH////8AAf////gAAAAAAAAf////gAP////wAAAAAAAAD////4AD////4AAAAAAAAAf////A' +
  'A////8AAAAAAAAAD////wAf///8AAAAAAAAAAP///8AH///+AAAAAAAAAAB////gB////AAAAAAAAAAAP///4Af///gAAAAA' +
  'AAAAAB///+AH///wAAAAAAAAAAAP///gB///wAAAAAAAAAAAA///4Af//4AAAAAAAAAAAAH//+AH//8AAAAAAAAAAAAA///A' +
  'A//+AAAAAAAAAAAAAH//wAH/+AAAAAAAAAAAAAAf/4AA//AAAAAAAAAAAAAAD/8AAH/AAAAAAAAAAAAAAAP+AAAMAAAAAAAA' +
  'AAAAAAAAMAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── grandpiano — 비트맵 실루엣(128×128, 잉크 41.3%)
//   원본 마스크: grandpiano.png
var MASK_GRANDPIANO = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAP/4AAAAAAAAA' +
  'AAAAAAAAAH//8AAAAAAAAAAAAAAAAAD///4AAAAAAAAAAAAAAAAA////AAAAAAAAAAAAAAAAAf///8AAAAAAAAAAAAAAAAH/' +
  '///gAAAAAAAAAAAAAAAB////4AAAAAAAAAAAAAAAA/////AAAAAAAAAAAAAAAAP////wAAAAAAAAAAAAAAAH////+AAAAAAA' +
  'AAAAAAAAB/////gAAAAAAAAAAAAAAA/////4AAAAAAAAAAAAAAAP/////AAAAAAAAAAAAAAAH/////wAAAAAAAAAAAAAAB//' +
  '///8AAAAAAAAAAAAAAA//////AAAAAAAAAAAAAAAP/////wAAAAAAAAAAAAAAH/////8AAAAAAAAAAAAAAB//////AAAAAAA' +
  'AAAAAAAA//////wAAAAAAAAAAAAAAP/////+AAAAAAAAAAAAAAH//////gAAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAf//' +
  '////wAAAAAAAAAAAAAP///////AAAAAAAAAAAAAD///////8AAAAAAAAAAAAB////////4AAAAAAAAAAAAf////////8AAAA' +
  'AAAAAAAP/////////8AAAAAAAAAAD//////////4AAAAAAAAAB///////////4AAAAAAAAAf///////////gAAAAAAAAP///' +
  '////////+AAAAAAAAD////////////wAAAAAAAB////////////+AAAAAAAAf////////////wAAAAAAAP////////////8A' +
  'AAAAAAD/////////////gAAAAAAB/////////////4AAAAAAAf/////////////AAAAAAAP/////////////wAAAAAAD////' +
  '/////////8AAAAAAA//////////////AAAAAAAf/////////////wAAAAAAH/////////////8AAAAAAD//////////////A' +
  'AAAAAA//////////////wAAAAAAf/////////////8AAAAAAH/////////////+AAAAAAD//////////////gAAAAAB/////' +
  '/////////wAAAAAP//////////////8AAAAAf//////////////+AAAAAH///////////////AAAAAB///////////////gA' +
  'AAAAf//////////////4AAAAAP//////////////4AAAAAD//////////////8AAAAAA///////////////AAAAAAf//////' +
  '////////wAAAH//////////////////AAD//////////////////4AA//////////////////+AAP//////////////////g' +
  'AB//////////////////4AAP/////////////////+AAD//////////////////gAA//////////////////4AAP////////' +
  '/////////+AAH//////////////////gAf//////////////////4AP//////////////////+AH///////////////////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////AB//////////+AAAAAD/8AAf//////////gAAAAAf+AAD//////////wAAAAAH/AA' +
  'AAP/4AAB///wAAAAAA/wAAAB/8AAAP//4AAAAAAP8AAAAH/AAAAfj8AAAAAAD/AAAAB/gAAAD4eAAAAAAA/wAAAAf4AAAA+H' +
  'gAAAAAAP8AAAAH+AAAAPh4AAAAAAD/AAAAB/gAAAD4eAAAAAAA/wAAAAf4AAAAeHgAAAAAAP8AAAAH+AAAAHh4AAAAAAD/AA' +
  'AAB/gAAAB4eAAAAAAA/wAAAAf4AAAAeHgAAAAAAP8AAAAH+AAAAHhwAAAAAAD/AAAAA/gAAAB4cAAAAAAA/wAAAAP4AAAAeH' +
  'AAAAAAAP4AAAAD+AAAAHhwAAAAAAD+AAAAA/gAAAB4cAAAAAAAfgAAAAP4AAAAeHAAAAAAAH4AAAAD8AAAAHhwAAAAAAB+AA' +
  'AAA/AAAAB4cAAAAAAAfgAAAAPwAAAAeHAAAAAAAH4AAAAD8AAAAHhwAAAAAAB+AAAAA/AAAAB4eAAAAAAAfgAAAAP4AAAA/f' +
  'gAAAAAAP8AAAAH+AAAAf/+AAAAAAD/AAAAB/gAAAP//gAAAAAA/wAAAAPwAAAD//4AAAAAAH4AAAAB8AAAA//+AAAAAAB+AA' +
  'AAAeAAAAP//gAAAAAAPAAAAAHgAAAD//4AAAAAADwAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── heartorgan — 비트맵 실루엣(128×128, 잉크 40.0%)
//   원본 마스크: heartorgan.png
var MASK_HEARTORGAN = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAA/8AAAAAAAAAAAAAAAAAAB//wAAAAAAAAAAAAAAAAAB//8DgAAAAAAAAAAAAAAAAf//B/AAAAAAA' +
  'AAAAAAAAAH//wf8AAAAAAAAAAAAAB+A//8P/gAAAAAAAAAAAAB/wP//D/4AAAAAAAAAAAAB/8B//h/+AAAAAAAAAAAAB//Af' +
  '/4f/AAAAAAAAAAAAA//4H/+H/wAAAAAAAAAAAAf/+B//z/4AAAAAAAAAAAAH//wf///+AAAAAAAAAAAAB//8D////AAAAAAA' +
  'AAAAAAf//A////wAAAAAAAAAAAAD//4P///8AAAAAAAAAAAAAf/+H////AAAAAAAAAAAAAD//x////wAAAAAAAAAAAAA//4f' +
  '///8AAAAAAAAAAAAAP/+P////gAAAAAAAAAAAAB//H////8AAAAAAAAAAAAAf/j/////gAAAAAAAAAAAAD/x/////8AAAAAA' +
  'AAAAA/w/8f/////AAAAAAAAAAA//P+P/////4AAAAAAAAAAf/z/n/////+AAAAAAAAAAP/8fx//////gAAAAAAAAAH//H8//' +
  '////4AAAAAAAAAB//x+P/////+AAAAAAAAAAP/8fj/////8B8AAAAAAAAD//H5/////8H/wAAAAAAAA//x+f//8B8P/+AAAA' +
  'AAAAH/8fH//8AEH//gAAAAAAAB//Hx//+AAH//8AAAAAAAAP/58///AAD///AAAAAAAAD/+fP//wAD///wAAAAAAAA//jz//' +
  '8AB///8AAAAAAAAP/8Y//+AA////AAAAAAAAD//gP//gAf///gAAAAAAAA//+D//4AH///4AAAAAAAAP//w//+AD///4AAAA' +
  'AAAAD//+P//gB///wAAAAAAAAA///x//4Af//gAAAAAAAAAP//8f/+AP//wAAAAAAAAAD//+H//wD//4wAAAAAAAAA///j//' +
  '/h//8eAAAAAAAAAP//w//////+PwAAAAAAAAH//wf//////H+AAAAAAAAB//4f//////z/gAAAAAAAA//8P//////4/8AAAA' +
  'AAAAP/+H//////+P/AAAAAAAAD//H///////j/4AAAAAAAB//x///////4f+AAAAAAAAf/4////////D/gAAAAAAAH/8f///' +
  '////8P4AAAAAAAB//P////////h+AAAAAAAAf/j////////+PgAAAAAAAP/5/////////x4AAAAAAAD/8f////////+OAAAA' +
  'AAAA//P/////////xgAAAAAAAP/z/////////+IAAAAAAAD/4//////////wAAAAAAAA//f/////////+AAAAAAAAP//////' +
  '//////gAAAAAAAB////////////8AAAAAAAAf////////////AAAAAAAAH////////////4AAAAAAAB////////////+AAAA' +
  'AAAAP////////////gAAAAAAAD////////////4AAAAAAAA/////////////AAAAAAAAH////////////wAAAAAAAB//////' +
  '//////8AAAAAAAAP////////////AAAAAAAAB////////////wAAAAAAAAf///////////8AAAAAAAAD////////////AAAA' +
  'AAAAA////////////wAAAAAAAAH///////////8AAAAAAAAB////////////AAAAAAAAAf///////////gAAAAAAAAH/////' +
  '//////4AAAAAAAAA///////////+AAAAAAAAAP///////////gAAAAAAAAD///////////4AAAAAAAAAf//////////8AAAA' +
  'AAAAAH///////////AAAAAAAAAA///////////wAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAAAAAAAP////' +
  '//////gAAAAAAAAAD//////////wAAAAAAAAAAf/////////8AAAAAAAAAAD/////////+AAAAAAAAAAA//////////gAAAA' +
  'AAAAAAH/////////wAAAAAAAAAAA/////////4AAAAAAAAAAAH////////+AAAAAAAAAAAA/////////AAAAAAAAAAAAH///' +
  '/////wAAAAAAAAAAAA////////4AAAAAAAAAAAAH///////8AAAAAAAAAAAAA////////AAAAAAAAAAAAAH///////gAAAAA' +
  'AAAAAAAA///////wAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAA//////+AAAAAAAAAAAAAAD//////AAAAAAAAAAAAAAA//' +
  '////gAAAAAAAAAAAAAAD/////wAAAAAAAAAAAAAAAf////4AAAAAAAAAAAAAAAD////8AAAAAAAAAAAAAAAAf///+AAAAAAA' +
  'AAAAAAAAAD////AAAAAAAAAAAAAAAAAP///gAAAAAAAAAAAAAAAAB///wAAAAAAAAAAAAAAAAAP//4AAAAAAAAAAAAAAAAAA' +
  '//8AAAAAAAAAAAAAAAAAAB/4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── chat — 비트맵 실루엣(128×128, 잉크 55.3%)
//   원본 마스크: chat.png
var MASK_CHAT = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAD///+AAAAAAAAAAAAAAAAf/////AAAAAAAAAAAAAAA//////+AAAAAAAAAAAAAD///////8AAAAAAAAAAAAH////' +
  '////4AAAAAAAAAAAH/////////wAAAAAAAAAAH//////////AAAAAAAAAAH//////////8AAAAAAAAAH///////////gAAAA' +
  'AAAAH///////////+AAAAAAAAD////////////4AAAAAAAD/////////////AAAAAAAB/////////////4AAAAAAA///////' +
  '///////gAAAAAAf/////////////8AAAAAAP//////////////gAAAAAP//////////////8AAAAAH///////////////gAA' +
  'AAB///////////////8AAAAA////////////////gAAAAf///////////////8AAAAP////////////////AAAAH////////' +
  '////////4AAAB/////////////////AAAA/////////////////4AAAf////////////////+AAAH/////////////////wA' +
  'AD/////////////////8AAB//////////////////gAAf/////////////////4AAP//////////////////AAD/////////' +
  '/////////wAA//////////////////+AAf//////////////////gAH//////////////////4AB///////////////////A' +
  'A///////////////////wAP//////////////////8AD///////////////////gA///////////////////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AD///////////////////g' +
  'A///////////////////4AP//////////////////8AD///////////////////AAf//////////////////wAH/////////' +
  '/////////4AB//////////////////+AAP//////////////////gAD//////////////////wAA//////////////////8A' +
  'AH/////////////////+AAB//////////////////gAAP/////////////////wAAD/////////////////8AAAf////////' +
  '////////+AAAD/////////////////gAAA/////////////////wAAAH////////////////4AAAA////////////////+AA' +
  'AAH////////////////AAAAA////////////////gAAAAP///////////////wAAAAB///////////////4AAAAAP///////' +
  '///////8AAAAAB//////////////+AAAAAAH//////////////AAAAAAA//////////////gAAAAAAH/////////////gAAA' +
  'AAAA/////////////wAAAAAAAD////////////4AAAAAAAAP///////////4AAAAAAAAB///////////4AAAAAAAAAH/////' +
  '/////8AAAAAAAAAA//////////8AAAAAAAAAAP/////////8AAAAAAAAAAD/////////8AAAAAAAAAAA/////////4AAAAAA' +
  'AAAAAf////////wAAAAAAAAAAAH////////gAAAAAAAAAAAB///////8AAAAAAAAAAAAA///+P//AAAAAAAAAAAAAAP//+AA' +
  'AAAAAAAAAAAAAAAD///AAAAAAAAAAAAAAAAAA///gAAAAAAAAAAAAAAAAAP//gAAAAAAAAAAAAAAAAAH//wAAAAAAAAAAAAA' +
  'AAAAB//wAAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAAD/8AAAAAAAAAAAAAAAAAAA/+AAAA' +
  'AAAAAAAAAAAAAAAP+AAAAAAAAAAAAAAAAAAAH/AAAAAAAAAAAAAAAAAAAB/gAAAAAAAAAAAAAAAAAAAfgAAAAAAAAAAAAAAA' +
  'AAAAHwAAAAAAAAAAAAAAAAAAABwAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── book — 비트맵 실루엣(128×128, 잉크 42.1%)
//   원본 마스크: book.png
var MASK_BOOK = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAD//AAAAAAAAAAAAD//AAAH///wAAAAAAAAAA///+AAD////4AAAA' +
  'AAAAH////4AD/////4AAAAAAAf/////AA//////wAAAAAAf/////wAf//////AAAAAA//////+AH//////+AAAAB///////g' +
  'B///////4AAAB///////4Af///////wAAB///////+AH////////AAB////////gB////////4AB////////4Af////////g' +
  'B////////+AH////////+B/////////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4Af//wD//////////wD//+AH//4AD/////////AAf//gB//8AAH///////+AAD//4Af/+AAAP///' +
  '///+AAAf/+AH//gAAA//////8AAAH//gB//wAAAD/////8AAAA//4Af/8AAAAf////+AAAAP/+AH//AAAAB////+AAAAD//g' +
  'B//wAAAAH////AAAAA//4Af/8AAAAA////AAAAAP/+AH//AAAAAH///gAAAAD//gB//wAAAAA///wAAAAA//4Af/8AAAAAH/' +
  '/4AAAAAP/+AH//AAAAAB//+AAAAAD//gB//wAAAAAP//AAAAAA//4Af/8AAAAAD//wAAAAAP/+AH//AAAAAA//8AAAAAD//g' +
  'B//wAAAAAP//AAAAAA//4Af/8AAAAAD//wAAAAAP/+AH//AAAAAA//8AAAAAD//gB//wAAAAAP//AAAAAA//4Af/8AAAAAD/' +
  '/wAAAAAP/+AH//AAAAAA//8AAAAAD//gB//wAAAAAP//AAAAAA//4Af/8AAAAAD//wAAAAAP/+AH//AAAAAA//8AAAAAD//g' +
  'B//wAAAAAP//AAAAAA//4Af/8AAAAAD//wAAAAAP/+AH//AAAAAA//8AAAAAD//gB//wAAAAAP//AAAAAA//4Af/8AAAAAD/' +
  '/wAAAAAP/+AH//AAAAAA//8AAAAAD//gB//wAAAAAP//AAAAAA//4Af/8AAAAAD//wAAAAAP/+AH//AAAAAA//8AAAAAD//g' +
  'B//wAAAAAP//AAAAAA//4Af/8AAAAAD//wAAAAAP/+AH//AAAAAA//8AAAAAD//gB//wAAAAAP//AAAAAA//4Af/8AAAAAD/' +
  '/wAAAAAP/+AH//AAAAAA//8AAAAAD//gB//wAAAAAP//AAAAAA//4Af/8AAAAAD//wAAAAAP/+AH//AAAAAA//8AAAAAD//g' +
  'B//wAAAAAP//AAAAAA//4Af/8AAAAAD//wAAAAAP/+AH//AAAAAA//8AAAAAD//gB//wAAAAAP//AAAAAA//4Af/8AAAAAD/' +
  '/wAAAAAP/+AH//AAAAAA//8AAAAAD//gB//wAAAAAP//AAAAAA//4Af/8AAAAAD//wAAAAAP/+AH//gAAAAA//8AAAAAH//g' +
  'B//4AAAAAP//AAAAAB//4Af//AAAAAD//wAAAAA//+AH//4AAAAA//+AAAAAf//gB///gAAAAP//gAAAAf//4Af///4AAAD/' +
  '/4AAAH///+AH////8AAB//+AAA/////gB/////4AAf//wAB/////4Af/////wAP//+AD/////+AH//////gP///4H//////g' +
  'B///////f///////////4Af//////////////////+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4AP//////////////////8AB//////////////////+AAH/////////////////+AAAAAD//////' +
  '//////wAAAAAAAAAf/////////4AAAAAAAAAAAf////////gAAAAAAAAAAAAf//////+AAAAAAAAAAAAAA//////8AAAAAAA' +
  'AAAAAAAD/////8AAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAA////8AAAAAAAAAAAAAAAAD///8AAAAAAAAAAAAAAAAAf/' +
  '/+AAAAAAAAAAAAAAAAAB///AAAAAAAAAAAAAAAAAAP//AAAAAAAAAAAAAAAAAAB//gAAAAAAAAAAAAAAAAAAH/gAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── kiosk — 비트맵 실루엣(128×128, 잉크 36.0%)
//   원본 마스크: kiosk.png
var MASK_KIOSK = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAD/////////AAAAAAAAAAAB/////////4AAAAAAAAAAA//////////AAAAAAAAAAAP/////////wAAAAA' +
  'AAAAAH/////////+AAAAAAAAAAB//////////gAAAAAAAAAAf/////////4AAAAAAAAAAH/////////+AAAAAAAAAAB/wAAA' +
  'AAAD/gAAAAAAAAAAf4AAAAAAAf4AAAAAAAAAAH+AAAAAAAH+AAAAAAAAAAB/gAAAAAAB/gAAAAAAAAAAf4AAAAAAAf4AAAAA' +
  'AAAAAH+AAAAAAAD+AAAAAAAAAAB/gAAAAAAA/wAAAAAAAAAAf4AAAAAAAP8AAAAAAAAAAH+AAAAAAAD/AAAAAAAAAAB/gAAA' +
  'AAAA/wAAAAAAAAAAfwAAAAAAAP8AAAAAAAAAAP8AAAAAAAD/AAAAAAAAAAD/AAAAAAAA/wAAAAAAAAAA/wAAAAAAAP8AAAAA' +
  'AAAAAP8AAAAAAAD/AAAAAAAAAAD/AAAAAAAA/wAAAAAAAAAA/wAAAAAAAP8AAAAAAAAAAP8AAAAAAAD/AAAAAAAAAAD/AAAA' +
  'AAAA/wAAAAAAAAAA/wAAAAAAAP8AAAAAAAAAAP8AAAAAAAD/AAAAAAAAAAD/AAAAAAAA/wAAAAAAAAAA/wAAAAAAAP8AAAAA' +
  'AAAAAP8AAAAAAAB/AAAAAAAAAAD+AAAAAAAAfwAAAAAAAAAB/gAAAAAAAH+AAAAAAAAAAf4AAAAAAAB/gAAAAAAAAAH+AAAA' +
  'AAAAf4AAAAAAAAAB/gAAAAAAAH+AAAAAAAAAAf4AAAAAAAB/gAAAAAAAAAH+AAAAAAAAf4AAAAAAAAAB/gAAAAAAAH+AAAAA' +
  'AAAAAf4AAAAAAAB/gAAAAAAAAAH+AAAAAAAAf4AAAAAAAAAB/gAAAAAAAH+AAAAAAAAAAf4AAAAAAAB/gAAAAAAAAAH/AAAA' +
  'AAAAf4AAAAAAAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAA' +
  'AAAAAf//////////gAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAH/////' +
  '/////4AAAAAAAAAB//////8AAf+AAAAAAAAAAf/////+AAH/gAAAAAAAAAH//////gAA/4AAAAAAAAAB/4AAf/4AAf+AAAAA' +
  'AAAAAf+AAD//AAP/gAAAAAAAAAH/gAA//////4AAAAAAAAAB/4AAP/////+AAAAAAAAAAf/AAH//////gAAAAAAAAAH/////' +
  '/7/3/4AAAAAAAAAB//////4AAf+AAAAAAAAAAf/////+AAH/gAAAAAAAAAH//////gAA/4AAAAAAAAAB//////4AAf+AAAAA' +
  'AAAAAf//////AAH/gAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAH/////' +
  '/////4AAAAAAAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAA' +
  'AAAAAf//////////gAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAH/////' +
  '/////4AAAAAAAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAA' +
  'AAAAAf//////////gAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAH/////' +
  '/////4AAAAAAAAAB///////////AAAAAAAAAA///////////wAAAAAAAAAP//////////8AAAAAAAAAD///////////gAAAA' +
  'AAAAB///////////4AAAAAAAAAf//////////+AAAAAAAAAH///////////wAAAAAAAAD///////////8AAAAAAAAA//////' +
  '//////AAAAAAAAAP///////////4AAAAAAAAH///////////+AAAAAAAAB////////////gAAAAAAAA////////////8AAAA' +
  'AAAAP////////////AAAAAAAAD////////////wAAAAAAAB////////////+AAAAAAAAf////////////gAAAAAAAH//////' +
  '//////8AAAAAAAD/////////////AAAAAAAA/////////////wAAAAAAAP////////////8AAAAAAAD/////////////AAAA' +
  'AAAA/////////////wAAAAAAAP////////////8AAAAAAAD/////////////AAAAAAAA/////////////wAAAAAAAH//////' +
  '//////4AAAAAAAA////////////8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── gimbap — 비트맵 실루엣(128×128, 잉크 40.2%)
//   원본 마스크: gimbap.png
var MASK_GIMBAP = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAD/wAAAAAAAAAAAAAAAAAAf//4AAAAAAAAAAAAAAA' +
  'AA////wAAAAAAAAAAAAAAAB/////AAAAAAAAAAAAAAAB/////8AAAAAAAAAAAAAAB//////wAAAAAAAAAAAAAA//////+AAA' +
  'AAAAAAAAAAAP+H////wAAAAAAAAAAAAB//+H////AAAAAAAAAAAAD///8f///4AAAAAAAAAAAH////x////AAAAAAAAAAAH/' +
  '////H///4AAAAAAAAAAH/////8f///AAAAAAAAAAH//////z///4AAAAAAAAAH//////+f//+AAAAAAAAAH///////x///wA' +
  'AAAAAAAAAH/////+P//8AAAAAAAAAAAD/////5///gAAAAAAAP//gD/////P//8AAAAAAA////gP////5///AAAAAAB/////' +
  'A/////P//wAAAAAD/////+D////z//+AAAAAD//////4P///+f//gAAAAD///////g////z//4AAAAB///////8H///+f//A' +
  'AAAB////////w////n//wAAAA/////////H///8//8AAAAf////////4////v//AAAAf///+D////H///5//4AAAP///8AH/' +
  '//4////f/+AAAH///8AAf///H///z//gAAD///+AAD///4///+//4AAB////AAAf///H///n/+AAAf///gAAD///4///5//g' +
  'AAP///4AAA////P///f/4AAH///+AAAP///5///3/+AAD////gAAD///+P//8//gAA//wf8AAA////z///v/4AAf/wD/AAAf' +
  '8P/+f//7/+AAP/4Af4AAH+A//j//+//gAD/8AH/AAH/AD/8///n/wAB/+AA/8AD/wAf/H//9/8AAf/AAP/+f/4AH/5///f/A' +
  'AH/gAD////+AA/+f//3/wAD/4AA/////gAH/z//9/4AA/8AAP////4AB/8///f+AAf/AAD////+AAf/H//3/gAH/wAB/////' +
  'wAD/5////wAB/8AAf////8AA/+f///8AAf+AAP/////AAP/z////AAP/gAD/+AP/4AD/8////gAD/4AB/+AA/+AA//P///wA' +
  'A//AA/+AAD/wAP/z///8AAP/wAP/AAAf+AD/+f//+AAH/8AH/wAAH/wB//n///gAB//gH/4AAA/+Af/5///wAAf/8H/+AAAP' +
  '/4f/+f//4AAH/////AAAB/////n//8AAB/////wAAAf////5//+AAAf////8AAAH////+f//AAAH////+AAAB/////n//gAA' +
  'B/////gAAAP////5//wAAAf////4AAAH////+f/4AAAH/////AAAB/////n/4AAAB//4f/wAAAf/w//5/8AAAAf/4B/8AAAH' +
  '/wH/+f/AAAAH/+AH/gAAD/wA//n/gAAAB//AA/4AAA/4AH/5/4AAAAP/wAH/AAAf8AB/+f8AAAAD/4AB/4AAP/AAf/v+AAAA' +
  'A/+AAP/AAH/gAH/7/gAAAAP/gAD/4AD/4AB/8/wAAAAD/4AAf/AD/8AAf/P4AAAAAf+AAH//v//AAH/z+AAAAAH/wAB/////' +
  'wAB/9/AAAAAB/8AAP////4AA/+fgAAAAAP/AAD////+AAP/nwAAAAAD/4AA/////gAD/74AAAAAA/+AAP////4AB/88AAAAA' +
  'AH/wAD////+AA//OAAAAAAB/+AB/8AP/wAP/ngAAAAAAP/gAf8AA/8AH/5wAAAAAAD/8AH+AAD/AD/84AAAAAAAf/wD/AAA/' +
  '4D//MAAAAAAAH//D/gAAH/j//gAAAAAAAA////4AAA////4AAAAAAAAH///8AAAP///8AAAAAAAAB////AAAD///+AAAAAAA' +
  'AAP///wAAA////gAAAAAAAAB///8AAAP///wAAAAAAAAAP///gAAD///4AAAAAAAAAB///4AAB///8AAAAAAAAAAP///AAA/' +
  '//+AAAAAAAAAAB///4AAf///AAAAAAAAAAAP///gAP///gAAAAAAAAAAB///+Af///wAAAAAAAAAAAP////////4AAAAAAAA' +
  'AAAA////////4AAAAAAAAAAAAH///////8AAAAAAAAAAAAAf//////8AAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAH/////' +
  '+AAAAAAAAAAAAAAA/////+AAAAAAAAAAAAAAAB////8AAAAAAAAAAAAAAAAD///4AAAAAAAAAAAAAAAAAB//gAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── elevator — 비트맵 실루엣(128×128, 잉크 45.6%)
//   원본 마스크: elevator.png
var MASK_ELEVATOR = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAf///////4AAAAAAAAAAAAf////////gAAAAAAAAAAAP////////8AAAAAAAAAAAD/////////gAAAAA' +
  'AAAAAB/////////4AAAAAAAAAAAf/////////AAAAAAAAAAAH/////////wAAAAAAAAAAB/////////8AAAAAAAAAAAf/8//' +
  '//n//AAAAAAAAAAAH/+H///w//wAAAAAAAAAAB//A///4P/8AAAAAAAAAAAf/gH//+D//AAAAAAAAAAAH/wA///g//wAAAAA' +
  'AAAAA//4AH//4P//wAAAAAAAAD//8AA//+D///wAAAAAAAB//+AAH//g////AAAAAAAB///AAA//4P///4AAAAAAAf//gAAP' +
  '/+D////AAAAAAAP//4GGD//g////wAAAAAAH///Dhw/4YOH//+AAAAAAB////4f/8CDB///gAAAAAA////+H//AAgf//8AAA' +
  'AAAP////h//4AAH///AAAAAAD////4f//AAD///wAAAAAA////+H//4AB///8AAAAAAP////h///AA////AAAAAAD////4f/' +
  '/4Af///wAAAAAA////+H///AP///8AAAAAAP////h///4H////AAAAAAD////4f///D////wAAAAAA//////////////8AAA' +
  'AAAP//////////////AAAAAAD//////////////wAAAAAA//////////////8AAAAAAP//////////////AAAAAAD///////' +
  '///////wAAAAAA//////////////8AAAAAAP/4AAAAAAAAAB//AAAAAAD/8AAAAAAAAAAP/wAAAAAA//AAAAAAAAAAD/8AAA' +
  'AAAP/wAAAAAAAAAA//AAAAAAD/8AAAAAAAAAAP/wAAAAAA//AAAAAAAAAAD/8AAAAAAP/wAAAAAAAAAA//AAAAAAD/8AAAAA' +
  'AAAAAP/wAAAAAA//AAAAAAAEAAD/8AAAAAAP/wAB/AAAP4AA//AAAAAAD/8AA/8AAH/AAP/wAAAAAA//AAf/gAD/4AD/8AAA' +
  'AAAP/wAP/4AB//AA//AAAAAAD/8AD//AAf/wAP/wAAAAAA//AA//wAP/8AD/8AAAAAAP/wAP/8AD//gA//AAAAAAD/8AD//A' +
  'A//wAP/wAAAAAA//AA//wAH/8AD/8AAAAAAP/wAP/4AB//AA//AAAAAAD/8AB/+AAf/gAP/wAAAAAA//AAf/AAD/4AD/8AAA' +
  'AAAP/wAD/gAAf8AA//AAAAAAD/8AAPgAAB8AAP/wAAAAAA//AAD4AAAfAAD/8AAAAAAP/wAP/8AD//gA//AAAAAAD/8AP//g' +
  'B//8AP/wAAAAAA//AH//+A///gD/8AAAAAAP/wD///gf//8A//AAAAAAD/8A///8H///AP/wAAAAAA//AP///D///4D/8AAA' +
  'AAAP/wH///w///+A//AAAAAAD/8B///8P///gP/wAAAAAA//Af///D///4D/8AAAAAAP/wH///w///+A//AAAAAAD/8B///8' +
  'P///gP/wAAAAAA//Af///D///4D/8AAAAAAP/wH///w///+A//AAAAAAD/8B///8P///gP/wAAAAAA//Af///D///4D/8AAA' +
  'AAAP/wH///w///+A//AAAAAAD/8B///8P///gP/wAAAAAA//Af///D///4D/8AAAAAAP/wH///w///+A//AAAAAAD/8B///8' +
  'P///gP/wAAAAAA//Af///D///4D/8AAAAAAP/wH///w///+A//AAAAAAD/8B///8P///gP/wAAAAAA//Af///D///4D/8AAA' +
  'AAAP/wH///w///+A//AAAAAAD/8B///8P///gP/wAAAAAA//Af///D///4D/8AAAAAAP/wH///w///+A//AAAAAAD/8B///8' +
  'P///gP/wAAAAAA//Af///D///4D/8AAAAAAP/wH///w///+A//AAAAAAD/8B///8P///gP/wAAAAAA//Af///D///4D/8AAA' +
  'AAAP/wH///w///+A//AAAAAAD/8B///8P///gP/wAAAAAA//Af///D///4D/8AAAAAAP/wH///w///+A//AAAAAAD/8B///8' +
  'P///gP/wAAAAAA//Af///D///4D/8AAAAAAP/wH///w///+A//AAAAAAD/8B///8P///gP/wAAAAAA//Af///D///4D/8AAA' +
  'AAAP/wH///w///+A//AAAAAAD/8B///8P///gP/wAAAAAA//gf///D///4H/8AAAAAAP//////////////AAAAAAD///////' +
  '///////wAAAAAA//////////////8AAAAAAP//////////////AAAAAAD//////////////wAAAAAAf/////////////4AAA' +
  'AAAH/////////////+AAAAAAA//////////////AAAAAAAH/////////////gAAAAAAA/////////////wAAAAAAAH//////' +
  '//////4AAAAAAAAP///////////4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── manse — 비트맵 실루엣(128×128, 잉크 35.8%)
//   원본 마스크: manse.png
var MASK_MANSE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAH/wAAAAAAAAA' +
  'AAAAAAAAAf//gAAAAAAAAAAAAAAAAAf//+AAAAAAAAAAAAAAAAAf///4AAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAH//' +
  '//8AAAAAAAAAAAAAAAD/////gAAAAAAAAAAAAAAB/////8AAAAAAAAAAAAAAA//////AAAAAAAAAAAAAAAf/////4AAAAAAA' +
  'AAAAAAAP//////AAAAAAAAAAAAAAD//////4AAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAf//////wAAAAAAAAeAAAAP///' +
  '///8AAAAfAAAf4AAAD///////AAAAf8AAf/gAAA///////4AAAf/gAP/8AAAf//////+AAAP/8AD//AAAH///////gAAD//A' +
  'B//4AAB///////4AAB//4Af/+AAAf///////AAAf/+AH//gAAH///////wAAH//gB//8AAB///////8AAD//4Af//AAAf///' +
  '////AAA//+AH//8AAH///////wAA///gB///wAB///////8AA///4AP//8AAf///////AAP//8AD///AAH///////wAD///A' +
  'A///wAB///////8AA///wAP//8AAf/8AAP/+AAP//8AB///AAH/8AAA//gAD//+AAf//wAA//AAAP/4AA///gAD//+AAP/wA' +
  'AD/8AAf//wAA///wAD/+AAA//AAP//8AAP//8AA//wAAf/wAD///AAB///gAH/+AAP/4AB///gAAf//8AA//wAH/+AA///4A' +
  'AD///gAP/+AH//AAf//8AAA///4AB//4H//wAH///AAAH///AAP/////4AD///gAAB///4AD/////8AB///4AAAP///AAf//' +
  '//+AA///8AAAB///4AD/////AAf///AAAAf///AAP////gAP///gAAAD///8AB////gAP///wAAAA////gAP///wAH///8AA' +
  'AAH///8AA///4AD///+AAAAA////wAH//4AD////AAAAAH////AB//+AD////wAAAAB////8Af//gD////4AAAAAP////4P/' +
  '/8H////8AAAAAB//////////////+AAAAAAP//////////////AAAAAAD//////////////wAAAAAAf/////////////4AAA' +
  'AAAD/////////////8AAAAAAAf////////////+AAAAAAAD/////////////AAAAAAAAf////////////gAAAAAAAD//////' +
  '//////wAAAAAAAAf///////////4AAAAAAAAD///////////8AAAAAAAAAf//////////+AAAAAAAAAD///////////gAAAA' +
  'AAAAAf//////////gAAAAAAAAAD//////////wAAAAAAAAAAf/////////4AAAAAAAAAAH/////////+AAAAAAAAAAA/////' +
  '/////AAAAAAAAAAAH/////////wAAAAAAAAAAB/////////4AAAAAAAAAAAP////////+AAAAAAAAAAAD/////////AAAAAA' +
  'AAAAAA/////////wAAAAAAAAAAAP////////8AAAAAAAAAAAD/////////AAAAAAAAAAAAf////////gAAAAAAAAAAAH////' +
  '////4AAAAAAAAAAAB////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAH////////4AAAAAAAAAAAB////////+AAAAAA' +
  'AAAAAAf////////gAAAAAAAAAAAH////////4AAAAAAAAAAAB////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAH////' +
  '////4AAAAAAAAAAAB////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAH////////4AAAAAAAAAAAB////////+AAAAAA' +
  'AAAAAAH///////+AAAAAAAAAAAAAH//////4AAAAAAAAAAAAAAAf///4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── lonely — 비트맵 실루엣(128×128, 잉크 48.0%)
//   원본 마스크: lonely.png
var MASK_LONELY = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAA//AAAAAAAAAAAAAAAAAAB///AAAAAAAAAAAAAAAAAD///8AAAAAAAAAAAAAAAAD////wAAAAAAAA' +
  'AAAAAAAB////+AAAAAAAAAAAAAAAB/////4AAAAAAAAAAAAAAA//////AAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAP///' +
  '///AAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAD///////AAAAAAAAAAAAAB///////4AAAAAAAAAAAAAf//////+AAAAAAA' +
  'AAAAAAP///////wAAAAAAAAAAAAH///////+AAAAAAAAAAAAB////////gAAAAAAAAAAAA////////4AAAAAAAAAAAAP////' +
  '////AAAAAAAAAAAAH////////wAAAAAAAAAAAB////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAH////////4AAAAAA' +
  'AAAAAD////////+AAAAAAAAAAAA/////////wAAAAAAAAAAAP////////8AAAAAAAAAAAD/////////AAAAAAAAAAAA/////' +
  '////wAAAAAAAAAAAP////////8AAAAAAAAAAAD/////////AAAAAAAAAAAA/////////wAAAAAAAAAAAP////////8AAAAAA' +
  'AAAAAD/////////AAAAAAAAAAAA/////////wAAAAAAAAAAAH////////4AAAAAAAAAAAB////////+AAAAAAAAAAAAf////' +
  '////gAAAAAAAAAAAD////////wAAAAAAAAAAAA////////8AAAAAAAAAAAAP////////AAAAAAAAAAAAB///xx///gAAAAAA' +
  'AAAAAAf//wcH//4AAAAAAAAAAAAD//4Pg//8AAAAAAAAAAAAAf/4P/H//AAAAAAAAAAAAAH/8P/4//gAAAAAAAAAAAAA//H/' +
  '/H/wAAAAAAAAAAAAAH/7//5/4AAAAAAAAAAAAD4////+P8P4AAAAAAAAAAD+H////j/H/wAAAAAAAAAD/w////8/D//AAAAA' +
  'AAAAB/+D/////h//4AAAAAAAAB//4H////w///AAAAAAAAA///B///8wf//8AAAAAAAAf//4f//+AP///gAAAAAAAP//+P//' +
  '/gP///4AAAAAAAH///////4P////AAAAAAAB///////8P////4AAAAAAA///////+H/////AAAAAAAP///////j/////wAAA' +
  'AAAH/////////////8AAAAAAB//////////////gAAAAAA///////+f8f///4AAAAAAP///////H/D////AAAAAAD///////' +
  '7/g////wAAAAAA/////////4H///8AAAAAAP//////8f8B////AAAAAAD///////v/Af///wAAAAAA/////////wH///8AAA' +
  'AAAP//////x/8B////AAAAAAD//////8//Af///wAAAAAA///////v/4P///8AAAAAAP//////////////AAAAAAD//////z' +
  '///////wAAAAAA//////4///////8AAAAAAP/////+P///////AAAAAAD//////////////wAAAAAA//////n///////8AAA' +
  'AAAP/////x////////AAAAAAD/////8f///////wAAAAAA/////+P///////8AAAAAAP/////j////////AAAAAAD/////x/' +
  '///////wAAAAAA/////4////////8AAAAAAP////+P////////AAAAAAD/////H////////wAAAAAA/////j////////8AAA' +
  'AAAP////w/////////AAAAAAD////8f////////wAAAAAA////+P////////8AAAAAAP////D/////////AAAAAAB////x//' +
  '///////wAAAAAAf///4/////////8AAAAAAD///8f/////////AAAAAAA///+H/////////wAAAAAAH///j/////////8AAA' +
  'AAAA///x//////////AAAAAAAH//4//////////wAAAAAAA//8f/////////8AAAAAAAD/8P//////////AAAAAAAAD8H///' +
  '///////wAAAAAABgeH//////////8AAAAAAAeH////////////AAAAAAAH/////////////wAAAAAAB/////////////8AAA' +
  'AAAAf/////////////AAAAAAAH/////////////wAAAAAAB/////////////8AAAAAAAf/////////////AAAAAAAH//////' +
  '///////wAAAAAAB/////////////8AAAAAAAf/////////////AAAAAAAH/////////////wAAAAAAB/////////////8AAA' +
  'AAAAf/////////////AAAAAAAH/////////////wAAAAAAA/////////////4AAAAAAAP////////////+AAAAAAAB//////' +
  '///////AAAAAAAAH////////////AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── together — 비트맵 실루엣(128×128, 잉크 59.7%)
//   원본 마스크: together.png
var MASK_TOGETHER = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAHgAAAAAHgAAAAAAAAAAAAf/wAAAAf/gAAAAAAAAAAA///AA' +
  'AA///AAAAAAAAAAAf//8AAAf//4AAAAAAAAAAf///gAAf///gAAAAAAAAAP///8AAP///8AAAAAAAAAH////gAH////gAAAA' +
  'AAAAD////8AD////8AAAAAAAAA/////gB/////AAAAAAAAAf////4Af////4AAAAAAAAP/////AP/////AAAAAAAAD/////4' +
  'H/////wAAAAAAAB////////////+AAAAAAAAf////////////gAAAAAAAH////////////4AAAAAAAD/////////////AAAA' +
  'AAAA/////////////wAAAAAAAP////////////8AAAAAAAD/////////////AAAAAAAA/////////////wAAAAAAAP//////' +
  '//////8AAAAAAAD/////////////AAAAAAAA/////////////wAAAAAAAP////////////8AAAAAAAB////////////+AAAA' +
  'AAAAf////////////gAAAAAAAH////////////4AAAAAAAA////////////8AAAAAAAAP////////////AAAAAAAAD//////' +
  '//////wAAAAAAAAf///////////4AAAAAAAAP////////////AAAAAAAAD////////////wAAAAAAAA////////////8AAAA' +
  'AAAAf////////////gAAAAAAAP////////////8AAAAAAAf/////////////4AAAAAA///////////////gAAAAAf///////' +
  '///////+AAAAAP///////////////wAAAAH///////////////+AAAAD////////////////wAAAB////////////////+AA' +
  'AA/////////////////wAAAP////////////////8AAAH/////////////////gAAB/////////////////4AAA/////////' +
  '/////////AAAP/////////////////wAAH/////////////////+AAH//////////////////4AH///////////////////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gAP//////////////////AAB/////////' +
  '/////////gAAP/////////////////wAAD/////////////////8AAAf////////////////+AAAD/////////////////AA' +
  'AA/////////////////wAAAP////////////////8AAAD/////////////////AAAA/////////////////wAAAP////////' +
  '////////8AAAD/////////////////AAAB/////////////////4AAAf////////////////+AAAH/////////////////gA' +
  'AB/////////////////4AAAf////////////////+AAAH/////////////////gAAB/////////////////4AAAf////////' +
  '////////+AAAH/////////////////gAAB/////////////////4AAAf////////////////+AAAH/////////////////gA' +
  'AB/////////////////4AAAf////////////////+AAAH/////////////////gAAB/////////////////4AAAf////////' +
  '////////+AAAH/////////////////gAAB/////////////////4AAAf////////////////+AAAH/////////////////gA' +
  'AB/////////////////4AAAf////////////////+AAAH/////////////////gAAB/////////////////4AAAf////////' +
  '////////+AAAH/////////////////gAAB/////////////////4AAAP////////////////8AAAD/////////////////AA' +
  'AA/////////////////wAAAH////////////////4AAAB////////////////+AAAAP////////////////AAAAB////////' +
  '////////gAAAAP///////////////wAAAAB///////////////4AAAAAH//////////////4AAAAAAH/////////////wAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── comfort — 비트맵 실루엣(128×128, 잉크 58.6%)
//   원본 마스크: comfort.png
var MASK_COMFORT = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAB/wAAAAAAAAAAAAAAAAAAD//AAAAHAAAAAAAAAAAAAD//8AAA//AAAAAAAAAAAAD///wAA//+AAAAAA' +
  'AAAAAB///+AA///wAAAAAAAAAAA////wAf///AAAAAAAAAAAf///+AP///4AAAAAAAAAAP////wH////AAAAAAAAAAH////+' +
  'D////4AAAAAAAAAB/////h////+AAAAAAAAAA/////8/////wAAAAAAAAAP//////////+AAAAAAAAAH///////////gAAAA' +
  'AAAAB///////////4AAAAAAAAAf///////////AAAAAAAAAH///////////wAAAAAAAAB///////////8AAAAAAAAAf/////' +
  '//////AAAAAAAAAH///////////wAAAAAAAAB///////////8AAAAAAAAAf///////////AAAAAAAAAH///////////wAAAA' +
  'AAAAB///////////8AAAAAAAAAf///////////AAAAAAAAAD///////////gAAAAAAAAA///////////4AAAAAAAAAH/////' +
  '/////+AAAAAAAAAB///////////AAAAAAAAAAP//////////wAAAAAAAAAD//////////4AAAAAAAAAA//////////8AAAAA' +
  'AAAAB///////////AAAAAAAAAD///////////gAAAAAAAAB///////////8AAAAAAAAA////////////wAAAAAAAAf//////' +
  '//////AAAAAAAAP////////////4AAAAAAAD/////////////AAAAAAAB/////////////4AAAAAAAf/////////////AAAA' +
  'AAAH/////////////4AAAAAAB/////////////+AAAAAAAf/////////////wAAAAAAH/////////////8AAAAAAB///////' +
  '///////gAAAAAAP/////////////4AAAAAAH/////////////+AAAAAAD//////////////gAAAAAA//////////////4AAA' +
  'AAAP/////////////+AAAAAAH//////////////AAAAAAB//////////////wAAAAAAf/////////////8AAAAAAH///////' +
  '///////gAAAAAB//////////////8AAAAAAf//////////////AAAAAAH//////////////wAAAAAB//////////////8AAA' +
  'AAAf//////////////AAAAAAH//////////////wAAAAAA//////////////4AAAAAAP//////////////AAAAAAH///////' +
  '///////wAAAAAB//////////////+AAAAAA///////////////gAAAAAP//////////////4AAAAAD//////////////+AAA' +
  'AAA///////////////wAAAAAH//////////////8AAAAAB///////////////AAAAAAf//////////////wAAAAAP///////' +
  '///////8AAAAAD///////////////AAAAAA///////////////wAAAAAP//////////////8AAAAAD///////////////gAA' +
  'AAB///////////////4AAAAAf//////////////+AAAAAH///////////////gAAAAB///////////////4AAAAAf///////' +
  '///////+AAAAAH///////////////gAAAAB///////////////4AAAAAf//////////////+AAAAAH///////////////gAA' +
  'AAB///////////////4AAAAAf//////////////+AAAAAH///////////////gAAAAB///////////////4AAAAAf///////' +
  '///////+AAAAAH///////////////gAAAAB///////////////4AAAAAf//////////////+AAAAAH///////////////gAA' +
  'AAB///////////////4AAAAAf//////////////+AAAAAH///////////////gAAAAB///////////////4AAAAAf///////' +
  '///////+AAAAAH///////////////gAAAAB///////////////4AAAAAf//////////////+AAAAAH///////////////gAA' +
  'AAB///////////////4AAAAAf//////////////+AAAAAH///////////////gAAAAB///////////////4AAAAAf///////' +
  '///////+AAAAAH///////////////gAAAAB///////////////4AAAAAP//////////////8AAAAAD//////+P///////AAA' +
  'AAA///////j///////wAAAAAH//////4f//////4AAAAAB//////8H//////8AAAAAAP/////+A//////+AAAAAAB//////A' +
  'D//////AAAAAAAD/////AAP/////AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── dolhareubang — 비트맵 실루엣(128×128, 잉크 38.9%)
//   원본 마스크: dolhareubang.png
var MASK_DOLHAREUBANG = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAH/gAAAAAAAAAAAAAAAAAAP//AAAAAAAAAAAAAAAAAAP//8AAAAAAAAAAAAAAAAAH///gAAAAAAAA' +
  'AAAAAAAAD///+AAAAAAAAAAAAAAAAB////wAAAAAAAAAAAAAAAA////8AAAAAAAAAAAAAAAAf////gAAAAAAAAAAAAAAAP//' +
  '//8AAAAAAAAAAAAAAAD/////AAAAAAAAAAAAAAAB/////4AAAAAAAAAAAAAAAf////+AAAAAAAAAAAAAAAH/////gAAAAAAA' +
  'AAAAAAAD/////8AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAB//////+AAAAAAAAAAAAAB///////4AAAAAAAAAAAAA////' +
  '////AAAAAAAAAAAAAf///////4AAAAAAAAAAAAP////////AAAAAAAAAAAAH////////4AAAAAAAAAAAB////////+AAAAAA' +
  'AAAAAAf////////gAAAAAAAAAAAH////////4AAAAAAAAAAAB////////+AAAAAAAAAAAAf/+AAAD//gAAAAAAAAAAAH/wP/' +
  '/8D/4AAAAAAAAAAAA/h////+H+AAAAAAAAAAAAPj/////8fAAAAAAAAAAAABz//////xgAAAAAAAAAAAAB//////+AAAAAAA' +
  'AAAAABg///////wYAAAAAAAAAAB+P//////8fgAAAAAAAAAAfj/Af/8D/n4AAAAAAAAAAP5/gD/8AP5/AAAAAAAAAAD+fwA/' +
  '/AB+fwAAAAAAAAAA/n4AH/gAfn8AAAAAAAAAAP5+HB/8OH9/AAAAAAAAAAD+/v////9/fwAAAAAAAAAA/v///////38AAAAA' +
  'AAAAAP7///////8/gAAAAAAAAAD+////////P4AAAAAAAAAA/v///////z+AAAAAAAAAAPz///////8/AAAAAAAAAAD8//8f' +
  '+P//PwAAAAAAAAAA/P/+D/B//78AAAAAAAAAAH3//g/wf/++AAAAAAAAAAA5//4P8H//nAAAAAAAAAAAAf/+D/B//4AAAAAA' +
  'AAAAAAH//7/8//+AAAAAAAAAAAAB////////wAAAAAAAAAAAB////////+AAAAAAAAAAAA/////////wAAAAAAAAAAAf////' +
  '////+AAAAAAAAAAAP/+H///h//wAAAAAAAAAAH//gH/+Af/+AAAAAAAAAAB//4AAAAH//wAAAAAAAAAA///AAAAD//8AAAAA' +
  'AAAAAP//4AAAB///gAAAAAAAAAH///AAAA///4AAAAAAAAAB///8AAAf///AAAAAAAAAA////wAAf///wAAAAAAAAAP////g' +
  'B////8AAAAAAAAAH///////////gAAAAAAAAB///////////4AAAAAAAAAf//////////+AAAAAAAAAP///////////wAAAA' +
  'AAAAD///////////8AAAAAAAAA////////////AAAAAAAAAf+AH/////gB/4AAAAAAAAH/AAf////gAP+AAAAAAAAB/wAD//' +
  '//wAB/gAAAAAAAA//+AP///wB//8AAAAAAAAP//8B///4D///AAAAAAAAD///wP//8D///wAAAAAAAA////D///D///8AAAA' +
  'AAAAP///4f//h////gAAAAAAAH///+D//4f///4AAAAAAAB////w//8P///+AAAAAAAAf///+P//H////gAAAAAAAH////h/' +
  '/x////4AAAAAAAB////8f/4f///+AAAAAAAAf////H/+P////gAAAAAAAH////x//j////4AAAAAAAB////8f/4////+AAAA' +
  'AAAAP////H/+P////AAAAAAAAD////x//j////wAAAAAAAA////8f/4////8AAAAAAAAH////H/+P///+AAAAAAAAB////x/' +
  '/j////gAAAAAAAAP///4f/8f///wAAAAAAAAB///+P//H///4AAAAAAAAAH///D//4///4AAAAAAAAAAf//h//+H//4AAAAA' +
  'AAAABg//g///wf/wYAAAAAAAAA+A/Af//+A/AfAAAAAAAAAP8AAP///4AA/wAAAAAAAAD/8Af////gD/8AAAAAAAAA//////' +
  '//////gAAAAAAAAf///////////4AAAAAAAAH///////////+AAAAAAAAB////////////gAAAAAAAAf///////////4AAAA' +
  'AAAAH///////////+AAAAAAAAB////////////gAAAAAAAAf///////////4AAAAAAAAH///////////+AAAAAAAAB//////' +
  '//////gAAAAAAAAf///////////4AAAAAAAAD///////////8AAAAAAAAA////////////AAAAAAAAAP///////////wAAAA' +
  'AAAAB///////////4AAAAAAAAAP//////////8AAAAAAAAAA//////////+AAAAAAAAAAB/////////8AAAAAAAAAAAB////' +
  '////gAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── tree — 비트맵 실루엣(128×128, 잉크 42.5%)
//   원본 마스크: tree.png
var MASK_TREE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAAf/4AAAAAAAAA' +
  'AAAAAAAAAf//gAAAAAAAAAAAAAAAAAP//8AAAAAAAAAAAAAAAAAH///gAAAAAAAAAAAAAAAAB///8AAAAAAAAAAAAAAAAA//' +
  '//AAAAAAAAAAAAAAAAD/////AAAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAA//////+AAAAAAA' +
  'AAAAAAAf//////gAAAAAAAAAAAAAH//////8AAAAAAAAAAADgD///////AHgAAAAAAAAP/A///////wP/AAAAAAAAP/8P///' +
  '///8P/8AAAAAAAH//j///////H//wAAAAAAD//8///////z//8AAAAAAB///H//////4///gAAAAAA///5//////+f//8AAA' +
  'AAAP//+P//////H///AAAAAAD///7//////3///wAAAAAA//////////////8AAAAAAP//////////////AAAAAAD///////' +
  '///////wAAAAAA//////////////8AAAAAA///////////////wAAAAA////////////////AAAAAf///////////////4AA' +
  'AAP///+B///////////AAAAD///+AH//////////wAAAA////AA//////////+AAAAf///gAH/////v////gAAAH///wAA//' +
  '//8AH///4AAAB///8AAP///+AA///+AAAAf//+AAB///+AAH///gAAAH///gAAP///AAA///4AAAA///wAAB///wAAP//+AA' +
  'AAP//8AAAP//4AAB///AAAAD///AAAB//8AAAf//wAAAAf//wAAAP//AAAH//4AAAAD//8AAAB//gAAB//8AAAAAf//gAAAf' +
  '/4AAAf/+AAAAAAf/4AAAD/+AAAP/4AAAAAAAf+AAAA//gAAD/gAAAAAAAD/wAAAP/4AAA/wAAAAAAPwf+AAAD/+AAAf4PwAA' +
  'AAP/H/wAAA//gAAH+P/AAAAP/8/+AAAf/4AAD/v/8AAAH////wAAH//AAB////gAAD/////AAD//wAB////8AAA/////8AB/' +
  '/+AA/////AAAf/////4B///4A/////4AAH/////////////////+AAB//////////////////gAAf/////////////////4A' +
  'AP//////////////////AAH//////////////////4AD/////4//////4f/////AA/////+D/////4P/////wAf/////4P//' +
  '//4H/////+AH//////A////4D//////gB//////wD///8B//////4Af/////+Af//+Af/////+AH//////wH///AP//////g' +
  'B///////A///wP//////4AP//////+P//4P//////8AD///////7//+P///////AAf//////////////////gAD/////////' +
  '/////////wAAP/////////////////wAAAf///8P/////8D////gAAAD///+A/////+Af///wAAAAf///AD////+AH///4AA' +
  'AAH///wAf////AA///+AAAAA///4AD//h/gAH///AAAAAH//4AAf/wf4AA///gAAAAAf/8AAH/4D8AAD//gAAAAAA/wAAA/8' +
  'A/AAAH/AAAAAAAAAAAAP/APgAAAAAAAAAAAAAAAAB/wD4AAAAAAAAAAAAAAAAAf8A+AAAAAAAAAAAAAAAAAH/APAAAAAAAAA' +
  'AAAAAAAAB/wHwAAAAAAAAAAAAAAAAAf8B8AAAAAAAAAAAAAAAAAH/AfAAAAAAAAAAAAAAAAAB/4PwAAAAAAAAAAAAAAAAAf/' +
  'H+AAAAAAAAAAAAAAAAAH///gAAAAAAAAAAAAAAAAB///4AAAAAAAAAAAAAAAAA///+AAAAAAAAAAAAAAAAAP///gAAAAAAAA' +
  'AAAAAAAAH///8AAAAAAAAAAAAAAAAB////AAAAAAAAAAAAAAAAA////4AAAAAAAAAAAAAAAAf////AAAAAAAAAAAAAAAAP//' +
  '//4AAAAAAAAAAAAAAAP/////AAAAAAAAAAAAAAAP/////8AAAAAAAAAAAAAAP//////4AAAAAAAAAAAAB////////8AAAAAA' +
  'AAAAAD/////////8AAAAAAAAAAB//7////3//wAAAAAAAAAA//4////4f/+AAAAAAAAAA//4P////B//wAAAAAAAAB//4P//' +
  '//8P//AAAAAAAAH//AP/////wP//AAAAAAAH/+AP//////AP/+AAAAAAD/+Af//////+A//wAAAAAA/+Af///////4D/8AAA' +
  'AAAH8Af///Af///gH+AAAAAAAAAH//8AAf//4AAAAAAAAAAAB//wAAAf/+AAAAAAAAAAAAP+AAAAAD/AAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── cup — 비트맵 실루엣(128×128, 잉크 42.0%)
//   원본 마스크: cup.png
var MASK_CUP = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAADgAAAAAAAAAAAAAAAAAAAA+AAAAAAAAAAAAAAAAAAAAPwAAAAAAAAAAAAAAAAAAAB+AAAAAAAAAA' +
  'AAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAH+AAAAAAAAAAAAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAAD' +
  '/gAAAAAAAAAAAAAAAAAAA/4AAAAAAAAAAAAAAAAAAAP+AAAAAAAAAAAAAAAAAAAD/wAAAAAAAAAAAAAAAAAAB/8AAAAAAAAA' +
  'AAAAAAPAAAf/AAAAAAAAAAAAAAAB8AAH/wAAAAAAAAAAAAAAAfgAD/+AAAAAAAAAAAAAAAD8AB//h4AAAAAAAAAAAAAA/gA/' +
  '/gPgAAAAAAAAAAAAAP4Af/4D8AAAAAAAAAAAAAD/Af/+AfgAAAAAAAAAAAAA/wP//AH4AAAAAAAAAAAAAP/H//wB/AAAAAAA' +
  'AAAAAAD////4AfwAAAAAAAAAAAAA////8AH+AAAAAAAAAAAAAf///+AB/gAAAAAAAAAAAAH////AAf4AAAAAAAAAAAAD////' +
  'wAP+AAAAAAAAAAAAB////4AH/gAAAAAAAAAAAAf///8AD/4AAAAAAAAAAAAP////AB/+AAAAAAAAAAAAH////gB//AAAAAAA' +
  'AAAAAB////4Af/gAAAAAAAAAAAA////+AP/wAAAAAAAAAAAAP////gH/4AAAAAAAAAAAAD////8B/8AAAAAAAAAAAAA/////' +
  'A/+AAAAAAAAAAAAAP////8f/gAAAAAAAAAAAAB///////wAAAAAAAAAAAAAf//////8AAAAAAAAAAAAAD///////AAAAAAAA' +
  'AAAAAAf//////4AAAAAAAAAAAAAH//////+AAAAAAAAAAAAAB///////gAAAAAAAAAAAAAf//////8AAAAAAAAAAAAAP////' +
  '///AAAAAAAAAAAAAH///////+AAAAAAAAAAAAP////////+AAAAAAAAAAAf/////////8AAAAAAAAAA///////////wAAAAA' +
  'AAAA////AAAA////AAAAAAAAAf/+AAAAAAB//4AAAAAAAAP/+AAAAAAAP//AAAAAAAAH//AAAAAAAB//4AAAAAAAB//wAAAA' +
  'AAAf/+AAAAAAAA//8AAAAAAAH//wAAAAAAAP//AAAAAAAB///PwAAAAAD//wAAAAAAAf////wAAAAA//+AAAAAAAP/////AA' +
  'AAAP//+AAAAAA//////8AAAAD////8AAB////////gAAAA////////////////8AAAAP////////////////gAAAD///////' +
  '/////////4AAAA//////////////gf/AAAAP/////////////gB/wAAAD/////////////wAP8AAAA/////////////4AD/g' +
  'AAAP////////////8AAf4AAAD/////////////AAH+AAAA/////////////wAB/gAAAH////////////8AAf4AAAB///////' +
  '//////AAH+AAAAf////////////gAB/gAAAH////////////4AAf4AAAB////////////+AAH+AAAAf////////////gAB/A' +
  'AAAD////////////wAA/wAAAA////////////8AAP8AAAAP////////////AAH+AAAAD////////////wAD/gAAAAf//////' +
  '/////8AB/4AAAAH////////////gA/8AAAAB////////////8A/+AAAAAP////////////4//AAAAAD///////////////wA' +
  'AAAAf//////////////4AAAAAH//////////////4AAAAAB//////////////8AAAAAA//////////////8AAAAAAP//////' +
  '///////+AAAAAAP//////////////gAAAAAf//////////////4AAAAA///////////////8AAAAB////////////////gAA' +
  'AB////////////////4AAAB//wH/////////gP//AAAA//gA/////////wAf/8AAA//gAH////////wAB//gAAP/wAA/////' +
  '///8AAP/8AAH/4AAH///////+AAB//AAB/+AAA////////AAAf/wAAf/gAAP///////wAAH/8AAH/4AAD///////8AAB//AA' +
  'B//AAA///////+AAA//wAAP/4AAH///////gAAP/4AAB//gAB///////4AAP/8AAAf/+AA////////AAf/+AAAB//8AP////' +
  '///wA//+AAAAP//8H////////D///AAAAA////////////////AAAAAH///////////////gAAAAAf//////////////AAAA' +
  'AAA/////////////+AAAAAAAB////////////8AAAAAAAAH///////////4AAAAAAAAAH//////////AAAAAAAAAAAB/////' +
  '///gAAAAAAAAAAAAAD////+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── car — 비트맵 실루엣(128×128, 잉크 44.5%)
//   원본 마스크: car.png
var MASK_CAR = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAP//////AAAAAAAAAAAAAB////////gAAAAAAAAAAAB////////+AAAAAAAAAAAB/////////4AAAAAAAAAAB/////' +
  '/////AAAAAAAAAAA//////////8AAAAAAAAAAf//////////AAAAAAAAAAH//////////4AAAAAAAAAD//AAAAAAD//AAAAA' +
  'AAAAB/8AAAAAAAD/4AAAAAAAAA/8AAAAAAAAP+AAAAAAAAAP+AAAAAAAAB/wAAAAAAAAH/AAAAAAAAAP8AAAAAAAAB/gAAAA' +
  'AAAAD/gAAAAAAAA/4AAAAAAAAAf8AAAAAAAAP8AAAAAAAAAD/AAAAAAAAH/AAAAAAAAAA/4AAAAAAAD/gAAAAAAAAAH+AAAA' +
  'AAAA/4AAAAAAAAAB/wAAAAAAAf8AAAAAAAAAAP+AAAAAAAP/AAAAAAAAAAD/wAAAAA///wAAAAAAAAAA///wAAA///8AAAAA' +
  'AAAAAP///AAAf///AAAAAAAAAAD///4AAH///4AAAAAAAAAB////AAD//////////////////wAA//////////////////8A' +
  'AP//////////////////AAD//////////////////wAAf/////////////////8AAH/////////////////+AAA/////////' +
  '/////////gAAP/////////////////wAAD/////////////////8AAA//////////////////AAAP/////////////////wA' +
  'AD/////////////////+AAB//////////////////wAA//////////////////8AAf//////////////////gAH/////////' +
  '/////////4AD///////////////////AA///////////////////wAP//////////////////8AD///////////////////g' +
  'B/4A/////////////4B/4Af+AB////////////wAP+AH/AAH///////////wAD/gB/wAA///////////wAA/4Af8AAH/////' +
  '/////8AAP+AH/AAB//////////+AAD/gB/wAAf//////////gAA/4Af+AAH//////////4AAP+AH/gAB//////////+AAH/g' +
  'B/+AA///////////wAD/4Af/+Af//////////+Af/+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB/////gAAAAAAAAP////4Af////gAAAAAAAAA////+AH////wAAAAAAAAAH////g' +
  'B////4AAAAAAAAAB////4Af///+AAAAAAAAAAP///+AH////gAAAAAAAAAD////gB////4AAAAAAAAAB////4Af////AAAAA' +
  'AAAAAf///+AH////4AAAAAAAAAP////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gB///////////////////4AP///4AAAAA' +
  'AAAAAD///+AD///4AAAAAAAAAAAf///AA///+AAAAAAAAAAAD///wAH///AAAAAAAAAAAA///4AA///wAAAAAAAAAAAH//8A' +
  'AH//4AAAAAAAAAAAA//+AAAf/4AAAAAAAAAAAAH/+AAAA/wAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── dog — 비트맵 실루엣(128×128, 잉크 47.0%)
//   원본 마스크: dog.png
var MASK_DOG = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAA//8AAAAAAAAAAAAAAAAAH///+AAAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAP/////wAAAAAAA' +
  'AAAAAAAP//////AAAAAAAAAAAAAAP//////8AAAAAAAAAAAAD/////////AAAAAAAAAAAH/////////+AAAAAAAAAAH/////' +
  '/////4AAAAAAAAAH///////////gAAAAAAAAD///////////+AAAAAAAAD////////////wAAAAAAAB////////////+AAAA' +
  'AAAB/////////////4AAAAAAA//////////////AAAAAAAf/////////////4AAAAAAP//////////////AAAAAAH///////' +
  '///////4AAAAAD///////////////AAAAAB///////////////4AAAAAf//////////////+AAAAAP///////////////wAA' +
  'AAH///////////////+AAAAB////////////////wAAAA////////////////8AAAAP////8/////x/////gAAAH////8D//' +
  '//wP////4AAAB////+Af///8B////+AAAA/////gH///+AP////wAAAP////wA////gD////8AAAD////8AP///wA/////AA' +
  'AA/////AD///8AP////4AAAP////wA////AD////+AAAD////8AP///4A/////gAAA/////AH///+AP////4AAAP////4B//' +
  '//gH////+AAAD/////A////8B/////gAAA/////4f////g/////wAAAP////////////////8AAAD///////+B////////AA' +
  'AAf//////4AB///////gAAAH//////wAAD//////4AAAB//////wAAAP/////+AAAAP/////4AGAB//////AAAAD/////8Af' +
  '+AP/////wAAAAf////+AP/4B/////4AAAAD/////AH/+AP////8AAAAAf////gB//gB////+AAAAAD/H//wAf/4AP//j/AAA' +
  'AAAHh//8AH/+AD//4PAAAAAAAAP//AA//AAf/+AAAAAAAAAD//gAH/gAH//AAAAAAAAAA//4AA/wAB//wAAAAAAAAAH/+AAH' +
  '4AAf/4AAAAAAAAAB//gAA8AAH/+AAAAAAAAAAP/4AAPAAB//AAAAAAAAAAB/+AAD4AAf/gAAAAAAAAAAf/gAD/AAH/4AAAAA' +
  'AAAAAD/8AH//AD/8AAAAAAAAAAA//AD//wA//AAAAAAAAAAAf/4AfD8Af/4AAAAAAAAAAP//AAAAAH//AAAAAAAAAAD//4AA' +
  'AAD//4AAAAAAAAAB///AAAAD//+AAAAAAAAAAf//4AAAB///wAAAAAAAAAP///gAAB///8AAAAAAAAAD////AAD////gAAAA' +
  'AAAAB/////9/////4AAAAAAAAAf//////////+AAAAAAAAAP///////////wAAAAAAAAD//8P////D//8AAAAAAAAA//+Af/' +
  '/+Af//AAAAAAAAAP//AAf/8AD//wAAAAAAAAH//wAAAAAA//+AAAAAAAAB//4AAAAAAH//gAAAAAAAAf/+AAAAAAB//4AAAA' +
  'AAAAH//gAAAAAAf/+AAAAAAAAD//4AAAAAAH//wAAAAAAAB//+AAAAAAB///AAAAAAAA///gAAAAAAf//4AAAAAAAf//4AAA' +
  'AAAH///AAAAAAAP//+AAAAAAB///wAAAAAAH///wAAAAAA///+AAAAAAB///8AAAAAAP///gAAAAAA////gAAAAAH///8AAA' +
  'AAAP///8AAAAAD////AAAAAAD////gAAAAB////wAAAAAB////8AAAAA////+AAAAAAf////gAAAAf////gAAAAAH////+AA' +
  'AAP////4AAAAAB/////wAAAH////+AAAAAAf////+AAAD/////gAAAAAH/////gAAB/////4AAAAAA/////8AAA/////8AAA' +
  'AAAP/////gAAP/////AAAAAAD/////4AAH/////wAAAAAA/////+AAB/////8AAAAAAH/////wAA//////AAAAAAB/////8A' +
  'AP/////gAAAAAAf/////AAD/////4AAAAAAH/////wAA/////+AAAAAAB/////8AAP/////wAAAAAB//////gAH/////+AAA' +
  'AAAf/////4AB//////gAAAAAP/////+AAf/////8AAAAAD///////////////AAAAAA///////////////wAAAAAP///////' +
  '///////+AAAAAD///////////////gAAAAA///////////////wAAAAAP//////////////8AAAAAD///////////////AAA' +
  'AAAf//////////////gAAAAAD/////+AAP/////wAAAAAAf/////AAD/////4AAAAAAAP4///gAAf//x/AAAAAAAAAAH//wA' +
  'AD//4AAAAAAAAAAAAP/wAAAH/4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── cat — 비트맵 실루엣(128×128, 잉크 43.9%)
//   원본 마스크: cat.png
var MASK_CAT = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAABgAAAAAAAAAPAAAAAAAAAB/AAAAAAAAAf8AAAAAAAAA/8AAAAAAAAf/AAAAAAAAAf/wAAAAAAAf/4AAAA' +
  'AAAAH//AAAAAAAP/+AAAAAAAAB//4AAAAAAH//gAAAAAAAAf//AAAAAAD//8AAAAAAAAP//4AAAAAD///AAAAAAAAD///gAA' +
  'AAB///wAAAAAAAA///4AAAAA///8AAAAAAAAP///gAAAAf///AAAAAAAAD///4H//4P///wAAAAAAAA////////////8AAAA' +
  'AAAAP////////////AAAAAAAAD////////////wAAAAAAAAf///////////8AAAAAAAAH///////////+AAAAAAAAB//////' +
  '//////gAAAAAAAAf///////////4AAAAAAAAH///////////+AAAAAAAAB////////////gAAAAAAAAP///////////wAAAA' +
  'AAAAD///////////8AAAAAAAAA////////////AAAAAAAAAH///////////wAAAAAAAAB///////////4AAAAAAAAAf/////' +
  '/////+AAAAAAAAAH///////////gAAAAAAAAB///////////8AAAAAAAAA////////////AAAAAAAAAf///////////4AAAA' +
  'AAAAH///////////+AAAAAAAAD//4P////8D//wAAAAAAAA//4A////8Af/8AAAAAAAAP/8AH////AD//gAAAAAAAH/+AA//' +
  '//gAf/4AAAAAAAB//gAP///wAD/+AAAAAAAA//wAB///8AA//wAAAAAAAP/8AAf//+AAP/8AAAAAAAD//AAH///gAB//AAAA' +
  'AAAA//gAB///4AAf/wAAAAAAAP/4AAf//+AAH/8AAAAAAAD/+AAD///gAB//AAAAAAAB//gAB///4AAf/4AAAAAAAf/4AAf/' +
  '/+AAH/+AAAAAAAH/+AAH///gAB//gAAAAAAB//wAB///4AAf/4AAAAAAAP/8AAf///AAP/8AAAAAAAD//AAP///wAD//AAAA' +
  'AAAA//4AD///8AB//wAAAAAAAP//AB////gAf/8AAAAAAAD//wA////8AP//AAAAAAAA///Af////gH//wAAAAAAAH//////' +
  '//////4AAAAAAAB////////////+AAAAAAAAP////////////gAAAAAAAD////////////wAAAAAAAAf///////////4AAAA' +
  'AAAAH///////////+AAAAAAAAA////////////AAAAAAAAAH///////////gAAAAAAAAB///////////4AAAAAAAAAH/////' +
  '/////8AAAAAAAAAA//////////+AAAAAAAAAAH/////////+AAAAAAAAAAA//////////AAAAAAAAAAAD/////////AAAAAA' +
  'AAAAAAf////////gAAAAAAAAAAAH////////4AAAAAAAAAAAD/////////AAAAAAAAAAAA/////////4AAAAAAAAAAAf////' +
  '////+AAAAAAAAAAAP/////////wAAAAAAAAAAD/////////8AAAAAAAAAAB//////////gAAAAAAAAAAf/////////4AAAAA' +
  'AAAAAH//////////AAAAAAAAAAD//////////wAAAAAAAAAA//////////8AAAAAAAAAAP//////////AAAAAAAAAAH/////' +
  '/////4AAAAAAAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAA' +
  'AAAAA///////////wAAAAAAAAAP//////////8AAAAAAAAAH///////////gAAAAAAAAB///////////8AAAAAAAAA//////' +
  '//////AAAAAAAAAP///////////wAAAAAAAAH///////////+AAAAAAAAB////////////gAAAAAAAAf///////////4AAAA' +
  'AAAAH///////////+AAAAAAAAB////////////gAAAAAAAAf///////////4AAAAAAAAH///////////+AAAAAAAAB//////' +
  '//////gAAAAAAAAf///////////4AAAAAAAAH///////////+AAAAAAAAB////////////gAAAAAAAAP///////////wAAAA' +
  'AAAAD///////////8AAAAAAAAAf//////////+AAAAAAAAAH///////////gAAAAAAAAA///////////wAAAAAAAAAH/////' +
  '/////4AAAAAAAAAAf/////////4AAAAAAAAAAD////v////8AAAAAAAAAAA////4H////AAAAAAAAAAAP///8A////wAAAAA' +
  'AAAAAB////AP///4AAAAAAAAAAAf///wB///+AAAAAAAAAAAD///wAP///AAAAAAAAAAAAf//8AB///gAAAAAAAAAAAB//8A' +
  'AP//wAAAAAAAAAAAAD/wAAAP/AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── fish — 비트맵 실루엣(128×128, 잉크 43.9%)
//   원본 마스크: fish.png
var MASK_FISH = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAHAAAAAAAAAAAAAAAAAAAA/+AAAAAAAAAAAAAAAAAAA//wAAAAAAAAAAAAAAAAAA//+AAAAAAAAAAAAAAAAAA//' +
  '/gAAAAAAAAAAAAAAAAAf//4AAAAAAAAAAAAAAAAAP//+AAAAAAAAAAAAAAAAAP///gAAAAAAAAAAAAAAAAH///4AAAAAAAAA' +
  'AAAAAAAD///+AAAAAAAAAAAAAAAAA////gAAAAAAAAAAAAAAAAf///wAAAAAAAAAAAAAAAAP///8AAAAAAAAAAAAAAAAH///' +
  '+AAAAAAAAAAAAAAAAD////gAAAAAH/gAAAAAAAA////4AAAAAP/+AAAAAAAAf///+AAAAAf//4AAAAAAAP////gAAAAP//+A' +
  'AAAAAAD////4AAAAP///wAAAAAAA/////AAAAH///8AAAAAAAf////wAAAH////gAAAAAAH////+AAAD////4AAAAAAD////' +
  '/wAAB////+AAAAAAA/////8AAA/////gAAAAAAP/////gAAf////4AAAAAAD/////4AAP////+AAAAAAB/////+AAH/////g' +
  'AAAAAB//////gAD/////wAAAAAB//////4AA/////8AAAAAB//////+AAf/////AAAAAB///////AAP/////gAAAAB//////' +
  '/gAD/////wAAAAA///////4AB/////8AAAAAf//////+AAf////+AAAAAf///////gAP/////gAAAAP///////4AD/////4A' +
  'AAAH////////AB//////AAAAD////////4Af/////wAAAA/////////AP/////8AAAAf////////4D//////gAAAP///////' +
  '//B//////4AAAD/////////wf/////+AAAB/////////////////gAAA/////////////////4AAAP/B//////////////+A' +
  'AAH/wH//////////////AAAB/8A//////////////wAAA//AH/8H//////////4AAAP/4B//Af/////////+AAAD//AP/gD/' +
  '/////////AAAB//4D/8Af/////////gAAA///A//AD/////////wAAAf//wH/wAf////////4AAAH//8B/+AD////////4AA' +
  'AB///Af/gA////////8AAAA///wP/4AH///////+AAAAP//8D/+AB////////AAAAD///A//gAP///////wAAAD///gP/8AD' +
  '///////+AAAD///wH//AAf///////wAAA///4B//wAH////////AAAf//+A//8AB////////4AAH///Af//AAf////////AA' +
  'B///4P//wAH////////4AAP//+H//4AB/////////AAD//////+AAf////////4AA///////gAH////////+AAP//////4AB' +
  '/////////wAH//////8AAf////////8AB///////AAH//x//////gAf//////gAD//4P/////4AD//////4AA//+D/////+A' +
  'Af/////8AAf//Af/////gAD//////AAH//wH/////4AAD/////gAD//4A/////+AAAH////4AB//8AP/////gAAA////8AAf' +
  '//AD/////wAAAH///+AAf//gAf////8AAAA////gAP//wAH/////AAAAD///4AP//wAA/////gAAAAf///AP//4AAP////4A' +
  'AAAD///8f//+AAB////+AAAAAP///////gAAf////gAAAAD///////4AAD////4AAAAA////////AAAf///+AAAAAP//////' +
  '/8AAD////wAAAAD////////gAA////8AAAAA////////8AAH////gAAAAf////////gAA////4AAAAH//+Af///4AAH///+A' +
  'AAAB///AH////AAAf///gAAAAP//wA////wAAD///4AAAAD//8AP///+AAAf//+AAAAA///AB////gAAB///AAAAAP//wAf/' +
  '//4AAAD//gAAAAB//8AD///+AAAAP/gAAAAAf//AA////gAAAAAAAAAAAH//gAH///4AAAAAAAAAAAA//4AA///+AAAAAAAA' +
  'AAAAH/8AAH///gAAAAAAAAAAAA//AAA///4AAAAAAAAAAAAH/wAAH//+AAAAAAAAAAAAA/4AAA///gAAAAAAAAAAAAH+AAAH' +
  '//8AAAAAAAAAAAAAfgAAA///AAAAAAAAAAAAAAAAAAD//wAAAAAAAAAAAAAAAAAAf/8AAAAAAAAAAAAAAAAAAB/+AAAAAAAA' +
  'AAAAAAAAAAAH/AAAAAAAAAAAAAAAAAAAAHAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── bear — 비트맵 실루엣(128×128, 잉크 50.5%)
//   원본 마스크: bear.png
var MASK_BEAR = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAfwAAAAAP4AAAAAAAAAAAAf+AAAAAP/AAAAAAAAAAAAP/4AAAAH/8AAAAAAAAAAAH//Af/4D//gAAAAA' +
  'AAAAAD//x///5//4AAAAAAAAAAA//////////AAAAAAAAAAAP/////////wAAAAAAAAAAH/////////+AAAAAAAAAAB/////' +
  '/////gAAAAAAAAAAf/////////4AAAAAAAAAAH/////////+AAAAAAAAAAB//////////gAAAAAAAAAAP/////////wAAAAA' +
  'AAAAAD/////////8AAAAAAAAAAA//////////AAAAAAAAAAAH//3//////gAAAAAAAAAAA//4///x//wAAAAAAAAAAAP/8H/' +
  '/4P/8AAAAAAAAAAAD//B//+D//AAAAAAAAAAAA//wf//g//wAAAAAAAAAAAf/8H//4P/+AAAAAAAAAAAH//B//+D//gAAAAA' +
  'AAAAAB//w///w//4AAAAAAAAAAA//+P//8f//AAAAAAAAAAAP///+B////wAAAAAAAAAAD///wAA///8AAAAAAAAAAA///wA' +
  'AD///AAAAAAAAAAAP//wD/Af//wAAAAAAAAAAD//4D/8B//8AAAAAAAAAAA//+A//Af//gAAAAAAAAAAP//AP/wD//4AAAAA' +
  'AAAAAD//wD/8A//8AAAAAAAAAAA//8A//AP//AAAAAAAAAAAP//AH/gD//wAAAAAAAAAAD//wA/wA//8AAAAAAAAAAA//8AH' +
  '4AP//AAAAAAAAAAAP//AAcAD//wAAAAAAAAAAB//4AAAB//4AAAAAAAAAAAf/+AAAAf/+AAAAAAAAAAAH//4AAAf//gAAAAA' +
  'AAAAAA///wAA///wAAAAAAAAAAAP////////8AAAAAAAAAAAB////////+AAAAAAAAAAAA/////////wAAAAAAAAAAA/////' +
  '/////AAAAAAAAAAA//////////8AAAAAAAAAAf//////////gAAAAAAAAAf//////////+AAAAAAAAAP///////////wAAAA' +
  'AAAAH///////////+AAAAAAAAH////////////4AAAAAAAD/////////////AAAAAAAB/////////////4AAAAAAA///////' +
  '///////AAAAAAAf/////////////4AAAAAAP//////////////AAAAAAD//////////////wAAAAAB//////////////+AAA' +
  'AAA///////////////wAAAAAf//////////////+AAAAAH///////////////gAAAAD///////////////8AAAAA////////' +
  '////////gAAAAf///////////////4AAAAP////////////////AAAAD////////////////wAAAB///8////////8///+AA' +
  'AAf//+H///////+H///gAAAH///B////////g///4AAAD///wf///////4P///AAAA///4H///////+B///wAAAP//+B////' +
  '////gf//8AAAD///Af///////4D///AAAB///wH///////+A///wAAAf//4B////////gP//+AAAH//+Af///////4B///gA' +
  'AB///gP////////Af//4AAAf//4D////////wH//+AAAH//+A////////8A///gAAB///AP////////AP//wAAAf//wH////' +
  '////4D//8AAAD///B////////+D///AAAA///8f////////j///wAAAP////////////////4AAAB////////////////+AA' +
  'AAf////////////////AAAAH////////////////4AAAB////////////////+AAAA/////////////////wAAAP/g//////' +
  '//////h/8AAAH/gD///////////AH/gAAB/wAf//////////gA/4AAAf8AB//////////gAP+AAAP+AAf/////////4AB/wA' +
  'AD/gAD/////////8AAf8AAA/4AAf////////+AAH/AAAP8AAH/////////gAB/wAAD/AAA/////////wAAf8AAA/4AAP////' +
  '////8AAH/AAAP+AAB/////////AAB/wAAD/gAAf////////gAAf4AAAf4AAH////////4AAH+AAAH/AAB////////+AAD/gA' +
  'AB/wAAf////////gAA/4AAAP+AAH////////4AAf8AAAD/gAB////////+AAH/AAAAf8AA/////////wAD/wAAAH/gAP////' +
  '////8AB/4AAAA/8AH/////////gA/8AAAAH/wD/////////8Af/AAAAB//B//////////wf/gAAAAP///////////////wAA' +
  'AAB///////////////4AAAAAP///+f////+P///8AAAAAA///+AP///wB///+AAAAAAH///AAAAAAAP//+AAAAAAAf//AAAA' +
  'AAAA//+AAAAAAAA/8AAAAAAAAA/8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── rabbit — 비트맵 실루엣(128×128, 잉크 36.7%)
//   원본 마스크: rabbit.png
var MASK_RABBIT = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAPgAAAAAPwAAAAAAAAAAAAP+AAAAAP/AAAAAAAAAAAAH/4AAAAH/4AAAAAAAAAAAD//AAAAD//AAAAAA' +
  'AAAAAA//4AAAB//wAAAAAAAAAAAf//AAAA//+AAAAAAAAAAAH//wAAAf//gAAAAAAAAAAD//+AAAH//4AAAAAAAAAAA///gA' +
  'AD//+AAAAAAAAAAAP//8AAA///wAAAAAAAAAAD///AAAP//8AAAAAAAAAAA///4AAH///AAAAAAAAAAAP//+AAB///wAAAAA' +
  'AAAAAD///gAA///8AAAAAAAAAAA///8AAP///AAAAAAAAAAAP///AAD///gAAAAAAAAAAD///wAB///4AAAAAAAAAAA///+A' +
  'Af//+AAAAAAAAAAAH///gAH///gAAAAAAAAAAB///4AB///4AAAAAAAAAAAf///AA///8AAAAAAAAAAAH///wAP///AAAAAA' +
  'AAAAAA///8AD///wAAAAAAAAAAAP///AA///8AAAAAAAAAAAD///wAP//+AAAAAAAAAAAAf//+AH///gAAAAAAAAAAAH///g' +
  'B///4AAAAAAAAAAAB///4Af//8AAAAAAAAAAAAP//+AH///AAAAAAAAAAAAD///gB///wAAAAAAAAAAAA///4A///4AAAAAA' +
  'AAAAAAH///AP//+AAAAAAAAAAAAB///wD///AAAAAAAAAAAAAP//8A///wAAAAAAAAAAAAD///AP//4AAAAAAAAAAAAAf//w' +
  'D//+AAAAAAAAAAAAAH//8A///AAAAAAAAAAAAAA///////wAAAAAAAAAAAAAP//////4AAAAAAAAAAAAAB//////+AAAAAAA' +
  'AAAAAAAf//////AAAAAAAAAAAAAAD//////wAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB///' +
  '///gAAAAAAAAAAAAAA//////8AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAP//////8AAAAAAAAAAAAAH///////gAAAAAA' +
  'AAAAAAD///////8AAAAAAAAAAAAA////////AAAAAAAAAAAAAf///////4AAAAAAAAAAAAH///////+AAAAAAAAAAAAD////' +
  '////gAAAAAAAAAAAA//f///7/8AAAAAAAAAAAAP/B///4P/AAAAAAAAAAAAH/wf//+D/wAAAAAAAAAAAB/4D///Af+AAAAAA' +
  'AAAAAAf+A///wH/gAAAAAAAAAAAH/gP//8B/4AAAAAAAAAAAB/4D///Af+AAAAAAAAAAAAf+A///wH/gAAAAAAAAAAAH/gP/' +
  '/8B/4AAAAAAAAAAAB/4D///Af+AAAAAAAAAAAAf/B///4P/gAAAAAAAAAAAH/wf4H+D/4AAAAAAAAAAAB/+PwAPx/8AAAAAA' +
  'AAAAAAP//4AB///AAAAAAAAAAAAD//+AAf//wAAAAAAAAAAAA///gAH//8AAAAAAAAAAAAf//4AB///gAAAAAAAAAAAP//+A' +
  'Af//4AAAAAAAAAAAD///wAP///AAAAAAAAAAAB///8AD///4AAAAAAAAAAA////gB////AAAAAAAAAAAP///8A////wAAAAA' +
  'AAAAAH////gf///+AAAAAAAAAAB//////////gAAAAAAAAAA//////////8AAAAAAAAAAP//////////AAAAAAAAAAH/////' +
  '/////4AAAAAAAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAH//////////4AAAAAAAAAD///////////AAAAA' +
  'AAAAA///////////wAAAAAAAAAP//////////8AAAAAAAAAD///////////AAAAAAAAAA///////////wAAAAAAAAAP/////' +
  '/////8AAAAAAAAAD///////////AAAAAAAAAA///////////wAAAAAAAAAP//////////8AAAAAAAAAB//////////+AAAAA' +
  'AAAAAf//////////gAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAP/////' +
  '/////8AAAAAAAAAP///////////wAAAAAAAAH///////////+AAAAAAAAD////////////wAAAAAAAB////////////+AAAA' +
  'AAAAf////////////gAAAAAAAH////////////4AAAAAAAD/////////////AAAAAAAA/////////////wAAAAAAAP//////' +
  '//////8AAAAAAAD/////////////AAAAAAAAf////////////gAAAAAAAH////////////4AAAAAAAB////////////+AAAA' +
  'AAAAP////////////AAAAAAAAB////////////gAAAAAAAAP///////////wAAAAAAAAB///////////4AAAAAAAAAH//wf/' +
  '/+D//4AAAAAAAAAAP/gAf/4AH/wAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── turtle — 비트맵 실루엣(128×128, 잉크 40.9%)
//   원본 마스크: turtle.png
var MASK_TURTLE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAD/AAAAAAAAAAAAAAAAAAAD/8AAAAAAAAAAAAAAAAAAB//gAAAAAAAAAAAAAAAAAA//8AAAAAAAAA' +
  'AAAAAAAAAf//gAAAAAAAAAAAAAAAAAH//4AAAAAAAAAAAAAAAAAD///AAAAAAAAAAAAAAAAAA///wAAAAAAAAAAAAAAAAAf/' +
  '/+AAAAAAAAAAAAAAAAAH///gAAAAAAAAAAAAAAAAB///4AAAAAAAAAAD/AAAAAf//+AAAAA/wAAAH//AAAAH///gAAAD//gA' +
  'AD//+AAAB///4AAAH//8AAD///4AAAf//+AAAH///wAB////gAAH///gAAH///+AAf///+AAB///4AAD////gAP////wAAf/' +
  '/+AAD////8AD////+AAH///gAB/////AA/////4AA///wAB/////wAP/////AAP//8AA/////8AB/////4AH///gAf/////A' +
  'Af/////AP////AP/////gAD/////4P////8H/////wAA//////P/////7/////8AAD/////////////////+AAAf////////' +
  '////////+AAAD/////////////////AAAAP////////////////AAAAA////////////////AAAAAD///////////////AAA' +
  'AAAf////wH////////gAAAAAB////gAf///////gAAAAAAH///gAD///////wAAAAAAA///wAAf//////wAAAAAAAH//4AAD' +
  '//////4AAAAAAAAf/4AAA//////4AAAAAAAAD/8AAAH/////8AAAAAAAAA//AAAB//////AAAAAAAAAH/gAAAf/////gAAAA' +
  'AAAAD/wAAAH/////8AAAAAAAAA/4AAAB//////gAAAAAAAAf8AAAA//////4AAAAAAAAH/AAAAP//4D/+AAAAAAAAD/gAAAD' +
  '//4AP/wAAAAAAAA/4AAAB//+AB/8AAAAAAAAf8AAAAf//AAH/gAAAAAAAH/AAAAH//wAB/4AAAAAAAB/gAAAD//4AAP/AAAA' +
  'AAAA/4AAAB//+AAB/wAAAAAAAP+AAAAf//gAAf8AAAAAAAD/AAAAP//4AAD/AAAAAAAB/wAAAD//+AAA/4AAAAAAAf8AAAB/' +
  '//gAAH+AAAAAAAH/AAAA///8AAB/gAAAAAAB/wAAAf///AAAf4AAAAAAAf8AAAP///wAAH/AAAAAAAH/AAAH///+AAB/wAAA' +
  'AAAB/wAAD////gAAf8AAAAAAA/8AAD////4AAH/AAAAAAAP/gAB/////AAB/wAAAAAAD/4AB/////wAAf8AAAAAAA//AB///' +
  '//+AAP/AAAAAAAP/8B//////wAD/wAAAAAAD/////////8AB/8AAAAAAAf/////////gA//AAAAAAAH/////////+Af/wAAA' +
  'AAAB//////////4f/4AAAAAAAf////////////+AAAAAAAH//////z//////gAAAAAAB//////AH/////4AAAAAAAP/////g' +
  'Af////8AAAAAAAD/////gAD/////AAAAAAAA/////wAAf////wAAAAAAAP////4AAD////8AAAAAAAB////8AAA////+AAAA' +
  'AAAAf////AAAH////gAAAAAAAH////gAAA////4AAAAAAAA////wAAAP///8AAAAAAAAP///4AAAB////AAAAAAAAB///+AA' +
  'AAf///gAAAAAAAAf///AAAAD///4AAAAAAAAD///wAAAA///8AAAAAAAAA///4AAAAH///AAAAAAAAAf//+AAAAB///4AAAA' +
  'AAAAP///gAAAAf///AAAAAAAAP///wAAAAH///8AAAAAAAH///8AAAAB////gAAAAAAD////AAAAAf///8AAAAAAB////wAA' +
  'AAH////gAAAAAA////+AAAAB////8AAAAAAf////gAAAAf////gAAAAAP////8AAAAP////8AAAAAH/////AAAAH/////gAA' +
  'AAD/////4AAAD/////8AAAAA//////gAAB//////AAAAAf/////8AAA//////4AAAAP//////8AD///////AAAAD////////' +
  '////////wAAAB////5//////+f///+AAAAf///8P//////D////gAAAP////A//////A////8AAAD////gD/////gH////AA' +
  'AA////wAf////gA////4AAAf///4AB////gAH///+AAAH///8AAD///AAA////gAAB///+AAAH/+AAAH///4AAAf///AAAAD' +
  'wAAAA///+AAAH///gAAAAAAAAAH///gAAB///wAAAAAAAAAA///4AAAf//4AAAAAAAAAAH//+AAAH//8AAAAAAAAAAA///gA' +
  'AB//8AAAAAAAAAAAD//4AAAf/+AAAAAAAAAAAAf/+AAAD//AAAAAAAAAAAAB//AAAA//AAAAAAAAAAAAAP/wAAAH/AAAAAAA' +
  'AAAAAAA/4AAAA/AAAAAAAAAAAAAAB8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── whale — 비트맵 실루엣(128×128, 잉크 18.8%)
//   원본 마스크: whale.png
var MASK_WHALE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA+AAAAAAAAAAAAAAAAAAAA/' +
  'gAAAAAAAAAAAAAAAAAAAf4AAAAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAAH/AAAAAAAAAAAAAAAAAAAD/wAAAAAAAAAA' +
  'AAAAAAAAB/8AAAAAAAAAAAAAAAAAAAf+AAAAAAAAAAAAAAAAAAAP/gAAAAAAAAAAAAAAAAAAH/4AAAADAAAAAAAAAAAAAB/+' +
  'AAAAAwAAAAAAAAAAAAA//gAAAAeAAAAAAAAAAAAAP/4AAAAH4AAAAAAAAAAAAH/+AAAAB/gAAAAAAAAAAAB//gAAAAf+AAAA' +
  'AAAAAAAA//4AAAAH/wAAYAAAAAAAAf/+AAAAB//AB+AAAAAAAH///wAAAAf/5//gAAAAAB////+AAAAD////4AAAAAH/////' +
  '8AAAA////8AAAAAP//////4AAAH////AAAAAf///////gAAB////wAAAAf////////AAAP///4AAAAf////////8AAB///8A' +
  'AAAf/////////wAAP//+AAAAf//////////AAB///AAAAP//////////8AAP//gAAAH///////////wAD//AAAAD////////' +
  '////AB//AAAAB////////////8A//AAAAA//+A////////////wAAAAf/8AB///////////4AAAAP/8AAP//////////+AAA' +
  'AH/+AAD///////////AAAAB//AAB///////////wAAAA//gAA///////////4AAAAP/wAA///////////+AAAAD/8AA/////' +
  '///////AAAAB//wB////////////gAAAA///////////8f///4AAAAf//////////gAP//8AAAAH//////////AAAf/+AAAA' +
  'B//////////AAAD//AAAAAf/+AH/////AAAAf/gAAAAD/8AAP////gAAAH/wAAAAAfAAAAf///gAAAB/4AAAAAHwAAAB///w' +
  'AAAA/8AAAAAA8AAAAP//wAAAAf+AAAAAADwAAAD//4AAAAf+AAAAAAAfAAAAf/+AAAA//AAAAAAAD8AAAH//wAAA//AAAAAA' +
  'AAH+AAB//+AAH//AAAAAAAAAP/AAP//gA//+AAAAAAAAAAP//3//////4AAAAAAAAAAAP////////AAAAAAAAAAAAAA/////' +
  '/AAAAAAAAAAAAAAAAA///AAAAAAAAAAAAAAAAAAP//wAAAAAAAAAAAAAAAAAB//+AAAAAAAAAAAAAAAAAAP//gAAAAAAAAAA' +
  'AAAAAAAB//8AAAAAAAAAAAAAAAAAAP//gAAAAAAAAAAAAAAAAAB//4AAAAAAAAAAAAAAAAAAP//AAAAAAAAAAAAAAAAAAB//' +
  '4AAAAAAAAAAAAAAAAAAH//AAAAAAAAAAAAAAAAAAAf/wAAAAAAAAAAAAAAAAAAB/4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── horse — 비트맵 실루엣(128×128, 잉크 41.7%)
//   원본 마스크: horse.png
var MASK_HORSE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAADAAAAAAAAAAAAAAAAAAAABwAAAAAAAAAAAAAAAAAAAA8AAAAAAAAAAAAAAAAAAAAfgAMAAAAAA' +
  'AAAAAAAAAAAP4AHAAAAAAAAAAAAAAAAAD+AB4AAAAAAAAAAAAAAAAB/gA+AAAAAAAAAAAAAAAAA/8APgAAAAAAAAAAAAAAAA' +
  'P/AH4AAAAAAAAAAAAAAAAD/wD+AAAAAAAAAAAAAAAAB/8B/gAAAAAAAAAAAAAAAAf+A/4AAAAAAAAAAAAAAAAH/wf8AAAAAA' +
  'AAAAAAAAAAD////AAAAAAAAAAAAAAAf/////wAAAAAAAAAAAAAA//////8AAAAAAAAAAAAAD//////+AAAAAAAAAAAAAD///' +
  '////gAAAAAAAAAAAAB///////wAAAAAAAAAAAAB///////8AAAAAAAAAAAAA////////gAAAAAAAAAAAAf///////8AAAAAA' +
  'AAAAAAAP///////gAAAAAAAAAAAAH///////8AAAAAAAAAAAAH////////gAAAAAAAAAAAH////////4AAAAAAAAAAAH////' +
  '/////AAAAAAAAAAAH/////////4AAAAAAAAAAD/////////+AAAAAAAAAAB//////////wAAAAAAAAAA//////////8AAAAA' +
  'AAAAAf//////////AAAAAAAAAAP//////////wAAAAAAAAAH//////////8AAAAAAAAAD///8///////AAAAAAAAAB///+f/' +
  '/////wAAAAAAAAAf///n//////+AAAAAAAAAP///z///////gAAAAAAAAH///4///////8AAAAAAAAB///+f///////gAAAA' +
  'AAAAf///H///////4AAAAAAAAPn//j////////AAAAAAAADj//4////////wAAAAAAAAx//8P///////+AAAAAAAAA///D//' +
  '//////gAAAAAAAAf//h////////8AAAAAAAAP//4f////////AAAAAAAAD//8H////////4AAAAAAAB///B////////+AAAA' +
  'AAAA///wf////////wAAAAAAAf//4P////////+AAAAAAAH//+D/////////gAAAAAAD///g/////////8AAAAAAA///wP//' +
  '///////gAAAAAAf//8D/////////4AAAAAAH///A//////////AAAAAAB///wP///5/////wAAAAAA///4D///+D////8AAA' +
  'AAAP//+A////gP////gAAAAAD///gP///8A////4AAAAAA///4D////AD///8AAAAAAf//+Af///4Af///AAAAAAH///AH//' +
  '/+AD///wAAAAAB///wB////wAf//8AAAAAAfv/8Af///8AD//+AAAAAAHz//AH////gA///gAAAAAB8//wA////8AP//4AAA' +
  'AAAOf/8AP////AB//8AAAAAADn//AD////4Af//AAAAAAAx//wAf////AD//gAAAAAAM//8AH////4Af/wAAAAAAAP/+AA//' +
  '//+AB/gAAAAAAAD//gAP////wAAAAAAAAAAA//4AD////+AAAAAAAAAAAP/+AAf////wAAAAAAAAAAH//gAD////+AAAAAAA' +
  'AAAB//4AA/////wAAAAAAAAAAf/8AAH////+AAAAAAAAAAH//AAA/////wAAAAAAAAAB//wP/v////+AAAAAAAAAAf/4P///' +
  '////wAAAAAAAAAH/+P///////+AAAAAAAAAA//H////////wAAAAAAAAAP/n////////+AAAAAAAAAD/7/////////gAAAAA' +
  'AAAA///////////8AAAAAAAAAP///////////gAAAAAAAAB///////////4AAAAAAAAAf3//////////AAAAAAAAAH5/////' +
  '/////4AAAAAAAAA+//////////+AAAAAAAAAPP//////////wAAAAAAAAB3//////////8AAAAAAAAAZ///////////gAAAA' +
  'AAAAAf//////////4AAAAAAAAAP//////////+AAAAAAAAAD///////////wAAAAAAAAA///////////8AAAAAAAAAf/////' +
  '//////AAAAAAAAAH///////////4AAAAAAAAB///////////+AAAAAAAAAf///////////gAAAAAAAAH///////////4AAAA' +
  'AAAAB///////////+AAAAAAAAAf///////////gAAAAAAAAH///////////4AAAAAAAAB///////////+AAAAAAAAAf/////' +
  '//////gAAAAAAAAH///////////4AAAAAAAAB///////////+AAAAAAAAAP///////////AAAAAAAAAD///////////wAAAA' +
  'AAAAAP//////////gAAAAAAAAAAf/////////AAAAAAAAAAAA/////////AAAAAAAAAAAAB///////8AAAAAAAAAAAAAA///' +
  '///gAAAAAAAAAAAAAAAH///wAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── bird — 비트맵 실루엣(128×128, 잉크 36.1%)
//   원본 마스크: bird.png
var MASK_BIRD = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAB8AAAAAAAAAAAAAAAAAAAB/4AAAAAAAAAAAAAAAAAAA//AAAAAA' +
  'AAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAH//AAAAAAAAAAAAAAAAAAB//4AAAAAAAAAAAAAAAAAA///AAAAAAAAAAAAAAAA' +
  'AAP//8AAAAAAAAAAAAAAAAAD///gAAAAAAAAAAAAAAAAA///8AAAAAAAAAAAAAAAAAP///gAAAAAAAAAAAAAAAAD///8AAAA' +
  'AAAAAAAAAAAAA////wAAAAAAAAAAAAAAAAP///+AAAAAAAAAAAAAAAAD////4AAAAAAAAAAAAAAAAf////AAAAAAAAAAAAAA' +
  'AAH////8AAAAAAAAAAAOAAAB/////gAAAAAAAAAAPgAAAf////+AAAAAAAAAAH4AAAD/////wAAAAAAAAAD+AAAA//////AA' +
  'AAAAAAAA/gAAAH/////4AAAAAAAAAf4AAAB//////gAAAAAAAwH+AAAAP/////8AAAAAAAeD/AAAAB//////gAAAAAAH5/wA' +
  'AAAf/////8AAAAAAB//4AAAAH//////gAAAAAA//8AAAAD//////8AAAAAAP//AAAAH///////AAAAAAD//wAAAH///////4' +
  'AAAAAA//+AAAD///////+AAAAAAH//8AAA////////wAAAAAB///4AAf///////8AAAP8Af//+AAH////////gAAf/wD///A' +
  'AB////////4AAP/+Af//wAAf///////+AAH//wD//4AAH////////wAD//8A//4AAB////////8AB///gH/4AAAf////////' +
  'AA///4D/8AAAD////////wAf///A//AAAA////////8AH///4f/wAAAP////////AD////v/8AAAD////////4B///////gA' +
  'AAf///////+Af//////8AAAD////////gP//////+AAAA////////8H///////AAAAH////////D/////j/AAAAB////////' +
  '//////wAAAAAAP/////////////4AAAAAAB/////////////+AAAAAAAP/////////////AAAAAAAB/////////////wAAAA' +
  'AAAf////////////8AAAAAAAB/////////////AAAAAAAAP///////////jgAAAAAAAB///////////wYAAAAAAAAP//////' +
  '////8AAAAAAAAAB///////////AAAAAAAAAAH//////////wAAAAAAAAAA//////////8AAAAAAAAAAB//////////AAAAAA' +
  'AAAAAP/////////wAAAAAAAAAAAf////////8AAAAAAAAAAAD/////////AAAAAAAAAAAA/////////wAAAAAAAAAAAH////' +
  '////4AAAAAAAAAAAD////////+AAAAAAAAAAAB/////////gAAAAAABwAAD/////////4AAAAAAB//Af/////////+AAAAAA' +
  'A//////////////AAAAAAAf/////////////wAAAAAAH/////////////8AAAAAAB/////////////+AAAAAAAf/////////' +
  '////gAAAAAAH/////////////wAAAAAAB/////////////4AAAAAAAP////////////+AAAAAAAB/////////////AAAAAAA' +
  'Af////////////gAAAAAAAB////////////wAAAAAAAAf///////////4AAAAAAAAH///////////8AAAAAAAAB/////////' +
  '//+AAAAAAAAA////////////AAAAAAAAAP///////////AAAAAAAAAD///////////gAAAAAAAAA///////////AAAAAAAAA' +
  'AP/////wP///AAAAAAAAAAD/////4AH/8AAAAAAAAAAA/////8AAAAAAAAAAAAAAAH////+AAAAAAAAAAAAAAAA/////gAAA' +
  'AAAAAAAAAAAAH////wAAAAAAAAAAAAAAAAf///4AAAAAAAAAAAAAAAAB///+AAAAAAAAAAAAAAAAAf///AAAAAAAAAAAAAAA' +
  'AAD///gAAAAAAAAAAAAAAAAA///4AAAAAAAAAAAAAAAAAP//8AAAAAAAAAAAAAAAAAD///AAAAAAAAAAAAAAAAAAf//gAAAA' +
  'AAAAAAAAAAAAAD//wAAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAD/8AAAAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── deer — 비트맵 실루엣(128×128, 잉크 31.5%)
//   원본 마스크: deer.png
var MASK_DEER = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAD4AAAAAAAAAAAfAAAAAAAD/AAAAAAAAAAAP8AAAAAAB/4AAAAAAAAAAH/gAAAAAA//AAAAAAAAAAD/8AAA' +
  'AAAf/4AAAAAAAAAA//gAAAAAP/+AAAAAAAAAAf/8AAAAAD//gAAAAAAAAAH//AAAAAB//4AAAAAAAAAA//4AAAAAf/8AAAAA' +
  'AAAAAP/+AAAAAH//AAAAAAAAAAD//wAAAAD//gAAAAAAAAAAf/8AAAAA//4AAAAAAAAAAH//AAAAAP/8AAAAAAAAAAA//wAA' +
  'AAD//AAAAAAAAAAAP/8AAAAA//wAAAAAAAAAAD//AAAAAP/8AAAAAAAAAAA//wAAAAD//AAAAAAAAAAAP/8AAAAA//wAAAAA' +
  'AAAAAD//AAAAAP/+AAAAAAAAAAB//wAAAAD//wAAAAAAAAAA//8AAAAA///AAAAAAAAAA///AAAAAP///AAAAAAAAD///wAA' +
  'AAD///8AAAAAAAD///8AAAAA////wAAAAAAD////AAAAAH///+AAAAAAB////wAAAAB////wAAAAAA////4AAAAAf///+AAA' +
  'AAAf///+AAAAAD////wAAAAAP////AAAAAA////8AAAAAD////wAAAAAH////gAAAAB////4AAAAAA////4AAAAAf///+AAA' +
  'AAAP////AAAAAH////AAAAAAB////wAAAAD////gAAAAAAP///8AAAAA////wAAAAAAA////gAAAAf///wAAAAAAAH///4AA' +
  'AAH///4AAAAAAAAf///AAAAD///4AAAAAAAAB///wAAAA///4AAAAAAAAAH//+AAAAf//4AAAAAAAAAAP//wAAAP//wAAAAA' +
  'AAAAAA//+AAAD//wAAAAAAAAA/gD//wAAD//wA/AAAAAAA//gP/+AAB//4H/8AAAAAAf//B///////4P//gAAAAAD//8P///' +
  '///8P//wAAAAAA///x//////+P//8AAAAAAP//+P//////H///AAAAAAD///5//////n///wAAAAAAf///P/////z///4AAA' +
  'AAAH/////////////+AAAAAAA//////////////gAAAAAAP/////////////wAAAAAAB/////////////4AAAAAAAP//////' +
  '//////+AAAAAAAD/////////////AAAAAAAAf////////////gAAAAAAAD////////////wAAAAAAAAf///////////4AAAA' +
  'AAAAB///////////4AAAAAAAAAH//////////4AAAAAAAAAAP/////////wAAAAAAAAAAAB///////4AAAAAAAAAAAAAfh//' +
  '//h+AAAAAAAAAAAAAD4P///wfgAAAAAAAAAAAAA+B///4HwAAAAAAAAAAAAAPwP//+D8AAAAAAAAAAAAAD+D///B/AAAAAAA' +
  'AAAAAAAfw///w/gAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAA//////+AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAB///' +
  '///gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////4AAAAAAA' +
  'AAAAAAAH/////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB//g' +
  'B//gAAAAAAAAAAAAAAf/wAP/4AAAAAAAAAAAAAAH/8AD/+AAAAAAAAAAAAAAB//AA//gAAAAAAAAAAAAAAf/wAP/4AAAAAAA' +
  'AAAAAAAP/8AD//AAAAAAAAAAAAAAD//gB//wAAAAAAAAAAAAAA//4Af/8AAAAAAAAAAAAAAP//AP//gAAAAAAAAAAAAAH//4' +
  'H//4AAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAP//////8AAAAAAAAAAAAAD///////AAAAAAA' +
  'AAAAAAA///////4AAAAAAAAAAAAAf//////+AAAAAAAAAAAAAH///////gAAAAAAAAAAAAB///////4AAAAAAAAAAAAA////' +
  '////AAAAAAAAAAAAAP///////wAAAAAAAAAAAAD///////8AAAAAAAAAAAAA////////AAAAAAAAAAAAAP///////wAAAAAA' +
  'AAAAAAD///////8AAAAAAAAAAAAA////////AAAAAAAAAAAAAH///////gAAAAAAAAAAAAA///////wAAAAAAAAAAAAAH///' +
  '///4AAAAAAAAAAAAAA//////8AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAAf/////AAAAAAAAAAAAAAAD/////AAAAAAAA' +
  'AAAAAAAAf////gAAAAAAAAAAAAAAAB////gAAAAAAAAAAAAAAAAH///gAAAAAAAAAAAAAAAAAf//gAAAAAAAAAAAAAAAAAA/' +
  '/gAAAAAAAAAAAAAAAAAAB+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── hearingaid — 비트맵 실루엣(128×128, 잉크 46.0%)
//   원본 마스크: hearingaid.png
var MASK_HEARINGAID = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAf//wAAAAAAAAAAAAAAAAB////wAAAAAAAAAAAAAAAD/////gAAAAAAAAAAAAAAH/////+AAAAAAA' +
  'AAAAAAAH//////4AAAAAAAAAAAAAP///////gAAAAAAAAAAAAH///////+AAAAAAAAAAAAH////////4AAAAAAAAAAAH////' +
  '/////AAAAAAAAAAAD/////////4AAAAAAAAAAD//////////gAAAAAAAAAB//////////8AAAAAAAAAA///////////gAAAA' +
  'AAAAAf//////////8AAAAAAAAAf///////////gAAAAAAAAP///////////4AAAAAAAAH////////////AAAAAAAAD//////' +
  '//////4AAAAAAAB/////////////AAAAAAAA///////wAH///wAAAAAAAP//////wAAf//+AAAAAAAH//////wAAB///gAAA' +
  'AAAD//////4AAAH//8AAAAAAB//////4AAAA///AAAAAAA//////8AAAAH//4AAAAAAP/////+AAAAA//+AAAAAAH//////A' +
  'AAAAH//wAAAAAD//////wAAAAB//8AAAAAA//////4AAAAAP//AAAAAAf/////+AAAAAB//4AAAAAP//////AAAAAAf/+AAA' +
  'AAD//////gAAAAAH//gAAAAB//////4AAAAAA//8AAAAAf/////+AAAAAAP//AAAAAP//////AAAAAAB//wAAAAD//////wA' +
  'AAAAAf/8AAAAB//////8AAAAAAH//gAAAAf//////AAAAAAA//4AAAAP//////gAAAAAAP/+AAAAD//////4AAAAAAD//gAA' +
  'AA//////+AAAAAAA//4AAAAf//////gAAAAAAP/+AAAAH//////4AAAAAAD//gAAAD//////+AAAAAAAf/4AAAA///////gA' +
  'AAAAAH/+AAAAP//////4AAAAAAB//gAAAH//////+AAAAAAAf/4AAAB///////gAAAAAAH/+AAAAf//////4AAAAAAB//gAA' +
  'AH//////+AAAAAAAf/4AAAD///////gAAAAAAH/+AAAA///////4AAAAAAB//gAAAP//////+AAAAAAAf/4AAAD///////gA' +
  'AAAAAP/+AAAB///////4AAAAAAD//gAAAf//////+AAAAAAA//wAAAH///////gAAAAAAP/8AAAB///////4AAAAAAD//AAA' +
  'Af//////+AAAAAAA//wAAAH///////wAAAAAAP/8AAAD///////8AAAAAAD//AAAA////////AAAAAAB//wAAAP///////wA' +
  'AAAAAf/+AAAD///////8AAAAAAP//gAAA////////AAAAAAH//8AAAP///////wAAAAAB///gAAD///////+AAAAAA///4AA' +
  'A////////gAAAAAP///AAAP///////4AAAAAD///wAAD///////+AAAAAB///8AAA////////gAAAAAf///AAAP///////4A' +
  'AAAAP///wAAD///////+AAAAAH///8AAA////////wAAAAD////AAAP///////8AAAAD////wAAD////////AAAAB////+AA' +
  'A////////wAAAA/////gAAP///////8AAAAf////8AAD////////gAAAH/////gAA////////4AAAD/////+AAP///////+A' +
  'AAB//////wAD////////gAAAf/////8AA////////4AAAH//////gAP///////+AAAD//////4AB////////wAAA///////A' +
  'Af///////8AAAP//////wAH////////AAAD//////8AB////////wAAA///////AAf///////8AAAP//////wAH////////A' +
  'AAD//////8AA////////wAAA///////AAP///////8AAAP//////wAD////////AAAD//////8AA////////wAAA//////+A' +
  'AP///////8AAAH//////gAB////////AAAB//////wAAf///////wAAAf/////8AAH///////8AAAD/////+AAB////////A' +
  'AAAf/////AAAP///////wAAAH/////gAAD///////8AAAA/////gAAA////////AAAAH////gAAAH///////wAAAA////gAA' +
  'AB///////8AAAAH///gAAAAf//////+AAAAAf//AAAAAD///////gAAAAA/8AAAAAA///////4AAAAAAAAAAAAAH//////8A' +
  'AAAAAAAAAAAAB///////AAAAAAAAAAAAAAP//////gAAAAAAAAAAAAAB//////wAAAAAAAAAAAAAAf/////4AAAAAAAAAAAA' +
  'AAB/////8AAAAAAAAAAAAAAAP////+AAAAAAAAAAAAAAAB/////AAAAAAAAAAAAAAAAH///+AAAAAAAAAAAAAAAAAP//+AAA' +
  'AAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── tooth — 비트맵 실루엣(128×128, 잉크 56.4%)
//   원본 마스크: tooth.png
var MASK_TOOTH = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAB/+AAAAAAA//AAAAAAAAAD//8AAAAAB//+AAAAAAAAH///4AAAAD///4AAAAAAAD////gAAAD////gAAA' +
  'AAAD////+AAAD////8AAAAAAB/////4AAD/////wAAAAAA//////gAD/////+AAAAAAf//////AH//////wAAAAAP///////' +
  '///////+AAAAAH///////////////gAAAAD///////////////8AAAAB////////////////gAAAAf///////////////4AA' +
  'AAP////////////////AAAAD////////////////wAAAB////////////////+AAAAf////////////////gAAAH////////' +
  '////////8AAAD/////////////////AAAA/////////////////wAAAP////////////////+AAAH/////////////////gA' +
  'AB/////////////////4AAAf////////////////+AAAH/////////////////gAAB/////////////////4AAAf////////' +
  '////////+AAAP/////////////////wAAD/////////////////8AAA//////////////////AAAP/////////////////wA' +
  'AB/////////////////4AAAf////////////////+AAAH/////////////////gAAB/////////////////4AAAf////////' +
  '////////+AAAH/////////////////gAAA/////////////////4AAAP////////////////8AAAD/////////////////AA' +
  'AA/////////////////wAAAH////////////////4AAAB////////////////+AAAAP////////////////gAAAD////////' +
  '////////wAAAA////////////////8AAAAH///////////////+AAAAB////////////////gAAAAP///////////////wAA' +
  'AAB///////////////8AAAAAf//////////////+AAAAAD///////////////AAAAAA///////////////wAAAAAH///////' +
  '///////4AAAAAA//////////////+AAAAAAP//////////////AAAAAAB//////////////gAAAAAAf/////////////4AAA' +
  'AAAD/////////////8AAAAAAA//////////////AAAAAAAH/////////////wAAAAAAB/////////////4AAAAAAAf//////' +
  '//////+AAAAAAAH/////////////gAAAAAAA/////////////4AAAAAAAP////////////8AAAAAAAD/////////////AAAA' +
  'AAAA/////////////wAAAAAAAP////////////8AAAAAAAD/////////////AAAAAAAA/////////////wAAAAAAAP//////' +
  '//////8AAAAAAAD/////////////AAAAAAAAf////////////wAAAAAAAH////////////4AAAAAAAB////////////+AAAA' +
  'AAAAf////////////gAAAAAAAH////////////4AAAAAAAB////////////+AAAAAAAAf////////////gAAAAAAAH//////' +
  '//////4AAAAAAAB////////////+AAAAAAAAf////////////gAAAAAAAH////////////4AAAAAAAB////////////+AAAA' +
  'AAAAP////////////gAAAAAAAD/////+f/////4AAAAAAAA/////+B/////+AAAAAAAAP/////AP/////AAAAAAAAD/////g' +
  'B/////wAAAAAAAA/////wAP////8AAAAAAAAP////8AD/////AAAAAAAAB////+AAf////gAAAAAAAAf////gAH////4AAAA' +
  'AAAAH////wAA////+AAAAAAAAB////8AAP////gAAAAAAAAP////AAD////wAAAAAAAAD////gAAf///8AAAAAAAAA////4A' +
  'AH////AAAAAAAAAP///+AAB////wAAAAAAAAB////gAAf///4AAAAAAAAAf///wAAD///+AAAAAAAAAD///8AAA////AAAAA' +
  'AAAAA////AAAP///wAAAAAAAAAP///wAAB///8AAAAAAAAAB///8AAAf//+AAAAAAAAAAf//+AAAH///gAAAAAAAAAD///gA' +
  'AB///wAAAAAAAAAA///4AAAf//8AAAAAAAAAAH//8AAAD//+AAAAAAAAAAB///AAAA///gAAAAAAAAAAP//wAAAH//wAAAAA' +
  'AAAAAB//4AAAB//4AAAAAAAAAAAP/+AAAAf/+AAAAAAAAAAAB//AAAAD//AAAAAAAAAAAAP/gAAAAf/AAAAAAAAAAAAB/wAA' +
  'AAD/gAAAAAAAAAAAAH4AAAAAfgAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── mouth — 비트맵 실루엣(128×128, 잉크 34.1%)
//   원본 마스크: mouth.png
var MASK_MOUTH = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAD/wAAAP/AAAAAAAAAAAAAH//gAAf/+AAAAAAAAAAAAH//+A' +
  'Af//4AAAAAAAAAAAH///wAP///gAAAAAAAAAAD////AH///8AAAAAAAAAAB////4H////gAAAAAAAAAB//////////+AAAAA' +
  'AAAAA///////////wAAAAAAAAAf//////////+AAAAAAAAAP///////////wAAAAAAAAH///////////+AAAAAAAAD//////' +
  '//////wAAAAAAAA////////////8AAAAAAAAf////////////gAAAAAAAP////////////8AAAAAAAH/////////////gAAA' +
  'AAAD/////////////8AAAAAAB//////////////gAAAAAA//////////////8AAAAAAf//////////////gAAAAAP///////' +
  '///////8AAAAAH///////////////gAAAAD///////////////8AAAAB////////////////gAAAA////////////////8AA' +
  'AAf////////////////gAAAP////////////////8AAAP/////////////////wAAH/////////////////+AAH/////////' +
  '/////////4AH///////////////////gB///////////////////4AH//////////////////4AA//////////////////8A' +
  'AH/////////////////+AAA//////////////////AAAH/////////////////wAAA/////////////////4AAAP////////' +
  '////////8AAAB////////////////+AAAAP////////////////AAAAB////////////////gAAAAf///////////////4AA' +
  'AAD///////////////8AAAAAf//////////////+AAAAAD///////////////AAAAAAf//////////////wAAAAAH///////' +
  '///////4AAAAAA//////////////8AAAAAAH/////////////+AAAAAAA//////////////AAAAAAAH/////////////wAAA' +
  'AAAA/////////////4AAAAAAAH////////////8AAAAAAAA////////////+AAAAAAAAH////////////AAAAAAAAA//////' +
  '//////gAAAAAAAAH///////////gAAAAAAAAA///////////wAAAAAAAAAH//////////4AAAAAAAAAAf/////////8AAAAA' +
  'AAAAAD/////////8AAAAAAAAAAAP////////+AAAAAAAAAAAB////////+AAAAAAAAAAAAH///////+AAAAAAAAAAAAAf///' +
  '///+AAAAAAAAAAAAAA//////+AAAAAAAAAAAAAAB/////8AAAAAAAAAAAAAAAD////wAAAAAAAAAAAAAAAAB//+AAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── lungs — 비트맵 실루엣(128×128, 잉크 43.1%)
//   원본 마스크: lungs.png
var MASK_LUNGS = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAD/AAAAAAAAAAAAAAAAAAAD/8AAAAAAAAAAAAAAAAAAB//AAAAAAAAAAAAAAAAAAAf/4AAAAAAAAA' +
  'AAAAAAAAAH/+AAAAAAAAAAAAAAAAAAB//gAAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAH/+AAAAAAAAAAAAAAAAAAB/' +
  '/gAAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAH/+AAAAAAAAAAAAAAAAAAB//gAAAAAAAAAAAAAAAAAAf/4AAAAAAAAA' +
  'AAAAAAAAAH/+AAAAAAAAAAAAAAAAAAB//gAAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAH/+AAAAAAAAAAAAAAAAAAB/' +
  '/gAAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAH/+AAAAAAAAAAAAAAAAAAB//gAAAAAAAAAAAAAAAAAAf/4AAAAAAAAA' +
  'AAAAAAAAAH/+AAAAAAAAAAAAAAAAgAB//gABAAAAAAAAAAAAH/gAf/4AH/gAAAAAAAAAAH/8AH/+AH/+AAAAAAAAAAH//wB/' +
  '/gD//4AAAAAAAAAH//+Af/4B///gAAAAAAAAD///wH/+A///8AAAAAAAAB///8B//gP///gAAAAAAAA////gf/4H///8AAAA' +
  'AAAAf///4H/+B////gAAAAAAAP///+B//g////8AAAAAAAH////wf/4P////gAAAAAAD////8H/+D////8AAAAAAB/////B/' +
  '/h/////gAAAAAA/////4f/4f////8AAAAAAP////+H/+H/////AAAAAAH/////h//h/////4AAAAAD/////4f/4f/////AAA' +
  'AAA/////+H/+H/////wAAAAAf/////h//h/////+AAAAAP/////4f/4f/////gAAAAD/////+H/+H/////8AAAAB//////h/' +
  '/h//////AAAAAf/////wf/4f/////4AAAAP/////8P/+D/////+AAAAD//////D//w//////wAAAB//////h//8H/////8AA' +
  'AAf/////wf//h//////gAAAH/////8P//8P/////4AAAD/+H//+H///h///h//AAAA//Af//D///8P//gP/wAAAf/gD//h//' +
  '//h//wB/+AAAH/4Af/w////8P/4Af/gAAB/8AD/4f////h/8AH/4AAA//AAf4P////8H+AB//AAAP/4ABwH/////geAAf/wA' +
  'AD/+AAAD/////4AAAH/8AAA//wAAA//4P//AAAD//gAAf/8AAAf/8A//4AAA//4AAH//gAAP/+AH//AAAf/+AAD//+AAD//A' +
  'A//4AAf//gAA///4AB//gAH/+AAf//8AAP////g//wAA//wf////AAD////4P/4AAH/8H////wAB////8H/8AAA//h////8A' +
  'Af////B/+AAAH/4P////gAH////g//gAAB//B////4AB////wP/wAAAP/wP///+AAf///4H/8AAAD/8B////gAP///8B/+AA' +
  'AAf/gP///4AD///+Af/gAAAH/4B////AA////AH/4AAAB/+AP///wAP///AD/+AAAAf/wA///8AD///AA//gAAAP/8AH///A' +
  'A///gAP/8AAAD//AAf//wAP//wAD//AAAA//wAD//8AH//4AB//wAAAP/+AAf//AB//8AAf/8AAAH//gAD//wAf/+AAH//AA' +
  'AB//4AAf/+AH//gAB//4AAAf/+AAH//gB//wAA//+AAAH//wAA//4Af/8AAP//gAAB//8AAP/+AH//AAH//4AAAf//gAD//g' +
  'B//gAB//+AAAH//4AA//4Af/4AA///gAAB///AAP/+AH/+AAP//4AAAf//wAD//gB//gAH//+AAAH//+AA//4Af/8AD///gA' +
  'AB///wAP/+AH//AA///4AAAf//8AD//gB//4A///+AAAH///wB//4Af/+Af///gAAB///+Af/+AH//4f///wAAAf///4f//g' +
  'B///////8AAAD///////4Af///////AAAA///////+AH///////wAAAP///////gB///////4AAAB///////4Af//////+AA' +
  'AAf//////+AH///////AAAAH///////AB///////wAAAA///////wAP//////4AAAAH//////8AD//////+AAAAB///////A' +
  'A///////AAAAAP//////wAP//////gAAAAB//////8AD//////wAAAAAP/////+AAf/////4AAAAAB//////gAH/////8AAA' +
  'AAAP/////4AB/////8AAAAAAA/////+AAf////8AAAAAAAD/////AAD////wAAAAAAAAH////wAA////gAAAAAAAAAH///8A' +
  'AH///AAAAAAAAAAAP//+AAA///AAAAAAAAAAAA///AAAP//AAAAAAAAAAAAD//wAAB//AAAAAAAAAAAAAP/4AAAP/AAAAAAA' +
  'AAAAAAA/4AAAAeAAAAAAAAAAAAAAB4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── bone — 비트맵 실루엣(128×128, 잉크 40.8%)
//   원본 마스크: bone.png
var MASK_BONE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAD/4AAAAAAAAAAD/4AAAAAH//gAAAAAAAAAH//wAA' +
  'AAH//+AAAAAAAAAH///AAAAH///4AAAAAAAAH///4AAAD////AAAAAAAAD////AAAB////4AAAAAAAB////8AAA/////AAAA' +
  'AAAA/////AAAf////4AAAAAAAf////4AAP/////AAAAAAAP/////AAD/////4AAAAAAD/////4AB/////+AAAAAAB/////+A' +
  'Af/////wAAAAAAf/////gAP/////8AAAAAAP/////8AD//////gAAAAAD//////AA//////4AAAAAB//////wAf/////+AAA' +
  'AAAf/////+AH//////wAAAAAP//////gB//////8AAAAAD//////4Af//////gAAAAB//////+AH//////8AAAAA///////g' +
  'B///////gAAAAf//////4Af//////+AAAAf//////+AH///////////////////gA///////////////////4AP/////////' +
  '/////////8AD///////////////////AAf//////////////////wAH//////////////////4AA//////////////////+A' +
  'AP//////////////////AAB//////////////////gAAP/////////////////wAAB/////////////////4AAAP////////' +
  '////////8AAAB////////////////+AAAAf////////////////gAAAD////////////////wAAAA////////////////8AA' +
  'AAH////////////////AAAAB////////////////wAAAA////////////////8AAAAP////////////////AAAAH////////' +
  '////////4AAAD/////////////////AAAB/////////////////4AAA//////////////////AAAf/////////////////4A' +
  'AH//////////////////AAD//////////////////wAB//////////////////+AAf//////////////////gAP/////////' +
  '/////////8AD///////////////////AA///////////////////wAf//////////////////+AH///////////////////g' +
  'B///////4AAAB///////4Af//////4AAAAH//////+AH//////8AAAAAf//////gB//////+AAAAAD//////4Af//////AAA' +
  'AAA//////+AH//////wAAAAAH//////gB//////4AAAAAB//////4AP/////+AAAAAAf/////8AD//////gAAAAAD//////A' +
  'A//////wAAAAAA//////wAH/////8AAAAAAH/////4AB/////+AAAAAAB/////+AAP/////gAAAAAAP/////AAD/////wAAA' +
  'AAAD/////wAAf////4AAAAAAAf////4AAD////8AAAAAAAD////8AAAf///+AAAAAAAAf///+AAAD////AAAAAAAAD////AA' +
  'AAf///gAAAAAAAAf///gAAAB///wAAAAAAAAB///wAAAAH//wAAAAAAAAAP//wAAAAAf/gAAAAAAAAAAf/gAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── arena — 비트맵 실루엣(128×128, 잉크 31.1%)
//   원본 마스크: arena.png
var MASK_ARENA = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAADAAAAAAAOAAAAAAAAAAAAA4AAA' +
  'AAADgAAAAAAAAAAAAOAAAAAAA4AAAAAAAAAAAAHgAAAAAAeAAAAAAAAAAAAB8AAAAAAHwAAAAAAAAAAAA/gAAAAAD+AAAAAA' +
  'AAAAAAf4AAAAAB/gAAAAAAAAHAAP/AAAAAA/8AAYAAAAABwAGfYAAAAAZ5gAOAAAAAAcADHjgAAAAceMADgAAAAAHABh4cAA' +
  'AAOHhgA4AAAAAB4AweDwAAAPB4MAfAAAAAA/AYHx/////4eBwPwAAAAAP4eB////////wOH8AAAAAH/v3/////////n3/gAA' +
  'AAB+f////////////j4AAAAA/n////////////4/AAAAAP5////////////+fwAAAAH///////////////+AAAABP///////' +
  '///////4gAAAAx//////////////+MAAAAYf//////////////hgAAAGH//////////////4YAAADB//////////////+DAA' +
  'AAg///////////////wQAAAY/////+AAAAf/////GAAAHf////gAAAAAH////7gAAD////+AAAAAAAH////8AAB////+AAAA' +
  'AAAAf////AAAf///+AAAAAAAAB////4AAP////AAAAAAAAAP////AAD////gAAAAAAAAB////wAB////4AAAAAAAAAf///+A' +
  'Af////AAAAAAAAAP////gAP////4AAAAAAAAH////8AH/////gAAAAAAAH/////gB//////wAAAAAA//////4Af//////+AA' +
  'AAf//////+AD///////////////////AA///////////////////wAP//////////////////8AB//////////////////+A' +
  'Af//////////////////gAD//////////////////wAAf/////////////////4AAH/////////////////8AAA/////////' +
  '/////////AAAH/////////////////gAAB/////////////////4AAAP////////////////8AAAD/////////////////AA' +
  'AA/////////////////wAAAP////////////////8AAAD/////////////////AAAA/////////////////wAAAP////////' +
  '////////8AAAH//////8Pnw///////gAAA///////D58P//////wAAAP//////w//D//////8AAAA//////8f/4//////8AA' +
  'AAH//////H/+P/////+AAAAAf/////x//j/////+AAAAAA/////8f/4/////8AAAAAAB/////H/+P////4AAAAAAAD////x/' +
  '/j////wAAAAAAAAB///8//8///+AAAAAAAAAAA/////////wAAAAAAAAAAAAD//////wAAAAAAAAAAAAAAAAP/wAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── pestbug — 비트맵 실루엣(128×128, 잉크 42.4%)
//   원본 마스크: pestbug.png
var MASK_PESTBUG = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAwAAAAAAAYAAAAAAAAAAAA/AAAAAAAfAAAAAAAAAAAAP8AAAAAAf4AAAAAAAAAAAD/gAAAAAf8AAAAAA' +
  'AAAAAA/+AAAAAP/AAAAAAAAAIAAD/wAAAAH/AAAAAAAAAHAAAP+AAAAH/gAAAAAAAAD4AAB/wAAAD/gAAAAAAAAB/AAAH+AA' +
  'AB/gAAAAAAAAA/4AAA/wAAAfwAAAAAAAAAf/AAAH8AAAP4AAAAAAAAAP/4AAA/gAAH8AAAAAAAAAH//AAAH8AAD+AAAAAAAA' +
  'AD//4AAA/gAA/AAAAAAAAAB///AAAH4AAfgAAAAAAAAA///4AAA/AAH4AAAAAAAAAP///AAAP4ED8AAAAAAAAAB///4AAB//' +
  '//AAAAAAAAAAP///AAAP///gAAAAAAAAAB///4AAD///4AAAAAAAAAAP///AAA///+AAAAAAAAAAB///4AAP///gAAAAAAAA' +
  'AAP///AAH///8AAAAAAAAAAB///4AB////AAAAAAAAAAAP///AA////4AAAAAAAAAAB///4Af////AAAAAAAAAAAP///AH//' +
  '//wAAAYAAAAAAB///4D////+AAAPAAAAAAAP///A/////gAAH4AAAAAAB///4P////4AAB+AAAAAAAP///H////+AAAfgAAA' +
  'AAAB/////////gAAPwAAAAAAAP////////4AAD8AAAAAAAB////////+AAA/AAAAAAAAP////////gAAPwAAAAAAAB//////' +
  '//4AAH8AAAAAAAAP///////+AAB+AAAAAAAAB////////wAAfgAAAAAAHAP///////+AAH4AAAAAAD4B////////wAB+AAAA' +
  'AAB+AP///////+AB/AAAAAAAfwB////////wA/wAAAAAAD8AP///////+B/8AAAAAAA/AB////////x/+AAAAAAAP4AP////' +
  '//////AAAAAAAB+AB//////////gAAAAAAAfgAP/////////gAAAAAAAH8AB/////////gAAAAAAAA/AAf////////gAAAAA' +
  'AAAP+AH////////gAAAAAAAAD/4B////////4AAAAAAAAAf/x////////+AAAAAAAAAH///////////gAAAAAAAAA///////' +
  '////4AAAAAAAAAD//////////+AAAAAAAAAAH//////////wAAAAAAAAAAP/////////8AAAAAAAAAAAf/////////AAAAAA' +
  'AAAAAD/////////4AAAAAAAAAAA//////////AAAAAAAAAAAP///////////wAAAAAAAAD////////////AAAAAAAAB/////' +
  '///////4AAAAAAAA/////////////AAAAAAAA/////////////wAAAAAH//////////////+AAAAAD////////////4A/wAA' +
  'AAB////////////+AH+AAAAA/////////////AA/wAAAAf////////////wAH+AAAAP////////////8AA/wAAAH/AD/////' +
  '/////AAH+AAAD/gA//////////wAA/wAAB/wAH/////////8AAH8AAA/4AB/////////8AAB/gAAf4AAf////////+AAAP4A' +
  'AP8AAH/////////gAAB8AAD+AAB/////////4AAAOAAA/AAAf/////////AAAAAAAPgAAH/////////4AAAAAABwAAB/////' +
  '/////AAAAAAAAAAAf/////////4AAAAAAAAAAP//////////AAAAAAAAAAP//////////4AAAAAAAAAH///////////AAAAA' +
  'AAAAH///////////4AAAAAAAAH////////////AAAAAAAAD////////////4AAAAAAAB/////////////gAAAAAAA/8f////' +
  '//////4AAAAAAAP8D///////////AAAAAAAD+A///////////4AAAAAAA/gP///////////AAAAAAAPwB///////////4AAA' +
  'AAAD8Af///////////AAAAAAA/AH///////////4AAAAAAPwA////////////AAAAAAD8AP///////////4AAAAAA/AB////' +
  '////////AAAAAAfwAf///////h///4AAAAAH8AD///////wP///AAAAAB/AA///////8B///4AAAAAfwAH//////+AP///AA' +
  'AAAH4AA///////gB///4AAAAB+AAP//////wAP///AAAAAfgAB//////4AB///4AAAAH4AAP/////8AAP///AAAAB+AAD///' +
  '///AAB///wAAAAfgAAf/////gAAP//4AAAAH4AAD/////wAAB//8AAAAD+AAAf////4AAAP/+AAAAA/gAAD////8AAAB//gA' +
  'AAAfwAAAf///+AAAAP/wAAAAH4AAAD////AAAAB/4AAAAA+AAAAP///AAAAAP8AAAAAHAAAAA///gAAAAB+AAAAAAAAAAAD/' +
  '/gAAAAAOAAAAAAAAAAAAP/AAAAAABAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── finger — 비트맵 실루엣(128×128, 잉크 27.4%)
//   원본 마스크: finger.png
var MASK_FINGER = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAB+AAAAAAAAAAAAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAf+AAAAAAAAAAAAAAAAAAAP/gAAAAAAA' +
  'AAAAAAAAAAAD/8AAAAAAAAAAAAAAAAAAA//AAAAAAAAAAAAAAAAAAAP/wAAAAAAAAAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAA' +
  'B//AAAAAAAAAAAAAAAAAAAf/wAAAAAAAAAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAAB//AAAAAAAAAAAAAAAAAAAf/wAAAAAAA' +
  'AAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAAB//AAAAAAAAAAAAAAAAAAAf/wAAAAAAAAAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAA' +
  'B//AAAAAAAAAAAAAAAAAAAf/wAAAAAAAAAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAAB//AAAAAAAAAAAAAAAAAAAf/wAAAAAAA' +
  'AAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAAB//AAAAAAAAAAAAAAAAAAAf/wAAAAAAAAAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAA' +
  'B//AAAAAAAAAAAAAAAAAAAf/wAAAAAAAAAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAAB//AAAAAAAAAAAAAAAAAAAf/wAAAAAAA' +
  'AAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAAB//AAAAAAAAAAAAAAAAAAAf/wAAAAAAAAAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAA' +
  'B//AAAAAAAAAAAAAAAAAAgf/wAAAAAAAAAAAAAAAAB/H/8AAAAAAAAAAAAAAAAB/5//AAAAAAAAAAAAAAAHgf///wAAAAAAA' +
  'AAAAAAAD+P///8AAAAAAAAAAAAAAB/z////AAAAAAAAAAAAAAA//////wAAAAAAAAAAAAB4f/////8AAAAAAAAAAAAB/H///' +
  '///AAAAAAAAAAAAA////////wAAAAAAAAAAAAf///////8AAAAAAAAAAAAH////////AAAAAAAAAAAAB////////wAAAAAAA' +
  'AAAAAf///////8AAAAAAAAAAAAH////////AAAAAAAAAAAAB////////wAAAAAAAAAAAAf///////8D/AAAAAAAAAAH/////' +
  '///B/4AAAAAAAAAB////////w//AAAAAAAAAAf///////+//wAAAAAAAAAH//////////8AAAAAAAAAB///////////AAAAA' +
  'AAAAAf//////////wAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAAAAAAAf//////////AAAAAAAAAAH/////' +
  '/////gAAAAAAAAAB//////////wAAAAAAAAAAf/////////4AAAAAAAAAAH/////////+AAAAAAAAAAB//////////AAAAAA' +
  'AAAAA//////////wAAAAAAAAAAP/////////4AAAAAAAAAAD/////////+AAAAAAAAAAA//////////AAAAAAAAAAAP/////' +
  '////wAAAAAAAAAAD/////////8AAAAAAAAAAA/////////+AAAAAAAAAAAP/////////gAAAAAAAAAAD/////////4AAAAAA' +
  'AAAAA/////////+AAAAAAAAAAAP/////////gAAAAAAAAAAD/////////4AAAAAAAAAAA/////////8AAAAAAAAAAAH/////' +
  '////AAAAAAAAAAAB/////////wAAAAAAAAAAAf////////4AAAAAAAAAAAH////////+AAAAAAAAAAAB/////////gAAAAAA' +
  'AAAAAf////////wAAAAAAAAAAAD////////8AAAAAAAAAAAA////////+AAAAAAAAAAAAP////////gAAAAAAAAAAAB/////' +
  '///wAAAAAAAAAAAAf///////4AAAAAAAAAAAAH///////+AAAAAAAAAAAAA////////AAAAAAAAAAAAAP///////gAAAAAAA' +
  'AAAAAB///////4AAAAAAAAAAAAAf//////8AAAAAAAAAAAAAD//////+AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAH////' +
  '//wAAAAAAAAAAAAAA//////8AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAD//////gAAAAAAAAAAAAAA//////4AAAAAAAA' +
  'AAAAAAH/////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB////' +
  '//gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////4AAAAAAAA' +
  'AAAAAAH/////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAP/////wAAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAAP///' +
  '/+AAAAAAAAAAAAAAAA////+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── like — 비트맵 실루엣(128×128, 잉크 44.4%)
//   원본 마스크: like.png
var MASK_LIKE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAB+AAAAAAAAAAAAAAAAAAAA/4AAAAAAAAAAAAAAAAAAAf/AAAAAAAAAAAAAAAAAAAH/4AAAAAAAA' +
  'AAAAAAAAAAB//AAAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAH/+AAAAAAAAAAAAAAAAAAB//wAAAAAAAAAAAAAAAAAA' +
  'f/8AAAAAAAAAAAAAAAAAAH//gAAAAAAAAAAAAAAAAAB//4AAAAAAAAAAAAAAAAAAf//AAAAAAAAAAAAAAAAAAH//wAAAAAAA' +
  'AAAAAAAAAAD//8AAAAAAAAAAAAAAAAAA///AAAAAAAAAAAAAAAAAAP//4AAAAAAAAAAAAAAAAAD//+AAAAAAAAAAAAAAAAAA' +
  '///gAAAAAAAAAAAAAAAAAP//4AAAAAAAAAAAAAAAAAH//+AAAAAAAAAAAAAAAAAB///gAAAAAAAAAAAAAAAAAf//4AAAAAAA' +
  'AAAAAAAAAAP//+AAAAAAAAAAAAAAAAAD///gAAAAAAAAAAAAAAAAB///4AAAAAAAAAAAAAAAAAf//+AAAAAAAAAAAAAAAAAP' +
  '///AAAAAAAAAAAAAAAAAD///wAAAAAAAAAAAAAAAAB///8AAAAAAAAAAAAAAAAAf///AAAAAAAAAAAAAAAAAP///gAAAAAAA' +
  'AAAAAAAAAH///4AAAAAAAAAAAAAAAAB///+AAAAAAAAAAAAAAAAA////AAAAAAAAAAAAAAAAAf///wAAAAAAAAAAAAAAAAP/' +
  '//8AAAAAAAAAAAAAAAAD///+AAAAAAAAAAAAAAAAB////gAAAAAAAAAAAAAAAA////4AAAAAAAAAAAAAAAAf///8AAAAAAAA' +
  'AAAAAAAAH////AAAAAAAAAAAAAAAAD////wAAAAAAAAAAAAAAAB////8AAAAAAAAAAAAAAAA/////AAAAAAAAAAAAAAAAP//' +
  '//wAAAAAAAAAAAAAAAH////8AAAAAAAAAAAAAAAD/////AAAAAAAAAAAAAAAA/////4AAAAAAAAAAAAAAAf/////AAAAAAAA' +
  'AAAAAAAP//////////4AAAAAAAAAD///////////AAAAAAAAAB///////////4AAAAAAAAAf///////////AAAAAAAAAP///' +
  '////////wAAAAAAAAH///////////+AAAAAAAAB////////////gAAAAf//A////////////4AAAAf//8f///////////+AA' +
  'AAP////////////////gAAAD////////////////wAAAB////////////////8AAAAf///////////////+AAAAH////////' +
  '////////AAAAB////////////////AAAAAf///////////////gAAAAH///////////////4AAAAB////////////////AAA' +
  'AAf///////////////8AAAAH////////////////gAAAB////////////////8AAAAf////////////////gAAAH////////' +
  '////////4AAAB////////////////+AAAAf////////////////gAAAH////////////////4AAAB////////////////+AA' +
  'AAf////////////////AAAAH////////////////wAAAB////////////////4AAAAf///////////////8AAAAH////////' +
  '///////8AAAAB///////////////+AAAAAf///////////////AAAAAH///////////////4AAAAB////////////////gAA' +
  'AAf///////////////8AAAAH////////////////gAAAB////////////////4AAAAf///////////////+AAAAH////////' +
  '////////gAAAB////////////////4AAAAf///////////////+AAAAH////////////////gAAAB////////////////4AA' +
  'AAf///////////////8AAAAH///////////////+AAAAB////////////////AAAAAf///////////////AAAAAH////////' +
  '//////+AAAAAB///////////////gAAAAAf//////////////4AAAAAH///////////////gAAAAB///////////////8AAA' +
  'AAf///////////////AAAAAH///////////////wAAAAB///////////////+AAAAAf///////////////gAAAAH////////' +
  '///////4AAAAB///////////////+AAAAAP///////////////AAAAAD///////////////wAAAAAf//8D//////////4AAA' +
  'AAB//8AH/////////8AAAAAAAAAAAf////////+AAAAAAAAAAAA/////x//gAAAAAAAAAAAAD////4AAAAAAAAAAAAAAAAH/' +
  '//8AAAAAAAAAAAAAAAAAD//wAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── victory — 비트맵 실루엣(128×128, 잉크 29.8%)
//   원본 마스크: victory.png
var MASK_VICTORY = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAPgAAAAAAAAAAAAAH4AAAAP8AAAAAAAAAAAAAD/gAAAH/gAAAAAAAAAAAAB/4AAAB/8AAAAAAA' +
  'AAAAAA//AAAA//AAAAAAAAAAAAAP/wAAAP/wAAAAAAAAAAAAD/+AAAD/+AAAAAAAAAAAAA//gAAB//AAAAAAAAAAAAAP/4AA' +
  'Af/wAAAAAAAAAAAAD/+AAAH/8AAAAAAAAAAAAA//wAAB//AAAAAAAAAAAAAP/8AAA//gAAAAAAAAAAAAD//AAAP/4AAAAAAA' +
  'AAAAAAf/4AAD/+AAAAAAAAAAAAAH/+AAB//gAAAAAAAAAAAAB//gAAf/4AAAAAAAAAAAAAf/4AAH/8AAAAAAAAAAAAAD//AA' +
  'B//AAAAAAAAAAAAAA//wAA//wAAAAAAAAAAAAAP/8AAP/8AAAAAAAAAAAAAD//AAD/+AAAAAAAAAAAAAAf/4AB//gAAAAAAA' +
  'AAAAAAH/+AAf/4AAAAAAAAAAAAAB//gAH/+AAAAAAAAAAAAAAf/4AB//AAAAAAAAAAAAAAH//AA//wAAAAAAAAAAAAAA//wA' +
  'P/8AAAAAAAAAAAAAAP/8AD//AAAAAAAAAAAAAAD//AA//gAAAAAAAAAAAAAA//4Af/4AAAAAAAAAAAAAAH/+AH/+AAAAAAAA' +
  'AAAAAAB//gB//AAAAAAAAAAAAAAAf/4A//wAAAAAAAAAAAAAAH//AP/8AAAAAAAAAAAAAAA//wD//AAAAAAAAAAAAAAAP/8B' +
  '//gAAAAAAAAAAAAAAD//gf/4AAAAAAAAAAAAAAA//8P/+AAAAAAAAAAAAAAAP/////gAAAAAAAAAAAAAAB/////wAAAAAAAA' +
  'AAAAAAAf////+AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAB/////4AAAAAAAAAAAAAAAP/////gAAAAAAAAAAAAAAD///' +
  '///gAAAAAAAAAAAAAA//////+AAAAAAAAAAAAAAP//////wAAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAf//////gAAAAAA' +
  'AAAAAAAH//////4AAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAf//////wAAAAAAAAAAAAAH//////+AAAAAAAAAAAAAB///' +
  '////8AAAAAAAAAAAAAf///////wAAAAAAAAAAAAH///////+AAAAAAAAAAAAB////////wAAAAAAAAAAAA////////8AAAAA' +
  'AAAAAAAf////////AAAAAAAAAAAAf////////wAAAAAAAAAAAf////////8AAAAAAAAAAAP/////////AAAAAAAAAAAH////' +
  '/////gAAAAAAAAAAD/////////4AAAAAAAAAAB/////////8AAAAAAAAAAA//////////AAAAAAAAAAAP/////////gAAAAA' +
  'AAAAAH/////////4AAAAAAAAAAB/////////+AAAAAAAAAAA//////////wAAAAAAAAAAP/////////8AAAAAAAAAAD/////' +
  '/////AAAAAAAAAAA//////////wAAAAAAAAAAP/////////8AAAAAAAAAAD//////////AAAAAAAAAAA//////////wAAAAA' +
  'AAAAAP/////////8AAAAAAAAAAD//////////AAAAAAAAAAA//////////gAAAAAAAAAAH/////////4AAAAAAAAAAB/////' +
  '////+AAAAAAAAAAAf/////////gAAAAAAAAAAH/////////4AAAAAAAAAAA/////////+AAAAAAAAAAAP/////////AAAAAA' +
  'AAAAAD/////////wAAAAAAAAAAAf////////8AAAAAAAAAAAH////////+AAAAAAAAAAAA/////////gAAAAAAAAAAAP////' +
  '////4AAAAAAAAAAAB////////8AAAAAAAAAAAAf////////AAAAAAAAAAAAD////////gAAAAAAAAAAAA////////4AAAAAA' +
  'AAAAAAH///////8AAAAAAAAAAAAA////////AAAAAAAAAAAAAP///////gAAAAAAAAAAAAB///////4AAAAAAAAAAAAAf///' +
  '///8AAAAAAAAAAAAAD///////AAAAAAAAAAAAAA///////gAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAB//////8AAAAAAA' +
  'AAAAAAAf//////AAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAP/////+AAAAAAAAAAAAAAD///' +
  '///gAAAAAAAAAAAAAA//////4AAAAAAAAAAAAAAP/////+AAAAAAAAAAAAAAD//////gAAAAAAAAAAAAAA//////4AAAAAAA' +
  'AAAAAAAH/////+AAAAAAAAAAAAAAB//////AAAAAAAAAAAAAAAP/////wAAAAAAAAAAAAAAB/////wAAAAAAAAAAAAAAAD//' +
  '//wAAAAAAAAAAAAAAAAB//+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── oksign — 비트맵 실루엣(128×128, 잉크 31.5%)
//   원본 마스크: oksign.png
var MASK_OKSIGN = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAA+AAAAAAAAAAAAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAP+AAAAAAAAAAAAAAAAA8AD/gAAAAAAAA' +
  'AAAAAAAAfwB/8AAAAAAAAAAAAAAAAP8Af/AAAAAAAAAAAAAAAAD/gP/wAAAAAAAAAAAAAAAB/4D/8AAAAAAAAAAAAAAAAf/B' +
  '/+AAAAAAAAAAAAAAAAH////gAAAAAAAAAAAAAAAB////4AAAAAAAAAAAAAAAA////+AAAAAAAAAAAAAAAAP////AAAAAAAAA' +
  'AAAAAAAD////wAAAAAAAAAAAAAAAA////4AAAAAAAAAAAAAAAAf///+AAAAAAAAAAAAAAAAH////AAAAAAAAAAAAAAAAB///' +
  '/wAAAAAAAAAAAAAB8A////8AAAAAAAAAAAAAA/gP///+AAAAAAAAAAAAAAf4H////gAAAAAAAAAAAAAH/B////wAAAAAAAAA' +
  'AAAAB//////8AAAAAAAAAAAAAA//////+AAAAAAAAAAAAAAP//////gAAAAAAAAAAAAAD//////4AAAAAAAAAAAAAA//////' +
  '8AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAD//////gAAAAAAAAAAAAAA//////4AAAAAAAAAAAAAAP/////8AAAAAAAAAA' +
  'AAAAD//////AAAAAAAAAAAAAAA//////wAAAAAAAAAAAAAAP/////4AAAAAAAAAAAAAAD//////AAAAAAAAAAAAAAA//////' +
  'wAAAAAAAAAAAAAAP/////+D4AAAAAAAAAAAAD////////gAAAAAAAAAAAA////////+AAAAAAAAAAAAP////////4AAAAAAA' +
  'AAAAD/////////gAAAAAAAAAAA/////////8AAAAAAAAAAAP/////////wAAAAAAAAAAD/////////+AAAAAAAAAAA//////' +
  '////wAAAAAAAAAAP/////////+AAAAAAAAAAD//////////wAAAAAAAAAA//////////+AAAAAAAAAAP//////////gAAAAA' +
  'AAAAB//////////8AAAAAAAAAAf//////4P//AAAAAAAAAAH//////4Af/4AAAAAAAAAB//////8AB/+AAAAAAAAAAf/////' +
  '+AAP/gAAAAAAAAAH//////AAB/8AAAAAAAAAB//////AAAP/AAAAAAAAAAf/////wAAB/4AAAAAAAAAH/////4AAAH/gAAAA' +
  'AAAAB/////8AAAB/4AAAAAAAAAf/////AAAAf+AAAAAAAAAH/////wAAAH/gAAAAAAAAB/////4AAAB/4AAAAAAAAAf////+' +
  'AAAA/+AAAAAAAAAH/////gAAAP/gAAAAAAAAB/////4AAAD/4AAAAAAAAAf////+AAAB/+AAAAAAAAAH/////wAAAf/AAAAA' +
  'AAAAB/////8AAAH/wAAAAAAAAAf/////AAAB/8AAAAAAAAAH/////4AAA//AAAAAAAAAB//////AAAf/wAAAAAAAAAf/////' +
  '4AAP/8AAAAAAAAAD//////AAH//AAAAAAAAAA//////4AD//wAAAAAAAAAP//////AD//8AAAAAAAAAD//////8B///AAAAA' +
  'AAAAA///////////gAAAAAAAAAP//////////4AAAAAAAAAD//////////8AAAAAAAAAA//////////+AAAAAAAAAAP/////' +
  '/////gAAAAAAAAAD//////////wAAAAAAAAAA//////////4AAAAAAAAAAP/////////8AAAAAAAAAAB/////////+AAAAAA' +
  'AAAAAf/////////gAAAAAAAAAAH/////////wAAAAAAAAAAB/////////4AAAAAAAAAAAP////////8AAAAAAAAAAAD/////' +
  '///+AAAAAAAAAAAA/////////gAAAAAAAAAAAH////////wAAAAAAAAAAAB////////4AAAAAAAAAAAAf///////8AAAAAAA' +
  'AAAAAD///////+AAAAAAAAAAAAA////////AAAAAAAAAAAAAP///////gAAAAAAAAAAAAB///////wAAAAAAAAAAAAAf////' +
  '//4AAAAAAAAAAAAAH//////8AAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAD//////gAAAAAAAA' +
  'AAAAAA//////wAAAAAAAAAAAAAAP/////4AAAAAAAAAAAAAAD/////+AAAAAAAAAAAAAAA//////AAAAAAAAAAAAAAAP////' +
  '/gAAAAAAAAAAAAAAD/////4AAAAAAAAAAAAAAA/////+AAAAAAAAAAAAAAAP/////gAAAAAAAAAAAAAAD/////4AAAAAAAAA' +
  'AAAAAAf////8AAAAAAAAAAAAAAAH/////AAAAAAAAAAAAAAAB/////wAAAAAAAAAAAAAAAP////4AAAAAAAAAAAAAAAA////' +
  '8AAAAAAAAAAAAAAAAA///wAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── lovesign — 비트맵 실루엣(128×128, 잉크 33.0%)
//   원본 마스크: lovesign.png
var MASK_LOVESIGN = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAHgAAAAAAAAAAAAAAAAAAAH+AAAAAAAAAAAAAAAAAAAD/wAAAAAAAAAAAAAAAAAAB/8AAAAAAAAAAAA' +
  'AAAAAAAf/gAAAAAAAAAAAAAAAAAAP/4AAAAAAAAAAAAAAAAAAD/+AAAAAAAAAAAAAAAAAAA//wAAAAAAAAAAAAAAAAAAP/8A' +
  'AAAAAAAAAAAAAAAAAD//AAAAAAAAAAAAAAAAAAA//wAAAAAAAAAAAAAAAAAAP/8AAAAAAAAAAAAAAAAAAD//AAAAAAAAAAAA' +
  'AAAAAAAf/wAAAAAAAAAAAAAAAAAAH/+AAAAAAAPwAAAAAAAAAB//gAAAAAAH+AAAAAAAAAAf/4AAAAAAD/wAAAAAAAAAH/+A' +
  'AAAAAA/8AAAAAAAAAB//gAAAAAAf/gAAAAAAAAAf/4AAAAAAH/4AAAAAAAAAH/+AAAAAAB/+AAAAAAAAAA//wAAAAAAf/gAA' +
  'AAAAAAAP/8AAAAAAP/wAAAAAAAAAD//AAAAAAD/8AAAAAAAAAA//wAAAAAA//AAAAAAAAAAP/8AAAAAAP/wAAAAAAAAAD//A' +
  'AAAAAH/8AAAAAAAAAA//wAAAAAB/+AAAAAAAAAAH/+AAAAAAf/gAAAAAAAAAB//gAAAAAP/4AAAAAAAAAAf/4AAAAAD/+AAA' +
  'AAAAAAAH/+AAAAAA//gAAAAAAAAAB//gAAAAAP/wAAAAAAAAAAf/4AAAAAH/8AAAAAAAAAAH/+AAAAAB//AAAAAAAAAAB//w' +
  'AAAAAf/wAAAAAAAAAAP/8AAAAAP/8AAAAAAAAAAD//g+AAAD/+AAAAAAAAAAA////4D8A//gAAAAAAAAAAP////B/wf/4AAA' +
  'AAAAAAAD/////////+AAAAAAAAAAA//////////gAAAAAAAAAAP/////////wAAAAAAAAAAB/////////8AAAAAAAAAAAf//' +
  '///////AAAAAAAAAAAH/////////wAAAAAAAAAAB/////////4AAAAAAAAAAAf////////+AAAAAAAAAAAH/////////gAAA' +
  'AAAAAAAB/////////4AAAAAAAAAAAP////////+AAAAAAAAAAAD/////////AAAAAAAAAAAA/////////wAAAAAAAAAAAP//' +
  '//////8AAAAAAAP8AAD/////////AAAAAAAP/4AA/////////wAAAAAAH//AAP////////8AAAAAAD//8AD/////////AAAA' +
  'AAA///wA/////////wAAAAAAf//+AP////////8AAAAAAH///wD////////+AAAAAAA///+B/////////gAAAAAAH///w///' +
  '//////4AAAAAAA/////////////+AAAAAAAD/////////////gAAAAAAAP////////////4AAAAAAAB////////////+AAAA' +
  'AAAAH////////////gAAAAAAAA////////////4AAAAAAAAH///////////+AAAAAAAAA////////////gAAAAAAAAP/////' +
  '//////4AAAAAAAAB///////////+AAAAAAAAAP///////////gAAAAAAAAB///////////4AAAAAAAAAf//////////8AAAA' +
  'AAAAAD///////////AAAAAAAAAA///////////wAAAAAAAAAH//////////8AAAAAAAAAB///////////AAAAAAAAAAP////' +
  '//////wAAAAAAAAAB//////////8AAAAAAAAAAf/////////+AAAAAAAAAAD//////////gAAAAAAAAAAf/////////4AAAA' +
  'AAAAAAH/////////+AAAAAAAAAAA//////////AAAAAAAAAAAH/////////wAAAAAAAAAAA/////////8AAAAAAAAAAAH///' +
  '/////+AAAAAAAAAAAB/////////gAAAAAAAAAAAP////////4AAAAAAAAAAAA////////8AAAAAAAAAAAAH////////AAAAA' +
  'AAAAAAAB////////wAAAAAAAAAAAAP///////4AAAAAAAAAAAAB///////+AAAAAAAAAAAAAP///////AAAAAAAAAAAAAB//' +
  '/////wAAAAAAAAAAAAAf//////4AAAAAAAAAAAAAD//////+AAAAAAAAAAAAAA///////gAAAAAAAAAAAAAH//////wAAAAA' +
  'AAAAAAAAB//////8AAAAAAAAAAAAAAf//////AAAAAAAAAAAAAAH//////gAAAAAAAAAAAAAB//////4AAAAAAAAAAAAAAP/' +
  '////+AAAAAAAAAAAAAAD//////gAAAAAAAAAAAAAA//////4AAAAAAAAAAAAAAP/////+AAAAAAAAAAAAAAD//////gAAAAA' +
  'AAAAAAAAA//////4AAAAAAAAAAAAAAH/////8AAAAAAAAAAAAAAB/////+AAAAAAAAAAAAAAAH/////AAAAAAAAAAAAAAAAf' +
  '///+AAAAAAAAAAAAAAAAAf//4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── prayer — 비트맵 실루엣(128×128, 잉크 27.6%)
//   원본 마스크: prayer.png
var MASK_PRAYER = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAeB4AAAAAAAAAAAAAAAAAAP4/AAAAAAAAAAAAAAAAAAD+fwAAAAAAAAAAAAAAAAAA/n8AAAAAAAAA' +
  'AAAAAAAAAP5/AAAAAAAAAAAAAAAAAAH+f4AAAAAAAAAAAAAAAAAB/n+AAAAAAAAAAAAAAAAAAf5/gAAAAAAAAAAAAAAAAAH+' +
  'f4AAAAAAAAAAAAAAAAAB/n+AAAAAAAAAAAAAAAAAAf5/gAAAAAAAAAAAAAAAAAP+f8AAAAAAAAAAAAAAAAAD/n/AAAAAAAAA' +
  'AAAAAAAAA/5/wAAAAAAAAAAAAAAAAAP+f8AAAAAAAAAAAAAAAAAD/n/AAAAAAAAAAAAAAAAAB/5/wAAAAAAAAAAAAAAAAAf+' +
  'f+AAAAAAAAAAAAAAAAAH/n/gAAAAAAAAAAAAAAAAB/5/4AAAAAAAAAAAAAAAAAf+f+AAAAAAAAAAAAAAAAAH/n/gAAAAAAAA' +
  'AAAAAAAAD/5/8AAAAAAAAAAAAAAAAA/+f/AAAAAAAAAAAAAAAAAP/n/wAAAAAAAAAAAAAAAAD/5/8AAAAAAAAAAAAAAAAA/+' +
  'f/AAAAAAAAAAAAAAAAAf/n/wAAAAAAAAAAAAAAAAH/5/+AAAAAAAAAAAAAAAAB/+f/gAAAAAAAAAAAAAAAAf/n/4AAAAAAAA' +
  'AAAAAAAAH/5/+AAAAAAAAAAAAAAAAD/+f/wAAAAAAAAAAAAAAAA//n/8AAAAAAAAAAAAAAAAP/5//AAAAAAAAAAAAAAAAD/+' +
  'f/wAAAAAAAAAAAAAAAA//n/+AAAAAAAAAAAAAAAAf/5//gAAAAAAAAAAAAAAAH/+f/4AAAAAAAAAAAAAAAD//n/+AAAAAAAA' +
  'AAAAAAAA/4JB/wAAAAAAAAAAAAAAAP84PP8AAAAAAAAAAAAAAAH/fH7/gAAAAAAAAAAAAAAB/v5/f4AAAAAAAAAAAAAAA/7+' +
  'f3/AAAAAAAAAAAAAAAP+/n9/wAAAAAAAAAAAAAAD/f5/v8AAAAAAAAAAAAAAB/3+f7/gAAAAAAAAAAAAAAf9/n+/4AAAAAAA' +
  'AAAAAAAH/f5/v/AAAAAAAAAAAAAAD//+f//wAAAAAAAAAAAAAA/7/n/f8AAAAAAAAAAAAAAP//5///AAAAAAAAAAAAAAD/v+' +
  'f9/wAAAAAAAAAAAAAA/7/n/f8AAAAAAAAAAAAAAf//5/3/gAAAAAAAAAAAAAH/f+f+/4AAAAAAAAAAAAAB/3/n/v+AAAAAAA' +
  'AAAAAAAf9/5/7/gAAAAAAAAAAAAAH/f+f+/4AAAAAAAAAAAAAB///n/v+AAAAAAAAAAAAAAf//5///gAAAAAAAAAAAAAH//+' +
  'f//4AAAAAAAAAAAAAB///n//+AAAAAAAAAAAAAAf//5///gAAAAAAAAAAAAAH//+f//8AAAAAAAAAAAAAD///n///AAAAAAA' +
  'AAAAAAA///5///wAAAAAAAAAAAAAP//+f//8AAAAAAAAAAAAAD///n///AAAAAAAAAAAAAA///5///4AAAAAAAAAAAAAf//+' +
  'f//+AAAAAAAAAAAAAH///n///gAAAAAAAAAAAAB///5///4AAAAAAAAAAAAAf//+f///AAAAAAAAAAAAAP///n///wAAAAAA' +
  'AAAAAAD///5///8AAAAAAAAAAAAB///+f///gAAAAAAAAAAAAf///n///8AAAAAAAAAAAAP///5////AAAAAAAAAAAAH///+' +
  'f///4AAAAAAAAAAAD////n////AAAAAAAAAAAB////5////4AAAAAAAAAAA////+f////AAAAAAAAAAAf////n////4AAAAA' +
  'AAAAAf////5/////gAAAAAAAAAP////+f////8AAAAAAAAAH/////n/////gAAAAAAAAH/////5/////+AAAAAAAAD/////8' +
  'P/////wAAAAAAAB//////D//////AAAAAAAB//////w//////4AAAAAAB//////4H//////gAAAAAA//////+B//////8AAA' +
  'AAA///////AP//////wAAAAAf//////wD//////+AAAAAH//////4Af//////gAAAAD//////8AD//////4AAAAAf/////+A' +
  'Af/////+AAAAAH//////AAD//////gAAAAA//////gAAf/////wAAAAAH/////wAAD/////4AAAAAB/////wAAAP////+AAA' +
  'AAAP////4AAAB/////AAAAAAB////8AAAAP////gAAAAAAP///+AAAAB////wAAAAAAB////AAAAAP///8AAAAAAAP///gAA' +
  'AAB///+AAAAAAAD///wAAAAAP///AAAAAAAAf//4AAAAAB///gAAAAAAAD//8AAAAAAP//wAAAAAAAAf/+AAAAAAB//4AAAA' +
  'AAAAD//AAAAAAAP/8AAAAAAAAAf/gAAAAAAB/+AAAAAAAAAD/wAAAAAAAP/AAAAAAAAAAf4AAAAAAAB/gAAAAAAAAAD8AAAA' +
  'AAAAPwAAAAAAAAAAeAAAAAAAAB4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── fist — 비트맵 실루엣(128×128, 잉크 57.2%)
//   원본 마스크: fist.png
var MASK_FIST = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAf4AAAAAAAAAAAAAAAA/gA//gAAAAAAAAAAAAAAB//Af/8AAAAAAAAAAAAAAA//4P//gB8AAAAA' +
  'AAAAAwAf//H//8D/8AAAAAAAAB/wP//5///B//wAAAAAAAB/+D//////4//+AAAAAAAB//w//////+f//wAAAAAAA///////' +
  '//////8AAAAAAAf/////////////gAAAAAAH/////////////4AAAAAAD/////////////+AAAAAAA//////////////wAAA' +
  'AAAP/////////////8AAAAAAD//////////////AAAAAAB//////////////wAAAAAAP/////////////4AAAAAAD///////' +
  '//////+AAAAAAA//////////////gAAAAAAP/////////////4AAAAAAD/////////////+AAAAAAA//////////////gAAA' +
  'AAAP/////////////4AAAAAAD/////////////+AAAAAAA//////////////gAAAAAAP/////////////4AAAAAAD///////' +
  '//////+AAAAAAA//////////////gAAAAAAP/////////////4AAAAAAD/////////////+AAAAAAB//////////////gAAA' +
  'AAA//////////////4AAAAAAf/////////////+AAAAAAH//////////////gAAAAAB//////////////4AAAAAA////////' +
  '//////+AAAAAAP//////////////AAAAAAD//////////////wAAAAAA//////////////+AAAAAAf//////////////4AAA' +
  'AAH///////////////gAAAAB///////////////8AAAAAf///////////////AAAAAH///////////////4AAAAB////////' +
  '////////AAAAAf///////////////wAAAAP///////////////+AAAAD////////////////gAAAA////////////////4AA' +
  'AAP///////////////+AAAAD////////////////wAAAA////////////////8AAAAP////////////////AAAAD////////' +
  '////////wAAAA////////////////8AAAAP////////////////AAAAB////////////////wAAAAf///////////////4AA' +
  'AAH///////////////+AAAAB////////////////gAAAAf///////////////4AAAAH///////////////+AAAAB////////' +
  '////////AAAAAf///////////////wAAAAD///////////////8AAAAA////////////////AAAAAP///////////////gAA' +
  'AAD///////////////4AAAAAf//////////////8AAAAAH///////////////AAAAAB///////////////wAAAAAP///////' +
  '///////4AAAAAD//////////////+AAAAAA///////////////AAAAAAH//////////////wAAAAAB//////////////4AAA' +
  'AAAP/////////////8AAAAAAD//////////////AAAAAAAf/////////////gAAAAAAH/////////////wAAAAAAA///////' +
  '//////8AAAAAAAH////////////+AAAAAAAB/////////////AAAAAAAAP////////////gAAAAAAAD////////////4AAAA' +
  'AAAAf///////////8AAAAAAAAD///////////+AAAAAAAAA////////////gAAAAAAAAH///////////wAAAAAAAAB//////' +
  '/////4AAAAAAAAAP//////////+AAAAAAAAAD///////////gAAAAAAAAAf//////////wAAAAAAAAAH//////////8AAAAA' +
  'AAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAAAAAAAP/////' +
  '/////gAAAAAAAAAD//////////4AAAAAAAAAA//////////+AAAAAAAAAAP//////////gAAAAAAAAAD//////////4AAAAA' +
  'AAAAA//////////+AAAAAAAAAAP//////////gAAAAAAAAAD//////////4AAAAAAAAAA//////////8AAAAAAAAAAP/////' +
  '/////AAAAAAAAAAD//////////wAAAAAAAAAAf/////////4AAAAAAAAAAH/////////+AAAAAAAAAAA//////////AAAAAA' +
  'AAAAAH/////////gAAAAAAAAAAAf////////wAAAAAAAAAAAB////////wAAAAAAAAAAAAH///////gAAAAAAAAAAAAAD///' +
  '//8AAAAAAAAAAAAAAAA////AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

/* ── 내부 백선(골) — 실루엣 안쪽을 분절하는 문법 (2026-09-02, B4) ─────────────
 *   장미(`pointInRose` ②)가 로그나선 골로 꽃잎을 갈랐던 그 기법을 **일반화**한 것이다.
 *   왜 필요한가: B1 인지 게이트 실측(`~/wiacode-shape-lab/recog/BASELINE_2026-09-01.json`)에서
 *   손 계열은 **세운 손가락은 이미 갈라져 있는데 접힌 손가락(주먹·손바닥 덩어리)이 통짜**라
 *   판정자가 "hand"·"shield"·"bear"·"rabbit"·"kidney" 로 읽었다. 덩어리에 접힌 마디선을
 *   그어 주면 제스처가 읽힌다 — 그림을 다시 그리지 않고(=B2-b ⛔ 우회) 코드로 해결한다.
 *
 *   ★설계 제약 세 가지 (지키지 않으면 다른 게 깨진다)
 *   ① **골은 실루엣 경계에 닿지 않는다.** 밀착 풀이의 `_filledOutline` 은 바깥에서 flood fill
 *      하므로 안쪽 백선은 무시하지만, 골이 경계까지 열리면 채움이 새어 들어와 **윤곽이 달라지고
 *      기존 HUG_PINS 가 무효가 된다.** 그래서 끝을 경계에서 띄운다(닫힌 골).
 *   ② **폭은 3셀 이상.** 규격 v1.1 속살형 조항. 격자 L 에서 1셀 = 2/128 = 0.015625(정규화)
 *      이므로 반폭 0.024 ≈ 3.1셀. 이보다 얇으면 축소본에서 뭉개져 아무 효과가 없다.
 *   ③ **중앙 예약 근처를 피한다.** 코어·궤도는 골 위에 덮여 그려지지만, 데이터 셀을 잃는
 *      자리를 굳이 겹칠 이유가 없다.
 *   ★골은 데이터 셀을 먹는다 — 넣거나 고치면 용량표(`shape-registry.js` qrTable)를 다시 뽑을 것. */
var GROOVE_HW = 0.024;          // 골 반폭(정규화) ≈ 3.1셀
var SHAPE_GROOVES = {
  /* fist(주먹) — B1 실측 최악(카탈로그 1/2 "shield", 디테일 1/2 "bear"). 위 경계에 주먹뼈
   *   융기 4개는 있는데 아래가 통짜라 방패로 읽힌다. 융기 사이 골(x≈-0.40 / -0.03 / +0.35,
   *   경계 프로파일 실측)에서 아래로 손가락 분할선을 긋고, 접힌 마디선 1개와 엄지선 1개를 더한다. */
  fist: [
    { pts: [[-0.40, -0.72], [-0.42, -0.30], [-0.44, -0.02]] },   // 손가락 분할 ①
    { pts: [[-0.03, -0.76], [-0.04, -0.32], [-0.05, -0.02]] },   // 손가락 분할 ②
    { pts: [[ 0.35, -0.76], [ 0.37, -0.32], [ 0.39, -0.02]] },   // 손가락 분할 ③
    { pts: [[-0.46,  0.06], [ 0.00,  0.10], [ 0.42,  0.04]] },   // 접힌 마디선(가로)
    { pts: [[-0.48,  0.34], [-0.05,  0.44], [ 0.30,  0.40]] },   // 엄지가 앞을 가로지르는 선
    /* ★오너 지시(2026-09-03) — 오른쪽 위 기둥이 엄지인데 구분이 없어 손가락과 붙어 보였다.
     *   그 자리에 얕은 아크를 그어 엄지 마디를 낸다(실측: 그 구간 잉크는 x 0.30~0.64). */
    { pts: [[ 0.34, -0.46], [ 0.48, -0.53], [ 0.61, -0.45]] },   // 엄지 마디 아크
  ],
  /* lovesign(사랑해요) — 디테일 판정 0/2, 둘 다 그냥 "hand". 검지·새끼는 이미 갈라져 있고
   *   그 사이 **접힌 중지·약지**가 손바닥 덩어리에 묻혀 있다. 둘을 가르고 마디선을 얹는다. */
  lovesign: [
    { pts: [[ 0.00, -0.20], [ 0.01,  0.02], [ 0.02,  0.16]] },   // 접힌 중지/약지 사이
    { pts: [[ 0.22, -0.20], [ 0.23,  0.02], [ 0.24,  0.16]] },   // 접힌 약지/새끼 사이
    { pts: [[-0.28,  0.26], [ 0.10,  0.32], [ 0.44,  0.24]] },   // 접힌 마디선(가로)
  ],
  /* victory(브이) — 디테일 판정에서 "rabbit". V 만 보이고 접힌 약지·새끼가 안 읽힌다.
   *   덩어리 오른쪽(접힌 손가락 쪽)에 분할선을, 아래에 마디선을 넣는다. */
  victory: [
    { pts: [[ 0.24, -0.02], [ 0.26,  0.14], [ 0.27,  0.30]] },   // 접힌 약지/새끼 사이
    /* ★x 0.50 은 실루엣 밖이었다(골 커버리지 0% 로 검출) — 잉크는 x≈0.44 까지다. */
    { pts: [[ 0.38,  0.02], [ 0.39,  0.16], [ 0.40,  0.28]] },   // 새끼 쪽
    { pts: [[-0.26,  0.44], [ 0.10,  0.50], [ 0.40,  0.42]] },   // 접힌 마디선(가로)
  ],
  /* oksign(오케이) — 디테일 판정에서 "kidney". O 는 읽히는데 손이 안 읽힌다.
   *   O 근처는 건드리지 않고(정체성), 손바닥 덩어리에만 마디선을 넣는다. */
  oksign: [
    /* ★첫 안은 x -0.62 / 마디선 좌단 -0.68 이 실루엣 밖이었다(커버리지 0%·76% 로 검출).
     *   실측 잉크 범위는 x -0.50~-0.10 이다. 눈으로 보지 말고 커버리지로 확인할 것. */
    { pts: [[-0.42, -0.28], [-0.41, -0.06], [-0.40,  0.12]] },   // 접힌 손가락 분할 ①
    { pts: [[-0.26, -0.28], [-0.25, -0.06], [-0.24,  0.12]] },   // 접힌 손가락 분할 ②
    { pts: [[-0.44,  0.28], [-0.28,  0.32], [-0.14,  0.30]] },   // 접힌 마디선(가로)
  ],
  /* finger(검지) — 디테일 판정에서 "hand". 세운 검지 말고는 구분이 없다.
   *   접힌 중지·약지·새끼를 가르고 마디선을 얹는다. */
  finger: [
    { pts: [[-0.22, -0.06], [-0.21,  0.10], [-0.20,  0.24]] },   // 접힌 손가락 분할 ①
    { pts: [[-0.04, -0.06], [-0.03,  0.10], [-0.02,  0.24]] },   // 접힌 손가락 분할 ②
    { pts: [[-0.30,  0.36], [ 0.04,  0.42], [ 0.30,  0.36]] },   // 접힌 마디선(가로)
  ],
};
function _segDist2(px, py, ax, ay, bx, by) {
  var vx = bx - ax, vy = by - ay, wx = px - ax, wy = py - ay;
  var L2 = vx * vx + vy * vy;
  var t = L2 > 0 ? (wx * vx + wy * vy) / L2 : 0;
  if (t < 0) t = 0; else if (t > 1) t = 1;
  var dx = px - (ax + t * vx), dy = py - (ay + t * vy);
  return dx * dx + dy * dy;
}
function inGroove(list, nx, ny) {
  for (var i = 0; i < list.length; i++) {
    var g = list[i], pts = g.pts, hw = g.hw || GROOVE_HW, hw2 = hw * hw;
    for (var j = 0; j + 1 < pts.length; j++)
      if (_segDist2(nx, ny, pts[j][0], pts[j][1], pts[j + 1][0], pts[j + 1][1]) < hw2) return true;
  }
  return false;
}
/* 실루엣 판정 = 본체(아래) − 내부 골. 골이 없는 형상은 픽셀까지 예전 그대로다. */
// ── badge — 비트맵 실루엣(128×128, 잉크 56.4%)
//   원본 마스크: badge.png
var MASK_BADGE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAACAA/AAAAAAAAAAAAAAAAAPwAf+AAAAAAAAAAAAAAAAf+AH/4AAAAAAAAAAAAAAAf/g' +
  'B//wAAAAAAAAAAAAAA//4Af//gAAAAAAAAAAAAB//+AH///AAAAAAAAAAAAD///gA///+AAAAAAAAAAAH///wAP///8AAAAA' +
  'AAAAAP///8AD////wAAAAAAAAAf////AA/////wAAAAAAAA/////wAH/////AAAAAAAA/////4AB/////+AAP/wAB/////+A' +
  'Af/////8Af//gD//////gAH//////wP//8D//////4AA///////f//////////8AAP//////////////////AAD/////////' +
  '/////////wAA//////////////////8AAH//////////////////AAB//////////////////wAAf/////////////////4A' +
  'AH/////////////////+AAB//////////////////gAAf/////////////////4AAD/////////////////8AAA/////////' +
  '/////////AAAP/////////////////wAAD/////////////////8AAB//////////////////gAAf/////////////////4A' +
  'AH/////////////////+AAB//////////////////gAAf/////////////////8AAP//////////////////AAD/////////' +
  '/////////wAA//////////////////8AAP//////////////////AAH//////////////////4AB//////////////////+A' +
  'Af//////////////////gAH//////////////////4AD///////////////////AA///////////////////wAP/////////' +
  '/////////8AD///////////////////AB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gA///////////////////wAB/////////' +
  '/////////gAAH/////////////////gAAAP////////////////AAAAA////////////////AAAAAD///////////////AAA' +
  'AAAH/////////////+AAAAAAAf////////////+AAAAAAAA////////////8AAAAAAAAB///////////4AAAAAAAAAP/////' +
  '/////8AAAAAAAAAB//////////+AAAAAAAAAAf//////////gAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAA' +
  'AAAAAf//////////gAAAAAAAAAH//////////8AAAAAAAAAD///////////AAAAAAAAAA///////////wAAAAAAAAAf/////' +
  '/////+AAAAAAAAAH///////////gAAAAAAAAB///////////4AAAAAAAAAf//////////+AAAAAAAAAP///////////wAAAA' +
  'AAAAD///////////8AAAAAAAAA////////////AAAAAAAAAP///////////wAAAAAAAAD///////////8AAAAAAAAA//////' +
  '//////AAAAAAAAAP///////////wAAAAAAAAD///////////8AAAAAAAAA////////////AAAAAAAAAP///////////wAAAA' +
  'AAAAD///////////8AAAAAAAAA////////////AAAAAAAAAP///////////wAAAAAAAAB///////////8AAAAAAAAAf/////' +
  '/////+AAAAAAAAAH///////////gAAAAAAAAB///////////4AAAAAAAAAP//////////8AAAAAAAAAD///////////AAAAA' +
  'AAAAA///////////wAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAAAAAAAP//////////AAAAAAAAAAD/////' +
  '/////wAAAAAAAAAAf/////////4AAAAAAAAAAH/////////+AAAAAAAAAAA//////////AAAAAAAAAAAH/////////gAAAAA' +
  'AAAAAA/////////4AAAAAAAAAAAP////////8AAAAAAAAAAAB////////+AAAAAAAAAAAAP////////AAAAAAAAAAAAB////' +
  '////gAAAAAAAAAAAAP///////wAAAAAAAAAAAAB///////4AAAAAAAAAAAAAH//////8AAAAAAAAAAAAAA//////8AAAAAAA' +
  'AAAAAAAD/////+AAAAAAAAAAAAAAAf////+AAAAAAAAAAAAAAAA////+AAAAAAAAAAAAAAAAD///8AAAAAAAAAAAAAAAAAH/' +
  '/8AAAAAAAAAAAAAAAAAAB+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── waterdrop — 비트맵 실루엣(128×128, 잉크 42.2%)
//   원본 마스크: waterdrop.png
var MASK_WATERDROP = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAA8AAAAAAAAAAAAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAAH/gAAAAAAAAA' +
  'AAAAAAAAAB/4AAAAAAAAAAAAAAAAAAAf/AAAAAAAAAAAAAAAAAAAP/wAAAAAAAAAAAAAAAAAAD/8AAAAAAAAAAAAAAAAAAB/' +
  '/gAAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAP//AAAAAAAAAAAAAAAAAAD//wAAAAAAAAAAAAAAAAAB//+AAAAAAAAA' +
  'AAAAAAAAAf//gAAAAAAAAAAAAAAAAAP//8AAAAAAAAAAAAAAAAAD///AAAAAAAAAAAAAAAAAB///4AAAAAAAAAAAAAAAAA//' +
  '//AAAAAAAAAAAAAAAAAP///wAAAAAAAAAAAAAAAAH///+AAAAAAAAAAAAAAAAD////wAAAAAAAAAAAAAAAA////+AAAAAAAA' +
  'AAAAAAAAf////gAAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAA///' +
  '///AAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAD///////AAAAAAA' +
  'AAAAAAB///////4AAAAAAAAAAAAAf///////AAAAAAAAAAAAAf///////4AAAAAAAAAAAAH////////AAAAAAAAAAAAD////' +
  '////wAAAAAAAAAAAB////////+AAAAAAAAAAAA/////////wAAAAAAAAAAAf////////+AAAAAAAAAAAP/////////wAAAAA' +
  'AAAAAH/////////+AAAAAAAAAAB//////////gAAAAAAAAAA//////////8AAAAAAAAAAf//////////gAAAAAAAAAP/////' +
  '/////8AAAAAAAAAD///////////AAAAAAAAAB///////////4AAAAAAAAA////////////AAAAAAAAAP///////////4AAAA' +
  'AAAAH///////////+AAAAAAAAD////////////wAAAAAAAA////////////8AAAAAAAAf////////////gAAAAAAAH//////' +
  '//////4AAAAAAAD/////////////AAAAAAAA/////////////wAAAAAAAf////////////+AAAAAAAH/////////////gAAA' +
  'AAAB/////////////4AAAAAAA//////////////AAAAAAAP/////////////wAAAAAAD/////////////+AAAAAAB///////' +
  '///////gAAAAAAf/////////////4AAAAAAH/////////////+AAAAAAB//////////////gAAAAAA//////////////8AAA' +
  'AAAP//////////////AAAAAAD//////////////wAAAAAA//////////////8AAAAAAP//////////////AAAAAAD///////' +
  '///////wAAAAAA//////////////8AAAAAAP//////////////AAAAAAD//////////////wAAAAAA//////////////8AAA' +
  'AAAP//////////////AAAAAAD//////////////wAAAAAA//////////////8AAAAAAP//////////////AAAAAAD///////' +
  '///////wAAAAAA//////////////8AAAAAAH/////////////+AAAAAAB//////////////gAAAAAAf/////////////4AAA' +
  'AAAH/////////////+AAAAAAA//////////////AAAAAAAP/////////////wAAAAAAD/////////////8AAAAAAAf//////' +
  '//////+AAAAAAAH/////////////gAAAAAAA/////////////4AAAAAAAP////////////8AAAAAAAD/////////////AAAA' +
  'AAAAf////////////gAAAAAAAD////////////4AAAAAAAA////////////8AAAAAAAAH///////////+AAAAAAAAB//////' +
  '//////gAAAAAAAAP///////////wAAAAAAAAB///////////4AAAAAAAAAP//////////+AAAAAAAAAB///////////AAAAA' +
  'AAAAAf//////////gAAAAAAAAAD//////////wAAAAAAAAAAf/////////4AAAAAAAAAAB/////////8AAAAAAAAAAAP////' +
  '////+AAAAAAAAAAAB/////////AAAAAAAAAAAAP////////AAAAAAAAAAAAA////////gAAAAAAAAAAAAH///////gAAAAAA' +
  'AAAAAAAf//////wAAAAAAAAAAAAAB//////wAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAAf////gAAAAAAAAAAAAAAAA//' +
  '//AAAAAAAAAAAAAAAAAA//8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── crown — 비트맵 실루엣(128×128, 잉크 39.1%)
//   원본 마스크: crown.png
var MASK_CROWN = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAEAAAAAAQAAAAAEAAAAAAg' +
  'BgAAAAAGAAAAADgAAAAAIAYAAAAADgAAAAA4AAAAAGAHAAAAAA8AAAAAeAAAAADgB4AAAAAPAAAAAHwAAAAA4AOAAAAAH4AA' +
  'AAB8AAAAAeADwAAAAB+AAAAA/gAAAAPAA+AAAAA/wAAAAP4AAAADwAPgAAAAP8AAAAH/AAAAB8AD8AAAAH/gAAAB/wAAAA/A' +
  'A/gAAAB/4AAAA/8AAAAPwAP4AAAA//AAAAP/gAAAH8AB/AAAAP/wAAAH/4AAAD/AAf4AAAH/8AAAB//AAAA/wAH+AAAB//gA' +
  'AA//wAAAf8AB/wAAAf/4AAAP/8AAAP/AAf+AAAP//AAAH//gAAD/gAH/gAAD//wAAB//4AAB/4AB/8AAB//+AAA///AAA/+A' +
  'Af/gAAf//gAAP//wAAP/gAH/4AAP//8AAH//+AAH/4AB//AAD///AAB///gAB/+AAP/4AA///4AA///8AA//gAD/+AAf//+A' +
  'AP///AAf/4AA//wAH///wAD///wAH/8AAP/+AD///8AB///+AD//AAD//gB////gAf///wB//wAA//8Af///4AP///+A//8A' +
  'AP//wf////AH////////AAB////////4D////////wAAf/////////////////8AAH//////////////////AAB/////////' +
  '/////////wAAf/////////////////8AAH/////////////////+AAB//////////////////gAAf/////////////////4A' +
  'AH/////////////////+AAB//////////////////gAAP/////////////////4AAD/////////////////+AAA/////////' +
  '/////////gAAP/////////////////wAAD/////////////////8AAA//////////////////AAAP/////////////////wA' +
  'AD/////////////////8AAA//////////////////AAAP/////////////////wAAD/////////////////8AAA/////////' +
  '/////////AAAP/////////////////wAAD/////////////////8AAA//////////////////AAAP/////////////////wA' +
  'AD/////////////////8AAA//////////////////AAAP/////////////////wAAD/////////////////8AAA/////////' +
  '/////////AAAP/////////////////wAAD/////////////////8AAA//////////////////AAAP/////////////////wA' +
  'AD/////////////////8AAA//////////////////AAAP/////////////////wAAD/////////////////8AAA/////////' +
  '/////////AAAP/////////////////wAAD/////////////////8AAA//////////////////AAAP/////////////////wA' +
  'AB/////////////////4AAAP////////////////+AAAB////////////////+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── bag — 비트맵 실루엣(128×128, 잉크 42.3%)
//   원본 마스크: bag.png
var MASK_BAG = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAH/wAAAAAAAAAAAAAAAAAAf//gAAAAAAAAAAAAAAAAA////AAAAAAAAAAAAAAAAA////8AAAAAAAA' +
  'AAAAAAAA/////wAAAAAAAAAAAAAAAf////+AAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAH///' +
  '///4AAAAAAAAAAAAAD///////AAAAAAAAAAAAAB///////4AAAAAAAAAAAAA///gA///AAAAAAAAAAAAAP//AAD//wAAAAAA' +
  'AAAAAAH//AAAP/+AAAAAAAAAAAAD//AAAA//wAAAAAAAAAAAA//gAAAH/8AAAAAAAAAAAAf/wAAAA//gAAAAAAAAAAAH/4AA' +
  'AAH/4AAAAAAAAAAAD/8AAAAA//AAAAAAAAAAAA/+AAAAAH/wAAAAAAAAAAAf/gAAAAB/+AAAAAAAAAAAH/wAAAAAP/gAAAAA' +
  'AAAAAB/8AAAAAB/4AAAAAAAAAAA/+AAAAAAf/AAAAAAAAAAAP/gAAAAAH/wAAAAAAAAAAD/4AAAAAB/8AAAAAAAAAAA/+AAA' +
  'AAAf/AAAAAAAAAAAf/gAAAAAH/4AAAAAAAAAAH/4AAAAAB/+AAAAAAAAAAB//AAAAAA//gAAAAAAAAAAf/wAAAAAP/4AAAAA' +
  'AAAAAP/+AAAAAH//AAAAAAAAAAH//wAAAAD//4AAAAAAAAAD///AAAAD///AAAAAAAAAB///////////4AAAAAAAAB//////' +
  '//////gAAAAAAAA////////////+AAAAAAAA/////////////wAAAAAAAf////////////+AAAAAAAH/////////////wAAA' +
  'AAAD/////////////8AAAAAAA//////////////gAAAAAAf/////////////4AAAAAAH/////////////+AAAAAAB///////' +
  '///////gAAAAAA//////////////8AAAAAAP//////////////AAAAAAD//////////////wAAAAAA//////////////8AAA' +
  'AAAf//////////////gAAAAAH//////////////4AAAAAB//////////////+AAAAAAf//////////////wAAAAAP///////' +
  '///////8AAAAAD///////////////AAAAAA///////////////wAAAAAf//////////////+AAAAAH///////////////gAA' +
  'AAB///////////////4AAAAAf//////////////+AAAAAP///////////////wAAAAD///////////////8AAAAA////////' +
  '////////AAAAAP///////////////wAAAAH///////////////+AAAAB////////////////gAAAAf///////////////4AA' +
  'AAH///////////////+AAAAD////////////////wAAAA////////////////8AAAAP//wAAAAAAAAAA///AAAAD//gAAAAA' +
  'AAAAAB//wAAAB//wAAAAAAAAAAAP/+AAAAf/8AAAAAAAAAAAD//gAAAH/+AAAAAAAAAAAAf/4AAAB//gAAAAAAAAAAAH//AA' +
  'AA//4AAAAAAAAAAAB//wAAAP/8AAAAAAAAAAAAP/8AAAD//AAAAAAAAAAAAD//AAAA//wAAAAAAAAAAAA//4AAAf/+AAAAAA' +
  'AAAAAAP/+AAAH//gAAAAAAAAAAAH//gAAB//4AAAAAAAAAAAB//4AAA//+AAAAAAAAAAAAf//AAAP//gAAAAAAAAAAAH//wA' +
  'AD//8AAAAAAAAAAAB//8AAA///AAAAAAAAAAAA///AAAP//wAAAAAAAAAAAP//4AAD//+AAAAAAAAAAAH//+AAA///gAAAAA' +
  'AAAAAB///gAAf//8AAAAAAAAAAA///4AAH///AAAAAAAAAAAP//+AAB///4AAAAAAAAAAH///gAAf///AAAAAAAAAAD///4A' +
  'AD///wAAAAAAAAAA///+AAA///+AAAAAAAAAAf///gAAP///wAAAAAAAAAP///wAAD////AAAAAAAAAP///8AAA////4AAAA' +
  'AAAAH////AAAP////gAAAAAAAH////wAAB////+AAAAAAAH////4AAAf////4AAAAAAH////+AAAD/////////////////AA' +
  'AA/////////////////wAAAH////////////////4AAAB////////////////+AAAAP////////////////AAAAB////////' +
  '////////wAAAAf///////////////4AAAAD///////////////8AAAAAf//////////////+AAAAAD///////////////AAA' +
  'AAAf//////////////gAAAAAB//////////////wAAAAAAP/////////////wAAAAAAA/////////////wAAAAAAAD//////' +
  '//////wAAAAAAAAH///////////gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── watch — 비트맵 실루엣(128×128, 잉크 31.6%)
//   원본 마스크: watch.png
var MASK_WATCH = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAB//////8AAAAAAA' +
  'AAAAAAAf//////AAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAf//////AAAAAAAAAAAAAAH///' +
  '///wAAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAf//////AAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAB//////8AAAAAAA' +
  'AAAAAAAf//////AAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAf//////AAAAAAAAAAAAAAH///' +
  '///wAAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAf//////AAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAB//////8AAAAAAA' +
  'AAAAAAAf//////gAAAAAAAAAAAAAP//////8AAAAAAAAAAAAAH///////gAAAAAAAAAAAAB///////4AAAAAAAAAAAAA////' +
  '///+AAAAAAAAAAAAAP///////wAAAAAAAAAAAAD///////8AAAAAAAAAAAAA////////AAAAAAAAAAAAAf///////4AAAAAA' +
  'AAAAAAH//4AB//+AAAAAAAAAAAAD//wAAD//wAAAAAAAAAAAB//wAAAP/+AAAAAAAAAAAA//wAAAA//wAAAAAAAAAAAf/wAA' +
  'AAD/+AAAAAAAAAAAH/4AAAAAf/gAAAAAAAAAAD/8AAAAAD/8AAAAAAAAAAB/+AAP8AAf/gAAAAAAAAAAf/AA//4AD/4AAAAA' +
  'AAAAAP/gA///wAf/AAAAAAAAAAD/wAf//+AD/wAAAAAAAAAB/4Af///4Af+AAAAAAAAAAf+AP////AH/gAAAAAAAAAP/AH//' +
  '//4A/8AAAAAAAAAD/wD/////AP/AAAAAAAAAA/4B/////4B/wAAAAAAAAAf+Af////+Af+AAAAAAAAAH/AP/////wD/gAAAA' +
  'AAAAB/wH/////8A/4AAAAAAAAA/4B//////gH/AAAAAAAAAP+Af/////4B/wAAAAAAAAD/gP/////+Af8AAAAAAAAA/4D///' +
  '///wH/AAAAAAAAAP+A//////8B/wAAAAAAAAD/AP//////AP8AAAAAAAAA/wH//////wD/AAAAAAAAAP8B//////8A/wAAAA' +
  'AAAAD/Af//////AP+AAAAAAAAA/wH//////wD/AAAAAAAAAP8B//////8A/wAAAAAAAAD/AP//////AP8AAAAAAAAA/4D///' +
  '///wD/AAAAAAAAAP+A//////8B/wAAAAAAAAD/gP//////Af8AAAAAAAAA/4B//////gH/AAAAAAAAAP+Af/////4B/wAAAA' +
  'AAAAB/wH/////+A/4AAAAAAAAAf8A//////AP+AAAAAAAAAH/AP/////gH/gAAAAAAAAA/4B/////4B/4AAAAAAAAAP+AP//' +
  '//8A/8AAAAAAAAAD/wB////+AP/AAAAAAAAAAf8AP////AH/gAAAAAAAAAH/gB////gB/4AAAAAAAAAB/8AP///wA/8AAAAA' +
  'AAAAAP/gA///wAf/AAAAAAAAAAB/4AH//4AP/gAAAAAAAAAAf/AAP/wAH/4AAAAAAAAAAD/4AAAAAD/8AAAAAAAAAAAf/gAA' +
  'AAB/+AAAAAAAAAAAH/8AAAAA//gAAAAAAAAAAA//gAAAA//wAAAAAAAAAAAH/+AAAAf/4AAAAAAAAAAAA//4AAAf/8AAAAAA' +
  'AAAAAAH//wAA//+AAAAAAAAAAAAB///4H///gAAAAAAAAAAAAP///////wAAAAAAAAAAAAD///////8AAAAAAAAAAAAA////' +
  '////AAAAAAAAAAAAAP///////wAAAAAAAAAAAAB///////4AAAAAAAAAAAAAf//////+AAAAAAAAAAAAAH///////gAAAAAA' +
  'AAAAAAA///////wAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAf//////AAAAAAAAAAAAAAH///' +
  '///wAAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAf//////AAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAB//////8AAAAAAA' +
  'AAAAAAAf//////AAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAf//////AAAAAAAAAAAAAAH///' +
  '///wAAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAf//////AAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAB//////8AAAAAAA' +
  'AAAAAAAf//////AAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAf//////AAAAAAAAAAAAAAD///' +
  '///wAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── ring — 비트맵 실루엣(128×128, 잉크 25.9%)
//   원본 마스크: ring.png
var MASK_RING = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAB/////AAAAAAAAAAAAAAAA/////wAAAAAAAAAAAAAAAf////+AAAAAAAAAAAAAAAP/////wAAAAAAA' +
  'AAAAAAAD/////+AAAAAAAAAAAAAAB//gB//gAAAAAAAAAAAAAA/n4Afj8AAAAAAAAAAAAAAPw+AD4/gAAAAAAAAAAAAAH4PA' +
  'A+H8AAAAAAAAAAAAAD+HwAPg/AAAAAAAAAAAAAB/h///8f4AAAAAAAAAAAAAf//////+AAAAAAAAAAAAAH///////gAAAAAA' +
  'AAAAAAA///////4AAAAAAAAAAAAAP//////8AAAAAAAAAAAAAB//4Af/+AAAAAAAAAAAAAAf/8AD//gAAAAAAAAAAAAAD//A' +
  'A//wAAAAAAAAAAAAAA//wAP/8AAAAAAAAAAAAAAH/+AH/+AAAAAAAAAAAAAAA//wD//gAAAAAAAAAAAAAAP/8A//wAAAAAAA' +
  'AAAAAAAB//gf/8AAAAAAAAAAAAAAAf/8H/+AAAAAAAAAAAAAAAH//D//gAAAAAAAAAAAAAAH//4//+AAAAAAAAAAAAAAH///' +
  '///4AAAAAAAAAAAAAD///////gAAAAAAAAAAAAD///////+AAAAAAAAAAAAD////////wAAAAAAAAAAAB/////////AAAAAA' +
  'AAAAAB/////////4AAAAAAAAAAA//////////AAAAAAAAAAAf/////////4AAAAAAAAAAP//////////gAAAAAAAAAH/////' +
  '/////8AAAAAAAAAD////AAD////gAAAAAAAAB///+AAAD///8AAAAAAAAA///+AAAAP///AAAAAAAAAf//8AAAAA///4AAAA' +
  'AAAAP//8AAAAAD///AAAAAAAAH//+AAAAAAf//4AAAAAAAB//+AAAAAAB///AAAAAAAA///AAAAAAAP//wAAAAAAAf//gAAA' +
  'AAAB//+AAAAAAAH//wAAAAAAAP//gAAAAAAD//4AAAAAAAB//8AAAAAAA//8AAAAAAAAP//gAAAAAAf/+AAAAAAAAB//4AAA' +
  'AAAP//AAAAAAAAAP//AAAAAAD//wAAAAAAAAD//wAAAAAA//4AAAAAAAAAf/8AAAAAAf/8AAAAAAAAAD//gAAAAAH//AAAAA' +
  'AAAAA//4AAAAAB//gAAAAAAAAAH/+AAAAAA//4AAAAAAAAAB//wAAAAAP/8AAAAAAAAAAP/8AAAAAD//AAAAAAAAAAD//AAA' +
  'AAB//gAAAAAAAAAAf/4AAAAAf/4AAAAAAAAAAH/+AAAAAH/+AAAAAAAAAAB//gAAAAB//gAAAAAAAAAAP/4AAAAAf/wAAAAA' +
  'AAAAAD/+AAAAAP/8AAAAAAAAAAA//wAAAAD//AAAAAAAAAAAP/8AAAAA//wAAAAAAAAAAD//AAAAAP/8AAAAAAAAAAA//wAA' +
  'AAD//AAAAAAAAAAAH/8AAAAA//wAAAAAAAAAAB//AAAAAP/8AAAAAAAAAAAf/wAAAAD//AAAAAAAAAAAH/8AAAAA//wAAAAA' +
  'AAAAAB//AAAAAP/8AAAAAAAAAAAf/wAAAAD//AAAAAAAAAAAP/8AAAAA//wAAAAAAAAAAD//AAAAAP/8AAAAAAAAAAA//wAA' +
  'AAD//AAAAAAAAAAAP/8AAAAAf/wAAAAAAAAAAD/+AAAAAH/+AAAAAAAAAAA//gAAAAB//gAAAAAAAAAAf/4AAAAAf/4AAAAA' +
  'AAAAAH/+AAAAAH//AAAAAAAAAAB//gAAAAA//wAAAAAAAAAA//wAAAAAP/8AAAAAAAAAAP/8AAAAAD//gAAAAAAAAAH//AAA' +
  'AAAf/4AAAAAAAAAB//gAAAAAH//AAAAAAAAAA//4AAAAAB//wAAAAAAAAAP/+AAAAAAP/+AAAAAAAAAH//AAAAAAD//wAAAA' +
  'AAAAB//wAAAAAAf/8AAAAAAAAA//8AAAAAAH//gAAAAAAAAf/+AAAAAAA//8AAAAAAAAP//gAAAAAAP//gAAAAAAAH//wAAA' +
  'AAAB//8AAAAAAAD//4AAAAAAAf//gAAAAAAB//+AAAAAAAD//8AAAAAAA///AAAAAAAAf//gAAAAAAf//gAAAAAAAH//+AAA' +
  'AAAP//4AAAAAAAA///wAAAAAP//8AAAAAAAAH///AAAAAP//+AAAAAAAAA///8AAAAP///AAAAAAAAAH///wAAAP///gAAAA' +
  'AAAAA////gAAf///wAAAAAAAAAH////8P////4AAAAAAAAAA//////////8AAAAAAAAAAH/////////+AAAAAAAAAAA/////' +
  '/////AAAAAAAAAAAH/////////gAAAAAAAAAAAf////////gAAAAAAAAAAAB////////wAAAAAAAAAAAAP///////wAAAAAA' +
  'AAAAAAA///////4AAAAAAAAAAAAAD//////4AAAAAAAAAAAAAAP/////4AAAAAAAAAAAAAAAf////wAAAAAAAAAAAAAAAA//' +
  '//AAAAAAAAAAAAAAAAAAf/8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── perfume — 비트맵 실루엣(128×128, 잉크 36.6%)
//   원본 마스크: perfume.png
var MASK_PERFUME = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAf//////+AAAAAAAAAAAAAP///////wAAAAAA' +
  'AAAAAAH///////+AAAAAAAAAAAAB////////wAAAAAAAAAAAA////////8AAAAAAAAAAAAP////////AAAAAAAAAAAAH////' +
  '////4AAAAAAAAAAAB////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAH////////4AAAAAAAAAAAB////////+AAAAAA' +
  'AAAAAAf////////gAAAAAAAAAAAH////////4AAAAAAAAAAAB////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAH////' +
  '////4AAAAAAAAAAAB////////+AAAAAAAAAAAAf////////gAAAAAAAAAAAH////////4AAAAAAAAAAAB////////+AAAAAA' +
  'AAAAAAf////////gAAAAAAAAAAAD////////4AAAAAAAAAAAA////////8AAAAAAAAAAAAP////////AAAAAAAAAAAAB////' +
  '////gAAAAAAAAAAAAP///////wAAAAAAAAAAAAB///////8AAAAAAAAAAAAAf//////+AAAAAAAAAAAAAH///////gAAAAAA' +
  'AAAAAAB///////4AAAAAAAAAAAAAf//////+AAAAAAAAAAAAAH///////gAAAAAAAAAAAAB///////4AAAAAAAAAAAAAf///' +
  '///+AAAAAAAAAAAAAH///////gAAAAAAAAAAAAD///////8AAAAAAAAAAAAA////////AAAAAAAAAAAAAf///////4AAAAAA' +
  'AAAAAAf////////gAAAAAAAAAAA//////////AAAAAAAAAAH///////////gAAAAAAAAH///////////+AAAAAAAAH//////' +
  '//////4AAAAAAAH/////////////gAAAAAAD/////////////8AAAAAAD//////////////wAAAAAB//////////////+AAA' +
  'AAA///////////////wAAAAAf//////////////+AAAAAH///////////////gAAAAD///////////////8AAAAB////////' +
  '////////gAAAAf///gAAAAAAAH///4AAAAP//+AAAAAAAAAH///AAAAD//+AAAAAAAAAAf//wAAAA///AAAAAAAAAAD//+AA' +
  'AAf//gAAAAAAAAAAf//gAAAH//wAAAAAAAAAAD//4AAAB//4AAAAAAAAAAAf/+AAAA//+AAAAAAAAAAAD//wAAAP//AAAAAA' +
  'AAAAAA//8AAAD//wAAAAAAAAAAAP//AAAA//8AAAAAAAAAAAD//wAAAP/+AAAAAAAAAAAAf/8AAAD//gAAAAAAAAAAAH//AA' +
  'AA//4AAAAAAAAAAAB//wAAAP/+AAAAAAAAAAAAf/8AAAD//gAAAAAAAAAAAH//AAAA//4AAAAAAAAAAAB//wAAAP/+AAAAAA' +
  'AAAAAAf/8AAAD//gAAAAAAAAAAAH//AAAA//4AAAAAAAAAAAB//wAAAP/+AAAAAAAAAAAAf/8AAAD//gAAAAAAAAAAAH//AA' +
  'AA//4AAAAAAAAAAAB//wAAAP/+AAAAAAAAAAAAf/8AAAD//gAAAAAAAAAAAH//AAAA//4AAAAAAAAAAAB//wAAAP/+AAAAAA' +
  'AAAAAAf/8AAAD//gAAAAAAAAAAAH//AAAA//4AAAAAAAAAAAB//wAAAP/+AAAAAAAAAAAAf/8AAAD//gAAAAAAAAAAAH//AA' +
  'AA//4AAAAAAAAAAAB//wAAAP/+AAAAAAAAAAAAf/8AAAD//gAAAAAAAAAAAH//AAAA//4AAAAAAAAAAAB//wAAAP/+AAAAAA' +
  'AAAAAAf/8AAAD//gAAAAAAAAAAAH//AAAA//4AAAAAAAAAAAB//wAAAP/+AAAAAAAAAAAAf/8AAAD//gAAAAAAAAAAAH//AA' +
  'AA//8AAAAAAAAAAAD//wAAAP//AAAAAAAAAAAA//8AAAD//wAAAAAAAAAAAP//AAAA//+AAAAAAAAAAAH//wAAAH//wAAAAA' +
  'AAAAAB//4AAAB//+AAAAAAAAAAA//+AAAAf//gAAAAAAAAAAf//gAAAH//+AAAAAAAAAAf//4AAAA///4AAAAAAAAAf//8AA' +
  'AAP///gAAAAAAAAf///AAAAB////////////////gAAAAf///////////////4AAAAD///////////////8AAAAA////////' +
  '////////AAAAAH///////////////gAAAAA///////////////wAAAAAH//////////////4AAAAAA//////////////8AAA' +
  'AAAH/////////////+AAAAAAA//////////////AAAAAAAD/////////////AAAAAAAAP////////////AAAAAAAAAf/////' +
  '/////+AAAAAAAAAAf/////////4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── helmet — 비트맵 실루엣(128×128, 잉크 50.3%)
//   원본 마스크: helmet.png
var MASK_HELMET = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA/wAAAAAAAAAAAAAAAAAAH//4AAAAAAAAAAAAAAAAAD///AAAAAAAAA' +
  'AAAAAAAAA///4AAAAAAAAAAAAAAAAAf//+AAAAAAAAAAAAAAAAAP///wAAAAAAAAAAAAAAAAH///+AAAAAAAAAAAAAAAAf//' +
  '//+AAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAf//////+AAAAAAAAAAAAAf///////4AAAAAA' +
  'AAAAAAH////////AAAAAAAAAAAAD////////wAAAAAAAAAAAB////////+AAAAAAAAAAAB/////////4AAAAAAAAAAA/////' +
  '/////gAAAAAAAAAA//////////8AAAAAAAAAAf//////////gAAAAAAAAAP//////////8AAAAAAAAAH///////////wAAAA' +
  'AAAAH///////////+AAAAAAAAD////////////wAAAAAAAA////////////+AAAAAAAAf////////////wAAAAAAAP//////' +
  '//////+AAAAAAAH/////////////gAAAAAAD/////////////8AAAAAAB//////////////gAAAAAAf/////////////8AAA' +
  'AAAP//////////////AAAAAAH//////////////4AAAAAB//////////////+AAAAAA///////////////wAAAAAP///////' +
  '///////8AAAAAH///////////////gAAAAB///////////////4AAAAA////////////////AAAAAP///////////////wAA' +
  'AAD///////////////+AAAAB////////////////gAAAAf///////////////4AAAAP////////////////AAAAD////////' +
  '////////wAAAA////////////////8AAAAP////////////////AAAAH////////////////4AAAB////////////////+AA' +
  'AAf////////////////gAAAH////////////////4AAAB////////////////+AAAAf////////////////wAAAH////////' +
  '////////8AAAB/////////////////AAAA/////////////////wAAAP////////////////8AAAD/////////////////AA' +
  'AA/////////////////wAAAP////////////////8AAAD/////////////////AAAB/////////////////4AAAf////////' +
  '////////+AAAP/////////////////wAAH/////////////////+AAD//////////////////wAA//////////////////8A' +
  'AP//////////////////AAD//////////////////wAB//////////////////+AAf//////////////////gAP/////////' +
  '/////////8AH///////////////////gB///////////////////4Af//////////////////+AD///////////////////A' +
  'A///////////////////wAH//////////////////4AAf/////////////////4AAD/////////////////8AAAf////////' +
  '////////+AAAD/////////////////AAAAf////////////////gAAAD////////////////wAAAAP///////////////4AA' +
  'AAB///////////////4AAAAAP//////////////8AAAAAB//////////////+AAAAAAP//////////////AAAAAAA///////' +
  '///////AAAAAAAH/////////////gAAAAAAAf////////////wAAAAAAAD////////////wAAAAAAAAP///////////wAAAA' +
  'AAAAA///////////wAAAAAAAAAA//////////AAAAAAAAAAAB////////+AAAAAAAAAAAAAf//////gAAAAAAAAAAAAAAAP/' +
  '/8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── fan — 비트맵 실루엣(128×128, 잉크 37.6%)
//   원본 마스크: fan.png
var MASK_FAN = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAP/wAAAAAAAAAAAAAAAAAD///8AAAAAAAAAAAAAAAAH////4AAAAAAAAAAAAAAAH/////gAAAAAAA' +
  'AAAAAAAD/////8AAAAAAAAAAAAAAA//////gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB///' +
  '///gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAA//////AAAAAAAAAAAAAAAP/////wAAAAAAA' +
  'AAAAAAAD/////8AAAAAAAAAAAAAAA/////+AAAAAAAAAAAAAAAP/////gAAAAAAAAAAAAAAD/////4AAAAAAAAAAAAAAAf//' +
  '//8AAAAAAAAAAAAAAAH/////AAAAAAAAAAAAAAAB/////wAAAAAAAAAAAAAAAf////4AAAAAAAAAAAAAAAD////+AAAAAAAA' +
  'AAAAAAAA/////gAAAAAAAAAAAAAAAP////wAAAAAAAAAAAAAAAB////8AAAAAAAAAAAAAAAAf////AAAAAAAAAAAAAAAAH//' +
  '//gAAAAAAAAAAAAAAAA////4AAAAAAAAAAAAAAAAP///+AAAAAAAAAAAAAAAAD////AAAAAAAAAAAAAAAAAf///wAAAAAAAA' +
  'AAAAAAAAH///8AAAAAAAAAAAAAAAAB///+AAAAAAAAAAAAAAAAAP///gAAAAAAAAAAAAAAAAD///4AAAAAAAAAAAAAAAAAf/' +
  '/8AAAAAAAAAAAAAAAAAH///AAAAAAAAAAAAAAAAAA///wAAAAAAAAAAGAAAAAAP//8AAAAAf/AAAH+AAAAAB//+AAAAB//8A' +
  'AD/8AAAAAf//gAAAH///AAB//wAAAAD//4AAAP///4AAf//gAAAA//+AAAf///+AAP//+AAAA///wAAf////wAD///8AAAf/' +
  '/+AAf////8AB////4AAP///4A//////AAf////wAH////A//////wAH/////AD////wf/////8AB//////B////+f//////g' +
  'A///////////////////4AP//////////////////+AD///////////////////gA///////////////////4AP/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////wAf//////////////////8AH///////////////////A' +
  'B////////////z//////wAP//////n////8D/////8AD//////g////+AH////+AA//////gP////AAP////gAP/////gB//' +
  '//gAA////4AD/////gAP///wAAB///8AAf////AAA///4AAAH///AAH////AAAH//4AAAAP//wAA///+AAAB//+AAAAA//4A' +
  'AP//4AAAAf//gAAAAB/8AAB//gAAAAH//8AAAAAH+AAAHwAAAAAB///gAAAAAHAAAAAAAAAAA///4AAAAAAAAAAAAAAAAAP/' +
  '//AAAAAAAAAAAAAAAAAD///wAAAAAAAAAAAAAAAAA///+AAAAAAAAAAAAAAAAAf///gAAAAAAAAAAAAAAAAH///8AAAAAAAA' +
  'AAAAAAAAB////AAAAAAAAAAAAAAAAA////4AAAAAAAAAAAAAAAAP///+AAAAAAAAAAAAAAAAD////gAAAAAAAAAAAAAAAB//' +
  '//8AAAAAAAAAAAAAAAAf////AAAAAAAAAAAAAAAAH////wAAAAAAAAAAAAAAAD////+AAAAAAAAAAAAAAAA/////gAAAAAAA' +
  'AAAAAAAAP////4AAAAAAAAAAAAAAAH/////AAAAAAAAAAAAAAAB/////wAAAAAAAAAAAAAAAf////8AAAAAAAAAAAAAAAP//' +
  '///AAAAAAAAAAAAAAAD/////4AAAAAAAAAAAAAAB/////+AAAAAAAAAAAAAAAf/////gAAAAAAAAAAAAAAH/////4AAAAAAA' +
  'AAAAAAAD/////+AAAAAAAAAAAAAAA//////gAAAAAAAAAAAAAAP/////8AAAAAAAAAAAAAAH//////AAAAAAAAAAAAAAB///' +
  '///wAAAAAAAAAAAAAAf/////8AAAAAAAAAAAAAAH//////AAAAAAAAAAAAAAB//////wAAAAAAAAAAAAAAf/////8AAAAAAA' +
  'AAAAAAAD/////+AAAAAAAAAAAAAAAf/////gAAAAAAAAAAAAAAB/////gAAAAAAAAAAAAAAAH////wAAAAAAAAAAAAAAAAP/' +
  '//gAAAAAAAAAAAAAAAAAD/4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── goggles — 비트맵 실루엣(128×128, 잉크 16.8%)
//   원본 마스크: goggles.png
var MASK_GOGGLES = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAB/////4AAAAAAAAAAAAAD////' +
  '////4AAAAAAAAAAA//////////+AAAAAAAAAH///////////+AAAAAAAAP////////////+AAAAAAA//////////////8AAA' +
  'AAAf//////////////gAAAAAf//////////////+AAAAAP///////////////wAAAAH///////////////+AAAAB////+AAA' +
  'AAAP////wAAAA///4AAAAAAAAAf//8AAAAP//AAAAAAAAAAAH//gAAAD/4AAAAAAAAAAAAH/4AAAB/4AAAAAAAAAAAAAP+AA' +
  'AAf4AAAAAAAAAAAAAB/gAAAH8AAAAAAAAAAAAAAP8AAAD/AAAAAAAAAAAAAAB/AAAB/gAAAAAAAAAAAAAAf8AAP/4AAAAAAA' +
  'AAAAAAAH/+AH/+AAAAAAAAAAAAAAA//gB//gAAAAAAAAAAAAAAP/4Af/4AAAAAAAAAAAAAAD/+AH/+AAAAAAAAAAAAAAA//g' +
  'B//gAAAAAAAAAAAAAAP/4Af/4AAAAAAAAAAAAAAD/+AH/+AAAAAAAAAAAAAAA//gB//gAAAAAAAAAAAAAAP/4Af/4AAAAAAA' +
  'AAAAAAAD/+AH/+AAAAAAAAAAAAAAA//gB//gAAAAAAAAAAAAAAP/4Af/4AAAAAAAAAAAAAAD/+AH/+AAAAAAAAAAAAAAA//g' +
  'B//gAAAAAAAAAAAAAAP/4Af/4AAAAAAAAAAAAAAD/+AH/+AAAAAAA8AAAAAAA//gB//gAAAAAA/4AAAAAAP/4Af/4AAAAAAf' +
  '/AAAAAAD/+AH/+AAAAAAP/4AAAAAA//gB//gAAAAAH//AAAAAAP/4Af/4AAAAAD//wAAAAAD/+AD/+AAAAAA//+AAAAAB//A' +
  'AA/gAAAAAf//gAAAAAfwAAAP4AAAAAH//8AAAAAH8AAAB+AAAAAD/D/AAAAAB/AAAAfgAAAAA/gf4AAAAAfgAAAH8AAAAAf4' +
  'D+AAAAAH4AAAB/AAAAAH8A/wAAAAD+AAAAf4AAAAD/AH8AAAAA/gAAAD+AAAAA/gB/AAAAAf4AAAA/wAAAAf4AP4AAAAP8AA' +
  'AAP+AAAAH8AD+AAAAD/AAAAB/wAAAB/AAfwAAAD/wAAAAf/AAAA/wAH8AAAB/4AAAAD/4AAAP4AB/gAAB/+AAAAA//gAAH+A' +
  'AP4AAB//AAAAAH/+AAD/AAD/AAB//gAAAAA//8AA/wAAf4AD//wAAAAAH//8A/4AAH/AP//4AAAAAA/////+AAA/////8AAA' +
  'AAAH/////AAAP////+AAAAAAAf////wAAB/////AAAAAAAD////4AAAf////AAAAAAAAP///8AAAD////AAAAAAAAA////AA' +
  'AAf///AAAAAAAAAB///gAAAD///AAAAAAAAAAH//gAAAAf/+AAAAAAAAAAAH/wAAAAB/4AAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── harness — 비트맵 실루엣(128×128, 잉크 29.6%)
//   원본 마스크: harness.png
var MASK_HARNESS = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAA/AAAAAAH8AAAAAAAAAAAH/4AAAAAB/+AAAAAAAAAAD//AAAAAA//gAAAAAAAAAA//wAAAAAf/4AAAAA' +
  'AAAAAP/+AAAAAH/+AAAAAAAAAAD//wAAAAD//gAAAAAAAAAA//8AAAAB//4AAAAAAAAAAP//gAAAAf/+AAAAAAAAAAD//8AA' +
  'AAP//gAAAAAAAAAA///AAAAH//4AAAAAAAAAAP//4AAAB//+AAAAAAAAAAD//+AAAA///gAAAAAAAAAA///wAAAf//4AAAAA' +
  'AAAAAP//+AAAH//+AAAAAAAAAAD///gAAD///gAAAAAAAAAAf//8AAB///4AAAAAAAAAAH///gAAf//+AAAAAAAAAAB///4A' +
  'AP///gAAAAAAAAAAf+f/AAH/7/wAAAAAAAAAAH/3/4AB/8/8AAAAAAAAAAB/8/+AA/+P/AAAAAAAAAAAf/H/wAf/j/wAAAAA' +
  'AAAAAD/x/+AH/x/8AAAAAAAAAAA/8P/gD/4f/AAAAAAAAAAAP/B/8A/+H/gAAAAAAAAAAD/wf/gf/B/4AAAAAAAAAAA/+D//' +
  '//wf+AAAAAAAAAAAH/g////4H/gAAAAAAAAAAB/4H///+D/4AAAAAAAAAAAf+A////A/8AAAAAAAAAAAH/wP///gP/AAAAAA' +
  'AAAAAA/8B///4D/wAAAAAAAAAAAP/A/gB/B/8AAAAAAAAAAAD/wPwAPwf+AAAAAAAAAAAA/+D8AD8H/gAAAAAAAAAAAH/g/A' +
  'A/B/4AAAAAAAAAAAB/4PwAPw/+AAAAAAAAAAAAf+D8AD8P/AAAAAAAAAAAAH/w/AA/D/wAAAAAAAAAAAA/8PwAPw/8AAAAAA' +
  'AAAAAAP/D8AD8f+AAAAAAAAAAAAD/4/AA/H/gAAAAAAAAAAAAf+P4Afx/4AAAAAAAAAAAAH/n///+f+AAAAAAAAAAAAB/7//' +
  '//n/AAAAAAAAAAAAAf9////8/wAAAAAAAAAAAAD+f////n4AAAAAAAAAAAAA/P////8+AAAAAAAAAAAAAP3/////vgAAAAAA' +
  'AAAAAAB5/////5wAAAAAAAAAAAAAc//////MAAAAAAAAAAAAACf/n/n/7AAAAAAAAAAAAAAv/x/5//AAAAAAAAAAAAAAD/4f' +
  '+P/wAAAAAAAAAAAAAB/+H/h/+AAAAAAAAAAAAAA//B/4P/wAAAAAAAAAAAAAf/gf+D/+AAAAAAAAAAAAAH/wH/gf/gAAAAAA' +
  'AAAAAAD/4B/4D/8AAAAAAAAAAAAB/+Af+Af/gAAAAAAAAAAAAf/AH/gH/4AAAAAAAAAAAAP/gB/4A//AAAAAAAAAAAAH/4Af' +
  '+AH/4AAAAAAAAAAAB/8AH/gA//AAAAAAAAAAAA/+AB/4AP/wAAAAAAAAAAAf/gAf+AB/+AAAAAAAAAAAH/wAH/gAP/wAAAAA' +
  'AAAAAD/4AAAAAB/8AAAAAAAAAAA/8AAAAAAf/AAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAA////v//f///wAAAAAAAAAP////g' +
  'B3///8AAAAAAAAAD////7/d////AAAAAAAAAA///////f///wAAAAAAAAAP//////3///8AAAAAAAAAD//////9////AAAAA' +
  'AAAAA///////f///wAAAAAAAAAP////v93///8AAAAAAAAAD////4Ad////AAAAAAAAAAf///v/+f///wAAAAAAAAAH/gAB/' +
  '/gAB/4AAAAAAAAAB/4AAP/gAAf+AAAAAAAAAA/8AAB/4AAD/wAAAAAAAAAP/AAAf+AAA/8AAAAAAAAAD/gAAH/gAAP/AAAAA' +
  'AAAAB/4AAB/4AAB/4AAAAAAAAAf+AAAf+AAAf+AAAAAAAAAP/AAAH/gAAD/wAAAAAAAAD/wAAB/4AAA/8AAAAAAAAA/4AAAf' +
  '+AAAP/AAAAAAAAAf+AAAH/gAAB/4AAAAAAAAH/gAAB/4AAAf+AAAAAAAAD/8AAAf+AAAP/wAAAAAAAA//wAAH/gAAf/8AAAA' +
  'AAAAP//gAB/4AAf//gAAAAAAAH//+AAf+AAf//4AAAAAAAB///4AH/gAf//+AAAAAAAAf///gB/4Af///wAAAAAAAP///+Af' +
  '+Af///8AAAAAAAD////wH/gP////AAAAAAAA/z///B/4P//8/wAAAAAAAP4H//4f+H//4H8AAAAAAAD+Af//H/j//4B/AAAA' +
  'AAAA/AA//9/5//4APwAAAAAAAH4AD//////wAH4AAAAAAAB/AAP/////wAB+AAAAAAAAfwAA//AP/4AA/gAAAAAAAD/AAH/w' +
  'D/4AA/wAAAAAAAA/4AAf8A/4AAf8AAAAAAAAH/wAD/AP8AA/+AAAAAAAAA//gAfwD/AB//AAAAAAAAAP////8A/////wAAAA' +
  'AAAAA/////AP////4AAAAAAAAAH////wD////4AAAAAAAAAAf///4Af///8AAAAAAAAAAD///+AH///8AAAAAAAAAAAH//+A' +
  'Af//4AAAAAAAAAAAAH/8AAA//gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── extinguisher — 비트맵 실루엣(128×128, 잉크 36.8%)
//   원본 마스크: extinguisher.png
var MASK_EXTINGUISHER = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAA//+AAAAAAAAAAAAAAAAAB///wAAAAAAAAAAAAAAAAD///8AAAA' +
  'AAAAAAAAAAAAH///+AAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAf///+AAAAAAAAAAAAAAAA////8AAAAAAAAAAAAAAAB' +
  '////wAAAAAAAAAAAAAAAA////gAAAAAAAAAAAAAAAAf///AAAAAAAAAAAAAAAAAH//+AAAAAAAAAAAAAAAAAB//+AAAAAAAA' +
  'AAAAAAAAAAf//gAAAAAAAAAAAAAAAAAH///AAAAAAAAAAAAAAAAP5////AAAAAAAAAAAAAAP//f///+AAAAAAAAAAAAA///3' +
  '////8AAAAAAAAAAAA///9/////4AAAAAAAAAAA////f/////4AAAAAAAAAAf///3//////wAAAAAAAAAP///9//4///+AAAA' +
  'AAAAAP////f/+B///wAAAAAAAAH////3//gB//8AAAAAAAAD//wH///8AD//AAAAAAAAA//gAA///gAH/wAAAAAAAAf/gAAP' +
  '//4AAP8AAAAAAAAP/gAAD//+AAAfAAAAAAAAD/wAAA///gAAAAAAAAAAAB/4AAA///8AAAAAAAAAAAAf8AAA////wAAAAAAA' +
  'AAAAP+AAA/////AAAAAAAAAAAD/gAA/////8AAAAAAAAAAB/wAAf/////gAAAAAAAAAAf8AAP/////8AAAAAAAAAAH+AAH//' +
  '////gAAAAAAAAAB/gAD//////8AAAAAAAAAA/4AA///////gAAAAAAAAAP8AAf//////4AAAAAAAAAD/AAP///////AAAAAA' +
  'AAAA/wAD///////wAAAAAAAAAP8AA///////+AAAAAAAAAD/AAf///////gAAAAAAAAA/wAH///////4AAAAAAAAAP8AB///' +
  '////+AAAAAAAAAD/AAf///////wAAAAAAAAA/wAP///////8AAAAAAAAAP8AD////////AAAAAAAAAD/AA////////wAAAAA' +
  'AAAA/wAP///////8AAAAAAAAAP+AD////////AAAAAAAAAD/gA////////wAAAAAAAAAf4AP///////8AAAAAAAAAH+AD///' +
  '/////AAAAAAAAAB/wA////////wAAAAAAAAAf8AP///////8AAAAAAAAAD/AD////////AAAAAAAAAA/4A////////wAAAAA' +
  'AAAAP+AP///////8AAAAAAAAAB/gD////////AAAAAAAAAAf8A////////wAAAAAAAAAH/AP///////8AAAAAAAAAA/wD///' +
  '/////AAAAAAAAAAP8A////////wAAAAAAAAAD/gP///////8AAAAAAAAAA/4D////////AAAAAAAAAAH+A////////wAAAAA' +
  'AAAAB/gP///////8AAAAAAAAAAf4D////////AAAAAAAAAAH/A////////wAAAAAAAAAB/wP///////8AAAAAAAAAAf8D///' +
  '/////AAAAAAAAAAH/A////////wAAAAAAAAAB/wP///////8AAAAAAAAAAf8D////////AAAAAAAAAAH/A////////wAAAAA' +
  'AAAAB/wP///////8AAAAAAAAAAf8D////////AAAAAAAAAAD/A////////wAAAAAAAAAA/wP///////8AAAAAAAAAAf+D///' +
  '/////AAAAAAAAAAH/g////////wAAAAAAAAAB/4P///////8AAAAAAAAAAf+D////////AAAAAAAAAAH/g////////wAAAAA' +
  'AAAAB/4P///////8AAAAAAAAAAf8D////////AAAAAAAAAAH/g////////wAAAAAAAAAD/4P///////8AAAAAAAAAA/+D///' +
  '/////AAAAAAAAAAP/g////////wAAAAAAAAAD/8P///////8AAAAAAAAAB//D////////AAAAAAAAAAf/w////////wAAAAA' +
  'AAAAH/8P///////8AAAAAAAAAB//D////////AAAAAAAAAAf/w////////wAAAAAAAAAH/8P///////8AAAAAAAAAB//D///' +
  '/////AAAAAAAAAAf/wAAAAAAAAAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAAD//h///////+AAAAAAAAAA//4////////wAAAAA' +
  'AAAAP/+P///////8AAAAAAAAAD//j////////AAAAAAAAAA//4////////wAAAAAAAAAP/+P///////8AAAAAAAAAD//h///' +
  '/////AAAAAAAAAA//4f///////wAAAAAAAAAf/+H///////4AAAAAAAAAH//h///////+AAAAAAAAAB//4P///////gAAAAA' +
  'AAAAP/+D///////wAAAAAAAAAAAAAf//////4AAAAAAAAAAAAAD//////+AAAAAAAAAAAAAAf//////AAAAAAAAAAAAAAD//' +
  '////AAAAAAAAAAAAAAAP/////AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── safetycone — 비트맵 실루엣(128×128, 잉크 36.2%)
//   원본 마스크: safetycone.png
var MASK_SAFETYCONE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAD+AAAAAAAAAAAAAAAAAAAD/4AAAAAAAAAAAAAAAAAAB//AAAAAAAAAAAAAAAAAAAf/4AAAAAAAAA' +
  'AAAAAAAAAP/+AAAAAAAAAAAAAAAAAAD//wAAAAAAAAAAAAAAAAAA//8AAAAAAAAAAAAAAAAAAf//AAAAAAAAAAAAAAAAAAH/' +
  '/wAAAAAAAAAAAAAAAAAB//+AAAAAAAAAAAAAAAAAA///gAAAAAAAAAAAAAAAAAP//4AAAAAAAAAAAAAAAAAD///AAAAAAAAA' +
  'AAAAAAAAA///wAAAAAAAAAAAAAAAAAf//8AAAAAAAAAAAAAAAAAH///AAAAAAAAAAAAAAAAAB///4AAAAAAAAAAAAAAAAAf/' +
  '/+AAAAAAAAAAAAAAAAAP///gAAAAAAAAAAAAAAAAD///8AAAAAAAAAAAAAAAAA////AAAAAAAAAAAAAAAAAf///wAAAAAAAA' +
  'AAAAAAAAH///8AAAAAAAAAAAAAAAAB////gAAAAAAAAAAAAAAAAf///4AAAAAAAAAAAAAAAAP///+AAAAAAAAAAAAAAAAD//' +
  '//wAAAAAAAAAAAAAAAA////8AAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAH////wAAAAAAAAAAAAAAAB////+AAAAAAAA' +
  'AAAAAAAAf////gAAAAAAAAAAAAAAAH////4AAAAAAAAAAAAAAAD/////AAAAAAAAAAAAAAAA/////wAAAAAAAAAAAAAAAP//' +
  '//8AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAB/////4AAAAAAAAAAAAAAAf////+AAAAAAAAAAAAAAAH/////gAAAAAAA' +
  'AAAAAAAD/////8AAAAAAAAAAAAAAA//////AAAAAAAAAAAAAAAP/////wAAAAAAAAAAAAAAD/////+AAAAAAAAAAAAAAB///' +
  '///gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB//////wAAAAAAAAAAAAAA//////8AAAAAAA' +
  'AAAAAAAP//////AAAAAAAAAAAAAAD//////wAAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAH///' +
  '///4AAAAAAAAAAAAAB///////AAAAAAAAAAAAAA///////wAAAAAAAAAAAAAP//////8AAAAAAAAAAAAAD///////AAAAAAA' +
  'AAAAAAA///////4AAAAAAAAAAAAAf//////+AAAAAAAAAAAAAH///////gAAAAAAAAAAAAB///////8AAAAAAAAAAAAA////' +
  '////AAAAAAAAAAAAAP///////wAAAAAAAAAAAAD///////8AAAAAAAAAAAAA////////gAAAAAAAAAAAAP///////4AAAAAA' +
  'AAAAAAH///////+AAAAAAAAAAAAB////////gAAAAAAAAAAAAf///////8AAAAAAAAAAAAP////////AAAAAAAAAAAAD////' +
  '////wAAAAAAAAAAAA////////+AAAAAAAAAAAAP////////gAAAAAAAAAAAH////////4AAAAAAAAAAAB////////+AAAAAA' +
  'AAAAAAf////////wAAAAAAAAAAAH////////8AAAAAAAAAAAD/////////gAAAAAAAAAAB/////////8AAAAAAAAAAA/////' +
  '/////gAAAAAAAAAD///////////AAAAAAAAAD///////////8AAAAAAAAB////////////gAAAAAAAAf///////////4AAAA' +
  'AAAAP////////////AAAAAAAAD////////////wAAAAAAAA////////////+AAAAAAAAf////////////gAAAAAAAH//////' +
  '//////4AAAAAAAB////////////+AAAAAAAA/////////////wAAAAAAAP////////////8AAAAAAAD/////////////AAAA' +
  'AAAB/////////////4AAAAAAAf////////////+AAAAAAAH/////////////wAAAAAAD/////////////8AAAAAAA///////' +
  '///////AAAAAAAP/////////////wAAAAAAH/////////////+AAAAAAB//////////////gAAAAAAf/////////////4AAA' +
  'AAAP//////////////AAAAAAD//////////////wAAAAAA//////////////+AAAAAAf//////////////gAAAAAH///////' +
  '///////4AAAAAB///////////////AAAAAA///////////////wAAAAAP//////////////8AAAAAD///////////////AAA' +
  'AAA///////////////wAAAAAH//////////////8AAAAAB//////////////+AAAAAAP//////////////gAAAAAD///////' +
  '///////wAAAAAAP/////////////wAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── gloves — 비트맵 실루엣(128×128, 잉크 57.0%)
//   원본 마스크: gloves.png
var MASK_GLOVES = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAD4AAAAAAAAAAAAAAAAAAAD/AAAAAAAAAAAAAAAAAAAB/4AgAAAAAAAAAAAAAAAAAf/B/AAAAAAAAAAAA' +
  'AAAAAP/w/4AAAAAAAAAAAAAAAAD/8f+AAAAAAAAAAAAAAAAA////wAAAADgAAAAAAAAB8P///8AAAAD+AAAAAAAAB/j////A' +
  'AAAB/wAAAAAAAA//////wAA/A/+AAAAAAAAP/////+AAf4P/gAAAAAAAH//////gAP/H/4AAAAAAAB//////4AH/5/+AAAAA' +
  'AAAf/////+AB////gAAAAAAAH//////gAf///4PAAAAAAA//////4AH////P4AAAAAAP/////+AB//////AAAAAAD//////g' +
  'Af/////4AAAAHw//////8AP/////+AAAAH/f//////AD//////wAAAD////////wA//////8AAAA////////8AP//////AAA' +
  'AP////////AD//////gAAAH////////wA//////4AAAB////////8Af/////+AAAAP////////gH//////gwAAD////////4' +
  'B//////4/AAA////////+Af///////4AAP////////gH////////AAD////////4B////////4AAf///////+A////////+A' +
  'AH////////wP////////gAB////////8D////////4AAf////////A////////+AAD////////wP////////AAA////////8' +
  'D////////wAAP////////h////////8AAD////////4f////////AAAf///////+H////////gAAH////////h////////4A' +
  'AB////////4f///////+AAAf///////+P////////AAAH////////j////////wAAA/////////////////8AAAP///////8' +
  'f////////AAAD////////D////////wAAA////////w////////4AAAH///////8P///////+AAAB////////H////////gA' +
  'AAf///////5////////4AAAH////////////////8AAAB/////////////////AAAAP////////////////wAAAD////////' +
  '////////8AAAA////////////////+AAAAP////////////////gAAAD////////////////4AAAAf///////////////+AA' +
  'AAH////////////////AAAAB////////////////wAAAAf///////////////8AAAAD////////////////AAAAA////////' +
  '////////gAAAAP///////////////4AAAAD///////////////+AAAAAf///////////////gAAAAH///////////////wAA' +
  'AAB///////////////8AAAAAP///////////////AAAAAD///////////////wAAAAAf//////////////4AAAAAH///////' +
  '///////+AAAAAA///////////////gAAAAAP//////////////wAAAAAB//////////////8AAAAAAf//////////////AAA' +
  'AAAD//////////////gAAAAAAf/////////////4AAAAAAH/////////////8AAAAAAA//////////////AAAAAAAP//////' +
  '///////gAAAAAAB/////////////4AAAAAAAf////////////8AAAAAAAH/////////////AAAAAAAA/////////////gAAA' +
  'AAAAP////////////wAAAAAAAD////////////8AAAAAAAA////////////+AAAAAAAAP////////////gAAAAAAAH//////' +
  '//////wAAAAAAAB////////////8AAAAAAAAf////////////AAAAAAAAH////////////wAAAAAAAB////////////4AAAA' +
  'AAAAP///////////+AAAAAAAAD////////////wAAAAAAAA////////////+AAAAAAAAP////////////gAAAAAAAD//////' +
  '//////4AAAAAAAAf///////////+AAAAAAAAH////////////AAAAAAAAB////////////wAAAAAAAAf///////////8AAAA' +
  'AAAAH////////////AAAAAAAAB////////////wAAAAAAAAP///////////8AAAAAAAAD///wP//////+AAAAAAAAA//+AD/' +
  '//////gAAAAAAAAD/AAAf//////4AAAAAAAAAAAAAH//////+AAAAAAAAAAAAAA///////gAAAAAAAAAAAAAH//////4AAAA' +
  'AAAAAAAAAAf/////8AAAAAAAAAAAAAAAP/////AAAAAAAAAAAAAAAAH////wAAAAAAAAAAAAAAAAD///8AAAAAAAAAAAAAAA' +
  'AAAf/+AAAAAAAAAAAAAAAAAAAH/AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── dustmask — 비트맵 실루엣(128×128, 잉크 32.1%)
//   원본 마스크: dustmask.png
var MASK_DUSTMASK = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAPAAAAAAAAAAAAAAAAAAAAf+AAAAAAAAAAAAAAAAAAAP/4AAAAAAAAAAAAAAAAAAP//AAAAAAAAAAAAAAAAAAP/' +
  '/8AAAAAAAAAAAAAAAAAH///gAAAAAAAAAAAAAAAAH///+AAAAAAAAAAAAAAAAD////wAAAAAAAAAAAAAAAB/////AAAAAAAA' +
  'AAAAAAAB//w//4AAAAAAAAAAAAAAA//wD//AAAAAAAAAAAAAAA//wAP/8AAAAAAAAAAAAAAf/4PA//gAAAAAAAAAAAAAP/4H' +
  '4H/8AAAAAAAAAAAAAP/4H/gf/wAAAAAAAAAAAAH/8D/8D/+AAAAAAAAAAAAD/+D//wf/wAAAAAAAAAAAB/+B//+B/+AAAAAA' +
  'AAAAAA//B/gf4P/wAAAAAAAAAAAf/g/gB/B/+AAAAAAAAAAAP/gfwAP8H/wAAAAAAAAAAH/wfwPA/g/+AAAAAAAAAAD/4P4H' +
  '4H8H/wAAAAAAAAAB/8H8H/g/w/+AAAAAAAAAAf+H8D/+D+H/gAAAAAAAAAP/j+D//wfx/8AAAAAAAAAD/9/B//+D+//gAAAA' +
  'AAAAB///B///4P//4AAAAAAAAAf//g////B///AAAAAAAAAP//wf///4P//wAAAAAAB8D//4P////h//8D8AAAAH/9//8P//' +
  '//8P//v/4AAAH/////n/////n/////gAAD/////////////////8AAB//////////////////gAA//////////////////8A' +
  'AP//////////////////AAH+Af////////////+Af4AB/AA////////////8AD+AA/gAP////////////AAfwAPwAD//////' +
  '//////wAD8AD8AA////////////8AA/gB/AAP////////////AAP4AfgAD////////////wAB+AH4AA////////////8AAfg' +
  'B+AAH////////////AAH4AfgAB////////////gAB+AH4AAf///////////4AAfgB+AAH///////////+AAH4AfwAB//////' +
  '//////gAD+AD8AAP///////////4AA/AA/gAD///////////8AAfwAP8AA////////////AAP8AB/gAP///////////wAH+A' +
  'Af+AB///////////4AH/gAD/4Af//////////+AH/wAAf/8P///////////w//4AAD/////////////////8AAAf////////' +
  '////////+AAAB////////////////+AAAAH///////////////+AAAAAf//////////////+AAAAAAf/////////////4AAA' +
  'AAAAAP//////////AAAAAAAAAAB//////////gAAAAAAAAAAf/////////4AAAAAAAAAAD/////////8AAAAAAAAAAAf////' +
  '////+AAAAAAAAAAAH/////////gAAAAAAAAAAA/////////wAAAAAAAAAAAH////////4AAAAAAAAAAAA////////+AAAAAA' +
  'AAAAAAP////////AAAAAAAAAAAAB////////gAAAAAAAAAAAAP///////wAAAAAAAAAAAAB//gAAf/4AAAAAAAAAAAAAP/4A' +
  'AH/+AAAAAAAAAAAAAB//AAD/+AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAB//////wAAAAAAAAAAAAAAP/////wAAAAAAA' +
  'AAAAAAAB/////4AAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAB////+AAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAA//' +
  '//gAAAAAAAAAAAAAAAAH///gAAAAAAAAAAAAAAAAA///wAAAAAAAAAAAAAAAAAD//4AAAAAAAAAAAAAAAAAAf/4AAAAAAAAA' +
  'AAAAAAAAAB/4AAAAAAAAAAAAAAAAAAADwAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── haetae — 비트맵 실루엣(128×128, 잉크 48.4%)
//   원본 마스크: haetae.png
var MASK_HAETAE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAfAAAAAAAAAAAAAAAAAAAAP4AAAAAAAAAAAAAAAAAAAB/AAAAAAAAAAAAAAAAAAAAfwAQAAAAAAAAAAA' +
  'AAAAAAH+AOAAAAAAAAAAAAAAAAAB/gHwAAAAAAAAAAAAAAAAA/4B8AAAAAAAAAAAAAAAAA/8AfgAAAAAAAAAAAAAAAD//AH4' +
  'AAAAAAAAAAAAAAAD//wD+AAAAAAAAAAAAAAAB//+D/gAAAAAAAAAAAAAAA/////wAAAAAAAAAAAAAAAP////8AAAAAAAAAAA' +
  'AAAAH////+AAAAAAAAAAAAAAAD/////AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAH/////wAAAAAAAAAAAAAAD/////8A' +
  'AAAAAAAAAAAAAB/////+AAAAAAAAAAAAAAA//////gAAAAAAAAAMAAAAf/////8AAAAAAAAAHAAAAP//////AAAAAAAAAD4A' +
  'AAD//////wAAAAAAAAB+AAAA//////8AAAAAAAAB/gAA4///////gAAAAAAAD/4AAf///////4AAAAAAAf/+AAP////////A' +
  'AAAAAAf//gAD////////wAAAAAAf//4AA////////8AAAAAAf//+AAP////////AAAAAAP///gAD////////wAAAAAH///wA' +
  'Af///////8AAAAAB///8AAH////////AAAAAA///+AAB////////wAAAAAP///gAAP///////+AAAAAH///4AAD////////w' +
  'AAAAB////AAAf///////+AAAAAf///wAAD////////gAAAAH///+AAAf///////4AAAAD////wAAH///////+AAAAB////+A' +
  'AB////////gAAAA/////gAAf///////4AAAAP////4AAH///////8AAAAH/////AAA////////AAAAB/////wAAP///////g' +
  'AAAAP////8AAB///////4AAAAD/////AAAP//////+AAAAA/////wAAB///////gAAAAP////8AAAH//////4AAAAB/////A' +
  'AAAf/////+AAAAAP////wAAAD//////gAAAAD////8AAAAf/////wAAAAAf////AAAAH/////8AAAAAH////wAAAD/////+A' +
  'AAAAB////4AAAA//////AAAAAA////+AAAAf/////gAAAP//////gAAAP/////4AAB///////4AAAD/////+AAD///////8A' +
  'AAB//////gAf////////AAAAf/////+P/////////gAAAP////////////////wAAAD////////////////8AAAB////////' +
  '////////+AAAAf////////////////AAAAH////////////////gAAAD////////////////gAAAA////////////////wAA' +
  'AAP///////////////wAAAAD///////////////4AAAAA///////////////8AAAAAP///////////////AAAAAD////////' +
  '///////wAAAAA///////////////8AAAAAP///////////////AAAAAD///////////////wAAAAA///////////////8AAA' +
  'AAH///////////////AAAAAB///////////////wAAAAAf//////////////8AAAAAD///////////////AAAAAA////////' +
  '///////wAAAAAH//////////////8AAAAAA//////////////+AAAAAAP//////////////gAAAAAB//////////////4AAA' +
  'AAAP/////////////+BAAAAAB//////////////wYAAAAAf/////////////8OAAAAAH///////////////wAAAAB///////' +
  '////////8AAAAAf//////+////////AAAAAP//////8D///////4AAAAD//////wAf//////8AAAAA/////AAAD///////AA' +
  'AAAP////AAAA///////wAAAAD////wAAAH//wP//8AAAAB////8AAAA//8A//+AAAAAf////AAAAP/+AD//gAAAAH////gAA' +
  'AD//gAf/4AAAAB////4AAAA//4AD/+AAAAAf///8AAAAP/8AA//gAAAAH///+AAAAD//AAH/4AAAAD////AAAAB//gAB/+AA' +
  'AAA////AAAAAf/4AAf/wAAAAP///gAAAAP/8AAH/8AAAAH///wAAAAD/+AAB//AAAAB///4AAAAB//gAAf/wAAAB///+AAAA' +
  'A//wAAH/8AAAD////AAAAD//4AAB//AAAB////gAAAD//+AAD//4AAA////4AAAB///AAD//+AAAf///8AAAAf//wAB///gA' +
  'AH////AAAAP//4AA///4AAB////wAAAD//+AAP//+AAAf///8AAAA///gAD///AAAH////AAAAP//4AA///wAAA////wAAAB' +
  '//8AAP//8AAAAP//4AAAAAAAAAB//+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── safetymanager — 비트맵 실루엣(128×128, 잉크 35.8%)
//   원본 마스크: safetymanager.png
var MASK_SAFETYMANAGER = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAD/gAAAAAAAAAAAAAAAAAAB/4AAAAAAAAAAAAAAAAAAA//AAAAAAAAAAAAAAAAAAAP/wAAAAAAAAA' +
  'AAAAAAAAAz/8wAAAAAAAAAAAAAAAAA+//fAAAAAAAAAAAAAAAAAfv/34AAAAAAAAAAAAAAAAf7/9/gAAAAAAAAAAAAAAAP+/' +
  '/f8AAAAAAAAAAAAAAAH/v/3/AAAAAAAAAAAAAAAB/7/9/4AAAAAAAAAAAAAAA////f/AAAAAAAAAAAAAAAf///3/wAAAAAAA' +
  'AAAAAAAH///9/+AAAAAAAAAAAAAAB////f/gAAAAAAAAAAAAAA//////8AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAD///' +
  '///wAAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAB//////+AAAAAAA' +
  'AAAAAAB///////wAAAAAAAAAAAAAf//////+AAAAAAAAAAAAAH///////gAAAAAAAAAAAAB///////4AAAAAAAAAAAAAf///' +
  '///8AAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAAH////8AAAAAAAAAAAAAAA+rf/7Z+AAAAAAA' +
  'AAAAAAAf/3/+//gAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAB+Af/8B+AAAAAAAAAAAAAAfgAAAAfgAAAAAAAAAAAAAH4AA' +
  'AAH4AAAAAAAAAAAAAB4AAAAAeAAAAAAAAAAAAAAeAAAAAHgAAAAAAAAAAAAAHgAAAAB4AAAAAAAAAAAAAA4AAAAAcAAAAAAA' +
  'AAAAAAAOAAAAAHAAAAAAAAAAAAAAB4AAAAHgAAAAAAAAAAAAAAeAAAAB4AAAAAAAAAAAAAABwAAAAYAAAAAAAAAAAAAAAcAA' +
  'AAOAAAAAAAAAAAAAAADAAAADgAAAAAAAAAAAAAAA4AAABwAAAAAAAAAAAAAAAOAAAAcAAAAAAAAAAAAAAABwAAAOAAAAAAAA' +
  'AAAAAAAAeAAADgAAAAAAAAAAAAAAADgAABwAAAAAAAAAAAAAAAAcAAA8AAAAAAAAAAAAAAAAHgAAeAAAAAAAAAAAAAAAAA8A' +
  'APAAAAAAAAAAAAAAAAAHwAHwAAAAAAAAAAAAAAAAM/AH2AAAAAAAAAAAAAAAAHj//7wAAAAAAAAAAAAAAAB8f/5+AAAAAAAA' +
  'AAAAAAAA/w/4/wAAAAAAAAAAAAAAB/+AAf9gAAAAAAAAAAAAAD//wAP//AAAAAAAAAAAAAH///AP/v+AAAAAAAAAAAAP/3/4' +
  'H/7/8AAAAAAAAAAAf/+//b/9//4AAAAAAAAAAf//v///////gAAAAAAAAAf//9///////+AAAAAAAAAP///f///////wAAAA' +
  'AAAAH///75/7////+AAAAAAAAD///++f+f////wAAAAAAAB////3z/Pv///+AAAAAAAAf///9+/3L////wAAAAAAAP////n3' +
  '55////8AAAAAAAD/////9+//////gAAAAAAB/////////////4AAAAAAAf////////////+AAAAAAAH/////////////wAAA' +
  'AAAD//////////3//8AAAAAAA//////////4f//AAAAAAAP/////////4x//wAAAAAAD/////////4+H/+AAAAAAB///////' +
  '//g/8P/gAAAAAAf////////5//7/4AAAAAAH////////+f/+/+AAAAAAB/////////n//v/gAAAAAAf////////5//7/4AAA' +
  'AAAH////////+f/e/+AAAAAAB/////////n/Pv/gAAAAAAf////////5/z7/8AAAAAAP/////////cz+//AAAAAAD///////' +
  '//3h/v/wAAAAAA/////////98/z/8AAAAAAP/////////v/9//AAAAAAD/////////7/+f/wAAAAAA/////////+f/v/8AAA' +
  'AAAP/////////z/3//AAAAAAD/////////+f5//wAAAAAB//////////z8//+AAAAAAf/////////+c///gAAAAAH///////' +
  '///4f//4AAAAAB//////////////+AAAAAAf//////////////gAAAAAH//////////////4AAAAAB//////////////+AAA' +
  'AAAf//////////////wAAAAAH//////////////8AAAAAB///////////////AAAAAA///////////////wAAAAAP///////' +
  '///////8AAAAAB///////////////AAAAAAf//////////////gAAAAAD//////////////wAAAAAAH/////////////wAAA' +
  'AAAAP////////////AAAAAAAAAf//////////+AAAAAAAAAAf/////////4AAAAAAAAAAAP////////AAAAAAAAAAAAAD///' +
  '///4AAAAAAAAAAAAAAAP///4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── healthmanager — 비트맵 실루엣(128×128, 잉크 34.2%)
//   원본 마스크: healthmanager.png
var MASK_HEALTHMANAGER = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAD/gAAAAAAAAAAAAAAAAAAB/8AAAAAAAAAAAAAAAAAAAf/AAAAAAAAAAAAAAAAAAAH/wAAAAAAAAA' +
  'AAAAAAAABx/88AAAAAAAAAAAAAAAAB8f/PgAAAAAAAAAAAAAAAA/H/z+AAAAAAAAAAAAAAAAfx/8/gAAAAAAAAAAAAAAAP8f' +
  '/P8AAAAAAAAAAAAAAAH/n/z/gAAAAAAAAAAAAAAB/5/8/8AAAAAAAAAAAAAAA/+f/P/AAAAAAAAAAAAAAAP/n/z/4AAAAAAA' +
  'AAAAAAAH/5/9/+AAAAAAAAAAAAAAB/+f/f/wAAAAAAAAAAAAAA//n/3/8AAAAAAAAAAAAAAP/5/9//AAAAAAAAAAAAAAD/+f' +
  '/f/wAAAAAAAAAAAAAA//////+AAAAAAAAAAAAAAP//////gAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAB//////+AAAAAAA' +
  'AAAAAAA///////wAAAAAAAAAAAAAf//////+AAAAAAAAAAAAAH///////wAAAAAAAAAAAAB///////4AAAAAAAAAAAAAf///' +
  '///+AAAAAAAAAAAAAB//////+AAAAAAAAAAAAAAD/////+AAAAAAAAAAAAAAC//////wAAAAAAAAAAAAAB//////+AAAAAAA' +
  'AAAAAAAf//////gAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAB+A//8B+AAAAAAAAAAAAAAfgAAAAfgAAAAAAAAAAAAAHgAA' +
  'AAB4AAAAAAAAAAAAABwAAAAAOAAAAAAAAAAAAAAeAAAAADgAAAAAAAAAAAAADgAAAAB4AAAAAAAAAAAAAA4AAAAAcAAAAAAA' +
  'AAAAAAAPAAAAAPAAAAAAAAAAAAAAB4AAAAHgAAAAAAAAAAAAAAHAAAABwAAAAAAAAAAAAAABwAAAA4AAAAAAAAAAAAAAAMAA' +
  'AAOAAAAAAAAAAAAAAADgAAAHAAAAAAAAAAAAAAAA4AAABwAAAAAAAAAAAAAAAHAAAAYAAAAAAAAAAAAAAABwAAAOAAAAAAAA' +
  'AAAAAAAAOAAAHAAAAAAAAAAAAAAAABwAABwAAAAAAAAAAAAAAAAeAAA4AAAAAAAAAAAAAAAADwAAcAAAAAAAAAAAAAAAAEeA' +
  'AOIAAAAAAAAAAAAAAADjwAPnAAAAAAAAAAAAAAAB8fAPj4AAAAAAAAAAAAAAAfh//x/AAAAAAAAAAAAAAAP8P/w/wAAAAAAA' +
  'AAAAAAAf/wfgf/gAAAAAAAAAAAAAf/+AAf/+AAAAAAAAAAAAA///wAP/38AAAAAAAAAAAB/9/+AH/7/4AAAAAAAAAAD//f/w' +
  'D/+//wAAAAAAAAAD//7/8A/////AAAAAAAAAD//+//w//3//8AAAAAAAAB///n/8P/5///gAAAAAAAA///9//n/+///8AAAA' +
  'AAAAf///vv5//P///gAAAAAAAP///73///3///8AAAAAAAD////f///5////AAAAAAAB////z///+////4AAAAAAAf//////' +
  '//////+AAAAAAAH/////////////wAAAAAAD/////////////8AAAAAAA//////////////AAAAAAAP/////////////wAAA' +
  'AAAD/////////////+AAAAAAA//////wD//////gAAAAAAP/////4Af/////4AAAAAAH/////+AH/////+AAAAAAB//////g' +
  'B//////gAAAAAAf/////4Af/////4AAAAAAH/////+AH/////+AAAAAAB//////gB//////gAAAAAAf/////4Af/////4AAA' +
  'AAAH/////+AH/////+AAAAAAB////+AAAAP////wAAAAAA/////AAAAD////8AAAAAAP////wAAAA/////AAAAAAD////8AA' +
  'AAP////wAAAAAA/////AAAAD////8AAAAAAP////wAAAA/////AAAAAAD////8AAAAP////wAAAAAA/////AAAAD////8AAA' +
  'AAAP////wAAAA/////AAAAAAD////+AAAAP////wAAAAAA//////4Af/////8AAAAAAP/////+AH//////gAAAAAH//////g' +
  'B//////4AAAAAB//////4Af/////+AAAAAAf/////+AH//////gAAAAAH//////gB//////4AAAAAB//////4Af/////+AAA' +
  'AAAf/////+AH//////gAAAAAH//////wB//////4AAAAAB//////////////+AAAAAAf//////////////wAAAAAH///////' +
  '///////4AAAAAB//////////////+AAAAAAP//////////////gAAAAAB//////////////wAAAAAAP/////////////wAAA' +
  'AAAAf////////////gAAAAAAAA////////////AAAAAAAAAA//////////8AAAAAAAAAAA/////////wAAAAAAAAAAAAP///' +
  '///8AAAAAAAAAAAAAAAf///4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── safetyboot — 비트맵 실루엣(128×128, 잉크 38.8%)
//   원본 마스크: safetyboot.png
var MASK_SAFETYBOOT = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAB/4AAAAAAAAAAAAAAAAAAD//AAAAAAAAAAAAAAAAAAB//4' +
  'AAAAAAAAAAAAAAAAAA//+AAAAAAAAAAAAAAAAAAP//gAAAAAAAAAAAAAAAAAD//4AAAAAAAAAAAAAAAAAA//+AAAAAAAAAAA' +
  'AAAAAA////wAAAAAAAAAAAAAD//////8AAAAAAAAAAAAAf///////AAAAAAAAAAAAAP///////wAAAAAAAAAAAAH///////8' +
  'AAAAAAAAAAAAB////////AAAAAAAAAAAAAf///////wAAAAAAAAAAAAH///////8AAAAAAAAAAAAB////////gAAAAAAAAAA' +
  'AAf///////4AAAAAAAAAAAAH///////+AAAAAAAAAAAAB////////gAAAAAAAAAAAAf///////4AAAAAAAAAAAAH///////+' +
  'AAAAAAAAAAAAB////////wAAAAAAAAAAAAf///////8AAAAAAAAAAAAH////////AAAAAAAAAAAAA////////wAAAAAAAAAA' +
  'AAP///////+AAAAAAAAAAAAB////////gAAAAAAAAAAAAf///////4AAAAAAAAAAAAP///////+AAAAAAAAAAAAD////////' +
  'wAAAAAAAAAAAA////////8AAAAAAAAAAAAP////////gAAAAAAAAAAAD////////4AAAAAAAAAAAA////////+AAAAAAAAAA' +
  'AAf////////wAAAAAAAAAAAH////////8AAAAAAAAAAAB/////////AAAAAAAAAAAA/////////8AAAAAAAAAAAP////////' +
  '/AAAAAAAAAAAD/////////4AAAAAAAAAAA//////////AAAAAAAAAAAP/////////4AAAAAAAAAAH//////////AAAAAAAAA' +
  'AB//////////4AAAAAAAAAA//////////+AAAAAAAAAAP//////////4AAAAAAAAAD///////////AAAAAAAAAB/////////' +
  '//8AAAAAAAAAf///////////gAAAAAAAAH///////////+AAAAAAAAB////////////4AAAAAAAA/////////////gAAAAAA' +
  'AP/////////////gP/AAAAD/////////////////AAAA/////////////////4AAAP/////////////////gAAH/////////' +
  '////////8AAB//////////////////gAAf/////////////////4AAH//////////////////AAB//////////////////wA' +
  'Af/////////////////+AAH//////////////////gAB//////////////////4AAf/////////////////+AAH/////////' +
  '/////////gAB//////////////////8AAf//////////////////AAP//////////////////4AH///////////////////A' +
  'B///////////////////wAf//////////////////8AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////8AH//////////////////+A' +
  'B///////////////////AAf/////gf///////////AAH/////gAf//////////AAB/////4AAf////////4AAAf////8AAB/' +
  '///////+AAAD/////AAAP///////8AAAAf5/7/wAAD//////8IAAAAB+P8f8AAAfz/n/n/AAAAAAAAAAAAAAAIf4/4wAAAAA' +
  'AAAAAAAAAAAAAGAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── safetyvest — 비트맵 실루엣(128×128, 잉크 61.2%)
//   원본 마스크: safetyvest.png
var MASK_SAFETYVEST = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAP8AAAAA/4AAAAAAAAAAAAf/4AAAA//gAAAAAAAAAAA///4AAB///AAAAAAAAAAD///////////gAAAA' +
  'AAAAH///////////+AAAAAAAAD////////////wAAAAAAAA////////////8AAAAAAAAP////////////AAAAAAAAD//////' +
  '//////wAAAAAAAA////////////8AAAAAAAAP////////////AAAAAAAAD////////////wAAAAAAAA////////////8AAAA' +
  'AAAAP////////////AAAAAAAAD////////////wAAAAAAAAf///////////4AAAAAAAAH///////////+AAAAAAAAB//////' +
  '//////gAAAAAAAAf///////////4AAAAAAAAH///////////+AAAAAAAAB////////////gAAAAAAAAf///////////4AAAA' +
  'AAAAH///////////+AAAAAAAAB////////////gAAAAAAAAf///////////4AAAAAAAAD///////////8AAAAAAAAA//////' +
  '//////AAAAAAAAAP///////////wAAAAAAAAD///////////8AAAAAAAAA////////////AAAAAAAAAP///////////wAAAA' +
  'AAAAD///////////8AAAAAAAAB////////////gAAAAAAAAf///////////4AAAAAAAAH///////////+AAAAAAAAB//////' +
  '//////gAAAAAAAAf///////////4AAAAAAAAH///////////+AAAAAAAAB////////////gAAAAAAAA////////////8AAAA' +
  'AAAAP////////////AAAAAAAAD////////////wAAAAAAAA////////////+AAAAAAAAf////////////gAAAAAAAH//////' +
  '//////4AAAAAAAD/////////////AAAAAAAA/////////////4AAAAAAAf////////////+AAAAAAAP/////////////wAAA' +
  'AAAH/////////////+AAAAAAD//////////////wAAAAAB///////////////AAAAAB///////////////4AAAAA////////' +
  '////////AAAAAP///////////////wAAAAD///////////////8AAAAA////////////////AAAAAP///////////////wAA' +
  'AAD///////////////8AAAAA////////////////AAAAAP///////////////wAAAAD///////////////8AAAAA////////' +
  '////////AAAAAP///////////////wAAAAD///////////////8AAAAA////////////////AAAAAP///////////////wAA' +
  'AAD///////////////8AAAAA////////////////AAAAAP///////////////wAAAAD///////////////8AAAAA////////' +
  '////////AAAAAP///////////////wAAAAB///////////////4AAAAAf//////////////+AAAAAH///////////////gAA' +
  'AAB///////////////4AAAAAf//////////////+AAAAAH///////////////gAAAAD///////////////8AAAAA////////' +
  '////////AAAAAP///////////////wAAAAD///////////////8AAAAA////////////////AAAAAP///////////////wAA' +
  'AAD///////////////8AAAAA////////////////AAAAAP///////////////wAAAAD///////////////8AAAAA////////' +
  '////////AAAAAH///////////////gAAAAB///////////////4AAAAAf//////////////+AAAAAH///////////////gAA' +
  'AAB///////////////4AAAAAf//////////////+AAAAAP///////////////wAAAAD///////////////8AAAAA////////' +
  '////////AAAAAP///////////////wAAAAD///////////////8AAAAA////////////////AAAAAP///////////////wAA' +
  'AAD///////////////8AAAAA////////////////AAAAAP///////////////wAAAAD///////////////8AAAAA////////' +
  '////////AAAAAP///////////////wAAAAD///////////////8AAAAA////////////////AAAAAP///////////////wAA' +
  'AAB///////////////4AAAAAP//////////////8AAAAAD///////n///////AAAAAAP//////gf//////AAAAAAA//////w' +
  'D//////AAAAAAAA/////wAP////8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── elephant — 비트맵 실루엣(128×128, 잉크 54.2%)
//   원본 마스크: elephant.png
var MASK_ELEPHANT = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAA//8AAAAAAAAAAAAAAAAAB///4AAAAAAAAAAAB//AAD////wAA//gAAAAAB//8AD/////AA//+AAA' +
  'AAD///4D/////8B///8AAAAB////h//////h////gAAAA////////////////8AAAA/////////////////wAAAP////////' +
  '////////+AAAH/////////////////gAAD/////////////////8AAB//////////////////gAAf/////////////////8A' +
  'AP//////////////////AAH//////////////////4AB//////////////////+AAf//////////////////wAP/////////' +
  '/////////8AD///////////////////AA///////////////////wAf//////////////////+AH///////////////////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'A///////////////////wAP//////////////////8AD///////////////////AAf//////////////////wAH/////////' +
  '/////////4AB//////////////////+AAP//////////////////AAD//////////////////wAAf/////////////////4A' +
  'AH/////////////////+AAA//////////////////AAAP/////////////////wAAB/////////////////4AAAf////////' +
  '////////+AAAD/////////////////AAAA/////////////////wAAAH////////////////4AAAB////////////////+AA' +
  'AAP////////////////AAAAD////////////////wAAAAf///////////////4AAAAD///////////////8AAAAA////////' +
  '////////AAAAAH///////////////gAAAAA///////////////wAAAAAH//////////////4AAAAAA//////////////8AAA' +
  'AAAH/////////////+AAAAAAAf////////////+AAAAAAAB////////////+AAAAAAAAP////////////AAAAAAAAD//////' +
  '//////wAAAAAAAA////////////8AAAAAAAAf////////////gAAAAAAAH////////////4AAAAAAAB////////////+AAAA' +
  'AAAAf////////////wAAAAAAAH////////////8AAAAAAAD/////////////AAAAAAAA/////////////wAAAAAAAP///8//' +
  '//P///8AAAAAAAD///+P///x////AAAAAAAB////j///8P///4AAAAAAAf///wf//+D///+AAAAAAAH///8H///g////gAAA' +
  'AAAB////B///4P///4AAAAAAAf///wf//+D///+AAAAAAAH///8H///g////gAAAAAAB////B///4P///4AAAAAAAf///wf/' +
  '/+D///+AAAAAAAH///8H///g////gAAAAAAB////B///4P///4AAAAAAAf///wf//+D///+AAAAAAAH///8H///g////gAAA' +
  'AAAB////B///4P///4AAAAAAAf///wP//8D///+AAAAAAAH///8D///A////gAAAAAAB////A///wP///4AAAAAAAf///wP/' +
  '/8D///+AAAAAAAH///8D///A////gAAAAAAB///+A///wH///4AAAAAAAP///gP//8B///8AAAAAAAD///4D///Af///AAAA' +
  'AAAAf//8A///wD///gAAAAAAAD//+AP//8Af//wAAAAAAAAf//AB//+AD//4AAAAAAAAAAAAAP//AAAAAAAAAAAAAAAAAAB/' +
  '/gAAAAAAAAAAAAAAAAAAH/gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── owl — 비트맵 실루엣(128×128, 잉크 30.4%)
//   원본 마스크: owl.png
var MASK_OWL = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAeAAAAH/gAAAB4AAAAAAAAH8AAA///wAAD+AAAAAAAAB/wAD////wAD/gAAAAAAAAf/AD/////AD/4AAAA' +
  'AAAAH/8H/////+D/+AAAAAAAAB////////////gAAAAAAAAf///////////4AAAAAAAAD///////////8AAAAAAAAA//////' +
  '//////AAAAAAAAAP///////////wAAAAAAAAB///////////8AAAAAAAAAf//////////+AAAAAAAAAH///////////gAAAA' +
  'AAAAA///////////wAAAAAAAAAP//////////8AAAAAAAAAD///////////AAAAAAAAAA//gH///+Af/wAAAAAAAAAf/AAP/' +
  '/8AA/+AAAAAAAAAH/AAA//8AAD/gAAAAAAAAD/gAAD/8AAAf8AAAAAAAAA/gAAAf+AAAB/AAAAAAAAAfwAAAD/AAAAP4AAAA' +
  'AAAAH8AAAAfgAAAD+AAAAAAAAD+AAAADwAAAAfwAAAAAAAA/AAAAAYAAAAD8AAAAAAAAPwAAAAGAAAAA/AAAAAAAAD4AAAAA' +
  'AAAAAHwAAAAAAAB+AAAAAAAAAAB+AAAAAAAAfgAA4AAABwAAfgAAAAAAAH4AA/gAAB/AAD4AAAAAAAB8AAf+AAA/4AA+AAAA' +
  'AAAAfAAP/gAAf/AAPgAAAAAAAHwAD/8AAP/wAD4AAAAAAAB8AA//AAD/+AA+AAAAAAAAfAAf/wAA//gAPgAAAAAAAHwAH/8A' +
  'AP/4AD4AAAAAAAB8AB//gAD/+AA+AAAAAAAAfAAP/4AB//AAPgAAAAAAAH4AD/+AAf/wAD4AAAAAAAB+AAf/wYP/4AB+AAAA' +
  'AAAAfgAD/////8AAfgAAAAAAAH8AAAB//gAAAP4AAAAAAAB/AAAAP/wAAAD+AAAAAAAA/4AAAB/8AAAB/wAAAAAAAf+AAAAf' +
  '/AAAAf+AAAAAAAH/wAAAH/gAAAP/wAAAAAAD/+AAAB/4AAAH/8AAAAAAB//wAAAf+AAAD//gAAAAAAf/+AAAH/gAAB//4AAA' +
  'AAAP//wAAB/4AAA///AAAAAAD//+AAA//AAAf//wAAAAAB///4AAP/4AAf//+AAAAAAf///wAP//AA////gAAAAAH////w//' +
  '7//////8AAAAAD//////gYH//////AAAAAA//////wAAf/////wAAAAAP/////gAAB/////+AAAAAD/////gAAAH/////gAA' +
  'AAB////AAAAAAAP///4AAAAAf///AAAAAAAA///+AAAAAH///gAAAAAAAH///gAAAAB///4AAAAAAAB///4AAAAAf//+AAAA' +
  'AAAAf//+AAAAAH///AAAAAAAAD///gAAAAB///wAAAAAAAA///4AAAAA///8AAAAAAAAP///AAAAAH///AAAAAAAAD///gAA' +
  'AAB///wAAAAAAAA///4AAAAAf//4AAAAAAAAH//+AAAAAH//+AAAAAAAAB///gAAAAB///gAAAAAAAAf//4AAAAAf//4AAAA' +
  'AAAAH//+AAAAAH//+AAAAAAAAB///gAAAAB///gAAAAAAAAf//4AAAAAP//4AAAAAAAAH//+AAAAAD//+AAAAAAAAB///AAA' +
  'AAA///wAAAAAAAA///wAAAAAP//8AAAAAAAAP//8AAAAAB///AAAAAAAAD//+AAAAAAf//wAAAAAAAA///gAAAAAH//8AAAA' +
  'AAAAP//4AAAAAA///gAAAAAAAH//8AAAAAAP//4AAAAAAAB///AAAAAAD//+AAAAAAAAf//wAAAAAAf//wAAAAAAAH//4AAA' +
  'AAAH//8AAAAAAAD//+AAAAAAA///AAAAAAAA///AAAAAAAP//4AAAAAAAf//wAAAAAAB//+AAAAAAAH//8AAAAAAAf//wAAA' +
  'AAAD//+AAAAAAAD//8AAAAAAA///AAAAAAAA///gAAAAAAf//wAAAAAAAH//4AAAAAAH//4AAAAAAAB///AAAAAAD//+AAAA' +
  'AAAAP//4AAAAAB///AAAAAAAAB//+AAAAAAf//wAAAAAAAAf//wAAAAAP//4AAAAAAAAD//+AAAAAH//8AAAAAAAAAf//wAA' +
  'AAD///AAAAAAAAAH//+AAAAB///gAAAAAAAAA///wAAAA///wAAAAAAAAAH//+AAAAf//4AAAAAAAAAA///wAAAP//8AAAAA' +
  'AAAAAH//+AAAH//+AAAAAAAAAAB///wAAD///gAAAAAAAAAAP///AAB///wAAAAAAAAAAB///4AB///4AAAAAAAAAAAP///g' +
  'B///8AAAAAAAAAAAB///8A///+AAAAAAAAAAAAH///5///+AAAAAAAAAAAAA////////AAAAAAAAAAAAAH///////gAAAAAA' +
  'AAAAAAAf//////gAAAAAAAAAAAAAD//////wAAAAAAAAAAAAAAP/////wAAAAAAAAAAAAAAAf////wAAAAAAAAAAAAAAAB//' +
  '//gAAAAAAAAAAAAAAAAA//8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── crab — 비트맵 실루엣(128×128, 잉크 34.2%)
//   원본 마스크: crab.png
var MASK_CRAB = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAB+AAAAAAB+AAAAAA' +
  'AAAAAH/+AAAAAH/+AAAAAAAAAAP//wAAAAD//8AAAAAAAAAP//8AAAAA///wAAAAAAAAP///AAAAAP///AAAAAAAAH//+AAA' +
  'AAAf//4AAAAAAAD//8AAAAAAA///AAAAAAAD//8AAAAAAAD//8AAAAAAB//8AEAAAAIAH//gAAAAAA//8AHAAAADgA//8AAA' +
  'AAAP/+ADwAAAA8AH//AAAAAAH//AD8AAAAPwA//4AAAAAD//gB/AAAAD+AH//AAAAAA//4B/wAAAA/4B//wAAAAAf/8D/4AA' +
  'AAH/wP/+AAAAAH//D/+AAAAB//D//gAAAAD//z//AAAAAP/8//8AAAAA/////gAAAAB/////AAAAAP////4AAAAAf////wAA' +
  'AAD////8AAAAAD////8AAAAB////+AAAAAAf////gAAAAf////AAAAAAD////4AAAAH////gAAAAAAf///+AAAAB////wAAA' +
  'AAAD////gAAAAf///4AAAAAAAf///4AAAAD///8AAAAAAAD///8AAAAA///8AD4AAHwAP///AAAAAH//8AD/gAH/AA///gAA' +
  'AAB//8AA44ABxwAD//4AAAAB//8AAcHAA4OAAP//gAAAAf/4AAHBwAODgAAf/4AAAAH/8AABwMADA4AAD/+AAAAD//AAAcDg' +
  'BwOAAA//wAAAA//4AAHA4AcDgAAf/8AAAAP/+AABwcADg4AAH//AAAAB//wAAOHAA4cAAD//gAAAAf/8AAB/z/v+AAA//4AA' +
  'AAD//AAAP////AAAH/8AAAAA//+ABx////jgAP//AAAAAH//wA//////8AP//gAAAAA///Af//////gP//wAAAAAH//4f///' +
  '///+H//4AAAAAA///P///////z//8AAAAAAH//3//////////+AAAAAAAP////////////8AAAAAAAD/////////////AAAA' +
  'AAAAf////////////gAAAAAAAD////////////wAAAAAAAwP///////////wMAAAAAP/w///////////w//AAAAP//D/////' +
  '/////w//8AAAH//8P/////////w///gAAD///z/////////8f//8AAB//////////////////gAA/5//////////////+f8A' +
  'Af8P//////////////D/gAP+A//////////////Af8AD/AB////////////+AD/AB/gAD///////////8AAfwAfwAAHP////' +
  '////88AAD+AH4A//5/////////f/8AfgB8Af//////////////gD4AeAf//////////////+AeAHgf///////////////4Hg' +
  'BwP///4///////x////A4AIH///+P//////8f///4EAAD/P/////////////z/AAAB/gAP//////////AAf4AAAf4AP/////' +
  '/////8AH+AAAP8AH///////////gA/wAAH/AB//+f////n//4AP+AAB/gA///n////5///AB/gAA/wA//+f/////5//8AP8A' +
  'AP4Af/4P+f//n/B//gB/AAD8AP8AP/gDwB/8AP8APwAA/AH/AH/wAAAP/gD/gD8AAPgB/gD/4AAAB/8Af4AfAAB4A/4A/8AA' +
  'AAP/AH/AHgAAeAP+AP+AAAAB/wB/wB4AAHAH/AH+AAAAAH+AP+AOAAAwB/wD8AAAAAAPwD/gDAAAAAf4A/gAAAAAH8Af4AAA' +
  'AAAH8AP4AAAAAB/AD+AAAAAAB/AD+AAAAAAfwA/gAAAAAAfgA/gAAAAAH8AH4AAAAAAH4AP8AAAAAD/AB+AAAAAAB+AD/AAA' +
  'AAA/wAfgAAAAAAPgA/4AAAAAP8AHwAAAAAAD4AH/AAAAAP+AB8AAAAAAAOAB/4AAAAH/gAcAAAAAAADgAP/AAAAD/wAHAAAA' +
  'AAAAAAB/wAAAA/4AAAAAAAAAAAAAH8AAAAP4AAAAAAAAAAAAAAHAAAADgAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── octopus — 비트맵 실루엣(128×128, 잉크 42.4%)
//   원본 마스크: octopus.png
var MASK_OCTOPUS = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAH/+AAAAAAAAAAAAAAAAAAP//8AAAAAAAAAAAAAAAAAP///4AAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAH//' +
  '//8AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAA//////8AAAAAAA' +
  'AAAAAAAP//////gAAAAAAAAAAAAAH//////8AAAAAAAAAAAAAD///////AAAAAAAAAAAAAA///////4AAAAAAAAAAAAAf///' +
  '///+AAAAAAAAAAAAAH///////wAAAAAAAAAAAAD///////8AAAAAAAAAAAAA////////AAAAAAAAAAAAAP///////4AAAAAA' +
  'AAAAAAH///////+AAAAAAAAAAAAB////////gAAAAAAAAAAAAf///////4AAAAAAAAAAAAH///////+AAAAAAAAAAAAB////' +
  '////gAAAAAAAAD/AAf///////8AD/AAAAAD/8AH////////AD/8AAAAB//gB////////wB//gAAAA//4Af///////4Af/8AA' +
  'AAf//AH///////+AP//gAAAH//wB////////gD//4AAAD+P8Af///////4A/x/AAAA+B/gH///////+AP4HwAAAPgP4B////' +
  '////gH+B+AAAHwD+AP///////wB/gPgAAB8A/gD///////8Af4D4AAAfAf4A////////AH+A+AAAHwH+AP///////wB/gPgA' +
  'AB8B/gB///////4Af8D4AAAfB/8Af//////+AP/g+AAAH9//AD///////gD/+/gAAA///4A///////wB///wAAAP//+AP///' +
  '///8Af//8AAAD///wD///////AP///AAAAf//+A/gf//gfwH///gAAAH///wP4D//wH8D///4AAAA////D8Af/8A/D///8AA' +
  'AAH///9/AH/+AP7///+AAAAA/////wA//AB/////AAAAAH////8AP/wAf////gAAAAA/////AD/8AH////wAAAAAH////wAf' +
  '+AB////4AAAAAAf///8AH/gAf///4AAAAAAB////AB/4AP///8AAAAAAAH///wAf+AD///8AAAAAAAAf//+AH/wB///4AAAA' +
  'AAEAAf//gD/8Af//gADAAAAf8AAP/8A//AP/+AAH+AAAf/4AB//gf/4H/+AAP/4AAP//gAf/+P//n//gAf//AAH///AP////' +
  '////8A///4AB///////////////////AA///////////////////wAP//////////////////8AH+H///////////////h/g' +
  'B+Af//////////////gH4AfgD//////////////wB+AH4Af/////////////4AfgB+AH/////////////+AH4AfgB///////' +
  '///////gB+AH8Af/////////////4A/gA/gP//8f//////h///AfwAP+H//8D//////4H//4f8AB////8B//////+A////+A' +
  'AP///+Af//////gD////AAB///+AP//////8Af///gAAP//+AH///////gB///wAAA//+AH///////+AH//wAAAA/4AH////' +
  '////4AH/AAAAAAAAH//+P/x///gAAAAAAAAAAP///D/8P///AAAAAAAAAAf///g//B///+AAAAAAAAAf///4P/wf///4AAAA' +
  'AAAAP///8D/8D////AAAAAAAAH////A//A////4AAAAAAAD////wP/wH////gAAAAAAB////4H/+B////4AAAAAAA////+B/' +
  '/gf////AAAAAAAP////Af/8D////4AAAAAAH+A//wP//A//wH+AAAAAAB/AH/8D//4P/8A/gAAAAAAfgB/+B//+B/+AH4AAA' +
  'AAAP4Af/gf//wf/gA/AAAAAAD+AH/wP//8D/4APwAAAAAAfgD/8D///Af/AD8AAAAAAH4B/+B///4H/4B+AAAAAAB/A//Af/' +
  '/+A//A/gAAAAAAf///gH+B/gH///4AAAAAAD///wB/AP4A///8AAAAAAAf//4AfwB/AH///AAAAAAAH//8AP4AfwA///gAAA' +
  'AAAAf/+AD+AH8AD//wAAAAAAAD/+AAfgB/AAf/wAAAAAAAAH4AAH8AfgAAfgAAAAAAAAAAAAB/AP4AAAAAAAAAAAAAAAAAf4' +
  'H+AAAAAAAAAAAAAAAAAD/j/AAAAAAAAAAAAAAAAAA///wAAAAAAAAAAAAAAAAAH//4AAAAAAAAAAAAAAAAAA//8AAAAAAAAA' +
  'AAAAAAAAAH/+AAAAAAAAAAAAAAAAAAAf+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── snail — 비트맵 실루엣(128×128, 잉크 57.2%)
//   원본 마스크: snail.png
var MASK_SNAIL = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAB/8AAAAAAAAAAAAAAAAAAP///AAAAAAAAAAAAAAAAA////+AAAAAAAAAA' +
  'AAAAAB/////8AAAAAAAAAAAAAAB//////wAAAAAAAAAAAAAB//////+AAAAAAAAAAAAAB///////4AAAAAAAAAAAAA//////' +
  '//AAAAAAAAAAAAA////////8AAAAAAAAAAAAf////////gAAAAAAAAAAAP////////8AAAAAAAAAAAP/////////gAAAAAAA' +
  'AAAH/////////8AAAAAAAAAAD//////////gAAAAAAAAAB//////////8AAAAAAAAAA///////////gAAAAAAAAAf///////' +
  '///8AAAAAAAAAP///n///////AAAAAAAAAD///AH//////4AAAAAAAAB///AAf//////AAAAAAAAA///gAB//////wAAAAAA' +
  'AAP//4AAH/////+AAAAAAAAH//8AAA//////gAAA+AAAD///AAAH/////8AAB/4AAA///wAAA//////AAA//gAAf//8AAAH/' +
  '////wAAf/8AAH///AAAB/////+AAP//AAD///4AAAP/////gAD//4AB///+AAAB/////8AB//+AAf///wAAAf/////AAf//g' +
  'AH////AAAD/////4AP//4AD////8AAA//////AH//+AA/////gAAP/////4D///gAP////8AAB//////3///4AH/////gAAf' +
  '/////////8AB/////4AAH/////////+AAf/////AAB//////////AAP/////wAAf/////////gAD/////+AAD/////////gA' +
  'A//////gAA/////////gAAP/////4AAP////////wAAH/////+AAD////////4AAB//////gAA////////8AAAf/////4AAP' +
  '////////AAAH/////+AAD////////wAAB//////gAA////////4AAAf/////4AAP///////+AAAH/////+AAH////////AAA' +
  'B//////gAB////////wAAAf/////wAAf///////8AAAH/////8AAH////////AAAB//////AAB////////wAAAf/////wAAf' +
  '///////8AAAH/////4AAP////////AAAB/////+AAD////////wAAAf/////AAA////////+AAAH/////wAAf////////wAA' +
  'A/////4AAH////////8AAAP////+AAD/////////gAAD/////AAA/////////8AAA/////gAAf/////////AAAH////4AAH/' +
  '////////4AAB////8AAD/////////+AAAP///+AAA//////////gAAD////gAAf/////////8AAA////wAAP//////////AA' +
  'AP///8AAH//////////wAAB////AAB//////////+AAAf///wAA///////////gAAD///+AAf//////////4AAA////gAP//' +
  '////////+AAAH///8AP///////////gAAB////gH///////////4AAAP///+H///////////+AAAB/////////////////AA' +
  'AAf////////////////wAAAD////////////////8AAAAf///////////////+AAAAH////////////////gAAAA////////' +
  '////////wAAAAH///////////////8AAAAA///////////////+AAAAAH///////////////AAAAAA///////////////gAA' +
  'AAAP//////////////4AAAAAB//////////////8AAAAAAf//////////////AAAAAAH//////////////wAAAAAB///////' +
  '///////8AAAAAAf//////////////AAAAAAH//////////////wAAAAAD//////////////+AAAAAB///////////////gAA' +
  'AAA///////////////8AAAAAf///////////////gAAAAf///////////////8AAAAP////////////////wAAAH////////' +
  '////////+AAAD/////////////////wAAB/////////////////+AAAf/////////////////gAAH/////////////////4A' +
  'AB/////////////////+AAAf/////////////////gAAD/////////////////wAAAP////////////////wAAAA////////' +
  '////////wAAAAA///////////////gAAAAAAA///////////4AAAAAAAAAAP/////////AAAAAAAAAAAAf///////+AAAAAA' +
  'AAAAAAB///////+AAAAAAAAAAAAAH//////+AAAAAAAAAAAAAAP/////8AAAAAAAAAAAAAAAf////wAAAAAAAAAAAAAAAAH/' +
  '/8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── bee — 비트맵 실루엣(128×128, 잉크 29.3%)
//   원본 마스크: bee.png
var MASK_BEE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAOAAAAAABwAAAAAAAAAAAAP4AAAAAB/AAAAAAAAAAAAH/AAA' +
  'AAA/4AAAAAAAAAAAD/4AAAAAf/AAAAAAAAAAAA//AAAAAP/wAAAAAAAAAAAP/4AAAAH/8AAAAAAAAAAAD//gAAAH//AAAAAA' +
  'AAAAAA//8AAAD//wAAAAAAAAAAAP//gAAB//8AAAAAAAAAAAB//8AAA//+AAAAAAAAAAAAPw/AAAPw/AAAAAAAAAAAAAwD4A' +
  'AHwDAAAAAAAAAAAAAAAfAAD4AAAAAAAAAAAAAAAAHwAA+AAAAAAAAAAAAAAAAA+AAfAAAAAAAAAAAAAAAAAHwAPgAAAAAAAA' +
  'AAAAAAAAB///4AAAAAAAAAAAAAAAAAf//+AAAAAAAAAAAAAAAAAH///gAAAAAAAAAAAAAAAAB///4AAAAAAAAAAAAAAAAB//' +
  '//gAAAAAAAAAAAAAAAAf///8AAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAH////4AAAAAAAAAB/8AAAD/////AAAA/+AA' +
  'AB//4AAA/////wAAB//4AAA///gAAf////+AAB///AAA///+AAH/////gAB///8AAP+P/4AD/////8AB//H/AAH8AP/AA///' +
  '///AA/8AP4AD8AA/8AP/////wA/8AA/AA+AAD/gD/////8Af8AAHwAPgAAP8A//////AP8AAB8AHwAAB/gP/////wH+AAAPg' +
  'B8AAAP8H/////+D/AAAD4AfAAAA/h//////h/AAAA+AHwAAAH/////////gAAAPgB8AAAA/////////wAAAD4AfAAAAH////' +
  '////4AAAA+AH4AAAA////////8AAAAfgB+AAAAP////////AAAAH4AfwAAAB////////gAAAD+AD+AAAAP///////wAAAB/A' +
  'A/wAAAB///////4AAAA/wAP/AAAAP//////+AAAA/8AD/8AAAD///////AAAA//AAf/4AAAf/B/4P/gAAB//gAH//4AAH/wA' +
  'AD/4AAH//4AA///+AD/8AAA//AB///8AAP8AP/5//AAAP/5//AD/AAB+AAf///4AAH///+AAfgAAPgAA////AAD///8AAHwA' +
  'AD8AAD///4AB///8AAD8AAAfAAAH///wD///4AAA+AAAD4AAA////////8AAAfAAAAfAAAP////////AAAPgAAAD4AAD////' +
  '////wAAHwAAAAfgAB////////+AAH4AAAAD+AB/////////4AH8AAAAAP+P//////////8f8AAAAAB//////////////+AAA' +
  'AAAH/////////////+AAAAAAAP////////////8AAAAAAAAfx////////+P4AAAAAAAAAAP////////AAAAAAAAAAAAD////' +
  '////wAAAAAAAAAAAA////////8AAAAAAAAAAAAf////////gAAAAAAAAAAAP////////8AAAAAAAAAAAD/+P///x//AAAAAA' +
  'AAAAAB//AP//AP/4AAAAAAAAAAA//gAAAAB//AAAAAAAAAAAPz4AAAAAf/wAAAAAAAAAAH4eAAAAAHh+AAAAAAAAAAB8HgAA' +
  'AAB4PgAAAAAAAAAA/B8AAAAA+D8AAAAAAAAAAfwfAAAAAPg/gAAAAAAAAAP8H8AAAAP4P8AAAAAAAAAH/B/wAAAP+D/gAAAA' +
  'AAAAD/4f/wAA//x/8AAAAAAAAA////////////AAAAAAAAAP///////////wAAAAAAAAD///////////8AAAAAAAAA//////' +
  '//////AAAAAAAAAH///////////gAAAAAAAAAef////////ngAAAAAAAAAAD/+Af+Af/wAAAAAAAAAAAA//AAAAD/8AAAAAA' +
  'AAAAAAP/wAAAA//AAAAAAAAAAAAD9+AAAAfPwAAAAAAAAAAAA+HgAAAHh8AAAAAAAAAAAAfB8AAAD4PgAAAAAAAAAAAHwPwA' +
  'AD8D4AAAAAAAAAAAD+B/AAD+B/AAAAAAAAAAAB/gP/AP/Af4AAAAAAAAAAA/4D////wH/AAAAAAAAAAAf/Af///4D/4AAAAA' +
  'AAAAAH/wD///8A/+AAAAAAAAAAB/8Af//+AP/gAAAAAAAAAAf+AD///AB/4AAAAAAAAAAH/gAf//gAf+AAAAAAAAAAB/wAD/' +
  '/wAD/gAAAAAAAAAAPwAAf/wAAPwAAAAAAAAAAAwAAB/4AAAwAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAAAYAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── frog — 비트맵 실루엣(128×128, 잉크 45.3%)
//   원본 마스크: frog.png
var MASK_FROG = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAP/AAAAAP/AAAAAAAAAAAAP/8AAAAP/8AAAAAAAAAAAH//wAAAP//wAAAAA' +
  'AAAAAD//+AAAH//8AAAAAAAAAAB///wAAD///gAAAAAAAAAA/B//AAD/+D8AAAAAAAAAAPgH/////+AfAAAAAAAAAAHwA///' +
  '///AD4AAAAAAAAAB8AH/////gAeAAAAAAAAAAeAA/////wAHgAAAAAAAAAHgAP////8AB4AAAAAAAAAB4AB/////AAOAAAAA' +
  'AAAAAcAAf////gADgAAAAAAAAAPAAH////4AA8AAAAAAAAADwAB/////AAPAAAAAAAAAA8AA/////wADwAAAAAAAAAPgAf//' +
  '//+AA8AAAAAAAAAD4AH/////gAfAAAAAAAAAA+AH/////+AHwAAAAAAAAAfwD//////wD+AAAAAAAAAH/H///////j/gAAAA' +
  'AAAAD///////////8AAAAAAAAB////////////gAAAAAAAA////////////8AAAAAAAAP////////////AAAAAAAAH//////' +
  '//////4AAAAAAAB////////////+AAAAAAAA/////////////wAAAAAAAP////////////8AAAAAAAD/////////////gAAA' +
  'AAAA/////////////4AAAAAAAf////////////+AAAAAAAH/////////////gAAAAAAB/////////////4AAAAAAAf//////' +
  '//////+AAAAAAAD/////////////gAAAAAAA/////////////wAAAAAAAP////////////8AAAAAAAD/////////////AAAA' +
  'AAAAf////////////gAAAAAAAH////////////4AAAAAA/A////////////8D8AAAB/+f////////////n/4AAA/////////' +
  '/////////AAAf/////////////////4AAP//////////////////AAH//////////////////4AB//////////////////+A' +
  'A///////////////////wAP//////////////////8AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4Af//////////////////+AD///////////////////AA///////////////////wAP/////////' +
  '/////////8AD///////////////////AAf//////////////////gAH//////////////////4AA///////+AAB///////8A' +
  'AH//////wAAAA///////AAB//////gAAAAB//////gAAP/////gAAAAAH/////wAAD/////gAAAAAAf////8AAAf////wAAA' +
  'AAAD////+AAAD////4AAAAAAAf////AAAAf///8AAAAAAAD////wAAAD////AAAAAAAA////4AAAA////wAAAAAAAP///8AA' +
  'AAH///8AAAAAAAD///+AAAAA////AAAAAAAA////AAAAAH///wAAAAAAAP///gAAAAA///8AAAAAAAD///wAAAAAH///gAAA' +
  'AAAB///4AAAAAA///4AAAAAAAf//8AAAAAAH///AAAAAAAP//+AAAAAAB///wAAAAAAD///gAAAAAB///+AAAAAAB///+AAA' +
  'AAD////wAAAAAA////8AAAAD////+AAAAAAf////wAAAD/////wAAAAAP/////AAAB/////+AAAAAH/////4AAA//////wAA' +
  'AAD//////AAAP/////+AAAAB//////wAAH//////wAAAA//////+AAB///////AAAA///////gAAf//////+AAB///////4A' +
  'AD/////////////////8AAA//////////////////AAAH/////////////////gAAAP////////////////AAAAAB/gP/5//' +
  '//j/8B/wAAAAAAAAAHAAAAAADwAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── swan — 비트맵 실루엣(128×128, 잉크 43.1%)
//   원본 마스크: swan.png
var MASK_SWAN = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAP/AAAAAAAAAAAAAAAAAAAf/+AAAAAAAAAAAAAAAAAAf//4AAAAAAAAAAAAAAAAAP///AAAAAAAAA' +
  'AAAAAAAAH///4AAAAAAAAAAAAAAAAH////gAAAAAAAAAAAAAAAB////4AAAAAAAAAAAAAAAA/////AAAAAAAAAAAAAAAAf//' +
  '//4AAAAAAAAAAAAAAAH////+AAAAAAAAAAAAAAAD/////wAAAAAAAAAAAAAAA/////+AAAAAAAAAAAAAAAf/////gAAAAAAA' +
  'AAAAAAAH/////4AAAAAAAAAAAAAAB//////AAAAAAAAAAAAAAAf/////wAAAAAAAAAAAAAAP/////8AAAAAAAAAAAAAAB///' +
  '///AAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////4AAAAAAA' +
  'AAAAAAAD/////+AAAAAAAAAAAAAAA//////gAAAAAAAAAAAAAAP/////4AAAAAAAAAAAAAAD/////+AAAAAAAAAAAAAAA///' +
  '///gAAAAAAAAAAAAAAP/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB//////AAAAAAAAAAAAAAAf/4f//wAAAAAAA' +
  'AAAAAAAH/8D//8AAAAAAAAAAAAAAD/+A///AAAAAAAAAAAAAAA//AP//gAAAAAAAAAAAAAAP/gD//4AAAAAAAAAAAAAAH/wB' +
  '//+AAAAAAAAAAAAAAB/8A///AAAAAAAAAAAAAAA/+AP//wAAAAAAAAAAAAAAP/AH//8AAAAAAAAAAAAAAD/gD//+AAAAAAAA' +
  'AAAAAAA/gB///gAAAAAAAAAAAAAAPwAf//wAAAAAAAAAAAAAAAgAP//8AAAAAAAAAAAAAAAAAH//+AAAAAAAAAAAAAgAAAD/' +
  '//gAADAAAAAAAAA/gAAB///wAAP8AAAAAAAAf+AAA///8AAP/gAAAAAAAP/4AAP//+AAP/8AAAAAAAH//AAH///gAH//gAAA' +
  'AAAD//4AD///wAH//8AAAAAAA///AB///8AD///AAAAAAAf//8Af///AB///4AAAAAAP///AP///gA////AAAAAAD///4H//' +
  '/4Af///wAAAAAB////B///+AP///+AAAAAAf///4////gH////gAAAAAH////////4D////8AAAAAD////////+A/////AAA' +
  'AAA/////////gf////wAAAAAP////////8P////+AAAAAH///////////////gAAAAB///////////////4AAAAAf///////' +
  '///////+AAAAAH///////////////gAAAAD///////////////8AAAAA////////////////AAAAAP///////////////wAA' +
  'AAD///////////////8AAAAA////////////////AAAAAP///////////////wAAAAD///////////////8AAAAA////////' +
  '////////AAAAAP///////////////wAAAAD///////////////8AAAAA////////////////AAAAAP///////////////wAA' +
  'AAD///////////////8AAAAA////////////////AAAAAP///////////////wAAAAD///////////////8AAAAAf///////' +
  '////////AAAAAH///////////////gAAAAB///////////////4AAAAAf//////////////+AAAAAH///////////////gAA' +
  'AAA///////////////wAAAAAP//////////////8AAAAAD///////////////AAAAAAf//////////////gAAAAAH///////' +
  '///////4AAAAAA//////////////8AAAAAAP//////////////AAAAAAB//////////////gAAAAAAf/////////////4AAA' +
  'AAAD/////////////+AAAAAAA//////////////AAAAAAAH/////////////gAAAAAAB/////////////4AAAAAAAP//////' +
  '//////8AAAAAAAB////////////+AAAAAAAAP////////////AAAAAAAAB////////////wAAAAAAAAP///////////4AAAA' +
  'AAAAD///////////8AAAAAAAAAP//////////+AAAAAAAAAB///////////AAAAAAAAAAP//////////gAAAAAAAAAB/////' +
  '/////gAAAAAAAAAAP/////////wAAAAAAAAAAA/////////4AAAAAAAAAAAH////////4AAAAAAAAAAAAf///////8AAAAAA' +
  'AAAAAAB///////8AAAAAAAAAAAAAH//////8AAAAAAAAAAAAAAf/////8AAAAAAAAAAAAAAA/////4AAAAAAAAAAAAAAAA//' +
  '//gAAAAAAAAAAAAAAAAA//+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── umbrella — 비트맵 실루엣(128×128, 잉크 29.6%)
//   원본 마스크: umbrella.png
var MASK_UMBRELLA = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAA8AAAAAAAAAAAAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAAD/AAAAAAAAAA' +
  'AAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAP+AAAAAAAAAAAAAAAAAAB//+AAAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAf//' +
  '///AAAAAAAAAAAAAAA//////8AAAAAAAAAAAAAB///5///4AAAAAAAAAAAAB///8P///gAAAAAAAAAAAB////B///+AAAAAA' +
  'AAAAAB////gP///4AAAAAAAAAAA////wD////gAAAAAAAAAA////8Af///+AAAAAAAAAA////+AH////wAAAAAAAAAf////g' +
  'A////+AAAAAAAAAP////wAP////4AAAAAAAAH////8AB/////AAAAAAAAD/////AAf////4AAAAAAAD/////gAH/////AAAA' +
  'AAAB/////4AB/////4AAAAAAA///g/+AAP/A///AAAAAAAf//AP/gAD/wB//4AAAAAAP//AD/4AA/8AP//AAAAAAD//gA/+A' +
  'AP/AA//4AAAAAB//gAP/gAD/4AH//AAAAAA//wAD/4AA/+AAf/4AAAAAf/wAB/+AAP/gAD/+AAAAAH/4AAf/gAD/8AAf/wAA' +
  'AAD/8AAP/4AA//AAD/+AAAAB/+AAD/+AAP/wAAf/gAAAAf/AAB//gAD/+AAD/8AAAAP/wAAf/4AB//gAAf/AAAAH/4AAH/+A' +
  'Af/8AAD/4AAAB/8AAD//w+H//AAA//AAAA/+AAB//9////4AAH/wAAAP/gAAf///////AAA/+AAAH/wAAP///////wAAP/gA' +
  'AB/8AAH///////+AAB/4AAA/+AAD////////wAAf/AAAP/AAB////////+AAD/wAAD/wAA/////////wAA/8AAB/4AAf////' +
  '////+AAH/gAAf+AAP/////////wAB/4AAH/gAP//////////AAP+AAD/wAH//////////4AD/wAA/8AH///////////gA/8A' +
  'AP/AH///////////+AH/AAD/gP////////////8B/4AB/4f/////////////4f+AAf//////////////////gAH/////////' +
  '/////////4AB//////////////////+AAf//////////////////wAP//////////////////8AD///////////////////A' +
  'A///////////////////wAP//////////////////8AD///////////////////AA///////////////////wAP/////////' +
  '/////////8AD8B/////w//8P////+A/AA8AH/gH/wB/4A/+Af+ADwAOAA/gAfwAP8AD+AB/AAMADAAHwAD4AB/AAfAAHgABA' +
  'AAAA4AAcAAfwADgAAwAAAAAAAEAACAAH8AAQAAMAAAAAAAAAAAAAB/AAAAAAAAAAAAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAH' +
  '8AAAAAAAAAAAAAAAAAAAB/AAAAAAAAAAAAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAH8AAAAAAAAAAAAAAAAAAAB/AAAAAAAAAA' +
  'AAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAH8AAAAAAAAAAAAAAAAAAAB/AAAAAAAAAAAAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAH' +
  '8AAAAAAAAAAAAAAAAAAAB/AAAAAAAAAAAAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAH8AAAAAAAAAAAAAAAAAAAB/AAAAAAAAAA' +
  'AAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAH8AAAAAAAAAAAAAAAAAAAB/AAAAAAAAAAAAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAH' +
  '8AAAAAAAAAAAAAAAAAAAB/AAAAAAAAAAAAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAH8AAAAAAAAAAAAAAAAAAAB/AAAAAAAAAA' +
  'AAAAAAAAAAfwAAAAAAAAAAAAAAAAAAAH8AAAAAAAAAAAAAAAAAAAB/AAAAAAAAAAAAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAP' +
  '8AAAAAAAAAAAAAAAAAAAD/AAAAAAAAAAAAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAAD/AeAAAAAAAA' +
  'AAAAAAAAAA/wPwAAAAAAAAAAAAAAAAAP8D+AAAAAAAAAAAAAAAAAD/B/gAAAAAAAAAAAAAAAAA/wf4AAAAAAAAAAAAAAAAAP' +
  '8H+AAAAAAAAAAAAAAAAAD/B/gAAAAAAAAAAAAAAAAA/4f4AAAAAAAAAAAAAAAAAP/P+AAAAAAAAAAAAAAAAAB///AAAAAAAA' +
  'AAAAAAAAAAf//wAAAAAAAAAAAAAAAAAD//4AAAAAAAAAAAAAAAAAA//+AAAAAAAAAAAAAAAAAAH//AAAAAAAAAAAAAAAAAAA' +
  'f/AAAAAAAAAAAAAAAAAAAB/AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── key — 비트맵 실루엣(128×128, 잉크 20.6%)
//   원본 마스크: key.png
var MASK_KEY = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAD/AAAAAAAAAAAAAAAAAAAH/+AAAAAAAAAAAAAAAAAAH//4AAAAAAAAAAAAAAAAAD///AAAAAAAAA' +
  'AAAAAAAAB///4AAAAAAAAAAAAAAAAA////AAAAAAAAAAAAAAAAAf///4AAAAAAAAAAAAAAAAP/w//AAAAAAAAAAAAAAAAD/w' +
  'D/4AAAAAAAAAAAAAAAB/4Af+AAAAAAAAAAAAAAAA/8AD/4AAAAAAAAAAAAAAD//AA//wAAAAAAAAAAAAAH//wAP//gAAAAAA' +
  'AAAAAAH//8AD//+AAAAAAAAAAAAH///AA///4AAAAAAAAAAAD///wAP///gAAAAAAAAAAB///8AD///8AAAAAAAAAAB////g' +
  'B////gAAAAAAAAAAf///8A////8AAAAAAAAAAP//////////gAAAAAAAAAH/8A////AP/4AAAAAAAAAD/8AD//+AA//AAAAA' +
  'AAAAA/+AAP//AAD/4AAAAAAAAAf+AAB//gAAf+AAAAAAAAAH/gAAP/wAAD/gAAAAAAAAD/wAAD/4AAA/8AAAAAAAAA/4AAAf' +
  '+AAAH/AAAAAAAAAP+AAAH/AAAB/wAAAAAAAAH/AAAA/wAAAP+AAAAAAAAB/wAAAP8AAAD/gAAAAAAAAf8AAAD/AAAA/4AAAA' +
  'AAAAH/AAAA/gAAAP+AAAAAAAAB/wAAAP4AAAD/gAAAAAAAAf8AAAD+AAAA/4AAAAAAAAH/AAAA/wAAAP+AAAAAAAAB/wAAAP' +
  '8AAAD/gAAAAAAAAf8AAAD/AAAA/4AAAAAAAAD/gAAB/wAAAf8AAAAAAAAA/4AAAf+AAAH/AAAAAAAAAP/AAAP/wAAD/wAAAA' +
  'AAAAB/4AAH/8AAB/4AAAAAAAAAf+AAB//gAA/+AAAAAAAAAD/4AA//8AAf/AAAAAAAAAA//AA/gfwAP/wAAAAAAAAAH/8A/w' +
  'D/AP/4AAAAAAAAAA////4Af///8AAAAAAAAAAH///8AD///+AAAAAAAAAAA////AA////AAAAAAAAAAAH///wAP///gAAAAA' +
  'AAAAAA///8AD///wAAAAAAAAAAAD///AB///wAAAAAAAAAAAAP//4Af//wAAAAAAAAAAAAAf//AP//AAAAAAAAAAAAAAAP/4' +
  'P/4AAAAAAAAAAAAAAAA////8AAAAAAAAAAAAAAAAP///+AAAAAAAAAAAAAAAAB////AAAAAAAAAAAAAAAAAH///AAAAAAAAA' +
  'AAAAAAAAA///gAAAAAAAAAAAAAAAAAf//8AAAAAAAAAAAAAAAAAf///wAAAAAAAAAAAAAAAAH///+AAAAAAAAAAAAAAAAD//' +
  '//gAAAAAAAAAAAAAAAA////4AAAAAAAAAAAAAAAAP///+AAAAAAAAAAAAAAAAB////gAAAAAAAAAAAAAAAAP///wAAAAAAAA' +
  'AAAAAAAAA///gAAAAAAAAAAAAAAAAAH//wAAAAAAAAAAAAAAAAAB//8AAAAAAAAAAAAAAAAAAf//AAAAAAAAAAAAAAAAAAH/' +
  '/wAAAAAAAAAAAAAAAAAB//8AAAAAAAAAAAAAAAAAAf//AAAAAAAAAAAAAAAAAAH//wAAAAAAAAAAAAAAAAAB//8AAAAAAAAA' +
  'AAAAAAAAAf//AAAAAAAAAAAAAAAAAAH//wAAAAAAAAAAAAAAAAAB//8AAAAAAAAAAAAAAAAAAf//AAAAAAAAAAAAAAAAAAH/' +
  '/wAAAAAAAAAAAAAAAAAB//8AAAAAAAAAAAAAAAAAAf//AAAAAAAAAAAAAAAAAAH//wAAAAAAAAAAAAAAAAAB//8AAAAAAAAA' +
  'AAAAAAAAAf//AAAAAAAAAAAAAAAAAAP//8AAAAAAAAAAAAAAAAAP///gAAAAAAAAAAAAAAAAD///8AAAAAAAAAAAAAAAAB//' +
  '////8AAAAAAAAAAAAAAP//////gAAAAAAAAAAAAAD//////4AAAAAAAAAAAAAAP/////+AAAAAAAAAAAAAAB//////gAAAAA' +
  'AAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB/////+AAAAAAAAAAAAAAAf///+AAAAAAAAAAAAAAAAH/' +
  '///gAAAAAAAAAAAAAAAB////4AAAAAAAAAAAAAAAAf///+AAAAAAAAAAAAAAAAH////gAAAAAAAAAAAAAAAB/////+AAAAAA' +
  'AAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/' +
  '////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH//4AAAAAAAAAAAAAAAAAB//+AAAAAAAAA' +
  'AAAAAAAAAf//AAAAAAAAAAAAAAAAAAD//wAAAAAAAAAAAAAAAAAA//4AAAAAAAAAAAAAAAAAAH/8AAAAAAAAAAAAAAAAAAA/' +
  '+AAAAAAAAAAAAAAAAAAAD+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── lightbulb — 비트맵 실루엣(128×128, 잉크 30.6%)
//   원본 마스크: lightbulb.png
var MASK_LIGHTBULB = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAP/wAAAAAAAAAAAAAAAAAAf//wAAAAAAAAAAAAAAAAA////gAAAAAAAAAAAAAAAB////+AAAAAAAA' +
  'AAAAAAAB/////4AAAAAAAAAAAAAAA//////gAAAAAAAAAAAAAA//////8AAAAAAAAAAAAAAf//////wAAAAAAAAAAAAAf///' +
  '///+AAAAAAAAAAAAAP/wf////wAAAAAAAAAAAAH/wD////+AAAAAAAAAAAAD/wAf////wAAAAAAAAAAAB/4AD////+AAAAAA' +
  'AAAAAAf8AAf////wAAAAAAAAAAAP8AAH////8AAAAAAAAAAAH+AAB/////gAAAAAAAAAAD/AAAf////8AAAAAAAAAAA/gAAH' +
  '/////gAAAAAAAAAAf4AAD/////4AAAAAAAAAAH8AAA//////AAAAAAAAAAD+AAA//////wAAAAAAAAAA/gAAP/////+AAAAA' +
  'AAAAAfwAAH//////gAAAAAAAAAH8AAD//////4AAAAAAAAAD+AAB///////AAAAAAAAAA/gAB///////wAAAAAAAAAP4AA//' +
  '/////8AAAAAAAAAD8AAP///////gAAAAAAAAB/AAH///////4AAAAAAAAAfwAD///////+AAAAAAAAAH8AB////////gAAAA' +
  'AAAAB/AAf///////4AAAAAAAAAf4AP////////AAAAAAAAAH+AH////////wAAAAAAAAB/wD////////8AAAAAAAAAf/B///' +
  '//////AAAAAAAAAH///////////wAAAAAAAAB///////////8AAAAAAAAAf///////////AAAAAAAAAH///////////gAAAA' +
  'AAAAB///////////4AAAAAAAAAf//////////+AAAAAAAAAH///////////gAAAAAAAAA///////////4AAAAAAAAAP/////' +
  '/////8AAAAAAAAAD///////////AAAAAAAAAA///////////wAAAAAAAAAH//////////4AAAAAAAAAB//////////+AAAAA' +
  'AAAAAP//////////gAAAAAAAAAD//////////wAAAAAAAAAA//////////4AAAAAAAAAAH/////////+AAAAAAAAAAA/////' +
  '/////AAAAAAAAAAAP/////////wAAAAAAAAAAB/////////4AAAAAAAAAAAP////////8AAAAAAAAAAAD////////+AAAAAA' +
  'AAAAAAf////////AAAAAAAAAAAAD////////wAAAAAAAAAAAAf///////4AAAAAAAAAAAAD///////8AAAAAAAAAAAAAf///' +
  '///+AAAAAAAAAAAAAD///////AAAAAAAAAAAAAA///////wAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAA//////8AAAAAAA' +
  'AAAAAAAP//////AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAA///' +
  '///AAAAAAAAAAAAAAAP/////wAAAAAAAAAAAAAAD/////8AAAAAAAAAAAAAAAf/////AAAAAAAAAAAAAAAH/////wAAAAAAA' +
  'AAAAAAAB/////8AAAAAAAAAAAAAAAf/////AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAB/////4AAAAAAAAAAAAAAAf//' +
  '//+AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAA/////wAAAAAAAAAAAAAAAP////4AAAAAAAAAAAAAAAB////+AAAAAAAA' +
  'AAAAAAAAf////gAAAAAAAAAAAAAAAH////4AAAAAAAAAAAAAAAB////+AAAAAAAAAAAAAAAAf////gAAAAAAAAAAAAAAAP//' +
  '//8AAAAAAAAAAAAAAAD/////AAAAAAAAAAAAAAAA/////wAAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAD/////AAAAAAAA' +
  'AAAAAAAA/////wAAAAAAAAAAAAAAAH////8AAAAAAAAAAAAAAAD/////AAAAAAAAAAAAAAAA/////wAAAAAAAAAAAAAAAP//' +
  '//8AAAAAAAAAAAAAAAD/////AAAAAAAAAAAAAAAA/////wAAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAD/////AAAAAAAA' +
  'AAAAAAAA/////wAAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAD/////AAAAAAAAAAAAAAAA/////wAAAAAAAAAAAAAAAH//' +
  '//8AAAAAAAAAAAAAAAB////+AAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAA////AAAAAAAAAAAAAAAAAD///gAAAAAAAA' +
  'AAAAAAAAAf//wAAAAAAAAAAAAAAAAAH//4AAAAAAAAAAAAAAAAAA//8AAAAAAAAAAAAAAAAAAH/+AAAAAAAAAAAAAAAAAAA/' +
  '/AAAAAAAAAAAAAAAAAAAD/AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── scissors — 비트맵 실루엣(128×128, 잉크 31.5%)
//   원본 마스크: scissors.png
var MASK_SCISSORS = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAD4AAAAAAAAAAAAAfAAAAAB/gAAAAAAAAAAAAf8AAAAB/8AAAAAAAAAAAAP/gAAAAf/gAAAAAAAAAAAH/8AA' +
  'AAP/8AAAAAAAAAAAD//AAAAD//gAAAAAAAAAAB//4AAAA//8AAAAAAAAAAA//+AAAAf//gAAAAAAAAAAf//gAAAD//8AAAAA' +
  'AAAAAH//4AAAA///gAAAAAAAAAD//+AAAAP//8AAAAAAAAAB///gAAAD///AAAAAAAAAA///wAAAAf//4AAAAAAAAAf//8AA' +
  'AAH///AAAAAAAAAP//+AAAAB///4AAAAAAAAH///gAAAAP///AAAAAAAAD///wAAAAD///4AAAAAAAA///8AAAAAf///AAAA' +
  'AAAAf//+AAAAAH///4AAAAAAAP///gAAAAA///+AAAAAAAH///wAAAAAP///wAAAAAAD///8AAAAAB///+AAAAAAB///+AAA' +
  'AAAf///wAAAAAA////gAAAAAD///+AAAAAAf///wAAAAAAf///wAAAAAH///8AAAAAAH///+AAAAAD///+AAAAAAA////gAA' +
  'AAB////AAAAAAAH///8AAAAA////wAAAAAAB////gAAAAf///4AAAAAAAP///8AAAAH///8AAAAAAAB////gAAAD///+AAAA' +
  'AAAAf///8AAAB////gAAAAAAAD////AAAA////wAAAAAAAAf///4AAAf///4AAAAAAAAD////AAAP///8AAAAAAAAAf///4A' +
  'AH////AAAAAAAAAH////AAB////gAAAAAAAAA////wAA////wAAAAAAAAAH///+AAf///4AAAAAAAAAA////wAP///8AAAAA' +
  'AAAAAH///+AH///+AAAAAAAAAAA////wB////AAAAAAAAAAAP///8A////gAAAAAAAAAAB////gf///wAAAAAAAAAAAP///8' +
  'P///4AAAAAAAAAAAB////j///8AAAAAAAAAAAAP///9///+AAAAAAAAAAAAB////////AAAAAAAAAAAAAP///////gAAAAAA' +
  'AAAAAAB///////wAAAAAAAAAAAAAP//////4AAAAAAAAAAAAAB//////8AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAA///' +
  '///AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAA/////wAAAAAAAAAAAAAAAH////4AAAAAAAAAAAAAAAA////8AAAAAAAA' +
  'AAAAAAAAH///+AAAAAAAAAAAAAAAAB////gAAAAAAAAAAAAAAAAf///4AAAAAAAAAAAAAAAAH///+AAAAAAAAAAAAAAAAA//' +
  '//gAAAAAAAAAAAAAAAAP///4AAAAAAAAAAAAAAAAH////AAAAAAAAAAAAAAAAD////4AAAAAAAAAAAAAAAB/////AAAAAAAA' +
  'AAAAAAAA/////4AAAAAAAAAAAAAAAf////+AAAAAAAAAAAAAAAP/////wAAAAAAAAAAAAAAH/////+AAAAAAAAAAAf+AD///' +
  '///wAf/AAAAAAA//+B//////+A//+AAAAAA///4///gf//5///4AAAAA///////wD///////gAAAAf//////4Af//////8AA' +
  'AAf//////+AD///////gAAAP///////AAf//////8AAAH///////gAD///////gAAD//gP///wAA////Af/8AAA//gAf//4A' +
  'AH///AA//gAAf/gAB//8AAA///AAH/4AAP/gAAP/+AAAH//AAAf/AAD/wAAB//gAAA//gAAD/wAB/4AAAP/wAAAP/wAAAf+A' +
  'Af+AAAD/4AAAB/8AAAH/gAH/AAAAf+AAAAP+AAAA/8AD/gAAAH/AAAAD/gAAAH/AA/4AAAA/wAAAA/wAAAB/wAP8AAAAP8AA' +
  'AAP8AAAAP+AH/AAAAD/AAAAD/AAAAD/gB/wAAAAf4AAAA/wAAAA/4Af8AAAAH+AAAAP4AAAAP+AH+AAAAB/gAAAH+AAAAD/g' +
  'B/gAAAAf4AAAB/gAAAA/4Af4AAAAH+AAAAf4AAAAP+AH+AAAAB/gAAAD/AAAAD/gB/gAAAAfwAAAA/wAAAA/4Af8AAAAP8AA' +
  'AAP8AAAAP+AH/AAAAD/AAAAD/AAAAD/gB/wAAAA/wAAAA/wAAAA/4AP8AAAAf4AAAAH+AAAAP8AD/gAAAH+AAAAB/gAAAH/A' +
  'A/4AAAD/gAAAAf8AAAB/wAH/AAAA/wAAAAD/gAAA/4AB/wAAAf8AAAAA/4AAAf+AAP+AAAP+AAAAAH/AAAH/AAD/wAAH/AAA' +
  'AAA/4AAD/wAAf/AAH/wAAAAAP/gAD/4AAH/8AH/4AAAAAB/+AD/8AAA//wH/8AAAAAAP/8D/+AAAH////+AAAAAAB/////AA' +
  'AA/////AAAAAAAP////gAAAH////AAAAAAAB////wAAAAf///gAAAAAAAH///4AAAAD///gAAAAAAAAf//4AAAAAH//gAAAA' +
  'AAAAB//4AAAAAAH+AAAAAAAAAAB/gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── hammer — 비트맵 실루엣(128×128, 잉크 30.4%)
//   원본 마스크: hammer.png
var MASK_HAMMER = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAPwAAAAAAAAAAAAAAAAAAAf+AAAAAAAAAAAAAAAAAAA//wAAAAAAAAAAAAAAAAAB//+AAAAAAAAA' +
  'AAAAAAAAD///wAAAAAAAAAAAAAAAAH////AAAAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAH///////gAAAAAAAAAAAAB///' +
  '/////AAAAAAAAAAAAA////////+AAAAAAAAAAAAP////////4AAAAAAAAAAAD/////////gAAAAAAAAAAA/////////+AAAA' +
  'AAAAAAAf/////////wAAAAAAAAAAH/////////+AAAAAAAAAAD//////////wAAAAAAAHwD//////////+AAAAAAAP/3////' +
  '///////wAAAAAAf/////////////+AAAAAAf//////////////wAAAAAf//////////////+AAAAA////////////////wAA' +
  'AAf///////////////+AAAAH////////////////gAAAD////////////8Af/8AAAA////////////4AA//AAAAH////////' +
  '///8AAB/4AAAB///////////+AAAH+AAAAf///////////gAAAfgAAAD///////////wAAAD4AAAA///////////8AAAAMAA' +
  'AAH///////////AAAAAAAAAB///////////wAAAAAAAAAP//////////8AAAAAAAAAD///////////AAAAAAAAAA////////' +
  '///gAAAAAAAAAH//////////4AAAAAAAAAB//////////8AAAAAAAAAAf//////////AAAAAAAAAAD//////////wAAAAAAA' +
  'AAA//////////8AAAAAAAAAAH//////////AAAAAAAAAAB//////////wAAAAAAAAAAf////Af///+AAAAAAAAAAD////AD/' +
  '///gAAAAAAAAAA////gAP///8AAAAAAAAAAH///wAAf///AAAAAAAAAAB///8AAD///4AAAAAAAAAAf//+AAAf//+AAAAAAA' +
  'AAAD///AAAD///wAAAAAAAAAA///AAAAf//8AAAAAAAAAAP/+AAAAH///gAAAAAAAAAB/+AAAAB///4AAAAAAAAAAf8AAAAA' +
  'P//+AAAAAAAAAAD4AAAAAD///wAAAAAAAAAAAAAAAAA///8AAAAAAAAAAAAAAAAAH///gAAAAAAAAAAAAAAAAB///4AAAAAA' +
  'AAAAAAAAAAAf///AAAAAAAAAAAAAAAAAD///4AAAAAAAAAAAAAAAAA///+AAAAAAAAAAAAAAAAAP///wAAAAAAAAAAAAAAAA' +
  'D///8AAAAAAAAAAAAAAAAAf///gAAAAAAAAAAAAAAAAH///4AAAAAAAAAAAAAAAAB////AAAAAAAAAAAAAAAAAf///wAAAAA' +
  'AAAAAAAAAAAD///+AAAAAAAAAAAAAAAAA////gAAAAAAAAAAAAAAAAP///8AAAAAAAAAAAAAAAAD////AAAAAAAAAAAAAAAA' +
  'Af///4AAAAAAAAAAAAAAAAH///+AAAAAAAAAAAAAAAAB////wAAAAAAAAAAAAAAAAf///8AAAAAAAAAAAAAAAAH////gAAAA' +
  'AAAAAAAAAAAA////8AAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAD////4AAAAAAAAAAAAAAAA////+AAAAAAAAAAAAAAA' +
  'AP////wAAAAAAAAAAAAAAAB////8AAAAAAAAAAAAAAAAf////AAAAAAAAAAAAAAAAH////4AAAAAAAAAAAAAAAB////+AAAA' +
  'AAAAAAAAAAAAP////wAAAAAAAAAAAAAAAD////8AAAAAAAAAAAAAAAA/////gAAAAAAAAAAAAAAAP////4AAAAAAAAAAAAAA' +
  'AB/////AAAAAAAAAAAAAAAAf////4AAAAAAAAAAAAAAAH////+AAAAAAAAAAAAAAAA/////wAAAAAAAAAAAAAAAP////8AAA' +
  'AAAAAAAAAAAAD/////gAAAAAAAAAAAAAAAf////4AAAAAAAAAAAAAAAH/////AAAAAAAAAAAAAAAB/////wAAAAAAAAAAAAA' +
  'AAf////+AAAAAAAAAAAAAAAD/////gAAAAAAAAAAAAAAA/////8AAAAAAAAAAAAAAAP/////AAAAAAAAAAAAAAAB/////4AA' +
  'AAAAAAAAAAAAAf////+AAAAAAAAAAAAAAAH/////AAAAAAAAAAAAAAAB/////wAAAAAAAAAAAAAAAP////8AAAAAAAAAAAAA' +
  'AAD////+AAAAAAAAAAAAAAAA/////gAAAAAAAAAAAAAAAH////wAAAAAAAAAAAAAAAB////4AAAAAAAAAAAAAAAAf///8AAA' +
  'AAAAAAAAAAAAAH///8AAAAAAAAAAAAAAAAA///8AAAAAAAAAAAAAAAAAP//8AAAAAAAAAAAAAAAAAB//4AAAAAAAAAAAAAAA' +
  'AAAP/wAAAAAAAAAAAAAAAAAAAfgAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── glasses — 비트맵 실루엣(128×128, 잉크 15.4%)
//   원본 마스크: glasses.png
var MASK_GLASSES = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAB///+AAAAAAB///+AAAAAf////+AAAAAD/////gAAA//////8AA' +
  'AAH//////AAB///////wAAAP//////+AA////////AAAP///////wAf///////8AAP///////+AH///////////////////g' +
  'B////AP////////gH///4Af//wAAB//////wAAB//+AH//wAAAH/////wAAAH//gB//wAAAAf////wAAAA//4Af/4AAAAD//' +
  '//wAAAAH/+AH/8AAAAAf///4AAAAA//gB//AAAAAD///+AAAAAH/4Af/gAAAAA////AAAAAB/+AH/4AAAAAH///wAAAAAP/g' +
  'A/8AAAAAB///4AAAAAD/wAH/AAAAAAf//+AAAAAA/4AA/wAAAAAD///gAAAAAP8AAH8AAAAAA///wAAAAAD+AAB/AAAAAAP/' +
  '/8AAAAAA/gAAfwAAAAAD///AAAAAAP4AAH8AAAAAA///wAAAAAD+AAB/AAAAAAP//8AAAAAA/gAAfwAAAAAD///AAAAAAP4A' +
  'AH8AAAAAA///wAAAAAD+AAB/AAAAAAP//8AAAAAA/gAAPwAAAAAD///AAAAAAPwAAD8AAAAAA///4AAAAAD8AAA/gAAAAAf/' +
  '/+AAAAAA/AAAP4AAAAAH///gAAAAAfwAAD+AAAAAB///4AAAAAH8AAAfgAAAAAf///AAAAAB+AAAH4AAAAAP///wAAAAAfgA' +
  'AB/AAAAAD/AP8AAAAAP4AAAPwAAAAB/gB/gAAAAD8AAAD+AAAAAfwAP4AAAAB/AAAA/gAAAAP8AD/AAAAAfwAAAH8AAAAD+A' +
  'AfwAAAAP4AAAB/gAAAB/gAH+AAAAH+AAAAP4AAAA/wAA/wAAAB/AAAAD/gAAAf8AAP+AAAA/wAAAAf8AAAP+AAB/wAAA/4AA' +
  'AAD/gAAH/AAAP+AAAf8AAAAA//AAP/gAAB/8AAf/AAAAAH//B//4AAAf/8D//gAAAAA/////8AAAD/////wAAAAAH////8AA' +
  'AAP////4AAAAAAf///+AAAAB////4AAAAAAD///+AAAAAH///8AAAAAAAH//+AAAAAAf//8AAAAAAAAf/+AAAAAAB//4AAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── envelope — 비트맵 실루엣(128×128, 잉크 42.1%)
//   원본 마스크: envelope.png
var MASK_ENVELOPE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAf//////////////////+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AH////AAAAAAAAAAB////g' +
  'B///8AAAAAAAAAAAD///4Af//+AAAAAAAAAAAAf//+AH///AAAAAAAAAAAAD///gB///gAAAAAAAAAAAAf//4Af//4AAAAAA' +
  'AAAAAAH//+AH//+AAAAAAAAAAAAA///gB///gAAAAAAAAAAAAP//4Af//4AAAAAAAAAAAAD//+AH//+AAAAAAAAAAAAA///g' +
  'B///gAAAAAAAAAAAAf//4Af//8AAAAAAAAAAAAH//+AH///AAAAAAAAAAAAD///gB///8AAAAAAAAAAAB///4Af///AAAAAA' +
  'AAAAAA///+AH///8AAAAAAAAAAAf///gB////AAAAAAAAAAAP///4Af///8AAAAAAAAAAH///+AH////gAAAAAAAAAD////g' +
  'B////8AAAAAAAAAB////4Af////gAAAAAAAAB////+AH////8AAAAAAAAA/////gB/////gAAAAAAAAf////4Af////8AAAA' +
  'AAAAP////+AH/////gAAAAAAAH/////gB/////8AAAAAAAD/////4Af/////gAAAAAAB/////+AH/////8AAAAAAA//////g' +
  'B//////gAAAAAAf/////4Af/////8AD//wAP/////+AH//////gD///AH//////gB//////+D///8H//////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4Af//////////////////+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4Af/////4P////8H/////+AH/////4A////8Af/////gB/////8AH///+AD/////4Af////+AB//' +
  '//AAf////+AH/////AAP///gAD/////gB/////gAA///wAAf////4Af////wAAAAAAAAD////+AH////4AAAAAAAAAP////g' +
  'B////4AAAAAAAAAB////4Af///8AAAAAAAAAAP///+AH///+AAAAAAAAAAB////gB////AAAAAAAAAAAP///4Af///gAAAAA' +
  'AAAAAB///+AH///wAAAAAAAAAAAP///gB///4AAAAAAAAAAAB///4Af//8AAAAAAAAAAAAP//+AH//+AAAAAAAAAAAAB///g' +
  'B///AAAAAAAAAAAAAP//4Af//wAAAAAAAAAAAAD//+AH//8AAAAAAAAAAAAAf//gB///AAAAAAAAAAAAAH//4Af//gAAAAAA' +
  'AAAAAAB//+AH//8AAAAAAAAAAAAAf//gB///AAAAAAAAAAAAAP//4Af//wAAAAAAAAAAAAD//+AH//+AAAAAAAAAAAAB///g' +
  'B///wAAAAAAAAAAAA///4Af///AAAAAAAAAAAAf//+AH///////////////////gB///////////////////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── gift — 비트맵 실루엣(128×128, 잉크 50.7%)
//   원본 마스크: gift.png
var MASK_GIFT = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAD8AAAAAAB/gAAAAAAAAAAH/4AAAAAB/+AAAAAAAAAAH//gAAAAB//4AAAAAAAAAD//8AAAAA///gAAAA' +
  'AAAAB///gAAAAf//8AAAAAAAAA///8AAAAP///gAAAAAAAAf///gAAAH///4AAAAAAAAP///8AAAD////AAAAAAAAH////gA' +
  'AB////4AAAAAAAB////8AAA////+AAAAAAAA/wH//gAAP/+A/wAAAAAAAP4AP/4AAH/8AH8AAAAAAAD+AB//AAD/+AA/AAAA' +
  'AAAB/AAH/wAA/+AAP4AAAAAAAfwAA/+AAf/AAD+AAAAAAAH+AAH/gAH/gAA/gAAAAAAB/gAA/8AD/wAAf4AAAAAAAf4AAP/A' +
  'A/8AAH+AAAAAAAH/AAB/4Af+AAD/gAAAAAAB/4AAP+AH/AAB/4AAAAAAAf/AAB/wD/gAA/+AAAAAAAD/4AAf+9/4AAf/AAAA' +
  'AAAA//AAD///8AAP/wAAAAAAAP/8AAf//+AAH/8AAAAAAAB//gAH///gAH/+AAAAAAAAP/+AA///wAH//AAAAAAAAB//8AP/' +
  '/8AH//gAAAAAAAAP//wB///AP//wAAAAAAAAB///w///wf//4AAAAAAAAAP//////////8AAAAAAAAAA//////////8AAAAA' +
  'AAAAAB/////////8AAAAAAAAAAAP////////8AAAAAAAAAAAH/////////gAAAAAAAAAAD/////////8AAAAAAAP//////P/' +
  '/8//////8AAAH//////B//+D//////gAAD//////gf//gP/////8AAA//////gD//wB//////AAAP/////wA//8AH/////wA' +
  'AD/////wAf//gA/////8AAA/////4AH//8AD/////AAAP////4AD4AfAAf////wAAD////4AB8AD4AB////8AAA////8AA/A' +
  'A/AAP////AAAP///+AAPgAHwAB////wAAD////gAH4AA+AAf///8AAA////+AD+AAPwAf////AAAP////4A/gAD8Af////wA' +
  'AD/////Af4AA/gP////8AAA/////4P+AAP8H/////AAAP////+D/gAD/B/////wAAD/////x/4AA/4/////8AAA/////+/+A' +
  'AP/P/////AAAP///////gAD///////wAAD///////4AA///////8AAAf//////+AAP//////+AAAD///////gAD///////AA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAB//////8A' +
  'AP//////gAAAA///////gAH//////8AAAAP//////4AB///////AAAAD//////+AAf//////wAAAA///////gAH//////8AA' +
  'AAP//////4AB///////AAAAD//////+AAf//////wAAAA///////gAH//////8AAAAP//////4AB///////AAAAD//////+A' +
  'Af//////wAAAA///////gAH//////8AAAAP//////4AB///////AAAAD//////+AAf//////wAAAA///////gAH//////8AA' +
  'AAP//////4AB///////AAAAD//////+AAf//////wAAAA///////gAH//////8AAAAP//////4AB///////AAAAD//////+A' +
  'Af//////wAAAA///////gAH//////8AAAAP//////4AB///////AAAAD//////+AAf//////wAAAA///////gAH//////8AA' +
  'AAP//////4AB///////AAAAD//////+AAf//////wAAAA///////gAH//////8AAAAP//////4AB///////AAAAD//////+A' +
  'Af//////wAAAA///////gAH//////8AAAAP//////4AB///////AAAAD//////+AAf//////wAAAA///////gAH//////8AA' +
  'AAP//////4AB///////AAAAD//////+AAf//////wAAAA///////gAH//////8AAAAP//////4AB///////AAAAD//////+A' +
  'Af//////wAAAA///////gAH//////8AAAAP//////4AB///////AAAAD//////+AAf//////wAAAA///////gAH//////8AA' +
  'AAP//////4AB///////AAAAD//////+AAf//////wAAAA///////gAH//////8AAAAP//////4AB///////AAAAD//////+A' +
  'Af//////wAAAA///////gAH//////8AAAAP//////4AB///////AAAAD//////+AAf//////wAAAA///////gAH//////8AA' +
  'AAP//////4AB///////AAAAB//////+AAf//////gAAAAf//////gAH//////4AAAAD//////4AB//////8AAAAAf/////+A' +
  'Af/////+AAAAAD//////gAH//////AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── apple — 비트맵 실루엣(128×128, 잉크 56.2%)
//   원본 마스크: apple.png
var MASK_APPLE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAB//4AAAAAAAAAAAAAAAAAH///8AAAAAAAAAAAAAAAAH////AAAAAAAAAAAAAAAAP////wAA' +
  'AAAAAAAAAAAAAH////8AAAAAAAAAAADwAAH////+AAAAAAAAAAAD+AAH/////gAAAAAAAAAAB/gAD/////4AAAAAAAAAAA/8' +
  'AB/////8AAAAAAAAAAAf/AA//////AAAAAAAAAAAH/4Af/////gAAAAAAAAAAA/+AH/////4AAAAAAAAAAAH/wD/////8AAA' +
  'AAAAAAAAA/8B/////+AAAAAAAAAAAAH/g//////gAAAAAAAAAAAB/4P/////wAAAAAAAAAAAAP+H/////4AAAAAAAAAAAAB/' +
  'x/////8AAAAAAAAAAAAAf//////+AAAAAAAAAAAAAD///////AAAAAAAAAAAAAA///////gAAAAAAAAAAAAAH//////gAAAA' +
  'AAAAAAAAAB//////gAAAAAAAAAAAAAAf/////gAAAAAAAAAAAAAAD/////wAAAAAAAAAAAAAAA/////+AAAAAAAAAAA/+AAP' +
  '/////gAAAAAAAAAB//8AD//////AAAAAAAAAD///4A//////+AAAAAAAAD////gP//////4AAAAAAAB/////D///////AAAA' +
  'AAAB/////////////4AAAAAAA//////////////AAAAAAAf/////////////8AAAAAAP//////////////AAAAAAH///////' +
  '///////4AAAAAD///////////////AAAAAA///////////////4AAAAAf//////////////+AAAAAH///////////////wAA' +
  'AAD///////////////8AAAAB////////////////gAAAAf///////////////4AAAAH////////////////AAAAD////////' +
  '////////wAAAA////////////////+AAAAf////////////////gAAAH////////////////4AAAB/////////////////AA' +
  'AAf////////////////wAAAP////////////////8AAAD/////////////////AAAA/////////////////wAAAP////////' +
  '////////+AAAD/////////////////gAAB/////////////////4AAAf////////////////+AAAH/////////////////gA' +
  'AB/////////////////4AAAf////////////////+AAAH/////////////////gAAB/////////////////4AAAf////////' +
  '////////+AAAH/////////////////gAAB/////////////////4AAAf////////////////+AAAD/////////////////gA' +
  'AA/////////////////wAAAP////////////////8AAAD/////////////////AAAA/////////////////wAAAP////////' +
  '////////8AAAB/////////////////AAAAf////////////////gAAAH////////////////4AAAB////////////////+AA' +
  'AAP////////////////gAAAD////////////////wAAAA////////////////8AAAAP////////////////AAAAB////////' +
  '////////wAAAAf///////////////4AAAAH///////////////+AAAAA////////////////gAAAAP///////////////wAA' +
  'AAD///////////////8AAAAAf///////////////AAAAAH///////////////gAAAAA///////////////4AAAAAP///////' +
  '///////8AAAAAB///////////////AAAAAAf//////////////gAAAAAD//////////////4AAAAAA//////////////8AAA' +
  'AAAH//////////////AAAAAAB//////////////gAAAAAAP/////////////4AAAAAAB/////////////8AAAAAAAf//////' +
  '//////+AAAAAAAD/////////////gAAAAAAAf////////////wAAAAAAAH////////////4AAAAAAAA////////////8AAAA' +
  'AAAAH////////////AAAAAAAAA////////////gAAAAAAAAP///////////wAAAAAAAAB///////////4AAAAAAAAAP/////' +
  '/////8AAAAAAAAAB//////////+AAAAAAAAAAP//////////AAAAAAAAAAA//////////gAAAAAAAAAAH/////////wAAAAA' +
  'AAAAAA/////////wAAAAAAAAAAAH////////4AAAAAAAAAAAAf///////4AAAAAAAAAAAAB///////8AAAAAAAAAAAAAH///' +
  '///4AAAAAAAAAAAAAAP/wAP/4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── icecream — 비트맵 실루엣(128×128, 잉크 24.1%)
//   원본 마스크: icecream.png
var MASK_ICECREAM = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAB///wAAAAAAAAAAAAAAAAB////gAAAAAAAAAAAAAAAD////+AAAAAAAA' +
  'AAAAAAAD/////4AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAB/wP///8AAAAAAAAAAAAAA/wB////gAAAAAAAAAAAAAfwAf' +
  '///8AAAAAAAAAAAAAPwAP////gAAAAAAAAAAAAH4AH////8AAAAAAAAAAAAD4AD/////gAAAAAAAAAAAB8AB/////8AAAAAA' +
  'AAAAAA+AA//////gAAAAAAAAAAAPAA//////4AAAAAAAAAAAHwAf//////AAAAAAAAAAAD4AP//////4AAAAAAAAAAA8AH//' +
  '////+AAAAAAAAAAAfAD/////x/wAAAAAAAAAAHgA/////4P8AAAAAAAAAAB4Af////+B/AAAAAAAAAAA8AP/////wP4AAAAA' +
  'AAAAAPAD/////8D+AAAAAAAAAADwB//////AfgAAAAAAAAAA8A//////4H8AAAAAAAAAAfAP//////A/AAAAAAAAAAHwH///' +
  '///wPwAAAAAAAAAB8D//////+H8AAAAAAAAAAfA///////7/AAAAAAAAAAH4/////////wAAAAAAAAAB//////////8AAAAA' +
  'AAAAAf//////////gAAAAAAAAAP//////////8AAAAAAAAAH///////////gAAAAAAAAD///////////8AAAAAAAAB//////' +
  '//////gAAAAAAAA////////////4AAAAAAAAP////////////AAAAAAAAD////////////wAAAAAAAB////////////8AAAA' +
  'AAAAff///////////AAAAAAAAH3///////////4AAAAAAAB8//Af///////+AAAAAAAAPH/AB////////AAAAAAAADweAAP/' +
  '//////wAAAAAAAA8AAAB//////88AAAAAAAAHAAAAP//4B/+OAAAAAAAAB4AAwB//wAH/DgAAAAAAAAPAB+AP/wAAGBwAAAA' +
  'AAAAB4B/wAfwAAAA8AAAAAAAAAPz/8AAAAAAAeAAAAAAAAAB///AAAAIAAOAAAAAAAAAAP//4AAAfwAHAAAAAAAAAAB///AA' +
  'Af/ABwAAAAAAAAAAf//8AAf/4A4AAAAAAAAAAD///8A///A8AAAAAAAAAAAf/////////AAAAAAAAAAAD4P///////AAAAAA' +
  'AAAAAAeAf////h/gAAAAAAAAAAAHwH////wDwAAAAAAAAAAAA8B/gAP+B8AAAAAAAAAAAAPg/8AD/weAAAAAAAAAAAAD8f/g' +
  'B/+PgAAAAAAAAAAAAf/z8A/P34AAAAAAAAAAAAH/4fgfh/8AAAAAAAAAAAAB/8D8PwP/AAAAAAAAAAAAAP+Afn4B/gAAAAAA' +
  'AAAAAAD/AD/8AP4AAAAAAAAAAAAAfgAf+AD+AAAAAAAAAAAAAH4AD/AA/AAAAAAAAAAAAAB/AA/gAPwAAAAAAAAAAAAAP4AP' +
  '8AH4AAAAAAAAAAAAAD+AH/gD+AAAAAAAAAAAAAA/wD/8B/gAAAAAAAAAAAAAH+B8fgfwAAAAAAAAAAAAAB/w+D8P8AAAAAAA' +
  'AAAAAAAP8fAfn+AAAAAAAAAAAAAAD//gD//gAAAAAAAAAAAAAA//wAf/4AAAAAAAAAAAAAAHv4AD+8AAAAAAAAAAAAAAB5+A' +
  'A/PAAAAAAAAAAAAAAAf/gAP/gAAAAAAAAAAAAAAD/8AH/4AAAAAAAAAAAAAAA//gD/+AAAAAAAAAAAAAAAH58B8fAAAAAAAA' +
  'AAAAAAAB8Pg+HwAAAAAAAAAAAAAAAeB8fA4AAAAAAAAAAAAAAADwP/geAAAAAAAAAAAAAAAA8B/wHgAAAAAAAAAAAAAAAPgP' +
  '4DwAAAAAAAAAAAAAAAB4D+A8AAAAAAAAAAAAAAAAeA/gOAAAAAAAAAAAAAAAADwf8HgAAAAAAAAAAAAAAAA8P/hwAAAAAAAA' +
  'AAAAAAAAPnx88AAAAAAAAAAAAAAAAB/4P/AAAAAAAAAAAAAAAAAf8B/gAAAAAAAAAAAAAAAAH+AP4AAAAAAAAAAAAAAAAA/A' +
  'B8AAAAAAAAAAAAAAAAAPgAfAAAAAAAAAAAAAAAAAB4AHgAAAAAAAAAAAAAAAAAfAD4AAAAAAAAAAAAAAAAAH4A+AAAAAAAAA' +
  'AAAAAAAAA+AfAAAAAAAAAAAAAAAAAAPwPwAAAAAAAAAAAAAAAAAB+D4AAAAAAAAAAAAAAAAAAfh+AAAAAAAAAAAAAAAAAAH8' +
  '/AAAAAAAAAAAAAAAAAAA//wAAAAAAAAAAAAAAAAAAP/8AAAAAAAAAAAAAAAAAAB/+AAAAAAAAAAAAAAAAAAAf/gAAAAAAAAA' +
  'AAAAAAAAAH/wAAAAAAAAAAAAAAAAAAA/8AAAAAAAAAAAAAAAAAAAP+AAAAAAAAAAAAAAAAAAAB/gAAAAAAAAAAAAAAAAAAAP' +
  'wAAAAAAAAAAAAAAAAAAAB4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── cupcake — 비트맵 실루엣(128×128, 잉크 52.2%)
//   원본 마스크: cupcake.png
var MASK_CUPCAKE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAcAAAAAAAAAAAAAAAAAAAAHwAAAAAAAAAAAAAAAAAAAB/AAAAAAAAAAAAAAAAAAAAf8AAAAAAAAA' +
  'AAAAAAAAAAP/wAAAAAAAAAAAAAAAAAAD//AAAAAAAAAAAAAAAAAAB//4AAAAAAAAAAAAAAAAAA///AAAAAAAAAAAAAAAAAAf' +
  '//8AAAAAAAAAAAAAAAAAP///gAAAAAAAAAAAAAAAAH///8AAAAAAAAAAAAAAAAH////gAAAAAAAAAAAAAAAP////8AAAAAAA' +
  'AAAAAAAAf/////AAAAAAAAAAAAAAA//////4AAAAAAAAAAAAAA//////+AAAAAAAAAAAAAA///////wAAAAAAAAAAAAA////' +
  '///8AAAAAAAAAAAAA////////gAAAAAAAAAAAAf////////AAAAAAAAAAAAf////////+AAAAAAAAAAAP/////////wAAAAA' +
  'AAAAAH//////////AAAAAAAAAAD//////////4AAAAAAAAAA///////////AAAAAAAAAAf//////////4AAAAAAAAAP/////' +
  '//////AAAAAAAAAD///////////4AAAAAAAAB////////////AAAAAAAAAf///////////wAAAAAAAAH///////////+AAAA' +
  'AAAAD////////////gAAAAAAAA////////////4AAAAAAAAP////////////AAAAAAAAD////////////wAAAAAAAD//////' +
  '//////8AAAAAAAD/////////////AAAAAAAD/////////////4AAAAAAB//////////////AAAAAAA//////////////8AAA' +
  'AAAf//////////////gAAAAAf//////////////8AAAAAH///////////////gAAAAD///////////////8AAAAB////////' +
  '////////gAAAAf///////////////8AAAAP////////////////AAAAD////////////////4AAAB////////////////+AA' +
  'AAf////////////////wAAAP////////////////8AAAD/////////////////AAAA/////////////////4AAAP////////' +
  '////////+AAAD/////////////////gAAA/////////////////4AAAf////////////////+AAAH/////////////////gA' +
  'AA/////////////////4AAAP////////////////+AAAD/////////////////AAAA/////////////////wAAAP////////' +
  '////////8AAAB////////////////+AAAAf////////////////gAAAH////////////////4AAAA////////////////8AA' +
  'AAH///////////////+AAAAB////////////////gAAAAP///////////////wAAAAA///////////////4AAAAAH///////' +
  '///////4AAAAAAf/////////////4AAAAAAH/////////////+AAAAAAA//////////////AAAAAAAP/////////////wAAA' +
  'AAAD/////////////8AAAAAAAf/////////////AAAAAAAH/////////////gAAAAAAB/////////////4AAAAAAAf//////' +
  '//////+AAAAAAAD/////////////AAAAAAAA/////////////wAAAAAAAP////////////8AAAAAAAD/////////////AAAA' +
  'AAAAf////////////gAAAAAAAH////////////4AAAAAAAB////////////+AAAAAAAAP////////////AAAAAAAAD//////' +
  '//////wAAAAAAAA////////////8AAAAAAAAP////////////AAAAAAAAB////////////gAAAAAAAAf///////////4AAAA' +
  'AAAAH///////////+AAAAAAAAA////////////gAAAAAAAAP///////////wAAAAAAAAD///////////8AAAAAAAAA//////' +
  '//////AAAAAAAAAH///////////gAAAAAAAAB///////////4AAAAAAAAAf//////////+AAAAAAAAAD///////////gAAAA' +
  'AAAAA///////////wAAAAAAAAAP//////////8AAAAAAAAAD///////////AAAAAAAAAAf//////////gAAAAAAAAAH/////' +
  '/////4AAAAAAAAAB//////////+AAAAAAAAAAP//////////gAAAAAAAAAD//////////wAAAAAAAAAA//////////8AAAAA' +
  'AAAAAP//////////AAAAAAAAAAB//////////gAAAAAAAAAAf/////////4AAAAAAAAAAD/////////+AAAAAAAAAAAf////' +
  '////+AAAAAAAAAAAB////////+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── donut — 비트맵 실루엣(128×128, 잉크 55.9%)
//   원본 마스크: donut.png
var MASK_DONUT = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAH/gAAAAAAAAA' +
  'AAAAAAAAH///+AAAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAf/////8AAAAAAAAAAAAAA///////wAAAAAAAAAAAAA////' +
  '////gAAAAAAAAAAAB////////+AAAAAAAAAAAB/////////4AAAAAAAAAAA//////////AAAAAAAAAAA//////////8AAAAA' +
  'AAAAAf//////////wAAAAAAAAAf//////////+AAAAAAAAAP///////////wAAAAAAAAP////////////AAAAAAAAH//////' +
  '//////4AAAAAAAD/////////////AAAAAAAB/////////////4AAAAAAA//////////////AAAAAAAf/////////////4AAA' +
  'AAAP//////////////AAAAAAH//////////////4AAAAAD///////////////AAAAAB///////////////4AAAAAf///////' +
  '//gH///+AAAAAP////////+AAH///wAAAAH////////8AAAf//+AAAAD////////8AAAB///wAAAA////////+AAAAP//8AA' +
  'AAf////////AAAAB///gAAAH////////gAAAAP//4AAAD////////wAAAAB///AAAB////////4AAAAAP//4AAAf///////8' +
  'AAAAAB//+AAAP////////AAAAAAf//wAAD////////gAAAAAD//8AAB////////4AAAAAA///gAAf///////8AAAAAAH//4A' +
  'AH////////AAAAAAB//+AAD////////wAAAAAAf//wAA////////8AAAAAAH//8AAP////////AAAAAAB///AAH////////w' +
  'AAAAAAf//4AB////////8AAAAAAH//+AAf////////AAAAAAB///gAP////////wAAAAAAf//4AD////////8AAAAAAH///A' +
  'A/////////gAAAAAD///wAP////////4AAAAAA///8AH////////+AAAAAAf///AB/////////wAAAAAH///4Af////////8' +
  'AAAAAD///+AH/////////gAAAAB////gB/////////8AAAAAf///4Af/////////gAAAAP///+AH/////////8AAAAP////g' +
  'B//////////gAAAH////4Af/////////+AAAH////+AH//////////4AAH/////gB///////////wAP/////4Af/////////' +
  '/////////+AH///////////////////gB///////////////////4Af//////////////////+AH///////////////////g' +
  'B///////////////////wAf//////////////////8AD///////////////////AA///////////////////wAP/////////' +
  '/////////8AD//////////////////+AAf//////////////////gAH//////////////////4AB//////////////////8A' +
  'AP//////////////////AAD//////////////////wAA//////////////////4AAH/////////////////+AAB/////////' +
  '/////////gAAf/////////////////wAAD/////////////////8AAA/////////////////+AAAH/////////////////gA' +
  'AA/////////////////wAAAP////////////////8AAAB////////////////+AAAAP////////////////AAAAD////////' +
  '////////wAAAAf///////////////4AAAAD///////////////8AAAAA///////////////+AAAAAH///////////////gAA' +
  'AAA///////////////wAAAAAH//////////////4AAAAAA//////////////8AAAAAAH/////////////+AAAAAAA///////' +
  '///////AAAAAAAH/////////////gAAAAAAA/////////////wAAAAAAAH////////////wAAAAAAAA////////////4AAAA' +
  'AAAAD///////////8AAAAAAAAAf//////////8AAAAAAAAAB//////////+AAAAAAAAAAP/////////+AAAAAAAAAAA/////' +
  '////+AAAAAAAAAAAD////////+AAAAAAAAAAAAP///////+AAAAAAAAAAAAA///////+AAAAAAAAAAAAAA//////8AAAAAAA' +
  'AAAAAAAD/////4AAAAAAAAAAAAAAAB////gAAAAAAAAAAAAAAAAAH/gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── airplane — 비트맵 실루엣(128×128, 잉크 24.3%)
//   원본 마스크: airplane.png
var MASK_AIRPLANE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAA8AAAAAAAAAAAAAAAAAAAAfgAAAAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAAH/gAAAAAAAAA' +
  'AAAAAAAAAD/8AAAAAAAAAAAAAAAAAAA//AAAAAAAAAAAAAAAAAAAf/4AAAAAAAAAAAAAAAAAAH/+AAAAAAAAAAAAAAAAAAB/' +
  '/gAAAAAAAAAAAAAAAAAA//8AAAAAAAAAAAAAAAAAAP//AAAAAAAAAAAAAAAAAAD//wAAAAAAAAAAAAAAAAAA//+AAAAAAAAA' +
  'AAAAAAAAAf//gAAAAAAAAAAAAAAAAAH//4AAAAAAAAAAAAAAAAAB//+AAAAAAAAAAAAAAAAAAf//gAAAAAAAAAAAAAAAAAH/' +
  '/4AAAAAAAAAAAAAAAAAB//+AAAAAAAAAAAAAAAAAAf//wAAAAAAAAAAAAAAAAAH//8AAAAAAAAAAAAAAAAAB///AAAAAAAAA' +
  'AAAAAAAAAf//wAAAAAAAAAAAAAAAAAH//8AAAAAAAAAAAAAAAAAB///AAAAAAAAAAAAAAAAAAf//wAAAAAAAAAAAAAAAAAH/' +
  '/8AAAAAAAAAAAAAAAAAB///AAAAAAAAAAAAAAAAAAf//wAAAAAAAAAAAAAAAAAH//8AAAAAAAAAAAAAAAAAB///AAAAAAAAA' +
  'AAAAAAAAAf//wAAAAAAAAAAAAAAAAAH//8AAAAAAAAAAAAAAAAAB///AAAAAAAAAAAAAAAAAAf//wAAAAAAAAAAAAAAAAAH/' +
  '/8AAAAAAAAAAAAAAAAAB///AAAAAAAAAAAAAAAAAAf//wAAAAAAAAAAAAAAAAAH//8AAAAAAAAAAAAAAAAAD///AAAAAAAAA' +
  'AAAAAAAAA///4AAAAAAAAAAAAAAAAA////AAAAAAAAAAAAAAAAAf///8AAAAAAAAAAAAAAAAf////wAAAAAAAAAAAAAAAf//' +
  '//+AAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAf//////gAAAAAAAAAAAAAf//////+AAAAAAAAAAAAAP///////4AAAAAA' +
  'AAAAAAf/j///8f/gAAAAAAAAAAAP/gP//+B/+AAAAAAAAAAAP/gD///AH/4AAAAAAAAAAP/wAf//wA//gAAAAAAAAAP/4AH/' +
  '/4AH/8AAAAAAAAAP/+AB//+AB//wAAAAAAAAH//gAf//gAf//AAAAAAAAH//4AH//4AH//8AAAAAAAH//+AB//+AB///wAAA' +
  'AAAH///gAf//gAf//+AAAAAAH///4AH//4AH///4AAAAAD///+AB//+AB////gAAAAH////gAf//wAf///+AAAAD////4AH/' +
  '/8AH////4AAAD////+AD///AB/////gAAD/////wB///8A/////8AAB/////+D////wf/////wAA//////////////////8A' +
  'AP//////////////////AAD//////////////////wAA//////////////////8AAP//////B///4H//////AAD/////gAP/' +
  '/8AB/////wAA////8AAB//+AAA////8AAP///AAAAf//gAAAP///AAD//4AAAAH//4AAAAH//wAA/+AAAAAB//+AAAAAB/8A' +
  'APAAAAAAAP//gAAAAAAPAAAAAAAAAAD//4AAAAAAAAAAAAAAAAAA//8AAAAAAAAAAAAAAAAAAP//AAAAAAAAAAAAAAAAAAD/' +
  '/wAAAAAAAAAAAAAAAAAA//8AAAAAAAAAAAAAAAAAAP//AAAAAAAAAAAAAAAAAAD//wAAAAAAAAAAAAAAAAAA//8AAAAAAAAA' +
  'AAAAAAAAAP//AAAAAAAAAAAAAAAAAAD//wAAAAAAAAAAAAAAAAAA//8AAAAAAAAAAAAAAAAAAH//AAAAAAAAAAAAAAAAAAB/' +
  '/wAAAAAAAAAAAAAAAAAAf/8AAAAAAAAAAAAAAAAAAP//AAAAAAAAAAAAAAAAAAD//wAAAAAAAAAAAAAAAAAB//+AAAAAAAAA' +
  'AAAAAAAAA///wAAAAAAAAAAAAAAAAAf//+AAAAAAAAAAAAAAAAAf///4AAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAP//' +
  '//8AAAAAAAAAAAAAAAH/////gAAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAD//////wAAAAAAAAAAAAAB///////AAAAAAA' +
  'AAAAAAB///////4AAAAAAAAAAAAA////////gAAAAAAAAAAAAf///////4AAAAAAAAAAAAH////////AAAAAAAAAAAAB////' +
  '////wAAAAAAAAAAAAf///////8AAAAAAAAAAAAH////////AAAAAAAAAAAAB//x//j//wAAAAAAAAAAAAf/gH/wH/8AAAAAA' +
  'AAAAAAH+AA/4AH/AAAAAAAAAAAAB8AAP8AAPgAAAAAAAAAAAAAAAD/AAAAAAAAAAAAAAAAAAAAfgAAAAAAAAAAAAAAAAAAAD' +
  'wAAAAAAAAAAAAAAAAAAAAYAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── sailboat — 비트맵 실루엣(128×128, 잉크 35.2%)
//   원본 마스크: sailboat.png
var MASK_SAILBOAT = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAA/AAAAAAAAAAAAAAAAAAAAf4AAAAAAAAAAAAAAAAAAAP/AAAAAAAAAAAAAAAAAAAH/wAAAAAAAAA' +
  'AAAAAAAAAB/+AAAAAAAAAAAAAAAAAAAf/gAAAAAAAAAAAAAAAAAAH/4AAAAAAAAAAAAAAAAAAB/+AAAAAAAAAAAAAAAAAAAf' +
  '/gAAAAAAAAAAAAAAAAAAH/4AAAAAAAAAAAAAAAAAAD/+AAAAAAAAAAAAAAAAAAA//gAAAAAAAAAAAAAAAAAAf/8AAAAAAAAA' +
  'AAAAAAAAAP//gAAAAAAAAAAAAAAAAAH//8AAAAAAAAAAAAAAAAAH///gAAAAAAAAAAAAAAAAD///8AAAAAAAAAAAAAAAAB//' +
  '//wAAAAAAAAAAAAAAAA////+AAAAAAAAAAAAAAAA/////4AAAAAAAAAAAAAAAf/////AAAAAAAAAAAAAAAP/////4AAAAAAA' +
  'AAAAAAAH//////AAAAAAAAAAAAAAD//////4AAAAAAAAAAAAAB///////AAAAAAAAAAAAAA///////4AAAAAAAAAAAAAf///' +
  '////AAAAAAAAAAAAAP///////4AAAAAAAAAAAAH////////AAAAAAAAAAAAD////////4AAAAAAAAAAAA//h//////AAAAAA' +
  'AAAAAAf/gH/////4AAAAAAAAAAAP/wA/////+AAAAAAAAAAAH/4AH/////wAAAAAAAAAAB/8AB/////+AAAAAAAAAAA/+AAP' +
  '/////wAAAAAAAAAAf/gAD/////+AAAAAAAAAAP/wAA///w//gAAAAAAAAAD/4AAP//wD/8AAAAAAAAAB/8AAD//4Af/gAAAA' +
  'AAAAA//AAA//8AD/4AAAAAAAAAP/gAAP//AAf/AAAAAAAAAH/wAAD//wAH/wAAAAAAAAD/4AAA//4AA/+AAAAAAAAA/+AAAP' +
  '/+AAH/wAAAAAAAAf/AAAD//gAB/8AAAAAAAAH/wAAA//4AAP/gAAAAAAAD/4AAAP/+AAB/4AAAAAAAA/8AAAD//gAAf/AAAA' +
  'AAAAf/AAAA//4AAD/wAAAAAAAH/gAAAP/+AAA/8AAAAAAAD/4AAAD//gAAH/gAAAAAAA/8AAAA//4AAB/4AAAAAAAP/AAAAP' +
  '/+AAAP/AAAAAAAH/gAAAD//gAAD/wAAAAAAB/4AAAA//4AAA/8AAAAAAA/8AAAAP/+AAAH/gAAAAAAP/AAAAD//gAAB/4AAA' +
  'AAAD/gAAAA//4AAAP+AAAAAAB/4AAAAP/+AAAD/gAAAAAAf+AAAAD//gAAA/8AAAAAAH/AAAAA//4AAAH/AAAAAAD/wAAAAP' +
  '/+AAAB/wAAAAAA/8AAAAD//gAAAf8AAAAAAP+AAAAA//4AAAH/AAAAAAD/gAAAAP/+AAAB/4AAAAAB/4AAAAD//gAAAf+AAA' +
  'AAAf+AAAAA//4AAAD/gAAAAAH/gAAAAP/+AAAA/4AAAAAB/4AAAAH//wAAAP+AAAAAAf+AAAAB//8AAAH/gAAAAAH/gAAAA/' +
  '//AAAB/4AAAAAB/8AAAAf//4AAAf+AAAAAA//gAAA////AAAH/gAAAAAP/+A//////+AAD/4AAAAAD///////////AB//AAA' +
  'AAA////////////3//gAAAAAP//////////////4AAAAAD//////////////+AAAAAAf//////////////gAAAAAH///////' +
  '///////4AAAAAB//////////////+AAAAAAf//////////////gAAAAAH//////////////4AAAAAB//////////////+AAA' +
  'AAAf//////////////gAAAAAD//////////////wAAAAAA//////////////8AAAAAAP//////////////AAAAAAD///////' +
  '///////wAAAAAAf/////////////4AAAAAAH/////////////+AAAAAAB//////////////gAAAAAAP/////////////4AAA' +
  'AAAD/////////////8AAAAAAA//////////////AAAAAAAH/////////////gAAAAAAB/////////////4AAAAAAAP//////' +
  '//////+AAAAAAAD/////////////AAAAAAAAf////////////gAAAAAAAD////////////4AAAAAAAA////////////8AAAA' +
  'AAAAH///////////+AAAAAAAAA////////////gAAAAAAAAP///////////wAAAAAAAAB///////////4AAAAAAAAAP/////' +
  '/////8AAAAAAAAAB//////////+AAAAAAAAAAP//////////AAAAAAAAAAB//////////gAAAAAAAAAAH/////////wAAAAA' +
  'AAAAAA/////////wAAAAAAAAAAAD////////wAAAAAAAAAAAAH///////wAAAAAAAAAAAAAP//////gAAAAAAAAAAAAAAf//' +
  '//+AAAAAAAAAAAAAAAAD///gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── rocket — 비트맵 실루엣(128×128, 잉크 25.6%)
//   원본 마스크: rocket.png
var MASK_ROCKET = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAA8AAAAAAAAAAAAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAf+AAAAAAAAAAAAAAAAAAAP/wAAAAAAAAA' +
  'AAAAAAAAAH/+AAAAAAAAAAAAAAAAAAD//wAAAAAAAAAAAAAAAAAB//+AAAAAAAAAAAAAAAAAA///wAAAAAAAAAAAAAAAAAf/' +
  '/+AAAAAAAAAAAAAAAAAH///gAAAAAAAAAAAAAAAAD///8AAAAAAAAAAAAAAAAB////gAAAAAAAAAAAAAAAAf///4AAAAAAAA' +
  'AAAAAAAAP////AAAAAAAAAAAAAAAAD////wAAAAAAAAAAAAAAAB////8AAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAB8AAAHwAAAAAAAAAAAAAAAf////+AAAAAAAAAAAAAAAH/////gAAAAAAA' +
  'AAAAAAAD/////8AAAAAAAAAAAAAAA//////AAAAAAAAAAAAAAAf/////wAAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB///' +
  '///gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAD//wD//wAAAAAAAAAAAAAA//wAP/8AAAAAAA' +
  'AAAAAAAf/wAA//gAAAAAAAAAAAAAH/4AAH/4AAAAAAAAAAAAAB/8AAA/+AAAAAAAAAAAAAAf+AAAH/gAAAAAAAAAAAAAP/gA' +
  'AB/4AAAAAAAAAAAAAD/wAAAP/AAAAAAAAAAAAAA/8AAAD/wAAAAAAAAAAAAAP+AAAAf8AAAAAAAAAAAAAD/gAAAH/AAAAAAA' +
  'AAAAAAA/4AAAB/4AAAAAAAAAAAAAf+AAAAf+AAAAAAAAAAAAAH/gAAAH/gAAAAAAAAAAAAB/4AAAB/4AAAAAAAAAAAAAf+AA' +
  'AAf+AAAAAAAAAAAAAH/gAAAH/gAAAAAAAAAAAAB/8AAAD/4AAAAAAAAAAAAAf/AAAA/+AAAAAAAAAAAAAP/wAAAf/gAAAAAA' +
  'AAAAAAD/+AAAH/4AAAAAAAAAAAAA//wAAD/+AAAAAAAAAAAAAP/+AAB//gAAAAAAAAAAAAD//wAA//8AAAAAAAAAAAAA//+A' +
  'A///AAAAAAAAAAAAAP//4Af//wAAAAAAAAAAAAD///////8AAAAAAAAAAAAA///////+AAAAAAAAAAAAAP///////gAAAAAA' +
  'AAAAAAD///////8AAAAAAAAAAAAD////////wAAAAAAAAAAAD////////+AAAAAAAAAAAB/////////4AAAAAAAAAAA/////' +
  '/////AAAAAAAAAAA//////////4AAAAAAAAAAP9///////7/AAAAAAAAAAP+P//////8f4AAAAAAAAAH/D///////D/AAAAA' +
  'AAAAB/g///////wf4AAAAAAAAA/wP//////8H/AAAAAAAAAf8D//////+A/4AAAAAAAAP+Af//////gP+AAAAAAAAD/gH///' +
  '///4B/wAAAAAAAB/4B//////8Af+AAAAAAAA/8AP//////AD/gAAAAAAAP/AD//////wA/8AAAAAAAD/gAf/////4AP/AAAA' +
  'AAAB/4AH/////+AB/4AAAAAAAf+AB//////AAf+AAAAAAAP/gAP/////wAH/wAAAAAAD/4AD/////4AB/8AAAAAAA/+AA///' +
  '///AAf/AAAAAAAf/AAf/////4AH/4AAAAAAH/wAf//////gB/+AAAAAAB/8Af//////8Af/gAAAAAAf/AP///////wH/4AAA' +
  'AAAP/wH/h///4f+B/+AAAAAAD/+D/g////B/wf/gAAAAAA//h/gP///wH+H/8AAAAAAP/4/gD///+Afx//AAAAAAD/+fwB//' +
  '//gD+f/wAAAAAA//v4Af///4Afn/4AAAAAAP//4AH///+AD//+AAAAAAB//8AB////gAf//gAAAAAAf/+AAf///4AB//4AAA' +
  'AAAH//AAH///+AAP/+AAAAAAB//gAB////gAB//gAAAAAAP/wAAf///4AAP/wAAAAAAD/4AAH///+AAB/8AAAAAAAf4AAA//' +
  '//AAAH+AAAAAAAD8AAAP///wAAA/AAAAAAAAAAAAD///8AAAAAAAAAAAAAAAAA///+AAAAAAAAAAAAAAAAAH///gAAAAAAAA' +
  'AAAAAAAAB///4AAAAAAAAAAAAAAAAAf//8AAAAAAAAAAAAAAAAAD///AAAAAAAAAAAAAAAAAA///gAAAAAAAAAAAAAAAAAH/' +
  '/4AAAAAAAAAAAAAAAAAB//8AAAAAAAAAAAAAAAAAAP//AAAAAAAAAAAAAAAAAAD//gAAAAAAAAAAAAAAAAAAf/4AAAAAAAAA' +
  'AAAAAAAAAH/8AAAAAAAAAAAAAAAAAAA/+AAAAAAAAAAAAAAAAAAAH/gAAAAAAAAAAAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAH' +
  '4AAAAAAAAAAAAAAAAAAAA8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── crescent — 비트맵 실루엣(128×128, 잉크 33.3%)
//   원본 마스크: crescent.png
var MASK_CRESCENT = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAA//8AAAAAAAAAAAAAAAAAP////AAAAAAAAAAAAAAAA//////AAAAAAAAAAAAAAA//////8AAAAAA' +
  'AAAAAAAB///////4AAAAAAAAAAAAB////////gAAAAAAAAAAAB////////+AAAAAAAAAAAB/////////4AAAAAAAAAAB////' +
  '//////gAAAAAAAAAA//////////8AAAAAAAAAA///////////gAAAAAAAAAf//////////+AAAAAAAAAP///////////wAAA' +
  'AAAAAP///////////+AAAAAAAAH////////4AAAgAAAAAAAD////////gAAAAAAAAAAAB////////gAAAAAAAAAAAA//////' +
  '//AAAAAAAAAAAAAf///////AAAAAAAAAAAAAP///////AAAAAAAAAAAAAH///////AAAAAAAAAAAAAB///////gAAAAAAAAA' +
  'AAAA///////gAAAAAAAAAAAAAf//////wAAAAAAAAAAAAAP//////4AAAAAAAAAAAAAH//////8AAAAAAAAAAAAAB//////8' +
  'AAAAAAAAAAAAAA//////+AAAAAAAAAAAAAAP//////AAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAD//////4AAAAAAAAAAA' +
  'AAA//////8AAAAAAAAAAAAAAf/////+AAAAAAAAAAAAAAH//////gAAAAAAAAAAAAAD//////wAAAAAAAAAAAAAA//////4A' +
  'AAAAAAAAAAAAAP/////+AAAAAAAAAAAAAAH//////AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAA//////4AAAAAAAAAAAA' +
  'AAP/////8AAAAAAAAAAAAAAD//////AAAAAAAAAAAAAAA//////wAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AA' +
  'AAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////wAAAAAAAAAAAAAAP/////8AAAAAAAAAAAAAAD//////AAAAAAAAAAAAA' +
  'AA//////wAAAAAAAAAAAAAAP/////4AAAAAAAAAAAAAAD/////+AAAAAAAAAAAAAAA//////gAAAAAAAAAAAAAAf/////4AA' +
  'AAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAA' +
  'AB//////gAAAAAAAAAAAAAAf/////4AAAAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////4AA' +
  'AAAAAAAAAAAAH/////+AAAAAAAAAAAAAAB//////gAAAAAAAAAAAAAAf/////8AAAAAAAAAAAAAAH//////AAAAAAAAAAAAA' +
  'AA//////wAAAAAAAAAAAAAAP/////8AAAAAAAAAAAAAAD//////AAAAAAAAAAAAAAA//////4AAAAAAAAAAAAAAP/////+AA' +
  'AAAAAAAAAAAAB//////wAAAAAAAAAAAAAAf/////8AAAAAAAAAAAAAAH//////AAAAAAAAAAAAAAB//////4AAAAAAAAAAAA' +
  'AAf/////+AAAAAAAAAAAAAAD//////wAAAAAAAAAAAAAA//////+AAAAAAAAAAAAAAP//////gAAAAAAAAAAAAAB//////8A' +
  'AAAAAAAAAAAAAf//////AAAAAAAAAAAAAAH//////4AAAAAAAAAAAAAA///////AAAAAAAAAAAAAAP//////4AAAAAAAAAAA' +
  'AAB///////AAAAAAAAAAAAAAf//////4AAAAAAAAAAAAAD///////AAAAAAAAAAAAAA///////4AAAAAAAAAAAAAH///////' +
  'AAAAAAAAAAAAAA///////4AAAAAAAAgAAAAP///////AAAAAAAAwAAAAB///////8AAAAAAA8AAAAAf///////gAAAAAAeAA' +
  'AAAD///////+AAAAAAfAAAAAAf///////4AAAAAfwAAAAAH////////gAAAA/4AAAAAA/////////gAAB/8AAAAAAH//////' +
  '///AAH/+AAAAAAA//////////////AAAAAAAH/////////////gAAAAAAA/////////////wAAAAAAAH////////////4AAA' +
  'AAAAA////////////8AAAAAAAAH///////////+AAAAAAAAA////////////AAAAAAAAAH///////////AAAAAAAAAAf////' +
  '//////gAAAAAAAAAD//////////wAAAAAAAAAAP/////////wAAAAAAAAAAB/////////wAAAAAAAAAAAH////////4AAAAA' +
  'AAAAAAAf///////4AAAAAAAAAAAAB///////wAAAAAAAAAAAAAH//////wAAAAAAAAAAAAAAP/////gAAAAAAAAAAAAAAAP/' +
  '///AAAAAAAAAAAAAAAAAf//4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

// ── snowflake — 비트맵 실루엣(128×128, 잉크 29.5%)
//   원본 마스크: snowflake.png
var MASK_SNOWFLAKE = decodeMask(128,
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAAD/AAAAAAAAAA' +
  'AAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAcD/A4AAAAAAAAAAAAAAAAPg/wfAAAAAAAAAAAAAAAAH8P' +
  '8P4AAAAAAAAAAAAAAAD/n/n/AAAAAAAAAAAAAAAA/////wAAAAAAAAAAAAAAAP////8AAAAAAAAAAAAAAAB////+AAAAAAAA' +
  'AAAAAAAAP////AAAAAAAAAAAAAAAAB////gAAAAAAAAAAAAA/gAP///gAH8AAAAAAAAAAf4AB///wAB/gAAAAAAAAcH+AAH/' +
  '/4AAf4OAAAAAAAPh/gAA//8AAH+HwAAAAAAH8f4AAH/+AAB/j+AAAAAAD/v+AAB//gAAf9/wAAAAAA///gAAf/4AAH//8AAA' +
  'AAAP//4AAHgeAAB///AAAAAAD//+AABwDgAAf//wAAAAAAf//gAAcA4AAH//4AAAAAAD//4AAGAGAAB//8AAAAAAAf/+AABg' +
  'BgAAf/+AAAAAAAD//gAAYAYAAH//AAAAAAAB//8AAHAOAAD//4AAAAAAP///gABwDgAB///8AAAAAH///8AAeB4AA////gAA' +
  'AAB////gAH/+AAf///4AAAAAf//4cAB//gAMH//+AAAAAH//8BgAf/4AGA///gAAAAB//+AMAH/+ADAH//4AAAAAf//gDgB/' +
  '/gBwB//+AAAAAAAD4A8Af/4A8AfAAAAAAAAAAeAPgH/+AfAHgAAAAAAAAADgD8D//wPwBwAAAAAAAAAAYB/j///H+A4AAAAA' +
  'AAAAADA///w///wMAAAAAAAAAAAf///4H///+AAAAAAAAAAAD///+B////AAAAAAAAAAAAf///gf///gAAAAAAAAAAAD/+f4' +
  'H8f/gAAAAAAABgAAAf/B+B+D/wAAAGAAAA8AAAD/gP//Af4AAADwAAAfgAAAf4D//wH8AAAB+AAAP8AAAD+A//8B+AAAA/wA' +
  'AH/gAAAfgfgfgfgAAAf+AAA/8AAAH8HgB4P4AAAP/AAAH/gAAD//gAH//AAAH/wAAA/8AAA//wAA//wAAD/4AAAP/gAAf/4A' +
  'AH/+AAB/8AAAB//////8AAA//////+AAAAf//////AAAP//////gAAP//+B//PgAAB8//gf//8AH///AP/A4AAAcD/wD///g' +
  'B///wB/gOAAAHAf4A///4Af//8Af4BgAABgH+AP//+AH///AH+AYAAAYB/gB///gB///wB/gOAAAGAf4A///4Af//8A/8DgA' +
  'ABwP/AP//+AD///gP/h4AAAeH/wH///AAAf/8P///AAAP///D//gAAAH//////wAAD//////4AAAB/5AHP/+AAB//gAB/+AA' +
  'AA/8AAA//gAAf/wAAD/wAAA/+AAAP/8AAP/8AAAf+AAAf/AAAD+DwAPB+AAAD/wAAH/gAAAfgeAHgfgAAAf+AAA/wAAAHwH+' +
  'f4D4AAAD/AAAH4AAAD8B//+A/AAAAfgAAA8AAAB/Af5/gP4AAADwAAAGAAAA/4H4H4H/AAAAYAAAAAAAAf/D+B/D/4AAAAAA' +
  'AAAAAAP///AP///AAAAAAAAAAAAH///wD///4AAAAAAAAAAAD///8A////AAAAAAAAAAABg///gf//gYAAAAAAAAAAAwH+f8' +
  'P+fwDAAAAAAAAAAAYA/B//+D8AYAAAAAAAAAAOAPgH/+AfAHAAAAAAAAAAHgDwB//gDwB4AAAAAAAD//4A4Af/4AcAf//AAA' +
  'AAB//+AMAH/+ADAP//4AAAAAf//wGAB//gAYD//+AAAAAH//+DAAf/4ADD///gAAAAB////gAHgeAAf///4AAAAAf///wABw' +
  'DgAD///+AAAAAD///4AAcA4AAf///AAAAAAB//8AAGAGAAD//4AAAAAAAf//AABgBgAA//8AAAAAAAH//wAAYAYAAP//gAAA' +
  'AAAD//8AAHAOAAD//8AAAAAAB///AABwDgAA///gAAAAAA///wAAeD4AAP//8AAAAAAP//8AAH/+AAD///AAAAAAD///AAB/' +
  '/gAA///wAAAAAAfx/wAAf/4AAP+f4AAAAAAD4f8AAP//AAD/D8AAAAAAAcH/AAP//8AA/wOAAAAAAACA/wAH///gAP8BAAAA' +
  'AAAAAP4AD///8AB/AAAAAAAAAAB8AB////gAAAAAAAAAAAAAAAA////8AAAAAAAAAAAAAAAA/////gAAAAAAAAAAAAAAAP//' +
  '//8AAAAAAAAAAAAAAAD/3/v/AAAAAAAAAAAAAAAAfw/w/wAAAAAAAAAAAAAAAD4P8H4AAAAAAAAAAAAAAAAcD/A4AAAAAAAA' +
  'AAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAP8AAAAAAAAAAAAAAAAAAAD/AAAAAAAAAAAAAAAAAAAA/wAAAAAAAAAAAAAAAAAAAP' +
  '8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA' +
  'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=');

function insideShape(mx, my, L) {
  if (!insideShapeBase(mx, my, L)) return false;
  /* ★noGrooves = **랩 비교 전용**이다. 이걸로 만든 코드는 데이터 셀 집합이 지금 디코더와
   *   달라 **읽히지 않는다**(실측: 골 5종 전부 "해독 실패"). 발행 경로에 절대 쓰지 말 것.
   *   반대로 **골 도입 전에 이미 발행된 코드**는 디코더의 변형 3 이 이 경로로 읽어 준다. */
  var gl = L.noGrooves ? null : SHAPE_GROOVES[L.shape];
  if (!gl) return true;
  var c = L.coreCenter.mx, R = L.Rdata || (L.N / 2 - 0.8);
  return !inGroove(gl, (mx - c) / R, (my - c) / R);
}
function insideShapeBase(mx, my, L) {
  const c = L.coreCenter.mx, R = L.Rdata || (L.N / 2 - 0.8);
  const nx = (mx - c) / R, ny = (my - c) / R;
  // ── 'custom' — 사용자 이미지에서 뽑은 임의 마스크 (2026-08-26, "내 모양 만들기") ──
  //   L.customMask = { n, bits } : n×n 비트맵을 정규화 좌표 [-1,1]²에 매핑한 것.
  //   ★★디코더 호환성 경고: 이 마스크는 스캐너가 알 방법이 없다 — `codec.readCode`가
  //   `dataCells(layout)`로 셀 집합·순서를 재구성하는데, 그 순서가 마스크에 의존하기
  //   때문이다(§B 실측). 따라서 'custom'은 **마스크를 아는 쪽**(생성기 자신의 셀프테스트,
  //   또는 마스크를 함께 전달받은 디코더)에서만 해독된다. 일반 스캐너용 코드를 만들 때
  //   쓰면 안 된다 — 호출부가 반드시 이 제약을 처리할 것.
  if (L.shape === 'custom') {
    const M = L.customMask;
    if (!M || !M.n || !M.bits) return true;          // 마스크 없으면 사각(=제약 없음)
    const ix = Math.floor((nx + 1) * 0.5 * M.n);
    const iy = Math.floor((ny + 1) * 0.5 * M.n);
    if (ix < 0 || iy < 0 || ix >= M.n || iy >= M.n) return false;
    return !!M.bits[iy * M.n + ix];
  }
  if (L.shape === 'piano') return inMask(MASK_PIANO, nx, ny);
  if (L.shape === 'butterfly') return inMask(MASK_BUTTERFLY, nx, ny);
  if (L.shape === 'earth') return inMask(MASK_EARTH, nx, ny);
  if (L.shape === 'eye') return inMask(MASK_EYE, nx, ny);
  if (L.shape === 'nose') return inMask(MASK_NOSE, nx, ny);
  if (L.shape === 'ear') return inMask(MASK_EAR, nx, ny);
  if (L.shape === 'hand') return inMask(MASK_HAND, nx, ny);
  if (L.shape === 'foot') return inMask(MASK_FOOT, nx, ny);
  if (L.shape === 'brain') return inMask(MASK_BRAIN, nx, ny);
  if (L.shape === 'dove') return inMask(MASK_DOVE, nx, ny);
  if (L.shape === 'love') return inMask(MASK_LOVE, nx, ny);
  if (L.shape === 'hope') return inMask(MASK_HOPE, nx, ny);
  if (L.shape === 'connection') return inMask(MASK_CONNECTION, nx, ny);
  if (L.shape === 'family') return inMask(MASK_FAMILY, nx, ny);
  if (L.shape === 'smile') return inMask(MASK_SMILE, nx, ny);
  if (L.shape === 'sadness') return inMask(MASK_SADNESS, nx, ny);
  if (L.shape === 'friendship') return inMask(MASK_FRIENDSHIP, nx, ny);
  if (L.shape === 'embrace') return inMask(MASK_EMBRACE, nx, ny);
  if (L.shape === 'boomer') return inMask(MASK_BOOMER, nx, ny);
  if (L.shape === 'grandpiano') return inMask(MASK_GRANDPIANO, nx, ny);
  if (L.shape === 'heartorgan') return inMask(MASK_HEARTORGAN, nx, ny);
  if (L.shape === 'chat') return inMask(MASK_CHAT, nx, ny);
  if (L.shape === 'book') return inMask(MASK_BOOK, nx, ny);
  if (L.shape === 'kiosk') return inMask(MASK_KIOSK, nx, ny);
  if (L.shape === 'gimbap') return inMask(MASK_GIMBAP, nx, ny);
  if (L.shape === 'elevator') return inMask(MASK_ELEVATOR, nx, ny);
  if (L.shape === 'manse') return inMask(MASK_MANSE, nx, ny);
  if (L.shape === 'lonely') return inMask(MASK_LONELY, nx, ny);
  if (L.shape === 'together') return inMask(MASK_TOGETHER, nx, ny);
  if (L.shape === 'comfort') return inMask(MASK_COMFORT, nx, ny);
  if (L.shape === 'dolhareubang') return inMask(MASK_DOLHAREUBANG, nx, ny);
  if (L.shape === 'tree') return inMask(MASK_TREE, nx, ny);
  if (L.shape === 'cup') return inMask(MASK_CUP, nx, ny);
  if (L.shape === 'car') return inMask(MASK_CAR, nx, ny);
  if (L.shape === 'dog') return inMask(MASK_DOG, nx, ny);
  if (L.shape === 'cat') return inMask(MASK_CAT, nx, ny);
  if (L.shape === 'fish') return inMask(MASK_FISH, nx, ny);
  if (L.shape === 'bear') return inMask(MASK_BEAR, nx, ny);
  if (L.shape === 'rabbit') return inMask(MASK_RABBIT, nx, ny);
  if (L.shape === 'turtle') return inMask(MASK_TURTLE, nx, ny);
  if (L.shape === 'whale') return inMask(MASK_WHALE, nx, ny);
  if (L.shape === 'horse') return inMask(MASK_HORSE, nx, ny);
  if (L.shape === 'bird') return inMask(MASK_BIRD, nx, ny);
  if (L.shape === 'deer') return inMask(MASK_DEER, nx, ny);
  if (L.shape === 'hearingaid') return inMask(MASK_HEARINGAID, nx, ny);
  if (L.shape === 'tooth') return inMask(MASK_TOOTH, nx, ny);
  if (L.shape === 'mouth') return inMask(MASK_MOUTH, nx, ny);
  if (L.shape === 'lungs') return inMask(MASK_LUNGS, nx, ny);
  if (L.shape === 'bone') return inMask(MASK_BONE, nx, ny);
  if (L.shape === 'arena') return inMask(MASK_ARENA, nx, ny);
  if (L.shape === 'pestbug') return inMask(MASK_PESTBUG, nx, ny);
  if (L.shape === 'finger') return inMask(MASK_FINGER, nx, ny);
  if (L.shape === 'like') return inMask(MASK_LIKE, nx, ny);
  if (L.shape === 'victory') return inMask(MASK_VICTORY, nx, ny);
  if (L.shape === 'oksign') return inMask(MASK_OKSIGN, nx, ny);
  if (L.shape === 'lovesign') return inMask(MASK_LOVESIGN, nx, ny);
  if (L.shape === 'prayer') return inMask(MASK_PRAYER, nx, ny);
  if (L.shape === 'fist') return inMask(MASK_FIST, nx, ny);
  if (L.shape === 'badge') return inMask(MASK_BADGE, nx, ny);
  if (L.shape === 'waterdrop') return inMask(MASK_WATERDROP, nx, ny);
  if (L.shape === 'crown') return inMask(MASK_CROWN, nx, ny);
  if (L.shape === 'bag') return inMask(MASK_BAG, nx, ny);
  if (L.shape === 'watch') return inMask(MASK_WATCH, nx, ny);
  if (L.shape === 'ring') return inMask(MASK_RING, nx, ny);
  if (L.shape === 'perfume') return inMask(MASK_PERFUME, nx, ny);
  if (L.shape === 'fan') return inMask(MASK_FAN, nx, ny);
  if (L.shape === 'helmet') return inMask(MASK_HELMET, nx, ny);
  if (L.shape === 'goggles') return inMask(MASK_GOGGLES, nx, ny);
  if (L.shape === 'harness') return inMask(MASK_HARNESS, nx, ny);
  if (L.shape === 'extinguisher') return inMask(MASK_EXTINGUISHER, nx, ny);
  if (L.shape === 'safetycone') return inMask(MASK_SAFETYCONE, nx, ny);
  if (L.shape === 'gloves') return inMask(MASK_GLOVES, nx, ny);
  if (L.shape === 'dustmask') return inMask(MASK_DUSTMASK, nx, ny);
  if (L.shape === 'haetae') return inMask(MASK_HAETAE, nx, ny);
  if (L.shape === 'safetymanager') return inMask(MASK_SAFETYMANAGER, nx, ny);
  if (L.shape === 'healthmanager') return inMask(MASK_HEALTHMANAGER, nx, ny);
  if (L.shape === 'safetyboot') return inMask(MASK_SAFETYBOOT, nx, ny);
  if (L.shape === 'safetyvest') return inMask(MASK_SAFETYVEST, nx, ny);
  if (L.shape === 'elephant') return inMask(MASK_ELEPHANT, nx, ny);
  if (L.shape === 'owl') return inMask(MASK_OWL, nx, ny);
  if (L.shape === 'crab') return inMask(MASK_CRAB, nx, ny);
  if (L.shape === 'octopus') return inMask(MASK_OCTOPUS, nx, ny);
  if (L.shape === 'snail') return inMask(MASK_SNAIL, nx, ny);
  if (L.shape === 'bee') return inMask(MASK_BEE, nx, ny);
  if (L.shape === 'frog') return inMask(MASK_FROG, nx, ny);
  if (L.shape === 'swan') return inMask(MASK_SWAN, nx, ny);
  if (L.shape === 'umbrella') return inMask(MASK_UMBRELLA, nx, ny);
  if (L.shape === 'key') return inMask(MASK_KEY, nx, ny);
  if (L.shape === 'lightbulb') return inMask(MASK_LIGHTBULB, nx, ny);
  if (L.shape === 'scissors') return inMask(MASK_SCISSORS, nx, ny);
  if (L.shape === 'hammer') return inMask(MASK_HAMMER, nx, ny);
  if (L.shape === 'glasses') return inMask(MASK_GLASSES, nx, ny);
  if (L.shape === 'envelope') return inMask(MASK_ENVELOPE, nx, ny);
  if (L.shape === 'gift') return inMask(MASK_GIFT, nx, ny);
  if (L.shape === 'apple') return inMask(MASK_APPLE, nx, ny);
  if (L.shape === 'icecream') return inMask(MASK_ICECREAM, nx, ny);
  if (L.shape === 'cupcake') return inMask(MASK_CUPCAKE, nx, ny);
  if (L.shape === 'donut') return inMask(MASK_DONUT, nx, ny);
  if (L.shape === 'airplane') return inMask(MASK_AIRPLANE, nx, ny);
  if (L.shape === 'sailboat') return inMask(MASK_SAILBOAT, nx, ny);
  if (L.shape === 'rocket') return inMask(MASK_ROCKET, nx, ny);
  if (L.shape === 'crescent') return inMask(MASK_CRESCENT, nx, ny);
  if (L.shape === 'snowflake') return inMask(MASK_SNOWFLAKE, nx, ny);
  if (L.shape === 'round') return nx * nx + ny * ny <= 1;
  if (L.shape === 'heart') return pointInHeart(nx, ny);
  if (L.shape === 'bubble') return pointInBubble(nx, ny);
  if (L.shape === 'chat') return pointInChat(nx, ny);
  if (L.shape === 'lid') return pointInLid(nx, ny);
  if (L.shape === 'rose') return pointInRose(nx, ny);
  if (L.shape === 'fish') return pointInFish(nx, ny);
  if (L.shape === 'piano') return pointInPianoKeys(nx, ny);
  // ★2026-08-27 §7-36 유령 로스터 사건 — 3겹 방어 ②.
  //   AI 팩 5종(pixel-crab·spark·knot6·bot·twinkle)이 **기하 함수 없이** 여기와 로스터에
  //   등재돼 있었다. 정의 없는 식별자를 호출해 ReferenceError 가 나면 "못 읽음"이 아니라
  //   **해독 자체가 중단**된다 — 스캐너 프레임 루프가 죽고 진단도 가려진다.
  //   그 5종은 뿌리에서 제거했고(방어 ①), 여기서는 **미래에 같은 실수가 나도** 크래시가
  //   아니라 깨끗한 false 로 떨어지게 한다. typeof 는 선언되지 않은 식별자에도 안전하다.
  //   (인체 팩 4종은 기하가 실존해 그대로 유지 — 파일럿이라 생성기에는 미노출.)
  if (SHAPE_POINT_FN.hasOwnProperty(L.shape)) {
    var _sfn = SHAPE_POINT_FN[L.shape];
    return _sfn ? _sfn(nx, ny) : false;
  }
  // clover/star/hex: 4개 위성이 항상 상/우/하/좌(0°,90°,180°,270°) 고정이므로, 이 세 모양의
  // 4중 대칭축도 같은 각도에 맞춰(|cos(2θ)|류) 위성이 잎/꼭짓점 자리에 정확히 올라가게 한다.
  if (L.shape === 'clover') {
    // 잎 4개 = 카디널 방향으로 띄운 원 4개의 합집합(둥근 잎 모양) + 중심 허브(연결 보장).
    // lobeR=0.50(잎이 서로 붙어 하나의 사각 꽃봉오리처럼 보임, 실사용자 지적)에서 0.33으로 줄여
    // 잎 사이에 뚜렷한 잘록함을 만듦 — 허브가 여전히 4개 잎을 중앙에서 이어준다.
    const lobeD = 0.55, lobeR = 0.33, hubR = 0.30;
    if ((nx * nx + ny * ny) <= hubR * hubR) return true;
    const centers = [[0, -lobeD], [lobeD, 0], [0, lobeD], [-lobeD, 0]];
    for (const [cx, cy] of centers) {
      const dx = nx - cx, dy = ny - cy;
      if (dx * dx + dy * dy <= lobeR * lobeR) return true;
    }
    return false;
  }
  if (L.shape === 'boomerang') {
    // 옛 'star'(4꼭짓점) — 진짜 5각별을 새로 넣으면서 이 모양은 이름 그대로 부메랑으로 개명.
    // 4개 꼭짓점(위성 자리) + 4개 안쪽 오목점을 잇는 실제 폴리곤(대칭은 유지, 이름만 정정).
    // ★2026-08-16(Fable 5 자문): Ri=0.24는 허리가 너무 좁아 다리QR 박스(코어 아래, 캔버스 폭의
    // 15%)가 실루엣 밖으로 삐져나왔다(실측: 264px QR박스 vs 134~238px 허리 폭). 다리QR을 줄이면
    // 실제 스캔이 안 되는 게 이미 확인된 사실이라 축소는 불가 — 대신 허리를 넓혀 QR을 완전히
    // 감싸도록 Ri를 키움. 독자 기하 재구현으로 grid L 기준 "박스가 완전히 안에 들어가는 최소값"을
    // 이진탐색: 0.39. Fable 5가 별도로 유도한 값(0.43)과 근접해 서로 검증됨 — 두 grid 크기(S/M)
    // 차이·라운딩 여유까지 감안해 0.44로 적용(5각별의 Ri=0.42와도 톤이 맞음). 이름은 "boomerang"
    // 그대로 유지(형이 "살짝 바뀌어도 된다"고 했지만 부메랑이라는 정체성 자체는 여전히 유효한
    // 폭넓은 4꼭짓점 모양이라 새 이름 없이 파라미터만 조정) — 실측 검증 통과 후 라이브 반영.
    const Ro = 1.0, Ri = 0.44;
    const th = Math.atan2(ny, nx);
    const sect = Math.PI / 2, half = sect / 2;
    let a = ((th + half) % sect + sect) % sect - half;
    const phi = Math.abs(a);
    const x1 = Ro, y1 = 0, x2 = Ri * Math.cos(half), y2 = Ri * Math.sin(half);
    const dx = x2 - x1, dy = y2 - y1;
    const rr = (x1 * dy - y1 * dx) / (Math.cos(phi) * dy - Math.sin(phi) * dx);
    return (nx * nx + ny * ny) <= rr * rr;
  }
  if (L.shape === 'star') {
    // 진짜 5각별(꼭짓점 5개, 오각 대칭) — 위성 4개는 4중 대칭이라 별의 5개 꼭짓점과는 애초에
    // 안 맞는다(5와 4는 공약수가 1). 그래도 안전한 이유: 위성은 오목점(반경 Ri=0.42) 근방에
    // 걸려도 이 반경이 위성 예약반경(Rs, 대략 0.14)보다 훨씬 커서 어느 회전에서도 위성이
    // 별 몸통 밖으로 삐져나오지 않는다(reservedAt이 앵커존을 모양과 무관하게 먼저 예약).
    // rot=90°: 꼭짓점 하나가 정확히 하단 위성과 정렬 — 육안으로 "위가 넓고 아래가 뾰족한" 표준
    // 별 실루엣이 나오도록 고른 값(실측 비교 후 결정, Ri 0.24/0.5도 시험했으나 0.42가 가장 또렷).
    const Ro = 1.0, Ri = 0.42, m = 5, sect = 2 * Math.PI / m, half = sect / 2, rot = Math.PI / 2;
    const th = Math.atan2(ny, nx) - rot;
    let a = ((th + half) % sect + sect) % sect - half;
    const phi = Math.abs(a);
    const x1 = Ro, y1 = 0, x2 = Ri * Math.cos(half), y2 = Ri * Math.sin(half);
    const dx = x2 - x1, dy = y2 - y1;
    const rr = (x1 * dy - y1 * dx) / (Math.cos(phi) * dy - Math.sin(phi) * dx);
    return (nx * nx + ny * ny) <= rr * rr;
  }
  if (L.shape === 'hex') {
    const th = Math.atan2(ny, nx), m = 6, ro = 0.98;
    const sect = 2 * Math.PI / m;
    const a = ((th % sect) + sect) % sect;
    const rr = ro * Math.cos(Math.PI / m) / Math.cos(a - Math.PI / m);
    return (nx * nx + ny * ny) <= rr * rr;
  }
  return true;
}

// ── 예약 판정 (앵커/궤도 영역엔 데이터 도트 금지) ─────────────────────────
// ── 반경 비교 가속 (2026-08-29) ─────────────────────────────────────────────
// inkAt/reservedAt 은 픽셀마다 6번 넘게 불린다(격자 L·cellPx4·ss2 = 3,200만 회).
// Math.hypot 은 오버플로 안전 알고리즘이라 곱셈 몇 번보다 훨씬 비싸다.
//
// ★그런데 hypot 을 제곱비교로 **그냥** 바꾸면 픽셀이 달라질 수 있다.
//   반경이 2.5(위성)처럼 딱 떨어지는 값이라 3-4-5 직각삼각형 같은 **정확한 경계점**이
//   실제로 생긴다 — 그 점에서 두 계산이 마지막 자리에서 갈리면 도트 하나가 바뀐다.
//   그래서 확실한 안/밖만 제곱으로 끝내고, 경계 극소수(상대오차 1e-7 띠)만 hypot 으로
//   되받는다. 결과는 **비트 동일**이고 hypot 호출은 사실상 0 이 된다.
function within(dx, dy, r) {
  const q = dx * dx + dy * dy, r2 = r * r;
  if (q < r2 * 0.9999999) return true;
  if (q > r2 * 1.0000001) return false;
  return Math.hypot(dx, dy) <= r;   // 경계 — 원래 계산 그대로
}

function reservedAt(mx, my, L) {
  const { coreCenter, anchors, orbit, spec } = L; const S = spec || SPEC;
  if (L.shape && L.shape !== 'square' && !insideShape(mx, my, L)) return true;   // 모양 밖 = 데이터 없음
  const cdx = mx - coreCenter.mx, cdy = my - coreCenter.my;
  const cq = cdx * cdx + cdy * cdy;
  const RO2 = S.orbitRadius + S.orbitDotR + 0.4, RC2 = S.coreOuter + 0.6;
  const ROUT2 = (RO2 > RC2 ? RO2 : RC2);
  if (cq <= ROUT2 * ROUT2 * 1.0000001) {
    const dCore = Math.hypot(cdx, cdy);
    if (dCore <= S.coreOuter + 0.6) return true;
    if (Math.abs(dCore - S.orbitRadius) <= S.orbitDotR + 0.4) return true;
  }
  const SR2 = S.satRadius + 0.6 + ((L.hug && S.satHalo) ? S.satHalo : 0);
  for (let i = 1; i < anchors.length; i++) {
    const dx = mx - anchors[i].mx; if (dx > SR2 || dx < -SR2) continue;
    const dy = my - anchors[i].my; if (dy > SR2 || dy < -SR2) continue;
    if (within(dx, dy, SR2)) return true;
  }
  if (L.bridgeRect) { const r = L.bridgeRect.reserve;
    if (mx >= r.mx0 && mx < r.mx1 && my >= r.my0 && my < r.my1) return true; }
  return false;
}

// ── 데이터 셀 순서 열거 (예약 밖 셀, 래스터 순서) — 인코드/디코드 공용 계약 ──
//   render(opts.bits) 배치 순서와 반드시 동일해야 한다(디코더가 같은 순서로 읽음).
/* ★C10-b(2026-09-04) — dataCells 예약마스크 메모이즈.
 *   dataCells 는 순수 함수다 — 같은 레이아웃이면 같은 셀 목록이 나온다. 그런데 해독기의
 *   순회(pass1/pass2 = 후보 x 84형상 x 부스터변형, 밀착 = 후보 x 변형)는 매 호출마다
 *   layout() 을 새로 만들어 넘기므로 **같은 계산을 프레임당 수백~수천 번 반복**한다.
 *   실측(격자 L, N=128): dataCells 66.4ms 중 reservedAt 16,384회가 거의 전부이고,
 *   예약마스크만 있으면 셀 목록 재구성은 0.30ms — **221배**.
 *   ★자르는 게 아니라 같은 답을 다시 구하지 않는 것이라 **원리적으로 무손실**이다
 *     (순서·집합 그대로. 캐시가 비어도 예전 경로 그대로 돈다).
 *   ★저장하는 건 배열이 아니라 **비트마스크**다 — 셀 목록을 통째로 캐시하면 격자 L 한 건이
 *     ~0.6MB 라 폰에서 못 쓴다. 마스크는 N^2 비트 = 2KB.
 *   키 = reservedAt/insideShape 가 실제로 읽는 필드 전부:
 *     shape · N · hug(satHalo) · noGrooves · Rdata · coreCenter · anchors · bridgeRect.reserve
 *     + reservedAt 이 읽는 SPEC 값(satRadius·satHalo·coreOuter·orbitRadius·orbitDotR).
 *     ★SPEC 은 지금 코드베이스 어디서도 변형되지 않지만(전수 확인), 키가 '읽는 필드 전부'라고
 *       적어 놓고 일부를 빼면 그 주석이 다음 사람을 속인다. 상수 5개는 문자열 한 번이면 된다.
 *   customMask(임의 비트맵)와 SPEC 이 아닌 스펙은 키를 만들 수 없어 캐시하지 않는다.
 *   반환 배열은 읽기 전용으로만 쓰인다(codec.sampleCellGrays/sampleCellColor · render ·
 *   locate 격자재적합 — 전수 확인, 변형하는 소비처 없음). */
const _DC_CACHE = new Map(), _DC_MAX = 2048;
//   SPEC 은 위 가드(L.spec !== SPEC 이면 캐시 안 함)로 하나에 고정되므로 한 번만 만든다.
const _DC_SPEC_K = '|' + SPEC.satRadius + ',' + (SPEC.satHalo || 0) + ',' + SPEC.coreOuter
                 + ',' + SPEC.orbitRadius + ',' + SPEC.orbitDotR;
function _dcKey(L) {
  if (!L || L.shape === 'custom' || L.customMask) return null;
  if ((L.spec || SPEC) !== SPEC) return null;
  let k = L.shape + '|' + L.N + '|' + (L.hug ? 1 : 0) + '|' + (L.noGrooves ? 1 : 0)
        + '|' + (L.Rdata || 0) + '|' + L.coreCenter.mx + ',' + L.coreCenter.my;
  const A = L.anchors || [];
  for (let i = 0; i < A.length; i++) k += '|' + A[i].mx + ',' + A[i].my;
  k += _DC_SPEC_K;
  const r = L.bridgeRect && L.bridgeRect.reserve;
  k += '|' + (r ? r.mx0 + ',' + r.my0 + ',' + r.mx1 + ',' + r.my1 : '-');
  return k;
}
function dataCells(L) {
  const N = L.N, key = _dcKey(L);
  if (key === null) {                       // 캐시 불가(custom 등) — 예전 경로 그대로
    const out0 = [];
    for (let cy = 0; cy < N; cy++) for (let cx = 0; cx < N; cx++) {
      if (!reservedAt(cx + 0.5, cy + 0.5, L)) out0.push([cx, cy]);
    }
    return out0;
  }
  let m = _DC_CACHE.get(key);
  if (!m) {
    m = new Uint8Array((N * N + 7) >> 3);
    for (let cy = 0; cy < N; cy++) for (let cx = 0; cx < N; cx++) {
      if (!reservedAt(cx + 0.5, cy + 0.5, L)) { const i = cy * N + cx; m[i >> 3] |= (1 << (i & 7)); }
    }
    if (_DC_CACHE.size >= _DC_MAX) _DC_CACHE.clear();
    _DC_CACHE.set(key, m);
  }
  const out = [];
  for (let cy = 0; cy < N; cy++) for (let cx = 0; cx < N; cx++) {
    const i = cy * N + cx; if (m[i >> 3] & (1 << (i & 7))) out.push([cx, cy]);
  }
  return out;
}

// ── 한 점(모듈좌표)의 잉크 여부: true=검정 ────────────────────────────────
function inkAt(mx, my, L, cellBit) {
  const { coreCenter, anchors, orbit, spec } = L; const S = spec || SPEC;

  // ★코어+궤도를 한 번에 걸러낸다 — 둘 다 코어 중심 기준이라 바깥 원 하나로 끝난다.
  //   이미지의 98%는 여기서 곱셈 두 번에 빠져나간다(예전엔 매번 hypot 6회).
  const cdx = mx - coreCenter.mx, cdy = my - coreCenter.my;
  const cq = cdx * cdx + cdy * cdy;
  const RO = S.orbitRadius + S.orbitDotR + 0.4, RC = S.coreOuter;
  const ROUT = (RO > RC ? RO : RC);
  if (cq <= ROUT * ROUT * 1.0000001) {
    // 코어 동심원
    const dCore = Math.hypot(cdx, cdy);
    if (dCore <= S.coreOuter) {
      for (const ring of S.coreRings) if (dCore <= ring.r) return ring.ink;
      return false;
    }
    // 포맷 궤도 도트
    if (Math.abs(dCore - S.orbitRadius) <= S.orbitDotR + 0.4) {
      const od = S.orbitDotR;
      for (const o of orbit) {
        if (!o.on) continue;
        const dx = mx - o.mx; if (dx > od || dx < -od) continue;   // 정확한 기각(|dx|>r ⇒ 거리>r)
        const dy = my - o.my; if (dy > od || dy < -od) continue;
        if (within(dx, dy, od)) return true;
      }
    }
  }
  // 위성 — 사각 경계로 먼저 쳐낸다
  // ★밀착 모드에서는 위성이 **데이터 한복판**에 앉으므로 바깥에 흰 고리를 두른다.
  //   고리가 없으면 데이터 점과 붙어 위성 원판의 경계가 사라지고 무게중심이 편향된다.
  const SR = S.satRadius;
  const HALO = (L.hug && S.satHalo) ? S.satHalo : 0;
  const SRH = SR + HALO;
  for (let i = 1; i < anchors.length; i++) {
    const a = anchors[i];
    const dx = mx - a.mx; if (dx > SRH || dx < -SRH) continue;
    const dy = my - a.my; if (dy > SRH || dy < -SRH) continue;
    const ds = Math.hypot(dx, dy);
    if (ds <= SR) {
      if (a.type === 'donut') return ds >= S.satInner; // 도넛: 흰 중심
      return true;                                     // 원판
    }
    if (ds <= SRH) return false;                       // 고리 = 강제 흰색
  }
  // 데이터 도트 (라운드 채움): 예약 밖 + 셀 비트 on
  if (cellBit) {
    const cx = Math.floor(mx) + 0.5, cy = Math.floor(my) + 0.5;
    if (within(mx - cx, my - cy, S.dataDotR)) return true;
  }
  return false;
}

// ── 렌더: 오르빗 코드 이미지(RGBA) + 정답 앵커 픽셀좌표 ──────────────────
//   opts: { grid:'S', cellPx:6, quiet:4, seed:1, ss:2(슈퍼샘플), data:true }
/* ── 외곽 스트로크(인지 보강) ─────────────────────────────────────────────────
 *   실루엣 **밖 여백**에 흰 해자(gap 모듈)를 띄우고 그 바깥으로 w 모듈 두께의 검정 선을 두른다.
 *   데이터 셀·예약·셀 순서를 하나도 건드리지 않는다 → 포맷 무변경, 스캐너 수정 0.
 *
 *   ★왜 필요한가(B1 인지 게이트 실측): 코드를 크게 그릴수록 데이터 도트가 **개별 점으로 분해**되어
 *     실루엣이 "면"이 아니라 "점 구름"으로 보이고 가장자리 윤곽이 헤진다. 인지율이 오히려 떨어진다
 *     (catalog 80% → detail 71%, 두 모델 같은 방향). 외곽선을 명시적으로 그어야 면으로 읽힌다.
 *
 *   ★왜 해자(gap)가 필수인가(outline-lab probe 실측): 스트로크를 실루엣에 **밀착**시키면
 *     데이터 셀을 한 칸도 안 건드렸는데도 해독이 죽는다(pestbug·oksign). 적응형 이진화의 지역
 *     평균이 내려가 경계의 안티앨리어싱된 도트가 흰색으로 뒤집히기 때문. 1모듈 띄우면 산다.
 *
 *   ★왜 형상별 핀인가: 더 두껍거나 더 멀면 오목한 자리(고래 꼬리·지느러미 사이)를 메워
 *     새 검정 덩어리를 만든다 — 형상마다 (gap,w) 가 다르다. 전역 상수 불가.
 *
 *   ★QR 존은 비운다: 실루엣이 QR 자리를 벗어나는 형상에서 스트로크가 QR 을 가로질러 죽인다.
 */
/* ── B2-d 무지개 외곽선 (2026-09-04, 오너 제안) ─────────────────────────────
 *  wiabook.com 화제작 카드의 linear-gradient(135deg, …) 를 외곽 스트로크에 입힌다.
 *
 *  ★"밝기는 그대로 두고 색만" 은 **물리적으로 불가능**하다 — 검정 스트로크는 Y=0 이고
 *    Y=0 인 색은 검정 하나뿐이다(Y=0.299R+0.587G+0.114B). 그래서 대신 **밝기 상한**을 둔다:
 *    색상(hue)은 그대로 두고 Y 가 maxY 를 넘는 색만 눌러 내린다. 결과적으로 스트로크 전체가
 *    어두운 띠로 남아 적응형 이진화에서 여전히 "검정"으로 떨어진다.
 *    참고: 원본 그라디언트의 Y 는 노랑 225.9 ~ 보라 69.7 로 3배 넘게 벌어져 있어, 상한을 안 두면
 *    노랑·주황 구간이 회색변환에서 **흰색으로 뒤집혀 외곽선이 끊긴다**(스트로크의 존재 이유인
 *    인지율이 오히려 나빠진다).
 *  ★스트로크는 데이터 셀 밖(해자 바깥 띠)에만 칠해지므로 페이로드에 닿지 않는다. 그래도
 *    B2-c 가 닫은 결정(기울여 찍으면 3/15 종이 실루엣 경계와 겹쳐 깨진다)은 **색과 무관한
 *    기하 문제**라 무지개판도 똑같이 깨진다 — 그래서 이것도 **옵트인**이다. 기본값 켬 금지.
 */
const RAINBOW_STOPS = [
  [0xff,0x00,0x00],[0xff,0x88,0x00],[0xff,0xff,0x00],[0x00,0xcc,0x00],
  [0x00,0x88,0xff],[0x88,0x00,0xff],[0xff,0x00,0xff],[0xff,0x00,0x00]
];
const RAINBOW_MAX_Y = 96;          // 기본 밝기 상한. 낮출수록 차분·안전, 올릴수록 선명.
function _rainbowRGB(t, maxY) {
  const n = RAINBOW_STOPS.length - 1;
  let u = t - Math.floor(t);                      // 0~1 로 순환
  const f = u * n, i = Math.min(n - 1, Math.floor(f)), k = f - i;
  const a = RAINBOW_STOPS[i], b = RAINBOW_STOPS[i + 1];
  let R = a[0] + (b[0] - a[0]) * k, G = a[1] + (b[1] - a[1]) * k, B = a[2] + (b[2] - a[2]) * k;
  const Y = 0.299 * R + 0.587 * G + 0.114 * B;
  const cap = maxY != null ? maxY : RAINBOW_MAX_Y;
  if (Y > cap && Y > 0) { const q = cap / Y; R *= q; G *= q; B *= q; }   // 색상 유지, 밝기만 하강
  return [Math.round(R), Math.round(G), Math.round(B)];
}

function _dilateMask(src, W, H, r) {
  if (r <= 0) return src;
  const tmp = new Uint8Array(W * H), out = new Uint8Array(W * H);
  for (let y = 0; y < H; y++) { let run = 0; const row = y * W;
    for (let x = 0; x < W + r; x++) { if (x < W && src[row + x]) run = 2 * r + 1;
      if (run > 0 && x - r >= 0 && x - r < W) tmp[row + x - r] = 1; if (run > 0) run--; } }
  for (let x = 0; x < W; x++) { let run = 0;
    for (let y = 0; y < H + r; y++) { if (y < H && tmp[y * W + x]) run = 2 * r + 1;
      if (run > 0 && y - r >= 0 && y - r < H) out[(y - r) * W + x] = 1; if (run > 0) run--; } }
  return out;
}
function strokeOutline(data, dim, L, cellPx, quiet, gap, w, rainbow) {
  if (!(w > 0)) return 0;
  const N = L.N;
  const inside = new Uint8Array(dim * dim);
  for (let y = 0; y < dim; y++) for (let x = 0; x < dim; x++) {
    const mx = (x + 0.5) / cellPx - quiet, my = (y + 0.5) / cellPx - quiet;
    if (mx < 0 || my < 0 || mx >= N || my >= N) continue;
    if (insideShape(mx, my, L)) inside[y * dim + x] = 1;
  }
  const rGap = Math.round(gap * cellPx), rAll = Math.round((gap + w) * cellPx);
  const moat = rGap > 0 ? _dilateMask(inside, dim, dim, rGap) : inside;
  const band = _dilateMask(inside, dim, dim, rAll);
  // QR 예약 사각형(모듈) → 픽셀. 스트로크가 QR 을 가로지르지 않게 비운다.
  const br = L.bridgeRect || null;
  let qx0 = -1, qy0 = -1, qx1 = -1, qy1 = -1;
  if (br && br.reserve) {
    qx0 = (br.reserve.mx0 + quiet) * cellPx; qy0 = (br.reserve.my0 + quiet) * cellPx;
    qx1 = (br.reserve.mx1 + quiet) * cellPx; qy1 = (br.reserve.my1 + quiet) * cellPx;
  }
  let painted = 0;
  for (let y = 0; y < dim; y++) for (let x = 0; x < dim; x++) {
    const i = y * dim + x;
    if (!band[i] || moat[i]) continue;
    if (qx0 >= 0 && x >= qx0 && x <= qx1 && y >= qy0 && y <= qy1) continue;   // QR 존 비움
    const o = i * 4;
    if (rainbow) {
      // CSS linear-gradient(135deg) 와 같은 축: 화면좌표에서 오른쪽아래 방향 → t ∝ (x+y).
      const c = _rainbowRGB((x + y) / (2 * dim), rainbow.maxY);
      data[o] = c[0]; data[o + 1] = c[1]; data[o + 2] = c[2];
    } else { data[o] = 0; data[o + 1] = 0; data[o + 2] = 0; }
    data[o + 3] = 255; painted++;
  }
  return painted;
}

function render(opts) {
  const grid = opts.grid || 'S';
  const cellPx = opts.cellPx || 6;
  const quiet = opts.quiet != null ? opts.quiet : 4;
  const ss = opts.ss || 2;
  const withData = opts.data !== false;
  const spec = mergeSpec(opts.spec);
  const L = layout(grid, spec, opts.shape, { bridge: opts.bridge, bridgeLegacy: opts.bridgeLegacy, customMask: opts.customMask, hug: opts.hug, noGrooves: opts.noGrooves });
  const N = L.N;

  // 데이터 셀 → "도트 그레이" 맵 cellG (N*N, -1=도트없음). 다단계 지원:
  //   opts.cellGray(데이터셀 순서 0~255 그레이) = 다단계 payload(도트를 그 그레이로 채움),
  //   opts.bits = 1비트(1→검정도트 gray0, 0→도트없음) — 기존과 픽셀 동일,
  //   없으면 랜덤 검정도트(검출 클러터). 앵커(코어·궤도·위성)는 항상 검정(inkAt).
  const cellG = new Int16Array(N * N).fill(-1);
  // 컬러(hue) 레이어: opts.cellChroma[i]=[Cb',Cr'](데이터셀 순서, 0,0=무채색=오늘과 동일).
  //   밝기 Y=cellGray 는 그대로 → toGray 가 색을 무시(루마 레이어 무손상). 색은 Cb/Cr 에만.
  let cellCb = null, cellCr = null;
  if (withData && opts.cellChroma) {
    cellCb = new Int16Array(N * N); cellCr = new Int16Array(N * N);
    const dc = dataCells(L);
    for (let i = 0; i < dc.length; i++) { const c = dc[i], ch = opts.cellChroma[i]; if (ch) { cellCb[c[1] * N + c[0]] = ch[0]; cellCr[c[1] * N + c[0]] = ch[1]; } }
  }
  if (withData) {
    if (opts.cellGray) {
      const dc = dataCells(L);
      for (let i = 0; i < dc.length; i++) { const c = dc[i]; cellG[c[1] * N + c[0]] = opts.cellGray[i]; }
    } else if (opts.bits) {
      const dc = dataCells(L);
      for (let i = 0; i < dc.length; i++) { const c = dc[i]; if (opts.bits[i]) cellG[c[1] * N + c[0]] = 0; }
    } else {
      const rng = makeRng(opts.seed || 1);
      for (let cy = 0; cy < N; cy++) for (let cx = 0; cx < N; cx++) {
        if (!reservedAt(cx + 0.5, cy + 0.5, L) && rng() < 0.5) cellG[cy * N + cx] = 0;
      }
    }
  }
  const dataDotR = spec.dataDotR;

  // Y,Cb,Cr(0..255) → RGB (Rec.601). 컬러셀 렌더용. 무채색(Cb=Cr=128)이면 R=G=B=Y.
  const ycc2rgb = (Y, Cb, Cr) => {
    const cb = Cb - 128, cr = Cr - 128;
    return [Math.max(0, Math.min(255, Y + 1.402 * cr)),
            Math.max(0, Math.min(255, Y - 0.344136 * cb - 0.714136 * cr)),
            Math.max(0, Math.min(255, Y + 1.772 * cb))];
  };
  const dim = (N + 2 * quiet) * cellPx;
  const data = new Uint8ClampedArray(dim * dim * 4).fill(255);
  const inv = 1 / ss, base = (inv - 1) / 2 * inv; // 슈퍼샘플 오프셋
  const ss2 = ss * ss;

  // ── 앵커 근접 맵 (2026-08-29) ────────────────────────────────────────────
  // 앵커(코어·궤도·위성)가 차지하는 넓이는 그림의 **1.7%** 뿐인데, 예전엔 나머지 98%의
  // 픽셀도 서브샘플마다 inkAt() 을 불러 전수검사를 받았다. 모듈 한 칸 단위로 "여기 앵커가
  // 닿을 수 있나"를 미리 구워두고, 닿을 수 없는 칸은 호출 자체를 건너뛴다.
  //   · 판정은 **원판의 사각 경계**로 넉넉하게 — 실제보다 넓게 잡히는 건 안전한 쪽이다.
  //   · 한 칸 부풀린다(픽셀 하나가 모듈 두 칸에 걸칠 수 있으므로).
  //   · inkAt 은 여기서 cellBit=0 으로만 불리므로 데이터 도트는 이 맵과 무관하다.
  const AW = N + 2 * quiet;
  const AMASK = new Uint8Array(AW * AW);
  {
    const S2 = L.spec || spec;
    const rc = Math.max(S2.coreOuter, S2.orbitRadius + S2.orbitDotR + 0.4);
    const discs = [[L.coreCenter.mx, L.coreCenter.my, rc]];
    for (let i = 1; i < L.anchors.length; i++) discs.push([L.anchors[i].mx, L.anchors[i].my, S2.satRadius]);
    for (const [dx0, dy0, r] of discs) {
      const x0 = Math.floor(dx0 - r) - 1, x1 = Math.ceil(dx0 + r) + 1;
      const y0 = Math.floor(dy0 - r) - 1, y1 = Math.ceil(dy0 + r) + 1;
      for (let iy = y0; iy <= y1; iy++) {
        const ay = iy + quiet; if (ay < 0 || ay >= AW) continue;
        for (let ix = x0; ix <= x1; ix++) {
          const ax = ix + quiet; if (ax < 0 || ax >= AW) continue;
          AMASK[ay * AW + ax] = 1;
        }
      }
    }
  }

  // ── 좌표표 (2026-08-29) ─────────────────────────────────────────────────
  // 서브샘플 좌표는 **픽셀 좌표에만** 의존한다(x·y 공식이 같다) — 그런데 예전엔
  // 서브샘플마다 다시 계산했다: 나눗셈 2회 × 4서브샘플 × 168만 픽셀.
  // 축 하나 분량만 미리 굽고 x·y 양쪽이 같이 쓴다.
  //   ★식을 **한 글자도 바꾸지 않고** 그대로 옮겨 담는다 — 부동소수 결과가 달라지면
  //     도트 경계에서 픽셀이 흔들린다(그래서 1/cellPx 곱셈으로 바꾸지 않았다).
  const TM = new Float64Array(dim * ss);   // 모듈 좌표
  const TCB = new Int32Array(dim * ss);    // 그 좌표가 속한 셀
  const TD = new Float64Array(dim * ss);   // 셀 중심으로부터의 변위
  const TDQ = new Float64Array(dim * ss);  // 그 변위의 제곱
  const TA = new Int32Array(dim);          // 픽셀 → 모듈칸(quiet 포함)
  for (let c = 0; c < dim; c++) {
    TA[c] = Math.floor(c / cellPx);
    for (let k = 0; k < ss; k++) {
      const f = c + inv * k + base;
      const m = f / cellPx - quiet;
      const cb = Math.floor(m);
      const d = m - (cb + 0.5);
      const i = c * ss + k;
      TM[i] = m; TCB[i] = cb; TD[i] = d; TDQ[i] = d * d;
    }
  }
  const dotR2 = dataDotR * dataDotR;

  for (let py = 0; py < dim; py++) {
    const arow = TA[py];                             // 픽셀 → 모듈칸(quiet 포함 좌표)
    const arowOff = (arow >= 0 && arow < AW) ? arow * AW : -1;
    for (let px = 0; px < dim; px++) {
      const acol = TA[px];
      const nearAnchor = arowOff >= 0 && acol < AW && AMASK[arowOff + acol] === 1;
      let accR = 0, accG = 0, accB = 0;              // 서브샘플 RGB 누적
      for (let sy = 0; sy < ss; sy++) {
        const iy = py * ss + sy;
        const my = TM[iy], cby = TCB[iy], dy = TD[iy], dyq = TDQ[iy];
        const rowIn = cby >= 0 && cby < N, rowBase = cby * N;
        for (let sx = 0; sx < ss; sx++) {
          const ix = px * ss + sx;
          const mx = TM[ix];
          if (nearAnchor && inkAt(mx, my, L, 0)) continue;   // 앵커 잉크 = 검정(0,0,0)
          let R = 255, G = 255, B = 255;             // 기본 배경 = 흰색
          const cbx = TCB[ix];
          if (rowIn && cbx >= 0 && cbx < N) {
            const idx = rowBase + cbx, gv = cellG[idx];
            if (gv >= 0) {
              // within(dx,dy,dataDotR) 을 표에서 꺼낸 값으로 편 것 — 연산 순서까지 동일하다
              const q = TDQ[ix] + dyq;
              let hit;
              if (q < dotR2 * 0.9999999) hit = true;
              else if (q > dotR2 * 1.0000001) hit = false;
              else hit = Math.hypot(TD[ix], dy) <= dataDotR;
              if (hit) {
                if (cellCb && (cellCb[idx] || cellCr[idx])) { const c = ycc2rgb(gv, 128 + cellCb[idx], 128 + cellCr[idx]); R = c[0]; G = c[1]; B = c[2]; }
                else { R = G = B = gv; }
              }
            }
          }
          accR += R; accG += G; accB += B;
        }
      }
      const o = (py * dim + px) * 4;
      data[o] = Math.round(accR / ss2); data[o + 1] = Math.round(accG / ss2); data[o + 2] = Math.round(accB / ss2); data[o + 3] = 255;
    }
  }

  // 정답 앵커 픽셀좌표 (모듈중심 → 이미지 픽셀)
  const toPx = (m) => (m + quiet) * cellPx;
  const anchors = L.anchors.map(a => ({
    name: a.name, type: a.type,
    x: toPx(a.mx), y: toPx(a.my),
    mx: a.mx, my: a.my,
  }));

  // 외곽 스트로크 — **옵션이 있을 때만**. 없으면 위 픽셀 루프 결과가 그대로 나간다(픽셀 불변).
  if (opts.outline && opts.outline.w > 0) {
    strokeOutline(data, dim, L, cellPx, quiet,
      opts.outline.gap != null ? opts.outline.gap : 1, opts.outline.w,
      opts.outline.rainbow || null);
  }
  return { img: { data, width: dim, height: dim }, width: dim, height: dim,
           grid, N, cellPx, quiet, anchors, toPx, spec };
}

module.exports = { SPEC, GRID_CELLS, mergeSpec, layout, render, makeRng, dataCells, reservedAt, BRIDGE, BRIDGE_POS, BRIDGE_LEGACY, SHAPE_GROOVES, bridgeRect, strokeOutline, RAINBOW_STOPS, RAINBOW_MAX_Y, _rainbowRGB,
  // ★§7-36: 로스터 자가시험(test-codec.js)이 실루엣 판정을 직접 호출하려면 필요하다.
  insideShape };

});

__def("locate", function(module, exports){
'use strict';
/*
 * ============================================================================
 *  WIA Code v2 — 오르빗 앵커 정렬(localization) 매처
 * ============================================================================
 *  FRST 극대점 → 5개 앵커(코어 + 위성4) 배정 + 회전 라벨(북극성) + 모듈→이미지
 *  호모그래피. 이 파이프가 성공하면 하류(v1 코덱)는 검증된 셀 샘플링·RS·CRC를
 *  그대로 쓴다. 즉 이 파일이 "QR 파인더 목발"을 대체하는 유일한 신규 지점.
 *
 *  검출기는 이미지 외 정보를 쓰지 않는다(스케일/회전/코너위치 모름). 오직:
 *    1) 코어 = 불스아이 링 프로파일이 가장 뚜렷한 극대점
 *    2) 위성 = 코어 주위 4사분면에 하나씩, 링 프로파일 통과한 강한 극대점
 *    3) 북극성 = 위성 중 중심이 밝은(도넛) 하나 → TL 고정 → 회전 확정
 * ============================================================================
 */

const { radialProfile, refineCentroid } = require('./frst.js');
const { homography, applyH, sample } = require('./degrade.js');
const CN = require('./conic.js');

// 바이리니어 그레이 샘플
function grayAt(G, x, y) {
  const { g, w, h } = G;
  if (x < 0 || y < 0 || x >= w - 1 || y >= h - 1) return 255;
  const x0 = Math.floor(x), y0 = Math.floor(y), fx = x - x0, fy = y - y0, i = y0 * w + x0;
  return (g[i] * (1 - fx) + g[i + 1] * fx) * (1 - fy) + (g[i + w] * (1 - fx) + g[i + w + 1] * fx) * fy;
}

// 프로파일이 "밝은 바깥 배경 + 어두운 중심"인지(위성·코어 공통) + 반전 횟수.
function ringScore(prof) {
  // 정규화
  let mn = Infinity, mx = -Infinity;
  for (const v of prof) { if (v < mn) mn = v; if (v > mx) mx = v; }
  const rng = Math.max(1, mx - mn);
  const norm = prof.map(v => (v - mn) / rng);
  // 중심(첫 20%)은 어두워야, 바깥(마지막 20%)은 밝아야
  const k = norm.length;
  let cin = 0, nin = 0, cout = 0, nout = 0;
  for (let i = 0; i < k; i++) {
    if (i < k * 0.22) { cin += norm[i]; nin++; }
    if (i > k * 0.78) { cout += norm[i]; nout++; }
  }
  const centerDark = (cout / nout) - (cin / nin);   // >0 = 중심 어두움
  // 링 반전 횟수(불스아이일수록 큼)
  let rev = 0; const mid = 0.5;
  let above = norm[0] > mid;
  for (let i = 1; i < k; i++) { const a = norm[i] > mid; if (a !== above) { rev++; above = a; } }
  return { centerDark, rev, contrast: rng };
}

// 코어 후보 반경 추정: 프로파일 첫 상승 에지(어두운 중심 끝) × 경험비 → 모듈px.
// 실패해도 위성 탐색은 기하 일관성(호모그래피 잔차)로 커버하므로 대략치면 충분.
function estimateModulePx(grayObj, cx, cy) {
  const prof = radialProfile(grayObj, cx, cy, 60, 60);
  const rs = ringScore(prof);
  // 코어 바깥 검정 링(반경 ~5.5모듈)까지의 물리 반경을 프로파일 마지막 어두운 골로 추정
  // 간단화: 대칭맵이 이미 중심을 줬으므로 여기선 스케일만 러프 추정
  return { profile: prof, ring: rs };
}

// 두 벡터 각도(코어 기준) 사분면
function quadrant(dx, dy) { // 이미지 좌표(y 아래로 증가)
  if (dx >= 0 && dy < 0) return 0;   // 우상
  if (dx >= 0 && dy >= 0) return 1;  // 우하
  if (dx < 0 && dy >= 0) return 2;   // 좌하
  return 3;                          // 좌상
}

/*
 * locate(grayObj, peaksList, layout) → 결과
 *   layout = geometry.layout(grid) 의 { N, coreCenter, anchors } (모듈 좌표 정답 구조)
 *            ※ 스케일/회전 아님 — "모듈 격자 상의 앵커 배치"라는 포맷 상수일 뿐.
 *   반환: { ok, core:{x,y}, corners:{TL,TR,BR,BL}, northStar, Hmod2img, residPx }
 */
function locate(grayObj, peaksList, layout) {
  if (!peaksList || peaksList.length < 5) return { ok: false, reason: 'peaks<5', found: peaksList ? peaksList.length : 0 };

  // 1+2) 코어 + 위성4를 기하 불변 성질로 동시 선택:
  //   · 코어 후보 h 주위 4사분면에 각각 위성이 하나씩 있어야(원근·회전 불변)
  //   · 코어는 4위성의 무게중심에 위치(dCen≈0) — 코어가 아닌 후보(예: 코너 도넛)는
  //     나머지가 한쪽으로 몰려 사분면이 비거나 dCen이 큼 → 자동 배제.
  //   링 프로파일 기반 코어선별이 원근에서 도넛을 코어로 오인하던 문제를 근본 해결.
  const MINSEP = 12;
  const topH = peaksList.slice(0, 8);
  let core = null, sats = null, bestDCen = Infinity;
  for (const h of topH) {
    // h 를 코어로 가정. 주위 극대점 중 4개가 h 를 "둘러싸는가"를 사분면 비닝 없이
    // 각도 분산으로 판정(회전 불변 — roll 45°에서 사분면 경계 축퇴 문제 제거).
    const cand = peaksList.filter(p => Math.hypot(p.x - h.x, p.y - h.y) > MINSEP)
                          .sort((a, b) => b.score - a.score);
    const pick = [];
    for (const p of cand) { if (pick.some(s => Math.hypot(s.x - p.x, s.y - p.y) < MINSEP)) continue; pick.push(p); if (pick.length === 6) break; }
    if (pick.length < 4) continue;
    // 상위 후보들 중, h 를 가장 잘 둘러싸는(무게중심 근접 + 각도 고른) 4개 선택.
    // 상위 4개를 기본으로 하되 각도 분산 검사.
    const four = pick.slice(0, 4);
    const cx = four.reduce((s, p) => s + p.x, 0) / 4, cy = four.reduce((s, p) => s + p.y, 0) / 4;
    const meanD = four.reduce((s, p) => s + Math.hypot(p.x - h.x, p.y - h.y), 0) / 4;
    const dCen = Math.hypot(h.x - cx, h.y - cy) / (meanD || 1);
    // 각도 최대 간극: 4개가 한쪽에 몰리면 큰 간극 → 둘러싸지 못함
    const angs = four.map(p => Math.atan2(p.y - h.y, p.x - h.x)).sort((a, b) => a - b);
    let maxGap = 2 * Math.PI - (angs[3] - angs[0]);
    for (let i = 1; i < 4; i++) maxGap = Math.max(maxGap, angs[i] - angs[i - 1]);
    if (maxGap > 2.7) continue;                          // >155° 간극 = 둘러싸지 못함
    if (dCen < bestDCen) { bestDCen = dCen; core = { x: h.x, y: h.y }; sats = four.map(p => ({ x: p.x, y: p.y, score: p.score })); }
  }
  if (!core || !sats) return { ok: false, reason: 'no-core-surround' };
  if (bestDCen > 0.35) return { ok: false, reason: 'core-offcenter', dCen: +bestDCen.toFixed(2) };

  const spec = layout.spec || {};
  const satR = spec.satRadius || 2.5;
  const c2c = Math.hypot(layout.coreCenter.mx - layout.anchors[1].mx, layout.coreCenter.my - layout.anchors[1].my);
  const meanSatD = sats.reduce((s, p) => s + Math.hypot(p.x - core.x, p.y - core.y), 0) / 4;
  const satPx = Math.max(3, meanSatD * satR / (c2c || 1));   // 위성 반경(px) 러프추정

  // 3) 위성 중심 정밀화(라벨 무관). FRST 극대점은 도넛의 흰 중심이 아니라 링 위에
  //    찍힐 수 있음 → 먼저 암부 무게중심으로 참 중심(대칭중심=흰 구멍)으로 이동.
  //    원판은 무게중심=원판중심. 이후 밝기로 도넛 식별이 신뢰 가능해진다.
  const satRef = sats.map(s => {
    const r = refineCentroid(grayObj, s.x, s.y, satPx * 1.1);
    return (r.ok && Math.hypot(r.x - s.x, r.y - s.y) < satPx) ? { x: r.x, y: r.y } : { x: s.x, y: s.y };
  });

  // 4) 북극성(도넛): 정밀화된 중심의 그레이가 가장 밝은 위성. 도넛=흰 중심(≈255),
  //    원판=검정 중심(≈0). 정밀화 후엔 far 압축 원판(회색)도 도넛보다 어두워 안정.
  //    이 신호가 4겹 회전대칭을 깨므로 여기서 틀리면 배정이 90° 돈다.
  let donutIdx = 0, brightest = -Infinity;
  for (let i = 0; i < 4; i++) { const v = grayAt(grayObj, satRef[i].x, satRef[i].y); if (v > brightest) { brightest = v; donutIdx = i; } }

  // 5) 각도순(시계방향, 이미지 y-down) 정렬 → 도넛을 TL 에 맞춰 순환 배치 = 회전 확정.
  const orderCW = ['TL', 'TR', 'BR', 'BL'];
  const withAng = satRef.map((s, i) => ({ x: s.x, y: s.y, i, ang: Math.atan2(s.y - core.y, s.x - core.x) }));
  withAng.sort((a, b) => a.ang - b.ang);
  const donutPos = withAng.findIndex(s => s.i === donutIdx);
  const cornR = {};
  for (let i = 0; i < 4; i++) cornR[orderCW[i]] = withAng[(donutPos + i) % 4];

  // 6) 코어 정밀화(중앙 원판만 — 바깥 링은 원근에서 무게중심 편향) + 호모그래피.
  const corePx = Math.max(2, meanSatD * 1.5 / (c2c || 1));
  const cr = refineCentroid(grayObj, core.x, core.y, corePx * 1.1);
  const coreR = (cr.ok && Math.hypot(cr.x - core.x, cr.y - core.y) < corePx) ? { x: cr.x, y: cr.y } : { x: core.x, y: core.y };

  const A = orderCW.map(k => { const a = layout.anchors.find(x => x.name === k); return [a.mx, a.my]; });
  const B = orderCW.map(k => [cornR[k].x, cornR[k].y]);
  A.push([layout.coreCenter.mx, layout.coreCenter.my]); B.push([coreR.x, coreR.y]);
  const Hseed = homographyLS(A, B);
  if (!Hseed) return { ok: false, reason: 'homography-fail' };

  // 7) 2단계 정밀화: seed H의 앵커별 국소 스케일로 창을 재조정. 균일창(1단계)은
  //    도넛 식별엔 충분하나 압축 far 위성엔 과대 → 클러터 혼입. 앵커별 창이 정밀도↑.
  function localMod(mx, my) { const a = applyH(Hseed, mx, my), b = applyH(Hseed, mx + 1, my), c = applyH(Hseed, mx, my + 1);
    return (Math.hypot(b[0] - a[0], b[1] - a[1]) + Math.hypot(c[0] - a[0], c[1] - a[1])) / 2; }
  const satInner = spec.satInner || 1.0;
  function fine(pt, mx, my, rMod, bright) {
    const R = Math.max(2, rMod * localMod(mx, my) * 1.05);
    const r = refineCentroid(grayObj, pt.x, pt.y, R, bright);
    return (r.ok && Math.hypot(r.x - pt.x, r.y - pt.y) < R * 0.6) ? { x: r.x, y: r.y } : { x: pt.x, y: pt.y };
  }
  const coreF = fine(coreR, layout.coreCenter.mx, layout.coreCenter.my, 1.5);
  const cornF = {};
  // TL=도넛: 흰 구멍(solid 밝은 blob)을 밝기중심으로 → 링 암부중심보다 정밀.
  //          창은 구멍 반경(satInner)에 맞춤. 나머지 원판은 암부중심.
  cornF.TL = fine(cornR.TL, layout.anchors.find(a => a.name === 'TL').mx, layout.anchors.find(a => a.name === 'TL').my, satInner * 1.3, true);
  for (const k of ['TR', 'BR', 'BL']) { const a = layout.anchors.find(x => x.name === k); cornF[k] = fine(cornR[k], a.mx, a.my, satR); }

  const A2 = orderCW.map(k => { const a = layout.anchors.find(x => x.name === k); return [a.mx, a.my]; });
  const B2 = orderCW.map(k => [cornF[k].x, cornF[k].y]);
  A2.push([layout.coreCenter.mx, layout.coreCenter.my]); B2.push([coreF.x, coreF.y]);
  const H = homographyLS(A2, B2) || Hseed;
  let resid = 0;
  for (let i = 0; i < A2.length; i++) { const [px, py] = applyH(H, A2[i][0], A2[i][1]); resid += Math.hypot(px - B2[i][0], py - B2[i][1]); }
  resid /= A2.length;

  const vf = verifyLayout(grayObj, H, layout);   // residPx 는 grid 판별불가 → 독립 검증신호 동봉
  return { ok: true, core: coreF,
           corners: { TL: cornF.TL, TR: cornF.TR, BR: cornF.BR, BL: cornF.BL },
           northStar: 'TL', donutGray: +brightest.toFixed(0),
           Hmod2img: H, residPx: +resid.toFixed(2), dCen: +bestDCen.toFixed(3),
           modPx: vf.modPx, orbitMatch: vf.orbitMatch, orbitContrast: vf.orbitContrast };
}

/*
 * verifyLayout — 잠금(H)의 grid/cellPx 검증 신호 2종 (2026-08-10).
 *   residPx(5점 자기잔차)는 grid 판별력이 0 이다: S/M/L(및 사각↔링) 앵커 배치가
 *   전부 "중심+대칭4점" 닮음꼴이라, 8자유도 호모그래피가 스케일·회전을 통째로
 *   흡수해 어떤 layout 가정에도 같은 물리 4점에 거의 정확히 들어맞는다(실측:
 *   클린 합성서도 오답 grid residPx 0.00~0.25). → 앵커 밖의 독립 증거로 판별:
 *   · modPx      : H가 함의하는 코어 국소 모듈크기(px). 코어 링 실측 cellPx와
 *                  대조하면 스케일 흡수를 잡는다(오답 grid는 ≥36% 어긋남).
 *   · orbitMatch : 포맷 궤도 24도트 on/off 패턴의 재투영 일치율(0~1). 스케일이
 *                  거의 같은 근사합동 쌍(M-square↔L-round, 45° 회전 차)까지 판별.
 *                  실측: 정답 1.000, 오답 0.33~0.63 (실사진·노이즈σ45·블러r2·
 *                  0.5축소·yaw30 전부). 대비<30 이면 정보없음 → 0.5 반환.
 */
function verifyLayout(grayObj, H, layout) {
  const mx = layout.coreCenter.mx, my = layout.coreCenter.my;
  const a = applyH(H, mx, my), b = applyH(H, mx + 1, my), c = applyH(H, mx, my + 1);
  const modPx = (Math.hypot(b[0] - a[0], b[1] - a[1]) + Math.hypot(c[0] - a[0], c[1] - a[1])) / 2;
  let orbitMatch = 0.5, orbitContrast = 0;
  if (layout.orbit && layout.orbit.length) {
    const vals = layout.orbit.map(d => { const p = applyH(H, d.mx, d.my); return grayAt(grayObj, p[0], p[1]); });
    let mn = Infinity, mxv = -Infinity;
    for (const v of vals) { if (v < mn) mn = v; if (v > mxv) mxv = v; }
    orbitContrast = mxv - mn;
    if (orbitContrast >= 30) {
      const t = (mn + mxv) / 2;
      let m = 0;
      for (let i = 0; i < vals.length; i++) if ((vals[i] < t) === layout.orbit[i].on) m++;
      orbitMatch = m / vals.length;
    }
  }
  return { modPx: +modPx.toFixed(2), orbitMatch: +orbitMatch.toFixed(3), orbitContrast: Math.round(orbitContrast) };
}

// 최소자승 호모그래피 (n≥4 대응, DLT 정규방정식). A[i]→B[i].
function homographyLS(A, B) {
  const n = A.length;
  if (n === 4) return homography(A, B);
  // 8미지수 정규방정식 (2n×8)
  const rows = [], rhs = [];
  for (let i = 0; i < n; i++) {
    const [X, Y] = A[i], [x, y] = B[i];
    rows.push([X, Y, 1, 0, 0, 0, -X * x, -Y * x]); rhs.push(x);
    rows.push([0, 0, 0, X, Y, 1, -X * y, -Y * y]); rhs.push(y);
  }
  // 정규방정식 M^T M h = M^T r  (8×8)
  const MtM = Array.from({ length: 8 }, () => new Float64Array(8));
  const Mtr = new Float64Array(8);
  for (let k = 0; k < rows.length; k++) {
    const row = rows[k], rv = rhs[k];
    for (let a = 0; a < 8; a++) { Mtr[a] += row[a] * rv; for (let b = 0; b < 8; b++) MtM[a][b] += row[a] * row[b]; }
  }
  const h = solveN(MtM, Mtr, 8); if (!h) return null;
  return [h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], 1];
}

// 일반 n×n 가우스 소거
function solveN(A, b, n) {
  const M = A.map(r => Array.from(r)), v = Array.from(b);
  for (let i = 0; i < n; i++) {
    let piv = i; for (let r = i + 1; r < n; r++) if (Math.abs(M[r][i]) > Math.abs(M[piv][i])) piv = r;
    if (Math.abs(M[piv][i]) < 1e-12) return null;
    [M[i], M[piv]] = [M[piv], M[i]]; [v[i], v[piv]] = [v[piv], v[i]];
    const d = M[i][i];
    for (let c = i; c < n; c++) M[i][c] /= d; v[i] /= d;
    for (let r = 0; r < n; r++) { if (r === i) continue; const f = M[r][i]; for (let c = i; c < n; c++) M[r][c] -= f * M[i][c]; v[r] -= f * v[i]; }
  }
  return v;
}

/* ★2026-09-01 — 불스아이 템플릿 적합도 (0.05..~1). 배율을 후보 **자신의** 첫 상승 에지에서
 *   유도한다(r1 ≈ 1.5c). 그러면 코어는 1.5c 어둠 / 3.5c 밝음 / 5.5c 어둠 / 바깥 밝음이
 *   제자리에 오고, 위성 원판(첫 에지 2.5c)이나 도넛(중심이 밝아 첫 에지 없음)은 이 밴드가
 *   맞지 않는다. 프레임 비율 하나(0.05·mn)에 묶인 탐침은 배율에 따라 코어 서명을 깼고
 *   (heart/M: rev 2·centerDark 0), "여러 R 중 최대"는 위성이 유리한 R 을 고르게 했다. */
function bullseyeFit(grayObj, x, y) {
  const mn = Math.min(grayObj.w, grayObj.h), R = 0.2 * mn, steps = Math.max(40, Math.round(R));
  const prof = radialProfile(grayObj, x, y, R, steps), k = prof.length, st = R / k;
  let mn2 = Infinity, mx2 = -Infinity; for (let i = 0; i < k; i++) { if (prof[i] < mn2) mn2 = prof[i]; if (prof[i] > mx2) mx2 = prof[i]; }
  const rng = Math.max(1, mx2 - mn2);
  const nz = i => (prof[Math.min(k - 1, Math.max(0, i))] - mn2) / rng;
  if (nz(0) > 0.5) return 0.05;                         // 중심이 밝다(도넛) → 불스아이 아님
  let i1 = -1; for (let i = 1; i < k; i++) if (nz(i) > 0.5) { i1 = i; break; }
  if (i1 < 2) return 0.05;                              // 첫 에지가 없거나 너무 가깝다
  const c = ((i1 + 0.5) * st) / 1.5;                    // 모듈 px 추정
  if (6.5 * c > R) return 0.05;                         // 탐침이 바깥 밝음 밴드까지 못 미친다
  const band = (a, b) => { let s = 0, n = 0; for (let i = Math.round(a * c / st); i <= Math.round(b * c / st); i++) { s += nz(i); n++; } return n ? s / n : 0.5; };
  const D0 = band(0.2, 1.1), L1 = band(1.9, 3.1), D2 = band(3.9, 5.1), L3 = band(5.9, 6.5);
  const fit = (L1 - D0 + L1 - D2 + L3 - D2) / 3;      // 이상적 1, 원판 ≈ 0.3 이하, 잡음 ≈ 0
  return Math.max(0.05, fit);
}

// ── 코어 시드 선택 (surround 실패 시): 불스아이 템플릿에 가장 잘 맞는 피크 ──
//   ★2026-09-01 개정. 이전 식 score×(1+rev)×(centerDark>0?1:0.3) 은 두 가지로 깨졌다
//   (SVG_CORESEED_HANDOFF_2026-09-01.md 실측):
//   ① rProbe(0.05·프레임)가 배율에 안 맞으면 코어 서명이 사라진다 — heart/M 640px 에선
//      탐침이 코어 바깥 링(5.5모듈=33.8px) 안에서 끝나 rev 2·centerDark 0 → 코어(FRST 15,792,
//      위성의 2.2배)가 도넛에 졌다. ② 탐침이 충분한 cellPx=10(생성기 기본 1360px)에선 위성
//      반경 25px 가 coarse 투표 반경 25 와 겹쳐 위성 원시점수가 코어를 넘고, 코어는 rev 배수로만
//      ×1.02~1.06 여유로 이겼다 → 벡터 SVG 래스터의 앨리어싱 한 번에 뒤집혀 weak-core.
//   "코어는 압도적"이라는 옛 주석은 밀착(8/30) 이후 사실이 아니다. 66종×3배율 198건 최소여유:
//   옛 식 ×1.02 → 이 식 ×2.16(bird@10). bench-detect 회귀 0 · HUG_ON 왕복 500/500 · arena/yaw30 통과.
//   (rProbe 인자는 호환을 위해 남겨 두지만 쓰지 않는다 — 배율은 후보 자신이 말한다.)
function pickCoreSeed(grayObj, peaksList, rProbe) {
  let best = null, bestS = -Infinity;
  for (const p of peaksList.slice(0, 10)) {
    const s = (p.score || 1) * bullseyeFit(grayObj, p.x, p.y);
    if (s > bestS) { bestS = s; best = p; }
  }
  return best;
}

// H(3×3)로 이미지 워프(역샘플). 반환 RGBA {data,width,height}.
function warpImage(img, Himg2rect, dim) {
  const Hinv = CN.inv3(Himg2rect); if (!Hinv) return null;
  const out = { data: new Uint8ClampedArray(dim * dim * 4).fill(255), width: dim, height: dim };
  for (let Y = 0; Y < dim; Y++) for (let X = 0; X < dim; X++) {
    const p = CN.matVec3(Hinv, [X, Y, 1]); const px = sample(img, p[0] / p[2], p[1] / p[2]);
    const o = (Y * dim + X) * 4; out.data[o] = px[0]; out.data[o + 1] = px[1]; out.data[o + 2] = px[2]; out.data[o + 3] = 255;
  }
  return out;
}

/*
 * locateRobust — orbit 매처 우선, 실패(주로 급격 원근의 no-core-surround)시
 *   코어 동심원 conic-pencil 로 정면화(rectify)→재검출→호모그래피 합성.
 *   img      : 원본(열화된) RGBA. 정면화 워프용.
 *   grayObj  : img 의 그레이. peaksList : 그 위 FRST 극대점.
 *   opts.redetect(rectImg) → { gray, peaks } : 정면화 이미지 재검출(하네스 파라미터 재사용).
 *   opts.cellPx : 모듈 픽셀(정면화 목표 스케일).
 *   반환은 locate 와 동일한 필드 + method:'orbit'|'conic'. 좌표는 항상 원본 이미지 프레임.
 */
/* fitSimilarity — (X,Y) → (u,v) 를 similarity 4자유도로 최소제곱 적합 (2026-08-30 Q4)
 *   u = aX − bY + tx ,  v = bX + aY + ty       (a,b 가 배율·회전을 함께 담는다)
 *   점 n 개면 식이 2n 개, 미지수는 4개 → **n ≥ 3 이면 과결정**이라 잔차가 의미를 갖는다.
 *   (자유 호모그래피는 미지수 8개라 4점에서 잔차가 항상 0 — 어제 locate3 가 죽은 이유다.) */
function fitSimilarity(A, B) {
  var M = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]], r = [0,0,0,0];
  for (var i = 0; i < A.length; i++) {
    var X = A[i][0], Y = A[i][1], u = B[i][0], v = B[i][1];
    var rows = [[X, -Y, 1, 0, u], [Y, X, 0, 1, v]];
    for (var k = 0; k < 2; k++) {
      var row = rows[k];
      for (var a = 0; a < 4; a++) { r[a] += row[a] * row[4]; for (var b = 0; b < 4; b++) M[a][b] += row[a] * row[b]; }
    }
  }
  var G = M.map(function (row, i2) { return row.concat([r[i2]]); });
  for (var p = 0; p < 4; p++) {
    var piv = p;
    for (var q = p + 1; q < 4; q++) if (Math.abs(G[q][p]) > Math.abs(G[piv][p])) piv = q;
    var t = G[p]; G[p] = G[piv]; G[piv] = t;
    var d = G[p][p]; if (Math.abs(d) < 1e-12) return null;
    for (var c = p; c < 5; c++) G[p][c] /= d;
    for (var q2 = 0; q2 < 4; q2++) { if (q2 === p) continue; var f = G[q2][p]; for (var c2 = p; c2 < 5; c2++) G[q2][c2] -= f * G[p][c2]; }
  }
  return { a: G[0][4], b: G[1][4], tx: G[2][4], ty: G[3][4] };
}

/*
 * locateSim3 — 정면화 프레임에서 **위성 3개**로 확정한다 (2026-08-30 Q4)
 *   conic 정면화가 사영+아핀 4자유도를 이미 고정했으므로 잔여는 similarity 4자유도다.
 *   코어 + 위성 3 = 8식/4미지수 = **과결정** → 잔차 게이트가 살아난다.
 *   ★prep(Q1-B 의 공유 정면화)을 그대로 쓴다 — 워프를 새로 하지 않는다(추가 비용 0).
 *   ★도넛(12시)이 가려지면 포기한다 — 회전을 정할 근거가 없다.
 */
function locateSim3(grayObj, peaksList, layout, prep, cellPx) {
  if (!prep || !prep.gray || !prep.peaks) return { ok: false, reason: 'no-prep' };
  var spec = layout.spec || {};
  var satR = spec.satRadius || 2.5;
  var cx = prep.dim / 2, cy = prep.dim / 2;      // 정면화가 코어를 캔버스 중앙에 놓는다
  // 정면화 프레임에서 1 모듈 = cellPx 픽셀 (rectifyHomography 가 코어 바깥링 5.5모듈을
  //   5.5*cellPx 로 맞추므로). 그래서 아래 거리 비교가 **모듈 단위**로 성립한다.
  var Rs = Math.hypot(layout.anchors[1].mx - layout.coreCenter.mx,
                      layout.anchors[1].my - layout.coreCenter.my);
  // ★C3(2026-09-04): 밀착 핀은 마름모(rA≠rB — hug-on 79종 중 56종이 12% 넘게 다르다)라
  //   반경 하나로 거르면 위성 절반이 창 밖이다(whale/L 13.75 vs 21.75 → 위성 0개 → sats<3, 실측).
  //   레이아웃의 위성 4개 거리를 전부 모아(5% 안이면 하나로) 어느 하나에라도 15% 안이면 받는다.
  //   원형 배치는 Rlist 가 원소 1개라 예전과 완전히 같다.
  var Rlist = [];
  ['TL','TR','BR','BL'].forEach(function(nm){ var a = layout.anchors.find(function(x){ return x.name === nm; }); if (!a) return;
    var d = Math.hypot(a.mx - layout.coreCenter.mx, a.my - layout.coreCenter.my);
    for (var ri = 0; ri < Rlist.length; ri++) if (Math.abs(Rlist[ri] - d) <= 0.05 * Rlist[ri]) return;
    Rlist.push(d); });
  if (!Rlist.length) Rlist = [Rs];

  // 코어 정밀화(정면화 프레임)
  var cr = refineCentroid(prep.gray, cx, cy, 1.5 * cellPx * 1.1);
  var coreR = (cr.ok && Math.hypot(cr.x - cx, cr.y - cy) < 2 * cellPx) ? { x: cr.x, y: cr.y } : { x: cx, y: cy };

  // 위성 후보 — ★**정밀화는 원본 프레임에서** 한다 (2026-08-30 실측으로 고침).
  //   처음엔 정면화 이미지(prep.gray)에서 정밀화했더니 잔차 1.17px 였다(cellPx 4.7 의 25%).
  //   정면화본은 워프로 리샘플된 이미지라 가장자리가 뭉개져 무게중심이 부정확하다.
  //   원본에서 정밀화한 뒤 정면화 좌표로 사상하면 원본의 선명도를 그대로 쓴다.
  var toRect = function (pt) { var q = CN.matVec3(prep.H, [pt.x, pt.y, 1]); return { x: q[0] / q[2], y: q[1] / q[2] }; };
  var sats = [];
  var src = (peaksList && peaksList.length) ? peaksList : prep.peaks;
  var srcIsImg = !!(peaksList && peaksList.length);
  for (var i = 0; i < src.length; i++) {
    var p0 = src[i];
    // 정면화 좌표로 옮겨 거리(모듈)를 잰다 — 정면화 프레임은 metric 이다.
    var pr = srcIsImg ? toRect(p0) : p0;
    var dMod = Math.hypot(pr.x - coreR.x, pr.y - coreR.y) / cellPx;
    var okR = false; for (var rj = 0; rj < Rlist.length; rj++) if (Math.abs(dMod - Rlist[rj]) <= Rlist[rj] * 0.15) { okR = true; break; }
    if (!okR) continue;
    var fxi, fyi;
    if (srcIsImg) {
      var rf = refineCentroid(grayObj, p0.x, p0.y, satR * cellPx * 1.1);   // ★원본에서 정밀화
      fxi = rf.ok ? rf.x : p0.x; fyi = rf.ok ? rf.y : p0.y;
    } else { fxi = p0.x; fyi = p0.y; }
    var pf = srcIsImg ? toRect({ x: fxi, y: fyi }) : { x: fxi, y: fyi };
    var dup = false;
    for (var j = 0; j < sats.length; j++) if (Math.hypot(sats[j].x - pf.x, sats[j].y - pf.y) < 3 * cellPx) { dup = true; break; }
    if (dup) continue;
    sats.push({ x: pf.x, y: pf.y,
                bright: srcIsImg ? grayAt(grayObj, fxi, fyi) : grayAt(prep.gray, fxi, fyi),
                score: p0.score });
  }
  if (sats.length < 3) return { ok: false, reason: 'sim3-sats<3', found: sats.length };
  sats.sort(function (a, b) { return b.score - a.score; });
  var use = sats.slice(0, 4);

  // 도넛(북극성) = 중심이 가장 밝은 것. 원판과의 차가 작으면 도넛이 가려진 것으로 보고 포기.
  var di = 0, bm = -Infinity;
  for (var k2 = 0; k2 < use.length; k2++) if (use[k2].bright > bm) { bm = use[k2].bright; di = k2; }
  var others = [];
  for (var k3 = 0; k3 < use.length; k3++) if (k3 !== di) others.push(use[k3].bright);
  // ★도넛이 안 보이는 경우 — 포기하지 않는다 (2026-08-30 Fable 2차).
  //   위성 3개가 90° 간격으로 보이면 **빈 슬롯 자체가 회전**이다:
  //   셋이 전부 암부(원판)면 가려진 것이 도넛이고, **빈 자리가 TL** 이다.
  //   안전판: 정확히 3개이고 최대 밝기가 128 미만(= 전부 어둡다)일 때만 추론한다.
  //   밝은 가짜 피크가 슬롯을 채우면 지금처럼 CRC 가 거른다(현행 대비 악화 없음).
  var donutInferred = false;
  if (others.length && bm - Math.max.apply(null, others) < 40) {
    // ★C3(2026-09-04): 전부 어두운데 후보가 4개면 하나는 가짜(데이터 도트 뭉치·궤도점)다. 그 가짜가 낀 채로
    //   '정확히 3개'를 요구하면 도넛 가림이 통째로 죽는다(whale TL 실측). 점수 상위 3개만 남기고 추론한다 —
    //   가짜가 진짜보다 높으면 슬롯 충돌·적합 잔차·CRC 가 거른다(현행 대비 악화 없음).
    if (bm < 128 && use.length >= 3) { use = use.slice(0, 3); donutInferred = true; }
    else return { ok: false, reason: 'sim3-donut-missing' };
  }

  // 각도 라벨 — 정면화 프레임은 metric 이라 90° 간격으로 떨어진다.
  var names = ['TL', 'TR', 'BR', 'BL'];
  var labeled = {};
  if (donutInferred) {
    // 셋 다 원판이다 → 빈 슬롯이 도넛(TL). 임의의 하나를 기준 0 으로 잡고 슬롯을 채운 뒤,
    // 비어 있는 슬롯이 TL 이 되도록 전체를 회전시킨다.
    var base = use[0];
    var a0 = Math.atan2(base.y - coreR.y, base.x - coreR.x);
    var slots = [null, null, null, null];
    for (var mi = 0; mi < use.length; mi++) {
      var rl = (Math.atan2(use[mi].y - coreR.y, use[mi].x - coreR.x) - a0) / (Math.PI / 2);
      var sl = ((Math.round(rl) % 4) + 4) % 4;
      if (slots[sl]) return { ok: false, reason: 'sim3-slot-collide' };
      slots[sl] = use[mi];
    }
    var empty = slots.indexOf(null);
    if (empty < 0) return { ok: false, reason: 'sim3-no-empty' };
    for (var si = 0; si < 4; si++) {
      if (!slots[si]) continue;
      labeled[names[(si - empty + 4) % 4]] = slots[si];   // 빈 자리를 TL(0)로 맞춰 회전
    }
  } else {
    var donut = use[di];
    var ang0 = Math.atan2(donut.y - coreR.y, donut.x - coreR.x);
    labeled.TL = donut;
    for (var m = 0; m < use.length; m++) {
      if (use[m] === donut) continue;
      var rel = (Math.atan2(use[m].y - coreR.y, use[m].x - coreR.x) - ang0) / (Math.PI / 2);
      var slot = ((Math.round(rel) % 4) + 4) % 4;
      if (slot >= 1 && slot <= 3 && !labeled[names[slot]]) labeled[names[slot]] = use[m];
    }
  }
  var found = Object.keys(labeled);
  if (found.length < 3) return { ok: false, reason: 'sim3-slots<3', found: found.length };

  // similarity 적합: 모듈좌표 → 정면화 프레임
  var A = [[layout.coreCenter.mx, layout.coreCenter.my]], B = [[coreR.x, coreR.y]];
  for (var f1 = 0; f1 < found.length; f1++) {
    var an = layout.anchors.find(function (x) { return x.name === found[f1]; });
    A.push([an.mx, an.my]); B.push([labeled[found[f1]].x, labeled[found[f1]].y]);
  }
  var sim = fitSimilarity(A, B);
  if (!sim) return { ok: false, reason: 'sim3-fit' };

  // ★2단계 정밀화 — `locate()` 의 fine() 과 같은 원리 (2026-08-30).
  //   1단계는 FRST 극대점을 균일 창으로 정밀화한 것이라 오차가 남는다(실측 resid 1.19px,
  //   cellPx 4.7 의 25% — 셀 표본을 어긋나게 하기에 충분해서 "락만" 으로 끝났다).
  //   적합된 sim 이 각 앵커의 **예상 위치**를 주므로, 그 자리에서 앵커에 맞는 창으로
  //   다시 정밀화한다: 도넛은 **밝은 중심**(흰 구멍, satInner 창), 원판은 **암부 중심**(satR 창).
  //   ★정밀화는 원본 프레임에서 한다 — 정면화본은 리샘플이라 가장자리가 뭉개진다.
  var satInner = spec.satInner || 1.0;
  var toImg = function (pt) { var q = CN.matVec3(prep.Hinv, [pt.x, pt.y, 1]); return { x: q[0] / q[2], y: q[1] / q[2] }; };
  var imgPts = null;      // ★원본 프레임 좌표를 보관한다 — 아래 내삽 재적합이 이걸 쓴다
  for (var pass = 0; pass < 2; pass++) {
    var B2 = [], IP = [];
    for (var a2 = 0; a2 < A.length; a2++) {
      var ex = sim.a * A[a2][0] - sim.b * A[a2][1] + sim.tx;
      var ey = sim.b * A[a2][0] + sim.a * A[a2][1] + sim.ty;
      var isCore = (a2 === 0);
      var isDonut = (!isCore && found[a2 - 1] === 'TL' && !donutInferred);
      var rMod = isCore ? 1.5 : (isDonut ? satInner * 1.3 : satR);
      var pi2 = toImg({ x: ex, y: ey });
      var rf2 = refineCentroid(grayObj, pi2.x, pi2.y, Math.max(2, rMod * cellPx * 1.05), isDonut);
      if (rf2.ok && Math.hypot(rf2.x - pi2.x, rf2.y - pi2.y) < rMod * cellPx * 0.6) {
        var pr2 = toRect({ x: rf2.x, y: rf2.y });
        B2.push([pr2.x, pr2.y]); IP.push([rf2.x, rf2.y]);
      } else { B2.push([ex, ey]); IP.push([pi2.x, pi2.y]); }
    }
    var sim2 = fitSimilarity(A, B2);
    if (!sim2) break;
    sim = sim2; B = B2; imgPts = IP;
  }

  // ★배율 검증 — 정면화 프레임의 1모듈은 cellPx 픽셀이므로 similarity 배율은 cellPx 여야 한다.
  //   벗어나면 위성이 아닌 것을 주웠다는 뜻이다(잔차만으로는 못 거르던 자리를 여기서 막는다).
  var scale = Math.hypot(sim.a, sim.b);
  if (!(scale > 0) || Math.abs(Math.log(scale / cellPx)) > 0.12) {
    return { ok: false, reason: 'sim3-scale', scale: +scale.toFixed(2), cellPx: +cellPx.toFixed(2) };
  }

  // 과결정 잔차 — 여기서는 **의미가 있다**(8식/4미지수).
  var resid = 0;
  for (var r2 = 0; r2 < A.length; r2++) {
    var ux = sim.a * A[r2][0] - sim.b * A[r2][1] + sim.tx;
    var uy = sim.b * A[r2][0] + sim.a * A[r2][1] + sim.ty;
    resid += Math.hypot(ux - B[r2][0], uy - B[r2][1]);
  }
  resid /= A.length;

  // 정면화 프레임 → 원본 이미지 프레임으로 합성
  // ★★행렬 형식이 두 벌이다 (2026-08-30 실측으로 잡음):
  //   `homographyLS` 는 **평탄 9배열** `[h0..h8]` 을 돌려주고 `applyH` 도 그 형식을 받는다.
  //   `CN.mul3`·`CN.matVec3` 는 **중첩 3×3** `[[..],[..],[..]]` 을 쓴다.
  //   섞으면 `applyH` 가 조용히 `[null,null]` 을 낸다 — 던지지 않는다.
  //   실제 피해: 기하는 완벽했는데(코어 오차 0.2px, 회전 0.0°) `verifyLayout` 의
  //   modPx 가 NaN 이 되어 scaleErr 가 NaN → loose-lock 으로 버려졌다.
  //   여기서는 mul3 로 곱한 뒤 **평탄화해서** 내보낸다.
  var Hsim = [[sim.a, -sim.b, sim.tx], [sim.b, sim.a, sim.ty], [0, 0, 1]];
  var Hn = CN.mul3(prep.Hinv, Hsim);
  if (!Hn) return { ok: false, reason: 'sim3-compose' };
  var H = [Hn[0][0], Hn[0][1], Hn[0][2], Hn[1][0], Hn[1][1], Hn[1][2], Hn[2][0], Hn[2][1], Hn[2][2]];
  if (H[8]) { for (var hi = 0; hi < 9; hi++) H[hi] /= H[8]; }   // h8=1 로 정규화(다른 경로와 동일)

  var back = function (pt) { var q = CN.matVec3(prep.Hinv, [pt.x, pt.y, 1]); return { x: q[0] / q[2], y: q[1] / q[2] }; };
  var corners = {};
  for (var c3 = 0; c3 < 4; c3++) {
    var nm = names[c3];
    if (labeled[nm]) { corners[nm] = back(labeled[nm]); continue; }
    var an2 = layout.anchors.find(function (x) { return x.name === nm; });   // 빠진 위성은 되짚어 채운다
    var ex = sim.a * an2.mx - sim.b * an2.my + sim.tx, ey = sim.b * an2.mx + sim.a * an2.my + sim.ty;
    corners[nm] = back({ x: ex, y: ey }); corners[nm].inferred = true;
  }
  // ★★내삽 재적합 (2026-08-30 Fable 2차) — **이것이 잔차 1.12 → 0.1 을 만든다.**
  //   위 sim-H 는 코어(5.5모듈)에서 위성(60.5모듈)으로 **외삽**한 결과라
  //   코어 링 중심차 0.025px 가 131배로 자라 쌍극 오차를 만든다.
  //   여기서는 **원본 프레임에서** 코어 + 위성3 + 궤도 on-도트로 8DOF 를 다시 적합한다.
  //   궤도(8.5모듈)와 위성(60.5모듈) 사이가 되므로 데이터판이 **내삽 구간**에 들어온다.
  //   ★homographyLS 는 평탄 9배열을 바로 돌려준다 — mul3 합성·평탄화가 필요 없다.
  var Horb = null, residOrb = null;
  if (imgPts && imgPts.length >= 4 && layout.orbit && layout.orbit.length) {
    var oa = [], ob = [];
    for (var q1 = 0; q1 < A.length; q1++) { oa.push([A[q1][0], A[q1][1]]); ob.push([imgPts[q1][0], imgPts[q1][1]]); }
    var dots = 0;
    for (var d1 = 0; d1 < layout.orbit.length; d1++) {
      var od = layout.orbit[d1];
      if (!od.on) continue;                                  // 켜진 도트(암부)만 무게중심이 성립한다
      var pe = applyH(H, od.mx, od.my);
      var win = Math.max(1.5, 0.9 * cellPx);
      var rd2 = refineCentroid(grayObj, pe[0], pe[1], win);
      if (!rd2.ok) continue;
      if (Math.hypot(rd2.x - pe[0], rd2.y - pe[1]) > 0.7 * win) continue;   // 예측에서 너무 멀면 딴 것
      oa.push([od.mx, od.my]); ob.push([rd2.x, rd2.y]); dots++;
    }
    if (dots >= 6) {                                          // 도트가 적으면 신뢰하지 않는다
      var Hf = homographyLS(oa, ob);
      if (Hf) {
        var ro2 = 0;
        for (var q2 = 0; q2 < A.length; q2++) {               // 잔차는 코어+위성에서만(도트는 작아 편향)
          var pq = applyH(Hf, oa[q2][0], oa[q2][1]);
          ro2 += Math.hypot(pq[0] - ob[q2][0], pq[1] - ob[q2][1]);
        }
        ro2 /= A.length;
        if (ro2 <= 0.35 * cellPx) { Horb = Hf; residOrb = ro2; }
      }
    }
  }

  var Huse = Horb || H, residUse = (residOrb != null) ? residOrb : resid;
  var vf = verifyLayout(grayObj, Huse, layout);
  return { ok: true, method: Horb ? 'sim3orb' : 'sim3', core: back(coreR), corners: corners,
           northStar: 'TL', Hmod2img: Huse, residPx: +residUse.toFixed(2), dCen: 0,
           // ★orb-H 가 실패해도 sim-H 로는 읽히는 경우가 있다(블러가 쌍극을 뭉갠다) — 보조로 실어 보낸다.
           altH: Horb ? H : null,
           sats3: found.length, donutInferred: donutInferred,
           missing: names.filter(function (n) { return !labeled[n]; })[0] || null,
           modPx: vf.modPx, orbitMatch: vf.orbitMatch, orbitContrast: vf.orbitContrast };
}

/* ★refitWithOrbit — 궤도 24도트로 **내삽 재적합** (2026-08-30, 밀착 전용)
 *
 *   `locate` 의 4점 호모그래피는 코어(5.5)와 위성으로 만든 **외삽**이다.
 *   위성이 멀면(60.5) 문제가 없지만, **밀착하면 위성이 안으로 들어와** 외삽 배율이 커진다:
 *     foot/L 짧은 축 16모듈 → 격자 가장자리(64)까지 **4배 외삽**
 *   그러면 작은 각도 오차가 4배로 자라 데이터 표본이 셀을 벗어난다.
 *   실측: foot/L 을 **정확히 원본 크기(816px)** 로 주면 locate 는 resid 0.13 으로 성공하는데
 *   readCode 가 실패한다(815·817px 는 성공 — 딱 정수배에서만 표본이 셀 경계에 앉는다).
 *
 *   여기서는 궤도(8.5모듈) on-도트를 추가로 실측해 **코어+위성4+도트**로 8DOF 를 다시 적합한다.
 *   궤도가 안쪽에 있으니 데이터판이 **내삽 구간**에 들어와 오차가 안 자란다.
 *   `locateSim3` 안에 있던 같은 기법을 4위성 경로에서도 쓸 수 있게 꺼낸 것이다.
 *
 *   ★안전: 도트가 6개 미만이거나 재적합 잔차가 나빠지면 **원래 H 를 그대로 돌려준다.**
 *     즉 이 함수는 결과를 나쁘게 만들지 않는다. */
function refitWithOrbit(grayObj, res, layout, cellPx) {
  if (!res || !res.ok || !res.Hmod2img || !layout || !layout.orbit || !layout.orbit.length) return res;
  if (!res.core || !res.corners) return res;
  const H = res.Hmod2img;
  const oa = [], ob = [];
  const c = layout.coreCenter;
  oa.push([c.mx, c.my]); ob.push([res.core.x, res.core.y]);
  for (const a of layout.anchors) {
    if (a.name === 'core') continue;
    const p = res.corners[a.name];
    if (!p) continue;
    oa.push([a.mx, a.my]); ob.push([p.x !== undefined ? p.x : p[0], p.y !== undefined ? p.y : p[1]]);
  }
  if (oa.length < 4) return res;
  const nAnchor = oa.length;
  let dots = 0;
  for (const od of layout.orbit) {
    if (!od.on) continue;
    const pe = applyH(H, od.mx, od.my);
    const win = Math.max(1.5, 0.9 * cellPx);
    const rd = refineCentroid(grayObj, pe[0], pe[1], win);
    if (!rd.ok) continue;
    if (Math.hypot(rd.x - pe[0], rd.y - pe[1]) > 0.7 * win) continue;
    oa.push([od.mx, od.my]); ob.push([rd.x, rd.y]); dots++;
  }
  if (dots < 6) return res;
  const Hf = homographyLS(oa, ob);
  if (!Hf) return res;
  let ro = 0;
  for (let i = 0; i < nAnchor; i++) {
    const pq = applyH(Hf, oa[i][0], oa[i][1]);
    ro += Math.hypot(pq[0] - ob[i][0], pq[1] - ob[i][1]);
  }
  ro /= nAnchor;
  if (ro > 0.35 * cellPx) return res;
  const vf = verifyLayout(grayObj, Hf, layout);
  const out = {};
  for (const k in res) out[k] = res[k];
  out.Hmod2img = Hf; out.residPx = +ro.toFixed(2);
  out.altH = H;                       // ★원래 H 도 실어 보낸다 — 재적합이 나쁠 때 호출부가 되돌린다
  out.method = (res.method || 'locate') + 'orb';
  out.modPx = vf.modPx; out.orbitMatch = vf.orbitMatch; out.orbitContrast = vf.orbitContrast;
  return out;
}

/*
 * prepareConic — conic 정면화의 **layout 무관 부분**을 한 번만 한다 (2026-08-30 Q1-B)
 *   반환한 `prep` 을 여러 layout 이 공유하면 warp·재-FRST 를 6회 → 1회로 줄인다.
 *   실측 근거: 실패 프레임에서 detectAuto 가 locateRobust 를 최대 18회 부르고,
 *   그때마다 이 전부(링추출·conic적합·워프 약600²·FRST전체)를 다시 했다.
 *   layout 이 관여하는 건 캔버스 크기(dim)뿐이라 **가장 큰 dim 으로 한 번**이면 된다.
 *   opts.redetect 가 없으면 준비하지 않는다(호출부가 재검출을 못 하면 의미가 없다).
 */
function prepareConic(img, grayObj, peaksList, cellPx, maxDistMod, opts) {
  opts = opts || {};
  if (!opts.redetect) return null;
  const rProbe = 3.5 * cellPx;
  // ★씨앗 힌트(2026-08-31) — 호출자가 코어 좌표를 이미 알면 그걸 쓴다.
  //   원근(yaw)에서 코어의 5.5모듈 링이 타원이 되면 원형 FRST 투표가 흩어져
  //   pickCoreSeed 가 **위성을 코어로 고른다**(실측: 잘못 고른 씨앗의 코어 거리가
  //   그 실루엣의 밀착 핀 반경과 정확히 일치했다). 힌트가 없으면 기존 경로 그대로.
  const seed = (opts && opts.seed && isFinite(opts.seed.x) && isFinite(opts.seed.y))
             ? opts.seed : pickCoreSeed(grayObj, peaksList, rProbe);
  if (!seed) return null;
  const rings = CN.extractCoreRings(grayObj, seed.x, seed.y, 0.18 * Math.min(grayObj.w, grayObj.h), 180);
  const cOut = CN.fitConic(rings.outer), cIn = CN.fitConic(rings.inner);
  if (!cOut || !cIn) return null;
  const rec = CN.recoverFromConcentric(cOut.C, cIn.C);
  if (!rec.ok || rec.ringRatio < 2.6 || rec.ringRatio > 4.6) return null;
  const dim = 2 * Math.ceil(maxDistMod * cellPx + 4 * cellPx);
  const H = CN.rectifyHomography(rec.l, cOut.C, rec.center, 5.5 * cellPx, [dim / 2, dim / 2]);
  if (!H) return null;
  const rimg = warpImage(img, H, dim);
  if (!rimg) return null;
  const rd = opts.redetect(rimg);
  if (!rd || !rd.gray) return null;
  const Hinv = CN.inv3(H);
  if (!Hinv) return null;
  return { H: H, Hinv: Hinv, dim: dim, gray: rd.gray, peaks: rd.peaks, ringRatio: rec.ringRatio };
}

/*
 * locateRobustShared — 준비된 conic(prep)을 써서 한 layout 을 확정한다 (2026-08-30 Q1-B)
 *   prep 이 null 이면 **기존 locateRobust 를 그대로 부른다**(동작 동일, 안전판).
 *   prep 이 있으면 primary 실패 시 warp 없이 정면화 이미지에서 바로 locate 한다.
 */
function locateRobustShared(img, grayObj, peaksList, layout, opts, prep) {
  opts = opts || {};
  const cellPx = opts.cellPx || 6;
  const primary = locate(grayObj, peaksList, layout);
  const passPx = opts.passPx != null ? opts.passPx : 0.45 * cellPx;
  if (primary.ok && primary.residPx <= passPx) return Object.assign({ method: 'orbit' }, primary);
  if (!prep) return locateRobust(img, grayObj, peaksList, layout, opts);

  const rres = locate(prep.gray, prep.peaks, layout);
  // ★2026-08-30 Q4 — 위성 3개 경로(similarity 과결정). 정면화 프레임에서는 잔여 자유도가
  //   similarity 4개뿐이라 **위성 3개로도 과결정**이다(코어 포함 8식/4미지수).
  //   워프는 이미 prep 에 있으므로 추가 비용이 없다.
  // ★★게이트를 `!rres.ok` 로 잡았다가 **블러에서 한 번도 안 불렸다**(2026-08-30 실측).
  //   블러가 생기면 극대점이 늘어(4→11개) locate 가 **엉터리 4점으로 "성공"**한다 —
  //   ok:true 에 잔차만 14~15px 인 상태다. 그러면 위 조건이 거짓이라 sim3 를 건너뛴다.
  //   (어제 locate3 에서 똑같은 실수를 했다: "실패했을 때만"이 아니라 "잘 안 됐을 때"여야 한다.)
  //   → 4위성 결과가 **깨끗하지 않으면** 항상 시도하고, 잔차가 더 좋은 쪽을 쓴다.
  if (!rres.ok || rres.residPx > passPx) {
    const t3 = locateSim3(grayObj, peaksList, layout, prep, cellPx);
    if (t3.ok && (!rres.ok || t3.residPx < rres.residPx)) return t3;
  }
  if (!rres.ok) {
    return primary.ok ? Object.assign({ method: 'orbit' }, primary)
                      : Object.assign({ method: 'conic-fail', reason: 'rect-' + rres.reason }, { ok: false });
  }
  const back = (pt) => { const q = CN.matVec3(prep.Hinv, [pt.x, pt.y, 1]); return { x: q[0] / q[2], y: q[1] / q[2] }; };
  const cornersImg = { TL: back(rres.corners.TL), TR: back(rres.corners.TR),
                       BR: back(rres.corners.BR), BL: back(rres.corners.BL) };
  const coreImg = back(rres.core);
  const order = ['TL', 'TR', 'BR', 'BL'];
  const A = order.map(k => { const a = layout.anchors.find(x => x.name === k); return [a.mx, a.my]; });
  const B = order.map(k => [cornersImg[k].x, cornersImg[k].y]);
  A.push([layout.coreCenter.mx, layout.coreCenter.my]); B.push([coreImg.x, coreImg.y]);
  const Hfin = homographyLS(A, B);
  let resid = 0;
  if (Hfin) { for (let i = 0; i < A.length; i++) { const p = applyH(Hfin, A[i][0], A[i][1]); resid += Math.hypot(p[0] - B[i][0], p[1] - B[i][1]); } resid /= A.length; }
  const vf = Hfin ? verifyLayout(grayObj, Hfin, layout) : { modPx: 0, orbitMatch: 0.5, orbitContrast: 0 };
  return { ok: true, method: 'conic', core: coreImg, corners: cornersImg,
           northStar: rres.northStar, Hmod2img: Hfin, residPx: +resid.toFixed(2),
           conicCenterErr: null, ringRatio: +prep.ringRatio.toFixed(2),
           modPx: vf.modPx, orbitMatch: vf.orbitMatch, orbitContrast: vf.orbitContrast };
}

function locateRobust(img, grayObj, peaksList, layout, opts) {
  opts = opts || {};
  const cellPx = opts.cellPx || 6;
  const primary = locate(grayObj, peaksList, layout);
  const passPx = opts.passPx != null ? opts.passPx : 0.45 * cellPx;
  if (primary.ok && primary.residPx <= passPx) return Object.assign({ method: 'orbit' }, primary);

  // ── conic 폴백 ──
  // 코어 판별 probe 는 작게(코어 내부 링만): 크면 이웃 위성이 프로파일에 섞여
  //   코어의 다중반전 이점이 희석돼 위성을 코어로 오인한다.
  const rProbe = 3.5 * cellPx;
  const seed = pickCoreSeed(grayObj, peaksList, rProbe);
  if (!seed) return primary.ok ? Object.assign({ method: 'orbit' }, primary) : primary;

  const rings = CN.extractCoreRings(grayObj, seed.x, seed.y, 0.18 * Math.min(grayObj.w, grayObj.h), 180);
  const cOut = CN.fitConic(rings.outer), cIn = CN.fitConic(rings.inner);
  if (!cOut || !cIn) return primary;
  const rec = CN.recoverFromConcentric(cOut.C, cIn.C);
  // ringRatio 는 바깥/안쪽 반경비(코어 설계=5.5/1.5≈3.67). 크게 벗어나면 링 오추출 → 폴백 포기.
  if (!rec.ok || rec.ringRatio < 2.6 || rec.ringRatio > 4.6) return primary;

  const targetR = 5.5 * cellPx;
  // 정면화 캔버스 크기 = 코어→가장 먼 앵커 거리(사각·원형 모양 무관).
  const maxDist = Math.max.apply(null, layout.anchors.slice(1).map(a => Math.hypot(a.mx - layout.coreCenter.mx, a.my - layout.coreCenter.my))) * cellPx;
  const dim = 2 * Math.ceil(maxDist + 4 * cellPx);
  const H = CN.rectifyHomography(rec.l, cOut.C, rec.center, targetR, [dim / 2, dim / 2]);
  if (!H) return primary;
  const rimg = warpImage(img, H, dim);
  if (!rimg || !opts.redetect) return primary;
  const rd = opts.redetect(rimg);
  const rres = locate(rd.gray, rd.peaks, layout);
  if (!rres.ok) return primary.ok ? Object.assign({ method: 'orbit' }, primary) : Object.assign({ method: 'conic-fail', reason: 'rect-' + rres.reason }, rres.ok ? {} : { ok: false });

  // 정면좌표 앵커 → 원본 이미지 좌표(Hinv)
  const Hinv = CN.inv3(H);
  const back = (pt) => { const q = CN.matVec3(Hinv, [pt.x, pt.y, 1]); return { x: q[0] / q[2], y: q[1] / q[2] }; };
  const cornersImg = { TL: back(rres.corners.TL), TR: back(rres.corners.TR), BR: back(rres.corners.BR), BL: back(rres.corners.BL) };
  const coreImg = back(rres.core);

  // 원본 프레임 최종 호모그래피 + 재투영오차
  const order = ['TL', 'TR', 'BR', 'BL'];
  const A = order.map(k => { const a = layout.anchors.find(x => x.name === k); return [a.mx, a.my]; });
  const B = order.map(k => [cornersImg[k].x, cornersImg[k].y]);
  A.push([layout.coreCenter.mx, layout.coreCenter.my]); B.push([coreImg.x, coreImg.y]);
  const Hfin = homographyLS(A, B);
  let resid = 0; if (Hfin) { for (let i = 0; i < A.length; i++) { const [px, py] = applyH(Hfin, A[i][0], A[i][1]); resid += Math.hypot(px - B[i][0], py - B[i][1]); } resid /= A.length; }

  const vf = Hfin ? verifyLayout(grayObj, Hfin, layout) : { modPx: 0, orbitMatch: 0.5, orbitContrast: 0 };
  return { ok: true, method: 'conic', core: coreImg, corners: cornersImg,
           northStar: rres.northStar, Hmod2img: Hfin, residPx: +resid.toFixed(2),
           conicCenterErr: null, ringRatio: +rec.ringRatio.toFixed(2),
           modPx: vf.modPx, orbitMatch: vf.orbitMatch, orbitContrast: vf.orbitContrast };
}

module.exports = { locate, locateRobust, locateRobustShared, locateSim3, refitWithOrbit, prepareConic, fitSimilarity, homographyLS, pickCoreSeed, bullseyeFit, verifyLayout };

});

__def("rs", function(module, exports){
'use strict';
/*
 * ============================================================================
 *  WIA Code — 순수 Reed–Solomon (GF(256)) + CRC-16 + 블록 플랜
 * ============================================================================
 *  독립 구현(외부 엔진 의존 0). 표준 알고리즘(GF(256) 0x11d, α=2; 다항식은
 *  최고차 우선; Berlekamp–Massey + Chien + Forney). codec.js 인터페이스 제공:
 *  crc16·planBlocks·rsEncodeAll·rsDecodeAll·interleaveBytes·deinterleaveBytes.
 * ============================================================================
 */
const EXP = new Uint8Array(512), LOG = new Uint8Array(256);
(function () { let x = 1; for (let i = 0; i < 255; i++) { EXP[i] = x; LOG[x] = i; x <<= 1; if (x & 0x100) x ^= 0x11d; } for (let i = 255; i < 512; i++) EXP[i] = EXP[i - 255]; })();
function mul(a, b) { return (a === 0 || b === 0) ? 0 : EXP[LOG[a] + LOG[b]]; }
function div(a, b) { if (a === 0) return 0; return EXP[(LOG[a] + 255 - LOG[b]) % 255]; }
function inv(a) { return EXP[255 - LOG[a]]; }
function pw(a, n) { return a === 0 ? 0 : EXP[(LOG[a] * ((n % 255) + 255)) % 255]; }

// 다항식(최고차 우선)
function pScale(p, s) { const r = new Array(p.length); for (let i = 0; i < p.length; i++) r[i] = mul(p[i], s); return r; }
function pAdd(a, b) { const r = new Array(Math.max(a.length, b.length)).fill(0); for (let i = 0; i < a.length; i++) r[i + r.length - a.length] = a[i]; for (let i = 0; i < b.length; i++) r[i + r.length - b.length] ^= b[i]; return r; }
function pMul(a, b) { const r = new Array(a.length + b.length - 1).fill(0); for (let i = 0; i < a.length; i++) for (let j = 0; j < b.length; j++) r[i + j] ^= mul(a[i], b[j]); return r; }
function pEval(p, x) { let y = p[0]; for (let i = 1; i < p.length; i++) y = mul(y, x) ^ p[i]; return y; }

const GENC = {};
function genPoly(nsym) {
  if (GENC[nsym]) return GENC[nsym];
  let g = [1]; for (let i = 0; i < nsym; i++) g = pMul(g, [1, EXP[i]]);
  return (GENC[nsym] = g);
}
function encodeBlock(msg, nsym) {                        // → parity(nsym)
  const g = genPoly(nsym), out = new Uint8Array(msg.length + nsym); out.set(msg, 0);
  for (let i = 0; i < msg.length; i++) { const c = out[i]; if (c !== 0) for (let j = 1; j < g.length; j++) out[i + j] ^= mul(g[j], c); }
  return out.subarray(msg.length);
}
function pDiv(dividend, divisor) {                        // → 나머지(remainder)
  const out = dividend.slice();
  for (let i = 0; i < dividend.length - (divisor.length - 1); i++) {
    const c = out[i]; if (c !== 0) for (let j = 1; j < divisor.length; j++) if (divisor[j] !== 0) out[i + j] ^= mul(divisor[j], c);
  }
  return out.slice(-(divisor.length - 1));
}
// 신드롬: [0, S0, S1, …, S_{nsym-1}] (선행 0 = reedsolo 규약)
function syndromes(msg, nsym) { const s = [0]; for (let i = 0; i < nsym; i++) s.push(pEval(msg, EXP[i])); return s; }
function anyNZ(a) { for (const v of a) if (v) return true; return false; }
function errLocator(synd, nsym) {                          // Berlekamp–Massey (선행0 synd)
  let errLoc = [1], oldLoc = [1];
  const shift = synd.length - nsym;
  for (let i = 0; i < nsym; i++) {
    const K = i + shift;
    let delta = synd[K];
    for (let j = 1; j < errLoc.length; j++) delta ^= mul(errLoc[errLoc.length - 1 - j], synd[K - j]);
    oldLoc = oldLoc.concat([0]);
    if (delta !== 0) {
      if (oldLoc.length > errLoc.length) { const nl = pScale(oldLoc, delta); oldLoc = pScale(errLoc, inv(delta)); errLoc = nl; }
      errLoc = pAdd(errLoc, pScale(oldLoc, delta));
    }
  }
  while (errLoc.length && errLoc[0] === 0) errLoc.shift();
  return errLoc;
}
function findErrors(errLoc, n) {                           // Chien
  const nerr = errLoc.length - 1, pos = [];
  for (let i = 0; i < n; i++) if (pEval(errLoc, EXP[(255 - i) % 255]) === 0) pos.push(n - 1 - i);
  if (pos.length !== nerr) throw new Error('rs: too many errors');
  return pos;
}
function correctErrata(msg, synd, pos) {                   // Forney (reedsolo 충실)
  const coefPos = pos.map(p => msg.length - 1 - p);
  let eLoc = [1]; for (const i of coefPos) eLoc = pMul(eLoc, pAdd([1], [pw(2, i), 0]));
  const sRev = synd.slice().reverse();
  const errEvalRev = pDiv(pMul(sRev, eLoc), [1].concat(new Array(eLoc.length).fill(0)));
  const errEval = errEvalRev.slice().reverse();
  const X = coefPos.map(cp => pw(2, cp - 0));
  for (let i = 0; i < X.length; i++) {
    const Xi = X[i], Xinv = inv(Xi);
    let prime = 1;
    for (let j = 0; j < X.length; j++) if (j !== i) prime = mul(prime, 1 ^ mul(Xinv, X[j]));
    let y = pEval(errEval.slice().reverse(), Xinv);
    y = mul(Xi, y);
    if (prime === 0) throw new Error('rs: forney zero');
    msg[pos[i]] ^= div(y, prime);
  }
}
/* 소거 위치를 알 때 쓰는 다항식 — Π(1 − X·α^i). 신드롬을 이걸로 먼저 나눠
 * "이미 아는 오염"을 걷어내면, 남은 예산 전부를 **모르는 오류**에 쓸 수 있다. */
function erasureLocator(erasePos, n) {
  let e = [1];
  for (const p of erasePos) e = pMul(e, pAdd([1], [pw(2, n - 1 - p), 0]));
  return e;
}
/* 소거를 반영한 수정 신드롬(Forney syndromes). */
function forneySyndromes(synd, erasePos, n) {
  const fs = synd.slice().reverse();
  for (const p of erasePos) {
    const x = pw(2, n - 1 - p);
    for (let i = 0; i < fs.length - 1; i++) fs[i] = mul(fs[i], x) ^ fs[i + 1];
    fs.pop();
  }
  return fs;
}

/* ★2026-08-30 B단계 — 소거(erasure) 판정.
 *   `erasures` 는 **오염이 확실한 바이트 위치** 배열이다(없으면 옛 경로 그대로).
 *   RS 는 위치를 모르면 예산의 절반을 찾는 데 쓴다: 오류 nsym/2, 소거 nsym.
 *   위성 하나가 가려진 것을 `locateSim3` 가 이미 알므로, 그 주변을 소거로 넘기면
 *   같은 패리티로 **2배**를 복구한다(실측: 27 → 55 바이트/블록).
 *   상한: 2*errors + erasures ≤ nsym. */
function decodeBlock(cw, nsym, erasures) {
  const msg = new Uint8Array(cw), s = syndromes(msg, nsym);
  if (!anyNZ(s)) return { data: msg.subarray(0, msg.length - nsym), errors: 0 };

  const erasePos = [];
  if (erasures && erasures.length) {
    for (const p of erasures) if (p >= 0 && p < msg.length && erasePos.indexOf(p) < 0) erasePos.push(p);
    if (erasePos.length > nsym) throw new Error('rs: too many erasures');
  }
  if (!erasePos.length) {                       // ── 옛 경로(한 글자도 안 바뀐다) ──
    const eloc = errLocator(s, nsym), pos = findErrors(eloc, msg.length);
    correctErrata(msg, s, pos);
    if (anyNZ(syndromes(msg, nsym))) throw new Error('rs: uncorrectable');
    return { data: msg.subarray(0, msg.length - nsym), errors: pos.length };
  }

  // ── 소거 경로 ──
  const fs = forneySyndromes(s, erasePos, msg.length);
  const room = nsym - erasePos.length;          // 남은 예산으로 찾을 수 있는 오류 수 × 2
  let pos = erasePos.slice();
  if (room >= 2 && anyNZ(fs)) {
    const eloc = errLocator(fs.slice().reverse(), room);
    let extra = [];
    try { extra = findErrors(eloc, msg.length); } catch (e) { extra = []; }
    for (const p of extra) if (pos.indexOf(p) < 0) pos.push(p);
  }
  if (2 * (pos.length - erasePos.length) + erasePos.length > nsym) throw new Error('rs: uncorrectable');
  correctErrata(msg, s, pos);
  if (anyNZ(syndromes(msg, nsym))) throw new Error('rs: uncorrectable');
  return { data: msg.subarray(0, msg.length - nsym), errors: pos.length, erasures: erasePos.length };
}

// ── 블록 플랜 + 전체 + 인터리브 (codec 인터페이스) ──────────────────────────
const ECC = { '25%': 0.25, '30%': 0.30, '35%': 0.35, '50%': 0.50 };
function planBlocks(rawBytes, ratioStr) {
  const ratio = ECC[ratioStr]; if (ratio === undefined) throw new Error('ECC?' + ratioStr);
  const b = Math.max(1, Math.ceil(rawBytes / 255));
  const ceilN = Math.ceil(rawBytes / b), floorN = Math.floor(rawBytes / b), cc = rawBytes - floorN * b;
  const blocks = []; let totalK = 0, totalN = 0;
  for (let i = 0; i < b; i++) { const n = i < cc ? ceilN : floorN, nsym = Math.round(n * ratio), k = n - nsym; if (k <= 0) throw new Error('블록 과소'); blocks.push({ n, k, nsym }); totalK += k; totalN += n; }
  return { blocks, totalK, totalN };
}
function rsEncodeAll(data, plan) {
  const dp = [], pp = []; let off = 0;
  for (const b of plan.blocks) { const s = data.subarray(off, off + b.k); off += b.k; dp.push(s); pp.push(encodeBlock(s, b.nsym)); }
  const out = new Uint8Array(plan.totalN); let p = 0;
  for (const s of dp) { out.set(s, p); p += s.length; } for (const s of pp) { out.set(s, p); p += s.length; }
  return out;
}
function rsDecodeAll(cw, plan) {
  const dp = [], pp = []; let off = 0;
  for (const b of plan.blocks) { dp.push(cw.subarray(off, off + b.k)); off += b.k; }
  for (const b of plan.blocks) { pp.push(cw.subarray(off, off + b.nsym)); off += b.nsym; }
  const out = new Uint8Array(plan.totalK); let oo = 0, errs = 0;
  for (let i = 0; i < plan.blocks.length; i++) { const b = plan.blocks[i], blk = new Uint8Array(b.k + b.nsym); blk.set(dp[i], 0); blk.set(pp[i], b.k); const r = decodeBlock(blk, b.nsym); out.set(r.data, oo); oo += b.k; errs += r.errors; }
  return { data: out, errors: errs };
}
function ilOrder(plan) {
  const o = [], ds = [], ps = []; let off = 0;
  for (const b of plan.blocks) { ds.push(off); off += b.k; } for (const b of plan.blocks) { ps.push(off); off += b.nsym; }
  let mK = 0, mS = 0; for (const b of plan.blocks) { if (b.k > mK) mK = b.k; if (b.nsym > mS) mS = b.nsym; }
  for (let i = 0; i < mK; i++) for (let k = 0; k < plan.blocks.length; k++) if (i < plan.blocks[k].k) o.push(ds[k] + i);
  for (let i = 0; i < mS; i++) for (let k = 0; k < plan.blocks.length; k++) if (i < plan.blocks[k].nsym) o.push(ps[k] + i);
  return o;
}
function interleaveBytes(seq, plan) { const o = ilOrder(plan), out = new Uint8Array(seq.length); for (let j = 0; j < o.length; j++) out[j] = seq[o[j]]; return out; }
function deinterleaveBytes(raw, plan) { const o = ilOrder(plan), out = new Uint8Array(raw.length); for (let j = 0; j < o.length; j++) out[o[j]] = raw[j]; return out; }
function crc16(buf) { let c = 0xffff; for (let i = 0; i < buf.length; i++) { c ^= buf[i] << 8; for (let k = 0; k < 8; k++) c = (c & 0x8000) ? ((c << 1) ^ 0x1021) & 0xffff : (c << 1) & 0xffff; } return c & 0xffff; }

module.exports = { crc16, planBlocks, rsEncodeAll, rsDecodeAll, interleaveBytes, deinterleaveBytes, encodeBlock, decodeBlock, erasureLocator };

});

__def("codec", function(module, exports){
'use strict';
/*
 * ============================================================================
 *  WIA Code v2 — 오르빗 데이터 계층 (payload ↔ 데이터셀 비트)
 * ============================================================================
 *  설계 전제("코덱은 기하와 분리") 실행: v1 의 검증된 바이트 코덱(Reed-Solomon
 *  오류정정·인터리브·CRC)을 _internal 로 그대로 계승하고, 픽셀 배치만 오르빗
 *  레이아웃(geometry.dataCells 순서)으로 바꾼다. QR 파인더 목발 없이 데이터를
 *  싣고 읽는다.
 *
 *  프레임: [0x57 'W', 0x01 ver, lenHi, lenLo] + payload(UTF-8) + crc16(2)
 *          VER=2 는 payload 자리에 **세그먼트 비트열**(숫자·URL·한글 압축) — 아래 P3 절.
 *  이후 RS(ECC) + 인터리브 → 데이터셀 비트(MSB-first) → geometry.render(opts.bits)
 * ============================================================================
 */
const I = require('./rs.js');            // 순수 표준 RS/CRC (v1 엔진 의존 제거)
const GEO = require('./geometry.js');
const { applyH } = require('./degrade.js');

const ECC = '25%';                 // 오류정정 비율(카메라 비트오류 흡수)
const MAGIC = 0x57, VER = 0x01;

// 결정적 화이트닝(스크램블): RS가 systematic이라 zero-padding→대부분 0바이트→대부분 흰 셀.
//   그러면 다단계 분류의 퍼센타일(lo/hi)이 skew돼 검정/중간단계를 놓친다. XOR 마스크로
//   코드워드를 균등분포화 → 셀 레벨 균등 → 캘리브레이션 견고 + 버스트오류 분산. XOR=자기역.
function scramble(bytes) {
  let s = 0x9e3779b9 >>> 0;
  const out = new Uint8Array(bytes.length);
  for (let i = 0; i < bytes.length; i++) { s = (Math.imul(s, 1664525) + 1013904223) >>> 0; out[i] = bytes[i] ^ ((s >>> 16) & 0xff); }
  return out;
}

function toBytes(str) {
  if (typeof TextEncoder !== 'undefined') return new TextEncoder().encode(str);
  return Uint8Array.from(Buffer.from(str, 'utf8'));
}
function fromBytes(u8) {
  if (typeof TextDecoder !== 'undefined') return new TextDecoder().decode(u8);
  return Buffer.from(u8).toString('utf8');
}

// ── 프레임(헤더+길이+CRC) ─────────────────────────────────────────────────
function frameEncode(payload) {
  const len = payload.length;
  const body = new Uint8Array(4 + len);
  body[0] = MAGIC; body[1] = VER; body[2] = (len >>> 8) & 255; body[3] = len & 255;
  body.set(payload, 4);
  const crc = I.crc16(body) & 0xffff;
  const out = new Uint8Array(body.length + 2);
  out.set(body, 0); out[body.length] = (crc >>> 8) & 255; out[body.length + 1] = crc & 255;
  return out;
}
function frameDecode(bytes) {
  if (bytes.length < 6 || bytes[0] !== MAGIC) return null;
  const ver = bytes[1];
  if (ver !== VER && ver !== VER2) return null;      // 모르는 버전 → 깨끗이 실패(오염 아님)
  const len = (bytes[2] << 8) | bytes[3];
  if (4 + len + 2 > bytes.length) return null;
  const body = bytes.subarray(0, 4 + len);
  const crc = I.crc16(body) & 0xffff;
  const got = (bytes[4 + len] << 8) | bytes[4 + len + 1];
  if (crc !== got) return null;
  if (ver === VER) return body.subarray(4, 4 + len);
  // VER=2 — 세그먼트 비트열을 풀어 UTF-8 바이트로 돌려준다(호출부 인터페이스 무변경).
  const text = segDecode(body.subarray(4, 4 + len));
  if (text === null) return null;
  return toBytes(text);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  세그먼트 모드 코덱 (프레임 VER=2) — P3 / MASTER LINE F7
 * ══════════════════════════════════════════════════════════════════════════
 *  VER=1 은 payload 를 UTF-8 바이트열로만 싣는다. 숫자·URL·한글은 그 표현이
 *  낭비다(숫자 1자리 8비트, 한글 1음절 24비트). VER=2 는 Han Xin/QR 식으로
 *  **내용물 종류별 비트폭**을 쓰고, 어떤 종류로 자를지는 DP 로 최적 분할한다.
 *
 *  프레임: [0x57 'W', 0x02 ver, lenHi, lenLo] + 세그먼트비트열(len 바이트) + crc16(2)
 *          — 껍데기는 VER=1 과 완전히 같다(RS·인터리브·산포·CRC 무변경).
 *
 *  세그먼트 비트열(MSB-first):
 *      flags   2b   예약(0 고정). ★P8 마스크 시드 2비트의 자리 — 지금 디코더는 무시한다.
 *      [ mode 3b + count 12b + body ] * n
 *      mode 0(END) 로 끝내고 바이트 경계까지 0 으로 채운다.
 *
 *  모드:
 *    0 END   종결
 *    1 N     숫자 '0'-'9'          3자리→10b, 나머지 1자리→4b / 2자리→7b
 *    2 A     소문자 URL 45자표      2자→11b, 홀수 끝 1자→6b
 *    3 H     한글음절 11172 + ASCII 128 = 11300  2자→27b(13.5b/자), 홀수 끝→14b
 *    4 B     UTF-8 바이트          8b/바이트 (count 는 **바이트 수**)
 *    5 C7    ASCII 0x00-0x7F       7b/자
 *    6 S64   base64url 64자표      6b/자
 *    7 —     예약(만나면 해독 실패)
 *
 *  ★설계가 원안(PROPOSALS_FABLE §P3)에서 벗어난 곳은 이 파일 하단 주석과
 *    `test-codec-seg.js` 의 측정표에 근거와 함께 적혀 있다. 요약:
 *    ① count 는 전 모드 **12비트 통일**(원안의 A:11b·H:10b 는 최대용량에서 넘친다)
 *    ② N 은 count 가 있으므로 Han Xin 종결자를 쓰지 않는다(중복) — QR 식 4/7b 잔여
 *    ③ H 는 14b 단일 대신 **27b/2자(13.5b)** + 알파벳에 ASCII 포함
 *       (ASCII 를 빼면 띄어쓰기마다 모드가 끊겨 한글 산문에서 이득이 1.05배로 죽는다)
 *    ④ C7·S64 는 원안에 없던 추가 모드(자격증 페이로드 1.05→1.19배)
 * ══════════════════════════════════════════════════════════════════════════ */
const VER2 = 0x02;
const SEG_END = 0, SEG_N = 1, SEG_A = 2, SEG_H = 3, SEG_B = 4, SEG_C7 = 5, SEG_S64 = 6;
const SEG_MODE_BITS = 3, SEG_CNT_BITS = 12, SEG_MAX_COUNT = (1 << SEG_CNT_BITS) - 1;
const SEG_HDR_BITS = SEG_MODE_BITS + SEG_CNT_BITS;          // 15
const SEG_FLAG_BITS = 2;
// A 표 45자 = a-z(26) + 0-9(10) + 구두점 9. 45² = 2025 ≤ 2048 이라 2자가 11비트에 든다.
//   ★구두점 9칸의 선정은 실측(test-codec-seg.js §A): URL 필수 8개(- . _ : / ? & =) +
//     9번째는 **공백**. 공백은 URL 에 안 나오므로 URL 이득을 1도 깎지 않으면서
//     소문자 산문을 0.99배(손해)에서 1.44배로 바꾼다. 탈락: ~ # % (우리 페이로드 빈도 낮음).
const A_TABLE = 'abcdefghijklmnopqrstuvwxyz0123456789-._:/?&= ';
const S64_TABLE = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_';
const HAN_BASE = 0xAC00, HAN_CNT = 11172, H_ASCII = 128, H_CNT = HAN_CNT + H_ASCII;  // 11300
// 11300² = 127,689,999 < 2²⁷ — 두 자가 27비트에 든다(13.5b/자). 14b 단일보다 3.6% 낫다.
const _A_IDX = (() => { const m = new Map(); for (let i = 0; i < A_TABLE.length; i++) m.set(A_TABLE.codePointAt(i), i); return m; })();
const _S_IDX = (() => { const m = new Map(); for (let i = 0; i < S64_TABLE.length; i++) m.set(S64_TABLE.codePointAt(i), i); return m; })();

function _bitWriter() {
  const out = []; let acc = 0, n = 0;
  return {
    put(v, nb) { for (let i = nb - 1; i >= 0; i--) { acc = ((acc << 1) | ((v >>> i) & 1)) & 255; if (++n === 8) { out.push(acc); acc = 0; n = 0; } } },
    bits() { return out.length * 8 + n; },
    finish() { if (n) { out.push((acc << (8 - n)) & 255); acc = 0; n = 0; } return Uint8Array.from(out); },
  };
}
function _bitReader(u8) {
  let p = 0; const total = u8.length * 8;
  return {
    left() { return total - p; },
    get(nb) { let v = 0; for (let i = 0; i < nb; i++) { v = v * 2 + ((u8[p >> 3] >>> (7 - (p & 7))) & 1); p++; } return v; },
    restZero() { while (p < total) { if ((u8[p >> 3] >>> (7 - (p & 7))) & 1) return false; p++; } return true; },
  };
}
function _u8len(cp) { return cp < 0x80 ? 1 : cp < 0x800 ? 2 : cp < 0x10000 ? 3 : 4; }
function _hIdx(cp) { return (cp >= HAN_BASE && cp < HAN_BASE + HAN_CNT) ? cp - HAN_BASE : (cp < 0x80 ? HAN_CNT + cp : -1); }
function _hCp(ix) { return ix < HAN_CNT ? HAN_BASE + ix : ix - HAN_CNT; }

// 모드 서술자. six = 문자당 비용(1/6비트 단위) — 3·2 자 묶음의 최소공배수가 6이라 정수화된다.
const SEG_MODES = [
  { id: SEG_N,   six: 20,   ok: cp => cp >= 48 && cp <= 57 },                       // 10b/3자
  { id: SEG_A,   six: 33,   ok: cp => _A_IDX.has(cp) },                             // 11b/2자
  { id: SEG_H,   six: 81,   ok: cp => _hIdx(cp) >= 0 },                             // 27b/2자
  { id: SEG_S64, six: 36,   ok: cp => _S_IDX.has(cp) },                             // 6b/자
  { id: SEG_C7,  six: 42,   ok: cp => cp < 0x80 },                                  // 7b/자
  { id: SEG_B,   six: null, ok: () => true },                                       // 8b/바이트
];
const _SEG_HDR_SIX = SEG_HDR_BITS * 6;

/* 문자열 → 세그먼트 목록. Han Xin `hx_define_modes` 와 같은 DP(비용 1/6비트).
 *   묶음 경계(3자·2자)의 반올림은 무시한다 — QR/Han Xin 도 같다. 오차 상한은
 *   세그먼트당 N 6b · A 5b · H 13b 이고, 그 대가로 O(L·6) 로 끝난다. */
function segPlan(text) {
  const cp = [];
  for (const ch of text) cp.push(ch.codePointAt(0));
  const L = cp.length;
  if (!L) return [];
  const K = SEG_MODES.length, INF = Infinity;
  const w = (m, c) => m.six === null ? 48 * _u8len(c) : m.six;
  let prev = new Array(K).fill(_SEG_HDR_SIX);
  const from = new Int8Array(L * K);
  for (let i = 0; i < L; i++) {
    let bi = -1, bv = INF;
    for (let k = 0; k < K; k++) if (prev[k] < bv) { bv = prev[k]; bi = k; }
    const cur = new Array(K).fill(INF);
    for (let k = 0; k < K; k++) {
      const M = SEG_MODES[k];
      if (!M.ok(cp[i])) continue;
      const stay = prev[k], sw = bv + _SEG_HDR_SIX;
      if (stay <= sw) { cur[k] = stay + w(M, cp[i]); from[i * K + k] = k; }
      else { cur[k] = sw + w(M, cp[i]); from[i * K + k] = bi; }
    }
    prev = cur;
  }
  let bi = -1, bv = INF;
  for (let k = 0; k < K; k++) if (prev[k] < bv) { bv = prev[k]; bi = k; }
  const pick = new Int8Array(L);
  for (let i = L - 1; i >= 0; i--) { pick[i] = bi; bi = from[i * K + bi]; }
  // 같은 모드 연속을 하나의 세그먼트로 묶고, count 상한(4095)에서 자른다.
  const segs = [];
  let s = 0;
  for (let i = 1; i <= L; i++) {
    if (i === L || pick[i] !== pick[s]) { segs.push({ mode: SEG_MODES[pick[s]].id, from: s, to: i, cps: cp.slice(s, i) }); s = i; }
  }
  const out = [];
  for (const g of segs) {
    if (g.mode === SEG_B) {                       // count 가 바이트 수라 바이트로 자른다
      let acc = 0, st = 0;
      for (let i = 0; i < g.cps.length; i++) {
        const n = _u8len(g.cps[i]);
        if (acc + n > SEG_MAX_COUNT) { out.push({ mode: SEG_B, cps: g.cps.slice(st, i) }); st = i; acc = 0; }
        acc += n;
      }
      out.push({ mode: SEG_B, cps: g.cps.slice(st) });
    } else {
      const step = g.mode === SEG_N ? 4095 - (4095 % 3) : SEG_MAX_COUNT - (SEG_MAX_COUNT % 2);  // 묶음 경계에서 자른다
      for (let i = 0; i < g.cps.length; i += step) out.push({ mode: g.mode, cps: g.cps.slice(i, i + step) });
    }
  }
  return out;
}

/* 세그먼트 목록 → 비트열 바이트. */
function segEncode(text) {
  const segs = segPlan(text), bw = _bitWriter();
  bw.put(0, SEG_FLAG_BITS);                       // flags 예약 = 0
  for (const g of segs) {
    const c = g.cps, n = c.length;
    if (g.mode === SEG_B) {
      const by = toBytes(String.fromCodePoint.apply(String, c));
      bw.put(SEG_B, SEG_MODE_BITS); bw.put(by.length, SEG_CNT_BITS);
      for (let i = 0; i < by.length; i++) bw.put(by[i], 8);
      continue;
    }
    bw.put(g.mode, SEG_MODE_BITS); bw.put(n, SEG_CNT_BITS);
    if (g.mode === SEG_N) {
      let i = 0;
      for (; i + 3 <= n; i += 3) bw.put((c[i] - 48) * 100 + (c[i + 1] - 48) * 10 + (c[i + 2] - 48), 10);
      const r = n - i;
      if (r === 1) bw.put(c[i] - 48, 4);
      else if (r === 2) bw.put((c[i] - 48) * 10 + (c[i + 1] - 48), 7);
    } else if (g.mode === SEG_A) {
      let i = 0;
      for (; i + 2 <= n; i += 2) bw.put(_A_IDX.get(c[i]) * 45 + _A_IDX.get(c[i + 1]), 11);
      if (i < n) bw.put(_A_IDX.get(c[i]), 6);
    } else if (g.mode === SEG_H) {
      let i = 0;
      for (; i + 2 <= n; i += 2) bw.put(_hIdx(c[i]) * H_CNT + _hIdx(c[i + 1]), 27);
      if (i < n) bw.put(_hIdx(c[i]), 14);
    } else if (g.mode === SEG_C7) {
      for (let i = 0; i < n; i++) bw.put(c[i], 7);
    } else if (g.mode === SEG_S64) {
      for (let i = 0; i < n; i++) bw.put(_S_IDX.get(c[i]), 6);
    } else throw new Error('세그먼트 모드 미상: ' + g.mode);
  }
  bw.put(SEG_END, SEG_MODE_BITS);
  return bw.finish();
}

/* 비트열 바이트 → 문자열. 규격 위반이면 **null**(엉뚱한 문자열을 내지 않는다). */
function segDecode(u8) {
  const br = _bitReader(u8);
  if (br.left() < SEG_FLAG_BITS) return null;
  br.get(SEG_FLAG_BITS);                          // flags — 예약, 지금은 무시(전방호환)
  const parts = [];
  for (;;) {
    if (br.left() < SEG_MODE_BITS) return br.restZero() ? parts.join('') : null;
    const mode = br.get(SEG_MODE_BITS);
    if (mode === SEG_END) return br.restZero() ? parts.join('') : null;
    if (mode > SEG_S64) return null;              // 예약 모드 → 깨끗이 실패
    if (br.left() < SEG_CNT_BITS) return null;
    const n = br.get(SEG_CNT_BITS);
    let need;
    if (mode === SEG_N) { const f = Math.floor(n / 3), r = n % 3; need = f * 10 + (r === 1 ? 4 : r === 2 ? 7 : 0); }
    else if (mode === SEG_A) need = (n >> 1) * 11 + (n & 1 ? 6 : 0);
    else if (mode === SEG_H) need = (n >> 1) * 27 + (n & 1 ? 14 : 0);
    else if (mode === SEG_B) need = n * 8;
    else if (mode === SEG_C7) need = n * 7;
    else need = n * 6;
    if (br.left() < need) return null;
    if (mode === SEG_N) {
      let i = 0, s = '';
      for (; i + 3 <= n; i += 3) { const v = br.get(10); if (v > 999) return null; s += String(v).padStart(3, '0'); }
      const r = n - i;
      if (r === 1) { const v = br.get(4); if (v > 9) return null; s += String(v); }
      else if (r === 2) { const v = br.get(7); if (v > 99) return null; s += String(v).padStart(2, '0'); }
      parts.push(s);
    } else if (mode === SEG_A) {
      let i = 0, s = '';
      for (; i + 2 <= n; i += 2) { const v = br.get(11); if (v >= 45 * 45) return null; s += A_TABLE[(v / 45) | 0] + A_TABLE[v % 45]; }
      if (i < n) { const v = br.get(6); if (v >= 45) return null; s += A_TABLE[v]; }
      parts.push(s);
    } else if (mode === SEG_H) {
      let i = 0, s = '';
      for (; i + 2 <= n; i += 2) { const v = br.get(27); if (v >= H_CNT * H_CNT) return null; s += String.fromCodePoint(_hCp(Math.floor(v / H_CNT)), _hCp(v % H_CNT)); }
      if (i < n) { const v = br.get(14); if (v >= H_CNT) return null; s += String.fromCodePoint(_hCp(v)); }
      parts.push(s);
    } else if (mode === SEG_B) {
      const by = new Uint8Array(n);
      for (let i = 0; i < n; i++) by[i] = br.get(8);
      parts.push(fromBytes(by));
    } else if (mode === SEG_C7) {
      let s = '';
      for (let i = 0; i < n; i++) s += String.fromCharCode(br.get(7));
      parts.push(s);
    } else {
      let s = '';
      for (let i = 0; i < n; i++) s += S64_TABLE[br.get(6)];
      parts.push(s);
    }
  }
}

/* 세그먼트 프레임(VER=2). 껍데기·CRC 는 VER=1 과 동일. */
function frameEncodeSeg(text) {
  const seg = segEncode(text);
  if (seg.length > 0xffff) throw new Error('세그먼트 길이초과: ' + seg.length + 'B');
  const body = new Uint8Array(4 + seg.length);
  body[0] = MAGIC; body[1] = VER2; body[2] = (seg.length >>> 8) & 255; body[3] = seg.length & 255;
  body.set(seg, 4);
  const crc = I.crc16(body) & 0xffff;
  const out = new Uint8Array(body.length + 2);
  out.set(body, 0); out[body.length] = (crc >>> 8) & 255; out[body.length + 1] = crc & 255;
  return out;
}

/* 인코더가 쓸 프레임을 고른다.
 *   ver=1(기본)  : 현행 그대로 — **옛 스캐너가 읽는다**
 *   ver=2        : 무조건 세그먼트
 *   ver='auto'   : 더 짧은 쪽(동률이면 1) — 바이트 콘텐츠는 자동으로 VER=1 이라 불변
 *   ★기본값을 'auto' 로 넘기는 것이 곧 "표준 채택" 이다. 스캐너 배포가 끝난 뒤
 *     한 줄로 바꾼다(그전에 바꾸면 옛 스캐너가 숫자·한글 코드를 못 읽는다). */
function pickFrame(text, opts) {
  // ★2026-09-05 P3 ON(오너 결정): 기본 'auto' — v2 가 더 짧을 때만 v2, 아니면 v1 바이트동일.
  //   v1 로 고정하려면 opts.ver=1. 스캐너·킷 엔진·API 는 같은 codec 을 쓰므로 함께 바뀐다.
  const ver = (opts && opts.ver) || 'auto';
  if (ver === 2) return frameEncodeSeg(text);
  if (ver !== 'auto') return frameEncode(toBytes(text));
  const f1 = frameEncode(toBytes(text));
  let f2 = null;
  try { f2 = frameEncodeSeg(text); } catch (e) { f2 = null; }
  return (f2 && f2.length < f1.length) ? f2 : f1;
}

// nCells(데이터셀 수) → RS 계획(인코드/디코드 공용, 결정적)
function planFor(nCells) { return I.planBlocks(Math.floor(nCells / 8), ECC); }

// payload 문자열 → 데이터셀 비트배열(길이 nCells). capacity 초과 시 예외.
function encodeToBits(text, nCells, opts) {
  const frame = pickFrame(text, opts);
  const plan = planFor(nCells);
  if (frame.length > plan.totalK) throw new Error('용량초과: ' + frame.length + 'B > ' + plan.totalK + 'B');
  const dataIn = new Uint8Array(plan.totalK); dataIn.set(frame, 0);   // 나머지 0패딩
  let cw = I.rsEncodeAll(dataIn, plan);
  cw = scramble(I.interleaveBytes(cw, plan));
  const bits = new Uint8Array(nCells);
  const nb = Math.min(cw.length * 8, nCells), pos = buildPerm(nCells);
  for (let k = 0; k < nb; k++) bits[pos[k]] = (cw[k >> 3] >>> (7 - (k & 7))) & 1;   // 위치 산포
  return { bits: bits, plan: plan, usedBytes: frame.length, capBytes: plan.totalK, ver: frame[1] };
}

/* ★2026-08-30 B단계 — 오염이 **확실한 셀**들을 RS 소거 위치로 옮긴다.
 *   RS 는 위치를 모르면 예산의 절반을 찾는 데 쓴다(오류 nsym/2 vs 소거 nsym).
 *   위성 하나가 가려진 것을 locateSim3 가 이미 알므로, 그 주변 셀을 여기로 넘기면
 *   같은 패리티로 **2배**를 복구한다(실측: 블록당 27 → 55 바이트).
 *   ★buildPerm 이 인접 셀을 비트스트림에서 흩어 놓으므로 **그 산포를 그대로 따라가야** 한다 —
 *     셀 번호를 바이트 번호로 그냥 쓰면 엉뚱한 자리를 소거로 표시한다.
 *   반환: 블록 인덱스별 소거 바이트 위치 배열. */
function cellsToErasures(badCells, nCells, plan, bpc) {
  bpc = bpc || 1;
  if (!badCells || !badCells.length) return null;
  const bad = new Uint8Array(nCells);
  for (const c of badCells) if (c >= 0 && c < nCells) bad[c] = 1;
  const pos = buildPerm(nCells);
  const totalN = plan.totalN, nb = totalN * 8;
  // 셀 → 비트 → 코드워드 바이트. **decodeFromCells 와 완전히 같은 순서**로 돈다
  //   (bpc 가 1이 아니면 셀 하나가 여러 비트를 낸다).
  const cwBad = new Uint8Array(totalN);
  let bit = 0;
  for (let k = 0; k < nCells && bit < nb; k++) {
    const dirty = bad[pos[k]];
    for (let b = bpc - 1; b >= 0 && bit < nb; b--) { if (dirty) cwBad[bit >> 3] = 1; bit++; }
  }
  // scramble 은 값만 XOR 하므로 위치가 안 바뀐다. deinterleave 는 위치를 바꾼다.
  const seqBad = I.deinterleaveBytes(cwBad, plan);
  // 블록별로 나눈다 — rsDecodeAll 과 **같은 순서**(데이터 전부 → 패리티 전부)
  const out = [];
  let off = 0;
  const dOff = [], pOff = [];
  for (const b of plan.blocks) { dOff.push(off); off += b.k; }
  for (const b of plan.blocks) { pOff.push(off); off += b.nsym; }
  for (let i = 0; i < plan.blocks.length; i++) {
    const b = plan.blocks[i], e = [];
    for (let j = 0; j < b.k; j++) if (seqBad[dOff[i] + j]) e.push(j);
    for (let j = 0; j < b.nsym; j++) if (seqBad[pOff[i] + j]) e.push(b.k + j);
    out.push(e);
  }
  return out;
}

// 데이터셀 비트배열 → payload 문자열(오류정정 포함). 실패 시 {ok:false}.
function decodeFromBits(bits, nCells) {
  const plan = planFor(nCells);
  const totalN = plan.totalN;
  const cw = new Uint8Array(totalN);
  const nb = Math.min(totalN * 8, bits.length), pos = buildPerm(bits.length);
  for (let k = 0; k < nb; k++) if (bits[pos[k]]) cw[k >> 3] |= (1 << (7 - (k & 7)));   // 위치 산포 역
  const seq = I.deinterleaveBytes(scramble(cw), plan);
  let res;
  try { res = I.rsDecodeAll(seq, plan); }
  catch (e) { return { ok: false, reason: 'rs-uncorrectable' }; }
  const payload = frameDecode(res.data);
  if (!payload) return { ok: false, reason: 'bad-frame/crc', errors: res.errors };
  return { ok: true, text: fromBytes(payload), errors: res.errors };
}

// ── 이미지에서 데이터셀 비트 읽기 (locate 의 Hmod2img 사용) ─────────────────
//   각 데이터셀 중심(모듈) → 이미지(H) → 작은 원판 평균 그레이 → Otsu 임계로 0/1.
function grayAt(G, x, y) {
  const g = G.g, w = G.w, h = G.h;
  if (x < 0 || y < 0 || x >= w - 1 || y >= h - 1) return 255;
  const x0 = Math.floor(x), y0 = Math.floor(y), fx = x - x0, fy = y - y0, i = y0 * w + x0;
  return (g[i] * (1 - fx) + g[i + 1] * fx) * (1 - fy) + (g[i + w] * (1 - fx) + g[i + w + 1] * fx) * fy;
}
function sampleCellGrays(grayObj, H, layout, cellPx) {
  const dc = GEO.dataCells(layout);
  const grays = new Float32Array(dc.length);
  // 셀 내부 작은 원판(반경 ~0.3모듈) 다중샘플 평균 — 블러/저해상도 강건.
  const offs = [[0, 0], [0.22, 0], [-0.22, 0], [0, 0.22], [0, -0.22]];
  for (let i = 0; i < dc.length; i++) {
    const mx = dc[i][0] + 0.5, my = dc[i][1] + 0.5;
    let s = 0, n = 0;
    for (const o of offs) { const p = applyH(H, mx + o[0], my + o[1]); s += grayAt(grayObj, p[0], p[1]); n++; }
    grays[i] = s / n;
  }
  return { grays: grays, cells: dc };
}
// Otsu 임계 (0~255 히스토그램)
function otsu(grays) {
  const hist = new Float64Array(256);
  for (const v of grays) hist[Math.max(0, Math.min(255, Math.round(v)))]++;
  const total = grays.length; let sum = 0; for (let t = 0; t < 256; t++) sum += t * hist[t];
  let sumB = 0, wB = 0, maxVar = -1, tLo = 128, tHi = 128;
  for (let t = 0; t < 256; t++) {
    wB += hist[t]; if (wB === 0) continue; const wF = total - wB; if (wF === 0) break;
    sumB += t * hist[t];
    const mB = sumB / wB, mF = (sum - sumB) / wF, v = wB * wF * (mB - mF) * (mB - mF);
    // 완전 양극단(깨끗한 렌더: 그레이 0/255 만, 중간 빈 다수)에선 분산이 넓은 평탄부에서
    //   동일 최대 → 그 평탄부의 '중앙'을 임계로. maxVar 갱신=평탄부 시작, 동률=끝 연장.
    if (v > maxVar * (1 + 1e-9)) { maxVar = v; tLo = t; tHi = t; }
    else if (v >= maxVar * (1 - 1e-9)) { tHi = t; }
  }
  return Math.round((tLo + tHi) / 2);
}

// ── 다단계(그레이) 데이터 셀 (Fable5 설계) ─────────────────────────────────
//   셀당 bpc 비트 = 2^bpc 그레이 단계. QR(흑백 1비트)을 용량에서 넘는 구조적 레버.
//   ·팔레트: 밝기 rank 0=흰(255) … max=검(0), 등간격. levelGray(rank).
//   ·그레이코드: 인접 밝기단계 오독 = 1비트 오류(RS 정정) → symbol=grayEnc, 역=grayDec.
//   ·캘리브레이션: 코드의 알려진 앵커(코어·위성=검, 도넛구멍·흰링·콰이엇=흰)로 매프레임
//     흑점/백점 역산 → 조명 무관. ·모드판별: RS/CRC 가 통과하는 bpc 를 trial-decode.
const PAL_ECC = { 1: '25%', 2: '35%', 3: '50%' };     // 단계 많을수록 노이즈↑ → ECC 상향(원근 far쪽 강건)
// ★bpc2 는 2026-08-16 에 50%→35% (다리QR 존 예약으로 예산 낭비가 사라진 뒤). 순용량 +30~31%.
//   열화 스윕 실측(하트/사각/원형, 다리 on·off): 노이즈·블러 파단점은 50%와 완전 동일 —
//   그 구간의 실패는 예비분 고갈이 아니라 흑백 대비붕괴라 ECC 를 올려도 못 살린다. 유일한 손실은
//   극단 원근(50°→48°, 2°). 반대로 50% 는 정정능력 절반을 놀리고 용량만 30% 깎고 있었다.
// 구형 호환: 그 이전 bpc2 코드는 50% 로 인코딩됐다 → 디코더는 두 비율을 모두 시도한다.
//   ★"먼저 성공한 쪽"이 아니라 "errors 가 적은 쪽"을 채택해야 한다. payload 가 짧으면 셀 대부분이
//   패딩이라 틀린 비율로도 RS 가 억지로 복원해내는데(실측 76 errors 소모), 그대로 채택하면
//   정정예산을 태운 채 성공한 것이라 실제 노이즈가 얹히면 그때 터진다.
const ECC_TRY = { 1: ['25%'], 2: ['35%', '50%'], 3: ['50%'] };
function nlevOf(bpc) { return 1 << bpc; }
function gcd(a, b) { while (b) { const t = a % b; a = b; b = t; } return a; }
// 셀 위치 산포 permutation. 바이트=인접 4셀이라 원근 far쪽 오류가 바이트로 뭉쳐(5%셀→20%바이트)
//   ECC 초과. 공간 인접 셀을 비트스트림서 멀리 흩으면 각 바이트의 셀이 서로 다른 구역 → 바이트
//   오류≈셀오류. pos[k]=k번째 데이터유닛의 공간위치(큰 coprime 승수 = 결정적 선형 permutation).
const _permCache = {};
function buildPerm(n) {
  if (_permCache[n]) return _permCache[n];
  let A = Math.floor(n * 0.6180339887) | 1;      // 황금비 근처 홀수
  while (gcd(A, n) !== 1) A += 2;
  const pos = new Int32Array(n);
  for (let k = 0; k < n; k++) pos[k] = (k * A) % n;
  return (_permCache[n] = pos);
}
function levelGray(rank, nlev) { return Math.round(255 * (1 - rank / (nlev - 1))); } // rank0=흰..max=검
function grayEnc(v) { return v ^ (v >> 1); }          // symbol → 밝기 rank (binary→Gray)
function grayDec(r) { let v = 0; for (; r > 0; r >>= 1) v ^= r; return v; }

// payload → 데이터셀 심볼(0..2^bpc-1) + 렌더용 그레이(cellGray). bpc=1 은 기존 흑백과 동일.
function encodeToCells(text, nCells, bpc, ecc, opts) {
  bpc = bpc || 1;
  const frame = pickFrame(text, opts);
  const rawBytes = Math.floor(nCells * bpc / 8);
  const plan = I.planBlocks(rawBytes, ecc || PAL_ECC[bpc] || '25%');
  if (frame.length > plan.totalK) throw new Error('용량초과: ' + frame.length + 'B > ' + plan.totalK + 'B');
  const dataIn = new Uint8Array(plan.totalK); dataIn.set(frame, 0);
  const cw = scramble(I.interleaveBytes(I.rsEncodeAll(dataIn, plan), plan));
  const totalBits = cw.length * 8, getBit = (i) => i < totalBits ? (cw[i >> 3] >>> (7 - (i & 7))) & 1 : 0;
  const nlev = nlevOf(bpc), symbols = new Uint8Array(nCells), cellGray = new Uint8ClampedArray(nCells);
  const pos = buildPerm(nCells);                 // k번째 데이터유닛 → 산포된 공간위치 pos[k]
  let bit = 0;
  for (let k = 0; k < nCells; k++) {
    let v = 0; for (let b = 0; b < bpc; b++) v = (v << 1) | getBit(bit++);
    const p = pos[k]; symbols[p] = v; cellGray[p] = levelGray(grayEnc(v), nlev);
  }
  return { cells: symbols, cellGray: cellGray, plan: plan, usedBytes: frame.length, capBytes: plan.totalK, bitsPerCell: bpc, ver: frame[1] };
}

// 원시 바이트 왕복(텍스트 아님) — encodeToCells/decodeFromCells과 완전히 동일한 파이프라인이지만
// toBytes(text)/fromBytes(payload)의 UTF-8 변환을 건너뛴다(임의 바이너리는 유효한 UTF-8이 아닐 수
// 있어 TextEncoder/Decoder를 거치면 손실·팽창됨 — 이미지 등 순수 바이트 페이로드 데모용, 2026-08-10 추가).
function encodeBytesToCells(bytes, nCells, bpc, ecc) {
  bpc = bpc || 1;
  const frame = frameEncode(bytes);
  const rawBytes = Math.floor(nCells * bpc / 8);
  const plan = I.planBlocks(rawBytes, ecc || PAL_ECC[bpc] || '25%');
  if (frame.length > plan.totalK) throw new Error('용량초과: ' + frame.length + 'B > ' + plan.totalK + 'B');
  const dataIn = new Uint8Array(plan.totalK); dataIn.set(frame, 0);
  const cw = scramble(I.interleaveBytes(I.rsEncodeAll(dataIn, plan), plan));
  const totalBits = cw.length * 8, getBit = (i) => i < totalBits ? (cw[i >> 3] >>> (7 - (i & 7))) & 1 : 0;
  const nlev = nlevOf(bpc), symbols = new Uint8Array(nCells), cellGray = new Uint8ClampedArray(nCells);
  const pos = buildPerm(nCells);
  let bit = 0;
  for (let k = 0; k < nCells; k++) {
    let v = 0; for (let b = 0; b < bpc; b++) v = (v << 1) | getBit(bit++);
    const p = pos[k]; symbols[p] = v; cellGray[p] = levelGray(grayEnc(v), nlev);
  }
  return { cells: symbols, cellGray: cellGray, plan: plan, usedBytes: frame.length, capBytes: plan.totalK, bitsPerCell: bpc };
}
function decodeBytesFromCells(symbols, nCells, bpc, ecc) {
  const plan = I.planBlocks(Math.floor(nCells * bpc / 8), ecc || PAL_ECC[bpc] || '25%');
  const totalN = plan.totalN, cw = new Uint8Array(totalN), nb = totalN * 8;
  const pos = buildPerm(nCells);
  let bit = 0;
  for (let k = 0; k < nCells && bit < nb; k++) {
    const v = symbols[pos[k]];
    for (let b = bpc - 1; b >= 0 && bit < nb; b--) { if ((v >> b) & 1) cw[bit >> 3] |= (1 << (7 - (bit & 7))); bit++; }
  }
  const seq = I.deinterleaveBytes(scramble(cw), plan);
  let res; try { res = I.rsDecodeAll(seq, plan); } catch (e) { return { ok: false, reason: 'rs-uncorrectable' }; }
  const payload = frameDecode(res.data);
  if (!payload) return { ok: false, reason: 'bad-frame/crc', errors: res.errors };
  return { ok: true, bytes: payload, errors: res.errors };
}

// 알려진 앵커로 흑점/백점 캘리브레이션(전역 M1). 반환 {b0,w0}.
function calibPointsOf(layout) {
  if (layout.calibPoints) return layout.calibPoints;       // 모양이 명시(주로 round)
  const N = layout.N, c = layout.coreCenter.mx, d = (layout.spec.satInset || 4.5);
  const black = [[c, c]]; for (const a of layout.anchors) if (a.type === 'disk') black.push([a.mx, a.my]);
  const white = [[d, d], [c + 2.5, c], [-2, -2], [N + 2, -2], [N + 2, N + 2], [-2, N + 2]];
  return { black, white };
}
function calibrate(grayObj, H, layout) {
  const smp = (mx, my) => { const p = applyH(H, mx, my); return grayAt(grayObj, p[0], p[1]); };
  const cp = calibPointsOf(layout);
  const mean = (arr) => { let s = 0; for (const p of arr) s += smp(p[0], p[1]); return s / arr.length; };
  return { b0: mean(cp.black), w0: mean(cp.white) };
}

// 퍼센타일 lo/hi.
function pct(arr, p) { const s = Float32Array.from(arr).sort(); return s[Math.max(0, Math.min(s.length - 1, Math.floor(p * s.length)))]; }

// 셀 그레이 → 심볼. 흑/백 범위를 "공간 구역별(K×K)" 데이터 분포에서 잡는다(Fable M2).
//   원근서 far쪽이 압축·어두워져 전역 lo/hi 하나론 그쪽 오분류(far 33%). 구역별 lo/hi 로
//   그쪽만의 범위 → far 오류 33%→11%. 도트분포 퍼센타일은 blur/조명서도 균등간격 보존(affine).
//   scramble 로 각 구역도 레벨 균등이라 구역 퍼센타일이 견고. cells=데이터셀 좌표, layout 필요.
function classifyCells(grays, cells, bpc, layout) {
  const nlev = nlevOf(bpc), N = layout.N, K = 6;
  const gLo = pct(grays, 0.02), gHi = pct(grays, 0.98);
  const blocks = Array.from({ length: K * K }, () => []);
  const bidx = (i) => (Math.min(K - 1, Math.floor(cells[i][1] / N * K)) * K + Math.min(K - 1, Math.floor(cells[i][0] / N * K)));
  for (let i = 0; i < grays.length; i++) blocks[bidx(i)].push(grays[i]);
  const lh = blocks.map(a => a.length < 8 ? null : [pct(a, 0.02), pct(a, 0.98)]);
  const sym = new Uint8Array(grays.length);
  for (let i = 0; i < grays.length; i++) {
    const x = lh[bidx(i)] || [gLo, gHi];
    const lo = x[0], span = (x[1] - x[0]) || 1;
    let t = (grays[i] - lo) / span; t = t < 0 ? 0 : t > 1 ? 1 : t;
    sym[i] = grayDec(Math.round((1 - t) * (nlev - 1)));
  }
  return { sym, lo: gLo, hi: gHi, span: gHi - gLo };
}

// 심볼(bpc비트/셀) → 바이트 → RS 디코드 → payload.
function decodeFromCells(symbols, nCells, bpc, ecc, badCells) {
  const plan = I.planBlocks(Math.floor(nCells * bpc / 8), ecc || PAL_ECC[bpc] || '25%');
  const totalN = plan.totalN, cw = new Uint8Array(totalN), nb = totalN * 8;
  const pos = buildPerm(nCells);                 // 위치 산포 역변환: 데이터유닛 k = symbols[pos[k]]
  let bit = 0;
  for (let k = 0; k < nCells && bit < nb; k++) {
    const v = symbols[pos[k]];
    for (let b = bpc - 1; b >= 0 && bit < nb; b--) { if ((v >> b) & 1) cw[bit >> 3] |= (1 << (7 - (bit & 7))); bit++; }
  }
  const seq = I.deinterleaveBytes(scramble(cw), plan);
  let res = null;
  // ★소거 우선 — 오염이 확실한 셀을 알면 예산이 2배가 된다(블록당 nsym/2 → nsym).
  //   실패하면 조용히 옛 경로로 떨어진다(소거 위치가 틀렸을 수도 있으므로).
  const eras = badCells && badCells.length ? cellsToErasures(badCells, nCells, plan, bpc) : null;
  if (eras) { try { res = rsDecodeAllErased(seq, plan, eras); } catch (e) { res = null; } }
  if (!res) { try { res = I.rsDecodeAll(seq, plan); } catch (e) { return { ok: false, reason: 'rs-uncorrectable' }; } }
  const payload = frameDecode(res.data);
  if (!payload) return { ok: false, reason: 'bad-frame/crc', errors: res.errors };
  return { ok: true, text: fromBytes(payload), errors: res.errors, erased: res.erasures || 0 };
}

/* 블록별 소거 위치를 받아 복호한다. rsDecodeAll 과 **같은 블록 순서**를 쓴다. */
function rsDecodeAllErased(cw, plan, erasByBlock) {
  const dp = [], pp = []; let off = 0;
  for (const b of plan.blocks) { dp.push(cw.subarray(off, off + b.k)); off += b.k; }
  for (const b of plan.blocks) { pp.push(cw.subarray(off, off + b.nsym)); off += b.nsym; }
  const out = new Uint8Array(plan.totalK);
  let oo = 0, errs = 0, ers = 0;
  for (let i = 0; i < plan.blocks.length; i++) {
    const b = plan.blocks[i], blk = new Uint8Array(b.k + b.nsym);
    blk.set(dp[i], 0); blk.set(pp[i], b.k);
    const r = I.decodeBlock(blk, b.nsym, erasByBlock[i]);
    out.set(r.data, oo); oo += b.k; errs += r.errors; ers += (r.erasures || 0);
  }
  return { data: out, errors: errs, erasures: ers };
}

// ── 컬러(hue) 레이어 (Fable5 설계, luma×hue 곱코드) ────────────────────────
//   밝기 Y=luma 그대로(toGray 무손상), 색(hue)만 Cb/Cr 에 별도 RS 평면으로. K=2=파랑/노랑축(Cb).
//   hue 는 "중간밝기 셀"만 탑승(검/흰은 색 헤드룸 없음). eligible 은 디코드된 루마로 결정.
const HUE_ECC = '50%', HUE_RHO = 72;   // 채도. 실폰 카메라가 채도를 깎으므로 진하게(gamut 헤드룸 내).
function rgbAtC(img, x, y) {           // 바이리니어 RGB 샘플
  const w = img.width, h = img.height, d = img.data;
  if (x < 0 || y < 0 || x >= w - 1 || y >= h - 1) return [255, 255, 255];
  const x0 = Math.floor(x), y0 = Math.floor(y), fx = x - x0, fy = y - y0, i = (y0 * w + x0) * 4, j = i + w * 4;
  const s = (a, b) => (d[a] * (1 - fx) + d[a + 4] * fx) * (1 - fy) + (d[b] * (1 - fx) + d[b + 4] * fx) * fy;
  return [s(i, j), s(i + 1, j + 1), s(i + 2, j + 2)];
}
function rgb2cbcr(R, G, B) { return [-0.168736 * R - 0.331264 * G + 0.5 * B, 0.5 * R - 0.418688 * G - 0.081312 * B]; }
function hueChroma(sym, hueBits) {
  if (hueBits === 1) return sym === 0 ? [HUE_RHO, 0] : [-HUE_RHO, 0];    // 0=파랑(+Cb) 1=노랑(-Cb)
  const pts = [[HUE_RHO, 0], [-HUE_RHO, 0], [0, HUE_RHO], [0, -HUE_RHO]];  // K=4: +Cr=빨강 -Cr=초록(색맹 비안전)
  return pts[sym & 3];
}
// 루마 심볼(셀별) → hue 자격 셀 인덱스(밝기 rank 극단 제외).
function eligibleFromLuma(lumaSymbols, bpc) {
  const nlev = nlevOf(bpc), out = [];
  for (let i = 0; i < lumaSymbols.length; i++) { const rank = grayEnc(lumaSymbols[i]); if (rank > 0 && rank < nlev - 1) out.push(i); }
  return out;
}
// 각 데이터셀 중심 RGB 평균(타이트 디스크 — 4:2:0 색압축이 셀 안에 머물게).
function sampleCellColor(rgbaImg, H, layout) {
  const dc = GEO.dataCells(layout), rgb = new Array(dc.length);
  const offs = [[0, 0], [0.12, 0], [-0.12, 0], [0, 0.12], [0, -0.12]];
  for (let i = 0; i < dc.length; i++) {
    const mx = dc[i][0] + 0.5, my = dc[i][1] + 0.5; let R = 0, G = 0, B = 0;
    for (const o of offs) { const p = applyH(H, mx + o[0], my + o[1]), c = rgbAtC(rgbaImg, p[0], p[1]); R += c[0]; G += c[1]; B += c[2]; }
    rgb[i] = [R / offs.length, G / offs.length, B / offs.length];
  }
  return { rgb: rgb, cells: dc };
}
// 흑/백 앵커 RGB → 채널별 화이트밸런스 아핀 {Ck[3], Cw[3]}.
function calibrateColor(rgbaImg, H, layout) {
  const N = layout.N, c = layout.coreCenter.mx, d = (layout.spec.satInset || 4.5);
  const smp = (mx, my) => { const p = applyH(H, mx, my); return rgbAtC(rgbaImg, p[0], p[1]); };
  const blk = [[c, c]]; for (const a of layout.anchors) if (a.type === 'disk') blk.push([a.mx, a.my]);
  const wht = [[d, d], [c + 2.5, c], [-2, -2], [N + 2, -2], [N + 2, N + 2], [-2, N + 2]];
  const mean = (arr) => { const s = [0, 0, 0]; for (const p of arr) { const rc = smp(p[0], p[1]); s[0] += rc[0]; s[1] += rc[1]; s[2] += rc[2]; } return s.map(x => x / arr.length); };
  return { Ck: mean(blk), Cw: mean(wht) };
}
// eligible 셀 색 → hue 심볼(화이트밸런스 + 구역별 무채색 기준 + Cb 부호/축 최근접).
function classifyHue(rgbAll, eligibleIdx, cells, lumaSymbols, bpc, cc, layout, hueBits) {
  const N = layout.N, K = 6, nlev = nlevOf(bpc);
  const gain = [0, 1, 2].map(k => 255 / ((cc.Cw[k] - cc.Ck[k]) || 1));
  const cb = new Float32Array(rgbAll.length), cr = new Float32Array(rgbAll.length);
  for (let i = 0; i < rgbAll.length; i++) {
    const R = (rgbAll[i][0] - cc.Ck[0]) * gain[0], G = (rgbAll[i][1] - cc.Ck[1]) * gain[1], B = (rgbAll[i][2] - cc.Ck[2]) * gain[2];
    const v = rgb2cbcr(R, G, B); cb[i] = v[0]; cr[i] = v[1];
  }
  // 구역별 무채색 기준 = 그 구역의 hue-ineligible(극단밝기) 셀 CbCr 평균.
  const bidx = (i) => (Math.min(K - 1, (cells[i][1] / N * K) | 0) * K + Math.min(K - 1, (cells[i][0] / N * K) | 0));
  const nCb = new Float64Array(K * K), nCr = new Float64Array(K * K), nn = new Float64Array(K * K);
  for (let i = 0; i < cells.length; i++) { const rank = grayEnc(lumaSymbols[i]); if (rank === 0 || rank === nlev - 1) { const b = bidx(i); nCb[b] += cb[i]; nCr[b] += cr[i]; nn[b]++; } }
  const gCb = nCb.reduce((a, b) => a + b, 0) / (nn.reduce((a, b) => a + b, 0) || 1), gCr = nCr.reduce((a, b) => a + b, 0) / (nn.reduce((a, b) => a + b, 0) || 1);
  const sym = new Uint8Array(eligibleIdx.length);
  for (let e = 0; e < eligibleIdx.length; e++) {
    const i = eligibleIdx[e], b = bidx(i);
    const bx = nn[b] >= 4 ? nCb[b] / nn[b] : gCb, by = nn[b] >= 4 ? nCr[b] / nn[b] : gCr;
    const x = cb[i] - bx, y = cr[i] - by;
    if (hueBits === 1) sym[e] = x >= 0 ? 0 : 1;               // 파랑/노랑
    else { const pts = [[1, 0], [-1, 0], [0, 1], [0, -1]]; let best = 0, bd = -1e9; for (let k = 0; k < 4; k++) { const dot = x * pts[k][0] + y * pts[k][1]; if (dot > bd) { bd = dot; best = k; } } sym[e] = best; }
  }
  return sym;
}
// 컬러 인코드: 루마 payload + hue payload → {cellGray, cellChroma}.
function encodeColor(lumaText, hueText, nCells, bpc, hueBits) {
  const lu = encodeToCells(lumaText, nCells, bpc);
  const elig = eligibleFromLuma(lu.cells, bpc);
  const hu = encodeToCells(hueText, elig.length, hueBits, HUE_ECC);
  const cellChroma = new Array(nCells).fill(null);
  for (let e = 0; e < elig.length; e++) cellChroma[elig[e]] = hueChroma(hu.cells[e], hueBits);
  return { cellGray: lu.cellGray, cellChroma: cellChroma, bitsPerCell: bpc, hueBits: hueBits,
           eligible: elig.length, lumaCapBytes: lu.capBytes - 6, hueCapBytes: hu.capBytes - 6 };
}
// 원시 바이트 버전(이미지 데모용) — 루마 채널에 lumaBytes, 색 채널에 hueBytes를 각자 독립된
// 바이트 스트림으로 싣는다(같은 이미지의 앞/뒷부분을 나눠 담는 방식 — 픽셀 단위 색변환 아님).
function encodeColorBytes(lumaBytes, hueBytes, nCells, bpc, hueBits) {
  const lu = encodeBytesToCells(lumaBytes, nCells, bpc);
  const elig = eligibleFromLuma(lu.cells, bpc);
  const hu = encodeBytesToCells(hueBytes, elig.length, hueBits, HUE_ECC);
  const cellChroma = new Array(nCells).fill(null);
  for (let e = 0; e < elig.length; e++) cellChroma[elig[e]] = hueChroma(hu.cells[e], hueBits);
  return { cellGray: lu.cellGray, cellChroma: cellChroma, bitsPerCell: bpc, hueBits: hueBits,
           eligible: elig.length, lumaCapBytes: lu.capBytes - 6, hueCapBytes: hu.capBytes - 6 };
}

// 종합: grayObj + locate 결과(res.Hmod2img) → 디코드. 앵커 캘리브레이션 + bpc trial-decode.
//   opts.bpcTry: 시도할 bpc 목록(기본 [1,2,3]). opts.hue=true + rgbaImg → hue 평면도 디코드.
// ── PSF-ISI 디코더 (블러 강건, Fable5 설계) ────────────────────────────────
//   블러는 랜덤이 아니라 결정적 선형채널(저역통과)이다. 셀 독립판독은 이웃 도트 번짐
//   (ISI=심볼간간섭)을 노이즈로 취급해 블러 2px부터 죽는다. 여기선 전체 모듈 그리드를
//   샘플해 가우시안 PSF로 역합성곱(Landweber 최소자승)한 뒤 기존 classify+decode 재사용.
//   PSF σ 는 코어피팅 대신 스윕 + CRC 게이트(trial-decode 철학) — 통과하는 σ 만 채택.
function sampleModuleGrid(grayObj, H, N) {
  const grid = new Float32Array(N * N);
  for (let my = 0; my < N; my++) for (let mx = 0; mx < N; mx++) {
    const p = applyH(H, mx + 0.5, my + 0.5); grid[my * N + mx] = grayAt(grayObj, p[0], p[1]);
  }
  return grid;
}
function gaussKernel1D(sigma) {
  const rad = Math.max(1, Math.ceil(3 * sigma)), k = new Float64Array(2 * rad + 1); let s = 0;
  for (let d = -rad; d <= rad; d++) { const v = Math.exp(-d * d / (2 * sigma * sigma)); k[d + rad] = v; s += v; }
  for (let i = 0; i < k.length; i++) k[i] /= s; return { k, rad };
}
function blurSep(src, N, K) {                       // 분리형 대칭 가우시안(경계 반사) — K=Kᵀ
  const k = K.k, rad = K.rad, tmp = new Float64Array(N * N), out = new Float64Array(N * N);
  for (let y = 0; y < N; y++) for (let x = 0; x < N; x++) { let s = 0; for (let d = -rad; d <= rad; d++) { let xx = x + d; if (xx < 0) xx = -xx; if (xx >= N) xx = 2 * N - 2 - xx; s += src[y * N + xx] * k[d + rad]; } tmp[y * N + x] = s; }
  for (let y = 0; y < N; y++) for (let x = 0; x < N; x++) { let s = 0; for (let d = -rad; d <= rad; d++) { let yy = y + d; if (yy < 0) yy = -yy; if (yy >= N) yy = 2 * N - 2 - yy; s += tmp[yy * N + x] * k[d + rad]; } out[y * N + x] = s; }
  return out;
}
function deconvGauss(obs, N, sigma, iters) {        // Landweber: T ← clamp(T + Kᵀ(O − K T))
  const K = gaussKernel1D(sigma), T = Float64Array.from(obs);
  for (let it = 0; it < iters; it++) {
    const KT = blurSep(T, N, K), resid = new Float64Array(N * N);
    for (let i = 0; i < N * N; i++) resid[i] = obs[i] - KT[i];
    const KtR = blurSep(resid, N, K);
    for (let i = 0; i < N * N; i++) { let v = T[i] + KtR[i]; T[i] = v < 0 ? 0 : v > 255 ? 255 : v; }
  }
  return T;
}
/* ★2026-08-30 Q1-C — 역합성곱 결과 재사용 캐시.
 *   프로파일러 실측: 실패 프레임 22초 중 **14.4초(66%)가 deconvGauss** 였다.
 *   원인은 알고리즘이 아니라 **같은 계산의 반복**이다 — decodePSF 의 `obs` 와 `dec` 는
 *   (grayObj, H, N, sigma) 로만 정해지고 **모양과 무관**한데, detectAuto 의 pass2 는
 *   같은 후보로 26모양 × 2변형을 순회하며 매번 처음부터 다시 계산했다(최대 104회).
 *   모양은 `dec[cells[i]]` 로 **어느 셀을 읽을지**만 정한다.
 *   호출이 연속이라 **크기 1 캐시**로 충분하다(같은 후보의 52회가 연달아 온다). */
var _psfCache = null;   // { gray, H, N, obs, decs:{sigma: Float64Array} }
function _sameH(a, b) {
  if (a === b) return true;
  if (!a || !b || a.length !== b.length) return false;
  for (var i = 0; i < a.length; i++) if (a[i] !== b[i]) return false;
  return true;
}
function _psfGet(grayObj, H, N, sigma) {
  if (_psfCache && _psfCache.gray === grayObj && _psfCache.N === N && _sameH(_psfCache.H, H)) {
    if (_psfCache.decs[sigma]) return _psfCache.decs[sigma];
  } else {
    _psfCache = { gray: grayObj, H: H, N: N, obs: sampleModuleGrid(grayObj, H, N), decs: {} };
  }
  var d = deconvGauss(_psfCache.obs, N, sigma, 45);
  _psfCache.decs[sigma] = d;
  return d;
}

// 표준 셀판독이 블러로 실패했을 때 재시도(1비트). 통과 σ 반환, 실패 시 null.
function decodePSF(grayObj, H, layout, cells) {
  const N = layout.N, dc = new Float32Array(cells.length);
  for (const sig of [0.5, 0.75]) {
    const dec = _psfGet(grayObj, H, N, sig);
    for (let i = 0; i < cells.length; i++) dc[i] = dec[cells[i][1] * N + cells[i][0]];
    const cl = classifyCells(dc, cells, 1, layout);
    if (cl.hi - cl.lo < 25) continue;
    const out = decodeFromCells(cl.sym, cells.length, 1);
    if (out.ok) { out.psfSigma = sig; return out; }
  }
  return null;
}

/* ★2026-08-30 — RS 소거(erasure) 배관은 **남겨 두되, 부르는 곳은 없다.**
 *   `rs.js decodeBlock(cw,nsym,erasures)` + `cellsToErasures` + `decodeFromCells(...,badCells)`
 *   까지는 검증돼 있고(합성 시험: 오류만 250셀 벽 → 소거 350셀, 약 1.4배), 소거를 안 주면
 *   동작이 한 글자도 안 바뀐다. 그러나 **오염 자리를 짚는 층은 전부 걷어냈다** —
 *   회색지대 추정·앵커 근처 원판·구역 대비붕괴 셋 다 실측에서 경계를 한 칸도 못 옮겼다.
 *   왜 안 되는지는 `~/wiacode-perf/SCANNER_AIM_FINDINGS_2026-08-29.md` §I 에 있다.
 *   요약: 실제 광학 손상은 소거가 이기는 구간(오염 250~350셀)에 **거의 안 떨어진다** —
 *   작으면 평범한 RS 가 이미 살리고, 크면 소거 예산(513셀)도 넘긴다(실측 786셀).
 *   다시 쓰려면 `decodeFromCells(sym, n, bpc, ecc, badCells)` 에 **확실한** 자리를 주면 된다. */

function readCode(grayObj, res, layout, cellPx, opts) {
  if (!res || !res.ok || !res.Hmod2img) return { ok: false, reason: 'no-lock' };
  const s = sampleCellGrays(grayObj, res.Hmod2img, layout, cellPx);
  let tries = (opts && opts.bpcTry) || [1, 2, 3];
  for (const bpc of tries) {
    const cl = classifyCells(s.grays, s.cells, bpc, layout);
    if (cl.hi - cl.lo < 25) continue;                    // 대비붕괴 → 이 모드 스킵
    // ECC 비율 trial(신형35%/구형50%) — 오류가 적은 쪽 채택. 추가비용 실측 3.2ms.
    let out = null, outEcc = null;
    for (const ecc of (ECC_TRY[bpc] || [PAL_ECC[bpc]])) {
      const r = decodeFromCells(cl.sym, s.cells.length, bpc, ecc);
      if (r.ok && (!out || r.errors < out.errors)) { out = r; outEcc = ecc; }
    }
    if (out) {
      out.bitsPerCell = bpc; out.ecc = outEcc; out.range = { lo: Math.round(cl.lo), hi: Math.round(cl.hi) };
      // ── hue 평면(있으면) — 루마 성공 후에만. 실패해도 루마 결과는 그대로 반환(우아한 열화). ──
      if (opts && opts.hue && opts.rgbaImg) {
        try {
          const hueBits = opts.hueBits || 1;
          // ★루마와 반드시 같은 ECC 로 재인코딩해야 한다(outEcc 누락 시 컬러 코드만 조용히 깨짐).
          const canon = encodeToCells(out.text, s.cells.length, bpc, outEcc);   // 오류정정된 정규 루마심볼
          const elig = eligibleFromLuma(canon.cells, bpc);
          const col = sampleCellColor(opts.rgbaImg, res.Hmod2img, layout);
          const cc = calibrateColor(opts.rgbaImg, res.Hmod2img, layout);
          const hsym = classifyHue(col.rgb, elig, col.cells, canon.cells, bpc, cc, layout, hueBits);
          const hout = decodeFromCells(hsym, elig.length, hueBits, HUE_ECC);
          out.hue = hout.ok ? { text: hout.text, hueBits: hueBits, errors: hout.errors } : false;
        } catch (e) { out.hue = false; }
      }
      return out;
    }
  }
  // 표준 셀판독 실패(주로 블러) → PSF-ISI 재시도(1비트). 우아한 열화: 안 되면 원래대로 실패.
  if (!(opts && opts.psf === false)) {
    const p = decodePSF(grayObj, res.Hmod2img, layout, s.cells);
    if (p && p.ok) { p.bitsPerCell = 1; return p; }
  }
  return { ok: false, reason: 'all-modes-failed' };
}

module.exports = {
  ECC, encodeToBits, decodeFromBits, frameEncode, frameDecode, planFor, sampleCellGrays, otsu, readCode,
  encodeToCells, decodeFromCells, calibrate, classifyCells, levelGray, grayEnc, grayDec, PAL_ECC, ECC_TRY,
  encodeColor, eligibleFromLuma, sampleCellColor, calibrateColor, classifyHue, hueChroma, HUE_ECC, HUE_RHO,
  encodeBytesToCells, decodeBytesFromCells, encodeColorBytes,
  // ── P3 세그먼트 모드(VER=2) ──
  VER, VER2, MAGIC, frameEncodeSeg, pickFrame, segEncode, segDecode, segPlan,
  A_TABLE, S64_TABLE, SEG_MODES, SEG_END, SEG_N, SEG_A, SEG_H, SEG_B, SEG_C7, SEG_S64,
  SEG_CNT_BITS, SEG_MAX_COUNT, SEG_HDR_BITS, H_CNT, HAN_CNT,
};

});


// ===== WiaScan 실시간 API =====
var FR = require('./frst'), CN = require('./conic'), GEO = require('./geometry'), LOC = require('./locate'), DEG = require('./degrade');
var toGray = FR.toGray, frst = FR.frst, peaks = FR.peaks;
// 배율추정 전용 경량 디노이즈: 강한 노이즈가 코어 링에지를 가짜로 만들어 cellPx 를
//   과소추정하는 것 방지. 정밀 locate 는 원본 그레이(노이즈 강건) 사용.
function _denoiseGray(imageData){ var b = DEG.boxBlur({ data:imageData.data, width:imageData.width, height:imageData.height }, 1, 1); return toGray(b.img); }
var layout = GEO.layout, SPEC = GEO.SPEC, render = GEO.render;
/* ★다리QR 배치 변형 (2026-09-02) — 0=존 없음(구형 코드) · 1=현행(형상별 자리 + pad 0.35)
 *   · 2=옛 다리QR(가로중앙 topMy 77.5 + pad 1.4, 2026-09-02 이전 발행분).
 *
 *   ★이 skip 은 두 번 뒤집혔다 — 기록해 둔다:
 *     ① 처음엔 "2 는 자리를 옮긴 34종에서만" 으로 짰다 → 왕복 시험에서 하트·별·입·뇌의
 *        옛 코드가 해독 실패. 당시엔 pad 축소를 **전 형상**에 걸어서, 자리를 안 옮겨도
 *        예약 사각이 줄어 셀 집합이 달라졌기 때문이었다.
 *     ② 그래서 전 형상에 2 를 시도하게 바꿨더니 이번엔 hugon-battery 가 foot/L 을 잡았다
 *        (pad 축소가 그림을 바꿔 격자 오판을 유발 — geometry.js bridgeRect 주석 참조).
 *     ③ 결론: **pad 축소도 자리를 옮긴 형상에만** 건다. 그러면 나머지 32종은 픽셀까지
 *        예전 그대로라 변형 2 가 아예 불필요하다 → 여기서 건너뛴다(해독 비용 그대로).
 *   비용: 새 형식 코드는 변형 1 에서 끝나므로 영향 없다. 옮긴 34종의 옛 발행분만
 *   변형 2 까지 가면서 readCode 가 한 번 더 든다(이미 인쇄된 코드를 살리는 값).
 *
 *   ★★2026-09-03 — ②를 되살린다(오너 지시). ③의 후퇴는 **foot/L 격자 오판 하나 때문**이었고
 *     그건 C7(밀착 아핀 대안 + 원거리격자 재적합, 09-02)이 그 뒤에 근치했다. 재현 확인:
 *     foot/L 을 정확히 원본 816px 로 줘도 격자를 L 로 맞힌다(09-03 실측). 그래서 pad 축소를
 *     square 를 뺀 전 형상으로 넓히고(geometry.js BRIDGE_PAD_FROZEN), 변형 2 도 다시
 *     전 형상에서 시도한다 — ①의 하위호환 붕괴를 막는 것이 정확히 이 변형이다.
 *   ★목록을 손으로 적지 않는다. **"옛 배치와 지금 배치가 실제로 다른 형상"**을 bridgeRect 로
 *     직접 비교해 판정한다(형상당 1회 캐시). pad 정책이 또 바뀌어도 이 게이트가 따라온다 —
 *     square 처럼 옛 배치와 동일한 형상은 자동으로 건너뛴다(비용 0). */
var BPOS = GEO.BRIDGE_POS || {};
var _BR_N = GEO.GRID_CELLS[GEO.BRIDGE.grid], _bv2 = {};
function _bridgeDiffers(shape) {
  try {
    var a = GEO.bridgeRect(_BR_N, shape, false), b = GEO.bridgeRect(_BR_N, shape, true);
    if (!a || !b) return false;
    var x = a.reserve, y = b.reserve;
    return x.mx0 !== y.mx0 || x.my0 !== y.my0 || x.mx1 !== y.mx1 || x.my1 !== y.my1;
  } catch (e) { return !!BPOS[shape]; }   // 계산 실패 시 예전 규칙으로 안전 복귀
}
/* ★변형 3 = **골 도입 전 발행분**(2026-09-02 B4). 내부 백선(SHAPE_GROOVES)이 들어간 5종은
 *   데이터 셀 집합이 달라져 옛 코드가 안 읽힌다 — C9 의 옛 다리QR 배치와 같은 종류의 문제다.
 *   골이 있는 형상에서만 한 번 더 시도한다(나머지 61종은 비용 그대로). */
var GROOVED = GEO.SHAPE_GROOVES || {};
function bvSkip(v, shape) {
  if (v === 2) { if (!(shape in _bv2)) _bv2[shape] = _bridgeDiffers(shape); return !_bv2[shape]; }
  if (v === 3) return !GROOVED[shape];
  return false;
}
function bvOpt(v, extra) {
  var o = { bridge: (v !== 0), bridgeLegacy: (v === 2), noGrooves: (v === 3) };
  if (extra && extra.hug !== undefined) o.hug = extra.hug;
  return o;
}
var pickCoreSeed = LOC.pickCoreSeed, locateRobust = LOC.locateRobust;
// ★2026-08-30 Q1-B — conic 정면화 공유(아래 격자 루프 주석 참고)
var prepareConic = LOC.prepareConic, locateRobustShared = LOC.locateRobustShared;
var extractCoreRings = CN.extractCoreRings, estimateCoreScale = CN.estimateCoreScale;
var CODEC = require('codec'), dataCells = GEO.dataCells;
function _now(){ return (typeof performance!=='undefined'&&performance.now)?performance.now():Date.now(); }

/* ★2026-08-30 Q2 — 밀착 실루엣의 **4반경 시그니처** 표.
 *   밀착은 실루엣마다 위성 반경이 다르므로 'round' 하나로 대표할 수 없다.
 *   전수로 후보에 넣으면 격자3 × 모양N = 최대 84 조합(지금 6). 그래서 **먼저 분류**한다.
 *   Fable 측정: 정면화 프레임에서 반경 측정 오차 평균 0.85%·최대 2.76%,
 *   top-1 분류 93/96, 격자 간 충돌 0쌍(시그니처가 격자까지 동시에 확정한다).
 *   ★표는 첫 호출 시 1회만 만든다. 밀착 대상은 **이득 ≥1.25배**인 실루엣만
 *     (꽉 찬 것은 밀착해도 얻는 게 없고 시그니처 충돌만 만든다 — 실측). */
/* 그레이 이중선형 표본 — frst.js 의 grayAt 은 export 되지 않아 여기서 쓴다. */
function _gAt(G, x, y){
  var w = G.w != null ? G.w : G.width, h = G.h != null ? G.h : G.height, g = G.g || G.data;
  if (x < 0 || y < 0 || x > w-2 || y > h-2) return 255;
  var x0 = Math.floor(x), y0 = Math.floor(y), fx = x-x0, fy = y-y0, i = y0*w+x0;
  return (g[i]*(1-fx)+g[i+1]*fx)*(1-fy) + (g[i+w]*(1-fx)+g[i+w+1]*fx)*fy;
}

/* 프레임에서 4반경을 잰다(모듈). prep 이 있으면 정면화 프레임(metric)에서,
 * 없으면 원시 프레임에서. 도넛(가장 밝은 위성)을 TL 로 놓고 각도순 시계방향. */
function _hugMeasure(gray, peaksList, cellPx, prep, seedHint){
  var G, cx, cy, pts;
  if (prep && prep.gray && prep.peaks) {
    G = prep.gray; cx = prep.dim/2; cy = prep.dim/2; pts = prep.peaks;
  } else {
    G = gray; pts = peaksList;
    // ★힌트 우선(2026-08-31) — 위 locate.js 와 같은 이유. 원시(비정면화) 모드에서
    //   yaw 가 들어가면 여기가 위성을 코어로 집어 배율·반경이 통째로 어긋났다.
    var seed = (seedHint && isFinite(seedHint.x) && isFinite(seedHint.y))
             ? seedHint : pickCoreSeed(gray, peaksList, 3.5*cellPx);
    if (!seed) return null;
    cx = seed.x; cy = seed.y;
  }
  var cand = [];
  for (var i = 0; i < pts.length; i++) {
    var d = Math.hypot(pts[i].x - cx, pts[i].y - cy) / cellPx;
    if (d < 8 || d > 66) continue;                 // 궤도(8.5)보다 밖, 콰이엇 안쪽
    var dup = false;
    for (var j = 0; j < cand.length; j++) if (Math.hypot(cand[j].x-pts[i].x, cand[j].y-pts[i].y) < 3*cellPx) { dup = true; break; }
    if (dup) continue;
    cand.push({ x:pts[i].x, y:pts[i].y, d:d, ang:Math.atan2(pts[i].y-cy, pts[i].x-cx),
                bright: _gAt(G, pts[i].x, pts[i].y), score: pts[i].score });
  }
  if (cand.length < 3) return null;
  cand.sort(function(a,b){ return b.score - a.score; });
  cand = cand.slice(0, 4);
  if (cand.length < 3) return null;
  // 도넛 = 가장 밝은 것. 없으면(전부 어두우면) 각도 최소인 것을 임시 기준으로.
  var di = 0, bm = -Infinity;
  for (var k2 = 0; k2 < cand.length; k2++) if (cand[k2].bright > bm) { bm = cand[k2].bright; di = k2; }
  var a0 = cand[di].ang;
  var slots = [null,null,null,null];
  for (var m = 0; m < cand.length; m++) {
    var rel = (cand[m].ang - a0) / (Math.PI/2);
    var sl = ((Math.round(rel) % 4) + 4) % 4;
    if (!slots[sl]) slots[sl] = cand[m];
  }
  var r = [];
  for (var q = 0; q < 4; q++) r.push(slots[q] ? slots[q].d : -1);
  // 빠진 슬롯은 인접 평균으로 채운다(가림 대비) — 분류 창이 6% 라 이 정도는 흡수된다.
  for (var q2 = 0; q2 < 4; q2++) if (r[q2] < 0) {
    // ★C3(2026-09-04): 마름모는 마주 보는 앵커가 같다(아래 잠금표 주석) — 그런데 여기는 **인접**(다른 축)
    //   평균으로 채우고 있었다. whale/L(13.75/21.75)에서 BR 가림 → 13.75 자리가 21.75 로 채워져 mA 가
    //   17.74 로 틀어지고 6% 창 안에서 heart/S 에 오매칭됐다(실측). 마주 보는 슬롯이 있으면 그걸 쓴다.
    var opp = r[(q2+2)%4];
    if (opp > 0) { r[q2] = opp; continue; }
    var a = r[(q2+3)%4], b = r[(q2+1)%4];
    r[q2] = (a > 0 && b > 0) ? (a+b)/2 : (a > 0 ? a : (b > 0 ? b : -1));
  }
  for (var q3 = 0; q3 < 4; q3++) if (r[q3] < 0) return null;
  return { r: r };
}

/* ★밀착 잠금표 — **(R긴, R짧) 쌍**으로 묶는다 (2026-08-30, 마름모 정책).
 *   마름모는 마주 보는 앵커가 같으므로 앵커 배치를 정하는 값이 **둘뿐**이다.
 *   그리고 계단이 Rs×0.8^k 라 쌍끼리 **25% 씩** 벌어진다(측정오차 2.8% 대비 9배).
 *   ★같은 쌍을 쓰는 실루엣들은 **앵커 배치가 완전히 동일**하다 — 잠금은 한 번만 하고,
 *     어느 형상인지는 이미 있는 readCode 의 shape 순회가 CRC 로 가른다.
 *     그래서 표의 단위가 '형상'이 아니라 '쌍'이다(이전 4벡터 최근접 방식과 다른 점). */
var _hugSigTable = null;
function _hugSig(){
  if (_hugSigTable) return _hugSigTable;
  var byKey = {};
  var ids = SHAPE_REG.currentIds ? SHAPE_REG.currentIds() : [];
  ['S','M','L'].forEach(function(g){
    for (var i = 0; i < ids.length; i++) {
      var sh = ids[i]; if (sh === 'square') continue;
      // ★핀이 없으면 건너뛴다 — 없으면 layout 이 solver 를 돌려 **종당 ~1.2초**가 든다.
      if (!(SHAPE_REG.hugPin && SHAPE_REG.hugPin(sh, g))) continue;
      var L; try { L = layout(g, SPEC, sh, { hug: true }); } catch (e) { continue; }
      var c = L.N / 2, Rs = L.N / 2 - (SPEC.satRimMargin || 3.5);
      var rA = null, rB = null;
      for (var k = 1; k < L.anchors.length; k++) {
        var an = L.anchors[k], d = Math.hypot(an.mx - c, an.my - c);
        if (an.name === 'TL') rA = d; else if (an.name === 'TR') rB = d;
      }
      if (rA == null || rB == null) continue;
      /* ★★게이트 **삭제** (2026-08-30, Fable).
       *   여기 있던 "이득<1.25 이고 φ==0 이면 제외" 규칙이 **boomerang M·L 을 표에서 떨어뜨렸다**
       *   (이득 1.148·1.105, φ=0 — 25종 중 이 둘만 걸린다). 생성기는 HUG_ON 으로 밀착
       *   boomerang 을 굽는데 해독기는 그 레이아웃을 **한 번도 제안하지 않아** 락은 되고
       *   readCode 만 영원히 실패했다(legacy-battery 200/208 의 그 8건 = boomerang×격자2×열화4).
       *   ★규칙: **핀이 박혀 있으면 = 생성될 수 있으면 표에 넣는다.** 표를 줄이는 최적화가
       *     해독 가능성을 깎으면 안 된다. 실측 비용: 항목 63 → 65, 다중형상 키 0(증가 없음),
       *     레거시 프레임이 6% 창에 헛짚는 항목도 3개로 변화 없다. */
      var _phi = (SHAPE_REG.hugPin(sh, g) || [])[2] || 0;
      /* ★φ 는 **키에 들어가야 한다** — 반경이 같아도 φ 가 다르면 앵커 자리가 다르다.
       *   키에서 빠뜨리면 φ 가 다른 실루엣끼리 한 항목으로 뭉쳐 엉뚱한 자리에 잠근다. */
      var key = g + '|' + rA.toFixed(2) + '|' + rB.toFixed(2) + '|' + _phi;
      if (!byKey[key]) byKey[key] = { grid: g, rA: rA, rB: rB, phi: _phi, shapes: [], N: L.N };
      byKey[key].shapes.push(sh);
    }
  });
  var t = []; for (var kk in byKey) t.push(byKey[kk]);
  _hugSigTable = t;
  return t;
}
/* 잰 두 반경(TL/BR 축, TR/BL 축)으로 잠금 쌍을 고른다. 최대 상대편차로 거리를 잰다.
 * 창이 6% 인데 쌍 간격이 25% 라 실질적으로 top-1 이 확정된다. */
var HUG_PIN_D = 0.002;   // C10 밀착 갈래 판정 핀거리 상한(실측 근거는 아래 갈래 주석)
function _hugMatch(meas, topK){
  var t = _hugSig(), out = [];
  for (var i = 0; i < t.length; i++) {
    var dA = Math.abs(meas[0] - t[i].rA) / Math.max(meas[0], t[i].rA);
    var dB = Math.abs(meas[1] - t[i].rB) / Math.max(meas[1], t[i].rB);
    // ★C3(2026-09-04): 도넛이 가려지면 _hugMeasure 의 슬롯 기준이 임의의 원판이 되어 (mA,mB) 가 뒤집힌 채
    //   온다 — 위치 의존 비교는 그때 엉뚱한 형상(ear/S)에 매치돼 시간을 버린다(실측). 축 배정은 어차피
    //   locate 가 피크에서 다시 정하므로 매칭은 순서를 무시해도 안전하다.
    var dA2 = Math.abs(meas[1] - t[i].rA) / Math.max(meas[1], t[i].rA);
    var dB2 = Math.abs(meas[0] - t[i].rB) / Math.max(meas[0], t[i].rB);
    out.push({ grid: t[i].grid, shapes: t[i].shapes, shape: t[i].shapes[0], d: Math.min(Math.max(dA, dB), Math.max(dA2, dB2)) });
  }
  out.sort(function(a,b){ return a.d - b.d; });
  return out.slice(0, topK || 3).filter(function(x){ return x.d < 0.06; });
}

/* ★밀착 1급 시도 (2026-08-30, Fable) — **밀착이 기본값인 세계의 순서**.
 *   왜 필요한가: 밀착 코드는 기존(비밀착) 레이아웃으로는 **원리적으로 절대 안 읽힌다.**
 *   그런데 예전 구조는 밀착을 맨 뒤에 두었으므로, 밀착 코드 한 장을 읽으려면 그 전에
 *   1차(비psf) 26형상×2변형×후보6×배율3 = 324회 + 2차(psf) 108회 = **432회의 readCode 를
 *   전부 헛돌린 뒤에야** 밀착에 도달했다(실측: 성공 프레임 readCode 542회 중 540회가 헛일,
 *   7.4s/9.0s). 밀착이 소수일 때는 옳은 순서였지만 기본값이 되면서 완전히 뒤집혔다.
 *   → 이 함수는 **이미 있는 락 재료(gray·pk·cellPx·prep)만 재사용**해서 밀착을 먼저 친다.
 *     새 FRST 도, 새 정면화도 만들지 않는다 — 그래서 레거시 프레임에 비용을 얹지 않는다. */
function _hugAttempt(imageData, gray, pk, cellPx, prep, redetect, psfArr, seedHint, maxD){
  var hm = _hugMeasure(gray, pk, cellPx, prep, seedHint);
  if (!hm) return null;
  // 마주 보는 짝을 평균해 두 축 반경으로 (마름모라 짝이 같다 — 한쪽이 가려져도 반대쪽이 받친다)
  var mA = (hm.r[0] + hm.r[2]) / 2, mB = (hm.r[1] + hm.r[3]) / 2;
  var hc = _hugMatch([mA, mB], 3);
  if (!hc.length) return null;
  // ★C10(2026-09-04): 핀거리 상한 — locate 도 readCode 도 하기 전에 자른다.
  //   _hugMeasure 는 피크 기하만 보므로 실측 0~1ms(1회)로 사실상 공짜다.
  if (maxD != null && hc[0].d > maxD) return null;
  var list = [];
  for (var i = 0; i < hc.length; i++) {
    var lay = layout(hc[i].grid, SPEC, hc[i].shape, { hug: true });
    var r = prep ? locateRobustShared(imageData, gray, pk, lay, { cellPx:cellPx, redetect:redetect }, prep)
                 : LOC.locate(gray, pk, lay);
    if (!(r && r.ok)) continue;
    // 궤도 재적합 — 밀착은 위성이 안으로 들어와 가장자리 외삽이 커진다. 궤도(8.5)를 끼워 내삽으로.
    var r2 = LOC.refitWithOrbit(gray, r, lay, cellPx) || r;
    list.push({ grid:hc[i].grid, anchorShape:hc[i].shape, lay:lay, hugShapes:hc[i].shapes,
                res: r2.method ? r2 : Object.assign({ method:'orbit' }, r2) });
    if (r2 !== r) list.push({ grid:hc[i].grid, anchorShape:hc[i].shape, lay:lay, hugShapes:hc[i].shapes,
                res: Object.assign({}, r, { method:'hugraw' }) });
  }
  if (!list.length) return null;
  // ★부스터(bridge) 변형을 반드시 같이 돌린다 — L 은 부스터 자리가 예약돼 있어
  //   bridge 레이아웃이 아니면 데이터셀 집합 자체가 어긋난다(밀착 왕복 152→72 사고).
  function _tryList(lst) {
    for (var p2 = 0; p2 < psfArr.length; p2++) {
      for (var j = 0; j < lst.length; j++) {
        var C = lst[j], vs = [0]; if (C.grid === 'L') { vs.push(1); vs.push(2); vs.push(3); }
        var shs = C.hugShapes || [C.anchorShape];
        for (var b = 0; b < vs.length; b++) for (var q = 0; q < shs.length; q++) {
          if (bvSkip(vs[b], shs[q])) continue;
          var l2 = (vs[b] === 0 && shs[q] === C.anchorShape) ? C.lay
                 : layout(C.grid, SPEC, shs[q], bvOpt(vs[b], { hug: true }));
          var d = CODEC.readCode(gray, C.res, l2, cellPx, { hue:true, hueBits:1, rgbaImg:imageData, psf:psfArr[p2] });
          if (d && d.ok) return { dec:d, cand:C, shape:shs[q], bridge:(vs[b] !== 0) };
        }
      }
    }
    return null;
  }
  var hit = _tryList(list);
  if (hit) return hit;
  /* ★밀착 **아핀 대안** (2026-09-02, Fable · C7 착수 중 실측으로 발견)
   *   foot/L 이 원본 크기 816px 근처에서 "락은 완벽(반경 16.01/51.03 = 핀과 0.1%)인데
   *   readCode 만 실패" 했다. 셀 단위로 재보니 사영 호모그래피 H 가 코어에서 먼 셀(40모듈+)
   *   에서 2.5~5px 벗어나 있었고(cellPx 6), 오분류 101/3895 = 2.6% 가 전부 그 바깥 띠에 몰렸다.
   *   원인은 **밀착 마름모의 종횡비**다 — foot 은 짧은축 16·긴축 51 이라 위성 4점이 거의
   *   한 줄에 놓이고, 그러면 H 의 사영항(h6·h7)이 제약을 못 받아 정면 사진에도 가짜
   *   원근이 붙는다(h6 -3.3e-4). 앵커 자리에선 정확(잔차 0.36px)하고 멀어질수록 틀리는 전형.
   *   ★같은 5점(코어+위성4)에 H 가 찍는 자리를 **그대로 통과하는 아핀**으로 바꾸면
   *   오분류 0/3895, 최대 오차 1.9px 로 떨어지고 해독된다 — 도트 0.42·0.50 양쪽, 816·813px 양쪽.
   *   즉 새 정보 없이 **모델만 바꾼다**(H 가 믿을 만한 자리의 값만 쓰고 사영항을 버린다).
   *   ★도트 0.42 에서도 foot@816 은 오분류 86(2.2%) 로 칼날 위였다 — 이 대안은 C7 과 무관하게
   *   지금의 잠재 취약점을 없앤다. 기울어진 프레임(yaw)은 사영 H 가 맞으므로 원본을 먼저
   *   시도하고, **전부 실패했을 때만** 아핀으로 한 번 더 돈다(성공 프레임 비용 0). */
  function _apH(H, mx, my) { var Z = H[6]*mx + H[7]*my + H[8]; return [(H[0]*mx + H[1]*my + H[2]) / Z, (H[3]*mx + H[4]*my + H[5]) / Z]; }
  function _affineThrough(H, pts) {           // pts: [{mx,my}] — H 의 자기투영을 지나는 최소제곱 아핀
    var S = [[0,0,0],[0,0,0],[0,0,0]], bx = [0,0,0], by = [0,0,0], i, j, k;
    for (k = 0; k < pts.length; k++) {
      var p = _apH(H, pts[k].mx, pts[k].my), v = [pts[k].mx, pts[k].my, 1];
      for (i = 0; i < 3; i++) { for (j = 0; j < 3; j++) S[i][j] += v[i]*v[j]; bx[i] += v[i]*p[0]; by[i] += v[i]*p[1]; }
    }
    function solve(A, bb) {
      var M = [A[0].concat([bb[0]]), A[1].concat([bb[1]]), A[2].concat([bb[2]])], c, r, piv, f, kk, t;
      for (c = 0; c < 3; c++) {
        piv = c; for (r = c+1; r < 3; r++) if (Math.abs(M[r][c]) > Math.abs(M[piv][c])) piv = r;
        t = M[c]; M[c] = M[piv]; M[piv] = t;
        if (Math.abs(M[c][c]) < 1e-9) return null;
        for (r = 0; r < 3; r++) { if (r === c) continue; f = M[r][c] / M[c][c]; for (kk = c; kk < 4; kk++) M[r][kk] -= f * M[c][kk]; }
      }
      return [M[0][3]/M[0][0], M[1][3]/M[1][1], M[2][3]/M[2][2]];
    }
    var a = solve(S, bx), b2 = solve(S, by);
    if (!a || !b2) return null;
    return [a[0], a[1], a[2], b2[0], b2[1], b2[2], 0, 0, 1];
  }
  /* ★비용 상한: 이 대안은 **가늘고 긴 마름모**에서만 의미가 있다(위성 4점이 한 줄에 가까울 때만
   *   사영항이 풀린다). 종횡비 < 1.8 인 밀착 형상(하트·원형류)은 원래 H 가 이미 정확하므로
   *   건너뛴다 — 실패 프레임(심블러 등)에서 헛도는 readCode 를 안 늘리기 위해서다.
   *   L 기준 노출 형상: victory 3.38 · foot 3.19 · rabbit 2.76 · like 2.73 · hand 2.63 · pestbug 2.61 · finger 2.21 · nose 2.20. */
  var alt = [];
  for (var ai2 = 0; ai2 < list.length; ai2++) {
    var E = list[ai2], Hs = E.res && E.res.Hmod2img;
    if (!Hs || !E.lay || !E.lay.anchors) continue;
    var _cc = E.lay.coreCenter, _rA = null, _rB = null;
    for (var _ak = 1; _ak < E.lay.anchors.length; _ak++) { var _an = E.lay.anchors[_ak], _dd = Math.hypot(_an.mx - _cc.mx, _an.my - _cc.my);
      if (_an.name === 'TL') _rA = _dd; else if (_an.name === 'TR') _rB = _dd; }
    if (_rA == null || _rB == null || Math.max(_rA, _rB) / Math.max(1e-6, Math.min(_rA, _rB)) < 1.8) continue;
    var Ha = _affineThrough(Hs, E.lay.anchors);
    if (!Ha) continue;
    var ra = {}; for (var kx in E.res) ra[kx] = E.res[kx];
    ra.Hmod2img = Ha; ra.altH = null; ra.method = (E.res.method || 'hug') + '+aff';
    alt.push({ grid:E.grid, anchorShape:E.anchorShape, lay:E.lay, hugShapes:E.hugShapes, res:ra });
  }
  if (alt.length) { hit = _tryList(alt); if (hit) return hit; }
  /* ★원거리 격자 재적합 (2026-09-02, Fable · C7) — 아핀 대안이 못 살리는 **기울어진 프레임**용.
   *   실측(foot/L yaw30, 도트 0.50): 사영 H 가 40모듈 바깥 띠에서 85/1294 셀을 틀리게 읽었다.
   *   아핀은 진짜 원근이 있으면 오히려 40% 오분류(무용). 그런데 코드 자체가 **먼 자리에 도트를
   *   수천 개** 갖고 있다 — H 가 예측한 자리 근처의 어두운 무게중심을 찾아 대응점으로 삼고
   *   (코어+위성 5점에 더해) 사영 H 를 다시 맞추면 85→57→21→2 로 떨어져 1회 만에 해독된다.
   *   정면 816px 도 101→67→20→0. 즉 "H 는 중심 근처에서만 정확하다"는 근본 문제를 코드의
   *   원거리 도트로 직접 보정한다. 실패 경로에서만 돌고(성공 프레임 비용 0), 최대 3회.
   *   후보 도트 선별: 예측 자리가 어둡고(≤90), 창 안 어두운 비율 25~90%(빈 창·통짜 검정 제외),
   *   무게중심 이동 < 0.45 cellPx. 재적합은 locate.js homographyLS(코어·위성 대응 + 원거리 대응). */
  function _latticeRefit(E, Hcur) {
    var Lr = E.lay, cc = Lr.coreCenter, res = E.res;
    var A = [], Bp = [];
    if (res.core) { A.push([cc.mx, cc.my]); Bp.push([res.core.x, res.core.y]); }
    for (var ak = 0; ak < Lr.anchors.length; ak++) { var an = Lr.anchors[ak]; if (an.name === 'core' || !res.corners || !res.corners[an.name]) continue;
      var cp = res.corners[an.name]; A.push([an.mx, an.my]); Bp.push([cp.x !== undefined ? cp.x : cp[0], cp.y !== undefined ? cp.y : cp[1]]); }
    var dcs = dataCells(Lr), win = Math.max(2, 0.55 * cellPx), used = 0;
    for (var ci = 0; ci < dcs.length; ci++) {
      var mx = dcs[ci][0] + 0.5, my = dcs[ci][1] + 0.5;
      if (Math.hypot(mx - cc.mx, my - cc.my) < 25) continue;
      var Z = Hcur[6]*mx + Hcur[7]*my + Hcur[8], px = (Hcur[0]*mx + Hcur[1]*my + Hcur[2]) / Z, py = (Hcur[3]*mx + Hcur[4]*my + Hcur[5]) / Z;
      if (_gAt(gray, px, py) > 90) continue;
      var sw = 0, sx = 0, sy = 0, dark = 0, tot = 0;
      for (var dy = -win; dy <= win; dy += 0.5) for (var dx = -win; dx <= win; dx += 0.5) {
        if (dx*dx + dy*dy > win*win) continue;
        var v = 255 - _gAt(gray, px + dx, py + dy); tot++; if (v > 128) dark++; sw += v; sx += v*dx; sy += v*dy; }
      if (!sw || dark / tot < 0.25 || dark / tot > 0.9) continue;
      var ox = sx / sw, oy = sy / sw; if (Math.hypot(ox, oy) > 0.45 * cellPx) continue;
      A.push([mx, my]); Bp.push([px + ox, py + oy]); used++;
    }
    if (used < 24) return null;
    var Hn = null; try { Hn = LOC.homographyLS(A, Bp); } catch (e) { Hn = null; }
    if (!Hn || Hn.length !== 9 || !isFinite(Hn[0])) return null;
    return Hn;
  }
  for (var li3 = 0; li3 < list.length; li3++) {
    var E3 = list[li3]; if (!E3.res || !E3.res.Hmod2img) continue;
    var Hc = E3.res.Hmod2img;
    for (var it = 1; it <= 3; it++) {
      var Hn2 = _latticeRefit(E3, Hc); if (!Hn2) break;
      Hc = Hn2;
      var r3 = {}; for (var k3 in E3.res) r3[k3] = E3.res[k3];
      r3.Hmod2img = Hc; r3.altH = null; r3.method = (E3.res.method || 'hug') + '+lat' + it;
      hit = _tryList([{ grid:E3.grid, anchorShape:E3.anchorShape, lay:E3.lay, hugShapes:E3.hugShapes, res:r3 }]);
      if (hit) return hit;
    }
  }
  return { dec:null, list:list };
}

// ★2026-08-16 버그수정: 앵커모양(사각/링) 판정과 데이터모양(square/round/heart/clover/boomerang/
//   star/hex) 판정은 별개다 — round·heart·clover·boomerang·star·hex 6개 실루엣이 전부 같은 "링"
//   앵커 배치를 공유한다(layout()이 shape!=='square'면 전부 동일 위성좌표를 씀, geometry.js 참고).
//   그런데 아래 CRC 트라이얼 목록이 오랫동안 ['round','heart'] 두 개만 시도해서, 나머지 4개
//   실루엣은 위치는 정확히 잡히는데(anchorShape='round') insideShape 셀마스크를 한 번도 시도 안 해
//   CRC가 영원히 안 맞아 디코딩 자체가 불가능했다(생성기·기하는 멀쩡했는데 스캐너 트라이얼목록
//   누락이 원인 — clover/boomerang/star/hex 4개 모양 전수 재현·수정 확인, MEMORY 기록 참고).
// ★2026-08-18: 'bubble'(말풍선) 추가 — 같은 사고를 반복하지 않으려면 새 실루엣은 반드시
//   여기에도 등록해야 한다(insideShape에만 추가하고 이 배열을 빠뜨리면 위와 완전히 같은
//   증상: 생성은 되는데 트라이얼 목록 누락으로 디코드가 영원히 안 됨).
// ★2026-08-26: 'lid'/'piano'/'fish'(옛 파일럿명 'flatfish', 오너 재가로 정식 개명) 3종
//   추가 — 산업 실루엣 팩, geometry.js insideShape 참고. 정품 승격(오너 재가 2026-08-26)
//   generate.html "산업 실루엣" 섹션에 노출.
// ★2026-08-26 파일럿(디자인 게이트용, 생성기 UI 미노출): AI 팩 3종 + 인체 팩 4종.
//   보고서→오너 검수 후 승격분만 남기고 나머지는 이 배열에서도 함께 뺄 것.
// ★★2026-08-28(오너 지시 "유령 로스터 원천 삭제") — 이 배열은 **해독 트라이얼 목록**이다.
//   스캔 한 번마다 여기 있는 모양을 전부 시도하므로, 만들 수 없는 모양이 남아 있으면
//   유령일 뿐 아니라 **매 스캔의 순수 손해**다. 정리 전 16종 중 생성 가능한 건 7종뿐이었다.
//   뺀 9종과 사유:
//     bubble  실물이 말풍선 같지 않아 생성 중단(2026-08-27). 발행분은 오너 본인 것뿐이고
//             자격증은 하트를 쓰며 모핑 영상은 프레임마다 같은 링크라 손실 없음(오너 확인).
//     chat    bubble 의 생성 후속으로 만들었으나 어느 경로에서도 생성된 적 없음.
//     piano · lid · fish   2026-08-26 오너 재검수 불합격(소형 품질 미달) + 배포 자산 2종이
//             해독 불가로 격리됨. 발행분은 오너가 만든 piano 한 장뿐(2026-08-26).
//     ear · palm · footprint · brain   인체 팩 파일럿. 생성기·API 어디에도 노출된 적 없음
//             — 이 파일 자신의 규칙("게이트 전 파일럿은 정본 명단에 올리지 말 것") 위반 상태였다.
//   ★기하 함수(insideShape)는 지우지 않았다 — 되살릴 땐 여기에 이름만 다시 올리면 된다.
//     단, 등재 조건은 그대로다: **기하 함수 실존 + 오너 게이트 통과**.
// ★★2026-08-28 — 이 목록은 이제 **정본 레지스트리에서 파생**된다(orbit/shape-registry.js).
//   예전엔 여기에 손으로 적었고, 같은 목록이 네 군데에 복사돼 있었다. 하나만 빠뜨리면
//   **생성은 되는데 디코드가 영원히 안 되는** 사고가 났다(2026-08-18 clover/boomerang/star/hex).
//   새 실루엣은 shape-registry.js 에만 추가하면 여기로 자동 반영된다.
//   square 는 앵커 배치가 달라 별도 경로이므로 이 목록에서 제외된다(레지스트리가 걸러 준다).
var SHAPE_REG = require('./shape-registry');
var NON_SQUARE_SHAPES = SHAPE_REG.nonSquareDecodable();

/*
 * detectAuto(imageData) — 코드의 픽셀 크기(cellPx)·격자(S/M/L)를 모르는 실사 프레임에서
 *   코어 불스아이로 배율을 역산하고 위치확정까지 한 번에.
 *   1) 스케일 무지 상태로 넓은 반경 FRST → 코어 후보 → pickCoreSeed
 *   2) 코어 링에지 추출(스케일 불변) → 바깥원 반경/5.5 = cellPx
 *   3) 추정 cellPx 로 정식 FRST → locateRobust(orbit+conic 폴백) → 격자 3종 시도, 최소 잔차 채택
 */
function _detectAutoHinted(imageData, dopts){
  // ★2026-09-01 C0 — 힌트 실패 시의 "무힌트 재시도(안전망)"는 넣었다가 뺐다가 09-03 다시 넣었다.
  //   제거 근거였던 실측(netcost.js, 96건=8실루엣×2크롭×6열화, round·boomer 는 애초에 표본에 없었음)은
  //   "힌트가 잃은 해독 0건"이었다. 그 뒤 C7(도트반경 0.42→0.50 + 밀착 아핀/원거리격자 재적합, 09-02)이
  //   들어오면서 전제가 깨졌다 — 09-03 66실루엣 전종 재검증(c0-lab/sweep.js)에서 round·dove·boomer 의
  //   tilt30 3건이 힌트 경로에서만 실패했다(무힌트는 전부 성공). 원인은 원거리격자 재적합
  //   함수(_latticeRefit, 3회 반복)가 아핀 대안과 달리 종횡비 게이트가 없어 힌트 폴백 경로에서
  //   불필요하게 돌면서 11~18초(!)를 쓰고도 틀린 자리에 수렴한 것으로 보인다(진짜 원인은 더 깊이
  //   봐야 하지만, 그 내부를 고치는 대신 이 안전망을 되살리는 쪽이 안전하다 — 실패해도
  //   무힌트 경로가 그대로 답을 찾아준다는 게 이번에도 확인됐다). 아래 detectAuto(얇은 래퍼)가
  //   그 재시도를 담당하고, 이 함수(_detectAutoHinted) 자체는 그대로 둔다.
  var t0 = _now();
  var usePsf = !(dopts && dopts.psf === false);   // 라이브 스캐너는 프레임별로 끄고 주기적으로만 켬
  var deep = !(dopts && dopts.deep === false);    // 극단블러 배율추정 폴백(실패 프레임만 비용 발생)
  var W = imageData.width, H = imageData.height, mn = Math.min(W,H);
  var gray = toGray(imageData);
  var grayD = _denoiseGray(imageData);         // 배율추정 전용(경량 디노이즈)
  // ★2026-09-01 (MASTER LINE C0 = vs-QR P1 1단계) — 코어 힌트로 coarse FRST 생략.
  //   라이브 크롭 경로(scan.js shot())는 프리스크린이 찾은 코어를 **정확히 크롭 중앙에**, cellPx 를
  //   **정확히 CROP_OUT/(2·rMod)** 에 놓고 자른다 — 즉 ① 단계가 찾을 답이 크롭 구성으로 이미 확정돼
  //   있다. 그런데도 coarse FRST 8반경(640px 에서 255ms, 성공 프레임의 41~46% — c0-lab/stage-timing.js
  //   실측)을 매번 돌려 같은 좌표(319.7,319.7)를 다시 찾고 있었다.
  //   ★안전 원칙(SCANNER_NEXT §5 "후보를 제거하지 말고 아는 정보를 넘겨라"): 힌트는 **검증을 통과할
  //   때만** 쓴다 — 힌트 자리에서 extractCoreRings(2ms)로 코어 바깥원을 실측해 cellPx 가 힌트와 15%
  //   안에 들면 그 실측값을 씨앗으로 채택, 아니면 힌트를 버리고 **기존 경로 그대로**(후보 0개 제거).
  //   8/31 "씨앗 힌트"(coarse 가 찾은 좌표를 prepareConic/hug 에 재사용)는 그대로 살아 있다 — 이건
  //   그 앞 단계(coarse 자체)를 프리스크린 좌표로 대체하는 것이라 자리가 다르다.
  var hint = (dopts && dopts.hint && isFinite(dopts.hint.x) && isFinite(dopts.hint.y) && dopts.hint.cellPx > 1.2) ? dopts.hint : null;
  var hinted = false, pkC = null, seed = null, attempts = [], failReason = null, rings = null;
  if (hint && LOC.bullseyeFit(grayD, hint.x, hint.y) >= 0.6) {
    // ★불스아이 템플릿 검증(c0-lab/fit-dist.js 실측 n=85/595: 진짜 코어 최소 0.766, 틀린 자리 최대 0.485).
    //   링 반경만 보면 데이터 도트 영역도 통과해 버려(배터리에서 회전·기울임 프레임의 엉뚱한 힌트가 통과 →
    //   6초 폭주 + 해독 실패) — 템플릿이 그걸 막는다.
    rings = extractCoreRings(grayD, hint.x, hint.y, 0.18*mn, 180);
    if (rings.outer.length >= 20) {
      var roH = rings.outer.map(function(p){ return Math.hypot(p[0]-hint.x, p[1]-hint.y); }).sort(function(a,b){ return a-b; });
      var cH = roH[roH.length>>1] / 5.5;
      if (cH > 1.2 && cH < 60 && Math.abs(cH - hint.cellPx) / hint.cellPx <= 0.15) {
        seed = { x:hint.x, y:hint.y }; pkC = [seed];
        attempts.push({ x:hint.x, y:hint.y, cellPx:cH, src:'hint' });
        hinted = true;
      }
    }
  }
  if (!hinted) {
  // 1) 스케일 무지 coarse 코어탐색
  var coarse = [3,5,7,10,14,19,25,33].filter(function(r){ return r < 0.25*mn; });
  if (coarse.length < 2) coarse = [3,5,7,10];
  var Sc = frst(grayD, coarse, { gradFrac:0.10, alpha:2 });
  pkC = peaks(Sc, { win: Math.max(6, Math.round(0.02*mn)), topK:24, thrFrac:0.05 });
  if (!pkC.length) return { ok:false, reason:'no-core', ms:Math.round(_now()-t0) };
  seed = pickCoreSeed(grayD, pkC, 0.05*mn);
  if (!seed) return { ok:false, reason:'no-core-seed', ms:Math.round(_now()-t0) };
  // 2) 배율추정 — 시도목록(seed,cellPx) 구성.
  //   2a) 빠른 경로: 코어 링에지(각도별 상승에지 2개) 중앙값. 정상~중블러에서 최선(무변경).
  //   2b) deep 폴백: 각도평균 프로파일의 마지막 반값교차(estimateCoreScale). 링에지가
  //       죽는 극단블러(box r6+)에서도 코어 바깥경계가 남아 배율을 준다. 동시에
  //       블롭반경(코어 5.5모듈 vs 위성 2.5모듈)이 "어느 피크가 코어인가"를 가려
  //       pickCoreSeed 가 위성을 고르는 r7+ 도 함께 복구한다.
  rings = extractCoreRings(grayD, seed.x, seed.y, 0.18*mn, 180);
  if (rings.outer.length >= 20) {
    var ro = rings.outer.map(function(p){ return Math.hypot(p[0]-seed.x, p[1]-seed.y); }).sort(function(a,b){ return a-b; });
    var c0 = ro[ro.length>>1] / 5.5;           // 코어 바깥원 = 5.5 모듈
    if (c0 > 1.2 && c0 < 60) attempts.push({ x:seed.x, y:seed.y, cellPx:c0, src:'rings' });
    else failReason = 'bad-scale';
  } else failReason = 'weak-core';
  }
  if (deep) {
    var pool = [seed], pi;
    for (pi = 0; pi < pkC.length && pool.length < 9; pi++) {
      var pk = pkC[pi], dup = false;
      for (var pj = 0; pj < pool.length; pj++) if (Math.hypot(pool[pj].x-pk.x, pool[pj].y-pk.y) < 3) { dup = true; break; }
      if (!dup) pool.push(pk);
    }
    var ests = [], maxCt = 0;
    for (pi = 0; pi < pool.length; pi++) {
      var e = estimateCoreScale(grayD, pool[pi].x, pool[pi].y, 0.18*mn);
      if (!e.ok) continue;
      if (e.contrast > maxCt) maxCt = e.contrast;
      ests.push({ x:pool[pi].x, y:pool[pi].y, cellPx:e.cellPx, rOuter:e.rOuter, contrast:e.contrast, src:'profile' });
    }
    // 저대비 유령블롭 제거 후, 블롭반경 큰 순(코어가 위성보다 2배 이상 크다) 상위 3개.
    ests = ests.filter(function(a){ return a.contrast >= Math.max(12, 0.25*maxCt); })
               .sort(function(a,b){ return b.rOuter - a.rOuter; }).slice(0,3);
    for (pi = 0; pi < ests.length; pi++) {
      var dup2 = false;
      for (var aj = 0; aj < attempts.length; aj++) {
        var A = attempts[aj];
        if (Math.hypot(A.x-ests[pi].x, A.y-ests[pi].y) < 3 && Math.abs(A.cellPx-ests[pi].cellPx)/A.cellPx < 0.08) { dup2 = true; break; }
      }
      if (!dup2) attempts.push(ests[pi]);
    }
  }
  if (!attempts.length) return { ok:false, reason: failReason || 'weak-core', ms:Math.round(_now()-t0) };
  // 3) 정식 검출 + 격자 시도 (시도목록 순서대로).
  //   "잠금 인정" 기준 = ① CRC 해독 성공(=증명) 또는 ② 기하 잔차가 타이트(resid ≤ TIGHT·cellPx).
  //   실측(20시드×블러/노이즈/원근/저해상도): 진짜 잠금 resid/cellPx ≤0.17, 가짜 잠금 ≥0.71 로
  //   깨끗이 갈린다 → 0.5 게이트. 느슨한 잠금은 버리고 다음 배율후보로 계속(오검출 억제).
  if (dopts) dopts.__hinted = hinted;   // 배터리·HUD 확인용(부작용 없음)
  var TIGHT = 0.5;
  var chosen = null, cands = [], cellPx = attempts[0].cellPx, tried = 0;
  var best = null, decoded = null, decShape = 'square', decBridge = false;
  for (var ai = 0; ai < attempts.length && !decoded; ai++) {
    cellPx = attempts[ai].cellPx; tried++;
    var radii = [1.5,2.5,3.5,4.5,5.5].map(function(r){ return r*cellPx; });
    var fo = { gradFrac:0.10, alpha:2 }, po = { win: Math.max(4, Math.round(1.6*cellPx)), topK:30, thrFrac:0.03 };
    var pk2;
    if (attempts[ai].src === 'hint') {
      // ★2026-09-01 C0-b (P1 2단계) — 힌트 경로에선 fine FRST 를 **위성 반경(2.5c) 하나**만 돈다.
      //   5반경 중 1.5/3.5/5.5 는 코어 링을 찾기 위한 것인데 코어는 힌트로 이미 확정됐다.
      //   pk2 의 세 소비처(locate / _hugMeasure / prepareConic)가 코어 밖에서 필요로 하는 건
      //   전부 **위성 원판 피크**뿐이다(_hugMeasure 는 8~66모듈 거리로 코어를 애초에 거른다).
      //   코어는 최고 점수로 맨 앞에 주입한다(locate 의 topH 후보 순회가 첫 항목부터 본다).
      //   ★conic 폴백의 redetect 는 아래에서 **5반경 그대로** 만든다 — 정면화 뒤 재검출은 무변경.
      //   ★반경 3개 [2.0,2.5,3.0]c — 1개(2.5c)만 쓰면 정면은 ×2.97 이지만 **기울임이 C0 보다 후퇴**했다
      //     (tilt30 1113→1490ms): 기울면 위성이 타원(단축 2.5·cosθ)이 돼 한 반경으로는 투표가 흩어진다.
      //     양옆 반경이 그걸 받친다. 코어 링용 1.5/3.5/5.5 는 여전히 뺀다.
      var S1 = frst(gray, [2.0*cellPx, 2.5*cellPx, 3.0*cellPx], fo);
      pk2 = peaks(S1, po);
      var topS = pk2.length ? pk2[0].score : 1;
      pk2 = [{ x: attempts[ai].x, y: attempts[ai].y, score: topS * 2 }].concat(
        pk2.filter(function(p){ return Math.hypot(p.x - attempts[ai].x, p.y - attempts[ai].y) > 4*cellPx; }));
    } else {
      var S2 = frst(gray, radii, fo);
      pk2 = peaks(S2, po);
    }
    var redetect = (function(radii, fo, po){ return function(im){ var g = toGray(im); var s = frst(g, radii, fo); return { gray:g, peaks:peaks(s, po) }; }; })(radii, fo, po);
    // 격자 S/M/L × 앵커모양(사각-코너 / 링) 위치확정 → 후보(잔차 오름차순).
    //   round·heart·clover·boomerang·star·hex 는 앵커가 동일(링) → locate 는 'round'로 1번,
    //   readCode 는 NON_SQUARE_SHAPES 순회로 모양별.
    cands = [];
    // ★2026-08-30 Q1-B — conic 정면화를 6개 layout 이 **공유**한다.
    //   전에는 locateRobust 를 6번 부르며 그때마다 conic 폴백 전체(링추출·conic적합·
    //   워프 약600²·FRST전체)를 다시 했다. attempt 3회까지 곱해 **최대 18회**.
    //   측정으로 확정: 실패 프레임(위성 하나 가림)에서 cands 가 **아예 비고**(6조합 전부
    //   rect-peaks<5) readCode 는 한 번도 안 돈다 — 비용은 전부 여기 있었다.
    //   rectifyHomography 가 layout 에서 쓰는 건 캔버스 크기(dim)뿐이고 6개 layout 의
    //   앵커 반경은 같으므로, **가장 큰 dim 으로 한 번**이면 전부 공유할 수 있다.
    //   ★primary(위성4 locate)가 전부 통과하면 준비 자체를 안 한다 — 성공 프레임 비용 불변.
    var _lays = [];
    ['S','M','L'].forEach(function(grid){
      ['square','round'].forEach(function(sh){ _lays.push({ grid:grid, sh:sh, lay:layout(grid, SPEC, sh) }); });
    });
    var _needConic = false, _maxDist = 0;
    for (var li = 0; li < _lays.length; li++) {
      var _L = _lays[li].lay;
      var _p = LOC.locate(gray, pk2, _L);
      _lays[li].prim = _p;
      if (!(_p.ok && _p.residPx <= 0.45 * cellPx)) _needConic = true;
      var _md = Math.max.apply(null, _L.anchors.slice(1).map(function(a){
        return Math.hypot(a.mx - _L.coreCenter.mx, a.my - _L.coreCenter.my); }));
      if (_md > _maxDist) _maxDist = _md;
    }
    var _prep = _needConic ? prepareConic(imageData, gray, pk2, cellPx, _maxDist, { redetect:redetect }) : null;
    for (var li2 = 0; li2 < _lays.length; li2++) {
      var _E = _lays[li2];
      var res = locateRobustShared(imageData, gray, pk2, _E.lay, { cellPx:cellPx, redetect:redetect }, _prep);
      if (res.ok) {
        cands.push({ grid:_E.grid, anchorShape:_E.sh, lay:_E.lay, res:res });
        // ★sim3orb 는 보조 H(altH)를 함께 준다 — 내삽 재적합(orb)이 실패하는 프레임에서
        //   외삽본(sim)이 오히려 읽히는 경우가 있다(블러가 쌍극 오차를 뭉갠다).
        //   실패 프레임에서만 후보가 하나 느는 것이라 비용이 거의 없다.
        if (res.altH) {
          var _alt = {}; for (var _k in res) _alt[_k] = res[_k];
          _alt.Hmod2img = res.altH; _alt.altH = null; _alt.method = 'sim3alt';
          cands.push({ grid:_E.grid, anchorShape:_E.sh, lay:_E.lay, res:_alt });
        }
      }
    }
    if (!cands.length) {
      // ★C3(2026-09-04): 범용 6레이아웃이 후보를 못 만든 프레임. 밀착 형상에서 위성 하나가 가려지면 정확히
      //   여기로 온다(sim3 가 범용 반경으로는 마름모 위성을 못 찾는다) — 그런데 continue 가 밀착 시도까지
      //   건너뛰어 whale TR/TL 가림이 통째로 죽었다(실측). 비용 게이트 = **정면화(_prep)가 이미 있을 때만**.
      //   실측(09-04): 실제로 회복에 성공한 발화는 전부 prep=true 였고, prep 없는 발화는 전부 헛일이면서
      //   가장 비쌌다(무코드에 가까운 '너무 먼' 프레임 2.05s -> 3.2s). 여기서 정면화를 새로 만들지 않는다.
      //   게이트 후 그 프레임은 최소 2050ms / 중앙 2107ms 로 라이브(2053 / 2107)와 같다. 성공 경로 불변.
      if (!_prep) continue;
      var _hf0 = null, _hs0 = { x: attempts[ai].x, y: attempts[ai].y };
      try { _hf0 = _hugAttempt(imageData, gray, pk2, cellPx, _prep, redetect, usePsf ? [false, true] : [false], _hs0); } catch (e0a) {}
      if (_hf0 && _hf0.dec) {
        decoded = _hf0.dec; best = _hf0.cand; decShape = _hf0.shape; decBridge = _hf0.bridge;
        cands.push(_hf0.cand);
        chosen = { cands:cands, cellPx:cellPx, pk:pk2, prep:_prep, redetect:redetect, maxDist:_maxDist, sx:attempts[ai].x, sy:attempts[ai].y };
        break;
      }
      continue;
    }
    // ★격자/모양 판별 (2026-08-10 수정): residPx 는 판별력이 0 이다 — S/M/L(사각↔링)
    //   앵커 배치가 전부 닮음꼴이라 호모그래피가 스케일·회전을 흡수, 오답 layout 도
    //   같은 물리 4점에 잔차 0.0x 로 들어맞는다(클린 합성서도 동일, 실측 확인).
    //   → 앵커 밖 독립 증거로 순위 결정:
    //   · orbitMatch(궤도 24도트 패턴 일치율): 정답 1.0 / 오답 0.33~0.63 — 주신호.
    //   · scaleErr = |ln(modPx/cellPx)|: H-함의 모듈px vs 코어링 실측 cellPx —
    //     오답 grid 는 ≥0.31, 근사합동 쌍(M-square↔L-round)만 못 가름(궤도가 가름).
    //   · residPx 는 미세 타이브레이커로만.
    cands.forEach(function(c){
      c.scaleErr = c.res.modPx ? Math.abs(Math.log(c.res.modPx / cellPx)) : 1;
      var om = (typeof c.res.orbitMatch === 'number') ? c.res.orbitMatch : 0.5;
      c.score = (1 - om) * 2 + Math.min(c.scaleErr, 1) + c.res.residPx / (10 * cellPx);
    });
    cands.sort(function(a,b){ return a.score - b.score; });

    /* ★★밀착/레거시 **갈라 태우기** (2026-08-30, Fable 실측).
     *   PM 가설은 "①정면화 ②배율반복 ③readCode 폭발 ④궤도재적합" 이었는데
     *   프로파일은 **③만** 맞다고 답했다(정면화 0.6s=5.9%, 재적합은 프로파일에 안 뜸,
     *   밀착 경로 자체는 readCode 2회·254ms). 나머지 7.4초는 전부 **비밀착 레이아웃으로
     *   밀착 코드를 읽으려는 헛수고**였다.
     *   → 그러니 필요한 건 밀착을 싸게 만드는 게 아니라 **먼저 갈래를 정하는 것**이다.
     *
     *   갈래 판정 = **orbitMatch**. 공짜다(이미 후보마다 계산돼 있다). 궤도 24도트는
     *   앵커 밖 독립 증거라, 레이아웃이 프레임을 실제로 설명할 때만 1.0 이 된다.
     *   실측(640px 크롭):
     *     레거시 heart/round/star/square → 최대 om = **1.00** (넷 다)
     *     밀착   heart 0.50 · round 0.75 · star 0.46 · boomerang 0.46
     *   0.90 으로 자르면 둘이 깨끗이 갈린다(여유 0.15).
     *   ★오분류는 **느려질 뿐 틀리지 않는다** — 밀착으로 잘못 갈라져도 아래 1차·2차가
     *     그대로 돌고, 레거시로 잘못 갈라져도 뒤의 무거운 밀착 폴백이 받는다. */
    var _omMax = 0;
    for (var _oi = 0; _oi < cands.length; _oi++) {
      var _ov = cands[_oi].res.orbitMatch;
      if (typeof _ov === 'number' && _ov > _omMax) _omMax = _ov;
    }
    var _hugTried = false;
    if (_omMax < 0.9) {
      _hugTried = true;
      var _hf = null;
      var _hseed = { x: attempts[ai].x, y: attempts[ai].y };
      try { _hf = _hugAttempt(imageData, gray, pk2, cellPx, _prep, redetect, usePsf ? [false, true] : [false], _hseed); } catch (e3) {}
      /* ★싼 원시 시도가 못 읽었고 정면화가 없으면 — 아는 씨앗으로 정면화를 만들어 한 번 더.
         원시 반경은 yaw 20°부터 최대 21% 어긋나 6% 매치 창을 못 넘는다(이 파일 아래 주석).
         정면화가 정답인데 인루프 경로엔 배선이 없었다. clean 은 위 원시 시도에서 이미
         끝나므로 이 블록에 안 들어온다 = 무비용. */
      if ((!_hf || !_hf.dec) && !_prep) {
        var _hprep = null;
        try { _hprep = prepareConic(imageData, gray, pk2, cellPx, _maxDist, { redetect:redetect, seed:_hseed }); } catch (e4) {}
        if (_hprep) { try { var _hf2 = _hugAttempt(imageData, gray, pk2, cellPx, _hprep, redetect, usePsf ? [false, true] : [false], _hseed); if (_hf2) _hf = _hf2; } catch (e5) {} }
      }
      if (_hf && _hf.dec) {
        decoded = _hf.dec; best = _hf.cand; decShape = _hf.shape; decBridge = _hf.bridge;
        cands.push(_hf.cand);
        chosen = { cands:cands, cellPx:cellPx, pk:pk2, prep:_prep, redetect:redetect, maxDist:_maxDist, sx:attempts[ai].x, sy:attempts[ai].y };
        break;
      }
    } else {
      /* ★C10(2026-09-04): om 이 **우연히** 높아 레거시로 갈린 밀착 형상 구제.
         0.9 게이트는 4종(heart/round/star/square)만 보고 정한 값이라 83종에서는 샌다 —
         실측 manse om=1.000 - helmet 0.958 - smile 1.0(C4 가 개별 대응했던 그 건).
         새는 동안 1차 순회 429회를 통째로 헛돌고 나서야 밀착이 2회에 읽는다(4.4s / 4.5s).
         → 갈래를 **핀거리**로 한 번 더 본다. locate 도 readCode 도 없이 피크 기하만 본다.
         83형상 x 5열화 400표본: 밀착 86%가 0.002 안, 가짜 매치는 전부 0.00802 이상(4배 여유).
         문턱을 넘는 것(주로 기울임)은 예전 경로 그대로라 **느려지는 형상이 없다**.
         ★여기서 정면화를 새로 만들지 않는다 — 만들면 핀거리가 큰 프레임에서 정면화 한 번을
         통째로 버린다(실측 safetyboot 2.9s -> 4.7s). 기울어진 프레임은 1차 뒤 기존
         재시도(C4)가 정면화까지 포함해 예전 그대로 받는다.
         실측: manse 4382 -> 524ms - helmet 4546 -> 561ms - 그 외 형상 시간 불변. */
      var _hg = null, _gseed = { x: attempts[ai].x, y: attempts[ai].y };
      try { _hg = _hugAttempt(imageData, gray, pk2, cellPx, _prep, redetect, usePsf ? [false, true] : [false], _gseed, HUG_PIN_D); } catch (e9) {}
      if (_hg && _hg.dec) {
        _hugTried = true;
        decoded = _hg.dec; best = _hg.cand; decShape = _hg.shape; decBridge = _hg.bridge;
        cands.push(_hg.cand);
        chosen = { cands:cands, cellPx:cellPx, pk:pk2, prep:_prep, redetect:redetect, maxDist:_maxDist, sx:attempts[ai].x, sy:attempts[ai].y };
        break;
      }
    }
    // Pass 1: 빠른 표준 셀판독(PSF 끔) — 모든 후보. 정상/약열화는 여기서 끝(빠름).
    //   ★2026-08-16: 다리QR 존 예약(신형) vs 예약없음(구형) 두 레이아웃을 CRC로 판별 — 존은
    //   grid L에서만 정의되므로 그 격자에서만 변형을 추가로 시도한다(다른 격자는 비용 증가 없음).
    for (var i = 0; i < cands.length && !decoded; i++) {
      var shapes = cands[i].anchorShape === 'square' ? ['square'] : NON_SQUARE_SHAPES;
      for (var j = 0; j < shapes.length && !decoded; j++) {
        var variants = [0]; if (cands[i].grid === 'L') { variants.push(1); variants.push(2); variants.push(3); }
        for (var v = 0; v < variants.length; v++) {
          if (bvSkip(variants[v], shapes[j])) continue;
          var lay = (variants[v] === 0 && shapes[j] === cands[i].anchorShape) ? cands[i].lay : layout(cands[i].grid, SPEC, shapes[j], bvOpt(variants[v]));
          var dec = CODEC.readCode(gray, cands[i].res, lay, cellPx, { hue: true, hueBits: 1, rgbaImg: imageData, psf: false });
          if (dec && dec.ok) { decoded = dec; best = cands[i]; decShape = shapes[j]; decBridge = (variants[v] !== 0); chosen = { cands:cands, cellPx:cellPx, pk:pk2, prep:_prep, redetect:redetect, maxDist:_maxDist, sx:attempts[ai].x, sy:attempts[ai].y }; break; }
        }
      }
    }
    /* ★밀착 후보를 **여기서** 만든다 (2026-08-30 재배치).
     *   처음엔 배율 루프 안에 뒀다가 성공 프레임이 1.7~5.3배 느려져 맨 뒤로 뺐다.
     *   그런데 밀착이 **생성 기본값**이 되면서 상황이 뒤집혔다 — 이제 거의 모든 코드가
     *   밀착이라, 맨 뒤 경로만 쓰면 **전부 느린 길로 간다**(실측 0.86s → 6.6s, 7.7배).
     *   사각(밀착 안 함)만 0.63s 로 빨랐던 게 증거다.
     *   → 1차 해독이 실패한 **그 자리에서** 밀착을 시도한다. 밀착 코드는 여기서 끝나고,
     *     밀착이 아닌 코드(사각·구형 발행분)는 이 블록을 그냥 지나친다.
     *   ★chosen 이 아직 없을 수 있으므로 이 배율의 값을 직접 쓴다.
     *
     *   ★★2026-09-03 수리(MASTER LINE C4) — 이 주석이 약속한 재시도가 실제로는 없었다.
     *   omMax>=0.9(레거시 후보가 궤도를 우연히 완벽히 설명)면 위에서 밀착을 아예 안
     *   불러(hugTried=false) 1차 해독도 실패하는데, 여기서 그냥 넘어가 버렸다.
     *   smile/L 이 정확히 이 경우다 — 레거시 궤도합치 1.0이 우연이라 매 시도마다 밀착이
     *   건너뛰어지고, 결국 이 attempts 루프가 다 끝난 뒤의 최후 스윕(무게이트)에서만 찾아
     *   전 열화 ~2.4s(다른 실루엣 대역 ~0.7s)로 느렸다. 핀(phi)은 안 건드린다 — 발행분
     *   보존. 성공 프레임 비용 불변: hugTried 가 이미 true(밀착을 시도했었음)거나
     *   decoded 면 이 블록에 안 들어온다. */
    if (decoded) break;
    if (!_hugTried) {
      var _hf5 = null;
      var _hseed2 = { x: attempts[ai].x, y: attempts[ai].y };
      try { _hf5 = _hugAttempt(imageData, gray, pk2, cellPx, _prep, redetect, usePsf ? [false, true] : [false], _hseed2); } catch (e6) {}
      if ((!_hf5 || !_hf5.dec) && !_prep) {
        var _hprep2 = null;
        try { _hprep2 = prepareConic(imageData, gray, pk2, cellPx, _maxDist, { redetect:redetect, seed:_hseed2 }); } catch (e7) {}
        if (_hprep2) { try { var _hf6 = _hugAttempt(imageData, gray, pk2, cellPx, _hprep2, redetect, usePsf ? [false, true] : [false], _hseed2); if (_hf6) _hf5 = _hf6; } catch (e8) {} }
      }
      if (_hf5 && _hf5.dec) {
        decoded = _hf5.dec; best = _hf5.cand; decShape = _hf5.shape; decBridge = _hf5.bridge;
        cands.push(_hf5.cand);
        chosen = { cands:cands, cellPx:cellPx, pk:pk2, prep:_prep, redetect:redetect, maxDist:_maxDist, sx:attempts[ai].x, sy:attempts[ai].y };
        break;
      }
    }
    if (!chosen) chosen = { cands:cands, cellPx:cellPx, pk:pk2, prep:_prep, redetect:redetect, maxDist:_maxDist, sx:attempts[ai].x, sy:attempts[ai].y };   // 첫 잠금(느슨할 수 있음) 보관
    // 조기종료는 "스케일도 맞는" 타이트 잠금만: residPx 는 오답 cellPx 가정에도 0.0x
    //   로 나와(자기잔차) 잘못된 배율후보에서 탐색을 끊던 구멍을 scaleErr 로 막음.
    /* ★조기종료에 **orbitMatch 게이트**를 건다 (2026-08-30, Fable 권고 + 실측 재현).
     *   residPx 는 자기잔차라 판별력이 0 이고, scaleErr 도 가짜 배율을 다 못 거른다.
     *   실측: foot/L 을 **정확히 원본 크기(816px)** 로 주면 격자를 S 로 오판한 잠금이
     *   조기종료를 걸어 진짜 후보(L)를 못 본다(815·817px 는 정상 — 딱 정수배에서만).
     *   Fable 측정: 가짜 잠금의 orbitMatch 는 0.33~0.667, 진짜는 1.0. 그래서 0.8 로 자른다.
     *   ★orbitMatch 가 없는 경로(궤도를 못 본 경우)는 예전대로 통과시킨다 — 새 실패를 만들지 않기 위해. */
    var _om0 = (typeof cands[0].res.orbitMatch === 'number') ? cands[0].res.orbitMatch : null;
    if (cands[0].res.residPx <= TIGHT * cellPx && cands[0].scaleErr <= 0.25
        && (_om0 === null || _om0 >= 0.8)) { chosen = { cands:cands, cellPx:cellPx, pk:pk2, prep:_prep, redetect:redetect, maxDist:_maxDist, sx:attempts[ai].x, sy:attempts[ai].y }; break; }
  }
  if (!chosen) return { ok:false, reason:'no-lock', cellPx:+cellPx.toFixed(2), tried:tried, ms:Math.round(_now()-t0) };
  cands = chosen.cands; cellPx = chosen.cellPx;
  // Pass 2: 전부 실패(주로 블러) → PSF-ISI 폴백. 잔차 최소가 정답격자가 아닐 수 있어
  //   (블러 하 오답격자가 spurious 저잔차) → CRC가 격자를 판별하도록 후보 순회, 성공시 조기종료.
  if (!decoded && usePsf) {
    for (var i2 = 0; i2 < cands.length && !decoded; i2++) {
      var sh2 = cands[i2].lay && cands[i2].lay.hug ? [cands[i2].anchorShape]
              : (cands[i2].anchorShape === 'square' ? ['square'] : NON_SQUARE_SHAPES);
      var _hug2 = !!(cands[i2].lay && cands[i2].lay.hug);
      for (var j2 = 0; j2 < sh2.length && !decoded; j2++) {
        var variants2 = [0]; if (cands[i2].grid === 'L') { variants2.push(1); variants2.push(2); variants2.push(3); }
        for (var v2 = 0; v2 < variants2.length; v2++) {
          if (bvSkip(variants2[v2], sh2[j2])) continue;
          var lay2 = (variants2[v2] === 0 && sh2[j2] === cands[i2].anchorShape) ? cands[i2].lay : layout(cands[i2].grid, SPEC, sh2[j2], bvOpt(variants2[v2], { hug: _hug2 }));
          var dec2 = CODEC.readCode(gray, cands[i2].res, lay2, cellPx, { hue: true, hueBits: 1, rgbaImg: imageData, psf: true });
          if (dec2 && dec2.ok) { decoded = dec2; best = cands[i2]; decShape = sh2[j2]; decBridge = (variants2[v2] !== 0); break; }
        }
      }
    }
  }
  /* ★밀착 후보 — **PSF 2차까지 전멸한 뒤에만** 만든다 (2026-08-30).
   *   처음엔 배율 후보 루프 **안**에 뒀다가 성공 프레임이 1.7~5.3배 느려졌다
   *   (배율마다 정면화 1회 + 락 3회를 더 했다). 1차 뒤로 내리니 성공 프레임은 회복됐지만
   *   **블러 프레임이 1.4배** 남았다(그건 2차 psf 로 읽히는 정상 코드다).
   *   그래서 2차 뒤로 한 번 더 내렸다 — 밀착 코드는 어차피 기존 레이아웃으로 안 읽히므로
   *   순서를 늦춰도 잃는 게 없고, **기존 코드는 어떤 경로에서도 비용이 0** 이 된다.
   *   비용은 (a) 밀착 코드 (b) 어차피 실패하는 프레임 — 둘 뿐이다.
   *   시그니처 분류로 top≤3 만 넣는다(전수 84 대신) — Fable 측정 top-1 93/96.
   *   정면화가 없으면 여기서 만든다: 원시 프레임 반경은 yaw 20°부터 최대 21% 어긋난다. */
  if (!decoded) {
    try {
      /* ★FIX(Fable 2026-08-30 Q3): 밀착 측정을 chosen 배율 하나에 묶지 않는다.
       *   실패 5건 전부 = 조기종료가 위성 유래 배율(cellPx≈2.7 = 2.5/5.5×6)에 걸려
       *   chosen 이 오염 → 밀착 측정이 엉뚱한 스케일에서 죽었다.
       *   여기서는 chosen 먼저, 안 되면 다른 attempts 배율로도 측정을 재시도한다.
       *   (실패 프레임에서만 도는 코드라 성공 프레임 비용 불변.) */
      var _hscales = [{ cellPx: cellPx, pk: chosen.pk, prep: chosen.prep, redetect: chosen.redetect, maxDist: chosen.maxDist, sx: chosen.sx, sy: chosen.sy }];
      for (var _si = 0; _si < attempts.length; _si++) {
        var _scp = attempts[_si].cellPx, _dupS = false;
        for (var _sj = 0; _sj < _hscales.length; _sj++)
          if (Math.abs(_hscales[_sj].cellPx - _scp) / _hscales[_sj].cellPx < 0.08) { _dupS = true; break; }
        if (!_dupS) _hscales.push({ cellPx: _scp, pk: null, prep: null, redetect: null, maxDist: chosen.maxDist, sx: attempts[_si].x, sy: attempts[_si].y });
      }
      for (var _hsi = 0; _hsi < _hscales.length && !decoded; _hsi++) {
      var _HS = _hscales[_hsi];
      var _hcp = _HS.cellPx, _hpk = _HS.pk, _hrd = _HS.redetect;
      if (!_hpk) {
        var _hradii = [1.5,2.5,3.5,4.5,5.5].map(function(r){ return r*_hcp; });
        var _hfo = { gradFrac:0.10, alpha:2 }, _hpo = { win: Math.max(4, Math.round(1.6*_hcp)), topK:30, thrFrac:0.03 };
        _hpk = peaks(frst(gray, _hradii, _hfo), _hpo);
        _hrd = (function(radii, fo, po){ return function(im){ var g = toGray(im); var s = frst(g, radii, fo); return { gray:g, peaks:peaks(s, po) }; }; })(_hradii, _hfo, _hpo);
      }
      var _hp = _HS.prep;
      var _hsd = (_HS.sx != null) ? { x:_HS.sx, y:_HS.sy } : null;   // ★배율마다 제 씨앗을 갖고 다닌다
      if (!_hp) { try { _hp = prepareConic(imageData, gray, _hpk, _hcp, _HS.maxDist, { redetect:_hrd, seed:_hsd }); } catch (e2) {} }
      var _hm = _hugMeasure(gray, _hpk, _hcp, _hp, _hsd);
      if (_hm) {
        // 마주 보는 짝을 평균해 **두 축 반경**으로 만든다 — 마름모는 짝이 같으므로
        //   평균이 곧 잡음 절반이다(한쪽이 가려져도 반대쪽이 받쳐 준다).
        var _mA = (_hm.r[0] + _hm.r[2]) / 2, _mB = (_hm.r[1] + _hm.r[3]) / 2;
        var _hc = _hugMatch([_mA, _mB], 3);
        for (var hi = 0; hi < _hc.length; hi++) {
          // 같은 쌍을 쓰는 실루엣들은 앵커 배치가 완전히 같다 → **잠금은 한 번**,
          //   형상 판별은 아래 readCode 순회가 CRC 로 한다.
          var _hl = layout(_hc[hi].grid, SPEC, _hc[hi].shape, { hug: true });
          var _hr = _hp ? locateRobustShared(imageData, gray, _hpk, _hl, { cellPx:_hcp, redetect:_hrd }, _hp)
                        : LOC.locate(gray, _hpk, _hl);
          /* ★밀착 잠금에는 **궤도 재적합을 상시 적용**한다 (2026-08-30).
           *   밀착하면 위성이 안으로 들어와 격자 가장자리까지의 외삽 배율이 커진다
           *   (foot/L 짧은 축 16모듈 → 4배). 궤도(8.5)를 끼워 내삽으로 바꾸면 안 자란다.
           *   재적합이 실패하거나 잔차가 나쁘면 함수가 **원래 결과를 그대로** 돌려준다. */
          if (_hr && _hr.ok) {
            var _hr2 = LOC.refitWithOrbit(gray, _hr, _hl, _hcp) || _hr;
            cands.push({ grid:_hc[hi].grid, anchorShape:_hc[hi].shape, lay:_hl,
                         hugShapes:_hc[hi].shapes,
                         res: _hr2.method ? _hr2 : Object.assign({ method:'orbit' }, _hr2) });
            // 재적합본이 실패하는 프레임에서 원본이 읽히는 경우가 있다 — 보조 후보로 같이 넣는다.
            if (_hr2 !== _hr) cands.push({ grid:_hc[hi].grid, anchorShape:_hc[hi].shape, lay:_hl,
                         hugShapes:_hc[hi].shapes,
                         res: Object.assign({}, _hr, { method:'hugraw' }) });
          }
        }
        /* 밀착 후보를 읽는다 — psf 없이 먼저, 안 되면 psf 로(블러 대비).
         * ★부스터(bridge) 변형을 **반드시 같이 돌린다**. 처음 이식할 때 이걸 빠뜨려
         *   밀착 왕복이 152/152 → 72/152 로 떨어졌다. 실패가 전부 grid L 이었던 것이
         *   단서다 — 부스터 자리는 L 에서만 정의되므로, L 코드는 bridge 레이아웃이
         *   아니면 데이터셀 집합 자체가 어긋난다. */
        var _hpsf = usePsf ? [false, true] : [false];
        for (var hp = 0; hp < _hpsf.length && !decoded; hp++) {
          for (var hj = 0; hj < cands.length && !decoded; hj++) {
            if (!(cands[hj].lay && cands[hj].lay.hug)) continue;
            var _hv = [0]; if (cands[hj].grid === 'L') { _hv.push(1); _hv.push(2); _hv.push(3); }
            // ★같은 앵커 배치를 공유하는 실루엣을 전부 시도한다 — 잠금은 이미 끝났고
            //   여기서 드는 건 readCode 뿐이다. CRC 가 어느 형상인지 가른다.
            var _hs = cands[hj].hugShapes || [cands[hj].anchorShape];
            for (var hb = 0; hb < _hv.length && !decoded; hb++) {
              for (var hq = 0; hq < _hs.length && !decoded; hq++) {
                if (bvSkip(_hv[hb], _hs[hq])) continue;
                var _hlay = (_hv[hb] === 0 && _hs[hq] === cands[hj].anchorShape) ? cands[hj].lay
                          : layout(cands[hj].grid, SPEC, _hs[hq], bvOpt(_hv[hb], { hug: true }));
                var _hd = CODEC.readCode(gray, cands[hj].res, _hlay, _hcp, { hue: true, hueBits: 1, rgbaImg: imageData, psf: _hpsf[hp] });
                if (_hd && _hd.ok) { decoded = _hd; best = cands[hj]; decShape = _hs[hq]; decBridge = (_hv[hb] !== 0); }
              }
            }
          }
        }
      }
      }   /* _hscales 루프 끝 */
    } catch (e) { /* 밀착 분류 실패는 무해 — 기존 후보로 계속한다 */ }
  }
  if (!best) best = cands[0];
  // 해독으로 증명되지 않은 느슨한 잠금은 거절(오검출 차단).
  //   스케일 불일치(>0.5 ≈ 65% 어긋남)도 미해독 상태에선 유령 잠금으로 취급.
  if (!decoded && (best.res.residPx > TIGHT * cellPx || best.scaleErr > 0.5))
    return { ok:false, reason:'loose-lock', residPx:+best.res.residPx.toFixed(2), scaleErr:+(best.scaleErr||0).toFixed(3), cellPx:+cellPx.toFixed(2), tried:tried, ms:Math.round(_now()-t0) };
  var r = best.res;
  return { ok:true, method:r.method, core:r.core, corners:r.corners,
           northStar:r.northStar||'TL', residPx:+(r.residPx||0).toFixed(2),
           modPx: r.modPx || null, orbitMatch: (typeof r.orbitMatch === 'number') ? r.orbitMatch : null,
           orbitVariant: r.orbitVariant || null,   // ★C8: 'v2'=현행 궤도 패턴 · 'v1'=2026-09-05 이전 발급분
           scaleErr: +(best.scaleErr || 0).toFixed(3),
           cellPx:+cellPx.toFixed(2), grid:best.grid, shape: decoded ? decShape : null,
           bridge: decoded ? decBridge : null,
           decoded: !!decoded, text: decoded ? decoded.text : null, errors: decoded ? decoded.errors : null,
           bitsPerCell: decoded ? decoded.bitsPerCell : null,
           ecc: decoded ? (decoded.ecc || null) : null,   // 채택된 ECC 비율(신형35%/구형50% 판별 결과)
           hue: (decoded && decoded.hue) ? decoded.hue.text : null,
           ms:Math.round(_now()-t0) };
}

/* detectAuto — 얇은 래퍼. 힌트가 있었는데 실제 해독(decoded)까지 못 갔으면 힌트를 버리고
 * 무힌트로 한 번 더 돈다(2026-09-03 복구, 위 주석 참고). 힌트가 없었거나 힌트로 이미
 * 성공했으면 재시도 없이 그대로 반환 — 성공 프레임의 비용은 0(기존과 동일). */
function detectAuto(imageData, dopts){
  var r = _detectAutoHinted(imageData, dopts);
  if (dopts && dopts.hint && !r.decoded) {
    var dopts2 = {}; for (var k in dopts) dopts2[k] = dopts[k];
    dopts2.hint = null;
    var r2 = _detectAutoHinted(imageData, dopts2);
    if (r2 && r2.decoded) { r2.retriedNoHint = true; dopts.__hinted = 'fallback'; return r2; }
  }
  return r;
}

// renderTestCode — 스캔할 샘플 코드 PNG(dataURL) + 실제 payload 삽입. 브라우저 canvas 필요.
//   hueText 주면 컬러(2luma+1hue) 코드로 렌더 → 스캔 시 루마+색 payload 둘 다 표시.
function renderTestCode(grid, cellPx, text, hueText){
  grid = grid || 'S'; cellPx = cellPx || 6;
  if (text == null) text = '홍익인간 · WIA CODE — 설치 없이 읽히는 차세대 코드';
  if (hueText == null) hueText = '🎨 색에 담긴 추가 데이터 (hue layer)';
  var opts = { grid:grid, cellPx:cellPx, quiet:4, seed:7, ss:2, data:true };
  try {
    var L = layout(grid, SPEC), nC = dataCells(L).length;
    var col = CODEC.encodeColor(text, hueText, nC, 2, 1);   // 2luma + 1hue 컬러코드
    opts.cellGray = col.cellGray; opts.cellChroma = col.cellChroma;
  } catch (e) {
    try { opts.bits = CODEC.encodeToBits(text, dataCells(layout(grid, SPEC)).length).bits; } catch (e2) {}
  }
  var r = render(opts);
  // DOM 캔버스 우선(동기 toDataURL 있음). OffscreenCanvas 는 toDataURL 이 없어 제외.
  if (typeof document === 'undefined')
    return { dataURL:null, width:r.width, height:r.height, text:text, raw:r.img };
  var cv = document.createElement('canvas'); cv.width = r.width; cv.height = r.height;
  var ctx = cv.getContext('2d');
  var id = ctx.createImageData(r.width, r.height); id.data.set(r.img.data); ctx.putImageData(id, 0, 0);
  return { dataURL: cv.toDataURL('image/png'), width:r.width, height:r.height, text:text };
}

// generate(opts) — 임의 payload → WIA 코드 PNG. opts:{text, hueText?, grid, cellPx, bpc}.
//   hueText 있으면 컬러(2luma+1hue), 없으면 흑백/그레이(bpc). 반환 {dataURL,width,height,bytes,...}.
/* ★잘라내기 상자 (2026-08-30, 오너 제안) — 형상 밖 여백을 어디까지 버릴 수 있나.
 *
 *   밀착 전에는 위성이 형상 **밖** 흰 공간에 있어서 그 여백을 못 버렸다.
 *   이제 위성이 안으로 들어왔으니 **형상 잉크 + 위성 + 콰이엇** 까지만 남기면 된다.
 *   자르면 같은 물리 크기에서 모듈이 커진다 = **더 작게 인쇄해도, 더 멀리서도 읽힌다.**
 *   실측: 장미 화면점유 한계 40% → 28%, 비둘기 33% → 28% 로 회복.
 *
 *   ★이득이 적으면 null 을 돌려준다 — 하트·원은 1% 라 자를 값어치가 없다
 *     (오너 지시: "이득 없는 것들은 억지로 자르려 하지 않는다").
 *   ★콰이엇 4모듈은 반드시 남긴다. 없으면 배경과 안 갈린다.
 *   ★반환은 **픽셀 좌표**다. 자르는 건 호출부가 **합성 마지막에** 한다 —
 *     부스터 QR 좌표가 원본 기준이라, 먼저 자르면 그게 어긋난다. */
function cropRectOf(L, cellPx, quiet, minGain, outline){
  if (!L || !L.anchors) return null;
  var N = L.N, Q = quiet;
  // ★스트로크는 실루엣 **밖** (gap+w) 모듈까지 나간다 — 크롭 박스를 그만큼 넓히지 않으면 잘린다.
  var OUT = (outline && outline.w > 0) ? ((outline.gap != null ? outline.gap : 1) + outline.w) : 0;
  var mnx = 1e9, mxx = -1e9, mny = 1e9, mxy = -1e9, any = false;
  for (var y = 0; y < N; y++) for (var x = 0; x < N; x++) {
    if (!GEO.insideShape(x + 0.5, y + 0.5, L)) continue;
    any = true;
    if (x < mnx) mnx = x; if (x > mxx) mxx = x;
    if (y < mny) mny = y; if (y > mxy) mxy = y;
  }
  if (!any) return null;
  if (OUT) { mnx -= OUT; mxx += OUT; mny -= OUT; mxy += OUT; }
  var SR = (SPEC.satRadius || 2.5) + ((L.hug && SPEC.satHalo) ? SPEC.satHalo : 0);
  for (var i = 1; i < L.anchors.length; i++) {
    var a = L.anchors[i];
    if (a.mx - SR < mnx) mnx = a.mx - SR; if (a.mx + SR > mxx) mxx = a.mx + SR;
    if (a.my - SR < mny) mny = a.my - SR; if (a.my + SR > mxy) mxy = a.my + SR;
  }
  if (L.bridgeRect && L.bridgeRect.inner) {           // 부스터 QR 자리도 담는다
    var b = L.bridgeRect.inner;
    if (b.mx0 < mnx) mnx = b.mx0; if (b.mx1 > mxx) mxx = b.mx1;
    if (b.my0 < mny) mny = b.my0; if (b.my1 > mxy) mxy = b.my1;
  }
  var side = Math.max(mxx - mnx, mxy - mny) + 2 * Q;
  var full = N + 2 * Q, gain = full / side;
  if (!(gain >= (minGain || 1.05))) return null;      // 자를 값어치가 없다
  var ccx = (mnx + mxx) / 2, ccy = (mny + mxy) / 2;
  var W = full * cellPx, w = Math.round(side * cellPx);
  var x0 = Math.round((ccx - side / 2 + Q) * cellPx), y0 = Math.round((ccy - side / 2 + Q) * cellPx);
  if (x0 < 0) x0 = 0; if (y0 < 0) y0 = 0;
  if (x0 + w > W) x0 = W - w; if (y0 + w > W) y0 = W - w;
  if (w <= 0 || w >= W) return null;
  // ★full = 원본 캔버스 폭(px). 미리보기는 더 작게 굽는 별도 패스가 있어
  //   호출부가 비율 환산을 해야 한다 — 그때 기준이 이 값이다.
  return { x: x0, y: y0, w: w, h: w, full: W, gain: +gain.toFixed(3) };
}

function generate(opts){
  opts = opts || {};
  var grid = opts.grid || 'M', cellPx = opts.cellPx || 8, text = opts.text || '', bpc = opts.bpc || 2, shape = opts.shape || 'square';
  var bridge = !!opts.bridge;
  // ★2026-08-27 §7-36 방어 ②-b — **생성기는 모르는 실루엣을 조용히 받으면 안 된다.**
  //   디코더는 못 읽는 실루엣을 깨끗한 false 로 넘겨야 하지만(방어 ②), 생성기가 같은 관용을
  //   부리면 "장미를 달라"는 요청에 사각을 내주고도 성공이라 답한다(실측: 유령 이름으로
  //   생성이 그냥 됐다). 로스터에 없는 이름은 여기서 **깨끗한 오류**로 되돌린다.
  //   ★2026-08-27 재발견: 이 가드는 원래 **생성물(wiascan-core.js)에만 손편집**돼 있었다 —
  //   'node build-core.js' 를 한 번만 돌려도 조용히 사라질 상태였다(유령 로스터를 막으려고
  //   만든 가드가 자기도 같은 지뢰 위에 앉아 있었던 셈). 소스인 여기로 옮겨 근치한다.
  //   ★2026-08-29 추가 — 위 목록은 **해독 대상**이라 legacy(생성 중단분)까지 포함한다.
  //   그걸로 생성을 판정하면 생성 중단한 실루엣이 계속 만들어진다(실측: bubble 이 그랬다).
  //   생성 가부는 레지스트리의 status='current' 로만 판정한다.
  if (shape !== 'square' && shape !== 'custom' && NON_SQUARE_SHAPES.indexOf(shape) < 0) {
    return { error: 'unknown shape: ' + shape };
  }
  if (shape !== 'square' && shape !== 'custom' && !SHAPE_REG.isCurrent(shape)) {
    return { error: 'retired shape (decode-only): ' + shape };
  }
  var quiet = 4;
  // 'custom' 실루엣(사용자 이미지 마스크) — layout·render 양쪽에 같은 마스크를 넘겨야
  //   셀 집합(dataCells)과 픽셀(render)이 일치한다. 자세한 제약은 geometry.js insideShape 주석.
  var customMask = opts.customMask || null;
  /* ★밀착 기본값 — 레지스트리 HUG_ON 에 켜진 실루엣만 (2026-08-30).
   *   ※이 파일은 JS **문자열 생성기**라 주석에 백틱을 쓰면 템플릿 리터럴이 깨진다.
   *   opts.hug 를 명시하면 그게 이긴다(시험·비교용). 안 주면 레지스트리가 정한다.
   *   되돌리기는 shape-registry.js 의 HUG_ON 한 줄이다. */
  var _hugDflt = !!(SHAPE_REG.hugOn && SHAPE_REG.hugOn(shape));
  var _hug = (opts.hug !== undefined) ? !!opts.hug : _hugDflt;
  /* ★opts.bridgeLegacy (2026-09-02) — 옛 다리QR 배치(가로중앙 topMy 77.5 · pad 1.4)로 만든다.
   *   A/B 비교 페이지와 호환 시험 전용이다. 평상시 생성 경로는 이 값을 주지 않는다. */
  var _bleg = !!opts.bridgeLegacy;
  var _nog = !!opts.noGrooves;   // 골 적용 전 비교용(랩 전용)
  var L = layout(grid, SPEC, shape, { bridge: bridge, bridgeLegacy: _bleg, customMask: customMask, hug: _hug, noGrooves: _nog }), nC = dataCells(L).length;
  // ★outline: 외곽 스트로크(인지 보강, MASTER LINE B2). 없으면 렌더 픽셀 불변 — 순수 옵트인.
  //   opts.outline===true 면 레지스트리의 형상별 핀을 쓰고, {gap,w} 를 직접 주면 그대로 쓴다.
  var _outline = null;
  if (opts.outline) {
    _outline = (opts.outline === true)
      ? (SHAPE_REG.outlinePin ? SHAPE_REG.outlinePin(shape) : null)   // ★전역이 아니라 빌드시 주입되는 SHAPE_REG
      : opts.outline;
    // B2-d 무지개 외곽선(2026-09-04) — 색만 입힌다. 기하(gap,w)는 그대로라 크롭·해독 경로에 영향 없음.
    //   opts.outlineRainbow === true 면 엔진 기본 밝기 상한, 객체면 {maxY} 를 그대로 넘긴다.
    if (_outline && opts.outlineRainbow) {
      var _rb = (opts.outlineRainbow === true) ? {} : opts.outlineRainbow;
      // 형상별 밝기 상한 핀(레지스트리). 호출자가 maxY 를 직접 주면 그게 우선한다(실험용).
      if (_rb.maxY == null && SHAPE_REG.rainbowMaxY) {
        var _my = SHAPE_REG.rainbowMaxY(shape);
        if (_my != null) { var _c = {}; for (var _k in _rb) _c[_k] = _rb[_k]; _c.maxY = _my; _rb = _c; }
      }
      _outline = { gap: _outline.gap, w: _outline.w, rainbow: _rb };
    }
  }
  var ropts = { grid:grid, shape:shape, cellPx:cellPx, quiet:quiet, seed:7, ss:2, data:true, bridge:bridge, bridgeLegacy: _bleg, customMask:customMask, hug: _hug, noGrooves: _nog, outline: _outline }, bytes = 0, capBytes = 0;
  try {
    if (opts.hueText) { var col = CODEC.encodeColor(text, opts.hueText, nC, bpc, 1); ropts.cellGray = col.cellGray; ropts.cellChroma = col.cellChroma; bytes = text.length + opts.hueText.length; capBytes = col.lumaCapBytes + col.hueCapBytes; }
    else if (bpc === 1) { var e1 = CODEC.encodeToBits(text, nC); ropts.bits = e1.bits; bytes = (e1.usedBytes != null ? e1.usedBytes - 6 : text.length); capBytes = e1.capBytes - 6; }   // ★P3: v2 는 글자수≠바이트
    else { var e = CODEC.encodeToCells(text, nC, bpc); ropts.cellGray = e.cellGray; bytes = text.length; capBytes = e.capBytes - 6; }
  } catch (err) { return { error: String((err && err.message) || err) }; }
  var r = render(ropts);
  // 다리QR 존 픽셀 좌표(모듈→픽셀은 quiet·cellPx를 아는 여기서만 변환 — geometry.js는 모듈단위만 다룸).
  var bridgeOut = null;
  if (L.bridgeRect) {
    bridgeOut = {
      x0: (L.bridgeRect.inner.mx0 + quiet) * cellPx,
      y0: (L.bridgeRect.inner.my0 + quiet) * cellPx,
      qpx: GEO.BRIDGE.qMod * cellPx,
      padPx: (_bleg ? GEO.BRIDGE_LEGACY.padQrMod : GEO.BRIDGE.padQrMod) * cellPx,
      qrModules: GEO.BRIDGE.qrModules,
    };
  }
  var cropOut = cropRectOf(L, cellPx, quiet, opts.cropMinGain, _outline);
  if (typeof document === 'undefined') return { dataURL:null, img:r.img, width:r.width, height:r.height, bytes:bytes, capBytes:capBytes, bridge:bridgeOut, crop:cropOut };
  // ★raw (2026-08-29) — 호출자가 캔버스에 바로 얹을 거면 PNG 를 거치지 않는다.
  //   기존 경로는 픽셀을 이미 다 만들어 놓고 **PNG 로 압축**했고, 받는 쪽은 <img> 로
  //   **다시 풀어서** 그린 뒤 부스터 QR 을 얹으려 getImageData 로 **또 읽어냈다**.
  //   압축·해제·읽기·쓰기가 전부 군더더기였다(카드 한 장당 실측 60ms 안팎).
  //   덤으로 img.onload 가 사라져 **동기**가 된다 — 그리기 완료 시점이 애매하지 않다.
  if (opts.raw) return { dataURL:null, img:r.img, width:r.width, height:r.height, grid:grid, shape:shape, bpc:bpc, color:!!opts.hueText, bytes:bytes, capBytes:capBytes, bridge:bridgeOut, crop:cropOut };
  var cv = document.createElement('canvas'); cv.width = r.width; cv.height = r.height;
  var ctx = cv.getContext('2d'); var id = ctx.createImageData(r.width, r.height); id.data.set(r.img.data); ctx.putImageData(id, 0, 0);
  return { dataURL: cv.toDataURL('image/png'), width:r.width, height:r.height, grid:grid, shape:shape, bpc:bpc, color:!!opts.hueText, bytes:bytes, capBytes:capBytes, bridge:bridgeOut, crop:cropOut };
}

var G = (typeof window!=='undefined') ? window : globalThis;
// ★2026-08-28 — 엔진도 실루엣 정본을 노출한다(shapes).
//   생성기는 <script src="/orbit/shape-registry.js"> 로 먼저 읽지만, 그 한 줄이 빠지거나
//   캐시로 못 받으면 로스터가 **조용히 빈다**. 엔진에 이미 같은 파일이 번들돼 있으므로
//   두 번째 경로로 내준다 — 출처가 같은 파일이라 두 경로가 어긋날 수 없다.
G.WiaScan = { ready: Promise.resolve(true), detectAuto: detectAuto, renderTestCode: renderTestCode, generate: generate,
  shapes: require('./shape-registry'), _version:'proto-3-gen' };

})();
