/**
 * WIHP Engine - IPA to Hangul Conversion
 * WIA International Hangul Phonology
 *
 * 홍익인간 (弘益人間) - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

const WIHPEngine = {
  // IPA Consonants → Hangul
  consonants: {
    // Plosives
    'p': 'ㅍ', 'b': 'ㅂ', 't': 'ㅌ', 'd': 'ㄷ', 'k': 'ㅋ', 'g': 'ㄱ',
    'ʈ': 'ㅌ', 'ɖ': 'ㄷ', 'c': 'ㅊ', 'ɟ': 'ㅈ', 'q': 'ㅋ', 'ɢ': 'ㄱ',
    'ʔ': 'ㅇ',

    // Nasals
    'm': 'ㅁ', 'ɱ': 'ㅁ', 'n': 'ㄴ', 'ɳ': 'ㄴ', 'ɲ': 'ㄴ', 'ŋ': 'ㅇ', 'ɴ': 'ㅇ',

    // Trills & Taps
    'ʙ': 'ㅂ', 'r': 'ㄹ', 'ʀ': 'ㄹ', 'ⱱ': 'ㅂ', 'ɾ': 'ㄹ', 'ɽ': 'ㄹ',

    // Fricatives
    'ɸ': 'ㅍ', 'β': 'ㅂ', 'f': 'ㅍ', 'v': 'ㅂ',
    'θ': 'ㅅ', 'ð': 'ㄷ', 's': 'ㅅ', 'z': 'ㅈ',
    'ʃ': 'ㅅ', 'ʒ': 'ㅈ', 'ʂ': 'ㅅ', 'ʐ': 'ㅈ',
    'ç': 'ㅎ', 'ʝ': 'ㅈ', 'x': 'ㅎ', 'ɣ': 'ㄱ',
    'χ': 'ㅎ', 'ʁ': 'ㄹ', 'ħ': 'ㅎ', 'ʕ': 'ㅇ',
    'h': 'ㅎ', 'ɦ': 'ㅎ',

    // Affricates
    'tʃ': 'ㅊ', 'dʒ': 'ㅈ', 'ts': 'ㅈ', 'dz': 'ㅈ',
    'tɕ': 'ㅊ', 'dʑ': 'ㅈ',

    // Approximants
    'ʋ': 'ㅂ', 'ɹ': 'ㄹ', 'ɻ': 'ㄹ', 'j': 'ㅇ', 'ɰ': 'ㅇ',
    'l': 'ㄹ', 'ɭ': 'ㄹ', 'ʎ': 'ㄹ', 'ʟ': 'ㄹ',
    'w': 'ㅇ', 'ʍ': 'ㅎ', 'ɥ': 'ㅇ',

    // Lateral fricatives
    'ɬ': 'ㄹ', 'ɮ': 'ㄹ',

    // Ejectives (approximate)
    "p'": 'ㅃ', "t'": 'ㄸ', "k'": 'ㄲ', "s'": 'ㅆ',
  },

  // IPA Vowels → Hangul
  vowels: {
    // Close
    'i': 'ㅣ', 'y': 'ㅟ', 'ɨ': 'ㅡ', 'ʉ': 'ㅜ', 'ɯ': 'ㅡ', 'u': 'ㅜ',

    // Near-close
    'ɪ': 'ㅣ', 'ʏ': 'ㅟ', 'ʊ': 'ㅜ',

    // Close-mid
    'e': 'ㅔ', 'ø': 'ㅚ', 'ɘ': 'ㅓ', 'ɵ': 'ㅗ', 'ɤ': 'ㅓ', 'o': 'ㅗ',

    // Mid
    'ə': 'ㅓ', 'ɚ': 'ㅓ',

    // Open-mid
    'ɛ': 'ㅔ', 'œ': 'ㅚ', 'ɜ': 'ㅓ', 'ɞ': 'ㅓ', 'ʌ': 'ㅓ', 'ɔ': 'ㅗ',

    // Near-open
    'æ': 'ㅐ', 'ɐ': 'ㅏ',

    // Open
    'a': 'ㅏ', 'ɶ': 'ㅚ', 'ɑ': 'ㅏ', 'ɒ': 'ㅗ',

    // Nasalized (approximate)
    'ã': 'ㅏ', 'ẽ': 'ㅔ', 'ĩ': 'ㅣ', 'õ': 'ㅗ', 'ũ': 'ㅜ',
  },

  // Diphthongs
  diphthongs: {
    'aɪ': 'ㅏㅣ', 'aʊ': 'ㅏㅜ', 'eɪ': 'ㅔㅣ', 'oʊ': 'ㅗㅜ',
    'ɔɪ': 'ㅗㅣ', 'ɪə': 'ㅣㅓ', 'ʊə': 'ㅜㅓ', 'eə': 'ㅔㅓ',
    'əʊ': 'ㅓㅜ',
    'ju': 'ㅠ', 'jɑ': 'ㅑ', 'jʌ': 'ㅕ', 'jo': 'ㅛ',
    'wi': 'ㅟ', 'wɑ': 'ㅘ', 'wʌ': 'ㅝ', 'we': 'ㅞ',
  },

  // Common words lookup (for better accuracy)
  wordLookup: {
    // English
    'hello': '헬로', 'world': '월드', 'thank': '땡크', 'you': '유',
    'good': '굿', 'morning': '모닝', 'night': '나잇', 'bye': '바이',
    'please': '플리즈', 'sorry': '쏘리', 'love': '러브', 'friend': '프렌드',
    'water': '워터', 'food': '푸드', 'coffee': '커피', 'tea': '티',
    'yes': '예스', 'no': '노', 'okay': '오케이', 'beautiful': '뷰티풀',
    'phonology': '퍼날러지', 'language': '랭귀지', 'korea': '코리아',
    'japan': '저팬', 'china': '차이나', 'america': '어메리카',

    // Japanese (Romaji)
    'konnichiwa': '곤니치와', 'arigatou': '아리가또', 'sayonara': '사요나라',
    'ohayou': '오하요', 'oyasumi': '오야스미', 'sumimasen': '스미마셍',
    'kawaii': '카와이', 'sugoi': '스고이', 'baka': '바카',

    // Spanish
    'hola': '올라', 'gracias': '그라시아스', 'adios': '아디오스',
    'buenos': '부에노스', 'dias': '디아스', 'noches': '노체스',
    'amigo': '아미고', 'amor': '아모르',

    // French
    'bonjour': '봉주르', 'merci': '메르시', 'bonsoir': '봉수아',
    'aurevoir': '오르부아', 'croissant': '크루아상', 'cafe': '카페',

    // German
    'guten': '구텐', 'morgen': '모르겐', 'tag': '탁', 'danke': '당케',
    'bitte': '비테', 'schon': '쇤',

    // Chinese (Pinyin)
    'nihao': '니하오', 'xiexie': '셰셰', 'zaijian': '짜이찌앤',
    'beijing': '베이징', 'shanghai': '상하이',

    // Arabic (Romanized)
    'marhaba': '마르하바', 'shukran': '슈크란', 'salam': '살람',
    'inshallah': '인샬라', 'habibi': '하비비',

    // Russian (Romanized)
    'privet': '프리벳', 'spasibo': '스파시바', 'dosvidaniya': '다스비다니야',
    'da': '다', 'net': '넷',
  },

  // Hangul composition
  CHOSUNG: ['ㄱ','ㄲ','ㄴ','ㄷ','ㄸ','ㄹ','ㅁ','ㅂ','ㅃ','ㅅ','ㅆ','ㅇ','ㅈ','ㅉ','ㅊ','ㅋ','ㅌ','ㅍ','ㅎ'],
  JUNGSUNG: ['ㅏ','ㅐ','ㅑ','ㅒ','ㅓ','ㅔ','ㅕ','ㅖ','ㅗ','ㅘ','ㅙ','ㅚ','ㅛ','ㅜ','ㅝ','ㅞ','ㅟ','ㅠ','ㅡ','ㅢ','ㅣ'],
  JONGSUNG: ['','ㄱ','ㄲ','ㄳ','ㄴ','ㄵ','ㄶ','ㄷ','ㄹ','ㄺ','ㄻ','ㄼ','ㄽ','ㄾ','ㄿ','ㅀ','ㅁ','ㅂ','ㅄ','ㅅ','ㅆ','ㅇ','ㅈ','ㅊ','ㅋ','ㅌ','ㅍ','ㅎ'],

  /**
   * Compose Hangul syllable from jamo
   */
  composeHangul(cho, jung, jong = '') {
    const choIdx = this.CHOSUNG.indexOf(cho);
    const jungIdx = this.JUNGSUNG.indexOf(jung);
    const jongIdx = this.JONGSUNG.indexOf(jong);

    if (choIdx < 0 || jungIdx < 0) return cho + jung + jong;

    const code = 0xAC00 + (choIdx * 21 * 28) + (jungIdx * 28) + (jongIdx > 0 ? jongIdx : 0);
    return String.fromCharCode(code);
  },

  /**
   * Convert IPA string to Hangul jamo sequence
   */
  ipaToJamo(ipa) {
    let result = '';
    let i = 0;

    // Clean IPA string
    ipa = ipa.replace(/[\/\[\]ˈˌːˑ̩̯̃ʰʷʲˠˤⁿˡ]/g, '').trim();

    while (i < ipa.length) {
      let matched = false;

      // Try diphthongs first (2-3 chars)
      for (let len = 3; len >= 2; len--) {
        if (i + len <= ipa.length) {
          const segment = ipa.substring(i, i + len);
          if (this.diphthongs[segment]) {
            result += this.diphthongs[segment];
            i += len;
            matched = true;
            break;
          }
          if (this.consonants[segment]) {
            result += this.consonants[segment];
            i += len;
            matched = true;
            break;
          }
        }
      }

      if (!matched) {
        const char = ipa[i];
        if (this.consonants[char]) {
          result += this.consonants[char];
        } else if (this.vowels[char]) {
          result += this.vowels[char];
        }
        i++;
      }
    }

    return result;
  },

  /**
   * Compose jamo sequence into syllables
   */
  jamoToSyllables(jamo) {
    let result = '';
    let buffer = { cho: '', jung: '', jong: '' };
    let state = 'cho'; // 'cho', 'jung', 'jong'

    const isChosung = (c) => this.CHOSUNG.includes(c);
    const isJungsung = (c) => this.JUNGSUNG.includes(c);
    const isJongsung = (c) => this.JONGSUNG.includes(c) && c !== '';

    for (let i = 0; i < jamo.length; i++) {
      const c = jamo[i];

      if (state === 'cho') {
        if (isChosung(c)) {
          buffer.cho = c;
          state = 'jung';
        } else if (isJungsung(c)) {
          buffer.cho = 'ㅇ';
          buffer.jung = c;
          state = 'jong';
        }
      } else if (state === 'jung') {
        if (isJungsung(c)) {
          buffer.jung = c;
          state = 'jong';
        } else if (isChosung(c)) {
          // No vowel found, output consonant and restart
          result += buffer.cho;
          buffer = { cho: c, jung: '', jong: '' };
          state = 'jung';
        }
      } else if (state === 'jong') {
        if (isChosung(c) && isJongsung(c)) {
          // Look ahead
          const next = jamo[i + 1];
          if (next && isJungsung(next)) {
            // This consonant is cho of next syllable
            result += this.composeHangul(buffer.cho, buffer.jung, buffer.jong);
            buffer = { cho: c, jung: '', jong: '' };
            state = 'jung';
          } else {
            // This is jong
            buffer.jong = c;
          }
        } else if (isJungsung(c)) {
          // New syllable starting with vowel
          result += this.composeHangul(buffer.cho, buffer.jung, buffer.jong);
          buffer = { cho: 'ㅇ', jung: c, jong: '' };
          state = 'jong';
        } else if (isChosung(c)) {
          result += this.composeHangul(buffer.cho, buffer.jung, buffer.jong);
          buffer = { cho: c, jung: '', jong: '' };
          state = 'jung';
        }
      }
    }

    // Flush remaining buffer
    if (buffer.cho && buffer.jung) {
      result += this.composeHangul(buffer.cho, buffer.jung, buffer.jong);
    } else if (buffer.cho) {
      result += buffer.cho;
    }

    return result;
  },

  /**
   * Convert any text to WIHP Hangul
   */
  convert(text, lang = 'en') {
    text = text.toLowerCase().trim();

    // Check word lookup first
    if (this.wordLookup[text]) {
      return this.wordLookup[text];
    }

    // Check if it's IPA (contains IPA characters)
    const hasIPA = /[əɪʊɛɔæɑʌɒŋθðʃʒɹɾɻʔɥɰɲɳɴɱʙʀⱱɽɸβçʝχʁħʕɦɬɮʎʟɨʉɯɵɘɜɞɐɶœʏɒ]/.test(text);

    if (hasIPA) {
      const jamo = this.ipaToJamo(text);
      return this.jamoToSyllables(jamo);
    }

    // Convert romanization to approximate IPA then to Hangul
    const jamo = this.romanToJamo(text);
    return this.jamoToSyllables(jamo);
  },

  /**
   * Convert romanization to jamo (simplified)
   */
  romanToJamo(text) {
    const romanMap = {
      // Consonants
      'ch': 'ㅊ', 'sh': 'ㅅ', 'th': 'ㅅ', 'ph': 'ㅍ', 'ng': 'ㅇ',
      'ts': 'ㅈ', 'dz': 'ㅈ', 'zh': 'ㅈ', 'kh': 'ㅋ', 'gh': 'ㄱ',
      'ck': 'ㅋ', 'qu': 'ㅋㅜ', 'x': 'ㅋㅅ', 'c': 'ㅋ',
      'p': 'ㅍ', 'b': 'ㅂ', 't': 'ㅌ', 'd': 'ㄷ', 'k': 'ㅋ', 'g': 'ㄱ',
      'f': 'ㅍ', 'v': 'ㅂ', 's': 'ㅅ', 'z': 'ㅈ', 'j': 'ㅈ',
      'm': 'ㅁ', 'n': 'ㄴ', 'l': 'ㄹ', 'r': 'ㄹ', 'h': 'ㅎ',
      'w': 'ㅜ', 'y': 'ㅣ',

      // Vowels
      'ee': 'ㅣ', 'ea': 'ㅣ', 'oo': 'ㅜ', 'ou': 'ㅜ', 'au': 'ㅗ',
      'ai': 'ㅐ', 'ay': 'ㅐ', 'ei': 'ㅔ', 'ey': 'ㅔ',
      'oi': 'ㅗㅣ', 'oy': 'ㅗㅣ', 'ow': 'ㅗㅜ',
      'a': 'ㅏ', 'e': 'ㅔ', 'i': 'ㅣ', 'o': 'ㅗ', 'u': 'ㅜ',
    };

    let result = '';
    let i = 0;

    while (i < text.length) {
      let matched = false;

      // Try longer patterns first
      for (let len = 2; len >= 1; len--) {
        if (i + len <= text.length) {
          const segment = text.substring(i, i + len);
          if (romanMap[segment]) {
            result += romanMap[segment];
            i += len;
            matched = true;
            break;
          }
        }
      }

      if (!matched) {
        // Skip unknown characters
        i++;
      }
    }

    return result;
  },

  /**
   * Get Braille representation
   */
  toBraille(hangul) {
    const brailleMap = {
      // Cho
      'ㄱ': '⠈', 'ㄴ': '⠉', 'ㄷ': '⠊', 'ㄹ': '⠐', 'ㅁ': '⠑',
      'ㅂ': '⠘', 'ㅅ': '⠠', 'ㅇ': '⠛', 'ㅈ': '⠨', 'ㅊ': '⠰',
      'ㅋ': '⠋', 'ㅌ': '⠓', 'ㅍ': '⠙', 'ㅎ': '⠚',
      // Jung
      'ㅏ': '⠣', 'ㅓ': '⠎', 'ㅗ': '⠥', 'ㅜ': '⠍', 'ㅡ': '⠪', 'ㅣ': '⠕',
      'ㅐ': '⠗', 'ㅔ': '⠝', 'ㅚ': '⠽', 'ㅟ': '⠍⠗',
      'ㅑ': '⠜', 'ㅕ': '⠱', 'ㅛ': '⠬', 'ㅠ': '⠩',
      'ㅘ': '⠧', 'ㅝ': '⠏', 'ㅞ': '⠏⠗', 'ㅢ': '⠪⠕',
    };

    let result = '';
    for (const char of hangul) {
      if (brailleMap[char]) {
        result += brailleMap[char];
      } else if (char >= '가' && char <= '힣') {
        // Decompose syllable
        const code = char.charCodeAt(0) - 0xAC00;
        const cho = Math.floor(code / (21 * 28));
        const jung = Math.floor((code % (21 * 28)) / 28);
        const jong = code % 28;

        result += brailleMap[this.CHOSUNG[cho]] || '';
        result += brailleMap[this.JUNGSUNG[jung]] || '';
        if (jong > 0) {
          result += brailleMap[this.JONGSUNG[jong]] || '';
        }
      }
    }
    return result;
  }
};

// Export for Node.js / Electron
module.exports = WIHPEngine;
