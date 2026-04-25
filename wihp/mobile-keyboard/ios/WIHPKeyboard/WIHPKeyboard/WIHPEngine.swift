import Foundation

/**
 * WIHP Engine - IPA to Hangul Conversion
 * WIA International Hangul Phonology
 *
 * 홍익인간 (弘益人間) - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

class WIHPEngine {
    static let shared = WIHPEngine()

    // IPA Consonants → Hangul
    private let consonants: [String: String] = [
        // Plosives
        "p": "ㅍ", "b": "ㅂ", "t": "ㅌ", "d": "ㄷ", "k": "ㅋ", "g": "ㄱ",
        "ʈ": "ㅌ", "ɖ": "ㄷ", "c": "ㅊ", "ɟ": "ㅈ", "q": "ㅋ", "ɢ": "ㄱ",
        "ʔ": "ㅇ",

        // Nasals
        "m": "ㅁ", "ɱ": "ㅁ", "n": "ㄴ", "ɳ": "ㄴ", "ɲ": "ㄴ", "ŋ": "ㅇ", "ɴ": "ㅇ",

        // Trills & Taps
        "ʙ": "ㅂ", "r": "ㄹ", "ʀ": "ㄹ", "ⱱ": "ㅂ", "ɾ": "ㄹ", "ɽ": "ㄹ",

        // Fricatives
        "ɸ": "ㅍ", "β": "ㅂ", "f": "ㅍ", "v": "ㅂ",
        "θ": "ㅅ", "ð": "ㄷ", "s": "ㅅ", "z": "ㅈ",
        "ʃ": "ㅅ", "ʒ": "ㅈ", "ʂ": "ㅅ", "ʐ": "ㅈ",
        "ç": "ㅎ", "ʝ": "ㅈ", "x": "ㅎ", "ɣ": "ㄱ",
        "χ": "ㅎ", "ʁ": "ㄹ", "ħ": "ㅎ", "ʕ": "ㅇ",
        "h": "ㅎ", "ɦ": "ㅎ",

        // Affricates
        "tʃ": "ㅊ", "dʒ": "ㅈ", "ts": "ㅈ", "dz": "ㅈ",
        "tɕ": "ㅊ", "dʑ": "ㅈ",

        // Approximants
        "ʋ": "ㅂ", "ɹ": "ㄹ", "ɻ": "ㄹ", "j": "ㅇ", "ɰ": "ㅇ",
        "l": "ㄹ", "ɭ": "ㄹ", "ʎ": "ㄹ", "ʟ": "ㄹ",
        "w": "ㅇ", "ʍ": "ㅎ", "ɥ": "ㅇ",

        // Lateral fricatives
        "ɬ": "ㄹ", "ɮ": "ㄹ"
    ]

    // IPA Vowels → Hangul
    private let vowels: [String: String] = [
        // Close
        "i": "ㅣ", "y": "ㅟ", "ɨ": "ㅡ", "ʉ": "ㅜ", "ɯ": "ㅡ", "u": "ㅜ",

        // Near-close
        "ɪ": "ㅣ", "ʏ": "ㅟ", "ʊ": "ㅜ",

        // Close-mid
        "e": "ㅔ", "ø": "ㅚ", "ɘ": "ㅓ", "ɵ": "ㅗ", "ɤ": "ㅓ", "o": "ㅗ",

        // Mid
        "ə": "ㅓ", "ɚ": "ㅓ",

        // Open-mid
        "ɛ": "ㅔ", "œ": "ㅚ", "ɜ": "ㅓ", "ɞ": "ㅓ", "ʌ": "ㅓ", "ɔ": "ㅗ",

        // Near-open
        "æ": "ㅐ", "ɐ": "ㅏ",

        // Open
        "a": "ㅏ", "ɶ": "ㅚ", "ɑ": "ㅏ", "ɒ": "ㅗ"
    ]

    // Common words lookup
    private let wordLookup: [String: String] = [
        "hello": "헬로", "world": "월드", "thank": "땡크", "you": "유",
        "good": "굿", "morning": "모닝", "night": "나잇", "bye": "바이",
        "please": "플리즈", "sorry": "쏘리", "love": "러브", "friend": "프렌드",
        "water": "워터", "food": "푸드", "coffee": "커피", "tea": "티",
        "yes": "예스", "no": "노", "okay": "오케이", "beautiful": "뷰티풀",
        "konnichiwa": "곤니치와", "arigatou": "아리가또", "sayonara": "사요나라",
        "hola": "올라", "gracias": "그라시아스", "adios": "아디오스",
        "bonjour": "봉주르", "merci": "메르시",
        "guten": "구텐", "morgen": "모르겐", "danke": "당케",
        "nihao": "니하오", "xiexie": "셰셰"
    ]

    // Romanization mappings
    private let romanMap: [String: String] = [
        "ch": "ㅊ", "sh": "ㅅ", "th": "ㅅ", "ph": "ㅍ", "ng": "ㅇ",
        "ts": "ㅈ", "dz": "ㅈ", "zh": "ㅈ", "kh": "ㅋ", "gh": "ㄱ",
        "ck": "ㅋ", "qu": "ㅋㅜ", "x": "ㅋㅅ",
        "p": "ㅍ", "b": "ㅂ", "t": "ㅌ", "d": "ㄷ", "k": "ㅋ", "g": "ㄱ",
        "f": "ㅍ", "v": "ㅂ", "s": "ㅅ", "z": "ㅈ", "j": "ㅈ", "c": "ㅋ",
        "m": "ㅁ", "n": "ㄴ", "l": "ㄹ", "r": "ㄹ", "h": "ㅎ",
        "w": "ㅜ", "y": "ㅣ",
        "ee": "ㅣ", "ea": "ㅣ", "oo": "ㅜ", "ou": "ㅜ", "au": "ㅗ",
        "ai": "ㅐ", "ay": "ㅐ", "ei": "ㅔ", "ey": "ㅔ",
        "oi": "ㅗㅣ", "oy": "ㅗㅣ", "ow": "ㅗㅜ",
        "a": "ㅏ", "e": "ㅔ", "i": "ㅣ", "o": "ㅗ", "u": "ㅜ"
    ]

    // Hangul composition constants
    private let CHOSUNG = ["ㄱ","ㄲ","ㄴ","ㄷ","ㄸ","ㄹ","ㅁ","ㅂ","ㅃ","ㅅ","ㅆ","ㅇ","ㅈ","ㅉ","ㅊ","ㅋ","ㅌ","ㅍ","ㅎ"]
    private let JUNGSUNG = ["ㅏ","ㅐ","ㅑ","ㅒ","ㅓ","ㅔ","ㅕ","ㅖ","ㅗ","ㅘ","ㅙ","ㅚ","ㅛ","ㅜ","ㅝ","ㅞ","ㅟ","ㅠ","ㅡ","ㅢ","ㅣ"]
    private let JONGSUNG = ["","ㄱ","ㄲ","ㄳ","ㄴ","ㄵ","ㄶ","ㄷ","ㄹ","ㄺ","ㄻ","ㄼ","ㄽ","ㄾ","ㄿ","ㅀ","ㅁ","ㅂ","ㅄ","ㅅ","ㅆ","ㅇ","ㅈ","ㅊ","ㅋ","ㅌ","ㅍ","ㅎ"]

    private init() {}

    /// Compose Hangul syllable from jamo
    private func composeHangul(cho: String, jung: String, jong: String = "") -> String {
        guard let choIdx = CHOSUNG.firstIndex(of: cho),
              let jungIdx = JUNGSUNG.firstIndex(of: jung) else {
            return cho + jung + jong
        }

        let jongIdx = JONGSUNG.firstIndex(of: jong) ?? 0
        let code = 0xAC00 + (choIdx * 21 * 28) + (jungIdx * 28) + (jongIdx > 0 ? jongIdx : 0)

        return String(UnicodeScalar(code)!)
    }

    /// Convert romanization to jamo
    private func romanToJamo(_ text: String) -> String {
        var result = ""
        var i = text.startIndex

        while i < text.endIndex {
            var matched = false

            // Try 2-char patterns first
            if text.distance(from: i, to: text.endIndex) >= 2 {
                let end = text.index(i, offsetBy: 2)
                let segment = String(text[i..<end])
                if let jamo = romanMap[segment] {
                    result += jamo
                    i = end
                    matched = true
                }
            }

            // Try single char
            if !matched {
                let segment = String(text[i])
                if let jamo = romanMap[segment] {
                    result += jamo
                }
                i = text.index(after: i)
            }
        }

        return result
    }

    /// Compose jamo sequence into syllables
    private func jamoToSyllables(_ jamo: String) -> String {
        var result = ""
        var buffer = (cho: "", jung: "", jong: "")
        var state = "cho"

        func isChosung(_ c: String) -> Bool { CHOSUNG.contains(c) }
        func isJungsung(_ c: String) -> Bool { JUNGSUNG.contains(c) }
        func isJongsung(_ c: String) -> Bool { JONGSUNG.contains(c) && !c.isEmpty }

        let chars = Array(jamo)
        for i in 0..<chars.count {
            let c = String(chars[i])

            switch state {
            case "cho":
                if isChosung(c) {
                    buffer = (c, "", "")
                    state = "jung"
                } else if isJungsung(c) {
                    buffer = ("ㅇ", c, "")
                    state = "jong"
                }

            case "jung":
                if isJungsung(c) {
                    buffer.jung = c
                    state = "jong"
                } else if isChosung(c) {
                    result += buffer.cho
                    buffer = (c, "", "")
                    state = "jung"
                }

            case "jong":
                if isChosung(c) && isJongsung(c) {
                    let next = i + 1 < chars.count ? String(chars[i + 1]) : ""
                    if !next.isEmpty && isJungsung(next) {
                        result += composeHangul(cho: buffer.cho, jung: buffer.jung, jong: buffer.jong)
                        buffer = (c, "", "")
                        state = "jung"
                    } else {
                        buffer.jong = c
                    }
                } else if isJungsung(c) {
                    result += composeHangul(cho: buffer.cho, jung: buffer.jung, jong: buffer.jong)
                    buffer = ("ㅇ", c, "")
                    state = "jong"
                } else if isChosung(c) {
                    result += composeHangul(cho: buffer.cho, jung: buffer.jung, jong: buffer.jong)
                    buffer = (c, "", "")
                    state = "jung"
                }

            default:
                break
            }
        }

        // Flush remaining buffer
        if !buffer.cho.isEmpty && !buffer.jung.isEmpty {
            result += composeHangul(cho: buffer.cho, jung: buffer.jung, jong: buffer.jong)
        } else if !buffer.cho.isEmpty {
            result += buffer.cho
        }

        return result
    }

    /// Convert any text to WIHP Hangul
    func convert(_ text: String) -> String {
        let normalized = text.lowercased().trimmingCharacters(in: .whitespaces)

        // Check word lookup first
        if let hangul = wordLookup[normalized] {
            return hangul
        }

        // Check if it's IPA
        let ipaPattern = try! NSRegularExpression(pattern: "[əɪʊɛɔæɑʌɒŋθðʃʒɹɾɻʔ]")
        let hasIPA = ipaPattern.firstMatch(in: normalized, range: NSRange(normalized.startIndex..., in: normalized)) != nil

        if hasIPA {
            let jamo = ipaToJamo(normalized)
            return jamoToSyllables(jamo)
        }

        // Convert romanization
        let jamo = romanToJamo(normalized)
        return jamoToSyllables(jamo)
    }

    private func ipaToJamo(_ ipa: String) -> String {
        let cleaned = ipa.replacingOccurrences(of: "[/\\[\\]ˈˌːˑ]", with: "", options: .regularExpression)
        var result = ""

        for char in cleaned {
            let c = String(char)
            if let jamo = consonants[c] {
                result += jamo
            } else if let jamo = vowels[c] {
                result += jamo
            }
        }

        return result
    }

    /// Convert Hangul to Braille
    func toBraille(_ hangul: String) -> String {
        let brailleMap: [String: String] = [
            "ㄱ": "⠈", "ㄴ": "⠉", "ㄷ": "⠊", "ㄹ": "⠐", "ㅁ": "⠑",
            "ㅂ": "⠘", "ㅅ": "⠠", "ㅇ": "⠛", "ㅈ": "⠨", "ㅊ": "⠰",
            "ㅋ": "⠋", "ㅌ": "⠓", "ㅍ": "⠙", "ㅎ": "⠚",
            "ㅏ": "⠣", "ㅓ": "⠎", "ㅗ": "⠥", "ㅜ": "⠍", "ㅡ": "⠪", "ㅣ": "⠕",
            "ㅐ": "⠗", "ㅔ": "⠝", "ㅚ": "⠽", "ㅟ": "⠍⠗"
        ]

        var result = ""
        for char in hangul {
            if char >= "가" && char <= "힣" {
                let code = Int(char.unicodeScalars.first!.value) - 0xAC00
                let cho = code / (21 * 28)
                let jung = (code % (21 * 28)) / 28
                let jong = code % 28

                if let b = brailleMap[CHOSUNG[cho]] { result += b }
                if let b = brailleMap[JUNGSUNG[jung]] { result += b }
                if jong > 0, let b = brailleMap[JONGSUNG[jong]] { result += b }
            } else if let b = brailleMap[String(char)] {
                result += b
            }
        }
        return result
    }
}
