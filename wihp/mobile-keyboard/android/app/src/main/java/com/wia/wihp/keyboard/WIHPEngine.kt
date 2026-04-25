package com.wia.wihp.keyboard

/**
 * WIHP Engine - IPA to Hangul Conversion
 * WIA International Hangul Phonology
 *
 * 홍익인간 (弘益人間) - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */
object WIHPEngine {

    // IPA Consonants → Hangul
    private val consonants = mapOf(
        // Plosives
        "p" to "ㅍ", "b" to "ㅂ", "t" to "ㅌ", "d" to "ㄷ", "k" to "ㅋ", "g" to "ㄱ",
        "ʈ" to "ㅌ", "ɖ" to "ㄷ", "c" to "ㅊ", "ɟ" to "ㅈ", "q" to "ㅋ", "ɢ" to "ㄱ",
        "ʔ" to "ㅇ",

        // Nasals
        "m" to "ㅁ", "ɱ" to "ㅁ", "n" to "ㄴ", "ɳ" to "ㄴ", "ɲ" to "ㄴ", "ŋ" to "ㅇ", "ɴ" to "ㅇ",

        // Trills & Taps
        "ʙ" to "ㅂ", "r" to "ㄹ", "ʀ" to "ㄹ", "ⱱ" to "ㅂ", "ɾ" to "ㄹ", "ɽ" to "ㄹ",

        // Fricatives
        "ɸ" to "ㅍ", "β" to "ㅂ", "f" to "ㅍ", "v" to "ㅂ",
        "θ" to "ㅅ", "ð" to "ㄷ", "s" to "ㅅ", "z" to "ㅈ",
        "ʃ" to "ㅅ", "ʒ" to "ㅈ", "ʂ" to "ㅅ", "ʐ" to "ㅈ",
        "ç" to "ㅎ", "ʝ" to "ㅈ", "x" to "ㅎ", "ɣ" to "ㄱ",
        "χ" to "ㅎ", "ʁ" to "ㄹ", "ħ" to "ㅎ", "ʕ" to "ㅇ",
        "h" to "ㅎ", "ɦ" to "ㅎ",

        // Affricates
        "tʃ" to "ㅊ", "dʒ" to "ㅈ", "ts" to "ㅈ", "dz" to "ㅈ",
        "tɕ" to "ㅊ", "dʑ" to "ㅈ",

        // Approximants
        "ʋ" to "ㅂ", "ɹ" to "ㄹ", "ɻ" to "ㄹ", "j" to "ㅇ", "ɰ" to "ㅇ",
        "l" to "ㄹ", "ɭ" to "ㄹ", "ʎ" to "ㄹ", "ʟ" to "ㄹ",
        "w" to "ㅇ", "ʍ" to "ㅎ", "ɥ" to "ㅇ",

        // Lateral fricatives
        "ɬ" to "ㄹ", "ɮ" to "ㄹ"
    )

    // IPA Vowels → Hangul
    private val vowels = mapOf(
        // Close
        "i" to "ㅣ", "y" to "ㅟ", "ɨ" to "ㅡ", "ʉ" to "ㅜ", "ɯ" to "ㅡ", "u" to "ㅜ",

        // Near-close
        "ɪ" to "ㅣ", "ʏ" to "ㅟ", "ʊ" to "ㅜ",

        // Close-mid
        "e" to "ㅔ", "ø" to "ㅚ", "ɘ" to "ㅓ", "ɵ" to "ㅗ", "ɤ" to "ㅓ", "o" to "ㅗ",

        // Mid
        "ə" to "ㅓ", "ɚ" to "ㅓ",

        // Open-mid
        "ɛ" to "ㅔ", "œ" to "ㅚ", "ɜ" to "ㅓ", "ɞ" to "ㅓ", "ʌ" to "ㅓ", "ɔ" to "ㅗ",

        // Near-open
        "æ" to "ㅐ", "ɐ" to "ㅏ",

        // Open
        "a" to "ㅏ", "ɶ" to "ㅚ", "ɑ" to "ㅏ", "ɒ" to "ㅗ"
    )

    // Common words lookup
    private val wordLookup = mapOf(
        "hello" to "헬로", "world" to "월드", "thank" to "땡크", "you" to "유",
        "good" to "굿", "morning" to "모닝", "night" to "나잇", "bye" to "바이",
        "please" to "플리즈", "sorry" to "쏘리", "love" to "러브", "friend" to "프렌드",
        "water" to "워터", "food" to "푸드", "coffee" to "커피", "tea" to "티",
        "yes" to "예스", "no" to "노", "okay" to "오케이", "beautiful" to "뷰티풀",
        "konnichiwa" to "곤니치와", "arigatou" to "아리가또", "sayonara" to "사요나라",
        "hola" to "올라", "gracias" to "그라시아스", "adios" to "아디오스",
        "bonjour" to "봉주르", "merci" to "메르시",
        "guten" to "구텐", "morgen" to "모르겐", "danke" to "당케",
        "nihao" to "니하오", "xiexie" to "셰셰"
    )

    // Romanization mappings
    private val romanMap = mapOf(
        "ch" to "ㅊ", "sh" to "ㅅ", "th" to "ㅅ", "ph" to "ㅍ", "ng" to "ㅇ",
        "ts" to "ㅈ", "dz" to "ㅈ", "zh" to "ㅈ", "kh" to "ㅋ", "gh" to "ㄱ",
        "ck" to "ㅋ", "qu" to "ㅋㅜ", "x" to "ㅋㅅ",
        "p" to "ㅍ", "b" to "ㅂ", "t" to "ㅌ", "d" to "ㄷ", "k" to "ㅋ", "g" to "ㄱ",
        "f" to "ㅍ", "v" to "ㅂ", "s" to "ㅅ", "z" to "ㅈ", "j" to "ㅈ", "c" to "ㅋ",
        "m" to "ㅁ", "n" to "ㄴ", "l" to "ㄹ", "r" to "ㄹ", "h" to "ㅎ",
        "w" to "ㅜ", "y" to "ㅣ",
        "ee" to "ㅣ", "ea" to "ㅣ", "oo" to "ㅜ", "ou" to "ㅜ", "au" to "ㅗ",
        "ai" to "ㅐ", "ay" to "ㅐ", "ei" to "ㅔ", "ey" to "ㅔ",
        "oi" to "ㅗㅣ", "oy" to "ㅗㅣ", "ow" to "ㅗㅜ",
        "a" to "ㅏ", "e" to "ㅔ", "i" to "ㅣ", "o" to "ㅗ", "u" to "ㅜ"
    )

    // Hangul composition constants
    private val CHOSUNG = listOf("ㄱ","ㄲ","ㄴ","ㄷ","ㄸ","ㄹ","ㅁ","ㅂ","ㅃ","ㅅ","ㅆ","ㅇ","ㅈ","ㅉ","ㅊ","ㅋ","ㅌ","ㅍ","ㅎ")
    private val JUNGSUNG = listOf("ㅏ","ㅐ","ㅑ","ㅒ","ㅓ","ㅔ","ㅕ","ㅖ","ㅗ","ㅘ","ㅙ","ㅚ","ㅛ","ㅜ","ㅝ","ㅞ","ㅟ","ㅠ","ㅡ","ㅢ","ㅣ")
    private val JONGSUNG = listOf("","ㄱ","ㄲ","ㄳ","ㄴ","ㄵ","ㄶ","ㄷ","ㄹ","ㄺ","ㄻ","ㄼ","ㄽ","ㄾ","ㄿ","ㅀ","ㅁ","ㅂ","ㅄ","ㅅ","ㅆ","ㅇ","ㅈ","ㅊ","ㅋ","ㅌ","ㅍ","ㅎ")

    /**
     * Compose Hangul syllable from jamo
     */
    private fun composeHangul(cho: String, jung: String, jong: String = ""): String {
        val choIdx = CHOSUNG.indexOf(cho)
        val jungIdx = JUNGSUNG.indexOf(jung)
        val jongIdx = JONGSUNG.indexOf(jong)

        if (choIdx < 0 || jungIdx < 0) return cho + jung + jong

        val code = 0xAC00 + (choIdx * 21 * 28) + (jungIdx * 28) + (if (jongIdx > 0) jongIdx else 0)
        return code.toChar().toString()
    }

    /**
     * Convert romanization to jamo
     */
    private fun romanToJamo(text: String): String {
        val result = StringBuilder()
        var i = 0

        while (i < text.length) {
            var matched = false

            // Try longer patterns first (2 chars)
            if (i + 2 <= text.length) {
                val segment = text.substring(i, i + 2)
                romanMap[segment]?.let {
                    result.append(it)
                    i += 2
                    matched = true
                }
            }

            // Try single char
            if (!matched && i < text.length) {
                val segment = text.substring(i, i + 1)
                romanMap[segment]?.let {
                    result.append(it)
                    matched = true
                }
                i++
            }
        }

        return result.toString()
    }

    /**
     * Compose jamo sequence into syllables
     */
    private fun jamoToSyllables(jamo: String): String {
        val result = StringBuilder()
        var buffer = Triple("", "", "") // cho, jung, jong
        var state = "cho"

        fun isChosung(c: String) = CHOSUNG.contains(c)
        fun isJungsung(c: String) = JUNGSUNG.contains(c)
        fun isJongsung(c: String) = JONGSUNG.contains(c) && c.isNotEmpty()

        for (i in jamo.indices) {
            val c = jamo[i].toString()

            when (state) {
                "cho" -> {
                    if (isChosung(c)) {
                        buffer = Triple(c, "", "")
                        state = "jung"
                    } else if (isJungsung(c)) {
                        buffer = Triple("ㅇ", c, "")
                        state = "jong"
                    }
                }
                "jung" -> {
                    if (isJungsung(c)) {
                        buffer = Triple(buffer.first, c, "")
                        state = "jong"
                    } else if (isChosung(c)) {
                        result.append(buffer.first)
                        buffer = Triple(c, "", "")
                        state = "jung"
                    }
                }
                "jong" -> {
                    if (isChosung(c) && isJongsung(c)) {
                        val next = if (i + 1 < jamo.length) jamo[i + 1].toString() else ""
                        if (next.isNotEmpty() && isJungsung(next)) {
                            result.append(composeHangul(buffer.first, buffer.second, buffer.third))
                            buffer = Triple(c, "", "")
                            state = "jung"
                        } else {
                            buffer = Triple(buffer.first, buffer.second, c)
                        }
                    } else if (isJungsung(c)) {
                        result.append(composeHangul(buffer.first, buffer.second, buffer.third))
                        buffer = Triple("ㅇ", c, "")
                        state = "jong"
                    } else if (isChosung(c)) {
                        result.append(composeHangul(buffer.first, buffer.second, buffer.third))
                        buffer = Triple(c, "", "")
                        state = "jung"
                    }
                }
            }
        }

        // Flush remaining buffer
        if (buffer.first.isNotEmpty() && buffer.second.isNotEmpty()) {
            result.append(composeHangul(buffer.first, buffer.second, buffer.third))
        } else if (buffer.first.isNotEmpty()) {
            result.append(buffer.first)
        }

        return result.toString()
    }

    /**
     * Convert any text to WIHP Hangul
     */
    fun convert(text: String): String {
        val normalized = text.lowercase().trim()

        // Check word lookup first
        wordLookup[normalized]?.let { return it }

        // Check if it's IPA (contains IPA characters)
        val hasIPA = normalized.contains(Regex("[əɪʊɛɔæɑʌɒŋθðʃʒɹɾɻʔ]"))

        if (hasIPA) {
            // IPA conversion (simplified for mobile)
            val jamo = ipaToJamo(normalized)
            return jamoToSyllables(jamo)
        }

        // Convert romanization
        val jamo = romanToJamo(normalized)
        return jamoToSyllables(jamo)
    }

    private fun ipaToJamo(ipa: String): String {
        val cleaned = ipa.replace(Regex("[/\\[\\]ˈˌːˑ]"), "").trim()
        val result = StringBuilder()

        for (c in cleaned) {
            val ch = c.toString()
            consonants[ch]?.let {
                result.append(it)
            } ?: vowels[ch]?.let {
                result.append(it)
            }
        }

        return result.toString()
    }

    /**
     * Convert Hangul to Braille
     */
    fun toBraille(hangul: String): String {
        val brailleMap = mapOf(
            "ㄱ" to "⠈", "ㄴ" to "⠉", "ㄷ" to "⠊", "ㄹ" to "⠐", "ㅁ" to "⠑",
            "ㅂ" to "⠘", "ㅅ" to "⠠", "ㅇ" to "⠛", "ㅈ" to "⠨", "ㅊ" to "⠰",
            "ㅋ" to "⠋", "ㅌ" to "⠓", "ㅍ" to "⠙", "ㅎ" to "⠚",
            "ㅏ" to "⠣", "ㅓ" to "⠎", "ㅗ" to "⠥", "ㅜ" to "⠍", "ㅡ" to "⠪", "ㅣ" to "⠕",
            "ㅐ" to "⠗", "ㅔ" to "⠝", "ㅚ" to "⠽", "ㅟ" to "⠍⠗"
        )

        val result = StringBuilder()
        for (char in hangul) {
            if (char in '가'..'힣') {
                val code = char.code - 0xAC00
                val cho = code / (21 * 28)
                val jung = (code % (21 * 28)) / 28
                val jong = code % 28

                brailleMap[CHOSUNG[cho]]?.let { result.append(it) }
                brailleMap[JUNGSUNG[jung]]?.let { result.append(it) }
                if (jong > 0) {
                    brailleMap[JONGSUNG[jong]]?.let { result.append(it) }
                }
            } else {
                brailleMap[char.toString()]?.let { result.append(it) }
            }
        }
        return result.toString()
    }
}
