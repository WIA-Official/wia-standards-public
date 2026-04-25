package com.wia.wihp.keyboard

import android.inputmethodservice.InputMethodService
import android.os.Build
import android.os.VibrationEffect
import android.os.Vibrator
import android.view.View
import android.view.inputmethod.EditorInfo
import android.widget.Button
import android.widget.LinearLayout
import android.widget.TextView
import androidx.core.content.ContextCompat
import android.content.Context
import android.graphics.Color
import android.view.Gravity
import android.widget.HorizontalScrollView

/**
 * WIHP Keyboard - InputMethodService
 * Universal Hangul Phonology Keyboard
 *
 * 홍익인간 (弘益人間) - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */
class WIHPInputMethodService : InputMethodService() {

    private lateinit var keyboardView: LinearLayout
    private lateinit var candidateView: HorizontalScrollView
    private lateinit var candidateContainer: LinearLayout
    private lateinit var inputBuffer: StringBuilder
    private lateinit var vibrator: Vibrator

    private var isShifted = false
    private var isNumeric = false

    private val qwertyRows = listOf(
        listOf("q", "w", "e", "r", "t", "y", "u", "i", "o", "p"),
        listOf("a", "s", "d", "f", "g", "h", "j", "k", "l"),
        listOf("⇧", "z", "x", "c", "v", "b", "n", "m", "⌫"),
        listOf("123", " ", "한글", "↵")
    )

    private val numericRows = listOf(
        listOf("1", "2", "3", "4", "5", "6", "7", "8", "9", "0"),
        listOf("@", "#", "$", "%", "&", "*", "-", "+", "(", ")"),
        listOf("!", "\"", "'", ":", ";", "/", "?", ",", ".", "⌫"),
        listOf("ABC", " ", "한글", "↵")
    )

    override fun onCreate() {
        super.onCreate()
        inputBuffer = StringBuilder()
        vibrator = getSystemService(Context.VIBRATOR_SERVICE) as Vibrator
    }

    override fun onCreateInputView(): View {
        keyboardView = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setBackgroundColor(Color.parseColor("#1f2937"))
            setPadding(4, 8, 4, 8)
        }

        // Create candidate bar
        candidateView = HorizontalScrollView(this).apply {
            setBackgroundColor(Color.parseColor("#111827"))
            layoutParams = LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                dpToPx(40)
            )
        }

        candidateContainer = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            setPadding(dpToPx(8), 0, dpToPx(8), 0)
        }
        candidateView.addView(candidateContainer)
        keyboardView.addView(candidateView)

        // Build keyboard
        buildKeyboard()

        return keyboardView
    }

    private fun buildKeyboard() {
        // Remove old rows (keep candidate bar)
        while (keyboardView.childCount > 1) {
            keyboardView.removeViewAt(1)
        }

        val rows = if (isNumeric) numericRows else qwertyRows

        for (row in rows) {
            val rowLayout = LinearLayout(this).apply {
                orientation = LinearLayout.HORIZONTAL
                gravity = Gravity.CENTER
                layoutParams = LinearLayout.LayoutParams(
                    LinearLayout.LayoutParams.MATCH_PARENT,
                    dpToPx(52)
                ).apply {
                    setMargins(0, 2, 0, 2)
                }
            }

            for (key in row) {
                val button = createKeyButton(key)
                rowLayout.addView(button)
            }

            keyboardView.addView(rowLayout)
        }
    }

    private fun createKeyButton(key: String): Button {
        return Button(this).apply {
            text = if (isShifted && key.length == 1 && key[0].isLetter()) {
                key.uppercase()
            } else {
                key
            }
            textSize = when {
                key == " " -> 12f
                key.length > 2 -> 14f
                else -> 18f
            }
            setTextColor(Color.WHITE)

            // Styling based on key type
            val bgColor = when (key) {
                "⇧" -> if (isShifted) "#059669" else "#1e40af"
                "⌫", "↵" -> "#1e40af"
                "123", "ABC" -> "#374151"
                "한글" -> "#059669"
                " " -> "#374151"
                else -> "#4b5563"
            }
            setBackgroundColor(Color.parseColor(bgColor))

            // Layout params
            val weight = when (key) {
                " " -> 3f
                "한글" -> 1.5f
                "⇧", "⌫" -> 1.2f
                else -> 1f
            }
            layoutParams = LinearLayout.LayoutParams(
                0,
                LinearLayout.LayoutParams.MATCH_PARENT,
                weight
            ).apply {
                setMargins(2, 0, 2, 0)
            }

            // Click handler
            setOnClickListener { onKeyPressed(key) }
        }
    }

    private fun onKeyPressed(key: String) {
        vibrate()

        when (key) {
            "⇧" -> {
                isShifted = !isShifted
                buildKeyboard()
            }
            "⌫" -> {
                if (inputBuffer.isNotEmpty()) {
                    inputBuffer.deleteCharAt(inputBuffer.length - 1)
                    updateCandidates()
                } else {
                    currentInputConnection?.deleteSurroundingText(1, 0)
                }
            }
            "↵" -> {
                commitBuffer()
                currentInputConnection?.performEditorAction(EditorInfo.IME_ACTION_DONE)
            }
            "123" -> {
                isNumeric = true
                buildKeyboard()
            }
            "ABC" -> {
                isNumeric = false
                buildKeyboard()
            }
            "한글" -> {
                convertAndCommit()
            }
            " " -> {
                commitBuffer()
                currentInputConnection?.commitText(" ", 1)
            }
            else -> {
                val char = if (isShifted && key.length == 1) key.uppercase() else key
                inputBuffer.append(char)
                updateCandidates()

                if (isShifted && key.length == 1 && key[0].isLetter()) {
                    isShifted = false
                    buildKeyboard()
                }
            }
        }
    }

    private fun updateCandidates() {
        candidateContainer.removeAllViews()

        if (inputBuffer.isEmpty()) return

        val text = inputBuffer.toString()

        // Original text
        addCandidate(text, false)

        // WIHP Hangul conversion
        val hangul = WIHPEngine.convert(text)
        if (hangul.isNotEmpty() && hangul != text) {
            addCandidate(hangul, true)

            // Braille
            val braille = WIHPEngine.toBraille(hangul)
            if (braille.isNotEmpty()) {
                addCandidate(braille, false)
            }
        }
    }

    private fun addCandidate(text: String, isHangul: Boolean) {
        val tv = TextView(this).apply {
            this.text = text
            textSize = if (isHangul) 20f else 16f
            setTextColor(if (isHangul) Color.parseColor("#fbbf24") else Color.WHITE)
            setPadding(dpToPx(12), dpToPx(4), dpToPx(12), dpToPx(4))
            setBackgroundColor(Color.parseColor("#1f2937"))

            setOnClickListener {
                inputBuffer.clear()
                currentInputConnection?.commitText(text, 1)
                updateCandidates()
            }
        }
        candidateContainer.addView(tv)
    }

    private fun convertAndCommit() {
        if (inputBuffer.isEmpty()) return

        val hangul = WIHPEngine.convert(inputBuffer.toString())
        currentInputConnection?.commitText(hangul, 1)
        inputBuffer.clear()
        updateCandidates()
    }

    private fun commitBuffer() {
        if (inputBuffer.isNotEmpty()) {
            currentInputConnection?.commitText(inputBuffer.toString(), 1)
            inputBuffer.clear()
            updateCandidates()
        }
    }

    private fun vibrate() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            vibrator.vibrate(VibrationEffect.createOneShot(20, VibrationEffect.DEFAULT_AMPLITUDE))
        } else {
            @Suppress("DEPRECATION")
            vibrator.vibrate(20)
        }
    }

    private fun dpToPx(dp: Int): Int {
        return (dp * resources.displayMetrics.density).toInt()
    }

    override fun onStartInputView(info: EditorInfo?, restarting: Boolean) {
        super.onStartInputView(info, restarting)
        inputBuffer.clear()
        updateCandidates()
    }

    override fun onFinishInput() {
        super.onFinishInput()
        inputBuffer.clear()
    }
}
