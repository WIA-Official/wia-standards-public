package com.wia.wihp.keyboard

import android.content.Intent
import android.os.Bundle
import android.provider.Settings
import android.view.inputmethod.InputMethodManager
import android.widget.Button
import android.widget.EditText
import android.widget.TextView
import android.widget.LinearLayout
import android.graphics.Color
import android.view.Gravity
import android.widget.ScrollView
import androidx.appcompat.app.AppCompatActivity

/**
 * WIHP Keyboard - Main Activity
 * Setup and demo interface
 */
class MainActivity : AppCompatActivity() {

    private lateinit var previewText: TextView
    private lateinit var inputField: EditText

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val scrollView = ScrollView(this)
        val layout = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dpToPx(24), dpToPx(32), dpToPx(24), dpToPx(32))
            setBackgroundColor(Color.parseColor("#1f2937"))
        }
        scrollView.addView(layout)
        setContentView(scrollView)

        // Header
        val header = TextView(this).apply {
            text = "WIHP Keyboard"
            textSize = 32f
            setTextColor(Color.WHITE)
            gravity = Gravity.CENTER
            setPadding(0, 0, 0, dpToPx(8))
        }
        layout.addView(header)

        val subtitle = TextView(this).apply {
            text = "Universal Hangul Phonology"
            textSize = 16f
            setTextColor(Color.parseColor("#9ca3af"))
            gravity = Gravity.CENTER
            setPadding(0, 0, 0, dpToPx(24))
        }
        layout.addView(subtitle)

        // Enable Keyboard Button
        val enableBtn = createButton("키보드 활성화하기", "#1e40af") {
            startActivity(Intent(Settings.ACTION_INPUT_METHOD_SETTINGS))
        }
        layout.addView(enableBtn)

        // Switch Keyboard Button
        val switchBtn = createButton("키보드 전환하기", "#059669") {
            val imm = getSystemService(INPUT_METHOD_SERVICE) as InputMethodManager
            imm.showInputMethodPicker()
        }
        layout.addView(switchBtn)

        // Settings Button
        val settingsBtn = createButton("설정", "#374151") {
            startActivity(Intent(this, SettingsActivity::class.java))
        }
        layout.addView(settingsBtn)

        // Demo Section
        val demoLabel = TextView(this).apply {
            text = "테스트 입력"
            textSize = 14f
            setTextColor(Color.parseColor("#9ca3af"))
            setPadding(0, dpToPx(32), 0, dpToPx(8))
        }
        layout.addView(demoLabel)

        inputField = EditText(this).apply {
            hint = "hello, bonjour, konnichiwa..."
            textSize = 18f
            setTextColor(Color.WHITE)
            setHintTextColor(Color.parseColor("#6b7280"))
            setBackgroundColor(Color.parseColor("#374151"))
            setPadding(dpToPx(16), dpToPx(16), dpToPx(16), dpToPx(16))
        }
        layout.addView(inputField)

        // Convert Button
        val convertBtn = createButton("한글로 변환", "#fbbf24", Color.parseColor("#1f2937")) {
            val text = inputField.text.toString()
            if (text.isNotEmpty()) {
                previewText.text = WIHPEngine.convert(text)
            }
        }
        layout.addView(convertBtn)

        // Preview
        previewText = TextView(this).apply {
            text = "한글"
            textSize = 48f
            setTextColor(Color.parseColor("#fbbf24"))
            gravity = Gravity.CENTER
            setPadding(0, dpToPx(24), 0, dpToPx(24))
        }
        layout.addView(previewText)

        // Instructions
        val instructions = TextView(this).apply {
            text = """
                📌 설치 방법

                1. '키보드 활성화하기' 버튼 클릭
                2. WIHP Keyboard 스위치 ON
                3. '키보드 전환하기'로 WIHP 선택

                ⌨️ 사용법

                • 영어/외국어 입력 후 '한글' 버튼
                • 자동으로 한글 발음 변환
                • 후보 영역에서 선택 가능
            """.trimIndent()
            textSize = 14f
            setTextColor(Color.parseColor("#9ca3af"))
            setPadding(0, dpToPx(16), 0, 0)
        }
        layout.addView(instructions)

        // Philosophy
        val philosophy = TextView(this).apply {
            text = "\n홍익인간 (弘益人間)\nBenefit All Humanity"
            textSize = 14f
            setTextColor(Color.parseColor("#6b7280"))
            gravity = Gravity.CENTER
            setPadding(0, dpToPx(32), 0, 0)
        }
        layout.addView(philosophy)

        // Footer
        val footer = TextView(this).apply {
            text = "© 2025 SmileStory Inc. / WIA"
            textSize = 12f
            setTextColor(Color.parseColor("#4b5563"))
            gravity = Gravity.CENTER
            setPadding(0, dpToPx(16), 0, 0)
        }
        layout.addView(footer)
    }

    private fun createButton(
        text: String,
        bgColor: String,
        textColor: Int = Color.WHITE,
        onClick: () -> Unit
    ): Button {
        return Button(this).apply {
            this.text = text
            textSize = 16f
            setTextColor(textColor)
            setBackgroundColor(Color.parseColor(bgColor))
            layoutParams = LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                dpToPx(56)
            ).apply {
                setMargins(0, dpToPx(8), 0, dpToPx(8))
            }
            setOnClickListener { onClick() }
        }
    }

    private fun dpToPx(dp: Int): Int {
        return (dp * resources.displayMetrics.density).toInt()
    }
}
