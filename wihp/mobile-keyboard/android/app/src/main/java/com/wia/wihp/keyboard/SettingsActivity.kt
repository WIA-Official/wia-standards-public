package com.wia.wihp.keyboard

import android.content.SharedPreferences
import android.graphics.Color
import android.os.Bundle
import android.view.Gravity
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.Switch
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity

/**
 * WIHP Keyboard - Settings Activity
 */
class SettingsActivity : AppCompatActivity() {

    private lateinit var prefs: SharedPreferences

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        prefs = getSharedPreferences("wihp_prefs", MODE_PRIVATE)

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
            text = "WIHP 설정"
            textSize = 28f
            setTextColor(Color.WHITE)
            setPadding(0, 0, 0, dpToPx(24))
        }
        layout.addView(header)

        // Vibration Setting
        addSetting(layout, "vibration", "진동 피드백", "키 입력 시 진동", true)

        // Auto Convert Setting
        addSetting(layout, "auto_convert", "자동 변환", "스페이스 입력 시 자동으로 한글 변환", false)

        // Show Braille Setting
        addSetting(layout, "show_braille", "점자 표시", "변환된 한글과 함께 점자 표시", true)

        // Sound Setting
        addSetting(layout, "sound", "키 소리", "키 입력 시 소리 재생", false)

        // About Section
        val aboutHeader = TextView(this).apply {
            text = "정보"
            textSize = 18f
            setTextColor(Color.WHITE)
            setPadding(0, dpToPx(32), 0, dpToPx(16))
        }
        layout.addView(aboutHeader)

        val versionInfo = TextView(this).apply {
            text = """
                WIHP Keyboard v1.0.0

                WIA International Hangul Phonology
                모든 언어의 발음을 한글로 변환

                188+ 언어 지원
                IPA (국제음성기호) 지원
                점자 (Braille) 연동
            """.trimIndent()
            textSize = 14f
            setTextColor(Color.parseColor("#9ca3af"))
        }
        layout.addView(versionInfo)

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

    private fun addSetting(
        parent: LinearLayout,
        key: String,
        title: String,
        summary: String,
        defaultValue: Boolean
    ) {
        val container = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            setPadding(dpToPx(16), dpToPx(16), dpToPx(16), dpToPx(16))
            setBackgroundColor(Color.parseColor("#374151"))
            layoutParams = LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.WRAP_CONTENT
            ).apply {
                setMargins(0, dpToPx(4), 0, dpToPx(4))
            }
        }

        val textContainer = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            layoutParams = LinearLayout.LayoutParams(
                0,
                LinearLayout.LayoutParams.WRAP_CONTENT,
                1f
            )
        }

        val titleView = TextView(this).apply {
            text = title
            textSize = 16f
            setTextColor(Color.WHITE)
        }
        textContainer.addView(titleView)

        val summaryView = TextView(this).apply {
            text = summary
            textSize = 12f
            setTextColor(Color.parseColor("#9ca3af"))
        }
        textContainer.addView(summaryView)

        container.addView(textContainer)

        val switch = Switch(this).apply {
            isChecked = prefs.getBoolean(key, defaultValue)
            setOnCheckedChangeListener { _, isChecked ->
                prefs.edit().putBoolean(key, isChecked).apply()
            }
        }
        container.addView(switch)

        parent.addView(container)
    }

    private fun dpToPx(dp: Int): Int {
        return (dp * resources.displayMetrics.density).toInt()
    }
}
