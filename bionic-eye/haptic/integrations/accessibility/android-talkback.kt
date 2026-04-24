/**
 * WIA Haptic Standard - Android TalkBack Integration
 *
 * Kotlin implementation for integrating WIA haptic feedback with Android TalkBack.
 */

package com.wia.haptic.accessibility

import android.accessibilityservice.AccessibilityService
import android.content.Context
import android.os.Build
import android.os.VibrationEffect
import android.os.Vibrator
import android.os.VibratorManager
import android.view.accessibility.AccessibilityEvent
import android.view.accessibility.AccessibilityNodeInfo
import androidx.annotation.RequiresApi

// MARK: - Data Classes

/**
 * WIA haptic waveform types
 */
enum class WIAWaveformType {
    SINE,
    SQUARE,
    TRIANGLE,
    SAWTOOTH,
    NOISE
}

/**
 * WIA haptic primitive
 */
data class WIAHapticPrimitive(
    val waveform: WIAWaveformType,
    val frequency: Int,      // Hz
    val intensity: Float,    // 0-1
    val duration: Long,      // ms
    val delay: Long? = null  // ms
)

/**
 * WIA haptic pattern
 */
data class WIAHapticPattern(
    val id: String,
    val name: String,
    val description: String? = null,
    val primitives: List<WIAHapticPrimitive>,
    val totalDuration: Long
)

// MARK: - TalkBack Haptic Patterns

/**
 * Pre-defined patterns for TalkBack events
 */
object TalkBackHapticPatterns {

    /** Focus moved to new element */
    val FOCUS_CHANGE = WIAHapticPattern(
        id = "tb.focus.change",
        name = "Focus Change",
        description = "TalkBack focus moved to new element",
        primitives = listOf(
            WIAHapticPrimitive(WIAWaveformType.SINE, 150, 0.4f, 30)
        ),
        totalDuration = 30
    )

    /** Focus on button */
    val FOCUS_BUTTON = WIAHapticPattern(
        id = "tb.focus.button",
        name = "Button Focus",
        description = "Focus on interactive button",
        primitives = listOf(
            WIAHapticPrimitive(WIAWaveformType.SQUARE, 180, 0.5f, 40)
        ),
        totalDuration = 40
    )

    /** Focus on edit text */
    val FOCUS_EDIT_TEXT = WIAHapticPattern(
        id = "tb.focus.edittext",
        name = "Edit Text Focus",
        description = "Focus on editable text field",
        primitives = listOf(
            WIAHapticPrimitive(WIAWaveformType.SINE, 120, 0.4f, 50),
            WIAHapticPrimitive(WIAWaveformType.SINE, 140, 0.5f, 50, delay = 30)
        ),
        totalDuration = 130
    )

    /** Focus on link */
    val FOCUS_LINK = WIAHapticPattern(
        id = "tb.focus.link",
        name = "Link Focus",
        description = "Focus on clickable link",
        primitives = listOf(
            WIAHapticPrimitive(WIAWaveformType.SINE, 200, 0.5f, 40)
        ),
        totalDuration = 40
    )

    /** Focus on checkbox/switch */
    val FOCUS_CHECKBOX = WIAHapticPattern(
        id = "tb.focus.checkbox",
        name = "Checkbox Focus",
        description = "Focus on checkbox or switch",
        primitives = listOf(
            WIAHapticPrimitive(WIAWaveformType.SQUARE, 160, 0.4f, 40)
        ),
        totalDuration = 40
    )

    /** Checkbox checked */
    val CHECKBOX_CHECKED = WIAHapticPattern(
        id = "tb.action.checked",
        name = "Checked",
        description = "Checkbox or switch turned on",
        primitives = listOf(
            WIAHapticPrimitive(WIAWaveformType.SINE, 180, 0.6f, 50),
            WIAHapticPrimitive(WIAWaveformType.SINE, 220, 0.7f, 50, delay = 30)
        ),
        totalDuration = 130
    )

    /** Checkbox unchecked */
    val CHECKBOX_UNCHECKED = WIAHapticPattern(
        id = "tb.action.unchecked",
        name = "Unchecked",
        description = "Checkbox or switch turned off",
        primitives = listOf(
            WIAHapticPrimitive(WIAWaveformType.SINE, 180, 0.5f, 50),
            WIAHapticPrimitive(WIAWaveformType.SINE, 140, 0.4f, 50, delay = 30)
        ),
        totalDuration = 130
    )

    /** Element clicked/activated */
    val CLICK = WIAHapticPattern(
        id = "tb.action.click",
        name = "Click",
        description = "Element activated",
        primitives = listOf(
            WIAHapticPrimitive(WIAWaveformType.SQUARE, 200, 0.7f, 60),
            WIAHapticPrimitive(WIAWaveformType.SQUARE, 200, 0.5f, 40, delay = 40)
        ),
        totalDuration = 140
    )

    /** Long press */
    val LONG_CLICK = WIAHapticPattern(
        id = "tb.action.longclick",
        name = "Long Click",
        description = "Long press action",
        primitives = listOf(
            WIAHapticPrimitive(WIAWaveformType.SINE, 150, 0.6f, 100),
            WIAHapticPrimitive(WIAWaveformType.SINE, 180, 0.7f, 100, delay = 50)
        ),
        totalDuration = 250
    )

    /** Scroll event */
    val SCROLL = WIAHapticPattern(
        id = "tb.action.scroll",
        name = "Scroll",
        description = "Content scrolled",
        primitives = listOf(
            WIAHapticPrimitive(WIAWaveformType.SINE, 100, 0.3f, 30)
        ),
        totalDuration = 30
    )

    /** Window changed */
    val WINDOW_CHANGE = WIAHapticPattern(
        id = "tb.window.change",
        name = "Window Changed",
        description = "Window or screen changed",
        primitives = listOf(
            WIAHapticPrimitive(WIAWaveformType.SINE, 140, 0.5f, 80),
            WIAHapticPrimitive(WIAWaveformType.SINE, 180, 0.6f, 80, delay = 50)
        ),
        totalDuration = 210
    )

    /** Reading mode - continuous reading */
    val READING_START = WIAHapticPattern(
        id = "tb.reading.start",
        name = "Reading Started",
        description = "Continuous reading mode started",
        primitives = listOf(
            WIAHapticPrimitive(WIAWaveformType.SINE, 150, 0.4f, 60),
            WIAHapticPrimitive(WIAWaveformType.SINE, 175, 0.5f, 60, delay = 40),
            WIAHapticPrimitive(WIAWaveformType.SINE, 200, 0.6f, 60, delay = 40)
        ),
        totalDuration = 260
    )

    /** Reading stopped */
    val READING_STOP = WIAHapticPattern(
        id = "tb.reading.stop",
        name = "Reading Stopped",
        description = "Continuous reading mode stopped",
        primitives = listOf(
            WIAHapticPrimitive(WIAWaveformType.SINE, 180, 0.5f, 60),
            WIAHapticPrimitive(WIAWaveformType.SINE, 140, 0.4f, 60, delay = 40)
        ),
        totalDuration = 160
    )

    /** Error or invalid action */
    val ERROR = WIAHapticPattern(
        id = "tb.error",
        name = "Error",
        description = "Invalid action or error",
        primitives = listOf(
            WIAHapticPrimitive(WIAWaveformType.SQUARE, 80, 0.7f, 100),
            WIAHapticPrimitive(WIAWaveformType.SQUARE, 80, 0.7f, 100, delay = 80)
        ),
        totalDuration = 280
    )

    /** End of content */
    val END_OF_CONTENT = WIAHapticPattern(
        id = "tb.boundary.end",
        name = "End of Content",
        description = "Reached end of scrollable content",
        primitives = listOf(
            WIAHapticPrimitive(WIAWaveformType.SQUARE, 100, 0.6f, 80)
        ),
        totalDuration = 80
    )
}

// MARK: - Haptic Engine

/**
 * WIA Haptic Engine for Android
 */
class WIAHapticEngine(private val context: Context) {

    private val vibrator: Vibrator by lazy {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            val vibratorManager = context.getSystemService(Context.VIBRATOR_MANAGER_SERVICE) as VibratorManager
            vibratorManager.defaultVibrator
        } else {
            @Suppress("DEPRECATION")
            context.getSystemService(Context.VIBRATOR_SERVICE) as Vibrator
        }
    }

    var isEnabled: Boolean = true
    var intensityMultiplier: Float = 1.0f

    /**
     * Check if device supports haptics
     */
    fun supportsHaptics(): Boolean {
        return vibrator.hasVibrator()
    }

    /**
     * Check if device supports amplitude control
     */
    fun supportsAmplitudeControl(): Boolean {
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            vibrator.hasAmplitudeControl()
        } else {
            false
        }
    }

    /**
     * Play a WIA haptic pattern
     */
    fun playPattern(pattern: WIAHapticPattern) {
        if (!isEnabled || !supportsHaptics()) return

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            playPatternApi26(pattern)
        } else {
            playPatternLegacy(pattern)
        }
    }

    @RequiresApi(Build.VERSION_CODES.O)
    private fun playPatternApi26(pattern: WIAHapticPattern) {
        val timings = mutableListOf<Long>()
        val amplitudes = mutableListOf<Int>()

        var currentDelay = 0L

        for (primitive in pattern.primitives) {
            // Add delay if present
            val delay = primitive.delay ?: 0L
            if (delay > 0 || currentDelay > 0) {
                timings.add(delay + currentDelay)
                amplitudes.add(0)
                currentDelay = 0
            }

            // Add vibration
            timings.add(primitive.duration)
            val amplitude = (primitive.intensity * intensityMultiplier * 255).toInt().coerceIn(0, 255)
            amplitudes.add(amplitude)
        }

        if (timings.isEmpty()) return

        val effect = VibrationEffect.createWaveform(
            timings.toLongArray(),
            amplitudes.toIntArray(),
            -1 // Don't repeat
        )

        vibrator.vibrate(effect)
    }

    @Suppress("DEPRECATION")
    private fun playPatternLegacy(pattern: WIAHapticPattern) {
        val timings = mutableListOf<Long>()

        var currentDelay = 0L

        for (primitive in pattern.primitives) {
            val delay = primitive.delay ?: 0L
            if (delay > 0 || currentDelay > 0) {
                timings.add(delay + currentDelay)
                currentDelay = 0
            } else if (timings.isEmpty()) {
                timings.add(0) // Initial delay
            }

            timings.add(primitive.duration)
        }

        if (timings.isEmpty()) return

        vibrator.vibrate(timings.toLongArray(), -1)
    }

    /**
     * Stop any ongoing vibration
     */
    fun stop() {
        vibrator.cancel()
    }
}

// MARK: - TalkBack Haptic Service

/**
 * TalkBack Haptic Integration Service
 *
 * This accessibility service provides haptic feedback for TalkBack events.
 *
 * To use:
 * 1. Declare in AndroidManifest.xml as an AccessibilityService
 * 2. Configure accessibility service metadata
 * 3. User enables service in Settings > Accessibility
 */
class TalkBackHapticService : AccessibilityService() {

    private lateinit var hapticEngine: WIAHapticEngine

    // Configuration
    var focusHapticsEnabled = true
    var actionHapticsEnabled = true
    var scrollHapticsEnabled = true
    var windowHapticsEnabled = true

    override fun onCreate() {
        super.onCreate()
        hapticEngine = WIAHapticEngine(this)
    }

    override fun onAccessibilityEvent(event: AccessibilityEvent?) {
        event ?: return

        when (event.eventType) {
            AccessibilityEvent.TYPE_VIEW_FOCUSED,
            AccessibilityEvent.TYPE_VIEW_ACCESSIBILITY_FOCUSED -> {
                if (focusHapticsEnabled) handleFocusEvent(event)
            }

            AccessibilityEvent.TYPE_VIEW_CLICKED -> {
                if (actionHapticsEnabled) handleClickEvent(event)
            }

            AccessibilityEvent.TYPE_VIEW_LONG_CLICKED -> {
                if (actionHapticsEnabled) hapticEngine.playPattern(TalkBackHapticPatterns.LONG_CLICK)
            }

            AccessibilityEvent.TYPE_VIEW_SCROLLED -> {
                if (scrollHapticsEnabled) handleScrollEvent(event)
            }

            AccessibilityEvent.TYPE_WINDOW_STATE_CHANGED -> {
                if (windowHapticsEnabled) hapticEngine.playPattern(TalkBackHapticPatterns.WINDOW_CHANGE)
            }

            AccessibilityEvent.TYPE_ANNOUNCEMENT -> {
                // Handle TalkBack announcements
            }
        }
    }

    private fun handleFocusEvent(event: AccessibilityEvent) {
        val source = event.source ?: run {
            hapticEngine.playPattern(TalkBackHapticPatterns.FOCUS_CHANGE)
            return
        }

        val pattern = when {
            source.className?.contains("Button") == true -> TalkBackHapticPatterns.FOCUS_BUTTON
            source.className?.contains("EditText") == true -> TalkBackHapticPatterns.FOCUS_EDIT_TEXT
            source.className?.contains("CheckBox") == true -> TalkBackHapticPatterns.FOCUS_CHECKBOX
            source.className?.contains("Switch") == true -> TalkBackHapticPatterns.FOCUS_CHECKBOX
            source.isClickable && source.className?.contains("TextView") == true -> TalkBackHapticPatterns.FOCUS_LINK
            else -> TalkBackHapticPatterns.FOCUS_CHANGE
        }

        hapticEngine.playPattern(pattern)
        source.recycle()
    }

    private fun handleClickEvent(event: AccessibilityEvent) {
        val source = event.source

        if (source != null) {
            // Check for checkbox/switch toggle
            if (source.className?.contains("CheckBox") == true ||
                source.className?.contains("Switch") == true) {
                val pattern = if (source.isChecked) {
                    TalkBackHapticPatterns.CHECKBOX_CHECKED
                } else {
                    TalkBackHapticPatterns.CHECKBOX_UNCHECKED
                }
                hapticEngine.playPattern(pattern)
                source.recycle()
                return
            }
            source.recycle()
        }

        hapticEngine.playPattern(TalkBackHapticPatterns.CLICK)
    }

    private fun handleScrollEvent(event: AccessibilityEvent) {
        // Check for end of content
        val source = event.source
        if (source != null) {
            val info = source.collectionInfo
            if (info != null) {
                // At end of scrollable content
                // Note: More sophisticated detection would be needed
            }
            source.recycle()
        }

        hapticEngine.playPattern(TalkBackHapticPatterns.SCROLL)
    }

    override fun onInterrupt() {
        hapticEngine.stop()
    }

    override fun onDestroy() {
        super.onDestroy()
        hapticEngine.stop()
    }

    // MARK: - Public Methods

    /**
     * Manually trigger haptic for reading start
     */
    fun playReadingStart() {
        hapticEngine.playPattern(TalkBackHapticPatterns.READING_START)
    }

    /**
     * Manually trigger haptic for reading stop
     */
    fun playReadingStop() {
        hapticEngine.playPattern(TalkBackHapticPatterns.READING_STOP)
    }

    /**
     * Manually trigger haptic for error
     */
    fun playError() {
        hapticEngine.playPattern(TalkBackHapticPatterns.ERROR)
    }

    /**
     * Manually trigger haptic for end of content
     */
    fun playEndOfContent() {
        hapticEngine.playPattern(TalkBackHapticPatterns.END_OF_CONTENT)
    }
}

// MARK: - Extension Functions

/**
 * Extension to play haptic when focusing an accessibility node
 */
fun AccessibilityNodeInfo.playFocusHaptic(engine: WIAHapticEngine) {
    val pattern = when {
        className?.contains("Button") == true -> TalkBackHapticPatterns.FOCUS_BUTTON
        className?.contains("EditText") == true -> TalkBackHapticPatterns.FOCUS_EDIT_TEXT
        className?.contains("CheckBox") == true -> TalkBackHapticPatterns.FOCUS_CHECKBOX
        isClickable -> TalkBackHapticPatterns.FOCUS_LINK
        else -> TalkBackHapticPatterns.FOCUS_CHANGE
    }
    engine.playPattern(pattern)
}
