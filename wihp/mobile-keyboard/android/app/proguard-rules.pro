# WIHP Keyboard ProGuard Rules

# Keep WIHPEngine
-keep class com.wia.wihp.keyboard.WIHPEngine { *; }

# Keep InputMethodService
-keep class com.wia.wihp.keyboard.WIHPInputMethodService { *; }

# Keep Activities
-keep class com.wia.wihp.keyboard.MainActivity { *; }
-keep class com.wia.wihp.keyboard.SettingsActivity { *; }

# Kotlin
-dontwarn kotlin.**
-keep class kotlin.** { *; }
