//! Basic Usage Example
//! 弘益人間 - Gaming for Everyone
//!
//! This example demonstrates basic usage of the WIA Game API.

use wia_game::{GameController, types::*};

fn main() {
    println!("🎮 WIA Game Accessibility API - Basic Usage\n");

    // Create a game controller
    let mut controller = GameController::new();

    // ========================================================================
    // 1. Create profiles for different disabilities
    // ========================================================================
    println!("1. Creating accessibility profiles...\n");

    // Profile for a blind player
    let blind_profile = controller.create_profile_for_disability(DisabilityType::Blind);
    println!("   Blind Profile:");
    println!("   - Screen Reader: {}", blind_profile.visual_settings.screen_reader.enabled);
    println!("   - Auto Aim: {}", blind_profile.motor_settings.aim_assist.auto_aim);
    println!("   - Aim Assist: {:?}", blind_profile.motor_settings.aim_assist.strength);

    // Profile for a deaf player
    let deaf_profile = controller.create_profile_for_disability(DisabilityType::Deaf);
    println!("\n   Deaf Profile:");
    println!("   - Subtitles: {}", deaf_profile.audio_settings.subtitles.enabled);
    println!("   - Visual Sound Cues: {}", deaf_profile.audio_settings.visual_sound_cues.enabled);
    println!("   - Haptic Audio: {}", deaf_profile.audio_settings.haptic_audio.enabled);

    // Profile for one-handed player
    let one_handed_profile = controller.create_profile_for_disability(DisabilityType::OneHanded);
    println!("\n   One-Handed Profile:");
    println!("   - One-Handed Mode: {}", one_handed_profile.motor_settings.one_handed_mode.enabled);
    println!("   - Hand: {:?}", one_handed_profile.motor_settings.one_handed_mode.hand);
    println!("   - Auto Run: {}", one_handed_profile.motor_settings.auto_actions.auto_run);

    // ========================================================================
    // 2. System presets
    // ========================================================================
    println!("\n2. Available System Presets:\n");

    let presets = controller.get_system_presets();
    for preset in presets {
        println!("   - {} ({:?})", preset.name, preset.target_disability);
    }

    // ========================================================================
    // 3. Apply preset to profile
    // ========================================================================
    println!("\n3. Applying preset to profile...\n");

    let mut profile = controller.create_profile();
    let preset_id = controller.find_presets_for_disability(DisabilityType::Cognitive)[0].preset_id;

    controller.apply_preset(profile.profile_id, preset_id).unwrap();

    let updated = controller.get_profile(profile.profile_id).unwrap();
    println!("   After applying Cognitive Support preset:");
    println!("   - Difficulty: {:?}", updated.cognitive_settings.difficulty.combat_difficulty);
    println!("   - Hint System: {:?}", updated.cognitive_settings.guidance.hint_system);
    println!("   - Simplified Controls: {}", updated.cognitive_settings.simplification.simplified_controls);

    // ========================================================================
    // 4. Export/Import profiles
    // ========================================================================
    println!("\n4. Profile Export/Import...\n");

    let json = controller.export_profile(blind_profile.profile_id).unwrap();
    println!("   Exported profile (first 200 chars):");
    println!("   {}", &json[..200.min(json.len())]);

    let imported = controller.import_profile(&json).unwrap();
    println!("\n   Imported profile ID: {}", imported.profile_id);
    println!("   (New ID assigned, settings preserved)");

    // ========================================================================
    // 5. Validation
    // ========================================================================
    println!("\n5. Profile Validation...\n");

    let valid_profile = PlayerProfile::default();
    match GameController::validate_profile(&valid_profile) {
        Ok(_) => println!("   ✅ Default profile is valid"),
        Err(e) => println!("   ❌ Validation failed: {}", e),
    }

    println!("\n🤟 弘益人間 - Gaming for Everyone\n");
}
