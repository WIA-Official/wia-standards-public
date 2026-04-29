//! Utility functions for cosmic communication

use crate::{
    error::{Error, Result},
    types::*,
};
use std::f64::consts::PI;

/// Convert frequency to wavelength
pub fn frequency_to_wavelength(frequency: f64) -> f64 {
    const SPEED_OF_LIGHT: f64 = 299792458.0; // m/s
    SPEED_OF_LIGHT / frequency
}

/// Convert wavelength to frequency
pub fn wavelength_to_frequency(wavelength: f64) -> f64 {
    const SPEED_OF_LIGHT: f64 = 299792458.0; // m/s
    SPEED_OF_LIGHT / wavelength
}

/// Calculate signal-to-noise ratio
pub fn calculate_snr(signal_strength: f64, noise_level: f64) -> f64 {
    10.0 * (signal_strength / noise_level).log10()
}

/// Convert equatorial to galactic coordinates
pub fn equatorial_to_galactic(ra: f64, dec: f64) -> (f64, f64) {
    // Simplified conversion (approximate)
    let ra_rad = ra * 15.0 * PI / 180.0; // Convert hours to radians
    let dec_rad = dec * PI / 180.0;

    // North Galactic Pole
    let ngp_ra = 192.859508 * PI / 180.0;
    let ngp_dec = 27.128336 * PI / 180.0;
    let gal_center = 122.932 * PI / 180.0;

    let sin_b = dec_rad.sin() * ngp_dec.sin()
        + dec_rad.cos() * ngp_dec.cos() * (ra_rad - ngp_ra).cos();
    let b = sin_b.asin();

    let y = dec_rad.cos() * (ra_rad - ngp_ra).sin();
    let x = dec_rad.sin() * ngp_dec.cos()
        - dec_rad.cos() * ngp_dec.sin() * (ra_rad - ngp_ra).cos();
    let l = (gal_center - y.atan2(x)) * 180.0 / PI;

    let l = if l < 0.0 { l + 360.0 } else { l };
    let b = b * 180.0 / PI;

    (l, b)
}

/// Calculate distance in parsecs
pub fn light_years_to_parsecs(light_years: f64) -> f64 {
    light_years * 0.306601
}

/// Calculate distance in light years
pub fn parsecs_to_light_years(parsecs: f64) -> f64 {
    parsecs * 3.26156
}

/// Estimate transmission time
pub fn estimate_transmission_time(distance_ly: f64) -> f64 {
    distance_ly // Time in years for light to travel
}

/// Calculate Drake equation probability
pub fn drake_equation(
    star_formation_rate: f64,
    fraction_with_planets: f64,
    planets_per_system: f64,
    fraction_with_life: f64,
    fraction_intelligent: f64,
    fraction_communicating: f64,
    civilization_lifetime: f64,
) -> f64 {
    star_formation_rate
        * fraction_with_planets
        * planets_per_system
        * fraction_with_life
        * fraction_intelligent
        * fraction_communicating
        * civilization_lifetime
}

/// Format coordinate as string
pub fn format_coordinate(coord: &CosmicCoordinate) -> String {
    format!(
        "RA: {:.2}h, Dec: {:.2}°, Dist: {:.2} ly",
        coord.right_ascension, coord.declination, coord.distance
    )
}

/// Generate universal greeting message
pub fn generate_greeting() -> UniversalMessage {
    use chrono::Utc;
    use uuid::Uuid;

    UniversalMessage {
        message_id: Uuid::new_v4().to_string(),
        message_type: MessageType::Greeting,
        content: "Hello from Earth! We come in peace. 홍익인간 (弘益人間)".to_string(),
        encoding: EncodingFormat::Binary,
        protocol_version: "1.0".to_string(),
        created_at: Utc::now(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frequency_wavelength_conversion() {
        let freq = 1420e6; // Hydrogen line (Hz)
        let wavelength = frequency_to_wavelength(freq);
        assert!((wavelength - 0.211).abs() < 0.01); // ~21.1 cm
    }

    #[test]
    fn test_snr_calculation() {
        let snr = calculate_snr(100.0, 10.0);
        assert!((snr - 10.0).abs() < 0.01);
    }

    #[test]
    fn test_distance_conversion() {
        let ly = 100.0;
        let pc = light_years_to_parsecs(ly);
        let ly2 = parsecs_to_light_years(pc);
        assert!((ly - ly2).abs() < 0.01);
    }

    #[test]
    fn test_drake_equation() {
        let n = drake_equation(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1000.0);
        assert_eq!(n, 1000.0);
    }

    #[test]
    fn test_generate_greeting() {
        let greeting = generate_greeting();
        assert_eq!(greeting.message_type, MessageType::Greeting);
        assert!(!greeting.content.is_empty());
    }
}
