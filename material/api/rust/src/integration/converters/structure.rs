//! Structure conversion utilities
//!
//! Provides utilities for converting between different structure representations.

use crate::integration::error::{IntegrationError, IntegrationResult};
use crate::types::{CrystalSystem, LatticeParameters};

/// Structure conversion utilities
pub struct StructureConverter;

impl StructureConverter {
    /// Convert fractional coordinates to Cartesian
    pub fn fractional_to_cartesian(
        frac: (f64, f64, f64),
        params: &LatticeParameters,
    ) -> IntegrationResult<(f64, f64, f64)> {
        let a = params.a_angstrom.ok_or(IntegrationError::ConversionError(
            "Missing lattice parameter a".to_string(),
        ))?;
        let b = params.b_angstrom.ok_or(IntegrationError::ConversionError(
            "Missing lattice parameter b".to_string(),
        ))?;
        let c = params.c_angstrom.ok_or(IntegrationError::ConversionError(
            "Missing lattice parameter c".to_string(),
        ))?;

        let alpha = params.alpha_degree.unwrap_or(90.0).to_radians();
        let beta = params.beta_degree.unwrap_or(90.0).to_radians();
        let gamma = params.gamma_degree.unwrap_or(90.0).to_radians();

        let cos_alpha = alpha.cos();
        let cos_beta = beta.cos();
        let cos_gamma = gamma.cos();
        let sin_gamma = gamma.sin();

        // Transformation matrix elements
        let v = (1.0 - cos_alpha.powi(2) - cos_beta.powi(2) - cos_gamma.powi(2)
            + 2.0 * cos_alpha * cos_beta * cos_gamma)
            .sqrt();

        let x = a * frac.0 + b * cos_gamma * frac.1 + c * cos_beta * frac.2;
        let y = b * sin_gamma * frac.1
            + c * (cos_alpha - cos_beta * cos_gamma) / sin_gamma * frac.2;
        let z = c * v / sin_gamma * frac.2;

        Ok((x, y, z))
    }

    /// Convert Cartesian coordinates to fractional
    pub fn cartesian_to_fractional(
        cart: (f64, f64, f64),
        params: &LatticeParameters,
    ) -> IntegrationResult<(f64, f64, f64)> {
        let a = params.a_angstrom.ok_or(IntegrationError::ConversionError(
            "Missing lattice parameter a".to_string(),
        ))?;
        let b = params.b_angstrom.ok_or(IntegrationError::ConversionError(
            "Missing lattice parameter b".to_string(),
        ))?;
        let c = params.c_angstrom.ok_or(IntegrationError::ConversionError(
            "Missing lattice parameter c".to_string(),
        ))?;

        let alpha = params.alpha_degree.unwrap_or(90.0).to_radians();
        let beta = params.beta_degree.unwrap_or(90.0).to_radians();
        let gamma = params.gamma_degree.unwrap_or(90.0).to_radians();

        let cos_alpha = alpha.cos();
        let cos_beta = beta.cos();
        let cos_gamma = gamma.cos();
        let sin_gamma = gamma.sin();

        let v = (1.0 - cos_alpha.powi(2) - cos_beta.powi(2) - cos_gamma.powi(2)
            + 2.0 * cos_alpha * cos_beta * cos_gamma)
            .sqrt();

        // Inverse transformation
        let z_frac = cart.2 * sin_gamma / (c * v);
        let y_frac = (cart.1 - c * (cos_alpha - cos_beta * cos_gamma) / sin_gamma * z_frac)
            / (b * sin_gamma);
        let x_frac = (cart.0 - b * cos_gamma * y_frac - c * cos_beta * z_frac) / a;

        Ok((x_frac, y_frac, z_frac))
    }

    /// Calculate unit cell volume from lattice parameters
    pub fn calculate_volume(params: &LatticeParameters) -> IntegrationResult<f64> {
        let a = params.a_angstrom.unwrap_or(1.0);
        let b = params.b_angstrom.unwrap_or(1.0);
        let c = params.c_angstrom.unwrap_or(1.0);
        let alpha = params.alpha_degree.unwrap_or(90.0).to_radians();
        let beta = params.beta_degree.unwrap_or(90.0).to_radians();
        let gamma = params.gamma_degree.unwrap_or(90.0).to_radians();

        let cos_alpha = alpha.cos();
        let cos_beta = beta.cos();
        let cos_gamma = gamma.cos();

        let volume = a
            * b
            * c
            * (1.0 - cos_alpha.powi(2) - cos_beta.powi(2) - cos_gamma.powi(2)
                + 2.0 * cos_alpha * cos_beta * cos_gamma)
                .sqrt();

        Ok(volume)
    }

    /// Determine crystal system from lattice parameters
    pub fn determine_crystal_system(params: &LatticeParameters) -> Option<CrystalSystem> {
        let a = params.a_angstrom?;
        let b = params.b_angstrom?;
        let c = params.c_angstrom?;
        let alpha = params.alpha_degree.unwrap_or(90.0);
        let beta = params.beta_degree.unwrap_or(90.0);
        let gamma = params.gamma_degree.unwrap_or(90.0);

        const ANGLE_TOL: f64 = 0.5; // degrees
        const LENGTH_TOL: f64 = 0.01; // relative

        let angles_90 = |angle: f64| (angle - 90.0).abs() < ANGLE_TOL;
        let angles_120 = |angle: f64| (angle - 120.0).abs() < ANGLE_TOL;
        let lengths_equal = |l1: f64, l2: f64| (l1 - l2).abs() / l1 < LENGTH_TOL;

        // Check crystal systems in order of specificity
        if lengths_equal(a, b) && lengths_equal(b, c) {
            if angles_90(alpha) && angles_90(beta) && angles_90(gamma) {
                return Some(CrystalSystem::Cubic);
            }
            if (alpha - beta).abs() < ANGLE_TOL
                && (beta - gamma).abs() < ANGLE_TOL
                && !angles_90(alpha)
            {
                return Some(CrystalSystem::Trigonal);
            }
        }

        if lengths_equal(a, b) && !lengths_equal(b, c) {
            if angles_90(alpha) && angles_90(beta) {
                if angles_90(gamma) {
                    return Some(CrystalSystem::Tetragonal);
                }
                if angles_120(gamma) {
                    return Some(CrystalSystem::Hexagonal);
                }
            }
        }

        if !lengths_equal(a, b) && !lengths_equal(b, c) && !lengths_equal(a, c) {
            if angles_90(alpha) && angles_90(beta) && angles_90(gamma) {
                return Some(CrystalSystem::Orthorhombic);
            }
            if angles_90(alpha) && !angles_90(beta) && angles_90(gamma) {
                return Some(CrystalSystem::Monoclinic);
            }
        }

        // Default to triclinic if no other system matches
        Some(CrystalSystem::Triclinic)
    }

    /// Calculate distance between two positions
    pub fn distance(
        pos1: (f64, f64, f64),
        pos2: (f64, f64, f64),
        params: Option<&LatticeParameters>,
    ) -> f64 {
        if let Some(p) = params {
            // Use lattice-aware distance (simplified: assumes orthogonal cell)
            let a = p.a_angstrom.unwrap_or(1.0);
            let b = p.b_angstrom.unwrap_or(1.0);
            let c = p.c_angstrom.unwrap_or(1.0);

            let dx = (pos1.0 - pos2.0) * a;
            let dy = (pos1.1 - pos2.1) * b;
            let dz = (pos1.2 - pos2.2) * c;

            (dx * dx + dy * dy + dz * dz).sqrt()
        } else {
            // Simple Euclidean distance
            let dx = pos1.0 - pos2.0;
            let dy = pos1.1 - pos2.1;
            let dz = pos1.2 - pos2.2;

            (dx * dx + dy * dy + dz * dz).sqrt()
        }
    }

    /// Generate lattice vectors from parameters
    pub fn lattice_vectors_from_params(
        params: &LatticeParameters,
    ) -> IntegrationResult<((f64, f64, f64), (f64, f64, f64), (f64, f64, f64))> {
        let a = params.a_angstrom.ok_or(IntegrationError::ConversionError(
            "Missing lattice parameter a".to_string(),
        ))?;
        let b = params.b_angstrom.ok_or(IntegrationError::ConversionError(
            "Missing lattice parameter b".to_string(),
        ))?;
        let c = params.c_angstrom.ok_or(IntegrationError::ConversionError(
            "Missing lattice parameter c".to_string(),
        ))?;

        let alpha = params.alpha_degree.unwrap_or(90.0).to_radians();
        let beta = params.beta_degree.unwrap_or(90.0).to_radians();
        let gamma = params.gamma_degree.unwrap_or(90.0).to_radians();

        let cos_alpha = alpha.cos();
        let cos_beta = beta.cos();
        let cos_gamma = gamma.cos();
        let sin_gamma = gamma.sin();

        let v1 = (a, 0.0, 0.0);
        let v2 = (b * cos_gamma, b * sin_gamma, 0.0);

        let c_x = c * cos_beta;
        let c_y = c * (cos_alpha - cos_beta * cos_gamma) / sin_gamma;
        let c_z = (c * c - c_x * c_x - c_y * c_y).sqrt();
        let v3 = (c_x, c_y, c_z);

        Ok((v1, v2, v3))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cubic_params() -> LatticeParameters {
        LatticeParameters {
            a_angstrom: Some(5.0),
            b_angstrom: Some(5.0),
            c_angstrom: Some(5.0),
            alpha_degree: Some(90.0),
            beta_degree: Some(90.0),
            gamma_degree: Some(90.0),
        }
    }

    #[test]
    fn test_fractional_to_cartesian_cubic() {
        let params = cubic_params();

        let cart = StructureConverter::fractional_to_cartesian((0.5, 0.5, 0.5), &params).unwrap();

        assert!((cart.0 - 2.5).abs() < 0.001);
        assert!((cart.1 - 2.5).abs() < 0.001);
        assert!((cart.2 - 2.5).abs() < 0.001);
    }

    #[test]
    fn test_cartesian_to_fractional_cubic() {
        let params = cubic_params();

        let frac = StructureConverter::cartesian_to_fractional((2.5, 2.5, 2.5), &params).unwrap();

        assert!((frac.0 - 0.5).abs() < 0.001);
        assert!((frac.1 - 0.5).abs() < 0.001);
        assert!((frac.2 - 0.5).abs() < 0.001);
    }

    #[test]
    fn test_roundtrip_conversion() {
        let params = LatticeParameters {
            a_angstrom: Some(5.0),
            b_angstrom: Some(6.0),
            c_angstrom: Some(7.0),
            alpha_degree: Some(80.0),
            beta_degree: Some(85.0),
            gamma_degree: Some(90.0),
        };

        let original = (0.3, 0.4, 0.5);
        let cart = StructureConverter::fractional_to_cartesian(original, &params).unwrap();
        let back = StructureConverter::cartesian_to_fractional(cart, &params).unwrap();

        assert!((back.0 - original.0).abs() < 0.001);
        assert!((back.1 - original.1).abs() < 0.001);
        assert!((back.2 - original.2).abs() < 0.001);
    }

    #[test]
    fn test_calculate_volume_cubic() {
        let params = cubic_params();
        let volume = StructureConverter::calculate_volume(&params).unwrap();

        assert!((volume - 125.0).abs() < 0.001);
    }

    #[test]
    fn test_determine_crystal_system() {
        // Cubic
        let cubic = LatticeParameters {
            a_angstrom: Some(5.0),
            b_angstrom: Some(5.0),
            c_angstrom: Some(5.0),
            alpha_degree: Some(90.0),
            beta_degree: Some(90.0),
            gamma_degree: Some(90.0),
        };
        assert_eq!(
            StructureConverter::determine_crystal_system(&cubic),
            Some(CrystalSystem::Cubic)
        );

        // Tetragonal
        let tetra = LatticeParameters {
            a_angstrom: Some(5.0),
            b_angstrom: Some(5.0),
            c_angstrom: Some(7.0),
            alpha_degree: Some(90.0),
            beta_degree: Some(90.0),
            gamma_degree: Some(90.0),
        };
        assert_eq!(
            StructureConverter::determine_crystal_system(&tetra),
            Some(CrystalSystem::Tetragonal)
        );

        // Orthorhombic
        let ortho = LatticeParameters {
            a_angstrom: Some(5.0),
            b_angstrom: Some(6.0),
            c_angstrom: Some(7.0),
            alpha_degree: Some(90.0),
            beta_degree: Some(90.0),
            gamma_degree: Some(90.0),
        };
        assert_eq!(
            StructureConverter::determine_crystal_system(&ortho),
            Some(CrystalSystem::Orthorhombic)
        );
    }

    #[test]
    fn test_distance() {
        let params = cubic_params();

        let dist = StructureConverter::distance((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), Some(&params));

        assert!((dist - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_lattice_vectors_from_params() {
        let params = cubic_params();

        let (v1, v2, v3) = StructureConverter::lattice_vectors_from_params(&params).unwrap();

        assert!((v1.0 - 5.0).abs() < 0.001);
        assert!((v2.1 - 5.0).abs() < 0.001);
        assert!((v3.2 - 5.0).abs() < 0.001);
    }
}
