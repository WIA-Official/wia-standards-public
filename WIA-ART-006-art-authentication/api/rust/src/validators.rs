//! Validation functions for art authentication

use crate::error::{Error, Result};
use crate::types::*;

pub fn validate_artwork(artwork: &Artwork) -> Result<()> {
    if artwork.title.is_empty() {
        return Err(Error::InvalidInput("Title cannot be empty".into()));
    }
    if artwork.artist.is_empty() {
        return Err(Error::InvalidInput("Artist cannot be empty".into()));
    }
    Ok(())
}

pub fn validate_certificate(cert: &Certificate) -> Result<()> {
    if cert.issuer.is_empty() {
        return Err(Error::InvalidInput("Issuer cannot be empty".into()));
    }
    Ok(())
}
