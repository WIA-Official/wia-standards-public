//! WIHP data types

#[cfg(not(feature = "std"))]
use alloc::string::String;
#[cfg(not(feature = "std"))]
use alloc::vec::Vec;

/// A WIHP symbol mapping
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WihpSymbol {
    /// IPA character
    pub ipa: char,

    /// WIHP systematic code (e.g., "PL01-MN01-AR01")
    pub code: String,

    /// Korean Hangul representation
    pub hangul: String,

    /// Symbol category
    pub category: SymbolCategory,

    /// Detailed subcategory (e.g., "plosive", "fricative")
    pub subcategory: String,

    /// Description in Korean
    pub description_ko: String,

    /// Description in English
    pub description_en: String,

    /// Example languages using this sound
    pub examples: Vec<String>,
}

impl WihpSymbol {
    /// Create a new WIHP symbol
    pub fn new(
        ipa: char,
        code: &str,
        hangul: &str,
        category: SymbolCategory,
        subcategory: &str,
        description_ko: &str,
        description_en: &str,
    ) -> Self {
        Self {
            ipa,
            code: code.into(),
            hangul: hangul.into(),
            category,
            subcategory: subcategory.into(),
            description_ko: description_ko.into(),
            description_en: description_en.into(),
            examples: Vec::new(),
        }
    }

    /// Add example languages
    pub fn with_examples(mut self, examples: &[&str]) -> Self {
        self.examples = examples.iter().map(|s| (*s).into()).collect();
        self
    }
}

/// Symbol category
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum SymbolCategory {
    /// Consonant (자음)
    Consonant,
    /// Vowel (모음)
    Vowel,
    /// Diacritic (다이아크리틱)
    Diacritic,
    /// Suprasegmental (초분절음)
    Suprasegmental,
}

impl SymbolCategory {
    /// Get Korean name
    pub fn korean_name(&self) -> &'static str {
        match self {
            Self::Consonant => "자음",
            Self::Vowel => "모음",
            Self::Diacritic => "다이아크리틱",
            Self::Suprasegmental => "초분절음",
        }
    }

    /// Get English name
    pub fn english_name(&self) -> &'static str {
        match self {
            Self::Consonant => "Consonant",
            Self::Vowel => "Vowel",
            Self::Diacritic => "Diacritic",
            Self::Suprasegmental => "Suprasegmental",
        }
    }
}

/// Place of articulation
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PlaceOfArticulation {
    Bilabial = 1,
    Labiodental = 2,
    Dental = 3,
    Alveolar = 4,
    Postalveolar = 5,
    Retroflex = 6,
    Palatal = 7,
    Velar = 8,
    Uvular = 9,
    Pharyngeal = 10,
    Glottal = 11,
}

/// Manner of articulation
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MannerOfArticulation {
    Plosive = 1,
    Nasal = 2,
    Trill = 3,
    TapFlap = 4,
    Fricative = 5,
    LateralFricative = 6,
    Approximant = 7,
    LateralApproximant = 8,
}

/// Voicing
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Voicing {
    Voiceless = 1,
    Voiced = 2,
    Aspirated = 3,
    Ejective = 4,
    Click = 5,
    Implosive = 6,
}

/// Tone level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ToneLevel {
    /// ˥ High (5)
    High = 5,
    /// ˦ Mid-High (4)
    MidHigh = 4,
    /// ˧ Mid (3)
    Mid = 3,
    /// ˨ Mid-Low (2)
    MidLow = 2,
    /// ˩ Low (1)
    Low = 1,
}

impl ToneLevel {
    /// Get IPA character
    pub fn ipa_char(&self) -> char {
        match self {
            Self::High => '˥',
            Self::MidHigh => '˦',
            Self::Mid => '˧',
            Self::MidLow => '˨',
            Self::Low => '˩',
        }
    }

    /// Get numeric representation
    pub fn number(&self) -> u8 {
        *self as u8
    }
}
