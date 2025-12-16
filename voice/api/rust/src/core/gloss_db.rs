//! Sign language gloss database

use std::collections::HashMap;

use crate::types::*;

/// Entry in the gloss database
#[derive(Debug, Clone)]
pub struct GlossEntry {
    /// The gloss identifier (e.g., "HELLO")
    pub gloss: String,

    /// Sign type
    pub sign_type: SignType,

    /// HamNoSys notation
    pub hamnosys: Option<String>,

    /// SiGML notation
    pub sigml: Option<String>,

    /// SignWriting notation
    pub signwriting: Option<String>,

    /// Alternative glosses (synonyms)
    pub alternatives: Vec<String>,

    /// Default duration in milliseconds
    pub default_duration_ms: f64,
}

/// Gloss database for a specific sign language
pub struct GlossDatabase {
    language: SignLanguageCode,
    entries: HashMap<String, GlossEntry>,
    word_to_gloss: HashMap<String, Vec<String>>,
}

impl GlossDatabase {
    /// Create a new gloss database
    pub fn new(language: SignLanguageCode) -> Self {
        let mut db = Self {
            language,
            entries: HashMap::new(),
            word_to_gloss: HashMap::new(),
        };
        db.load_default_entries();
        db
    }

    /// Load default entries for the language
    fn load_default_entries(&mut self) {
        match self.language {
            SignLanguageCode::Asl => self.load_asl_entries(),
            SignLanguageCode::Ksl => self.load_ksl_entries(),
            _ => self.load_generic_entries(),
        }
    }

    /// Load ASL entries
    fn load_asl_entries(&mut self) {
        let entries = vec![
            GlossEntry {
                gloss: "HELLO".to_string(),
                sign_type: SignType::Lexical,
                hamnosys: Some("hamfinger2,hamextfingeru,hampalml".to_string()),
                sigml: None,
                signwriting: None,
                alternatives: vec!["HI".to_string()],
                default_duration_ms: 500.0,
            },
            GlossEntry {
                gloss: "THANK-YOU".to_string(),
                sign_type: SignType::Lexical,
                hamnosys: Some("hamflathand,hamextfingerr,hampalmr".to_string()),
                sigml: None,
                signwriting: None,
                alternatives: vec!["THANKS".to_string()],
                default_duration_ms: 600.0,
            },
            GlossEntry {
                gloss: "YES".to_string(),
                sign_type: SignType::Lexical,
                hamnosys: Some("hamfist,hampalmr".to_string()),
                sigml: None,
                signwriting: None,
                alternatives: vec![],
                default_duration_ms: 400.0,
            },
            GlossEntry {
                gloss: "NO".to_string(),
                sign_type: SignType::Lexical,
                hamnosys: Some("hamfinger2,hampalmr".to_string()),
                sigml: None,
                signwriting: None,
                alternatives: vec![],
                default_duration_ms: 400.0,
            },
            GlossEntry {
                gloss: "PLEASE".to_string(),
                sign_type: SignType::Lexical,
                hamnosys: Some("hamflathand,hamchest".to_string()),
                sigml: None,
                signwriting: None,
                alternatives: vec![],
                default_duration_ms: 500.0,
            },
            GlossEntry {
                gloss: "SORRY".to_string(),
                sign_type: SignType::Lexical,
                hamnosys: Some("hamfist,hamchest,hamcircle".to_string()),
                sigml: None,
                signwriting: None,
                alternatives: vec!["APOLOGIZE".to_string()],
                default_duration_ms: 600.0,
            },
            GlossEntry {
                gloss: "HELP".to_string(),
                sign_type: SignType::Lexical,
                hamnosys: Some("hamflathand,hamfist".to_string()),
                sigml: None,
                signwriting: None,
                alternatives: vec!["ASSIST".to_string()],
                default_duration_ms: 500.0,
            },
            GlossEntry {
                gloss: "I".to_string(),
                sign_type: SignType::Pointing,
                hamnosys: Some("hamfinger1,hamchest".to_string()),
                sigml: None,
                signwriting: None,
                alternatives: vec!["ME".to_string()],
                default_duration_ms: 300.0,
            },
            GlossEntry {
                gloss: "YOU".to_string(),
                sign_type: SignType::Pointing,
                hamnosys: Some("hamfinger1,hamforward".to_string()),
                sigml: None,
                signwriting: None,
                alternatives: vec![],
                default_duration_ms: 300.0,
            },
            GlossEntry {
                gloss: "WHAT".to_string(),
                sign_type: SignType::Lexical,
                hamnosys: Some("hamfinger5,hampalmu".to_string()),
                sigml: None,
                signwriting: None,
                alternatives: vec![],
                default_duration_ms: 400.0,
            },
            GlossEntry {
                gloss: "HOW".to_string(),
                sign_type: SignType::Lexical,
                hamnosys: Some("hamfist,hamfist,hampalmd".to_string()),
                sigml: None,
                signwriting: None,
                alternatives: vec![],
                default_duration_ms: 500.0,
            },
            GlossEntry {
                gloss: "GOOD".to_string(),
                sign_type: SignType::Lexical,
                hamnosys: Some("hamflathand,hammouth,hamdown".to_string()),
                sigml: None,
                signwriting: None,
                alternatives: vec!["WELL".to_string()],
                default_duration_ms: 500.0,
            },
            GlossEntry {
                gloss: "NAME".to_string(),
                sign_type: SignType::Lexical,
                hamnosys: Some("hamfinger2,hamtouch".to_string()),
                sigml: None,
                signwriting: None,
                alternatives: vec!["CALLED".to_string()],
                default_duration_ms: 500.0,
            },
        ];

        for entry in entries {
            self.add_entry(entry);
        }

        // Word mappings
        self.add_word_mapping("hello", vec!["HELLO"]);
        self.add_word_mapping("hi", vec!["HELLO"]);
        self.add_word_mapping("thank", vec!["THANK-YOU"]);
        self.add_word_mapping("thanks", vec!["THANK-YOU"]);
        self.add_word_mapping("yes", vec!["YES"]);
        self.add_word_mapping("no", vec!["NO"]);
        self.add_word_mapping("please", vec!["PLEASE"]);
        self.add_word_mapping("sorry", vec!["SORRY"]);
        self.add_word_mapping("help", vec!["HELP"]);
        self.add_word_mapping("i", vec!["I"]);
        self.add_word_mapping("me", vec!["I"]);
        self.add_word_mapping("you", vec!["YOU"]);
        self.add_word_mapping("what", vec!["WHAT"]);
        self.add_word_mapping("how", vec!["HOW"]);
        self.add_word_mapping("good", vec!["GOOD"]);
        self.add_word_mapping("well", vec!["GOOD"]);
        self.add_word_mapping("name", vec!["NAME"]);
        self.add_word_mapping("are", vec![]); // Function word, often omitted
    }

    /// Load KSL (Korean Sign Language) entries
    fn load_ksl_entries(&mut self) {
        let entries = vec![
            GlossEntry {
                gloss: "안녕".to_string(),
                sign_type: SignType::Lexical,
                hamnosys: Some("hamflathand,hamwave".to_string()),
                sigml: None,
                signwriting: None,
                alternatives: vec!["인사".to_string()],
                default_duration_ms: 500.0,
            },
            GlossEntry {
                gloss: "감사".to_string(),
                sign_type: SignType::Lexical,
                hamnosys: Some("hamflathand,hamforehead,hamdown".to_string()),
                sigml: None,
                signwriting: None,
                alternatives: vec!["고맙다".to_string()],
                default_duration_ms: 600.0,
            },
            GlossEntry {
                gloss: "네".to_string(),
                sign_type: SignType::Lexical,
                hamnosys: Some("hamfist,hamnod".to_string()),
                sigml: None,
                signwriting: None,
                alternatives: vec!["예".to_string()],
                default_duration_ms: 400.0,
            },
            GlossEntry {
                gloss: "아니오".to_string(),
                sign_type: SignType::Lexical,
                hamnosys: Some("hamfinger1,hamshake".to_string()),
                sigml: None,
                signwriting: None,
                alternatives: vec![],
                default_duration_ms: 400.0,
            },
            GlossEntry {
                gloss: "나".to_string(),
                sign_type: SignType::Pointing,
                hamnosys: Some("hamfinger1,hamchest".to_string()),
                sigml: None,
                signwriting: None,
                alternatives: vec!["저".to_string()],
                default_duration_ms: 300.0,
            },
        ];

        for entry in entries {
            self.add_entry(entry);
        }

        // Korean word mappings
        self.add_word_mapping("안녕", vec!["안녕"]);
        self.add_word_mapping("안녕하세요", vec!["안녕"]);
        self.add_word_mapping("감사합니다", vec!["감사"]);
        self.add_word_mapping("고맙습니다", vec!["감사"]);
        self.add_word_mapping("네", vec!["네"]);
        self.add_word_mapping("예", vec!["네"]);
        self.add_word_mapping("아니오", vec!["아니오"]);
        self.add_word_mapping("아니요", vec!["아니오"]);
        self.add_word_mapping("나", vec!["나"]);
        self.add_word_mapping("저", vec!["나"]);
    }

    /// Load generic entries for other languages
    fn load_generic_entries(&mut self) {
        // Basic universal concepts
        let entries = vec![
            GlossEntry {
                gloss: "HELLO".to_string(),
                sign_type: SignType::Lexical,
                hamnosys: None,
                sigml: None,
                signwriting: None,
                alternatives: vec![],
                default_duration_ms: 500.0,
            },
            GlossEntry {
                gloss: "YES".to_string(),
                sign_type: SignType::Lexical,
                hamnosys: None,
                sigml: None,
                signwriting: None,
                alternatives: vec![],
                default_duration_ms: 400.0,
            },
            GlossEntry {
                gloss: "NO".to_string(),
                sign_type: SignType::Lexical,
                hamnosys: None,
                sigml: None,
                signwriting: None,
                alternatives: vec![],
                default_duration_ms: 400.0,
            },
        ];

        for entry in entries {
            self.add_entry(entry);
        }
    }

    /// Add an entry to the database
    pub fn add_entry(&mut self, entry: GlossEntry) {
        let gloss = entry.gloss.clone();
        self.entries.insert(gloss, entry);
    }

    /// Add a word to gloss mapping
    pub fn add_word_mapping(&mut self, word: &str, glosses: Vec<&str>) {
        self.word_to_gloss.insert(
            word.to_lowercase(),
            glosses.iter().map(|s| s.to_string()).collect(),
        );
    }

    /// Look up a gloss by identifier
    pub fn get(&self, gloss: &str) -> Option<&GlossEntry> {
        self.entries.get(gloss)
    }

    /// Look up glosses for a word
    pub fn lookup_word(&self, word: &str) -> Vec<&GlossEntry> {
        let lower = word.to_lowercase();
        if let Some(glosses) = self.word_to_gloss.get(&lower) {
            glosses
                .iter()
                .filter_map(|g| self.entries.get(g))
                .collect()
        } else {
            // Try to find by gloss name directly
            if let Some(entry) = self.entries.get(&word.to_uppercase()) {
                vec![entry]
            } else {
                vec![]
            }
        }
    }

    /// Get the language code
    pub fn language(&self) -> SignLanguageCode {
        self.language
    }

    /// Get entry count
    pub fn entry_count(&self) -> usize {
        self.entries.len()
    }

    /// Check if a word has a sign
    pub fn has_sign(&self, word: &str) -> bool {
        !self.lookup_word(word).is_empty()
    }
}

/// Multi-language gloss database manager
pub struct GlossDatabaseManager {
    databases: HashMap<SignLanguageCode, GlossDatabase>,
}

impl GlossDatabaseManager {
    /// Create a new database manager
    pub fn new() -> Self {
        Self {
            databases: HashMap::new(),
        }
    }

    /// Get or create a database for a language
    pub fn get_or_create(&mut self, language: SignLanguageCode) -> &GlossDatabase {
        self.databases
            .entry(language)
            .or_insert_with(|| GlossDatabase::new(language))
    }

    /// Get a database for a language
    pub fn get(&self, language: SignLanguageCode) -> Option<&GlossDatabase> {
        self.databases.get(&language)
    }

    /// Preload databases for common languages
    pub fn preload_common(&mut self) {
        self.get_or_create(SignLanguageCode::Asl);
        self.get_or_create(SignLanguageCode::Bsl);
        self.get_or_create(SignLanguageCode::Ksl);
    }
}

impl Default for GlossDatabaseManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_asl_database() {
        let db = GlossDatabase::new(SignLanguageCode::Asl);
        assert!(db.entry_count() > 0);

        let hello = db.get("HELLO");
        assert!(hello.is_some());
        assert_eq!(hello.unwrap().sign_type, SignType::Lexical);
    }

    #[test]
    fn test_word_lookup() {
        let db = GlossDatabase::new(SignLanguageCode::Asl);

        let glosses = db.lookup_word("hello");
        assert!(!glosses.is_empty());
        assert_eq!(glosses[0].gloss, "HELLO");
    }

    #[test]
    fn test_ksl_database() {
        let db = GlossDatabase::new(SignLanguageCode::Ksl);
        assert!(db.entry_count() > 0);

        let hello = db.get("안녕");
        assert!(hello.is_some());
    }

    #[test]
    fn test_database_manager() {
        let mut manager = GlossDatabaseManager::new();
        manager.preload_common();

        assert!(manager.get(SignLanguageCode::Asl).is_some());
        assert!(manager.get(SignLanguageCode::Ksl).is_some());
    }
}
