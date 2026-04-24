//! Molecular memory trait definitions

use crate::error::NanoResult;
use async_trait::async_trait;

/// Trait for molecular memory systems
#[async_trait]
pub trait MolecularMemory: Send + Sync {
    /// Get memory ID
    fn memory_id(&self) -> &str;

    /// Get storage type
    fn storage_type(&self) -> StorageType;

    /// Get total capacity in bits
    fn capacity_bits(&self) -> u64;

    /// Get used capacity in bits
    fn used_bits(&self) -> u64;

    /// Get available capacity
    fn available_bits(&self) -> u64 {
        self.capacity_bits() - self.used_bits()
    }

    /// Write data
    async fn write(&mut self, address: u64, data: &[u8]) -> NanoResult<WriteResult>;

    /// Read data
    async fn read(&self, address: u64, length: usize) -> NanoResult<Vec<u8>>;

    /// Erase data
    async fn erase(&mut self, address: u64, length: usize) -> NanoResult<()>;

    /// Get memory status
    fn status(&self) -> MemoryStatus;

    /// Get error correction info
    fn error_correction(&self) -> &ErrorCorrection;

    /// Run integrity check
    async fn verify_integrity(&mut self) -> NanoResult<IntegrityReport>;
}

/// Storage medium type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StorageType {
    /// DNA-based storage
    Dna,
    /// Protein-based storage
    Protein,
    /// Synthetic polymer
    Polymer,
    /// Molecular switch array
    SwitchArray,
    /// Quantum dots
    QuantumDot,
    /// Custom storage
    Custom,
}

/// Memory status information
#[derive(Debug, Clone)]
pub struct MemoryStatus {
    pub state: MemoryState,
    pub temperature_k: Option<f64>,
    pub degradation_level: f64,  // 0.0 = pristine, 1.0 = fully degraded
    pub error_rate: f64,
    pub last_access: Option<String>,
}

/// Memory operational state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MemoryState {
    Ready,
    Reading,
    Writing,
    Verifying,
    Error,
    Offline,
}

/// Write operation result
#[derive(Debug, Clone)]
pub struct WriteResult {
    pub bytes_written: usize,
    pub address: u64,
    pub duration_ms: u64,
    pub verified: bool,
    pub errors_corrected: u32,
}

/// Error correction configuration
#[derive(Debug, Clone)]
pub struct ErrorCorrection {
    pub scheme: ErrorCorrectionScheme,
    pub redundancy_factor: f64,
    pub max_correctable_errors: u32,
}

/// Error correction schemes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorCorrectionScheme {
    /// No error correction
    None,
    /// Reed-Solomon codes
    ReedSolomon,
    /// LDPC codes
    Ldpc,
    /// Hamming codes
    Hamming,
    /// Repetition codes
    Repetition,
    /// Custom scheme
    Custom,
}

/// Integrity check report
#[derive(Debug, Clone)]
pub struct IntegrityReport {
    pub valid: bool,
    pub blocks_checked: u64,
    pub errors_found: u32,
    pub errors_corrected: u32,
    pub uncorrectable_errors: u32,
    pub checksum: Option<String>,
}

/// DNA-specific memory trait
#[async_trait]
pub trait DnaMemory: MolecularMemory {
    /// Get encoding scheme
    fn encoding(&self) -> DnaEncoding;

    /// Get sequence length
    fn sequence_length(&self) -> u64;

    /// Get GC content (for stability)
    fn gc_content(&self) -> f64;

    /// Encode data to DNA sequence
    fn encode(&self, data: &[u8]) -> NanoResult<String>;

    /// Decode DNA sequence to data
    fn decode(&self, sequence: &str) -> NanoResult<Vec<u8>>;

    /// Get physical strand information
    fn strand_info(&self) -> &DnaStrandInfo;

    /// Synthesize new strand
    async fn synthesize(&mut self, sequence: &str) -> NanoResult<SynthesisResult>;

    /// Sequence (read) strand
    async fn sequence_strand(&mut self) -> NanoResult<SequencingResult>;
}

/// DNA encoding scheme
#[derive(Debug, Clone)]
pub struct DnaEncoding {
    pub scheme_name: String,
    pub bits_per_nucleotide: f64,
    pub homopolymer_limit: u8,
    pub gc_target_range: (f64, f64),
}

impl DnaEncoding {
    /// Goldman encoding (1.77 bits/nucleotide)
    pub fn goldman() -> Self {
        Self {
            scheme_name: "Goldman".to_string(),
            bits_per_nucleotide: 1.77,
            homopolymer_limit: 1,
            gc_target_range: (0.45, 0.55),
        }
    }

    /// Church encoding (1 bit/nucleotide)
    pub fn church() -> Self {
        Self {
            scheme_name: "Church".to_string(),
            bits_per_nucleotide: 1.0,
            homopolymer_limit: 3,
            gc_target_range: (0.4, 0.6),
        }
    }

    /// Grass encoding (1.98 bits/nucleotide)
    pub fn grass() -> Self {
        Self {
            scheme_name: "Grass".to_string(),
            bits_per_nucleotide: 1.98,
            homopolymer_limit: 3,
            gc_target_range: (0.45, 0.55),
        }
    }
}

/// DNA strand physical information
#[derive(Debug, Clone)]
pub struct DnaStrandInfo {
    pub strand_type: StrandType,
    pub length_bp: u64,
    pub melting_temp_c: f64,
    pub concentration_nm: f64,
}

/// Type of DNA strand
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StrandType {
    SingleStranded,
    DoubleStranded,
    Circular,
    LinearCapped,
}

/// DNA synthesis result
#[derive(Debug, Clone)]
pub struct SynthesisResult {
    pub success: bool,
    pub nucleotides_synthesized: u64,
    pub error_rate: f64,
    pub duration_ms: u64,
}

/// DNA sequencing result
#[derive(Debug, Clone)]
pub struct SequencingResult {
    pub sequence: String,
    pub quality_scores: Vec<u8>,
    pub coverage: f64,
    pub duration_ms: u64,
}
