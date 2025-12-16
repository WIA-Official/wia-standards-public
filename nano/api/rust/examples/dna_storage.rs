//! Example: DNA Data Storage
//!
//! This example demonstrates using molecular memory based on DNA
//! for ultra-dense data storage.

use wia_nano::prelude::*;
use wia_nano::systems::DnaMolecularMemory;

#[tokio::main]
async fn main() -> NanoResult<()> {
    println!("=== WIA Nano SDK: DNA Storage Example ===\n");

    // Create DNA memory with 1 KB capacity
    let mut memory = DnaMolecularMemory::builder("dna-mem-001")
        .with_capacity(1024) // 1 KB
        .with_encoding(DnaEncoding::goldman())
        .with_environment(Environment::aqueous(7.5).with_temperature(277.0))
        .build();

    // Initialize
    memory.initialize().await?;
    println!("✓ DNA memory system initialized");
    println!("  Capacity: {} bits ({} bytes)",
             memory.capacity_bits(),
             memory.capacity_bits() / 8);
    println!("  Encoding: Goldman ({:.2} bits/nucleotide)",
             memory.encoding().bits_per_nucleotide);
    println!("  Error correction: Reed-Solomon\n");

    // Data to store
    let message = b"Hello, WIA Nano! This is data stored in DNA.";
    println!("Original message: \"{}\"", String::from_utf8_lossy(message));
    println!("Size: {} bytes\n", message.len());

    // Encode to DNA sequence (for display)
    let dna_sequence = memory.encode(message)?;
    println!("Encoded DNA sequence (first 60 bp):");
    println!("  {}...", &dna_sequence[..60.min(dna_sequence.len())]);
    println!("  Total length: {} nucleotides\n", dna_sequence.len());

    // Write data
    let write_result = memory.write(0, message).await?;
    println!("✓ Data written:");
    println!("  Address: {}", write_result.address);
    println!("  Bytes: {}", write_result.bytes_written);
    println!("  Duration: {} ms", write_result.duration_ms);
    println!("  Verified: {}", write_result.verified);

    // Check memory status
    println!("\nMemory utilization: {:.1}%",
             (memory.used_bits() as f64 / memory.capacity_bits() as f64) * 100.0);

    // Read data back
    let read_data = memory.read(0, message.len()).await?;
    let decoded_message = String::from_utf8_lossy(&read_data);
    println!("\nRead back: \"{}\"", decoded_message);

    // Verify integrity
    let integrity = memory.verify_integrity().await?;
    println!("\n✓ Integrity check:");
    println!("  Valid: {}", integrity.valid);
    println!("  Blocks checked: {}", integrity.blocks_checked);
    println!("  Errors found: {}", integrity.errors_found);

    // Demonstrate DNA strand info
    let strand = memory.strand_info();
    println!("\nDNA Strand Properties:");
    println!("  Type: {:?}", strand.strand_type);
    println!("  Length: {} bp", strand.length_bp);
    println!("  Melting temp: {:.1}°C", strand.melting_temp_c);
    println!("  GC content: {:.1}%", memory.gc_content() * 100.0);

    // Calculate storage density
    let bits_stored = message.len() as f64 * 8.0;
    let nucleotides_used = (bits_stored / memory.encoding().bits_per_nucleotide) as u64;
    let density_bits_per_nm3 = bits_stored / (nucleotides_used as f64 * 0.34); // 0.34 nm per bp

    println!("\nStorage Metrics:");
    println!("  Density: {:.2} bits/nm³", density_bits_per_nm3);
    println!("  Equivalent: ~{:.0} PB/gram", bits_stored / 8.0 / 1e15 * 1e21);

    // Get status message
    let status = memory.to_message()?;
    println!("\nMemory Status:");
    println!("{}", serde_json::to_string_pretty(&status)?);

    // Shutdown
    memory.shutdown().await?;
    println!("\n✓ DNA storage session complete!");

    Ok(())
}
