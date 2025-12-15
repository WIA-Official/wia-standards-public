//! Post-Quantum Cryptography Example
//!
//! Demonstrates PQC key generation using NIST standards.

use wia_quantum::crypto::{
    PqcAlgorithm, PqcKeyPair, PqcSignature,
    QkdSession, QkdProtocol, QkdParticipant, QkdChannel, ChannelType,
};
use wia_quantum::Result;

fn main() -> Result<()> {
    println!("=== WIA Quantum: Post-Quantum Cryptography Example ===\n");

    // =========================================
    // Part 1: ML-KEM Key Generation
    // =========================================
    println!("--- ML-KEM (Kyber) Key Encapsulation ---\n");

    // Generate ML-KEM-768 key pair (NIST Level 3)
    let kem_keypair = PqcKeyPair::generate(PqcAlgorithm::MlKem768)?;

    println!("Algorithm: {} ({})",
        kem_keypair.algorithm.name,
        kem_keypair.algorithm.variant
    );
    println!("NIST Level: {}", kem_keypair.algorithm.nist_level);
    println!("Key Type: {:?}", kem_keypair.algorithm.key_type);
    println!("Key ID: {}", kem_keypair.metadata.key_id);
    println!("Public Key Size: {} bytes", kem_keypair.key.key_size_bytes.public);
    println!("Generated: {}", kem_keypair.metadata.generated_at);

    // Get public key only (for sharing)
    let public_key = kem_keypair.public_key();
    println!("\nPublic key extracted for sharing.");

    // =========================================
    // Part 2: ML-DSA Signature
    // =========================================
    println!("\n--- ML-DSA (Dilithium) Digital Signature ---\n");

    // Generate ML-DSA-65 key pair (NIST Level 3)
    let sig_keypair = PqcKeyPair::generate(PqcAlgorithm::MlDsa65)?;

    println!("Algorithm: {} ({})",
        sig_keypair.algorithm.name,
        sig_keypair.algorithm.variant
    );
    println!("NIST Level: {}", sig_keypair.algorithm.nist_level);
    println!("Key ID: {}", sig_keypair.metadata.key_id);

    // Create a signature (mock)
    let message = b"Hello, Quantum World!";
    let signature = PqcSignature::sign_mock(
        PqcAlgorithm::MlDsa65,
        message,
        sig_keypair.metadata.key_id,
    )?;

    println!("\nMessage: \"{}\"", String::from_utf8_lossy(message));
    println!("Signature Size: {} bytes", signature.signature.size_bytes);
    println!("Hash Algorithm: {}", signature.message_hash.algorithm);

    // =========================================
    // Part 3: Available Algorithms
    // =========================================
    println!("\n--- NIST PQC Algorithms Summary ---\n");

    let algorithms = [
        PqcAlgorithm::MlKem512,
        PqcAlgorithm::MlKem768,
        PqcAlgorithm::MlKem1024,
        PqcAlgorithm::MlDsa44,
        PqcAlgorithm::MlDsa65,
        PqcAlgorithm::MlDsa87,
        PqcAlgorithm::SlhDsaShake128f,
        PqcAlgorithm::Hqc128,
    ];

    println!("{:<20} {:>6} {:>8} {:>10}", "Algorithm", "Level", "Type", "PK Size");
    println!("{:-<50}", "");

    for alg in algorithms {
        let info = wia_quantum::crypto::AlgorithmInfo::from(alg);
        println!(
            "{:<20} {:>6} {:>8} {:>10}",
            info.variant,
            info.nist_level,
            format!("{:?}", info.key_type),
            format!("{} B", alg.public_key_size())
        );
    }

    // =========================================
    // Part 4: QKD Session
    // =========================================
    println!("\n--- QKD Session Example ---\n");

    let session = QkdSession::new(
        QkdProtocol::BB84,
        QkdParticipant {
            node_id: "node-seoul".to_string(),
            address: "192.168.1.10".to_string(),
        },
        QkdParticipant {
            node_id: "node-busan".to_string(),
            address: "192.168.1.20".to_string(),
        },
        QkdChannel {
            channel_type: ChannelType::Fiber,
            length_km: 325.0,
            loss_db: 65.0,
        },
    );

    println!("Protocol: {:?}", session.protocol);
    println!("Session ID: {}", session.session_id);
    println!("Alice: {} ({})", session.participants.alice.node_id, session.participants.alice.address);
    println!("Bob: {} ({})", session.participants.bob.node_id, session.participants.bob.address);
    println!("Channel: {:?}, {} km", session.channel.channel_type, session.channel.length_km);
    println!("Status: {:?}", session.status);

    // Export to JSON
    println!("\n--- Export to WIA Quantum Format ---\n");
    println!("ML-KEM Key (truncated):");
    let json = kem_keypair.to_json()?;
    let lines: Vec<&str> = json.lines().take(15).collect();
    for line in lines {
        println!("{}", line);
    }
    println!("  ...");
    println!("}}");

    Ok(())
}
