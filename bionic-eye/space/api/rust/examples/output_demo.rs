//! WIA Space Standard Phase 4: Output Layer Demo
//!
//! This example demonstrates the output layer functionality including:
//! - CCSDS OEM/OPM export
//! - JSON/CSV export
//! - GMAT script generation
//! - CZML visualization export
//! - Output Manager for coordinating multiple adapters

use wia_space::output::{
    CcsdsOemExporter, CcsdsOpmExporter, CsvExporter, CzmlExporter, GmatScriptExporter,
    JsonExporter, MockOutputAdapter, OutputData, OutputManager, OutputType,
};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🚀 WIA Space Standard - Phase 4 Output Layer Demo\n");
    println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    // Create sample orbital data
    let orbital_data = create_sample_data();

    // Demo 1: CCSDS OEM Export
    demo_ccsds_oem(&orbital_data).await?;

    // Demo 2: CCSDS OPM Export
    demo_ccsds_opm(&orbital_data).await?;

    // Demo 3: JSON Export
    demo_json_export(&orbital_data).await?;

    // Demo 4: CSV Export
    demo_csv_export(&orbital_data).await?;

    // Demo 5: GMAT Script Export
    demo_gmat_export(&orbital_data).await?;

    // Demo 6: CZML Visualization Export
    demo_czml_export(&orbital_data).await?;

    // Demo 7: Output Manager
    demo_output_manager().await?;

    println!("\n✨ Phase 4 Output Layer demo complete!");
    println!("   弘益人間 - Benefit All Humanity");

    Ok(())
}

fn create_sample_data() -> OutputData {
    OutputData::from_json(serde_json::json!({
        "ephemeris": [
            {
                "time": "2025-01-01T00:00:00Z",
                "x": 6778.137,
                "y": 0.0,
                "z": 0.0,
                "vx": 0.0,
                "vy": 7.669,
                "vz": 0.0
            },
            {
                "time": "2025-01-01T00:01:00Z",
                "x": 6773.821,
                "y": 460.095,
                "z": 105.234,
                "vx": -0.052,
                "vy": 7.667,
                "vz": 0.012
            },
            {
                "time": "2025-01-01T00:02:00Z",
                "x": 6760.891,
                "y": 919.856,
                "z": 210.402,
                "vx": -0.104,
                "vy": 7.661,
                "vz": 0.024
            }
        ],
        "keplerian": {
            "semi_major_axis": 6778.137,
            "eccentricity": 0.0001,
            "inclination": 51.6,
            "raan": 0.0,
            "arg_of_pericenter": 0.0,
            "true_anomaly": 0.0
        },
        "state_vector": {
            "epoch": "2025-01-01T00:00:00Z",
            "x": 6778.137,
            "y": 0.0,
            "z": 0.0,
            "vx": 0.0,
            "vy": 7.669,
            "vz": 0.0
        }
    }))
    .with_metadata("object_name", "ISS (ZARYA)")
    .with_metadata("object_id", "1998-067A")
    .with_metadata("start_time", "2025-01-01T00:00:00Z")
    .with_metadata("stop_time", "2025-01-01T00:02:00Z")
}

async fn demo_ccsds_oem(data: &OutputData) -> Result<(), Box<dyn std::error::Error>> {
    println!("📄 Demo 1: CCSDS OEM Export");
    println!("   ─────────────────────────");

    let exporter = CcsdsOemExporter::new("oem-exporter")
        .with_originator("WIA Space Standard")
        .with_object_name("ISS (ZARYA)")
        .with_object_id("1998-067A");

    let oem_content = exporter.export_oem(data)?;
    println!("   Generated OEM content ({} bytes):\n", oem_content.len());

    // Print first few lines
    for (i, line) in oem_content.lines().take(15).enumerate() {
        println!("   {:2} | {}", i + 1, line);
    }
    println!("   ...\n");

    Ok(())
}

async fn demo_ccsds_opm(data: &OutputData) -> Result<(), Box<dyn std::error::Error>> {
    println!("📄 Demo 2: CCSDS OPM Export");
    println!("   ─────────────────────────");

    let exporter = CcsdsOpmExporter::new("opm-exporter").with_originator("WIA Space Standard");

    let opm_content = exporter.export_opm(data)?;
    println!("   Generated OPM content ({} bytes):\n", opm_content.len());

    for (i, line) in opm_content.lines().take(20).enumerate() {
        println!("   {:2} | {}", i + 1, line);
    }
    println!("   ...\n");

    Ok(())
}

async fn demo_json_export(data: &OutputData) -> Result<(), Box<dyn std::error::Error>> {
    println!("📄 Demo 3: JSON Export");
    println!("   ────────────────────");

    let exporter = JsonExporter::new("json-exporter").with_pretty(true);

    let json_content = exporter.export_json(data)?;
    println!("   Generated JSON content ({} bytes):\n", json_content.len());

    for (i, line) in json_content.lines().take(15).enumerate() {
        println!("   {:2} | {}", i + 1, line);
    }
    println!("   ...\n");

    Ok(())
}

async fn demo_csv_export(data: &OutputData) -> Result<(), Box<dyn std::error::Error>> {
    println!("📄 Demo 4: CSV Export");
    println!("   ───────────────────");

    let exporter = CsvExporter::new("csv-exporter");

    let csv_content = exporter.export_csv(data)?;
    println!("   Generated CSV content ({} bytes):\n", csv_content.len());

    for (i, line) in csv_content.lines().enumerate() {
        println!("   {:2} | {}", i + 1, line);
    }
    println!();

    Ok(())
}

async fn demo_gmat_export(data: &OutputData) -> Result<(), Box<dyn std::error::Error>> {
    println!("📄 Demo 5: GMAT Script Export");
    println!("   ───────────────────────────");

    let exporter = GmatScriptExporter::new("gmat-exporter");

    let gmat_content = exporter.export_gmat(data)?;
    println!(
        "   Generated GMAT script ({} bytes):\n",
        gmat_content.len()
    );

    for (i, line) in gmat_content.lines().take(20).enumerate() {
        println!("   {:2} | {}", i + 1, line);
    }
    println!("   ...\n");

    Ok(())
}

async fn demo_czml_export(data: &OutputData) -> Result<(), Box<dyn std::error::Error>> {
    println!("📄 Demo 6: CZML Visualization Export");
    println!("   ──────────────────────────────────");

    let exporter = CzmlExporter::new("czml-exporter").with_document_name("ISS Orbit Visualization");

    let czml_content = exporter.export_czml(data)?;
    println!("   Generated CZML content ({} bytes):\n", czml_content.len());

    for (i, line) in czml_content.lines().take(20).enumerate() {
        println!("   {:2} | {}", i + 1, line);
    }
    println!("   ...\n");

    Ok(())
}

async fn demo_output_manager() -> Result<(), Box<dyn std::error::Error>> {
    println!("📄 Demo 7: Output Manager");
    println!("   ───────────────────────");

    // Create manager
    let manager = OutputManager::new();

    // Register multiple adapters
    let vis_adapter = MockOutputAdapter::visualization("cesium-vis");
    let export_adapter = MockOutputAdapter::exporter("json-export");
    let export_adapter2 = MockOutputAdapter::exporter("csv-export");

    manager.register("cesium", Box::new(vis_adapter)).await;
    manager.register("json", Box::new(export_adapter)).await;
    manager.register("csv", Box::new(export_adapter2)).await;

    println!("   Registered {} adapters:", manager.adapter_count().await);

    let adapter_names = manager.get_adapter_names().await;
    for name in &adapter_names {
        let available = manager.is_adapter_available(name).await;
        println!("   - {} (available: {})", name, available);
    }

    // Get adapters by type
    let exporters = manager.get_available_by_type(OutputType::Export).await;
    println!("\n   Export adapters: {:?}", exporters);

    let visualizers = manager
        .get_available_by_type(OutputType::Visualization)
        .await;
    println!("   Visualization adapters: {:?}", visualizers);

    // Output to specific adapter
    let data = OutputData::new();
    let result = manager.output_to("json", &data).await;
    println!("\n   Output to 'json': {:?}", result.is_ok());

    // Broadcast to all exporters
    let results = manager.broadcast(OutputType::Export, &data).await;
    println!(
        "   Broadcast to exporters: {} results",
        results.len()
    );

    // Output to all available
    let results = manager.output_all(&data).await;
    println!(
        "   Output to all: {} results\n",
        results.len()
    );

    Ok(())
}
