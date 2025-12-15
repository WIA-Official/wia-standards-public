//! WIA Physics Integration Demo
//!
//! Demonstrates ecosystem integration with HDF5, InfluxDB, and EPICS.
//!
//! Run with: cargo run --example integration_demo

use wia_physics::prelude::*;
use wia_physics::integration::*;
use std::collections::HashMap;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== WIA Physics Integration Demo ===\n");

    // 1. Create physics data
    println!("1. Creating fusion data...");
    let fusion_data = FusionDataBuilder::new()
        .experiment("ITER")
        .plasma_simple(150e6, "K", 1e20, "m^-3")
        .tokamak(5.3, 6.2, 2.0)
        .build()?;

    let physics_data = PhysicsData {
        data_type: PhysicsDataType::Fusion,
        metadata: DataMetadata {
            id: "fusion-demo-001".to_string(),
            experiment: Some("ITER".to_string()),
            created: Some(chrono::Utc::now().to_rfc3339()),
            schema_version: Some("1.0".to_string()),
            ..Default::default()
        },
        payload: serde_json::to_value(&fusion_data)?,
    };

    println!("   Created: {}", physics_data.metadata.id);
    println!("   Type: {:?}", physics_data.data_type);
    println!();

    // 2. HDF5 Integration
    println!("2. HDF5 Integration Demo...");
    demo_hdf5(&physics_data)?;
    println!();

    // 3. InfluxDB Integration
    println!("3. InfluxDB Integration Demo...");
    demo_influxdb(&physics_data)?;
    println!();

    // 4. EPICS Integration
    println!("4. EPICS Integration Demo...");
    demo_epics(&physics_data)?;
    println!();

    // 5. Integration Manager Demo
    println!("5. Integration Manager Demo...");
    demo_manager(&physics_data)?;
    println!();

    println!("=== Demo Complete ===");
    Ok(())
}

fn demo_hdf5(data: &PhysicsData) -> Result<(), IntegrationError> {
    let mut hdf5 = HDF5Adapter::new();

    // Initialize and connect
    hdf5.initialize(IntegrationOptions {
        endpoint: Some("/tmp/wia_physics_demo.h5".to_string()),
        ..Default::default()
    })?;
    hdf5.connect()?;

    println!("   Connected to: {}", hdf5.file_path().unwrap_or("unknown"));
    println!("   Compression level: {}", hdf5.compression_level());

    // Create groups
    hdf5.create_group("/fusion")?;
    hdf5.create_group("/fusion/plasma")?;
    println!("   Created HDF5 groups");

    // Write attributes
    hdf5.write_attribute("/", "experiment", AttributeValue::String("ITER".to_string()))?;
    hdf5.write_attribute("/", "wia_version", AttributeValue::String("1.0".to_string()))?;
    println!("   Wrote metadata attributes");

    // Write dataset
    hdf5.write_dataset("/fusion/plasma/temperature", &[150e6], DatasetOptions {
        compression: 6,
        shuffle: true,
        ..Default::default()
    })?;
    println!("   Wrote temperature dataset");

    // Export physics data
    let result = hdf5.export(data)?;
    println!("   Export result: {} records to {}", result.records_exported, result.destination);

    // Cleanup
    hdf5.disconnect()?;
    println!("   Disconnected");

    Ok(())
}

fn demo_influxdb(data: &PhysicsData) -> Result<(), IntegrationError> {
    let mut influx = InfluxDBAdapter::new("http://localhost:8086", "wia_physics");

    // Initialize and connect
    influx.initialize(IntegrationOptions::default())?;
    influx.connect()?;

    println!("   Endpoint: {}", influx.endpoint());
    println!("   Database: {}", influx.database());

    // Create data points
    let point = DataPoint::new("wia_fusion_plasma")
        .tag("experiment", "ITER")
        .tag("location", "core")
        .field_float("temperature", 150e6)
        .field_float("density", 1e20)
        .field_float("confinement_time", 3.0);

    influx.write_point(point)?;
    println!("   Wrote plasma data point");

    // Write batch
    let points = vec![
        DataPoint::new("wia_fusion_energy")
            .tag("experiment", "ITER")
            .field_float("q_factor", 10.0)
            .field_float("heating_power", 50e6),
        DataPoint::new("wia_fusion_magnetics")
            .tag("experiment", "ITER")
            .field_float("toroidal_field", 5.3)
            .field_float("plasma_current", 15e6),
    ];
    influx.write_batch(points)?;
    println!("   Wrote batch of 2 points");

    println!("   Total points: {}", influx.points_count());

    // Query data
    let query = TimeSeriesQuery::new("wia_fusion_plasma")
        .select(vec!["temperature", "density"])
        .filter("experiment", "ITER")
        .limit(10);

    println!("   Query: {}", query.to_influxql());

    let result = influx.query(&query)?;
    println!("   Query returned {} series", result.series.len());

    // Export physics data
    let export_result = influx.export(data)?;
    println!("   Export result: {} records", export_result.records_exported);

    // Cleanup
    influx.disconnect()?;
    println!("   Disconnected");

    Ok(())
}

fn demo_epics(data: &PhysicsData) -> Result<(), IntegrationError> {
    let mut epics = EPICSAdapter::new()
        .with_endpoint("localhost:5064")
        .with_protocol(EPICSProtocol::ChannelAccess);

    // Initialize and connect
    epics.initialize(IntegrationOptions::default())?;
    epics.connect()?;

    println!("   Protocol: {:?}", epics.protocol());
    println!("   Endpoint: {}", epics.endpoint());

    // Put PV values
    epics.put_pv("ITER:PLASMA:TEMP:CORE", PVValue::double(150e6).with_units("K"))?;
    epics.put_pv("ITER:PLASMA:DENSITY:AVG", PVValue::double(1e20).with_units("m^-3"))?;
    epics.put_pv("ITER:MAGNETS:TF:FIELD", PVValue::double(5.3).with_units("T"))?;
    epics.put_pv("ITER:ENERGY:QFACTOR", PVValue::double(10.0))?;
    println!("   Set 4 PV values");

    // Get PV value
    let temp = epics.get_pv("ITER:PLASMA:TEMP:CORE")?;
    match temp.value {
        PVData::Double(v) => println!("   Temperature: {:.2e} {}", v, temp.units.unwrap_or_default()),
        _ => {}
    }

    // List PVs
    let plasma_pvs = epics.list_pvs("ITER:PLASMA:*")?;
    println!("   Plasma PVs: {:?}", plasma_pvs);

    // PV with alarm
    let high_temp = PVValue::double(300e6)
        .with_units("K")
        .with_alarm(AlarmSeverity::Major, AlarmStatus::HiHi)
        .with_limits(PVLimits {
            low_alarm: 0.0,
            low_warn: 50e6,
            high_warn: 200e6,
            high_alarm: 250e6,
        });
    epics.put_pv("ITER:PLASMA:TEMP:EDGE", high_temp)?;
    println!("   Set edge temperature with MAJOR alarm");

    // Export physics data
    let export_result = epics.export(data)?;
    println!("   Export result: {} PVs", export_result.records_exported);

    // Cleanup
    epics.disconnect()?;
    println!("   Disconnected");

    Ok(())
}

fn demo_manager(data: &PhysicsData) -> Result<(), IntegrationError> {
    let mut manager = IntegrationManager::new();

    // Register adapters
    manager.register(Box::new(HDF5Adapter::new()));
    manager.register(Box::new(InfluxDBAdapter::new("http://localhost:8086", "wia_physics")));
    manager.register(Box::new(EPICSAdapter::new()));
    println!("   Registered 3 adapters");

    // List adapters
    let adapters = manager.get_all();
    for adapter in adapters {
        println!("   - {} ({:?})", adapter.name(), adapter.integration_type());
    }

    // Initialize all
    let mut options = HashMap::new();
    options.insert(IntegrationType::Hdf5, IntegrationOptions {
        endpoint: Some("/tmp/manager_demo.h5".to_string()),
        ..Default::default()
    });
    manager.initialize_all(options)?;
    println!("   Initialized all adapters");

    // Connect all
    manager.connect_all();
    println!("   Connected all adapters");

    // Set preferences
    manager.set_preferences(IntegrationPreferences {
        enabled_integrations: vec![IntegrationType::Hdf5, IntegrationType::InfluxDb],
        primary_archive: IntegrationType::Hdf5,
        auto_export: false,
        export_interval_ms: 1000,
    });
    println!("   Set preferences: {:?}", manager.preferences().enabled_integrations);

    // Export to all enabled
    let results = manager.export_all(data);
    println!("   Export results:");
    for result in results {
        println!("     - {}: {} records", result.destination, result.records_exported);
    }

    // Export to specific adapter
    let epics_result = manager.export_to(IntegrationType::Epics, data)?;
    println!("   EPICS export: {} records", epics_result.records_exported);

    // Dispose all
    manager.dispose();
    println!("   Disposed all adapters");

    Ok(())
}
