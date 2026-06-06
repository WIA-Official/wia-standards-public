//! Simulate a stream of clarity changes for one sensor.
//!
//! Drives a [`SimulatedSensor`] through accumulating contamination, emitting a
//! `sensor.state_change` event each time it crosses a band threshold (the same
//! event the WebSocket path would push, Phase 2 §9.2), then runs a cleaning
//! cycle and watches it recover to CLEAR.

use wia_perception_clarity::prelude::*;

fn main() -> Result<()> {
    println!("=== WIA Perception Clarity SDK — Report Stream Simulation ===\n");

    let mut sim = SimulatedSensor::new("nav-lidar-front", SensorClass::LidarWindow)
        .soiling_rate(0.06)
        .contaminant(ContaminantType::MudDust)
        .tick_seconds(5.0);

    let mut prev_state = ClarityState::Clear;

    for tick in 1..=25 {
        let snap = sim.tick()?;
        let state = snap.state;

        if state != prev_state {
            // Emit the threshold-crossing event the stream would push.
            let event = ClarityEvent::StateChange {
                sensor_id: snap.sensor_id.clone(),
                from: prev_state,
                to: state,
                pci: snap.pci,
                dwell_seconds: snap.dwell_seconds,
            };
            let worsening = if event.is_worsening() { "worse" } else { "better" };
            println!(
                "tick {:>2}  pci={:>3}  {:?} -> {:?}  ({})  [{}]",
                tick,
                snap.pci.value(),
                prev_state,
                state,
                worsening,
                event.to_json().unwrap_or_default(),
            );
            prev_state = state;
        } else {
            println!("tick {:>2}  pci={:>3}  {:?}", tick, snap.pci.value(), state);
        }

        if state == ClarityState::Blind {
            println!("\n-> BLIND reached. Required safe actions: {:?}", required_safe_actions(state));
            break;
        }
    }

    // Run a cleaning cycle and confirm recovery.
    println!("\nRunning cleaning cycle...");
    sim.clean();
    let recovered = sim.tick()?;
    println!(
        "after cleaning: pci={} state={:?}",
        recovered.pci.value(),
        recovered.state
    );

    Ok(())
}
