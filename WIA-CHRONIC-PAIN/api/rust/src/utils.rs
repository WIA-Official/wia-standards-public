//! Utility functions for chronic pain management

use crate::types::*;

pub fn calculate_pain_trend(logs: &[PainLog]) -> String {
    if logs.len() < 2 {
        return "Insufficient data".to_string();
    }

    let avg_recent = logs.iter().rev().take(7).map(|l| l.pain_level).sum::<f64>() / 7.0;
    let avg_older = logs.iter().take(7).map(|l| l.pain_level).sum::<f64>() / 7.0;

    if avg_recent < avg_older {
        "Improving".to_string()
    } else if avg_recent > avg_older {
        "Worsening".to_string()
    } else {
        "Stable".to_string()
    }
}

pub fn recommend_interventions(profile: &PainProfile) -> Vec<String> {
    let mut recommendations = Vec::new();

    if profile.severity > 7.0 {
        recommendations.push("Consider pain specialist consultation".to_string());
        recommendations.push("Evaluate multimodal pain management".to_string());
    }

    match profile.pain_type {
        PainType::Neuropathic => {
            recommendations.push("Consider neuropathic pain medications".to_string());
        }
        PainType::Nociceptive => {
            recommendations.push("Consider physical therapy".to_string());
        }
        _ => {}
    }

    recommendations
}

pub fn identify_pain_triggers(logs: &[PainLog]) -> Vec<String> {
    let mut trigger_counts: std::collections::HashMap<String, usize> = std::collections::HashMap::new();

    for log in logs.iter().filter(|l| l.pain_level > 7.0) {
        for trigger in &log.triggers {
            *trigger_counts.entry(trigger.clone()).or_insert(0) += 1;
        }
    }

    trigger_counts.into_iter()
        .filter(|(_, count)| *count > 2)
        .map(|(trigger, _)| trigger)
        .collect()
}
