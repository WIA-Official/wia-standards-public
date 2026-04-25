//! Integration tests for WIA-DATA_QUALITY SDK

use wia_data_quality::*;

#[test]
fn test_issue_severity() {
    let severity = IssueSeverity::Critical;
    assert_eq!(severity, IssueSeverity::Critical);
}

#[test]
fn test_calculate_completeness() {
    use wia_data_quality::utils::calculate_completeness;
    let score = calculate_completeness(100, 10);
    assert_eq!(score, 90.0);
}

#[test]
fn test_calculate_accuracy() {
    use wia_data_quality::utils::calculate_accuracy;
    let score = calculate_accuracy(100, 85);
    assert_eq!(score, 85.0);
}

#[test]
fn test_categorize_score() {
    use wia_data_quality::utils::categorize_score;
    assert_eq!(categorize_score(95.0), "Excellent");
    assert_eq!(categorize_score(80.0), "Good");
    assert_eq!(categorize_score(60.0), "Fair");
}
