//! Filter types for query operations
//!
//! Implements the WIA Material Protocol filter syntax for querying materials.

use serde::{Deserialize, Serialize};

/// Filter operator
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum FilterOperator {
    /// Equals
    Eq,
    /// Not equals
    Ne,
    /// Greater than
    Gt,
    /// Greater than or equal
    Gte,
    /// Less than
    Lt,
    /// Less than or equal
    Lte,
    /// Contains string
    Contains,
    /// In list
    In,
    /// Field exists
    Exists,
}

/// Single filter condition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FilterCondition {
    /// Field path (dot notation for nested fields)
    pub field: String,

    /// Filter operator
    pub operator: FilterOperator,

    /// Value to compare against
    pub value: serde_json::Value,
}

impl FilterCondition {
    /// Create a new filter condition
    pub fn new(field: &str, operator: FilterOperator, value: impl Into<serde_json::Value>) -> Self {
        Self {
            field: field.to_string(),
            operator,
            value: value.into(),
        }
    }

    /// Equals filter
    pub fn eq(field: &str, value: impl Into<serde_json::Value>) -> Self {
        Self::new(field, FilterOperator::Eq, value)
    }

    /// Not equals filter
    pub fn ne(field: &str, value: impl Into<serde_json::Value>) -> Self {
        Self::new(field, FilterOperator::Ne, value)
    }

    /// Greater than filter
    pub fn gt(field: &str, value: impl Into<serde_json::Value>) -> Self {
        Self::new(field, FilterOperator::Gt, value)
    }

    /// Greater than or equal filter
    pub fn gte(field: &str, value: impl Into<serde_json::Value>) -> Self {
        Self::new(field, FilterOperator::Gte, value)
    }

    /// Less than filter
    pub fn lt(field: &str, value: impl Into<serde_json::Value>) -> Self {
        Self::new(field, FilterOperator::Lt, value)
    }

    /// Less than or equal filter
    pub fn lte(field: &str, value: impl Into<serde_json::Value>) -> Self {
        Self::new(field, FilterOperator::Lte, value)
    }

    /// Contains string filter
    pub fn contains(field: &str, value: &str) -> Self {
        Self::new(field, FilterOperator::Contains, value)
    }

    /// In list filter
    pub fn in_list(field: &str, values: Vec<serde_json::Value>) -> Self {
        Self::new(field, FilterOperator::In, serde_json::Value::Array(values))
    }

    /// Exists filter
    pub fn exists(field: &str) -> Self {
        Self::new(field, FilterOperator::Exists, true)
    }

    /// Not exists filter
    pub fn not_exists(field: &str) -> Self {
        Self::new(field, FilterOperator::Exists, false)
    }
}

/// Compound filter with AND/OR logic
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Filter {
    /// Single condition
    Condition(FilterCondition),

    /// AND combination
    And {
        #[serde(rename = "and")]
        conditions: Vec<Filter>,
    },

    /// OR combination
    Or {
        #[serde(rename = "or")]
        conditions: Vec<Filter>,
    },

    /// NOT negation
    Not {
        #[serde(rename = "not")]
        condition: Box<Filter>,
    },
}

impl Filter {
    /// Create a single condition filter
    pub fn condition(condition: FilterCondition) -> Self {
        Filter::Condition(condition)
    }

    /// Create an AND filter
    pub fn and(conditions: Vec<Filter>) -> Self {
        Filter::And { conditions }
    }

    /// Create an OR filter
    pub fn or(conditions: Vec<Filter>) -> Self {
        Filter::Or { conditions }
    }

    /// Create a NOT filter
    pub fn not(condition: Filter) -> Self {
        Filter::Not {
            condition: Box::new(condition),
        }
    }

    /// Combine this filter with another using AND
    pub fn and_with(self, other: Filter) -> Self {
        match self {
            Filter::And { mut conditions } => {
                conditions.push(other);
                Filter::And { conditions }
            }
            _ => Filter::And {
                conditions: vec![self, other],
            },
        }
    }

    /// Combine this filter with another using OR
    pub fn or_with(self, other: Filter) -> Self {
        match self {
            Filter::Or { mut conditions } => {
                conditions.push(other);
                Filter::Or { conditions }
            }
            _ => Filter::Or {
                conditions: vec![self, other],
            },
        }
    }
}

/// Builder for creating complex filters
#[derive(Debug, Default)]
pub struct FilterBuilder {
    conditions: Vec<Filter>,
}

impl FilterBuilder {
    /// Create a new filter builder
    pub fn new() -> Self {
        Self::default()
    }

    /// Add an equals condition
    pub fn eq(mut self, field: &str, value: impl Into<serde_json::Value>) -> Self {
        self.conditions
            .push(Filter::Condition(FilterCondition::eq(field, value)));
        self
    }

    /// Add a greater than condition
    pub fn gt(mut self, field: &str, value: impl Into<serde_json::Value>) -> Self {
        self.conditions
            .push(Filter::Condition(FilterCondition::gt(field, value)));
        self
    }

    /// Add a greater than or equal condition
    pub fn gte(mut self, field: &str, value: impl Into<serde_json::Value>) -> Self {
        self.conditions
            .push(Filter::Condition(FilterCondition::gte(field, value)));
        self
    }

    /// Add a less than condition
    pub fn lt(mut self, field: &str, value: impl Into<serde_json::Value>) -> Self {
        self.conditions
            .push(Filter::Condition(FilterCondition::lt(field, value)));
        self
    }

    /// Add a less than or equal condition
    pub fn lte(mut self, field: &str, value: impl Into<serde_json::Value>) -> Self {
        self.conditions
            .push(Filter::Condition(FilterCondition::lte(field, value)));
        self
    }

    /// Add a contains condition
    pub fn contains(mut self, field: &str, value: &str) -> Self {
        self.conditions
            .push(Filter::Condition(FilterCondition::contains(field, value)));
        self
    }

    /// Add an exists condition
    pub fn exists(mut self, field: &str) -> Self {
        self.conditions
            .push(Filter::Condition(FilterCondition::exists(field)));
        self
    }

    /// Build the filter with AND logic
    pub fn build_and(self) -> Option<Filter> {
        match self.conditions.len() {
            0 => None,
            1 => Some(self.conditions.into_iter().next().unwrap()),
            _ => Some(Filter::And {
                conditions: self.conditions,
            }),
        }
    }

    /// Build the filter with OR logic
    pub fn build_or(self) -> Option<Filter> {
        match self.conditions.len() {
            0 => None,
            1 => Some(self.conditions.into_iter().next().unwrap()),
            _ => Some(Filter::Or {
                conditions: self.conditions,
            }),
        }
    }
}

// ============================================================================
// OPTIMADE Compatibility
// ============================================================================

/// OPTIMADE-style filter parser (simplified)
pub struct OptimadeFilter;

impl OptimadeFilter {
    /// Parse a simple OPTIMADE filter string
    ///
    /// Supports basic syntax like:
    /// - `elements HAS "Fe"`
    /// - `nelements = 2`
    /// - `band_gap > 0.5`
    pub fn parse(filter_str: &str) -> Option<Filter> {
        let parts: Vec<&str> = filter_str.split_whitespace().collect();

        if parts.len() < 3 {
            return None;
        }

        let field = parts[0];
        let op = parts[1].to_uppercase();
        let value = parts[2..].join(" ").trim_matches('"').to_string();

        let operator = match op.as_str() {
            "=" | "==" => FilterOperator::Eq,
            "!=" | "<>" => FilterOperator::Ne,
            ">" => FilterOperator::Gt,
            ">=" => FilterOperator::Gte,
            "<" => FilterOperator::Lt,
            "<=" => FilterOperator::Lte,
            "HAS" | "CONTAINS" => FilterOperator::Contains,
            _ => return None,
        };

        let json_value = if let Ok(num) = value.parse::<f64>() {
            serde_json::Value::Number(
                serde_json::Number::from_f64(num).unwrap_or_else(|| serde_json::Number::from(0)),
            )
        } else if let Ok(num) = value.parse::<i64>() {
            serde_json::Value::Number(serde_json::Number::from(num))
        } else if value == "true" {
            serde_json::Value::Bool(true)
        } else if value == "false" {
            serde_json::Value::Bool(false)
        } else {
            serde_json::Value::String(value)
        };

        Some(Filter::Condition(FilterCondition {
            field: field.to_string(),
            operator,
            value: json_value,
        }))
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_filter_condition() {
        let condition = FilterCondition::gt("temperature", 77.0);
        assert_eq!(condition.field, "temperature");
        assert_eq!(condition.operator, FilterOperator::Gt);
    }

    #[test]
    fn test_filter_builder() {
        let filter = FilterBuilder::new()
            .eq("material_type", "superconductor")
            .gt("properties.superconductor.critical_temperature_k", 77.0)
            .build_and()
            .unwrap();

        let json = serde_json::to_string(&filter).unwrap();
        assert!(json.contains("and"));
        assert!(json.contains("superconductor"));
    }

    #[test]
    fn test_filter_combination() {
        let f1 = Filter::Condition(FilterCondition::eq("type", "superconductor"));
        let f2 = Filter::Condition(FilterCondition::gt("tc", 77.0));

        let combined = f1.and_with(f2);
        let json = serde_json::to_string(&combined).unwrap();
        assert!(json.contains("and"));
    }

    #[test]
    fn test_optimade_parse() {
        let filter = OptimadeFilter::parse("nelements = 2").unwrap();
        let json = serde_json::to_string(&filter).unwrap();
        assert!(json.contains("nelements"));
        assert!(json.contains("eq"));
    }

    #[test]
    fn test_optimade_parse_gt() {
        let filter = OptimadeFilter::parse("band_gap > 0.5").unwrap();
        if let Filter::Condition(c) = filter {
            assert_eq!(c.field, "band_gap");
            assert_eq!(c.operator, FilterOperator::Gt);
        } else {
            panic!("Expected condition");
        }
    }
}
