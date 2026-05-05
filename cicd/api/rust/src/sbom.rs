//! Minimum SBOM emitters (CycloneDX 1.5 + SPDX 2.3 tag-value).

/// SBOM serialisation formats.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SbomFormat { /// CycloneDX JSON 1.5
    CycloneDx,
    /// SPDX 2.3 tag-value
    Spdx,
    /// SPDX 2.3 JSON
    SpdxJson
}

/// SLSA Provenance levels.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SlsaLevel { /// L0
    L0,
    /// L1
    L1,
    /// L2
    L2,
    /// L3
    L3
}

/// Component metadata.
pub struct SbomComponent {
    /// CycloneDX `type` (`library`, `application`, `container`, `framework`, `os`).
    pub kind:     &'static str,
    /// Component name.
    pub name:     String,
    /// Component version.
    pub version:  String,
    /// Optional Package URL.
    pub purl:     Option<String>,
    /// Optional license expressions / SPDX IDs.
    pub licenses: Vec<String>,
}

/// SBOM document metadata.
pub struct SbomMeta {
    /// Document URN.
    pub serial_number: String,
    /// Build timestamp (RFC 3339).
    pub timestamp:     String,
    /// Tool vendor.
    pub tool_vendor:   String,
    /// Tool name.
    pub tool_name:     String,
    /// Tool version.
    pub tool_version:  String,
}

/// Emit a minimum CycloneDX 1.5 JSON document as a `String`.
pub fn emit_cyclonedx(meta: &SbomMeta, components: &[SbomComponent]) -> String {
    let comps: Vec<String> = components.iter().map(|c| {
        let mut parts = vec![
            format!("\"type\":\"{}\"", c.kind),
            format!("\"name\":\"{}\"", c.name),
            format!("\"version\":\"{}\"", c.version),
        ];
        if let Some(p) = &c.purl { parts.push(format!("\"purl\":\"{}\"", p)); }
        if !c.licenses.is_empty() {
            let lic: Vec<String> = c.licenses.iter().map(|id| format!("{{\"license\":{{\"id\":\"{}\"}}}}", id)).collect();
            parts.push(format!("\"licenses\":[{}]", lic.join(",")));
        }
        format!("{{{}}}", parts.join(","))
    }).collect();
    format!(
        "{{\"bomFormat\":\"CycloneDX\",\"specVersion\":\"1.5\",\"serialNumber\":\"{}\",\"version\":1,\"metadata\":{{\"timestamp\":\"{}\",\"tools\":[{{\"vendor\":\"{}\",\"name\":\"{}\",\"version\":\"{}\"}}]}},\"components\":[{}]}}",
        meta.serial_number, meta.timestamp, meta.tool_vendor, meta.tool_name, meta.tool_version, comps.join(",")
    )
}

/// Emit a minimum SPDX 2.3 tag-value document.
pub fn emit_spdx_tag_value(meta: &SbomMeta, components: &[SbomComponent]) -> String {
    let mut out = vec![
        "SPDXVersion: SPDX-2.3".to_string(),
        "DataLicense: CC0-1.0".to_string(),
        "SPDXID: SPDXRef-DOCUMENT".to_string(),
        format!("DocumentName: {}-sbom", meta.tool_name),
        format!("Created: {}", meta.timestamp),
        format!("Creator: Tool: {} {} {}", meta.tool_vendor, meta.tool_name, meta.tool_version),
    ];
    for (i, c) in components.iter().enumerate() {
        out.push(String::new());
        out.push(format!("PackageName: {}", c.name));
        out.push(format!("SPDXID: SPDXRef-Package-{}", i));
        out.push(format!("PackageVersion: {}", c.version));
        out.push("PackageDownloadLocation: NOASSERTION".to_string());
        out.push(format!("PackageLicenseConcluded: {}", c.licenses.first().cloned().unwrap_or_else(|| "NOASSERTION".to_string())));
    }
    out.join("\n")
}

/// Determine if `(level, signed)` satisfies WIA-CICD's L3 signing target.
pub fn meets_slsa_target(level: &SlsaLevel, signed: bool) -> bool {
    matches!(level, SlsaLevel::L3) && signed
}
