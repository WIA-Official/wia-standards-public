# WIA-SOIL-MICROBIOME Phase 1: Data Format Specification

**Version:** 1.0.0
**Date:** 2025-12-29
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## Overview

Phase 1 defines standardized data formats for soil microbiome analysis, including sample metadata, microbial community composition, functional gene data, and environmental parameters. These formats enable interoperability across laboratories, sequencing platforms, and agricultural systems worldwide.

## 1. Soil Sample Metadata Schema

### JSON Schema for Sample Collection

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA-SOIL-MICROBIOME Sample Metadata",
  "type": "object",
  "required": ["standard", "version", "sample", "location", "collection"],
  "properties": {
    "standard": {
      "type": "string",
      "const": "WIA-SOIL-MICROBIOME"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "sample": {
      "type": "object",
      "required": ["id", "type", "depth_cm", "mass_g"],
      "properties": {
        "id": {
          "type": "string",
          "pattern": "^WSM-[A-Z]{2}-\\d{8}-\\d{4}$",
          "description": "Format: WSM-{CountryCode}-{YYYYMMDD}-{SequenceNumber}"
        },
        "type": {
          "type": "string",
          "enum": ["bulk", "rhizosphere", "aggregate", "core", "composite"]
        },
        "depth_cm": {
          "type": "object",
          "required": ["min", "max"],
          "properties": {
            "min": {"type": "number", "minimum": 0, "maximum": 500},
            "max": {"type": "number", "minimum": 0, "maximum": 500}
          }
        },
        "mass_g": {
          "type": "number",
          "minimum": 0.1,
          "description": "Fresh soil mass in grams"
        },
        "replicates": {
          "type": "integer",
          "minimum": 1,
          "maximum": 100
        },
        "parent_sample_id": {
          "type": "string",
          "description": "ID of parent composite sample if applicable"
        }
      }
    },
    "location": {
      "type": "object",
      "required": ["coordinates", "datum"],
      "properties": {
        "coordinates": {
          "type": "object",
          "required": ["latitude", "longitude"],
          "properties": {
            "latitude": {"type": "number", "minimum": -90, "maximum": 90},
            "longitude": {"type": "number", "minimum": -180, "maximum": 180},
            "elevation_m": {"type": "number"},
            "accuracy_m": {"type": "number", "minimum": 0}
          }
        },
        "datum": {
          "type": "string",
          "enum": ["WGS84", "NAD83", "ETRS89"],
          "default": "WGS84"
        },
        "field_id": {"type": "string"},
        "farm_name": {"type": "string"},
        "plot_id": {"type": "string"},
        "management_zone": {"type": "string"}
      }
    },
    "collection": {
      "type": "object",
      "required": ["timestamp", "collector", "method"],
      "properties": {
        "timestamp": {
          "type": "string",
          "format": "date-time"
        },
        "collector": {
          "type": "object",
          "required": ["name", "organization"],
          "properties": {
            "name": {"type": "string"},
            "organization": {"type": "string"},
            "email": {"type": "string", "format": "email"},
            "certification": {"type": "string"}
          }
        },
        "method": {
          "type": "string",
          "enum": ["manual_auger", "hydraulic_probe", "excavation", "drone_sampling"]
        },
        "weather_conditions": {
          "type": "object",
          "properties": {
            "temperature_c": {"type": "number"},
            "precipitation_mm_24h": {"type": "number"},
            "soil_moisture_percent": {"type": "number"},
            "days_since_rain": {"type": "integer"}
          }
        }
      }
    },
    "soil_properties": {
      "type": "object",
      "properties": {
        "texture": {
          "type": "object",
          "properties": {
            "sand_percent": {"type": "number", "minimum": 0, "maximum": 100},
            "silt_percent": {"type": "number", "minimum": 0, "maximum": 100},
            "clay_percent": {"type": "number", "minimum": 0, "maximum": 100},
            "texture_class": {
              "type": "string",
              "enum": ["sand", "loamy_sand", "sandy_loam", "loam", "silt_loam", "silt", "sandy_clay_loam", "clay_loam", "silty_clay_loam", "sandy_clay", "silty_clay", "clay"]
            }
          }
        },
        "chemistry": {
          "type": "object",
          "properties": {
            "ph": {"type": "number", "minimum": 3, "maximum": 11},
            "ph_method": {"type": "string", "enum": ["water", "CaCl2", "KCl"]},
            "organic_matter_percent": {"type": "number", "minimum": 0, "maximum": 100},
            "total_carbon_percent": {"type": "number", "minimum": 0, "maximum": 100},
            "total_nitrogen_percent": {"type": "number", "minimum": 0, "maximum": 10},
            "c_n_ratio": {"type": "number", "minimum": 0, "maximum": 100},
            "cec_cmol_kg": {"type": "number", "minimum": 0, "description": "Cation exchange capacity"},
            "electrical_conductivity_ds_m": {"type": "number", "minimum": 0}
          }
        },
        "nutrients": {
          "type": "object",
          "properties": {
            "nitrogen_ppm": {"type": "number", "minimum": 0},
            "phosphorus_ppm": {"type": "number", "minimum": 0},
            "potassium_ppm": {"type": "number", "minimum": 0},
            "calcium_ppm": {"type": "number", "minimum": 0},
            "magnesium_ppm": {"type": "number", "minimum": 0},
            "sulfur_ppm": {"type": "number", "minimum": 0},
            "micronutrients": {
              "type": "object",
              "properties": {
                "iron_ppm": {"type": "number"},
                "manganese_ppm": {"type": "number"},
                "zinc_ppm": {"type": "number"},
                "copper_ppm": {"type": "number"},
                "boron_ppm": {"type": "number"},
                "molybdenum_ppm": {"type": "number"}
              }
            }
          }
        },
        "physical": {
          "type": "object",
          "properties": {
            "bulk_density_g_cm3": {"type": "number", "minimum": 0.5, "maximum": 2.5},
            "porosity_percent": {"type": "number", "minimum": 0, "maximum": 100},
            "water_holding_capacity_percent": {"type": "number", "minimum": 0, "maximum": 100},
            "aggregate_stability_percent": {"type": "number", "minimum": 0, "maximum": 100},
            "penetration_resistance_mpa": {"type": "number", "minimum": 0}
          }
        }
      }
    },
    "land_use": {
      "type": "object",
      "properties": {
        "current_crop": {"type": "string"},
        "previous_crop": {"type": "string"},
        "tillage_practice": {
          "type": "string",
          "enum": ["no_till", "reduced_till", "conventional_till", "strip_till"]
        },
        "cover_crop": {"type": "boolean"},
        "cover_crop_species": {"type": "array", "items": {"type": "string"}},
        "rotation_years": {"type": "integer"},
        "organic_certified": {"type": "boolean"},
        "years_in_current_management": {"type": "integer"},
        "irrigation": {
          "type": "string",
          "enum": ["none", "drip", "sprinkler", "flood", "subsurface"]
        },
        "fertilization": {
          "type": "object",
          "properties": {
            "type": {"type": "string", "enum": ["synthetic", "organic", "mixed", "none"]},
            "last_application_days": {"type": "integer"},
            "annual_n_kg_ha": {"type": "number"}
          }
        }
      }
    }
  }
}
```

### Example Sample Metadata

```json
{
  "standard": "WIA-SOIL-MICROBIOME",
  "version": "1.0.0",
  "sample": {
    "id": "WSM-US-20251229-0001",
    "type": "rhizosphere",
    "depth_cm": {"min": 0, "max": 15},
    "mass_g": 25.5,
    "replicates": 3
  },
  "location": {
    "coordinates": {
      "latitude": 40.7128,
      "longitude": -74.0060,
      "elevation_m": 125,
      "accuracy_m": 2.5
    },
    "datum": "WGS84",
    "field_id": "FIELD-A-01",
    "farm_name": "Sunrise Organic Farm",
    "management_zone": "ZONE-3"
  },
  "collection": {
    "timestamp": "2025-12-29T10:30:00Z",
    "collector": {
      "name": "Dr. Jane Smith",
      "organization": "Agricultural Research Institute",
      "email": "j.smith@agri-research.org",
      "certification": "SSSA-CERT-2024"
    },
    "method": "manual_auger",
    "weather_conditions": {
      "temperature_c": 18.5,
      "precipitation_mm_24h": 0,
      "soil_moisture_percent": 22.3,
      "days_since_rain": 5
    }
  },
  "soil_properties": {
    "texture": {
      "sand_percent": 45,
      "silt_percent": 35,
      "clay_percent": 20,
      "texture_class": "loam"
    },
    "chemistry": {
      "ph": 6.8,
      "ph_method": "water",
      "organic_matter_percent": 3.2,
      "total_carbon_percent": 1.86,
      "total_nitrogen_percent": 0.15,
      "c_n_ratio": 12.4,
      "cec_cmol_kg": 18.5
    },
    "nutrients": {
      "nitrogen_ppm": 25,
      "phosphorus_ppm": 42,
      "potassium_ppm": 180
    }
  },
  "land_use": {
    "current_crop": "corn",
    "previous_crop": "soybean",
    "tillage_practice": "no_till",
    "cover_crop": true,
    "cover_crop_species": ["rye", "vetch"],
    "organic_certified": true,
    "years_in_current_management": 7
  }
}
```

## 2. Microbial Community Composition Schema

### Taxonomic Abundance Format

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA-SOIL-MICROBIOME Taxonomic Profile",
  "type": "object",
  "required": ["standard", "version", "sample_id", "sequencing", "taxonomy"],
  "properties": {
    "standard": {"type": "string", "const": "WIA-SOIL-MICROBIOME"},
    "version": {"type": "string"},
    "sample_id": {"type": "string"},
    "sequencing": {
      "type": "object",
      "required": ["platform", "target_gene", "read_count"],
      "properties": {
        "platform": {
          "type": "string",
          "enum": ["illumina_miseq", "illumina_nextseq", "illumina_novaseq", "pacbio_sequel", "oxford_nanopore", "ion_torrent"]
        },
        "target_gene": {
          "type": "string",
          "enum": ["16S_v3v4", "16S_v4", "16S_full_length", "ITS1", "ITS2", "18S", "shotgun_metagenome"]
        },
        "read_count": {"type": "integer", "minimum": 1000},
        "read_length": {"type": "integer"},
        "quality_score_mean": {"type": "number", "minimum": 0, "maximum": 50},
        "primers": {
          "type": "object",
          "properties": {
            "forward": {"type": "string"},
            "reverse": {"type": "string"}
          }
        }
      }
    },
    "taxonomy": {
      "type": "object",
      "required": ["database", "classifier", "taxa"],
      "properties": {
        "database": {
          "type": "string",
          "enum": ["SILVA_138", "SILVA_139", "Greengenes_13_8", "GTDB_r207", "UNITE_9.0", "RDP_18"]
        },
        "classifier": {
          "type": "string",
          "enum": ["naive_bayes", "vsearch", "blast", "kraken2", "metaphlan4"]
        },
        "classification_confidence": {"type": "number", "minimum": 0, "maximum": 1},
        "taxa": {
          "type": "array",
          "items": {
            "type": "object",
            "required": ["otu_id", "abundance", "taxonomy"],
            "properties": {
              "otu_id": {"type": "string"},
              "asv_sequence": {"type": "string", "pattern": "^[ACGT]+$"},
              "abundance": {"type": "number", "minimum": 0},
              "abundance_type": {"type": "string", "enum": ["relative", "absolute_cells_per_g"]},
              "taxonomy": {
                "type": "object",
                "properties": {
                  "domain": {"type": "string"},
                  "phylum": {"type": "string"},
                  "class": {"type": "string"},
                  "order": {"type": "string"},
                  "family": {"type": "string"},
                  "genus": {"type": "string"},
                  "species": {"type": "string"}
                }
              },
              "confidence_scores": {
                "type": "object",
                "properties": {
                  "domain": {"type": "number", "minimum": 0, "maximum": 1},
                  "phylum": {"type": "number", "minimum": 0, "maximum": 1},
                  "class": {"type": "number", "minimum": 0, "maximum": 1},
                  "order": {"type": "number", "minimum": 0, "maximum": 1},
                  "family": {"type": "number", "minimum": 0, "maximum": 1},
                  "genus": {"type": "number", "minimum": 0, "maximum": 1},
                  "species": {"type": "number", "minimum": 0, "maximum": 1}
                }
              }
            }
          }
        }
      }
    },
    "diversity_metrics": {
      "type": "object",
      "properties": {
        "alpha_diversity": {
          "type": "object",
          "properties": {
            "observed_otus": {"type": "integer", "minimum": 0},
            "shannon_index": {"type": "number", "minimum": 0},
            "simpson_index": {"type": "number", "minimum": 0, "maximum": 1},
            "chao1": {"type": "number", "minimum": 0},
            "ace": {"type": "number", "minimum": 0},
            "evenness": {"type": "number", "minimum": 0, "maximum": 1}
          }
        },
        "rarefaction_depth": {"type": "integer"},
        "good_coverage": {"type": "number", "minimum": 0, "maximum": 1}
      }
    }
  }
}
```

### Example Taxonomic Profile

```json
{
  "standard": "WIA-SOIL-MICROBIOME",
  "version": "1.0.0",
  "sample_id": "WSM-US-20251229-0001",
  "sequencing": {
    "platform": "illumina_miseq",
    "target_gene": "16S_v3v4",
    "read_count": 45230,
    "read_length": 300,
    "quality_score_mean": 35.2,
    "primers": {
      "forward": "CCTACGGGNGGCWGCAG",
      "reverse": "GACTACHVGGGTATCTAATCC"
    }
  },
  "taxonomy": {
    "database": "SILVA_138",
    "classifier": "naive_bayes",
    "classification_confidence": 0.8,
    "taxa": [
      {
        "otu_id": "ASV_0001",
        "asv_sequence": "TACGTAGGTGGCAAGCGTTGTCCGGATTTACTGGGTGTAAAGGGAGCGTAGACGGCAAG...",
        "abundance": 0.082,
        "abundance_type": "relative",
        "taxonomy": {
          "domain": "Bacteria",
          "phylum": "Proteobacteria",
          "class": "Alphaproteobacteria",
          "order": "Rhizobiales",
          "family": "Bradyrhizobiaceae",
          "genus": "Bradyrhizobium",
          "species": "Bradyrhizobium japonicum"
        },
        "confidence_scores": {
          "domain": 1.0,
          "phylum": 0.99,
          "class": 0.98,
          "order": 0.95,
          "family": 0.92,
          "genus": 0.87,
          "species": 0.75
        }
      },
      {
        "otu_id": "ASV_0002",
        "abundance": 0.065,
        "abundance_type": "relative",
        "taxonomy": {
          "domain": "Bacteria",
          "phylum": "Actinobacteria",
          "class": "Actinobacteria",
          "order": "Streptomycetales",
          "family": "Streptomycetaceae",
          "genus": "Streptomyces",
          "species": "unclassified"
        }
      }
    ]
  },
  "diversity_metrics": {
    "alpha_diversity": {
      "observed_otus": 1247,
      "shannon_index": 6.82,
      "simpson_index": 0.98,
      "chao1": 1456.3,
      "evenness": 0.94
    },
    "rarefaction_depth": 10000,
    "good_coverage": 0.96
  }
}
```

## 3. Functional Gene Data Schema

### Gene Abundance Profile

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA-SOIL-MICROBIOME Functional Profile",
  "type": "object",
  "required": ["standard", "version", "sample_id", "functional_data"],
  "properties": {
    "standard": {"type": "string", "const": "WIA-SOIL-MICROBIOME"},
    "version": {"type": "string"},
    "sample_id": {"type": "string"},
    "functional_data": {
      "type": "object",
      "properties": {
        "annotation_database": {
          "type": "string",
          "enum": ["KEGG", "COG", "eggNOG", "SEED", "MetaCyc", "CAZy", "Pfam"]
        },
        "analysis_method": {
          "type": "string",
          "enum": ["picrust2", "humann3", "metaphlan4", "direct_metagenomic"]
        },
        "pathways": {
          "type": "array",
          "items": {
            "type": "object",
            "required": ["pathway_id", "pathway_name", "abundance"],
            "properties": {
              "pathway_id": {"type": "string"},
              "pathway_name": {"type": "string"},
              "abundance": {"type": "number"},
              "abundance_units": {"type": "string", "enum": ["rpk", "tpm", "copies_per_g_soil"]},
              "category": {"type": "string"},
              "genes": {
                "type": "array",
                "items": {
                  "type": "object",
                  "properties": {
                    "gene_id": {"type": "string"},
                    "gene_name": {"type": "string"},
                    "ec_number": {"type": "string"},
                    "abundance": {"type": "number"}
                  }
                }
              }
            }
          }
        },
        "functional_guilds": {
          "type": "object",
          "properties": {
            "nitrogen_cycle": {
              "type": "object",
              "properties": {
                "nitrogen_fixation": {"type": "number"},
                "nitrification": {"type": "number"},
                "denitrification": {"type": "number"},
                "anammox": {"type": "number"},
                "dissimilatory_nitrate_reduction": {"type": "number"}
              }
            },
            "carbon_cycle": {
              "type": "object",
              "properties": {
                "carbon_fixation": {"type": "number"},
                "methane_oxidation": {"type": "number"},
                "methanogenesis": {"type": "number"},
                "cellulose_degradation": {"type": "number"},
                "lignin_degradation": {"type": "number"}
              }
            },
            "phosphorus_cycle": {
              "type": "object",
              "properties": {
                "phosphate_solubilization": {"type": "number"},
                "polyphosphate_accumulation": {"type": "number"},
                "phytase_activity": {"type": "number"}
              }
            },
            "sulfur_cycle": {
              "type": "object",
              "properties": {
                "sulfur_oxidation": {"type": "number"},
                "sulfate_reduction": {"type": "number"},
                "dimethylsulfoniopropionate_degradation": {"type": "number"}
              }
            }
          }
        }
      }
    }
  }
}
```

## 4. Biomarker Catalog

### Soil Health Biomarkers

| Biomarker Category | Biomarker Name | Measurement Type | Target Range | Units | WIA Code |
|-------------------|----------------|------------------|--------------|-------|----------|
| **Microbial Biomass** | Total bacterial abundance | qPCR | 10^8 - 10^10 | cells/g dry soil | WSM-BM-001 |
| | Total fungal abundance | qPCR | 10^6 - 10^8 | cells/g dry soil | WSM-BM-002 |
| | Bacteria:Fungi ratio | Calculated | 10:1 - 100:1 | ratio | WSM-BM-003 |
| | Microbial biomass carbon | Fumigation-extraction | 200-1000 | mg C/kg soil | WSM-BM-004 |
| **Nitrogen Cycling** | nifH gene (N-fixation) | qPCR | 10^6 - 10^8 | copies/g soil | WSM-BM-101 |
| | amoA bacteria (Nitrification) | qPCR | 10^5 - 10^7 | copies/g soil | WSM-BM-102 |
| | amoA archaea (Nitrification) | qPCR | 10^4 - 10^6 | copies/g soil | WSM-BM-103 |
| | nirK/nirS (Denitrification) | qPCR | 10^6 - 10^8 | copies/g soil | WSM-BM-104 |
| | nosZ (N2O reduction) | qPCR | 10^5 - 10^7 | copies/g soil | WSM-BM-105 |
| **Carbon Cycling** | Cellulase genes | Metagenomics | Variable | RPK | WSM-BM-201 |
| | Lignin peroxidase | Metagenomics | Variable | RPK | WSM-BM-202 |
| | Methanotroph abundance | qPCR/16S | 10^4 - 10^6 | cells/g soil | WSM-BM-203 |
| **Phosphorus** | phoD gene (P-solubilization) | qPCR | 10^6 - 10^8 | copies/g soil | WSM-BM-301 |
| | Arbuscular mycorrhizal fungi | Microscopy/qPCR | 5-50 | % root colonization | WSM-BM-302 |
| **Disease Suppression** | Pseudomonas spp. | 16S/qPCR | >1% | relative abundance | WSM-BM-401 |
| | Trichoderma spp. | ITS/qPCR | >0.5% | relative abundance | WSM-BM-402 |
| | Bacillus spp. | 16S/qPCR | >1% | relative abundance | WSM-BM-403 |
| **Diversity Indices** | Shannon diversity (bacteria) | 16S sequencing | >5.0 | index | WSM-BM-501 |
| | Shannon diversity (fungi) | ITS sequencing | >3.0 | index | WSM-BM-502 |
| | Functional diversity | Metagenomics | >4.0 | index | WSM-BM-503 |

### Climate-Relevant Biomarkers

| Function | Gene/Taxa | Method | Climate Impact | WIA Code |
|----------|-----------|--------|----------------|----------|
| Carbon sequestration | Glomalin-related proteins | Bradford assay | +Carbon storage | WSM-CL-001 |
| GHG emissions | mcrA (methane production) | qPCR | +CH4 emissions | WSM-CL-002 |
| GHG mitigation | pmoA (methane oxidation) | qPCR | -CH4 emissions | WSM-CL-003 |
| N2O emissions | nirK/nirS ratio | qPCR | +N2O production | WSM-CL-004 |
| N2O mitigation | nosZ clade II | qPCR | -N2O emissions | WSM-CL-005 |
| SOC formation | Actinobacteria abundance | 16S | +SOC stability | WSM-CL-006 |
| Aggregate formation | Fungal:Bacterial ratio | 16S+ITS | +SOC protection | WSM-CL-007 |

## 5. Data Validation Rules

### Required Field Validation

```javascript
// Validation rules for sample metadata
const validationRules = {
  sample_id: {
    pattern: /^WSM-[A-Z]{2}-\d{8}-\d{4}$/,
    required: true,
    uniqueness: "global"
  },
  location: {
    latitude: {
      min: -90,
      max: 90,
      precision: 6, // decimal places
      required: true
    },
    longitude: {
      min: -180,
      max: 180,
      precision: 6,
      required: true
    }
  },
  soil_properties: {
    texture: {
      sum_check: ["sand_percent", "silt_percent", "clay_percent"],
      sum_must_equal: 100,
      tolerance: 2 // ±2%
    },
    ph: {
      min: 3.0,
      max: 11.0,
      precision: 1,
      plausibility_warning: {
        min: 4.0,
        max: 9.0
      }
    }
  },
  sequencing: {
    read_count: {
      min: 1000,
      recommended_min: 10000,
      optimal_min: 30000
    },
    quality_score: {
      min: 20,
      recommended_min: 30
    }
  }
};
```

### Data Quality Flags

```json
{
  "quality_control": {
    "flags": [
      {
        "code": "QC-PASS",
        "description": "All quality checks passed",
        "severity": "info"
      },
      {
        "code": "QC-WARN-DEPTH",
        "description": "Sequencing depth below recommended threshold",
        "severity": "warning",
        "threshold": 10000
      },
      {
        "code": "QC-WARN-CONTAMINATION",
        "description": "Potential contamination detected",
        "severity": "warning",
        "criteria": "chloroplast_mitochondrial_reads > 5%"
      },
      {
        "code": "QC-FAIL-QUALITY",
        "description": "Quality score below minimum threshold",
        "severity": "error",
        "threshold": 20
      },
      {
        "code": "QC-FAIL-BLANK",
        "description": "Sample similar to extraction blank",
        "severity": "error",
        "criteria": "bray_curtis_distance < 0.3"
      }
    ]
  }
}
```

## 6. Binary Data Formats

### Compressed Abundance Matrix (CAM)

For efficient storage of large-scale microbiome data:

**Header (128 bytes):**

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 4 | magic | Magic number: 0x57534D43 ("WSMC") |
| 4 | 2 | version_major | Major version |
| 6 | 2 | version_minor | Minor version |
| 8 | 4 | num_samples | Number of samples |
| 12 | 4 | num_taxa | Number of taxa/OTUs |
| 16 | 4 | matrix_offset | Offset to abundance matrix |
| 20 | 4 | taxa_offset | Offset to taxa names |
| 24 | 4 | sample_offset | Offset to sample IDs |
| 28 | 1 | compression | Compression method (0=none, 1=gzip, 2=lz4) |
| 29 | 1 | data_type | Data type (0=uint16, 1=uint32, 2=float32) |
| 30 | 2 | scale_factor | Scale factor for integer encoding |
| 32 | 96 | reserved | Reserved for future use |

**Data sections:**
1. Taxa names (NULL-terminated strings)
2. Sample IDs (NULL-terminated strings)
3. Abundance matrix (row-major, samples × taxa)
4. Optional metadata (JSON)

### FASTQ Quality Encoding

Standard Illumina 1.8+ encoding (Phred+33):

```
Quality scores: 0-41
ASCII range: ! (33) to J (74)
Formula: Q = -10 * log10(P_error)
```

## 7. File Format Specifications

### File Extensions and MIME Types

| Data Type | Extension | MIME Type | Compression |
|-----------|-----------|-----------|-------------|
| Sample metadata | .wsm-sample.json | application/vnd.wia.soil-microbiome+json | Optional .gz |
| Taxonomic profile | .wsm-taxa.json | application/vnd.wia.soil-microbiome+json | Optional .gz |
| Functional profile | .wsm-func.json | application/vnd.wia.soil-microbiome+json | Optional .gz |
| Abundance matrix | .wsm-matrix.cam | application/vnd.wia.soil-microbiome.matrix | Built-in |
| FASTQ sequences | .wsm.fastq.gz | application/gzip | Required |
| FASTA sequences | .wsm.fasta | text/plain | Optional .gz |
| BIOM format | .wsm.biom | application/json | Optional .gz |

### CSV Export Format

For data interchange and Excel compatibility:

```csv
WIA-SOIL-MICROBIOME,1.0.0
sample_id,WSM-US-20251229-0001
collection_date,2025-12-29T10:30:00Z
latitude,40.7128
longitude,-74.0060

taxonomy_level,otu_id,abundance_percent,classification
phylum,OTU_0001,12.5,Proteobacteria
phylum,OTU_0002,8.3,Actinobacteria
phylum,OTU_0003,7.1,Acidobacteria
class,OTU_0001,12.5,Alphaproteobacteria
```

## 8. Data Type Reference

### Enumerated Types

```typescript
enum SampleType {
  BULK = "bulk",
  RHIZOSPHERE = "rhizosphere",
  AGGREGATE = "aggregate",
  CORE = "core",
  COMPOSITE = "composite"
}

enum SequencingPlatform {
  ILLUMINA_MISEQ = "illumina_miseq",
  ILLUMINA_NEXTSEQ = "illumina_nextseq",
  ILLUMINA_NOVASEQ = "illumina_novaseq",
  PACBIO_SEQUEL = "pacbio_sequel",
  OXFORD_NANOPORE = "oxford_nanopore",
  ION_TORRENT = "ion_torrent"
}

enum TargetGene {
  SIXTEENS_V3V4 = "16S_v3v4",
  SIXTEENS_V4 = "16S_v4",
  SIXTEENS_FULL = "16S_full_length",
  ITS1 = "ITS1",
  ITS2 = "ITS2",
  EIGHTEENS = "18S",
  SHOTGUN = "shotgun_metagenome"
}

enum TillagePractice {
  NO_TILL = "no_till",
  REDUCED_TILL = "reduced_till",
  CONVENTIONAL_TILL = "conventional_till",
  STRIP_TILL = "strip_till"
}

enum TextureClass {
  SAND = "sand",
  LOAMY_SAND = "loamy_sand",
  SANDY_LOAM = "sandy_loam",
  LOAM = "loam",
  SILT_LOAM = "silt_loam",
  SILT = "silt",
  SANDY_CLAY_LOAM = "sandy_clay_loam",
  CLAY_LOAM = "clay_loam",
  SILTY_CLAY_LOAM = "silty_clay_loam",
  SANDY_CLAY = "sandy_clay",
  SILTY_CLAY = "silty_clay",
  CLAY = "clay"
}
```

## 9. Metadata Standards Compliance

### Compatibility with Existing Standards

WIA-SOIL-MICROBIOME maintains compatibility with:

| Standard | Compliance Level | Mapping |
|----------|-----------------|---------|
| MIxS (Minimum Information about any Sequence) | Full | Direct field mapping |
| GSC (Genomic Standards Consortium) | Full | Superset of required fields |
| MIMARKS (Marker gene) | Full | Extended with soil-specific fields |
| MIMS (Metagenome) | Full | Compatible data structure |
| Darwin Core | Partial | Location and temporal data |
| NCBI BioSample | Full | Can export to BioSample XML |
| ENA (European Nucleotide Archive) | Full | Compatible metadata |
| ISO 23400 (Soil quality) | Partial | Physical/chemical methods |

### Field Mapping to MIxS

```json
{
  "mixs_mapping": {
    "env_broad_scale": "land_use.current_crop",
    "env_local_scale": "location.field_id",
    "env_medium": "soil_properties.texture.texture_class",
    "geo_loc_name": "location.farm_name",
    "lat_lon": "location.coordinates",
    "collection_date": "collection.timestamp",
    "depth": "sample.depth_cm",
    "elev": "location.coordinates.elevation_m",
    "temp": "collection.weather_conditions.temperature_c",
    "ph": "soil_properties.chemistry.ph",
    "tot_org_carb": "soil_properties.chemistry.total_carbon_percent",
    "tot_nitro": "soil_properties.chemistry.total_nitrogen_percent"
  }
}
```

## 10. Implementation Requirements

### Minimum Viable Implementation (MVI)

A WIA-SOIL-MICROBIOME compliant system MUST:

1. **Parse and validate** all required fields in sample metadata schema
2. **Generate valid sample IDs** following WSM-{CC}-{DATE}-{SEQ} format
3. **Store GPS coordinates** with minimum 5 decimal places precision
4. **Support at least one sequencing platform** (Illumina recommended)
5. **Handle taxonomic data** with domain through genus levels
6. **Calculate alpha diversity** metrics (Shannon, Simpson minimum)
7. **Export data** in JSON format with proper schema validation
8. **Implement quality control** flags and validation rules

### Recommended Implementation

A WIA-SOIL-MICROBIOME compliant system SHOULD:

1. Support multiple sequencing platforms and target genes
2. Provide functional gene analysis capabilities
3. Calculate beta diversity metrics between samples
4. Support spatial data visualization
5. Enable data export to BIOM, CSV, and binary CAM formats
6. Implement automated QC pipelines
7. Provide API access to data (see Phase 2)
8. Support bulk upload and processing

### Full Implementation

A WIA-SOIL-MICROBIOME compliant system MAY:

1. Integrate with laboratory information management systems (LIMS)
2. Provide real-time quality control during sequencing
3. Implement machine learning for sample classification
4. Support multi-omics integration (metatranscriptomics, metaproteomics)
5. Provide carbon credit calculation from microbiome data
6. Enable federated data queries across institutions
7. Support blockchain-based data provenance tracking
8. Implement privacy-preserving data sharing mechanisms

## 11. Data Versioning and Provenance

### Provenance Tracking

```json
{
  "provenance": {
    "data_version": "1.0.0",
    "created_at": "2025-12-29T10:30:00Z",
    "created_by": {
      "name": "Dr. Jane Smith",
      "organization": "Agricultural Research Institute",
      "orcid": "0000-0001-2345-6789"
    },
    "processing_pipeline": {
      "name": "QIIME2",
      "version": "2024.5",
      "steps": [
        {
          "step": "demultiplexing",
          "tool": "cutadapt",
          "version": "4.1",
          "parameters": {"error_rate": 0.1}
        },
        {
          "step": "denoising",
          "tool": "DADA2",
          "version": "1.28",
          "parameters": {"trunc_len_f": 250, "trunc_len_r": 200}
        },
        {
          "step": "taxonomy_classification",
          "tool": "sklearn",
          "version": "1.3.0",
          "database": "SILVA_138",
          "confidence": 0.8
        }
      ]
    },
    "modifications": [
      {
        "timestamp": "2025-12-29T14:00:00Z",
        "modified_by": "automated_qc",
        "change": "Flagged low quality samples",
        "affected_fields": ["quality_control.flags"]
      }
    ],
    "data_integrity": {
      "checksum_algorithm": "SHA-256",
      "checksum": "a1b2c3d4e5f6..."
    }
  }
}
```

---

**© 2025 SmileStory Inc. / WIA - World Certification Industry Association**
**弘益人間 · Benefit All Humanity**
