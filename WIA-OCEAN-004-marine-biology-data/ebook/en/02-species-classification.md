# Chapter 2: Species Classification and Taxonomy

## Organizing Marine Biodiversity

The ocean contains an estimated 2.2 million species, from microscopic bacteria to the blue whale - the largest animal ever to exist. Organizing this incredible diversity requires sophisticated classification systems that enable scientists worldwide to communicate precisely about marine life.

### The Challenge of Marine Taxonomy

Unlike terrestrial organisms, marine species present unique taxonomic challenges:

**Cryptic Species:** Organisms that look identical but are genetically distinct. The European shore crab (*Carcinus maenas*) actually comprises several cryptic species.

**Morphological Plasticity:** Environmental conditions dramatically affect appearance. Deep-sea fish at different depths may look completely different despite being the same species.

**Larval Stages:** Many marine organisms have planktonic larvae that look nothing like adults, making identification difficult.

**Undiscovered Diversity:** The deep ocean remains 95% unexplored. We're discovering **2,000+ new marine species annually**.

**Historical Complexity:** Species described in the 1700s-1800s often have incomplete or inaccurate descriptions, requiring modern revision.

### Linnaean Taxonomy

Modern taxonomy traces back to Carl Linnaeus (1758), who established binomial nomenclature - the two-part scientific name system still used today.

#### Taxonomic Hierarchy

```typescript
interface TaxonomicClassification {
  domain: string;      // Eukarya, Bacteria, Archaea
  kingdom: string;     // Animalia, Plantae, Chromista, etc.
  phylum: string;      // Chordata, Arthropoda, Mollusca, etc.
  class: string;       // Mammalia, Actinopterygii, Gastropoda
  order: string;       // Cetacea, Perciformes, Nudibranchia
  family: string;      // Balaenopteridae, Labridae, Chromodorididae
  genus: string;       // Balaenoptera, Thalassoma, Chromodoris
  species: string;     // musculus, bifasciatum, quadricolor
  subspecies?: string; // Optional subspecific designation
  scientificName: string;  // Full binomial name
  authority: string;       // Author and year (e.g., "Linnaeus, 1758")
  taxonomicStatus: "accepted" | "synonym" | "unaccepted";
}

// Example: Blue whale
const blueWhale: TaxonomicClassification = {
  domain: "Eukarya",
  kingdom: "Animalia",
  phylum: "Chordata",
  class: "Mammalia",
  order: "Cetacea",
  family: "Balaenopteridae",
  genus: "Balaenoptera",
  species: "musculus",
  scientificName: "Balaenoptera musculus",
  authority: "(Linnaeus, 1758)",
  taxonomicStatus: "accepted"
};
```

### World Register of Marine Species (WoRMS)

WoRMS serves as the **authoritative global registry** for marine organism names, maintained by 300+ taxonomic editors worldwide.

#### WoRMS Architecture

**Database Scale:**
- **240,000+ accepted species** names
- **480,000+ synonyms** and historical names
- **56 phyla** represented
- **740,000+ total taxonomic names**

**Update Frequency:**
- **Daily updates** from expert editors
- **2,000-3,000 new species** added annually
- **Continuous revision** of existing records

#### AphiaID System

Every species in WoRMS receives a unique **AphiaID** - a permanent identifier that never changes, even if the scientific name is revised.

```typescript
interface WoRMSRecord {
  AphiaID: number;              // Unique identifier
  scientificname: string;       // Full scientific name
  authority: string;            // Author and year
  status: "accepted" | "unaccepted";
  rank: string;                 // Kingdom, Phylum, Class, etc.
  valid_AphiaID: number;        // Points to accepted name if synonym
  valid_name: string;           // Accepted scientific name
  kingdom: string;
  phylum: string;
  class: string;
  order: string;
  family: string;
  genus: string;
  citation: string;             // How to cite this record
  lsid: string;                 // Life Science Identifier
  isMarine: 0 | 1;
  isBrackish: 0 | 1;
  isFreshwater: 0 | 1;
  isTerrestrial: 0 | 1;
  isExtinct: 0 | 1;
  match_type: string;
  modified: string;             // ISO 8601 timestamp
}

// Real example: Great white shark
const greatWhiteShark: WoRMSRecord = {
  AphiaID: 105838,
  scientificname: "Carcharodon carcharias",
  authority: "(Linnaeus, 1758)",
  status: "accepted",
  rank: "Species",
  valid_AphiaID: 105838,
  valid_name: "Carcharodon carcharias",
  kingdom: "Animalia",
  phylum: "Chordata",
  class: "Elasmobranchii",
  order: "Lamniformes",
  family: "Lamnidae",
  genus: "Carcharodon",
  citation: "Froese, R. and D. Pauly. Editors. (2024). FishBase...",
  lsid: "urn:lsid:marinespecies.org:taxname:105838",
  isMarine: 1,
  isBrackish: 0,
  isFreshwater: 0,
  isTerrestrial: 0,
  isExtinct: 0,
  match_type: "exact",
  modified: "2023-01-15T10:34:12Z"
};
```

#### WoRMS REST API

WoRMS provides programmatic access for automated taxonomy validation:

```typescript
class WoRMSClient {
  private baseURL = "https://www.marinespecies.org/rest";

  // Get species record by AphiaID
  async getRecordByAphiaID(aphiaID: number): Promise<WoRMSRecord> {
    const response = await fetch(`${this.baseURL}/AphiaRecordByAphiaID/${aphiaID}`);
    return response.json();
  }

  // Search for species by name
  async searchByName(name: string): Promise<WoRMSRecord[]> {
    const response = await fetch(
      `${this.baseURL}/AphiaRecordsByName/${encodeURIComponent(name)}?like=false&marine_only=true`
    );
    return response.json();
  }

  // Get full taxonomic classification
  async getClassification(aphiaID: number): Promise<TaxonomicClassification[]> {
    const response = await fetch(`${this.baseURL}/AphiaClassificationByAphiaID/${aphiaID}`);
    return response.json();
  }

  // Match multiple names (fuzzy matching)
  async matchNames(names: string[]): Promise<WoRMSRecord[]> {
    const response = await fetch(`${this.baseURL}/AphiaRecordsByMatchNames`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ scientificnames: names, marine_only: true })
    });
    return response.json();
  }

  // Get vernacular names (common names)
  async getVernaculars(aphiaID: number): Promise<VernacularName[]> {
    const response = await fetch(`${this.baseURL}/AphiaVernacularsByAphiaID/${aphiaID}`);
    return response.json();
  }
}

interface VernacularName {
  vernacular: string;     // Common name
  language_code: string;  // ISO 639 language code
  language: string;       // Language name
}

// Example usage
const worms = new WoRMSClient();

// Validate species name
const records = await worms.searchByName("Tursiops truncatus");
console.log(`Bottlenose dolphin AphiaID: ${records[0].AphiaID}`);

// Get full classification
const classification = await worms.getClassification(137111);
classification.forEach(taxon => {
  console.log(`${taxon.rank}: ${taxon.scientificname}`);
});

// Get common names
const vernaculars = await worms.getVernaculars(137111);
// Returns: "Common bottlenose dolphin" (English), "Gran delfín" (Spanish), etc.
```

### Handling Taxonomic Changes

Species names change as taxonomy is revised. Data systems must handle this gracefully:

```typescript
interface TaxonomicHistory {
  current: {
    aphiaID: number;
    scientificName: string;
    status: "accepted";
  };

  synonyms: {
    aphiaID: number;
    scientificName: string;
    status: "synonym" | "unaccepted";
    reason: string;
  }[];

  revisionHistory: {
    date: Date;
    previousName: string;
    newName: string;
    reason: string;
    reference: string;
  }[];
}

// Example: Indo-Pacific bottlenose dolphin
const dolphinTaxonomy: TaxonomicHistory = {
  current: {
    aphiaID: 254997,
    scientificName: "Tursiops aduncus",
    status: "accepted"
  },
  synonyms: [
    {
      aphiaID: 137111,
      scientificName: "Tursiops truncatus aduncus",
      status: "synonym",
      reason: "Elevated to species rank (2011)"
    }
  ],
  revisionHistory: [
    {
      date: new Date("2011-01-01"),
      previousName: "Tursiops truncatus aduncus",
      newName: "Tursiops aduncus",
      reason: "Genetic and morphological evidence supports species status"
    }
  ]
};
```

### Functional Traits Database

Beyond taxonomy, marine biologists need functional trait data - ecological characteristics that affect ecosystem function:

```typescript
interface MarineSpeciesTraits {
  aphiaID: number;
  scientificName: string;

  morphology: {
    bodySize: {
      typical: Measurement;      // mm, cm, or m
      maximum: Measurement;
      dimorphism?: "male_larger" | "female_larger" | "none";
    };
    bodyShape: string;           // "fusiform", "compressed", "elongate"
    mobility: "sessile" | "mobile" | "planktonic";
  };

  ecology: {
    habitat: string[];           // "coral_reef", "deep_sea", "pelagic"
    depth: {
      minimum: number;           // Meters
      maximum: number;
      typical: number;
    };
    temperature: {
      minimum: number;           // Celsius
      maximum: number;
      optimal: number;
    };
    salinity: {
      minimum: number;           // PSU
      maximum: number;
    };
  };

  trophicEcology: {
    trophicLevel: number;        // 1-5 scale
    feedingType: string;         // "carnivore", "herbivore", "filter_feeder"
    preyItems: string[];         // List of prey types
    predators: string[];         // Known predators
  };

  reproduction: {
    strategy: string;            // "broadcast_spawner", "brooder", "live_bearer"
    seasonality: string;         // "year_round", "seasonal"
    fecundity?: number;          // Eggs per reproductive event
    larvalType?: "planktonic" | "direct_development";
  };

  conservation: {
    iucnStatus?: "LC" | "NT" | "VU" | "EN" | "CR" | "EW" | "EX" | "DD";
    populationTrend?: "increasing" | "stable" | "decreasing" | "unknown";
    threats: string[];
  };
}

// Example: Coral grouper
const coralGrouper: MarineSpeciesTraits = {
  aphiaID: 212844,
  scientificName: "Plectropomus leopardus",

  morphology: {
    bodySize: {
      typical: { value: 50, unit: "cm", precision: 5, method: "field_measurement" },
      maximum: { value: 120, unit: "cm", precision: 5, method: "literature" }
    },
    bodyShape: "fusiform",
    mobility: "mobile"
  },

  ecology: {
    habitat: ["coral_reef", "reef_slope", "lagoon"],
    depth: { minimum: 3, maximum: 100, typical: 20 },
    temperature: { minimum: 22, maximum: 30, optimal: 26 },
    salinity: { minimum: 32, maximum: 37 }
  },

  trophicEcology: {
    trophicLevel: 4.2,
    feedingType: "carnivore",
    preyItems: ["fish", "cephalopods", "crustaceans"],
    predators: ["sharks", "larger_groupers", "humans"]
  },

  reproduction: {
    strategy: "broadcast_spawner",
    seasonality: "seasonal",
    fecundity: 200000,
    larvalType: "planktonic"
  },

  conservation: {
    iucnStatus: "VU",
    populationTrend: "decreasing",
    threats: ["overfishing", "habitat_degradation", "live_fish_trade"]
  }
};
```

### Molecular Taxonomy and DNA Barcoding

DNA sequencing has revolutionized species identification, especially for:
- Larvae and juvenile stages
- Cryptic species complexes
- Environmental DNA (eDNA) samples
- Gut content analysis
- Museum specimens

#### The Barcode of Life

The **Cytochrome c Oxidase I (COI)** gene serves as the primary DNA barcode for animals:

```typescript
interface DNABarcode {
  processID: string;          // BOLD process ID
  sampleID: string;           // Museum/lab specimen ID
  aphiaID: number;            // WoRMS linkage
  scientificName: string;

  marker: "COI" | "16S" | "18S" | "ITS";
  sequence: string;           // DNA sequence (A, T, G, C)
  sequenceLength: number;     // Base pairs

  similarity: {
    nearestMatch: string;     // Closest species in database
    identity: number;         // 0-100% similarity
    alignmentScore: number;
  };

  specimen: {
    catalogNumber: string;
    institution: string;
    locality: string;
    coordinates: GeographicCoordinate;
    depth: Measurement;
    collectionDate: Date;
    collector: string;
    identifier: string;        // Taxonomist who identified it
  };

  sequence_quality: {
    ambiguousPositions: number;
    averageQualityScore: number;  // Phred score
    traceAvailable: boolean;
  };
}

// DNA barcode matching algorithm
class BarcodeMatcher {
  private database: Map<string, DNABarcode[]>;

  async identifySpecies(querySequence: string): Promise<{
    species: string;
    confidence: number;
    matches: DNABarcode[];
  }> {
    const matches = await this.searchDatabase(querySequence);

    // Sort by similarity
    matches.sort((a, b) => b.similarity.identity - a.similarity.identity);

    const topMatch = matches[0];

    // Species-level identification threshold: 98% similarity for COI
    if (topMatch.similarity.identity >= 98) {
      return {
        species: topMatch.scientificName,
        confidence: topMatch.similarity.identity / 100,
        matches: matches.slice(0, 5)
      };
    } else if (topMatch.similarity.identity >= 95) {
      return {
        species: `${topMatch.scientificName.split(' ')[0]} sp.`,  // Genus level
        confidence: topMatch.similarity.identity / 100,
        matches: matches.slice(0, 5)
      };
    } else {
      return {
        species: "Unknown",
        confidence: 0,
        matches: matches.slice(0, 5)
      };
    }
  }

  private async searchDatabase(sequence: string): Promise<DNABarcode[]> {
    // Use BLAST or similar algorithm for sequence alignment
    // Implementation details omitted for brevity
    return [];
  }
}
```

### Integrating Multiple Taxonomies

Marine data systems often need to integrate multiple taxonomic authorities:

```typescript
interface CrossTaxonomyMapping {
  worms: {
    aphiaID: number;
    scientificName: string;
  };

  ncbi: {
    taxonomyID: number;        // NCBI Taxonomy ID
    scientificName: string;
  };

  itis: {
    tsn: number;               // Taxonomic Serial Number
    scientificName: string;
  };

  gbif: {
    taxonKey: number;          // GBIF taxon key
    scientificName: string;
  };

  iucn: {
    taxonID: number;
    scientificName: string;
    redListCategory: string;
  };

  reconciliation: {
    status: "exact_match" | "fuzzy_match" | "conflict";
    confidence: number;
    notes: string;
  };
}

// Example: Blue whale across databases
const blueWhaleMapping: CrossTaxonomyMapping = {
  worms: {
    aphiaID: 137091,
    scientificName: "Balaenoptera musculus"
  },
  ncbi: {
    taxonomyID: 9771,
    scientificName: "Balaenoptera musculus"
  },
  itis: {
    tsn: 180528,
    scientificName: "Balaenoptera musculus"
  },
  gbif: {
    taxonKey: 2440843,
    scientificName: "Balaenoptera musculus"
  },
  iucn: {
    taxonID: 2477,
    scientificName: "Balaenoptera musculus",
    redListCategory: "Endangered"
  },
  reconciliation: {
    status: "exact_match",
    confidence: 1.0,
    notes: "All databases agree on species name and classification"
  }
};
```

### Practical Taxonomy Validation

When building marine biology databases, implement robust taxonomy validation:

```typescript
class TaxonomyValidator {
  private wormsClient: WoRMSClient;

  async validateSpeciesName(name: string): Promise<{
    valid: boolean;
    aphiaID?: number;
    acceptedName?: string;
    issues: string[];
  }> {
    const issues: string[] = [];

    // Check if name exists in WoRMS
    const results = await this.wormsClient.searchByName(name);

    if (results.length === 0) {
      return {
        valid: false,
        issues: [`Species name "${name}" not found in WoRMS`]
      };
    }

    // Check if name is accepted or synonym
    const record = results[0];

    if (record.status === "unaccepted") {
      issues.push(`"${name}" is an unaccepted name`);

      // Get accepted name
      const acceptedRecord = await this.wormsClient.getRecordByAphiaID(record.valid_AphiaID);
      issues.push(`Use accepted name: "${acceptedRecord.scientificname}"`);

      return {
        valid: false,
        aphiaID: acceptedRecord.AphiaID,
        acceptedName: acceptedRecord.scientificname,
        issues
      };
    }

    // Check if truly marine
    if (record.isMarine === 0) {
      issues.push(`"${name}" is not classified as a marine species in WoRMS`);
    }

    return {
      valid: issues.length === 0,
      aphiaID: record.AphiaID,
      acceptedName: record.scientificname,
      issues
    };
  }

  async validateDataset(observations: { scientificName: string }[]): Promise<{
    validCount: number;
    invalidCount: number;
    issues: { name: string; problems: string[] }[];
  }> {
    let validCount = 0;
    let invalidCount = 0;
    const issues: { name: string; problems: string[] }[] = [];

    for (const obs of observations) {
      const validation = await this.validateSpeciesName(obs.scientificName);

      if (validation.valid) {
        validCount++;
      } else {
        invalidCount++;
        issues.push({
          name: obs.scientificName,
          problems: validation.issues
        });
      }
    }

    return { validCount, invalidCount, issues };
  }
}
```

### Best Practices for Taxonomic Data

**1. Always Use AphiaIDs**
Store the WoRMS AphiaID alongside scientific names. Names change; IDs don't.

**2. Version Your Taxonomy**
Record when you queried WoRMS. Taxonomy evolves; track which version you used.

**3. Store Full Classification**
Keep the complete taxonomic hierarchy (kingdom through species) for filtering and analysis.

**4. Handle Synonyms Gracefully**
When users search for old names, redirect to accepted names with explanation.

**5. Validate on Input**
Check species names against WoRMS when data is entered, not months later.

**6. Link to Authoritative Sources**
Provide URLs to WoRMS, GBIF, IUCN records for verification.

**7. Document Uncertain Identifications**
Use qualifiers like "cf." (compare to), "aff." (affinis - similar to), "sp." (species unknown).

### Philosophy: Naming the Ocean

Every species name represents centuries of exploration, study, and dedication. The 240,000 species in WoRMS are the work of millions of hours by taxonomists worldwide, freely shared for the benefit of all humanity (弘益人間).

When you correctly identify a species, you connect your observation to:
- All other observations of that species worldwide
- Its evolutionary relationships
- Its ecological role
- Its conservation status
- Centuries of scientific knowledge

Taxonomy is not just naming - it's organizing our understanding of life itself.

---

**Next Chapter:** Environmental Monitoring Data - measuring the ocean's physical and chemical parameters that shape marine ecosystems.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
