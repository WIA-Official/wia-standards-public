# Chapter 5: Intellectual Property and Open Source

## WIA-SYNTHETIC-BIO-REGISTRY Standard
**Version:** 1.0
**Status:** Official WIA Standard
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 5.1 The Tension Between Openness and Protection

Biological part registries sit at the intersection of two powerful and sometimes conflicting forces: the open science movement that emphasizes sharing and accessibility, and the intellectual property system designed to incentivize innovation through exclusive rights.

### The Open Science Imperative

The synthetic biology community embraced open principles from its inception:

**Historical Context:**
The BioBricks Foundation, established in 2006, was built on open source principles borrowed from software development. Early pioneers believed that biology, like software, would advance fastest through sharing and collaboration rather than secrecy and exclusivity.

**Core Open Science Values:**

**Access:** Scientific knowledge and tools should be available to all researchers regardless of geographic location or institutional resources.

**Transparency:** Methods, data, and materials should be openly documented, enabling verification and reproducibility.

**Collaboration:** Progress accelerates when researchers build upon each other's work rather than duplicating efforts.

**Attribution:** Contributors receive credit for their innovations through citations and recognition rather than exclusive control.

**Public Benefit:** Science should prioritize societal benefit over private gain.

### The Role of Intellectual Property

Despite open ideals, intellectual property (IP) systems serve important functions:

**Investment Incentive:** Patent protection encourages companies to invest in developing biological technologies by providing temporary monopolies that enable recouping R&D costs.

**Disclosure:** Patent applications require detailed public disclosure of inventions, adding to collective knowledge.

**Quality Signal:** Patents undergo examination, providing some assurance of novelty and utility.

**Commercialization:** IP rights facilitate technology transfer from academic labs to industry, enabling real-world applications.

**Attribution:** IP systems provide legal recognition of inventorship and contribution.

### The Challenge for Registries

Biological part registries must navigate these competing interests:

**Contributor Concerns:**
- Academics worry about publishing research if parts are patented
- Industry researchers fear giving away competitive advantages
- Students uncertain about IP implications of contributions
- Institutions concerned about protecting potential commercial interests

**User Concerns:**
- Researchers need assurance they can use parts without infringement
- Companies require freedom to operate for commercial applications
- Downstream users worried about reach-through claims
- International users face varying IP landscapes

## 5.2 Patent Landscape for Biological Parts

Understanding how patent law applies to synthetic biology components is essential for registry operation.

### Patentability of Biological Parts

**Legal Requirements for Patents:**

**Novelty:** The invention must be new—not previously known or publicly disclosed.

**Non-Obviousness (Inventive Step):** The invention must not be obvious to a person skilled in the relevant field.

**Utility (Industrial Applicability):** The invention must have a practical application.

**Enablement:** The patent must describe the invention in sufficient detail that others can reproduce it.

**Patentable Subject Matter:** Must fall within categories eligible for patent protection (varies by jurisdiction).

### Types of Relevant Patents

Several patent categories may cover biological parts:

#### Composition of Matter Patents

Cover the DNA sequence itself as a new chemical composition.

**Example:**
"An isolated DNA molecule comprising the nucleotide sequence SEQ ID NO: 1, wherein said molecule encodes a protein with enhanced fluorescence compared to wild-type GFP."

**Scope:**
- The specific sequence claimed
- Substantially similar sequences (typically >95% identity)
- Functional equivalents (depends on claim language)

**Registry Implications:**
- Exact sequence may be patented
- Variants may infringe or be separately patentable
- Natural sequences generally not patentable (post-Myriad in US)

#### Method Patents

Cover processes for making or using biological parts.

**Example:**
"A method for expressing a heterologous protein in E. coli comprising:
1) Inserting a DNA sequence encoding said protein downstream of promoter SEQ ID NO: 1
2) Culturing the resulting strain under conditions that induce expression
3) Isolating the expressed protein"

**Scope:**
- Specific method steps claimed
- May cover using particular promoters/parts
- Doesn't prevent others from making/selling the part itself (in some jurisdictions)

**Registry Implications:**
- Part sequence may be freely shared
- Using the part in claimed method may infringe
- Alternative methods may avoid infringement

#### System/Device Patents

Cover combinations of parts or complete genetic circuits.

**Example:**
"A genetic circuit comprising:
- An inducible promoter responsive to Signal A
- Operably linked to a repressor protein gene for Signal B
- Said repressor controlling a second promoter
- Said second promoter driving an output gene
wherein the circuit implements Boolean AND logic"

**Scope:**
- Complete system as claimed
- Individual parts likely not covered
- Equivalent circuits may or may not infringe

**Registry Implications:**
- Individual parts typically freely usable
- Specific combinations may be restricted
- Alternative designs may avoid infringement

### Patent Searching and Freedom to Operate

Before using registry parts commercially, prudent organizations conduct patent landscape analysis:

**Search Strategy:**
```
1. Identify relevant parts and applications

2. Search patent databases:
   - USPTO (patents.google.com, USPTO.gov)
   - EPO (espacenet.com)
   - WIPO (patentscope.wipo.int)
   - National offices (JPO, CNIPA, etc.)

3. Search strategies:
   a) Sequence-based: BLAST against patent sequence databases
   b) Keyword-based: Terms for function and application
   c) Classification-based: IPC/CPC codes (C12N, C07K, etc.)
   d) Assignee-based: Key companies and universities

4. Analyze results:
   - Identify relevant patents
   - Check legal status (granted, pending, expired, abandoned)
   - Evaluate claim scope
   - Assess likelihood of infringement

5. Freedom to Operate (FTO) Opinion:
   - Legal analysis of infringement risk
   - Typically requires patent attorney
   - Jurisdiction-specific
   - Consider design-around options
```

**Registry Features Supporting FTO:**

**Patent Flagging:**
```json
{
  "part_id": "BBa_J23100",
  "patent_information": {
    "potentially_covered": false,
    "known_patents": [],
    "last_search_date": "2024-01-15",
    "notes": "Sequence search found no exact matches in patent databases"
  }
}
```

**Prior Art Documentation:**
Registry submissions with dated, public disclosure establish prior art that can prevent future patents:

```
Part: BBa_J23100
Published: 2006-07-15 (registry submission date)
Status: Public domain, predates most synthetic biology patents
Note: May serve as prior art against later patent applications
```

## 5.3 Open Material Transfer Agreement (OpenMTA)

The BioBricks Foundation developed the OpenMTA as a legal framework for open sharing of biological materials.

### OpenMTA Principles

**Key Provisions:**

**1. Freedom to Use:**
Recipients may use materials for any purpose, including commercial applications.

**2. Freedom to Distribute:**
Recipients may redistribute materials to others, provided recipients agree to the same terms.

**3. Attribution:**
Recipients should acknowledge the source of materials in publications.

**4. Sharing Improvements:**
Recipients who improve materials are encouraged (but not required) to share improvements under the same terms.

**5. No Additional Restrictions:**
Recipients may not add additional restrictions beyond those in the OpenMTA.

**6. No Warranty:**
Materials provided "as is" without warranties of any kind.

### OpenMTA Text (Simplified Summary)

```
OPENMTA (Open Material Transfer Agreement)

This Material Transfer Agreement ("Agreement") is between the
PROVIDER and the RECIPIENT.

MATERIALS: Biological materials described in attached documentation.

TERMS:

1. The PROVIDER retains ownership of the MATERIALS.

2. The RECIPIENT may:
   a) Use MATERIALS for any lawful purpose
   b) Modify and improve MATERIALS
   c) Distribute MATERIALS and modifications to others under these terms
   d) Use MATERIALS for commercial purposes

3. The RECIPIENT agrees to:
   a) Acknowledge source in publications
   b) Not add additional restrictions when redistributing
   c) Require recipients to agree to these terms

4. The RECIPIENT may NOT:
   a) Claim ownership of unmodified MATERIALS
   b) Patent unmodified MATERIALS
   c) Add restrictions limiting others' use

5. NO WARRANTY: MATERIALS provided "as is" without warranty.

6. LIABILITY: PROVIDER not liable for any outcomes from use.

7. This Agreement governed by laws of [jurisdiction].
```

### Alternative Licensing Models

While OpenMTA dominates academic registries, other models exist:

#### Creative Commons Licenses

Adapted from copyrightable works, applied to biological materials by analogy:

**CC0 (Public Domain Dedication):**
- Creator waives all rights
- Maximum freedom
- No attribution required (though appreciated)
- Used for some reference standards

**CC-BY (Attribution):**
- Use for any purpose
- Attribution required
- Most similar to OpenMTA
- Common in data sharing

**CC-BY-SA (Attribution-ShareAlike):**
- Attribution required
- Derivatives must use same license
- "Copyleft" approach
- Less common for biological parts

#### BSD/MIT-Style Licenses

Permissive licenses borrowed from software:

```
Permission is hereby granted, free of charge, to any person obtaining
a copy of this biological material and associated documentation, to
deal in the Material without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Material, and to permit persons to whom the
Material is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Material.

THE MATERIAL IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND...
```

#### GPL-Style "Copyleft" Licenses

Require derivatives to be shared under same terms:

```
You may use, modify, and distribute this biological material provided
that:

1. You provide source (sequence and documentation) to all recipients
2. You grant recipients the same rights you received
3. You apply this license to all derivatives
4. You clearly mark all modifications

This ensures the material and all derivatives remain open.
```

**Challenges:**
- Defining "derivative" for biological materials is complex
- "Source code" concept doesn't map cleanly to biology
- Enforcement mechanisms unclear
- Less commonly used than permissive licenses

## 5.4 Registry Licensing Policies

Different registries adopt different approaches:

### iGEM Registry Policy

**Default License:** OpenMTA (or compatible terms)

**Submission Agreement:**
```
By submitting parts to the Registry, you agree:

1. Parts will be made available under OpenMTA terms

2. You have authority to submit (proper authorization from institution)

3. You represent that submission does not violate others' rights

4. You grant iGEM Foundation right to distribute parts

5. You acknowledge parts become part of public registry

6. You retain any intellectual property rights you may have

7. Registry may update part information and categorization
```

**Exceptions:**
- Parts may be flagged with known IP restrictions
- Contributors can note if parts are subject to patents
- Users responsible for their own FTO analysis
- Registry provides information, not legal advice

### AddGene Policy

**Model:** Repository with multiple license options

**Available Licenses:**
- OpenMTA (most common)
- Uniform Biological Material Transfer Agreement (UBMTA)
- Custom institutional MTAs
- Non-commercial only restrictions (some materials)

**Depositor Choice:**
Scientists depositing plasmids choose license terms

**User Obligations:**
- Review and agree to specific license for each material
- Institutional approval may be required
- Commercial users may face additional requirements

### Proprietary Registry Models

Some industry-focused registries use restrictive terms:

**Typical Restrictions:**
- Evaluation/research use only
- No commercial applications without separate license
- No redistribution
- No reverse engineering
- Field-of-use restrictions
- Reach-through royalties on products

**Rationale:**
- Protect commercial interests
- Generate revenue
- Control quality and applications
- Enable investment recovery

## 5.5 Patent Pools and Licensing Initiatives

Collective approaches to manage IP complexity:

### Patent Pools

Multiple patent holders contribute patents to a common pool accessible under single license:

**Advantages:**
- One-stop licensing
- Reduced transaction costs
- Lower royalty rates
- Eliminates blocking patents
- Encourages innovation

**Challenges:**
- Requires cooperation among competitors
- Complex to establish and govern
- Antitrust concerns
- Valuation disputes
- Enforcement questions

**Synthetic Biology Examples:**
- Discussions ongoing, no major pool established yet
- Some focused efforts in specific areas (CRISPR, gene therapy)

### CAMBIA BiOS Initiative

Biological Open Source Initiative explored open source approaches:

**Concepts:**
- Protected Commons: Patents held but licensed openly
- Improvement Commons: Sharing of improvements
- Clearinghouse: Central licensing for multiple technologies
- Defensive Publication: Establish prior art to prevent patents

**Outcomes:**
- Raised awareness of open approaches
- Some technologies licensed openly
- Influenced registry development
- Limited broad adoption

### Defensive Patent Strategies

Organizations acquire patents not to exclude but to defend:

**Defensive Patent License (DPL):**
```
Patent holder commits:
1. Grant royalty-free license to all DPL members
2. All members cross-license patents
3. Community protected from patent assertions
4. Can practice each other's inventions freely

If you sue a DPL member for patent infringement:
→ You lose access to all DPL patents

Creates incentive for cooperation over litigation.
```

**Application to Registries:**
- Registry could maintain DPL pool
- Contributors grant defensive licenses
- Community protected from patent trolls
- Maintains freedom to operate

## 5.6 International IP Considerations

Patent laws vary significantly across jurisdictions:

### Key Jurisdictional Differences

**United States:**
- First-to-file (since 2013)
- Grace period (1 year after public disclosure)
- Strict utility requirement
- Subject matter broadly patentable
- Natural sequences generally not patentable (post-Myriad)

**Europe:**
- First-to-file
- No grace period (disclosure destroys patentability)
- Morality/ordre public exclusions
- Plant and animal varieties excluded
- Stricter patentable subject matter requirements

**Japan:**
- First-to-file
- No grace period (limited exceptions)
- Broad patentable subject matter
- Strong examination
- Requires industrial applicability

**China:**
- First-to-file
- Limited grace period (6 months, limited circumstances)
- Inventive step threshold
- Increasing sophistication of examination
- Growing synthetic biology patent activity

**Developing Countries:**
- Variable patent systems
- Often aligned with TRIPS requirements
- Some exclude biological materials
- Access and benefit sharing considerations
- Traditional knowledge protections

### International Treaties and Agreements

**Patent Cooperation Treaty (PCT):**
- Streamlines international filing
- Single application → protection in 150+ countries
- Delays national stage decisions
- Commonly used for biotech patents

**TRIPS (Trade-Related Aspects of IP Rights):**
- Minimum IP standards for WTO members
- Requires patent protection for inventions (with some exclusions)
- Flexibilities for public health, research
- Developing country considerations

**Nagoya Protocol:**
- Access and benefit sharing for genetic resources
- Prior informed consent required
- Mutually agreed terms
- Implications for sourcing natural DNA sequences

### Registry Strategies for International IP

**Global Access Despite IP Complexity:**

**Multi-Jurisdictional Freedom to Operate:**
```json
{
  "part_id": "BBa_J23100",
  "ip_status": {
    "US": "No known patents",
    "EP": "No known patents",
    "JP": "No known patents",
    "CN": "Patent pending review",
    "other": "Not reviewed"
  },
  "last_updated": "2024-01-15"
}
```

**Geographic Restrictions:**
Some parts may be freely available in some jurisdictions but restricted in others.

**Harmonization Efforts:**
Registries can advocate for more consistent international treatment of synthetic biology tools.

## 5.7 Balancing Openness and Commercialization

Registry policies must accommodate both academic and industrial needs:

### Dual Licensing Models

Parts available under different terms for different uses:

**Academic/Non-Profit:**
- OpenMTA or equivalent
- Free access
- Minimal restrictions
- Encourage sharing

**Commercial:**
- Separate license required
- Negotiated terms
- Royalty considerations
- Field-of-use limitations possible

**Implementation:**
```
Part: BBa_X12345

License Terms:
✓ Academic research: OpenMTA (free)
✓ Educational use: OpenMTA (free)
✓ Non-profit applications: OpenMTA (free)
✓ Commercial R&D: Contact depositor
✓ Commercial products: License required

Contact: tech-transfer@university.edu
```

### "Research Exemption" Approaches

Many jurisdictions provide limited protection for research use:

**Typical Exemptions:**
- Non-commercial research permitted
- Experimentation on patented invention
- Does not extend to commercialization
- Scope varies by jurisdiction

**Registry Alignment:**
- Parts available for research under open terms
- Commercial users must conduct FTO analysis
- Registry facilitates contacts for licensing

### Industry Participation Models

Encourage companies to contribute to open registries:

**Incentives for Companies:**
- Reputation and recognition
- Access to community innovations
- Influence on standards
- Recruitment of talent
- Lower cost than proprietary development
- Public benefit (CSR)

**Protections for Companies:**
- Can patent applications/uses while sharing parts
- Can maintain trade secret processes
- Attribution for contributions
- Option to commercialize derivatives

**Examples:**
- Ginkgo Bioworks shares some parts openly
- Zymergen contributed characterized promoters
- Twist Bioscience supports open gene synthesis

## 5.8 Ethical and Policy Considerations

Beyond legal IP issues, registries face ethical questions:

### Access and Equity

**Global South Access:**
- Patent systems may limit access in developing countries
- Registry commitment to worldwide availability
- Licensing terms should not discriminate geographically
- Capacity building for effective use

**Small Organization Access:**
- Small companies and startups need affordable access
- Can't afford extensive FTO analysis and licensing
- Registry provides clearance information
- Simplified licensing for basic tools

**Educational Access:**
- Students and teachers require free, unrestricted access
- Educational use clearly permitted
- Distribution kits for schools
- No institutional barriers

### Biosecurity and Responsible Innovation

**Dual-Use Concerns:**
- Some parts have legitimate and problematic uses
- Screening for dangerous sequences
- Access controls for sensitive materials
- Balance openness with safety

**Codes of Conduct:**
```
Registry Users Agree to:

1. Use parts responsibly and ethically
2. Comply with biosafety regulations
3. Conduct risk assessments
4. Respect export controls
5. Not use for harmful purposes
6. Report safety concerns
7. Participate in self-governance
```

### Benefit Sharing

**Nagoya Protocol Compliance:**
- If parts derived from natural genetic resources
- Access and benefit sharing agreements may apply
- Prior informed consent required
- Monetary and non-monetary benefits

**Recognition of Indigenous Knowledge:**
- Traditional knowledge may inform part design
- Appropriate recognition and compensation
- Community consultation
- Respectful engagement

### Future Policy Development

**Evolving Needs:**
- AI-designed parts (who owns?)
- Synthetic genomes (patentability?)
- Standardization of licensing
- International harmonization
- Platform regulation

**Multi-Stakeholder Governance:**
- Academic researchers
- Industry representatives
- Policy makers
- Ethics committees
- Public interest groups
- International organizations

---

## Key Takeaways

✓ Biological registries balance open science principles with IP realities
✓ OpenMTA provides legal framework for open sharing of biological materials
✓ Patent landscape is complex; parts may be covered by composition, method, or system patents
✓ Freedom to operate analysis essential for commercial applications
✓ Different registries adopt different licensing policies (open, dual-license, proprietary)
✓ International IP considerations add complexity; patent laws vary by jurisdiction
✓ Strategies exist for balancing openness and commercialization (dual licensing, research exemptions)
✓ Ethical considerations include access equity, biosecurity, and benefit sharing
✓ Patent pools, defensive licenses, and collective approaches may address IP complexity
✓ Registry policies continue evolving to meet community needs

---

**Next Chapter:** Chapter 6 examines the global infrastructure supporting biological part registries, including physical repositories, distribution networks, and international collaborations.

---

*WIA-SYNTHETIC-BIO-REGISTRY Standard v1.0*
*© 2025 SmileStory Inc. / WIA*
*弘益人間 (Benefit All Humanity)*
