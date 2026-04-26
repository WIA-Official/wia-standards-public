# WIA-CITY-008: 3D Printing Construction — Glossary

**Version:** 1.0
**Standard ID:** WIA-CITY-008
**Last Updated:** 2026-04-27

This glossary defines terms used across PHASE-1 to PHASE-4 of the WIA-CITY-008
standard. Terms are grouped alphabetically. Definitions are non-normative; they
clarify usage but do not override the normative text in PHASE documents.

---

## A

### Additive Manufacturing (AM)
A family of fabrication processes in which a part is built by depositing material
layer by layer from a digital model. Construction 3D printing is the
building-scale subset of AM that targets walls, partitions, and structural
elements rather than parts or assemblies.

### Anchor Point
A geometric reference embedded in the printed structure used to align subsequent
layers, attach reinforcement, or mount finishing elements. PHASE-1 specifies how
anchor points are encoded in the building model so downstream tools can find
them deterministically.

### API Versioning
The discipline of evolving the WIA-CITY-008 API without breaking existing
clients. PHASE-2 adopts URL-path versioning (`/wia/v1/`) as the primary scheme
and reserves header-based versioning for negotiated integrations.

### Audit Trail
The chronological record of system events that allows reconstruction of who did
what, when, and to which resource. PHASE-2 mandates audit logging for project
lifecycle events; PHASE-4 elaborates on retention windows and integrity
controls.

---

## B

### Bead
A continuous strand of extruded material laid down by the printer head. Bead
geometry (width, height, overlap) determines layer adhesion and surface finish.
Bead parameters are part of the print job specification in PHASE-2.

### Build Envelope
The maximum three-dimensional volume in which a printer can deposit material.
Gantry systems typically support large rectangular envelopes; robotic-arm and
delta systems trade volume for flexibility.

### Building Information Model (BIM)
A digital representation of a building's physical and functional characteristics.
WIA-CITY-008 is BIM-aware: PHASE-1 schemas can ingest IFC and glTF
representations and emit construction-ready instructions.

---

## C

### Cementitious Material
Any binder material that hardens through hydration or chemical activation
(Portland cement, geopolymer, calcium-aluminate). PHASE-1 catalogues standard
mixes; PHASE-3 covers protocol-level coordination of mixing and pumping
equipment.

### Compliance Tier
A graded conformance level (Basic / Full / Advanced) that lets implementations
declare which subset of normative requirements they meet. Each PHASE document
ends with a Compliance Checklist mapping requirements to tiers.

### Cure Window
The interval after deposition during which a layer must remain undisturbed for
hydration or polymerization to proceed correctly. PHASE-3 protocol messages
include cure-window metadata so robots and supervisory systems can sequence
operations safely.

---

## D

### Deposition Rate
The mass of material extruded per unit time, typically expressed in kg/h or
litre/h. PHASE-1 records the rated and measured deposition rates; PHASE-3
streams the live value as part of the sensor channel.

### Digital Thread
The end-to-end information chain linking design intent (BIM/IFC), production
control (G-code, robot programs), and as-built records. WIA-CITY-008 is
designed so each PHASE contributes a defined link in this thread.

---

## E

### Emergency Stop (E-Stop)
A safety control that halts all motion and material flow within a bounded
deadline. PHASE-3 specifies an end-to-end deadline target; the exact figure is
deployment-specific and SHALL be documented in the safety case.

### Extrusion
The continuous deposition of material through a nozzle or print head. Construction
extrusion is generally larger-scale and slower than desktop extrusion, with
different rheological constraints.

---

## G

### Geopolymer
An inorganic binder formed by alkali activation of aluminosilicate precursors
(fly ash, slag, metakaolin). Geopolymers offer lower embodied carbon than
Portland cement and are explicitly supported in PHASE-1 material schemas.

### G-code
A widely used, line-oriented control language for additive manufacturing.
WIA-CITY-008 does not invent a new dialect; it standardises the metadata
wrapper around vendor-flavoured G-code so jobs can be exchanged between
systems.

---

## I

### Interlayer Bond
The mechanical adhesion between successive printed layers. ASTM C1583 (pull-off
adhesion) and analogous methods provide measurement procedures. PHASE-4
recommends sampling regimes for interlayer bond verification.

### Interoperability
The ability of independent systems (printers, robots, supervisory software,
inspection tools) to exchange information and act on it without bespoke
adapters. WIA-CITY-008 prioritises interoperability through PHASE-2 and PHASE-3.

---

## L

### Layer Height
The thickness of a single printed layer, typically 10–30 mm for construction
3D printing. Layer height influences surface finish, mechanical performance,
and total print time.

### Layer Time
The elapsed time between completing one layer and beginning the next on the
same wall segment. Layer time interacts with cure window: too short can wash
out layers; too long can compromise interlayer bond.

---

## M

### Material Lot
A traceable batch of binder, aggregate, or admixture associated with a
deterministic identifier. PHASE-2 requires material lot identifiers in
project records so PHASE-4 audits can reconstruct material provenance.

---

## N

### Nozzle
The terminal element of the print head through which material exits. Nozzle
diameter, orientation, and condition all influence bead geometry and surface
finish.

---

## P

### Print Path
The two- or three-dimensional trajectory along which material is deposited.
Print paths are derived from a sliced model and may be optimised for time,
strength, or surface finish.

### Pumpability
The ease with which a fresh mix can be transported through hoses to the print
head. Pumpability depends on rheology, particle size, and admixture chemistry.

---

## Q

### Quality Tier
A normative grade attached to a finished structure (e.g., Provisional,
Validated, Audited). PHASE-4 governs the procedure by which a structure is
graded and how the grade is recorded.

---

## R

### Reinforcement
Steel, fibre, or alternative material added to a printed structure to carry
tensile loads. Reinforcement strategies (post-tensioning, embedded rebar,
fibre-reinforced mixes) are catalogued in PHASE-1.

### Rheology
The science of flow and deformation. For printable mixes, rheology is the
discipline that explains why a mix can be pumped, deposited, and stand up
without slumping.

---

## S

### Slump
The vertical sag of fresh material under its own weight. Construction 3D
printing requires controlled slump: too low and the bead will not bond; too
high and the wall will collapse.

### Slicing
The process of decomposing a three-dimensional model into a stack of printable
layers with associated tool paths. Slicing parameters live in PHASE-1 and are
referenced by PHASE-2 print jobs.

---

## T

### Test Vector
A reproducible input/output pair used to verify conformance to a normative
requirement. Each PHASE document references vectors stored under
`tests/phase-vectors/` so audits can mechanise verification.

---

## V

### Veracity Gate
The CITATION-POLICY check that prevents fabricated academic citations,
synthetic clinical claims, and unverifiable institutional attributions from
appearing in normative text. WIA-CITY-008 documents are validated against the
gate at every release.

---

## W

### Wall Segment
A contiguous span of printed material conforming to a single design intent
(e.g., an exterior load-bearing wall on the north elevation). Wall segments
are the primary unit of progress reporting in PHASE-3.

---

## See Also

- PHASE-1-DATA-FORMAT — formal schemas and field definitions
- PHASE-2-API-INTERFACE — REST endpoints, error codes, authentication
- PHASE-3-PROTOCOL — real-time control and telemetry protocols
- PHASE-4-INTEGRATION — deployment, audit, and lifecycle management
- WIA-CITY-008-v1.0.md — master specification document

---

**License:** CC BY 4.0
**弘益人間 (Benefit All Humanity)**
