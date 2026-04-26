# WIA-metamaterial PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-metamaterial
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for WIA-metamaterial.
Metamaterials are engineered composites whose sub-wavelength geometry
produces electromagnetic, acoustic, or elastic responses that are not
available from their constituent bulk materials. The format captures the
unit-cell geometry, the simulation conditions under which a unit cell is
characterised, the S-parameter and dispersion data that the simulation
produces, the effective constitutive parameters retrieved from those data,
the fabrication tolerances under which the unit cell is realised, and the
measurement evidence that correlates the realised structure with the
simulated prediction.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO 10110 (optics and photonics — preparation of drawings; relevant to
  unit-cell drawing exchange when the metamaterial operates at optical
  wavelengths)
- ISO/IEC 11578 (UUID)
- IETF RFC 4122 (UUID URN namespace)
- IETF RFC 8259 (JSON Data Interchange Format)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- W3C XML Schema Definition 1.1 (legacy CAD interchange envelope only)
- IEEE Std 1597.1 (terminology and validation for computational
  electromagnetics; cited only normatively for definitions)

---

## §1 Scope

This PHASE document defines the persistent shapes for unit-cell descriptors
and their associated simulation, retrieval, and measurement records. It is
intended for use by:

- Metamaterial design teams that exchange unit-cell descriptions between
  electromagnetic, acoustic, and mechanical solvers.
- Fabrication facilities (lithography houses, additive-manufacturing
  vendors) that consume design files and emit measurement reports.
- Reference laboratories that characterise realised metamaterial samples
  and certify the correlation between simulation and measurement.
- Publication and citation tools that resolve a published metamaterial
  result to its underlying simulation and measurement evidence.

Bulk-material constitutive databases, polymer-blend recipes, and
process-control records that already have community-accepted exchange
formats are referenced rather than re-defined here.

## §2 Design Identifier

Every metamaterial design carries a stable identifier. The identifier is a
UUIDv7 (RFC 4122 / ISO/IEC 11578) so that records sort by acquisition order
in lexicographic storage.

```
designId          : string (uuidv7)
designCreatedAt   : string (ISO 8601 / RFC 3339, UTC, second precision)
designAuthor      : string (institutional author identifier)
designDomain      : enum  ("electromagnetic" | "acoustic" | "elastic" |
                      "thermal" | "mechanical")
designOperatingBand:
  centerHz        : number (centre frequency, Hertz)
  fractionalBW    : number (fractional bandwidth, dimensionless)
designStatus      : enum ("draft" | "characterised" | "fabricated" |
                      "verified" | "deprecated")
```

The `designDomain` selects which subset of the format applies. A design that
operates simultaneously in two domains (a phononic-photonic dual-band cell,
for example) emits one design record per domain that share the same
`designId` prefix and differ in their `designDomain` field.

## §3 Unit-Cell Geometry

The unit-cell geometry is the primary record that downstream solvers
consume. It carries the periodic lattice description, the sub-cell
inclusions, and the boundary specification.

```
unitCell:
  designId        : string (uuidv7)
  lattice:
    type          : enum ("cubic" | "tetragonal" | "hexagonal" |
                      "rectangular" | "triclinic" | "honeycomb" | "kagome")
    constantsM    : array of number (lattice constants in metres; exactly
                      three entries for 3-D, exactly two for 2-D, one for 1-D)
    angleRad      : array of number (lattice angles, radians; omitted for
                      cubic and rectangular families)
  inclusions      : array of Inclusion
  boundary:
    domainSize    : array of number (domain extents in metres)
    pbcAxes       : array of integer (axes on which periodic boundary
                      conditions apply, 0/1/2)
    portFaces     : array of PortFace (faces that act as scattering ports)

Inclusion:
  inclusionId     : string (uuidv7)
  geometryRef     : string (URI of the CAD file or parametric script)
  geometryFormat  : enum ("step" | "iges" | "stl" | "openscad" | "freecad" |
                      "parametric-json")
  material:
    materialKey   : string (key into the bulk-material database, see §6)
    constitutive  : enum ("dielectric" | "magnetic" | "conductor" |
                      "kerr-nonlinear" | "phase-change" | "elastic" |
                      "viscoelastic")
  placement:
    rotationDeg   : array of number (Euler angles, degrees)
    translationM  : array of number (metres)
    scale         : number (dimensionless multiplier on the geometry)

PortFace:
  faceId          : string (e.g. "x+", "x-", "y+", "y-")
  modeOrders      : array of integer (mode indices to capture at this port)
  referencePlane  : number (offset from the geometric face, metres)
```

Implementations MUST validate that the inclusion geometries fit within
`domainSize` after rotation, translation, and scaling; out-of-domain
geometries are rejected with a Problem Details (RFC 9457) response of
type `urn:wia:metamaterial:geometry-out-of-domain`.

## §4 Simulation Record

Every unit-cell geometry is paired with at least one simulation record that
captures the conditions under which the cell was characterised. The format
is solver-agnostic so that identical conditions can be reproduced across
finite-element, finite-difference, and method-of-moments solvers.

```
simulation:
  simulationId    : string (uuidv7)
  designId        : string (uuidv7)
  unitCellId      : string (uuidv7, references §3)
  solverFamily    : enum ("finite-element" | "fdtd" | "method-of-moments" |
                      "spectral-element" | "transfer-matrix")
  meshing:
    elementOrder  : integer (polynomial order for finite-element solvers)
    elementsPerLambda : number (target spatial resolution in elements per
                      operating wavelength)
    refinementZones : array of RefinementZone
  excitation:
    portId        : string (references PortFace.faceId)
    waveform      : enum ("plane-wave" | "modal" | "gaussian-pulse")
    polarization  : enum ("te" | "tm" | "linear" | "circular-rh" |
                      "circular-lh")
    incidenceDeg  : array of number (theta, phi in degrees)
    bandwidthHz   : number (excitation bandwidth)
  outputs:
    sParameters   : boolean
    nearField     : boolean
    farField      : boolean
    dispersion    : boolean
  convergence:
    sParamDeltaDb : number (convergence threshold on S-parameter magnitude)
    energyResidual: number (relative residual on stored energy)
    maxIterations : integer
```

## §5 S-Parameter and Dispersion Records

Simulation outputs are recorded as discrete frequency- (or wavenumber-)
sampled spectra. The on-disk shape is independent of the solver that
produced it.

```
sParameters:
  simulationId    : string
  ports           : array of string (port IDs in canonical order)
  samples         : array of FrequencySample

FrequencySample:
  frequencyHz     : number
  scatteringMatrix: array of array of complex (NxN, complex represented as
                      [real, imag] pairs)

dispersion:
  simulationId    : string
  brillouinZone   : string (e.g. "Gamma-X-M-Gamma")
  bands           : array of Band

Band:
  bandIndex       : integer
  kPoints         : array of array of number (k-vectors in 1/metre)
  frequenciesHz   : array of number (one frequency per k-point)
```

## §6 Bulk-Material Reference

Constituent materials are referenced by key into a versioned material
database shared by the design, simulation, and measurement records. The
database itself is governed by community-maintained sources;
implementations MUST record both the database identifier and the database
version, so that re-running a simulation against the same record produces
the same constitutive data.

```
materialReference:
  materialKey     : string
  databaseId      : string (e.g. "wia-meta-mat-db-2026q2")
  databaseVersion : string (Semantic Versioning 2.0.0)
  modelType       : enum ("constant" | "drude" | "lorentz" | "drude-lorentz" |
                      "tabulated" | "custom")
  parameters      : object (model-specific; tabulated models reference an
                      external tabulation file by content-address)
```

## §7 Effective-Parameter Retrieval

Retrieved effective parameters (effective permittivity, permeability,
refractive index, impedance, density, bulk modulus, depending on the
domain) accompany the S-parameter spectra. The retrieval method is
recorded so that consumers can reproduce or revise the inversion.

```
retrieval:
  simulationId    : string
  method          : enum ("nicolson-ross-weir" | "kramers-kronig-causality" |
                      "homogenisation-asymptotic" | "field-averaging" |
                      "transfer-matrix")
  branchSelection : string (description of how branch ambiguity in
                      logarithm-based inversions is resolved)
  retrievedSamples: array of RetrievedSample
```

## §8 Fabrication-Tolerance Record

Realisation of a unit-cell geometry is bounded by fabrication tolerances:
linewidth bias, sidewall angle, layer-thickness variation, alignment
error, and surface roughness. Tolerance budgets are recorded against the
unit cell so that downstream consumers can correlate measured deviations
with predicted performance shifts.

```
toleranceBudget:
  designId        : string
  fabrication:
    process       : enum ("ebeam-lithography" | "stepper-photolithography" |
                      "two-photon" | "selective-laser-sintering" |
                      "stereolithography" | "cnc-milling" | "etching")
    expectedSigmaM: number (1-sigma feature deviation, metres)
  sensitivities   : array of Sensitivity
```

## §9 Measurement Record

Realised samples are characterised in a reference laboratory and the
measurement record is appended to the design.

```
measurement:
  measurementId   : string (uuidv7)
  designId        : string (uuidv7)
  laboratoryId    : string (ISO/IEC 17025-accredited laboratory ID)
  instrument      : string (instrument register entry, see §10)
  samplesUnderTest: integer (number of physical samples measured)
  rawDataRef      : string (content-addressed URI of the raw data archive)
  reducedSamples  : array of FrequencySample (in the same shape as §5)
  uncertainty     : Uncertainty (type-A and type-B contributions)
```

## §10 Instrument Register

Each measurement instrument carries a stable identifier so that operational
drift can be traced to the specific instrument. The register lists vector
network analysers, optical frequency-domain reflectometers, ultrasonic
phased arrays, dynamic mechanical analysers, and similar instrumentation
appropriate to the design domain.

## §11 Field-Distribution Records

For designs whose downstream applications depend on field distribution
(near-field cloaking, sub-wavelength imaging, sensor integration), the
simulation record MAY carry a field-distribution payload alongside the
S-parameter spectra. The payload describes the spatial sampling grid, the
field components captured, and the storage encoding.

```
fieldDistribution:
  simulationId    : string
  gridType        : enum ("uniform-cartesian" | "uniform-cylindrical" |
                      "uniform-spherical" | "adaptive-tetrahedral")
  gridSpacingM    : array of number (per-axis spacing in metres for
                      uniform grids; absent for adaptive grids)
  components      : array of enum ("Ex" | "Ey" | "Ez" | "Hx" | "Hy" |
                      "Hz" | "ux" | "uy" | "uz" | "p" | "T")
  encoding        : enum ("hdf5" | "netcdf" | "vti" | "binary-flat")
  artefactRef     : string (content-addressed URI of the field archive)
  samplingPlanes  : array of SamplingPlane (planar slices of interest)
  samplingPoints  : array of SamplingPoint (specific probe coordinates)
```

The artefact itself is held in evidence storage (PHASE-4 §3) and is
referenced from the simulation record by content-address. Implementations
MUST NOT inline raw field arrays in the JSON record; the JSON carries
metadata only.

## §12 Conformance

Implementations claiming PHASE-1 conformance MUST emit each of the records
defined above for every characterised design and MUST honour the
content-addressing rules in §6, §9, and §11. Conformance is independent
across PHASE documents, but Deep certification under WIA-metamaterial
requires attestations against all four PHASEs.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-metamaterial
- **Last Updated:** 2026-04-27
