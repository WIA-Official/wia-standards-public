# WIA-SEMI-001: Semiconductor Design Standard - Technical Specification v1.0

**Status**: Draft
**Version**: 1.0.0
**Date**: 2025-01-15
**Authors**: WIA Standards Committee

---

## 弘益人間 (홍익인간) · Benefit All Humanity

---

## Abstract

This document specifies the WIA-SEMI-001 standard for semiconductor design, providing a comprehensive framework for RTL design, verification, synthesis, and physical implementation. The standard aims to enable tool interoperability, design reuse, and collaboration across the global semiconductor ecosystem.

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Scope](#2-scope)
3. [Normative References](#3-normative-references)
4. [Terms and Definitions](#4-terms-and-definitions)
5. [RTL Design Standards](#5-rtl-design-standards)
6. [Verification Standards](#6-verification-standards)
7. [Synthesis Standards](#7-synthesis-standards)
8. [Physical Design Standards](#8-physical-design-standards)
9. [API Specifications](#9-api-specifications)
10. [Conformance](#10-conformance)

---

## 1. Introduction

### 1.1 Purpose

The purpose of WIA-SEMI-001 is to standardize semiconductor design flows, enabling:
- **Interoperability**: Seamless integration between different EDA tools
- **Portability**: Designs that can be migrated across tool chains
- **Reusability**: IP cores and design components that work across projects
- **Quality**: Consistent design practices leading to better results

### 1.2 Background

Modern semiconductor design involves complex interactions between multiple tools, languages, and methodologies. The lack of standardization leads to:
- Vendor lock-in
- Manual translation between formats
- Inconsistent design quality
- Difficult IP reuse

WIA-SEMI-001 addresses these challenges through open, well-defined standards.

### 1.3 Design Principles

- **Openness**: Specifications are publicly available
- **Vendor Neutrality**: No preference for specific tool vendors
- **Backward Compatibility**: Support for existing industry formats
- **Extensibility**: Ability to add new features without breaking existing implementations

---

## 2. Scope

This standard covers:

### 2.1 In Scope

- RTL coding guidelines (Verilog, VHDL, SystemVerilog)
- Verification methodologies (directed, random, UVM, formal)
- Synthesis constraints and optimization
- Physical design data formats
- API specifications for tool integration
- Design data exchange formats

### 2.2 Out of Scope

- Specific tool implementations
- Process technology details (foundry-specific)
- Analog/mixed-signal design (covered by WIA-AMS-001)
- Software development for embedded processors

---

## 3. Normative References

The following standards are referenced in this specification:

- **IEEE 1364-2005**: Verilog Hardware Description Language
- **IEEE 1076-2019**: VHDL Language Reference Manual
- **IEEE 1800-2017**: SystemVerilog Unified Hardware Design, Specification, and Verification Language
- **IEEE 1801-2018**: UPF (Unified Power Format)
- **Accellera UVM 1.2**: Universal Verification Methodology
- **Liberty NCX**: Liberty Timing Model Format
- **LEF/DEF**: Library Exchange Format / Design Exchange Format
- **GDSII**: Graphic Database System II
- **OASIS**: Open Artwork System Interchange Standard

---

## 4. Terms and Definitions

### 4.1 Acronyms

- **RTL**: Register Transfer Level
- **EDA**: Electronic Design Automation
- **HDL**: Hardware Description Language
- **DUT**: Design Under Test
- **UVM**: Universal Verification Methodology
- **DFT**: Design for Test
- **STA**: Static Timing Analysis
- **P&R**: Place and Route
- **PPA**: Power, Performance, Area
- **IP**: Intellectual Property (design component)
- **SoC**: System on Chip

### 4.2 Definitions

- **Module**: A basic unit of RTL design with defined inputs, outputs, and behavior
- **Netlist**: A description of circuit connectivity at gate level
- **Synthesis**: The process of transforming RTL into gate-level netlist
- **Floorplan**: The arrangement of major blocks and IO on the chip die
- **Sign-off**: Final verification that design meets all requirements

---

## 5. RTL Design Standards

### 5.1 Coding Style

#### 5.1.1 General Guidelines

All RTL code SHALL follow these principles:

1. **Readability**: Code should be self-documenting
2. **Maintainability**: Easy to modify and extend
3. **Synthesizability**: Translatable to hardware
4. **Reusability**: Parameterized and modular

#### 5.1.2 Naming Conventions

```verilog
// Module names: lowercase with underscores
module fifo_sync_ram (...);

// Parameters: UPPERCASE with underscores
parameter DATA_WIDTH = 32;
parameter DEPTH = 16;

// Signals:
//   - Inputs: lowercase with underscores
//   - Outputs: lowercase with underscores
//   - Internal: lowercase with underscores
//   - Active-low: suffix _n
input  wire clk;
input  wire rst_n;
output wire valid;

// Constants: UPPERCASE
localparam IDLE_STATE = 2'b00;
```

#### 5.1.3 File Organization

```
rtl/
├── top.sv              # Top-level module
├── core/
│   ├── cpu.sv
│   ├── alu.sv
│   └── regfile.sv
├── memory/
│   ├── cache.sv
│   └── sram.sv
└── peripherals/
    ├── uart.sv
    └── spi.sv
```

### 5.2 RTL Constructs

#### 5.2.1 Always Blocks

**Combinational Logic**:
```verilog
// Use always_comb (SystemVerilog) or always @(*)
always_comb begin
  sum = a + b;
  carry = (a & b) | (b & cin) | (a & cin);
end
```

**Sequential Logic**:
```verilog
// Use always_ff (SystemVerilog) or always @(posedge clk)
always_ff @(posedge clk or negedge rst_n) begin
  if (!rst_n)
    q <= '0;
  else
    q <= d;
end
```

#### 5.2.2 Reset Strategy

All registers SHALL have explicit reset logic:

```verilog
// Asynchronous reset (preferred for FPGAs)
always_ff @(posedge clk or negedge rst_n) begin
  if (!rst_n)
    state <= IDLE;
  else
    state <= next_state;
end

// Synchronous reset (preferred for ASICs)
always_ff @(posedge clk) begin
  if (!rst_n)
    state <= IDLE;
  else
    state <= next_state;
end
```

#### 5.2.3 Clock Domain Crossing

CDC SHALL use recognized safe techniques:

```systemverilog
// Two-flop synchronizer
always_ff @(posedge clk_dst) begin
  sync_ff1 <= signal_src;
  sync_ff2 <= sync_ff1;
end
assign signal_dst = sync_ff2;

// For control signals only, not buses!
```

### 5.3 IP-XACT Integration

Design modules SHALL provide IP-XACT metadata:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<ipxact:component>
  <ipxact:vendor>wia-official.org</ipxact:vendor>
  <ipxact:library>standard_cells</ipxact:library>
  <ipxact:name>fifo_sync</ipxact:name>
  <ipxact:version>1.0.0</ipxact:version>
  <ipxact:busInterfaces>
    <ipxact:busInterface>
      <ipxact:name>clk</ipxact:name>
      <ipxact:busType>clock</ipxact:busType>
    </ipxact:busInterface>
  </ipxact:busInterfaces>
  <ipxact:model>
    <ipxact:views>
      <ipxact:view>
        <ipxact:name>rtl</ipxact:name>
        <ipxact:fileSetRef>rtlFiles</ipxact:fileSetRef>
      </ipxact:view>
    </ipxact:views>
  </ipxact:model>
</ipxact:component>
```

---

## 6. Verification Standards

### 6.1 Verification Methodology

Designs SHALL be verified using at least one of:

1. **Directed Testing**: Manual test vectors
2. **Constrained-Random**: UVM/SystemVerilog
3. **Formal Verification**: Property checking
4. **Emulation**: FPGA-based verification

### 6.2 UVM Compliance

UVM testbenches SHALL follow UVM 1.2 standard:

```systemverilog
class my_test extends uvm_test;
  `uvm_component_utils(my_test)

  my_env env;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    env = my_env::type_id::create("env", this);
  endfunction

  virtual task run_phase(uvm_phase phase);
    phase.raise_objection(this);
    // Test execution
    phase.drop_objection(this);
  endtask
endclass
```

### 6.3 Coverage Requirements

Minimum coverage targets:

| Coverage Type | Minimum Target |
|---------------|----------------|
| Code Coverage | 95% |
| Functional Coverage | 90% |
| Toggle Coverage | 85% |
| FSM Coverage | 100% |
| Assertion Coverage | 100% |

### 6.4 Assertion-Based Verification

Critical properties SHALL be specified using SVA:

```systemverilog
// Property: Valid must remain stable when ready is low
property valid_stable;
  @(posedge clk) disable iff (!rst_n)
  (!ready && valid) |=> ($stable(valid) && $stable(data));
endproperty

assert_valid_stable: assert property (valid_stable)
  else $error("Valid signal not stable!");
```

---

## 7. Synthesis Standards

### 7.1 Constraint Format

Timing constraints SHALL use SDC format:

```tcl
# Clock definition
create_clock -name clk -period 10.0 [get_ports clk]

# Input/Output delays
set_input_delay -clock clk -max 3.0 [all_inputs]
set_output_delay -clock clk -max 2.0 [all_outputs]

# False paths
set_false_path -from [get_ports rst_n]

# Multi-cycle paths
set_multicycle_path 2 -setup -from [get_pins reg_a/Q] -to [get_pins reg_b/D]
```

### 7.2 Power Intent

Power domains SHALL be specified using UPF:

```tcl
# Create power domains
create_power_domain PD_TOP -include_scope
create_power_domain PD_CORE -elements {u_core}

# Define supply nets
create_supply_net VDD -domain PD_TOP
create_supply_net VSS -domain PD_TOP

# Connect supplies
connect_supply_net VDD -ports VDD
connect_supply_net VSS -ports VSS

# Power states
add_power_state PD_CORE \
  -state ON {-supply_expr {VDD == 1.0}} \
  -state OFF {-supply_expr {VDD == 0.0}}
```

### 7.3 Optimization Goals

Synthesis tools SHALL support optimization for:

- **Area**: Minimize gate count
- **Power**: Minimize dynamic + static power
- **Performance**: Maximize frequency
- **Balanced**: Trade-off between all three

---

## 8. Physical Design Standards

### 8.1 Floorplan Specification

```json
{
  "floorplan": {
    "die": {
      "width": 1000,
      "height": 1000,
      "unit": "um"
    },
    "core": {
      "utilization": 0.70,
      "aspect_ratio": 1.0
    },
    "power_plan": {
      "vdd_stripes": 12,
      "vss_stripes": 12,
      "stripe_width": 2.0,
      "unit": "um"
    },
    "io": {
      "signal_pads": 64,
      "power_pads": 16
    }
  }
}
```

### 8.2 LEF/DEF Standards

Physical libraries SHALL provide LEF files:

```lef
MACRO NAND2X1
  CLASS CORE ;
  FOREIGN NAND2X1 ;
  ORIGIN 0.000 0.000 ;
  SIZE 1.2 BY 2.8 ;
  SYMMETRY X Y ;
  SITE core ;
  PIN A
    DIRECTION INPUT ;
    PORT
      LAYER M1 ;
        RECT 0.1 0.5 0.3 0.7 ;
    END
  END A
  PIN B
    DIRECTION INPUT ;
    PORT
      LAYER M1 ;
        RECT 0.5 0.5 0.7 0.7 ;
    END
  END B
  PIN Y
    DIRECTION OUTPUT ;
    PORT
      LAYER M1 ;
        RECT 0.9 1.3 1.1 1.5 ;
    END
  END Y
END NAND2X1
```

### 8.3 Sign-off Requirements

Before tape-out, designs SHALL pass:

| Check | Tool | Criterion |
|-------|------|-----------|
| Timing | STA | All paths meet timing |
| Power | Power analysis | <5% IR drop |
| DRC | Physical verification | 0 violations |
| LVS | Netlist comparison | 100% match |
| Antenna | Antenna check | 0 violations |
| ERC | Electrical rules | 0 errors |

---

## 9. API Specifications

### 9.1 Design Database API

```typescript
interface DesignDatabase {
  // Design creation
  createDesign(name: string, config: DesignConfig): Design;

  // Module operations
  addModule(design: Design, module: Module): void;
  getModule(design: Design, name: string): Module;

  // Hierarchy navigation
  getTopModule(design: Design): Module;
  getSubmodules(module: Module): Module[];

  // Signal queries
  getSignals(module: Module, filter?: SignalFilter): Signal[];

  // Netlist operations
  exportNetlist(design: Design, format: 'verilog' | 'vhdl'): string;
}
```

### 9.2 Simulation API

```typescript
interface Simulation {
  // Setup
  compile(files: string[], options: CompileOptions): void;
  elaborate(top: string): void;

  // Execution
  run(cycles: number): SimulationResult;
  runUntil(condition: string): SimulationResult;

  // Debug
  setBreakpoint(time: number | string): void;
  dumpWaveform(file: string, signals?: string[]): void;

  // Coverage
  getCoverage(): CoverageReport;
}
```

### 9.3 Synthesis API

```typescript
interface Synthesis {
  // Configuration
  setTechnology(library: string): void;
  setConstraints(sdc: string): void;

  // Optimization
  setGoal(goal: 'area' | 'power' | 'performance' | 'balanced'): void;

  // Execution
  synthesize(): SynthesisResult;

  // Reporting
  getTimingReport(): TimingReport;
  getPowerReport(): PowerReport;
  getAreaReport(): AreaReport;
}
```

---

## 10. Conformance

### 10.1 Conformance Levels

**Level 1 (Basic)**:
- RTL syntax checking
- Basic simulation
- Single-module designs

**Level 2 (Standard)**:
- Multi-module SoC
- UVM verification
- Synthesis to gates

**Level 3 (Advanced)**:
- Complete tape-out flow
- Multi-voltage design
- Advanced DFT

### 10.2 Certification Process

1. Submit design for review
2. Run conformance test suite
3. Review results with WIA committee
4. Receive certification badge

### 10.3 Test Suite

Reference implementations and test cases available at:
```
https://github.com/WIA-Official/wia-semi-001-tests
```

---

## Appendix A: Example Designs

### A.1 Simple Counter

```verilog
module counter #(
  parameter WIDTH = 8
)(
  input  wire             clk,
  input  wire             rst_n,
  input  wire             enable,
  output reg [WIDTH-1:0]  count
);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      count <= {WIDTH{1'b0}};
    else if (enable)
      count <= count + 1'b1;
  end

endmodule
```

### A.2 FIFO

See full example in `examples/fifo_sync.sv`

---

## Appendix B: Change History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01-15 | Initial release |

---

## References

1. IEEE Standards Association, "IEEE Standard for Verilog Hardware Description Language," IEEE Std 1364-2005.
2. Accellera, "Universal Verification Methodology (UVM) 1.2 User's Guide," 2014.
3. Synopsys, "Liberty User Guides and Reference Manual Suite," 2020.

---

<div align="center">

## 弘益人間 (홍익인간)
**Benefit All Humanity**

© 2025 WIA (World Certification Industry Association)

</div>
