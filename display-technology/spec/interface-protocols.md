# Display Interface Protocols Specification

**WIA-SEMI-008 Display Technology Standard**
**Version**: 1.0
**Last Updated**: 2025

---

## Overview

This specification defines requirements and guidelines for display interface protocols including HDMI, DisplayPort, MIPI DSI, eDP, LVDS, and V-by-One HS used in consumer electronics, mobile devices, automotive, and professional applications.

---

## 1. HDMI Specifications

### 1.1 HDMI 2.1 Requirements

**Bandwidth:**
- Fixed Rate Link (FRL): 48 Gbps maximum
- Transition Minimized Differential Signaling (TMDS): 18 Gbps (backward compatibility)

**Resolutions and Refresh Rates:**
- 10K @ 60Hz
- 8K @ 60Hz (uncompressed), 120Hz (with DSC)
- 4K @ 120Hz/144Hz
- 1080p @ 240Hz

**Color Depth:**
- 24-bit (8 bpc): Standard
- 30-bit (10 bpc): High color depth
- 36-bit (12 bpc): Deep color
- 48-bit (16 bpc): Maximum

**Features:**
- VRR (Variable Refresh Rate): FreeSync and G-Sync compatible
- ALLM (Auto Low Latency Mode): Automatic game mode
- QMS (Quick Media Switching): Eliminate black screen on source changes
- QFT (Quick Frame Transport): Reduced latency
- eARC (Enhanced Audio Return Channel): Lossless audio
- DSC 1.2a (Display Stream Compression): 3:1 visually lossless

**Cable Requirements:**
- Ultra High Speed HDMI Cable certification mandatory
- Maximum passive cable length: 1-3 meters (48 Gbps)
- Active cables required for longer distances

### 1.2 HDMI 2.0 Requirements

**Bandwidth:** 18 Gbps

**Resolutions:**
- 4K @ 60Hz (24-bit color)
- 4K @ 30Hz (48-bit deep color)
- 1080p @ 120Hz/144Hz

**Features:**
- HDR10, HLG support
- ARC (Audio Return Channel)
- BT.2020 color space signaling

**Cable:** Premium High Speed HDMI

### 1.3 HDCP (High-bandwidth Digital Content Protection)

**Versions:**
- HDCP 2.3: Required for 4K content protection
- HDCP 2.2: Previous 4K standard
- HDCP 1.4: Legacy 1080p

**Requirements:**
- Mandatory for premium video services (Netflix 4K, etc.)
- Authentication between source and sink
- Revocation list checking

---

## 2. DisplayPort Specifications

### 2.1 DisplayPort 2.0 (UHBR)

**Bandwidth:**
- UHBR 10: 10 Gbps/lane, 40 Gbps total (4 lanes)
- UHBR 13.5: 13.5 Gbps/lane, 54 Gbps total
- UHBR 20: 20 Gbps/lane, 80 Gbps total

**Resolutions:**
- 16K @ 60Hz (with DSC)
- 10K @ 60Hz
- 8K @ 120Hz (uncompressed)
- 4K @ 240Hz

**Features:**
- DSC 1.2a support (mandatory for extreme resolutions)
- Panel Replay (enhanced PSR)
- USB-C Alt Mode compatible
- MST (Multi-Stream Transport): Daisy-chain displays

**Cable:**
- DP80 certification for UHBR 20
- Passive: <1 meter for UHBR 20
- Active/optical: Extended lengths

### 2.2 DisplayPort 1.4 (HBR3)

**Bandwidth:**
- HBR3: 8.1 Gbps/lane, 32.4 Gbps total (4 lanes)
- HBR2: 5.4 Gbps/lane, 21.6 Gbps total
- RBR/HBR: Legacy rates

**Resolutions:**
- 8K @ 60Hz (with DSC)
- 4K @ 120Hz
- 5K @ 60Hz
- 1440p @ 240Hz

**Features:**
- DSC 1.2 (Display Stream Compression)
- HDR10 metadata support
- Forward Error Correction (FEC)
- Adaptive Sync (VRR)

### 2.3 DisplayPort Alt Mode (USB-C)

**Configurations:**
- 2-lane DP + USB 3.x data
- 4-lane DP + USB 2.0 data

**Benefits:**
- Single cable for video, data, power
- Reversible connector
- Up to 100W USB Power Delivery

**Requirements:**
- USB-C receptacle with DP Alt Mode support
- Proper cable (USB-C to USB-C or USB-C to DP)
- Mux/demux hardware

---

## 3. MIPI DSI Specifications

### 3.1 MIPI DSI-2 Requirements

**Physical Layer:**
- D-PHY: 2.5 Gbps/lane maximum
- C-PHY: 2.28 bits/symbol, lower power

**Lanes:**
- 1-4 data lanes + 1 clock lane (D-PHY)
- 1-3 trios (C-PHY)

**Operating Modes:**
- High-Speed (HS): 80 Mbps - 2.5 Gbps/lane
- Low-Power (LP): <10 Mbps for commands
- Command Mode: Bidirectional
- Video Mode: Unidirectional streaming

**Bandwidth Example:**
- 4-lane D-PHY @ 2.5 Gbps = 10 Gbps effective
- Supports: 2400×1080 @ 120Hz (with compression)

### 3.2 Display Command Set (DCS)

**Commands:**
- Enter/exit sleep mode
- Set display on/off
- Set brightness
- Set pixel format
- Read display ID
- Panel-specific extensions

**Implementation:**
- Standardized command set
- Vendor-specific extensions allowed
- Bidirectional communication

### 3.3 MIPI DSI Test Specifications

**Electrical:**
- Eye diagram measurements
- Jitter tolerance: < 0.1 UI
- Rise/fall times: < 200ps

**Protocol:**
- CRC error rate: < 10⁻⁹
- ECC single-bit correction
- Packet format compliance

---

## 4. eDP (Embedded DisplayPort) Specifications

### 4.1 eDP 1.4b Requirements

**Bandwidth:**
- HBR3: 8.1 Gbps/lane (32.4 Gbps total)
- Backward compatible: HBR2, HBR, RBR

**Lanes:**
- 1, 2, or 4 lanes
- Configurable based on panel requirements

**Resolutions:**
- 4K @ 60Hz (no compression)
- 4K @ 120Hz (with DSC)
- 5K @ 60Hz

**Features:**
- PSR (Panel Self-Refresh): GPU sleeps while panel refreshes from buffer
- PSR2: Selective Update (only changed regions)
- Adaptive Sync (VRR)
- DRRS (Dynamic Refresh Rate Switching)

### 4.2 Panel Self-Refresh (PSR)

**PSR1:**
- Panel stores full frame
- GPU can sleep
- Wake on content change
- 30-50% power savings (static content)

**PSR2 (Selective Update):**
- Only changed regions updated
- Cursor, partial screen updates
- Further power reduction vs PSR1

**Requirements:**
- Panel-side frame buffer (SRAM)
- Communication protocol for updates
- Fast wake-up from sleep

### 4.3 eDP Testing

**Link Training:**
- Automatic rate negotiation
- Channel equalization
- Voltage swing and pre-emphasis tuning

**Power Measurements:**
- PSR active vs inactive comparison
- Transition latency measurement

**Compliance:**
- VESA eDP CTS (Compliance Test Specification)
- Interoperability testing

---

## 5. LVDS (Low-Voltage Differential Signaling)

### 5.1 LVDS Requirements

**Electrical:**
- Differential voltage: 350 mV typical
- Common-mode voltage: 1.25V
- Current: 3.5 mA per driver

**Configurations:**
- Single LVDS: 1 pixel/clock (~7 Gbps)
- Dual LVDS: 2 pixels/clock (~14 Gbps)
- Quad LVDS: 4 pixels/clock (~28 Gbps)

**Data Lanes:**
- 3-5 data pairs + 1 clock pair (single)
- 6-10 data pairs + 1 clock pair (dual)

**Pixel Formats:**
- 18-bit RGB (6-6-6)
- 24-bit RGB (8-8-8)

### 5.2 LVDS Applications

**Current Use:**
- Industrial displays
- Automotive legacy systems
- Cost-sensitive applications
- Simple embedded systems

**Advantages:**
- Proven, mature technology
- Low EMI
- Robust signaling

**Disadvantages:**
- Many wires (thick cables)
- Limited bandwidth vs modern interfaces
- No advanced features (VRR, compression, etc.)

### 5.3 Transition to eDP

**Migration Path:**
- New designs: Use eDP instead of LVDS
- Existing designs: LVDS-to-eDP bridge ICs available
- Cost: eDP becoming cost-competitive

---

## 6. V-by-One HS Specifications

### 6.1 V-by-One HS Requirements

**Purpose:**
High-speed serial interface for TV internal connections (board-to-panel).

**Architecture:**
- Serial differential pairs
- Embedded clock (no separate clock lane)
- 3.75 Gbps per lane

**Configurations:**
- 4K panels: 8 lanes (30 Gbps)
- 8K panels: 16 lanes (60 Gbps)

**Advantages:**
- Fewer wires vs LVDS (critical for thin TVs)
- High bandwidth for 4K/8K
- Robust signaling

### 6.2 V-by-One HS Applications

**Primary Use:**
- Large LCD TV (internal connection)
- Monitor panels (internal)
- Digital signage

**Competitors:**
- LVDS (legacy, more wires)
- DisplayPort (external, not typical for internal)

---

## 7. Bandwidth Calculation Guidelines

### 7.1 Pixel Clock Calculation

**Formula:**
```
Pixel Clock (Hz) = Horizontal Total × Vertical Total × Refresh Rate
```

**Example: 4K @ 60Hz**
- Active: 3840 × 2160
- With blanking: ~4400 × 2250 (CVT-RB timing)
- Pixel Clock: 4400 × 2250 × 60 = 594 MHz

### 7.2 Data Rate Calculation

**Uncompressed:**
```
Data Rate (bps) = Pixel Clock × Bits Per Pixel
```

**Example: 4K @ 60Hz, 24-bit**
- 594 MHz × 24 bits = 14.256 Gbps (raw pixel data)

**With Encoding Overhead:**
- 8b/10b encoding: 14.256 × 1.25 = 17.82 Gbps
- Fits within HDMI 2.0 (18 Gbps)

### 7.3 Compression (DSC)

**Display Stream Compression (DSC):**
- 3:1 compression ratio (typical)
- Visually lossless
- Enables higher resolutions/refresh rates

**Example: 4K @ 144Hz**
- Uncompressed: ~43 Gbps
- With DSC 3:1: ~14.3 Gbps (fits in DP 1.4 HBR3)

---

## 8. Interface Selection Guidelines

### 8.1 Application-Specific Recommendations

**Mobile Devices:**
- Primary: MIPI DSI (D-PHY or C-PHY)
- Lanes: 4-lane for premium, 2-lane for mid-range
- Features: Command mode, low-power modes

**Laptops:**
- Primary: eDP (embedded)
- External: USB-C with DP Alt Mode
- Features: PSR, Adaptive Sync

**Desktop Monitors:**
- Primary: DisplayPort 1.4/2.0
- Alternative: HDMI 2.1
- Features: VRR, high refresh, HDR

**TVs:**
- Consumer input: HDMI 2.1
- Internal (board-to-panel): V-by-One HS or LVDS

**Automotive:**
- Dashboard: LVDS or MIPI DSI
- Infotainment: LVDS, eDP, or HDMI
- Requirements: Automotive-grade compliance

### 8.2 Performance Requirements

**Resolution and Refresh Rate:**
- 1080p @ 60Hz: HDMI 1.4, DP 1.2, LVDS
- 4K @ 60Hz: HDMI 2.0, DP 1.2
- 4K @ 120Hz: HDMI 2.1, DP 1.4
- 8K @ 60Hz: HDMI 2.1, DP 2.0

**Special Features:**
- VRR/Adaptive Sync: HDMI 2.1 VRR, DP Adaptive Sync
- HDR: HDMI 2.0+, DP 1.4+
- Low latency: HDMI 2.1 ALLM, gaming-optimized DP

---

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025 | Initial release |

---

**Copyright**: © 2025 SmileStory Inc. / WIA
**License**: 弘益人間 (Benefit All Humanity)
