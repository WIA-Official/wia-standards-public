/**
 * WIA EMG 2-Channel Enclosure
 * OpenSCAD Design File
 *
 * Version: 1.0.0
 * Author: WIA Standards Committee
 *
 * Designed for:
 * - ESP32-S3-DevKitC board
 * - Custom EMG amplifier PCB (50x30mm)
 * - LiPo battery (503035 - 30x35x5mm)
 *
 * Print Settings:
 * - Material: PLA or PETG
 * - Layer Height: 0.2mm
 * - Infill: 20%
 * - Supports: Yes (for USB port)
 */

// ============================================================================
// Parameters
// ============================================================================

// Enclosure dimensions
case_length = 70;      // mm
case_width = 45;       // mm
case_height = 25;      // mm
wall_thickness = 2;    // mm
corner_radius = 3;     // mm

// PCB dimensions
pcb_length = 50;       // mm
pcb_width = 30;        // mm
pcb_thickness = 1.6;   // mm
pcb_standoff = 3;      // mm

// ESP32 DevKit dimensions
esp32_length = 44;     // mm
esp32_width = 26;      // mm

// Battery dimensions
battery_length = 35;   // mm
battery_width = 30;    // mm
battery_height = 5;    // mm

// Connector holes
usb_c_width = 9;       // mm
usb_c_height = 3.5;    // mm
audio_jack_dia = 6;    // mm

// Button hole
button_dia = 4;        // mm

// LED holes
led_dia = 3;           // mm

// Strap slots
strap_width = 25;      // mm
strap_height = 3;      // mm

// ============================================================================
// Modules
// ============================================================================

// Rounded box
module rounded_box(l, w, h, r) {
    hull() {
        translate([r, r, 0])
            cylinder(h=h, r=r, $fn=30);
        translate([l-r, r, 0])
            cylinder(h=h, r=r, $fn=30);
        translate([r, w-r, 0])
            cylinder(h=h, r=r, $fn=30);
        translate([l-r, w-r, 0])
            cylinder(h=h, r=r, $fn=30);
    }
}

// PCB standoff
module standoff(h, d_outer, d_inner) {
    difference() {
        cylinder(h=h, d=d_outer, $fn=20);
        cylinder(h=h+1, d=d_inner, $fn=20);
    }
}

// Strap slot
module strap_slot(w, h, depth) {
    cube([w, depth, h]);
}

// ============================================================================
// Bottom Case
// ============================================================================

module case_bottom() {
    difference() {
        // Outer shell
        rounded_box(case_length, case_width, case_height - 5, corner_radius);

        // Inner cavity
        translate([wall_thickness, wall_thickness, wall_thickness])
            rounded_box(
                case_length - 2*wall_thickness,
                case_width - 2*wall_thickness,
                case_height,
                corner_radius - wall_thickness/2
            );

        // USB-C port hole
        translate([case_length - wall_thickness - 1, (case_width - usb_c_width)/2, wall_thickness + 3])
            cube([wall_thickness + 2, usb_c_width, usb_c_height]);

        // Audio jack holes (2x for electrode cables)
        translate([-1, case_width/3, wall_thickness + 6])
            rotate([0, 90, 0])
            cylinder(h=wall_thickness + 2, d=audio_jack_dia, $fn=20);

        translate([-1, 2*case_width/3, wall_thickness + 6])
            rotate([0, 90, 0])
            cylinder(h=wall_thickness + 2, d=audio_jack_dia, $fn=20);

        // Strap slots (both sides)
        translate([(case_length - strap_width)/2, -1, wall_thickness])
            strap_slot(strap_width, strap_height, wall_thickness + 2);

        translate([(case_length - strap_width)/2, case_width - wall_thickness - 1, wall_thickness])
            strap_slot(strap_width, strap_height, wall_thickness + 2);
    }

    // PCB standoffs
    standoff_positions = [
        [wall_thickness + 5, wall_thickness + 5],
        [wall_thickness + 5, case_width - wall_thickness - 5],
        [case_length - wall_thickness - 15, wall_thickness + 5],
        [case_length - wall_thickness - 15, case_width - wall_thickness - 5]
    ];

    for (pos = standoff_positions) {
        translate([pos[0], pos[1], wall_thickness])
            standoff(pcb_standoff, 5, 2.2);
    }
}

// ============================================================================
// Top Case (Lid)
// ============================================================================

module case_top() {
    lid_height = 8;
    lip_height = 3;
    lip_thickness = 1.5;

    difference() {
        union() {
            // Main lid
            rounded_box(case_length, case_width, lid_height, corner_radius);

            // Inner lip for snap fit
            translate([wall_thickness + 0.3, wall_thickness + 0.3, -lip_height])
                rounded_box(
                    case_length - 2*wall_thickness - 0.6,
                    case_width - 2*wall_thickness - 0.6,
                    lip_height,
                    corner_radius - wall_thickness/2
                );
        }

        // Hollow out the lip
        translate([wall_thickness + lip_thickness + 0.3, wall_thickness + lip_thickness + 0.3, -lip_height - 1])
            rounded_box(
                case_length - 2*wall_thickness - 2*lip_thickness - 0.6,
                case_width - 2*wall_thickness - 2*lip_thickness - 0.6,
                lip_height + lid_height,
                corner_radius - wall_thickness/2 - lip_thickness/2
            );

        // Button hole
        translate([case_length - 15, case_width/2, -1])
            cylinder(h=lid_height + 2, d=button_dia, $fn=20);

        // LED holes
        translate([case_length - 25, case_width/3, -1])
            cylinder(h=lid_height + 2, d=led_dia, $fn=20);

        translate([case_length - 25, 2*case_width/3, -1])
            cylinder(h=lid_height + 2, d=led_dia, $fn=20);

        // Ventilation slots
        for (i = [0:4]) {
            translate([15 + i*8, case_width/2 - 10, -1])
                cube([4, 20, lid_height + 2]);
        }

        // Label area (recessed)
        translate([10, 8, lid_height - 0.5])
            cube([30, 10, 1]);
    }
}

// ============================================================================
// Battery Holder
// ============================================================================

module battery_holder() {
    holder_wall = 1.5;

    difference() {
        cube([
            battery_length + 2*holder_wall,
            battery_width + 2*holder_wall,
            battery_height + 2
        ]);

        translate([holder_wall, holder_wall, 2])
            cube([battery_length, battery_width, battery_height + 1]);

        // Wire channel
        translate([battery_length/2 + holder_wall - 3, -1, 2])
            cube([6, holder_wall + 2, 3]);
    }
}

// ============================================================================
// Render Options
// ============================================================================

// Uncomment to render individual parts:

// Bottom case
case_bottom();

// Top case (translated for viewing)
translate([case_length + 10, 0, 0])
    case_top();

// Battery holder (translated for viewing)
translate([0, case_width + 10, 0])
    battery_holder();

// Assembly view (uncomment to see assembled)
/*
color("gray", 0.8)
    case_bottom();

color("gray", 0.6)
    translate([0, 0, case_height - 5 + 3])
    rotate([180, 0, 0])
    translate([0, -case_width, 0])
        case_top();
*/

// ============================================================================
// Export Notes
// ============================================================================
/*
To export STL files:
1. Comment out all but one module
2. Render (F6)
3. Export as STL (F7)

Files to export:
- wia_emg_case_bottom.stl
- wia_emg_case_top.stl
- wia_emg_battery_holder.stl (optional)

Print orientation:
- Bottom: Print as-is (flat bottom down)
- Top: Print upside down (flat top on bed)
- Battery holder: Print as-is
*/
