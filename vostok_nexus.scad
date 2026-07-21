/*
  VOSTOK NEXUS - Enclosure
  Cleaned and optimized geometry.
*/

$fn = 60;

// --- Case Dimensions ---
w = 120;
d = 80;
h = 40;
wall = 2;
cr = 4; // Outer corner radius

// ==========================================
// MAIN ASSEMBLY
// ==========================================
difference() {

    // 1. BASE GEOMETRY
    // Strict single union containing the box and all internal structures.
    union() {
        outer_box();
        four_corner_columns();
        l_shaped_wall();
        all_standoffs();
    }

    // 2. SUBTRACTIONS
    main_inner_cavity();
    screwholes();
    port_cutouts();
    vent_slots();
    front_buttons();
}

// ==========================================
// GEOMETRY MODULES
// ==========================================

module box_profile(width, depth, r) {
    hull() {
        translate([r, r]) circle(r=r);
        translate([width-r, r]) circle(r=r);
        translate([r, depth-r]) circle(r=r);
        translate([width-r, depth-r]) circle(r=r);
    }
}

// Solid outer block
module outer_box() {
    linear_extrude(h) box_profile(w, d, cr);
}

// 4 Corner structural posts from floor to ceiling
module four_corner_columns() {
    positions = [
        [cr, cr],
        [w-cr, cr],
        [cr, d-cr],
        [w-cr, d-cr]
    ];
    for (p = positions) {
        translate([p[0], p[1], 0]) cylinder(d=7, h=h);
    }
}

// Internal partition for BME280 room
module l_shaped_wall() {
    // Creates a 32x32mm internal chamber at the rear-right corner.

    // Y-axis aligned partition
    translate([w - 32, d - 32, 0]) cube([2, 32, h]);
    // X-axis aligned partition
    translate([w - 32, d - 32, 0]) cube([32, 2, h]);
}

// All standoffs combined
module all_standoffs() {
    // Main board standoffs (generic)
    main_pts = [[20, 20], [70, 20], [20, 60], [70, 60]];
    for (p = main_pts) {
        translate([p[0], p[1], 0]) cylinder(d=6, h=wall+6);
    }

    // BME280 standoffs inside rear-right chamber (OD 7mm, H 6mm above floor)
    bme_pts = [[99, 59], [109, 59], [99, 69], [109, 69]];
    for (p = bme_pts) {
        translate([p[0], p[1], 0]) cylinder(d=7, h=wall+6);
    }
}

// ==========================================
// SUBTRACTION MODULES
// ==========================================

module main_inner_cavity() {
    difference() {
        // Raw hollow space (leaves a solid unperforated floor at Z=2)
        translate([0, 0, wall])
        linear_extrude(h + 1) // +1 to cut cleanly through the open top
        offset(r=-wall) box_profile(w, d, cr);

        // Protect internal structures so they aren't erased by the hollow space!
        four_corner_columns();
        l_shaped_wall();
        all_standoffs();
    }
}

module screwholes() {
    // Blind holes for corner lid screws (starts above the floor)
    positions = [
        [cr, cr],
        [w-cr, cr],
        [cr, d-cr],
        [w-cr, d-cr]
    ];
    for (p = positions) {
        translate([p[0], p[1], wall]) cylinder(d=3, h=h);
    }

    // Blind holes for main board
    main_pts = [[20, 20], [70, 20], [20, 60], [70, 60]];
    for (p = main_pts) {
        translate([p[0], p[1], wall]) cylinder(d=2.5, h=6+1);
    }

    // Blind holes for BME280 (4.2mm ID exactly)
    bme_pts = [[99, 59], [109, 59], [99, 69], [109, 69]];
    for (p = bme_pts) {
        translate([p[0], p[1], wall]) cylinder(d=4.2, h=6+1);
    }
}

module port_cutouts() {
    // Generic side cutouts for USB / Power (left wall)
    translate([-1, 30, 15]) cube([wall+2, 12, 6]);
    translate([-1, 50, 18]) rotate([0, 90, 0]) cylinder(d=8, h=wall+2);
}

module vent_slots() {
    // Teardrop slots on BME280 exterior walls (Rear wall)
    for (x = [95, 103, 111]) {
        translate([x, d - wall - 1, 20])
        rotate([-90, 0, 0]) // Orients the teardrop tip upward
        linear_extrude(wall + 2) {
            hull() {
                circle(d=4);
                polygon([[-2, 0], [2, 0], [0, 3]]);
            }
        }
    }

    // Teardrop slots on BME280 exterior walls (Right wall)
    for (y = [55, 63, 71]) {
        translate([w - wall - 1, y, 20])
        rotate([0, 0, -90]) // Points the extrusion right (+X)
        rotate([-90, 0, 0]) // Orients the teardrop tip upward
        linear_extrude(wall + 2) {
            hull() {
                circle(d=4);
                polygon([[-2, 0], [2, 0], [0, 3]]);
            }
        }
    }
}

module front_buttons() {
    // 16mm circular teardrop holes, cleanly shifted left (X=35 and X=60)
    // to avoid clipping the L-shaped chamber wall.
    for (x = [35, 60]) {
        translate([x, -1, 20])
        rotate([-90, 0, 0]) // Orients the teardrop tip upward
        linear_extrude(wall + 2) {
            hull() {
                circle(d=16);
                polygon([[-8, 0], [8, 0], [0, 10]]);
            }
        }
    }
}
