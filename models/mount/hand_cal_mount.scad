// total width of the mount [mm]
w = 85;

// depth of the rails [mm]
dr = 5;
// height of the rails [mm]
hr = 0.8;
// offset of the rails [mm]
or = 1.1;

// depth of the connector [mm]
dc = 19.8;
// height of the connector [mm]
hc = 8.5;

// height of the mount [mm]
h = 14;

// screw offset [mm]
os = 9.5;
// screw diameter [mm]
ds1 = 4.2;
ds2 = 5.5;


difference() {
    // base
    cylinder(d=w, h=h+hr);
    // cut out connector
    for(r = [0, 180]) {
        rotate([0, 0, r]) 
            translate([-w/2-1, dc/2, h-hc])
                cube([w+2, w/2, h]);
    }
    for(r = [0, 180]) {
        rotate([0, 0, r]) {
            // cut out rails
            translate([-w/2, -dc/2+or, h])
                cube([w+2, dr, hr+1]);
            // drill connector holes
            for(x = [os : -os : -w/2]) {
                translate([x, dc/2-or-dr/2, -1])
                    cylinder(d=ds1, h=h+2);
            }
            // drill base holes
            for(a = [30 : 30 : 150]) {
                rotate([0, 0, a])
                    translate([w/2-ds2, 0, -1])
                        cylinder(d=ds2, h=h+1);
            }
        }
    }
}
