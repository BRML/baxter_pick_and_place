//Copyright (c) 2016, BRML
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions are met:
//
//1. Redistributions of source code must retain the above copyright notice,
//this list of conditions and the following disclaimer.
//
//2. Redistributions in binary form must reproduce the above copyright notice,
//this list of conditions and the following disclaimer in the documentation
//and/or other materials provided with the distribution.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.


width = 0.032;  // [m]
length = 0.064;  // [m]
height = 0.019;  // [m]
thickness = 0.0015;  // [m]

nub_d_o = 0.0095;  // [m]
nub_d_i = 0.0065;
nub_h = 0.005;

hole_d_o = 0.0135;
hole_d_i = 0.011;
hole_h = 0.016;


module nubsi(d_o, d_i, h) {
    difference() {
        cylinder(h=h, d=d_o, center=false);
        translate([0, 0, -0.05])
            cylinder(h=h+0.1, d=d_i, center=false);
    }
}


translate([-width/2, -length/2, 0])
color("red")
union() {
    // base block
    difference() {
        cube([width, length, height], center=false);
        translate([thickness, thickness, -thickness-1])
            cube([width-2*thickness, length-2*thickness, height-thickness+1], center=false);
    }
    
    // nubs on top
    help_a = (width-2*nub_d_o)/4;
    help_b = help_a+nub_d_o/2;
    help_c = 2*help_a+nub_d_o;
    for (x=[help_b, help_b+help_c]) {
        for (y=[help_b : help_c : length]) {
            translate([x, y, height])
                nubsi(d_o=nub_d_o, d_i=nub_d_i, h=nub_h);
        }
    }
    
    // inner structure
    
    for (y=[-help_c, 0, help_c]) {
        translate([width/2, length/2+y, height-thickness-hole_h])
            nubsi(d_o=hole_d_o, d_i=hole_d_i, h=hole_h);
    }
}