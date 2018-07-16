// PhaseSpace camera dimensions
cam_w=0.1078;
cam_h=0.0570;
cam_d=0.0925;
cam_r=0.004;
cam_w2=cam_w/2;
cam_w2r=cam_w2-cam_r;
cam_h2=cam_h/2;
cam_h2r=cam_h2-cam_r;
cam_d2=cam_d/2;
lens_d=0.004;
mount_d=0.008;

translate([0,0,-cam_d2])
difference()
{
    // Camera body
    linear_extrude(cam_d)
    {
        hull()
        {
            translate([cam_w2r,cam_h2r,0]) { circle(cam_r,$fn=100); }
            translate([-cam_w2r,cam_h2r,0]) { circle(cam_r,$fn=100); }
            translate([-cam_w2r,-cam_h2r,0]) { circle(cam_r,$fn=100); }
            translate([cam_w2r,-cam_h2r,0]) { circle(cam_r,$fn=100); }
        }
    }

    // Lens openings
    translate([-(cam_w2-cam_h2),0,lens_d/2])
    cylinder(lens_d,0.0045,0.0025,$fn=100,true);
    
    translate([(cam_w2-cam_h2),0,lens_d/2])
    cylinder(lens_d,0.0045,0.0025,$fn=100,true);

    // Mount hole
    translate([0,-cam_h2+mount_d/2,cam_d2])
    rotate([90,0,0])
    cylinder(mount_d,0.003,0.003,$fn=100,true);
}
