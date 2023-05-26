#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

const float angle_max_deg = 60;
const float angle_limit_deg = 60;
float roll_out_deg, pitch_out_deg;

// Test the corners of the input range
TEST(RC2RPTest, Corners) {
    // (-1,-1), (-1,1), (1,-1), (1,1)
    float roll_in[]  = {-1, -1,  1, 1};
    float pitch_in[] = {-1,  1, -1, 1};
    float rc = 37.761f;  // roll at 60 deg max/limit
    float pc = 50.768f;  // pitch at 60 deg max/limit
    float roll_val_deg[]  = {-rc, -rc, rc, rc};
    float pitch_val_deg[] = {-pc, pc, -pc, pc};

    for (uint i=0; i<ARRAY_SIZE(roll_val_deg); i++) {
        rc_input_to_roll_pitch(roll_in[i], pitch_in[i], angle_max_deg, angle_limit_deg, roll_out_deg, pitch_out_deg);

        EXPECT_TRUE(fabsf(roll_out_deg  - roll_val_deg[i])  < .0005f);
        EXPECT_TRUE(fabsf(pitch_out_deg - pitch_val_deg[i]) < .0005f);
    }
}

// Test some points on the axes
TEST(RC2RPTest, Axes) {
    // (0,-1), (0,1), (1,0), (-1,1)
    float roll_in[]  = { 0,  0,  0, -1,  0.5,  0.0, 1};
    float pitch_in[] = {-1,  1,  0,  0,  0.0,  0.5, 0};
    float roll_val_deg[] = {0,
                            0,
                            0, 
                            -angle_max_deg,
                            angle_max_deg/2,
                            0,
                            angle_max_deg};
    float pitch_val_deg[] = {-angle_max_deg,
                             angle_max_deg, 0,
                             0,
                             0,
                             angle_max_deg/2,
                             0};

    for (uint i=0; i<ARRAY_SIZE(roll_val_deg); i++) {
        rc_input_to_roll_pitch(roll_in[i], pitch_in[i], angle_max_deg, angle_limit_deg, roll_out_deg, pitch_out_deg);

        EXPECT_TRUE(fabsf(roll_out_deg  - roll_val_deg[i])  < .00005f);
        EXPECT_TRUE(fabsf(pitch_out_deg - pitch_val_deg[i]) < .00005f);
    }

}

// Test some points on the circle at 60 degrees
TEST(RC2RPTest, Circle) {

    // values generated by VPython implementation; roll/pitch deltas are < .02 degrees w.r.t C++ code
    float xy_rp[][4] = {
        {1.000, 0.000, 60.000, 0.000},
        {0.924, 0.383, 53.193, 22.961},
        {0.707, 0.707, 34.005, 42.426},
        {0.383, 0.924, 13.516, 55.433},
        {0.000, 1.000, 0.000, 60.000},
        {-0.383, 0.924, -13.516, 55.433},
        {-0.707, 0.707, -34.005, 42.426},
        {-0.924, 0.383, -53.193, 22.961},
        {-1.000, 0.000, -60.000, 0.000},
        {-0.924, -0.383, -53.193, -22.961},
        {-0.707, -0.707, -34.005, -42.426},
        {-0.383, -0.924, -13.516, -55.433},
        {-0.000, -1.000, -0.000, -60.000},
        {0.383, -0.924, 13.516, -55.433},
        {0.707, -0.707, 34.005, -42.426},
        {0.924, -0.383, 53.193, -22.961}
    };

    for (uint row=0; row<ARRAY_SIZE(xy_rp); row++) {
        float roll_in = xy_rp[row][0];
        float pitch_in = xy_rp[row][1];
        rc_input_to_roll_pitch(roll_in, pitch_in, angle_max_deg, angle_limit_deg, roll_out_deg, pitch_out_deg);

        EXPECT_TRUE(fabsf(roll_out_deg  - xy_rp[row][2]) < .02f);
        EXPECT_TRUE(fabsf(pitch_out_deg - xy_rp[row][3]) < .02f);
    }
}


AP_GTEST_MAIN();