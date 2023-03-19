#include "Copter.h"

// land_init - initialise land controller
bool ModePigeonLand::init(bool ignore_checks)
{
    // check if we have GPS and decide which LAND we're going to do
    control_position = copter.position_ok();

    if(control_position){
        set_mode(Mode::Number::AUTO, ModeReason::PIGEON_GPS_LOCK);
    }

    // set horizontal speed and acceleration limits
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    land_start_time = millis();
    land_pause = false;

    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    return true;
}

// land_run - runs the land controller
// should be called at 100hz or more
void ModeLand::run()
{
    control_position = copter.position_ok();
    if (control_position) {
        set_mode(Mode::Number::AUTO, ModeReason::PIGEON_GPS_LOCK);
    } else {
        nogps_run();
    }
}

// land_nogps_run - runs the land controller
//      pilot controls roll and pitch angles
//      should be called at 100hz or more
void ModeLand::nogps_run()
{
    float target_roll = 0.0f, target_pitch = 0.0f;

    // disarm when the landing detector says we've landed
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // Land State Machine Determination
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
    } else {
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // pause before beginning land descent
        if (land_pause && millis()-land_start_time >= LAND_WITH_DELAY_MS) {
            land_pause = false;
        }

        land_run_vertical_control(land_pause);
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, auto_yaw.get_heading().yaw_rate_cds);
}