#include "Copter.h"

// throw_init - initialise throw controller
bool ModePigeonFall::init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to use throw to start
    return false;
#endif

    // do not enter the mode when already armed or when flying
    if (motors->armed()) {
        return false;
    }

    // init state
    stage = PigeonFall_Disarmed;
    gcs().send_text(MAV_SEVERITY_INFO,"fall mode!");
    nextmode_attempted = false;

    // initialise pos controller speed and acceleration
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), BRAKE_MODE_DECEL_RATE);

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);

    return true;
}

// runs the throw to start controller
// should be called at 100hz or more
void ModePigeonFall::run()
{
    /* Throw State Machine
    Throw_Disarmed - motors are off
    Throw_Detecting -  motors are on and we are waiting for the throw
    Throw_Uprighting - the throw has been detected and the copter is being uprighted
    Throw_HgtStabilise - the copter is kept level and  height is stabilised about the target height
    Throw_PosHold - the copter is kept at a constant position and height
    */

    if (!motors->armed()) {
        // state machine entry is always from a disarmed state
        stage = PigeonFall_Disarmed;

    } else if (stage == PigeonFall_Disarmed && motors->armed()) {
        gcs().send_text(MAV_SEVERITY_INFO,"waiting for unfold :)");
        stage = PigeonFall_Detecting;

    } else if (stage == PigeonFall_Detecting && unfold_detected()){
        gcs().send_text(MAV_SEVERITY_INFO,"unfold detected - it's flying time");
        copter.set_land_complete(false);
        stage = PigeonFall_Wait_Throttle_Unlimited;

        // Cancel the waiting for throw tone sequence
        AP_Notify::flags.waiting_for_throw = false;

    } else if (stage == PigeonFall_Wait_Throttle_Unlimited &&
               motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        gcs().send_text(MAV_SEVERITY_INFO,"throttle is unlimited - uprighting");
        stage = PigeonFall_Uprighting;
    } else if (stage == PigeonFall_Uprighting && throw_attitude_good()) {
        gcs().send_text(MAV_SEVERITY_INFO,"uprighted - controlling height");
        stage = PigeonFall_HgtStabilise;

        // initialise the z controller
        pos_control->init_z_controller_no_descent();

        if(g2.pigeon_fall_type == PigeonFallType::RocketDrop) {
            // in the rocket, we want the drone to shoot for a 4m below since its
            // being slammed downwards
            pos_control->set_pos_target_z_cm(inertial_nav.get_position_z_up_cm() - 400);
            // TODO :
        } else if (g2.pigeon_fall_type == PigeonFallType::Drop) {
            // initialise the demanded height to 1m below the throw height
            // this is for throw mode
            pos_control->set_pos_target_z_cm(inertial_nav.get_position_z_up_cm() - 100);
        } else {
            // initialise the demanded height to 3m above the throw height
            // this is for throw mode
            pos_control->set_pos_target_z_cm(inertial_nav.get_position_z_up_cm() + 300);
        }

        // Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
        copter.set_auto_armed(true);

    } else if (stage == PigeonFall_HgtStabilise && throw_height_good()) {
        gcs().send_text(MAV_SEVERITY_INFO,"height achieved - controlling position");
        stage = PigeonFall_PosHold;

        // initialise position controller
        pos_control->init_xy_controller();

        // Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
        copter.set_auto_armed(true);
    } else if (stage == PigeonFall_PosHold && throw_position_good()) {
        if (!nextmode_attempted) {
            switch ((Mode::Number)g2.pigeon_fall_nextmode.get()) {
                case Mode::Number::AUTO:
                case Mode::Number::GUIDED:
                case Mode::Number::RTL:
                case Mode::Number::LAND:
                case Mode::Number::BRAKE:
                case Mode::Number::LOITER:
                case Mode::Number::PIGEON_LAND:
                    set_mode((Mode::Number)g2.pigeon_fall_nextmode.get(), ModeReason::THROW_COMPLETE);
                    break;
                default:
                    // do nothing
                    break;
            }
            nextmode_attempted = true;
        }
    }

    // Throw State Processing
    switch (stage) {

    case PigeonFall_Disarmed:

        // prevent motors from rotating before the throw is detected unless enabled by the user
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);

        // demand zero throttle (motors will be stopped anyway) and continually reset the attitude controller
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        break;

    case PigeonFall_Detecting:

        // prevent motors from rotating before the throw is detected unless enabled by the user
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);

        // Hold throttle at zero during the throw and continually reset the attitude controller
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_throttle_out(0,true,g.throttle_filt);

        // Play the waiting for throw tone sequence to alert the user
        AP_Notify::flags.waiting_for_throw = true;

        break;

    case PigeonFall_Wait_Throttle_Unlimited:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        break;

    case PigeonFall_Uprighting:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // demand a level roll/pitch attitude with zero yaw rate
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);

        // output 50% throttle and turn off angle boost to maximise righting moment
        attitude_control->set_throttle_out(0.5f, false, g.throttle_filt);

        break;

    case PigeonFall_HgtStabilise:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);

        // call height controller
        pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);
        pos_control->update_z_controller();

        break;

    case PigeonFall_PosHold:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // use position controller to stop
        Vector2f vel;
        Vector2f accel;
        pos_control->input_vel_accel_xy(vel, accel);
        pos_control->update_xy_controller();

        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), 0.0f);

        // call height controller
        pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);
        pos_control->update_z_controller();

        break;
    }

    // log at 10hz or if stage changes
    uint32_t now = AP_HAL::millis();
    if ((stage != prev_stage) || (now - last_log_ms) > 100) {
        prev_stage = stage;
        last_log_ms = now;
        const float velocity = inertial_nav.get_velocity_neu_cms().length();
        const float velocity_z = inertial_nav.get_velocity_z_up_cms();
        const float accel = copter.ins.get_accel().length();
        const float ef_accel_z = ahrs.get_accel_ef().z;
        const bool throw_detect = (stage > PigeonFall_Detecting) || unfold_detected();
        const bool attitude_ok = (stage > PigeonFall_Uprighting) || throw_attitude_good();
        const bool height_ok = (stage > PigeonFall_HgtStabilise) || throw_height_good();
        const bool pos_ok = (stage > PigeonFall_PosHold) || throw_position_good();
        
// @LoggerMessage: PGFL
// @Description: Pigeon Fall Messages
// @Field: TimeUS: Time since system startup
// @Field: Stage: Current stage of the Pigeon Fall Mode
// @Field: Vel: Magnitude of the velocity vector
// @Field: VelZ: Vertical Velocity
// @Field: Acc: Magnitude of the vector of the current acceleration
// @Field: AccEfZ: Vertical earth frame accelerometer value
// @Field: Unfold: True if a unfold has been detected since entering this mode
// @Field: AttOk: True if the vehicle is upright 
// @Field: HgtOk: True if the vehicle is within 50cm of the demanded height
// @Field: PosOk: True if the vehicle is within 50cm of the demanded horizontal position
        
        AP::logger().WriteStreaming(
            "PGFL",
            "TimeUS,Stage,Vel,VelZ,Acc,AccEfZ,Unfold,AttOk,HgtOk,PosOk",
            "s-nnoo----",
            "F-0000----",
            "QBffffbbbb",
            AP_HAL::micros64(),
            (uint8_t)stage,
            (double)velocity,
            (double)velocity_z,
            (double)accel,
            (double)ef_accel_z,
            throw_detect,
            attitude_ok,
            height_ok,
            pos_ok);
    }
}

bool ModePigeonFall::unfold_detected()
{
    gcs().send_text(MAV_SEVERITY_INFO, "%d", hal.rcin->read(PIGEON_UNFOLD_PORT));
    return hal.rcin->read(PIGEON_UNFOLD_PORT) > PIGEON_UNFOLD_THRES;
}

bool ModePigeonFall::throw_attitude_good() const
{
    // Check that we have uprighted the copter
    const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
    return (rotMat.c.z > 0.866f); // is_upright
}

bool ModePigeonFall::throw_height_good() const
{
    // Check that we are within 0.5m of the demanded height
    return (pos_control->get_pos_error_z_cm() < 50.0f);
}

bool ModePigeonFall::throw_position_good() const
{
    // check that our horizontal position error is within 50cm
    return (pos_control->get_pos_error_xy_cm() < 50.0f);
}