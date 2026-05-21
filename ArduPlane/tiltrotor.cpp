#include "tiltrotor.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED
const AP_Param::GroupInfo Tiltrotor::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable Tiltrotor functionality
    // @Values: 0:Disable, 1:Enable
    // @Description: This enables Tiltrotor functionality
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, Tiltrotor, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: MASK
    // @DisplayName: Tiltrotor mask
    // @Description: This is a bitmask of motors that are tiltable in a tiltrotor (or tiltwing). The mask is in terms of the standard motor order for the frame type.
    // @User: Standard
    // @Bitmask: 0:Motor 1, 1:Motor 2, 2:Motor 3, 3:Motor 4, 4:Motor 5, 5:Motor 6, 6:Motor 7, 7:Motor 8, 8:Motor 9, 9:Motor 10, 10:Motor 11, 11:Motor 12
    AP_GROUPINFO("MASK", 2, Tiltrotor, tilt_mask, 0),

    // @Param: RATE_UP
    // @DisplayName: Tiltrotor upwards tilt rate
    // @Description: This is the maximum speed at which the motor angle will change for a tiltrotor when moving from forward flight to hover
    // @Units: deg/s
    // @Increment: 1
    // @Range: 10 300
    // @User: Standard
    AP_GROUPINFO("RATE_UP", 3, Tiltrotor, max_rate_up_dps, 40),

    // @Param: MAX
    // @DisplayName: Tiltrotor maximum VTOL angle
    // @Description: This is the maximum angle of the tiltable motors at which multicopter control will be enabled. Beyond this angle the plane will fly solely as a fixed wing aircraft and the motors will tilt to their maximum angle at the TILT_RATE
    // @Units: deg
    // @Increment: 1
    // @Range: 20 80
    // @User: Standard
    AP_GROUPINFO("MAX", 4, Tiltrotor, max_angle_deg, 45),

    // @Param: TYPE
    // @DisplayName: Tiltrotor type
    // @Description: This is the type of tiltrotor when TILT_MASK is non-zero. A continuous tiltrotor can tilt the rotors to any angle on demand. A binary tiltrotor assumes a retract style servo where the servo is either fully forward or fully up. In both cases the servo can't move faster than Q_TILT_RATE. A vectored yaw tiltrotor will use the tilt of the motors to control yaw in hover, Bicopter tiltrotor must use the tailsitter frame class (10)
    // @Values: 0:Continuous,1:Binary,2:VectoredYaw,3:Bicopter
    AP_GROUPINFO("TYPE", 5, Tiltrotor, type, TILT_TYPE_CONTINUOUS),

    // @Param: RATE_DN
    // @DisplayName: Tiltrotor downwards tilt rate
    // @Description: This is the maximum speed at which the motor angle will change for a tiltrotor when moving from hover to forward flight. When this is zero the Q_TILT_RATE_UP value is used.
    // @Units: deg/s
    // @Increment: 1
    // @Range: 10 300
    // @User: Standard
    AP_GROUPINFO("RATE_DN", 6, Tiltrotor, max_rate_down_dps, 0),

    // @Param: YAW_ANGLE
    // @DisplayName: Tilt minimum angle for vectored yaw
    // @Description: This is the angle of the tilt servos when in VTOL mode and at minimum output (fully back). This needs to be set in addition to Q_TILT_TYPE=2, to enable vectored control for yaw in tilt quadplanes. This is also used to limit the forward travel of bicopter tilts(Q_TILT_TYPE=3) when in VTOL modes.
    // @Range: 0 30
    AP_GROUPINFO("YAW_ANGLE", 7, Tiltrotor, tilt_yaw_angle, 0),

    // @Param: FIX_ANGLE
    // @DisplayName: Fixed wing tiltrotor angle
    // @Description: This is the angle the motors tilt down when at maximum output for forward flight. Set this to a non-zero value to enable vectoring for roll/pitch in forward flight on tilt-vectored aircraft
    // @Units: deg
    // @Range: 0 30
    // @User: Standard
    AP_GROUPINFO("FIX_ANGLE", 8, Tiltrotor, fixed_angle, 0),

    // @Param: FIX_GAIN
    // @DisplayName: Fixed wing tiltrotor gain
    // @Description: This is the gain for use of tilting motors in fixed wing flight for tilt vectored quadplanes
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("FIX_GAIN", 9, Tiltrotor, fixed_gain, 0),

    // @Param: WING_FLAP
    // @DisplayName: Tiltrotor tilt angle that will be used as flap
    // @Description: For use on tilt wings, the wing will tilt up to this angle for flap, transition will be complete when the wing reaches this angle from the forward fight position, 0 disables
    // @Units: deg
    // @Increment: 1
    // @Range: 0 15
    // @User: Standard
    AP_GROUPINFO("WING_FLAP", 10, Tiltrotor, flap_angle_deg, 0),

    // @Param: YAW_THR
    // @DisplayName: Yaw blend start angle
    // @Description: Rear motor tilt angle (deg) at which torque-to-vectored yaw blend begins. Below this angle rear motors use full RPM torque yaw. Blend completes at YAW_THR + YAW_RNG degrees.
    // @Units: deg
    // @Range: 5 30
    // @User: Advanced
    AP_GROUPINFO("YAW_THR", 11, Tiltrotor, yaw_bld_thr, 15),

    // @Param: YAW_RNG
    // @DisplayName: Yaw blend range
    // @Description: Tilt angle range (deg) over which yaw transitions from RPM torque to vectored servo. Blend is complete at Q_TILT_YAW_THR + Q_TILT_YAW_RNG.
    // @Units: deg
    // @Range: 5 30
    // @User: Advanced
    AP_GROUPINFO("YAW_RNG", 12, Tiltrotor, yaw_bld_rng, 10),

    // @Param: CT_PTRIM
    // @DisplayName: CT-shift pitch trim
    // @Description: Nose-up pitch angle (deg) commanded at full tilt (tilt_frac=1.0) to compensate for the rearward centre-of-thrust shift as rear motors tilt backward. Applied linearly with tilt fraction in QTILTCRUISE cruise sub-mode only. Default 0 (disabled). Tune empirically: positive values trim nose up. Typical range 2-8 deg depending on rotor arm length and payload distribution.
    // @Units: deg
    // @Range: -10 10
    // @User: Advanced
    AP_GROUPINFO("CT_PTRIM", 13, Tiltrotor, ctrim_deg, 0),

    // @Param: CSINV
    // @DisplayName: QTiltCruise stick invert
    // @Description: When 0 (default), pushing the pitch stick forward (positive) engages tilt-cruise sub-mode in QTILTCRUISE. When 1, pulling the pitch stick back (negative) engages tilt-cruise instead. Use this when the natural flying posture calls for a pull-to-cruise input without reversing the RC channel globally.
    // @Values: 0:ForwardToEngage,1:BackToEngage
    // @User: Standard
    AP_GROUPINFO("CSINV", 14, Tiltrotor, cruise_inv, 0),

    // @Param: CSDZ
    // @DisplayName: QTiltCruise stick dead zone
    // @Description: Pitch stick deflection (centidegrees) required to engage or exit tilt-cruise sub-mode in QTILTCRUISE. The stick must exceed this threshold in the engage direction (forward by default, back when Q_TILT_CSINV=1) to enter cruise, and drop below it to return to quad sub-mode. Increase to reduce sensitivity to stick trim offset or RC noise. Range 50-1000 cd (1.1%-22% travel).
    // @Units: cdeg
    // @Range: 50 1000
    // @User: Standard
    AP_GROUPINFO("CSDZ", 15, Tiltrotor, cruise_dz, 200),

    // TC-submode shadow params — override the base param only when QTILTCRUISE cruise
    // sub-mode is engaged.  Set to -1 (default) to inherit the corresponding base param.

    // @Param: TC_MAX
    // @DisplayName: QTiltCruise max tilt angle
    // @Description: Maximum tilt angle (deg) used in QTILTCRUISE cruise sub-mode. -1 inherits Q_TILT_MAX.
    // @Units: deg
    // @Range: -1 80
    // @User: Advanced
    AP_GROUPINFO("TC_MAX", 16, Tiltrotor, tc_max_angle_deg, -1),

    // @Param: TC_RATEUP
    // @DisplayName: QTiltCruise tilt-back rate
    // @Description: Rate (deg/s) at which tilt servos return toward vertical while in QTILTCRUISE cruise sub-mode. -1 inherits Q_TILT_RATE_UP.
    // @Units: deg/s
    // @Range: -1 300
    // @User: Advanced
    AP_GROUPINFO("TC_RATEUP", 17, Tiltrotor, tc_max_rate_up_dps, -1),

    // @Param: TC_RATEDN
    // @DisplayName: QTiltCruise tilt-forward rate
    // @Description: Rate (deg/s) at which tilt servos move forward while in QTILTCRUISE cruise sub-mode. -1 inherits Q_TILT_RATE_DN (or Q_TILT_RATE_UP if that is zero).
    // @Units: deg/s
    // @Range: -1 300
    // @User: Advanced
    AP_GROUPINFO("TC_RATEDN", 18, Tiltrotor, tc_max_rate_down_dps, -1),

    // @Param: TC_YTHR
    // @DisplayName: QTiltCruise yaw-blend start angle
    // @Description: Tilt angle (deg) at which the torque-to-vectored yaw blend begins in QTILTCRUISE cruise sub-mode. -1 inherits Q_TILT_YAW_THR.
    // @Units: deg
    // @Range: -1 30
    // @User: Advanced
    AP_GROUPINFO("TC_YTHR", 19, Tiltrotor, tc_yaw_bld_thr, -1),

    // @Param: TC_YRNG
    // @DisplayName: QTiltCruise yaw-blend range
    // @Description: Tilt angle range (deg) over which yaw authority transitions in QTILTCRUISE cruise sub-mode. -1 inherits Q_TILT_YAW_RNG.
    // @Units: deg
    // @Range: -1 30
    // @User: Advanced
    AP_GROUPINFO("TC_YRNG", 20, Tiltrotor, tc_yaw_bld_rng, -1),

    // @Param: TC_SPDUP
    // @DisplayName: QTiltCruise Z-ctrl climb rate
    // @Description: Maximum pilot-commanded climb rate (m/s) for the Z-axis controller in QTILTCRUISE cruise sub-mode. -1 inherits Q_PILOT_SPD_UP.
    // @Units: m/s
    // @Range: -1 10
    // @User: Advanced
    AP_GROUPINFO("TC_SPDUP", 21, Tiltrotor, tc_pilot_spd_up_ms, -1),

    // @Param: TC_SPDDN
    // @DisplayName: QTiltCruise Z-ctrl descent rate
    // @Description: Maximum pilot-commanded descent rate (m/s) for the Z-axis controller in QTILTCRUISE cruise sub-mode. -1 inherits Q_PILOT_SPD_DN (or Q_PILOT_SPD_UP if that is zero).
    // @Units: m/s
    // @Range: -1 10
    // @User: Advanced
    AP_GROUPINFO("TC_SPDDN", 22, Tiltrotor, tc_pilot_spd_dn_ms, -1),

    // @Param: TC_ACCZ
    // @DisplayName: QTiltCruise Z-ctrl acceleration
    // @Description: Maximum vertical acceleration (m/s²) for the Z-axis controller in QTILTCRUISE cruise sub-mode. -1 inherits Q_PILOT_ACCEL_Z.
    // @Units: m/s/s
    // @Range: -1 20
    // @User: Advanced
    AP_GROUPINFO("TC_ACCZ", 23, Tiltrotor, tc_pilot_accel_mss, -1),

    // @Param: TC_YAWP
    // @DisplayName: QTiltCruise yaw rate P
    // @Description: Overrides Q_A_RAT_YAW_P in QTILTCRUISE cruise sub-mode. -1 inherits Q_A_RAT_YAW_P.
    // @Range: -1 0.5
    // @User: Advanced
    AP_GROUPINFO("TC_YAWP", 24, Tiltrotor, tc_yaw_rate_p, -1),

    // @Param: TC_YAWI
    // @DisplayName: QTiltCruise yaw rate I
    // @Description: Overrides Q_A_RAT_YAW_I in QTILTCRUISE cruise sub-mode. -1 inherits Q_A_RAT_YAW_I.
    // @Range: -1 0.5
    // @User: Advanced
    AP_GROUPINFO("TC_YAWI", 25, Tiltrotor, tc_yaw_rate_i, -1),

    // @Param: TC_YAWD
    // @DisplayName: QTiltCruise yaw rate D
    // @Description: Overrides Q_A_RAT_YAW_D in QTILTCRUISE cruise sub-mode. -1 inherits Q_A_RAT_YAW_D.
    // @Range: -1 0.02
    // @User: Advanced
    AP_GROUPINFO("TC_YAWD", 26, Tiltrotor, tc_yaw_rate_d, -1),

    // @Param: TC_YAWFF
    // @DisplayName: QTiltCruise yaw rate FF
    // @Description: Overrides Q_A_RAT_YAW_FF in QTILTCRUISE cruise sub-mode. -1 inherits Q_A_RAT_YAW_FF.
    // @Range: -1 1.0
    // @User: Advanced
    AP_GROUPINFO("TC_YAWFF", 27, Tiltrotor, tc_yaw_rate_ff, -1),

    // @Param: TC_YAWAP
    // @DisplayName: QTiltCruise yaw angle P
    // @Description: Overrides Q_A_ANG_YAW_P in QTILTCRUISE cruise sub-mode. -1 inherits Q_A_ANG_YAW_P.
    // @Range: -1 30
    // @User: Advanced
    AP_GROUPINFO("TC_YAWAP", 28, Tiltrotor, tc_ang_yaw_p, -1),

    AP_GROUPEND
};

/*
  control code for tiltrotors and tiltwings. Enabled by setting
  Q_TILT_MASK to a non-zero value
 */

Tiltrotor::Tiltrotor(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors):_rear_yaw_blend(0.0f),_in_tc_cruise(false),quadplane(_quadplane),motors(_motors)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void Tiltrotor::setup()
{

    if (!enable.configured() && ((tilt_mask != 0) || (type == TILT_TYPE_BICOPTER))) {
        enable.set_and_save(1);
    }

    if (enable <= 0) {
        return;
    }

    quadplane.thrust_type = QuadPlane::ThrustType::TILTROTOR;

    _is_vectored = tilt_mask != 0 && type == TILT_TYPE_VECTORED_YAW;

    // true if a fixed forward motor is configured, either throttle, throttle left  or throttle right.
    // bicopter tiltrotors use throttle left and right as tilting motors, so they don't count in that case.
    _have_fw_motor = SRV_Channels::function_assigned(SRV_Channel::k_throttle) ||
                    ((SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft) || SRV_Channels::function_assigned(SRV_Channel::k_throttleRight))
                        && (type != TILT_TYPE_BICOPTER));


    // check if there are any permanent VTOL motors
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        if (motors->is_motor_enabled(i) && !is_motor_tilting(i)) {
            // enabled motor not set in tilt mask
            _have_vtol_motor = true;
            break;
        }
    }

    if (_is_vectored) {
        if (_have_vtol_motor) {
            // Partial-tilt: some motors are fixed (non-tilting), others tilt.
            // Save normalised yaw factors after normalise_rpy_factors() has run.
            // Fixed motors retain their saved factor permanently (full torque yaw always).
            // Tilting motors start at zero; update_yaw_blend() scales them each 400Hz cycle.
            motors->save_yaw_factors();

            // Sanity-check: if all non-tilting (fixed) motors have zero saved yaw
            // factors, normalise_rpy_factors() has not run yet and the snapshot is
            // invalid.  This would silently zero all front-motor yaw authority in hover.
            bool any_nonzero = false;
            for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motors->is_motor_enabled(i) && !is_motor_tilting(i)) {
                    if (!is_zero(motors->get_saved_yaw_factor(i))) {
                        any_nonzero = true;
                        break;
                    }
                }
            }
            if (!any_nonzero) {
                gcs().send_text(MAV_SEVERITY_ERROR,
                    "TiltYaw: yaw factors zero at save — check motor init order");
            }

            for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (is_motor_tilting(i)) {
                    motors->set_yaw_factor(i, 0.0f);
                }
            }
            _rear_yaw_blend = 0.0f;  // hover: full torque yaw on fixed motors, no vectored differential
        } else {
            // Full-tilt: all motors tilt, so all yaw authority is vectored throughout.
            // Disable torque yaw for all motors (original upstream behaviour).
            // Set blend to 1.0 so vectoring() applies full differential unconditionally —
            // matching the old hard-coded rear servo outputs that existed before this PR.
            motors->disable_yaw_torque();
            _rear_yaw_blend = 1.0f;
        }
    }

    if (tilt_mask != 0) {
        // setup tilt compensation
        motors->set_thrust_compensation_callback(FUNCTOR_BIND_MEMBER(&Tiltrotor::tilt_compensate, void, float *, uint8_t));
        if (type == TILT_TYPE_VECTORED_YAW) {
            // setup tilt servos for vectored yaw
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorLeft,  1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRight, 1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRear,  1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRearLeft, 1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRearRight, 1000);
        }
    }

    transition = NEW_NOTHROW Tiltrotor_Transition(quadplane, motors, *this);
    if (!transition) {
        AP_BoardConfig::allocation_error("tiltrotor transition");
    }
    quadplane.transition = transition;

    setup_complete = true;
}

// ---------------------------------------------------------------------------
// TC-submode Z-controller limit getters
// Each returns the TC-specific param if set (≥ 0), otherwise the quad param.
// ---------------------------------------------------------------------------

float Tiltrotor::get_tc_pilot_spd_up_ms() const
{
    if (tc_pilot_spd_up_ms.get() >= 0.0f) {
        return tc_pilot_spd_up_ms.get();
    }
    return (float)quadplane.pilot_speed_z_max_up_ms;
}

float Tiltrotor::get_tc_pilot_spd_dn_ms() const
{
    if (tc_pilot_spd_dn_ms.get() >= 0.0f) {
        return tc_pilot_spd_dn_ms.get();
    }
    return (float)quadplane.get_pilot_velocity_z_max_dn_m();
}

float Tiltrotor::get_tc_pilot_accel_mss() const
{
    if (tc_pilot_accel_mss.get() >= 0.0f) {
        return tc_pilot_accel_mss.get();
    }
    return (float)quadplane.pilot_accel_z_mss;
}

/*
  calculate maximum tilt change as a proportion from 0 to 1 of tilt
 */
float Tiltrotor::tilt_max_change(bool up, bool in_flap_range) const
{
    float rate;
    if (_in_tc_cruise) {
        // TC submode: use TC-specific rates when > 0, else inherit base param.
        // Note: check is > 0 (not >= 0) because a zero rate would freeze the
        // servo — treat 0 as "not configured" same as -1.
        if (up) {
            rate = (tc_max_rate_up_dps.get() > 0)
                   ? (float)tc_max_rate_up_dps.get()
                   : (float)max_rate_up_dps;
        } else {
            if (tc_max_rate_down_dps.get() > 0) {
                rate = (float)tc_max_rate_down_dps.get();
            } else if (max_rate_down_dps > 0) {
                rate = (float)max_rate_down_dps;
            } else {
                rate = (float)max_rate_up_dps;
            }
        }
        // fast_tilt override is intentionally skipped in TC sub-mode.
    } else {
        if (up || max_rate_down_dps <= 0) {
            rate = max_rate_up_dps;
        } else {
            rate = max_rate_down_dps;
        }
        if (type != TILT_TYPE_BINARY && !up && !in_flap_range) {
            bool fast_tilt = false;
            if (plane.control_mode == &plane.mode_manual) {
                fast_tilt = true;
            }
            if (plane.arming.is_armed_and_safety_off() && !quadplane.in_vtol_mode() && !quadplane.assisted_flight) {
                fast_tilt = true;
            }
            if (fast_tilt) {
                // allow a minimum of 90 DPS in manual or if we are not
                // stabilising, to give fast control
                rate = MAX(rate, 90);
            }
        }
    }
    return rate * plane.G_Dt * (1/90.0);
}

/*
  output a slew limited tiltrotor angle. tilt is from 0 to 1
 */
void Tiltrotor::slew(float newtilt)
{
    float max_change = tilt_max_change(newtilt<current_tilt, newtilt > get_fully_forward_tilt());
    current_tilt = constrain_float(newtilt, current_tilt-max_change, current_tilt+max_change);

    angle_achieved = is_equal(newtilt, current_tilt);

    // translate to 0..1000 range and output
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, 1000 * current_tilt);
}

// return the current tilt value that represents forward flight
// tilt wings can sustain forward flight with some amount of wing tilt
float Tiltrotor::get_fully_forward_tilt() const
{
    return 1.0 - (flap_angle_deg * (1/90.0));
}

// return the target tilt value for forward flight
float Tiltrotor::get_forward_flight_tilt() const
{
    return 1.0 - ((flap_angle_deg * (1/90.0)) * SRV_Channels::get_slew_limited_output_scaled(SRV_Channel::k_flap_auto) * 0.01);
}

/*
  update motor tilt for continuous tilt servos
 */
void Tiltrotor::continuous_update(void)
{
    // default to inactive
    _motors_active = false;

    // the maximum rate of throttle change
    float max_change;

    if (!quadplane.in_vtol_mode() && (!plane.arming.is_armed_and_safety_off() || !quadplane.assisted_flight)) {
        // we are in pure fixed wing mode. Move the tiltable motors all the way forward and run them as
        // a forward motor

        // option set then if disarmed move to VTOL position to prevent ground strikes, allow tilt forward in manual mode for testing
        const bool disarmed_tilt_up = !plane.arming.is_armed_and_safety_off() && (plane.control_mode != &plane.mode_manual) && quadplane.option_is_set(QuadPlane::Option::DISARMED_TILT_UP);
        slew(disarmed_tilt_up ? 0.0 : get_forward_flight_tilt());

        max_change = tilt_max_change(false);

        float new_throttle = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01, 0, 1);
        if (current_tilt < get_fully_forward_tilt()) {
            current_throttle = constrain_float(new_throttle,
                                                    current_throttle-max_change,
                                                    current_throttle+max_change);
        } else {
            current_throttle = new_throttle;
        }
        if (!plane.arming.is_armed_and_safety_off()) {
            current_throttle = 0;
        } else {
            // prevent motor shutdown
            _motors_active = true;
        }
        if (!quadplane.motor_test.running) {
            // the motors are all the way forward, start using them for fwd thrust
            const uint32_t mask = is_zero(current_throttle) ? 0U : tilt_mask.get();
            motors->output_motor_mask(current_throttle, mask, plane.rudder_dt);
        }
        return;
    }

    // remember the throttle level we're using for VTOL flight
    float motors_throttle = motors->get_throttle();
    max_change = tilt_max_change(motors_throttle<current_throttle);
    current_throttle = constrain_float(motors_throttle,
                                            current_throttle-max_change,
                                            current_throttle+max_change);

    /*
      we are in a VTOL mode. We need to work out how much tilt is
      needed. There are 5 strategies we will use:

      1) With use of a forward throttle controlled by Q_FWD_THR_GAIN in
         VTOL modes except Q_AUTOTUNE determined by Q_FWD_THR_USE. We set the angle based on a calculated
         forward throttle.

      2) With manual forward throttle control we set the angle based on the
         RC input demanded forward throttle for QACRO, QSTABILIZE and QHOVER.

      3) Without a RC input or calculated forward throttle value, the angle
         will be set to zero in QAUTOTUNE, QACRO, QSTABILIZE and QHOVER.
         This enables these modes to be used as a safe recovery mode.

      4) In fixed wing assisted flight or velocity controlled modes we will
         set the angle based on the demanded forward throttle, with a maximum
         tilt given by Q_TILT_MAX. This relies on Q_FWD_THR_GAIN or Q_VFWD_GAIN
         being set.

      5) if we are in TRANSITION_TIMER mode then we are transitioning
         to forward flight and should put the rotors all the way forward
    */

    // --- QTILTCRUISE: altitude-hold tilt scheduling ---
    // When the RC pitch stick engages cruise sub-mode (forward by default, or
    // backward when Q_TILT_CSINV=1), compute the tilt angle at which the
    // vertical component of rotor thrust equals the estimated hover thrust.
    // This keeps altitude constant as the pilot varies throttle.
    //
    // For vehicles where only some motors tilt (e.g. rear-only), fixed motors
    // always contribute vertical thrust so the effective balance is:
    //   fixed_frac * T + rear_frac * T * cos(θ) = hover_thr
    //   cos(θ) = (hover_thr - fixed_frac * pilot_thr) / (rear_frac * pilot_thr)
    // When all motors tilt (rear_frac = 1, fixed_frac = 0) this reduces to
    // the original cos(θ) = hover_thr / pilot_thr.
    //
    // IMPORTANT: tilt is computed from the PILOT'S desired throttle, not from
    // motors->get_throttle() (which includes Z-PID corrections). This decouples
    // the two subsystems: tilt tracks pilot-intended speed, while the Z-PID in
    // hold_hover() adds/subtracts actual motor throttle on top for altitude
    // corrections. Using motors->get_throttle() would neutralise every upward
    // Z-PID correction by tilting further forward, leaving the aircraft unable
    // to recover from downward disturbances.
    //
    // When pitch stick is not in the cruise zone, slew back to vertical (θ = 0).
    if (plane.control_mode == &plane.mode_qtiltcruise) {
        const float pitch_in = plane.channel_pitch->get_control_in(); // -4500..+4500
        const bool cruise_inverted = (cruise_inv.get() != 0);
        const float dz = MAX((float)cruise_dz.get(), 50.0f);
        const bool in_cruise_submode = cruise_inverted ? (pitch_in < -dz) : (pitch_in > dz);
        // Gate TC-param overrides in tilt_max_change() and update_yaw_blend().
        _in_tc_cruise = in_cruise_submode;
        if (in_cruise_submode) {
            // Pilot-desired throttle sets the forward-speed/tilt target.
            // Z-PID corrections act on actual motor throttle independently.
            const float pilot_thr  = constrain_float(quadplane.get_pilot_throttle(), 0.01f, 1.0f);
            const float hover_thr  = constrain_float(motors->get_throttle_hover(), 0.10f, 0.90f);
            float tilt_frac = 0.0f;
            if (pilot_thr > hover_thr) {
                // Corrected tilt formula for partial-tilt vehicles where only
                // some motors tilt and the rest are fixed vertical.
                // Force balance: fixed_frac*T + rear_frac*T*cos(θ) = hover_thr
                // Solved for cos(θ) at T = pilot_thr (the designed equilibrium).
                // Falls back to arccos(hover_thr/pilot_thr) when all motors tilt.
                const uint8_t num_tilt  = (uint8_t)__builtin_popcount((uint32_t)tilt_mask.get());
                const uint8_t num_total = (uint8_t)__builtin_popcount(motors->get_motor_mask());
                const float rear_frac   = (num_total > 0) ? ((float)num_tilt / (float)num_total) : 1.0f;
                const float fixed_frac  = 1.0f - rear_frac;
                const float cos_theta   = (hover_thr - fixed_frac * pilot_thr) /
                                           MAX(rear_frac * pilot_thr, 0.001f);
                tilt_frac = degrees(acosf(constrain_float(cos_theta, -1.0f, 1.0f))) / 90.0f;
                // Use TC-specific max angle if configured, else fall back to Q_TILT_MAX.
                // Reserve tilt_yaw_angle degrees of servo headroom for yaw corrections.
                // Floor at 5° guards against tilt_yaw_angle >= effective max.
                const float eff_max = (tc_max_angle_deg.get() > 0)
                                      ? (float)tc_max_angle_deg.get()
                                      : (float)max_angle_deg;
                const float tc_max_frac = MAX(eff_max - tilt_yaw_angle.get(), 5.0f) / 90.0f;
                tilt_frac = constrain_float(tilt_frac, 0.0f, tc_max_frac);
            }
            slew(tilt_frac);
        } else {
            // Not in cruise sub-mode — return servos to vertical at the base rate
            // (Q_TILT_RATE_UP).  _in_tc_cruise is false here so tilt_max_change()
            // uses the base param regardless of any TC_RATEUP setting.
            slew(0.0f);
        }
        return;
    }
    // --- end QTILTCRUISE ---
    _in_tc_cruise = false;

#if QAUTOTUNE_ENABLED
    if (plane.control_mode == &plane.mode_qautotune) {
        slew(0);
        return;
    }
#endif

    if (!quadplane.assisted_flight &&
        quadplane.get_vfwd_method() == QuadPlane::ActiveFwdThr::NEW &&
        quadplane.is_flying_vtol())
    {
        // We are using the rotor tilt functionality controlled by Q_FWD_THR_GAIN which can
        // operate in all VTOL modes except Q_AUTOTUNE. Forward rotor tilt is used to produce
        // forward thrust equivalent to what would have been produced by a forward thrust motor
        // set to quadplane.forward_throttle_pct()
        const float fwd_g_demand = 0.01 * quadplane.forward_throttle_pct();
        const float fwd_tilt_deg = MIN(degrees(atanf(fwd_g_demand)), (float)max_angle_deg);
        slew(MIN(fwd_tilt_deg * (1/90.0), get_forward_flight_tilt()));
        return;
    } else if (!quadplane.assisted_flight &&
               (plane.control_mode == &plane.mode_qacro ||
               plane.control_mode == &plane.mode_qstabilize ||
               plane.control_mode == &plane.mode_qhover))
    {
        if (quadplane.rc_fwd_thr_ch == nullptr) {
            // no manual throttle control, set angle to zero
            slew(0);
        } else {
            // manual control of forward throttle up to max VTOL angle
            float settilt = 0.01f * quadplane.forward_throttle_pct();
            slew(MIN(settilt * max_angle_deg * (1/90.0), get_forward_flight_tilt())); 
        }
        return;
    }

    if (quadplane.assisted_flight &&
        transition->transition_state >= Tiltrotor_Transition::State::TIMER) {
        // we are transitioning to fixed wing - tilt the motors all
        // the way forward
        slew(get_forward_flight_tilt());
    } else {
        // until we have completed the transition we limit the tilt to
        // Q_TILT_MAX. Anything above 50% throttle gets
        // Q_TILT_MAX. Below 50% throttle we decrease linearly. This
        // relies heavily on Q_VFWD_GAIN being set appropriately.
       float settilt = constrain_float((SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)-MAX(plane.aparm.throttle_min.get(),0)) * 0.02, 0, 1);
       slew(MIN(settilt * max_angle_deg * (1/90.0), get_forward_flight_tilt())); 
    }
}


/*
  output a slew limited tiltrotor angle. tilt is 0 or 1
 */
void Tiltrotor::binary_slew(bool forward)
{
    // The servo output is binary, not slew rate limited
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, forward?1000:0);

    // rate limiting current_tilt has the effect of delaying throttle in tiltrotor_binary_update
    float max_change = tilt_max_change(!forward);
    if (forward) {
        current_tilt = constrain_float(current_tilt+max_change, 0, 1);
    } else {
        current_tilt = constrain_float(current_tilt-max_change, 0, 1);
    }
}

/*
  update motor tilt for binary tilt servos
 */
void Tiltrotor::binary_update(void)
{
    // motors always active
    _motors_active = true;

    if (!quadplane.in_vtol_mode()) {
        // we are in pure fixed wing mode. Move the tiltable motors
        // all the way forward and run them as a forward motor
        binary_slew(true);

        float new_throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01f;
        if (current_tilt >= 1) {
            const uint32_t mask = is_zero(new_throttle) ? 0 : tilt_mask.get();
            // the motors are all the way forward, start using them for fwd thrust
            motors->output_motor_mask(new_throttle, mask, plane.rudder_dt);
        }
    } else {
        binary_slew(false);
    }
}


/*
  update rear motor yaw factors and _rear_yaw_blend as a function of current_tilt.
  Called between continuous_update() (which updates current_tilt) and vectoring()
  (which reads _rear_yaw_blend for servo differential blending).

  At blend=0 (hover, tilt < YAW_THR): rear motors use full torque yaw via _yaw_factor.
  At blend=1 (cruise, tilt > YAW_THR + YAW_RNG): rear motors use full vectored yaw;
  _yaw_factor is zero so no RPM differential yaw from them.
  Front non-tilting motors always retain their saved yaw factor (100% torque yaw).
*/
void Tiltrotor::update_yaw_blend(void)
{
    if (!_is_vectored || !_have_vtol_motor) {
        // Full-tilt vehicles: _rear_yaw_blend is permanently 1.0 (set in setup()).
        // No per-cycle blend work needed — all yaw authority is already vectored.
        return;
    }
    // In TC submode use TC-specific yaw blend params if configured (≥ 0), else base params.
    const float thr = (_in_tc_cruise && tc_yaw_bld_thr.get() >= 0.0f)
                      ? tc_yaw_bld_thr.get() : yaw_bld_thr.get();
    const float rng = (_in_tc_cruise && tc_yaw_bld_rng.get() >= 0.0f)
                      ? tc_yaw_bld_rng.get() : yaw_bld_rng.get();
    const float thr_frac = thr / 90.0f;
    const float rng_frac = MAX(rng, 1.0f) / 90.0f;
    _rear_yaw_blend = constrain_float((current_tilt - thr_frac) / rng_frac, 0.0f, 1.0f);

    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (is_motor_tilting(i)) {
            motors->set_yaw_factor(i,
                motors->get_saved_yaw_factor(i) * (1.0f - _rear_yaw_blend));
        }
    }
}

/*
  update motor tilt
 */
void Tiltrotor::update(void)
{
    if (!enabled() || tilt_mask == 0) {
        // no motors to tilt
        return;
    }

    if (type == TILT_TYPE_BINARY) {
        binary_update();
    } else {
        continuous_update();
    }

    if (type == TILT_TYPE_VECTORED_YAW) {
        update_yaw_blend();   // reads current_tilt → updates _yaw_factor[] and _rear_yaw_blend
        vectoring();          // reads _rear_yaw_blend → writes servo outputs
    }
}

#if HAL_LOGGING_ENABLED
// Write tiltrotor specific log
void Tiltrotor::write_log()
{
    // Only valid on a tiltrotor
    if (!enabled()) {
        return;
    }

    struct log_tiltrotor pkt {
        LOG_PACKET_HEADER_INIT(LOG_TILT_MSG),
        time_us        : AP_HAL::micros64(),
        current_tilt   : current_tilt * 90.0,
        front_left_tilt  : AP_Logger::quiet_nanf(),
        front_right_tilt : AP_Logger::quiet_nanf(),
        rear_yaw_blend : _rear_yaw_blend,
    };

    if (type != TILT_TYPE_VECTORED_YAW) {
        // Left and right tilt are invalid
        pkt.front_left_tilt = AP_Logger::quiet_nanf();
        pkt.front_right_tilt = AP_Logger::quiet_nanf();

    } else {
        // Calculate tilt angle from servo outputs.
        // For partial-tilt vehicles (_have_vtol_motor) the front motors are fixed
        // and k_tiltMotorLeft/Right are unassigned — read rear servo channels instead
        // so the log shows actual tilting-motor positions.
        // For full-tilt vehicles all four channels are valid; rear channels are equally
        // informative (they share the same base_output in non-differential cruise).
        const float total_angle = 90.0 + tilt_yaw_angle + fixed_angle;
        const float scale = total_angle * 0.001;
        if (_have_vtol_motor) {
            pkt.front_left_tilt  = (SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRearLeft)  * scale) - tilt_yaw_angle;
            pkt.front_right_tilt = (SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRearRight) * scale) - tilt_yaw_angle;
        } else {
            pkt.front_left_tilt  = (SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft)  * scale) - tilt_yaw_angle;
            pkt.front_right_tilt = (SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight) * scale) - tilt_yaw_angle;
        }
    }

    plane.logger.WriteBlock(&pkt, sizeof(pkt));
}
#endif

/*
  tilt compensation for angle of tilt. When the rotors are tilted the
  roll effect of differential thrust on the tilted rotors is decreased
  and the yaw effect increased
  We have two factors we apply.

  1) when we are transitioning to fwd flight we scale the tilted rotors by 1/cos(angle). This pushes us towards more flight speed

  2) when we are transitioning to hover we scale the non-tilted rotors by cos(angle). This pushes us towards lower fwd thrust

  We also apply an equalisation to the tilted motors in proportion to
  how much tilt we have. This smoothly reduces the impact of the roll
  gains as we tilt further forward.

  For yaw, we apply differential thrust in proportion to the demanded
  yaw control and sin of the tilt angle

  Finally we ensure no requested thrust is over 1 by scaling back all
  motors so the largest thrust is at most 1.0
 */
void Tiltrotor::tilt_compensate_angle(float *thrust, uint8_t num_motors, float non_tilted_mul, float tilted_mul)
{
    float tilt_total = 0;
    uint8_t tilt_count = 0;
    
    // apply tilt_factors first
    for (uint8_t i=0; i<num_motors; i++) {
        if (!is_motor_tilting(i)) {
            thrust[i] *= non_tilted_mul;
        } else {
            thrust[i] *= tilted_mul;
            tilt_total += thrust[i];
            tilt_count++;
        }
    }

    float largest_tilted = 0;
    const float sin_tilt = sinf(radians(current_tilt*90));
    // yaw_gain relates the amount of differential thrust we get from
    // tilt, so that the scaling of the yaw control is the same at any
    // tilt angle
    const float yaw_gain = sinf(radians(tilt_yaw_angle));
    const float avg_tilt_thrust = tilt_total / tilt_count;

    for (uint8_t i=0; i<num_motors; i++) {
        if (is_motor_tilting(i)) {
            // as we tilt we need to reduce the impact of the roll
            // controller. This simple method keeps the same average,
            // but moves us to no roll control as the angle increases
            thrust[i] = current_tilt * avg_tilt_thrust + thrust[i] * (1-current_tilt);
            // add in differential thrust for yaw control, scaled by tilt angle.
            // For partial-tilt vehicles (_have_vtol_motor), fade this term out as
            // _rear_yaw_blend rises: at blend=1 the rear _yaw_factor is already
            // zeroed, so allowing diff_thrust to run at full gain would re-inject
            // yaw via the roll_factor path and fight the vectored servo yaw.
            // Set Q_TILT_YAW_ANGLE=0 for this vehicle so yaw_gain=0 as well
            // (belt-and-suspenders); this scale handles the case where it is non-zero.
            const float blend_scale = _have_vtol_motor ? (1.0f - _rear_yaw_blend) : 1.0f;
            const float diff_thrust = motors->get_roll_factor(i) * (motors->get_yaw()+motors->get_yaw_ff()) * sin_tilt * yaw_gain * blend_scale;
            thrust[i] += diff_thrust;
            largest_tilted = MAX(largest_tilted, thrust[i]);
        }
    }

    // if we are saturating one of the motors then reduce all motors
    // to keep them in proportion to the original thrust. This helps
    // maintain stability when tilted at a large angle
    if (largest_tilted > 1.0f) {
        float scale = 1.0f / largest_tilted;
        for (uint8_t i=0; i<num_motors; i++) {
            thrust[i] *= scale;
        }
    }
}

/*
  choose up or down tilt compensation based on flight mode When going
  to a fixed wing mode we use tilt_compensate_down, when going to a
  VTOL mode we use tilt_compensate_up
 */
void Tiltrotor::tilt_compensate(float *thrust, uint8_t num_motors)
{
    if (current_tilt <= 0) {
        // the motors are not tilted, no compensation needed
        return;
    }

    // QTILTCRUISE computes tilt via arccos(hover_thr / pilot_thr), which already
    // guarantees the rear motors' vertical component equals hover thrust.
    // Applying additional scaling here would double-correct and fight altitude hold.
    if (plane.control_mode == &plane.mode_qtiltcruise) {
        return;
    }

    if (quadplane.in_vtol_mode()) {
        if (_have_vtol_motor) {
            // Partial-tilt vehicle: non-tilting (front) motors always point straight
            // up — no thrust reduction applies to them.  Tilting (rear) motors need
            // a 1/cos(θ) boost to maintain their vertical thrust component as they
            // tilt backward.  The upstream full-tilt path (cos on non-tilted, 1 on
            // tilted) is inverted for this configuration and must not be used.
            float inv_tilt_factor;
            if (current_tilt > 0.98f) {
                inv_tilt_factor = 1.0f / cosf(radians(0.98f * 90.0f));
            } else {
                inv_tilt_factor = 1.0f / cosf(radians(current_tilt * 90.0f));
            }
            tilt_compensate_angle(thrust, num_motors, 1.0f, inv_tilt_factor);
        } else {
            // Full-tilt vehicle: original upstream behavior.
            const float tilt_factor = cosf(radians(current_tilt*90));
            tilt_compensate_angle(thrust, num_motors, tilt_factor, 1);
        }
    } else {
        float inv_tilt_factor;
        if (current_tilt > 0.98f) {
            inv_tilt_factor = 1.0 / cosf(radians(0.98f*90));
        } else {
            inv_tilt_factor = 1.0 / cosf(radians(current_tilt*90));
        }
        tilt_compensate_angle(thrust, num_motors, 1, inv_tilt_factor);
    }
}

/*
  return true if the rotors are fully tilted forward
 */
bool Tiltrotor::fully_fwd(void) const
{
    if (!enabled() || (tilt_mask == 0)) {
        return false;
    }
    return (current_tilt >= get_fully_forward_tilt());
}

/*
  return true if the rotors are fully tilted up
 */
bool Tiltrotor::fully_up(void) const
{
    if (!enabled() || (tilt_mask == 0)) {
        return false;
    }
    return (current_tilt <= 0);
}

/*
  control vectoring for tilt multicopters
 */
void Tiltrotor::vectoring(void)
{
    // total angle the tilt can go through
    const float total_angle = 90 + tilt_yaw_angle + fixed_angle;
    // output value (0 to 1) to get motors pointed straight up
    const float zero_out = tilt_yaw_angle / total_angle;
    const float fixed_tilt_limit = fixed_angle / total_angle;
    const float level_out = 1.0 - fixed_tilt_limit;

    // calculate the basic tilt amount from current_tilt
    float base_output = zero_out + (current_tilt * (level_out - zero_out));
    // for testing when disarmed, apply vectored yaw in proportion to rudder stick
    // Wait TILT_DELAY_MS after disarming to allow props to spin down first.
    constexpr uint32_t TILT_DELAY_MS = 3000;
    uint32_t now = AP_HAL::millis();
    if (!plane.arming.is_armed_and_safety_off() && plane.quadplane.option_is_set(QuadPlane::Option::DISARMED_TILT)) {
        // this test is subject to wrapping at ~49 days, but the consequences are insignificant
        if ((now - hal.util->get_last_armed_change()) > TILT_DELAY_MS) {
            if (quadplane.in_vtol_mode()) {
                float yaw_out = plane.channel_rudder->get_control_in();
                yaw_out /= plane.channel_rudder->get_range();
                float yaw_range = zero_out;

                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  1000 * constrain_float(base_output + yaw_out * yaw_range,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 1000 * constrain_float(base_output - yaw_out * yaw_range,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear,  1000 * constrain_float(base_output,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft,  1000 * constrain_float(base_output + yaw_out * yaw_range,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight, 1000 * constrain_float(base_output - yaw_out * yaw_range,0,1));
            } else {
                // fixed wing tilt
                const float gain = fixed_gain * fixed_tilt_limit;
                // base the tilt on elevon mixing, which means it
                // takes account of the MIXING_GAIN. The rear tilt is
                // based on elevator
                const float right = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_right) * (1/4500.0);
                const float left  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_left) * (1/4500.0);
                const float mid  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) * (1/4500.0);
                // front tilt is effective canards, so need to swap and use negative. Rear motors are treated live elevons.
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,1000 * constrain_float(base_output - right,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight,1000 * constrain_float(base_output - left,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft,1000 * constrain_float(base_output + left,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight,1000 * constrain_float(base_output + right,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear,  1000 * constrain_float(base_output + mid,0,1));
            }
        }
        return;
    }

    const bool no_yaw = tilt_over_max_angle();
    if (no_yaw) {
        // fixed wing  We need to apply inverse scaling with throttle, and remove the surface speed scaling as
        // we don't want tilt impacted by airspeed
        const float scaler = plane.control_mode == &plane.mode_manual?1:(quadplane.FW_vector_throttle_scaling() / plane.get_speed_scaler());
        const float gain = fixed_gain * fixed_tilt_limit * scaler;
        const float right = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_right) * (1/4500.0);
        const float left  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_left) * (1/4500.0);
        const float mid  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) * (1/4500.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,1000 * constrain_float(base_output - right,0,1));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight,1000 * constrain_float(base_output - left,0,1));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft,1000 * constrain_float(base_output + left,0,1));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight,1000 * constrain_float(base_output + right,0,1));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear,  1000 * constrain_float(base_output + mid,0,1));
    } else {
        const float yaw_out = motors->get_yaw()+motors->get_yaw_ff();
        const float roll_out = motors->get_roll()+motors->get_roll_ff();
        const float yaw_range = zero_out;

        // Scaling yaw with throttle
        const float throttle = motors->get_throttle_out();
        const float scale_min = 0.5;
        const float scale_max = 2.0;
        float throttle_scaler = scale_max;
        if (is_positive(throttle)) {
            throttle_scaler = constrain_float(motors->get_throttle_hover() / throttle, scale_min, scale_max);
        }

        // now apply vectored thrust for yaw and roll.
        const float tilt_rad = radians(current_tilt*90);
        const float sin_tilt = sinf(tilt_rad);
        const float cos_tilt = cosf(tilt_rad);
        // the MotorsMatrix library normalises roll factor to 0.5, so
        // we need to use the same factor here to keep the same roll
        // gains when tilted as we have when not tilted
        const float avg_roll_factor = 0.5;
        float tilt_scale = throttle_scaler * yaw_out * cos_tilt + avg_roll_factor * roll_out * sin_tilt;

        if (fabsf(tilt_scale) > 1.0) {
            tilt_scale = constrain_float(tilt_scale, -1.0, 1.0);
            motors->limit.yaw = true;
        }

        const float tilt_offset = tilt_scale * yaw_range;

        float left_tilt = base_output + tilt_offset;
        float right_tilt = base_output - tilt_offset;

        // if output saturation of both left and right then set yaw limit flag
        if (((left_tilt > 1.0) || (left_tilt < 0.0)) &&
            ((right_tilt > 1.0) || (right_tilt < 0.0))) {
            motors->limit.yaw = true;
        }

        // constrain and scale to output range
        left_tilt = constrain_float(left_tilt,0.0,1.0) * 1000.0;
        right_tilt = constrain_float(right_tilt,0.0,1.0) * 1000.0;

        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, left_tilt);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, right_tilt);

        // Rear servos blend between base_output (no differential, full torque yaw from _yaw_factor)
        // and full vectored differential. At blend=0 (hover): no differential, yaw via motor RPM.
        // At blend=1 (cruise): full differential, _yaw_factor zeroed, servos do all the yaw work.
        const float base_scaled = 1000.0f * constrain_float(base_output, 0.0f, 1.0f);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, base_scaled);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft,
            base_scaled * (1.0f - _rear_yaw_blend) + left_tilt  * _rear_yaw_blend);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight,
            base_scaled * (1.0f - _rear_yaw_blend) + right_tilt * _rear_yaw_blend);
    }
}

/*
  control bicopter tiltrotors
 */
void Tiltrotor::bicopter_output(void)
{
    if (type != TILT_TYPE_BICOPTER || quadplane.motor_test.running) {
        // don't override motor test with motors_output
        return;
    }

    if (!quadplane.in_vtol_mode() && fully_fwd()) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  -SERVO_MAX);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, -SERVO_MAX);
        return;
    }

    float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    if (quadplane.assisted_flight) {
        quadplane.hold_stabilize(throttle * 0.01f);
        quadplane.motors_output(true);
    } else {
        quadplane.motors_output(false);
    }

    // bicopter assumes that trim is up so we scale down so match
    float tilt_left = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft);
    float tilt_right = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight);

    if (is_negative(tilt_left)) {
        tilt_left *= tilt_yaw_angle * (1/90.0);
    }
    if (is_negative(tilt_right)) {
        tilt_right *= tilt_yaw_angle * (1/90.0);
    }

    // reduce authority of bicopter as motors are tilted forwards
    const float scaling = cosf(current_tilt * M_PI_2);
    tilt_left  *= scaling;
    tilt_right *= scaling;

    // add current tilt and constrain
    tilt_left  = constrain_float(-(current_tilt * SERVO_MAX) + tilt_left,  -SERVO_MAX, SERVO_MAX);
    tilt_right = constrain_float(-(current_tilt * SERVO_MAX) + tilt_right, -SERVO_MAX, SERVO_MAX);

    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  tilt_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
}

/*
  when doing a forward transition of a tilt-vectored quadplane we use
  euler angle control to maintain good yaw. This updates the yaw
  target based on pilot input and target roll
 */
void Tiltrotor::update_yaw_target(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - transition_yaw_set_ms > 100 ||
        !is_zero(quadplane.get_pilot_input_yaw_rate_cds())) {
        // lock initial yaw when transition is started or when
        // pilot commands a yaw change. This allows us to track
        // straight in transitions for tilt-vectored planes, but
        // allows for turns when level transition is not wanted
        transition_yaw_cd = quadplane.ahrs.yaw_sensor;
    }

    /*
      now calculate the equivalent yaw rate for a coordinated turn for
      the desired bank angle given the airspeed
     */
    float aspeed;
    bool have_airspeed = quadplane.ahrs.airspeed_EAS(aspeed);
    if (have_airspeed && labs(plane.nav_roll_cd)>1000) {
        float dt = (now - transition_yaw_set_ms) * 0.001;
        // calculate the yaw rate to achieve the desired turn rate
        const float airspeed_min = MAX(plane.aparm.airspeed_min,5);
        const float yaw_rate_cds = fixedwing_turn_rate(plane.nav_roll_cd*0.01, MAX(aspeed,airspeed_min))*100;
        transition_yaw_cd += yaw_rate_cds * dt;
    }
    transition_yaw_set_ms = now;
}


/*
  control use of multirotor rate control in forward transition
 */
bool Tiltrotor_Transition::use_multirotor_control_in_fwd_transition() const
{
    if (!tiltrotor.is_vectored()) {
        return false;
    }
    switch (transition_state) {
    case State::AIRSPEED_WAIT:
    case State::TIMER:
        return true;
    case State::DONE:
        return false;
    }
    return false;
}

bool Tiltrotor_Transition::update_yaw_target(float& yaw_target_cd)
{
    if (!use_multirotor_control_in_fwd_transition()) {
        return false;
    }
    tiltrotor.update_yaw_target();
    yaw_target_cd = tiltrotor.transition_yaw_cd;
    return true;
}

// return true if we should show VTOL view
bool Tiltrotor_Transition::show_vtol_view() const
{
    bool show_vtol = quadplane.in_vtol_mode();

    if (!show_vtol && tiltrotor.is_vectored() && transition_state <= State::TIMER) {
        // we use multirotor controls during fwd transition for
        // vectored yaw vehicles
        return true;
    }

    return show_vtol;
}

// Return true if forward throttle should be allowed for position control, see Q_FWD_THR_USE
bool Tiltrotor_Transition::allow_vfwd() const
{
    // Don't allow forward throttle (tilt) if a tilting motor has failed which would result in a yaw imbalance

    // No resultant yaw imbalance if not vectored
    if (!tiltrotor.is_vectored()) {
        return true;
    }

    // No failed motor
    if (!motors->get_thrust_boost()) {
        return true;
    }

    // Get index of failed motor
    const uint8_t lost_motor = motors->get_lost_motor();

    // Failed motor is not tilting
    if (!tiltrotor.is_motor_tilting(lost_motor)) {
        return true;
    }

    // Failed tilting motor is on the center line, so will not cause a yaw imbalance
    if (is_zero(motors->get_roll_factor(lost_motor))) {
        return true;
    }

    // Disable forward throttle (tilt)
    return false;
}

// return true if we are tilted over the max angle threshold
bool Tiltrotor::tilt_over_max_angle(void) const
{
    const float tilt_threshold = (max_angle_deg/90.0f);
    return (current_tilt > MIN(tilt_threshold, get_forward_flight_tilt()));
}

// throttle of forward flight motors including any tilting motors
bool Tiltrotor::get_forward_throttle(float &throttle) const
{
    if (!enabled() || !_is_vectored) {
        return false;
    }
    const float throttle_range = motors->thr_lin.get_spin_max() - motors->thr_lin.get_spin_min();
    if (!is_positive(throttle_range)) {
        return false;
    }
    float throttle_sum = 0.0f;
    uint8_t num_vectored_motors = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        if (is_motor_tilting(i)) {
            float thrust;
            if (motors->get_thrust(i, thrust)) {
                throttle_sum += (motors->thr_lin.thrust_to_actuator(thrust) - motors->thr_lin.get_spin_min()) / throttle_range;
                num_vectored_motors ++;
            }
        }
    }
    if (num_vectored_motors > 0) {
        throttle = throttle_sum / (float)num_vectored_motors;
        return true;
    }
    return false;
}

#endif  // HAL_QUADPLANE_ENABLED
