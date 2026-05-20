#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

/*
  QTILTCRUISE flight mode — altitude-hold tilt-cruise for heavy-lift push/pull VTOL.

  Control law:
    Cruise sub-mode engaged (pitch stick forward by default; back when Q_TILT_CSINV=1):
      Altitude is held automatically via the Z-axis controller (zero climb rate
      target). The tilt servos are simultaneously driven to the angle at which
      the vertical thrust component equals the estimated hover thrust, so
      altitude stays constant regardless of how the pilot varies throttle.
      More throttle → more forward tilt → more forward speed, same altitude.
      Roll, yaw, and throttle all work normally.

    Cruise sub-mode not engaged (stick not past the threshold):
      No altitude control. Throttle is passed directly to the motors (identical
      to QSTABILIZE). Tilt servos slew back to vertical at Q_TILT_RATE_UP deg/s.
      The aircraft behaves like a plain quadcopter — pilot has full manual
      control of altitude via throttle.

  Tilt angle is computed inside Tiltrotor::continuous_update() (tiltrotor.cpp)
  so that servo slew-rate limiting, vectored-yaw, and Q_TILT_MAX clamping all
  continue to apply unchanged.
*/

bool ModeQTiltCruise::_enter()
{
    // Deny entry if airspeed is too high for safe VTOL attitude control.
    // This prevents switching to QTILTCRUISE from a fixed-wing mode (FBWA, CRUISE, etc.)
    // at cruise speeds, which would hand authority to the multirotor controller while
    // flying fast — a recipe for catastrophic attitude divergence.
    // Use Q_ASSIST_SPEED as the upper bound: it already defines the VTOL/FW crossover
    // speed for this vehicle. Floor at 10 m/s in case Q_ASSIST_SPEED is not set.
    float aspeed_ms;
    if (plane.ahrs.airspeed_EAS(aspeed_ms)) {
        const float max_entry_ms = MAX(quadplane.assist.speed.get(), 10.0f);
        if (aspeed_ms > max_entry_ms) {
            gcs().send_text(MAV_SEVERITY_WARNING,
                "QTiltCruise: airspeed %.1f m/s > %.1f limit, mode denied",
                (double)aspeed_ms, (double)max_entry_ms);
            return false;
        }
    }
    // If no airspeed sensor is fitted, allow entry (rely on pilot to only switch
    // from hover modes). With a sensor, entry is denied above the threshold.

    // Snapshot current yaw PID coefficients so they can be restored when exiting
    // TC sub-mode or leaving the mode entirely.  Done here (before any TC swap)
    // so we always capture the true unmodified base values.
    {
        AC_PID& ryaw = attitude_control->get_rate_yaw_pid();
        _saved_yaw_p      = ryaw.kP().get();
        _saved_yaw_i      = ryaw.kI().get();
        _saved_yaw_d      = ryaw.kD().get();
        _saved_yaw_ff     = ryaw.ff().get();
        _saved_ang_yaw_p  = attitude_control->get_angle_yaw_p().kP().get();
    }
    _prev_in_tc_cruise = false;

    // Initialise Z-axis controller limits for the altitude-hold (cruise) sub-mode.
    // Use TC-specific params when configured (Q_TILT_TC_SPDUP / _SPDDN / _ACCZ),
    // falling back to the quad params when they are left at their default of -1.
    const float tc_spd_up  = quadplane.tiltrotor.get_tc_pilot_spd_up_ms();
    const float tc_spd_dn  = quadplane.tiltrotor.get_tc_pilot_spd_dn_ms();
    const float tc_accel   = quadplane.tiltrotor.get_tc_pilot_accel_mss();
    pos_control->D_set_max_speed_accel_m(tc_spd_dn, tc_spd_up, tc_accel);
    pos_control->D_set_correction_speed_accel_m(tc_spd_dn, tc_spd_up, tc_accel);
    // Relax the Z controller so it is marked inactive.  The first TC
    // sub-mode engagement calls init_z_for_altitude_hold() which re-inits
    // with _vel_desired=0 for a spike-free bumpless transfer.
    pos_control->D_relax_controller(quadplane.motors->get_throttle_hover());
    quadplane.init_throttle_wait();
    return true;
}

void ModeQTiltCruise::update()
{
    // QSTABILIZE sets nav_roll_cd from the roll stick (correct — keep it) and
    // nav_pitch_cd from the pitch stick (wrong in cruise sub-mode — see below).
    plane.mode_qstabilize.update();

    // In cruise sub-mode the stick only gates entry; it must not simultaneously
    // command a nose-down attitude. Override nav_pitch_cd so the aircraft holds
    // level pitch (plus CT-shift trim) while the Z-PID handles altitude.
    // CT-shift trim: as rear motors tilt backward the centre of thrust shifts
    // rearward, producing a nose-down pitch moment.  Q_TILT_CT_PTRIM (deg at full
    // tilt) is applied linearly with current_tilt to trim the nose up.  Default 0.
    // In quad sub-mode leave nav_pitch_cd from QSTABILIZE so the pilot retains
    // full attitude control via the stick.
    const bool cruise_inverted = (quadplane.tiltrotor.cruise_inv.get() != 0);
    const float pitch_ctl = plane.channel_pitch->get_control_in();
    const float cruise_dz = MAX((float)quadplane.tiltrotor.cruise_dz.get(), 50.0f);
    const bool in_cruise_submode = cruise_inverted ? (pitch_ctl < -cruise_dz) : (pitch_ctl > cruise_dz);
    if (in_cruise_submode) {
        const float trim_cd = quadplane.tiltrotor.ctrim_deg.get()
                              * quadplane.tiltrotor.current_tilt * 100.0f;
        plane.nav_pitch_cd = (int32_t)trim_cd;
    }
}

void ModeQTiltCruise::run()
{
    // Allow VTOL recovery assist
    quadplane.assist.check_VTOL_recovery();

    const uint32_t now = AP_HAL::millis();
    if (quadplane.tailsitter.in_vtol_transition(now)) {
        // Tailsitters in FW pull-up run fixed-wing controllers
        Mode::run();
        return;
    }

    if (quadplane.throttle_wait) {
        // Pilot has not yet raised throttle from minimum — stay on ground.
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, 0);
        quadplane.relax_attitude_control();
        pos_control->D_relax_controller(0);
    } else {
        // Read pitch stick: range is -4500 to +4500 centidegrees of stick travel.
        // Q_TILT_CSDZ (default 200 cd, ~4.4% travel) prevents RC noise / trim
        // offset from rapidly toggling between sub-modes at stick centre.
        const float pitch_in = plane.channel_pitch->get_control_in();
        const bool cruise_inverted = (quadplane.tiltrotor.cruise_inv.get() != 0);
        const float cruise_dz = MAX((float)quadplane.tiltrotor.cruise_dz.get(), 50.0f);
        const bool in_cruise_submode = cruise_inverted ? (pitch_in < -cruise_dz) : (pitch_in > cruise_dz);

        // assign_tilt_to_fwd_thr() zeroes q_fwd_throttle when not in a
        // position-control forward-thrust mode.  Call it here, inside the
        // throttle_wait guard, so it only runs when motors are active —
        // consistent with ModeQHover::run().
        quadplane.assign_tilt_to_fwd_thr();

        // Detect submode transition and swap yaw PIDs on the boundary only.
        // steady-state cost: one bool compare per cycle.
        if (in_cruise_submode && !_prev_in_tc_cruise) {
            quadplane.init_z_for_altitude_hold();
            _apply_tc_yaw_pids();
        } else if (!in_cruise_submode && _prev_in_tc_cruise) {
            _restore_yaw_pids();
        }
        _prev_in_tc_cruise = in_cruise_submode;

        if (in_cruise_submode) {
            // -----------------------------------------------------------
            // TILT-CRUISE sub-mode
            // Altitude is held at zero climb rate by the Z-axis PID
            // controller. Tilt scheduling (arccos formula based on pilot
            // throttle) runs in Tiltrotor::continuous_update(). Z-PID
            // corrections act on actual motor throttle independently.
            // -----------------------------------------------------------
            // Prevent heading-error accumulation while vectored-yaw authority
            // is limited at high tilt angles.  When the pilot is not actively
            // commanding yaw (rudder near centre), snap the attitude target's
            // yaw to the current heading so angle-P never builds a stale
            // heading error that would overwhelm the available servo headroom.
            // reset_yaw_target_and_rate(false) is a pure z-axis quaternion
            // rotation — it does not touch roll/pitch targets or I terms.
            if (fabsf(plane.channel_rudder->get_control_in()) < cruise_dz) {
                attitude_control->reset_yaw_target_and_rate(false);
            }
            quadplane.hold_hover(0.0f);
        } else {
            // -----------------------------------------------------------
            // QUAD sub-mode: cruise stick threshold not met.
            // No altitude control — throttle goes straight to motors,
            // exactly like QSTABILIZE. Tiltrotor::continuous_update()
            // slews servos back to vertical independently.
            //
            // Relax the Z-axis controller so it carries no stale
            // integrator state into the next tilt-cruise engagement.
            // Without this, a rapid stick-back/stick-forward cycle would
            // resume hold_hover() with accumulated position error and
            // produce a sudden throttle spike.
            // -----------------------------------------------------------
            pos_control->D_relax_controller(quadplane.motors->get_throttle_hover());
            quadplane.hold_stabilize(quadplane.get_pilot_throttle());
        }
    }

    // Stabilise with fixed-wing surfaces (ailerons, elevator).
    // Always runs — including during throttle_wait — to keep surfaces neutral.
    plane.stabilize_roll();
    plane.stabilize_pitch();

    // Centre rudder; yaw via differential tilt or motor torque
    output_rudder_and_steering(0.0f);

    // Spin-recovery assist if applicable
    quadplane.assist.output_spin_recovery();
}

void ModeQTiltCruise::_exit()
{
    // Restore base yaw PIDs if we left the mode while TC sub-mode was active.
    // Guards other VTOL modes from inheriting TC-tuned gains.
    if (_prev_in_tc_cruise) {
        _restore_yaw_pids();
        _prev_in_tc_cruise = false;
    }
}

// Apply TC-specific yaw PID overrides.
// Only terms whose TC shadow param is ≥ 0 are changed; the rest stay at
// their saved base values (already live in the controller).
// The yaw rate I-term is reset to prevent integrator artefacts from the
// previous (hover) sub-mode feeding into cruise yaw authority.
void ModeQTiltCruise::_apply_tc_yaw_pids()
{
    const auto& tp = quadplane.tiltrotor;
    AC_PID& ryaw = attitude_control->get_rate_yaw_pid();
    if (tp.tc_yaw_rate_p.get()  >= 0.0f) { ryaw.set_kP(tp.tc_yaw_rate_p.get()); }
    if (tp.tc_yaw_rate_i.get()  >= 0.0f) { ryaw.set_kI(tp.tc_yaw_rate_i.get()); }
    if (tp.tc_yaw_rate_d.get()  >= 0.0f) { ryaw.set_kD(tp.tc_yaw_rate_d.get()); }
    if (tp.tc_yaw_rate_ff.get() >= 0.0f) { ryaw.set_ff(tp.tc_yaw_rate_ff.get()); }
    if (tp.tc_ang_yaw_p.get()   >= 0.0f) {
        attitude_control->get_angle_yaw_p().set_kP(tp.tc_ang_yaw_p.get());
    }
    ryaw.reset_I();
}

// Restore base yaw PID values saved at mode entry.
// All five terms are unconditionally restored so the controller is guaranteed
// to match its pre-TC state regardless of which TC params were set.
// The yaw rate I-term is reset to prevent TC integrator state from carrying
// into the next (hover) sub-mode.
void ModeQTiltCruise::_restore_yaw_pids()
{
    AC_PID& ryaw = attitude_control->get_rate_yaw_pid();
    ryaw.set_kP(_saved_yaw_p);
    ryaw.set_kI(_saved_yaw_i);
    ryaw.set_kD(_saved_yaw_d);
    ryaw.set_ff(_saved_yaw_ff);
    attitude_control->get_angle_yaw_p().set_kP(_saved_ang_yaw_p);
    ryaw.reset_I();
}

#endif  // HAL_QUADPLANE_ENABLED
