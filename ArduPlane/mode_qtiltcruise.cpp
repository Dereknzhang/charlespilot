#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

/*
  QTILTCRUISE flight mode — altitude-hold tilt-cruise for heavy-lift push/pull VTOL.

  Control law:
    RC pitch stick > 0 (forward of centre):
      Altitude is held automatically via the Z-axis controller (zero climb rate
      target). The tilt servos are simultaneously driven to the angle at which
      the vertical thrust component equals the estimated hover thrust, so
      altitude stays constant regardless of how the pilot varies throttle.
      More throttle → more forward tilt → more forward speed, same altitude.
      Roll, yaw, and throttle all work normally.

    RC pitch stick <= 0 (at or behind centre):
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
    // Initialise Z-axis controller limits for the altitude-hold (forward-pitch)
    // sub-mode. These are only active when pitch stick > 0.
    pos_control->D_set_max_speed_accel_m(
        quadplane.get_pilot_velocity_z_max_dn_m(),
        quadplane.pilot_speed_z_max_up_ms,
        quadplane.pilot_accel_z_mss);
    pos_control->D_set_correction_speed_accel_m(
        quadplane.get_pilot_velocity_z_max_dn_m(),
        quadplane.pilot_speed_z_max_up_ms,
        quadplane.pilot_accel_z_mss);
    quadplane.set_climb_rate_ms(0);
    quadplane.init_throttle_wait();
    return true;
}

void ModeQTiltCruise::update()
{
    // Reuse QHOVER / QSTABILIZE roll-pitch target logic
    plane.mode_qstabilize.update();
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
        // Positive = pilot pushing stick forward (forward-flight intent).
        const float pitch_in = plane.channel_pitch->get_control_in();

        // assign_tilt_to_fwd_thr() zeroes q_fwd_throttle when not in a
        // position-control forward-thrust mode.  Call it here, inside the
        // throttle_wait guard, so it only runs when motors are active —
        // consistent with ModeQHover::run().
        quadplane.assign_tilt_to_fwd_thr();

        if (pitch_in > 0) {
            // -----------------------------------------------------------
            // TILT-CRUISE sub-mode
            // Altitude is held at zero climb rate by the Z-axis PID
            // controller. Tilt scheduling (arccos formula based on pilot
            // throttle) runs in Tiltrotor::continuous_update(). Z-PID
            // corrections act on actual motor throttle independently.
            // -----------------------------------------------------------
            quadplane.hold_hover(0.0f);
        } else {
            // -----------------------------------------------------------
            // QUAD sub-mode: pitch stick at or behind centre.
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
            pos_control->D_relax_controller(0);
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

#endif  // HAL_QUADPLANE_ENABLED
