/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_Param/AP_Param.h>
#include "transition.h"
#include <AP_Logger/LogStructure.h>

class QuadPlane;
class AP_MotorsMulticopter;
class Tiltrotor_Transition;
class Tiltrotor
{
friend class QuadPlane;
friend class Plane;
friend class Tiltrotor_Transition;
public:

    Tiltrotor(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors);

    bool enabled() const { return (enable > 0) && setup_complete;}

    void setup();

    void slew(float tilt);
    void binary_slew(bool forward);
    void update();
    void continuous_update();
    void binary_update();
    void vectoring();
    void bicopter_output();
    void tilt_compensate_angle(float *thrust, uint8_t num_motors, float non_tilted_mul, float tilted_mul);
    void tilt_compensate(float *thrust, uint8_t num_motors);
    bool tilt_over_max_angle(void) const;

    bool is_motor_tilting(uint8_t motor) const {
        return tilt_mask.get() & (1U<<motor);
    }

    bool fully_fwd() const;
    bool fully_up() const;
    float tilt_max_change(bool up, bool in_flap_range = false) const;
    float get_fully_forward_tilt() const;
    float get_forward_flight_tilt() const;

    // update yaw target for tiltrotor transition
    void update_yaw_target();

    bool is_vectored() const { return enabled() && _is_vectored; }

    bool has_fw_motor() const { return _have_fw_motor; }

    bool has_vtol_motor() const { return _have_vtol_motor; }

    bool motors_active() const { return enabled() && _motors_active; }

    // true if the tilts have completed slewing
    // always return true if not enabled or not a continuous type
    bool tilt_angle_achieved() const { return !enabled() || (type != TILT_TYPE_CONTINUOUS) || angle_achieved; }

    // throttle of tilting motors used for forward flight
    bool get_forward_throttle(float &throttle) const;

    // Write tiltrotor specific log
    void write_log();

    AP_Int8 enable;
    AP_Int16 tilt_mask;
    AP_Int16 max_rate_up_dps;
    AP_Int16 max_rate_down_dps;
    AP_Int8  max_angle_deg;
    AP_Int8  type;
    AP_Float tilt_yaw_angle;
    AP_Float fixed_angle;
    AP_Float fixed_gain;
    AP_Float flap_angle_deg;
    AP_Float yaw_bld_thr;   // Q_TILT_YAW_THR — tilt angle (deg) where torque→vectored yaw blend starts
    AP_Float yaw_bld_rng;   // Q_TILT_YAW_RNG — tilt range (deg) to complete the blend
    AP_Float ctrim_deg;     // Q_TILT_CT_PTRIM — nose-up pitch (deg) at full tilt for CT-shift compensation
    AP_Int8  cruise_inv;    // Q_TILT_CSINV   — 0: forward stick engages tilt-cruise, 1: back stick engages
    AP_Int16 cruise_dz;     // Q_TILT_CSDZ    — stick dead zone (cd) for cruise sub-mode entry, default 200

    // TC-submode shadow params — active only when QTILTCRUISE cruise sub-mode is engaged.
    // Sentinel -1 means "inherit from the corresponding base param".
    AP_Int8  tc_max_angle_deg;     // Q_TILT_TC_MAX    — max tilt angle (deg) in TC; -1→max_angle_deg
    AP_Int16 tc_max_rate_up_dps;   // Q_TILT_TC_RATEUP — tilt-back rate (deg/s) in TC; -1→max_rate_up_dps
    AP_Int16 tc_max_rate_down_dps; // Q_TILT_TC_RATEDN — tilt-forward rate (deg/s) in TC; -1→max_rate_down_dps
    AP_Float tc_yaw_bld_thr;       // Q_TILT_TC_YTHR   — yaw-blend start angle (deg) in TC; -1→yaw_bld_thr
    AP_Float tc_yaw_bld_rng;       // Q_TILT_TC_YRNG   — yaw-blend range (deg) in TC; -1→yaw_bld_rng
    AP_Float tc_pilot_spd_up_ms;   // Q_TILT_TC_SPDUP  — Z-ctrl climb rate (m/s) in TC; -1→Q_PILOT_SPD_UP
    AP_Float tc_pilot_spd_dn_ms;   // Q_TILT_TC_SPDDN  — Z-ctrl descent rate (m/s) in TC; -1→Q_PILOT_SPD_DN
    AP_Float tc_pilot_accel_mss;   // Q_TILT_TC_ACCZ   — Z-ctrl accel (m/s²) in TC; -1→Q_PILOT_ACCEL_Z

    // Effective TC Z-controller limit getters (inherit from quad params when sentinel -1)
    float get_tc_pilot_spd_up_ms() const;
    float get_tc_pilot_spd_dn_ms() const;
    float get_tc_pilot_accel_mss() const;

    float current_tilt;
    float current_throttle;
    bool _motors_active;
    float transition_yaw_cd;
    uint32_t transition_yaw_set_ms;
    bool _is_vectored;

    // types of tilt mechanisms
    enum {TILT_TYPE_CONTINUOUS    =0,
          TILT_TYPE_BINARY        =1,
          TILT_TYPE_VECTORED_YAW  =2,
          TILT_TYPE_BICOPTER      =3
    };

    static const struct AP_Param::GroupInfo var_info[];

private:

    // Tiltrotor specific log message
    struct PACKED log_tiltrotor {
        LOG_PACKET_HEADER;
        uint64_t time_us;
        float current_tilt;
        float front_left_tilt;
        float front_right_tilt;
        float rear_yaw_blend;
    };

    float _rear_yaw_blend;  // 0=full torque yaw (hover), 1=full vectored yaw (cruise)

    // true when QTILTCRUISE cruise sub-mode is active; gates TC-param overrides
    bool _in_tc_cruise;

    // update rear motor yaw factors and _rear_yaw_blend based on current_tilt
    void update_yaw_blend(void);

    bool setup_complete;

    // true if a fixed forward motor is setup
    bool _have_fw_motor;

    // true if at least one enabled motor is NOT in Q_TILT_MASK (a fixed permanent VTOL motor)
    bool _have_vtol_motor;

    // true if the current tilt angle is equal to the desired
    // with slow tilt rates the tilt angle can lag
    bool angle_achieved;

    // references for convenience
    QuadPlane& quadplane;
    AP_MotorsMulticopter*& motors;

    Tiltrotor_Transition* transition;

};

// Transition for separate left thrust quadplanes
class Tiltrotor_Transition : public SLT_Transition
{
friend class Tiltrotor;
public:

    Tiltrotor_Transition(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors, Tiltrotor& _tiltrotor):SLT_Transition(_quadplane, _motors), tiltrotor(_tiltrotor) {};

    bool update_yaw_target(float& yaw_target_cd) override;

    bool show_vtol_view() const override;

    bool use_multirotor_control_in_fwd_transition() const override;

    // Return true if forward throttle should be allowed for position control, see Q_FWD_THR_USE
    virtual bool allow_vfwd() const override;

private:

    Tiltrotor& tiltrotor;

};
