// ****************************************************************************
// NOTICE
//
// This is the copyright work of The MITRE Corporation, and was produced
// for the U. S. Government under Contract Number DTFAWA-10-C-00080, and
// is subject to Federal Aviation Administration Acquisition Management
// System Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV
// (Oct. 1996).  No other use other than that granted to the U. S.
// Government, or to those acting on behalf of the U. S. Government,
// under that Clause is authorized without the express written
// permission of The MITRE Corporation. For further information, please
// contact The MITRE Corporation, Contracts Office, 7515 Colshire Drive,
// McLean, VA  22102-7539, (703) 983-6000. 
//
// Copyright 2017 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once


#include "aaesim/BadaWithCalc.h"
#include "public/Guidance.h"
#include "public/ControlCommands.h"
#include "public/EquationsOfMotionState.h"
#include "public/TangentPlaneSequence.h"
#include "public/WindStack.h"

/**
 * This class is really intended to be sub-classes to allow specialized behavior and gain values.
 */
class AircraftControl {

public:
    AircraftControl() {
        speedBrakeGain = 0.0;
        aircraftPerformance = 0;
    }

    const Units::Frequency &getAltGain() const {
        return altGain;
    }

    const Units::Frequency &getGammaGain() const {
        return gammaGain;
    }

    const Units::Frequency &getPhiGain() const {
        return phiGain;
    }

    double getSpeedBrakeGain() const {
        return speedBrakeGain;
    }

    const Units::Frequency &getThrustGain() const {
        return thrustGain;
    }

    /**
     * This should be reimplemented in a subclass.
     */
    virtual ControlCommands calculateControlCommands(const Guidance &guidance, const EquationsOfMotionState& eqmState, const WindStack &wind_x, const WindStack &wind_y) {
        const Units::Angle phi = Units::ZERO_ANGLE;
        const Units::Force thrust = Units::ZERO_FORCE;
        const Units::Angle gamma = Units::ZERO_ANGLE;
        const Units::Speed trueAirspeed = Units::ZERO_SPEED;
        const double speedBrake = 0;
        const int flapMode = 0;
        return ControlCommands(phi,thrust,gamma,trueAirspeed,speedBrake,flapMode);
    };

    virtual void init(BadaWithCalc &aircraftPerformance, const Units::Length &altAtFAF, const Units::Angle &mMaxBankAngle, const PrecalcWaypoint &finalWaypoint);

protected:
    /**
     * Use this to estimate the kinetic forces of lift and drag on the aircraft
     * in it's current state.
     */
    void estimateKineticForces(const EquationsOfMotionState& eqmState, Units::Force& lift, Units::Force& drag, int& newFlapConfiguration);
    virtual Units::Angle doLateralControl(const Guidance& guidance,
                                          const EquationsOfMotionState& eqmState);
    virtual void doVerticalControl(const Guidance& guidance,
                                   const EquationsOfMotionState& eqmState,
                                   Units::Force& T_com,
                                   Units::Angle& gamma_com,
                                   Units::Speed& tas_com,
                                   double& speedBrakeCom,
                                   int& newFlapConfig){};
    void calculateSensedWind(const WindStack &wind_x, const WindStack &wind_y, const Units::MetersLength &altitude);
    Units::Frequency calculateThrustGain();

    Units::Mass ac_mass;
    Units::Area wing_area;
    Units::Frequency altGain, gammaGain, phiGain, thrustGain, naturalFrequency;
    Units::Speed Vwx, Vwy;
    Units::Frequency dVwx_dh, dVwy_dh;
    double speedBrakeGain;
    Units::Angle mMaxBankAngle; // Maximum bank angle for dynamics and speed_on_pitch_control_dynamics calculations (parameter max_bank_angle)
    Units::Length mAltAtFAF;
    BadaWithCalc *aircraftPerformance;
    PrecalcWaypoint mFinalWaypoint; // the last waypoint on the planned route

private:
    static log4cplus::Logger logger;
};



