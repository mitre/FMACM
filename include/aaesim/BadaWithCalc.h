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
// Copyright 2018 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once
#include <string>

#include "Bada.h"

class BadaWithCalc : public Bada
{
public:
    // Set enum values for different modes
    enum FlapConfiguration // AAES-771 use this enum fully
    {
        CRUISE = 0,
        APPROACH = 1,
        LANDING = 2,
        GEAR_DOWN = 3
    };

    // Speeds and values used for flaps speed calculations.
    struct FlapSpeeds
    {
        Units::Speed VappMin;  // Target approach speed
        Units::Speed VappMax;
        Units::Speed VlndMin;
        Units::Speed VlndMax;
        Units::Speed VgearMin;
        Units::Speed VgearMax;
    };


    BadaWithCalc();
    virtual ~BadaWithCalc();

    /**
     * getAircraftParameters() does two things:
     *
     * 1.  Loads the data files by calling init(), if not already done.
     * 2.  Sets ac_mass based on the percentile.
     */
    void getAircraftParameters(std::string aircraft_type,     /** Aircraft type to be passed to init() */
                               double      mass_percentile); /** Mass percentile: 0.0=mass.m_min, 1.0=mass.m_max */

    /**
     * getConfigTrajGen() calculates drag coefficients during trajectory pregeneration.
     */
    void getConfigTrajGen(const Units::Speed&  airspeed,     /** [in] Calibrated airspeed in knots */
                          const Units::Length& altitude,             /** [in] Altitude in meters */
                          const Units::Length& altitude_faf, /** [in] Altitude at final approach fix */
                          double &cd0,            /** [out] parasitic drag coefficient */
                          double &cd2,            /** [out] induced drag coefficient */
                          double &gear,           /** [out] landing gear drag coefficient */
                          int &mode) const;             /** [out] flap configuration (0-3): 0=cruise, 1=approach, 2=landing, 3=gear down */

    /**
     * getConfig() calculates drag coefficients and increments flap
     * configuration mode when appropriate.
     */
    void getConfig(const Units::Speed& airspeed, /** [in] Calibrated airspeed in knots */
                   const Units::Length& altitude,         /** [in] Altitude in meters */
                   const Units::Length& altitude_faf, /** [in] Altitude at final approach fix */
                   int modeLast,       /** [in] previous flap configuration (0-3) */
                   double &cd0,        /** [out] parasitic drag coefficient */
                   double &cd2,        /** [out] induced drag coefficient */
                   double &gear,       /** [out] landing gear drag coefficient */
                   int &mode) const;         /** [out] flap configuration (0-3): 0=cruise, 1=approach, 2=landing, 3=gear down */

    /**
     * getConfigForDrag() determines when flap configuration mode
     * should change.
     */
    void getConfigForDrag(const Units::Speed&  airspeed, /** [in] Calibrated airspeed in knots */
                          const Units::Length& altitude,     /** [in] Altitude in meters */
                          const Units::Length& altitude_faf, /** [in] Altitude at final approach fix */
                          int                  modeLast,     /** [in] previous flap configuration (0-3) */
                          int&                 mode) const;        /** [out] flap configuration (0-3):
                                                                0=cruise, 1=approach, 2=landing, 3=gear down */

    /**
     * Calculates the maximum available thrust in Newtons.
     */
    double getMaxThrust(const Units::Length& altitude,          /** [in] altitude */
                        int                  mode = 0,          /** [in] flap configuration */
                        std::string          type = "cruise") const;  /** [in] "cruise" or "descent" */

    /**
     * Populate the flapsSpeeds structure based on
     * aircraft type and current ac_mass.
     */
    void setFlapSpeeds(const std::string& aircraftType);

    BadaWithCalc& operator=(const BadaWithCalc &obj);

    bool operator==(const BadaWithCalc &obj) const;
    bool operator!=(const BadaWithCalc &obj) const;

    FlapSpeeds  mFlapSpeeds;
    Units::Mass mAircraftMass;

    /**
    * Calculates fuel flow rate in kg/second
    * based on true airspeed, altitude, and thrust.
    *
    * CURRENTLY UNUSED
    */

    // NOTE:This commented out because it is not being used though it may be used in the future sdg

    /*
    double calcFuelFlow(
            double tas // [in] True airspeed in meters/second,
        double h // [in] Altitude in meters,
        double thrust // [in] Thrust in Newtons);
    */


private:
    FlapConfiguration getFlapConfiguration(const Units::Speed&  airspeed, /** [in] Calibrated airspeed */
                                           const Units::Length& altitude, /** [in] Altitude */
                                           const Units::Length& altitude_faf /** [in] Altitude at final approach fix */) const;

    double massFactor();

    bool CloseToFinalApproachFixAltitude(const Units::Length &altitude, const Units::Length &altitude_faf) const;
};
