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

#include "public/LoggingLoadable.h"

class Waypoint : public LoggingLoadable
{
public:

    static const Units::FeetLength MAX_ALTITUDE_CONSTRAINT;
    static const Units::FeetLength MIN_ALTITUDE_CONSTRAINT;
    static const Units::FeetLength UNDEFINED_ALTITUDE_CONSTRAINT;
    static const Units::KnotsSpeed MAX_SPEED_CONSTRAINT;
    static const Units::KnotsSpeed MIN_SPEED_CONSTRAINT;
    static const Units::KnotsSpeed UNDEFINED_SPEED_CONSTRAINT;

    enum AltitudeConstraintType {
        ABOVE = 100,
        BELOW,
        AT,
        BETWEEN,
        NONE,
        INVALID
    };

    Waypoint();
    Waypoint(const std::string& name_in,
             Units::Angle       latitude_in,
             Units::Angle       longitude_in,
             Units::Length      altitude_constraint_in = UNDEFINED_ALTITUDE_CONSTRAINT,
             Units::Length      altitude_constraint_2_in = UNDEFINED_ALTITUDE_CONSTRAINT,
             Units::Speed       speed_constraint_in = UNDEFINED_SPEED_CONSTRAINT,
             Units::Length      altitude_in = Units::ZERO_LENGTH,
             Units::Speed       nominal_ias_in = Units::ZERO_SPEED);

    virtual ~Waypoint();

    bool load(DecodedStream *input);

    void processAltitudeConstraints(Units::Length altitude_in,
                                    Units::Length altitude_2_in);

    void processSpeedConstraints(Units::Speed speed_in);

    const std::string& getName();
    void setName(const std::string name_in);

    Units::Angle getLatitude() const;
    void setLatitude(const Units::Angle& latitude_in);

    Units::Angle getLongitude() const;
    void setLongitude(const Units::Angle& longitude_in);

    void setWaypointLatLon(const Units::Angle& latitude_in,
                           const Units::Angle& longitude_in);

    Units::Length getAltitude() const;
    void setAltitude(const Units::Length& altitude_in);

    Units::Angle getDescentAngle() const;

    void setNominalIas(const Units::Speed& nominal_ias_in);
    Units::Speed getNominalIas() const;

    double getMach() const;

    Units::Acceleration getDescentRate() const;

    void setAltitudeConstraintHigh(const Units::Length& altitude_high_in);
    Units::Length getAltitudeConstraintHigh() const;

    void setAltitudeConstraintLow(const Units::Length& altitude_low_in);
    Units::Length getAltitudeConstraintLow() const;

    void setSpeedConstraintHigh(const Units::Speed &speed_high_in);
    Units::Speed getSpeedConstraintHigh() const;

    void setSpeedConstraintLow(const Units::Speed &speed_low_in);
    Units::Speed getSpeedConstraintLow() const;

    void setRfTurnCenterLatitude(const Units::Angle &rf_turn_center_latitude_in);
    Units::Angle getRfTurnCenterLatitude() const;

    void setRfTurnCenterLongitude(const Units::Angle &rf_turn_center_longitude_in);
    Units::Angle getRfTurnCenterLongitude() const;

    void setRfTurnArcRadius(const Units::Length &rf_turn_radius_in);
    Units::Length getRfTurnArcRadius() const;

private:
    std::string   name;
    Units::Angle  latitude;
    Units::Angle  longitude;
    Units::Angle  descent_angle;
    Units::Length altitude;
    Units::Speed  nominal_ias;
    Units::Acceleration descent_rate;
    double        mach;


    // previously public data members which store the altitude and speed constraints
    // Getters are public.
    Units::Length altitude_constraint_high;
    Units::Length altitude_constraint_low;
    Units::Speed speed_constraint_high;
    Units::Speed speed_constraint_low;

    // Added data members for RF legs
    Units::Angle rf_turn_center_latitude;
    Units::Angle rf_turn_center_longitude;
    Units::Length rf_turn_arc_radius;
};

inline const std::string& Waypoint::getName() {
    return name;
}

inline void Waypoint::setName(const std::string name_in) {
    name.assign(name_in);
}

inline Units::Angle Waypoint::getLatitude() const {
    return latitude;
}

inline void Waypoint::setLatitude(const Units::Angle& latitude_in) {
    latitude = latitude_in;
}

inline Units::Angle Waypoint::getLongitude() const {
    return longitude;
}

inline void Waypoint::setLongitude(const Units::Angle& longitude_in) {
    longitude = longitude_in;
}

inline void Waypoint::setWaypointLatLon(const Units::Angle& latitude_in, const Units::Angle& longitude_in) {
    setLatitude(latitude_in);
    setLongitude(longitude_in);
}

inline Units::Length Waypoint::getAltitude() const {
    return altitude;
}

inline void Waypoint::setAltitude(const Units::Length& altitude_in) {
    altitude = altitude_in;
}

inline Units::Angle Waypoint::getDescentAngle() const {
    return descent_angle;
}

inline Units::Speed Waypoint::getNominalIas() const {
    return nominal_ias;
}

inline void Waypoint::setNominalIas(const Units::Speed& nominal_ias_in) {
    nominal_ias = nominal_ias_in;
}

inline double Waypoint::getMach() const {
    return mach;
}

inline Units::Acceleration Waypoint::getDescentRate() const {
    return descent_rate;
}

inline void Waypoint::setAltitudeConstraintHigh(const Units::Length &altitude_high_in) {
    altitude_constraint_high = altitude_high_in;
}

inline Units::Length Waypoint::getAltitudeConstraintHigh() const {
    return altitude_constraint_high;
}

inline void Waypoint::setAltitudeConstraintLow(const Units::Length &altitude_low_in) {
    altitude_constraint_low = altitude_low_in;
}

inline Units::Length Waypoint::getAltitudeConstraintLow() const {
    return altitude_constraint_low;
}

inline void Waypoint::setSpeedConstraintHigh(const Units::Speed &speed_high_in) {
    speed_constraint_high = speed_high_in;
}

inline Units::Speed Waypoint::getSpeedConstraintHigh() const {
    return speed_constraint_high;
}

inline void Waypoint::setSpeedConstraintLow(const Units::Speed &speed_low_in) {
    speed_constraint_low = speed_low_in;
}

inline Units::Speed Waypoint::getSpeedConstraintLow() const {
    return speed_constraint_low;
}

inline Units::Angle Waypoint::getRfTurnCenterLatitude() const {
    return	rf_turn_center_latitude;
}

inline void Waypoint::setRfTurnCenterLatitude(const Units::Angle &rf_turn_center_latitude_in) {
    rf_turn_center_latitude = rf_turn_center_latitude_in;
}

inline Units::Angle Waypoint::getRfTurnCenterLongitude() const {
    return rf_turn_center_longitude;
}

inline void Waypoint::setRfTurnCenterLongitude(const Units::Angle &rf_turn_center_longitude_in) {
    rf_turn_center_longitude = rf_turn_center_longitude_in;
}

inline Units::Length Waypoint::getRfTurnArcRadius() const {
    return rf_turn_arc_radius;
}

inline void Waypoint::setRfTurnArcRadius(const Units::Length &rf_turn_radius_in) {
    rf_turn_arc_radius = rf_turn_radius_in;
}
