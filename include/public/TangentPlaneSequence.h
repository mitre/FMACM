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

/*
 * TangentPlaneSequence.h
 *
 *  Created on: Jul 5, 2015
 *      Author: klewis
 */

#pragma once

class TangentPlaneSequence;	// avoid dependency loop

#include <list>
#include <vector>
#include <memory>
#include "public/Waypoint.h"
#include "public/LocalTangentPlane.h"

/**
 * This class takes a series of waypoints and creates a LocalTangentPlane
 * for each, using the waypoint as the point of tangency.
 */
class TangentPlaneSequence {
public:
    TangentPlaneSequence();
    /**
     * Constructs a sequence of LocalTangentPlane objects, one
     * for each waypoint.  The final waypoint in the list maps
     * to the origin in ENU coordinates, and each of the other
     * waypoints maps to the same coordinates in its own plane
     * as it does in its successor's plane.  The altitudes of
     * the waypoints are ignored and treated as zero (sea level);
     * only the latitudes and longitudes are used.
     */
	TangentPlaneSequence(std::list<Waypoint> &waypoint_list);
	~TangentPlaneSequence();
    TangentPlaneSequence(const TangentPlaneSequence &in); // copy constructor

	/**
	 * Converts a local ENU point to geodetic coordinates
	 * using the default EarthModel and the nearest
	 * point of tangency in the sequence.
	 *
	 * Note that altitude is intentionally ignored (treated as
	 * zero) by EllipsoidalEarthModel.
	 *
	 * @param localPosition
	 * @param waypoint
	 */
	void convertLocalToGeodetic(EarthModel::LocalPositionEnu localPosition,
			EarthModel::GeodeticPosition &geoPosition) const;
	/**
	 * Converts a geodetic point to local ENU coordinates
	 * using the default EarthModel and the nearest
	 * point of tangency in the sequence.
	 *
	 * Note that returned altitude is reset to zero by EllipsoidalEarthModel.
	 *
	 * @param localPosition
	 * @param waypoint
	 */
	void convertGeodeticToLocal(EarthModel::GeodeticPosition geoPosition,
			EarthModel::LocalPositionEnu &localPosition) const;
	/**
	 * Returns the ENU coordinates of each of the waypoints
	 * supplied during construction.
	 */
	const std::vector<EarthModel::LocalPositionEnu> &getLocalPositionsFromInitialization() const;
	/**
	 * Returns copies of the waypoints used during construction.
	 */
	const std::vector<Waypoint> &getWaypointsFromInitialization() const;
	/**
	 * Returns tangent planes for each of the waypoints used during
	 * construction.
	 */
    const std::vector<std::shared_ptr<LocalTangentPlane> > &getTangentPlanesFromInitialization() const;

private:
    static log4cplus::Logger logger;
    void copy(const TangentPlaneSequence &in); // helper method for copy constructor and assignment operator

protected:
    virtual void initialize(std::list<Waypoint> &waypoint_list);
    std::vector<Waypoint> waypointsFromInitialization;
    std::vector<std::shared_ptr<LocalTangentPlane> > tangentPlanesFromInitialization;
    std::vector<EarthModel::LocalPositionEnu> localPositionsFromInitialization;
};
