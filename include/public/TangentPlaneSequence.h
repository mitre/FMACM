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
#include "public/Waypoint.h"
#include "public/LocalTangentPlane.h"

/**
 * This class takes a series of waypoints and creates a LocalTangentPlane
 * for each, using the waypoint as the point of tangency.
 */
class TangentPlaneSequence {
public:
    TangentPlaneSequence();
	TangentPlaneSequence(std::list<Waypoint> &waypoint_list);
	~TangentPlaneSequence();
    TangentPlaneSequence(const TangentPlaneSequence &in); // copy constructor

	/**
	 * Algorithm provided by Lesley. This will convert lat/long to
	 * local-tangent-plane using the default EarthModel and the nearest
	 * point of tangency in the chain.
	 *
	 * Note that altitude is intentionally ignored here (treated as zero). Therefore, this is not
	 * a complete transformation algorithm.
	 *
	 * @param localPosition
	 * @param waypoint
	 */
	void convertLocalToGeodetic(EarthModel::LocalPositionEnu localPosition,
			EarthModel::GeodeticPosition &waypoint) const;
	void convertGeodeticToLocal(EarthModel::GeodeticPosition geoPosition,
			EarthModel::LocalPositionEnu &localPosition) const;
	const std::vector<EarthModel::LocalPositionEnu> &getLocalPositionsFromInitialization() const;
	const std::vector<Waypoint> &getWaypointsFromInitialization() const;
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
