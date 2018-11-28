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

#include "public/AircraftCalculations.h"
#include <stdexcept>
#include <public/CoreUtils.h>

using namespace std;

log4cplus::Logger AircraftCalculations::logger = log4cplus::Logger::getInstance("AircraftCalculations");

AircraftCalculations::AircraftCalculations(void)
{
}

AircraftCalculations::~AircraftCalculations(void)
{
}


// method to get the position and course of an Aircraft based on current distance (in meters) and precalculated Horizontal Trajectory
bool AircraftCalculations::getPosFromPathLength(const Units::Length& dist_in,
                                                const std::vector<HorizontalPath> &traj_in,
                                                Units::Length &x_out,
                                                Units::Length &y_out,
                                                Units::UnsignedAngle &course_out,
                                                int &traj_index)
{
    int index = -1; // stores the index last position < current distance
    const double metersIn = Units::MetersLength(dist_in).value();

    // loop to find the distance
    bool found = false;
    for( int loop = 0; loop < (int) traj_in.size() && !found; loop++)
    {
        if( metersIn <= traj_in[loop].L )
        {
            found = true;
            index = loop - 1; // sets position to the last position that was less than the given distance

            if( index < 0 )
            {
                index = 0;
            }
        }
    }

    // See AAES-639 for explanation. In FAS scenario, metersIn is different from total pathlength in the 9th decimal place.
    if ( !found && (metersIn < (traj_in[traj_in.size()-1].L + 0.0001))) {
        found = true;
        index = traj_in.size()-1;
    }

    if( found ==  true )
    {
        // calculate position based on if it's a straight or turning path
        if(traj_in[index].segment == "straight")
        {
            Units::Angle course = Units::RadiansAngle(traj_in[index].course); // get the course for the given index

            // calculate output values
            x_out = Units::MetersLength(traj_in[index].x) + ((dist_in - Units::MetersLength(traj_in[index].L)) * cos(course));
            y_out = Units::MetersLength(traj_in[index].y) + ((dist_in - Units::MetersLength(traj_in[index].L)) * sin(course));
            course_out = AircraftCalculations::convert0to2Pi(course + Units::PI_RADIANS_ANGLE);
        }
        else if(traj_in[index].segment == "turn")
        {
            if ((dist_in - Units::MetersLength(traj_in[index].L) ) < Units::MetersLength(3))
            {
                x_out = Units::MetersLength(traj_in[index].x);
                y_out = Units::MetersLength(traj_in[index].y);
                course_out = Units::UnsignedRadiansAngle(traj_in[index].course) + Units::PI_RADIANS_ANGLE;
            } else {
                Units::Length radius = Units::MetersLength(traj_in[index].turns.radius);
                Units::UnsignedAngle start = Units::UnsignedRadiansAngle(traj_in[index].turns.q_start);
                Units::UnsignedAngle end = Units::UnsignedRadiansAngle(traj_in[index].turns.q_end);

                // calculate course change between the start and end of turn
                Units::SignedRadiansAngle course_change = convertPitoPi(end - start);

                // calculate difference in distance
                Units::Angle delta = Units::RadiansAngle((dist_in - Units::MetersLength(traj_in[index].L)) / radius);

                // calculate the theta of the turn
                Units::Angle theta = start + delta * CoreUtils::sign(course_change.value());

                // calculate X and Y positions
                x_out = Units::MetersLength(traj_in[index].turns.x_turn) + radius * cos(theta);
                y_out = Units::MetersLength(traj_in[index].turns.y_turn) + radius * sin(theta);

                course_out = AircraftCalculations::convert0to2Pi(
                        theta - Units::PI_RADIANS_ANGLE / 2.0 * CoreUtils::sign(course_change.value()));
            }
        }
        traj_index = index;
        return true;
    } else {
        traj_index = -1;
        return false;
    }
}


void AircraftCalculations::getPathLengthFromPos(const Units::Length x, const Units::Length y, const vector<HorizontalPath> &hTraj, Units::Length &dist, Units::Angle &trk) {

    // Computes distance and course based on aircraft position
    // and horizontal trajectory.  Distance is from aircraft
    // to nearest point from the horizontal trajectory along
    // the path.
    //
    // x,y:aircraft position in meters.
    // hTraj:horizontal trajectory of aircraft,
    //       (point positions in meters).
    // dist:computed distance in meters (output).
    // trk:aircraft course to point in radians (output).


    // Dummy values

    dist = Units::NegInfinity();
    trk = Units::RadiansAngle(99999.99999);

    // Find closest point

    // Compute Euclidean distances for all horizontal trajectory points
    // and order in ascending sequence.

    vector<mPathDistance> distances =
            AircraftCalculations::computePathDistances(x, y, hTraj);


    // Find smallest distance with an acceptable cross track error.

    int nextTrajIx = -1;
    const Units::NauticalMilesLength cte_tolerance(2.5);
    Units::NauticalMilesLength cte;
    for (auto i = 0; ((i < distances.size()) && (nextTrajIx == -1)); i++) {

        int computedNextIx;

        AircraftCalculations::crossTrackError(x,y,
                                              distances[i].mIx,hTraj,computedNextIx,cte);

        if (cte <= cte_tolerance) {
            nextTrajIx = computedNextIx;
        }
    }

    if (nextTrajIx == -1) {
        /*
         * Developer's: note that this if logic is no longer a FATAL condition, but we
         * must allow it to throw. Some callers will catch and handle the situation because
         * it is sometimes normal. See AAES-382, AAES-633
         */
        char msg[150];
        sprintf(msg,
                "Trajectory point with acceptable cross track error not found %lf nmi",
                cte.value());
//        LOG4CPLUS_FATAL(logger, "x: " << Units::MetersLength(x).value());
//        LOG4CPLUS_FATAL(logger, "y: " << Units::MetersLength(y).value());
//        LOG4CPLUS_FATAL(logger, "distances: ");
//        for (int i = 0; i < distances.size(); ++i) {
//            LOG4CPLUS_FATAL(logger, "\t" << Units::MetersLength(distances[i].mDist).value());
//        }
//        LOG4CPLUS_FATAL(logger, "hTraj:");
//        for (int j = 0; j < hTraj.size(); ++j) {
//            LOG4CPLUS_FATAL(logger, "\tx(m): " << hTraj[j].x);
//            LOG4CPLUS_FATAL(logger, "\ty(m): " << hTraj[j].y);
//            LOG4CPLUS_FATAL(logger, "\tL(m): " << hTraj[j].L);
//        }
        throw logic_error(msg);
    }


    // Calculate DTG and course of aircraft.

    Units::Length dap;
    Units::RadiansAngle theta;

    if (hTraj[nextTrajIx].segment == "straight") {

        Units::Length d = sqrt(Units::sqr(x-Units::MetersLength(hTraj[nextTrajIx].x)) +
                               Units::sqr(y-Units::MetersLength(hTraj[nextTrajIx].y)));

        //trk = convert0to2Pi( Units::RadiansAngle(hTraj[nextTrajIx].course) + Units::PI_RADIANS_ANGLE);
        trk = AircraftCalculations::convert0to2Pi(Units::RadiansAngle(hTraj[nextTrajIx].course) + Units::PI_RADIANS_ANGLE);

        if (fabs(Units::MetersLength(d).value()) < 1E-5) {
            dap = Units::MetersLength(0.0);
        } else {
            Units::MetersLength dx = Units::MetersLength(hTraj[nextTrajIx].x) - x;
            Units::MetersLength dy = Units::MetersLength(hTraj[nextTrajIx].y) - y;
            theta = Units::RadiansAngle(atan2(dy.value(), dx.value()));

            Units::SignedAngle deltaTheta = convertPitoPi(theta - trk);

            dap = d * cos(deltaTheta);
        }

    } else if (hTraj[nextTrajIx].segment == "turn") {

        Units::MetersLength dx = x - Units::MetersLength(hTraj[nextTrajIx].turns.x_turn);
        Units::MetersLength dy = y - Units::MetersLength(hTraj[nextTrajIx].turns.y_turn);

        // theta is undefined if dx and dy are both zero
        // if within 5 meters of the point, consider at the point
        double lx = (Units::MetersLength(x)).value();
        double ly = (Units::MetersLength(y)).value();

        if (pow(lx-(hTraj[nextTrajIx]).x, 2) + pow(ly-(hTraj[nextTrajIx]).y, 2) < 9) {
            dist = Units::MetersLength(hTraj[nextTrajIx].L);
            trk = AircraftCalculations::convert0to2Pi(Units::RadiansAngle(hTraj[nextTrajIx].course) + Units::PI_RADIANS_ANGLE);
            return;
        }

        theta = Units::UnsignedRadiansAngle(atan2(dy.value(), dx.value()));

        Units::RadiansAngle deltaTheta =
                AircraftCalculations::convertPitoPi(Units::UnsignedRadiansAngle(hTraj[nextTrajIx].turns.q_start)-theta);

        dap = Units::MetersLength(hTraj[nextTrajIx].turns.radius) * fabs(deltaTheta.value());

        //trk = convert0to2Pi(Units::RadiansAngle(hTraj[nextTrajIx-1].course)
        //                    + Units::PI_RADIANS_ANGLE - deltaTheta);
        if (CoreUtils::sign(deltaTheta.value()) > 0)
            trk = theta + Units::PI_RADIANS_ANGLE / 2;
        else
            trk = theta - Units::PI_RADIANS_ANGLE / 2;

        trk = convert0to2Pi(trk);
    } else {
        throw logic_error("Non straight non turn segment in getPathLengthFromPos");
    }

    dist = dap + Units::MetersLength(hTraj[nextTrajIx].L);

} // getPathLengthFromPos

vector<AircraftCalculations::mPathDistance> AircraftCalculations::computePathDistances(
        const Units::Length x, const Units::Length y, const vector<HorizontalPath> &hTraj) {

    // Computes distances for a position along the horiontal
    // trajectory.
    //
    // x,y:position in meters.
    // hTraj:horizontal trajectory-containing points in meters.
    // returns distances and cross track errors for all points in a vector
    //         ordered ascending by distance.

    vector<mPathDistance> pdVect;

    for (auto i=0; i<hTraj.size(); i++) {

        mPathDistance pd;

        Units::MetersLength d = sqrt(Units::sqr(x-Units::MetersLength(hTraj[i].x))
                                     + Units::sqr(y-Units::MetersLength(hTraj[i].y)));
        pd.mDist = d;
        pd.mIx = i;

        if (std::isnan(d.value())) {
            string msg = "undefined distance (NaN) computed for path distance";
            LOG4CPLUS_FATAL(logger, msg);
        }

        if (pdVect.empty()) {
            pdVect.push_back(pd);
        } else {

            vector<mPathDistance>::iterator iter = pdVect.begin();

            while (iter < pdVect.end()) {

                if ((*iter).mDist > pd.mDist) {
                    break;
                }

                iter++;
            }

            pdVect.insert(iter, pd);
        }
    }

    return pdVect;

}


// coverts angle into a range from 0 to 2PI (0 to 360 degrees)
Units::UnsignedRadiansAngle AircraftCalculations::convert0to2Pi(Units::Angle course_in)
{
    Units::UnsignedRadiansAngle result = course_in;
    result.normalize();

    return result;
}

// coverts angle into a range from -PI to PI (-180 to 180 degrees)
Units::SignedRadiansAngle AircraftCalculations::convertPitoPi(Units::Angle course_in)
{
    Units::SignedRadiansAngle result = course_in;
    result.normalize();

    return result;
}


// method for calculating the Cas ESF
//double v_tas: true airspeed (m/s)
//double alt: altitude (meters)
double AircraftCalculations::ESFconstantCAS(const Units::Speed v_tas, const Units::Length alt)
{
    double esf;
    Units::KelvinTemperature temperature;
    double mach;
    double temp1, temp2, temp3;

    temperature = ATMOSPHERE()->getTemp(Units::MetersLength(alt));
    mach = v_tas/sqrt(GAMMA*R*temperature);

    temp1 = 1.0 + (GAMMA-1.0)/2* pow(mach, 2);
    temp2 = (pow(temp1,(-1.0/(GAMMA-1))))*(pow(temp1,(GAMMA/(GAMMA-1))) - 1.0);

    if( alt <= H_TROP)
    {
        temp3 = 1.0 + (GAMMA*R*K_T)/(Units::ONE_G_ACCELERATION*2) * pow(mach, 2) + temp2;
    }
    else
    {
        temp3 = 1.0 + temp2;
    }

    esf = 1.0/temp3;
    return esf;
}

Units::NauticalMilesLength AircraftCalculations::ptToPtDist(Units::Length x0, Units::Length y0, Units::Length x1, Units::Length y1)
{
    // Computes distance between points.
    //
    // x0:x component of first point in feet.
    // y0:y component of first point in feet.
    // x1:x component of last point in feet.
    // y1:y component of last point in feet.
    //
    // returns distance between (x0,y0) to (x1,y1) in nmi.

    Units::NauticalMilesLength dist = sqrt(Units::sqr(x1-x0) + Units::sqr(y1-y0));

    return dist;
}

Units::Speed AircraftCalculations::gsAtACS(AircraftState acs)
{
    // method to compute ground speed from the position at an aircraft state.
    //
    // acs:aircraft state containing position to compute for.
    //
    // returns:computed ground speed.

    return Units::FeetPerSecondSpeed(sqrt(pow(acs.xd, 2) + pow(acs.yd, 2)));
}

void AircraftCalculations::crossTrackError(Units::Length x,
                                           Units::Length y,
                                           int trajIx,
                                           vector<HorizontalPath> hTraj,
                                           int &nextTrajIx,
                                           Units::Length &cte) {

    //  Calculates the cross track for a segment leading to a horizontal
    //  trajectory point.  The calculation is performed differently for a
    //  turn segment and for a straight segment.
    //
    //  x,y: aircraft position in meters.
    //  trajIx: index of trajectory point.
    //  hTraj: horizontal trajectory for aircraft; positions, turn radius
    //         in meters; course in radians.  Segment type, course, and
    //         position are used from here.
    //  nextTrajIx: downstream trajectory point (output).
    //  cte: cross track error in meters (output).
    //
    //  Turn Segment:
    //  Position is tested to see if it is in the space formed by the two
    //  directed line segments that form the turn arc.
    //  First, determine the turn orientation: see if the points center of
    //  turn, start turn, and end turn are clockwise or counter-clockwise.
    //  If the orientation of the points center of turn, start turn, position
    //  do not have the same orientation, then return a large number for cte.
    //  If the orientation of the points center of turn, turn stop, position
    //  does not have the opposite orientation, return a large number for cte.
    //  If the conditions are met, then return the distance from the turn
    //  center minus the radius for cte.  It is possible for two sequential
    //  turn segments to be slightly misaligned so that a position could be
    //  after one segment and before another segment.  In this case, the
    //  position will be considered as on the next segment.
    //
    //  Straight Segment:
    //  Straight segments could have a small turn, and still be considered
    //  straight. A perpendicular to the course segment through the
    //  position is calculated.  If this perpendicular intersects the
    //  segment, its distance from the segment is returned.  If the position
    //  is after the horizontal trajectory point, a large value for cte is
    //  returned.  If the position is before the beginning of the segment,
    //  the distance from the position to the beginning of the segment
    //  (preceding horizontal trajectory point) is returned for cte and the
    //  horizontal trajectory point is returned for nextTrajIx.
    //
    //  In either case, the preceding segment could be of the other type and
    //  a gap could exist.  Must test if in the gap and not within the
    //  preceding segment.

    // dummy values

    cte = Units::Infinity();
    nextTrajIx = (int)-INFINITY;

    if (trajIx >= hTraj.size() - 1) {
        // No segment going to first point on route (last trajectory point)
        return;
    }

    if (hTraj[trajIx].segment == "turn") {
        // Compute cross track error for turn.

        // Allowed for RF Legs
        //if (trajIx == 0) {
        //  cout << "Illegal condition computing cross track error." << endl;
        //  cout << "First trajectory point cannot be for a turn." << endl;
        //  exit(-39);
        //}


        double x0 = hTraj[trajIx].turns.x_turn; // turn center
        double y0 = hTraj[trajIx].turns.y_turn;
        double x1 = hTraj[trajIx+1].x;          // start turn
        double y1 = hTraj[trajIx+1].y;
        double x2 = hTraj[trajIx].x;            // stop turn
        double y2 = hTraj[trajIx].y;

        double dx = Units::MetersLength(x).value();
        double dy = Units::MetersLength(y).value();

        // determine orientation of turn.  If crossproduct is zero, then points are colinear
        // sign(P0, P1, p2) i.e., center, start, end.
        double crossProdSign0 = (y0 - y1) * x2 + (x1 - x0) * y2 + (x0 * y1 - x1 * y0);
        if (crossProdSign0 == 0) // zero length turn
            return;

        // determine orientation of turn center, start turn, position
        double crossProdSign1 = (y0 - y1) * dx + (x1 - x0) * dy + (x0 * y1 - x1 * y0);
        if (((crossProdSign0 > 0) && (crossProdSign1 < 0)) || ((crossProdSign0 < 0)
                                                               && (crossProdSign1 > 0))) {
            // position is before start turn
            // need to test if not in previous segment, i.e. in the V between two segments (see AAES-360)
            if (hTraj[trajIx+1].segment == "turn")
            {
                double x3 = hTraj[trajIx+1].turns.x_turn;
                double y3 = hTraj[trajIx+1].turns.y_turn;
                double crossProdSign3 = (y3 - y1) * dx + (x1 - x3) * dy + (x3 * y1 - x1 * y3);
                if (((crossProdSign3 > 0) && (crossProdSign1 < 0)) || ((crossProdSign3 < 0)
                                                                      && (crossProdSign1 > 0)))
                { // not in previous section - so calculate error
                    nextTrajIx = trajIx;
                    cte = abs(Units::MetersLength(sqrt(pow(x0 - dx, 2) + pow(y0 - dy, 2))) -
                              Units::MetersLength(hTraj[trajIx].turns.radius));
                } // otherwise return not in segment
            } else { // previous segment is straight
                // Use Pythagorean Theorem
                if (trajIx >= hTraj.size()-2)
                    return;
                double x3 = hTraj[trajIx+2].x;
                double y3 = hTraj[trajIx+2].y;
                if (pow(x3 - dx, 2) + pow(y3 - dy, 2) >
                        (pow(x1 - dx, 2) + pow(y1 - dy, 2) + pow(x3 - x1, 2) + pow(y3 - y1, 2))) {
                    // not in preceding segment so calculate cte
                    cte = abs(Units::MetersLength(sqrt(pow(x1 - dx, 2) + pow(y1 - dy, 2))));
                    nextTrajIx = trajIx;
                }
            }
            return; // position is before start turn
        }

        double crossProdSign2 = (y0 - y2) * dx + (x2 - x0) * dy + (x0 * y2 - x2 * y0);
        if (((crossProdSign1 > 0) && (crossProdSign2 > 0)) || ((crossProdSign1 < 0)
                                                               && (crossProdSign2 < 0))) {
            if (trajIx == 0) {
                Units::MetersLength endDistance = abs(Units::MetersLength(sqrt(pow(x2 - dx, 2) + pow(y2 - dy, 2))));
                if (endDistance < Units::MetersLength(500)) {
                    cte = endDistance;
                    nextTrajIx = trajIx;
                }
            }
            return; // position is after stop turn
        }

        cte = abs(Units::MetersLength(sqrt(pow(x0 - dx, 2) + pow(y0 - dy, 2))) -
                  Units::MetersLength(hTraj[trajIx].turns.radius));
        nextTrajIx = trajIx;
        return;

    } else if (hTraj[trajIx].segment == "straight") {
        // test for zero length leg
        if (fabs(hTraj[trajIx].x - hTraj[trajIx+1].x) < 0.00001
            && fabs(hTraj[trajIx].y - hTraj[trajIx+1].y) < 0.00001) {
            return;
        }

        double x0 = hTraj[trajIx+1].x;                // start of segment
        double y0 = hTraj[trajIx+1].y;
        double x1 = hTraj[trajIx].x;                  // end of segment
        double y1 = hTraj[trajIx].y;
        double dx = Units::MetersLength(x).value();   // position
        double dy = Units::MetersLength(y).value();

        double vx = x1 - x0;
        double vy = y1 - y0;
        double wx = dx - x0;
        double wy = dy - y0;

        double c1 = vx * wx + vy * wy;
        if ( c1 < 0 ) { // before start of segment
            // See if in preceding segment
            if (trajIx+2 < hTraj.size()) {
                if (hTraj[trajIx+1].segment == "straight") {
                    double x2 = hTraj[trajIx + 2].x;
                    double y2 = hTraj[trajIx + 2].y;
                    double ux = x2 - x0;
                    double uy = y2 - y0;
                    double c3 = ux * wx + uy * wy;
                    if (c3 >= 0) {
                        // position is in the preceding segment
                        return;
                    }
                } else { // previous segment is turn}
                    double x3 = hTraj[trajIx].turns.x_turn; // turn center
                    double y3 = hTraj[trajIx].turns.y_turn;
                    double x2 = hTraj[trajIx+1].x;          // start turn
                    double y2 = hTraj[trajIx+1].y;
                    double crossProdSign0 = (y3 - y0) * x2 + (x0 - x3) * y2 + (x3 * y0 - x0 * y3);
                    if (crossProdSign0 == 0) // zero length turn
                        return;
                    double crossProdSign1 = (y3 - y0) * dx + (x0 - x3) * dy + (x3 * y0 - x0 * y3);
                    if (((crossProdSign0 > 0) && (crossProdSign1 > 0)) ||
                        ((crossProdSign0 < 0) && (crossProdSign1 < 0))) {
                        return; // in previous section
                    }
                }
            }
            // Not in preeceding segment so include with this segment
            cte = abs(Units::MetersLength(sqrt(pow(x0 - dx, 2) + pow(y0 - dy, 2))));
            nextTrajIx = trajIx;
            return;
        }

        double c2 = vx * vx + vy * vy;
        if (c2 < c1) { // after end of segment
            // check that not at end of route (trajIx == 0)
            if (trajIx > 0) {
                return;
            }
            // Need to include positions after end of route or simulation will
            // not terminate properly
            cte = abs(Units::MetersLength(sqrt(pow(x1 - dx, 2) + pow(y1 - dy, 2))));
            nextTrajIx = trajIx;
            return;
        }

        // position is between the end points of the segment
        double b = c1 / c2;
        double xb = x0 + b * vx;
        double yb = y0 + b * vy;
        cte = abs(Units::MetersLength(sqrt(pow(xb - dx, 2) + pow(yb - dy, 2))));
        nextTrajIx = trajIx;
        return;

    } else {
        // Error
        throw logic_error("Invalid segment condition found in crossTrackError");
    }

} // crossTrackError()


Units::SignedRadiansAngle AircraftCalculations::computeAngleBetweenVectors(const Units::Length &xvertex, const Units::Length &yvertex, const Units::Length &x1, const Units::Length &y1, const Units::Length &x2, const Units::Length &y2)
{
    // vector 1 is from vertex to x1,y1
    Units::MetersLength dx1 = x1 - xvertex;
    Units::MetersLength dy1 = y1 - yvertex;
    double norm1 = sqrt(dx1.value()*dx1.value() + dy1.value()*dy1.value());

    // vector 2 is from turn center to turn end
    Units::MetersLength dx2 = x2 - xvertex;
    Units::MetersLength dy2 = y2 - yvertex;
    double norm2 = sqrt(dx2.value()*dx2.value() + dy2.value()*dy2.value());

    // theta is acos(dot product)
    double dotp = dx1.value()/norm1*dx2.value()/norm2 + dy1.value()/norm1*dy2.value()/norm2;
    if (fabs(dotp) > 1) {
        dotp = CoreUtils::sign(dotp) * 1.0;
    }
    Units::SignedRadiansAngle theta(acos(dotp));  // acos is on [0,pi]
    return theta;
}

double AircraftCalculations::computeCrossProduct(const Units::Length &xvertex, const Units::Length &yvertex, const Units::Length &x1, const Units::Length &y1, const Units::Length &x2, const Units::Length &y2)
{
    double x0d = Units::MetersLength(xvertex).value();
    double y0d = Units::MetersLength(yvertex).value();
    double x1d = Units::MetersLength(x1).value();
    double y1d = Units::MetersLength(y1).value();
    double x2d = Units::MetersLength(x2).value();
    double y2d = Units::MetersLength(y2).value();
    return (y0d - y1d) * x2d +
           (x1d - x0d) * y2d +
           (x0d * y1d - x1d * y0d);
}
