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

#include "public/AircraftIntent.h"
#include "public/AircraftCalculations.h"
#include "public/SingleTangentPlaneSequence.h"
#include <stdexcept>

using namespace std;

log4cplus::Logger AircraftIntent::logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("AircraftIntent"));

const int AircraftIntent::UNINITIALIZED_AIRCRAFT_ID = -1;


AircraftIntent::AircraftIntent()
        : id (UNINITIALIZED_AIRCRAFT_ID),
          number_of_waypoints(0),
          mIsLoaded(false)
{
    waypoint_x[20] = {};
    waypoint_y[20] = {};
    waypoint_altitude[20] = {};
    waypoint_descent_angle[20] = {};
    waypoint_nominal_ias[20] = {};
    waypoint_mach[20] = {};
    waypoint_descent_rate[20] = {};

    initialize();
}

// copy constructor for AircraftIntent
AircraftIntent::AircraftIntent(const AircraftIntent &in)
        : id (UNINITIALIZED_AIRCRAFT_ID),
          number_of_waypoints(0),
          mIsLoaded(false)
{
    waypoint_x[20] = {};
    waypoint_y[20] = {};
    waypoint_altitude[20] = {};
    waypoint_descent_angle[20] = {};
    waypoint_nominal_ias[20] = {};
    waypoint_mach[20] = {};
    waypoint_descent_rate[20] = {};

    initialize();

    copy(in);
}


AircraftIntent::~AircraftIntent()
{
}

void AircraftIntent::initialize()
{
    tangentPlaneSequence.reset();

    for(unsigned int c = 0; c < MAX_NUM_WAYPOINTS; ++c)
    {
        waypoint_name[c] = "";
        waypoint_altitude[c] = Units::ZERO_LENGTH;
        waypoint_descent_angle[c] = Units::ZERO_ANGLE;
        waypoint_nominal_ias[c] = Units::ZERO_SPEED;
        waypoint_mach[c] = 0;
        waypoint_descent_rate[c] = Units::Acceleration(0);

        fms.Name[c]  = "";
        fms.AltWp[c] = Units::ZERO_LENGTH;
        fms.LatWp[c] = Units::ZERO_ANGLE;
        fms.LonWp[c] = Units::ZERO_ANGLE;
        fms.nominal_IAS_at_waypoint[c] = Units::ZERO_SPEED;
        fms.MACH_at_waypoint[c] = 0;

        // the new constraint values
        fms.altHi[c] = Units::ZERO_LENGTH;
        fms.altLow[c] = Units::ZERO_LENGTH;
        fms.speedHi[c] = Units::ZERO_SPEED;
        fms.speedLow[c] = Units::ZERO_SPEED;

        // RF Leg values
        fms.LatCp[c] = Units::ZERO_ANGLE;
        fms.LonCp[c] = Units::ZERO_ANGLE;
        fms.radiusCp[c] = Units::ZERO_LENGTH;
        fms.LatCp[c] = Units::ZERO_ANGLE;
        fms.LonCp[c] = Units::ZERO_ANGLE;
    }
}

// assignment operator overload
AircraftIntent& AircraftIntent::operator=(const AircraftIntent &in)
{
    copy(in);

    return *this;
}

// call this after the load to finish things up 

void AircraftIntent::load_waypoints_from_list(list<Waypoint>& waypoint_list)
{
    int c = 0;
    list<Waypoint>::iterator i = waypoint_list.begin();
    list<Waypoint>::iterator e = waypoint_list.end();

    while(i != e)
    {
        assert((*i).getName().size() < 16);

        waypoint_name[c] = (*i).getName();

        // waypoint_x and waypoint_y are set after loading.
        waypoint_altitude[c]      = (*i).getAltitude();
        waypoint_descent_angle[c] = (*i).getDescentAngle();
        waypoint_nominal_ias[c]   = (*i).getNominalIas();
        waypoint_mach[c]          = (*i).getMach();
        waypoint_descent_rate[c]  = (*i).getDescentRate();

        fms.Name[c]  = (*i).getName();
        fms.AltWp[c] = (*i).getAltitude();
        fms.LatWp[c] = (*i).getLatitude();
        fms.LonWp[c] = (*i).getLongitude();
        fms.nominal_IAS_at_waypoint[c] = (*i).getNominalIas();
        fms.MACH_at_waypoint[c] = (*i).getMach();

        // the new constraint values
        fms.altHi[c] = (*i).getAltitudeConstraintHigh();
        fms.altLow[c] = (*i).getAltitudeConstraintLow();
        fms.speedHi[c] = (*i).getSpeedConstraintHigh();
        fms.speedLow[c] = (*i).getSpeedConstraintLow();

        // RF Leg values
        fms.LatCp[c] = (*i).getRfTurnCenterLatitude();
        fms.LonCp[c] = (*i).getRfTurnCenterLongitude();
        fms.radiusCp[c] = (*i).getRfTurnArcRadius();

        //------
        //gwang 2009-09: handle NASA ASTAR route (waypoint format)
//        todo: commented this out, because I'm not sure it's really doing anything helpful. ALso doesn't check if it's
//       todo: off the end. handled up above.
//      if (fms.AltWp[c] == 0)
//      {
//         fms.AltWp[c] = fms.AltWp[c-1];
//      }
        //end gwang 2009-09

        //gwang 2009-09: handle the NASA ASTAR route (waypoint format)
        //If Mach is zero, it means there is no restriction.
        //It should be equal to the value of the Mach at the previous waypoint.
        //The logic is as follows:
        double mach = waypoint_mach[c];
        Units::FeetPerSecondSpeed ias = waypoint_nominal_ias[c];

        if(mach != 0) //This is for most cases. IAS should be the IAS at mach-IAS transition.
        {
            fms.nominal_IAS_at_waypoint[c] = mach_transition_cas;
            fms.MACH_at_waypoint[c] = waypoint_mach[c];
        }
        else if(c >= 1 && fms.MACH_at_waypoint[c-1] != 0
                && ias.value() == 0)
        {
            fms.nominal_IAS_at_waypoint[c] = mach_transition_cas;
            fms.MACH_at_waypoint[c] = fms.MACH_at_waypoint[c-1];
        }
        else if(c >= 1 && ias.value() != 0)
        {
            fms.nominal_IAS_at_waypoint[c] = ias;
            fms.MACH_at_waypoint[c] = 0;
        }
        else if(c >= 1 && fms.nominal_IAS_at_waypoint[c-1].value() != 0
                && ias.value() == 0)
        {
            fms.nominal_IAS_at_waypoint[c] = fms.nominal_IAS_at_waypoint[c-1];
            fms.MACH_at_waypoint[c] = 0;
        }
        else
        {
            fms.nominal_IAS_at_waypoint[c] = ias;
            fms.MACH_at_waypoint[c] = mach;
        }

        //------
        ++i;
        ++c;
    }


    number_of_waypoints = waypoint_list.size();
    waypoints = waypoint_list;

    // the waypoints are loaded. Initialize the earth model with them
    tangentPlaneSequence = shared_ptr<TangentPlaneSequence>(new SingleTangentPlaneSequence(waypoint_list));
}

void AircraftIntent::update_xyz_from_latlon_wgs84()
{
    // Call the earth model
    vector<EarthModel::LocalPositionEnu> localPositions;
    localPositions = tangentPlaneSequence->getLocalPositionsFromInitialization();
    EarthModel::GeodeticPosition geoPosition;
    EarthModel::LocalPositionEnu xyPosition;

    // Process the waypoints
    for (int var = number_of_waypoints-1; var >= 0; --var)
    {
        // Waypoints
        geoPosition.altitude = Units::ZERO_LENGTH;
        geoPosition.latitude = Units::RadiansAngle(fms.LatWp[var]);
        geoPosition.longitude = Units::RadiansAngle(fms.LonWp[var]);
        tangentPlaneSequence->convertGeodeticToLocal(geoPosition, xyPosition);
        waypoint_x[var] = fms.xWp[var] = xyPosition.x; // yup, waypoint_x always holds the same value as fms.xWp
        waypoint_y[var] = fms.yWp[var] = xyPosition.y; // yup, waypoint_y always holds the same value as fms.yWp
        fms.zWp[var] = xyPosition.z;

        // Center points
        if (fms.radiusCp[var].value() == 0)
        {
            fms.xCp[var] = Units::ZERO_LENGTH;
            fms.yCp[var] = Units::ZERO_LENGTH;
        }
        else
        {
            geoPosition.altitude = Units::FeetLength(0);
            geoPosition.latitude = Units::RadiansAngle(fms.LatCp[var]);
            geoPosition.longitude = Units::RadiansAngle(fms.LonCp[var]);
            tangentPlaneSequence->convertGeodeticToLocal(geoPosition, xyPosition);
            fms.xCp[var] = xyPosition.x;
            fms.yCp[var] = xyPosition.y;
        }
    }
}

double AircraftIntent::calcGreatCircleDist(double lat1, double lon1, double lat2, double lon2)
{
    double d, dNM, dist;

    d = 2*asin(sqrt(pow(sin((lat1-lat2)/2),2.0) + cos(lat1)*cos(lat2)*pow(sin((lon1-lon2)/2),2.0)));

    dNM = ((180.0*60.0)/PI)*d;

    dist = dNM*NM_M;

    return dist;
}

double AircraftIntent::calcGreatCircleCrs(double lat1, double lon1, double lat2, double lon2, double dist)
{
    double dNM, d, crs;

    dNM = dist/NM_M;

    d = dNM/((180.0*60.0)/PI);

    if( sin(lon2-lon1) < 0 )
    {
        crs = acos((sin(lat2)-sin(lat1)*cos(d))/(sin(d)*cos(lat1)));
    }
    else
    {
        double temp;
        temp = (sin(lat2)-sin(lat1)*cos(d))/(sin(d)*cos(lat1));

        if(temp > 1.0)
            temp = 1.0;

        if(temp < -1.0)
            temp = -1.0;

        crs = 2*PI-acos(temp);
    }

    // Convert course from great-circle convention to mathematical convention
    crs = crs-3.0*PI/2.0;

    return crs;

}

int AircraftIntent::findWaypointIx(std::string inwaypoint) const {

    // Finds index into waypoint list for input waypoint.
    //
    // inwaypoint:input waypoint to search for.
    // returns index into AircraftIntent waypoint list for
    //         input waypoint.  If waypoint not found,
    //         -1 returned.

    int ix = -1;

    for (int i = 0;((i < number_of_waypoints) && (ix == -1));++i)
    {
        if (inwaypoint == waypoint_name[i])
        {
            ix = i;
        }
    }

    return ix;

}

void AircraftIntent::dump(ofstream& fileOut) const
{
    fileOut << "------------"  << endl;
    fileOut << "Intent of aircraft  " << id << ":" << endl;
    fileOut << "mach_transition_cas " << Units::KnotsSpeed(mach_transition_cas).value() << endl;


    for(unsigned int i=0; i<number_of_waypoints; i++)
    {
        fileOut << "-----"  << endl;
        fileOut << "Waypoint  " << i << ":" << endl;
        fileOut << 	"waypoint_name[i] "   << waypoint_name[i] << endl;
        fileOut << 	"nominal_IAS_at_waypoint[i] "   << waypoint_nominal_ias[i] << endl;
        fileOut << 	"MACH_at_waypoint[i] "   << waypoint_mach[i] << endl;
        fileOut << 	"waypoint_Alt[i] "   << waypoint_altitude[i].value() << endl;
        fileOut << 	"waypoint_Descent_angle_degree[i] "   << waypoint_descent_angle[i] << endl;
        fileOut << 	"waypoint_Descent_rate_knot_per_second[i] "   << waypoint_descent_rate[i] << endl;
        fileOut << 	"waypoint_x[i] "   << waypoint_x[i].value()<< endl;
        fileOut << 	"waypoint_y[i] "   << waypoint_y[i].value()<< endl;
//        fileOut << 	"Fms.Name[i] "   << fms.Name[i] << endl;
//        fileOut << 	"Fms.xWp[i] "   << fms.xWp[i]/FT_M << endl;
//        fileOut << 	"Fms.yWp[i] "   << fms.yWp[i]/FT_M << endl;
//        fileOut << 	"Fms.AltWp[i] "   << fms.AltWp[i]/FT_M << endl;
//        fileOut << 	"Fms.MACH_at_waypoint[i] "   << fms.MACH_at_waypoint[i] << endl;
//        fileOut << 	"Fms.nominal_IAS_at_waypoint[i] "   << fms.nominal_IAS_at_waypoint[i] << endl;
    }
}

int AircraftIntent::getId() const {
    return id;
}

void AircraftIntent::setId(int id_in) {
    id = id_in;
}

unsigned int AircraftIntent::getNumberOfWaypoints() const {
    return number_of_waypoints;
}

void AircraftIntent::setNumberOfWaypoints(unsigned int n)  {
    number_of_waypoints = n;
}

const string& AircraftIntent::getWaypointName(unsigned int i) const {
    return waypoint_name[i];
}

Units::MetersLength AircraftIntent::getWaypointX(unsigned int i) const {
    return Units::MetersLength(waypoint_x[i]);
}

Units::MetersLength AircraftIntent::getWaypointY(unsigned int i) const {
    return Units::MetersLength(waypoint_y[i]);
}

// helper method for copy constructor and assignment operator
void AircraftIntent::copy(const AircraftIntent &in)
{
    id = in.id;
    number_of_waypoints = in.number_of_waypoints;
    mIsLoaded = in.mIsLoaded;

    // for loop to copy all waypoint information
    for( int loop = 0; loop < MAX_NUM_WAYPOINTS; loop++)
    {
        waypoint_name[loop] = in.waypoint_name[loop];
        waypoint_y[loop] = in.waypoint_y[loop];
        waypoint_x[loop] = in.waypoint_x[loop];
        waypoint_altitude[loop] = in.waypoint_altitude[loop];
        waypoint_descent_angle[loop] = in.waypoint_descent_angle[loop];
        waypoint_nominal_ias[loop] = in.waypoint_nominal_ias[loop];
        waypoint_mach[loop] = in.waypoint_mach[loop];
        waypoint_descent_rate[loop] = in.waypoint_descent_rate[loop];
    }
    mach_transition_cas = in.mach_transition_cas;

    // loop to copy FMS values
    for(int loop2 = 0; loop2 < MAX_NUM_WAYPOINTS; loop2++)
    {
        fms.Name[loop2] = in.fms.Name[loop2];
        fms.xWp[loop2] = in.fms.xWp[loop2];
        fms.yWp[loop2] = in.fms.yWp[loop2];
        fms.LatWp[loop2] = in.fms.LatWp[loop2];
        fms.LonWp[loop2] = in.fms.LonWp[loop2];
        fms.AltWp[loop2] = in.fms.AltWp[loop2];
        fms.nominal_IAS_at_waypoint[loop2] = in.fms.nominal_IAS_at_waypoint[loop2];
        fms.MACH_at_waypoint[loop2] = in.fms.MACH_at_waypoint[loop2];
        fms.altHi[loop2] = in.fms.altHi[loop2];
        fms.altLow[loop2] = in.fms.altLow[loop2];
        fms.speedHi[loop2] = in.fms.speedHi[loop2];
        fms.speedLow[loop2] = in.fms.speedLow[loop2];
        fms.LatCp[loop2] = in.fms.LatCp[loop2];
        fms.LonCp[loop2] = in.fms.LonCp[loop2];
        fms.xCp[loop2] = in.fms.xCp[loop2];
        fms.yCp[loop2] = in.fms.yCp[loop2];
        fms.radiusCp[loop2] = in.fms.radiusCp[loop2];
    }

    //Fms.number_of_waypoints = in.Fms.number_of_waypoints;
    tangentPlaneSequence = in.tangentPlaneSequence;
    waypoints = in.waypoints;
}

bool AircraftIntent::load(DecodedStream *input)
{

    list<Waypoint> waypoint_list;

    set_stream(input);

    double tmp1;
    double tmp2;

    Units::KnotsSpeed machtransitioncasload;

    // register all the variables used by the Aircraft Intent
    register_var("number_of_waypoints",&number_of_waypoints);
    register_var("MachTransitionCas",&machtransitioncasload);


    // TODO:Get these scenario parameters removed.  They are not
    // actually used.

    register_var("plannedCruiseMach",&tmp1);
    register_var("plannedCruiseAltitude",&tmp2);

    // register the waypoint list
    register_named_list("waypoints",&waypoint_list, true);

    //do the actual reading:
    mIsLoaded = complete();

    mach_transition_cas = machtransitioncasload;

    // loads all the waypoint information from the list
    load_waypoints_from_list(waypoint_list);

    return mIsLoaded;
}

void AircraftIntent::dumpParms(std::string str) const {

    // Dumps AircraftIntent objects.
    // To use this, logger properties level must be set to DEBUG.
    //
    // str:Header string for output.


    LOG4CPLUS_DEBUG(AircraftIntent::logger,endl << str.c_str() << endl);


    LOG4CPLUS_DEBUG(AircraftIntent::logger,"mach_transition_cas "
            << Units::KnotsSpeed(mach_transition_cas).value() << endl);

    LOG4CPLUS_DEBUG(AircraftIntent::logger,"No waypoints " << number_of_waypoints << "  FMS no waypoints " << number_of_waypoints << endl << endl);

    for (unsigned int i=0; i<number_of_waypoints; i++) {

        LOG4CPLUS_DEBUG(AircraftIntent::logger,"Waypoint " << i << waypoint_name[i].c_str() << endl);
        LOG4CPLUS_DEBUG(AircraftIntent::logger,"y   x   " << waypoint_y[i] << "  " << waypoint_x[i] << endl);
        LOG4CPLUS_DEBUG(AircraftIntent::logger,"Alt     " << waypoint_altitude[i] << endl);
        LOG4CPLUS_DEBUG(AircraftIntent::logger,"des ang " << waypoint_descent_angle[i] << endl);
        LOG4CPLUS_DEBUG(AircraftIntent::logger,"nom IAS " << waypoint_nominal_ias[i] << endl);
        LOG4CPLUS_DEBUG(AircraftIntent::logger,"wpt mch " << waypoint_mach[i] << endl);
        LOG4CPLUS_DEBUG(AircraftIntent::logger,"des rte " << waypoint_descent_rate[i] << endl);

        LOG4CPLUS_DEBUG(AircraftIntent::logger,endl);

        LOG4CPLUS_DEBUG(AircraftIntent::logger,"FMS name    " << fms.Name[i].c_str() << endl);
        LOG4CPLUS_DEBUG(AircraftIntent::logger,"FMS Lat Lon " << fms.LatWp[i] << "  " << fms.LonWp[i] << endl);
        LOG4CPLUS_DEBUG(AircraftIntent::logger,"FMS y   x   " << fms.yWp[i] << "  " << fms.xWp[i] << endl);
        LOG4CPLUS_DEBUG(AircraftIntent::logger,"FMS alt z   " << fms.AltWp[i] << "  " << fms.zWp[i] << endl);
        LOG4CPLUS_DEBUG(AircraftIntent::logger,"FMS ias mch " << fms.nominal_IAS_at_waypoint[i] << "  " << fms.MACH_at_waypoint[i] << endl);
        LOG4CPLUS_DEBUG(AircraftIntent::logger,"FMS alt h l " << fms.altHi[i] << "  " << fms.altLow[i] << endl);
        LOG4CPLUS_DEBUG(AircraftIntent::logger,"FMS spd h l " << fms.speedHi[i] << "  " << fms.speedLow[i] << endl);
        LOG4CPLUS_DEBUG(AircraftIntent::logger,endl << endl);

        LOG4CPLUS_DEBUG(AircraftIntent::logger,"FMS CP Lat Lon " << fms.LatCp[i] << "  " << fms.LonCp[i] << endl);
        LOG4CPLUS_DEBUG(AircraftIntent::logger,"FMS CP y   x   " << fms.yCp[i] << "  " << fms.xCp[i] << endl);
        LOG4CPLUS_DEBUG(AircraftIntent::logger,"FMS CP radius   " << fms.radiusCp[i] << "  " << endl);
    }

}

void AircraftIntent::get_lat_lon_from_xyz(const Units::Length &xMeters, const Units::Length &yMeters, const Units::Length &zMeters, Units::Angle &lat, Units::Angle &lon) {
    /*
     * Fms struct must have been initialized first
     */
    if (number_of_waypoints < 1) {
        throw logic_error("No waypoints in AircraftIntent");
    }

    // use the ellipsoidal model
    EarthModel::LocalPositionEnu localPos;
    localPos.x = xMeters;
    localPos.y = yMeters;
    localPos.z = zMeters;

    EarthModel::GeodeticPosition geo;
    tangentPlaneSequence->convertLocalToGeodetic(localPos, geo);
    lat = geo.latitude;
    lon = geo.longitude;

    return;
}

const shared_ptr<TangentPlaneSequence> &AircraftIntent::getTangentPlaneSequence() const
{
    return tangentPlaneSequence;
}

const struct AircraftIntent::Fms& AircraftIntent::getFms() const {
    return fms;
}

pair<int, int> AircraftIntent::findCommonWaypoint(const AircraftIntent &intent) const {
    /*
     * Find the earliest common waypoint (closest to the start of own intent).
     *
     * Returns -1 if not found.
     */

    int ix = getNumberOfWaypoints() - 1;
    int tx = intent.getNumberOfWaypoints() - 1;
    int thisIndex = -1, thatIndex = -1;  // default. Will return this if no common waypoint found

    while ((ix >= 0) && (tx >= 0))
    {
        if (getWaypointName(ix).compare(intent.getWaypointName(tx)) == 0)
        {
            // Still have a match from end.
            thisIndex = ix;
            thatIndex = tx;
            ix--;
            tx--;
        } else
            break; // Exit out of loop
    }

    return pair<int, int>(thisIndex, thatIndex);
}

void AircraftIntent::insertPairAtIndex(const std::string wpname, const Units::Length &x, const Units::Length &y, const int index) {
    /*
     * The incoming point must have been validated by the caller.
     *
     * The point will be converted to a waypoint and inserted into the waypoint list that this object
     * was initialized with. Then the object will be re-initialized.
     */
    Units::Angle lat, lon;
    get_lat_lon_from_xyz(x, y, Units::ZERO_LENGTH, lat, lon);

    Waypoint wp;
    wp.setRfTurnArcRadius(Units::ZERO_LENGTH);
    wp.setWaypointLatLon(lat, lon);
    wp.setName(wpname);

    // Insert new waypoint
    std::list<Waypoint>::iterator itr = std::next(waypoints.begin(), index); // get an iterator pointing to index
    waypoints.insert(itr, wp);  // insert BEFORE iterator

    // Re-initialize this object with the new waypoints
    load_waypoints_from_list(waypoints);
    update_xyz_from_latlon_wgs84();
}
