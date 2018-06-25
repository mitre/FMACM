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

#include "public/VerticalPath.h"
#include "utility/constants.h"

using namespace std;

VerticalPath::VerticalPath(void)
{
}

VerticalPath::~VerticalPath(void)
{
}

// copy constructor and assignment operator for vertical paths.
VerticalPath::VerticalPath(const VerticalPath &in)
{
    copy(in);
}

VerticalPath& VerticalPath::operator=(const VerticalPath &in)
{
    if (this != &in) {
        // clear old data
        x.clear();
        h.clear();
        v.clear();
        h_dot.clear();
        v_dot.clear();
        theta.clear();
        gs.clear();
        time.clear();
        mass.clear();
        vwe.clear();
        vwn.clear();

        // copy new values
        copy(in);
    }

    return *this;
}

// helper method for copy and assignment operations
void VerticalPath::copy(const VerticalPath &in)
{
    append(in);
}

// method to append another Trajectory to this one
void VerticalPath::append(const VerticalPath &in)
{
    // loop to add the given list to the end of the current list
    for( int index = 0; index < (int)in.x.size(); index++)
    {
        x.push_back(in.x[index]);
        h.push_back(in.h[index]);
        v.push_back(in.v[index]);
        h_dot.push_back(in.h_dot[index]);
        v_dot.push_back(in.v_dot[index]);
        theta.push_back(in.theta[index]);
        gs.push_back(in.gs[index]);
        time.push_back(in.time[index]);
        mass.push_back(in.mass[index]);
        vwe.push_back(in.vwe[index]);
        vwn.push_back(in.vwn[index]);
    }
}

// operator+= to act as mathematical append operation
void VerticalPath::operator+=(const VerticalPath &in)
{
    this->append(in);
}

// Generic == implementation.
bool VerticalPath::operator==(const VerticalPath &obj) const
{

    bool match = (this->x.size() == obj.x.size());
    match = match && (this->h.size() == obj.h.size());
    match = match && (this->v.size() == obj.v.size());
    match = match && (this->h_dot.size() == obj.h_dot.size());
    match = match && (this->v_dot.size() == obj.v_dot.size());
    match = match && (this->theta.size() == obj.theta.size());
    match = match && (this->gs.size() == obj.gs.size());
    match = match && (this->time.size() == obj.time.size());
    match = match && (this->mass.size() == obj.mass.size());
    match = match && (this->vwe.size() == obj.vwe.size());
    match = match && (this->vwn.size() == obj.vwn.size());


    for (auto ix = 0; match && (ix < x.size()); ix++)
    {
        match = match && (this->x[ix] == obj.x[ix]);
    }


    for (auto ix = 0; match && (ix < h.size()); ix++)
    {
        match = match && (this->h[ix] == obj.h[ix]);
    }


    for (auto ix = 0; match && (ix < v.size()); ix++)
    {
        match = match && (this->v[ix] == obj.v[ix]);
    }


    for (auto ix = 0; match && (ix < h_dot.size()); ix++)
    {
        match = match && (this->h_dot[ix] == obj.h_dot[ix]);
    }


    for (auto ix = 0; match && (ix < v_dot.size()); ix++)
    {
        match = match && (this->v_dot[ix] == obj.v_dot[ix]);
    }


    for (auto ix = 0; match && (ix < theta.size()); ix++)
    {
        match = match && (this->theta[ix] == obj.theta[ix]);
    }


    for (auto ix = 0; match && (ix < gs.size()); ix++)
    {
        match = match && (this->gs[ix] == obj.gs[ix]);
    }


    for (auto ix = 0; match && (ix < time.size()); ix++)
    {
        match = match && (this->time[ix] == obj.time[ix]);
    }


    for (auto ix = 0; match && (ix < mass.size()); ix++)
    {
        match = match && (this->mass[ix] == obj.mass[ix]);
    }

    for (auto ix = 0; match && (ix < vwe.size()); ix++)
    {
        match = match && (this->vwe[ix] == obj.vwe[ix]);
    }

    for (auto ix = 0; match && (ix < vwn.size()); ix++)
    {
        match = match && (this->vwn[ix] == obj.vwn[ix]);
    }

    return match;

}

std::vector<double> VerticalPath::getWindVelocityEast() {
    std::vector<double> vwe_mps_vector;
    for (auto i = vwe.begin(); i != vwe.end(); ++i) {
        vwe_mps_vector.push_back(Units::MetersPerSecondSpeed(*i).value());
    }

    return vwe_mps_vector;
}

std::vector<double> VerticalPath::getWindVelocityNorth() {
    std::vector<double> vwn_mps_vector;
    for (auto i = vwe.begin(); i != vwe.end(); ++i) {
        vwn_mps_vector.push_back(Units::MetersPerSecondSpeed(*i).value());
    }

    return vwn_mps_vector;
}
