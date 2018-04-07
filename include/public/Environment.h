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
 * Environment.h
 *
 *  Created on: Jun 25, 2015
 *      Author: klewis
 */

#pragma once
class Wind;
#include "public/EarthModel.h"
#include "public/Atmosphere.h"

class Environment {
public:
	static Environment *getInstance();
	Wind *getWind() const;
	EarthModel *getEarthModel() const;
	Atmosphere *getAtmosphere() const;

private:
	static Environment *mInstance;
	Wind *wind;
	EarthModel *earthModel;
	Atmosphere *atmosphere;
	Environment();
	virtual ~Environment();
};

// external C functions with short names
Environment *ENVIRONMENT();
EarthModel *EARTH_MODEL();
Wind *WIND();
Atmosphere *ATMOSPHERE();
