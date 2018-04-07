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
 * Environment.cpp
 *
 *  Created on: Jun 25, 2015
 *      Author: klewis
 */

#include "public/Environment.h"
#include "public/EllipsoidalEarthModel.h"

Environment *Environment::mInstance = NULL;

Environment::Environment() :
		// wind(new Wind()),
		earthModel(new EllipsoidalEarthModel()),
		atmosphere(new Atmosphere())
{
}
Environment::~Environment()
{
   //delete wind;
   delete earthModel;
   delete atmosphere;
}
Environment *Environment::getInstance()
{
   if(mInstance == NULL)
      mInstance = new Environment();
   return mInstance;
}

Wind *Environment::getWind() const
{
   return wind;
}

EarthModel *Environment::getEarthModel() const  {
	return earthModel;
}

Atmosphere *Environment::getAtmosphere() const {
	return atmosphere;
}

Environment *ENVIRONMENT() {
	return Environment::getInstance();
}

EarthModel *EARTH_MODEL() {
	return Environment::getInstance()->getEarthModel();
}

Wind *WIND() {
	return Environment::getInstance()->getWind();
}

Atmosphere *ATMOSPHERE() {
	return Environment::getInstance()->getAtmosphere();
}
