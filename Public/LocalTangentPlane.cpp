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
 * LocalTangentPlane.cpp
 *
 *  Created on: Jun 25, 2015
 *      Author: klewis
 */

#include "public/LocalTangentPlane.h"
#include "math/CustomMath.h"

using namespace std;

log4cplus::Logger LocalTangentPlane::logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("LocalTangentPlane"));
const double LocalTangentPlane::identity3x3[3][3] = { { 1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
const double yzx[3][3] = {{0, 0, 1}, {1, 0, 0}, {0, 1, 0}};
const double zxy[3][3] = {{0, 1, 0}, {0, 0, 1}, {1, 0, 0}};

LocalTangentPlane::LocalTangentPlane(const EarthModel *earthModel,
		const EarthModel::AbsolutePositionEcef &ecefPointOfTangency,
		const EarthModel::LocalPositionEnu &enuPointOfTangency) :
	earthModel(earthModel),
	pointOfTangencyEcef(ecefPointOfTangency),
	pointOfTangencyEnu(enuPointOfTangency),
	ecefToEnu(DMatrix((double **)&identity3x3, 0, 2, 0, 2)),
	enuToEcef(DMatrix((double **)&identity3x3, 0, 2, 0, 2))
{
}

LocalTangentPlane::~LocalTangentPlane() {
}

void LocalTangentPlane::initializeRotationForGeodeticOrigin() {
	ecefToEnu = DMatrix((double **)zxy, 0, 2, 0, 2);
	enuToEcef = DMatrix((double **)yzx, 0, 2, 0, 2);
}

void LocalTangentPlane::rotateEnuFrame(const double x, const double y,
		const double z, const Units::Angle theta) {
	DMatrix &r = createRotationMatrix(x, y, z, theta);
	DMatrix &ecefToEnu1 = ecefToEnu * r;
	ecefToEnu = ecefToEnu1;
	delete &r;
	delete &ecefToEnu1;

	DMatrix &rInv = createRotationMatrix(x, y, z, -theta);
	DMatrix &enuToEcef1 = rInv * enuToEcef;
	enuToEcef = enuToEcef1;
	delete &rInv;
	delete &enuToEcef1;
}

void LocalTangentPlane::convertGeodeticToAbsolute(
		const EarthModel::GeodeticPosition &geo,
		EarthModel::AbsolutePositionEcef &ecef) const {
	earthModel->convertGeodeticToAbsolute(geo, ecef);
}
void LocalTangentPlane::convertAbsoluteToGeodetic(
		const EarthModel::AbsolutePositionEcef &ecef,
		EarthModel::GeodeticPosition &geo) const {
	earthModel->convertAbsoluteToGeodetic(ecef, geo);
}

void LocalTangentPlane::printCoordinates(string title, Units::Length x, Units::Length y, Units::Length z) {
	LOG4CPLUS_TRACE(logger, title << " (" <<
			Units::MetersLength(x).value() << "," <<
			Units::MetersLength(y).value() << "," <<
			Units::MetersLength(z).value() << ")");
}

/**
 * Converts local coordinates to absolute.
 */
void LocalTangentPlane::convertLocalToAbsolute(
		const EarthModel::LocalPositionEnu &enu,
		EarthModel::AbsolutePositionEcef &ecef) const {
	Units::Length x1 = enu.x - pointOfTangencyEnu.x;
	Units::Length y1 = enu.y - pointOfTangencyEnu.y;
	Units::Length z1 = enu.z - pointOfTangencyEnu.z;
	printCoordinates("Local relative: ", x1, y1, z1);

	ecef.x = enuToEcef[0][0] * x1 +
			enuToEcef[0][1] * y1 +
			enuToEcef[0][2] * z1;
	ecef.y = enuToEcef[1][0] * x1 +
			enuToEcef[1][1] * y1 +
			enuToEcef[1][2] * z1;
	ecef.z = enuToEcef[2][0] * x1 +
			enuToEcef[2][1] * y1 +
			enuToEcef[2][2] * z1;

	printCoordinates("Rotated relative: ", ecef.x, ecef.y, ecef.z);

	ecef.x += pointOfTangencyEcef.x;
	ecef.y += pointOfTangencyEcef.y;
	ecef.z += pointOfTangencyEcef.z;

	printCoordinates("Absolute: ", ecef.x, ecef.y, ecef.z);

}

/**
 * Converts absolute coordinates to local.
 */
void LocalTangentPlane::convertAbsoluteToLocal(
		const EarthModel::AbsolutePositionEcef &ecef,
		EarthModel::LocalPositionEnu &enu) const {
	Units::Length x1 = ecef.x - pointOfTangencyEcef.x;
	Units::Length y1 = ecef.y - pointOfTangencyEcef.y;
	Units::Length z1 = ecef.z - pointOfTangencyEcef.z;

	enu.x = ecefToEnu[0][0] * x1 +
			ecefToEnu[0][1] * y1 +
			ecefToEnu[0][2] * z1;
	enu.y = ecefToEnu[1][0] * x1 +
			ecefToEnu[1][1] * y1 +
			ecefToEnu[1][2] * z1;
	enu.z = ecefToEnu[2][0] * x1 +
			ecefToEnu[2][1] * y1 +
			ecefToEnu[2][2] * z1;

	enu.x += pointOfTangencyEnu.x;
	enu.y += pointOfTangencyEnu.y;
	enu.z += pointOfTangencyEnu.z;
}

void LocalTangentPlane::convertGeodeticToLocal(
		const EarthModel::GeodeticPosition &geo,
		EarthModel::LocalPositionEnu &enu) const {
	EarthModel::AbsolutePositionEcef temp;
	earthModel->convertGeodeticToAbsolute(geo, temp);
	convertAbsoluteToLocal(temp, enu);
}
void LocalTangentPlane::convertLocalToGeodetic(
		const EarthModel::LocalPositionEnu &enu,
		EarthModel::GeodeticPosition &geo) const {
	EarthModel::AbsolutePositionEcef temp;
	convertLocalToAbsolute(enu, temp);
	earthModel->convertAbsoluteToGeodetic(temp, geo);
}

const EarthModel::LocalPositionEnu& LocalTangentPlane::getPointOfTangencyEnu() const {
	return pointOfTangencyEnu;
}

const EarthModel::AbsolutePositionEcef& LocalTangentPlane::getPointOfTangencyEcef() const {
	return pointOfTangencyEcef;
}
