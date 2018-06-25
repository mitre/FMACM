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

#include "public/ADSBSVReport.h"
#include "math/CustomMath.h"

namespace Sensor {

    namespace ADSB {
    	const ADSBSVReport ADSBSVReport::blank_report;

        ADSBSVReport::ADSBSVReport(void) {
        	clear();
        }

        ADSBSVReport::~ADSBSVReport(void) {

        }

        // Copy constructor
        ADSBSVReport::ADSBSVReport(const ADSBSVReport &in) {
            id = in.id;
            time = in.time;
            x = in.x;
            y = in.y;
            z = in.z;
            xd = in.xd;
            yd = in.yd;
            zd = in.zd;
            nacp = in.nacp;
            nacv = in.nacv;
            nicp = in.nicp;
            nicv = in.nicv;
            mHasPosition = in.mHasPosition;
            mHasVelocity = in.mHasVelocity;
            horizontalPositionQuantum = in.horizontalPositionQuantum;
            horizontalVelocityQuantum = in.horizontalVelocityQuantum;
            verticalPositionQuantum = in.verticalPositionQuantum;
            verticalVelocityQuantum = in.verticalVelocityQuantum;
            timeQuantum = in.timeQuantum;
        }

        // Assignment operator
        ADSBSVReport &ADSBSVReport::operator=(const ADSBSVReport &in) {
            if (this != &in) {
                id = in.id;
                time = in.time;
                x = in.x;
                y = in.y;
                z = in.z;
                xd = in.xd;
                yd = in.yd;
                zd = in.zd;
                nacp = in.nacp;
                nacv = in.nacv;
                nicp = in.nicp;
                nicv = in.nicv;
                mHasPosition = in.mHasPosition;
                mHasVelocity = in.mHasVelocity;
                horizontalPositionQuantum = in.horizontalPositionQuantum;
                horizontalVelocityQuantum = in.horizontalVelocityQuantum;
                verticalPositionQuantum = in.verticalPositionQuantum;
                verticalVelocityQuantum = in.verticalVelocityQuantum;
                timeQuantum = in.timeQuantum;
            }
            return *this;
        }

        // Clear
        void ADSBSVReport::clear(void) {
            // Reset reports to original condition
            id = -1;
            time = Units::SecondsTime(-1);
            x = y = z = Units::FeetLength(0);
            xd = yd = zd = Units::FeetPerSecondSpeed(0);
            nacp = nacv = nicp = nicv = 0;
            mHasPosition = mHasVelocity = false;
            horizontalPositionQuantum = verticalPositionQuantum = Units::FeetLength(-1);
            horizontalVelocityQuantum = verticalVelocityQuantum = Units::FeetPerSecondSpeed(-1);
        }

        bool ADSBSVReport::operator==(const ADSBSVReport &in) {
        	// rewritten 4/14/16 to ignore fields not included in message

        	// fields which must always match
        	if (this->id != in.id || this->time != in.time ||
        			this->mHasPosition != in.mHasPosition ||
        			this->mHasVelocity != in.mHasVelocity) {
        		return false;
        	}

        	// position fields
        	if (this->mHasPosition) {
        		if (this->x != in.x || this->y != in.y || this->z != in.z ||
        				this->nacp != in.nacp || this->nicp != in.nicp ||
        				this->horizontalPositionQuantum != in.horizontalPositionQuantum ||
        				this->verticalPositionQuantum != in.verticalPositionQuantum) {
        			return false;
        		}
        	}

        	// velocity fields
        	if (this->mHasVelocity) {
        		if (this->xd != in.xd || this->yd != in.yd || this->zd != in.zd ||
        				this->nacv != in.nacv || this->nicv != in.nicv ||
        				this->horizontalVelocityQuantum != in.horizontalVelocityQuantum ||
        				this->verticalVelocityQuantum != in.verticalVelocityQuantum) {
        			return false;
        		}
        	}

        	// everything matches that needs to
        	return true;
        }


        bool ADSBSVReport::isHasPosition() const {
        	return mHasPosition;
        }

        void ADSBSVReport::setHasPosition(bool hasPosition) {
        	mHasPosition = hasPosition;
        }

        bool ADSBSVReport::isHasVelocity() const {
        	return mHasVelocity;
        }

        void ADSBSVReport::setHasVelocity(bool hasVelocity) {
        	mHasVelocity = hasVelocity;
        }

        Units::FeetLength ADSBSVReport::getHorizontalPositionQuantum() const {
        	return horizontalPositionQuantum;
        }

        Units::FeetPerSecondSpeed ADSBSVReport::getHorizontalVelocityQuantum() const {
        	return horizontalVelocityQuantum;
        }


        void ADSBSVReport::setId(int id) {
        	this->id = id;
        }

        void ADSBSVReport::setTime(Units::Time time) {
        	this->time = time;
        }

        int ADSBSVReport::getId() const {
        	return id;
        }

        int ADSBSVReport::getNacp() const {
        	return nacp;
        }

        int ADSBSVReport::getNacv() const {
        	return nacv;
        }

        int ADSBSVReport::getNicp() const {
        	return nicp;
        }

        int ADSBSVReport::getNicv() const {
        	return nicv;
        }

        Units::SecondsTime ADSBSVReport::getTime() const {
        	return time;
        }

        Units::SecondsTime ADSBSVReport::getTimeQuantum() const {
        	return timeQuantum;
        }

        Units::FeetLength ADSBSVReport::getVerticalPositionQuantum() const {
        	return verticalPositionQuantum;
        }

        Units::FeetPerSecondSpeed ADSBSVReport::getVerticalVelocityQuantum() const {
        	return verticalVelocityQuantum;
        }

        Units::FeetLength ADSBSVReport::getX() const {
        	return x;
        }

        Units::FeetPerSecondSpeed ADSBSVReport::getXd() const {
        	return xd;
        }

        Units::FeetLength ADSBSVReport::getY() const {
        	return y;
        }

        Units::FeetPerSecondSpeed ADSBSVReport::getYd() const {
        	return yd;
        }

        Units::FeetLength ADSBSVReport::getZ() const {
        	return z;
        }

        Units::FeetPerSecondSpeed ADSBSVReport::getZd() const {
        	return zd;
        }

        void ADSBSVReport::setNacp(int nacp) {
        	this->nacp = nacp;
        }

        void ADSBSVReport::setNacv(int nacv) {
        	this->nacv = nacv;
        }

        void ADSBSVReport::setNicp(int nicp) {
        	this->nicp = nicp;
        }

        void ADSBSVReport::setNicv(int nicv) {
        	this->nicv = nicv;
        }

        void ADSBSVReport::setPosition(const Units::Length x, const Units::Length y,
        		const Units::Length z, const Units::Length horizontalQuantum,
        		const Units::Length verticalQuantum) {
        	this->horizontalPositionQuantum = horizontalQuantum;
        	this->x = quantize(x, horizontalPositionQuantum);
        	this->y = quantize(y, horizontalPositionQuantum);
        	this->verticalPositionQuantum = verticalQuantum;
        	this->z = quantize(z, verticalPositionQuantum);
        }

        void ADSBSVReport::setVelocity(const Units::Speed xd, const Units::Speed yd,
        		const Units::Speed zd, const Units::Speed horizontalQuantum,
        		const Units::Speed verticalQuantum) {
        	this->horizontalVelocityQuantum = horizontalQuantum;
        	this->xd = quantize(xd, horizontalVelocityQuantum);
        	this->yd = quantize(yd, horizontalVelocityQuantum);
        	this->verticalVelocityQuantum = verticalQuantum;
        	this->zd = quantize(zd, verticalVelocityQuantum);
        }
    }
}

