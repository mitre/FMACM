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

#pragma once

#include <Length.h>
#include <Speed.h>
#include <Time.h>

namespace Sensor {

    namespace ADSB {

        class ADSBSVReport {
        public:
            ADSBSVReport(void);

            ~ADSBSVReport(void);

            // Copy constructor and overload operator
            ADSBSVReport(const ADSBSVReport &in);

            ADSBSVReport &operator=(const ADSBSVReport &in);

            bool operator==(const ADSBSVReport &in);

            void clear(void);

            bool isHasPosition() const;
            void setHasPosition(bool hasPosition);
            bool isHasVelocity() const;
            void setHasVelocity(bool hasVelocity);
            Units::FeetLength getHorizontalPositionQuantum() const;
            Units::FeetPerSecondSpeed getHorizontalVelocityQuantum() const;
            int getId() const;
            int getNacp() const;
            int getNacv() const;
            int getNicp() const;
            int getNicv() const;
            Units::SecondsTime getTime() const;
            Units::SecondsTime getTimeQuantum() const;
            Units::FeetLength getVerticalPositionQuantum() const;
            Units::FeetPerSecondSpeed getVerticalVelocityQuantum() const;
            Units::FeetLength getX() const;
            Units::FeetPerSecondSpeed getXd() const;
            Units::FeetLength getY() const;
            Units::FeetPerSecondSpeed getYd() const;
            Units::FeetLength getZ() const;
            Units::FeetPerSecondSpeed getZd() const;

            void setId(int id);
            void setTime(Units::Time time);

            void setPosition(const Units::Length x, const Units::Length y, const Units::Length z,
            		const Units::Length horizontalQuantum, const Units::Length verticalQuantum);
            void setVelocity(const Units::Speed xd, const Units::Speed yd, const Units::Speed zd,
            		const Units::Speed horizontalQuantum, const Units::Speed verticalQuantum);

            void setNacp(int nacp);
            void setNacv(int nacv);
            void setNicp(int nicp);
            void setNicv(int nicv);

            static const ADSBSVReport blank_report;

//Input Data:
            //none


//Other Data:
        private:
            int id;
            Units::SecondsTime time;
            Units::FeetLength x, y, z;
            Units::FeetPerSecondSpeed xd, yd, zd;
            int nacp, nacv, nicp, nicv;
            bool mHasPosition, mHasVelocity;
            Units::FeetLength horizontalPositionQuantum, verticalPositionQuantum;
            Units::FeetPerSecondSpeed horizontalVelocityQuantum, verticalVelocityQuantum;
            Units::SecondsTime timeQuantum;
        };

    }
}
