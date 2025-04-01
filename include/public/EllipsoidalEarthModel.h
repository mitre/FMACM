// ****************************************************************************
// NOTICE
//
// This work was produced for the U.S. Government under Contract 693KA8-22-C-00001
// and is subject to Federal Aviation Administration Acquisition Management System
// Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV (Oct. 1996).
//
// The contents of this document reflect the views of the author and The MITRE
// Corporation and do not necessarily reflect the views of the Federal Aviation
// Administration (FAA) or the Department of Transportation (DOT). Neither the FAA
// nor the DOT makes any warranty or guarantee, expressed or implied, concerning
// the content or accuracy of these views.
//
// For further information, please contact The MITRE Corporation, Contracts Management
// Office, 7515 Colshire Drive, McLean, VA 22102-7539, (703) 983-6000.
//
// 2023 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include "public/EarthModel.h"
#include <scalar/Area.h>
#include "public/LocalTangentPlane.h"
#include "public/Logging.h"
#include "public/WGS84EarthModelConstants.h"

class EllipsoidalEarthModel final : public EarthModel {
  public:
   EllipsoidalEarthModel()
      : m_semi_major_radius_squared(aaesim::open_source::WGS84_SEMIMAJOR_AXIS *
                                    aaesim::open_source::WGS84_SEMIMAJOR_AXIS),
        m_eccentricity_4(aaesim::open_source::WGS84_ECCENTRICITY_SQUARED *
                         aaesim::open_source::WGS84_ECCENTRICITY_SQUARED) {}

   ~EllipsoidalEarthModel() = default;

   void ConvertGeodeticToAbsolute(const EarthModel::GeodeticPosition &geo,
                                  EarthModel::AbsolutePositionEcef &ecef) const override;

   void ConvertAbsoluteToGeodetic(const EarthModel::AbsolutePositionEcef &ecef,
                                  EarthModel::GeodeticPosition &geo) const override;

   std::shared_ptr<LocalTangentPlane> MakeEnuConverter(const GeodeticPosition &pointOfTangencyGeo,
                                                       const LocalPositionEnu &pointOfTangencyEnu) const override;

  private:
   inline static log4cplus::Logger m_logger{log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("EllipsoidalEarthModel"))};
   const Units::Area m_semi_major_radius_squared;
   const double m_eccentricity_4;
};
