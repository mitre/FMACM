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
// 2022 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include "public/ShapeOnEllipsoid.h"
#include "public/LatitudeLongitudePoint.h"
#include "geolib/Shape.h"

namespace aaesim {
   class ArcOnEllipsoid : public ShapeOnEllipsoid {
    public:

      ArcOnEllipsoid() = default;

      ArcOnEllipsoid(const geolib_idealab::Arc &arc);

      SHAPE_TYPE GetShapeType() const override;

      Units::Length GetShapeLength() const override;

      Units::SignedAngle GetArcAngularExtent() const;

      Units::SignedAngle GetCourseEnuTangentToStartPoint() const;

      Units::SignedAngle GetForwardCourseEnuAtStartPoint() const override;

      Units::SignedAngle GetCourseEnuTangentToEndPoint() const;

      Units::SignedAngle GetForwardCourseEnuAtEndPoint() const override;

      LatitudeLongitudePoint GetCenterPoint() const;

      LatitudeLongitudePoint GetStartPoint() const override;

      LatitudeLongitudePoint GetEndPoint() const override;

      Units::Length GetRadius() const;

      /**
       * @return the angle, in ENU coords, that points from the center point to the start point of the arc.
       */
      Units::SignedAngle GetStartAzimuthEnu() const;

      /**
       * @return the angle, in ENU coords, that points from the center point to the end point of the arc.
       */
      Units::SignedAngle GetEndAzimuthEnu() const;

      geolib_idealab::ArcDirection GetArcDirection() const;

      bool IsPointOnShape(const LatitudeLongitudePoint &test_point) const override;

      bool IsPointInsideArc(const LatitudeLongitudePoint &test_point) const;

      DIRECTION_RELATIVE_TO_SHAPE GetRelativeDirection(const LatitudeLongitudePoint &latitude_longitude_point) const override;

      Units::Length GetDistanceToEndPoint(const LatitudeLongitudePoint &latitude_longitude_point) const override;

      LatitudeLongitudePoint GetNearestPointOnShape(const LatitudeLongitudePoint &latitude_longitude_point) const override;

      LatitudeLongitudePoint CalculatePointAtDistanceFromStartPoint(const Units::Length &distance_along_shape_from_start_point) const override;

      std::pair<Units::SignedAngle,
                LatitudeLongitudePoint> CalculateCourseAtDistanceFromStartPoint(const Units::Length &distance_along_shape_from_start_point) const override;

    protected:
      Units::Length CalculateDistanceFromPointOnShapeToEnd(const LatitudeLongitudePoint &point_on_shape) const override;

    private:
      static log4cplus::Logger m_logger;
      geolib_idealab::Arc m_arc_primitive;

   };

   inline Units::SignedAngle ArcOnEllipsoid::GetForwardCourseEnuAtStartPoint() const {
      return GetCourseEnuTangentToStartPoint();
   }

   inline Units::SignedAngle ArcOnEllipsoid::GetForwardCourseEnuAtEndPoint() const {
      return GetCourseEnuTangentToEndPoint();
   }

   inline ShapeOnEllipsoid::SHAPE_TYPE ArcOnEllipsoid::GetShapeType() const {
      return ARC;
   }

}