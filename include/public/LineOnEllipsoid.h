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
// (c) 2026 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <scalar/Length.h>

#include <utility>

#include "public/LatitudeLongitudePoint.h"
#include "public/ShapeOnEllipsoid.h"

namespace aaesim {
class LineOnEllipsoid final : public ShapeOnEllipsoid {
  public:
   LineOnEllipsoid() = default;

   LineOnEllipsoid(const geolib_idealab::Geodesic &geodesic);

   ~LineOnEllipsoid() = default;

   kShapeType GetShapeType() const override;

   Units::SignedAngle GetForwardCourseEnuAtStartPoint() const override;

   Units::SignedAngle GetForwardCourseEnuAtEndPoint() const override;

   Units::Length GetShapeLength() const override;

   LatitudeLongitudePoint GetStartPoint() const override;

   LatitudeLongitudePoint GetEndPoint() const override;

   const geolib_idealab::Geodesic &GetGeolibPrimitiveGeodesic() const;

   const geolib_idealab::LineType GetLineType() const;

   bool IsPointOnShape(const LatitudeLongitudePoint &test_point) const override;

   LineOnEllipsoid CreateExtendedLine(const Units::Length extended_distance) const;

   kDirectionRelativeToShape GetRelativeDirection(const LatitudeLongitudePoint &point_not_on_shape) const override;

   Units::Length GetDistanceToEndPoint(const LatitudeLongitudePoint &latitude_longitude_point) const override;

   static LineOnEllipsoid CreateFromPoints(const aaesim::LatitudeLongitudePoint &start_point,
                                           const aaesim::LatitudeLongitudePoint &end_point);

   LatitudeLongitudePoint GetNearestPointOnShape(const LatitudeLongitudePoint &point_not_on_shape) const override;

   LatitudeLongitudePoint CalculatePointAtDistanceFromStartPoint(
         const Units::Length &distance_along_shape_from_start_point) const override;

   std::pair<Units::SignedAngle, LatitudeLongitudePoint> CalculateCourseAtDistanceFromStartPoint(
         const Units::Length &distance_along_shape_from_start_point) const override;

  protected:
   Units::Length CalculateDistanceFromPointOnShapeToEnd(const LatitudeLongitudePoint &point_on_shape) const override;

  private:
   static log4cplus::Logger m_logger;
   void ComputeUnitVectorNormalToLineStartEnd();

   geolib_idealab::Geodesic geolib_geodesic_{};
   EarthModel::AbsolutePositionEcef unit_vector_normal_to_line_start_end_;
};

inline ShapeOnEllipsoid::kShapeType LineOnEllipsoid::GetShapeType() const { return LINE; }

}  // namespace aaesim
