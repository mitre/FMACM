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

#include <scalar/Length.h>
#include "LatitudeLongitudePoint.h"

/*
 * An interface class for shapes on an ellipsoid. Lines, arcs, and any other defined shape
 * should inherit from this class.
 */
namespace aaesim {
   class ShapeOnEllipsoid {
    public:

      enum SHAPE_TYPE {
         NO_SHAPE = INT32_MIN,
         LINE = 0,
         ARC = 1
      };

      enum DIRECTION_RELATIVE_TO_SHAPE {
         UNSET = INT32_MIN,
         LEFT_OF_SHAPE = -1,
         RIGHT_OF_SHAPE = 1,
         ON_SHAPE = 0
      };

      ShapeOnEllipsoid() = default;

      ~ShapeOnEllipsoid() = default;

      virtual SHAPE_TYPE GetShapeType() const {
         return NO_SHAPE;
      };

      virtual Units::SignedAngle GetForwardCourseEnuAtStartPoint() const {
         return Units::ZERO_ANGLE;
      };

      virtual Units::SignedAngle GetForwardCourseEnuAtEndPoint() const {
         return Units::ZERO_ANGLE;
      }

      virtual LatitudeLongitudePoint GetStartPoint() const {
         return LatitudeLongitudePoint();
      };

      virtual LatitudeLongitudePoint GetEndPoint() const {
         return LatitudeLongitudePoint();
      };

      virtual LatitudeLongitudePoint CalculatePointAtDistanceFromStartPoint(const Units::Length &distance_along_shape_from_start_point) const {
         return LatitudeLongitudePoint();
      }

      virtual std::pair<Units::SignedAngle, LatitudeLongitudePoint> CalculateCourseAtDistanceFromStartPoint(const Units::Length &distance_along_shape_from_start_point) const {
         return std::make_pair(Units::ZERO_ANGLE,LatitudeLongitudePoint());
      }

      /**
       * Returns the length of the shape along the ellipsoid.
       *
       * @return a Units::Length object. Any negative value indicates a failed calculation.
       */
      virtual Units::Length GetShapeLength() const {
         return Units::negInfinity();
      }

      /**
       * For a given latitude_longitude_point, determines the side relative to the shape's direction (defined by the start and end point).
       *
       * @param latitude_longitude_point
       * @return see return enum for possibilities
       */
      virtual DIRECTION_RELATIVE_TO_SHAPE GetRelativeDirection(const LatitudeLongitudePoint &latitude_longitude_point) const {
         return UNSET;
      }

      /**
       * Test a point for being on the shape.
       *
       * Does not project to the shape.
       *
       * @param test_point
       * @return true if on the shape, false otherwise
       */
      virtual bool IsPointOnShape(const LatitudeLongitudePoint &test_point) const {
         return false;
      }

      /**
       * For a test point which may or may not be on the shape, return the distance along the shape to the end
       * point. If the test point is not on the shape, it will be projected to the shape and that point used to
       * determine the length to return.
       *
       * @param latlon_point
       * @return a Units::Length object. Any negative value indicates a failed calculation.
       */
      virtual Units::Length GetDistanceToEndPoint(const LatitudeLongitudePoint &latlon_point) const {
         Units::Length distance_to_end = Units::negInfinity();
         if (IsPointOnShape(latlon_point)) {
            distance_to_end = CalculateDistanceFromPointOnShapeToEnd(latlon_point);
         } else {
            const LatitudeLongitudePoint perpendicular_point = GetNearestPointOnShape(latlon_point);
            distance_to_end = CalculateDistanceFromPointOnShapeToEnd(perpendicular_point);
         }

         return distance_to_end;
      }

      /**
       * Project from point_not_on_shape to the shape, making a perpendicular line through the shape. Return that point that has been
       * calculated as the projected point.
       *
       * This returned point also defines the nearest location on the shape to point_not_on_shape.
       *
       * @param point_not_on_shape
       * @return
       */
      virtual LatitudeLongitudePoint GetNearestPointOnShape(const LatitudeLongitudePoint &point_not_on_shape) const {
         return LatitudeLongitudePoint();
      }

    protected:
      /**
       * This is intentionally not public.
       *
       * @see GetDistanceToEndPoint() for the public implementation
       * @param point_on_shape. The caller is responsible for ensuring that this point is on the shape.
       * @return a Units::Length object
       */
      virtual Units::Length CalculateDistanceFromPointOnShapeToEnd(const LatitudeLongitudePoint &point_on_shape) const {
         return Units::negInfinity();
      }
   };
}
