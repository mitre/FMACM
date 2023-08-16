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

#include <string>
#include "public/HorizontalTurnPath.h"

class HorizontalPath {
  public:
   enum SegmentType { STRAIGHT, TURN, UNSET };

   HorizontalPath();

   virtual ~HorizontalPath();

   bool operator==(const HorizontalPath &that) const;
   double GetXPositionMeters() const;
   double GetYPositionMeters() const;
   void SetXYPositionMeters(double x_position_meters, double y_position_meters);

   SegmentType m_segment_type;
   double m_path_length_cumulative_meters;
   double m_path_course;
   HorizontalTurnPath m_turn_info;

   std::string GetSegmentTypeAsString() const {
      std::string return_this = "UNSET";
      switch (m_segment_type) {
         case STRAIGHT:
            return_this = "STRAIGHT";
            break;

         case TURN:
            return_this = "TURN";
            break;

         default:
            break;
      }
      return return_this;
   }

  private:
   double m_x_position_meters;
   double m_y_position_meters;
};
