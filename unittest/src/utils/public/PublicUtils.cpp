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

#include "utils/public/PublicUtils.h"
#include "public/SingleTangentPlaneSequence.h"

using namespace std;
using namespace aaesim::open_source;

std::vector<HorizontalPath> aaesim::test::utils::PublicUtils::CreateStraightHorizontalPath(Quadrant quadrant) {
   static const Units::MetersLength unity = Units::MetersLength(1);
   double course_radians = INFINITY;
   Units::MetersLength x_sign(0), y_sign(0);

   switch (quadrant) {
      case Quadrant::FIRST:
         x_sign = unity;
         y_sign = unity;
         course_radians = M_PI / 4;
         break;
      case Quadrant ::SECOND:
         x_sign = -unity;
         y_sign = unity;
         course_radians = 3 * M_PI / 4;
         break;
      case Quadrant ::THIRD:
         x_sign = -unity;
         y_sign = -unity;
         course_radians = -3 * M_PI / 4;
         break;
      case Quadrant ::FOURTH:
         x_sign = unity;
         y_sign = -unity;
         course_radians = -M_PI / 4;
         break;
      default:
         break;
   }

   vector<HorizontalPath> horizontal_traj;
   HorizontalPath hp0, hp1, hp2;
   hp0.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
   hp0.SetXYPositionMeters(0 * unity.value(), 0 * unity.value());
   hp0.m_path_length_cumulative_meters = 0;
   hp0.m_path_course = course_radians;
   horizontal_traj.push_back(hp0);
   hp1.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
   hp1.SetXYPositionMeters(1 * unity.value() * x_sign.value(), 1 * unity.value() * y_sign.value());
   hp1.m_path_length_cumulative_meters = hp0.m_path_length_cumulative_meters + sqrt(2);
   hp1.m_path_course = course_radians;
   horizontal_traj.push_back(hp1);
   hp2.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
   hp2.SetXYPositionMeters(2 * unity.value() * x_sign.value(), 2 * unity.value() * y_sign.value());
   hp2.m_path_length_cumulative_meters = hp1.m_path_length_cumulative_meters + sqrt(2);
   hp2.m_path_course = course_radians;
   horizontal_traj.push_back(hp2);

   return horizontal_traj;
}

AircraftIntent aaesim::test::utils::PublicUtils::LoadAircraftIntent(std::string parmsfile) {
   DecodedStream intentstream;
   AircraftIntent intent;
   FILE *fp = fopen(parmsfile.c_str(), "r");
   if (fp == nullptr) {
      std::cout << "Intent file " << parmsfile.c_str() << " not found" << std::endl;
   } else {

      bool r = intentstream.open_file(parmsfile);

      if (!r) {
         std::cout << "Can't open intent parameters file " << parmsfile.c_str() << std::endl;
      } else {
         SingleTangentPlaneSequence::ClearStaticMembers();  // make sure singleton is clear

         intentstream.set_echo(false);  // default set to false, must turn it on in input file

         intent.load(&intentstream);
      }
   }

   fclose(fp);
   return intent;
}

AircraftIntent aaesim::test::utils::PublicUtils::PrepareAircraftIntent(std::string parmsfile) {
   AircraftIntent intent = LoadAircraftIntent(parmsfile);
   if (intent.IsLoaded()) {
      intent.UpdateXYZFromLatLonWgs84();
   }

   return intent;
}