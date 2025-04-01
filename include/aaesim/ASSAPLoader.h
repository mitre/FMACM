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

#include "public/LoggingLoadable.h"
#include "public/AircraftState.h"
#include "public/ASSAP.h"

namespace aaesim {
namespace loaders {

class ASSAPLoader final : public LoggingLoadable {
  public:
   ASSAPLoader();

   virtual ~ASSAPLoader() = default;

   static std::shared_ptr<aaesim::open_source::ASSAP> ConstructAssap(std::string algorithm_name,
                                                                     Units::Time maximum_coast_time);

   std::shared_ptr<aaesim::open_source::ASSAP> BuildLoadedAssap();

   bool load(DecodedStream *input);

   /**
    * Returns a new AircraftState that is time-synchronized
    * based on the ADSBSVReport object passed in. The algorithm used
    * to update the state is selected by the user in the input file.
    *
    * @param state_to_sync_with
    * @param most_recent_ads_b
    * @return
    */
   aaesim::open_source::AircraftState Update(const aaesim::open_source::AircraftState &state_to_sync_with,
                                             const aaesim::open_source::ADSBSVReport &most_recent_ads_b);

   const bool IsLoaded() const;

  private:
   enum AssapType { NONE = 0, LEGACY = 1, EXTRAPOLATE = 2 };
   static std::map<std::string, AssapType> ASSAP_TYPES;

   std::string m_assap_algorithm;
   double m_max_coast_time;
   bool m_is_loaded;
};

inline const bool ASSAPLoader::IsLoaded() const { return m_is_loaded; }

}  // namespace loaders
}  // namespace aaesim
