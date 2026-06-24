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

#include <loader/LoggingLoadable.h>
#include <log4cplus/logger.h>
#include <log4cplus/loggingmacros.h>

#include "public/AircraftIntent.h"

namespace aaesim {
namespace loaders {

class AircraftIntentLoader final : public LoggingLoadable {
  public:
   AircraftIntentLoader() = default;
   ~AircraftIntentLoader() override = default;

   bool load(DecodedStream *input) override;

   const AircraftIntent &BuildAircraftIntent() const;
   AircraftIntent &GetAircraftIntent();
   const AircraftIntent &GetAircraftIntent() const;
   bool IsLoaded() const;

  private:
   static inline log4cplus::Logger logger_{log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("AircraftIntentLoader"))};

   AircraftIntent aircraft_intent_{};
};

inline const AircraftIntent &AircraftIntentLoader::BuildAircraftIntent() const { return aircraft_intent_; }

inline AircraftIntent &AircraftIntentLoader::GetAircraftIntent() { return aircraft_intent_; }

inline const AircraftIntent &AircraftIntentLoader::GetAircraftIntent() const { return aircraft_intent_; }

inline bool AircraftIntentLoader::IsLoaded() const { return aircraft_intent_.IsLoaded(); }

}  // namespace loaders
}  // namespace aaesim
