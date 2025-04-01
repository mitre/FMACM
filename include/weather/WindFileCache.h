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

#include <filesystem>
#include <vector>
#include <map>

#include "public/Wind.h"

namespace aaesim::weather {
class WindFileCache {
  public:
   static std::shared_ptr<Wind> LoadWindTruthFile(const std::filesystem::path &truth_file_path,
                                                  const WindDataFormat format, const bool useWind,
                                                  const std::shared_ptr<Atmosphere> atmosphere);

   static std::shared_ptr<Wind> LoadWindFile(const std::filesystem::path &file, const WindDataFormat format,
                                             const std::shared_ptr<Atmosphere> atmosphere);

  private:
   static log4cplus::Logger m_logger;

   /** The number of Wind objects to strongly keep */
   inline static const size_t MAX_RECENT_WINDS{10};
   /** Strong pointers to the most recent Wind objects */
   inline static std::vector<std::shared_ptr<Wind> > m_recent_winds{};
   /** Weak pointers to more Wind objects, mapped by filename */
   inline static std::map<std::string, std::weak_ptr<Wind> > m_past_winds{};
   /** The file corresponding to Wind::m_wind_truth_instance */
   inline static std::filesystem::path m_truth_file_name{};
};
}  // namespace aaesim::weather