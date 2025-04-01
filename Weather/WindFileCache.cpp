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

#include "weather/WindFileCache.h"
#include "weather/WindCaasd.h"
#include "weather/WindLegacy.h"
#include "public/WindZero.h"

using namespace std;
using namespace aaesim::weather;

log4cplus::Logger WindFileCache::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("WindFileCache"));

std::shared_ptr<Wind> WindFileCache::LoadWindTruthFile(const std::filesystem::path &truth_file_path,
                                                       const WindDataFormat format, const bool useWind,
                                                       const std::shared_ptr<Atmosphere> atmosphere) {
   std::shared_ptr<Wind> w;  // threadsafe local variable
   Wind::SetUseWind(useWind);
   if (!useWind) {
      // ignore the rest of the parameters
      m_truth_file_name = "";
      w = make_unique<aaesim::open_source::WindZero>(atmosphere);
      Wind::SetWindTruthInstance(w);
   } else if (truth_file_path != m_truth_file_name) {
      // load truth file
      m_truth_file_name = truth_file_path;
      w = LoadWindFile(truth_file_path, format, atmosphere);
      Wind::SetWindTruthInstance(w);
   }

   return Wind::GetWindTruthInstance();
}

shared_ptr<Wind> WindFileCache::LoadWindFile(const std::filesystem::path &file, const WindDataFormat format,
                                             const std::shared_ptr<Atmosphere> atmosphere) {

   shared_ptr<Wind> instance;

   // If use_wind is false, return a WindZero object
   if (!Wind::UseWind()) {
      instance = shared_ptr<Wind>(new aaesim::open_source::WindZero(atmosphere));
      return instance;
   }

   // check the cache
   weak_ptr<Wind> cached = m_past_winds[file];
   instance = cached.lock();

   if (instance) {
      LOG4CPLUS_TRACE(m_logger, "Wind cache hit for " << file);
   } else {
      LOG4CPLUS_TRACE(m_logger, "Attempting to load wind file " << file);
      // failed to get a cached copy
      string ext = file.extension();
      if (ext != ".txt" && ext != ".csv") {
         // try the Wind API
         try {
            instance = shared_ptr<Wind>(new WindCaasd(file));
         } catch (string s) {
            instance = shared_ptr<Wind>((Wind *)NULL);
         }
      }
   }

   if (!instance) {
      // failed to read file in API, fall back to legacy text formats
      if (format == RAP_FORMAT) {
         WindLegacy *wptr = new WindLegacy();
         wptr->readRAPWindFile(file);
         instance = shared_ptr<Wind>(wptr);
      } else if (format == RUC_FORMAT) {
         WindLegacy *wptr = new WindLegacy();
         wptr->readRUCWindFile(file);
         instance = shared_ptr<Wind>(wptr);
      }
   }

   // save it in the cache
   cached = instance;
   m_past_winds[file] = cached;
   for (auto it = m_recent_winds.begin(); it != m_recent_winds.end();) {
      if (*it == instance) {
         m_recent_winds.erase(it);
         break;  // there can be a max of one, and we just deleted it.
      } else {
         it++;
      }
   }
   m_recent_winds.push_back(instance);
   if (m_recent_winds.size() > MAX_RECENT_WINDS) {
      m_recent_winds.erase(m_recent_winds.begin());
   }

   return instance;
}
