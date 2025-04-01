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

#include "public/Atmosphere.h"
#include "public/CustomMath.h"
#include <list>
#include <log4cplus/loggingmacros.h>
#include <nlohmann/json.hpp>

log4cplus::Logger Atmosphere::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("Atmosphere"));

void Atmosphere::AirDensity_Log(const Units::MetersLength h, const Units::KelvinTemperature t,
                                const Units::PascalsPressure p, const Units::KilogramsMeterDensity rho) const {

   if (m_logger.getLogLevel() <= log4cplus::TRACE_LOG_LEVEL) {
      nlohmann::json j;
      j["altitude"] = h.value();
      j["temperature"] = t.value();
      j["pressure"] = p.value();
      j["density"] = rho.value();
      LOG4CPLUS_TRACE(m_logger, j);
   }
}
