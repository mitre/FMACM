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
#include "MiniCSV/minicsv.h"

struct OutputHandler {

  public:
   OutputHandler() = default;

   /*
    * The constructor takes the scenario name, and the desired file extension to describe the
    * type of output contained in this file.
    */
   OutputHandler(const std::string &scenario_name, const std::string &file_suffix)
      : m_file_suffix(file_suffix), filename(scenario_name + file_suffix), os(), m_finished(false) {}

   virtual ~OutputHandler() = default;

   /**
    * Writes the file, closes it, and clears data stores to save memory.
    * Must also set m_finished to true.
    * Not implemented at the OutputHandler level, must be done in a subclass.
    * Finish() should only be called once during the life cycle of the object,
    * to avoid overwriting the file.
    */
   virtual void Finish() = 0;

   /**
    * Implementations which don't open the output file immediately can
    * use a dummy scenario name in the constructor and set it later
    * using this function.
    */
   virtual void SetScenarioName(const std::string &scenario_name);

   virtual std::string GetOutputFilename() const { return filename; }

   std::string GetFileSuffix() const { return m_file_suffix; }

  protected:
   // Everything in the filename after the scenario name, e.g. "-waypoints.csv"
   std::string m_file_suffix;

   // Full name of file to be written, including suffix
   std::string filename;

   // Output stream that handles writing when object is destroyed
   mini::csv::ofstream os;

   // indicates whether Finish() has been called, to complete writing
   bool m_finished;
};

inline void OutputHandler::SetScenarioName(const std::string &scenario_name) {
   filename.assign(scenario_name + m_file_suffix);
}