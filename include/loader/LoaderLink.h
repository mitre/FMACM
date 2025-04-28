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

#include "loader/DecodedStream.h"

/**
 * Meta info for the concept of deprecated in the Loadable system.
 */
struct LoaderDeprecatedMetaInfo {
   bool isDeprecated;
   std::string supersededByTagName;
   //	std::string deprecatedInVersion; // keep this commented out until a final concept of versioning is implemented in
   // AAESim
};

class LoaderLink {
  public:
   LoaderLink(void);

   virtual ~LoaderLink() = 0;

   bool load(DecodedStream *ds);

   virtual bool load_s(DecodedStream *ds) = 0;

   //-----------------------------------------------------------

   bool get_loaded_status() { return loaded; }

   //-----------------------------------------------------------

   bool is_a_must_load() { return must_load; }

   //-----------------------------------------------------------

   void set_must_load(bool b) { must_load = b; }

   //-----------------------------------------------------------

   bool was_loaded() { return loaded; }

   //-----------------------------------------------------------

   bool ok() {
      if (!must_load) {
         return true;
      } else if (loaded) {
         return true;
      } else {
         return false;
      }
   }

   void set_deprecated_info(const LoaderDeprecatedMetaInfo &info) { deprecatedInfo = info; }

   LoaderDeprecatedMetaInfo get_deprecated_info() { return deprecatedInfo; }

  protected:
   bool loaded;
   bool must_load;            // if true you must load one or more times
   bool must_load_only_once;  // if true you can only load it once or 0 times
   bool is_a_list;            // if set high the two above do not apply
   LoaderDeprecatedMetaInfo deprecatedInfo;
};
