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

#include "loader/LoaderLink.h"
#include "loader/DecodedStream.h"

template <class DATA>
class NativeLoaderLink : public LoaderLink {
  public:
   NativeLoaderLink(DATA *address) { var_address = address; }

   //-------------------------------------------------

   NativeLoaderLink(DATA *address, bool required, bool no_reset) {
      var_address = address;
      must_load = required;
      must_load_only_once = no_reset;
   }

   //-------------------------------------------------

   bool load_s(DecodedStream *ds) { return ds->get_datum(*var_address); }

  private:
   DATA *var_address;
};
