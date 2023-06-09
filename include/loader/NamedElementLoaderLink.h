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

template <class DATA>
class NamedElementLoaderLink : public LoaderLink {
  public:
   NamedElementLoaderLink(std::vector<DATA> *address) {
      var_address = address;
      must_load_only_once = false;
      is_a_list = true;
   }

   //-------------------------------------------------

   bool load_s(DecodedStream *ds) {
      DATA temp;
      std::string token;
      bool f;

      // read off the {

      f = ds->get_datum(token);

      if (!f) {
         std::string msg = "load attempt failed:  no first token";
         ds->highlight_on();
         ds->report_error("\n" + msg + "\n");
         ds->highlight_off();
         throw std::runtime_error(msg);
      }

      if (token.compare("{") != 0) {
         std::string msg = "load attempt failed:  first token not {";
         ds->highlight_on();
         ds->report_error("\n" + msg + "\n");
         ds->highlight_off();
         throw std::runtime_error(msg);
      }

      // load the entry ----------------------------------------------

      f = temp.load(ds);

      if (!f) {
         std::string msg = "load attempt failed.";
         ds->highlight_on();
         ds->report_error("\n" + msg + "\n");
         ds->highlight_off();
         throw std::runtime_error(msg);
      }

      // add it to the list ----------------------------------------------

      var_address->push_back(temp);

      // get the end of the entry ----------------------------------------------

      f = ds->get_datum(token);

      if (!f) {
         std::string msg = "load attempt failed:  no last token";
         ds->highlight_on();
         ds->report_error("\n" + msg + "\n");
         ds->highlight_off();
         throw std::runtime_error(msg);
      }

      // look for the } at the end of the list ----------------------------------------------

      if (token.compare("}") == 0) {
         return true;
      } else {
         std::string msg = "load attempt failed:  last token not }";
         ds->highlight_on();
         ds->report_error("\n" + msg + "\n");
         ds->highlight_off();
         throw std::runtime_error(msg);
      }

      return true;
   }

  private:
   std::vector<DATA> *var_address;
};
