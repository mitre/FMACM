// ****************************************************************************
// NOTICE
//
// This is the copyright work of The MITRE Corporation, and was produced
// for the U. S. Government under Contract Number DTFAWA-10-C-00080, and
// is subject to Federal Aviation Administration Acquisition Management
// System Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV
// (Oct. 1996).  No other use other than that granted to the U. S.
// Government, or to those acting on behalf of the U. S. Government,
// under that Clause is authorized without the express written
// permission of The MITRE Corporation. For further information, please
// contact The MITRE Corporation, Contracts Office, 7515 Colshire Drive,
// McLean, VA  22102-7539, (703) 983-6000. 
//
// Copyright 2020 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include "loader/LoaderLink.h"

template<class DATA>
class NamedElementLoaderLink : public LoaderLink
{
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
