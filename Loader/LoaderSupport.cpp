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
// Copyright 2018 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "loader/LoaderSupport.h"

using namespace std;

LoaderSupport::LoaderSupport(void) {
}

LoaderSupport::~LoaderSupport(void) {
}

string LoaderSupport::clean_token(const string &token) {
   string out = token;

   for (unsigned int i = 0; i < token.size(); i++) //turning string to lower case
   {
      out[i] = tolower(token[i]);
   }

   int pos = out.find(":");
   int size = out.size();

   if (pos == size - 1) //cut the : off the end of a token
   {
      out.resize(size - 1);
   }

   return out;
}
