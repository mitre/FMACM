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
