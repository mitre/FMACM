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
// Copyright 2017 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#if ! defined (AAESIM_VERSION_H)
#define AAESIM_VERSION_H

#if defined (AAESIM_HAVE_PRAGMA_ONCE)
#pragma once
#endif

#include <iostream>
#include <string>

// if suffix is empty, this will end with a hyphen. That will be removed at run-time
#define AAESIM_MAKE_VERSION_STR(major, minor, point, suffix) \
#major "." #minor "." #point "-" #suffix

//! This is AAESIM version number as a string.
//! Do not wrap the suffix in quotes, but it may be left empty for a release
//! Do not leave a space before the right parenthesis
#define AAESIM_VERSION_STR AAESIM_MAKE_VERSION_STR(1, 4, 4, SNAPSHOT)

namespace aaesim {
    static std::string getVersion() {
        std::string verStr(AAESIM_VERSION_STR);
        bool stripLastChar = verStr.find("-") == verStr.length()-1;
        if (stripLastChar) {
            // dump the last char
            verStr.resize(verStr.find("-"));
        }
        return verStr;
    }
}
#endif


#if ! defined (CPPMANIFEST_VERSION_H)
#define CPPMANIFEST_VERSION_H

#if defined (CPPMANIFEST_HAVE_PRAGMA_ONCE)
#pragma once
#endif

#include <iostream>
#include <string>

// if suffix is empty, this will end with a hyphen. That will be removed at run-time
#define CPPMANIFEST_MAKE_VERSION_STR(major, minor, point, suffix) \
#major "." #minor "." #point "-" #suffix

//! This is CPPMANIFEST version number as a string.
//! Do not wrap the suffix in quotes, but it may be left empty for a release
#define CPPMANIFEST_VERSION_STR CPPMANIFEST_MAKE_VERSION_STR(1, 0, 0, alpha)

#endif
