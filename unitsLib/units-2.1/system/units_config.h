//
// $Id: units_config.h,v 1.1.2.2 2008-06-17 19:55:45 knicewar Exp $
//
// Copyright SRI.  All rights reserved.
//
// This file contains everything needed to make the code compile
// uniformly on all platforms.
//

#ifndef Units_config_h
#define Units_config_h

#if defined(_MSC_VER)
#  pragma warning(disable:4661)
#endif

#if defined(_MSC_VER) && (_MSC_VER >= 1300)
#  pragma warning(disable:4290)
#endif

#define UNITS_TEMPLATES_REQUIRE_SOURCE


#ifndef UNITS_DEFAULT_VALUE_TYPE
#define UNITS_DEFAULT_VALUE_TYPE double
#endif


#endif  // Units_config_h
