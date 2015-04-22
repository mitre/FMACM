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
// Copyright 2015 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include <Angle.h>


class AngleOperations {
public:

	/**
	 * A utility method that ensures that the incoming angle object
	 * is wrapped on the interval [0,360] degrees.
	 * @see test code in aaesim_tester.cpp
	 */
	static Units::Angle createWrappedAngle(Units::Angle angle) {
		// check for a negative value
	        while( angle < Units::DegreesAngle(0.0))
		{
		        angle += Units::DegreesAngle(360.0); // make a positive value
		}

		// Check for positive value
		while( angle > Units::DegreesAngle(360.0))
		{
		        angle -= Units::DegreesAngle(360.0); // subtract
		}

		// return
		return angle;
	}

	/**
	 * Perform a circular difference operation on the interval (0,360]. All operations will return the smaller (interior)
	 * angle, including those that wrap across the 0/360 boundary. The returned
	 * Angle is not signed. Mathematically, the difference
	 * operation is always (a1-a2).
	 *
	 * @code
	 * 	Units::Angle result = performWrappedSubtraction(Units::DegreesAngle(1.0),Units::DegreesAngle(359.0));
	 *  // result will contain 2 degrees
	 *  @endcode
	 *
	 * @see test code in aaesim_tester.cpp
	 */
	static Units::Angle performWrappedSubtraction(Units::Angle a1, Units::Angle a2) {
		Units::Angle result = createWrappedAngle(a1)-createWrappedAngle(a2);

		// Get interior angle
		const Units::Angle a180 = Units::DegreesAngle(180.0);
		const Units::Angle a360 = Units::DegreesAngle(360.0);
		if (result > a180) {
			result -= a360;
		} else if (result < a180*-1) {
			result += a360;
		}

		// Mod the resulting angle with 360
		return Units::DegreesAngle(fabs(fmod(Units::DegreesAngle(result).value(),360.0)));
	}
};
