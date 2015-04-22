#
# $Id: Makefile,v 1.7.2.3 2007-10-01 05:44:30 knicewar Exp $
#
# Copyright Keith Nicewarner.  All rights reserved.
#
# Build the scalar units library.
#

SUBDIRS = test


INSTALL_HTML_DIR = scalar
INSTALL_HTML     = scalar.html unitList.html \
		   Time.html \
		   Frequency.html \
		   Mass.html \
		   MassFlowRate.html \
		   Length.html \
		   Area.html \
		   Volume.html \
		   Speed.html \
		   Acceleration.html \
		   Force.html \
		   Power.html \
		   Angle.html \
		   SignedAngle.html \
		   UnsignedAngle.html \
		   AngularSpeed.html \
		   Temperature.html \
		   Pressure.html \
		   Density.html 

INSTALL_HEADER_DIR = scalar
INSTALL_HEADERS    = Unit.h Unit.i \
		     ProductUnitFormat.h ProductUnitFormat.i \
		     RatioUnitFormat.h RatioUnitFormat.i \
		     SpecificUnit.h SpecificUnit.i \
		     Time.h \
		     Frequency.h \
		     Mass.h \
		     MassFlowRate.h \
		     Length.h \
		     Area.h \
		     Volume.h \
		     Speed.h \
		     Acceleration.h \
		     Force.h \
		     Inertia.h \
		     Angle.h \
		     SignedAngle.h SignedAngle.i \
		     UnsignedAngle.h UnsignedAngle.i \
		     AngularSpeed.h \
		     AngularAcceleration.h \
		     Torque.h \
		     Voltage.h \
		     Current.h \
		     Power.h \
		     MagneticField.h \
		     MagneticFluxDensity.h \
		     MagneticFlux.h \
		     Pressure.h \
		     Density.h \
		     Temperature.h Temperature.i


#
# Must define TARGETS, SOURCE, LIB_OBJS before including this.
#
UNITS_ROOT = ..
include ../make_rules/common.rules
