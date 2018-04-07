cmake_minimum_required(VERSION 3.0)

set(SOURCE_FILES
        units-2.1/scalar/Acceleration.h
        units-2.1/scalar/Angle.h
        units-2.1/scalar/Angle.i
        units-2.1/scalar/AngularAcceleration.h
        units-2.1/scalar/AngularSpeed.h
        units-2.1/scalar/Area.h
        units-2.1/scalar/Current.h
        units-2.1/scalar/Density.h
        units-2.1/scalar/Force.h
        units-2.1/scalar/Format.h
        units-2.1/scalar/Format.i
        units-2.1/scalar/Frequency.h
        units-2.1/scalar/Inertia.h
        units-2.1/scalar/Length.h
        units-2.1/scalar/MagneticField.h
        units-2.1/scalar/MagneticFlux.h
        units-2.1/scalar/MagneticFluxDensity.h
        units-2.1/scalar/Mass.h
        units-2.1/scalar/MassFlowRate.h
        units-2.1/scalar/Power.h
        units-2.1/scalar/Pressure.h
        units-2.1/scalar/ProductUnitFormat.h
        units-2.1/scalar/ProductUnitFormat.i
        units-2.1/scalar/RatioUnitFormat.h
        units-2.1/scalar/RatioUnitFormat.i
        units-2.1/scalar/SignedAngle.h
        units-2.1/scalar/SignedAngle.i
        units-2.1/scalar/SpecificUnit.h
        units-2.1/scalar/SpecificUnit.i
        units-2.1/scalar/Speed.h
        units-2.1/scalar/Temperature.h
        units-2.1/scalar/Temperature.i
        units-2.1/scalar/Time.h
        units-2.1/scalar/Torque.h
        units-2.1/scalar/Unit.h
        units-2.1/scalar/Unit.i
        units-2.1/scalar/UnsignedAngle.h
        units-2.1/scalar/UnsignedAngle.i
        units-2.1/scalar/Voltage.h
        units-2.1/scalar/Volume.h
        units-2.1/system/units_config.h
)

set (LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_library(units-2.1 ${SOURCE_FILES})
SET_TARGET_PROPERTIES(units-2.1 PROPERTIES LINKER_LANGUAGE C)
