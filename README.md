#NOTICE#
This is the copyright work of The MITRE Corporation, and was produced
for the U. S. Government under Contract Number DTFAWA-10-C-00080, and
is subject to Federal Aviation Administration Acquisition Management
System Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV
(Oct. 1996).  No other use other than that granted to the U. S.
Government, or to those acting on behalf of the U. S. Government,
under that Clause is authorized without the express written
permission of The MITRE Corporation. For further information, please
contact The MITRE Corporation, Contracts Office, 7515 Colshire Drive,
McLean, VA  22102-7539, (703) 983-6000. 

Copyright 2015 The MITRE Corporation. All Rights Reserved.
Approved for Public Release; Distribution Unlimited. 15-1482

This project contains content developed by The MITRE Corporation. If this code is used in a deployment or embedded within another project, it is requested that you send an email to opensource@mitre.org in order to let us know where this software is being used. 

#Licensing#
[Apache 2.0](https://github.com/mitre/FMACM/blob/master/LICENSE)

#Documentation#
Please refer to applicable [RTCA SC-186](http://www.rtca.org/content.asp?pl=108&sl=33&contentid=88) documentation.

#Licensing questions#
Any questions related to MITRE Open Source technologies may be emailed to opensource@mitre.org

#Developer Notice#

###EUROCONTROL BADA Development Necessary###
This code uses [EUROCONTROL's BADA](https://eurocontrol.int/services/bada) for aircraft performance data that drive the aircraft dynamics modeling. However, BADA functionality and code cannot be provided due to licensing restrictions imposed by EUROCONTROL. Therefore, stub classes exist in this code repository that represent MITRE's suggested implementation in order to use this software. See /CoreII/bada.cpp and /CoreII/BadaWithCalc.cpp. Please complete the implementation in these classes before using the code.
 
###Compile###
No attempt has been made to ensure that this code will compile on all operating systems. This code compiles successfully on Linux machines, specifically Red Hat Enterprise versions 5 and 6. For all other computing environments, YMMV.

The CMake utility is used to compile this code. Please use version 2.6 or greater. From the root directory, execute:

> cmake . 
<br>
> make

The resulting executable will be found in <root>/CoreII/ and is named FMACM

###Run###
A configuration file must be provided as the only command-line argument to the FMACM program. The file must be formatted as plain text and contain paths to each scenario that is to be run. The contents of the configuration file must look like this:

> 1 # number of scenarios to run
> 
> path/to/scenario.txt # path to each scenario file, one per line, as many as the number above indicates

Users may list as many scenario files as desired. The scenario file provides detailed instructions about the scenario that FMACM is to run. An example scenario file is provided in <root>/Run_Files/test-framework-scenario.txt.

The executable is then executed in this manner:
> ./CoreII/FMACM configuration.txt 
