#!/bin/sh
# Substitute script by Keith Lewis
#
# I wanted to make this a sed script, but the escaping was too much.
# This was used to change the #include directives.
# It is preserved here mostly for historical purposes, 
# but it might come in handy during a dirty merge.
#
# usage:  subs.sh file_expresssion
#
# if file_expression contains a wildcard, it should be enclosed 
# in single quotes on the command line to prevent expansion by sh
# into multiple arguments.
#
# example:  ./include/subs.sh '*/*.cpp'
#
sed -i 's|\"aaesim/cppmanifest.h\"|\"public/cppmanifest.h\"|g' $1
sed -i 's|\"aaesim/version.h\"|\"public/version.h\"|g' $1
