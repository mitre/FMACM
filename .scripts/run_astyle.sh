#!/bin/sh

# Playing with Astyle inputs to get what I want for us
/usr/depot/astyle-3.1/astyle --style=google --indent=spaces=3 --convert-tabs --indent-cases --indent-switches Core/*.cpp,*.h

