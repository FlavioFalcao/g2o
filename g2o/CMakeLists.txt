CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

ADD_SUBDIRECTORY(stuff)
ADD_SUBDIRECTORY(core)

#IF(G2O_BUILD_APPS)
#  ADD_SUBDIRECTORY(apps)
#ENDIF(G2O_BUILD_APPS)

ADD_SUBDIRECTORY(math_groups)

# Pre-canned types
ADD_SUBDIRECTORY(types)

# Solvers
ADD_SUBDIRECTORY(solvers)

# Examples
#IF(G2O_BUILD_EXAMPLES)
#  ADD_SUBDIRECTORY(examples)
#ENDIF(G2O_BUILD_EXAMPLES)
