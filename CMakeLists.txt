cmake_minimum_required(VERSION 3.7.2)

##################################################
set (VERSION_NUMBER "0.1.0")
project("SolARModulePCL")
set (SOURCES 
	src/SolARModuleRealSense.cpp
	src/SolARRGBDCameraRealSense.cpp
)

set (HEADERS
	interfaces/SolARRealSenseAPI.h
    interfaces/SolARModuleRealSense_traits.h
	interfaces/SolARRGBDCameraRealSense.h
)

#find_package(RealSense REQUIRED)

include_directories(
	src/
)

install (FILES "${CMAKE_CURRENT_SOURCE_DIR}/xpcf_SolARModuleRealSense_registry.xml" DESTINATION $ENV{BCOMDEVROOT}/.xpcf/SolAR/	)

##################################################

# various macros
include("$ENV{BCOMDEVROOT}/bcomBuild/SolARFramework/solarmacros.cmake")
# config setup
setup()
# process packagedependencies.txt
processPackagedependencies()
# define targets (type library)
defineTargets("library" "")


