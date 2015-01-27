
option(BUILD_ROBOTSYNC_PLUGIN "Building a robotSync plugin \"RobotSyncPlugin\"" OFF)

set(_srcs RobotSyncPlugin.cpp RobotSync.cpp RobotSync.h)

if(NOT BUILD_OPENRTM_PLUGIN)
  return()
else()
  if(NOT ENABLE_CORBA)
    message(FATAL_ERROR "OpenRTM Plugin requires the ENABLE_CORBA option.")
  endif()
  if(NOT BUILD_CORBA_PLUGIN)
    # message(FATAL_ERROR "OpenRTM Plugin requires CorbaPlugin.")
  endif()
endif()

set(OPENRTM_DIR ${OPENRTM_DIR} CACHE PATH "set the top directory of OpenRTM-aist")
if(UNIX)
  if(NOT OPENRTM_DIR)
    pkg_check_modules(OPENRTM REQUIRED openrtm-aist)
  endif()
elseif(MSVC)
  if(NOT OPENRTM_DIR)
    message(FATAL_ERROR "Please specify the directory of OpenRTM-aist to OPENRTM_DIR.")
  endif()
endif()

if(OPENRTM_DIR)

  set(OPENRTM_PREFIX ${OPENRTM_DIR})

  set(OPENRTM_INCLUDE_DIRS
    ${OPENRTM_DIR}/include 
    ${OPENRTM_DIR}/include/coil-1.1
    ${OPENRTM_DIR}/include/openrtm-1.1 
    ${OPENRTM_DIR}/include/openrtm-1.1/rtm/idl
    ${OPENRTM_DIR}/rtm # for the Windows installer version
    ${OPENRTM_DIR}/rtm/idl # for the Windows installer version
    ${OPENRTM_DIR}
    )

  if(MSVC)
    set(OPENRTM_LIBRARY_DIRS ${OPENRTM_DIR}/bin)
  else()
    set(OPENRTM_LIBRARY_DIRS ${OPENRTM_DIR}/lib)
  endif()

  if(UNIX)
    set(OPENRTM_LIBRARIES RTC coil)
  elseif(MSVC)
    file(GLOB librtc RELATIVE ${OPENRTM_LIBRARY_DIRS} "${OPENRTM_LIBRARY_DIRS}/RTC???.lib")
    get_filename_component(librtc ${librtc} NAME_WE)
    file(GLOB libcoil RELATIVE ${OPENRTM_LIBRARY_DIRS} "${OPENRTM_LIBRARY_DIRS}/coil???.lib")
    get_filename_component(libcoil ${libcoil} NAME_WE)
    set(OPENRTM_LIBRARIES 
      optimized ${librtc} debug ${librtc}d
      optimized ${libcoil} debug ${libcoil}d)
    message(STATUS "bin dir = ${OPENRTM_DIR}/bin, lib dir = ${OPENRTM_LIBRARY_DIRS}, rtm libs = ${librtc}, ${libcoil}")
    install_external_libraries(${OPENRTM_DIR}/bin ${OPENRTM_LIBRARY_DIRS} ${librtc} ${libcoil})
  endif()
endif()

# Make the following variables accessible from other directories
set(OPENRTM_INCLUDE_DIRS ${OPENRTM_INCLUDE_DIRS} CACHE INTERNAL "OpenRTM's include directories")
set(OPENRTM_LIBRARY_DIRS ${OPENRTM_LIBRARY_DIRS} CACHE INTERNAL "OpenRTM's library directories")
set(OPENRTM_LIBRARIES ${OPENRTM_LIBRARIES} CACHE INTERNAL "OpenRTM's library files")

#add_definitions(${OPENRTM_CFLAGS})
include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})

if(UNIX)
  set_target_properties(${library} PROPERTIES COMPILE_FLAGS -fPIC)
endif()


if(MSVC)
add_definitions(-D__WIN32__ -D__x86__ -D__NT__ -D__OSVERSION__=4 -D_CRT_SECURE_NO_DEPRECATE -D_WIN32_WINNT=0x0500 -DRTC_CORBA_CXXMAPPING11)
endif()


if(BUILD_ROBOTSYNC_PLUGIN)
  set(target CnoidRobotSyncPlugin)
  add_cnoid_plugin(${target} SHARED ${_srcs})
  target_link_libraries(${target} CnoidBodyPlugin CnoidPoseSeqPlugin CnoidCorbaPlugin CnoidOpenRTM ${OPENRTM_LIBRARIES})
  apply_common_setting_for_plugin(${target})
endif()

# install_sample_source(${_srcs})



