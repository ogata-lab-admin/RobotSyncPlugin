// -*- C++ -*-
/*!
 * @file  RobotSync.cpp
 * @brief Robot Sync RTC
 * @date $Date$
 *
 * $Id$
 */

#include "RobotSync.h"

// Module specification
// <rtc-template block="module_spec">
static const char* robotsync_spec[] =
  {
    "implementation_id", "RobotSync",
    "type_name",         "RobotSync",
    "description",       "Robot Sync RTC",
    "version",           "1.0.0",
    "vendor",            "Sugar Sweet Robotics",
    "category",          "Experimenta",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debug", "0",
    "conf.default.default_mask", "1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0",
    "conf.default.default_angle_radian", "2.234,2.234,2.234,2.234,0,2.234,2.234,2.234,0,0,0,1.58,0,0,0,0,0,0,1.58,0,0,0,0,2.234,0",
    // Widget
    "conf.__widget__.debug", "text",
    "conf.__widget__.default_mask", "text",
    "conf.__widget__.default_angle_radian", "text",
    // Constraints
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
RobotSync::RobotSync(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_inIn("in", m_in),
    m_outOut("out", m_out)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
RobotSync::~RobotSync()
{
}



RTC::ReturnCode_t RobotSync::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("in", m_inIn);
  
  // Set OutPort buffer
  addOutPort("out", m_outOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug", m_debug, "0");
  bindParameter("default_mask", m_default_mask_str, "1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0");
  bindParameter("default_angle_radian", m_default_angle_radian_str, "2.234,2.234,2.234,2.234,0,2.234,2.234,2.234,0,0,0,1.58,0,0,0,0,0,0,1.58,0,0,0,0,2.234,0");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RobotSync::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotSync::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotSync::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t RobotSync::onActivated(RTC::UniqueId ec_id)
{
  m_default_mask.clear();
  std::stringstream nss(m_default_mask_str);
  std::string token;
  int buf;
  while(std::getline(nss, token, ',')) {
    std::stringstream trimmer;
    trimmer << token;
    token.clear();
    trimmer >> buf;
    m_default_mask.push_back(buf);
  }

  m_default_angle_radian.clear();
  std::stringstream nss2(m_default_angle_radian_str);
  double angle;
  while(std::getline(nss2, token, ',')) {
    std::stringstream trimmer;
    trimmer << token;
    token.clear();
    trimmer >> angle;
    m_default_angle_radian.push_back(angle);
  }

  return RTC::RTC_OK;
}


RTC::ReturnCode_t RobotSync::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t RobotSync::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RobotSync::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotSync::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotSync::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotSync::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotSync::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



void RobotSync::registerFactory(RTC::Manager* manager, const char* componentTypeName)
{
  static const char* spec[] = {
    "implementation_id", "RobotSync",
    "type_name",         "RobotSync",
    "description",       "This component enables controller components to"
    "access the I/O of a virtual robot in a Choreonoid simulation",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "Choreonoid",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "100",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };


  RTC::Properties profile(spec);
  profile.setDefault("type_name", componentTypeName);

  manager->registerFactory(profile,
			   RTC::Create<RobotSync>,
			   RTC::Delete<RobotSync>);
}



extern "C"
{
 
  void RobotSyncInit(RTC::Manager* manager)
  {
    coil::Properties profile(robotsync_spec);
    manager->registerFactory(profile,
                             RTC::Create<RobotSync>,
                             RTC::Delete<RobotSync>);
  }
  
};


