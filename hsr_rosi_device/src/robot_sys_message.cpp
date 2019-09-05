#ifndef FLATHEADERS
#include "robot_sys_message.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"
#else
#include "robot_status_message.h"
#include "robot_status.h"
#include "byte_array.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;
using namespace industrial::byte_array;
using namespace industrial::simple_message;

namespace industrial
{
namespace robot_sys_message
{

RobotSysMessage::RobotSysMessage(void)
{
  this->init();
}

RobotSysMessage::~RobotSysMessage(void)
{

}

bool RobotSysMessage::init(industrial::simple_message::SimpleMessage & msg)
{
    return true;
}

bool RobotSysMessage::init(industrial::simple_message::SimpleMessage & msg,int mssg_type)
{
  bool rtn = true;
  this->init(mssg_type);
  //this->setCommType(industrial::simple_message::CommTypes::TOPIC);
  return rtn;
}

void RobotSysMessage::init()
{
    return;
}

void RobotSysMessage::init(int mssg_type)
{
  this->setMessageType(mssg_type);
}

bool RobotSysMessage::load(ByteArray *buffer)
{
  bool rtn = true;
  LOG_COMM("Executing robot enable message load");
  return rtn;
}

bool RobotSysMessage::unload(ByteArray *buffer)
{
  bool rtn = true;
  LOG_COMM("Executing robot enable message unload");
  return rtn;
}

}
}

