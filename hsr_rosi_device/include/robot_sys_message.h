#ifndef ROBOT_ENABLE_MESSAGE_H
#define ROBOT_ENABLE_MESSAGE_H
#ifndef FLATHEADERS
//#include "typed_message.h"
#include "simple_message/typed_message.h"
#include "simple_message/simple_message.h"
#include "simple_message/shared_types.h"
//#include "robot_enable.h"
#else
#include "typed_message.h"
#include "simple_message.h"
#include "shared_types.h"
#include "robot_status.h"
#endif

namespace industrial
{
namespace robot_sys_message
{

/**
 * \brief Class encapsulated robot status message generation methods
 * (either to or from a industrial::simple_message::SimpleMessage type.
 *
 * This message simply wraps the industrial::robot_status::RobotStatus data type.
 * The data portion of this typed message matches RobotStatus.
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class RobotSysMessage : public industrial::typed_message::TypedMessage
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates an empty message.
   *
   */
  RobotSysMessage(void);
  /**
   * \brief Destructor
   *
   */
  ~RobotSysMessage(void);
  /**
   * \brief Initializes message from a simple message
   *
   * \param simple message to construct from
   *
   * \return true if message successfully initialized, otherwise false
   */
  bool init(industrial::simple_message::SimpleMessage & msg,int mssg_type);

  bool init(industrial::simple_message::SimpleMessage & msg);

  /**
   * \brief Initializes message from a robot status structure
   *
   * \param status strcutre to initialize from
   *
   */
//  void init(industrial::robot_enable::RobotEnable & enable,int mssg_type);

//  void init(industrial::robot_enable::RobotEnable & enable);

  /**
   * \brief Initializes a new robot status message
   *
   */
  void init();

  void init(int mssg_type);


  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);

  unsigned int byteLength()
  {
    return 1;
  }


  //industrial::robot_enable::RobotEnable enable_;

};

}
}
#endif // ROBOT_ENABLE_MESSAGE_H
