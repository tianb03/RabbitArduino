#ifndef _ROS_learning_actionlib_AveragingAction_h
#define _ROS_learning_actionlib_AveragingAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "learning_actionlib/AveragingActionGoal.h"
#include "learning_actionlib/AveragingActionResult.h"
#include "learning_actionlib/AveragingActionFeedback.h"

namespace learning_actionlib
{

  class AveragingAction : public ros::Msg
  {
    public:
      learning_actionlib::AveragingActionGoal action_goal;
      learning_actionlib::AveragingActionResult action_result;
      learning_actionlib::AveragingActionFeedback action_feedback;

    AveragingAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "learning_actionlib/AveragingAction"; };
    const char * getMD5(){ return "33cffbf68581da44c6d2a65cc21fb68b"; };

  };

}
#endif