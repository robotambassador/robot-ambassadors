/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Prevas A/S
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Prevas A/S nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Morten Kjaergaard
 */

#ifndef _ROSCPPLIGHT_MESSAGE_PACKET_H_INCLUDED_
#define _ROSCPPLIGHT_MESSAGE_PACKET_H_INCLUDED_

#include "ros/log.h"
#include "ros/rospacket.h"
#include <ros/serialization.h>

namespace ros
{

template<typename M>
class MessagePacket : public RosPacket
{
private:
  M value_;
public:
  MessagePacket() : value_()
  {
  }

  size_t writeToBuffer( char * data, size_t length )
  {
    // Serialize message
    ros::serialization::OStream s( (uint8_t*)(data+4), length-4 );
    size_t l1 = s.getLength();
    s << value_;
    size_t datalen = l1 - s.getLength();

    // Write packet length
    writeInt( data, datalen );
    return datalen + 4;
  }

  size_t parseFromBuffer( const char * data, size_t data_length )
  {
    // Verify packet
    if( data_length < 8 )
    {
      ROS_ERROR("Invalid Message length %u", data_length);
      return 0;
    }

    // Read packet length
    size_t msg_len = parseInt( data );
    data += 4;

    // Read message length
    size_t field_len = parseInt( data );
    if( msg_len != field_len + 4 )
    {
      ROS_ERROR("Message packet length  and field length mismatch %u %u", msg_len, field_len);
      return 0;
    }
    ros::serialization::IStream s( (uint8_t*)(data), msg_len );
    s >> value_;

    return msg_len + 4;
  }

  M& value()
  {
    return value_;
  }

};

}

#endif
