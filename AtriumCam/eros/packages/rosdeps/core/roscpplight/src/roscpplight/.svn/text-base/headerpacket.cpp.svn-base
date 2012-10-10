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

#include "ros/headerpacket.h"
#include "ros/log.h"
#include <boost/regex.hpp>

namespace ros
{

bool Header::parseField( const std::string& field )
{
  boost::regex expr("([^=]+)=([^=]+)");
  boost::smatch what;
  if (boost::regex_search( field, what, expr ))
  {
    std::string fieldname = what[1].str();
    std::string fieldvalue = what[2].str();
    (*this)[fieldname] = fieldvalue;
    return true;
  }
  else
  {
    ROS_ERROR( "Invalid field in TCPROS header: %s", field.c_str() );
    return false;
  }
}

size_t Header::parseFromBuffer( const char* data, unsigned int data_length )
{
  // Clear the map
  clear();

  // Verify packet
  if( data_length < 8 )
  {
    ROS_ERROR("Invalid Header length %u", data_length);
    return 0;
  }

  // Read packet length
  unsigned int msg_len = parseInt( data );
  data += 4;

  // Verify that we have complete packet
  if( msg_len > data_length - 4 )
  {
    ROS_ERROR("Incomplete Header: Exp %u Has %u", msg_len, data_length - 4);
    return 0;
  }

  // Parse all fields
  unsigned int data_left = msg_len;
  while( data_left != 0 )
  {
    // Read field length
    unsigned int field_len = parseInt( data );
    data += 4;
    data_left -= 4;

    // Verify that we have a complete field
    if( data_left < field_len )
    {
      ROS_ERROR("Incomplete Header Field: Exp %u Has %u", field_len, data_left)
      return 0;
    }

    // Read Field String
    std::string field("");
    for( unsigned int i = 0; i < field_len; i++ )
    {
      field.push_back( data[i] );
    }

    // Parse Field String
    parseField( field );
    data += field_len;
    data_left -= field_len;
  }
  return msg_len + 4;
}

unsigned int Header::writeToBuffer( char* data, unsigned int buffer_length ) const
{
  // todo: check that buffer_len is large enough :o)
  unsigned messagelen = 0;

  // Make room for message length
  char* pos = data + 4;

  // Copy in all the fields
  for( type::const_iterator i = begin(); i != end(); i++ )
  {
    const std::string& key = (*i).first;
    const std::string& value = (*i).second;
    unsigned field_len = key.size() + value.size() + 1;

    // Make sure the buffer is large enough
    ROS_ASSERT( buffer_length >= messagelen + 4 + field_len + 4 );

    // Write field length
    writeInt( pos, field_len );
    pos += 4;

    // Copy in field
    memcpy( pos, key.c_str(), key.size() );
    pos += key.size();
    *pos = '=';
    pos++;
    memcpy( pos, value.c_str(), value.size() );
    pos += value.size();

    // Increase message length
    messagelen += field_len + 4;
  }

  // Write message length in the beginning
  writeInt( data, messagelen );
  messagelen += 4;

  return messagelen;
}

}
