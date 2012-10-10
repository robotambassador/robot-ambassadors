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

#ifndef _ROSCPPLIGHT_SUBSCRIBER_CONNECTION_H_INCLUDED
#define _ROSCPPLIGHT_SUBSCRIBER_CONNECTION_H_INCLUDED

#include "ros/messagepacket.h"
#include "ros/headerpacket.h"
#include "ros/log.h"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

namespace ros
{

using boost::asio::ip::tcp;

template<typename M>
class SubscriberConnection
{
public:
  typedef enum { NOT_CONNECTED, WAIT_FOR_HEADER, ESTABLISHED, ERROR } stateType;

private:
  boost::asio::io_service * io_service_;
  boost::asio::ip::tcp::socket socket_;
  enum { max_length = 1024 };
  char data_[max_length];
  boost::function<void (M)> callback_;
  stateType state_;

private:
  void sendHeader(const Header& h)
  {
    unsigned int length = h.writeToBuffer(data_, max_length);

    boost::asio::async_write(socket_, boost::asio::buffer(data_, length),
        boost::bind(&SubscriberConnection::handleWrite, this,
            boost::asio::placeholders::error));
  }

  void parseData( const char* data, size_t length )
  {
    switch( state_ )
    {
      case WAIT_FOR_HEADER:
      {
        Header reply;
        size_t size = reply.parseFromBuffer( data, length );
        if( size == 0 )
        {
          state_ = ERROR;
          return;
        }
        ROS_INFO("TCPROS Header Received");
        state_ = ESTABLISHED;

        // Check if we have more data in the buffer
        if( length > size )
        {
          parseData( &data[size], length - size );
        }
        break;
      }
      case ESTABLISHED:
      {
        MessagePacket<M> message;
        size_t size = message.parseFromBuffer( data, length );
        if( size == 0 )
        {
          state_ = ERROR;
          return;
        }
        callback_(message.value());

        // Check if we have more data in the buffer
        if( length > size )
        {
          parseData( data + size, length - size );
        }
        break;
      }
      default:
        break;
    }
  }

  void acceptRead()
  {
    socket_.async_read_some(boost::asio::buffer(data_, max_length), boost::bind(
        &SubscriberConnection::handleRead, this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
  }

  void handleRead(const boost::system::error_code& error, size_t bytes_transferred)
  {
    if (!error)
    {
      parseData( data_, bytes_transferred );
      acceptRead();
    }
    else
    {
      ROS_ERROR("Subscriber Connection: Read error" );
      state_ = ERROR;
    }
  }

  void handleWrite(const boost::system::error_code& error )
  {
    if (!error)
    {
      acceptRead();
    }
    else
    {
      ROS_ERROR("Subscriber Connection: Error sending header" );
      state_ = ERROR;
    }
  }

public:
  SubscriberConnection(boost::asio::io_service* io_service, boost::function<void (M)> callback) :
    io_service_(io_service),
    socket_(*io_service),
    callback_(callback),
    state_( NOT_CONNECTED )
  {
  }

  void connect(const std::string& host, const int port,
      const std::string& topic, const std::string& caller_id)
  {
    ROS_ASSERT( state_ == NOT_CONNECTED );
    ROS_INFO("Connecting to publisher of %s on %s:%i", topic.c_str(), host.c_str(), port);

    char port_string[10];
    sprintf(port_string, "%i", port);

    //todo: async resolver?
    tcp::resolver resolver(*io_service_);

    tcp::resolver::query query(host, port_string);
    tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
    tcp::resolver::iterator end;

    //todo: async connect
    boost::system::error_code error = boost::asio::error::host_not_found;
    while (error && endpoint_iterator != end)
    {
      socket_.close();
      socket_.connect(*endpoint_iterator++, error);
    }
    if (error)
    {
      ROS_ERROR("Could not connect to publisher %s %u", host.c_str(), port);
      state_ = ERROR;
    }
    else
    {
      state_ = WAIT_FOR_HEADER;

      Header request;
      request["md5sum"] = ros::message_traits::MD5Sum<M>::value();
      request["type"] = ros::message_traits::DataType<M>::value();
      request["callerid"] = caller_id;
      request["topic"] = topic;
      request["tcp_nodelay"] = "0";
      request["message_definition"] = ros::message_traits::Definition<M>::value();

      sendHeader(request);
    }
  }

  stateType getState()
  {
    return state_;
  }

};

}

#endif
