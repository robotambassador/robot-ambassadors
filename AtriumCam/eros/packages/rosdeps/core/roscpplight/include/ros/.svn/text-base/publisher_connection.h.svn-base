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

#ifndef _PUBLISHER_CONNECTION_H_INCLUDED_
#define _PUBLISHER_CONNECTION_H_INCLUDED_

#include "ros/log.h"
#include "ros/headerpacket.h"
#include "ros/messagepacket.h"
#include <boost/asio.hpp>
#include <boost/bind.hpp>

namespace ros
{

template <typename M>
class PublisherConnection
{
private:
  boost::asio::io_service * io_service_;
  boost::asio::ip::tcp::acceptor acceptor_;
  boost::asio::ip::tcp::socket socket_;

  enum { max_length = 1024 };
  char data_[max_length];
  enum { NOT_CONNECTED, WAIT_FOR_HEADER, ESTABLISHED, ERROR } state_;
  bool has_data_;
  M message_;
  const std::string caller_id_;

void handleRead(const boost::system::error_code& error, size_t bytes_transferred)
{
  if( bytes_transferred >= 4 )
  {
    Header h;
    h.parseFromBuffer(data_, bytes_transferred );

    if( h["md5sum"] != std::string(ros::message_traits::MD5Sum<M>::value()) )
    {
      ROS_FATAL("md5sum mismatch %s %s", h["md5sum"].c_str(), ros::message_traits::MD5Sum<M>::value() )
    }

    Header reply;
    reply["md5sum"] = ros::message_traits::MD5Sum<M>::value();
    reply["type"] = ros::message_traits::DataType<M>::value();
    reply["message_definition"] = ros::message_traits::Definition<M>::value();
    reply["caller_id"] = caller_id_;
    reply["latching"] = "0";

    size_t datalen = reply.writeToBuffer(data_, max_length);

    if (datalen == 0)
    {
      ROS_WARN("Error while streaming response header");
      state_ = ERROR;
    }
    else
    {
      boost::asio::async_write(socket_, boost::asio::buffer(data_, datalen),
          boost::bind(&PublisherConnection::handleWrite, this,
          boost::asio::placeholders::error));
      state_ = ESTABLISHED;
    }
  }

  if (!error)
  {
  }
  else
  {
  }
}

void handleWrite(const boost::system::error_code& error )
{
  if (!error)
  {
    send();
  }
  else
  {
    state_ = ERROR;
  }

}

void handleAccept(const boost::system::error_code& error)
{
  if (!error)
  {
    ROS_INFO("Remote Subscriber Connected to the Publisher");
    state_ = WAIT_FOR_HEADER;
    socket_.async_read_some(boost::asio::buffer(data_, max_length),
        boost::bind(&PublisherConnection::handleRead, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
  }
  else
  {
    state_ = ERROR;
  }
}

void send()
{
  if (has_data_)
  {
    MessagePacket<M> m;
    m.value() = message_;
    size_t datalen = m.writeToBuffer(data_, max_length);
    boost::asio::async_write(socket_,
                             boost::asio::buffer(data_, datalen),
                             boost::bind(&PublisherConnection::handleWrite, this, boost::asio::placeholders::error)
                             );
    has_data_ = false;
  }
}

public:
  PublisherConnection(boost::asio::io_service * io_service, const std::string& caller_id) :
    io_service_(io_service),
    acceptor_(*io_service),
    socket_(*io_service),
    state_(NOT_CONNECTED),
    has_data_( false ),
    caller_id_(caller_id)
  {
  }

  void open( int port )
  {
    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::tcp::v4(), port);
    boost::system::error_code error;
    if( acceptor_.open(endpoint.protocol(), error) )
    {
      ROS_WARN("PublisherConnection Error Opening Endpoint");
      state_ = ERROR;
      return;
    }
    if(acceptor_.bind(endpoint, error))
    {
      ROS_WARN("PublisherConnection Error Binding to Port %u", port);
      state_ = ERROR;
      return;
    }
    acceptor_.listen();
    acceptor_.async_accept(socket_, boost::bind(&PublisherConnection::handleAccept, this, boost::asio::placeholders::error));
  }

  void publish( const M& message )
  {
    has_data_ = true;
    message_ = message;
    send();
  }

  ~PublisherConnection()
  {
    acceptor_.close();
  }
};

}

#endif
