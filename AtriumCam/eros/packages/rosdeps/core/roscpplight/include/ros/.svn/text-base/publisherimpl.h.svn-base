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

#ifndef _ROSCPPLIGHT_PUBLISHERIMPL_H_INCLUDED_
#define _ROSCPPLIGHT_PUBLISHERIMPL_H_INCLUDED_

#include "ros/publisher_connection.h"
#include "ros/log.h"

namespace ros
{

// Abstract implementation of publisher to hide template type
class PublisherImpl
{
protected:

public:
  typedef boost::shared_ptr<PublisherImpl> Ptr;
  typedef boost::weak_ptr<PublisherImpl> WkPtr;

public:
  virtual void newRemoteSubscriberRegistered( std::string& r_host, int& r_port ) = 0;
  virtual const char* getMessageMD5Sum() = 0;
  virtual const std::string& getTopic() = 0;

};

// Templated implementation of publisher
template<class M>
class PublisherTImpl : public PublisherImpl
{
private:
  //typedef M mytype;

  typedef std::vector<PublisherConnection<M>*> ConnectionsType;
  ConnectionsType connections_;

  boost::asio::io_service* io_service_;
  const std::string topic_;
  const std::string caller_id_;

public:
  PublisherTImpl( boost::asio::io_service* io_service, const std::string& topic, const std::string& caller_id ) :
    io_service_(io_service),
    topic_(topic),
    caller_id_(caller_id)
  {
  }

  ~PublisherTImpl()
  {
    for( typename ConnectionsType::iterator it = connections_.begin(); it != connections_.end(); it++ )
    {
      delete (*it);
    }
  }

  const std::string& getTopic()
  {
    return topic_;
  }

  void publish( const M& message )
  {
    for( typename ConnectionsType::iterator it = connections_.begin(); it != connections_.end(); it++ )
    {
      (*it)->publish(message);
    }
  }

  void newRemoteSubscriberRegistered( std::string& r_host, int& r_port )
  {
    r_host = boost::asio::ip::host_name();
    r_port = 9991;

    PublisherConnection<M> * p = new PublisherConnection<M>( io_service_, caller_id_ );
    p->open(r_port);
    connections_.push_back( p );
  }

  const char* getMessageMD5Sum()
  {
    return ros::message_traits::MD5Sum<M>::value();
  }

};

}

#endif
