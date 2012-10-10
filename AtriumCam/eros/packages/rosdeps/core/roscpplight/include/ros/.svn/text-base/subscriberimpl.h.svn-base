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

#ifndef _ROSCPPLIGHT_SUBSCRIBER_IMPL_H_INCLUDED_
#define _ROSCPPLIGHT_SUBSCRIBER_IMPL_H_INCLUDED_

#include "ros/subscriber_connection.h"
#include <boost/asio.hpp>
#include <boost/function.hpp>

namespace ros
{

class SubscriberImpl
{
public:
  typedef boost::shared_ptr<SubscriberImpl> Ptr;
  typedef boost::weak_ptr<SubscriberImpl> WkPtr;

protected:
  SubscriberImpl()
  {
  }

public:
  virtual ~SubscriberImpl()
  {
  }

  virtual void newRemotePublisherRegistered( const std::string& host, int port ) = 0;
};

template<typename M>
class SubscriberTImpl : public SubscriberImpl
{
private:
  boost::asio::io_service * io_service_;

  typedef std::vector<SubscriberConnection<M>*> ConnectionsType;
  ConnectionsType connections_;
  boost::function<void (M)> callback_;
  const std::string topic_;
  const std::string caller_id_;

public:
  SubscriberTImpl( boost::asio::io_service* io_service, boost::function<void (M)> callback, const std::string& topic, const std::string& caller_id ) :
    io_service_(io_service),
    callback_(callback),
    topic_(topic),
    caller_id_(caller_id)
  {
  }

  ~SubscriberTImpl()
  {
    for( typename ConnectionsType::iterator i = connections_.begin(); i != connections_.end(); i++)
    {
      delete (*i);
    }
  }

  void newRemotePublisherRegistered( const std::string& host, int port )
  {
    SubscriberConnection<M> * conn = new SubscriberConnection<M>( io_service_, callback_ );
    conn->connect( host, port, topic_, caller_id_ );
    if( conn->getState() != SubscriberConnection<M>::ERROR )
    {
      connections_.push_back( conn );
    }
    else
    {
      delete conn;
    }
  }
};

}

#endif
