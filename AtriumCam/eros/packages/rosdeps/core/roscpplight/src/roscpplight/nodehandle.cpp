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

#include "ros/nodehandle.h"
#include "ros/publisher.h"
#include "ros/nodexmlrpcserver.h"

namespace ros
{

NodeHandle::NodeHandle( const std::string& node_id ) :
  master_("localhost", 11311, node_id, this),
  server_( this, 9999 ),
  caller_id_(node_id)
{
}

NodeHandle::~NodeHandle()
{
}

void NodeHandle::work()
{
  server_.Work();
  io_service_.poll();
  io_service_.reset();
}

void NodeHandle::wait( double seconds )
{
  // todo: do this async to allow for other work to be completed while waiting
  boost::asio::deadline_timer t(io_service_, boost::posix_time::seconds(seconds));
  t.wait();
}

void NodeHandle::newRemotePublisherRegistered( const std::string& topic, const std::string& host, int port )
{
  NodeXmlRpcClient other_node( host, port, caller_id_, this );
  std::string conn_host;
  int conn_port;
  other_node.requestTopic( topic, conn_host, conn_port );
  // todo: find correct subscriber
  if( !subscribers_.empty() )
  {
    SubscriberImpl::WkPtr w_sub = subscribers_.at(0);
    if( SubscriberImpl::Ptr sub = w_sub.lock())
    {
      sub->newRemotePublisherRegistered( conn_host, conn_port );
    }
    else
    {
      // todo: the subscriber has been killed
    }
  }
}

void NodeHandle::newRemoteSubscriberRegistered( const std::string& topic, std::string& r_host, int& r_port )
{
  // todo: find correct publisher
  if( !publishers_.empty() )
  {
    PublisherImpl::WkPtr w_pub = publishers_.at(0);
    if(PublisherImpl::Ptr pub = w_pub.lock())
    {
      pub->newRemoteSubscriberRegistered( r_host, r_port );
    }
    else
    {
      // todo: the publisher has been killed
    }
  }
}

}
