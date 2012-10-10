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

#ifndef _ROSCPPLIGHT_NODEHANDLE_H_INCLUDED_
#define _ROSCPPLIGHT_NODEHANDLE_H_INCLUDED_

#include "ros/nodexmlrpcclient.h"
#include "ros/nodexmlrpcserver.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <boost/asio.hpp>

namespace ros
{

class NodeHandle
{
private:
  // XmlRpc and Network Stuff
  NodeXmlRpcClient master_;
  NodeXmlRpcServer server_;
  boost::asio::io_service io_service_;

  // Child Instances
  typedef std::vector<PublisherImpl::WkPtr> PublishersType;
  typedef std::vector<SubscriberImpl::WkPtr> SubscribersType;
  PublishersType publishers_;
  SubscribersType subscribers_;

  // Node Info
  const std::string caller_id_;

public:
  NodeHandle( const std::string& node_id );
  ~NodeHandle();

  void work();
  void wait( double seconds );

  template<typename M>
  Publisher advertise(const std::string& topic, unsigned int queue_size, bool latch = false)
  {
    Publisher p = Publisher::create<M>( &io_service_, topic, caller_id_ );
    publishers_.push_back(p.getWeakPtr());
    master_.registerPublisher(topic, ros::message_traits::DataType<M>::value(), "http://morten-s760:9999/");
    return p;
  }

  template<typename M>
  Subscriber subscribe(const std::string& topic, unsigned int queue_size, boost::function<void (M)> callback )
  {
    Subscriber s = Subscriber::create<M>( &io_service_, callback, topic, caller_id_ );
    subscribers_.push_back(s.getWeakPtr());
    master_.registerSubscriber(topic, ros::message_traits::DataType<M>::value(), "http://morten-s760:9999/");
    return s;
  }

  void newRemotePublisherRegistered( const std::string& topic, const std::string& host, int port );
  void newRemoteSubscriberRegistered( const std::string& topic, std::string& r_host, int& r_port );

};

}

#endif
