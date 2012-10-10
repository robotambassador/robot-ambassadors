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

#ifndef _ROSCPPLIGHT_PUBLISHER_H_INCLUDED_
#define _ROSCPPLIGHT_PUBLISHER_H_INCLUDED_

#include "ros/log.h"
#include "ros/publisherimpl.h"

namespace ros
{

// Wrapper class to hide template
class Publisher
{
protected:
  PublisherImpl::Ptr impl_;

protected:
  Publisher()
  {
  }

public:
  template<typename M>
  static Publisher create( boost::asio::io_service* io_service, const std::string& topic, const std::string& caller_id )
  {
    Publisher p;
    p.impl_.reset(new PublisherTImpl<M> (io_service, topic, caller_id ));
    return p;
  }

  template<typename M>
  void publish(const M& message) const
  {
    if( impl_->getMessageMD5Sum() != ros::message_traits::MD5Sum<M>::value() )
    {
      ROS_FATAL("Call to publish() on an invalid Publisher (topic [%s])", impl_->getTopic().c_str());
      return;
    }

    boost::shared_ptr<PublisherTImpl<M> > t_ptr = boost::dynamic_pointer_cast<PublisherTImpl<M> >(impl_);
    if( t_ptr == 0 )
    {
      ROS_FATAL("dynamic cast error");
      return;
    }

    t_ptr->publish(message);

  }

  PublisherImpl::WkPtr getWeakPtr()
  {
    return PublisherImpl::WkPtr( impl_ );
  }

  ~Publisher()
  {
  }
};

}

#endif
