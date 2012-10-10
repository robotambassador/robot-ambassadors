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

#include "ros/nodexmlrpcparser.h"
#include "ros/nodehandle.h"
#include <boost/regex.hpp>

namespace ros
{

bool NodeXmlRpcParser::parseUrl(const std::string& url, std::string& r_host, int& r_port )
{
  boost::regex expr("http://([^:]+):([\\d]+)/*");
  boost::smatch what;
  if (boost::regex_search(url, what, expr))
  {
    r_host = what[1].str();
    r_port = atoi(what[2].str().c_str());
    return true;
  }
  else
  {
    ROS_ERROR("Invalid XmlRpc URL received: %s", url.c_str());
    return false;
  }
}

void NodeXmlRpcParser::handlePublisherList( const std::string& topic, XmlRpc::XmlRpcValue& publishers )
{
  for( int i = 0; i < publishers.size(); i++ )
  {
    std::string url = publishers[i];

    std::string host;
    int port;
    if( parseUrl( url, host, port ) )
    {
      nh_->newRemotePublisherRegistered( topic, host, port );
    }
    else
    {
      ROS_WARN("Ignoring publisher %s for topic %s", url.c_str(), topic.c_str());
    }
  }
}

}
