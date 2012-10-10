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

#include "ros/nodexmlrpcclient.h"
#include "ros/nodehandle.h"

using namespace XmlRpc;

namespace ros
{

NodeXmlRpcClient::NodeXmlRpcClient( const std::string& host, int port, const std::string& caller_id, NodeHandle * nh ) :
  XmlRpcClient(host.c_str(), port),
  NodeXmlRpcParser( nh ),
  caller_id_(caller_id)
{
}

NodeXmlRpcClient::~NodeXmlRpcClient()
{
  XmlRpcClient::close();
}

void NodeXmlRpcClient::registerPublisher( const std::string& topic, const std::string& msg_type, const std::string& url)
{
  XmlRpcValue noArgs, result;

  XmlRpcValue args;
  args[0] = XmlRpcValue(caller_id_);
  args[1] = XmlRpcValue(topic);
  args[2] = XmlRpcValue(msg_type);
  args[3] = XmlRpcValue(url);

  if (execute("registerPublisher", args, result))
  {
    std::string r = result[1];
    ROS_INFO("XmlRpc Response: %s", r.c_str() )
  }

  //todo: check for existing subscribers
}

void NodeXmlRpcClient::registerSubscriber( const std::string& topic, const std::string& msg_type, const std::string& url)
{
  XmlRpcValue noArgs, result;

  XmlRpcValue args;
  args[0] = XmlRpcValue(caller_id_);
  args[1] = XmlRpcValue(topic);
  args[2] = XmlRpcValue(msg_type);
  args[3] = XmlRpcValue(url);

  if (execute("registerSubscriber", args, result))
  {
    std::string r = result[1];
    ROS_INFO("XmlRpc Response: %s", r.c_str() )
    NodeXmlRpcParser::handlePublisherList(topic, result[2]);
  }

}

bool NodeXmlRpcClient::requestTopic( const std::string& topic, std::string& r_host, int& r_port )
{
  XmlRpcValue args;
  args[0] = XmlRpcValue(caller_id_);
  args[1] = XmlRpcValue(topic);

  XmlRpcValue parameters;
  parameters[0] = XmlRpcValue("TCPROS");

  XmlRpcValue protocols;
  protocols[0] = parameters;

  args[2] = protocols;

  // {1,ready on <host>:<port>,{TCPROS,<host>,<port>}}
  XmlRpcValue result;
  if (execute("requestTopic", args, result))
  {
    std::string r = result[1];
    ROS_INFO("XmlRpc Response: %s", r.c_str() )

    std::string host = result[2][1];
    r_host = host;
    r_port = result[2][2];
    return true;
  }
  return false;
}

}
