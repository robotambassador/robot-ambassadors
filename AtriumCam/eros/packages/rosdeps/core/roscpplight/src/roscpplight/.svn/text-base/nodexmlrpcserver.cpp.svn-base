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

#include "ros/nodexmlrpcserver.h"
#include "ros/nodehandle.h"

using namespace XmlRpc;

namespace ros
{

void NodeXmlRpcServer::ClientMethod::execute(XmlRpcValue& params, XmlRpcValue& result)
{
  ROS_INFO("XmlRpcServer method called: %s", methodName_.c_str() );

  if( methodName_.compare("requestTopic") == 0 )
  {
    return executeRequestTopic(params, result);
  }
  else if( methodName_.compare("publisherUpdate") == 0 )
  {
    return executePublisherUpdate(params, result);
  }
}

void NodeXmlRpcServer::ClientMethod::executeRequestTopic(XmlRpcValue& params, XmlRpcValue& result)
{
  std::string topic = params[1];
  std::string host;
  int port;
  nh_->newRemoteSubscriberRegistered( topic, host, port );

  result[0] = 1;
  result[1] = "ready on xxx:xxx";

  XmlRpcValue protocol_params;
  protocol_params[0] = "TCPROS";
  protocol_params[1] = host;
  protocol_params[2] = port;

  result[2] = protocol_params;
}

void NodeXmlRpcServer::ClientMethod::executePublisherUpdate(XmlRpcValue& params, XmlRpcValue& result)
{
  std::string topic = params[1];
  NodeXmlRpcParser::handlePublisherList(topic, params[2]);
}

NodeXmlRpcServer::NodeXmlRpcServer( NodeHandle * nh, int port ) :
    methodPublisherUpdate_( &rpcserver_, "publisherUpdate", nh ),
    methodRequestTopic_( &rpcserver_, "requestTopic", nh )
{
  bool result = rpcserver_.bindAndListen( port );

  // todo: handle bind error (result == false)
  ROS_ASSERT(result);

  ROS_INFO("NodeXmlRpcServer listening on port: %i", port );
}

NodeXmlRpcServer::~NodeXmlRpcServer()
{
  rpcserver_.close();
}

void NodeXmlRpcServer::Work()
{
  rpcserver_.work(0.01);
}

}
