/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Worcester Polytechnic Institute
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
 *   * Neither the name of the Worcester Polytechnic Institute nor the names of its
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
 *********************************************************************/

#include "web_video_server/vp8_streamer.h"

namespace web_video_server
{

Vp8Streamer::Vp8Streamer(const async_web_server_cpp::HttpRequest& request,
                         async_web_server_cpp::HttpConnectionPtr connection, ros::NodeHandle& nh) :
    LibavStreamer(request, connection, nh, "webm", "libvpx", "video/webm")
{
  quality_ = request.get_query_param_value_or_default("quality", "realtime");
}
Vp8Streamer::~Vp8Streamer()
{
}

void Vp8Streamer::initializeEncoder()
{
  typedef std::map<std::string, std::string> AvOptMap;
  AvOptMap av_opt_map;
  av_opt_map["quality"] = quality_;
  av_opt_map["deadline"] = "1";
  av_opt_map["auto-alt-ref"] = "0";
  av_opt_map["lag-in-frames"] = "1";
  av_opt_map["rc_lookahead"] = "1";
  av_opt_map["drop_frame"] = "1";
  av_opt_map["error-resilient"] = "1";

  for (AvOptMap::iterator itr = av_opt_map.begin(); itr != av_opt_map.end(); ++itr)
  {
    av_opt_set(codec_context_->priv_data, itr->first.c_str(), itr->second.c_str(), 0);
  }

  // Buffering settings
  int bufsize = 10;
  codec_context_->rc_buffer_size = bufsize;
  codec_context_->rc_initial_buffer_occupancy = bufsize; //bitrate/3;
  av_opt_set_int(codec_context_->priv_data, "bufsize", bufsize, 0);
  av_opt_set_int(codec_context_->priv_data, "buf-initial", bufsize, 0);
  av_opt_set_int(codec_context_->priv_data, "buf-optimal", bufsize, 0);
  av_opt_set_int(codec_context_->priv_data, "skip_threshold", 10, 0);
}

Vp8StreamerType::Vp8StreamerType() :
    LibavStreamerType("webm", "libvpx", "video/webm")
{
}

boost::shared_ptr<ImageStreamer> Vp8StreamerType::create_streamer(const async_web_server_cpp::HttpRequest& request,
                                                                  async_web_server_cpp::HttpConnectionPtr connection,
                                                                  ros::NodeHandle& nh)
{
  return boost::shared_ptr<ImageStreamer>(new Vp8Streamer(request, connection, nh));
}

}
