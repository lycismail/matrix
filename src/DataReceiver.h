#ifndef __DATA_RECEIVER_H__
#define __DATA_RECEIVER_H__

#include "zeromq/zmq.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "protocol/meta.pb.h"

#define fast_clamp(a ,lo,hi) (a + ((lo-a) & (lo-a < 0) - 1) + (hi-a & (hi-a > 0) - 1))

class FrameInfo{
public:
    Meta::Meta meta;
    std::vector<std::vector<uint8_t> > img_data;
    std::vector<std::vector<int8_t> > parsing_data;
    int64_t ts;   // timestamp for all messages received for current frame
};

class DataReceiver
{
private:
  void * m_context;
  void * m_requester;
  char   m_endpoint[100];
  zmq_msg_t m_recv_msg;

public:
  DataReceiver();
  ~DataReceiver();

  void init(const char* endpoint);

  void reconnect();

  int RecvFrame(FrameInfo *frame);
  void RecvImageFrame(FrameInfo *frame);
  void RecvParsingFrame(FrameInfo *frame);
};
#endif
