#include "DataReceiver.h"
#include <assert.h>
#include <opencv2/opencv.hpp>


#include "YUVColorConvert.h"


static YUVColorConvert *cvt = YUVColorConvert::GetInstance();


DataReceiver::DataReceiver()
{

};

void DataReceiver::init(const char* endpoint)
{
  if (m_endpoint != endpoint)
  {
    strcpy(m_endpoint, endpoint);
  }

  m_context = zmq_ctx_new();
  // Socket to talk to server
  m_requester = zmq_socket(m_context, ZMQ_SUB);//
  int rc = zmq_connect(m_requester, m_endpoint);
  //assert(rc == 0);
  int recvhwm = 10; // recv buffer size
  zmq_setsockopt(m_requester, ZMQ_SUBSCRIBE, "", 0);
  zmq_setsockopt(m_requester, ZMQ_RCVHWM, &recvhwm, sizeof(int));
  zmq_msg_init(&m_recv_msg);
}

DataReceiver::~DataReceiver()
{
  zmq_msg_close(&m_recv_msg);
  zmq_close(m_requester);
  zmq_ctx_destroy(m_context);
}

void DataReceiver::reconnect() {

  zmq_msg_close(&m_recv_msg);
  zmq_close(m_requester);
  zmq_ctx_destroy(m_context);

  init(m_endpoint);
}

int DataReceiver::RecvFrame(FrameInfo *frame) {
    int recv_to = 1000;   // 50 ms;
    zmq_setsockopt(m_requester, ZMQ_RCVTIMEO, &recv_to, sizeof(int));

    //int len = zmq_msg_recv(&m_recv_msg, m_requester, 0);

    int more;
    size_t more_size = sizeof(more);
    {
      zmq_msg_t msg_proto;
      int rc = zmq_msg_init(&msg_proto);
      assert(rc == 0);

      rc = zmq_msg_recv(&msg_proto, m_requester, 0);
      if (rc == -1) {
        return 0;
      }

      int protolen = zmq_msg_size(&msg_proto);
      printf("protolen = %d\n", protolen);

      frame->meta.ParseFromArray(zmq_msg_data(&msg_proto), protolen);

      zmq_msg_close(&msg_proto);
    }

    RecvImageFrame(frame);

    zmq_getsockopt(m_requester, ZMQ_RCVMORE, &more, &more_size);
    if (!more) {
      return 1;
    }

    RecvParsingFrame(frame);

    zmq_getsockopt(m_requester, ZMQ_RCVMORE, &more, &more_size);
    if (!more) {
      return 1;
    }

    return 1;
}

void DataReceiver::RecvImageFrame(FrameInfo *frame) {
    int cnt = frame->meta.data().image_size();
    frame->img_data.resize(cnt);
    for(int i = 0; i < cnt; i++){
        zmq_msg_t msg;
        int rc = zmq_msg_init(&msg);
        assert(rc == 0);
        rc = zmq_msg_recv(&msg, m_requester, 0);
        int dlen = zmq_msg_size(&msg);
        frame->img_data[i].resize(dlen);
        memcpy(frame->img_data[i].data(), zmq_msg_data(&msg), zmq_msg_size(&msg));
        zmq_msg_close(&msg);
    }
}

  void DataReceiver::RecvParsingFrame(FrameInfo *frame) {
      int cnt = frame->meta.data().structure_perception().parsing_size();
      frame->parsing_data.resize(cnt);
      for (int i = 0; i < cnt; i++) {
        zmq_msg_t msg;
        int rc = zmq_msg_init(&msg);
        rc = zmq_msg_recv(&msg, m_requester, 0);
        int parsing_len = zmq_msg_size(&msg);
        frame->parsing_data[i].resize(parsing_len);
        memcpy(frame->parsing_data[i].data(), zmq_msg_data(&msg), parsing_len);
        zmq_msg_close(&msg);
      }
  }

