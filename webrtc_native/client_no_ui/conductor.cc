/*
 * libjingle
 * Copyright 2012 Google Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *  3. The name of the author may not be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "conductor.h"

#include <utility>
#include <vector>

#include "talk/app/webrtc/videosourceinterface.h"
#include "defaults.h"
#include "talk/media/devices/devicemanager.h"
#include "talk/app/webrtc/test/fakeconstraints.h"
#include "webrtc/base/common.h"
#include "webrtc/base/json.h"
#include "webrtc/base/logging.h"

#include "dbg.h"

// Names used for a IceCandidate JSON object.
const char kCandidateSdpMidName[] = "sdpMid";
const char kCandidateSdpMlineIndexName[] = "sdpMLineIndex";
const char kCandidateSdpName[] = "candidate";

// Names used for a SessionDescription JSON object.
const char kSessionDescriptionTypeName[] = "type";
const char kSessionDescriptionSdpName[] = "sdp";

#define DTLS_ON  true
#define DTLS_OFF false

class DummySetSessionDescriptionObserver
    : public webrtc::SetSessionDescriptionObserver {
 public:
  static DummySetSessionDescriptionObserver* Create() {
    return
        new rtc::RefCountedObject<DummySetSessionDescriptionObserver>();
  }
  virtual void OnSuccess() {
    LOG(INFO) << __FUNCTION__;
  }
  virtual void OnFailure(const std::string& error) {
    LOG(INFO) << __FUNCTION__ << " " << error;
  }

 protected:
  DummySetSessionDescriptionObserver() {}
  ~DummySetSessionDescriptionObserver() {}
};

Conductor::Conductor(PeerConnectionClient* client, MsgQueue* msgQueue)
  : peer_id_(-1),
    loopback_(false),
    client_(client),
    msg_queue_(msgQueue) {
  client_->RegisterObserver(this);
  msg_queue_->RegisterCallback(this);
}

Conductor::~Conductor() {
  ASSERT(peer_connection_.get() == NULL);
}

bool Conductor::connection_active() const {
  return peer_connection_.get() != NULL;
}

/*
* GtkMainWnd::OnDestroyed() 关闭主窗口时
*/
void Conductor::Close() {
  client_->SignOut();
  DeletePeerConnection();
}

bool Conductor::InitializePeerConnection() {
  ASSERT(peer_connection_factory_.get() == NULL);
  ASSERT(peer_connection_.get() == NULL);

  DBG("Initializing PeerConnection...");

  peer_connection_factory_  = webrtc::CreatePeerConnectionFactory();

  if (!peer_connection_factory_.get()) {
    printf( "Failed to initialize PeerConnectionFactory!\n");
    DeletePeerConnection();
    return false;
  }

  if (!CreatePeerConnection(DTLS_OFF)) {
    printf( "CreatePeerConnection failed!\n");
    DeletePeerConnection();
  }
  AddStreams();
  DBG_COND( peer_connection_.get(), "Conductor::InitializePeerConnection Success!" );
  return peer_connection_.get() != NULL;
}

bool Conductor::ReinitializePeerConnectionForLoopback() {
  DBG( "loopback_ is true!" );
  loopback_ = true;
  rtc::scoped_refptr<webrtc::StreamCollectionInterface> streams(
      peer_connection_->local_streams());
  peer_connection_ = NULL;
  if (CreatePeerConnection(DTLS_OFF)) {
    for (size_t i = 0; i < streams->count(); ++i)
      peer_connection_->AddStream(streams->at(i));
    peer_connection_->CreateOffer(this, NULL);
  }
  return peer_connection_.get() != NULL;
}

bool Conductor::CreatePeerConnection(bool dtls) {
  ASSERT(peer_connection_factory_.get() != NULL);
  ASSERT(peer_connection_.get() == NULL);

  DBG("DTLS = %d", dtls);  

  // GetPeerConnectionString()
  // return GetEnvVarOrDefault("WEBRTC_CONNECT", "stun:stun.l.google.com:19302");
  webrtc::PeerConnectionInterface::IceServers servers;
  // webrtc::PeerConnectionInterface::IceServer server;
  // server.uri = GetPeerConnectionString();
  // DBG( "Adding server: %s", server.uri.c_str() );
  // servers.push_back(server);

  // local
  // {
  // webrtc::PeerConnectionInterface::IceServer server;
  // server.uri = "123.57.237.175:12580?transport=udp";
  // server.username = "charles";
  // server.password = "sunchao";
  // DBG( "Adding server: %s", server.uri.c_str() );
  // servers.push_back(server);
  // }

  // local
  {
  webrtc::PeerConnectionInterface::IceServer server;
  server.uri = "turn:123.57.237.175:12580";
  server.username = "charles";
  server.password = "sunchao";
  DBG( "Adding server: %s", server.uri.c_str() );
  servers.push_back(server);
  }
  
    // FakeConstraints 用法
  webrtc::FakeConstraints constraints;
  if (dtls) {
    constraints.AddOptional(webrtc::MediaConstraintsInterface::kEnableDtlsSrtp,
                            "true");
  }
  else
  {
    constraints.AddOptional(webrtc::MediaConstraintsInterface::kEnableDtlsSrtp,
                            "false");
  }
    //!! 建立 peer_connection 需要指定一组服务器 函数原型：
/*
       CreatePeerConnection(
           const PeerConnectionInterface::IceServers& servers,
           const MediaConstraintsInterface* constraints,
           PortAllocatorFactoryInterface* allocator_factory,
           DTLSIdentityServiceInterface* dtls_identity_service,
           PeerConnectionObserver* observer)
 */
  peer_connection_ =
      peer_connection_factory_->CreatePeerConnection(servers,
                                                     &constraints,
                                                     NULL,
                                                     NULL,
                                                     this);
  return peer_connection_.get() != NULL;
}

void Conductor::DeletePeerConnection() {
  DBG("DeletePeerConnection peer_id_=%d", peer_id_);
  peer_connection_ = NULL;
  active_streams_.clear();
  // main_wnd_->StopLocalRenderer();
  // main_wnd_->StopRemoteRenderer();
  peer_connection_factory_ = NULL;
  peer_id_ = -1;
  loopback_ = false;
}

//
// PeerConnectionObserver implementation.
//

// Called when a remote stream is added // window new VideoRender 在窗口上显示视频
void Conductor::OnAddStream(webrtc::MediaStreamInterface* stream) {
  LOG(INFO) << __FUNCTION__ << " " << stream->label();
  DBG("New stream %s added.", stream->label().c_str());
  // stream->AddRef();
  // main_wnd_->QueueUIThreadCallback(NEW_STREAM_ADDED,
                                   // stream);
}

void Conductor::OnRemoveStream(webrtc::MediaStreamInterface* stream) {
  LOG(INFO) << __FUNCTION__ << " " << stream->label();
  DBG("Stream %s removed.", stream->label().c_str());
  // stream->AddRef();
  // msg_queue_->QueueMsgHandleCallback(STREAM_REMOVED,
                                   // stream);
}

// called when network candidates become available, 由rtc系统调用
// 实现： 收到candidate要告知对方, See OnMessageFromPeer() peer_connection_->AddIceCandidate(candidate.get())
void Conductor::OnIceCandidate(const webrtc::IceCandidateInterface* candidate) {
  LOG(INFO) << __FUNCTION__ << " " << candidate->sdp_mline_index();
  // For loopback test. To save some connecting delay.
  if (loopback_) {
    DBG( "loopback_ is true" );
    if (!peer_connection_->AddIceCandidate(candidate)) { // 添加到本连接 OnMessageFromPeer() 也有
      LOG(WARNING) << "Failed to apply the received candidate";
    }
    return;
  }

  Json::StyledWriter writer;
  Json::Value jmessage;

  jmessage[kCandidateSdpMidName] = candidate->sdp_mid();
  jmessage[kCandidateSdpMlineIndexName] = candidate->sdp_mline_index();
  std::string sdp;
  if (!candidate->ToString(&sdp)) {
    LOG(LS_ERROR) << "Failed to serialize candidate";
    return;
  }
  DBG("sdp: [%u] = %s", checksum(sdp.c_str()), sdp.c_str());
  jmessage[kCandidateSdpName] = sdp;
  SendMessage(writer.write(jmessage));
}

//
// PeerConnectionClientObserver implementation.
//
//!! 这些负责更新UI的回调函数由 PeerConnectionClient 调用
void Conductor::OnSignedIn() {
  LOG(INFO) << __FUNCTION__;
  DBG("Signin Success!");
  // main_wnd_->SwitchToPeerList(client_->peers());
}

void Conductor::OnDisconnected() {
  LOG(INFO) << __FUNCTION__;

  DeletePeerConnection();
}

void Conductor::OnPeerConnected(int id, const std::string& name) {
  LOG(INFO) << __FUNCTION__;
  // Refresh the list if we're showing it.
  DBG("peer id=%d, name=%s connected.", id, name.c_str());
}

void Conductor::OnPeerDisconnected(int id) {
  LOG(INFO) << __FUNCTION__;
  if (id == peer_id_) {
    LOG(INFO) << "Our peer disconnected";
    msg_queue_->QueueMsgHandleCallback(PEER_CONNECTION_CLOSED, NULL);
  }
}

// call from PeerConnectionClient::OnMessageFromPeer()
void Conductor::OnMessageFromPeer(int peer_id, const std::string& message) {
  ASSERT(peer_id_ == peer_id || peer_id_ == -1);
  ASSERT(!message.empty());

  DBG("Received peer msg: peer_id=%d, msg[%u]=%s", peer_id, checksum(message.c_str()), message.c_str());

  if (!peer_connection_.get()) { // 还没有建立连接，第一次被呼叫
    ASSERT(peer_id_ == -1);
    peer_id_ = peer_id;     //!! 设置正在通信对方peerid

    if (!InitializePeerConnection()) {
      LOG(LS_ERROR) << "Failed to initialize our PeerConnection instance";
      DBG("Fail to init peerconnection!");
      client_->SignOut();
      return;
    } // if
  } else if (peer_id != peer_id_) {
    ASSERT(peer_id_ != -1);
    LOG(WARNING) << "Received a message from unknown peer while already in a "
                    "conversation with a different peer.";
    return;
  } // if !peer_connection_.get() 连接不存在就建立

  Json::Reader reader;
  Json::Value jmessage;
  if (!reader.parse(message, jmessage)) {
    LOG(WARNING) << "Received unknown message. " << message;
    return;
  }
  std::string type;
  std::string json_object;

  rtc::GetStringFromJsonObject(jmessage, kSessionDescriptionTypeName, &type);
  if (!type.empty()) { // 收到 Offer
    if (type == "offer-loopback") {
      // This is a loopback call.
      // Recreate the peerconnection with DTLS disabled.
      DBG("Calling ReinitializePeerConnectionForLoopback()...");
      if (!ReinitializePeerConnectionForLoopback()) {
        LOG(LS_ERROR) << "Failed to initialize our PeerConnection instance";
        DeletePeerConnection();
        client_->SignOut();
      }
      return;
    }
    // 收到Offer，回复Answer
    std::string sdp;
    if (!rtc::GetStringFromJsonObject(jmessage, kSessionDescriptionSdpName,
                                      &sdp)) {
      LOG(WARNING) << "Can't parse received session description message.";
      return;
    }
    DBG("Received remote sdp[%u]:%s", checksum(sdp.c_str()), sdp.c_str());
    webrtc::SessionDescriptionInterface* session_description(
        webrtc::CreateSessionDescription(type, sdp));
    if (!session_description) {
      LOG(WARNING) << "Can't parse received session description message.";
      return;
    }
    LOG(INFO) << " Received session description :" << message;
    peer_connection_->SetRemoteDescription(
        DummySetSessionDescriptionObserver::Create(), session_description);
    if (session_description->type() ==
        webrtc::SessionDescriptionInterface::kOffer) {
      DBG("Answering remote request...");
      peer_connection_->CreateAnswer(this, NULL);
    }
    return;
  } else { // received candidate
    std::string sdp_mid;
    int sdp_mlineindex = 0;
    std::string sdp;
    if (!rtc::GetStringFromJsonObject(jmessage, kCandidateSdpMidName,
                                      &sdp_mid) ||
        !rtc::GetIntFromJsonObject(jmessage, kCandidateSdpMlineIndexName,
                                   &sdp_mlineindex) ||
        !rtc::GetStringFromJsonObject(jmessage, kCandidateSdpName, &sdp)) {
      LOG(WARNING) << "Can't parse received message.";
      return;
    }
    rtc::scoped_ptr<webrtc::IceCandidateInterface> candidate(
        webrtc::CreateIceCandidate(sdp_mid, sdp_mlineindex, sdp));
    if (!candidate.get()) {
      LOG(WARNING) << "Can't parse received candidate message.";
      return;
    }
    if (!peer_connection_->AddIceCandidate(candidate.get())) {  //!! 收到对方peer发来的candidate
      LOG(WARNING) << "Failed to apply the received candidate";
      return;
    }
    DBG("peer_connection_->AddIceCandidate() success: sdp_mid=%s, sdp[%u]=%s",
                sdp_mid.c_str(), checksum(sdp.c_str()), sdp.c_str() );
    LOG(INFO) << " Received candidate :" << message;
    return;
  } // if type.empty() is true
}

void Conductor::OnMessageSent(int err) {
  // Process the next pending message if any.
  msg_queue_->QueueMsgHandleCallback(SEND_MESSAGE_TO_PEER, NULL);
}

void Conductor::OnServerConnectionFailure() {
    printf("Failed to connect to server: %s\n", server_.c_str());
}

//
// MainWndCallback implementation.
//

/*
* GtkMainWnd::OnClicked()
*/
void Conductor::StartLogin(const std::string& server, int port) {
  if (client_->is_connected())
    return;
  server_ = server;
  DBG("Trying to login %s:%d", server.c_str(), port);
  client_->Connect(server, port, GetPeerName());
}

/*
* GtkMainWnd::OnKeyPress()
*/
void Conductor::DisconnectFromServer() {
  if (client_->is_connected())
    client_->SignOut();
}

/*
* GtkMainWnd::OnRowActivated()
*/
void Conductor::ConnectToPeer(int peer_id) {
  ASSERT(peer_id_ == -1);
  ASSERT(peer_id != -1);

  DBG( "Trying to connect to peer: %d", peer_id );

    // 和其他的peer正在通话中
  if (peer_connection_.get()) {
    printf("ERROR! We only support connecting to one peer at a time!\n");
    return;
  }

  if (InitializePeerConnection()) {
    DBG("InitializePeerConnection() Success! createing offer...");
    peer_id_ = peer_id;
    // virtual void CreateOffer(CreateSessionDescriptionObserver* observer,
                           // const RTCOfferAnswerOptions& options);
    // CreateSessionDescriptionObserver OnSuccess() OnFailure()
    peer_connection_->CreateOffer(this, NULL);
  } else {
    printf("ERROR! Failed to initialize PeerConnection");
  }
}

cricket::VideoCapturer* Conductor::OpenVideoCaptureDevice() {
  rtc::scoped_ptr<cricket::DeviceManagerInterface> dev_manager(
      cricket::DeviceManagerFactory::Create());
  if (!dev_manager->Init()) {
    LOG(LS_ERROR) << "Can't create device manager";
    return NULL;
  }
  std::vector<cricket::Device> devs;
  if (!dev_manager->GetVideoCaptureDevices(&devs)) {
    LOG(LS_ERROR) << "Can't enumerate video devices";
    return NULL;
  }
  std::vector<cricket::Device>::iterator dev_it = devs.begin();

  DBG("Video capture devices on this computer:");
  for( ; dev_it != devs.end(); ++dev_it  )
    DBG( "name = %s, id = %s", dev_it->name.c_str(), dev_it->id.c_str() );

  cricket::VideoCapturer* capturer = NULL;
  dev_it = devs.begin();
  for (; dev_it != devs.end(); ++dev_it) {
      if( dev_it->id == CameraId ) {
          capturer = dev_manager->CreateVideoCapturer(*dev_it);
          break;
      } // if
  } // for

  if( !capturer ) {
      printf( "Cannot open specified video device, trying to open the first available!\n"  );
      dev_it = devs.begin();
      for (; dev_it != devs.end(); ++dev_it) {
        capturer = dev_manager->CreateVideoCapturer(*dev_it);
        if (capturer != NULL)
          break;
      } // for
  } // if capture

  if( !capturer )
      printf("NO VIDEO DEVICE AVAILABLE!\n");
  return capturer;
}


//!! 添加本地stream，会导致remote端的OnAddStream被调用
void Conductor::AddStreams() {
  if (active_streams_.find(kStreamLabel) != active_streams_.end())
    return;  // Already added.

  rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track(
      peer_connection_factory_->CreateAudioTrack(
          kAudioLabel, peer_connection_factory_->CreateAudioSource(NULL)));

  DBG("Create Audio track %s!", audio_track.get() ? "Success" : "Fail");

    webrtc::FakeConstraints constraints;
    // constraints.AddMandatory(webrtc::MediaConstraintsInterface::kMaxWidth, MaxVideoWidth);
    // constraints.AddMandatory(webrtc::MediaConstraintsInterface::kMaxHeight, MaxVideoHeight);
    // constraints.AddMandatory(webrtc::MediaConstraintsInterface::kMaxFrameRate, MaxFps);

    constraints.AddMandatory(webrtc::MediaConstraintsInterface::kMinWidth, 320);
    constraints.AddMandatory(webrtc::MediaConstraintsInterface::kMinHeight, 320);
    // constraints.AddMandatory(webrtc::MediaConstraintsInterface::kMaxWidth, 320);
    // constraints.AddMandatory(webrtc::MediaConstraintsInterface::kMaxHeight, 240);
    // constraints.AddMandatory(webrtc::MediaConstraintsInterface::kMinFrameRate, 25);
    constraints.AddMandatory(webrtc::MediaConstraintsInterface::kMaxFrameRate, 15);

  rtc::scoped_refptr<webrtc::VideoTrackInterface> video_track(
      peer_connection_factory_->CreateVideoTrack(
          kVideoLabel,
          peer_connection_factory_->CreateVideoSource(OpenVideoCaptureDevice(),
                                                      &constraints)));
  // CreateVideoSource (cricket::VideoCapturer *capturer, const MediaConstraintsInterface *constraints)
  DBG("Create video track %s!", video_track.get() ? "Success" : "Fail");

  // main_wnd_->StartLocalRenderer(video_track);
  // DBG("Main window local render started!");

  rtc::scoped_refptr<webrtc::MediaStreamInterface> stream =
      peer_connection_factory_->CreateLocalMediaStream(kStreamLabel);

  DBG("CreateLocalMediaStream %s!", stream.get() ? "Success" : "Fail");

  stream->AddTrack(audio_track);
  stream->AddTrack(video_track);
  if (!peer_connection_->AddStream(stream)) {
    LOG(LS_ERROR) << "Adding stream to PeerConnection failed";
    DBG("Adding stream to PeerConnection failed!");
  }
  DBG("peer_connection_->AddStream() Success!");
  typedef std::pair<std::string,
                    rtc::scoped_refptr<webrtc::MediaStreamInterface> >
      MediaStreamPair;
  active_streams_.insert(MediaStreamPair(stream->label(), stream));
  // DBG("MainWindow switching to StreamingUI...");
  // main_wnd_->SwitchToStreamingUI();
}

/*
* GtkMainWnd::OnKeyPress()
*/
void Conductor::DisconnectFromCurrentPeer() {
  LOG(INFO) << __FUNCTION__;
  DBG("Conductor::DisconnectFromCurrentPeer()");
  if (peer_connection_.get()) {
    client_->SendHangUp(peer_id_);
    DeletePeerConnection();
  }
}

// MainWndCallback接口实现 在MainWnd.cc中指定作为主窗口消息处理
void Conductor::MsgThreadCallback(int msg_id, void* data) {
  switch (msg_id) {
    case PEER_CONNECTION_CLOSED:
      LOG(INFO) << "PEER_CONNECTION_CLOSED";
      DBG("PEER_CONNECTION_CLOSED");
      DeletePeerConnection();

      ASSERT(active_streams_.empty());
      break;

    case SEND_MESSAGE_TO_PEER: {
      LOG(INFO) << "SEND_MESSAGE_TO_PEER";
      std::string* msg = reinterpret_cast<std::string*>(data);
      if (msg) {
        // For convenience, we always run the message through the queue.
        // This way we can be sure that messages are sent to the server
        // in the same order they were signaled without much hassle.
        pending_messages_.push_back(msg);
      }

      if (!pending_messages_.empty() && !client_->IsSendingMessage()) {
        msg = pending_messages_.front();
        pending_messages_.pop_front();

        if (!client_->SendToPeer(peer_id_, *msg) && peer_id_ != -1) {
          LOG(LS_ERROR) << "SendToPeer failed";
          DisconnectFromServer();
        }
        delete msg;
      }

      if (!peer_connection_.get())
        peer_id_ = -1;

      break;
    }
/*
    case NEW_STREAM_ADDED: {
      DBG( "NEW_STREAM_ADDED" );
      webrtc::MediaStreamInterface* stream =
          reinterpret_cast<webrtc::MediaStreamInterface*>(
          data);
      webrtc::VideoTrackVector tracks = stream->GetVideoTracks();
      // Only render the first track.
      if (!tracks.empty()) {
        webrtc::VideoTrackInterface* track = tracks[0];
        main_wnd_->StartRemoteRenderer(track);
      }
      stream->Release();
      break;
    }

    case STREAM_REMOVED: {
      // Remote peer stopped sending a stream.
      webrtc::MediaStreamInterface* stream =
          reinterpret_cast<webrtc::MediaStreamInterface*>(
          data);
      stream->Release();
      break;
    }
 */
    default:
      ASSERT(false);
      break;
  }
}

// for CreateOffer success
void Conductor::OnSuccess(webrtc::SessionDescriptionInterface* desc) {
  peer_connection_->SetLocalDescription(
      DummySetSessionDescriptionObserver::Create(), desc);

  std::string sdp;
  desc->ToString(&sdp);

  // For loopback test. To save some connecting delay.
  if (loopback_) {
    // Replace message type from "offer" to "answer"
    webrtc::SessionDescriptionInterface* session_description(
        webrtc::CreateSessionDescription("answer", sdp));
    peer_connection_->SetRemoteDescription(
        DummySetSessionDescriptionObserver::Create(), session_description);
    return;
  }

  Json::StyledWriter writer;
  Json::Value jmessage;
  jmessage[kSessionDescriptionTypeName] = desc->type();
  jmessage[kSessionDescriptionSdpName] = sdp;
  SendMessage(writer.write(jmessage));      //!! Conductor::UIThreadCallback() 实际发送过程在这里
}

void Conductor::OnFailure(const std::string& error) {
    LOG(LERROR) << error;
}

void Conductor::SendMessage(const std::string& json_object) {
  std::string* msg = new std::string(json_object);
  msg_queue_->QueueMsgHandleCallback(SEND_MESSAGE_TO_PEER, msg);
}
