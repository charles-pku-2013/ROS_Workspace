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

#ifndef TALK_EXAMPLES_PEERCONNECTION_CLIENT_CONDUCTOR_H_
#define TALK_EXAMPLES_PEERCONNECTION_CLIENT_CONDUCTOR_H_
#pragma once

#include <deque>
#include <map>
#include <set>
#include <string>

#include "talk/app/webrtc/mediastreaminterface.h"
#include "talk/app/webrtc/peerconnectioninterface.h"
#include "main_wnd.h"
#include "peer_connection_client.h"
#include "webrtc/base/scoped_ptr.h"

namespace webrtc {
class VideoCaptureModule;
}  // namespace webrtc

namespace cricket {
class VideoRenderer;
}  // namespace cricket

class Conductor
  : public webrtc::PeerConnectionObserver,
    public webrtc::CreateSessionDescriptionObserver,
    public PeerConnectionClientObserver,
    public MsgHandleCallback {
 public:
 // main_wnd_->QueueUIThreadCallback 要指定一个id
  enum CallbackID {
    MEDIA_CHANNELS_INITIALIZED = 1,
    PEER_CONNECTION_CLOSED,
    SEND_MESSAGE_TO_PEER,
    NEW_STREAM_ADDED,
    STREAM_REMOVED,
  };

  Conductor(PeerConnectionClient* client, MsgQueue* msgQueue);

  bool connection_active() const;

  virtual void Close();

 public:
  ~Conductor();
  bool InitializePeerConnection();
  bool ReinitializePeerConnectionForLoopback();
  bool CreatePeerConnection(bool dtls);
  void DeletePeerConnection();
  void AddStreams();
  cricket::VideoCapturer* OpenVideoCaptureDevice();

  //
  // PeerConnectionObserver implementation.
  //
  virtual void OnStateChange(
      webrtc::PeerConnectionObserver::StateType state_changed) {}
  virtual void OnAddStream(webrtc::MediaStreamInterface* stream);	//?? 只处理 remote stream
  virtual void OnRemoveStream(webrtc::MediaStreamInterface* stream);
  virtual void OnDataChannel(webrtc::DataChannelInterface* channel) {}	//!! 考虑一下用DataChannel api
  virtual void OnRenegotiationNeeded() {}
  virtual void OnIceChange() {}
  virtual void OnIceCandidate(const webrtc::IceCandidateInterface* candidate);

  //
  // PeerConnectionClientObserver implementation.
  //

  virtual void OnSignedIn();

  virtual void OnDisconnected();

  virtual void OnPeerConnected(int id, const std::string& name);

  virtual void OnPeerDisconnected(int id);

  virtual void OnMessageFromPeer(int peer_id, const std::string& message);

  virtual void OnMessageSent(int err);

  virtual void OnServerConnectionFailure();

  //
  // MainWndCallback implementation.
  //

  virtual void StartLogin(const std::string& server, int port);

  virtual void DisconnectFromServer();

  virtual void ConnectToPeer(int peer_id);

  virtual void DisconnectFromCurrentPeer();

  virtual void MsgThreadCallback(int msg_id, void* data);

  // CreateSessionDescriptionObserver implementation.
  virtual void OnSuccess(webrtc::SessionDescriptionInterface* desc);
  virtual void OnFailure(const std::string& error);

 protected:
  // Send a message to the remote peer.
  void SendMessage(const std::string& json_object);

  int peer_id_;		// 正在通信对方的peerid 由 ConnectToPeer 每次赋值,DeletePeerConnection 每次置为-1
  bool loopback_;
  rtc::scoped_refptr<webrtc::PeerConnectionInterface> peer_connection_; // CreatePeerConnection() 建立 DeletePeerConnection 删除
  rtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface>
      peer_connection_factory_;		// InitializePeerConnection() 中建立
  PeerConnectionClient* client_;
  MsgQueue* msg_queue_;
  std::deque<std::string*> pending_messages_;
  std::map<std::string, rtc::scoped_refptr<webrtc::MediaStreamInterface> >
      active_streams_;		// AddStreams()
  std::string server_;		// StartLogin()
};

#endif  // TALK_EXAMPLES_PEERCONNECTION_CLIENT_CONDUCTOR_H_