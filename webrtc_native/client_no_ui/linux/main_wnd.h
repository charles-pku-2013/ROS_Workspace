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

#ifndef PEERCONNECTION_SAMPLES_CLIENT_MAIN_WND_H_
#define PEERCONNECTION_SAMPLES_CLIENT_MAIN_WND_H_
#pragma once

// #include <map>
// #include <string>

#include <stdarg.h>
#include "talk/app/webrtc/mediastreaminterface.h"
#include "peer_connection_client.h"
#include "talk/media/base/mediachannel.h"
#include "talk/media/base/videocommon.h"
#include "talk/media/base/videoframe.h"
#include "talk/media/base/videorenderer.h"
// #include "webrtc/base/win32.h"

#include <deque>
#include <pthread.h>
#include <signal.h>
#include <sys/types.h>

// wrapper for signal handling =======================================
typedef void    Sigfunc(int);   /* for signal handlers */

inline
void err_sys(const char *fmt, ...)
{
    va_list     ap;

    va_start(ap, fmt);
    vfprintf( stderr, fmt, ap );
    va_end(ap);
    exit(1);
}

inline
Sigfunc* signal(int signo, Sigfunc *func)
{
    struct sigaction    act, oact;

    act.sa_handler = func;
    sigemptyset(&act.sa_mask);
    act.sa_flags = 0;
    if (signo == SIGALRM) {
#ifdef  SA_INTERRUPT
        act.sa_flags |= SA_INTERRUPT;   /* SunOS 4.x */
#endif
    } else {
#ifdef  SA_RESTART
        act.sa_flags |= SA_RESTART;     /* SVR4, 44BSD */
#endif
    }
    if (sigaction(signo, &act, &oact) < 0)
        return(SIG_ERR);
    return(oact.sa_handler);
}

inline
Sigfunc* Signal(int signo, Sigfunc *func)   /* for our signal() function */
{
    Sigfunc *sigfunc;

    if ( (sigfunc = signal(signo, func)) == SIG_ERR)
        err_sys("signal error");
    return(sigfunc);
}
// ===================================================================


class MsgHandleCallback {
public:
    virtual void MsgThreadCallback(int msg_id, void* data) = 0;
    virtual ~MsgHandleCallback() {}
};


struct MsgHandleCallbackData {
    explicit MsgHandleCallbackData( int id, void* d)
                : msg_id(id), data(d) {}
    int msg_id;
    void* data;
};


// TODO singleton
class MsgQueue : public std::deque< MsgHandleCallbackData > {
public:
    MsgQueue() : _lock(PTHREAD_MUTEX_INITIALIZER)
               , _cond(PTHREAD_COND_INITIALIZER)
               , _callback(NULL) {}

    // NOTE!!! 启动前应该先调用它
    void RegisterCallback( MsgHandleCallback *cb ) { _callback = cb; }
    void QueueMsgHandleCallback( int msg_id, void* data );
    void StartEventLoop();  // start the thread
public:
    void DoWork();          // thread routine
protected:
    pthread_mutex_t     _lock;
    pthread_cond_t      _cond;
    MsgHandleCallback   *_callback;
    pthread_t           _tid;
};



/*
class MainWndCallback {
 public:
  virtual void StartLogin(const std::string& server, int port) = 0;
  virtual void DisconnectFromServer() = 0;
  virtual void ConnectToPeer(int peer_id) = 0;
  virtual void DisconnectFromCurrentPeer() = 0;
  virtual void UIThreadCallback(int msg_id, void* data) = 0;
  virtual void Close() = 0;
 protected:
  virtual ~MainWndCallback() {}
};


// Pure virtual interface for the main window.
class MainWindow {
 public:
  virtual ~MainWindow() {}

  enum UI {
    CONNECT_TO_SERVER,
    LIST_PEERS,
    STREAMING,
  };

  virtual void RegisterObserver(MainWndCallback* callback) = 0;

  virtual bool IsWindow() = 0;
  virtual void MessageBox(const char* caption, const char* text,
                          bool is_error) = 0;

  virtual UI current_ui() = 0;

  virtual void SwitchToConnectUI() = 0;
  virtual void SwitchToPeerList(const Peers& peers) = 0;
  virtual void SwitchToStreamingUI() = 0;

  virtual void StartLocalRenderer(webrtc::VideoTrackInterface* local_video) = 0;
  virtual void StopLocalRenderer() = 0;
  virtual void StartRemoteRenderer(webrtc::VideoTrackInterface* remote_video) = 0;
  virtual void StopRemoteRenderer() = 0;

  virtual void QueueUIThreadCallback(int msg_id, void* data) = 0;
};

#ifdef WIN32

class MainWnd : public MainWindow {
 public:
  static const wchar_t kClassName[];

  enum WindowMessages {
    UI_THREAD_CALLBACK = WM_APP + 1,
  };

  MainWnd(const char* server, int port, bool auto_connect, bool auto_call);
  ~MainWnd();

  bool Create();
  bool Destroy();
  bool PreTranslateMessage(MSG* msg);

  virtual void RegisterObserver(MainWndCallback* callback);
  virtual bool IsWindow();
  virtual void SwitchToConnectUI();
  virtual void SwitchToPeerList(const Peers& peers);
  virtual void SwitchToStreamingUI();
  virtual void MessageBox(const char* caption, const char* text,
                          bool is_error);
  virtual UI current_ui() { return ui_; }

  virtual void StartLocalRenderer(webrtc::VideoTrackInterface* local_video);
  virtual void StopLocalRenderer();
  virtual void StartRemoteRenderer(webrtc::VideoTrackInterface* remote_video);
  virtual void StopRemoteRenderer();

  virtual void QueueUIThreadCallback(int msg_id, void* data);

  HWND handle() const { return wnd_; }

  class VideoRenderer : public webrtc::VideoRendererInterface {
   public:
    VideoRenderer(HWND wnd, int width, int height,
                  webrtc::VideoTrackInterface* track_to_render);
    virtual ~VideoRenderer();

    void Lock() {
      ::EnterCriticalSection(&buffer_lock_);
    }

    void Unlock() {
      ::LeaveCriticalSection(&buffer_lock_);
    }

    // VideoRendererInterface implementation
    virtual void SetSize(int width, int height);
    virtual void RenderFrame(const cricket::VideoFrame* frame);

    const BITMAPINFO& bmi() const { return bmi_; }
    const uint8* image() const { return image_.get(); }

   protected:
    enum {
      SET_SIZE,
      RENDER_FRAME,
    };

    HWND wnd_;
    BITMAPINFO bmi_;
    rtc::scoped_ptr<uint8[]> image_;
    CRITICAL_SECTION buffer_lock_;
    rtc::scoped_refptr<webrtc::VideoTrackInterface> rendered_track_;
  };

  // A little helper class to make sure we always to proper locking and
  // unlocking when working with VideoRenderer buffers.
  template <typename T>
  class AutoLock {
   public:
    explicit AutoLock(T* obj) : obj_(obj) { obj_->Lock(); }
    ~AutoLock() { obj_->Unlock(); }
   protected:
    T* obj_;
  };

 protected:
  enum ChildWindowID {
    EDIT_ID = 1,
    BUTTON_ID,
    LABEL1_ID,
    LABEL2_ID,
    LISTBOX_ID,
  };

  void OnPaint();
  void OnDestroyed();

  void OnDefaultAction();

  bool OnMessage(UINT msg, WPARAM wp, LPARAM lp, LRESULT* result);

  static LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp);
  static bool RegisterWindowClass();

  void CreateChildWindow(HWND* wnd, ChildWindowID id, const wchar_t* class_name,
                         DWORD control_style, DWORD ex_style);
  void CreateChildWindows();

  void LayoutConnectUI(bool show);
  void LayoutPeerListUI(bool show);

  void HandleTabbing();

 private:
  rtc::scoped_ptr<VideoRenderer> local_renderer_;
  rtc::scoped_ptr<VideoRenderer> remote_renderer_;
  UI ui_;
  HWND wnd_;
  DWORD ui_thread_id_;
  HWND edit1_;
  HWND edit2_;
  HWND label1_;
  HWND label2_;
  HWND button_;
  HWND listbox_;
  bool destroyed_;
  void* nested_msg_;
  MainWndCallback* callback_;
  static ATOM wnd_class_;
  std::string server_;
  std::string port_;
  bool auto_connect_;
  bool auto_call_;
};
#endif  // WIN32
 */


#endif  // PEERCONNECTION_SAMPLES_CLIENT_MAIN_WND_H_
