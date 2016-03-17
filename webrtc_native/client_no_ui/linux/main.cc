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
#include "flagdefs.h"
#include "linux/main_wnd.h"
#include "peer_connection_client.h"
#include "defaults.h"

#include "webrtc/base/ssladapter.h"
#include "webrtc/base/thread.h"

#include "dbg.h"
#include <signal.h>


static volatile int	is_running = 1;

static Conductor                *pConductor = NULL;
static PeerConnectionClient     *pClient = NULL;

static
void sig_int(int signo)
{ 
    DBG("Received signal SIGINT");
    if( pConductor )
        pConductor->Close();
    is_running = 0;
}

class CustomSocketServer : public rtc::PhysicalSocketServer {
public:
    CustomSocketServer(rtc::Thread* thread)
        : thread_(thread), conductor_(NULL), client_(NULL) {}
    virtual ~CustomSocketServer() {}

    void set_client(PeerConnectionClient* client) { client_ = client; }
    void set_conductor(Conductor* conductor) { conductor_ = conductor; }

    // Override so that we can also pump the GTK message loop.
    virtual bool Wait(int cms, bool process_io) 
    {
        if ( !is_running && !conductor_->connection_active() &&
                   client_ != NULL && !client_->is_connected() ) {
            DBG("terminating main thread.");
            thread_->Quit();
        }
        return rtc::PhysicalSocketServer::Wait(0/*cms == -1 ? 1 : cms*/,
                process_io);
    }

protected:		// 所有成员变量只在Wait()中使用
    rtc::Thread* thread_;
    Conductor* conductor_;	// connection_active()
    PeerConnectionClient* client_;	// is_connected()
};


static 
void ParseArgs( int argc, char **argv )
{
    if( argc < 2 ) {
        DBG( "No arg specified, all use defaults." );
        return;
    } // if

    const char *prefixServer = "--server=";
    const size_t lenPrefixServer = strlen(prefixServer);
    const char *prefixCamera = "--camera=";
    const size_t lenPrefixCamera = strlen(prefixCamera);
    const char *prefixMaxVideoSize = "--maxvideosize=";
    const size_t lenPrefixMaxVideoSize = strlen(prefixMaxVideoSize);
    const char *prefixMaxFps = "--maxfps=";
    const size_t lenPrefixMaxFps = strlen(prefixMaxFps);

    for( int i = 1; i < argc; ++i ) {
        if( strncmp(argv[i], prefixServer, lenPrefixServer) == 0 ) {
            char *fullServerName = argv[i] + lenPrefixServer;
            char *sep = strchr( fullServerName, ':' );
            if( sep == NULL || *(sep+1) == 0 ) {
                printf( "Invalid server format!\n" );
            } else {
                *sep++ = 0;
                ServerName = fullServerName;
                ServerPort = atoi(sep);
            } // if sep
        } else if( strncmp(argv[i], prefixCamera, lenPrefixCamera) == 0 ) {
            CameraId = argv[i] + lenPrefixCamera;
        } else if( strncmp(argv[i], prefixMaxVideoSize, lenPrefixMaxVideoSize) == 0 ) {
            char *videoSizeStr = argv[i] + lenPrefixMaxVideoSize;
            char *sep = strchr( videoSizeStr, 'x' );
            if( !sep ) {
                printf("Invalid video size argument!\n");
            } else {
                *sep++ = 0;
                MaxVideoWidth = atoi( videoSizeStr );
                MaxVideoHeight = atoi( sep );
            } // if sep
        } else if( strncmp(argv[i], prefixMaxFps, lenPrefixMaxFps) == 0 ) {
            char *str = argv[i] + lenPrefixMaxFps;
            MaxFps = atoi(str);
        } else {
            printf( "%s is not a valid argument!\n", argv[i] );
        } // if
    } // for

    DBG( "Specified server: %s:%u", ServerName.c_str(), ServerPort );
    DBG( "Specified camera: %s", CameraId.c_str() );
    DBG( "Specified video arg: MaxVideoWidth = %d, MaxVideoHeight = %d, MaxFps = %d",
            MaxVideoWidth, MaxVideoHeight, MaxFps );
}


int main(int argc, char* argv[]) {
    // gtk_init(&argc, &argv);
    // g_type_init();
    // g_thread_init API is deprecated since glib 2.31.0, see release note:
    // http://mail.gnome.org/archives/gnome-announce-list/2011-October/msg00041.html
    // #if !GLIB_CHECK_VERSION(2, 31, 0)
    // g_thread_init(NULL);
    // #endif

    DBG("Compile time: %s %s", __DATE__, __TIME__);

    ParseArgs( argc, argv );

    rtc::FlagList::SetFlagsFromCommandLine(&argc, argv, true);
    if (FLAG_help) {
        rtc::FlagList::Print(NULL, false);
        return 0;
    }

    // Abort if the user specifies a port that is outside the allowed
    // range [1, 65535].
    if ((FLAG_port < 1) || (FLAG_port > 65535)) {
        printf("Error: %i is not a valid port.\n", FLAG_port);
        return -1;
    }
    // ui
    // GtkMainWnd wnd(FLAG_server, FLAG_port, FLAG_autoconnect, FLAG_autocall);
    // wnd.Create();

    Signal( SIGINT, sig_int );

    /*   
     AutoThread   This wraps the existing operating system thread with a libjingle Thread object 
     and makes it the current thread in the ThreadManager object's thread pool (that is, will return 
     the thread if Thread::CurrentThread is called). 
     */
    rtc::AutoThread auto_thread;
    rtc::Thread* thread = rtc::Thread::Current();	// the main thread
    MsgQueue msgQueue;
    CustomSocketServer socket_server(thread);
    thread->set_socketserver(&socket_server);

    rtc::InitializeSSL();
    // Must be constructed after we set the socketserver.
    PeerConnectionClient client;  
    // client wnd -> registerObserver(conductor)
    rtc::scoped_refptr<Conductor> conductor(
            new rtc::RefCountedObject<Conductor>(&client, &msgQueue));
    socket_server.set_client(&client);
    socket_server.set_conductor(conductor);

    pConductor = conductor.get();
    pClient = &client;

    DBG("Starting msg queue event loop...");
    msgQueue.StartEventLoop();

    conductor->StartLogin( GetDefaultServerName(), GetDefaultServerPort() );
    DBG("going to thread->Run()...");
    thread->Run();	// then socket_server->Wait() 阻塞

    // gtk_main();
    // wnd.Destroy();

    DBG( "going to stop the program..." );
    thread->set_socketserver(NULL);
    // TODO: Run the Gtk main loop to tear down the connection.
    //while (gtk_events_pending()) {
    //  gtk_main_iteration();
    //}
    rtc::CleanupSSL();
    return 0;
}
