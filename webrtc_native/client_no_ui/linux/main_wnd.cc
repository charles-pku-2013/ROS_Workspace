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

#include "linux/main_wnd.h"

// #include <gdk/gdkkeysyms.h>
// #include <gtk/gtk.h>
#include <stddef.h>

#include "defaults.h"
#include "webrtc/base/common.h"
#include "webrtc/base/logging.h"
#include "webrtc/base/stringutils.h"

#include "dbg.h"

#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <errno.h>


// pthread func wrapper ========================================
static inline
void Pthread_create(pthread_t *tid, const pthread_attr_t *attr,
               void * (*func)(void *), void *arg)
{
    int     n;

    if ( (n = pthread_create(tid, attr, func, arg)) == 0)
        return;
    errno = n;
    err_sys("pthread_create error");
}

static inline
void Pthread_detach(pthread_t tid)
{
    int     n;

    if ( (n = pthread_detach(tid)) == 0)
        return;
    errno = n;
    err_sys("pthread_detach error");
}

static inline
void Pthread_mutex_lock(pthread_mutex_t *mptr)
{
    int     n;

    if ( (n = pthread_mutex_lock(mptr)) == 0 )
        return;
    errno = n;
    err_sys("pthread_mutex_lock error");
}

static inline
void Pthread_mutex_unlock(pthread_mutex_t *mptr)
{
    int     n;

    if ( (n = pthread_mutex_unlock(mptr)) == 0)
        return;
    errno = n;
    err_sys("pthread_mutex_unlock error");
}

static inline
void Pthread_cond_wait(pthread_cond_t *cptr, pthread_mutex_t *mptr)
{
    int     n;

    if ( (n = pthread_cond_wait(cptr, mptr)) == 0)
        return;
    errno = n;
    err_sys("pthread_cond_wait error");
}

static inline
void Pthread_cond_signal(pthread_cond_t *cptr)
{
    int     n;

    if ( (n = pthread_cond_signal(cptr)) == 0)
        return;
    errno = n;
    err_sys("pthread_cond_signal error");
}
// =============================================================


// first unlock, then signal, see apue
void MsgQueue::QueueMsgHandleCallback( int msg_id, void* data )
{
    Pthread_mutex_lock( &_lock );
    this->push_back( MsgHandleCallbackData(msg_id, data) );
    Pthread_mutex_unlock( &_lock );
    Pthread_cond_signal( &_cond );
}


void MsgQueue::DoWork()
{
    assert( this->_callback );

    for( ; ; ) {        // TODO should set a var to stop the loop
        Pthread_mutex_lock( &_lock );
        while( this->empty() )
            Pthread_cond_wait( &_cond, &_lock );
        MsgHandleCallbackData cbData = this->front();
        this->pop_front();
        Pthread_mutex_unlock( &_lock );
        DBG("Processing msg %d", cbData.msg_id);
        _callback->MsgThreadCallback( cbData.msg_id, cbData.data );
    } // for
}


static
void* msg_handle_routine( void *arg )
{
    MsgQueue *qMsg = (MsgQueue*)arg;

    Pthread_detach(pthread_self());
    qMsg->DoWork();

    return (void*)NULL;
}


void MsgQueue::StartEventLoop()
{
    Pthread_create( &_tid, NULL, msg_handle_routine, (void*)this );
    // DoWork();
}






















