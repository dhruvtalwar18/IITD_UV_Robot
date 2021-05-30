package org.ros.internal.transport.queue;

import java.util.concurrent.ExecutorService;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.concurrent.CancellableLoop;
import org.ros.concurrent.CircularBlockingDeque;
import org.ros.concurrent.EventDispatcher;
import org.ros.concurrent.ListenerGroup;
import org.ros.concurrent.SignalRunnable;
import org.ros.message.MessageListener;

public class MessageDispatcher<T> extends CancellableLoop {
    private static final boolean DEBUG = false;
    private static final Log log = LogFactory.getLog(MessageDispatcher.class);
    private boolean latchMode = false;
    private LazyMessage<T> latchedMessage;
    private final CircularBlockingDeque<LazyMessage<T>> lazyMessages;
    private final ListenerGroup<MessageListener<T>> messageListeners;
    private final Object mutex = new Object();

    public MessageDispatcher(CircularBlockingDeque<LazyMessage<T>> lazyMessages2, ExecutorService executorService) {
        this.lazyMessages = lazyMessages2;
        this.messageListeners = new ListenerGroup<>(executorService);
    }

    public void addListener(MessageListener<T> messageListener, int limit) {
        synchronized (this.mutex) {
            EventDispatcher<MessageListener<T>> eventDispatcher = this.messageListeners.add(messageListener, limit);
            if (this.latchMode && this.latchedMessage != null) {
                eventDispatcher.signal(newSignalRunnable(this.latchedMessage));
            }
        }
    }

    public boolean removeListener(MessageListener<T> messageListener) {
        boolean remove;
        synchronized (this.mutex) {
            remove = this.messageListeners.remove(messageListener);
        }
        return remove;
    }

    public void removeAllListeners() {
        synchronized (this.mutex) {
            this.messageListeners.shutdown();
        }
    }

    private SignalRunnable<MessageListener<T>> newSignalRunnable(final LazyMessage<T> lazyMessage) {
        return new SignalRunnable<MessageListener<T>>() {
            public void run(MessageListener<T> messageListener) {
                messageListener.onNewMessage(lazyMessage.get());
            }
        };
    }

    public void setLatchMode(boolean enabled) {
        this.latchMode = enabled;
    }

    public boolean getLatchMode() {
        return this.latchMode;
    }

    public void loop() throws InterruptedException {
        LazyMessage<T> lazyMessage = this.lazyMessages.takeFirst();
        synchronized (this.mutex) {
            this.latchedMessage = lazyMessage;
            this.messageListeners.signal(newSignalRunnable(this.latchedMessage));
        }
    }

    /* access modifiers changed from: protected */
    public void handleInterruptedException(InterruptedException e) {
        this.messageListeners.shutdown();
    }
}
