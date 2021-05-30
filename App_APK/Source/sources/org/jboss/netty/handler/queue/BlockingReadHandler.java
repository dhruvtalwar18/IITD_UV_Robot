package org.jboss.netty.handler.queue;

import java.io.IOException;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.TimeUnit;
import org.jboss.netty.channel.ChannelEvent;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.ChannelStateEvent;
import org.jboss.netty.channel.ExceptionEvent;
import org.jboss.netty.channel.MessageEvent;
import org.jboss.netty.channel.SimpleChannelUpstreamHandler;
import org.jboss.netty.util.internal.DeadLockProofWorker;
import org.jboss.netty.util.internal.QueueFactory;

public class BlockingReadHandler<E> extends SimpleChannelUpstreamHandler {
    static final /* synthetic */ boolean $assertionsDisabled = false;
    private volatile boolean closed;
    private final BlockingQueue<ChannelEvent> queue;

    public BlockingReadHandler() {
        this(QueueFactory.createQueue(ChannelEvent.class));
    }

    public BlockingReadHandler(BlockingQueue<ChannelEvent> queue2) {
        if (queue2 != null) {
            this.queue = queue2;
            return;
        }
        throw new NullPointerException("queue");
    }

    /* access modifiers changed from: protected */
    public BlockingQueue<ChannelEvent> getQueue() {
        return this.queue;
    }

    public boolean isClosed() {
        return this.closed;
    }

    public E read() throws IOException, InterruptedException {
        ChannelEvent e = readEvent();
        if (e == null) {
            return null;
        }
        if (e instanceof MessageEvent) {
            return getMessage((MessageEvent) e);
        }
        if (e instanceof ExceptionEvent) {
            throw ((IOException) new IOException().initCause(((ExceptionEvent) e).getCause()));
        }
        throw new IllegalStateException();
    }

    public E read(long timeout, TimeUnit unit) throws IOException, InterruptedException {
        ChannelEvent e = readEvent(timeout, unit);
        if (e == null) {
            return null;
        }
        if (e instanceof MessageEvent) {
            return getMessage((MessageEvent) e);
        }
        if (e instanceof ExceptionEvent) {
            throw ((IOException) new IOException().initCause(((ExceptionEvent) e).getCause()));
        }
        throw new IllegalStateException();
    }

    public ChannelEvent readEvent() throws InterruptedException {
        detectDeadLock();
        if (isClosed() && getQueue().isEmpty()) {
            return null;
        }
        ChannelEvent e = getQueue().take();
        if (e instanceof ChannelStateEvent) {
            return null;
        }
        return e;
    }

    public ChannelEvent readEvent(long timeout, TimeUnit unit) throws InterruptedException, BlockingReadTimeoutException {
        detectDeadLock();
        if (isClosed() && getQueue().isEmpty()) {
            return null;
        }
        ChannelEvent e = getQueue().poll(timeout, unit);
        if (e == null) {
            throw new BlockingReadTimeoutException();
        } else if (e instanceof ChannelStateEvent) {
            return null;
        } else {
            return e;
        }
    }

    private static void detectDeadLock() {
        if (DeadLockProofWorker.PARENT.get() != null) {
            throw new IllegalStateException("read*(...) in I/O thread causes a dead lock or sudden performance drop. Implement a state machine or call read*() from a different thread.");
        }
    }

    public void messageReceived(ChannelHandlerContext ctx, MessageEvent e) throws Exception {
        getQueue().put(e);
    }

    public void exceptionCaught(ChannelHandlerContext ctx, ExceptionEvent e) throws Exception {
        getQueue().put(e);
    }

    public void channelClosed(ChannelHandlerContext ctx, ChannelStateEvent e) throws Exception {
        this.closed = true;
        getQueue().put(e);
    }

    private E getMessage(MessageEvent e) {
        return e.getMessage();
    }
}
