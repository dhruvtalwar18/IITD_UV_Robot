package org.jboss.netty.handler.queue;

import java.io.IOException;
import java.nio.channels.ClosedChannelException;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.ChannelFutureListener;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.ChannelStateEvent;
import org.jboss.netty.channel.Channels;
import org.jboss.netty.channel.LifeCycleAwareChannelHandler;
import org.jboss.netty.channel.MessageEvent;
import org.jboss.netty.channel.SimpleChannelHandler;
import org.jboss.netty.util.internal.QueueFactory;

public class BufferedWriteHandler extends SimpleChannelHandler implements LifeCycleAwareChannelHandler {
    static final /* synthetic */ boolean $assertionsDisabled = false;
    private final boolean consolidateOnFlush;
    private volatile ChannelHandlerContext ctx;
    private final Queue<MessageEvent> queue;

    public BufferedWriteHandler() {
        this(false);
    }

    public BufferedWriteHandler(Queue<MessageEvent> queue2) {
        this(queue2, false);
    }

    public BufferedWriteHandler(boolean consolidateOnFlush2) {
        this(QueueFactory.createQueue(MessageEvent.class), consolidateOnFlush2);
    }

    public BufferedWriteHandler(Queue<MessageEvent> queue2, boolean consolidateOnFlush2) {
        if (queue2 != null) {
            this.queue = queue2;
            this.consolidateOnFlush = consolidateOnFlush2;
            return;
        }
        throw new NullPointerException("queue");
    }

    public boolean isConsolidateOnFlush() {
        return this.consolidateOnFlush;
    }

    /* access modifiers changed from: protected */
    public Queue<MessageEvent> getQueue() {
        return this.queue;
    }

    public void flush() {
        flush(this.consolidateOnFlush);
    }

    public void flush(boolean consolidateOnFlush2) {
        ChannelHandlerContext ctx2 = this.ctx;
        if (ctx2 != null) {
            Queue<MessageEvent> queue2 = getQueue();
            if (!consolidateOnFlush2) {
                synchronized (this) {
                    while (true) {
                        MessageEvent e = queue2.poll();
                        if (e != null) {
                            ctx2.sendDownstream(e);
                        }
                    }
                }
            } else if (!queue2.isEmpty()) {
                List<MessageEvent> pendingWrites = new ArrayList<>();
                synchronized (this) {
                    while (true) {
                        MessageEvent e2 = queue2.poll();
                        if (e2 == null) {
                            consolidatedWrite(pendingWrites);
                        } else if (!(e2.getMessage() instanceof ChannelBuffer)) {
                            List<MessageEvent> consolidatedWrite = consolidatedWrite(pendingWrites);
                            pendingWrites = consolidatedWrite;
                            if (consolidatedWrite == null) {
                                pendingWrites = new ArrayList<>();
                            }
                            ctx2.sendDownstream(e2);
                        } else {
                            pendingWrites.add(e2);
                        }
                    }
                }
            }
        }
    }

    private List<MessageEvent> consolidatedWrite(final List<MessageEvent> pendingWrites) {
        int size = pendingWrites.size();
        if (size == 1) {
            this.ctx.sendDownstream(pendingWrites.remove(0));
            return pendingWrites;
        } else if (size == 0) {
            return pendingWrites;
        } else {
            ChannelBuffer[] data = new ChannelBuffer[size];
            for (int i = 0; i < data.length; i++) {
                data[i] = (ChannelBuffer) pendingWrites.get(i).getMessage();
            }
            ChannelBuffer composite = ChannelBuffers.wrappedBuffer(data);
            ChannelFuture future = Channels.future(this.ctx.getChannel());
            future.addListener(new ChannelFutureListener() {
                public void operationComplete(ChannelFuture future) throws Exception {
                    if (future.isSuccess()) {
                        for (MessageEvent e : pendingWrites) {
                            e.getFuture().setSuccess();
                        }
                        return;
                    }
                    Throwable cause = future.getCause();
                    for (MessageEvent e2 : pendingWrites) {
                        e2.getFuture().setFailure(cause);
                    }
                }
            });
            Channels.write(this.ctx, future, (Object) composite);
            return null;
        }
    }

    public void writeRequested(ChannelHandlerContext ctx2, MessageEvent e) throws Exception {
        if (this.ctx == null) {
            this.ctx = ctx2;
        }
        getQueue().add(e);
    }

    public void disconnectRequested(ChannelHandlerContext ctx2, ChannelStateEvent e) throws Exception {
        try {
            flush(this.consolidateOnFlush);
        } finally {
            ctx2.sendDownstream(e);
        }
    }

    public void closeRequested(ChannelHandlerContext ctx2, ChannelStateEvent e) throws Exception {
        try {
            flush(this.consolidateOnFlush);
        } finally {
            ctx2.sendDownstream(e);
        }
    }

    public void channelClosed(ChannelHandlerContext ctx2, ChannelStateEvent e) throws Exception {
        Throwable cause = null;
        while (true) {
            MessageEvent ev = this.queue.poll();
            if (ev == null) {
                break;
            }
            if (cause == null) {
                cause = new ClosedChannelException();
            }
            ev.getFuture().setFailure(cause);
        }
        if (cause != null) {
            Channels.fireExceptionCaught(ctx2.getChannel(), cause);
        }
        super.channelClosed(ctx2, e);
    }

    public void beforeAdd(ChannelHandlerContext ctx2) throws Exception {
    }

    public void afterAdd(ChannelHandlerContext ctx2) throws Exception {
    }

    public void beforeRemove(ChannelHandlerContext ctx2) throws Exception {
        flush(this.consolidateOnFlush);
    }

    public void afterRemove(ChannelHandlerContext ctx2) throws Exception {
        Throwable cause = null;
        while (true) {
            MessageEvent ev = this.queue.poll();
            if (ev == null) {
                break;
            }
            if (cause == null) {
                cause = new IOException("Unable to flush message");
            }
            ev.getFuture().setFailure(cause);
        }
        if (cause != null) {
            Channels.fireExceptionCaughtLater(ctx2.getChannel(), cause);
        }
    }
}
