package org.jboss.netty.handler.stream;

import java.io.IOException;
import java.nio.channels.ClosedChannelException;
import java.util.Queue;
import java.util.concurrent.atomic.AtomicBoolean;
import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelDownstreamHandler;
import org.jboss.netty.channel.ChannelEvent;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.ChannelFutureListener;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.ChannelStateEvent;
import org.jboss.netty.channel.ChannelUpstreamHandler;
import org.jboss.netty.channel.Channels;
import org.jboss.netty.channel.LifeCycleAwareChannelHandler;
import org.jboss.netty.channel.MessageEvent;
import org.jboss.netty.logging.InternalLogger;
import org.jboss.netty.logging.InternalLoggerFactory;
import org.jboss.netty.util.internal.QueueFactory;

public class ChunkedWriteHandler implements ChannelUpstreamHandler, ChannelDownstreamHandler, LifeCycleAwareChannelHandler {
    static final /* synthetic */ boolean $assertionsDisabled = false;
    private static final InternalLogger logger = InternalLoggerFactory.getInstance((Class<?>) ChunkedWriteHandler.class);
    private volatile ChannelHandlerContext ctx;
    private MessageEvent currentEvent;
    private final AtomicBoolean flush = new AtomicBoolean(false);
    private final Queue<MessageEvent> queue = QueueFactory.createQueue(MessageEvent.class);

    public void resumeTransfer() {
        ChannelHandlerContext ctx2 = this.ctx;
        if (ctx2 != null) {
            try {
                flush(ctx2, false);
            } catch (Exception e) {
                if (logger.isWarnEnabled()) {
                    logger.warn("Unexpected exception while sending chunks.", e);
                }
            }
        }
    }

    public void handleDownstream(ChannelHandlerContext ctx2, ChannelEvent e) throws Exception {
        if (!(e instanceof MessageEvent)) {
            ctx2.sendDownstream(e);
            return;
        }
        boolean offer = this.queue.offer((MessageEvent) e);
        Channel channel = ctx2.getChannel();
        if (channel.isWritable() || !channel.isConnected()) {
            this.ctx = ctx2;
            flush(ctx2, false);
        }
    }

    public void handleUpstream(ChannelHandlerContext ctx2, ChannelEvent e) throws Exception {
        ChannelStateEvent cse;
        if (e instanceof ChannelStateEvent) {
            switch (((ChannelStateEvent) e).getState()) {
                case INTEREST_OPS:
                    flush(ctx2, true);
                    break;
                case OPEN:
                    if (!Boolean.TRUE.equals(cse.getValue())) {
                        flush(ctx2, true);
                        break;
                    }
                    break;
            }
        }
        ctx2.sendUpstream(e);
    }

    private void discard(ChannelHandlerContext ctx2, boolean fireNow) {
        ClosedChannelException cause = null;
        while (true) {
            MessageEvent currentEvent2 = this.currentEvent;
            if (this.currentEvent == null) {
                currentEvent2 = this.queue.poll();
            } else {
                this.currentEvent = null;
            }
            if (currentEvent2 == null) {
                break;
            }
            Object m = currentEvent2.getMessage();
            if (m instanceof ChunkedInput) {
                closeInput((ChunkedInput) m);
            }
            if (cause == null) {
                cause = new ClosedChannelException();
            }
            currentEvent2.getFuture().setFailure(cause);
        }
        if (cause == null) {
            return;
        }
        if (fireNow) {
            Channels.fireExceptionCaught(ctx2.getChannel(), (Throwable) cause);
        } else {
            Channels.fireExceptionCaughtLater(ctx2.getChannel(), (Throwable) cause);
        }
    }

    private void flush(ChannelHandlerContext ctx2, boolean fireNow) throws Exception {
        ChunkedWriteHandler chunkedWriteHandler;
        final MessageEvent currentEvent2;
        final ChunkedInput chunks;
        boolean suspend;
        ChannelFuture writeFuture;
        Channel channel = ctx2.getChannel();
        boolean compareAndSet = this.flush.compareAndSet(false, true);
        boolean acquired = compareAndSet;
        if (compareAndSet) {
            try {
                if (!channel.isConnected()) {
                    discard(ctx2, fireNow);
                    this.flush.set(false);
                    return;
                }
                do {
                    if (channel.isWritable()) {
                        if (this.currentEvent == null) {
                            this.currentEvent = this.queue.poll();
                        }
                        if (this.currentEvent != null) {
                            if (this.currentEvent.getFuture().isDone()) {
                                this.currentEvent = null;
                            } else {
                                currentEvent2 = this.currentEvent;
                                Object m = currentEvent2.getMessage();
                                if (m instanceof ChunkedInput) {
                                    chunks = (ChunkedInput) m;
                                    Object chunk = chunks.nextChunk();
                                    boolean endOfInput = chunks.isEndOfInput();
                                    if (chunk == null) {
                                        chunk = ChannelBuffers.EMPTY_BUFFER;
                                        suspend = !endOfInput;
                                    } else {
                                        suspend = false;
                                    }
                                    if (!suspend) {
                                        if (endOfInput) {
                                            this.currentEvent = null;
                                            writeFuture = currentEvent2.getFuture();
                                            writeFuture.addListener(new ChannelFutureListener() {
                                                public void operationComplete(ChannelFuture future) throws Exception {
                                                    ChunkedWriteHandler.closeInput(chunks);
                                                }
                                            });
                                        } else {
                                            writeFuture = Channels.future(channel);
                                            writeFuture.addListener(new ChannelFutureListener() {
                                                public void operationComplete(ChannelFuture future) throws Exception {
                                                    if (!future.isSuccess()) {
                                                        currentEvent2.getFuture().setFailure(future.getCause());
                                                        ChunkedWriteHandler.closeInput((ChunkedInput) currentEvent2.getMessage());
                                                    }
                                                }
                                            });
                                        }
                                        Channels.write(ctx2, writeFuture, chunk, currentEvent2.getRemoteAddress());
                                    }
                                } else {
                                    this.currentEvent = null;
                                    ctx2.sendDownstream(currentEvent2);
                                }
                            }
                        }
                    }
                    chunkedWriteHandler = this;
                    chunkedWriteHandler.flush.set(false);
                } while (channel.isConnected());
                discard(ctx2, fireNow);
                this.flush.set(false);
                return;
            } catch (Throwable th) {
                this.flush.set(false);
                throw th;
            }
        } else {
            chunkedWriteHandler = this;
        }
        if (!acquired) {
            return;
        }
        if (!channel.isConnected() || (channel.isWritable() && !chunkedWriteHandler.queue.isEmpty())) {
            chunkedWriteHandler.flush(ctx2, fireNow);
        }
    }

    static void closeInput(ChunkedInput chunks) {
        try {
            chunks.close();
        } catch (Throwable t) {
            if (logger.isWarnEnabled()) {
                logger.warn("Failed to close a chunked input.", t);
            }
        }
    }

    public void beforeAdd(ChannelHandlerContext ctx2) throws Exception {
    }

    public void afterAdd(ChannelHandlerContext ctx2) throws Exception {
    }

    public void beforeRemove(ChannelHandlerContext ctx2) throws Exception {
        flush(ctx2, false);
    }

    public void afterRemove(ChannelHandlerContext ctx2) throws Exception {
        Throwable cause = null;
        boolean fireExceptionCaught = false;
        while (true) {
            MessageEvent currentEvent2 = this.currentEvent;
            if (this.currentEvent == null) {
                currentEvent2 = this.queue.poll();
            } else {
                this.currentEvent = null;
            }
            if (currentEvent2 == null) {
                break;
            }
            Object m = currentEvent2.getMessage();
            if (m instanceof ChunkedInput) {
                closeInput((ChunkedInput) m);
            }
            if (cause == null) {
                cause = new IOException("Unable to flush event, discarding");
            }
            currentEvent2.getFuture().setFailure(cause);
            fireExceptionCaught = true;
        }
        if (fireExceptionCaught) {
            Channels.fireExceptionCaughtLater(ctx2.getChannel(), cause);
        }
    }
}
