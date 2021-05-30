package org.jboss.netty.handler.timeout;

import java.util.concurrent.TimeUnit;
import org.jboss.netty.channel.ChannelHandler;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.ChannelStateEvent;
import org.jboss.netty.channel.Channels;
import org.jboss.netty.channel.LifeCycleAwareChannelHandler;
import org.jboss.netty.channel.MessageEvent;
import org.jboss.netty.channel.SimpleChannelUpstreamHandler;
import org.jboss.netty.channel.WriteCompletionEvent;
import org.jboss.netty.util.ExternalResourceReleasable;
import org.jboss.netty.util.Timeout;
import org.jboss.netty.util.Timer;
import org.jboss.netty.util.TimerTask;

@ChannelHandler.Sharable
public class IdleStateHandler extends SimpleChannelUpstreamHandler implements LifeCycleAwareChannelHandler, ExternalResourceReleasable {
    final long allIdleTimeMillis;
    final long readerIdleTimeMillis;
    final Timer timer;
    final long writerIdleTimeMillis;

    public IdleStateHandler(Timer timer2, int readerIdleTimeSeconds, int writerIdleTimeSeconds, int allIdleTimeSeconds) {
        this(timer2, (long) readerIdleTimeSeconds, (long) writerIdleTimeSeconds, (long) allIdleTimeSeconds, TimeUnit.SECONDS);
    }

    public IdleStateHandler(Timer timer2, long readerIdleTime, long writerIdleTime, long allIdleTime, TimeUnit unit) {
        if (timer2 == null) {
            throw new NullPointerException("timer");
        } else if (unit != null) {
            this.timer = timer2;
            if (readerIdleTime <= 0) {
                this.readerIdleTimeMillis = 0;
            } else {
                this.readerIdleTimeMillis = Math.max(unit.toMillis(readerIdleTime), 1);
            }
            if (writerIdleTime <= 0) {
                this.writerIdleTimeMillis = 0;
            } else {
                this.writerIdleTimeMillis = Math.max(unit.toMillis(writerIdleTime), 1);
            }
            if (allIdleTime <= 0) {
                this.allIdleTimeMillis = 0;
            } else {
                this.allIdleTimeMillis = Math.max(unit.toMillis(allIdleTime), 1);
            }
        } else {
            throw new NullPointerException("unit");
        }
    }

    public long getReaderIdleTimeInMillis() {
        return this.readerIdleTimeMillis;
    }

    public long getWriterIdleTimeInMillis() {
        return this.writerIdleTimeMillis;
    }

    public long getAllIdleTimeInMillis() {
        return this.allIdleTimeMillis;
    }

    public void releaseExternalResources() {
        this.timer.stop();
    }

    public void beforeAdd(ChannelHandlerContext ctx) throws Exception {
        if (ctx.getPipeline().isAttached()) {
            initialize(ctx);
        }
    }

    public void afterAdd(ChannelHandlerContext ctx) throws Exception {
    }

    public void beforeRemove(ChannelHandlerContext ctx) throws Exception {
        destroy(ctx);
    }

    public void afterRemove(ChannelHandlerContext ctx) throws Exception {
    }

    public void channelOpen(ChannelHandlerContext ctx, ChannelStateEvent e) throws Exception {
        initialize(ctx);
        ctx.sendUpstream(e);
    }

    public void channelClosed(ChannelHandlerContext ctx, ChannelStateEvent e) throws Exception {
        destroy(ctx);
        ctx.sendUpstream(e);
    }

    public void messageReceived(ChannelHandlerContext ctx, MessageEvent e) throws Exception {
        ((State) ctx.getAttachment()).lastReadTime = System.currentTimeMillis();
        ctx.sendUpstream(e);
    }

    public void writeComplete(ChannelHandlerContext ctx, WriteCompletionEvent e) throws Exception {
        if (e.getWrittenAmount() > 0) {
            ((State) ctx.getAttachment()).lastWriteTime = System.currentTimeMillis();
        }
        ctx.sendUpstream(e);
    }

    private void initialize(ChannelHandlerContext ctx) {
        State state = state(ctx);
        if (!state.destroyed) {
            long currentTimeMillis = System.currentTimeMillis();
            state.lastWriteTime = currentTimeMillis;
            state.lastReadTime = currentTimeMillis;
            if (this.readerIdleTimeMillis > 0) {
                state.readerIdleTimeout = this.timer.newTimeout(new ReaderIdleTimeoutTask(ctx), this.readerIdleTimeMillis, TimeUnit.MILLISECONDS);
            }
            if (this.writerIdleTimeMillis > 0) {
                state.writerIdleTimeout = this.timer.newTimeout(new WriterIdleTimeoutTask(ctx), this.writerIdleTimeMillis, TimeUnit.MILLISECONDS);
            }
            if (this.allIdleTimeMillis > 0) {
                state.allIdleTimeout = this.timer.newTimeout(new AllIdleTimeoutTask(ctx), this.allIdleTimeMillis, TimeUnit.MILLISECONDS);
            }
        }
    }

    /* JADX WARNING: Code restructure failed: missing block: B:10:0x000e, code lost:
        r1.readerIdleTimeout.cancel();
        r1.readerIdleTimeout = null;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:12:0x0017, code lost:
        if (r1.writerIdleTimeout == null) goto L_0x0020;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:13:0x0019, code lost:
        r1.writerIdleTimeout.cancel();
        r1.writerIdleTimeout = null;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:15:0x0022, code lost:
        if (r1.allIdleTimeout == null) goto L_?;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:16:0x0024, code lost:
        r1.allIdleTimeout.cancel();
        r1.allIdleTimeout = null;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:23:?, code lost:
        return;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:24:?, code lost:
        return;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:9:0x000c, code lost:
        if (r1.readerIdleTimeout == null) goto L_0x0015;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private static void destroy(org.jboss.netty.channel.ChannelHandlerContext r4) {
        /*
            monitor-enter(r4)
            r0 = 0
            org.jboss.netty.handler.timeout.IdleStateHandler$State r1 = state(r4)     // Catch:{ all -> 0x002e }
            r2 = 1
            r1.destroyed = r2     // Catch:{ all -> 0x002c }
            monitor-exit(r4)     // Catch:{ all -> 0x002c }
            org.jboss.netty.util.Timeout r2 = r1.readerIdleTimeout
            if (r2 == 0) goto L_0x0015
            org.jboss.netty.util.Timeout r2 = r1.readerIdleTimeout
            r2.cancel()
            r1.readerIdleTimeout = r0
        L_0x0015:
            org.jboss.netty.util.Timeout r2 = r1.writerIdleTimeout
            if (r2 == 0) goto L_0x0020
            org.jboss.netty.util.Timeout r2 = r1.writerIdleTimeout
            r2.cancel()
            r1.writerIdleTimeout = r0
        L_0x0020:
            org.jboss.netty.util.Timeout r2 = r1.allIdleTimeout
            if (r2 == 0) goto L_0x002b
            org.jboss.netty.util.Timeout r2 = r1.allIdleTimeout
            r2.cancel()
            r1.allIdleTimeout = r0
        L_0x002b:
            return
        L_0x002c:
            r0 = move-exception
            goto L_0x0032
        L_0x002e:
            r1 = move-exception
            r3 = r1
            r1 = r0
            r0 = r3
        L_0x0032:
            monitor-exit(r4)     // Catch:{ all -> 0x002c }
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.handler.timeout.IdleStateHandler.destroy(org.jboss.netty.channel.ChannelHandlerContext):void");
    }

    private static State state(ChannelHandlerContext ctx) {
        synchronized (ctx) {
            State state = (State) ctx.getAttachment();
            if (state != null) {
                return state;
            }
            State state2 = new State();
            ctx.setAttachment(state2);
            return state2;
        }
    }

    /* access modifiers changed from: protected */
    public void channelIdle(ChannelHandlerContext ctx, IdleState state, long lastActivityTimeMillis) throws Exception {
        ctx.sendUpstream(new DefaultIdleStateEvent(ctx.getChannel(), state, lastActivityTimeMillis));
    }

    private final class ReaderIdleTimeoutTask implements TimerTask {
        private final ChannelHandlerContext ctx;

        ReaderIdleTimeoutTask(ChannelHandlerContext ctx2) {
            this.ctx = ctx2;
        }

        public void run(Timeout timeout) throws Exception {
            if (!timeout.isCancelled() && this.ctx.getChannel().isOpen()) {
                State state = (State) this.ctx.getAttachment();
                long currentTime = System.currentTimeMillis();
                long lastReadTime = state.lastReadTime;
                long nextDelay = IdleStateHandler.this.readerIdleTimeMillis - (currentTime - lastReadTime);
                if (nextDelay <= 0) {
                    state.readerIdleTimeout = IdleStateHandler.this.timer.newTimeout(this, IdleStateHandler.this.readerIdleTimeMillis, TimeUnit.MILLISECONDS);
                    try {
                        IdleStateHandler.this.channelIdle(this.ctx, IdleState.READER_IDLE, lastReadTime);
                    } catch (Throwable t) {
                        Channels.fireExceptionCaught(this.ctx, t);
                    }
                } else {
                    state.readerIdleTimeout = IdleStateHandler.this.timer.newTimeout(this, nextDelay, TimeUnit.MILLISECONDS);
                }
            }
        }
    }

    private final class WriterIdleTimeoutTask implements TimerTask {
        private final ChannelHandlerContext ctx;

        WriterIdleTimeoutTask(ChannelHandlerContext ctx2) {
            this.ctx = ctx2;
        }

        public void run(Timeout timeout) throws Exception {
            if (!timeout.isCancelled() && this.ctx.getChannel().isOpen()) {
                State state = (State) this.ctx.getAttachment();
                long currentTime = System.currentTimeMillis();
                long lastWriteTime = state.lastWriteTime;
                long nextDelay = IdleStateHandler.this.writerIdleTimeMillis - (currentTime - lastWriteTime);
                if (nextDelay <= 0) {
                    state.writerIdleTimeout = IdleStateHandler.this.timer.newTimeout(this, IdleStateHandler.this.writerIdleTimeMillis, TimeUnit.MILLISECONDS);
                    try {
                        IdleStateHandler.this.channelIdle(this.ctx, IdleState.WRITER_IDLE, lastWriteTime);
                    } catch (Throwable t) {
                        Channels.fireExceptionCaught(this.ctx, t);
                    }
                } else {
                    state.writerIdleTimeout = IdleStateHandler.this.timer.newTimeout(this, nextDelay, TimeUnit.MILLISECONDS);
                }
            }
        }
    }

    private final class AllIdleTimeoutTask implements TimerTask {
        private final ChannelHandlerContext ctx;

        AllIdleTimeoutTask(ChannelHandlerContext ctx2) {
            this.ctx = ctx2;
        }

        public void run(Timeout timeout) throws Exception {
            if (!timeout.isCancelled() && this.ctx.getChannel().isOpen()) {
                State state = (State) this.ctx.getAttachment();
                long currentTime = System.currentTimeMillis();
                long lastIoTime = Math.max(state.lastReadTime, state.lastWriteTime);
                long nextDelay = IdleStateHandler.this.allIdleTimeMillis - (currentTime - lastIoTime);
                if (nextDelay <= 0) {
                    state.allIdleTimeout = IdleStateHandler.this.timer.newTimeout(this, IdleStateHandler.this.allIdleTimeMillis, TimeUnit.MILLISECONDS);
                    try {
                        IdleStateHandler.this.channelIdle(this.ctx, IdleState.ALL_IDLE, lastIoTime);
                    } catch (Throwable t) {
                        Channels.fireExceptionCaught(this.ctx, t);
                    }
                } else {
                    state.allIdleTimeout = IdleStateHandler.this.timer.newTimeout(this, nextDelay, TimeUnit.MILLISECONDS);
                }
            }
        }
    }

    private static final class State {
        volatile Timeout allIdleTimeout;
        volatile boolean destroyed;
        volatile long lastReadTime;
        volatile long lastWriteTime;
        volatile Timeout readerIdleTimeout;
        volatile Timeout writerIdleTimeout;

        State() {
        }
    }
}
