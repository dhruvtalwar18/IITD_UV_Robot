package org.jboss.netty.handler.timeout;

import java.util.concurrent.TimeUnit;
import org.jboss.netty.channel.ChannelHandler;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.ChannelStateEvent;
import org.jboss.netty.channel.Channels;
import org.jboss.netty.channel.LifeCycleAwareChannelHandler;
import org.jboss.netty.channel.MessageEvent;
import org.jboss.netty.channel.SimpleChannelUpstreamHandler;
import org.jboss.netty.util.ExternalResourceReleasable;
import org.jboss.netty.util.Timeout;
import org.jboss.netty.util.Timer;
import org.jboss.netty.util.TimerTask;

@ChannelHandler.Sharable
public class ReadTimeoutHandler extends SimpleChannelUpstreamHandler implements LifeCycleAwareChannelHandler, ExternalResourceReleasable {
    static final ReadTimeoutException EXCEPTION = new ReadTimeoutException();
    final long timeoutMillis;
    final Timer timer;

    public ReadTimeoutHandler(Timer timer2, int timeoutSeconds) {
        this(timer2, (long) timeoutSeconds, TimeUnit.SECONDS);
    }

    public ReadTimeoutHandler(Timer timer2, long timeout, TimeUnit unit) {
        if (timer2 == null) {
            throw new NullPointerException("timer");
        } else if (unit != null) {
            this.timer = timer2;
            if (timeout <= 0) {
                this.timeoutMillis = 0;
            } else {
                this.timeoutMillis = Math.max(unit.toMillis(timeout), 1);
            }
        } else {
            throw new NullPointerException("unit");
        }
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

    private void initialize(ChannelHandlerContext ctx) {
        State state = state(ctx);
        if (!state.destroyed && this.timeoutMillis > 0) {
            state.timeout = this.timer.newTimeout(new ReadTimeoutTask(ctx), this.timeoutMillis, TimeUnit.MILLISECONDS);
        }
    }

    /* JADX WARNING: Code restructure failed: missing block: B:10:0x000e, code lost:
        r1.timeout.cancel();
        r1.timeout = null;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:17:?, code lost:
        return;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:18:?, code lost:
        return;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:9:0x000c, code lost:
        if (r1.timeout == null) goto L_?;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private static void destroy(org.jboss.netty.channel.ChannelHandlerContext r4) {
        /*
            monitor-enter(r4)
            r0 = 0
            org.jboss.netty.handler.timeout.ReadTimeoutHandler$State r1 = state(r4)     // Catch:{ all -> 0x0018 }
            r2 = 1
            r1.destroyed = r2     // Catch:{ all -> 0x0016 }
            monitor-exit(r4)     // Catch:{ all -> 0x0016 }
            org.jboss.netty.util.Timeout r2 = r1.timeout
            if (r2 == 0) goto L_0x0015
            org.jboss.netty.util.Timeout r2 = r1.timeout
            r2.cancel()
            r1.timeout = r0
        L_0x0015:
            return
        L_0x0016:
            r0 = move-exception
            goto L_0x001c
        L_0x0018:
            r1 = move-exception
            r3 = r1
            r1 = r0
            r0 = r3
        L_0x001c:
            monitor-exit(r4)     // Catch:{ all -> 0x0016 }
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.handler.timeout.ReadTimeoutHandler.destroy(org.jboss.netty.channel.ChannelHandlerContext):void");
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
    public void readTimedOut(ChannelHandlerContext ctx) throws Exception {
        Channels.fireExceptionCaught(ctx, (Throwable) EXCEPTION);
    }

    private final class ReadTimeoutTask implements TimerTask {
        private final ChannelHandlerContext ctx;

        ReadTimeoutTask(ChannelHandlerContext ctx2) {
            this.ctx = ctx2;
        }

        public void run(Timeout timeout) throws Exception {
            if (!timeout.isCancelled() && this.ctx.getChannel().isOpen()) {
                State state = (State) this.ctx.getAttachment();
                long nextDelay = ReadTimeoutHandler.this.timeoutMillis - (System.currentTimeMillis() - state.lastReadTime);
                if (nextDelay <= 0) {
                    state.timeout = ReadTimeoutHandler.this.timer.newTimeout(this, ReadTimeoutHandler.this.timeoutMillis, TimeUnit.MILLISECONDS);
                    try {
                        ReadTimeoutHandler.this.readTimedOut(this.ctx);
                    } catch (Throwable t) {
                        Channels.fireExceptionCaught(this.ctx, t);
                    }
                } else {
                    state.timeout = ReadTimeoutHandler.this.timer.newTimeout(this, nextDelay, TimeUnit.MILLISECONDS);
                }
            }
        }
    }

    private static final class State {
        volatile boolean destroyed;
        volatile long lastReadTime = System.currentTimeMillis();
        volatile Timeout timeout;

        State() {
        }
    }
}
