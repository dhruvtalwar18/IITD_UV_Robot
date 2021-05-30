package org.jboss.netty.handler.timeout;

import java.util.concurrent.TimeUnit;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.ChannelFutureListener;
import org.jboss.netty.channel.ChannelHandler;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.Channels;
import org.jboss.netty.channel.MessageEvent;
import org.jboss.netty.channel.SimpleChannelDownstreamHandler;
import org.jboss.netty.util.ExternalResourceReleasable;
import org.jboss.netty.util.Timeout;
import org.jboss.netty.util.Timer;
import org.jboss.netty.util.TimerTask;

@ChannelHandler.Sharable
public class WriteTimeoutHandler extends SimpleChannelDownstreamHandler implements ExternalResourceReleasable {
    static final WriteTimeoutException EXCEPTION = new WriteTimeoutException();
    private final long timeoutMillis;
    private final Timer timer;

    public WriteTimeoutHandler(Timer timer2, int timeoutSeconds) {
        this(timer2, (long) timeoutSeconds, TimeUnit.SECONDS);
    }

    public WriteTimeoutHandler(Timer timer2, long timeout, TimeUnit unit) {
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

    /* access modifiers changed from: protected */
    public long getTimeoutMillis(MessageEvent e) {
        return this.timeoutMillis;
    }

    public void writeRequested(ChannelHandlerContext ctx, MessageEvent e) throws Exception {
        long timeoutMillis2 = getTimeoutMillis(e);
        if (timeoutMillis2 > 0) {
            ChannelFuture future = e.getFuture();
            future.addListener(new TimeoutCanceller(this.timer.newTimeout(new WriteTimeoutTask(ctx, future), timeoutMillis2, TimeUnit.MILLISECONDS)));
        }
        super.writeRequested(ctx, e);
    }

    /* access modifiers changed from: protected */
    public void writeTimedOut(ChannelHandlerContext ctx) throws Exception {
        Channels.fireExceptionCaughtLater(ctx, (Throwable) EXCEPTION);
    }

    private final class WriteTimeoutTask implements TimerTask {
        private final ChannelHandlerContext ctx;
        private final ChannelFuture future;

        WriteTimeoutTask(ChannelHandlerContext ctx2, ChannelFuture future2) {
            this.ctx = ctx2;
            this.future = future2;
        }

        public void run(Timeout timeout) throws Exception {
            if (!timeout.isCancelled() && this.ctx.getChannel().isOpen() && this.future.setFailure(WriteTimeoutHandler.EXCEPTION)) {
                try {
                    WriteTimeoutHandler.this.writeTimedOut(this.ctx);
                } catch (Throwable t) {
                    Channels.fireExceptionCaught(this.ctx, t);
                }
            }
        }
    }

    private static final class TimeoutCanceller implements ChannelFutureListener {
        private final Timeout timeout;

        TimeoutCanceller(Timeout timeout2) {
            this.timeout = timeout2;
        }

        public void operationComplete(ChannelFuture future) throws Exception {
            this.timeout.cancel();
        }
    }
}
