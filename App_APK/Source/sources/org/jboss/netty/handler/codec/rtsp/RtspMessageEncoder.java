package org.jboss.netty.handler.codec.rtsp;

import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelHandler;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.handler.codec.http.HttpMessage;
import org.jboss.netty.handler.codec.http.HttpMessageEncoder;

@ChannelHandler.Sharable
public abstract class RtspMessageEncoder extends HttpMessageEncoder {
    protected RtspMessageEncoder() {
    }

    /* access modifiers changed from: protected */
    public Object encode(ChannelHandlerContext ctx, Channel channel, Object msg) throws Exception {
        if (!(msg instanceof HttpMessage)) {
            return msg;
        }
        return super.encode(ctx, channel, msg);
    }
}
