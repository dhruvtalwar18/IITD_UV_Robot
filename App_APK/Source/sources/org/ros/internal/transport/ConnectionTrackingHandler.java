package org.ros.internal.transport;

import java.io.IOException;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.ChannelStateEvent;
import org.jboss.netty.channel.ExceptionEvent;
import org.jboss.netty.channel.SimpleChannelHandler;
import org.jboss.netty.channel.group.ChannelGroup;
import org.ros.exception.RosRuntimeException;

public class ConnectionTrackingHandler extends SimpleChannelHandler {
    private static final boolean DEBUG = false;
    private static final Log log = LogFactory.getLog(ConnectionTrackingHandler.class);
    private final ChannelGroup channelGroup;

    public ConnectionTrackingHandler(ChannelGroup channelGroup2) {
        this.channelGroup = channelGroup2;
    }

    public void channelOpen(ChannelHandlerContext ctx, ChannelStateEvent e) throws Exception {
        this.channelGroup.add(e.getChannel());
        super.channelOpen(ctx, e);
    }

    public void channelClosed(ChannelHandlerContext ctx, ChannelStateEvent e) throws Exception {
        super.channelClosed(ctx, e);
    }

    public void exceptionCaught(ChannelHandlerContext ctx, ExceptionEvent e) throws Exception {
        ctx.getChannel().close();
        if (e.getCause() instanceof IOException) {
            Log log2 = log;
            log2.error("Channel exception: " + e.getCause());
            return;
        }
        throw new RosRuntimeException(e.getCause());
    }
}
