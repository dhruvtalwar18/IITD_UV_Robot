package org.ros.internal.node.service;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.handler.codec.oneone.OneToOneEncoder;
import org.ros.internal.message.MessageBuffers;

public final class ServiceResponseEncoder extends OneToOneEncoder {
    /* access modifiers changed from: protected */
    public Object encode(ChannelHandlerContext ctx, Channel channel, Object msg) throws Exception {
        if (!(msg instanceof ServiceServerResponse)) {
            return msg;
        }
        ServiceServerResponse response = (ServiceServerResponse) msg;
        ChannelBuffer buffer = MessageBuffers.dynamicBuffer();
        buffer.writeByte(response.getErrorCode());
        buffer.writeInt(response.getMessageLength());
        buffer.writeBytes(response.getMessage());
        return buffer;
    }
}
