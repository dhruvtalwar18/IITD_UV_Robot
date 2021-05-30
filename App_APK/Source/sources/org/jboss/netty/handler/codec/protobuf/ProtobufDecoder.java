package org.jboss.netty.handler.codec.protobuf;

import com.google.protobuf.ExtensionRegistry;
import com.google.protobuf.MessageLite;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBufferInputStream;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelHandler;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.handler.codec.oneone.OneToOneDecoder;

@ChannelHandler.Sharable
public class ProtobufDecoder extends OneToOneDecoder {
    private final ExtensionRegistry extensionRegistry;
    private final MessageLite prototype;

    public ProtobufDecoder(MessageLite prototype2) {
        this(prototype2, (ExtensionRegistry) null);
    }

    public ProtobufDecoder(MessageLite prototype2, ExtensionRegistry extensionRegistry2) {
        if (prototype2 != null) {
            this.prototype = prototype2.getDefaultInstanceForType();
            this.extensionRegistry = extensionRegistry2;
            return;
        }
        throw new NullPointerException("prototype");
    }

    /* access modifiers changed from: protected */
    public Object decode(ChannelHandlerContext ctx, Channel channel, Object msg) throws Exception {
        if (!(msg instanceof ChannelBuffer)) {
            return msg;
        }
        ChannelBuffer buf = (ChannelBuffer) msg;
        if (buf.hasArray()) {
            int offset = buf.readerIndex();
            if (this.extensionRegistry == null) {
                return this.prototype.newBuilderForType().mergeFrom(buf.array(), buf.arrayOffset() + offset, buf.readableBytes()).build();
            }
            return this.prototype.newBuilderForType().mergeFrom(buf.array(), buf.arrayOffset() + offset, buf.readableBytes(), this.extensionRegistry).build();
        } else if (this.extensionRegistry == null) {
            return this.prototype.newBuilderForType().mergeFrom(new ChannelBufferInputStream((ChannelBuffer) msg)).build();
        } else {
            return this.prototype.newBuilderForType().mergeFrom(new ChannelBufferInputStream((ChannelBuffer) msg), this.extensionRegistry).build();
        }
    }
}
