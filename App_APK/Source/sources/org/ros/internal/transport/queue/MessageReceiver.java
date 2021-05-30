package org.ros.internal.transport.queue;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.MessageEvent;
import org.ros.concurrent.CircularBlockingDeque;
import org.ros.internal.transport.tcp.AbstractNamedChannelHandler;
import org.ros.message.MessageDeserializer;

public class MessageReceiver<T> extends AbstractNamedChannelHandler {
    private static final boolean DEBUG = false;
    private static final Log log = LogFactory.getLog(MessageReceiver.class);
    private final MessageDeserializer<T> deserializer;
    private final CircularBlockingDeque<LazyMessage<T>> lazyMessages;

    public MessageReceiver(CircularBlockingDeque<LazyMessage<T>> lazyMessages2, MessageDeserializer<T> deserializer2) {
        this.lazyMessages = lazyMessages2;
        this.deserializer = deserializer2;
    }

    public String getName() {
        return "IncomingMessageQueueChannelHandler";
    }

    public void messageReceived(ChannelHandlerContext ctx, MessageEvent e) throws Exception {
        this.lazyMessages.addLast(new LazyMessage(((ChannelBuffer) e.getMessage()).copy(), this.deserializer));
        super.messageReceived(ctx, e);
    }
}
