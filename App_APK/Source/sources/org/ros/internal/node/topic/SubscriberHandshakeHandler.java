package org.ros.internal.node.topic;

import java.util.concurrent.ExecutorService;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.jboss.netty.channel.ChannelHandler;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.MessageEvent;
import org.ros.internal.transport.BaseClientHandshakeHandler;
import org.ros.internal.transport.ConnectionHeader;
import org.ros.internal.transport.ConnectionHeaderFields;
import org.ros.internal.transport.queue.IncomingMessageQueue;
import org.ros.internal.transport.tcp.NamedChannelHandler;

class SubscriberHandshakeHandler<T> extends BaseClientHandshakeHandler {
    private static final Log log = LogFactory.getLog(SubscriberHandshakeHandler.class);
    private final IncomingMessageQueue<T> incomingMessageQueue;

    public SubscriberHandshakeHandler(ConnectionHeader outgoingConnectionHeader, IncomingMessageQueue<T> incomingMessageQueue2, ExecutorService executorService) {
        super(new SubscriberHandshake(outgoingConnectionHeader), executorService);
        this.incomingMessageQueue = incomingMessageQueue2;
    }

    /* access modifiers changed from: protected */
    public void onSuccess(ConnectionHeader incomingConnectionHeader, ChannelHandlerContext ctx, MessageEvent e) {
        ChannelPipeline pipeline = e.getChannel().getPipeline();
        pipeline.remove((ChannelHandler) this);
        NamedChannelHandler namedChannelHandler = this.incomingMessageQueue.getMessageReceiver();
        pipeline.addLast(namedChannelHandler.getName(), namedChannelHandler);
        String latching = incomingConnectionHeader.getField(ConnectionHeaderFields.LATCHING);
        if (latching != null && latching.equals("1")) {
            this.incomingMessageQueue.setLatchMode(true);
        }
    }

    /* access modifiers changed from: protected */
    public void onFailure(String errorMessage, ChannelHandlerContext ctx, MessageEvent e) {
        Log log2 = log;
        log2.error("Subscriber handshake failed: " + errorMessage);
        e.getChannel().close();
    }

    public String getName() {
        return "SubscriberHandshakeHandler";
    }
}
