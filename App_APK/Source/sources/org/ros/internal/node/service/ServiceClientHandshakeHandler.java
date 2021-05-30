package org.ros.internal.node.service;

import java.util.Queue;
import java.util.concurrent.ExecutorService;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.jboss.netty.channel.ChannelHandler;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.MessageEvent;
import org.ros.internal.transport.BaseClientHandshakeHandler;
import org.ros.internal.transport.ConnectionHeader;
import org.ros.message.MessageDeserializer;
import org.ros.node.service.ServiceResponseListener;

class ServiceClientHandshakeHandler<T, S> extends BaseClientHandshakeHandler {
    private static final Log log = LogFactory.getLog(ServiceClientHandshakeHandler.class);
    private final MessageDeserializer<S> deserializer;
    private final ExecutorService executorService;
    private final Queue<ServiceResponseListener<S>> responseListeners;

    public ServiceClientHandshakeHandler(ConnectionHeader outgoingConnectionHeader, Queue<ServiceResponseListener<S>> responseListeners2, MessageDeserializer<S> deserializer2, ExecutorService executorService2) {
        super(new ServiceClientHandshake(outgoingConnectionHeader), executorService2);
        this.responseListeners = responseListeners2;
        this.deserializer = deserializer2;
        this.executorService = executorService2;
    }

    /* access modifiers changed from: protected */
    public void onSuccess(ConnectionHeader incommingConnectionHeader, ChannelHandlerContext ctx, MessageEvent e) {
        ChannelPipeline pipeline = e.getChannel().getPipeline();
        pipeline.remove("LengthFieldBasedFrameDecoder");
        pipeline.remove((ChannelHandler) this);
        pipeline.addLast("ResponseDecoder", new ServiceResponseDecoder());
        pipeline.addLast("ResponseHandler", new ServiceResponseHandler(this.responseListeners, this.deserializer, this.executorService));
    }

    /* access modifiers changed from: protected */
    public void onFailure(String errorMessage, ChannelHandlerContext ctx, MessageEvent e) {
        Log log2 = log;
        log2.error("Service client handshake failed: " + errorMessage);
        e.getChannel().close();
    }

    public String getName() {
        return "ServiceClientHandshakeHandler";
    }
}
