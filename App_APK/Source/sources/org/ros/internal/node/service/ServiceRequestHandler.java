package org.ros.internal.node.service;

import java.nio.ByteBuffer;
import java.nio.charset.Charset;
import java.util.concurrent.ExecutorService;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.MessageEvent;
import org.jboss.netty.channel.SimpleChannelHandler;
import org.ros.exception.ServiceException;
import org.ros.internal.message.MessageBufferPool;
import org.ros.message.MessageDeserializer;
import org.ros.message.MessageFactory;
import org.ros.message.MessageSerializer;
import org.ros.node.service.ServiceResponseBuilder;

class ServiceRequestHandler<T, S> extends SimpleChannelHandler {
    private final MessageDeserializer<T> deserializer;
    private final ExecutorService executorService;
    /* access modifiers changed from: private */
    public final MessageBufferPool messageBufferPool = new MessageBufferPool();
    private final MessageFactory messageFactory;
    private final ServiceResponseBuilder<T, S> responseBuilder;
    private final MessageSerializer<S> serializer;
    private final ServiceDeclaration serviceDeclaration;

    public ServiceRequestHandler(ServiceDeclaration serviceDeclaration2, ServiceResponseBuilder<T, S> responseBuilder2, MessageDeserializer<T> deserializer2, MessageSerializer<S> serializer2, MessageFactory messageFactory2, ExecutorService executorService2) {
        this.serviceDeclaration = serviceDeclaration2;
        this.deserializer = deserializer2;
        this.serializer = serializer2;
        this.responseBuilder = responseBuilder2;
        this.messageFactory = messageFactory2;
        this.executorService = executorService2;
    }

    /* access modifiers changed from: private */
    public void handleRequest(ChannelBuffer requestBuffer, ChannelBuffer responseBuffer) throws ServiceException {
        T request = this.deserializer.deserialize(requestBuffer);
        S response = this.messageFactory.newFromType(this.serviceDeclaration.getType());
        this.responseBuilder.build(request, response);
        this.serializer.serialize(response, responseBuffer);
    }

    /* access modifiers changed from: private */
    public void handleSuccess(ChannelHandlerContext ctx, ServiceServerResponse response, ChannelBuffer responseBuffer) {
        response.setErrorCode(1);
        response.setMessageLength(responseBuffer.readableBytes());
        response.setMessage(responseBuffer);
        ctx.getChannel().write(response);
    }

    /* access modifiers changed from: private */
    public void handleError(ChannelHandlerContext ctx, ServiceServerResponse response, String message) {
        response.setErrorCode(0);
        ByteBuffer encodedMessage = Charset.forName("US-ASCII").encode(message);
        response.setMessageLength(encodedMessage.limit());
        response.setMessage(ChannelBuffers.wrappedBuffer(encodedMessage));
        ctx.getChannel().write(response);
    }

    public void messageReceived(final ChannelHandlerContext ctx, MessageEvent e) throws Exception {
        final ChannelBuffer requestBuffer = ((ChannelBuffer) e.getMessage()).copy();
        this.executorService.execute(new Runnable() {
            public void run() {
                boolean success;
                ServiceServerResponse response = new ServiceServerResponse();
                ChannelBuffer responseBuffer = ServiceRequestHandler.this.messageBufferPool.acquire();
                try {
                    ServiceRequestHandler.this.handleRequest(requestBuffer, responseBuffer);
                    success = true;
                } catch (ServiceException ex) {
                    ServiceRequestHandler.this.handleError(ctx, response, ex.getMessage());
                    success = false;
                }
                if (success) {
                    ServiceRequestHandler.this.handleSuccess(ctx, response, responseBuffer);
                }
                ServiceRequestHandler.this.messageBufferPool.release(responseBuffer);
            }
        });
        super.messageReceived(ctx, e);
    }
}
