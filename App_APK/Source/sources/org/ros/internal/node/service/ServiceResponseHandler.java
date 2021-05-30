package org.ros.internal.node.service;

import com.google.common.base.Preconditions;
import java.nio.charset.Charset;
import java.util.Queue;
import java.util.concurrent.ExecutorService;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.MessageEvent;
import org.jboss.netty.channel.SimpleChannelHandler;
import org.ros.exception.RemoteException;
import org.ros.internal.node.response.StatusCode;
import org.ros.message.MessageDeserializer;
import org.ros.node.service.ServiceResponseListener;

class ServiceResponseHandler<ResponseType> extends SimpleChannelHandler {
    /* access modifiers changed from: private */
    public final MessageDeserializer<ResponseType> deserializer;
    private final ExecutorService executorService;
    private final Queue<ServiceResponseListener<ResponseType>> responseListeners;

    public ServiceResponseHandler(Queue<ServiceResponseListener<ResponseType>> messageListeners, MessageDeserializer<ResponseType> deserializer2, ExecutorService executorService2) {
        this.responseListeners = messageListeners;
        this.deserializer = deserializer2;
        this.executorService = executorService2;
    }

    public void messageReceived(ChannelHandlerContext ctx, MessageEvent e) throws Exception {
        final ServiceResponseListener<ResponseType> listener = this.responseListeners.poll();
        Preconditions.checkNotNull(listener, "No listener for incoming service response.");
        final ServiceServerResponse response = (ServiceServerResponse) e.getMessage();
        final ChannelBuffer buffer = response.getMessage();
        this.executorService.execute(new Runnable() {
            public void run() {
                if (response.getErrorCode() == 1) {
                    listener.onSuccess(ServiceResponseHandler.this.deserializer.deserialize(buffer));
                    return;
                }
                listener.onFailure(new RemoteException(StatusCode.ERROR, Charset.forName("US-ASCII").decode(buffer.toByteBuffer()).toString()));
            }
        });
    }
}
