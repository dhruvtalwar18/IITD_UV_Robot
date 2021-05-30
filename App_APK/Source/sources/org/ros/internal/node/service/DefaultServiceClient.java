package org.ros.internal.node.service;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import java.net.InetSocketAddress;
import java.net.URI;
import java.util.Queue;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.exception.RosRuntimeException;
import org.ros.internal.message.MessageBufferPool;
import org.ros.internal.transport.ClientHandshakeListener;
import org.ros.internal.transport.ConnectionHeader;
import org.ros.internal.transport.ConnectionHeaderFields;
import org.ros.internal.transport.tcp.TcpClient;
import org.ros.internal.transport.tcp.TcpClientManager;
import org.ros.message.MessageDeserializer;
import org.ros.message.MessageFactory;
import org.ros.message.MessageSerializer;
import org.ros.namespace.GraphName;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

public class DefaultServiceClient<T, S> implements ServiceClient<T, S> {
    private final ConnectionHeader connectionHeader = new ConnectionHeader();
    private final DefaultServiceClient<T, S>.HandshakeLatch handshakeLatch;
    private final MessageBufferPool messageBufferPool = new MessageBufferPool();
    private final MessageFactory messageFactory;
    private final Queue<ServiceResponseListener<S>> responseListeners = Lists.newLinkedList();
    private final MessageSerializer<T> serializer;
    private final ServiceDeclaration serviceDeclaration;
    private TcpClient tcpClient;
    private final TcpClientManager tcpClientManager;

    private final class HandshakeLatch implements ClientHandshakeListener {
        private String errorMessage;
        private CountDownLatch latch;
        private boolean success;

        private HandshakeLatch() {
        }

        public void onSuccess(ConnectionHeader outgoingConnectionHeader, ConnectionHeader incomingConnectionHeader) {
            this.success = true;
            this.latch.countDown();
        }

        public void onFailure(ConnectionHeader outgoingConnectionHeader, String errorMessage2) {
            this.errorMessage = errorMessage2;
            this.success = false;
            this.latch.countDown();
        }

        public boolean await(long timeout, TimeUnit unit) throws InterruptedException {
            this.latch.await(timeout, unit);
            return this.success;
        }

        public String getErrorMessage() {
            return this.errorMessage;
        }

        public void reset() {
            this.latch = new CountDownLatch(1);
            this.success = false;
            this.errorMessage = null;
        }
    }

    public static <S, T> DefaultServiceClient<S, T> newDefault(GraphName nodeName, ServiceDeclaration serviceDeclaration2, MessageSerializer<S> serializer2, MessageDeserializer<T> deserializer, MessageFactory messageFactory2, ScheduledExecutorService executorService) {
        return new DefaultServiceClient(nodeName, serviceDeclaration2, serializer2, deserializer, messageFactory2, executorService);
    }

    private DefaultServiceClient(GraphName nodeName, ServiceDeclaration serviceDeclaration2, MessageSerializer<T> serializer2, MessageDeserializer<S> deserializer, MessageFactory messageFactory2, ScheduledExecutorService executorService) {
        this.serviceDeclaration = serviceDeclaration2;
        this.serializer = serializer2;
        this.messageFactory = messageFactory2;
        this.connectionHeader.addField(ConnectionHeaderFields.CALLER_ID, nodeName.toString());
        this.connectionHeader.addField(ConnectionHeaderFields.PERSISTENT, "1");
        this.connectionHeader.merge(serviceDeclaration2.toConnectionHeader());
        this.tcpClientManager = new TcpClientManager(executorService);
        ServiceClientHandshakeHandler<T, S> serviceClientHandshakeHandler = new ServiceClientHandshakeHandler<>(this.connectionHeader, this.responseListeners, deserializer, executorService);
        this.handshakeLatch = new HandshakeLatch();
        serviceClientHandshakeHandler.addListener(this.handshakeLatch);
        this.tcpClientManager.addNamedChannelHandler(serviceClientHandshakeHandler);
    }

    public void connect(URI uri) {
        Preconditions.checkNotNull(uri, "URI must be specified.");
        Preconditions.checkArgument(uri.getScheme().equals("rosrpc"), "Invalid service URI.");
        Preconditions.checkState(this.tcpClient == null, "Already connected.");
        InetSocketAddress address = new InetSocketAddress(uri.getHost(), uri.getPort());
        this.handshakeLatch.reset();
        this.tcpClient = this.tcpClientManager.connect(toString(), address);
        try {
            if (!this.handshakeLatch.await(1, TimeUnit.SECONDS)) {
                throw new RosRuntimeException(this.handshakeLatch.getErrorMessage());
            }
        } catch (InterruptedException e) {
            throw new RosRuntimeException("Handshake timed out.");
        }
    }

    public void shutdown() {
        this.tcpClientManager.shutdown();
    }

    public void call(T request, ServiceResponseListener<S> listener) {
        ChannelBuffer buffer = this.messageBufferPool.acquire();
        this.serializer.serialize(request, buffer);
        this.responseListeners.add(listener);
        this.tcpClient.write(buffer).awaitUninterruptibly();
        this.messageBufferPool.release(buffer);
    }

    public GraphName getName() {
        return this.serviceDeclaration.getName();
    }

    public String toString() {
        return "ServiceClient<" + this.serviceDeclaration + ">";
    }

    public T newMessage() {
        return this.messageFactory.newFromType(this.serviceDeclaration.getType());
    }

    public boolean isConnected() {
        return this.tcpClient.getChannel().isConnected();
    }
}
