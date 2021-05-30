package org.ros.internal.node.service;

import com.google.common.base.Preconditions;
import java.net.URI;
import java.util.concurrent.ScheduledExecutorService;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.channel.ChannelHandler;
import org.ros.address.AdvertiseAddress;
import org.ros.concurrent.ListenerGroup;
import org.ros.concurrent.SignalRunnable;
import org.ros.internal.message.service.ServiceDescription;
import org.ros.internal.node.topic.DefaultPublisher;
import org.ros.internal.transport.ConnectionHeader;
import org.ros.internal.transport.ConnectionHeaderFields;
import org.ros.message.MessageDeserializer;
import org.ros.message.MessageFactory;
import org.ros.message.MessageSerializer;
import org.ros.namespace.GraphName;
import org.ros.node.service.DefaultServiceServerListener;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import org.ros.node.service.ServiceServerListener;

public class DefaultServiceServer<T, S> implements ServiceServer<T, S> {
    private static final boolean DEBUG = false;
    /* access modifiers changed from: private */
    public static final Log log = LogFactory.getLog(DefaultPublisher.class);
    private final AdvertiseAddress advertiseAddress;
    private final ListenerGroup<ServiceServerListener<T, S>> listenerGroup;
    private final MessageDeserializer<T> messageDeserializer;
    private final MessageFactory messageFactory;
    private final MessageSerializer<S> messageSerializer;
    private final ScheduledExecutorService scheduledExecutorService;
    private final ServiceDeclaration serviceDeclaration;
    private final ServiceResponseBuilder<T, S> serviceResponseBuilder;

    public DefaultServiceServer(ServiceDeclaration serviceDeclaration2, ServiceResponseBuilder<T, S> serviceResponseBuilder2, AdvertiseAddress advertiseAddress2, MessageDeserializer<T> messageDeserializer2, MessageSerializer<S> messageSerializer2, MessageFactory messageFactory2, ScheduledExecutorService scheduledExecutorService2) {
        this.serviceDeclaration = serviceDeclaration2;
        this.serviceResponseBuilder = serviceResponseBuilder2;
        this.advertiseAddress = advertiseAddress2;
        this.messageDeserializer = messageDeserializer2;
        this.messageSerializer = messageSerializer2;
        this.messageFactory = messageFactory2;
        this.scheduledExecutorService = scheduledExecutorService2;
        this.listenerGroup = new ListenerGroup<>(scheduledExecutorService2);
        this.listenerGroup.add(new DefaultServiceServerListener<T, S>() {
            public void onMasterRegistrationSuccess(ServiceServer<T, S> serviceServer) {
                Log access$000 = DefaultServiceServer.log;
                access$000.info("Service registered: " + DefaultServiceServer.this);
            }

            public void onMasterRegistrationFailure(ServiceServer<T, S> serviceServer) {
                Log access$000 = DefaultServiceServer.log;
                access$000.info("Service registration failed: " + DefaultServiceServer.this);
            }

            public void onMasterUnregistrationSuccess(ServiceServer<T, S> serviceServer) {
                Log access$000 = DefaultServiceServer.log;
                access$000.info("Service unregistered: " + DefaultServiceServer.this);
            }

            public void onMasterUnregistrationFailure(ServiceServer<T, S> serviceServer) {
                Log access$000 = DefaultServiceServer.log;
                access$000.info("Service unregistration failed: " + DefaultServiceServer.this);
            }
        });
    }

    public ChannelBuffer finishHandshake(ConnectionHeader incomingConnectionHeader) {
        ConnectionHeader connectionHeader = toDeclaration().toConnectionHeader();
        String expectedChecksum = connectionHeader.getField(ConnectionHeaderFields.MD5_CHECKSUM);
        String incomingChecksum = incomingConnectionHeader.getField(ConnectionHeaderFields.MD5_CHECKSUM);
        Preconditions.checkState(incomingChecksum.equals(expectedChecksum) || incomingChecksum.equals("*"));
        return connectionHeader.encode();
    }

    public URI getUri() {
        return this.advertiseAddress.toUri("rosrpc");
    }

    public GraphName getName() {
        return this.serviceDeclaration.getName();
    }

    /* access modifiers changed from: package-private */
    public ServiceDeclaration toDeclaration() {
        return new ServiceDeclaration(new ServiceIdentifier(this.serviceDeclaration.getName(), getUri()), new ServiceDescription(this.serviceDeclaration.getType(), this.serviceDeclaration.getDefinition(), this.serviceDeclaration.getMd5Checksum()));
    }

    public ChannelHandler newRequestHandler() {
        return new ServiceRequestHandler(this.serviceDeclaration, this.serviceResponseBuilder, this.messageDeserializer, this.messageSerializer, this.messageFactory, this.scheduledExecutorService);
    }

    public void signalOnMasterRegistrationSuccess() {
        this.listenerGroup.signal(new SignalRunnable<ServiceServerListener<T, S>>() {
            public void run(ServiceServerListener<T, S> listener) {
                listener.onMasterRegistrationSuccess(this);
            }
        });
    }

    public void signalOnMasterRegistrationFailure() {
        this.listenerGroup.signal(new SignalRunnable<ServiceServerListener<T, S>>() {
            public void run(ServiceServerListener<T, S> listener) {
                listener.onMasterRegistrationFailure(this);
            }
        });
    }

    public void signalOnMasterUnregistrationSuccess() {
        this.listenerGroup.signal(new SignalRunnable<ServiceServerListener<T, S>>() {
            public void run(ServiceServerListener<T, S> listener) {
                listener.onMasterUnregistrationSuccess(this);
            }
        });
    }

    public void signalOnMasterUnregistrationFailure() {
        this.listenerGroup.signal(new SignalRunnable<ServiceServerListener<T, S>>() {
            public void run(ServiceServerListener<T, S> listener) {
                listener.onMasterUnregistrationFailure(this);
            }
        });
    }

    public void shutdown() {
        throw new UnsupportedOperationException();
    }

    public void addListener(ServiceServerListener<T, S> listener) {
        this.listenerGroup.add(listener);
    }

    public String toString() {
        return "ServiceServer<" + toDeclaration() + ">";
    }
}
