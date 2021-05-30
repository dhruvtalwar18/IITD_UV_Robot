package org.ros.internal.node;

import com.google.common.annotations.VisibleForTesting;
import java.net.InetSocketAddress;
import java.net.URI;
import java.util.Collection;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import org.ros.Parameters;
import org.ros.concurrent.CancellableLoop;
import org.ros.concurrent.ListenerGroup;
import org.ros.concurrent.SignalRunnable;
import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.internal.node.client.MasterClient;
import org.ros.internal.node.client.Registrar;
import org.ros.internal.node.parameter.DefaultParameterTree;
import org.ros.internal.node.parameter.ParameterManager;
import org.ros.internal.node.response.Response;
import org.ros.internal.node.response.StatusCode;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.node.server.SlaveServer;
import org.ros.internal.node.service.ServiceDeclaration;
import org.ros.internal.node.service.ServiceFactory;
import org.ros.internal.node.service.ServiceIdentifier;
import org.ros.internal.node.service.ServiceManager;
import org.ros.internal.node.topic.PublisherFactory;
import org.ros.internal.node.topic.SubscriberFactory;
import org.ros.internal.node.topic.TopicDeclaration;
import org.ros.internal.node.topic.TopicParticipantManager;
import org.ros.internal.node.xmlrpc.XmlRpcTimeoutException;
import org.ros.message.MessageDeserializer;
import org.ros.message.MessageFactory;
import org.ros.message.MessageSerializationFactory;
import org.ros.message.MessageSerializer;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.namespace.NodeNameResolver;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeListener;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import org.ros.node.topic.DefaultPublisherListener;
import org.ros.node.topic.DefaultSubscriberListener;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.time.ClockTopicTimeProvider;
import org.ros.time.TimeProvider;
import rosgraph_msgs.Clock;
import rosgraph_msgs.Log;

public class DefaultNode implements ConnectedNode {
    private static final boolean DEBUG = false;
    private static final int MAX_SHUTDOWN_DELAY_DURATION = 5;
    private static final TimeUnit MAX_SHUTDOWN_DELAY_UNITS = TimeUnit.SECONDS;
    private RosoutLogger log;
    private final MasterClient masterClient = new MasterClient(this.masterUri);
    private final URI masterUri;
    private final NodeConfiguration nodeConfiguration;
    private final ListenerGroup<NodeListener> nodeListeners;
    private final GraphName nodeName;
    private final ParameterManager parameterManager;
    private final ParameterTree parameterTree;
    private final PublisherFactory publisherFactory;
    private final Registrar registrar;
    private final NodeNameResolver resolver;
    private final ScheduledExecutorService scheduledExecutorService;
    private final ServiceFactory serviceFactory;
    private final ServiceManager serviceManager = new ServiceManager();
    private final SlaveServer slaveServer;
    private final SubscriberFactory subscriberFactory;
    private TimeProvider timeProvider;
    private final TopicParticipantManager topicParticipantManager = new TopicParticipantManager();

    public DefaultNode(NodeConfiguration nodeConfiguration2, Collection<NodeListener> nodeListeners2, ScheduledExecutorService scheduledExecutorService2) {
        ScheduledExecutorService scheduledExecutorService3 = scheduledExecutorService2;
        this.nodeConfiguration = NodeConfiguration.copyOf(nodeConfiguration2);
        this.nodeListeners = new ListenerGroup<>(scheduledExecutorService3);
        this.nodeListeners.addAll(nodeListeners2);
        this.scheduledExecutorService = scheduledExecutorService3;
        this.masterUri = nodeConfiguration2.getMasterUri();
        this.parameterManager = new ParameterManager(scheduledExecutorService3);
        GraphName basename = nodeConfiguration2.getNodeName();
        NameResolver parentResolver = nodeConfiguration2.getParentResolver();
        this.nodeName = parentResolver.getNamespace().join(basename);
        this.resolver = new NodeNameResolver(this.nodeName, parentResolver);
        SlaveServer slaveServer2 = r1;
        SlaveServer slaveServer3 = new SlaveServer(this.nodeName, nodeConfiguration2.getTcpRosBindAddress(), nodeConfiguration2.getTcpRosAdvertiseAddress(), nodeConfiguration2.getXmlRpcBindAddress(), nodeConfiguration2.getXmlRpcAdvertiseAddress(), this.masterClient, this.topicParticipantManager, this.serviceManager, this.parameterManager, scheduledExecutorService2);
        this.slaveServer = slaveServer2;
        this.slaveServer.start();
        NodeIdentifier nodeIdentifier = this.slaveServer.toNodeIdentifier();
        this.parameterTree = DefaultParameterTree.newFromNodeIdentifier(nodeIdentifier, this.masterClient.getRemoteUri(), this.resolver, this.parameterManager);
        this.publisherFactory = new PublisherFactory(nodeIdentifier, this.topicParticipantManager, nodeConfiguration2.getTopicMessageFactory(), scheduledExecutorService3);
        this.subscriberFactory = new SubscriberFactory(nodeIdentifier, this.topicParticipantManager, scheduledExecutorService3);
        this.serviceFactory = new ServiceFactory(this.nodeName, this.slaveServer, this.serviceManager, scheduledExecutorService3);
        this.registrar = new Registrar(this.masterClient, scheduledExecutorService3);
        this.topicParticipantManager.setListener(this.registrar);
        this.serviceManager.setListener(this.registrar);
        scheduledExecutorService3.execute(new Runnable() {
            public void run() {
                DefaultNode.this.start();
            }
        });
    }

    /* access modifiers changed from: private */
    public void start() {
        this.registrar.start(this.slaveServer.toNodeIdentifier());
        final CountDownLatch rosoutLatch = new CountDownLatch(1);
        this.log = new RosoutLogger(this);
        this.log.getPublisher().addListener(new DefaultPublisherListener<Log>() {
            public void onMasterRegistrationSuccess(Publisher<Log> publisher) {
                rosoutLatch.countDown();
            }
        });
        try {
            rosoutLatch.await();
            boolean useSimTime = false;
            try {
                if (this.parameterTree.has(Parameters.USE_SIM_TIME) && this.parameterTree.getBoolean(Parameters.USE_SIM_TIME)) {
                    useSimTime = true;
                }
                final CountDownLatch timeLatch = new CountDownLatch(1);
                if (useSimTime) {
                    ClockTopicTimeProvider clockTopicTimeProvider = new ClockTopicTimeProvider(this);
                    clockTopicTimeProvider.getSubscriber().addSubscriberListener(new DefaultSubscriberListener<Clock>() {
                        public void onMasterRegistrationSuccess(Subscriber<Clock> subscriber) {
                            timeLatch.countDown();
                        }
                    });
                    this.timeProvider = clockTopicTimeProvider;
                } else {
                    this.timeProvider = this.nodeConfiguration.getTimeProvider();
                    timeLatch.countDown();
                }
                try {
                    timeLatch.await();
                    signalOnStart();
                } catch (InterruptedException e) {
                    signalOnError(e);
                    shutdown();
                }
            } catch (Exception e2) {
                signalOnError(e2);
                shutdown();
            }
        } catch (InterruptedException e3) {
            signalOnError(e3);
            shutdown();
        }
    }

    /* access modifiers changed from: package-private */
    @VisibleForTesting
    public Registrar getRegistrar() {
        return this.registrar;
    }

    private <T> MessageSerializer<T> newMessageSerializer(String messageType) {
        return this.nodeConfiguration.getMessageSerializationFactory().newMessageSerializer(messageType);
    }

    private <T> MessageDeserializer<T> newMessageDeserializer(String messageType) {
        return this.nodeConfiguration.getMessageSerializationFactory().newMessageDeserializer(messageType);
    }

    private <T> MessageSerializer<T> newServiceResponseSerializer(String serviceType) {
        return this.nodeConfiguration.getMessageSerializationFactory().newServiceResponseSerializer(serviceType);
    }

    private <T> MessageDeserializer<T> newServiceResponseDeserializer(String serviceType) {
        return this.nodeConfiguration.getMessageSerializationFactory().newServiceResponseDeserializer(serviceType);
    }

    private <T> MessageSerializer<T> newServiceRequestSerializer(String serviceType) {
        return this.nodeConfiguration.getMessageSerializationFactory().newServiceRequestSerializer(serviceType);
    }

    private <T> MessageDeserializer<T> newServiceRequestDeserializer(String serviceType) {
        return this.nodeConfiguration.getMessageSerializationFactory().newServiceRequestDeserializer(serviceType);
    }

    public <T> Publisher<T> newPublisher(GraphName topicName, String messageType) {
        return this.publisherFactory.newOrExisting(TopicDeclaration.newFromTopicName(resolveName(topicName), this.nodeConfiguration.getTopicDescriptionFactory().newFromType(messageType)), newMessageSerializer(messageType));
    }

    public <T> Publisher<T> newPublisher(String topicName, String messageType) {
        return newPublisher(GraphName.of(topicName), messageType);
    }

    public <T> Subscriber<T> newSubscriber(GraphName topicName, String messageType) {
        return this.subscriberFactory.newOrExisting(TopicDeclaration.newFromTopicName(resolveName(topicName), this.nodeConfiguration.getTopicDescriptionFactory().newFromType(messageType)), newMessageDeserializer(messageType));
    }

    public <T> Subscriber<T> newSubscriber(String topicName, String messageType) {
        return newSubscriber(GraphName.of(topicName), messageType);
    }

    public <T, S> ServiceServer<T, S> newServiceServer(GraphName serviceName, String serviceType, ServiceResponseBuilder<T, S> responseBuilder) {
        ServiceDeclaration definition = new ServiceDeclaration(new ServiceIdentifier(resolveName(serviceName), (URI) null), this.nodeConfiguration.getServiceDescriptionFactory().newFromType(serviceType));
        MessageDeserializer<T> requestDeserializer = newServiceRequestDeserializer(serviceType);
        MessageSerializer<S> responseSerializer = newServiceResponseSerializer(serviceType);
        return this.serviceFactory.newServer(definition, responseBuilder, requestDeserializer, responseSerializer, this.nodeConfiguration.getServiceResponseMessageFactory());
    }

    public <T, S> ServiceServer<T, S> newServiceServer(String serviceName, String serviceType, ServiceResponseBuilder<T, S> responseBuilder) {
        return newServiceServer(GraphName.of(serviceName), serviceType, responseBuilder);
    }

    public <T, S> ServiceServer<T, S> getServiceServer(GraphName serviceName) {
        return this.serviceManager.getServer(serviceName);
    }

    public <T, S> ServiceServer<T, S> getServiceServer(String serviceName) {
        return getServiceServer(GraphName.of(serviceName));
    }

    public URI lookupServiceUri(GraphName serviceName) {
        Response<URI> response = this.masterClient.lookupService(this.slaveServer.toNodeIdentifier().getName(), resolveName(serviceName).toString());
        if (response.getStatusCode() == StatusCode.SUCCESS) {
            return response.getResult();
        }
        return null;
    }

    public URI lookupServiceUri(String serviceName) {
        return lookupServiceUri(GraphName.of(serviceName));
    }

    public <T, S> ServiceClient<T, S> newServiceClient(GraphName serviceName, String serviceType) throws ServiceNotFoundException {
        GraphName resolvedServiceName = resolveName(serviceName);
        URI uri = lookupServiceUri(resolvedServiceName);
        if (uri != null) {
            return this.serviceFactory.newClient(new ServiceDeclaration(new ServiceIdentifier(resolvedServiceName, uri), this.nodeConfiguration.getServiceDescriptionFactory().newFromType(serviceType)), newServiceRequestSerializer(serviceType), newServiceResponseDeserializer(serviceType), this.nodeConfiguration.getServiceRequestMessageFactory());
        }
        throw new ServiceNotFoundException("No such service " + resolvedServiceName + " of type " + serviceType);
    }

    public <T, S> ServiceClient<T, S> newServiceClient(String serviceName, String serviceType) throws ServiceNotFoundException {
        return newServiceClient(GraphName.of(serviceName), serviceType);
    }

    public Time getCurrentTime() {
        return this.timeProvider.getCurrentTime();
    }

    public GraphName getName() {
        return this.nodeName;
    }

    public org.apache.commons.logging.Log getLog() {
        return this.log;
    }

    public GraphName resolveName(GraphName name) {
        return this.resolver.resolve(name);
    }

    public GraphName resolveName(String name) {
        return this.resolver.resolve(GraphName.of(name));
    }

    public void shutdown() {
        signalOnShutdown();
        this.slaveServer.shutdown();
        this.topicParticipantManager.shutdown();
        for (ServiceServer<?, ?> serviceServer : this.serviceManager.getServers()) {
            try {
                this.masterClient.unregisterService(this.slaveServer.toNodeIdentifier(), serviceServer);
            } catch (XmlRpcTimeoutException e) {
                this.log.error(e);
            } catch (RemoteException e2) {
                this.log.error(e2);
            }
        }
        for (ServiceClient<?, ?> serviceClient : this.serviceManager.getClients()) {
            serviceClient.shutdown();
        }
        this.registrar.shutdown();
        this.slaveServer.shutdown();
        signalOnShutdownComplete();
    }

    public URI getMasterUri() {
        return this.masterUri;
    }

    public NodeNameResolver getResolver() {
        return this.resolver;
    }

    public ParameterTree getParameterTree() {
        return this.parameterTree;
    }

    public URI getUri() {
        return this.slaveServer.getUri();
    }

    public MessageSerializationFactory getMessageSerializationFactory() {
        return this.nodeConfiguration.getMessageSerializationFactory();
    }

    public MessageFactory getTopicMessageFactory() {
        return this.nodeConfiguration.getTopicMessageFactory();
    }

    public MessageFactory getServiceRequestMessageFactory() {
        return this.nodeConfiguration.getServiceRequestMessageFactory();
    }

    public MessageFactory getServiceResponseMessageFactory() {
        return this.nodeConfiguration.getServiceResponseMessageFactory();
    }

    public void addListener(NodeListener listener) {
        this.nodeListeners.add(listener);
    }

    private void signalOnError(final Throwable throwable) {
        this.nodeListeners.signal(new SignalRunnable<NodeListener>() {
            public void run(NodeListener listener) {
                listener.onError(this, throwable);
            }
        });
    }

    public void removeListeners() {
        this.nodeListeners.shutdown();
    }

    private void signalOnStart() {
        this.nodeListeners.signal(new SignalRunnable<NodeListener>() {
            public void run(NodeListener listener) {
                listener.onStart(this);
            }
        });
    }

    private void signalOnShutdown() {
        try {
            this.nodeListeners.signal(new SignalRunnable<NodeListener>() {
                public void run(NodeListener listener) {
                    listener.onShutdown(this);
                }
            }, 5, MAX_SHUTDOWN_DELAY_UNITS);
        } catch (InterruptedException e) {
        }
    }

    private void signalOnShutdownComplete() {
        this.nodeListeners.signal(new SignalRunnable<NodeListener>() {
            public void run(NodeListener listener) {
                try {
                    listener.onShutdownComplete(this);
                } catch (Throwable th) {
                    System.out.println(listener);
                }
            }
        });
    }

    /* access modifiers changed from: package-private */
    @VisibleForTesting
    public InetSocketAddress getAddress() {
        return this.slaveServer.getAddress();
    }

    public ScheduledExecutorService getScheduledExecutorService() {
        return this.scheduledExecutorService;
    }

    public void executeCancellableLoop(final CancellableLoop cancellableLoop) {
        this.scheduledExecutorService.execute(cancellableLoop);
        addListener(new NodeListener() {
            public void onStart(ConnectedNode connectedNode) {
            }

            public void onShutdown(Node node) {
                cancellableLoop.cancel();
            }

            public void onShutdownComplete(Node node) {
            }

            public void onError(Node node, Throwable throwable) {
                cancellableLoop.cancel();
            }
        });
    }
}
