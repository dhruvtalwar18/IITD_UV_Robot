package org.ros.node;

import java.io.File;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import org.ros.address.AdvertiseAddress;
import org.ros.address.AdvertiseAddressFactory;
import org.ros.address.BindAddress;
import org.ros.address.PrivateAdvertiseAddressFactory;
import org.ros.address.PublicAdvertiseAddressFactory;
import org.ros.exception.RosRuntimeException;
import org.ros.internal.message.DefaultMessageFactory;
import org.ros.internal.message.DefaultMessageSerializationFactory;
import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import org.ros.internal.message.service.ServiceDescriptionFactory;
import org.ros.internal.message.service.ServiceRequestMessageFactory;
import org.ros.internal.message.service.ServiceResponseMessageFactory;
import org.ros.internal.message.topic.TopicDescriptionFactory;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageFactory;
import org.ros.message.MessageSerializationFactory;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.time.TimeProvider;
import org.ros.time.WallTimeProvider;

public class NodeConfiguration {
    public static final URI DEFAULT_MASTER_URI;
    private URI masterUri;
    private MessageSerializationFactory messageSerializationFactory;
    private GraphName nodeName;
    private NameResolver parentResolver;
    private List<File> rosPackagePath;
    private File rosRoot;
    private ScheduledExecutorService scheduledExecutorService;
    private ServiceDescriptionFactory serviceDescriptionFactory;
    private MessageFactory serviceRequestMessageFactory;
    private MessageFactory serviceResponseMessageFactory;
    private AdvertiseAddressFactory tcpRosAdvertiseAddressFactory;
    private BindAddress tcpRosBindAddress;
    private TimeProvider timeProvider;
    private TopicDescriptionFactory topicDescriptionFactory;
    private MessageFactory topicMessageFactory;
    private AdvertiseAddressFactory xmlRpcAdvertiseAddressFactory;
    private BindAddress xmlRpcBindAddress;

    static {
        try {
            DEFAULT_MASTER_URI = new URI("http://localhost:11311/");
        } catch (URISyntaxException e) {
            throw new RosRuntimeException((Throwable) e);
        }
    }

    public static NodeConfiguration copyOf(NodeConfiguration nodeConfiguration) {
        NodeConfiguration copy = new NodeConfiguration();
        copy.parentResolver = nodeConfiguration.parentResolver;
        copy.masterUri = nodeConfiguration.masterUri;
        copy.rosRoot = nodeConfiguration.rosRoot;
        copy.rosPackagePath = nodeConfiguration.rosPackagePath;
        copy.nodeName = nodeConfiguration.nodeName;
        copy.topicDescriptionFactory = nodeConfiguration.topicDescriptionFactory;
        copy.topicMessageFactory = nodeConfiguration.topicMessageFactory;
        copy.serviceDescriptionFactory = nodeConfiguration.serviceDescriptionFactory;
        copy.serviceRequestMessageFactory = nodeConfiguration.serviceRequestMessageFactory;
        copy.serviceResponseMessageFactory = nodeConfiguration.serviceResponseMessageFactory;
        copy.messageSerializationFactory = nodeConfiguration.messageSerializationFactory;
        copy.tcpRosBindAddress = nodeConfiguration.tcpRosBindAddress;
        copy.tcpRosAdvertiseAddressFactory = nodeConfiguration.tcpRosAdvertiseAddressFactory;
        copy.xmlRpcBindAddress = nodeConfiguration.xmlRpcBindAddress;
        copy.xmlRpcAdvertiseAddressFactory = nodeConfiguration.xmlRpcAdvertiseAddressFactory;
        copy.scheduledExecutorService = nodeConfiguration.scheduledExecutorService;
        copy.timeProvider = nodeConfiguration.timeProvider;
        return copy;
    }

    public static NodeConfiguration newPublic(String host, URI masterUri2) {
        NodeConfiguration configuration = new NodeConfiguration();
        configuration.setXmlRpcBindAddress(BindAddress.newPublic());
        configuration.setXmlRpcAdvertiseAddressFactory(new PublicAdvertiseAddressFactory(host));
        configuration.setTcpRosBindAddress(BindAddress.newPublic());
        configuration.setTcpRosAdvertiseAddressFactory(new PublicAdvertiseAddressFactory(host));
        configuration.setMasterUri(masterUri2);
        return configuration;
    }

    public static NodeConfiguration newPublic(String host) {
        return newPublic(host, DEFAULT_MASTER_URI);
    }

    public static NodeConfiguration newPrivate(URI masterUri2) {
        NodeConfiguration configuration = new NodeConfiguration();
        configuration.setXmlRpcBindAddress(BindAddress.newPrivate());
        configuration.setXmlRpcAdvertiseAddressFactory(new PrivateAdvertiseAddressFactory());
        configuration.setTcpRosBindAddress(BindAddress.newPrivate());
        configuration.setTcpRosAdvertiseAddressFactory(new PrivateAdvertiseAddressFactory());
        configuration.setMasterUri(masterUri2);
        return configuration;
    }

    public static NodeConfiguration newPrivate() {
        return newPrivate(DEFAULT_MASTER_URI);
    }

    private NodeConfiguration() {
        MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
        setTopicDescriptionFactory(new TopicDescriptionFactory(messageDefinitionProvider));
        setTopicMessageFactory(new DefaultMessageFactory(messageDefinitionProvider));
        setServiceDescriptionFactory(new ServiceDescriptionFactory(messageDefinitionProvider));
        setServiceRequestMessageFactory(new ServiceRequestMessageFactory(messageDefinitionProvider));
        setServiceResponseMessageFactory(new ServiceResponseMessageFactory(messageDefinitionProvider));
        setMessageSerializationFactory(new DefaultMessageSerializationFactory(messageDefinitionProvider));
        setParentResolver(NameResolver.newRoot());
        setTimeProvider(new WallTimeProvider());
    }

    public NameResolver getParentResolver() {
        return this.parentResolver;
    }

    public NodeConfiguration setParentResolver(NameResolver resolver) {
        this.parentResolver = resolver;
        return this;
    }

    public URI getMasterUri() {
        return this.masterUri;
    }

    public NodeConfiguration setMasterUri(URI masterUri2) {
        this.masterUri = masterUri2;
        return this;
    }

    public File getRosRoot() {
        return this.rosRoot;
    }

    public NodeConfiguration setRosRoot(File rosRoot2) {
        this.rosRoot = rosRoot2;
        return this;
    }

    public List<File> getRosPackagePath() {
        return this.rosPackagePath;
    }

    public NodeConfiguration setRosPackagePath(List<File> rosPackagePath2) {
        this.rosPackagePath = rosPackagePath2;
        return this;
    }

    public GraphName getNodeName() {
        return this.nodeName;
    }

    public NodeConfiguration setNodeName(GraphName nodeName2) {
        this.nodeName = nodeName2;
        return this;
    }

    public NodeConfiguration setNodeName(String nodeName2) {
        return setNodeName(GraphName.of(nodeName2));
    }

    public NodeConfiguration setDefaultNodeName(GraphName nodeName2) {
        if (this.nodeName == null) {
            setNodeName(nodeName2);
        }
        return this;
    }

    public NodeConfiguration setDefaultNodeName(String nodeName2) {
        return setDefaultNodeName(GraphName.of(nodeName2));
    }

    public MessageSerializationFactory getMessageSerializationFactory() {
        return this.messageSerializationFactory;
    }

    public NodeConfiguration setMessageSerializationFactory(MessageSerializationFactory messageSerializationFactory2) {
        this.messageSerializationFactory = messageSerializationFactory2;
        return this;
    }

    public NodeConfiguration setTopicMessageFactory(MessageFactory topicMessageFactory2) {
        this.topicMessageFactory = topicMessageFactory2;
        return this;
    }

    public MessageFactory getTopicMessageFactory() {
        return this.topicMessageFactory;
    }

    public NodeConfiguration setServiceRequestMessageFactory(ServiceRequestMessageFactory serviceRequestMessageFactory2) {
        this.serviceRequestMessageFactory = serviceRequestMessageFactory2;
        return this;
    }

    public MessageFactory getServiceRequestMessageFactory() {
        return this.serviceRequestMessageFactory;
    }

    public NodeConfiguration setServiceResponseMessageFactory(ServiceResponseMessageFactory serviceResponseMessageFactory2) {
        this.serviceResponseMessageFactory = serviceResponseMessageFactory2;
        return this;
    }

    public MessageFactory getServiceResponseMessageFactory() {
        return this.serviceResponseMessageFactory;
    }

    public NodeConfiguration setTopicDescriptionFactory(TopicDescriptionFactory topicDescriptionFactory2) {
        this.topicDescriptionFactory = topicDescriptionFactory2;
        return this;
    }

    public TopicDescriptionFactory getTopicDescriptionFactory() {
        return this.topicDescriptionFactory;
    }

    public NodeConfiguration setServiceDescriptionFactory(ServiceDescriptionFactory serviceDescriptionFactory2) {
        this.serviceDescriptionFactory = serviceDescriptionFactory2;
        return this;
    }

    public ServiceDescriptionFactory getServiceDescriptionFactory() {
        return this.serviceDescriptionFactory;
    }

    public BindAddress getTcpRosBindAddress() {
        return this.tcpRosBindAddress;
    }

    public NodeConfiguration setTcpRosBindAddress(BindAddress tcpRosBindAddress2) {
        this.tcpRosBindAddress = tcpRosBindAddress2;
        return this;
    }

    public AdvertiseAddressFactory getTcpRosAdvertiseAddressFactory() {
        return this.tcpRosAdvertiseAddressFactory;
    }

    public NodeConfiguration setTcpRosAdvertiseAddressFactory(AdvertiseAddressFactory tcpRosAdvertiseAddressFactory2) {
        this.tcpRosAdvertiseAddressFactory = tcpRosAdvertiseAddressFactory2;
        return this;
    }

    public AdvertiseAddress getTcpRosAdvertiseAddress() {
        return this.tcpRosAdvertiseAddressFactory.newDefault();
    }

    public BindAddress getXmlRpcBindAddress() {
        return this.xmlRpcBindAddress;
    }

    public NodeConfiguration setXmlRpcBindAddress(BindAddress xmlRpcBindAddress2) {
        this.xmlRpcBindAddress = xmlRpcBindAddress2;
        return this;
    }

    public AdvertiseAddress getXmlRpcAdvertiseAddress() {
        return this.xmlRpcAdvertiseAddressFactory.newDefault();
    }

    public AdvertiseAddressFactory getXmlRpcAdvertiseAddressFactory() {
        return this.xmlRpcAdvertiseAddressFactory;
    }

    public NodeConfiguration setXmlRpcAdvertiseAddressFactory(AdvertiseAddressFactory xmlRpcAdvertiseAddressFactory2) {
        this.xmlRpcAdvertiseAddressFactory = xmlRpcAdvertiseAddressFactory2;
        return this;
    }

    public TimeProvider getTimeProvider() {
        return this.timeProvider;
    }

    public NodeConfiguration setTimeProvider(TimeProvider timeProvider2) {
        this.timeProvider = timeProvider2;
        return this;
    }
}
