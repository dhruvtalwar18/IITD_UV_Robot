package org.ros.internal.node.server.master;

import com.google.common.annotations.VisibleForTesting;
import com.google.common.collect.Lists;
import java.net.URI;
import java.util.Collection;
import java.util.List;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.address.AdvertiseAddress;
import org.ros.address.BindAddress;
import org.ros.internal.node.client.SlaveClient;
import org.ros.internal.node.server.XmlRpcServer;
import org.ros.internal.node.xmlrpc.MasterXmlRpcEndpointImpl;
import org.ros.namespace.GraphName;

public class MasterServer extends XmlRpcServer implements MasterRegistrationListener {
    private static final boolean DEBUG = false;
    private static final GraphName MASTER_NODE_NAME = GraphName.of("/master");
    public static final int SYSTEM_STATE_PUBLISHERS = 0;
    public static final int SYSTEM_STATE_SERVICES = 2;
    public static final int SYSTEM_STATE_SUBSCRIBERS = 1;
    private static final Log log = LogFactory.getLog(MasterServer.class);
    private final MasterRegistrationManagerImpl masterRegistrationManager = new MasterRegistrationManagerImpl(this);

    public MasterServer(BindAddress bindAddress, AdvertiseAddress advertiseAddress) {
        super(bindAddress, advertiseAddress);
    }

    public void start() {
        super.start(MasterXmlRpcEndpointImpl.class, new MasterXmlRpcEndpointImpl(this));
    }

    public void registerService(GraphName nodeName, URI nodeSlaveUri, GraphName serviceName, URI serviceUri) {
        synchronized (this.masterRegistrationManager) {
            this.masterRegistrationManager.registerService(nodeName, nodeSlaveUri, serviceName, serviceUri);
        }
    }

    public boolean unregisterService(GraphName nodeName, GraphName serviceName, URI serviceUri) {
        boolean unregisterService;
        synchronized (this.masterRegistrationManager) {
            unregisterService = this.masterRegistrationManager.unregisterService(nodeName, serviceName, serviceUri);
        }
        return unregisterService;
    }

    public List<URI> registerSubscriber(GraphName nodeName, URI nodeSlaveUri, GraphName topicName, String topicMessageType) {
        List<URI> publisherUris;
        synchronized (this.masterRegistrationManager) {
            TopicRegistrationInfo topicInfo = this.masterRegistrationManager.registerSubscriber(nodeName, nodeSlaveUri, topicName, topicMessageType);
            publisherUris = Lists.newArrayList();
            for (NodeRegistrationInfo publisherNodeInfo : topicInfo.getPublishers()) {
                publisherUris.add(publisherNodeInfo.getNodeSlaveUri());
            }
        }
        return publisherUris;
    }

    public boolean unregisterSubscriber(GraphName nodeName, GraphName topicName) {
        boolean unregisterSubscriber;
        synchronized (this.masterRegistrationManager) {
            unregisterSubscriber = this.masterRegistrationManager.unregisterSubscriber(nodeName, topicName);
        }
        return unregisterSubscriber;
    }

    public List<URI> registerPublisher(GraphName nodeName, URI nodeSlaveUri, GraphName topicName, String topicMessageType) {
        List<URI> subscriberSlaveUris;
        synchronized (this.masterRegistrationManager) {
            TopicRegistrationInfo topicInfo = this.masterRegistrationManager.registerPublisher(nodeName, nodeSlaveUri, topicName, topicMessageType);
            subscriberSlaveUris = Lists.newArrayList();
            for (NodeRegistrationInfo publisherNodeInfo : topicInfo.getSubscribers()) {
                subscriberSlaveUris.add(publisherNodeInfo.getNodeSlaveUri());
            }
            publisherUpdate(topicInfo, subscriberSlaveUris);
        }
        return subscriberSlaveUris;
    }

    private void publisherUpdate(TopicRegistrationInfo topicInfo, List<URI> subscriberSlaveUris) {
        List<URI> publisherUris = Lists.newArrayList();
        for (NodeRegistrationInfo publisherNodeInfo : topicInfo.getPublishers()) {
            publisherUris.add(publisherNodeInfo.getNodeSlaveUri());
        }
        GraphName topicName = topicInfo.getTopicName();
        for (URI subscriberSlaveUri : subscriberSlaveUris) {
            contactSubscriberForPublisherUpdate(subscriberSlaveUri, topicName, publisherUris);
        }
    }

    /* access modifiers changed from: protected */
    @VisibleForTesting
    public void contactSubscriberForPublisherUpdate(URI subscriberSlaveUri, GraphName topicName, List<URI> publisherUris) {
        new SlaveClient(MASTER_NODE_NAME, subscriberSlaveUri).publisherUpdate(topicName, publisherUris);
    }

    public boolean unregisterPublisher(GraphName nodeName, GraphName topicName) {
        boolean unregisterPublisher;
        synchronized (this.masterRegistrationManager) {
            unregisterPublisher = this.masterRegistrationManager.unregisterPublisher(nodeName, topicName);
        }
        return unregisterPublisher;
    }

    public URI lookupNode(GraphName nodeName) {
        synchronized (this.masterRegistrationManager) {
            NodeRegistrationInfo node = this.masterRegistrationManager.getNodeRegistrationInfo(nodeName);
            if (node == null) {
                return null;
            }
            URI nodeSlaveUri = node.getNodeSlaveUri();
            return nodeSlaveUri;
        }
    }

    public List<List<String>> getTopicTypes(GraphName calledId) {
        List<List<String>> result;
        synchronized (this.masterRegistrationManager) {
            result = Lists.newArrayList();
            for (TopicRegistrationInfo topic : this.masterRegistrationManager.getAllTopics()) {
                result.add(Lists.newArrayList((E[]) new String[]{topic.getTopicName().toString(), topic.getMessageType()}));
            }
        }
        return result;
    }

    public List<Object> getSystemState() {
        List<Object> result;
        synchronized (this.masterRegistrationManager) {
            result = Lists.newArrayList();
            Collection<TopicRegistrationInfo> topics = this.masterRegistrationManager.getAllTopics();
            result.add(getSystemStatePublishers(topics));
            result.add(getSystemStateSubscribers(topics));
            result.add(getSystemStateServices());
        }
        return result;
    }

    private List<Object> getSystemStatePublishers(Collection<TopicRegistrationInfo> topics) {
        List<Object> result = Lists.newArrayList();
        for (TopicRegistrationInfo topic : topics) {
            if (topic.hasPublishers()) {
                List<Object> topicInfo = Lists.newArrayList();
                topicInfo.add(topic.getTopicName().toString());
                List<String> publist = Lists.newArrayList();
                for (NodeRegistrationInfo node : topic.getPublishers()) {
                    publist.add(node.getNodeName().toString());
                }
                topicInfo.add(publist);
                result.add(topicInfo);
            }
        }
        return result;
    }

    private List<Object> getSystemStateSubscribers(Collection<TopicRegistrationInfo> topics) {
        List<Object> result = Lists.newArrayList();
        for (TopicRegistrationInfo topic : topics) {
            if (topic.hasSubscribers()) {
                List<Object> topicInfo = Lists.newArrayList();
                topicInfo.add(topic.getTopicName().toString());
                List<Object> sublist = Lists.newArrayList();
                for (NodeRegistrationInfo node : topic.getSubscribers()) {
                    sublist.add(node.getNodeName().toString());
                }
                topicInfo.add(sublist);
                result.add(topicInfo);
            }
        }
        return result;
    }

    private List<Object> getSystemStateServices() {
        List<Object> result = Lists.newArrayList();
        for (ServiceRegistrationInfo service : this.masterRegistrationManager.getAllServices()) {
            List<Object> topicInfo = Lists.newArrayList();
            topicInfo.add(service.getServiceName().toString());
            topicInfo.add(Lists.newArrayList((E[]) new String[]{service.getServiceName().toString()}));
            result.add(topicInfo);
        }
        return result;
    }

    public URI lookupService(GraphName serviceName) {
        synchronized (this.masterRegistrationManager) {
            ServiceRegistrationInfo service = this.masterRegistrationManager.getServiceRegistrationInfo(serviceName);
            if (service == null) {
                return null;
            }
            URI serviceUri = service.getServiceUri();
            return serviceUri;
        }
    }

    public List<Object> getPublishedTopics(GraphName caller, GraphName subgraph) {
        List<Object> result;
        synchronized (this.masterRegistrationManager) {
            result = Lists.newArrayList();
            for (TopicRegistrationInfo topic : this.masterRegistrationManager.getAllTopics()) {
                if (topic.hasPublishers()) {
                    result.add(Lists.newArrayList((E[]) new String[]{topic.getTopicName().toString(), topic.getMessageType()}));
                }
            }
        }
        return result;
    }

    public void onNodeReplacement(NodeRegistrationInfo nodeInfo) {
        if (log.isWarnEnabled()) {
            log.warn(String.format("Existing node %s with slave URI %s will be shutdown.", new Object[]{nodeInfo.getNodeName(), nodeInfo.getNodeSlaveUri()}));
        }
        new SlaveClient(MASTER_NODE_NAME, nodeInfo.getNodeSlaveUri()).shutdown("Replaced by new slave");
    }
}
