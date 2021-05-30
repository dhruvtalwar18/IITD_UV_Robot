package org.ros.internal.node.server.master;

import com.google.common.collect.Maps;
import java.net.URI;
import java.util.Collection;
import java.util.Collections;
import java.util.Map;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.namespace.GraphName;

public class MasterRegistrationManagerImpl {
    private static final Log log = LogFactory.getLog(MasterRegistrationManagerImpl.class);
    private final MasterRegistrationListener listener;
    private final Map<GraphName, NodeRegistrationInfo> nodes = Maps.newHashMap();
    private final Map<GraphName, ServiceRegistrationInfo> services = Maps.newConcurrentMap();
    private final Map<GraphName, TopicRegistrationInfo> topics = Maps.newHashMap();

    public MasterRegistrationManagerImpl(MasterRegistrationListener listener2) {
        this.listener = listener2;
    }

    public TopicRegistrationInfo registerPublisher(GraphName nodeName, URI nodeSlaveUri, GraphName topicName, String topicMessageType) {
        if (log.isDebugEnabled()) {
            log.debug(String.format("Registering publisher topic %s with message type %s on node %s with slave URI %s", new Object[]{topicName, topicMessageType, nodeName, nodeSlaveUri}));
        }
        TopicRegistrationInfo topic = obtainTopicRegistrationInfo(topicName, true);
        NodeRegistrationInfo node = obtainNodeRegistrationInfo(nodeName, nodeSlaveUri);
        topic.addPublisher(node, topicMessageType);
        node.addPublisher(topic);
        return topic;
    }

    public boolean unregisterPublisher(GraphName nodeName, GraphName topicName) {
        if (log.isDebugEnabled()) {
            log.debug(String.format("Unregistering publisher of topic %s from node %s", new Object[]{topicName, nodeName}));
        }
        TopicRegistrationInfo topic = obtainTopicRegistrationInfo(topicName, false);
        if (topic != null) {
            NodeRegistrationInfo node = this.nodes.get(nodeName);
            if (node != null) {
                node.removePublisher(topic);
                topic.removePublisher(node);
                potentiallyDeleteNode(node);
                return true;
            }
            if (log.isWarnEnabled()) {
                log.warn(String.format("Received unregister publisher for topic %s on unknown node %s", new Object[]{topicName, nodeName}));
            }
            return false;
        }
        if (log.isWarnEnabled()) {
            log.warn(String.format("Received unregister publisher for unknown topic %s on node %s", new Object[]{topicName, nodeName}));
        }
        return false;
    }

    public TopicRegistrationInfo registerSubscriber(GraphName nodeName, URI nodeSlaveUri, GraphName topicName, String topicMessageType) {
        if (log.isDebugEnabled()) {
            log.debug(String.format("Registering subscriber topic %s with message type %s on node %s with slave URI %s", new Object[]{topicName, topicMessageType, nodeName, nodeSlaveUri}));
        }
        TopicRegistrationInfo topic = obtainTopicRegistrationInfo(topicName, true);
        NodeRegistrationInfo node = obtainNodeRegistrationInfo(nodeName, nodeSlaveUri);
        topic.addSubscriber(node, topicMessageType);
        node.addSubscriber(topic);
        return topic;
    }

    public boolean unregisterSubscriber(GraphName nodeName, GraphName topicName) {
        if (log.isDebugEnabled()) {
            log.debug(String.format("Unregistering subscriber of topic %s from node %s", new Object[]{topicName, nodeName}));
        }
        TopicRegistrationInfo topic = obtainTopicRegistrationInfo(topicName, false);
        if (topic != null) {
            NodeRegistrationInfo node = this.nodes.get(nodeName);
            if (node != null) {
                node.removeSubscriber(topic);
                topic.removeSubscriber(node);
                potentiallyDeleteNode(node);
                return true;
            }
            if (log.isWarnEnabled()) {
                log.warn(String.format("Received unregister subscriber for topic %s on unknown node %s", new Object[]{topicName, nodeName}));
            }
            return false;
        }
        if (log.isWarnEnabled()) {
            log.warn(String.format("Received unregister subscriber for unknown topic %s on node %s", new Object[]{topicName, nodeName}));
        }
        return false;
    }

    public ServiceRegistrationInfo registerService(GraphName nodeName, URI nodeSlaveUri, GraphName serviceName, URI serviceUri) {
        if (log.isDebugEnabled()) {
            log.debug(String.format("Registering service %s with server URI %s on node %s with slave URI %s", new Object[]{serviceName, serviceUri, nodeName, nodeSlaveUri}));
        }
        NodeRegistrationInfo node = obtainNodeRegistrationInfo(nodeName, nodeSlaveUri);
        ServiceRegistrationInfo service = this.services.get(serviceName);
        if (service != null) {
            NodeRegistrationInfo previousServiceNode = service.getNode();
            if (previousServiceNode == node) {
                if (log.isWarnEnabled()) {
                    log.warn(String.format("Registering already known service %s with server URI %s on node %s with slave URI %s", new Object[]{serviceName, serviceUri, nodeName, nodeSlaveUri}));
                }
                return service;
            }
            previousServiceNode.removeService(service);
            potentiallyDeleteNode(previousServiceNode);
        }
        ServiceRegistrationInfo service2 = new ServiceRegistrationInfo(serviceName, serviceUri, node);
        node.addService(service2);
        this.services.put(serviceName, service2);
        return service2;
    }

    public boolean unregisterService(GraphName nodeName, GraphName serviceName, URI serviceUri) {
        if (log.isDebugEnabled()) {
            log.debug(String.format("Unregistering service %s from node %s", new Object[]{serviceName, nodeName}));
        }
        ServiceRegistrationInfo service = this.services.get(serviceName);
        if (service != null) {
            NodeRegistrationInfo node = this.nodes.get(nodeName);
            if (node != null) {
                this.services.remove(serviceName);
                node.removeService(service);
                potentiallyDeleteNode(node);
                return true;
            }
            if (log.isWarnEnabled()) {
                log.warn(String.format("Received unregister for service %s on unknown node %s", new Object[]{serviceName, nodeName}));
            }
            return false;
        }
        if (log.isWarnEnabled()) {
            log.warn(String.format("Received unregister for unknown service %s on node %s", new Object[]{serviceName, nodeName}));
        }
        return false;
    }

    public Collection<TopicRegistrationInfo> getAllTopics() {
        return Collections.unmodifiableCollection(this.topics.values());
    }

    public TopicRegistrationInfo getTopicRegistrationInfo(GraphName topicName) {
        return this.topics.get(topicName);
    }

    public NodeRegistrationInfo getNodeRegistrationInfo(GraphName nodeName) {
        return this.nodes.get(nodeName);
    }

    public Collection<ServiceRegistrationInfo> getAllServices() {
        return Collections.unmodifiableCollection(this.services.values());
    }

    public ServiceRegistrationInfo getServiceRegistrationInfo(GraphName serviceName) {
        return this.services.get(serviceName);
    }

    private TopicRegistrationInfo obtainTopicRegistrationInfo(GraphName topicName, boolean shouldCreate) {
        TopicRegistrationInfo info = this.topics.get(topicName);
        if (info != null || !shouldCreate) {
            return info;
        }
        TopicRegistrationInfo info2 = new TopicRegistrationInfo(topicName);
        this.topics.put(topicName, info2);
        return info2;
    }

    private NodeRegistrationInfo obtainNodeRegistrationInfo(GraphName nodeName, URI nodeSlaveUri) {
        NodeRegistrationInfo node = this.nodes.get(nodeName);
        if (node != null) {
            if (node.getNodeSlaveUri().equals(nodeSlaveUri)) {
                return node;
            }
            potentiallyDeleteNode(node);
            cleanupNode(node);
            try {
                this.listener.onNodeReplacement(node);
            } catch (Exception e) {
                log.error("Error during onNodeReplacement call", e);
            }
        }
        NodeRegistrationInfo node2 = new NodeRegistrationInfo(nodeName, nodeSlaveUri);
        this.nodes.put(nodeName, node2);
        return node2;
    }

    private void cleanupNode(NodeRegistrationInfo node) {
        for (TopicRegistrationInfo topic : node.getPublishers()) {
            topic.removePublisher(node);
        }
        for (TopicRegistrationInfo topic2 : node.getSubscribers()) {
            topic2.removeSubscriber(node);
        }
        for (ServiceRegistrationInfo service : node.getServices()) {
            this.services.remove(service.getServiceName());
        }
    }

    private void potentiallyDeleteNode(NodeRegistrationInfo node) {
        if (!node.hasRegistrations()) {
            this.nodes.remove(node.getNodeName());
        }
    }
}
