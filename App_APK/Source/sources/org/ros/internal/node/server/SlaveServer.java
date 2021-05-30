package org.ros.internal.node.server;

import com.google.common.collect.Lists;
import java.net.URI;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import org.ros.address.AdvertiseAddress;
import org.ros.address.BindAddress;
import org.ros.internal.node.client.MasterClient;
import org.ros.internal.node.parameter.ParameterManager;
import org.ros.internal.node.service.ServiceManager;
import org.ros.internal.node.topic.DefaultPublisher;
import org.ros.internal.node.topic.DefaultSubscriber;
import org.ros.internal.node.topic.PublisherIdentifier;
import org.ros.internal.node.topic.SubscriberIdentifier;
import org.ros.internal.node.topic.TopicParticipantManager;
import org.ros.internal.node.xmlrpc.SlaveXmlRpcEndpointImpl;
import org.ros.internal.system.Process;
import org.ros.internal.transport.ProtocolDescription;
import org.ros.internal.transport.ProtocolNames;
import org.ros.internal.transport.tcp.TcpRosProtocolDescription;
import org.ros.internal.transport.tcp.TcpRosServer;
import org.ros.namespace.GraphName;

public class SlaveServer extends XmlRpcServer {
    private final MasterClient masterClient;
    private final GraphName nodeName;
    private final ParameterManager parameterManager;
    private final TcpRosServer tcpRosServer;
    private final TopicParticipantManager topicParticipantManager;

    /* JADX INFO: super call moved to the top of the method (can break code semantics) */
    public SlaveServer(GraphName nodeName2, BindAddress tcpRosBindAddress, AdvertiseAddress tcpRosAdvertiseAddress, BindAddress xmlRpcBindAddress, AdvertiseAddress xmlRpcAdvertiseAddress, MasterClient master, TopicParticipantManager topicParticipantManager2, ServiceManager serviceManager, ParameterManager parameterManager2, ScheduledExecutorService executorService) {
        super(xmlRpcBindAddress, xmlRpcAdvertiseAddress);
        this.nodeName = nodeName2;
        this.masterClient = master;
        this.topicParticipantManager = topicParticipantManager2;
        this.parameterManager = parameterManager2;
        this.tcpRosServer = new TcpRosServer(tcpRosBindAddress, tcpRosAdvertiseAddress, topicParticipantManager2, serviceManager, executorService);
    }

    public AdvertiseAddress getTcpRosAdvertiseAddress() {
        return this.tcpRosServer.getAdvertiseAddress();
    }

    public void start() {
        super.start(SlaveXmlRpcEndpointImpl.class, new SlaveXmlRpcEndpointImpl(this));
        this.tcpRosServer.start();
    }

    public void shutdown() {
        super.shutdown();
        this.tcpRosServer.shutdown();
    }

    public List<Object> getBusStats(String callerId) {
        throw new UnsupportedOperationException();
    }

    public List<Object> getBusInfo(String callerId) {
        List<Object> busInfo = Lists.newArrayList();
        int id = 0;
        for (DefaultPublisher<?> publisher : getPublications()) {
            for (SubscriberIdentifier subscriberIdentifier : this.topicParticipantManager.getPublisherConnections(publisher)) {
                List<String> publisherBusInfo = Lists.newArrayList();
                publisherBusInfo.add(Integer.toString(id));
                publisherBusInfo.add(subscriberIdentifier.getNodeIdentifier().getName().toString());
                publisherBusInfo.add("o");
                publisherBusInfo.add(ProtocolNames.TCPROS);
                publisherBusInfo.add(publisher.getTopicName().toString());
                busInfo.add(publisherBusInfo);
                id++;
            }
        }
        for (DefaultSubscriber<?> subscriber : getSubscriptions()) {
            for (PublisherIdentifier publisherIdentifer : this.topicParticipantManager.getSubscriberConnections(subscriber)) {
                List<String> subscriberBusInfo = Lists.newArrayList();
                subscriberBusInfo.add(Integer.toString(id));
                subscriberBusInfo.add(publisherIdentifer.getNodeIdentifier().getUri().toString());
                subscriberBusInfo.add("i");
                subscriberBusInfo.add(ProtocolNames.TCPROS);
                subscriberBusInfo.add(publisherIdentifer.getTopicName().toString());
                busInfo.add(subscriberBusInfo);
                id++;
            }
        }
        return busInfo;
    }

    public URI getMasterUri() {
        return this.masterClient.getRemoteUri();
    }

    public int getPid() {
        return Process.getPid();
    }

    public Collection<DefaultSubscriber<?>> getSubscriptions() {
        return this.topicParticipantManager.getSubscribers();
    }

    public Collection<DefaultPublisher<?>> getPublications() {
        return this.topicParticipantManager.getPublishers();
    }

    public int paramUpdate(GraphName parameterName, Object parameterValue) {
        return this.parameterManager.updateParameter(parameterName, parameterValue);
    }

    public void publisherUpdate(String callerId, String topicName, Collection<URI> publisherUris) {
        GraphName graphName = GraphName.of(topicName);
        if (this.topicParticipantManager.hasSubscriber(graphName)) {
            DefaultSubscriber<?> subscriber = this.topicParticipantManager.getSubscriber(graphName);
            subscriber.updatePublishers(PublisherIdentifier.newCollectionFromUris(publisherUris, subscriber.getTopicDeclaration()));
        }
    }

    public ProtocolDescription requestTopic(String topicName, Collection<String> protocols) throws ServerException {
        GraphName graphName = GraphName.of(topicName).toGlobal();
        if (this.topicParticipantManager.hasPublisher(graphName)) {
            for (String protocol : protocols) {
                if (protocol.equals(ProtocolNames.TCPROS)) {
                    try {
                        return new TcpRosProtocolDescription(this.tcpRosServer.getAdvertiseAddress());
                    } catch (Exception e) {
                        throw new ServerException(e);
                    }
                }
            }
            throw new ServerException("No supported protocols specified.");
        }
        throw new ServerException("No publishers for topic: " + graphName);
    }

    public NodeIdentifier toNodeIdentifier() {
        return new NodeIdentifier(this.nodeName, getUri());
    }
}
