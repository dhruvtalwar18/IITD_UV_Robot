package org.ros.internal.node.client;

import java.net.URI;
import java.util.List;
import org.ros.internal.node.response.IntegerResultFactory;
import org.ros.internal.node.response.Response;
import org.ros.internal.node.response.SystemStateResultFactory;
import org.ros.internal.node.response.TopicListResultFactory;
import org.ros.internal.node.response.TopicTypeListResultFactory;
import org.ros.internal.node.response.UriListResultFactory;
import org.ros.internal.node.response.UriResultFactory;
import org.ros.internal.node.response.VoidResultFactory;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.node.topic.PublisherDeclaration;
import org.ros.internal.node.topic.PublisherIdentifier;
import org.ros.internal.node.topic.TopicDeclaration;
import org.ros.internal.node.xmlrpc.MasterXmlRpcEndpoint;
import org.ros.master.client.SystemState;
import org.ros.master.client.TopicType;
import org.ros.namespace.GraphName;
import org.ros.node.service.ServiceServer;
import org.ros.node.topic.Subscriber;

public class MasterClient extends Client<MasterXmlRpcEndpoint> {
    public /* bridge */ /* synthetic */ URI getRemoteUri() {
        return super.getRemoteUri();
    }

    public MasterClient(URI uri) {
        super(uri, MasterXmlRpcEndpoint.class);
    }

    public Response<Void> registerService(NodeIdentifier slave, ServiceServer<?, ?> service) {
        return Response.fromListChecked(((MasterXmlRpcEndpoint) this.xmlRpcEndpoint).registerService(slave.getName().toString(), service.getName().toString(), service.getUri().toString(), slave.getUri().toString()), new VoidResultFactory());
    }

    public Response<Integer> unregisterService(NodeIdentifier slave, ServiceServer<?, ?> service) {
        return Response.fromListChecked(((MasterXmlRpcEndpoint) this.xmlRpcEndpoint).unregisterService(slave.getName().toString(), service.getName().toString(), service.getUri().toString()), new IntegerResultFactory());
    }

    public Response<List<URI>> registerSubscriber(NodeIdentifier slave, Subscriber<?> subscriber) {
        return Response.fromListChecked(((MasterXmlRpcEndpoint) this.xmlRpcEndpoint).registerSubscriber(slave.getName().toString(), subscriber.getTopicName().toString(), subscriber.getTopicMessageType(), slave.getUri().toString()), new UriListResultFactory());
    }

    public Response<Integer> unregisterSubscriber(NodeIdentifier slave, Subscriber<?> subscriber) {
        return Response.fromListChecked(((MasterXmlRpcEndpoint) this.xmlRpcEndpoint).unregisterSubscriber(slave.getName().toString(), subscriber.getTopicName().toString(), slave.getUri().toString()), new IntegerResultFactory());
    }

    public Response<List<URI>> registerPublisher(PublisherDeclaration publisherDeclaration) {
        String slaveName = publisherDeclaration.getSlaveName().toString();
        String slaveUri = publisherDeclaration.getSlaveUri().toString();
        return Response.fromListChecked(((MasterXmlRpcEndpoint) this.xmlRpcEndpoint).registerPublisher(slaveName, publisherDeclaration.getTopicName().toString(), publisherDeclaration.getTopicMessageType(), slaveUri), new UriListResultFactory());
    }

    public Response<Integer> unregisterPublisher(PublisherIdentifier publisherIdentifier) {
        String slaveName = publisherIdentifier.getNodeName().toString();
        String slaveUri = publisherIdentifier.getNodeUri().toString();
        return Response.fromListChecked(((MasterXmlRpcEndpoint) this.xmlRpcEndpoint).unregisterPublisher(slaveName, publisherIdentifier.getTopicName().toString(), slaveUri), new IntegerResultFactory());
    }

    public Response<URI> lookupNode(GraphName slaveName, String nodeName) {
        return Response.fromListChecked(((MasterXmlRpcEndpoint) this.xmlRpcEndpoint).lookupNode(slaveName.toString(), nodeName), new UriResultFactory());
    }

    public Response<URI> getUri(GraphName slaveName) {
        return Response.fromListChecked(((MasterXmlRpcEndpoint) this.xmlRpcEndpoint).getUri(slaveName.toString()), new UriResultFactory());
    }

    public Response<URI> lookupService(GraphName callerName, String serviceName) {
        return Response.fromListCheckedFailure(((MasterXmlRpcEndpoint) this.xmlRpcEndpoint).lookupService(callerName.toString(), serviceName), new UriResultFactory());
    }

    public Response<List<TopicDeclaration>> getPublishedTopics(GraphName callerName, String subgraph) {
        return Response.fromListChecked(((MasterXmlRpcEndpoint) this.xmlRpcEndpoint).getPublishedTopics(callerName.toString(), subgraph), new TopicListResultFactory());
    }

    public Response<List<TopicType>> getTopicTypes(GraphName callerName) {
        return Response.fromListChecked(((MasterXmlRpcEndpoint) this.xmlRpcEndpoint).getTopicTypes(callerName.toString()), new TopicTypeListResultFactory());
    }

    public Response<SystemState> getSystemState(GraphName callerName) {
        return Response.fromListChecked(((MasterXmlRpcEndpoint) this.xmlRpcEndpoint).getSystemState(callerName.toString()), new SystemStateResultFactory());
    }
}
