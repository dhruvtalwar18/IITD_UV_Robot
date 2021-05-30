package org.ros.internal.node.client;

import com.google.common.collect.Lists;
import java.net.URI;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import org.ros.internal.node.response.IntegerResultFactory;
import org.ros.internal.node.response.ProtocolDescriptionResultFactory;
import org.ros.internal.node.response.Response;
import org.ros.internal.node.response.TopicListResultFactory;
import org.ros.internal.node.response.UriResultFactory;
import org.ros.internal.node.response.VoidResultFactory;
import org.ros.internal.node.topic.TopicDeclaration;
import org.ros.internal.node.xmlrpc.SlaveXmlRpcEndpoint;
import org.ros.internal.transport.ProtocolDescription;
import org.ros.namespace.GraphName;

public class SlaveClient extends Client<SlaveXmlRpcEndpoint> {
    private final GraphName nodeName;

    public /* bridge */ /* synthetic */ URI getRemoteUri() {
        return super.getRemoteUri();
    }

    public SlaveClient(GraphName nodeName2, URI uri) {
        super(uri, SlaveXmlRpcEndpoint.class);
        this.nodeName = nodeName2;
    }

    public List<Object> getBusStats() {
        throw new UnsupportedOperationException();
    }

    public List<Object> getBusInfo() {
        throw new UnsupportedOperationException();
    }

    public Response<URI> getMasterUri() {
        return Response.fromListChecked(((SlaveXmlRpcEndpoint) this.xmlRpcEndpoint).getMasterUri(this.nodeName.toString()), new UriResultFactory());
    }

    public Response<Void> shutdown(String message) {
        return Response.fromListChecked(((SlaveXmlRpcEndpoint) this.xmlRpcEndpoint).shutdown("/master", message), new VoidResultFactory());
    }

    public Response<Integer> getPid() {
        return Response.fromListChecked(((SlaveXmlRpcEndpoint) this.xmlRpcEndpoint).getPid(this.nodeName.toString()), new IntegerResultFactory());
    }

    public Response<List<TopicDeclaration>> getSubscriptions() {
        return Response.fromListChecked(((SlaveXmlRpcEndpoint) this.xmlRpcEndpoint).getSubscriptions(this.nodeName.toString()), new TopicListResultFactory());
    }

    public Response<List<TopicDeclaration>> getPublications() {
        return Response.fromListChecked(((SlaveXmlRpcEndpoint) this.xmlRpcEndpoint).getPublications(this.nodeName.toString()), new TopicListResultFactory());
    }

    public Response<Void> paramUpdate(GraphName name, boolean value) {
        return Response.fromListChecked(((SlaveXmlRpcEndpoint) this.xmlRpcEndpoint).paramUpdate(this.nodeName.toString(), name.toString(), value), new VoidResultFactory());
    }

    public Response<Void> paramUpdate(GraphName name, char value) {
        return Response.fromListChecked(((SlaveXmlRpcEndpoint) this.xmlRpcEndpoint).paramUpdate(this.nodeName.toString(), name.toString(), value), new VoidResultFactory());
    }

    public Response<Void> paramUpdate(GraphName name, int value) {
        return Response.fromListChecked(((SlaveXmlRpcEndpoint) this.xmlRpcEndpoint).paramUpdate(this.nodeName.toString(), name.toString(), value), new VoidResultFactory());
    }

    public Response<Void> paramUpdate(GraphName name, double value) {
        return Response.fromListChecked(((SlaveXmlRpcEndpoint) this.xmlRpcEndpoint).paramUpdate(this.nodeName.toString(), name.toString(), value), new VoidResultFactory());
    }

    public Response<Void> paramUpdate(GraphName name, String value) {
        return Response.fromListChecked(((SlaveXmlRpcEndpoint) this.xmlRpcEndpoint).paramUpdate(this.nodeName.toString(), name.toString(), value), new VoidResultFactory());
    }

    public Response<Void> paramUpdate(GraphName name, List<?> value) {
        return Response.fromListChecked(((SlaveXmlRpcEndpoint) this.xmlRpcEndpoint).paramUpdate(this.nodeName.toString(), name.toString(), value), new VoidResultFactory());
    }

    public Response<Void> paramUpdate(GraphName name, Map<?, ?> value) {
        return Response.fromListChecked(((SlaveXmlRpcEndpoint) this.xmlRpcEndpoint).paramUpdate(this.nodeName.toString(), name.toString(), value), new VoidResultFactory());
    }

    public Response<Void> publisherUpdate(GraphName topic, Collection<URI> publisherUris) {
        List<String> publishers = Lists.newArrayList();
        for (URI uri : publisherUris) {
            publishers.add(uri.toString());
        }
        return Response.fromListChecked(((SlaveXmlRpcEndpoint) this.xmlRpcEndpoint).publisherUpdate(this.nodeName.toString(), topic.toString(), publishers.toArray()), new VoidResultFactory());
    }

    public Response<ProtocolDescription> requestTopic(GraphName topic, Collection<String> requestedProtocols) {
        return Response.fromListChecked(((SlaveXmlRpcEndpoint) this.xmlRpcEndpoint).requestTopic(this.nodeName.toString(), topic.toString(), new Object[][]{requestedProtocols.toArray()}), new ProtocolDescriptionResultFactory());
    }
}
