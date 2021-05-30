package org.ros.internal.node.client;

import com.google.common.collect.Lists;
import java.net.URI;
import java.util.List;
import java.util.Map;
import org.ros.internal.node.response.BooleanResultFactory;
import org.ros.internal.node.response.IntegerResultFactory;
import org.ros.internal.node.response.ObjectResultFactory;
import org.ros.internal.node.response.Response;
import org.ros.internal.node.response.StringListResultFactory;
import org.ros.internal.node.response.StringResultFactory;
import org.ros.internal.node.response.VoidResultFactory;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.node.xmlrpc.ParameterServerXmlRpcEndpoint;
import org.ros.namespace.GraphName;

public class ParameterClient extends Client<ParameterServerXmlRpcEndpoint> {
    private final NodeIdentifier nodeIdentifier;
    private final String nodeName;

    public /* bridge */ /* synthetic */ URI getRemoteUri() {
        return super.getRemoteUri();
    }

    public ParameterClient(NodeIdentifier nodeIdentifier2, URI uri) {
        super(uri, ParameterServerXmlRpcEndpoint.class);
        this.nodeIdentifier = nodeIdentifier2;
        this.nodeName = nodeIdentifier2.getName().toString();
    }

    public Response<Object> getParam(GraphName parameterName) {
        return Response.fromListCheckedFailure(((ParameterServerXmlRpcEndpoint) this.xmlRpcEndpoint).getParam(this.nodeName, parameterName.toString()), new ObjectResultFactory());
    }

    public Response<Void> setParam(GraphName parameterName, Boolean parameterValue) {
        return Response.fromListChecked(((ParameterServerXmlRpcEndpoint) this.xmlRpcEndpoint).setParam(this.nodeName, parameterName.toString(), parameterValue), new VoidResultFactory());
    }

    public Response<Void> setParam(GraphName parameterName, Integer parameterValue) {
        return Response.fromListChecked(((ParameterServerXmlRpcEndpoint) this.xmlRpcEndpoint).setParam(this.nodeName, parameterName.toString(), parameterValue), new VoidResultFactory());
    }

    public Response<Void> setParam(GraphName parameterName, Double parameterValue) {
        return Response.fromListChecked(((ParameterServerXmlRpcEndpoint) this.xmlRpcEndpoint).setParam(this.nodeName, parameterName.toString(), parameterValue), new VoidResultFactory());
    }

    public Response<Void> setParam(GraphName parameterName, String parameterValue) {
        return Response.fromListChecked(((ParameterServerXmlRpcEndpoint) this.xmlRpcEndpoint).setParam(this.nodeName, parameterName.toString(), parameterValue), new VoidResultFactory());
    }

    public Response<Void> setParam(GraphName parameterName, List<?> parameterValue) {
        return Response.fromListChecked(((ParameterServerXmlRpcEndpoint) this.xmlRpcEndpoint).setParam(this.nodeName, parameterName.toString(), parameterValue), new VoidResultFactory());
    }

    public Response<Void> setParam(GraphName parameterName, Map<?, ?> parameterValue) {
        return Response.fromListChecked(((ParameterServerXmlRpcEndpoint) this.xmlRpcEndpoint).setParam(this.nodeName, parameterName.toString(), parameterValue), new VoidResultFactory());
    }

    public Response<GraphName> searchParam(GraphName parameterName) {
        Response<String> response = Response.fromListCheckedFailure(((ParameterServerXmlRpcEndpoint) this.xmlRpcEndpoint).searchParam(this.nodeName, parameterName.toString()), new StringResultFactory());
        return new Response<>(response.getStatusCode(), response.getStatusMessage(), GraphName.of(response.getResult()));
    }

    public Response<Object> subscribeParam(GraphName parameterName) {
        return Response.fromListChecked(((ParameterServerXmlRpcEndpoint) this.xmlRpcEndpoint).subscribeParam(this.nodeName, this.nodeIdentifier.getUri().toString(), parameterName.toString()), new ObjectResultFactory());
    }

    public Response<Integer> unsubscribeParam(GraphName parameterName) {
        return Response.fromListChecked(((ParameterServerXmlRpcEndpoint) this.xmlRpcEndpoint).unsubscribeParam(this.nodeName, this.nodeIdentifier.getUri().toString(), parameterName.toString()), new IntegerResultFactory());
    }

    public Response<Boolean> hasParam(GraphName parameterName) {
        return Response.fromListChecked(((ParameterServerXmlRpcEndpoint) this.xmlRpcEndpoint).hasParam(this.nodeName, parameterName.toString()), new BooleanResultFactory());
    }

    public Response<Void> deleteParam(GraphName parameterName) {
        return Response.fromListChecked(((ParameterServerXmlRpcEndpoint) this.xmlRpcEndpoint).deleteParam(this.nodeName, parameterName.toString()), new VoidResultFactory());
    }

    public Response<List<GraphName>> getParamNames() {
        Response<List<String>> response = Response.fromListChecked(((ParameterServerXmlRpcEndpoint) this.xmlRpcEndpoint).getParamNames(this.nodeName), new StringListResultFactory());
        List<GraphName> graphNames = Lists.newArrayList();
        for (String name : response.getResult()) {
            graphNames.add(GraphName.of(name));
        }
        return new Response<>(response.getStatusCode(), response.getStatusMessage(), graphNames);
    }
}
