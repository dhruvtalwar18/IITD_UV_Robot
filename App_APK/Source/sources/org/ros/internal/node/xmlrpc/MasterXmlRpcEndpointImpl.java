package org.ros.internal.node.xmlrpc;

import com.google.common.collect.Lists;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.ros.exception.RosRuntimeException;
import org.ros.internal.node.response.Response;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.node.server.ParameterServer;
import org.ros.internal.node.server.master.MasterServer;
import org.ros.namespace.GraphName;

public class MasterXmlRpcEndpointImpl implements MasterXmlRpcEndpoint, ParameterServerXmlRpcEndpoint {
    private final MasterServer master;
    private final ParameterServer parameterServer = new ParameterServer();

    public MasterXmlRpcEndpointImpl(MasterServer master2) {
        this.master = master2;
    }

    public List<Object> getPid(String callerId) {
        return Response.newSuccess("server pid", Integer.valueOf(this.master.getPid())).toList();
    }

    public List<Object> getPublishedTopics(String callerId, String subgraph) {
        return Response.newSuccess("current topics", this.master.getPublishedTopics(GraphName.of(callerId), GraphName.of(subgraph))).toList();
    }

    public List<Object> getTopicTypes(String callerId) {
        return Response.newSuccess("topic types", this.master.getTopicTypes(GraphName.of(callerId))).toList();
    }

    public List<Object> getSystemState(String callerId) {
        return Response.newSuccess("current system state", this.master.getSystemState()).toList();
    }

    public List<Object> getUri(String callerId) {
        return Response.newSuccess("Success", this.master.getUri().toString()).toList();
    }

    public List<Object> lookupNode(String callerId, String nodeName) {
        URI nodeSlaveUri = this.master.lookupNode(GraphName.of(nodeName));
        if (nodeSlaveUri != null) {
            return Response.newSuccess("Success", nodeSlaveUri.toString()).toList();
        }
        return Response.newError("No such node", null).toList();
    }

    public List<Object> registerPublisher(String callerId, String topicName, String topicMessageType, String callerSlaveUri) {
        try {
            List<URI> subscribers = this.master.registerPublisher(GraphName.of(callerId), new URI(callerSlaveUri), GraphName.of(topicName), topicMessageType);
            List<String> urls = Lists.newArrayList();
            for (URI uri : subscribers) {
                urls.add(uri.toString());
            }
            return Response.newSuccess("Success", urls).toList();
        } catch (URISyntaxException e) {
            throw new RosRuntimeException(String.format("Improperly formatted URI %s for publisher", new Object[]{callerSlaveUri}), e);
        }
    }

    public List<Object> unregisterPublisher(String callerId, String topicName, String callerSlaveUri) {
        return Response.newSuccess("Success", Integer.valueOf(this.master.unregisterPublisher(GraphName.of(callerId), GraphName.of(topicName)))).toList();
    }

    public List<Object> registerSubscriber(String callerId, String topicName, String topicMessageType, String callerSlaveUri) {
        try {
            List<URI> publishers = this.master.registerSubscriber(GraphName.of(callerId), new URI(callerSlaveUri), GraphName.of(topicName), topicMessageType);
            List<String> urls = Lists.newArrayList();
            for (URI uri : publishers) {
                urls.add(uri.toString());
            }
            return Response.newSuccess("Success", urls).toList();
        } catch (URISyntaxException e) {
            throw new RosRuntimeException(String.format("Improperly formatted URI %s for subscriber", new Object[]{callerSlaveUri}), e);
        }
    }

    public List<Object> unregisterSubscriber(String callerId, String topicName, String callerSlaveUri) {
        return Response.newSuccess("Success", Integer.valueOf(this.master.unregisterSubscriber(GraphName.of(callerId), GraphName.of(topicName)))).toList();
    }

    public List<Object> lookupService(String callerId, String serviceName) {
        URI slaveUri = this.master.lookupService(GraphName.of(serviceName));
        if (slaveUri != null) {
            return Response.newSuccess("Success", slaveUri.toString()).toList();
        }
        return Response.newError("No such service.", null).toList();
    }

    public List<Object> registerService(String callerId, String serviceName, String serviceUri, String callerSlaveUri) {
        try {
            this.master.registerService(GraphName.of(callerId), new URI(callerSlaveUri), GraphName.of(serviceName), new URI(serviceUri));
            return Response.newSuccess("Success", 0).toList();
        } catch (URISyntaxException e) {
            throw new RosRuntimeException((Throwable) e);
        }
    }

    public List<Object> unregisterService(String callerId, String serviceName, String serviceUri) {
        try {
            return Response.newSuccess("Success", Integer.valueOf(this.master.unregisterService(GraphName.of(callerId), GraphName.of(serviceName), new URI(serviceUri)) ? 1 : 0)).toList();
        } catch (URISyntaxException e) {
            throw new RosRuntimeException((Throwable) e);
        }
    }

    public List<Object> setParam(String callerId, String key, Boolean value) {
        this.parameterServer.set(GraphName.of(key), value.booleanValue());
        return Response.newSuccess("Success", null).toList();
    }

    public List<Object> setParam(String callerId, String key, Integer value) {
        this.parameterServer.set(GraphName.of(key), value.intValue());
        return Response.newSuccess("Success", null).toList();
    }

    public List<Object> setParam(String callerId, String key, Double value) {
        this.parameterServer.set(GraphName.of(key), value.doubleValue());
        return Response.newSuccess("Success", null).toList();
    }

    public List<Object> setParam(String callerId, String key, String value) {
        this.parameterServer.set(GraphName.of(key), value);
        return Response.newSuccess("Success", null).toList();
    }

    public List<Object> setParam(String callerId, String key, List<?> value) {
        this.parameterServer.set(GraphName.of(key), value);
        return Response.newSuccess("Success", null).toList();
    }

    public List<Object> setParam(String callerId, String key, Map<?, ?> value) {
        this.parameterServer.set(GraphName.of(key), value);
        return Response.newSuccess("Success", null).toList();
    }

    public List<Object> getParam(String callerId, String key) {
        Object value = this.parameterServer.get(GraphName.of(key));
        if (value != null) {
            return Response.newSuccess("Success", value).toList();
        }
        return Response.newError("Parameter \"" + key + "\" is not set.", null).toList();
    }

    public List<Object> searchParam(String callerId, String key) {
        GraphName ns = GraphName.of(callerId);
        Object value = this.parameterServer.search(ns, GraphName.of(key));
        if (value != null) {
            return Response.newSuccess("Success", value.toString()).toList();
        }
        return Response.newError("Parameter \"" + key + "\" in namespace \"" + ns.toString() + "\" not found in parameter server", null).toList();
    }

    public List<Object> subscribeParam(String callerId, String callerSlaveUri, String key) {
        this.parameterServer.subscribe(GraphName.of(key), NodeIdentifier.forNameAndUri(callerId, callerSlaveUri));
        Object value = this.parameterServer.get(GraphName.of(key));
        if (value == null) {
            value = new HashMap();
        }
        return Response.newSuccess("Success", value).toList();
    }

    public List<Object> unsubscribeParam(String callerId, String callerSlaveUri, String key) {
        throw new UnsupportedOperationException();
    }

    public List<Object> deleteParam(String callerId, String key) {
        this.parameterServer.delete(GraphName.of(key));
        return Response.newSuccess("Success", null).toList();
    }

    public List<Object> hasParam(String callerId, String key) {
        return Response.newSuccess("Success", Boolean.valueOf(this.parameterServer.has(GraphName.of(key)))).toList();
    }

    public List<Object> getParamNames(String callerId) {
        Collection<GraphName> names = this.parameterServer.getNames();
        List<String> stringNames = Lists.newArrayList();
        for (GraphName name : names) {
            stringNames.add(name.toString());
        }
        return Response.newSuccess("Success", stringNames).toList();
    }
}
