package org.ros.internal.node.xmlrpc;

import com.google.common.collect.Lists;
import com.google.common.collect.Sets;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Vector;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.internal.node.response.Response;
import org.ros.internal.node.response.StatusCode;
import org.ros.internal.node.server.ServerException;
import org.ros.internal.node.server.SlaveServer;
import org.ros.internal.node.topic.DefaultPublisher;
import org.ros.internal.node.topic.DefaultSubscriber;
import org.ros.internal.transport.ProtocolDescription;
import org.ros.namespace.GraphName;

public class SlaveXmlRpcEndpointImpl implements SlaveXmlRpcEndpoint {
    private static final boolean DEBUG = false;
    private static final Log log = LogFactory.getLog(SlaveXmlRpcEndpointImpl.class);
    private final SlaveServer slave;

    public SlaveXmlRpcEndpointImpl(SlaveServer slave2) {
        this.slave = slave2;
    }

    public List<Object> getBusStats(String callerId) {
        return this.slave.getBusStats(callerId);
    }

    public List<Object> getBusInfo(String callerId) {
        return Response.newSuccess("bus info", this.slave.getBusInfo(callerId)).toList();
    }

    public List<Object> getMasterUri(String callerId) {
        return new Response(StatusCode.SUCCESS, "", this.slave.getMasterUri().toString()).toList();
    }

    public List<Object> shutdown(String callerId, String message) {
        Log log2 = log;
        log2.info("Shutdown requested by " + callerId + " with message \"" + message + "\"");
        this.slave.shutdown();
        return Response.newSuccess("Shutdown successful.", null).toList();
    }

    public List<Object> getPid(String callerId) {
        try {
            int pid = this.slave.getPid();
            return Response.newSuccess("PID: " + pid, Integer.valueOf(pid)).toList();
        } catch (UnsupportedOperationException e) {
            return Response.newFailure("Cannot retrieve PID on this platform.", -1).toList();
        }
    }

    public List<Object> getSubscriptions(String callerId) {
        Collection<DefaultSubscriber<?>> subscribers = this.slave.getSubscriptions();
        List<List<String>> subscriptions = Lists.newArrayList();
        for (DefaultSubscriber<?> subscriber : subscribers) {
            subscriptions.add(subscriber.getTopicDeclarationAsList());
        }
        return Response.newSuccess("Success", subscriptions).toList();
    }

    public List<Object> getPublications(String callerId) {
        Collection<DefaultPublisher<?>> publishers = this.slave.getPublications();
        List<List<String>> publications = Lists.newArrayList();
        for (DefaultPublisher<?> publisher : publishers) {
            publications.add(publisher.getTopicDeclarationAsList());
        }
        return Response.newSuccess("Success", publications).toList();
    }

    private List<Object> parameterUpdate(String parameterName, Object parameterValue) {
        if (this.slave.paramUpdate(GraphName.of(parameterName), parameterValue) > 0) {
            return Response.newSuccess("Success", null).toList();
        }
        return Response.newError("No subscribers for parameter key \"" + parameterName + "\".", null).toList();
    }

    public List<Object> paramUpdate(String callerId, String key, boolean value) {
        return parameterUpdate(key, Boolean.valueOf(value));
    }

    public List<Object> paramUpdate(String callerId, String key, char value) {
        return parameterUpdate(key, Character.valueOf(value));
    }

    public List<Object> paramUpdate(String callerId, String key, byte value) {
        return parameterUpdate(key, Byte.valueOf(value));
    }

    public List<Object> paramUpdate(String callerId, String key, short value) {
        return parameterUpdate(key, Short.valueOf(value));
    }

    public List<Object> paramUpdate(String callerId, String key, int value) {
        return parameterUpdate(key, Integer.valueOf(value));
    }

    public List<Object> paramUpdate(String callerId, String key, double value) {
        return parameterUpdate(key, Double.valueOf(value));
    }

    public List<Object> paramUpdate(String callerId, String key, String value) {
        return parameterUpdate(key, value);
    }

    public List<Object> paramUpdate(String callerId, String key, List<?> value) {
        return parameterUpdate(key, value);
    }

    public List<Object> paramUpdate(String callerId, String key, Vector<?> value) {
        return parameterUpdate(key, value);
    }

    public List<Object> paramUpdate(String callerId, String key, Map<?, ?> value) {
        return parameterUpdate(key, value);
    }

    public List<Object> publisherUpdate(String callerId, String topicName, Object[] publishers) {
        try {
            ArrayList<URI> publisherUris = new ArrayList<>(publishers.length);
            for (Object publisher : publishers) {
                URI uri = new URI(publisher.toString());
                if (!uri.getScheme().equals("http") && !uri.getScheme().equals("https")) {
                    return Response.newError("Unknown URI scheme sent in update.", 0).toList();
                }
                publisherUris.add(uri);
            }
            this.slave.publisherUpdate(callerId, topicName, publisherUris);
            return Response.newSuccess("Publisher update received.", 0).toList();
        } catch (URISyntaxException e) {
            return Response.newError("Invalid URI sent in update.", 0).toList();
        }
    }

    public List<Object> requestTopic(String callerId, String topic, Object[] protocols) {
        Set<String> requestedProtocols = Sets.newHashSet();
        for (Object[] objArr : protocols) {
            requestedProtocols.add((String) objArr[0]);
        }
        try {
            ProtocolDescription protocol = this.slave.requestTopic(topic, requestedProtocols);
            return Response.newSuccess(protocol.toString(), protocol.toList()).toList();
        } catch (ServerException e) {
            return Response.newError(e.getMessage(), null).toList();
        }
    }
}
