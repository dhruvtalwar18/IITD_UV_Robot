package org.ros.node;

import java.net.URI;
import java.util.concurrent.ScheduledExecutorService;
import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageFactory;
import org.ros.message.MessageSerializationFactory;
import org.ros.namespace.GraphName;
import org.ros.namespace.NodeNameResolver;

public interface Node {
    void addListener(NodeListener nodeListener);

    void executeCancellableLoop(CancellableLoop cancellableLoop);

    Log getLog();

    URI getMasterUri();

    MessageSerializationFactory getMessageSerializationFactory();

    GraphName getName();

    NodeNameResolver getResolver();

    ScheduledExecutorService getScheduledExecutorService();

    MessageFactory getServiceRequestMessageFactory();

    MessageFactory getServiceResponseMessageFactory();

    MessageFactory getTopicMessageFactory();

    URI getUri();

    void removeListeners();

    GraphName resolveName(String str);

    GraphName resolveName(GraphName graphName);

    void shutdown();
}
