package org.ros.node;

import java.util.Collection;
import java.util.LinkedList;
import java.util.concurrent.ScheduledExecutorService;
import org.ros.concurrent.SharedScheduledExecutorService;
import org.ros.internal.node.DefaultNode;

public class DefaultNodeFactory implements NodeFactory {
    private final ScheduledExecutorService scheduledExecutorService;

    public DefaultNodeFactory(ScheduledExecutorService scheduledExecutorService2) {
        this.scheduledExecutorService = new SharedScheduledExecutorService(scheduledExecutorService2);
    }

    public Node newNode(NodeConfiguration nodeConfiguration, Collection<NodeListener> listeners) {
        return new DefaultNode(nodeConfiguration, listeners, this.scheduledExecutorService);
    }

    public Node newNode(NodeConfiguration nodeConfiguration) {
        return newNode(nodeConfiguration, new LinkedList());
    }
}
