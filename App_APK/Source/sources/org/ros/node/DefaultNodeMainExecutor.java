package org.ros.node;

import com.google.common.base.Preconditions;
import com.google.common.collect.BiMap;
import com.google.common.collect.HashBiMap;
import com.google.common.collect.HashMultimap;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.collect.Multimap;
import com.google.common.collect.Multimaps;
import java.util.Collection;
import java.util.concurrent.ScheduledExecutorService;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.concurrent.DefaultScheduledExecutorService;
import org.ros.namespace.GraphName;

public class DefaultNodeMainExecutor implements NodeMainExecutor {
    private static final boolean DEBUG = false;
    /* access modifiers changed from: private */
    public static final Log log = LogFactory.getLog(DefaultNodeMainExecutor.class);
    private final Multimap<GraphName, ConnectedNode> connectedNodes = Multimaps.synchronizedMultimap(HashMultimap.create());
    /* access modifiers changed from: private */
    public final NodeFactory nodeFactory;
    /* access modifiers changed from: private */
    public final BiMap<Node, NodeMain> nodeMains = Maps.synchronizedBiMap(HashBiMap.create());
    private final ScheduledExecutorService scheduledExecutorService;

    private class RegistrationListener implements NodeListener {
        private RegistrationListener() {
        }

        public void onStart(ConnectedNode connectedNode) {
            DefaultNodeMainExecutor.this.registerNode(connectedNode);
        }

        public void onShutdown(Node node) {
        }

        public void onShutdownComplete(Node node) {
            DefaultNodeMainExecutor.this.unregisterNode(node);
        }

        public void onError(Node node, Throwable throwable) {
            DefaultNodeMainExecutor.log.error("Node error.", throwable);
            DefaultNodeMainExecutor.this.unregisterNode(node);
        }
    }

    public static NodeMainExecutor newDefault() {
        return newDefault(new DefaultScheduledExecutorService());
    }

    public static NodeMainExecutor newDefault(ScheduledExecutorService executorService) {
        return new DefaultNodeMainExecutor(new DefaultNodeFactory(executorService), executorService);
    }

    private DefaultNodeMainExecutor(NodeFactory nodeFactory2, ScheduledExecutorService scheduledExecutorService2) {
        this.nodeFactory = nodeFactory2;
        this.scheduledExecutorService = scheduledExecutorService2;
        Runtime.getRuntime().addShutdownHook(new Thread(new Runnable() {
            public void run() {
                DefaultNodeMainExecutor.this.shutdown();
            }
        }));
    }

    public ScheduledExecutorService getScheduledExecutorService() {
        return this.scheduledExecutorService;
    }

    public void execute(final NodeMain nodeMain, NodeConfiguration nodeConfiguration, final Collection<NodeListener> nodeListeners) {
        final NodeConfiguration nodeConfigurationCopy = NodeConfiguration.copyOf(nodeConfiguration);
        nodeConfigurationCopy.setDefaultNodeName(nodeMain.getDefaultNodeName());
        Preconditions.checkNotNull(nodeConfigurationCopy.getNodeName(), "Node name not specified.");
        this.scheduledExecutorService.execute(new Runnable() {
            public void run() {
                Collection<NodeListener> nodeListenersCopy = Lists.newArrayList();
                nodeListenersCopy.add(new RegistrationListener());
                nodeListenersCopy.add(nodeMain);
                if (nodeListeners != null) {
                    nodeListenersCopy.addAll(nodeListeners);
                }
                DefaultNodeMainExecutor.this.nodeMains.put(DefaultNodeMainExecutor.this.nodeFactory.newNode(nodeConfigurationCopy, nodeListenersCopy), nodeMain);
            }
        });
    }

    public void execute(NodeMain nodeMain, NodeConfiguration nodeConfiguration) {
        execute(nodeMain, nodeConfiguration, (Collection<NodeListener>) null);
    }

    public void shutdownNodeMain(NodeMain nodeMain) {
        Node node = (Node) this.nodeMains.inverse().get(nodeMain);
        if (node != null) {
            safelyShutdownNode(node);
        }
    }

    public void shutdown() {
        synchronized (this.connectedNodes) {
            for (ConnectedNode connectedNode : this.connectedNodes.values()) {
                safelyShutdownNode(connectedNode);
            }
        }
    }

    private void safelyShutdownNode(Node node) {
        boolean success = true;
        try {
            node.shutdown();
        } catch (Exception e) {
            log.error("Exception thrown while shutting down node.", e);
            unregisterNode(node);
            success = false;
        }
        if (success) {
            log.info("Shutdown successful.");
        }
    }

    /* access modifiers changed from: private */
    public void registerNode(ConnectedNode connectedNode) {
        GraphName nodeName = connectedNode.getName();
        synchronized (this.connectedNodes) {
            for (ConnectedNode illegalConnectedNode : this.connectedNodes.get(nodeName)) {
                System.err.println(String.format("Node name collision. Existing node %s (%s) will be shutdown.", new Object[]{nodeName, illegalConnectedNode.getUri()}));
                illegalConnectedNode.shutdown();
            }
            this.connectedNodes.put(nodeName, connectedNode);
        }
    }

    /* access modifiers changed from: private */
    public void unregisterNode(Node node) {
        node.removeListeners();
        this.connectedNodes.get(node.getName()).remove(node);
        this.nodeMains.remove(node);
    }
}
