package org.ros.node;

import java.util.Collection;
import java.util.concurrent.ScheduledExecutorService;

public interface NodeMainExecutor {
    void execute(NodeMain nodeMain, NodeConfiguration nodeConfiguration);

    void execute(NodeMain nodeMain, NodeConfiguration nodeConfiguration, Collection<NodeListener> collection);

    ScheduledExecutorService getScheduledExecutorService();

    void shutdown();

    void shutdownNodeMain(NodeMain nodeMain);
}
