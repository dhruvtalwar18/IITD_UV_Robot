package org.ros.node;

public interface NodeListener {
    void onError(Node node, Throwable th);

    void onShutdown(Node node);

    void onShutdownComplete(Node node);

    void onStart(ConnectedNode connectedNode);
}
