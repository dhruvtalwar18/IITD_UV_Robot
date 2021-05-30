package org.ros.internal.node.server.master;

public interface MasterRegistrationListener {
    void onNodeReplacement(NodeRegistrationInfo nodeRegistrationInfo);
}
