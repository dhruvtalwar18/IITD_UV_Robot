package org.ros.node;

import java.util.Collection;

public interface NodeFactory {
    Node newNode(NodeConfiguration nodeConfiguration);

    Node newNode(NodeConfiguration nodeConfiguration, Collection<NodeListener> collection);
}
