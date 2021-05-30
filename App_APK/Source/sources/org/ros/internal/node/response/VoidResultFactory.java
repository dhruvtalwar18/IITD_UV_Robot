package org.ros.internal.node.response;

public class VoidResultFactory implements ResultFactory<Void> {
    public Void newFromValue(Object value) {
        return null;
    }
}
