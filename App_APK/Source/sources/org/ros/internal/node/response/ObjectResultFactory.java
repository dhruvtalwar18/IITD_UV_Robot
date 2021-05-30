package org.ros.internal.node.response;

public class ObjectResultFactory implements ResultFactory<Object> {
    public Object newFromValue(Object value) {
        return value;
    }
}
