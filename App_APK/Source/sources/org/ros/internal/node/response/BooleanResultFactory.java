package org.ros.internal.node.response;

public class BooleanResultFactory implements ResultFactory<Boolean> {
    public Boolean newFromValue(Object value) {
        return (Boolean) value;
    }
}
