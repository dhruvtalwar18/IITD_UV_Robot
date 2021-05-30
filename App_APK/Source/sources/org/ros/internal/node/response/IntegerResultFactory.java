package org.ros.internal.node.response;

public class IntegerResultFactory implements ResultFactory<Integer> {
    public Integer newFromValue(Object value) {
        return (Integer) value;
    }
}
