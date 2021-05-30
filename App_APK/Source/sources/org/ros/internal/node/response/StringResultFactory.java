package org.ros.internal.node.response;

public class StringResultFactory implements ResultFactory<String> {
    public String newFromValue(Object value) {
        return (String) value;
    }
}
