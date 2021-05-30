package org.ros.internal.node.response;

public interface ResultFactory<T> {
    T newFromValue(Object obj);
}
