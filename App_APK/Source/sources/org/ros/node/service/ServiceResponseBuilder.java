package org.ros.node.service;

import org.ros.exception.ServiceException;

public interface ServiceResponseBuilder<T, S> {
    void build(T t, S s) throws ServiceException;
}
