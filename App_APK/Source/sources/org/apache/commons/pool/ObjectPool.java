package org.apache.commons.pool;

import java.util.NoSuchElementException;

public interface ObjectPool<T> {
    void addObject() throws Exception, IllegalStateException, UnsupportedOperationException;

    T borrowObject() throws Exception, NoSuchElementException, IllegalStateException;

    void clear() throws Exception, UnsupportedOperationException;

    void close() throws Exception;

    int getNumActive() throws UnsupportedOperationException;

    int getNumIdle() throws UnsupportedOperationException;

    void invalidateObject(T t) throws Exception;

    void returnObject(T t) throws Exception;

    @Deprecated
    void setFactory(PoolableObjectFactory<T> poolableObjectFactory) throws IllegalStateException, UnsupportedOperationException;
}
