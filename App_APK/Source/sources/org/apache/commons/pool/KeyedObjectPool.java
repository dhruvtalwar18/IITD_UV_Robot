package org.apache.commons.pool;

import java.util.NoSuchElementException;

public interface KeyedObjectPool<K, V> {
    void addObject(K k) throws Exception, IllegalStateException, UnsupportedOperationException;

    V borrowObject(K k) throws Exception, NoSuchElementException, IllegalStateException;

    void clear() throws Exception, UnsupportedOperationException;

    void clear(K k) throws Exception, UnsupportedOperationException;

    void close() throws Exception;

    int getNumActive() throws UnsupportedOperationException;

    int getNumActive(K k) throws UnsupportedOperationException;

    int getNumIdle() throws UnsupportedOperationException;

    int getNumIdle(K k) throws UnsupportedOperationException;

    void invalidateObject(K k, V v) throws Exception;

    void returnObject(K k, V v) throws Exception;

    @Deprecated
    void setFactory(KeyedPoolableObjectFactory<K, V> keyedPoolableObjectFactory) throws IllegalStateException, UnsupportedOperationException;
}
