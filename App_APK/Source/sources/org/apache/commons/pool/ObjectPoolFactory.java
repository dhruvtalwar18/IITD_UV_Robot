package org.apache.commons.pool;

public interface ObjectPoolFactory<T> {
    ObjectPool<T> createPool() throws IllegalStateException;
}
