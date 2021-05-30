package org.apache.commons.pool;

public interface KeyedObjectPoolFactory<K, V> {
    KeyedObjectPool<K, V> createPool() throws IllegalStateException;
}
