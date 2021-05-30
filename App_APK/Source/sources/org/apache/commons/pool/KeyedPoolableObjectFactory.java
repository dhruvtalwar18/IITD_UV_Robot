package org.apache.commons.pool;

public interface KeyedPoolableObjectFactory<K, V> {
    void activateObject(K k, V v) throws Exception;

    void destroyObject(K k, V v) throws Exception;

    V makeObject(K k) throws Exception;

    void passivateObject(K k, V v) throws Exception;

    boolean validateObject(K k, V v);
}
