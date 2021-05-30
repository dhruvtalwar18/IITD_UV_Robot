package org.apache.commons.pool;

public abstract class BaseKeyedPoolableObjectFactory<K, V> implements KeyedPoolableObjectFactory<K, V> {
    public abstract V makeObject(K k) throws Exception;

    public void destroyObject(K k, V v) throws Exception {
    }

    public boolean validateObject(K k, V v) {
        return true;
    }

    public void activateObject(K k, V v) throws Exception {
    }

    public void passivateObject(K k, V v) throws Exception {
    }
}
