package org.apache.commons.pool;

public abstract class BasePoolableObjectFactory<T> implements PoolableObjectFactory<T> {
    public abstract T makeObject() throws Exception;

    public void destroyObject(T t) throws Exception {
    }

    public boolean validateObject(T t) {
        return true;
    }

    public void activateObject(T t) throws Exception {
    }

    public void passivateObject(T t) throws Exception {
    }
}
