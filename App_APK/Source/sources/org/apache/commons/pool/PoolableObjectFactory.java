package org.apache.commons.pool;

public interface PoolableObjectFactory<T> {
    void activateObject(T t) throws Exception;

    void destroyObject(T t) throws Exception;

    T makeObject() throws Exception;

    void passivateObject(T t) throws Exception;

    boolean validateObject(T t);
}
