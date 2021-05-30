package org.apache.commons.pool;

public abstract class BaseObjectPool<T> implements ObjectPool<T> {
    private volatile boolean closed = false;

    public abstract T borrowObject() throws Exception;

    public abstract void invalidateObject(T t) throws Exception;

    public abstract void returnObject(T t) throws Exception;

    public int getNumIdle() throws UnsupportedOperationException {
        return -1;
    }

    public int getNumActive() throws UnsupportedOperationException {
        return -1;
    }

    public void clear() throws Exception, UnsupportedOperationException {
        throw new UnsupportedOperationException();
    }

    public void addObject() throws Exception, UnsupportedOperationException {
        throw new UnsupportedOperationException();
    }

    public void close() throws Exception {
        this.closed = true;
    }

    @Deprecated
    public void setFactory(PoolableObjectFactory<T> poolableObjectFactory) throws IllegalStateException, UnsupportedOperationException {
        throw new UnsupportedOperationException();
    }

    public final boolean isClosed() {
        return this.closed;
    }

    /* access modifiers changed from: protected */
    public final void assertOpen() throws IllegalStateException {
        if (isClosed()) {
            throw new IllegalStateException("Pool not open");
        }
    }
}
