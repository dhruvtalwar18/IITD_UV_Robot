package org.apache.commons.pool;

public abstract class BaseKeyedObjectPool<K, V> implements KeyedObjectPool<K, V> {
    private volatile boolean closed = false;

    public abstract V borrowObject(K k) throws Exception;

    public abstract void invalidateObject(K k, V v) throws Exception;

    public abstract void returnObject(K k, V v) throws Exception;

    public void addObject(K k) throws Exception, UnsupportedOperationException {
        throw new UnsupportedOperationException();
    }

    public int getNumIdle(K k) throws UnsupportedOperationException {
        return -1;
    }

    public int getNumActive(K k) throws UnsupportedOperationException {
        return -1;
    }

    public int getNumIdle() throws UnsupportedOperationException {
        return -1;
    }

    public int getNumActive() throws UnsupportedOperationException {
        return -1;
    }

    public void clear() throws Exception, UnsupportedOperationException {
        throw new UnsupportedOperationException();
    }

    public void clear(K k) throws Exception, UnsupportedOperationException {
        throw new UnsupportedOperationException();
    }

    public void close() throws Exception {
        this.closed = true;
    }

    @Deprecated
    public void setFactory(KeyedPoolableObjectFactory<K, V> keyedPoolableObjectFactory) throws IllegalStateException, UnsupportedOperationException {
        throw new UnsupportedOperationException();
    }

    /* access modifiers changed from: protected */
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
