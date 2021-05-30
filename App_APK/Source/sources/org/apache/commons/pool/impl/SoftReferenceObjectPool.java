package org.apache.commons.pool.impl;

import java.lang.ref.Reference;
import java.lang.ref.ReferenceQueue;
import java.lang.ref.SoftReference;
import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import org.apache.commons.pool.BaseObjectPool;
import org.apache.commons.pool.ObjectPool;
import org.apache.commons.pool.PoolUtils;
import org.apache.commons.pool.PoolableObjectFactory;

public class SoftReferenceObjectPool<T> extends BaseObjectPool<T> implements ObjectPool<T> {
    private PoolableObjectFactory<T> _factory;
    private int _numActive;
    private final List<SoftReference<T>> _pool;
    private final ReferenceQueue<T> refQueue;

    @Deprecated
    public SoftReferenceObjectPool() {
        this._factory = null;
        this.refQueue = new ReferenceQueue<>();
        this._numActive = 0;
        this._pool = new ArrayList();
        this._factory = null;
    }

    public SoftReferenceObjectPool(PoolableObjectFactory<T> factory) {
        this._factory = null;
        this.refQueue = new ReferenceQueue<>();
        this._numActive = 0;
        this._pool = new ArrayList();
        this._factory = factory;
    }

    @Deprecated
    public SoftReferenceObjectPool(PoolableObjectFactory<T> factory, int initSize) throws Exception, IllegalArgumentException {
        this._factory = null;
        this.refQueue = new ReferenceQueue<>();
        this._numActive = 0;
        if (factory != null) {
            this._pool = new ArrayList(initSize);
            this._factory = factory;
            PoolUtils.prefill(this, initSize);
            return;
        }
        throw new IllegalArgumentException("factory required to prefill the pool.");
    }

    public synchronized T borrowObject() throws Exception {
        T obj;
        boolean newlyCreated;
        assertOpen();
        obj = null;
        newlyCreated = false;
        while (obj == null) {
            if (!this._pool.isEmpty()) {
                SoftReference<T> ref = this._pool.remove(this._pool.size() - 1);
                obj = ref.get();
                ref.clear();
            } else if (this._factory != null) {
                newlyCreated = true;
                obj = this._factory.makeObject();
            } else {
                throw new NoSuchElementException();
            }
            if (!(this._factory == null || obj == null)) {
                try {
                    this._factory.activateObject(obj);
                    if (!this._factory.validateObject(obj)) {
                        throw new Exception("ValidateObject failed");
                    }
                } catch (Throwable t2) {
                    PoolUtils.checkRethrow(t2);
                }
            }
        }
        this._numActive++;
        return obj;
        obj = null;
        if (newlyCreated) {
            throw new NoSuchElementException("Could not create a validated object, cause: " + t.getMessage());
        }
    }

    public synchronized void returnObject(T obj) throws Exception {
        boolean success = !isClosed();
        if (this._factory != null) {
            if (!this._factory.validateObject(obj)) {
                success = false;
            } else {
                try {
                    this._factory.passivateObject(obj);
                } catch (Exception e) {
                    success = false;
                }
            }
        }
        boolean shouldDestroy = !success;
        this._numActive--;
        if (success) {
            this._pool.add(new SoftReference(obj, this.refQueue));
        }
        notifyAll();
        if (shouldDestroy && this._factory != null) {
            try {
                this._factory.destroyObject(obj);
            } catch (Exception e2) {
            }
        }
    }

    public synchronized void invalidateObject(T obj) throws Exception {
        this._numActive--;
        if (this._factory != null) {
            this._factory.destroyObject(obj);
        }
        notifyAll();
    }

    public synchronized void addObject() throws Exception {
        assertOpen();
        if (this._factory != null) {
            T obj = this._factory.makeObject();
            boolean success = true;
            if (!this._factory.validateObject(obj)) {
                success = false;
            } else {
                this._factory.passivateObject(obj);
            }
            boolean shouldDestroy = !success;
            if (success) {
                this._pool.add(new SoftReference(obj, this.refQueue));
                notifyAll();
            }
            if (shouldDestroy) {
                try {
                    this._factory.destroyObject(obj);
                } catch (Exception e) {
                }
            }
        } else {
            throw new IllegalStateException("Cannot add objects without a factory.");
        }
    }

    public synchronized int getNumIdle() {
        pruneClearedReferences();
        return this._pool.size();
    }

    public synchronized int getNumActive() {
        return this._numActive;
    }

    public synchronized void clear() {
        if (this._factory != null) {
            for (SoftReference<T> softReference : this._pool) {
                try {
                    T obj = softReference.get();
                    if (obj != null) {
                        this._factory.destroyObject(obj);
                    }
                } catch (Exception e) {
                }
            }
        }
        this._pool.clear();
        pruneClearedReferences();
    }

    public void close() throws Exception {
        super.close();
        clear();
    }

    @Deprecated
    public synchronized void setFactory(PoolableObjectFactory<T> factory) throws IllegalStateException {
        assertOpen();
        if (getNumActive() <= 0) {
            clear();
            this._factory = factory;
        } else {
            throw new IllegalStateException("Objects are already active");
        }
    }

    private void pruneClearedReferences() {
        while (true) {
            Reference<? extends T> poll = this.refQueue.poll();
            Reference<? extends T> ref = poll;
            if (poll != null) {
                try {
                    this._pool.remove(ref);
                } catch (UnsupportedOperationException e) {
                }
            } else {
                return;
            }
        }
    }

    public synchronized PoolableObjectFactory<T> getFactory() {
        return this._factory;
    }
}
