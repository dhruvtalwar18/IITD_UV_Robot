package org.apache.commons.pool.impl;

import java.util.Iterator;
import java.util.NoSuchElementException;
import java.util.Stack;
import org.apache.commons.pool.BaseObjectPool;
import org.apache.commons.pool.ObjectPool;
import org.apache.commons.pool.PoolUtils;
import org.apache.commons.pool.PoolableObjectFactory;

public class StackObjectPool<T> extends BaseObjectPool<T> implements ObjectPool<T> {
    protected static final int DEFAULT_INIT_SLEEPING_CAPACITY = 4;
    protected static final int DEFAULT_MAX_SLEEPING = 8;
    @Deprecated
    protected PoolableObjectFactory<T> _factory;
    @Deprecated
    protected int _maxSleeping;
    @Deprecated
    protected int _numActive;
    @Deprecated
    protected Stack<T> _pool;

    @Deprecated
    public StackObjectPool() {
        this((PoolableObjectFactory) null, 8, 4);
    }

    @Deprecated
    public StackObjectPool(int maxIdle) {
        this((PoolableObjectFactory) null, maxIdle, 4);
    }

    @Deprecated
    public StackObjectPool(int maxIdle, int initIdleCapacity) {
        this((PoolableObjectFactory) null, maxIdle, initIdleCapacity);
    }

    public StackObjectPool(PoolableObjectFactory<T> factory) {
        this(factory, 8, 4);
    }

    public StackObjectPool(PoolableObjectFactory<T> factory, int maxIdle) {
        this(factory, maxIdle, 4);
    }

    public StackObjectPool(PoolableObjectFactory<T> factory, int maxIdle, int initIdleCapacity) {
        this._pool = null;
        this._factory = null;
        int i = 8;
        this._maxSleeping = 8;
        this._numActive = 0;
        this._factory = factory;
        this._maxSleeping = maxIdle >= 0 ? maxIdle : i;
        int initcapacity = initIdleCapacity < 1 ? 4 : initIdleCapacity;
        this._pool = new Stack<>();
        this._pool.ensureCapacity(initcapacity > this._maxSleeping ? this._maxSleeping : initcapacity);
    }

    public synchronized T borrowObject() throws Exception {
        T obj;
        boolean newlyCreated;
        assertOpen();
        obj = null;
        newlyCreated = false;
        while (obj == null) {
            if (!this._pool.empty()) {
                obj = this._pool.pop();
            } else if (this._factory != null) {
                obj = this._factory.makeObject();
                newlyCreated = true;
                if (obj == null) {
                    throw new NoSuchElementException("PoolableObjectFactory.makeObject() returned null.");
                }
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
            T toBeDestroyed = null;
            if (this._pool.size() >= this._maxSleeping) {
                shouldDestroy = true;
                toBeDestroyed = this._pool.remove(0);
            }
            this._pool.push(obj);
            obj = toBeDestroyed;
        }
        notifyAll();
        if (shouldDestroy) {
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

    public synchronized int getNumIdle() {
        return this._pool.size();
    }

    public synchronized int getNumActive() {
        return this._numActive;
    }

    public synchronized void clear() {
        if (this._factory != null) {
            Iterator<T> it = this._pool.iterator();
            while (it.hasNext()) {
                try {
                    this._factory.destroyObject(it.next());
                } catch (Exception e) {
                }
            }
        }
        this._pool.clear();
    }

    public void close() throws Exception {
        super.close();
        clear();
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
                T toBeDestroyed = null;
                if (this._pool.size() >= this._maxSleeping) {
                    shouldDestroy = true;
                    toBeDestroyed = this._pool.remove(0);
                }
                this._pool.push(obj);
                obj = toBeDestroyed;
            }
            notifyAll();
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

    public synchronized PoolableObjectFactory<T> getFactory() {
        return this._factory;
    }

    public int getMaxSleeping() {
        return this._maxSleeping;
    }
}
