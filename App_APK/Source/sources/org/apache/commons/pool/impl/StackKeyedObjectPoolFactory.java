package org.apache.commons.pool.impl;

import org.apache.commons.pool.KeyedObjectPool;
import org.apache.commons.pool.KeyedObjectPoolFactory;
import org.apache.commons.pool.KeyedPoolableObjectFactory;

public class StackKeyedObjectPoolFactory<K, V> implements KeyedObjectPoolFactory<K, V> {
    @Deprecated
    protected KeyedPoolableObjectFactory<K, V> _factory;
    @Deprecated
    protected int _initCapacity;
    @Deprecated
    protected int _maxSleeping;

    public StackKeyedObjectPoolFactory() {
        this((KeyedPoolableObjectFactory) null, 8, 4);
    }

    public StackKeyedObjectPoolFactory(int maxSleeping) {
        this((KeyedPoolableObjectFactory) null, maxSleeping, 4);
    }

    public StackKeyedObjectPoolFactory(int maxSleeping, int initialCapacity) {
        this((KeyedPoolableObjectFactory) null, maxSleeping, initialCapacity);
    }

    public StackKeyedObjectPoolFactory(KeyedPoolableObjectFactory<K, V> factory) {
        this(factory, 8, 4);
    }

    public StackKeyedObjectPoolFactory(KeyedPoolableObjectFactory<K, V> factory, int maxSleeping) {
        this(factory, maxSleeping, 4);
    }

    public StackKeyedObjectPoolFactory(KeyedPoolableObjectFactory<K, V> factory, int maxSleeping, int initialCapacity) {
        this._factory = null;
        this._maxSleeping = 8;
        this._initCapacity = 4;
        this._factory = factory;
        this._maxSleeping = maxSleeping;
        this._initCapacity = initialCapacity;
    }

    public KeyedObjectPool<K, V> createPool() {
        return new StackKeyedObjectPool(this._factory, this._maxSleeping, this._initCapacity);
    }

    public KeyedPoolableObjectFactory<K, V> getFactory() {
        return this._factory;
    }

    public int getMaxSleeping() {
        return this._maxSleeping;
    }

    public int getInitialCapacity() {
        return this._initCapacity;
    }
}
