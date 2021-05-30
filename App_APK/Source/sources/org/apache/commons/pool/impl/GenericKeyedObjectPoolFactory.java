package org.apache.commons.pool.impl;

import org.apache.commons.pool.KeyedObjectPool;
import org.apache.commons.pool.KeyedObjectPoolFactory;
import org.apache.commons.pool.KeyedPoolableObjectFactory;

public class GenericKeyedObjectPoolFactory<K, V> implements KeyedObjectPoolFactory<K, V> {
    @Deprecated
    protected KeyedPoolableObjectFactory<K, V> _factory;
    @Deprecated
    protected boolean _lifo;
    @Deprecated
    protected int _maxActive;
    @Deprecated
    protected int _maxIdle;
    @Deprecated
    protected int _maxTotal;
    @Deprecated
    protected long _maxWait;
    @Deprecated
    protected long _minEvictableIdleTimeMillis;
    @Deprecated
    protected int _minIdle;
    @Deprecated
    protected int _numTestsPerEvictionRun;
    @Deprecated
    protected boolean _testOnBorrow;
    @Deprecated
    protected boolean _testOnReturn;
    @Deprecated
    protected boolean _testWhileIdle;
    @Deprecated
    protected long _timeBetweenEvictionRunsMillis;
    @Deprecated
    protected byte _whenExhaustedAction;

    public GenericKeyedObjectPoolFactory(KeyedPoolableObjectFactory<K, V> factory) {
        this(factory, 8, (byte) 1, -1, 8, false, false, -1, 3, 1800000, false);
    }

    /* JADX WARNING: Illegal instructions before constructor call */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public GenericKeyedObjectPoolFactory(org.apache.commons.pool.KeyedPoolableObjectFactory<K, V> r22, org.apache.commons.pool.impl.GenericKeyedObjectPool.Config r23) throws java.lang.NullPointerException {
        /*
            r21 = this;
            r0 = r23
            r1 = r21
            r2 = r22
            int r3 = r0.maxActive
            byte r4 = r0.whenExhaustedAction
            long r5 = r0.maxWait
            int r7 = r0.maxIdle
            int r8 = r0.maxTotal
            int r9 = r0.minIdle
            boolean r10 = r0.testOnBorrow
            boolean r11 = r0.testOnReturn
            long r12 = r0.timeBetweenEvictionRunsMillis
            int r14 = r0.numTestsPerEvictionRun
            r19 = r1
            r20 = r2
            long r1 = r0.minEvictableIdleTimeMillis
            r15 = r1
            boolean r1 = r0.testWhileIdle
            r17 = r1
            boolean r1 = r0.lifo
            r18 = r1
            r1 = r19
            r2 = r20
            r1.<init>(r2, r3, r4, r5, r7, r8, r9, r10, r11, r12, r14, r15, r17, r18)
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.pool.impl.GenericKeyedObjectPoolFactory.<init>(org.apache.commons.pool.KeyedPoolableObjectFactory, org.apache.commons.pool.impl.GenericKeyedObjectPool$Config):void");
    }

    public GenericKeyedObjectPoolFactory(KeyedPoolableObjectFactory<K, V> factory, int maxActive) {
        this(factory, maxActive, (byte) 1, -1, 8, -1, false, false, -1, 3, 1800000, false);
    }

    public GenericKeyedObjectPoolFactory(KeyedPoolableObjectFactory<K, V> factory, int maxActive, byte whenExhaustedAction, long maxWait) {
        this(factory, maxActive, whenExhaustedAction, maxWait, 8, -1, false, false, -1, 3, 1800000, false);
    }

    public GenericKeyedObjectPoolFactory(KeyedPoolableObjectFactory<K, V> factory, int maxActive, byte whenExhaustedAction, long maxWait, boolean testOnBorrow, boolean testOnReturn) {
        this(factory, maxActive, whenExhaustedAction, maxWait, 8, -1, testOnBorrow, testOnReturn, -1, 3, 1800000, false);
    }

    public GenericKeyedObjectPoolFactory(KeyedPoolableObjectFactory<K, V> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle) {
        this(factory, maxActive, whenExhaustedAction, maxWait, maxIdle, -1, false, false, -1, 3, 1800000, false);
    }

    public GenericKeyedObjectPoolFactory(KeyedPoolableObjectFactory<K, V> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle, int maxTotal) {
        this(factory, maxActive, whenExhaustedAction, maxWait, maxIdle, maxTotal, false, false, -1, 3, 1800000, false);
    }

    public GenericKeyedObjectPoolFactory(KeyedPoolableObjectFactory<K, V> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle, boolean testOnBorrow, boolean testOnReturn) {
        this(factory, maxActive, whenExhaustedAction, maxWait, maxIdle, -1, testOnBorrow, testOnReturn, -1, 3, 1800000, false);
    }

    public GenericKeyedObjectPoolFactory(KeyedPoolableObjectFactory<K, V> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle, boolean testOnBorrow, boolean testOnReturn, long timeBetweenEvictionRunsMillis, int numTestsPerEvictionRun, long minEvictableIdleTimeMillis, boolean testWhileIdle) {
        this(factory, maxActive, whenExhaustedAction, maxWait, maxIdle, -1, testOnBorrow, testOnReturn, timeBetweenEvictionRunsMillis, numTestsPerEvictionRun, minEvictableIdleTimeMillis, testWhileIdle);
    }

    public GenericKeyedObjectPoolFactory(KeyedPoolableObjectFactory<K, V> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle, int maxTotal, boolean testOnBorrow, boolean testOnReturn, long timeBetweenEvictionRunsMillis, int numTestsPerEvictionRun, long minEvictableIdleTimeMillis, boolean testWhileIdle) {
        this(factory, maxActive, whenExhaustedAction, maxWait, maxIdle, maxTotal, 0, testOnBorrow, testOnReturn, timeBetweenEvictionRunsMillis, numTestsPerEvictionRun, minEvictableIdleTimeMillis, testWhileIdle);
    }

    public GenericKeyedObjectPoolFactory(KeyedPoolableObjectFactory<K, V> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle, int maxTotal, int minIdle, boolean testOnBorrow, boolean testOnReturn, long timeBetweenEvictionRunsMillis, int numTestsPerEvictionRun, long minEvictableIdleTimeMillis, boolean testWhileIdle) {
        this(factory, maxActive, whenExhaustedAction, maxWait, maxIdle, maxTotal, minIdle, testOnBorrow, testOnReturn, timeBetweenEvictionRunsMillis, numTestsPerEvictionRun, minEvictableIdleTimeMillis, testWhileIdle, true);
    }

    public GenericKeyedObjectPoolFactory(KeyedPoolableObjectFactory<K, V> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle, int maxTotal, int minIdle, boolean testOnBorrow, boolean testOnReturn, long timeBetweenEvictionRunsMillis, int numTestsPerEvictionRun, long minEvictableIdleTimeMillis, boolean testWhileIdle, boolean lifo) {
        this._maxIdle = 8;
        this._maxActive = 8;
        this._maxTotal = -1;
        this._minIdle = 0;
        this._maxWait = -1;
        this._whenExhaustedAction = 1;
        this._testOnBorrow = false;
        this._testOnReturn = false;
        this._testWhileIdle = false;
        this._timeBetweenEvictionRunsMillis = -1;
        this._numTestsPerEvictionRun = 3;
        this._minEvictableIdleTimeMillis = 1800000;
        this._factory = null;
        this._lifo = true;
        this._maxIdle = maxIdle;
        this._maxActive = maxActive;
        this._maxTotal = maxTotal;
        this._minIdle = minIdle;
        this._maxWait = maxWait;
        this._whenExhaustedAction = whenExhaustedAction;
        this._testOnBorrow = testOnBorrow;
        this._testOnReturn = testOnReturn;
        this._testWhileIdle = testWhileIdle;
        this._timeBetweenEvictionRunsMillis = timeBetweenEvictionRunsMillis;
        this._numTestsPerEvictionRun = numTestsPerEvictionRun;
        this._minEvictableIdleTimeMillis = minEvictableIdleTimeMillis;
        this._factory = factory;
        this._lifo = lifo;
    }

    public KeyedObjectPool<K, V> createPool() {
        return new GenericKeyedObjectPool(this._factory, this._maxActive, this._whenExhaustedAction, this._maxWait, this._maxIdle, this._maxTotal, this._minIdle, this._testOnBorrow, this._testOnReturn, this._timeBetweenEvictionRunsMillis, this._numTestsPerEvictionRun, this._minEvictableIdleTimeMillis, this._testWhileIdle, this._lifo);
    }

    public int getMaxIdle() {
        return this._maxIdle;
    }

    public int getMaxActive() {
        return this._maxActive;
    }

    public int getMaxTotal() {
        return this._maxTotal;
    }

    public int getMinIdle() {
        return this._minIdle;
    }

    public long getMaxWait() {
        return this._maxWait;
    }

    public byte getWhenExhaustedAction() {
        return this._whenExhaustedAction;
    }

    public boolean getTestOnBorrow() {
        return this._testOnBorrow;
    }

    public boolean getTestOnReturn() {
        return this._testOnReturn;
    }

    public boolean getTestWhileIdle() {
        return this._testWhileIdle;
    }

    public long getTimeBetweenEvictionRunsMillis() {
        return this._timeBetweenEvictionRunsMillis;
    }

    public int getNumTestsPerEvictionRun() {
        return this._numTestsPerEvictionRun;
    }

    public long getMinEvictableIdleTimeMillis() {
        return this._minEvictableIdleTimeMillis;
    }

    public KeyedPoolableObjectFactory<K, V> getFactory() {
        return this._factory;
    }

    public boolean getLifo() {
        return this._lifo;
    }
}
