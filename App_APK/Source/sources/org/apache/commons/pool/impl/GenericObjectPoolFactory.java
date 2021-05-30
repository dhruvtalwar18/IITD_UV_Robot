package org.apache.commons.pool.impl;

import org.apache.commons.pool.ObjectPool;
import org.apache.commons.pool.ObjectPoolFactory;
import org.apache.commons.pool.PoolableObjectFactory;

public class GenericObjectPoolFactory<T> implements ObjectPoolFactory<T> {
    @Deprecated
    protected PoolableObjectFactory<T> _factory;
    @Deprecated
    protected boolean _lifo;
    @Deprecated
    protected int _maxActive;
    @Deprecated
    protected int _maxIdle;
    @Deprecated
    protected long _maxWait;
    @Deprecated
    protected long _minEvictableIdleTimeMillis;
    @Deprecated
    protected int _minIdle;
    @Deprecated
    protected int _numTestsPerEvictionRun;
    @Deprecated
    protected long _softMinEvictableIdleTimeMillis;
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

    public GenericObjectPoolFactory(PoolableObjectFactory<T> factory) {
        this(factory, 8, (byte) 1, -1, 8, 0, false, false, -1, 3, 1800000, false);
    }

    /* JADX WARNING: Illegal instructions before constructor call */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public GenericObjectPoolFactory(org.apache.commons.pool.PoolableObjectFactory<T> r23, org.apache.commons.pool.impl.GenericObjectPool.Config r24) throws java.lang.NullPointerException {
        /*
            r22 = this;
            r0 = r24
            r1 = r22
            r2 = r23
            int r3 = r0.maxActive
            byte r4 = r0.whenExhaustedAction
            long r5 = r0.maxWait
            int r7 = r0.maxIdle
            int r8 = r0.minIdle
            boolean r9 = r0.testOnBorrow
            boolean r10 = r0.testOnReturn
            long r11 = r0.timeBetweenEvictionRunsMillis
            int r13 = r0.numTestsPerEvictionRun
            long r14 = r0.minEvictableIdleTimeMillis
            r20 = r1
            boolean r1 = r0.testWhileIdle
            r16 = r1
            r21 = r2
            long r1 = r0.softMinEvictableIdleTimeMillis
            r17 = r1
            boolean r1 = r0.lifo
            r19 = r1
            r1 = r20
            r2 = r21
            r1.<init>(r2, r3, r4, r5, r7, r8, r9, r10, r11, r13, r14, r16, r17, r19)
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.pool.impl.GenericObjectPoolFactory.<init>(org.apache.commons.pool.PoolableObjectFactory, org.apache.commons.pool.impl.GenericObjectPool$Config):void");
    }

    public GenericObjectPoolFactory(PoolableObjectFactory<T> factory, int maxActive) {
        this(factory, maxActive, (byte) 1, -1, 8, 0, false, false, -1, 3, 1800000, false);
    }

    public GenericObjectPoolFactory(PoolableObjectFactory<T> factory, int maxActive, byte whenExhaustedAction, long maxWait) {
        this(factory, maxActive, whenExhaustedAction, maxWait, 8, 0, false, false, -1, 3, 1800000, false);
    }

    public GenericObjectPoolFactory(PoolableObjectFactory<T> factory, int maxActive, byte whenExhaustedAction, long maxWait, boolean testOnBorrow, boolean testOnReturn) {
        this(factory, maxActive, whenExhaustedAction, maxWait, 8, 0, testOnBorrow, testOnReturn, -1, 3, 1800000, false);
    }

    public GenericObjectPoolFactory(PoolableObjectFactory<T> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle) {
        this(factory, maxActive, whenExhaustedAction, maxWait, maxIdle, 0, false, false, -1, 3, 1800000, false);
    }

    public GenericObjectPoolFactory(PoolableObjectFactory<T> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle, boolean testOnBorrow, boolean testOnReturn) {
        this(factory, maxActive, whenExhaustedAction, maxWait, maxIdle, 0, testOnBorrow, testOnReturn, -1, 3, 1800000, false);
    }

    public GenericObjectPoolFactory(PoolableObjectFactory<T> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle, boolean testOnBorrow, boolean testOnReturn, long timeBetweenEvictionRunsMillis, int numTestsPerEvictionRun, long minEvictableIdleTimeMillis, boolean testWhileIdle) {
        this(factory, maxActive, whenExhaustedAction, maxWait, maxIdle, 0, testOnBorrow, testOnReturn, timeBetweenEvictionRunsMillis, numTestsPerEvictionRun, minEvictableIdleTimeMillis, testWhileIdle, -1);
    }

    public GenericObjectPoolFactory(PoolableObjectFactory<T> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle, int minIdle, boolean testOnBorrow, boolean testOnReturn, long timeBetweenEvictionRunsMillis, int numTestsPerEvictionRun, long minEvictableIdleTimeMillis, boolean testWhileIdle) {
        this(factory, maxActive, whenExhaustedAction, maxWait, maxIdle, minIdle, testOnBorrow, testOnReturn, timeBetweenEvictionRunsMillis, numTestsPerEvictionRun, minEvictableIdleTimeMillis, testWhileIdle, -1);
    }

    public GenericObjectPoolFactory(PoolableObjectFactory<T> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle, int minIdle, boolean testOnBorrow, boolean testOnReturn, long timeBetweenEvictionRunsMillis, int numTestsPerEvictionRun, long minEvictableIdleTimeMillis, boolean testWhileIdle, long softMinEvictableIdleTimeMillis) {
        this(factory, maxActive, whenExhaustedAction, maxWait, maxIdle, minIdle, testOnBorrow, testOnReturn, timeBetweenEvictionRunsMillis, numTestsPerEvictionRun, minEvictableIdleTimeMillis, testWhileIdle, softMinEvictableIdleTimeMillis, true);
    }

    public GenericObjectPoolFactory(PoolableObjectFactory<T> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle, int minIdle, boolean testOnBorrow, boolean testOnReturn, long timeBetweenEvictionRunsMillis, int numTestsPerEvictionRun, long minEvictableIdleTimeMillis, boolean testWhileIdle, long softMinEvictableIdleTimeMillis, boolean lifo) {
        this._maxIdle = 8;
        this._minIdle = 0;
        this._maxActive = 8;
        this._maxWait = -1;
        this._whenExhaustedAction = 1;
        this._testOnBorrow = false;
        this._testOnReturn = false;
        this._testWhileIdle = false;
        this._timeBetweenEvictionRunsMillis = -1;
        this._numTestsPerEvictionRun = 3;
        this._minEvictableIdleTimeMillis = 1800000;
        this._softMinEvictableIdleTimeMillis = 1800000;
        this._lifo = true;
        this._factory = null;
        this._maxIdle = maxIdle;
        this._minIdle = minIdle;
        this._maxActive = maxActive;
        this._maxWait = maxWait;
        this._whenExhaustedAction = whenExhaustedAction;
        this._testOnBorrow = testOnBorrow;
        this._testOnReturn = testOnReturn;
        this._testWhileIdle = testWhileIdle;
        this._timeBetweenEvictionRunsMillis = timeBetweenEvictionRunsMillis;
        this._numTestsPerEvictionRun = numTestsPerEvictionRun;
        this._minEvictableIdleTimeMillis = minEvictableIdleTimeMillis;
        this._softMinEvictableIdleTimeMillis = softMinEvictableIdleTimeMillis;
        this._lifo = lifo;
        this._factory = factory;
    }

    public ObjectPool<T> createPool() {
        return new GenericObjectPool(this._factory, this._maxActive, this._whenExhaustedAction, this._maxWait, this._maxIdle, this._minIdle, this._testOnBorrow, this._testOnReturn, this._timeBetweenEvictionRunsMillis, this._numTestsPerEvictionRun, this._minEvictableIdleTimeMillis, this._testWhileIdle, this._softMinEvictableIdleTimeMillis, this._lifo);
    }

    public int getMaxIdle() {
        return this._maxIdle;
    }

    public int getMinIdle() {
        return this._minIdle;
    }

    public int getMaxActive() {
        return this._maxActive;
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

    public long getSoftMinEvictableIdleTimeMillis() {
        return this._softMinEvictableIdleTimeMillis;
    }

    public boolean getLifo() {
        return this._lifo;
    }

    public PoolableObjectFactory<T> getFactory() {
        return this._factory;
    }
}
