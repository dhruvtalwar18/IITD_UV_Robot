package org.apache.commons.pool.impl;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.TimerTask;
import org.apache.commons.pool.BaseObjectPool;
import org.apache.commons.pool.ObjectPool;
import org.apache.commons.pool.PoolableObjectFactory;
import org.apache.commons.pool.impl.GenericKeyedObjectPool;

public class GenericObjectPool<T> extends BaseObjectPool<T> implements ObjectPool<T> {
    public static final boolean DEFAULT_LIFO = true;
    public static final int DEFAULT_MAX_ACTIVE = 8;
    public static final int DEFAULT_MAX_IDLE = 8;
    public static final long DEFAULT_MAX_WAIT = -1;
    public static final long DEFAULT_MIN_EVICTABLE_IDLE_TIME_MILLIS = 1800000;
    public static final int DEFAULT_MIN_IDLE = 0;
    public static final int DEFAULT_NUM_TESTS_PER_EVICTION_RUN = 3;
    public static final long DEFAULT_SOFT_MIN_EVICTABLE_IDLE_TIME_MILLIS = -1;
    public static final boolean DEFAULT_TEST_ON_BORROW = false;
    public static final boolean DEFAULT_TEST_ON_RETURN = false;
    public static final boolean DEFAULT_TEST_WHILE_IDLE = false;
    public static final long DEFAULT_TIME_BETWEEN_EVICTION_RUNS_MILLIS = -1;
    public static final byte DEFAULT_WHEN_EXHAUSTED_ACTION = 1;
    public static final byte WHEN_EXHAUSTED_BLOCK = 1;
    public static final byte WHEN_EXHAUSTED_FAIL = 0;
    public static final byte WHEN_EXHAUSTED_GROW = 2;
    private final LinkedList<Latch<T>> _allocationQueue;
    private CursorableLinkedList<GenericKeyedObjectPool.ObjectTimestampPair<T>>.Cursor _evictionCursor;
    private GenericObjectPool<T>.Evictor _evictor;
    private PoolableObjectFactory<T> _factory;
    private boolean _lifo;
    private int _maxActive;
    private int _maxIdle;
    private long _maxWait;
    private long _minEvictableIdleTimeMillis;
    private int _minIdle;
    private int _numActive;
    private int _numInternalProcessing;
    private int _numTestsPerEvictionRun;
    private CursorableLinkedList<GenericKeyedObjectPool.ObjectTimestampPair<T>> _pool;
    private long _softMinEvictableIdleTimeMillis;
    private volatile boolean _testOnBorrow;
    private volatile boolean _testOnReturn;
    private boolean _testWhileIdle;
    private long _timeBetweenEvictionRunsMillis;
    private byte _whenExhaustedAction;

    public static class Config {
        public boolean lifo = true;
        public int maxActive = 8;
        public int maxIdle = 8;
        public long maxWait = -1;
        public long minEvictableIdleTimeMillis = 1800000;
        public int minIdle = 0;
        public int numTestsPerEvictionRun = 3;
        public long softMinEvictableIdleTimeMillis = -1;
        public boolean testOnBorrow = false;
        public boolean testOnReturn = false;
        public boolean testWhileIdle = false;
        public long timeBetweenEvictionRunsMillis = -1;
        public byte whenExhaustedAction = 1;
    }

    public GenericObjectPool() {
        this((PoolableObjectFactory) null, 8, (byte) 1, -1, 8, 0, false, false, -1, 3, 1800000, false);
    }

    public GenericObjectPool(PoolableObjectFactory<T> factory) {
        this(factory, 8, (byte) 1, -1, 8, 0, false, false, -1, 3, 1800000, false);
    }

    /* JADX WARNING: Illegal instructions before constructor call */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public GenericObjectPool(org.apache.commons.pool.PoolableObjectFactory<T> r23, org.apache.commons.pool.impl.GenericObjectPool.Config r24) {
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
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.pool.impl.GenericObjectPool.<init>(org.apache.commons.pool.PoolableObjectFactory, org.apache.commons.pool.impl.GenericObjectPool$Config):void");
    }

    public GenericObjectPool(PoolableObjectFactory<T> factory, int maxActive) {
        this(factory, maxActive, (byte) 1, -1, 8, 0, false, false, -1, 3, 1800000, false);
    }

    public GenericObjectPool(PoolableObjectFactory<T> factory, int maxActive, byte whenExhaustedAction, long maxWait) {
        this(factory, maxActive, whenExhaustedAction, maxWait, 8, 0, false, false, -1, 3, 1800000, false);
    }

    public GenericObjectPool(PoolableObjectFactory<T> factory, int maxActive, byte whenExhaustedAction, long maxWait, boolean testOnBorrow, boolean testOnReturn) {
        this(factory, maxActive, whenExhaustedAction, maxWait, 8, 0, testOnBorrow, testOnReturn, -1, 3, 1800000, false);
    }

    public GenericObjectPool(PoolableObjectFactory<T> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle) {
        this(factory, maxActive, whenExhaustedAction, maxWait, maxIdle, 0, false, false, -1, 3, 1800000, false);
    }

    public GenericObjectPool(PoolableObjectFactory<T> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle, boolean testOnBorrow, boolean testOnReturn) {
        this(factory, maxActive, whenExhaustedAction, maxWait, maxIdle, 0, testOnBorrow, testOnReturn, -1, 3, 1800000, false);
    }

    public GenericObjectPool(PoolableObjectFactory<T> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle, boolean testOnBorrow, boolean testOnReturn, long timeBetweenEvictionRunsMillis, int numTestsPerEvictionRun, long minEvictableIdleTimeMillis, boolean testWhileIdle) {
        this(factory, maxActive, whenExhaustedAction, maxWait, maxIdle, 0, testOnBorrow, testOnReturn, timeBetweenEvictionRunsMillis, numTestsPerEvictionRun, minEvictableIdleTimeMillis, testWhileIdle);
    }

    public GenericObjectPool(PoolableObjectFactory<T> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle, int minIdle, boolean testOnBorrow, boolean testOnReturn, long timeBetweenEvictionRunsMillis, int numTestsPerEvictionRun, long minEvictableIdleTimeMillis, boolean testWhileIdle) {
        this(factory, maxActive, whenExhaustedAction, maxWait, maxIdle, minIdle, testOnBorrow, testOnReturn, timeBetweenEvictionRunsMillis, numTestsPerEvictionRun, minEvictableIdleTimeMillis, testWhileIdle, -1);
    }

    public GenericObjectPool(PoolableObjectFactory<T> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle, int minIdle, boolean testOnBorrow, boolean testOnReturn, long timeBetweenEvictionRunsMillis, int numTestsPerEvictionRun, long minEvictableIdleTimeMillis, boolean testWhileIdle, long softMinEvictableIdleTimeMillis) {
        this(factory, maxActive, whenExhaustedAction, maxWait, maxIdle, minIdle, testOnBorrow, testOnReturn, timeBetweenEvictionRunsMillis, numTestsPerEvictionRun, minEvictableIdleTimeMillis, testWhileIdle, softMinEvictableIdleTimeMillis, true);
    }

    public GenericObjectPool(PoolableObjectFactory<T> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle, int minIdle, boolean testOnBorrow, boolean testOnReturn, long timeBetweenEvictionRunsMillis, int numTestsPerEvictionRun, long minEvictableIdleTimeMillis, boolean testWhileIdle, long softMinEvictableIdleTimeMillis, boolean lifo) {
        byte b = whenExhaustedAction;
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
        this._softMinEvictableIdleTimeMillis = -1;
        this._lifo = true;
        this._pool = null;
        this._evictionCursor = null;
        this._factory = null;
        this._numActive = 0;
        this._evictor = null;
        this._numInternalProcessing = 0;
        this._allocationQueue = new LinkedList<>();
        this._factory = factory;
        this._maxActive = maxActive;
        this._lifo = lifo;
        switch (b) {
            case 0:
            case 1:
            case 2:
                this._whenExhaustedAction = b;
                this._maxWait = maxWait;
                this._maxIdle = maxIdle;
                this._minIdle = minIdle;
                this._testOnBorrow = testOnBorrow;
                this._testOnReturn = testOnReturn;
                this._timeBetweenEvictionRunsMillis = timeBetweenEvictionRunsMillis;
                this._numTestsPerEvictionRun = numTestsPerEvictionRun;
                this._minEvictableIdleTimeMillis = minEvictableIdleTimeMillis;
                this._softMinEvictableIdleTimeMillis = softMinEvictableIdleTimeMillis;
                this._testWhileIdle = testWhileIdle;
                this._pool = new CursorableLinkedList<>();
                startEvictor(this._timeBetweenEvictionRunsMillis);
                return;
            default:
                long j = maxWait;
                int i = maxIdle;
                int i2 = minIdle;
                boolean z = testOnBorrow;
                boolean z2 = testOnReturn;
                long j2 = timeBetweenEvictionRunsMillis;
                int i3 = numTestsPerEvictionRun;
                long j3 = minEvictableIdleTimeMillis;
                throw new IllegalArgumentException("whenExhaustedAction " + b + " not recognized.");
        }
    }

    public synchronized int getMaxActive() {
        return this._maxActive;
    }

    public void setMaxActive(int maxActive) {
        synchronized (this) {
            this._maxActive = maxActive;
        }
        allocate();
    }

    public synchronized byte getWhenExhaustedAction() {
        return this._whenExhaustedAction;
    }

    public void setWhenExhaustedAction(byte whenExhaustedAction) {
        synchronized (this) {
            switch (whenExhaustedAction) {
                case 0:
                case 1:
                case 2:
                    this._whenExhaustedAction = whenExhaustedAction;
                    break;
                default:
                    throw new IllegalArgumentException("whenExhaustedAction " + whenExhaustedAction + " not recognized.");
            }
        }
        allocate();
    }

    public synchronized long getMaxWait() {
        return this._maxWait;
    }

    public void setMaxWait(long maxWait) {
        synchronized (this) {
            this._maxWait = maxWait;
        }
        allocate();
    }

    public synchronized int getMaxIdle() {
        return this._maxIdle;
    }

    public void setMaxIdle(int maxIdle) {
        synchronized (this) {
            this._maxIdle = maxIdle;
        }
        allocate();
    }

    public void setMinIdle(int minIdle) {
        synchronized (this) {
            this._minIdle = minIdle;
        }
        allocate();
    }

    public synchronized int getMinIdle() {
        return this._minIdle;
    }

    public boolean getTestOnBorrow() {
        return this._testOnBorrow;
    }

    public void setTestOnBorrow(boolean testOnBorrow) {
        this._testOnBorrow = testOnBorrow;
    }

    public boolean getTestOnReturn() {
        return this._testOnReturn;
    }

    public void setTestOnReturn(boolean testOnReturn) {
        this._testOnReturn = testOnReturn;
    }

    public synchronized long getTimeBetweenEvictionRunsMillis() {
        return this._timeBetweenEvictionRunsMillis;
    }

    public synchronized void setTimeBetweenEvictionRunsMillis(long timeBetweenEvictionRunsMillis) {
        this._timeBetweenEvictionRunsMillis = timeBetweenEvictionRunsMillis;
        startEvictor(this._timeBetweenEvictionRunsMillis);
    }

    public synchronized int getNumTestsPerEvictionRun() {
        return this._numTestsPerEvictionRun;
    }

    public synchronized void setNumTestsPerEvictionRun(int numTestsPerEvictionRun) {
        this._numTestsPerEvictionRun = numTestsPerEvictionRun;
    }

    public synchronized long getMinEvictableIdleTimeMillis() {
        return this._minEvictableIdleTimeMillis;
    }

    public synchronized void setMinEvictableIdleTimeMillis(long minEvictableIdleTimeMillis) {
        this._minEvictableIdleTimeMillis = minEvictableIdleTimeMillis;
    }

    public synchronized long getSoftMinEvictableIdleTimeMillis() {
        return this._softMinEvictableIdleTimeMillis;
    }

    public synchronized void setSoftMinEvictableIdleTimeMillis(long softMinEvictableIdleTimeMillis) {
        this._softMinEvictableIdleTimeMillis = softMinEvictableIdleTimeMillis;
    }

    public synchronized boolean getTestWhileIdle() {
        return this._testWhileIdle;
    }

    public synchronized void setTestWhileIdle(boolean testWhileIdle) {
        this._testWhileIdle = testWhileIdle;
    }

    public synchronized boolean getLifo() {
        return this._lifo;
    }

    public synchronized void setLifo(boolean lifo) {
        this._lifo = lifo;
    }

    public void setConfig(Config conf) {
        synchronized (this) {
            setMaxIdle(conf.maxIdle);
            setMinIdle(conf.minIdle);
            setMaxActive(conf.maxActive);
            setMaxWait(conf.maxWait);
            setWhenExhaustedAction(conf.whenExhaustedAction);
            setTestOnBorrow(conf.testOnBorrow);
            setTestOnReturn(conf.testOnReturn);
            setTestWhileIdle(conf.testWhileIdle);
            setNumTestsPerEvictionRun(conf.numTestsPerEvictionRun);
            setMinEvictableIdleTimeMillis(conf.minEvictableIdleTimeMillis);
            setTimeBetweenEvictionRunsMillis(conf.timeBetweenEvictionRunsMillis);
            setSoftMinEvictableIdleTimeMillis(conf.softMinEvictableIdleTimeMillis);
            setLifo(conf.lifo);
        }
        allocate();
    }

    /*  JADX ERROR: IndexOutOfBoundsException in pass: RegionMakerVisitor
        java.lang.IndexOutOfBoundsException: Index: 0, Size: 0
        	at java.util.ArrayList.rangeCheck(ArrayList.java:659)
        	at java.util.ArrayList.get(ArrayList.java:435)
        	at jadx.core.dex.nodes.InsnNode.getArg(InsnNode.java:101)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:611)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.processMonitorEnter(RegionMaker.java:561)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverse(RegionMaker.java:133)
        	at jadx.core.dex.visitors.regions.RegionMaker.makeRegion(RegionMaker.java:86)
        	at jadx.core.dex.visitors.regions.RegionMaker.processSwitch(RegionMaker.java:871)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverse(RegionMaker.java:128)
        	at jadx.core.dex.visitors.regions.RegionMaker.makeRegion(RegionMaker.java:86)
        	at jadx.core.dex.visitors.regions.RegionMaker.processIf(RegionMaker.java:693)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverse(RegionMaker.java:123)
        	at jadx.core.dex.visitors.regions.RegionMaker.makeRegion(RegionMaker.java:86)
        	at jadx.core.dex.visitors.regions.RegionMaker.processExcHandler(RegionMaker.java:1043)
        	at jadx.core.dex.visitors.regions.RegionMaker.processTryCatchBlocks(RegionMaker.java:975)
        	at jadx.core.dex.visitors.regions.RegionMakerVisitor.visit(RegionMakerVisitor.java:52)
        */
    /* JADX WARNING: Can't fix incorrect switch cases order */
    public T borrowObject() throws java.lang.Exception {
        /*
            r15 = this;
            long r0 = java.lang.System.currentTimeMillis()
            org.apache.commons.pool.impl.GenericObjectPool$Latch r2 = new org.apache.commons.pool.impl.GenericObjectPool$Latch
            r3 = 0
            r2.<init>()
            monitor-enter(r15)
            r3 = 0
            r4 = 0
            byte r6 = r15._whenExhaustedAction     // Catch:{ all -> 0x020b }
            long r7 = r15._maxWait     // Catch:{ all -> 0x0209 }
            java.util.LinkedList<org.apache.commons.pool.impl.GenericObjectPool$Latch<T>> r9 = r15._allocationQueue     // Catch:{ all -> 0x0206 }
            r9.add(r2)     // Catch:{ all -> 0x0206 }
            monitor-exit(r15)     // Catch:{ all -> 0x0206 }
            r15.allocate()
        L_0x001b:
            monitor-enter(r15)
            r15.assertOpen()     // Catch:{ all -> 0x0203 }
            monitor-exit(r15)     // Catch:{ all -> 0x0203 }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r9 = r2.getPair()
            r10 = 1
            if (r9 != 0) goto L_0x0145
            boolean r9 = r2.mayCreate()
            if (r9 == 0) goto L_0x002f
            goto L_0x0145
        L_0x002f:
            switch(r6) {
                case 0: goto L_0x0124;
                case 1: goto L_0x006b;
                case 2: goto L_0x004e;
                default: goto L_0x0032;
            }
        L_0x0032:
            java.lang.IllegalArgumentException r3 = new java.lang.IllegalArgumentException
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r5 = "WhenExhaustedAction property "
            r4.append(r5)
            r4.append(r6)
            java.lang.String r5 = " not recognized."
            r4.append(r5)
            java.lang.String r4 = r4.toString()
            r3.<init>(r4)
            throw r3
        L_0x004e:
            monitor-enter(r15)
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r9 = r2.getPair()     // Catch:{ all -> 0x0068 }
            if (r9 != 0) goto L_0x0065
            boolean r9 = r2.mayCreate()     // Catch:{ all -> 0x0068 }
            if (r9 != 0) goto L_0x0065
            java.util.LinkedList<org.apache.commons.pool.impl.GenericObjectPool$Latch<T>> r9 = r15._allocationQueue     // Catch:{ all -> 0x0068 }
            r9.remove(r2)     // Catch:{ all -> 0x0068 }
            int r9 = r15._numInternalProcessing     // Catch:{ all -> 0x0068 }
            int r9 = r9 + r10
            r15._numInternalProcessing = r9     // Catch:{ all -> 0x0068 }
        L_0x0065:
            monitor-exit(r15)     // Catch:{ all -> 0x0068 }
            goto L_0x0145
        L_0x0068:
            r3 = move-exception
            monitor-exit(r15)     // Catch:{ all -> 0x0068 }
            throw r3
        L_0x006b:
            monitor-enter(r2)     // Catch:{ InterruptedException -> 0x00d5 }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r9 = r2.getPair()     // Catch:{ all -> 0x00d2 }
            if (r9 != 0) goto L_0x00cf
            boolean r9 = r2.mayCreate()     // Catch:{ all -> 0x00d2 }
            if (r9 != 0) goto L_0x00cf
            int r9 = (r7 > r4 ? 1 : (r7 == r4 ? 0 : -1))
            if (r9 > 0) goto L_0x0080
            r2.wait()     // Catch:{ all -> 0x00d2 }
            goto L_0x0090
        L_0x0080:
            long r11 = java.lang.System.currentTimeMillis()     // Catch:{ all -> 0x00d2 }
            r9 = 0
            long r11 = r11 - r0
            long r13 = r7 - r11
            int r9 = (r13 > r4 ? 1 : (r13 == r4 ? 0 : -1))
            if (r9 <= 0) goto L_0x008f
            r2.wait(r13)     // Catch:{ all -> 0x00d2 }
        L_0x008f:
        L_0x0090:
            monitor-exit(r2)     // Catch:{ all -> 0x00d2 }
            boolean r9 = r15.isClosed()     // Catch:{ InterruptedException -> 0x00d5 }
            if (r9 == r10) goto L_0x00c7
            int r9 = (r7 > r4 ? 1 : (r7 == r4 ? 0 : -1))
            if (r9 <= 0) goto L_0x001b
            long r11 = java.lang.System.currentTimeMillis()
            long r11 = r11 - r0
            int r9 = (r11 > r7 ? 1 : (r11 == r7 ? 0 : -1))
            if (r9 < 0) goto L_0x001b
            monitor-enter(r15)
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r9 = r2.getPair()     // Catch:{ all -> 0x00c4 }
            if (r9 != 0) goto L_0x00c1
            boolean r9 = r2.mayCreate()     // Catch:{ all -> 0x00c4 }
            if (r9 == 0) goto L_0x00b3
            goto L_0x00c1
        L_0x00b3:
            java.util.LinkedList<org.apache.commons.pool.impl.GenericObjectPool$Latch<T>> r3 = r15._allocationQueue     // Catch:{ all -> 0x00c4 }
            r3.remove(r2)     // Catch:{ all -> 0x00c4 }
            monitor-exit(r15)     // Catch:{ all -> 0x00c4 }
            java.util.NoSuchElementException r3 = new java.util.NoSuchElementException
            java.lang.String r4 = "Timeout waiting for idle object"
            r3.<init>(r4)
            throw r3
        L_0x00c1:
            monitor-exit(r15)     // Catch:{ all -> 0x00c4 }
            goto L_0x0145
        L_0x00c4:
            r3 = move-exception
            monitor-exit(r15)     // Catch:{ all -> 0x00c4 }
            throw r3
        L_0x00c7:
            java.lang.IllegalStateException r3 = new java.lang.IllegalStateException     // Catch:{ InterruptedException -> 0x00d5 }
            java.lang.String r4 = "Pool closed"
            r3.<init>(r4)     // Catch:{ InterruptedException -> 0x00d5 }
            throw r3     // Catch:{ InterruptedException -> 0x00d5 }
        L_0x00cf:
            monitor-exit(r2)     // Catch:{ all -> 0x00d2 }
            goto L_0x0145
        L_0x00d2:
            r3 = move-exception
            monitor-exit(r2)     // Catch:{ all -> 0x00d2 }
            throw r3     // Catch:{ InterruptedException -> 0x00d5 }
        L_0x00d5:
            r3 = move-exception
            r9 = r3
            r3 = 0
            monitor-enter(r15)
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r4 = r2.getPair()     // Catch:{ all -> 0x0121 }
            if (r4 != 0) goto L_0x00eb
            boolean r4 = r2.mayCreate()     // Catch:{ all -> 0x0121 }
            if (r4 != 0) goto L_0x00eb
            java.util.LinkedList<org.apache.commons.pool.impl.GenericObjectPool$Latch<T>> r4 = r15._allocationQueue     // Catch:{ all -> 0x0121 }
            r4.remove(r2)     // Catch:{ all -> 0x0121 }
            goto L_0x0113
        L_0x00eb:
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r4 = r2.getPair()     // Catch:{ all -> 0x0121 }
            if (r4 != 0) goto L_0x00fe
            boolean r4 = r2.mayCreate()     // Catch:{ all -> 0x0121 }
            if (r4 == 0) goto L_0x00fe
            int r4 = r15._numInternalProcessing     // Catch:{ all -> 0x0121 }
            int r4 = r4 - r10
            r15._numInternalProcessing = r4     // Catch:{ all -> 0x0121 }
            r3 = 1
            goto L_0x0113
        L_0x00fe:
            int r4 = r15._numInternalProcessing     // Catch:{ all -> 0x0121 }
            int r4 = r4 - r10
            r15._numInternalProcessing = r4     // Catch:{ all -> 0x0121 }
            int r4 = r15._numActive     // Catch:{ all -> 0x0121 }
            int r4 = r4 + r10
            r15._numActive = r4     // Catch:{ all -> 0x0121 }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r4 = r2.getPair()     // Catch:{ all -> 0x0121 }
            java.lang.Object r4 = r4.getValue()     // Catch:{ all -> 0x0121 }
            r15.returnObject(r4)     // Catch:{ all -> 0x0121 }
        L_0x0113:
            monitor-exit(r15)     // Catch:{ all -> 0x0121 }
            if (r3 == 0) goto L_0x0119
            r15.allocate()
        L_0x0119:
            java.lang.Thread r4 = java.lang.Thread.currentThread()
            r4.interrupt()
            throw r9
        L_0x0121:
            r4 = move-exception
            monitor-exit(r15)     // Catch:{ all -> 0x0121 }
            throw r4
        L_0x0124:
            monitor-enter(r15)
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r9 = r2.getPair()     // Catch:{ all -> 0x0142 }
            if (r9 != 0) goto L_0x0140
            boolean r9 = r2.mayCreate()     // Catch:{ all -> 0x0142 }
            if (r9 == 0) goto L_0x0132
            goto L_0x0140
        L_0x0132:
            java.util.LinkedList<org.apache.commons.pool.impl.GenericObjectPool$Latch<T>> r3 = r15._allocationQueue     // Catch:{ all -> 0x0142 }
            r3.remove(r2)     // Catch:{ all -> 0x0142 }
            monitor-exit(r15)     // Catch:{ all -> 0x0142 }
            java.util.NoSuchElementException r3 = new java.util.NoSuchElementException
            java.lang.String r4 = "Pool exhausted"
            r3.<init>(r4)
            throw r3
        L_0x0140:
            monitor-exit(r15)     // Catch:{ all -> 0x0142 }
            goto L_0x0145
        L_0x0142:
            r3 = move-exception
            monitor-exit(r15)     // Catch:{ all -> 0x0142 }
            throw r3
        L_0x0145:
            r9 = 0
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r11 = r2.getPair()
            if (r11 != 0) goto L_0x017d
            org.apache.commons.pool.PoolableObjectFactory<T> r11 = r15._factory     // Catch:{ all -> 0x016b }
            java.lang.Object r11 = r11.makeObject()     // Catch:{ all -> 0x016b }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r12 = new org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair     // Catch:{ all -> 0x016b }
            r12.<init>(r11)     // Catch:{ all -> 0x016b }
            r2.setPair(r12)     // Catch:{ all -> 0x016b }
            r9 = 1
            if (r9 != 0) goto L_0x017d
            monitor-enter(r15)
            int r11 = r15._numInternalProcessing     // Catch:{ all -> 0x0168 }
            int r11 = r11 - r10
            r15._numInternalProcessing = r11     // Catch:{ all -> 0x0168 }
            monitor-exit(r15)     // Catch:{ all -> 0x0168 }
            r15.allocate()
            goto L_0x017d
        L_0x0168:
            r3 = move-exception
            monitor-exit(r15)     // Catch:{ all -> 0x0168 }
            throw r3
        L_0x016b:
            r3 = move-exception
            if (r9 != 0) goto L_0x017c
            monitor-enter(r15)
            int r4 = r15._numInternalProcessing     // Catch:{ all -> 0x0179 }
            int r4 = r4 - r10
            r15._numInternalProcessing = r4     // Catch:{ all -> 0x0179 }
            monitor-exit(r15)     // Catch:{ all -> 0x0179 }
            r15.allocate()
            goto L_0x017c
        L_0x0179:
            r3 = move-exception
            monitor-exit(r15)     // Catch:{ all -> 0x0179 }
            throw r3
        L_0x017c:
            throw r3
        L_0x017d:
            org.apache.commons.pool.PoolableObjectFactory<T> r11 = r15._factory     // Catch:{ Throwable -> 0x01b9 }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r12 = r2.getPair()     // Catch:{ Throwable -> 0x01b9 }
            T r12 = r12.value     // Catch:{ Throwable -> 0x01b9 }
            r11.activateObject(r12)     // Catch:{ Throwable -> 0x01b9 }
            boolean r11 = r15._testOnBorrow     // Catch:{ Throwable -> 0x01b9 }
            if (r11 == 0) goto L_0x01a3
            org.apache.commons.pool.PoolableObjectFactory<T> r11 = r15._factory     // Catch:{ Throwable -> 0x01b9 }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r12 = r2.getPair()     // Catch:{ Throwable -> 0x01b9 }
            T r12 = r12.value     // Catch:{ Throwable -> 0x01b9 }
            boolean r11 = r11.validateObject(r12)     // Catch:{ Throwable -> 0x01b9 }
            if (r11 == 0) goto L_0x019b
            goto L_0x01a3
        L_0x019b:
            java.lang.Exception r11 = new java.lang.Exception     // Catch:{ Throwable -> 0x01b9 }
            java.lang.String r12 = "ValidateObject failed"
            r11.<init>(r12)     // Catch:{ Throwable -> 0x01b9 }
            throw r11     // Catch:{ Throwable -> 0x01b9 }
        L_0x01a3:
            monitor-enter(r15)     // Catch:{ Throwable -> 0x01b9 }
            int r11 = r15._numInternalProcessing     // Catch:{ all -> 0x01b6 }
            int r11 = r11 - r10
            r15._numInternalProcessing = r11     // Catch:{ all -> 0x01b6 }
            int r11 = r15._numActive     // Catch:{ all -> 0x01b6 }
            int r11 = r11 + r10
            r15._numActive = r11     // Catch:{ all -> 0x01b6 }
            monitor-exit(r15)     // Catch:{ all -> 0x01b6 }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r11 = r2.getPair()     // Catch:{ Throwable -> 0x01b9 }
            T r11 = r11.value     // Catch:{ Throwable -> 0x01b9 }
            return r11
        L_0x01b6:
            r11 = move-exception
            monitor-exit(r15)     // Catch:{ all -> 0x01b6 }
            throw r11     // Catch:{ Throwable -> 0x01b9 }
        L_0x01b9:
            r11 = move-exception
            org.apache.commons.pool.PoolUtils.checkRethrow(r11)
            org.apache.commons.pool.PoolableObjectFactory<T> r12 = r15._factory     // Catch:{ Throwable -> 0x01c9 }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r13 = r2.getPair()     // Catch:{ Throwable -> 0x01c9 }
            T r13 = r13.value     // Catch:{ Throwable -> 0x01c9 }
            r12.destroyObject(r13)     // Catch:{ Throwable -> 0x01c9 }
            goto L_0x01cd
        L_0x01c9:
            r12 = move-exception
            org.apache.commons.pool.PoolUtils.checkRethrow(r12)
        L_0x01cd:
            monitor-enter(r15)
            int r12 = r15._numInternalProcessing     // Catch:{ all -> 0x0200 }
            int r12 = r12 - r10
            r15._numInternalProcessing = r12     // Catch:{ all -> 0x0200 }
            if (r9 != 0) goto L_0x01dd
            r2.reset()     // Catch:{ all -> 0x0200 }
            java.util.LinkedList<org.apache.commons.pool.impl.GenericObjectPool$Latch<T>> r10 = r15._allocationQueue     // Catch:{ all -> 0x0200 }
            r10.add(r3, r2)     // Catch:{ all -> 0x0200 }
        L_0x01dd:
            monitor-exit(r15)     // Catch:{ all -> 0x0200 }
            r15.allocate()
            if (r9 != 0) goto L_0x01e5
            goto L_0x001b
        L_0x01e5:
            java.util.NoSuchElementException r3 = new java.util.NoSuchElementException
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r5 = "Could not create a validated object, cause: "
            r4.append(r5)
            java.lang.String r5 = r11.getMessage()
            r4.append(r5)
            java.lang.String r4 = r4.toString()
            r3.<init>(r4)
            throw r3
        L_0x0200:
            r3 = move-exception
            monitor-exit(r15)     // Catch:{ all -> 0x0200 }
            throw r3
        L_0x0203:
            r3 = move-exception
            monitor-exit(r15)     // Catch:{ all -> 0x0203 }
            throw r3
        L_0x0206:
            r3 = move-exception
            r4 = r7
            goto L_0x020e
        L_0x0209:
            r3 = move-exception
            goto L_0x020e
        L_0x020b:
            r6 = move-exception
            r3 = r6
            r6 = 0
        L_0x020e:
            monitor-exit(r15)     // Catch:{ all -> 0x0209 }
            throw r3
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.pool.impl.GenericObjectPool.borrowObject():java.lang.Object");
    }

    /* JADX WARNING: Code restructure failed: missing block: B:38:0x006a, code lost:
        return;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private synchronized void allocate() {
        /*
            r3 = this;
            monitor-enter(r3)
            boolean r0 = r3.isClosed()     // Catch:{ all -> 0x006b }
            if (r0 == 0) goto L_0x0009
            monitor-exit(r3)
            return
        L_0x0009:
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<T>> r0 = r3._pool     // Catch:{ all -> 0x006b }
            boolean r0 = r0.isEmpty()     // Catch:{ all -> 0x006b }
            r1 = 1
            if (r0 != 0) goto L_0x003b
            java.util.LinkedList<org.apache.commons.pool.impl.GenericObjectPool$Latch<T>> r0 = r3._allocationQueue     // Catch:{ all -> 0x006b }
            boolean r0 = r0.isEmpty()     // Catch:{ all -> 0x006b }
            if (r0 != 0) goto L_0x003b
            java.util.LinkedList<org.apache.commons.pool.impl.GenericObjectPool$Latch<T>> r0 = r3._allocationQueue     // Catch:{ all -> 0x006b }
            java.lang.Object r0 = r0.removeFirst()     // Catch:{ all -> 0x006b }
            org.apache.commons.pool.impl.GenericObjectPool$Latch r0 = (org.apache.commons.pool.impl.GenericObjectPool.Latch) r0     // Catch:{ all -> 0x006b }
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<T>> r2 = r3._pool     // Catch:{ all -> 0x006b }
            java.lang.Object r2 = r2.removeFirst()     // Catch:{ all -> 0x006b }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r2 = (org.apache.commons.pool.impl.GenericKeyedObjectPool.ObjectTimestampPair) r2     // Catch:{ all -> 0x006b }
            r0.setPair(r2)     // Catch:{ all -> 0x006b }
            int r2 = r3._numInternalProcessing     // Catch:{ all -> 0x006b }
            int r2 = r2 + r1
            r3._numInternalProcessing = r2     // Catch:{ all -> 0x006b }
            monitor-enter(r0)     // Catch:{ all -> 0x006b }
            r0.notify()     // Catch:{ all -> 0x0038 }
            monitor-exit(r0)     // Catch:{ all -> 0x0038 }
            goto L_0x0009
        L_0x0038:
            r1 = move-exception
            monitor-exit(r0)     // Catch:{ all -> 0x0038 }
            throw r1     // Catch:{ all -> 0x006b }
        L_0x003b:
            java.util.LinkedList<org.apache.commons.pool.impl.GenericObjectPool$Latch<T>> r0 = r3._allocationQueue     // Catch:{ all -> 0x006b }
            boolean r0 = r0.isEmpty()     // Catch:{ all -> 0x006b }
            if (r0 != 0) goto L_0x0069
            int r0 = r3._maxActive     // Catch:{ all -> 0x006b }
            if (r0 < 0) goto L_0x0050
            int r0 = r3._numActive     // Catch:{ all -> 0x006b }
            int r2 = r3._numInternalProcessing     // Catch:{ all -> 0x006b }
            int r0 = r0 + r2
            int r2 = r3._maxActive     // Catch:{ all -> 0x006b }
            if (r0 >= r2) goto L_0x0069
        L_0x0050:
            java.util.LinkedList<org.apache.commons.pool.impl.GenericObjectPool$Latch<T>> r0 = r3._allocationQueue     // Catch:{ all -> 0x006b }
            java.lang.Object r0 = r0.removeFirst()     // Catch:{ all -> 0x006b }
            org.apache.commons.pool.impl.GenericObjectPool$Latch r0 = (org.apache.commons.pool.impl.GenericObjectPool.Latch) r0     // Catch:{ all -> 0x006b }
            r0.setMayCreate(r1)     // Catch:{ all -> 0x006b }
            int r2 = r3._numInternalProcessing     // Catch:{ all -> 0x006b }
            int r2 = r2 + r1
            r3._numInternalProcessing = r2     // Catch:{ all -> 0x006b }
            monitor-enter(r0)     // Catch:{ all -> 0x006b }
            r0.notify()     // Catch:{ all -> 0x0066 }
            monitor-exit(r0)     // Catch:{ all -> 0x0066 }
            goto L_0x003b
        L_0x0066:
            r1 = move-exception
            monitor-exit(r0)     // Catch:{ all -> 0x0066 }
            throw r1     // Catch:{ all -> 0x006b }
        L_0x0069:
            monitor-exit(r3)
            return
        L_0x006b:
            r0 = move-exception
            monitor-exit(r3)
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.pool.impl.GenericObjectPool.allocate():void");
    }

    public void invalidateObject(T obj) throws Exception {
        try {
            if (this._factory != null) {
                this._factory.destroyObject(obj);
            }
            synchronized (this) {
                this._numActive--;
            }
            allocate();
        } catch (Throwable th) {
            synchronized (this) {
                this._numActive--;
                allocate();
                throw th;
            }
        }
    }

    public void clear() {
        List<GenericKeyedObjectPool.ObjectTimestampPair<T>> toDestroy = new ArrayList<>();
        synchronized (this) {
            toDestroy.addAll(this._pool);
            this._numInternalProcessing += this._pool._size;
            this._pool.clear();
        }
        destroy(toDestroy, this._factory);
    }

    /*  JADX ERROR: IndexOutOfBoundsException in pass: RegionMakerVisitor
        java.lang.IndexOutOfBoundsException: Index: 0, Size: 0
        	at java.util.ArrayList.rangeCheck(ArrayList.java:659)
        	at java.util.ArrayList.get(ArrayList.java:435)
        	at jadx.core.dex.nodes.InsnNode.getArg(InsnNode.java:101)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:611)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.processMonitorEnter(RegionMaker.java:561)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverse(RegionMaker.java:133)
        	at jadx.core.dex.visitors.regions.RegionMaker.makeRegion(RegionMaker.java:86)
        	at jadx.core.dex.visitors.regions.RegionMaker.processExcHandler(RegionMaker.java:1043)
        	at jadx.core.dex.visitors.regions.RegionMaker.processTryCatchBlocks(RegionMaker.java:975)
        	at jadx.core.dex.visitors.regions.RegionMakerVisitor.visit(RegionMakerVisitor.java:52)
        */
    private void destroy(java.util.Collection<org.apache.commons.pool.impl.GenericKeyedObjectPool.ObjectTimestampPair<T>> r4, org.apache.commons.pool.PoolableObjectFactory<T> r5) {
        /*
            r3 = this;
            java.util.Iterator r0 = r4.iterator()
        L_0x0004:
            boolean r1 = r0.hasNext()
            if (r1 == 0) goto L_0x0041
            java.lang.Object r1 = r0.next()     // Catch:{ Exception -> 0x0031, all -> 0x0021 }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r1 = (org.apache.commons.pool.impl.GenericKeyedObjectPool.ObjectTimestampPair) r1     // Catch:{ Exception -> 0x0031, all -> 0x0021 }
            T r1 = r1.value     // Catch:{ Exception -> 0x0031, all -> 0x0021 }
            r5.destroyObject(r1)     // Catch:{ Exception -> 0x0031, all -> 0x0021 }
            monitor-enter(r3)
            int r1 = r3._numInternalProcessing     // Catch:{ all -> 0x001e }
            int r1 = r1 + -1
            r3._numInternalProcessing = r1     // Catch:{ all -> 0x001e }
            monitor-exit(r3)     // Catch:{ all -> 0x001e }
            goto L_0x003a
        L_0x001e:
            r1 = move-exception
            monitor-exit(r3)     // Catch:{ all -> 0x001e }
            throw r1
        L_0x0021:
            r1 = move-exception
            monitor-enter(r3)
            int r2 = r3._numInternalProcessing     // Catch:{ all -> 0x002e }
            int r2 = r2 + -1
            r3._numInternalProcessing = r2     // Catch:{ all -> 0x002e }
            monitor-exit(r3)     // Catch:{ all -> 0x002e }
            r3.allocate()
            throw r1
        L_0x002e:
            r1 = move-exception
            monitor-exit(r3)     // Catch:{ all -> 0x002e }
            throw r1
        L_0x0031:
            r1 = move-exception
            monitor-enter(r3)
            int r1 = r3._numInternalProcessing     // Catch:{ all -> 0x003e }
            int r1 = r1 + -1
            r3._numInternalProcessing = r1     // Catch:{ all -> 0x003e }
            monitor-exit(r3)     // Catch:{ all -> 0x003e }
        L_0x003a:
            r3.allocate()
            goto L_0x0004
        L_0x003e:
            r1 = move-exception
            monitor-exit(r3)     // Catch:{ all -> 0x003e }
            throw r1
        L_0x0041:
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.pool.impl.GenericObjectPool.destroy(java.util.Collection, org.apache.commons.pool.PoolableObjectFactory):void");
    }

    public synchronized int getNumActive() {
        return this._numActive;
    }

    public synchronized int getNumIdle() {
        return this._pool.size();
    }

    public void returnObject(T obj) throws Exception {
        try {
            addObjectToPool(obj, true);
        } catch (Exception e) {
            if (this._factory != null) {
                try {
                    this._factory.destroyObject(obj);
                } catch (Exception e2) {
                }
                synchronized (this) {
                    this._numActive--;
                    allocate();
                }
            }
        }
    }

    private void addObjectToPool(T obj, boolean decrementNumActive) throws Exception {
        boolean success = true;
        if (!this._testOnReturn || this._factory.validateObject(obj)) {
            this._factory.passivateObject(obj);
        } else {
            success = false;
        }
        boolean shouldDestroy = !success;
        boolean doAllocate = false;
        synchronized (this) {
            if (isClosed()) {
                shouldDestroy = true;
            } else if (this._maxIdle >= 0 && this._pool.size() >= this._maxIdle) {
                shouldDestroy = true;
            } else if (success) {
                if (this._lifo) {
                    this._pool.addFirst(new GenericKeyedObjectPool.ObjectTimestampPair(obj));
                } else {
                    this._pool.addLast(new GenericKeyedObjectPool.ObjectTimestampPair(obj));
                }
                if (decrementNumActive) {
                    this._numActive--;
                }
                doAllocate = true;
            }
        }
        if (doAllocate) {
            allocate();
        }
        if (shouldDestroy) {
            try {
                this._factory.destroyObject(obj);
            } catch (Exception e) {
            }
            if (decrementNumActive) {
                synchronized (this) {
                    this._numActive--;
                }
                allocate();
            }
        }
    }

    public void close() throws Exception {
        super.close();
        synchronized (this) {
            clear();
            startEvictor(-1);
            while (this._allocationQueue.size() > 0) {
                Latch<T> l = this._allocationQueue.removeFirst();
                synchronized (l) {
                    l.notify();
                }
            }
        }
    }

    @Deprecated
    public void setFactory(PoolableObjectFactory<T> factory) throws IllegalStateException {
        List<GenericKeyedObjectPool.ObjectTimestampPair<T>> toDestroy = new ArrayList<>();
        PoolableObjectFactory<T> oldFactory = this._factory;
        synchronized (this) {
            assertOpen();
            if (getNumActive() <= 0) {
                toDestroy.addAll(this._pool);
                this._numInternalProcessing += this._pool._size;
                this._pool.clear();
                this._factory = factory;
            } else {
                throw new IllegalStateException("Objects are already active");
            }
        }
        destroy(toDestroy, oldFactory);
    }

    /*  JADX ERROR: IndexOutOfBoundsException in pass: RegionMakerVisitor
        java.lang.IndexOutOfBoundsException: Index: 0, Size: 0
        	at java.util.ArrayList.rangeCheck(ArrayList.java:659)
        	at java.util.ArrayList.get(ArrayList.java:435)
        	at jadx.core.dex.nodes.InsnNode.getArg(InsnNode.java:101)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:611)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.processMonitorEnter(RegionMaker.java:561)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverse(RegionMaker.java:133)
        	at jadx.core.dex.visitors.regions.RegionMaker.makeRegion(RegionMaker.java:86)
        	at jadx.core.dex.visitors.regions.RegionMaker.processHandlersOutBlocks(RegionMaker.java:1008)
        	at jadx.core.dex.visitors.regions.RegionMaker.processTryCatchBlocks(RegionMaker.java:978)
        	at jadx.core.dex.visitors.regions.RegionMakerVisitor.visit(RegionMakerVisitor.java:52)
        */
    public void evict() throws java.lang.Exception {
        /*
            r12 = this;
            r12.assertOpen()
            monitor-enter(r12)
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<T>> r0 = r12._pool     // Catch:{ all -> 0x011b }
            boolean r0 = r0.isEmpty()     // Catch:{ all -> 0x011b }
            if (r0 == 0) goto L_0x000e
            monitor-exit(r12)     // Catch:{ all -> 0x011b }
            return
        L_0x000e:
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<T>>$Cursor r0 = r12._evictionCursor     // Catch:{ all -> 0x011b }
            r1 = 0
            if (r0 != 0) goto L_0x0027
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<T>> r0 = r12._pool     // Catch:{ all -> 0x011b }
            boolean r2 = r12._lifo     // Catch:{ all -> 0x011b }
            if (r2 == 0) goto L_0x0020
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<T>> r2 = r12._pool     // Catch:{ all -> 0x011b }
            int r2 = r2.size()     // Catch:{ all -> 0x011b }
            goto L_0x0021
        L_0x0020:
            r2 = 0
        L_0x0021:
            org.apache.commons.pool.impl.CursorableLinkedList$Cursor r0 = r0.cursor(r2)     // Catch:{ all -> 0x011b }
            r12._evictionCursor = r0     // Catch:{ all -> 0x011b }
        L_0x0027:
            monitor-exit(r12)     // Catch:{ all -> 0x011b }
            r0 = 0
            int r2 = r12.getNumTests()
            r3 = 0
        L_0x002e:
            if (r0 >= r2) goto L_0x0117
            monitor-enter(r12)
            boolean r4 = r12._lifo     // Catch:{ all -> 0x0114 }
            if (r4 == 0) goto L_0x003d
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<T>>$Cursor r4 = r12._evictionCursor     // Catch:{ all -> 0x0114 }
            boolean r4 = r4.hasPrevious()     // Catch:{ all -> 0x0114 }
            if (r4 == 0) goto L_0x0049
        L_0x003d:
            boolean r4 = r12._lifo     // Catch:{ all -> 0x0114 }
            if (r4 != 0) goto L_0x0062
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<T>>$Cursor r4 = r12._evictionCursor     // Catch:{ all -> 0x0114 }
            boolean r4 = r4.hasNext()     // Catch:{ all -> 0x0114 }
            if (r4 != 0) goto L_0x0062
        L_0x0049:
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<T>>$Cursor r4 = r12._evictionCursor     // Catch:{ all -> 0x0114 }
            r4.close()     // Catch:{ all -> 0x0114 }
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<T>> r4 = r12._pool     // Catch:{ all -> 0x0114 }
            boolean r5 = r12._lifo     // Catch:{ all -> 0x0114 }
            if (r5 == 0) goto L_0x005b
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<T>> r5 = r12._pool     // Catch:{ all -> 0x0114 }
            int r5 = r5.size()     // Catch:{ all -> 0x0114 }
            goto L_0x005c
        L_0x005b:
            r5 = 0
        L_0x005c:
            org.apache.commons.pool.impl.CursorableLinkedList$Cursor r4 = r4.cursor(r5)     // Catch:{ all -> 0x0114 }
            r12._evictionCursor = r4     // Catch:{ all -> 0x0114 }
        L_0x0062:
            boolean r4 = r12._lifo     // Catch:{ all -> 0x0114 }
            if (r4 == 0) goto L_0x006f
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<T>>$Cursor r4 = r12._evictionCursor     // Catch:{ all -> 0x0114 }
            java.lang.Object r4 = r4.previous()     // Catch:{ all -> 0x0114 }
        L_0x006c:
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r4 = (org.apache.commons.pool.impl.GenericKeyedObjectPool.ObjectTimestampPair) r4     // Catch:{ all -> 0x0114 }
            goto L_0x0076
        L_0x006f:
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<T>>$Cursor r4 = r12._evictionCursor     // Catch:{ all -> 0x0114 }
            java.lang.Object r4 = r4.next()     // Catch:{ all -> 0x0114 }
            goto L_0x006c
        L_0x0076:
            r3 = r4
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<T>>$Cursor r4 = r12._evictionCursor     // Catch:{ all -> 0x0114 }
            r4.remove()     // Catch:{ all -> 0x0114 }
            int r4 = r12._numInternalProcessing     // Catch:{ all -> 0x0114 }
            int r4 = r4 + 1
            r12._numInternalProcessing = r4     // Catch:{ all -> 0x0114 }
            monitor-exit(r12)     // Catch:{ all -> 0x0114 }
            r4 = 0
            long r5 = java.lang.System.currentTimeMillis()
            long r7 = r3.tstamp
            long r5 = r5 - r7
            long r7 = r12.getMinEvictableIdleTimeMillis()
            r9 = 0
            int r11 = (r7 > r9 ? 1 : (r7 == r9 ? 0 : -1))
            if (r11 <= 0) goto L_0x009f
            long r7 = r12.getMinEvictableIdleTimeMillis()
            int r11 = (r5 > r7 ? 1 : (r5 == r7 ? 0 : -1))
            if (r11 <= 0) goto L_0x009f
            r4 = 1
            goto L_0x00bc
        L_0x009f:
            long r7 = r12.getSoftMinEvictableIdleTimeMillis()
            int r11 = (r7 > r9 ? 1 : (r7 == r9 ? 0 : -1))
            if (r11 <= 0) goto L_0x00bc
            long r7 = r12.getSoftMinEvictableIdleTimeMillis()
            int r9 = (r5 > r7 ? 1 : (r5 == r7 ? 0 : -1))
            if (r9 <= 0) goto L_0x00bc
            int r7 = r12.getNumIdle()
            int r7 = r7 + 1
            int r8 = r12.getMinIdle()
            if (r7 <= r8) goto L_0x00bc
            r4 = 1
        L_0x00bc:
            boolean r7 = r12.getTestWhileIdle()
            if (r7 == 0) goto L_0x00e8
            if (r4 != 0) goto L_0x00e8
            r7 = r1
            org.apache.commons.pool.PoolableObjectFactory<T> r8 = r12._factory     // Catch:{ Exception -> 0x00ce }
            T r9 = r3.value     // Catch:{ Exception -> 0x00ce }
            r8.activateObject(r9)     // Catch:{ Exception -> 0x00ce }
            r7 = 1
            goto L_0x00d0
        L_0x00ce:
            r8 = move-exception
            r4 = 1
        L_0x00d0:
            if (r7 == 0) goto L_0x00e8
            org.apache.commons.pool.PoolableObjectFactory<T> r8 = r12._factory
            T r9 = r3.value
            boolean r8 = r8.validateObject(r9)
            if (r8 != 0) goto L_0x00de
            r4 = 1
            goto L_0x00e8
        L_0x00de:
            org.apache.commons.pool.PoolableObjectFactory<T> r8 = r12._factory     // Catch:{ Exception -> 0x00e6 }
            T r9 = r3.value     // Catch:{ Exception -> 0x00e6 }
            r8.passivateObject(r9)     // Catch:{ Exception -> 0x00e6 }
            goto L_0x00e8
        L_0x00e6:
            r8 = move-exception
            r4 = 1
        L_0x00e8:
            if (r4 == 0) goto L_0x00f3
            org.apache.commons.pool.PoolableObjectFactory<T> r7 = r12._factory     // Catch:{ Exception -> 0x00f2 }
            T r8 = r3.value     // Catch:{ Exception -> 0x00f2 }
            r7.destroyObject(r8)     // Catch:{ Exception -> 0x00f2 }
            goto L_0x00f3
        L_0x00f2:
            r7 = move-exception
        L_0x00f3:
            monitor-enter(r12)
            if (r4 != 0) goto L_0x0107
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<T>>$Cursor r7 = r12._evictionCursor     // Catch:{ all -> 0x0105 }
            r7.add(r3)     // Catch:{ all -> 0x0105 }
            boolean r7 = r12._lifo     // Catch:{ all -> 0x0105 }
            if (r7 == 0) goto L_0x0107
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<T>>$Cursor r7 = r12._evictionCursor     // Catch:{ all -> 0x0105 }
            r7.previous()     // Catch:{ all -> 0x0105 }
            goto L_0x0107
        L_0x0105:
            r1 = move-exception
            goto L_0x0112
        L_0x0107:
            int r7 = r12._numInternalProcessing     // Catch:{ all -> 0x0105 }
            int r7 = r7 + -1
            r12._numInternalProcessing = r7     // Catch:{ all -> 0x0105 }
            monitor-exit(r12)     // Catch:{ all -> 0x0105 }
            int r0 = r0 + 1
            goto L_0x002e
        L_0x0112:
            monitor-exit(r12)     // Catch:{ all -> 0x0105 }
            throw r1
        L_0x0114:
            r1 = move-exception
            monitor-exit(r12)     // Catch:{ all -> 0x0114 }
            throw r1
        L_0x0117:
            r12.allocate()
            return
        L_0x011b:
            r0 = move-exception
            monitor-exit(r12)     // Catch:{ all -> 0x011b }
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.pool.impl.GenericObjectPool.evict():void");
    }

    /* access modifiers changed from: private */
    public void ensureMinIdle() throws Exception {
        int j = 0;
        int objectDeficit = calculateDeficit(false);
        while (j < objectDeficit && calculateDeficit(true) > 0) {
            try {
                addObject();
                synchronized (this) {
                    this._numInternalProcessing--;
                }
                allocate();
                j++;
            } catch (Throwable th) {
                synchronized (this) {
                    this._numInternalProcessing--;
                    allocate();
                    throw th;
                }
            }
        }
    }

    private synchronized int calculateDeficit(boolean incrementInternal) {
        int objectDeficit;
        objectDeficit = getMinIdle() - getNumIdle();
        if (this._maxActive > 0) {
            objectDeficit = Math.min(objectDeficit, Math.max(0, ((getMaxActive() - getNumActive()) - getNumIdle()) - this._numInternalProcessing));
        }
        if (incrementInternal && objectDeficit > 0) {
            this._numInternalProcessing++;
        }
        return objectDeficit;
    }

    public void addObject() throws Exception {
        assertOpen();
        if (this._factory != null) {
            T obj = this._factory.makeObject();
            try {
                assertOpen();
                addObjectToPool(obj, false);
            } catch (IllegalStateException ex) {
                try {
                    this._factory.destroyObject(obj);
                } catch (Exception e) {
                }
                throw ex;
            }
        } else {
            throw new IllegalStateException("Cannot add objects without a factory.");
        }
    }

    /* access modifiers changed from: protected */
    public synchronized void startEvictor(long delay) {
        if (this._evictor != null) {
            EvictionTimer.cancel(this._evictor);
            this._evictor = null;
        }
        if (delay > 0) {
            this._evictor = new Evictor();
            EvictionTimer.schedule(this._evictor, delay, delay);
        }
    }

    /* access modifiers changed from: package-private */
    public synchronized String debugInfo() {
        StringBuffer buf;
        buf = new StringBuffer();
        buf.append("Active: ");
        buf.append(getNumActive());
        buf.append("\n");
        buf.append("Idle: ");
        buf.append(getNumIdle());
        buf.append("\n");
        buf.append("Idle Objects:\n");
        Iterator<GenericKeyedObjectPool.ObjectTimestampPair<T>> it = this._pool.iterator();
        long time = System.currentTimeMillis();
        while (it.hasNext()) {
            GenericKeyedObjectPool.ObjectTimestampPair<T> pair = it.next();
            buf.append("\t");
            buf.append(pair.value);
            buf.append("\t");
            buf.append(time - pair.tstamp);
            buf.append("\n");
        }
        return buf.toString();
    }

    private int getNumTests() {
        if (this._numTestsPerEvictionRun >= 0) {
            return Math.min(this._numTestsPerEvictionRun, this._pool.size());
        }
        double size = (double) this._pool.size();
        double abs = Math.abs((double) this._numTestsPerEvictionRun);
        Double.isNaN(size);
        return (int) Math.ceil(size / abs);
    }

    private class Evictor extends TimerTask {
        private Evictor() {
        }

        public void run() {
            try {
                GenericObjectPool.this.evict();
            } catch (Exception e) {
            } catch (OutOfMemoryError oome) {
                oome.printStackTrace(System.err);
            }
            try {
                GenericObjectPool.this.ensureMinIdle();
            } catch (Exception e2) {
            }
        }
    }

    private static final class Latch<T> {
        private boolean _mayCreate;
        private GenericKeyedObjectPool.ObjectTimestampPair<T> _pair;

        private Latch() {
            this._mayCreate = false;
        }

        /* access modifiers changed from: private */
        public synchronized GenericKeyedObjectPool.ObjectTimestampPair<T> getPair() {
            return this._pair;
        }

        /* access modifiers changed from: private */
        public synchronized void setPair(GenericKeyedObjectPool.ObjectTimestampPair<T> pair) {
            this._pair = pair;
        }

        /* access modifiers changed from: private */
        public synchronized boolean mayCreate() {
            return this._mayCreate;
        }

        /* access modifiers changed from: private */
        public synchronized void setMayCreate(boolean mayCreate) {
            this._mayCreate = mayCreate;
        }

        /* access modifiers changed from: private */
        public synchronized void reset() {
            this._pair = null;
            this._mayCreate = false;
        }
    }
}
