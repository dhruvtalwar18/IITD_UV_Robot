package org.apache.commons.pool.impl;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TimerTask;
import java.util.TreeMap;
import org.apache.commons.pool.BaseKeyedObjectPool;
import org.apache.commons.pool.KeyedObjectPool;
import org.apache.commons.pool.KeyedPoolableObjectFactory;
import org.xbill.DNS.TTL;

public class GenericKeyedObjectPool<K, V> extends BaseKeyedObjectPool<K, V> implements KeyedObjectPool<K, V> {
    public static final boolean DEFAULT_LIFO = true;
    public static final int DEFAULT_MAX_ACTIVE = 8;
    public static final int DEFAULT_MAX_IDLE = 8;
    public static final int DEFAULT_MAX_TOTAL = -1;
    public static final long DEFAULT_MAX_WAIT = -1;
    public static final long DEFAULT_MIN_EVICTABLE_IDLE_TIME_MILLIS = 1800000;
    public static final int DEFAULT_MIN_IDLE = 0;
    public static final int DEFAULT_NUM_TESTS_PER_EVICTION_RUN = 3;
    public static final boolean DEFAULT_TEST_ON_BORROW = false;
    public static final boolean DEFAULT_TEST_ON_RETURN = false;
    public static final boolean DEFAULT_TEST_WHILE_IDLE = false;
    public static final long DEFAULT_TIME_BETWEEN_EVICTION_RUNS_MILLIS = -1;
    public static final byte DEFAULT_WHEN_EXHAUSTED_ACTION = 1;
    public static final byte WHEN_EXHAUSTED_BLOCK = 1;
    public static final byte WHEN_EXHAUSTED_FAIL = 0;
    public static final byte WHEN_EXHAUSTED_GROW = 2;
    private LinkedList<GenericKeyedObjectPool<K, V>.Latch<K, V>> _allocationQueue;
    private CursorableLinkedList<ObjectTimestampPair<V>>.Cursor _evictionCursor;
    private CursorableLinkedList<K>.Cursor _evictionKeyCursor;
    private GenericKeyedObjectPool<K, V>.Evictor _evictor;
    private KeyedPoolableObjectFactory<K, V> _factory;
    private boolean _lifo;
    private int _maxActive;
    private int _maxIdle;
    private int _maxTotal;
    private long _maxWait;
    private long _minEvictableIdleTimeMillis;
    private volatile int _minIdle;
    private int _numTestsPerEvictionRun;
    private CursorableLinkedList<K> _poolList;
    private Map<K, GenericKeyedObjectPool<K, V>.ObjectQueue> _poolMap;
    private volatile boolean _testOnBorrow;
    private volatile boolean _testOnReturn;
    private boolean _testWhileIdle;
    private long _timeBetweenEvictionRunsMillis;
    private int _totalActive;
    private int _totalIdle;
    private int _totalInternalProcessing;
    private byte _whenExhaustedAction;

    public static class Config {
        public boolean lifo = true;
        public int maxActive = 8;
        public int maxIdle = 8;
        public int maxTotal = -1;
        public long maxWait = -1;
        public long minEvictableIdleTimeMillis = 1800000;
        public int minIdle = 0;
        public int numTestsPerEvictionRun = 3;
        public boolean testOnBorrow = false;
        public boolean testOnReturn = false;
        public boolean testWhileIdle = false;
        public long timeBetweenEvictionRunsMillis = -1;
        public byte whenExhaustedAction = 1;
    }

    static /* synthetic */ int access$1408(GenericKeyedObjectPool x0) {
        int i = x0._totalActive;
        x0._totalActive = i + 1;
        return i;
    }

    static /* synthetic */ int access$1410(GenericKeyedObjectPool x0) {
        int i = x0._totalActive;
        x0._totalActive = i - 1;
        return i;
    }

    static /* synthetic */ int access$1508(GenericKeyedObjectPool x0) {
        int i = x0._totalInternalProcessing;
        x0._totalInternalProcessing = i + 1;
        return i;
    }

    static /* synthetic */ int access$1510(GenericKeyedObjectPool x0) {
        int i = x0._totalInternalProcessing;
        x0._totalInternalProcessing = i - 1;
        return i;
    }

    public GenericKeyedObjectPool() {
        this((KeyedPoolableObjectFactory) null, 8, (byte) 1, -1, 8, false, false, -1, 3, 1800000, false);
    }

    public GenericKeyedObjectPool(KeyedPoolableObjectFactory<K, V> factory) {
        this(factory, 8, (byte) 1, -1, 8, false, false, -1, 3, 1800000, false);
    }

    /* JADX WARNING: Illegal instructions before constructor call */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public GenericKeyedObjectPool(org.apache.commons.pool.KeyedPoolableObjectFactory<K, V> r22, org.apache.commons.pool.impl.GenericKeyedObjectPool.Config r23) {
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
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.pool.impl.GenericKeyedObjectPool.<init>(org.apache.commons.pool.KeyedPoolableObjectFactory, org.apache.commons.pool.impl.GenericKeyedObjectPool$Config):void");
    }

    public GenericKeyedObjectPool(KeyedPoolableObjectFactory<K, V> factory, int maxActive) {
        this(factory, maxActive, (byte) 1, -1, 8, false, false, -1, 3, 1800000, false);
    }

    public GenericKeyedObjectPool(KeyedPoolableObjectFactory<K, V> factory, int maxActive, byte whenExhaustedAction, long maxWait) {
        this(factory, maxActive, whenExhaustedAction, maxWait, 8, false, false, -1, 3, 1800000, false);
    }

    public GenericKeyedObjectPool(KeyedPoolableObjectFactory<K, V> factory, int maxActive, byte whenExhaustedAction, long maxWait, boolean testOnBorrow, boolean testOnReturn) {
        this(factory, maxActive, whenExhaustedAction, maxWait, 8, testOnBorrow, testOnReturn, -1, 3, 1800000, false);
    }

    public GenericKeyedObjectPool(KeyedPoolableObjectFactory<K, V> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle) {
        this(factory, maxActive, whenExhaustedAction, maxWait, maxIdle, false, false, -1, 3, 1800000, false);
    }

    public GenericKeyedObjectPool(KeyedPoolableObjectFactory<K, V> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle, boolean testOnBorrow, boolean testOnReturn) {
        this(factory, maxActive, whenExhaustedAction, maxWait, maxIdle, testOnBorrow, testOnReturn, -1, 3, 1800000, false);
    }

    public GenericKeyedObjectPool(KeyedPoolableObjectFactory<K, V> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle, boolean testOnBorrow, boolean testOnReturn, long timeBetweenEvictionRunsMillis, int numTestsPerEvictionRun, long minEvictableIdleTimeMillis, boolean testWhileIdle) {
        this(factory, maxActive, whenExhaustedAction, maxWait, maxIdle, -1, testOnBorrow, testOnReturn, timeBetweenEvictionRunsMillis, numTestsPerEvictionRun, minEvictableIdleTimeMillis, testWhileIdle);
    }

    public GenericKeyedObjectPool(KeyedPoolableObjectFactory<K, V> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle, int maxTotal, boolean testOnBorrow, boolean testOnReturn, long timeBetweenEvictionRunsMillis, int numTestsPerEvictionRun, long minEvictableIdleTimeMillis, boolean testWhileIdle) {
        this(factory, maxActive, whenExhaustedAction, maxWait, maxIdle, maxTotal, 0, testOnBorrow, testOnReturn, timeBetweenEvictionRunsMillis, numTestsPerEvictionRun, minEvictableIdleTimeMillis, testWhileIdle);
    }

    public GenericKeyedObjectPool(KeyedPoolableObjectFactory<K, V> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle, int maxTotal, int minIdle, boolean testOnBorrow, boolean testOnReturn, long timeBetweenEvictionRunsMillis, int numTestsPerEvictionRun, long minEvictableIdleTimeMillis, boolean testWhileIdle) {
        this(factory, maxActive, whenExhaustedAction, maxWait, maxIdle, maxTotal, minIdle, testOnBorrow, testOnReturn, timeBetweenEvictionRunsMillis, numTestsPerEvictionRun, minEvictableIdleTimeMillis, testWhileIdle, true);
    }

    public GenericKeyedObjectPool(KeyedPoolableObjectFactory<K, V> factory, int maxActive, byte whenExhaustedAction, long maxWait, int maxIdle, int maxTotal, int minIdle, boolean testOnBorrow, boolean testOnReturn, long timeBetweenEvictionRunsMillis, int numTestsPerEvictionRun, long minEvictableIdleTimeMillis, boolean testWhileIdle, boolean lifo) {
        byte b = whenExhaustedAction;
        this._maxIdle = 8;
        this._minIdle = 0;
        this._maxActive = 8;
        this._maxTotal = -1;
        this._maxWait = -1;
        this._whenExhaustedAction = 1;
        this._testOnBorrow = false;
        this._testOnReturn = false;
        this._testWhileIdle = false;
        this._timeBetweenEvictionRunsMillis = -1;
        this._numTestsPerEvictionRun = 3;
        this._minEvictableIdleTimeMillis = 1800000;
        this._poolMap = null;
        this._totalActive = 0;
        this._totalIdle = 0;
        this._totalInternalProcessing = 0;
        this._factory = null;
        this._evictor = null;
        this._poolList = null;
        this._evictionCursor = null;
        this._evictionKeyCursor = null;
        this._lifo = true;
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
                this._maxTotal = maxTotal;
                this._minIdle = minIdle;
                this._testOnBorrow = testOnBorrow;
                this._testOnReturn = testOnReturn;
                this._timeBetweenEvictionRunsMillis = timeBetweenEvictionRunsMillis;
                this._numTestsPerEvictionRun = numTestsPerEvictionRun;
                this._minEvictableIdleTimeMillis = minEvictableIdleTimeMillis;
                this._testWhileIdle = testWhileIdle;
                this._poolMap = new HashMap();
                this._poolList = new CursorableLinkedList<>();
                startEvictor(this._timeBetweenEvictionRunsMillis);
                return;
            default:
                long j = maxWait;
                int i = maxIdle;
                int i2 = maxTotal;
                int i3 = minIdle;
                boolean z = testOnBorrow;
                boolean z2 = testOnReturn;
                long j2 = timeBetweenEvictionRunsMillis;
                int i4 = numTestsPerEvictionRun;
                boolean z3 = testWhileIdle;
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

    public synchronized int getMaxTotal() {
        return this._maxTotal;
    }

    public void setMaxTotal(int maxTotal) {
        synchronized (this) {
            this._maxTotal = maxTotal;
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

    public void setMinIdle(int poolSize) {
        this._minIdle = poolSize;
    }

    public int getMinIdle() {
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

    public synchronized boolean getTestWhileIdle() {
        return this._testWhileIdle;
    }

    public synchronized void setTestWhileIdle(boolean testWhileIdle) {
        this._testWhileIdle = testWhileIdle;
    }

    public synchronized void setConfig(Config conf) {
        setMaxIdle(conf.maxIdle);
        setMaxActive(conf.maxActive);
        setMaxTotal(conf.maxTotal);
        setMinIdle(conf.minIdle);
        setMaxWait(conf.maxWait);
        setWhenExhaustedAction(conf.whenExhaustedAction);
        setTestOnBorrow(conf.testOnBorrow);
        setTestOnReturn(conf.testOnReturn);
        setTestWhileIdle(conf.testWhileIdle);
        setNumTestsPerEvictionRun(conf.numTestsPerEvictionRun);
        setMinEvictableIdleTimeMillis(conf.minEvictableIdleTimeMillis);
        setTimeBetweenEvictionRunsMillis(conf.timeBetweenEvictionRunsMillis);
    }

    public synchronized boolean getLifo() {
        return this._lifo;
    }

    public synchronized void setLifo(boolean lifo) {
        this._lifo = lifo;
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
    public V borrowObject(K r15) throws java.lang.Exception {
        /*
            r14 = this;
            long r0 = java.lang.System.currentTimeMillis()
            org.apache.commons.pool.impl.GenericKeyedObjectPool$Latch r2 = new org.apache.commons.pool.impl.GenericKeyedObjectPool$Latch
            r3 = 0
            r2.<init>(r15)
            monitor-enter(r14)
            r3 = 0
            r4 = 0
            byte r6 = r14._whenExhaustedAction     // Catch:{ all -> 0x0221 }
            long r7 = r14._maxWait     // Catch:{ all -> 0x021f }
            java.util.LinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool<K, V>$Latch<K, V>> r9 = r14._allocationQueue     // Catch:{ all -> 0x021c }
            r9.add(r2)     // Catch:{ all -> 0x021c }
            monitor-exit(r14)     // Catch:{ all -> 0x021c }
            r14.allocate()
        L_0x001b:
            monitor-enter(r14)
            r14.assertOpen()     // Catch:{ all -> 0x0219 }
            monitor-exit(r14)     // Catch:{ all -> 0x0219 }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r9 = r2.getPair()
            if (r9 != 0) goto L_0x0151
            boolean r9 = r2.mayCreate()
            if (r9 == 0) goto L_0x002e
            goto L_0x0151
        L_0x002e:
            switch(r6) {
                case 0: goto L_0x0130;
                case 1: goto L_0x006c;
                case 2: goto L_0x004d;
                default: goto L_0x0031;
            }
        L_0x0031:
            java.lang.IllegalArgumentException r3 = new java.lang.IllegalArgumentException
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r5 = "whenExhaustedAction "
            r4.append(r5)
            r4.append(r6)
            java.lang.String r5 = " not recognized."
            r4.append(r5)
            java.lang.String r4 = r4.toString()
            r3.<init>(r4)
            throw r3
        L_0x004d:
            monitor-enter(r14)
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r9 = r2.getPair()     // Catch:{ all -> 0x0069 }
            if (r9 != 0) goto L_0x0066
            boolean r9 = r2.mayCreate()     // Catch:{ all -> 0x0069 }
            if (r9 != 0) goto L_0x0066
            java.util.LinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool<K, V>$Latch<K, V>> r9 = r14._allocationQueue     // Catch:{ all -> 0x0069 }
            r9.remove(r2)     // Catch:{ all -> 0x0069 }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectQueue r9 = r2.getPool()     // Catch:{ all -> 0x0069 }
            r9.incrementInternalProcessingCount()     // Catch:{ all -> 0x0069 }
        L_0x0066:
            monitor-exit(r14)     // Catch:{ all -> 0x0069 }
            goto L_0x0151
        L_0x0069:
            r3 = move-exception
            monitor-exit(r14)     // Catch:{ all -> 0x0069 }
            throw r3
        L_0x006c:
            monitor-enter(r2)     // Catch:{ InterruptedException -> 0x00d7 }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r9 = r2.getPair()     // Catch:{ all -> 0x00d4 }
            if (r9 != 0) goto L_0x00d1
            boolean r9 = r2.mayCreate()     // Catch:{ all -> 0x00d4 }
            if (r9 != 0) goto L_0x00d1
            int r9 = (r7 > r4 ? 1 : (r7 == r4 ? 0 : -1))
            if (r9 > 0) goto L_0x0081
            r2.wait()     // Catch:{ all -> 0x00d4 }
            goto L_0x0091
        L_0x0081:
            long r9 = java.lang.System.currentTimeMillis()     // Catch:{ all -> 0x00d4 }
            r11 = 0
            long r9 = r9 - r0
            long r11 = r7 - r9
            int r13 = (r11 > r4 ? 1 : (r11 == r4 ? 0 : -1))
            if (r13 <= 0) goto L_0x0090
            r2.wait(r11)     // Catch:{ all -> 0x00d4 }
        L_0x0090:
        L_0x0091:
            monitor-exit(r2)     // Catch:{ all -> 0x00d4 }
            boolean r9 = r14.isClosed()     // Catch:{ InterruptedException -> 0x00d7 }
            r10 = 1
            if (r9 == r10) goto L_0x00c9
            int r9 = (r7 > r4 ? 1 : (r7 == r4 ? 0 : -1))
            if (r9 <= 0) goto L_0x001b
            long r9 = java.lang.System.currentTimeMillis()
            long r9 = r9 - r0
            int r11 = (r9 > r7 ? 1 : (r9 == r7 ? 0 : -1))
            if (r11 < 0) goto L_0x001b
            monitor-enter(r14)
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r9 = r2.getPair()     // Catch:{ all -> 0x00c6 }
            if (r9 != 0) goto L_0x00c3
            boolean r9 = r2.mayCreate()     // Catch:{ all -> 0x00c6 }
            if (r9 == 0) goto L_0x00b5
            goto L_0x00c3
        L_0x00b5:
            java.util.LinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool<K, V>$Latch<K, V>> r3 = r14._allocationQueue     // Catch:{ all -> 0x00c6 }
            r3.remove(r2)     // Catch:{ all -> 0x00c6 }
            monitor-exit(r14)     // Catch:{ all -> 0x00c6 }
            java.util.NoSuchElementException r3 = new java.util.NoSuchElementException
            java.lang.String r4 = "Timeout waiting for idle object"
            r3.<init>(r4)
            throw r3
        L_0x00c3:
            monitor-exit(r14)     // Catch:{ all -> 0x00c6 }
            goto L_0x0151
        L_0x00c6:
            r3 = move-exception
            monitor-exit(r14)     // Catch:{ all -> 0x00c6 }
            throw r3
        L_0x00c9:
            java.lang.IllegalStateException r3 = new java.lang.IllegalStateException     // Catch:{ InterruptedException -> 0x00d7 }
            java.lang.String r4 = "Pool closed"
            r3.<init>(r4)     // Catch:{ InterruptedException -> 0x00d7 }
            throw r3     // Catch:{ InterruptedException -> 0x00d7 }
        L_0x00d1:
            monitor-exit(r2)     // Catch:{ all -> 0x00d4 }
            goto L_0x0151
        L_0x00d4:
            r3 = move-exception
            monitor-exit(r2)     // Catch:{ all -> 0x00d4 }
            throw r3     // Catch:{ InterruptedException -> 0x00d7 }
        L_0x00d7:
            r3 = move-exception
            r9 = r3
            r3 = 0
            monitor-enter(r14)
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r4 = r2.getPair()     // Catch:{ all -> 0x012d }
            if (r4 != 0) goto L_0x00ed
            boolean r4 = r2.mayCreate()     // Catch:{ all -> 0x012d }
            if (r4 != 0) goto L_0x00ed
            java.util.LinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool<K, V>$Latch<K, V>> r4 = r14._allocationQueue     // Catch:{ all -> 0x012d }
            r4.remove(r2)     // Catch:{ all -> 0x012d }
            goto L_0x011f
        L_0x00ed:
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r4 = r2.getPair()     // Catch:{ all -> 0x012d }
            if (r4 != 0) goto L_0x0102
            boolean r4 = r2.mayCreate()     // Catch:{ all -> 0x012d }
            if (r4 == 0) goto L_0x0102
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectQueue r4 = r2.getPool()     // Catch:{ all -> 0x012d }
            r4.decrementInternalProcessingCount()     // Catch:{ all -> 0x012d }
            r3 = 1
            goto L_0x011f
        L_0x0102:
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectQueue r4 = r2.getPool()     // Catch:{ all -> 0x012d }
            r4.decrementInternalProcessingCount()     // Catch:{ all -> 0x012d }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectQueue r4 = r2.getPool()     // Catch:{ all -> 0x012d }
            r4.incrementActiveCount()     // Catch:{ all -> 0x012d }
            java.lang.Object r4 = r2.getkey()     // Catch:{ all -> 0x012d }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r5 = r2.getPair()     // Catch:{ all -> 0x012d }
            java.lang.Object r5 = r5.getValue()     // Catch:{ all -> 0x012d }
            r14.returnObject(r4, r5)     // Catch:{ all -> 0x012d }
        L_0x011f:
            monitor-exit(r14)     // Catch:{ all -> 0x012d }
            if (r3 == 0) goto L_0x0125
            r14.allocate()
        L_0x0125:
            java.lang.Thread r4 = java.lang.Thread.currentThread()
            r4.interrupt()
            throw r9
        L_0x012d:
            r4 = move-exception
            monitor-exit(r14)     // Catch:{ all -> 0x012d }
            throw r4
        L_0x0130:
            monitor-enter(r14)
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r9 = r2.getPair()     // Catch:{ all -> 0x014e }
            if (r9 != 0) goto L_0x014c
            boolean r9 = r2.mayCreate()     // Catch:{ all -> 0x014e }
            if (r9 == 0) goto L_0x013e
            goto L_0x014c
        L_0x013e:
            java.util.LinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool<K, V>$Latch<K, V>> r3 = r14._allocationQueue     // Catch:{ all -> 0x014e }
            r3.remove(r2)     // Catch:{ all -> 0x014e }
            monitor-exit(r14)     // Catch:{ all -> 0x014e }
            java.util.NoSuchElementException r3 = new java.util.NoSuchElementException
            java.lang.String r4 = "Pool exhausted"
            r3.<init>(r4)
            throw r3
        L_0x014c:
            monitor-exit(r14)     // Catch:{ all -> 0x014e }
            goto L_0x0151
        L_0x014e:
            r3 = move-exception
            monitor-exit(r14)     // Catch:{ all -> 0x014e }
            throw r3
        L_0x0151:
            r9 = 0
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r10 = r2.getPair()
            if (r10 != 0) goto L_0x018d
            org.apache.commons.pool.KeyedPoolableObjectFactory<K, V> r10 = r14._factory     // Catch:{ all -> 0x0179 }
            java.lang.Object r10 = r10.makeObject(r15)     // Catch:{ all -> 0x0179 }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r11 = new org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair     // Catch:{ all -> 0x0179 }
            r11.<init>(r10)     // Catch:{ all -> 0x0179 }
            r2.setPair(r11)     // Catch:{ all -> 0x0179 }
            r9 = 1
            if (r9 != 0) goto L_0x018d
            monitor-enter(r14)
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectQueue r10 = r2.getPool()     // Catch:{ all -> 0x0176 }
            r10.decrementInternalProcessingCount()     // Catch:{ all -> 0x0176 }
            monitor-exit(r14)     // Catch:{ all -> 0x0176 }
            r14.allocate()
            goto L_0x018d
        L_0x0176:
            r3 = move-exception
            monitor-exit(r14)     // Catch:{ all -> 0x0176 }
            throw r3
        L_0x0179:
            r3 = move-exception
            if (r9 != 0) goto L_0x018c
            monitor-enter(r14)
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectQueue r4 = r2.getPool()     // Catch:{ all -> 0x0189 }
            r4.decrementInternalProcessingCount()     // Catch:{ all -> 0x0189 }
            monitor-exit(r14)     // Catch:{ all -> 0x0189 }
            r14.allocate()
            goto L_0x018c
        L_0x0189:
            r3 = move-exception
            monitor-exit(r14)     // Catch:{ all -> 0x0189 }
            throw r3
        L_0x018c:
            throw r3
        L_0x018d:
            org.apache.commons.pool.KeyedPoolableObjectFactory<K, V> r10 = r14._factory     // Catch:{ Throwable -> 0x01cd }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r11 = r2.getPair()     // Catch:{ Throwable -> 0x01cd }
            T r11 = r11.value     // Catch:{ Throwable -> 0x01cd }
            r10.activateObject(r15, r11)     // Catch:{ Throwable -> 0x01cd }
            boolean r10 = r14._testOnBorrow     // Catch:{ Throwable -> 0x01cd }
            if (r10 == 0) goto L_0x01b3
            org.apache.commons.pool.KeyedPoolableObjectFactory<K, V> r10 = r14._factory     // Catch:{ Throwable -> 0x01cd }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r11 = r2.getPair()     // Catch:{ Throwable -> 0x01cd }
            T r11 = r11.value     // Catch:{ Throwable -> 0x01cd }
            boolean r10 = r10.validateObject(r15, r11)     // Catch:{ Throwable -> 0x01cd }
            if (r10 == 0) goto L_0x01ab
            goto L_0x01b3
        L_0x01ab:
            java.lang.Exception r10 = new java.lang.Exception     // Catch:{ Throwable -> 0x01cd }
            java.lang.String r11 = "ValidateObject failed"
            r10.<init>(r11)     // Catch:{ Throwable -> 0x01cd }
            throw r10     // Catch:{ Throwable -> 0x01cd }
        L_0x01b3:
            monitor-enter(r14)     // Catch:{ Throwable -> 0x01cd }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectQueue r10 = r2.getPool()     // Catch:{ all -> 0x01ca }
            r10.decrementInternalProcessingCount()     // Catch:{ all -> 0x01ca }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectQueue r10 = r2.getPool()     // Catch:{ all -> 0x01ca }
            r10.incrementActiveCount()     // Catch:{ all -> 0x01ca }
            monitor-exit(r14)     // Catch:{ all -> 0x01ca }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r10 = r2.getPair()     // Catch:{ Throwable -> 0x01cd }
            T r10 = r10.value     // Catch:{ Throwable -> 0x01cd }
            return r10
        L_0x01ca:
            r10 = move-exception
            monitor-exit(r14)     // Catch:{ all -> 0x01ca }
            throw r10     // Catch:{ Throwable -> 0x01cd }
        L_0x01cd:
            r10 = move-exception
            org.apache.commons.pool.PoolUtils.checkRethrow(r10)
            org.apache.commons.pool.KeyedPoolableObjectFactory<K, V> r11 = r14._factory     // Catch:{ Throwable -> 0x01dd }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r12 = r2.getPair()     // Catch:{ Throwable -> 0x01dd }
            T r12 = r12.value     // Catch:{ Throwable -> 0x01dd }
            r11.destroyObject(r15, r12)     // Catch:{ Throwable -> 0x01dd }
            goto L_0x01e1
        L_0x01dd:
            r11 = move-exception
            org.apache.commons.pool.PoolUtils.checkRethrow(r11)
        L_0x01e1:
            monitor-enter(r14)
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectQueue r11 = r2.getPool()     // Catch:{ all -> 0x0216 }
            r11.decrementInternalProcessingCount()     // Catch:{ all -> 0x0216 }
            if (r9 != 0) goto L_0x01f3
            r2.reset()     // Catch:{ all -> 0x0216 }
            java.util.LinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool<K, V>$Latch<K, V>> r11 = r14._allocationQueue     // Catch:{ all -> 0x0216 }
            r11.add(r3, r2)     // Catch:{ all -> 0x0216 }
        L_0x01f3:
            monitor-exit(r14)     // Catch:{ all -> 0x0216 }
            r14.allocate()
            if (r9 != 0) goto L_0x01fb
            goto L_0x001b
        L_0x01fb:
            java.util.NoSuchElementException r3 = new java.util.NoSuchElementException
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r5 = "Could not create a validated object, cause: "
            r4.append(r5)
            java.lang.String r5 = r10.getMessage()
            r4.append(r5)
            java.lang.String r4 = r4.toString()
            r3.<init>(r4)
            throw r3
        L_0x0216:
            r3 = move-exception
            monitor-exit(r14)     // Catch:{ all -> 0x0216 }
            throw r3
        L_0x0219:
            r3 = move-exception
            monitor-exit(r14)     // Catch:{ all -> 0x0219 }
            throw r3
        L_0x021c:
            r3 = move-exception
            r4 = r7
            goto L_0x0224
        L_0x021f:
            r3 = move-exception
            goto L_0x0224
        L_0x0221:
            r6 = move-exception
            r3 = r6
            r6 = 0
        L_0x0224:
            monitor-exit(r14)     // Catch:{ all -> 0x021f }
            throw r3
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.pool.impl.GenericKeyedObjectPool.borrowObject(java.lang.Object):java.lang.Object");
    }

    /* JADX WARNING: Code restructure failed: missing block: B:54:0x00c1, code lost:
        if (r0 == false) goto L_?;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:55:0x00c3, code lost:
        clearOldest();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:71:?, code lost:
        return;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:72:?, code lost:
        return;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private void allocate() {
        /*
            r7 = this;
            r0 = 0
            monitor-enter(r7)
            boolean r1 = r7.isClosed()     // Catch:{ all -> 0x00c7 }
            if (r1 == 0) goto L_0x000a
            monitor-exit(r7)     // Catch:{ all -> 0x00c7 }
            return
        L_0x000a:
            java.util.LinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool<K, V>$Latch<K, V>> r1 = r7._allocationQueue     // Catch:{ all -> 0x00c7 }
            java.util.Iterator r1 = r1.iterator()     // Catch:{ all -> 0x00c7 }
        L_0x0010:
            boolean r2 = r1.hasNext()     // Catch:{ all -> 0x00c7 }
            if (r2 == 0) goto L_0x00c0
            java.lang.Object r2 = r1.next()     // Catch:{ all -> 0x00c7 }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$Latch r2 = (org.apache.commons.pool.impl.GenericKeyedObjectPool.Latch) r2     // Catch:{ all -> 0x00c7 }
            java.util.Map<K, org.apache.commons.pool.impl.GenericKeyedObjectPool<K, V>$ObjectQueue> r3 = r7._poolMap     // Catch:{ all -> 0x00c7 }
            java.lang.Object r4 = r2.getkey()     // Catch:{ all -> 0x00c7 }
            java.lang.Object r3 = r3.get(r4)     // Catch:{ all -> 0x00c7 }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectQueue r3 = (org.apache.commons.pool.impl.GenericKeyedObjectPool.ObjectQueue) r3     // Catch:{ all -> 0x00c7 }
            if (r3 != 0) goto L_0x0043
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectQueue r4 = new org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectQueue     // Catch:{ all -> 0x00c7 }
            r5 = 0
            r4.<init>()     // Catch:{ all -> 0x00c7 }
            r3 = r4
            java.util.Map<K, org.apache.commons.pool.impl.GenericKeyedObjectPool<K, V>$ObjectQueue> r4 = r7._poolMap     // Catch:{ all -> 0x00c7 }
            java.lang.Object r5 = r2.getkey()     // Catch:{ all -> 0x00c7 }
            r4.put(r5, r3)     // Catch:{ all -> 0x00c7 }
            org.apache.commons.pool.impl.CursorableLinkedList<K> r4 = r7._poolList     // Catch:{ all -> 0x00c7 }
            java.lang.Object r5 = r2.getkey()     // Catch:{ all -> 0x00c7 }
            r4.add(r5)     // Catch:{ all -> 0x00c7 }
        L_0x0043:
            r2.setPool(r3)     // Catch:{ all -> 0x00c7 }
            org.apache.commons.pool.impl.CursorableLinkedList r4 = r3.queue     // Catch:{ all -> 0x00c7 }
            boolean r4 = r4.isEmpty()     // Catch:{ all -> 0x00c7 }
            r5 = 1
            if (r4 != 0) goto L_0x0072
            r1.remove()     // Catch:{ all -> 0x00c7 }
            org.apache.commons.pool.impl.CursorableLinkedList r4 = r3.queue     // Catch:{ all -> 0x00c7 }
            java.lang.Object r4 = r4.removeFirst()     // Catch:{ all -> 0x00c7 }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r4 = (org.apache.commons.pool.impl.GenericKeyedObjectPool.ObjectTimestampPair) r4     // Catch:{ all -> 0x00c7 }
            r2.setPair(r4)     // Catch:{ all -> 0x00c7 }
            r3.incrementInternalProcessingCount()     // Catch:{ all -> 0x00c7 }
            int r4 = r7._totalIdle     // Catch:{ all -> 0x00c7 }
            int r4 = r4 - r5
            r7._totalIdle = r4     // Catch:{ all -> 0x00c7 }
            monitor-enter(r2)     // Catch:{ all -> 0x00c7 }
            r2.notify()     // Catch:{ all -> 0x006f }
            monitor-exit(r2)     // Catch:{ all -> 0x006f }
            goto L_0x0010
        L_0x006f:
            r4 = move-exception
            monitor-exit(r2)     // Catch:{ all -> 0x006f }
            throw r4     // Catch:{ all -> 0x00c7 }
        L_0x0072:
            int r4 = r7._maxTotal     // Catch:{ all -> 0x00c7 }
            if (r4 <= 0) goto L_0x0084
            int r4 = r7._totalActive     // Catch:{ all -> 0x00c7 }
            int r6 = r7._totalIdle     // Catch:{ all -> 0x00c7 }
            int r4 = r4 + r6
            int r6 = r7._totalInternalProcessing     // Catch:{ all -> 0x00c7 }
            int r4 = r4 + r6
            int r6 = r7._maxTotal     // Catch:{ all -> 0x00c7 }
            if (r4 < r6) goto L_0x0084
            r0 = 1
            goto L_0x00c0
        L_0x0084:
            int r4 = r7._maxActive     // Catch:{ all -> 0x00c7 }
            if (r4 < 0) goto L_0x0095
            int r4 = r3.activeCount     // Catch:{ all -> 0x00c7 }
            int r6 = r3.internalProcessingCount     // Catch:{ all -> 0x00c7 }
            int r4 = r4 + r6
            int r6 = r7._maxActive     // Catch:{ all -> 0x00c7 }
            if (r4 >= r6) goto L_0x00a6
        L_0x0095:
            int r4 = r7._maxTotal     // Catch:{ all -> 0x00c7 }
            if (r4 < 0) goto L_0x00ad
            int r4 = r7._totalActive     // Catch:{ all -> 0x00c7 }
            int r6 = r7._totalIdle     // Catch:{ all -> 0x00c7 }
            int r4 = r4 + r6
            int r6 = r7._totalInternalProcessing     // Catch:{ all -> 0x00c7 }
            int r4 = r4 + r6
            int r6 = r7._maxTotal     // Catch:{ all -> 0x00c7 }
            if (r4 >= r6) goto L_0x00a6
            goto L_0x00ad
        L_0x00a6:
            int r4 = r7._maxActive     // Catch:{ all -> 0x00c7 }
            if (r4 >= 0) goto L_0x00ab
            goto L_0x00c0
        L_0x00ab:
            goto L_0x0010
        L_0x00ad:
            r1.remove()     // Catch:{ all -> 0x00c7 }
            r2.setMayCreate(r5)     // Catch:{ all -> 0x00c7 }
            r3.incrementInternalProcessingCount()     // Catch:{ all -> 0x00c7 }
            monitor-enter(r2)     // Catch:{ all -> 0x00c7 }
            r2.notify()     // Catch:{ all -> 0x00bd }
            monitor-exit(r2)     // Catch:{ all -> 0x00bd }
            goto L_0x0010
        L_0x00bd:
            r4 = move-exception
            monitor-exit(r2)     // Catch:{ all -> 0x00bd }
            throw r4     // Catch:{ all -> 0x00c7 }
        L_0x00c0:
            monitor-exit(r7)     // Catch:{ all -> 0x00c7 }
            if (r0 == 0) goto L_0x00c6
            r7.clearOldest()
        L_0x00c6:
            return
        L_0x00c7:
            r1 = move-exception
            monitor-exit(r7)     // Catch:{ all -> 0x00c7 }
            throw r1
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.pool.impl.GenericKeyedObjectPool.allocate():void");
    }

    public void clear() {
        Map<K, List<ObjectTimestampPair<V>>> toDestroy = new HashMap<>();
        synchronized (this) {
            Iterator<K> it = this._poolMap.keySet().iterator();
            while (it.hasNext()) {
                K key = it.next();
                GenericKeyedObjectPool<K, V>.ObjectQueue pool = this._poolMap.get(key);
                List<ObjectTimestampPair<V>> objects = new ArrayList<>();
                objects.addAll(pool.queue);
                toDestroy.put(key, objects);
                it.remove();
                this._poolList.remove((Object) key);
                this._totalIdle -= pool.queue.size();
                this._totalInternalProcessing += pool.queue.size();
                pool.queue.clear();
            }
        }
        destroy(toDestroy, this._factory);
    }

    public void clearOldest() {
        Map<K, List<ObjectTimestampPair<V>>> toDestroy = new HashMap<>();
        Map<ObjectTimestampPair<V>, K> map = new TreeMap<>();
        synchronized (this) {
            for (K key : this._poolMap.keySet()) {
                for (ObjectTimestampPair<V> put : this._poolMap.get(key).queue) {
                    map.put(put, key);
                }
            }
            Set<Map.Entry<ObjectTimestampPair<V>, K>> setPairKeys = map.entrySet();
            double size = (double) map.size();
            Double.isNaN(size);
            int itemsToRemove = ((int) (size * 0.15d)) + 1;
            Iterator<Map.Entry<ObjectTimestampPair<V>, K>> iter = setPairKeys.iterator();
            while (iter.hasNext() && itemsToRemove > 0) {
                Map.Entry<ObjectTimestampPair<V>, K> entry = iter.next();
                K key2 = entry.getValue();
                ObjectTimestampPair<V> pairTimeStamp = entry.getKey();
                GenericKeyedObjectPool<K, V>.ObjectQueue objectQueue = this._poolMap.get(key2);
                objectQueue.queue.remove(pairTimeStamp);
                if (toDestroy.containsKey(key2)) {
                    toDestroy.get(key2).add(pairTimeStamp);
                } else {
                    List<ObjectTimestampPair<V>> listForKey = new ArrayList<>();
                    listForKey.add(pairTimeStamp);
                    toDestroy.put(key2, listForKey);
                }
                objectQueue.incrementInternalProcessingCount();
                this._totalIdle--;
                itemsToRemove--;
            }
        }
        destroy(toDestroy, this._factory);
    }

    public void clear(K key) {
        Map<K, List<ObjectTimestampPair<V>>> toDestroy = new HashMap<>();
        synchronized (this) {
            GenericKeyedObjectPool<K, V>.ObjectQueue pool = this._poolMap.remove(key);
            if (pool != null) {
                this._poolList.remove((Object) key);
                List<ObjectTimestampPair<V>> objects = new ArrayList<>();
                objects.addAll(pool.queue);
                toDestroy.put(key, objects);
                this._totalIdle -= pool.queue.size();
                this._totalInternalProcessing += pool.queue.size();
                pool.queue.clear();
                destroy(toDestroy, this._factory);
            }
        }
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
    private void destroy(java.util.Map<K, java.util.List<org.apache.commons.pool.impl.GenericKeyedObjectPool.ObjectTimestampPair<V>>> r18, org.apache.commons.pool.KeyedPoolableObjectFactory<K, V> r19) {
        /*
            r17 = this;
            java.util.Set r0 = r18.entrySet()
            java.util.Iterator r0 = r0.iterator()
            r1 = r17
            r2 = r18
            r3 = r19
        L_0x000e:
            boolean r4 = r0.hasNext()
            if (r4 == 0) goto L_0x011b
            java.lang.Object r4 = r0.next()
            java.util.Map$Entry r4 = (java.util.Map.Entry) r4
            java.lang.Object r5 = r4.getKey()
            java.lang.Object r6 = r4.getValue()
            java.util.List r6 = (java.util.List) r6
            java.util.Iterator r7 = r6.iterator()
            r8 = r0
        L_0x0029:
            boolean r0 = r7.hasNext()
            if (r0 == 0) goto L_0x0117
            java.lang.Object r0 = r7.next()     // Catch:{ Exception -> 0x00ce, all -> 0x0087 }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r0 = (org.apache.commons.pool.impl.GenericKeyedObjectPool.ObjectTimestampPair) r0     // Catch:{ Exception -> 0x00ce, all -> 0x0087 }
            T r0 = r0.value     // Catch:{ Exception -> 0x00ce, all -> 0x0087 }
            r3.destroyObject(r5, r0)     // Catch:{ Exception -> 0x00ce, all -> 0x0087 }
            r9 = r4
            r10 = r5
            r11 = r6
            r12 = r8
            r13 = r1
            r14 = r2
            r15 = r3
            monitor-enter(r13)
            java.util.Map<K, org.apache.commons.pool.impl.GenericKeyedObjectPool<K, V>$ObjectQueue> r0 = r13._poolMap     // Catch:{ all -> 0x0084 }
            java.lang.Object r0 = r0.get(r10)     // Catch:{ all -> 0x0084 }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectQueue r0 = (org.apache.commons.pool.impl.GenericKeyedObjectPool.ObjectQueue) r0     // Catch:{ all -> 0x0084 }
            if (r0 == 0) goto L_0x0071
            r0.decrementInternalProcessingCount()     // Catch:{ all -> 0x0084 }
            int r1 = r0.internalProcessingCount     // Catch:{ all -> 0x0084 }
            if (r1 != 0) goto L_0x0077
            int r1 = r0.activeCount     // Catch:{ all -> 0x0084 }
            if (r1 != 0) goto L_0x0077
            org.apache.commons.pool.impl.CursorableLinkedList r1 = r0.queue     // Catch:{ all -> 0x0084 }
            boolean r1 = r1.isEmpty()     // Catch:{ all -> 0x0084 }
            if (r1 == 0) goto L_0x0077
            java.util.Map<K, org.apache.commons.pool.impl.GenericKeyedObjectPool<K, V>$ObjectQueue> r1 = r13._poolMap     // Catch:{ all -> 0x0084 }
            r1.remove(r10)     // Catch:{ all -> 0x0084 }
            org.apache.commons.pool.impl.CursorableLinkedList<K> r1 = r13._poolList     // Catch:{ all -> 0x0084 }
            r1.remove((java.lang.Object) r10)     // Catch:{ all -> 0x0084 }
            goto L_0x0077
        L_0x0071:
            int r1 = r13._totalInternalProcessing     // Catch:{ all -> 0x0084 }
            int r1 = r1 + -1
            r13._totalInternalProcessing = r1     // Catch:{ all -> 0x0084 }
        L_0x0077:
            monitor-exit(r13)     // Catch:{ all -> 0x0084 }
            r13.allocate()
            r4 = r9
            r5 = r10
            r6 = r11
            r8 = r12
            r1 = r13
            r2 = r14
            r3 = r15
            goto L_0x0029
        L_0x0084:
            r0 = move-exception
            monitor-exit(r13)     // Catch:{ all -> 0x0084 }
            throw r0
        L_0x0087:
            r0 = move-exception
            r9 = r7
            r10 = r4
            r11 = r5
            r12 = r6
            r13 = r8
            r14 = r1
            r15 = r2
            r16 = r3
            monitor-enter(r14)
            java.util.Map<K, org.apache.commons.pool.impl.GenericKeyedObjectPool<K, V>$ObjectQueue> r1 = r14._poolMap     // Catch:{ all -> 0x00cb }
            java.lang.Object r1 = r1.get(r11)     // Catch:{ all -> 0x00cb }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectQueue r1 = (org.apache.commons.pool.impl.GenericKeyedObjectPool.ObjectQueue) r1     // Catch:{ all -> 0x00cb }
            if (r1 == 0) goto L_0x00c0
            r1.decrementInternalProcessingCount()     // Catch:{ all -> 0x00cb }
            int r2 = r1.internalProcessingCount     // Catch:{ all -> 0x00cb }
            if (r2 != 0) goto L_0x00c6
            int r2 = r1.activeCount     // Catch:{ all -> 0x00cb }
            if (r2 != 0) goto L_0x00c6
            org.apache.commons.pool.impl.CursorableLinkedList r2 = r1.queue     // Catch:{ all -> 0x00cb }
            boolean r2 = r2.isEmpty()     // Catch:{ all -> 0x00cb }
            if (r2 == 0) goto L_0x00c6
            java.util.Map<K, org.apache.commons.pool.impl.GenericKeyedObjectPool<K, V>$ObjectQueue> r2 = r14._poolMap     // Catch:{ all -> 0x00cb }
            r2.remove(r11)     // Catch:{ all -> 0x00cb }
            org.apache.commons.pool.impl.CursorableLinkedList<K> r2 = r14._poolList     // Catch:{ all -> 0x00cb }
            r2.remove((java.lang.Object) r11)     // Catch:{ all -> 0x00cb }
            goto L_0x00c6
        L_0x00c0:
            int r2 = r14._totalInternalProcessing     // Catch:{ all -> 0x00cb }
            int r2 = r2 + -1
            r14._totalInternalProcessing = r2     // Catch:{ all -> 0x00cb }
        L_0x00c6:
            monitor-exit(r14)     // Catch:{ all -> 0x00cb }
            r14.allocate()
            throw r0
        L_0x00cb:
            r0 = move-exception
            monitor-exit(r14)     // Catch:{ all -> 0x00cb }
            throw r0
        L_0x00ce:
            r0 = move-exception
            r9 = r8
            r10 = r1
            r11 = r2
            r12 = r3
            monitor-enter(r10)
            java.util.Map<K, org.apache.commons.pool.impl.GenericKeyedObjectPool<K, V>$ObjectQueue> r0 = r10._poolMap     // Catch:{ all -> 0x0114 }
            java.lang.Object r0 = r0.get(r5)     // Catch:{ all -> 0x0114 }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectQueue r0 = (org.apache.commons.pool.impl.GenericKeyedObjectPool.ObjectQueue) r0     // Catch:{ all -> 0x0114 }
            if (r0 == 0) goto L_0x0103
            r0.decrementInternalProcessingCount()     // Catch:{ all -> 0x0114 }
            int r1 = r0.internalProcessingCount     // Catch:{ all -> 0x0114 }
            if (r1 != 0) goto L_0x0109
            int r1 = r0.activeCount     // Catch:{ all -> 0x0114 }
            if (r1 != 0) goto L_0x0109
            org.apache.commons.pool.impl.CursorableLinkedList r1 = r0.queue     // Catch:{ all -> 0x0114 }
            boolean r1 = r1.isEmpty()     // Catch:{ all -> 0x0114 }
            if (r1 == 0) goto L_0x0109
            java.util.Map<K, org.apache.commons.pool.impl.GenericKeyedObjectPool<K, V>$ObjectQueue> r1 = r10._poolMap     // Catch:{ all -> 0x0114 }
            r1.remove(r5)     // Catch:{ all -> 0x0114 }
            org.apache.commons.pool.impl.CursorableLinkedList<K> r1 = r10._poolList     // Catch:{ all -> 0x0114 }
            r1.remove((java.lang.Object) r5)     // Catch:{ all -> 0x0114 }
            goto L_0x0109
        L_0x0103:
            int r1 = r10._totalInternalProcessing     // Catch:{ all -> 0x0114 }
            int r1 = r1 + -1
            r10._totalInternalProcessing = r1     // Catch:{ all -> 0x0114 }
        L_0x0109:
            monitor-exit(r10)     // Catch:{ all -> 0x0114 }
            r10.allocate()
            r8 = r9
            r1 = r10
            r2 = r11
            r3 = r12
            goto L_0x0029
        L_0x0114:
            r0 = move-exception
            monitor-exit(r10)     // Catch:{ all -> 0x0114 }
            throw r0
        L_0x0117:
            r0 = r8
            goto L_0x000e
        L_0x011b:
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.pool.impl.GenericKeyedObjectPool.destroy(java.util.Map, org.apache.commons.pool.KeyedPoolableObjectFactory):void");
    }

    public synchronized int getNumActive() {
        return this._totalActive;
    }

    public synchronized int getNumIdle() {
        return this._totalIdle;
    }

    public synchronized int getNumActive(Object key) {
        GenericKeyedObjectPool<K, V>.ObjectQueue pool;
        pool = this._poolMap.get(key);
        return pool != null ? pool.activeCount : 0;
    }

    public synchronized int getNumIdle(Object key) {
        GenericKeyedObjectPool<K, V>.ObjectQueue pool;
        pool = this._poolMap.get(key);
        return pool != null ? pool.queue.size() : 0;
    }

    public void returnObject(K key, V obj) throws Exception {
        try {
            addObjectToPool(key, obj, true);
        } catch (Exception e) {
            if (this._factory != null) {
                try {
                    this._factory.destroyObject(key, obj);
                } catch (Exception e2) {
                }
                GenericKeyedObjectPool<K, V>.ObjectQueue pool = this._poolMap.get(key);
                if (pool != null) {
                    synchronized (this) {
                        pool.decrementActiveCount();
                        if (pool.queue.isEmpty() && pool.activeCount == 0 && pool.internalProcessingCount == 0) {
                            this._poolMap.remove(key);
                            this._poolList.remove((Object) key);
                        }
                        allocate();
                    }
                }
            }
        }
    }

    /* JADX WARNING: Code restructure failed: missing block: B:42:0x0082, code lost:
        if (r3 == false) goto L_0x0087;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:43:0x0084, code lost:
        allocate();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:44:0x0087, code lost:
        if (r2 == false) goto L_?;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:46:?, code lost:
        r7._factory.destroyObject(r8, r9);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:71:?, code lost:
        return;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private void addObjectToPool(K r8, V r9, boolean r10) throws java.lang.Exception {
        /*
            r7 = this;
            r0 = 1
            boolean r1 = r7._testOnReturn
            if (r1 == 0) goto L_0x000f
            org.apache.commons.pool.KeyedPoolableObjectFactory<K, V> r1 = r7._factory
            boolean r1 = r1.validateObject(r8, r9)
            if (r1 != 0) goto L_0x000f
            r0 = 0
            goto L_0x0014
        L_0x000f:
            org.apache.commons.pool.KeyedPoolableObjectFactory<K, V> r1 = r7._factory
            r1.passivateObject(r8, r9)
        L_0x0014:
            r1 = 1
            if (r0 != 0) goto L_0x0019
            r2 = 1
            goto L_0x001a
        L_0x0019:
            r2 = 0
        L_0x001a:
            r3 = 0
            monitor-enter(r7)
            r4 = 0
            java.util.Map<K, org.apache.commons.pool.impl.GenericKeyedObjectPool<K, V>$ObjectQueue> r5 = r7._poolMap     // Catch:{ all -> 0x00bf }
            java.lang.Object r5 = r5.get(r8)     // Catch:{ all -> 0x00bf }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectQueue r5 = (org.apache.commons.pool.impl.GenericKeyedObjectPool.ObjectQueue) r5     // Catch:{ all -> 0x00bf }
            if (r5 != 0) goto L_0x003c
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectQueue r6 = new org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectQueue     // Catch:{ all -> 0x0038 }
            r6.<init>()     // Catch:{ all -> 0x0038 }
            r4 = r6
            java.util.Map<K, org.apache.commons.pool.impl.GenericKeyedObjectPool<K, V>$ObjectQueue> r5 = r7._poolMap     // Catch:{ all -> 0x00bf }
            r5.put(r8, r4)     // Catch:{ all -> 0x00bf }
            org.apache.commons.pool.impl.CursorableLinkedList<K> r5 = r7._poolList     // Catch:{ all -> 0x00bf }
            r5.add(r8)     // Catch:{ all -> 0x00bf }
            goto L_0x003d
        L_0x0038:
            r1 = move-exception
            r4 = r5
            goto L_0x00c0
        L_0x003c:
            r4 = r5
        L_0x003d:
            boolean r5 = r7.isClosed()     // Catch:{ all -> 0x00bf }
            if (r5 == 0) goto L_0x0045
            r2 = 1
            goto L_0x0081
        L_0x0045:
            int r5 = r7._maxIdle     // Catch:{ all -> 0x00bf }
            if (r5 < 0) goto L_0x0057
            org.apache.commons.pool.impl.CursorableLinkedList r5 = r4.queue     // Catch:{ all -> 0x00bf }
            int r5 = r5.size()     // Catch:{ all -> 0x00bf }
            int r6 = r7._maxIdle     // Catch:{ all -> 0x00bf }
            if (r5 < r6) goto L_0x0057
            r2 = 1
            goto L_0x0081
        L_0x0057:
            if (r0 == 0) goto L_0x0081
            boolean r5 = r7._lifo     // Catch:{ all -> 0x00bf }
            if (r5 == 0) goto L_0x006a
            org.apache.commons.pool.impl.CursorableLinkedList r5 = r4.queue     // Catch:{ all -> 0x00bf }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r6 = new org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair     // Catch:{ all -> 0x00bf }
            r6.<init>(r9)     // Catch:{ all -> 0x00bf }
            r5.addFirst(r6)     // Catch:{ all -> 0x00bf }
            goto L_0x0076
        L_0x006a:
            org.apache.commons.pool.impl.CursorableLinkedList r5 = r4.queue     // Catch:{ all -> 0x00bf }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r6 = new org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair     // Catch:{ all -> 0x00bf }
            r6.<init>(r9)     // Catch:{ all -> 0x00bf }
            r5.addLast(r6)     // Catch:{ all -> 0x00bf }
        L_0x0076:
            int r5 = r7._totalIdle     // Catch:{ all -> 0x00bf }
            int r5 = r5 + r1
            r7._totalIdle = r5     // Catch:{ all -> 0x00bf }
            if (r10 == 0) goto L_0x0080
            r4.decrementActiveCount()     // Catch:{ all -> 0x00bf }
        L_0x0080:
            r3 = 1
        L_0x0081:
            monitor-exit(r7)     // Catch:{ all -> 0x00bf }
            if (r3 == 0) goto L_0x0087
            r7.allocate()
        L_0x0087:
            if (r2 == 0) goto L_0x00be
            org.apache.commons.pool.KeyedPoolableObjectFactory<K, V> r1 = r7._factory     // Catch:{ Exception -> 0x008f }
            r1.destroyObject(r8, r9)     // Catch:{ Exception -> 0x008f }
            goto L_0x0090
        L_0x008f:
            r1 = move-exception
        L_0x0090:
            if (r10 == 0) goto L_0x00be
            monitor-enter(r7)
            r4.decrementActiveCount()     // Catch:{ all -> 0x00bb }
            org.apache.commons.pool.impl.CursorableLinkedList r1 = r4.queue     // Catch:{ all -> 0x00bb }
            boolean r1 = r1.isEmpty()     // Catch:{ all -> 0x00bb }
            if (r1 == 0) goto L_0x00b6
            int r1 = r4.activeCount     // Catch:{ all -> 0x00bb }
            if (r1 != 0) goto L_0x00b6
            int r1 = r4.internalProcessingCount     // Catch:{ all -> 0x00bb }
            if (r1 != 0) goto L_0x00b6
            java.util.Map<K, org.apache.commons.pool.impl.GenericKeyedObjectPool<K, V>$ObjectQueue> r1 = r7._poolMap     // Catch:{ all -> 0x00bb }
            r1.remove(r8)     // Catch:{ all -> 0x00bb }
            org.apache.commons.pool.impl.CursorableLinkedList<K> r1 = r7._poolList     // Catch:{ all -> 0x00bb }
            r1.remove((java.lang.Object) r8)     // Catch:{ all -> 0x00bb }
        L_0x00b6:
            monitor-exit(r7)     // Catch:{ all -> 0x00bb }
            r7.allocate()
            goto L_0x00be
        L_0x00bb:
            r1 = move-exception
            monitor-exit(r7)     // Catch:{ all -> 0x00bb }
            throw r1
        L_0x00be:
            return
        L_0x00bf:
            r1 = move-exception
        L_0x00c0:
            monitor-exit(r7)     // Catch:{ all -> 0x00bf }
            throw r1
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.pool.impl.GenericKeyedObjectPool.addObjectToPool(java.lang.Object, java.lang.Object, boolean):void");
    }

    public void invalidateObject(K key, V obj) throws Exception {
        try {
            this._factory.destroyObject(key, obj);
            synchronized (this) {
                GenericKeyedObjectPool<K, V>.ObjectQueue pool = this._poolMap.get(key);
                if (pool == null) {
                    pool = new ObjectQueue();
                    this._poolMap.put(key, pool);
                    this._poolList.add(key);
                }
                pool.decrementActiveCount();
            }
            allocate();
        } catch (Throwable th) {
            synchronized (this) {
                GenericKeyedObjectPool<K, V>.ObjectQueue pool2 = this._poolMap.get(key);
                if (pool2 == null) {
                    pool2 = new ObjectQueue();
                    this._poolMap.put(key, pool2);
                    this._poolList.add(key);
                }
                pool2.decrementActiveCount();
                allocate();
                throw th;
            }
        }
    }

    public void addObject(K key) throws Exception {
        assertOpen();
        if (this._factory != null) {
            V obj = this._factory.makeObject(key);
            try {
                assertOpen();
                addObjectToPool(key, obj, false);
            } catch (IllegalStateException ex) {
                try {
                    this._factory.destroyObject(key, obj);
                } catch (Exception e) {
                }
                throw ex;
            }
        } else {
            throw new IllegalStateException("Cannot add objects without a factory.");
        }
    }

    public synchronized void preparePool(K key, boolean populateImmediately) {
        if (this._poolMap.get(key) == null) {
            this._poolMap.put(key, new ObjectQueue());
            this._poolList.add(key);
        }
        if (populateImmediately) {
            try {
                ensureMinIdle(key);
            } catch (Exception e) {
            }
        }
    }

    public void close() throws Exception {
        super.close();
        synchronized (this) {
            clear();
            if (this._evictionCursor != null) {
                this._evictionCursor.close();
                this._evictionCursor = null;
            }
            if (this._evictionKeyCursor != null) {
                this._evictionKeyCursor.close();
                this._evictionKeyCursor = null;
            }
            startEvictor(-1);
            while (this._allocationQueue.size() > 0) {
                GenericKeyedObjectPool<K, V>.Latch<K, V> l = this._allocationQueue.removeFirst();
                synchronized (l) {
                    l.notify();
                }
            }
        }
    }

    @Deprecated
    public void setFactory(KeyedPoolableObjectFactory<K, V> factory) throws IllegalStateException {
        Map<K, List<ObjectTimestampPair<V>>> toDestroy = new HashMap<>();
        KeyedPoolableObjectFactory<K, V> oldFactory = this._factory;
        synchronized (this) {
            assertOpen();
            if (getNumActive() <= 0) {
                Iterator<K> it = this._poolMap.keySet().iterator();
                while (it.hasNext()) {
                    K key = it.next();
                    GenericKeyedObjectPool<K, V>.ObjectQueue pool = this._poolMap.get(key);
                    if (pool != null) {
                        List<ObjectTimestampPair<V>> objects = new ArrayList<>();
                        objects.addAll(pool.queue);
                        toDestroy.put(key, objects);
                        it.remove();
                        this._poolList.remove((Object) key);
                        this._totalIdle -= pool.queue.size();
                        this._totalInternalProcessing += pool.queue.size();
                        pool.queue.clear();
                    }
                }
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
            r15 = this;
            r0 = 0
            monitor-enter(r15)
            r1 = 0
            r2 = 0
            boolean r4 = r15._testWhileIdle     // Catch:{ all -> 0x019f }
            long r5 = r15._minEvictableIdleTimeMillis     // Catch:{ all -> 0x019d }
            org.apache.commons.pool.impl.CursorableLinkedList<K>$Cursor r7 = r15._evictionKeyCursor     // Catch:{ all -> 0x019a }
            if (r7 == 0) goto L_0x001c
            org.apache.commons.pool.impl.CursorableLinkedList<K>$Cursor r7 = r15._evictionKeyCursor     // Catch:{ all -> 0x019a }
            org.apache.commons.pool.impl.CursorableLinkedList$Listable r7 = r7._lastReturned     // Catch:{ all -> 0x019a }
            if (r7 == 0) goto L_0x001c
            org.apache.commons.pool.impl.CursorableLinkedList<K>$Cursor r7 = r15._evictionKeyCursor     // Catch:{ all -> 0x019a }
            org.apache.commons.pool.impl.CursorableLinkedList$Listable r7 = r7._lastReturned     // Catch:{ all -> 0x019a }
            java.lang.Object r7 = r7.value()     // Catch:{ all -> 0x019a }
            r0 = r7
        L_0x001c:
            monitor-exit(r15)     // Catch:{ all -> 0x019a }
            r7 = 0
            int r8 = r15.getNumTests()
            r9 = 0
        L_0x0023:
            if (r7 >= r8) goto L_0x0196
            monitor-enter(r15)
            java.util.Map<K, org.apache.commons.pool.impl.GenericKeyedObjectPool<K, V>$ObjectQueue> r10 = r15._poolMap     // Catch:{ all -> 0x0193 }
            if (r10 == 0) goto L_0x018e
            java.util.Map<K, org.apache.commons.pool.impl.GenericKeyedObjectPool<K, V>$ObjectQueue> r10 = r15._poolMap     // Catch:{ all -> 0x0193 }
            int r10 = r10.size()     // Catch:{ all -> 0x0193 }
            if (r10 != 0) goto L_0x0034
            goto L_0x018e
        L_0x0034:
            org.apache.commons.pool.impl.CursorableLinkedList<K>$Cursor r10 = r15._evictionKeyCursor     // Catch:{ all -> 0x0193 }
            if (r10 != 0) goto L_0x003c
            r15.resetEvictionKeyCursor()     // Catch:{ all -> 0x0193 }
            r0 = 0
        L_0x003c:
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<V>>$Cursor r10 = r15._evictionCursor     // Catch:{ all -> 0x0193 }
            if (r10 != 0) goto L_0x006c
            org.apache.commons.pool.impl.CursorableLinkedList<K>$Cursor r10 = r15._evictionKeyCursor     // Catch:{ all -> 0x0193 }
            boolean r10 = r10.hasNext()     // Catch:{ all -> 0x0193 }
            if (r10 == 0) goto L_0x0053
            org.apache.commons.pool.impl.CursorableLinkedList<K>$Cursor r10 = r15._evictionKeyCursor     // Catch:{ all -> 0x0193 }
            java.lang.Object r10 = r10.next()     // Catch:{ all -> 0x0193 }
            r0 = r10
            r15.resetEvictionObjectCursor(r0)     // Catch:{ all -> 0x0193 }
            goto L_0x006c
        L_0x0053:
            r15.resetEvictionKeyCursor()     // Catch:{ all -> 0x0193 }
            org.apache.commons.pool.impl.CursorableLinkedList<K>$Cursor r10 = r15._evictionKeyCursor     // Catch:{ all -> 0x0193 }
            if (r10 == 0) goto L_0x006c
            org.apache.commons.pool.impl.CursorableLinkedList<K>$Cursor r10 = r15._evictionKeyCursor     // Catch:{ all -> 0x0193 }
            boolean r10 = r10.hasNext()     // Catch:{ all -> 0x0193 }
            if (r10 == 0) goto L_0x006c
            org.apache.commons.pool.impl.CursorableLinkedList<K>$Cursor r10 = r15._evictionKeyCursor     // Catch:{ all -> 0x0193 }
            java.lang.Object r10 = r10.next()     // Catch:{ all -> 0x0193 }
            r0 = r10
            r15.resetEvictionObjectCursor(r0)     // Catch:{ all -> 0x0193 }
        L_0x006c:
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<V>>$Cursor r10 = r15._evictionCursor     // Catch:{ all -> 0x0193 }
            if (r10 != 0) goto L_0x0073
            monitor-exit(r15)     // Catch:{ all -> 0x0193 }
            goto L_0x018f
        L_0x0073:
            boolean r10 = r15._lifo     // Catch:{ all -> 0x0193 }
            if (r10 == 0) goto L_0x007f
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<V>>$Cursor r10 = r15._evictionCursor     // Catch:{ all -> 0x0193 }
            boolean r10 = r10.hasPrevious()     // Catch:{ all -> 0x0193 }
            if (r10 == 0) goto L_0x008b
        L_0x007f:
            boolean r10 = r15._lifo     // Catch:{ all -> 0x0193 }
            if (r10 != 0) goto L_0x00bb
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<V>>$Cursor r10 = r15._evictionCursor     // Catch:{ all -> 0x0193 }
            boolean r10 = r10.hasNext()     // Catch:{ all -> 0x0193 }
            if (r10 != 0) goto L_0x00bb
        L_0x008b:
            org.apache.commons.pool.impl.CursorableLinkedList<K>$Cursor r10 = r15._evictionKeyCursor     // Catch:{ all -> 0x0193 }
            if (r10 == 0) goto L_0x00bb
            org.apache.commons.pool.impl.CursorableLinkedList<K>$Cursor r10 = r15._evictionKeyCursor     // Catch:{ all -> 0x0193 }
            boolean r10 = r10.hasNext()     // Catch:{ all -> 0x0193 }
            if (r10 == 0) goto L_0x00a2
            org.apache.commons.pool.impl.CursorableLinkedList<K>$Cursor r10 = r15._evictionKeyCursor     // Catch:{ all -> 0x0193 }
            java.lang.Object r10 = r10.next()     // Catch:{ all -> 0x0193 }
            r0 = r10
            r15.resetEvictionObjectCursor(r0)     // Catch:{ all -> 0x0193 }
            goto L_0x00bb
        L_0x00a2:
            r15.resetEvictionKeyCursor()     // Catch:{ all -> 0x0193 }
            org.apache.commons.pool.impl.CursorableLinkedList<K>$Cursor r10 = r15._evictionKeyCursor     // Catch:{ all -> 0x0193 }
            if (r10 == 0) goto L_0x00bb
            org.apache.commons.pool.impl.CursorableLinkedList<K>$Cursor r10 = r15._evictionKeyCursor     // Catch:{ all -> 0x0193 }
            boolean r10 = r10.hasNext()     // Catch:{ all -> 0x0193 }
            if (r10 == 0) goto L_0x00bb
            org.apache.commons.pool.impl.CursorableLinkedList<K>$Cursor r10 = r15._evictionKeyCursor     // Catch:{ all -> 0x0193 }
            java.lang.Object r10 = r10.next()     // Catch:{ all -> 0x0193 }
            r0 = r10
            r15.resetEvictionObjectCursor(r0)     // Catch:{ all -> 0x0193 }
        L_0x00bb:
            boolean r10 = r15._lifo     // Catch:{ all -> 0x0193 }
            if (r10 == 0) goto L_0x00c7
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<V>>$Cursor r10 = r15._evictionCursor     // Catch:{ all -> 0x0193 }
            boolean r10 = r10.hasPrevious()     // Catch:{ all -> 0x0193 }
            if (r10 == 0) goto L_0x00d3
        L_0x00c7:
            boolean r10 = r15._lifo     // Catch:{ all -> 0x0193 }
            if (r10 != 0) goto L_0x00d6
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<V>>$Cursor r10 = r15._evictionCursor     // Catch:{ all -> 0x0193 }
            boolean r10 = r10.hasNext()     // Catch:{ all -> 0x0193 }
            if (r10 != 0) goto L_0x00d6
        L_0x00d3:
            monitor-exit(r15)     // Catch:{ all -> 0x0193 }
            goto L_0x018f
        L_0x00d6:
            boolean r10 = r15._lifo     // Catch:{ all -> 0x0193 }
            if (r10 == 0) goto L_0x00e3
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<V>>$Cursor r10 = r15._evictionCursor     // Catch:{ all -> 0x0193 }
            java.lang.Object r10 = r10.previous()     // Catch:{ all -> 0x0193 }
        L_0x00e0:
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair r10 = (org.apache.commons.pool.impl.GenericKeyedObjectPool.ObjectTimestampPair) r10     // Catch:{ all -> 0x0193 }
            goto L_0x00ea
        L_0x00e3:
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<V>>$Cursor r10 = r15._evictionCursor     // Catch:{ all -> 0x0193 }
            java.lang.Object r10 = r10.next()     // Catch:{ all -> 0x0193 }
            goto L_0x00e0
        L_0x00ea:
            r9 = r10
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<V>>$Cursor r10 = r15._evictionCursor     // Catch:{ all -> 0x0193 }
            r10.remove()     // Catch:{ all -> 0x0193 }
            java.util.Map<K, org.apache.commons.pool.impl.GenericKeyedObjectPool<K, V>$ObjectQueue> r10 = r15._poolMap     // Catch:{ all -> 0x0193 }
            java.lang.Object r10 = r10.get(r0)     // Catch:{ all -> 0x0193 }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectQueue r10 = (org.apache.commons.pool.impl.GenericKeyedObjectPool.ObjectQueue) r10     // Catch:{ all -> 0x0193 }
            r10.incrementInternalProcessingCount()     // Catch:{ all -> 0x0193 }
            int r11 = r15._totalIdle     // Catch:{ all -> 0x0193 }
            int r11 = r11 + -1
            r15._totalIdle = r11     // Catch:{ all -> 0x0193 }
            monitor-exit(r15)     // Catch:{ all -> 0x0193 }
            r10 = 0
            int r11 = (r5 > r2 ? 1 : (r5 == r2 ? 0 : -1))
            if (r11 <= 0) goto L_0x0113
            long r11 = java.lang.System.currentTimeMillis()
            long r13 = r9.tstamp
            long r11 = r11 - r13
            int r13 = (r11 > r5 ? 1 : (r11 == r5 ? 0 : -1))
            if (r13 <= 0) goto L_0x0113
            r10 = 1
        L_0x0113:
            if (r4 == 0) goto L_0x013b
            if (r10 != 0) goto L_0x013b
            r11 = r1
            org.apache.commons.pool.KeyedPoolableObjectFactory<K, V> r12 = r15._factory     // Catch:{ Exception -> 0x0121 }
            T r13 = r9.value     // Catch:{ Exception -> 0x0121 }
            r12.activateObject(r0, r13)     // Catch:{ Exception -> 0x0121 }
            r11 = 1
            goto L_0x0123
        L_0x0121:
            r12 = move-exception
            r10 = 1
        L_0x0123:
            if (r11 == 0) goto L_0x013b
            org.apache.commons.pool.KeyedPoolableObjectFactory<K, V> r12 = r15._factory
            T r13 = r9.value
            boolean r12 = r12.validateObject(r0, r13)
            if (r12 != 0) goto L_0x0131
            r10 = 1
            goto L_0x013b
        L_0x0131:
            org.apache.commons.pool.KeyedPoolableObjectFactory<K, V> r12 = r15._factory     // Catch:{ Exception -> 0x0139 }
            T r13 = r9.value     // Catch:{ Exception -> 0x0139 }
            r12.passivateObject(r0, r13)     // Catch:{ Exception -> 0x0139 }
            goto L_0x013b
        L_0x0139:
            r12 = move-exception
            r10 = 1
        L_0x013b:
            if (r10 == 0) goto L_0x0146
            org.apache.commons.pool.KeyedPoolableObjectFactory<K, V> r11 = r15._factory     // Catch:{ Exception -> 0x0145 }
            T r12 = r9.value     // Catch:{ Exception -> 0x0145 }
            r11.destroyObject(r0, r12)     // Catch:{ Exception -> 0x0145 }
            goto L_0x0146
        L_0x0145:
            r11 = move-exception
        L_0x0146:
            monitor-enter(r15)
            java.util.Map<K, org.apache.commons.pool.impl.GenericKeyedObjectPool<K, V>$ObjectQueue> r11 = r15._poolMap     // Catch:{ all -> 0x018b }
            java.lang.Object r11 = r11.get(r0)     // Catch:{ all -> 0x018b }
            org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectQueue r11 = (org.apache.commons.pool.impl.GenericKeyedObjectPool.ObjectQueue) r11     // Catch:{ all -> 0x018b }
            r11.decrementInternalProcessingCount()     // Catch:{ all -> 0x018b }
            if (r10 == 0) goto L_0x0175
            org.apache.commons.pool.impl.CursorableLinkedList r12 = r11.queue     // Catch:{ all -> 0x018b }
            boolean r12 = r12.isEmpty()     // Catch:{ all -> 0x018b }
            if (r12 == 0) goto L_0x0189
            int r12 = r11.activeCount     // Catch:{ all -> 0x018b }
            if (r12 != 0) goto L_0x0189
            int r12 = r11.internalProcessingCount     // Catch:{ all -> 0x018b }
            if (r12 != 0) goto L_0x0189
            java.util.Map<K, org.apache.commons.pool.impl.GenericKeyedObjectPool<K, V>$ObjectQueue> r12 = r15._poolMap     // Catch:{ all -> 0x018b }
            r12.remove(r0)     // Catch:{ all -> 0x018b }
            org.apache.commons.pool.impl.CursorableLinkedList<K> r12 = r15._poolList     // Catch:{ all -> 0x018b }
            r12.remove((java.lang.Object) r0)     // Catch:{ all -> 0x018b }
            goto L_0x0189
        L_0x0175:
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<V>>$Cursor r12 = r15._evictionCursor     // Catch:{ all -> 0x018b }
            r12.add(r9)     // Catch:{ all -> 0x018b }
            int r12 = r15._totalIdle     // Catch:{ all -> 0x018b }
            int r12 = r12 + 1
            r15._totalIdle = r12     // Catch:{ all -> 0x018b }
            boolean r12 = r15._lifo     // Catch:{ all -> 0x018b }
            if (r12 == 0) goto L_0x0189
            org.apache.commons.pool.impl.CursorableLinkedList<org.apache.commons.pool.impl.GenericKeyedObjectPool$ObjectTimestampPair<V>>$Cursor r12 = r15._evictionCursor     // Catch:{ all -> 0x018b }
            r12.previous()     // Catch:{ all -> 0x018b }
        L_0x0189:
            monitor-exit(r15)     // Catch:{ all -> 0x018b }
            goto L_0x018f
        L_0x018b:
            r1 = move-exception
            monitor-exit(r15)     // Catch:{ all -> 0x018b }
            throw r1
        L_0x018e:
            monitor-exit(r15)     // Catch:{ all -> 0x0193 }
        L_0x018f:
            int r7 = r7 + 1
            goto L_0x0023
        L_0x0193:
            r1 = move-exception
            monitor-exit(r15)     // Catch:{ all -> 0x0193 }
            throw r1
        L_0x0196:
            r15.allocate()
            return
        L_0x019a:
            r1 = move-exception
            r2 = r5
            goto L_0x01a2
        L_0x019d:
            r1 = move-exception
            goto L_0x01a2
        L_0x019f:
            r4 = move-exception
            r1 = r4
            r4 = 0
        L_0x01a2:
            monitor-exit(r15)     // Catch:{ all -> 0x019d }
            throw r1
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.pool.impl.GenericKeyedObjectPool.evict():void");
    }

    private void resetEvictionKeyCursor() {
        if (this._evictionKeyCursor != null) {
            this._evictionKeyCursor.close();
        }
        this._evictionKeyCursor = this._poolList.cursor();
        if (this._evictionCursor != null) {
            this._evictionCursor.close();
            this._evictionCursor = null;
        }
    }

    private void resetEvictionObjectCursor(Object key) {
        GenericKeyedObjectPool<K, V>.ObjectQueue pool;
        if (this._evictionCursor != null) {
            this._evictionCursor.close();
        }
        if (this._poolMap != null && (pool = this._poolMap.get(key)) != null) {
            CursorableLinkedList<ObjectTimestampPair<V>> queue = pool.queue;
            this._evictionCursor = queue.cursor(this._lifo ? queue.size() : 0);
        }
    }

    /* access modifiers changed from: private */
    public void ensureMinIdle() throws Exception {
        Object[] keysCopy;
        if (this._minIdle > 0) {
            synchronized (this) {
                keysCopy = this._poolMap.keySet().toArray();
            }
            for (Object ensureMinIdle : keysCopy) {
                ensureMinIdle(ensureMinIdle);
            }
        }
    }

    private void ensureMinIdle(K key) throws Exception {
        GenericKeyedObjectPool<K, V>.ObjectQueue pool;
        synchronized (this) {
            pool = this._poolMap.get(key);
        }
        if (pool != null) {
            int i = 0;
            int objectDeficit = calculateDeficit(pool, false);
            K key2 = key;
            GenericKeyedObjectPool genericKeyedObjectPool = this;
            while (i < objectDeficit && genericKeyedObjectPool.calculateDeficit(pool, true) > 0) {
                try {
                    genericKeyedObjectPool.addObject(key2);
                    GenericKeyedObjectPool genericKeyedObjectPool2 = genericKeyedObjectPool;
                    K key3 = key2;
                    GenericKeyedObjectPool<K, V>.ObjectQueue pool2 = pool;
                    int objectDeficit2 = objectDeficit;
                    synchronized (genericKeyedObjectPool2) {
                        pool2.decrementInternalProcessingCount();
                    }
                    genericKeyedObjectPool2.allocate();
                    i++;
                    genericKeyedObjectPool = genericKeyedObjectPool2;
                    key2 = key3;
                    pool = pool2;
                    objectDeficit = objectDeficit2;
                } catch (Throwable th) {
                    GenericKeyedObjectPool genericKeyedObjectPool3 = genericKeyedObjectPool;
                    K k = key2;
                    GenericKeyedObjectPool<K, V>.ObjectQueue pool3 = pool;
                    int i2 = objectDeficit;
                    synchronized (genericKeyedObjectPool3) {
                        pool3.decrementInternalProcessingCount();
                        genericKeyedObjectPool3.allocate();
                        throw th;
                    }
                }
            }
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
        for (K key : this._poolMap.keySet()) {
            buf.append("\t");
            buf.append(key);
            buf.append(" ");
            buf.append(this._poolMap.get(key));
            buf.append("\n");
        }
        return buf.toString();
    }

    private synchronized int getNumTests() {
        if (this._numTestsPerEvictionRun >= 0) {
            return Math.min(this._numTestsPerEvictionRun, this._totalIdle);
        }
        double d = (double) this._totalIdle;
        double abs = Math.abs((double) this._numTestsPerEvictionRun);
        Double.isNaN(d);
        return (int) Math.ceil(d / abs);
    }

    private synchronized int calculateDeficit(GenericKeyedObjectPool<K, V>.ObjectQueue pool, boolean incrementInternal) {
        int objectDefecit;
        objectDefecit = getMinIdle() - pool.queue.size();
        if (getMaxActive() > 0) {
            objectDefecit = Math.min(objectDefecit, Math.max(0, ((getMaxActive() - pool.activeCount) - pool.queue.size()) - pool.internalProcessingCount));
        }
        if (getMaxTotal() > 0) {
            objectDefecit = Math.min(objectDefecit, Math.max(0, ((getMaxTotal() - getNumActive()) - getNumIdle()) - this._totalInternalProcessing));
        }
        if (incrementInternal && objectDefecit > 0) {
            pool.incrementInternalProcessingCount();
        }
        return objectDefecit;
    }

    private class ObjectQueue {
        /* access modifiers changed from: private */
        public int activeCount;
        /* access modifiers changed from: private */
        public int internalProcessingCount;
        /* access modifiers changed from: private */
        public final CursorableLinkedList<ObjectTimestampPair<V>> queue;

        private ObjectQueue() {
            this.activeCount = 0;
            this.queue = new CursorableLinkedList<>();
            this.internalProcessingCount = 0;
        }

        /* access modifiers changed from: package-private */
        public void incrementActiveCount() {
            synchronized (GenericKeyedObjectPool.this) {
                GenericKeyedObjectPool.access$1408(GenericKeyedObjectPool.this);
            }
            this.activeCount++;
        }

        /* access modifiers changed from: package-private */
        public void decrementActiveCount() {
            synchronized (GenericKeyedObjectPool.this) {
                GenericKeyedObjectPool.access$1410(GenericKeyedObjectPool.this);
            }
            if (this.activeCount > 0) {
                this.activeCount--;
            }
        }

        /* access modifiers changed from: package-private */
        public void incrementInternalProcessingCount() {
            synchronized (GenericKeyedObjectPool.this) {
                GenericKeyedObjectPool.access$1508(GenericKeyedObjectPool.this);
            }
            this.internalProcessingCount++;
        }

        /* access modifiers changed from: package-private */
        public void decrementInternalProcessingCount() {
            synchronized (GenericKeyedObjectPool.this) {
                GenericKeyedObjectPool.access$1510(GenericKeyedObjectPool.this);
            }
            this.internalProcessingCount--;
        }
    }

    static class ObjectTimestampPair<T> implements Comparable<T> {
        @Deprecated
        long tstamp;
        @Deprecated
        T value;

        ObjectTimestampPair(T val) {
            this(val, System.currentTimeMillis());
        }

        ObjectTimestampPair(T val, long time) {
            this.value = val;
            this.tstamp = time;
        }

        public String toString() {
            return this.value + ";" + this.tstamp;
        }

        public int compareTo(Object obj) {
            return compareTo((ObjectTimestampPair) obj);
        }

        public int compareTo(ObjectTimestampPair<T> other) {
            long tstampdiff = this.tstamp - other.tstamp;
            if (tstampdiff == 0) {
                return System.identityHashCode(this) - System.identityHashCode(other);
            }
            return (int) Math.min(Math.max(tstampdiff, -2147483648L), TTL.MAX_VALUE);
        }

        public T getValue() {
            return this.value;
        }

        public long getTstamp() {
            return this.tstamp;
        }
    }

    private class Evictor extends TimerTask {
        private Evictor() {
        }

        public void run() {
            try {
                GenericKeyedObjectPool.this.evict();
            } catch (Exception e) {
            } catch (OutOfMemoryError oome) {
                oome.printStackTrace(System.err);
            }
            try {
                GenericKeyedObjectPool.this.ensureMinIdle();
            } catch (Exception e2) {
            }
        }
    }

    private final class Latch<LK, LV> {
        private final LK _key;
        private boolean _mayCreate;
        private ObjectTimestampPair<LV> _pair;
        private GenericKeyedObjectPool<K, V>.ObjectQueue _pool;

        private Latch(LK key) {
            this._mayCreate = false;
            this._key = key;
        }

        /* access modifiers changed from: private */
        public synchronized LK getkey() {
            return this._key;
        }

        /* access modifiers changed from: private */
        public synchronized GenericKeyedObjectPool<K, V>.ObjectQueue getPool() {
            return this._pool;
        }

        /* access modifiers changed from: private */
        public synchronized void setPool(GenericKeyedObjectPool<K, V>.ObjectQueue pool) {
            this._pool = pool;
        }

        /* access modifiers changed from: private */
        public synchronized ObjectTimestampPair<LV> getPair() {
            return this._pair;
        }

        /* access modifiers changed from: private */
        public synchronized void setPair(ObjectTimestampPair<LV> pair) {
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
