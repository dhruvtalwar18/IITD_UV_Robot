package org.apache.commons.pool.impl;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Stack;
import org.apache.commons.pool.BaseKeyedObjectPool;
import org.apache.commons.pool.KeyedObjectPool;
import org.apache.commons.pool.KeyedPoolableObjectFactory;
import org.apache.commons.pool.PoolUtils;

public class StackKeyedObjectPool<K, V> extends BaseKeyedObjectPool<K, V> implements KeyedObjectPool<K, V> {
    protected static final int DEFAULT_INIT_SLEEPING_CAPACITY = 4;
    protected static final int DEFAULT_MAX_SLEEPING = 8;
    @Deprecated
    protected HashMap<K, Integer> _activeCount;
    @Deprecated
    protected KeyedPoolableObjectFactory<K, V> _factory;
    @Deprecated
    protected int _initSleepingCapacity;
    @Deprecated
    protected int _maxSleeping;
    @Deprecated
    protected HashMap<K, Stack<V>> _pools;
    @Deprecated
    protected int _totActive;
    @Deprecated
    protected int _totIdle;

    public StackKeyedObjectPool() {
        this((KeyedPoolableObjectFactory) null, 8, 4);
    }

    public StackKeyedObjectPool(int max) {
        this((KeyedPoolableObjectFactory) null, max, 4);
    }

    public StackKeyedObjectPool(int max, int init) {
        this((KeyedPoolableObjectFactory) null, max, init);
    }

    public StackKeyedObjectPool(KeyedPoolableObjectFactory<K, V> factory) {
        this(factory, 8);
    }

    public StackKeyedObjectPool(KeyedPoolableObjectFactory<K, V> factory, int max) {
        this(factory, max, 4);
    }

    public StackKeyedObjectPool(KeyedPoolableObjectFactory<K, V> factory, int max, int init) {
        this._pools = null;
        this._factory = null;
        int i = 8;
        this._maxSleeping = 8;
        int i2 = 4;
        this._initSleepingCapacity = 4;
        this._totActive = 0;
        this._totIdle = 0;
        this._activeCount = null;
        this._factory = factory;
        this._maxSleeping = max >= 0 ? max : i;
        this._initSleepingCapacity = init >= 1 ? init : i2;
        this._pools = new HashMap<>();
        this._activeCount = new HashMap<>();
    }

    public synchronized V borrowObject(K key) throws Exception {
        boolean newlyMade;
        V obj;
        assertOpen();
        Stack<V> stack = this._pools.get(key);
        if (stack == null) {
            stack = new Stack<>();
            stack.ensureCapacity(this._initSleepingCapacity > this._maxSleeping ? this._maxSleeping : this._initSleepingCapacity);
            this._pools.put(key, stack);
        }
        do {
            newlyMade = false;
            if (!stack.empty()) {
                obj = stack.pop();
                this._totIdle--;
            } else if (this._factory != null) {
                obj = this._factory.makeObject(key);
                newlyMade = true;
            } else {
                throw new NoSuchElementException("pools without a factory cannot create new objects as needed.");
            }
            if (!(this._factory == null || obj == null)) {
                try {
                    this._factory.activateObject(key, obj);
                    if (this._factory.validateObject(key, obj)) {
                        continue;
                    } else {
                        throw new Exception("ValidateObject failed");
                    }
                } catch (Throwable t2) {
                    PoolUtils.checkRethrow(t2);
                }
            }
        } while (obj != null);
        incrementActiveCount(key);
        return obj;
        obj = null;
        if (newlyMade) {
            throw new NoSuchElementException("Could not create a validated object, cause: " + t.getMessage());
        } else if (obj != null) {
            incrementActiveCount(key);
            return obj;
        }
    }

    public synchronized void returnObject(K key, V obj) throws Exception {
        V staleObj;
        decrementActiveCount(key);
        if (this._factory != null) {
            if (this._factory.validateObject(key, obj)) {
                try {
                    this._factory.passivateObject(key, obj);
                } catch (Exception e) {
                    this._factory.destroyObject(key, obj);
                    return;
                }
            } else {
                return;
            }
        }
        if (!isClosed()) {
            Stack<V> stack = this._pools.get(key);
            if (stack == null) {
                stack = new Stack<>();
                stack.ensureCapacity(this._initSleepingCapacity > this._maxSleeping ? this._maxSleeping : this._initSleepingCapacity);
                this._pools.put(key, stack);
            }
            int stackSize = stack.size();
            if (stackSize >= this._maxSleeping) {
                if (stackSize > 0) {
                    staleObj = stack.remove(0);
                    this._totIdle--;
                } else {
                    staleObj = obj;
                }
                if (this._factory != null) {
                    try {
                        this._factory.destroyObject(key, staleObj);
                    } catch (Exception e2) {
                    }
                }
            }
            stack.push(obj);
            this._totIdle++;
        } else if (this._factory != null) {
            try {
                this._factory.destroyObject(key, obj);
            } catch (Exception e3) {
            }
        }
    }

    public synchronized void invalidateObject(K key, V obj) throws Exception {
        decrementActiveCount(key);
        if (this._factory != null) {
            this._factory.destroyObject(key, obj);
        }
        notifyAll();
    }

    /* JADX WARNING: Code restructure failed: missing block: B:31:0x006d, code lost:
        return;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public synchronized void addObject(K r6) throws java.lang.Exception {
        /*
            r5 = this;
            monitor-enter(r5)
            r5.assertOpen()     // Catch:{ all -> 0x0080 }
            org.apache.commons.pool.KeyedPoolableObjectFactory<K, V> r0 = r5._factory     // Catch:{ all -> 0x0080 }
            if (r0 == 0) goto L_0x0078
            org.apache.commons.pool.KeyedPoolableObjectFactory<K, V> r0 = r5._factory     // Catch:{ all -> 0x0080 }
            java.lang.Object r0 = r0.makeObject(r6)     // Catch:{ all -> 0x0080 }
            org.apache.commons.pool.KeyedPoolableObjectFactory<K, V> r1 = r5._factory     // Catch:{ Exception -> 0x006e }
            boolean r1 = r1.validateObject(r6, r0)     // Catch:{ Exception -> 0x006e }
            if (r1 != 0) goto L_0x0018
            monitor-exit(r5)
            return
        L_0x0018:
            org.apache.commons.pool.KeyedPoolableObjectFactory<K, V> r1 = r5._factory     // Catch:{ all -> 0x0080 }
            r1.passivateObject(r6, r0)     // Catch:{ all -> 0x0080 }
            java.util.HashMap<K, java.util.Stack<V>> r1 = r5._pools     // Catch:{ all -> 0x0080 }
            java.lang.Object r1 = r1.get(r6)     // Catch:{ all -> 0x0080 }
            java.util.Stack r1 = (java.util.Stack) r1     // Catch:{ all -> 0x0080 }
            if (r1 != 0) goto L_0x0041
            java.util.Stack r2 = new java.util.Stack     // Catch:{ all -> 0x0080 }
            r2.<init>()     // Catch:{ all -> 0x0080 }
            r1 = r2
            int r2 = r5._initSleepingCapacity     // Catch:{ all -> 0x0080 }
            int r3 = r5._maxSleeping     // Catch:{ all -> 0x0080 }
            if (r2 <= r3) goto L_0x0037
            int r2 = r5._maxSleeping     // Catch:{ all -> 0x0080 }
            goto L_0x0039
        L_0x0037:
            int r2 = r5._initSleepingCapacity     // Catch:{ all -> 0x0080 }
        L_0x0039:
            r1.ensureCapacity(r2)     // Catch:{ all -> 0x0080 }
            java.util.HashMap<K, java.util.Stack<V>> r2 = r5._pools     // Catch:{ all -> 0x0080 }
            r2.put(r6, r1)     // Catch:{ all -> 0x0080 }
        L_0x0041:
            int r2 = r1.size()     // Catch:{ all -> 0x0080 }
            int r3 = r5._maxSleeping     // Catch:{ all -> 0x0080 }
            if (r2 < r3) goto L_0x0063
            if (r2 <= 0) goto L_0x0057
            r3 = 0
            java.lang.Object r3 = r1.remove(r3)     // Catch:{ all -> 0x0080 }
            int r4 = r5._totIdle     // Catch:{ all -> 0x0080 }
            int r4 = r4 + -1
            r5._totIdle = r4     // Catch:{ all -> 0x0080 }
            goto L_0x0058
        L_0x0057:
            r3 = r0
        L_0x0058:
            org.apache.commons.pool.KeyedPoolableObjectFactory<K, V> r4 = r5._factory     // Catch:{ Exception -> 0x005e }
            r4.destroyObject(r6, r3)     // Catch:{ Exception -> 0x005e }
            goto L_0x0061
        L_0x005e:
            r4 = move-exception
            if (r0 == r3) goto L_0x0062
        L_0x0061:
            goto L_0x006c
        L_0x0062:
            throw r4     // Catch:{ all -> 0x0080 }
        L_0x0063:
            r1.push(r0)     // Catch:{ all -> 0x0080 }
            int r3 = r5._totIdle     // Catch:{ all -> 0x0080 }
            int r3 = r3 + 1
            r5._totIdle = r3     // Catch:{ all -> 0x0080 }
        L_0x006c:
            monitor-exit(r5)
            return
        L_0x006e:
            r1 = move-exception
            org.apache.commons.pool.KeyedPoolableObjectFactory<K, V> r2 = r5._factory     // Catch:{ Exception -> 0x0075 }
            r2.destroyObject(r6, r0)     // Catch:{ Exception -> 0x0075 }
            goto L_0x0076
        L_0x0075:
            r2 = move-exception
        L_0x0076:
            monitor-exit(r5)
            return
        L_0x0078:
            java.lang.IllegalStateException r0 = new java.lang.IllegalStateException     // Catch:{ all -> 0x0080 }
            java.lang.String r1 = "Cannot add objects without a factory."
            r0.<init>(r1)     // Catch:{ all -> 0x0080 }
            throw r0     // Catch:{ all -> 0x0080 }
        L_0x0080:
            r6 = move-exception
            monitor-exit(r5)
            throw r6
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.pool.impl.StackKeyedObjectPool.addObject(java.lang.Object):void");
    }

    public synchronized int getNumIdle() {
        return this._totIdle;
    }

    public synchronized int getNumActive() {
        return this._totActive;
    }

    public synchronized int getNumActive(K key) {
        return getActiveCount(key);
    }

    /* JADX WARNING: Code restructure failed: missing block: B:11:0x0015, code lost:
        return 0;
     */
    /* JADX WARNING: Exception block dominator not found, dom blocks: [] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public synchronized int getNumIdle(K r3) {
        /*
            r2 = this;
            monitor-enter(r2)
            java.util.HashMap<K, java.util.Stack<V>> r0 = r2._pools     // Catch:{ Exception -> 0x0012, all -> 0x000f }
            java.lang.Object r0 = r0.get(r3)     // Catch:{ Exception -> 0x0012, all -> 0x000f }
            java.util.Stack r0 = (java.util.Stack) r0     // Catch:{ Exception -> 0x0012, all -> 0x000f }
            int r0 = r0.size()     // Catch:{ Exception -> 0x0012, all -> 0x000f }
            monitor-exit(r2)
            return r0
        L_0x000f:
            r3 = move-exception
            monitor-exit(r2)
            throw r3
        L_0x0012:
            r0 = move-exception
            r1 = 0
            monitor-exit(r2)
            return r1
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.pool.impl.StackKeyedObjectPool.getNumIdle(java.lang.Object):int");
    }

    public synchronized void clear() {
        for (K key : this._pools.keySet()) {
            destroyStack(key, this._pools.get(key));
        }
        this._totIdle = 0;
        this._pools.clear();
        this._activeCount.clear();
    }

    public synchronized void clear(K key) {
        destroyStack(key, this._pools.remove(key));
    }

    private synchronized void destroyStack(K key, Stack<V> stack) {
        if (stack != null) {
            if (this._factory != null) {
                Iterator<V> it = stack.iterator();
                while (it.hasNext()) {
                    try {
                        this._factory.destroyObject(key, it.next());
                    } catch (Exception e) {
                    }
                }
            }
            this._totIdle -= stack.size();
            this._activeCount.remove(key);
            stack.clear();
        }
    }

    public synchronized String toString() {
        StringBuffer buf;
        buf = new StringBuffer();
        buf.append(getClass().getName());
        buf.append(" contains ");
        buf.append(this._pools.size());
        buf.append(" distinct pools: ");
        for (K key : this._pools.keySet()) {
            buf.append(" |");
            buf.append(key);
            buf.append("|=");
            buf.append(this._pools.get(key).size());
        }
        return buf.toString();
    }

    public void close() throws Exception {
        super.close();
        clear();
    }

    @Deprecated
    public synchronized void setFactory(KeyedPoolableObjectFactory<K, V> factory) throws IllegalStateException {
        if (getNumActive() <= 0) {
            clear();
            this._factory = factory;
        } else {
            throw new IllegalStateException("Objects are already active");
        }
    }

    public synchronized KeyedPoolableObjectFactory<K, V> getFactory() {
        return this._factory;
    }

    private int getActiveCount(K key) {
        try {
            return this._activeCount.get(key).intValue();
        } catch (NoSuchElementException e) {
            return 0;
        } catch (NullPointerException e2) {
            return 0;
        }
    }

    private void incrementActiveCount(K key) {
        this._totActive++;
        Integer old = this._activeCount.get(key);
        if (old == null) {
            this._activeCount.put(key, new Integer(1));
        } else {
            this._activeCount.put(key, new Integer(old.intValue() + 1));
        }
    }

    private void decrementActiveCount(K key) {
        this._totActive--;
        Integer active = this._activeCount.get(key);
        if (active != null) {
            if (active.intValue() <= 1) {
                this._activeCount.remove(key);
            } else {
                this._activeCount.put(key, new Integer(active.intValue() - 1));
            }
        }
    }

    public Map<K, Stack<V>> getPools() {
        return this._pools;
    }

    public int getMaxSleeping() {
        return this._maxSleeping;
    }

    public int getInitSleepingCapacity() {
        return this._initSleepingCapacity;
    }

    public int getTotActive() {
        return this._totActive;
    }

    public int getTotIdle() {
        return this._totIdle;
    }

    public Map<K, Integer> getActiveCount() {
        return this._activeCount;
    }
}
