package org.jboss.netty.util.internal;

import java.lang.ref.Reference;
import java.lang.ref.ReferenceQueue;
import java.lang.ref.WeakReference;
import java.util.AbstractCollection;
import java.util.AbstractMap;
import java.util.AbstractSet;
import java.util.Collection;
import java.util.Enumeration;
import java.util.Iterator;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Set;
import java.util.concurrent.ConcurrentMap;
import java.util.concurrent.locks.ReentrantLock;
import org.xbill.DNS.TTL;

public final class ConcurrentWeakKeyHashMap<K, V> extends AbstractMap<K, V> implements ConcurrentMap<K, V> {
    static final int DEFAULT_CONCURRENCY_LEVEL = 16;
    static final int DEFAULT_INITIAL_CAPACITY = 16;
    static final float DEFAULT_LOAD_FACTOR = 0.75f;
    static final int MAXIMUM_CAPACITY = 1073741824;
    static final int MAX_SEGMENTS = 65536;
    static final int RETRIES_BEFORE_LOCK = 2;
    Set<Map.Entry<K, V>> entrySet;
    Set<K> keySet;
    final int segmentMask;
    final int segmentShift;
    final Segment<K, V>[] segments;
    Collection<V> values;

    private static int hash(int h) {
        int h2 = h + ((h << 15) ^ -12931);
        int h3 = h2 ^ (h2 >>> 10);
        int h4 = h3 + (h3 << 3);
        int h5 = h4 ^ (h4 >>> 6);
        int h6 = h5 + (h5 << 2) + (h5 << 14);
        return (h6 >>> 16) ^ h6;
    }

    /* access modifiers changed from: package-private */
    public Segment<K, V> segmentFor(int hash) {
        return this.segments[(hash >>> this.segmentShift) & this.segmentMask];
    }

    private static int hashOf(Object key) {
        return hash(key.hashCode());
    }

    static final class WeakKeyReference<K> extends WeakReference<K> {
        final int hash;

        WeakKeyReference(K key, int hash2, ReferenceQueue<Object> refQueue) {
            super(key, refQueue);
            this.hash = hash2;
        }

        public int keyHash() {
            return this.hash;
        }

        public Object keyRef() {
            return this;
        }
    }

    static final class HashEntry<K, V> {
        final int hash;
        final Object keyRef;
        final HashEntry<K, V> next;
        volatile Object valueRef;

        HashEntry(K key, int hash2, HashEntry<K, V> next2, V value, ReferenceQueue<Object> refQueue) {
            this.hash = hash2;
            this.next = next2;
            this.keyRef = new WeakKeyReference(key, hash2, refQueue);
            this.valueRef = value;
        }

        /* access modifiers changed from: package-private */
        public K key() {
            return ((WeakReference) this.keyRef).get();
        }

        /* access modifiers changed from: package-private */
        public V value() {
            return dereferenceValue(this.valueRef);
        }

        /* access modifiers changed from: package-private */
        public V dereferenceValue(Object value) {
            if (value instanceof WeakKeyReference) {
                return ((Reference) value).get();
            }
            return value;
        }

        /* access modifiers changed from: package-private */
        public void setValue(V value) {
            this.valueRef = value;
        }

        static <K, V> HashEntry<K, V>[] newArray(int i) {
            return new HashEntry[i];
        }
    }

    static final class Segment<K, V> extends ReentrantLock {
        private static final long serialVersionUID = -8328104880676891126L;
        volatile transient int count;
        final float loadFactor;
        int modCount;
        volatile transient ReferenceQueue<Object> refQueue;
        volatile transient HashEntry<K, V>[] table;
        int threshold;

        Segment(int initialCapacity, float lf) {
            this.loadFactor = lf;
            setTable(HashEntry.newArray(initialCapacity));
        }

        static <K, V> Segment<K, V>[] newArray(int i) {
            return new Segment[i];
        }

        private static boolean keyEq(Object src, Object dest) {
            return src.equals(dest);
        }

        /* access modifiers changed from: package-private */
        public void setTable(HashEntry<K, V>[] newTable) {
            this.threshold = (int) (((float) newTable.length) * this.loadFactor);
            this.table = newTable;
            this.refQueue = new ReferenceQueue<>();
        }

        /* access modifiers changed from: package-private */
        public HashEntry<K, V> getFirst(int hash) {
            HashEntry<K, V>[] tab = this.table;
            return tab[(tab.length - 1) & hash];
        }

        /* access modifiers changed from: package-private */
        public HashEntry<K, V> newHashEntry(K key, int hash, HashEntry<K, V> next, V value) {
            return new HashEntry(key, hash, next, value, this.refQueue);
        }

        /* access modifiers changed from: package-private */
        public V readValueUnderLock(HashEntry<K, V> e) {
            lock();
            try {
                removeStale();
                return e.value();
            } finally {
                unlock();
            }
        }

        /* access modifiers changed from: package-private */
        public V get(Object key, int hash) {
            if (this.count == 0) {
                return null;
            }
            HashEntry<K, V> e = getFirst(hash);
            while (e != null) {
                if (e.hash != hash || !keyEq(key, e.key())) {
                    e = e.next;
                } else {
                    Object opaque = e.valueRef;
                    if (opaque != null) {
                        return e.dereferenceValue(opaque);
                    }
                    return readValueUnderLock(e);
                }
            }
            return null;
        }

        /* access modifiers changed from: package-private */
        public boolean containsKey(Object key, int hash) {
            if (this.count == 0) {
                return false;
            }
            for (HashEntry<K, V> e = getFirst(hash); e != null; e = e.next) {
                if (e.hash == hash && keyEq(key, e.key())) {
                    return true;
                }
            }
            return false;
        }

        /* access modifiers changed from: package-private */
        public boolean containsValue(Object value) {
            V v;
            if (this.count != 0) {
                for (HashEntry<K, V> e : this.table) {
                    while (e != null) {
                        Object opaque = e.valueRef;
                        if (opaque == null) {
                            v = readValueUnderLock(e);
                        } else {
                            v = e.dereferenceValue(opaque);
                        }
                        if (value.equals(v)) {
                            return true;
                        }
                        e = e.next;
                    }
                }
            }
            return false;
        }

        /* access modifiers changed from: package-private */
        public boolean replace(K key, int hash, V oldValue, V newValue) {
            lock();
            try {
                removeStale();
                HashEntry<K, V> e = getFirst(hash);
                while (e != null && (e.hash != hash || !keyEq(key, e.key()))) {
                    e = e.next;
                }
                boolean replaced = false;
                if (e != null && oldValue.equals(e.value())) {
                    replaced = true;
                    e.setValue(newValue);
                }
                return replaced;
            } finally {
                unlock();
            }
        }

        /* access modifiers changed from: package-private */
        public V replace(K key, int hash, V newValue) {
            lock();
            try {
                removeStale();
                HashEntry<K, V> e = getFirst(hash);
                while (e != null && (e.hash != hash || !keyEq(key, e.key()))) {
                    e = e.next;
                }
                V oldValue = null;
                if (e != null) {
                    oldValue = e.value();
                    e.setValue(newValue);
                }
                return oldValue;
            } finally {
                unlock();
            }
        }

        /* access modifiers changed from: package-private */
        public V put(K key, int hash, V value, boolean onlyIfAbsent) {
            V oldValue;
            int reduced;
            lock();
            try {
                removeStale();
                int c = this.count;
                int c2 = c + 1;
                if (c > this.threshold && (reduced = rehash()) > 0) {
                    int i = c2 - reduced;
                    c2 = i;
                    this.count = i - 1;
                }
                HashEntry<K, V>[] tab = this.table;
                int index = (tab.length - 1) & hash;
                HashEntry<K, V> first = tab[index];
                HashEntry<K, V> e = first;
                while (e != null && (e.hash != hash || !keyEq(key, e.key()))) {
                    e = e.next;
                }
                if (e != null) {
                    oldValue = e.value();
                    if (!onlyIfAbsent) {
                        e.setValue(value);
                    }
                } else {
                    oldValue = null;
                    this.modCount++;
                    tab[index] = newHashEntry(key, hash, first, value);
                    this.count = c2;
                }
                return oldValue;
            } finally {
                unlock();
            }
        }

        /* access modifiers changed from: package-private */
        public int rehash() {
            int oldCapacity;
            HashEntry<K, V>[] oldTable;
            int oldCapacity2;
            HashEntry<K, V>[] oldTable2;
            HashEntry<K, V>[] oldTable3 = this.table;
            int oldCapacity3 = oldTable3.length;
            int i = 0;
            if (oldCapacity3 >= 1073741824) {
                return 0;
            }
            HashEntry<K, V>[] newTable = HashEntry.newArray(oldCapacity3 << 1);
            this.threshold = (int) (((float) newTable.length) * this.loadFactor);
            int sizeMask = newTable.length - 1;
            int reduce = 0;
            while (i < oldCapacity3) {
                HashEntry<K, V> e = oldTable3[i];
                if (e != null) {
                    HashEntry<K, V> next = e.next;
                    int idx = e.hash & sizeMask;
                    if (next == null) {
                        newTable[idx] = e;
                        oldTable = oldTable3;
                        oldCapacity = oldCapacity3;
                    } else {
                        int lastIdx = idx;
                        HashEntry<K, V> lastRun = e;
                        for (HashEntry<K, V> last = next; last != null; last = last.next) {
                            int k = last.hash & sizeMask;
                            if (k != lastIdx) {
                                lastIdx = k;
                                lastRun = last;
                            }
                        }
                        newTable[lastIdx] = lastRun;
                        int reduce2 = reduce;
                        HashEntry<K, V> p = e;
                        while (p != lastRun) {
                            K key = p.key();
                            if (key == null) {
                                reduce2++;
                                oldTable2 = oldTable3;
                                oldCapacity2 = oldCapacity3;
                            } else {
                                int k2 = p.hash & sizeMask;
                                oldTable2 = oldTable3;
                                oldCapacity2 = oldCapacity3;
                                newTable[k2] = newHashEntry(key, p.hash, newTable[k2], p.value());
                            }
                            p = p.next;
                            oldTable3 = oldTable2;
                            oldCapacity3 = oldCapacity2;
                        }
                        oldTable = oldTable3;
                        oldCapacity = oldCapacity3;
                        reduce = reduce2;
                    }
                } else {
                    oldTable = oldTable3;
                    oldCapacity = oldCapacity3;
                }
                i++;
                oldTable3 = oldTable;
                oldCapacity3 = oldCapacity;
            }
            int i2 = oldCapacity3;
            this.table = newTable;
            return reduce;
        }

        /* access modifiers changed from: package-private */
        public V remove(Object key, int hash, Object value, boolean refRemove) {
            Object obj = key;
            int i = hash;
            Object obj2 = value;
            lock();
            if (!refRemove) {
                try {
                    removeStale();
                } catch (Throwable th) {
                    Object obj3 = key;
                    int i2 = hash;
                    Object obj4 = value;
                    boolean z = refRemove;
                    unlock();
                    throw th;
                }
            }
            int c = this.count - 1;
            HashEntry<K, V>[] tab = this.table;
            int index = (tab.length - 1) & i;
            HashEntry<K, V> first = tab[index];
            HashEntry<K, V> e = first;
            while (e != null && obj != e.keyRef && (refRemove || i != e.hash || !keyEq(obj, e.key()))) {
                e = e.next;
            }
            V oldValue = null;
            if (e != null) {
                V v = e.value();
                if (obj2 == null || obj2.equals(v)) {
                    oldValue = v;
                    this.modCount++;
                    HashEntry<K, V> newFirst = e.next;
                    int c2 = c;
                    HashEntry<K, V> p = first;
                    while (p != e) {
                        K pKey = p.key();
                        if (pKey == null) {
                            c2--;
                        } else {
                            newFirst = newHashEntry(pKey, p.hash, newFirst, p.value());
                        }
                        p = p.next;
                        Object obj5 = key;
                    }
                    tab[index] = newFirst;
                    this.count = c2;
                    Object obj6 = key;
                    int i3 = hash;
                    Object obj7 = value;
                    boolean z2 = refRemove;
                    unlock();
                    return oldValue;
                }
            }
            Object obj62 = key;
            int i32 = hash;
            Object obj72 = value;
            boolean z22 = refRemove;
            unlock();
            return oldValue;
        }

        /* access modifiers changed from: package-private */
        public void removeStale() {
            while (true) {
                WeakKeyReference weakKeyReference = (WeakKeyReference) this.refQueue.poll();
                WeakKeyReference ref = weakKeyReference;
                if (weakKeyReference != null) {
                    remove(ref.keyRef(), ref.keyHash(), (Object) null, true);
                } else {
                    return;
                }
            }
        }

        /* access modifiers changed from: package-private */
        public void clear() {
            if (this.count != 0) {
                lock();
                try {
                    HashEntry<K, V>[] tab = this.table;
                    for (int i = 0; i < tab.length; i++) {
                        tab[i] = null;
                    }
                    this.modCount++;
                    this.refQueue = new ReferenceQueue<>();
                    this.count = 0;
                    unlock();
                } catch (Throwable th) {
                    unlock();
                    throw th;
                }
            }
        }
    }

    public ConcurrentWeakKeyHashMap(int initialCapacity, float loadFactor, int concurrencyLevel) {
        if (loadFactor <= 0.0f || initialCapacity < 0 || concurrencyLevel <= 0) {
            throw new IllegalArgumentException();
        }
        int cap = 1;
        int sshift = 0;
        int ssize = 1;
        while (ssize < (concurrencyLevel > 65536 ? 65536 : concurrencyLevel)) {
            sshift++;
            ssize <<= 1;
        }
        this.segmentShift = 32 - sshift;
        this.segmentMask = ssize - 1;
        this.segments = Segment.newArray(ssize);
        initialCapacity = initialCapacity > 1073741824 ? 1073741824 : initialCapacity;
        int c = initialCapacity / ssize;
        while (cap < (c * ssize < initialCapacity ? c + 1 : c)) {
            cap <<= 1;
        }
        for (int i = 0; i < this.segments.length; i++) {
            this.segments[i] = new Segment<>(cap, loadFactor);
        }
    }

    public ConcurrentWeakKeyHashMap(int initialCapacity, float loadFactor) {
        this(initialCapacity, loadFactor, 16);
    }

    public ConcurrentWeakKeyHashMap(int initialCapacity) {
        this(initialCapacity, DEFAULT_LOAD_FACTOR, 16);
    }

    public ConcurrentWeakKeyHashMap() {
        this(16, DEFAULT_LOAD_FACTOR, 16);
    }

    public ConcurrentWeakKeyHashMap(Map<? extends K, ? extends V> m) {
        this(Math.max(((int) (((float) m.size()) / DEFAULT_LOAD_FACTOR)) + 1, 16), DEFAULT_LOAD_FACTOR, 16);
        putAll(m);
    }

    public boolean isEmpty() {
        Segment<K, V>[] segments2 = this.segments;
        int[] mc = new int[segments2.length];
        int mcsum = 0;
        for (int i = 0; i < segments2.length; i++) {
            if (segments2[i].count != 0) {
                return false;
            }
            int i2 = segments2[i].modCount;
            mc[i] = i2;
            mcsum += i2;
        }
        if (mcsum == 0) {
            return true;
        }
        for (int i3 = 0; i3 < segments2.length; i3++) {
            if (segments2[i3].count != 0 || mc[i3] != segments2[i3].modCount) {
                return false;
            }
        }
        return true;
    }

    public int size() {
        Segment<K, V>[] segments2 = this.segments;
        long check = 0;
        int[] mc = new int[segments2.length];
        int i = 0;
        long sum = 0;
        for (int k = 0; k < 2; k++) {
            sum = 0;
            int mcsum = 0;
            for (int i2 = 0; i2 < segments2.length; i2++) {
                sum += (long) segments2[i2].count;
                int i3 = segments2[i2].modCount;
                mc[i2] = i3;
                mcsum += i3;
            }
            if (mcsum != 0) {
                check = 0;
                int i4 = 0;
                while (true) {
                    if (i4 >= segments2.length) {
                        break;
                    }
                    check += (long) segments2[i4].count;
                    if (mc[i4] != segments2[i4].modCount) {
                        check = -1;
                        break;
                    }
                    i4++;
                }
            } else {
                check = 0;
            }
            if (check == sum) {
                break;
            }
        }
        if (check != sum) {
            for (Segment<K, V> lock : segments2) {
                lock.lock();
            }
            long sum2 = 0;
            for (Segment<K, V> segment : segments2) {
                sum2 = sum + ((long) segment.count);
            }
            while (true) {
                int i5 = i;
                if (i5 >= segments2.length) {
                    break;
                }
                segments2[i5].unlock();
                i = i5 + 1;
            }
        }
        if (sum > TTL.MAX_VALUE) {
            return Integer.MAX_VALUE;
        }
        return (int) sum;
    }

    public V get(Object key) {
        int hash = hashOf(key);
        return segmentFor(hash).get(key, hash);
    }

    public boolean containsKey(Object key) {
        int hash = hashOf(key);
        return segmentFor(hash).containsKey(key, hash);
    }

    /*  JADX ERROR: StackOverflow in pass: MarkFinallyVisitor
        jadx.core.utils.exceptions.JadxOverflowException: 
        	at jadx.core.utils.ErrorsCounter.addError(ErrorsCounter.java:47)
        	at jadx.core.utils.ErrorsCounter.methodError(ErrorsCounter.java:81)
        */
    public boolean containsValue(java.lang.Object r10) {
        /*
            r9 = this;
            if (r10 == 0) goto L_0x007b
            org.jboss.netty.util.internal.ConcurrentWeakKeyHashMap$Segment<K, V>[] r0 = r9.segments
            int r1 = r0.length
            int[] r1 = new int[r1]
            r2 = 0
            r3 = 0
        L_0x0009:
            r4 = 2
            if (r3 >= r4) goto L_0x0040
            r4 = 0
            r5 = r4
            r4 = 0
        L_0x000f:
            int r6 = r0.length
            if (r4 >= r6) goto L_0x0026
            r6 = r0[r4]
            int r6 = r6.modCount
            r1[r4] = r6
            int r5 = r5 + r6
            r6 = r0[r4]
            boolean r6 = r6.containsValue(r10)
            if (r6 == 0) goto L_0x0023
            r2 = 1
            return r2
        L_0x0023:
            int r4 = r4 + 1
            goto L_0x000f
        L_0x0026:
            r4 = 1
            if (r5 == 0) goto L_0x003a
            r6 = 0
        L_0x002a:
            int r7 = r0.length
            if (r6 >= r7) goto L_0x003a
            r7 = r1[r6]
            r8 = r0[r6]
            int r8 = r8.modCount
            if (r7 == r8) goto L_0x0037
            r4 = 0
            goto L_0x003a
        L_0x0037:
            int r6 = r6 + 1
            goto L_0x002a
        L_0x003a:
            if (r4 == 0) goto L_0x003d
            return r2
        L_0x003d:
            int r3 = r3 + 1
            goto L_0x0009
        L_0x0040:
            r3 = 0
        L_0x0041:
            int r4 = r0.length
            if (r3 >= r4) goto L_0x004c
            r4 = r0[r3]
            r4.lock()
            int r3 = r3 + 1
            goto L_0x0041
        L_0x004c:
            r3 = r2
            r4 = 0
        L_0x004e:
            int r5 = r0.length     // Catch:{ all -> 0x006d }
            if (r4 >= r5) goto L_0x005e
            r5 = r0[r4]     // Catch:{ all -> 0x006d }
            boolean r5 = r5.containsValue(r10)     // Catch:{ all -> 0x006d }
            if (r5 == 0) goto L_0x005b
            r3 = 1
            goto L_0x005e
        L_0x005b:
            int r4 = r4 + 1
            goto L_0x004e
        L_0x005e:
            r4 = r9
        L_0x0060:
            int r5 = r0.length
            if (r2 >= r5) goto L_0x006b
            r5 = r0[r2]
            r5.unlock()
            int r2 = r2 + 1
            goto L_0x0060
        L_0x006b:
            return r3
        L_0x006d:
            r4 = move-exception
            r5 = r9
        L_0x006f:
            int r6 = r0.length
            if (r2 >= r6) goto L_0x007a
            r6 = r0[r2]
            r6.unlock()
            int r2 = r2 + 1
            goto L_0x006f
        L_0x007a:
            throw r4
        L_0x007b:
            java.lang.NullPointerException r0 = new java.lang.NullPointerException
            r0.<init>()
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.util.internal.ConcurrentWeakKeyHashMap.containsValue(java.lang.Object):boolean");
    }

    public boolean contains(Object value) {
        return containsValue(value);
    }

    public V put(K key, V value) {
        if (value != null) {
            int hash = hashOf(key);
            return segmentFor(hash).put(key, hash, value, false);
        }
        throw new NullPointerException();
    }

    public V putIfAbsent(K key, V value) {
        if (value != null) {
            int hash = hashOf(key);
            return segmentFor(hash).put(key, hash, value, true);
        }
        throw new NullPointerException();
    }

    public void putAll(Map<? extends K, ? extends V> m) {
        for (Map.Entry<? extends K, ? extends V> e : m.entrySet()) {
            put(e.getKey(), e.getValue());
        }
    }

    public V remove(Object key) {
        int hash = hashOf(key);
        return segmentFor(hash).remove(key, hash, (Object) null, false);
    }

    public boolean remove(Object key, Object value) {
        int hash = hashOf(key);
        if (value == null || segmentFor(hash).remove(key, hash, value, false) == null) {
            return false;
        }
        return true;
    }

    public boolean replace(K key, V oldValue, V newValue) {
        if (oldValue == null || newValue == null) {
            throw new NullPointerException();
        }
        int hash = hashOf(key);
        return segmentFor(hash).replace(key, hash, oldValue, newValue);
    }

    public V replace(K key, V value) {
        if (value != null) {
            int hash = hashOf(key);
            return segmentFor(hash).replace(key, hash, value);
        }
        throw new NullPointerException();
    }

    public void clear() {
        for (Segment<K, V> clear : this.segments) {
            clear.clear();
        }
    }

    public void purgeStaleEntries() {
        for (Segment<K, V> removeStale : this.segments) {
            removeStale.removeStale();
        }
    }

    public Set<K> keySet() {
        Set<K> ks = this.keySet;
        if (ks != null) {
            return ks;
        }
        KeySet keySet2 = new KeySet();
        this.keySet = keySet2;
        return keySet2;
    }

    public Collection<V> values() {
        Collection<V> vs = this.values;
        if (vs != null) {
            return vs;
        }
        Values values2 = new Values();
        this.values = values2;
        return values2;
    }

    public Set<Map.Entry<K, V>> entrySet() {
        Set<Map.Entry<K, V>> es = this.entrySet;
        if (es != null) {
            return es;
        }
        EntrySet entrySet2 = new EntrySet();
        this.entrySet = entrySet2;
        return entrySet2;
    }

    public Enumeration<K> keys() {
        return new KeyIterator();
    }

    public Enumeration<V> elements() {
        return new ValueIterator();
    }

    abstract class HashIterator {
        K currentKey;
        HashEntry<K, V>[] currentTable;
        HashEntry<K, V> lastReturned;
        HashEntry<K, V> nextEntry;
        int nextSegmentIndex;
        int nextTableIndex = -1;

        HashIterator() {
            this.nextSegmentIndex = ConcurrentWeakKeyHashMap.this.segments.length - 1;
            advance();
        }

        public void rewind() {
            this.nextSegmentIndex = ConcurrentWeakKeyHashMap.this.segments.length - 1;
            this.nextTableIndex = -1;
            this.currentTable = null;
            this.nextEntry = null;
            this.lastReturned = null;
            this.currentKey = null;
            advance();
        }

        public boolean hasMoreElements() {
            return hasNext();
        }

        /* access modifiers changed from: package-private */
        public final void advance() {
            if (this.nextEntry != null) {
                HashEntry<K, V> hashEntry = this.nextEntry.next;
                this.nextEntry = hashEntry;
                if (hashEntry != null) {
                    return;
                }
            }
            while (this.nextTableIndex >= 0) {
                HashEntry<K, V>[] hashEntryArr = this.currentTable;
                int i = this.nextTableIndex;
                this.nextTableIndex = i - 1;
                HashEntry<K, V> hashEntry2 = hashEntryArr[i];
                this.nextEntry = hashEntry2;
                if (hashEntry2 != null) {
                    return;
                }
            }
            while (this.nextSegmentIndex >= 0) {
                Segment<K, V>[] segmentArr = ConcurrentWeakKeyHashMap.this.segments;
                int i2 = this.nextSegmentIndex;
                this.nextSegmentIndex = i2 - 1;
                Segment<K, V> seg = segmentArr[i2];
                if (seg.count != 0) {
                    this.currentTable = seg.table;
                    for (int j = this.currentTable.length - 1; j >= 0; j--) {
                        HashEntry<K, V> hashEntry3 = this.currentTable[j];
                        this.nextEntry = hashEntry3;
                        if (hashEntry3 != null) {
                            this.nextTableIndex = j - 1;
                            return;
                        }
                    }
                    continue;
                }
            }
        }

        public boolean hasNext() {
            while (this.nextEntry != null) {
                if (this.nextEntry.key() != null) {
                    return true;
                }
                advance();
            }
            return false;
        }

        /* access modifiers changed from: package-private */
        public HashEntry<K, V> nextEntry() {
            while (this.nextEntry != null) {
                this.lastReturned = this.nextEntry;
                this.currentKey = this.lastReturned.key();
                advance();
                if (this.currentKey != null) {
                    return this.lastReturned;
                }
            }
            throw new NoSuchElementException();
        }

        public void remove() {
            if (this.lastReturned != null) {
                ConcurrentWeakKeyHashMap.this.remove(this.currentKey);
                this.lastReturned = null;
                return;
            }
            throw new IllegalStateException();
        }
    }

    final class KeyIterator extends ConcurrentWeakKeyHashMap<K, V>.HashIterator implements ReusableIterator<K>, Enumeration<K> {
        KeyIterator() {
            super();
        }

        public K next() {
            return super.nextEntry().key();
        }

        public K nextElement() {
            return super.nextEntry().key();
        }
    }

    final class ValueIterator extends ConcurrentWeakKeyHashMap<K, V>.HashIterator implements ReusableIterator<V>, Enumeration<V> {
        ValueIterator() {
            super();
        }

        public V next() {
            return super.nextEntry().value();
        }

        public V nextElement() {
            return super.nextEntry().value();
        }
    }

    static class SimpleEntry<K, V> implements Map.Entry<K, V> {
        private final K key;
        private V value;

        public SimpleEntry(K key2, V value2) {
            this.key = key2;
            this.value = value2;
        }

        public SimpleEntry(Map.Entry<? extends K, ? extends V> entry) {
            this.key = entry.getKey();
            this.value = entry.getValue();
        }

        public K getKey() {
            return this.key;
        }

        public V getValue() {
            return this.value;
        }

        public V setValue(V value2) {
            V oldValue = this.value;
            this.value = value2;
            return oldValue;
        }

        public boolean equals(Object o) {
            if (!(o instanceof Map.Entry)) {
                return false;
            }
            Map.Entry e = (Map.Entry) o;
            if (!eq(this.key, e.getKey()) || !eq(this.value, e.getValue())) {
                return false;
            }
            return true;
        }

        public int hashCode() {
            int i = 0;
            int hashCode = this.key == null ? 0 : this.key.hashCode();
            if (this.value != null) {
                i = this.value.hashCode();
            }
            return hashCode ^ i;
        }

        public String toString() {
            return this.key + "=" + this.value;
        }

        private static boolean eq(Object o1, Object o2) {
            if (o1 == null) {
                return o2 == null;
            }
            return o1.equals(o2);
        }
    }

    final class WriteThroughEntry extends SimpleEntry<K, V> {
        WriteThroughEntry(K k, V v) {
            super(k, v);
        }

        public V setValue(V value) {
            if (value != null) {
                V v = super.setValue(value);
                ConcurrentWeakKeyHashMap.this.put(getKey(), value);
                return v;
            }
            throw new NullPointerException();
        }
    }

    final class EntryIterator extends ConcurrentWeakKeyHashMap<K, V>.HashIterator implements ReusableIterator<Map.Entry<K, V>> {
        EntryIterator() {
            super();
        }

        public Map.Entry<K, V> next() {
            HashEntry<K, V> e = super.nextEntry();
            return new WriteThroughEntry(e.key(), e.value());
        }
    }

    final class KeySet extends AbstractSet<K> {
        KeySet() {
        }

        public Iterator<K> iterator() {
            return new KeyIterator();
        }

        public int size() {
            return ConcurrentWeakKeyHashMap.this.size();
        }

        public boolean isEmpty() {
            return ConcurrentWeakKeyHashMap.this.isEmpty();
        }

        public boolean contains(Object o) {
            return ConcurrentWeakKeyHashMap.this.containsKey(o);
        }

        public boolean remove(Object o) {
            return ConcurrentWeakKeyHashMap.this.remove(o) != null;
        }

        public void clear() {
            ConcurrentWeakKeyHashMap.this.clear();
        }
    }

    final class Values extends AbstractCollection<V> {
        Values() {
        }

        public Iterator<V> iterator() {
            return new ValueIterator();
        }

        public int size() {
            return ConcurrentWeakKeyHashMap.this.size();
        }

        public boolean isEmpty() {
            return ConcurrentWeakKeyHashMap.this.isEmpty();
        }

        public boolean contains(Object o) {
            return ConcurrentWeakKeyHashMap.this.containsValue(o);
        }

        public void clear() {
            ConcurrentWeakKeyHashMap.this.clear();
        }
    }

    final class EntrySet extends AbstractSet<Map.Entry<K, V>> {
        EntrySet() {
        }

        public Iterator<Map.Entry<K, V>> iterator() {
            return new EntryIterator();
        }

        public boolean contains(Object o) {
            if (!(o instanceof Map.Entry)) {
                return false;
            }
            Map.Entry<?, ?> e = (Map.Entry) o;
            V v = ConcurrentWeakKeyHashMap.this.get(e.getKey());
            if (v == null || !v.equals(e.getValue())) {
                return false;
            }
            return true;
        }

        public boolean remove(Object o) {
            if (!(o instanceof Map.Entry)) {
                return false;
            }
            Map.Entry<?, ?> e = (Map.Entry) o;
            return ConcurrentWeakKeyHashMap.this.remove(e.getKey(), e.getValue());
        }

        public int size() {
            return ConcurrentWeakKeyHashMap.this.size();
        }

        public boolean isEmpty() {
            return ConcurrentWeakKeyHashMap.this.isEmpty();
        }

        public void clear() {
            ConcurrentWeakKeyHashMap.this.clear();
        }
    }
}
