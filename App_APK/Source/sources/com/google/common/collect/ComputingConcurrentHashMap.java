package com.google.common.collect;

import com.google.common.base.Equivalence;
import com.google.common.base.Function;
import com.google.common.base.Preconditions;
import com.google.common.base.Throwables;
import com.google.common.collect.MapMaker;
import com.google.common.collect.MapMakerInternalMap;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.lang.ref.ReferenceQueue;
import java.util.concurrent.ConcurrentMap;
import java.util.concurrent.ExecutionException;
import javax.annotation.Nullable;
import javax.annotation.concurrent.GuardedBy;

class ComputingConcurrentHashMap<K, V> extends MapMakerInternalMap<K, V> {
    private static final long serialVersionUID = 4;
    final Function<? super K, ? extends V> computingFunction;

    ComputingConcurrentHashMap(MapMaker builder, Function<? super K, ? extends V> computingFunction2) {
        super(builder);
        this.computingFunction = (Function) Preconditions.checkNotNull(computingFunction2);
    }

    /* access modifiers changed from: package-private */
    public MapMakerInternalMap.Segment<K, V> createSegment(int initialCapacity, int maxSegmentSize) {
        return new ComputingSegment(this, initialCapacity, maxSegmentSize);
    }

    /* access modifiers changed from: package-private */
    public ComputingSegment<K, V> segmentFor(int hash) {
        return (ComputingSegment) super.segmentFor(hash);
    }

    /* access modifiers changed from: package-private */
    public V getOrCompute(K key) throws ExecutionException {
        int hash = hash(Preconditions.checkNotNull(key));
        return segmentFor(hash).getOrCompute(key, hash, this.computingFunction);
    }

    static final class ComputingSegment<K, V> extends MapMakerInternalMap.Segment<K, V> {
        ComputingSegment(MapMakerInternalMap<K, V> map, int initialCapacity, int maxSegmentSize) {
            super(map, initialCapacity, maxSegmentSize);
        }

        /* access modifiers changed from: package-private */
        /* JADX WARNING: Code restructure failed: missing block: B:11:0x001e, code lost:
            if (r0.getValueReference().isComputingReference() == false) goto L_0x0020;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:22:0x005c, code lost:
            if (r0.getValueReference().isComputingReference() == false) goto L_0x0060;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:23:0x005e, code lost:
            r1 = false;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:24:0x0060, code lost:
            r9 = r0.getValueReference().get();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:25:0x0068, code lost:
            if (r9 != null) goto L_0x0070;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:26:0x006a, code lost:
            enqueueNotification(r7, r13, r9, com.google.common.collect.MapMaker.RemovalCause.COLLECTED);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:28:0x0076, code lost:
            if (r11.map.expires() == false) goto L_0x0092;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:30:0x007e, code lost:
            if (r11.map.isExpired(r0) == false) goto L_0x0092;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:31:0x0080, code lost:
            enqueueNotification(r7, r13, r9, com.google.common.collect.MapMaker.RemovalCause.EXPIRED);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:32:0x0085, code lost:
            r11.evictionQueue.remove(r0);
            r11.expirationQueue.remove(r0);
            r11.count = r3;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:6:0x0010, code lost:
            postReadCleanup();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:7:0x0013, code lost:
            return r1;
         */
        /* JADX WARNING: Removed duplicated region for block: B:0:0x0000 A[LOOP_START, MTH_ENTER_BLOCK, SYNTHETIC, Splitter:B:0:0x0000] */
        /* Code decompiled incorrectly, please refer to instructions dump. */
        public V getOrCompute(K r12, int r13, com.google.common.base.Function<? super K, ? extends V> r14) throws java.util.concurrent.ExecutionException {
            /*
                r11 = this;
            L_0x0000:
                com.google.common.collect.MapMakerInternalMap$ReferenceEntry r0 = r11.getEntry(r12, r13)     // Catch:{ all -> 0x00f2 }
                if (r0 == 0) goto L_0x0014
                java.lang.Object r1 = r11.getLiveValue(r0)     // Catch:{ all -> 0x00f2 }
                if (r1 == 0) goto L_0x0014
                r11.recordRead(r0)     // Catch:{ all -> 0x00f2 }
            L_0x0010:
                r11.postReadCleanup()
                return r1
            L_0x0014:
                if (r0 == 0) goto L_0x0020
                com.google.common.collect.MapMakerInternalMap$ValueReference r1 = r0.getValueReference()     // Catch:{ all -> 0x00f2 }
                boolean r1 = r1.isComputingReference()     // Catch:{ all -> 0x00f2 }
                if (r1 != 0) goto L_0x00d0
            L_0x0020:
                r1 = 1
                r2 = 0
                r11.lock()     // Catch:{ all -> 0x00f2 }
                r11.preWriteCleanup()     // Catch:{ all -> 0x00ea }
                int r3 = r11.count     // Catch:{ all -> 0x00ea }
                int r3 = r3 + -1
                java.util.concurrent.atomic.AtomicReferenceArray r4 = r11.table     // Catch:{ all -> 0x00ea }
                int r5 = r4.length()     // Catch:{ all -> 0x00ea }
                int r5 = r5 + -1
                r5 = r5 & r13
                java.lang.Object r6 = r4.get(r5)     // Catch:{ all -> 0x00ea }
                com.google.common.collect.MapMakerInternalMap$ReferenceEntry r6 = (com.google.common.collect.MapMakerInternalMap.ReferenceEntry) r6     // Catch:{ all -> 0x00ea }
                r0 = r6
            L_0x003c:
                if (r0 == 0) goto L_0x00a6
                java.lang.Object r7 = r0.getKey()     // Catch:{ all -> 0x00ea }
                int r8 = r0.getHash()     // Catch:{ all -> 0x00ea }
                if (r8 != r13) goto L_0x00a0
                if (r7 == 0) goto L_0x00a0
                com.google.common.collect.MapMakerInternalMap r8 = r11.map     // Catch:{ all -> 0x00ea }
                com.google.common.base.Equivalence<java.lang.Object> r8 = r8.keyEquivalence     // Catch:{ all -> 0x00ea }
                boolean r8 = r8.equivalent(r12, r7)     // Catch:{ all -> 0x00ea }
                if (r8 == 0) goto L_0x00a0
                com.google.common.collect.MapMakerInternalMap$ValueReference r8 = r0.getValueReference()     // Catch:{ all -> 0x00ea }
                boolean r9 = r8.isComputingReference()     // Catch:{ all -> 0x00ea }
                if (r9 == 0) goto L_0x0060
                r1 = 0
                goto L_0x00a6
            L_0x0060:
                com.google.common.collect.MapMakerInternalMap$ValueReference r9 = r0.getValueReference()     // Catch:{ all -> 0x00ea }
                java.lang.Object r9 = r9.get()     // Catch:{ all -> 0x00ea }
                if (r9 != 0) goto L_0x0070
                com.google.common.collect.MapMaker$RemovalCause r10 = com.google.common.collect.MapMaker.RemovalCause.COLLECTED     // Catch:{ all -> 0x00ea }
                r11.enqueueNotification(r7, r13, r9, r10)     // Catch:{ all -> 0x00ea }
                goto L_0x0085
            L_0x0070:
                com.google.common.collect.MapMakerInternalMap r10 = r11.map     // Catch:{ all -> 0x00ea }
                boolean r10 = r10.expires()     // Catch:{ all -> 0x00ea }
                if (r10 == 0) goto L_0x0092
                com.google.common.collect.MapMakerInternalMap r10 = r11.map     // Catch:{ all -> 0x00ea }
                boolean r10 = r10.isExpired(r0)     // Catch:{ all -> 0x00ea }
                if (r10 == 0) goto L_0x0092
                com.google.common.collect.MapMaker$RemovalCause r10 = com.google.common.collect.MapMaker.RemovalCause.EXPIRED     // Catch:{ all -> 0x00ea }
                r11.enqueueNotification(r7, r13, r9, r10)     // Catch:{ all -> 0x00ea }
            L_0x0085:
                java.util.Queue r10 = r11.evictionQueue     // Catch:{ all -> 0x00ea }
                r10.remove(r0)     // Catch:{ all -> 0x00ea }
                java.util.Queue r10 = r11.expirationQueue     // Catch:{ all -> 0x00ea }
                r10.remove(r0)     // Catch:{ all -> 0x00ea }
                r11.count = r3     // Catch:{ all -> 0x00ea }
                goto L_0x00a6
            L_0x0092:
                r11.recordLockedRead(r0)     // Catch:{ all -> 0x00ea }
                r11.unlock()     // Catch:{ all -> 0x00f2 }
                r11.postWriteCleanup()     // Catch:{ all -> 0x00f2 }
                r11.postReadCleanup()
                return r9
            L_0x00a0:
                com.google.common.collect.MapMakerInternalMap$ReferenceEntry r7 = r0.getNext()     // Catch:{ all -> 0x00ea }
                r0 = r7
                goto L_0x003c
            L_0x00a6:
                if (r1 == 0) goto L_0x00bf
                com.google.common.collect.ComputingConcurrentHashMap$ComputingValueReference r7 = new com.google.common.collect.ComputingConcurrentHashMap$ComputingValueReference     // Catch:{ all -> 0x00ea }
                r7.<init>(r14)     // Catch:{ all -> 0x00ea }
                r2 = r7
                if (r0 != 0) goto L_0x00bc
                com.google.common.collect.MapMakerInternalMap$ReferenceEntry r7 = r11.newEntry(r12, r13, r6)     // Catch:{ all -> 0x00ea }
                r0 = r7
                r0.setValueReference(r2)     // Catch:{ all -> 0x00ea }
                r4.set(r5, r0)     // Catch:{ all -> 0x00ea }
                goto L_0x00bf
            L_0x00bc:
                r0.setValueReference(r2)     // Catch:{ all -> 0x00ea }
            L_0x00bf:
                r11.unlock()     // Catch:{ all -> 0x00f2 }
                r11.postWriteCleanup()     // Catch:{ all -> 0x00f2 }
                if (r1 == 0) goto L_0x00d0
                java.lang.Object r3 = r11.compute(r12, r13, r0, r2)     // Catch:{ all -> 0x00f2 }
                r11.postReadCleanup()
                return r3
            L_0x00d0:
                boolean r1 = java.lang.Thread.holdsLock(r0)     // Catch:{ all -> 0x00f2 }
                r1 = r1 ^ 1
                java.lang.String r2 = "Recursive computation"
                com.google.common.base.Preconditions.checkState(r1, r2)     // Catch:{ all -> 0x00f2 }
                com.google.common.collect.MapMakerInternalMap$ValueReference r1 = r0.getValueReference()     // Catch:{ all -> 0x00f2 }
                java.lang.Object r1 = r1.waitForValue()     // Catch:{ all -> 0x00f2 }
                if (r1 == 0) goto L_0x0000
                r11.recordRead(r0)     // Catch:{ all -> 0x00f2 }
                goto L_0x0010
            L_0x00ea:
                r3 = move-exception
                r11.unlock()     // Catch:{ all -> 0x00f2 }
                r11.postWriteCleanup()     // Catch:{ all -> 0x00f2 }
                throw r3     // Catch:{ all -> 0x00f2 }
            L_0x00f2:
                r0 = move-exception
                r11.postReadCleanup()
                throw r0
            */
            throw new UnsupportedOperationException("Method not decompiled: com.google.common.collect.ComputingConcurrentHashMap.ComputingSegment.getOrCompute(java.lang.Object, int, com.google.common.base.Function):java.lang.Object");
        }

        /* access modifiers changed from: package-private */
        public V compute(K key, int hash, MapMakerInternalMap.ReferenceEntry<K, V> e, ComputingValueReference<K, V> computingValueReference) throws ExecutionException {
            V value = null;
            long nanoTime = System.nanoTime();
            long end = 0;
            try {
                synchronized (e) {
                    value = computingValueReference.compute(key, hash);
                    end = System.nanoTime();
                }
                if (value != null) {
                    if (put(key, hash, value, true) != null) {
                        enqueueNotification(key, hash, value, MapMaker.RemovalCause.REPLACED);
                    }
                }
                if (end == 0) {
                    long end2 = System.nanoTime();
                }
                if (value == null) {
                    clearValue(key, hash, computingValueReference);
                }
                return value;
            } catch (Throwable th) {
                if (end == 0) {
                    long end3 = System.nanoTime();
                }
                if (value == null) {
                    clearValue(key, hash, computingValueReference);
                }
                throw th;
            }
        }
    }

    private static final class ComputationExceptionReference<K, V> implements MapMakerInternalMap.ValueReference<K, V> {
        final Throwable t;

        ComputationExceptionReference(Throwable t2) {
            this.t = t2;
        }

        public V get() {
            return null;
        }

        public MapMakerInternalMap.ReferenceEntry<K, V> getEntry() {
            return null;
        }

        public MapMakerInternalMap.ValueReference<K, V> copyFor(ReferenceQueue<V> referenceQueue, V v, MapMakerInternalMap.ReferenceEntry<K, V> referenceEntry) {
            return this;
        }

        public boolean isComputingReference() {
            return false;
        }

        public V waitForValue() throws ExecutionException {
            throw new ExecutionException(this.t);
        }

        public void clear(MapMakerInternalMap.ValueReference<K, V> valueReference) {
        }
    }

    private static final class ComputedReference<K, V> implements MapMakerInternalMap.ValueReference<K, V> {
        final V value;

        ComputedReference(@Nullable V value2) {
            this.value = value2;
        }

        public V get() {
            return this.value;
        }

        public MapMakerInternalMap.ReferenceEntry<K, V> getEntry() {
            return null;
        }

        public MapMakerInternalMap.ValueReference<K, V> copyFor(ReferenceQueue<V> referenceQueue, V v, MapMakerInternalMap.ReferenceEntry<K, V> referenceEntry) {
            return this;
        }

        public boolean isComputingReference() {
            return false;
        }

        public V waitForValue() {
            return get();
        }

        public void clear(MapMakerInternalMap.ValueReference<K, V> valueReference) {
        }
    }

    private static final class ComputingValueReference<K, V> implements MapMakerInternalMap.ValueReference<K, V> {
        @GuardedBy("ComputingValueReference.this")
        volatile MapMakerInternalMap.ValueReference<K, V> computedReference = MapMakerInternalMap.unset();
        final Function<? super K, ? extends V> computingFunction;

        public ComputingValueReference(Function<? super K, ? extends V> computingFunction2) {
            this.computingFunction = computingFunction2;
        }

        public V get() {
            return null;
        }

        public MapMakerInternalMap.ReferenceEntry<K, V> getEntry() {
            return null;
        }

        public MapMakerInternalMap.ValueReference<K, V> copyFor(ReferenceQueue<V> referenceQueue, V v, MapMakerInternalMap.ReferenceEntry<K, V> referenceEntry) {
            return this;
        }

        public boolean isComputingReference() {
            return true;
        }

        /* JADX WARNING: Code restructure failed: missing block: B:22:0x0024, code lost:
            r0 = th;
         */
        /* Code decompiled incorrectly, please refer to instructions dump. */
        public V waitForValue() throws java.util.concurrent.ExecutionException {
            /*
                r4 = this;
                com.google.common.collect.MapMakerInternalMap$ValueReference<K, V> r0 = r4.computedReference
                com.google.common.collect.MapMakerInternalMap$ValueReference<java.lang.Object, java.lang.Object> r1 = com.google.common.collect.MapMakerInternalMap.UNSET
                if (r0 != r1) goto L_0x0034
                r0 = 0
                monitor-enter(r4)     // Catch:{ all -> 0x0026 }
                r1 = r0
            L_0x0009:
                com.google.common.collect.MapMakerInternalMap$ValueReference<K, V> r0 = r4.computedReference     // Catch:{ all -> 0x0021 }
                com.google.common.collect.MapMakerInternalMap$ValueReference<java.lang.Object, java.lang.Object> r2 = com.google.common.collect.MapMakerInternalMap.UNSET     // Catch:{ all -> 0x0021 }
                if (r0 != r2) goto L_0x0016
                r4.wait()     // Catch:{ InterruptedException -> 0x0013 }
            L_0x0012:
                goto L_0x0009
            L_0x0013:
                r0 = move-exception
                r1 = 1
                goto L_0x0012
            L_0x0016:
                monitor-exit(r4)     // Catch:{ all -> 0x0021 }
                if (r1 == 0) goto L_0x0034
                java.lang.Thread r0 = java.lang.Thread.currentThread()
                r0.interrupt()
                goto L_0x0034
            L_0x0021:
                r0 = move-exception
                monitor-exit(r4)     // Catch:{ all -> 0x0021 }
                throw r0     // Catch:{ all -> 0x0024 }
            L_0x0024:
                r0 = move-exception
                goto L_0x002a
            L_0x0026:
                r1 = move-exception
                r3 = r1
                r1 = r0
                r0 = r3
            L_0x002a:
                if (r1 == 0) goto L_0x0033
                java.lang.Thread r2 = java.lang.Thread.currentThread()
                r2.interrupt()
            L_0x0033:
                throw r0
            L_0x0034:
                com.google.common.collect.MapMakerInternalMap$ValueReference<K, V> r0 = r4.computedReference
                java.lang.Object r0 = r0.waitForValue()
                return r0
            */
            throw new UnsupportedOperationException("Method not decompiled: com.google.common.collect.ComputingConcurrentHashMap.ComputingValueReference.waitForValue():java.lang.Object");
        }

        public void clear(MapMakerInternalMap.ValueReference<K, V> newValue) {
            setValueReference(newValue);
        }

        /* access modifiers changed from: package-private */
        public V compute(K key, int hash) throws ExecutionException {
            try {
                V value = this.computingFunction.apply(key);
                setValueReference(new ComputedReference(value));
                return value;
            } catch (Throwable t) {
                setValueReference(new ComputationExceptionReference(t));
                throw new ExecutionException(t);
            }
        }

        /* access modifiers changed from: package-private */
        public void setValueReference(MapMakerInternalMap.ValueReference<K, V> valueReference) {
            synchronized (this) {
                if (this.computedReference == MapMakerInternalMap.UNSET) {
                    this.computedReference = valueReference;
                    notifyAll();
                }
            }
        }
    }

    static final class ComputingMapAdapter<K, V> extends ComputingConcurrentHashMap<K, V> implements Serializable {
        private static final long serialVersionUID = 0;

        /* access modifiers changed from: package-private */
        public /* bridge */ /* synthetic */ MapMakerInternalMap.Segment segmentFor(int x0) {
            return ComputingConcurrentHashMap.super.segmentFor(x0);
        }

        ComputingMapAdapter(MapMaker mapMaker, Function<? super K, ? extends V> computingFunction) {
            super(mapMaker, computingFunction);
        }

        public V get(Object key) {
            try {
                V value = getOrCompute(key);
                if (value != null) {
                    return value;
                }
                throw new NullPointerException(this.computingFunction + " returned null for key " + key + ".");
            } catch (ExecutionException e) {
                Throwable cause = e.getCause();
                Throwables.propagateIfInstanceOf(cause, ComputationException.class);
                throw new ComputationException(cause);
            }
        }
    }

    /* access modifiers changed from: package-private */
    public Object writeReplace() {
        return new ComputingSerializationProxy(this.keyStrength, this.valueStrength, this.keyEquivalence, this.valueEquivalence, this.expireAfterWriteNanos, this.expireAfterAccessNanos, this.maximumSize, this.concurrencyLevel, this.removalListener, this, this.computingFunction);
    }

    static final class ComputingSerializationProxy<K, V> extends MapMakerInternalMap.AbstractSerializationProxy<K, V> {
        private static final long serialVersionUID = 4;
        final Function<? super K, ? extends V> computingFunction;

        ComputingSerializationProxy(MapMakerInternalMap.Strength keyStrength, MapMakerInternalMap.Strength valueStrength, Equivalence<Object> keyEquivalence, Equivalence<Object> valueEquivalence, long expireAfterWriteNanos, long expireAfterAccessNanos, int maximumSize, int concurrencyLevel, MapMaker.RemovalListener<? super K, ? super V> removalListener, ConcurrentMap<K, V> delegate, Function<? super K, ? extends V> computingFunction2) {
            super(keyStrength, valueStrength, keyEquivalence, valueEquivalence, expireAfterWriteNanos, expireAfterAccessNanos, maximumSize, concurrencyLevel, removalListener, delegate);
            this.computingFunction = computingFunction2;
        }

        private void writeObject(ObjectOutputStream out) throws IOException {
            out.defaultWriteObject();
            writeMapTo(out);
        }

        private void readObject(ObjectInputStream in) throws IOException, ClassNotFoundException {
            in.defaultReadObject();
            this.delegate = readMapMaker(in).makeComputingMap(this.computingFunction);
            readEntries(in);
        }

        /* access modifiers changed from: package-private */
        public Object readResolve() {
            return this.delegate;
        }
    }
}
