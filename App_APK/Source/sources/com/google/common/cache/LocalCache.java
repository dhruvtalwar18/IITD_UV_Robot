package com.google.common.cache;

import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Equivalence;
import com.google.common.base.Equivalences;
import com.google.common.base.Preconditions;
import com.google.common.base.Stopwatch;
import com.google.common.base.Ticker;
import com.google.common.cache.AbstractCache;
import com.google.common.cache.CacheBuilder;
import com.google.common.cache.CacheLoader;
import com.google.common.collect.AbstractSequentialIterator;
import com.google.common.collect.ImmutableMap;
import com.google.common.collect.Iterators;
import com.google.common.collect.Maps;
import com.google.common.collect.Sets;
import com.google.common.primitives.Ints;
import com.google.common.util.concurrent.ExecutionError;
import com.google.common.util.concurrent.Futures;
import com.google.common.util.concurrent.ListenableFuture;
import com.google.common.util.concurrent.ListeningExecutorService;
import com.google.common.util.concurrent.MoreExecutors;
import com.google.common.util.concurrent.SettableFuture;
import com.google.common.util.concurrent.UncheckedExecutionException;
import com.google.common.util.concurrent.Uninterruptibles;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.Serializable;
import java.lang.ref.Reference;
import java.lang.ref.ReferenceQueue;
import java.lang.ref.SoftReference;
import java.lang.ref.WeakReference;
import java.util.AbstractCollection;
import java.util.AbstractMap;
import java.util.AbstractQueue;
import java.util.AbstractSet;
import java.util.Collection;
import java.util.Iterator;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.Callable;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ConcurrentMap;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReferenceArray;
import java.util.concurrent.locks.ReentrantLock;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.annotation.Nullable;
import javax.annotation.concurrent.GuardedBy;

class LocalCache<K, V> extends AbstractMap<K, V> implements ConcurrentMap<K, V> {
    static final int CONTAINS_VALUE_RETRIES = 3;
    static final Queue<? extends Object> DISCARDING_QUEUE = new AbstractQueue<Object>() {
        public boolean offer(Object o) {
            return true;
        }

        public Object peek() {
            return null;
        }

        public Object poll() {
            return null;
        }

        public int size() {
            return 0;
        }

        public Iterator<Object> iterator() {
            return Iterators.emptyIterator();
        }
    };
    static final int DRAIN_MAX = 16;
    static final int DRAIN_THRESHOLD = 63;
    static final int MAXIMUM_CAPACITY = 1073741824;
    static final int MAX_SEGMENTS = 65536;
    static final ValueReference<Object, Object> UNSET = new ValueReference<Object, Object>() {
        public Object get() {
            return null;
        }

        public int getWeight() {
            return 0;
        }

        public ReferenceEntry<Object, Object> getEntry() {
            return null;
        }

        public ValueReference<Object, Object> copyFor(ReferenceQueue<Object> referenceQueue, Object value, ReferenceEntry<Object, Object> referenceEntry) {
            return this;
        }

        public boolean isLoading() {
            return false;
        }

        public boolean isActive() {
            return false;
        }

        public Object waitForValue() {
            return null;
        }

        public void notifyNewValue(Object newValue) {
        }
    };
    static final Logger logger = Logger.getLogger(LocalCache.class.getName());
    static final ListeningExecutorService sameThreadExecutor = MoreExecutors.sameThreadExecutor();
    final int concurrencyLevel;
    @Nullable
    final CacheLoader<? super K, V> defaultLoader;
    final EntryFactory entryFactory;
    Set<Map.Entry<K, V>> entrySet;
    final long expireAfterAccessNanos;
    final long expireAfterWriteNanos;
    final AbstractCache.StatsCounter globalStatsCounter;
    final Equivalence<Object> keyEquivalence;
    Set<K> keySet;
    final Strength keyStrength;
    final long maxWeight;
    final long refreshNanos;
    final RemovalListener<K, V> removalListener;
    final Queue<RemovalNotification<K, V>> removalNotificationQueue;
    final int segmentMask;
    final int segmentShift;
    final Segment<K, V>[] segments;
    final Ticker ticker;
    final Equivalence<Object> valueEquivalence;
    final Strength valueStrength;
    Collection<V> values;
    final Weigher<K, V> weigher;

    interface ReferenceEntry<K, V> {
        long getAccessTime();

        int getHash();

        @Nullable
        K getKey();

        @Nullable
        ReferenceEntry<K, V> getNext();

        ReferenceEntry<K, V> getNextInAccessQueue();

        ReferenceEntry<K, V> getNextInWriteQueue();

        ReferenceEntry<K, V> getPreviousInAccessQueue();

        ReferenceEntry<K, V> getPreviousInWriteQueue();

        ValueReference<K, V> getValueReference();

        long getWriteTime();

        void setAccessTime(long j);

        void setNextInAccessQueue(ReferenceEntry<K, V> referenceEntry);

        void setNextInWriteQueue(ReferenceEntry<K, V> referenceEntry);

        void setPreviousInAccessQueue(ReferenceEntry<K, V> referenceEntry);

        void setPreviousInWriteQueue(ReferenceEntry<K, V> referenceEntry);

        void setValueReference(ValueReference<K, V> valueReference);

        void setWriteTime(long j);
    }

    enum Strength {
        STRONG {
            /* access modifiers changed from: package-private */
            public <K, V> ValueReference<K, V> referenceValue(Segment<K, V> segment, ReferenceEntry<K, V> referenceEntry, V value, int weight) {
                return weight == 1 ? new StrongValueReference(value) : new WeightedStrongValueReference(value, weight);
            }

            /* access modifiers changed from: package-private */
            public Equivalence<Object> defaultEquivalence() {
                return Equivalences.equals();
            }
        },
        SOFT {
            /* access modifiers changed from: package-private */
            public <K, V> ValueReference<K, V> referenceValue(Segment<K, V> segment, ReferenceEntry<K, V> entry, V value, int weight) {
                return weight == 1 ? new SoftValueReference(segment.valueReferenceQueue, value, entry) : new WeightedSoftValueReference(segment.valueReferenceQueue, value, entry, weight);
            }

            /* access modifiers changed from: package-private */
            public Equivalence<Object> defaultEquivalence() {
                return Equivalences.identity();
            }
        },
        WEAK {
            /* access modifiers changed from: package-private */
            public <K, V> ValueReference<K, V> referenceValue(Segment<K, V> segment, ReferenceEntry<K, V> entry, V value, int weight) {
                return weight == 1 ? new WeakValueReference(segment.valueReferenceQueue, value, entry) : new WeightedWeakValueReference(segment.valueReferenceQueue, value, entry, weight);
            }

            /* access modifiers changed from: package-private */
            public Equivalence<Object> defaultEquivalence() {
                return Equivalences.identity();
            }
        };

        /* access modifiers changed from: package-private */
        public abstract Equivalence<Object> defaultEquivalence();

        /* access modifiers changed from: package-private */
        public abstract <K, V> ValueReference<K, V> referenceValue(Segment<K, V> segment, ReferenceEntry<K, V> referenceEntry, V v, int i);
    }

    interface ValueReference<K, V> {
        ValueReference<K, V> copyFor(ReferenceQueue<V> referenceQueue, V v, ReferenceEntry<K, V> referenceEntry);

        @Nullable
        V get();

        @Nullable
        ReferenceEntry<K, V> getEntry();

        int getWeight();

        boolean isActive();

        boolean isLoading();

        void notifyNewValue(@Nullable V v);

        V waitForValue() throws ExecutionException;
    }

    LocalCache(CacheBuilder<? super K, ? super V> builder, @Nullable CacheLoader<? super K, V> loader) {
        this.concurrencyLevel = Math.min(builder.getConcurrencyLevel(), 65536);
        this.keyStrength = builder.getKeyStrength();
        this.valueStrength = builder.getValueStrength();
        this.keyEquivalence = builder.getKeyEquivalence();
        this.valueEquivalence = builder.getValueEquivalence();
        this.maxWeight = builder.getMaximumWeight();
        this.weigher = builder.getWeigher();
        this.expireAfterAccessNanos = builder.getExpireAfterAccessNanos();
        this.expireAfterWriteNanos = builder.getExpireAfterWriteNanos();
        this.refreshNanos = builder.getRefreshNanos();
        this.removalListener = builder.getRemovalListener();
        this.removalNotificationQueue = this.removalListener == CacheBuilder.NullListener.INSTANCE ? discardingQueue() : new ConcurrentLinkedQueue<>();
        this.ticker = builder.getTicker(recordsTime());
        this.entryFactory = EntryFactory.getFactory(this.keyStrength, usesAccessEntries(), usesWriteEntries());
        this.globalStatsCounter = (AbstractCache.StatsCounter) builder.getStatsCounterSupplier().get();
        this.defaultLoader = loader;
        int initialCapacity = Math.min(builder.getInitialCapacity(), 1073741824);
        if (evictsBySize() && !customWeigher()) {
            initialCapacity = Math.min(initialCapacity, (int) this.maxWeight);
        }
        int segmentSize = 1;
        int segmentShift2 = 0;
        int segmentCount = 1;
        while (segmentCount < this.concurrencyLevel && (!evictsBySize() || ((long) (segmentCount * 20)) <= this.maxWeight)) {
            segmentShift2++;
            segmentCount <<= 1;
        }
        this.segmentShift = 32 - segmentShift2;
        this.segmentMask = segmentCount - 1;
        this.segments = newSegmentArray(segmentCount);
        int segmentCapacity = initialCapacity / segmentCount;
        while (segmentSize < (segmentCapacity * segmentCount < initialCapacity ? segmentCapacity + 1 : segmentCapacity)) {
            segmentSize <<= 1;
        }
        if (evictsBySize()) {
            long j = 1;
            long maxSegmentWeight = (this.maxWeight / ((long) segmentCount)) + 1;
            long remainder = this.maxWeight % ((long) segmentCount);
            int i = 0;
            while (true) {
                int i2 = i;
                if (i2 < this.segments.length) {
                    if (((long) i2) == remainder) {
                        maxSegmentWeight -= j;
                    }
                    long maxSegmentWeight2 = maxSegmentWeight;
                    this.segments[i2] = createSegment(segmentSize, maxSegmentWeight2, (AbstractCache.StatsCounter) builder.getStatsCounterSupplier().get());
                    i = i2 + 1;
                    maxSegmentWeight = maxSegmentWeight2;
                    j = 1;
                } else {
                    return;
                }
            }
        } else {
            int i3 = 0;
            while (true) {
                int i4 = i3;
                if (i4 < this.segments.length) {
                    this.segments[i4] = createSegment(segmentSize, -1, (AbstractCache.StatsCounter) builder.getStatsCounterSupplier().get());
                    i3 = i4 + 1;
                } else {
                    return;
                }
            }
        }
    }

    /* access modifiers changed from: package-private */
    public boolean evictsBySize() {
        return this.maxWeight >= 0;
    }

    /* access modifiers changed from: package-private */
    public boolean customWeigher() {
        return this.weigher != CacheBuilder.OneWeigher.INSTANCE;
    }

    /* access modifiers changed from: package-private */
    public boolean expires() {
        return expiresAfterWrite() || expiresAfterAccess();
    }

    /* access modifiers changed from: package-private */
    public boolean expiresAfterWrite() {
        return this.expireAfterWriteNanos > 0;
    }

    /* access modifiers changed from: package-private */
    public boolean expiresAfterAccess() {
        return this.expireAfterAccessNanos > 0;
    }

    /* access modifiers changed from: package-private */
    public boolean refreshes() {
        return this.refreshNanos > 0;
    }

    /* access modifiers changed from: package-private */
    public boolean usesAccessQueue() {
        return expiresAfterAccess() || evictsBySize();
    }

    /* access modifiers changed from: package-private */
    public boolean usesWriteQueue() {
        return expiresAfterWrite();
    }

    /* access modifiers changed from: package-private */
    public boolean recordsWrite() {
        return expiresAfterWrite() || refreshes();
    }

    /* access modifiers changed from: package-private */
    public boolean recordsAccess() {
        return expiresAfterAccess();
    }

    /* access modifiers changed from: package-private */
    public boolean recordsTime() {
        return recordsWrite() || recordsAccess();
    }

    /* access modifiers changed from: package-private */
    public boolean usesWriteEntries() {
        return usesWriteQueue() || recordsWrite();
    }

    /* access modifiers changed from: package-private */
    public boolean usesAccessEntries() {
        return usesAccessQueue() || recordsAccess();
    }

    /* access modifiers changed from: package-private */
    public boolean usesKeyReferences() {
        return this.keyStrength != Strength.STRONG;
    }

    /* access modifiers changed from: package-private */
    public boolean usesValueReferences() {
        return this.valueStrength != Strength.STRONG;
    }

    enum EntryFactory {
        STRONG {
            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> newEntry(Segment<K, V> segment, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
                return new StrongEntry(key, hash, next);
            }
        },
        STRONG_ACCESS {
            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> newEntry(Segment<K, V> segment, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
                return new StrongAccessEntry(key, hash, next);
            }

            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> copyEntry(Segment<K, V> segment, ReferenceEntry<K, V> original, ReferenceEntry<K, V> newNext) {
                ReferenceEntry<K, V> newEntry = super.copyEntry(segment, original, newNext);
                copyAccessEntry(original, newEntry);
                return newEntry;
            }
        },
        STRONG_WRITE {
            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> newEntry(Segment<K, V> segment, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
                return new StrongWriteEntry(key, hash, next);
            }

            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> copyEntry(Segment<K, V> segment, ReferenceEntry<K, V> original, ReferenceEntry<K, V> newNext) {
                ReferenceEntry<K, V> newEntry = super.copyEntry(segment, original, newNext);
                copyWriteEntry(original, newEntry);
                return newEntry;
            }
        },
        STRONG_ACCESS_WRITE {
            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> newEntry(Segment<K, V> segment, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
                return new StrongAccessWriteEntry(key, hash, next);
            }

            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> copyEntry(Segment<K, V> segment, ReferenceEntry<K, V> original, ReferenceEntry<K, V> newNext) {
                ReferenceEntry<K, V> newEntry = super.copyEntry(segment, original, newNext);
                copyAccessEntry(original, newEntry);
                copyWriteEntry(original, newEntry);
                return newEntry;
            }
        },
        WEAK {
            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> newEntry(Segment<K, V> segment, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
                return new WeakEntry(segment.keyReferenceQueue, key, hash, next);
            }
        },
        WEAK_ACCESS {
            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> newEntry(Segment<K, V> segment, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
                return new WeakAccessEntry(segment.keyReferenceQueue, key, hash, next);
            }

            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> copyEntry(Segment<K, V> segment, ReferenceEntry<K, V> original, ReferenceEntry<K, V> newNext) {
                ReferenceEntry<K, V> newEntry = super.copyEntry(segment, original, newNext);
                copyAccessEntry(original, newEntry);
                return newEntry;
            }
        },
        WEAK_WRITE {
            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> newEntry(Segment<K, V> segment, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
                return new WeakWriteEntry(segment.keyReferenceQueue, key, hash, next);
            }

            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> copyEntry(Segment<K, V> segment, ReferenceEntry<K, V> original, ReferenceEntry<K, V> newNext) {
                ReferenceEntry<K, V> newEntry = super.copyEntry(segment, original, newNext);
                copyWriteEntry(original, newEntry);
                return newEntry;
            }
        },
        WEAK_ACCESS_WRITE {
            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> newEntry(Segment<K, V> segment, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
                return new WeakAccessWriteEntry(segment.keyReferenceQueue, key, hash, next);
            }

            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> copyEntry(Segment<K, V> segment, ReferenceEntry<K, V> original, ReferenceEntry<K, V> newNext) {
                ReferenceEntry<K, V> newEntry = super.copyEntry(segment, original, newNext);
                copyAccessEntry(original, newEntry);
                copyWriteEntry(original, newEntry);
                return newEntry;
            }
        };
        
        static final int ACCESS_MASK = 1;
        static final int WEAK_MASK = 4;
        static final int WRITE_MASK = 2;
        static final EntryFactory[] factories = null;

        /* access modifiers changed from: package-private */
        public abstract <K, V> ReferenceEntry<K, V> newEntry(Segment<K, V> segment, K k, int i, @Nullable ReferenceEntry<K, V> referenceEntry);

        static {
            factories = new EntryFactory[]{STRONG, STRONG_ACCESS, STRONG_WRITE, STRONG_ACCESS_WRITE, WEAK, WEAK_ACCESS, WEAK_WRITE, WEAK_ACCESS_WRITE};
        }

        static EntryFactory getFactory(Strength keyStrength, boolean usesAccessQueue, boolean usesWriteQueue) {
            int i = 0;
            int i2 = (keyStrength == Strength.WEAK ? 4 : 0) | usesAccessQueue;
            if (usesWriteQueue) {
                i = 2;
            }
            return factories[i2 | i];
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public <K, V> ReferenceEntry<K, V> copyEntry(Segment<K, V> segment, ReferenceEntry<K, V> original, ReferenceEntry<K, V> newNext) {
            return newEntry(segment, original.getKey(), original.getHash(), newNext);
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public <K, V> void copyAccessEntry(ReferenceEntry<K, V> original, ReferenceEntry<K, V> newEntry) {
            newEntry.setAccessTime(original.getAccessTime());
            LocalCache.connectAccessOrder(original.getPreviousInAccessQueue(), newEntry);
            LocalCache.connectAccessOrder(newEntry, original.getNextInAccessQueue());
            LocalCache.nullifyAccessOrder(original);
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public <K, V> void copyWriteEntry(ReferenceEntry<K, V> original, ReferenceEntry<K, V> newEntry) {
            newEntry.setWriteTime(original.getWriteTime());
            LocalCache.connectWriteOrder(original.getPreviousInWriteQueue(), newEntry);
            LocalCache.connectWriteOrder(newEntry, original.getNextInWriteQueue());
            LocalCache.nullifyWriteOrder(original);
        }
    }

    static <K, V> ValueReference<K, V> unset() {
        return UNSET;
    }

    private enum NullEntry implements ReferenceEntry<Object, Object> {
        INSTANCE;

        public ValueReference<Object, Object> getValueReference() {
            return null;
        }

        public void setValueReference(ValueReference<Object, Object> valueReference) {
        }

        public ReferenceEntry<Object, Object> getNext() {
            return null;
        }

        public int getHash() {
            return 0;
        }

        public Object getKey() {
            return null;
        }

        public long getAccessTime() {
            return 0;
        }

        public void setAccessTime(long time) {
        }

        public ReferenceEntry<Object, Object> getNextInAccessQueue() {
            return this;
        }

        public void setNextInAccessQueue(ReferenceEntry<Object, Object> referenceEntry) {
        }

        public ReferenceEntry<Object, Object> getPreviousInAccessQueue() {
            return this;
        }

        public void setPreviousInAccessQueue(ReferenceEntry<Object, Object> referenceEntry) {
        }

        public long getWriteTime() {
            return 0;
        }

        public void setWriteTime(long time) {
        }

        public ReferenceEntry<Object, Object> getNextInWriteQueue() {
            return this;
        }

        public void setNextInWriteQueue(ReferenceEntry<Object, Object> referenceEntry) {
        }

        public ReferenceEntry<Object, Object> getPreviousInWriteQueue() {
            return this;
        }

        public void setPreviousInWriteQueue(ReferenceEntry<Object, Object> referenceEntry) {
        }
    }

    static abstract class AbstractReferenceEntry<K, V> implements ReferenceEntry<K, V> {
        AbstractReferenceEntry() {
        }

        public ValueReference<K, V> getValueReference() {
            throw new UnsupportedOperationException();
        }

        public void setValueReference(ValueReference<K, V> valueReference) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getNext() {
            throw new UnsupportedOperationException();
        }

        public int getHash() {
            throw new UnsupportedOperationException();
        }

        public K getKey() {
            throw new UnsupportedOperationException();
        }

        public long getAccessTime() {
            throw new UnsupportedOperationException();
        }

        public void setAccessTime(long time) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getNextInAccessQueue() {
            throw new UnsupportedOperationException();
        }

        public void setNextInAccessQueue(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getPreviousInAccessQueue() {
            throw new UnsupportedOperationException();
        }

        public void setPreviousInAccessQueue(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public long getWriteTime() {
            throw new UnsupportedOperationException();
        }

        public void setWriteTime(long time) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getNextInWriteQueue() {
            throw new UnsupportedOperationException();
        }

        public void setNextInWriteQueue(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getPreviousInWriteQueue() {
            throw new UnsupportedOperationException();
        }

        public void setPreviousInWriteQueue(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }
    }

    static <K, V> ReferenceEntry<K, V> nullEntry() {
        return NullEntry.INSTANCE;
    }

    static <E> Queue<E> discardingQueue() {
        return DISCARDING_QUEUE;
    }

    static class StrongEntry<K, V> implements ReferenceEntry<K, V> {
        final int hash;
        final K key;
        final ReferenceEntry<K, V> next;
        volatile ValueReference<K, V> valueReference = LocalCache.unset();

        StrongEntry(K key2, int hash2, @Nullable ReferenceEntry<K, V> next2) {
            this.key = key2;
            this.hash = hash2;
            this.next = next2;
        }

        public K getKey() {
            return this.key;
        }

        public long getAccessTime() {
            throw new UnsupportedOperationException();
        }

        public void setAccessTime(long time) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getNextInAccessQueue() {
            throw new UnsupportedOperationException();
        }

        public void setNextInAccessQueue(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getPreviousInAccessQueue() {
            throw new UnsupportedOperationException();
        }

        public void setPreviousInAccessQueue(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public long getWriteTime() {
            throw new UnsupportedOperationException();
        }

        public void setWriteTime(long time) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getNextInWriteQueue() {
            throw new UnsupportedOperationException();
        }

        public void setNextInWriteQueue(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getPreviousInWriteQueue() {
            throw new UnsupportedOperationException();
        }

        public void setPreviousInWriteQueue(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public ValueReference<K, V> getValueReference() {
            return this.valueReference;
        }

        public void setValueReference(ValueReference<K, V> valueReference2) {
            this.valueReference = valueReference2;
        }

        public int getHash() {
            return this.hash;
        }

        public ReferenceEntry<K, V> getNext() {
            return this.next;
        }
    }

    static final class StrongAccessEntry<K, V> extends StrongEntry<K, V> implements ReferenceEntry<K, V> {
        volatile long accessTime = Long.MAX_VALUE;
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> nextAccess = LocalCache.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> previousAccess = LocalCache.nullEntry();

        StrongAccessEntry(K key, int hash, @Nullable ReferenceEntry<K, V> next) {
            super(key, hash, next);
        }

        public long getAccessTime() {
            return this.accessTime;
        }

        public void setAccessTime(long time) {
            this.accessTime = time;
        }

        public ReferenceEntry<K, V> getNextInAccessQueue() {
            return this.nextAccess;
        }

        public void setNextInAccessQueue(ReferenceEntry<K, V> next) {
            this.nextAccess = next;
        }

        public ReferenceEntry<K, V> getPreviousInAccessQueue() {
            return this.previousAccess;
        }

        public void setPreviousInAccessQueue(ReferenceEntry<K, V> previous) {
            this.previousAccess = previous;
        }
    }

    static final class StrongWriteEntry<K, V> extends StrongEntry<K, V> implements ReferenceEntry<K, V> {
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> nextWrite = LocalCache.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> previousWrite = LocalCache.nullEntry();
        volatile long writeTime = Long.MAX_VALUE;

        StrongWriteEntry(K key, int hash, @Nullable ReferenceEntry<K, V> next) {
            super(key, hash, next);
        }

        public long getWriteTime() {
            return this.writeTime;
        }

        public void setWriteTime(long time) {
            this.writeTime = time;
        }

        public ReferenceEntry<K, V> getNextInWriteQueue() {
            return this.nextWrite;
        }

        public void setNextInWriteQueue(ReferenceEntry<K, V> next) {
            this.nextWrite = next;
        }

        public ReferenceEntry<K, V> getPreviousInWriteQueue() {
            return this.previousWrite;
        }

        public void setPreviousInWriteQueue(ReferenceEntry<K, V> previous) {
            this.previousWrite = previous;
        }
    }

    static final class StrongAccessWriteEntry<K, V> extends StrongEntry<K, V> implements ReferenceEntry<K, V> {
        volatile long accessTime = Long.MAX_VALUE;
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> nextAccess = LocalCache.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> nextWrite = LocalCache.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> previousAccess = LocalCache.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> previousWrite = LocalCache.nullEntry();
        volatile long writeTime = Long.MAX_VALUE;

        StrongAccessWriteEntry(K key, int hash, @Nullable ReferenceEntry<K, V> next) {
            super(key, hash, next);
        }

        public long getAccessTime() {
            return this.accessTime;
        }

        public void setAccessTime(long time) {
            this.accessTime = time;
        }

        public ReferenceEntry<K, V> getNextInAccessQueue() {
            return this.nextAccess;
        }

        public void setNextInAccessQueue(ReferenceEntry<K, V> next) {
            this.nextAccess = next;
        }

        public ReferenceEntry<K, V> getPreviousInAccessQueue() {
            return this.previousAccess;
        }

        public void setPreviousInAccessQueue(ReferenceEntry<K, V> previous) {
            this.previousAccess = previous;
        }

        public long getWriteTime() {
            return this.writeTime;
        }

        public void setWriteTime(long time) {
            this.writeTime = time;
        }

        public ReferenceEntry<K, V> getNextInWriteQueue() {
            return this.nextWrite;
        }

        public void setNextInWriteQueue(ReferenceEntry<K, V> next) {
            this.nextWrite = next;
        }

        public ReferenceEntry<K, V> getPreviousInWriteQueue() {
            return this.previousWrite;
        }

        public void setPreviousInWriteQueue(ReferenceEntry<K, V> previous) {
            this.previousWrite = previous;
        }
    }

    static class WeakEntry<K, V> extends WeakReference<K> implements ReferenceEntry<K, V> {
        final int hash;
        final ReferenceEntry<K, V> next;
        volatile ValueReference<K, V> valueReference = LocalCache.unset();

        WeakEntry(ReferenceQueue<K> queue, K key, int hash2, @Nullable ReferenceEntry<K, V> next2) {
            super(key, queue);
            this.hash = hash2;
            this.next = next2;
        }

        public K getKey() {
            return get();
        }

        public long getAccessTime() {
            throw new UnsupportedOperationException();
        }

        public void setAccessTime(long time) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getNextInAccessQueue() {
            throw new UnsupportedOperationException();
        }

        public void setNextInAccessQueue(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getPreviousInAccessQueue() {
            throw new UnsupportedOperationException();
        }

        public void setPreviousInAccessQueue(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public long getWriteTime() {
            throw new UnsupportedOperationException();
        }

        public void setWriteTime(long time) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getNextInWriteQueue() {
            throw new UnsupportedOperationException();
        }

        public void setNextInWriteQueue(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getPreviousInWriteQueue() {
            throw new UnsupportedOperationException();
        }

        public void setPreviousInWriteQueue(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public ValueReference<K, V> getValueReference() {
            return this.valueReference;
        }

        public void setValueReference(ValueReference<K, V> valueReference2) {
            this.valueReference = valueReference2;
        }

        public int getHash() {
            return this.hash;
        }

        public ReferenceEntry<K, V> getNext() {
            return this.next;
        }
    }

    static final class WeakAccessEntry<K, V> extends WeakEntry<K, V> implements ReferenceEntry<K, V> {
        volatile long accessTime = Long.MAX_VALUE;
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> nextAccess = LocalCache.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> previousAccess = LocalCache.nullEntry();

        WeakAccessEntry(ReferenceQueue<K> queue, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
            super(queue, key, hash, next);
        }

        public long getAccessTime() {
            return this.accessTime;
        }

        public void setAccessTime(long time) {
            this.accessTime = time;
        }

        public ReferenceEntry<K, V> getNextInAccessQueue() {
            return this.nextAccess;
        }

        public void setNextInAccessQueue(ReferenceEntry<K, V> next) {
            this.nextAccess = next;
        }

        public ReferenceEntry<K, V> getPreviousInAccessQueue() {
            return this.previousAccess;
        }

        public void setPreviousInAccessQueue(ReferenceEntry<K, V> previous) {
            this.previousAccess = previous;
        }
    }

    static final class WeakWriteEntry<K, V> extends WeakEntry<K, V> implements ReferenceEntry<K, V> {
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> nextWrite = LocalCache.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> previousWrite = LocalCache.nullEntry();
        volatile long writeTime = Long.MAX_VALUE;

        WeakWriteEntry(ReferenceQueue<K> queue, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
            super(queue, key, hash, next);
        }

        public long getWriteTime() {
            return this.writeTime;
        }

        public void setWriteTime(long time) {
            this.writeTime = time;
        }

        public ReferenceEntry<K, V> getNextInWriteQueue() {
            return this.nextWrite;
        }

        public void setNextInWriteQueue(ReferenceEntry<K, V> next) {
            this.nextWrite = next;
        }

        public ReferenceEntry<K, V> getPreviousInWriteQueue() {
            return this.previousWrite;
        }

        public void setPreviousInWriteQueue(ReferenceEntry<K, V> previous) {
            this.previousWrite = previous;
        }
    }

    static final class WeakAccessWriteEntry<K, V> extends WeakEntry<K, V> implements ReferenceEntry<K, V> {
        volatile long accessTime = Long.MAX_VALUE;
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> nextAccess = LocalCache.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> nextWrite = LocalCache.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> previousAccess = LocalCache.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> previousWrite = LocalCache.nullEntry();
        volatile long writeTime = Long.MAX_VALUE;

        WeakAccessWriteEntry(ReferenceQueue<K> queue, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
            super(queue, key, hash, next);
        }

        public long getAccessTime() {
            return this.accessTime;
        }

        public void setAccessTime(long time) {
            this.accessTime = time;
        }

        public ReferenceEntry<K, V> getNextInAccessQueue() {
            return this.nextAccess;
        }

        public void setNextInAccessQueue(ReferenceEntry<K, V> next) {
            this.nextAccess = next;
        }

        public ReferenceEntry<K, V> getPreviousInAccessQueue() {
            return this.previousAccess;
        }

        public void setPreviousInAccessQueue(ReferenceEntry<K, V> previous) {
            this.previousAccess = previous;
        }

        public long getWriteTime() {
            return this.writeTime;
        }

        public void setWriteTime(long time) {
            this.writeTime = time;
        }

        public ReferenceEntry<K, V> getNextInWriteQueue() {
            return this.nextWrite;
        }

        public void setNextInWriteQueue(ReferenceEntry<K, V> next) {
            this.nextWrite = next;
        }

        public ReferenceEntry<K, V> getPreviousInWriteQueue() {
            return this.previousWrite;
        }

        public void setPreviousInWriteQueue(ReferenceEntry<K, V> previous) {
            this.previousWrite = previous;
        }
    }

    static class WeakValueReference<K, V> extends WeakReference<V> implements ValueReference<K, V> {
        final ReferenceEntry<K, V> entry;

        WeakValueReference(ReferenceQueue<V> queue, V referent, ReferenceEntry<K, V> entry2) {
            super(referent, queue);
            this.entry = entry2;
        }

        public int getWeight() {
            return 1;
        }

        public ReferenceEntry<K, V> getEntry() {
            return this.entry;
        }

        public void notifyNewValue(V v) {
        }

        public ValueReference<K, V> copyFor(ReferenceQueue<V> queue, V value, ReferenceEntry<K, V> entry2) {
            return new WeakValueReference(queue, value, entry2);
        }

        public boolean isLoading() {
            return false;
        }

        public boolean isActive() {
            return true;
        }

        public V waitForValue() {
            return get();
        }
    }

    static class SoftValueReference<K, V> extends SoftReference<V> implements ValueReference<K, V> {
        final ReferenceEntry<K, V> entry;

        SoftValueReference(ReferenceQueue<V> queue, V referent, ReferenceEntry<K, V> entry2) {
            super(referent, queue);
            this.entry = entry2;
        }

        public int getWeight() {
            return 1;
        }

        public ReferenceEntry<K, V> getEntry() {
            return this.entry;
        }

        public void notifyNewValue(V v) {
        }

        public ValueReference<K, V> copyFor(ReferenceQueue<V> queue, V value, ReferenceEntry<K, V> entry2) {
            return new SoftValueReference(queue, value, entry2);
        }

        public boolean isLoading() {
            return false;
        }

        public boolean isActive() {
            return true;
        }

        public V waitForValue() {
            return get();
        }
    }

    static class StrongValueReference<K, V> implements ValueReference<K, V> {
        final V referent;

        StrongValueReference(V referent2) {
            this.referent = referent2;
        }

        public V get() {
            return this.referent;
        }

        public int getWeight() {
            return 1;
        }

        public ReferenceEntry<K, V> getEntry() {
            return null;
        }

        public ValueReference<K, V> copyFor(ReferenceQueue<V> referenceQueue, V v, ReferenceEntry<K, V> referenceEntry) {
            return this;
        }

        public boolean isLoading() {
            return false;
        }

        public boolean isActive() {
            return true;
        }

        public V waitForValue() {
            return get();
        }

        public void notifyNewValue(V v) {
        }
    }

    static final class WeightedWeakValueReference<K, V> extends WeakValueReference<K, V> {
        final int weight;

        WeightedWeakValueReference(ReferenceQueue<V> queue, V referent, ReferenceEntry<K, V> entry, int weight2) {
            super(queue, referent, entry);
            this.weight = weight2;
        }

        public int getWeight() {
            return this.weight;
        }

        public ValueReference<K, V> copyFor(ReferenceQueue<V> queue, V value, ReferenceEntry<K, V> entry) {
            return new WeightedWeakValueReference(queue, value, entry, this.weight);
        }
    }

    static final class WeightedSoftValueReference<K, V> extends SoftValueReference<K, V> {
        final int weight;

        WeightedSoftValueReference(ReferenceQueue<V> queue, V referent, ReferenceEntry<K, V> entry, int weight2) {
            super(queue, referent, entry);
            this.weight = weight2;
        }

        public int getWeight() {
            return this.weight;
        }

        public ValueReference<K, V> copyFor(ReferenceQueue<V> queue, V value, ReferenceEntry<K, V> entry) {
            return new WeightedSoftValueReference(queue, value, entry, this.weight);
        }
    }

    static final class WeightedStrongValueReference<K, V> extends StrongValueReference<K, V> {
        final int weight;

        WeightedStrongValueReference(V referent, int weight2) {
            super(referent);
            this.weight = weight2;
        }

        public int getWeight() {
            return this.weight;
        }
    }

    static int rehash(int h) {
        int h2 = h + ((h << 15) ^ -12931);
        int h3 = h2 ^ (h2 >>> 10);
        int h4 = h3 + (h3 << 3);
        int h5 = h4 ^ (h4 >>> 6);
        int h6 = h5 + (h5 << 2) + (h5 << 14);
        return (h6 >>> 16) ^ h6;
    }

    /* access modifiers changed from: package-private */
    @GuardedBy("Segment.this")
    @VisibleForTesting
    public ReferenceEntry<K, V> newEntry(K key, int hash, @Nullable ReferenceEntry<K, V> next) {
        return segmentFor(hash).newEntry(key, hash, next);
    }

    /* access modifiers changed from: package-private */
    @GuardedBy("Segment.this")
    @VisibleForTesting
    public ReferenceEntry<K, V> copyEntry(ReferenceEntry<K, V> original, ReferenceEntry<K, V> newNext) {
        return segmentFor(original.getHash()).copyEntry(original, newNext);
    }

    /* access modifiers changed from: package-private */
    @GuardedBy("Segment.this")
    @VisibleForTesting
    public ValueReference<K, V> newValueReference(ReferenceEntry<K, V> entry, V value, int weight) {
        return this.valueStrength.referenceValue(segmentFor(entry.getHash()), entry, value, weight);
    }

    /* access modifiers changed from: package-private */
    public int hash(Object key) {
        return rehash(this.keyEquivalence.hash(key));
    }

    /* access modifiers changed from: package-private */
    public void reclaimValue(ValueReference<K, V> valueReference) {
        ReferenceEntry<K, V> entry = valueReference.getEntry();
        int hash = entry.getHash();
        segmentFor(hash).reclaimValue(entry.getKey(), hash, valueReference);
    }

    /* access modifiers changed from: package-private */
    public void reclaimKey(ReferenceEntry<K, V> entry) {
        int hash = entry.getHash();
        segmentFor(hash).reclaimKey(entry, hash);
    }

    /* access modifiers changed from: package-private */
    @VisibleForTesting
    public boolean isLive(ReferenceEntry<K, V> entry, long now) {
        return segmentFor(entry.getHash()).getLiveValue(entry, now) != null;
    }

    /* access modifiers changed from: package-private */
    public Segment<K, V> segmentFor(int hash) {
        return this.segments[(hash >>> this.segmentShift) & this.segmentMask];
    }

    /* access modifiers changed from: package-private */
    public Segment<K, V> createSegment(int initialCapacity, long maxSegmentWeight, AbstractCache.StatsCounter statsCounter) {
        return new Segment(this, initialCapacity, maxSegmentWeight, statsCounter);
    }

    /* access modifiers changed from: package-private */
    @Nullable
    public V getLiveValue(ReferenceEntry<K, V> entry, long now) {
        V value;
        if (entry.getKey() == null || (value = entry.getValueReference().get()) == null || isExpired(entry, now)) {
            return null;
        }
        return value;
    }

    /* access modifiers changed from: package-private */
    public boolean isExpired(ReferenceEntry<K, V> entry, long now) {
        if (expiresAfterAccess() && now - entry.getAccessTime() > this.expireAfterAccessNanos) {
            return true;
        }
        if (!expiresAfterWrite() || now - entry.getWriteTime() <= this.expireAfterWriteNanos) {
            return false;
        }
        return true;
    }

    @GuardedBy("Segment.this")
    static <K, V> void connectAccessOrder(ReferenceEntry<K, V> previous, ReferenceEntry<K, V> next) {
        previous.setNextInAccessQueue(next);
        next.setPreviousInAccessQueue(previous);
    }

    @GuardedBy("Segment.this")
    static <K, V> void nullifyAccessOrder(ReferenceEntry<K, V> nulled) {
        ReferenceEntry<K, V> nullEntry = nullEntry();
        nulled.setNextInAccessQueue(nullEntry);
        nulled.setPreviousInAccessQueue(nullEntry);
    }

    @GuardedBy("Segment.this")
    static <K, V> void connectWriteOrder(ReferenceEntry<K, V> previous, ReferenceEntry<K, V> next) {
        previous.setNextInWriteQueue(next);
        next.setPreviousInWriteQueue(previous);
    }

    @GuardedBy("Segment.this")
    static <K, V> void nullifyWriteOrder(ReferenceEntry<K, V> nulled) {
        ReferenceEntry<K, V> nullEntry = nullEntry();
        nulled.setNextInWriteQueue(nullEntry);
        nulled.setPreviousInWriteQueue(nullEntry);
    }

    /* access modifiers changed from: package-private */
    public void processPendingNotifications() {
        while (true) {
            RemovalNotification<K, V> poll = this.removalNotificationQueue.poll();
            RemovalNotification<K, V> notification = poll;
            if (poll != null) {
                try {
                    this.removalListener.onRemoval(notification);
                } catch (Throwable e) {
                    logger.log(Level.WARNING, "Exception thrown by removal listener", e);
                }
            } else {
                return;
            }
        }
    }

    /* access modifiers changed from: package-private */
    public final Segment<K, V>[] newSegmentArray(int ssize) {
        return new Segment[ssize];
    }

    static class Segment<K, V> extends ReentrantLock {
        @GuardedBy("Segment.this")
        final Queue<ReferenceEntry<K, V>> accessQueue;
        volatile int count;
        final ReferenceQueue<K> keyReferenceQueue;
        final LocalCache<K, V> map;
        final long maxSegmentWeight;
        int modCount;
        final AtomicInteger readCount = new AtomicInteger();
        final Queue<ReferenceEntry<K, V>> recencyQueue;
        final AbstractCache.StatsCounter statsCounter;
        volatile AtomicReferenceArray<ReferenceEntry<K, V>> table;
        int threshold;
        @GuardedBy("Segment.this")
        int totalWeight;
        final ReferenceQueue<V> valueReferenceQueue;
        @GuardedBy("Segment.this")
        final Queue<ReferenceEntry<K, V>> writeQueue;

        Segment(LocalCache<K, V> map2, int initialCapacity, long maxSegmentWeight2, AbstractCache.StatsCounter statsCounter2) {
            this.map = map2;
            this.maxSegmentWeight = maxSegmentWeight2;
            this.statsCounter = statsCounter2;
            initTable(newEntryArray(initialCapacity));
            ReferenceQueue<V> referenceQueue = null;
            this.keyReferenceQueue = map2.usesKeyReferences() ? new ReferenceQueue<>() : null;
            this.valueReferenceQueue = map2.usesValueReferences() ? new ReferenceQueue<>() : referenceQueue;
            this.recencyQueue = map2.usesAccessQueue() ? new ConcurrentLinkedQueue<>() : LocalCache.discardingQueue();
            this.writeQueue = map2.usesWriteQueue() ? new WriteQueue<>() : LocalCache.discardingQueue();
            this.accessQueue = map2.usesAccessQueue() ? new AccessQueue<>() : LocalCache.discardingQueue();
        }

        /* access modifiers changed from: package-private */
        public AtomicReferenceArray<ReferenceEntry<K, V>> newEntryArray(int size) {
            return new AtomicReferenceArray<>(size);
        }

        /* access modifiers changed from: package-private */
        public void initTable(AtomicReferenceArray<ReferenceEntry<K, V>> newTable) {
            this.threshold = (newTable.length() * 3) / 4;
            if (!this.map.customWeigher() && ((long) this.threshold) == this.maxSegmentWeight) {
                this.threshold++;
            }
            this.table = newTable;
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public ReferenceEntry<K, V> newEntry(K key, int hash, @Nullable ReferenceEntry<K, V> next) {
            return this.map.entryFactory.newEntry(this, key, hash, next);
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public ReferenceEntry<K, V> copyEntry(ReferenceEntry<K, V> original, ReferenceEntry<K, V> newNext) {
            if (original.getKey() == null) {
                return null;
            }
            ValueReference<K, V> valueReference = original.getValueReference();
            V value = valueReference.get();
            if (value == null && valueReference.isActive()) {
                return null;
            }
            ReferenceEntry<K, V> newEntry = this.map.entryFactory.copyEntry(this, original, newNext);
            newEntry.setValueReference(valueReference.copyFor(this.valueReferenceQueue, value, newEntry));
            return newEntry;
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public void setValue(ReferenceEntry<K, V> entry, K key, V value, long now) {
            ValueReference<K, V> previous = entry.getValueReference();
            int weight = this.map.weigher.weigh(key, value);
            Preconditions.checkState(weight >= 0, "Weights must be non-negative");
            entry.setValueReference(this.map.valueStrength.referenceValue(this, entry, value, weight));
            recordWrite(entry, weight, now);
            previous.notifyNewValue(value);
        }

        /* access modifiers changed from: package-private */
        public V get(K key, int hash, CacheLoader<? super K, V> loader) throws ExecutionException {
            ReferenceEntry<K, V> e;
            try {
                if (!(this.count == 0 || (e = getEntry(key, hash)) == null)) {
                    long now = this.map.ticker.read();
                    V value = getLiveValue(e, now);
                    if (value != null) {
                        recordRead(e, now);
                        this.statsCounter.recordHits(1);
                        V scheduleRefresh = scheduleRefresh(e, key, hash, value, now, loader);
                        postReadCleanup();
                        return scheduleRefresh;
                    }
                    ValueReference<K, V> valueReference = e.getValueReference();
                    if (valueReference.isLoading()) {
                        V waitForLoadingValue = waitForLoadingValue(e, key, valueReference);
                        postReadCleanup();
                        return waitForLoadingValue;
                    }
                }
                V lockedGetOrLoad = lockedGetOrLoad(key, hash, loader);
                postReadCleanup();
                return lockedGetOrLoad;
            } catch (ExecutionException ee) {
                Throwable cause = ee.getCause();
                if (cause instanceof Error) {
                    throw new ExecutionError((Error) cause);
                } else if (cause instanceof RuntimeException) {
                    throw new UncheckedExecutionException(cause);
                } else {
                    throw ee;
                }
            } catch (Throwable th) {
                postReadCleanup();
                throw th;
            }
        }

        /* access modifiers changed from: package-private */
        /* JADX WARNING: Code restructure failed: missing block: B:12:0x0048, code lost:
            r4 = r13.getValueReference();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:15:0x004d, code lost:
            if (r4.isLoading() == false) goto L_0x0051;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:16:0x004f, code lost:
            r6 = false;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:17:0x0051, code lost:
            r15 = r4.get();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:18:0x0055, code lost:
            if (r15 != null) goto L_0x005d;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:20:?, code lost:
            enqueueNotification(r14, r3, r4, com.google.common.cache.RemovalCause.COLLECTED);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:23:0x0063, code lost:
            if (r1.map.isExpired(r13, r8) == false) goto L_0x0077;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:25:?, code lost:
            enqueueNotification(r14, r3, r4, com.google.common.cache.RemovalCause.EXPIRED);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:26:0x006a, code lost:
            r1.writeQueue.remove(r13);
            r1.accessQueue.remove(r13);
            r1.count = r0;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:28:?, code lost:
            recordLockedRead(r13, r8);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:29:0x007c, code lost:
            r16 = r4;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:31:?, code lost:
            r1.statsCounter.recordHits(1);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:32:0x0082, code lost:
            unlock();
            postWriteCleanup();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:33:0x0089, code lost:
            return r15;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:34:0x008a, code lost:
            r0 = th;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:35:0x008b, code lost:
            r8 = r20;
            r4 = r16;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:36:0x0091, code lost:
            r0 = th;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:37:0x0092, code lost:
            r16 = r4;
            r8 = r20;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:68:0x00db, code lost:
            r0 = th;
         */
        /* Code decompiled incorrectly, please refer to instructions dump. */
        public V lockedGetOrLoad(K r18, int r19, com.google.common.cache.CacheLoader<? super K, V> r20) throws java.util.concurrent.ExecutionException {
            /*
                r17 = this;
                r1 = r17
                r2 = r18
                r3 = r19
                r4 = 0
                r5 = 0
                r6 = 1
                r17.lock()
                r7 = 0
                com.google.common.cache.LocalCache<K, V> r0 = r1.map     // Catch:{ all -> 0x00ee }
                com.google.common.base.Ticker r0 = r0.ticker     // Catch:{ all -> 0x00ee }
                long r8 = r0.read()     // Catch:{ all -> 0x00ee }
                r1.preWriteCleanup(r8)     // Catch:{ all -> 0x00ee }
                int r0 = r1.count     // Catch:{ all -> 0x00ee }
                r10 = 1
                int r0 = r0 - r10
                java.util.concurrent.atomic.AtomicReferenceArray<com.google.common.cache.LocalCache$ReferenceEntry<K, V>> r11 = r1.table     // Catch:{ all -> 0x00ee }
                int r12 = r11.length()     // Catch:{ all -> 0x00ee }
                int r12 = r12 - r10
                r12 = r12 & r3
                java.lang.Object r13 = r11.get(r12)     // Catch:{ all -> 0x00ee }
                com.google.common.cache.LocalCache$ReferenceEntry r13 = (com.google.common.cache.LocalCache.ReferenceEntry) r13     // Catch:{ all -> 0x00ee }
                r7 = r13
            L_0x002c:
                if (r13 == 0) goto L_0x00a2
                java.lang.Object r14 = r13.getKey()     // Catch:{ all -> 0x009e }
                int r15 = r13.getHash()     // Catch:{ all -> 0x009e }
                if (r15 != r3) goto L_0x0097
                if (r14 == 0) goto L_0x0097
                com.google.common.cache.LocalCache<K, V> r15 = r1.map     // Catch:{ all -> 0x009e }
                com.google.common.base.Equivalence<java.lang.Object> r15 = r15.keyEquivalence     // Catch:{ all -> 0x009e }
                boolean r15 = r15.equivalent(r2, r14)     // Catch:{ all -> 0x009e }
                if (r15 == 0) goto L_0x0097
                com.google.common.cache.LocalCache$ValueReference r15 = r13.getValueReference()     // Catch:{ all -> 0x009e }
                r4 = r15
                boolean r15 = r4.isLoading()     // Catch:{ all -> 0x0091 }
                if (r15 == 0) goto L_0x0051
                r6 = 0
                goto L_0x00a2
            L_0x0051:
                java.lang.Object r15 = r4.get()     // Catch:{ all -> 0x0091 }
                if (r15 != 0) goto L_0x005d
                com.google.common.cache.RemovalCause r10 = com.google.common.cache.RemovalCause.COLLECTED     // Catch:{ all -> 0x009e }
                r1.enqueueNotification(r14, r3, r4, r10)     // Catch:{ all -> 0x009e }
                goto L_0x006a
            L_0x005d:
                com.google.common.cache.LocalCache<K, V> r10 = r1.map     // Catch:{ all -> 0x0091 }
                boolean r10 = r10.isExpired(r13, r8)     // Catch:{ all -> 0x0091 }
                if (r10 == 0) goto L_0x0077
                com.google.common.cache.RemovalCause r10 = com.google.common.cache.RemovalCause.EXPIRED     // Catch:{ all -> 0x009e }
                r1.enqueueNotification(r14, r3, r4, r10)     // Catch:{ all -> 0x009e }
            L_0x006a:
                java.util.Queue<com.google.common.cache.LocalCache$ReferenceEntry<K, V>> r10 = r1.writeQueue     // Catch:{ all -> 0x009e }
                r10.remove(r13)     // Catch:{ all -> 0x009e }
                java.util.Queue<com.google.common.cache.LocalCache$ReferenceEntry<K, V>> r10 = r1.accessQueue     // Catch:{ all -> 0x009e }
                r10.remove(r13)     // Catch:{ all -> 0x009e }
                r1.count = r0     // Catch:{ all -> 0x009e }
                goto L_0x00a2
            L_0x0077:
                r1.recordLockedRead(r13, r8)     // Catch:{ all -> 0x0091 }
                com.google.common.cache.AbstractCache$StatsCounter r10 = r1.statsCounter     // Catch:{ all -> 0x0091 }
                r16 = r4
                r4 = 1
                r10.recordHits(r4)     // Catch:{ all -> 0x008a }
                r17.unlock()
                r17.postWriteCleanup()
                return r15
            L_0x008a:
                r0 = move-exception
                r8 = r20
                r4 = r16
                goto L_0x00f2
            L_0x0091:
                r0 = move-exception
                r16 = r4
                r8 = r20
                goto L_0x00f2
            L_0x0097:
                com.google.common.cache.LocalCache$ReferenceEntry r10 = r13.getNext()     // Catch:{ all -> 0x009e }
                r13 = r10
                r10 = 1
                goto L_0x002c
            L_0x009e:
                r0 = move-exception
                r8 = r20
                goto L_0x00f2
            L_0x00a2:
                if (r6 == 0) goto L_0x00c0
                com.google.common.cache.LocalCache$LoadingValueReference r10 = new com.google.common.cache.LocalCache$LoadingValueReference     // Catch:{ all -> 0x009e }
                r10.<init>()     // Catch:{ all -> 0x009e }
                r5 = r10
                if (r13 != 0) goto L_0x00bd
                com.google.common.cache.LocalCache$ReferenceEntry r10 = r1.newEntry(r2, r3, r7)     // Catch:{ all -> 0x009e }
                r10.setValueReference(r5)     // Catch:{ all -> 0x00b8 }
                r11.set(r12, r10)     // Catch:{ all -> 0x00b8 }
                r13 = r10
                goto L_0x00c0
            L_0x00b8:
                r0 = move-exception
                r8 = r20
                r13 = r10
                goto L_0x00f2
            L_0x00bd:
                r13.setValueReference(r5)     // Catch:{ all -> 0x009e }
            L_0x00c0:
                r17.unlock()
                r17.postWriteCleanup()
                if (r6 == 0) goto L_0x00e7
                monitor-enter(r13)     // Catch:{ all -> 0x00dd }
                r8 = r20
                java.lang.Object r0 = r1.loadSync(r2, r3, r5, r8)     // Catch:{ all -> 0x00d8 }
                monitor-exit(r13)     // Catch:{ all -> 0x00d8 }
                com.google.common.cache.AbstractCache$StatsCounter r7 = r1.statsCounter
                r9 = 1
                r7.recordMisses(r9)
                return r0
            L_0x00d8:
                r0 = move-exception
                monitor-exit(r13)     // Catch:{ all -> 0x00d8 }
                throw r0     // Catch:{ all -> 0x00db }
            L_0x00db:
                r0 = move-exception
                goto L_0x00e0
            L_0x00dd:
                r0 = move-exception
                r8 = r20
            L_0x00e0:
                com.google.common.cache.AbstractCache$StatsCounter r7 = r1.statsCounter
                r9 = 1
                r7.recordMisses(r9)
                throw r0
            L_0x00e7:
                r8 = r20
                java.lang.Object r0 = r1.waitForLoadingValue(r13, r2, r4)
                return r0
            L_0x00ee:
                r0 = move-exception
                r8 = r20
                r13 = r7
            L_0x00f2:
                r17.unlock()
                r17.postWriteCleanup()
                throw r0
            */
            throw new UnsupportedOperationException("Method not decompiled: com.google.common.cache.LocalCache.Segment.lockedGetOrLoad(java.lang.Object, int, com.google.common.cache.CacheLoader):java.lang.Object");
        }

        /* access modifiers changed from: package-private */
        public V waitForLoadingValue(ReferenceEntry<K, V> e, K key, ValueReference<K, V> valueReference) throws ExecutionException {
            if (valueReference.isLoading()) {
                Preconditions.checkState(!Thread.holdsLock(e), "Recursive load");
                try {
                    V value = valueReference.waitForValue();
                    if (value != null) {
                        recordRead(e, this.map.ticker.read());
                        return value;
                    }
                    throw new CacheLoader.InvalidCacheLoadException("CacheLoader returned null for key " + key + ".");
                } finally {
                    this.statsCounter.recordMisses(1);
                }
            } else {
                throw new AssertionError();
            }
        }

        /* access modifiers changed from: package-private */
        public V loadSync(K key, int hash, LoadingValueReference<K, V> loadingValueReference, CacheLoader<? super K, V> loader) throws ExecutionException {
            return getAndRecordStats(key, hash, loadingValueReference, loadingValueReference.loadFuture(key, loader));
        }

        /* access modifiers changed from: package-private */
        public ListenableFuture<V> loadAsync(K key, int hash, LoadingValueReference<K, V> loadingValueReference, CacheLoader<? super K, V> loader) {
            ListenableFuture<V> loadingFuture = loadingValueReference.loadFuture(key, loader);
            final K k = key;
            final int i = hash;
            final LoadingValueReference<K, V> loadingValueReference2 = loadingValueReference;
            final ListenableFuture<V> listenableFuture = loadingFuture;
            loadingFuture.addListener(new Runnable() {
                public void run() {
                    try {
                        loadingValueReference2.set(Segment.this.getAndRecordStats(k, i, loadingValueReference2, listenableFuture));
                    } catch (Throwable t) {
                        LocalCache.logger.log(Level.WARNING, "Exception thrown during refresh", t);
                        loadingValueReference2.setException(t);
                    }
                }
            }, LocalCache.sameThreadExecutor);
            return loadingFuture;
        }

        /* access modifiers changed from: package-private */
        public V getAndRecordStats(K key, int hash, LoadingValueReference<K, V> loadingValueReference, ListenableFuture<V> newValue) throws ExecutionException {
            V value = null;
            try {
                value = Uninterruptibles.getUninterruptibly(newValue);
                if (value != null) {
                    this.statsCounter.recordLoadSuccess(loadingValueReference.elapsedNanos());
                    storeLoadedValue(key, hash, loadingValueReference, value);
                    return value;
                }
                throw new CacheLoader.InvalidCacheLoadException("CacheLoader returned null for key " + key + ".");
            } finally {
                if (value == null) {
                    this.statsCounter.recordLoadException(loadingValueReference.elapsedNanos());
                    removeLoadingValue(key, hash, loadingValueReference);
                }
            }
        }

        /* access modifiers changed from: package-private */
        public V scheduleRefresh(ReferenceEntry<K, V> entry, K key, int hash, V oldValue, long now, CacheLoader<? super K, V> loader) {
            V newValue;
            if (!this.map.refreshes() || now - entry.getWriteTime() <= this.map.refreshNanos || (newValue = refresh(key, hash, loader)) == null) {
                return oldValue;
            }
            return newValue;
        }

        /* access modifiers changed from: package-private */
        @Nullable
        public V refresh(K key, int hash, CacheLoader<? super K, V> loader) {
            LoadingValueReference<K, V> loadingValueReference = insertLoadingValueReference(key, hash);
            if (loadingValueReference == null) {
                return null;
            }
            ListenableFuture<V> result = loadAsync(key, hash, loadingValueReference, loader);
            if (result.isDone()) {
                try {
                    return Uninterruptibles.getUninterruptibly(result);
                } catch (Throwable th) {
                }
            }
            return null;
        }

        /* access modifiers changed from: package-private */
        @Nullable
        public LoadingValueReference<K, V> insertLoadingValueReference(K key, int hash) {
            LoadingValueReference<K, V> loadingValueReference;
            lock();
            try {
                preWriteCleanup(this.map.ticker.read());
                AtomicReferenceArray<ReferenceEntry<K, V>> table2 = this.table;
                int index = (table2.length() - 1) & hash;
                ReferenceEntry<K, V> first = table2.get(index);
                ReferenceEntry<K, V> e = first;
                while (e != null) {
                    K entryKey = e.getKey();
                    if (e.getHash() != hash || entryKey == null || !this.map.keyEquivalence.equivalent(key, entryKey)) {
                        e = e.getNext();
                    } else {
                        ValueReference<K, V> valueReference = e.getValueReference();
                        if (valueReference.isLoading()) {
                            loadingValueReference = null;
                        } else {
                            this.modCount++;
                            loadingValueReference = new LoadingValueReference<>(valueReference);
                            e.setValueReference(loadingValueReference);
                        }
                        return loadingValueReference;
                    }
                }
                this.modCount++;
                LoadingValueReference<K, V> loadingValueReference2 = new LoadingValueReference<>();
                ReferenceEntry<K, V> e2 = newEntry(key, hash, first);
                e2.setValueReference(loadingValueReference2);
                table2.set(index, e2);
                unlock();
                postWriteCleanup();
                return loadingValueReference2;
            } finally {
                unlock();
                postWriteCleanup();
            }
        }

        /* access modifiers changed from: package-private */
        public void tryDrainReferenceQueues() {
            if (tryLock()) {
                try {
                    drainReferenceQueues();
                } finally {
                    unlock();
                }
            }
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public void drainReferenceQueues() {
            if (this.map.usesKeyReferences()) {
                drainKeyReferenceQueue();
            }
            if (this.map.usesValueReferences()) {
                drainValueReferenceQueue();
            }
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public void drainKeyReferenceQueue() {
            int i = 0;
            do {
                Reference<? extends K> poll = this.keyReferenceQueue.poll();
                Reference<? extends K> ref = poll;
                if (poll != null) {
                    this.map.reclaimKey((ReferenceEntry) ref);
                    i++;
                } else {
                    return;
                }
            } while (i != 16);
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public void drainValueReferenceQueue() {
            int i = 0;
            do {
                Reference<? extends V> poll = this.valueReferenceQueue.poll();
                Reference<? extends V> ref = poll;
                if (poll != null) {
                    this.map.reclaimValue((ValueReference) ref);
                    i++;
                } else {
                    return;
                }
            } while (i != 16);
        }

        /* access modifiers changed from: package-private */
        public void clearReferenceQueues() {
            if (this.map.usesKeyReferences()) {
                clearKeyReferenceQueue();
            }
            if (this.map.usesValueReferences()) {
                clearValueReferenceQueue();
            }
        }

        /* access modifiers changed from: package-private */
        public void clearKeyReferenceQueue() {
            do {
            } while (this.keyReferenceQueue.poll() != null);
        }

        /* access modifiers changed from: package-private */
        public void clearValueReferenceQueue() {
            do {
            } while (this.valueReferenceQueue.poll() != null);
        }

        /* access modifiers changed from: package-private */
        public void recordRead(ReferenceEntry<K, V> entry, long now) {
            if (this.map.recordsAccess()) {
                entry.setAccessTime(now);
            }
            this.recencyQueue.add(entry);
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public void recordLockedRead(ReferenceEntry<K, V> entry, long now) {
            if (this.map.recordsAccess()) {
                entry.setAccessTime(now);
            }
            this.accessQueue.add(entry);
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public void recordWrite(ReferenceEntry<K, V> entry, int weight, long now) {
            drainRecencyQueue();
            this.totalWeight += weight;
            if (this.map.recordsAccess()) {
                entry.setAccessTime(now);
            }
            if (this.map.recordsWrite()) {
                entry.setWriteTime(now);
            }
            this.accessQueue.add(entry);
            this.writeQueue.add(entry);
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public void drainRecencyQueue() {
            while (true) {
                ReferenceEntry<K, V> poll = this.recencyQueue.poll();
                ReferenceEntry<K, V> e = poll;
                if (poll == null) {
                    return;
                }
                if (this.accessQueue.contains(e)) {
                    this.accessQueue.add(e);
                }
            }
        }

        /* access modifiers changed from: package-private */
        public void tryExpireEntries(long now) {
            if (tryLock()) {
                try {
                    expireEntries(now);
                } finally {
                    unlock();
                }
            }
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public void expireEntries(long now) {
            ReferenceEntry<K, V> e;
            ReferenceEntry<K, V> e2;
            drainRecencyQueue();
            do {
                ReferenceEntry<K, V> peek = this.writeQueue.peek();
                e = peek;
                if (peek == null || !this.map.isExpired(e, now)) {
                    do {
                        ReferenceEntry<K, V> peek2 = this.accessQueue.peek();
                        e2 = peek2;
                        if (peek2 == null || !this.map.isExpired(e2, now)) {
                            return;
                        }
                    } while (removeEntry(e2, e2.getHash(), RemovalCause.EXPIRED));
                    throw new AssertionError();
                }
            } while (removeEntry(e, e.getHash(), RemovalCause.EXPIRED));
            throw new AssertionError();
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public void enqueueNotification(ReferenceEntry<K, V> entry, RemovalCause cause) {
            enqueueNotification(entry.getKey(), entry.getHash(), entry.getValueReference(), cause);
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public void enqueueNotification(@Nullable K key, int hash, ValueReference<K, V> valueReference, RemovalCause cause) {
            this.totalWeight -= valueReference.getWeight();
            if (cause.wasEvicted()) {
                this.statsCounter.recordEviction();
            }
            if (this.map.removalNotificationQueue != LocalCache.DISCARDING_QUEUE) {
                this.map.removalNotificationQueue.offer(new RemovalNotification<>(key, valueReference.get(), cause));
            }
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public void evictEntries() {
            if (this.map.evictsBySize()) {
                drainRecencyQueue();
                while (((long) this.totalWeight) > this.maxSegmentWeight) {
                    ReferenceEntry<K, V> e = getNextEvictable();
                    if (!removeEntry(e, e.getHash(), RemovalCause.SIZE)) {
                        throw new AssertionError();
                    }
                }
            }
        }

        /* access modifiers changed from: package-private */
        public ReferenceEntry<K, V> getNextEvictable() {
            for (ReferenceEntry<K, V> e : this.accessQueue) {
                if (e.getValueReference().getWeight() > 0) {
                    return e;
                }
            }
            throw new AssertionError();
        }

        /* access modifiers changed from: package-private */
        public ReferenceEntry<K, V> getFirst(int hash) {
            AtomicReferenceArray<ReferenceEntry<K, V>> table2 = this.table;
            return table2.get((table2.length() - 1) & hash);
        }

        /* access modifiers changed from: package-private */
        @Nullable
        public ReferenceEntry<K, V> getEntry(Object key, int hash) {
            for (ReferenceEntry<K, V> e = getFirst(hash); e != null; e = e.getNext()) {
                if (e.getHash() == hash) {
                    K entryKey = e.getKey();
                    if (entryKey == null) {
                        tryDrainReferenceQueues();
                    } else if (this.map.keyEquivalence.equivalent(key, entryKey)) {
                        return e;
                    }
                }
            }
            return null;
        }

        /* access modifiers changed from: package-private */
        @Nullable
        public ReferenceEntry<K, V> getLiveEntry(Object key, int hash, long now) {
            ReferenceEntry<K, V> e = getEntry(key, hash);
            if (e == null) {
                return null;
            }
            if (!this.map.isExpired(e, now)) {
                return e;
            }
            tryExpireEntries(now);
            return null;
        }

        /* access modifiers changed from: package-private */
        public V getLiveValue(ReferenceEntry<K, V> entry, long now) {
            if (entry.getKey() == null) {
                tryDrainReferenceQueues();
                return null;
            }
            V value = entry.getValueReference().get();
            if (value == null) {
                tryDrainReferenceQueues();
                return null;
            } else if (!this.map.isExpired(entry, now)) {
                return value;
            } else {
                tryExpireEntries(now);
                return null;
            }
        }

        /* access modifiers changed from: package-private */
        /* JADX WARNING: Code restructure failed: missing block: B:3:0x0005, code lost:
            r2 = r13.map.ticker.read();
         */
        @javax.annotation.Nullable
        /* Code decompiled incorrectly, please refer to instructions dump. */
        public V get(java.lang.Object r14, int r15) {
            /*
                r13 = this;
                int r0 = r13.count     // Catch:{ all -> 0x003c }
                r1 = 0
                if (r0 == 0) goto L_0x003b
                com.google.common.cache.LocalCache<K, V> r0 = r13.map     // Catch:{ all -> 0x003c }
                com.google.common.base.Ticker r0 = r0.ticker     // Catch:{ all -> 0x003c }
                long r2 = r0.read()     // Catch:{ all -> 0x003c }
                com.google.common.cache.LocalCache$ReferenceEntry r0 = r13.getLiveEntry(r14, r15, r2)     // Catch:{ all -> 0x003c }
                if (r0 != 0) goto L_0x0018
            L_0x0014:
                r13.postReadCleanup()
                return r1
            L_0x0018:
                com.google.common.cache.LocalCache$ValueReference r4 = r0.getValueReference()     // Catch:{ all -> 0x003c }
                java.lang.Object r4 = r4.get()     // Catch:{ all -> 0x003c }
                r12 = r4
                if (r12 == 0) goto L_0x0038
                r13.recordRead(r0, r2)     // Catch:{ all -> 0x003c }
                java.lang.Object r6 = r0.getKey()     // Catch:{ all -> 0x003c }
                com.google.common.cache.LocalCache<K, V> r1 = r13.map     // Catch:{ all -> 0x003c }
                com.google.common.cache.CacheLoader<? super K, V> r11 = r1.defaultLoader     // Catch:{ all -> 0x003c }
                r4 = r13
                r5 = r0
                r7 = r15
                r8 = r12
                r9 = r2
                java.lang.Object r1 = r4.scheduleRefresh(r5, r6, r7, r8, r9, r11)     // Catch:{ all -> 0x003c }
                goto L_0x0014
            L_0x0038:
                r13.tryDrainReferenceQueues()     // Catch:{ all -> 0x003c }
            L_0x003b:
                goto L_0x0014
            L_0x003c:
                r0 = move-exception
                r13.postReadCleanup()
                throw r0
            */
            throw new UnsupportedOperationException("Method not decompiled: com.google.common.cache.LocalCache.Segment.get(java.lang.Object, int):java.lang.Object");
        }

        /* access modifiers changed from: package-private */
        public boolean containsKey(Object key, int hash) {
            ReferenceEntry<K, V> e;
            try {
                boolean z = false;
                if (!(this.count == 0 || (e = getLiveEntry(key, hash, this.map.ticker.read())) == null)) {
                    if (e.getValueReference().get() != null) {
                        z = true;
                    }
                }
                return z;
            } finally {
                postReadCleanup();
            }
        }

        /* JADX INFO: finally extract failed */
        /* access modifiers changed from: package-private */
        @VisibleForTesting
        public boolean containsValue(Object value) {
            try {
                if (this.count != 0) {
                    long now = this.map.ticker.read();
                    AtomicReferenceArray<ReferenceEntry<K, V>> table2 = this.table;
                    int length = table2.length();
                    for (int i = 0; i < length; i++) {
                        for (ReferenceEntry<K, V> e = table2.get(i); e != null; e = e.getNext()) {
                            V entryValue = getLiveValue(e, now);
                            if (entryValue != null) {
                                if (this.map.valueEquivalence.equivalent(value, entryValue)) {
                                    postReadCleanup();
                                    return true;
                                }
                            }
                        }
                    }
                }
                postReadCleanup();
                return false;
            } catch (Throwable th) {
                postReadCleanup();
                throw th;
            }
        }

        /* access modifiers changed from: package-private */
        /* JADX WARNING: Code restructure failed: missing block: B:13:0x0053, code lost:
            r6 = r15.getValueReference();
            r17 = r6.get();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:14:0x005e, code lost:
            if (r17 != null) goto L_0x00a5;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:15:0x0060, code lost:
            r7.modCount++;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:16:0x006a, code lost:
            if (r6.isActive() == false) goto L_0x0084;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:17:0x006c, code lost:
            enqueueNotification(r8, r9, r6, com.google.common.cache.RemovalCause.COLLECTED);
            r19 = r0;
            r18 = r5;
            r0 = r6;
            setValue(r15, r21, r23, r10);
            r1 = r7.count;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:18:0x0084, code lost:
            r19 = r0;
            r18 = r5;
            r0 = r6;
            setValue(r15, r21, r23, r10);
            r1 = r7.count + 1;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:19:0x0098, code lost:
            r7.count = r1;
            evictEntries();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:22:0x00a5, code lost:
            r19 = r0;
            r18 = r5;
            r0 = r6;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:23:0x00aa, code lost:
            if (r24 == false) goto L_0x00b7;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:25:?, code lost:
            recordLockedRead(r15, r10);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:26:0x00b0, code lost:
            unlock();
            postWriteCleanup();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:27:0x00b6, code lost:
            return r17;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:29:?, code lost:
            r7.modCount++;
            enqueueNotification(r8, r9, r0, com.google.common.cache.RemovalCause.REPLACED);
            setValue(r15, r21, r23, r10);
            evictEntries();
         */
        @javax.annotation.Nullable
        /* Code decompiled incorrectly, please refer to instructions dump. */
        public V put(K r21, int r22, V r23, boolean r24) {
            /*
                r20 = this;
                r7 = r20
                r8 = r21
                r9 = r22
                r20.lock()
                com.google.common.cache.LocalCache<K, V> r0 = r7.map     // Catch:{ all -> 0x0100 }
                com.google.common.base.Ticker r0 = r0.ticker     // Catch:{ all -> 0x0100 }
                long r0 = r0.read()     // Catch:{ all -> 0x0100 }
                r10 = r0
                r7.preWriteCleanup(r10)     // Catch:{ all -> 0x0100 }
                int r0 = r7.count     // Catch:{ all -> 0x0100 }
                int r0 = r0 + 1
                int r1 = r7.threshold     // Catch:{ all -> 0x0100 }
                if (r0 <= r1) goto L_0x0024
                r20.expand()     // Catch:{ all -> 0x0100 }
                int r1 = r7.count     // Catch:{ all -> 0x0100 }
                int r0 = r1 + 1
            L_0x0024:
                java.util.concurrent.atomic.AtomicReferenceArray<com.google.common.cache.LocalCache$ReferenceEntry<K, V>> r1 = r7.table     // Catch:{ all -> 0x0100 }
                r12 = r1
                int r1 = r12.length()     // Catch:{ all -> 0x0100 }
                int r1 = r1 + -1
                r13 = r9 & r1
                java.lang.Object r1 = r12.get(r13)     // Catch:{ all -> 0x0100 }
                com.google.common.cache.LocalCache$ReferenceEntry r1 = (com.google.common.cache.LocalCache.ReferenceEntry) r1     // Catch:{ all -> 0x0100 }
                r14 = r1
            L_0x0037:
                r15 = r1
                r16 = 0
                if (r15 == 0) goto L_0x00dc
                java.lang.Object r1 = r15.getKey()     // Catch:{ all -> 0x0100 }
                r5 = r1
                int r1 = r15.getHash()     // Catch:{ all -> 0x0100 }
                if (r1 != r9) goto L_0x00d1
                if (r5 == 0) goto L_0x00d1
                com.google.common.cache.LocalCache<K, V> r1 = r7.map     // Catch:{ all -> 0x0100 }
                com.google.common.base.Equivalence<java.lang.Object> r1 = r1.keyEquivalence     // Catch:{ all -> 0x0100 }
                boolean r1 = r1.equivalent(r8, r5)     // Catch:{ all -> 0x0100 }
                if (r1 == 0) goto L_0x00d1
                com.google.common.cache.LocalCache$ValueReference r1 = r15.getValueReference()     // Catch:{ all -> 0x0100 }
                r6 = r1
                java.lang.Object r1 = r6.get()     // Catch:{ all -> 0x0100 }
                r17 = r1
                if (r17 != 0) goto L_0x00a5
                int r1 = r7.modCount     // Catch:{ all -> 0x0100 }
                int r1 = r1 + 1
                r7.modCount = r1     // Catch:{ all -> 0x0100 }
                boolean r1 = r6.isActive()     // Catch:{ all -> 0x0100 }
                if (r1 == 0) goto L_0x0084
                com.google.common.cache.RemovalCause r1 = com.google.common.cache.RemovalCause.COLLECTED     // Catch:{ all -> 0x0100 }
                r7.enqueueNotification(r8, r9, r6, r1)     // Catch:{ all -> 0x0100 }
                r1 = r20
                r2 = r15
                r3 = r21
                r4 = r23
                r19 = r0
                r18 = r5
                r0 = r6
                r5 = r10
                r1.setValue(r2, r3, r4, r5)     // Catch:{ all -> 0x0100 }
                int r1 = r7.count     // Catch:{ all -> 0x0100 }
                goto L_0x0098
            L_0x0084:
                r19 = r0
                r18 = r5
                r0 = r6
                r1 = r20
                r2 = r15
                r3 = r21
                r4 = r23
                r5 = r10
                r1.setValue(r2, r3, r4, r5)     // Catch:{ all -> 0x0100 }
                int r1 = r7.count     // Catch:{ all -> 0x0100 }
                int r1 = r1 + 1
            L_0x0098:
                r7.count = r1     // Catch:{ all -> 0x0100 }
                r20.evictEntries()     // Catch:{ all -> 0x0100 }
            L_0x009e:
                r20.unlock()
                r20.postWriteCleanup()
                return r16
            L_0x00a5:
                r19 = r0
                r18 = r5
                r0 = r6
                if (r24 == 0) goto L_0x00b7
                r7.recordLockedRead(r15, r10)     // Catch:{ all -> 0x0100 }
            L_0x00b0:
                r20.unlock()
                r20.postWriteCleanup()
                return r17
            L_0x00b7:
                int r1 = r7.modCount     // Catch:{ all -> 0x0100 }
                int r1 = r1 + 1
                r7.modCount = r1     // Catch:{ all -> 0x0100 }
                com.google.common.cache.RemovalCause r1 = com.google.common.cache.RemovalCause.REPLACED     // Catch:{ all -> 0x0100 }
                r7.enqueueNotification(r8, r9, r0, r1)     // Catch:{ all -> 0x0100 }
                r1 = r20
                r2 = r15
                r3 = r21
                r4 = r23
                r5 = r10
                r1.setValue(r2, r3, r4, r5)     // Catch:{ all -> 0x0100 }
                r20.evictEntries()     // Catch:{ all -> 0x0100 }
                goto L_0x00b0
            L_0x00d1:
                r19 = r0
                com.google.common.cache.LocalCache$ReferenceEntry r0 = r15.getNext()     // Catch:{ all -> 0x0100 }
                r1 = r0
                r0 = r19
                goto L_0x0037
            L_0x00dc:
                r19 = r0
                int r0 = r7.modCount     // Catch:{ all -> 0x0100 }
                int r0 = r0 + 1
                r7.modCount = r0     // Catch:{ all -> 0x0100 }
                com.google.common.cache.LocalCache$ReferenceEntry r0 = r7.newEntry(r8, r9, r14)     // Catch:{ all -> 0x0100 }
                r1 = r20
                r2 = r0
                r3 = r21
                r4 = r23
                r5 = r10
                r1.setValue(r2, r3, r4, r5)     // Catch:{ all -> 0x0100 }
                r12.set(r13, r0)     // Catch:{ all -> 0x0100 }
                int r1 = r7.count     // Catch:{ all -> 0x0100 }
                int r1 = r1 + 1
                r7.count = r1     // Catch:{ all -> 0x0100 }
                r20.evictEntries()     // Catch:{ all -> 0x0100 }
                goto L_0x009e
            L_0x0100:
                r0 = move-exception
                r20.unlock()
                r20.postWriteCleanup()
                throw r0
            */
            throw new UnsupportedOperationException("Method not decompiled: com.google.common.cache.LocalCache.Segment.put(java.lang.Object, int, java.lang.Object, boolean):java.lang.Object");
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public void expand() {
            AtomicReferenceArray<ReferenceEntry<K, V>> oldTable = this.table;
            int oldCapacity = oldTable.length();
            if (oldCapacity < 1073741824) {
                int newCount = this.count;
                AtomicReferenceArray<ReferenceEntry<K, V>> newTable = newEntryArray(oldCapacity << 1);
                this.threshold = (newTable.length() * 3) / 4;
                int newMask = newTable.length() - 1;
                for (int oldIndex = 0; oldIndex < oldCapacity; oldIndex++) {
                    ReferenceEntry<K, V> head = oldTable.get(oldIndex);
                    if (head != null) {
                        ReferenceEntry<K, V> next = head.getNext();
                        int headIndex = head.getHash() & newMask;
                        if (next == null) {
                            newTable.set(headIndex, head);
                        } else {
                            int tailIndex = headIndex;
                            ReferenceEntry<K, V> tail = head;
                            for (ReferenceEntry<K, V> e = next; e != null; e = e.getNext()) {
                                int newIndex = e.getHash() & newMask;
                                if (newIndex != tailIndex) {
                                    tailIndex = newIndex;
                                    tail = e;
                                }
                            }
                            newTable.set(tailIndex, tail);
                            int newCount2 = newCount;
                            for (ReferenceEntry<K, V> e2 = head; e2 != tail; e2 = e2.getNext()) {
                                int newIndex2 = e2.getHash() & newMask;
                                ReferenceEntry<K, V> newFirst = copyEntry(e2, newTable.get(newIndex2));
                                if (newFirst != null) {
                                    newTable.set(newIndex2, newFirst);
                                } else {
                                    removeCollectedEntry(e2);
                                    newCount2--;
                                }
                            }
                            newCount = newCount2;
                        }
                    }
                }
                this.table = newTable;
                this.count = newCount;
            }
        }

        /* access modifiers changed from: package-private */
        /* JADX WARNING: Code restructure failed: missing block: B:10:0x0043, code lost:
            r6 = r8.getValueReference();
            r5 = r6.get();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:11:0x004d, code lost:
            if (r5 != null) goto L_0x0088;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:13:0x0053, code lost:
            if (r6.isActive() == false) goto L_0x007b;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:14:0x0055, code lost:
            r17 = r9.count - 1;
            r9.modCount++;
            r19 = r5;
            r20 = r6;
            r21 = r7;
            r0.set(r15, removeValueFromChain(r2, r8, r7, r25, r6, com.google.common.cache.RemovalCause.COLLECTED));
            r9.count--;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:15:0x007b, code lost:
            r19 = r5;
            r20 = r6;
            r21 = r7;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:18:0x0088, code lost:
            r20 = r6;
            r21 = r7;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:20:?, code lost:
            r6 = r5;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:21:0x009a, code lost:
            if (r9.map.valueEquivalence.equivalent(r26, r6) == false) goto L_0x00c3;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:22:0x009c, code lost:
            r9.modCount++;
            r5 = r20;
            enqueueNotification(r10, r11, r5, com.google.common.cache.RemovalCause.REPLACED);
            r1 = r5;
            r17 = r6;
            r22 = r8;
            setValue(r8, r24, r27, r12);
            evictEntries();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:23:0x00bb, code lost:
            unlock();
            postWriteCleanup();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:24:0x00c2, code lost:
            return true;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:25:0x00c3, code lost:
            r17 = r6;
            r1 = r20;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:27:?, code lost:
            recordLockedRead(r8, r12);
         */
        /* Code decompiled incorrectly, please refer to instructions dump. */
        public boolean replace(K r24, int r25, V r26, V r27) {
            /*
                r23 = this;
                r9 = r23
                r10 = r24
                r11 = r25
                r23.lock()
                com.google.common.cache.LocalCache<K, V> r0 = r9.map     // Catch:{ all -> 0x00d7 }
                com.google.common.base.Ticker r0 = r0.ticker     // Catch:{ all -> 0x00d7 }
                long r0 = r0.read()     // Catch:{ all -> 0x00d7 }
                r12 = r0
                r9.preWriteCleanup(r12)     // Catch:{ all -> 0x00d7 }
                java.util.concurrent.atomic.AtomicReferenceArray<com.google.common.cache.LocalCache$ReferenceEntry<K, V>> r0 = r9.table     // Catch:{ all -> 0x00d7 }
                int r1 = r0.length()     // Catch:{ all -> 0x00d7 }
                r14 = 1
                int r1 = r1 - r14
                r15 = r11 & r1
                java.lang.Object r1 = r0.get(r15)     // Catch:{ all -> 0x00d7 }
                r2 = r1
                com.google.common.cache.LocalCache$ReferenceEntry r2 = (com.google.common.cache.LocalCache.ReferenceEntry) r2     // Catch:{ all -> 0x00d7 }
                r1 = r2
            L_0x0027:
                r8 = r1
                r16 = 0
                if (r8 == 0) goto L_0x00d6
                java.lang.Object r1 = r8.getKey()     // Catch:{ all -> 0x00d7 }
                r7 = r1
                int r1 = r8.getHash()     // Catch:{ all -> 0x00d7 }
                if (r1 != r11) goto L_0x00cf
                if (r7 == 0) goto L_0x00cf
                com.google.common.cache.LocalCache<K, V> r1 = r9.map     // Catch:{ all -> 0x00d7 }
                com.google.common.base.Equivalence<java.lang.Object> r1 = r1.keyEquivalence     // Catch:{ all -> 0x00d7 }
                boolean r1 = r1.equivalent(r10, r7)     // Catch:{ all -> 0x00d7 }
                if (r1 == 0) goto L_0x00cf
                com.google.common.cache.LocalCache$ValueReference r1 = r8.getValueReference()     // Catch:{ all -> 0x00d7 }
                r6 = r1
                java.lang.Object r1 = r6.get()     // Catch:{ all -> 0x00d7 }
                r5 = r1
                if (r5 != 0) goto L_0x0088
                boolean r1 = r6.isActive()     // Catch:{ all -> 0x00d7 }
                if (r1 == 0) goto L_0x007b
                int r1 = r9.count     // Catch:{ all -> 0x00d7 }
                int r17 = r1 + -1
                int r1 = r9.modCount     // Catch:{ all -> 0x00d7 }
                int r1 = r1 + r14
                r9.modCount = r1     // Catch:{ all -> 0x00d7 }
                com.google.common.cache.RemovalCause r18 = com.google.common.cache.RemovalCause.COLLECTED     // Catch:{ all -> 0x00d7 }
                r1 = r23
                r3 = r8
                r4 = r7
                r19 = r5
                r5 = r25
                r20 = r6
                r21 = r7
                r7 = r18
                com.google.common.cache.LocalCache$ReferenceEntry r1 = r1.removeValueFromChain(r2, r3, r4, r5, r6, r7)     // Catch:{ all -> 0x00d7 }
                int r3 = r9.count     // Catch:{ all -> 0x00d7 }
                int r3 = r3 - r14
                r0.set(r15, r1)     // Catch:{ all -> 0x00d7 }
                r9.count = r3     // Catch:{ all -> 0x00d7 }
                goto L_0x0081
            L_0x007b:
                r19 = r5
                r20 = r6
                r21 = r7
            L_0x0081:
                r23.unlock()
                r23.postWriteCleanup()
                return r16
            L_0x0088:
                r19 = r5
                r20 = r6
                r21 = r7
                com.google.common.cache.LocalCache<K, V> r1 = r9.map     // Catch:{ all -> 0x00d7 }
                com.google.common.base.Equivalence<java.lang.Object> r1 = r1.valueEquivalence     // Catch:{ all -> 0x00d7 }
                r7 = r26
                r6 = r19
                boolean r1 = r1.equivalent(r7, r6)     // Catch:{ all -> 0x00d7 }
                if (r1 == 0) goto L_0x00c3
                int r1 = r9.modCount     // Catch:{ all -> 0x00d7 }
                int r1 = r1 + r14
                r9.modCount = r1     // Catch:{ all -> 0x00d7 }
                com.google.common.cache.RemovalCause r1 = com.google.common.cache.RemovalCause.REPLACED     // Catch:{ all -> 0x00d7 }
                r5 = r20
                r9.enqueueNotification(r10, r11, r5, r1)     // Catch:{ all -> 0x00d7 }
                r3 = r23
                r4 = r8
                r1 = r5
                r5 = r24
                r17 = r6
                r6 = r27
                r22 = r8
                r7 = r12
                r3.setValue(r4, r5, r6, r7)     // Catch:{ all -> 0x00d7 }
                r23.evictEntries()     // Catch:{ all -> 0x00d7 }
                r23.unlock()
                r23.postWriteCleanup()
                return r14
            L_0x00c3:
                r17 = r6
                r22 = r8
                r1 = r20
                r3 = r22
                r9.recordLockedRead(r3, r12)     // Catch:{ all -> 0x00d7 }
                goto L_0x0081
            L_0x00cf:
                r3 = r8
                com.google.common.cache.LocalCache$ReferenceEntry r1 = r3.getNext()     // Catch:{ all -> 0x00d7 }
                goto L_0x0027
            L_0x00d6:
                goto L_0x0081
            L_0x00d7:
                r0 = move-exception
                r23.unlock()
                r23.postWriteCleanup()
                throw r0
            */
            throw new UnsupportedOperationException("Method not decompiled: com.google.common.cache.LocalCache.Segment.replace(java.lang.Object, int, java.lang.Object, java.lang.Object):boolean");
        }

        /* access modifiers changed from: package-private */
        /* JADX WARNING: Code restructure failed: missing block: B:10:0x0042, code lost:
            r6 = r15.getValueReference();
            r16 = r6.get();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:11:0x004d, code lost:
            if (r16 != null) goto L_0x0086;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:13:0x0053, code lost:
            if (r6.isActive() == false) goto L_0x007b;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:14:0x0055, code lost:
            r17 = r9.count - 1;
            r9.modCount++;
            r19 = r6;
            r20 = r7;
            r0.set(r14, removeValueFromChain(r2, r15, r7, r23, r6, com.google.common.cache.RemovalCause.COLLECTED));
            r9.count--;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:15:0x007b, code lost:
            r19 = r6;
            r20 = r7;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:18:0x0086, code lost:
            r19 = r6;
            r20 = r7;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:20:?, code lost:
            r9.modCount++;
            r7 = r19;
            enqueueNotification(r10, r11, r7, com.google.common.cache.RemovalCause.REPLACED);
            r1 = r7;
            setValue(r15, r22, r24, r12);
            evictEntries();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:21:0x00a6, code lost:
            unlock();
            postWriteCleanup();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:22:0x00ad, code lost:
            return r16;
         */
        @javax.annotation.Nullable
        /* Code decompiled incorrectly, please refer to instructions dump. */
        public V replace(K r22, int r23, V r24) {
            /*
                r21 = this;
                r9 = r21
                r10 = r22
                r11 = r23
                r21.lock()
                com.google.common.cache.LocalCache<K, V> r0 = r9.map     // Catch:{ all -> 0x00b5 }
                com.google.common.base.Ticker r0 = r0.ticker     // Catch:{ all -> 0x00b5 }
                long r0 = r0.read()     // Catch:{ all -> 0x00b5 }
                r12 = r0
                r9.preWriteCleanup(r12)     // Catch:{ all -> 0x00b5 }
                java.util.concurrent.atomic.AtomicReferenceArray<com.google.common.cache.LocalCache$ReferenceEntry<K, V>> r0 = r9.table     // Catch:{ all -> 0x00b5 }
                int r1 = r0.length()     // Catch:{ all -> 0x00b5 }
                int r1 = r1 + -1
                r14 = r11 & r1
                java.lang.Object r1 = r0.get(r14)     // Catch:{ all -> 0x00b5 }
                r2 = r1
                com.google.common.cache.LocalCache$ReferenceEntry r2 = (com.google.common.cache.LocalCache.ReferenceEntry) r2     // Catch:{ all -> 0x00b5 }
                r1 = r2
            L_0x0027:
                r15 = r1
                r8 = 0
                if (r15 == 0) goto L_0x00b4
                java.lang.Object r1 = r15.getKey()     // Catch:{ all -> 0x00b5 }
                r7 = r1
                int r1 = r15.getHash()     // Catch:{ all -> 0x00b5 }
                if (r1 != r11) goto L_0x00ae
                if (r7 == 0) goto L_0x00ae
                com.google.common.cache.LocalCache<K, V> r1 = r9.map     // Catch:{ all -> 0x00b5 }
                com.google.common.base.Equivalence<java.lang.Object> r1 = r1.keyEquivalence     // Catch:{ all -> 0x00b5 }
                boolean r1 = r1.equivalent(r10, r7)     // Catch:{ all -> 0x00b5 }
                if (r1 == 0) goto L_0x00ae
                com.google.common.cache.LocalCache$ValueReference r1 = r15.getValueReference()     // Catch:{ all -> 0x00b5 }
                r6 = r1
                java.lang.Object r1 = r6.get()     // Catch:{ all -> 0x00b5 }
                r16 = r1
                if (r16 != 0) goto L_0x0086
                boolean r1 = r6.isActive()     // Catch:{ all -> 0x00b5 }
                if (r1 == 0) goto L_0x007b
                int r1 = r9.count     // Catch:{ all -> 0x00b5 }
                int r17 = r1 + -1
                int r1 = r9.modCount     // Catch:{ all -> 0x00b5 }
                int r1 = r1 + 1
                r9.modCount = r1     // Catch:{ all -> 0x00b5 }
                com.google.common.cache.RemovalCause r18 = com.google.common.cache.RemovalCause.COLLECTED     // Catch:{ all -> 0x00b5 }
                r1 = r21
                r3 = r15
                r4 = r7
                r5 = r23
                r19 = r6
                r20 = r7
                r7 = r18
                com.google.common.cache.LocalCache$ReferenceEntry r1 = r1.removeValueFromChain(r2, r3, r4, r5, r6, r7)     // Catch:{ all -> 0x00b5 }
                int r3 = r9.count     // Catch:{ all -> 0x00b5 }
                int r3 = r3 + -1
                r0.set(r14, r1)     // Catch:{ all -> 0x00b5 }
                r9.count = r3     // Catch:{ all -> 0x00b5 }
                goto L_0x007f
            L_0x007b:
                r19 = r6
                r20 = r7
            L_0x007f:
                r21.unlock()
                r21.postWriteCleanup()
                return r8
            L_0x0086:
                r19 = r6
                r20 = r7
                int r1 = r9.modCount     // Catch:{ all -> 0x00b5 }
                int r1 = r1 + 1
                r9.modCount = r1     // Catch:{ all -> 0x00b5 }
                com.google.common.cache.RemovalCause r1 = com.google.common.cache.RemovalCause.REPLACED     // Catch:{ all -> 0x00b5 }
                r7 = r19
                r9.enqueueNotification(r10, r11, r7, r1)     // Catch:{ all -> 0x00b5 }
                r3 = r21
                r4 = r15
                r5 = r22
                r6 = r24
                r1 = r7
                r7 = r12
                r3.setValue(r4, r5, r6, r7)     // Catch:{ all -> 0x00b5 }
                r21.evictEntries()     // Catch:{ all -> 0x00b5 }
                r21.unlock()
                r21.postWriteCleanup()
                return r16
            L_0x00ae:
                com.google.common.cache.LocalCache$ReferenceEntry r1 = r15.getNext()     // Catch:{ all -> 0x00b5 }
                goto L_0x0027
            L_0x00b4:
                goto L_0x007f
            L_0x00b5:
                r0 = move-exception
                r21.unlock()
                r21.postWriteCleanup()
                throw r0
            */
            throw new UnsupportedOperationException("Method not decompiled: com.google.common.cache.LocalCache.Segment.replace(java.lang.Object, int, java.lang.Object):java.lang.Object");
        }

        /* access modifiers changed from: package-private */
        @Nullable
        public V remove(Object key, int hash) {
            RemovalCause removalCause;
            int i = hash;
            lock();
            try {
                preWriteCleanup(this.map.ticker.read());
                int i2 = this.count - 1;
                AtomicReferenceArray<ReferenceEntry<K, V>> table2 = this.table;
                int index = i & (table2.length() - 1);
                ReferenceEntry<K, V> first = table2.get(index);
                ReferenceEntry<K, V> e = first;
                while (true) {
                    ReferenceEntry<K, V> e2 = e;
                    if (e2 == null) {
                        break;
                    }
                    K entryKey = e2.getKey();
                    if (e2.getHash() != i || entryKey == null || !this.map.keyEquivalence.equivalent(key, entryKey)) {
                        e = e2.getNext();
                    } else {
                        ValueReference<K, V> valueReference = e2.getValueReference();
                        V entryValue = valueReference.get();
                        if (entryValue != null) {
                            removalCause = RemovalCause.EXPLICIT;
                        } else if (valueReference.isActive()) {
                            removalCause = RemovalCause.COLLECTED;
                        }
                        RemovalCause cause = removalCause;
                        this.modCount++;
                        table2.set(index, removeValueFromChain(first, e2, entryKey, hash, valueReference, cause));
                        this.count--;
                        return entryValue;
                    }
                }
                unlock();
                postWriteCleanup();
                return null;
            } finally {
                unlock();
                postWriteCleanup();
            }
        }

        /* access modifiers changed from: package-private */
        /* JADX WARNING: Code restructure failed: missing block: B:10:0x0048, code lost:
            r4 = r16.getValueReference();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:11:0x0051, code lost:
            r17 = r4.get();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:12:0x0053, code lost:
            if (r17 == null) goto L_0x0072;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:13:0x0055, code lost:
            if (r10 != r4) goto L_0x0058;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:18:?, code lost:
            enqueueNotification(r8, r9, new com.google.common.cache.LocalCache.WeightedStrongValueReference<>(r25, 0), com.google.common.cache.RemovalCause.REPLACED);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:19:0x0065, code lost:
            unlock();
            postWriteCleanup();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:20:0x006c, code lost:
            return false;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:21:0x006d, code lost:
            r0 = th;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:22:0x006e, code lost:
            r3 = r25;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:23:0x0072, code lost:
            r3 = r25;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:25:?, code lost:
            r7.modCount++;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:26:0x007d, code lost:
            if (r24.isActive() == false) goto L_0x008b;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:27:0x007f, code lost:
            if (r17 != null) goto L_0x0084;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:28:0x0081, code lost:
            r1 = com.google.common.cache.RemovalCause.COLLECTED;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:29:0x0084, code lost:
            r1 = com.google.common.cache.RemovalCause.REPLACED;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:30:0x0086, code lost:
            enqueueNotification(r8, r9, r10, r1);
            r0 = r0 - 1;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:31:0x008b, code lost:
            r18 = r4;
            r19 = r5;
            r20 = r6;
            setValue(r16, r22, r25, r11);
            r7.count = r0;
            evictEntries();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:32:0x00a2, code lost:
            unlock();
            postWriteCleanup();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:33:0x00a9, code lost:
            return true;
         */
        /* JADX WARNING: Multi-variable type inference failed */
        /* Code decompiled incorrectly, please refer to instructions dump. */
        public boolean storeLoadedValue(K r22, int r23, com.google.common.cache.LocalCache.LoadingValueReference<K, V> r24, V r25) {
            /*
                r21 = this;
                r7 = r21
                r8 = r22
                r9 = r23
                r10 = r24
                r21.lock()
                com.google.common.cache.LocalCache<K, V> r0 = r7.map     // Catch:{ all -> 0x00e1 }
                com.google.common.base.Ticker r0 = r0.ticker     // Catch:{ all -> 0x00e1 }
                long r0 = r0.read()     // Catch:{ all -> 0x00e1 }
                r11 = r0
                r7.preWriteCleanup(r11)     // Catch:{ all -> 0x00e1 }
                int r0 = r7.count     // Catch:{ all -> 0x00e1 }
                r13 = 1
                int r0 = r0 + r13
                java.util.concurrent.atomic.AtomicReferenceArray<com.google.common.cache.LocalCache$ReferenceEntry<K, V>> r1 = r7.table     // Catch:{ all -> 0x00e1 }
                r14 = r1
                int r1 = r14.length()     // Catch:{ all -> 0x00e1 }
                int r1 = r1 - r13
                r15 = r9 & r1
                java.lang.Object r1 = r14.get(r15)     // Catch:{ all -> 0x00e1 }
                com.google.common.cache.LocalCache$ReferenceEntry r1 = (com.google.common.cache.LocalCache.ReferenceEntry) r1     // Catch:{ all -> 0x00e1 }
                r5 = r1
            L_0x002d:
                r16 = r1
                if (r16 == 0) goto L_0x00b4
                java.lang.Object r1 = r16.getKey()     // Catch:{ all -> 0x00e1 }
                r6 = r1
                int r1 = r16.getHash()     // Catch:{ all -> 0x00e1 }
                if (r1 != r9) goto L_0x00aa
                if (r6 == 0) goto L_0x00aa
                com.google.common.cache.LocalCache<K, V> r1 = r7.map     // Catch:{ all -> 0x00e1 }
                com.google.common.base.Equivalence<java.lang.Object> r1 = r1.keyEquivalence     // Catch:{ all -> 0x00e1 }
                boolean r1 = r1.equivalent(r8, r6)     // Catch:{ all -> 0x00e1 }
                if (r1 == 0) goto L_0x00aa
                com.google.common.cache.LocalCache$ValueReference r1 = r16.getValueReference()     // Catch:{ all -> 0x00e1 }
                r4 = r1
                java.lang.Object r1 = r4.get()     // Catch:{ all -> 0x00e1 }
                r17 = r1
                if (r17 == 0) goto L_0x0072
                if (r10 != r4) goto L_0x0058
                goto L_0x0072
            L_0x0058:
                com.google.common.cache.LocalCache$WeightedStrongValueReference r1 = new com.google.common.cache.LocalCache$WeightedStrongValueReference     // Catch:{ all -> 0x006d }
                r2 = 0
                r3 = r25
                r1.<init>(r3, r2)     // Catch:{ all -> 0x00e1 }
                com.google.common.cache.RemovalCause r4 = com.google.common.cache.RemovalCause.REPLACED     // Catch:{ all -> 0x00e1 }
                r7.enqueueNotification(r8, r9, r1, r4)     // Catch:{ all -> 0x00e1 }
                r21.unlock()
                r21.postWriteCleanup()
                return r2
            L_0x006d:
                r0 = move-exception
                r3 = r25
                goto L_0x00e2
            L_0x0072:
                r3 = r25
                int r1 = r7.modCount     // Catch:{ all -> 0x00e1 }
                int r1 = r1 + r13
                r7.modCount = r1     // Catch:{ all -> 0x00e1 }
                boolean r1 = r24.isActive()     // Catch:{ all -> 0x00e1 }
                if (r1 == 0) goto L_0x008b
                if (r17 != 0) goto L_0x0084
                com.google.common.cache.RemovalCause r1 = com.google.common.cache.RemovalCause.COLLECTED     // Catch:{ all -> 0x00e1 }
                goto L_0x0086
            L_0x0084:
                com.google.common.cache.RemovalCause r1 = com.google.common.cache.RemovalCause.REPLACED     // Catch:{ all -> 0x00e1 }
            L_0x0086:
                r7.enqueueNotification(r8, r9, r10, r1)     // Catch:{ all -> 0x00e1 }
                int r0 = r0 + -1
            L_0x008b:
                r1 = r21
                r2 = r16
                r3 = r22
                r18 = r4
                r4 = r25
                r19 = r5
                r20 = r6
                r5 = r11
                r1.setValue(r2, r3, r4, r5)     // Catch:{ all -> 0x00e1 }
                r7.count = r0     // Catch:{ all -> 0x00e1 }
                r21.evictEntries()     // Catch:{ all -> 0x00e1 }
                r21.unlock()
                r21.postWriteCleanup()
                return r13
            L_0x00aa:
                r19 = r5
                com.google.common.cache.LocalCache$ReferenceEntry r1 = r16.getNext()     // Catch:{ all -> 0x00e1 }
                r5 = r19
                goto L_0x002d
            L_0x00b4:
                r19 = r5
                int r1 = r7.modCount     // Catch:{ all -> 0x00e1 }
                int r1 = r1 + r13
                r7.modCount = r1     // Catch:{ all -> 0x00e1 }
                r5 = r19
                com.google.common.cache.LocalCache$ReferenceEntry r1 = r7.newEntry(r8, r9, r5)     // Catch:{ all -> 0x00e1 }
                r6 = r1
                r1 = r21
                r2 = r6
                r3 = r22
                r4 = r25
                r16 = r5
                r13 = r6
                r5 = r11
                r1.setValue(r2, r3, r4, r5)     // Catch:{ all -> 0x00e1 }
                r14.set(r15, r13)     // Catch:{ all -> 0x00e1 }
                r7.count = r0     // Catch:{ all -> 0x00e1 }
                r21.evictEntries()     // Catch:{ all -> 0x00e1 }
                r21.unlock()
                r21.postWriteCleanup()
                r1 = 1
                return r1
            L_0x00e1:
                r0 = move-exception
            L_0x00e2:
                r21.unlock()
                r21.postWriteCleanup()
                throw r0
            */
            throw new UnsupportedOperationException("Method not decompiled: com.google.common.cache.LocalCache.Segment.storeLoadedValue(java.lang.Object, int, com.google.common.cache.LocalCache$LoadingValueReference, java.lang.Object):boolean");
        }

        /* access modifiers changed from: package-private */
        public boolean remove(Object key, int hash, Object value) {
            boolean z;
            K cause;
            int i = hash;
            lock();
            try {
                preWriteCleanup(this.map.ticker.read());
                int i2 = this.count - 1;
                AtomicReferenceArray<ReferenceEntry<K, V>> table2 = this.table;
                int index = i & (table2.length() - 1);
                ReferenceEntry<K, V> first = table2.get(index);
                ReferenceEntry<K, V> e = first;
                while (true) {
                    ReferenceEntry<K, V> e2 = e;
                    z = false;
                    if (e2 == null) {
                        break;
                    }
                    K entryKey = e2.getKey();
                    if (e2.getHash() != i || entryKey == null || !this.map.keyEquivalence.equivalent(key, entryKey)) {
                        e = e2.getNext();
                    } else {
                        ValueReference<K, V> valueReference = e2.getValueReference();
                        V entryValue = valueReference.get();
                        if (this.map.valueEquivalence.equivalent(value, entryValue)) {
                            cause = RemovalCause.EXPLICIT;
                        } else if (entryValue != null || !valueReference.isActive()) {
                            V v = entryValue;
                            K k = entryKey;
                        } else {
                            cause = RemovalCause.COLLECTED;
                        }
                        this.modCount++;
                        K cause2 = cause;
                        V v2 = entryValue;
                        K k2 = entryKey;
                        table2.set(index, removeValueFromChain(first, e2, entryKey, hash, valueReference, cause2));
                        this.count--;
                        if (cause2 == RemovalCause.EXPLICIT) {
                            z = true;
                        }
                    }
                }
                return z;
            } finally {
                unlock();
                postWriteCleanup();
            }
        }

        /* access modifiers changed from: package-private */
        public void clear() {
            if (this.count != 0) {
                lock();
                try {
                    AtomicReferenceArray<ReferenceEntry<K, V>> table2 = this.table;
                    for (int i = 0; i < table2.length(); i++) {
                        for (ReferenceEntry<K, V> e = table2.get(i); e != null; e = e.getNext()) {
                            if (e.getValueReference().isActive()) {
                                enqueueNotification(e, RemovalCause.EXPLICIT);
                            }
                        }
                    }
                    for (int i2 = 0; i2 < table2.length(); i2++) {
                        table2.set(i2, (Object) null);
                    }
                    clearReferenceQueues();
                    this.writeQueue.clear();
                    this.accessQueue.clear();
                    this.readCount.set(0);
                    this.modCount++;
                    this.count = 0;
                } finally {
                    unlock();
                    postWriteCleanup();
                }
            }
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        @Nullable
        public ReferenceEntry<K, V> removeValueFromChain(ReferenceEntry<K, V> first, ReferenceEntry<K, V> entry, @Nullable K key, int hash, ValueReference<K, V> valueReference, RemovalCause cause) {
            enqueueNotification(key, hash, valueReference, cause);
            this.writeQueue.remove(entry);
            this.accessQueue.remove(entry);
            if (!valueReference.isLoading()) {
                return removeEntryFromChain(first, entry);
            }
            valueReference.notifyNewValue(null);
            return first;
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        @Nullable
        public ReferenceEntry<K, V> removeEntryFromChain(ReferenceEntry<K, V> first, ReferenceEntry<K, V> entry) {
            int newCount = this.count;
            ReferenceEntry<K, V> newFirst = entry.getNext();
            int newCount2 = newCount;
            for (ReferenceEntry<K, V> e = first; e != entry; e = e.getNext()) {
                ReferenceEntry<K, V> next = copyEntry(e, newFirst);
                if (next != null) {
                    newFirst = next;
                } else {
                    removeCollectedEntry(e);
                    newCount2--;
                }
            }
            this.count = newCount2;
            return newFirst;
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public void removeCollectedEntry(ReferenceEntry<K, V> entry) {
            enqueueNotification(entry, RemovalCause.COLLECTED);
            this.writeQueue.remove(entry);
            this.accessQueue.remove(entry);
        }

        /* access modifiers changed from: package-private */
        public boolean reclaimKey(ReferenceEntry<K, V> entry, int hash) {
            lock();
            try {
                boolean z = true;
                int i = this.count - 1;
                AtomicReferenceArray<ReferenceEntry<K, V>> table2 = this.table;
                int index = (table2.length() - 1) & hash;
                ReferenceEntry<K, V> first = table2.get(index);
                ReferenceEntry<K, V> e = first;
                while (true) {
                    if (e == null) {
                        z = false;
                        break;
                    } else if (e == entry) {
                        this.modCount++;
                        table2.set(index, removeValueFromChain(first, e, e.getKey(), hash, e.getValueReference(), RemovalCause.COLLECTED));
                        this.count--;
                        break;
                    } else {
                        e = e.getNext();
                    }
                }
                return z;
            } finally {
                unlock();
                postWriteCleanup();
            }
        }

        /* access modifiers changed from: package-private */
        /* JADX WARNING: Removed duplicated region for block: B:36:0x00a3  */
        /* Code decompiled incorrectly, please refer to instructions dump. */
        public boolean reclaimValue(K r19, int r20, com.google.common.cache.LocalCache.ValueReference<K, V> r21) {
            /*
                r18 = this;
                r8 = r18
                r9 = r20
                r18.lock()
                int r0 = r8.count     // Catch:{ all -> 0x0097 }
                r10 = 1
                int r0 = r0 - r10
                java.util.concurrent.atomic.AtomicReferenceArray<com.google.common.cache.LocalCache$ReferenceEntry<K, V>> r1 = r8.table     // Catch:{ all -> 0x0097 }
                r11 = r1
                int r1 = r11.length()     // Catch:{ all -> 0x0097 }
                int r1 = r1 - r10
                r12 = r9 & r1
                java.lang.Object r1 = r11.get(r12)     // Catch:{ all -> 0x0097 }
                r2 = r1
                com.google.common.cache.LocalCache$ReferenceEntry r2 = (com.google.common.cache.LocalCache.ReferenceEntry) r2     // Catch:{ all -> 0x0097 }
                r1 = r2
            L_0x001d:
                r13 = r1
                r1 = 0
                if (r13 == 0) goto L_0x0088
                java.lang.Object r3 = r13.getKey()     // Catch:{ all -> 0x0097 }
                r14 = r3
                int r3 = r13.getHash()     // Catch:{ all -> 0x0097 }
                if (r3 != r9) goto L_0x007f
                if (r14 == 0) goto L_0x007f
                com.google.common.cache.LocalCache<K, V> r3 = r8.map     // Catch:{ all -> 0x0097 }
                com.google.common.base.Equivalence<java.lang.Object> r3 = r3.keyEquivalence     // Catch:{ all -> 0x0097 }
                r15 = r19
                boolean r3 = r3.equivalent(r15, r14)     // Catch:{ all -> 0x0086 }
                if (r3 == 0) goto L_0x0081
                com.google.common.cache.LocalCache$ValueReference r3 = r13.getValueReference()     // Catch:{ all -> 0x0086 }
                r7 = r3
                r6 = r21
                if (r7 != r6) goto L_0x0070
                int r1 = r8.modCount     // Catch:{ all -> 0x0086 }
                int r1 = r1 + r10
                r8.modCount = r1     // Catch:{ all -> 0x0086 }
                com.google.common.cache.RemovalCause r16 = com.google.common.cache.RemovalCause.COLLECTED     // Catch:{ all -> 0x0086 }
                r1 = r18
                r3 = r13
                r4 = r14
                r5 = r20
                r6 = r21
                r17 = r7
                r7 = r16
                com.google.common.cache.LocalCache$ReferenceEntry r1 = r1.removeValueFromChain(r2, r3, r4, r5, r6, r7)     // Catch:{ all -> 0x0086 }
                int r3 = r8.count     // Catch:{ all -> 0x0086 }
                int r3 = r3 - r10
                r11.set(r12, r1)     // Catch:{ all -> 0x0086 }
                r8.count = r3     // Catch:{ all -> 0x0086 }
                r18.unlock()
                boolean r0 = r18.isHeldByCurrentThread()
                if (r0 != 0) goto L_0x006f
                r18.postWriteCleanup()
            L_0x006f:
                return r10
            L_0x0070:
                r17 = r7
                r18.unlock()
                boolean r3 = r18.isHeldByCurrentThread()
                if (r3 != 0) goto L_0x007e
                r18.postWriteCleanup()
            L_0x007e:
                return r1
            L_0x007f:
                r15 = r19
            L_0x0081:
                com.google.common.cache.LocalCache$ReferenceEntry r1 = r13.getNext()     // Catch:{ all -> 0x0086 }
                goto L_0x001d
            L_0x0086:
                r0 = move-exception
                goto L_0x009a
            L_0x0088:
                r15 = r19
                r18.unlock()
                boolean r3 = r18.isHeldByCurrentThread()
                if (r3 != 0) goto L_0x0096
                r18.postWriteCleanup()
            L_0x0096:
                return r1
            L_0x0097:
                r0 = move-exception
                r15 = r19
            L_0x009a:
                r18.unlock()
                boolean r1 = r18.isHeldByCurrentThread()
                if (r1 != 0) goto L_0x00a6
                r18.postWriteCleanup()
            L_0x00a6:
                throw r0
            */
            throw new UnsupportedOperationException("Method not decompiled: com.google.common.cache.LocalCache.Segment.reclaimValue(java.lang.Object, int, com.google.common.cache.LocalCache$ValueReference):boolean");
        }

        /* access modifiers changed from: package-private */
        public boolean removeLoadingValue(K key, int hash, LoadingValueReference<K, V> valueReference) {
            lock();
            try {
                AtomicReferenceArray<ReferenceEntry<K, V>> table2 = this.table;
                int index = (table2.length() - 1) & hash;
                ReferenceEntry<K, V> first = table2.get(index);
                ReferenceEntry<K, V> e = first;
                while (true) {
                    if (e == null) {
                        break;
                    }
                    K entryKey = e.getKey();
                    if (e.getHash() != hash || entryKey == null || !this.map.keyEquivalence.equivalent(key, entryKey)) {
                        e = e.getNext();
                    } else if (e.getValueReference() == valueReference) {
                        if (valueReference.isActive()) {
                            e.setValueReference(valueReference.getOldValue());
                        } else {
                            table2.set(index, removeEntryFromChain(first, e));
                        }
                        return true;
                    }
                }
                unlock();
                postWriteCleanup();
                return false;
            } finally {
                unlock();
                postWriteCleanup();
            }
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public boolean removeEntry(ReferenceEntry<K, V> entry, int hash, RemovalCause cause) {
            int i = this.count - 1;
            AtomicReferenceArray<ReferenceEntry<K, V>> table2 = this.table;
            int index = hash & (table2.length() - 1);
            ReferenceEntry<K, V> first = table2.get(index);
            ReferenceEntry<K, V> newFirst = first;
            while (true) {
                ReferenceEntry<K, V> e = newFirst;
                if (e == null) {
                    ReferenceEntry<K, V> referenceEntry = entry;
                    return false;
                } else if (e == entry) {
                    this.modCount++;
                    table2.set(index, removeValueFromChain(first, e, e.getKey(), hash, e.getValueReference(), cause));
                    this.count--;
                    return true;
                } else {
                    newFirst = e.getNext();
                }
            }
        }

        /* access modifiers changed from: package-private */
        public void postReadCleanup() {
            if ((this.readCount.incrementAndGet() & 63) == 0) {
                cleanUp();
            }
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public void preWriteCleanup(long now) {
            runLockedCleanup(now);
        }

        /* access modifiers changed from: package-private */
        public void postWriteCleanup() {
            runUnlockedCleanup();
        }

        /* access modifiers changed from: package-private */
        public void cleanUp() {
            runLockedCleanup(this.map.ticker.read());
            runUnlockedCleanup();
        }

        /* access modifiers changed from: package-private */
        public void runLockedCleanup(long now) {
            if (tryLock()) {
                try {
                    drainReferenceQueues();
                    expireEntries(now);
                    this.readCount.set(0);
                } finally {
                    unlock();
                }
            }
        }

        /* access modifiers changed from: package-private */
        public void runUnlockedCleanup() {
            if (!isHeldByCurrentThread()) {
                this.map.processPendingNotifications();
            }
        }
    }

    static class LoadingValueReference<K, V> implements ValueReference<K, V> {
        final SettableFuture<V> futureValue;
        volatile ValueReference<K, V> oldValue;
        final Stopwatch stopwatch;

        public LoadingValueReference() {
            this(LocalCache.unset());
        }

        public LoadingValueReference(ValueReference<K, V> oldValue2) {
            this.futureValue = SettableFuture.create();
            this.stopwatch = new Stopwatch();
            this.oldValue = oldValue2;
        }

        public boolean isLoading() {
            return true;
        }

        public boolean isActive() {
            return this.oldValue.isActive();
        }

        public int getWeight() {
            return this.oldValue.getWeight();
        }

        public boolean set(@Nullable V newValue) {
            return this.futureValue.set(newValue);
        }

        public boolean setException(Throwable t) {
            return setException(this.futureValue, t);
        }

        private static boolean setException(SettableFuture<?> future, Throwable t) {
            try {
                return future.setException(t);
            } catch (Error e) {
                return false;
            }
        }

        private ListenableFuture<V> fullyFailedFuture(Throwable t) {
            SettableFuture<V> future = SettableFuture.create();
            setException(future, t);
            return future;
        }

        public void notifyNewValue(@Nullable V newValue) {
            if (newValue != null) {
                set(newValue);
            } else {
                this.oldValue = LocalCache.unset();
            }
        }

        public ListenableFuture<V> loadFuture(K key, CacheLoader<? super K, V> loader) {
            this.stopwatch.start();
            V previousValue = this.oldValue.get();
            if (previousValue == null) {
                try {
                    V newValue = loader.load(key);
                    return set(newValue) ? this.futureValue : Futures.immediateFuture(newValue);
                } catch (Throwable t) {
                    if (t instanceof InterruptedException) {
                        Thread.currentThread().interrupt();
                    }
                    return setException(t) ? this.futureValue : fullyFailedFuture(t);
                }
            } else {
                ListenableFuture<V> newValue2 = loader.reload(key, previousValue);
                return newValue2 != null ? newValue2 : Futures.immediateFuture(null);
            }
        }

        public long elapsedNanos() {
            return this.stopwatch.elapsedTime(TimeUnit.NANOSECONDS);
        }

        public V waitForValue() throws ExecutionException {
            return Uninterruptibles.getUninterruptibly(this.futureValue);
        }

        public V get() {
            return this.oldValue.get();
        }

        public ValueReference<K, V> getOldValue() {
            return this.oldValue;
        }

        public ReferenceEntry<K, V> getEntry() {
            return null;
        }

        public ValueReference<K, V> copyFor(ReferenceQueue<V> referenceQueue, V v, ReferenceEntry<K, V> referenceEntry) {
            return this;
        }
    }

    static final class WriteQueue<K, V> extends AbstractQueue<ReferenceEntry<K, V>> {
        final ReferenceEntry<K, V> head = new AbstractReferenceEntry<K, V>() {
            ReferenceEntry<K, V> nextWrite = this;
            ReferenceEntry<K, V> previousWrite = this;

            public long getWriteTime() {
                return Long.MAX_VALUE;
            }

            public void setWriteTime(long time) {
            }

            public ReferenceEntry<K, V> getNextInWriteQueue() {
                return this.nextWrite;
            }

            public void setNextInWriteQueue(ReferenceEntry<K, V> next) {
                this.nextWrite = next;
            }

            public ReferenceEntry<K, V> getPreviousInWriteQueue() {
                return this.previousWrite;
            }

            public void setPreviousInWriteQueue(ReferenceEntry<K, V> previous) {
                this.previousWrite = previous;
            }
        };

        WriteQueue() {
        }

        public boolean offer(ReferenceEntry<K, V> entry) {
            LocalCache.connectWriteOrder(entry.getPreviousInWriteQueue(), entry.getNextInWriteQueue());
            LocalCache.connectWriteOrder(this.head.getPreviousInWriteQueue(), entry);
            LocalCache.connectWriteOrder(entry, this.head);
            return true;
        }

        public ReferenceEntry<K, V> peek() {
            ReferenceEntry<K, V> next = this.head.getNextInWriteQueue();
            if (next == this.head) {
                return null;
            }
            return next;
        }

        public ReferenceEntry<K, V> poll() {
            ReferenceEntry<K, V> next = this.head.getNextInWriteQueue();
            if (next == this.head) {
                return null;
            }
            remove(next);
            return next;
        }

        public boolean remove(Object o) {
            ReferenceEntry<K, V> e = (ReferenceEntry) o;
            ReferenceEntry<K, V> previous = e.getPreviousInWriteQueue();
            ReferenceEntry<K, V> next = e.getNextInWriteQueue();
            LocalCache.connectWriteOrder(previous, next);
            LocalCache.nullifyWriteOrder(e);
            return next != NullEntry.INSTANCE;
        }

        public boolean contains(Object o) {
            return ((ReferenceEntry) o).getNextInWriteQueue() != NullEntry.INSTANCE;
        }

        public boolean isEmpty() {
            return this.head.getNextInWriteQueue() == this.head;
        }

        public int size() {
            int size = 0;
            for (ReferenceEntry<K, V> e = this.head.getNextInWriteQueue(); e != this.head; e = e.getNextInWriteQueue()) {
                size++;
            }
            return size;
        }

        public void clear() {
            ReferenceEntry<K, V> e = this.head.getNextInWriteQueue();
            while (e != this.head) {
                ReferenceEntry<K, V> next = e.getNextInWriteQueue();
                LocalCache.nullifyWriteOrder(e);
                e = next;
            }
            this.head.setNextInWriteQueue(this.head);
            this.head.setPreviousInWriteQueue(this.head);
        }

        public Iterator<ReferenceEntry<K, V>> iterator() {
            return new AbstractSequentialIterator<ReferenceEntry<K, V>>(peek()) {
                /* access modifiers changed from: protected */
                public ReferenceEntry<K, V> computeNext(ReferenceEntry<K, V> previous) {
                    ReferenceEntry<K, V> next = previous.getNextInWriteQueue();
                    if (next == WriteQueue.this.head) {
                        return null;
                    }
                    return next;
                }
            };
        }
    }

    static final class AccessQueue<K, V> extends AbstractQueue<ReferenceEntry<K, V>> {
        final ReferenceEntry<K, V> head = new AbstractReferenceEntry<K, V>() {
            ReferenceEntry<K, V> nextAccess = this;
            ReferenceEntry<K, V> previousAccess = this;

            public long getAccessTime() {
                return Long.MAX_VALUE;
            }

            public void setAccessTime(long time) {
            }

            public ReferenceEntry<K, V> getNextInAccessQueue() {
                return this.nextAccess;
            }

            public void setNextInAccessQueue(ReferenceEntry<K, V> next) {
                this.nextAccess = next;
            }

            public ReferenceEntry<K, V> getPreviousInAccessQueue() {
                return this.previousAccess;
            }

            public void setPreviousInAccessQueue(ReferenceEntry<K, V> previous) {
                this.previousAccess = previous;
            }
        };

        AccessQueue() {
        }

        public boolean offer(ReferenceEntry<K, V> entry) {
            LocalCache.connectAccessOrder(entry.getPreviousInAccessQueue(), entry.getNextInAccessQueue());
            LocalCache.connectAccessOrder(this.head.getPreviousInAccessQueue(), entry);
            LocalCache.connectAccessOrder(entry, this.head);
            return true;
        }

        public ReferenceEntry<K, V> peek() {
            ReferenceEntry<K, V> next = this.head.getNextInAccessQueue();
            if (next == this.head) {
                return null;
            }
            return next;
        }

        public ReferenceEntry<K, V> poll() {
            ReferenceEntry<K, V> next = this.head.getNextInAccessQueue();
            if (next == this.head) {
                return null;
            }
            remove(next);
            return next;
        }

        public boolean remove(Object o) {
            ReferenceEntry<K, V> e = (ReferenceEntry) o;
            ReferenceEntry<K, V> previous = e.getPreviousInAccessQueue();
            ReferenceEntry<K, V> next = e.getNextInAccessQueue();
            LocalCache.connectAccessOrder(previous, next);
            LocalCache.nullifyAccessOrder(e);
            return next != NullEntry.INSTANCE;
        }

        public boolean contains(Object o) {
            return ((ReferenceEntry) o).getNextInAccessQueue() != NullEntry.INSTANCE;
        }

        public boolean isEmpty() {
            return this.head.getNextInAccessQueue() == this.head;
        }

        public int size() {
            int size = 0;
            for (ReferenceEntry<K, V> e = this.head.getNextInAccessQueue(); e != this.head; e = e.getNextInAccessQueue()) {
                size++;
            }
            return size;
        }

        public void clear() {
            ReferenceEntry<K, V> e = this.head.getNextInAccessQueue();
            while (e != this.head) {
                ReferenceEntry<K, V> next = e.getNextInAccessQueue();
                LocalCache.nullifyAccessOrder(e);
                e = next;
            }
            this.head.setNextInAccessQueue(this.head);
            this.head.setPreviousInAccessQueue(this.head);
        }

        public Iterator<ReferenceEntry<K, V>> iterator() {
            return new AbstractSequentialIterator<ReferenceEntry<K, V>>(peek()) {
                /* access modifiers changed from: protected */
                public ReferenceEntry<K, V> computeNext(ReferenceEntry<K, V> previous) {
                    ReferenceEntry<K, V> next = previous.getNextInAccessQueue();
                    if (next == AccessQueue.this.head) {
                        return null;
                    }
                    return next;
                }
            };
        }
    }

    public void cleanUp() {
        for (Segment cleanUp : this.segments) {
            cleanUp.cleanUp();
        }
    }

    public boolean isEmpty() {
        Segment<K, V>[] segments2 = this.segments;
        long sum = 0;
        for (int i = 0; i < segments2.length; i++) {
            if (segments2[i].count != 0) {
                return false;
            }
            sum += (long) segments2[i].modCount;
        }
        if (sum == 0) {
            return true;
        }
        long sum2 = sum;
        for (int i2 = 0; i2 < segments2.length; i2++) {
            if (segments2[i2].count != 0) {
                return false;
            }
            sum2 -= (long) segments2[i2].modCount;
        }
        if (sum2 != 0) {
            return false;
        }
        long j = sum2;
        return true;
    }

    /* access modifiers changed from: package-private */
    public long longSize() {
        Segment<K, V>[] segments2 = this.segments;
        long sum = 0;
        for (Segment<K, V> segment : segments2) {
            sum += (long) segment.count;
        }
        return sum;
    }

    public int size() {
        return Ints.saturatedCast(longSize());
    }

    @Nullable
    public V get(@Nullable Object key) {
        if (key == null) {
            return null;
        }
        int hash = hash(key);
        return segmentFor(hash).get(key, hash);
    }

    @Nullable
    public V getIfPresent(Object key) {
        int hash = hash(Preconditions.checkNotNull(key));
        V value = segmentFor(hash).get(key, hash);
        if (value == null) {
            this.globalStatsCounter.recordMisses(1);
        } else {
            this.globalStatsCounter.recordHits(1);
        }
        return value;
    }

    /* access modifiers changed from: package-private */
    public V get(K key, CacheLoader<? super K, V> loader) throws ExecutionException {
        int hash = hash(Preconditions.checkNotNull(key));
        return segmentFor(hash).get(key, hash, loader);
    }

    /* access modifiers changed from: package-private */
    public V getOrLoad(K key) throws ExecutionException {
        return get(key, this.defaultLoader);
    }

    /* access modifiers changed from: package-private */
    public ImmutableMap<K, V> getAllPresent(Iterable<?> keys) {
        int hits = 0;
        int misses = 0;
        Map<K, V> result = Maps.newLinkedHashMap();
        for (Object next : keys) {
            V value = get(next);
            if (value == null) {
                misses++;
            } else {
                result.put(next, value);
                hits++;
            }
        }
        this.globalStatsCounter.recordHits(hits);
        this.globalStatsCounter.recordMisses(misses);
        return ImmutableMap.copyOf(result);
    }

    /* access modifiers changed from: package-private */
    public ImmutableMap<K, V> getAll(Iterable<? extends K> keys) throws ExecutionException {
        int hits = 0;
        int misses = 0;
        Map<K, V> result = Maps.newLinkedHashMap();
        Set<K> keysToLoad = Sets.newLinkedHashSet();
        for (K key : keys) {
            V value = get(key);
            if (!result.containsKey(key)) {
                result.put(key, value);
                if (value == null) {
                    misses++;
                    keysToLoad.add(key);
                } else {
                    hits++;
                }
            }
        }
        try {
            if (!keysToLoad.isEmpty()) {
                Map<K, V> newEntries = loadAll(keysToLoad, this.defaultLoader);
                for (K key2 : keysToLoad) {
                    V value2 = newEntries.get(key2);
                    if (value2 != null) {
                        result.put(key2, value2);
                    } else {
                        throw new CacheLoader.InvalidCacheLoadException("loadAll failed to return a value for " + key2);
                    }
                }
            }
        } catch (CacheLoader.UnsupportedLoadingOperationException e) {
            for (K key3 : keysToLoad) {
                misses--;
                result.put(key3, get(key3, this.defaultLoader));
            }
        } catch (Throwable th) {
            this.globalStatsCounter.recordHits(hits);
            this.globalStatsCounter.recordMisses(misses);
            throw th;
        }
        ImmutableMap<K, V> copyOf = ImmutableMap.copyOf(result);
        this.globalStatsCounter.recordHits(hits);
        this.globalStatsCounter.recordMisses(misses);
        return copyOf;
    }

    /* access modifiers changed from: package-private */
    @Nullable
    public Map<K, V> loadAll(Set<? extends K> keys, CacheLoader<? super K, V> loader) throws ExecutionException {
        Stopwatch stopwatch = new Stopwatch().start();
        try {
            Map<? super K, V> loadAll = loader.loadAll(keys);
            if (1 == 0) {
                this.globalStatsCounter.recordLoadException(stopwatch.elapsedTime(TimeUnit.NANOSECONDS));
            }
            if (loadAll != null) {
                stopwatch.stop();
                boolean nullsPresent = false;
                for (Map.Entry<K, V> entry : loadAll.entrySet()) {
                    K key = entry.getKey();
                    V value = entry.getValue();
                    if (key == null || value == null) {
                        nullsPresent = true;
                    } else {
                        put(key, value);
                    }
                }
                if (!nullsPresent) {
                    this.globalStatsCounter.recordLoadSuccess(stopwatch.elapsedTime(TimeUnit.NANOSECONDS));
                    return loadAll;
                }
                this.globalStatsCounter.recordLoadException(stopwatch.elapsedTime(TimeUnit.NANOSECONDS));
                throw new CacheLoader.InvalidCacheLoadException(loader + " returned null keys or values from loadAll");
            }
            this.globalStatsCounter.recordLoadException(stopwatch.elapsedTime(TimeUnit.NANOSECONDS));
            throw new CacheLoader.InvalidCacheLoadException(loader + " returned null map from loadAll");
        } catch (CacheLoader.UnsupportedLoadingOperationException e) {
            throw e;
        } catch (InterruptedException e2) {
            Thread.currentThread().interrupt();
            throw new ExecutionException(e2);
        } catch (RuntimeException e3) {
            throw new UncheckedExecutionException((Throwable) e3);
        } catch (Exception e4) {
            throw new ExecutionException(e4);
        } catch (Error e5) {
            throw new ExecutionError(e5);
        } catch (Throwable th) {
            if (0 == 0) {
                this.globalStatsCounter.recordLoadException(stopwatch.elapsedTime(TimeUnit.NANOSECONDS));
            }
            throw th;
        }
    }

    /* access modifiers changed from: package-private */
    public ReferenceEntry<K, V> getEntry(@Nullable Object key) {
        if (key == null) {
            return null;
        }
        int hash = hash(key);
        return segmentFor(hash).getEntry(key, hash);
    }

    /* access modifiers changed from: package-private */
    public ReferenceEntry<K, V> getLiveEntry(@Nullable Object key) {
        if (key == null) {
            return null;
        }
        int hash = hash(key);
        return segmentFor(hash).getLiveEntry(key, hash, this.ticker.read());
    }

    /* access modifiers changed from: package-private */
    public void refresh(K key) {
        int hash = hash(Preconditions.checkNotNull(key));
        segmentFor(hash).refresh(key, hash, this.defaultLoader);
    }

    public boolean containsKey(@Nullable Object key) {
        if (key == null) {
            return false;
        }
        int hash = hash(key);
        return segmentFor(hash).containsKey(key, hash);
    }

    /* JADX WARNING: Code restructure failed: missing block: B:21:0x0067, code lost:
        r20 = r2;
        r13 = r13 + ((long) r10.modCount);
        r9 = r9 + 1;
        r5 = r18;
        r11 = r19;
        r3 = r3;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public boolean containsValue(@javax.annotation.Nullable java.lang.Object r24) {
        /*
            r23 = this;
            r0 = r23
            r1 = r24
            r2 = 0
            if (r1 != 0) goto L_0x0008
            return r2
        L_0x0008:
            com.google.common.base.Ticker r3 = r0.ticker
            long r3 = r3.read()
            com.google.common.cache.LocalCache$Segment<K, V>[] r5 = r0.segments
            r6 = -1
            r7 = r6
            r6 = 0
        L_0x0014:
            r9 = 3
            if (r6 >= r9) goto L_0x008b
            r9 = 0
            r11 = r5
            int r12 = r11.length
            r13 = r9
            r9 = 0
        L_0x001d:
            if (r9 >= r12) goto L_0x0079
            r10 = r11[r9]
            int r15 = r10.count
            java.util.concurrent.atomic.AtomicReferenceArray<com.google.common.cache.LocalCache$ReferenceEntry<K, V>> r2 = r10.table
            r16 = 0
        L_0x0027:
            r17 = r16
            r18 = r5
            int r5 = r2.length()
            r19 = r11
            r11 = r17
            if (r11 >= r5) goto L_0x0067
            java.lang.Object r5 = r2.get(r11)
            com.google.common.cache.LocalCache$ReferenceEntry r5 = (com.google.common.cache.LocalCache.ReferenceEntry) r5
        L_0x003b:
            if (r5 == 0) goto L_0x005c
            r20 = r2
            java.lang.Object r2 = r10.getLiveValue(r5, r3)
            if (r2 == 0) goto L_0x0051
            r21 = r3
            com.google.common.base.Equivalence<java.lang.Object> r3 = r0.valueEquivalence
            boolean r3 = r3.equivalent(r1, r2)
            if (r3 == 0) goto L_0x0053
            r3 = 1
            return r3
        L_0x0051:
            r21 = r3
        L_0x0053:
            com.google.common.cache.LocalCache$ReferenceEntry r5 = r5.getNext()
            r2 = r20
            r3 = r21
            goto L_0x003b
        L_0x005c:
            r20 = r2
            r21 = r3
            int r16 = r11 + 1
            r5 = r18
            r11 = r19
            goto L_0x0027
        L_0x0067:
            r20 = r2
            r21 = r3
            int r2 = r10.modCount
            long r2 = (long) r2
            long r13 = r13 + r2
            int r9 = r9 + 1
            r5 = r18
            r11 = r19
            r3 = r21
            r2 = 0
            goto L_0x001d
        L_0x0079:
            r21 = r3
            r18 = r5
            int r2 = (r13 > r7 ? 1 : (r13 == r7 ? 0 : -1))
            if (r2 != 0) goto L_0x0082
            goto L_0x008f
        L_0x0082:
            r7 = r13
            int r6 = r6 + 1
            r5 = r18
            r3 = r21
            r2 = 0
            goto L_0x0014
        L_0x008b:
            r21 = r3
            r18 = r5
        L_0x008f:
            r2 = 0
            return r2
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.common.cache.LocalCache.containsValue(java.lang.Object):boolean");
    }

    public V put(K key, V value) {
        Preconditions.checkNotNull(key);
        Preconditions.checkNotNull(value);
        int hash = hash(key);
        return segmentFor(hash).put(key, hash, value, false);
    }

    public V putIfAbsent(K key, V value) {
        Preconditions.checkNotNull(key);
        Preconditions.checkNotNull(value);
        int hash = hash(key);
        return segmentFor(hash).put(key, hash, value, true);
    }

    public void putAll(Map<? extends K, ? extends V> m) {
        for (Map.Entry<? extends K, ? extends V> e : m.entrySet()) {
            put(e.getKey(), e.getValue());
        }
    }

    public V remove(@Nullable Object key) {
        if (key == null) {
            return null;
        }
        int hash = hash(key);
        return segmentFor(hash).remove(key, hash);
    }

    public boolean remove(@Nullable Object key, @Nullable Object value) {
        if (key == null || value == null) {
            return false;
        }
        int hash = hash(key);
        return segmentFor(hash).remove(key, hash, value);
    }

    public boolean replace(K key, @Nullable V oldValue, V newValue) {
        Preconditions.checkNotNull(key);
        Preconditions.checkNotNull(newValue);
        if (oldValue == null) {
            return false;
        }
        int hash = hash(key);
        return segmentFor(hash).replace(key, hash, oldValue, newValue);
    }

    public V replace(K key, V value) {
        Preconditions.checkNotNull(key);
        Preconditions.checkNotNull(value);
        int hash = hash(key);
        return segmentFor(hash).replace(key, hash, value);
    }

    public void clear() {
        for (Segment<K, V> segment : this.segments) {
            segment.clear();
        }
    }

    /* access modifiers changed from: package-private */
    public void invalidateAll(Iterable<?> keys) {
        for (Object key : keys) {
            remove(key);
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

    abstract class HashIterator {
        Segment<K, V> currentSegment;
        AtomicReferenceArray<ReferenceEntry<K, V>> currentTable;
        LocalCache<K, V>.WriteThroughEntry lastReturned;
        ReferenceEntry<K, V> nextEntry;
        LocalCache<K, V>.WriteThroughEntry nextExternal;
        int nextSegmentIndex;
        int nextTableIndex = -1;

        HashIterator() {
            this.nextSegmentIndex = LocalCache.this.segments.length - 1;
            advance();
        }

        /* access modifiers changed from: package-private */
        public final void advance() {
            this.nextExternal = null;
            if (!nextInChain() && !nextInTable()) {
                while (this.nextSegmentIndex >= 0) {
                    Segment<K, V>[] segmentArr = LocalCache.this.segments;
                    int i = this.nextSegmentIndex;
                    this.nextSegmentIndex = i - 1;
                    this.currentSegment = segmentArr[i];
                    if (this.currentSegment.count != 0) {
                        this.currentTable = this.currentSegment.table;
                        this.nextTableIndex = this.currentTable.length() - 1;
                        if (nextInTable()) {
                            return;
                        }
                    }
                }
            }
        }

        /* access modifiers changed from: package-private */
        public boolean nextInChain() {
            if (this.nextEntry == null) {
                return false;
            }
            do {
                this.nextEntry = this.nextEntry.getNext();
                if (this.nextEntry == null) {
                    return false;
                }
            } while (!advanceTo(this.nextEntry));
            return true;
        }

        /* access modifiers changed from: package-private */
        public boolean nextInTable() {
            while (this.nextTableIndex >= 0) {
                AtomicReferenceArray<ReferenceEntry<K, V>> atomicReferenceArray = this.currentTable;
                int i = this.nextTableIndex;
                this.nextTableIndex = i - 1;
                ReferenceEntry<K, V> referenceEntry = atomicReferenceArray.get(i);
                this.nextEntry = referenceEntry;
                if (referenceEntry != null && (advanceTo(this.nextEntry) || nextInChain())) {
                    return true;
                }
            }
            return false;
        }

        /* access modifiers changed from: package-private */
        public boolean advanceTo(ReferenceEntry<K, V> entry) {
            boolean z;
            try {
                long now = LocalCache.this.ticker.read();
                K key = entry.getKey();
                V value = LocalCache.this.getLiveValue(entry, now);
                if (value != null) {
                    this.nextExternal = new WriteThroughEntry(key, value);
                    z = true;
                } else {
                    z = false;
                }
                return z;
            } finally {
                this.currentSegment.postReadCleanup();
            }
        }

        public boolean hasNext() {
            return this.nextExternal != null;
        }

        /* access modifiers changed from: package-private */
        public LocalCache<K, V>.WriteThroughEntry nextEntry() {
            if (this.nextExternal != null) {
                this.lastReturned = this.nextExternal;
                advance();
                return this.lastReturned;
            }
            throw new NoSuchElementException();
        }

        public void remove() {
            Preconditions.checkState(this.lastReturned != null);
            LocalCache.this.remove(this.lastReturned.getKey());
            this.lastReturned = null;
        }
    }

    final class KeyIterator extends LocalCache<K, V>.HashIterator implements Iterator<K> {
        KeyIterator() {
            super();
        }

        public K next() {
            return nextEntry().getKey();
        }
    }

    final class ValueIterator extends LocalCache<K, V>.HashIterator implements Iterator<V> {
        ValueIterator() {
            super();
        }

        public V next() {
            return nextEntry().getValue();
        }
    }

    final class WriteThroughEntry implements Map.Entry<K, V> {
        final K key;
        V value;

        WriteThroughEntry(K key2, V value2) {
            this.key = key2;
            this.value = value2;
        }

        public K getKey() {
            return this.key;
        }

        public V getValue() {
            return this.value;
        }

        public boolean equals(@Nullable Object object) {
            if (!(object instanceof Map.Entry)) {
                return false;
            }
            Map.Entry<?, ?> that = (Map.Entry) object;
            if (!this.key.equals(that.getKey()) || !this.value.equals(that.getValue())) {
                return false;
            }
            return true;
        }

        public int hashCode() {
            return this.key.hashCode() ^ this.value.hashCode();
        }

        public V setValue(V v) {
            throw new UnsupportedOperationException();
        }

        public String toString() {
            return getKey() + "=" + getValue();
        }
    }

    final class EntryIterator extends LocalCache<K, V>.HashIterator implements Iterator<Map.Entry<K, V>> {
        EntryIterator() {
            super();
        }

        public Map.Entry<K, V> next() {
            return nextEntry();
        }
    }

    final class KeySet extends AbstractSet<K> {
        KeySet() {
        }

        public Iterator<K> iterator() {
            return new KeyIterator();
        }

        public int size() {
            return LocalCache.this.size();
        }

        public boolean isEmpty() {
            return LocalCache.this.isEmpty();
        }

        public boolean contains(Object o) {
            return LocalCache.this.containsKey(o);
        }

        public boolean remove(Object o) {
            return LocalCache.this.remove(o) != null;
        }

        public void clear() {
            LocalCache.this.clear();
        }
    }

    final class Values extends AbstractCollection<V> {
        Values() {
        }

        public Iterator<V> iterator() {
            return new ValueIterator();
        }

        public int size() {
            return LocalCache.this.size();
        }

        public boolean isEmpty() {
            return LocalCache.this.isEmpty();
        }

        public boolean contains(Object o) {
            return LocalCache.this.containsValue(o);
        }

        public void clear() {
            LocalCache.this.clear();
        }
    }

    final class EntrySet extends AbstractSet<Map.Entry<K, V>> {
        EntrySet() {
        }

        public Iterator<Map.Entry<K, V>> iterator() {
            return new EntryIterator();
        }

        /* JADX WARNING: Code restructure failed: missing block: B:3:0x0006, code lost:
            r0 = (java.util.Map.Entry) r7;
         */
        /* Code decompiled incorrectly, please refer to instructions dump. */
        public boolean contains(java.lang.Object r7) {
            /*
                r6 = this;
                boolean r0 = r7 instanceof java.util.Map.Entry
                r1 = 0
                if (r0 != 0) goto L_0x0006
                return r1
            L_0x0006:
                r0 = r7
                java.util.Map$Entry r0 = (java.util.Map.Entry) r0
                java.lang.Object r2 = r0.getKey()
                if (r2 != 0) goto L_0x0010
                return r1
            L_0x0010:
                com.google.common.cache.LocalCache r3 = com.google.common.cache.LocalCache.this
                java.lang.Object r3 = r3.get(r2)
                if (r3 == 0) goto L_0x0028
                com.google.common.cache.LocalCache r4 = com.google.common.cache.LocalCache.this
                com.google.common.base.Equivalence<java.lang.Object> r4 = r4.valueEquivalence
                java.lang.Object r5 = r0.getValue()
                boolean r4 = r4.equivalent(r5, r3)
                if (r4 == 0) goto L_0x0028
                r1 = 1
            L_0x0028:
                return r1
            */
            throw new UnsupportedOperationException("Method not decompiled: com.google.common.cache.LocalCache.EntrySet.contains(java.lang.Object):boolean");
        }

        /* JADX WARNING: Code restructure failed: missing block: B:3:0x0006, code lost:
            r0 = (java.util.Map.Entry) r6;
         */
        /* Code decompiled incorrectly, please refer to instructions dump. */
        public boolean remove(java.lang.Object r6) {
            /*
                r5 = this;
                boolean r0 = r6 instanceof java.util.Map.Entry
                r1 = 0
                if (r0 != 0) goto L_0x0006
                return r1
            L_0x0006:
                r0 = r6
                java.util.Map$Entry r0 = (java.util.Map.Entry) r0
                java.lang.Object r2 = r0.getKey()
                if (r2 == 0) goto L_0x001d
                com.google.common.cache.LocalCache r3 = com.google.common.cache.LocalCache.this
                java.lang.Object r4 = r0.getValue()
                boolean r3 = r3.remove(r2, r4)
                if (r3 == 0) goto L_0x001d
                r1 = 1
            L_0x001d:
                return r1
            */
            throw new UnsupportedOperationException("Method not decompiled: com.google.common.cache.LocalCache.EntrySet.remove(java.lang.Object):boolean");
        }

        public int size() {
            return LocalCache.this.size();
        }

        public boolean isEmpty() {
            return LocalCache.this.isEmpty();
        }

        public void clear() {
            LocalCache.this.clear();
        }
    }

    static class ManualSerializationProxy<K, V> extends ForwardingCache<K, V> implements Serializable {
        private static final long serialVersionUID = 1;
        final int concurrencyLevel;
        transient Cache<K, V> delegate;
        final long expireAfterAccessNanos;
        final long expireAfterWriteNanos;
        final Equivalence<Object> keyEquivalence;
        final Strength keyStrength;
        final CacheLoader<? super K, V> loader;
        final long maxWeight;
        final RemovalListener<? super K, ? super V> removalListener;
        final Ticker ticker;
        final Equivalence<Object> valueEquivalence;
        final Strength valueStrength;
        final Weigher<K, V> weigher;

        /* JADX WARNING: Illegal instructions before constructor call */
        /* Code decompiled incorrectly, please refer to instructions dump. */
        ManualSerializationProxy(com.google.common.cache.LocalCache<K, V> r18) {
            /*
                r17 = this;
                r0 = r18
                com.google.common.cache.LocalCache$Strength r2 = r0.keyStrength
                com.google.common.cache.LocalCache$Strength r3 = r0.valueStrength
                com.google.common.base.Equivalence<java.lang.Object> r4 = r0.keyEquivalence
                com.google.common.base.Equivalence<java.lang.Object> r5 = r0.valueEquivalence
                long r6 = r0.expireAfterWriteNanos
                long r8 = r0.expireAfterAccessNanos
                long r10 = r0.maxWeight
                com.google.common.cache.Weigher<K, V> r12 = r0.weigher
                int r13 = r0.concurrencyLevel
                com.google.common.cache.RemovalListener<K, V> r14 = r0.removalListener
                com.google.common.base.Ticker r15 = r0.ticker
                com.google.common.cache.CacheLoader<? super K, V> r1 = r0.defaultLoader
                r16 = r1
                r1 = r17
                r1.<init>(r2, r3, r4, r5, r6, r8, r10, r12, r13, r14, r15, r16)
                return
            */
            throw new UnsupportedOperationException("Method not decompiled: com.google.common.cache.LocalCache.ManualSerializationProxy.<init>(com.google.common.cache.LocalCache):void");
        }

        private ManualSerializationProxy(Strength keyStrength2, Strength valueStrength2, Equivalence<Object> keyEquivalence2, Equivalence<Object> valueEquivalence2, long expireAfterWriteNanos2, long expireAfterAccessNanos2, long maxWeight2, Weigher<K, V> weigher2, int concurrencyLevel2, RemovalListener<? super K, ? super V> removalListener2, Ticker ticker2, CacheLoader<? super K, V> loader2) {
            Ticker ticker3 = ticker2;
            this.keyStrength = keyStrength2;
            this.valueStrength = valueStrength2;
            this.keyEquivalence = keyEquivalence2;
            this.valueEquivalence = valueEquivalence2;
            this.expireAfterWriteNanos = expireAfterWriteNanos2;
            this.expireAfterAccessNanos = expireAfterAccessNanos2;
            this.maxWeight = maxWeight2;
            this.weigher = weigher2;
            this.concurrencyLevel = concurrencyLevel2;
            this.removalListener = removalListener2;
            this.ticker = (ticker3 == Ticker.systemTicker() || ticker3 == CacheBuilder.NULL_TICKER) ? null : ticker3;
            this.loader = loader2;
        }

        /* access modifiers changed from: package-private */
        public CacheBuilder<Object, Object> recreateCacheBuilder() {
            CacheBuilder<Object, Object> builder = CacheBuilder.newBuilder().setKeyStrength(this.keyStrength).setValueStrength(this.valueStrength).keyEquivalence(this.keyEquivalence).valueEquivalence(this.valueEquivalence).concurrencyLevel(this.concurrencyLevel);
            builder.strictParsing = false;
            builder.removalListener(this.removalListener);
            if (this.expireAfterWriteNanos > 0) {
                builder.expireAfterWrite(this.expireAfterWriteNanos, TimeUnit.NANOSECONDS);
            }
            if (this.expireAfterAccessNanos > 0) {
                builder.expireAfterAccess(this.expireAfterAccessNanos, TimeUnit.NANOSECONDS);
            }
            if (this.weigher != CacheBuilder.OneWeigher.INSTANCE) {
                builder.weigher(this.weigher);
                if (this.maxWeight != -1) {
                    builder.maximumWeight(this.maxWeight);
                }
            } else if (this.maxWeight != -1) {
                builder.maximumSize(this.maxWeight);
            }
            if (this.ticker != null) {
                builder.ticker(this.ticker);
            }
            return builder;
        }

        private void readObject(ObjectInputStream in) throws IOException, ClassNotFoundException {
            in.defaultReadObject();
            this.delegate = recreateCacheBuilder().build();
        }

        private Object readResolve() {
            return this.delegate;
        }

        /* access modifiers changed from: protected */
        public Cache<K, V> delegate() {
            return this.delegate;
        }
    }

    static final class LoadingSerializationProxy<K, V> extends ManualSerializationProxy<K, V> implements LoadingCache<K, V>, Serializable {
        private static final long serialVersionUID = 1;
        transient LoadingCache<K, V> autoDelegate;

        LoadingSerializationProxy(LocalCache<K, V> cache) {
            super(cache);
        }

        private void readObject(ObjectInputStream in) throws IOException, ClassNotFoundException {
            in.defaultReadObject();
            this.autoDelegate = recreateCacheBuilder().build(this.loader);
        }

        public V get(K key) throws ExecutionException {
            return this.autoDelegate.get(key);
        }

        public V getUnchecked(K key) {
            return this.autoDelegate.getUnchecked(key);
        }

        public ImmutableMap<K, V> getAll(Iterable<? extends K> keys) throws ExecutionException {
            return this.autoDelegate.getAll(keys);
        }

        public final V apply(K key) {
            return this.autoDelegate.apply(key);
        }

        public void refresh(K key) {
            this.autoDelegate.refresh(key);
        }

        private Object readResolve() {
            return this.autoDelegate;
        }
    }

    static class LocalManualCache<K, V> implements Cache<K, V>, Serializable {
        private static final long serialVersionUID = 1;
        final LocalCache<K, V> localCache;

        LocalManualCache(CacheBuilder<? super K, ? super V> builder) {
            this(new LocalCache(builder, (CacheLoader) null));
        }

        private LocalManualCache(LocalCache<K, V> localCache2) {
            this.localCache = localCache2;
        }

        @Nullable
        public V getIfPresent(Object key) {
            return this.localCache.getIfPresent(key);
        }

        public V get(K key, final Callable<? extends V> valueLoader) throws ExecutionException {
            Preconditions.checkNotNull(valueLoader);
            return this.localCache.get(key, new CacheLoader<Object, V>() {
                public V load(Object key) throws Exception {
                    return valueLoader.call();
                }
            });
        }

        public ImmutableMap<K, V> getAllPresent(Iterable<?> keys) {
            return this.localCache.getAllPresent(keys);
        }

        public void put(K key, V value) {
            this.localCache.put(key, value);
        }

        public void putAll(Map<? extends K, ? extends V> m) {
            this.localCache.putAll(m);
        }

        public void invalidate(Object key) {
            Preconditions.checkNotNull(key);
            this.localCache.remove(key);
        }

        public void invalidateAll(Iterable<?> keys) {
            this.localCache.invalidateAll(keys);
        }

        public void invalidateAll() {
            this.localCache.clear();
        }

        public long size() {
            return this.localCache.longSize();
        }

        public ConcurrentMap<K, V> asMap() {
            return this.localCache;
        }

        public CacheStats stats() {
            AbstractCache.SimpleStatsCounter aggregator = new AbstractCache.SimpleStatsCounter();
            aggregator.incrementBy(this.localCache.globalStatsCounter);
            for (Segment<K, V> segment : this.localCache.segments) {
                aggregator.incrementBy(segment.statsCounter);
            }
            return aggregator.snapshot();
        }

        public void cleanUp() {
            this.localCache.cleanUp();
        }

        /* access modifiers changed from: package-private */
        public Object writeReplace() {
            return new ManualSerializationProxy(this.localCache);
        }
    }

    static class LocalLoadingCache<K, V> extends LocalManualCache<K, V> implements LoadingCache<K, V> {
        private static final long serialVersionUID = 1;

        LocalLoadingCache(CacheBuilder<? super K, ? super V> builder, CacheLoader<? super K, V> loader) {
            super();
        }

        public V get(K key) throws ExecutionException {
            return this.localCache.getOrLoad(key);
        }

        public V getUnchecked(K key) {
            try {
                return get(key);
            } catch (ExecutionException e) {
                throw new UncheckedExecutionException(e.getCause());
            }
        }

        public ImmutableMap<K, V> getAll(Iterable<? extends K> keys) throws ExecutionException {
            return this.localCache.getAll(keys);
        }

        public void refresh(K key) {
            this.localCache.refresh(key);
        }

        public final V apply(K key) {
            return getUnchecked(key);
        }

        /* access modifiers changed from: package-private */
        public Object writeReplace() {
            return new LoadingSerializationProxy(this.localCache);
        }
    }
}
