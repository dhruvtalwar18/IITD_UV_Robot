package com.google.common.collect;

import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Equivalence;
import com.google.common.base.Equivalences;
import com.google.common.base.Preconditions;
import com.google.common.base.Ticker;
import com.google.common.collect.GenericMapMaker;
import com.google.common.collect.MapMaker;
import com.google.common.primitives.Ints;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
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
import java.util.concurrent.CancellationException;
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

class MapMakerInternalMap<K, V> extends AbstractMap<K, V> implements ConcurrentMap<K, V>, Serializable {
    static final long CLEANUP_EXECUTOR_DELAY_SECS = 60;
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

        public ReferenceEntry<Object, Object> getEntry() {
            return null;
        }

        public ValueReference<Object, Object> copyFor(ReferenceQueue<Object> referenceQueue, Object value, ReferenceEntry<Object, Object> referenceEntry) {
            return this;
        }

        public boolean isComputingReference() {
            return false;
        }

        public Object waitForValue() {
            return null;
        }

        public void clear(ValueReference<Object, Object> valueReference) {
        }
    };
    private static final Logger logger = Logger.getLogger(MapMakerInternalMap.class.getName());
    private static final long serialVersionUID = 5;
    final int concurrencyLevel;
    final transient EntryFactory entryFactory = EntryFactory.getFactory(this.keyStrength, expires(), evictsBySize());
    Set<Map.Entry<K, V>> entrySet;
    final long expireAfterAccessNanos;
    final long expireAfterWriteNanos;
    final Equivalence<Object> keyEquivalence;
    Set<K> keySet;
    final Strength keyStrength;
    final int maximumSize;
    final MapMaker.RemovalListener<K, V> removalListener;
    final Queue<MapMaker.RemovalNotification<K, V>> removalNotificationQueue;
    final transient int segmentMask;
    final transient int segmentShift;
    final transient Segment<K, V>[] segments;
    final Ticker ticker;
    final Equivalence<Object> valueEquivalence;
    final Strength valueStrength;
    Collection<V> values;

    interface ReferenceEntry<K, V> {
        long getExpirationTime();

        int getHash();

        K getKey();

        ReferenceEntry<K, V> getNext();

        ReferenceEntry<K, V> getNextEvictable();

        ReferenceEntry<K, V> getNextExpirable();

        ReferenceEntry<K, V> getPreviousEvictable();

        ReferenceEntry<K, V> getPreviousExpirable();

        ValueReference<K, V> getValueReference();

        void setExpirationTime(long j);

        void setNextEvictable(ReferenceEntry<K, V> referenceEntry);

        void setNextExpirable(ReferenceEntry<K, V> referenceEntry);

        void setPreviousEvictable(ReferenceEntry<K, V> referenceEntry);

        void setPreviousExpirable(ReferenceEntry<K, V> referenceEntry);

        void setValueReference(ValueReference<K, V> valueReference);
    }

    enum Strength {
        STRONG {
            /* access modifiers changed from: package-private */
            public <K, V> ValueReference<K, V> referenceValue(Segment<K, V> segment, ReferenceEntry<K, V> referenceEntry, V value) {
                return new StrongValueReference(value);
            }

            /* access modifiers changed from: package-private */
            public Equivalence<Object> defaultEquivalence() {
                return Equivalences.equals();
            }
        },
        SOFT {
            /* access modifiers changed from: package-private */
            public <K, V> ValueReference<K, V> referenceValue(Segment<K, V> segment, ReferenceEntry<K, V> entry, V value) {
                return new SoftValueReference(segment.valueReferenceQueue, value, entry);
            }

            /* access modifiers changed from: package-private */
            public Equivalence<Object> defaultEquivalence() {
                return Equivalences.identity();
            }
        },
        WEAK {
            /* access modifiers changed from: package-private */
            public <K, V> ValueReference<K, V> referenceValue(Segment<K, V> segment, ReferenceEntry<K, V> entry, V value) {
                return new WeakValueReference(segment.valueReferenceQueue, value, entry);
            }

            /* access modifiers changed from: package-private */
            public Equivalence<Object> defaultEquivalence() {
                return Equivalences.identity();
            }
        };

        /* access modifiers changed from: package-private */
        public abstract Equivalence<Object> defaultEquivalence();

        /* access modifiers changed from: package-private */
        public abstract <K, V> ValueReference<K, V> referenceValue(Segment<K, V> segment, ReferenceEntry<K, V> referenceEntry, V v);
    }

    interface ValueReference<K, V> {
        void clear(@Nullable ValueReference<K, V> valueReference);

        ValueReference<K, V> copyFor(ReferenceQueue<V> referenceQueue, V v, ReferenceEntry<K, V> referenceEntry);

        V get();

        ReferenceEntry<K, V> getEntry();

        boolean isComputingReference();

        V waitForValue() throws ExecutionException;
    }

    MapMakerInternalMap(MapMaker builder) {
        this.concurrencyLevel = Math.min(builder.getConcurrencyLevel(), 65536);
        this.keyStrength = builder.getKeyStrength();
        this.valueStrength = builder.getValueStrength();
        this.keyEquivalence = builder.getKeyEquivalence();
        this.valueEquivalence = builder.getValueEquivalence();
        this.maximumSize = builder.maximumSize;
        this.expireAfterAccessNanos = builder.getExpireAfterAccessNanos();
        this.expireAfterWriteNanos = builder.getExpireAfterWriteNanos();
        this.ticker = builder.getTicker();
        this.removalListener = builder.getRemovalListener();
        this.removalNotificationQueue = this.removalListener == GenericMapMaker.NullListener.INSTANCE ? discardingQueue() : new ConcurrentLinkedQueue<>();
        int initialCapacity = Math.min(builder.getInitialCapacity(), 1073741824);
        initialCapacity = evictsBySize() ? Math.min(initialCapacity, this.maximumSize) : initialCapacity;
        int segmentShift2 = 0;
        int segmentCount = 1;
        while (segmentCount < this.concurrencyLevel && (!evictsBySize() || segmentCount * 2 <= this.maximumSize)) {
            segmentShift2++;
            segmentCount <<= 1;
        }
        this.segmentShift = 32 - segmentShift2;
        this.segmentMask = segmentCount - 1;
        this.segments = newSegmentArray(segmentCount);
        int segmentCapacity = initialCapacity / segmentCount;
        int segmentSize = 1;
        while (segmentSize < (segmentCapacity * segmentCount < initialCapacity ? segmentCapacity + 1 : segmentCapacity)) {
            segmentSize <<= 1;
        }
        int i = 0;
        if (evictsBySize()) {
            int maximumSegmentSize = (this.maximumSize / segmentCount) + 1;
            int remainder = this.maximumSize % segmentCount;
            while (i < this.segments.length) {
                if (i == remainder) {
                    maximumSegmentSize--;
                }
                this.segments[i] = createSegment(segmentSize, maximumSegmentSize);
                i++;
            }
            return;
        }
        while (true) {
            int i2 = i;
            if (i2 < this.segments.length) {
                this.segments[i2] = createSegment(segmentSize, -1);
                i = i2 + 1;
            } else {
                return;
            }
        }
    }

    /* access modifiers changed from: package-private */
    public boolean evictsBySize() {
        return this.maximumSize != -1;
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
        STRONG_EXPIRABLE {
            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> newEntry(Segment<K, V> segment, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
                return new StrongExpirableEntry(key, hash, next);
            }

            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> copyEntry(Segment<K, V> segment, ReferenceEntry<K, V> original, ReferenceEntry<K, V> newNext) {
                ReferenceEntry<K, V> newEntry = super.copyEntry(segment, original, newNext);
                copyExpirableEntry(original, newEntry);
                return newEntry;
            }
        },
        STRONG_EVICTABLE {
            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> newEntry(Segment<K, V> segment, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
                return new StrongEvictableEntry(key, hash, next);
            }

            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> copyEntry(Segment<K, V> segment, ReferenceEntry<K, V> original, ReferenceEntry<K, V> newNext) {
                ReferenceEntry<K, V> newEntry = super.copyEntry(segment, original, newNext);
                copyEvictableEntry(original, newEntry);
                return newEntry;
            }
        },
        STRONG_EXPIRABLE_EVICTABLE {
            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> newEntry(Segment<K, V> segment, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
                return new StrongExpirableEvictableEntry(key, hash, next);
            }

            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> copyEntry(Segment<K, V> segment, ReferenceEntry<K, V> original, ReferenceEntry<K, V> newNext) {
                ReferenceEntry<K, V> newEntry = super.copyEntry(segment, original, newNext);
                copyExpirableEntry(original, newEntry);
                copyEvictableEntry(original, newEntry);
                return newEntry;
            }
        },
        SOFT {
            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> newEntry(Segment<K, V> segment, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
                return new SoftEntry(segment.keyReferenceQueue, key, hash, next);
            }
        },
        SOFT_EXPIRABLE {
            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> newEntry(Segment<K, V> segment, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
                return new SoftExpirableEntry(segment.keyReferenceQueue, key, hash, next);
            }

            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> copyEntry(Segment<K, V> segment, ReferenceEntry<K, V> original, ReferenceEntry<K, V> newNext) {
                ReferenceEntry<K, V> newEntry = super.copyEntry(segment, original, newNext);
                copyExpirableEntry(original, newEntry);
                return newEntry;
            }
        },
        SOFT_EVICTABLE {
            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> newEntry(Segment<K, V> segment, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
                return new SoftEvictableEntry(segment.keyReferenceQueue, key, hash, next);
            }

            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> copyEntry(Segment<K, V> segment, ReferenceEntry<K, V> original, ReferenceEntry<K, V> newNext) {
                ReferenceEntry<K, V> newEntry = super.copyEntry(segment, original, newNext);
                copyEvictableEntry(original, newEntry);
                return newEntry;
            }
        },
        SOFT_EXPIRABLE_EVICTABLE {
            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> newEntry(Segment<K, V> segment, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
                return new SoftExpirableEvictableEntry(segment.keyReferenceQueue, key, hash, next);
            }

            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> copyEntry(Segment<K, V> segment, ReferenceEntry<K, V> original, ReferenceEntry<K, V> newNext) {
                ReferenceEntry<K, V> newEntry = super.copyEntry(segment, original, newNext);
                copyExpirableEntry(original, newEntry);
                copyEvictableEntry(original, newEntry);
                return newEntry;
            }
        },
        WEAK {
            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> newEntry(Segment<K, V> segment, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
                return new WeakEntry(segment.keyReferenceQueue, key, hash, next);
            }
        },
        WEAK_EXPIRABLE {
            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> newEntry(Segment<K, V> segment, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
                return new WeakExpirableEntry(segment.keyReferenceQueue, key, hash, next);
            }

            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> copyEntry(Segment<K, V> segment, ReferenceEntry<K, V> original, ReferenceEntry<K, V> newNext) {
                ReferenceEntry<K, V> newEntry = super.copyEntry(segment, original, newNext);
                copyExpirableEntry(original, newEntry);
                return newEntry;
            }
        },
        WEAK_EVICTABLE {
            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> newEntry(Segment<K, V> segment, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
                return new WeakEvictableEntry(segment.keyReferenceQueue, key, hash, next);
            }

            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> copyEntry(Segment<K, V> segment, ReferenceEntry<K, V> original, ReferenceEntry<K, V> newNext) {
                ReferenceEntry<K, V> newEntry = super.copyEntry(segment, original, newNext);
                copyEvictableEntry(original, newEntry);
                return newEntry;
            }
        },
        WEAK_EXPIRABLE_EVICTABLE {
            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> newEntry(Segment<K, V> segment, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
                return new WeakExpirableEvictableEntry(segment.keyReferenceQueue, key, hash, next);
            }

            /* access modifiers changed from: package-private */
            public <K, V> ReferenceEntry<K, V> copyEntry(Segment<K, V> segment, ReferenceEntry<K, V> original, ReferenceEntry<K, V> newNext) {
                ReferenceEntry<K, V> newEntry = super.copyEntry(segment, original, newNext);
                copyExpirableEntry(original, newEntry);
                copyEvictableEntry(original, newEntry);
                return newEntry;
            }
        };
        
        static final int EVICTABLE_MASK = 2;
        static final int EXPIRABLE_MASK = 1;
        static final EntryFactory[][] factories = null;

        /* access modifiers changed from: package-private */
        public abstract <K, V> ReferenceEntry<K, V> newEntry(Segment<K, V> segment, K k, int i, @Nullable ReferenceEntry<K, V> referenceEntry);

        static {
            factories = new EntryFactory[][]{new EntryFactory[]{STRONG, STRONG_EXPIRABLE, STRONG_EVICTABLE, STRONG_EXPIRABLE_EVICTABLE}, new EntryFactory[]{SOFT, SOFT_EXPIRABLE, SOFT_EVICTABLE, SOFT_EXPIRABLE_EVICTABLE}, new EntryFactory[]{WEAK, WEAK_EXPIRABLE, WEAK_EVICTABLE, WEAK_EXPIRABLE_EVICTABLE}};
        }

        static EntryFactory getFactory(Strength keyStrength, boolean expireAfterWrite, boolean evictsBySize) {
            return factories[keyStrength.ordinal()][(evictsBySize ? 2 : 0) | expireAfterWrite];
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public <K, V> ReferenceEntry<K, V> copyEntry(Segment<K, V> segment, ReferenceEntry<K, V> original, ReferenceEntry<K, V> newNext) {
            return newEntry(segment, original.getKey(), original.getHash(), newNext);
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public <K, V> void copyExpirableEntry(ReferenceEntry<K, V> original, ReferenceEntry<K, V> newEntry) {
            newEntry.setExpirationTime(original.getExpirationTime());
            MapMakerInternalMap.connectExpirables(original.getPreviousExpirable(), newEntry);
            MapMakerInternalMap.connectExpirables(newEntry, original.getNextExpirable());
            MapMakerInternalMap.nullifyExpirable(original);
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public <K, V> void copyEvictableEntry(ReferenceEntry<K, V> original, ReferenceEntry<K, V> newEntry) {
            MapMakerInternalMap.connectEvictables(original.getPreviousEvictable(), newEntry);
            MapMakerInternalMap.connectEvictables(newEntry, original.getNextEvictable());
            MapMakerInternalMap.nullifyEvictable(original);
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

        public long getExpirationTime() {
            return 0;
        }

        public void setExpirationTime(long time) {
        }

        public ReferenceEntry<Object, Object> getNextExpirable() {
            return this;
        }

        public void setNextExpirable(ReferenceEntry<Object, Object> referenceEntry) {
        }

        public ReferenceEntry<Object, Object> getPreviousExpirable() {
            return this;
        }

        public void setPreviousExpirable(ReferenceEntry<Object, Object> referenceEntry) {
        }

        public ReferenceEntry<Object, Object> getNextEvictable() {
            return this;
        }

        public void setNextEvictable(ReferenceEntry<Object, Object> referenceEntry) {
        }

        public ReferenceEntry<Object, Object> getPreviousEvictable() {
            return this;
        }

        public void setPreviousEvictable(ReferenceEntry<Object, Object> referenceEntry) {
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

        public long getExpirationTime() {
            throw new UnsupportedOperationException();
        }

        public void setExpirationTime(long time) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getNextExpirable() {
            throw new UnsupportedOperationException();
        }

        public void setNextExpirable(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getPreviousExpirable() {
            throw new UnsupportedOperationException();
        }

        public void setPreviousExpirable(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getNextEvictable() {
            throw new UnsupportedOperationException();
        }

        public void setNextEvictable(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getPreviousEvictable() {
            throw new UnsupportedOperationException();
        }

        public void setPreviousEvictable(ReferenceEntry<K, V> referenceEntry) {
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
        volatile ValueReference<K, V> valueReference = MapMakerInternalMap.unset();

        StrongEntry(K key2, int hash2, @Nullable ReferenceEntry<K, V> next2) {
            this.key = key2;
            this.hash = hash2;
            this.next = next2;
        }

        public K getKey() {
            return this.key;
        }

        public long getExpirationTime() {
            throw new UnsupportedOperationException();
        }

        public void setExpirationTime(long time) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getNextExpirable() {
            throw new UnsupportedOperationException();
        }

        public void setNextExpirable(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getPreviousExpirable() {
            throw new UnsupportedOperationException();
        }

        public void setPreviousExpirable(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getNextEvictable() {
            throw new UnsupportedOperationException();
        }

        public void setNextEvictable(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getPreviousEvictable() {
            throw new UnsupportedOperationException();
        }

        public void setPreviousEvictable(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public ValueReference<K, V> getValueReference() {
            return this.valueReference;
        }

        public void setValueReference(ValueReference<K, V> valueReference2) {
            ValueReference<K, V> previous = this.valueReference;
            this.valueReference = valueReference2;
            previous.clear(valueReference2);
        }

        public int getHash() {
            return this.hash;
        }

        public ReferenceEntry<K, V> getNext() {
            return this.next;
        }
    }

    static final class StrongExpirableEntry<K, V> extends StrongEntry<K, V> implements ReferenceEntry<K, V> {
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> nextExpirable = MapMakerInternalMap.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> previousExpirable = MapMakerInternalMap.nullEntry();
        volatile long time = Long.MAX_VALUE;

        StrongExpirableEntry(K key, int hash, @Nullable ReferenceEntry<K, V> next) {
            super(key, hash, next);
        }

        public long getExpirationTime() {
            return this.time;
        }

        public void setExpirationTime(long time2) {
            this.time = time2;
        }

        public ReferenceEntry<K, V> getNextExpirable() {
            return this.nextExpirable;
        }

        public void setNextExpirable(ReferenceEntry<K, V> next) {
            this.nextExpirable = next;
        }

        public ReferenceEntry<K, V> getPreviousExpirable() {
            return this.previousExpirable;
        }

        public void setPreviousExpirable(ReferenceEntry<K, V> previous) {
            this.previousExpirable = previous;
        }
    }

    static final class StrongEvictableEntry<K, V> extends StrongEntry<K, V> implements ReferenceEntry<K, V> {
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> nextEvictable = MapMakerInternalMap.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> previousEvictable = MapMakerInternalMap.nullEntry();

        StrongEvictableEntry(K key, int hash, @Nullable ReferenceEntry<K, V> next) {
            super(key, hash, next);
        }

        public ReferenceEntry<K, V> getNextEvictable() {
            return this.nextEvictable;
        }

        public void setNextEvictable(ReferenceEntry<K, V> next) {
            this.nextEvictable = next;
        }

        public ReferenceEntry<K, V> getPreviousEvictable() {
            return this.previousEvictable;
        }

        public void setPreviousEvictable(ReferenceEntry<K, V> previous) {
            this.previousEvictable = previous;
        }
    }

    static final class StrongExpirableEvictableEntry<K, V> extends StrongEntry<K, V> implements ReferenceEntry<K, V> {
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> nextEvictable = MapMakerInternalMap.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> nextExpirable = MapMakerInternalMap.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> previousEvictable = MapMakerInternalMap.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> previousExpirable = MapMakerInternalMap.nullEntry();
        volatile long time = Long.MAX_VALUE;

        StrongExpirableEvictableEntry(K key, int hash, @Nullable ReferenceEntry<K, V> next) {
            super(key, hash, next);
        }

        public long getExpirationTime() {
            return this.time;
        }

        public void setExpirationTime(long time2) {
            this.time = time2;
        }

        public ReferenceEntry<K, V> getNextExpirable() {
            return this.nextExpirable;
        }

        public void setNextExpirable(ReferenceEntry<K, V> next) {
            this.nextExpirable = next;
        }

        public ReferenceEntry<K, V> getPreviousExpirable() {
            return this.previousExpirable;
        }

        public void setPreviousExpirable(ReferenceEntry<K, V> previous) {
            this.previousExpirable = previous;
        }

        public ReferenceEntry<K, V> getNextEvictable() {
            return this.nextEvictable;
        }

        public void setNextEvictable(ReferenceEntry<K, V> next) {
            this.nextEvictable = next;
        }

        public ReferenceEntry<K, V> getPreviousEvictable() {
            return this.previousEvictable;
        }

        public void setPreviousEvictable(ReferenceEntry<K, V> previous) {
            this.previousEvictable = previous;
        }
    }

    static class SoftEntry<K, V> extends SoftReference<K> implements ReferenceEntry<K, V> {
        final int hash;
        final ReferenceEntry<K, V> next;
        volatile ValueReference<K, V> valueReference = MapMakerInternalMap.unset();

        SoftEntry(ReferenceQueue<K> queue, K key, int hash2, @Nullable ReferenceEntry<K, V> next2) {
            super(key, queue);
            this.hash = hash2;
            this.next = next2;
        }

        public K getKey() {
            return get();
        }

        public long getExpirationTime() {
            throw new UnsupportedOperationException();
        }

        public void setExpirationTime(long time) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getNextExpirable() {
            throw new UnsupportedOperationException();
        }

        public void setNextExpirable(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getPreviousExpirable() {
            throw new UnsupportedOperationException();
        }

        public void setPreviousExpirable(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getNextEvictable() {
            throw new UnsupportedOperationException();
        }

        public void setNextEvictable(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getPreviousEvictable() {
            throw new UnsupportedOperationException();
        }

        public void setPreviousEvictable(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public ValueReference<K, V> getValueReference() {
            return this.valueReference;
        }

        public void setValueReference(ValueReference<K, V> valueReference2) {
            ValueReference<K, V> previous = this.valueReference;
            this.valueReference = valueReference2;
            previous.clear(valueReference2);
        }

        public int getHash() {
            return this.hash;
        }

        public ReferenceEntry<K, V> getNext() {
            return this.next;
        }
    }

    static final class SoftExpirableEntry<K, V> extends SoftEntry<K, V> implements ReferenceEntry<K, V> {
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> nextExpirable = MapMakerInternalMap.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> previousExpirable = MapMakerInternalMap.nullEntry();
        volatile long time = Long.MAX_VALUE;

        SoftExpirableEntry(ReferenceQueue<K> queue, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
            super(queue, key, hash, next);
        }

        public long getExpirationTime() {
            return this.time;
        }

        public void setExpirationTime(long time2) {
            this.time = time2;
        }

        public ReferenceEntry<K, V> getNextExpirable() {
            return this.nextExpirable;
        }

        public void setNextExpirable(ReferenceEntry<K, V> next) {
            this.nextExpirable = next;
        }

        public ReferenceEntry<K, V> getPreviousExpirable() {
            return this.previousExpirable;
        }

        public void setPreviousExpirable(ReferenceEntry<K, V> previous) {
            this.previousExpirable = previous;
        }
    }

    static final class SoftEvictableEntry<K, V> extends SoftEntry<K, V> implements ReferenceEntry<K, V> {
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> nextEvictable = MapMakerInternalMap.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> previousEvictable = MapMakerInternalMap.nullEntry();

        SoftEvictableEntry(ReferenceQueue<K> queue, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
            super(queue, key, hash, next);
        }

        public ReferenceEntry<K, V> getNextEvictable() {
            return this.nextEvictable;
        }

        public void setNextEvictable(ReferenceEntry<K, V> next) {
            this.nextEvictable = next;
        }

        public ReferenceEntry<K, V> getPreviousEvictable() {
            return this.previousEvictable;
        }

        public void setPreviousEvictable(ReferenceEntry<K, V> previous) {
            this.previousEvictable = previous;
        }
    }

    static final class SoftExpirableEvictableEntry<K, V> extends SoftEntry<K, V> implements ReferenceEntry<K, V> {
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> nextEvictable = MapMakerInternalMap.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> nextExpirable = MapMakerInternalMap.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> previousEvictable = MapMakerInternalMap.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> previousExpirable = MapMakerInternalMap.nullEntry();
        volatile long time = Long.MAX_VALUE;

        SoftExpirableEvictableEntry(ReferenceQueue<K> queue, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
            super(queue, key, hash, next);
        }

        public long getExpirationTime() {
            return this.time;
        }

        public void setExpirationTime(long time2) {
            this.time = time2;
        }

        public ReferenceEntry<K, V> getNextExpirable() {
            return this.nextExpirable;
        }

        public void setNextExpirable(ReferenceEntry<K, V> next) {
            this.nextExpirable = next;
        }

        public ReferenceEntry<K, V> getPreviousExpirable() {
            return this.previousExpirable;
        }

        public void setPreviousExpirable(ReferenceEntry<K, V> previous) {
            this.previousExpirable = previous;
        }

        public ReferenceEntry<K, V> getNextEvictable() {
            return this.nextEvictable;
        }

        public void setNextEvictable(ReferenceEntry<K, V> next) {
            this.nextEvictable = next;
        }

        public ReferenceEntry<K, V> getPreviousEvictable() {
            return this.previousEvictable;
        }

        public void setPreviousEvictable(ReferenceEntry<K, V> previous) {
            this.previousEvictable = previous;
        }
    }

    static class WeakEntry<K, V> extends WeakReference<K> implements ReferenceEntry<K, V> {
        final int hash;
        final ReferenceEntry<K, V> next;
        volatile ValueReference<K, V> valueReference = MapMakerInternalMap.unset();

        WeakEntry(ReferenceQueue<K> queue, K key, int hash2, @Nullable ReferenceEntry<K, V> next2) {
            super(key, queue);
            this.hash = hash2;
            this.next = next2;
        }

        public K getKey() {
            return get();
        }

        public long getExpirationTime() {
            throw new UnsupportedOperationException();
        }

        public void setExpirationTime(long time) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getNextExpirable() {
            throw new UnsupportedOperationException();
        }

        public void setNextExpirable(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getPreviousExpirable() {
            throw new UnsupportedOperationException();
        }

        public void setPreviousExpirable(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getNextEvictable() {
            throw new UnsupportedOperationException();
        }

        public void setNextEvictable(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public ReferenceEntry<K, V> getPreviousEvictable() {
            throw new UnsupportedOperationException();
        }

        public void setPreviousEvictable(ReferenceEntry<K, V> referenceEntry) {
            throw new UnsupportedOperationException();
        }

        public ValueReference<K, V> getValueReference() {
            return this.valueReference;
        }

        public void setValueReference(ValueReference<K, V> valueReference2) {
            ValueReference<K, V> previous = this.valueReference;
            this.valueReference = valueReference2;
            previous.clear(valueReference2);
        }

        public int getHash() {
            return this.hash;
        }

        public ReferenceEntry<K, V> getNext() {
            return this.next;
        }
    }

    static final class WeakExpirableEntry<K, V> extends WeakEntry<K, V> implements ReferenceEntry<K, V> {
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> nextExpirable = MapMakerInternalMap.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> previousExpirable = MapMakerInternalMap.nullEntry();
        volatile long time = Long.MAX_VALUE;

        WeakExpirableEntry(ReferenceQueue<K> queue, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
            super(queue, key, hash, next);
        }

        public long getExpirationTime() {
            return this.time;
        }

        public void setExpirationTime(long time2) {
            this.time = time2;
        }

        public ReferenceEntry<K, V> getNextExpirable() {
            return this.nextExpirable;
        }

        public void setNextExpirable(ReferenceEntry<K, V> next) {
            this.nextExpirable = next;
        }

        public ReferenceEntry<K, V> getPreviousExpirable() {
            return this.previousExpirable;
        }

        public void setPreviousExpirable(ReferenceEntry<K, V> previous) {
            this.previousExpirable = previous;
        }
    }

    static final class WeakEvictableEntry<K, V> extends WeakEntry<K, V> implements ReferenceEntry<K, V> {
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> nextEvictable = MapMakerInternalMap.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> previousEvictable = MapMakerInternalMap.nullEntry();

        WeakEvictableEntry(ReferenceQueue<K> queue, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
            super(queue, key, hash, next);
        }

        public ReferenceEntry<K, V> getNextEvictable() {
            return this.nextEvictable;
        }

        public void setNextEvictable(ReferenceEntry<K, V> next) {
            this.nextEvictable = next;
        }

        public ReferenceEntry<K, V> getPreviousEvictable() {
            return this.previousEvictable;
        }

        public void setPreviousEvictable(ReferenceEntry<K, V> previous) {
            this.previousEvictable = previous;
        }
    }

    static final class WeakExpirableEvictableEntry<K, V> extends WeakEntry<K, V> implements ReferenceEntry<K, V> {
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> nextEvictable = MapMakerInternalMap.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> nextExpirable = MapMakerInternalMap.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> previousEvictable = MapMakerInternalMap.nullEntry();
        @GuardedBy("Segment.this")
        ReferenceEntry<K, V> previousExpirable = MapMakerInternalMap.nullEntry();
        volatile long time = Long.MAX_VALUE;

        WeakExpirableEvictableEntry(ReferenceQueue<K> queue, K key, int hash, @Nullable ReferenceEntry<K, V> next) {
            super(queue, key, hash, next);
        }

        public long getExpirationTime() {
            return this.time;
        }

        public void setExpirationTime(long time2) {
            this.time = time2;
        }

        public ReferenceEntry<K, V> getNextExpirable() {
            return this.nextExpirable;
        }

        public void setNextExpirable(ReferenceEntry<K, V> next) {
            this.nextExpirable = next;
        }

        public ReferenceEntry<K, V> getPreviousExpirable() {
            return this.previousExpirable;
        }

        public void setPreviousExpirable(ReferenceEntry<K, V> previous) {
            this.previousExpirable = previous;
        }

        public ReferenceEntry<K, V> getNextEvictable() {
            return this.nextEvictable;
        }

        public void setNextEvictable(ReferenceEntry<K, V> next) {
            this.nextEvictable = next;
        }

        public ReferenceEntry<K, V> getPreviousEvictable() {
            return this.previousEvictable;
        }

        public void setPreviousEvictable(ReferenceEntry<K, V> previous) {
            this.previousEvictable = previous;
        }
    }

    static final class WeakValueReference<K, V> extends WeakReference<V> implements ValueReference<K, V> {
        final ReferenceEntry<K, V> entry;

        WeakValueReference(ReferenceQueue<V> queue, V referent, ReferenceEntry<K, V> entry2) {
            super(referent, queue);
            this.entry = entry2;
        }

        public ReferenceEntry<K, V> getEntry() {
            return this.entry;
        }

        public void clear(ValueReference<K, V> valueReference) {
            clear();
        }

        public ValueReference<K, V> copyFor(ReferenceQueue<V> queue, V value, ReferenceEntry<K, V> entry2) {
            return new WeakValueReference(queue, value, entry2);
        }

        public boolean isComputingReference() {
            return false;
        }

        public V waitForValue() {
            return get();
        }
    }

    static final class SoftValueReference<K, V> extends SoftReference<V> implements ValueReference<K, V> {
        final ReferenceEntry<K, V> entry;

        SoftValueReference(ReferenceQueue<V> queue, V referent, ReferenceEntry<K, V> entry2) {
            super(referent, queue);
            this.entry = entry2;
        }

        public ReferenceEntry<K, V> getEntry() {
            return this.entry;
        }

        public void clear(ValueReference<K, V> valueReference) {
            clear();
        }

        public ValueReference<K, V> copyFor(ReferenceQueue<V> queue, V value, ReferenceEntry<K, V> entry2) {
            return new SoftValueReference(queue, value, entry2);
        }

        public boolean isComputingReference() {
            return false;
        }

        public V waitForValue() {
            return get();
        }
    }

    static final class StrongValueReference<K, V> implements ValueReference<K, V> {
        final V referent;

        StrongValueReference(V referent2) {
            this.referent = referent2;
        }

        public V get() {
            return this.referent;
        }

        public ReferenceEntry<K, V> getEntry() {
            return null;
        }

        public ValueReference<K, V> copyFor(ReferenceQueue<V> referenceQueue, V v, ReferenceEntry<K, V> referenceEntry) {
            return this;
        }

        public boolean isComputingReference() {
            return false;
        }

        public V waitForValue() {
            return get();
        }

        public void clear(ValueReference<K, V> valueReference) {
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
    public ValueReference<K, V> newValueReference(ReferenceEntry<K, V> entry, V value) {
        return this.valueStrength.referenceValue(segmentFor(entry.getHash()), entry, value);
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
    public boolean isLive(ReferenceEntry<K, V> entry) {
        return segmentFor(entry.getHash()).getLiveValue(entry) != null;
    }

    /* access modifiers changed from: package-private */
    public Segment<K, V> segmentFor(int hash) {
        return this.segments[(hash >>> this.segmentShift) & this.segmentMask];
    }

    /* access modifiers changed from: package-private */
    public Segment<K, V> createSegment(int initialCapacity, int maxSegmentSize) {
        return new Segment<>(this, initialCapacity, maxSegmentSize);
    }

    /* access modifiers changed from: package-private */
    public V getLiveValue(ReferenceEntry<K, V> entry) {
        V value;
        if (entry.getKey() == null || (value = entry.getValueReference().get()) == null) {
            return null;
        }
        if (!expires() || !isExpired(entry)) {
            return value;
        }
        return null;
    }

    /* access modifiers changed from: package-private */
    public boolean isExpired(ReferenceEntry<K, V> entry) {
        return isExpired(entry, this.ticker.read());
    }

    /* access modifiers changed from: package-private */
    public boolean isExpired(ReferenceEntry<K, V> entry, long now) {
        return now - entry.getExpirationTime() > 0;
    }

    @GuardedBy("Segment.this")
    static <K, V> void connectExpirables(ReferenceEntry<K, V> previous, ReferenceEntry<K, V> next) {
        previous.setNextExpirable(next);
        next.setPreviousExpirable(previous);
    }

    @GuardedBy("Segment.this")
    static <K, V> void nullifyExpirable(ReferenceEntry<K, V> nulled) {
        ReferenceEntry<K, V> nullEntry = nullEntry();
        nulled.setNextExpirable(nullEntry);
        nulled.setPreviousExpirable(nullEntry);
    }

    /* access modifiers changed from: package-private */
    public void processPendingNotifications() {
        while (true) {
            MapMaker.RemovalNotification<K, V> poll = this.removalNotificationQueue.poll();
            MapMaker.RemovalNotification<K, V> notification = poll;
            if (poll != null) {
                try {
                    this.removalListener.onRemoval(notification);
                } catch (Exception e) {
                    logger.log(Level.WARNING, "Exception thrown by removal listener", e);
                }
            } else {
                return;
            }
        }
    }

    @GuardedBy("Segment.this")
    static <K, V> void connectEvictables(ReferenceEntry<K, V> previous, ReferenceEntry<K, V> next) {
        previous.setNextEvictable(next);
        next.setPreviousEvictable(previous);
    }

    @GuardedBy("Segment.this")
    static <K, V> void nullifyEvictable(ReferenceEntry<K, V> nulled) {
        ReferenceEntry<K, V> nullEntry = nullEntry();
        nulled.setNextEvictable(nullEntry);
        nulled.setPreviousEvictable(nullEntry);
    }

    /* access modifiers changed from: package-private */
    public final Segment<K, V>[] newSegmentArray(int ssize) {
        return new Segment[ssize];
    }

    static class Segment<K, V> extends ReentrantLock {
        volatile int count;
        @GuardedBy("Segment.this")
        final Queue<ReferenceEntry<K, V>> evictionQueue;
        @GuardedBy("Segment.this")
        final Queue<ReferenceEntry<K, V>> expirationQueue;
        final ReferenceQueue<K> keyReferenceQueue;
        final MapMakerInternalMap<K, V> map;
        final int maxSegmentSize;
        int modCount;
        final AtomicInteger readCount = new AtomicInteger();
        final Queue<ReferenceEntry<K, V>> recencyQueue;
        volatile AtomicReferenceArray<ReferenceEntry<K, V>> table;
        int threshold;
        final ReferenceQueue<V> valueReferenceQueue;

        Segment(MapMakerInternalMap<K, V> map2, int initialCapacity, int maxSegmentSize2) {
            this.map = map2;
            this.maxSegmentSize = maxSegmentSize2;
            initTable(newEntryArray(initialCapacity));
            ReferenceQueue<V> referenceQueue = null;
            this.keyReferenceQueue = map2.usesKeyReferences() ? new ReferenceQueue<>() : null;
            this.valueReferenceQueue = map2.usesValueReferences() ? new ReferenceQueue<>() : referenceQueue;
            this.recencyQueue = (map2.evictsBySize() || map2.expiresAfterAccess()) ? new ConcurrentLinkedQueue<>() : MapMakerInternalMap.discardingQueue();
            this.evictionQueue = map2.evictsBySize() ? new EvictionQueue<>() : MapMakerInternalMap.discardingQueue();
            this.expirationQueue = map2.expires() ? new ExpirationQueue<>() : MapMakerInternalMap.discardingQueue();
        }

        /* access modifiers changed from: package-private */
        public AtomicReferenceArray<ReferenceEntry<K, V>> newEntryArray(int size) {
            return new AtomicReferenceArray<>(size);
        }

        /* access modifiers changed from: package-private */
        public void initTable(AtomicReferenceArray<ReferenceEntry<K, V>> newTable) {
            this.threshold = (newTable.length() * 3) / 4;
            if (this.threshold == this.maxSegmentSize) {
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
            if (value == null && !valueReference.isComputingReference()) {
                return null;
            }
            ReferenceEntry<K, V> newEntry = this.map.entryFactory.copyEntry(this, original, newNext);
            newEntry.setValueReference(valueReference.copyFor(this.valueReferenceQueue, value, newEntry));
            return newEntry;
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public void setValue(ReferenceEntry<K, V> entry, V value) {
            entry.setValueReference(this.map.valueStrength.referenceValue(this, entry, value));
            recordWrite(entry);
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
        public void recordRead(ReferenceEntry<K, V> entry) {
            if (this.map.expiresAfterAccess()) {
                recordExpirationTime(entry, this.map.expireAfterAccessNanos);
            }
            this.recencyQueue.add(entry);
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public void recordLockedRead(ReferenceEntry<K, V> entry) {
            this.evictionQueue.add(entry);
            if (this.map.expiresAfterAccess()) {
                recordExpirationTime(entry, this.map.expireAfterAccessNanos);
                this.expirationQueue.add(entry);
            }
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public void recordWrite(ReferenceEntry<K, V> entry) {
            drainRecencyQueue();
            this.evictionQueue.add(entry);
            if (this.map.expires()) {
                recordExpirationTime(entry, this.map.expiresAfterAccess() ? this.map.expireAfterAccessNanos : this.map.expireAfterWriteNanos);
                this.expirationQueue.add(entry);
            }
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public void drainRecencyQueue() {
            while (true) {
                ReferenceEntry<K, V> poll = this.recencyQueue.poll();
                ReferenceEntry<K, V> e = poll;
                if (poll != null) {
                    if (this.evictionQueue.contains(e)) {
                        this.evictionQueue.add(e);
                    }
                    if (this.map.expiresAfterAccess() && this.expirationQueue.contains(e)) {
                        this.expirationQueue.add(e);
                    }
                } else {
                    return;
                }
            }
        }

        /* access modifiers changed from: package-private */
        public void recordExpirationTime(ReferenceEntry<K, V> entry, long expirationNanos) {
            entry.setExpirationTime(this.map.ticker.read() + expirationNanos);
        }

        /* access modifiers changed from: package-private */
        public void tryExpireEntries() {
            if (tryLock()) {
                try {
                    expireEntries();
                } finally {
                    unlock();
                }
            }
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public void expireEntries() {
            ReferenceEntry<K, V> e;
            drainRecencyQueue();
            if (!this.expirationQueue.isEmpty()) {
                long now = this.map.ticker.read();
                do {
                    ReferenceEntry<K, V> peek = this.expirationQueue.peek();
                    e = peek;
                    if (peek == null || !this.map.isExpired(e, now)) {
                        return;
                    }
                } while (removeEntry(e, e.getHash(), MapMaker.RemovalCause.EXPIRED));
                throw new AssertionError();
            }
        }

        /* access modifiers changed from: package-private */
        public void enqueueNotification(ReferenceEntry<K, V> entry, MapMaker.RemovalCause cause) {
            enqueueNotification(entry.getKey(), entry.getHash(), entry.getValueReference().get(), cause);
        }

        /* access modifiers changed from: package-private */
        public void enqueueNotification(@Nullable K key, int hash, @Nullable V value, MapMaker.RemovalCause cause) {
            if (this.map.removalNotificationQueue != MapMakerInternalMap.DISCARDING_QUEUE) {
                this.map.removalNotificationQueue.offer(new MapMaker.RemovalNotification<>(key, value, cause));
            }
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public boolean evictEntries() {
            if (!this.map.evictsBySize() || this.count < this.maxSegmentSize) {
                return false;
            }
            drainRecencyQueue();
            ReferenceEntry<K, V> e = this.evictionQueue.remove();
            if (removeEntry(e, e.getHash(), MapMaker.RemovalCause.SIZE)) {
                return true;
            }
            throw new AssertionError();
        }

        /* access modifiers changed from: package-private */
        public ReferenceEntry<K, V> getFirst(int hash) {
            AtomicReferenceArray<ReferenceEntry<K, V>> table2 = this.table;
            return table2.get((table2.length() - 1) & hash);
        }

        /* access modifiers changed from: package-private */
        public ReferenceEntry<K, V> getEntry(Object key, int hash) {
            if (this.count == 0) {
                return null;
            }
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
        public ReferenceEntry<K, V> getLiveEntry(Object key, int hash) {
            ReferenceEntry<K, V> e = getEntry(key, hash);
            if (e == null) {
                return null;
            }
            if (!this.map.expires() || !this.map.isExpired(e)) {
                return e;
            }
            tryExpireEntries();
            return null;
        }

        /* access modifiers changed from: package-private */
        public V get(Object key, int hash) {
            V value;
            try {
                ReferenceEntry<K, V> e = getLiveEntry(key, hash);
                if (e == null) {
                    value = null;
                } else {
                    value = e.getValueReference().get();
                    if (value != null) {
                        recordRead(e);
                    } else {
                        tryDrainReferenceQueues();
                    }
                }
                return value;
            } finally {
                postReadCleanup();
            }
        }

        /* access modifiers changed from: package-private */
        public boolean containsKey(Object key, int hash) {
            ReferenceEntry<K, V> e;
            try {
                boolean z = false;
                if (!(this.count == 0 || (e = getLiveEntry(key, hash)) == null)) {
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
                    AtomicReferenceArray<ReferenceEntry<K, V>> table2 = this.table;
                    int length = table2.length();
                    for (int i = 0; i < length; i++) {
                        for (ReferenceEntry<K, V> e = table2.get(i); e != null; e = e.getNext()) {
                            V entryValue = getLiveValue(e);
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
        /* JADX WARNING: Code restructure failed: missing block: B:13:0x003e, code lost:
            r7 = r4.getValueReference();
            r8 = r7.get();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:14:0x0046, code lost:
            if (r8 != null) goto L_0x0074;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:15:0x0048, code lost:
            r10.modCount++;
            setValue(r4, r13);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:16:0x0055, code lost:
            if (r7.isComputingReference() != false) goto L_0x0060;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:17:0x0057, code lost:
            enqueueNotification(r11, r12, r8, com.google.common.collect.MapMaker.RemovalCause.COLLECTED);
            r0 = r10.count;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:19:0x0064, code lost:
            if (evictEntries() == false) goto L_0x006a;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:20:0x0066, code lost:
            r0 = r10.count + 1;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:21:0x006a, code lost:
            r10.count = r0;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:24:0x0074, code lost:
            if (r14 == false) goto L_0x0081;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:26:?, code lost:
            recordLockedRead(r4);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:27:0x007a, code lost:
            unlock();
            postWriteCleanup();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:28:0x0080, code lost:
            return r8;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:30:?, code lost:
            r10.modCount++;
            enqueueNotification(r11, r12, r8, com.google.common.collect.MapMaker.RemovalCause.REPLACED);
            setValue(r4, r13);
         */
        /* Code decompiled incorrectly, please refer to instructions dump. */
        public V put(K r11, int r12, V r13, boolean r14) {
            /*
                r10 = this;
                r10.lock()
                r10.preWriteCleanup()     // Catch:{ all -> 0x00b3 }
                int r0 = r10.count     // Catch:{ all -> 0x00b3 }
                int r0 = r0 + 1
                int r1 = r10.threshold     // Catch:{ all -> 0x00b3 }
                if (r0 <= r1) goto L_0x0015
                r10.expand()     // Catch:{ all -> 0x00b3 }
                int r1 = r10.count     // Catch:{ all -> 0x00b3 }
                int r0 = r1 + 1
            L_0x0015:
                java.util.concurrent.atomic.AtomicReferenceArray<com.google.common.collect.MapMakerInternalMap$ReferenceEntry<K, V>> r1 = r10.table     // Catch:{ all -> 0x00b3 }
                int r2 = r1.length()     // Catch:{ all -> 0x00b3 }
                int r2 = r2 + -1
                r2 = r2 & r12
                java.lang.Object r3 = r1.get(r2)     // Catch:{ all -> 0x00b3 }
                com.google.common.collect.MapMakerInternalMap$ReferenceEntry r3 = (com.google.common.collect.MapMakerInternalMap.ReferenceEntry) r3     // Catch:{ all -> 0x00b3 }
                r4 = r3
            L_0x0025:
                r5 = 0
                if (r4 == 0) goto L_0x0096
                java.lang.Object r6 = r4.getKey()     // Catch:{ all -> 0x00b3 }
                int r7 = r4.getHash()     // Catch:{ all -> 0x00b3 }
                if (r7 != r12) goto L_0x0090
                if (r6 == 0) goto L_0x0090
                com.google.common.collect.MapMakerInternalMap<K, V> r7 = r10.map     // Catch:{ all -> 0x00b3 }
                com.google.common.base.Equivalence<java.lang.Object> r7 = r7.keyEquivalence     // Catch:{ all -> 0x00b3 }
                boolean r7 = r7.equivalent(r11, r6)     // Catch:{ all -> 0x00b3 }
                if (r7 == 0) goto L_0x0090
                com.google.common.collect.MapMakerInternalMap$ValueReference r7 = r4.getValueReference()     // Catch:{ all -> 0x00b3 }
                java.lang.Object r8 = r7.get()     // Catch:{ all -> 0x00b3 }
                if (r8 != 0) goto L_0x0074
                int r9 = r10.modCount     // Catch:{ all -> 0x00b3 }
                int r9 = r9 + 1
                r10.modCount = r9     // Catch:{ all -> 0x00b3 }
                r10.setValue(r4, r13)     // Catch:{ all -> 0x00b3 }
                boolean r9 = r7.isComputingReference()     // Catch:{ all -> 0x00b3 }
                if (r9 != 0) goto L_0x0060
                com.google.common.collect.MapMaker$RemovalCause r9 = com.google.common.collect.MapMaker.RemovalCause.COLLECTED     // Catch:{ all -> 0x00b3 }
                r10.enqueueNotification(r11, r12, r8, r9)     // Catch:{ all -> 0x00b3 }
                int r9 = r10.count     // Catch:{ all -> 0x00b3 }
                r0 = r9
                goto L_0x006a
            L_0x0060:
                boolean r9 = r10.evictEntries()     // Catch:{ all -> 0x00b3 }
                if (r9 == 0) goto L_0x006a
                int r9 = r10.count     // Catch:{ all -> 0x00b3 }
                int r0 = r9 + 1
            L_0x006a:
                r10.count = r0     // Catch:{ all -> 0x00b3 }
            L_0x006d:
                r10.unlock()
                r10.postWriteCleanup()
                return r5
            L_0x0074:
                if (r14 == 0) goto L_0x0081
                r10.recordLockedRead(r4)     // Catch:{ all -> 0x00b3 }
            L_0x007a:
                r10.unlock()
                r10.postWriteCleanup()
                return r8
            L_0x0081:
                int r5 = r10.modCount     // Catch:{ all -> 0x00b3 }
                int r5 = r5 + 1
                r10.modCount = r5     // Catch:{ all -> 0x00b3 }
                com.google.common.collect.MapMaker$RemovalCause r5 = com.google.common.collect.MapMaker.RemovalCause.REPLACED     // Catch:{ all -> 0x00b3 }
                r10.enqueueNotification(r11, r12, r8, r5)     // Catch:{ all -> 0x00b3 }
                r10.setValue(r4, r13)     // Catch:{ all -> 0x00b3 }
                goto L_0x007a
            L_0x0090:
                com.google.common.collect.MapMakerInternalMap$ReferenceEntry r5 = r4.getNext()     // Catch:{ all -> 0x00b3 }
                r4 = r5
                goto L_0x0025
            L_0x0096:
                int r4 = r10.modCount     // Catch:{ all -> 0x00b3 }
                int r4 = r4 + 1
                r10.modCount = r4     // Catch:{ all -> 0x00b3 }
                com.google.common.collect.MapMakerInternalMap$ReferenceEntry r4 = r10.newEntry(r11, r12, r3)     // Catch:{ all -> 0x00b3 }
                r10.setValue(r4, r13)     // Catch:{ all -> 0x00b3 }
                r1.set(r2, r4)     // Catch:{ all -> 0x00b3 }
                boolean r6 = r10.evictEntries()     // Catch:{ all -> 0x00b3 }
                if (r6 == 0) goto L_0x00b0
                int r6 = r10.count     // Catch:{ all -> 0x00b3 }
                int r0 = r6 + 1
            L_0x00b0:
                r10.count = r0     // Catch:{ all -> 0x00b3 }
                goto L_0x006d
            L_0x00b3:
                r0 = move-exception
                r10.unlock()
                r10.postWriteCleanup()
                throw r0
            */
            throw new UnsupportedOperationException("Method not decompiled: com.google.common.collect.MapMakerInternalMap.Segment.put(java.lang.Object, int, java.lang.Object, boolean):java.lang.Object");
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
        /* JADX WARNING: Code restructure failed: missing block: B:10:0x0034, code lost:
            r10 = r7.getValueReference();
            r11 = r10.get();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:11:0x003c, code lost:
            if (r11 != null) goto L_0x0065;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:13:0x0042, code lost:
            if (isCollected(r10) == false) goto L_0x005e;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:14:0x0044, code lost:
            r12 = r1.count - 1;
            r1.modCount++;
            enqueueNotification(r9, r3, r11, com.google.common.collect.MapMaker.RemovalCause.COLLECTED);
            r0.set(r4, removeFromChain(r6, r7));
            r1.count--;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:22:0x006f, code lost:
            if (r1.map.valueEquivalence.equivalent(r18, r11) == false) goto L_0x0088;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:23:0x0071, code lost:
            r1.modCount++;
            enqueueNotification(r2, r3, r11, com.google.common.collect.MapMaker.RemovalCause.REPLACED);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:26:?, code lost:
            setValue(r7, r19);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:27:0x0080, code lost:
            unlock();
            postWriteCleanup();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:28:0x0087, code lost:
            return true;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:29:0x0088, code lost:
            r12 = r19;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:31:?, code lost:
            recordLockedRead(r7);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:32:0x008e, code lost:
            r0 = th;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:36:0x009a, code lost:
            r0 = th;
         */
        /* Code decompiled incorrectly, please refer to instructions dump. */
        public boolean replace(K r16, int r17, V r18, V r19) {
            /*
                r15 = this;
                r1 = r15
                r2 = r16
                r3 = r17
                r15.lock()
                r15.preWriteCleanup()     // Catch:{ all -> 0x00a1 }
                java.util.concurrent.atomic.AtomicReferenceArray<com.google.common.collect.MapMakerInternalMap$ReferenceEntry<K, V>> r0 = r1.table     // Catch:{ all -> 0x00a1 }
                int r4 = r0.length()     // Catch:{ all -> 0x00a1 }
                r5 = 1
                int r4 = r4 - r5
                r4 = r4 & r3
                java.lang.Object r6 = r0.get(r4)     // Catch:{ all -> 0x00a1 }
                com.google.common.collect.MapMakerInternalMap$ReferenceEntry r6 = (com.google.common.collect.MapMakerInternalMap.ReferenceEntry) r6     // Catch:{ all -> 0x00a1 }
                r7 = r6
            L_0x001b:
                r8 = 0
                if (r7 == 0) goto L_0x009c
                java.lang.Object r9 = r7.getKey()     // Catch:{ all -> 0x00a1 }
                int r10 = r7.getHash()     // Catch:{ all -> 0x00a1 }
                if (r10 != r3) goto L_0x0090
                if (r9 == 0) goto L_0x0090
                com.google.common.collect.MapMakerInternalMap<K, V> r10 = r1.map     // Catch:{ all -> 0x00a1 }
                com.google.common.base.Equivalence<java.lang.Object> r10 = r10.keyEquivalence     // Catch:{ all -> 0x00a1 }
                boolean r10 = r10.equivalent(r2, r9)     // Catch:{ all -> 0x00a1 }
                if (r10 == 0) goto L_0x0090
                com.google.common.collect.MapMakerInternalMap$ValueReference r10 = r7.getValueReference()     // Catch:{ all -> 0x00a1 }
                java.lang.Object r11 = r10.get()     // Catch:{ all -> 0x00a1 }
                if (r11 != 0) goto L_0x0065
                boolean r12 = r15.isCollected(r10)     // Catch:{ all -> 0x00a1 }
                if (r12 == 0) goto L_0x005d
                int r12 = r1.count     // Catch:{ all -> 0x00a1 }
                int r12 = r12 - r5
                int r13 = r1.modCount     // Catch:{ all -> 0x00a1 }
                int r13 = r13 + r5
                r1.modCount = r13     // Catch:{ all -> 0x00a1 }
                com.google.common.collect.MapMaker$RemovalCause r13 = com.google.common.collect.MapMaker.RemovalCause.COLLECTED     // Catch:{ all -> 0x00a1 }
                r15.enqueueNotification(r9, r3, r11, r13)     // Catch:{ all -> 0x00a1 }
                com.google.common.collect.MapMakerInternalMap$ReferenceEntry r13 = r15.removeFromChain(r6, r7)     // Catch:{ all -> 0x00a1 }
                int r14 = r1.count     // Catch:{ all -> 0x00a1 }
                int r14 = r14 - r5
                r0.set(r4, r13)     // Catch:{ all -> 0x00a1 }
                r1.count = r14     // Catch:{ all -> 0x00a1 }
            L_0x005d:
            L_0x005e:
                r15.unlock()
                r15.postWriteCleanup()
                return r8
            L_0x0065:
                com.google.common.collect.MapMakerInternalMap<K, V> r12 = r1.map     // Catch:{ all -> 0x00a1 }
                com.google.common.base.Equivalence<java.lang.Object> r12 = r12.valueEquivalence     // Catch:{ all -> 0x00a1 }
                r13 = r18
                boolean r12 = r12.equivalent(r13, r11)     // Catch:{ all -> 0x008e }
                if (r12 == 0) goto L_0x0088
                int r8 = r1.modCount     // Catch:{ all -> 0x008e }
                int r8 = r8 + r5
                r1.modCount = r8     // Catch:{ all -> 0x008e }
                com.google.common.collect.MapMaker$RemovalCause r8 = com.google.common.collect.MapMaker.RemovalCause.REPLACED     // Catch:{ all -> 0x008e }
                r15.enqueueNotification(r2, r3, r11, r8)     // Catch:{ all -> 0x008e }
                r12 = r19
                r15.setValue(r7, r12)     // Catch:{ all -> 0x009a }
                r15.unlock()
                r15.postWriteCleanup()
                return r5
            L_0x0088:
                r12 = r19
                r15.recordLockedRead(r7)     // Catch:{ all -> 0x009a }
                goto L_0x005e
            L_0x008e:
                r0 = move-exception
                goto L_0x00a4
            L_0x0090:
                r13 = r18
                r12 = r19
                com.google.common.collect.MapMakerInternalMap$ReferenceEntry r8 = r7.getNext()     // Catch:{ all -> 0x009a }
                r7 = r8
                goto L_0x001b
            L_0x009a:
                r0 = move-exception
                goto L_0x00a6
            L_0x009c:
                r13 = r18
                r12 = r19
                goto L_0x005e
            L_0x00a1:
                r0 = move-exception
                r13 = r18
            L_0x00a4:
                r12 = r19
            L_0x00a6:
                r15.unlock()
                r15.postWriteCleanup()
                throw r0
            */
            throw new UnsupportedOperationException("Method not decompiled: com.google.common.collect.MapMakerInternalMap.Segment.replace(java.lang.Object, int, java.lang.Object, java.lang.Object):boolean");
        }

        /* access modifiers changed from: package-private */
        /* JADX WARNING: Code restructure failed: missing block: B:10:0x002f, code lost:
            r6 = r3.getValueReference();
            r7 = r6.get();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:11:0x0037, code lost:
            if (r7 != null) goto L_0x0063;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:13:0x003d, code lost:
            if (isCollected(r6) == false) goto L_0x005c;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:14:0x003f, code lost:
            r8 = r11.count - 1;
            r11.modCount++;
            enqueueNotification(r5, r13, r7, com.google.common.collect.MapMaker.RemovalCause.COLLECTED);
            r0.set(r1, removeFromChain(r2, r3));
            r11.count--;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:18:?, code lost:
            r11.modCount++;
            enqueueNotification(r12, r13, r7, com.google.common.collect.MapMaker.RemovalCause.REPLACED);
            setValue(r3, r14);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:19:0x0071, code lost:
            unlock();
            postWriteCleanup();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:20:0x0078, code lost:
            return r7;
         */
        /* Code decompiled incorrectly, please refer to instructions dump. */
        public V replace(K r12, int r13, V r14) {
            /*
                r11 = this;
                r11.lock()
                r11.preWriteCleanup()     // Catch:{ all -> 0x0080 }
                java.util.concurrent.atomic.AtomicReferenceArray<com.google.common.collect.MapMakerInternalMap$ReferenceEntry<K, V>> r0 = r11.table     // Catch:{ all -> 0x0080 }
                int r1 = r0.length()     // Catch:{ all -> 0x0080 }
                int r1 = r1 + -1
                r1 = r1 & r13
                java.lang.Object r2 = r0.get(r1)     // Catch:{ all -> 0x0080 }
                com.google.common.collect.MapMakerInternalMap$ReferenceEntry r2 = (com.google.common.collect.MapMakerInternalMap.ReferenceEntry) r2     // Catch:{ all -> 0x0080 }
                r3 = r2
            L_0x0016:
                r4 = 0
                if (r3 == 0) goto L_0x007f
                java.lang.Object r5 = r3.getKey()     // Catch:{ all -> 0x0080 }
                int r6 = r3.getHash()     // Catch:{ all -> 0x0080 }
                if (r6 != r13) goto L_0x0079
                if (r5 == 0) goto L_0x0079
                com.google.common.collect.MapMakerInternalMap<K, V> r6 = r11.map     // Catch:{ all -> 0x0080 }
                com.google.common.base.Equivalence<java.lang.Object> r6 = r6.keyEquivalence     // Catch:{ all -> 0x0080 }
                boolean r6 = r6.equivalent(r12, r5)     // Catch:{ all -> 0x0080 }
                if (r6 == 0) goto L_0x0079
                com.google.common.collect.MapMakerInternalMap$ValueReference r6 = r3.getValueReference()     // Catch:{ all -> 0x0080 }
                java.lang.Object r7 = r6.get()     // Catch:{ all -> 0x0080 }
                if (r7 != 0) goto L_0x0063
                boolean r8 = r11.isCollected(r6)     // Catch:{ all -> 0x0080 }
                if (r8 == 0) goto L_0x005b
                int r8 = r11.count     // Catch:{ all -> 0x0080 }
                int r8 = r8 + -1
                int r9 = r11.modCount     // Catch:{ all -> 0x0080 }
                int r9 = r9 + 1
                r11.modCount = r9     // Catch:{ all -> 0x0080 }
                com.google.common.collect.MapMaker$RemovalCause r9 = com.google.common.collect.MapMaker.RemovalCause.COLLECTED     // Catch:{ all -> 0x0080 }
                r11.enqueueNotification(r5, r13, r7, r9)     // Catch:{ all -> 0x0080 }
                com.google.common.collect.MapMakerInternalMap$ReferenceEntry r9 = r11.removeFromChain(r2, r3)     // Catch:{ all -> 0x0080 }
                int r10 = r11.count     // Catch:{ all -> 0x0080 }
                int r10 = r10 + -1
                r0.set(r1, r9)     // Catch:{ all -> 0x0080 }
                r11.count = r10     // Catch:{ all -> 0x0080 }
            L_0x005b:
            L_0x005c:
                r11.unlock()
                r11.postWriteCleanup()
                return r4
            L_0x0063:
                int r4 = r11.modCount     // Catch:{ all -> 0x0080 }
                int r4 = r4 + 1
                r11.modCount = r4     // Catch:{ all -> 0x0080 }
                com.google.common.collect.MapMaker$RemovalCause r4 = com.google.common.collect.MapMaker.RemovalCause.REPLACED     // Catch:{ all -> 0x0080 }
                r11.enqueueNotification(r12, r13, r7, r4)     // Catch:{ all -> 0x0080 }
                r11.setValue(r3, r14)     // Catch:{ all -> 0x0080 }
                r11.unlock()
                r11.postWriteCleanup()
                return r7
            L_0x0079:
                com.google.common.collect.MapMakerInternalMap$ReferenceEntry r4 = r3.getNext()     // Catch:{ all -> 0x0080 }
                r3 = r4
                goto L_0x0016
            L_0x007f:
                goto L_0x005c
            L_0x0080:
                r0 = move-exception
                r11.unlock()
                r11.postWriteCleanup()
                throw r0
            */
            throw new UnsupportedOperationException("Method not decompiled: com.google.common.collect.MapMakerInternalMap.Segment.replace(java.lang.Object, int, java.lang.Object):java.lang.Object");
        }

        /* access modifiers changed from: package-private */
        public V remove(Object key, int hash) {
            MapMaker.RemovalCause cause;
            lock();
            try {
                preWriteCleanup();
                int i = this.count - 1;
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
                    } else {
                        ValueReference<K, V> valueReference = e.getValueReference();
                        V entryValue = valueReference.get();
                        if (entryValue != null) {
                            cause = MapMaker.RemovalCause.EXPLICIT;
                        } else if (isCollected(valueReference)) {
                            cause = MapMaker.RemovalCause.COLLECTED;
                        }
                        this.modCount++;
                        enqueueNotification(entryKey, hash, entryValue, cause);
                        table2.set(index, removeFromChain(first, e));
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
        /* JADX WARNING: Code restructure failed: missing block: B:12:0x0037, code lost:
            r10 = r7.getValueReference();
            r12 = r10.get();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:16:0x0049, code lost:
            if (r1.map.valueEquivalence.equivalent(r20, r12) == false) goto L_0x004e;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:17:0x004b, code lost:
            r13 = com.google.common.collect.MapMaker.RemovalCause.EXPLICIT;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:19:0x0052, code lost:
            if (isCollected(r10) == false) goto L_0x007d;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:20:0x0054, code lost:
            r13 = com.google.common.collect.MapMaker.RemovalCause.COLLECTED;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:21:0x0056, code lost:
            r1.modCount++;
            enqueueNotification(r9, r2, r12, r13);
            r4.set(r5, removeFromChain(r6, r7));
            r1.count--;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:22:0x006d, code lost:
            if (r13 != com.google.common.collect.MapMaker.RemovalCause.EXPLICIT) goto L_0x0072;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:23:0x006f, code lost:
            r16 = true;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:24:0x0072, code lost:
            r16 = false;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:25:0x0075, code lost:
            unlock();
            postWriteCleanup();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:26:0x007b, code lost:
            return r16;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:35:0x0091, code lost:
            r0 = th;
         */
        /* Code decompiled incorrectly, please refer to instructions dump. */
        public boolean remove(java.lang.Object r18, int r19, java.lang.Object r20) {
            /*
                r17 = this;
                r1 = r17
                r2 = r19
                r17.lock()
                r17.preWriteCleanup()     // Catch:{ all -> 0x0098 }
                int r0 = r1.count     // Catch:{ all -> 0x0098 }
                r3 = 1
                int r0 = r0 - r3
                java.util.concurrent.atomic.AtomicReferenceArray<com.google.common.collect.MapMakerInternalMap$ReferenceEntry<K, V>> r4 = r1.table     // Catch:{ all -> 0x0098 }
                int r5 = r4.length()     // Catch:{ all -> 0x0098 }
                int r5 = r5 - r3
                r5 = r5 & r2
                java.lang.Object r6 = r4.get(r5)     // Catch:{ all -> 0x0098 }
                com.google.common.collect.MapMakerInternalMap$ReferenceEntry r6 = (com.google.common.collect.MapMakerInternalMap.ReferenceEntry) r6     // Catch:{ all -> 0x0098 }
                r7 = r6
            L_0x001d:
                if (r7 == 0) goto L_0x0093
                java.lang.Object r9 = r7.getKey()     // Catch:{ all -> 0x0098 }
                int r10 = r7.getHash()     // Catch:{ all -> 0x0098 }
                if (r10 != r2) goto L_0x0087
                if (r9 == 0) goto L_0x0087
                com.google.common.collect.MapMakerInternalMap<K, V> r10 = r1.map     // Catch:{ all -> 0x0098 }
                com.google.common.base.Equivalence<java.lang.Object> r10 = r10.keyEquivalence     // Catch:{ all -> 0x0098 }
                r11 = r18
                boolean r10 = r10.equivalent(r11, r9)     // Catch:{ all -> 0x0085 }
                if (r10 == 0) goto L_0x0089
                com.google.common.collect.MapMakerInternalMap$ValueReference r10 = r7.getValueReference()     // Catch:{ all -> 0x0085 }
                java.lang.Object r12 = r10.get()     // Catch:{ all -> 0x0085 }
                com.google.common.collect.MapMakerInternalMap<K, V> r13 = r1.map     // Catch:{ all -> 0x0085 }
                com.google.common.base.Equivalence<java.lang.Object> r13 = r13.valueEquivalence     // Catch:{ all -> 0x0085 }
                r14 = r20
                boolean r13 = r13.equivalent(r14, r12)     // Catch:{ all -> 0x0091 }
                if (r13 == 0) goto L_0x004e
                com.google.common.collect.MapMaker$RemovalCause r13 = com.google.common.collect.MapMaker.RemovalCause.EXPLICIT     // Catch:{ all -> 0x0091 }
                goto L_0x0056
            L_0x004e:
                boolean r13 = r1.isCollected(r10)     // Catch:{ all -> 0x0091 }
                if (r13 == 0) goto L_0x007c
                com.google.common.collect.MapMaker$RemovalCause r13 = com.google.common.collect.MapMaker.RemovalCause.COLLECTED     // Catch:{ all -> 0x0091 }
            L_0x0056:
                int r15 = r1.modCount     // Catch:{ all -> 0x0091 }
                int r15 = r15 + r3
                r1.modCount = r15     // Catch:{ all -> 0x0091 }
                r1.enqueueNotification(r9, r2, r12, r13)     // Catch:{ all -> 0x0091 }
                com.google.common.collect.MapMakerInternalMap$ReferenceEntry r15 = r1.removeFromChain(r6, r7)     // Catch:{ all -> 0x0091 }
                int r8 = r1.count     // Catch:{ all -> 0x0091 }
                int r8 = r8 - r3
                r4.set(r5, r15)     // Catch:{ all -> 0x0091 }
                r1.count = r8     // Catch:{ all -> 0x0091 }
                com.google.common.collect.MapMaker$RemovalCause r0 = com.google.common.collect.MapMaker.RemovalCause.EXPLICIT     // Catch:{ all -> 0x0091 }
                if (r13 != r0) goto L_0x0072
                r16 = 1
                goto L_0x0075
            L_0x0072:
                r16 = 0
            L_0x0075:
                r17.unlock()
                r17.postWriteCleanup()
                return r16
            L_0x007c:
            L_0x007d:
                r17.unlock()
                r17.postWriteCleanup()
                r3 = 0
                return r3
            L_0x0085:
                r0 = move-exception
                goto L_0x009b
            L_0x0087:
                r11 = r18
            L_0x0089:
                r14 = r20
                com.google.common.collect.MapMakerInternalMap$ReferenceEntry r8 = r7.getNext()     // Catch:{ all -> 0x0091 }
                r7 = r8
                goto L_0x001d
            L_0x0091:
                r0 = move-exception
                goto L_0x009d
            L_0x0093:
                r11 = r18
                r14 = r20
                goto L_0x007d
            L_0x0098:
                r0 = move-exception
                r11 = r18
            L_0x009b:
                r14 = r20
            L_0x009d:
                r17.unlock()
                r17.postWriteCleanup()
                throw r0
            */
            throw new UnsupportedOperationException("Method not decompiled: com.google.common.collect.MapMakerInternalMap.Segment.remove(java.lang.Object, int, java.lang.Object):boolean");
        }

        /* access modifiers changed from: package-private */
        public void clear() {
            if (this.count != 0) {
                lock();
                try {
                    AtomicReferenceArray<ReferenceEntry<K, V>> table2 = this.table;
                    if (this.map.removalNotificationQueue != MapMakerInternalMap.DISCARDING_QUEUE) {
                        for (int i = 0; i < table2.length(); i++) {
                            for (ReferenceEntry<K, V> e = table2.get(i); e != null; e = e.getNext()) {
                                if (!e.getValueReference().isComputingReference()) {
                                    enqueueNotification(e, MapMaker.RemovalCause.EXPLICIT);
                                }
                            }
                        }
                    }
                    for (int i2 = 0; i2 < table2.length(); i2++) {
                        table2.set(i2, (Object) null);
                    }
                    clearReferenceQueues();
                    this.evictionQueue.clear();
                    this.expirationQueue.clear();
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
        public ReferenceEntry<K, V> removeFromChain(ReferenceEntry<K, V> first, ReferenceEntry<K, V> entry) {
            this.evictionQueue.remove(entry);
            this.expirationQueue.remove(entry);
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
        public void removeCollectedEntry(ReferenceEntry<K, V> entry) {
            enqueueNotification(entry, MapMaker.RemovalCause.COLLECTED);
            this.evictionQueue.remove(entry);
            this.expirationQueue.remove(entry);
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
                        enqueueNotification(e.getKey(), hash, e.getValueReference().get(), MapMaker.RemovalCause.COLLECTED);
                        table2.set(index, removeFromChain(first, e));
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
        public boolean reclaimValue(K key, int hash, ValueReference<K, V> valueReference) {
            lock();
            try {
                int i = this.count - 1;
                AtomicReferenceArray<ReferenceEntry<K, V>> table2 = this.table;
                int index = (table2.length() - 1) & hash;
                ReferenceEntry<K, V> first = table2.get(index);
                ReferenceEntry<K, V> e = first;
                while (e != null) {
                    K entryKey = e.getKey();
                    if (e.getHash() != hash || entryKey == null || !this.map.keyEquivalence.equivalent(key, entryKey)) {
                        e = e.getNext();
                    } else if (e.getValueReference() == valueReference) {
                        this.modCount++;
                        enqueueNotification(key, hash, valueReference.get(), MapMaker.RemovalCause.COLLECTED);
                        table2.set(index, removeFromChain(first, e));
                        this.count--;
                        return true;
                    } else {
                        unlock();
                        if (!isHeldByCurrentThread()) {
                            postWriteCleanup();
                        }
                        return false;
                    }
                }
                unlock();
                if (!isHeldByCurrentThread()) {
                    postWriteCleanup();
                }
                return false;
            } finally {
                unlock();
                if (!isHeldByCurrentThread()) {
                    postWriteCleanup();
                }
            }
        }

        /* access modifiers changed from: package-private */
        public boolean clearValue(K key, int hash, ValueReference<K, V> valueReference) {
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
                        table2.set(index, removeFromChain(first, e));
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
        public boolean removeEntry(ReferenceEntry<K, V> entry, int hash, MapMaker.RemovalCause cause) {
            int i = this.count - 1;
            AtomicReferenceArray<ReferenceEntry<K, V>> table2 = this.table;
            int index = (table2.length() - 1) & hash;
            ReferenceEntry<K, V> first = table2.get(index);
            for (ReferenceEntry<K, V> e = first; e != null; e = e.getNext()) {
                if (e == entry) {
                    this.modCount++;
                    enqueueNotification(e.getKey(), hash, e.getValueReference().get(), cause);
                    table2.set(index, removeFromChain(first, e));
                    this.count--;
                    return true;
                }
            }
            return false;
        }

        /* access modifiers changed from: package-private */
        public boolean isCollected(ReferenceEntry<K, V> entry) {
            if (entry.getKey() == null) {
                return true;
            }
            return isCollected(entry.getValueReference());
        }

        /* access modifiers changed from: package-private */
        public boolean isCollected(ValueReference<K, V> valueReference) {
            if (!valueReference.isComputingReference() && valueReference.get() == null) {
                return true;
            }
            return false;
        }

        /* access modifiers changed from: package-private */
        public V getLiveValue(ReferenceEntry<K, V> entry) {
            if (entry.getKey() == null) {
                tryDrainReferenceQueues();
                return null;
            }
            V value = entry.getValueReference().get();
            if (value == null) {
                tryDrainReferenceQueues();
                return null;
            } else if (!this.map.expires() || !this.map.isExpired(entry)) {
                return value;
            } else {
                tryExpireEntries();
                return null;
            }
        }

        /* access modifiers changed from: package-private */
        public void postReadCleanup() {
            if ((this.readCount.incrementAndGet() & 63) == 0) {
                runCleanup();
            }
        }

        /* access modifiers changed from: package-private */
        @GuardedBy("Segment.this")
        public void preWriteCleanup() {
            runLockedCleanup();
        }

        /* access modifiers changed from: package-private */
        public void postWriteCleanup() {
            runUnlockedCleanup();
        }

        /* access modifiers changed from: package-private */
        public void runCleanup() {
            runLockedCleanup();
            runUnlockedCleanup();
        }

        /* access modifiers changed from: package-private */
        public void runLockedCleanup() {
            if (tryLock()) {
                try {
                    drainReferenceQueues();
                    expireEntries();
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

    static final class EvictionQueue<K, V> extends AbstractQueue<ReferenceEntry<K, V>> {
        final ReferenceEntry<K, V> head = new AbstractReferenceEntry<K, V>() {
            ReferenceEntry<K, V> nextEvictable = this;
            ReferenceEntry<K, V> previousEvictable = this;

            public ReferenceEntry<K, V> getNextEvictable() {
                return this.nextEvictable;
            }

            public void setNextEvictable(ReferenceEntry<K, V> next) {
                this.nextEvictable = next;
            }

            public ReferenceEntry<K, V> getPreviousEvictable() {
                return this.previousEvictable;
            }

            public void setPreviousEvictable(ReferenceEntry<K, V> previous) {
                this.previousEvictable = previous;
            }
        };

        EvictionQueue() {
        }

        public boolean offer(ReferenceEntry<K, V> entry) {
            MapMakerInternalMap.connectEvictables(entry.getPreviousEvictable(), entry.getNextEvictable());
            MapMakerInternalMap.connectEvictables(this.head.getPreviousEvictable(), entry);
            MapMakerInternalMap.connectEvictables(entry, this.head);
            return true;
        }

        public ReferenceEntry<K, V> peek() {
            ReferenceEntry<K, V> next = this.head.getNextEvictable();
            if (next == this.head) {
                return null;
            }
            return next;
        }

        public ReferenceEntry<K, V> poll() {
            ReferenceEntry<K, V> next = this.head.getNextEvictable();
            if (next == this.head) {
                return null;
            }
            remove(next);
            return next;
        }

        public boolean remove(Object o) {
            ReferenceEntry<K, V> e = (ReferenceEntry) o;
            ReferenceEntry<K, V> previous = e.getPreviousEvictable();
            ReferenceEntry<K, V> next = e.getNextEvictable();
            MapMakerInternalMap.connectEvictables(previous, next);
            MapMakerInternalMap.nullifyEvictable(e);
            return next != NullEntry.INSTANCE;
        }

        public boolean contains(Object o) {
            return ((ReferenceEntry) o).getNextEvictable() != NullEntry.INSTANCE;
        }

        public boolean isEmpty() {
            return this.head.getNextEvictable() == this.head;
        }

        public int size() {
            int size = 0;
            ReferenceEntry<K, V> e = this.head;
            while (true) {
                e = e.getNextEvictable();
                if (e == this.head) {
                    return size;
                }
                size++;
            }
        }

        public void clear() {
            ReferenceEntry<K, V> e = this.head.getNextEvictable();
            while (e != this.head) {
                ReferenceEntry<K, V> next = e.getNextEvictable();
                MapMakerInternalMap.nullifyEvictable(e);
                e = next;
            }
            this.head.setNextEvictable(this.head);
            this.head.setPreviousEvictable(this.head);
        }

        public Iterator<ReferenceEntry<K, V>> iterator() {
            return new AbstractSequentialIterator<ReferenceEntry<K, V>>(peek()) {
                /* access modifiers changed from: protected */
                public ReferenceEntry<K, V> computeNext(ReferenceEntry<K, V> previous) {
                    ReferenceEntry<K, V> next = previous.getNextEvictable();
                    if (next == EvictionQueue.this.head) {
                        return null;
                    }
                    return next;
                }
            };
        }
    }

    static final class ExpirationQueue<K, V> extends AbstractQueue<ReferenceEntry<K, V>> {
        final ReferenceEntry<K, V> head = new AbstractReferenceEntry<K, V>() {
            ReferenceEntry<K, V> nextExpirable = this;
            ReferenceEntry<K, V> previousExpirable = this;

            public long getExpirationTime() {
                return Long.MAX_VALUE;
            }

            public void setExpirationTime(long time) {
            }

            public ReferenceEntry<K, V> getNextExpirable() {
                return this.nextExpirable;
            }

            public void setNextExpirable(ReferenceEntry<K, V> next) {
                this.nextExpirable = next;
            }

            public ReferenceEntry<K, V> getPreviousExpirable() {
                return this.previousExpirable;
            }

            public void setPreviousExpirable(ReferenceEntry<K, V> previous) {
                this.previousExpirable = previous;
            }
        };

        ExpirationQueue() {
        }

        public boolean offer(ReferenceEntry<K, V> entry) {
            MapMakerInternalMap.connectExpirables(entry.getPreviousExpirable(), entry.getNextExpirable());
            MapMakerInternalMap.connectExpirables(this.head.getPreviousExpirable(), entry);
            MapMakerInternalMap.connectExpirables(entry, this.head);
            return true;
        }

        public ReferenceEntry<K, V> peek() {
            ReferenceEntry<K, V> next = this.head.getNextExpirable();
            if (next == this.head) {
                return null;
            }
            return next;
        }

        public ReferenceEntry<K, V> poll() {
            ReferenceEntry<K, V> next = this.head.getNextExpirable();
            if (next == this.head) {
                return null;
            }
            remove(next);
            return next;
        }

        public boolean remove(Object o) {
            ReferenceEntry<K, V> e = (ReferenceEntry) o;
            ReferenceEntry<K, V> previous = e.getPreviousExpirable();
            ReferenceEntry<K, V> next = e.getNextExpirable();
            MapMakerInternalMap.connectExpirables(previous, next);
            MapMakerInternalMap.nullifyExpirable(e);
            return next != NullEntry.INSTANCE;
        }

        public boolean contains(Object o) {
            return ((ReferenceEntry) o).getNextExpirable() != NullEntry.INSTANCE;
        }

        public boolean isEmpty() {
            return this.head.getNextExpirable() == this.head;
        }

        public int size() {
            int size = 0;
            ReferenceEntry<K, V> e = this.head;
            while (true) {
                e = e.getNextExpirable();
                if (e == this.head) {
                    return size;
                }
                size++;
            }
        }

        public void clear() {
            ReferenceEntry<K, V> e = this.head.getNextExpirable();
            while (e != this.head) {
                ReferenceEntry<K, V> next = e.getNextExpirable();
                MapMakerInternalMap.nullifyExpirable(e);
                e = next;
            }
            this.head.setNextExpirable(this.head);
            this.head.setPreviousExpirable(this.head);
        }

        public Iterator<ReferenceEntry<K, V>> iterator() {
            return new AbstractSequentialIterator<ReferenceEntry<K, V>>(peek()) {
                /* access modifiers changed from: protected */
                public ReferenceEntry<K, V> computeNext(ReferenceEntry<K, V> previous) {
                    ReferenceEntry<K, V> next = previous.getNextExpirable();
                    if (next == ExpirationQueue.this.head) {
                        return null;
                    }
                    return next;
                }
            };
        }
    }

    static final class CleanupMapTask implements Runnable {
        final WeakReference<MapMakerInternalMap<?, ?>> mapReference;

        public CleanupMapTask(MapMakerInternalMap<?, ?> map) {
            this.mapReference = new WeakReference<>(map);
        }

        public void run() {
            MapMakerInternalMap<?, ?> map = (MapMakerInternalMap) this.mapReference.get();
            if (map != null) {
                for (Segment runCleanup : map.segments) {
                    runCleanup.runCleanup();
                }
                return;
            }
            throw new CancellationException();
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

    public int size() {
        Segment<K, V>[] segments2 = this.segments;
        long sum = 0;
        for (Segment<K, V> segment : segments2) {
            sum += (long) segment.count;
        }
        return Ints.saturatedCast(sum);
    }

    public V get(@Nullable Object key) {
        if (key == null) {
            return null;
        }
        int hash = hash(key);
        return segmentFor(hash).get(key, hash);
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
        return segmentFor(hash).getLiveEntry(key, hash);
    }

    public boolean containsKey(@Nullable Object key) {
        if (key == null) {
            return false;
        }
        int hash = hash(key);
        return segmentFor(hash).containsKey(key, hash);
    }

    public boolean containsValue(@Nullable Object value) {
        Segment[] arr$;
        Object obj = value;
        if (obj == null) {
            return false;
        }
        Segment[] segments2 = this.segments;
        long last = -1;
        int i = 0;
        while (i < 3) {
            Segment[] arr$2 = segments2;
            int len$ = arr$2.length;
            long sum = 0;
            int i$ = 0;
            while (i$ < len$) {
                Segment<K, V> segment = arr$2[i$];
                int i2 = segment.count;
                AtomicReferenceArray<ReferenceEntry<K, V>> table = segment.table;
                for (int j = 0; j < table.length(); j++) {
                    ReferenceEntry<K, V> e = table.get(j);
                    while (e != null) {
                        Segment<K, V>[] segments3 = segments2;
                        V v = segment.getLiveValue(e);
                        if (v != null) {
                            arr$ = arr$2;
                            if (this.valueEquivalence.equivalent(obj, v)) {
                                return true;
                            }
                        } else {
                            arr$ = arr$2;
                        }
                        e = e.getNext();
                        segments2 = segments3;
                        arr$2 = arr$;
                    }
                    Segment[] segmentArr = arr$2;
                }
                Segment[] segmentArr2 = arr$2;
                sum += (long) segment.modCount;
                i$++;
                segments2 = segments2;
            }
            Segment<K, V>[] segments4 = segments2;
            if (sum == last) {
                return false;
            }
            last = sum;
            i++;
            segments2 = segments4;
        }
        return false;
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
        MapMakerInternalMap<K, V>.WriteThroughEntry lastReturned;
        ReferenceEntry<K, V> nextEntry;
        MapMakerInternalMap<K, V>.WriteThroughEntry nextExternal;
        int nextSegmentIndex;
        int nextTableIndex = -1;

        HashIterator() {
            this.nextSegmentIndex = MapMakerInternalMap.this.segments.length - 1;
            advance();
        }

        /* access modifiers changed from: package-private */
        public final void advance() {
            this.nextExternal = null;
            if (!nextInChain() && !nextInTable()) {
                while (this.nextSegmentIndex >= 0) {
                    Segment<K, V>[] segmentArr = MapMakerInternalMap.this.segments;
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
                K key = entry.getKey();
                V value = MapMakerInternalMap.this.getLiveValue(entry);
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
        public MapMakerInternalMap<K, V>.WriteThroughEntry nextEntry() {
            if (this.nextExternal != null) {
                this.lastReturned = this.nextExternal;
                advance();
                return this.lastReturned;
            }
            throw new NoSuchElementException();
        }

        public void remove() {
            Preconditions.checkState(this.lastReturned != null);
            MapMakerInternalMap.this.remove(this.lastReturned.getKey());
            this.lastReturned = null;
        }
    }

    final class KeyIterator extends MapMakerInternalMap<K, V>.HashIterator implements Iterator<K> {
        KeyIterator() {
            super();
        }

        public K next() {
            return nextEntry().getKey();
        }
    }

    final class ValueIterator extends MapMakerInternalMap<K, V>.HashIterator implements Iterator<V> {
        ValueIterator() {
            super();
        }

        public V next() {
            return nextEntry().getValue();
        }
    }

    final class WriteThroughEntry extends AbstractMapEntry<K, V> {
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

        public V setValue(V newValue) {
            V oldValue = MapMakerInternalMap.this.put(this.key, newValue);
            this.value = newValue;
            return oldValue;
        }
    }

    final class EntryIterator extends MapMakerInternalMap<K, V>.HashIterator implements Iterator<Map.Entry<K, V>> {
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
            return MapMakerInternalMap.this.size();
        }

        public boolean isEmpty() {
            return MapMakerInternalMap.this.isEmpty();
        }

        public boolean contains(Object o) {
            return MapMakerInternalMap.this.containsKey(o);
        }

        public boolean remove(Object o) {
            return MapMakerInternalMap.this.remove(o) != null;
        }

        public void clear() {
            MapMakerInternalMap.this.clear();
        }
    }

    final class Values extends AbstractCollection<V> {
        Values() {
        }

        public Iterator<V> iterator() {
            return new ValueIterator();
        }

        public int size() {
            return MapMakerInternalMap.this.size();
        }

        public boolean isEmpty() {
            return MapMakerInternalMap.this.isEmpty();
        }

        public boolean contains(Object o) {
            return MapMakerInternalMap.this.containsValue(o);
        }

        public void clear() {
            MapMakerInternalMap.this.clear();
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
                com.google.common.collect.MapMakerInternalMap r3 = com.google.common.collect.MapMakerInternalMap.this
                java.lang.Object r3 = r3.get(r2)
                if (r3 == 0) goto L_0x0028
                com.google.common.collect.MapMakerInternalMap r4 = com.google.common.collect.MapMakerInternalMap.this
                com.google.common.base.Equivalence<java.lang.Object> r4 = r4.valueEquivalence
                java.lang.Object r5 = r0.getValue()
                boolean r4 = r4.equivalent(r5, r3)
                if (r4 == 0) goto L_0x0028
                r1 = 1
            L_0x0028:
                return r1
            */
            throw new UnsupportedOperationException("Method not decompiled: com.google.common.collect.MapMakerInternalMap.EntrySet.contains(java.lang.Object):boolean");
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
                com.google.common.collect.MapMakerInternalMap r3 = com.google.common.collect.MapMakerInternalMap.this
                java.lang.Object r4 = r0.getValue()
                boolean r3 = r3.remove(r2, r4)
                if (r3 == 0) goto L_0x001d
                r1 = 1
            L_0x001d:
                return r1
            */
            throw new UnsupportedOperationException("Method not decompiled: com.google.common.collect.MapMakerInternalMap.EntrySet.remove(java.lang.Object):boolean");
        }

        public int size() {
            return MapMakerInternalMap.this.size();
        }

        public boolean isEmpty() {
            return MapMakerInternalMap.this.isEmpty();
        }

        public void clear() {
            MapMakerInternalMap.this.clear();
        }
    }

    /* access modifiers changed from: package-private */
    public Object writeReplace() {
        return new SerializationProxy(this.keyStrength, this.valueStrength, this.keyEquivalence, this.valueEquivalence, this.expireAfterWriteNanos, this.expireAfterAccessNanos, this.maximumSize, this.concurrencyLevel, this.removalListener, this);
    }

    static abstract class AbstractSerializationProxy<K, V> extends ForwardingConcurrentMap<K, V> implements Serializable {
        private static final long serialVersionUID = 3;
        final int concurrencyLevel;
        transient ConcurrentMap<K, V> delegate;
        final long expireAfterAccessNanos;
        final long expireAfterWriteNanos;
        final Equivalence<Object> keyEquivalence;
        final Strength keyStrength;
        final int maximumSize;
        final MapMaker.RemovalListener<? super K, ? super V> removalListener;
        final Equivalence<Object> valueEquivalence;
        final Strength valueStrength;

        AbstractSerializationProxy(Strength keyStrength2, Strength valueStrength2, Equivalence<Object> keyEquivalence2, Equivalence<Object> valueEquivalence2, long expireAfterWriteNanos2, long expireAfterAccessNanos2, int maximumSize2, int concurrencyLevel2, MapMaker.RemovalListener<? super K, ? super V> removalListener2, ConcurrentMap<K, V> delegate2) {
            this.keyStrength = keyStrength2;
            this.valueStrength = valueStrength2;
            this.keyEquivalence = keyEquivalence2;
            this.valueEquivalence = valueEquivalence2;
            this.expireAfterWriteNanos = expireAfterWriteNanos2;
            this.expireAfterAccessNanos = expireAfterAccessNanos2;
            this.maximumSize = maximumSize2;
            this.concurrencyLevel = concurrencyLevel2;
            this.removalListener = removalListener2;
            this.delegate = delegate2;
        }

        /* access modifiers changed from: protected */
        public ConcurrentMap<K, V> delegate() {
            return this.delegate;
        }

        /* access modifiers changed from: package-private */
        public void writeMapTo(ObjectOutputStream out) throws IOException {
            out.writeInt(this.delegate.size());
            for (Map.Entry<K, V> entry : this.delegate.entrySet()) {
                out.writeObject(entry.getKey());
                out.writeObject(entry.getValue());
            }
            out.writeObject((Object) null);
        }

        /* access modifiers changed from: package-private */
        public MapMaker readMapMaker(ObjectInputStream in) throws IOException {
            MapMaker mapMaker = new MapMaker().initialCapacity(in.readInt()).setKeyStrength(this.keyStrength).setValueStrength(this.valueStrength).keyEquivalence((Equivalence) this.keyEquivalence).valueEquivalence((Equivalence) this.valueEquivalence).concurrencyLevel(this.concurrencyLevel);
            mapMaker.removalListener(this.removalListener);
            if (this.expireAfterWriteNanos > 0) {
                mapMaker.expireAfterWrite(this.expireAfterWriteNanos, TimeUnit.NANOSECONDS);
            }
            if (this.expireAfterAccessNanos > 0) {
                mapMaker.expireAfterAccess(this.expireAfterAccessNanos, TimeUnit.NANOSECONDS);
            }
            if (this.maximumSize != -1) {
                mapMaker.maximumSize(this.maximumSize);
            }
            return mapMaker;
        }

        /* access modifiers changed from: package-private */
        public void readEntries(ObjectInputStream in) throws IOException, ClassNotFoundException {
            while (true) {
                K key = in.readObject();
                if (key != null) {
                    this.delegate.put(key, in.readObject());
                } else {
                    return;
                }
            }
        }
    }

    private static final class SerializationProxy<K, V> extends AbstractSerializationProxy<K, V> {
        private static final long serialVersionUID = 3;

        SerializationProxy(Strength keyStrength, Strength valueStrength, Equivalence<Object> keyEquivalence, Equivalence<Object> valueEquivalence, long expireAfterWriteNanos, long expireAfterAccessNanos, int maximumSize, int concurrencyLevel, MapMaker.RemovalListener<? super K, ? super V> removalListener, ConcurrentMap<K, V> delegate) {
            super(keyStrength, valueStrength, keyEquivalence, valueEquivalence, expireAfterWriteNanos, expireAfterAccessNanos, maximumSize, concurrencyLevel, removalListener, delegate);
        }

        private void writeObject(ObjectOutputStream out) throws IOException {
            out.defaultWriteObject();
            writeMapTo(out);
        }

        private void readObject(ObjectInputStream in) throws IOException, ClassNotFoundException {
            in.defaultReadObject();
            this.delegate = readMapMaker(in).makeMap();
            readEntries(in);
        }

        private Object readResolve() {
            return this.delegate;
        }
    }
}
