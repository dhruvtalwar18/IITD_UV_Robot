package com.google.common.cache;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import com.google.common.collect.ImmutableMap;
import com.google.common.collect.Maps;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.concurrent.ConcurrentMap;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.atomic.AtomicLong;

@GwtCompatible
@Beta
public abstract class AbstractCache<K, V> implements Cache<K, V> {

    @Beta
    public interface StatsCounter {
        void recordEviction();

        void recordHits(int i);

        void recordLoadException(long j);

        void recordLoadSuccess(long j);

        void recordMisses(int i);

        CacheStats snapshot();
    }

    protected AbstractCache() {
    }

    public V get(K k, Callable<? extends V> callable) throws ExecutionException {
        throw new UnsupportedOperationException();
    }

    public ImmutableMap<K, V> getAllPresent(Iterable<?> keys) {
        Map<K, V> result = Maps.newLinkedHashMap();
        for (Object next : keys) {
            if (!result.containsKey(next)) {
                result.put(next, getIfPresent(next));
            }
        }
        return ImmutableMap.copyOf(result);
    }

    public void put(K k, V v) {
        throw new UnsupportedOperationException();
    }

    public void putAll(Map<? extends K, ? extends V> m) {
        for (Map.Entry<? extends K, ? extends V> entry : m.entrySet()) {
            put(entry.getKey(), entry.getValue());
        }
    }

    public void cleanUp() {
    }

    public long size() {
        throw new UnsupportedOperationException();
    }

    public void invalidate(Object key) {
        throw new UnsupportedOperationException();
    }

    public void invalidateAll(Iterable<?> keys) {
        for (Object key : keys) {
            invalidate(key);
        }
    }

    public void invalidateAll() {
        throw new UnsupportedOperationException();
    }

    public CacheStats stats() {
        throw new UnsupportedOperationException();
    }

    public ConcurrentMap<K, V> asMap() {
        throw new UnsupportedOperationException();
    }

    @Beta
    public static class SimpleStatsCounter implements StatsCounter {
        private final AtomicLong evictionCount = new AtomicLong();
        private final AtomicLong hitCount = new AtomicLong();
        private final AtomicLong loadExceptionCount = new AtomicLong();
        private final AtomicLong loadSuccessCount = new AtomicLong();
        private final AtomicLong missCount = new AtomicLong();
        private final AtomicLong totalLoadTime = new AtomicLong();

        public void recordHits(int count) {
            this.hitCount.addAndGet((long) count);
        }

        public void recordMisses(int count) {
            this.missCount.addAndGet((long) count);
        }

        public void recordLoadSuccess(long loadTime) {
            this.loadSuccessCount.incrementAndGet();
            this.totalLoadTime.addAndGet(loadTime);
        }

        public void recordLoadException(long loadTime) {
            this.loadExceptionCount.incrementAndGet();
            this.totalLoadTime.addAndGet(loadTime);
        }

        public void recordEviction() {
            this.evictionCount.incrementAndGet();
        }

        public CacheStats snapshot() {
            return new CacheStats(this.hitCount.get(), this.missCount.get(), this.loadSuccessCount.get(), this.loadExceptionCount.get(), this.totalLoadTime.get(), this.evictionCount.get());
        }

        public void incrementBy(StatsCounter other) {
            CacheStats otherStats = other.snapshot();
            this.hitCount.addAndGet(otherStats.hitCount());
            this.missCount.addAndGet(otherStats.missCount());
            this.loadSuccessCount.addAndGet(otherStats.loadSuccessCount());
            this.loadExceptionCount.addAndGet(otherStats.loadExceptionCount());
            this.totalLoadTime.addAndGet(otherStats.totalLoadTime());
            this.evictionCount.addAndGet(otherStats.evictionCount());
        }
    }
}
