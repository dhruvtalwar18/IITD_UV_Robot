package com.google.common.cache;

import com.google.common.annotations.Beta;
import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableMap;
import java.util.concurrent.ExecutionException;

@Beta
public abstract class ForwardingLoadingCache<K, V> extends ForwardingCache<K, V> implements LoadingCache<K, V> {
    /* access modifiers changed from: protected */
    public abstract LoadingCache<K, V> delegate();

    protected ForwardingLoadingCache() {
    }

    public V get(K key) throws ExecutionException {
        return delegate().get(key);
    }

    public V getUnchecked(K key) {
        return delegate().getUnchecked(key);
    }

    public ImmutableMap<K, V> getAll(Iterable<? extends K> keys) throws ExecutionException {
        return delegate().getAll(keys);
    }

    public V apply(K key) {
        return delegate().apply(key);
    }

    public void refresh(K key) {
        delegate().refresh(key);
    }

    @Beta
    public static abstract class SimpleForwardingLoadingCache<K, V> extends ForwardingLoadingCache<K, V> {
        private final LoadingCache<K, V> delegate;

        protected SimpleForwardingLoadingCache(LoadingCache<K, V> delegate2) {
            this.delegate = (LoadingCache) Preconditions.checkNotNull(delegate2);
        }

        /* access modifiers changed from: protected */
        public final LoadingCache<K, V> delegate() {
            return this.delegate;
        }
    }
}
