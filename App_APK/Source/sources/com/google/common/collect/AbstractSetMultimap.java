package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import java.util.Collection;
import java.util.Map;
import java.util.Set;
import javax.annotation.Nullable;

@GwtCompatible
abstract class AbstractSetMultimap<K, V> extends AbstractMultimap<K, V> implements SetMultimap<K, V> {
    private static final long serialVersionUID = 7431625294878419160L;

    /* access modifiers changed from: package-private */
    public abstract Set<V> createCollection();

    protected AbstractSetMultimap(Map<K, Collection<V>> map) {
        super(map);
    }

    public Set<V> get(@Nullable K key) {
        return (Set) super.get(key);
    }

    public Set<Map.Entry<K, V>> entries() {
        return (Set) super.entries();
    }

    public Set<V> removeAll(@Nullable Object key) {
        return (Set) super.removeAll(key);
    }

    public Set<V> replaceValues(@Nullable K key, Iterable<? extends V> values) {
        return (Set) super.replaceValues(key, values);
    }

    public Map<K, Collection<V>> asMap() {
        return super.asMap();
    }

    public boolean put(K key, V value) {
        return super.put(key, value);
    }

    public boolean equals(@Nullable Object object) {
        return super.equals(object);
    }
}
