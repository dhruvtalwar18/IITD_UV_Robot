package com.google.common.collect;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import com.google.common.annotations.GwtIncompatible;
import com.google.common.base.Function;
import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableMultiset;
import com.google.common.collect.Serialization;
import java.io.Serializable;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import javax.annotation.Nullable;

@GwtCompatible(emulated = true)
public abstract class ImmutableMultimap<K, V> implements Multimap<K, V>, Serializable {
    private static final long serialVersionUID = 0;
    private transient ImmutableCollection<Map.Entry<K, V>> entries;
    private transient ImmutableMultiset<K> keys;
    final transient ImmutableMap<K, ? extends ImmutableCollection<V>> map;
    final transient int size;
    private transient ImmutableCollection<V> values;

    public abstract ImmutableCollection<V> get(K k);

    @Beta
    public abstract ImmutableMultimap<V, K> inverse();

    public static <K, V> ImmutableMultimap<K, V> of() {
        return ImmutableListMultimap.of();
    }

    public static <K, V> ImmutableMultimap<K, V> of(K k1, V v1) {
        return ImmutableListMultimap.of(k1, v1);
    }

    public static <K, V> ImmutableMultimap<K, V> of(K k1, V v1, K k2, V v2) {
        return ImmutableListMultimap.of(k1, v1, k2, v2);
    }

    public static <K, V> ImmutableMultimap<K, V> of(K k1, V v1, K k2, V v2, K k3, V v3) {
        return ImmutableListMultimap.of(k1, v1, k2, v2, k3, v3);
    }

    public static <K, V> ImmutableMultimap<K, V> of(K k1, V v1, K k2, V v2, K k3, V v3, K k4, V v4) {
        return ImmutableListMultimap.of(k1, v1, k2, v2, k3, v3, k4, v4);
    }

    public static <K, V> ImmutableMultimap<K, V> of(K k1, V v1, K k2, V v2, K k3, V v3, K k4, V v4, K k5, V v5) {
        return ImmutableListMultimap.of(k1, v1, k2, v2, k3, v3, k4, v4, k5, v5);
    }

    public static <K, V> Builder<K, V> builder() {
        return new Builder<>();
    }

    private static class BuilderMultimap<K, V> extends AbstractMultimap<K, V> {
        private static final long serialVersionUID = 0;

        BuilderMultimap() {
            super(new LinkedHashMap());
        }

        /* access modifiers changed from: package-private */
        public Collection<V> createCollection() {
            return Lists.newArrayList();
        }
    }

    public static class Builder<K, V> {
        Multimap<K, V> builderMultimap = new BuilderMultimap();
        Comparator<? super K> keyComparator;
        Comparator<? super V> valueComparator;

        public Builder<K, V> put(K key, V value) {
            this.builderMultimap.put(Preconditions.checkNotNull(key), Preconditions.checkNotNull(value));
            return this;
        }

        public Builder<K, V> put(Map.Entry<? extends K, ? extends V> entry) {
            this.builderMultimap.put(Preconditions.checkNotNull(entry.getKey()), Preconditions.checkNotNull(entry.getValue()));
            return this;
        }

        public Builder<K, V> putAll(K key, Iterable<? extends V> values) {
            Collection<V> valueList = this.builderMultimap.get(Preconditions.checkNotNull(key));
            for (V value : values) {
                valueList.add(Preconditions.checkNotNull(value));
            }
            return this;
        }

        public Builder<K, V> putAll(K key, V... values) {
            return putAll(key, Arrays.asList(values));
        }

        public Builder<K, V> putAll(Multimap<? extends K, ? extends V> multimap) {
            for (Map.Entry<? extends K, ? extends Collection<? extends V>> entry : multimap.asMap().entrySet()) {
                putAll(entry.getKey(), (Iterable) entry.getValue());
            }
            return this;
        }

        @Beta
        public Builder<K, V> orderKeysBy(Comparator<? super K> keyComparator2) {
            this.keyComparator = (Comparator) Preconditions.checkNotNull(keyComparator2);
            return this;
        }

        @Beta
        public Builder<K, V> orderValuesBy(Comparator<? super V> valueComparator2) {
            this.valueComparator = (Comparator) Preconditions.checkNotNull(valueComparator2);
            return this;
        }

        public ImmutableMultimap<K, V> build() {
            if (this.valueComparator != null) {
                Iterator i$ = this.builderMultimap.asMap().values().iterator();
                while (i$.hasNext()) {
                    Collections.sort((List) i$.next(), this.valueComparator);
                }
            }
            if (this.keyComparator != null) {
                Multimap<K, V> sortedCopy = new BuilderMultimap<>();
                List<Map.Entry<K, Collection<V>>> entries = Lists.newArrayList(this.builderMultimap.asMap().entrySet());
                Collections.sort(entries, Ordering.from(this.keyComparator).onResultOf(new Function<Map.Entry<K, Collection<V>>, K>() {
                    public K apply(Map.Entry<K, Collection<V>> entry) {
                        return entry.getKey();
                    }
                }));
                for (Map.Entry<K, Collection<V>> entry : entries) {
                    sortedCopy.putAll(entry.getKey(), entry.getValue());
                }
                this.builderMultimap = sortedCopy;
            }
            return ImmutableMultimap.copyOf(this.builderMultimap);
        }
    }

    public static <K, V> ImmutableMultimap<K, V> copyOf(Multimap<? extends K, ? extends V> multimap) {
        if (multimap instanceof ImmutableMultimap) {
            ImmutableMultimap<K, V> kvMultimap = (ImmutableMultimap) multimap;
            if (!kvMultimap.isPartialView()) {
                return kvMultimap;
            }
        }
        return ImmutableListMultimap.copyOf(multimap);
    }

    @GwtIncompatible("java serialization is not supported")
    static class FieldSettersHolder {
        static final Serialization.FieldSetter<ImmutableMultimap> MAP_FIELD_SETTER = Serialization.getFieldSetter(ImmutableMultimap.class, "map");
        static final Serialization.FieldSetter<ImmutableMultimap> SIZE_FIELD_SETTER = Serialization.getFieldSetter(ImmutableMultimap.class, "size");

        FieldSettersHolder() {
        }
    }

    ImmutableMultimap(ImmutableMap<K, ? extends ImmutableCollection<V>> map2, int size2) {
        this.map = map2;
        this.size = size2;
    }

    public ImmutableCollection<V> removeAll(Object key) {
        throw new UnsupportedOperationException();
    }

    public ImmutableCollection<V> replaceValues(K k, Iterable<? extends V> iterable) {
        throw new UnsupportedOperationException();
    }

    public void clear() {
        throw new UnsupportedOperationException();
    }

    public boolean put(K k, V v) {
        throw new UnsupportedOperationException();
    }

    public boolean putAll(K k, Iterable<? extends V> iterable) {
        throw new UnsupportedOperationException();
    }

    public boolean putAll(Multimap<? extends K, ? extends V> multimap) {
        throw new UnsupportedOperationException();
    }

    public boolean remove(Object key, Object value) {
        throw new UnsupportedOperationException();
    }

    /* access modifiers changed from: package-private */
    public boolean isPartialView() {
        return this.map.isPartialView();
    }

    public boolean containsEntry(@Nullable Object key, @Nullable Object value) {
        Collection<V> values2 = (Collection) this.map.get(key);
        return values2 != null && values2.contains(value);
    }

    public boolean containsKey(@Nullable Object key) {
        return this.map.containsKey(key);
    }

    public boolean containsValue(@Nullable Object value) {
        Iterator i$ = this.map.values().iterator();
        while (i$.hasNext()) {
            if (((ImmutableCollection) i$.next()).contains(value)) {
                return true;
            }
        }
        return false;
    }

    public boolean isEmpty() {
        return this.size == 0;
    }

    public int size() {
        return this.size;
    }

    public boolean equals(@Nullable Object object) {
        if (object instanceof Multimap) {
            return this.map.equals(((Multimap) object).asMap());
        }
        return false;
    }

    public int hashCode() {
        return this.map.hashCode();
    }

    public String toString() {
        return this.map.toString();
    }

    public ImmutableSet<K> keySet() {
        return this.map.keySet();
    }

    public ImmutableMap<K, Collection<V>> asMap() {
        return this.map;
    }

    public ImmutableCollection<Map.Entry<K, V>> entries() {
        ImmutableCollection<Map.Entry<K, V>> result = this.entries;
        if (result != null) {
            return result;
        }
        EntryCollection entryCollection = new EntryCollection(this);
        this.entries = entryCollection;
        return entryCollection;
    }

    private static class EntryCollection<K, V> extends ImmutableCollection<Map.Entry<K, V>> {
        private static final long serialVersionUID = 0;
        final ImmutableMultimap<K, V> multimap;

        EntryCollection(ImmutableMultimap<K, V> multimap2) {
            this.multimap = multimap2;
        }

        public UnmodifiableIterator<Map.Entry<K, V>> iterator() {
            final Iterator<? extends Map.Entry<K, ? extends ImmutableCollection<V>>> mapIterator = this.multimap.map.entrySet().iterator();
            return new UnmodifiableIterator<Map.Entry<K, V>>() {
                K key;
                Iterator<V> valueIterator;

                public boolean hasNext() {
                    return (this.key != null && this.valueIterator.hasNext()) || mapIterator.hasNext();
                }

                public Map.Entry<K, V> next() {
                    if (this.key == null || !this.valueIterator.hasNext()) {
                        Map.Entry<K, ? extends ImmutableCollection<V>> entry = (Map.Entry) mapIterator.next();
                        this.key = entry.getKey();
                        this.valueIterator = ((ImmutableCollection) entry.getValue()).iterator();
                    }
                    return Maps.immutableEntry(this.key, this.valueIterator.next());
                }
            };
        }

        /* access modifiers changed from: package-private */
        public boolean isPartialView() {
            return this.multimap.isPartialView();
        }

        public int size() {
            return this.multimap.size();
        }

        public boolean contains(Object object) {
            if (!(object instanceof Map.Entry)) {
                return false;
            }
            Map.Entry<?, ?> entry = (Map.Entry) object;
            return this.multimap.containsEntry(entry.getKey(), entry.getValue());
        }
    }

    public ImmutableMultiset<K> keys() {
        ImmutableMultiset<K> result = this.keys;
        if (result != null) {
            return result;
        }
        ImmutableMultiset<K> createKeys = createKeys();
        this.keys = createKeys;
        return createKeys;
    }

    private ImmutableMultiset<K> createKeys() {
        ImmutableMultiset.Builder<K> builder = ImmutableMultiset.builder();
        Iterator i$ = this.map.entrySet().iterator();
        while (i$.hasNext()) {
            Map.Entry<K, ? extends ImmutableCollection<V>> entry = i$.next();
            builder.addCopies(entry.getKey(), ((ImmutableCollection) entry.getValue()).size());
        }
        return builder.build();
    }

    public ImmutableCollection<V> values() {
        ImmutableCollection<V> result = this.values;
        if (result != null) {
            return result;
        }
        Values values2 = new Values(this);
        this.values = values2;
        return values2;
    }

    private static class Values<V> extends ImmutableCollection<V> {
        private static final long serialVersionUID = 0;
        final ImmutableMultimap<?, V> multimap;

        Values(ImmutableMultimap<?, V> multimap2) {
            this.multimap = multimap2;
        }

        public UnmodifiableIterator<V> iterator() {
            return Maps.valueIterator(this.multimap.entries().iterator());
        }

        public int size() {
            return this.multimap.size();
        }

        /* access modifiers changed from: package-private */
        public boolean isPartialView() {
            return true;
        }
    }
}
