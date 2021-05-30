package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableSet;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import javax.annotation.Nullable;

@GwtCompatible(emulated = true, serializable = true)
public abstract class ImmutableMap<K, V> implements Map<K, V>, Serializable {
    private transient ImmutableSet<Map.Entry<K, V>> entrySet;
    private transient ImmutableSet<K> keySet;
    private transient ImmutableCollection<V> values;

    public abstract boolean containsValue(@Nullable Object obj);

    /* access modifiers changed from: package-private */
    public abstract ImmutableSet<Map.Entry<K, V>> createEntrySet();

    public abstract V get(@Nullable Object obj);

    /* access modifiers changed from: package-private */
    public abstract boolean isPartialView();

    public static <K, V> ImmutableMap<K, V> of() {
        return EmptyImmutableMap.INSTANCE;
    }

    public static <K, V> ImmutableMap<K, V> of(K k1, V v1) {
        return new SingletonImmutableMap(Preconditions.checkNotNull(k1), Preconditions.checkNotNull(v1));
    }

    public static <K, V> ImmutableMap<K, V> of(K k1, V v1, K k2, V v2) {
        return new RegularImmutableMap(entryOf(k1, v1), entryOf(k2, v2));
    }

    public static <K, V> ImmutableMap<K, V> of(K k1, V v1, K k2, V v2, K k3, V v3) {
        return new RegularImmutableMap(entryOf(k1, v1), entryOf(k2, v2), entryOf(k3, v3));
    }

    public static <K, V> ImmutableMap<K, V> of(K k1, V v1, K k2, V v2, K k3, V v3, K k4, V v4) {
        return new RegularImmutableMap(entryOf(k1, v1), entryOf(k2, v2), entryOf(k3, v3), entryOf(k4, v4));
    }

    public static <K, V> ImmutableMap<K, V> of(K k1, V v1, K k2, V v2, K k3, V v3, K k4, V v4, K k5, V v5) {
        return new RegularImmutableMap(entryOf(k1, v1), entryOf(k2, v2), entryOf(k3, v3), entryOf(k4, v4), entryOf(k5, v5));
    }

    public static <K, V> Builder<K, V> builder() {
        return new Builder<>();
    }

    static <K, V> Map.Entry<K, V> entryOf(K key, V value) {
        return Maps.immutableEntry(Preconditions.checkNotNull(key, "null key"), Preconditions.checkNotNull(value, "null value"));
    }

    public static class Builder<K, V> {
        final ArrayList<Map.Entry<K, V>> entries = Lists.newArrayList();

        public Builder<K, V> put(K key, V value) {
            this.entries.add(ImmutableMap.entryOf(key, value));
            return this;
        }

        public Builder<K, V> put(Map.Entry<? extends K, ? extends V> entry) {
            K key = entry.getKey();
            V value = entry.getValue();
            if (entry instanceof ImmutableEntry) {
                Preconditions.checkNotNull(key);
                Preconditions.checkNotNull(value);
                this.entries.add(entry);
            } else {
                this.entries.add(ImmutableMap.entryOf(key, value));
            }
            return this;
        }

        public Builder<K, V> putAll(Map<? extends K, ? extends V> map) {
            this.entries.ensureCapacity(this.entries.size() + map.size());
            for (Map.Entry<? extends K, ? extends V> entry : map.entrySet()) {
                put(entry.getKey(), entry.getValue());
            }
            return this;
        }

        public ImmutableMap<K, V> build() {
            return fromEntryList(this.entries);
        }

        private static <K, V> ImmutableMap<K, V> fromEntryList(List<Map.Entry<K, V>> entries2) {
            switch (entries2.size()) {
                case 0:
                    return ImmutableMap.of();
                case 1:
                    return new SingletonImmutableMap((Map.Entry) Iterables.getOnlyElement(entries2));
                default:
                    return new RegularImmutableMap((Map.Entry[]) entries2.toArray(new Map.Entry[entries2.size()]));
            }
        }
    }

    public static <K, V> ImmutableMap<K, V> copyOf(Map<? extends K, ? extends V> map) {
        if ((map instanceof ImmutableMap) && !(map instanceof ImmutableSortedMap)) {
            ImmutableMap<K, V> kvMap = (ImmutableMap) map;
            if (!kvMap.isPartialView()) {
                return kvMap;
            }
        }
        Map.Entry<K, V>[] entries = (Map.Entry[]) map.entrySet().toArray(new Map.Entry[0]);
        switch (entries.length) {
            case 0:
                return of();
            case 1:
                return new SingletonImmutableMap(entryOf(entries[0].getKey(), entries[0].getValue()));
        }
        for (int i = 0; i < entries.length; i++) {
            entries[i] = entryOf(entries[i].getKey(), entries[i].getValue());
        }
        return new RegularImmutableMap(entries);
    }

    ImmutableMap() {
    }

    public final V put(K k, V v) {
        throw new UnsupportedOperationException();
    }

    public final V remove(Object o) {
        throw new UnsupportedOperationException();
    }

    public final void putAll(Map<? extends K, ? extends V> map) {
        throw new UnsupportedOperationException();
    }

    public final void clear() {
        throw new UnsupportedOperationException();
    }

    public boolean isEmpty() {
        return size() == 0;
    }

    public boolean containsKey(@Nullable Object key) {
        return get(key) != null;
    }

    public ImmutableSet<Map.Entry<K, V>> entrySet() {
        ImmutableSet<Map.Entry<K, V>> result = this.entrySet;
        if (result != null) {
            return result;
        }
        ImmutableSet<Map.Entry<K, V>> createEntrySet = createEntrySet();
        this.entrySet = createEntrySet;
        return createEntrySet;
    }

    abstract class EntrySet extends ImmutableSet<Map.Entry<K, V>> {
        EntrySet() {
        }

        public int size() {
            return ImmutableMap.this.size();
        }

        public boolean contains(@Nullable Object object) {
            if (!(object instanceof Map.Entry)) {
                return false;
            }
            Map.Entry<?, ?> entry = (Map.Entry) object;
            V value = ImmutableMap.this.get(entry.getKey());
            if (value == null || !value.equals(entry.getValue())) {
                return false;
            }
            return true;
        }

        /* access modifiers changed from: package-private */
        public boolean isPartialView() {
            return ImmutableMap.this.isPartialView();
        }

        /* access modifiers changed from: package-private */
        public Object writeReplace() {
            return new EntrySetSerializedForm(ImmutableMap.this);
        }
    }

    private static class EntrySetSerializedForm<K, V> implements Serializable {
        private static final long serialVersionUID = 0;
        final ImmutableMap<K, V> map;

        EntrySetSerializedForm(ImmutableMap<K, V> map2) {
            this.map = map2;
        }

        /* access modifiers changed from: package-private */
        public Object readResolve() {
            return this.map.entrySet();
        }
    }

    public ImmutableSet<K> keySet() {
        ImmutableSet<K> result = this.keySet;
        if (result != null) {
            return result;
        }
        ImmutableSet<K> createKeySet = createKeySet();
        this.keySet = createKeySet;
        return createKeySet;
    }

    /* access modifiers changed from: package-private */
    public ImmutableSet<K> createKeySet() {
        return new KeySet();
    }

    class KeySet extends ImmutableSet.TransformedImmutableSet<Map.Entry<K, V>, K> {
        KeySet() {
            super(ImmutableMap.this.entrySet());
        }

        KeySet(int hashCode) {
            super(ImmutableMap.this.entrySet(), hashCode);
        }

        /* access modifiers changed from: package-private */
        public K transform(Map.Entry<K, V> entry) {
            return entry.getKey();
        }

        public boolean contains(@Nullable Object object) {
            return ImmutableMap.this.containsKey(object);
        }

        /* access modifiers changed from: package-private */
        public boolean isPartialView() {
            return true;
        }

        /* access modifiers changed from: package-private */
        public ImmutableList<K> createAsList() {
            return new TransformedImmutableList<Map.Entry<K, V>, K>(ImmutableMap.this.entrySet().asList()) {
                /* access modifiers changed from: package-private */
                public K transform(Map.Entry<K, V> entry) {
                    return entry.getKey();
                }
            };
        }

        /* access modifiers changed from: package-private */
        public Object writeReplace() {
            return new KeySetSerializedForm(ImmutableMap.this);
        }
    }

    private static class KeySetSerializedForm<K> implements Serializable {
        private static final long serialVersionUID = 0;
        final ImmutableMap<K, ?> map;

        KeySetSerializedForm(ImmutableMap<K, ?> map2) {
            this.map = map2;
        }

        /* access modifiers changed from: package-private */
        public Object readResolve() {
            return this.map.keySet();
        }
    }

    public ImmutableCollection<V> values() {
        ImmutableCollection<V> result = this.values;
        if (result != null) {
            return result;
        }
        ImmutableCollection<V> createValues = createValues();
        this.values = createValues;
        return createValues;
    }

    /* access modifiers changed from: package-private */
    public ImmutableCollection<V> createValues() {
        return new Values();
    }

    class Values extends ImmutableCollection<V> {
        Values() {
        }

        public int size() {
            return ImmutableMap.this.size();
        }

        public UnmodifiableIterator<V> iterator() {
            return Maps.valueIterator(ImmutableMap.this.entrySet().iterator());
        }

        public boolean contains(Object object) {
            return ImmutableMap.this.containsValue(object);
        }

        /* access modifiers changed from: package-private */
        public boolean isPartialView() {
            return true;
        }

        /* access modifiers changed from: package-private */
        public ImmutableList<V> createAsList() {
            return new TransformedImmutableList<Map.Entry<K, V>, V>(ImmutableMap.this.entrySet().asList()) {
                /* access modifiers changed from: package-private */
                public V transform(Map.Entry<K, V> entry) {
                    return entry.getValue();
                }
            };
        }

        /* access modifiers changed from: package-private */
        public Object writeReplace() {
            return new ValuesSerializedForm(ImmutableMap.this);
        }
    }

    private static class ValuesSerializedForm<V> implements Serializable {
        private static final long serialVersionUID = 0;
        final ImmutableMap<?, V> map;

        ValuesSerializedForm(ImmutableMap<?, V> map2) {
            this.map = map2;
        }

        /* access modifiers changed from: package-private */
        public Object readResolve() {
            return this.map.values();
        }
    }

    public boolean equals(@Nullable Object object) {
        if (object == this) {
            return true;
        }
        if (object instanceof Map) {
            return entrySet().equals(((Map) object).entrySet());
        }
        return false;
    }

    public int hashCode() {
        return entrySet().hashCode();
    }

    public String toString() {
        return Maps.toStringImpl(this);
    }

    static class SerializedForm implements Serializable {
        private static final long serialVersionUID = 0;
        private final Object[] keys;
        private final Object[] values;

        SerializedForm(ImmutableMap<?, ?> map) {
            this.keys = new Object[map.size()];
            this.values = new Object[map.size()];
            int i = 0;
            Iterator i$ = map.entrySet().iterator();
            while (i$.hasNext()) {
                Map.Entry<?, ?> entry = i$.next();
                this.keys[i] = entry.getKey();
                this.values[i] = entry.getValue();
                i++;
            }
        }

        /* access modifiers changed from: package-private */
        public Object readResolve() {
            return createMap(new Builder<>());
        }

        /* access modifiers changed from: package-private */
        public Object createMap(Builder<Object, Object> builder) {
            for (int i = 0; i < this.keys.length; i++) {
                builder.put(this.keys[i], this.values[i]);
            }
            return builder.build();
        }
    }

    /* access modifiers changed from: package-private */
    public Object writeReplace() {
        return new SerializedForm(this);
    }
}
