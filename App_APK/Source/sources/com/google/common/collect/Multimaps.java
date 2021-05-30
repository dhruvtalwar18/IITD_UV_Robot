package com.google.common.collect;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import com.google.common.annotations.GwtIncompatible;
import com.google.common.base.Function;
import com.google.common.base.Joiner;
import com.google.common.base.Objects;
import com.google.common.base.Preconditions;
import com.google.common.base.Predicate;
import com.google.common.base.Predicates;
import com.google.common.base.Supplier;
import com.google.common.collect.Collections2;
import com.google.common.collect.ImmutableListMultimap;
import com.google.common.collect.Maps;
import com.google.common.collect.Multiset;
import com.google.common.collect.Multisets;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.AbstractCollection;
import java.util.AbstractSet;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Set;
import java.util.SortedSet;
import javax.annotation.Nullable;

@GwtCompatible(emulated = true)
public final class Multimaps {
    private Multimaps() {
    }

    public static <K, V> Multimap<K, V> newMultimap(Map<K, Collection<V>> map, Supplier<? extends Collection<V>> factory) {
        return new CustomMultimap(map, factory);
    }

    private static class CustomMultimap<K, V> extends AbstractMultimap<K, V> {
        @GwtIncompatible("java serialization not supported")
        private static final long serialVersionUID = 0;
        transient Supplier<? extends Collection<V>> factory;

        CustomMultimap(Map<K, Collection<V>> map, Supplier<? extends Collection<V>> factory2) {
            super(map);
            this.factory = (Supplier) Preconditions.checkNotNull(factory2);
        }

        /* access modifiers changed from: protected */
        public Collection<V> createCollection() {
            return (Collection) this.factory.get();
        }

        @GwtIncompatible("java.io.ObjectOutputStream")
        private void writeObject(ObjectOutputStream stream) throws IOException {
            stream.defaultWriteObject();
            stream.writeObject(this.factory);
            stream.writeObject(backingMap());
        }

        @GwtIncompatible("java.io.ObjectInputStream")
        private void readObject(ObjectInputStream stream) throws IOException, ClassNotFoundException {
            stream.defaultReadObject();
            this.factory = (Supplier) stream.readObject();
            setMap((Map) stream.readObject());
        }
    }

    public static <K, V> ListMultimap<K, V> newListMultimap(Map<K, Collection<V>> map, Supplier<? extends List<V>> factory) {
        return new CustomListMultimap(map, factory);
    }

    private static class CustomListMultimap<K, V> extends AbstractListMultimap<K, V> {
        @GwtIncompatible("java serialization not supported")
        private static final long serialVersionUID = 0;
        transient Supplier<? extends List<V>> factory;

        CustomListMultimap(Map<K, Collection<V>> map, Supplier<? extends List<V>> factory2) {
            super(map);
            this.factory = (Supplier) Preconditions.checkNotNull(factory2);
        }

        /* access modifiers changed from: protected */
        public List<V> createCollection() {
            return (List) this.factory.get();
        }

        @GwtIncompatible("java.io.ObjectOutputStream")
        private void writeObject(ObjectOutputStream stream) throws IOException {
            stream.defaultWriteObject();
            stream.writeObject(this.factory);
            stream.writeObject(backingMap());
        }

        @GwtIncompatible("java.io.ObjectInputStream")
        private void readObject(ObjectInputStream stream) throws IOException, ClassNotFoundException {
            stream.defaultReadObject();
            this.factory = (Supplier) stream.readObject();
            setMap((Map) stream.readObject());
        }
    }

    public static <K, V> SetMultimap<K, V> newSetMultimap(Map<K, Collection<V>> map, Supplier<? extends Set<V>> factory) {
        return new CustomSetMultimap(map, factory);
    }

    private static class CustomSetMultimap<K, V> extends AbstractSetMultimap<K, V> {
        @GwtIncompatible("not needed in emulated source")
        private static final long serialVersionUID = 0;
        transient Supplier<? extends Set<V>> factory;

        CustomSetMultimap(Map<K, Collection<V>> map, Supplier<? extends Set<V>> factory2) {
            super(map);
            this.factory = (Supplier) Preconditions.checkNotNull(factory2);
        }

        /* access modifiers changed from: protected */
        public Set<V> createCollection() {
            return (Set) this.factory.get();
        }

        @GwtIncompatible("java.io.ObjectOutputStream")
        private void writeObject(ObjectOutputStream stream) throws IOException {
            stream.defaultWriteObject();
            stream.writeObject(this.factory);
            stream.writeObject(backingMap());
        }

        @GwtIncompatible("java.io.ObjectInputStream")
        private void readObject(ObjectInputStream stream) throws IOException, ClassNotFoundException {
            stream.defaultReadObject();
            this.factory = (Supplier) stream.readObject();
            setMap((Map) stream.readObject());
        }
    }

    public static <K, V> SortedSetMultimap<K, V> newSortedSetMultimap(Map<K, Collection<V>> map, Supplier<? extends SortedSet<V>> factory) {
        return new CustomSortedSetMultimap(map, factory);
    }

    private static class CustomSortedSetMultimap<K, V> extends AbstractSortedSetMultimap<K, V> {
        @GwtIncompatible("not needed in emulated source")
        private static final long serialVersionUID = 0;
        transient Supplier<? extends SortedSet<V>> factory;
        transient Comparator<? super V> valueComparator;

        CustomSortedSetMultimap(Map<K, Collection<V>> map, Supplier<? extends SortedSet<V>> factory2) {
            super(map);
            this.factory = (Supplier) Preconditions.checkNotNull(factory2);
            this.valueComparator = ((SortedSet) factory2.get()).comparator();
        }

        /* access modifiers changed from: protected */
        public SortedSet<V> createCollection() {
            return (SortedSet) this.factory.get();
        }

        public Comparator<? super V> valueComparator() {
            return this.valueComparator;
        }

        @GwtIncompatible("java.io.ObjectOutputStream")
        private void writeObject(ObjectOutputStream stream) throws IOException {
            stream.defaultWriteObject();
            stream.writeObject(this.factory);
            stream.writeObject(backingMap());
        }

        @GwtIncompatible("java.io.ObjectInputStream")
        private void readObject(ObjectInputStream stream) throws IOException, ClassNotFoundException {
            stream.defaultReadObject();
            this.factory = (Supplier) stream.readObject();
            this.valueComparator = ((SortedSet) this.factory.get()).comparator();
            setMap((Map) stream.readObject());
        }
    }

    public static <K, V, M extends Multimap<K, V>> M invertFrom(Multimap<? extends V, ? extends K> source, M dest) {
        Preconditions.checkNotNull(dest);
        for (Map.Entry<? extends V, ? extends K> entry : source.entries()) {
            dest.put(entry.getValue(), entry.getKey());
        }
        return dest;
    }

    public static <K, V> Multimap<K, V> synchronizedMultimap(Multimap<K, V> multimap) {
        return Synchronized.multimap(multimap, (Object) null);
    }

    public static <K, V> Multimap<K, V> unmodifiableMultimap(Multimap<K, V> delegate) {
        if ((delegate instanceof UnmodifiableMultimap) || (delegate instanceof ImmutableMultimap)) {
            return delegate;
        }
        return new UnmodifiableMultimap(delegate);
    }

    @Deprecated
    public static <K, V> Multimap<K, V> unmodifiableMultimap(ImmutableMultimap<K, V> delegate) {
        return (Multimap) Preconditions.checkNotNull(delegate);
    }

    private static class UnmodifiableMultimap<K, V> extends ForwardingMultimap<K, V> implements Serializable {
        private static final long serialVersionUID = 0;
        final Multimap<K, V> delegate;
        transient Collection<Map.Entry<K, V>> entries;
        transient Set<K> keySet;
        transient Multiset<K> keys;
        transient Map<K, Collection<V>> map;
        transient Collection<V> values;

        UnmodifiableMultimap(Multimap<K, V> delegate2) {
            this.delegate = (Multimap) Preconditions.checkNotNull(delegate2);
        }

        /* access modifiers changed from: protected */
        public Multimap<K, V> delegate() {
            return this.delegate;
        }

        public void clear() {
            throw new UnsupportedOperationException();
        }

        public Map<K, Collection<V>> asMap() {
            Map<K, Collection<V>> result = this.map;
            if (result != null) {
                return result;
            }
            final Map<K, Collection<V>> unmodifiableMap = Collections.unmodifiableMap(this.delegate.asMap());
            Map<K, Collection<V>> r2 = new ForwardingMap<K, Collection<V>>() {
                Collection<Collection<V>> asMapValues;
                Set<Map.Entry<K, Collection<V>>> entrySet;

                /* access modifiers changed from: protected */
                public Map<K, Collection<V>> delegate() {
                    return unmodifiableMap;
                }

                public Set<Map.Entry<K, Collection<V>>> entrySet() {
                    Set<Map.Entry<K, Collection<V>>> result = this.entrySet;
                    if (result != null) {
                        return result;
                    }
                    Set<Map.Entry<K, Collection<V>>> access$000 = Multimaps.unmodifiableAsMapEntries(unmodifiableMap.entrySet());
                    this.entrySet = access$000;
                    return access$000;
                }

                public Collection<V> get(Object key) {
                    Collection<V> collection = (Collection) unmodifiableMap.get(key);
                    if (collection == null) {
                        return null;
                    }
                    return Multimaps.unmodifiableValueCollection(collection);
                }

                public Collection<Collection<V>> values() {
                    Collection<Collection<V>> result = this.asMapValues;
                    if (result != null) {
                        return result;
                    }
                    UnmodifiableAsMapValues unmodifiableAsMapValues = new UnmodifiableAsMapValues(unmodifiableMap.values());
                    this.asMapValues = unmodifiableAsMapValues;
                    return unmodifiableAsMapValues;
                }

                public boolean containsValue(Object o) {
                    return values().contains(o);
                }
            };
            Map<K, Collection<V>> result2 = r2;
            this.map = r2;
            return result2;
        }

        public Collection<Map.Entry<K, V>> entries() {
            Collection<Map.Entry<K, V>> result = this.entries;
            if (result != null) {
                return result;
            }
            Collection<Map.Entry<K, V>> access$200 = Multimaps.unmodifiableEntries(this.delegate.entries());
            Collection<Map.Entry<K, V>> result2 = access$200;
            this.entries = access$200;
            return result2;
        }

        public Collection<V> get(K key) {
            return Multimaps.unmodifiableValueCollection(this.delegate.get(key));
        }

        public Multiset<K> keys() {
            Multiset<K> result = this.keys;
            if (result != null) {
                return result;
            }
            Multiset<K> unmodifiableMultiset = Multisets.unmodifiableMultiset(this.delegate.keys());
            Multiset<K> result2 = unmodifiableMultiset;
            this.keys = unmodifiableMultiset;
            return result2;
        }

        public Set<K> keySet() {
            Set<K> result = this.keySet;
            if (result != null) {
                return result;
            }
            Set<K> unmodifiableSet = Collections.unmodifiableSet(this.delegate.keySet());
            Set<K> result2 = unmodifiableSet;
            this.keySet = unmodifiableSet;
            return result2;
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

        public Collection<V> removeAll(Object key) {
            throw new UnsupportedOperationException();
        }

        public Collection<V> replaceValues(K k, Iterable<? extends V> iterable) {
            throw new UnsupportedOperationException();
        }

        public Collection<V> values() {
            Collection<V> result = this.values;
            if (result != null) {
                return result;
            }
            Collection<V> unmodifiableCollection = Collections.unmodifiableCollection(this.delegate.values());
            Collection<V> result2 = unmodifiableCollection;
            this.values = unmodifiableCollection;
            return result2;
        }
    }

    private static class UnmodifiableAsMapValues<V> extends ForwardingCollection<Collection<V>> {
        final Collection<Collection<V>> delegate;

        UnmodifiableAsMapValues(Collection<Collection<V>> delegate2) {
            this.delegate = Collections.unmodifiableCollection(delegate2);
        }

        /* access modifiers changed from: protected */
        public Collection<Collection<V>> delegate() {
            return this.delegate;
        }

        public Iterator<Collection<V>> iterator() {
            final Iterator<Collection<V>> iterator = this.delegate.iterator();
            return new UnmodifiableIterator<Collection<V>>() {
                public boolean hasNext() {
                    return iterator.hasNext();
                }

                public Collection<V> next() {
                    return Multimaps.unmodifiableValueCollection((Collection) iterator.next());
                }
            };
        }

        public Object[] toArray() {
            return standardToArray();
        }

        public <T> T[] toArray(T[] array) {
            return standardToArray(array);
        }

        public boolean contains(Object o) {
            return standardContains(o);
        }

        public boolean containsAll(Collection<?> c) {
            return standardContainsAll(c);
        }
    }

    private static class UnmodifiableListMultimap<K, V> extends UnmodifiableMultimap<K, V> implements ListMultimap<K, V> {
        private static final long serialVersionUID = 0;

        UnmodifiableListMultimap(ListMultimap<K, V> delegate) {
            super(delegate);
        }

        public ListMultimap<K, V> delegate() {
            return (ListMultimap) super.delegate();
        }

        public List<V> get(K key) {
            return Collections.unmodifiableList(delegate().get(key));
        }

        public List<V> removeAll(Object key) {
            throw new UnsupportedOperationException();
        }

        public List<V> replaceValues(K k, Iterable<? extends V> iterable) {
            throw new UnsupportedOperationException();
        }
    }

    private static class UnmodifiableSetMultimap<K, V> extends UnmodifiableMultimap<K, V> implements SetMultimap<K, V> {
        private static final long serialVersionUID = 0;

        UnmodifiableSetMultimap(SetMultimap<K, V> delegate) {
            super(delegate);
        }

        public SetMultimap<K, V> delegate() {
            return (SetMultimap) super.delegate();
        }

        public Set<V> get(K key) {
            return Collections.unmodifiableSet(delegate().get(key));
        }

        public Set<Map.Entry<K, V>> entries() {
            return Maps.unmodifiableEntrySet(delegate().entries());
        }

        public Set<V> removeAll(Object key) {
            throw new UnsupportedOperationException();
        }

        public Set<V> replaceValues(K k, Iterable<? extends V> iterable) {
            throw new UnsupportedOperationException();
        }
    }

    private static class UnmodifiableSortedSetMultimap<K, V> extends UnmodifiableSetMultimap<K, V> implements SortedSetMultimap<K, V> {
        private static final long serialVersionUID = 0;

        UnmodifiableSortedSetMultimap(SortedSetMultimap<K, V> delegate) {
            super(delegate);
        }

        public SortedSetMultimap<K, V> delegate() {
            return (SortedSetMultimap) super.delegate();
        }

        public SortedSet<V> get(K key) {
            return Collections.unmodifiableSortedSet(delegate().get(key));
        }

        public SortedSet<V> removeAll(Object key) {
            throw new UnsupportedOperationException();
        }

        public SortedSet<V> replaceValues(K k, Iterable<? extends V> iterable) {
            throw new UnsupportedOperationException();
        }

        public Comparator<? super V> valueComparator() {
            return delegate().valueComparator();
        }
    }

    public static <K, V> SetMultimap<K, V> synchronizedSetMultimap(SetMultimap<K, V> multimap) {
        return Synchronized.setMultimap(multimap, (Object) null);
    }

    public static <K, V> SetMultimap<K, V> unmodifiableSetMultimap(SetMultimap<K, V> delegate) {
        if ((delegate instanceof UnmodifiableSetMultimap) || (delegate instanceof ImmutableSetMultimap)) {
            return delegate;
        }
        return new UnmodifiableSetMultimap(delegate);
    }

    @Deprecated
    public static <K, V> SetMultimap<K, V> unmodifiableSetMultimap(ImmutableSetMultimap<K, V> delegate) {
        return (SetMultimap) Preconditions.checkNotNull(delegate);
    }

    public static <K, V> SortedSetMultimap<K, V> synchronizedSortedSetMultimap(SortedSetMultimap<K, V> multimap) {
        return Synchronized.sortedSetMultimap(multimap, (Object) null);
    }

    public static <K, V> SortedSetMultimap<K, V> unmodifiableSortedSetMultimap(SortedSetMultimap<K, V> delegate) {
        if (delegate instanceof UnmodifiableSortedSetMultimap) {
            return delegate;
        }
        return new UnmodifiableSortedSetMultimap(delegate);
    }

    public static <K, V> ListMultimap<K, V> synchronizedListMultimap(ListMultimap<K, V> multimap) {
        return Synchronized.listMultimap(multimap, (Object) null);
    }

    public static <K, V> ListMultimap<K, V> unmodifiableListMultimap(ListMultimap<K, V> delegate) {
        if ((delegate instanceof UnmodifiableListMultimap) || (delegate instanceof ImmutableListMultimap)) {
            return delegate;
        }
        return new UnmodifiableListMultimap(delegate);
    }

    @Deprecated
    public static <K, V> ListMultimap<K, V> unmodifiableListMultimap(ImmutableListMultimap<K, V> delegate) {
        return (ListMultimap) Preconditions.checkNotNull(delegate);
    }

    /* access modifiers changed from: private */
    public static <V> Collection<V> unmodifiableValueCollection(Collection<V> collection) {
        if (collection instanceof SortedSet) {
            return Collections.unmodifiableSortedSet((SortedSet) collection);
        }
        if (collection instanceof Set) {
            return Collections.unmodifiableSet((Set) collection);
        }
        if (collection instanceof List) {
            return Collections.unmodifiableList((List) collection);
        }
        return Collections.unmodifiableCollection(collection);
    }

    /* access modifiers changed from: private */
    public static <K, V> Map.Entry<K, Collection<V>> unmodifiableAsMapEntry(final Map.Entry<K, Collection<V>> entry) {
        Preconditions.checkNotNull(entry);
        return new AbstractMapEntry<K, Collection<V>>() {
            public K getKey() {
                return entry.getKey();
            }

            public Collection<V> getValue() {
                return Multimaps.unmodifiableValueCollection((Collection) entry.getValue());
            }
        };
    }

    /* access modifiers changed from: private */
    public static <K, V> Collection<Map.Entry<K, V>> unmodifiableEntries(Collection<Map.Entry<K, V>> entries) {
        if (entries instanceof Set) {
            return Maps.unmodifiableEntrySet((Set) entries);
        }
        return new Maps.UnmodifiableEntries(Collections.unmodifiableCollection(entries));
    }

    /* access modifiers changed from: private */
    public static <K, V> Set<Map.Entry<K, Collection<V>>> unmodifiableAsMapEntries(Set<Map.Entry<K, Collection<V>>> asMapEntries) {
        return new UnmodifiableAsMapEntries(Collections.unmodifiableSet(asMapEntries));
    }

    static class UnmodifiableAsMapEntries<K, V> extends ForwardingSet<Map.Entry<K, Collection<V>>> {
        private final Set<Map.Entry<K, Collection<V>>> delegate;

        UnmodifiableAsMapEntries(Set<Map.Entry<K, Collection<V>>> delegate2) {
            this.delegate = delegate2;
        }

        /* access modifiers changed from: protected */
        public Set<Map.Entry<K, Collection<V>>> delegate() {
            return this.delegate;
        }

        public Iterator<Map.Entry<K, Collection<V>>> iterator() {
            final Iterator<Map.Entry<K, Collection<V>>> iterator = this.delegate.iterator();
            return new ForwardingIterator<Map.Entry<K, Collection<V>>>() {
                /* access modifiers changed from: protected */
                public Iterator<Map.Entry<K, Collection<V>>> delegate() {
                    return iterator;
                }

                public Map.Entry<K, Collection<V>> next() {
                    return Multimaps.unmodifiableAsMapEntry((Map.Entry) iterator.next());
                }
            };
        }

        public Object[] toArray() {
            return standardToArray();
        }

        public <T> T[] toArray(T[] array) {
            return standardToArray(array);
        }

        public boolean contains(Object o) {
            return Maps.containsEntryImpl(delegate(), o);
        }

        public boolean containsAll(Collection<?> c) {
            return standardContainsAll(c);
        }

        public boolean equals(@Nullable Object object) {
            return standardEquals(object);
        }
    }

    public static <K, V> SetMultimap<K, V> forMap(Map<K, V> map) {
        return new MapMultimap(map);
    }

    private static class MapMultimap<K, V> implements SetMultimap<K, V>, Serializable {
        private static final Joiner.MapJoiner JOINER = Joiner.on("], ").withKeyValueSeparator("=[").useForNull("null");
        private static final long serialVersionUID = 7845222491160860175L;
        transient Map<K, Collection<V>> asMap;
        final Map<K, V> map;

        MapMultimap(Map<K, V> map2) {
            this.map = (Map) Preconditions.checkNotNull(map2);
        }

        public int size() {
            return this.map.size();
        }

        public boolean isEmpty() {
            return this.map.isEmpty();
        }

        public boolean containsKey(Object key) {
            return this.map.containsKey(key);
        }

        public boolean containsValue(Object value) {
            return this.map.containsValue(value);
        }

        public boolean containsEntry(Object key, Object value) {
            return this.map.entrySet().contains(Maps.immutableEntry(key, value));
        }

        public Set<V> get(final K key) {
            return new AbstractSet<V>() {
                public Iterator<V> iterator() {
                    return new Iterator<V>() {
                        int i;

                        public boolean hasNext() {
                            return this.i == 0 && MapMultimap.this.map.containsKey(key);
                        }

                        public V next() {
                            if (hasNext()) {
                                this.i++;
                                return MapMultimap.this.map.get(key);
                            }
                            throw new NoSuchElementException();
                        }

                        public void remove() {
                            boolean z = true;
                            if (this.i != 1) {
                                z = false;
                            }
                            Preconditions.checkState(z);
                            this.i = -1;
                            MapMultimap.this.map.remove(key);
                        }
                    };
                }

                public int size() {
                    return MapMultimap.this.map.containsKey(key) ? 1 : 0;
                }
            };
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

        public Set<V> replaceValues(K k, Iterable<? extends V> iterable) {
            throw new UnsupportedOperationException();
        }

        public boolean remove(Object key, Object value) {
            return this.map.entrySet().remove(Maps.immutableEntry(key, value));
        }

        public Set<V> removeAll(Object key) {
            Set<V> values = new HashSet<>(2);
            if (!this.map.containsKey(key)) {
                return values;
            }
            values.add(this.map.remove(key));
            return values;
        }

        public void clear() {
            this.map.clear();
        }

        public Set<K> keySet() {
            return this.map.keySet();
        }

        public Multiset<K> keys() {
            return Multisets.forSet(this.map.keySet());
        }

        public Collection<V> values() {
            return this.map.values();
        }

        public Set<Map.Entry<K, V>> entries() {
            return this.map.entrySet();
        }

        public Map<K, Collection<V>> asMap() {
            Map<K, Collection<V>> result = this.asMap;
            if (result != null) {
                return result;
            }
            Map<K, Collection<V>> asMap2 = new AsMap();
            Map<K, Collection<V>> result2 = asMap2;
            this.asMap = asMap2;
            return result2;
        }

        public boolean equals(@Nullable Object object) {
            if (object == this) {
                return true;
            }
            if (!(object instanceof Multimap)) {
                return false;
            }
            Multimap<?, ?> that = (Multimap) object;
            if (size() != that.size() || !asMap().equals(that.asMap())) {
                return false;
            }
            return true;
        }

        public int hashCode() {
            return this.map.hashCode();
        }

        public String toString() {
            if (this.map.isEmpty()) {
                return "{}";
            }
            StringBuilder builder = Collections2.newStringBuilderForCollection(this.map.size()).append('{');
            JOINER.appendTo(builder, (Map<?, ?>) this.map);
            builder.append("]}");
            return builder.toString();
        }

        class AsMapEntries extends AbstractSet<Map.Entry<K, Collection<V>>> {
            AsMapEntries() {
            }

            public int size() {
                return MapMultimap.this.map.size();
            }

            public Iterator<Map.Entry<K, Collection<V>>> iterator() {
                return new TransformedIterator<K, Map.Entry<K, Collection<V>>>(MapMultimap.this.map.keySet().iterator()) {
                    /* access modifiers changed from: package-private */
                    public Map.Entry<K, Collection<V>> transform(final K key) {
                        return new AbstractMapEntry<K, Collection<V>>() {
                            public K getKey() {
                                return key;
                            }

                            public Collection<V> getValue() {
                                return MapMultimap.this.get(key);
                            }
                        };
                    }
                };
            }

            public boolean contains(Object o) {
                if (!(o instanceof Map.Entry)) {
                    return false;
                }
                Map.Entry<?, ?> entry = (Map.Entry) o;
                if (!(entry.getValue() instanceof Set)) {
                    return false;
                }
                Set<?> set = (Set) entry.getValue();
                if (set.size() != 1 || !MapMultimap.this.containsEntry(entry.getKey(), set.iterator().next())) {
                    return false;
                }
                return true;
            }

            public boolean remove(Object o) {
                if (!(o instanceof Map.Entry)) {
                    return false;
                }
                Map.Entry<?, ?> entry = (Map.Entry) o;
                if (!(entry.getValue() instanceof Set)) {
                    return false;
                }
                Set<?> set = (Set) entry.getValue();
                if (set.size() != 1 || !MapMultimap.this.map.entrySet().remove(Maps.immutableEntry(entry.getKey(), set.iterator().next()))) {
                    return false;
                }
                return true;
            }
        }

        class AsMap extends Maps.ImprovedAbstractMap<K, Collection<V>> {
            AsMap() {
            }

            /* access modifiers changed from: protected */
            public Set<Map.Entry<K, Collection<V>>> createEntrySet() {
                return new AsMapEntries();
            }

            public boolean containsKey(Object key) {
                return MapMultimap.this.map.containsKey(key);
            }

            public Collection<V> get(Object key) {
                Collection<V> collection = MapMultimap.this.get(key);
                if (collection.isEmpty()) {
                    return null;
                }
                return collection;
            }

            public Collection<V> remove(Object key) {
                Collection<V> collection = MapMultimap.this.removeAll(key);
                if (collection.isEmpty()) {
                    return null;
                }
                return collection;
            }
        }
    }

    public static <K, V1, V2> Multimap<K, V2> transformValues(Multimap<K, V1> fromMultimap, final Function<? super V1, V2> function) {
        Preconditions.checkNotNull(function);
        return transformEntries(fromMultimap, new Maps.EntryTransformer<K, V1, V2>() {
            public V2 transformEntry(K k, V1 value) {
                return function.apply(value);
            }
        });
    }

    public static <K, V1, V2> Multimap<K, V2> transformEntries(Multimap<K, V1> fromMap, Maps.EntryTransformer<? super K, ? super V1, V2> transformer) {
        return new TransformedEntriesMultimap(fromMap, transformer);
    }

    private static class TransformedEntriesMultimap<K, V1, V2> implements Multimap<K, V2> {
        private transient Map<K, Collection<V2>> asMap;
        private transient Collection<Map.Entry<K, V2>> entries;
        final Multimap<K, V1> fromMultimap;
        final Maps.EntryTransformer<? super K, ? super V1, V2> transformer;
        private transient Collection<V2> values;

        TransformedEntriesMultimap(Multimap<K, V1> fromMultimap2, Maps.EntryTransformer<? super K, ? super V1, V2> transformer2) {
            this.fromMultimap = (Multimap) Preconditions.checkNotNull(fromMultimap2);
            this.transformer = (Maps.EntryTransformer) Preconditions.checkNotNull(transformer2);
        }

        /* access modifiers changed from: package-private */
        public Collection<V2> transform(final K key, Collection<V1> values2) {
            return Collections2.transform(values2, new Function<V1, V2>() {
                public V2 apply(V1 value) {
                    return TransformedEntriesMultimap.this.transformer.transformEntry(key, value);
                }
            });
        }

        public Map<K, Collection<V2>> asMap() {
            if (this.asMap != null) {
                return this.asMap;
            }
            Map<K, Collection<V2>> aM = Maps.transformEntries(this.fromMultimap.asMap(), new Maps.EntryTransformer<K, Collection<V1>, Collection<V2>>() {
                public Collection<V2> transformEntry(K key, Collection<V1> value) {
                    return TransformedEntriesMultimap.this.transform(key, value);
                }
            });
            this.asMap = aM;
            return aM;
        }

        public void clear() {
            this.fromMultimap.clear();
        }

        public boolean containsEntry(Object key, Object value) {
            return get(key).contains(value);
        }

        public boolean containsKey(Object key) {
            return this.fromMultimap.containsKey(key);
        }

        public boolean containsValue(Object value) {
            return values().contains(value);
        }

        public Collection<Map.Entry<K, V2>> entries() {
            if (this.entries != null) {
                return this.entries;
            }
            Collection<Map.Entry<K, V2>> es = new TransformedEntries(this.transformer);
            this.entries = es;
            return es;
        }

        private class TransformedEntries extends Collections2.TransformedCollection<Map.Entry<K, V1>, Map.Entry<K, V2>> {
            TransformedEntries(final Maps.EntryTransformer<? super K, ? super V1, V2> transformer) {
                super(TransformedEntriesMultimap.this.fromMultimap.entries(), new Function<Map.Entry<K, V1>, Map.Entry<K, V2>>() {
                    public Map.Entry<K, V2> apply(final Map.Entry<K, V1> entry) {
                        return new AbstractMapEntry<K, V2>() {
                            public K getKey() {
                                return entry.getKey();
                            }

                            public V2 getValue() {
                                return transformer.transformEntry(entry.getKey(), entry.getValue());
                            }
                        };
                    }
                });
            }

            public boolean contains(Object o) {
                if (!(o instanceof Map.Entry)) {
                    return false;
                }
                Map.Entry<?, ?> entry = (Map.Entry) o;
                return TransformedEntriesMultimap.this.containsEntry(entry.getKey(), entry.getValue());
            }

            public boolean remove(Object o) {
                if (!(o instanceof Map.Entry)) {
                    return false;
                }
                Map.Entry<?, ?> entry = (Map.Entry) o;
                return TransformedEntriesMultimap.this.get(entry.getKey()).remove(entry.getValue());
            }
        }

        public Collection<V2> get(K key) {
            return transform(key, this.fromMultimap.get(key));
        }

        public boolean isEmpty() {
            return this.fromMultimap.isEmpty();
        }

        public Set<K> keySet() {
            return this.fromMultimap.keySet();
        }

        public Multiset<K> keys() {
            return this.fromMultimap.keys();
        }

        public boolean put(K k, V2 v2) {
            throw new UnsupportedOperationException();
        }

        public boolean putAll(K k, Iterable<? extends V2> iterable) {
            throw new UnsupportedOperationException();
        }

        public boolean putAll(Multimap<? extends K, ? extends V2> multimap) {
            throw new UnsupportedOperationException();
        }

        public boolean remove(Object key, Object value) {
            return get(key).remove(value);
        }

        public Collection<V2> removeAll(Object key) {
            return transform(key, this.fromMultimap.removeAll(key));
        }

        public Collection<V2> replaceValues(K k, Iterable<? extends V2> iterable) {
            throw new UnsupportedOperationException();
        }

        public int size() {
            return this.fromMultimap.size();
        }

        public Collection<V2> values() {
            if (this.values != null) {
                return this.values;
            }
            Collection<V2> vs = Collections2.transform(this.fromMultimap.entries(), new Function<Map.Entry<K, V1>, V2>() {
                public V2 apply(Map.Entry<K, V1> entry) {
                    return TransformedEntriesMultimap.this.transformer.transformEntry(entry.getKey(), entry.getValue());
                }
            });
            this.values = vs;
            return vs;
        }

        public boolean equals(Object obj) {
            if (obj instanceof Multimap) {
                return asMap().equals(((Multimap) obj).asMap());
            }
            return false;
        }

        public int hashCode() {
            return asMap().hashCode();
        }

        public String toString() {
            return asMap().toString();
        }
    }

    public static <K, V1, V2> ListMultimap<K, V2> transformValues(ListMultimap<K, V1> fromMultimap, final Function<? super V1, V2> function) {
        Preconditions.checkNotNull(function);
        return transformEntries(fromMultimap, new Maps.EntryTransformer<K, V1, V2>() {
            public V2 transformEntry(K k, V1 value) {
                return function.apply(value);
            }
        });
    }

    public static <K, V1, V2> ListMultimap<K, V2> transformEntries(ListMultimap<K, V1> fromMap, Maps.EntryTransformer<? super K, ? super V1, V2> transformer) {
        return new TransformedEntriesListMultimap(fromMap, transformer);
    }

    private static final class TransformedEntriesListMultimap<K, V1, V2> extends TransformedEntriesMultimap<K, V1, V2> implements ListMultimap<K, V2> {
        TransformedEntriesListMultimap(ListMultimap<K, V1> fromMultimap, Maps.EntryTransformer<? super K, ? super V1, V2> transformer) {
            super(fromMultimap, transformer);
        }

        /* access modifiers changed from: package-private */
        public List<V2> transform(final K key, Collection<V1> values) {
            return Lists.transform((List) values, new Function<V1, V2>() {
                public V2 apply(V1 value) {
                    return TransformedEntriesListMultimap.this.transformer.transformEntry(key, value);
                }
            });
        }

        public List<V2> get(K key) {
            return transform((Object) key, this.fromMultimap.get(key));
        }

        public List<V2> removeAll(Object key) {
            return transform(key, this.fromMultimap.removeAll(key));
        }

        public List<V2> replaceValues(K k, Iterable<? extends V2> iterable) {
            throw new UnsupportedOperationException();
        }
    }

    public static <K, V> ImmutableListMultimap<K, V> index(Iterable<V> values, Function<? super V, K> keyFunction) {
        return index(values.iterator(), keyFunction);
    }

    public static <K, V> ImmutableListMultimap<K, V> index(Iterator<V> values, Function<? super V, K> keyFunction) {
        Preconditions.checkNotNull(keyFunction);
        ImmutableListMultimap.Builder<K, V> builder = ImmutableListMultimap.builder();
        while (values.hasNext()) {
            V value = values.next();
            Preconditions.checkNotNull(value, values);
            builder.put(keyFunction.apply(value), value);
        }
        return builder.build();
    }

    static abstract class Keys<K, V> extends AbstractMultiset<K> {
        /* access modifiers changed from: package-private */
        public abstract Multimap<K, V> multimap();

        Keys() {
        }

        /* access modifiers changed from: package-private */
        public Iterator<Multiset.Entry<K>> entryIterator() {
            return new TransformedIterator<Map.Entry<K, Collection<V>>, Multiset.Entry<K>>(multimap().asMap().entrySet().iterator()) {
                /* access modifiers changed from: package-private */
                public Multiset.Entry<K> transform(final Map.Entry<K, Collection<V>> backingEntry) {
                    return new Multisets.AbstractEntry<K>() {
                        public K getElement() {
                            return backingEntry.getKey();
                        }

                        public int getCount() {
                            return ((Collection) backingEntry.getValue()).size();
                        }
                    };
                }
            };
        }

        /* access modifiers changed from: package-private */
        public int distinctElements() {
            return multimap().asMap().size();
        }

        /* access modifiers changed from: package-private */
        public Set<Multiset.Entry<K>> createEntrySet() {
            return new KeysEntrySet();
        }

        class KeysEntrySet extends Multisets.EntrySet<K> {
            KeysEntrySet() {
            }

            /* access modifiers changed from: package-private */
            public Multiset<K> multiset() {
                return Keys.this;
            }

            public Iterator<Multiset.Entry<K>> iterator() {
                return Keys.this.entryIterator();
            }

            public int size() {
                return Keys.this.distinctElements();
            }

            public boolean isEmpty() {
                return Keys.this.multimap().isEmpty();
            }

            public boolean contains(@Nullable Object o) {
                if (!(o instanceof Multiset.Entry)) {
                    return false;
                }
                Multiset.Entry<?> entry = (Multiset.Entry) o;
                Collection<V> collection = (Collection) Keys.this.multimap().asMap().get(entry.getElement());
                if (collection == null || collection.size() != entry.getCount()) {
                    return false;
                }
                return true;
            }

            public boolean remove(@Nullable Object o) {
                if (!(o instanceof Multiset.Entry)) {
                    return false;
                }
                Multiset.Entry<?> entry = (Multiset.Entry) o;
                Collection<V> collection = (Collection) Keys.this.multimap().asMap().get(entry.getElement());
                if (collection == null || collection.size() != entry.getCount()) {
                    return false;
                }
                collection.clear();
                return true;
            }
        }

        public boolean contains(@Nullable Object element) {
            return multimap().containsKey(element);
        }

        public Iterator<K> iterator() {
            return Maps.keyIterator(multimap().entries().iterator());
        }

        public int count(@Nullable Object element) {
            try {
                if (!multimap().containsKey(element)) {
                    return 0;
                }
                Collection<V> values = (Collection) multimap().asMap().get(element);
                if (values == null) {
                    return 0;
                }
                return values.size();
            } catch (ClassCastException e) {
                return 0;
            } catch (NullPointerException e2) {
                return 0;
            }
        }

        public int remove(@Nullable Object element, int occurrences) {
            Preconditions.checkArgument(occurrences >= 0);
            if (occurrences == 0) {
                return count(element);
            }
            try {
                Collection<V> values = (Collection) multimap().asMap().get(element);
                if (values == null) {
                    return 0;
                }
                int oldCount = values.size();
                if (occurrences >= oldCount) {
                    values.clear();
                } else {
                    Iterator<V> iterator = values.iterator();
                    for (int i = 0; i < occurrences; i++) {
                        iterator.next();
                        iterator.remove();
                    }
                }
                return oldCount;
            } catch (ClassCastException e) {
                return 0;
            } catch (NullPointerException e2) {
                return 0;
            }
        }

        public void clear() {
            multimap().clear();
        }

        public Set<K> elementSet() {
            return multimap().keySet();
        }
    }

    static abstract class Values<K, V> extends AbstractCollection<V> {
        /* access modifiers changed from: package-private */
        public abstract Multimap<K, V> multimap();

        Values() {
        }

        public Iterator<V> iterator() {
            return Maps.valueIterator(multimap().entries().iterator());
        }

        public int size() {
            return multimap().size();
        }

        public boolean contains(@Nullable Object o) {
            return multimap().containsValue(o);
        }

        public void clear() {
            multimap().clear();
        }
    }

    static abstract class Entries<K, V> extends AbstractCollection<Map.Entry<K, V>> {
        /* access modifiers changed from: package-private */
        public abstract Multimap<K, V> multimap();

        Entries() {
        }

        public int size() {
            return multimap().size();
        }

        public boolean contains(@Nullable Object o) {
            if (!(o instanceof Map.Entry)) {
                return false;
            }
            Map.Entry<?, ?> entry = (Map.Entry) o;
            return multimap().containsEntry(entry.getKey(), entry.getValue());
        }

        public boolean remove(@Nullable Object o) {
            if (!(o instanceof Map.Entry)) {
                return false;
            }
            Map.Entry<?, ?> entry = (Map.Entry) o;
            return multimap().remove(entry.getKey(), entry.getValue());
        }

        public void clear() {
            multimap().clear();
        }
    }

    static abstract class EntrySet<K, V> extends Entries<K, V> implements Set<Map.Entry<K, V>> {
        EntrySet() {
        }

        public int hashCode() {
            return Sets.hashCodeImpl(this);
        }

        public boolean equals(@Nullable Object obj) {
            return Sets.equalsImpl(this, obj);
        }
    }

    static abstract class AsMap<K, V> extends Maps.ImprovedAbstractMap<K, Collection<V>> {
        /* access modifiers changed from: package-private */
        public abstract Iterator<Map.Entry<K, Collection<V>>> entryIterator();

        /* access modifiers changed from: package-private */
        public abstract Multimap<K, V> multimap();

        public abstract int size();

        AsMap() {
        }

        /* access modifiers changed from: protected */
        public Set<Map.Entry<K, Collection<V>>> createEntrySet() {
            return new EntrySet();
        }

        /* access modifiers changed from: package-private */
        public void removeValuesForKey(Object key) {
            multimap().removeAll(key);
        }

        class EntrySet extends Maps.EntrySet<K, Collection<V>> {
            EntrySet() {
            }

            /* access modifiers changed from: package-private */
            public Map<K, Collection<V>> map() {
                return AsMap.this;
            }

            public Iterator<Map.Entry<K, Collection<V>>> iterator() {
                return AsMap.this.entryIterator();
            }

            public boolean remove(Object o) {
                if (!contains(o)) {
                    return false;
                }
                AsMap.this.removeValuesForKey(((Map.Entry) o).getKey());
                return true;
            }
        }

        public Collection<V> get(Object key) {
            if (containsKey(key)) {
                return multimap().get(key);
            }
            return null;
        }

        public Collection<V> remove(Object key) {
            if (containsKey(key)) {
                return multimap().removeAll(key);
            }
            return null;
        }

        public Set<K> keySet() {
            return multimap().keySet();
        }

        public boolean isEmpty() {
            return multimap().isEmpty();
        }

        public boolean containsKey(Object key) {
            return multimap().containsKey(key);
        }

        public void clear() {
            multimap().clear();
        }
    }

    @GwtIncompatible("untested")
    @Beta
    public static <K, V> Multimap<K, V> filterKeys(Multimap<K, V> unfiltered, final Predicate<? super K> keyPredicate) {
        Preconditions.checkNotNull(keyPredicate);
        return filterEntries(unfiltered, new Predicate<Map.Entry<K, V>>() {
            public boolean apply(Map.Entry<K, V> input) {
                return keyPredicate.apply(input.getKey());
            }
        });
    }

    @GwtIncompatible("untested")
    @Beta
    public static <K, V> Multimap<K, V> filterValues(Multimap<K, V> unfiltered, final Predicate<? super V> valuePredicate) {
        Preconditions.checkNotNull(valuePredicate);
        return filterEntries(unfiltered, new Predicate<Map.Entry<K, V>>() {
            public boolean apply(Map.Entry<K, V> input) {
                return valuePredicate.apply(input.getValue());
            }
        });
    }

    @GwtIncompatible("untested")
    @Beta
    public static <K, V> Multimap<K, V> filterEntries(Multimap<K, V> unfiltered, Predicate<? super Map.Entry<K, V>> entryPredicate) {
        Preconditions.checkNotNull(entryPredicate);
        return unfiltered instanceof FilteredMultimap ? filterFiltered((FilteredMultimap) unfiltered, entryPredicate) : new FilteredMultimap((Multimap) Preconditions.checkNotNull(unfiltered), entryPredicate);
    }

    private static <K, V> Multimap<K, V> filterFiltered(FilteredMultimap<K, V> map, Predicate<? super Map.Entry<K, V>> entryPredicate) {
        return new FilteredMultimap(map.unfiltered, Predicates.and(map.predicate, entryPredicate));
    }

    private static class FilteredMultimap<K, V> implements Multimap<K, V> {
        static final Predicate<Collection<?>> NOT_EMPTY = new Predicate<Collection<?>>() {
            public boolean apply(Collection<?> input) {
                return !input.isEmpty();
            }
        };
        Map<K, Collection<V>> asMap;
        Collection<Map.Entry<K, V>> entries;
        AbstractMultiset<K> keys;
        final Predicate<? super Map.Entry<K, V>> predicate;
        final Multimap<K, V> unfiltered;
        Collection<V> values;

        FilteredMultimap(Multimap<K, V> unfiltered2, Predicate<? super Map.Entry<K, V>> predicate2) {
            this.unfiltered = unfiltered2;
            this.predicate = predicate2;
        }

        public int size() {
            return entries().size();
        }

        public boolean isEmpty() {
            return entries().isEmpty();
        }

        public boolean containsKey(Object key) {
            return asMap().containsKey(key);
        }

        public boolean containsValue(Object value) {
            return values().contains(value);
        }

        /* access modifiers changed from: package-private */
        public boolean satisfiesPredicate(Object key, Object value) {
            return this.predicate.apply(Maps.immutableEntry(key, value));
        }

        public boolean containsEntry(Object key, Object value) {
            return this.unfiltered.containsEntry(key, value) && satisfiesPredicate(key, value);
        }

        public boolean put(K key, V value) {
            Preconditions.checkArgument(satisfiesPredicate(key, value));
            return this.unfiltered.put(key, value);
        }

        public boolean remove(Object key, Object value) {
            if (containsEntry(key, value)) {
                return this.unfiltered.remove(key, value);
            }
            return false;
        }

        public boolean putAll(K key, Iterable<? extends V> values2) {
            for (V value : values2) {
                Preconditions.checkArgument(satisfiesPredicate(key, value));
            }
            return this.unfiltered.putAll(key, values2);
        }

        public boolean putAll(Multimap<? extends K, ? extends V> multimap) {
            for (Map.Entry<? extends K, ? extends V> entry : multimap.entries()) {
                Preconditions.checkArgument(satisfiesPredicate(entry.getKey(), entry.getValue()));
            }
            return this.unfiltered.putAll(multimap);
        }

        public Collection<V> replaceValues(K key, Iterable<? extends V> values2) {
            for (V value : values2) {
                Preconditions.checkArgument(satisfiesPredicate(key, value));
            }
            Collection<V> oldValues = removeAll(key);
            this.unfiltered.putAll(key, values2);
            return oldValues;
        }

        public Collection<V> removeAll(Object key) {
            List<V> removed = Lists.newArrayList();
            Collection<V> values2 = this.unfiltered.asMap().get(key);
            if (values2 != null) {
                Iterator<V> iterator = values2.iterator();
                while (iterator.hasNext()) {
                    V value = iterator.next();
                    if (satisfiesPredicate(key, value)) {
                        removed.add(value);
                        iterator.remove();
                    }
                }
            }
            if (this.unfiltered instanceof SetMultimap) {
                return Collections.unmodifiableSet(Sets.newLinkedHashSet(removed));
            }
            return Collections.unmodifiableList(removed);
        }

        public void clear() {
            entries().clear();
        }

        public boolean equals(@Nullable Object object) {
            if (object == this) {
                return true;
            }
            if (object instanceof Multimap) {
                return asMap().equals(((Multimap) object).asMap());
            }
            return false;
        }

        public int hashCode() {
            return asMap().hashCode();
        }

        public String toString() {
            return asMap().toString();
        }

        class ValuePredicate implements Predicate<V> {
            final K key;

            ValuePredicate(K key2) {
                this.key = key2;
            }

            public boolean apply(V value) {
                return FilteredMultimap.this.satisfiesPredicate(this.key, value);
            }
        }

        /* access modifiers changed from: package-private */
        public Collection<V> filterCollection(Collection<V> collection, Predicate<V> predicate2) {
            if (collection instanceof Set) {
                return Sets.filter((Set) collection, predicate2);
            }
            return Collections2.filter(collection, predicate2);
        }

        public Collection<V> get(K key) {
            return filterCollection(this.unfiltered.get(key), new ValuePredicate(key));
        }

        public Set<K> keySet() {
            return asMap().keySet();
        }

        public Collection<V> values() {
            if (this.values != null) {
                return this.values;
            }
            Values values2 = new Values();
            this.values = values2;
            return values2;
        }

        class Values extends Values<K, V> {
            Values() {
            }

            /* access modifiers changed from: package-private */
            public Multimap<K, V> multimap() {
                return FilteredMultimap.this;
            }

            public boolean contains(@Nullable Object o) {
                return Iterators.contains(iterator(), o);
            }

            public boolean remove(Object o) {
                Iterator<Map.Entry<K, V>> iterator = FilteredMultimap.this.unfiltered.entries().iterator();
                while (iterator.hasNext()) {
                    Map.Entry<K, V> entry = iterator.next();
                    if (Objects.equal(o, entry.getValue()) && FilteredMultimap.this.predicate.apply(entry)) {
                        iterator.remove();
                        return true;
                    }
                }
                return false;
            }

            public boolean removeAll(Collection<?> c) {
                boolean changed = false;
                Iterator<Map.Entry<K, V>> iterator = FilteredMultimap.this.unfiltered.entries().iterator();
                while (iterator.hasNext()) {
                    Map.Entry<K, V> entry = iterator.next();
                    if (c.contains(entry.getValue()) && FilteredMultimap.this.predicate.apply(entry)) {
                        iterator.remove();
                        changed = true;
                    }
                }
                return changed;
            }

            public boolean retainAll(Collection<?> c) {
                boolean changed = false;
                Iterator<Map.Entry<K, V>> iterator = FilteredMultimap.this.unfiltered.entries().iterator();
                while (iterator.hasNext()) {
                    Map.Entry<K, V> entry = iterator.next();
                    if (!c.contains(entry.getValue()) && FilteredMultimap.this.predicate.apply(entry)) {
                        iterator.remove();
                        changed = true;
                    }
                }
                return changed;
            }
        }

        public Collection<Map.Entry<K, V>> entries() {
            if (this.entries != null) {
                return this.entries;
            }
            Collection<Map.Entry<K, V>> filter = Collections2.filter(this.unfiltered.entries(), this.predicate);
            this.entries = filter;
            return filter;
        }

        /* access modifiers changed from: package-private */
        public boolean removeEntriesIf(Predicate<Map.Entry<K, Collection<V>>> removalPredicate) {
            Iterator<Map.Entry<K, Collection<V>>> iterator = this.unfiltered.asMap().entrySet().iterator();
            boolean changed = false;
            while (iterator.hasNext()) {
                Map.Entry<K, Collection<V>> entry = iterator.next();
                K key = entry.getKey();
                Collection<V> collection = entry.getValue();
                Predicate<V> valuePredicate = new ValuePredicate(key);
                Collection<V> filteredCollection = filterCollection(collection, valuePredicate);
                if (removalPredicate.apply(Maps.immutableEntry(key, filteredCollection)) && !filteredCollection.isEmpty()) {
                    changed = true;
                    if (Iterables.all(collection, valuePredicate)) {
                        iterator.remove();
                    } else {
                        filteredCollection.clear();
                    }
                }
            }
            return changed;
        }

        public Map<K, Collection<V>> asMap() {
            if (this.asMap != null) {
                return this.asMap;
            }
            Map<K, Collection<V>> createAsMap = createAsMap();
            this.asMap = createAsMap;
            return createAsMap;
        }

        /* access modifiers changed from: package-private */
        public Map<K, Collection<V>> createAsMap() {
            return new AsMap(Maps.filterValues(Maps.transformEntries(this.unfiltered.asMap(), new Maps.EntryTransformer<K, Collection<V>, Collection<V>>() {
                public Collection<V> transformEntry(K key, Collection<V> collection) {
                    return FilteredMultimap.this.filterCollection(collection, new ValuePredicate(key));
                }
            }), NOT_EMPTY));
        }

        class AsMap extends ForwardingMap<K, Collection<V>> {
            FilteredMultimap<K, V>.AsMap.Values asMapValues;
            final Map<K, Collection<V>> delegate;
            FilteredMultimap<K, V>.AsMap.EntrySet entrySet;
            Set<K> keySet;

            AsMap(Map<K, Collection<V>> delegate2) {
                this.delegate = delegate2;
            }

            /* access modifiers changed from: protected */
            public Map<K, Collection<V>> delegate() {
                return this.delegate;
            }

            public Collection<V> remove(Object o) {
                Collection<V> output = FilteredMultimap.this.removeAll(o);
                if (output.isEmpty()) {
                    return null;
                }
                return output;
            }

            public void clear() {
                FilteredMultimap.this.clear();
            }

            public Set<K> keySet() {
                if (this.keySet != null) {
                    return this.keySet;
                }
                KeySet keySet2 = new KeySet();
                this.keySet = keySet2;
                return keySet2;
            }

            class KeySet extends Maps.KeySet<K, Collection<V>> {
                KeySet() {
                }

                /* access modifiers changed from: package-private */
                public Map<K, Collection<V>> map() {
                    return AsMap.this;
                }

                public boolean remove(Object o) {
                    Collection<V> collection = AsMap.this.delegate.get(o);
                    if (collection == null) {
                        return false;
                    }
                    collection.clear();
                    return true;
                }

                public boolean removeAll(Collection<?> c) {
                    return Sets.removeAllImpl((Set<?>) this, c.iterator());
                }

                public boolean retainAll(final Collection<?> c) {
                    return FilteredMultimap.this.removeEntriesIf(new Predicate<Map.Entry<K, Collection<V>>>() {
                        public boolean apply(Map.Entry<K, Collection<V>> entry) {
                            return !c.contains(entry.getKey());
                        }
                    });
                }
            }

            public Collection<Collection<V>> values() {
                if (this.asMapValues != null) {
                    return this.asMapValues;
                }
                Values values = new Values();
                this.asMapValues = values;
                return values;
            }

            class Values extends Maps.Values<K, Collection<V>> {
                Values() {
                }

                /* access modifiers changed from: package-private */
                public Map<K, Collection<V>> map() {
                    return AsMap.this;
                }

                public boolean remove(Object o) {
                    Iterator i$ = iterator();
                    while (i$.hasNext()) {
                        Collection<V> collection = (Collection) i$.next();
                        if (collection.equals(o)) {
                            collection.clear();
                            return true;
                        }
                    }
                    return false;
                }

                public boolean removeAll(final Collection<?> c) {
                    return FilteredMultimap.this.removeEntriesIf(new Predicate<Map.Entry<K, Collection<V>>>() {
                        public boolean apply(Map.Entry<K, Collection<V>> entry) {
                            return c.contains(entry.getValue());
                        }
                    });
                }

                public boolean retainAll(final Collection<?> c) {
                    return FilteredMultimap.this.removeEntriesIf(new Predicate<Map.Entry<K, Collection<V>>>() {
                        public boolean apply(Map.Entry<K, Collection<V>> entry) {
                            return !c.contains(entry.getValue());
                        }
                    });
                }
            }

            public Set<Map.Entry<K, Collection<V>>> entrySet() {
                if (this.entrySet != null) {
                    return this.entrySet;
                }
                EntrySet entrySet2 = new EntrySet(super.entrySet());
                this.entrySet = entrySet2;
                return entrySet2;
            }

            class EntrySet extends Maps.EntrySet<K, Collection<V>> {
                Set<Map.Entry<K, Collection<V>>> delegateEntries;

                public EntrySet(Set<Map.Entry<K, Collection<V>>> delegateEntries2) {
                    this.delegateEntries = delegateEntries2;
                }

                /* access modifiers changed from: package-private */
                public Map<K, Collection<V>> map() {
                    return AsMap.this;
                }

                public Iterator<Map.Entry<K, Collection<V>>> iterator() {
                    return this.delegateEntries.iterator();
                }

                public boolean remove(Object o) {
                    if (!(o instanceof Map.Entry)) {
                        return false;
                    }
                    Map.Entry<?, ?> entry = (Map.Entry) o;
                    Collection<V> collection = AsMap.this.delegate.get(entry.getKey());
                    if (collection == null || !collection.equals(entry.getValue())) {
                        return false;
                    }
                    collection.clear();
                    return true;
                }

                public boolean removeAll(Collection<?> c) {
                    return Sets.removeAllImpl((Set<?>) this, c);
                }

                public boolean retainAll(final Collection<?> c) {
                    return FilteredMultimap.this.removeEntriesIf(new Predicate<Map.Entry<K, Collection<V>>>() {
                        public boolean apply(Map.Entry<K, Collection<V>> entry) {
                            return !c.contains(entry);
                        }
                    });
                }
            }
        }

        public Multiset<K> keys() {
            if (this.keys != null) {
                return this.keys;
            }
            Keys keys2 = new Keys();
            this.keys = keys2;
            return keys2;
        }

        class Keys extends Keys<K, V> {
            Keys() {
            }

            /* access modifiers changed from: package-private */
            public Multimap<K, V> multimap() {
                return FilteredMultimap.this;
            }

            public int remove(Object o, int occurrences) {
                Preconditions.checkArgument(occurrences >= 0);
                Collection<V> values = FilteredMultimap.this.unfiltered.asMap().get(o);
                if (values == null) {
                    return 0;
                }
                int priorCount = 0;
                int removed = 0;
                Iterator<V> iterator = values.iterator();
                while (iterator.hasNext()) {
                    if (FilteredMultimap.this.satisfiesPredicate(o, iterator.next())) {
                        priorCount++;
                        if (removed < occurrences) {
                            iterator.remove();
                            removed++;
                        }
                    }
                }
                return priorCount;
            }

            /* access modifiers changed from: package-private */
            public Set<Multiset.Entry<K>> createEntrySet() {
                return new EntrySet();
            }

            class EntrySet extends Keys.KeysEntrySet {
                EntrySet() {
                    super();
                }

                public boolean removeAll(Collection<?> c) {
                    return Sets.removeAllImpl((Set<?>) this, c.iterator());
                }

                public boolean retainAll(final Collection<?> c) {
                    return FilteredMultimap.this.removeEntriesIf(new Predicate<Map.Entry<K, Collection<V>>>() {
                        public boolean apply(Map.Entry<K, Collection<V>> entry) {
                            return !c.contains(Multisets.immutableEntry(entry.getKey(), entry.getValue().size()));
                        }
                    });
                }
            }
        }
    }
}
