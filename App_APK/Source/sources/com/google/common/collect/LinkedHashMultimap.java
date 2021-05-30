package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.annotations.GwtIncompatible;
import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Preconditions;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.Collection;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;
import javax.annotation.Nullable;
import org.apache.commons.io.FileUtils;

@GwtCompatible(emulated = true, serializable = true)
public final class LinkedHashMultimap<K, V> extends AbstractSetMultimap<K, V> {
    private static final int DEFAULT_VALUES_PER_KEY = 8;
    @GwtIncompatible("java serialization not supported")
    private static final long serialVersionUID = 0;
    @VisibleForTesting
    transient int expectedValuesPerKey;
    transient Collection<Map.Entry<K, V>> linkedEntries;

    public /* bridge */ /* synthetic */ Map asMap() {
        return super.asMap();
    }

    public /* bridge */ /* synthetic */ void clear() {
        super.clear();
    }

    public /* bridge */ /* synthetic */ boolean containsEntry(Object x0, Object x1) {
        return super.containsEntry(x0, x1);
    }

    public /* bridge */ /* synthetic */ boolean containsKey(Object x0) {
        return super.containsKey(x0);
    }

    public /* bridge */ /* synthetic */ boolean containsValue(Object x0) {
        return super.containsValue(x0);
    }

    public /* bridge */ /* synthetic */ boolean equals(Object x0) {
        return super.equals(x0);
    }

    public /* bridge */ /* synthetic */ Set get(Object x0) {
        return super.get(x0);
    }

    public /* bridge */ /* synthetic */ int hashCode() {
        return super.hashCode();
    }

    public /* bridge */ /* synthetic */ boolean isEmpty() {
        return super.isEmpty();
    }

    public /* bridge */ /* synthetic */ Set keySet() {
        return super.keySet();
    }

    public /* bridge */ /* synthetic */ Multiset keys() {
        return super.keys();
    }

    public /* bridge */ /* synthetic */ boolean put(Object x0, Object x1) {
        return super.put(x0, x1);
    }

    public /* bridge */ /* synthetic */ boolean putAll(Multimap x0) {
        return super.putAll(x0);
    }

    public /* bridge */ /* synthetic */ boolean putAll(Object x0, Iterable x1) {
        return super.putAll(x0, x1);
    }

    public /* bridge */ /* synthetic */ boolean remove(Object x0, Object x1) {
        return super.remove(x0, x1);
    }

    public /* bridge */ /* synthetic */ Set removeAll(Object x0) {
        return super.removeAll(x0);
    }

    public /* bridge */ /* synthetic */ int size() {
        return super.size();
    }

    public /* bridge */ /* synthetic */ String toString() {
        return super.toString();
    }

    public static <K, V> LinkedHashMultimap<K, V> create() {
        return new LinkedHashMultimap<>();
    }

    public static <K, V> LinkedHashMultimap<K, V> create(int expectedKeys, int expectedValuesPerKey2) {
        return new LinkedHashMultimap<>(expectedKeys, expectedValuesPerKey2);
    }

    public static <K, V> LinkedHashMultimap<K, V> create(Multimap<? extends K, ? extends V> multimap) {
        return new LinkedHashMultimap<>(multimap);
    }

    private LinkedHashMultimap() {
        super(new LinkedHashMap());
        this.expectedValuesPerKey = 8;
        this.linkedEntries = Sets.newLinkedHashSet();
    }

    private LinkedHashMultimap(int expectedKeys, int expectedValuesPerKey2) {
        super(new LinkedHashMap(expectedKeys));
        this.expectedValuesPerKey = 8;
        Preconditions.checkArgument(expectedValuesPerKey2 >= 0);
        this.expectedValuesPerKey = expectedValuesPerKey2;
        this.linkedEntries = new LinkedHashSet((int) Math.min(FileUtils.ONE_GB, ((long) expectedKeys) * ((long) expectedValuesPerKey2)));
    }

    private LinkedHashMultimap(Multimap<? extends K, ? extends V> multimap) {
        super(new LinkedHashMap(Maps.capacity(multimap.keySet().size())));
        this.expectedValuesPerKey = 8;
        this.linkedEntries = new LinkedHashSet(Maps.capacity(multimap.size()));
        putAll(multimap);
    }

    /* access modifiers changed from: package-private */
    public Set<V> createCollection() {
        return new LinkedHashSet(Maps.capacity(this.expectedValuesPerKey));
    }

    /* access modifiers changed from: package-private */
    public Collection<V> createCollection(@Nullable K key) {
        return new SetDecorator(key, createCollection());
    }

    private class SetDecorator extends ForwardingSet<V> {
        final Set<V> delegate;
        final K key;

        SetDecorator(@Nullable K key2, Set<V> delegate2) {
            this.delegate = delegate2;
            this.key = key2;
        }

        /* access modifiers changed from: protected */
        public Set<V> delegate() {
            return this.delegate;
        }

        /* access modifiers changed from: package-private */
        public <E> Map.Entry<K, E> createEntry(@Nullable E value) {
            return Maps.immutableEntry(this.key, value);
        }

        /* access modifiers changed from: package-private */
        public <E> Collection<Map.Entry<K, E>> createEntries(Collection<E> values) {
            Collection<Map.Entry<K, E>> entries = Lists.newArrayListWithExpectedSize(values.size());
            for (E value : values) {
                entries.add(createEntry(value));
            }
            return entries;
        }

        public boolean add(@Nullable V value) {
            boolean changed = this.delegate.add(value);
            if (changed) {
                LinkedHashMultimap.this.linkedEntries.add(createEntry(value));
            }
            return changed;
        }

        public boolean addAll(Collection<? extends V> values) {
            boolean changed = this.delegate.addAll(values);
            if (changed) {
                LinkedHashMultimap.this.linkedEntries.addAll(createEntries(delegate()));
            }
            return changed;
        }

        public void clear() {
            for (V value : this.delegate) {
                LinkedHashMultimap.this.linkedEntries.remove(createEntry(value));
            }
            this.delegate.clear();
        }

        public Iterator<V> iterator() {
            final Iterator<V> delegateIterator = this.delegate.iterator();
            return new Iterator<V>() {
                V value;

                public boolean hasNext() {
                    return delegateIterator.hasNext();
                }

                public V next() {
                    this.value = delegateIterator.next();
                    return this.value;
                }

                public void remove() {
                    delegateIterator.remove();
                    LinkedHashMultimap.this.linkedEntries.remove(SetDecorator.this.createEntry(this.value));
                }
            };
        }

        public boolean remove(@Nullable Object value) {
            boolean changed = this.delegate.remove(value);
            if (changed) {
                LinkedHashMultimap.this.linkedEntries.remove(createEntry(value));
            }
            return changed;
        }

        public boolean removeAll(Collection<?> values) {
            boolean changed = this.delegate.removeAll(values);
            if (changed) {
                LinkedHashMultimap.this.linkedEntries.removeAll(createEntries(values));
            }
            return changed;
        }

        public boolean retainAll(Collection<?> values) {
            boolean changed = false;
            Iterator<V> iterator = this.delegate.iterator();
            while (iterator.hasNext()) {
                V value = iterator.next();
                if (!values.contains(value)) {
                    iterator.remove();
                    LinkedHashMultimap.this.linkedEntries.remove(Maps.immutableEntry(this.key, value));
                    changed = true;
                }
            }
            return changed;
        }
    }

    /* access modifiers changed from: package-private */
    public Iterator<Map.Entry<K, V>> createEntryIterator() {
        final Iterator<Map.Entry<K, V>> delegateIterator = this.linkedEntries.iterator();
        return new Iterator<Map.Entry<K, V>>() {
            Map.Entry<K, V> entry;

            public boolean hasNext() {
                return delegateIterator.hasNext();
            }

            public Map.Entry<K, V> next() {
                this.entry = (Map.Entry) delegateIterator.next();
                return this.entry;
            }

            public void remove() {
                delegateIterator.remove();
                LinkedHashMultimap.this.remove(this.entry.getKey(), this.entry.getValue());
            }
        };
    }

    public Set<V> replaceValues(@Nullable K key, Iterable<? extends V> values) {
        return super.replaceValues((Object) key, (Iterable) values);
    }

    public Set<Map.Entry<K, V>> entries() {
        return super.entries();
    }

    public Collection<V> values() {
        return super.values();
    }

    @GwtIncompatible("java.io.ObjectOutputStream")
    private void writeObject(ObjectOutputStream stream) throws IOException {
        stream.defaultWriteObject();
        stream.writeInt(this.expectedValuesPerKey);
        Serialization.writeMultimap(this, stream);
        for (Map.Entry<K, V> entry : this.linkedEntries) {
            stream.writeObject(entry.getKey());
            stream.writeObject(entry.getValue());
        }
    }

    @GwtIncompatible("java.io.ObjectInputStream")
    private void readObject(ObjectInputStream stream) throws IOException, ClassNotFoundException {
        stream.defaultReadObject();
        this.expectedValuesPerKey = stream.readInt();
        int distinctKeys = Serialization.readCount(stream);
        setMap(new LinkedHashMap(Maps.capacity(distinctKeys)));
        this.linkedEntries = new LinkedHashSet(this.expectedValuesPerKey * distinctKeys);
        Serialization.populateMultimap(this, stream, distinctKeys);
        this.linkedEntries.clear();
        for (int i = 0; i < size(); i++) {
            this.linkedEntries.add(Maps.immutableEntry(stream.readObject(), stream.readObject()));
        }
    }
}
