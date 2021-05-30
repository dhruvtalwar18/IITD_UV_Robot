package com.google.common.collect;

import WrappedCollection.WrappedIterator;
import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Preconditions;
import com.google.common.collect.Maps;
import com.google.common.collect.Multimaps;
import java.io.Serializable;
import java.util.AbstractCollection;
import java.util.AbstractMap;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.ConcurrentModificationException;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;
import java.util.RandomAccess;
import java.util.Set;
import java.util.SortedMap;
import java.util.SortedSet;
import javax.annotation.Nullable;

@GwtCompatible
abstract class AbstractMultimap<K, V> implements Multimap<K, V>, Serializable {
    private static final long serialVersionUID = 2447537837011683357L;
    private transient Map<K, Collection<V>> asMap;
    private transient Collection<Map.Entry<K, V>> entries;
    private transient Set<K> keySet;
    /* access modifiers changed from: private */
    public transient Map<K, Collection<V>> map;
    private transient Multiset<K> multiset;
    private transient int totalSize;
    private transient Collection<V> valuesCollection;

    /* access modifiers changed from: package-private */
    public abstract Collection<V> createCollection();

    static /* synthetic */ int access$208(AbstractMultimap x0) {
        int i = x0.totalSize;
        x0.totalSize = i + 1;
        return i;
    }

    static /* synthetic */ int access$210(AbstractMultimap x0) {
        int i = x0.totalSize;
        x0.totalSize = i - 1;
        return i;
    }

    static /* synthetic */ int access$212(AbstractMultimap x0, int x1) {
        int i = x0.totalSize + x1;
        x0.totalSize = i;
        return i;
    }

    static /* synthetic */ int access$220(AbstractMultimap x0, int x1) {
        int i = x0.totalSize - x1;
        x0.totalSize = i;
        return i;
    }

    protected AbstractMultimap(Map<K, Collection<V>> map2) {
        Preconditions.checkArgument(map2.isEmpty());
        this.map = map2;
    }

    /* access modifiers changed from: package-private */
    public final void setMap(Map<K, Collection<V>> map2) {
        this.map = map2;
        this.totalSize = 0;
        for (Collection<V> values : map2.values()) {
            Preconditions.checkArgument(!values.isEmpty());
            this.totalSize += values.size();
        }
    }

    /* access modifiers changed from: package-private */
    public Collection<V> createCollection(@Nullable K k) {
        return createCollection();
    }

    /* access modifiers changed from: package-private */
    public Map<K, Collection<V>> backingMap() {
        return this.map;
    }

    public int size() {
        return this.totalSize;
    }

    public boolean isEmpty() {
        return this.totalSize == 0;
    }

    public boolean containsKey(@Nullable Object key) {
        return this.map.containsKey(key);
    }

    public boolean containsValue(@Nullable Object value) {
        for (Collection<V> collection : this.map.values()) {
            if (collection.contains(value)) {
                return true;
            }
        }
        return false;
    }

    public boolean containsEntry(@Nullable Object key, @Nullable Object value) {
        Collection<V> collection = this.map.get(key);
        return collection != null && collection.contains(value);
    }

    public boolean put(@Nullable K key, @Nullable V value) {
        Collection<V> collection = this.map.get(key);
        if (collection == null) {
            Collection<V> collection2 = createCollection(key);
            if (collection2.add(value)) {
                this.totalSize++;
                this.map.put(key, collection2);
                return true;
            }
            throw new AssertionError("New Collection violated the Collection spec");
        } else if (!collection.add(value)) {
            return false;
        } else {
            this.totalSize++;
            return true;
        }
    }

    private Collection<V> getOrCreateCollection(@Nullable K key) {
        Collection<V> collection = this.map.get(key);
        if (collection != null) {
            return collection;
        }
        Collection<V> collection2 = createCollection(key);
        this.map.put(key, collection2);
        return collection2;
    }

    public boolean remove(@Nullable Object key, @Nullable Object value) {
        Collection<V> collection = this.map.get(key);
        if (collection == null) {
            return false;
        }
        boolean changed = collection.remove(value);
        if (changed) {
            this.totalSize--;
            if (collection.isEmpty()) {
                this.map.remove(key);
            }
        }
        return changed;
    }

    public boolean putAll(@Nullable K key, Iterable<? extends V> values) {
        if (!values.iterator().hasNext()) {
            return false;
        }
        Collection<V> collection = getOrCreateCollection(key);
        int oldSize = collection.size();
        boolean changed = false;
        if (values instanceof Collection) {
            changed = collection.addAll(Collections2.cast(values));
        } else {
            for (V value : values) {
                changed |= collection.add(value);
            }
        }
        this.totalSize += collection.size() - oldSize;
        return changed;
    }

    public boolean putAll(Multimap<? extends K, ? extends V> multimap) {
        boolean changed = false;
        for (Map.Entry<? extends K, ? extends V> entry : multimap.entries()) {
            changed |= put(entry.getKey(), entry.getValue());
        }
        return changed;
    }

    public Collection<V> replaceValues(@Nullable K key, Iterable<? extends V> values) {
        Iterator<? extends V> iterator = values.iterator();
        if (!iterator.hasNext()) {
            return removeAll(key);
        }
        Collection<V> collection = getOrCreateCollection(key);
        Collection<V> oldValues = createCollection();
        oldValues.addAll(collection);
        this.totalSize -= collection.size();
        collection.clear();
        while (iterator.hasNext()) {
            if (collection.add(iterator.next())) {
                this.totalSize++;
            }
        }
        return unmodifiableCollectionSubclass(oldValues);
    }

    public Collection<V> removeAll(@Nullable Object key) {
        Collection<V> collection = this.map.remove(key);
        Collection<V> output = createCollection();
        if (collection != null) {
            output.addAll(collection);
            this.totalSize -= collection.size();
            collection.clear();
        }
        return unmodifiableCollectionSubclass(output);
    }

    private Collection<V> unmodifiableCollectionSubclass(Collection<V> collection) {
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

    public void clear() {
        for (Collection<V> collection : this.map.values()) {
            collection.clear();
        }
        this.map.clear();
        this.totalSize = 0;
    }

    public Collection<V> get(@Nullable K key) {
        Collection<V> collection = this.map.get(key);
        if (collection == null) {
            collection = createCollection(key);
        }
        return wrapCollection(key, collection);
    }

    /* access modifiers changed from: private */
    public Collection<V> wrapCollection(@Nullable K key, Collection<V> collection) {
        if (collection instanceof SortedSet) {
            return new WrappedSortedSet(key, (SortedSet) collection, (AbstractMultimap<K, V>.WrappedCollection) null);
        }
        if (collection instanceof Set) {
            return new WrappedSet(key, (Set) collection);
        }
        if (collection instanceof List) {
            return wrapList(key, (List) collection, (AbstractMultimap<K, V>.WrappedCollection) null);
        }
        return new WrappedCollection(key, collection, (AbstractMultimap<K, V>.WrappedCollection) null);
    }

    /* access modifiers changed from: private */
    public List<V> wrapList(@Nullable K key, List<V> list, @Nullable AbstractMultimap<K, V>.WrappedCollection ancestor) {
        return list instanceof RandomAccess ? new RandomAccessWrappedList(key, list, ancestor) : new WrappedList(key, list, ancestor);
    }

    private class WrappedCollection extends AbstractCollection<V> {
        final AbstractMultimap<K, V>.WrappedCollection ancestor;
        final Collection<V> ancestorDelegate;
        Collection<V> delegate;
        final K key;

        WrappedCollection(@Nullable K key2, Collection<V> delegate2, @Nullable AbstractMultimap<K, V>.WrappedCollection ancestor2) {
            this.key = key2;
            this.delegate = delegate2;
            this.ancestor = ancestor2;
            this.ancestorDelegate = ancestor2 == null ? null : ancestor2.getDelegate();
        }

        /* access modifiers changed from: package-private */
        public void refreshIfEmpty() {
            Collection<V> newDelegate;
            if (this.ancestor != null) {
                this.ancestor.refreshIfEmpty();
                if (this.ancestor.getDelegate() != this.ancestorDelegate) {
                    throw new ConcurrentModificationException();
                }
            } else if (this.delegate.isEmpty() && (newDelegate = (Collection) AbstractMultimap.this.map.get(this.key)) != null) {
                this.delegate = newDelegate;
            }
        }

        /* access modifiers changed from: package-private */
        public void removeIfEmpty() {
            if (this.ancestor != null) {
                this.ancestor.removeIfEmpty();
            } else if (this.delegate.isEmpty()) {
                AbstractMultimap.this.map.remove(this.key);
            }
        }

        /* access modifiers changed from: package-private */
        public K getKey() {
            return this.key;
        }

        /* access modifiers changed from: package-private */
        public void addToMap() {
            if (this.ancestor != null) {
                this.ancestor.addToMap();
            } else {
                AbstractMultimap.this.map.put(this.key, this.delegate);
            }
        }

        public int size() {
            refreshIfEmpty();
            return this.delegate.size();
        }

        public boolean equals(@Nullable Object object) {
            if (object == this) {
                return true;
            }
            refreshIfEmpty();
            return this.delegate.equals(object);
        }

        public int hashCode() {
            refreshIfEmpty();
            return this.delegate.hashCode();
        }

        public String toString() {
            refreshIfEmpty();
            return this.delegate.toString();
        }

        /* access modifiers changed from: package-private */
        public Collection<V> getDelegate() {
            return this.delegate;
        }

        public Iterator<V> iterator() {
            refreshIfEmpty();
            return new WrappedIterator();
        }

        class WrappedIterator implements Iterator<V> {
            final Iterator<V> delegateIterator;
            final Collection<V> originalDelegate = WrappedCollection.this.delegate;

            WrappedIterator() {
                this.delegateIterator = AbstractMultimap.this.iteratorOrListIterator(WrappedCollection.this.delegate);
            }

            WrappedIterator(Iterator<V> delegateIterator2) {
                this.delegateIterator = delegateIterator2;
            }

            /* access modifiers changed from: package-private */
            public void validateIterator() {
                WrappedCollection.this.refreshIfEmpty();
                if (WrappedCollection.this.delegate != this.originalDelegate) {
                    throw new ConcurrentModificationException();
                }
            }

            public boolean hasNext() {
                validateIterator();
                return this.delegateIterator.hasNext();
            }

            public V next() {
                validateIterator();
                return this.delegateIterator.next();
            }

            public void remove() {
                this.delegateIterator.remove();
                AbstractMultimap.access$210(AbstractMultimap.this);
                WrappedCollection.this.removeIfEmpty();
            }

            /* access modifiers changed from: package-private */
            public Iterator<V> getDelegateIterator() {
                validateIterator();
                return this.delegateIterator;
            }
        }

        public boolean add(V value) {
            refreshIfEmpty();
            boolean wasEmpty = this.delegate.isEmpty();
            boolean changed = this.delegate.add(value);
            if (changed) {
                AbstractMultimap.access$208(AbstractMultimap.this);
                if (wasEmpty) {
                    addToMap();
                }
            }
            return changed;
        }

        /* access modifiers changed from: package-private */
        public AbstractMultimap<K, V>.WrappedCollection getAncestor() {
            return this.ancestor;
        }

        public boolean addAll(Collection<? extends V> collection) {
            if (collection.isEmpty()) {
                return false;
            }
            int oldSize = size();
            boolean changed = this.delegate.addAll(collection);
            if (changed) {
                AbstractMultimap.access$212(AbstractMultimap.this, this.delegate.size() - oldSize);
                if (oldSize == 0) {
                    addToMap();
                }
            }
            return changed;
        }

        public boolean contains(Object o) {
            refreshIfEmpty();
            return this.delegate.contains(o);
        }

        public boolean containsAll(Collection<?> c) {
            refreshIfEmpty();
            return this.delegate.containsAll(c);
        }

        public void clear() {
            int oldSize = size();
            if (oldSize != 0) {
                this.delegate.clear();
                AbstractMultimap.access$220(AbstractMultimap.this, oldSize);
                removeIfEmpty();
            }
        }

        public boolean remove(Object o) {
            refreshIfEmpty();
            boolean changed = this.delegate.remove(o);
            if (changed) {
                AbstractMultimap.access$210(AbstractMultimap.this);
                removeIfEmpty();
            }
            return changed;
        }

        public boolean removeAll(Collection<?> c) {
            if (c.isEmpty()) {
                return false;
            }
            int oldSize = size();
            boolean changed = this.delegate.removeAll(c);
            if (changed) {
                AbstractMultimap.access$212(AbstractMultimap.this, this.delegate.size() - oldSize);
                removeIfEmpty();
            }
            return changed;
        }

        public boolean retainAll(Collection<?> c) {
            Preconditions.checkNotNull(c);
            int oldSize = size();
            boolean changed = this.delegate.retainAll(c);
            if (changed) {
                AbstractMultimap.access$212(AbstractMultimap.this, this.delegate.size() - oldSize);
                removeIfEmpty();
            }
            return changed;
        }
    }

    /* access modifiers changed from: private */
    public Iterator<V> iteratorOrListIterator(Collection<V> collection) {
        return collection instanceof List ? ((List) collection).listIterator() : collection.iterator();
    }

    private class WrappedSet extends AbstractMultimap<K, V>.WrappedCollection implements Set<V> {
        WrappedSet(@Nullable K key, Set<V> delegate) {
            super(key, delegate, (AbstractMultimap<K, V>.WrappedCollection) null);
        }
    }

    private class WrappedSortedSet extends AbstractMultimap<K, V>.WrappedCollection implements SortedSet<V> {
        WrappedSortedSet(@Nullable K key, SortedSet<V> delegate, @Nullable AbstractMultimap<K, V>.WrappedCollection ancestor) {
            super(key, delegate, ancestor);
        }

        /* access modifiers changed from: package-private */
        public SortedSet<V> getSortedSetDelegate() {
            return (SortedSet) getDelegate();
        }

        public Comparator<? super V> comparator() {
            return getSortedSetDelegate().comparator();
        }

        public V first() {
            refreshIfEmpty();
            return getSortedSetDelegate().first();
        }

        public V last() {
            refreshIfEmpty();
            return getSortedSetDelegate().last();
        }

        public SortedSet<V> headSet(V toElement) {
            refreshIfEmpty();
            return new WrappedSortedSet(getKey(), getSortedSetDelegate().headSet(toElement), getAncestor() == null ? this : getAncestor());
        }

        public SortedSet<V> subSet(V fromElement, V toElement) {
            refreshIfEmpty();
            return new WrappedSortedSet(getKey(), getSortedSetDelegate().subSet(fromElement, toElement), getAncestor() == null ? this : getAncestor());
        }

        public SortedSet<V> tailSet(V fromElement) {
            refreshIfEmpty();
            return new WrappedSortedSet(getKey(), getSortedSetDelegate().tailSet(fromElement), getAncestor() == null ? this : getAncestor());
        }
    }

    private class WrappedList extends AbstractMultimap<K, V>.WrappedCollection implements List<V> {
        WrappedList(@Nullable K key, List<V> delegate, @Nullable AbstractMultimap<K, V>.WrappedCollection ancestor) {
            super(key, delegate, ancestor);
        }

        /* access modifiers changed from: package-private */
        public List<V> getListDelegate() {
            return (List) getDelegate();
        }

        public boolean addAll(int index, Collection<? extends V> c) {
            if (c.isEmpty()) {
                return false;
            }
            int oldSize = size();
            boolean changed = getListDelegate().addAll(index, c);
            if (changed) {
                AbstractMultimap.access$212(AbstractMultimap.this, getDelegate().size() - oldSize);
                if (oldSize == 0) {
                    addToMap();
                }
            }
            return changed;
        }

        public V get(int index) {
            refreshIfEmpty();
            return getListDelegate().get(index);
        }

        public V set(int index, V element) {
            refreshIfEmpty();
            return getListDelegate().set(index, element);
        }

        public void add(int index, V element) {
            refreshIfEmpty();
            boolean wasEmpty = getDelegate().isEmpty();
            getListDelegate().add(index, element);
            AbstractMultimap.access$208(AbstractMultimap.this);
            if (wasEmpty) {
                addToMap();
            }
        }

        public V remove(int index) {
            refreshIfEmpty();
            V value = getListDelegate().remove(index);
            AbstractMultimap.access$210(AbstractMultimap.this);
            removeIfEmpty();
            return value;
        }

        public int indexOf(Object o) {
            refreshIfEmpty();
            return getListDelegate().indexOf(o);
        }

        public int lastIndexOf(Object o) {
            refreshIfEmpty();
            return getListDelegate().lastIndexOf(o);
        }

        public ListIterator<V> listIterator() {
            refreshIfEmpty();
            return new WrappedListIterator();
        }

        public ListIterator<V> listIterator(int index) {
            refreshIfEmpty();
            return new WrappedListIterator(index);
        }

        public List<V> subList(int fromIndex, int toIndex) {
            refreshIfEmpty();
            return AbstractMultimap.this.wrapList(getKey(), getListDelegate().subList(fromIndex, toIndex), getAncestor() == null ? this : getAncestor());
        }

        private class WrappedListIterator extends AbstractMultimap<K, V>.WrappedIterator implements ListIterator<V> {
            WrappedListIterator() {
                super();
            }

            public WrappedListIterator(int index) {
                super(WrappedList.this.getListDelegate().listIterator(index));
            }

            private ListIterator<V> getDelegateListIterator() {
                return (ListIterator) getDelegateIterator();
            }

            public boolean hasPrevious() {
                return getDelegateListIterator().hasPrevious();
            }

            public V previous() {
                return getDelegateListIterator().previous();
            }

            public int nextIndex() {
                return getDelegateListIterator().nextIndex();
            }

            public int previousIndex() {
                return getDelegateListIterator().previousIndex();
            }

            public void set(V value) {
                getDelegateListIterator().set(value);
            }

            public void add(V value) {
                boolean wasEmpty = WrappedList.this.isEmpty();
                getDelegateListIterator().add(value);
                AbstractMultimap.access$208(AbstractMultimap.this);
                if (wasEmpty) {
                    WrappedList.this.addToMap();
                }
            }
        }
    }

    private class RandomAccessWrappedList extends WrappedList implements RandomAccess {
        RandomAccessWrappedList(@Nullable K key, List<V> delegate, @Nullable AbstractMultimap<K, V>.WrappedCollection ancestor) {
            super(key, delegate, ancestor);
        }
    }

    public Set<K> keySet() {
        Set<K> result = this.keySet;
        if (result != null) {
            return result;
        }
        Set<K> createKeySet = createKeySet();
        this.keySet = createKeySet;
        return createKeySet;
    }

    private Set<K> createKeySet() {
        return this.map instanceof SortedMap ? new SortedKeySet((SortedMap) this.map) : new KeySet(this.map);
    }

    private class KeySet extends Maps.KeySet<K, Collection<V>> {
        final Map<K, Collection<V>> subMap;

        KeySet(Map<K, Collection<V>> subMap2) {
            this.subMap = subMap2;
        }

        /* access modifiers changed from: package-private */
        public Map<K, Collection<V>> map() {
            return this.subMap;
        }

        public Iterator<K> iterator() {
            return new Iterator<K>() {
                Map.Entry<K, Collection<V>> entry;
                final Iterator<Map.Entry<K, Collection<V>>> entryIterator = KeySet.this.subMap.entrySet().iterator();

                public boolean hasNext() {
                    return this.entryIterator.hasNext();
                }

                public K next() {
                    this.entry = this.entryIterator.next();
                    return this.entry.getKey();
                }

                public void remove() {
                    Iterators.checkRemove(this.entry != null);
                    Collection<V> collection = this.entry.getValue();
                    this.entryIterator.remove();
                    AbstractMultimap.access$220(AbstractMultimap.this, collection.size());
                    collection.clear();
                }
            };
        }

        public boolean remove(Object key) {
            int count = 0;
            Collection<V> collection = this.subMap.remove(key);
            if (collection != null) {
                count = collection.size();
                collection.clear();
                AbstractMultimap.access$220(AbstractMultimap.this, count);
            }
            return count > 0;
        }

        public void clear() {
            Iterators.clear(iterator());
        }

        public boolean containsAll(Collection<?> c) {
            return this.subMap.keySet().containsAll(c);
        }

        public boolean equals(@Nullable Object object) {
            return this == object || this.subMap.keySet().equals(object);
        }

        public int hashCode() {
            return this.subMap.keySet().hashCode();
        }
    }

    private class SortedKeySet extends AbstractMultimap<K, V>.KeySet implements SortedSet<K> {
        SortedKeySet(SortedMap<K, Collection<V>> subMap) {
            super(subMap);
        }

        /* access modifiers changed from: package-private */
        public SortedMap<K, Collection<V>> sortedMap() {
            return (SortedMap) this.subMap;
        }

        public Comparator<? super K> comparator() {
            return sortedMap().comparator();
        }

        public K first() {
            return sortedMap().firstKey();
        }

        public SortedSet<K> headSet(K toElement) {
            return new SortedKeySet(sortedMap().headMap(toElement));
        }

        public K last() {
            return sortedMap().lastKey();
        }

        public SortedSet<K> subSet(K fromElement, K toElement) {
            return new SortedKeySet(sortedMap().subMap(fromElement, toElement));
        }

        public SortedSet<K> tailSet(K fromElement) {
            return new SortedKeySet(sortedMap().tailMap(fromElement));
        }
    }

    public Multiset<K> keys() {
        Multiset<K> result = this.multiset;
        if (result != null) {
            return result;
        }
        AnonymousClass1 r1 = new Multimaps.Keys<K, V>() {
            /* access modifiers changed from: package-private */
            public Multimap<K, V> multimap() {
                return AbstractMultimap.this;
            }
        };
        this.multiset = r1;
        return r1;
    }

    /* access modifiers changed from: private */
    public int removeValuesForKey(Object key) {
        try {
            Collection<V> collection = this.map.remove(key);
            if (collection == null) {
                return 0;
            }
            int count = collection.size();
            collection.clear();
            this.totalSize -= count;
            return count;
        } catch (NullPointerException e) {
            return 0;
        } catch (ClassCastException e2) {
            return 0;
        }
    }

    public Collection<V> values() {
        Collection<V> result = this.valuesCollection;
        if (result != null) {
            return result;
        }
        AnonymousClass2 r1 = new Multimaps.Values<K, V>() {
            /* access modifiers changed from: package-private */
            public Multimap<K, V> multimap() {
                return AbstractMultimap.this;
            }
        };
        this.valuesCollection = r1;
        return r1;
    }

    public Collection<Map.Entry<K, V>> entries() {
        Collection<Map.Entry<K, V>> result = this.entries;
        if (result != null) {
            return result;
        }
        Collection<Map.Entry<K, V>> createEntries = createEntries();
        this.entries = createEntries;
        return createEntries;
    }

    /* access modifiers changed from: package-private */
    public Collection<Map.Entry<K, V>> createEntries() {
        if (this instanceof SetMultimap) {
            return new Multimaps.EntrySet<K, V>() {
                /* access modifiers changed from: package-private */
                public Multimap<K, V> multimap() {
                    return AbstractMultimap.this;
                }

                public Iterator<Map.Entry<K, V>> iterator() {
                    return AbstractMultimap.this.createEntryIterator();
                }
            };
        }
        return new Multimaps.Entries<K, V>() {
            /* access modifiers changed from: package-private */
            public Multimap<K, V> multimap() {
                return AbstractMultimap.this;
            }

            public Iterator<Map.Entry<K, V>> iterator() {
                return AbstractMultimap.this.createEntryIterator();
            }
        };
    }

    /* access modifiers changed from: package-private */
    public Iterator<Map.Entry<K, V>> createEntryIterator() {
        return new EntryIterator();
    }

    private class EntryIterator implements Iterator<Map.Entry<K, V>> {
        Collection<V> collection;
        K key;
        final Iterator<Map.Entry<K, Collection<V>>> keyIterator;
        Iterator<V> valueIterator;

        EntryIterator() {
            this.keyIterator = AbstractMultimap.this.map.entrySet().iterator();
            if (this.keyIterator.hasNext()) {
                findValueIteratorAndKey();
            } else {
                this.valueIterator = Iterators.emptyModifiableIterator();
            }
        }

        /* access modifiers changed from: package-private */
        public void findValueIteratorAndKey() {
            Map.Entry<K, Collection<V>> entry = this.keyIterator.next();
            this.key = entry.getKey();
            this.collection = entry.getValue();
            this.valueIterator = this.collection.iterator();
        }

        public boolean hasNext() {
            return this.keyIterator.hasNext() || this.valueIterator.hasNext();
        }

        public Map.Entry<K, V> next() {
            if (!this.valueIterator.hasNext()) {
                findValueIteratorAndKey();
            }
            return Maps.immutableEntry(this.key, this.valueIterator.next());
        }

        public void remove() {
            this.valueIterator.remove();
            if (this.collection.isEmpty()) {
                this.keyIterator.remove();
            }
            AbstractMultimap.access$210(AbstractMultimap.this);
        }
    }

    public Map<K, Collection<V>> asMap() {
        Map<K, Collection<V>> result = this.asMap;
        if (result != null) {
            return result;
        }
        Map<K, Collection<V>> createAsMap = createAsMap();
        this.asMap = createAsMap;
        return createAsMap;
    }

    private Map<K, Collection<V>> createAsMap() {
        return this.map instanceof SortedMap ? new SortedAsMap((SortedMap) this.map) : new AsMap(this.map);
    }

    private class AsMap extends AbstractMap<K, Collection<V>> {
        transient Set<Map.Entry<K, Collection<V>>> entrySet;
        final transient Map<K, Collection<V>> submap;

        AsMap(Map<K, Collection<V>> submap2) {
            this.submap = submap2;
        }

        public Set<Map.Entry<K, Collection<V>>> entrySet() {
            Set<Map.Entry<K, Collection<V>>> result = this.entrySet;
            if (result != null) {
                return result;
            }
            AsMapEntries asMapEntries = new AsMapEntries();
            this.entrySet = asMapEntries;
            return asMapEntries;
        }

        public boolean containsKey(Object key) {
            return Maps.safeContainsKey(this.submap, key);
        }

        public Collection<V> get(Object key) {
            Collection<V> collection = (Collection) Maps.safeGet(this.submap, key);
            if (collection == null) {
                return null;
            }
            return AbstractMultimap.this.wrapCollection(key, collection);
        }

        public Set<K> keySet() {
            return AbstractMultimap.this.keySet();
        }

        public int size() {
            return this.submap.size();
        }

        public Collection<V> remove(Object key) {
            Collection<V> collection = this.submap.remove(key);
            if (collection == null) {
                return null;
            }
            Collection<V> output = AbstractMultimap.this.createCollection();
            output.addAll(collection);
            AbstractMultimap.access$220(AbstractMultimap.this, collection.size());
            collection.clear();
            return output;
        }

        public boolean equals(@Nullable Object object) {
            return this == object || this.submap.equals(object);
        }

        public int hashCode() {
            return this.submap.hashCode();
        }

        public String toString() {
            return this.submap.toString();
        }

        public void clear() {
            if (this.submap == AbstractMultimap.this.map) {
                AbstractMultimap.this.clear();
            } else {
                Iterators.clear(new AsMapIterator());
            }
        }

        class AsMapEntries extends Maps.EntrySet<K, Collection<V>> {
            AsMapEntries() {
            }

            /* access modifiers changed from: package-private */
            public Map<K, Collection<V>> map() {
                return AsMap.this;
            }

            public Iterator<Map.Entry<K, Collection<V>>> iterator() {
                return new AsMapIterator();
            }

            public boolean contains(Object o) {
                return Collections2.safeContains(AsMap.this.submap.entrySet(), o);
            }

            public boolean remove(Object o) {
                if (!contains(o)) {
                    return false;
                }
                int unused = AbstractMultimap.this.removeValuesForKey(((Map.Entry) o).getKey());
                return true;
            }
        }

        class AsMapIterator implements Iterator<Map.Entry<K, Collection<V>>> {
            Collection<V> collection;
            final Iterator<Map.Entry<K, Collection<V>>> delegateIterator = AsMap.this.submap.entrySet().iterator();

            AsMapIterator() {
            }

            public boolean hasNext() {
                return this.delegateIterator.hasNext();
            }

            public Map.Entry<K, Collection<V>> next() {
                Map.Entry<K, Collection<V>> entry = this.delegateIterator.next();
                K key = entry.getKey();
                this.collection = entry.getValue();
                return Maps.immutableEntry(key, AbstractMultimap.this.wrapCollection(key, this.collection));
            }

            public void remove() {
                this.delegateIterator.remove();
                AbstractMultimap.access$220(AbstractMultimap.this, this.collection.size());
                this.collection.clear();
            }
        }
    }

    private class SortedAsMap extends AbstractMultimap<K, V>.AsMap implements SortedMap<K, Collection<V>> {
        SortedSet<K> sortedKeySet;

        SortedAsMap(SortedMap<K, Collection<V>> submap) {
            super(submap);
        }

        /* access modifiers changed from: package-private */
        public SortedMap<K, Collection<V>> sortedMap() {
            return (SortedMap) this.submap;
        }

        public Comparator<? super K> comparator() {
            return sortedMap().comparator();
        }

        public K firstKey() {
            return sortedMap().firstKey();
        }

        public K lastKey() {
            return sortedMap().lastKey();
        }

        public SortedMap<K, Collection<V>> headMap(K toKey) {
            return new SortedAsMap(sortedMap().headMap(toKey));
        }

        public SortedMap<K, Collection<V>> subMap(K fromKey, K toKey) {
            return new SortedAsMap(sortedMap().subMap(fromKey, toKey));
        }

        public SortedMap<K, Collection<V>> tailMap(K fromKey) {
            return new SortedAsMap(sortedMap().tailMap(fromKey));
        }

        public SortedSet<K> keySet() {
            SortedSet<K> result = this.sortedKeySet;
            if (result != null) {
                return result;
            }
            SortedKeySet sortedKeySet2 = new SortedKeySet(sortedMap());
            this.sortedKeySet = sortedKeySet2;
            return sortedKeySet2;
        }
    }

    public boolean equals(@Nullable Object object) {
        if (object == this) {
            return true;
        }
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
}
