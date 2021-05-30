package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableMap;
import com.google.common.collect.SortedLists;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
import java.util.NoSuchElementException;
import java.util.SortedMap;
import javax.annotation.Nullable;

@GwtCompatible(emulated = true, serializable = true)
public class ImmutableSortedMap<K, V> extends ImmutableSortedMapFauxverideShim<K, V> implements NavigableMap<K, V> {
    private static final ImmutableSortedMap<Comparable, Object> NATURAL_EMPTY_MAP = new ImmutableSortedMap<>(ImmutableList.of(), NATURAL_ORDER);
    private static final Comparator<Comparable> NATURAL_ORDER = Ordering.natural();
    private static final long serialVersionUID = 0;
    private final transient Comparator<? super K> comparator;
    private transient ImmutableSortedMap<K, V> descendingMap;
    final transient ImmutableList<Map.Entry<K, V>> entries;

    public static <K, V> ImmutableSortedMap<K, V> of() {
        return NATURAL_EMPTY_MAP;
    }

    private static <K, V> ImmutableSortedMap<K, V> emptyMap(Comparator<? super K> comparator2) {
        if (NATURAL_ORDER.equals(comparator2)) {
            return NATURAL_EMPTY_MAP;
        }
        return new ImmutableSortedMap<>(ImmutableList.of(), comparator2);
    }

    public static <K extends Comparable<? super K>, V> ImmutableSortedMap<K, V> of(K k1, V v1) {
        return new ImmutableSortedMap<>(ImmutableList.of(entryOf(k1, v1)), Ordering.natural());
    }

    public static <K extends Comparable<? super K>, V> ImmutableSortedMap<K, V> of(K k1, V v1, K k2, V v2) {
        return new Builder(Ordering.natural()).put((Object) k1, (Object) v1).put((Object) k2, (Object) v2).build();
    }

    public static <K extends Comparable<? super K>, V> ImmutableSortedMap<K, V> of(K k1, V v1, K k2, V v2, K k3, V v3) {
        return new Builder(Ordering.natural()).put((Object) k1, (Object) v1).put((Object) k2, (Object) v2).put((Object) k3, (Object) v3).build();
    }

    public static <K extends Comparable<? super K>, V> ImmutableSortedMap<K, V> of(K k1, V v1, K k2, V v2, K k3, V v3, K k4, V v4) {
        return new Builder(Ordering.natural()).put((Object) k1, (Object) v1).put((Object) k2, (Object) v2).put((Object) k3, (Object) v3).put((Object) k4, (Object) v4).build();
    }

    public static <K extends Comparable<? super K>, V> ImmutableSortedMap<K, V> of(K k1, V v1, K k2, V v2, K k3, V v3, K k4, V v4, K k5, V v5) {
        return new Builder(Ordering.natural()).put((Object) k1, (Object) v1).put((Object) k2, (Object) v2).put((Object) k3, (Object) v3).put((Object) k4, (Object) v4).put((Object) k5, (Object) v5).build();
    }

    public static <K, V> ImmutableSortedMap<K, V> copyOf(Map<? extends K, ? extends V> map) {
        return copyOfInternal(map, Ordering.natural());
    }

    public static <K, V> ImmutableSortedMap<K, V> copyOf(Map<? extends K, ? extends V> map, Comparator<? super K> comparator2) {
        return copyOfInternal(map, (Comparator) Preconditions.checkNotNull(comparator2));
    }

    /* JADX WARNING: type inference failed for: r2v0, types: [java.util.Map, java.util.SortedMap<K, ? extends V>, java.util.SortedMap] */
    /* JADX WARNING: Unknown variable types count: 1 */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public static <K, V> com.google.common.collect.ImmutableSortedMap<K, V> copyOfSorted(java.util.SortedMap<K, ? extends V> r2) {
        /*
            java.util.Comparator r0 = r2.comparator()
            if (r0 != 0) goto L_0x0008
            java.util.Comparator<java.lang.Comparable> r0 = NATURAL_ORDER
        L_0x0008:
            com.google.common.collect.ImmutableSortedMap r1 = copyOfInternal(r2, r0)
            return r1
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.common.collect.ImmutableSortedMap.copyOfSorted(java.util.SortedMap):com.google.common.collect.ImmutableSortedMap");
    }

    private static <K, V> ImmutableSortedMap<K, V> copyOfInternal(Map<? extends K, ? extends V> map, Comparator<? super K> comparator2) {
        boolean sameComparator = false;
        if (map instanceof SortedMap) {
            Comparator<? super Object> comparator3 = ((SortedMap) map).comparator();
            sameComparator = comparator3 == null ? comparator2 == NATURAL_ORDER : comparator2.equals(comparator3);
        }
        if (sameComparator && (map instanceof ImmutableSortedMap)) {
            ImmutableSortedMap<K, V> kvMap = (ImmutableSortedMap) map;
            if (!kvMap.isPartialView()) {
                return kvMap;
            }
        }
        Map.Entry<K, V>[] entries2 = (Map.Entry[]) map.entrySet().toArray(new Map.Entry[0]);
        for (int i = 0; i < entries2.length; i++) {
            Map.Entry<K, V> entry = entries2[i];
            entries2[i] = entryOf(entry.getKey(), entry.getValue());
        }
        List<Map.Entry<K, V>> list = Arrays.asList(entries2);
        if (!sameComparator) {
            sortEntries(list, comparator2);
            validateEntries(list, comparator2);
        }
        return new ImmutableSortedMap<>(ImmutableList.copyOf(list), comparator2);
    }

    /* access modifiers changed from: private */
    public static <K, V> void sortEntries(List<Map.Entry<K, V>> entries2, final Comparator<? super K> comparator2) {
        Collections.sort(entries2, new Comparator<Map.Entry<K, V>>() {
            public int compare(Map.Entry<K, V> entry1, Map.Entry<K, V> entry2) {
                return comparator2.compare(entry1.getKey(), entry2.getKey());
            }
        });
    }

    /* access modifiers changed from: private */
    public static <K, V> void validateEntries(List<Map.Entry<K, V>> entries2, Comparator<? super K> comparator2) {
        int i = 1;
        while (i < entries2.size()) {
            if (comparator2.compare(entries2.get(i - 1).getKey(), entries2.get(i).getKey()) != 0) {
                i++;
            } else {
                throw new IllegalArgumentException("Duplicate keys in mappings " + entries2.get(i - 1) + " and " + entries2.get(i));
            }
        }
    }

    public static <K extends Comparable<K>, V> Builder<K, V> naturalOrder() {
        return new Builder<>(Ordering.natural());
    }

    public static <K, V> Builder<K, V> orderedBy(Comparator<K> comparator2) {
        return new Builder<>(comparator2);
    }

    public static <K extends Comparable<K>, V> Builder<K, V> reverseOrder() {
        return new Builder<>(Ordering.natural().reverse());
    }

    public static class Builder<K, V> extends ImmutableMap.Builder<K, V> {
        private final Comparator<? super K> comparator;

        public Builder(Comparator<? super K> comparator2) {
            this.comparator = (Comparator) Preconditions.checkNotNull(comparator2);
        }

        public Builder<K, V> put(K key, V value) {
            this.entries.add(ImmutableMap.entryOf(key, value));
            return this;
        }

        public Builder<K, V> put(Map.Entry<? extends K, ? extends V> entry) {
            super.put(entry);
            return this;
        }

        public Builder<K, V> putAll(Map<? extends K, ? extends V> map) {
            for (Map.Entry<? extends K, ? extends V> entry : map.entrySet()) {
                put((Object) entry.getKey(), (Object) entry.getValue());
            }
            return this;
        }

        public ImmutableSortedMap<K, V> build() {
            ImmutableSortedMap.sortEntries(this.entries, this.comparator);
            ImmutableSortedMap.validateEntries(this.entries, this.comparator);
            return new ImmutableSortedMap<>(ImmutableList.copyOf(this.entries), this.comparator);
        }
    }

    ImmutableSortedMap(ImmutableList<Map.Entry<K, V>> entries2, Comparator<? super K> comparator2) {
        this.entries = entries2;
        this.comparator = comparator2;
    }

    public int size() {
        return this.entries.size();
    }

    /* access modifiers changed from: package-private */
    public Comparator<Object> unsafeComparator() {
        return this.comparator;
    }

    public V get(@Nullable Object key) {
        if (key == null) {
            return null;
        }
        try {
            int i = index(key, SortedLists.KeyPresentBehavior.ANY_PRESENT, SortedLists.KeyAbsentBehavior.INVERTED_INSERTION_INDEX);
            if (i >= 0) {
                return ((Map.Entry) this.entries.get(i)).getValue();
            }
            return null;
        } catch (ClassCastException e) {
            return null;
        }
    }

    public boolean containsValue(@Nullable Object value) {
        return value != null && Maps.containsValueImpl(this, value);
    }

    /* access modifiers changed from: package-private */
    public boolean isPartialView() {
        return this.entries.isPartialView();
    }

    public ImmutableSet<Map.Entry<K, V>> entrySet() {
        return super.entrySet();
    }

    /* access modifiers changed from: package-private */
    public ImmutableSet<Map.Entry<K, V>> createEntrySet() {
        return isEmpty() ? ImmutableSet.of() : new EntrySet();
    }

    private class EntrySet extends ImmutableMap.EntrySet {
        private EntrySet() {
            super();
        }

        public UnmodifiableIterator<Map.Entry<K, V>> iterator() {
            return ImmutableSortedMap.this.entries.iterator();
        }

        /* access modifiers changed from: package-private */
        public ImmutableList<Map.Entry<K, V>> createAsList() {
            return ImmutableSortedMap.this.entries;
        }
    }

    public ImmutableSortedSet<K> keySet() {
        return (ImmutableSortedSet) super.keySet();
    }

    /* access modifiers changed from: package-private */
    public ImmutableSortedSet<K> createKeySet() {
        if (isEmpty()) {
            return ImmutableSortedSet.emptySet(this.comparator);
        }
        return new RegularImmutableSortedSet(new TransformedImmutableList<Map.Entry<K, V>, K>(this.entries) {
            /* access modifiers changed from: package-private */
            public K transform(Map.Entry<K, V> entry) {
                return entry.getKey();
            }
        }, this.comparator);
    }

    public ImmutableCollection<V> values() {
        return super.values();
    }

    public Comparator<? super K> comparator() {
        return this.comparator;
    }

    public K firstKey() {
        if (!isEmpty()) {
            return ((Map.Entry) this.entries.get(0)).getKey();
        }
        throw new NoSuchElementException();
    }

    public K lastKey() {
        if (!isEmpty()) {
            return ((Map.Entry) this.entries.get(size() - 1)).getKey();
        }
        throw new NoSuchElementException();
    }

    public ImmutableSortedMap<K, V> headMap(K toKey) {
        return headMap(toKey, false);
    }

    public ImmutableSortedMap<K, V> headMap(K toKey, boolean inclusive) {
        int index;
        if (inclusive) {
            index = index(toKey, SortedLists.KeyPresentBehavior.ANY_PRESENT, SortedLists.KeyAbsentBehavior.NEXT_LOWER) + 1;
        } else {
            index = index(toKey, SortedLists.KeyPresentBehavior.ANY_PRESENT, SortedLists.KeyAbsentBehavior.NEXT_HIGHER);
        }
        return createSubmap(0, index);
    }

    public ImmutableSortedMap<K, V> subMap(K fromKey, K toKey) {
        return subMap(fromKey, true, toKey, false);
    }

    public ImmutableSortedMap<K, V> subMap(K fromKey, boolean fromInclusive, K toKey, boolean toInclusive) {
        Preconditions.checkNotNull(fromKey);
        Preconditions.checkNotNull(toKey);
        Preconditions.checkArgument(this.comparator.compare(fromKey, toKey) <= 0);
        return tailMap(fromKey, fromInclusive).headMap(toKey, toInclusive);
    }

    public ImmutableSortedMap<K, V> tailMap(K fromKey) {
        return tailMap(fromKey, true);
    }

    public ImmutableSortedMap<K, V> tailMap(K fromKey, boolean inclusive) {
        int index;
        if (inclusive) {
            index = index(fromKey, SortedLists.KeyPresentBehavior.ANY_PRESENT, SortedLists.KeyAbsentBehavior.NEXT_HIGHER);
        } else {
            index = index(fromKey, SortedLists.KeyPresentBehavior.ANY_PRESENT, SortedLists.KeyAbsentBehavior.NEXT_LOWER) + 1;
        }
        return createSubmap(index, size());
    }

    public Map.Entry<K, V> lowerEntry(K key) {
        return headMap(key, false).lastEntry();
    }

    public K lowerKey(K key) {
        return Maps.keyOrNull(lowerEntry(key));
    }

    public Map.Entry<K, V> floorEntry(K key) {
        return headMap(key, true).lastEntry();
    }

    public K floorKey(K key) {
        return Maps.keyOrNull(floorEntry(key));
    }

    public Map.Entry<K, V> ceilingEntry(K key) {
        return tailMap(key, true).firstEntry();
    }

    public K ceilingKey(K key) {
        return Maps.keyOrNull(ceilingEntry(key));
    }

    public Map.Entry<K, V> higherEntry(K key) {
        return tailMap(key, false).firstEntry();
    }

    public K higherKey(K key) {
        return Maps.keyOrNull(higherEntry(key));
    }

    public Map.Entry<K, V> firstEntry() {
        if (isEmpty()) {
            return null;
        }
        return (Map.Entry) this.entries.get(0);
    }

    public Map.Entry<K, V> lastEntry() {
        if (isEmpty()) {
            return null;
        }
        return (Map.Entry) this.entries.get(this.entries.size() - 1);
    }

    public final Map.Entry<K, V> pollFirstEntry() {
        throw new UnsupportedOperationException();
    }

    public final Map.Entry<K, V> pollLastEntry() {
        throw new UnsupportedOperationException();
    }

    public ImmutableSortedMap<K, V> descendingMap() {
        ImmutableSortedMap<K, V> result = this.descendingMap;
        if (result != null) {
            return result;
        }
        ImmutableSortedMap<K, V> immutableSortedMap = new ImmutableSortedMap<>(this.entries.reverse(), Ordering.from(comparator()).reverse());
        this.descendingMap = immutableSortedMap;
        ImmutableSortedMap<K, V> result2 = immutableSortedMap;
        result2.descendingMap = this;
        return result2;
    }

    public ImmutableSortedSet<K> navigableKeySet() {
        return keySet();
    }

    public ImmutableSortedSet<K> descendingKeySet() {
        return descendingMap().keySet();
    }

    private ImmutableList<K> keyList() {
        return new TransformedImmutableList<Map.Entry<K, V>, K>(this.entries) {
            /* access modifiers changed from: package-private */
            public K transform(Map.Entry<K, V> entry) {
                return entry.getKey();
            }
        };
    }

    private int index(Object key, SortedLists.KeyPresentBehavior presentBehavior, SortedLists.KeyAbsentBehavior absentBehavior) {
        return SortedLists.binarySearch(keyList(), Preconditions.checkNotNull(key), unsafeComparator(), presentBehavior, absentBehavior);
    }

    private ImmutableSortedMap<K, V> createSubmap(int newFromIndex, int newToIndex) {
        if (newFromIndex < newToIndex) {
            return new ImmutableSortedMap<>(this.entries.subList(newFromIndex, newToIndex), this.comparator);
        }
        return emptyMap(this.comparator);
    }

    private static class SerializedForm extends ImmutableMap.SerializedForm {
        private static final long serialVersionUID = 0;
        private final Comparator<Object> comparator;

        SerializedForm(ImmutableSortedMap<?, ?> sortedMap) {
            super(sortedMap);
            this.comparator = sortedMap.comparator();
        }

        /* access modifiers changed from: package-private */
        public Object readResolve() {
            return createMap(new Builder<>(this.comparator));
        }
    }

    /* access modifiers changed from: package-private */
    public Object writeReplace() {
        return new SerializedForm(this);
    }
}
