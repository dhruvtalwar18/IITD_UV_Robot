package com.google.common.collect;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import com.google.common.annotations.GwtIncompatible;
import com.google.common.base.Function;
import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableMap;
import com.google.common.collect.ImmutableMultimap;
import java.io.IOException;
import java.io.InvalidObjectException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import javax.annotation.Nullable;

@GwtCompatible(emulated = true, serializable = true)
public class ImmutableSetMultimap<K, V> extends ImmutableMultimap<K, V> implements SetMultimap<K, V> {
    @GwtIncompatible("not needed in emulated source.")
    private static final long serialVersionUID = 0;
    private final transient ImmutableSortedSet<V> emptySet;
    private transient ImmutableSet<Map.Entry<K, V>> entries;
    private transient ImmutableSetMultimap<V, K> inverse;

    public static <K, V> ImmutableSetMultimap<K, V> of() {
        return EmptyImmutableSetMultimap.INSTANCE;
    }

    public static <K, V> ImmutableSetMultimap<K, V> of(K k1, V v1) {
        Builder<K, V> builder = builder();
        builder.put((Object) k1, (Object) v1);
        return builder.build();
    }

    public static <K, V> ImmutableSetMultimap<K, V> of(K k1, V v1, K k2, V v2) {
        Builder<K, V> builder = builder();
        builder.put((Object) k1, (Object) v1);
        builder.put((Object) k2, (Object) v2);
        return builder.build();
    }

    public static <K, V> ImmutableSetMultimap<K, V> of(K k1, V v1, K k2, V v2, K k3, V v3) {
        Builder<K, V> builder = builder();
        builder.put((Object) k1, (Object) v1);
        builder.put((Object) k2, (Object) v2);
        builder.put((Object) k3, (Object) v3);
        return builder.build();
    }

    public static <K, V> ImmutableSetMultimap<K, V> of(K k1, V v1, K k2, V v2, K k3, V v3, K k4, V v4) {
        Builder<K, V> builder = builder();
        builder.put((Object) k1, (Object) v1);
        builder.put((Object) k2, (Object) v2);
        builder.put((Object) k3, (Object) v3);
        builder.put((Object) k4, (Object) v4);
        return builder.build();
    }

    public static <K, V> ImmutableSetMultimap<K, V> of(K k1, V v1, K k2, V v2, K k3, V v3, K k4, V v4, K k5, V v5) {
        Builder<K, V> builder = builder();
        builder.put((Object) k1, (Object) v1);
        builder.put((Object) k2, (Object) v2);
        builder.put((Object) k3, (Object) v3);
        builder.put((Object) k4, (Object) v4);
        builder.put((Object) k5, (Object) v5);
        return builder.build();
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
            return Sets.newLinkedHashSet();
        }
    }

    private static class SortedKeyBuilderMultimap<K, V> extends AbstractMultimap<K, V> {
        private static final long serialVersionUID = 0;

        SortedKeyBuilderMultimap(Comparator<? super K> keyComparator, Multimap<K, V> multimap) {
            super(new TreeMap(keyComparator));
            putAll(multimap);
        }

        /* access modifiers changed from: package-private */
        public Collection<V> createCollection() {
            return Sets.newLinkedHashSet();
        }
    }

    public static final class Builder<K, V> extends ImmutableMultimap.Builder<K, V> {
        public Builder() {
            this.builderMultimap = new BuilderMultimap();
        }

        public Builder<K, V> put(K key, V value) {
            this.builderMultimap.put(Preconditions.checkNotNull(key), Preconditions.checkNotNull(value));
            return this;
        }

        public Builder<K, V> put(Map.Entry<? extends K, ? extends V> entry) {
            this.builderMultimap.put(Preconditions.checkNotNull(entry.getKey()), Preconditions.checkNotNull(entry.getValue()));
            return this;
        }

        public Builder<K, V> putAll(K key, Iterable<? extends V> values) {
            Collection<V> collection = this.builderMultimap.get(Preconditions.checkNotNull(key));
            for (V value : values) {
                collection.add(Preconditions.checkNotNull(value));
            }
            return this;
        }

        public Builder<K, V> putAll(K key, V... values) {
            return putAll((Object) key, (Iterable) Arrays.asList(values));
        }

        public Builder<K, V> putAll(Multimap<? extends K, ? extends V> multimap) {
            for (Map.Entry<? extends K, ? extends Collection<? extends V>> entry : multimap.asMap().entrySet()) {
                putAll((Object) entry.getKey(), (Iterable) entry.getValue());
            }
            return this;
        }

        @Beta
        public Builder<K, V> orderKeysBy(Comparator<? super K> keyComparator) {
            this.keyComparator = (Comparator) Preconditions.checkNotNull(keyComparator);
            return this;
        }

        @Beta
        public Builder<K, V> orderValuesBy(Comparator<? super V> valueComparator) {
            super.orderValuesBy(valueComparator);
            return this;
        }

        public ImmutableSetMultimap<K, V> build() {
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
            return ImmutableSetMultimap.copyOf(this.builderMultimap, this.valueComparator);
        }
    }

    public static <K, V> ImmutableSetMultimap<K, V> copyOf(Multimap<? extends K, ? extends V> multimap) {
        return copyOf(multimap, (Comparator) null);
    }

    /* JADX WARNING: type inference failed for: r9v0, types: [java.util.Comparator<? super V>, java.util.Comparator] */
    /* access modifiers changed from: private */
    /* JADX WARNING: Unknown variable types count: 1 */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public static <K, V> com.google.common.collect.ImmutableSetMultimap<K, V> copyOf(com.google.common.collect.Multimap<? extends K, ? extends V> r8, java.util.Comparator<? super V> r9) {
        /*
            com.google.common.base.Preconditions.checkNotNull(r8)
            boolean r0 = r8.isEmpty()
            if (r0 == 0) goto L_0x0010
            if (r9 != 0) goto L_0x0010
            com.google.common.collect.ImmutableSetMultimap r0 = of()
            return r0
        L_0x0010:
            boolean r0 = r8 instanceof com.google.common.collect.ImmutableSetMultimap
            if (r0 == 0) goto L_0x001e
            r0 = r8
            com.google.common.collect.ImmutableSetMultimap r0 = (com.google.common.collect.ImmutableSetMultimap) r0
            boolean r1 = r0.isPartialView()
            if (r1 != 0) goto L_0x001e
            return r0
        L_0x001e:
            com.google.common.collect.ImmutableMap$Builder r0 = com.google.common.collect.ImmutableMap.builder()
            r1 = 0
            java.util.Map r2 = r8.asMap()
            java.util.Set r2 = r2.entrySet()
            java.util.Iterator r2 = r2.iterator()
        L_0x002f:
            boolean r3 = r2.hasNext()
            if (r3 == 0) goto L_0x005f
            java.lang.Object r3 = r2.next()
            java.util.Map$Entry r3 = (java.util.Map.Entry) r3
            java.lang.Object r4 = r3.getKey()
            java.lang.Object r5 = r3.getValue()
            java.util.Collection r5 = (java.util.Collection) r5
            if (r9 != 0) goto L_0x004c
            com.google.common.collect.ImmutableSet r6 = com.google.common.collect.ImmutableSet.copyOf(r5)
            goto L_0x0050
        L_0x004c:
            com.google.common.collect.ImmutableSortedSet r6 = com.google.common.collect.ImmutableSortedSet.copyOf(r9, r5)
        L_0x0050:
            boolean r7 = r6.isEmpty()
            if (r7 != 0) goto L_0x005e
            r0.put(r4, r6)
            int r7 = r6.size()
            int r1 = r1 + r7
        L_0x005e:
            goto L_0x002f
        L_0x005f:
            com.google.common.collect.ImmutableSetMultimap r2 = new com.google.common.collect.ImmutableSetMultimap
            com.google.common.collect.ImmutableMap r3 = r0.build()
            r2.<init>(r3, r1, r9)
            return r2
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.common.collect.ImmutableSetMultimap.copyOf(com.google.common.collect.Multimap, java.util.Comparator):com.google.common.collect.ImmutableSetMultimap");
    }

    ImmutableSetMultimap(ImmutableMap<K, ImmutableSet<V>> map, int size, @Nullable Comparator<? super V> valueComparator) {
        super(map, size);
        this.emptySet = valueComparator == null ? null : ImmutableSortedSet.emptySet(valueComparator);
    }

    public ImmutableSet<V> get(@Nullable K key) {
        ImmutableSet<V> set = (ImmutableSet) this.map.get(key);
        if (set != null) {
            return set;
        }
        if (this.emptySet != null) {
            return this.emptySet;
        }
        return ImmutableSet.of();
    }

    @Beta
    public ImmutableSetMultimap<V, K> inverse() {
        ImmutableSetMultimap<V, K> result = this.inverse;
        if (result != null) {
            return result;
        }
        ImmutableSetMultimap<V, K> invert = invert();
        this.inverse = invert;
        return invert;
    }

    private ImmutableSetMultimap<V, K> invert() {
        Builder<V, K> builder = builder();
        Iterator i$ = entries().iterator();
        while (i$.hasNext()) {
            Map.Entry<K, V> entry = (Map.Entry) i$.next();
            builder.put((Object) entry.getValue(), (Object) entry.getKey());
        }
        ImmutableSetMultimap<V, K> invertedMultimap = builder.build();
        invertedMultimap.inverse = this;
        return invertedMultimap;
    }

    public ImmutableSet<V> removeAll(Object key) {
        throw new UnsupportedOperationException();
    }

    public ImmutableSet<V> replaceValues(K k, Iterable<? extends V> iterable) {
        throw new UnsupportedOperationException();
    }

    public ImmutableSet<Map.Entry<K, V>> entries() {
        ImmutableSet<Map.Entry<K, V>> result = this.entries;
        if (result != null) {
            return result;
        }
        ImmutableSet<Map.Entry<K, V>> copyOf = ImmutableSet.copyOf(super.entries());
        this.entries = copyOf;
        return copyOf;
    }

    @GwtIncompatible("java.io.ObjectOutputStream")
    private void writeObject(ObjectOutputStream stream) throws IOException {
        stream.defaultWriteObject();
        Serialization.writeMultimap(this, stream);
    }

    @GwtIncompatible("java.io.ObjectInputStream")
    private void readObject(ObjectInputStream stream) throws IOException, ClassNotFoundException {
        stream.defaultReadObject();
        int keyCount = stream.readInt();
        if (keyCount >= 0) {
            ImmutableMap.Builder<Object, ImmutableSet<Object>> builder = ImmutableMap.builder();
            int tmpSize = 0;
            int i = 0;
            while (i < keyCount) {
                Object key = stream.readObject();
                int valueCount = stream.readInt();
                if (valueCount > 0) {
                    Object[] array = new Object[valueCount];
                    for (int j = 0; j < valueCount; j++) {
                        array[j] = stream.readObject();
                    }
                    ImmutableSet<Object> valueSet = ImmutableSet.copyOf((E[]) array);
                    if (valueSet.size() == array.length) {
                        builder.put(key, valueSet);
                        tmpSize += valueCount;
                        i++;
                    } else {
                        throw new InvalidObjectException("Duplicate key-value pairs exist for key " + key);
                    }
                } else {
                    throw new InvalidObjectException("Invalid value count " + valueCount);
                }
            }
            try {
                ImmutableMultimap.FieldSettersHolder.MAP_FIELD_SETTER.set(this, (Object) builder.build());
                ImmutableMultimap.FieldSettersHolder.SIZE_FIELD_SETTER.set(this, tmpSize);
            } catch (IllegalArgumentException e) {
                throw ((InvalidObjectException) new InvalidObjectException(e.getMessage()).initCause(e));
            }
        } else {
            throw new InvalidObjectException("Invalid key count " + keyCount);
        }
    }
}
