package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.collect.ImmutableMultiset;
import com.google.common.collect.Multiset;
import java.util.Map;
import javax.annotation.Nullable;

@GwtCompatible(serializable = true)
class RegularImmutableMultiset<E> extends ImmutableMultiset<E> {
    /* access modifiers changed from: private */
    public final transient ImmutableMap<E, Integer> map;
    private final transient int size;

    RegularImmutableMultiset(ImmutableMap<E, Integer> map2, int size2) {
        this.map = map2;
        this.size = size2;
    }

    /* access modifiers changed from: package-private */
    public boolean isPartialView() {
        return this.map.isPartialView();
    }

    public int count(@Nullable Object element) {
        Integer value = this.map.get(element);
        if (value == null) {
            return 0;
        }
        return value.intValue();
    }

    public int size() {
        return this.size;
    }

    public boolean contains(@Nullable Object element) {
        return this.map.containsKey(element);
    }

    public ImmutableSet<E> elementSet() {
        return this.map.keySet();
    }

    /* access modifiers changed from: private */
    public static <E> Multiset.Entry<E> entryFromMapEntry(Map.Entry<E, Integer> entry) {
        return Multisets.immutableEntry(entry.getKey(), entry.getValue().intValue());
    }

    /* access modifiers changed from: package-private */
    public ImmutableSet<Multiset.Entry<E>> createEntrySet() {
        return new ImmutableMultiset.EntrySet() {
            public int size() {
                return RegularImmutableMultiset.this.map.size();
            }

            public UnmodifiableIterator<Multiset.Entry<E>> iterator() {
                return asList().iterator();
            }

            /* access modifiers changed from: package-private */
            public ImmutableList<Multiset.Entry<E>> createAsList() {
                return new TransformedImmutableList<Map.Entry<E, Integer>, Multiset.Entry<E>>(RegularImmutableMultiset.this.map.entrySet().asList()) {
                    /* access modifiers changed from: package-private */
                    public Multiset.Entry<E> transform(Map.Entry<E, Integer> entry) {
                        return RegularImmutableMultiset.entryFromMapEntry(entry);
                    }
                };
            }
        };
    }

    public int hashCode() {
        return this.map.hashCode();
    }
}
