package com.google.common.collect;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtIncompatible;
import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableMultiset;
import com.google.common.collect.Multiset;
import java.io.Serializable;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;

@GwtIncompatible("hasn't been tested yet")
@Beta
public abstract class ImmutableSortedMultiset<E> extends ImmutableSortedMultisetFauxverideShim<E> implements SortedMultiset<E> {
    private static final ImmutableSortedMultiset<Comparable> NATURAL_EMPTY_MULTISET = new EmptyImmutableSortedMultiset(NATURAL_ORDER);
    private static final Comparator<Comparable> NATURAL_ORDER = Ordering.natural();
    private final transient Comparator<? super E> comparator;
    transient ImmutableSortedMultiset<E> descendingMultiset;
    private transient ImmutableSortedSet<E> elementSet;
    private transient Comparator<? super E> reverseComparator;

    /* access modifiers changed from: package-private */
    public abstract ImmutableSortedSet<E> createDescendingElementSet();

    /* access modifiers changed from: package-private */
    public abstract ImmutableSortedSet<E> createElementSet();

    public abstract ImmutableSortedMultiset<E> headMultiset(E e, BoundType boundType);

    public abstract ImmutableSortedMultiset<E> tailMultiset(E e, BoundType boundType);

    public static <E> ImmutableSortedMultiset<E> of() {
        return NATURAL_EMPTY_MULTISET;
    }

    public static <E extends Comparable<? super E>> ImmutableSortedMultiset<E> of(E element) {
        return RegularImmutableSortedMultiset.createFromSorted(NATURAL_ORDER, ImmutableList.of(Multisets.immutableEntry(Preconditions.checkNotNull(element), 1)));
    }

    public static <E extends Comparable<? super E>> ImmutableSortedMultiset<E> of(E e1, E e2) {
        return copyOf(Ordering.natural(), Arrays.asList(new Comparable[]{e1, e2}));
    }

    public static <E extends Comparable<? super E>> ImmutableSortedMultiset<E> of(E e1, E e2, E e3) {
        return copyOf(Ordering.natural(), Arrays.asList(new Comparable[]{e1, e2, e3}));
    }

    public static <E extends Comparable<? super E>> ImmutableSortedMultiset<E> of(E e1, E e2, E e3, E e4) {
        return copyOf(Ordering.natural(), Arrays.asList(new Comparable[]{e1, e2, e3, e4}));
    }

    public static <E extends Comparable<? super E>> ImmutableSortedMultiset<E> of(E e1, E e2, E e3, E e4, E e5) {
        return copyOf(Ordering.natural(), Arrays.asList(new Comparable[]{e1, e2, e3, e4, e5}));
    }

    public static <E extends Comparable<? super E>> ImmutableSortedMultiset<E> of(E e1, E e2, E e3, E e4, E e5, E e6, E... remaining) {
        List<E> all = Lists.newArrayListWithCapacity(remaining.length + 6);
        Collections.addAll(all, new Comparable[]{e1, e2, e3, e4, e5, e6});
        Collections.addAll(all, remaining);
        return copyOf(Ordering.natural(), all);
    }

    public static <E extends Comparable<? super E>> ImmutableSortedMultiset<E> copyOf(E[] elements) {
        return copyOf(Ordering.natural(), Arrays.asList(elements));
    }

    public static <E> ImmutableSortedMultiset<E> copyOf(Iterable<? extends E> elements) {
        return copyOf(Ordering.natural(), elements);
    }

    public static <E> ImmutableSortedMultiset<E> copyOf(Iterator<? extends E> elements) {
        return copyOfInternal(Ordering.natural(), elements);
    }

    public static <E> ImmutableSortedMultiset<E> copyOf(Comparator<? super E> comparator2, Iterator<? extends E> elements) {
        Preconditions.checkNotNull(comparator2);
        return copyOfInternal(comparator2, elements);
    }

    public static <E> ImmutableSortedMultiset<E> copyOf(Comparator<? super E> comparator2, Iterable<? extends E> elements) {
        Preconditions.checkNotNull(comparator2);
        return copyOfInternal(comparator2, elements);
    }

    /* JADX WARNING: type inference failed for: r2v0, types: [com.google.common.collect.SortedMultiset, com.google.common.collect.SortedMultiset<E>, java.lang.Iterable] */
    /* JADX WARNING: Unknown variable types count: 1 */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public static <E> com.google.common.collect.ImmutableSortedMultiset<E> copyOfSorted(com.google.common.collect.SortedMultiset<E> r2) {
        /*
            java.util.Comparator r0 = r2.comparator()
            if (r0 != 0) goto L_0x0008
            java.util.Comparator<java.lang.Comparable> r0 = NATURAL_ORDER
        L_0x0008:
            com.google.common.collect.ImmutableSortedMultiset r1 = copyOfInternal(r0, r2)
            return r1
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.common.collect.ImmutableSortedMultiset.copyOfSorted(com.google.common.collect.SortedMultiset):com.google.common.collect.ImmutableSortedMultiset");
    }

    private static <E> ImmutableSortedMultiset<E> copyOfInternal(Comparator<? super E> comparator2, Iterable<? extends E> iterable) {
        if (SortedIterables.hasSameComparator(comparator2, iterable) && (iterable instanceof ImmutableSortedMultiset) && !((ImmutableSortedMultiset) iterable).isPartialView()) {
            return (ImmutableSortedMultiset) iterable;
        }
        ImmutableList<Multiset.Entry<E>> entries = ImmutableList.copyOf(SortedIterables.sortedCounts(comparator2, iterable));
        if (entries.isEmpty()) {
            return emptyMultiset(comparator2);
        }
        verifyEntries(entries);
        return RegularImmutableSortedMultiset.createFromSorted(comparator2, entries);
    }

    private static <E> ImmutableSortedMultiset<E> copyOfInternal(Comparator<? super E> comparator2, Iterator<? extends E> iterator) {
        ImmutableList<Multiset.Entry<E>> entries = ImmutableList.copyOf(SortedIterables.sortedCounts(comparator2, iterator));
        if (entries.isEmpty()) {
            return emptyMultiset(comparator2);
        }
        verifyEntries(entries);
        return RegularImmutableSortedMultiset.createFromSorted(comparator2, entries);
    }

    private static <E> void verifyEntries(Collection<Multiset.Entry<E>> entries) {
        for (Multiset.Entry<E> entry : entries) {
            Preconditions.checkNotNull(entry.getElement());
        }
    }

    static <E> ImmutableSortedMultiset<E> emptyMultiset(Comparator<? super E> comparator2) {
        if (NATURAL_ORDER.equals(comparator2)) {
            return NATURAL_EMPTY_MULTISET;
        }
        return new EmptyImmutableSortedMultiset(comparator2);
    }

    ImmutableSortedMultiset(Comparator<? super E> comparator2) {
        this.comparator = (Comparator) Preconditions.checkNotNull(comparator2);
    }

    public Comparator<? super E> comparator() {
        return this.comparator;
    }

    /* access modifiers changed from: package-private */
    public Comparator<Object> unsafeComparator() {
        return this.comparator;
    }

    /* access modifiers changed from: package-private */
    public Comparator<? super E> reverseComparator() {
        Comparator<? super E> result = this.reverseComparator;
        if (result != null) {
            return result;
        }
        Ordering<S> reverse = Ordering.from(this.comparator).reverse();
        this.reverseComparator = reverse;
        return reverse;
    }

    public ImmutableSortedSet<E> elementSet() {
        ImmutableSortedSet<E> result = this.elementSet;
        if (result != null) {
            return result;
        }
        ImmutableSortedSet<E> createElementSet = createElementSet();
        this.elementSet = createElementSet;
        return createElementSet;
    }

    public ImmutableSortedMultiset<E> descendingMultiset() {
        ImmutableSortedMultiset<E> result = this.descendingMultiset;
        if (result != null) {
            return result;
        }
        DescendingImmutableSortedMultiset descendingImmutableSortedMultiset = new DescendingImmutableSortedMultiset(this);
        this.descendingMultiset = descendingImmutableSortedMultiset;
        return descendingImmutableSortedMultiset;
    }

    public final Multiset.Entry<E> pollFirstEntry() {
        throw new UnsupportedOperationException();
    }

    public Multiset.Entry<E> pollLastEntry() {
        throw new UnsupportedOperationException();
    }

    public ImmutableSortedMultiset<E> subMultiset(E lowerBound, BoundType lowerBoundType, E upperBound, BoundType upperBoundType) {
        return tailMultiset(lowerBound, lowerBoundType).headMultiset(upperBound, upperBoundType);
    }

    public static <E> Builder<E> orderedBy(Comparator<E> comparator2) {
        return new Builder<>(comparator2);
    }

    public static <E extends Comparable<E>> Builder<E> reverseOrder() {
        return new Builder<>(Ordering.natural().reverse());
    }

    public static <E extends Comparable<E>> Builder<E> naturalOrder() {
        return new Builder<>(Ordering.natural());
    }

    public static class Builder<E> extends ImmutableMultiset.Builder<E> {
        private final Comparator<? super E> comparator;

        public Builder(Comparator<? super E> comparator2) {
            super(TreeMultiset.create(comparator2));
            this.comparator = (Comparator) Preconditions.checkNotNull(comparator2);
        }

        public Builder<E> add(E element) {
            super.add((Object) element);
            return this;
        }

        public Builder<E> addCopies(E element, int occurrences) {
            super.addCopies(element, occurrences);
            return this;
        }

        public Builder<E> setCount(E element, int count) {
            super.setCount(element, count);
            return this;
        }

        public Builder<E> add(E... elements) {
            super.add((Object[]) elements);
            return this;
        }

        public Builder<E> addAll(Iterable<? extends E> elements) {
            super.addAll((Iterable) elements);
            return this;
        }

        public Builder<E> addAll(Iterator<? extends E> elements) {
            super.addAll((Iterator) elements);
            return this;
        }

        public ImmutableSortedMultiset<E> build() {
            return ImmutableSortedMultiset.copyOf(this.comparator, this.contents);
        }
    }

    private static final class SerializedForm implements Serializable {
        Comparator comparator;
        int[] counts;
        Object[] elements;

        SerializedForm(SortedMultiset<?> multiset) {
            this.comparator = multiset.comparator();
            int n = multiset.entrySet().size();
            this.elements = new Object[n];
            this.counts = new int[n];
            int i = 0;
            for (Multiset.Entry<?> entry : multiset.entrySet()) {
                this.elements[i] = entry.getElement();
                this.counts[i] = entry.getCount();
                i++;
            }
        }

        /* access modifiers changed from: package-private */
        public Object readResolve() {
            int n = this.elements.length;
            Builder<Object> builder = ImmutableSortedMultiset.orderedBy(this.comparator);
            for (int i = 0; i < n; i++) {
                builder.addCopies(this.elements[i], this.counts[i]);
            }
            return builder.build();
        }
    }

    /* access modifiers changed from: package-private */
    public Object writeReplace() {
        return new SerializedForm(this);
    }
}
