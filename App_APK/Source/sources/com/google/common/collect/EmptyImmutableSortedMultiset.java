package com.google.common.collect;

import com.google.common.base.Preconditions;
import com.google.common.collect.Multiset;
import java.util.Comparator;
import javax.annotation.Nullable;

final class EmptyImmutableSortedMultiset<E> extends ImmutableSortedMultiset<E> {
    EmptyImmutableSortedMultiset(Comparator<? super E> comparator) {
        super(comparator);
    }

    public Multiset.Entry<E> firstEntry() {
        return null;
    }

    public Multiset.Entry<E> lastEntry() {
        return null;
    }

    public int count(@Nullable Object element) {
        return 0;
    }

    public int size() {
        return 0;
    }

    /* access modifiers changed from: package-private */
    public ImmutableSortedSet<E> createElementSet() {
        return ImmutableSortedSet.emptySet(comparator());
    }

    /* access modifiers changed from: package-private */
    public ImmutableSortedSet<E> createDescendingElementSet() {
        return ImmutableSortedSet.emptySet(reverseComparator());
    }

    /* access modifiers changed from: package-private */
    public ImmutableSet<Multiset.Entry<E>> createEntrySet() {
        return ImmutableSet.of();
    }

    public ImmutableSortedMultiset<E> headMultiset(E upperBound, BoundType boundType) {
        Preconditions.checkNotNull(upperBound);
        Preconditions.checkNotNull(boundType);
        return this;
    }

    public ImmutableSortedMultiset<E> tailMultiset(E lowerBound, BoundType boundType) {
        Preconditions.checkNotNull(lowerBound);
        Preconditions.checkNotNull(boundType);
        return this;
    }

    /* access modifiers changed from: package-private */
    public boolean isPartialView() {
        return false;
    }
}
