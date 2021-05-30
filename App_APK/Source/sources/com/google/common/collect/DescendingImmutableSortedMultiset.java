package com.google.common.collect;

import com.google.common.collect.ImmutableMultiset;
import com.google.common.collect.Multiset;
import javax.annotation.Nullable;

final class DescendingImmutableSortedMultiset<E> extends ImmutableSortedMultiset<E> {
    private final transient ImmutableSortedMultiset<E> forward;

    DescendingImmutableSortedMultiset(ImmutableSortedMultiset<E> forward2) {
        super(forward2.reverseComparator());
        this.forward = forward2;
    }

    public int count(@Nullable Object element) {
        return this.forward.count(element);
    }

    public Multiset.Entry<E> firstEntry() {
        return this.forward.lastEntry();
    }

    public Multiset.Entry<E> lastEntry() {
        return this.forward.firstEntry();
    }

    public int size() {
        return this.forward.size();
    }

    /* access modifiers changed from: package-private */
    public ImmutableSortedSet<E> createElementSet() {
        return this.forward.createDescendingElementSet();
    }

    /* access modifiers changed from: package-private */
    public ImmutableSortedSet<E> createDescendingElementSet() {
        return this.forward.elementSet();
    }

    /* access modifiers changed from: package-private */
    public ImmutableSet<Multiset.Entry<E>> createEntrySet() {
        final ImmutableSet<Multiset.Entry<E>> forwardEntrySet = this.forward.entrySet();
        return new ImmutableMultiset.EntrySet() {
            public int size() {
                return forwardEntrySet.size();
            }

            public UnmodifiableIterator<Multiset.Entry<E>> iterator() {
                return asList().iterator();
            }

            /* access modifiers changed from: package-private */
            public ImmutableList<Multiset.Entry<E>> createAsList() {
                return forwardEntrySet.asList().reverse();
            }
        };
    }

    public ImmutableSortedMultiset<E> descendingMultiset() {
        return this.forward;
    }

    public ImmutableSortedMultiset<E> headMultiset(E upperBound, BoundType boundType) {
        return this.forward.tailMultiset(upperBound, boundType).descendingMultiset();
    }

    public ImmutableSortedMultiset<E> tailMultiset(E lowerBound, BoundType boundType) {
        return this.forward.headMultiset(lowerBound, boundType).descendingMultiset();
    }

    /* access modifiers changed from: package-private */
    public boolean isPartialView() {
        return this.forward.isPartialView();
    }
}
