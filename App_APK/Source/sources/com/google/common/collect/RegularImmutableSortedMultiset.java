package com.google.common.collect;

import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableMultiset;
import com.google.common.collect.Multiset;
import com.google.common.collect.Multisets;
import com.google.common.collect.SortedLists;
import com.google.common.primitives.Ints;
import java.util.Comparator;
import java.util.List;
import javax.annotation.Nullable;

final class RegularImmutableSortedMultiset<E> extends ImmutableSortedMultiset<E> {
    static final /* synthetic */ boolean $assertionsDisabled = false;
    final transient ImmutableList<CumulativeCountEntry<E>> entries;

    private static final class CumulativeCountEntry<E> extends Multisets.AbstractEntry<E> {
        final int count;
        final long cumulativeCount;
        final E element;

        CumulativeCountEntry(E element2, int count2, @Nullable CumulativeCountEntry<E> previous) {
            this.element = element2;
            this.count = count2;
            this.cumulativeCount = ((long) count2) + (previous == null ? 0 : previous.cumulativeCount);
        }

        public E getElement() {
            return this.element;
        }

        public int getCount() {
            return this.count;
        }
    }

    static <E> RegularImmutableSortedMultiset<E> createFromSorted(Comparator<? super E> comparator, List<? extends Multiset.Entry<E>> entries2) {
        List<CumulativeCountEntry<E>> newEntries = Lists.newArrayListWithCapacity(entries2.size());
        CumulativeCountEntry<E> previous = null;
        for (Multiset.Entry<E> entry : entries2) {
            CumulativeCountEntry<E> cumulativeCountEntry = new CumulativeCountEntry<>(entry.getElement(), entry.getCount(), previous);
            previous = cumulativeCountEntry;
            newEntries.add(cumulativeCountEntry);
        }
        return new RegularImmutableSortedMultiset<>(comparator, ImmutableList.copyOf(newEntries));
    }

    RegularImmutableSortedMultiset(Comparator<? super E> comparator, ImmutableList<CumulativeCountEntry<E>> entries2) {
        super(comparator);
        this.entries = entries2;
    }

    /* access modifiers changed from: package-private */
    public ImmutableList<E> elementList() {
        return new TransformedImmutableList<CumulativeCountEntry<E>, E>(this.entries) {
            /* access modifiers changed from: package-private */
            public E transform(CumulativeCountEntry<E> entry) {
                return entry.getElement();
            }
        };
    }

    /* access modifiers changed from: package-private */
    public ImmutableSortedSet<E> createElementSet() {
        return new RegularImmutableSortedSet(elementList(), comparator());
    }

    /* access modifiers changed from: package-private */
    public ImmutableSortedSet<E> createDescendingElementSet() {
        return new RegularImmutableSortedSet(elementList().reverse(), reverseComparator());
    }

    /* access modifiers changed from: package-private */
    public ImmutableSet<Multiset.Entry<E>> createEntrySet() {
        return new ImmutableMultiset.EntrySet() {
            public int size() {
                return RegularImmutableSortedMultiset.this.entries.size();
            }

            public UnmodifiableIterator<Multiset.Entry<E>> iterator() {
                return asList().iterator();
            }

            /* access modifiers changed from: package-private */
            public ImmutableList<Multiset.Entry<E>> createAsList() {
                return RegularImmutableSortedMultiset.this.entries;
            }
        };
    }

    public CumulativeCountEntry<E> firstEntry() {
        return (CumulativeCountEntry) this.entries.get(0);
    }

    public CumulativeCountEntry<E> lastEntry() {
        return (CumulativeCountEntry) this.entries.get(this.entries.size() - 1);
    }

    public int size() {
        CumulativeCountEntry<E> firstEntry = firstEntry();
        return Ints.saturatedCast((lastEntry().cumulativeCount - firstEntry.cumulativeCount) + ((long) firstEntry.count));
    }

    /* access modifiers changed from: package-private */
    public boolean isPartialView() {
        return this.entries.isPartialView();
    }

    public int count(@Nullable Object element) {
        if (element == null) {
            return 0;
        }
        try {
            int index = SortedLists.binarySearch(elementList(), element, comparator(), SortedLists.KeyPresentBehavior.ANY_PRESENT, SortedLists.KeyAbsentBehavior.INVERTED_INSERTION_INDEX);
            if (index >= 0) {
                return ((CumulativeCountEntry) this.entries.get(index)).getCount();
            }
            return 0;
        } catch (ClassCastException e) {
            return 0;
        }
    }

    public ImmutableSortedMultiset<E> headMultiset(E upperBound, BoundType boundType) {
        int index;
        switch (boundType) {
            case OPEN:
                index = SortedLists.binarySearch(elementList(), Preconditions.checkNotNull(upperBound), comparator(), SortedLists.KeyPresentBehavior.ANY_PRESENT, SortedLists.KeyAbsentBehavior.NEXT_HIGHER);
                break;
            case CLOSED:
                index = SortedLists.binarySearch(elementList(), Preconditions.checkNotNull(upperBound), comparator(), SortedLists.KeyPresentBehavior.ANY_PRESENT, SortedLists.KeyAbsentBehavior.NEXT_LOWER) + 1;
                break;
            default:
                throw new AssertionError();
        }
        return createSubMultiset(0, index);
    }

    public ImmutableSortedMultiset<E> tailMultiset(E lowerBound, BoundType boundType) {
        int index;
        switch (boundType) {
            case OPEN:
                index = SortedLists.binarySearch(elementList(), Preconditions.checkNotNull(lowerBound), comparator(), SortedLists.KeyPresentBehavior.ANY_PRESENT, SortedLists.KeyAbsentBehavior.NEXT_LOWER) + 1;
                break;
            case CLOSED:
                index = SortedLists.binarySearch(elementList(), Preconditions.checkNotNull(lowerBound), comparator(), SortedLists.KeyPresentBehavior.ANY_PRESENT, SortedLists.KeyAbsentBehavior.NEXT_HIGHER);
                break;
            default:
                throw new AssertionError();
        }
        return createSubMultiset(index, this.entries.size());
    }

    private ImmutableSortedMultiset<E> createSubMultiset(int newFromIndex, int newToIndex) {
        if (newFromIndex == 0 && newToIndex == this.entries.size()) {
            return this;
        }
        if (newFromIndex >= newToIndex) {
            return emptyMultiset(comparator());
        }
        return new RegularImmutableSortedMultiset(comparator(), this.entries.subList(newFromIndex, newToIndex));
    }
}
