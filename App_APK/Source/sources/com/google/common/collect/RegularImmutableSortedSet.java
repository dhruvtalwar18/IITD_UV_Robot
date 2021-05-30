package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Preconditions;
import com.google.common.collect.SortedLists;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import javax.annotation.Nullable;

@GwtCompatible(emulated = true, serializable = true)
final class RegularImmutableSortedSet<E> extends ImmutableSortedSet<E> {
    private final transient ImmutableList<E> elements;

    RegularImmutableSortedSet(ImmutableList<E> elements2, Comparator<? super E> comparator) {
        super(comparator);
        this.elements = elements2;
        Preconditions.checkArgument(!elements2.isEmpty());
    }

    public UnmodifiableIterator<E> iterator() {
        return this.elements.iterator();
    }

    public boolean isEmpty() {
        return false;
    }

    public int size() {
        return this.elements.size();
    }

    public boolean contains(Object o) {
        if (o == null) {
            return false;
        }
        try {
            if (binarySearch(o) >= 0) {
                return true;
            }
            return false;
        } catch (ClassCastException e) {
            return false;
        }
    }

    public boolean containsAll(Collection<?> targets) {
        if (!SortedIterables.hasSameComparator(comparator(), targets) || targets.size() <= 1) {
            return super.containsAll(targets);
        }
        Iterator<E> thisIterator = iterator();
        Iterator<?> thatIterator = targets.iterator();
        Object target = thatIterator.next();
        while (thisIterator.hasNext()) {
            try {
                int cmp = unsafeCompare(thisIterator.next(), target);
                if (cmp == 0) {
                    if (!thatIterator.hasNext()) {
                        return true;
                    }
                    target = thatIterator.next();
                } else if (cmp > 0) {
                    return false;
                }
            } catch (NullPointerException e) {
                return false;
            } catch (ClassCastException e2) {
                return false;
            }
        }
        return false;
    }

    private int binarySearch(Object key) {
        return Collections.binarySearch(this.elements, key, this.comparator);
    }

    /* access modifiers changed from: package-private */
    public boolean isPartialView() {
        return this.elements.isPartialView();
    }

    public Object[] toArray() {
        return this.elements.toArray();
    }

    public <T> T[] toArray(T[] array) {
        return this.elements.toArray(array);
    }

    /* JADX WARNING: Removed duplicated region for block: B:16:0x002e A[Catch:{ ClassCastException -> 0x0044, NoSuchElementException -> 0x0042 }] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public boolean equals(@javax.annotation.Nullable java.lang.Object r9) {
        /*
            r8 = this;
            r0 = 1
            if (r9 != r8) goto L_0x0004
            return r0
        L_0x0004:
            boolean r1 = r9 instanceof java.util.Set
            r2 = 0
            if (r1 != 0) goto L_0x000a
            return r2
        L_0x000a:
            r1 = r9
            java.util.Set r1 = (java.util.Set) r1
            int r3 = r8.size()
            int r4 = r1.size()
            if (r3 == r4) goto L_0x0018
            return r2
        L_0x0018:
            java.util.Comparator r3 = r8.comparator
            boolean r3 = com.google.common.collect.SortedIterables.hasSameComparator(r3, r1)
            if (r3 == 0) goto L_0x0046
            java.util.Iterator r3 = r1.iterator()
            com.google.common.collect.UnmodifiableIterator r4 = r8.iterator()     // Catch:{ ClassCastException -> 0x0044, NoSuchElementException -> 0x0042 }
        L_0x0028:
            boolean r5 = r4.hasNext()     // Catch:{ ClassCastException -> 0x0044, NoSuchElementException -> 0x0042 }
            if (r5 == 0) goto L_0x0041
            java.lang.Object r5 = r4.next()     // Catch:{ ClassCastException -> 0x0044, NoSuchElementException -> 0x0042 }
            java.lang.Object r6 = r3.next()     // Catch:{ ClassCastException -> 0x0044, NoSuchElementException -> 0x0042 }
            if (r6 == 0) goto L_0x0040
            int r7 = r8.unsafeCompare(r5, r6)     // Catch:{ ClassCastException -> 0x0044, NoSuchElementException -> 0x0042 }
            if (r7 == 0) goto L_0x003f
            goto L_0x0040
        L_0x003f:
            goto L_0x0028
        L_0x0040:
            return r2
        L_0x0041:
            return r0
        L_0x0042:
            r0 = move-exception
            return r2
        L_0x0044:
            r0 = move-exception
            return r2
        L_0x0046:
            boolean r0 = r8.containsAll(r1)
            return r0
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.common.collect.RegularImmutableSortedSet.equals(java.lang.Object):boolean");
    }

    public E first() {
        return this.elements.get(0);
    }

    public E last() {
        return this.elements.get(size() - 1);
    }

    /* access modifiers changed from: package-private */
    public ImmutableSortedSet<E> headSetImpl(E toElement, boolean inclusive) {
        int index;
        if (inclusive) {
            index = SortedLists.binarySearch(this.elements, Preconditions.checkNotNull(toElement), comparator(), SortedLists.KeyPresentBehavior.FIRST_AFTER, SortedLists.KeyAbsentBehavior.NEXT_HIGHER);
        } else {
            index = SortedLists.binarySearch(this.elements, Preconditions.checkNotNull(toElement), comparator(), SortedLists.KeyPresentBehavior.FIRST_PRESENT, SortedLists.KeyAbsentBehavior.NEXT_HIGHER);
        }
        return createSubset(0, index);
    }

    /* access modifiers changed from: package-private */
    public ImmutableSortedSet<E> subSetImpl(E fromElement, boolean fromInclusive, E toElement, boolean toInclusive) {
        return tailSetImpl(fromElement, fromInclusive).headSetImpl(toElement, toInclusive);
    }

    /* access modifiers changed from: package-private */
    public ImmutableSortedSet<E> tailSetImpl(E fromElement, boolean inclusive) {
        int index;
        if (inclusive) {
            index = SortedLists.binarySearch(this.elements, Preconditions.checkNotNull(fromElement), comparator(), SortedLists.KeyPresentBehavior.FIRST_PRESENT, SortedLists.KeyAbsentBehavior.NEXT_HIGHER);
        } else {
            index = SortedLists.binarySearch(this.elements, Preconditions.checkNotNull(fromElement), comparator(), SortedLists.KeyPresentBehavior.FIRST_AFTER, SortedLists.KeyAbsentBehavior.NEXT_HIGHER);
        }
        return createSubset(index, size());
    }

    /* access modifiers changed from: package-private */
    public Comparator<Object> unsafeComparator() {
        return this.comparator;
    }

    private ImmutableSortedSet<E> createSubset(int newFromIndex, int newToIndex) {
        if (newFromIndex == 0 && newToIndex == size()) {
            return this;
        }
        if (newFromIndex < newToIndex) {
            return new RegularImmutableSortedSet(this.elements.subList(newFromIndex, newToIndex), this.comparator);
        }
        return emptySet(this.comparator);
    }

    /* access modifiers changed from: package-private */
    public int indexOf(@Nullable Object target) {
        if (target == null) {
            return -1;
        }
        try {
            int position = SortedLists.binarySearch(this.elements, target, comparator(), SortedLists.KeyPresentBehavior.ANY_PRESENT, SortedLists.KeyAbsentBehavior.INVERTED_INSERTION_INDEX);
            if (position < 0 || !this.elements.get(position).equals(target)) {
                return -1;
            }
            return position;
        } catch (ClassCastException e) {
            return -1;
        }
    }

    /* access modifiers changed from: package-private */
    public ImmutableList<E> createAsList() {
        return new ImmutableSortedAsList(this, this.elements);
    }

    /* access modifiers changed from: package-private */
    public ImmutableSortedSet<E> createDescendingSet() {
        return new RegularImmutableSortedSet(this.elements.reverse(), Ordering.from(this.comparator).reverse());
    }
}
