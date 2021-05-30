package com.google.common.collect;

import com.google.common.collect.ImmutableAsList;
import java.util.Comparator;
import javax.annotation.Nullable;

final class ImmutableSortedAsList<E> extends ImmutableList<E> implements SortedIterable<E> {
    private final transient ImmutableList<E> backingList;
    private final transient ImmutableSortedSet<E> backingSet;

    ImmutableSortedAsList(ImmutableSortedSet<E> backingSet2, ImmutableList<E> backingList2) {
        this.backingSet = backingSet2;
        this.backingList = backingList2;
    }

    public Comparator<? super E> comparator() {
        return this.backingSet.comparator();
    }

    public int indexOf(@Nullable Object target) {
        return this.backingSet.indexOf(target);
    }

    public int lastIndexOf(@Nullable Object target) {
        return this.backingSet.indexOf(target);
    }

    /* access modifiers changed from: package-private */
    public ImmutableList<E> subListUnchecked(int fromIndex, int toIndex) {
        return new RegularImmutableSortedSet(this.backingList.subList(fromIndex, toIndex), this.backingSet.comparator()).asList();
    }

    /* access modifiers changed from: package-private */
    public Object writeReplace() {
        return new ImmutableAsList.SerializedForm(this.backingSet);
    }

    public UnmodifiableIterator<E> iterator() {
        return this.backingList.iterator();
    }

    public E get(int index) {
        return this.backingList.get(index);
    }

    public UnmodifiableListIterator<E> listIterator() {
        return this.backingList.listIterator();
    }

    public UnmodifiableListIterator<E> listIterator(int index) {
        return this.backingList.listIterator(index);
    }

    public int size() {
        return this.backingList.size();
    }

    public boolean equals(@Nullable Object obj) {
        return this.backingList.equals(obj);
    }

    public int hashCode() {
        return this.backingList.hashCode();
    }

    /* access modifiers changed from: package-private */
    public boolean isPartialView() {
        return this.backingList.isPartialView();
    }
}
