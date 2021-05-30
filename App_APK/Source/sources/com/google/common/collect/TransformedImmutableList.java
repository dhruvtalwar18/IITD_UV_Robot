package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Preconditions;
import java.util.List;
import javax.annotation.Nullable;

@GwtCompatible
abstract class TransformedImmutableList<D, E> extends ImmutableList<E> {
    private final transient ImmutableList<D> backingList;

    /* access modifiers changed from: package-private */
    public abstract E transform(D d);

    private class TransformedView extends TransformedImmutableList<D, E> {
        public /* bridge */ /* synthetic */ List subList(int x0, int x1) {
            return TransformedImmutableList.super.subList(x0, x1);
        }

        TransformedView(ImmutableList<D> backingList) {
            super(backingList);
        }

        /* access modifiers changed from: package-private */
        public E transform(D d) {
            return TransformedImmutableList.this.transform(d);
        }
    }

    TransformedImmutableList(ImmutableList<D> backingList2) {
        this.backingList = (ImmutableList) Preconditions.checkNotNull(backingList2);
    }

    public E get(int index) {
        return transform(this.backingList.get(index));
    }

    public int size() {
        return this.backingList.size();
    }

    public ImmutableList<E> subList(int fromIndex, int toIndex) {
        return new TransformedView(this.backingList.subList(fromIndex, toIndex));
    }

    public boolean equals(@Nullable Object obj) {
        if (obj == this) {
            return true;
        }
        if (!(obj instanceof List)) {
            return false;
        }
        List<?> list = (List) obj;
        if (size() != list.size() || !Iterators.elementsEqual(iterator(), list.iterator())) {
            return false;
        }
        return true;
    }

    public int hashCode() {
        return Lists.hashCodeImpl(this);
    }

    /* access modifiers changed from: package-private */
    public boolean isPartialView() {
        return true;
    }
}
