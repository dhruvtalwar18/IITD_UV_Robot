package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Preconditions;
import java.util.Collection;
import java.util.List;
import java.util.NoSuchElementException;
import javax.annotation.Nullable;

@GwtCompatible(emulated = true, serializable = true)
final class EmptyImmutableList extends ImmutableList<Object> {
    private static final Object[] EMPTY_ARRAY = new Object[0];
    static final EmptyImmutableList INSTANCE = new EmptyImmutableList();
    static final UnmodifiableListIterator<Object> ITERATOR = new UnmodifiableListIterator<Object>() {
        public boolean hasNext() {
            return false;
        }

        public boolean hasPrevious() {
            return false;
        }

        public Object next() {
            throw new NoSuchElementException();
        }

        public int nextIndex() {
            return 0;
        }

        public Object previous() {
            throw new NoSuchElementException();
        }

        public int previousIndex() {
            return -1;
        }
    };
    private static final long serialVersionUID = 0;

    private EmptyImmutableList() {
    }

    public int size() {
        return 0;
    }

    public boolean isEmpty() {
        return true;
    }

    /* access modifiers changed from: package-private */
    public boolean isPartialView() {
        return false;
    }

    public boolean contains(Object target) {
        return false;
    }

    public UnmodifiableIterator<Object> iterator() {
        return Iterators.emptyIterator();
    }

    public Object[] toArray() {
        return EMPTY_ARRAY;
    }

    public <T> T[] toArray(T[] a) {
        if (a.length > 0) {
            a[0] = null;
        }
        return a;
    }

    public Object get(int index) {
        Preconditions.checkElementIndex(index, 0);
        throw new AssertionError("unreachable");
    }

    public int indexOf(@Nullable Object target) {
        return -1;
    }

    public int lastIndexOf(@Nullable Object target) {
        return -1;
    }

    public ImmutableList<Object> subList(int fromIndex, int toIndex) {
        Preconditions.checkPositionIndexes(fromIndex, toIndex, 0);
        return this;
    }

    public ImmutableList<Object> reverse() {
        return this;
    }

    public UnmodifiableListIterator<Object> listIterator() {
        return ITERATOR;
    }

    public UnmodifiableListIterator<Object> listIterator(int start) {
        Preconditions.checkPositionIndex(start, 0);
        return ITERATOR;
    }

    public boolean containsAll(Collection<?> targets) {
        return targets.isEmpty();
    }

    public boolean equals(@Nullable Object object) {
        if (object instanceof List) {
            return ((List) object).isEmpty();
        }
        return false;
    }

    public int hashCode() {
        return 1;
    }

    public String toString() {
        return "[]";
    }

    /* access modifiers changed from: package-private */
    public Object readResolve() {
        return INSTANCE;
    }
}
