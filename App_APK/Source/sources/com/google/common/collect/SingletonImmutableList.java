package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Preconditions;
import java.util.List;
import javax.annotation.Nullable;

@GwtCompatible(emulated = true, serializable = true)
final class SingletonImmutableList<E> extends ImmutableList<E> {
    final transient E element;

    SingletonImmutableList(E element2) {
        this.element = Preconditions.checkNotNull(element2);
    }

    public E get(int index) {
        Preconditions.checkElementIndex(index, 1);
        return this.element;
    }

    public int indexOf(@Nullable Object object) {
        return this.element.equals(object) ? 0 : -1;
    }

    public UnmodifiableIterator<E> iterator() {
        return Iterators.singletonIterator(this.element);
    }

    public int lastIndexOf(@Nullable Object object) {
        return indexOf(object);
    }

    public int size() {
        return 1;
    }

    public ImmutableList<E> subList(int fromIndex, int toIndex) {
        Preconditions.checkPositionIndexes(fromIndex, toIndex, 1);
        return fromIndex == toIndex ? ImmutableList.of() : this;
    }

    public ImmutableList<E> reverse() {
        return this;
    }

    public boolean contains(@Nullable Object object) {
        return this.element.equals(object);
    }

    public boolean equals(Object object) {
        if (object == this) {
            return true;
        }
        if (!(object instanceof List)) {
            return false;
        }
        List<?> that = (List) object;
        if (that.size() != 1 || !this.element.equals(that.get(0))) {
            return false;
        }
        return true;
    }

    public int hashCode() {
        return this.element.hashCode() + 31;
    }

    public String toString() {
        String elementToString = this.element.toString();
        StringBuilder sb = new StringBuilder(elementToString.length() + 2);
        sb.append('[');
        sb.append(elementToString);
        sb.append(']');
        return sb.toString();
    }

    public boolean isEmpty() {
        return false;
    }

    /* access modifiers changed from: package-private */
    public boolean isPartialView() {
        return false;
    }

    public Object[] toArray() {
        return new Object[]{this.element};
    }

    public <T> T[] toArray(T[] array) {
        if (array.length == 0) {
            array = ObjectArrays.newArray(array, 1);
        } else if (array.length > 1) {
            array[1] = null;
        }
        array[0] = this.element;
        return array;
    }
}
