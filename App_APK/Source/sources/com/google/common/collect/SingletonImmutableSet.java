package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Preconditions;
import java.util.Set;
import javax.annotation.Nullable;

@GwtCompatible(emulated = true, serializable = true)
final class SingletonImmutableSet<E> extends ImmutableSet<E> {
    private transient int cachedHashCode;
    final transient E element;

    SingletonImmutableSet(E element2) {
        this.element = Preconditions.checkNotNull(element2);
    }

    SingletonImmutableSet(E element2, int hashCode) {
        this.element = element2;
        this.cachedHashCode = hashCode;
    }

    public int size() {
        return 1;
    }

    public boolean isEmpty() {
        return false;
    }

    public boolean contains(Object target) {
        return this.element.equals(target);
    }

    public UnmodifiableIterator<E> iterator() {
        return Iterators.singletonIterator(this.element);
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

    public boolean equals(@Nullable Object object) {
        if (object == this) {
            return true;
        }
        if (!(object instanceof Set)) {
            return false;
        }
        Set<?> that = (Set) object;
        if (that.size() != 1 || !this.element.equals(that.iterator().next())) {
            return false;
        }
        return true;
    }

    public final int hashCode() {
        int code = this.cachedHashCode;
        if (code != 0) {
            return code;
        }
        int hashCode = this.element.hashCode();
        int code2 = hashCode;
        this.cachedHashCode = hashCode;
        return code2;
    }

    /* access modifiers changed from: package-private */
    public boolean isHashCodeFast() {
        return this.cachedHashCode != 0;
    }

    public String toString() {
        String elementToString = this.element.toString();
        StringBuilder sb = new StringBuilder(elementToString.length() + 2);
        sb.append('[');
        sb.append(elementToString);
        sb.append(']');
        return sb.toString();
    }
}
