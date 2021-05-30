package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Preconditions;
import java.util.List;
import javax.annotation.Nullable;

@GwtCompatible(emulated = true, serializable = true)
class RegularImmutableList<E> extends ImmutableList<E> {
    private final transient Object[] array;
    private final transient int offset;
    private final transient int size;

    RegularImmutableList(Object[] array2, int offset2, int size2) {
        this.offset = offset2;
        this.size = size2;
        this.array = array2;
    }

    RegularImmutableList(Object[] array2) {
        this(array2, 0, array2.length);
    }

    public int size() {
        return this.size;
    }

    public boolean isEmpty() {
        return false;
    }

    /* access modifiers changed from: package-private */
    public boolean isPartialView() {
        return (this.offset == 0 && this.size == this.array.length) ? false : true;
    }

    public Object[] toArray() {
        Object[] newArray = new Object[size()];
        System.arraycopy(this.array, this.offset, newArray, 0, this.size);
        return newArray;
    }

    public <T> T[] toArray(T[] other) {
        if (other.length < this.size) {
            other = ObjectArrays.newArray(other, this.size);
        } else if (other.length > this.size) {
            other[this.size] = null;
        }
        System.arraycopy(this.array, this.offset, other, 0, this.size);
        return other;
    }

    public E get(int index) {
        Preconditions.checkElementIndex(index, this.size);
        return this.array[this.offset + index];
    }

    /* access modifiers changed from: package-private */
    public ImmutableList<E> subListUnchecked(int fromIndex, int toIndex) {
        return new RegularImmutableList(this.array, this.offset + fromIndex, toIndex - fromIndex);
    }

    public UnmodifiableListIterator<E> listIterator(int index) {
        return Iterators.forArray(this.array, this.offset, this.size, index);
    }

    public boolean equals(@Nullable Object object) {
        if (object == this) {
            return true;
        }
        if (!(object instanceof List)) {
            return false;
        }
        List<?> that = (List) object;
        if (size() != that.size()) {
            return false;
        }
        int index = this.offset;
        if (object instanceof RegularImmutableList) {
            RegularImmutableList<?> other = (RegularImmutableList) object;
            int i = other.offset;
            while (i < other.offset + other.size) {
                int index2 = index + 1;
                if (!this.array[index].equals(other.array[i])) {
                    return false;
                }
                i++;
                index = index2;
            }
        } else {
            for (Object element : that) {
                int index3 = index + 1;
                if (this.array[index].equals(element) == 0) {
                    return false;
                }
                index = index3;
            }
        }
        return true;
    }

    public String toString() {
        StringBuilder newStringBuilderForCollection = Collections2.newStringBuilderForCollection(size());
        newStringBuilderForCollection.append('[');
        StringBuilder sb = newStringBuilderForCollection.append(this.array[this.offset]);
        int i = this.offset;
        while (true) {
            i++;
            if (i < this.offset + this.size) {
                sb.append(", ");
                sb.append(this.array[i]);
            } else {
                sb.append(']');
                return sb.toString();
            }
        }
    }
}
