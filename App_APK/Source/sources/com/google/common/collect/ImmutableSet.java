package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableCollection;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.Set;
import javax.annotation.Nullable;

@GwtCompatible(emulated = true, serializable = true)
public abstract class ImmutableSet<E> extends ImmutableCollection<E> implements Set<E> {
    private static final int CUTOFF = ((int) Math.floor(7.516192768E8d));
    private static final double DESIRED_LOAD_FACTOR = 0.7d;
    static final int MAX_TABLE_SIZE = 1073741824;

    public abstract UnmodifiableIterator<E> iterator();

    public static <E> ImmutableSet<E> of() {
        return EmptyImmutableSet.INSTANCE;
    }

    public static <E> ImmutableSet<E> of(E element) {
        return new SingletonImmutableSet(element);
    }

    public static <E> ImmutableSet<E> of(E e1, E e2) {
        return construct(e1, e2);
    }

    public static <E> ImmutableSet<E> of(E e1, E e2, E e3) {
        return construct(e1, e2, e3);
    }

    public static <E> ImmutableSet<E> of(E e1, E e2, E e3, E e4) {
        return construct(e1, e2, e3, e4);
    }

    public static <E> ImmutableSet<E> of(E e1, E e2, E e3, E e4, E e5) {
        return construct(e1, e2, e3, e4, e5);
    }

    public static <E> ImmutableSet<E> of(E e1, E e2, E e3, E e4, E e5, E e6, E... others) {
        Object[] elements = new Object[(others.length + 6)];
        elements[0] = e1;
        elements[1] = e2;
        elements[2] = e3;
        elements[3] = e4;
        elements[4] = e5;
        elements[5] = e6;
        for (int i = 6; i < elements.length; i++) {
            elements[i] = others[i - 6];
        }
        return construct(elements);
    }

    private static <E> ImmutableSet<E> construct(Object... elements) {
        int tableSize = chooseTableSize(elements.length);
        Object[] table = new Object[tableSize];
        int mask = tableSize - 1;
        int hashCode = 0;
        ArrayList<Object> uniqueElementsList = null;
        for (int i = 0; i < elements.length; i++) {
            Object element = elements[i];
            int hash = element.hashCode();
            int j = Hashing.smear(hash);
            while (true) {
                int index = j & mask;
                Object value = table[index];
                if (value == null) {
                    if (uniqueElementsList != null) {
                        uniqueElementsList.add(element);
                    }
                    table[index] = element;
                    hashCode += hash;
                } else if (!value.equals(element)) {
                    j++;
                } else if (uniqueElementsList == null) {
                    uniqueElementsList = new ArrayList<>(elements.length);
                    for (int k = 0; k < i; k++) {
                        uniqueElementsList.add(elements[k]);
                    }
                }
            }
        }
        Object[] uniqueElements = uniqueElementsList == null ? elements : uniqueElementsList.toArray();
        if (uniqueElements.length == 1) {
            return new SingletonImmutableSet(uniqueElements[0], hashCode);
        }
        if (tableSize != chooseTableSize(uniqueElements.length)) {
            return construct(uniqueElements);
        }
        return new RegularImmutableSet(uniqueElements, hashCode, table, mask);
    }

    @VisibleForTesting
    static int chooseTableSize(int setSize) {
        boolean z = true;
        if (setSize < CUTOFF) {
            int tableSize = Integer.highestOneBit(setSize - 1) << 1;
            while (true) {
                double d = (double) tableSize;
                Double.isNaN(d);
                if (d * DESIRED_LOAD_FACTOR >= ((double) setSize)) {
                    return tableSize;
                }
                tableSize <<= 1;
            }
        } else {
            if (setSize >= 1073741824) {
                z = false;
            }
            Preconditions.checkArgument(z, "collection too large");
            return 1073741824;
        }
    }

    public static <E> ImmutableSet<E> copyOf(E[] elements) {
        switch (elements.length) {
            case 0:
                return of();
            case 1:
                return of(elements[0]);
            default:
                return construct((Object[]) elements.clone());
        }
    }

    public static <E> ImmutableSet<E> copyOf(Iterable<? extends E> elements) {
        return elements instanceof Collection ? copyOf(Collections2.cast(elements)) : copyOf(elements.iterator());
    }

    public static <E> ImmutableSet<E> copyOf(Iterator<? extends E> elements) {
        return copyFromCollection(Lists.newArrayList(elements));
    }

    public static <E> ImmutableSet<E> copyOf(Collection<? extends E> elements) {
        if ((elements instanceof ImmutableSet) && !(elements instanceof ImmutableSortedSet)) {
            ImmutableSet<E> set = (ImmutableSet) elements;
            if (!set.isPartialView()) {
                return set;
            }
        }
        return copyFromCollection(elements);
    }

    private static <E> ImmutableSet<E> copyFromCollection(Collection<? extends E> collection) {
        E[] elements = collection.toArray();
        switch (elements.length) {
            case 0:
                return of();
            case 1:
                return of(elements[0]);
            default:
                return construct(elements);
        }
    }

    ImmutableSet() {
    }

    /* access modifiers changed from: package-private */
    public boolean isHashCodeFast() {
        return false;
    }

    public boolean equals(@Nullable Object object) {
        if (object == this) {
            return true;
        }
        if (!(object instanceof ImmutableSet) || !isHashCodeFast() || !((ImmutableSet) object).isHashCodeFast() || hashCode() == object.hashCode()) {
            return Sets.equalsImpl(this, object);
        }
        return false;
    }

    public int hashCode() {
        return Sets.hashCodeImpl(this);
    }

    static abstract class ArrayImmutableSet<E> extends ImmutableSet<E> {
        final transient Object[] elements;

        ArrayImmutableSet(Object[] elements2) {
            this.elements = elements2;
        }

        public int size() {
            return this.elements.length;
        }

        public boolean isEmpty() {
            return false;
        }

        public UnmodifiableIterator<E> iterator() {
            return Iterators.forArray(this.elements);
        }

        public Object[] toArray() {
            Object[] array = new Object[size()];
            System.arraycopy(this.elements, 0, array, 0, size());
            return array;
        }

        public <T> T[] toArray(T[] array) {
            int size = size();
            if (array.length < size) {
                array = ObjectArrays.newArray(array, size);
            } else if (array.length > size) {
                array[size] = null;
            }
            System.arraycopy(this.elements, 0, array, 0, size);
            return array;
        }

        public boolean containsAll(Collection<?> targets) {
            if (targets == this) {
                return true;
            }
            if (!(targets instanceof ArrayImmutableSet)) {
                return ImmutableSet.super.containsAll(targets);
            }
            if (targets.size() > size()) {
                return false;
            }
            for (Object target : ((ArrayImmutableSet) targets).elements) {
                if (!contains(target)) {
                    return false;
                }
            }
            return true;
        }

        /* access modifiers changed from: package-private */
        public boolean isPartialView() {
            return false;
        }

        /* access modifiers changed from: package-private */
        public ImmutableList<E> createAsList() {
            return new ImmutableAsList(this.elements, this);
        }
    }

    static abstract class TransformedImmutableSet<D, E> extends ImmutableSet<E> {
        final int hashCode;
        final ImmutableCollection<D> source;

        /* access modifiers changed from: package-private */
        public abstract E transform(D d);

        TransformedImmutableSet(ImmutableCollection<D> source2) {
            this.source = source2;
            this.hashCode = Sets.hashCodeImpl(this);
        }

        TransformedImmutableSet(ImmutableCollection<D> source2, int hashCode2) {
            this.source = source2;
            this.hashCode = hashCode2;
        }

        public int size() {
            return this.source.size();
        }

        public boolean isEmpty() {
            return false;
        }

        public UnmodifiableIterator<E> iterator() {
            final Iterator<D> backingIterator = this.source.iterator();
            return new UnmodifiableIterator<E>() {
                public boolean hasNext() {
                    return backingIterator.hasNext();
                }

                public E next() {
                    return TransformedImmutableSet.this.transform(backingIterator.next());
                }
            };
        }

        public Object[] toArray() {
            return toArray(new Object[size()]);
        }

        public <T> T[] toArray(T[] array) {
            int size = size();
            if (array.length < size) {
                array = ObjectArrays.newArray(array, size);
            } else if (array.length > size) {
                array[size] = null;
            }
            Object[] objectArray = array;
            int i = 0;
            Iterator i$ = this.source.iterator();
            while (i$.hasNext()) {
                objectArray[i] = transform(i$.next());
                i++;
            }
            return array;
        }

        public final int hashCode() {
            return this.hashCode;
        }

        /* access modifiers changed from: package-private */
        public boolean isHashCodeFast() {
            return true;
        }
    }

    private static class SerializedForm implements Serializable {
        private static final long serialVersionUID = 0;
        final Object[] elements;

        SerializedForm(Object[] elements2) {
            this.elements = elements2;
        }

        /* access modifiers changed from: package-private */
        public Object readResolve() {
            return ImmutableSet.copyOf((E[]) this.elements);
        }
    }

    /* access modifiers changed from: package-private */
    public Object writeReplace() {
        return new SerializedForm(toArray());
    }

    public static <E> Builder<E> builder() {
        return new Builder<>();
    }

    public static class Builder<E> extends ImmutableCollection.Builder<E> {
        final ArrayList<E> contents = Lists.newArrayList();

        public Builder<E> add(E element) {
            this.contents.add(Preconditions.checkNotNull(element));
            return this;
        }

        public Builder<E> add(E... elements) {
            this.contents.ensureCapacity(this.contents.size() + elements.length);
            super.add(elements);
            return this;
        }

        public Builder<E> addAll(Iterable<? extends E> elements) {
            if (elements instanceof Collection) {
                this.contents.ensureCapacity(this.contents.size() + ((Collection) elements).size());
            }
            super.addAll(elements);
            return this;
        }

        public Builder<E> addAll(Iterator<? extends E> elements) {
            super.addAll(elements);
            return this;
        }

        public ImmutableSet<E> build() {
            return ImmutableSet.copyOf(this.contents);
        }
    }
}
