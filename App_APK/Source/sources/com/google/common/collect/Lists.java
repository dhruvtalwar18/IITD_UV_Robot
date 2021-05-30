package com.google.common.collect;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import com.google.common.annotations.GwtIncompatible;
import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Function;
import com.google.common.base.Objects;
import com.google.common.base.Preconditions;
import com.google.common.primitives.Ints;
import java.io.Serializable;
import java.util.AbstractList;
import java.util.AbstractSequentialList;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.NoSuchElementException;
import java.util.RandomAccess;
import java.util.concurrent.CopyOnWriteArrayList;
import javax.annotation.Nullable;

@GwtCompatible(emulated = true)
public final class Lists {
    private Lists() {
    }

    @GwtCompatible(serializable = true)
    public static <E> ArrayList<E> newArrayList() {
        return new ArrayList<>();
    }

    @GwtCompatible(serializable = true)
    public static <E> ArrayList<E> newArrayList(E... elements) {
        Preconditions.checkNotNull(elements);
        ArrayList<E> list = new ArrayList<>(computeArrayListCapacity(elements.length));
        Collections.addAll(list, elements);
        return list;
    }

    @VisibleForTesting
    static int computeArrayListCapacity(int arraySize) {
        Preconditions.checkArgument(arraySize >= 0);
        return Ints.saturatedCast(((long) arraySize) + 5 + ((long) (arraySize / 10)));
    }

    @GwtCompatible(serializable = true)
    public static <E> ArrayList<E> newArrayList(Iterable<? extends E> elements) {
        Preconditions.checkNotNull(elements);
        return elements instanceof Collection ? new ArrayList<>(Collections2.cast(elements)) : newArrayList(elements.iterator());
    }

    @GwtCompatible(serializable = true)
    public static <E> ArrayList<E> newArrayList(Iterator<? extends E> elements) {
        Preconditions.checkNotNull(elements);
        ArrayList<E> list = newArrayList();
        while (elements.hasNext()) {
            list.add(elements.next());
        }
        return list;
    }

    @GwtCompatible(serializable = true)
    public static <E> ArrayList<E> newArrayListWithCapacity(int initialArraySize) {
        Preconditions.checkArgument(initialArraySize >= 0);
        return new ArrayList<>(initialArraySize);
    }

    @GwtCompatible(serializable = true)
    public static <E> ArrayList<E> newArrayListWithExpectedSize(int estimatedSize) {
        return new ArrayList<>(computeArrayListCapacity(estimatedSize));
    }

    @GwtCompatible(serializable = true)
    public static <E> LinkedList<E> newLinkedList() {
        return new LinkedList<>();
    }

    @GwtCompatible(serializable = true)
    public static <E> LinkedList<E> newLinkedList(Iterable<? extends E> elements) {
        LinkedList<E> list = newLinkedList();
        for (E element : elements) {
            list.add(element);
        }
        return list;
    }

    @GwtIncompatible("CopyOnWriteArrayList")
    @Beta
    public static <E> CopyOnWriteArrayList<E> newCopyOnWriteArrayList() {
        return new CopyOnWriteArrayList<>();
    }

    /* JADX WARNING: type inference failed for: r2v0, types: [java.lang.Iterable<? extends E>, java.lang.Iterable] */
    /* JADX WARNING: Unknown variable types count: 1 */
    @com.google.common.annotations.GwtIncompatible("CopyOnWriteArrayList")
    @com.google.common.annotations.Beta
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public static <E> java.util.concurrent.CopyOnWriteArrayList<E> newCopyOnWriteArrayList(java.lang.Iterable<? extends E> r2) {
        /*
            boolean r0 = r2 instanceof java.util.Collection
            if (r0 == 0) goto L_0x0009
            java.util.Collection r0 = com.google.common.collect.Collections2.cast(r2)
            goto L_0x000d
        L_0x0009:
            java.util.ArrayList r0 = newArrayList(r2)
        L_0x000d:
            java.util.concurrent.CopyOnWriteArrayList r1 = new java.util.concurrent.CopyOnWriteArrayList
            r1.<init>(r0)
            return r1
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.common.collect.Lists.newCopyOnWriteArrayList(java.lang.Iterable):java.util.concurrent.CopyOnWriteArrayList");
    }

    public static <E> List<E> asList(@Nullable E first, E[] rest) {
        return new OnePlusArrayList(first, rest);
    }

    private static class OnePlusArrayList<E> extends AbstractList<E> implements Serializable, RandomAccess {
        private static final long serialVersionUID = 0;
        final E first;
        final E[] rest;

        OnePlusArrayList(@Nullable E first2, E[] rest2) {
            this.first = first2;
            this.rest = (Object[]) Preconditions.checkNotNull(rest2);
        }

        public int size() {
            return this.rest.length + 1;
        }

        public E get(int index) {
            Preconditions.checkElementIndex(index, size());
            return index == 0 ? this.first : this.rest[index - 1];
        }
    }

    public static <E> List<E> asList(@Nullable E first, @Nullable E second, E[] rest) {
        return new TwoPlusArrayList(first, second, rest);
    }

    private static class TwoPlusArrayList<E> extends AbstractList<E> implements Serializable, RandomAccess {
        private static final long serialVersionUID = 0;
        final E first;
        final E[] rest;
        final E second;

        TwoPlusArrayList(@Nullable E first2, @Nullable E second2, E[] rest2) {
            this.first = first2;
            this.second = second2;
            this.rest = (Object[]) Preconditions.checkNotNull(rest2);
        }

        public int size() {
            return this.rest.length + 2;
        }

        public E get(int index) {
            switch (index) {
                case 0:
                    return this.first;
                case 1:
                    return this.second;
                default:
                    Preconditions.checkElementIndex(index, size());
                    return this.rest[index - 2];
            }
        }
    }

    public static <F, T> List<T> transform(List<F> fromList, Function<? super F, ? extends T> function) {
        return fromList instanceof RandomAccess ? new TransformingRandomAccessList(fromList, function) : new TransformingSequentialList(fromList, function);
    }

    private static class TransformingSequentialList<F, T> extends AbstractSequentialList<T> implements Serializable {
        private static final long serialVersionUID = 0;
        final List<F> fromList;
        final Function<? super F, ? extends T> function;

        TransformingSequentialList(List<F> fromList2, Function<? super F, ? extends T> function2) {
            this.fromList = (List) Preconditions.checkNotNull(fromList2);
            this.function = (Function) Preconditions.checkNotNull(function2);
        }

        public void clear() {
            this.fromList.clear();
        }

        public int size() {
            return this.fromList.size();
        }

        public ListIterator<T> listIterator(int index) {
            final ListIterator<F> delegate = this.fromList.listIterator(index);
            return new ListIterator<T>() {
                public void add(T t) {
                    throw new UnsupportedOperationException();
                }

                public boolean hasNext() {
                    return delegate.hasNext();
                }

                public boolean hasPrevious() {
                    return delegate.hasPrevious();
                }

                public T next() {
                    return TransformingSequentialList.this.function.apply(delegate.next());
                }

                public int nextIndex() {
                    return delegate.nextIndex();
                }

                public T previous() {
                    return TransformingSequentialList.this.function.apply(delegate.previous());
                }

                public int previousIndex() {
                    return delegate.previousIndex();
                }

                public void remove() {
                    delegate.remove();
                }

                public void set(T t) {
                    throw new UnsupportedOperationException("not supported");
                }
            };
        }
    }

    private static class TransformingRandomAccessList<F, T> extends AbstractList<T> implements RandomAccess, Serializable {
        private static final long serialVersionUID = 0;
        final List<F> fromList;
        final Function<? super F, ? extends T> function;

        TransformingRandomAccessList(List<F> fromList2, Function<? super F, ? extends T> function2) {
            this.fromList = (List) Preconditions.checkNotNull(fromList2);
            this.function = (Function) Preconditions.checkNotNull(function2);
        }

        public void clear() {
            this.fromList.clear();
        }

        public T get(int index) {
            return this.function.apply(this.fromList.get(index));
        }

        public boolean isEmpty() {
            return this.fromList.isEmpty();
        }

        public T remove(int index) {
            return this.function.apply(this.fromList.remove(index));
        }

        public int size() {
            return this.fromList.size();
        }
    }

    public static <T> List<List<T>> partition(List<T> list, int size) {
        Preconditions.checkNotNull(list);
        Preconditions.checkArgument(size > 0);
        return list instanceof RandomAccess ? new RandomAccessPartition(list, size) : new Partition(list, size);
    }

    private static class Partition<T> extends AbstractList<List<T>> {
        final List<T> list;
        final int size;

        Partition(List<T> list2, int size2) {
            this.list = list2;
            this.size = size2;
        }

        public List<T> get(int index) {
            Preconditions.checkElementIndex(index, size());
            int start = this.size * index;
            return this.list.subList(start, Math.min(this.size + start, this.list.size()));
        }

        public int size() {
            int result = this.list.size() / this.size;
            if (this.size * result != this.list.size()) {
                return result + 1;
            }
            return result;
        }

        public boolean isEmpty() {
            return this.list.isEmpty();
        }
    }

    private static class RandomAccessPartition<T> extends Partition<T> implements RandomAccess {
        RandomAccessPartition(List<T> list, int size) {
            super(list, size);
        }
    }

    @Beta
    public static ImmutableList<Character> charactersOf(String string) {
        return new StringAsImmutableList((String) Preconditions.checkNotNull(string));
    }

    private static final class StringAsImmutableList extends ImmutableList<Character> {
        int hash = 0;
        private final String string;

        StringAsImmutableList(String string2) {
            this.string = string2;
        }

        public int indexOf(@Nullable Object object) {
            if (object instanceof Character) {
                return this.string.indexOf(((Character) object).charValue());
            }
            return -1;
        }

        public int lastIndexOf(@Nullable Object object) {
            if (object instanceof Character) {
                return this.string.lastIndexOf(((Character) object).charValue());
            }
            return -1;
        }

        public ImmutableList<Character> subList(int fromIndex, int toIndex) {
            Preconditions.checkPositionIndexes(fromIndex, toIndex, size());
            return Lists.charactersOf(this.string.substring(fromIndex, toIndex));
        }

        /* access modifiers changed from: package-private */
        public boolean isPartialView() {
            return false;
        }

        public Character get(int index) {
            Preconditions.checkElementIndex(index, size());
            return Character.valueOf(this.string.charAt(index));
        }

        public int size() {
            return this.string.length();
        }

        public boolean equals(@Nullable Object obj) {
            if (!(obj instanceof List)) {
                return false;
            }
            List<?> list = (List) obj;
            int n = this.string.length();
            if (n != list.size()) {
                return false;
            }
            Iterator<?> iterator = list.iterator();
            for (int i = 0; i < n; i++) {
                Object next = iterator.next();
                if (!(next instanceof Character) || ((Character) next).charValue() != this.string.charAt(i)) {
                    return false;
                }
            }
            return true;
        }

        public int hashCode() {
            int h = this.hash;
            if (h == 0) {
                h = 1;
                for (int i = 0; i < this.string.length(); i++) {
                    h = (h * 31) + this.string.charAt(i);
                }
                this.hash = h;
            }
            return h;
        }
    }

    @Beta
    public static List<Character> charactersOf(CharSequence sequence) {
        return new CharSequenceAsList((CharSequence) Preconditions.checkNotNull(sequence));
    }

    private static final class CharSequenceAsList extends AbstractList<Character> {
        private final CharSequence sequence;

        CharSequenceAsList(CharSequence sequence2) {
            this.sequence = sequence2;
        }

        public Character get(int index) {
            Preconditions.checkElementIndex(index, size());
            return Character.valueOf(this.sequence.charAt(index));
        }

        public boolean contains(@Nullable Object o) {
            return indexOf(o) >= 0;
        }

        public int indexOf(@Nullable Object o) {
            if (!(o instanceof Character)) {
                return -1;
            }
            char c = ((Character) o).charValue();
            for (int i = 0; i < this.sequence.length(); i++) {
                if (this.sequence.charAt(i) == c) {
                    return i;
                }
            }
            return -1;
        }

        public int lastIndexOf(@Nullable Object o) {
            if (!(o instanceof Character)) {
                return -1;
            }
            char c = ((Character) o).charValue();
            for (int i = this.sequence.length() - 1; i >= 0; i--) {
                if (this.sequence.charAt(i) == c) {
                    return i;
                }
            }
            return -1;
        }

        public int size() {
            return this.sequence.length();
        }

        public List<Character> subList(int fromIndex, int toIndex) {
            Preconditions.checkPositionIndexes(fromIndex, toIndex, size());
            return Lists.charactersOf(this.sequence.subSequence(fromIndex, toIndex));
        }

        public int hashCode() {
            int hash = 1;
            for (int i = 0; i < this.sequence.length(); i++) {
                hash = (hash * 31) + this.sequence.charAt(i);
            }
            return hash;
        }

        public boolean equals(@Nullable Object o) {
            if (!(o instanceof List)) {
                return false;
            }
            List<?> list = (List) o;
            int n = this.sequence.length();
            if (n != list.size()) {
                return false;
            }
            Iterator<?> iterator = list.iterator();
            for (int i = 0; i < n; i++) {
                Object next = iterator.next();
                if (!(next instanceof Character) || ((Character) next).charValue() != this.sequence.charAt(i)) {
                    return false;
                }
            }
            return true;
        }
    }

    public static <T> List<T> reverse(List<T> list) {
        if (list instanceof ReverseList) {
            return ((ReverseList) list).getForwardList();
        }
        if (list instanceof RandomAccess) {
            return new RandomAccessReverseList(list);
        }
        return new ReverseList(list);
    }

    private static class ReverseList<T> extends AbstractList<T> {
        private final List<T> forwardList;

        ReverseList(List<T> forwardList2) {
            this.forwardList = (List) Preconditions.checkNotNull(forwardList2);
        }

        /* access modifiers changed from: package-private */
        public List<T> getForwardList() {
            return this.forwardList;
        }

        private int reverseIndex(int index) {
            int size = size();
            Preconditions.checkElementIndex(index, size);
            return (size - 1) - index;
        }

        /* access modifiers changed from: private */
        public int reversePosition(int index) {
            int size = size();
            Preconditions.checkPositionIndex(index, size);
            return size - index;
        }

        public void add(int index, @Nullable T element) {
            this.forwardList.add(reversePosition(index), element);
        }

        public void clear() {
            this.forwardList.clear();
        }

        public T remove(int index) {
            return this.forwardList.remove(reverseIndex(index));
        }

        /* access modifiers changed from: protected */
        public void removeRange(int fromIndex, int toIndex) {
            subList(fromIndex, toIndex).clear();
        }

        public T set(int index, @Nullable T element) {
            return this.forwardList.set(reverseIndex(index), element);
        }

        public T get(int index) {
            return this.forwardList.get(reverseIndex(index));
        }

        public boolean isEmpty() {
            return this.forwardList.isEmpty();
        }

        public int size() {
            return this.forwardList.size();
        }

        public boolean contains(@Nullable Object o) {
            return this.forwardList.contains(o);
        }

        public boolean containsAll(Collection<?> c) {
            return this.forwardList.containsAll(c);
        }

        public List<T> subList(int fromIndex, int toIndex) {
            Preconditions.checkPositionIndexes(fromIndex, toIndex, size());
            return Lists.reverse(this.forwardList.subList(reversePosition(toIndex), reversePosition(fromIndex)));
        }

        public int indexOf(@Nullable Object o) {
            int index = this.forwardList.lastIndexOf(o);
            if (index >= 0) {
                return reverseIndex(index);
            }
            return -1;
        }

        public int lastIndexOf(@Nullable Object o) {
            int index = this.forwardList.indexOf(o);
            if (index >= 0) {
                return reverseIndex(index);
            }
            return -1;
        }

        public Iterator<T> iterator() {
            return listIterator();
        }

        public ListIterator<T> listIterator(int index) {
            final ListIterator<T> forwardIterator = this.forwardList.listIterator(reversePosition(index));
            return new ListIterator<T>() {
                boolean canRemove;
                boolean canSet;

                public void add(T e) {
                    forwardIterator.add(e);
                    forwardIterator.previous();
                    this.canRemove = false;
                    this.canSet = false;
                }

                public boolean hasNext() {
                    return forwardIterator.hasPrevious();
                }

                public boolean hasPrevious() {
                    return forwardIterator.hasNext();
                }

                public T next() {
                    if (hasNext()) {
                        this.canRemove = true;
                        this.canSet = true;
                        return forwardIterator.previous();
                    }
                    throw new NoSuchElementException();
                }

                public int nextIndex() {
                    return ReverseList.this.reversePosition(forwardIterator.nextIndex());
                }

                public T previous() {
                    if (hasPrevious()) {
                        this.canRemove = true;
                        this.canSet = true;
                        return forwardIterator.next();
                    }
                    throw new NoSuchElementException();
                }

                public int previousIndex() {
                    return nextIndex() - 1;
                }

                public void remove() {
                    Preconditions.checkState(this.canRemove);
                    forwardIterator.remove();
                    this.canSet = false;
                    this.canRemove = false;
                }

                public void set(T e) {
                    Preconditions.checkState(this.canSet);
                    forwardIterator.set(e);
                }
            };
        }
    }

    private static class RandomAccessReverseList<T> extends ReverseList<T> implements RandomAccess {
        RandomAccessReverseList(List<T> forwardList) {
            super(forwardList);
        }
    }

    static int hashCodeImpl(List<?> list) {
        int hashCode = 1;
        Iterator i$ = list.iterator();
        while (i$.hasNext()) {
            Object o = i$.next();
            hashCode = (hashCode * 31) + (o == null ? 0 : o.hashCode());
        }
        return hashCode;
    }

    static boolean equalsImpl(List<?> list, @Nullable Object object) {
        if (object == Preconditions.checkNotNull(list)) {
            return true;
        }
        if (!(object instanceof List)) {
            return false;
        }
        List<?> o = (List) object;
        if (list.size() != o.size() || !Iterators.elementsEqual(list.iterator(), o.iterator())) {
            return false;
        }
        return true;
    }

    static <E> boolean addAllImpl(List<E> list, int index, Iterable<? extends E> elements) {
        boolean changed = false;
        ListIterator<E> listIterator = list.listIterator(index);
        for (E e : elements) {
            listIterator.add(e);
            changed = true;
        }
        return changed;
    }

    static int indexOfImpl(List<?> list, @Nullable Object element) {
        ListIterator<?> listIterator = list.listIterator();
        while (listIterator.hasNext()) {
            if (Objects.equal(element, listIterator.next())) {
                return listIterator.previousIndex();
            }
        }
        return -1;
    }

    static int lastIndexOfImpl(List<?> list, @Nullable Object element) {
        ListIterator<?> listIterator = list.listIterator(list.size());
        while (listIterator.hasPrevious()) {
            if (Objects.equal(element, listIterator.previous())) {
                return listIterator.nextIndex();
            }
        }
        return -1;
    }

    static <E> ListIterator<E> listIteratorImpl(List<E> list, int index) {
        return new AbstractListWrapper(list).listIterator(index);
    }

    static <E> List<E> subListImpl(List<E> list, int fromIndex, int toIndex) {
        List<E> wrapper;
        if (list instanceof RandomAccess) {
            wrapper = new RandomAccessListWrapper<E>(list) {
                private static final long serialVersionUID = 0;

                public ListIterator<E> listIterator(int index) {
                    return this.backingList.listIterator(index);
                }
            };
        } else {
            wrapper = new AbstractListWrapper<E>(list) {
                private static final long serialVersionUID = 0;

                public ListIterator<E> listIterator(int index) {
                    return this.backingList.listIterator(index);
                }
            };
        }
        return wrapper.subList(fromIndex, toIndex);
    }

    private static class AbstractListWrapper<E> extends AbstractList<E> {
        final List<E> backingList;

        AbstractListWrapper(List<E> backingList2) {
            this.backingList = (List) Preconditions.checkNotNull(backingList2);
        }

        public void add(int index, E element) {
            this.backingList.add(index, element);
        }

        public boolean addAll(int index, Collection<? extends E> c) {
            return this.backingList.addAll(index, c);
        }

        public E get(int index) {
            return this.backingList.get(index);
        }

        public E remove(int index) {
            return this.backingList.remove(index);
        }

        public E set(int index, E element) {
            return this.backingList.set(index, element);
        }

        public boolean contains(Object o) {
            return this.backingList.contains(o);
        }

        public int size() {
            return this.backingList.size();
        }
    }

    private static class RandomAccessListWrapper<E> extends AbstractListWrapper<E> implements RandomAccess {
        RandomAccessListWrapper(List<E> backingList) {
            super(backingList);
        }
    }

    static <T> List<T> cast(Iterable<T> iterable) {
        return (List) iterable;
    }
}
