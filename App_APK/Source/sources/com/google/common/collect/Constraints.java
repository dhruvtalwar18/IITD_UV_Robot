package com.google.common.collect;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Preconditions;
import java.util.Collection;
import java.util.List;
import java.util.ListIterator;
import java.util.RandomAccess;
import java.util.Set;
import java.util.SortedSet;

@GwtCompatible
@Beta
public final class Constraints {
    private Constraints() {
    }

    private enum NotNullConstraint implements Constraint<Object> {
        INSTANCE;

        public Object checkElement(Object element) {
            return Preconditions.checkNotNull(element);
        }

        public String toString() {
            return "Not null";
        }
    }

    public static <E> Constraint<E> notNull() {
        return NotNullConstraint.INSTANCE;
    }

    public static <E> Collection<E> constrainedCollection(Collection<E> collection, Constraint<? super E> constraint) {
        return new ConstrainedCollection(collection, constraint);
    }

    static class ConstrainedCollection<E> extends ForwardingCollection<E> {
        private final Constraint<? super E> constraint;
        private final Collection<E> delegate;

        public ConstrainedCollection(Collection<E> delegate2, Constraint<? super E> constraint2) {
            this.delegate = (Collection) Preconditions.checkNotNull(delegate2);
            this.constraint = (Constraint) Preconditions.checkNotNull(constraint2);
        }

        /* access modifiers changed from: protected */
        public Collection<E> delegate() {
            return this.delegate;
        }

        public boolean add(E element) {
            this.constraint.checkElement(element);
            return this.delegate.add(element);
        }

        public boolean addAll(Collection<? extends E> elements) {
            return this.delegate.addAll(Constraints.checkElements(elements, this.constraint));
        }
    }

    public static <E> Set<E> constrainedSet(Set<E> set, Constraint<? super E> constraint) {
        return new ConstrainedSet(set, constraint);
    }

    static class ConstrainedSet<E> extends ForwardingSet<E> {
        private final Constraint<? super E> constraint;
        private final Set<E> delegate;

        public ConstrainedSet(Set<E> delegate2, Constraint<? super E> constraint2) {
            this.delegate = (Set) Preconditions.checkNotNull(delegate2);
            this.constraint = (Constraint) Preconditions.checkNotNull(constraint2);
        }

        /* access modifiers changed from: protected */
        public Set<E> delegate() {
            return this.delegate;
        }

        public boolean add(E element) {
            this.constraint.checkElement(element);
            return this.delegate.add(element);
        }

        public boolean addAll(Collection<? extends E> elements) {
            return this.delegate.addAll(Constraints.checkElements(elements, this.constraint));
        }
    }

    public static <E> SortedSet<E> constrainedSortedSet(SortedSet<E> sortedSet, Constraint<? super E> constraint) {
        return new ConstrainedSortedSet(sortedSet, constraint);
    }

    private static class ConstrainedSortedSet<E> extends ForwardingSortedSet<E> {
        final Constraint<? super E> constraint;
        final SortedSet<E> delegate;

        ConstrainedSortedSet(SortedSet<E> delegate2, Constraint<? super E> constraint2) {
            this.delegate = (SortedSet) Preconditions.checkNotNull(delegate2);
            this.constraint = (Constraint) Preconditions.checkNotNull(constraint2);
        }

        /* access modifiers changed from: protected */
        public SortedSet<E> delegate() {
            return this.delegate;
        }

        public SortedSet<E> headSet(E toElement) {
            return Constraints.constrainedSortedSet(this.delegate.headSet(toElement), this.constraint);
        }

        public SortedSet<E> subSet(E fromElement, E toElement) {
            return Constraints.constrainedSortedSet(this.delegate.subSet(fromElement, toElement), this.constraint);
        }

        public SortedSet<E> tailSet(E fromElement) {
            return Constraints.constrainedSortedSet(this.delegate.tailSet(fromElement), this.constraint);
        }

        public boolean add(E element) {
            this.constraint.checkElement(element);
            return this.delegate.add(element);
        }

        public boolean addAll(Collection<? extends E> elements) {
            return this.delegate.addAll(Constraints.checkElements(elements, this.constraint));
        }
    }

    public static <E> List<E> constrainedList(List<E> list, Constraint<? super E> constraint) {
        return list instanceof RandomAccess ? new ConstrainedRandomAccessList(list, constraint) : new ConstrainedList(list, constraint);
    }

    @GwtCompatible
    private static class ConstrainedList<E> extends ForwardingList<E> {
        final Constraint<? super E> constraint;
        final List<E> delegate;

        ConstrainedList(List<E> delegate2, Constraint<? super E> constraint2) {
            this.delegate = (List) Preconditions.checkNotNull(delegate2);
            this.constraint = (Constraint) Preconditions.checkNotNull(constraint2);
        }

        /* access modifiers changed from: protected */
        public List<E> delegate() {
            return this.delegate;
        }

        public boolean add(E element) {
            this.constraint.checkElement(element);
            return this.delegate.add(element);
        }

        public void add(int index, E element) {
            this.constraint.checkElement(element);
            this.delegate.add(index, element);
        }

        public boolean addAll(Collection<? extends E> elements) {
            return this.delegate.addAll(Constraints.checkElements(elements, this.constraint));
        }

        public boolean addAll(int index, Collection<? extends E> elements) {
            return this.delegate.addAll(index, Constraints.checkElements(elements, this.constraint));
        }

        public ListIterator<E> listIterator() {
            return Constraints.constrainedListIterator(this.delegate.listIterator(), this.constraint);
        }

        public ListIterator<E> listIterator(int index) {
            return Constraints.constrainedListIterator(this.delegate.listIterator(index), this.constraint);
        }

        public E set(int index, E element) {
            this.constraint.checkElement(element);
            return this.delegate.set(index, element);
        }

        public List<E> subList(int fromIndex, int toIndex) {
            return Constraints.constrainedList(this.delegate.subList(fromIndex, toIndex), this.constraint);
        }
    }

    static class ConstrainedRandomAccessList<E> extends ConstrainedList<E> implements RandomAccess {
        ConstrainedRandomAccessList(List<E> delegate, Constraint<? super E> constraint) {
            super(delegate, constraint);
        }
    }

    /* access modifiers changed from: private */
    public static <E> ListIterator<E> constrainedListIterator(ListIterator<E> listIterator, Constraint<? super E> constraint) {
        return new ConstrainedListIterator(listIterator, constraint);
    }

    static class ConstrainedListIterator<E> extends ForwardingListIterator<E> {
        private final Constraint<? super E> constraint;
        private final ListIterator<E> delegate;

        public ConstrainedListIterator(ListIterator<E> delegate2, Constraint<? super E> constraint2) {
            this.delegate = delegate2;
            this.constraint = constraint2;
        }

        /* access modifiers changed from: protected */
        public ListIterator<E> delegate() {
            return this.delegate;
        }

        public void add(E element) {
            this.constraint.checkElement(element);
            this.delegate.add(element);
        }

        public void set(E element) {
            this.constraint.checkElement(element);
            this.delegate.set(element);
        }
    }

    static <E> Collection<E> constrainedTypePreservingCollection(Collection<E> collection, Constraint<E> constraint) {
        if (collection instanceof SortedSet) {
            return constrainedSortedSet((SortedSet) collection, constraint);
        }
        if (collection instanceof Set) {
            return constrainedSet((Set) collection, constraint);
        }
        if (collection instanceof List) {
            return constrainedList((List) collection, constraint);
        }
        return constrainedCollection(collection, constraint);
    }

    public static <E> Multiset<E> constrainedMultiset(Multiset<E> multiset, Constraint<? super E> constraint) {
        return new ConstrainedMultiset(multiset, constraint);
    }

    static class ConstrainedMultiset<E> extends ForwardingMultiset<E> {
        private final Constraint<? super E> constraint;
        private Multiset<E> delegate;

        public ConstrainedMultiset(Multiset<E> delegate2, Constraint<? super E> constraint2) {
            this.delegate = (Multiset) Preconditions.checkNotNull(delegate2);
            this.constraint = (Constraint) Preconditions.checkNotNull(constraint2);
        }

        /* access modifiers changed from: protected */
        public Multiset<E> delegate() {
            return this.delegate;
        }

        public boolean add(E element) {
            return standardAdd(element);
        }

        public boolean addAll(Collection<? extends E> elements) {
            return this.delegate.addAll(Constraints.checkElements(elements, this.constraint));
        }

        public int add(E element, int occurrences) {
            this.constraint.checkElement(element);
            return this.delegate.add(element, occurrences);
        }

        public int setCount(E element, int count) {
            this.constraint.checkElement(element);
            return this.delegate.setCount(element, count);
        }

        public boolean setCount(E element, int oldCount, int newCount) {
            this.constraint.checkElement(element);
            return this.delegate.setCount(element, oldCount, newCount);
        }
    }

    /* access modifiers changed from: private */
    public static <E> Collection<E> checkElements(Collection<E> elements, Constraint<? super E> constraint) {
        Collection<E> copy = Lists.newArrayList(elements);
        for (E element : copy) {
            constraint.checkElement(element);
        }
        return copy;
    }
}
