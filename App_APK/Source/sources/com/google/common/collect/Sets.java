package com.google.common.collect;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import com.google.common.annotations.GwtIncompatible;
import com.google.common.base.Preconditions;
import com.google.common.base.Predicate;
import com.google.common.base.Predicates;
import com.google.common.collect.Collections2;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.ImmutableSet;
import com.google.common.math.IntMath;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.Serializable;
import java.util.AbstractSet;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.EnumSet;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.NavigableSet;
import java.util.NoSuchElementException;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.concurrent.CopyOnWriteArraySet;
import javax.annotation.Nullable;

@GwtCompatible(emulated = true)
public final class Sets {
    private Sets() {
    }

    @GwtCompatible(serializable = true)
    public static <E extends Enum<E>> ImmutableSet<E> immutableEnumSet(E anElement, E... otherElements) {
        return new ImmutableEnumSet(EnumSet.of(anElement, otherElements));
    }

    @GwtCompatible(serializable = true)
    public static <E extends Enum<E>> ImmutableSet<E> immutableEnumSet(Iterable<E> elements) {
        Iterator<E> iterator = elements.iterator();
        if (!iterator.hasNext()) {
            return ImmutableSet.of();
        }
        if (elements instanceof EnumSet) {
            return new ImmutableEnumSet(EnumSet.copyOf((EnumSet) elements));
        }
        EnumSet<E> set = EnumSet.of((Enum) iterator.next());
        while (iterator.hasNext()) {
            set.add(iterator.next());
        }
        return new ImmutableEnumSet(set);
    }

    public static <E extends Enum<E>> EnumSet<E> newEnumSet(Iterable<E> iterable, Class<E> elementType) {
        Preconditions.checkNotNull(iterable);
        EnumSet<E> set = EnumSet.noneOf(elementType);
        Iterables.addAll(set, iterable);
        return set;
    }

    public static <E> HashSet<E> newHashSet() {
        return new HashSet<>();
    }

    public static <E> HashSet<E> newHashSet(E... elements) {
        HashSet<E> set = newHashSetWithExpectedSize(elements.length);
        Collections.addAll(set, elements);
        return set;
    }

    public static <E> HashSet<E> newHashSetWithExpectedSize(int expectedSize) {
        return new HashSet<>(Maps.capacity(expectedSize));
    }

    public static <E> HashSet<E> newHashSet(Iterable<? extends E> elements) {
        return elements instanceof Collection ? new HashSet<>(Collections2.cast(elements)) : newHashSet(elements.iterator());
    }

    public static <E> HashSet<E> newHashSet(Iterator<? extends E> elements) {
        HashSet<E> set = newHashSet();
        while (elements.hasNext()) {
            set.add(elements.next());
        }
        return set;
    }

    public static <E> LinkedHashSet<E> newLinkedHashSet() {
        return new LinkedHashSet<>();
    }

    public static <E> LinkedHashSet<E> newLinkedHashSetWithExpectedSize(int expectedSize) {
        return new LinkedHashSet<>(Maps.capacity(expectedSize));
    }

    public static <E> LinkedHashSet<E> newLinkedHashSet(Iterable<? extends E> elements) {
        if (elements instanceof Collection) {
            return new LinkedHashSet<>(Collections2.cast(elements));
        }
        LinkedHashSet<E> set = newLinkedHashSet();
        for (E element : elements) {
            set.add(element);
        }
        return set;
    }

    public static <E extends Comparable> TreeSet<E> newTreeSet() {
        return new TreeSet<>();
    }

    public static <E extends Comparable> TreeSet<E> newTreeSet(Iterable<? extends E> elements) {
        TreeSet<E> set = newTreeSet();
        for (E element : elements) {
            set.add(element);
        }
        return set;
    }

    public static <E> TreeSet<E> newTreeSet(Comparator<? super E> comparator) {
        return new TreeSet<>((Comparator) Preconditions.checkNotNull(comparator));
    }

    public static <E> Set<E> newIdentityHashSet() {
        return newSetFromMap(Maps.newIdentityHashMap());
    }

    @GwtIncompatible("CopyOnWriteArraySet")
    @Beta
    public static <E> CopyOnWriteArraySet<E> newCopyOnWriteArraySet() {
        return new CopyOnWriteArraySet<>();
    }

    /* JADX WARNING: type inference failed for: r2v0, types: [java.lang.Iterable<? extends E>, java.lang.Iterable] */
    /* JADX WARNING: Unknown variable types count: 1 */
    @com.google.common.annotations.GwtIncompatible("CopyOnWriteArraySet")
    @com.google.common.annotations.Beta
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public static <E> java.util.concurrent.CopyOnWriteArraySet<E> newCopyOnWriteArraySet(java.lang.Iterable<? extends E> r2) {
        /*
            boolean r0 = r2 instanceof java.util.Collection
            if (r0 == 0) goto L_0x0009
            java.util.Collection r0 = com.google.common.collect.Collections2.cast(r2)
            goto L_0x000d
        L_0x0009:
            java.util.ArrayList r0 = com.google.common.collect.Lists.newArrayList(r2)
        L_0x000d:
            java.util.concurrent.CopyOnWriteArraySet r1 = new java.util.concurrent.CopyOnWriteArraySet
            r1.<init>(r0)
            return r1
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.common.collect.Sets.newCopyOnWriteArraySet(java.lang.Iterable):java.util.concurrent.CopyOnWriteArraySet");
    }

    public static <E extends Enum<E>> EnumSet<E> complementOf(Collection<E> collection) {
        if (collection instanceof EnumSet) {
            return EnumSet.complementOf((EnumSet) collection);
        }
        Preconditions.checkArgument(!collection.isEmpty(), "collection is empty; use the other version of this method");
        return makeComplementByHand(collection, ((Enum) collection.iterator().next()).getDeclaringClass());
    }

    public static <E extends Enum<E>> EnumSet<E> complementOf(Collection<E> collection, Class<E> type) {
        Preconditions.checkNotNull(collection);
        return collection instanceof EnumSet ? EnumSet.complementOf((EnumSet) collection) : makeComplementByHand(collection, type);
    }

    private static <E extends Enum<E>> EnumSet<E> makeComplementByHand(Collection<E> collection, Class<E> type) {
        EnumSet<E> result = EnumSet.allOf(type);
        result.removeAll(collection);
        return result;
    }

    public static <E> Set<E> newSetFromMap(Map<E, Boolean> map) {
        return new SetFromMap(map);
    }

    private static class SetFromMap<E> extends AbstractSet<E> implements Set<E>, Serializable {
        @GwtIncompatible("not needed in emulated source")
        private static final long serialVersionUID = 0;
        private final Map<E, Boolean> m;
        private transient Set<E> s;

        SetFromMap(Map<E, Boolean> map) {
            Preconditions.checkArgument(map.isEmpty(), "Map is non-empty");
            this.m = map;
            this.s = map.keySet();
        }

        public void clear() {
            this.m.clear();
        }

        public int size() {
            return this.m.size();
        }

        public boolean isEmpty() {
            return this.m.isEmpty();
        }

        public boolean contains(Object o) {
            return this.m.containsKey(o);
        }

        public boolean remove(Object o) {
            return this.m.remove(o) != null;
        }

        public boolean add(E e) {
            return this.m.put(e, Boolean.TRUE) == null;
        }

        public Iterator<E> iterator() {
            return this.s.iterator();
        }

        public Object[] toArray() {
            return this.s.toArray();
        }

        public <T> T[] toArray(T[] a) {
            return this.s.toArray(a);
        }

        public String toString() {
            return this.s.toString();
        }

        public int hashCode() {
            return this.s.hashCode();
        }

        public boolean equals(@Nullable Object object) {
            return this == object || this.s.equals(object);
        }

        public boolean containsAll(Collection<?> c) {
            return this.s.containsAll(c);
        }

        public boolean removeAll(Collection<?> c) {
            return this.s.removeAll(c);
        }

        public boolean retainAll(Collection<?> c) {
            return this.s.retainAll(c);
        }

        @GwtIncompatible("java.io.ObjectInputStream")
        private void readObject(ObjectInputStream stream) throws IOException, ClassNotFoundException {
            stream.defaultReadObject();
            this.s = this.m.keySet();
        }
    }

    public static abstract class SetView<E> extends AbstractSet<E> {
        private SetView() {
        }

        public ImmutableSet<E> immutableCopy() {
            return ImmutableSet.copyOf(this);
        }

        public <S extends Set<E>> S copyInto(S set) {
            set.addAll(this);
            return set;
        }
    }

    public static <E> SetView<E> union(final Set<? extends E> set1, final Set<? extends E> set2) {
        Preconditions.checkNotNull(set1, "set1");
        Preconditions.checkNotNull(set2, "set2");
        final Set<? extends E> set2minus1 = difference(set2, set1);
        return new SetView<E>() {
            public int size() {
                return set1.size() + set2minus1.size();
            }

            public boolean isEmpty() {
                return set1.isEmpty() && set2.isEmpty();
            }

            public Iterator<E> iterator() {
                return Iterators.unmodifiableIterator(Iterators.concat(set1.iterator(), set2minus1.iterator()));
            }

            public boolean contains(Object object) {
                return set1.contains(object) || set2.contains(object);
            }

            public <S extends Set<E>> S copyInto(S set) {
                set.addAll(set1);
                set.addAll(set2);
                return set;
            }

            public ImmutableSet<E> immutableCopy() {
                return new ImmutableSet.Builder().addAll((Iterable) set1).addAll((Iterable) set2).build();
            }
        };
    }

    public static <E> SetView<E> intersection(final Set<E> set1, final Set<?> set2) {
        Preconditions.checkNotNull(set1, "set1");
        Preconditions.checkNotNull(set2, "set2");
        final Predicate<Object> inSet2 = Predicates.in(set2);
        return new SetView<E>() {
            public Iterator<E> iterator() {
                return Iterators.filter(set1.iterator(), inSet2);
            }

            public int size() {
                return Iterators.size(iterator());
            }

            public boolean isEmpty() {
                return !iterator().hasNext();
            }

            public boolean contains(Object object) {
                return set1.contains(object) && set2.contains(object);
            }

            public boolean containsAll(Collection<?> collection) {
                return set1.containsAll(collection) && set2.containsAll(collection);
            }
        };
    }

    public static <E> SetView<E> difference(final Set<E> set1, final Set<?> set2) {
        Preconditions.checkNotNull(set1, "set1");
        Preconditions.checkNotNull(set2, "set2");
        final Predicate<Object> notInSet2 = Predicates.not(Predicates.in(set2));
        return new SetView<E>() {
            public Iterator<E> iterator() {
                return Iterators.filter(set1.iterator(), notInSet2);
            }

            public int size() {
                return Iterators.size(iterator());
            }

            public boolean isEmpty() {
                return set2.containsAll(set1);
            }

            public boolean contains(Object element) {
                return set1.contains(element) && !set2.contains(element);
            }
        };
    }

    public static <E> SetView<E> symmetricDifference(Set<? extends E> set1, Set<? extends E> set2) {
        Preconditions.checkNotNull(set1, "set1");
        Preconditions.checkNotNull(set2, "set2");
        return difference(union(set1, set2), intersection(set1, set2));
    }

    public static <E> Set<E> filter(Set<E> unfiltered, Predicate<? super E> predicate) {
        if (unfiltered instanceof SortedSet) {
            return filter((SortedSet) unfiltered, predicate);
        }
        if (!(unfiltered instanceof FilteredSet)) {
            return new FilteredSet((Set) Preconditions.checkNotNull(unfiltered), (Predicate) Preconditions.checkNotNull(predicate));
        }
        FilteredSet<E> filtered = (FilteredSet) unfiltered;
        return new FilteredSet((Set) filtered.unfiltered, Predicates.and(filtered.predicate, predicate));
    }

    private static class FilteredSet<E> extends Collections2.FilteredCollection<E> implements Set<E> {
        FilteredSet(Set<E> unfiltered, Predicate<? super E> predicate) {
            super(unfiltered, predicate);
        }

        public boolean equals(@Nullable Object object) {
            return Sets.equalsImpl(this, object);
        }

        public int hashCode() {
            return Sets.hashCodeImpl(this);
        }
    }

    @Beta
    public static <E> SortedSet<E> filter(SortedSet<E> unfiltered, Predicate<? super E> predicate) {
        if (!(unfiltered instanceof FilteredSet)) {
            return new FilteredSortedSet((SortedSet) Preconditions.checkNotNull(unfiltered), (Predicate) Preconditions.checkNotNull(predicate));
        }
        FilteredSet<E> filtered = (FilteredSet) unfiltered;
        return new FilteredSortedSet((SortedSet) filtered.unfiltered, Predicates.and(filtered.predicate, predicate));
    }

    private static class FilteredSortedSet<E> extends Collections2.FilteredCollection<E> implements SortedSet<E> {
        FilteredSortedSet(SortedSet<E> unfiltered, Predicate<? super E> predicate) {
            super(unfiltered, predicate);
        }

        public boolean equals(@Nullable Object object) {
            return Sets.equalsImpl(this, object);
        }

        public int hashCode() {
            return Sets.hashCodeImpl(this);
        }

        public Comparator<? super E> comparator() {
            return ((SortedSet) this.unfiltered).comparator();
        }

        public SortedSet<E> subSet(E fromElement, E toElement) {
            return new FilteredSortedSet(((SortedSet) this.unfiltered).subSet(fromElement, toElement), this.predicate);
        }

        public SortedSet<E> headSet(E toElement) {
            return new FilteredSortedSet(((SortedSet) this.unfiltered).headSet(toElement), this.predicate);
        }

        public SortedSet<E> tailSet(E fromElement) {
            return new FilteredSortedSet(((SortedSet) this.unfiltered).tailSet(fromElement), this.predicate);
        }

        public E first() {
            return iterator().next();
        }

        public E last() {
            SortedSet<E> sortedUnfiltered = (SortedSet) this.unfiltered;
            while (true) {
                E element = sortedUnfiltered.last();
                if (this.predicate.apply(element)) {
                    return element;
                }
                sortedUnfiltered = sortedUnfiltered.headSet(element);
            }
        }
    }

    public static <B> Set<List<B>> cartesianProduct(List<? extends Set<? extends B>> sets) {
        for (Set<? extends B> set : sets) {
            if (set.isEmpty()) {
                return ImmutableSet.of();
            }
        }
        return new CartesianSet<>(sets);
    }

    public static <B> Set<List<B>> cartesianProduct(Set<? extends B>... sets) {
        return cartesianProduct(Arrays.asList(sets));
    }

    private static class CartesianSet<B> extends AbstractSet<List<B>> {
        final ImmutableList<CartesianSet<B>.Axis> axes;
        final int size;

        CartesianSet(List<? extends Set<? extends B>> sets) {
            int dividend = 1;
            ImmutableList.Builder<CartesianSet<B>.Axis> builder = ImmutableList.builder();
            try {
                for (Set<? extends B> set : sets) {
                    CartesianSet<B>.Axis axis = new Axis(set, dividend);
                    builder.add((Object) axis);
                    dividend = IntMath.checkedMultiply(dividend, axis.size());
                }
                this.axes = builder.build();
                this.size = dividend;
            } catch (ArithmeticException e) {
                throw new IllegalArgumentException("cartesian product too big");
            }
        }

        public int size() {
            return this.size;
        }

        public UnmodifiableIterator<List<B>> iterator() {
            return new AbstractIndexedListIterator<List<B>>(this.size) {
                /* access modifiers changed from: protected */
                public List<B> get(int index) {
                    Object[] tuple = new Object[CartesianSet.this.axes.size()];
                    for (int i = 0; i < tuple.length; i++) {
                        tuple[i] = ((Axis) CartesianSet.this.axes.get(i)).getForIndex(index);
                    }
                    return ImmutableList.copyOf((E[]) tuple);
                }
            };
        }

        public boolean contains(Object element) {
            if (!(element instanceof List)) {
                return false;
            }
            List<?> tuple = (List) element;
            int dimensions = this.axes.size();
            if (tuple.size() != dimensions) {
                return false;
            }
            for (int i = 0; i < dimensions; i++) {
                if (!((Axis) this.axes.get(i)).contains(tuple.get(i))) {
                    return false;
                }
            }
            return true;
        }

        public boolean equals(@Nullable Object object) {
            if (object instanceof CartesianSet) {
                return this.axes.equals(((CartesianSet) object).axes);
            }
            return super.equals(object);
        }

        public int hashCode() {
            int adjust = this.size - 1;
            for (int i = 0; i < this.axes.size(); i++) {
                adjust *= 31;
            }
            return this.axes.hashCode() + adjust;
        }

        private class Axis {
            final ImmutableSet<? extends B> choices;
            final ImmutableList<? extends B> choicesList = this.choices.asList();
            final int dividend;

            Axis(Set<? extends B> set, int dividend2) {
                this.choices = ImmutableSet.copyOf(set);
                this.dividend = dividend2;
            }

            /* access modifiers changed from: package-private */
            public int size() {
                return this.choices.size();
            }

            /* access modifiers changed from: package-private */
            public B getForIndex(int index) {
                return this.choicesList.get((index / this.dividend) % size());
            }

            /* access modifiers changed from: package-private */
            public boolean contains(Object target) {
                return this.choices.contains(target);
            }

            public boolean equals(Object obj) {
                if (obj instanceof Axis) {
                    return this.choices.equals(((Axis) obj).choices);
                }
                return false;
            }

            public int hashCode() {
                return (CartesianSet.this.size / this.choices.size()) * this.choices.hashCode();
            }
        }
    }

    @GwtCompatible(serializable = false)
    public static <E> Set<Set<E>> powerSet(Set<E> set) {
        ImmutableSet<E> input = ImmutableSet.copyOf(set);
        Preconditions.checkArgument(input.size() <= 30, "Too many elements to create power set: %s > 30", Integer.valueOf(input.size()));
        return new PowerSet(input);
    }

    private static final class PowerSet<E> extends AbstractSet<Set<E>> {
        final ImmutableList<E> inputList;
        final ImmutableSet<E> inputSet;
        final int powerSetSize;

        PowerSet(ImmutableSet<E> input) {
            this.inputSet = input;
            this.inputList = input.asList();
            this.powerSetSize = 1 << input.size();
        }

        public int size() {
            return this.powerSetSize;
        }

        public boolean isEmpty() {
            return false;
        }

        public Iterator<Set<E>> iterator() {
            return new AbstractIndexedListIterator<Set<E>>(this.powerSetSize) {
                /* access modifiers changed from: protected */
                public Set<E> get(final int setBits) {
                    return new AbstractSet<E>() {
                        public int size() {
                            return Integer.bitCount(setBits);
                        }

                        public Iterator<E> iterator() {
                            return new BitFilteredSetIterator(PowerSet.this.inputList, setBits);
                        }
                    };
                }
            };
        }

        private static final class BitFilteredSetIterator<E> extends UnmodifiableIterator<E> {
            final ImmutableList<E> input;
            int remainingSetBits;

            BitFilteredSetIterator(ImmutableList<E> input2, int allSetBits) {
                this.input = input2;
                this.remainingSetBits = allSetBits;
            }

            public boolean hasNext() {
                return this.remainingSetBits != 0;
            }

            public E next() {
                int index = Integer.numberOfTrailingZeros(this.remainingSetBits);
                if (index != 32) {
                    this.remainingSetBits &= (1 << index) ^ -1;
                    return this.input.get(index);
                }
                throw new NoSuchElementException();
            }
        }

        public boolean contains(@Nullable Object obj) {
            if (!(obj instanceof Set)) {
                return false;
            }
            return this.inputSet.containsAll((Set) obj);
        }

        public boolean equals(@Nullable Object obj) {
            if (obj instanceof PowerSet) {
                return this.inputSet.equals(((PowerSet) obj).inputSet);
            }
            return super.equals(obj);
        }

        public int hashCode() {
            return this.inputSet.hashCode() << (this.inputSet.size() - 1);
        }

        public String toString() {
            return "powerSet(" + this.inputSet + ")";
        }
    }

    static int hashCodeImpl(Set<?> s) {
        int hashCode = 0;
        Iterator i$ = s.iterator();
        while (i$.hasNext()) {
            Object o = i$.next();
            hashCode += o != null ? o.hashCode() : 0;
        }
        return hashCode;
    }

    static boolean equalsImpl(Set<?> s, @Nullable Object object) {
        if (s == object) {
            return true;
        }
        if (!(object instanceof Set)) {
            return false;
        }
        Set<?> o = (Set) object;
        try {
            if (s.size() != o.size() || !s.containsAll(o)) {
                return false;
            }
            return true;
        } catch (NullPointerException e) {
            return false;
        } catch (ClassCastException e2) {
            return false;
        }
    }

    @GwtIncompatible("NavigableSet")
    public static <E> NavigableSet<E> unmodifiableNavigableSet(NavigableSet<E> set) {
        if ((set instanceof ImmutableSortedSet) || (set instanceof UnmodifiableNavigableSet)) {
            return set;
        }
        return new UnmodifiableNavigableSet(set);
    }

    @GwtIncompatible("NavigableSet")
    static final class UnmodifiableNavigableSet<E> extends ForwardingSortedSet<E> implements NavigableSet<E>, Serializable {
        private static final long serialVersionUID = 0;
        private final NavigableSet<E> delegate;
        private transient UnmodifiableNavigableSet<E> descendingSet;

        UnmodifiableNavigableSet(NavigableSet<E> delegate2) {
            this.delegate = (NavigableSet) Preconditions.checkNotNull(delegate2);
        }

        /* access modifiers changed from: protected */
        public SortedSet<E> delegate() {
            return Collections.unmodifiableSortedSet(this.delegate);
        }

        public E lower(E e) {
            return this.delegate.lower(e);
        }

        public E floor(E e) {
            return this.delegate.floor(e);
        }

        public E ceiling(E e) {
            return this.delegate.ceiling(e);
        }

        public E higher(E e) {
            return this.delegate.higher(e);
        }

        public E pollFirst() {
            throw new UnsupportedOperationException();
        }

        public E pollLast() {
            throw new UnsupportedOperationException();
        }

        public NavigableSet<E> descendingSet() {
            UnmodifiableNavigableSet<E> result = this.descendingSet;
            if (result != null) {
                return result;
            }
            UnmodifiableNavigableSet<E> unmodifiableNavigableSet = new UnmodifiableNavigableSet<>(this.delegate.descendingSet());
            this.descendingSet = unmodifiableNavigableSet;
            UnmodifiableNavigableSet<E> result2 = unmodifiableNavigableSet;
            result2.descendingSet = this;
            return result2;
        }

        public Iterator<E> descendingIterator() {
            return Iterators.unmodifiableIterator(this.delegate.descendingIterator());
        }

        public NavigableSet<E> subSet(E fromElement, boolean fromInclusive, E toElement, boolean toInclusive) {
            return Sets.unmodifiableNavigableSet(this.delegate.subSet(fromElement, fromInclusive, toElement, toInclusive));
        }

        public NavigableSet<E> headSet(E toElement, boolean inclusive) {
            return Sets.unmodifiableNavigableSet(this.delegate.headSet(toElement, inclusive));
        }

        public NavigableSet<E> tailSet(E fromElement, boolean inclusive) {
            return Sets.unmodifiableNavigableSet(this.delegate.tailSet(fromElement, inclusive));
        }
    }

    static boolean removeAllImpl(Set<?> set, Iterator<?> iterator) {
        boolean changed = false;
        while (iterator.hasNext()) {
            changed |= set.remove(iterator.next());
        }
        return changed;
    }

    static boolean removeAllImpl(Set<?> set, Collection<?> collection) {
        if (collection instanceof Multiset) {
            collection = ((Multiset) collection).elementSet();
        }
        if (collection.size() < set.size()) {
            return removeAllImpl(set, collection.iterator());
        }
        return Iterators.removeAll(set.iterator(), collection);
    }

    @GwtIncompatible("NavigableSet")
    static class DescendingSet<E> extends ForwardingNavigableSet<E> {
        private final NavigableSet<E> forward;

        DescendingSet(NavigableSet<E> forward2) {
            this.forward = forward2;
        }

        /* access modifiers changed from: protected */
        public NavigableSet<E> delegate() {
            return this.forward;
        }

        public E lower(E e) {
            return this.forward.higher(e);
        }

        public E floor(E e) {
            return this.forward.ceiling(e);
        }

        public E ceiling(E e) {
            return this.forward.floor(e);
        }

        public E higher(E e) {
            return this.forward.lower(e);
        }

        public E pollFirst() {
            return this.forward.pollLast();
        }

        public E pollLast() {
            return this.forward.pollFirst();
        }

        public NavigableSet<E> descendingSet() {
            return this.forward;
        }

        public Iterator<E> descendingIterator() {
            return this.forward.iterator();
        }

        public NavigableSet<E> subSet(E fromElement, boolean fromInclusive, E toElement, boolean toInclusive) {
            return this.forward.subSet(toElement, toInclusive, fromElement, fromInclusive).descendingSet();
        }

        public NavigableSet<E> headSet(E toElement, boolean inclusive) {
            return this.forward.tailSet(toElement, inclusive).descendingSet();
        }

        public NavigableSet<E> tailSet(E fromElement, boolean inclusive) {
            return this.forward.headSet(fromElement, inclusive).descendingSet();
        }

        public Comparator<? super E> comparator() {
            Comparator<? super E> forwardComparator = this.forward.comparator();
            if (forwardComparator == null) {
                return Ordering.natural().reverse();
            }
            return reverse(forwardComparator);
        }

        private static <T> Ordering<T> reverse(Comparator<T> forward2) {
            return Ordering.from(forward2).reverse();
        }

        public E first() {
            return this.forward.last();
        }

        public SortedSet<E> headSet(E toElement) {
            return standardHeadSet(toElement);
        }

        public E last() {
            return this.forward.first();
        }

        public SortedSet<E> subSet(E fromElement, E toElement) {
            return standardSubSet(fromElement, toElement);
        }

        public SortedSet<E> tailSet(E fromElement) {
            return standardTailSet(fromElement);
        }

        public Iterator<E> iterator() {
            return this.forward.descendingIterator();
        }

        public Object[] toArray() {
            return standardToArray();
        }

        public <T> T[] toArray(T[] array) {
            return standardToArray(array);
        }

        public String toString() {
            return standardToString();
        }
    }

    static <T> SortedSet<T> cast(Iterable<T> iterable) {
        return (SortedSet) iterable;
    }
}
