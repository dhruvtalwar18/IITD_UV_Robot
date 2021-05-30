package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Function;
import com.google.common.base.Preconditions;
import com.google.common.collect.Multiset;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import java.util.Set;
import java.util.SortedSet;

@GwtCompatible
final class SortedIterables {
    private SortedIterables() {
    }

    public static boolean hasSameComparator(Comparator<?> comparator, Iterable<?> elements) {
        Comparator<? super Object> comparator2;
        Preconditions.checkNotNull(comparator);
        Preconditions.checkNotNull(elements);
        if (elements instanceof SortedSet) {
            comparator2 = ((SortedSet) elements).comparator();
            if (comparator2 == null) {
                comparator2 = Ordering.natural();
            }
        } else if (elements instanceof SortedIterable) {
            comparator2 = ((SortedIterable) elements).comparator();
        } else {
            comparator2 = null;
        }
        return comparator.equals(comparator2);
    }

    public static <E> Collection<E> sortedUnique(Comparator<? super E> comparator, Iterator<E> elements) {
        SortedSet<E> sortedSet = Sets.newTreeSet(comparator);
        Iterators.addAll(sortedSet, elements);
        return sortedSet;
    }

    public static <E> Collection<E> sortedUnique(Comparator<? super E> comparator, Iterable<E> elements) {
        if (elements instanceof Multiset) {
            elements = ((Multiset) elements).elementSet();
        }
        if (!(elements instanceof Set)) {
            E[] array = Iterables.toArray(elements);
            if (!hasSameComparator(comparator, elements)) {
                Arrays.sort(array, comparator);
            }
            return uniquifySortedArray(comparator, array);
        } else if (hasSameComparator(comparator, elements)) {
            return (Set) elements;
        } else {
            List<E> list = Lists.newArrayList(elements);
            Collections.sort(list, comparator);
            return list;
        }
    }

    private static <E> Collection<E> uniquifySortedArray(Comparator<? super E> comparator, E[] array) {
        if (array.length == 0) {
            return Collections.emptySet();
        }
        int length = 1;
        for (int i = 1; i < array.length; i++) {
            if (comparator.compare(array[i], array[length - 1]) != 0) {
                array[length] = array[i];
                length++;
            }
        }
        if (length < array.length) {
            array = ObjectArrays.arraysCopyOf(array, length);
        }
        return Arrays.asList(array);
    }

    public static <E> Collection<Multiset.Entry<E>> sortedCounts(Comparator<? super E> comparator, Iterator<E> elements) {
        TreeMultiset<E> multiset = TreeMultiset.create(comparator);
        Iterators.addAll(multiset, elements);
        return multiset.entrySet();
    }

    public static <E> Collection<Multiset.Entry<E>> sortedCounts(Comparator<? super E> comparator, Iterable<E> elements) {
        List<E> list;
        if (elements instanceof Multiset) {
            Multiset<E> multiset = (Multiset) elements;
            if (hasSameComparator(comparator, elements)) {
                return multiset.entrySet();
            }
            List<Multiset.Entry<E>> entries = Lists.newArrayList(multiset.entrySet());
            Collections.sort(entries, Ordering.from(comparator).onResultOf(new Function<Multiset.Entry<E>, E>() {
                public E apply(Multiset.Entry<E> entry) {
                    return entry.getElement();
                }
            }));
            return entries;
        } else if (elements instanceof Set) {
            if (hasSameComparator(comparator, elements)) {
                list = (Collection) elements;
            } else {
                List<E> list2 = Lists.newArrayList(elements);
                Collections.sort(list2, comparator);
                list = list2;
            }
            return singletonEntries(list);
        } else if (hasSameComparator(comparator, elements)) {
            E current = null;
            int currentCount = 0;
            List<Multiset.Entry<E>> sortedEntries = Lists.newArrayList();
            for (E e : elements) {
                if (currentCount <= 0) {
                    current = e;
                    currentCount = 1;
                } else if (comparator.compare(current, e) == 0) {
                    currentCount++;
                } else {
                    sortedEntries.add(Multisets.immutableEntry(current, currentCount));
                    current = e;
                    currentCount = 1;
                }
            }
            if (currentCount > 0) {
                sortedEntries.add(Multisets.immutableEntry(current, currentCount));
            }
            return sortedEntries;
        } else {
            E current2 = TreeMultiset.create(comparator);
            Iterables.addAll(current2, elements);
            return current2.entrySet();
        }
    }

    static <E> Collection<Multiset.Entry<E>> singletonEntries(Collection<E> set) {
        return Collections2.transform(set, new Function<E, Multiset.Entry<E>>() {
            public Multiset.Entry<E> apply(E elem) {
                return Multisets.immutableEntry(elem, 1);
            }
        });
    }
}
