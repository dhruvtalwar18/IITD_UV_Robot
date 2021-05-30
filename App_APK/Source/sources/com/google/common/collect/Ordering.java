package com.google.common.collect;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Function;
import com.google.common.base.Preconditions;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import javax.annotation.Nullable;

@GwtCompatible
public abstract class Ordering<T> implements Comparator<T> {
    static final int LEFT_IS_GREATER = 1;
    static final int RIGHT_IS_GREATER = -1;

    public abstract int compare(@Nullable T t, @Nullable T t2);

    @GwtCompatible(serializable = true)
    public static <C extends Comparable> Ordering<C> natural() {
        return NaturalOrdering.INSTANCE;
    }

    @GwtCompatible(serializable = true)
    public static <T> Ordering<T> from(Comparator<T> comparator) {
        return comparator instanceof Ordering ? (Ordering) comparator : new ComparatorOrdering(comparator);
    }

    @GwtCompatible(serializable = true)
    @Deprecated
    public static <T> Ordering<T> from(Ordering<T> ordering) {
        return (Ordering) Preconditions.checkNotNull(ordering);
    }

    @GwtCompatible(serializable = true)
    public static <T> Ordering<T> explicit(List<T> valuesInOrder) {
        return new ExplicitOrdering(valuesInOrder);
    }

    @GwtCompatible(serializable = true)
    public static <T> Ordering<T> explicit(T leastValue, T... remainingValuesInOrder) {
        return explicit(Lists.asList(leastValue, remainingValuesInOrder));
    }

    @VisibleForTesting
    static class IncomparableValueException extends ClassCastException {
        private static final long serialVersionUID = 0;
        final Object value;

        IncomparableValueException(Object value2) {
            super("Cannot compare value: " + value2);
            this.value = value2;
        }
    }

    public static Ordering<Object> arbitrary() {
        return ArbitraryOrderingHolder.ARBITRARY_ORDERING;
    }

    private static class ArbitraryOrderingHolder {
        static final Ordering<Object> ARBITRARY_ORDERING = new ArbitraryOrdering();

        private ArbitraryOrderingHolder() {
        }
    }

    @VisibleForTesting
    static class ArbitraryOrdering extends Ordering<Object> {
        private Map<Object, Integer> uids = Platform.tryWeakKeys(new MapMaker()).makeComputingMap(new Function<Object, Integer>() {
            final AtomicInteger counter = new AtomicInteger(0);

            public Integer apply(Object from) {
                return Integer.valueOf(this.counter.getAndIncrement());
            }
        });

        ArbitraryOrdering() {
        }

        public int compare(Object left, Object right) {
            if (left == right) {
                return 0;
            }
            int leftCode = identityHashCode(left);
            int rightCode = identityHashCode(right);
            if (leftCode != rightCode) {
                return leftCode < rightCode ? -1 : 1;
            }
            int result = this.uids.get(left).compareTo(this.uids.get(right));
            if (result != 0) {
                return result;
            }
            throw new AssertionError();
        }

        public String toString() {
            return "Ordering.arbitrary()";
        }

        /* access modifiers changed from: package-private */
        public int identityHashCode(Object object) {
            return System.identityHashCode(object);
        }
    }

    @GwtCompatible(serializable = true)
    public static Ordering<Object> usingToString() {
        return UsingToStringOrdering.INSTANCE;
    }

    @GwtCompatible(serializable = true)
    public static <T> Ordering<T> compound(Iterable<? extends Comparator<? super T>> comparators) {
        return new CompoundOrdering(comparators);
    }

    protected Ordering() {
    }

    @GwtCompatible(serializable = true)
    public <U extends T> Ordering<U> compound(Comparator<? super U> secondaryComparator) {
        return new CompoundOrdering(this, (Ordering) (Comparator) Preconditions.checkNotNull(secondaryComparator));
    }

    @GwtCompatible(serializable = true)
    public <S extends T> Ordering<S> reverse() {
        return new ReverseOrdering(this);
    }

    @GwtCompatible(serializable = true)
    public <F> Ordering<F> onResultOf(Function<F, ? extends T> function) {
        return new ByFunctionOrdering(function, this);
    }

    @GwtCompatible(serializable = true)
    public <S extends T> Ordering<Iterable<S>> lexicographical() {
        return new LexicographicalOrdering(this);
    }

    @GwtCompatible(serializable = true)
    public <S extends T> Ordering<S> nullsFirst() {
        return new NullsFirstOrdering(this);
    }

    @GwtCompatible(serializable = true)
    public <S extends T> Ordering<S> nullsLast() {
        return new NullsLastOrdering(this);
    }

    @Beta
    public <E extends T> List<E> leastOf(Iterable<E> iterable, int k) {
        E[] tmp;
        Preconditions.checkArgument(k >= 0, "%d is negative", Integer.valueOf(k));
        E[] values = Iterables.toArray(iterable);
        if (values.length <= k) {
            Arrays.sort(values, this);
            tmp = values;
        } else {
            quicksortLeastK(values, 0, values.length - 1, k);
            tmp = (Object[]) new Object[k];
            System.arraycopy(values, 0, tmp, 0, k);
        }
        return Collections.unmodifiableList(Arrays.asList(tmp));
    }

    @Beta
    public <E extends T> List<E> greatestOf(Iterable<E> iterable, int k) {
        return reverse().leastOf(iterable, k);
    }

    private <E extends T> void quicksortLeastK(E[] values, int left, int right, int k) {
        if (right > left) {
            int pivotNewIndex = partition(values, left, right, (left + right) >>> 1);
            quicksortLeastK(values, left, pivotNewIndex - 1, k);
            if (pivotNewIndex < k) {
                quicksortLeastK(values, pivotNewIndex + 1, right, k);
            }
        }
    }

    private <E extends T> int partition(E[] values, int left, int right, int pivotIndex) {
        E pivotValue = values[pivotIndex];
        values[pivotIndex] = values[right];
        values[right] = pivotValue;
        int i = left;
        int storeIndex = i;
        while (i < right) {
            if (compare(values[i], pivotValue) < 0) {
                ObjectArrays.swap(values, storeIndex, i);
                storeIndex++;
            }
            i++;
        }
        ObjectArrays.swap(values, right, storeIndex);
        return storeIndex;
    }

    public int binarySearch(List<? extends T> sortedList, @Nullable T key) {
        return Collections.binarySearch(sortedList, key, this);
    }

    public <E extends T> List<E> sortedCopy(Iterable<E> iterable) {
        List<E> list = Lists.newArrayList(iterable);
        Collections.sort(list, this);
        return list;
    }

    public <E extends T> ImmutableList<E> immutableSortedCopy(Iterable<E> iterable) {
        return ImmutableList.copyOf(sortedCopy(iterable));
    }

    public boolean isOrdered(Iterable<? extends T> iterable) {
        Iterator<? extends T> it = iterable.iterator();
        if (!it.hasNext()) {
            return true;
        }
        T prev = it.next();
        while (it.hasNext()) {
            T next = it.next();
            if (compare(prev, next) > 0) {
                return false;
            }
            prev = next;
        }
        return true;
    }

    public boolean isStrictlyOrdered(Iterable<? extends T> iterable) {
        Iterator<? extends T> it = iterable.iterator();
        if (!it.hasNext()) {
            return true;
        }
        T prev = it.next();
        while (it.hasNext()) {
            T next = it.next();
            if (compare(prev, next) >= 0) {
                return false;
            }
            prev = next;
        }
        return true;
    }

    @Beta
    public <E extends T> E max(Iterator<E> iterator) {
        E maxSoFar = iterator.next();
        while (iterator.hasNext()) {
            maxSoFar = max(maxSoFar, iterator.next());
        }
        return maxSoFar;
    }

    public <E extends T> E max(Iterable<E> iterable) {
        return max(iterable.iterator());
    }

    public <E extends T> E max(@Nullable E a, @Nullable E b, @Nullable E c, E... rest) {
        E maxSoFar = max(max(a, b), c);
        for (E r : rest) {
            maxSoFar = max(maxSoFar, r);
        }
        return maxSoFar;
    }

    public <E extends T> E max(@Nullable E a, @Nullable E b) {
        return compare(a, b) >= 0 ? a : b;
    }

    @Beta
    public <E extends T> E min(Iterator<E> iterator) {
        E minSoFar = iterator.next();
        while (iterator.hasNext()) {
            minSoFar = min(minSoFar, iterator.next());
        }
        return minSoFar;
    }

    public <E extends T> E min(Iterable<E> iterable) {
        return min(iterable.iterator());
    }

    public <E extends T> E min(@Nullable E a, @Nullable E b, @Nullable E c, E... rest) {
        E minSoFar = min(min(a, b), c);
        for (E r : rest) {
            minSoFar = min(minSoFar, r);
        }
        return minSoFar;
    }

    public <E extends T> E min(@Nullable E a, @Nullable E b) {
        return compare(a, b) <= 0 ? a : b;
    }
}
