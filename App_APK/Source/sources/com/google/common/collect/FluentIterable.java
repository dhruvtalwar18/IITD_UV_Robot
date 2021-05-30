package com.google.common.collect;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import com.google.common.annotations.GwtIncompatible;
import com.google.common.base.Function;
import com.google.common.base.Optional;
import com.google.common.base.Preconditions;
import com.google.common.base.Predicate;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import java.util.SortedSet;
import javax.annotation.Nullable;

@GwtCompatible(emulated = true)
@Beta
public abstract class FluentIterable<E> implements Iterable<E> {
    private final Iterable<E> iterable;

    protected FluentIterable() {
        this.iterable = this;
    }

    FluentIterable(Iterable<E> iterable2) {
        this.iterable = (Iterable) Preconditions.checkNotNull(iterable2);
    }

    public static <E> FluentIterable<E> from(final Iterable<E> iterable2) {
        return iterable2 instanceof FluentIterable ? (FluentIterable) iterable2 : new FluentIterable<E>(iterable2) {
            public Iterator<E> iterator() {
                return iterable2.iterator();
            }
        };
    }

    @Deprecated
    public static <E> FluentIterable<E> from(FluentIterable<E> iterable2) {
        return (FluentIterable) Preconditions.checkNotNull(iterable2);
    }

    public String toString() {
        return Iterables.toString(this.iterable);
    }

    public final int size() {
        return Iterables.size(this.iterable);
    }

    public final boolean contains(@Nullable Object element) {
        return Iterables.contains(this.iterable, element);
    }

    public final FluentIterable<E> cycle() {
        return from(Iterables.cycle(this.iterable));
    }

    public final FluentIterable<E> filter(Predicate<? super E> predicate) {
        return from(Iterables.filter(this.iterable, predicate));
    }

    @GwtIncompatible("Class.isInstance")
    public final <T> FluentIterable<T> filter(Class<T> type) {
        return from(Iterables.filter((Iterable<?>) this.iterable, type));
    }

    public final boolean anyMatch(Predicate<? super E> predicate) {
        return Iterables.any(this.iterable, predicate);
    }

    public final boolean allMatch(Predicate<? super E> predicate) {
        return Iterables.all(this.iterable, predicate);
    }

    public final Optional<E> firstMatch(Predicate<? super E> predicate) {
        return Iterables.tryFind(this.iterable, predicate);
    }

    public final <T> FluentIterable<T> transform(Function<? super E, T> function) {
        return from(Iterables.transform(this.iterable, function));
    }

    public final Optional<E> first() {
        Iterator<E> iterator = this.iterable.iterator();
        return iterator.hasNext() ? Optional.of(iterator.next()) : Optional.absent();
    }

    public final Optional<E> last() {
        E current;
        if (this.iterable instanceof List) {
            List<E> list = (List) this.iterable;
            if (list.isEmpty()) {
                return Optional.absent();
            }
            return Optional.of(list.get(list.size() - 1));
        }
        Iterator<E> iterator = this.iterable.iterator();
        if (!iterator.hasNext()) {
            return Optional.absent();
        }
        if (this.iterable instanceof SortedSet) {
            return Optional.of(((SortedSet) this.iterable).last());
        }
        do {
            current = iterator.next();
        } while (iterator.hasNext());
        return Optional.of(current);
    }

    public final FluentIterable<E> skip(int numberToSkip) {
        return from(Iterables.skip(this.iterable, numberToSkip));
    }

    public final FluentIterable<E> limit(int size) {
        return from(Iterables.limit(this.iterable, size));
    }

    public final boolean isEmpty() {
        return !this.iterable.iterator().hasNext();
    }

    public final ImmutableList<E> toImmutableList() {
        return ImmutableList.copyOf(this.iterable);
    }

    public final ImmutableSet<E> toImmutableSet() {
        return ImmutableSet.copyOf(this.iterable);
    }

    public final ImmutableSortedSet<E> toImmutableSortedSet(Comparator<? super E> comparator) {
        return ImmutableSortedSet.copyOf(comparator, this.iterable);
    }

    @GwtIncompatible("Array.newArray(Class, int)")
    public final E[] toArray(Class<E> type) {
        return Iterables.toArray(this.iterable, type);
    }

    public final E get(int position) {
        return Iterables.get(this.iterable, position);
    }

    private static class FromIterableFunction<E> implements Function<Iterable<E>, FluentIterable<E>> {
        private FromIterableFunction() {
        }

        public FluentIterable<E> apply(Iterable<E> fromObject) {
            return FluentIterable.from(fromObject);
        }
    }
}
