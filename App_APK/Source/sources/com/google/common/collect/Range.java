package com.google.common.collect;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Preconditions;
import com.google.common.base.Predicate;
import java.io.Serializable;
import java.lang.Comparable;
import java.util.Comparator;
import java.util.NoSuchElementException;
import java.util.SortedSet;
import javax.annotation.Nullable;

@GwtCompatible
@Beta
public final class Range<C extends Comparable> implements Predicate<C>, Serializable {
    private static final long serialVersionUID = 0;
    final Cut<C> lowerBound;
    final Cut<C> upperBound;

    Range(Cut<C> lowerBound2, Cut<C> upperBound2) {
        if (lowerBound2.compareTo(upperBound2) <= 0) {
            this.lowerBound = lowerBound2;
            this.upperBound = upperBound2;
            return;
        }
        throw new IllegalArgumentException("Invalid range: " + toString(lowerBound2, upperBound2));
    }

    public boolean hasLowerBound() {
        return this.lowerBound != Cut.belowAll();
    }

    public C lowerEndpoint() {
        return this.lowerBound.endpoint();
    }

    public BoundType lowerBoundType() {
        return this.lowerBound.typeAsLowerBound();
    }

    public boolean hasUpperBound() {
        return this.upperBound != Cut.aboveAll();
    }

    public C upperEndpoint() {
        return this.upperBound.endpoint();
    }

    public BoundType upperBoundType() {
        return this.upperBound.typeAsUpperBound();
    }

    public boolean isEmpty() {
        return this.lowerBound.equals(this.upperBound);
    }

    public boolean contains(C value) {
        Preconditions.checkNotNull(value);
        return this.lowerBound.isLessThan(value) && !this.upperBound.isLessThan(value);
    }

    public boolean apply(C input) {
        return contains(input);
    }

    public boolean containsAll(Iterable<? extends C> values) {
        if (Iterables.isEmpty(values)) {
            return true;
        }
        if (values instanceof SortedSet) {
            SortedSet<? extends C> set = cast(values);
            Comparator<? super Object> comparator = set.comparator();
            if (Ordering.natural().equals(comparator) || comparator == null) {
                if (!contains((Comparable) set.first()) || !contains((Comparable) set.last())) {
                    return false;
                }
                return true;
            }
        }
        for (C value : values) {
            if (!contains(value)) {
                return false;
            }
        }
        return true;
    }

    public boolean encloses(Range<C> other) {
        return this.lowerBound.compareTo(other.lowerBound) <= 0 && this.upperBound.compareTo(other.upperBound) >= 0;
    }

    public boolean isConnected(Range<C> other) {
        return this.lowerBound.compareTo(other.upperBound) <= 0 && other.lowerBound.compareTo(this.upperBound) <= 0;
    }

    public Range<C> intersection(Range<C> connectedRange) {
        return Ranges.create((Cut) Ordering.natural().max(this.lowerBound, connectedRange.lowerBound), (Cut) Ordering.natural().min(this.upperBound, connectedRange.upperBound));
    }

    public Range<C> span(Range<C> other) {
        return Ranges.create((Cut) Ordering.natural().min(this.lowerBound, other.lowerBound), (Cut) Ordering.natural().max(this.upperBound, other.upperBound));
    }

    @GwtCompatible(serializable = false)
    public ContiguousSet<C> asSet(DiscreteDomain<C> domain) {
        Preconditions.checkNotNull(domain);
        Range range = this;
        try {
            if (!hasLowerBound()) {
                range = range.intersection(Ranges.atLeast(domain.minValue()));
            }
            if (!hasUpperBound()) {
                range = range.intersection(Ranges.atMost(domain.maxValue()));
            }
            return range.isEmpty() || compareOrThrow(this.lowerBound.leastValueAbove(domain), this.upperBound.greatestValueBelow(domain)) > 0 ? new EmptyContiguousSet(domain) : new RegularContiguousSet(range, domain);
        } catch (NoSuchElementException e) {
            throw new IllegalArgumentException(e);
        }
    }

    public Range<C> canonical(DiscreteDomain<C> domain) {
        Preconditions.checkNotNull(domain);
        Cut<C> lower = this.lowerBound.canonical(domain);
        Cut<C> upper = this.upperBound.canonical(domain);
        return (lower == this.lowerBound && upper == this.upperBound) ? this : Ranges.create(lower, upper);
    }

    public boolean equals(@Nullable Object object) {
        if (!(object instanceof Range)) {
            return false;
        }
        Range<?> other = (Range) object;
        if (!this.lowerBound.equals(other.lowerBound) || !this.upperBound.equals(other.upperBound)) {
            return false;
        }
        return true;
    }

    public int hashCode() {
        return (this.lowerBound.hashCode() * 31) + this.upperBound.hashCode();
    }

    public String toString() {
        return toString(this.lowerBound, this.upperBound);
    }

    private static String toString(Cut<?> lowerBound2, Cut<?> upperBound2) {
        StringBuilder sb = new StringBuilder(16);
        lowerBound2.describeAsLowerBound(sb);
        sb.append(8229);
        upperBound2.describeAsUpperBound(sb);
        return sb.toString();
    }

    private static <T> SortedSet<T> cast(Iterable<T> iterable) {
        return (SortedSet) iterable;
    }

    static int compareOrThrow(Comparable left, Comparable right) {
        return left.compareTo(right);
    }
}
