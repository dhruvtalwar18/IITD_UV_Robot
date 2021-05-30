package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.annotations.GwtIncompatible;
import com.google.common.base.Preconditions;
import java.io.Serializable;
import java.lang.Comparable;
import java.util.Collection;
import javax.annotation.Nullable;
import org.xbill.DNS.TTL;

@GwtCompatible(emulated = true)
final class RegularContiguousSet<C extends Comparable> extends ContiguousSet<C> {
    private static final long serialVersionUID = 0;
    private final Range<C> range;

    RegularContiguousSet(Range<C> range2, DiscreteDomain<C> domain) {
        super(domain);
        this.range = range2;
    }

    private ContiguousSet<C> intersectionInCurrentDomain(Range<C> other) {
        return this.range.isConnected(other) ? this.range.intersection(other).asSet(this.domain) : new EmptyContiguousSet(this.domain);
    }

    /* access modifiers changed from: package-private */
    public ContiguousSet<C> headSetImpl(C toElement, boolean inclusive) {
        return intersectionInCurrentDomain(Ranges.upTo(toElement, BoundType.forBoolean(inclusive)));
    }

    /* access modifiers changed from: package-private */
    public ContiguousSet<C> subSetImpl(C fromElement, boolean fromInclusive, C toElement, boolean toInclusive) {
        if (fromElement.compareTo(toElement) != 0 || fromInclusive || toInclusive) {
            return intersectionInCurrentDomain(Ranges.range(fromElement, BoundType.forBoolean(fromInclusive), toElement, BoundType.forBoolean(toInclusive)));
        }
        return new EmptyContiguousSet(this.domain);
    }

    /* access modifiers changed from: package-private */
    public ContiguousSet<C> tailSetImpl(C fromElement, boolean inclusive) {
        return intersectionInCurrentDomain(Ranges.downTo(fromElement, BoundType.forBoolean(inclusive)));
    }

    /* access modifiers changed from: package-private */
    @GwtIncompatible("not used by GWT emulation")
    public int indexOf(Object target) {
        if (contains(target)) {
            return (int) this.domain.distance(first(), (Comparable) target);
        }
        return -1;
    }

    public UnmodifiableIterator<C> iterator() {
        return new AbstractSequentialIterator<C>(first()) {
            final C last = RegularContiguousSet.this.last();

            /* access modifiers changed from: protected */
            public C computeNext(C previous) {
                if (RegularContiguousSet.equalsOrThrow(previous, this.last)) {
                    return null;
                }
                return RegularContiguousSet.this.domain.next(previous);
            }
        };
    }

    /* access modifiers changed from: private */
    public static boolean equalsOrThrow(Comparable<?> left, @Nullable Comparable<?> right) {
        return right != null && Range.compareOrThrow(left, right) == 0;
    }

    /* access modifiers changed from: package-private */
    public boolean isPartialView() {
        return false;
    }

    public C first() {
        return this.range.lowerBound.leastValueAbove(this.domain);
    }

    public C last() {
        return this.range.upperBound.greatestValueBelow(this.domain);
    }

    public int size() {
        long distance = this.domain.distance(first(), last());
        if (distance >= TTL.MAX_VALUE) {
            return Integer.MAX_VALUE;
        }
        return ((int) distance) + 1;
    }

    public boolean contains(Object object) {
        if (object == null) {
            return false;
        }
        try {
            return this.range.contains((Comparable) object);
        } catch (ClassCastException e) {
            return false;
        }
    }

    public boolean containsAll(Collection<?> targets) {
        return Collections2.containsAllImpl(this, targets);
    }

    public boolean isEmpty() {
        return false;
    }

    public Object[] toArray() {
        return ObjectArrays.toArrayImpl(this);
    }

    public <T> T[] toArray(T[] other) {
        return ObjectArrays.toArrayImpl(this, other);
    }

    public ContiguousSet<C> intersection(ContiguousSet<C> other) {
        Preconditions.checkNotNull(other);
        Preconditions.checkArgument(this.domain.equals(other.domain));
        if (other.isEmpty()) {
            return other;
        }
        C lowerEndpoint = (Comparable) Ordering.natural().max(first(), other.first());
        C upperEndpoint = (Comparable) Ordering.natural().min(last(), other.last());
        return lowerEndpoint.compareTo(upperEndpoint) < 0 ? Ranges.closed(lowerEndpoint, upperEndpoint).asSet(this.domain) : new EmptyContiguousSet(this.domain);
    }

    public Range<C> range() {
        return range(BoundType.CLOSED, BoundType.CLOSED);
    }

    public Range<C> range(BoundType lowerBoundType, BoundType upperBoundType) {
        return Ranges.create(this.range.lowerBound.withLowerBoundType(lowerBoundType, this.domain), this.range.upperBound.withUpperBoundType(upperBoundType, this.domain));
    }

    public boolean equals(Object object) {
        if (object == this) {
            return true;
        }
        if (object instanceof RegularContiguousSet) {
            RegularContiguousSet<?> that = (RegularContiguousSet) object;
            if (this.domain.equals(that.domain)) {
                if (!first().equals(that.first()) || !last().equals(that.last())) {
                    return false;
                }
                return true;
            }
        }
        return super.equals(object);
    }

    public int hashCode() {
        return Sets.hashCodeImpl(this);
    }

    @GwtIncompatible("serialization")
    private static final class SerializedForm<C extends Comparable> implements Serializable {
        final DiscreteDomain<C> domain;
        final Range<C> range;

        private SerializedForm(Range<C> range2, DiscreteDomain<C> domain2) {
            this.range = range2;
            this.domain = domain2;
        }

        private Object readResolve() {
            return new RegularContiguousSet(this.range, this.domain);
        }
    }

    /* access modifiers changed from: package-private */
    @GwtIncompatible("serialization")
    public Object writeReplace() {
        return new SerializedForm(this.range, this.domain);
    }

    /* access modifiers changed from: package-private */
    @GwtIncompatible("NavigableSet")
    public ImmutableSortedSet<C> createDescendingSet() {
        return new DescendingContiguousSet();
    }

    @GwtIncompatible("NavigableSet")
    private final class DescendingContiguousSet extends ImmutableSortedSet<C> {
        private DescendingContiguousSet() {
            super(Ordering.natural().reverse());
        }

        public C first() {
            return RegularContiguousSet.this.last();
        }

        public C last() {
            return RegularContiguousSet.this.first();
        }

        public int size() {
            return RegularContiguousSet.this.size();
        }

        public UnmodifiableIterator<C> iterator() {
            return new AbstractSequentialIterator<C>(first()) {
                final C last = DescendingContiguousSet.this.last();

                /* access modifiers changed from: protected */
                public C computeNext(C previous) {
                    if (RegularContiguousSet.equalsOrThrow(previous, this.last)) {
                        return null;
                    }
                    return RegularContiguousSet.this.domain.previous(previous);
                }
            };
        }

        /* access modifiers changed from: package-private */
        public ImmutableSortedSet<C> headSetImpl(C toElement, boolean inclusive) {
            return RegularContiguousSet.this.tailSetImpl(toElement, inclusive).descendingSet();
        }

        /* access modifiers changed from: package-private */
        public ImmutableSortedSet<C> subSetImpl(C fromElement, boolean fromInclusive, C toElement, boolean toInclusive) {
            return RegularContiguousSet.this.subSetImpl(toElement, toInclusive, fromElement, fromInclusive).descendingSet();
        }

        /* access modifiers changed from: package-private */
        public ImmutableSortedSet<C> tailSetImpl(C fromElement, boolean inclusive) {
            return RegularContiguousSet.this.headSetImpl(fromElement, inclusive).descendingSet();
        }

        /* access modifiers changed from: package-private */
        public ImmutableSortedSet<C> createDescendingSet() {
            return RegularContiguousSet.this;
        }

        /* access modifiers changed from: package-private */
        public int indexOf(Object target) {
            if (contains(target)) {
                return (int) RegularContiguousSet.this.domain.distance(last(), (Comparable) target);
            }
            return -1;
        }

        /* access modifiers changed from: package-private */
        public boolean isPartialView() {
            return false;
        }
    }
}
