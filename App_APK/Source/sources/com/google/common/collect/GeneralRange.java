package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Objects;
import com.google.common.base.Preconditions;
import java.io.Serializable;
import java.util.Comparator;
import javax.annotation.Nullable;

@GwtCompatible(serializable = true)
final class GeneralRange<T> implements Serializable {
    private final Comparator<? super T> comparator;
    private final boolean hasLowerBound;
    private final boolean hasUpperBound;
    private final BoundType lowerBoundType;
    @Nullable
    private final T lowerEndpoint;
    private transient GeneralRange<T> reverse;
    private final BoundType upperBoundType;
    @Nullable
    private final T upperEndpoint;

    static <T extends Comparable> GeneralRange<T> from(Range<T> range) {
        T upperEndpoint2 = null;
        T lowerEndpoint2 = range.hasLowerBound() ? range.lowerEndpoint() : null;
        BoundType lowerBoundType2 = range.hasLowerBound() ? range.lowerBoundType() : BoundType.OPEN;
        if (range.hasUpperBound()) {
            upperEndpoint2 = range.upperEndpoint();
        }
        return new GeneralRange(Ordering.natural(), range.hasLowerBound(), lowerEndpoint2, lowerBoundType2, range.hasUpperBound(), upperEndpoint2, range.hasUpperBound() ? range.upperBoundType() : BoundType.OPEN);
    }

    static <T> GeneralRange<T> all(Comparator<? super T> comparator2) {
        return new GeneralRange(comparator2, false, (Object) null, BoundType.OPEN, false, (Object) null, BoundType.OPEN);
    }

    static <T> GeneralRange<T> downTo(Comparator<? super T> comparator2, @Nullable T endpoint, BoundType boundType) {
        return new GeneralRange(comparator2, true, endpoint, boundType, false, (T) null, BoundType.OPEN);
    }

    static <T> GeneralRange<T> upTo(Comparator<? super T> comparator2, @Nullable T endpoint, BoundType boundType) {
        return new GeneralRange(comparator2, false, (Object) null, BoundType.OPEN, true, endpoint, boundType);
    }

    static <T> GeneralRange<T> range(Comparator<? super T> comparator2, @Nullable T lower, BoundType lowerType, @Nullable T upper, BoundType upperType) {
        return new GeneralRange(comparator2, true, lower, lowerType, true, upper, upperType);
    }

    private GeneralRange(Comparator<? super T> comparator2, boolean hasLowerBound2, @Nullable T lowerEndpoint2, BoundType lowerBoundType2, boolean hasUpperBound2, @Nullable T upperEndpoint2, BoundType upperBoundType2) {
        this.comparator = (Comparator) Preconditions.checkNotNull(comparator2);
        this.hasLowerBound = hasLowerBound2;
        this.hasUpperBound = hasUpperBound2;
        this.lowerEndpoint = lowerEndpoint2;
        this.lowerBoundType = (BoundType) Preconditions.checkNotNull(lowerBoundType2);
        this.upperEndpoint = upperEndpoint2;
        this.upperBoundType = (BoundType) Preconditions.checkNotNull(upperBoundType2);
        if (hasLowerBound2) {
            comparator2.compare(lowerEndpoint2, lowerEndpoint2);
        }
        if (hasUpperBound2) {
            comparator2.compare(upperEndpoint2, upperEndpoint2);
        }
        if (hasLowerBound2 && hasUpperBound2) {
            int cmp = comparator2.compare(lowerEndpoint2, upperEndpoint2);
            boolean z = false;
            Preconditions.checkArgument(cmp <= 0, "lowerEndpoint (%s) > upperEndpoint (%s)", lowerEndpoint2, upperEndpoint2);
            if (cmp == 0) {
                Preconditions.checkArgument((upperBoundType2 != BoundType.OPEN ? true : z) | (lowerBoundType2 != BoundType.OPEN));
            }
        }
    }

    /* access modifiers changed from: package-private */
    public Comparator<? super T> comparator() {
        return this.comparator;
    }

    /* access modifiers changed from: package-private */
    public boolean hasLowerBound() {
        return this.hasLowerBound;
    }

    /* access modifiers changed from: package-private */
    public boolean hasUpperBound() {
        return this.hasUpperBound;
    }

    /* access modifiers changed from: package-private */
    public boolean isEmpty() {
        return (hasUpperBound() && tooLow(this.upperEndpoint)) || (hasLowerBound() && tooHigh(this.lowerEndpoint));
    }

    /* access modifiers changed from: package-private */
    public boolean tooLow(@Nullable T t) {
        boolean z = false;
        if (!hasLowerBound()) {
            return false;
        }
        int cmp = this.comparator.compare(t, this.lowerEndpoint);
        boolean z2 = cmp < 0;
        boolean z3 = cmp == 0;
        if (this.lowerBoundType == BoundType.OPEN) {
            z = true;
        }
        return (z & z3) | z2;
    }

    /* access modifiers changed from: package-private */
    public boolean tooHigh(@Nullable T t) {
        boolean z = false;
        if (!hasUpperBound()) {
            return false;
        }
        int cmp = this.comparator.compare(t, this.upperEndpoint);
        boolean z2 = cmp > 0;
        boolean z3 = cmp == 0;
        if (this.upperBoundType == BoundType.OPEN) {
            z = true;
        }
        return (z & z3) | z2;
    }

    /* access modifiers changed from: package-private */
    public boolean contains(@Nullable T t) {
        return !tooLow(t) && !tooHigh(t);
    }

    /* access modifiers changed from: package-private */
    public GeneralRange<T> intersect(GeneralRange<T> other) {
        int cmp;
        BoundType upType;
        BoundType lowType;
        int cmp2;
        int cmp3;
        GeneralRange<T> generalRange = other;
        Preconditions.checkNotNull(other);
        Preconditions.checkArgument(this.comparator.equals(generalRange.comparator));
        boolean hasLowBound = this.hasLowerBound;
        T lowEnd = this.lowerEndpoint;
        BoundType lowType2 = this.lowerBoundType;
        if (!hasLowerBound()) {
            hasLowBound = generalRange.hasLowerBound;
            lowEnd = generalRange.lowerEndpoint;
            lowType2 = generalRange.lowerBoundType;
        } else if (other.hasLowerBound() && ((cmp3 = this.comparator.compare(this.lowerEndpoint, generalRange.lowerEndpoint)) < 0 || (cmp3 == 0 && generalRange.lowerBoundType == BoundType.OPEN))) {
            lowEnd = generalRange.lowerEndpoint;
            lowType2 = generalRange.lowerBoundType;
        }
        boolean hasUpBound = this.hasUpperBound;
        T upEnd = this.upperEndpoint;
        BoundType upType2 = this.upperBoundType;
        if (!hasUpperBound()) {
            hasUpBound = generalRange.hasUpperBound;
            upEnd = generalRange.upperEndpoint;
            upType2 = generalRange.upperBoundType;
        } else if (other.hasUpperBound() && ((cmp = this.comparator.compare(this.upperEndpoint, generalRange.upperEndpoint)) > 0 || (cmp == 0 && generalRange.upperBoundType == BoundType.OPEN))) {
            upEnd = generalRange.upperEndpoint;
            upType2 = generalRange.upperBoundType;
        }
        boolean hasUpBound2 = hasUpBound;
        T upEnd2 = upEnd;
        if (!hasLowBound || !hasUpBound2 || ((cmp2 = this.comparator.compare(lowEnd, upEnd2)) <= 0 && !(cmp2 == 0 && lowType2 == BoundType.OPEN && upType2 == BoundType.OPEN))) {
            lowType = lowType2;
            upType = upType2;
        } else {
            lowEnd = upEnd2;
            lowType = BoundType.OPEN;
            upType = BoundType.CLOSED;
        }
        return new GeneralRange(this.comparator, hasLowBound, lowEnd, lowType, hasUpBound2, upEnd2, upType);
    }

    public boolean equals(@Nullable Object obj) {
        if (!(obj instanceof GeneralRange)) {
            return false;
        }
        GeneralRange<?> r = (GeneralRange) obj;
        if (!this.comparator.equals(r.comparator) || this.hasLowerBound != r.hasLowerBound || this.hasUpperBound != r.hasUpperBound || !this.lowerBoundType.equals(r.lowerBoundType) || !this.upperBoundType.equals(r.upperBoundType) || !Objects.equal(this.lowerEndpoint, r.lowerEndpoint) || !Objects.equal(this.upperEndpoint, r.upperEndpoint)) {
            return false;
        }
        return true;
    }

    public int hashCode() {
        return Objects.hashCode(this.comparator, this.lowerEndpoint, this.lowerBoundType, this.upperEndpoint, this.upperBoundType);
    }

    public GeneralRange<T> reverse() {
        GeneralRange<T> result = this.reverse;
        if (result != null) {
            return result;
        }
        GeneralRange<T> result2 = new GeneralRange<>(Ordering.from(this.comparator).reverse(), this.hasUpperBound, this.upperEndpoint, this.upperBoundType, this.hasLowerBound, this.lowerEndpoint, this.lowerBoundType);
        result2.reverse = this;
        this.reverse = result2;
        return result2;
    }

    public String toString() {
        StringBuilder builder = new StringBuilder();
        builder.append(this.comparator);
        builder.append(":");
        switch (this.lowerBoundType) {
            case CLOSED:
                builder.append('[');
                break;
            case OPEN:
                builder.append('(');
                break;
        }
        if (hasLowerBound()) {
            builder.append(this.lowerEndpoint);
        } else {
            builder.append("-∞");
        }
        builder.append(',');
        if (hasUpperBound()) {
            builder.append(this.upperEndpoint);
        } else {
            builder.append("∞");
        }
        switch (this.upperBoundType) {
            case CLOSED:
                builder.append(']');
                break;
            case OPEN:
                builder.append(')');
                break;
        }
        return builder.toString();
    }
}
