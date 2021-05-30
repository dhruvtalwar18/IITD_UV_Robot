package org.apache.commons.lang;

import org.bytedeco.javacpp.opencv_stitching;

public final class NumberRange {
    private final Number max;
    private final Number min;

    public NumberRange(Number num) {
        if (num != null) {
            this.min = num;
            this.max = num;
            return;
        }
        throw new NullPointerException("The number must not be null");
    }

    public NumberRange(Number min2, Number max2) {
        if (min2 == null) {
            throw new NullPointerException("The minimum value must not be null");
        } else if (max2 == null) {
            throw new NullPointerException("The maximum value must not be null");
        } else if (max2.doubleValue() < min2.doubleValue()) {
            this.max = min2;
            this.min = min2;
        } else {
            this.min = min2;
            this.max = max2;
        }
    }

    public Number getMinimum() {
        return this.min;
    }

    public Number getMaximum() {
        return this.max;
    }

    public boolean includesNumber(Number number) {
        if (number != null && this.min.doubleValue() <= number.doubleValue() && this.max.doubleValue() >= number.doubleValue()) {
            return true;
        }
        return false;
    }

    public boolean includesRange(NumberRange range) {
        if (range != null && includesNumber(range.min) && includesNumber(range.max)) {
            return true;
        }
        return false;
    }

    public boolean overlaps(NumberRange range) {
        if (range == null) {
            return false;
        }
        if (range.includesNumber(this.min) || range.includesNumber(this.max) || includesRange(range)) {
            return true;
        }
        return false;
    }

    public boolean equals(Object obj) {
        if (obj == this) {
            return true;
        }
        if (!(obj instanceof NumberRange)) {
            return false;
        }
        NumberRange range = (NumberRange) obj;
        if (!this.min.equals(range.min) || !this.max.equals(range.max)) {
            return false;
        }
        return true;
    }

    public int hashCode() {
        return (((17 * 37) + this.min.hashCode()) * 37) + this.max.hashCode();
    }

    public String toString() {
        StringBuffer sb = new StringBuffer();
        if (this.min.doubleValue() < opencv_stitching.Stitcher.ORIG_RESOL) {
            sb.append('(');
            sb.append(this.min);
            sb.append(')');
        } else {
            sb.append(this.min);
        }
        sb.append('-');
        if (this.max.doubleValue() < opencv_stitching.Stitcher.ORIG_RESOL) {
            sb.append('(');
            sb.append(this.max);
            sb.append(')');
        } else {
            sb.append(this.max);
        }
        return sb.toString();
    }
}
