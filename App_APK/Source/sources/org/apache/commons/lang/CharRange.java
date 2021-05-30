package org.apache.commons.lang;

import java.io.Serializable;

public final class CharRange implements Serializable {
    private static final long serialVersionUID = 8270183163158333422L;
    private final char end;
    private transient String iToString;
    private final boolean negated;
    private final char start;

    public CharRange(char ch) {
        this(ch, ch, false);
    }

    public CharRange(char ch, boolean negated2) {
        this(ch, ch, negated2);
    }

    public CharRange(char start2, char end2) {
        this(start2, end2, false);
    }

    public CharRange(char start2, char end2, boolean negated2) {
        if (start2 > end2) {
            char temp = start2;
            start2 = end2;
            end2 = temp;
        }
        this.start = start2;
        this.end = end2;
        this.negated = negated2;
    }

    public char getStart() {
        return this.start;
    }

    public char getEnd() {
        return this.end;
    }

    public boolean isNegated() {
        return this.negated;
    }

    public boolean contains(char ch) {
        return (ch >= this.start && ch <= this.end) != this.negated;
    }

    public boolean contains(CharRange range) {
        if (range == null) {
            throw new IllegalArgumentException("The Range must not be null");
        } else if (this.negated) {
            if (range.negated) {
                if (this.start < range.start || this.end > range.end) {
                    return false;
                }
                return true;
            } else if (range.end < this.start || range.start > this.end) {
                return true;
            } else {
                return false;
            }
        } else if (range.negated) {
            if (this.start == 0 && this.end == 65535) {
                return true;
            }
            return false;
        } else if (this.start > range.start || this.end < range.end) {
            return false;
        } else {
            return true;
        }
    }

    public boolean equals(Object obj) {
        if (obj == this) {
            return true;
        }
        if (!(obj instanceof CharRange)) {
            return false;
        }
        CharRange other = (CharRange) obj;
        if (this.start == other.start && this.end == other.end && this.negated == other.negated) {
            return true;
        }
        return false;
    }

    public int hashCode() {
        return this.start + 'S' + (this.end * 7) + (this.negated ? 1 : 0);
    }

    public String toString() {
        if (this.iToString == null) {
            StringBuffer buf = new StringBuffer(4);
            if (isNegated()) {
                buf.append('^');
            }
            buf.append(this.start);
            if (this.start != this.end) {
                buf.append('-');
                buf.append(this.end);
            }
            this.iToString = buf.toString();
        }
        return this.iToString;
    }
}
