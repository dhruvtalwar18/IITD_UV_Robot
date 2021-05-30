package com.google.common.base;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import javax.annotation.CheckReturnValue;
import org.apache.commons.lang.CharUtils;

@GwtCompatible
@Beta
public abstract class CharMatcher implements Predicate<Character> {
    public static final CharMatcher ANY = new CharMatcher() {
        public /* bridge */ /* synthetic */ boolean apply(Object x0) {
            return CharMatcher.super.apply((Character) x0);
        }

        public boolean matches(char c) {
            return true;
        }

        public int indexIn(CharSequence sequence) {
            return sequence.length() == 0 ? -1 : 0;
        }

        public int indexIn(CharSequence sequence, int start) {
            int length = sequence.length();
            Preconditions.checkPositionIndex(start, length);
            if (start == length) {
                return -1;
            }
            return start;
        }

        public int lastIndexIn(CharSequence sequence) {
            return sequence.length() - 1;
        }

        public boolean matchesAllOf(CharSequence sequence) {
            Preconditions.checkNotNull(sequence);
            return true;
        }

        public boolean matchesNoneOf(CharSequence sequence) {
            return sequence.length() == 0;
        }

        public String removeFrom(CharSequence sequence) {
            Preconditions.checkNotNull(sequence);
            return "";
        }

        public String replaceFrom(CharSequence sequence, char replacement) {
            char[] array = new char[sequence.length()];
            Arrays.fill(array, replacement);
            return new String(array);
        }

        public String replaceFrom(CharSequence sequence, CharSequence replacement) {
            StringBuilder retval = new StringBuilder(sequence.length() * replacement.length());
            for (int i = 0; i < sequence.length(); i++) {
                retval.append(replacement);
            }
            return retval.toString();
        }

        public String collapseFrom(CharSequence sequence, char replacement) {
            return sequence.length() == 0 ? "" : String.valueOf(replacement);
        }

        public String trimFrom(CharSequence sequence) {
            Preconditions.checkNotNull(sequence);
            return "";
        }

        public int countIn(CharSequence sequence) {
            return sequence.length();
        }

        public CharMatcher and(CharMatcher other) {
            return (CharMatcher) Preconditions.checkNotNull(other);
        }

        public CharMatcher or(CharMatcher other) {
            Preconditions.checkNotNull(other);
            return this;
        }

        public CharMatcher negate() {
            return NONE;
        }

        public CharMatcher precomputed() {
            return this;
        }

        public String toString() {
            return "CharMatcher.ANY";
        }
    };
    public static final CharMatcher ASCII = inRange(0, Ascii.MAX).withToString("CharMatcher.ASCII");
    public static final CharMatcher BREAKING_WHITESPACE = anyOf("\t\n\u000b\f\r     　").or(inRange(8192, 8198)).or(inRange(8200, 8202)).withToString("CharMatcher.BREAKING_WHITESPACE").precomputed();
    public static final CharMatcher DIGIT;
    public static final CharMatcher INVISIBLE = inRange(0, ' ').or(inRange(Ascii.MAX, 160)).or(is(173)).or(inRange(1536, 1539)).or(anyOf("۝܏ ឴឵᠎")).or(inRange(8192, 8207)).or(inRange(8232, 8239)).or(inRange(8287, 8292)).or(inRange(8298, 8303)).or(is(12288)).or(inRange(55296, 63743)).or(anyOf("﻿￹￺￻")).withToString("CharMatcher.INVISIBLE").precomputed();
    public static final CharMatcher JAVA_DIGIT = new CharMatcher() {
        public /* bridge */ /* synthetic */ boolean apply(Object x0) {
            return CharMatcher.super.apply((Character) x0);
        }

        public boolean matches(char c) {
            return Character.isDigit(c);
        }

        public String toString() {
            return "CharMatcher.JAVA_DIGIT";
        }
    };
    public static final CharMatcher JAVA_ISO_CONTROL = inRange(0, 31).or(inRange(Ascii.MAX, 159)).withToString("CharMatcher.JAVA_ISO_CONTROL");
    public static final CharMatcher JAVA_LETTER = new CharMatcher() {
        public /* bridge */ /* synthetic */ boolean apply(Object x0) {
            return CharMatcher.super.apply((Character) x0);
        }

        public boolean matches(char c) {
            return Character.isLetter(c);
        }

        public String toString() {
            return "CharMatcher.JAVA_LETTER";
        }
    };
    public static final CharMatcher JAVA_LETTER_OR_DIGIT = new CharMatcher() {
        public /* bridge */ /* synthetic */ boolean apply(Object x0) {
            return CharMatcher.super.apply((Character) x0);
        }

        public boolean matches(char c) {
            return Character.isLetterOrDigit(c);
        }

        public String toString() {
            return "CharMatcher.JAVA_LETTER_OR_DIGIT";
        }
    };
    public static final CharMatcher JAVA_LOWER_CASE = new CharMatcher() {
        public /* bridge */ /* synthetic */ boolean apply(Object x0) {
            return CharMatcher.super.apply((Character) x0);
        }

        public boolean matches(char c) {
            return Character.isLowerCase(c);
        }

        public String toString() {
            return "CharMatcher.JAVA_LOWER_CASE";
        }
    };
    public static final CharMatcher JAVA_UPPER_CASE = new CharMatcher() {
        public /* bridge */ /* synthetic */ boolean apply(Object x0) {
            return CharMatcher.super.apply((Character) x0);
        }

        public boolean matches(char c) {
            return Character.isUpperCase(c);
        }

        public String toString() {
            return "CharMatcher.JAVA_UPPER_CASE";
        }
    };
    public static final CharMatcher NONE = new CharMatcher() {
        public /* bridge */ /* synthetic */ boolean apply(Object x0) {
            return CharMatcher.super.apply((Character) x0);
        }

        public boolean matches(char c) {
            return false;
        }

        public int indexIn(CharSequence sequence) {
            Preconditions.checkNotNull(sequence);
            return -1;
        }

        public int indexIn(CharSequence sequence, int start) {
            Preconditions.checkPositionIndex(start, sequence.length());
            return -1;
        }

        public int lastIndexIn(CharSequence sequence) {
            Preconditions.checkNotNull(sequence);
            return -1;
        }

        public boolean matchesAllOf(CharSequence sequence) {
            return sequence.length() == 0;
        }

        public boolean matchesNoneOf(CharSequence sequence) {
            Preconditions.checkNotNull(sequence);
            return true;
        }

        public String removeFrom(CharSequence sequence) {
            return sequence.toString();
        }

        public String replaceFrom(CharSequence sequence, char replacement) {
            return sequence.toString();
        }

        public String replaceFrom(CharSequence sequence, CharSequence replacement) {
            Preconditions.checkNotNull(replacement);
            return sequence.toString();
        }

        public String collapseFrom(CharSequence sequence, char replacement) {
            return sequence.toString();
        }

        public String trimFrom(CharSequence sequence) {
            return sequence.toString();
        }

        public int countIn(CharSequence sequence) {
            Preconditions.checkNotNull(sequence);
            return 0;
        }

        public CharMatcher and(CharMatcher other) {
            Preconditions.checkNotNull(other);
            return this;
        }

        public CharMatcher or(CharMatcher other) {
            return (CharMatcher) Preconditions.checkNotNull(other);
        }

        public CharMatcher negate() {
            return ANY;
        }

        /* access modifiers changed from: package-private */
        public void setBits(LookupTable table) {
        }

        public CharMatcher precomputed() {
            return this;
        }

        public String toString() {
            return "CharMatcher.NONE";
        }
    };
    public static final CharMatcher SINGLE_WIDTH = inRange(0, 1273).or(is(1470)).or(inRange(1488, 1514)).or(is(1523)).or(is(1524)).or(inRange(1536, 1791)).or(inRange(1872, 1919)).or(inRange(3584, 3711)).or(inRange(7680, 8367)).or(inRange(8448, 8506)).or(inRange(64336, 65023)).or(inRange(65136, 65279)).or(inRange(65377, 65500)).withToString("CharMatcher.SINGLE_WIDTH").precomputed();
    public static final CharMatcher WHITESPACE = new CharMatcher() {
        private final char[] table = {1, 0, 160, 0, 0, 0, 0, 0, 0, 9, 10, 11, 12, CharUtils.CR, 0, 0, 8232, 8233, 0, 0, 0, 0, 0, 8239, 0, 0, 0, 0, 0, 0, 0, 0, ' ', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 12288, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 133, 8192, 8193, 8194, 8195, 8196, 8197, 8198, 8199, 8200, 8201, 8202, 0, 0, 0, 0, 0, 8287, 5760, 0, 0, 6158, 0, 0, 0};

        public /* bridge */ /* synthetic */ boolean apply(Object x0) {
            return CharMatcher.super.apply((Character) x0);
        }

        public boolean matches(char c) {
            return this.table[c % 'O'] == c;
        }

        public CharMatcher precomputed() {
            return this;
        }

        public String toString() {
            return "CharMatcher.WHITESPACE";
        }
    };

    public abstract boolean matches(char c);

    static {
        CharMatcher digit = inRange('0', '9');
        CharMatcher digit2 = digit;
        for (char base : "٠۰߀०০੦૦୦௦౦೦൦๐໐༠၀႐០᠐᥆᧐᭐᮰᱀᱐꘠꣐꤀꩐０".toCharArray()) {
            digit2 = digit2.or(inRange(base, (char) (base + 9)));
        }
        DIGIT = digit2.withToString("CharMatcher.DIGIT").precomputed();
    }

    public static CharMatcher is(final char match) {
        return new CharMatcher() {
            public /* bridge */ /* synthetic */ boolean apply(Object x0) {
                return CharMatcher.super.apply((Character) x0);
            }

            public boolean matches(char c) {
                return c == match;
            }

            public String replaceFrom(CharSequence sequence, char replacement) {
                return sequence.toString().replace(match, replacement);
            }

            public CharMatcher and(CharMatcher other) {
                return other.matches(match) ? this : NONE;
            }

            public CharMatcher or(CharMatcher other) {
                return other.matches(match) ? other : CharMatcher.super.or(other);
            }

            public CharMatcher negate() {
                return isNot(match);
            }

            /* access modifiers changed from: package-private */
            public void setBits(LookupTable table) {
                table.set(match);
            }

            public CharMatcher precomputed() {
                return this;
            }

            public String toString() {
                return "CharMatcher.is(" + Integer.toHexString(match) + ")";
            }
        };
    }

    public static CharMatcher isNot(final char match) {
        return new CharMatcher() {
            public /* bridge */ /* synthetic */ boolean apply(Object x0) {
                return CharMatcher.super.apply((Character) x0);
            }

            public boolean matches(char c) {
                return c != match;
            }

            public CharMatcher and(CharMatcher other) {
                return other.matches(match) ? CharMatcher.super.and(other) : other;
            }

            public CharMatcher or(CharMatcher other) {
                return other.matches(match) ? ANY : this;
            }

            public CharMatcher negate() {
                return is(match);
            }

            public String toString() {
                return "CharMatcher.isNot(" + Integer.toHexString(match) + ")";
            }
        };
    }

    public static CharMatcher anyOf(CharSequence sequence) {
        switch (sequence.length()) {
            case 0:
                return NONE;
            case 1:
                return is(sequence.charAt(0));
            case 2:
                final char match1 = sequence.charAt(0);
                final char match2 = sequence.charAt(1);
                return new CharMatcher() {
                    public /* bridge */ /* synthetic */ boolean apply(Object x0) {
                        return CharMatcher.super.apply((Character) x0);
                    }

                    public boolean matches(char c) {
                        return c == match1 || c == match2;
                    }

                    /* access modifiers changed from: package-private */
                    public void setBits(LookupTable table) {
                        table.set(match1);
                        table.set(match2);
                    }

                    public CharMatcher precomputed() {
                        return this;
                    }
                };
            default:
                final char[] chars = sequence.toString().toCharArray();
                Arrays.sort(chars);
                return new CharMatcher() {
                    public /* bridge */ /* synthetic */ boolean apply(Object x0) {
                        return CharMatcher.super.apply((Character) x0);
                    }

                    public boolean matches(char c) {
                        return Arrays.binarySearch(chars, c) >= 0;
                    }

                    /* access modifiers changed from: package-private */
                    public void setBits(LookupTable table) {
                        for (char c : chars) {
                            table.set(c);
                        }
                    }

                    public String toString() {
                        return "CharMatcher.anyOf(\"" + chars + "\")";
                    }
                };
        }
    }

    public static CharMatcher noneOf(CharSequence sequence) {
        return anyOf(sequence).negate();
    }

    public static CharMatcher inRange(final char startInclusive, final char endInclusive) {
        Preconditions.checkArgument(endInclusive >= startInclusive);
        return new CharMatcher() {
            public /* bridge */ /* synthetic */ boolean apply(Object x0) {
                return CharMatcher.super.apply((Character) x0);
            }

            public boolean matches(char c) {
                return startInclusive <= c && c <= endInclusive;
            }

            /* access modifiers changed from: package-private */
            public void setBits(LookupTable table) {
                char c = startInclusive;
                while (true) {
                    table.set(c);
                    char c2 = (char) (c + 1);
                    if (c != endInclusive) {
                        c = c2;
                    } else {
                        return;
                    }
                }
            }

            public CharMatcher precomputed() {
                return this;
            }

            public String toString() {
                return "CharMatcher.inRange(" + Integer.toHexString(startInclusive) + ", " + Integer.toHexString(endInclusive) + ")";
            }
        };
    }

    public static CharMatcher forPredicate(final Predicate<? super Character> predicate) {
        Preconditions.checkNotNull(predicate);
        if (predicate instanceof CharMatcher) {
            return (CharMatcher) predicate;
        }
        return new CharMatcher() {
            public boolean matches(char c) {
                return predicate.apply(Character.valueOf(c));
            }

            public boolean apply(Character character) {
                return predicate.apply(Preconditions.checkNotNull(character));
            }

            public String toString() {
                return "CharMatcher.forPredicate(" + predicate + ')';
            }
        };
    }

    protected CharMatcher() {
    }

    public CharMatcher negate() {
        return new CharMatcher() {
            public /* bridge */ /* synthetic */ boolean apply(Object x0) {
                return CharMatcher.super.apply((Character) x0);
            }

            public boolean matches(char c) {
                return !this.matches(c);
            }

            public boolean matchesAllOf(CharSequence sequence) {
                return this.matchesNoneOf(sequence);
            }

            public boolean matchesNoneOf(CharSequence sequence) {
                return this.matchesAllOf(sequence);
            }

            public int countIn(CharSequence sequence) {
                return sequence.length() - this.countIn(sequence);
            }

            public CharMatcher negate() {
                return this;
            }

            public String toString() {
                return this + ".negate()";
            }
        };
    }

    public CharMatcher and(CharMatcher other) {
        return new And(Arrays.asList(new CharMatcher[]{this, (CharMatcher) Preconditions.checkNotNull(other)}));
    }

    private static class And extends CharMatcher {
        List<CharMatcher> components;

        public /* bridge */ /* synthetic */ boolean apply(Object x0) {
            return CharMatcher.super.apply((Character) x0);
        }

        And(List<CharMatcher> components2) {
            this.components = components2;
        }

        public boolean matches(char c) {
            for (CharMatcher matcher : this.components) {
                if (!matcher.matches(c)) {
                    return false;
                }
            }
            return true;
        }

        public CharMatcher and(CharMatcher other) {
            List<CharMatcher> newComponents = new ArrayList<>(this.components);
            newComponents.add(Preconditions.checkNotNull(other));
            return new And(newComponents);
        }

        public String toString() {
            StringBuilder builder = new StringBuilder("CharMatcher.and(");
            Joiner.on(", ").appendTo(builder, (Iterable<?>) this.components);
            builder.append(')');
            return builder.toString();
        }
    }

    public CharMatcher or(CharMatcher other) {
        return new Or(Arrays.asList(new CharMatcher[]{this, (CharMatcher) Preconditions.checkNotNull(other)}));
    }

    private static class Or extends CharMatcher {
        List<CharMatcher> components;

        public /* bridge */ /* synthetic */ boolean apply(Object x0) {
            return CharMatcher.super.apply((Character) x0);
        }

        Or(List<CharMatcher> components2) {
            this.components = components2;
        }

        public boolean matches(char c) {
            for (CharMatcher matcher : this.components) {
                if (matcher.matches(c)) {
                    return true;
                }
            }
            return false;
        }

        public CharMatcher or(CharMatcher other) {
            List<CharMatcher> newComponents = new ArrayList<>(this.components);
            newComponents.add(Preconditions.checkNotNull(other));
            return new Or(newComponents);
        }

        /* access modifiers changed from: package-private */
        public void setBits(LookupTable table) {
            for (CharMatcher matcher : this.components) {
                matcher.setBits(table);
            }
        }

        public String toString() {
            StringBuilder builder = new StringBuilder("CharMatcher.or(");
            Joiner.on(", ").appendTo(builder, (Iterable<?>) this.components);
            builder.append(')');
            return builder.toString();
        }
    }

    public CharMatcher precomputed() {
        return Platform.precomputeCharMatcher(this);
    }

    /* access modifiers changed from: package-private */
    public CharMatcher precomputedInternal() {
        final LookupTable table = new LookupTable();
        setBits(table);
        return new CharMatcher() {
            public /* bridge */ /* synthetic */ boolean apply(Object x0) {
                return CharMatcher.super.apply((Character) x0);
            }

            public boolean matches(char c) {
                return table.get(c);
            }

            public CharMatcher precomputed() {
                return this;
            }

            public String toString() {
                return this.toString();
            }
        };
    }

    /* access modifiers changed from: package-private */
    public CharMatcher withToString(final String toString) {
        return new CharMatcher() {
            public /* bridge */ /* synthetic */ boolean apply(Object x0) {
                return CharMatcher.super.apply((Character) x0);
            }

            public boolean matches(char c) {
                return this.matches(c);
            }

            /* access modifiers changed from: package-private */
            public void setBits(LookupTable table) {
                this.setBits(table);
            }

            public String toString() {
                return toString;
            }
        };
    }

    /* access modifiers changed from: package-private */
    public void setBits(LookupTable table) {
        char c = 0;
        while (true) {
            if (matches(c)) {
                table.set(c);
            }
            char c2 = (char) (c + 1);
            if (c != 65535) {
                c = c2;
            } else {
                return;
            }
        }
    }

    private static final class LookupTable {
        int[] data;

        private LookupTable() {
            this.data = new int[2048];
        }

        /* access modifiers changed from: package-private */
        public void set(char index) {
            int[] iArr = this.data;
            int i = index >> 5;
            iArr[i] = iArr[i] | (1 << index);
        }

        /* access modifiers changed from: package-private */
        public boolean get(char index) {
            return (this.data[index >> 5] & (1 << index)) != 0;
        }
    }

    public boolean matchesAnyOf(CharSequence sequence) {
        return !matchesNoneOf(sequence);
    }

    public boolean matchesAllOf(CharSequence sequence) {
        for (int i = sequence.length() - 1; i >= 0; i--) {
            if (!matches(sequence.charAt(i))) {
                return false;
            }
        }
        return true;
    }

    public boolean matchesNoneOf(CharSequence sequence) {
        return indexIn(sequence) == -1;
    }

    public int indexIn(CharSequence sequence) {
        int length = sequence.length();
        for (int i = 0; i < length; i++) {
            if (matches(sequence.charAt(i))) {
                return i;
            }
        }
        return -1;
    }

    public int indexIn(CharSequence sequence, int start) {
        int length = sequence.length();
        Preconditions.checkPositionIndex(start, length);
        for (int i = start; i < length; i++) {
            if (matches(sequence.charAt(i))) {
                return i;
            }
        }
        return -1;
    }

    public int lastIndexIn(CharSequence sequence) {
        for (int i = sequence.length() - 1; i >= 0; i--) {
            if (matches(sequence.charAt(i))) {
                return i;
            }
        }
        return -1;
    }

    public int countIn(CharSequence sequence) {
        int count = 0;
        for (int i = 0; i < sequence.length(); i++) {
            if (matches(sequence.charAt(i))) {
                count++;
            }
        }
        return count;
    }

    @CheckReturnValue
    public String removeFrom(CharSequence sequence) {
        String string = sequence.toString();
        int pos = indexIn(string);
        if (pos == -1) {
            return string;
        }
        char[] chars = string.toCharArray();
        int pos2 = pos;
        int spread = 1;
        while (true) {
            pos2++;
            while (pos2 != chars.length) {
                if (matches(chars[pos2])) {
                    spread++;
                } else {
                    chars[pos2 - spread] = chars[pos2];
                    pos2++;
                }
            }
            return new String(chars, 0, pos2 - spread);
        }
    }

    @CheckReturnValue
    public String retainFrom(CharSequence sequence) {
        return negate().removeFrom(sequence);
    }

    @CheckReturnValue
    public String replaceFrom(CharSequence sequence, char replacement) {
        String string = sequence.toString();
        int pos = indexIn(string);
        if (pos == -1) {
            return string;
        }
        char[] chars = string.toCharArray();
        chars[pos] = replacement;
        for (int i = pos + 1; i < chars.length; i++) {
            if (matches(chars[i])) {
                chars[i] = replacement;
            }
        }
        return new String(chars);
    }

    @CheckReturnValue
    public String replaceFrom(CharSequence sequence, CharSequence replacement) {
        int replacementLen = replacement.length();
        if (replacementLen == 0) {
            return removeFrom(sequence);
        }
        int oldpos = 0;
        if (replacementLen == 1) {
            return replaceFrom(sequence, replacement.charAt(0));
        }
        String string = sequence.toString();
        int pos = indexIn(string);
        if (pos == -1) {
            return string;
        }
        int len = string.length();
        StringBuilder buf = new StringBuilder(((len * 3) / 2) + 16);
        do {
            buf.append(string, oldpos, pos);
            buf.append(replacement);
            oldpos = pos + 1;
            pos = indexIn(string, oldpos);
        } while (pos != -1);
        buf.append(string, oldpos, len);
        return buf.toString();
    }

    @CheckReturnValue
    public String trimFrom(CharSequence sequence) {
        int len = sequence.length();
        int first = 0;
        while (first < len && matches(sequence.charAt(first))) {
            first++;
        }
        int last = len - 1;
        while (last > first && matches(sequence.charAt(last))) {
            last--;
        }
        return sequence.subSequence(first, last + 1).toString();
    }

    @CheckReturnValue
    public String trimLeadingFrom(CharSequence sequence) {
        int len = sequence.length();
        int first = 0;
        while (first < len && matches(sequence.charAt(first))) {
            first++;
        }
        return sequence.subSequence(first, len).toString();
    }

    @CheckReturnValue
    public String trimTrailingFrom(CharSequence sequence) {
        int last = sequence.length() - 1;
        while (last >= 0 && matches(sequence.charAt(last))) {
            last--;
        }
        return sequence.subSequence(0, last + 1).toString();
    }

    @CheckReturnValue
    public String collapseFrom(CharSequence sequence, char replacement) {
        int first = indexIn(sequence);
        if (first == -1) {
            return sequence.toString();
        }
        StringBuilder sb = new StringBuilder(sequence.length());
        sb.append(sequence.subSequence(0, first));
        StringBuilder builder = sb.append(replacement);
        boolean in = true;
        for (int i = first + 1; i < sequence.length(); i++) {
            char c = sequence.charAt(i);
            if (!matches(c)) {
                builder.append(c);
                in = false;
            } else if (!in) {
                builder.append(replacement);
                in = true;
            }
        }
        return builder.toString();
    }

    @CheckReturnValue
    public String trimAndCollapseFrom(CharSequence sequence, char replacement) {
        int first = negate().indexIn(sequence);
        if (first == -1) {
            return "";
        }
        StringBuilder builder = new StringBuilder(sequence.length());
        boolean inMatchingGroup = false;
        for (int i = first; i < sequence.length(); i++) {
            char c = sequence.charAt(i);
            if (matches(c)) {
                inMatchingGroup = true;
            } else {
                if (inMatchingGroup) {
                    builder.append(replacement);
                    inMatchingGroup = false;
                }
                builder.append(c);
            }
        }
        return builder.toString();
    }

    public boolean apply(Character character) {
        return matches(character.charValue());
    }

    public String toString() {
        return super.toString();
    }
}
