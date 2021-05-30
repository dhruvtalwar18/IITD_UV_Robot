package com.google.common.primitives;

import com.google.common.base.Preconditions;
import javax.annotation.CheckForNull;

final class AndroidInteger {
    @CheckForNull
    static Integer tryParse(String string) {
        return tryParse(string, 10);
    }

    @CheckForNull
    static Integer tryParse(String string, int radix) {
        Preconditions.checkNotNull(string);
        boolean negative = false;
        Preconditions.checkArgument(radix >= 2, "Invalid radix %s, min radix is %s", Integer.valueOf(radix), 2);
        Preconditions.checkArgument(radix <= 36, "Invalid radix %s, max radix is %s", Integer.valueOf(radix), 36);
        int length = string.length();
        int i = 0;
        if (length == 0) {
            return null;
        }
        if (string.charAt(0) == '-') {
            negative = true;
        }
        if (!negative || (i = 0 + 1) != length) {
            return tryParse(string, i, radix, negative);
        }
        return null;
    }

    @CheckForNull
    private static Integer tryParse(String string, int offset, int radix, boolean negative) {
        int next;
        int max = Integer.MIN_VALUE / radix;
        int result = 0;
        int length = string.length();
        while (offset < length) {
            int offset2 = offset + 1;
            int offset3 = Character.digit(string.charAt(offset), radix);
            if (offset3 == -1 || max > result || (next = (result * radix) - offset3) > result) {
                return null;
            }
            result = next;
            offset = offset2;
        }
        if ((negative || (result = -result) >= 0) && result <= Integer.MAX_VALUE && result >= Integer.MIN_VALUE) {
            return Integer.valueOf(result);
        }
        return null;
    }

    private AndroidInteger() {
    }
}
