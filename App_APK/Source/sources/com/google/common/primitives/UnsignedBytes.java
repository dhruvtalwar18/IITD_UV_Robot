package com.google.common.primitives;

import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Preconditions;
import java.lang.reflect.Field;
import java.nio.ByteOrder;
import java.security.AccessController;
import java.security.PrivilegedAction;
import java.util.Comparator;
import sensor_msgs.NavSatStatus;
import sun.misc.Unsafe;

public final class UnsignedBytes {
    public static final byte MAX_POWER_OF_TWO = Byte.MIN_VALUE;

    private UnsignedBytes() {
    }

    public static int toInt(byte value) {
        return value & NavSatStatus.STATUS_NO_FIX;
    }

    public static byte checkedCast(long value) {
        Preconditions.checkArgument((value >> 8) == 0, "out of range: %s", Long.valueOf(value));
        return (byte) ((int) value);
    }

    public static byte saturatedCast(long value) {
        if (value > 255) {
            return -1;
        }
        if (value < 0) {
            return 0;
        }
        return (byte) ((int) value);
    }

    public static int compare(byte a, byte b) {
        return toInt(a) - toInt(b);
    }

    public static byte min(byte... array) {
        int next = 1;
        Preconditions.checkArgument(array.length > 0);
        int min = toInt(array[0]);
        while (true) {
            int i = next;
            if (i >= array.length) {
                return (byte) min;
            }
            int next2 = toInt(array[i]);
            if (next2 < min) {
                min = next2;
            }
            next = i + 1;
        }
    }

    public static byte max(byte... array) {
        int next = 1;
        Preconditions.checkArgument(array.length > 0);
        int max = toInt(array[0]);
        while (true) {
            int i = next;
            if (i >= array.length) {
                return (byte) max;
            }
            int next2 = toInt(array[i]);
            if (next2 > max) {
                max = next2;
            }
            next = i + 1;
        }
    }

    public static String join(String separator, byte... array) {
        Preconditions.checkNotNull(separator);
        if (array.length == 0) {
            return "";
        }
        StringBuilder builder = new StringBuilder(array.length * 5);
        builder.append(toInt(array[0]));
        for (int i = 1; i < array.length; i++) {
            builder.append(separator);
            builder.append(toInt(array[i]));
        }
        return builder.toString();
    }

    public static Comparator<byte[]> lexicographicalComparator() {
        return LexicographicalComparatorHolder.BEST_COMPARATOR;
    }

    @VisibleForTesting
    static Comparator<byte[]> lexicographicalComparatorJavaImpl() {
        return LexicographicalComparatorHolder.PureJavaComparator.INSTANCE;
    }

    @VisibleForTesting
    static class LexicographicalComparatorHolder {
        static final Comparator<byte[]> BEST_COMPARATOR = getBestComparator();
        static final String UNSAFE_COMPARATOR_NAME = (LexicographicalComparatorHolder.class.getName() + "$UnsafeComparator");

        LexicographicalComparatorHolder() {
        }

        @VisibleForTesting
        enum UnsafeComparator implements Comparator<byte[]> {
            INSTANCE;
            
            static final int BYTE_ARRAY_BASE_OFFSET = 0;
            static final boolean littleEndian = false;
            static final Unsafe theUnsafe = null;

            static {
                littleEndian = ByteOrder.nativeOrder().equals(ByteOrder.LITTLE_ENDIAN);
                theUnsafe = (Unsafe) AccessController.doPrivileged(new PrivilegedAction<Object>() {
                    public Object run() {
                        try {
                            Field f = Unsafe.class.getDeclaredField("theUnsafe");
                            f.setAccessible(true);
                            return f.get((Object) null);
                        } catch (NoSuchFieldException e) {
                            throw new Error();
                        } catch (IllegalAccessException e2) {
                            throw new Error();
                        }
                    }
                });
                BYTE_ARRAY_BASE_OFFSET = theUnsafe.arrayBaseOffset(byte[].class);
                if (theUnsafe.arrayIndexScale(byte[].class) != 1) {
                    throw new AssertionError();
                }
            }

            public int compare(byte[] left, byte[] right) {
                byte[] bArr = left;
                byte[] bArr2 = right;
                int minLength = Math.min(bArr.length, bArr2.length);
                int minWords = minLength / 8;
                int i = 0;
                while (i < minWords * 8) {
                    long lw = theUnsafe.getLong(bArr, ((long) BYTE_ARRAY_BASE_OFFSET) + ((long) i));
                    long rw = theUnsafe.getLong(bArr2, ((long) BYTE_ARRAY_BASE_OFFSET) + ((long) i));
                    long diff = lw ^ rw;
                    if (diff == 0) {
                        i += 8;
                    } else if (!littleEndian) {
                        return UnsignedLongs.compare(lw, rw);
                    } else {
                        int n = 0;
                        int x = (int) diff;
                        if (x == 0) {
                            x = (int) (diff >>> 32);
                            n = 32;
                        }
                        int y = x << 16;
                        if (y == 0) {
                            n += 16;
                        } else {
                            x = y;
                        }
                        if ((x << 8) == 0) {
                            n += 8;
                        }
                        return (int) (((lw >>> n) & 255) - ((rw >>> n) & 255));
                    }
                }
                for (int i2 = minWords * 8; i2 < minLength; i2++) {
                    int result = UnsignedBytes.compare(bArr[i2], bArr2[i2]);
                    if (result != 0) {
                        return result;
                    }
                }
                return bArr.length - bArr2.length;
            }
        }

        enum PureJavaComparator implements Comparator<byte[]> {
            INSTANCE;

            public int compare(byte[] left, byte[] right) {
                int minLength = Math.min(left.length, right.length);
                for (int i = 0; i < minLength; i++) {
                    int result = UnsignedBytes.compare(left[i], right[i]);
                    if (result != 0) {
                        return result;
                    }
                }
                return left.length - right.length;
            }
        }

        static Comparator<byte[]> getBestComparator() {
            try {
                return (Comparator) Class.forName(UNSAFE_COMPARATOR_NAME).getEnumConstants()[0];
            } catch (Throwable th) {
                return UnsignedBytes.lexicographicalComparatorJavaImpl();
            }
        }
    }
}
