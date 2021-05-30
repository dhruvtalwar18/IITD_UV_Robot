package org.apache.commons.lang.math;

import java.math.BigDecimal;
import java.math.BigInteger;
import org.apache.commons.lang.StringUtils;
import org.bytedeco.javacpp.opencv_stitching;

public class NumberUtils {
    public static final Byte BYTE_MINUS_ONE = new Byte((byte) -1);
    public static final Byte BYTE_ONE = new Byte((byte) 1);
    public static final Byte BYTE_ZERO = new Byte((byte) 0);
    public static final Double DOUBLE_MINUS_ONE = new Double(-1.0d);
    public static final Double DOUBLE_ONE = new Double(1.0d);
    public static final Double DOUBLE_ZERO = new Double(opencv_stitching.Stitcher.ORIG_RESOL);
    public static final Float FLOAT_MINUS_ONE = new Float(-1.0f);
    public static final Float FLOAT_ONE = new Float(1.0f);
    public static final Float FLOAT_ZERO = new Float(0.0f);
    public static final Integer INTEGER_MINUS_ONE = new Integer(-1);
    public static final Integer INTEGER_ONE = new Integer(1);
    public static final Integer INTEGER_ZERO = new Integer(0);
    public static final Long LONG_MINUS_ONE = new Long(-1);
    public static final Long LONG_ONE = new Long(1);
    public static final Long LONG_ZERO = new Long(0);
    public static final Short SHORT_MINUS_ONE = new Short(-1);
    public static final Short SHORT_ONE = new Short(1);
    public static final Short SHORT_ZERO = new Short(0);

    public static int stringToInt(String str) {
        return toInt(str);
    }

    public static int toInt(String str) {
        return toInt(str, 0);
    }

    public static int stringToInt(String str, int defaultValue) {
        return toInt(str, defaultValue);
    }

    public static int toInt(String str, int defaultValue) {
        if (str == null) {
            return defaultValue;
        }
        try {
            return Integer.parseInt(str);
        } catch (NumberFormatException e) {
            return defaultValue;
        }
    }

    public static long toLong(String str) {
        return toLong(str, 0);
    }

    public static long toLong(String str, long defaultValue) {
        if (str == null) {
            return defaultValue;
        }
        try {
            return Long.parseLong(str);
        } catch (NumberFormatException e) {
            return defaultValue;
        }
    }

    public static float toFloat(String str) {
        return toFloat(str, 0.0f);
    }

    public static float toFloat(String str, float defaultValue) {
        if (str == null) {
            return defaultValue;
        }
        try {
            return Float.parseFloat(str);
        } catch (NumberFormatException e) {
            return defaultValue;
        }
    }

    public static double toDouble(String str) {
        return toDouble(str, opencv_stitching.Stitcher.ORIG_RESOL);
    }

    public static double toDouble(String str, double defaultValue) {
        if (str == null) {
            return defaultValue;
        }
        try {
            return Double.parseDouble(str);
        } catch (NumberFormatException e) {
            return defaultValue;
        }
    }

    /* JADX WARNING: Code restructure failed: missing block: B:48:0x00cb, code lost:
        if (r1 == 'l') goto L_0x00cd;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public static java.lang.Number createNumber(java.lang.String r15) throws java.lang.NumberFormatException {
        /*
            r0 = 0
            if (r15 != 0) goto L_0x0004
            return r0
        L_0x0004:
            boolean r1 = org.apache.commons.lang.StringUtils.isBlank(r15)
            if (r1 != 0) goto L_0x01c7
            java.lang.String r1 = "--"
            boolean r1 = r15.startsWith(r1)
            if (r1 == 0) goto L_0x0013
            return r0
        L_0x0013:
            java.lang.String r1 = "0x"
            boolean r1 = r15.startsWith(r1)
            if (r1 != 0) goto L_0x01c2
            java.lang.String r1 = "-0x"
            boolean r1 = r15.startsWith(r1)
            if (r1 == 0) goto L_0x0025
            goto L_0x01c2
        L_0x0025:
            int r1 = r15.length()
            r2 = 1
            int r1 = r1 - r2
            char r1 = r15.charAt(r1)
            r3 = 46
            int r3 = r15.indexOf(r3)
            r4 = 101(0x65, float:1.42E-43)
            int r4 = r15.indexOf(r4)
            r5 = 69
            int r5 = r15.indexOf(r5)
            int r4 = r4 + r5
            int r4 = r4 + r2
            r5 = -1
            r6 = 0
            if (r3 <= r5) goto L_0x0074
            if (r4 <= r5) goto L_0x0069
            if (r4 < r3) goto L_0x0052
            int r7 = r3 + 1
            java.lang.String r7 = r15.substring(r7, r4)
            goto L_0x006f
        L_0x0052:
            java.lang.NumberFormatException r0 = new java.lang.NumberFormatException
            java.lang.StringBuffer r2 = new java.lang.StringBuffer
            r2.<init>()
            r2.append(r15)
            java.lang.String r5 = " is not a valid number."
            r2.append(r5)
            java.lang.String r2 = r2.toString()
            r0.<init>(r2)
            throw r0
        L_0x0069:
            int r7 = r3 + 1
            java.lang.String r7 = r15.substring(r7)
        L_0x006f:
            java.lang.String r8 = r15.substring(r6, r3)
            goto L_0x007e
        L_0x0074:
            if (r4 <= r5) goto L_0x007b
            java.lang.String r7 = r15.substring(r6, r4)
            goto L_0x007c
        L_0x007b:
            r7 = r15
        L_0x007c:
            r8 = r7
            r7 = r0
        L_0x007e:
            boolean r9 = java.lang.Character.isDigit(r1)
            r10 = 0
            r11 = 0
            if (r9 != 0) goto L_0x0157
            if (r4 <= r5) goto L_0x009c
            int r5 = r15.length()
            int r5 = r5 - r2
            if (r4 >= r5) goto L_0x009c
            int r0 = r4 + 1
            int r5 = r15.length()
            int r5 = r5 - r2
            java.lang.String r0 = r15.substring(r0, r5)
            goto L_0x009d
        L_0x009c:
        L_0x009d:
            int r5 = r15.length()
            int r5 = r5 - r2
            java.lang.String r5 = r15.substring(r6, r5)
            boolean r9 = isAllZeros(r8)
            if (r9 == 0) goto L_0x00b4
            boolean r9 = isAllZeros(r0)
            if (r9 == 0) goto L_0x00b4
            r9 = 1
            goto L_0x00b5
        L_0x00b4:
            r9 = 0
        L_0x00b5:
            r13 = 68
            if (r1 == r13) goto L_0x0122
            r13 = 70
            if (r1 == r13) goto L_0x010b
            r13 = 76
            if (r1 == r13) goto L_0x00cd
            r13 = 100
            if (r1 == r13) goto L_0x0122
            r13 = 102(0x66, float:1.43E-43)
            if (r1 == r13) goto L_0x010b
            r10 = 108(0x6c, float:1.51E-43)
            if (r1 != r10) goto L_0x0140
        L_0x00cd:
            if (r7 != 0) goto L_0x00f4
            if (r0 != 0) goto L_0x00f4
            char r6 = r5.charAt(r6)
            r10 = 45
            if (r6 != r10) goto L_0x00e3
            java.lang.String r2 = r5.substring(r2)
            boolean r2 = isDigits(r2)
            if (r2 != 0) goto L_0x00e9
        L_0x00e3:
            boolean r2 = isDigits(r5)
            if (r2 == 0) goto L_0x00f4
        L_0x00e9:
            java.lang.Long r2 = createLong(r5)     // Catch:{ NumberFormatException -> 0x00ee }
            return r2
        L_0x00ee:
            r2 = move-exception
            java.math.BigInteger r2 = createBigInteger(r5)
            return r2
        L_0x00f4:
            java.lang.NumberFormatException r2 = new java.lang.NumberFormatException
            java.lang.StringBuffer r6 = new java.lang.StringBuffer
            r6.<init>()
            r6.append(r15)
            java.lang.String r10 = " is not a valid number."
            r6.append(r10)
            java.lang.String r6 = r6.toString()
            r2.<init>(r6)
            throw r2
        L_0x010b:
            java.lang.Float r2 = createFloat(r5)     // Catch:{ NumberFormatException -> 0x0121 }
            boolean r6 = r2.isInfinite()     // Catch:{ NumberFormatException -> 0x0121 }
            if (r6 != 0) goto L_0x0120
            float r6 = r2.floatValue()     // Catch:{ NumberFormatException -> 0x0121 }
            int r6 = (r6 > r10 ? 1 : (r6 == r10 ? 0 : -1))
            if (r6 != 0) goto L_0x011f
            if (r9 == 0) goto L_0x0120
        L_0x011f:
            return r2
        L_0x0120:
            goto L_0x0122
        L_0x0121:
            r2 = move-exception
        L_0x0122:
            java.lang.Double r2 = createDouble(r5)     // Catch:{ NumberFormatException -> 0x0139 }
            boolean r6 = r2.isInfinite()     // Catch:{ NumberFormatException -> 0x0139 }
            if (r6 != 0) goto L_0x0138
            float r6 = r2.floatValue()     // Catch:{ NumberFormatException -> 0x0139 }
            double r13 = (double) r6
            int r6 = (r13 > r11 ? 1 : (r13 == r11 ? 0 : -1))
            if (r6 != 0) goto L_0x0137
            if (r9 == 0) goto L_0x0138
        L_0x0137:
            return r2
        L_0x0138:
            goto L_0x013a
        L_0x0139:
            r2 = move-exception
        L_0x013a:
            java.math.BigDecimal r2 = createBigDecimal(r5)     // Catch:{ NumberFormatException -> 0x013f }
            return r2
        L_0x013f:
            r2 = move-exception
        L_0x0140:
            java.lang.NumberFormatException r2 = new java.lang.NumberFormatException
            java.lang.StringBuffer r6 = new java.lang.StringBuffer
            r6.<init>()
            r6.append(r15)
            java.lang.String r10 = " is not a valid number."
            r6.append(r10)
            java.lang.String r6 = r6.toString()
            r2.<init>(r6)
            throw r2
        L_0x0157:
            if (r4 <= r5) goto L_0x016b
            int r5 = r15.length()
            int r5 = r5 - r2
            if (r4 >= r5) goto L_0x016b
            int r0 = r4 + 1
            int r5 = r15.length()
            java.lang.String r0 = r15.substring(r0, r5)
            goto L_0x016c
        L_0x016b:
        L_0x016c:
            if (r7 != 0) goto L_0x0181
            if (r0 != 0) goto L_0x0181
            java.lang.Integer r2 = createInteger(r15)     // Catch:{ NumberFormatException -> 0x0175 }
            return r2
        L_0x0175:
            r2 = move-exception
            java.lang.Long r2 = createLong(r15)     // Catch:{ NumberFormatException -> 0x017b }
            return r2
        L_0x017b:
            r2 = move-exception
            java.math.BigInteger r2 = createBigInteger(r15)
            return r2
        L_0x0181:
            boolean r5 = isAllZeros(r8)
            if (r5 == 0) goto L_0x018e
            boolean r5 = isAllZeros(r0)
            if (r5 == 0) goto L_0x018e
            goto L_0x018f
        L_0x018e:
            r2 = 0
        L_0x018f:
            java.lang.Float r5 = createFloat(r15)     // Catch:{ NumberFormatException -> 0x01a5 }
            boolean r6 = r5.isInfinite()     // Catch:{ NumberFormatException -> 0x01a5 }
            if (r6 != 0) goto L_0x01a4
            float r6 = r5.floatValue()     // Catch:{ NumberFormatException -> 0x01a5 }
            int r6 = (r6 > r10 ? 1 : (r6 == r10 ? 0 : -1))
            if (r6 != 0) goto L_0x01a3
            if (r2 == 0) goto L_0x01a4
        L_0x01a3:
            return r5
        L_0x01a4:
            goto L_0x01a6
        L_0x01a5:
            r5 = move-exception
        L_0x01a6:
            java.lang.Double r5 = createDouble(r15)     // Catch:{ NumberFormatException -> 0x01bc }
            boolean r6 = r5.isInfinite()     // Catch:{ NumberFormatException -> 0x01bc }
            if (r6 != 0) goto L_0x01bb
            double r9 = r5.doubleValue()     // Catch:{ NumberFormatException -> 0x01bc }
            int r6 = (r9 > r11 ? 1 : (r9 == r11 ? 0 : -1))
            if (r6 != 0) goto L_0x01ba
            if (r2 == 0) goto L_0x01bb
        L_0x01ba:
            return r5
        L_0x01bb:
            goto L_0x01bd
        L_0x01bc:
            r5 = move-exception
        L_0x01bd:
            java.math.BigDecimal r5 = createBigDecimal(r15)
            return r5
        L_0x01c2:
            java.lang.Integer r0 = createInteger(r15)
            return r0
        L_0x01c7:
            java.lang.NumberFormatException r0 = new java.lang.NumberFormatException
            java.lang.String r1 = "A blank string is not a valid number"
            r0.<init>(r1)
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.lang.math.NumberUtils.createNumber(java.lang.String):java.lang.Number");
    }

    private static boolean isAllZeros(String str) {
        if (str == null) {
            return true;
        }
        for (int i = str.length() - 1; i >= 0; i--) {
            if (str.charAt(i) != '0') {
                return false;
            }
        }
        if (str.length() > 0) {
            return true;
        }
        return false;
    }

    public static Float createFloat(String str) {
        if (str == null) {
            return null;
        }
        return Float.valueOf(str);
    }

    public static Double createDouble(String str) {
        if (str == null) {
            return null;
        }
        return Double.valueOf(str);
    }

    public static Integer createInteger(String str) {
        if (str == null) {
            return null;
        }
        return Integer.decode(str);
    }

    public static Long createLong(String str) {
        if (str == null) {
            return null;
        }
        return Long.valueOf(str);
    }

    public static BigInteger createBigInteger(String str) {
        if (str == null) {
            return null;
        }
        return new BigInteger(str);
    }

    public static BigDecimal createBigDecimal(String str) {
        if (str == null) {
            return null;
        }
        if (!StringUtils.isBlank(str)) {
            return new BigDecimal(str);
        }
        throw new NumberFormatException("A blank string is not a valid number");
    }

    public static long min(long[] array) {
        if (array == null) {
            throw new IllegalArgumentException("The Array must not be null");
        } else if (array.length != 0) {
            long min = array[0];
            for (int i = 1; i < array.length; i++) {
                if (array[i] < min) {
                    min = array[i];
                }
            }
            return min;
        } else {
            throw new IllegalArgumentException("Array cannot be empty.");
        }
    }

    public static int min(int[] array) {
        if (array == null) {
            throw new IllegalArgumentException("The Array must not be null");
        } else if (array.length != 0) {
            int min = array[0];
            for (int j = 1; j < array.length; j++) {
                if (array[j] < min) {
                    min = array[j];
                }
            }
            return min;
        } else {
            throw new IllegalArgumentException("Array cannot be empty.");
        }
    }

    public static short min(short[] array) {
        if (array == null) {
            throw new IllegalArgumentException("The Array must not be null");
        } else if (array.length != 0) {
            short min = array[0];
            for (int i = 1; i < array.length; i++) {
                if (array[i] < min) {
                    min = array[i];
                }
            }
            return min;
        } else {
            throw new IllegalArgumentException("Array cannot be empty.");
        }
    }

    public static byte min(byte[] array) {
        if (array == null) {
            throw new IllegalArgumentException("The Array must not be null");
        } else if (array.length != 0) {
            byte min = array[0];
            for (int i = 1; i < array.length; i++) {
                if (array[i] < min) {
                    min = array[i];
                }
            }
            return min;
        } else {
            throw new IllegalArgumentException("Array cannot be empty.");
        }
    }

    public static double min(double[] array) {
        if (array == null) {
            throw new IllegalArgumentException("The Array must not be null");
        } else if (array.length != 0) {
            double min = array[0];
            for (int i = 1; i < array.length; i++) {
                if (Double.isNaN(array[i])) {
                    return Double.NaN;
                }
                if (array[i] < min) {
                    min = array[i];
                }
            }
            return min;
        } else {
            throw new IllegalArgumentException("Array cannot be empty.");
        }
    }

    public static float min(float[] array) {
        if (array == null) {
            throw new IllegalArgumentException("The Array must not be null");
        } else if (array.length != 0) {
            float min = array[0];
            for (int i = 1; i < array.length; i++) {
                if (Float.isNaN(array[i])) {
                    return Float.NaN;
                }
                if (array[i] < min) {
                    min = array[i];
                }
            }
            return min;
        } else {
            throw new IllegalArgumentException("Array cannot be empty.");
        }
    }

    public static long max(long[] array) {
        if (array == null) {
            throw new IllegalArgumentException("The Array must not be null");
        } else if (array.length != 0) {
            long max = array[0];
            for (int j = 1; j < array.length; j++) {
                if (array[j] > max) {
                    max = array[j];
                }
            }
            return max;
        } else {
            throw new IllegalArgumentException("Array cannot be empty.");
        }
    }

    public static int max(int[] array) {
        if (array == null) {
            throw new IllegalArgumentException("The Array must not be null");
        } else if (array.length != 0) {
            int max = array[0];
            for (int j = 1; j < array.length; j++) {
                if (array[j] > max) {
                    max = array[j];
                }
            }
            return max;
        } else {
            throw new IllegalArgumentException("Array cannot be empty.");
        }
    }

    public static short max(short[] array) {
        if (array == null) {
            throw new IllegalArgumentException("The Array must not be null");
        } else if (array.length != 0) {
            short max = array[0];
            for (int i = 1; i < array.length; i++) {
                if (array[i] > max) {
                    max = array[i];
                }
            }
            return max;
        } else {
            throw new IllegalArgumentException("Array cannot be empty.");
        }
    }

    public static byte max(byte[] array) {
        if (array == null) {
            throw new IllegalArgumentException("The Array must not be null");
        } else if (array.length != 0) {
            byte max = array[0];
            for (int i = 1; i < array.length; i++) {
                if (array[i] > max) {
                    max = array[i];
                }
            }
            return max;
        } else {
            throw new IllegalArgumentException("Array cannot be empty.");
        }
    }

    public static double max(double[] array) {
        if (array == null) {
            throw new IllegalArgumentException("The Array must not be null");
        } else if (array.length != 0) {
            double max = array[0];
            for (int j = 1; j < array.length; j++) {
                if (Double.isNaN(array[j])) {
                    return Double.NaN;
                }
                if (array[j] > max) {
                    max = array[j];
                }
            }
            return max;
        } else {
            throw new IllegalArgumentException("Array cannot be empty.");
        }
    }

    public static float max(float[] array) {
        if (array == null) {
            throw new IllegalArgumentException("The Array must not be null");
        } else if (array.length != 0) {
            float max = array[0];
            for (int j = 1; j < array.length; j++) {
                if (Float.isNaN(array[j])) {
                    return Float.NaN;
                }
                if (array[j] > max) {
                    max = array[j];
                }
            }
            return max;
        } else {
            throw new IllegalArgumentException("Array cannot be empty.");
        }
    }

    public static long min(long a, long b, long c) {
        if (b < a) {
            a = b;
        }
        if (c < a) {
            return c;
        }
        return a;
    }

    public static int min(int a, int b, int c) {
        if (b < a) {
            a = b;
        }
        if (c < a) {
            return c;
        }
        return a;
    }

    public static short min(short a, short b, short c) {
        if (b < a) {
            a = b;
        }
        if (c < a) {
            return c;
        }
        return a;
    }

    public static byte min(byte a, byte b, byte c) {
        if (b < a) {
            a = b;
        }
        if (c < a) {
            return c;
        }
        return a;
    }

    public static double min(double a, double b, double c) {
        return Math.min(Math.min(a, b), c);
    }

    public static float min(float a, float b, float c) {
        return Math.min(Math.min(a, b), c);
    }

    public static long max(long a, long b, long c) {
        if (b > a) {
            a = b;
        }
        if (c > a) {
            return c;
        }
        return a;
    }

    public static int max(int a, int b, int c) {
        if (b > a) {
            a = b;
        }
        if (c > a) {
            return c;
        }
        return a;
    }

    public static short max(short a, short b, short c) {
        if (b > a) {
            a = b;
        }
        if (c > a) {
            return c;
        }
        return a;
    }

    public static byte max(byte a, byte b, byte c) {
        if (b > a) {
            a = b;
        }
        if (c > a) {
            return c;
        }
        return a;
    }

    public static double max(double a, double b, double c) {
        return Math.max(Math.max(a, b), c);
    }

    public static float max(float a, float b, float c) {
        return Math.max(Math.max(a, b), c);
    }

    public static int compare(double lhs, double rhs) {
        if (lhs < rhs) {
            return -1;
        }
        if (lhs > rhs) {
            return 1;
        }
        long lhsBits = Double.doubleToLongBits(lhs);
        long rhsBits = Double.doubleToLongBits(rhs);
        if (lhsBits == rhsBits) {
            return 0;
        }
        if (lhsBits < rhsBits) {
            return -1;
        }
        return 1;
    }

    public static int compare(float lhs, float rhs) {
        if (lhs < rhs) {
            return -1;
        }
        if (lhs > rhs) {
            return 1;
        }
        int lhsBits = Float.floatToIntBits(lhs);
        int rhsBits = Float.floatToIntBits(rhs);
        if (lhsBits == rhsBits) {
            return 0;
        }
        if (lhsBits < rhsBits) {
            return -1;
        }
        return 1;
    }

    public static boolean isDigits(String str) {
        if (StringUtils.isEmpty(str)) {
            return false;
        }
        for (int i = 0; i < str.length(); i++) {
            if (!Character.isDigit(str.charAt(i))) {
                return false;
            }
        }
        return true;
    }

    public static boolean isNumber(String str) {
        boolean z;
        if (StringUtils.isEmpty(str)) {
            return false;
        }
        char[] chars = str.toCharArray();
        int sz = chars.length;
        boolean hasDecPoint = false;
        boolean allowSigns = false;
        boolean foundDigit = false;
        boolean z2 = true;
        int start = chars[0] == '-' ? 1 : 0;
        if (sz > start + 1 && chars[start] == '0' && chars[start + 1] == 'x') {
            int i = start + 2;
            if (i == sz) {
                return false;
            }
            while (i < chars.length) {
                if ((chars[i] < '0' || chars[i] > '9') && ((chars[i] < 'a' || chars[i] > 'f') && (chars[i] < 'A' || chars[i] > 'F'))) {
                    return false;
                }
                i++;
            }
            return true;
        }
        int sz2 = sz - 1;
        boolean hasExp = false;
        int i2 = start;
        while (true) {
            if (i2 < sz2 || (i2 < sz2 + 1 && allowSigns && !foundDigit)) {
                if (chars[i2] >= '0' && chars[i2] <= '9') {
                    allowSigns = false;
                    foundDigit = true;
                } else if (chars[i2] == '.') {
                    if (hasDecPoint || hasExp) {
                        return false;
                    }
                    hasDecPoint = true;
                } else if (chars[i2] != 'e' && chars[i2] != 'E') {
                    if (chars[i2] == '+') {
                        z = false;
                    } else if (chars[i2] != '-') {
                        return false;
                    } else {
                        z = false;
                    }
                    if (!allowSigns) {
                        return z;
                    }
                    allowSigns = false;
                    foundDigit = false;
                    i2++;
                    z2 = true;
                } else if (hasExp || !foundDigit) {
                    return false;
                } else {
                    allowSigns = true;
                    hasExp = true;
                    i2++;
                    z2 = true;
                }
                i2++;
                z2 = true;
            }
        }
        if (i2 >= chars.length) {
            return !allowSigns && foundDigit;
        }
        if (chars[i2] >= '0' && chars[i2] <= '9') {
            return z2;
        }
        if (chars[i2] == 'e' || chars[i2] == 'E') {
            return false;
        }
        if (allowSigns || (chars[i2] != 'd' && chars[i2] != 'D' && chars[i2] != 'f' && chars[i2] != 'F')) {
            return (chars[i2] == 'l' || chars[i2] == 'L') && foundDigit && !hasExp;
        }
        return foundDigit;
    }
}
