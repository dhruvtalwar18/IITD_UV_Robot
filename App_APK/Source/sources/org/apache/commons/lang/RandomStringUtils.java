package org.apache.commons.lang;

import java.util.Random;

public class RandomStringUtils {
    private static final Random RANDOM = new Random();

    public static String random(int count) {
        return random(count, false, false);
    }

    public static String randomAscii(int count) {
        return random(count, 32, 127, false, false);
    }

    public static String randomAlphabetic(int count) {
        return random(count, true, false);
    }

    public static String randomAlphanumeric(int count) {
        return random(count, true, true);
    }

    public static String randomNumeric(int count) {
        return random(count, false, true);
    }

    public static String random(int count, boolean letters, boolean numbers) {
        return random(count, 0, 0, letters, numbers);
    }

    public static String random(int count, int start, int end, boolean letters, boolean numbers) {
        return random(count, start, end, letters, numbers, (char[]) null, RANDOM);
    }

    public static String random(int count, int start, int end, boolean letters, boolean numbers, char[] chars) {
        return random(count, start, end, letters, numbers, chars, RANDOM);
    }

    public static String random(int count, int start, int end, boolean letters, boolean numbers, char[] chars, Random random) {
        char ch;
        if (count == 0) {
            return "";
        }
        if (count >= 0) {
            if (start == 0 && end == 0) {
                end = 123;
                start = 32;
                if (!letters && !numbers) {
                    start = 0;
                    end = Integer.MAX_VALUE;
                }
            }
            char[] buffer = new char[count];
            int gap = end - start;
            while (true) {
                int count2 = count - 1;
                if (count == 0) {
                    return new String(buffer);
                }
                if (chars == null) {
                    ch = (char) (random.nextInt(gap) + start);
                } else {
                    ch = chars[random.nextInt(gap) + start];
                }
                if ((!letters || !Character.isLetter(ch)) && ((!numbers || !Character.isDigit(ch)) && (letters || numbers))) {
                    count2++;
                } else if (ch < 56320 || ch > 57343) {
                    if (ch < 55296 || ch > 56191) {
                        if (ch < 56192 || ch > 56319) {
                            buffer[count2] = ch;
                        } else {
                            count2++;
                        }
                    } else if (count2 == 0) {
                        count2++;
                    } else {
                        buffer[count2] = (char) (random.nextInt(128) + 56320);
                        count2--;
                        buffer[count2] = ch;
                    }
                } else if (count2 == 0) {
                    count2++;
                } else {
                    buffer[count2] = ch;
                    count2--;
                    buffer[count2] = (char) (random.nextInt(128) + 55296);
                }
                count = count2;
            }
        } else {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Requested random string length ");
            stringBuffer.append(count);
            stringBuffer.append(" is less than 0.");
            throw new IllegalArgumentException(stringBuffer.toString());
        }
    }

    public static String random(int count, String chars) {
        if (chars != null) {
            return random(count, chars.toCharArray());
        }
        return random(count, 0, 0, false, false, (char[]) null, RANDOM);
    }

    public static String random(int count, char[] chars) {
        if (chars == null) {
            return random(count, 0, 0, false, false, (char[]) null, RANDOM);
        }
        return random(count, 0, chars.length, false, false, chars, RANDOM);
    }
}
