package org.yaml.snakeyaml.scanner;

import java.util.Arrays;

public final class Constant {
    public static final Constant ALPHA = new Constant(ALPHA_S);
    private static final String ALPHA_S = "abcdefghijklmnopqrstuvwxyz0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-_";
    public static final Constant FULL_LINEBR = new Constant(FULL_LINEBR_S);
    private static final String FULL_LINEBR_S = "\r\n  ";
    public static final Constant LINEBR = new Constant(LINEBR_S);
    private static final String LINEBR_S = "\n  ";
    public static final Constant NULL_BL_LINEBR = new Constant(NULL_BL_LINEBR_S);
    private static final String NULL_BL_LINEBR_S = " \u0000\r\n  ";
    public static final Constant NULL_BL_T = new Constant(NULL_BL_T_S);
    public static final Constant NULL_BL_T_LINEBR = new Constant(NULL_BL_T_LINEBR_S);
    private static final String NULL_BL_T_LINEBR_S = "\t \u0000\r\n  ";
    private static final String NULL_BL_T_S = "\u0000 \t";
    public static final Constant NULL_OR_LINEBR = new Constant(NULL_OR_LINEBR_S);
    private static final String NULL_OR_LINEBR_S = "\u0000\r\n  ";
    public static final Constant URI_CHARS = new Constant(URI_CHARS_S);
    private static final String URI_CHARS_S = "abcdefghijklmnopqrstuvwxyz0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-_-;/?:@&=+$,_.!~*'()[]%";
    boolean[] contains = new boolean[128];
    private String content;
    boolean noASCII;

    private Constant(String content2) {
        this.noASCII = false;
        Arrays.fill(this.contains, false);
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < content2.length(); i++) {
            char ch = content2.charAt(i);
            if (ch < 128) {
                this.contains[ch] = true;
            } else {
                sb.append(ch);
            }
        }
        if (sb.length() > 0) {
            this.noASCII = true;
            this.content = sb.toString();
        }
    }

    public boolean has(char ch) {
        if (ch < 128) {
            return this.contains[ch];
        }
        return this.noASCII && this.content.indexOf(ch, 0) != -1;
    }

    public boolean hasNo(char ch) {
        return !has(ch);
    }

    public boolean has(char ch, String additional) {
        return has(ch) || additional.indexOf(ch, 0) != -1;
    }

    public boolean hasNo(char ch, String additional) {
        return !has(ch, additional);
    }
}
