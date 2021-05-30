package org.apache.commons.httpclient.util;

import java.util.ArrayList;
import java.util.List;
import org.apache.commons.httpclient.NameValuePair;

public class ParameterParser {
    private char[] chars = null;
    private int i1 = 0;
    private int i2 = 0;
    private int len = 0;
    private int pos = 0;

    private boolean hasChar() {
        return this.pos < this.len;
    }

    private String getToken(boolean quoted) {
        while (this.i1 < this.i2 && Character.isWhitespace(this.chars[this.i1])) {
            this.i1++;
        }
        while (this.i2 > this.i1 && Character.isWhitespace(this.chars[this.i2 - 1])) {
            this.i2--;
        }
        if (quoted && this.i2 - this.i1 >= 2 && this.chars[this.i1] == '\"' && this.chars[this.i2 - 1] == '\"') {
            this.i1++;
            this.i2--;
        }
        if (this.i2 >= this.i1) {
            return new String(this.chars, this.i1, this.i2 - this.i1);
        }
        return null;
    }

    private boolean isOneOf(char ch, char[] charray) {
        for (char c : charray) {
            if (ch == c) {
                return true;
            }
        }
        return false;
    }

    private String parseToken(char[] terminators) {
        this.i1 = this.pos;
        this.i2 = this.pos;
        while (hasChar() && !isOneOf(this.chars[this.pos], terminators)) {
            this.i2++;
            this.pos++;
        }
        return getToken(false);
    }

    private String parseQuotedToken(char[] terminators) {
        this.i1 = this.pos;
        this.i2 = this.pos;
        boolean quoted = false;
        boolean charEscaped = false;
        while (hasChar()) {
            char ch = this.chars[this.pos];
            if (!quoted && isOneOf(ch, terminators)) {
                break;
            }
            if (!charEscaped && ch == '\"') {
                quoted = !quoted;
            }
            charEscaped = !charEscaped && ch == '\\';
            this.i2++;
            this.pos++;
        }
        return getToken(true);
    }

    public List parse(String str, char separator) {
        if (str == null) {
            return new ArrayList();
        }
        return parse(str.toCharArray(), separator);
    }

    public List parse(char[] chars2, char separator) {
        if (chars2 == null) {
            return new ArrayList();
        }
        return parse(chars2, 0, chars2.length, separator);
    }

    public List parse(char[] chars2, int offset, int length, char separator) {
        if (chars2 == null) {
            return new ArrayList();
        }
        List params = new ArrayList();
        this.chars = chars2;
        this.pos = offset;
        this.len = length;
        while (hasChar()) {
            String paramName = parseToken(new char[]{'=', separator});
            String paramValue = null;
            if (hasChar() && chars2[this.pos] == '=') {
                this.pos++;
                paramValue = parseQuotedToken(new char[]{separator});
            }
            if (hasChar() && chars2[this.pos] == separator) {
                this.pos++;
            }
            if (paramName != null && (!paramName.equals("") || paramValue != null)) {
                params.add(new NameValuePair(paramName, paramValue));
            }
        }
        return params;
    }
}
