package org.yaml.snakeyaml.reader;

import java.io.IOException;
import java.io.Reader;
import java.nio.charset.Charset;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.yaml.snakeyaml.error.Mark;
import org.yaml.snakeyaml.error.YAMLException;
import org.yaml.snakeyaml.scanner.Constant;

public class StreamReader {
    static final Pattern NON_PRINTABLE = Pattern.compile("[^\t\n\r -~ -퟿-￼]");
    private String buffer;
    private int column;
    private char[] data;
    private boolean eof;
    private int index;
    private int line;
    private String name;
    private int pointer;
    private final Reader stream;

    public StreamReader(String stream2) {
        this.pointer = 0;
        this.eof = true;
        this.index = 0;
        this.line = 0;
        this.column = 0;
        this.name = "<string>";
        this.buffer = "";
        checkPrintable(stream2);
        this.buffer = stream2 + "\u0000";
        this.stream = null;
        this.eof = true;
        this.data = null;
    }

    public StreamReader(Reader reader) {
        this.pointer = 0;
        this.eof = true;
        this.index = 0;
        this.line = 0;
        this.column = 0;
        this.name = "<reader>";
        this.buffer = "";
        this.stream = reader;
        this.eof = false;
        this.data = new char[1024];
        update();
    }

    /* access modifiers changed from: package-private */
    public void checkPrintable(CharSequence data2) {
        Matcher em = NON_PRINTABLE.matcher(data2);
        if (em.find()) {
            throw new ReaderException(this.name, ((this.index + this.buffer.length()) - this.pointer) + em.start(), em.group().charAt(0), "special characters are not allowed");
        }
    }

    /* access modifiers changed from: package-private */
    public void checkPrintable(char[] chars, int begin, int end) {
        int i = begin;
        while (i < end) {
            char c = chars[i];
            if ((c >= ' ' && c <= '~') || c == 10 || c == 13 || c == 9 || c == 133 || ((c >= 160 && c <= 55295) || (c >= 57344 && c <= 65532))) {
                i++;
            } else {
                throw new ReaderException(this.name, ((this.index + this.buffer.length()) - this.pointer) + i, c, "special characters are not allowed");
            }
        }
    }

    public Mark getMark() {
        return new Mark(this.name, this.index, this.line, this.column, this.buffer, this.pointer);
    }

    public void forward() {
        forward(1);
    }

    public void forward(int length) {
        if (this.pointer + length + 1 >= this.buffer.length()) {
            update();
        }
        for (int i = 0; i < length; i++) {
            char ch = this.buffer.charAt(this.pointer);
            this.pointer++;
            this.index++;
            if (Constant.LINEBR.has(ch) || (ch == 13 && this.buffer.charAt(this.pointer) != 10)) {
                this.line++;
                this.column = 0;
            } else if (ch != 65279) {
                this.column++;
            }
        }
    }

    public char peek() {
        return this.buffer.charAt(this.pointer);
    }

    public char peek(int index2) {
        if (this.pointer + index2 + 1 > this.buffer.length()) {
            update();
        }
        return this.buffer.charAt(this.pointer + index2);
    }

    public String prefix(int length) {
        if (this.pointer + length >= this.buffer.length()) {
            update();
        }
        if (this.pointer + length > this.buffer.length()) {
            return this.buffer.substring(this.pointer);
        }
        return this.buffer.substring(this.pointer, this.pointer + length);
    }

    public String prefixForward(int length) {
        String prefix = prefix(length);
        this.pointer += length;
        this.index += length;
        this.column += length;
        return prefix;
    }

    private void update() {
        if (!this.eof) {
            this.buffer = this.buffer.substring(this.pointer);
            this.pointer = 0;
            try {
                int converted = this.stream.read(this.data);
                if (converted > 0) {
                    checkPrintable(this.data, 0, converted);
                    StringBuilder sb = new StringBuilder(this.buffer.length() + converted);
                    sb.append(this.buffer);
                    sb.append(this.data, 0, converted);
                    this.buffer = sb.toString();
                    return;
                }
                this.eof = true;
                this.buffer += "\u0000";
            } catch (IOException ioe) {
                throw new YAMLException((Throwable) ioe);
            }
        }
    }

    public int getColumn() {
        return this.column;
    }

    public Charset getEncoding() {
        return Charset.forName(((UnicodeReader) this.stream).getEncoding());
    }

    public int getIndex() {
        return this.index;
    }

    public int getLine() {
        return this.line;
    }
}
