package org.yaml.snakeyaml.error;

import org.yaml.snakeyaml.scanner.Constant;

public final class Mark {
    private String buffer;
    private int column;
    private int index;
    private int line;
    private String name;
    private int pointer;

    public Mark(String name2, int index2, int line2, int column2, String buffer2, int pointer2) {
        this.name = name2;
        this.index = index2;
        this.line = line2;
        this.column = column2;
        this.buffer = buffer2;
        this.pointer = pointer2;
    }

    private boolean isLineBreak(char ch) {
        return Constant.NULL_OR_LINEBR.has(ch);
    }

    public String get_snippet(int indent, int max_length) {
        if (this.buffer == null) {
            return null;
        }
        float half = (float) ((max_length / 2) - 1);
        int start = this.pointer;
        String head = "";
        while (true) {
            if (start <= 0 || isLineBreak(this.buffer.charAt(start - 1))) {
                break;
            }
            start--;
            if (((float) (this.pointer - start)) > half) {
                head = " ... ";
                start += 5;
                break;
            }
        }
        String tail = "";
        int end = this.pointer;
        while (true) {
            if (end >= this.buffer.length() || isLineBreak(this.buffer.charAt(end))) {
                break;
            }
            end++;
            if (((float) (end - this.pointer)) > half) {
                tail = " ... ";
                end -= 5;
                break;
            }
        }
        String snippet = this.buffer.substring(start, end);
        StringBuilder result = new StringBuilder();
        for (int i = 0; i < indent; i++) {
            result.append(" ");
        }
        result.append(head);
        result.append(snippet);
        result.append(tail);
        result.append("\n");
        for (int i2 = 0; i2 < ((this.pointer + indent) - start) + head.length(); i2++) {
            result.append(" ");
        }
        result.append("^");
        return result.toString();
    }

    public String get_snippet() {
        return get_snippet(4, 75);
    }

    public String toString() {
        String snippet = get_snippet();
        StringBuilder where = new StringBuilder(" in \"");
        where.append(this.name);
        where.append("\", line ");
        where.append(this.line + 1);
        where.append(", column ");
        where.append(this.column + 1);
        if (snippet != null) {
            where.append(":\n");
            where.append(snippet);
        }
        return where.toString();
    }

    public String getName() {
        return this.name;
    }

    public int getLine() {
        return this.line;
    }

    public int getColumn() {
        return this.column;
    }

    public int getIndex() {
        return this.index;
    }
}
