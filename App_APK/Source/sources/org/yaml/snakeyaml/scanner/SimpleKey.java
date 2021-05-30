package org.yaml.snakeyaml.scanner;

import org.yaml.snakeyaml.error.Mark;

final class SimpleKey {
    private int column;
    private int index;
    private int line;
    private Mark mark;
    private boolean required;
    private int tokenNumber;

    public SimpleKey(int tokenNumber2, boolean required2, int index2, int line2, int column2, Mark mark2) {
        this.tokenNumber = tokenNumber2;
        this.required = required2;
        this.index = index2;
        this.line = line2;
        this.column = column2;
        this.mark = mark2;
    }

    public int getTokenNumber() {
        return this.tokenNumber;
    }

    public int getColumn() {
        return this.column;
    }

    public Mark getMark() {
        return this.mark;
    }

    public int getIndex() {
        return this.index;
    }

    public int getLine() {
        return this.line;
    }

    public boolean isRequired() {
        return this.required;
    }

    public String toString() {
        return "SimpleKey - tokenNumber=" + this.tokenNumber + " required=" + this.required + " index=" + this.index + " line=" + this.line + " column=" + this.column;
    }
}
