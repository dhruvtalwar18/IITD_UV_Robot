package org.yaml.snakeyaml.tokens;

import org.yaml.snakeyaml.error.Mark;
import org.yaml.snakeyaml.tokens.Token;

public final class ScalarToken extends Token {
    private final boolean plain;
    private final char style;
    private final String value;

    public ScalarToken(String value2, Mark startMark, Mark endMark, boolean plain2) {
        this(value2, plain2, startMark, endMark, 0);
    }

    public ScalarToken(String value2, boolean plain2, Mark startMark, Mark endMark, char style2) {
        super(startMark, endMark);
        this.value = value2;
        this.plain = plain2;
        this.style = style2;
    }

    public boolean getPlain() {
        return this.plain;
    }

    public String getValue() {
        return this.value;
    }

    public char getStyle() {
        return this.style;
    }

    /* access modifiers changed from: protected */
    public String getArguments() {
        return "value=" + this.value + ", plain=" + this.plain + ", style=" + this.style;
    }

    public Token.ID getTokenId() {
        return Token.ID.Scalar;
    }
}
