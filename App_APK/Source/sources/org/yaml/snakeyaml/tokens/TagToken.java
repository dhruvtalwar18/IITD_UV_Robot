package org.yaml.snakeyaml.tokens;

import org.yaml.snakeyaml.error.Mark;
import org.yaml.snakeyaml.tokens.Token;

public final class TagToken extends Token {
    private final TagTuple value;

    public TagToken(TagTuple value2, Mark startMark, Mark endMark) {
        super(startMark, endMark);
        this.value = value2;
    }

    public TagTuple getValue() {
        return this.value;
    }

    /* access modifiers changed from: protected */
    public String getArguments() {
        return "value=[" + this.value.getHandle() + ", " + this.value.getSuffix() + "]";
    }

    public Token.ID getTokenId() {
        return Token.ID.Tag;
    }
}
