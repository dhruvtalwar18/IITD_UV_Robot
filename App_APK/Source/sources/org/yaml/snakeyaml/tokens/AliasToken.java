package org.yaml.snakeyaml.tokens;

import org.yaml.snakeyaml.error.Mark;
import org.yaml.snakeyaml.tokens.Token;

public final class AliasToken extends Token {
    private final String value;

    public AliasToken(String value2, Mark startMark, Mark endMark) {
        super(startMark, endMark);
        this.value = value2;
    }

    public String getValue() {
        return this.value;
    }

    /* access modifiers changed from: protected */
    public String getArguments() {
        return "value=" + this.value;
    }

    public Token.ID getTokenId() {
        return Token.ID.Alias;
    }
}
