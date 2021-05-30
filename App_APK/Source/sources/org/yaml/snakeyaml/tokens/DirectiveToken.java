package org.yaml.snakeyaml.tokens;

import java.util.List;
import org.yaml.snakeyaml.error.Mark;
import org.yaml.snakeyaml.error.YAMLException;
import org.yaml.snakeyaml.tokens.Token;

public final class DirectiveToken<T> extends Token {
    private final String name;
    private final List<T> value;

    public DirectiveToken(String name2, List<T> value2, Mark startMark, Mark endMark) {
        super(startMark, endMark);
        this.name = name2;
        if (value2 == null || value2.size() == 2) {
            this.value = value2;
            return;
        }
        throw new YAMLException("Two strings must be provided instead of " + String.valueOf(value2.size()));
    }

    public String getName() {
        return this.name;
    }

    public List<T> getValue() {
        return this.value;
    }

    /* access modifiers changed from: protected */
    public String getArguments() {
        if (this.value != null) {
            return "name=" + this.name + ", value=[" + this.value.get(0) + ", " + this.value.get(1) + "]";
        }
        return "name=" + this.name;
    }

    public Token.ID getTokenId() {
        return Token.ID.Directive;
    }
}
