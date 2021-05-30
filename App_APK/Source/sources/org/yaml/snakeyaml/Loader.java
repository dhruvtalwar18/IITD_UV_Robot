package org.yaml.snakeyaml;

import org.yaml.snakeyaml.constructor.BaseConstructor;
import org.yaml.snakeyaml.constructor.Constructor;
import org.yaml.snakeyaml.resolver.Resolver;

public final class Loader {
    protected final BaseConstructor constructor;
    protected Resolver resolver;

    public Loader(BaseConstructor constructor2) {
        this.constructor = constructor2;
    }

    public Loader() {
        this(new Constructor());
    }
}
