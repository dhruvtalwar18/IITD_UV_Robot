package org.yaml.snakeyaml;

import org.yaml.snakeyaml.representer.Representer;

public final class Dumper {
    protected final DumperOptions options;
    protected final Representer representer;

    public Dumper(Representer representer2, DumperOptions options2) {
        this.representer = representer2;
        this.options = options2;
    }

    public Dumper(DumperOptions options2) {
        this(new Representer(), options2);
    }

    public Dumper(Representer representer2) {
        this(representer2, new DumperOptions());
    }

    public Dumper() {
        this(new Representer(), new DumperOptions());
    }
}
