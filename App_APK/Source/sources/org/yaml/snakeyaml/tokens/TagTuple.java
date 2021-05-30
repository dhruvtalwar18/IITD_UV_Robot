package org.yaml.snakeyaml.tokens;

public final class TagTuple {
    private final String handle;
    private final String suffix;

    public TagTuple(String handle2, String suffix2) {
        if (suffix2 != null) {
            this.handle = handle2;
            this.suffix = suffix2;
            return;
        }
        throw new NullPointerException("Suffix must be provided.");
    }

    public String getHandle() {
        return this.handle;
    }

    public String getSuffix() {
        return this.suffix;
    }
}
