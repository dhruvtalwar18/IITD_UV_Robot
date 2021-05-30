package org.yaml.snakeyaml;

public class LoaderOptions {
    private TypeDescription rootTypeDescription;

    public LoaderOptions() {
        this(new TypeDescription(Object.class));
    }

    public LoaderOptions(TypeDescription rootTypeDescription2) {
        this.rootTypeDescription = rootTypeDescription2;
    }

    public TypeDescription getRootTypeDescription() {
        return this.rootTypeDescription;
    }

    public void setRootTypeDescription(TypeDescription rootTypeDescription2) {
        this.rootTypeDescription = rootTypeDescription2;
    }
}
