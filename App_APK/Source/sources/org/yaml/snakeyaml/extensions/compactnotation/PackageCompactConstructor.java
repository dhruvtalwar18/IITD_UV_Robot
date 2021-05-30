package org.yaml.snakeyaml.extensions.compactnotation;

public class PackageCompactConstructor extends CompactConstructor {
    private String packageName;

    public PackageCompactConstructor(String packageName2) {
        this.packageName = packageName2;
    }

    /* access modifiers changed from: protected */
    public Class<?> getClassForName(String name) throws ClassNotFoundException {
        if (name.indexOf(46) < 0) {
            try {
                return Class.forName(this.packageName + "." + name);
            } catch (ClassNotFoundException e) {
            }
        }
        return super.getClassForName(name);
    }
}
