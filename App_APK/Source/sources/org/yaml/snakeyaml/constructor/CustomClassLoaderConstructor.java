package org.yaml.snakeyaml.constructor;

public class CustomClassLoaderConstructor extends Constructor {
    private ClassLoader loader;

    public CustomClassLoaderConstructor(ClassLoader cLoader) {
        this(Object.class, cLoader);
    }

    public CustomClassLoaderConstructor(Class<? extends Object> theRoot, ClassLoader theLoader) {
        super(theRoot);
        this.loader = CustomClassLoaderConstructor.class.getClassLoader();
        if (theLoader != null) {
            this.loader = theLoader;
            return;
        }
        throw new NullPointerException("Loader must be provided.");
    }

    /* access modifiers changed from: protected */
    public Class<?> getClassForName(String name) throws ClassNotFoundException {
        return Class.forName(name, true, this.loader);
    }
}
