package org.apache.commons.io.input;

import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectInputStream;
import java.io.ObjectStreamClass;
import java.io.StreamCorruptedException;

public class ClassLoaderObjectInputStream extends ObjectInputStream {
    private ClassLoader classLoader;

    public ClassLoaderObjectInputStream(ClassLoader classLoader2, InputStream inputStream) throws IOException, StreamCorruptedException {
        super(inputStream);
        this.classLoader = classLoader2;
    }

    /* access modifiers changed from: protected */
    public Class resolveClass(ObjectStreamClass objectStreamClass) throws IOException, ClassNotFoundException {
        Class clazz = Class.forName(objectStreamClass.getName(), false, this.classLoader);
        if (clazz != null) {
            return clazz;
        }
        return super.resolveClass(objectStreamClass);
    }
}
