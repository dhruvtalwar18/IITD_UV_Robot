package org.bytedeco.javacpp.tools;

import java.io.File;
import java.net.MalformedURLException;
import java.net.URL;
import java.net.URLClassLoader;
import java.util.ArrayList;
import java.util.List;

class UserClassLoader extends URLClassLoader {
    private List<String> paths = new ArrayList();

    public UserClassLoader() {
        super(new URL[0]);
    }

    public UserClassLoader(ClassLoader parent) {
        super(new URL[0], parent);
    }

    public void addPaths(String... paths2) {
        if (paths2 != null) {
            for (String path : paths2) {
                File f = new File(path);
                if (f.exists()) {
                    this.paths.add(path);
                    try {
                        addURL(f.toURI().toURL());
                    } catch (MalformedURLException e) {
                        throw new RuntimeException(e);
                    }
                }
            }
        }
    }

    public String[] getPaths() {
        if (this.paths.isEmpty()) {
            addPaths(System.getProperty("user.dir"));
        }
        return (String[]) this.paths.toArray(new String[this.paths.size()]);
    }

    /* access modifiers changed from: protected */
    public Class<?> findClass(String name) throws ClassNotFoundException {
        if (this.paths.isEmpty()) {
            addPaths(System.getProperty("user.dir"));
        }
        return super.findClass(name);
    }
}
