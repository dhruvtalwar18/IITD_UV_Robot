package org.bytedeco.javacpp.tools;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Arrays;
import java.util.Collection;
import java.util.jar.JarInputStream;
import java.util.zip.ZipEntry;
import org.apache.commons.httpclient.cookie.CookieSpec;
import org.apache.commons.io.IOUtils;

class ClassScanner {
    final Collection<Class> classes;
    final UserClassLoader loader;
    final Logger logger;

    ClassScanner(Logger logger2, Collection<Class> classes2, UserClassLoader loader2) {
        this.logger = logger2;
        this.classes = classes2;
        this.loader = loader2;
    }

    public Collection<Class> getClasses() {
        return this.classes;
    }

    public UserClassLoader getClassLoader() {
        return this.loader;
    }

    public void addClass(String className) throws ClassNotFoundException, NoClassDefFoundError {
        if (className != null) {
            if (className.endsWith(".class")) {
                className = className.substring(0, className.length() - 6);
            }
            addClass(Class.forName(className, false, this.loader));
        }
    }

    public void addClass(Class c) {
        if (!this.classes.contains(c)) {
            this.classes.add(c);
        }
    }

    public void addMatchingFile(String filename, String packagePath, boolean recursive) throws ClassNotFoundException, NoClassDefFoundError {
        if (filename != null && filename.endsWith(".class")) {
            if (packagePath == null || ((recursive && filename.startsWith(packagePath)) || filename.regionMatches(0, packagePath, 0, Math.max(filename.lastIndexOf(47), packagePath.lastIndexOf(47))))) {
                addClass(filename.replace(IOUtils.DIR_SEPARATOR_UNIX, '.'));
            }
        }
    }

    public void addMatchingDir(String parentName, File dir, String packagePath, boolean recursive) throws ClassNotFoundException, NoClassDefFoundError {
        String pathName;
        File[] files = dir.listFiles();
        Arrays.sort(files);
        for (File f : files) {
            if (parentName == null) {
                pathName = f.getName();
            } else {
                pathName = parentName + f.getName();
            }
            if (f.isDirectory()) {
                addMatchingDir(pathName + CookieSpec.PATH_DELIM, f, packagePath, recursive);
            } else {
                addMatchingFile(pathName, packagePath, recursive);
            }
        }
    }

    public void addPackage(String packageName, boolean recursive) throws IOException, ClassNotFoundException, NoClassDefFoundError {
        String packagePath;
        String[] paths = this.loader.getPaths();
        if (packageName == null) {
            packagePath = null;
        } else {
            packagePath = packageName.replace('.', IOUtils.DIR_SEPARATOR_UNIX) + CookieSpec.PATH_DELIM;
        }
        int prevSize = this.classes.size();
        for (String p : paths) {
            File file = new File(p);
            if (file.isDirectory()) {
                addMatchingDir((String) null, file, packagePath, recursive);
            } else {
                JarInputStream jis = new JarInputStream(new FileInputStream(file));
                for (ZipEntry e = jis.getNextEntry(); e != null; e = jis.getNextEntry()) {
                    addMatchingFile(e.getName(), packagePath, recursive);
                    jis.closeEntry();
                }
                jis.close();
            }
        }
        if (this.classes.size() == 0 && packageName == null) {
            this.logger.warn("No classes found in the unnamed package");
            Builder.printHelp();
        } else if (prevSize == this.classes.size() && packageName != null) {
            this.logger.warn("No classes found in package " + packageName);
        }
    }

    public void addClassOrPackage(String name) throws IOException, ClassNotFoundException, NoClassDefFoundError {
        if (name != null) {
            String name2 = name.replace(IOUtils.DIR_SEPARATOR_UNIX, '.');
            if (name2.endsWith(".**")) {
                addPackage(name2.substring(0, name2.length() - 3), true);
            } else if (name2.endsWith(".*")) {
                addPackage(name2.substring(0, name2.length() - 2), false);
            } else {
                addClass(name2);
            }
        }
    }
}
