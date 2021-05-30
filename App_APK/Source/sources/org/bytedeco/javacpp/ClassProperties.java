package org.bytedeco.javacpp;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import org.apache.commons.httpclient.cookie.CookieSpec;
import org.apache.commons.io.IOUtils;
import org.bytedeco.javacpp.annotation.Platform;

public class ClassProperties extends HashMap<String, List<String>> {
    String[] defaultNames = new String[0];
    List<Class> effectiveClasses = null;
    List<Class> inheritedClasses = null;
    boolean loaded = false;
    String pathSeparator;
    String platform;
    String platformExtension;
    String platformRoot;

    public ClassProperties() {
    }

    public ClassProperties(Properties properties) {
        this.platform = properties.getProperty("platform");
        this.platformExtension = properties.getProperty("platform.extension");
        this.platformRoot = properties.getProperty("platform.root");
        this.pathSeparator = properties.getProperty("platform.path.separator");
        if (this.platformRoot == null || this.platformRoot.length() == 0) {
            this.platformRoot = ".";
        }
        if (!this.platformRoot.endsWith(File.separator)) {
            this.platformRoot += File.separator;
        }
        for (Map.Entry e : properties.entrySet()) {
            String k = (String) e.getKey();
            String v = (String) e.getValue();
            if (!(v == null || v.length() == 0)) {
                if (k.equals("platform.includepath") || k.equals("platform.includeresource") || k.equals("platform.include") || k.equals("platform.linkpath") || k.equals("platform.linkresource") || k.equals("platform.link") || k.equals("platform.preloadpath") || k.equals("platform.preloadresource") || k.equals("platform.preload") || k.equals("platform.resourcepath") || k.equals("platform.resource") || k.equals("platform.frameworkpath") || k.equals("platform.framework") || k.equals("platform.library.suffix") || k.equals("platform.extension")) {
                    addAll(k, v.split(this.pathSeparator));
                } else {
                    setProperty(k, v);
                }
            }
        }
    }

    public List<String> get(String key) {
        List<String> list = (List) super.get(key);
        if (list != null) {
            return list;
        }
        List<String> arrayList = new ArrayList<>();
        List<String> list2 = arrayList;
        put(key, arrayList);
        return list2;
    }

    public void addAll(String key, String... values) {
        if (values != null) {
            addAll(key, (Collection<String>) Arrays.asList(values));
        }
    }

    public void addAll(String key, Collection<String> values) {
        if (values != null) {
            String root = null;
            if (key.equals("platform.compiler") || key.equals("platform.sysroot") || key.equals("platform.toolchain") || key.equals("platform.includepath") || key.equals("platform.linkpath")) {
                root = this.platformRoot;
            }
            List<String> values2 = get(key);
            Iterator<String> it = values.iterator();
            while (it.hasNext()) {
                String value = it.next();
                if (value != null) {
                    if (root != null && !new File(value).isAbsolute()) {
                        if (new File(root + value).exists()) {
                            value = root + value;
                        }
                    }
                    if (!values2.contains(value)) {
                        values2.add(value);
                    }
                }
            }
        }
    }

    public String getProperty(String key) {
        return getProperty(key, (String) null);
    }

    public String getProperty(String key, String defaultValue) {
        List<String> values = get(key);
        return values.isEmpty() ? defaultValue : values.get(0);
    }

    public String setProperty(String key, String value) {
        List<String> values = get(key);
        String oldValue = values.isEmpty() ? null : values.get(0);
        values.clear();
        addAll(key, value);
        return oldValue;
    }

    public void load(Class cls, boolean inherit) {
        char c;
        String[] frameworkpath;
        String[] link;
        String[] linkpath;
        boolean z = inherit;
        Class<?> c2 = Loader.getEnclosingClass(cls);
        List<Class> arrayList = new ArrayList<>();
        arrayList.add(0, c2);
        Class<?> c3 = c2;
        while (!c3.isAnnotationPresent(org.bytedeco.javacpp.annotation.Properties.class) && !c3.isAnnotationPresent(Platform.class) && c3.getSuperclass() != null && c3.getSuperclass() != Object.class && c3.getSuperclass() != Pointer.class) {
            Class<? super Object> superclass = c3.getSuperclass();
            c3 = superclass;
            arrayList.add(0, superclass);
        }
        if (this.effectiveClasses == null) {
            this.effectiveClasses = arrayList;
        }
        org.bytedeco.javacpp.annotation.Properties classProperties = (org.bytedeco.javacpp.annotation.Properties) c3.getAnnotation(org.bytedeco.javacpp.annotation.Properties.class);
        Platform[] platforms = null;
        if (classProperties == null) {
            Platform platform2 = (Platform) c3.getAnnotation(Platform.class);
            if (platform2 != null) {
                platforms = new Platform[]{platform2};
            }
        } else {
            Class[] classes = classProperties.inherit();
            if (z && classes != null) {
                if (this.inheritedClasses == null) {
                    this.inheritedClasses = new ArrayList();
                }
                for (Class c22 : classes) {
                    load(c22, z);
                    if (!this.inheritedClasses.contains(c22)) {
                        this.inheritedClasses.add(c22);
                    }
                }
            }
            String target = classProperties.target();
            if (target.length() > 0) {
                addAll("target", target);
            }
            String global = classProperties.global();
            if (global.length() == 0) {
                global = target;
            } else if (target.length() > 0 && !global.startsWith(target)) {
                global = target + "." + global;
            }
            if (global.length() > 0) {
                addAll("global", global);
            }
            String helper = classProperties.helper();
            if (helper.length() > 0) {
                addAll("helper", helper);
            }
            String[] names = classProperties.names();
            if (names.length > 0) {
                this.defaultNames = names;
            }
            platforms = classProperties.value();
        }
        Platform[] platforms2 = platforms;
        String[] define = new String[0];
        String[] exclude = new String[0];
        String[] include = new String[0];
        String[] cinclude = new String[0];
        String[] includepath = new String[0];
        String[] includeresource = new String[0];
        String[] compiler = new String[0];
        String[] linkpath2 = new String[0];
        String[] pragma = new String[0];
        String[] linkresource = new String[0];
        String[] link2 = new String[0];
        String[] frameworkpath2 = new String[0];
        String[] framework = new String[0];
        String[] preloadpath = new String[0];
        String[] preloadresource = new String[0];
        String[] preload = new String[0];
        String[] resourcepath = new String[0];
        String[] resource = new String[0];
        StringBuilder sb = new StringBuilder();
        String[] extension = new String[0];
        sb.append("jni");
        sb.append(c3.getSimpleName());
        String library = sb.toString();
        List<String> targets = get("global");
        String library2 = library;
        if (targets == null || targets.size() <= 0) {
            List<Class> classList = arrayList;
        } else {
            String target2 = targets.get(targets.size() - 1);
            StringBuilder sb2 = new StringBuilder();
            ArrayList arrayList2 = arrayList;
            sb2.append("jni");
            sb2.append(target2.substring(target2.lastIndexOf(46) + 1));
            library2 = sb2.toString();
        }
        Platform[] platformArr = platforms2 != null ? platforms2 : new Platform[0];
        int length = platformArr.length;
        List<String> list = targets;
        Class<?> c4 = c3;
        org.bytedeco.javacpp.annotation.Properties properties = classProperties;
        String[] linkpath3 = linkpath2;
        Platform[] platforms3 = platforms2;
        String[] pragma2 = pragma;
        String[] linkresource2 = linkresource;
        String[] link3 = link2;
        String[] frameworkpath3 = frameworkpath2;
        String[] framework2 = framework;
        String[] preloadpath2 = preloadpath;
        String[] preloadresource2 = preloadresource;
        String[] preload2 = preload;
        String[] resourcepath2 = resourcepath;
        String[] resource2 = resource;
        String[] extension2 = extension;
        String library3 = library2;
        int i = 0;
        while (i < length) {
            Platform p = platformArr[i];
            int i2 = length;
            Platform[] platformArr2 = platformArr;
            String[][] strArr = new String[2][];
            strArr[0] = p.value().length > 0 ? p.value() : this.defaultNames;
            strArr[1] = p.not();
            String[][] names2 = strArr;
            boolean[] matches = {false, false};
            int i3 = 0;
            while (true) {
                frameworkpath = frameworkpath3;
                link = link3;
                int i4 = i3;
                if (i4 >= names2.length) {
                    break;
                }
                String[] strArr2 = names2[i4];
                String[] linkpath4 = linkpath3;
                int length2 = strArr2.length;
                String[] compiler2 = compiler;
                int i5 = 0;
                while (true) {
                    if (i5 >= length2) {
                        break;
                    }
                    int i6 = length2;
                    String[] strArr3 = strArr2;
                    if (this.platform.startsWith(strArr2[i5])) {
                        matches[i4] = true;
                        break;
                    }
                    i5++;
                    length2 = i6;
                    strArr2 = strArr3;
                }
                i3 = i4 + 1;
                frameworkpath3 = frameworkpath;
                link3 = link;
                linkpath3 = linkpath4;
                compiler = compiler2;
            }
            String[] linkpath5 = linkpath3;
            String[] compiler3 = compiler;
            if ((names2[0].length == 0 || matches[0]) && (names2[1].length == 0 || !matches[1])) {
                boolean match = p.extension().length == 0 || (Loader.isLoadLibraries() && this.platformExtension == null);
                String[] extension3 = p.extension();
                int length3 = extension3.length;
                int i7 = 0;
                while (true) {
                    if (i7 >= length3) {
                        boolean[] zArr = matches;
                        break;
                    }
                    String[][] names3 = names2;
                    String s = extension3[i7];
                    boolean[] matches2 = matches;
                    if (this.platformExtension != null && this.platformExtension.length() > 0 && this.platformExtension.endsWith(s)) {
                        match = true;
                        break;
                    }
                    i7++;
                    names2 = names3;
                    matches = matches2;
                }
                if (match) {
                    if (p.pragma().length > 0) {
                        pragma2 = p.pragma();
                    }
                    if (p.define().length > 0) {
                        define = p.define();
                    }
                    if (p.exclude().length > 0) {
                        exclude = p.exclude();
                    }
                    if (p.include().length > 0) {
                        include = p.include();
                    }
                    if (p.cinclude().length > 0) {
                        cinclude = p.cinclude();
                    }
                    if (p.includepath().length > 0) {
                        includepath = p.includepath();
                    }
                    if (p.includeresource().length > 0) {
                        includeresource = p.includeresource();
                    }
                    if (p.compiler().length > 0) {
                        compiler = p.compiler();
                    } else {
                        compiler = compiler3;
                    }
                    if (p.linkpath().length > 0) {
                        linkpath = p.linkpath();
                    } else {
                        linkpath = linkpath5;
                    }
                    if (p.linkresource().length > 0) {
                        linkresource2 = p.linkresource();
                    }
                    if (p.link().length > 0) {
                        link = p.link();
                    }
                    if (p.frameworkpath().length > 0) {
                        frameworkpath = p.frameworkpath();
                    }
                    if (p.framework().length > 0) {
                        framework2 = p.framework();
                    }
                    if (p.preloadresource().length > 0) {
                        preloadresource2 = p.preloadresource();
                    }
                    if (p.preloadpath().length > 0) {
                        preloadpath2 = p.preloadpath();
                    }
                    if (p.preload().length > 0) {
                        preload2 = p.preload();
                    }
                    if (p.resourcepath().length > 0) {
                        resourcepath2 = p.resourcepath();
                    }
                    if (p.resource().length > 0) {
                        resource2 = p.resource();
                    }
                    if (p.extension().length > 0) {
                        extension2 = p.extension();
                    }
                    if (p.library().length() > 0) {
                        linkpath3 = linkpath;
                        library3 = p.library();
                    } else {
                        linkpath3 = linkpath;
                    }
                    frameworkpath3 = frameworkpath;
                    link3 = link;
                    i++;
                    length = i2;
                    platformArr = platformArr2;
                }
            }
            frameworkpath3 = frameworkpath;
            link3 = link;
            linkpath3 = linkpath5;
            compiler = compiler3;
            i++;
            length = i2;
            platformArr = platformArr2;
        }
        String[] linkpath6 = linkpath3;
        String[] frameworkpath4 = frameworkpath3;
        String[] link4 = link3;
        String[] compiler4 = compiler;
        int i8 = 0;
        while (true) {
            int length4 = includeresource.length;
            c = IOUtils.DIR_SEPARATOR_UNIX;
            if (i8 >= length4) {
                break;
            }
            String name = includeresource[i8];
            if (!name.startsWith(CookieSpec.PATH_DELIM)) {
                String s2 = cls.getName().replace('.', IOUtils.DIR_SEPARATOR_UNIX);
                int n = s2.lastIndexOf(47);
                if (n >= 0) {
                    name = s2.substring(0, n + 1) + name;
                }
                includeresource[i8] = CookieSpec.PATH_DELIM + name;
            }
            i8++;
        }
        int i9 = 0;
        while (i9 < linkresource2.length) {
            String name2 = linkresource2[i9];
            if (!name2.startsWith(CookieSpec.PATH_DELIM)) {
                String s3 = cls.getName().replace('.', c);
                int n2 = s3.lastIndexOf(c);
                if (n2 >= 0) {
                    name2 = s3.substring(0, n2 + 1) + name2;
                }
                linkresource2[i9] = CookieSpec.PATH_DELIM + name2;
            }
            i9++;
            c = IOUtils.DIR_SEPARATOR_UNIX;
        }
        addAll("platform.pragma", pragma2);
        addAll("platform.define", define);
        addAll("platform.exclude", exclude);
        addAll("platform.include", include);
        addAll("platform.cinclude", cinclude);
        addAll("platform.includepath", includepath);
        addAll("platform.includeresource", includeresource);
        addAll("platform.compiler.*", compiler4);
        addAll("platform.linkpath", linkpath6);
        addAll("platform.linkresource", linkresource2);
        addAll("platform.link", link4);
        addAll("platform.frameworkpath", frameworkpath4);
        String[] framework3 = framework2;
        addAll("platform.framework", framework3);
        addAll("platform.preloadresource", preloadresource2);
        String[] strArr4 = framework3;
        String[] preloadpath3 = preloadpath2;
        addAll("platform.preloadpath", preloadpath3);
        String[] strArr5 = preloadpath3;
        String[] preloadpath4 = preload2;
        addAll("platform.preload", preloadpath4);
        String[] strArr6 = preloadpath4;
        String[] resourcepath3 = resourcepath2;
        addAll("platform.resourcepath", resourcepath3);
        String[] strArr7 = resourcepath3;
        String[] resourcepath4 = resource2;
        addAll("platform.resource", resourcepath4);
        String[] strArr8 = resourcepath4;
        String[] extension4 = extension2;
        addAll("platform.extension", extension4);
        String[] strArr9 = extension4;
        String library4 = library3;
        setProperty("platform.library", library4);
        String str = library4;
        Class<?> c5 = c4;
        try {
            if (LoadEnabled.class.isAssignableFrom(c5)) {
                ((LoadEnabled) c5.newInstance()).init(this);
            }
        } catch (ClassCastException | IllegalAccessException | InstantiationException e) {
        }
        if (platforms3 != null) {
            Class<?> cls2 = c5;
            Platform[] platforms4 = platforms3;
            if (platforms4.length > 0) {
                Platform[] platformArr3 = platforms4;
                this.loaded = true;
                return;
            }
            return;
        }
        Platform[] platformArr4 = platforms3;
    }

    public List<Class> getInheritedClasses() {
        return this.inheritedClasses;
    }

    public List<Class> getEffectiveClasses() {
        return this.effectiveClasses;
    }

    public boolean isLoaded() {
        return this.loaded;
    }
}
