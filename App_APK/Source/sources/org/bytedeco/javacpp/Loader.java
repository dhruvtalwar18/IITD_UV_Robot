package org.bytedeco.javacpp;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.JarURLConnection;
import java.net.URI;
import java.net.URISyntaxException;
import java.net.URL;
import java.net.URLConnection;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import java.util.WeakHashMap;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;
import org.apache.commons.httpclient.HttpState;
import org.apache.commons.httpclient.cookie.CookieSpec;
import org.apache.commons.io.IOUtils;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Name;
import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.tools.Logger;

public class Loader {
    private static final String PLATFORM;
    static File cacheDir = null;
    static Map<String, String> loadedLibraries = new HashMap();
    private static final Logger logger = Logger.create(Loader.class);
    static WeakHashMap<Class<? extends Pointer>, HashMap<String, Integer>> memberOffsets = new WeakHashMap<>();
    static boolean pathsFirst;
    private static Properties platformProperties = null;
    static File tempDir = null;

    @Name({"JavaCPP_addressof"})
    public static native Pointer addressof(String str);

    @Cast({"JavaVM*"})
    @Name({"JavaCPP_getJavaVM"})
    public static native Pointer getJavaVM();

    @Name({"JavaCPP_totalChips"})
    public static native int totalChips();

    @Name({"JavaCPP_totalCores"})
    public static native int totalCores();

    @Name({"JavaCPP_totalProcessors"})
    public static native int totalProcessors();

    static {
        String jvmName = System.getProperty("java.vm.name", "").toLowerCase();
        String osName = System.getProperty("os.name", "").toLowerCase();
        String osArch = System.getProperty("os.arch", "").toLowerCase();
        String abiType = System.getProperty("sun.arch.abi", "").toLowerCase();
        String libPath = System.getProperty("sun.boot.library.path", "").toLowerCase();
        boolean z = false;
        if (jvmName.startsWith("dalvik") && osName.startsWith("linux")) {
            osName = "android";
        } else if (jvmName.startsWith("robovm") && osName.startsWith("darwin")) {
            osName = "ios";
            osArch = "arm";
        } else if (osName.startsWith("mac os x") || osName.startsWith("darwin")) {
            osName = "macosx";
        } else {
            int spaceIndex = osName.indexOf(32);
            if (spaceIndex > 0) {
                osName = osName.substring(0, spaceIndex);
            }
        }
        if (osArch.equals("i386") || osArch.equals("i486") || osArch.equals("i586") || osArch.equals("i686")) {
            osArch = "x86";
        } else if (osArch.equals("amd64") || osArch.equals("x86-64") || osArch.equals("x64")) {
            osArch = "x86_64";
        } else if (osArch.startsWith("aarch64") || osArch.startsWith("armv8") || osArch.startsWith("arm64")) {
            osArch = "arm64";
        } else if (osArch.startsWith("arm") && (abiType.equals("gnueabihf") || libPath.contains("openjdk-armhf"))) {
            osArch = "armhf";
        } else if (osArch.startsWith("arm")) {
            osArch = "arm";
        }
        PLATFORM = osName + "-" + osArch;
        pathsFirst = false;
        String s = System.getProperty("org.bytedeco.javacpp.pathsfirst", HttpState.PREEMPTIVE_DEFAULT).toLowerCase();
        if (s.equals("true") || s.equals("t") || s.equals("")) {
            z = true;
        }
        pathsFirst = z;
    }

    public static String getPlatform() {
        return System.getProperty("org.bytedeco.javacpp.platform", PLATFORM);
    }

    public static Properties loadProperties() {
        String name = getPlatform();
        if (platformProperties != null && name.equals(platformProperties.getProperty("platform"))) {
            return platformProperties;
        }
        Properties loadProperties = loadProperties(name, (String) null);
        platformProperties = loadProperties;
        return loadProperties;
    }

    /* JADX WARNING: Removed duplicated region for block: B:46:0x0122 A[SYNTHETIC, Splitter:B:46:0x0122] */
    /* JADX WARNING: Removed duplicated region for block: B:53:0x0143  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public static java.util.Properties loadProperties(java.lang.String r11, java.lang.String r12) {
        /*
            if (r12 != 0) goto L_0x0004
            java.lang.String r12 = "generic"
        L_0x0004:
            java.util.Properties r0 = new java.util.Properties
            r0.<init>()
            java.lang.String r1 = "platform"
            r0.put(r1, r11)
            java.lang.String r1 = "platform.path.separator"
            java.lang.String r2 = java.io.File.pathSeparator
            r0.put(r1, r2)
            java.lang.String r1 = "/"
            java.lang.String r1 = java.lang.System.mapLibraryName(r1)
            r2 = 47
            int r2 = r1.indexOf(r2)
            java.lang.String r3 = "platform.library.prefix"
            r4 = 0
            java.lang.String r4 = r1.substring(r4, r2)
            r0.put(r3, r4)
            java.lang.String r3 = "platform.library.suffix"
            int r4 = r2 + 1
            java.lang.String r4 = r1.substring(r4)
            r0.put(r3, r4)
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = "properties/"
            r3.append(r4)
            r3.append(r11)
            java.lang.String r4 = ".properties"
            r3.append(r4)
            java.lang.String r11 = r3.toString()
            java.lang.Class<org.bytedeco.javacpp.Loader> r3 = org.bytedeco.javacpp.Loader.class
            java.io.InputStream r3 = r3.getResourceAsStream(r11)
            java.io.InputStreamReader r4 = new java.io.InputStreamReader     // Catch:{ NoSuchMethodError -> 0x0060 }
            r4.<init>(r3)     // Catch:{ NoSuchMethodError -> 0x0060 }
            r0.load(r4)     // Catch:{ NoSuchMethodError -> 0x0060 }
            goto L_0x0064
        L_0x005b:
            r4 = move-exception
            goto L_0x0170
        L_0x005e:
            r4 = move-exception
            goto L_0x0089
        L_0x0060:
            r4 = move-exception
            r0.load(r3)     // Catch:{ Exception -> 0x005e }
        L_0x0064:
            if (r3 == 0) goto L_0x0087
            r3.close()     // Catch:{ IOException -> 0x006a }
            goto L_0x0087
        L_0x006a:
            r4 = move-exception
            org.bytedeco.javacpp.tools.Logger r5 = logger
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
        L_0x0072:
            java.lang.String r7 = "Unable to close resource : "
            r6.append(r7)
            java.lang.String r7 = r4.getMessage()
            r6.append(r7)
            java.lang.String r6 = r6.toString()
            r5.error(r6)
            goto L_0x0131
        L_0x0087:
            goto L_0x0131
        L_0x0089:
            java.lang.StringBuilder r5 = new java.lang.StringBuilder     // Catch:{ all -> 0x005b }
            r5.<init>()     // Catch:{ all -> 0x005b }
            java.lang.String r6 = "properties/"
            r5.append(r6)     // Catch:{ all -> 0x005b }
            r5.append(r12)     // Catch:{ all -> 0x005b }
            java.lang.String r6 = ".properties"
            r5.append(r6)     // Catch:{ all -> 0x005b }
            java.lang.String r5 = r5.toString()     // Catch:{ all -> 0x005b }
            r11 = r5
            java.lang.Class<org.bytedeco.javacpp.Loader> r5 = org.bytedeco.javacpp.Loader.class
            java.io.InputStream r5 = r5.getResourceAsStream(r11)     // Catch:{ all -> 0x005b }
            java.io.InputStreamReader r6 = new java.io.InputStreamReader     // Catch:{ NoSuchMethodError -> 0x00b4 }
            r6.<init>(r5)     // Catch:{ NoSuchMethodError -> 0x00b4 }
            r0.load(r6)     // Catch:{ NoSuchMethodError -> 0x00b4 }
            goto L_0x00b8
        L_0x00b0:
            r6 = move-exception
            goto L_0x00db
        L_0x00b2:
            r6 = move-exception
            goto L_0x0100
        L_0x00b4:
            r6 = move-exception
            r0.load(r5)     // Catch:{ Exception -> 0x00b2, all -> 0x00b0 }
        L_0x00b8:
            if (r5 == 0) goto L_0x00da
            r5.close()     // Catch:{ IOException -> 0x00be }
            goto L_0x00da
        L_0x00be:
            r6 = move-exception
            org.bytedeco.javacpp.tools.Logger r7 = logger     // Catch:{ all -> 0x005b }
            java.lang.StringBuilder r8 = new java.lang.StringBuilder     // Catch:{ all -> 0x005b }
            r8.<init>()     // Catch:{ all -> 0x005b }
            java.lang.String r9 = "Unable to close resource : "
            r8.append(r9)     // Catch:{ all -> 0x005b }
            java.lang.String r9 = r6.getMessage()     // Catch:{ all -> 0x005b }
            r8.append(r9)     // Catch:{ all -> 0x005b }
            java.lang.String r8 = r8.toString()     // Catch:{ all -> 0x005b }
        L_0x00d6:
            r7.error(r8)     // Catch:{ all -> 0x005b }
            goto L_0x0120
        L_0x00da:
            goto L_0x0120
        L_0x00db:
            if (r5 == 0) goto L_0x00fe
            r5.close()     // Catch:{ IOException -> 0x00e2 }
            goto L_0x00fe
        L_0x00e2:
            r7 = move-exception
            org.bytedeco.javacpp.tools.Logger r8 = logger     // Catch:{ all -> 0x005b }
            java.lang.StringBuilder r9 = new java.lang.StringBuilder     // Catch:{ all -> 0x005b }
            r9.<init>()     // Catch:{ all -> 0x005b }
            java.lang.String r10 = "Unable to close resource : "
            r9.append(r10)     // Catch:{ all -> 0x005b }
            java.lang.String r10 = r7.getMessage()     // Catch:{ all -> 0x005b }
            r9.append(r10)     // Catch:{ all -> 0x005b }
            java.lang.String r9 = r9.toString()     // Catch:{ all -> 0x005b }
            r8.error(r9)     // Catch:{ all -> 0x005b }
            goto L_0x00ff
        L_0x00fe:
        L_0x00ff:
            throw r6     // Catch:{ all -> 0x005b }
        L_0x0100:
            if (r5 == 0) goto L_0x00da
            r5.close()     // Catch:{ IOException -> 0x0107 }
            goto L_0x00da
        L_0x0107:
            r6 = move-exception
            org.bytedeco.javacpp.tools.Logger r7 = logger     // Catch:{ all -> 0x005b }
            java.lang.StringBuilder r8 = new java.lang.StringBuilder     // Catch:{ all -> 0x005b }
            r8.<init>()     // Catch:{ all -> 0x005b }
            java.lang.String r9 = "Unable to close resource : "
            r8.append(r9)     // Catch:{ all -> 0x005b }
            java.lang.String r9 = r6.getMessage()     // Catch:{ all -> 0x005b }
            r8.append(r9)     // Catch:{ all -> 0x005b }
            java.lang.String r8 = r8.toString()     // Catch:{ all -> 0x005b }
            goto L_0x00d6
        L_0x0120:
            if (r3 == 0) goto L_0x0087
            r3.close()     // Catch:{ IOException -> 0x0127 }
            goto L_0x0087
        L_0x0127:
            r4 = move-exception
            org.bytedeco.javacpp.tools.Logger r5 = logger
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            goto L_0x0072
        L_0x0131:
            java.util.Properties r4 = java.lang.System.getProperties()
            java.util.Set r4 = r4.entrySet()
            java.util.Iterator r4 = r4.iterator()
        L_0x013d:
            boolean r5 = r4.hasNext()
            if (r5 == 0) goto L_0x016f
            java.lang.Object r5 = r4.next()
            java.util.Map$Entry r5 = (java.util.Map.Entry) r5
            java.lang.Object r6 = r5.getKey()
            java.lang.String r6 = (java.lang.String) r6
            java.lang.Object r7 = r5.getValue()
            java.lang.String r7 = (java.lang.String) r7
            if (r6 == 0) goto L_0x016e
            if (r7 == 0) goto L_0x016e
            java.lang.String r8 = "org.bytedeco.javacpp.platform."
            boolean r8 = r6.startsWith(r8)
            if (r8 == 0) goto L_0x016e
            java.lang.String r8 = "platform."
            int r8 = r6.indexOf(r8)
            java.lang.String r8 = r6.substring(r8)
            r0.put(r8, r7)
        L_0x016e:
            goto L_0x013d
        L_0x016f:
            return r0
        L_0x0170:
            if (r3 == 0) goto L_0x0193
            r3.close()     // Catch:{ IOException -> 0x0177 }
            goto L_0x0193
        L_0x0177:
            r5 = move-exception
            org.bytedeco.javacpp.tools.Logger r6 = logger
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r8 = "Unable to close resource : "
            r7.append(r8)
            java.lang.String r8 = r5.getMessage()
            r7.append(r8)
            java.lang.String r7 = r7.toString()
            r6.error(r7)
            goto L_0x0194
        L_0x0193:
        L_0x0194:
            throw r4
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.Loader.loadProperties(java.lang.String, java.lang.String):java.util.Properties");
    }

    public static Class getEnclosingClass(Class cls) {
        Class cls2 = cls;
        while (cls2.getEnclosingClass() != null && !cls2.isAnnotationPresent(org.bytedeco.javacpp.annotation.Properties.class)) {
            if (cls2.isAnnotationPresent(Platform.class)) {
                Platform p = (Platform) cls2.getAnnotation(Platform.class);
                if (p.pragma().length > 0 || p.define().length > 0 || p.exclude().length > 0 || p.include().length > 0 || p.cinclude().length > 0 || p.includepath().length > 0 || p.includeresource().length > 0 || p.compiler().length > 0 || p.linkpath().length > 0 || p.linkresource().length > 0 || p.link().length > 0 || p.frameworkpath().length > 0 || p.framework().length > 0 || p.preloadresource().length > 0 || p.preloadpath().length > 0 || p.preload().length > 0 || p.resourcepath().length > 0 || p.resource().length > 0 || p.library().length() > 0) {
                    break;
                }
            }
            cls2 = cls2.getEnclosingClass();
        }
        return cls2;
    }

    public static ClassProperties loadProperties(Class[] cls, Properties properties, boolean inherit) {
        ClassProperties cp = new ClassProperties(properties);
        if (cls != null) {
            for (Class c : cls) {
                cp.load(c, inherit);
            }
        }
        return cp;
    }

    public static ClassProperties loadProperties(Class cls, Properties properties, boolean inherit) {
        ClassProperties cp = new ClassProperties(properties);
        if (cls != null) {
            cp.load(cls, inherit);
        }
        return cp;
    }

    public static Class getCallerClass(int i) {
        Class[] classContext = null;
        try {
            classContext = new SecurityManager() {
                public Class[] getClassContext() {
                    return super.getClassContext();
                }
            }.getClassContext();
        } catch (NoSuchMethodError | SecurityException e) {
            logger.warn("Could not create an instance of SecurityManager: " + e.getMessage());
        }
        int j = 0;
        if (classContext != null) {
            while (j < classContext.length) {
                if (classContext[j] == Loader.class) {
                    return classContext[i + j];
                }
                j++;
            }
        } else {
            try {
                StackTraceElement[] classNames = Thread.currentThread().getStackTrace();
                while (j < classNames.length) {
                    if (Class.forName(classNames[j].getClassName()) == Loader.class) {
                        return Class.forName(classNames[i + j].getClassName());
                    }
                    j++;
                }
            } catch (ClassNotFoundException e2) {
                logger.error("No definition for the class found : " + e2.getMessage());
            }
        }
        return null;
    }

    public static File cacheResource(String name) throws IOException {
        return cacheResource(getCallerClass(2), name);
    }

    public static File cacheResource(Class cls, String name) throws IOException {
        return cacheResource(findResource(cls, name));
    }

    public static File[] cacheResources(String name) throws IOException {
        return cacheResources(getCallerClass(2), name);
    }

    public static File[] cacheResources(Class cls, String name) throws IOException {
        URL[] urls = findResources(cls, name);
        File[] files = new File[urls.length];
        for (int i = 0; i < urls.length; i++) {
            files[i] = cacheResource(urls[i]);
        }
        return files;
    }

    public static File cacheResource(URL resourceURL) throws IOException {
        return cacheResource(resourceURL, (String) null);
    }

    /*  JADX ERROR: IndexOutOfBoundsException in pass: RegionMakerVisitor
        java.lang.IndexOutOfBoundsException: Index: 0, Size: 0
        	at java.util.ArrayList.rangeCheck(ArrayList.java:659)
        	at java.util.ArrayList.get(ArrayList.java:435)
        	at jadx.core.dex.nodes.InsnNode.getArg(InsnNode.java:101)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:611)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.processMonitorEnter(RegionMaker.java:561)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverse(RegionMaker.java:133)
        	at jadx.core.dex.visitors.regions.RegionMaker.makeRegion(RegionMaker.java:86)
        	at jadx.core.dex.visitors.regions.RegionMaker.processExcHandler(RegionMaker.java:1043)
        	at jadx.core.dex.visitors.regions.RegionMaker.processTryCatchBlocks(RegionMaker.java:975)
        	at jadx.core.dex.visitors.regions.RegionMakerVisitor.visit(RegionMakerVisitor.java:52)
        */
    /* JADX WARNING: Removed duplicated region for block: B:119:0x026b A[SYNTHETIC, Splitter:B:119:0x026b] */
    /* JADX WARNING: Removed duplicated region for block: B:122:0x0270 A[Catch:{ all -> 0x02c5 }] */
    /* JADX WARNING: Removed duplicated region for block: B:228:0x03e2 A[SYNTHETIC, Splitter:B:228:0x03e2] */
    /* JADX WARNING: Removed duplicated region for block: B:231:0x03e7 A[Catch:{ all -> 0x0439 }] */
    public static java.io.File cacheResource(java.net.URL r29, java.lang.String r30) throws java.io.IOException {
        /*
            r1 = r29
            r2 = r30
            r3 = 0
            java.io.File r0 = new java.io.File     // Catch:{ IllegalArgumentException | URISyntaxException -> 0x001c }
            java.net.URI r4 = new java.net.URI     // Catch:{ IllegalArgumentException | URISyntaxException -> 0x001c }
            java.lang.String r5 = r29.toString()     // Catch:{ IllegalArgumentException | URISyntaxException -> 0x001c }
            java.lang.String r6 = "#"
            java.lang.String[] r5 = r5.split(r6)     // Catch:{ IllegalArgumentException | URISyntaxException -> 0x001c }
            r5 = r5[r3]     // Catch:{ IllegalArgumentException | URISyntaxException -> 0x001c }
            r4.<init>(r5)     // Catch:{ IllegalArgumentException | URISyntaxException -> 0x001c }
            r0.<init>(r4)     // Catch:{ IllegalArgumentException | URISyntaxException -> 0x001c }
            goto L_0x0027
        L_0x001c:
            r0 = move-exception
            java.io.File r4 = new java.io.File
            java.lang.String r5 = r29.getPath()
            r4.<init>(r5)
            r0 = r4
        L_0x0027:
            r4 = r0
            java.lang.String r0 = r4.getName()
            r5 = 0
            java.io.File r6 = getCacheDir()
            java.io.File r7 = r6.getCanonicalFile()
            java.lang.String r8 = "org.bytedeco.javacpp.cachedir.nosubdir"
            java.lang.String r9 = "false"
            java.lang.String r8 = java.lang.System.getProperty(r8, r9)
            java.lang.String r8 = r8.toLowerCase()
            java.lang.String r9 = "true"
            boolean r9 = r8.equals(r9)
            r10 = 1
            if (r9 != 0) goto L_0x005d
            java.lang.String r9 = "t"
            boolean r9 = r8.equals(r9)
            if (r9 != 0) goto L_0x005d
            java.lang.String r9 = ""
            boolean r9 = r8.equals(r9)
            if (r9 == 0) goto L_0x005b
            goto L_0x005d
        L_0x005b:
            r9 = 0
            goto L_0x005e
        L_0x005d:
            r9 = 1
        L_0x005e:
            java.net.URLConnection r11 = r29.openConnection()
            boolean r12 = r11 instanceof java.net.JarURLConnection
            if (r12 == 0) goto L_0x00c7
            r10 = r11
            java.net.JarURLConnection r10 = (java.net.JarURLConnection) r10
            java.util.jar.JarFile r10 = r10.getJarFile()
            r12 = r11
            java.net.JarURLConnection r12 = (java.net.JarURLConnection) r12
            java.util.jar.JarEntry r12 = r12.getJarEntry()
            java.io.File r13 = new java.io.File
            java.lang.String r14 = r10.getName()
            r13.<init>(r14)
            java.io.File r14 = new java.io.File
            java.lang.String r15 = r12.getName()
            r14.<init>(r15)
            long r15 = r12.getSize()
            long r17 = r12.getTime()
            if (r9 != 0) goto L_0x00c0
            java.lang.String r3 = r13.getName()
            r19 = r5
            java.lang.String r5 = r14.getParent()
            if (r5 == 0) goto L_0x00b5
            r20 = r8
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            r8.append(r3)
            r21 = r3
            java.lang.String r3 = java.io.File.separator
            r8.append(r3)
            r8.append(r5)
            java.lang.String r3 = r8.toString()
            goto L_0x00b9
        L_0x00b5:
            r21 = r3
            r20 = r8
        L_0x00b9:
            java.io.File r8 = new java.io.File
            r8.<init>(r7, r3)
            r7 = r8
            goto L_0x00c4
        L_0x00c0:
            r19 = r5
            r20 = r8
        L_0x00c4:
            r12 = r17
            goto L_0x011f
        L_0x00c7:
            r19 = r5
            r20 = r8
            boolean r3 = r11 instanceof java.net.HttpURLConnection
            if (r3 == 0) goto L_0x0106
            int r3 = r11.getContentLength()
            long r12 = (long) r3
            long r17 = r11.getLastModified()
            if (r9 != 0) goto L_0x0104
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r5 = r29.getHost()
            r3.append(r5)
            java.lang.String r5 = r29.getPath()
            r3.append(r5)
            java.lang.String r3 = r3.toString()
            java.io.File r5 = new java.io.File
            r8 = 47
            int r8 = r3.lastIndexOf(r8)
            int r8 = r8 + r10
            r10 = 0
            java.lang.String r8 = r3.substring(r10, r8)
            r5.<init>(r7, r8)
            r7 = r5
        L_0x0104:
            r15 = r12
            goto L_0x00c4
        L_0x0106:
            long r15 = r4.length()
            long r17 = r4.lastModified()
            if (r9 != 0) goto L_0x00c4
            java.io.File r3 = new java.io.File
            java.io.File r5 = r4.getParentFile()
            java.lang.String r5 = r5.getName()
            r3.<init>(r7, r5)
            r7 = r3
            goto L_0x00c4
        L_0x011f:
            java.lang.String r3 = r29.getRef()
            if (r3 == 0) goto L_0x0131
            java.lang.String r3 = r29.getRef()
            boolean r5 = r3.equals(r0)
            r0 = r3
            r19 = r5
            goto L_0x0132
        L_0x0131:
            r3 = r0
        L_0x0132:
            java.io.File r0 = new java.io.File
            r0.<init>(r7, r3)
            r5 = r0
            java.io.File r0 = new java.io.File
            java.lang.String r8 = ".lock"
            r0.<init>(r6, r8)
            r8 = r0
            r10 = 0
            r14 = 0
            r22 = r3
            if (r2 == 0) goto L_0x02cf
            int r0 = r30.length()
            if (r0 <= 0) goto L_0x02cf
            java.lang.Runtime r17 = java.lang.Runtime.getRuntime()
            monitor-enter(r17)
            java.nio.file.Path r0 = r5.toPath()     // Catch:{ IOException | RuntimeException -> 0x027e, all -> 0x0278 }
            r23 = r0
            r3 = 0
            java.lang.String[] r0 = new java.lang.String[r3]     // Catch:{ IOException | RuntimeException -> 0x027e, all -> 0x0278 }
            java.nio.file.Path r0 = java.nio.file.Paths.get(r2, r0)     // Catch:{ IOException | RuntimeException -> 0x027e, all -> 0x0278 }
            r3 = r0
            boolean r0 = r5.exists()     // Catch:{ IOException | RuntimeException -> 0x027e, all -> 0x0278 }
            if (r0 == 0) goto L_0x0186
            boolean r0 = java.nio.file.Files.isSymbolicLink(r23)     // Catch:{ IOException | RuntimeException -> 0x0181, all -> 0x017c }
            if (r0 == 0) goto L_0x0186
            java.nio.file.Path r0 = java.nio.file.Files.readSymbolicLink(r23)     // Catch:{ IOException | RuntimeException -> 0x0181, all -> 0x017c }
            boolean r0 = r0.equals(r3)     // Catch:{ IOException | RuntimeException -> 0x0181, all -> 0x017c }
            if (r0 != 0) goto L_0x0176
            goto L_0x0186
        L_0x0176:
            r24 = r9
            r25 = r10
            goto L_0x0269
        L_0x017c:
            r0 = move-exception
            r24 = r9
            goto L_0x02bd
        L_0x0181:
            r0 = move-exception
            r24 = r9
            goto L_0x0283
        L_0x0186:
            boolean r0 = r3.isAbsolute()     // Catch:{ IOException | RuntimeException -> 0x027e, all -> 0x0278 }
            if (r0 == 0) goto L_0x0265
            r2 = r23
            boolean r0 = r3.equals(r2)     // Catch:{ IOException | RuntimeException -> 0x027e, all -> 0x0278 }
            if (r0 != 0) goto L_0x0265
            org.bytedeco.javacpp.tools.Logger r0 = logger     // Catch:{ IOException | RuntimeException -> 0x027e, all -> 0x0278 }
            boolean r0 = r0.isDebugEnabled()     // Catch:{ IOException | RuntimeException -> 0x027e, all -> 0x0278 }
            if (r0 == 0) goto L_0x01c6
            org.bytedeco.javacpp.tools.Logger r0 = logger     // Catch:{ IOException | RuntimeException -> 0x027e, all -> 0x0278 }
            r24 = r9
            java.lang.StringBuilder r9 = new java.lang.StringBuilder     // Catch:{ IOException | RuntimeException -> 0x01c1, all -> 0x01bc }
            r9.<init>()     // Catch:{ IOException | RuntimeException -> 0x01c1, all -> 0x01bc }
            r25 = r10
            java.lang.String r10 = "Locking "
            r9.append(r10)     // Catch:{ IOException | RuntimeException -> 0x0261, all -> 0x025f }
            r9.append(r6)     // Catch:{ IOException | RuntimeException -> 0x0261, all -> 0x025f }
            java.lang.String r10 = " to create symbolic link"
            r9.append(r10)     // Catch:{ IOException | RuntimeException -> 0x0261, all -> 0x025f }
            java.lang.String r9 = r9.toString()     // Catch:{ IOException | RuntimeException -> 0x0261, all -> 0x025f }
            r0.debug(r9)     // Catch:{ IOException | RuntimeException -> 0x0261, all -> 0x025f }
            goto L_0x01ca
        L_0x01bc:
            r0 = move-exception
            r25 = r10
            goto L_0x02bf
        L_0x01c1:
            r0 = move-exception
            r25 = r10
            goto L_0x0283
        L_0x01c6:
            r24 = r9
            r25 = r10
        L_0x01ca:
            java.io.FileOutputStream r0 = new java.io.FileOutputStream     // Catch:{ IOException | RuntimeException -> 0x0261, all -> 0x025f }
            r0.<init>(r8)     // Catch:{ IOException | RuntimeException -> 0x0261, all -> 0x025f }
            java.nio.channels.FileChannel r0 = r0.getChannel()     // Catch:{ IOException | RuntimeException -> 0x0261, all -> 0x025f }
            r10 = r0
            java.nio.channels.FileLock r0 = r10.lock()     // Catch:{ IOException | RuntimeException -> 0x025b, all -> 0x0254 }
            r14 = r0
            boolean r0 = r5.exists()     // Catch:{ IOException | RuntimeException -> 0x025b, all -> 0x0254 }
            if (r0 == 0) goto L_0x01f6
            boolean r0 = java.nio.file.Files.isSymbolicLink(r2)     // Catch:{ IOException | RuntimeException -> 0x01f3 }
            if (r0 == 0) goto L_0x01f6
            java.nio.file.Path r0 = java.nio.file.Files.readSymbolicLink(r2)     // Catch:{ IOException | RuntimeException -> 0x01f3 }
            boolean r0 = r0.equals(r3)     // Catch:{ IOException | RuntimeException -> 0x01f3 }
            if (r0 != 0) goto L_0x01f0
            goto L_0x01f6
        L_0x01f0:
            r26 = r10
            goto L_0x0251
        L_0x01f3:
            r0 = move-exception
            goto L_0x0283
        L_0x01f6:
            boolean r0 = r3.isAbsolute()     // Catch:{ IOException | RuntimeException -> 0x025b, all -> 0x0254 }
            if (r0 == 0) goto L_0x024f
            boolean r0 = r3.equals(r2)     // Catch:{ IOException | RuntimeException -> 0x025b, all -> 0x0254 }
            if (r0 != 0) goto L_0x024f
            org.bytedeco.javacpp.tools.Logger r0 = logger     // Catch:{ IOException | RuntimeException -> 0x025b, all -> 0x0254 }
            boolean r0 = r0.isDebugEnabled()     // Catch:{ IOException | RuntimeException -> 0x025b, all -> 0x0254 }
            if (r0 == 0) goto L_0x022b
            org.bytedeco.javacpp.tools.Logger r0 = logger     // Catch:{ IOException | RuntimeException -> 0x025b, all -> 0x0254 }
            java.lang.StringBuilder r9 = new java.lang.StringBuilder     // Catch:{ IOException | RuntimeException -> 0x025b, all -> 0x0254 }
            r9.<init>()     // Catch:{ IOException | RuntimeException -> 0x025b, all -> 0x0254 }
            r26 = r10
            java.lang.String r10 = "Creating symbolic link "
            r9.append(r10)     // Catch:{ IOException | RuntimeException -> 0x0240, all -> 0x023b }
            r9.append(r2)     // Catch:{ IOException | RuntimeException -> 0x0240, all -> 0x023b }
            java.lang.String r10 = " to "
            r9.append(r10)     // Catch:{ IOException | RuntimeException -> 0x0240, all -> 0x023b }
            r9.append(r3)     // Catch:{ IOException | RuntimeException -> 0x0240, all -> 0x023b }
            java.lang.String r9 = r9.toString()     // Catch:{ IOException | RuntimeException -> 0x0240, all -> 0x023b }
            r0.debug(r9)     // Catch:{ IOException | RuntimeException -> 0x0240, all -> 0x023b }
            goto L_0x022d
        L_0x022b:
            r26 = r10
        L_0x022d:
            java.io.File r0 = r5.getParentFile()     // Catch:{ FileAlreadyExistsException -> 0x0244, IOException | RuntimeException -> 0x0240 }
            r0.mkdirs()     // Catch:{ FileAlreadyExistsException -> 0x0244, IOException | RuntimeException -> 0x0240 }
            r9 = 0
            java.nio.file.attribute.FileAttribute[] r0 = new java.nio.file.attribute.FileAttribute[r9]     // Catch:{ FileAlreadyExistsException -> 0x0244, IOException | RuntimeException -> 0x0240 }
            java.nio.file.Files.createSymbolicLink(r2, r3, r0)     // Catch:{ FileAlreadyExistsException -> 0x0244, IOException | RuntimeException -> 0x0240 }
            goto L_0x0251
        L_0x023b:
            r0 = move-exception
            r25 = r26
            goto L_0x02bf
        L_0x0240:
            r0 = move-exception
            r10 = r26
            goto L_0x0283
        L_0x0244:
            r0 = move-exception
            r5.delete()     // Catch:{ IOException | RuntimeException -> 0x0240, all -> 0x023b }
            r9 = 0
            java.nio.file.attribute.FileAttribute[] r9 = new java.nio.file.attribute.FileAttribute[r9]     // Catch:{ IOException | RuntimeException -> 0x0240, all -> 0x023b }
            java.nio.file.Files.createSymbolicLink(r2, r3, r9)     // Catch:{ IOException | RuntimeException -> 0x0240, all -> 0x023b }
            goto L_0x0251
        L_0x024f:
            r26 = r10
        L_0x0251:
            r25 = r26
            goto L_0x0269
        L_0x0254:
            r0 = move-exception
            r26 = r10
            r25 = r26
            goto L_0x02bf
        L_0x025b:
            r0 = move-exception
            r26 = r10
            goto L_0x0283
        L_0x025f:
            r0 = move-exception
            goto L_0x02bf
        L_0x0261:
            r0 = move-exception
            r10 = r25
            goto L_0x0283
        L_0x0265:
            r24 = r9
            r25 = r10
        L_0x0269:
            if (r14 == 0) goto L_0x026e
            r14.release()     // Catch:{ all -> 0x02c5 }
        L_0x026e:
            if (r25 == 0) goto L_0x0273
            r25.close()     // Catch:{ all -> 0x02c5 }
        L_0x0273:
            monitor-exit(r17)     // Catch:{ all -> 0x02c5 }
            r27 = r4
            goto L_0x04f8
        L_0x0278:
            r0 = move-exception
            r24 = r9
            r25 = r10
            goto L_0x02bf
        L_0x027e:
            r0 = move-exception
            r24 = r9
            r25 = r10
        L_0x0283:
            org.bytedeco.javacpp.tools.Logger r2 = logger     // Catch:{ all -> 0x02bc }
            boolean r2 = r2.isDebugEnabled()     // Catch:{ all -> 0x02bc }
            if (r2 == 0) goto L_0x02a9
            org.bytedeco.javacpp.tools.Logger r2 = logger     // Catch:{ all -> 0x02bc }
            java.lang.StringBuilder r3 = new java.lang.StringBuilder     // Catch:{ all -> 0x02bc }
            r3.<init>()     // Catch:{ all -> 0x02bc }
            java.lang.String r9 = "Failed to create symbolic link "
            r3.append(r9)     // Catch:{ all -> 0x02bc }
            r3.append(r5)     // Catch:{ all -> 0x02bc }
            java.lang.String r9 = ": "
            r3.append(r9)     // Catch:{ all -> 0x02bc }
            r3.append(r0)     // Catch:{ all -> 0x02bc }
            java.lang.String r3 = r3.toString()     // Catch:{ all -> 0x02bc }
            r2.debug(r3)     // Catch:{ all -> 0x02bc }
        L_0x02a9:
            if (r14 == 0) goto L_0x02b4
            r14.release()     // Catch:{ all -> 0x02b0 }
            goto L_0x02b4
        L_0x02b0:
            r0 = move-exception
            r25 = r10
            goto L_0x02cd
        L_0x02b4:
            if (r10 == 0) goto L_0x02b9
            r10.close()     // Catch:{ all -> 0x02b0 }
        L_0x02b9:
            monitor-exit(r17)     // Catch:{ all -> 0x02b0 }
            r2 = 0
            return r2
        L_0x02bc:
            r0 = move-exception
        L_0x02bd:
            r25 = r10
        L_0x02bf:
            if (r14 == 0) goto L_0x02c7
            r14.release()     // Catch:{ all -> 0x02c5 }
            goto L_0x02c7
        L_0x02c5:
            r0 = move-exception
            goto L_0x02cd
        L_0x02c7:
            if (r25 == 0) goto L_0x02cc
            r25.close()     // Catch:{ all -> 0x02c5 }
        L_0x02cc:
            throw r0     // Catch:{ all -> 0x02c5 }
        L_0x02cd:
            monitor-exit(r17)     // Catch:{ all -> 0x02c5 }
            throw r0
        L_0x02cf:
            r24 = r9
            r25 = r10
            boolean r0 = r4.exists()
            if (r0 == 0) goto L_0x0443
            if (r19 == 0) goto L_0x0443
            java.lang.Runtime r2 = java.lang.Runtime.getRuntime()
            monitor-enter(r2)
            java.nio.file.Path r0 = r5.toPath()     // Catch:{ IOException | RuntimeException -> 0x03f2, all -> 0x03ec }
            r3 = r0
            java.nio.file.Path r0 = r4.toPath()     // Catch:{ IOException | RuntimeException -> 0x03f2, all -> 0x03ec }
            r9 = r0
            boolean r0 = r5.exists()     // Catch:{ IOException | RuntimeException -> 0x03f2, all -> 0x03ec }
            if (r0 == 0) goto L_0x0313
            boolean r0 = java.nio.file.Files.isSymbolicLink(r3)     // Catch:{ IOException | RuntimeException -> 0x030c, all -> 0x0305 }
            if (r0 == 0) goto L_0x0313
            java.nio.file.Path r0 = java.nio.file.Files.readSymbolicLink(r3)     // Catch:{ IOException | RuntimeException -> 0x030c, all -> 0x0305 }
            boolean r0 = r0.equals(r9)     // Catch:{ IOException | RuntimeException -> 0x030c, all -> 0x0305 }
            if (r0 != 0) goto L_0x0301
            goto L_0x0313
        L_0x0301:
            r27 = r4
            goto L_0x03de
        L_0x0305:
            r0 = move-exception
            r27 = r4
        L_0x0308:
            r28 = r25
            goto L_0x0433
        L_0x030c:
            r0 = move-exception
            r27 = r4
        L_0x030f:
            r10 = r25
            goto L_0x03f7
        L_0x0313:
            boolean r0 = r9.isAbsolute()     // Catch:{ IOException | RuntimeException -> 0x03f2, all -> 0x03ec }
            if (r0 == 0) goto L_0x03dc
            boolean r0 = r9.equals(r3)     // Catch:{ IOException | RuntimeException -> 0x03f2, all -> 0x03ec }
            if (r0 != 0) goto L_0x03dc
            org.bytedeco.javacpp.tools.Logger r0 = logger     // Catch:{ IOException | RuntimeException -> 0x03f2, all -> 0x03ec }
            boolean r0 = r0.isDebugEnabled()     // Catch:{ IOException | RuntimeException -> 0x03f2, all -> 0x03ec }
            if (r0 == 0) goto L_0x0345
            org.bytedeco.javacpp.tools.Logger r0 = logger     // Catch:{ IOException | RuntimeException -> 0x03f2, all -> 0x03ec }
            java.lang.StringBuilder r10 = new java.lang.StringBuilder     // Catch:{ IOException | RuntimeException -> 0x03f2, all -> 0x03ec }
            r10.<init>()     // Catch:{ IOException | RuntimeException -> 0x03f2, all -> 0x03ec }
            r27 = r4
            java.lang.String r4 = "Locking "
            r10.append(r4)     // Catch:{ IOException | RuntimeException -> 0x03d9, all -> 0x03d6 }
            r10.append(r6)     // Catch:{ IOException | RuntimeException -> 0x03d9, all -> 0x03d6 }
            java.lang.String r4 = " to create symbolic link"
            r10.append(r4)     // Catch:{ IOException | RuntimeException -> 0x03d9, all -> 0x03d6 }
            java.lang.String r4 = r10.toString()     // Catch:{ IOException | RuntimeException -> 0x03d9, all -> 0x03d6 }
            r0.debug(r4)     // Catch:{ IOException | RuntimeException -> 0x03d9, all -> 0x03d6 }
            goto L_0x0347
        L_0x0345:
            r27 = r4
        L_0x0347:
            java.io.FileOutputStream r0 = new java.io.FileOutputStream     // Catch:{ IOException | RuntimeException -> 0x03d9, all -> 0x03d6 }
            r0.<init>(r8)     // Catch:{ IOException | RuntimeException -> 0x03d9, all -> 0x03d6 }
            java.nio.channels.FileChannel r0 = r0.getChannel()     // Catch:{ IOException | RuntimeException -> 0x03d9, all -> 0x03d6 }
            r10 = r0
            java.nio.channels.FileLock r0 = r10.lock()     // Catch:{ IOException | RuntimeException -> 0x03d2, all -> 0x03ce }
            r14 = r0
            boolean r0 = r5.exists()     // Catch:{ IOException | RuntimeException -> 0x03d2, all -> 0x03ce }
            if (r0 == 0) goto L_0x0374
            boolean r0 = java.nio.file.Files.isSymbolicLink(r3)     // Catch:{ IOException | RuntimeException -> 0x0371 }
            if (r0 == 0) goto L_0x0374
            java.nio.file.Path r0 = java.nio.file.Files.readSymbolicLink(r3)     // Catch:{ IOException | RuntimeException -> 0x0371 }
            boolean r0 = r0.equals(r9)     // Catch:{ IOException | RuntimeException -> 0x0371 }
            if (r0 != 0) goto L_0x036d
            goto L_0x0374
        L_0x036d:
            r28 = r10
            goto L_0x03e0
        L_0x0371:
            r0 = move-exception
            goto L_0x03f7
        L_0x0374:
            boolean r0 = r9.isAbsolute()     // Catch:{ IOException | RuntimeException -> 0x03d2, all -> 0x03ce }
            if (r0 == 0) goto L_0x03cb
            boolean r0 = r9.equals(r3)     // Catch:{ IOException | RuntimeException -> 0x03d2, all -> 0x03ce }
            if (r0 != 0) goto L_0x03cb
            org.bytedeco.javacpp.tools.Logger r0 = logger     // Catch:{ IOException | RuntimeException -> 0x03d2, all -> 0x03ce }
            boolean r0 = r0.isDebugEnabled()     // Catch:{ IOException | RuntimeException -> 0x03d2, all -> 0x03ce }
            if (r0 == 0) goto L_0x03a9
            org.bytedeco.javacpp.tools.Logger r0 = logger     // Catch:{ IOException | RuntimeException -> 0x03d2, all -> 0x03ce }
            java.lang.StringBuilder r4 = new java.lang.StringBuilder     // Catch:{ IOException | RuntimeException -> 0x03d2, all -> 0x03ce }
            r4.<init>()     // Catch:{ IOException | RuntimeException -> 0x03d2, all -> 0x03ce }
            r28 = r10
            java.lang.String r10 = "Creating symbolic link "
            r4.append(r10)     // Catch:{ IOException | RuntimeException -> 0x03bc, all -> 0x03b9 }
            r4.append(r3)     // Catch:{ IOException | RuntimeException -> 0x03bc, all -> 0x03b9 }
            java.lang.String r10 = " to "
            r4.append(r10)     // Catch:{ IOException | RuntimeException -> 0x03bc, all -> 0x03b9 }
            r4.append(r9)     // Catch:{ IOException | RuntimeException -> 0x03bc, all -> 0x03b9 }
            java.lang.String r4 = r4.toString()     // Catch:{ IOException | RuntimeException -> 0x03bc, all -> 0x03b9 }
            r0.debug(r4)     // Catch:{ IOException | RuntimeException -> 0x03bc, all -> 0x03b9 }
            goto L_0x03ab
        L_0x03a9:
            r28 = r10
        L_0x03ab:
            java.io.File r0 = r5.getParentFile()     // Catch:{ FileAlreadyExistsException -> 0x03c0, IOException | RuntimeException -> 0x03bc }
            r0.mkdirs()     // Catch:{ FileAlreadyExistsException -> 0x03c0, IOException | RuntimeException -> 0x03bc }
            r4 = 0
            java.nio.file.attribute.FileAttribute[] r0 = new java.nio.file.attribute.FileAttribute[r4]     // Catch:{ FileAlreadyExistsException -> 0x03c0, IOException | RuntimeException -> 0x03bc }
            java.nio.file.Files.createSymbolicLink(r3, r9, r0)     // Catch:{ FileAlreadyExistsException -> 0x03c0, IOException | RuntimeException -> 0x03bc }
            goto L_0x03e0
        L_0x03b9:
            r0 = move-exception
            goto L_0x0433
        L_0x03bc:
            r0 = move-exception
            r10 = r28
            goto L_0x03f7
        L_0x03c0:
            r0 = move-exception
            r5.delete()     // Catch:{ IOException | RuntimeException -> 0x03bc, all -> 0x03b9 }
            r4 = 0
            java.nio.file.attribute.FileAttribute[] r4 = new java.nio.file.attribute.FileAttribute[r4]     // Catch:{ IOException | RuntimeException -> 0x03bc, all -> 0x03b9 }
            java.nio.file.Files.createSymbolicLink(r3, r9, r4)     // Catch:{ IOException | RuntimeException -> 0x03bc, all -> 0x03b9 }
            goto L_0x03e0
        L_0x03cb:
            r28 = r10
            goto L_0x03e0
        L_0x03ce:
            r0 = move-exception
            r28 = r10
            goto L_0x0433
        L_0x03d2:
            r0 = move-exception
            r28 = r10
            goto L_0x03f7
        L_0x03d6:
            r0 = move-exception
            goto L_0x0308
        L_0x03d9:
            r0 = move-exception
            goto L_0x030f
        L_0x03dc:
            r27 = r4
        L_0x03de:
            r28 = r25
        L_0x03e0:
            if (r14 == 0) goto L_0x03e5
            r14.release()     // Catch:{ all -> 0x0439 }
        L_0x03e5:
            if (r28 == 0) goto L_0x03ea
            r28.close()     // Catch:{ all -> 0x0439 }
        L_0x03ea:
            monitor-exit(r2)     // Catch:{ all -> 0x0439 }
            return r5
        L_0x03ec:
            r0 = move-exception
            r27 = r4
            r28 = r25
            goto L_0x0433
        L_0x03f2:
            r0 = move-exception
            r27 = r4
            r10 = r25
        L_0x03f7:
            org.bytedeco.javacpp.tools.Logger r3 = logger     // Catch:{ all -> 0x0430 }
            boolean r3 = r3.isDebugEnabled()     // Catch:{ all -> 0x0430 }
            if (r3 == 0) goto L_0x041d
            org.bytedeco.javacpp.tools.Logger r3 = logger     // Catch:{ all -> 0x0430 }
            java.lang.StringBuilder r4 = new java.lang.StringBuilder     // Catch:{ all -> 0x0430 }
            r4.<init>()     // Catch:{ all -> 0x0430 }
            java.lang.String r9 = "Could not create symbolic link "
            r4.append(r9)     // Catch:{ all -> 0x0430 }
            r4.append(r5)     // Catch:{ all -> 0x0430 }
            java.lang.String r9 = ": "
            r4.append(r9)     // Catch:{ all -> 0x0430 }
            r4.append(r0)     // Catch:{ all -> 0x0430 }
            java.lang.String r4 = r4.toString()     // Catch:{ all -> 0x0430 }
            r3.debug(r4)     // Catch:{ all -> 0x0430 }
        L_0x041d:
            if (r14 == 0) goto L_0x0427
            r14.release()     // Catch:{ all -> 0x0423 }
            goto L_0x0427
        L_0x0423:
            r0 = move-exception
            r28 = r10
            goto L_0x0441
        L_0x0427:
            if (r10 == 0) goto L_0x042c
            r10.close()     // Catch:{ all -> 0x0423 }
        L_0x042c:
            monitor-exit(r2)     // Catch:{ all -> 0x0423 }
            r25 = r10
            goto L_0x0445
        L_0x0430:
            r0 = move-exception
            r28 = r10
        L_0x0433:
            if (r14 == 0) goto L_0x043b
            r14.release()     // Catch:{ all -> 0x0439 }
            goto L_0x043b
        L_0x0439:
            r0 = move-exception
            goto L_0x0441
        L_0x043b:
            if (r28 == 0) goto L_0x0440
            r28.close()     // Catch:{ all -> 0x0439 }
        L_0x0440:
            throw r0     // Catch:{ all -> 0x0439 }
        L_0x0441:
            monitor-exit(r2)     // Catch:{ all -> 0x0439 }
            throw r0
        L_0x0443:
            r27 = r4
        L_0x0445:
            boolean r0 = r5.exists()
            if (r0 == 0) goto L_0x0469
            long r2 = r5.length()
            int r0 = (r2 > r15 ? 1 : (r2 == r15 ? 0 : -1))
            if (r0 != 0) goto L_0x0469
            long r2 = r5.lastModified()
            int r0 = (r2 > r12 ? 1 : (r2 == r12 ? 0 : -1))
            if (r0 != 0) goto L_0x0469
            java.io.File r0 = r5.getCanonicalFile()
            java.io.File r0 = r0.getParentFile()
            boolean r0 = r7.equals(r0)
            if (r0 != 0) goto L_0x04f8
        L_0x0469:
            java.lang.Runtime r2 = java.lang.Runtime.getRuntime()
            monitor-enter(r2)
            org.bytedeco.javacpp.tools.Logger r0 = logger     // Catch:{ all -> 0x04f9 }
            boolean r0 = r0.isDebugEnabled()     // Catch:{ all -> 0x04f9 }
            if (r0 == 0) goto L_0x0491
            org.bytedeco.javacpp.tools.Logger r0 = logger     // Catch:{ all -> 0x04f9 }
            java.lang.StringBuilder r3 = new java.lang.StringBuilder     // Catch:{ all -> 0x04f9 }
            r3.<init>()     // Catch:{ all -> 0x04f9 }
            java.lang.String r4 = "Locking "
            r3.append(r4)     // Catch:{ all -> 0x04f9 }
            r3.append(r6)     // Catch:{ all -> 0x04f9 }
            java.lang.String r4 = " before extracting"
            r3.append(r4)     // Catch:{ all -> 0x04f9 }
            java.lang.String r3 = r3.toString()     // Catch:{ all -> 0x04f9 }
            r0.debug(r3)     // Catch:{ all -> 0x04f9 }
        L_0x0491:
            java.io.FileOutputStream r0 = new java.io.FileOutputStream     // Catch:{ all -> 0x04f9 }
            r0.<init>(r8)     // Catch:{ all -> 0x04f9 }
            java.nio.channels.FileChannel r0 = r0.getChannel()     // Catch:{ all -> 0x04f9 }
            r25 = r0
            java.nio.channels.FileLock r0 = r25.lock()     // Catch:{ all -> 0x04f9 }
            r14 = r0
            boolean r0 = r5.exists()     // Catch:{ all -> 0x04f9 }
            if (r0 == 0) goto L_0x04c5
            long r3 = r5.length()     // Catch:{ all -> 0x04f9 }
            int r0 = (r3 > r15 ? 1 : (r3 == r15 ? 0 : -1))
            if (r0 != 0) goto L_0x04c5
            long r3 = r5.lastModified()     // Catch:{ all -> 0x04f9 }
            int r0 = (r3 > r12 ? 1 : (r3 == r12 ? 0 : -1))
            if (r0 != 0) goto L_0x04c5
            java.io.File r0 = r5.getCanonicalFile()     // Catch:{ all -> 0x04f9 }
            java.io.File r0 = r0.getParentFile()     // Catch:{ all -> 0x04f9 }
            boolean r0 = r7.equals(r0)     // Catch:{ all -> 0x04f9 }
            if (r0 != 0) goto L_0x04ed
        L_0x04c5:
            org.bytedeco.javacpp.tools.Logger r0 = logger     // Catch:{ all -> 0x04f9 }
            boolean r0 = r0.isDebugEnabled()     // Catch:{ all -> 0x04f9 }
            if (r0 == 0) goto L_0x04e3
            org.bytedeco.javacpp.tools.Logger r0 = logger     // Catch:{ all -> 0x04f9 }
            java.lang.StringBuilder r3 = new java.lang.StringBuilder     // Catch:{ all -> 0x04f9 }
            r3.<init>()     // Catch:{ all -> 0x04f9 }
            java.lang.String r4 = "Extracting "
            r3.append(r4)     // Catch:{ all -> 0x04f9 }
            r3.append(r1)     // Catch:{ all -> 0x04f9 }
            java.lang.String r3 = r3.toString()     // Catch:{ all -> 0x04f9 }
            r0.debug(r3)     // Catch:{ all -> 0x04f9 }
        L_0x04e3:
            r5.delete()     // Catch:{ all -> 0x04f9 }
            r3 = 0
            extractResource((java.net.URL) r1, (java.io.File) r5, (java.lang.String) r3, (java.lang.String) r3)     // Catch:{ all -> 0x04f9 }
            r5.setLastModified(r12)     // Catch:{ all -> 0x04f9 }
        L_0x04ed:
            if (r14 == 0) goto L_0x04f2
            r14.release()     // Catch:{ all -> 0x0500 }
        L_0x04f2:
            if (r25 == 0) goto L_0x04f7
            r25.close()     // Catch:{ all -> 0x0500 }
        L_0x04f7:
            monitor-exit(r2)     // Catch:{ all -> 0x0500 }
        L_0x04f8:
            return r5
        L_0x04f9:
            r0 = move-exception
            if (r14 == 0) goto L_0x0502
            r14.release()     // Catch:{ all -> 0x0500 }
            goto L_0x0502
        L_0x0500:
            r0 = move-exception
            goto L_0x0508
        L_0x0502:
            if (r25 == 0) goto L_0x0507
            r25.close()     // Catch:{ all -> 0x0500 }
        L_0x0507:
            throw r0     // Catch:{ all -> 0x0500 }
        L_0x0508:
            monitor-exit(r2)     // Catch:{ all -> 0x0500 }
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.Loader.cacheResource(java.net.URL, java.lang.String):java.io.File");
    }

    public static File extractResource(String name, File directory, String prefix, String suffix) throws IOException {
        return extractResource(getCallerClass(2), name, directory, prefix, suffix);
    }

    public static File extractResource(Class cls, String name, File directory, String prefix, String suffix) throws IOException {
        return extractResource(findResource(cls, name), directory, prefix, suffix);
    }

    public static File[] extractResources(String name, File directory, String prefix, String suffix) throws IOException {
        return extractResources(getCallerClass(2), name, directory, prefix, suffix);
    }

    public static File[] extractResources(Class cls, String name, File directory, String prefix, String suffix) throws IOException {
        URL[] urls = findResources(cls, name);
        File[] files = new File[urls.length];
        for (int i = 0; i < urls.length; i++) {
            files[i] = extractResource(urls[i], directory, prefix, suffix);
        }
        return files;
    }

    public static File extractResource(URL resourceURL, File directoryOrFile, String prefix, String suffix) throws IOException {
        File file;
        File directory;
        File file2;
        Enumeration<JarEntry> entries;
        File directoryOrFile2 = directoryOrFile;
        String str = prefix;
        String str2 = suffix;
        URLConnection urlConnection = resourceURL != null ? resourceURL.openConnection() : null;
        if (urlConnection instanceof JarURLConnection) {
            JarFile jarFile = ((JarURLConnection) urlConnection).getJarFile();
            JarEntry jarEntry = ((JarURLConnection) urlConnection).getJarEntry();
            String name = jarFile.getName();
            String jarEntryName = jarEntry.getName();
            if (!jarEntryName.endsWith(CookieSpec.PATH_DELIM)) {
                jarEntryName = jarEntryName + CookieSpec.PATH_DELIM;
            }
            if (jarEntry.isDirectory() || jarFile.getJarEntry(jarEntryName) != null) {
                Enumeration<JarEntry> entries2 = jarFile.entries();
                while (entries2.hasMoreElements()) {
                    JarEntry entry = entries2.nextElement();
                    String entryName = entry.getName();
                    if (entryName.startsWith(jarEntryName)) {
                        File file3 = new File(directoryOrFile2, entryName.substring(jarEntryName.length()));
                        if (entry.isDirectory()) {
                            file3.mkdirs();
                            entries = entries2;
                        } else {
                            String s = resourceURL.toString();
                            StringBuilder sb = new StringBuilder();
                            entries = entries2;
                            sb.append(s.substring(0, s.indexOf("!/") + 2));
                            sb.append(entryName);
                            file3 = extractResource(new URL(sb.toString()), file3, str, str2);
                        }
                        file3.setLastModified(entry.getTime());
                    } else {
                        entries = entries2;
                    }
                    entries2 = entries;
                }
                return directoryOrFile2;
            }
        }
        InputStream is = urlConnection != null ? urlConnection.getInputStream() : null;
        OutputStream os = null;
        if (is == null) {
            return null;
        }
        File file4 = null;
        if (str == null && str2 == null) {
            if (directoryOrFile2 == null) {
                try {
                    directoryOrFile2 = new File(System.getProperty("java.io.tmpdir"));
                } catch (IOException e) {
                    if (file4 != null && 0 == 0) {
                        file4.delete();
                    }
                    throw e;
                } catch (Throwable th) {
                    is.close();
                    if (os != null) {
                        os.close();
                    }
                    throw th;
                }
            }
            if (directoryOrFile2.isDirectory()) {
                directory = directoryOrFile2;
                try {
                    file2 = new File(directoryOrFile2, new File(new URI(resourceURL.toString().split("#")[0])).getName());
                } catch (IllegalArgumentException | URISyntaxException e2) {
                    file2 = new File(directoryOrFile2, new File(resourceURL.getPath()).getName());
                }
            } else {
                directory = directoryOrFile2.getParentFile();
                file2 = directoryOrFile2;
            }
            file = file2;
            if (directory != null) {
                directory.mkdirs();
            }
            boolean fileExisted = file.exists();
        } else {
            file = File.createTempFile(str, str2, directoryOrFile2);
        }
        file.delete();
        OutputStream os2 = new FileOutputStream(file);
        byte[] buffer = new byte[65536];
        while (true) {
            int read = is.read(buffer);
            int length = read;
            if (read != -1) {
                os2.write(buffer, 0, length);
            } else {
                is.close();
                os2.close();
                return file;
            }
        }
    }

    public static URL findResource(Class cls, String name) throws IOException {
        URL[] url = findResources(cls, name, 1);
        if (url.length > 0) {
            return url[0];
        }
        return null;
    }

    public static URL[] findResources(Class cls, String name) throws IOException {
        return findResources(cls, name, -1);
    }

    public static URL[] findResources(Class cls, String name, int maxLength) throws IOException {
        String path;
        if (maxLength == 0) {
            return new URL[0];
        }
        while (name.contains("//")) {
            name = name.replace("//", CookieSpec.PATH_DELIM);
        }
        URL url = cls.getResource(name);
        if (url == null || maxLength != 1) {
            String path2 = "";
            if (!name.startsWith(CookieSpec.PATH_DELIM)) {
                String s = cls.getName().replace('.', IOUtils.DIR_SEPARATOR_UNIX);
                int n = s.lastIndexOf(47);
                if (n >= 0) {
                    path2 = s.substring(0, n + 1);
                }
            } else {
                name = name.substring(1);
            }
            ClassLoader classLoader = cls.getClassLoader();
            if (classLoader == null) {
                classLoader = ClassLoader.getSystemClassLoader();
            }
            Enumeration<URL> urls = classLoader.getResources(path + name);
            ArrayList<URL> array = new ArrayList<>();
            if (url != null) {
                array.add(url);
            }
            while (url == null && !urls.hasMoreElements() && path.length() > 0) {
                int n2 = path.lastIndexOf(47, path.length() - 2);
                if (n2 >= 0) {
                    path = path.substring(0, n2 + 1);
                } else {
                    path = "";
                }
                urls = classLoader.getResources(path + name);
            }
            while (urls.hasMoreElements() && (maxLength < 0 || array.size() < maxLength)) {
                URL url2 = urls.nextElement();
                if (!array.contains(url2)) {
                    array.add(url2);
                }
            }
            return (URL[]) array.toArray(new URL[array.size()]);
        }
        return new URL[]{url};
    }

    /* JADX WARNING: Code restructure failed: missing block: B:13:0x006c, code lost:
        cacheDir = r4;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public static java.io.File getCacheDir() throws java.io.IOException {
        /*
            java.io.File r0 = cacheDir
            if (r0 != 0) goto L_0x0073
            r0 = 3
            java.lang.String[] r0 = new java.lang.String[r0]
            java.lang.String r1 = "org.bytedeco.javacpp.cachedir"
            java.lang.String r1 = java.lang.System.getProperty(r1)
            r2 = 0
            r0[r2] = r1
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r3 = "user.home"
            java.lang.String r3 = java.lang.System.getProperty(r3)
            r1.append(r3)
            java.lang.String r3 = "/.javacpp/cache/"
            r1.append(r3)
            java.lang.String r1 = r1.toString()
            r3 = 1
            r0[r3] = r1
            r1 = 2
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = "java.io.tmpdir"
            java.lang.String r4 = java.lang.System.getProperty(r4)
            r3.append(r4)
            java.lang.String r4 = "/.javacpp-"
            r3.append(r4)
            java.lang.String r4 = "user.name"
            java.lang.String r4 = java.lang.System.getProperty(r4)
            r3.append(r4)
            java.lang.String r4 = "/cache/"
            r3.append(r4)
            java.lang.String r3 = r3.toString()
            r0[r1] = r3
            int r1 = r0.length
        L_0x0053:
            if (r2 >= r1) goto L_0x0073
            r3 = r0[r2]
            if (r3 == 0) goto L_0x0070
            java.io.File r4 = new java.io.File     // Catch:{ SecurityException -> 0x006f }
            r4.<init>(r3)     // Catch:{ SecurityException -> 0x006f }
            boolean r5 = r4.exists()     // Catch:{ SecurityException -> 0x006f }
            if (r5 != 0) goto L_0x006c
            boolean r5 = r4.mkdirs()     // Catch:{ SecurityException -> 0x006f }
            if (r5 == 0) goto L_0x006b
            goto L_0x006c
        L_0x006b:
            goto L_0x0070
        L_0x006c:
            cacheDir = r4     // Catch:{ SecurityException -> 0x006f }
            goto L_0x0073
        L_0x006f:
            r4 = move-exception
        L_0x0070:
            int r2 = r2 + 1
            goto L_0x0053
        L_0x0073:
            java.io.File r0 = cacheDir
            if (r0 == 0) goto L_0x007a
            java.io.File r0 = cacheDir
            return r0
        L_0x007a:
            java.io.IOException r0 = new java.io.IOException
            java.lang.String r1 = "Could not create the cache: Set the \"org.bytedeco.javacpp.cachedir\" system property."
            r0.<init>(r1)
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.Loader.getCacheDir():java.io.File");
    }

    public static File getTempDir() {
        if (tempDir == null) {
            File tmpdir = new File(System.getProperty("java.io.tmpdir"));
            int i = 0;
            while (true) {
                if (i >= 1000) {
                    break;
                }
                File f = new File(tmpdir, "javacpp" + System.nanoTime());
                if (f.mkdir()) {
                    tempDir = f;
                    tempDir.deleteOnExit();
                    break;
                }
                i++;
            }
        }
        return tempDir;
    }

    public static synchronized Map<String, String> getLoadedLibraries() {
        HashMap hashMap;
        synchronized (Loader.class) {
            hashMap = new HashMap(loadedLibraries);
        }
        return hashMap;
    }

    public static boolean isLoadLibraries() {
        String s = System.getProperty("org.bytedeco.javacpp.loadlibraries", "true").toLowerCase();
        return s.equals("true") || s.equals("t") || s.equals("");
    }

    public static boolean checkPlatform(Class<?> cls, Properties properties) {
        Class<? super Object> cls2;
        Class<?> enclosingClass = getEnclosingClass(cls);
        Class<? super Object> cls3 = cls;
        while (!cls3.isAnnotationPresent(org.bytedeco.javacpp.annotation.Properties.class) && !cls3.isAnnotationPresent(Platform.class) && cls3.getSuperclass() != null) {
            if (enclosingClass == null || cls3.getSuperclass() != Object.class) {
                cls2 = cls3.getSuperclass();
            } else {
                cls2 = enclosingClass;
                enclosingClass = null;
            }
            cls3 = cls2;
        }
        org.bytedeco.javacpp.annotation.Properties classProperties = (org.bytedeco.javacpp.annotation.Properties) cls3.getAnnotation(org.bytedeco.javacpp.annotation.Properties.class);
        if (classProperties != null) {
            Class[] classes = classProperties.inherit();
            String[] defaultNames = classProperties.names();
            Deque<Class> queue = new ArrayDeque<>(Arrays.asList(classes));
            while (queue.size() > 0 && (defaultNames == null || defaultNames.length == 0)) {
                org.bytedeco.javacpp.annotation.Properties p = (org.bytedeco.javacpp.annotation.Properties) queue.removeFirst().getAnnotation(org.bytedeco.javacpp.annotation.Properties.class);
                if (p != null) {
                    defaultNames = p.names();
                    queue.addAll(Arrays.asList(p.inherit()));
                }
            }
            Platform[] platforms = classProperties.value();
            if (platforms != null && platforms.length > 0) {
                for (Platform p2 : platforms) {
                    if (checkPlatform(p2, properties, defaultNames)) {
                        return true;
                    }
                }
            } else if (classes != null && classes.length > 0) {
                for (Class c : classes) {
                    if (checkPlatform(c, properties)) {
                        return true;
                    }
                }
            }
        } else if (checkPlatform((Platform) cls3.getAnnotation(Platform.class), properties, new String[0])) {
            return true;
        }
        return false;
    }

    public static boolean checkPlatform(Platform platform, Properties properties, String... defaultNames) {
        boolean match = true;
        if (platform == null) {
            return true;
        }
        if (defaultNames == null) {
            defaultNames = new String[0];
        }
        String platform2 = properties.getProperty("platform");
        String platformExtension = properties.getProperty("platform.extension");
        String[][] names = new String[2][];
        names[0] = platform.value().length > 0 ? platform.value() : defaultNames;
        names[1] = platform.not();
        boolean[] matches = {false, false};
        for (int i = 0; i < names.length; i++) {
            String[] strArr = names[i];
            int length = strArr.length;
            int i2 = 0;
            while (true) {
                if (i2 >= length) {
                    break;
                } else if (platform2.startsWith(strArr[i2])) {
                    matches[i] = true;
                    break;
                } else {
                    i2++;
                }
            }
        }
        if ((names[0].length != 0 && !matches[0]) || (names[1].length != 0 && matches[1])) {
            return false;
        }
        if (platform.extension().length != 0 && (!isLoadLibraries() || platformExtension != null)) {
            match = false;
        }
        for (String s : platform.extension()) {
            if (platformExtension != null && platformExtension.length() > 0 && platformExtension.endsWith(s)) {
                return true;
            }
        }
        return match;
    }

    public static String load() {
        return load(getCallerClass(2), loadProperties(), pathsFirst);
    }

    public static String load(boolean pathsFirst2) {
        return load(getCallerClass(2), loadProperties(), pathsFirst2);
    }

    public static String load(Class cls) {
        return load(cls, loadProperties(), pathsFirst);
    }

    public static String load(Class cls, Properties properties, boolean pathsFirst2) {
        if (!isLoadLibraries() || cls == null) {
            return null;
        }
        if (checkPlatform(cls, properties)) {
            Class cls2 = getEnclosingClass(cls);
            ClassProperties p = loadProperties(cls2, properties, true);
            List<String> targets = p.get("global");
            if (targets.isEmpty()) {
                if (p.getInheritedClasses() != null) {
                    for (Class c : p.getInheritedClasses()) {
                        targets.add(c.getName());
                    }
                }
                targets.add(cls2.getName());
            }
            for (String s : targets) {
                try {
                    if (logger.isDebugEnabled()) {
                        logger.debug("Loading class " + s);
                    }
                    Class.forName(s, true, cls2.getClassLoader());
                } catch (ClassNotFoundException ex) {
                    if (logger.isDebugEnabled()) {
                        logger.debug("Failed to load class " + s + ": " + ex);
                    }
                    Error e = new NoClassDefFoundError(ex.toString());
                    e.initCause(ex);
                    throw e;
                }
            }
            String cacheDir2 = null;
            try {
                cacheDir2 = getCacheDir().getCanonicalPath();
            } catch (IOException e2) {
            }
            List<String> preloads = new ArrayList<>();
            List<String> preloaded = new ArrayList<>();
            preloads.addAll(p.get("platform.preload"));
            preloads.addAll(p.get("platform.link"));
            UnsatisfiedLinkError preloadError = null;
            for (String preload : preloads) {
                try {
                    String filename = loadLibrary(findLibrary(cls2, p, preload, pathsFirst2), preload, new String[0]);
                    if (filename != null) {
                        preloaded.add(filename);
                    }
                    if (!(cacheDir2 == null || filename == null || !filename.startsWith(cacheDir2))) {
                        createLibraryLink(filename, p, preload, new String[0]);
                    }
                } catch (UnsatisfiedLinkError e3) {
                    preloadError = e3;
                }
            }
            int librarySuffix = -1;
            while (true) {
                try {
                    String library = p.getProperty("platform.library");
                    if (librarySuffix >= 0) {
                        library = library + "#" + library + librarySuffix;
                    }
                    String filename2 = loadLibrary(findLibrary(cls2, p, library, pathsFirst2), library, (String[]) preloaded.toArray(new String[preloaded.size()]));
                    if (!(cacheDir2 == null || filename2 == null || !filename2.startsWith(cacheDir2))) {
                        createLibraryLink(filename2, p, library, new String[0]);
                    }
                    return filename2;
                } catch (UnsatisfiedLinkError e4) {
                    Throwable t = e4;
                    while (t != null) {
                        if (!(t instanceof UnsatisfiedLinkError) || !t.getMessage().contains("already loaded in another classloader")) {
                            t = t.getCause() != t ? t.getCause() : null;
                        } else {
                            librarySuffix++;
                        }
                    }
                    if (preloadError != null && e4.getCause() == null) {
                        e4.initCause(preloadError);
                    }
                    throw e4;
                }
            }
        } else {
            throw new UnsatisfiedLinkError("Platform \"" + properties.getProperty("platform") + "\" not supported by " + cls);
        }
    }

    public static URL[] findLibrary(Class cls, ClassProperties properties, String libnameversion) {
        return findLibrary(cls, properties, libnameversion, pathsFirst);
    }

    /*  JADX ERROR: JadxRuntimeException in pass: IfRegionVisitor
        jadx.core.utils.exceptions.JadxRuntimeException: Don't wrap MOVE or CONST insns: 0x0261: MOVE  (r2v10 'i' int) = (r28v0 'i' int)
        	at jadx.core.dex.instructions.args.InsnArg.wrapArg(InsnArg.java:164)
        	at jadx.core.dex.visitors.shrink.CodeShrinkVisitor.assignInline(CodeShrinkVisitor.java:133)
        	at jadx.core.dex.visitors.shrink.CodeShrinkVisitor.checkInline(CodeShrinkVisitor.java:118)
        	at jadx.core.dex.visitors.shrink.CodeShrinkVisitor.shrinkBlock(CodeShrinkVisitor.java:65)
        	at jadx.core.dex.visitors.shrink.CodeShrinkVisitor.shrinkMethod(CodeShrinkVisitor.java:43)
        	at jadx.core.dex.visitors.regions.TernaryMod.makeTernaryInsn(TernaryMod.java:122)
        	at jadx.core.dex.visitors.regions.TernaryMod.visitRegion(TernaryMod.java:34)
        	at jadx.core.dex.visitors.regions.DepthRegionTraversal.traverseIterativeStepInternal(DepthRegionTraversal.java:73)
        	at jadx.core.dex.visitors.regions.DepthRegionTraversal.traverseIterativeStepInternal(DepthRegionTraversal.java:78)
        	at jadx.core.dex.visitors.regions.DepthRegionTraversal.traverseIterativeStepInternal(DepthRegionTraversal.java:78)
        	at jadx.core.dex.visitors.regions.DepthRegionTraversal.traverseIterativeStepInternal(DepthRegionTraversal.java:78)
        	at jadx.core.dex.visitors.regions.DepthRegionTraversal.traverseIterative(DepthRegionTraversal.java:27)
        	at jadx.core.dex.visitors.regions.IfRegionVisitor.visit(IfRegionVisitor.java:31)
        */
    /* JADX WARNING: Removed duplicated region for block: B:122:0x033f A[SYNTHETIC] */
    /* JADX WARNING: Removed duplicated region for block: B:53:0x02bf  */
    /* JADX WARNING: Removed duplicated region for block: B:55:0x02c8  */
    /* JADX WARNING: Removed duplicated region for block: B:58:0x02e8  */
    /* JADX WARNING: Removed duplicated region for block: B:59:0x02eb  */
    /* JADX WARNING: Removed duplicated region for block: B:67:0x0313 A[Catch:{ IOException -> 0x0350 }] */
    public static java.net.URL[] findLibrary(java.lang.Class r44, org.bytedeco.javacpp.ClassProperties r45, java.lang.String r46, boolean r47) {
        /*
            r1 = r44
            r2 = r45
            java.lang.String r0 = r46.trim()
            java.lang.String r3 = "#"
            boolean r0 = r0.endsWith(r3)
            r3 = 0
            if (r0 == 0) goto L_0x0014
            java.net.URL[] r0 = new java.net.URL[r3]
            return r0
        L_0x0014:
            java.lang.String r0 = "#"
            r4 = r46
            java.lang.String[] r5 = r4.split(r0)
            int r0 = r5.length
            r6 = 1
            if (r0 <= r6) goto L_0x0022
            r0 = 1
            goto L_0x0023
        L_0x0022:
            r0 = 0
        L_0x0023:
            r7 = r0
            r0 = r5[r3]
            java.lang.String r8 = "@"
            java.lang.String[] r8 = r0.split(r8)
            if (r7 == 0) goto L_0x0031
            r0 = r5[r6]
            goto L_0x0033
        L_0x0031:
            r0 = r5[r3]
        L_0x0033:
            java.lang.String r9 = "@"
            java.lang.String[] r9 = r0.split(r9)
            r10 = r8[r3]
            r11 = r9[r3]
            int r0 = r8.length
            if (r0 <= r6) goto L_0x0045
            int r0 = r8.length
            int r0 = r0 - r6
            r0 = r8[r0]
            goto L_0x0047
        L_0x0045:
            java.lang.String r0 = ""
        L_0x0047:
            r12 = r0
            int r0 = r9.length
            if (r0 <= r6) goto L_0x0050
            int r0 = r9.length
            int r0 = r0 - r6
            r0 = r9[r0]
            goto L_0x0052
        L_0x0050:
            java.lang.String r0 = ""
        L_0x0052:
            r13 = r0
            java.lang.String r0 = "platform"
            java.lang.String r14 = r2.getProperty(r0)
            java.lang.String r0 = "platform.extension"
            java.util.List r0 = r2.get(r0)
            java.lang.String[] r6 = new java.lang.String[r3]
            java.lang.Object[] r0 = r0.toArray(r6)
            r6 = r0
            java.lang.String[] r6 = (java.lang.String[]) r6
            java.lang.String r0 = "platform.library.prefix"
            java.lang.String r3 = ""
            java.lang.String r3 = r2.getProperty(r0, r3)
            java.lang.String r0 = "platform.library.suffix"
            java.lang.String r4 = ""
            java.lang.String r4 = r2.getProperty(r0, r4)
            r0 = 3
            r17 = r5
            java.lang.String[] r5 = new java.lang.String[r0]
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r3)
            r0.append(r10)
            r0.append(r4)
            r0.append(r12)
            java.lang.String r0 = r0.toString()
            r16 = 0
            r5[r16] = r0
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r3)
            r0.append(r10)
            r0.append(r12)
            r0.append(r4)
            java.lang.String r0 = r0.toString()
            r15 = 1
            r5[r15] = r0
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r3)
            r0.append(r10)
            r0.append(r4)
            java.lang.String r0 = r0.toString()
            r19 = 2
            r5[r19] = r0
            r0 = r5
            r20 = r0
            r5 = 3
            java.lang.String[] r0 = new java.lang.String[r5]
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            r5.append(r3)
            r5.append(r11)
            r5.append(r4)
            r5.append(r13)
            java.lang.String r5 = r5.toString()
            r16 = 0
            r0[r16] = r5
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            r5.append(r3)
            r5.append(r11)
            r5.append(r13)
            r5.append(r4)
            java.lang.String r5 = r5.toString()
            r15 = 1
            r0[r15] = r5
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            r5.append(r3)
            r5.append(r11)
            r5.append(r4)
            java.lang.String r5 = r5.toString()
            r0[r19] = r5
            java.lang.String r5 = "platform.library.suffix"
            java.util.List r5 = r2.get(r5)
            r21 = r0
            r22 = r4
            r0 = 0
            java.lang.String[] r4 = new java.lang.String[r0]
            java.lang.Object[] r0 = r5.toArray(r4)
            r4 = r0
            java.lang.String[] r4 = (java.lang.String[]) r4
            int r0 = r4.length
            r5 = 1
            if (r0 <= r5) goto L_0x01f4
            int r0 = r4.length
            r5 = 3
            int r0 = r0 * 3
            java.lang.String[] r0 = new java.lang.String[r0]
            r23 = r8
            int r8 = r4.length
            int r8 = r8 * 3
            java.lang.String[] r5 = new java.lang.String[r8]
            r8 = 0
        L_0x0135:
            r24 = r9
            int r9 = r4.length
            if (r8 >= r9) goto L_0x01ec
            int r9 = r8 * 3
            r25 = r14
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            r14.append(r3)
            r14.append(r10)
            r26 = r6
            r6 = r4[r8]
            r14.append(r6)
            r14.append(r12)
            java.lang.String r6 = r14.toString()
            r0[r9] = r6
            int r6 = r8 * 3
            r9 = 1
            int r6 = r6 + r9
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            r9.append(r3)
            r9.append(r10)
            r9.append(r12)
            r14 = r4[r8]
            r9.append(r14)
            java.lang.String r9 = r9.toString()
            r0[r6] = r9
            int r6 = r8 * 3
            int r6 = r6 + 2
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            r9.append(r3)
            r9.append(r10)
            r14 = r4[r8]
            r9.append(r14)
            java.lang.String r9 = r9.toString()
            r0[r6] = r9
            int r6 = r8 * 3
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            r9.append(r3)
            r9.append(r11)
            r14 = r4[r8]
            r9.append(r14)
            r9.append(r13)
            java.lang.String r9 = r9.toString()
            r5[r6] = r9
            int r6 = r8 * 3
            r9 = 1
            int r6 = r6 + r9
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            r9.append(r3)
            r9.append(r11)
            r9.append(r13)
            r14 = r4[r8]
            r9.append(r14)
            java.lang.String r9 = r9.toString()
            r5[r6] = r9
            int r6 = r8 * 3
            int r6 = r6 + 2
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            r9.append(r3)
            r9.append(r11)
            r14 = r4[r8]
            r9.append(r14)
            java.lang.String r9 = r9.toString()
            r5[r6] = r9
            int r8 = r8 + 1
            r9 = r24
            r14 = r25
            r6 = r26
            goto L_0x0135
        L_0x01ec:
            r26 = r6
            r25 = r14
            r21 = r5
            r5 = r0
            goto L_0x01fe
        L_0x01f4:
            r26 = r6
            r23 = r8
            r24 = r9
            r25 = r14
            r5 = r20
        L_0x01fe:
            java.util.ArrayList r0 = new java.util.ArrayList
            r0.<init>()
            r6 = r0
            java.lang.String r0 = "platform.linkpath"
            java.util.List r0 = r2.get(r0)
            r6.addAll(r0)
            java.lang.String r0 = "platform.preloadpath"
            java.util.List r0 = r2.get(r0)
            r6.addAll(r0)
            java.lang.String r0 = "platform.preloadresource"
            java.util.List r0 = r2.get(r0)
            r8 = 0
            java.lang.String[] r9 = new java.lang.String[r8]
            java.lang.Object[] r0 = r0.toArray(r9)
            r9 = r0
            java.lang.String[] r9 = (java.lang.String[]) r9
            java.lang.String r0 = "java.library.path"
            java.lang.String r14 = ""
            java.lang.String r14 = java.lang.System.getProperty(r0, r14)
            int r0 = r14.length()
            if (r0 <= 0) goto L_0x024b
            if (r47 != 0) goto L_0x023e
            boolean r0 = isLoadLibraries()
            if (r0 == 0) goto L_0x023e
            if (r7 == 0) goto L_0x024b
        L_0x023e:
            java.lang.String r0 = java.io.File.pathSeparator
            java.lang.String[] r0 = r14.split(r0)
            java.util.List r0 = java.util.Arrays.asList(r0)
            r6.addAll(r0)
        L_0x024b:
            java.util.ArrayList r0 = new java.util.ArrayList
            int r8 = r5.length
            int r16 = r6.size()
            r15 = 1
            int r16 = r16 + 1
            int r8 = r8 * r16
            r0.<init>(r8)
            r8 = r0
            r0 = 0
        L_0x025c:
            r28 = r0
            if (r1 == 0) goto L_0x0387
            int r0 = r5.length
            r2 = r28
            if (r2 >= r0) goto L_0x0387
            r29 = r3
            r3 = r26
            int r0 = r3.length
            r15 = 1
            int r0 = r0 + r15
            java.lang.Object[] r0 = java.util.Arrays.copyOf(r3, r0)
            java.lang.String[] r0 = (java.lang.String[]) r0
            r30 = r3
            int r3 = r0.length
            r31 = r4
            r4 = 0
        L_0x0278:
            if (r4 >= r3) goto L_0x036f
            r16 = r0[r4]
            r32 = r0
            int r0 = r9.length
            int r0 = r0 + r15
            java.lang.Object[] r0 = java.util.Arrays.copyOf(r9, r0)
            java.lang.String[] r0 = (java.lang.String[]) r0
            r33 = r3
            int r3 = r0.length
            r34 = r9
            r9 = 0
        L_0x028c:
            if (r9 >= r3) goto L_0x035b
            r35 = r3
            r3 = r0[r9]
            if (r3 == 0) goto L_0x02b2
            r36 = r0
            java.lang.String r0 = "/"
            boolean r0 = r3.endsWith(r0)
            if (r0 != 0) goto L_0x02b4
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r3)
            r37 = r3
            java.lang.String r3 = "/"
            r0.append(r3)
            java.lang.String r3 = r0.toString()
            goto L_0x02b8
        L_0x02b2:
            r36 = r0
        L_0x02b4:
            r37 = r3
            r3 = r37
        L_0x02b8:
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            if (r3 != 0) goto L_0x02c8
            java.lang.String r18 = ""
            r38 = r10
            r39 = r11
        L_0x02c5:
            r10 = r18
            goto L_0x02de
        L_0x02c8:
            r38 = r10
            java.lang.StringBuilder r10 = new java.lang.StringBuilder
            r10.<init>()
            r39 = r11
            java.lang.String r11 = "/"
            r10.append(r11)
            r10.append(r3)
            java.lang.String r18 = r10.toString()
            goto L_0x02c5
        L_0x02de:
            r0.append(r10)
            r10 = r25
            r0.append(r10)
            if (r16 != 0) goto L_0x02eb
            java.lang.String r11 = ""
            goto L_0x02ed
        L_0x02eb:
            r11 = r16
        L_0x02ed:
            r0.append(r11)
            java.lang.String r11 = "/"
            r0.append(r11)
            java.lang.String r0 = r0.toString()
            r11 = r0
            java.lang.StringBuilder r0 = new java.lang.StringBuilder     // Catch:{ IOException -> 0x0352 }
            r0.<init>()     // Catch:{ IOException -> 0x0352 }
            r0.append(r11)     // Catch:{ IOException -> 0x0352 }
            r40 = r3
            r3 = r5[r2]     // Catch:{ IOException -> 0x0350 }
            r0.append(r3)     // Catch:{ IOException -> 0x0350 }
            java.lang.String r0 = r0.toString()     // Catch:{ IOException -> 0x0350 }
            java.net.URL r0 = findResource(r1, r0)     // Catch:{ IOException -> 0x0350 }
            if (r0 == 0) goto L_0x033f
            if (r7 == 0) goto L_0x0334
            java.net.URL r3 = new java.net.URL     // Catch:{ IOException -> 0x0350 }
            java.lang.StringBuilder r1 = new java.lang.StringBuilder     // Catch:{ IOException -> 0x0350 }
            r1.<init>()     // Catch:{ IOException -> 0x0350 }
            r1.append(r0)     // Catch:{ IOException -> 0x0350 }
            r41 = r0
            java.lang.String r0 = "#"
            r1.append(r0)     // Catch:{ IOException -> 0x0350 }
            r0 = r21[r2]     // Catch:{ IOException -> 0x0350 }
            r1.append(r0)     // Catch:{ IOException -> 0x0350 }
            java.lang.String r0 = r1.toString()     // Catch:{ IOException -> 0x0350 }
            r3.<init>(r0)     // Catch:{ IOException -> 0x0350 }
            r0 = r3
            goto L_0x0336
        L_0x0334:
            r41 = r0
        L_0x0336:
            boolean r1 = r8.contains(r0)     // Catch:{ IOException -> 0x0350 }
            if (r1 != 0) goto L_0x033f
            r8.add(r0)     // Catch:{ IOException -> 0x0350 }
        L_0x033f:
            int r9 = r9 + 1
            r25 = r10
            r3 = r35
            r0 = r36
            r10 = r38
            r11 = r39
            r1 = r44
            goto L_0x028c
        L_0x0350:
            r0 = move-exception
            goto L_0x0355
        L_0x0352:
            r0 = move-exception
            r40 = r3
        L_0x0355:
            java.lang.RuntimeException r1 = new java.lang.RuntimeException
            r1.<init>(r0)
            throw r1
        L_0x035b:
            r38 = r10
            r39 = r11
            r10 = r25
            int r4 = r4 + 1
            r0 = r32
            r3 = r33
            r9 = r34
            r10 = r38
            r1 = r44
            goto L_0x0278
        L_0x036f:
            r34 = r9
            r38 = r10
            r39 = r11
            r10 = r25
            int r0 = r2 + 1
            r3 = r29
            r26 = r30
            r4 = r31
            r10 = r38
            r1 = r44
            r2 = r45
            goto L_0x025c
        L_0x0387:
            r29 = r3
            r31 = r4
            r34 = r9
            r38 = r10
            r39 = r11
            r10 = r25
            r30 = r26
            if (r47 == 0) goto L_0x0399
            r3 = 0
            goto L_0x039d
        L_0x0399:
            int r3 = r8.size()
        L_0x039d:
            r0 = r3
            r27 = 0
        L_0x03a0:
            r1 = r27
            int r2 = r6.size()
            if (r2 <= 0) goto L_0x041b
            int r2 = r5.length
            if (r1 >= r2) goto L_0x041b
            java.util.Iterator r2 = r6.iterator()
            r3 = r0
        L_0x03b0:
            boolean r0 = r2.hasNext()
            if (r0 == 0) goto L_0x0417
            java.lang.Object r0 = r2.next()
            r4 = r0
            java.lang.String r4 = (java.lang.String) r4
            java.io.File r0 = new java.io.File
            r9 = r5[r1]
            r0.<init>(r4, r9)
            r9 = r0
            boolean r0 = r9.exists()
            if (r0 == 0) goto L_0x0412
            java.net.URI r0 = r9.toURI()     // Catch:{ IOException -> 0x040b }
            java.net.URL r0 = r0.toURL()     // Catch:{ IOException -> 0x040b }
            if (r7 == 0) goto L_0x03f6
            java.net.URL r11 = new java.net.URL     // Catch:{ IOException -> 0x040b }
            r42 = r2
            java.lang.StringBuilder r2 = new java.lang.StringBuilder     // Catch:{ IOException -> 0x040b }
            r2.<init>()     // Catch:{ IOException -> 0x040b }
            r2.append(r0)     // Catch:{ IOException -> 0x040b }
            r43 = r0
            java.lang.String r0 = "#"
            r2.append(r0)     // Catch:{ IOException -> 0x040b }
            r0 = r21[r1]     // Catch:{ IOException -> 0x040b }
            r2.append(r0)     // Catch:{ IOException -> 0x040b }
            java.lang.String r0 = r2.toString()     // Catch:{ IOException -> 0x040b }
            r11.<init>(r0)     // Catch:{ IOException -> 0x040b }
            r0 = r11
            goto L_0x03fa
        L_0x03f6:
            r43 = r0
            r42 = r2
        L_0x03fa:
            boolean r2 = r8.contains(r0)     // Catch:{ IOException -> 0x040b }
            if (r2 != 0) goto L_0x040a
            int r2 = r3 + 1
            r8.add(r3, r0)     // Catch:{ IOException -> 0x0407 }
            r3 = r2
            goto L_0x040a
        L_0x0407:
            r0 = move-exception
            r3 = r2
            goto L_0x040c
        L_0x040a:
            goto L_0x0414
        L_0x040b:
            r0 = move-exception
        L_0x040c:
            java.lang.RuntimeException r2 = new java.lang.RuntimeException
            r2.<init>(r0)
            throw r2
        L_0x0412:
            r42 = r2
        L_0x0414:
            r2 = r42
            goto L_0x03b0
        L_0x0417:
            int r27 = r1 + 1
            r0 = r3
            goto L_0x03a0
        L_0x041b:
            int r1 = r8.size()
            java.net.URL[] r1 = new java.net.URL[r1]
            java.lang.Object[] r1 = r8.toArray(r1)
            java.net.URL[] r1 = (java.net.URL[]) r1
            return r1
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.Loader.findLibrary(java.lang.Class, org.bytedeco.javacpp.ClassProperties, java.lang.String, boolean):java.net.URL[]");
    }

    /* JADX WARNING: Code restructure failed: missing block: B:130:0x0213, code lost:
        r0 = move-exception;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:131:0x0214, code lost:
        r3 = r0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:133:?, code lost:
        loadedLibraries.remove(r8);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:134:0x0220, code lost:
        if (logger.isDebugEnabled() != false) goto L_0x0222;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:135:0x0222, code lost:
        logger.debug("Failed to load " + r1 + ": " + r0);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:136:0x0240, code lost:
        r12 = r3;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:137:0x0242, code lost:
        r0 = e;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:138:0x0243, code lost:
        r12 = r3;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:139:0x0246, code lost:
        r0 = e;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:140:0x0247, code lost:
        r12 = r3;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:161:0x02af, code lost:
        r0 = e;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:173:0x02db, code lost:
        logger.debug("Failed to extract for " + r2 + ": " + r1);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:26:0x004a, code lost:
        r0 = e;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:27:0x004b, code lost:
        r26 = r6;
     */
    /* JADX WARNING: Failed to process nested try/catch */
    /* JADX WARNING: Removed duplicated region for block: B:102:0x017b  */
    /* JADX WARNING: Removed duplicated region for block: B:110:0x0197 A[Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }] */
    /* JADX WARNING: Removed duplicated region for block: B:117:0x01de  */
    /* JADX WARNING: Removed duplicated region for block: B:161:0x02af A[ExcHandler: URISyntaxException (e java.net.URISyntaxException), Splitter:B:107:0x018f] */
    /* JADX WARNING: Removed duplicated region for block: B:173:0x02db A[Catch:{ UnsatisfiedLinkError -> 0x02fa, IOException | URISyntaxException -> 0x02b3 }] */
    /* JADX WARNING: Removed duplicated region for block: B:184:0x0316 A[Catch:{ UnsatisfiedLinkError -> 0x02fa, IOException | URISyntaxException -> 0x02b3 }] */
    /* JADX WARNING: Removed duplicated region for block: B:190:0x01dc A[SYNTHETIC] */
    /* JADX WARNING: Removed duplicated region for block: B:196:0x01c8 A[SYNTHETIC] */
    /* JADX WARNING: Removed duplicated region for block: B:26:0x004a A[ExcHandler: IOException | URISyntaxException (e java.lang.Throwable), Splitter:B:48:0x0093] */
    /* JADX WARNING: Removed duplicated region for block: B:55:0x00be A[Catch:{ UnsatisfiedLinkError -> 0x025c, IOException | URISyntaxException -> 0x0258 }] */
    /* JADX WARNING: Removed duplicated region for block: B:88:0x013e A[Catch:{ IOException | RuntimeException -> 0x0180 }] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public static synchronized java.lang.String loadLibrary(java.net.URL[] r28, java.lang.String r29, java.lang.String... r30) {
        /*
            r1 = r28
            r2 = r29
            r3 = r30
            java.lang.Class<org.bytedeco.javacpp.Loader> r4 = org.bytedeco.javacpp.Loader.class
            monitor-enter(r4)
            boolean r0 = isLoadLibraries()     // Catch:{ all -> 0x0335 }
            r5 = 0
            if (r0 != 0) goto L_0x0012
            monitor-exit(r4)
            return r5
        L_0x0012:
            java.lang.String r0 = "#"
            java.lang.String[] r0 = r2.split(r0)     // Catch:{ all -> 0x0335 }
            r6 = r0
            r7 = 0
            r0 = r6[r7]     // Catch:{ all -> 0x0335 }
            int r8 = r6.length     // Catch:{ all -> 0x0335 }
            r9 = 1
            if (r8 <= r9) goto L_0x0024
            r8 = r6[r9]     // Catch:{ all -> 0x0335 }
            r0 = r8
            goto L_0x0025
        L_0x0024:
            r8 = r0
        L_0x0025:
            java.util.Map<java.lang.String, java.lang.String> r0 = loadedLibraries     // Catch:{ all -> 0x0335 }
            java.lang.Object r0 = r0.get(r8)     // Catch:{ all -> 0x0335 }
            java.lang.String r0 = (java.lang.String) r0     // Catch:{ all -> 0x0335 }
            r9 = r0
            r10 = r5
            int r11 = r1.length     // Catch:{ UnsatisfiedLinkError -> 0x02fa, IOException | URISyntaxException -> 0x02b3 }
            r12 = r10
            r10 = 0
        L_0x0032:
            if (r10 >= r11) goto L_0x0261
            r0 = r1[r10]     // Catch:{ UnsatisfiedLinkError -> 0x025c, IOException | URISyntaxException -> 0x0258 }
            r13 = r0
            java.net.URI r0 = r13.toURI()     // Catch:{ UnsatisfiedLinkError -> 0x025c, IOException | URISyntaxException -> 0x0258 }
            r14 = r0
            r15 = r5
            java.io.File r0 = new java.io.File     // Catch:{ Exception -> 0x0054 }
            r0.<init>(r14)     // Catch:{ Exception -> 0x0054 }
            r15 = r0
            r26 = r6
            r27 = r11
            goto L_0x01da
        L_0x004a:
            r0 = move-exception
            r26 = r6
            goto L_0x02b7
        L_0x004f:
            r0 = move-exception
            r26 = r6
            goto L_0x02fe
        L_0x0054:
            r0 = move-exception
            r16 = r0
            java.io.File r0 = cacheResource((java.net.URL) r13, (java.lang.String) r9)     // Catch:{ UnsatisfiedLinkError -> 0x025c, IOException | URISyntaxException -> 0x0258 }
            r17 = r0
            if (r17 == 0) goto L_0x0063
            r0 = r17
        L_0x0061:
            r15 = r0
            goto L_0x0089
        L_0x0063:
            java.io.File r0 = new java.io.File     // Catch:{ IllegalArgumentException | URISyntaxException -> 0x007d }
            java.net.URI r5 = new java.net.URI     // Catch:{ IllegalArgumentException | URISyntaxException -> 0x007d }
            java.lang.String r7 = r14.toString()     // Catch:{ IllegalArgumentException | URISyntaxException -> 0x007d }
            java.lang.String r1 = "#"
            java.lang.String[] r1 = r7.split(r1)     // Catch:{ IllegalArgumentException | URISyntaxException -> 0x007d }
            r7 = 0
            r1 = r1[r7]     // Catch:{ IllegalArgumentException | URISyntaxException -> 0x007d }
            r5.<init>(r1)     // Catch:{ IllegalArgumentException | URISyntaxException -> 0x007d }
            r0.<init>(r5)     // Catch:{ IllegalArgumentException | URISyntaxException -> 0x007d }
            goto L_0x0061
        L_0x007b:
            r0 = move-exception
            goto L_0x008a
        L_0x007d:
            r0 = move-exception
            java.io.File r1 = new java.io.File     // Catch:{ Exception -> 0x007b }
            java.lang.String r5 = r14.getPath()     // Catch:{ Exception -> 0x007b }
            r1.<init>(r5)     // Catch:{ Exception -> 0x007b }
            r0 = r1
            goto L_0x0061
        L_0x0089:
            goto L_0x00b1
        L_0x008a:
            org.bytedeco.javacpp.tools.Logger r1 = logger     // Catch:{ UnsatisfiedLinkError -> 0x025c, IOException | URISyntaxException -> 0x0258 }
            boolean r1 = r1.isDebugEnabled()     // Catch:{ UnsatisfiedLinkError -> 0x025c, IOException | URISyntaxException -> 0x0258 }
            if (r1 == 0) goto L_0x00b1
            org.bytedeco.javacpp.tools.Logger r1 = logger     // Catch:{ UnsatisfiedLinkError -> 0x004f, IOException | URISyntaxException -> 0x004a, IOException | URISyntaxException -> 0x004a }
            java.lang.StringBuilder r5 = new java.lang.StringBuilder     // Catch:{ UnsatisfiedLinkError -> 0x004f, IOException | URISyntaxException -> 0x004a, IOException | URISyntaxException -> 0x004a }
            r5.<init>()     // Catch:{ UnsatisfiedLinkError -> 0x004f, IOException | URISyntaxException -> 0x004a, IOException | URISyntaxException -> 0x004a }
            java.lang.String r7 = "Failed to access "
            r5.append(r7)     // Catch:{ UnsatisfiedLinkError -> 0x004f, IOException | URISyntaxException -> 0x004a, IOException | URISyntaxException -> 0x004a }
            r5.append(r14)     // Catch:{ UnsatisfiedLinkError -> 0x004f, IOException | URISyntaxException -> 0x004a, IOException | URISyntaxException -> 0x004a }
            java.lang.String r7 = ": "
            r5.append(r7)     // Catch:{ UnsatisfiedLinkError -> 0x004f, IOException | URISyntaxException -> 0x004a, IOException | URISyntaxException -> 0x004a }
            r5.append(r0)     // Catch:{ UnsatisfiedLinkError -> 0x004f, IOException | URISyntaxException -> 0x004a, IOException | URISyntaxException -> 0x004a }
            java.lang.String r5 = r5.toString()     // Catch:{ UnsatisfiedLinkError -> 0x004f, IOException | URISyntaxException -> 0x004a, IOException | URISyntaxException -> 0x004a }
            r1.debug(r5)     // Catch:{ UnsatisfiedLinkError -> 0x004f, IOException | URISyntaxException -> 0x004a, IOException | URISyntaxException -> 0x004a }
        L_0x00b1:
            if (r15 == 0) goto L_0x01d6
            if (r3 == 0) goto L_0x01d6
            java.io.File r0 = r15.getParentFile()     // Catch:{ UnsatisfiedLinkError -> 0x025c, IOException | URISyntaxException -> 0x0258 }
            r1 = r0
            int r5 = r3.length     // Catch:{ UnsatisfiedLinkError -> 0x025c, IOException | URISyntaxException -> 0x0258 }
            r7 = 0
        L_0x00bc:
            if (r7 >= r5) goto L_0x01d6
            r0 = r3[r7]     // Catch:{ UnsatisfiedLinkError -> 0x025c, IOException | URISyntaxException -> 0x0258 }
            r18 = r0
            java.io.File r0 = new java.io.File     // Catch:{ UnsatisfiedLinkError -> 0x025c, IOException | URISyntaxException -> 0x0258 }
            r3 = r18
            r0.<init>(r3)     // Catch:{ UnsatisfiedLinkError -> 0x025c, IOException | URISyntaxException -> 0x0258 }
            r19 = r0
            java.io.File r0 = r19.getParentFile()     // Catch:{ UnsatisfiedLinkError -> 0x025c, IOException | URISyntaxException -> 0x0258 }
            r20 = r0
            r21 = r3
            r3 = r20
            if (r3 == 0) goto L_0x01c0
            boolean r0 = r3.equals(r1)     // Catch:{ UnsatisfiedLinkError -> 0x025c, IOException | URISyntaxException -> 0x0258 }
            if (r0 != 0) goto L_0x01c0
            java.io.File r0 = new java.io.File     // Catch:{ UnsatisfiedLinkError -> 0x025c, IOException | URISyntaxException -> 0x0258 }
            r22 = r3
            java.lang.String r3 = r19.getName()     // Catch:{ UnsatisfiedLinkError -> 0x025c, IOException | URISyntaxException -> 0x0258 }
            r0.<init>(r1, r3)     // Catch:{ UnsatisfiedLinkError -> 0x025c, IOException | URISyntaxException -> 0x0258 }
            r3 = r0
            java.nio.file.Path r0 = r3.toPath()     // Catch:{ IOException | RuntimeException -> 0x0186 }
            java.nio.file.Path r18 = r19.toPath()     // Catch:{ IOException | RuntimeException -> 0x0186 }
            r23 = r18
            boolean r18 = r3.exists()     // Catch:{ IOException | RuntimeException -> 0x0186 }
            if (r18 == 0) goto L_0x012c
            boolean r18 = java.nio.file.Files.isSymbolicLink(r0)     // Catch:{ IOException | RuntimeException -> 0x0121 }
            if (r18 == 0) goto L_0x012c
            r24 = r1
            java.nio.file.Path r1 = java.nio.file.Files.readSymbolicLink(r0)     // Catch:{ IOException | RuntimeException -> 0x0118 }
            r25 = r5
            r5 = r23
            boolean r1 = r1.equals(r5)     // Catch:{ IOException | RuntimeException -> 0x0116 }
            if (r1 != 0) goto L_0x0110
            goto L_0x0132
        L_0x0110:
            r26 = r6
            r27 = r11
            goto L_0x017f
        L_0x0116:
            r0 = move-exception
            goto L_0x011b
        L_0x0118:
            r0 = move-exception
            r25 = r5
        L_0x011b:
            r26 = r6
        L_0x011d:
            r27 = r11
            goto L_0x018f
        L_0x0121:
            r0 = move-exception
            r24 = r1
            r25 = r5
            r26 = r6
            r27 = r11
            goto L_0x018f
        L_0x012c:
            r24 = r1
            r25 = r5
            r5 = r23
        L_0x0132:
            boolean r1 = r5.isAbsolute()     // Catch:{ IOException | RuntimeException -> 0x0180 }
            if (r1 == 0) goto L_0x017b
            boolean r1 = r5.equals(r0)     // Catch:{ IOException | RuntimeException -> 0x0180 }
            if (r1 != 0) goto L_0x017b
            org.bytedeco.javacpp.tools.Logger r1 = logger     // Catch:{ IOException | RuntimeException -> 0x0180 }
            boolean r1 = r1.isDebugEnabled()     // Catch:{ IOException | RuntimeException -> 0x0180 }
            if (r1 == 0) goto L_0x016b
            org.bytedeco.javacpp.tools.Logger r1 = logger     // Catch:{ IOException | RuntimeException -> 0x0180 }
            r26 = r6
            java.lang.StringBuilder r6 = new java.lang.StringBuilder     // Catch:{ IOException | RuntimeException -> 0x0169 }
            r6.<init>()     // Catch:{ IOException | RuntimeException -> 0x0169 }
            r27 = r11
            java.lang.String r11 = "Creating symbolic link "
            r6.append(r11)     // Catch:{ IOException | RuntimeException -> 0x0179 }
            r6.append(r0)     // Catch:{ IOException | RuntimeException -> 0x0179 }
            java.lang.String r11 = " to "
            r6.append(r11)     // Catch:{ IOException | RuntimeException -> 0x0179 }
            r6.append(r5)     // Catch:{ IOException | RuntimeException -> 0x0179 }
            java.lang.String r6 = r6.toString()     // Catch:{ IOException | RuntimeException -> 0x0179 }
            r1.debug(r6)     // Catch:{ IOException | RuntimeException -> 0x0179 }
            goto L_0x016f
        L_0x0169:
            r0 = move-exception
            goto L_0x011d
        L_0x016b:
            r26 = r6
            r27 = r11
        L_0x016f:
            r3.delete()     // Catch:{ IOException | RuntimeException -> 0x0179 }
            r1 = 0
            java.nio.file.attribute.FileAttribute[] r6 = new java.nio.file.attribute.FileAttribute[r1]     // Catch:{ IOException | RuntimeException -> 0x0179 }
            java.nio.file.Files.createSymbolicLink(r0, r5, r6)     // Catch:{ IOException | RuntimeException -> 0x0179 }
            goto L_0x017f
        L_0x0179:
            r0 = move-exception
            goto L_0x018f
        L_0x017b:
            r26 = r6
            r27 = r11
        L_0x017f:
            goto L_0x01c8
        L_0x0180:
            r0 = move-exception
            r26 = r6
            r27 = r11
            goto L_0x018f
        L_0x0186:
            r0 = move-exception
            r24 = r1
            r25 = r5
            r26 = r6
            r27 = r11
        L_0x018f:
            org.bytedeco.javacpp.tools.Logger r1 = logger     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            boolean r1 = r1.isDebugEnabled()     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            if (r1 == 0) goto L_0x01c8
            org.bytedeco.javacpp.tools.Logger r1 = logger     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            java.lang.StringBuilder r5 = new java.lang.StringBuilder     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            r5.<init>()     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            java.lang.String r6 = "Failed to create symbolic link "
            r5.append(r6)     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            r5.append(r3)     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            java.lang.String r6 = " to "
            r5.append(r6)     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            r6 = r19
            r5.append(r6)     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            java.lang.String r11 = ": "
            r5.append(r11)     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            r5.append(r0)     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            java.lang.String r5 = r5.toString()     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            r1.debug(r5)     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            goto L_0x01c8
        L_0x01c0:
            r24 = r1
            r25 = r5
            r26 = r6
            r27 = r11
        L_0x01c8:
            int r7 = r7 + 1
            r1 = r24
            r5 = r25
            r6 = r26
            r11 = r27
            r3 = r30
            goto L_0x00bc
        L_0x01d6:
            r26 = r6
            r27 = r11
        L_0x01da:
            if (r9 == 0) goto L_0x01de
            monitor-exit(r4)
            return r9
        L_0x01de:
            if (r15 == 0) goto L_0x024a
            boolean r0 = r15.exists()     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            if (r0 == 0) goto L_0x024a
            java.lang.String r0 = r15.getAbsolutePath()     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            r1 = r0
            org.bytedeco.javacpp.tools.Logger r0 = logger     // Catch:{ UnsatisfiedLinkError -> 0x0213, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            boolean r0 = r0.isDebugEnabled()     // Catch:{ UnsatisfiedLinkError -> 0x0213, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            if (r0 == 0) goto L_0x0209
            org.bytedeco.javacpp.tools.Logger r0 = logger     // Catch:{ UnsatisfiedLinkError -> 0x0213, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            java.lang.StringBuilder r3 = new java.lang.StringBuilder     // Catch:{ UnsatisfiedLinkError -> 0x0213, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            r3.<init>()     // Catch:{ UnsatisfiedLinkError -> 0x0213, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            java.lang.String r5 = "Loading "
            r3.append(r5)     // Catch:{ UnsatisfiedLinkError -> 0x0213, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            r3.append(r1)     // Catch:{ UnsatisfiedLinkError -> 0x0213, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            java.lang.String r3 = r3.toString()     // Catch:{ UnsatisfiedLinkError -> 0x0213, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            r0.debug(r3)     // Catch:{ UnsatisfiedLinkError -> 0x0213, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
        L_0x0209:
            java.util.Map<java.lang.String, java.lang.String> r0 = loadedLibraries     // Catch:{ UnsatisfiedLinkError -> 0x0213, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            r0.put(r8, r1)     // Catch:{ UnsatisfiedLinkError -> 0x0213, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            java.lang.System.load(r1)     // Catch:{ UnsatisfiedLinkError -> 0x0213, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            monitor-exit(r4)
            return r1
        L_0x0213:
            r0 = move-exception
            r3 = r0
            java.util.Map<java.lang.String, java.lang.String> r5 = loadedLibraries     // Catch:{ UnsatisfiedLinkError -> 0x0246, IOException | URISyntaxException -> 0x0242 }
            r5.remove(r8)     // Catch:{ UnsatisfiedLinkError -> 0x0246, IOException | URISyntaxException -> 0x0242 }
            org.bytedeco.javacpp.tools.Logger r5 = logger     // Catch:{ UnsatisfiedLinkError -> 0x0246, IOException | URISyntaxException -> 0x0242 }
            boolean r5 = r5.isDebugEnabled()     // Catch:{ UnsatisfiedLinkError -> 0x0246, IOException | URISyntaxException -> 0x0242 }
            if (r5 == 0) goto L_0x0240
            org.bytedeco.javacpp.tools.Logger r5 = logger     // Catch:{ UnsatisfiedLinkError -> 0x0246, IOException | URISyntaxException -> 0x0242 }
            java.lang.StringBuilder r6 = new java.lang.StringBuilder     // Catch:{ UnsatisfiedLinkError -> 0x0246, IOException | URISyntaxException -> 0x0242 }
            r6.<init>()     // Catch:{ UnsatisfiedLinkError -> 0x0246, IOException | URISyntaxException -> 0x0242 }
            java.lang.String r7 = "Failed to load "
            r6.append(r7)     // Catch:{ UnsatisfiedLinkError -> 0x0246, IOException | URISyntaxException -> 0x0242 }
            r6.append(r1)     // Catch:{ UnsatisfiedLinkError -> 0x0246, IOException | URISyntaxException -> 0x0242 }
            java.lang.String r7 = ": "
            r6.append(r7)     // Catch:{ UnsatisfiedLinkError -> 0x0246, IOException | URISyntaxException -> 0x0242 }
            r6.append(r0)     // Catch:{ UnsatisfiedLinkError -> 0x0246, IOException | URISyntaxException -> 0x0242 }
            java.lang.String r6 = r6.toString()     // Catch:{ UnsatisfiedLinkError -> 0x0246, IOException | URISyntaxException -> 0x0242 }
            r5.debug(r6)     // Catch:{ UnsatisfiedLinkError -> 0x0246, IOException | URISyntaxException -> 0x0242 }
        L_0x0240:
            r12 = r3
            goto L_0x024a
        L_0x0242:
            r0 = move-exception
            r12 = r3
            goto L_0x02b7
        L_0x0246:
            r0 = move-exception
            r12 = r3
            goto L_0x02fe
        L_0x024a:
            int r10 = r10 + 1
            r6 = r26
            r11 = r27
            r1 = r28
            r3 = r30
            r5 = 0
            r7 = 0
            goto L_0x0032
        L_0x0258:
            r0 = move-exception
            r26 = r6
            goto L_0x02b7
        L_0x025c:
            r0 = move-exception
            r26 = r6
            goto L_0x02fe
        L_0x0261:
            r26 = r6
            if (r9 == 0) goto L_0x0267
            monitor-exit(r4)
            return r9
        L_0x0267:
            java.lang.String r0 = r29.trim()     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            java.lang.String r1 = "#"
            boolean r0 = r0.endsWith(r1)     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            if (r0 != 0) goto L_0x02ac
            java.lang.String r0 = "#"
            java.lang.String[] r0 = r2.split(r0)     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            r1 = 0
            r0 = r0[r1]     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            java.lang.String r3 = "@"
            java.lang.String[] r0 = r0.split(r3)     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            r0 = r0[r1]     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            org.bytedeco.javacpp.tools.Logger r1 = logger     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            boolean r1 = r1.isDebugEnabled()     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            if (r1 == 0) goto L_0x02a2
            org.bytedeco.javacpp.tools.Logger r1 = logger     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            java.lang.StringBuilder r3 = new java.lang.StringBuilder     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            r3.<init>()     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            java.lang.String r5 = "Loading library "
            r3.append(r5)     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            r3.append(r0)     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            java.lang.String r3 = r3.toString()     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            r1.debug(r3)     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
        L_0x02a2:
            java.util.Map<java.lang.String, java.lang.String> r1 = loadedLibraries     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            r1.put(r8, r0)     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            java.lang.System.loadLibrary(r0)     // Catch:{ UnsatisfiedLinkError -> 0x02b1, URISyntaxException -> 0x02af, URISyntaxException -> 0x02af }
            monitor-exit(r4)
            return r0
        L_0x02ac:
            monitor-exit(r4)
            r1 = 0
            return r1
        L_0x02af:
            r0 = move-exception
            goto L_0x02b7
        L_0x02b1:
            r0 = move-exception
            goto L_0x02fe
        L_0x02b3:
            r0 = move-exception
            r26 = r6
            r12 = r10
        L_0x02b7:
            java.util.Map<java.lang.String, java.lang.String> r1 = loadedLibraries     // Catch:{ all -> 0x0335 }
            r1.remove(r8)     // Catch:{ all -> 0x0335 }
            if (r12 == 0) goto L_0x02c7
            java.lang.Throwable r1 = r0.getCause()     // Catch:{ all -> 0x0335 }
            if (r1 != 0) goto L_0x02c7
            r0.initCause(r12)     // Catch:{ all -> 0x0335 }
        L_0x02c7:
            java.lang.UnsatisfiedLinkError r1 = new java.lang.UnsatisfiedLinkError     // Catch:{ all -> 0x0335 }
            java.lang.String r3 = r0.toString()     // Catch:{ all -> 0x0335 }
            r1.<init>(r3)     // Catch:{ all -> 0x0335 }
            r1.initCause(r0)     // Catch:{ all -> 0x0335 }
            org.bytedeco.javacpp.tools.Logger r3 = logger     // Catch:{ all -> 0x0335 }
            boolean r3 = r3.isDebugEnabled()     // Catch:{ all -> 0x0335 }
            if (r3 == 0) goto L_0x02f9
            org.bytedeco.javacpp.tools.Logger r3 = logger     // Catch:{ all -> 0x0335 }
            java.lang.StringBuilder r5 = new java.lang.StringBuilder     // Catch:{ all -> 0x0335 }
            r5.<init>()     // Catch:{ all -> 0x0335 }
            java.lang.String r6 = "Failed to extract for "
            r5.append(r6)     // Catch:{ all -> 0x0335 }
            r5.append(r2)     // Catch:{ all -> 0x0335 }
            java.lang.String r6 = ": "
            r5.append(r6)     // Catch:{ all -> 0x0335 }
            r5.append(r1)     // Catch:{ all -> 0x0335 }
            java.lang.String r5 = r5.toString()     // Catch:{ all -> 0x0335 }
            r3.debug(r5)     // Catch:{ all -> 0x0335 }
        L_0x02f9:
            throw r1     // Catch:{ all -> 0x0335 }
        L_0x02fa:
            r0 = move-exception
            r26 = r6
            r12 = r10
        L_0x02fe:
            java.util.Map<java.lang.String, java.lang.String> r1 = loadedLibraries     // Catch:{ all -> 0x0335 }
            r1.remove(r8)     // Catch:{ all -> 0x0335 }
            if (r12 == 0) goto L_0x030e
            java.lang.Throwable r1 = r0.getCause()     // Catch:{ all -> 0x0335 }
            if (r1 != 0) goto L_0x030e
            r0.initCause(r12)     // Catch:{ all -> 0x0335 }
        L_0x030e:
            org.bytedeco.javacpp.tools.Logger r1 = logger     // Catch:{ all -> 0x0335 }
            boolean r1 = r1.isDebugEnabled()     // Catch:{ all -> 0x0335 }
            if (r1 == 0) goto L_0x0334
            org.bytedeco.javacpp.tools.Logger r1 = logger     // Catch:{ all -> 0x0335 }
            java.lang.StringBuilder r3 = new java.lang.StringBuilder     // Catch:{ all -> 0x0335 }
            r3.<init>()     // Catch:{ all -> 0x0335 }
            java.lang.String r5 = "Failed to load for "
            r3.append(r5)     // Catch:{ all -> 0x0335 }
            r3.append(r2)     // Catch:{ all -> 0x0335 }
            java.lang.String r5 = ": "
            r3.append(r5)     // Catch:{ all -> 0x0335 }
            r3.append(r0)     // Catch:{ all -> 0x0335 }
            java.lang.String r3 = r3.toString()     // Catch:{ all -> 0x0335 }
            r1.debug(r3)     // Catch:{ all -> 0x0335 }
        L_0x0334:
            throw r0     // Catch:{ all -> 0x0335 }
        L_0x0335:
            r0 = move-exception
            monitor-exit(r4)
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.Loader.loadLibrary(java.net.URL[], java.lang.String, java.lang.String[]):java.lang.String");
    }

    /* JADX WARNING: Code restructure failed: missing block: B:51:0x00f2, code lost:
        if (java.nio.file.Files.readSymbolicLink(r0).equals(r9) != false) goto L_0x0135;
     */
    /* JADX WARNING: Removed duplicated region for block: B:119:0x0243  */
    /* JADX WARNING: Removed duplicated region for block: B:135:? A[RETURN, SYNTHETIC] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public static java.lang.String createLibraryLink(java.lang.String r30, org.bytedeco.javacpp.ClassProperties r31, java.lang.String r32, java.lang.String... r33) {
        /*
            r1 = r30
            r2 = r31
            r3 = r32
            r4 = r33
            java.io.File r0 = new java.io.File
            r0.<init>(r1)
            r5 = r0
            java.lang.String r6 = r5.getParent()
            java.lang.String r7 = r5.getName()
            r0 = 0
            r8 = 1
            r9 = 0
            if (r3 == 0) goto L_0x0022
            java.lang.String r10 = "#"
            java.lang.String[] r10 = r3.split(r10)
            goto L_0x0028
        L_0x0022:
            java.lang.String[] r10 = new java.lang.String[r8]
            java.lang.String r11 = ""
            r10[r9] = r11
        L_0x0028:
            int r11 = r10.length
            if (r11 <= r8) goto L_0x002e
            r11 = r10[r8]
            goto L_0x0030
        L_0x002e:
            r11 = r10[r9]
        L_0x0030:
            java.lang.String r12 = "@"
            java.lang.String[] r11 = r11.split(r12)
            r12 = r11[r9]
            int r13 = r11.length
            if (r13 <= r8) goto L_0x0040
            int r13 = r11.length
            int r13 = r13 - r8
            r13 = r11[r13]
            goto L_0x0042
        L_0x0040:
            java.lang.String r13 = ""
        L_0x0042:
            boolean r14 = r7.contains(r12)
            if (r14 != 0) goto L_0x0049
            return r1
        L_0x0049:
            java.lang.String r14 = "platform.library.suffix"
            java.util.List r14 = r2.get(r14)
            java.util.Iterator r14 = r14.iterator()
        L_0x0053:
            boolean r15 = r14.hasNext()
            if (r15 == 0) goto L_0x00a1
            java.lang.Object r15 = r14.next()
            java.lang.String r15 = (java.lang.String) r15
            int r8 = r7.lastIndexOf(r15)
            int r17 = r13.length()
            if (r17 == 0) goto L_0x006e
            int r17 = r7.lastIndexOf(r13)
            goto L_0x0074
        L_0x006e:
            java.lang.String r9 = "."
            int r17 = r7.indexOf(r9)
        L_0x0074:
            r9 = r17
            if (r8 <= 0) goto L_0x0098
            if (r9 <= 0) goto L_0x0098
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            if (r8 >= r9) goto L_0x0085
            r18 = r0
            r0 = r8
            goto L_0x0088
        L_0x0085:
            r18 = r0
            r0 = r9
        L_0x0088:
            r1 = 0
            java.lang.String r0 = r7.substring(r1, r0)
            r14.append(r0)
            r14.append(r15)
            java.lang.String r0 = r14.toString()
            goto L_0x00a3
        L_0x0098:
            r18 = r0
            r0 = r18
            r1 = r30
            r8 = 1
            r9 = 0
            goto L_0x0053
        L_0x00a1:
            r18 = r0
        L_0x00a3:
            if (r0 != 0) goto L_0x00c4
            java.lang.String r1 = "platform.library.suffix"
            java.util.List r1 = r2.get(r1)
            java.util.Iterator r1 = r1.iterator()
        L_0x00af:
            boolean r8 = r1.hasNext()
            if (r8 == 0) goto L_0x00c4
            java.lang.Object r8 = r1.next()
            java.lang.String r8 = (java.lang.String) r8
            boolean r9 = r7.endsWith(r8)
            if (r9 == 0) goto L_0x00c3
            r0 = r7
            goto L_0x00c4
        L_0x00c3:
            goto L_0x00af
        L_0x00c4:
            r1 = r0
            if (r1 == 0) goto L_0x0263
            int r0 = r1.length()
            if (r0 <= 0) goto L_0x0263
            java.io.File r0 = new java.io.File
            r0.<init>(r6, r1)
            r8 = r0
            java.nio.file.Path r0 = r8.toPath()     // Catch:{ IOException | RuntimeException -> 0x0234 }
            r9 = 0
            java.lang.String[] r14 = new java.lang.String[r9]     // Catch:{ IOException | RuntimeException -> 0x0234 }
            java.nio.file.Path r9 = java.nio.file.Paths.get(r7, r14)     // Catch:{ IOException | RuntimeException -> 0x0234 }
            boolean r14 = r8.exists()     // Catch:{ IOException | RuntimeException -> 0x0234 }
            if (r14 == 0) goto L_0x00fe
            boolean r14 = java.nio.file.Files.isSymbolicLink(r0)     // Catch:{ IOException | RuntimeException -> 0x00f5 }
            if (r14 == 0) goto L_0x00fe
            java.nio.file.Path r14 = java.nio.file.Files.readSymbolicLink(r0)     // Catch:{ IOException | RuntimeException -> 0x00f5 }
            boolean r14 = r14.equals(r9)     // Catch:{ IOException | RuntimeException -> 0x00f5 }
            if (r14 != 0) goto L_0x0135
            goto L_0x00fe
        L_0x00f5:
            r0 = move-exception
            r22 = r1
            r29 = r5
            r21 = r30
            goto L_0x023b
        L_0x00fe:
            boolean r14 = r9.isAbsolute()     // Catch:{ IOException | RuntimeException -> 0x0234 }
            if (r14 != 0) goto L_0x0135
            java.nio.file.Path r14 = r0.getFileName()     // Catch:{ IOException | RuntimeException -> 0x00f5 }
            boolean r14 = r9.equals(r14)     // Catch:{ IOException | RuntimeException -> 0x00f5 }
            if (r14 != 0) goto L_0x0135
            org.bytedeco.javacpp.tools.Logger r14 = logger     // Catch:{ IOException | RuntimeException -> 0x00f5 }
            boolean r14 = r14.isDebugEnabled()     // Catch:{ IOException | RuntimeException -> 0x00f5 }
            if (r14 == 0) goto L_0x012c
            org.bytedeco.javacpp.tools.Logger r14 = logger     // Catch:{ IOException | RuntimeException -> 0x00f5 }
            java.lang.StringBuilder r15 = new java.lang.StringBuilder     // Catch:{ IOException | RuntimeException -> 0x00f5 }
            r15.<init>()     // Catch:{ IOException | RuntimeException -> 0x00f5 }
            java.lang.String r2 = "Creating symbolic link "
            r15.append(r2)     // Catch:{ IOException | RuntimeException -> 0x00f5 }
            r15.append(r0)     // Catch:{ IOException | RuntimeException -> 0x00f5 }
            java.lang.String r2 = r15.toString()     // Catch:{ IOException | RuntimeException -> 0x00f5 }
            r14.debug(r2)     // Catch:{ IOException | RuntimeException -> 0x00f5 }
        L_0x012c:
            r8.delete()     // Catch:{ IOException | RuntimeException -> 0x00f5 }
            r2 = 0
            java.nio.file.attribute.FileAttribute[] r14 = new java.nio.file.attribute.FileAttribute[r2]     // Catch:{ IOException | RuntimeException -> 0x00f5 }
            java.nio.file.Files.createSymbolicLink(r0, r9, r14)     // Catch:{ IOException | RuntimeException -> 0x00f5 }
        L_0x0135:
            java.lang.String r2 = r8.toString()     // Catch:{ IOException | RuntimeException -> 0x0234 }
            int r14 = r4.length     // Catch:{ IOException | RuntimeException -> 0x022c }
            r15 = 0
        L_0x013b:
            if (r15 >= r14) goto L_0x0225
            r17 = r4[r15]     // Catch:{ IOException | RuntimeException -> 0x022c }
            r19 = r17
            r20 = r0
            r0 = r19
            if (r0 != 0) goto L_0x0153
            r22 = r1
            r21 = r2
            r29 = r5
            r2 = 0
            r16 = 1
            goto L_0x020b
        L_0x0153:
            r21 = r2
            r2 = 2
            java.lang.String[] r2 = new java.lang.String[r2]     // Catch:{ IOException | RuntimeException -> 0x021f }
            r17 = 0
            r2[r17] = r1     // Catch:{ IOException | RuntimeException -> 0x021f }
            r16 = 1
            r2[r16] = r7     // Catch:{ IOException | RuntimeException -> 0x021f }
            r22 = r1
            int r1 = r2.length     // Catch:{ IOException | RuntimeException -> 0x021b }
            r3 = 0
        L_0x0164:
            if (r3 >= r1) goto L_0x0208
            r17 = r2[r3]     // Catch:{ IOException | RuntimeException -> 0x021b }
            r23 = r17
            r24 = r1
            java.io.File r1 = new java.io.File     // Catch:{ IOException | RuntimeException -> 0x021b }
            r25 = r2
            r2 = r23
            r1.<init>(r0, r2)     // Catch:{ IOException | RuntimeException -> 0x021b }
            java.nio.file.Path r17 = r1.toPath()     // Catch:{ IOException | RuntimeException -> 0x021b }
            r26 = r17
            r27 = r2
            r2 = 0
            java.lang.String[] r4 = new java.lang.String[r2]     // Catch:{ IOException | RuntimeException -> 0x021b }
            java.nio.file.Path r4 = java.nio.file.Paths.get(r0, r4)     // Catch:{ IOException | RuntimeException -> 0x021b }
            r28 = r0
            java.lang.String[] r0 = new java.lang.String[r2]     // Catch:{ IOException | RuntimeException -> 0x021b }
            java.nio.file.Path r0 = java.nio.file.Paths.get(r6, r0)     // Catch:{ IOException | RuntimeException -> 0x021b }
            java.nio.file.Path r0 = r4.relativize(r0)     // Catch:{ IOException | RuntimeException -> 0x021b }
            java.nio.file.Path r0 = r0.resolve(r7)     // Catch:{ IOException | RuntimeException -> 0x021b }
            boolean r2 = r1.exists()     // Catch:{ IOException | RuntimeException -> 0x021b }
            if (r2 == 0) goto L_0x01b4
            boolean r2 = java.nio.file.Files.isSymbolicLink(r26)     // Catch:{ IOException | RuntimeException -> 0x01af }
            if (r2 == 0) goto L_0x01b4
            java.nio.file.Path r2 = java.nio.file.Files.readSymbolicLink(r26)     // Catch:{ IOException | RuntimeException -> 0x01af }
            boolean r2 = r2.equals(r0)     // Catch:{ IOException | RuntimeException -> 0x01af }
            if (r2 != 0) goto L_0x01ab
            goto L_0x01b4
        L_0x01ab:
            r29 = r5
            r2 = 0
            goto L_0x01fa
        L_0x01af:
            r0 = move-exception
            r29 = r5
            goto L_0x023b
        L_0x01b4:
            boolean r2 = r0.isAbsolute()     // Catch:{ IOException | RuntimeException -> 0x021b }
            if (r2 != 0) goto L_0x01f7
            java.nio.file.Path r2 = r26.getFileName()     // Catch:{ IOException | RuntimeException -> 0x021b }
            boolean r2 = r0.equals(r2)     // Catch:{ IOException | RuntimeException -> 0x021b }
            if (r2 != 0) goto L_0x01f7
            org.bytedeco.javacpp.tools.Logger r2 = logger     // Catch:{ IOException | RuntimeException -> 0x021b }
            boolean r2 = r2.isDebugEnabled()     // Catch:{ IOException | RuntimeException -> 0x021b }
            if (r2 == 0) goto L_0x01e7
            org.bytedeco.javacpp.tools.Logger r2 = logger     // Catch:{ IOException | RuntimeException -> 0x021b }
            java.lang.StringBuilder r4 = new java.lang.StringBuilder     // Catch:{ IOException | RuntimeException -> 0x021b }
            r4.<init>()     // Catch:{ IOException | RuntimeException -> 0x021b }
            r29 = r5
            java.lang.String r5 = "Creating symbolic link "
            r4.append(r5)     // Catch:{ IOException | RuntimeException -> 0x01f5 }
            r5 = r26
            r4.append(r5)     // Catch:{ IOException | RuntimeException -> 0x01f5 }
            java.lang.String r4 = r4.toString()     // Catch:{ IOException | RuntimeException -> 0x01f5 }
            r2.debug(r4)     // Catch:{ IOException | RuntimeException -> 0x01f5 }
            goto L_0x01eb
        L_0x01e7:
            r29 = r5
            r5 = r26
        L_0x01eb:
            r1.delete()     // Catch:{ IOException | RuntimeException -> 0x01f5 }
            r2 = 0
            java.nio.file.attribute.FileAttribute[] r4 = new java.nio.file.attribute.FileAttribute[r2]     // Catch:{ IOException | RuntimeException -> 0x01f5 }
            java.nio.file.Files.createSymbolicLink(r5, r0, r4)     // Catch:{ IOException | RuntimeException -> 0x01f5 }
            goto L_0x01fa
        L_0x01f5:
            r0 = move-exception
            goto L_0x023b
        L_0x01f7:
            r29 = r5
            r2 = 0
        L_0x01fa:
            int r3 = r3 + 1
            r1 = r24
            r2 = r25
            r0 = r28
            r5 = r29
            r4 = r33
            goto L_0x0164
        L_0x0208:
            r29 = r5
            r2 = 0
        L_0x020b:
            int r15 = r15 + 1
            r0 = r20
            r2 = r21
            r1 = r22
            r5 = r29
            r3 = r32
            r4 = r33
            goto L_0x013b
        L_0x021b:
            r0 = move-exception
            r29 = r5
            goto L_0x023b
        L_0x021f:
            r0 = move-exception
            r22 = r1
            r29 = r5
            goto L_0x023b
        L_0x0225:
            r22 = r1
            r21 = r2
            r29 = r5
            goto L_0x0269
        L_0x022c:
            r0 = move-exception
            r22 = r1
            r21 = r2
            r29 = r5
            goto L_0x023b
        L_0x0234:
            r0 = move-exception
            r22 = r1
            r29 = r5
            r21 = r30
        L_0x023b:
            org.bytedeco.javacpp.tools.Logger r1 = logger
            boolean r1 = r1.isDebugEnabled()
            if (r1 == 0) goto L_0x0261
            org.bytedeco.javacpp.tools.Logger r1 = logger
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "Failed to create symbolic link "
            r2.append(r3)
            r2.append(r8)
            java.lang.String r3 = ": "
            r2.append(r3)
            r2.append(r0)
            java.lang.String r2 = r2.toString()
            r1.debug(r2)
        L_0x0261:
            r1 = 0
            return r1
        L_0x0263:
            r22 = r1
            r29 = r5
            r21 = r30
        L_0x0269:
            return r21
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.Loader.createLibraryLink(java.lang.String, org.bytedeco.javacpp.ClassProperties, java.lang.String, java.lang.String[]):java.lang.String");
    }

    static Class putMemberOffset(String typeName, String member, int offset) throws ClassNotFoundException {
        try {
            Class<?> c = Class.forName(typeName.replace(IOUtils.DIR_SEPARATOR_UNIX, '.'), false, Loader.class.getClassLoader());
            if (member != null) {
                putMemberOffset((Class<? extends Pointer>) c.asSubclass(Pointer.class), member, offset);
            }
            return c;
        } catch (ClassNotFoundException e) {
            Logger logger2 = logger;
            logger2.warn("Loader.putMemberOffset(): " + e);
            return null;
        }
    }

    static synchronized void putMemberOffset(Class<? extends Pointer> type, String member, int offset) {
        synchronized (Loader.class) {
            HashMap<String, Integer> offsets = memberOffsets.get(type);
            if (offsets == null) {
                WeakHashMap<Class<? extends Pointer>, HashMap<String, Integer>> weakHashMap = memberOffsets;
                HashMap<String, Integer> hashMap = new HashMap<>();
                offsets = hashMap;
                weakHashMap.put(type, hashMap);
            }
            offsets.put(member, Integer.valueOf(offset));
        }
    }

    public static int offsetof(Class<? extends Pointer> type, String member) {
        HashMap<String, Integer> offsets = memberOffsets.get(type);
        Class<? extends U> type2 = type;
        while (offsets == null && type2.getSuperclass() != null) {
            Class<? extends U> asSubclass = type2.getSuperclass().asSubclass(Pointer.class);
            offsets = memberOffsets.get(asSubclass);
            type2 = asSubclass;
        }
        return offsets.get(member).intValue();
    }

    public static int sizeof(Class<? extends Pointer> type) {
        return offsetof(type, "sizeof");
    }
}
