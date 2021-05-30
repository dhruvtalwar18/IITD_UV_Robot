package org.bytedeco.javacpp.tools;

import android.support.v4.os.EnvironmentCompat;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FilenameFilter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import java.util.jar.JarOutputStream;
import java.util.zip.ZipEntry;
import org.apache.commons.httpclient.HttpState;
import org.apache.commons.httpclient.cookie.CookiePolicy;
import org.apache.commons.io.IOUtils;
import org.bytedeco.javacpp.ClassProperties;
import org.bytedeco.javacpp.Loader;

public class Builder {
    String[] buildCommand;
    ClassScanner classScanner;
    boolean compile;
    Collection<String> compilerOptions;
    boolean copyLibs;
    boolean copyResources;
    boolean deleteJniFiles;
    String encoding;
    Map<String, String> environmentVariables;
    boolean generate;
    boolean header;
    String jarPrefix;
    final Logger logger;
    File outputDirectory;
    String outputName;
    Properties properties;
    File workingDirectory;

    /* access modifiers changed from: package-private */
    public File parse(String[] classPath, Class cls) throws IOException, ParserException {
        return new Parser(this.logger, this.properties, this.encoding, (String) null).parse(this.outputDirectory, classPath, cls);
    }

    /* access modifiers changed from: package-private */
    public void includeJavaPaths(ClassProperties properties2, boolean header2) {
        ClassProperties classProperties = properties2;
        if (!classProperties.getProperty("platform", "").startsWith("android")) {
            String platform = Loader.getPlatform();
            String[] jnipath = new String[2];
            String[] jvmpath = new String[2];
            final String[] strArr = jnipath;
            final String str = classProperties.getProperty("platform.link.prefix", "") + "jvm" + classProperties.getProperty("platform.link.suffix", "");
            final String[] strArr2 = jvmpath;
            final String str2 = classProperties.getProperty("platform.library.prefix", "") + "jvm" + classProperties.getProperty("platform.library.suffix", "");
            FilenameFilter filter = new FilenameFilter() {
                public boolean accept(File dir, String name) {
                    if (new File(dir, "jni.h").exists()) {
                        strArr[0] = dir.getAbsolutePath();
                    }
                    if (new File(dir, "jni_md.h").exists()) {
                        strArr[1] = dir.getAbsolutePath();
                    }
                    if (new File(dir, str).exists()) {
                        strArr2[0] = dir.getAbsolutePath();
                    }
                    if (new File(dir, str2).exists()) {
                        strArr2[1] = dir.getAbsolutePath();
                    }
                    return new File(dir, name).isDirectory();
                }
            };
            try {
                ArrayList<File> dirs = new ArrayList<>(Arrays.asList(new File(System.getProperty("java.home")).getParentFile().getCanonicalFile().listFiles(filter)));
                while (true) {
                    ArrayList<File> dirs2 = dirs;
                    if (dirs2.isEmpty()) {
                        break;
                    }
                    File d = dirs2.remove(dirs2.size() - 1);
                    String dpath = d.getPath();
                    File[] files = d.listFiles(filter);
                    if (!(dpath == null || files == null)) {
                        int length = files.length;
                        for (int i = 0; i < length; i++) {
                            File f = files[i];
                            try {
                                f = f.getCanonicalFile();
                            } catch (IOException e) {
                            }
                            if (!dpath.startsWith(f.getPath())) {
                                dirs2.add(f);
                            }
                        }
                    }
                    dirs = dirs2;
                }
                if (jnipath[0] != null && jnipath[0].equals(jnipath[1])) {
                    jnipath[1] = null;
                } else if (jnipath[0] == null && new File("/System/Library/Frameworks/JavaVM.framework/Headers/").isDirectory()) {
                    jnipath[0] = "/System/Library/Frameworks/JavaVM.framework/Headers/";
                }
                if (jvmpath[0] != null && jvmpath[0].equals(jvmpath[1])) {
                    jvmpath[1] = null;
                }
                classProperties.addAll("platform.includepath", jnipath);
                if (platform.equals(classProperties.getProperty("platform", platform))) {
                    if (header2) {
                        classProperties.get("platform.link").add(0, "jvm");
                        classProperties.addAll("platform.linkpath", jvmpath);
                    }
                    if (platform.startsWith("macosx")) {
                        classProperties.addAll("platform.framework", "JavaVM");
                    }
                }
            } catch (IOException | NullPointerException e2) {
                this.logger.warn("Could not include header files from java.home:" + e2);
            }
        }
    }

    /* access modifiers changed from: package-private */
    public int executeCommand(List<String> command, File workingDirectory2, Map<String, String> environmentVariables2) throws IOException, InterruptedException {
        boolean windows = Loader.getPlatform().startsWith("windows");
        for (int i = 0; i < command.size(); i++) {
            String arg = command.get(i);
            if (arg == null) {
                arg = "";
            }
            if (arg.trim().isEmpty() && windows) {
                arg = "\"\"";
            }
            command.set(i, arg);
        }
        String text = "";
        for (String s : command) {
            boolean hasSpaces = s.indexOf(" ") > 0 || s.isEmpty();
            if (hasSpaces) {
                StringBuilder sb = new StringBuilder();
                sb.append(text);
                sb.append(windows ? "\"" : "'");
                text = sb.toString();
            }
            String text2 = text + s;
            if (hasSpaces) {
                StringBuilder sb2 = new StringBuilder();
                sb2.append(text2);
                sb2.append(windows ? "\"" : "'");
                text2 = sb2.toString();
            }
            text = text2 + " ";
        }
        this.logger.info(text);
        ProcessBuilder pb = new ProcessBuilder(command);
        if (workingDirectory2 != null) {
            pb.directory(workingDirectory2);
        }
        if (environmentVariables2 != null) {
            for (Map.Entry<String, String> e : environmentVariables2.entrySet()) {
                if (!(e.getKey() == null || e.getValue() == null)) {
                    pb.environment().put(e.getKey(), e.getValue());
                }
            }
        }
        return pb.inheritIO().start().waitFor();
    }

    /* access modifiers changed from: package-private */
    public int compile(String[] sourceFilenames, String outputFilename, ClassProperties properties2, File workingDirectory2) throws IOException, InterruptedException {
        int i;
        List<String> allOptions;
        String compilerPath;
        Iterator<String> it;
        String s;
        String compilerPath2;
        String[] strArr = sourceFilenames;
        String str = outputFilename;
        ClassProperties classProperties = properties2;
        ArrayList<String> command = new ArrayList<>();
        includeJavaPaths(classProperties, this.header);
        String platform = Loader.getPlatform();
        String compilerPath3 = classProperties.getProperty("platform.compiler");
        command.add(compilerPath3);
        String p = classProperties.getProperty("platform.sysroot.prefix", "");
        for (String s2 : classProperties.get("platform.sysroot")) {
            if (new File(s2).isDirectory()) {
                if (p.endsWith(" ")) {
                    command.add(p.trim());
                    command.add(s2);
                } else {
                    command.add(p + s2);
                }
            }
        }
        String p2 = classProperties.getProperty("platform.toolchain.prefix", "");
        for (String s3 : classProperties.get("platform.toolchain")) {
            if (new File(s3).isDirectory()) {
                if (p2.endsWith(" ")) {
                    command.add(p2.trim());
                    command.add(s3);
                } else {
                    command.add(p2 + s3);
                }
            }
        }
        String p3 = classProperties.getProperty("platform.includepath.prefix", "");
        for (String s4 : classProperties.get("platform.includepath")) {
            if (new File(s4).isDirectory()) {
                if (p3.endsWith(" ")) {
                    command.add(p3.trim());
                    command.add(s4);
                } else {
                    command.add(p3 + s4);
                }
            }
        }
        Iterator<String> it2 = classProperties.get("platform.includeresource").iterator();
        while (true) {
            i = 0;
            if (!it2.hasNext()) {
                break;
            }
            File[] cacheResources = Loader.cacheResources(it2.next());
            int length = cacheResources.length;
            while (i < length) {
                File f = cacheResources[i];
                if (f.isDirectory()) {
                    if (p3.endsWith(" ")) {
                        command.add(p3.trim());
                        command.add(f.getCanonicalPath());
                    } else {
                        command.add(p3 + f.getCanonicalPath());
                    }
                }
                i++;
            }
        }
        for (int i2 = strArr.length - 1; i2 >= 0; i2--) {
            command.add(strArr[i2]);
        }
        List<String> allOptions2 = classProperties.get("platform.compiler.*");
        if (!allOptions2.contains("!default") && !allOptions2.contains(CookiePolicy.DEFAULT)) {
            allOptions2.add(0, CookiePolicy.DEFAULT);
        }
        for (String s5 : allOptions2) {
            if (!(s5 == null || s5.length() == 0)) {
                String p4 = "platform.compiler." + s5;
                String options = classProperties.getProperty(p4);
                if (options != null && options.length() > 0) {
                    command.addAll(Arrays.asList(options.split(" ")));
                } else if (!"!default".equals(s5) && !CookiePolicy.DEFAULT.equals(s5)) {
                    this.logger.warn("Could not get the property named \"" + p4 + "\"");
                }
            }
        }
        command.addAll(this.compilerOptions);
        String output = classProperties.getProperty("platform.compiler.output");
        int i3 = 1;
        while (true) {
            if (i3 >= 2) {
                if (output == null) {
                    break;
                }
                compilerPath = compilerPath3;
                allOptions = allOptions2;
                File file = workingDirectory2;
            } else {
                compilerPath = compilerPath3;
                allOptions = allOptions2;
                File file2 = workingDirectory2;
            }
            if (output != null && output.length() > 0) {
                command.addAll(Arrays.asList(output.split(" ")));
            }
            if (output == null || output.length() == 0 || output.endsWith(" ")) {
                command.add(str);
            } else {
                command.add(command.remove(command.size() - 1) + str);
            }
            i3++;
            output = classProperties.getProperty("platform.compiler.output" + i3);
            compilerPath3 = compilerPath;
            allOptions2 = allOptions;
            String[] strArr2 = sourceFilenames;
            i = 0;
        }
        String p5 = classProperties.getProperty("platform.linkpath.prefix", "");
        String p22 = classProperties.getProperty("platform.linkpath.prefix2");
        for (String s6 : classProperties.get("platform.linkpath")) {
            if (new File(s6).isDirectory()) {
                if (p5.endsWith(" ")) {
                    command.add(p5.trim());
                    command.add(s6);
                } else {
                    command.add(p5 + s6);
                }
                if (p22 != null) {
                    if (p22.endsWith(" ")) {
                        command.add(p22.trim());
                        command.add(s6);
                    } else {
                        command.add(p22 + s6);
                    }
                }
            }
        }
        for (String s7 : classProperties.get("platform.linkresource")) {
            File[] cacheResources2 = Loader.cacheResources(s7);
            int length2 = cacheResources2.length;
            while (i < length2) {
                File f2 = cacheResources2[i];
                if (f2.isDirectory()) {
                    if (p5.endsWith(" ")) {
                        command.add(p5.trim());
                        command.add(f2.getCanonicalPath());
                        compilerPath2 = compilerPath3;
                    } else {
                        StringBuilder sb = new StringBuilder();
                        sb.append(p5);
                        compilerPath2 = compilerPath3;
                        sb.append(f2.getCanonicalPath());
                        command.add(sb.toString());
                    }
                    if (p22 != null) {
                        if (p22.endsWith(" ")) {
                            command.add(p22.trim());
                            command.add(f2.getCanonicalPath());
                        } else {
                            command.add(p22 + f2.getCanonicalPath());
                        }
                    }
                } else {
                    compilerPath2 = compilerPath3;
                }
                i++;
                compilerPath3 = compilerPath2;
                String[] strArr3 = sourceFilenames;
            }
            String[] strArr4 = sourceFilenames;
            i = 0;
        }
        String p6 = classProperties.getProperty("platform.link.prefix", "");
        String x = classProperties.getProperty("platform.link.suffix", "");
        String linkPrefix = "";
        String linkSuffix = "";
        List<String> linkBeforeOptions = new ArrayList<>();
        List<String> linkAfterOptions = new ArrayList<>();
        if (p6.endsWith(" ")) {
            linkBeforeOptions.addAll(Arrays.asList(p6.trim().split(" ")));
        } else {
            int lastSpaceIndex = p6.lastIndexOf(" ");
            if (lastSpaceIndex != -1) {
                linkBeforeOptions.addAll(Arrays.asList(p6.substring(0, lastSpaceIndex).split(" ")));
                linkPrefix = p6.substring(lastSpaceIndex + 1);
            } else {
                linkPrefix = p6;
            }
        }
        if (x.startsWith(" ")) {
            linkAfterOptions.addAll(Arrays.asList(x.trim().split(" ")));
        } else {
            int firstSpaceIndex = x.indexOf(" ");
            if (firstSpaceIndex != -1) {
                linkSuffix = x.substring(0, firstSpaceIndex);
                linkAfterOptions.addAll(Arrays.asList(x.substring(firstSpaceIndex + 1).split(" ")));
            } else {
                linkSuffix = x;
            }
        }
        int i4 = command.size();
        Iterator<String> it3 = classProperties.get("platform.link").iterator();
        while (it3.hasNext()) {
            String p7 = p6;
            String x2 = x;
            String[] libnameversion = it3.next().split("#")[0].split("@");
            List<String> allOptions3 = allOptions2;
            if (libnameversion.length == 3 && libnameversion[1].length() == 0) {
                StringBuilder sb2 = new StringBuilder();
                it = it3;
                sb2.append(libnameversion[0]);
                sb2.append(libnameversion[2]);
                s = sb2.toString();
            } else {
                it = it3;
                s = libnameversion[0];
            }
            List<String> l = new ArrayList<>();
            l.addAll(linkBeforeOptions);
            l.add(linkPrefix + s + linkSuffix);
            l.addAll(linkAfterOptions);
            command.addAll(i4, l);
            p6 = p7;
            x = x2;
            allOptions2 = allOptions3;
            it3 = it;
        }
        String p8 = classProperties.getProperty("platform.frameworkpath.prefix", "");
        for (String s8 : classProperties.get("platform.frameworkpath")) {
            if (new File(s8).isDirectory()) {
                if (p8.endsWith(" ")) {
                    command.add(p8.trim());
                    command.add(s8);
                } else {
                    command.add(p8 + s8);
                }
            }
        }
        String p9 = classProperties.getProperty("platform.framework.prefix", "");
        String x3 = classProperties.getProperty("platform.framework.suffix", "");
        for (String s9 : classProperties.get("platform.framework")) {
            if (p9.endsWith(" ") && x3.startsWith(" ")) {
                command.add(p9.trim());
                command.add(s9);
                command.add(x3.trim());
            } else if (p9.endsWith(" ")) {
                command.add(p9.trim());
                command.add(s9 + x3);
            } else if (x3.startsWith(" ")) {
                command.add(p9 + s9);
                command.add(x3.trim());
            } else {
                command.add(p9 + s9 + x3);
            }
        }
        boolean windows = platform.startsWith("windows");
        int i5 = 0;
        while (true) {
            int i6 = i5;
            if (i6 < command.size()) {
                String arg = command.get(i6);
                if (arg == null) {
                    arg = "";
                }
                if (arg.trim().isEmpty() && windows) {
                    arg = "\"\"";
                }
                command.set(i6, arg);
                i5 = i6 + 1;
            } else {
                return executeCommand(command, workingDirectory2, this.environmentVariables);
            }
        }
    }

    /* access modifiers changed from: package-private */
    /* JADX WARNING: Removed duplicated region for block: B:173:0x054f  */
    /* JADX WARNING: Removed duplicated region for block: B:184:0x05b3  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public java.io.File[] generateAndCompile(java.lang.Class[] r55, java.lang.String r56, boolean r57, boolean r58) throws java.io.IOException, java.lang.InterruptedException {
        /*
            r54 = this;
            r1 = r54
            r2 = r55
            r3 = r56
            java.io.File r0 = r1.outputDirectory
            if (r0 == 0) goto L_0x0011
            java.io.File r0 = r1.outputDirectory
            java.io.File r0 = r0.getCanonicalFile()
            goto L_0x0012
        L_0x0011:
            r0 = 0
        L_0x0012:
            r6 = r0
            java.util.Properties r0 = r1.properties
            r7 = 1
            org.bytedeco.javacpp.ClassProperties r8 = org.bytedeco.javacpp.Loader.loadProperties((java.lang.Class[]) r2, (java.util.Properties) r0, (boolean) r7)
            java.util.Properties r0 = r1.properties
            java.lang.String r9 = "platform"
            java.lang.String r9 = r0.getProperty(r9)
            java.util.Properties r0 = r1.properties
            java.lang.String r10 = "platform.extension"
            java.lang.String r10 = r0.getProperty(r10)
            if (r6 == 0) goto L_0x0042
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r11 = r6.getPath()
            r0.append(r11)
            java.lang.String r11 = java.io.File.separator
            r0.append(r11)
            java.lang.String r0 = r0.toString()
            goto L_0x0044
        L_0x0042:
            java.lang.String r0 = ""
        L_0x0044:
            r11 = r0
            java.lang.String r0 = "platform.source.suffix"
            java.lang.String r12 = ".cpp"
            java.lang.String r12 = r8.getProperty(r0, r12)
            java.lang.String r0 = "platform.library.path"
            java.lang.String r13 = ""
            java.lang.String r13 = r8.getProperty(r0, r13)
            java.lang.String r0 = "platform.library.prefix"
            java.lang.String r14 = ""
            java.lang.String r14 = r8.getProperty(r0, r14)
            java.lang.String r0 = "platform.library.suffix"
            java.lang.String r15 = ""
            java.lang.String r15 = r8.getProperty(r0, r15)
            r0 = 2
            java.lang.String[] r5 = new java.lang.String[r0]
            r0 = 0
            r5[r0] = r11
            r5[r7] = r11
            if (r6 != 0) goto L_0x02dd
            r16 = 0
            r18 = r16
            java.lang.StringBuilder r7 = new java.lang.StringBuilder     // Catch:{ URISyntaxException -> 0x02cc, IllegalArgumentException -> 0x02a8 }
            r7.<init>()     // Catch:{ URISyntaxException -> 0x02cc, IllegalArgumentException -> 0x02a8 }
            r0 = 47
            r7.append(r0)     // Catch:{ URISyntaxException -> 0x02cc, IllegalArgumentException -> 0x02a8 }
            r19 = 0
            r20 = r2[r19]     // Catch:{ URISyntaxException -> 0x02cc, IllegalArgumentException -> 0x02a8 }
            java.lang.String r0 = r20.getName()     // Catch:{ URISyntaxException -> 0x02cc, IllegalArgumentException -> 0x02a8 }
            r21 = r5
            r5 = 46
            r22 = r6
            r6 = 47
            java.lang.String r0 = r0.replace(r5, r6)     // Catch:{ URISyntaxException -> 0x02a0, IllegalArgumentException -> 0x0296 }
            r7.append(r0)     // Catch:{ URISyntaxException -> 0x02a0, IllegalArgumentException -> 0x0296 }
            java.lang.String r0 = ".class"
            r7.append(r0)     // Catch:{ URISyntaxException -> 0x02a0, IllegalArgumentException -> 0x0296 }
            java.lang.String r0 = r7.toString()     // Catch:{ URISyntaxException -> 0x02a0, IllegalArgumentException -> 0x0296 }
            r6 = 0
            r7 = r2[r6]     // Catch:{ URISyntaxException -> 0x02a0, IllegalArgumentException -> 0x0296 }
            java.net.URL r6 = org.bytedeco.javacpp.Loader.findResource(r7, r0)     // Catch:{ URISyntaxException -> 0x02a0, IllegalArgumentException -> 0x0296 }
            java.lang.String r6 = r6.toString()     // Catch:{ URISyntaxException -> 0x02a0, IllegalArgumentException -> 0x0296 }
            r7 = 47
            int r20 = r6.lastIndexOf(r7)     // Catch:{ URISyntaxException -> 0x02a0, IllegalArgumentException -> 0x0296 }
            r7 = 1
            int r5 = r20 + 1
            r7 = 0
            java.lang.String r5 = r6.substring(r7, r5)     // Catch:{ URISyntaxException -> 0x02a0, IllegalArgumentException -> 0x0296 }
            r7 = r5
            r5 = 1
        L_0x00b8:
            r23 = r11
            int r11 = r2.length     // Catch:{ URISyntaxException -> 0x0290, IllegalArgumentException -> 0x0288 }
            if (r5 >= r11) goto L_0x0152
            java.lang.StringBuilder r11 = new java.lang.StringBuilder     // Catch:{ URISyntaxException -> 0x0290, IllegalArgumentException -> 0x0288 }
            r11.<init>()     // Catch:{ URISyntaxException -> 0x0290, IllegalArgumentException -> 0x0288 }
            r24 = r8
            r8 = 47
            r11.append(r8)     // Catch:{ URISyntaxException -> 0x014d, IllegalArgumentException -> 0x0146 }
            r20 = r2[r5]     // Catch:{ URISyntaxException -> 0x014d, IllegalArgumentException -> 0x0146 }
            java.lang.String r8 = r20.getName()     // Catch:{ URISyntaxException -> 0x014d, IllegalArgumentException -> 0x0146 }
            r25 = r15
            r4 = 47
            r15 = 46
            java.lang.String r8 = r8.replace(r15, r4)     // Catch:{ URISyntaxException -> 0x0286, IllegalArgumentException -> 0x0282 }
            r11.append(r8)     // Catch:{ URISyntaxException -> 0x0286, IllegalArgumentException -> 0x0282 }
            java.lang.String r4 = ".class"
            r11.append(r4)     // Catch:{ URISyntaxException -> 0x0286, IllegalArgumentException -> 0x0282 }
            java.lang.String r4 = r11.toString()     // Catch:{ URISyntaxException -> 0x0286, IllegalArgumentException -> 0x0282 }
            r8 = r2[r5]     // Catch:{ URISyntaxException -> 0x0286, IllegalArgumentException -> 0x0282 }
            java.net.URL r8 = org.bytedeco.javacpp.Loader.findResource(r8, r4)     // Catch:{ URISyntaxException -> 0x0286, IllegalArgumentException -> 0x0282 }
            java.lang.String r8 = r8.toString()     // Catch:{ URISyntaxException -> 0x0286, IllegalArgumentException -> 0x0282 }
            r11 = 47
            int r20 = r8.lastIndexOf(r11)     // Catch:{ URISyntaxException -> 0x0286, IllegalArgumentException -> 0x0282 }
            r11 = 1
            int r15 = r20 + 1
            r11 = 0
            java.lang.String r15 = r8.substring(r11, r15)     // Catch:{ URISyntaxException -> 0x0286, IllegalArgumentException -> 0x0282 }
            r11 = r15
            int r15 = r11.length()     // Catch:{ URISyntaxException -> 0x0286, IllegalArgumentException -> 0x0282 }
            r27 = r4
            int r4 = r7.length()     // Catch:{ URISyntaxException -> 0x0286, IllegalArgumentException -> 0x0282 }
            if (r15 <= r4) goto L_0x010c
            r4 = r11
            goto L_0x010d
        L_0x010c:
            r4 = r7
        L_0x010d:
            int r15 = r11.length()     // Catch:{ URISyntaxException -> 0x0286, IllegalArgumentException -> 0x0282 }
            r28 = r8
            int r8 = r7.length()     // Catch:{ URISyntaxException -> 0x0286, IllegalArgumentException -> 0x0282 }
            if (r15 >= r8) goto L_0x011b
            r8 = r11
            goto L_0x011c
        L_0x011b:
            r8 = r7
        L_0x011c:
            boolean r15 = r4.startsWith(r8)     // Catch:{ URISyntaxException -> 0x0286, IllegalArgumentException -> 0x0282 }
            if (r15 != 0) goto L_0x0139
            r15 = 47
            int r20 = r8.lastIndexOf(r15)     // Catch:{ URISyntaxException -> 0x0286, IllegalArgumentException -> 0x0282 }
            if (r20 <= 0) goto L_0x0139
            r29 = r4
            int r4 = r8.lastIndexOf(r15)     // Catch:{ URISyntaxException -> 0x0286, IllegalArgumentException -> 0x0282 }
            r15 = 0
            java.lang.String r4 = r8.substring(r15, r4)     // Catch:{ URISyntaxException -> 0x0286, IllegalArgumentException -> 0x0282 }
            r8 = r4
            r4 = r29
            goto L_0x011c
        L_0x0139:
            r29 = r4
            r7 = r8
            int r5 = r5 + 1
            r11 = r23
            r8 = r24
            r15 = r25
            goto L_0x00b8
        L_0x0146:
            r0 = move-exception
            r25 = r15
            r11 = r18
            goto L_0x02b5
        L_0x014d:
            r0 = move-exception
            r25 = r15
            goto L_0x02d7
        L_0x0152:
            r24 = r8
            r25 = r15
            java.net.URI r4 = new java.net.URI     // Catch:{ URISyntaxException -> 0x0286, IllegalArgumentException -> 0x0282 }
            r4.<init>(r7)     // Catch:{ URISyntaxException -> 0x0286, IllegalArgumentException -> 0x0282 }
            java.lang.String r5 = "file"
            java.lang.String r8 = r4.getScheme()     // Catch:{ URISyntaxException -> 0x027c, IllegalArgumentException -> 0x0276 }
            boolean r5 = r5.equals(r8)     // Catch:{ URISyntaxException -> 0x027c, IllegalArgumentException -> 0x0276 }
            java.io.File r8 = new java.io.File     // Catch:{ URISyntaxException -> 0x027c, IllegalArgumentException -> 0x0276 }
            org.bytedeco.javacpp.tools.ClassScanner r11 = r1.classScanner     // Catch:{ URISyntaxException -> 0x027c, IllegalArgumentException -> 0x0276 }
            org.bytedeco.javacpp.tools.UserClassLoader r11 = r11.getClassLoader()     // Catch:{ URISyntaxException -> 0x027c, IllegalArgumentException -> 0x0276 }
            java.lang.String[] r11 = r11.getPaths()     // Catch:{ URISyntaxException -> 0x027c, IllegalArgumentException -> 0x0276 }
            r15 = 0
            r11 = r11[r15]     // Catch:{ URISyntaxException -> 0x027c, IllegalArgumentException -> 0x0276 }
            r8.<init>(r11)     // Catch:{ URISyntaxException -> 0x027c, IllegalArgumentException -> 0x0276 }
            java.io.File r8 = r8.getCanonicalFile()     // Catch:{ URISyntaxException -> 0x027c, IllegalArgumentException -> 0x0276 }
            if (r5 == 0) goto L_0x018e
            java.io.File r11 = new java.io.File     // Catch:{ URISyntaxException -> 0x0189, IllegalArgumentException -> 0x0185 }
            r11.<init>(r4)     // Catch:{ URISyntaxException -> 0x0189, IllegalArgumentException -> 0x0185 }
            r30 = r4
            goto L_0x01a4
        L_0x0185:
            r0 = move-exception
            r11 = r4
            goto L_0x02b5
        L_0x0189:
            r0 = move-exception
            r18 = r4
            goto L_0x02d7
        L_0x018e:
            java.io.File r11 = new java.io.File     // Catch:{ URISyntaxException -> 0x027c, IllegalArgumentException -> 0x0276 }
            r15 = 47
            int r15 = r0.lastIndexOf(r15)     // Catch:{ URISyntaxException -> 0x027c, IllegalArgumentException -> 0x0276 }
            r18 = 1
            int r15 = r15 + 1
            r30 = r4
            r4 = 0
            java.lang.String r15 = r0.substring(r4, r15)     // Catch:{ URISyntaxException -> 0x0271, IllegalArgumentException -> 0x026d }
            r11.<init>(r8, r15)     // Catch:{ URISyntaxException -> 0x0271, IllegalArgumentException -> 0x026d }
        L_0x01a4:
            r4 = r11
            java.net.URI r11 = new java.net.URI     // Catch:{ URISyntaxException -> 0x0271, IllegalArgumentException -> 0x026d }
            int r15 = r6.length()     // Catch:{ URISyntaxException -> 0x0271, IllegalArgumentException -> 0x026d }
            int r18 = r0.length()     // Catch:{ URISyntaxException -> 0x0271, IllegalArgumentException -> 0x026d }
            int r15 = r15 - r18
            r18 = 1
            int r15 = r15 + 1
            r31 = r0
            r0 = 0
            java.lang.String r15 = r6.substring(r0, r15)     // Catch:{ URISyntaxException -> 0x0271, IllegalArgumentException -> 0x026d }
            r11.<init>(r15)     // Catch:{ URISyntaxException -> 0x0271, IllegalArgumentException -> 0x026d }
            int r0 = r13.length()     // Catch:{ URISyntaxException -> 0x0268, IllegalArgumentException -> 0x0266 }
            if (r0 <= 0) goto L_0x01d3
            if (r5 == 0) goto L_0x01cf
            java.io.File r0 = new java.io.File     // Catch:{ URISyntaxException -> 0x0268, IllegalArgumentException -> 0x0266 }
            r0.<init>(r11)     // Catch:{ URISyntaxException -> 0x0268, IllegalArgumentException -> 0x0266 }
            r32 = r5
            goto L_0x01f3
        L_0x01cf:
            r32 = r5
            r0 = r8
            goto L_0x01f3
        L_0x01d3:
            java.io.File r0 = new java.io.File     // Catch:{ URISyntaxException -> 0x0268, IllegalArgumentException -> 0x0266 }
            java.lang.StringBuilder r15 = new java.lang.StringBuilder     // Catch:{ URISyntaxException -> 0x0268, IllegalArgumentException -> 0x0266 }
            r15.<init>()     // Catch:{ URISyntaxException -> 0x0268, IllegalArgumentException -> 0x0266 }
            r15.append(r9)     // Catch:{ URISyntaxException -> 0x0268, IllegalArgumentException -> 0x0266 }
            if (r10 == 0) goto L_0x01e3
            r32 = r5
            r5 = r10
            goto L_0x01e9
        L_0x01e3:
            java.lang.String r18 = ""
            r32 = r5
            r5 = r18
        L_0x01e9:
            r15.append(r5)     // Catch:{ URISyntaxException -> 0x0268, IllegalArgumentException -> 0x0266 }
            java.lang.String r5 = r15.toString()     // Catch:{ URISyntaxException -> 0x0268, IllegalArgumentException -> 0x0266 }
            r0.<init>(r4, r5)     // Catch:{ URISyntaxException -> 0x0268, IllegalArgumentException -> 0x0266 }
        L_0x01f3:
            java.io.File r5 = new java.io.File     // Catch:{ URISyntaxException -> 0x0268, IllegalArgumentException -> 0x0266 }
            r5.<init>(r0, r13)     // Catch:{ URISyntaxException -> 0x0268, IllegalArgumentException -> 0x0266 }
            java.lang.StringBuilder r15 = new java.lang.StringBuilder     // Catch:{ URISyntaxException -> 0x025d, IllegalArgumentException -> 0x0256 }
            r15.<init>()     // Catch:{ URISyntaxException -> 0x025d, IllegalArgumentException -> 0x0256 }
            r33 = r0
            java.lang.String r0 = r4.getPath()     // Catch:{ URISyntaxException -> 0x025d, IllegalArgumentException -> 0x0256 }
            r15.append(r0)     // Catch:{ URISyntaxException -> 0x025d, IllegalArgumentException -> 0x0256 }
            java.lang.String r0 = java.io.File.separator     // Catch:{ URISyntaxException -> 0x025d, IllegalArgumentException -> 0x0256 }
            r15.append(r0)     // Catch:{ URISyntaxException -> 0x025d, IllegalArgumentException -> 0x0256 }
            java.lang.String r0 = r15.toString()     // Catch:{ URISyntaxException -> 0x025d, IllegalArgumentException -> 0x0256 }
            r15 = r0
            r34 = r4
            r0 = 2
            java.lang.String[] r4 = new java.lang.String[r0]     // Catch:{ URISyntaxException -> 0x024e, IllegalArgumentException -> 0x0248 }
            java.lang.StringBuilder r0 = new java.lang.StringBuilder     // Catch:{ URISyntaxException -> 0x024e, IllegalArgumentException -> 0x0248 }
            r0.<init>()     // Catch:{ URISyntaxException -> 0x024e, IllegalArgumentException -> 0x0248 }
            r35 = r5
            java.lang.String r5 = r8.getPath()     // Catch:{ URISyntaxException -> 0x023f, IllegalArgumentException -> 0x0238 }
            r0.append(r5)     // Catch:{ URISyntaxException -> 0x023f, IllegalArgumentException -> 0x0238 }
            java.lang.String r5 = java.io.File.separator     // Catch:{ URISyntaxException -> 0x023f, IllegalArgumentException -> 0x0238 }
            r0.append(r5)     // Catch:{ URISyntaxException -> 0x023f, IllegalArgumentException -> 0x0238 }
            java.lang.String r0 = r0.toString()     // Catch:{ URISyntaxException -> 0x023f, IllegalArgumentException -> 0x0238 }
            r5 = 0
            r4[r5] = r0     // Catch:{ URISyntaxException -> 0x023f, IllegalArgumentException -> 0x0238 }
            r0 = 1
            r4[r0] = r15     // Catch:{ URISyntaxException -> 0x023f, IllegalArgumentException -> 0x0238 }
            r5 = r4
            r0 = r35
            goto L_0x02eb
        L_0x0238:
            r0 = move-exception
            r23 = r15
            r22 = r35
            goto L_0x02b5
        L_0x023f:
            r0 = move-exception
            r18 = r11
            r23 = r15
            r22 = r35
            goto L_0x02d7
        L_0x0248:
            r0 = move-exception
            r35 = r5
            r23 = r15
            goto L_0x0259
        L_0x024e:
            r0 = move-exception
            r35 = r5
            r18 = r11
            r23 = r15
            goto L_0x0262
        L_0x0256:
            r0 = move-exception
            r35 = r5
        L_0x0259:
            r22 = r35
            goto L_0x02b5
        L_0x025d:
            r0 = move-exception
            r35 = r5
            r18 = r11
        L_0x0262:
            r22 = r35
            goto L_0x02d7
        L_0x0266:
            r0 = move-exception
            goto L_0x02b5
        L_0x0268:
            r0 = move-exception
            r18 = r11
            goto L_0x02d7
        L_0x026d:
            r0 = move-exception
            r11 = r30
            goto L_0x02b5
        L_0x0271:
            r0 = move-exception
            r18 = r30
            goto L_0x02d7
        L_0x0276:
            r0 = move-exception
            r30 = r4
            r11 = r30
            goto L_0x02b5
        L_0x027c:
            r0 = move-exception
            r30 = r4
            r18 = r30
            goto L_0x02d7
        L_0x0282:
            r0 = move-exception
            r11 = r18
            goto L_0x02b5
        L_0x0286:
            r0 = move-exception
            goto L_0x02d7
        L_0x0288:
            r0 = move-exception
            r24 = r8
            r25 = r15
            r11 = r18
            goto L_0x02b5
        L_0x0290:
            r0 = move-exception
            r24 = r8
            r25 = r15
            goto L_0x02d7
        L_0x0296:
            r0 = move-exception
            r24 = r8
            r23 = r11
            r25 = r15
            r11 = r18
            goto L_0x02b5
        L_0x02a0:
            r0 = move-exception
            r24 = r8
            r23 = r11
            r25 = r15
            goto L_0x02d7
        L_0x02a8:
            r0 = move-exception
            r21 = r5
            r22 = r6
            r24 = r8
            r23 = r11
            r25 = r15
            r11 = r18
        L_0x02b5:
            java.lang.RuntimeException r4 = new java.lang.RuntimeException
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "URI: "
            r5.append(r6)
            r5.append(r11)
            java.lang.String r5 = r5.toString()
            r4.<init>(r5, r0)
            throw r4
        L_0x02cc:
            r0 = move-exception
            r21 = r5
            r22 = r6
            r24 = r8
            r23 = r11
            r25 = r15
        L_0x02d7:
            java.lang.RuntimeException r4 = new java.lang.RuntimeException
            r4.<init>(r0)
            throw r4
        L_0x02dd:
            r21 = r5
            r22 = r6
            r24 = r8
            r23 = r11
            r25 = r15
            r0 = r22
            r15 = r23
        L_0x02eb:
            boolean r4 = r0.exists()
            if (r4 != 0) goto L_0x02f4
            r0.mkdirs()
        L_0x02f4:
            org.bytedeco.javacpp.tools.Generator r4 = new org.bytedeco.javacpp.tools.Generator
            org.bytedeco.javacpp.tools.Logger r6 = r1.logger
            java.util.Properties r7 = r1.properties
            java.lang.String r8 = r1.encoding
            r4.<init>(r6, r7, r8)
            r26 = r4
            r4 = 2
            java.lang.String[] r6 = new java.lang.String[r4]
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            r7 = 0
            r8 = r5[r7]
            r4.append(r8)
            java.lang.String r8 = "jnijavacpp"
            r4.append(r8)
            r4.append(r12)
            java.lang.String r4 = r4.toString()
            r6[r7] = r4
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            r7 = 1
            r8 = r5[r7]
            r4.append(r8)
            r4.append(r3)
            r4.append(r12)
            java.lang.String r4 = r4.toString()
            r6[r7] = r4
            r4 = r6
            r6 = 2
            java.lang.String[] r8 = new java.lang.String[r6]
            r6 = 0
            r11 = 0
            r8[r6] = r11
            boolean r6 = r1.header
            if (r6 == 0) goto L_0x0357
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            r11 = r5[r7]
            r6.append(r11)
            r6.append(r3)
            java.lang.String r11 = ".h"
            r6.append(r11)
            java.lang.String r6 = r6.toString()
            goto L_0x0358
        L_0x0357:
            r6 = 0
        L_0x0358:
            r8[r7] = r6
            r6 = r8
            r8 = 2
            java.lang.String[] r11 = new java.lang.String[r8]
            java.lang.String r17 = "_jnijavacpp"
            r18 = 0
            r11[r18] = r17
            r16 = 0
            r11[r7] = r16
            java.lang.String[] r7 = new java.lang.String[r8]
            r7[r18] = r16
            java.lang.String r8 = "_jnijavacpp"
            r18 = 1
            r7[r18] = r8
            java.lang.String r8 = "java.class.path"
            java.lang.String r8 = java.lang.System.getProperty(r8)
            r36 = r5
            org.bytedeco.javacpp.tools.ClassScanner r5 = r1.classScanner
            org.bytedeco.javacpp.tools.UserClassLoader r5 = r5.getClassLoader()
            java.lang.String[] r5 = r5.getPaths()
            r37 = r8
            int r8 = r5.length
            r38 = r9
            r39 = r10
            r10 = r37
            r9 = 0
        L_0x038e:
            if (r9 >= r8) goto L_0x03b5
            r40 = r8
            r8 = r5[r9]
            r41 = r5
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            r5.append(r10)
            r42 = r12
            java.lang.String r12 = java.io.File.pathSeparator
            r5.append(r12)
            r5.append(r8)
            java.lang.String r10 = r5.toString()
            int r9 = r9 + 1
            r8 = r40
            r5 = r41
            r12 = r42
            goto L_0x038e
        L_0x03b5:
            r42 = r12
            r5 = 2
            java.lang.String[] r8 = new java.lang.String[r5]
            r9 = 0
            r12 = 0
            r8[r9] = r12
            r16 = 1
            r8[r16] = r10
            r43 = r10
            java.lang.Class[][] r10 = new java.lang.Class[r5][]
            r10[r9] = r12
            r10[r16] = r2
            r9 = r10
            java.lang.String[] r5 = new java.lang.String[r5]
            java.lang.StringBuilder r10 = new java.lang.StringBuilder
            r10.<init>()
            r10.append(r14)
            java.lang.String r12 = "jnijavacpp"
            r10.append(r12)
            r12 = r25
            r10.append(r12)
            java.lang.String r10 = r10.toString()
            r16 = 0
            r5[r16] = r10
            java.lang.StringBuilder r10 = new java.lang.StringBuilder
            r10.<init>()
            r10.append(r14)
            r10.append(r3)
            r10.append(r12)
            java.lang.String r10 = r10.toString()
            r16 = 1
            r5[r16] = r10
            r10 = 0
            r16 = 1
            r17 = 0
        L_0x0402:
            r44 = r17
            int r2 = r4.length
            r3 = r44
            if (r3 >= r2) goto L_0x046d
            if (r3 != 0) goto L_0x0415
            if (r57 != 0) goto L_0x0415
            r45 = r10
            r46 = r12
            r47 = r13
            goto L_0x0460
        L_0x0415:
            r45 = r10
            org.bytedeco.javacpp.tools.Logger r10 = r1.logger
            r46 = r12
            java.lang.StringBuilder r12 = new java.lang.StringBuilder
            r12.<init>()
            r47 = r13
            java.lang.String r13 = "Generating "
            r12.append(r13)
            r13 = r4[r3]
            r12.append(r13)
            java.lang.String r12 = r12.toString()
            r10.info(r12)
            r27 = r4[r3]
            r28 = r6[r3]
            r29 = r11[r3]
            r30 = r7[r3]
            r31 = r8[r3]
            r32 = r9[r3]
            boolean r10 = r26.generate(r27, r28, r29, r30, r31, r32)
            if (r10 != 0) goto L_0x0460
            org.bytedeco.javacpp.tools.Logger r10 = r1.logger
            java.lang.StringBuilder r12 = new java.lang.StringBuilder
            r12.<init>()
            java.lang.String r13 = "Nothing generated for "
            r12.append(r13)
            r13 = r4[r3]
            r12.append(r13)
            java.lang.String r12 = r12.toString()
            r10.info(r12)
            r16 = 0
            goto L_0x0473
        L_0x0460:
            int r17 = r3 + 1
            r10 = r45
            r12 = r46
            r13 = r47
            r2 = r55
            r3 = r56
            goto L_0x0402
        L_0x046d:
            r45 = r10
            r46 = r12
            r47 = r13
        L_0x0473:
            if (r16 == 0) goto L_0x05f0
            boolean r3 = r1.compile
            if (r3 == 0) goto L_0x05ce
            r3 = 0
            java.util.Properties r10 = r1.properties
            java.lang.String r12 = "platform.library.static"
            java.lang.String r13 = "false"
            java.lang.String r10 = r10.getProperty(r12, r13)
            java.lang.String r10 = r10.toLowerCase()
            java.lang.String r12 = "true"
            boolean r12 = r10.equals(r12)
            if (r12 != 0) goto L_0x04eb
            java.lang.String r12 = "t"
            boolean r12 = r10.equals(r12)
            if (r12 != 0) goto L_0x04eb
            java.lang.String r12 = ""
            boolean r12 = r10.equals(r12)
            if (r12 == 0) goto L_0x04a7
            r48 = r3
            r49 = r6
            r3 = r24
            goto L_0x04f1
        L_0x04a7:
            int r12 = r5.length
            r13 = 1
            int r12 = r12 - r13
            r12 = r5[r12]
            org.bytedeco.javacpp.tools.Logger r13 = r1.logger
            r48 = r3
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            r49 = r6
            java.lang.String r6 = "Compiling "
            r3.append(r6)
            java.lang.String r6 = r0.getPath()
            r3.append(r6)
            java.lang.String r6 = java.io.File.separator
            r3.append(r6)
            r3.append(r12)
            java.lang.String r3 = r3.toString()
            r13.info(r3)
            r3 = r24
            int r6 = r1.compile(r4, r12, r3, r0)
            r50 = r6
            r13 = 1
            java.io.File[] r6 = new java.io.File[r13]
            java.io.File r13 = new java.io.File
            r13.<init>(r0, r12)
            r17 = 0
            r6[r17] = r13
            r51 = r7
            r2 = r50
            goto L_0x054d
        L_0x04eb:
            r48 = r3
            r49 = r6
            r3 = r24
        L_0x04f1:
            int r6 = r4.length
            java.io.File[] r6 = new java.io.File[r6]
            r12 = 0
        L_0x04f5:
            if (r48 != 0) goto L_0x0549
            int r13 = r4.length
            if (r12 >= r13) goto L_0x0549
            if (r12 != 0) goto L_0x0504
            if (r57 != 0) goto L_0x0504
            r51 = r7
            r19 = 0
            goto L_0x0544
        L_0x0504:
            org.bytedeco.javacpp.tools.Logger r13 = r1.logger
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            r51 = r7
            java.lang.String r7 = "Compiling "
            r2.append(r7)
            java.lang.String r7 = r0.getPath()
            r2.append(r7)
            java.lang.String r7 = java.io.File.separator
            r2.append(r7)
            r7 = r5[r12]
            r2.append(r7)
            java.lang.String r2 = r2.toString()
            r13.info(r2)
            r2 = 1
            java.lang.String[] r7 = new java.lang.String[r2]
            r2 = r4[r12]
            r19 = 0
            r7[r19] = r2
            r2 = r5[r12]
            int r2 = r1.compile(r7, r2, r3, r0)
            java.io.File r7 = new java.io.File
            r13 = r5[r12]
            r7.<init>(r0, r13)
            r6[r12] = r7
            r48 = r2
        L_0x0544:
            int r12 = r12 + 1
            r7 = r51
            goto L_0x04f5
        L_0x0549:
            r51 = r7
            r2 = r48
        L_0x054d:
            if (r2 != 0) goto L_0x05b3
            int r7 = r4.length
            r12 = 1
            int r7 = r7 - r12
        L_0x0552:
            if (r7 < 0) goto L_0x05ac
            if (r7 != 0) goto L_0x055e
            if (r58 != 0) goto L_0x055e
            r52 = r0
            r53 = r3
            goto L_0x05a5
        L_0x055e:
            boolean r13 = r1.deleteJniFiles
            if (r13 == 0) goto L_0x0589
            org.bytedeco.javacpp.tools.Logger r13 = r1.logger
            r52 = r0
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r53 = r3
            java.lang.String r3 = "Deleting "
            r0.append(r3)
            r3 = r4[r7]
            r0.append(r3)
            java.lang.String r0 = r0.toString()
            r13.info(r0)
            java.io.File r0 = new java.io.File
            r3 = r4[r7]
            r0.<init>(r3)
            r0.delete()
            goto L_0x05a5
        L_0x0589:
            r52 = r0
            r53 = r3
            org.bytedeco.javacpp.tools.Logger r0 = r1.logger
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r13 = "Keeping "
            r3.append(r13)
            r13 = r4[r7]
            r3.append(r13)
            java.lang.String r3 = r3.toString()
            r0.info(r3)
        L_0x05a5:
            int r7 = r7 + -1
            r0 = r52
            r3 = r53
            goto L_0x0552
        L_0x05ac:
            r52 = r0
            r53 = r3
            r45 = r6
            goto L_0x05f8
        L_0x05b3:
            r52 = r0
            r53 = r3
            java.lang.RuntimeException r0 = new java.lang.RuntimeException
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "Process exited with an error: "
            r3.append(r7)
            r3.append(r2)
            java.lang.String r3 = r3.toString()
            r0.<init>(r3)
            throw r0
        L_0x05ce:
            r52 = r0
            r49 = r6
            r51 = r7
            r53 = r24
            r19 = 0
            int r0 = r4.length
            java.io.File[] r10 = new java.io.File[r0]
        L_0x05dc:
            r0 = r19
            int r2 = r4.length
            if (r0 >= r2) goto L_0x05ed
            java.io.File r2 = new java.io.File
            r3 = r4[r0]
            r2.<init>(r3)
            r10[r0] = r2
            int r19 = r0 + 1
            goto L_0x05dc
        L_0x05ed:
            r45 = r10
            goto L_0x05f8
        L_0x05f0:
            r52 = r0
            r49 = r6
            r51 = r7
            r53 = r24
        L_0x05f8:
            return r45
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.Builder.generateAndCompile(java.lang.Class[], java.lang.String, boolean, boolean):java.io.File[]");
    }

    /* access modifiers changed from: package-private */
    public void createJar(File jarFile, String[] classPath, File... files) throws IOException {
        this.logger.info("Creating " + jarFile);
        JarOutputStream jos = new JarOutputStream(new FileOutputStream(jarFile));
        for (File f : files) {
            String name = f.getPath();
            if (classPath != null) {
                String[] names = new String[classPath.length];
                for (int i = 0; i < classPath.length; i++) {
                    String path = new File(classPath[i]).getCanonicalPath();
                    if (name.startsWith(path)) {
                        names[i] = name.substring(path.length() + 1);
                    }
                }
                String name2 = name;
                for (int i2 = 0; i2 < names.length; i2++) {
                    if (names[i2] != null && names[i2].length() < name2.length()) {
                        name2 = names[i2];
                    }
                }
                name = name2;
            }
            ZipEntry e = new ZipEntry(name.replace(File.separatorChar, IOUtils.DIR_SEPARATOR_UNIX));
            e.setTime(f.lastModified());
            jos.putNextEntry(e);
            FileInputStream fis = new FileInputStream(f);
            byte[] buffer = new byte[65536];
            while (true) {
                int read = fis.read(buffer);
                int length = read;
                if (read == -1) {
                    break;
                }
                jos.write(buffer, 0, length);
            }
            fis.close();
            jos.closeEntry();
        }
        jos.close();
    }

    public Builder() {
        this(Logger.create(Builder.class));
    }

    public Builder(Logger logger2) {
        this.encoding = null;
        this.outputDirectory = null;
        this.outputName = null;
        this.jarPrefix = null;
        this.generate = true;
        this.compile = true;
        this.deleteJniFiles = true;
        this.header = false;
        this.copyLibs = false;
        this.copyResources = false;
        this.properties = null;
        this.classScanner = null;
        this.buildCommand = null;
        this.workingDirectory = null;
        this.environmentVariables = null;
        this.compilerOptions = null;
        this.logger = logger2;
        System.setProperty("org.bytedeco.javacpp.loadlibraries", HttpState.PREEMPTIVE_DEFAULT);
        this.properties = Loader.loadProperties();
        this.classScanner = new ClassScanner(logger2, new ArrayList(), new UserClassLoader(Thread.currentThread().getContextClassLoader()));
        this.compilerOptions = new ArrayList();
    }

    public Builder classPaths(String classPaths) {
        classPaths(classPaths == null ? null : classPaths.split(File.pathSeparator));
        return this;
    }

    public Builder classPaths(String... classPaths) {
        this.classScanner.getClassLoader().addPaths(classPaths);
        return this;
    }

    public Builder encoding(String encoding2) {
        this.encoding = encoding2;
        return this;
    }

    public Builder outputDirectory(String outputDirectory2) {
        outputDirectory(outputDirectory2 == null ? null : new File(outputDirectory2));
        return this;
    }

    public Builder outputDirectory(File outputDirectory2) {
        this.outputDirectory = outputDirectory2;
        return this;
    }

    public Builder generate(boolean generate2) {
        this.generate = generate2;
        return this;
    }

    public Builder compile(boolean compile2) {
        this.compile = compile2;
        return this;
    }

    public Builder deleteJniFiles(boolean deleteJniFiles2) {
        this.deleteJniFiles = deleteJniFiles2;
        return this;
    }

    public Builder header(boolean header2) {
        this.header = header2;
        return this;
    }

    public Builder copyLibs(boolean copyLibs2) {
        this.copyLibs = copyLibs2;
        return this;
    }

    public Builder copyResources(boolean copyResources2) {
        this.copyResources = copyResources2;
        return this;
    }

    public Builder outputName(String outputName2) {
        this.outputName = outputName2;
        return this;
    }

    public Builder jarPrefix(String jarPrefix2) {
        this.jarPrefix = jarPrefix2;
        return this;
    }

    public Builder properties(String platform) {
        if (platform != null) {
            this.properties = Loader.loadProperties(platform, (String) null);
        }
        return this;
    }

    public Builder properties(Properties properties2) {
        if (properties2 != null) {
            for (Map.Entry e : properties2.entrySet()) {
                property((String) e.getKey(), (String) e.getValue());
            }
        }
        return this;
    }

    public Builder propertyFile(String filename) throws IOException {
        propertyFile(filename == null ? null : new File(filename));
        return this;
    }

    public Builder propertyFile(File propertyFile) throws IOException {
        if (propertyFile == null) {
            return this;
        }
        FileInputStream fis = new FileInputStream(propertyFile);
        this.properties = new Properties();
        try {
            this.properties.load(new InputStreamReader(fis));
        } catch (NoSuchMethodError e) {
            this.properties.load(fis);
        }
        fis.close();
        return this;
    }

    public Builder property(String keyValue) {
        int equalIndex = keyValue.indexOf(61);
        if (equalIndex < 0) {
            equalIndex = keyValue.indexOf(58);
        }
        property(keyValue.substring(2, equalIndex), keyValue.substring(equalIndex + 1));
        return this;
    }

    public Builder property(String key, String value) {
        if (key.length() > 0 && value.length() > 0) {
            this.properties.put(key, value);
        }
        return this;
    }

    public Builder classesOrPackages(String... classesOrPackages) throws IOException, ClassNotFoundException, NoClassDefFoundError {
        if (classesOrPackages == null) {
            this.classScanner.addPackage((String) null, true);
        } else {
            for (String s : classesOrPackages) {
                this.classScanner.addClassOrPackage(s);
            }
        }
        return this;
    }

    public Builder buildCommand(String[] buildCommand2) {
        this.buildCommand = buildCommand2;
        return this;
    }

    public Builder workingDirectory(String workingDirectory2) {
        workingDirectory(workingDirectory2 == null ? null : new File(workingDirectory2));
        return this;
    }

    public Builder workingDirectory(File workingDirectory2) {
        this.workingDirectory = workingDirectory2;
        return this;
    }

    public Builder environmentVariables(Map<String, String> environmentVariables2) {
        this.environmentVariables = environmentVariables2;
        return this;
    }

    public Builder compilerOptions(String... options) {
        if (options != null) {
            this.compilerOptions.addAll(Arrays.asList(options));
        }
        return this;
    }

    /* JADX WARNING: Removed duplicated region for block: B:168:0x0416  */
    /* JADX WARNING: Removed duplicated region for block: B:173:0x0460  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public java.io.File[] build() throws java.io.IOException, java.lang.InterruptedException, org.bytedeco.javacpp.tools.ParserException {
        /*
            r43 = this;
            r1 = r43
            java.lang.String[] r0 = r1.buildCommand
            r3 = 1
            if (r0 == 0) goto L_0x01dc
            java.lang.String[] r0 = r1.buildCommand
            int r0 = r0.length
            if (r0 <= 0) goto L_0x01dc
            java.lang.String[] r0 = r1.buildCommand
            java.util.List r0 = java.util.Arrays.asList(r0)
            java.util.Properties r5 = r1.properties
            java.lang.String r6 = "platform.buildpath"
            java.lang.String r7 = ""
            java.lang.String r5 = r5.getProperty(r6, r7)
            java.util.Properties r6 = r1.properties
            java.lang.String r7 = "platform.linkresource"
            java.lang.String r8 = ""
            java.lang.String r6 = r6.getProperty(r7, r8)
            java.util.Properties r7 = r1.properties
            java.lang.String r8 = "platform.buildresource"
            java.lang.String r9 = ""
            java.lang.String r7 = r7.getProperty(r8, r9)
            java.util.Properties r8 = r1.properties
            java.lang.String r9 = "platform.path.separator"
            java.lang.String r8 = r8.getProperty(r9)
            int r9 = r5.length()
            if (r9 > 0) goto L_0x004b
            int r9 = r7.length()
            if (r9 <= 0) goto L_0x0045
            goto L_0x004b
        L_0x0045:
            r20 = r6
            r21 = r7
            goto L_0x01b9
        L_0x004b:
            java.util.ArrayList r9 = new java.util.ArrayList
            r9.<init>()
            r10 = 0
            org.bytedeco.javacpp.tools.ClassScanner r11 = r1.classScanner
            java.util.Collection r11 = r11.getClasses()
            java.util.Iterator r11 = r11.iterator()
        L_0x005b:
            boolean r12 = r11.hasNext()
            if (r12 == 0) goto L_0x00a4
            java.lang.Object r12 = r11.next()
            java.lang.Class r12 = (java.lang.Class) r12
            java.lang.Class r13 = org.bytedeco.javacpp.Loader.getEnclosingClass(r12)
            if (r13 == r12) goto L_0x006e
            goto L_0x005b
        L_0x006e:
            java.util.Properties r13 = r1.properties
            org.bytedeco.javacpp.ClassProperties r10 = org.bytedeco.javacpp.Loader.loadProperties((java.lang.Class) r12, (java.util.Properties) r13, (boolean) r3)
            boolean r13 = r10.isLoaded()
            if (r13 != 0) goto L_0x0091
            org.bytedeco.javacpp.tools.Logger r13 = r1.logger
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            java.lang.String r15 = "Could not load platform properties for "
            r14.append(r15)
            r14.append(r12)
            java.lang.String r14 = r14.toString()
            r13.warn(r14)
            goto L_0x005b
        L_0x0091:
            java.lang.String r13 = "platform.preload"
            java.util.List r13 = r10.get(r13)
            r9.addAll(r13)
            java.lang.String r13 = "platform.link"
            java.util.List r13 = r10.get(r13)
            r9.addAll(r13)
            goto L_0x005b
        L_0x00a4:
            if (r10 != 0) goto L_0x00ae
            org.bytedeco.javacpp.ClassProperties r3 = new org.bytedeco.javacpp.ClassProperties
            java.util.Properties r11 = r1.properties
            r3.<init>(r11)
            r10 = r3
        L_0x00ae:
            java.lang.String[] r3 = r7.split(r8)
            int r11 = r3.length
            r12 = r5
            r5 = 0
        L_0x00b5:
            if (r5 >= r11) goto L_0x0193
            r13 = r3[r5]
            java.io.File[] r14 = org.bytedeco.javacpp.Loader.cacheResources(r13)
            int r15 = r14.length
            r16 = r12
            r12 = 0
        L_0x00c1:
            if (r12 >= r15) goto L_0x0184
            r17 = r14[r12]
            java.lang.String r4 = r17.getCanonicalPath()
            int r18 = r16.length()
            if (r18 <= 0) goto L_0x00ee
            r2 = r16
            boolean r16 = r2.endsWith(r8)
            if (r16 != 0) goto L_0x00eb
            r19 = r3
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            r3.append(r2)
            r3.append(r8)
            java.lang.String r16 = r3.toString()
            r2 = r16
            goto L_0x00f2
        L_0x00eb:
            r19 = r3
            goto L_0x00f2
        L_0x00ee:
            r19 = r3
            r2 = r16
        L_0x00f2:
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            r3.append(r2)
            r3.append(r4)
            java.lang.String r16 = r3.toString()
            java.util.ArrayList r2 = new java.util.ArrayList
            r2.<init>()
            java.lang.String[] r3 = r6.split(r8)
            r20 = r6
            int r6 = r3.length
            r21 = r7
            r7 = 0
        L_0x0110:
            if (r7 >= r6) goto L_0x0147
            r18 = r3[r7]
            r22 = r3
            java.io.File[] r3 = org.bytedeco.javacpp.Loader.cacheResources(r18)
            r23 = r6
            int r6 = r3.length
            r24 = r9
            r9 = 0
        L_0x0120:
            if (r9 >= r6) goto L_0x013e
            r25 = r3[r9]
            r26 = r3
            java.lang.String r3 = r25.getCanonicalPath()
            boolean r27 = r3.startsWith(r4)
            if (r27 == 0) goto L_0x0139
            boolean r27 = r3.equals(r4)
            if (r27 != 0) goto L_0x0139
            r2.add(r3)
        L_0x0139:
            int r9 = r9 + 1
            r3 = r26
            goto L_0x0120
        L_0x013e:
            int r7 = r7 + 1
            r3 = r22
            r6 = r23
            r9 = r24
            goto L_0x0110
        L_0x0147:
            r24 = r9
            java.io.File[] r3 = r17.listFiles()
            if (r3 == 0) goto L_0x0178
            int r6 = r3.length
            r7 = 0
        L_0x0151:
            if (r7 >= r6) goto L_0x0178
            r9 = r3[r7]
            r28 = r3
            java.lang.String r3 = r9.getAbsolutePath()
            r29 = r4
            int r4 = r2.size()
            java.lang.String[] r4 = new java.lang.String[r4]
            java.lang.Object[] r4 = r2.toArray(r4)
            java.lang.String[] r4 = (java.lang.String[]) r4
            r30 = r2
            r2 = 0
            org.bytedeco.javacpp.Loader.createLibraryLink(r3, r10, r2, r4)
            int r7 = r7 + 1
            r3 = r28
            r4 = r29
            r2 = r30
            goto L_0x0151
        L_0x0178:
            int r12 = r12 + 1
            r3 = r19
            r6 = r20
            r7 = r21
            r9 = r24
            goto L_0x00c1
        L_0x0184:
            r19 = r3
            r20 = r6
            r21 = r7
            r24 = r9
            r2 = r16
            int r5 = r5 + 1
            r12 = r2
            goto L_0x00b5
        L_0x0193:
            r20 = r6
            r21 = r7
            r24 = r9
            int r2 = r12.length()
            if (r2 <= 0) goto L_0x01b8
            java.util.Map<java.lang.String, java.lang.String> r2 = r1.environmentVariables
            if (r2 != 0) goto L_0x01aa
            java.util.LinkedHashMap r2 = new java.util.LinkedHashMap
            r2.<init>()
            r1.environmentVariables = r2
        L_0x01aa:
            java.util.Map<java.lang.String, java.lang.String> r2 = r1.environmentVariables
            java.lang.String r3 = "BUILD_PATH"
            r2.put(r3, r12)
            java.util.Map<java.lang.String, java.lang.String> r2 = r1.environmentVariables
            java.lang.String r3 = "BUILD_PATH_SEPARATOR"
            r2.put(r3, r8)
        L_0x01b8:
            r5 = r12
        L_0x01b9:
            java.io.File r2 = r1.workingDirectory
            java.util.Map<java.lang.String, java.lang.String> r3 = r1.environmentVariables
            int r2 = r1.executeCommand(r0, r2, r3)
            if (r2 != 0) goto L_0x01c5
            r3 = 0
            return r3
        L_0x01c5:
            java.lang.RuntimeException r3 = new java.lang.RuntimeException
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r6 = "Process exited with an error: "
            r4.append(r6)
            r4.append(r2)
            java.lang.String r4 = r4.toString()
            r3.<init>(r4)
            throw r3
        L_0x01dc:
            org.bytedeco.javacpp.tools.ClassScanner r0 = r1.classScanner
            java.util.Collection r0 = r0.getClasses()
            boolean r0 = r0.isEmpty()
            if (r0 == 0) goto L_0x01ea
            r2 = 0
            return r2
        L_0x01ea:
            java.util.ArrayList r0 = new java.util.ArrayList
            r0.<init>()
            r2 = r0
            java.util.LinkedHashMap r0 = new java.util.LinkedHashMap
            r0.<init>()
            r4 = r0
            org.bytedeco.javacpp.tools.ClassScanner r0 = r1.classScanner
            java.util.Collection r0 = r0.getClasses()
            java.util.Iterator r5 = r0.iterator()
        L_0x0200:
            boolean r0 = r5.hasNext()
            if (r0 == 0) goto L_0x02ef
            java.lang.Object r0 = r5.next()
            r6 = r0
            java.lang.Class r6 = (java.lang.Class) r6
            java.lang.Class r0 = org.bytedeco.javacpp.Loader.getEnclosingClass(r6)
            if (r0 == r6) goto L_0x0214
            goto L_0x0200
        L_0x0214:
            java.util.Properties r0 = r1.properties
            r7 = 0
            org.bytedeco.javacpp.ClassProperties r8 = org.bytedeco.javacpp.Loader.loadProperties((java.lang.Class) r6, (java.util.Properties) r0, (boolean) r7)
            boolean r0 = r8.isLoaded()
            if (r0 == 0) goto L_0x0290
            java.lang.Class[] r0 = r6.getInterfaces()     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x0241 }
            java.util.List r0 = java.util.Arrays.asList(r0)     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x0241 }
            java.lang.Class<org.bytedeco.javacpp.tools.BuildEnabled> r7 = org.bytedeco.javacpp.tools.BuildEnabled.class
            boolean r0 = r0.contains(r7)     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x0241 }
            if (r0 == 0) goto L_0x0240
            java.lang.Object r0 = r6.newInstance()     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x0241 }
            org.bytedeco.javacpp.tools.BuildEnabled r0 = (org.bytedeco.javacpp.tools.BuildEnabled) r0     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x0241 }
            org.bytedeco.javacpp.tools.Logger r7 = r1.logger     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x0241 }
            java.util.Properties r9 = r1.properties     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x0241 }
            java.lang.String r10 = r1.encoding     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x0241 }
            r0.init(r7, r9, r10)     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x0241 }
        L_0x0240:
            goto L_0x0242
        L_0x0241:
            r0 = move-exception
        L_0x0242:
            java.lang.String r0 = "global"
            java.lang.String r0 = r8.getProperty(r0)
            if (r0 == 0) goto L_0x0290
            java.lang.String r7 = r6.getName()
            boolean r7 = r7.equals(r0)
            if (r7 != 0) goto L_0x0290
            r7 = 0
            org.bytedeco.javacpp.tools.ClassScanner r9 = r1.classScanner
            java.util.Collection r9 = r9.getClasses()
            java.util.Iterator r9 = r9.iterator()
        L_0x025f:
            boolean r10 = r9.hasNext()
            if (r10 == 0) goto L_0x0275
            java.lang.Object r10 = r9.next()
            java.lang.Class r10 = (java.lang.Class) r10
            java.lang.String r11 = r10.getName()
            boolean r11 = r11.equals(r0)
            r7 = r7 | r11
            goto L_0x025f
        L_0x0275:
            boolean r9 = r1.generate
            if (r9 == 0) goto L_0x027b
            if (r7 != 0) goto L_0x0200
        L_0x027b:
            org.bytedeco.javacpp.tools.ClassScanner r9 = r1.classScanner
            org.bytedeco.javacpp.tools.UserClassLoader r9 = r9.getClassLoader()
            java.lang.String[] r9 = r9.getPaths()
            java.io.File r9 = r1.parse(r9, r6)
            if (r9 == 0) goto L_0x028e
            r2.add(r9)
        L_0x028e:
            goto L_0x0200
        L_0x0290:
            boolean r0 = r8.isLoaded()
            if (r0 != 0) goto L_0x029c
            java.util.Properties r0 = r1.properties
            org.bytedeco.javacpp.ClassProperties r8 = org.bytedeco.javacpp.Loader.loadProperties((java.lang.Class) r6, (java.util.Properties) r0, (boolean) r3)
        L_0x029c:
            boolean r0 = r8.isLoaded()
            if (r0 != 0) goto L_0x02ba
            org.bytedeco.javacpp.tools.Logger r0 = r1.logger
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r9 = "Could not load platform properties for "
            r7.append(r9)
            r7.append(r6)
            java.lang.String r7 = r7.toString()
            r0.warn(r7)
            goto L_0x0200
        L_0x02ba:
            java.lang.String r0 = r1.outputName
            if (r0 == 0) goto L_0x02c1
            java.lang.String r0 = r1.outputName
            goto L_0x02c9
        L_0x02c1:
            java.lang.String r0 = "platform.library"
            java.lang.String r7 = ""
            java.lang.String r0 = r8.getProperty(r0, r7)
        L_0x02c9:
            boolean r7 = r1.generate
            if (r7 == 0) goto L_0x0200
            int r7 = r0.length()
            if (r7 != 0) goto L_0x02d5
            goto L_0x0200
        L_0x02d5:
            java.lang.Object r7 = r4.get(r0)
            java.util.LinkedHashSet r7 = (java.util.LinkedHashSet) r7
            if (r7 != 0) goto L_0x02e6
            java.util.LinkedHashSet r9 = new java.util.LinkedHashSet
            r9.<init>()
            r7 = r9
            r4.put(r0, r9)
        L_0x02e6:
            java.util.List r9 = r8.getEffectiveClasses()
            r7.addAll(r9)
            goto L_0x0200
        L_0x02ef:
            r0 = 0
            java.util.Set r5 = r4.keySet()
            java.util.Iterator r5 = r5.iterator()
            r6 = r0
        L_0x02f9:
            boolean r0 = r5.hasNext()
            if (r0 == 0) goto L_0x0560
            java.lang.Object r0 = r5.next()
            r7 = r0
            java.lang.String r7 = (java.lang.String) r7
            java.lang.Object r0 = r4.get(r7)
            r8 = r0
            java.util.LinkedHashSet r8 = (java.util.LinkedHashSet) r8
            int r0 = r8.size()
            java.lang.Class[] r0 = new java.lang.Class[r0]
            java.lang.Object[] r0 = r8.toArray(r0)
            r9 = r0
            java.lang.Class[] r9 = (java.lang.Class[]) r9
            if (r6 != 0) goto L_0x031e
            r0 = 1
            goto L_0x031f
        L_0x031e:
            r0 = 0
        L_0x031f:
            int r10 = r4.size()
            int r10 = r10 - r3
            if (r6 != r10) goto L_0x0328
            r10 = 1
            goto L_0x0329
        L_0x0328:
            r10 = 0
        L_0x0329:
            java.io.File[] r10 = r1.generateAndCompile(r9, r7, r0, r10)
            if (r10 == 0) goto L_0x054c
            int r0 = r10.length
            if (r0 <= 0) goto L_0x054c
            int r0 = r10.length
            int r0 = r0 - r3
            r0 = r10[r0]
            java.io.File r11 = r0.getParentFile()
            java.util.List r0 = java.util.Arrays.asList(r10)
            r2.addAll(r0)
            boolean r0 = r1.copyLibs
            if (r0 == 0) goto L_0x04a4
            java.util.Properties r0 = r1.properties
            r12 = 0
            org.bytedeco.javacpp.ClassProperties r13 = org.bytedeco.javacpp.Loader.loadProperties((java.lang.Class[]) r9, (java.util.Properties) r0, (boolean) r12)
            java.util.ArrayList r0 = new java.util.ArrayList
            r0.<init>()
            r12 = r0
            java.lang.String r0 = "platform.preload"
            java.util.List r0 = r13.get(r0)
            r12.addAll(r0)
            java.lang.String r0 = "platform.link"
            java.util.List r0 = r13.get(r0)
            r12.addAll(r0)
            java.util.Properties r0 = r1.properties
            org.bytedeco.javacpp.ClassProperties r14 = org.bytedeco.javacpp.Loader.loadProperties((java.lang.Class[]) r9, (java.util.Properties) r0, (boolean) r3)
            java.util.Iterator r15 = r12.iterator()
        L_0x036e:
            boolean r0 = r15.hasNext()
            if (r0 == 0) goto L_0x04a4
            java.lang.Object r0 = r15.next()
            r3 = r0
            java.lang.String r3 = (java.lang.String) r3
            java.lang.String r0 = r3.trim()
            r31 = r4
            java.lang.String r4 = "#"
            boolean r0 = r0.endsWith(r4)
            if (r0 != 0) goto L_0x049f
            java.lang.String r0 = r3.trim()
            int r0 = r0.length()
            if (r0 != 0) goto L_0x0398
            r4 = r31
        L_0x0396:
            r3 = 1
            goto L_0x036e
        L_0x0398:
            r4 = 0
            java.net.URL[] r0 = org.bytedeco.javacpp.Loader.findLibrary(r4, r13, r3)
            r4 = r0
            java.io.File r0 = new java.io.File     // Catch:{ Exception -> 0x03ce }
            r32 = r5
            java.net.URI r5 = new java.net.URI     // Catch:{ Exception -> 0x03ca }
            r16 = 0
            r17 = r4[r16]     // Catch:{ Exception -> 0x03ca }
            java.net.URI r16 = r17.toURI()     // Catch:{ Exception -> 0x03ca }
            r33 = r4
            java.lang.String r4 = r16.toString()     // Catch:{ Exception -> 0x03c6 }
            r34 = r7
            java.lang.String r7 = "#"
            java.lang.String[] r4 = r4.split(r7)     // Catch:{ Exception -> 0x03c4 }
            r7 = 0
            r4 = r4[r7]     // Catch:{ Exception -> 0x03c4 }
            r5.<init>(r4)     // Catch:{ Exception -> 0x03c4 }
            r0.<init>(r5)     // Catch:{ Exception -> 0x03c4 }
            goto L_0x0401
        L_0x03c4:
            r0 = move-exception
            goto L_0x03d5
        L_0x03c6:
            r0 = move-exception
            r34 = r7
            goto L_0x03d5
        L_0x03ca:
            r0 = move-exception
            r33 = r4
            goto L_0x03d3
        L_0x03ce:
            r0 = move-exception
            r33 = r4
            r32 = r5
        L_0x03d3:
            r34 = r7
        L_0x03d5:
            r4 = r0
            r5 = 0
            java.net.URL[] r7 = org.bytedeco.javacpp.Loader.findLibrary(r5, r14, r3)
            java.io.File r0 = new java.io.File     // Catch:{ Exception -> 0x0473 }
            java.net.URI r5 = new java.net.URI     // Catch:{ Exception -> 0x0473 }
            r16 = 0
            r17 = r7[r16]     // Catch:{ Exception -> 0x0473 }
            java.net.URI r16 = r17.toURI()     // Catch:{ Exception -> 0x0473 }
            r35 = r4
            java.lang.String r4 = r16.toString()     // Catch:{ Exception -> 0x046b }
            r36 = r7
            java.lang.String r7 = "#"
            java.lang.String[] r4 = r4.split(r7)     // Catch:{ Exception -> 0x0465 }
            r7 = 0
            r4 = r4[r7]     // Catch:{ Exception -> 0x0465 }
            r5.<init>(r4)     // Catch:{ Exception -> 0x0465 }
            r0.<init>(r5)     // Catch:{ Exception -> 0x0465 }
            r33 = r36
        L_0x0401:
            java.io.File r4 = new java.io.File
            java.lang.String r5 = r0.getName()
            r4.<init>(r11, r5)
            boolean r5 = r0.exists()
            if (r5 == 0) goto L_0x0460
            boolean r5 = r2.contains(r4)
            if (r5 != 0) goto L_0x0460
            org.bytedeco.javacpp.tools.Logger r5 = r1.logger
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            r37 = r8
            java.lang.String r8 = "Copying "
            r7.append(r8)
            r7.append(r0)
            java.lang.String r7 = r7.toString()
            r5.info(r7)
            java.io.FileInputStream r5 = new java.io.FileInputStream
            r5.<init>(r0)
            java.io.FileOutputStream r7 = new java.io.FileOutputStream
            r7.<init>(r4)
            r8 = 65536(0x10000, float:9.18355E-41)
            byte[] r8 = new byte[r8]
        L_0x043c:
            r38 = r0
            int r0 = r5.read(r8)
            r39 = r0
            r40 = r10
            r10 = -1
            if (r0 == r10) goto L_0x0454
            r0 = r39
            r10 = 0
            r7.write(r8, r10, r0)
            r0 = r38
            r10 = r40
            goto L_0x043c
        L_0x0454:
            r0 = r39
            r7.close()
            r5.close()
            r2.add(r4)
            goto L_0x0464
        L_0x0460:
            r37 = r8
            r40 = r10
        L_0x0464:
            goto L_0x0493
        L_0x0465:
            r0 = move-exception
            r37 = r8
            r40 = r10
            goto L_0x047c
        L_0x046b:
            r0 = move-exception
            r36 = r7
            r37 = r8
            r40 = r10
            goto L_0x047c
        L_0x0473:
            r0 = move-exception
            r35 = r4
            r36 = r7
            r37 = r8
            r40 = r10
        L_0x047c:
            org.bytedeco.javacpp.tools.Logger r4 = r1.logger
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r7 = "Could not find library "
            r5.append(r7)
            r5.append(r3)
            java.lang.String r5 = r5.toString()
            r4.warn(r5)
        L_0x0493:
            r4 = r31
            r5 = r32
            r7 = r34
            r8 = r37
            r10 = r40
            goto L_0x0396
        L_0x049f:
            r4 = r31
            r3 = 1
            goto L_0x036e
        L_0x04a4:
            r31 = r4
            r32 = r5
            r34 = r7
            r37 = r8
            r40 = r10
            boolean r0 = r1.copyResources
            if (r0 == 0) goto L_0x0556
            java.util.Properties r0 = r1.properties
            r3 = 0
            org.bytedeco.javacpp.ClassProperties r0 = org.bytedeco.javacpp.Loader.loadProperties((java.lang.Class[]) r9, (java.util.Properties) r0, (boolean) r3)
            java.lang.String r3 = "platform.resource"
            java.util.List r3 = r0.get(r3)
            java.util.Properties r4 = r1.properties
            r5 = 1
            org.bytedeco.javacpp.ClassProperties r0 = org.bytedeco.javacpp.Loader.loadProperties((java.lang.Class[]) r9, (java.util.Properties) r4, (boolean) r5)
            java.lang.String r4 = "platform.resourcepath"
            java.util.List r4 = r0.get(r4)
            java.nio.file.Path r5 = r11.toPath()
            java.util.Iterator r7 = r3.iterator()
        L_0x04d4:
            boolean r8 = r7.hasNext()
            if (r8 == 0) goto L_0x0556
            java.lang.Object r8 = r7.next()
            java.lang.String r8 = (java.lang.String) r8
            java.nio.file.Path r10 = r5.resolve(r8)
            r12 = 0
            java.nio.file.LinkOption[] r13 = new java.nio.file.LinkOption[r12]
            boolean r13 = java.nio.file.Files.exists(r10, r13)
            if (r13 != 0) goto L_0x04f2
            java.nio.file.attribute.FileAttribute[] r13 = new java.nio.file.attribute.FileAttribute[r12]
            java.nio.file.Files.createDirectories(r10, r13)
        L_0x04f2:
            java.util.Iterator r12 = r4.iterator()
        L_0x04f6:
            boolean r13 = r12.hasNext()
            if (r13 == 0) goto L_0x0547
            java.lang.Object r13 = r12.next()
            java.lang.String r13 = (java.lang.String) r13
            r14 = 1
            java.lang.String[] r15 = new java.lang.String[r14]
            r14 = 0
            r15[r14] = r8
            java.nio.file.Path r15 = java.nio.file.Paths.get(r13, r15)
            r41 = r0
            java.nio.file.LinkOption[] r0 = new java.nio.file.LinkOption[r14]
            boolean r0 = java.nio.file.Files.exists(r15, r0)
            if (r0 == 0) goto L_0x0540
            org.bytedeco.javacpp.tools.Logger r0 = r1.logger
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            r42 = r3
            java.lang.String r3 = "Copying "
            r14.append(r3)
            r14.append(r15)
            java.lang.String r3 = r14.toString()
            r0.info(r3)
            java.nio.file.FileVisitOption r0 = java.nio.file.FileVisitOption.FOLLOW_LINKS
            java.util.EnumSet r0 = java.util.EnumSet.of(r0)
            r3 = 2147483647(0x7fffffff, float:NaN)
            org.bytedeco.javacpp.tools.Builder$2 r14 = new org.bytedeco.javacpp.tools.Builder$2
            r14.<init>(r10, r15)
            java.nio.file.Files.walkFileTree(r15, r0, r3, r14)
            goto L_0x0542
        L_0x0540:
            r42 = r3
        L_0x0542:
            r0 = r41
            r3 = r42
            goto L_0x04f6
        L_0x0547:
            r41 = r0
            r42 = r3
            goto L_0x04d4
        L_0x054c:
            r31 = r4
            r32 = r5
            r34 = r7
            r37 = r8
            r40 = r10
        L_0x0556:
            int r6 = r6 + 1
            r4 = r31
            r5 = r32
            r3 = 1
            goto L_0x02f9
        L_0x0560:
            r31 = r4
            int r0 = r2.size()
            java.io.File[] r0 = new java.io.File[r0]
            java.lang.Object[] r0 = r2.toArray(r0)
            java.io.File[] r0 = (java.io.File[]) r0
            java.lang.String r3 = r1.jarPrefix
            if (r3 == 0) goto L_0x05cc
            int r3 = r0.length
            if (r3 <= 0) goto L_0x05cc
            java.io.File r3 = new java.io.File
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r5 = r1.jarPrefix
            r4.append(r5)
            java.lang.String r5 = "-"
            r4.append(r5)
            java.util.Properties r5 = r1.properties
            java.lang.String r7 = "platform"
            java.lang.String r5 = r5.getProperty(r7)
            r4.append(r5)
            java.util.Properties r5 = r1.properties
            java.lang.String r7 = "platform.extension"
            java.lang.String r8 = ""
            java.lang.String r5 = r5.getProperty(r7, r8)
            r4.append(r5)
            java.lang.String r5 = ".jar"
            r4.append(r5)
            java.lang.String r4 = r4.toString()
            r3.<init>(r4)
            java.io.File r4 = r3.getParentFile()
            if (r4 == 0) goto L_0x05b9
            boolean r5 = r4.exists()
            if (r5 != 0) goto L_0x05b9
            r4.mkdir()
        L_0x05b9:
            java.io.File r5 = r1.outputDirectory
            if (r5 != 0) goto L_0x05c8
            org.bytedeco.javacpp.tools.ClassScanner r5 = r1.classScanner
            org.bytedeco.javacpp.tools.UserClassLoader r5 = r5.getClassLoader()
            java.lang.String[] r5 = r5.getPaths()
            goto L_0x05c9
        L_0x05c8:
            r5 = 0
        L_0x05c9:
            r1.createJar(r3, r5, r0)
        L_0x05cc:
            java.lang.String r3 = "org.bytedeco.javacpp.loadlibraries"
            java.lang.String r4 = "true"
            java.lang.System.setProperty(r3, r4)
            return r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.Builder.build():java.io.File[]");
    }

    public static void printHelp() {
        String version = Builder.class.getPackage().getImplementationVersion();
        if (version == null) {
            version = EnvironmentCompat.MEDIA_UNKNOWN;
        }
        PrintStream printStream = System.out;
        printStream.println("JavaCPP version " + version + "\nCopyright (C) 2011-2018 Samuel Audet <samuel.audet@gmail.com>\nProject site: https://github.com/bytedeco/javacpp");
        System.out.println();
        System.out.println("Usage: java -jar javacpp.jar [options] [class or package (suffixed with .* or .**)] [commands]");
        System.out.println();
        System.out.println("where options include:");
        System.out.println();
        System.out.println("    -classpath <path>      Load user classes from path");
        System.out.println("    -encoding <name>       Character encoding used for input and output files");
        System.out.println("    -d <directory>         Output all generated files to directory");
        System.out.println("    -o <name>              Output everything in a file named after given name");
        System.out.println("    -nogenerate            Do not try to generate C++ source files, only try to parse header files");
        System.out.println("    -nocompile             Do not compile or delete the generated C++ source files");
        System.out.println("    -nodelete              Do not delete generated C++ JNI files after compilation");
        System.out.println("    -header                Generate header file with declarations of callbacks functions");
        System.out.println("    -copylibs              Copy to output directory dependent libraries (link and preload)");
        System.out.println("    -copyresources         Copy to output directory resources listed in properties");
        System.out.println("    -jarprefix <prefix>    Also create a JAR file named \"<prefix>-<platform>.jar\"");
        System.out.println("    -properties <resource> Load all properties from resource");
        System.out.println("    -propertyfile <file>   Load all properties from file");
        System.out.println("    -D<property>=<value>   Set property to value");
        System.out.println("    -Xcompiler <option>    Pass option directly to compiler");
        System.out.println();
        System.out.println("and where optional commands include:");
        System.out.println();
        System.out.println("    -exec [args...]        After build, call java command on the first class");
        System.out.println();
    }

    public static void main(String[] args) throws Exception {
        Builder builder = new Builder();
        String[] execArgs = null;
        boolean addedClasses = false;
        int i = 0;
        while (i < args.length) {
            if ("-help".equals(args[i]) || "--help".equals(args[i])) {
                printHelp();
                System.exit(0);
            } else if ("-classpath".equals(args[i]) || "-cp".equals(args[i]) || "-lib".equals(args[i])) {
                i++;
                builder.classPaths(args[i]);
            } else if ("-encoding".equals(args[i])) {
                i++;
                builder.encoding(args[i]);
            } else if ("-d".equals(args[i])) {
                i++;
                builder.outputDirectory(args[i]);
            } else if ("-o".equals(args[i])) {
                i++;
                builder.outputName(args[i]);
            } else if ("-nocpp".equals(args[i]) || "-nogenerate".equals(args[i])) {
                builder.generate(false);
            } else if ("-cpp".equals(args[i]) || "-nocompile".equals(args[i])) {
                builder.compile(false);
            } else if ("-nodelete".equals(args[i])) {
                builder.deleteJniFiles(false);
            } else if ("-header".equals(args[i])) {
                builder.header(true);
            } else if ("-copylibs".equals(args[i])) {
                builder.copyLibs(true);
            } else if ("-copyresources".equals(args[i])) {
                builder.copyResources(true);
            } else if ("-jarprefix".equals(args[i])) {
                i++;
                builder.jarPrefix(args[i]);
            } else if ("-properties".equals(args[i])) {
                i++;
                builder.properties(args[i]);
            } else if ("-propertyfile".equals(args[i])) {
                i++;
                builder.propertyFile(args[i]);
            } else if (args[i].startsWith("-D")) {
                builder.property(args[i]);
            } else if ("-Xcompiler".equals(args[i])) {
                i++;
                builder.compilerOptions(args[i]);
            } else if ("-exec".equals(args[i])) {
                execArgs = (String[]) Arrays.copyOfRange(args, i + 1, args.length);
                i = args.length;
            } else if (args[i].startsWith("-")) {
                builder.logger.error("Invalid option \"" + args[i] + "\"");
                printHelp();
                System.exit(1);
            } else {
                String arg = args[i];
                if (arg.endsWith(".java")) {
                    ArrayList arrayList = new ArrayList(Arrays.asList(new String[]{"javac", "-cp"}));
                    String paths = System.getProperty("java.class.path");
                    for (String path : builder.classScanner.getClassLoader().getPaths()) {
                        paths = paths + File.pathSeparator + path;
                    }
                    arrayList.add(paths);
                    arrayList.add(arg);
                    int exitValue = builder.executeCommand(arrayList, builder.workingDirectory, builder.environmentVariables);
                    if (exitValue == 0) {
                        arg = arg.replace(File.separatorChar, '.').replace(IOUtils.DIR_SEPARATOR_UNIX, '.').substring(0, arg.length() - 5);
                    } else {
                        throw new RuntimeException("Could not compile " + arg + ": " + exitValue);
                    }
                }
                builder.classesOrPackages(arg);
                addedClasses = true;
            }
            i++;
        }
        if (!addedClasses) {
            builder.classesOrPackages((String[]) null);
        }
        File[] outputFiles = builder.build();
        Collection<Class> classes = builder.classScanner.getClasses();
        if (outputFiles != null && outputFiles.length > 0 && !classes.isEmpty() && execArgs != null) {
            Class c = classes.iterator().next();
            ArrayList arrayList2 = new ArrayList(Arrays.asList(new String[]{"java", "-cp"}));
            String paths2 = System.getProperty("java.class.path");
            for (String path2 : builder.classScanner.getClassLoader().getPaths()) {
                paths2 = paths2 + File.pathSeparator + path2;
            }
            arrayList2.add(paths2);
            arrayList2.add(c.getCanonicalName());
            arrayList2.addAll(Arrays.asList(execArgs));
            System.exit(builder.executeCommand(arrayList2, builder.workingDirectory, builder.environmentVariables));
        }
    }
}
