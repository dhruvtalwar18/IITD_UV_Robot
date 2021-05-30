package org.bytedeco.javacpp.tools;

import android.support.v4.app.NotificationCompat;
import java.io.PrintWriter;
import java.lang.annotation.Annotation;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.nio.Buffer;
import java.nio.ByteBuffer;
import java.nio.CharBuffer;
import java.nio.DoubleBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.nio.LongBuffer;
import java.nio.ShortBuffer;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import java.util.Set;
import org.apache.commons.io.IOUtils;
import org.apache.xmlrpc.serializer.TypeSerializerImpl;
import org.bytedeco.javacpp.BoolPointer;
import org.bytedeco.javacpp.BooleanPointer;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.CLongPointer;
import org.bytedeco.javacpp.CharPointer;
import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.FunctionPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.Loader;
import org.bytedeco.javacpp.LongPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.PointerPointer;
import org.bytedeco.javacpp.ShortPointer;
import org.bytedeco.javacpp.SizeTPointer;
import org.bytedeco.javacpp.annotation.Adapter;
import org.bytedeco.javacpp.annotation.Allocator;
import org.bytedeco.javacpp.annotation.ArrayAllocator;
import org.bytedeco.javacpp.annotation.ByPtr;
import org.bytedeco.javacpp.annotation.ByPtrPtr;
import org.bytedeco.javacpp.annotation.ByPtrRef;
import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Convention;
import org.bytedeco.javacpp.annotation.CriticalRegion;
import org.bytedeco.javacpp.annotation.Function;
import org.bytedeco.javacpp.annotation.MemberGetter;
import org.bytedeco.javacpp.annotation.MemberSetter;
import org.bytedeco.javacpp.annotation.Name;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.NoDeallocator;
import org.bytedeco.javacpp.annotation.NoException;
import org.bytedeco.javacpp.annotation.Opaque;
import org.bytedeco.javacpp.annotation.ValueGetter;
import org.bytedeco.javacpp.annotation.ValueSetter;
import org.bytedeco.javacpp.annotation.Virtual;

public class Generator {
    static final String JNI_VERSION = "JNI_VERSION_1_6";
    static final List<Class> baseClasses = Arrays.asList(new Class[]{Loader.class, Pointer.class, BytePointer.class, ShortPointer.class, IntPointer.class, LongPointer.class, FloatPointer.class, DoublePointer.class, CharPointer.class, BooleanPointer.class, PointerPointer.class, BoolPointer.class, CLongPointer.class, SizeTPointer.class});
    boolean accessesEnums;
    Map<Method, MethodInformation> annotationCache;
    IndexedSet<Class> arrayDeallocators;
    IndexedSet<String> callbacks;
    IndexedSet<Class> deallocators;
    final String encoding;
    IndexedSet<Class> functions;
    IndexedSet<Class> jclasses;
    final Logger logger;
    boolean mayThrowExceptions;
    Map<Class, Set<String>> members;
    PrintWriter out;
    PrintWriter out2;
    boolean passesStrings;
    final Properties properties;
    boolean usesAdapters;
    Map<Class, Set<String>> virtualFunctions;
    Map<Class, Set<String>> virtualMembers;

    enum ByteEnum {
        BYTE;
        
        byte value;
    }

    enum IntEnum {
        INT;
        
        int value;
    }

    enum LongEnum {
        LONG;
        
        long value;
    }

    enum ShortEnum {
        SHORT;
        
        short value;
    }

    public Generator(Logger logger2, Properties properties2) {
        this(logger2, properties2, (String) null);
    }

    public Generator(Logger logger2, Properties properties2, String encoding2) {
        this.logger = logger2;
        this.properties = properties2;
        this.encoding = encoding2;
    }

    /* JADX WARNING: Removed duplicated region for block: B:52:0x012b  */
    /* JADX WARNING: Removed duplicated region for block: B:55:0x0134  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public boolean generate(java.lang.String r15, java.lang.String r16, java.lang.String r17, java.lang.String r18, java.lang.String r19, java.lang.Class<?>... r20) throws java.io.IOException {
        /*
            r14 = this;
            r10 = r14
            r11 = r16
            java.io.PrintWriter r0 = new java.io.PrintWriter     // Catch:{ all -> 0x0125 }
            org.bytedeco.javacpp.tools.Generator$1 r1 = new org.bytedeco.javacpp.tools.Generator$1     // Catch:{ all -> 0x0125 }
            r1.<init>()     // Catch:{ all -> 0x0125 }
            r0.<init>(r1)     // Catch:{ all -> 0x0125 }
            r10.out = r0     // Catch:{ all -> 0x0125 }
            r0 = 0
            r10.out2 = r0     // Catch:{ all -> 0x0125 }
            org.bytedeco.javacpp.tools.IndexedSet r0 = new org.bytedeco.javacpp.tools.IndexedSet     // Catch:{ all -> 0x0125 }
            r0.<init>()     // Catch:{ all -> 0x0125 }
            r10.callbacks = r0     // Catch:{ all -> 0x0125 }
            org.bytedeco.javacpp.tools.IndexedSet r0 = new org.bytedeco.javacpp.tools.IndexedSet     // Catch:{ all -> 0x0125 }
            r0.<init>()     // Catch:{ all -> 0x0125 }
            r10.functions = r0     // Catch:{ all -> 0x0125 }
            org.bytedeco.javacpp.tools.IndexedSet r0 = new org.bytedeco.javacpp.tools.IndexedSet     // Catch:{ all -> 0x0125 }
            r0.<init>()     // Catch:{ all -> 0x0125 }
            r10.deallocators = r0     // Catch:{ all -> 0x0125 }
            org.bytedeco.javacpp.tools.IndexedSet r0 = new org.bytedeco.javacpp.tools.IndexedSet     // Catch:{ all -> 0x0125 }
            r0.<init>()     // Catch:{ all -> 0x0125 }
            r10.arrayDeallocators = r0     // Catch:{ all -> 0x0125 }
            org.bytedeco.javacpp.tools.IndexedSet r0 = new org.bytedeco.javacpp.tools.IndexedSet     // Catch:{ all -> 0x0125 }
            r0.<init>()     // Catch:{ all -> 0x0125 }
            r10.jclasses = r0     // Catch:{ all -> 0x0125 }
            java.util.HashMap r0 = new java.util.HashMap     // Catch:{ all -> 0x0125 }
            r0.<init>()     // Catch:{ all -> 0x0125 }
            r10.members = r0     // Catch:{ all -> 0x0125 }
            java.util.HashMap r0 = new java.util.HashMap     // Catch:{ all -> 0x0125 }
            r0.<init>()     // Catch:{ all -> 0x0125 }
            r10.virtualFunctions = r0     // Catch:{ all -> 0x0125 }
            java.util.HashMap r0 = new java.util.HashMap     // Catch:{ all -> 0x0125 }
            r0.<init>()     // Catch:{ all -> 0x0125 }
            r10.virtualMembers = r0     // Catch:{ all -> 0x0125 }
            java.util.HashMap r0 = new java.util.HashMap     // Catch:{ all -> 0x0125 }
            r0.<init>()     // Catch:{ all -> 0x0125 }
            r10.annotationCache = r0     // Catch:{ all -> 0x0125 }
            r0 = 0
            r10.mayThrowExceptions = r0     // Catch:{ all -> 0x0125 }
            r10.usesAdapters = r0     // Catch:{ all -> 0x0125 }
            r10.passesStrings = r0     // Catch:{ all -> 0x0125 }
            if (r18 == 0) goto L_0x0060
            boolean r1 = r18.isEmpty()     // Catch:{ all -> 0x0125 }
            if (r1 == 0) goto L_0x0078
        L_0x0060:
            java.util.List<java.lang.Class> r1 = baseClasses     // Catch:{ all -> 0x0125 }
            java.util.Iterator r1 = r1.iterator()     // Catch:{ all -> 0x0125 }
        L_0x0066:
            boolean r2 = r1.hasNext()     // Catch:{ all -> 0x0125 }
            if (r2 == 0) goto L_0x0078
            java.lang.Object r2 = r1.next()     // Catch:{ all -> 0x0125 }
            java.lang.Class r2 = (java.lang.Class) r2     // Catch:{ all -> 0x0125 }
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r3 = r10.jclasses     // Catch:{ all -> 0x0125 }
            r3.index(r2)     // Catch:{ all -> 0x0125 }
            goto L_0x0066
        L_0x0078:
            r2 = 1
            r3 = 1
            r4 = 1
            r5 = 1
            r1 = r14
            r6 = r17
            r7 = r18
            r8 = r19
            r9 = r20
            boolean r1 = r1.classes(r2, r3, r4, r5, r6, r7, r8, r9)     // Catch:{ all -> 0x0125 }
            if (r1 == 0) goto L_0x0111
            java.io.File r0 = new java.io.File     // Catch:{ all -> 0x0125 }
            r12 = r15
            r0.<init>(r15)     // Catch:{ all -> 0x010f }
            java.io.File r1 = r0.getParentFile()     // Catch:{ all -> 0x010f }
            r13 = r1
            if (r13 == 0) goto L_0x009b
            r13.mkdirs()     // Catch:{ all -> 0x010f }
        L_0x009b:
            java.lang.String r1 = r10.encoding     // Catch:{ all -> 0x010f }
            if (r1 == 0) goto L_0x00a7
            java.io.PrintWriter r1 = new java.io.PrintWriter     // Catch:{ all -> 0x010f }
            java.lang.String r2 = r10.encoding     // Catch:{ all -> 0x010f }
            r1.<init>(r0, r2)     // Catch:{ all -> 0x010f }
            goto L_0x00ac
        L_0x00a7:
            java.io.PrintWriter r1 = new java.io.PrintWriter     // Catch:{ all -> 0x010f }
            r1.<init>(r0)     // Catch:{ all -> 0x010f }
        L_0x00ac:
            r10.out = r1     // Catch:{ all -> 0x010f }
            if (r11 == 0) goto L_0x00e7
            org.bytedeco.javacpp.tools.Logger r1 = r10.logger     // Catch:{ all -> 0x010f }
            java.lang.StringBuilder r2 = new java.lang.StringBuilder     // Catch:{ all -> 0x010f }
            r2.<init>()     // Catch:{ all -> 0x010f }
            java.lang.String r3 = "Generating "
            r2.append(r3)     // Catch:{ all -> 0x010f }
            r2.append(r11)     // Catch:{ all -> 0x010f }
            java.lang.String r2 = r2.toString()     // Catch:{ all -> 0x010f }
            r1.info(r2)     // Catch:{ all -> 0x010f }
            java.io.File r1 = new java.io.File     // Catch:{ all -> 0x010f }
            r1.<init>(r11)     // Catch:{ all -> 0x010f }
            java.io.File r2 = r1.getParentFile()     // Catch:{ all -> 0x010f }
            if (r2 == 0) goto L_0x00d4
            r2.mkdirs()     // Catch:{ all -> 0x010f }
        L_0x00d4:
            java.lang.String r3 = r10.encoding     // Catch:{ all -> 0x010f }
            if (r3 == 0) goto L_0x00e0
            java.io.PrintWriter r3 = new java.io.PrintWriter     // Catch:{ all -> 0x010f }
            java.lang.String r4 = r10.encoding     // Catch:{ all -> 0x010f }
            r3.<init>(r1, r4)     // Catch:{ all -> 0x010f }
            goto L_0x00e5
        L_0x00e0:
            java.io.PrintWriter r3 = new java.io.PrintWriter     // Catch:{ all -> 0x010f }
            r3.<init>(r1)     // Catch:{ all -> 0x010f }
        L_0x00e5:
            r10.out2 = r3     // Catch:{ all -> 0x010f }
        L_0x00e7:
            boolean r2 = r10.mayThrowExceptions     // Catch:{ all -> 0x010f }
            boolean r3 = r10.usesAdapters     // Catch:{ all -> 0x010f }
            boolean r4 = r10.passesStrings     // Catch:{ all -> 0x010f }
            boolean r5 = r10.accessesEnums     // Catch:{ all -> 0x010f }
            r1 = r14
            r6 = r17
            r7 = r18
            r8 = r19
            r9 = r20
            boolean r1 = r1.classes(r2, r3, r4, r5, r6, r7, r8, r9)     // Catch:{ all -> 0x010f }
            java.io.PrintWriter r2 = r10.out
            if (r2 == 0) goto L_0x0105
            java.io.PrintWriter r2 = r10.out
            r2.close()
        L_0x0105:
            java.io.PrintWriter r2 = r10.out2
            if (r2 == 0) goto L_0x010e
            java.io.PrintWriter r2 = r10.out2
            r2.close()
        L_0x010e:
            return r1
        L_0x010f:
            r0 = move-exception
            goto L_0x0127
        L_0x0111:
            r12 = r15
            java.io.PrintWriter r1 = r10.out
            if (r1 == 0) goto L_0x011b
            java.io.PrintWriter r1 = r10.out
            r1.close()
        L_0x011b:
            java.io.PrintWriter r1 = r10.out2
            if (r1 == 0) goto L_0x0124
            java.io.PrintWriter r1 = r10.out2
            r1.close()
        L_0x0124:
            return r0
        L_0x0125:
            r0 = move-exception
            r12 = r15
        L_0x0127:
            java.io.PrintWriter r1 = r10.out
            if (r1 == 0) goto L_0x0130
            java.io.PrintWriter r1 = r10.out
            r1.close()
        L_0x0130:
            java.io.PrintWriter r1 = r10.out2
            if (r1 == 0) goto L_0x0139
            java.io.PrintWriter r1 = r10.out2
            r1.close()
        L_0x0139:
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.Generator.generate(java.lang.String, java.lang.String, java.lang.String, java.lang.String, java.lang.String, java.lang.Class[]):boolean");
    }

    /* access modifiers changed from: package-private */
    /* JADX WARNING: Removed duplicated region for block: B:219:0x213d  */
    /* JADX WARNING: Removed duplicated region for block: B:319:0x206f A[SYNTHETIC] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public boolean classes(boolean r36, boolean r37, boolean r38, boolean r39, java.lang.String r40, java.lang.String r41, java.lang.String r42, java.lang.Class<?>... r43) {
        /*
            r35 = this;
            r1 = r35
            r5 = r41
            r6 = r43
            java.lang.Class<org.bytedeco.javacpp.tools.Generator> r0 = org.bytedeco.javacpp.tools.Generator.class
            java.lang.Package r0 = r0.getPackage()
            java.lang.String r0 = r0.getImplementationVersion()
            if (r0 != 0) goto L_0x0014
            java.lang.String r0 = "unknown"
        L_0x0014:
            r7 = r0
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r8 = "// Generated by JavaCPP version "
            r0.append(r8)
            r0.append(r7)
            java.lang.String r8 = ": DO NOT EDIT THIS FILE"
            r0.append(r8)
            java.lang.String r8 = r0.toString()
            java.io.PrintWriter r0 = r1.out
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out2
            if (r0 == 0) goto L_0x0043
            java.io.PrintWriter r0 = r1.out2
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out2
            r0.println()
        L_0x0043:
            java.util.Properties r0 = r1.properties
            r9 = 1
            org.bytedeco.javacpp.ClassProperties r10 = org.bytedeco.javacpp.Loader.loadProperties((java.lang.Class[]) r6, (java.util.Properties) r0, (boolean) r9)
            java.lang.String r0 = "platform.pragma"
            java.util.List r0 = r10.get(r0)
            java.util.Iterator r0 = r0.iterator()
        L_0x0054:
            boolean r11 = r0.hasNext()
            if (r11 == 0) goto L_0x0077
            java.lang.Object r11 = r0.next()
            java.lang.String r11 = (java.lang.String) r11
            java.io.PrintWriter r12 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "#pragma "
            r13.append(r14)
            r13.append(r11)
            java.lang.String r13 = r13.toString()
            r12.println(r13)
            goto L_0x0054
        L_0x0077:
            java.lang.String r0 = "platform.define"
            java.util.List r0 = r10.get(r0)
            java.util.Iterator r0 = r0.iterator()
        L_0x0081:
            boolean r11 = r0.hasNext()
            if (r11 == 0) goto L_0x00a4
            java.lang.Object r11 = r0.next()
            java.lang.String r11 = (java.lang.String) r11
            java.io.PrintWriter r12 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "#define "
            r13.append(r14)
            r13.append(r11)
            java.lang.String r13 = r13.toString()
            r12.println(r13)
            goto L_0x0081
        L_0x00a4:
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#ifdef _WIN32"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #define _JAVASOFT_JNI_MD_H_"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #define JNIEXPORT __declspec(dllexport)"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #define JNIIMPORT __declspec(dllimport)"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #define JNICALL __stdcall"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    typedef int jint;"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    typedef long long jlong;"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    typedef signed char jbyte;"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#elif defined(__GNUC__) && !defined(__ANDROID__)"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #define _JAVASOFT_JNI_MD_H_"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #define JNIEXPORT __attribute__((visibility(\"default\")))"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #define JNIIMPORT"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #define JNICALL"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    typedef int jint;"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    typedef long long jlong;"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    typedef signed char jbyte;"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#endif"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#include <jni.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out2
            if (r0 == 0) goto L_0x014b
            java.io.PrintWriter r0 = r1.out2
            java.lang.String r11 = "#include <jni.h>"
            r0.println(r11)
        L_0x014b:
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#ifdef __ANDROID__"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #include <android/log.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #include <pthread.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#elif defined(__APPLE__) && defined(__OBJC__)"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #include <TargetConditionals.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #include <Foundation/Foundation.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#endif"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#ifdef __linux__"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #include <malloc.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #include <sys/types.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #include <sys/stat.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #include <sys/sysinfo.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #include <fcntl.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #include <unistd.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #include <dlfcn.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #include <link.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#elif defined(__APPLE__)"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #include <sys/types.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #include <sys/sysctl.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #include <mach/mach_init.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #include <mach/mach_host.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #include <mach/task.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #include <unistd.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #include <dlfcn.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #include <mach-o/dyld.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#elif defined(_WIN32)"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #define NOMINMAX"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #include <windows.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #include <psapi.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#endif"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#if defined(__ANDROID__) || TARGET_OS_IPHONE"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #define NewWeakGlobalRef(obj) NewGlobalRef(obj)"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #define DeleteWeakGlobalRef(obj) DeleteGlobalRef(obj)"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#endif"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#include <limits.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#include <stddef.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#ifndef _WIN32"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #include <stdint.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#endif"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#include <stdio.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#include <stdlib.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#include <string.h>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#include <exception>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#include <memory>"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#include <new>"
            r0.println(r11)
            if (r5 == 0) goto L_0x02a2
            boolean r0 = r41.isEmpty()
            if (r0 == 0) goto L_0x0333
        L_0x02a2:
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#if defined(NATIVE_ALLOCATOR) && defined(NATIVE_DEALLOCATOR)"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    void* operator new(std::size_t size, const std::nothrow_t&) throw() {"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "        return NATIVE_ALLOCATOR(size);"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    }"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    void* operator new[](std::size_t size, const std::nothrow_t&) throw() {"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "        return NATIVE_ALLOCATOR(size);"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    }"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    void* operator new(std::size_t size) throw(std::bad_alloc) {"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "        return NATIVE_ALLOCATOR(size);"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    }"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    void* operator new[](std::size_t size) throw(std::bad_alloc) {"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "        return NATIVE_ALLOCATOR(size);"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    }"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    void operator delete(void* ptr) throw() {"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "        NATIVE_DEALLOCATOR(ptr);"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    }"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    void operator delete[](void* ptr) throw() {"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "        NATIVE_DEALLOCATOR(ptr);"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    }"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#endif"
            r0.println(r11)
        L_0x0333:
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#define jlong_to_ptr(a) ((void*)(uintptr_t)(a))"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#define ptr_to_jlong(a) ((jlong)(uintptr_t)(a))"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#if defined(_MSC_VER)"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #define JavaCPP_noinline __declspec(noinline)"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #define JavaCPP_hidden /* hidden by default */"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#elif defined(__GNUC__)"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #define JavaCPP_noinline __attribute__((noinline)) __attribute__ ((unused))"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #define JavaCPP_hidden   __attribute__((visibility(\"hidden\"))) __attribute__ ((unused))"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#else"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #define JavaCPP_noinline"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "    #define JavaCPP_hidden"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r11 = "#endif"
            r0.println(r11)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            if (r40 != 0) goto L_0x03d7
            java.lang.String r0 = ""
            java.lang.String r11 = "platform.library.static"
            java.lang.String r12 = "false"
            java.lang.String r11 = r10.getProperty(r11, r12)
            java.lang.String r11 = r11.toLowerCase()
            java.lang.String r12 = "true"
            boolean r12 = r11.equals(r12)
            if (r12 != 0) goto L_0x03be
            java.lang.String r12 = "t"
            boolean r12 = r11.equals(r12)
            if (r12 != 0) goto L_0x03be
            java.lang.String r12 = ""
            boolean r12 = r11.equals(r12)
            if (r12 == 0) goto L_0x03d5
        L_0x03be:
            java.lang.StringBuilder r12 = new java.lang.StringBuilder
            r12.<init>()
            java.lang.String r13 = "_"
            r12.append(r13)
            java.lang.String r13 = "platform.library"
            java.lang.String r13 = r10.getProperty(r13)
            r12.append(r13)
            java.lang.String r0 = r12.toString()
        L_0x03d5:
            r11 = r0
            goto L_0x03d9
        L_0x03d7:
            r11 = r40
        L_0x03d9:
            r0 = 0
            if (r6 == 0) goto L_0x04df
            java.lang.String r12 = "platform.exclude"
            java.util.List r12 = r10.get(r12)
            r13 = 2
            java.util.List[] r13 = new java.util.List[r13]
            java.lang.String r14 = "platform.include"
            java.util.List r14 = r10.get(r14)
            r13[r0] = r14
            java.lang.String r14 = "platform.cinclude"
            java.util.List r14 = r10.get(r14)
            r13[r9] = r14
            r14 = 0
        L_0x03f6:
            int r15 = r13.length
            if (r14 >= r15) goto L_0x04df
            r15 = r13[r14]
            if (r15 == 0) goto L_0x04d5
            r15 = r13[r14]
            int r15 = r15.size()
            if (r15 <= 0) goto L_0x04d5
            if (r14 != r9) goto L_0x0427
            java.io.PrintWriter r15 = r1.out
            java.lang.String r0 = "extern \"C\" {"
            r15.println(r0)
            java.io.PrintWriter r0 = r1.out2
            if (r0 == 0) goto L_0x0427
            java.io.PrintWriter r0 = r1.out2
            java.lang.String r15 = "#ifdef __cplusplus"
            r0.println(r15)
            java.io.PrintWriter r0 = r1.out2
            java.lang.String r15 = "extern \"C\" {"
            r0.println(r15)
            java.io.PrintWriter r0 = r1.out2
            java.lang.String r15 = "#endif"
            r0.println(r15)
        L_0x0427:
            r0 = r13[r14]
            java.util.Iterator r0 = r0.iterator()
        L_0x042d:
            boolean r15 = r0.hasNext()
            if (r15 == 0) goto L_0x04aa
            java.lang.Object r15 = r0.next()
            java.lang.String r15 = (java.lang.String) r15
            boolean r17 = r12.contains(r15)
            if (r17 == 0) goto L_0x0440
            goto L_0x042d
        L_0x0440:
            java.lang.String r9 = "#include "
            r18 = r0
            java.lang.String r0 = "<"
            boolean r0 = r15.startsWith(r0)
            r19 = r7
            r7 = 34
            if (r0 != 0) goto L_0x0467
            java.lang.String r0 = "\""
            boolean r0 = r15.startsWith(r0)
            if (r0 != 0) goto L_0x0467
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r9)
            r0.append(r7)
            java.lang.String r9 = r0.toString()
        L_0x0467:
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r9)
            r0.append(r15)
            java.lang.String r0 = r0.toString()
            java.lang.String r9 = ">"
            boolean r9 = r15.endsWith(r9)
            if (r9 != 0) goto L_0x0495
            java.lang.String r9 = "\""
            boolean r9 = r15.endsWith(r9)
            if (r9 != 0) goto L_0x0495
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            r9.append(r0)
            r9.append(r7)
            java.lang.String r0 = r9.toString()
        L_0x0495:
            java.io.PrintWriter r7 = r1.out
            r7.println(r0)
            java.io.PrintWriter r7 = r1.out2
            if (r7 == 0) goto L_0x04a3
            java.io.PrintWriter r7 = r1.out2
            r7.println(r0)
        L_0x04a3:
            r0 = r18
            r7 = r19
            r9 = 1
            goto L_0x042d
        L_0x04aa:
            r19 = r7
            r0 = 1
            if (r14 != r0) goto L_0x04cf
            java.io.PrintWriter r0 = r1.out
            java.lang.String r7 = "}"
            r0.println(r7)
            java.io.PrintWriter r0 = r1.out2
            if (r0 == 0) goto L_0x04cf
            java.io.PrintWriter r0 = r1.out2
            java.lang.String r7 = "#ifdef __cplusplus"
            r0.println(r7)
            java.io.PrintWriter r0 = r1.out2
            java.lang.String r7 = "}"
            r0.println(r7)
            java.io.PrintWriter r0 = r1.out2
            java.lang.String r7 = "#endif"
            r0.println(r7)
        L_0x04cf:
            java.io.PrintWriter r0 = r1.out
            r0.println()
            goto L_0x04d7
        L_0x04d5:
            r19 = r7
        L_0x04d7:
            int r14 = r14 + 1
            r7 = r19
            r0 = 0
            r9 = 1
            goto L_0x03f6
        L_0x04df:
            r19 = r7
            java.io.PrintWriter r0 = r1.out
            java.lang.String r7 = "static JavaVM* JavaCPP_vm = NULL;"
            r0.println(r7)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r7 = "static bool JavaCPP_haveAllocObject = false;"
            r0.println(r7)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r7 = "static bool JavaCPP_haveNonvirtual = false;"
            r0.println(r7)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r9 = "static const char* JavaCPP_classNames["
            r7.append(r9)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r9 = r1.jclasses
            int r9 = r9.size()
            r7.append(r9)
            java.lang.String r9 = "] = {"
            r7.append(r9)
            java.lang.String r7 = r7.toString()
            r0.println(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r0 = r1.jclasses
            java.util.Iterator r7 = r0.iterator()
            r0 = 0
        L_0x051e:
            r9 = r0
            boolean r0 = r7.hasNext()
            r13 = 47
            if (r0 == 0) goto L_0x057a
            java.lang.Object r0 = r7.next()
            java.lang.Class r0 = (java.lang.Class) r0
            java.io.PrintWriter r14 = r1.out
            java.lang.StringBuilder r15 = new java.lang.StringBuilder
            r15.<init>()
            java.lang.String r12 = "        \""
            r15.append(r12)
            java.lang.String r12 = r0.getName()
            r20 = r8
            r8 = 46
            java.lang.String r8 = r12.replace(r8, r13)
            r15.append(r8)
            java.lang.String r8 = "\""
            r15.append(r8)
            java.lang.String r8 = r15.toString()
            r14.print(r8)
            boolean r8 = r7.hasNext()
            if (r8 == 0) goto L_0x0561
            java.io.PrintWriter r8 = r1.out
            java.lang.String r12 = ","
            r8.println(r12)
        L_0x0561:
            java.util.Map<java.lang.Class, java.util.Set<java.lang.String>> r8 = r1.members
            java.lang.Object r8 = r8.get(r0)
            java.util.Set r8 = (java.util.Set) r8
            if (r8 == 0) goto L_0x0576
            int r12 = r8.size()
            if (r12 <= r9) goto L_0x0576
            int r0 = r8.size()
            goto L_0x0577
        L_0x0576:
            r0 = r9
        L_0x0577:
            r8 = r20
            goto L_0x051e
        L_0x057a:
            r20 = r8
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = " };"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            java.lang.String r12 = "static jclass JavaCPP_classes["
            r8.append(r12)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r12 = r1.jclasses
            int r12 = r12.size()
            r8.append(r12)
            java.lang.String r12 = "] = { NULL };"
            r8.append(r12)
            java.lang.String r8 = r8.toString()
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static jfieldID JavaCPP_addressFID = NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static jfieldID JavaCPP_positionFID = NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static jfieldID JavaCPP_limitFID = NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static jfieldID JavaCPP_capacityFID = NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static jfieldID JavaCPP_deallocatorFID = NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static jfieldID JavaCPP_ownerAddressFID = NULL;"
            r0.println(r8)
            if (r39 == 0) goto L_0x05ec
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static jfieldID JavaCPP_byteValueFID = NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static jfieldID JavaCPP_shortValueFID = NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static jfieldID JavaCPP_intValueFID = NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static jfieldID JavaCPP_longValueFID = NULL;"
            r0.println(r8)
        L_0x05ec:
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static jmethodID JavaCPP_initMID = NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static jmethodID JavaCPP_arrayMID = NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static jmethodID JavaCPP_arrayOffsetMID = NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static jfieldID JavaCPP_bufferPositionFID = NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static jfieldID JavaCPP_bufferLimitFID = NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static jfieldID JavaCPP_bufferCapacityFID = NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static jmethodID JavaCPP_stringMID = NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static jmethodID JavaCPP_getBytesMID = NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static jmethodID JavaCPP_toStringMID = NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static inline void JavaCPP_log(const char* fmt, ...) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    va_list ap;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    va_start(ap, fmt);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#ifdef __ANDROID__"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    __android_log_vprint(ANDROID_LOG_ERROR, \"javacpp\", fmt, ap);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#elif defined(__APPLE__) && defined(__OBJC__)"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    NSLogv([NSString stringWithUTF8String:fmt], ap);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#else"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    vfprintf(stderr, fmt, ap);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    fprintf(stderr, \"\\n\");"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#endif"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    va_end(ap);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "}"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#ifdef __ANDROID__"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    static pthread_key_t JavaCPP_current_env;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    static JavaCPP_noinline void JavaCPP_detach_env(void *data)"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        if (JavaCPP_vm) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            JavaCPP_vm->DetachCurrentThread();"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    static JavaCPP_noinline void JavaCPP_create_pthread_key(void)"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        pthread_key_create(&JavaCPP_current_env, JavaCPP_detach_env);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#endif"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            if (r5 == 0) goto L_0x06f8
            boolean r0 = r41.isEmpty()
            if (r0 == 0) goto L_0x0eb6
        L_0x06f8:
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static inline jboolean JavaCPP_trimMemory() {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#if defined(__linux__) && !defined(__ANDROID__)"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    return (jboolean)malloc_trim(0);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#else"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    return 0;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#endif"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "}"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static inline jlong JavaCPP_physicalBytes() {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    jlong size = 0;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#ifdef __linux__"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    static int fd = open(\"/proc/self/statm\", O_RDONLY, 0);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (fd >= 0) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        char line[256];"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        char* s;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        int n;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        if ((n = pread(fd, line, sizeof(line), 0)) > 0 && (s = (char*)memchr(line, ' ', n)) != NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            size = (jlong)(atoll(s + 1) * getpagesize());"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        // no close(fd);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#elif defined(__APPLE__)"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    task_basic_info info;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    mach_msg_type_number_t count = TASK_BASIC_INFO_COUNT;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (task_info(current_task(), TASK_BASIC_INFO, (task_info_t)&info, &count) == KERN_SUCCESS) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        size = (jlong)info.resident_size;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#elif defined(_WIN32)"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    PROCESS_MEMORY_COUNTERS counters;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (GetProcessMemoryInfo(GetCurrentProcess(), &counters, sizeof(counters))) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        size = (jlong)counters.WorkingSetSize;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#endif"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    return size;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "}"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static inline jlong JavaCPP_totalPhysicalBytes() {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    jlong size = 0;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#ifdef __linux__"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    struct sysinfo info;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (sysinfo(&info) == 0) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        size = info.totalram;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#elif defined(__APPLE__)"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    size_t length = sizeof(size);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    sysctlbyname(\"hw.memsize\", &size, &length, NULL, 0);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#elif defined(_WIN32)"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    MEMORYSTATUSEX status;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    status.dwLength = sizeof(status);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (GlobalMemoryStatusEx(&status)) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        size = status.ullTotalPhys;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#endif"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    return size;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "}"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static inline jlong JavaCPP_availablePhysicalBytes() {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    jlong size = 0;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#ifdef __linux__"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    int fd = open(\"/proc/meminfo\", O_RDONLY, 0);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (fd >= 0) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        char temp[4096];"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        char *s;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        int n;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        if ((n = read(fd, temp, sizeof(temp))) > 0 && (s = (char*)memmem(temp, n, \"MemAvailable:\", 13)) != NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            size = (jlong)(atoll(s + 13) * 1024);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        close(fd);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (size == 0) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        struct sysinfo info;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        if (sysinfo(&info) == 0) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            size = info.freeram;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#elif defined(__APPLE__)"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    vm_statistics_data_t info;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    mach_msg_type_number_t count = HOST_VM_INFO_COUNT;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (host_statistics(mach_host_self(), HOST_VM_INFO, (host_info_t)&info, &count) == KERN_SUCCESS) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        size = (jlong)info.free_count * getpagesize();"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#elif defined(_WIN32)"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    MEMORYSTATUSEX status;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    status.dwLength = sizeof(status);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (GlobalMemoryStatusEx(&status)) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        size = status.ullAvailPhys;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#endif"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    return size;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "}"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static inline jint JavaCPP_totalProcessors() {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    jint total = 0;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#ifdef __linux__"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    total = sysconf(_SC_NPROCESSORS_CONF);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#elif defined(__APPLE__)"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    size_t length = sizeof(total);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    sysctlbyname(\"hw.logicalcpu_max\", &total, &length, NULL, 0);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#elif defined(_WIN32)"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    SYSTEM_INFO info;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    GetSystemInfo(&info);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    total = info.dwNumberOfProcessors;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#endif"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    return total;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "}"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static inline jint JavaCPP_totalCores() {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    jint total = 0;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#ifdef __linux__"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    const int n = sysconf(_SC_NPROCESSORS_CONF);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    int pids[n], cids[n];"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    for (int i = 0; i < n; i++) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        int fd = 0, pid = 0, cid = 0;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        char temp[256];"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        sprintf(temp, \"/sys/devices/system/cpu/cpu%d/topology/physical_package_id\", i);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        if ((fd = open(temp, O_RDONLY, 0)) >= 0) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            if (read(fd, temp, sizeof(temp)) > 0) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "                pid = atoi(temp);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            close(fd);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        sprintf(temp, \"/sys/devices/system/cpu/cpu%d/topology/core_id\", i);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        if ((fd = open(temp, O_RDONLY, 0)) >= 0) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            if (read(fd, temp, sizeof(temp)) > 0) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "                cid = atoi(temp);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            close(fd);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        bool found = false;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        for (int j = 0; j < total; j++) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            if (pids[j] == pid && cids[j] == cid) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "                found = true;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "                break;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        if (!found) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            pids[total] = pid;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            cids[total] = cid;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            total++;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#elif defined(__APPLE__)"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    size_t length = sizeof(total);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    sysctlbyname(\"hw.physicalcpu_max\", &total, &length, NULL, 0);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#elif defined(_WIN32)"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    SYSTEM_LOGICAL_PROCESSOR_INFORMATION *info = NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    DWORD length = 0;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    BOOL success = GetLogicalProcessorInformation(info, &length);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    while (!success && GetLastError() == ERROR_INSUFFICIENT_BUFFER) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        info = (SYSTEM_LOGICAL_PROCESSOR_INFORMATION*)realloc(info, length);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        success = GetLogicalProcessorInformation(info, &length);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (success && info != NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        length /= sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        for (DWORD i = 0; i < length; i++) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            if (info[i].Relationship == RelationProcessorCore) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "                total++;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    free(info);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#endif"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    return total;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "}"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static inline jint JavaCPP_totalChips() {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    jint total = 0;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#ifdef __linux__"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    const int n = sysconf(_SC_NPROCESSORS_CONF);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    int pids[n];"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    for (int i = 0; i < n; i++) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        int fd = 0, pid = 0;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        char temp[256];"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        sprintf(temp, \"/sys/devices/system/cpu/cpu%d/topology/physical_package_id\", i);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        if ((fd = open(temp, O_RDONLY, 0)) >= 0) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            if (read(fd, temp, sizeof(temp)) > 0) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "                pid = atoi(temp);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            close(fd);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        bool found = false;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        for (int j = 0; j < total; j++) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            if (pids[j] == pid) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "                found = true;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "                break;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        if (!found) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            pids[total] = pid;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            total++;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#elif defined(__APPLE__)"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    size_t length = sizeof(total);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    sysctlbyname(\"hw.packages\", &total, &length, NULL, 0);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#elif defined(_WIN32)"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    SYSTEM_LOGICAL_PROCESSOR_INFORMATION *info = NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    DWORD length = 0;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    BOOL success = GetLogicalProcessorInformation(info, &length);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    while (!success && GetLastError() == ERROR_INSUFFICIENT_BUFFER) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        info = (SYSTEM_LOGICAL_PROCESSOR_INFORMATION*)realloc(info, length);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        success = GetLogicalProcessorInformation(info, &length);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (success && info != NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        length /= sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        for (DWORD i = 0; i < length; i++) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            if (info[i].Relationship == RelationProcessorPackage) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "                total++;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    free(info);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#endif"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    return total;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "}"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#if defined(__linux__) && !(defined(__ANDROID__) && defined(__arm__))"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static int JavaCPP_dlcallback(dl_phdr_info *info, size_t size, void *data) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    void *handle = dlopen(info->dlpi_name, RTLD_LAZY);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (handle != NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        void *address = dlsym(handle, ((char**)data)[0]);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        dlclose(handle);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        if (address != NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            ((void**)data)[1] = address;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            return 1;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    return 0;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "}"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#endif"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static inline void* JavaCPP_addressof(const char* name) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    void *address = NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#ifdef __linux__"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    address = dlsym(RTLD_DEFAULT, name);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#if !(defined(__ANDROID__) && defined(__arm__))"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (address == NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        void *data[] = { (char*)name, NULL };"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        dl_iterate_phdr(JavaCPP_dlcallback, data);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        address = data[1];"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#endif"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#elif defined(__APPLE__)"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    address = dlsym(RTLD_DEFAULT, name);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (address == NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        for (uint32_t i = 0; i < _dyld_image_count(); i++) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            const char *libname = _dyld_get_image_name(i);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            if (libname != NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "                void *handle = dlopen(libname, RTLD_LAZY);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "                if (handle != NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "                    address = dlsym(handle, name);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "                    dlclose(handle);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "                    if (address != NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "                        break;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "                    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "                }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#elif defined(_WIN32)"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    HANDLE process = GetCurrentProcess();"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    HMODULE *modules = NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    DWORD length = 0, needed = 0;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    BOOL success = EnumProcessModules(process, modules, length, &needed);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    while (success && needed > length) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        modules = (HMODULE*)realloc(modules, length = needed);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        success = EnumProcessModules(process, modules, length, &needed);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (success && modules != NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        length = needed / sizeof(HMODULE);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        for (DWORD i = 0; i < length; i++) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            address = (void*)GetProcAddress(modules[i], name);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            if (address != NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "                break;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    free(modules);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#endif"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    return address;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "}"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static inline JavaVM* JavaCPP_getJavaVM() {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    return JavaCPP_vm;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "}"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
        L_0x0eb6:
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static JavaCPP_noinline jclass JavaCPP_getClass(JNIEnv* env, int i) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (JavaCPP_classes[i] == NULL && env->PushLocalFrame(1) == 0) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        jclass cls = env->FindClass(JavaCPP_classNames[i]);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        if (cls == NULL || env->ExceptionCheck()) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            JavaCPP_log(\"Error loading class %s.\", JavaCPP_classNames[i]);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            return NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        JavaCPP_classes[i] = (jclass)env->NewWeakGlobalRef(cls);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        if (JavaCPP_classes[i] == NULL || env->ExceptionCheck()) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            JavaCPP_log(\"Error creating global reference of class %s.\", JavaCPP_classNames[i]);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            return NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        env->PopLocalFrame(NULL);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    return JavaCPP_classes[i];"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "}"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static JavaCPP_noinline jfieldID JavaCPP_getFieldID(JNIEnv* env, int i, const char* name, const char* sig) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    jclass cls = JavaCPP_getClass(env, i);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (cls == NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        return NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    jfieldID fid = env->GetFieldID(cls, name, sig);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (fid == NULL || env->ExceptionCheck()) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        JavaCPP_log(\"Error getting field ID of %s/%s\", JavaCPP_classNames[i], name);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        return NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    return fid;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "}"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            if (r39 == 0) goto L_0x0fe6
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static JavaCPP_noinline jfieldID JavaCPP_getFieldID(JNIEnv* env, const char* clsName, const char* name, const char* sig) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    jclass cls = env->FindClass(clsName);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (cls == NULL || env->ExceptionCheck()) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        JavaCPP_log(\"Error loading class %s.\", clsName);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        return NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    jfieldID fid = env->GetFieldID(cls, name, sig);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (fid == NULL || env->ExceptionCheck()) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        JavaCPP_log(\"Error getting field ID of %s/%s\", clsName, name);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        return NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    return fid;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "}"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
        L_0x0fe6:
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static JavaCPP_noinline jmethodID JavaCPP_getMethodID(JNIEnv* env, int i, const char* name, const char* sig) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    jclass cls = JavaCPP_getClass(env, i);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (cls == NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        return NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    jmethodID mid = env->GetMethodID(cls, name, sig);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (mid == NULL || env->ExceptionCheck()) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        JavaCPP_log(\"Error getting method ID of %s/%s\", JavaCPP_classNames[i], name);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        return NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    return mid;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "}"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static JavaCPP_noinline jmethodID JavaCPP_getStaticMethodID(JNIEnv* env, int i, const char* name, const char* sig) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    jclass cls = JavaCPP_getClass(env, i);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (cls == NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        return NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    jmethodID mid = env->GetStaticMethodID(cls, name, sig);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (mid == NULL || env->ExceptionCheck()) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        JavaCPP_log(\"Error getting static method ID of %s/%s\", JavaCPP_classNames[i], name);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        return NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    return mid;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "}"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static JavaCPP_noinline jobject JavaCPP_createPointer(JNIEnv* env, int i, jclass cls = NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (cls == NULL && (cls = JavaCPP_getClass(env, i)) == NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        return NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (JavaCPP_haveAllocObject) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        return env->AllocObject(cls);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    } else {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        jmethodID mid = env->GetMethodID(cls, \"<init>\", \"(Lorg/bytedeco/javacpp/Pointer;)V\");"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        if (mid == NULL || env->ExceptionCheck()) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            JavaCPP_log(\"Error getting Pointer constructor of %s, while VM does not support AllocObject()\", JavaCPP_classNames[i]);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            return NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        return env->NewObject(cls, mid, NULL);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "}"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static JavaCPP_noinline void JavaCPP_initPointer(JNIEnv* env, jobject obj, const void* ptr, jlong size, void* owner, void (*deallocator)(void*)) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (deallocator != NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        jvalue args[4];"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        args[0].j = ptr_to_jlong(ptr);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        args[1].j = size;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        args[2].j = ptr_to_jlong(owner);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        args[3].j = ptr_to_jlong(deallocator);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        if (JavaCPP_haveNonvirtual) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            java.lang.String r12 = "            env->CallNonvirtualVoidMethodA(obj, JavaCPP_getClass(env, "
            r8.append(r12)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r12 = r1.jclasses
            java.lang.Class<org.bytedeco.javacpp.Pointer> r14 = org.bytedeco.javacpp.Pointer.class
            int r12 = r12.index(r14)
            r8.append(r12)
            java.lang.String r12 = "), JavaCPP_initMID, args);"
            r8.append(r12)
            java.lang.String r8 = r8.toString()
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        } else {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            env->CallVoidMethodA(obj, JavaCPP_initMID, args);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    } else {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        env->SetLongField(obj, JavaCPP_addressFID, ptr_to_jlong(ptr));"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        env->SetLongField(obj, JavaCPP_limitFID, (jlong)size);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        env->SetLongField(obj, JavaCPP_capacityFID, (jlong)size);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "}"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            if (r36 != 0) goto L_0x11a9
            if (r38 == 0) goto L_0x1225
        L_0x11a9:
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static JavaCPP_noinline jstring JavaCPP_createString(JNIEnv* env, const char* ptr) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (ptr == NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        return NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#ifdef MODIFIED_UTF8_STRING"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    return env->NewStringUTF(ptr);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#else"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    size_t length = strlen(ptr);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    jbyteArray bytes = env->NewByteArray(length < INT_MAX ? length : INT_MAX);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    env->SetByteArrayRegion(bytes, 0, length < INT_MAX ? length : INT_MAX, (signed char*)ptr);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            java.lang.String r12 = "    return (jstring)env->NewObject(JavaCPP_getClass(env, "
            r8.append(r12)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r12 = r1.jclasses
            java.lang.Class<java.lang.String> r14 = java.lang.String.class
            int r12 = r12.index(r14)
            r8.append(r12)
            java.lang.String r12 = "), JavaCPP_stringMID, bytes);"
            r8.append(r12)
            java.lang.String r8 = r8.toString()
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#endif"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "}"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
        L_0x1225:
            if (r38 == 0) goto L_0x1303
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static JavaCPP_noinline const char* JavaCPP_getStringBytes(JNIEnv* env, jstring str) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (str == NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        return NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#ifdef MODIFIED_UTF8_STRING"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    return env->GetStringUTFChars(str, NULL);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#else"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    jbyteArray bytes = (jbyteArray)env->CallObjectMethod(str, JavaCPP_getBytesMID);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (bytes == NULL || env->ExceptionCheck()) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        JavaCPP_log(\"Error getting bytes from string.\");"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        return NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    jsize length = env->GetArrayLength(bytes);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    signed char* ptr = new (std::nothrow) signed char[length + 1];"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (ptr != NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        env->GetByteArrayRegion(bytes, 0, length, ptr);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        ptr[length] = 0;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    return (const char*)ptr;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#endif"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "}"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static JavaCPP_noinline void JavaCPP_releaseStringBytes(JNIEnv* env, jstring str, const char* ptr) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#ifdef MODIFIED_UTF8_STRING"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (str != NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        env->ReleaseStringUTFChars(str, ptr);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#else"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    delete[] ptr;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#endif"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "}"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
        L_0x1303:
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "class JavaCPP_hidden JavaCPP_exception : public std::exception {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "public:"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    JavaCPP_exception(const char* str) throw() {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        if (str == NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            strcpy(msg, \"Unknown exception.\");"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        } else {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            strncpy(msg, str, sizeof(msg));"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "            msg[sizeof(msg) - 1] = 0;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    virtual const char* what() const throw() { return msg; }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    char msg[1024];"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "};"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            if (r36 == 0) goto L_0x13fd
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#ifndef GENERIC_EXCEPTION_CLASS"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#define GENERIC_EXCEPTION_CLASS std::exception"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#endif"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#ifndef GENERIC_EXCEPTION_TOSTRING"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#define GENERIC_EXCEPTION_TOSTRING what()"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "#endif"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "static JavaCPP_noinline jthrowable JavaCPP_handleException(JNIEnv* env, int i) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    jstring str = NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    try {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        throw;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    } catch (GENERIC_EXCEPTION_CLASS& e) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        str = JavaCPP_createString(env, e.GENERIC_EXCEPTION_TOSTRING);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    } catch (...) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        str = JavaCPP_createString(env, \"Unknown exception.\");"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    jmethodID mid = JavaCPP_getMethodID(env, i, \"<init>\", \"(Ljava/lang/String;)V\");"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    if (mid == NULL) {"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "        return NULL;"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    }"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "    return (jthrowable)env->NewObject(JavaCPP_getClass(env, i), mid, str);"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r8 = "}"
            r0.println(r8)
            java.io.PrintWriter r0 = r1.out
            r0.println()
        L_0x13fd:
            java.lang.StringBuilder r0 = new java.lang.StringBuilder     // Catch:{ ClassNotFoundException -> 0x2b17 }
            r0.<init>()     // Catch:{ ClassNotFoundException -> 0x2b17 }
            java.lang.Class<org.bytedeco.javacpp.Pointer> r8 = org.bytedeco.javacpp.Pointer.class
            java.lang.String r8 = r8.getName()     // Catch:{ ClassNotFoundException -> 0x2b17 }
            r0.append(r8)     // Catch:{ ClassNotFoundException -> 0x2b17 }
            java.lang.String r8 = "$Deallocator"
            r0.append(r8)     // Catch:{ ClassNotFoundException -> 0x2b17 }
            java.lang.String r0 = r0.toString()     // Catch:{ ClassNotFoundException -> 0x2b17 }
            java.lang.Class<org.bytedeco.javacpp.Pointer> r8 = org.bytedeco.javacpp.Pointer.class
            java.lang.ClassLoader r8 = r8.getClassLoader()     // Catch:{ ClassNotFoundException -> 0x2b17 }
            r12 = 0
            java.lang.Class r0 = java.lang.Class.forName(r0, r12, r8)     // Catch:{ ClassNotFoundException -> 0x2b17 }
            r8 = r0
            java.lang.StringBuilder r0 = new java.lang.StringBuilder     // Catch:{ ClassNotFoundException -> 0x2b17 }
            r0.<init>()     // Catch:{ ClassNotFoundException -> 0x2b17 }
            java.lang.Class<org.bytedeco.javacpp.Pointer> r12 = org.bytedeco.javacpp.Pointer.class
            java.lang.String r12 = r12.getName()     // Catch:{ ClassNotFoundException -> 0x2b17 }
            r0.append(r12)     // Catch:{ ClassNotFoundException -> 0x2b17 }
            java.lang.String r12 = "$NativeDeallocator"
            r0.append(r12)     // Catch:{ ClassNotFoundException -> 0x2b17 }
            java.lang.String r0 = r0.toString()     // Catch:{ ClassNotFoundException -> 0x2b17 }
            java.lang.Class<org.bytedeco.javacpp.Pointer> r12 = org.bytedeco.javacpp.Pointer.class
            java.lang.ClassLoader r12 = r12.getClassLoader()     // Catch:{ ClassNotFoundException -> 0x2b17 }
            r14 = 0
            java.lang.Class r0 = java.lang.Class.forName(r0, r14, r12)     // Catch:{ ClassNotFoundException -> 0x2b17 }
            r12 = r0
            if (r37 == 0) goto L_0x19c8
            java.io.PrintWriter r0 = r1.out
            java.lang.String r15 = "static JavaCPP_noinline void* JavaCPP_getPointerOwner(JNIEnv* env, jobject obj) {"
            r0.println(r15)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r15 = "    if (obj != NULL) {"
            r0.println(r15)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r15 = "        jobject deallocator = env->GetObjectField(obj, JavaCPP_deallocatorFID);"
            r0.println(r15)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r15 = new java.lang.StringBuilder
            r15.<init>()
            java.lang.String r13 = "        if (deallocator != NULL && env->IsInstanceOf(deallocator, JavaCPP_getClass(env, "
            r15.append(r13)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r13 = r1.jclasses
            int r13 = r13.index(r12)
            r15.append(r13)
            java.lang.String r13 = "))) {"
            r15.append(r13)
            java.lang.String r13 = r15.toString()
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            return jlong_to_ptr(env->GetLongField(deallocator, JavaCPP_ownerAddressFID));"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    return NULL;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "}"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "#include <vector>"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "template<typename P, typename T = P> class JavaCPP_hidden VectorAdapter {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "public:"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    VectorAdapter(const P* ptr, typename std::vector<T>::size_type size, void* owner) : ptr((P*)ptr), size(size), owner(owner),"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        vec2(ptr ? std::vector<T>((P*)ptr, (P*)ptr + size) : std::vector<T>()), vec(vec2) { }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    VectorAdapter(const std::vector<T>& vec) : ptr(0), size(0), owner(0), vec2(vec), vec(vec2) { }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    VectorAdapter(      std::vector<T>& vec) : ptr(0), size(0), owner(0), vec(vec) { }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    VectorAdapter(const std::vector<T>* vec) : ptr(0), size(0), owner(0), vec(*(std::vector<T>*)vec) { }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    void assign(P* ptr, typename std::vector<T>::size_type size, void* owner) {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        this->ptr = ptr;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        this->size = size;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        this->owner = owner;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        vec.assign(ptr, ptr + size);"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    static void deallocate(void* owner) { operator delete(owner); }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator P*() {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        if (vec.size() > size) {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            ptr = (P*)(operator new(sizeof(P) * vec.size(), std::nothrow_t()));"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        if (ptr) {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            std::copy(vec.begin(), vec.end(), ptr);"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        size = vec.size();"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        owner = ptr;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        return ptr;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator const P*()        { return &vec[0]; }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator std::vector<T>&() { return vec; }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator std::vector<T>*() { return ptr ? &vec : 0; }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    P* ptr;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    typename std::vector<T>::size_type size;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    void* owner;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    std::vector<T> vec2;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    std::vector<T>& vec;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "};"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "#include <string>"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "template<typename T = char> class JavaCPP_hidden StringAdapter {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "public:"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    StringAdapter(const          char* ptr, typename std::basic_string<T>::size_type size, void* owner) : ptr((T*)ptr), size(size), owner(owner),"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        str2(ptr ? (T*)ptr : \"\", ptr ? (size > 0 ? size : strlen((char*)ptr)) : 0), str(str2) { }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    StringAdapter(const signed   char* ptr, typename std::basic_string<T>::size_type size, void* owner) : ptr((T*)ptr), size(size), owner(owner),"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        str2(ptr ? (T*)ptr : \"\", ptr ? (size > 0 ? size : strlen((char*)ptr)) : 0), str(str2) { }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    StringAdapter(const unsigned char* ptr, typename std::basic_string<T>::size_type size, void* owner) : ptr((T*)ptr), size(size), owner(owner),"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        str2(ptr ? (T*)ptr : \"\", ptr ? (size > 0 ? size : strlen((char*)ptr)) : 0), str(str2) { }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    StringAdapter(const       wchar_t* ptr, typename std::basic_string<T>::size_type size, void* owner) : ptr((T*)ptr), size(size), owner(owner),"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        str2(ptr ? (T*)ptr : L\"\", ptr ? (size > 0 ? size : wcslen((wchar_t*)ptr)) : 0), str(str2) { }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    StringAdapter(const unsigned short* ptr, typename std::basic_string<T>::size_type size, void* owner) : ptr((T*)ptr), size(size), owner(owner),"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        str2(ptr ? (T*)ptr : L\"\", ptr ? (size > 0 ? size : wcslen((wchar_t*)ptr)) : 0), str(str2) { }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    StringAdapter(const   signed   int* ptr, typename std::basic_string<T>::size_type size, void* owner) : ptr((T*)ptr), size(size), owner(owner),"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        str2(ptr ? (T*)ptr : L\"\", ptr ? (size > 0 ? size : wcslen((wchar_t*)ptr)) : 0), str(str2) { }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    StringAdapter(const std::basic_string<T>& str) : ptr(0), size(0), owner(0), str2(str), str(str2) { }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    StringAdapter(      std::basic_string<T>& str) : ptr(0), size(0), owner(0), str(str) { }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    StringAdapter(const std::basic_string<T>* str) : ptr(0), size(0), owner(0), str(*(std::basic_string<T>*)str) { }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    void assign(char* ptr, typename std::basic_string<T>::size_type size, void* owner) {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        this->ptr = ptr;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        this->size = size;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        this->owner = owner;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        str.assign(ptr ? ptr : \"\", ptr ? (size > 0 ? size : strlen((char*)ptr)) : 0);"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    void assign(const          char* ptr, typename std::basic_string<T>::size_type size, void* owner) { assign((char*)ptr, size, owner); }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    void assign(const signed   char* ptr, typename std::basic_string<T>::size_type size, void* owner) { assign((char*)ptr, size, owner); }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    void assign(const unsigned char* ptr, typename std::basic_string<T>::size_type size, void* owner) { assign((char*)ptr, size, owner); }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    void assign(wchar_t* ptr, typename std::basic_string<T>::size_type size, void* owner) {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        this->ptr = ptr;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        this->size = size;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        this->owner = owner;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        str.assign(ptr ? ptr : L\"\", ptr ? (size > 0 ? size : wcslen((wchar_t*)ptr)) : 0);"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    void assign(const        wchar_t* ptr, typename std::basic_string<T>::size_type size, void* owner) { assign((wchar_t*)ptr, size, owner); }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    void assign(const unsigned short* ptr, typename std::basic_string<T>::size_type size, void* owner) { assign((wchar_t*)ptr, size, owner); }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    void assign(const   signed   int* ptr, typename std::basic_string<T>::size_type size, void* owner) { assign((wchar_t*)ptr, size, owner); }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    static void deallocate(void* owner) { delete[] (T*)owner; }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator char*() {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        const char* data = str.data();"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        if (str.size() > size) {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            ptr = new (std::nothrow) char[str.size()+1];"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            if (ptr) memset(ptr, 0, str.size()+1);"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        if (ptr && memcmp(ptr, data, str.size()) != 0) {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            memcpy(ptr, data, str.size());"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            if (size > str.size()) ptr[str.size()] = 0;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        size = str.size();"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        owner = ptr;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        return ptr;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator       signed   char*() { return (signed   char*)(operator char*)(); }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator       unsigned char*() { return (unsigned char*)(operator char*)(); }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator const          char*() { return                 str.c_str(); }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator const signed   char*() { return (signed   char*)str.c_str(); }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator const unsigned char*() { return (unsigned char*)str.c_str(); }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator wchar_t*() {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        const wchar_t* data = str.data();"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        if (str.size() > size) {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            ptr = new (std::nothrow) wchar_t[str.size()+1];"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            if (ptr) memset(ptr, 0, sizeof(wchar_t) * (str.size()+1));"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        if (ptr && memcmp(ptr, data, sizeof(wchar_t) * str.size()) != 0) {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            memcpy(ptr, data, sizeof(wchar_t) * str.size());"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            if (size > str.size()) ptr[str.size()] = 0;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        size = str.size();"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        owner = ptr;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        return ptr;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator     unsigned   short*() { return (unsigned short*)(operator wchar_t*)(); }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator       signed     int*() { return (  signed   int*)(operator wchar_t*)(); }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator const        wchar_t*() { return                  str.c_str(); }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator const unsigned short*() { return (unsigned short*)str.c_str(); }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator const   signed   int*() { return (  signed   int*)str.c_str(); }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator         std::basic_string<T>&() { return str; }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator         std::basic_string<T>*() { return ptr ? &str : 0; }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    T* ptr;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    typename std::basic_string<T>::size_type size;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    void* owner;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    std::basic_string<T> str2;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    std::basic_string<T>& str;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "};"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "#ifdef SHARED_PTR_NAMESPACE"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "template<class T> class SharedPtrAdapter {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "public:"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    typedef SHARED_PTR_NAMESPACE::shared_ptr<T> S;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    SharedPtrAdapter(const T* ptr, size_t size, void* owner) : ptr((T*)ptr), size(size), owner(owner),"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            sharedPtr2(owner != NULL && owner != ptr ? *(S*)owner : S((T*)ptr)), sharedPtr(sharedPtr2) { }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    SharedPtrAdapter(const S& sharedPtr) : ptr(0), size(0), owner(0), sharedPtr2(sharedPtr), sharedPtr(sharedPtr2) { }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    SharedPtrAdapter(      S& sharedPtr) : ptr(0), size(0), owner(0), sharedPtr(sharedPtr) { }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    SharedPtrAdapter(const S* sharedPtr) : ptr(0), size(0), owner(0), sharedPtr(*(S*)sharedPtr) { }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    void assign(T* ptr, size_t size, S* owner) {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        this->ptr = ptr;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        this->size = size;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        this->owner = owner;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        this->sharedPtr = owner != NULL && owner != ptr ? *(S*)owner : S((T*)ptr);"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    static void deallocate(void* owner) { delete (S*)owner; }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator typename SHARED_PTR_NAMESPACE::remove_const<T>::type*() {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        ptr = sharedPtr.get();"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        if (owner == NULL || owner == ptr) {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            owner = new S(sharedPtr);"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        return (typename SHARED_PTR_NAMESPACE::remove_const<T>::type*)ptr;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator S&() { return sharedPtr; }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator S*() { return &sharedPtr; }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    T* ptr;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    size_t size;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    void* owner;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    S sharedPtr2;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    S& sharedPtr;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "};"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "#endif"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "#ifdef UNIQUE_PTR_NAMESPACE"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "template<class T> class UniquePtrAdapter {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "public:"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    typedef UNIQUE_PTR_NAMESPACE::unique_ptr<T> U;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    UniquePtrAdapter(const T* ptr, size_t size, void* owner) : ptr((T*)ptr), size(size), owner(owner),"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            uniquePtr2(owner != NULL && owner != ptr ? U() : U((T*)ptr)),"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            uniquePtr(owner != NULL && owner != ptr ? *(U*)owner : uniquePtr2) { }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    UniquePtrAdapter(U&& uniquePtr) : ptr(0), size(0), owner(0), uniquePtr2(UNIQUE_PTR_NAMESPACE::move(uniquePtr)), uniquePtr(uniquePtr2) { }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    UniquePtrAdapter(const U& uniquePtr) : ptr(0), size(0), owner(0), uniquePtr((U&)uniquePtr) { }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    UniquePtrAdapter(      U& uniquePtr) : ptr(0), size(0), owner(0), uniquePtr(uniquePtr) { }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    UniquePtrAdapter(const U* uniquePtr) : ptr(0), size(0), owner(0), uniquePtr(*(U*)uniquePtr) { }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    void assign(T* ptr, size_t size, U* owner) {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        this->ptr = ptr;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        this->size = size;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        this->owner = owner;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        this->uniquePtr = owner != NULL && owner != ptr ? *(U*)owner : U((T*)ptr);"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    static void deallocate(void* owner) { delete (U*)owner; }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator typename UNIQUE_PTR_NAMESPACE::remove_const<T>::type*() {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        ptr = uniquePtr.get();"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        if (ptr == uniquePtr2.get() && (owner == NULL || owner == ptr)) {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            // only move the pointer if we actually own it through uniquePtr2"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            owner = new U(UNIQUE_PTR_NAMESPACE::move(uniquePtr));"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        return (typename UNIQUE_PTR_NAMESPACE::remove_const<T>::type*)ptr;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator U&() { return uniquePtr; }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    operator U*() { return &uniquePtr; }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    T* ptr;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    size_t size;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    void* owner;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    U uniquePtr2;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    U& uniquePtr;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "};"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "#endif"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            r0.println()
        L_0x19c8:
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r0 = r1.functions
            boolean r0 = r0.isEmpty()
            if (r0 == 0) goto L_0x19d8
            java.util.Map<java.lang.Class, java.util.Set<java.lang.String>> r0 = r1.virtualFunctions
            boolean r0 = r0.isEmpty()
            if (r0 != 0) goto L_0x1beb
        L_0x19d8:
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "#ifdef __ANDROID__"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "  static pthread_once_t JavaCPP_once = PTHREAD_ONCE_INIT;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "  static pthread_mutex_t JavaCPP_lock = PTHREAD_MUTEX_INITIALIZER;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "#endif"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "static JavaCPP_noinline void JavaCPP_detach(bool detach) {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "#if !defined(NO_JNI_DETACH_THREAD) && !defined(__ANDROID__)"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    if (detach && JavaCPP_vm->DetachCurrentThread() != JNI_OK) {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        JavaCPP_log(\"Could not detach the JavaVM from the current thread.\");"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "#endif"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "}"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            boolean r0 = r11.isEmpty()
            if (r0 != 0) goto L_0x1a5e
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "extern \"C\" {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r15 = "JNIEXPORT jint JNICALL JNI_OnLoad"
            r13.append(r15)
            r13.append(r11)
            java.lang.String r15 = "(JavaVM* vm, void* reserved);"
            r13.append(r15)
            java.lang.String r13 = r13.toString()
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "}"
            r0.println(r13)
        L_0x1a5e:
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "static JavaCPP_noinline bool JavaCPP_getEnv(JNIEnv** env) {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    bool attached = false;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    JavaVM *vm = JavaCPP_vm;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    if (vm == NULL) {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out2
            if (r0 == 0) goto L_0x1a9a
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "#if !defined(__ANDROID__) && !TARGET_OS_IPHONE"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        int size = 1;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        if (JNI_GetCreatedJavaVMs(&vm, 1, &size) != JNI_OK || size == 0) {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "#endif"
            r0.println(r13)
        L_0x1a9a:
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            JavaCPP_log(\"Could not get any created JavaVM.\");"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            *env = NULL;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            return false;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out2
            if (r0 == 0) goto L_0x1ac8
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "#if !defined(__ANDROID__) && !TARGET_OS_IPHONE"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "#endif"
            r0.println(r13)
        L_0x1ac8:
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "#ifdef __ANDROID__"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    pthread_mutex_lock(&JavaCPP_lock);"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    pthread_once(&JavaCPP_once, JavaCPP_create_pthread_key);"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    if ((*env = (JNIEnv *)pthread_getspecific(JavaCPP_current_env)) != NULL) {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        attached = true;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        goto done;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "#endif"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    if (vm->GetEnv((void**)env, JNI_VERSION_1_6) != JNI_OK) {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        struct {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            JNIEnv **env;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            operator JNIEnv**() { return env; } // Android JNI"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            operator void**() { return (void**)env; } // standard JNI"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        } env2 = { env };"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        if (vm->AttachCurrentThread(env2, NULL) != JNI_OK) {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            JavaCPP_log(\"Could not attach the JavaVM to the current thread.\");"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            *env = NULL;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            goto done;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "#ifdef __ANDROID__"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        pthread_setspecific(JavaCPP_current_env, *env);"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "#endif"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        attached = true;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    if (JavaCPP_vm == NULL) {"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r15 = "        if (JNI_OnLoad"
            r13.append(r15)
            r13.append(r11)
            java.lang.String r15 = "(vm, NULL) < 0) {"
            r13.append(r15)
            java.lang.String r13 = r13.toString()
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            JavaCPP_detach(attached);"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            *env = NULL;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "            goto done;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "        }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    }"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "done:"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "#ifdef __ANDROID__"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    pthread_mutex_unlock(&JavaCPP_lock);"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "#endif"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "    return attached;"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r13 = "}"
            r0.println(r13)
            java.io.PrintWriter r0 = r1.out
            r0.println()
        L_0x1beb:
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r0 = r1.functions
            java.util.Iterator r0 = r0.iterator()
        L_0x1bf1:
            boolean r13 = r0.hasNext()
            if (r13 == 0) goto L_0x1ce9
            java.lang.Object r13 = r0.next()
            java.lang.Class r13 = (java.lang.Class) r13
            java.lang.String[] r15 = r1.cppTypeName(r13)
            r21 = r0
            r16 = 0
            r0 = r15[r16]
            java.lang.String r2 = "\\("
            java.lang.String[] r0 = r0.split(r2)
            r2 = 1
            java.lang.String[] r3 = new java.lang.String[r2]
            r17 = r0[r2]
            r3[r16] = r17
            java.lang.String r3 = constValueTypeName(r3)
            r0[r2] = r3
            r3 = r15[r2]
            java.lang.String r3 = r3.substring(r2)
            java.lang.String r2 = functionClassName(r13)
            r22 = r7
            java.io.PrintWriter r7 = r1.out
            r23 = r10
            java.lang.StringBuilder r10 = new java.lang.StringBuilder
            r10.<init>()
            r24 = r13
            java.lang.String r13 = "struct JavaCPP_hidden "
            r10.append(r13)
            r10.append(r2)
            java.lang.String r13 = " {"
            r10.append(r13)
            java.lang.String r10 = r10.toString()
            r7.println(r10)
            java.io.PrintWriter r7 = r1.out
            java.lang.StringBuilder r10 = new java.lang.StringBuilder
            r10.<init>()
            java.lang.String r13 = "    "
            r10.append(r13)
            r10.append(r2)
            java.lang.String r13 = "() : ptr(NULL), obj(NULL) { }"
            r10.append(r13)
            java.lang.String r10 = r10.toString()
            r7.println(r10)
            if (r3 == 0) goto L_0x1c8e
            int r7 = r3.length()
            if (r7 <= 0) goto L_0x1c8e
            java.io.PrintWriter r7 = r1.out
            java.lang.StringBuilder r10 = new java.lang.StringBuilder
            r10.<init>()
            java.lang.String r13 = "    "
            r10.append(r13)
            r13 = 0
            r14 = r0[r13]
            r10.append(r14)
            java.lang.String r13 = "operator()"
            r10.append(r13)
            r10.append(r3)
            java.lang.String r13 = ";"
            r10.append(r13)
            java.lang.String r10 = r10.toString()
            r7.println(r10)
        L_0x1c8e:
            java.io.PrintWriter r7 = r1.out
            java.lang.StringBuilder r10 = new java.lang.StringBuilder
            r10.<init>()
            java.lang.String r13 = "    "
            r10.append(r13)
            r13 = 0
            r14 = r15[r13]
            r10.append(r14)
            java.lang.String r13 = "ptr"
            r10.append(r13)
            r13 = 1
            r14 = r15[r13]
            r10.append(r14)
            java.lang.String r13 = ";"
            r10.append(r13)
            java.lang.String r10 = r10.toString()
            r7.println(r10)
            java.io.PrintWriter r7 = r1.out
            java.lang.String r10 = "    jobject obj; static jmethodID mid;"
            r7.println(r10)
            java.io.PrintWriter r7 = r1.out
            java.lang.String r10 = "};"
            r7.println(r10)
            java.io.PrintWriter r7 = r1.out
            java.lang.StringBuilder r10 = new java.lang.StringBuilder
            r10.<init>()
            java.lang.String r13 = "jmethodID "
            r10.append(r13)
            r10.append(r2)
            java.lang.String r13 = "::mid = NULL;"
            r10.append(r13)
            java.lang.String r10 = r10.toString()
            r7.println(r10)
            r0 = r21
            r7 = r22
            r10 = r23
            goto L_0x1bf1
        L_0x1ce9:
            r22 = r7
            r23 = r10
            java.io.PrintWriter r0 = r1.out
            r0.println()
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r0 = r1.jclasses
            java.util.Iterator r0 = r0.iterator()
        L_0x1cf8:
            boolean r2 = r0.hasNext()
            if (r2 == 0) goto L_0x1e00
            java.lang.Object r2 = r0.next()
            java.lang.Class r2 = (java.lang.Class) r2
            java.util.Map<java.lang.Class, java.util.Set<java.lang.String>> r3 = r1.virtualFunctions
            java.lang.Object r3 = r3.get(r2)
            java.util.Set r3 = (java.util.Set) r3
            if (r3 != 0) goto L_0x1d0f
            goto L_0x1cf8
        L_0x1d0f:
            java.util.Map<java.lang.Class, java.util.Set<java.lang.String>> r7 = r1.virtualMembers
            java.lang.Object r7 = r7.get(r2)
            java.util.Set r7 = (java.util.Set) r7
            java.lang.String[] r10 = r1.cppTypeName(r2)
            java.lang.String r13 = valueTypeName(r10)
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            java.lang.String r15 = "JavaCPP_"
            r14.append(r15)
            java.lang.String r15 = mangle(r13)
            r14.append(r15)
            java.lang.String r14 = r14.toString()
            java.io.PrintWriter r15 = r1.out
            r25 = r0
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r26 = r2
            java.lang.String r2 = "class JavaCPP_hidden "
            r0.append(r2)
            r0.append(r14)
            java.lang.String r2 = " : public "
            r0.append(r2)
            r0.append(r13)
            java.lang.String r2 = " {"
            r0.append(r2)
            java.lang.String r0 = r0.toString()
            r15.println(r0)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r2 = "public:"
            r0.println(r2)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r2 = "    jobject obj;"
            r0.println(r2)
            java.util.Iterator r0 = r3.iterator()
        L_0x1d6d:
            boolean r2 = r0.hasNext()
            if (r2 == 0) goto L_0x1d9e
            java.lang.Object r2 = r0.next()
            java.lang.String r2 = (java.lang.String) r2
            java.io.PrintWriter r15 = r1.out
            r27 = r0
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r28 = r10
            java.lang.String r10 = "    static jmethodID "
            r0.append(r10)
            r0.append(r2)
            java.lang.String r10 = ";"
            r0.append(r10)
            java.lang.String r0 = r0.toString()
            r15.println(r0)
            r0 = r27
            r10 = r28
            goto L_0x1d6d
        L_0x1d9e:
            r28 = r10
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.util.Iterator r0 = r7.iterator()
        L_0x1da9:
            boolean r2 = r0.hasNext()
            if (r2 == 0) goto L_0x1dbb
            java.lang.Object r2 = r0.next()
            java.lang.String r2 = (java.lang.String) r2
            java.io.PrintWriter r10 = r1.out
            r10.println(r2)
            goto L_0x1da9
        L_0x1dbb:
            java.io.PrintWriter r0 = r1.out
            java.lang.String r2 = "};"
            r0.println(r2)
            java.util.Iterator r0 = r3.iterator()
        L_0x1dc6:
            boolean r2 = r0.hasNext()
            if (r2 == 0) goto L_0x1dfb
            java.lang.Object r2 = r0.next()
            java.lang.String r2 = (java.lang.String) r2
            java.io.PrintWriter r10 = r1.out
            java.lang.StringBuilder r15 = new java.lang.StringBuilder
            r15.<init>()
            r29 = r0
            java.lang.String r0 = "jmethodID "
            r15.append(r0)
            r15.append(r14)
            java.lang.String r0 = "::"
            r15.append(r0)
            r15.append(r2)
            java.lang.String r0 = " = NULL;"
            r15.append(r0)
            java.lang.String r0 = r15.toString()
            r10.println(r0)
            r0 = r29
            goto L_0x1dc6
        L_0x1dfb:
            r0 = r25
            goto L_0x1cf8
        L_0x1e00:
            java.io.PrintWriter r0 = r1.out
            r0.println()
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.String> r0 = r1.callbacks
            java.util.Iterator r0 = r0.iterator()
        L_0x1e0b:
            boolean r2 = r0.hasNext()
            if (r2 == 0) goto L_0x1e1d
            java.lang.Object r2 = r0.next()
            java.lang.String r2 = (java.lang.String) r2
            java.io.PrintWriter r3 = r1.out
            r3.println(r2)
            goto L_0x1e0b
        L_0x1e1d:
            java.io.PrintWriter r0 = r1.out
            r0.println()
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r0 = r1.deallocators
            java.util.Iterator r0 = r0.iterator()
        L_0x1e28:
            boolean r2 = r0.hasNext()
            if (r2 == 0) goto L_0x1f26
            java.lang.Object r2 = r0.next()
            java.lang.Class r2 = (java.lang.Class) r2
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "JavaCPP_"
            r3.append(r7)
            java.lang.String r7 = r2.getName()
            java.lang.String r7 = mangle(r7)
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            java.io.PrintWriter r7 = r1.out
            java.lang.StringBuilder r10 = new java.lang.StringBuilder
            r10.<init>()
            java.lang.String r13 = "static void "
            r10.append(r13)
            r10.append(r3)
            java.lang.String r13 = "_deallocate(void *p) { "
            r10.append(r13)
            java.lang.String r10 = r10.toString()
            r7.print(r10)
            java.lang.Class<org.bytedeco.javacpp.FunctionPointer> r7 = org.bytedeco.javacpp.FunctionPointer.class
            boolean r7 = r7.isAssignableFrom(r2)
            if (r7 == 0) goto L_0x1eac
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r10 = functionClassName(r2)
            r7.append(r10)
            java.lang.String r10 = "*"
            r7.append(r10)
            java.lang.String r7 = r7.toString()
            java.io.PrintWriter r10 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "JNIEnv *e; bool a = JavaCPP_getEnv(&e); if (e != NULL) e->DeleteWeakGlobalRef((jweak)(("
            r13.append(r14)
            r13.append(r7)
            java.lang.String r14 = ")p)->obj); delete ("
            r13.append(r14)
            r13.append(r7)
            java.lang.String r14 = ")p; JavaCPP_detach(a); }"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r10.println(r13)
            r30 = r0
            goto L_0x1f21
        L_0x1eac:
            java.util.Map<java.lang.Class, java.util.Set<java.lang.String>> r7 = r1.virtualFunctions
            boolean r7 = r7.containsKey(r2)
            if (r7 == 0) goto L_0x1ef7
            java.lang.String[] r7 = r1.cppTypeName(r2)
            java.lang.String r10 = valueTypeName(r7)
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "JavaCPP_"
            r13.append(r14)
            java.lang.String r14 = mangle(r10)
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            java.io.PrintWriter r14 = r1.out
            java.lang.StringBuilder r15 = new java.lang.StringBuilder
            r15.<init>()
            r30 = r0
            java.lang.String r0 = "JNIEnv *e; bool a = JavaCPP_getEnv(&e); if (e != NULL) e->DeleteWeakGlobalRef((jweak)(("
            r15.append(r0)
            r15.append(r13)
            java.lang.String r0 = "*)p)->obj); delete ("
            r15.append(r0)
            r15.append(r13)
            java.lang.String r0 = "*)p; JavaCPP_detach(a); }"
            r15.append(r0)
            java.lang.String r0 = r15.toString()
            r14.println(r0)
            goto L_0x1f21
        L_0x1ef7:
            r30 = r0
            java.lang.String[] r0 = r1.cppTypeName(r2)
            java.io.PrintWriter r7 = r1.out
            java.lang.StringBuilder r10 = new java.lang.StringBuilder
            r10.<init>()
            java.lang.String r13 = "delete ("
            r10.append(r13)
            r13 = 0
            r14 = r0[r13]
            r10.append(r14)
            r13 = 1
            r14 = r0[r13]
            r10.append(r14)
            java.lang.String r13 = ")p; }"
            r10.append(r13)
            java.lang.String r10 = r10.toString()
            r7.println(r10)
        L_0x1f21:
            r0 = r30
            goto L_0x1e28
        L_0x1f26:
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r0 = r1.arrayDeallocators
            java.util.Iterator r0 = r0.iterator()
        L_0x1f2c:
            boolean r2 = r0.hasNext()
            if (r2 == 0) goto L_0x1f82
            java.lang.Object r2 = r0.next()
            java.lang.Class r2 = (java.lang.Class) r2
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "JavaCPP_"
            r3.append(r7)
            java.lang.String r7 = r2.getName()
            java.lang.String r7 = mangle(r7)
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            java.lang.String[] r7 = r1.cppTypeName(r2)
            java.io.PrintWriter r10 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "static void "
            r13.append(r14)
            r13.append(r3)
            java.lang.String r14 = "_deallocateArray(void* p) { delete[] ("
            r13.append(r14)
            r14 = 0
            r15 = r7[r14]
            r13.append(r15)
            r14 = 1
            r15 = r7[r14]
            r13.append(r15)
            java.lang.String r14 = ")p; }"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r10.println(r13)
            goto L_0x1f2c
        L_0x1f82:
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "static const char* JavaCPP_members["
            r2.append(r3)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r3 = r1.jclasses
            int r3 = r3.size()
            r2.append(r3)
            java.lang.String r3 = "]["
            r2.append(r3)
            r2.append(r9)
            r3 = 1
            r2.append(r3)
            java.lang.String r3 = "] = {"
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r0.println(r2)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r0 = r1.jclasses
            java.util.Iterator r0 = r0.iterator()
        L_0x1fba:
            boolean r2 = r0.hasNext()
            if (r2 == 0) goto L_0x2035
            java.io.PrintWriter r2 = r1.out
            java.lang.String r7 = "        { "
            r2.print(r7)
            java.util.Map<java.lang.Class, java.util.Set<java.lang.String>> r2 = r1.members
            java.lang.Object r7 = r0.next()
            java.lang.Object r2 = r2.get(r7)
            java.util.Set r2 = (java.util.Set) r2
            if (r2 != 0) goto L_0x1fd7
            r3 = 0
            goto L_0x1fdb
        L_0x1fd7:
            java.util.Iterator r3 = r2.iterator()
        L_0x1fdb:
            if (r3 == 0) goto L_0x2019
            boolean r7 = r3.hasNext()
            if (r7 != 0) goto L_0x1fe4
            goto L_0x2019
        L_0x1fe4:
            boolean r7 = r3.hasNext()
            if (r7 == 0) goto L_0x2020
            java.io.PrintWriter r7 = r1.out
            java.lang.StringBuilder r10 = new java.lang.StringBuilder
            r10.<init>()
            java.lang.String r13 = "\""
            r10.append(r13)
            java.lang.Object r13 = r3.next()
            java.lang.String r13 = (java.lang.String) r13
            r10.append(r13)
            java.lang.String r13 = "\""
            r10.append(r13)
            java.lang.String r10 = r10.toString()
            r7.print(r10)
            boolean r7 = r3.hasNext()
            if (r7 == 0) goto L_0x1fe4
            java.io.PrintWriter r7 = r1.out
            java.lang.String r10 = ", "
            r7.print(r10)
            goto L_0x1fe4
        L_0x2019:
            java.io.PrintWriter r7 = r1.out
            java.lang.String r10 = "NULL"
            r7.print(r10)
        L_0x2020:
            java.io.PrintWriter r7 = r1.out
            java.lang.String r10 = " }"
            r7.print(r10)
            boolean r7 = r0.hasNext()
            if (r7 == 0) goto L_0x2034
            java.io.PrintWriter r7 = r1.out
            java.lang.String r10 = ","
            r7.println(r10)
        L_0x2034:
            goto L_0x1fba
        L_0x2035:
            java.io.PrintWriter r2 = r1.out
            java.lang.String r7 = " };"
            r2.println(r7)
            java.io.PrintWriter r2 = r1.out
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r10 = "static int JavaCPP_offsets["
            r7.append(r10)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r10 = r1.jclasses
            int r10 = r10.size()
            r7.append(r10)
            java.lang.String r10 = "]["
            r7.append(r10)
            r7.append(r9)
            r10 = 1
            r7.append(r10)
            java.lang.String r10 = "] = {"
            r7.append(r10)
            java.lang.String r7 = r7.toString()
            r2.println(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r2 = r1.jclasses
            java.util.Iterator r0 = r2.iterator()
        L_0x206f:
            boolean r2 = r0.hasNext()
            if (r2 == 0) goto L_0x2146
            java.io.PrintWriter r2 = r1.out
            java.lang.String r7 = "        { "
            r2.print(r7)
            java.lang.Object r2 = r0.next()
            java.lang.Class r2 = (java.lang.Class) r2
            java.util.Map<java.lang.Class, java.util.Set<java.lang.String>> r7 = r1.members
            java.lang.Object r7 = r7.get(r2)
            java.util.Set r7 = (java.util.Set) r7
            if (r7 != 0) goto L_0x208e
            r10 = 0
            goto L_0x2092
        L_0x208e:
            java.util.Iterator r10 = r7.iterator()
        L_0x2092:
            if (r10 == 0) goto L_0x2125
            boolean r13 = r10.hasNext()
            if (r13 != 0) goto L_0x20a0
            r31 = r2
            r32 = r7
            goto L_0x2129
        L_0x20a0:
            boolean r13 = r10.hasNext()
            if (r13 == 0) goto L_0x2120
            java.lang.String[] r13 = r1.cppTypeName(r2)
            java.lang.String r14 = valueTypeName(r13)
            java.lang.Object r15 = r10.next()
            java.lang.String r15 = (java.lang.String) r15
            java.lang.String r3 = "sizeof"
            boolean r3 = r3.equals(r15)
            if (r3 == 0) goto L_0x20e6
            java.lang.String r3 = "void"
            boolean r3 = r3.equals(r14)
            if (r3 == 0) goto L_0x20c6
            java.lang.String r14 = "void*"
        L_0x20c6:
            java.io.PrintWriter r3 = r1.out
            r31 = r2
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            r32 = r7
            java.lang.String r7 = "sizeof("
            r2.append(r7)
            r2.append(r14)
            java.lang.String r7 = ")"
            r2.append(r7)
            java.lang.String r2 = r2.toString()
            r3.print(r2)
            goto L_0x210d
        L_0x20e6:
            r31 = r2
            r32 = r7
            java.io.PrintWriter r2 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "offsetof("
            r3.append(r7)
            r3.append(r14)
            java.lang.String r7 = ", "
            r3.append(r7)
            r3.append(r15)
            java.lang.String r7 = ")"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r2.print(r3)
        L_0x210d:
            boolean r2 = r10.hasNext()
            if (r2 == 0) goto L_0x211a
            java.io.PrintWriter r2 = r1.out
            java.lang.String r3 = ", "
            r2.print(r3)
        L_0x211a:
            r2 = r31
            r7 = r32
            goto L_0x20a0
        L_0x2120:
            r31 = r2
            r32 = r7
            goto L_0x2130
        L_0x2125:
            r31 = r2
            r32 = r7
        L_0x2129:
            java.io.PrintWriter r2 = r1.out
            java.lang.String r3 = "-1"
            r2.print(r3)
        L_0x2130:
            java.io.PrintWriter r2 = r1.out
            java.lang.String r3 = " }"
            r2.print(r3)
            boolean r2 = r0.hasNext()
            if (r2 == 0) goto L_0x2144
            java.io.PrintWriter r2 = r1.out
            java.lang.String r3 = ","
            r2.println(r3)
        L_0x2144:
            goto L_0x206f
        L_0x2146:
            java.io.PrintWriter r2 = r1.out
            java.lang.String r3 = " };"
            r2.println(r3)
            java.io.PrintWriter r2 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "static int JavaCPP_memberOffsetSizes["
            r3.append(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r7 = r1.jclasses
            int r7 = r7.size()
            r3.append(r7)
            java.lang.String r7 = "] = { "
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r2.print(r3)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r2 = r1.jclasses
            java.util.Iterator r2 = r2.iterator()
        L_0x2174:
            boolean r0 = r2.hasNext()
            if (r0 == 0) goto L_0x21a1
            java.util.Map<java.lang.Class, java.util.Set<java.lang.String>> r0 = r1.members
            java.lang.Object r3 = r2.next()
            java.lang.Object r0 = r0.get(r3)
            java.util.Set r0 = (java.util.Set) r0
            java.io.PrintWriter r3 = r1.out
            if (r0 != 0) goto L_0x218c
            r7 = 1
            goto L_0x2190
        L_0x218c:
            int r7 = r0.size()
        L_0x2190:
            r3.print(r7)
            boolean r3 = r2.hasNext()
            if (r3 == 0) goto L_0x21a0
            java.io.PrintWriter r3 = r1.out
            java.lang.String r7 = ", "
            r3.print(r7)
        L_0x21a0:
            goto L_0x2174
        L_0x21a1:
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = " };"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "extern \"C\" {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out2
            if (r0 == 0) goto L_0x22c5
            java.io.PrintWriter r0 = r1.out2
            r0.println()
            java.io.PrintWriter r0 = r1.out2
            java.lang.String r3 = "#ifdef __cplusplus"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out2
            java.lang.String r3 = "extern \"C\" {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out2
            java.lang.String r3 = "#endif"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out2
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "JNIIMPORT int JavaCPP_init"
            r3.append(r7)
            r3.append(r11)
            java.lang.String r7 = "(int argc, const char *argv[]);"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "JNIEXPORT int JavaCPP_init"
            r3.append(r7)
            r3.append(r11)
            java.lang.String r7 = "(int argc, const char *argv[]) {"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "#if defined(__ANDROID__) || TARGET_OS_IPHONE"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    return JNI_OK;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "#else"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (JavaCPP_vm != NULL) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return JNI_OK;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    int err;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    JavaVM *vm;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    JNIEnv *env;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    int nOptions = 1 + (argc > 255 ? 255 : argc);"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    JavaVMOption options[256] = { { NULL } };"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    options[0].optionString = (char*)\"-Djava.class.path="
            r3.append(r7)
            r7 = 92
            r10 = r42
            r13 = 47
            java.lang.String r7 = r10.replace(r7, r13)
            r3.append(r7)
            java.lang.String r7 = "\";"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    for (int i = 1; i < nOptions && argv != NULL; i++) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        options[i].optionString = (char*)argv[i - 1];"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    JavaVMInitArgs vm_args = { JNI_VERSION_1_6, nOptions, options };"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    return (err = JNI_CreateJavaVM(&vm, (void**)&env, &vm_args)) == JNI_OK && vm != NULL && (err = JNI_OnLoad"
            r3.append(r7)
            r3.append(r11)
            java.lang.String r7 = "(vm, NULL)) >= 0 ? JNI_OK : err;"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "#endif"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "}"
            r0.println(r3)
            goto L_0x22c7
        L_0x22c5:
            r10 = r42
        L_0x22c7:
            if (r5 == 0) goto L_0x230a
            boolean r0 = r41.isEmpty()
            if (r0 != 0) goto L_0x230a
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "JNIEXPORT jint JNICALL JNI_OnLoad"
            r3.append(r7)
            r3.append(r5)
            java.lang.String r7 = "(JavaVM* vm, void* reserved);"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "JNIEXPORT void JNICALL JNI_OnUnload"
            r3.append(r7)
            r3.append(r5)
            java.lang.String r7 = "(JavaVM* vm, void* reserved);"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
        L_0x230a:
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "JNIEXPORT jint JNICALL JNI_OnLoad"
            r3.append(r7)
            r3.append(r11)
            java.lang.String r7 = "(JavaVM* vm, void* reserved) {"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            if (r5 == 0) goto L_0x235b
            boolean r0 = r41.isEmpty()
            if (r0 != 0) goto L_0x235b
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    if (JNI_OnLoad"
            r3.append(r7)
            r3.append(r5)
            java.lang.String r7 = "(vm, reserved) == JNI_ERR) {"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
        L_0x235b:
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    JNIEnv* env;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (vm->GetEnv((void**)&env, JNI_VERSION_1_6) != JNI_OK) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "        JavaCPP_log(\"Could not get JNIEnv for JNI_VERSION_1_6 inside JNI_OnLoad"
            r3.append(r7)
            r3.append(r11)
            java.lang.String r7 = "().\");"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (JavaCPP_vm == vm) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return env->GetVersion();"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    JavaCPP_vm = vm;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    JavaCPP_haveAllocObject = env->functions->AllocObject != NULL;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    JavaCPP_haveNonvirtual = env->functions->CallNonvirtualVoidMethodA != NULL;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    jmethodID putMemberOffsetMID = JavaCPP_getStaticMethodID(env, "
            r3.append(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r7 = r1.jclasses
            java.lang.Class<org.bytedeco.javacpp.Loader> r13 = org.bytedeco.javacpp.Loader.class
            int r7 = r7.index(r13)
            r3.append(r7)
            java.lang.String r7 = ", \"putMemberOffset\", \"(Ljava/lang/String;Ljava/lang/String;I)Ljava/lang/Class;\");"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (putMemberOffsetMID == NULL) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    for (int i = 0; i < "
            r3.append(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r7 = r1.jclasses
            int r7 = r7.size()
            r3.append(r7)
            java.lang.String r7 = " && !env->ExceptionCheck(); i++) {"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        for (int j = 0; j < JavaCPP_memberOffsetSizes[i] && !env->ExceptionCheck(); j++) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "            if (env->PushLocalFrame(3) == 0) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "                jvalue args[3];"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "                args[0].l = env->NewStringUTF(JavaCPP_classNames[i]);"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "                args[1].l = JavaCPP_members[i][j] == NULL ? NULL : env->NewStringUTF(JavaCPP_members[i][j]);"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "                args[2].i = JavaCPP_offsets[i][j];"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "                jclass cls = (jclass)env->CallStaticObjectMethodA(JavaCPP_getClass(env, "
            r3.append(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r7 = r1.jclasses
            java.lang.Class<org.bytedeco.javacpp.Loader> r13 = org.bytedeco.javacpp.Loader.class
            int r7 = r7.index(r13)
            r3.append(r7)
            java.lang.String r7 = "), putMemberOffsetMID, args);"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "                if (env->ExceptionCheck()) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "                    JavaCPP_log(\"Error putting member offsets for class %s.\", JavaCPP_classNames[i]);"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "                    return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "                }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "                JavaCPP_classes[i] = cls == NULL ? NULL : (jclass)env->NewWeakGlobalRef(cls);"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "                if (env->ExceptionCheck()) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "                    JavaCPP_log(\"Error creating global reference of class %s.\", JavaCPP_classNames[i]);"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "                    return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "                }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "                env->PopLocalFrame(NULL);"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "            }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    JavaCPP_addressFID = JavaCPP_getFieldID(env, "
            r3.append(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r7 = r1.jclasses
            java.lang.Class<org.bytedeco.javacpp.Pointer> r13 = org.bytedeco.javacpp.Pointer.class
            int r7 = r7.index(r13)
            r3.append(r7)
            java.lang.String r7 = ", \"address\", \"J\");"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (JavaCPP_addressFID == NULL) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    JavaCPP_positionFID = JavaCPP_getFieldID(env, "
            r3.append(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r7 = r1.jclasses
            java.lang.Class<org.bytedeco.javacpp.Pointer> r13 = org.bytedeco.javacpp.Pointer.class
            int r7 = r7.index(r13)
            r3.append(r7)
            java.lang.String r7 = ", \"position\", \"J\");"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (JavaCPP_positionFID == NULL) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    JavaCPP_limitFID = JavaCPP_getFieldID(env, "
            r3.append(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r7 = r1.jclasses
            java.lang.Class<org.bytedeco.javacpp.Pointer> r13 = org.bytedeco.javacpp.Pointer.class
            int r7 = r7.index(r13)
            r3.append(r7)
            java.lang.String r7 = ", \"limit\", \"J\");"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (JavaCPP_limitFID == NULL) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    JavaCPP_capacityFID = JavaCPP_getFieldID(env, "
            r3.append(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r7 = r1.jclasses
            java.lang.Class<org.bytedeco.javacpp.Pointer> r13 = org.bytedeco.javacpp.Pointer.class
            int r7 = r7.index(r13)
            r3.append(r7)
            java.lang.String r7 = ", \"capacity\", \"J\");"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (JavaCPP_capacityFID == NULL) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    JavaCPP_deallocatorFID = JavaCPP_getFieldID(env, "
            r3.append(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r7 = r1.jclasses
            java.lang.Class<org.bytedeco.javacpp.Pointer> r13 = org.bytedeco.javacpp.Pointer.class
            int r7 = r7.index(r13)
            r3.append(r7)
            java.lang.String r7 = ", \"deallocator\", \""
            r3.append(r7)
            r7 = 1
            java.lang.Class[] r7 = new java.lang.Class[r7]
            r13 = 0
            r7[r13] = r8
            java.lang.String r7 = signature(r7)
            r3.append(r7)
            java.lang.String r7 = "\");"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (JavaCPP_deallocatorFID == NULL) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    JavaCPP_ownerAddressFID = JavaCPP_getFieldID(env, "
            r3.append(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r7 = r1.jclasses
            int r7 = r7.index(r12)
            r3.append(r7)
            java.lang.String r7 = ", \"ownerAddress\", \"J\");"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (JavaCPP_ownerAddressFID == NULL) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            if (r39 == 0) goto L_0x2717
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    JavaCPP_byteValueFID = JavaCPP_getFieldID(env, \""
            r3.append(r7)
            java.lang.Class<org.bytedeco.javacpp.tools.Generator$ByteEnum> r7 = org.bytedeco.javacpp.tools.Generator.ByteEnum.class
            java.lang.String r7 = r7.getName()
            r14 = 47
            r15 = 46
            java.lang.String r7 = r7.replace(r15, r14)
            r3.append(r7)
            java.lang.String r7 = "\", \"value\", \"B\");"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (JavaCPP_byteValueFID == NULL) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    JavaCPP_shortValueFID = JavaCPP_getFieldID(env, \""
            r3.append(r7)
            java.lang.Class<org.bytedeco.javacpp.tools.Generator$ShortEnum> r7 = org.bytedeco.javacpp.tools.Generator.ShortEnum.class
            java.lang.String r7 = r7.getName()
            r14 = 47
            r15 = 46
            java.lang.String r7 = r7.replace(r15, r14)
            r3.append(r7)
            java.lang.String r7 = "\", \"value\", \"S\");"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (JavaCPP_shortValueFID == NULL) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    JavaCPP_intValueFID = JavaCPP_getFieldID(env, \""
            r3.append(r7)
            java.lang.Class<org.bytedeco.javacpp.tools.Generator$IntEnum> r7 = org.bytedeco.javacpp.tools.Generator.IntEnum.class
            java.lang.String r7 = r7.getName()
            r14 = 47
            r15 = 46
            java.lang.String r7 = r7.replace(r15, r14)
            r3.append(r7)
            java.lang.String r7 = "\", \"value\", \"I\");"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (JavaCPP_intValueFID == NULL) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    JavaCPP_longValueFID = JavaCPP_getFieldID(env, \""
            r3.append(r7)
            java.lang.Class<org.bytedeco.javacpp.tools.Generator$LongEnum> r7 = org.bytedeco.javacpp.tools.Generator.LongEnum.class
            java.lang.String r7 = r7.getName()
            r14 = 47
            r15 = 46
            java.lang.String r7 = r7.replace(r15, r14)
            r3.append(r7)
            java.lang.String r7 = "\", \"value\", \"J\");"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (JavaCPP_longValueFID == NULL) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
        L_0x2717:
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    JavaCPP_initMID = JavaCPP_getMethodID(env, "
            r3.append(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r7 = r1.jclasses
            java.lang.Class<org.bytedeco.javacpp.Pointer> r14 = org.bytedeco.javacpp.Pointer.class
            int r7 = r7.index(r14)
            r3.append(r7)
            java.lang.String r7 = ", \"init\", \"(JJJJ)V\");"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (JavaCPP_initMID == NULL) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    JavaCPP_arrayMID = JavaCPP_getMethodID(env, "
            r3.append(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r7 = r1.jclasses
            java.lang.Class<java.nio.Buffer> r14 = java.nio.Buffer.class
            int r7 = r7.index(r14)
            r3.append(r7)
            java.lang.String r7 = ", \"array\", \"()Ljava/lang/Object;\");"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (JavaCPP_arrayMID == NULL) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    JavaCPP_arrayOffsetMID = JavaCPP_getMethodID(env, "
            r3.append(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r7 = r1.jclasses
            java.lang.Class<java.nio.Buffer> r14 = java.nio.Buffer.class
            int r7 = r7.index(r14)
            r3.append(r7)
            java.lang.String r7 = ", \"arrayOffset\", \"()I\");"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (JavaCPP_arrayOffsetMID == NULL) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    JavaCPP_bufferPositionFID = JavaCPP_getFieldID(env, "
            r3.append(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r7 = r1.jclasses
            java.lang.Class<java.nio.Buffer> r14 = java.nio.Buffer.class
            int r7 = r7.index(r14)
            r3.append(r7)
            java.lang.String r7 = ", \"position\", \"I\");"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (JavaCPP_bufferPositionFID == NULL) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    JavaCPP_bufferLimitFID = JavaCPP_getFieldID(env, "
            r3.append(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r7 = r1.jclasses
            java.lang.Class<java.nio.Buffer> r14 = java.nio.Buffer.class
            int r7 = r7.index(r14)
            r3.append(r7)
            java.lang.String r7 = ", \"limit\", \"I\");"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (JavaCPP_bufferLimitFID == NULL) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    JavaCPP_bufferCapacityFID = JavaCPP_getFieldID(env, "
            r3.append(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r7 = r1.jclasses
            java.lang.Class<java.nio.Buffer> r14 = java.nio.Buffer.class
            int r7 = r7.index(r14)
            r3.append(r7)
            java.lang.String r7 = ", \"capacity\", \"I\");"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (JavaCPP_bufferCapacityFID == NULL) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    JavaCPP_stringMID = JavaCPP_getMethodID(env, "
            r3.append(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r7 = r1.jclasses
            java.lang.Class<java.lang.String> r14 = java.lang.String.class
            int r7 = r7.index(r14)
            r3.append(r7)
            java.lang.String r7 = ", \"<init>\", \"([B)V\");"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (JavaCPP_stringMID == NULL) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    JavaCPP_getBytesMID = JavaCPP_getMethodID(env, "
            r3.append(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r7 = r1.jclasses
            java.lang.Class<java.lang.String> r14 = java.lang.String.class
            int r7 = r7.index(r14)
            r3.append(r7)
            java.lang.String r7 = ", \"getBytes\", \"()[B\");"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (JavaCPP_getBytesMID == NULL) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    JavaCPP_toStringMID = JavaCPP_getMethodID(env, "
            r3.append(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r7 = r1.jclasses
            java.lang.Class<java.lang.Object> r14 = java.lang.Object.class
            int r7 = r7.index(r14)
            r3.append(r7)
            java.lang.String r7 = ", \"toString\", \"()Ljava/lang/String;\");"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (JavaCPP_toStringMID == NULL) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return JNI_ERR;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    return env->GetVersion();"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "}"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out2
            if (r0 == 0) goto L_0x29ad
            java.io.PrintWriter r0 = r1.out2
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "JNIIMPORT int JavaCPP_uninit"
            r3.append(r7)
            r3.append(r11)
            java.lang.String r7 = "();"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out2
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "JNIEXPORT int JavaCPP_uninit"
            r3.append(r7)
            r3.append(r11)
            java.lang.String r7 = "() {"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "#if defined(__ANDROID__) || TARGET_OS_IPHONE"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    return JNI_OK;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "#else"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    JavaVM *vm = JavaCPP_vm;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    JNI_OnUnload"
            r3.append(r7)
            r3.append(r11)
            java.lang.String r7 = "(JavaCPP_vm, NULL);"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    return vm->DestroyJavaVM();"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "#endif"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "}"
            r0.println(r3)
        L_0x29ad:
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "JNIEXPORT void JNICALL JNI_OnUnload"
            r3.append(r7)
            r3.append(r11)
            java.lang.String r7 = "(JavaVM* vm, void* reserved) {"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    JNIEnv* env;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (vm->GetEnv((void**)&env, JNI_VERSION_1_6) != JNI_OK) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "        JavaCPP_log(\"Could not get JNIEnv for JNI_VERSION_1_6 inside JNI_OnUnLoad"
            r3.append(r7)
            r3.append(r11)
            java.lang.String r7 = "().\");"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    for (int i = 0; i < "
            r3.append(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r7 = r1.jclasses
            int r7 = r7.size()
            r3.append(r7)
            java.lang.String r7 = "; i++) {"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        env->DeleteWeakGlobalRef((jweak)JavaCPP_classes[i]);"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        JavaCPP_classes[i] = NULL;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            if (r5 == 0) goto L_0x2a5d
            boolean r0 = r41.isEmpty()
            if (r0 != 0) goto L_0x2a5d
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    JNI_OnUnload"
            r3.append(r7)
            r3.append(r5)
            java.lang.String r7 = "(vm, reserved);"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
        L_0x2a5d:
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    JavaCPP_vm = NULL;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "}"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            r0 = 0
            java.util.LinkedHashSet r3 = new java.util.LinkedHashSet
            r3.<init>()
            if (r5 == 0) goto L_0x2a7e
            boolean r7 = r41.isEmpty()
            if (r7 == 0) goto L_0x2a84
        L_0x2a7e:
            r0 = 1
            java.util.List<java.lang.Class> r7 = baseClasses
            r3.addAll(r7)
        L_0x2a84:
            if (r6 == 0) goto L_0x2a9c
            java.util.List r7 = java.util.Arrays.asList(r43)
            r3.addAll(r7)
            int r7 = r6.length
        L_0x2a8e:
            if (r13 >= r7) goto L_0x2a9c
            r14 = r6[r13]
            java.util.Properties r15 = r1.properties
            boolean r15 = org.bytedeco.javacpp.Loader.checkPlatform(r14, r15)
            r0 = r0 | r15
            int r13 = r13 + 1
            goto L_0x2a8e
        L_0x2a9c:
            r7 = r0
            r0 = 0
            java.util.Iterator r13 = r3.iterator()
            r14 = r0
        L_0x2aa3:
            boolean r0 = r13.hasNext()
            if (r0 == 0) goto L_0x2aed
            java.lang.Object r0 = r13.next()
            java.lang.Class r0 = (java.lang.Class) r0
            r15 = r0
            boolean r0 = r1.methods(r15)     // Catch:{ NoClassDefFoundError -> 0x2abc }
            r0 = r0 | r14
            r14 = r0
            r33 = r2
            r34 = r3
            goto L_0x2ae7
        L_0x2abc:
            r0 = move-exception
            r16 = r0
            r0 = r16
            r33 = r2
            org.bytedeco.javacpp.tools.Logger r2 = r1.logger
            r34 = r3
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = "Could not generate code for class "
            r3.append(r4)
            java.lang.String r4 = r15.getCanonicalName()
            r3.append(r4)
            java.lang.String r4 = ": "
            r3.append(r4)
            r3.append(r0)
            java.lang.String r3 = r3.toString()
            r2.warn(r3)
        L_0x2ae7:
            r2 = r33
            r3 = r34
            goto L_0x2aa3
        L_0x2aed:
            r33 = r2
            r34 = r3
            java.io.PrintWriter r0 = r1.out
            java.lang.String r2 = "}"
            r0.println(r2)
            java.io.PrintWriter r0 = r1.out
            r0.println()
            java.io.PrintWriter r0 = r1.out2
            if (r0 == 0) goto L_0x2b16
            java.io.PrintWriter r0 = r1.out2
            java.lang.String r2 = "#ifdef __cplusplus"
            r0.println(r2)
            java.io.PrintWriter r0 = r1.out2
            java.lang.String r2 = "}"
            r0.println(r2)
            java.io.PrintWriter r0 = r1.out2
            java.lang.String r2 = "#endif"
            r0.println(r2)
        L_0x2b16:
            return r7
        L_0x2b17:
            r0 = move-exception
            r22 = r7
            r23 = r10
            r10 = r42
            java.lang.RuntimeException r2 = new java.lang.RuntimeException
            r2.<init>(r0)
            throw r2
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.Generator.classes(boolean, boolean, boolean, boolean, java.lang.String, java.lang.String, java.lang.String, java.lang.Class[]):boolean");
    }

    /* JADX WARNING: type inference failed for: r0v107, types: [java.lang.Object[]] */
    /* JADX WARNING: type inference failed for: r0v112, types: [java.lang.Object[]] */
    /* access modifiers changed from: package-private */
    /* JADX WARNING: Multi-variable type inference failed */
    /* JADX WARNING: Removed duplicated region for block: B:115:0x02b0  */
    /* JADX WARNING: Removed duplicated region for block: B:116:0x02b7  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public boolean methods(java.lang.Class<?> r28) {
        /*
            r27 = this;
            r6 = r27
            r7 = r28
            java.util.Properties r0 = r6.properties
            boolean r0 = org.bytedeco.javacpp.Loader.checkPlatform(r7, r0)
            r8 = 0
            if (r0 != 0) goto L_0x000e
            return r8
        L_0x000e:
            java.util.Map<java.lang.Class, java.util.Set<java.lang.String>> r0 = r6.members
            java.lang.Object r0 = r0.get(r7)
            java.util.Set r0 = (java.util.Set) r0
            java.lang.Class<org.bytedeco.javacpp.annotation.Opaque> r1 = org.bytedeco.javacpp.annotation.Opaque.class
            boolean r1 = r7.isAnnotationPresent(r1)
            if (r1 != 0) goto L_0x004c
            java.lang.Class<org.bytedeco.javacpp.Loader> r1 = org.bytedeco.javacpp.Loader.class
            if (r7 == r1) goto L_0x004c
            java.lang.Class<org.bytedeco.javacpp.FunctionPointer> r1 = org.bytedeco.javacpp.FunctionPointer.class
            boolean r1 = r1.isAssignableFrom(r7)
            if (r1 != 0) goto L_0x004c
            java.lang.Class r1 = r28.getEnclosingClass()
            java.lang.Class<org.bytedeco.javacpp.Pointer> r2 = org.bytedeco.javacpp.Pointer.class
            if (r1 == r2) goto L_0x004c
            if (r0 != 0) goto L_0x003f
            java.util.Map<java.lang.Class, java.util.Set<java.lang.String>> r1 = r6.members
            java.util.LinkedHashSet r2 = new java.util.LinkedHashSet
            r2.<init>()
            r0 = r2
            r1.put(r7, r2)
        L_0x003f:
            java.lang.String r1 = "sizeof"
            boolean r1 = r0.contains(r1)
            if (r1 != 0) goto L_0x004c
            java.lang.String r1 = "sizeof"
            r0.add(r1)
        L_0x004c:
            r9 = r0
            r0 = 0
            java.lang.Class[] r1 = r28.getDeclaredClasses()
            int r2 = r1.length
            r3 = r0
            r0 = 0
        L_0x0055:
            if (r0 >= r2) goto L_0x0075
            r4 = r1[r0]
            java.lang.Class<org.bytedeco.javacpp.Pointer> r5 = org.bytedeco.javacpp.Pointer.class
            boolean r5 = r5.isAssignableFrom(r4)
            if (r5 != 0) goto L_0x006d
            java.lang.Class<org.bytedeco.javacpp.Pointer> r5 = org.bytedeco.javacpp.Pointer.class
            java.lang.Class r10 = r4.getEnclosingClass()
            boolean r5 = r5.equals(r10)
            if (r5 == 0) goto L_0x0072
        L_0x006d:
            boolean r5 = r6.methods(r4)
            r3 = r3 | r5
        L_0x0072:
            int r0 = r0 + 1
            goto L_0x0055
        L_0x0075:
            java.lang.reflect.Method[] r0 = r28.getDeclaredMethods()
            int r1 = r0.length
            org.bytedeco.javacpp.tools.MethodInformation[] r1 = new org.bytedeco.javacpp.tools.MethodInformation[r1]
            r2 = 0
        L_0x007d:
            int r4 = r0.length
            if (r2 >= r4) goto L_0x008b
            r4 = r0[r2]
            org.bytedeco.javacpp.tools.MethodInformation r4 = r6.methodInformation(r4)
            r1[r2] = r4
            int r2 = r2 + 1
            goto L_0x007d
        L_0x008b:
            java.lang.Class r2 = r28.getSuperclass()
            r10 = r0
            r11 = r1
        L_0x0091:
            r12 = r2
            if (r12 == 0) goto L_0x010e
            java.lang.Class<java.lang.Object> r0 = java.lang.Object.class
            if (r12 == r0) goto L_0x010e
            java.lang.reflect.Method[] r0 = r12.getDeclaredMethods()
            int r1 = r0.length
            r2 = 0
        L_0x009e:
            if (r2 >= r1) goto L_0x0108
            r4 = r0[r2]
            java.lang.Class<org.bytedeco.javacpp.annotation.Virtual> r5 = org.bytedeco.javacpp.annotation.Virtual.class
            boolean r5 = r4.isAnnotationPresent(r5)
            if (r5 == 0) goto L_0x0100
            r5 = 0
            java.lang.String r14 = r4.getName()
            java.lang.Class[] r15 = r4.getParameterTypes()
            int r8 = r10.length
            r13 = 0
        L_0x00b5:
            if (r13 >= r8) goto L_0x00d6
            r16 = r10[r13]
            r17 = r0
            java.lang.String r0 = r16.getName()
            boolean r0 = r14.equals(r0)
            if (r0 == 0) goto L_0x00d1
            java.lang.Class[] r0 = r16.getParameterTypes()
            boolean r0 = java.util.Arrays.equals(r15, r0)
            if (r0 == 0) goto L_0x00d1
            r5 = 1
            goto L_0x00d8
        L_0x00d1:
            int r13 = r13 + 1
            r0 = r17
            goto L_0x00b5
        L_0x00d6:
            r17 = r0
        L_0x00d8:
            if (r5 != 0) goto L_0x0102
            int r0 = r10.length
            r8 = 1
            int r0 = r0 + r8
            java.lang.Object[] r0 = java.util.Arrays.copyOf(r10, r0)
            r10 = r0
            java.lang.reflect.Method[] r10 = (java.lang.reflect.Method[]) r10
            int r0 = r10.length
            int r0 = r0 - r8
            r10[r0] = r4
            int r0 = r11.length
            int r0 = r0 + r8
            java.lang.Object[] r0 = java.util.Arrays.copyOf(r11, r0)
            r11 = r0
            org.bytedeco.javacpp.tools.MethodInformation[] r11 = (org.bytedeco.javacpp.tools.MethodInformation[]) r11
            int r0 = r10.length
            int r0 = r0 - r8
            org.bytedeco.javacpp.tools.MethodInformation r13 = r6.methodInformation(r4)
            r11[r0] = r13
            int r0 = r10.length
            int r0 = r0 - r8
            r0 = r11[r0]
            r0.cls = r7
            goto L_0x0102
        L_0x0100:
            r17 = r0
        L_0x0102:
            int r2 = r2 + 1
            r0 = r17
            r8 = 0
            goto L_0x009e
        L_0x0108:
            java.lang.Class r2 = r12.getSuperclass()
            r8 = 0
            goto L_0x0091
        L_0x010e:
            int r0 = r10.length
            boolean[] r8 = new boolean[r0]
            java.lang.reflect.Method[] r13 = functionMethods(r7, r8)
            r0 = 1
            r1 = r0
            r0 = 0
        L_0x0118:
            r14 = r0
            int r0 = r10.length
            if (r14 >= r0) goto L_0x0543
            r0 = r10[r14]
            java.lang.Class<org.bytedeco.javacpp.annotation.Platform> r2 = org.bytedeco.javacpp.annotation.Platform.class
            java.lang.annotation.Annotation r0 = r0.getAnnotation(r2)
            org.bytedeco.javacpp.annotation.Platform r0 = (org.bytedeco.javacpp.annotation.Platform) r0
            java.util.Properties r2 = r6.properties
            r4 = 0
            java.lang.String[] r5 = new java.lang.String[r4]
            boolean r0 = org.bytedeco.javacpp.Loader.checkPlatform(r0, r2, r5)
            if (r0 != 0) goto L_0x0132
            goto L_0x01a3
        L_0x0132:
            r15 = r11[r14]
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r2 = r28.getName()
            java.lang.String r2 = mangle(r2)
            r0.append(r2)
            java.lang.String r2 = "_"
            r0.append(r2)
            r2 = r10[r14]
            java.lang.String r2 = r2.getName()
            java.lang.String r2 = mangle(r2)
            r0.append(r2)
            java.lang.String r5 = r0.toString()
            boolean r0 = r8[r14]
            if (r0 == 0) goto L_0x0165
            java.lang.Class<?>[] r0 = r15.parameterTypes
            int r0 = r0.length
            if (r0 <= 0) goto L_0x0165
            r0 = 0
            goto L_0x017b
        L_0x0165:
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r2 = "JavaCPP_"
            r0.append(r2)
            r0.append(r5)
            java.lang.String r2 = "_callback"
            r0.append(r2)
            java.lang.String r0 = r0.toString()
        L_0x017b:
            boolean r2 = r8[r14]
            if (r2 == 0) goto L_0x01ab
            if (r13 != 0) goto L_0x01ab
            org.bytedeco.javacpp.tools.Logger r2 = r6.logger
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            r18 = r0
            java.lang.String r0 = "No callback method call() or apply() has been not declared in \""
            r4.append(r0)
            java.lang.String r0 = r28.getCanonicalName()
            r4.append(r0)
            java.lang.String r0 = "\". No code will be generated for callback allocator."
            r4.append(r0)
            java.lang.String r0 = r4.toString()
            r2.warn(r0)
        L_0x01a3:
            r24 = r11
            r25 = r12
        L_0x01a7:
            r2 = 0
            r5 = 1
            goto L_0x053b
        L_0x01ab:
            r18 = r0
            if (r13 == 0) goto L_0x023f
            int r4 = r13.length
            r17 = r1
            r16 = r3
            r0 = r18
            r3 = 0
        L_0x01b7:
            if (r3 >= r4) goto L_0x0237
            r2 = r13[r3]
            if (r2 == 0) goto L_0x01c1
            boolean r1 = r8[r14]
            if (r1 != 0) goto L_0x01d5
        L_0x01c1:
            r1 = r10[r14]
            boolean r1 = r1.equals(r2)
            if (r1 == 0) goto L_0x0224
            r1 = r10[r14]
            int r1 = r1.getModifiers()
            boolean r1 = java.lang.reflect.Modifier.isNative(r1)
            if (r1 != 0) goto L_0x0224
        L_0x01d5:
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r1 = r6.functions
            r1.index(r7)
            r1 = r10[r14]
            r19 = r0
            java.lang.Class<org.bytedeco.javacpp.annotation.Name> r0 = org.bytedeco.javacpp.annotation.Name.class
            java.lang.annotation.Annotation r0 = r1.getAnnotation(r0)
            r18 = r0
            org.bytedeco.javacpp.annotation.Name r18 = (org.bytedeco.javacpp.annotation.Name) r18
            if (r18 == 0) goto L_0x0206
            java.lang.String[] r0 = r18.value()
            int r0 = r0.length
            if (r0 <= 0) goto L_0x0206
            java.lang.String[] r0 = r18.value()
            r1 = 0
            r0 = r0[r1]
            int r0 = r0.length()
            if (r0 <= 0) goto L_0x0206
            java.lang.String[] r0 = r18.value()
            r0 = r0[r1]
            r19 = r0
        L_0x0206:
            r20 = 0
            r0 = r27
            r1 = r28
            r21 = r2
            r22 = r3
            r3 = r19
            r23 = r4
            r4 = r17
            r24 = r11
            r11 = r5
            r5 = r20
            r0.callback(r1, r2, r3, r4, r5)
            r17 = 0
            r0 = 1
            r16 = r0
            goto L_0x022d
        L_0x0224:
            r19 = r0
            r22 = r3
            r23 = r4
            r24 = r11
            r11 = r5
        L_0x022d:
            r0 = r19
            int r3 = r22 + 1
            r5 = r11
            r4 = r23
            r11 = r24
            goto L_0x01b7
        L_0x0237:
            r19 = r0
            r24 = r11
            r11 = r5
            r5 = r19
            goto L_0x0248
        L_0x023f:
            r24 = r11
            r11 = r5
            r17 = r1
            r16 = r3
            r5 = r18
        L_0x0248:
            r0 = r10[r14]
            int r0 = r0.getModifiers()
            boolean r0 = java.lang.reflect.Modifier.isNative(r0)
            if (r0 != 0) goto L_0x0265
            r0 = r10[r14]
            int r0 = r0.getModifiers()
            boolean r0 = java.lang.reflect.Modifier.isAbstract(r0)
            if (r0 == 0) goto L_0x0261
            goto L_0x0265
        L_0x0261:
            r25 = r12
            r12 = r5
            goto L_0x02a4
        L_0x0265:
            boolean r0 = r15.valueGetter
            if (r0 != 0) goto L_0x02a1
            boolean r0 = r15.valueSetter
            if (r0 != 0) goto L_0x02a1
            boolean r0 = r15.memberGetter
            if (r0 != 0) goto L_0x02a1
            boolean r0 = r15.memberSetter
            if (r0 != 0) goto L_0x02a1
            boolean r0 = r28.isInterface()
            if (r0 != 0) goto L_0x02a1
            r0 = r10[r14]
            java.lang.Class<org.bytedeco.javacpp.annotation.Virtual> r1 = org.bytedeco.javacpp.annotation.Virtual.class
            boolean r0 = r0.isAnnotationPresent(r1)
            if (r0 != 0) goto L_0x0289
            boolean r0 = r15.allocator
            if (r0 == 0) goto L_0x0261
        L_0x0289:
            r2 = r10[r14]
            java.lang.String[] r0 = r15.memberName
            r1 = 0
            r3 = r0[r1]
            boolean r0 = r15.allocator
            r1 = 1
            r4 = r0 ^ 1
            r0 = r27
            r1 = r28
            r25 = r12
            r12 = r5
            r5 = r15
            r0.callback(r1, r2, r3, r4, r5)
            goto L_0x02a4
        L_0x02a1:
            r25 = r12
            r12 = r5
        L_0x02a4:
            r0 = r10[r14]
            int r0 = r0.getModifiers()
            boolean r0 = java.lang.reflect.Modifier.isNative(r0)
            if (r0 != 0) goto L_0x02b7
            r3 = r16
            r1 = r17
            goto L_0x01a7
        L_0x02b7:
            boolean r0 = r15.memberGetter
            if (r0 != 0) goto L_0x02bf
            boolean r0 = r15.memberSetter
            if (r0 == 0) goto L_0x02df
        L_0x02bf:
            boolean r0 = r15.noOffset
            if (r0 != 0) goto L_0x02df
            if (r9 == 0) goto L_0x02df
            int r0 = r15.modifiers
            boolean r0 = java.lang.reflect.Modifier.isStatic(r0)
            if (r0 != 0) goto L_0x02df
            java.lang.String[] r0 = r15.memberName
            r1 = 0
            r0 = r0[r1]
            boolean r0 = r9.contains(r0)
            if (r0 != 0) goto L_0x02df
            java.lang.String[] r0 = r15.memberName
            r0 = r0[r1]
            r9.add(r0)
        L_0x02df:
            r0 = 1
            java.io.PrintWriter r1 = r6.out
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "JNIEXPORT "
            r2.append(r3)
            java.lang.Class<?> r3 = r15.returnType
            java.lang.String r3 = jniTypeName(r3)
            r2.append(r3)
            java.lang.String r3 = " JNICALL Java_"
            r2.append(r3)
            r2.append(r11)
            java.lang.String r2 = r2.toString()
            r1.print(r2)
            boolean r1 = r15.overloaded
            if (r1 == 0) goto L_0x0328
            java.io.PrintWriter r1 = r6.out
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "__"
            r2.append(r3)
            java.lang.Class<?>[] r3 = r15.parameterTypes
            java.lang.String r3 = signature(r3)
            java.lang.String r3 = mangle(r3)
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r1.print(r2)
        L_0x0328:
            int r1 = r15.modifiers
            boolean r1 = java.lang.reflect.Modifier.isStatic(r1)
            if (r1 == 0) goto L_0x0338
            java.io.PrintWriter r1 = r6.out
            java.lang.String r2 = "(JNIEnv* env, jclass cls"
            r1.print(r2)
            goto L_0x033f
        L_0x0338:
            java.io.PrintWriter r1 = r6.out
            java.lang.String r2 = "(JNIEnv* env, jobject obj"
            r1.print(r2)
        L_0x033f:
            r1 = 0
        L_0x0340:
            java.lang.Class<?>[] r2 = r15.parameterTypes
            int r2 = r2.length
            if (r1 >= r2) goto L_0x036e
            java.io.PrintWriter r2 = r6.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = ", "
            r3.append(r4)
            java.lang.Class<?>[] r4 = r15.parameterTypes
            r4 = r4[r1]
            java.lang.String r4 = jniTypeName(r4)
            r3.append(r4)
            java.lang.String r4 = " arg"
            r3.append(r4)
            r3.append(r1)
            java.lang.String r3 = r3.toString()
            r2.print(r3)
            int r1 = r1 + 1
            goto L_0x0340
        L_0x036e:
            java.io.PrintWriter r1 = r6.out
            java.lang.String r2 = ") {"
            r1.println(r2)
            boolean r1 = r8[r14]
            if (r1 == 0) goto L_0x0383
            r6.callbackAllocator(r7, r12)
            r26 = r0
            r2 = 0
            r5 = 1
            goto L_0x0537
        L_0x0383:
            int r1 = r15.modifiers
            boolean r1 = java.lang.reflect.Modifier.isStatic(r1)
            if (r1 != 0) goto L_0x04f6
            java.lang.Class<org.bytedeco.javacpp.Pointer> r1 = org.bytedeco.javacpp.Pointer.class
            boolean r1 = r1.isAssignableFrom(r7)
            if (r1 == 0) goto L_0x04f6
            boolean r1 = r15.allocator
            if (r1 != 0) goto L_0x04f6
            boolean r1 = r15.arrayAllocator
            if (r1 != 0) goto L_0x04f6
            boolean r1 = r15.deallocator
            if (r1 != 0) goto L_0x04f6
            java.lang.String[] r1 = r27.cppTypeName(r28)
            java.lang.String r2 = "void*"
            r3 = 0
            r4 = r1[r3]
            boolean r2 = r2.equals(r4)
            if (r2 == 0) goto L_0x03bb
            java.lang.Class<org.bytedeco.javacpp.annotation.Opaque> r2 = org.bytedeco.javacpp.annotation.Opaque.class
            boolean r2 = r7.isAnnotationPresent(r2)
            if (r2 != 0) goto L_0x03bb
            java.lang.String r2 = "char*"
            r1[r3] = r2
            goto L_0x03e5
        L_0x03bb:
            java.lang.Class<org.bytedeco.javacpp.FunctionPointer> r2 = org.bytedeco.javacpp.FunctionPointer.class
            boolean r2 = r2.isAssignableFrom(r7)
            if (r2 == 0) goto L_0x03e5
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r2 = r6.functions
            r2.index(r7)
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = functionClassName(r28)
            r2.append(r3)
            java.lang.String r3 = "*"
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r3 = 0
            r1[r3] = r2
            java.lang.String r2 = ""
            r3 = 1
            r1[r3] = r2
        L_0x03e5:
            java.io.PrintWriter r2 = r6.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = "    "
            r3.append(r4)
            r4 = 0
            r5 = r1[r4]
            r3.append(r5)
            java.lang.String r5 = " ptr"
            r3.append(r5)
            r5 = 1
            r4 = r1[r5]
            r3.append(r4)
            java.lang.String r4 = " = ("
            r3.append(r4)
            r26 = r0
            r4 = 0
            r0 = r1[r4]
            r3.append(r0)
            r0 = r1[r5]
            r3.append(r0)
            java.lang.String r0 = ")jlong_to_ptr(env->GetLongField(obj, JavaCPP_addressFID));"
            r3.append(r0)
            java.lang.String r0 = r3.toString()
            r2.println(r0)
            java.io.PrintWriter r0 = r6.out
            java.lang.String r2 = "    if (ptr == NULL) {"
            r0.println(r2)
            java.io.PrintWriter r0 = r6.out
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "        env->ThrowNew(JavaCPP_getClass(env, "
            r2.append(r3)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r3 = r6.jclasses
            java.lang.Class<java.lang.NullPointerException> r4 = java.lang.NullPointerException.class
            int r3 = r3.index(r4)
            r2.append(r3)
            java.lang.String r3 = "), \"This pointer address is NULL.\");"
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r0.println(r2)
            java.io.PrintWriter r0 = r6.out
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "        return"
            r2.append(r3)
            java.lang.Class<?> r3 = r15.returnType
            java.lang.Class r4 = java.lang.Void.TYPE
            if (r3 != r4) goto L_0x045f
            java.lang.String r3 = ";"
            goto L_0x0461
        L_0x045f:
            java.lang.String r3 = " 0;"
        L_0x0461:
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r0.println(r2)
            java.io.PrintWriter r0 = r6.out
            java.lang.String r2 = "    }"
            r0.println(r2)
            java.lang.Class<org.bytedeco.javacpp.FunctionPointer> r0 = org.bytedeco.javacpp.FunctionPointer.class
            boolean r0 = r0.isAssignableFrom(r7)
            if (r0 == 0) goto L_0x04cc
            java.io.PrintWriter r0 = r6.out
            java.lang.String r2 = "    if (ptr->ptr == NULL) {"
            r0.println(r2)
            java.io.PrintWriter r0 = r6.out
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "        env->ThrowNew(JavaCPP_getClass(env, "
            r2.append(r3)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r3 = r6.jclasses
            java.lang.Class<java.lang.NullPointerException> r4 = java.lang.NullPointerException.class
            int r3 = r3.index(r4)
            r2.append(r3)
            java.lang.String r3 = "), \"This function pointer address is NULL.\");"
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r0.println(r2)
            java.io.PrintWriter r0 = r6.out
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "        return"
            r2.append(r3)
            java.lang.Class<?> r3 = r15.returnType
            java.lang.Class r4 = java.lang.Void.TYPE
            if (r3 != r4) goto L_0x04b9
            java.lang.String r3 = ";"
            goto L_0x04bb
        L_0x04b9:
            java.lang.String r3 = " 0;"
        L_0x04bb:
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r0.println(r2)
            java.io.PrintWriter r0 = r6.out
            java.lang.String r2 = "    }"
            r0.println(r2)
        L_0x04cc:
            java.lang.Class<org.bytedeco.javacpp.annotation.Opaque> r0 = org.bytedeco.javacpp.annotation.Opaque.class
            boolean r0 = r7.isAnnotationPresent(r0)
            if (r0 != 0) goto L_0x04f9
            java.io.PrintWriter r0 = r6.out
            java.lang.String r2 = "    jlong position = env->GetLongField(obj, JavaCPP_positionFID);"
            r0.println(r2)
            boolean r0 = r15.bufferGetter
            if (r0 == 0) goto L_0x04ee
            java.io.PrintWriter r0 = r6.out
            java.lang.String r2 = "    jlong limit = env->GetLongField(obj, JavaCPP_limitFID);"
            r0.println(r2)
            java.io.PrintWriter r0 = r6.out
            java.lang.String r2 = "    jlong capacity = env->GetLongField(obj, JavaCPP_capacityFID);"
            r0.println(r2)
            goto L_0x04f9
        L_0x04ee:
            java.io.PrintWriter r0 = r6.out
            java.lang.String r2 = "    ptr += position;"
            r0.println(r2)
            goto L_0x04f9
        L_0x04f6:
            r26 = r0
            r5 = 1
        L_0x04f9:
            r6.parametersBefore(r15)
            java.lang.String r0 = r6.returnBefore(r15)
            r2 = 0
            r6.call(r15, r0, r2)
            r6.returnAfter(r15)
            r6.parametersAfter(r15)
            java.lang.Class<?> r1 = r15.throwsException
            if (r1 == 0) goto L_0x0523
            java.io.PrintWriter r1 = r6.out
            java.lang.String r3 = "    if (exc != NULL) {"
            r1.println(r3)
            java.io.PrintWriter r1 = r6.out
            java.lang.String r3 = "        env->Throw(exc);"
            r1.println(r3)
            java.io.PrintWriter r1 = r6.out
            java.lang.String r3 = "    }"
            r1.println(r3)
        L_0x0523:
            java.lang.Class<?> r1 = r15.returnType
            java.lang.Class r3 = java.lang.Void.TYPE
            if (r1 == r3) goto L_0x0530
            java.io.PrintWriter r1 = r6.out
            java.lang.String r3 = "    return rarg;"
            r1.println(r3)
        L_0x0530:
            java.io.PrintWriter r1 = r6.out
            java.lang.String r3 = "}"
            r1.println(r3)
        L_0x0537:
            r1 = r17
            r3 = r26
        L_0x053b:
            int r0 = r14 + 1
            r11 = r24
            r12 = r25
            goto L_0x0118
        L_0x0543:
            r24 = r11
            r25 = r12
            java.io.PrintWriter r0 = r6.out
            r0.println()
            return r3
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.Generator.methods(java.lang.Class):boolean");
    }

    /*  JADX ERROR: JadxRuntimeException in pass: InitCodeVariables
        jadx.core.utils.exceptions.JadxRuntimeException: Several immutable types in one variable: [int, boolean], vars: [r5v0 ?, r5v1 ?, r5v3 ?, r5v2 ?, r5v14 ?, r5v7 ?, r5v111 ?]
        	at jadx.core.dex.visitors.InitCodeVariables.setCodeVarType(InitCodeVariables.java:102)
        	at jadx.core.dex.visitors.InitCodeVariables.setCodeVar(InitCodeVariables.java:78)
        	at jadx.core.dex.visitors.InitCodeVariables.initCodeVar(InitCodeVariables.java:69)
        	at jadx.core.dex.visitors.InitCodeVariables.initCodeVars(InitCodeVariables.java:51)
        	at jadx.core.dex.visitors.InitCodeVariables.visit(InitCodeVariables.java:32)
        */
    void parametersBefore(org.bytedeco.javacpp.tools.MethodInformation r18) {
        /*
            r17 = this;
            r0 = r17
            r1 = r18
            java.lang.String r2 = ""
            r3 = 0
            java.lang.Class<?>[] r4 = r1.parameterTypes
            int r4 = r4.length
            r5 = 1
            r6 = 0
            if (r4 <= 0) goto L_0x0018
            java.lang.Class<?>[] r4 = r1.parameterTypes
            r4 = r4[r6]
            java.lang.Class<java.lang.Class> r7 = java.lang.Class.class
            if (r4 != r7) goto L_0x0018
            r4 = 1
            goto L_0x0019
        L_0x0018:
            r4 = 0
        L_0x0019:
            r7 = r3
            r3 = r2
            r2 = r4
        L_0x001c:
            java.lang.Class<?>[] r8 = r1.parameterTypes
            int r8 = r8.length
            if (r2 >= r8) goto L_0x08fa
            java.lang.Class<?>[] r8 = r1.parameterTypes
            r8 = r8[r2]
            boolean r8 = r8.isPrimitive()
            if (r8 != 0) goto L_0x08f2
            java.lang.annotation.Annotation r8 = r0.by(r1, r2)
            java.lang.String r9 = r0.cast((org.bytedeco.javacpp.tools.MethodInformation) r1, (int) r2)
            boolean[] r10 = r1.parameterRaw
            boolean r10 = r10[r2]
            if (r10 == 0) goto L_0x0040
            java.lang.String[] r10 = new java.lang.String[r5]
            java.lang.String r11 = ""
            r10[r6] = r11
            goto L_0x0048
        L_0x0040:
            java.lang.Class<?>[] r10 = r1.parameterTypes
            r10 = r10[r2]
            java.lang.String[] r10 = r0.cppTypeName(r10)
        L_0x0048:
            boolean[] r11 = r1.parameterRaw
            boolean r11 = r11[r2]
            if (r11 == 0) goto L_0x0050
            r11 = 0
            goto L_0x0054
        L_0x0050:
            org.bytedeco.javacpp.tools.AdapterInformation r11 = r0.adapterInformation((boolean) r6, (org.bytedeco.javacpp.tools.MethodInformation) r1, (int) r2)
        L_0x0054:
            java.lang.Class<java.lang.Enum> r12 = java.lang.Enum.class
            java.lang.Class<?>[] r13 = r1.parameterTypes
            r13 = r13[r2]
            boolean r12 = r12.isAssignableFrom(r13)
            if (r12 == 0) goto L_0x0150
            r0.accessesEnums = r5
            java.lang.Class<?>[] r12 = r1.parameterTypes
            r12 = r12[r2]
            java.lang.String r12 = r0.enumValueType(r12)
            if (r12 == 0) goto L_0x08f2
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            char r14 = r12.charAt(r6)
            char r14 = java.lang.Character.toUpperCase(r14)
            r13.append(r14)
            java.lang.String r14 = r12.substring(r5)
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            java.io.PrintWriter r14 = r0.out
            java.lang.StringBuilder r15 = new java.lang.StringBuilder
            r15.<init>()
            java.lang.String r5 = "    if (arg"
            r15.append(r5)
            r15.append(r2)
            java.lang.String r5 = " == NULL) {"
            r15.append(r5)
            java.lang.String r5 = r15.toString()
            r14.println(r5)
            java.io.PrintWriter r5 = r0.out
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            java.lang.String r15 = "        env->ThrowNew(JavaCPP_getClass(env, "
            r14.append(r15)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r15 = r0.jclasses
            java.lang.Class<java.lang.NullPointerException> r6 = java.lang.NullPointerException.class
            int r6 = r15.index(r6)
            r14.append(r6)
            java.lang.String r6 = "), \"Enum for argument "
            r14.append(r6)
            r14.append(r2)
            java.lang.String r6 = " is NULL.\");"
            r14.append(r6)
            java.lang.String r6 = r14.toString()
            r5.println(r6)
            java.io.PrintWriter r5 = r0.out
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r14 = "        return"
            r6.append(r14)
            java.lang.Class<?> r14 = r1.returnType
            java.lang.Class r15 = java.lang.Void.TYPE
            if (r14 != r15) goto L_0x00e2
            java.lang.String r14 = ";"
            goto L_0x00e4
        L_0x00e2:
            java.lang.String r14 = " 0;"
        L_0x00e4:
            r6.append(r14)
            java.lang.String r6 = r6.toString()
            r5.println(r6)
            java.io.PrintWriter r5 = r0.out
            java.lang.String r6 = "    }"
            r5.println(r6)
            java.io.PrintWriter r5 = r0.out
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r14 = "    "
            r6.append(r14)
            r14 = 0
            r15 = r10[r14]
            r6.append(r15)
            java.lang.String r15 = " val"
            r6.append(r15)
            r6.append(r2)
            r15 = 1
            r14 = r10[r15]
            r6.append(r14)
            java.lang.String r14 = " = ("
            r6.append(r14)
            r16 = r3
            r14 = 0
            r3 = r10[r14]
            r6.append(r3)
            r3 = r10[r15]
            r6.append(r3)
            java.lang.String r3 = ")env->Get"
            r6.append(r3)
            r6.append(r13)
            java.lang.String r3 = "Field(arg"
            r6.append(r3)
            r6.append(r2)
            java.lang.String r3 = ", JavaCPP_"
            r6.append(r3)
            r6.append(r12)
            java.lang.String r3 = "ValueFID);"
            r6.append(r3)
            java.lang.String r3 = r6.toString()
            r5.println(r3)
            r5 = 1
            r6 = 0
            goto L_0x08f4
        L_0x0150:
            r16 = r3
            java.lang.Class<org.bytedeco.javacpp.FunctionPointer> r3 = org.bytedeco.javacpp.FunctionPointer.class
            java.lang.Class<?>[] r5 = r1.parameterTypes
            r5 = r5[r2]
            boolean r3 = r3.isAssignableFrom(r5)
            if (r3 == 0) goto L_0x01ae
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r3 = r0.functions
            java.lang.Class<?>[] r5 = r1.parameterTypes
            r5 = r5[r2]
            r3.index(r5)
            java.lang.Class<?>[] r3 = r1.parameterTypes
            r3 = r3[r2]
            java.lang.Class<org.bytedeco.javacpp.FunctionPointer> r5 = org.bytedeco.javacpp.FunctionPointer.class
            if (r3 != r5) goto L_0x018c
            org.bytedeco.javacpp.tools.Logger r3 = r0.logger
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "Method \""
            r5.append(r6)
            java.lang.reflect.Method r6 = r1.method
            r5.append(r6)
            java.lang.String r6 = "\" has an abstract FunctionPointer parameter, but a concrete subclass is required. Compilation will most likely fail."
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r3.warn(r5)
        L_0x018c:
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.Class<?>[] r5 = r1.parameterTypes
            r5 = r5[r2]
            java.lang.String r5 = functionClassName(r5)
            r3.append(r5)
            java.lang.String r5 = "*"
            r3.append(r5)
            java.lang.String r3 = r3.toString()
            r5 = 0
            r10[r5] = r3
            java.lang.String r3 = ""
            r6 = 1
            r10[r6] = r3
            goto L_0x01af
        L_0x01ae:
            r5 = 0
        L_0x01af:
            r3 = r10[r5]
            int r3 = r3.length()
            if (r3 == 0) goto L_0x08b4
            boolean[] r3 = r1.parameterRaw
            boolean r3 = r3[r2]
            if (r3 == 0) goto L_0x01bf
            goto L_0x08b4
        L_0x01bf:
            java.lang.String r3 = "void*"
            r6 = r10[r5]
            boolean r3 = r3.equals(r6)
            if (r3 == 0) goto L_0x01d9
            java.lang.Class<?>[] r3 = r1.parameterTypes
            r3 = r3[r2]
            java.lang.Class<org.bytedeco.javacpp.annotation.Opaque> r6 = org.bytedeco.javacpp.annotation.Opaque.class
            boolean r3 = r3.isAnnotationPresent(r6)
            if (r3 != 0) goto L_0x01d9
            java.lang.String r3 = "char*"
            r10[r5] = r3
        L_0x01d9:
            java.io.PrintWriter r3 = r0.out
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r12 = "    "
            r6.append(r12)
            r12 = r10[r5]
            r6.append(r12)
            java.lang.String r5 = " ptr"
            r6.append(r5)
            r6.append(r2)
            r5 = 1
            r12 = r10[r5]
            r6.append(r12)
            java.lang.String r5 = " = "
            r6.append(r5)
            java.lang.String r5 = r6.toString()
            r3.print(r5)
            java.lang.Class<org.bytedeco.javacpp.Pointer> r3 = org.bytedeco.javacpp.Pointer.class
            java.lang.Class<?>[] r5 = r1.parameterTypes
            r5 = r5[r2]
            boolean r3 = r3.isAssignableFrom(r5)
            if (r3 == 0) goto L_0x03bf
            java.io.PrintWriter r3 = r0.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "arg"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = " == NULL ? NULL : ("
            r5.append(r6)
            r6 = 0
            r12 = r10[r6]
            r5.append(r12)
            r6 = 1
            r12 = r10[r6]
            r5.append(r12)
            java.lang.String r6 = ")jlong_to_ptr(env->GetLongField(arg"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = ", JavaCPP_addressFID));"
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r3.println(r5)
            if (r2 != 0) goto L_0x025a
            java.lang.Class<org.bytedeco.javacpp.FunctionPointer> r3 = org.bytedeco.javacpp.FunctionPointer.class
            java.lang.Class<?> r5 = r1.cls
            boolean r3 = r3.isAssignableFrom(r5)
            if (r3 == 0) goto L_0x025a
            java.lang.Class<?> r3 = r1.cls
            java.lang.Class<org.bytedeco.javacpp.annotation.Namespace> r5 = org.bytedeco.javacpp.annotation.Namespace.class
            boolean r3 = r3.isAnnotationPresent(r5)
            if (r3 != 0) goto L_0x027c
        L_0x025a:
            boolean r3 = r8 instanceof org.bytedeco.javacpp.annotation.ByVal
            if (r3 == 0) goto L_0x026b
            r3 = r8
            org.bytedeco.javacpp.annotation.ByVal r3 = (org.bytedeco.javacpp.annotation.ByVal) r3
            java.lang.String r3 = r3.nullValue()
            int r3 = r3.length()
            if (r3 == 0) goto L_0x027c
        L_0x026b:
            boolean r3 = r8 instanceof org.bytedeco.javacpp.annotation.ByRef
            if (r3 == 0) goto L_0x02ea
            r3 = r8
            org.bytedeco.javacpp.annotation.ByRef r3 = (org.bytedeco.javacpp.annotation.ByRef) r3
            java.lang.String r3 = r3.nullValue()
            int r3 = r3.length()
            if (r3 != 0) goto L_0x02ea
        L_0x027c:
            java.io.PrintWriter r3 = r0.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "    if (ptr"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = " == NULL) {"
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r3.println(r5)
            java.io.PrintWriter r3 = r0.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "        env->ThrowNew(JavaCPP_getClass(env, "
            r5.append(r6)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r6 = r0.jclasses
            java.lang.Class<java.lang.NullPointerException> r12 = java.lang.NullPointerException.class
            int r6 = r6.index(r12)
            r5.append(r6)
            java.lang.String r6 = "), \"Pointer address of argument "
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = " is NULL.\");"
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r3.println(r5)
            java.io.PrintWriter r3 = r0.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "        return"
            r5.append(r6)
            java.lang.Class<?> r6 = r1.returnType
            java.lang.Class r12 = java.lang.Void.TYPE
            if (r6 != r12) goto L_0x02d7
            java.lang.String r6 = ";"
            goto L_0x02d9
        L_0x02d7:
            java.lang.String r6 = " 0;"
        L_0x02d9:
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r3.println(r5)
            java.io.PrintWriter r3 = r0.out
            java.lang.String r5 = "    }"
            r3.println(r5)
        L_0x02ea:
            if (r11 != 0) goto L_0x02ee
            if (r7 == 0) goto L_0x033c
        L_0x02ee:
            java.io.PrintWriter r3 = r0.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "    jlong size"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = " = arg"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = " == NULL ? 0 : env->GetLongField(arg"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = ", JavaCPP_limitFID);"
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r3.println(r5)
            java.io.PrintWriter r3 = r0.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "    void* owner"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = " = JavaCPP_getPointerOwner(env, arg"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = ");"
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r3.println(r5)
        L_0x033c:
            java.lang.Class<?>[] r3 = r1.parameterTypes
            r3 = r3[r2]
            java.lang.Class<org.bytedeco.javacpp.annotation.Opaque> r5 = org.bytedeco.javacpp.annotation.Opaque.class
            boolean r3 = r3.isAnnotationPresent(r5)
            if (r3 != 0) goto L_0x0812
            java.io.PrintWriter r3 = r0.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "    jlong position"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = " = arg"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = " == NULL ? 0 : env->GetLongField(arg"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = ", JavaCPP_positionFID);"
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r3.println(r5)
            java.io.PrintWriter r3 = r0.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "    ptr"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = " += position"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = ";"
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r3.println(r5)
            if (r11 != 0) goto L_0x039a
            if (r7 == 0) goto L_0x0812
        L_0x039a:
            java.io.PrintWriter r3 = r0.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "    size"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = " -= position"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = ";"
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r3.println(r5)
            goto L_0x0812
        L_0x03bf:
            java.lang.Class<?>[] r3 = r1.parameterTypes
            r3 = r3[r2]
            java.lang.Class<java.lang.String> r5 = java.lang.String.class
            if (r3 != r5) goto L_0x0429
            r3 = 1
            r0.passesStrings = r3
            java.io.PrintWriter r3 = r0.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "JavaCPP_getStringBytes(env, arg"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = ");"
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r3.println(r5)
            if (r11 != 0) goto L_0x03e9
            if (r7 == 0) goto L_0x0812
        L_0x03e9:
            java.io.PrintWriter r3 = r0.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "    jlong size"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = " = 0;"
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r3.println(r5)
            java.io.PrintWriter r3 = r0.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "    void* owner"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = " = (void*)ptr"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = ";"
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r3.println(r5)
            goto L_0x0812
        L_0x0429:
            java.lang.Class<?>[] r3 = r1.parameterTypes
            r3 = r3[r2]
            boolean r3 = r3.isArray()
            if (r3 == 0) goto L_0x0535
            java.lang.Class<?>[] r3 = r1.parameterTypes
            r3 = r3[r2]
            java.lang.Class r3 = r3.getComponentType()
            boolean r3 = r3.isPrimitive()
            if (r3 == 0) goto L_0x0535
            java.io.PrintWriter r3 = r0.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "arg"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = " == NULL ? NULL : "
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r3.print(r5)
            java.lang.Class<?>[] r3 = r1.parameterTypes
            r3 = r3[r2]
            java.lang.Class r3 = r3.getComponentType()
            java.lang.String r3 = r3.getName()
            boolean r5 = r1.criticalRegion
            if (r5 != 0) goto L_0x04be
            boolean r5 = r1.valueGetter
            if (r5 != 0) goto L_0x04be
            boolean r5 = r1.valueSetter
            if (r5 != 0) goto L_0x04be
            boolean r5 = r1.memberGetter
            if (r5 != 0) goto L_0x04be
            boolean r5 = r1.memberSetter
            if (r5 == 0) goto L_0x047d
            goto L_0x04be
        L_0x047d:
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            r6 = 0
            char r12 = r3.charAt(r6)
            char r6 = java.lang.Character.toUpperCase(r12)
            r5.append(r6)
            r6 = 1
            java.lang.String r12 = r3.substring(r6)
            r5.append(r12)
            java.lang.String r3 = r5.toString()
            java.io.PrintWriter r5 = r0.out
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r12 = "env->Get"
            r6.append(r12)
            r6.append(r3)
            java.lang.String r12 = "ArrayElements(arg"
            r6.append(r12)
            r6.append(r2)
            java.lang.String r12 = ", NULL);"
            r6.append(r12)
            java.lang.String r6 = r6.toString()
            r5.println(r6)
            goto L_0x04e1
        L_0x04be:
            java.io.PrintWriter r5 = r0.out
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r12 = "(j"
            r6.append(r12)
            r6.append(r3)
            java.lang.String r12 = "*)env->GetPrimitiveArrayCritical(arg"
            r6.append(r12)
            r6.append(r2)
            java.lang.String r12 = ", NULL);"
            r6.append(r12)
            java.lang.String r6 = r6.toString()
            r5.println(r6)
        L_0x04e1:
            if (r11 != 0) goto L_0x04e5
            if (r7 == 0) goto L_0x0533
        L_0x04e5:
            java.io.PrintWriter r5 = r0.out
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r12 = "    jlong size"
            r6.append(r12)
            r6.append(r2)
            java.lang.String r12 = " = arg"
            r6.append(r12)
            r6.append(r2)
            java.lang.String r12 = " == NULL ? 0 : env->GetArrayLength(arg"
            r6.append(r12)
            r6.append(r2)
            java.lang.String r12 = ");"
            r6.append(r12)
            java.lang.String r6 = r6.toString()
            r5.println(r6)
            java.io.PrintWriter r5 = r0.out
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r12 = "    void* owner"
            r6.append(r12)
            r6.append(r2)
            java.lang.String r12 = " = (void*)ptr"
            r6.append(r12)
            r6.append(r2)
            java.lang.String r12 = ";"
            r6.append(r12)
            java.lang.String r6 = r6.toString()
            r5.println(r6)
        L_0x0533:
            goto L_0x0812
        L_0x0535:
            java.lang.Class<java.nio.Buffer> r3 = java.nio.Buffer.class
            java.lang.Class<?>[] r5 = r1.parameterTypes
            r5 = r5[r2]
            boolean r3 = r3.isAssignableFrom(r5)
            if (r3 == 0) goto L_0x07ca
            java.io.PrintWriter r3 = r0.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "arg"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = " == NULL ? NULL : ("
            r5.append(r6)
            r6 = 0
            r12 = r10[r6]
            r5.append(r12)
            r6 = 1
            r12 = r10[r6]
            r5.append(r12)
            java.lang.String r6 = ")env->GetDirectBufferAddress(arg"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = ");"
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r3.println(r5)
            if (r11 != 0) goto L_0x0579
            if (r7 == 0) goto L_0x05c7
        L_0x0579:
            java.io.PrintWriter r3 = r0.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "    jlong size"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = " = arg"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = " == NULL ? 0 : env->GetIntField(arg"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = ", JavaCPP_bufferLimitFID);"
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r3.println(r5)
            java.io.PrintWriter r3 = r0.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "    void* owner"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = " = (void*)ptr"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = ";"
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r3.println(r5)
        L_0x05c7:
            java.lang.Class<?>[] r3 = r1.parameterTypes
            r3 = r3[r2]
            java.lang.Class<java.nio.Buffer> r5 = java.nio.Buffer.class
            if (r3 == r5) goto L_0x0754
            java.lang.Class<?>[] r3 = r1.parameterTypes
            r3 = r3[r2]
            java.lang.String r3 = r3.getSimpleName()
            int r5 = r3.length()
            int r5 = r5 + -6
            r6 = 0
            java.lang.String r3 = r3.substring(r6, r5)
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            char r12 = r3.charAt(r6)
            char r6 = java.lang.Character.toLowerCase(r12)
            r5.append(r6)
            r6 = 1
            java.lang.String r12 = r3.substring(r6)
            r5.append(r12)
            java.lang.String r5 = r5.toString()
            java.io.PrintWriter r6 = r0.out
            java.lang.StringBuilder r12 = new java.lang.StringBuilder
            r12.<init>()
            java.lang.String r13 = "    j"
            r12.append(r13)
            r12.append(r5)
            java.lang.String r13 = "Array arr"
            r12.append(r13)
            r12.append(r2)
            java.lang.String r13 = " = NULL;"
            r12.append(r13)
            java.lang.String r12 = r12.toString()
            r6.println(r12)
            java.io.PrintWriter r6 = r0.out
            java.lang.StringBuilder r12 = new java.lang.StringBuilder
            r12.<init>()
            java.lang.String r13 = "    jlong offset"
            r12.append(r13)
            r12.append(r2)
            java.lang.String r13 = " = 0;"
            r12.append(r13)
            java.lang.String r12 = r12.toString()
            r6.println(r12)
            java.io.PrintWriter r6 = r0.out
            java.lang.StringBuilder r12 = new java.lang.StringBuilder
            r12.<init>()
            java.lang.String r13 = "    if (arg"
            r12.append(r13)
            r12.append(r2)
            java.lang.String r13 = " != NULL && ptr"
            r12.append(r13)
            r12.append(r2)
            java.lang.String r13 = " == NULL) {"
            r12.append(r13)
            java.lang.String r12 = r12.toString()
            r6.println(r12)
            java.io.PrintWriter r6 = r0.out
            java.lang.StringBuilder r12 = new java.lang.StringBuilder
            r12.<init>()
            java.lang.String r13 = "        arr"
            r12.append(r13)
            r12.append(r2)
            java.lang.String r13 = " = (j"
            r12.append(r13)
            r12.append(r5)
            java.lang.String r13 = "Array)env->CallObjectMethod(arg"
            r12.append(r13)
            r12.append(r2)
            java.lang.String r13 = ", JavaCPP_arrayMID);"
            r12.append(r13)
            java.lang.String r12 = r12.toString()
            r6.println(r12)
            java.io.PrintWriter r6 = r0.out
            java.lang.StringBuilder r12 = new java.lang.StringBuilder
            r12.<init>()
            java.lang.String r13 = "        offset"
            r12.append(r13)
            r12.append(r2)
            java.lang.String r13 = " = env->CallIntMethod(arg"
            r12.append(r13)
            r12.append(r2)
            java.lang.String r13 = ", JavaCPP_arrayOffsetMID);"
            r12.append(r13)
            java.lang.String r12 = r12.toString()
            r6.println(r12)
            java.io.PrintWriter r6 = r0.out
            java.lang.String r12 = "        if (env->ExceptionOccurred() != NULL) {"
            r6.println(r12)
            java.io.PrintWriter r6 = r0.out
            java.lang.String r12 = "            env->ExceptionClear();"
            r6.println(r12)
            java.io.PrintWriter r6 = r0.out
            java.lang.String r12 = "        } else {"
            r6.println(r12)
            boolean r6 = r1.criticalRegion
            if (r6 == 0) goto L_0x070b
            java.io.PrintWriter r6 = r0.out
            java.lang.StringBuilder r12 = new java.lang.StringBuilder
            r12.<init>()
            java.lang.String r13 = "            ptr"
            r12.append(r13)
            r12.append(r2)
            java.lang.String r13 = " = arr"
            r12.append(r13)
            r12.append(r2)
            java.lang.String r13 = " == NULL ? NULL : ("
            r12.append(r13)
            r13 = 0
            r14 = r10[r13]
            r12.append(r14)
            r13 = 1
            r14 = r10[r13]
            r12.append(r14)
            java.lang.String r13 = ")env->GetPrimitiveArrayCritical(arr"
            r12.append(r13)
            r12.append(r2)
            java.lang.String r13 = ", NULL) + offset"
            r12.append(r13)
            r12.append(r2)
            java.lang.String r13 = ";"
            r12.append(r13)
            java.lang.String r12 = r12.toString()
            r6.println(r12)
            goto L_0x0746
        L_0x070b:
            java.io.PrintWriter r6 = r0.out
            java.lang.StringBuilder r12 = new java.lang.StringBuilder
            r12.<init>()
            java.lang.String r13 = "            ptr"
            r12.append(r13)
            r12.append(r2)
            java.lang.String r13 = " = arr"
            r12.append(r13)
            r12.append(r2)
            java.lang.String r13 = " == NULL ? NULL : env->Get"
            r12.append(r13)
            r12.append(r3)
            java.lang.String r13 = "ArrayElements(arr"
            r12.append(r13)
            r12.append(r2)
            java.lang.String r13 = ", NULL) + offset"
            r12.append(r13)
            r12.append(r2)
            java.lang.String r13 = ";"
            r12.append(r13)
            java.lang.String r12 = r12.toString()
            r6.println(r12)
        L_0x0746:
            java.io.PrintWriter r6 = r0.out
            java.lang.String r12 = "        }"
            r6.println(r12)
            java.io.PrintWriter r6 = r0.out
            java.lang.String r12 = "    }"
            r6.println(r12)
        L_0x0754:
            java.io.PrintWriter r3 = r0.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "    jlong position"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = " = arg"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = " == NULL ? 0 : env->GetIntField(arg"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = ", JavaCPP_bufferPositionFID);"
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r3.println(r5)
            java.io.PrintWriter r3 = r0.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "    ptr"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = " += position"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = ";"
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r3.println(r5)
            if (r11 != 0) goto L_0x07a6
            if (r7 == 0) goto L_0x0812
        L_0x07a6:
            java.io.PrintWriter r3 = r0.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "    size"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = " -= position"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = ";"
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r3.println(r5)
            goto L_0x0812
        L_0x07ca:
            java.io.PrintWriter r3 = r0.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "arg"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = ";"
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r3.println(r5)
            org.bytedeco.javacpp.tools.Logger r3 = r0.logger
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "Method \""
            r5.append(r6)
            java.lang.reflect.Method r6 = r1.method
            r5.append(r6)
            java.lang.String r6 = "\" has an unsupported parameter of type \""
            r5.append(r6)
            java.lang.Class<?>[] r6 = r1.parameterTypes
            r6 = r6[r2]
            java.lang.String r6 = r6.getCanonicalName()
            r5.append(r6)
            java.lang.String r6 = "\". Compilation will most likely fail."
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r3.warn(r5)
        L_0x0812:
            if (r11 == 0) goto L_0x0839
            r3 = 1
            r0.usesAdapters = r3
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r5 = "    "
            r3.append(r5)
            java.lang.String r5 = r11.name
            r3.append(r5)
            java.lang.String r5 = " adapter"
            r3.append(r5)
            r3.append(r2)
            java.lang.String r5 = "("
            r3.append(r5)
            java.lang.String r3 = r3.toString()
            r7 = r11
            goto L_0x083b
        L_0x0839:
            r3 = r16
        L_0x083b:
            if (r7 == 0) goto L_0x0893
            java.lang.Class<org.bytedeco.javacpp.FunctionPointer> r5 = org.bytedeco.javacpp.FunctionPointer.class
            java.lang.Class<?> r6 = r1.cls
            boolean r5 = r5.isAssignableFrom(r6)
            if (r5 != 0) goto L_0x0856
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            r5.append(r3)
            r5.append(r9)
            java.lang.String r3 = r5.toString()
        L_0x0856:
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            r5.append(r3)
            java.lang.String r6 = "ptr"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = ", size"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r6 = ", owner"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r3 = r5.toString()
            int r5 = r7.argc
            r6 = 1
            int r5 = r5 - r6
            r7.argc = r5
            if (r5 <= 0) goto L_0x0893
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            r5.append(r3)
            java.lang.String r6 = ", "
            r5.append(r6)
            java.lang.String r3 = r5.toString()
        L_0x0893:
            if (r7 == 0) goto L_0x08b1
            int r5 = r7.argc
            if (r5 > 0) goto L_0x08b1
            java.io.PrintWriter r5 = r0.out
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            r6.append(r3)
            java.lang.String r12 = ");"
            r6.append(r12)
            java.lang.String r6 = r6.toString()
            r5.println(r6)
            r5 = 0
            r7 = r5
        L_0x08b1:
            r5 = 1
            r6 = 0
            goto L_0x08f6
        L_0x08b4:
            boolean[] r3 = r1.parameterRaw
            r5 = 1
            r3[r2] = r5
            java.lang.Class<?>[] r3 = r1.parameterTypes
            r3 = r3[r2]
            java.lang.String r3 = jniTypeName(r3)
            r6 = 0
            r10[r6] = r3
            java.io.PrintWriter r3 = r0.out
            java.lang.StringBuilder r12 = new java.lang.StringBuilder
            r12.<init>()
            java.lang.String r13 = "    "
            r12.append(r13)
            r13 = r10[r6]
            r12.append(r13)
            java.lang.String r13 = " ptr"
            r12.append(r13)
            r12.append(r2)
            java.lang.String r13 = " = arg"
            r12.append(r13)
            r12.append(r2)
            java.lang.String r13 = ";"
            r12.append(r13)
            java.lang.String r12 = r12.toString()
            r3.println(r12)
            goto L_0x08f4
        L_0x08f2:
            r16 = r3
        L_0x08f4:
            r3 = r16
        L_0x08f6:
            int r2 = r2 + 1
            goto L_0x001c
        L_0x08fa:
            r16 = r3
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.Generator.parametersBefore(org.bytedeco.javacpp.tools.MethodInformation):void");
    }

    /* access modifiers changed from: package-private */
    public String returnBefore(MethodInformation methodInfo) {
        String returnPrefix = "";
        if (methodInfo.returnType != Void.TYPE) {
            String cast = cast(methodInfo.returnType, methodInfo.annotations);
            String[] typeName = methodInfo.returnRaw ? new String[]{""} : cppCastTypeName(methodInfo.returnType, methodInfo.annotations);
            Annotation returnBy = by(methodInfo.annotations);
            if (FunctionPointer.class.isAssignableFrom(methodInfo.cls) && !methodInfo.cls.isAnnotationPresent(Namespace.class) && methodInfo.valueGetter) {
                typeName = cppTypeName(methodInfo.cls);
            }
            if (methodInfo.valueSetter || methodInfo.memberSetter || methodInfo.noReturnGetter) {
                this.out.println("    jobject rarg = obj;");
            } else if (methodInfo.returnType.isPrimitive()) {
                this.out.println("    " + jniTypeName(methodInfo.returnType) + " rarg = 0;");
                returnPrefix = typeName[0] + " rval" + typeName[1] + " = " + cast;
                if ((returnBy instanceof ByPtr) || (returnBy instanceof ByPtrRef)) {
                    returnPrefix = returnPrefix + "*";
                }
            } else if (Enum.class.isAssignableFrom(methodInfo.returnType)) {
                this.accessesEnums = true;
                this.out.println("    jobject rarg = JavaCPP_createPointer(env, " + this.jclasses.index(methodInfo.returnType) + ");");
                returnPrefix = typeName[0] + " rval" + typeName[1] + " = " + cast;
            } else {
                String valueTypeName = valueTypeName(typeName);
                returnPrefix = "rptr = " + cast;
                if (typeName[0].length() == 0 || methodInfo.returnRaw) {
                    methodInfo.returnRaw = true;
                    typeName[0] = jniTypeName(methodInfo.returnType);
                    this.out.println("    " + typeName[0] + " rarg = NULL;");
                    this.out.println("    " + typeName[0] + " rptr;");
                } else if (Pointer.class.isAssignableFrom(methodInfo.returnType) || Buffer.class.isAssignableFrom(methodInfo.returnType) || (methodInfo.returnType.isArray() && methodInfo.returnType.getComponentType().isPrimitive())) {
                    if (FunctionPointer.class.isAssignableFrom(methodInfo.returnType)) {
                        this.functions.index(methodInfo.returnType);
                        returnPrefix = "if (rptr != NULL) rptr->ptr = ";
                        if (methodInfo.method.isAnnotationPresent(Virtual.class)) {
                            returnPrefix = returnPrefix + "(" + typeName[0] + typeName[1] + ")&";
                        }
                        typeName[0] = functionClassName(methodInfo.returnType) + "*";
                        typeName[1] = "";
                        valueTypeName = valueTypeName(typeName);
                    }
                    if (returnBy instanceof ByVal) {
                        StringBuilder sb = new StringBuilder();
                        sb.append(returnPrefix);
                        sb.append(noException(methodInfo.returnType, methodInfo.method) ? "new (std::nothrow) " : "new ");
                        sb.append(valueTypeName);
                        sb.append(typeName[1]);
                        sb.append("(");
                        returnPrefix = sb.toString();
                    } else if (returnBy instanceof ByRef) {
                        returnPrefix = returnPrefix + "&";
                    } else if (returnBy instanceof ByPtrPtr) {
                        if (cast.length() > 0) {
                            typeName[0] = typeName[0].substring(0, typeName[0].length() - 1);
                        }
                        returnPrefix = "rptr = NULL; " + typeName[0] + "* rptrptr" + typeName[1] + " = " + cast;
                    }
                    if (methodInfo.bufferGetter) {
                        this.out.println("    jobject rarg = NULL;");
                        this.out.println("    char* rptr;");
                    } else {
                        this.out.println("    " + jniTypeName(methodInfo.returnType) + " rarg = NULL;");
                        this.out.println("    " + typeName[0] + " rptr" + typeName[1] + ";");
                    }
                    if (FunctionPointer.class.isAssignableFrom(methodInfo.returnType)) {
                        this.out.println("    rptr = new (std::nothrow) " + valueTypeName + ";");
                    }
                } else if (methodInfo.returnType == String.class) {
                    this.out.println("    jstring rarg = NULL;");
                    this.out.println("    const char* rptr;");
                    if (returnBy instanceof ByRef) {
                        returnPrefix = "std::string rstr(";
                    } else if (returnBy instanceof ByPtrPtr) {
                        returnPrefix = "rptr = NULL; const char** rptrptr = (const char**)";
                    } else {
                        returnPrefix = returnPrefix + "(const char*)";
                    }
                } else {
                    this.logger.warn("Method \"" + methodInfo.method + "\" has unsupported return type \"" + methodInfo.returnType.getCanonicalName() + "\". Compilation will most likely fail.");
                }
                AdapterInformation adapterInfo = adapterInformation(false, valueTypeName, methodInfo.annotations);
                if (adapterInfo != null) {
                    this.usesAdapters = true;
                    returnPrefix = adapterInfo.name + " radapter(";
                }
            }
        } else if (methodInfo.allocator || methodInfo.arrayAllocator) {
            this.jclasses.index(methodInfo.cls);
            String[] typeName2 = cppTypeName(methodInfo.cls);
            returnPrefix = typeName2[0] + " rptr" + typeName2[1] + " = ";
        }
        if (methodInfo.throwsException != null) {
            this.out.println("    jthrowable exc = NULL;");
            this.out.println("    try {");
        }
        return returnPrefix;
    }

    /* JADX WARNING: type inference failed for: r12v128, types: [java.lang.annotation.Annotation] */
    /* access modifiers changed from: package-private */
    /* JADX WARNING: Multi-variable type inference failed */
    /* JADX WARNING: Removed duplicated region for block: B:193:0x0508  */
    /* JADX WARNING: Removed duplicated region for block: B:310:0x085d  */
    /* JADX WARNING: Removed duplicated region for block: B:312:0x0868  */
    /* JADX WARNING: Removed duplicated region for block: B:320:? A[RETURN, SYNTHETIC] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public void call(org.bytedeco.javacpp.tools.MethodInformation r18, java.lang.String r19, boolean r20) {
        /*
            r17 = this;
            r0 = r17
            r1 = r18
            r2 = r19
            r4 = 0
            if (r20 == 0) goto L_0x000c
            java.lang.String r5 = ""
            goto L_0x0015
        L_0x000c:
            java.lang.Class<?> r5 = r1.throwsException
            if (r5 == 0) goto L_0x0013
            java.lang.String r5 = "        "
            goto L_0x0015
        L_0x0013:
            java.lang.String r5 = "    "
        L_0x0015:
            java.lang.String r6 = "("
            java.lang.String r7 = ")"
            java.lang.Class<?>[] r8 = r1.parameterTypes
            int r8 = r8.length
            r9 = 0
            r10 = 1
            if (r8 <= 0) goto L_0x002a
            java.lang.Class<?>[] r8 = r1.parameterTypes
            r8 = r8[r9]
            java.lang.Class<java.lang.Class> r11 = java.lang.Class.class
            if (r8 != r11) goto L_0x002a
            r8 = 1
            goto L_0x002b
        L_0x002a:
            r8 = 0
        L_0x002b:
            java.lang.reflect.Method r11 = r1.method
            java.lang.Class<org.bytedeco.javacpp.annotation.Index> r12 = org.bytedeco.javacpp.annotation.Index.class
            java.lang.annotation.Annotation r11 = r11.getAnnotation(r12)
            org.bytedeco.javacpp.annotation.Index r11 = (org.bytedeco.javacpp.annotation.Index) r11
            if (r11 != 0) goto L_0x0046
            java.lang.reflect.Method r12 = r1.pairedMethod
            if (r12 == 0) goto L_0x0046
            java.lang.reflect.Method r12 = r1.pairedMethod
            java.lang.Class<org.bytedeco.javacpp.annotation.Index> r13 = org.bytedeco.javacpp.annotation.Index.class
            java.lang.annotation.Annotation r12 = r12.getAnnotation(r13)
            r11 = r12
            org.bytedeco.javacpp.annotation.Index r11 = (org.bytedeco.javacpp.annotation.Index) r11
        L_0x0046:
            boolean r12 = r1.deallocator
            if (r12 == 0) goto L_0x00b9
            java.io.PrintWriter r9 = r0.out
            java.lang.StringBuilder r10 = new java.lang.StringBuilder
            r10.<init>()
            r10.append(r5)
            java.lang.String r12 = "void* allocatedAddress = jlong_to_ptr(arg0);"
            r10.append(r12)
            java.lang.String r10 = r10.toString()
            r9.println(r10)
            java.io.PrintWriter r9 = r0.out
            java.lang.StringBuilder r10 = new java.lang.StringBuilder
            r10.<init>()
            r10.append(r5)
            java.lang.String r12 = "void (*deallocatorAddress)(void*) = (void(*)(void*))jlong_to_ptr(arg1);"
            r10.append(r12)
            java.lang.String r10 = r10.toString()
            r9.println(r10)
            java.io.PrintWriter r9 = r0.out
            java.lang.StringBuilder r10 = new java.lang.StringBuilder
            r10.<init>()
            r10.append(r5)
            java.lang.String r12 = "if (deallocatorAddress != NULL && allocatedAddress != NULL) {"
            r10.append(r12)
            java.lang.String r10 = r10.toString()
            r9.println(r10)
            java.io.PrintWriter r9 = r0.out
            java.lang.StringBuilder r10 = new java.lang.StringBuilder
            r10.<init>()
            r10.append(r5)
            java.lang.String r12 = "    (*deallocatorAddress)(allocatedAddress);"
            r10.append(r12)
            java.lang.String r10 = r10.toString()
            r9.println(r10)
            java.io.PrintWriter r9 = r0.out
            java.lang.StringBuilder r10 = new java.lang.StringBuilder
            r10.<init>()
            r10.append(r5)
            java.lang.String r12 = "}"
            r10.append(r12)
            java.lang.String r10 = r10.toString()
            r9.println(r10)
            return
        L_0x00b9:
            java.lang.Class<org.bytedeco.javacpp.FunctionPointer> r12 = org.bytedeco.javacpp.FunctionPointer.class
            java.lang.Class<?> r13 = r1.cls
            boolean r12 = r12.isAssignableFrom(r13)
            if (r12 != 0) goto L_0x0275
            boolean r12 = r1.valueGetter
            if (r12 != 0) goto L_0x00d8
            boolean r12 = r1.valueSetter
            if (r12 != 0) goto L_0x00d8
            boolean r12 = r1.memberGetter
            if (r12 != 0) goto L_0x00d8
            boolean r12 = r1.memberSetter
            if (r12 == 0) goto L_0x00d4
            goto L_0x00d8
        L_0x00d4:
            r16 = r4
            goto L_0x0277
        L_0x00d8:
            r12 = 0
            java.lang.Class<?>[] r13 = r1.parameterTypes
            int r13 = r13.length
            int r13 = r13 - r10
            boolean r14 = r1.valueSetter
            if (r14 != 0) goto L_0x00e5
            boolean r14 = r1.memberSetter
            if (r14 == 0) goto L_0x0116
        L_0x00e5:
            java.lang.annotation.Annotation r14 = r0.by(r1, r13)
            boolean r14 = r14 instanceof org.bytedeco.javacpp.annotation.ByRef
            if (r14 != 0) goto L_0x0116
            org.bytedeco.javacpp.tools.AdapterInformation r14 = r0.adapterInformation((boolean) r9, (org.bytedeco.javacpp.tools.MethodInformation) r1, (int) r13)
            if (r14 != 0) goto L_0x0116
            java.lang.Class<?>[] r14 = r1.parameterTypes
            r14 = r14[r13]
            java.lang.Class<java.lang.String> r15 = java.lang.String.class
            if (r14 != r15) goto L_0x0116
            java.io.PrintWriter r14 = r0.out
            java.lang.StringBuilder r15 = new java.lang.StringBuilder
            r15.<init>()
            r15.append(r5)
            java.lang.String r9 = "strcpy((char*)"
            r15.append(r9)
            java.lang.String r9 = r15.toString()
            r14.print(r9)
            r12 = 1
            java.lang.String r6 = ", "
            goto L_0x01b4
        L_0x0116:
            if (r13 < r10) goto L_0x018f
            java.lang.Class<?>[] r9 = r1.parameterTypes
            r14 = 0
            r9 = r9[r14]
            boolean r9 = r9.isArray()
            if (r9 == 0) goto L_0x018f
            java.lang.Class<?>[] r9 = r1.parameterTypes
            r9 = r9[r14]
            java.lang.Class r9 = r9.getComponentType()
            boolean r9 = r9.isPrimitive()
            if (r9 == 0) goto L_0x018f
            java.lang.Class<?>[] r9 = r1.parameterTypes
            r9 = r9[r10]
            java.lang.Class r14 = java.lang.Integer.TYPE
            if (r9 == r14) goto L_0x0141
            java.lang.Class<?>[] r9 = r1.parameterTypes
            r9 = r9[r10]
            java.lang.Class r14 = java.lang.Long.TYPE
            if (r9 != r14) goto L_0x018f
        L_0x0141:
            java.io.PrintWriter r9 = r0.out
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            r14.append(r5)
            java.lang.String r15 = "memcpy("
            r14.append(r15)
            java.lang.String r14 = r14.toString()
            r9.print(r14)
            r12 = 1
            java.lang.String r6 = ", "
            boolean r9 = r1.memberGetter
            if (r9 != 0) goto L_0x0175
            boolean r9 = r1.valueGetter
            if (r9 == 0) goto L_0x0163
            goto L_0x0175
        L_0x0163:
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            r9.append(r6)
            java.lang.String r14 = "ptr0 + arg1, "
            r9.append(r14)
            java.lang.String r6 = r9.toString()
            goto L_0x017c
        L_0x0175:
            java.io.PrintWriter r9 = r0.out
            java.lang.String r14 = "ptr0 + arg1, "
            r9.print(r14)
        L_0x017c:
            r8 = 2
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            java.lang.String r14 = " * sizeof(*ptr0)"
            r9.append(r14)
            r9.append(r7)
            java.lang.String r7 = r9.toString()
            goto L_0x01b4
        L_0x018f:
            java.io.PrintWriter r9 = r0.out
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            r14.append(r5)
            r14.append(r2)
            java.lang.String r14 = r14.toString()
            r9.print(r14)
            boolean r9 = r1.valueGetter
            if (r9 != 0) goto L_0x01af
            boolean r9 = r1.memberGetter
            if (r9 == 0) goto L_0x01ac
            goto L_0x01af
        L_0x01ac:
            java.lang.String r9 = " = "
            goto L_0x01b1
        L_0x01af:
            java.lang.String r9 = ""
        L_0x01b1:
            r6 = r9
            java.lang.String r7 = ""
        L_0x01b4:
            int r9 = r1.modifiers
            boolean r9 = java.lang.reflect.Modifier.isStatic(r9)
            if (r9 != 0) goto L_0x0268
            java.lang.Class<org.bytedeco.javacpp.Pointer> r9 = org.bytedeco.javacpp.Pointer.class
            java.lang.Class<?> r14 = r1.cls
            boolean r9 = r9.isAssignableFrom(r14)
            if (r9 != 0) goto L_0x01ca
            r16 = r4
            goto L_0x026a
        L_0x01ca:
            boolean r9 = r1.memberGetter
            if (r9 != 0) goto L_0x0226
            boolean r9 = r1.memberSetter
            if (r9 == 0) goto L_0x01d5
            r16 = r4
            goto L_0x0228
        L_0x01d5:
            java.lang.Class<?> r9 = r1.returnType
            java.lang.annotation.Annotation[] r14 = r1.annotations
            java.lang.String r9 = r0.cast((java.lang.Class<?>) r9, (java.lang.annotation.Annotation[]) r14)
            if (r11 != 0) goto L_0x020d
            int r14 = r9.length()
            if (r14 <= 0) goto L_0x020d
            java.io.PrintWriter r14 = r0.out
            java.lang.StringBuilder r15 = new java.lang.StringBuilder
            r15.<init>()
            java.lang.String r10 = "*("
            r15.append(r10)
            int r10 = r9.length()
            r16 = r4
            r4 = 1
            int r10 = r10 - r4
            java.lang.String r10 = r9.substring(r4, r10)
            r15.append(r10)
            java.lang.String r4 = "*)&"
            r15.append(r4)
            java.lang.String r4 = r15.toString()
            r14.print(r4)
            goto L_0x020f
        L_0x020d:
            r16 = r4
        L_0x020f:
            java.io.PrintWriter r4 = r0.out
            if (r11 == 0) goto L_0x0216
            java.lang.String r10 = "(*ptr)"
            goto L_0x0222
        L_0x0216:
            int r10 = r1.dim
            if (r10 > 0) goto L_0x0220
            if (r12 == 0) goto L_0x021d
            goto L_0x0220
        L_0x021d:
            java.lang.String r10 = "*ptr"
            goto L_0x0222
        L_0x0220:
            java.lang.String r10 = "ptr"
        L_0x0222:
            r4.print(r10)
            goto L_0x0273
        L_0x0226:
            r16 = r4
        L_0x0228:
            if (r11 == 0) goto L_0x024c
            java.io.PrintWriter r4 = r0.out
            java.lang.String r9 = "(*ptr)"
            r4.print(r9)
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r9 = "."
            r4.append(r9)
            java.lang.String[] r9 = r1.memberName
            r10 = 0
            r9 = r9[r10]
            r4.append(r9)
            r4.append(r6)
            java.lang.String r4 = r4.toString()
            r6 = r4
            goto L_0x0273
        L_0x024c:
            java.io.PrintWriter r4 = r0.out
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            java.lang.String r10 = "ptr->"
            r9.append(r10)
            java.lang.String[] r10 = r1.memberName
            r14 = 0
            r10 = r10[r14]
            r9.append(r10)
            java.lang.String r9 = r9.toString()
            r4.print(r9)
            goto L_0x0273
        L_0x0268:
            r16 = r4
        L_0x026a:
            java.io.PrintWriter r4 = r0.out
            java.lang.String r9 = cppScopeName((org.bytedeco.javacpp.tools.MethodInformation) r18)
            r4.print(r9)
        L_0x0273:
            goto L_0x0502
        L_0x0275:
            r16 = r4
        L_0x0277:
            boolean r4 = r1.bufferGetter
            if (r4 == 0) goto L_0x029a
            java.io.PrintWriter r4 = r0.out
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            r9.append(r5)
            r9.append(r2)
            java.lang.String r10 = "ptr"
            r9.append(r10)
            java.lang.String r9 = r9.toString()
            r4.print(r9)
            java.lang.String r6 = ""
            java.lang.String r7 = ""
            goto L_0x0502
        L_0x029a:
            java.io.PrintWriter r4 = r0.out
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            r9.append(r5)
            r9.append(r2)
            java.lang.String r9 = r9.toString()
            r4.print(r9)
            java.lang.Class<org.bytedeco.javacpp.FunctionPointer> r4 = org.bytedeco.javacpp.FunctionPointer.class
            java.lang.Class<?> r9 = r1.cls
            boolean r4 = r4.isAssignableFrom(r9)
            if (r4 == 0) goto L_0x0307
            java.lang.Class<?> r4 = r1.cls
            java.lang.Class<org.bytedeco.javacpp.annotation.Namespace> r9 = org.bytedeco.javacpp.annotation.Namespace.class
            boolean r4 = r4.isAnnotationPresent(r9)
            if (r4 == 0) goto L_0x02e0
            java.io.PrintWriter r4 = r0.out
            java.lang.String r9 = "(ptr0->*(ptr->ptr))"
            r4.print(r9)
            r8 = 1
            boolean r4 = r1.valueGetter
            if (r4 != 0) goto L_0x02d2
            boolean r4 = r1.valueSetter
            if (r4 == 0) goto L_0x0502
        L_0x02d2:
            boolean r4 = r1.valueGetter
            if (r4 == 0) goto L_0x02d9
            java.lang.String r4 = ""
            goto L_0x02db
        L_0x02d9:
            java.lang.String r4 = " = "
        L_0x02db:
            r6 = r4
            java.lang.String r7 = ""
            goto L_0x0502
        L_0x02e0:
            boolean r4 = r1.valueGetter
            if (r4 != 0) goto L_0x02f2
            boolean r4 = r1.valueSetter
            if (r4 == 0) goto L_0x02e9
            goto L_0x02f2
        L_0x02e9:
            java.io.PrintWriter r4 = r0.out
            java.lang.String r9 = "(*ptr->ptr)"
            r4.print(r9)
            goto L_0x0502
        L_0x02f2:
            java.io.PrintWriter r4 = r0.out
            java.lang.String r9 = "ptr->ptr"
            r4.print(r9)
            boolean r4 = r1.valueGetter
            if (r4 == 0) goto L_0x0300
            java.lang.String r4 = ""
            goto L_0x0302
        L_0x0300:
            java.lang.String r4 = " = "
        L_0x0302:
            r6 = r4
            java.lang.String r7 = ""
            goto L_0x0502
        L_0x0307:
            boolean r4 = r1.allocator
            if (r4 == 0) goto L_0x0374
            java.lang.Class<?> r4 = r1.cls
            java.lang.String[] r4 = r0.cppTypeName(r4)
            java.lang.String r9 = valueTypeName(r4)
            java.util.Map<java.lang.Class, java.util.Set<java.lang.String>> r10 = r0.virtualFunctions
            java.lang.Class<?> r12 = r1.cls
            boolean r10 = r10.containsKey(r12)
            if (r10 == 0) goto L_0x0335
            java.lang.StringBuilder r10 = new java.lang.StringBuilder
            r10.<init>()
            java.lang.String r12 = "JavaCPP_"
            r10.append(r12)
            java.lang.String r12 = mangle(r9)
            r10.append(r12)
            java.lang.String r10 = r10.toString()
            r9 = r10
        L_0x0335:
            java.lang.Class<?> r10 = r1.cls
            java.lang.Class<org.bytedeco.javacpp.Pointer> r12 = org.bytedeco.javacpp.Pointer.class
            if (r10 != r12) goto L_0x0340
            java.lang.String r6 = ""
            java.lang.String r7 = ""
            goto L_0x0372
        L_0x0340:
            java.io.PrintWriter r10 = r0.out
            java.lang.StringBuilder r12 = new java.lang.StringBuilder
            r12.<init>()
            java.lang.Class<?> r13 = r1.cls
            java.lang.reflect.Method r14 = r1.method
            boolean r13 = noException(r13, r14)
            if (r13 == 0) goto L_0x0354
            java.lang.String r13 = "new (std::nothrow) "
            goto L_0x0356
        L_0x0354:
            java.lang.String r13 = "new "
        L_0x0356:
            r12.append(r13)
            r12.append(r9)
            r13 = 1
            r14 = r4[r13]
            r12.append(r14)
            java.lang.String r12 = r12.toString()
            r10.print(r12)
            boolean r10 = r1.arrayAllocator
            if (r10 == 0) goto L_0x0372
            java.lang.String r6 = "["
            java.lang.String r4 = "]"
            r7 = r4
        L_0x0372:
            goto L_0x0502
        L_0x0374:
            int r4 = r1.modifiers
            boolean r4 = java.lang.reflect.Modifier.isStatic(r4)
            if (r4 != 0) goto L_0x04f9
            java.lang.Class<org.bytedeco.javacpp.Pointer> r4 = org.bytedeco.javacpp.Pointer.class
            java.lang.Class<?> r9 = r1.cls
            boolean r4 = r4.isAssignableFrom(r9)
            if (r4 != 0) goto L_0x0388
            goto L_0x04f9
        L_0x0388:
            java.lang.String[] r4 = r1.memberName
            r9 = 0
            r4 = r4[r9]
            java.lang.Class<?> r9 = r1.cls
            java.lang.String[] r9 = r0.cppTypeName(r9)
            java.lang.String r10 = valueTypeName(r9)
            java.util.Map<java.lang.Class, java.util.Set<java.lang.String>> r12 = r0.virtualFunctions
            java.lang.Class<?> r13 = r1.cls
            boolean r12 = r12.containsKey(r13)
            if (r12 == 0) goto L_0x0421
            if (r20 != 0) goto L_0x0421
            java.lang.StringBuilder r12 = new java.lang.StringBuilder
            r12.<init>()
            java.lang.String r13 = "JavaCPP_"
            r12.append(r13)
            java.lang.String r13 = mangle(r10)
            r12.append(r13)
            java.lang.String r12 = r12.toString()
            java.lang.reflect.Method r13 = r1.method
            int r13 = r13.getModifiers()
            boolean r13 = java.lang.reflect.Modifier.isPublic(r13)
            if (r13 == 0) goto L_0x03e1
            java.io.PrintWriter r13 = r0.out
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            java.lang.String r15 = "(dynamic_cast<"
            r14.append(r15)
            r14.append(r12)
            java.lang.String r15 = "*>(ptr) != NULL ? "
            r14.append(r15)
            java.lang.String r14 = r14.toString()
            r13.print(r14)
            r13 = 1
            goto L_0x03e3
        L_0x03e1:
            r13 = r16
        L_0x03e3:
            java.lang.reflect.Method r14 = r1.method
            java.lang.Class<org.bytedeco.javacpp.annotation.Virtual> r15 = org.bytedeco.javacpp.annotation.Virtual.class
            boolean r14 = r14.isAnnotationPresent(r15)
            if (r14 == 0) goto L_0x03fe
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            java.lang.String r15 = "super_"
            r14.append(r15)
            r14.append(r4)
            java.lang.String r4 = r14.toString()
        L_0x03fe:
            java.io.PrintWriter r14 = r0.out
            java.lang.StringBuilder r15 = new java.lang.StringBuilder
            r15.<init>()
            java.lang.String r2 = "(("
            r15.append(r2)
            r15.append(r12)
            java.lang.String r2 = "*)ptr)->"
            r15.append(r2)
            r15.append(r4)
            java.lang.String r2 = r15.toString()
            r14.print(r2)
            r16 = r13
            goto L_0x0502
        L_0x0421:
            if (r11 == 0) goto L_0x0440
            java.io.PrintWriter r2 = r0.out
            java.lang.String r12 = "(*ptr)"
            r2.print(r12)
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r12 = "."
            r2.append(r12)
            r2.append(r4)
            r2.append(r6)
            java.lang.String r6 = r2.toString()
            goto L_0x0502
        L_0x0440:
            java.lang.String r2 = "operator"
            boolean r2 = r4.startsWith(r2)
            if (r2 == 0) goto L_0x0453
            r2 = 8
            java.lang.String r2 = r4.substring(r2)
            java.lang.String r2 = r2.trim()
            goto L_0x0455
        L_0x0453:
            java.lang.String r2 = ""
        L_0x0455:
            java.lang.Class<?>[] r12 = r1.parameterTypes
            int r12 = r12.length
            if (r12 <= 0) goto L_0x04e2
            java.lang.String r12 = "="
            boolean r12 = r2.equals(r12)
            if (r12 != 0) goto L_0x04ba
            java.lang.String r12 = "+"
            boolean r12 = r2.equals(r12)
            if (r12 != 0) goto L_0x04ba
            java.lang.String r12 = "-"
            boolean r12 = r2.equals(r12)
            if (r12 != 0) goto L_0x04ba
            java.lang.String r12 = "*"
            boolean r12 = r2.equals(r12)
            if (r12 != 0) goto L_0x04ba
            java.lang.String r12 = "/"
            boolean r12 = r2.equals(r12)
            if (r12 != 0) goto L_0x04ba
            java.lang.String r12 = "%"
            boolean r12 = r2.equals(r12)
            if (r12 != 0) goto L_0x04ba
            java.lang.String r12 = "=="
            boolean r12 = r2.equals(r12)
            if (r12 != 0) goto L_0x04ba
            java.lang.String r12 = "!="
            boolean r12 = r2.equals(r12)
            if (r12 != 0) goto L_0x04ba
            java.lang.String r12 = "<"
            boolean r12 = r2.equals(r12)
            if (r12 != 0) goto L_0x04ba
            java.lang.String r12 = ">"
            boolean r12 = r2.equals(r12)
            if (r12 != 0) goto L_0x04ba
            java.lang.String r12 = "<="
            boolean r12 = r2.equals(r12)
            if (r12 != 0) goto L_0x04ba
            java.lang.String r12 = ">="
            boolean r12 = r2.equals(r12)
            if (r12 == 0) goto L_0x04e2
        L_0x04ba:
            java.io.PrintWriter r12 = r0.out
            java.lang.String r13 = "((*ptr)"
            r12.print(r13)
            java.lang.StringBuilder r12 = new java.lang.StringBuilder
            r12.<init>()
            r12.append(r2)
            r12.append(r6)
            java.lang.String r6 = r12.toString()
            java.lang.StringBuilder r12 = new java.lang.StringBuilder
            r12.<init>()
            r12.append(r7)
            java.lang.String r13 = ")"
            r12.append(r13)
            java.lang.String r7 = r12.toString()
            goto L_0x0502
        L_0x04e2:
            java.io.PrintWriter r12 = r0.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "ptr->"
            r13.append(r14)
            r13.append(r4)
            java.lang.String r13 = r13.toString()
            r12.print(r13)
            goto L_0x0502
        L_0x04f9:
            java.io.PrintWriter r2 = r0.out
            java.lang.String r4 = cppScopeName((org.bytedeco.javacpp.tools.MethodInformation) r18)
            r2.print(r4)
        L_0x0502:
            r2 = r8
        L_0x0503:
            java.lang.Class<?>[] r4 = r1.parameterTypes
            int r4 = r4.length
            if (r2 > r4) goto L_0x0852
            int r4 = r1.dim
            int r4 = r4 + r8
            if (r2 != r4) goto L_0x0547
            java.lang.String[] r4 = r1.memberName
            int r4 = r4.length
            r9 = 1
            if (r4 <= r9) goto L_0x051c
            java.io.PrintWriter r4 = r0.out
            java.lang.String[] r10 = r1.memberName
            r10 = r10[r9]
            r4.print(r10)
        L_0x051c:
            java.io.PrintWriter r4 = r0.out
            r4.print(r6)
            boolean r4 = r1.withEnv
            if (r4 == 0) goto L_0x0547
            java.io.PrintWriter r4 = r0.out
            int r9 = r1.modifiers
            boolean r9 = java.lang.reflect.Modifier.isStatic(r9)
            if (r9 == 0) goto L_0x0532
            java.lang.String r9 = "env, cls"
            goto L_0x0534
        L_0x0532:
            java.lang.String r9 = "env, obj"
        L_0x0534:
            r4.print(r9)
            java.lang.Class<?>[] r4 = r1.parameterTypes
            int r4 = r4.length
            int r4 = r4 - r8
            int r9 = r1.dim
            int r4 = r4 - r9
            if (r4 <= 0) goto L_0x0547
            java.io.PrintWriter r4 = r0.out
            java.lang.String r9 = ", "
            r4.print(r9)
        L_0x0547:
            java.lang.Class<?>[] r4 = r1.parameterTypes
            int r4 = r4.length
            if (r2 != r4) goto L_0x054e
            goto L_0x0852
        L_0x054e:
            int r4 = r1.dim
            int r4 = r4 + r8
            if (r2 >= r4) goto L_0x0587
            if (r11 == 0) goto L_0x0580
            java.lang.String r4 = r11.function()
            int r4 = r4.length()
            if (r4 != 0) goto L_0x0560
            goto L_0x0580
        L_0x0560:
            java.io.PrintWriter r4 = r0.out
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            java.lang.String r10 = "."
            r9.append(r10)
            java.lang.String r10 = r11.function()
            r9.append(r10)
            java.lang.String r10 = "("
            r9.append(r10)
            java.lang.String r9 = r9.toString()
            r4.print(r9)
            goto L_0x0587
        L_0x0580:
            java.io.PrintWriter r4 = r0.out
            java.lang.String r9 = "["
            r4.print(r9)
        L_0x0587:
            java.lang.annotation.Annotation r4 = r0.by(r1, r2)
            java.lang.String r9 = r0.cast((org.bytedeco.javacpp.tools.MethodInformation) r1, (int) r2)
            boolean[] r10 = r1.parameterRaw
            boolean r10 = r10[r2]
            if (r10 == 0) goto L_0x0598
            r10 = 0
            r12 = r10
            goto L_0x059e
        L_0x0598:
            r10 = 0
            org.bytedeco.javacpp.tools.AdapterInformation r12 = r0.adapterInformation((boolean) r10, (org.bytedeco.javacpp.tools.MethodInformation) r1, (int) r2)
        L_0x059e:
            r10 = r12
            java.lang.Class<org.bytedeco.javacpp.FunctionPointer> r12 = org.bytedeco.javacpp.FunctionPointer.class
            java.lang.Class<?> r13 = r1.cls
            boolean r12 = r12.isAssignableFrom(r13)
            if (r12 == 0) goto L_0x05dc
            java.lang.Class<?> r12 = r1.cls
            java.lang.Class<org.bytedeco.javacpp.annotation.Namespace> r13 = org.bytedeco.javacpp.annotation.Namespace.class
            boolean r12 = r12.isAnnotationPresent(r13)
            if (r12 != 0) goto L_0x05dc
            boolean r12 = r1.valueSetter
            if (r12 == 0) goto L_0x05dc
            java.lang.Class<?> r12 = r1.cls
            java.lang.String[] r12 = r0.cppTypeName(r12)
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "("
            r13.append(r14)
            r14 = 0
            r15 = r12[r14]
            r13.append(r15)
            r15 = 1
            r14 = r12[r15]
            r13.append(r14)
            java.lang.String r14 = ")"
            r13.append(r14)
            java.lang.String r9 = r13.toString()
        L_0x05dc:
            java.lang.Class<java.lang.Enum> r12 = java.lang.Enum.class
            java.lang.Class<?>[] r13 = r1.parameterTypes
            r13 = r13[r2]
            boolean r12 = r12.isAssignableFrom(r13)
            if (r12 == 0) goto L_0x0606
            r12 = 1
            r0.accessesEnums = r12
            java.io.PrintWriter r12 = r0.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            r13.append(r9)
            java.lang.String r14 = "val"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r13 = r13.toString()
            r12.print(r13)
            goto L_0x081e
        L_0x0606:
            java.lang.String r12 = "(void*)"
            boolean r12 = r12.equals(r9)
            if (r12 != 0) goto L_0x0616
            java.lang.String r12 = "(void *)"
            boolean r12 = r12.equals(r9)
            if (r12 == 0) goto L_0x063b
        L_0x0616:
            java.lang.Class<?>[] r12 = r1.parameterTypes
            r12 = r12[r2]
            java.lang.Class r13 = java.lang.Long.TYPE
            if (r12 != r13) goto L_0x063b
            java.io.PrintWriter r12 = r0.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "jlong_to_ptr(arg"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = ")"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r12.print(r13)
            goto L_0x081e
        L_0x063b:
            java.lang.Class<?>[] r12 = r1.parameterTypes
            r12 = r12[r2]
            boolean r12 = r12.isPrimitive()
            if (r12 == 0) goto L_0x066f
            boolean r12 = r4 instanceof org.bytedeco.javacpp.annotation.ByPtr
            if (r12 != 0) goto L_0x064d
            boolean r12 = r4 instanceof org.bytedeco.javacpp.annotation.ByPtrRef
            if (r12 == 0) goto L_0x0654
        L_0x064d:
            java.io.PrintWriter r12 = r0.out
            java.lang.String r13 = "&"
            r12.print(r13)
        L_0x0654:
            java.io.PrintWriter r12 = r0.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            r13.append(r9)
            java.lang.String r14 = "arg"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r13 = r13.toString()
            r12.print(r13)
            goto L_0x081e
        L_0x066f:
            if (r10 == 0) goto L_0x06f8
            java.lang.String r12 = r10.cast
            java.lang.String r9 = r12.trim()
            int r12 = r9.length()
            if (r12 <= 0) goto L_0x06a3
            java.lang.String r12 = "("
            boolean r12 = r9.startsWith(r12)
            if (r12 != 0) goto L_0x06a3
            java.lang.String r12 = ")"
            boolean r12 = r9.endsWith(r12)
            if (r12 != 0) goto L_0x06a3
            java.lang.StringBuilder r12 = new java.lang.StringBuilder
            r12.<init>()
            java.lang.String r13 = "("
            r12.append(r13)
            r12.append(r9)
            java.lang.String r13 = ")"
            r12.append(r13)
            java.lang.String r9 = r12.toString()
        L_0x06a3:
            java.lang.String r12 = r10.cast2
            java.lang.String r12 = r12.trim()
            int r13 = r12.length()
            if (r13 <= 0) goto L_0x06d5
            java.lang.String r13 = "("
            boolean r13 = r12.startsWith(r13)
            if (r13 != 0) goto L_0x06d5
            java.lang.String r13 = ")"
            boolean r13 = r12.endsWith(r13)
            if (r13 != 0) goto L_0x06d5
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "("
            r13.append(r14)
            r13.append(r12)
            java.lang.String r14 = ")"
            r13.append(r14)
            java.lang.String r12 = r13.toString()
        L_0x06d5:
            java.io.PrintWriter r13 = r0.out
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            r14.append(r9)
            r14.append(r12)
            java.lang.String r15 = "adapter"
            r14.append(r15)
            r14.append(r2)
            java.lang.String r14 = r14.toString()
            r13.print(r14)
            int r13 = r10.argc
            r14 = 1
            int r13 = r13 - r14
            int r2 = r2 + r13
            goto L_0x081e
        L_0x06f8:
            java.lang.Class<org.bytedeco.javacpp.FunctionPointer> r12 = org.bytedeco.javacpp.FunctionPointer.class
            java.lang.Class<?>[] r13 = r1.parameterTypes
            r13 = r13[r2]
            boolean r12 = r12.isAssignableFrom(r13)
            if (r12 == 0) goto L_0x0764
            boolean r12 = r4 instanceof org.bytedeco.javacpp.annotation.ByVal
            if (r12 != 0) goto L_0x0764
            boolean r12 = r4 instanceof org.bytedeco.javacpp.annotation.ByRef
            if (r12 != 0) goto L_0x0764
            boolean r12 = r4 instanceof org.bytedeco.javacpp.annotation.ByPtrRef
            if (r12 == 0) goto L_0x0730
            java.io.PrintWriter r12 = r0.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            r13.append(r9)
            java.lang.String r14 = "(ptr"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = "->ptr)"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r12.print(r13)
            goto L_0x081e
        L_0x0730:
            java.io.PrintWriter r12 = r0.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            r13.append(r9)
            java.lang.String r14 = "(ptr"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = " == NULL ? NULL : "
            r13.append(r14)
            boolean r14 = r4 instanceof org.bytedeco.javacpp.annotation.ByPtrPtr
            if (r14 == 0) goto L_0x074e
            java.lang.String r14 = "&ptr"
            goto L_0x0750
        L_0x074e:
            java.lang.String r14 = "ptr"
        L_0x0750:
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = "->ptr)"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r12.print(r13)
            goto L_0x081e
        L_0x0764:
            boolean r12 = r4 instanceof org.bytedeco.javacpp.annotation.ByVal
            if (r12 != 0) goto L_0x07bb
            boolean r12 = r4 instanceof org.bytedeco.javacpp.annotation.ByRef
            if (r12 == 0) goto L_0x0775
            java.lang.Class<?>[] r12 = r1.parameterTypes
            r12 = r12[r2]
            java.lang.Class<java.lang.String> r13 = java.lang.String.class
            if (r12 == r13) goto L_0x0775
            goto L_0x07bb
        L_0x0775:
            boolean r12 = r4 instanceof org.bytedeco.javacpp.annotation.ByPtrPtr
            if (r12 == 0) goto L_0x07a1
            java.io.PrintWriter r12 = r0.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            r13.append(r9)
            java.lang.String r14 = "(arg"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = " == NULL ? NULL : &ptr"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = ")"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r12.print(r13)
            goto L_0x081e
        L_0x07a1:
            java.io.PrintWriter r12 = r0.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            r13.append(r9)
            java.lang.String r14 = "ptr"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r13 = r13.toString()
            r12.print(r13)
            goto L_0x081e
        L_0x07bb:
            boolean r12 = r4 instanceof org.bytedeco.javacpp.annotation.ByVal
            if (r12 == 0) goto L_0x07c7
            r12 = r4
            org.bytedeco.javacpp.annotation.ByVal r12 = (org.bytedeco.javacpp.annotation.ByVal) r12
            java.lang.String r12 = r12.nullValue()
            goto L_0x07d5
        L_0x07c7:
            boolean r12 = r4 instanceof org.bytedeco.javacpp.annotation.ByRef
            if (r12 == 0) goto L_0x07d3
            r12 = r4
            org.bytedeco.javacpp.annotation.ByRef r12 = (org.bytedeco.javacpp.annotation.ByRef) r12
            java.lang.String r12 = r12.nullValue()
            goto L_0x07d5
        L_0x07d3:
            java.lang.String r12 = ""
        L_0x07d5:
            java.io.PrintWriter r13 = r0.out
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            int r15 = r12.length()
            if (r15 <= 0) goto L_0x0801
            java.lang.StringBuilder r15 = new java.lang.StringBuilder
            r15.<init>()
            java.lang.String r3 = "ptr"
            r15.append(r3)
            r15.append(r2)
            java.lang.String r3 = " == NULL ? "
            r15.append(r3)
            r15.append(r12)
            java.lang.String r3 = " : "
            r15.append(r3)
            java.lang.String r3 = r15.toString()
            goto L_0x0803
        L_0x0801:
            java.lang.String r3 = ""
        L_0x0803:
            r14.append(r3)
            java.lang.String r3 = "*"
            r14.append(r3)
            r14.append(r9)
            java.lang.String r3 = "ptr"
            r14.append(r3)
            r14.append(r2)
            java.lang.String r3 = r14.toString()
            r13.print(r3)
        L_0x081e:
            int r3 = r1.dim
            int r3 = r3 + r8
            if (r2 >= r3) goto L_0x0840
            if (r11 == 0) goto L_0x0838
            java.lang.String r3 = r11.function()
            int r3 = r3.length()
            if (r3 != 0) goto L_0x0830
            goto L_0x0838
        L_0x0830:
            java.io.PrintWriter r3 = r0.out
            java.lang.String r12 = ")"
            r3.print(r12)
            goto L_0x084e
        L_0x0838:
            java.io.PrintWriter r3 = r0.out
            java.lang.String r12 = "]"
            r3.print(r12)
            goto L_0x084e
        L_0x0840:
            java.lang.Class<?>[] r3 = r1.parameterTypes
            int r3 = r3.length
            r12 = 1
            int r3 = r3 - r12
            if (r2 >= r3) goto L_0x084e
            java.io.PrintWriter r3 = r0.out
            java.lang.String r12 = ", "
            r3.print(r12)
        L_0x084e:
            int r2 = r2 + 1
            goto L_0x0503
        L_0x0852:
            java.io.PrintWriter r2 = r0.out
            r2.print(r7)
            java.lang.String[] r2 = r1.memberName
            int r2 = r2.length
            r3 = 2
            if (r2 <= r3) goto L_0x0866
            java.io.PrintWriter r2 = r0.out
            java.lang.String[] r4 = r1.memberName
            r3 = r4[r3]
            r2.print(r3)
        L_0x0866:
            if (r16 == 0) goto L_0x0875
            java.lang.String r2 = " : "
            r3 = 1
            r0.call(r1, r2, r3)
            java.io.PrintWriter r2 = r0.out
            java.lang.String r3 = ")"
            r2.print(r3)
        L_0x0875:
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.Generator.call(org.bytedeco.javacpp.tools.MethodInformation, java.lang.String, boolean):void");
    }

    /* access modifiers changed from: package-private */
    public void returnAfter(MethodInformation methodInfo) {
        String indent = methodInfo.throwsException != null ? "        " : "    ";
        boolean noDeallocator = false;
        String[] typeName = methodInfo.returnRaw ? new String[]{""} : cppCastTypeName(methodInfo.returnType, methodInfo.annotations);
        Annotation returnBy = by(methodInfo.annotations);
        AdapterInformation adapterInfo = adapterInformation(false, valueTypeName(typeName), methodInfo.annotations);
        String suffix = methodInfo.deallocator ? "" : ";";
        if ((by(methodInfo.annotations) instanceof ByRef) && methodInfo.returnType == String.class && adapterInfo == null) {
            this.out.print(");\n" + indent + "rptr = rstr.c_str()");
        }
        if (!methodInfo.returnType.isPrimitive() && adapterInfo != null) {
            suffix = ")" + suffix;
        }
        if (Pointer.class.isAssignableFrom(methodInfo.returnType) || ((methodInfo.returnType.isArray() && methodInfo.returnType.getComponentType().isPrimitive()) || Buffer.class.isAssignableFrom(methodInfo.returnType) || methodInfo.returnType == String.class)) {
            if ((returnBy instanceof ByVal) && adapterInfo == null) {
                suffix = ")" + suffix;
            } else if (returnBy instanceof ByPtrPtr) {
                this.out.println(suffix);
                suffix = "";
                this.out.println(indent + "if (rptrptr == NULL) {");
                this.out.println(indent + "    env->ThrowNew(JavaCPP_getClass(env, " + this.jclasses.index(NullPointerException.class) + "), \"Return pointer address is NULL.\");");
                PrintWriter printWriter = this.out;
                StringBuilder sb = new StringBuilder();
                sb.append(indent);
                sb.append("} else {");
                printWriter.println(sb.toString());
                this.out.println(indent + "    rptr = *rptrptr;");
                this.out.println(indent + "}");
            }
        }
        this.out.println(suffix);
        if (methodInfo.returnType == Void.TYPE) {
            if (methodInfo.allocator || methodInfo.arrayAllocator) {
                PrintWriter printWriter2 = this.out;
                StringBuilder sb2 = new StringBuilder();
                sb2.append(indent);
                sb2.append("jlong rcapacity = ");
                sb2.append(methodInfo.arrayAllocator ? "arg0;" : "1;");
                printWriter2.println(sb2.toString());
                if (methodInfo.cls == Pointer.class || methodInfo.cls.isAnnotationPresent(NoDeallocator.class) || methodInfo.method.isAnnotationPresent(NoDeallocator.class)) {
                    noDeallocator = true;
                }
                this.out.print(indent + "JavaCPP_initPointer(env, obj, rptr, rcapacity, rptr, ");
                if (noDeallocator) {
                    this.out.println("NULL);");
                } else if (methodInfo.arrayAllocator) {
                    this.out.println("&JavaCPP_" + mangle(methodInfo.cls.getName()) + "_deallocateArray);");
                    this.arrayDeallocators.index(methodInfo.cls);
                } else {
                    this.out.println("&JavaCPP_" + mangle(methodInfo.cls.getName()) + "_deallocate);");
                    this.deallocators.index(methodInfo.cls);
                }
                if (this.virtualFunctions.containsKey(methodInfo.cls)) {
                    String valueTypeName = valueTypeName(cppTypeName(methodInfo.cls));
                    PrintWriter printWriter3 = this.out;
                    printWriter3.println(indent + "((" + ("JavaCPP_" + mangle(valueTypeName)) + "*)rptr)->obj = env->NewWeakGlobalRef(obj);");
                }
            }
        } else if (!methodInfo.valueSetter && !methodInfo.memberSetter && !methodInfo.noReturnGetter) {
            if (methodInfo.returnType.isPrimitive()) {
                this.out.println(indent + "rarg = (" + jniTypeName(methodInfo.returnType) + ")rval;");
            } else if (methodInfo.returnRaw) {
                this.out.println(indent + "rarg = rptr;");
            } else if (Enum.class.isAssignableFrom(methodInfo.returnType)) {
                this.accessesEnums = true;
                String s = enumValueType(methodInfo.returnType);
                if (s != null) {
                    this.out.println(indent + "if (rarg != NULL) {");
                    PrintWriter printWriter4 = this.out;
                    printWriter4.println(indent + "    env->Set" + (Character.toUpperCase(s.charAt(0)) + s.substring(1)) + "Field(rarg, JavaCPP_" + s + "ValueFID, (j" + s + ")rval);");
                    PrintWriter printWriter5 = this.out;
                    StringBuilder sb3 = new StringBuilder();
                    sb3.append(indent);
                    sb3.append("}");
                    printWriter5.println(sb3.toString());
                }
            } else {
                boolean needInit = false;
                if (adapterInfo != null) {
                    this.out.println(indent + "rptr = radapter;");
                    if (methodInfo.returnType != String.class) {
                        this.out.println(indent + "jlong rcapacity = (jlong)radapter.size;");
                        if (Pointer.class.isAssignableFrom(methodInfo.returnType)) {
                            this.out.println(indent + "void* rowner = radapter.owner;");
                        }
                        this.out.println(indent + "void (*deallocator)(void*) = &" + adapterInfo.name + "::deallocate;");
                    }
                    needInit = true;
                } else if ((returnBy instanceof ByVal) || FunctionPointer.class.isAssignableFrom(methodInfo.returnType)) {
                    this.out.println(indent + "jlong rcapacity = 1;");
                    this.out.println(indent + "void* rowner = (void*)rptr;");
                    this.out.println(indent + "void (*deallocator)(void*) = &JavaCPP_" + mangle(methodInfo.returnType.getName()) + "_deallocate;");
                    this.deallocators.index(methodInfo.returnType);
                    needInit = true;
                }
                if (Pointer.class.isAssignableFrom(methodInfo.returnType)) {
                    this.out.print(indent);
                    if (!(returnBy instanceof ByVal)) {
                        if (Modifier.isStatic(methodInfo.modifiers) && methodInfo.parameterTypes.length > 0) {
                            for (int i = 0; i < methodInfo.parameterTypes.length; i++) {
                                String cast = cast(methodInfo, i);
                                if (Arrays.equals(methodInfo.parameterAnnotations[i], methodInfo.annotations) && methodInfo.parameterTypes[i] == methodInfo.returnType && !(returnBy instanceof ByPtrPtr) && !(returnBy instanceof ByPtrRef)) {
                                    this.out.println("if (rptr == " + cast + "ptr" + i + ") {");
                                    this.out.println(indent + "    rarg = arg" + i + ";");
                                    PrintWriter printWriter6 = this.out;
                                    StringBuilder sb4 = new StringBuilder();
                                    sb4.append(indent);
                                    sb4.append("} else ");
                                    printWriter6.print(sb4.toString());
                                }
                            }
                        } else if (!Modifier.isStatic(methodInfo.modifiers) && methodInfo.cls == methodInfo.returnType) {
                            this.out.println("if (rptr == ptr) {");
                            this.out.println(indent + "    rarg = obj;");
                            this.out.print(indent + "} else ");
                        }
                    }
                    this.out.println("if (rptr != NULL) {");
                    PrintWriter printWriter7 = this.out;
                    StringBuilder sb5 = new StringBuilder();
                    sb5.append(indent);
                    sb5.append("    rarg = JavaCPP_createPointer(env, ");
                    sb5.append(this.jclasses.index(methodInfo.returnType));
                    sb5.append((methodInfo.parameterTypes.length <= 0 || methodInfo.parameterTypes[0] != Class.class) ? ");" : ", arg0);");
                    printWriter7.println(sb5.toString());
                    this.out.println(indent + "    if (rarg != NULL) {");
                    if (needInit) {
                        this.out.println(indent + "        JavaCPP_initPointer(env, rarg, rptr, rcapacity, rowner, deallocator);");
                    } else {
                        this.out.println(indent + "        env->SetLongField(rarg, JavaCPP_addressFID, ptr_to_jlong(rptr));");
                    }
                    this.out.println(indent + "    }");
                    this.out.println(indent + "}");
                } else if (methodInfo.returnType == String.class) {
                    this.passesStrings = true;
                    this.out.println(indent + "if (rptr != NULL) {");
                    this.out.println(indent + "    rarg = JavaCPP_createString(env, rptr);");
                    this.out.println(indent + "}");
                } else if (methodInfo.returnType.isArray() && methodInfo.returnType.getComponentType().isPrimitive()) {
                    if (adapterInfo == null && !(returnBy instanceof ByVal)) {
                        this.out.println(indent + "jlong rcapacity = rptr != NULL ? 1 : 0;");
                    }
                    String componentName = methodInfo.returnType.getComponentType().getName();
                    String componentNameUpperCase = Character.toUpperCase(componentName.charAt(0)) + componentName.substring(1);
                    this.out.println(indent + "if (rptr != NULL) {");
                    this.out.println(indent + "    rarg = env->New" + componentNameUpperCase + "Array(rcapacity < INT_MAX ? rcapacity : INT_MAX);");
                    this.out.println(indent + "    env->Set" + componentNameUpperCase + "ArrayRegion(rarg, 0, rcapacity < INT_MAX ? rcapacity : INT_MAX, (j" + componentName + "*)rptr);");
                    PrintWriter printWriter8 = this.out;
                    StringBuilder sb6 = new StringBuilder();
                    sb6.append(indent);
                    sb6.append("}");
                    printWriter8.println(sb6.toString());
                    if (adapterInfo != null) {
                        this.out.println(indent + "if (deallocator != 0 && rptr != NULL) {");
                        this.out.println(indent + "    (*(void(*)(void*))jlong_to_ptr(deallocator))((void*)rptr);");
                        this.out.println(indent + "}");
                    }
                } else if (Buffer.class.isAssignableFrom(methodInfo.returnType)) {
                    if (methodInfo.bufferGetter) {
                        this.out.println(indent + "jlong rposition = position;");
                        this.out.println(indent + "jlong rlimit = limit;");
                        this.out.println(indent + "jlong rcapacity = capacity;");
                    } else if (adapterInfo == null && !(returnBy instanceof ByVal)) {
                        this.out.println(indent + "jlong rcapacity = rptr != NULL ? 1 : 0;");
                    }
                    this.out.println(indent + "if (rptr != NULL) {");
                    this.out.println(indent + "    jlong rcapacityptr = rcapacity * sizeof(rptr[0]);");
                    this.out.println(indent + "    rarg = env->NewDirectByteBuffer((void*)rptr, rcapacityptr < INT_MAX ? rcapacityptr : INT_MAX);");
                    if (methodInfo.bufferGetter) {
                        this.out.println(indent + "    jlong rpositionptr = rposition * sizeof(rptr[0]);");
                        this.out.println(indent + "    jlong rlimitptr = rlimit * sizeof(rptr[0]);");
                        this.out.println(indent + "    env->SetIntField(rarg, JavaCPP_bufferPositionFID, rpositionptr < INT_MAX ? rpositionptr : INT_MAX);");
                        this.out.println(indent + "    env->SetIntField(rarg, JavaCPP_bufferLimitFID, rlimitptr < INT_MAX ? rlimitptr : INT_MAX);");
                    }
                    this.out.println(indent + "}");
                }
            }
        }
    }

    /* access modifiers changed from: package-private */
    public void parametersAfter(MethodInformation methodInfo) {
        String releaseArrayFlag;
        String str;
        String str2;
        String str3;
        String str4;
        String str5;
        if (methodInfo.throwsException != null) {
            this.mayThrowExceptions = true;
            this.out.println("    } catch (...) {");
            this.out.println("        exc = JavaCPP_handleException(env, " + this.jclasses.index(methodInfo.throwsException) + ");");
            this.out.println("    }");
            this.out.println();
        }
        for (int j = (methodInfo.parameterTypes.length <= 0 || methodInfo.parameterTypes[0] != Class.class) ? 0 : 1; j < methodInfo.parameterTypes.length; j++) {
            if (!methodInfo.parameterRaw[j] && !Enum.class.isAssignableFrom(methodInfo.parameterTypes[j])) {
                Annotation passBy = by(methodInfo, j);
                String cast = cast(methodInfo, j);
                String[] typeName = cppCastTypeName(methodInfo.parameterTypes[j], methodInfo.parameterAnnotations[j]);
                AdapterInformation adapterInfo = adapterInformation(true, methodInfo, j);
                if ("void*".equals(typeName[0]) && !methodInfo.parameterTypes[j].isAnnotationPresent(Opaque.class)) {
                    typeName[0] = "char*";
                }
                if (cast.contains(" const *") || cast.startsWith("(const ")) {
                    releaseArrayFlag = "JNI_ABORT";
                } else {
                    releaseArrayFlag = "0";
                }
                if (Pointer.class.isAssignableFrom(methodInfo.parameterTypes[j])) {
                    if (adapterInfo != null) {
                        for (int k = 0; k < adapterInfo.argc; k++) {
                            this.out.println("    " + typeName[0] + " rptr" + (j + k) + typeName[1] + " = " + cast + "adapter" + j + ";");
                            PrintWriter printWriter = this.out;
                            StringBuilder sb = new StringBuilder();
                            sb.append("    jlong rsize");
                            sb.append(j + k);
                            sb.append(" = (jlong)adapter");
                            sb.append(j);
                            sb.append(".size");
                            if (k > 0) {
                                str3 = (k + 1) + ";";
                            } else {
                                str3 = ";";
                            }
                            sb.append(str3);
                            printWriter.println(sb.toString());
                            PrintWriter printWriter2 = this.out;
                            StringBuilder sb2 = new StringBuilder();
                            sb2.append("    void* rowner");
                            sb2.append(j + k);
                            sb2.append(" = adapter");
                            sb2.append(j);
                            sb2.append(".owner");
                            if (k > 0) {
                                str4 = (k + 1) + ";";
                            } else {
                                str4 = ";";
                            }
                            sb2.append(str4);
                            printWriter2.println(sb2.toString());
                            this.out.println("    if (rptr" + (j + k) + " != " + cast + "ptr" + (j + k) + ") {");
                            this.out.println("        JavaCPP_initPointer(env, arg" + j + ", rptr" + (j + k) + ", rsize" + (j + k) + ", rowner" + (j + k) + ", &" + adapterInfo.name + "::deallocate);");
                            this.out.println("    } else {");
                            PrintWriter printWriter3 = this.out;
                            StringBuilder sb3 = new StringBuilder();
                            sb3.append("        env->SetLongField(arg");
                            sb3.append(j);
                            sb3.append(", JavaCPP_limitFID, rsize");
                            sb3.append(j + k);
                            if (!methodInfo.parameterTypes[j].isAnnotationPresent(Opaque.class)) {
                                str5 = " + position" + (j + k);
                            } else {
                                str5 = "";
                            }
                            sb3.append(str5);
                            sb3.append(");");
                            printWriter3.println(sb3.toString());
                            this.out.println("    }");
                        }
                    } else if (((passBy instanceof ByPtrPtr) != 0 || (passBy instanceof ByPtrRef)) && !methodInfo.valueSetter && !methodInfo.memberSetter) {
                        if (!methodInfo.parameterTypes[j].isAnnotationPresent(Opaque.class)) {
                            this.out.println("    ptr" + j + " -= position" + j + ";");
                        }
                        this.out.println("    if (arg" + j + " != NULL) env->SetLongField(arg" + j + ", JavaCPP_addressFID, ptr_to_jlong(ptr" + j + "));");
                    }
                } else if (methodInfo.parameterTypes[j] == String.class) {
                    this.out.println("    JavaCPP_releaseStringBytes(env, arg" + j + ", ptr" + j + ");");
                } else if (methodInfo.parameterTypes[j].isArray() && methodInfo.parameterTypes[j].getComponentType().isPrimitive()) {
                    int k2 = 0;
                    while (adapterInfo != null && k2 < adapterInfo.argc) {
                        this.out.println("    " + typeName[0] + " rptr" + (j + k2) + typeName[1] + " = " + cast + "adapter" + j + ";");
                        PrintWriter printWriter4 = this.out;
                        StringBuilder sb4 = new StringBuilder();
                        sb4.append("    void* rowner");
                        sb4.append(j + k2);
                        sb4.append(" = adapter");
                        sb4.append(j);
                        sb4.append(".owner");
                        if (k2 > 0) {
                            str2 = (k2 + 1) + ";";
                        } else {
                            str2 = ";";
                        }
                        sb4.append(str2);
                        printWriter4.println(sb4.toString());
                        this.out.println("    if (rptr" + (j + k2) + " != " + cast + "ptr" + (j + k2) + ") {");
                        PrintWriter printWriter5 = this.out;
                        StringBuilder sb5 = new StringBuilder();
                        sb5.append("        ");
                        sb5.append(adapterInfo.name);
                        sb5.append("::deallocate(rowner");
                        sb5.append(j + k2);
                        sb5.append(");");
                        printWriter5.println(sb5.toString());
                        this.out.println("    }");
                        k2++;
                    }
                    this.out.print("    if (arg" + j + " != NULL) ");
                    if (methodInfo.criticalRegion || methodInfo.valueGetter || methodInfo.valueSetter || methodInfo.memberGetter || methodInfo.memberSetter) {
                        this.out.println("env->ReleasePrimitiveArrayCritical(arg" + j + ", ptr" + j + ", " + releaseArrayFlag + ");");
                    } else {
                        String componentType = methodInfo.parameterTypes[j].getComponentType().getName();
                        PrintWriter printWriter6 = this.out;
                        printWriter6.println("env->Release" + (Character.toUpperCase(componentType.charAt(0)) + componentType.substring(1)) + "ArrayElements(arg" + j + ", (j" + componentType + "*)ptr" + j + ", " + releaseArrayFlag + ");");
                    }
                } else if (Buffer.class.isAssignableFrom(methodInfo.parameterTypes[j]) && methodInfo.parameterTypes[j] != Buffer.class) {
                    int k3 = 0;
                    while (adapterInfo != null && k3 < adapterInfo.argc) {
                        this.out.println("    " + typeName[0] + " rptr" + (j + k3) + typeName[1] + " = " + cast + "adapter" + j + ";");
                        PrintWriter printWriter7 = this.out;
                        StringBuilder sb6 = new StringBuilder();
                        sb6.append("    void* rowner");
                        sb6.append(j + k3);
                        sb6.append(" = adapter");
                        sb6.append(j);
                        sb6.append(".owner");
                        if (k3 > 0) {
                            str = (k3 + 1) + ";";
                        } else {
                            str = ";";
                        }
                        sb6.append(str);
                        printWriter7.println(sb6.toString());
                        this.out.println("    if (rptr" + (j + k3) + " != " + cast + "ptr" + (j + k3) + ") {");
                        PrintWriter printWriter8 = this.out;
                        StringBuilder sb7 = new StringBuilder();
                        sb7.append("        ");
                        sb7.append(adapterInfo.name);
                        sb7.append("::deallocate(rowner");
                        sb7.append(j + k3);
                        sb7.append(");");
                        printWriter8.println(sb7.toString());
                        this.out.println("    }");
                        k3++;
                    }
                    this.out.print("    if (arr" + j + " != NULL) ");
                    String parameterSimpleName = methodInfo.parameterTypes[j].getSimpleName();
                    String parameterSimpleName2 = parameterSimpleName.substring(0, parameterSimpleName.length() - 6);
                    String parameterSimpleNameLowerCase = Character.toLowerCase(parameterSimpleName2.charAt(0)) + parameterSimpleName2.substring(1);
                    if (methodInfo.criticalRegion) {
                        this.out.println("env->ReleasePrimitiveArrayCritical(arr" + j + ", ptr" + j + " - position" + j + ", " + releaseArrayFlag + ");");
                    } else {
                        this.out.println("env->Release" + parameterSimpleName2 + "ArrayElements(arr" + j + ", (j" + parameterSimpleNameLowerCase + "*)(ptr" + j + " - position" + j + "), " + releaseArrayFlag + ");");
                    }
                }
            }
        }
    }

    /* access modifiers changed from: package-private */
    /* JADX WARNING: Removed duplicated region for block: B:208:0x0bb2  */
    /* JADX WARNING: Removed duplicated region for block: B:209:0x0bee  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public void callback(java.lang.Class<?> r45, java.lang.reflect.Method r46, java.lang.String r47, boolean r48, org.bytedeco.javacpp.tools.MethodInformation r49) {
        /*
            r44 = this;
            r0 = r44
            r1 = r45
            r2 = r46
            r3 = r47
            r4 = r49
            java.lang.Class r5 = r46.getReturnType()
            java.lang.Class[] r6 = r46.getParameterTypes()
            java.lang.annotation.Annotation[] r7 = r46.getAnnotations()
            java.lang.annotation.Annotation[][] r8 = r46.getParameterAnnotations()
            java.lang.String r9 = functionClassName(r45)
            r10 = 1
            java.lang.reflect.Method[] r11 = new java.lang.reflect.Method[r10]
            r12 = 0
            r11[r12] = r2
            java.lang.String[] r11 = r0.cppFunctionTypeName(r11)
            r13 = r11[r12]
            java.lang.String r14 = "\\("
            java.lang.String[] r13 = r13.split(r14)
            java.lang.String[] r14 = new java.lang.String[r10]
            r15 = r13[r10]
            r14[r12] = r15
            java.lang.String r14 = constValueTypeName(r14)
            r13[r10] = r14
            r14 = r11[r10]
            java.lang.String r14 = r14.substring(r10)
            java.lang.StringBuilder r15 = new java.lang.StringBuilder
            r15.<init>()
            java.lang.String r16 = r46.getName()
            java.lang.String r10 = mangle(r16)
            r15.append(r10)
            java.lang.String r10 = "__"
            r15.append(r10)
            java.lang.Class[] r10 = r46.getParameterTypes()
            java.lang.String r10 = signature(r10)
            java.lang.String r10 = mangle(r10)
            r15.append(r10)
            java.lang.String r10 = r15.toString()
            java.lang.String r15 = ""
            if (r4 == 0) goto L_0x0322
            java.lang.String r12 = " const"
            boolean r12 = r14.endsWith(r12)
            if (r12 == 0) goto L_0x0084
            int r12 = r14.length()
            int r12 = r12 + -6
            r19 = r11
            r11 = 0
            java.lang.String r12 = r14.substring(r11, r12)
            goto L_0x0088
        L_0x0084:
            r19 = r11
            r11 = 0
            r12 = r14
        L_0x0088:
            boolean r11 = r4.returnRaw
            if (r11 == 0) goto L_0x0098
            r21 = r15
            r11 = 1
            java.lang.String[] r15 = new java.lang.String[r11]
            java.lang.String r11 = ""
            r16 = 0
            r15[r16] = r11
            goto L_0x00a0
        L_0x0098:
            r21 = r15
            java.lang.Class<?> r11 = r4.cls
            java.lang.String[] r15 = r0.cppTypeName(r11)
        L_0x00a0:
            r11 = r15
            java.lang.String r15 = valueTypeName(r11)
            r22 = r11
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            java.lang.String r2 = "JavaCPP_"
            r11.append(r2)
            java.lang.String r2 = mangle(r15)
            r11.append(r2)
            java.lang.String r2 = r11.toString()
            java.util.Map<java.lang.Class, java.util.Set<java.lang.String>> r11 = r0.virtualMembers
            java.lang.Object r11 = r11.get(r1)
            java.util.Set r11 = (java.util.Set) r11
            if (r11 != 0) goto L_0x00d7
            r23 = r11
            java.util.Map<java.lang.Class, java.util.Set<java.lang.String>> r11 = r0.virtualMembers
            r24 = r8
            java.util.LinkedHashSet r8 = new java.util.LinkedHashSet
            r8.<init>()
            r16 = r8
            r11.put(r1, r8)
            goto L_0x00dd
        L_0x00d7:
            r24 = r8
            r23 = r11
            r8 = r23
        L_0x00dd:
            java.lang.String r11 = "    "
            r25 = r7
            boolean r7 = r4.arrayAllocator
            if (r7 == 0) goto L_0x00e6
            return
        L_0x00e6:
            boolean r7 = r4.allocator
            if (r7 == 0) goto L_0x0156
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            r7.append(r11)
            r7.append(r2)
            r7.append(r12)
            r26 = r9
            java.lang.String r9 = " : "
            r7.append(r9)
            r7.append(r15)
            java.lang.String r9 = "("
            r7.append(r9)
            java.lang.String r7 = r7.toString()
            r9 = r7
            r7 = 0
        L_0x010d:
            int r11 = r6.length
            if (r7 >= r11) goto L_0x0140
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            r11.append(r9)
            java.lang.String r3 = "arg"
            r11.append(r3)
            r11.append(r7)
            java.lang.String r3 = r11.toString()
            int r9 = r6.length
            r11 = 1
            int r9 = r9 - r11
            if (r7 >= r9) goto L_0x013a
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            r9.append(r3)
            java.lang.String r11 = ", "
            r9.append(r11)
            java.lang.String r3 = r9.toString()
        L_0x013a:
            r9 = r3
            int r7 = r7 + 1
            r3 = r47
            goto L_0x010d
        L_0x0140:
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            r3.append(r9)
            java.lang.String r7 = "), obj(NULL) { }"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0 = r3
            r1 = r21
            goto L_0x0313
        L_0x0156:
            r26 = r9
            java.util.Map<java.lang.Class, java.util.Set<java.lang.String>> r3 = r0.virtualFunctions
            java.lang.Object r3 = r3.get(r1)
            java.util.Set r3 = (java.util.Set) r3
            if (r3 != 0) goto L_0x016d
            java.util.Map<java.lang.Class, java.util.Set<java.lang.String>> r7 = r0.virtualFunctions
            java.util.LinkedHashSet r9 = new java.util.LinkedHashSet
            r9.<init>()
            r3 = r9
            r7.put(r1, r9)
        L_0x016d:
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r9 = "using "
            r7.append(r9)
            r7.append(r15)
            java.lang.String r9 = "::"
            r7.append(r9)
            java.lang.String[] r9 = r4.memberName
            r16 = 0
            r9 = r9[r16]
            r7.append(r9)
            java.lang.String r9 = ";"
            r7.append(r9)
            java.lang.String r7 = r7.toString()
            r9 = 1
            java.util.Iterator r16 = r8.iterator()
        L_0x0196:
            boolean r20 = r16.hasNext()
            if (r20 == 0) goto L_0x01d2
            java.lang.Object r20 = r16.next()
            r27 = r9
            r9 = r20
            java.lang.String r9 = (java.lang.String) r9
            java.lang.String r1 = "\n"
            r0 = 2
            java.lang.String[] r0 = r9.split(r1, r0)
            r1 = 0
            r0 = r0[r1]
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            r1.append(r11)
            r1.append(r7)
            java.lang.String r1 = r1.toString()
            boolean r0 = r0.equals(r1)
            if (r0 == 0) goto L_0x01ca
            r0 = 0
            r27 = r0
            goto L_0x01d4
        L_0x01ca:
            r9 = r27
            r0 = r44
            r1 = r45
            goto L_0x0196
        L_0x01d2:
            r27 = r9
        L_0x01d4:
            if (r27 == 0) goto L_0x01ea
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r11)
            r0.append(r7)
            java.lang.String r1 = "\n    "
            r0.append(r1)
            java.lang.String r11 = r0.toString()
        L_0x01ea:
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r11)
            java.lang.String r1 = "virtual "
            r0.append(r1)
            r1 = 0
            r9 = r13[r1]
            r0.append(r9)
            int r1 = r13.length
            r9 = 1
            if (r1 <= r9) goto L_0x0204
            r1 = r13[r9]
            goto L_0x0206
        L_0x0204:
            java.lang.String r1 = ""
        L_0x0206:
            r0.append(r1)
            java.lang.String[] r1 = r4.memberName
            r9 = 0
            r1 = r1[r9]
            r0.append(r1)
            r0.append(r14)
            java.lang.String r1 = ";\n    "
            r0.append(r1)
            r1 = r13[r9]
            r0.append(r1)
            java.lang.String r1 = "super_"
            r0.append(r1)
            java.lang.String[] r1 = r4.memberName
            r1 = r1[r9]
            r0.append(r1)
            r0.append(r12)
            java.lang.String r1 = " { "
            r0.append(r1)
            java.lang.String r0 = r0.toString()
            java.lang.reflect.Method r1 = r4.method
            java.lang.Class<org.bytedeco.javacpp.annotation.Virtual> r9 = org.bytedeco.javacpp.annotation.Virtual.class
            java.lang.annotation.Annotation r1 = r1.getAnnotation(r9)
            org.bytedeco.javacpp.annotation.Virtual r1 = (org.bytedeco.javacpp.annotation.Virtual) r1
            boolean r1 = r1.value()
            if (r1 == 0) goto L_0x026d
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            r1.append(r0)
            java.lang.String r9 = "throw JavaCPP_exception(\"Cannot call pure virtual function "
            r1.append(r9)
            r1.append(r15)
            java.lang.String r9 = "::"
            r1.append(r9)
            java.lang.String[] r9 = r4.memberName
            r11 = 0
            r9 = r9[r11]
            r1.append(r9)
            java.lang.String r9 = "().\"); }"
            r1.append(r9)
            java.lang.String r0 = r1.toString()
            goto L_0x02dd
        L_0x026d:
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            r1.append(r0)
            java.lang.Class r9 = java.lang.Void.TYPE
            if (r5 == r9) goto L_0x027c
            java.lang.String r9 = "return "
            goto L_0x027e
        L_0x027c:
            java.lang.String r9 = ""
        L_0x027e:
            r1.append(r9)
            r1.append(r15)
            java.lang.String r9 = "::"
            r1.append(r9)
            java.lang.String[] r9 = r4.memberName
            r11 = 0
            r9 = r9[r11]
            r1.append(r9)
            java.lang.String r9 = "("
            r1.append(r9)
            java.lang.String r0 = r1.toString()
            r1 = r0
            r0 = 0
        L_0x029c:
            int r9 = r6.length
            if (r0 >= r9) goto L_0x02cc
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            r9.append(r1)
            java.lang.String r11 = "arg"
            r9.append(r11)
            r9.append(r0)
            java.lang.String r1 = r9.toString()
            int r9 = r6.length
            r11 = 1
            int r9 = r9 - r11
            if (r0 >= r9) goto L_0x02c9
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            r9.append(r1)
            java.lang.String r11 = ", "
            r9.append(r11)
            java.lang.String r1 = r9.toString()
        L_0x02c9:
            int r0 = r0 + 1
            goto L_0x029c
        L_0x02cc:
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r1)
            java.lang.String r9 = "); }"
            r0.append(r9)
            java.lang.String r0 = r0.toString()
        L_0x02dd:
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            r9 = 0
            r11 = r13[r9]
            r1.append(r11)
            int r9 = r13.length
            r11 = 1
            if (r9 <= r11) goto L_0x02ef
            r9 = r13[r11]
            goto L_0x02f1
        L_0x02ef:
            java.lang.String r9 = ""
        L_0x02f1:
            r1.append(r9)
            r1.append(r2)
            java.lang.String r9 = "::"
            r1.append(r9)
            java.lang.String[] r9 = r4.memberName
            r11 = 0
            r9 = r9[r11]
            r1.append(r9)
            r1.append(r14)
            java.lang.String r9 = " {"
            r1.append(r9)
            java.lang.String r1 = r1.toString()
            r3.add(r10)
        L_0x0313:
            r8.add(r0)
            r15 = r1
            r7 = r26
            r0 = r47
            r1 = r44
            r3 = r45
            goto L_0x04b6
        L_0x0322:
            r25 = r7
            r24 = r8
            r26 = r9
            r19 = r11
            r21 = r15
            r0 = r47
            if (r0 == 0) goto L_0x04ae
            r1 = r44
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.String> r2 = r1.callbacks
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "static "
            r3.append(r7)
            r7 = r26
            r3.append(r7)
            java.lang.String r8 = " "
            r3.append(r8)
            r3.append(r0)
            java.lang.String r8 = "_instance;"
            r3.append(r8)
            java.lang.String r3 = r3.toString()
            r2.index(r3)
            java.lang.Class<org.bytedeco.javacpp.annotation.Convention> r2 = org.bytedeco.javacpp.annotation.Convention.class
            r3 = r45
            java.lang.annotation.Annotation r2 = r3.getAnnotation(r2)
            org.bytedeco.javacpp.annotation.Convention r2 = (org.bytedeco.javacpp.annotation.Convention) r2
            if (r2 == 0) goto L_0x03b1
            java.lang.String r8 = r2.extern()
            java.lang.String r9 = "C"
            boolean r8 = r8.equals(r9)
            if (r8 != 0) goto L_0x03b1
            java.io.PrintWriter r8 = r1.out
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            java.lang.String r11 = "extern \""
            r9.append(r11)
            java.lang.String r11 = r2.extern()
            r9.append(r11)
            java.lang.String r11 = "\" {"
            r9.append(r11)
            java.lang.String r9 = r9.toString()
            r8.println(r9)
            java.io.PrintWriter r8 = r1.out2
            if (r8 == 0) goto L_0x03b1
            java.io.PrintWriter r8 = r1.out2
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            java.lang.String r11 = "extern \""
            r9.append(r11)
            java.lang.String r11 = r2.extern()
            r9.append(r11)
            java.lang.String r11 = "\" {"
            r9.append(r11)
            java.lang.String r9 = r9.toString()
            r8.println(r9)
        L_0x03b1:
            java.io.PrintWriter r8 = r1.out2
            if (r8 == 0) goto L_0x03e5
            java.io.PrintWriter r8 = r1.out2
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            java.lang.String r11 = "JNIIMPORT "
            r9.append(r11)
            r11 = 0
            r12 = r13[r11]
            r9.append(r12)
            int r11 = r13.length
            r12 = 1
            if (r11 <= r12) goto L_0x03ce
            r11 = r13[r12]
            goto L_0x03d0
        L_0x03ce:
            java.lang.String r11 = ""
        L_0x03d0:
            r9.append(r11)
            r9.append(r0)
            r9.append(r14)
            java.lang.String r11 = ";"
            r9.append(r11)
            java.lang.String r9 = r9.toString()
            r8.println(r9)
        L_0x03e5:
            java.io.PrintWriter r8 = r1.out
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            java.lang.String r11 = "JNIEXPORT "
            r9.append(r11)
            r11 = 0
            r12 = r13[r11]
            r9.append(r12)
            int r11 = r13.length
            r12 = 1
            if (r11 <= r12) goto L_0x03fe
            r11 = r13[r12]
            goto L_0x0400
        L_0x03fe:
            java.lang.String r11 = ""
        L_0x0400:
            r9.append(r11)
            r9.append(r0)
            r9.append(r14)
            java.lang.String r11 = " {"
            r9.append(r11)
            java.lang.String r9 = r9.toString()
            r8.println(r9)
            java.io.PrintWriter r8 = r1.out
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            java.lang.Class r11 = java.lang.Void.TYPE
            if (r5 == r11) goto L_0x0423
            java.lang.String r11 = "    return "
            goto L_0x0425
        L_0x0423:
            java.lang.String r11 = "    "
        L_0x0425:
            r9.append(r11)
            r9.append(r0)
            java.lang.String r11 = "_instance("
            r9.append(r11)
            java.lang.String r9 = r9.toString()
            r8.print(r9)
            r8 = 0
        L_0x0438:
            int r9 = r6.length
            if (r8 >= r9) goto L_0x0460
            java.io.PrintWriter r9 = r1.out
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            java.lang.String r12 = "arg"
            r11.append(r12)
            r11.append(r8)
            java.lang.String r11 = r11.toString()
            r9.print(r11)
            int r9 = r6.length
            r11 = 1
            int r9 = r9 - r11
            if (r8 >= r9) goto L_0x045d
            java.io.PrintWriter r9 = r1.out
            java.lang.String r11 = ", "
            r9.print(r11)
        L_0x045d:
            int r8 = r8 + 1
            goto L_0x0438
        L_0x0460:
            java.io.PrintWriter r8 = r1.out
            java.lang.String r9 = ");"
            r8.println(r9)
            java.io.PrintWriter r8 = r1.out
            java.lang.String r9 = "}"
            r8.println(r9)
            if (r2 == 0) goto L_0x048e
            java.lang.String r8 = r2.extern()
            java.lang.String r9 = "C"
            boolean r8 = r8.equals(r9)
            if (r8 != 0) goto L_0x048e
            java.io.PrintWriter r8 = r1.out
            java.lang.String r9 = "}"
            r8.println(r9)
            java.io.PrintWriter r8 = r1.out2
            if (r8 == 0) goto L_0x048e
            java.io.PrintWriter r8 = r1.out2
            java.lang.String r9 = "}"
            r8.println(r9)
        L_0x048e:
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            r9 = 0
            r11 = r13[r9]
            r8.append(r11)
            r8.append(r7)
            java.lang.String r9 = "::operator()"
            r8.append(r9)
            r8.append(r14)
            java.lang.String r9 = " {"
            r8.append(r9)
            java.lang.String r15 = r8.toString()
            goto L_0x04b6
        L_0x04ae:
            r7 = r26
            r1 = r44
            r3 = r45
            r15 = r21
        L_0x04b6:
            if (r48 != 0) goto L_0x04b9
            return
        L_0x04b9:
            java.io.PrintWriter r8 = r1.out
            r8.println(r15)
            java.lang.String r8 = ""
            java.lang.Class r9 = java.lang.Void.TYPE
            if (r5 == r9) goto L_0x04fa
            java.io.PrintWriter r9 = r1.out
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            java.lang.String r12 = "    "
            r11.append(r12)
            java.lang.String r12 = jniTypeName(r5)
            r11.append(r12)
            java.lang.String r12 = " rarg = 0;"
            r11.append(r12)
            java.lang.String r11 = r11.toString()
            r9.println(r11)
            java.lang.String r8 = "rarg = "
            java.lang.Class<java.lang.String> r9 = java.lang.String.class
            if (r5 != r9) goto L_0x04fa
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            r9.append(r8)
            java.lang.String r11 = "(jstring)"
            r9.append(r11)
            java.lang.String r8 = r9.toString()
        L_0x04fa:
            r9 = r25
            java.lang.String r11 = r1.cast((java.lang.Class<?>) r5, (java.lang.annotation.Annotation[]) r9)
            java.lang.annotation.Annotation r12 = r1.by(r9)
            java.lang.String[] r16 = r1.cppTypeName(r5)
            java.lang.String r2 = valueTypeName(r16)
            r28 = r7
            r29 = r14
            r7 = 0
            org.bytedeco.javacpp.tools.AdapterInformation r14 = r1.adapterInformation((boolean) r7, (java.lang.String) r2, (java.lang.annotation.Annotation[]) r9)
            boolean r7 = noException(r45, r46)
            r17 = 1
            r7 = r7 ^ 1
            if (r7 == 0) goto L_0x052b
            r30 = r2
            java.io.PrintWriter r2 = r1.out
            r31 = r9
            java.lang.String r9 = "    jthrowable exc = NULL;"
            r2.println(r9)
            goto L_0x052f
        L_0x052b:
            r30 = r2
            r31 = r9
        L_0x052f:
            java.io.PrintWriter r2 = r1.out
            java.lang.String r9 = "    JNIEnv* env;"
            r2.println(r9)
            java.io.PrintWriter r2 = r1.out
            java.lang.String r9 = "    bool attached = JavaCPP_getEnv(&env);"
            r2.println(r9)
            java.io.PrintWriter r2 = r1.out
            java.lang.String r9 = "    if (env == NULL) {"
            r2.println(r9)
            java.io.PrintWriter r2 = r1.out
            java.lang.String r9 = "        goto end;"
            r2.println(r9)
            java.io.PrintWriter r2 = r1.out
            java.lang.String r9 = "    }"
            r2.println(r9)
            java.io.PrintWriter r2 = r1.out
            java.lang.String r9 = "{"
            r2.println(r9)
            int r2 = r6.length
            if (r2 <= 0) goto L_0x0f11
            java.io.PrintWriter r2 = r1.out
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            r32 = r15
            java.lang.String r15 = "    jvalue args["
            r9.append(r15)
            int r15 = r6.length
            r9.append(r15)
            java.lang.String r15 = "];"
            r9.append(r15)
            java.lang.String r9 = r9.toString()
            r2.println(r9)
            r2 = 0
        L_0x057b:
            int r9 = r6.length
            if (r2 >= r9) goto L_0x0f00
            r9 = r24[r2]
            java.lang.annotation.Annotation r9 = r1.by(r9)
            r15 = r6[r2]
            boolean r15 = r15.isPrimitive()
            if (r15 == 0) goto L_0x05f9
            java.io.PrintWriter r15 = r1.out
            r33 = r13
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            r34 = r12
            java.lang.String r12 = "    args["
            r13.append(r12)
            r13.append(r2)
            java.lang.String r12 = "]."
            r13.append(r12)
            r35 = r11
            r12 = 1
            java.lang.Class[] r11 = new java.lang.Class[r12]
            r12 = r6[r2]
            r18 = 0
            r11[r18] = r12
            java.lang.String r11 = signature(r11)
            java.lang.String r11 = r11.toLowerCase()
            r13.append(r11)
            java.lang.String r11 = " = ("
            r13.append(r11)
            r11 = r6[r2]
            java.lang.String r11 = jniTypeName(r11)
            r13.append(r11)
            java.lang.String r11 = ")"
            r13.append(r11)
            boolean r11 = r9 instanceof org.bytedeco.javacpp.annotation.ByPtr
            if (r11 != 0) goto L_0x05d9
            boolean r11 = r9 instanceof org.bytedeco.javacpp.annotation.ByPtrRef
            if (r11 == 0) goto L_0x05d6
            goto L_0x05d9
        L_0x05d6:
            java.lang.String r11 = "arg"
            goto L_0x05db
        L_0x05d9:
            java.lang.String r11 = "*arg"
        L_0x05db:
            r13.append(r11)
            r13.append(r2)
            java.lang.String r11 = ";"
            r13.append(r11)
            java.lang.String r11 = r13.toString()
            r15.println(r11)
            r39 = r5
            r37 = r7
            r38 = r8
            r36 = r14
        L_0x05f5:
            r15 = r46
            goto L_0x0eee
        L_0x05f9:
            r35 = r11
            r34 = r12
            r33 = r13
            java.lang.Class<java.lang.Enum> r11 = java.lang.Enum.class
            r12 = r6[r2]
            boolean r11 = r11.isAssignableFrom(r12)
            if (r11 == 0) goto L_0x06eb
            r11 = 1
            r1.accessesEnums = r11
            r11 = r6[r2]
            java.lang.String r11 = r1.enumValueType(r11)
            if (r11 == 0) goto L_0x06e1
            java.lang.StringBuilder r12 = new java.lang.StringBuilder
            r12.<init>()
            r13 = 0
            char r15 = r11.charAt(r13)
            char r13 = java.lang.Character.toUpperCase(r15)
            r12.append(r13)
            r13 = 1
            java.lang.String r15 = r11.substring(r13)
            r12.append(r15)
            java.lang.String r12 = r12.toString()
            java.io.PrintWriter r13 = r1.out
            java.lang.StringBuilder r15 = new java.lang.StringBuilder
            r15.<init>()
            r36 = r14
            java.lang.String r14 = "    jobject obj"
            r15.append(r14)
            r15.append(r2)
            java.lang.String r14 = " = JavaCPP_createPointer(env, "
            r15.append(r14)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r14 = r1.jclasses
            r37 = r7
            r7 = r6[r2]
            int r7 = r14.index(r7)
            r15.append(r7)
            java.lang.String r7 = ");"
            r15.append(r7)
            java.lang.String r7 = r15.toString()
            r13.println(r7)
            java.io.PrintWriter r7 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "    args["
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = "].l = obj"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = ";"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r7.println(r13)
            java.io.PrintWriter r7 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "    if (obj"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = " != NULL) {"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r7.println(r13)
            java.io.PrintWriter r7 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "        env->Set"
            r13.append(r14)
            r13.append(r12)
            java.lang.String r14 = "Field(obj"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = ", JavaCPP_"
            r13.append(r14)
            r13.append(r11)
            java.lang.String r14 = "ValueFID, (j"
            r13.append(r14)
            r13.append(r11)
            java.lang.String r14 = ")arg"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = ");"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r7.println(r13)
            java.io.PrintWriter r7 = r1.out
            java.lang.String r13 = "    }"
            r7.println(r13)
            goto L_0x06e5
        L_0x06e1:
            r37 = r7
            r36 = r14
        L_0x06e5:
            r39 = r5
            r38 = r8
            goto L_0x05f5
        L_0x06eb:
            r37 = r7
            r36 = r14
            r7 = r6[r2]
            java.lang.String[] r7 = r1.cppTypeName(r7)
            java.lang.String r11 = valueTypeName(r7)
            r12 = r24[r2]
            r13 = 0
            org.bytedeco.javacpp.tools.AdapterInformation r12 = r1.adapterInformation((boolean) r13, (java.lang.String) r11, (java.lang.annotation.Annotation[]) r12)
            if (r12 == 0) goto L_0x0732
            r13 = 1
            r1.usesAdapters = r13
            java.io.PrintWriter r13 = r1.out
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            java.lang.String r15 = "    "
            r14.append(r15)
            java.lang.String r15 = r12.name
            r14.append(r15)
            java.lang.String r15 = " adapter"
            r14.append(r15)
            r14.append(r2)
            java.lang.String r15 = "(arg"
            r14.append(r15)
            r14.append(r2)
            java.lang.String r15 = ");"
            r14.append(r15)
            java.lang.String r14 = r14.toString()
            r13.println(r14)
        L_0x0732:
            java.lang.Class<org.bytedeco.javacpp.Pointer> r13 = org.bytedeco.javacpp.Pointer.class
            r14 = r6[r2]
            boolean r13 = r13.isAssignableFrom(r14)
            if (r13 != 0) goto L_0x0763
            java.lang.Class<java.nio.Buffer> r13 = java.nio.Buffer.class
            r14 = r6[r2]
            boolean r13 = r13.isAssignableFrom(r14)
            if (r13 != 0) goto L_0x0763
            r13 = r6[r2]
            boolean r13 = r13.isArray()
            if (r13 == 0) goto L_0x075b
            r13 = r6[r2]
            java.lang.Class r13 = r13.getComponentType()
            boolean r13 = r13.isPrimitive()
            if (r13 == 0) goto L_0x075b
            goto L_0x0763
        L_0x075b:
            r39 = r5
            r38 = r8
            r15 = r46
            goto L_0x0a11
        L_0x0763:
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "("
            r13.append(r14)
            r14 = 0
            r15 = r7[r14]
            r13.append(r15)
            r14 = 1
            r15 = r7[r14]
            r13.append(r15)
            java.lang.String r14 = ")"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            java.lang.Class<org.bytedeco.javacpp.FunctionPointer> r14 = org.bytedeco.javacpp.FunctionPointer.class
            r15 = r6[r2]
            boolean r14 = r14.isAssignableFrom(r15)
            if (r14 == 0) goto L_0x07b7
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r14 = r1.functions
            r15 = r6[r2]
            r14.index(r15)
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            r15 = r6[r2]
            java.lang.String r15 = functionClassName(r15)
            r14.append(r15)
            java.lang.String r15 = "*"
            r14.append(r15)
            java.lang.String r14 = r14.toString()
            r15 = 0
            r7[r15] = r14
            java.lang.String r14 = ""
            r15 = 1
            r7[r15] = r14
            java.lang.String r11 = valueTypeName(r7)
            goto L_0x07d7
        L_0x07b7:
            java.util.Map<java.lang.Class, java.util.Set<java.lang.String>> r14 = r1.virtualFunctions
            r15 = r6[r2]
            boolean r14 = r14.containsKey(r15)
            if (r14 == 0) goto L_0x07d7
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            java.lang.String r15 = "JavaCPP_"
            r14.append(r15)
            java.lang.String r15 = mangle(r11)
            r14.append(r15)
            java.lang.String r14 = r14.toString()
            r11 = r14
        L_0x07d7:
            java.io.PrintWriter r14 = r1.out
            java.lang.StringBuilder r15 = new java.lang.StringBuilder
            r15.<init>()
            r38 = r8
            java.lang.String r8 = "    "
            r15.append(r8)
            r8 = r6[r2]
            java.lang.String r8 = jniTypeName(r8)
            r15.append(r8)
            java.lang.String r8 = " obj"
            r15.append(r8)
            r15.append(r2)
            java.lang.String r8 = " = NULL;"
            r15.append(r8)
            java.lang.String r8 = r15.toString()
            r14.println(r8)
            java.io.PrintWriter r8 = r1.out
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            java.lang.String r15 = "    "
            r14.append(r15)
            r39 = r5
            r15 = 0
            r5 = r7[r15]
            r14.append(r5)
            java.lang.String r5 = " ptr"
            r14.append(r5)
            r14.append(r2)
            r5 = 1
            r15 = r7[r5]
            r14.append(r15)
            java.lang.String r5 = " = NULL;"
            r14.append(r5)
            java.lang.String r5 = r14.toString()
            r8.println(r5)
            java.lang.Class<org.bytedeco.javacpp.FunctionPointer> r5 = org.bytedeco.javacpp.FunctionPointer.class
            r8 = r6[r2]
            boolean r5 = r5.isAssignableFrom(r8)
            if (r5 == 0) goto L_0x08b0
            java.io.PrintWriter r5 = r1.out
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            java.lang.String r14 = "    ptr"
            r8.append(r14)
            r8.append(r2)
            java.lang.String r14 = " = new (std::nothrow) "
            r8.append(r14)
            r8.append(r11)
            java.lang.String r14 = ";"
            r8.append(r14)
            java.lang.String r8 = r8.toString()
            r5.println(r8)
            java.io.PrintWriter r5 = r1.out
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            java.lang.String r14 = "    if (ptr"
            r8.append(r14)
            r8.append(r2)
            java.lang.String r14 = " != NULL) {"
            r8.append(r14)
            java.lang.String r8 = r8.toString()
            r5.println(r8)
            java.io.PrintWriter r5 = r1.out
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            java.lang.String r14 = "        ptr"
            r8.append(r14)
            r8.append(r2)
            java.lang.String r14 = "->ptr = "
            r8.append(r14)
            r8.append(r13)
            java.lang.String r14 = "&arg"
            r8.append(r14)
            r8.append(r2)
            java.lang.String r14 = ";"
            r8.append(r14)
            java.lang.String r8 = r8.toString()
            r5.println(r8)
            java.io.PrintWriter r5 = r1.out
            java.lang.String r8 = "    }"
            r5.println(r8)
        L_0x08aa:
            r40 = r11
            r15 = r46
            goto L_0x0a0f
        L_0x08b0:
            if (r12 == 0) goto L_0x08d6
            java.io.PrintWriter r5 = r1.out
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            java.lang.String r14 = "    ptr"
            r8.append(r14)
            r8.append(r2)
            java.lang.String r14 = " = adapter"
            r8.append(r14)
            r8.append(r2)
            java.lang.String r14 = ";"
            r8.append(r14)
            java.lang.String r8 = r8.toString()
            r5.println(r8)
            goto L_0x08aa
        L_0x08d6:
            boolean r5 = r9 instanceof org.bytedeco.javacpp.annotation.ByVal
            if (r5 == 0) goto L_0x092a
            r5 = r6[r2]
            java.lang.Class<org.bytedeco.javacpp.Pointer> r8 = org.bytedeco.javacpp.Pointer.class
            if (r5 == r8) goto L_0x092a
            java.io.PrintWriter r5 = r1.out
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            java.lang.String r14 = "    ptr"
            r8.append(r14)
            r8.append(r2)
            r14 = r6[r2]
            r15 = r46
            boolean r14 = noException(r14, r15)
            if (r14 == 0) goto L_0x08fc
            java.lang.String r14 = " = new (std::nothrow) "
            goto L_0x08fe
        L_0x08fc:
            java.lang.String r14 = " = new "
        L_0x08fe:
            r8.append(r14)
            r8.append(r11)
            r40 = r11
            r14 = 1
            r11 = r7[r14]
            r8.append(r11)
            java.lang.String r11 = "(*"
            r8.append(r11)
            r8.append(r13)
            java.lang.String r11 = "&arg"
            r8.append(r11)
            r8.append(r2)
            java.lang.String r11 = ");"
            r8.append(r11)
            java.lang.String r8 = r8.toString()
            r5.println(r8)
            goto L_0x0a0f
        L_0x092a:
            r40 = r11
            r15 = r46
            boolean r5 = r9 instanceof org.bytedeco.javacpp.annotation.ByVal
            if (r5 != 0) goto L_0x09e4
            boolean r5 = r9 instanceof org.bytedeco.javacpp.annotation.ByRef
            if (r5 == 0) goto L_0x0938
            goto L_0x09e4
        L_0x0938:
            boolean r5 = r9 instanceof org.bytedeco.javacpp.annotation.ByPtrPtr
            if (r5 == 0) goto L_0x09b8
            java.io.PrintWriter r5 = r1.out
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            java.lang.String r11 = "    if (arg"
            r8.append(r11)
            r8.append(r2)
            java.lang.String r11 = " == NULL) {"
            r8.append(r11)
            java.lang.String r8 = r8.toString()
            r5.println(r8)
            java.io.PrintWriter r5 = r1.out
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            java.lang.String r11 = "        JavaCPP_log(\"Pointer address of argument "
            r8.append(r11)
            r8.append(r2)
            java.lang.String r11 = " is NULL in callback for "
            r8.append(r11)
            java.lang.String r11 = r45.getCanonicalName()
            r8.append(r11)
            java.lang.String r11 = ".\");"
            r8.append(r11)
            java.lang.String r8 = r8.toString()
            r5.println(r8)
            java.io.PrintWriter r5 = r1.out
            java.lang.String r8 = "    } else {"
            r5.println(r8)
            java.io.PrintWriter r5 = r1.out
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            java.lang.String r11 = "        ptr"
            r8.append(r11)
            r8.append(r2)
            java.lang.String r11 = " = "
            r8.append(r11)
            r8.append(r13)
            java.lang.String r11 = "*arg"
            r8.append(r11)
            r8.append(r2)
            java.lang.String r11 = ";"
            r8.append(r11)
            java.lang.String r8 = r8.toString()
            r5.println(r8)
            java.io.PrintWriter r5 = r1.out
            java.lang.String r8 = "    }"
            r5.println(r8)
            goto L_0x0a0f
        L_0x09b8:
            java.io.PrintWriter r5 = r1.out
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            java.lang.String r11 = "    ptr"
            r8.append(r11)
            r8.append(r2)
            java.lang.String r11 = " = "
            r8.append(r11)
            r8.append(r13)
            java.lang.String r11 = "arg"
            r8.append(r11)
            r8.append(r2)
            java.lang.String r11 = ";"
            r8.append(r11)
            java.lang.String r8 = r8.toString()
            r5.println(r8)
            goto L_0x0a0f
        L_0x09e4:
            java.io.PrintWriter r5 = r1.out
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            java.lang.String r11 = "    ptr"
            r8.append(r11)
            r8.append(r2)
            java.lang.String r11 = " = "
            r8.append(r11)
            r8.append(r13)
            java.lang.String r11 = "&arg"
            r8.append(r11)
            r8.append(r2)
            java.lang.String r11 = ";"
            r8.append(r11)
            java.lang.String r8 = r8.toString()
            r5.println(r8)
        L_0x0a0f:
            r11 = r40
        L_0x0a11:
            r5 = 0
            if (r12 == 0) goto L_0x0a88
            r8 = r6[r2]
            java.lang.Class<java.lang.String> r13 = java.lang.String.class
            if (r8 == r13) goto L_0x0a85
            java.io.PrintWriter r8 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "    jlong size"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = " = (jlong)adapter"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = ".size;"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r8.println(r13)
            java.io.PrintWriter r8 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "    void* owner"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = " = adapter"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = ".owner;"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r8.println(r13)
            java.io.PrintWriter r8 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "    void (*deallocator"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = ")(void*) = &"
            r13.append(r14)
            java.lang.String r14 = r12.name
            r13.append(r14)
            java.lang.String r14 = "::deallocate;"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r8.println(r13)
        L_0x0a85:
            r5 = 1
            goto L_0x0b0f
        L_0x0a88:
            boolean r8 = r9 instanceof org.bytedeco.javacpp.annotation.ByVal
            if (r8 == 0) goto L_0x0a92
            r8 = r6[r2]
            java.lang.Class<org.bytedeco.javacpp.Pointer> r13 = org.bytedeco.javacpp.Pointer.class
            if (r8 != r13) goto L_0x0a9c
        L_0x0a92:
            java.lang.Class<org.bytedeco.javacpp.FunctionPointer> r8 = org.bytedeco.javacpp.FunctionPointer.class
            r13 = r6[r2]
            boolean r8 = r8.isAssignableFrom(r13)
            if (r8 == 0) goto L_0x0b0f
        L_0x0a9c:
            java.io.PrintWriter r8 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "    jlong size"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = " = 1;"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r8.println(r13)
            java.io.PrintWriter r8 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "    void* owner"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = " = ptr"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = ";"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r8.println(r13)
            java.io.PrintWriter r8 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "    void (*deallocator"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = ")(void*) = &JavaCPP_"
            r13.append(r14)
            r14 = r6[r2]
            java.lang.String r14 = r14.getName()
            java.lang.String r14 = mangle(r14)
            r13.append(r14)
            java.lang.String r14 = "_deallocate;"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r8.println(r13)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r8 = r1.deallocators
            r13 = r6[r2]
            r8.index(r13)
            r5 = 1
        L_0x0b0f:
            java.lang.Class<org.bytedeco.javacpp.Pointer> r8 = org.bytedeco.javacpp.Pointer.class
            r13 = r6[r2]
            boolean r8 = r8.isAssignableFrom(r13)
            if (r8 == 0) goto L_0x0c3d
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            java.lang.String r13 = "    obj"
            r8.append(r13)
            r8.append(r2)
            java.lang.String r13 = " = JavaCPP_createPointer(env, "
            r8.append(r13)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r13 = r1.jclasses
            r14 = r6[r2]
            int r13 = r13.index(r14)
            r8.append(r13)
            java.lang.String r13 = ");"
            r8.append(r13)
            java.lang.String r8 = r8.toString()
            r13 = r24[r2]
            r14 = 1
            org.bytedeco.javacpp.tools.AdapterInformation r12 = r1.adapterInformation((boolean) r14, (java.lang.String) r11, (java.lang.annotation.Annotation[]) r13)
            if (r12 != 0) goto L_0x0b8e
            boolean r13 = r9 instanceof org.bytedeco.javacpp.annotation.ByPtrPtr
            if (r13 != 0) goto L_0x0b8e
            boolean r13 = r9 instanceof org.bytedeco.javacpp.annotation.ByPtrRef
            if (r13 == 0) goto L_0x0b53
            r41 = r7
            goto L_0x0b90
        L_0x0b53:
            java.io.PrintWriter r13 = r1.out
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            r41 = r7
            java.lang.String r7 = "    if (ptr"
            r14.append(r7)
            r14.append(r2)
            java.lang.String r7 = " != NULL) { "
            r14.append(r7)
            java.lang.String r7 = r14.toString()
            r13.println(r7)
            java.io.PrintWriter r7 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "    "
            r13.append(r14)
            r13.append(r8)
            java.lang.String r13 = r13.toString()
            r7.println(r13)
            java.io.PrintWriter r7 = r1.out
            java.lang.String r13 = "    }"
            r7.println(r13)
            goto L_0x0b95
        L_0x0b8e:
            r41 = r7
        L_0x0b90:
            java.io.PrintWriter r7 = r1.out
            r7.println(r8)
        L_0x0b95:
            java.io.PrintWriter r7 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "    if (obj"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = " != NULL) { "
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r7.println(r13)
            if (r5 == 0) goto L_0x0bee
            java.io.PrintWriter r7 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "        JavaCPP_initPointer(env, obj"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = ", ptr"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = ", size"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = ", owner"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = ", deallocator"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = ");"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r7.println(r13)
            goto L_0x0c11
        L_0x0bee:
            java.io.PrintWriter r7 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "        env->SetLongField(obj"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = ", JavaCPP_addressFID, ptr_to_jlong(ptr"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = "));"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r7.println(r13)
        L_0x0c11:
            java.io.PrintWriter r7 = r1.out
            java.lang.String r13 = "    }"
            r7.println(r13)
            java.io.PrintWriter r7 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "    args["
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = "].l = obj"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = ";"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r7.println(r13)
            goto L_0x0eee
        L_0x0c3d:
            r41 = r7
            r7 = r6[r2]
            java.lang.Class<java.lang.String> r8 = java.lang.String.class
            if (r7 != r8) goto L_0x0c9a
            r7 = 1
            r1.passesStrings = r7
            java.io.PrintWriter r7 = r1.out
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            java.lang.String r13 = "    jstring obj"
            r8.append(r13)
            r8.append(r2)
            java.lang.String r13 = " = JavaCPP_createString(env, (const char*)"
            r8.append(r13)
            if (r12 == 0) goto L_0x0c61
            java.lang.String r13 = "adapter"
            goto L_0x0c63
        L_0x0c61:
            java.lang.String r13 = "arg"
        L_0x0c63:
            r8.append(r13)
            r8.append(r2)
            java.lang.String r13 = ");"
            r8.append(r13)
            java.lang.String r8 = r8.toString()
            r7.println(r8)
            java.io.PrintWriter r7 = r1.out
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            java.lang.String r13 = "    args["
            r8.append(r13)
            r8.append(r2)
            java.lang.String r13 = "].l = obj"
            r8.append(r13)
            r8.append(r2)
            java.lang.String r13 = ";"
            r8.append(r13)
            java.lang.String r8 = r8.toString()
            r7.println(r8)
            goto L_0x0eee
        L_0x0c9a:
            r7 = r6[r2]
            boolean r7 = r7.isArray()
            if (r7 == 0) goto L_0x0e08
            r7 = r6[r2]
            java.lang.Class r7 = r7.getComponentType()
            boolean r7 = r7.isPrimitive()
            if (r7 == 0) goto L_0x0e08
            if (r12 != 0) goto L_0x0cd3
            java.io.PrintWriter r7 = r1.out
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            java.lang.String r13 = "    jlong size"
            r8.append(r13)
            r8.append(r2)
            java.lang.String r13 = " = ptr"
            r8.append(r13)
            r8.append(r2)
            java.lang.String r13 = " != NULL ? 1 : 0;"
            r8.append(r13)
            java.lang.String r8 = r8.toString()
            r7.println(r8)
        L_0x0cd3:
            r7 = r6[r2]
            java.lang.Class r7 = r7.getComponentType()
            java.lang.String r7 = r7.getName()
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            r13 = 0
            char r14 = r7.charAt(r13)
            char r13 = java.lang.Character.toUpperCase(r14)
            r8.append(r13)
            r13 = 1
            java.lang.String r14 = r7.substring(r13)
            r8.append(r14)
            java.lang.String r8 = r8.toString()
            java.io.PrintWriter r13 = r1.out
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            r42 = r5
            java.lang.String r5 = "    if (ptr"
            r14.append(r5)
            r14.append(r2)
            java.lang.String r5 = " != NULL) {"
            r14.append(r5)
            java.lang.String r5 = r14.toString()
            r13.println(r5)
            java.io.PrintWriter r5 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "        obj"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = " = env->New"
            r13.append(r14)
            r13.append(r8)
            java.lang.String r14 = "Array(size"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = " < INT_MAX ? size"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = " : INT_MAX);"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r5.println(r13)
            java.io.PrintWriter r5 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "        env->Set"
            r13.append(r14)
            r13.append(r8)
            java.lang.String r14 = "ArrayRegion(obj"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = ", 0, size"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = " < INT_MAX ? size"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = " : INT_MAX, (j"
            r13.append(r14)
            r13.append(r7)
            java.lang.String r14 = "*)ptr"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = ");"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r5.println(r13)
            java.io.PrintWriter r5 = r1.out
            java.lang.String r13 = "    }"
            r5.println(r13)
            if (r12 == 0) goto L_0x0de3
            java.io.PrintWriter r5 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "    if (deallocator"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = " != 0 && ptr"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = " != NULL) {"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r5.println(r13)
            java.io.PrintWriter r5 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "        (*(void(*)(void*))jlong_to_ptr(deallocator"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = "))((void*)ptr"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = ");"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r5.println(r13)
            java.io.PrintWriter r5 = r1.out
            java.lang.String r13 = "    }"
            r5.println(r13)
        L_0x0de3:
            java.io.PrintWriter r5 = r1.out
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            java.lang.String r14 = "    args["
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = "].l = obj"
            r13.append(r14)
            r13.append(r2)
            java.lang.String r14 = ";"
            r13.append(r14)
            java.lang.String r13 = r13.toString()
            r5.println(r13)
            goto L_0x0eee
        L_0x0e08:
            r42 = r5
            java.lang.Class<java.nio.Buffer> r5 = java.nio.Buffer.class
            r7 = r6[r2]
            boolean r5 = r5.isAssignableFrom(r7)
            if (r5 == 0) goto L_0x0ec5
            if (r12 != 0) goto L_0x0e39
            java.io.PrintWriter r5 = r1.out
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r8 = "    jlong size"
            r7.append(r8)
            r7.append(r2)
            java.lang.String r8 = " = ptr"
            r7.append(r8)
            r7.append(r2)
            java.lang.String r8 = " != NULL ? 1 : 0;"
            r7.append(r8)
            java.lang.String r7 = r7.toString()
            r5.println(r7)
        L_0x0e39:
            java.io.PrintWriter r5 = r1.out
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r8 = "    if (ptr"
            r7.append(r8)
            r7.append(r2)
            java.lang.String r8 = " != NULL) {"
            r7.append(r8)
            java.lang.String r7 = r7.toString()
            r5.println(r7)
            java.io.PrintWriter r5 = r1.out
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r8 = "        jlong sizeptr = size"
            r7.append(r8)
            r7.append(r2)
            java.lang.String r8 = " * sizeof(ptr"
            r7.append(r8)
            r7.append(r2)
            java.lang.String r8 = "[0]);"
            r7.append(r8)
            java.lang.String r7 = r7.toString()
            r5.println(r7)
            java.io.PrintWriter r5 = r1.out
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r8 = "        obj"
            r7.append(r8)
            r7.append(r2)
            java.lang.String r8 = " = env->NewDirectByteBuffer((void*)ptr"
            r7.append(r8)
            r7.append(r2)
            java.lang.String r8 = ", sizeptr < INT_MAX ? sizeptr : INT_MAX);"
            r7.append(r8)
            java.lang.String r7 = r7.toString()
            r5.println(r7)
            java.io.PrintWriter r5 = r1.out
            java.lang.String r7 = "    }"
            r5.println(r7)
            java.io.PrintWriter r5 = r1.out
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r8 = "    args["
            r7.append(r8)
            r7.append(r2)
            java.lang.String r8 = "].l = obj"
            r7.append(r8)
            r7.append(r2)
            java.lang.String r8 = ";"
            r7.append(r8)
            java.lang.String r7 = r7.toString()
            r5.println(r7)
            goto L_0x0eee
        L_0x0ec5:
            org.bytedeco.javacpp.tools.Logger r5 = r1.logger
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r8 = "Callback \""
            r7.append(r8)
            r7.append(r15)
            java.lang.String r8 = "\" has unsupported parameter type \""
            r7.append(r8)
            r8 = r6[r2]
            java.lang.String r8 = r8.getCanonicalName()
            r7.append(r8)
            java.lang.String r8 = "\". Compilation will most likely fail."
            r7.append(r8)
            java.lang.String r7 = r7.toString()
            r5.warn(r7)
        L_0x0eee:
            int r2 = r2 + 1
            r13 = r33
            r12 = r34
            r11 = r35
            r14 = r36
            r7 = r37
            r8 = r38
            r5 = r39
            goto L_0x057b
        L_0x0f00:
            r39 = r5
            r37 = r7
            r38 = r8
            r35 = r11
            r34 = r12
            r33 = r13
            r36 = r14
            r15 = r46
            goto L_0x0f23
        L_0x0f11:
            r39 = r5
            r37 = r7
            r38 = r8
            r35 = r11
            r34 = r12
            r33 = r13
            r36 = r14
            r32 = r15
            r15 = r46
        L_0x0f23:
            if (r4 == 0) goto L_0x0fc5
            java.io.PrintWriter r2 = r1.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r7 = "    if ("
            r5.append(r7)
            r5.append(r10)
            java.lang.String r7 = " == NULL) {"
            r5.append(r7)
            java.lang.String r5 = r5.toString()
            r2.println(r5)
            java.io.PrintWriter r2 = r1.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r7 = "        "
            r5.append(r7)
            r5.append(r10)
            java.lang.String r7 = " = JavaCPP_getMethodID(env, "
            r5.append(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r7 = r1.jclasses
            int r7 = r7.index(r3)
            r5.append(r7)
            java.lang.String r7 = ", \""
            r5.append(r7)
            java.lang.reflect.Method r7 = r4.method
            java.lang.String r7 = r7.getName()
            r5.append(r7)
            java.lang.String r7 = "\", \"("
            r5.append(r7)
            java.lang.reflect.Method r7 = r4.method
            java.lang.Class[] r7 = r7.getParameterTypes()
            java.lang.String r7 = signature(r7)
            r5.append(r7)
            java.lang.String r7 = ")"
            r5.append(r7)
            r7 = 1
            java.lang.Class[] r8 = new java.lang.Class[r7]
            java.lang.reflect.Method r7 = r4.method
            java.lang.Class r7 = r7.getReturnType()
            r9 = 0
            r8[r9] = r7
            java.lang.String r7 = signature(r8)
            r5.append(r7)
            java.lang.String r7 = "\");"
            r5.append(r7)
            java.lang.String r5 = r5.toString()
            r2.println(r5)
            java.io.PrintWriter r2 = r1.out
            java.lang.String r5 = "    }"
            r2.println(r5)
            java.io.PrintWriter r2 = r1.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r7 = "    jmethodID mid = "
            r5.append(r7)
            r5.append(r10)
            java.lang.String r7 = ";"
            r5.append(r7)
            java.lang.String r5 = r5.toString()
            r2.println(r5)
            goto L_0x10b4
        L_0x0fc5:
            if (r0 == 0) goto L_0x10b4
            java.io.PrintWriter r2 = r1.out
            java.lang.String r5 = "    if (obj == NULL) {"
            r2.println(r5)
            java.io.PrintWriter r2 = r1.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r7 = "        obj = JavaCPP_createPointer(env, "
            r5.append(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r7 = r1.jclasses
            int r7 = r7.index(r3)
            r5.append(r7)
            java.lang.String r7 = ");"
            r5.append(r7)
            java.lang.String r5 = r5.toString()
            r2.println(r5)
            java.io.PrintWriter r2 = r1.out
            java.lang.String r5 = "        obj = obj == NULL ? NULL : env->NewGlobalRef(obj);"
            r2.println(r5)
            java.io.PrintWriter r2 = r1.out
            java.lang.String r5 = "        if (obj == NULL) {"
            r2.println(r5)
            java.io.PrintWriter r2 = r1.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r7 = "            JavaCPP_log(\"Error creating global reference of "
            r5.append(r7)
            java.lang.String r7 = r45.getCanonicalName()
            r5.append(r7)
            java.lang.String r7 = " instance for callback.\");"
            r5.append(r7)
            java.lang.String r5 = r5.toString()
            r2.println(r5)
            java.io.PrintWriter r2 = r1.out
            java.lang.String r5 = "        } else {"
            r2.println(r5)
            java.io.PrintWriter r2 = r1.out
            java.lang.String r5 = "            env->SetLongField(obj, JavaCPP_addressFID, ptr_to_jlong(this));"
            r2.println(r5)
            java.io.PrintWriter r2 = r1.out
            java.lang.String r5 = "        }"
            r2.println(r5)
            java.io.PrintWriter r2 = r1.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r7 = "        ptr = &"
            r5.append(r7)
            r5.append(r0)
            java.lang.String r7 = ";"
            r5.append(r7)
            java.lang.String r5 = r5.toString()
            r2.println(r5)
            java.io.PrintWriter r2 = r1.out
            java.lang.String r5 = "    }"
            r2.println(r5)
            java.io.PrintWriter r2 = r1.out
            java.lang.String r5 = "    if (mid == NULL) {"
            r2.println(r5)
            java.io.PrintWriter r2 = r1.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r7 = "        mid = JavaCPP_getMethodID(env, "
            r5.append(r7)
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r7 = r1.jclasses
            int r7 = r7.index(r3)
            r5.append(r7)
            java.lang.String r7 = ", \""
            r5.append(r7)
            java.lang.String r7 = r46.getName()
            r5.append(r7)
            java.lang.String r7 = "\", \"("
            r5.append(r7)
            java.lang.Class[] r7 = r46.getParameterTypes()
            java.lang.String r7 = signature(r7)
            r5.append(r7)
            java.lang.String r7 = ")"
            r5.append(r7)
            r7 = 1
            java.lang.Class[] r8 = new java.lang.Class[r7]
            java.lang.Class r7 = r46.getReturnType()
            r9 = 0
            r8[r9] = r7
            java.lang.String r7 = signature(r8)
            r5.append(r7)
            java.lang.String r7 = "\");"
            r5.append(r7)
            java.lang.String r5 = r5.toString()
            r2.println(r5)
            java.io.PrintWriter r2 = r1.out
            java.lang.String r5 = "    }"
            r2.println(r5)
        L_0x10b4:
            java.io.PrintWriter r2 = r1.out
            java.lang.String r5 = "    if (obj == NULL) {"
            r2.println(r5)
            java.io.PrintWriter r2 = r1.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r7 = "        JavaCPP_log(\"Function pointer object is NULL in callback for "
            r5.append(r7)
            java.lang.String r7 = r45.getCanonicalName()
            r5.append(r7)
            java.lang.String r7 = ".\");"
            r5.append(r7)
            java.lang.String r5 = r5.toString()
            r2.println(r5)
            java.io.PrintWriter r2 = r1.out
            java.lang.String r5 = "    } else if (mid == NULL) {"
            r2.println(r5)
            java.io.PrintWriter r2 = r1.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r7 = "        JavaCPP_log(\"Error getting method ID of function caller \\\""
            r5.append(r7)
            r5.append(r15)
            java.lang.String r7 = "\\\" for callback.\");"
            r5.append(r7)
            java.lang.String r5 = r5.toString()
            r2.println(r5)
            java.io.PrintWriter r2 = r1.out
            java.lang.String r5 = "    } else {"
            r2.println(r5)
            java.lang.String r2 = "Object"
            boolean r5 = r39.isPrimitive()
            if (r5 == 0) goto L_0x112c
            java.lang.String r2 = r39.getName()
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            r7 = 0
            char r8 = r2.charAt(r7)
            char r7 = java.lang.Character.toUpperCase(r8)
            r5.append(r7)
            r7 = 1
            java.lang.String r8 = r2.substring(r7)
            r5.append(r8)
            java.lang.String r2 = r5.toString()
        L_0x112c:
            java.io.PrintWriter r5 = r1.out
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r8 = "        "
            r7.append(r8)
            r8 = r38
            r7.append(r8)
            java.lang.String r9 = "env->Call"
            r7.append(r9)
            r7.append(r2)
            java.lang.String r9 = "MethodA(obj, mid, "
            r7.append(r9)
            int r9 = r6.length
            if (r9 != 0) goto L_0x1150
            java.lang.String r9 = "NULL);"
            goto L_0x1152
        L_0x1150:
            java.lang.String r9 = "args);"
        L_0x1152:
            r7.append(r9)
            java.lang.String r7 = r7.toString()
            r5.println(r7)
            if (r37 == 0) goto L_0x1173
            java.io.PrintWriter r5 = r1.out
            java.lang.String r7 = "        if ((exc = env->ExceptionOccurred()) != NULL) {"
            r5.println(r7)
            java.io.PrintWriter r5 = r1.out
            java.lang.String r7 = "            env->ExceptionClear();"
            r5.println(r7)
            java.io.PrintWriter r5 = r1.out
            java.lang.String r7 = "        }"
            r5.println(r7)
        L_0x1173:
            java.io.PrintWriter r5 = r1.out
            java.lang.String r7 = "    }"
            r5.println(r7)
            r5 = 0
        L_0x117b:
            int r7 = r6.length
            if (r5 >= r7) goto L_0x13bd
            java.lang.Class<org.bytedeco.javacpp.Pointer> r7 = org.bytedeco.javacpp.Pointer.class
            r9 = r6[r5]
            boolean r7 = r7.isAssignableFrom(r9)
            if (r7 == 0) goto L_0x138c
            r7 = r6[r5]
            java.lang.String[] r7 = r1.cppTypeName(r7)
            r9 = r24[r5]
            java.lang.annotation.Annotation r9 = r1.by(r9)
            r11 = r6[r5]
            r12 = r24[r5]
            java.lang.String r11 = r1.cast((java.lang.Class<?>) r11, (java.lang.annotation.Annotation[]) r12)
            java.lang.String r12 = valueTypeName(r7)
            r13 = r24[r5]
            r14 = 1
            org.bytedeco.javacpp.tools.AdapterInformation r13 = r1.adapterInformation((boolean) r14, (java.lang.String) r12, (java.lang.annotation.Annotation[]) r13)
            java.lang.String r14 = "void*"
            r18 = 0
            r0 = r7[r18]
            boolean r0 = r14.equals(r0)
            if (r0 == 0) goto L_0x11c1
            r0 = r6[r5]
            java.lang.Class<org.bytedeco.javacpp.annotation.Opaque> r14 = org.bytedeco.javacpp.annotation.Opaque.class
            boolean r0 = r0.isAnnotationPresent(r14)
            if (r0 != 0) goto L_0x11c1
            java.lang.String r0 = "char*"
            r7[r18] = r0
        L_0x11c1:
            if (r13 != 0) goto L_0x11d0
            boolean r0 = r9 instanceof org.bytedeco.javacpp.annotation.ByPtrPtr
            if (r0 != 0) goto L_0x11d0
            boolean r0 = r9 instanceof org.bytedeco.javacpp.annotation.ByPtrRef
            if (r0 == 0) goto L_0x11cc
            goto L_0x11d0
        L_0x11cc:
            r43 = r2
            goto L_0x138e
        L_0x11d0:
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            r43 = r2
            java.lang.String r2 = "    "
            r14.append(r2)
            r2 = 0
            r3 = r7[r2]
            r14.append(r3)
            java.lang.String r3 = " rptr"
            r14.append(r3)
            r14.append(r5)
            r3 = 1
            r2 = r7[r3]
            r14.append(r2)
            java.lang.String r2 = " = ("
            r14.append(r2)
            r2 = 0
            r4 = r7[r2]
            r14.append(r4)
            r2 = r7[r3]
            r14.append(r2)
            java.lang.String r2 = ")jlong_to_ptr(env->GetLongField(obj"
            r14.append(r2)
            r14.append(r5)
            java.lang.String r2 = ", JavaCPP_addressFID));"
            r14.append(r2)
            java.lang.String r2 = r14.toString()
            r0.println(r2)
            if (r13 == 0) goto L_0x125e
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "    jlong rsize"
            r2.append(r3)
            r2.append(r5)
            java.lang.String r3 = " = env->GetLongField(obj"
            r2.append(r3)
            r2.append(r5)
            java.lang.String r3 = ", JavaCPP_limitFID);"
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r0.println(r2)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "    void* rowner"
            r2.append(r3)
            r2.append(r5)
            java.lang.String r3 = " = JavaCPP_getPointerOwner(env, obj"
            r2.append(r3)
            r2.append(r5)
            java.lang.String r3 = ");"
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r0.println(r2)
        L_0x125e:
            r0 = r6[r5]
            java.lang.Class<org.bytedeco.javacpp.annotation.Opaque> r2 = org.bytedeco.javacpp.annotation.Opaque.class
            boolean r0 = r0.isAnnotationPresent(r2)
            if (r0 != 0) goto L_0x12d3
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "    jlong rposition"
            r2.append(r3)
            r2.append(r5)
            java.lang.String r3 = " = env->GetLongField(obj"
            r2.append(r3)
            r2.append(r5)
            java.lang.String r3 = ", JavaCPP_positionFID);"
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r0.println(r2)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "    rptr"
            r2.append(r3)
            r2.append(r5)
            java.lang.String r3 = " += rposition"
            r2.append(r3)
            r2.append(r5)
            java.lang.String r3 = ";"
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r0.println(r2)
            if (r13 == 0) goto L_0x12d3
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "    rsize"
            r2.append(r3)
            r2.append(r5)
            java.lang.String r3 = " -= rposition"
            r2.append(r3)
            r2.append(r5)
            java.lang.String r3 = ";"
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r0.println(r2)
        L_0x12d3:
            if (r13 == 0) goto L_0x130a
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "    adapter"
            r2.append(r3)
            r2.append(r5)
            java.lang.String r3 = ".assign(rptr"
            r2.append(r3)
            r2.append(r5)
            java.lang.String r3 = ", rsize"
            r2.append(r3)
            r2.append(r5)
            java.lang.String r3 = ", rowner"
            r2.append(r3)
            r2.append(r5)
            java.lang.String r3 = ");"
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r0.println(r2)
            goto L_0x138e
        L_0x130a:
            boolean r0 = r9 instanceof org.bytedeco.javacpp.annotation.ByPtrPtr
            if (r0 == 0) goto L_0x135c
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "    if (arg"
            r2.append(r3)
            r2.append(r5)
            java.lang.String r3 = " != NULL) {"
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r0.println(r2)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "        *arg"
            r2.append(r3)
            r2.append(r5)
            java.lang.String r3 = " = *"
            r2.append(r3)
            r2.append(r11)
            java.lang.String r3 = "&rptr"
            r2.append(r3)
            r2.append(r5)
            java.lang.String r3 = ";"
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r0.println(r2)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r2 = "    }"
            r0.println(r2)
            goto L_0x138e
        L_0x135c:
            boolean r0 = r9 instanceof org.bytedeco.javacpp.annotation.ByPtrRef
            if (r0 == 0) goto L_0x138e
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "    arg"
            r2.append(r3)
            r2.append(r5)
            java.lang.String r3 = " = "
            r2.append(r3)
            r2.append(r11)
            java.lang.String r3 = "rptr"
            r2.append(r3)
            r2.append(r5)
            java.lang.String r3 = ";"
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r0.println(r2)
            goto L_0x138e
        L_0x138c:
            r43 = r2
        L_0x138e:
            r0 = r6[r5]
            boolean r0 = r0.isPrimitive()
            if (r0 != 0) goto L_0x13b1
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "    env->DeleteLocalRef(obj"
            r2.append(r3)
            r2.append(r5)
            java.lang.String r3 = ");"
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r0.println(r2)
        L_0x13b1:
            int r5 = r5 + 1
            r2 = r43
            r0 = r47
            r3 = r45
            r4 = r49
            goto L_0x117b
        L_0x13bd:
            r43 = r2
            java.io.PrintWriter r0 = r1.out
            java.lang.String r2 = "}"
            r0.println(r2)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r2 = "end:"
            r0.println(r2)
            java.lang.Class r0 = java.lang.Void.TYPE
            r2 = r39
            if (r2 == r0) goto L_0x15d6
            java.lang.String r0 = "void*"
            r3 = 0
            r4 = r16[r3]
            boolean r0 = r0.equals(r4)
            if (r0 == 0) goto L_0x13ea
            java.lang.Class<org.bytedeco.javacpp.annotation.Opaque> r0 = org.bytedeco.javacpp.annotation.Opaque.class
            boolean r0 = r2.isAnnotationPresent(r0)
            if (r0 != 0) goto L_0x13ea
            java.lang.String r0 = "char*"
            r16[r3] = r0
        L_0x13ea:
            java.lang.Class<java.lang.Enum> r0 = java.lang.Enum.class
            boolean r0 = r0.isAssignableFrom(r2)
            if (r0 == 0) goto L_0x1491
            r0 = 1
            r1.accessesEnums = r0
            java.lang.String r3 = r1.enumValueType(r2)
            if (r3 == 0) goto L_0x148d
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            r5 = 0
            char r7 = r3.charAt(r5)
            char r5 = java.lang.Character.toUpperCase(r7)
            r4.append(r5)
            java.lang.String r5 = r3.substring(r0)
            r4.append(r5)
            java.lang.String r0 = r4.toString()
            java.io.PrintWriter r4 = r1.out
            java.lang.String r5 = "    if (rarg == NULL) {"
            r4.println(r5)
            java.io.PrintWriter r4 = r1.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r7 = "        JavaCPP_log(\"Enum for return is NULL in callback for "
            r5.append(r7)
            java.lang.String r7 = r45.getCanonicalName()
            r5.append(r7)
            java.lang.String r7 = ".\");"
            r5.append(r7)
            java.lang.String r5 = r5.toString()
            r4.println(r5)
            java.io.PrintWriter r4 = r1.out
            java.lang.String r5 = "    }"
            r4.println(r5)
            java.io.PrintWriter r4 = r1.out
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r7 = "    "
            r5.append(r7)
            r7 = 0
            r9 = r16[r7]
            r5.append(r9)
            java.lang.String r9 = " rval"
            r5.append(r9)
            r9 = 1
            r11 = r16[r9]
            r5.append(r11)
            java.lang.String r11 = " = ("
            r5.append(r11)
            r11 = r16[r7]
            r5.append(r11)
            r7 = r16[r9]
            r5.append(r7)
            java.lang.String r7 = ")(rarg == NULL ? 0 : env->Get"
            r5.append(r7)
            r5.append(r0)
            java.lang.String r7 = "Field(rarg, JavaCPP_"
            r5.append(r7)
            r5.append(r3)
            java.lang.String r7 = "ValueFID));"
            r5.append(r7)
            java.lang.String r5 = r5.toString()
            r4.println(r5)
        L_0x148d:
            r43 = r3
            goto L_0x15d6
        L_0x1491:
            java.lang.Class<org.bytedeco.javacpp.Pointer> r0 = org.bytedeco.javacpp.Pointer.class
            boolean r0 = r0.isAssignableFrom(r2)
            if (r0 == 0) goto L_0x1502
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = "    "
            r3.append(r4)
            r4 = 0
            r5 = r16[r4]
            r3.append(r5)
            java.lang.String r5 = " rptr"
            r3.append(r5)
            r5 = 1
            r7 = r16[r5]
            r3.append(r7)
            java.lang.String r7 = " = rarg == NULL ? NULL : ("
            r3.append(r7)
            r7 = r16[r4]
            r3.append(r7)
            r4 = r16[r5]
            r3.append(r4)
            java.lang.String r4 = ")jlong_to_ptr(env->GetLongField(rarg, JavaCPP_addressFID));"
            r3.append(r4)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            if (r36 == 0) goto L_0x14e1
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    jlong rsize = rarg == NULL ? 0 : env->GetLongField(rarg, JavaCPP_limitFID);"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    void* rowner = JavaCPP_getPointerOwner(env, rarg);"
            r0.println(r3)
        L_0x14e1:
            java.lang.Class<org.bytedeco.javacpp.annotation.Opaque> r0 = org.bytedeco.javacpp.annotation.Opaque.class
            boolean r0 = r2.isAnnotationPresent(r0)
            if (r0 != 0) goto L_0x15d6
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    jlong rposition = rarg == NULL ? 0 : env->GetLongField(rarg, JavaCPP_positionFID);"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    rptr += rposition;"
            r0.println(r3)
            if (r36 == 0) goto L_0x15d6
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    rsize -= rposition;"
            r0.println(r3)
            goto L_0x15d6
        L_0x1502:
            java.lang.Class<java.lang.String> r0 = java.lang.String.class
            if (r2 != r0) goto L_0x1543
            r0 = 1
            r1.passesStrings = r0
            java.io.PrintWriter r3 = r1.out
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r5 = "    "
            r4.append(r5)
            r5 = 0
            r7 = r16[r5]
            r4.append(r7)
            java.lang.String r5 = " rptr"
            r4.append(r5)
            r5 = r16[r0]
            r4.append(r5)
            java.lang.String r0 = " = JavaCPP_getStringBytes(env, rarg);"
            r4.append(r0)
            java.lang.String r0 = r4.toString()
            r3.println(r0)
            if (r36 == 0) goto L_0x15d6
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    jlong rsize = 0;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    void* rowner = (void*)rptr;"
            r0.println(r3)
            goto L_0x15d6
        L_0x1543:
            java.lang.Class<java.nio.Buffer> r0 = java.nio.Buffer.class
            boolean r0 = r0.isAssignableFrom(r2)
            if (r0 == 0) goto L_0x15a9
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = "    "
            r3.append(r4)
            r4 = 0
            r5 = r16[r4]
            r3.append(r5)
            java.lang.String r5 = " rptr"
            r3.append(r5)
            r5 = 1
            r7 = r16[r5]
            r3.append(r7)
            java.lang.String r7 = " = rarg == NULL ? NULL : ("
            r3.append(r7)
            r7 = r16[r4]
            r3.append(r7)
            r4 = r16[r5]
            r3.append(r4)
            java.lang.String r4 = ")env->GetDirectBufferAddress(rarg);"
            r3.append(r4)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            if (r36 == 0) goto L_0x15d6
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    jlong rsize = rarg == NULL ? 0 : env->GetIntField(rarg, JavaCPP_bufferLimitFID);"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    void* rowner = (void*)rptr;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    jlong rposition = rarg == NULL ? 0 : env->GetIntField(rarg, JavaCPP_bufferPositionFID);"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    rptr += rposition;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    rsize -= rposition;"
            r0.println(r3)
            goto L_0x15d6
        L_0x15a9:
            boolean r0 = r2.isPrimitive()
            if (r0 != 0) goto L_0x15d6
            org.bytedeco.javacpp.tools.Logger r0 = r1.logger
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = "Callback \""
            r3.append(r4)
            r3.append(r15)
            java.lang.String r4 = "\" has unsupported return type \""
            r3.append(r4)
            java.lang.String r4 = r2.getCanonicalName()
            r3.append(r4)
            java.lang.String r4 = "\". Compilation will most likely fail."
            r3.append(r4)
            java.lang.String r3 = r3.toString()
            r0.warn(r3)
        L_0x15d6:
            r0 = 1
            r1.passesStrings = r0
            if (r37 == 0) goto L_0x1630
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (exc != NULL) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        jstring str = (jstring)env->CallObjectMethod(exc, JavaCPP_toStringMID);"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        env->DeleteLocalRef(exc);"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        const char *msg = JavaCPP_getStringBytes(env, str);"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        JavaCPP_exception e(msg);"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        JavaCPP_releaseStringBytes(env, str, msg);"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        env->DeleteLocalRef(str);"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        JavaCPP_detach(attached);"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        throw e;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    } else {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        JavaCPP_detach(attached);"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            goto L_0x1637
        L_0x1630:
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    JavaCPP_detach(attached);"
            r0.println(r3)
        L_0x1637:
            java.lang.Class r0 = java.lang.Void.TYPE
            if (r2 == r0) goto L_0x17b9
            boolean r0 = r2.isPrimitive()
            if (r0 == 0) goto L_0x166d
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = "    return "
            r3.append(r4)
            r4 = r35
            r3.append(r4)
            r5 = r34
            boolean r7 = r5 instanceof org.bytedeco.javacpp.annotation.ByPtr
            if (r7 != 0) goto L_0x1660
            boolean r7 = r5 instanceof org.bytedeco.javacpp.annotation.ByPtrRef
            if (r7 == 0) goto L_0x165d
            goto L_0x1660
        L_0x165d:
            java.lang.String r7 = "rarg;"
            goto L_0x1662
        L_0x1660:
            java.lang.String r7 = "&rarg;"
        L_0x1662:
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            goto L_0x1694
        L_0x166d:
            r5 = r34
            r4 = r35
            java.lang.Class<java.lang.Enum> r0 = java.lang.Enum.class
            boolean r0 = r0.isAssignableFrom(r2)
            if (r0 == 0) goto L_0x1698
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    return "
            r3.append(r7)
            r3.append(r4)
            java.lang.String r7 = "rval;"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
        L_0x1694:
            r7 = r36
            goto L_0x17bf
        L_0x1698:
            if (r36 == 0) goto L_0x16c6
            r0 = 1
            r1.usesAdapters = r0
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r7 = "    return "
            r3.append(r7)
            r7 = r36
            java.lang.String r9 = r7.name
            r3.append(r9)
            java.lang.String r9 = "("
            r3.append(r9)
            r3.append(r4)
            java.lang.String r9 = "rptr, rsize, rowner);"
            r3.append(r9)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            goto L_0x17bf
        L_0x16c6:
            r7 = r36
            java.lang.Class<org.bytedeco.javacpp.FunctionPointer> r0 = org.bytedeco.javacpp.FunctionPointer.class
            boolean r0 = r0.isAssignableFrom(r2)
            if (r0 == 0) goto L_0x16f2
            org.bytedeco.javacpp.tools.IndexedSet<java.lang.Class> r0 = r1.functions
            r0.index(r2)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r9 = "    return "
            r3.append(r9)
            r3.append(r4)
            java.lang.String r9 = "(rptr == NULL ? NULL : rptr->ptr);"
            r3.append(r9)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            goto L_0x17bf
        L_0x16f2:
            boolean r0 = r5 instanceof org.bytedeco.javacpp.annotation.ByVal
            if (r0 != 0) goto L_0x1739
            boolean r0 = r5 instanceof org.bytedeco.javacpp.annotation.ByRef
            if (r0 == 0) goto L_0x16fb
            goto L_0x1739
        L_0x16fb:
            boolean r0 = r5 instanceof org.bytedeco.javacpp.annotation.ByPtrPtr
            if (r0 == 0) goto L_0x171c
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r9 = "    return "
            r3.append(r9)
            r3.append(r4)
            java.lang.String r9 = "&rptr;"
            r3.append(r9)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            goto L_0x17bf
        L_0x171c:
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r9 = "    return "
            r3.append(r9)
            r3.append(r4)
            java.lang.String r9 = "rptr;"
            r3.append(r9)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            goto L_0x17bf
        L_0x1739:
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    if (rptr == NULL) {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r9 = "        JavaCPP_log(\"Return pointer address is NULL in callback for "
            r3.append(r9)
            java.lang.String r9 = r45.getCanonicalName()
            r3.append(r9)
            java.lang.String r9 = ".\");"
            r3.append(r9)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r9 = "        static "
            r3.append(r9)
            r9 = 0
            r9 = r33[r9]
            r3.append(r9)
            java.lang.String r9 = " empty"
            r3.append(r9)
            r9 = 1
            r9 = r16[r9]
            r3.append(r9)
            java.lang.String r9 = ";"
            r3.append(r9)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "        return empty;"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    } else {"
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r9 = "        return *"
            r3.append(r9)
            r3.append(r4)
            java.lang.String r9 = "rptr;"
            r3.append(r9)
            java.lang.String r3 = r3.toString()
            r0.println(r3)
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "    }"
            r0.println(r3)
            goto L_0x17bf
        L_0x17b9:
            r5 = r34
            r4 = r35
            r7 = r36
        L_0x17bf:
            java.io.PrintWriter r0 = r1.out
            java.lang.String r3 = "}"
            r0.println(r3)
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.Generator.callback(java.lang.Class, java.lang.reflect.Method, java.lang.String, boolean, org.bytedeco.javacpp.tools.MethodInformation):void");
    }

    /* access modifiers changed from: package-private */
    public void callbackAllocator(Class cls, String callbackName) {
        StringBuilder sb;
        String[] typeName = cppTypeName(cls);
        String instanceTypeName = functionClassName(cls);
        this.out.println("    obj = env->NewWeakGlobalRef(obj);");
        this.out.println("    if (obj == NULL) {");
        PrintWriter printWriter = this.out;
        printWriter.println("        JavaCPP_log(\"Error creating global reference of " + cls.getCanonicalName() + " instance for callback.\");");
        this.out.println("        return;");
        this.out.println("    }");
        PrintWriter printWriter2 = this.out;
        printWriter2.println("    " + instanceTypeName + "* rptr = new (std::nothrow) " + instanceTypeName + ";");
        this.out.println("    if (rptr != NULL) {");
        PrintWriter printWriter3 = this.out;
        StringBuilder sb2 = new StringBuilder();
        sb2.append("        rptr->ptr = ");
        if (callbackName == null) {
            sb = new StringBuilder();
            sb.append("(");
            sb.append(typeName[0]);
            sb.append(typeName[1]);
            sb.append(")jlong_to_ptr(arg0)");
        } else {
            sb = new StringBuilder();
            sb.append("&");
            sb.append(callbackName);
        }
        sb2.append(sb.toString());
        sb2.append(";");
        printWriter3.println(sb2.toString());
        this.out.println("        rptr->obj = obj;");
        PrintWriter printWriter4 = this.out;
        printWriter4.println("        JavaCPP_initPointer(env, obj, rptr, 1, rptr, &JavaCPP_" + mangle(cls.getName()) + "_deallocate);");
        this.deallocators.index(cls);
        if (callbackName != null) {
            PrintWriter printWriter5 = this.out;
            printWriter5.println("        " + callbackName + "_instance = *rptr;");
        }
        this.out.println("    }");
        this.out.println("}");
    }

    static String functionClassName(Class<?> cls) {
        Name name = (Name) cls.getAnnotation(Name.class);
        if (name != null) {
            return name.value()[0];
        }
        return "JavaCPP_" + mangle(cls.getName());
    }

    static Method[] functionMethods(Class<?> cls, boolean[] callbackAllocators) {
        if (!FunctionPointer.class.isAssignableFrom(cls)) {
            return null;
        }
        Method[] methods = cls.getDeclaredMethods();
        Method[] functionMethods = new Method[3];
        for (int i = 0; i < methods.length; i++) {
            String methodName = methods[i].getName();
            int modifiers = methods[i].getModifiers();
            Class[] parameterTypes = methods[i].getParameterTypes();
            Class returnType = methods[i].getReturnType();
            if (!Modifier.isStatic(modifiers)) {
                if (callbackAllocators != null && methodName.startsWith("allocate") && Modifier.isNative(modifiers) && returnType == Void.TYPE && (parameterTypes.length == 0 || (parameterTypes.length == 1 && (parameterTypes[0] == Integer.TYPE || parameterTypes[0] == Long.TYPE)))) {
                    callbackAllocators[i] = true;
                } else if (methodName.startsWith(NotificationCompat.CATEGORY_CALL) || methodName.startsWith("apply")) {
                    functionMethods[0] = methods[i];
                } else if (methodName.startsWith("get") && Modifier.isNative(modifiers) && cls.isAnnotationPresent(Namespace.class)) {
                    functionMethods[1] = methods[i];
                } else if (methodName.startsWith("put") && Modifier.isNative(modifiers) && cls.isAnnotationPresent(Namespace.class)) {
                    functionMethods[2] = methods[i];
                }
            }
        }
        if (functionMethods[0] == null && functionMethods[1] == null && functionMethods[2] == null) {
            return null;
        }
        return functionMethods;
    }

    /* access modifiers changed from: package-private */
    /* JADX WARNING: Removed duplicated region for block: B:103:0x0231  */
    /* JADX WARNING: Removed duplicated region for block: B:107:0x024a A[EDGE_INSN: B:365:0x024a->B:107:0x024a ?: BREAK  
    EDGE_INSN: B:366:0x024a->B:107:0x024a ?: BREAK  ] */
    /* JADX WARNING: Removed duplicated region for block: B:109:0x0250  */
    /* JADX WARNING: Removed duplicated region for block: B:179:0x0374  */
    /* JADX WARNING: Removed duplicated region for block: B:194:0x03b0  */
    /* JADX WARNING: Removed duplicated region for block: B:279:0x0514  */
    /* JADX WARNING: Removed duplicated region for block: B:282:0x0527  */
    /* JADX WARNING: Removed duplicated region for block: B:289:0x0544  */
    /* JADX WARNING: Removed duplicated region for block: B:290:0x0546  */
    /* JADX WARNING: Removed duplicated region for block: B:299:0x0566  */
    /* JADX WARNING: Removed duplicated region for block: B:300:0x0568  */
    /* JADX WARNING: Removed duplicated region for block: B:310:0x0584  */
    /* JADX WARNING: Removed duplicated region for block: B:315:0x0594  */
    /* JADX WARNING: Removed duplicated region for block: B:326:0x05bd  */
    /* JADX WARNING: Removed duplicated region for block: B:327:0x05c6  */
    /* JADX WARNING: Removed duplicated region for block: B:354:0x061a  */
    /* JADX WARNING: Removed duplicated region for block: B:355:0x061e  */
    /* JADX WARNING: Removed duplicated region for block: B:71:0x01a5  */
    /* JADX WARNING: Removed duplicated region for block: B:74:0x01b3  */
    /* JADX WARNING: Removed duplicated region for block: B:78:0x01bc  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public org.bytedeco.javacpp.tools.MethodInformation methodInformation(java.lang.reflect.Method r37) {
        /*
            r36 = this;
            r0 = r36
            r1 = r37
            org.bytedeco.javacpp.tools.MethodInformation r2 = new org.bytedeco.javacpp.tools.MethodInformation
            r2.<init>()
            java.lang.Class r3 = r37.getDeclaringClass()
            r2.cls = r3
            r2.method = r1
            java.lang.annotation.Annotation[] r3 = r37.getAnnotations()
            r2.annotations = r3
            int r3 = r37.getModifiers()
            r2.modifiers = r3
            java.lang.Class r3 = r37.getReturnType()
            r2.returnType = r3
            java.lang.String r3 = r37.getName()
            r2.name = r3
            java.lang.Class<org.bytedeco.javacpp.annotation.Name> r3 = org.bytedeco.javacpp.annotation.Name.class
            java.lang.annotation.Annotation r3 = r1.getAnnotation(r3)
            org.bytedeco.javacpp.annotation.Name r3 = (org.bytedeco.javacpp.annotation.Name) r3
            r4 = 0
            r5 = 1
            if (r3 == 0) goto L_0x003a
            java.lang.String[] r6 = r3.value()
            goto L_0x0040
        L_0x003a:
            java.lang.String[] r6 = new java.lang.String[r5]
            java.lang.String r7 = r2.name
            r6[r4] = r7
        L_0x0040:
            r2.memberName = r6
            java.lang.Class<org.bytedeco.javacpp.annotation.Index> r6 = org.bytedeco.javacpp.annotation.Index.class
            java.lang.annotation.Annotation r6 = r1.getAnnotation(r6)
            org.bytedeco.javacpp.annotation.Index r6 = (org.bytedeco.javacpp.annotation.Index) r6
            if (r6 == 0) goto L_0x0051
            int r7 = r6.value()
            goto L_0x0052
        L_0x0051:
            r7 = 0
        L_0x0052:
            r2.dim = r7
            java.lang.Class[] r7 = r37.getParameterTypes()
            r2.parameterTypes = r7
            java.lang.annotation.Annotation[][] r7 = r37.getParameterAnnotations()
            r2.parameterAnnotations = r7
            java.lang.Class<?> r7 = r2.cls
            java.lang.reflect.Method r8 = r2.method
            boolean r7 = criticalRegion(r7, r8)
            r2.criticalRegion = r7
            java.lang.Class<org.bytedeco.javacpp.annotation.Raw> r7 = org.bytedeco.javacpp.annotation.Raw.class
            boolean r7 = r1.isAnnotationPresent(r7)
            r2.returnRaw = r7
            boolean r7 = r2.returnRaw
            if (r7 == 0) goto L_0x0083
            java.lang.Class<org.bytedeco.javacpp.annotation.Raw> r7 = org.bytedeco.javacpp.annotation.Raw.class
            java.lang.annotation.Annotation r7 = r1.getAnnotation(r7)
            org.bytedeco.javacpp.annotation.Raw r7 = (org.bytedeco.javacpp.annotation.Raw) r7
            boolean r7 = r7.withEnv()
            goto L_0x0084
        L_0x0083:
            r7 = 0
        L_0x0084:
            r2.withEnv = r7
            java.lang.annotation.Annotation[][] r7 = r2.parameterAnnotations
            int r7 = r7.length
            boolean[] r7 = new boolean[r7]
            r2.parameterRaw = r7
            r7 = 0
        L_0x008e:
            java.lang.annotation.Annotation[][] r8 = r2.parameterAnnotations
            int r8 = r8.length
            if (r7 >= r8) goto L_0x00c0
            r8 = 0
        L_0x0094:
            java.lang.annotation.Annotation[][] r9 = r2.parameterAnnotations
            r9 = r9[r7]
            int r9 = r9.length
            if (r8 >= r9) goto L_0x00bd
            java.lang.annotation.Annotation[][] r9 = r2.parameterAnnotations
            r9 = r9[r7]
            r9 = r9[r8]
            boolean r9 = r9 instanceof org.bytedeco.javacpp.annotation.Raw
            if (r9 == 0) goto L_0x00ba
            boolean[] r9 = r2.parameterRaw
            r9[r7] = r5
            boolean r9 = r2.withEnv
            java.lang.annotation.Annotation[][] r10 = r2.parameterAnnotations
            r10 = r10[r7]
            r10 = r10[r8]
            org.bytedeco.javacpp.annotation.Raw r10 = (org.bytedeco.javacpp.annotation.Raw) r10
            boolean r10 = r10.withEnv()
            r9 = r9 | r10
            r2.withEnv = r9
        L_0x00ba:
            int r8 = r8 + 1
            goto L_0x0094
        L_0x00bd:
            int r7 = r7 + 1
            goto L_0x008e
        L_0x00c0:
            java.lang.Class<?> r7 = r2.returnType
            java.lang.Class r8 = java.lang.Void.TYPE
            if (r7 != r8) goto L_0x00e6
            java.lang.Class<?>[] r7 = r2.parameterTypes
            int r7 = r7.length
            if (r7 <= 0) goto L_0x00e4
            java.lang.Class<?>[] r7 = r2.parameterTypes
            r7 = r7[r4]
            boolean r7 = r7.isArray()
            if (r7 == 0) goto L_0x00e4
            java.lang.Class<?>[] r7 = r2.parameterTypes
            r7 = r7[r4]
            java.lang.Class r7 = r7.getComponentType()
            boolean r7 = r7.isPrimitive()
            if (r7 == 0) goto L_0x00e4
            goto L_0x00e6
        L_0x00e4:
            r7 = 0
            goto L_0x00e7
        L_0x00e6:
            r7 = 1
        L_0x00e7:
            java.lang.Class<?> r8 = r2.returnType
            java.lang.Class r9 = java.lang.Void.TYPE
            if (r8 == r9) goto L_0x00f3
            java.lang.Class<?> r8 = r2.returnType
            java.lang.Class<?> r9 = r2.cls
            if (r8 != r9) goto L_0x00fa
        L_0x00f3:
            java.lang.Class<?>[] r8 = r2.parameterTypes
            int r8 = r8.length
            if (r8 <= 0) goto L_0x00fa
            r8 = 1
            goto L_0x00fb
        L_0x00fa:
            r8 = 0
        L_0x00fb:
            int r9 = r2.modifiers
            boolean r9 = java.lang.reflect.Modifier.isStatic(r9)
            if (r9 != 0) goto L_0x010b
            java.lang.Class<?> r9 = r2.returnType
            java.lang.Class r10 = java.lang.Void.TYPE
            if (r9 != r10) goto L_0x010b
            r9 = 1
            goto L_0x010c
        L_0x010b:
            r9 = 0
        L_0x010c:
            if (r9 == 0) goto L_0x0125
            java.lang.Class<?>[] r10 = r2.parameterTypes
            int r10 = r10.length
            if (r10 != r5) goto L_0x0125
            java.lang.Class<?>[] r10 = r2.parameterTypes
            r10 = r10[r4]
            java.lang.Class r11 = java.lang.Integer.TYPE
            if (r10 == r11) goto L_0x0123
            java.lang.Class<?>[] r10 = r2.parameterTypes
            r10 = r10[r4]
            java.lang.Class r11 = java.lang.Long.TYPE
            if (r10 != r11) goto L_0x0125
        L_0x0123:
            r10 = 1
            goto L_0x0126
        L_0x0125:
            r10 = 0
        L_0x0126:
            r11 = 0
            r12 = 0
            r13 = 0
            r14 = 0
            r15 = 0
            r16 = 0
            java.lang.Class<?> r5 = r2.cls
            java.lang.reflect.Method[] r5 = r5.getDeclaredMethods()
            int r4 = r5.length
            r19 = r14
            r14 = r12
            r12 = r16
            r16 = r13
            r13 = r11
            r11 = 0
        L_0x013d:
            if (r11 >= r4) goto L_0x03ca
            r20 = r4
            r4 = r5[r11]
            r21 = r5
            java.util.Map<java.lang.reflect.Method, org.bytedeco.javacpp.tools.MethodInformation> r5 = r0.annotationCache
            java.lang.Object r5 = r5.get(r4)
            org.bytedeco.javacpp.tools.MethodInformation r5 = (org.bytedeco.javacpp.tools.MethodInformation) r5
            if (r5 != 0) goto L_0x0186
            r22 = r5
            java.util.Map<java.lang.reflect.Method, org.bytedeco.javacpp.tools.MethodInformation> r5 = r0.annotationCache
            r23 = r3
            org.bytedeco.javacpp.tools.MethodInformation r3 = new org.bytedeco.javacpp.tools.MethodInformation
            r3.<init>()
            r24 = r3
            r5.put(r4, r3)
            int r3 = r4.getModifiers()
            r5 = r24
            r5.modifiers = r3
            java.lang.Class r3 = r4.getReturnType()
            r5.returnType = r3
            java.lang.String r3 = r4.getName()
            r5.name = r3
            java.lang.Class[] r3 = r4.getParameterTypes()
            r5.parameterTypes = r3
            java.lang.annotation.Annotation[] r3 = r4.getAnnotations()
            r5.annotations = r3
            java.lang.annotation.Annotation[][] r3 = r4.getParameterAnnotations()
            r5.parameterAnnotations = r3
            goto L_0x018a
        L_0x0186:
            r23 = r3
            r22 = r5
        L_0x018a:
            java.lang.Class<?>[] r3 = r2.parameterTypes
            int r3 = r3.length
            if (r3 <= 0) goto L_0x019d
            java.lang.Class<?>[] r3 = r2.parameterTypes
            r18 = 0
            r3 = r3[r18]
            r25 = r12
            java.lang.Class<java.lang.Class> r12 = java.lang.Class.class
            if (r3 != r12) goto L_0x019f
            r3 = 1
            goto L_0x01a0
        L_0x019d:
            r25 = r12
        L_0x019f:
            r3 = 0
        L_0x01a0:
            java.lang.Class<?>[] r12 = r5.parameterTypes
            int r12 = r12.length
            if (r12 <= 0) goto L_0x01b3
            java.lang.Class<?>[] r12 = r5.parameterTypes
            r18 = 0
            r12 = r12[r18]
            r26 = r14
            java.lang.Class<java.lang.Class> r14 = java.lang.Class.class
            if (r12 != r14) goto L_0x01b5
            r12 = 1
            goto L_0x01b6
        L_0x01b3:
            r26 = r14
        L_0x01b5:
            r12 = 0
        L_0x01b6:
            boolean r14 = r1.equals(r4)
            if (r14 != 0) goto L_0x03b0
            int r14 = r5.modifiers
            boolean r14 = java.lang.reflect.Modifier.isNative(r14)
            if (r14 != 0) goto L_0x01cd
            r29 = r6
            r32 = r10
            r30 = r13
            goto L_0x03b6
        L_0x01cd:
            r14 = 0
            r22 = 0
            r24 = 0
            r27 = 0
            if (r7 == 0) goto L_0x01f2
            r28 = r14
            java.lang.String r14 = "get"
            r29 = r6
            java.lang.String r6 = r2.name
            boolean r6 = r14.equals(r6)
            if (r6 == 0) goto L_0x01f6
            java.lang.String r6 = "put"
            java.lang.String r14 = r5.name
            boolean r6 = r6.equals(r14)
            if (r6 == 0) goto L_0x01f6
            r14 = 1
            r28 = r14
            goto L_0x0220
        L_0x01f2:
            r29 = r6
            r28 = r14
        L_0x01f6:
            if (r8 == 0) goto L_0x020f
            java.lang.String r6 = "put"
            java.lang.String r14 = r2.name
            boolean r6 = r6.equals(r14)
            if (r6 == 0) goto L_0x020f
            java.lang.String r6 = "get"
            java.lang.String r14 = r5.name
            boolean r6 = r6.equals(r14)
            if (r6 == 0) goto L_0x020f
            r22 = 1
            goto L_0x0220
        L_0x020f:
            java.lang.String r6 = r5.name
            java.lang.String r14 = r2.name
            boolean r6 = r6.equals(r14)
            if (r6 == 0) goto L_0x03ab
            r6 = 1
            r2.overloaded = r6
            r24 = r7
            r27 = r8
        L_0x0220:
            r6 = 1
            r14 = r6
            r6 = 0
        L_0x0223:
            r30 = r13
            java.lang.Class<?>[] r13 = r2.parameterTypes
            int r13 = r13.length
            int r13 = r13 - r3
            if (r6 >= r13) goto L_0x024a
            java.lang.Class<?>[] r13 = r5.parameterTypes
            int r13 = r13.length
            int r13 = r13 - r12
            if (r6 >= r13) goto L_0x024a
            java.lang.Class<?>[] r13 = r2.parameterTypes
            int r31 = r6 + r3
            r13 = r13[r31]
            r32 = r10
            java.lang.Class<?>[] r10 = r5.parameterTypes
            int r31 = r6 + r12
            r10 = r10[r31]
            if (r13 == r10) goto L_0x0243
            r10 = 0
            r14 = r10
        L_0x0243:
            int r6 = r6 + 1
            r13 = r30
            r10 = r32
            goto L_0x0223
        L_0x024a:
            r32 = r10
            if (r14 != 0) goto L_0x0250
            goto L_0x03b6
        L_0x0250:
            if (r28 == 0) goto L_0x0272
            java.lang.Class<?>[] r6 = r2.parameterTypes
            int r6 = r6.length
            if (r6 <= 0) goto L_0x0272
            java.lang.Class<?>[] r6 = r2.parameterTypes
            r10 = 0
            r6 = r6[r10]
            boolean r6 = r6.isArray()
            if (r6 == 0) goto L_0x0272
            java.lang.Class<?>[] r6 = r2.parameterTypes
            r6 = r6[r10]
            java.lang.Class r6 = r6.getComponentType()
            boolean r6 = r6.isPrimitive()
            if (r6 == 0) goto L_0x0272
            r6 = 1
            goto L_0x0273
        L_0x0272:
            r6 = 0
        L_0x0273:
            if (r22 == 0) goto L_0x0295
            java.lang.Class<?>[] r10 = r5.parameterTypes
            int r10 = r10.length
            if (r10 <= 0) goto L_0x0295
            java.lang.Class<?>[] r10 = r5.parameterTypes
            r13 = 0
            r10 = r10[r13]
            boolean r10 = r10.isArray()
            if (r10 == 0) goto L_0x0295
            java.lang.Class<?>[] r10 = r5.parameterTypes
            r10 = r10[r13]
            java.lang.Class r10 = r10.getComponentType()
            boolean r10 = r10.isPrimitive()
            if (r10 == 0) goto L_0x0295
            r10 = 1
            goto L_0x0296
        L_0x0295:
            r10 = 0
        L_0x0296:
            if (r7 == 0) goto L_0x0302
            java.lang.Class<?>[] r13 = r5.parameterTypes
            int r13 = r13.length
            if (r6 == 0) goto L_0x02a0
            r31 = 0
            goto L_0x02a2
        L_0x02a0:
            r31 = 1
        L_0x02a2:
            int r13 = r13 - r31
            r33 = r14
            java.lang.Class<?>[] r14 = r2.parameterTypes
            int r14 = r14.length
            int r14 = r14 - r3
            if (r13 != r14) goto L_0x02ff
            if (r6 == 0) goto L_0x02ba
            java.lang.Class<?>[] r13 = r2.parameterTypes
            java.lang.Class<?>[] r14 = r2.parameterTypes
            int r14 = r14.length
            r17 = 1
            int r14 = r14 + -1
            r13 = r13[r14]
            goto L_0x02be
        L_0x02ba:
            r17 = 1
            java.lang.Class<?> r13 = r2.returnType
        L_0x02be:
            java.lang.Class<?>[] r14 = r5.parameterTypes
            r34 = r3
            java.lang.Class<?>[] r3 = r5.parameterTypes
            int r3 = r3.length
            int r3 = r3 + -1
            r3 = r14[r3]
            if (r13 != r3) goto L_0x0306
            java.lang.Class<?> r3 = r5.returnType
            java.lang.Class r13 = java.lang.Void.TYPE
            if (r3 == r13) goto L_0x02d7
            java.lang.Class<?> r3 = r5.returnType
            java.lang.Class<?> r13 = r2.cls
            if (r3 != r13) goto L_0x0306
        L_0x02d7:
            java.lang.annotation.Annotation[][] r3 = r5.parameterAnnotations
            java.lang.annotation.Annotation[][] r13 = r5.parameterAnnotations
            int r13 = r13.length
            r14 = 1
            int r13 = r13 - r14
            r3 = r3[r13]
            int r3 = r3.length
            if (r3 == 0) goto L_0x02f3
            java.lang.annotation.Annotation[][] r3 = r5.parameterAnnotations
            java.lang.annotation.Annotation[][] r13 = r5.parameterAnnotations
            int r13 = r13.length
            int r13 = r13 - r14
            r3 = r3[r13]
            java.lang.annotation.Annotation[] r13 = r2.annotations
            boolean r3 = java.util.Arrays.equals(r3, r13)
            if (r3 == 0) goto L_0x0306
        L_0x02f3:
            r3 = r4
            r13 = r28
            r16 = r24
            r15 = r6
            r35 = r6
            r30 = r13
            goto L_0x0366
        L_0x02ff:
            r34 = r3
            goto L_0x0306
        L_0x0302:
            r34 = r3
            r33 = r14
        L_0x0306:
            if (r8 == 0) goto L_0x0362
            java.lang.Class<?>[] r3 = r2.parameterTypes
            int r3 = r3.length
            if (r10 == 0) goto L_0x030f
            r13 = 0
            goto L_0x0310
        L_0x030f:
            r13 = 1
        L_0x0310:
            int r3 = r3 - r13
            java.lang.Class<?>[] r13 = r5.parameterTypes
            int r13 = r13.length
            int r13 = r13 - r12
            if (r3 != r13) goto L_0x0362
            if (r10 == 0) goto L_0x0323
            java.lang.Class<?>[] r3 = r5.parameterTypes
            java.lang.Class<?>[] r13 = r5.parameterTypes
            int r13 = r13.length
            r14 = 1
            int r13 = r13 - r14
            r3 = r3[r13]
            goto L_0x0326
        L_0x0323:
            r14 = 1
            java.lang.Class<?> r3 = r5.returnType
        L_0x0326:
            java.lang.Class<?>[] r13 = r2.parameterTypes
            r35 = r6
            java.lang.Class<?>[] r6 = r2.parameterTypes
            int r6 = r6.length
            int r6 = r6 - r14
            r6 = r13[r6]
            if (r3 != r6) goto L_0x0364
            java.lang.Class<?> r3 = r2.returnType
            java.lang.Class r6 = java.lang.Void.TYPE
            if (r3 == r6) goto L_0x033e
            java.lang.Class<?> r3 = r2.returnType
            java.lang.Class<?> r6 = r2.cls
            if (r3 != r6) goto L_0x0364
        L_0x033e:
            java.lang.annotation.Annotation[][] r3 = r2.parameterAnnotations
            java.lang.annotation.Annotation[][] r6 = r2.parameterAnnotations
            int r6 = r6.length
            r13 = 1
            int r6 = r6 - r13
            r3 = r3[r6]
            int r3 = r3.length
            if (r3 == 0) goto L_0x035a
            java.lang.annotation.Annotation[][] r3 = r2.parameterAnnotations
            java.lang.annotation.Annotation[][] r6 = r2.parameterAnnotations
            int r6 = r6.length
            int r6 = r6 - r13
            r3 = r3[r6]
            java.lang.annotation.Annotation[] r6 = r5.annotations
            boolean r3 = java.util.Arrays.equals(r3, r6)
            if (r3 == 0) goto L_0x0364
        L_0x035a:
            r3 = r4
            r14 = r22
            r19 = r27
            r26 = r14
            goto L_0x0366
        L_0x0362:
            r35 = r6
        L_0x0364:
            r3 = r25
        L_0x0366:
            if (r16 != 0) goto L_0x036d
            if (r19 == 0) goto L_0x036b
            goto L_0x036d
        L_0x036b:
            r12 = r3
            goto L_0x03b8
        L_0x036d:
            r6 = r34
        L_0x036f:
            java.lang.Class<?>[] r13 = r2.parameterTypes
            int r13 = r13.length
            if (r6 >= r13) goto L_0x036b
            java.lang.Class<org.bytedeco.javacpp.annotation.Index> r13 = org.bytedeco.javacpp.annotation.Index.class
            boolean r13 = r1.isAnnotationPresent(r13)
            if (r13 != 0) goto L_0x03a8
            if (r3 == 0) goto L_0x0386
            java.lang.Class<org.bytedeco.javacpp.annotation.Index> r13 = org.bytedeco.javacpp.annotation.Index.class
            boolean r13 = r3.isAnnotationPresent(r13)
            if (r13 != 0) goto L_0x03a8
        L_0x0386:
            java.lang.Class<?>[] r13 = r2.parameterTypes
            r13 = r13[r6]
            java.lang.Class r14 = java.lang.Integer.TYPE
            if (r13 == r14) goto L_0x03a8
            java.lang.Class<?>[] r13 = r2.parameterTypes
            r13 = r13[r6]
            java.lang.Class r14 = java.lang.Long.TYPE
            if (r13 == r14) goto L_0x03a8
            r13 = 0
            java.lang.Class<?>[] r14 = r2.parameterTypes
            int r14 = r14.length
            r16 = 1
            int r14 = r14 + -1
            if (r6 >= r14) goto L_0x03a6
            r14 = 0
            r16 = r13
            r19 = r14
            goto L_0x03a8
        L_0x03a6:
            r16 = r13
        L_0x03a8:
            int r6 = r6 + 1
            goto L_0x036f
        L_0x03ab:
            r32 = r10
            r30 = r13
            goto L_0x03b6
        L_0x03b0:
            r29 = r6
            r32 = r10
            r30 = r13
        L_0x03b6:
            r12 = r25
        L_0x03b8:
            r14 = r26
            r13 = r30
            int r11 = r11 + 1
            r4 = r20
            r5 = r21
            r3 = r23
            r6 = r29
            r10 = r32
            goto L_0x013d
        L_0x03ca:
            r23 = r3
            r29 = r6
            r32 = r10
            r25 = r12
            r30 = r13
            r26 = r14
            java.lang.annotation.Annotation[] r3 = r2.annotations
            java.lang.annotation.Annotation r3 = r0.behavior(r3)
            r4 = 0
            if (r7 == 0) goto L_0x03ec
            boolean r5 = r3 instanceof org.bytedeco.javacpp.annotation.ValueGetter
            if (r5 == 0) goto L_0x03ec
            r5 = 1
            r2.valueGetter = r5
            r2.noReturnGetter = r15
        L_0x03e8:
            r12 = r25
            goto L_0x050e
        L_0x03ec:
            if (r8 == 0) goto L_0x03f6
            boolean r5 = r3 instanceof org.bytedeco.javacpp.annotation.ValueSetter
            if (r5 == 0) goto L_0x03f6
            r5 = 1
            r2.valueSetter = r5
            goto L_0x03e8
        L_0x03f6:
            r5 = 1
            if (r7 == 0) goto L_0x0402
            boolean r6 = r3 instanceof org.bytedeco.javacpp.annotation.MemberGetter
            if (r6 == 0) goto L_0x0402
            r2.memberGetter = r5
            r2.noReturnGetter = r15
            goto L_0x03e8
        L_0x0402:
            if (r8 == 0) goto L_0x040c
            boolean r5 = r3 instanceof org.bytedeco.javacpp.annotation.MemberSetter
            if (r5 == 0) goto L_0x040c
            r5 = 1
            r2.memberSetter = r5
            goto L_0x03e8
        L_0x040c:
            r5 = 1
            if (r9 == 0) goto L_0x0416
            boolean r6 = r3 instanceof org.bytedeco.javacpp.annotation.Allocator
            if (r6 == 0) goto L_0x0416
            r2.allocator = r5
            goto L_0x03e8
        L_0x0416:
            if (r32 == 0) goto L_0x0421
            boolean r6 = r3 instanceof org.bytedeco.javacpp.annotation.ArrayAllocator
            if (r6 == 0) goto L_0x0421
            r2.arrayAllocator = r5
            r2.allocator = r5
            goto L_0x03e8
        L_0x0421:
            if (r3 != 0) goto L_0x04dc
            java.lang.Class<?> r5 = r2.returnType
            java.lang.Class r6 = java.lang.Void.TYPE
            if (r5 != r6) goto L_0x0456
            java.lang.String r5 = "deallocate"
            java.lang.String r6 = r2.name
            boolean r5 = r5.equals(r6)
            if (r5 == 0) goto L_0x0456
            int r5 = r2.modifiers
            boolean r5 = java.lang.reflect.Modifier.isStatic(r5)
            if (r5 != 0) goto L_0x0456
            java.lang.Class<?>[] r5 = r2.parameterTypes
            int r5 = r5.length
            r6 = 2
            if (r5 != r6) goto L_0x0456
            java.lang.Class<?>[] r5 = r2.parameterTypes
            r6 = 0
            r5 = r5[r6]
            java.lang.Class r6 = java.lang.Long.TYPE
            if (r5 != r6) goto L_0x0456
            java.lang.Class<?>[] r5 = r2.parameterTypes
            r6 = 1
            r5 = r5[r6]
            java.lang.Class r10 = java.lang.Long.TYPE
            if (r5 != r10) goto L_0x0456
            r2.deallocator = r6
            goto L_0x03e8
        L_0x0456:
            if (r9 == 0) goto L_0x0466
            java.lang.String r5 = "allocate"
            java.lang.String r6 = r2.name
            boolean r5 = r5.equals(r6)
            if (r5 == 0) goto L_0x0466
            r5 = 1
            r2.allocator = r5
            goto L_0x03e8
        L_0x0466:
            if (r32 == 0) goto L_0x0479
            java.lang.String r5 = "allocateArray"
            java.lang.String r6 = r2.name
            boolean r5 = r5.equals(r6)
            if (r5 == 0) goto L_0x0479
            r5 = 1
            r2.arrayAllocator = r5
            r2.allocator = r5
            goto L_0x03e8
        L_0x0479:
            java.lang.Class<?> r5 = r2.returnType
            java.lang.Class<java.nio.ByteBuffer> r6 = java.nio.ByteBuffer.class
            boolean r5 = r5.isAssignableFrom(r6)
            if (r5 == 0) goto L_0x049f
            java.lang.String r5 = "asDirectBuffer"
            java.lang.String r6 = r2.name
            boolean r5 = r5.equals(r6)
            if (r5 == 0) goto L_0x049f
            int r5 = r2.modifiers
            boolean r5 = java.lang.reflect.Modifier.isStatic(r5)
            if (r5 != 0) goto L_0x049f
            java.lang.Class<?>[] r5 = r2.parameterTypes
            int r5 = r5.length
            if (r5 != 0) goto L_0x049f
            r5 = 1
            r2.bufferGetter = r5
            goto L_0x03e8
        L_0x049f:
            if (r30 != 0) goto L_0x04d2
            if (r16 != 0) goto L_0x04b5
            if (r7 == 0) goto L_0x04b5
            java.lang.String r5 = "get"
            java.lang.String r6 = r2.name
            boolean r5 = r5.equals(r6)
            if (r5 == 0) goto L_0x04b5
            if (r29 == 0) goto L_0x04b5
            r12 = r25
            r5 = 1
            goto L_0x04d5
        L_0x04b5:
            if (r26 == 0) goto L_0x04bf
            r5 = 1
            r2.valueSetter = r5
            r12 = r25
            r2.pairedMethod = r12
            goto L_0x050e
        L_0x04bf:
            r12 = r25
            r5 = 1
            if (r16 == 0) goto L_0x04cb
            r2.memberGetter = r5
            r2.noReturnGetter = r15
            r2.pairedMethod = r12
            goto L_0x050e
        L_0x04cb:
            if (r19 == 0) goto L_0x050e
            r2.memberSetter = r5
            r2.pairedMethod = r12
            goto L_0x050e
        L_0x04d2:
            r12 = r25
            r5 = 1
        L_0x04d5:
            r2.valueGetter = r5
            r2.noReturnGetter = r15
            r2.pairedMethod = r12
            goto L_0x050e
        L_0x04dc:
            r12 = r25
            boolean r5 = r3 instanceof org.bytedeco.javacpp.annotation.Function
            if (r5 != 0) goto L_0x050e
            org.bytedeco.javacpp.tools.Logger r5 = r0.logger
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r10 = "Method \""
            r6.append(r10)
            r6.append(r1)
            java.lang.String r10 = "\" cannot behave like a \""
            r6.append(r10)
            java.lang.Class r10 = r3.annotationType()
            java.lang.String r10 = r10.getSimpleName()
            r6.append(r10)
            java.lang.String r10 = "\". No code will be generated."
            r6.append(r10)
            java.lang.String r6 = r6.toString()
            r5.warn(r6)
            return r4
        L_0x050e:
            if (r23 != 0) goto L_0x0527
            java.lang.reflect.Method r5 = r2.pairedMethod
            if (r5 == 0) goto L_0x0527
            java.lang.reflect.Method r5 = r2.pairedMethod
            java.lang.Class<org.bytedeco.javacpp.annotation.Name> r6 = org.bytedeco.javacpp.annotation.Name.class
            java.lang.annotation.Annotation r5 = r5.getAnnotation(r6)
            org.bytedeco.javacpp.annotation.Name r5 = (org.bytedeco.javacpp.annotation.Name) r5
            if (r5 == 0) goto L_0x0529
            java.lang.String[] r6 = r5.value()
            r2.memberName = r6
            goto L_0x0529
        L_0x0527:
            r5 = r23
        L_0x0529:
            java.lang.Class<?> r6 = r2.cls
            java.lang.Class<org.bytedeco.javacpp.annotation.NoOffset> r10 = org.bytedeco.javacpp.annotation.NoOffset.class
            boolean r6 = r6.isAnnotationPresent(r10)
            if (r6 != 0) goto L_0x0546
            java.lang.Class<org.bytedeco.javacpp.annotation.NoOffset> r6 = org.bytedeco.javacpp.annotation.NoOffset.class
            boolean r6 = r1.isAnnotationPresent(r6)
            if (r6 != 0) goto L_0x0546
            java.lang.Class<org.bytedeco.javacpp.annotation.Index> r6 = org.bytedeco.javacpp.annotation.Index.class
            boolean r6 = r1.isAnnotationPresent(r6)
            if (r6 == 0) goto L_0x0544
            goto L_0x0546
        L_0x0544:
            r6 = 0
            goto L_0x0547
        L_0x0546:
            r6 = 1
        L_0x0547:
            r2.noOffset = r6
            boolean r6 = r2.noOffset
            if (r6 != 0) goto L_0x056b
            java.lang.reflect.Method r6 = r2.pairedMethod
            if (r6 == 0) goto L_0x056b
            java.lang.reflect.Method r6 = r2.pairedMethod
            java.lang.Class<org.bytedeco.javacpp.annotation.NoOffset> r10 = org.bytedeco.javacpp.annotation.NoOffset.class
            boolean r6 = r6.isAnnotationPresent(r10)
            if (r6 != 0) goto L_0x0568
            java.lang.reflect.Method r6 = r2.pairedMethod
            java.lang.Class<org.bytedeco.javacpp.annotation.Index> r10 = org.bytedeco.javacpp.annotation.Index.class
            boolean r6 = r6.isAnnotationPresent(r10)
            if (r6 == 0) goto L_0x0566
            goto L_0x0568
        L_0x0566:
            r6 = 0
            goto L_0x0569
        L_0x0568:
            r6 = 1
        L_0x0569:
            r2.noOffset = r6
        L_0x056b:
            java.lang.Class<?>[] r6 = r2.parameterTypes
            int r6 = r6.length
            if (r6 == 0) goto L_0x057b
            java.lang.Class<?>[] r6 = r2.parameterTypes
            r10 = 0
            r6 = r6[r10]
            boolean r6 = r6.isArray()
            if (r6 != 0) goto L_0x05bb
        L_0x057b:
            boolean r6 = r2.valueGetter
            if (r6 != 0) goto L_0x0594
            boolean r6 = r2.memberGetter
            if (r6 == 0) goto L_0x0584
            goto L_0x0594
        L_0x0584:
            boolean r6 = r2.memberSetter
            if (r6 != 0) goto L_0x058c
            boolean r6 = r2.valueSetter
            if (r6 == 0) goto L_0x0599
        L_0x058c:
            java.lang.Class<?>[] r6 = r2.parameterTypes
            int r6 = r6.length
            r10 = 1
            int r6 = r6 - r10
            r2.dim = r6
            goto L_0x0599
        L_0x0594:
            java.lang.Class<?>[] r6 = r2.parameterTypes
            int r6 = r6.length
            r2.dim = r6
        L_0x0599:
            boolean r6 = r2.valueGetter
            if (r6 != 0) goto L_0x05a1
            boolean r6 = r2.valueSetter
            if (r6 == 0) goto L_0x05bb
        L_0x05a1:
            java.lang.Class<org.bytedeco.javacpp.FunctionPointer> r6 = org.bytedeco.javacpp.FunctionPointer.class
            java.lang.Class<?> r10 = r2.cls
            boolean r6 = r6.isAssignableFrom(r10)
            if (r6 == 0) goto L_0x05bb
            java.lang.Class<?> r6 = r2.cls
            java.lang.Class<org.bytedeco.javacpp.annotation.Namespace> r10 = org.bytedeco.javacpp.annotation.Namespace.class
            boolean r6 = r6.isAnnotationPresent(r10)
            if (r6 == 0) goto L_0x05bb
            int r6 = r2.dim
            r10 = 1
            int r6 = r6 - r10
            r2.dim = r6
        L_0x05bb:
            if (r12 == 0) goto L_0x05c6
            java.lang.Class<org.bytedeco.javacpp.annotation.Index> r6 = org.bytedeco.javacpp.annotation.Index.class
            java.lang.annotation.Annotation r6 = r12.getAnnotation(r6)
            org.bytedeco.javacpp.annotation.Index r6 = (org.bytedeco.javacpp.annotation.Index) r6
            goto L_0x05c7
        L_0x05c6:
            r6 = r4
        L_0x05c7:
            r2.throwsException = r4
            java.lang.Class<?> r4 = r2.cls
            boolean r4 = noException(r4, r1)
            if (r4 != 0) goto L_0x0622
            java.lang.annotation.Annotation[] r4 = r2.annotations
            java.lang.annotation.Annotation r4 = r0.by(r4)
            boolean r4 = r4 instanceof org.bytedeco.javacpp.annotation.ByVal
            if (r4 == 0) goto L_0x05e3
            java.lang.Class<?> r4 = r2.returnType
            boolean r4 = noException(r4, r1)
            if (r4 == 0) goto L_0x0613
        L_0x05e3:
            if (r29 == 0) goto L_0x05ef
            java.lang.String r4 = r29.function()
            int r4 = r4.length()
            if (r4 > 0) goto L_0x0613
        L_0x05ef:
            if (r6 == 0) goto L_0x05fb
            java.lang.String r4 = r6.function()
            int r4 = r4.length()
            if (r4 > 0) goto L_0x0613
        L_0x05fb:
            boolean r4 = r2.deallocator
            if (r4 != 0) goto L_0x0622
            boolean r4 = r2.valueGetter
            if (r4 != 0) goto L_0x0622
            boolean r4 = r2.valueSetter
            if (r4 != 0) goto L_0x0622
            boolean r4 = r2.memberGetter
            if (r4 != 0) goto L_0x0622
            boolean r4 = r2.memberSetter
            if (r4 != 0) goto L_0x0622
            boolean r4 = r2.bufferGetter
            if (r4 != 0) goto L_0x0622
        L_0x0613:
            java.lang.Class[] r4 = r37.getExceptionTypes()
            int r10 = r4.length
            if (r10 <= 0) goto L_0x061e
            r10 = 0
            r10 = r4[r10]
            goto L_0x0620
        L_0x061e:
            java.lang.Class<java.lang.RuntimeException> r10 = java.lang.RuntimeException.class
        L_0x0620:
            r2.throwsException = r10
        L_0x0622:
            return r2
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.Generator.methodInformation(java.lang.reflect.Method):org.bytedeco.javacpp.tools.MethodInformation");
    }

    static boolean criticalRegion(Class<?> cls, Method method) {
        Class<? super Object> cls2;
        boolean criticalRegion = baseClasses.contains(cls) || method.isAnnotationPresent(CriticalRegion.class);
        Class<?> cls3 = cls;
        while (!criticalRegion && cls3 != null) {
            boolean isAnnotationPresent = cls3.isAnnotationPresent(CriticalRegion.class);
            criticalRegion = isAnnotationPresent;
            if (isAnnotationPresent) {
                break;
            }
            if (cls3.getEnclosingClass() != null) {
                cls2 = cls3.getEnclosingClass();
            } else {
                cls2 = cls3.getSuperclass();
            }
            cls3 = cls2;
        }
        return criticalRegion;
    }

    static boolean noException(Class<?> cls, Method method) {
        Class<? super Object> cls2;
        boolean noException = baseClasses.contains(cls) || method.isAnnotationPresent(NoException.class);
        Class<?> cls3 = cls;
        while (!noException && cls3 != null) {
            boolean isAnnotationPresent = cls3.isAnnotationPresent(NoException.class);
            noException = isAnnotationPresent;
            if (isAnnotationPresent) {
                break;
            }
            if (cls3.getEnclosingClass() != null) {
                cls2 = cls3.getEnclosingClass();
            } else {
                cls2 = cls3.getSuperclass();
            }
            cls3 = cls2;
        }
        return noException;
    }

    /* access modifiers changed from: package-private */
    public AdapterInformation adapterInformation(boolean out3, MethodInformation methodInfo, int j) {
        if (out3 && (methodInfo.parameterTypes[j] == String.class || methodInfo.valueSetter || methodInfo.memberSetter)) {
            return null;
        }
        String typeName = cast(methodInfo, j);
        if (typeName != null && typeName.startsWith("(") && typeName.endsWith(")")) {
            typeName = typeName.substring(1, typeName.length() - 1);
        }
        if (typeName == null || typeName.length() == 0) {
            typeName = cppCastTypeName(methodInfo.parameterTypes[j], methodInfo.parameterAnnotations[j])[0];
        }
        String valueTypeName = valueTypeName(typeName);
        AdapterInformation adapter = adapterInformation(out3, valueTypeName, methodInfo.parameterAnnotations[j]);
        if (adapter != null || methodInfo.pairedMethod == null || j != methodInfo.parameterTypes.length - 1) {
            return adapter;
        }
        if (methodInfo.valueSetter || methodInfo.memberSetter) {
            return adapterInformation(out3, valueTypeName, methodInfo.pairedMethod.getAnnotations());
        }
        return adapter;
    }

    /* access modifiers changed from: package-private */
    public AdapterInformation adapterInformation(boolean out3, String valueTypeName, Annotation... annotations) {
        Annotation[] annotationArr = annotations;
        AdapterInformation adapterInfo = null;
        String cast = "";
        String cast2 = "";
        int i = 0;
        String valueTypeName2 = valueTypeName;
        for (Annotation a : annotationArr) {
            if (a instanceof Cast) {
                Cast c = (Cast) a;
                if (c.value().length > 0 && c.value()[0].length() > 0) {
                    valueTypeName2 = constValueTypeName(c.value()[0]);
                }
            }
        }
        int length = annotationArr.length;
        boolean constant = false;
        int i2 = 0;
        while (i2 < length) {
            Annotation a2 = annotationArr[i2];
            Adapter adapter = a2 instanceof Adapter ? (Adapter) a2 : (Adapter) a2.annotationType().getAnnotation(Adapter.class);
            if (adapter != null) {
                AdapterInformation adapterInfo2 = new AdapterInformation();
                adapterInfo2.name = adapter.value();
                adapterInfo2.argc = adapter.argc();
                if (a2 != adapter) {
                    try {
                        Class cls = a2.annotationType();
                        if (cls.isAnnotationPresent(Const.class)) {
                            constant = true;
                        }
                        try {
                            String value = cls.getDeclaredMethod(TypeSerializerImpl.VALUE_TAG, new Class[i]).invoke(a2, new Object[i]).toString();
                            if (value != null && value.length() > 0) {
                                valueTypeName2 = value;
                            }
                        } catch (NoSuchMethodException e) {
                            valueTypeName2 = null;
                        }
                        Cast c2 = (Cast) cls.getAnnotation(Cast.class);
                        if (c2 != null && cast.length() == 0) {
                            cast = c2.value()[i];
                            if (valueTypeName2 != null) {
                                cast = cast + "< " + valueTypeName2 + " >";
                            }
                            if (c2.value().length > 1) {
                                cast = cast + c2.value()[1];
                            }
                            if (c2.value().length > 2) {
                                cast2 = c2.value()[2];
                            }
                        }
                    } catch (Exception ex) {
                        this.logger.warn("Could not invoke the value() method on annotation \"" + a2 + "\": " + ex);
                    }
                    if (valueTypeName2 != null && valueTypeName2.length() > 0) {
                        adapterInfo2.name += "< " + valueTypeName2 + " >";
                    }
                }
                adapterInfo = adapterInfo2;
            } else {
                if (a2 instanceof Const) {
                    constant = true;
                } else if (a2 instanceof Cast) {
                    Cast c3 = (Cast) a2;
                    if (c3.value().length > 1) {
                        cast = c3.value()[1];
                    }
                    if (c3.value().length > 2) {
                        cast2 = c3.value()[2];
                    }
                }
            }
            i2++;
            i = 0;
        }
        if (adapterInfo != null) {
            adapterInfo.cast = cast;
            adapterInfo.cast2 = cast2;
            adapterInfo.constant = constant;
        }
        if (!out3 || !constant) {
            return adapterInfo;
        }
        return null;
    }

    /* access modifiers changed from: package-private */
    public String cast(MethodInformation methodInfo, int j) {
        String cast = cast(methodInfo.parameterTypes[j], methodInfo.parameterAnnotations[j]);
        if ((cast != null && cast.length() != 0) || j != methodInfo.parameterTypes.length - 1) {
            return cast;
        }
        if ((methodInfo.valueSetter || methodInfo.memberSetter) && methodInfo.pairedMethod != null) {
            return cast(methodInfo.pairedMethod.getReturnType(), methodInfo.pairedMethod.getAnnotations());
        }
        return cast;
    }

    /* access modifiers changed from: package-private */
    public String cast(Class<?> type, Annotation... annotations) {
        String[] typeName = null;
        int length = annotations.length;
        int i = 0;
        while (true) {
            if (i >= length) {
                break;
            }
            Cast cast = annotations[i];
            if ((!(cast instanceof Cast) || cast.value()[0].length() <= 0) && !(cast instanceof Const)) {
                i++;
            }
        }
        typeName = cppCastTypeName(type, annotations);
        if (typeName == null || typeName.length <= 0) {
            return "";
        }
        return "(" + typeName[0] + typeName[1] + ")";
    }

    /* access modifiers changed from: package-private */
    public Annotation by(MethodInformation methodInfo, int j) {
        Annotation passBy = by(methodInfo.parameterAnnotations[j]);
        if (passBy != null || methodInfo.pairedMethod == null) {
            return passBy;
        }
        if (methodInfo.valueSetter || methodInfo.memberSetter) {
            return by(methodInfo.pairedMethod.getAnnotations());
        }
        return passBy;
    }

    /* access modifiers changed from: package-private */
    public Annotation by(Annotation... annotations) {
        Annotation byAnnotation = null;
        for (Annotation a : annotations) {
            if ((a instanceof ByPtr) || (a instanceof ByPtrPtr) || (a instanceof ByPtrRef) || (a instanceof ByRef) || (a instanceof ByVal)) {
                if (byAnnotation != null) {
                    this.logger.warn("\"By\" annotation \"" + byAnnotation + "\" already found. Ignoring superfluous annotation \"" + a + "\".");
                } else {
                    byAnnotation = a;
                }
            }
        }
        return byAnnotation;
    }

    /* access modifiers changed from: package-private */
    public Annotation behavior(Annotation... annotations) {
        Annotation behaviorAnnotation = null;
        for (Annotation a : annotations) {
            if ((a instanceof Function) || (a instanceof Allocator) || (a instanceof ArrayAllocator) || (a instanceof ValueSetter) || (a instanceof ValueGetter) || (a instanceof MemberGetter) || (a instanceof MemberSetter)) {
                if (behaviorAnnotation != null) {
                    this.logger.warn("Behavior annotation \"" + behaviorAnnotation + "\" already found. Ignoring superfluous annotation \"" + a + "\".");
                } else {
                    behaviorAnnotation = a;
                }
            }
        }
        return behaviorAnnotation;
    }

    /* access modifiers changed from: package-private */
    public String enumValueType(Class<?> type) {
        try {
            Field f = type.getField(TypeSerializerImpl.VALUE_TAG);
            if (!f.getType().isPrimitive()) {
                Logger logger2 = this.logger;
                logger2.warn("Field \"value\" of enum type \"" + type.getCanonicalName() + "\" is not of a primitive type. Compilation will most likely fail.");
            }
            return f.getType().getName();
        } catch (NoSuchFieldException e) {
            Logger logger3 = this.logger;
            logger3.warn("Field \"value\" missing from enum type \"" + type.getCanonicalName() + ". Compilation will most likely fail.");
            return null;
        }
    }

    static String constValueTypeName(String... typeName) {
        String type = typeName[0];
        if (type.endsWith("*") || type.endsWith("&")) {
            return type.substring(0, type.length() - 1);
        }
        return type;
    }

    static String valueTypeName(String... typeName) {
        String type = typeName[0];
        if (type.startsWith("const ")) {
            return type.substring(6, type.length() - 1);
        }
        if (type.endsWith("*") || type.endsWith("&")) {
            return type.substring(0, type.length() - 1);
        }
        return type;
    }

    static boolean constFunction(Class<?> classType, Method functionMethod) {
        if (classType.isAnnotationPresent(Const.class)) {
            return true;
        }
        if (!functionMethod.isAnnotationPresent(Const.class)) {
            return false;
        }
        for (Annotation a : functionMethod.getDeclaredAnnotations()) {
            if (a instanceof Const) {
                boolean[] b = ((Const) a).value();
                if (b.length <= 2 || !b[2]) {
                    return false;
                }
                return true;
            }
        }
        return false;
    }

    /* access modifiers changed from: package-private */
    /* JADX WARNING: Code restructure failed: missing block: B:8:0x0028, code lost:
        r5 = true;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public java.lang.String[] cppAnnotationTypeName(java.lang.Class<?> r11, java.lang.annotation.Annotation... r12) {
        /*
            r10 = this;
            java.lang.String[] r0 = r10.cppCastTypeName(r11, r12)
            r1 = 0
            r2 = r0[r1]
            r3 = 1
            r4 = r0[r3]
            r5 = 0
            int r6 = r12.length
            r7 = 0
        L_0x000d:
            if (r7 >= r6) goto L_0x002d
            r8 = r12[r7]
            boolean r9 = r8 instanceof org.bytedeco.javacpp.annotation.Cast
            if (r9 == 0) goto L_0x0024
            r9 = r8
            org.bytedeco.javacpp.annotation.Cast r9 = (org.bytedeco.javacpp.annotation.Cast) r9
            java.lang.String[] r9 = r9.value()
            r9 = r9[r1]
            int r9 = r9.length()
            if (r9 > 0) goto L_0x0028
        L_0x0024:
            boolean r9 = r8 instanceof org.bytedeco.javacpp.annotation.Const
            if (r9 == 0) goto L_0x002a
        L_0x0028:
            r5 = 1
            goto L_0x002d
        L_0x002a:
            int r7 = r7 + 1
            goto L_0x000d
        L_0x002d:
            java.lang.annotation.Annotation r6 = r10.by(r12)
            boolean r7 = r6 instanceof org.bytedeco.javacpp.annotation.ByVal
            if (r7 == 0) goto L_0x003a
            java.lang.String r2 = constValueTypeName(r0)
            goto L_0x009d
        L_0x003a:
            boolean r7 = r6 instanceof org.bytedeco.javacpp.annotation.ByRef
            if (r7 == 0) goto L_0x0054
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r8 = constValueTypeName(r0)
            r7.append(r8)
            java.lang.String r8 = "&"
            r7.append(r8)
            java.lang.String r2 = r7.toString()
            goto L_0x009d
        L_0x0054:
            boolean r7 = r6 instanceof org.bytedeco.javacpp.annotation.ByPtrPtr
            if (r7 == 0) goto L_0x006c
            if (r5 != 0) goto L_0x006c
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            r7.append(r2)
            java.lang.String r8 = "*"
            r7.append(r8)
            java.lang.String r2 = r7.toString()
            goto L_0x009d
        L_0x006c:
            boolean r7 = r6 instanceof org.bytedeco.javacpp.annotation.ByPtrRef
            if (r7 == 0) goto L_0x0082
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            r7.append(r2)
            java.lang.String r8 = "&"
            r7.append(r8)
            java.lang.String r2 = r7.toString()
            goto L_0x009d
        L_0x0082:
            boolean r7 = r6 instanceof org.bytedeco.javacpp.annotation.ByPtr
            if (r7 == 0) goto L_0x009d
            boolean r7 = r11.isPrimitive()
            if (r7 == 0) goto L_0x009d
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            r7.append(r2)
            java.lang.String r8 = "*"
            r7.append(r8)
            java.lang.String r2 = r7.toString()
        L_0x009d:
            r0[r1] = r2
            r0[r3] = r4
            return r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.Generator.cppAnnotationTypeName(java.lang.Class, java.lang.annotation.Annotation[]):java.lang.String[]");
    }

    /* access modifiers changed from: package-private */
    public String[] cppCastTypeName(Class<?> type, Annotation... annotations) {
        Annotation[] annotationArr = annotations;
        boolean warning = false;
        boolean adapter = false;
        String[] typeName = null;
        for (Annotation a : annotationArr) {
            if (a instanceof Cast) {
                warning = typeName != null;
                String prefix = ((Cast) a).value()[0];
                String suffix = "";
                int templateCount = 0;
                int i = 0;
                while (true) {
                    if (i >= prefix.length()) {
                        break;
                    }
                    int c = prefix.charAt(i);
                    if (c != 60) {
                        if (c != 62) {
                            if (templateCount == 0 && c == 41) {
                                suffix = prefix.substring(i).trim();
                                prefix = prefix.substring(0, i).trim();
                                break;
                            }
                        } else {
                            templateCount--;
                        }
                    } else {
                        templateCount++;
                    }
                    i++;
                }
                typeName = prefix.length() > 0 ? new String[]{prefix, suffix} : null;
            } else if (a instanceof Const) {
                boolean[] b = ((Const) a).value();
                if ((b.length != 1 || b[0]) && (b.length <= 1 || b[0] || b[1])) {
                    boolean z = typeName != null;
                    warning = z;
                    if (!z) {
                        typeName = cppTypeName(type);
                        if (!typeName[0].contains("(*")) {
                            if (b.length > 1 && b[1] && !typeName[0].endsWith(" const *")) {
                                typeName[0] = valueTypeName(typeName) + " const *";
                            }
                            if (b.length > 0 && b[0] && !typeName[0].startsWith("const ")) {
                                typeName[0] = "const " + typeName[0];
                            }
                        } else if (b.length > 0 && b[0] && !typeName[0].endsWith("const")) {
                            typeName[0] = typeName[0] + "const";
                        }
                        Annotation by = by(annotationArr);
                        if (by instanceof ByPtrPtr) {
                            typeName[0] = typeName[0] + "*";
                        } else if (by instanceof ByPtrRef) {
                            typeName[0] = typeName[0] + "&";
                        }
                    }
                }
            } else if ((a instanceof Adapter) || a.annotationType().isAnnotationPresent(Adapter.class)) {
                adapter = true;
            }
        }
        if (warning && !adapter) {
            this.logger.warn("Without \"Adapter\", \"Cast\" and \"Const\" annotations are mutually exclusive.");
        }
        if (typeName == null) {
            return cppTypeName(type);
        }
        return typeName;
    }

    /* access modifiers changed from: package-private */
    public String[] cppTypeName(Class<?> type) {
        String prefix = "";
        if (type == Buffer.class || type == Pointer.class) {
            prefix = "void*";
        } else if (type == byte[].class || type == ByteBuffer.class || type == BytePointer.class) {
            prefix = "signed char*";
        } else if (type == short[].class || type == ShortBuffer.class || type == ShortPointer.class) {
            prefix = "short*";
        } else if (type == int[].class || type == IntBuffer.class || type == IntPointer.class) {
            prefix = "int*";
        } else if (type == long[].class || type == LongBuffer.class || type == LongPointer.class) {
            prefix = "jlong*";
        } else if (type == float[].class || type == FloatBuffer.class || type == FloatPointer.class) {
            prefix = "float*";
        } else if (type == double[].class || type == DoubleBuffer.class || type == DoublePointer.class) {
            prefix = "double*";
        } else if (type == char[].class || type == CharBuffer.class || type == CharPointer.class) {
            prefix = "unsigned short*";
        } else if (type == boolean[].class || type == BooleanPointer.class) {
            prefix = "unsigned char*";
        } else if (type == PointerPointer.class) {
            prefix = "void**";
        } else if (type == String.class) {
            prefix = "const char*";
        } else if (type == Byte.TYPE) {
            prefix = "signed char";
        } else if (type == Long.TYPE) {
            prefix = "jlong";
        } else if (type == Character.TYPE) {
            prefix = "unsigned short";
        } else if (type == Boolean.TYPE) {
            prefix = "unsigned char";
        } else if (type.isPrimitive()) {
            prefix = type.getName();
        } else if (FunctionPointer.class.isAssignableFrom(type)) {
            String[] prefixSuffix = cppFunctionTypeName(functionMethods(type, (boolean[]) null));
            if (prefixSuffix != null) {
                return prefixSuffix;
            }
        } else {
            String scopedType = cppScopeName(type);
            if (scopedType.length() > 0) {
                StringBuilder sb = new StringBuilder();
                sb.append(scopedType);
                sb.append(Enum.class.isAssignableFrom(type) ? "" : "*");
                prefix = sb.toString();
            } else {
                Logger logger2 = this.logger;
                logger2.warn("The class " + type.getCanonicalName() + " does not map to any C++ type. Compilation will most likely fail.");
            }
        }
        return new String[]{prefix, ""};
    }

    /* access modifiers changed from: package-private */
    public String[] cppFunctionTypeName(Method... functionMethods) {
        String callingConvention;
        String prefix;
        int i;
        Generator generator = this;
        Method[] methodArr = functionMethods;
        Method functionMethod = null;
        if (methodArr != null) {
            int length = methodArr.length;
            int i2 = 0;
            while (true) {
                if (i2 >= length) {
                    break;
                }
                Method m = methodArr[i2];
                if (m != null) {
                    functionMethod = m;
                    break;
                }
                i2++;
            }
        }
        Namespace namespace = null;
        if (functionMethod == null) {
            return null;
        }
        Class<?> type = functionMethod.getDeclaringClass();
        Convention convention = (Convention) type.getAnnotation(Convention.class);
        if (convention == null) {
            callingConvention = "";
        } else {
            callingConvention = convention.value() + " ";
        }
        if (FunctionPointer.class.isAssignableFrom(type)) {
            namespace = (Namespace) type.getAnnotation(Namespace.class);
        }
        if (namespace != null && namespace.value().length() == 0) {
            namespace = null;
        }
        String spaceName = namespace == null ? "" : namespace.value();
        if (spaceName.length() > 0 && !spaceName.endsWith("::")) {
            spaceName = spaceName + "::";
        }
        Class returnType = functionMethod.getReturnType();
        Class[] parameterTypes = functionMethod.getParameterTypes();
        Annotation[] annotations = functionMethod.getAnnotations();
        Annotation[][] parameterAnnotations = functionMethod.getParameterAnnotations();
        String[] returnTypeName = generator.cppAnnotationTypeName(returnType, annotations);
        AdapterInformation returnAdapterInfo = generator.adapterInformation(false, valueTypeName(returnTypeName), annotations);
        if (returnAdapterInfo == null || returnAdapterInfo.cast.length() <= 0) {
            StringBuilder sb = new StringBuilder();
            Convention convention2 = convention;
            sb.append(returnTypeName[0]);
            sb.append(returnTypeName[1]);
            prefix = sb.toString();
        } else {
            prefix = returnAdapterInfo.cast;
            Convention convention3 = convention;
        }
        String prefix2 = prefix + " (" + callingConvention + spaceName + "*";
        String suffix = ")";
        String str = callingConvention;
        if (functionMethod == methodArr[0]) {
            String suffix2 = suffix + "(";
            if (FunctionPointer.class.isAssignableFrom(type) && namespace != null && (parameterTypes.length == 0 || !Pointer.class.isAssignableFrom(parameterTypes[0]))) {
                generator.logger.warn("First parameter of caller method call() or apply() for member function pointer " + type.getCanonicalName() + " is not a Pointer. Compilation will most likely fail.");
            }
            int j = namespace == null ? 0 : 1;
            while (j < parameterTypes.length) {
                String[] paramTypeName = generator.cppAnnotationTypeName(parameterTypes[j], parameterAnnotations[j]);
                Namespace namespace2 = namespace;
                String spaceName2 = spaceName;
                AdapterInformation paramAdapterInfo = generator.adapterInformation(false, valueTypeName(paramTypeName), parameterAnnotations[j]);
                if (paramAdapterInfo != null && paramAdapterInfo.constant) {
                    suffix2 = suffix2 + "const ";
                }
                if (paramAdapterInfo == null || paramAdapterInfo.cast.length() <= 0) {
                    StringBuilder sb2 = new StringBuilder();
                    sb2.append(suffix2);
                    sb2.append(paramTypeName[0]);
                    sb2.append(" arg");
                    sb2.append(j);
                    i = 1;
                    sb2.append(paramTypeName[1]);
                    suffix2 = sb2.toString();
                } else {
                    suffix2 = suffix2 + paramAdapterInfo.cast + " arg" + j;
                    i = 1;
                }
                if (j < parameterTypes.length - i) {
                    suffix2 = suffix2 + ", ";
                }
                j++;
                namespace = namespace2;
                spaceName = spaceName2;
                generator = this;
            }
            String str2 = spaceName;
            suffix = suffix2 + ")";
        } else {
            String str3 = spaceName;
        }
        if (constFunction(type, functionMethod)) {
            suffix = suffix + " const";
        }
        return new String[]{prefix2, suffix};
    }

    static String cppScopeName(MethodInformation methodInfo) {
        String scopeName = cppScopeName(methodInfo.cls);
        if (methodInfo.method.isAnnotationPresent(Virtual.class)) {
            scopeName = "JavaCPP_" + mangle(scopeName);
        }
        Namespace namespace = (Namespace) methodInfo.method.getAnnotation(Namespace.class);
        String spaceName = namespace == null ? "" : namespace.value();
        if ((namespace != null && namespace.value().length() == 0) || spaceName.startsWith("::")) {
            scopeName = "";
        }
        if (scopeName.length() > 0 && !scopeName.endsWith("::")) {
            scopeName = scopeName + "::";
        }
        String scopeName2 = scopeName + spaceName;
        if (spaceName.length() > 0 && !spaceName.endsWith("::")) {
            scopeName2 = scopeName2 + "::";
        }
        return scopeName2 + methodInfo.memberName[0];
    }

    static String cppScopeName(Class<?> type) {
        String s;
        String scopeName = "";
        while (type != null) {
            Namespace namespace = (Namespace) type.getAnnotation(Namespace.class);
            String spaceName = namespace == null ? "" : namespace.value();
            if ((Enum.class.isAssignableFrom(type) || Pointer.class.isAssignableFrom(type)) && (!baseClasses.contains(type) || type.isAnnotationPresent(Name.class))) {
                Name name = (Name) type.getAnnotation(Name.class);
                if (name == null) {
                    String s2 = type.getName();
                    int i = s2.lastIndexOf("$");
                    if (i < 0) {
                        i = s2.lastIndexOf(".");
                    }
                    s = s2.substring(i + 1);
                } else {
                    s = name.value()[0];
                }
                if (spaceName.length() > 0 && !spaceName.endsWith("::")) {
                    spaceName = spaceName + "::";
                }
                spaceName = spaceName + s;
            }
            if (scopeName.length() > 0 && !scopeName.startsWith("class ") && !scopeName.startsWith("struct ") && !scopeName.startsWith("union ") && !spaceName.endsWith("::")) {
                spaceName = spaceName + "::";
            }
            scopeName = spaceName + scopeName;
            if ((namespace != null && namespace.value().length() == 0) || spaceName.startsWith("::")) {
                break;
            }
            type = type.getEnclosingClass();
        }
        return scopeName;
    }

    static String jniTypeName(Class type) {
        if (type == Byte.TYPE) {
            return "jbyte";
        }
        if (type == Short.TYPE) {
            return "jshort";
        }
        if (type == Integer.TYPE) {
            return "jint";
        }
        if (type == Long.TYPE) {
            return "jlong";
        }
        if (type == Float.TYPE) {
            return "jfloat";
        }
        if (type == Double.TYPE) {
            return "jdouble";
        }
        if (type == Character.TYPE) {
            return "jchar";
        }
        if (type == Boolean.TYPE) {
            return "jboolean";
        }
        if (type == byte[].class) {
            return "jbyteArray";
        }
        if (type == short[].class) {
            return "jshortArray";
        }
        if (type == int[].class) {
            return "jintArray";
        }
        if (type == long[].class) {
            return "jlongArray";
        }
        if (type == float[].class) {
            return "jfloatArray";
        }
        if (type == double[].class) {
            return "jdoubleArray";
        }
        if (type == char[].class) {
            return "jcharArray";
        }
        if (type == boolean[].class) {
            return "jbooleanArray";
        }
        if (type.isArray()) {
            return "jobjectArray";
        }
        if (type == String.class) {
            return "jstring";
        }
        if (type == Class.class) {
            return "jclass";
        }
        if (type == Void.TYPE) {
            return "void";
        }
        return "jobject";
    }

    static String signature(Class... types) {
        StringBuilder signature = new StringBuilder(types.length * 2);
        for (Class type : types) {
            if (type == Byte.TYPE) {
                signature.append("B");
            } else if (type == Short.TYPE) {
                signature.append("S");
            } else if (type == Integer.TYPE) {
                signature.append("I");
            } else if (type == Long.TYPE) {
                signature.append("J");
            } else if (type == Float.TYPE) {
                signature.append("F");
            } else if (type == Double.TYPE) {
                signature.append("D");
            } else if (type == Boolean.TYPE) {
                signature.append("Z");
            } else if (type == Character.TYPE) {
                signature.append("C");
            } else if (type == Void.TYPE) {
                signature.append("V");
            } else if (type.isArray()) {
                signature.append(type.getName().replace('.', IOUtils.DIR_SEPARATOR_UNIX));
            } else {
                signature.append("L");
                signature.append(type.getName().replace('.', IOUtils.DIR_SEPARATOR_UNIX));
                signature.append(";");
            }
        }
        return signature.toString();
    }

    /* JADX WARNING: Code restructure failed: missing block: B:32:0x006f, code lost:
        r0.append("0");
     */
    /* JADX WARNING: Code restructure failed: missing block: B:33:0x0074, code lost:
        r0.append("0");
     */
    /* JADX WARNING: Code restructure failed: missing block: B:34:0x0079, code lost:
        r0.append(r3);
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    static java.lang.String mangle(java.lang.String r5) {
        /*
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            int r1 = r5.length()
            int r1 = r1 * 2
            r0.<init>(r1)
            r1 = 0
        L_0x000c:
            int r2 = r5.length()
            if (r1 >= r2) goto L_0x0085
            char r2 = r5.charAt(r1)
            r3 = 48
            if (r2 < r3) goto L_0x001e
            r3 = 57
            if (r2 <= r3) goto L_0x002e
        L_0x001e:
            r3 = 65
            if (r2 < r3) goto L_0x0026
            r3 = 90
            if (r2 <= r3) goto L_0x002e
        L_0x0026:
            r3 = 97
            if (r2 < r3) goto L_0x0032
            r3 = 122(0x7a, float:1.71E-43)
            if (r2 > r3) goto L_0x0032
        L_0x002e:
            r0.append(r2)
            goto L_0x0082
        L_0x0032:
            r3 = 95
            if (r2 != r3) goto L_0x003c
            java.lang.String r3 = "_1"
            r0.append(r3)
            goto L_0x0082
        L_0x003c:
            r3 = 59
            if (r2 != r3) goto L_0x0046
            java.lang.String r3 = "_2"
            r0.append(r3)
            goto L_0x0082
        L_0x0046:
            r3 = 91
            if (r2 != r3) goto L_0x0050
            java.lang.String r3 = "_3"
            r0.append(r3)
            goto L_0x0082
        L_0x0050:
            r3 = 46
            if (r2 == r3) goto L_0x007d
            r3 = 47
            if (r2 != r3) goto L_0x0059
            goto L_0x007d
        L_0x0059:
            java.lang.String r3 = java.lang.Integer.toHexString(r2)
            java.lang.String r4 = "_0"
            r0.append(r4)
            int r4 = r3.length()
            switch(r4) {
                case 1: goto L_0x006a;
                case 2: goto L_0x006f;
                case 3: goto L_0x0074;
                default: goto L_0x0069;
            }
        L_0x0069:
            goto L_0x0079
        L_0x006a:
            java.lang.String r4 = "0"
            r0.append(r4)
        L_0x006f:
            java.lang.String r4 = "0"
            r0.append(r4)
        L_0x0074:
            java.lang.String r4 = "0"
            r0.append(r4)
        L_0x0079:
            r0.append(r3)
            goto L_0x0082
        L_0x007d:
            java.lang.String r3 = "_"
            r0.append(r3)
        L_0x0082:
            int r1 = r1 + 1
            goto L_0x000c
        L_0x0085:
            java.lang.String r1 = r0.toString()
            return r1
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.Generator.mangle(java.lang.String):java.lang.String");
    }
}
