package org.ros.gradle_plugins;

import groovy.lang.Closure;
import groovy.lang.GroovyObject;
import groovy.lang.MetaClass;
import groovy.lang.Reference;
import java.lang.ref.SoftReference;
import java.math.BigDecimal;
import org.codehaus.groovy.reflection.ClassInfo;
import org.codehaus.groovy.runtime.BytecodeInterface8;
import org.codehaus.groovy.runtime.GeneratedClosure;
import org.codehaus.groovy.runtime.ScriptBytecodeAdapter;
import org.codehaus.groovy.runtime.callsite.CallSite;
import org.codehaus.groovy.runtime.callsite.CallSiteArray;
import org.codehaus.groovy.runtime.typehandling.DefaultTypeTransformation;
import org.gradle.api.Plugin;
import org.gradle.api.Project;
import org.gradle.api.publish.maven.MavenPublication;

/* compiled from: RosJavaPlugin.groovy */
public class RosJavaPlugin implements Plugin<Project>, GroovyObject {
    private static /* synthetic */ SoftReference $callSiteArray;
    private static final /* synthetic */ BigDecimal $const$0 = null;
    private static /* synthetic */ ClassInfo $staticClassInfo;
    public static transient /* synthetic */ boolean __$stMC;
    private transient /* synthetic */ MetaClass metaClass = $getStaticMetaClass();
    private Project project;

    private static /* synthetic */ CallSiteArray $createCallSiteArray() {
        String[] strArr = new String[19];
        $createCallSiteArray_1(strArr);
        return new CallSiteArray(RosJavaPlugin.class, strArr);
    }

    private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
        strArr[0] = "findPlugin";
        strArr[1] = "plugins";
        strArr[2] = "apply";
        strArr[3] = "findPlugin";
        strArr[4] = "plugins";
        strArr[5] = "apply";
        strArr[6] = "findPlugin";
        strArr[7] = "plugins";
        strArr[8] = "apply";
        strArr[9] = "mavenDeploymentRepository";
        strArr[10] = "ros";
        strArr[11] = "mavenDeploymentRepository";
        strArr[12] = "ros";
        strArr[13] = "publishing";
        strArr[14] = "mavenDeploymentRepository";
        strArr[15] = "ros";
        strArr[16] = "mavenDeploymentRepository";
        strArr[17] = "ros";
        strArr[18] = "publishing";
    }

    private static /* synthetic */ CallSite[] $getCallSiteArray() {
        CallSiteArray callSiteArray;
        if ($callSiteArray == null || (callSiteArray = (CallSiteArray) $callSiteArray.get()) == null) {
            callSiteArray = $createCallSiteArray();
            $callSiteArray = new SoftReference(callSiteArray);
        }
        return callSiteArray.array;
    }

    static {
        __$swapInit();
    }

    public RosJavaPlugin() {
        $getCallSiteArray();
    }

    public static /* synthetic */ void __$swapInit() {
        $getCallSiteArray();
        $callSiteArray = null;
        $const$0 = new BigDecimal("1.7");
    }

    /* access modifiers changed from: protected */
    public /* synthetic */ MetaClass $getStaticMetaClass() {
        if (getClass() != RosJavaPlugin.class) {
            return ScriptBytecodeAdapter.initMetaClass(this);
        }
        ClassInfo classInfo = $staticClassInfo;
        if (classInfo == null) {
            classInfo = ClassInfo.getClassInfo(getClass());
            $staticClassInfo = classInfo;
        }
        return classInfo.getMetaClass();
    }

    public /* synthetic */ MetaClass getMetaClass() {
        MetaClass metaClass2 = this.metaClass;
        if (metaClass2 != null) {
            return metaClass2;
        }
        this.metaClass = $getStaticMetaClass();
        return this.metaClass;
    }

    public Project getProject() {
        return this.project;
    }

    public /* synthetic */ Object getProperty(String str) {
        return getMetaClass().getProperty(this, str);
    }

    public /* synthetic */ Object invokeMethod(String str, Object obj) {
        return getMetaClass().invokeMethod(this, str, obj);
    }

    public /* synthetic */ void setMetaClass(MetaClass metaClass2) {
        this.metaClass = metaClass2;
    }

    public void setProject(Project project2) {
        this.project = project2;
    }

    public /* synthetic */ void setProperty(String str, Object obj) {
        getMetaClass().setProperty(this, str, obj);
    }

    public void apply(Project project2) {
        CallSite callSite;
        Project project3;
        _apply_closure1 _apply_closure12;
        Reference project4 = new Reference(project2);
        CallSite[] $getCallSiteArray = $getCallSiteArray();
        this.project = (Project) ScriptBytecodeAdapter.castToType((Project) project4.get(), Project.class);
        boolean z = false;
        if (!DefaultTypeTransformation.booleanUnbox($getCallSiteArray[0].call($getCallSiteArray[1].callGetProperty((Project) project4.get()), "ros"))) {
            $getCallSiteArray[2].call((Project) project4.get(), ScriptBytecodeAdapter.createMap(new Object[]{"plugin", "ros"}));
        }
        if (!DefaultTypeTransformation.booleanUnbox($getCallSiteArray[3].call($getCallSiteArray[4].callGetProperty((Project) project4.get()), "java"))) {
            $getCallSiteArray[5].call((Project) project4.get(), ScriptBytecodeAdapter.createMap(new Object[]{"plugin", "java"}));
        }
        if (!DefaultTypeTransformation.booleanUnbox($getCallSiteArray[6].call($getCallSiteArray[7].callGetProperty((Project) project4.get()), "maven-publish"))) {
            $getCallSiteArray[8].call((Project) project4.get(), ScriptBytecodeAdapter.createMap(new Object[]{"plugin", "maven-publish"}));
        }
        ScriptBytecodeAdapter.setProperty($const$0, (Class) null, (Project) project4.get(), "sourceCompatibility");
        ScriptBytecodeAdapter.setProperty($const$0, (Class) null, (Project) project4.get(), "targetCompatibility");
        if (!BytecodeInterface8.isOrigZ() || __$stMC || BytecodeInterface8.disabledStandardMetaClass()) {
            if (ScriptBytecodeAdapter.compareNotEqual($getCallSiteArray[9].callGetProperty($getCallSiteArray[10].callGetProperty((Project) project4.get())), "null") && ScriptBytecodeAdapter.compareNotEqual($getCallSiteArray[11].callGetProperty($getCallSiteArray[12].callGetProperty((Project) project4.get())), "")) {
                z = true;
            }
            if (z) {
                callSite = $getCallSiteArray[13];
                project3 = (Project) project4.get();
                _apply_closure12 = new _apply_closure1(this, this, project4);
            } else {
                return;
            }
        } else {
            if (ScriptBytecodeAdapter.compareNotEqual($getCallSiteArray[14].callGetProperty($getCallSiteArray[15].callGetProperty((Project) project4.get())), "null") && ScriptBytecodeAdapter.compareNotEqual($getCallSiteArray[16].callGetProperty($getCallSiteArray[17].callGetProperty((Project) project4.get())), "")) {
                z = true;
            }
            if (z) {
                callSite = $getCallSiteArray[18];
                project3 = (Project) project4.get();
                _apply_closure12 = new _apply_closure1(this, this, project4);
            } else {
                return;
            }
        }
        callSite.call(project3, _apply_closure12);
    }

    /* compiled from: RosJavaPlugin.groovy */
    public class _apply_closure1 extends Closure implements GeneratedClosure {
        private static /* synthetic */ SoftReference $callSiteArray;
        private static /* synthetic */ ClassInfo $staticClassInfo;
        public static transient /* synthetic */ boolean __$stMC;
        private /* synthetic */ Reference project;

        private static /* synthetic */ CallSiteArray $createCallSiteArray() {
            String[] strArr = new String[2];
            $createCallSiteArray_1(strArr);
            return new CallSiteArray(_apply_closure1.class, strArr);
        }

        private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
            strArr[0] = "publications";
            strArr[1] = "repositories";
        }

        private static /* synthetic */ CallSite[] $getCallSiteArray() {
            CallSiteArray callSiteArray;
            if ($callSiteArray == null || (callSiteArray = (CallSiteArray) $callSiteArray.get()) == null) {
                callSiteArray = $createCallSiteArray();
                $callSiteArray = new SoftReference(callSiteArray);
            }
            return callSiteArray.array;
        }

        /* JADX INFO: super call moved to the top of the method (can break code semantics) */
        public _apply_closure1(Object obj, Object obj2, Reference reference) {
            super(obj, obj2);
            $getCallSiteArray();
            this.project = reference;
        }

        /* access modifiers changed from: protected */
        public /* synthetic */ MetaClass $getStaticMetaClass() {
            if (getClass() != _apply_closure1.class) {
                return ScriptBytecodeAdapter.initMetaClass(this);
            }
            ClassInfo classInfo = $staticClassInfo;
            if (classInfo == null) {
                classInfo = ClassInfo.getClassInfo(getClass());
                $staticClassInfo = classInfo;
            }
            return classInfo.getMetaClass();
        }

        public Object doCall() {
            $getCallSiteArray();
            return doCall((Object) null);
        }

        public Project getProject() {
            $getCallSiteArray();
            return (Project) ScriptBytecodeAdapter.castToType(this.project.get(), Project.class);
        }

        /* compiled from: RosJavaPlugin.groovy */
        public class _closure2 extends Closure implements GeneratedClosure {
            private static /* synthetic */ SoftReference $callSiteArray;
            private static /* synthetic */ ClassInfo $staticClassInfo;
            public static transient /* synthetic */ boolean __$stMC;
            private /* synthetic */ Reference project;

            private static /* synthetic */ CallSiteArray $createCallSiteArray() {
                String[] strArr = new String[1];
                $createCallSiteArray_1(strArr);
                return new CallSiteArray(_closure2.class, strArr);
            }

            private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
                strArr[0] = "mavenJava";
            }

            private static /* synthetic */ CallSite[] $getCallSiteArray() {
                CallSiteArray callSiteArray;
                if ($callSiteArray == null || (callSiteArray = (CallSiteArray) $callSiteArray.get()) == null) {
                    callSiteArray = $createCallSiteArray();
                    $callSiteArray = new SoftReference(callSiteArray);
                }
                return callSiteArray.array;
            }

            /* JADX INFO: super call moved to the top of the method (can break code semantics) */
            public _closure2(Object obj, Object obj2, Reference reference) {
                super(obj, obj2);
                $getCallSiteArray();
                this.project = reference;
            }

            /* access modifiers changed from: protected */
            public /* synthetic */ MetaClass $getStaticMetaClass() {
                if (getClass() != _closure2.class) {
                    return ScriptBytecodeAdapter.initMetaClass(this);
                }
                ClassInfo classInfo = $staticClassInfo;
                if (classInfo == null) {
                    classInfo = ClassInfo.getClassInfo(getClass());
                    $staticClassInfo = classInfo;
                }
                return classInfo.getMetaClass();
            }

            public Object doCall() {
                $getCallSiteArray();
                return doCall((Object) null);
            }

            public Project getProject() {
                $getCallSiteArray();
                return (Project) ScriptBytecodeAdapter.castToType(this.project.get(), Project.class);
            }

            /* compiled from: RosJavaPlugin.groovy */
            public class _closure4 extends Closure implements GeneratedClosure {
                private static /* synthetic */ SoftReference $callSiteArray;
                private static /* synthetic */ ClassInfo $staticClassInfo;
                public static transient /* synthetic */ boolean __$stMC;
                private /* synthetic */ Reference project;

                private static /* synthetic */ CallSiteArray $createCallSiteArray() {
                    String[] strArr = new String[3];
                    $createCallSiteArray_1(strArr);
                    return new CallSiteArray(_closure4.class, strArr);
                }

                private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
                    strArr[0] = "from";
                    strArr[1] = "java";
                    strArr[2] = "components";
                }

                private static /* synthetic */ CallSite[] $getCallSiteArray() {
                    CallSiteArray callSiteArray;
                    if ($callSiteArray == null || (callSiteArray = (CallSiteArray) $callSiteArray.get()) == null) {
                        callSiteArray = $createCallSiteArray();
                        $callSiteArray = new SoftReference(callSiteArray);
                    }
                    return callSiteArray.array;
                }

                /* JADX INFO: super call moved to the top of the method (can break code semantics) */
                public _closure4(Object obj, Object obj2, Reference reference) {
                    super(obj, obj2);
                    $getCallSiteArray();
                    this.project = reference;
                }

                /* access modifiers changed from: protected */
                public /* synthetic */ MetaClass $getStaticMetaClass() {
                    if (getClass() != _closure4.class) {
                        return ScriptBytecodeAdapter.initMetaClass(this);
                    }
                    ClassInfo classInfo = $staticClassInfo;
                    if (classInfo == null) {
                        classInfo = ClassInfo.getClassInfo(getClass());
                        $staticClassInfo = classInfo;
                    }
                    return classInfo.getMetaClass();
                }

                public Object doCall() {
                    $getCallSiteArray();
                    return doCall((Object) null);
                }

                public Project getProject() {
                    $getCallSiteArray();
                    return (Project) ScriptBytecodeAdapter.castToType(this.project.get(), Project.class);
                }

                public Object doCall(Object it) {
                    CallSite[] $getCallSiteArray = $getCallSiteArray();
                    return $getCallSiteArray[0].callCurrent(this, $getCallSiteArray[1].callGetProperty($getCallSiteArray[2].callGetProperty(this.project.get())));
                }
            }

            public Object doCall(Object it) {
                return $getCallSiteArray()[0].callCurrent(this, MavenPublication.class, new _closure4(this, getThisObject(), this.project));
            }
        }

        public Object doCall(Object it) {
            CallSite[] $getCallSiteArray = $getCallSiteArray();
            $getCallSiteArray[0].callCurrent(this, new _closure2(this, getThisObject(), this.project));
            return $getCallSiteArray[1].callCurrent(this, new _closure3(this, getThisObject(), this.project));
        }

        /* compiled from: RosJavaPlugin.groovy */
        public class _closure3 extends Closure implements GeneratedClosure {
            private static /* synthetic */ SoftReference $callSiteArray;
            private static /* synthetic */ ClassInfo $staticClassInfo;
            public static transient /* synthetic */ boolean __$stMC;
            private /* synthetic */ Reference project;

            private static /* synthetic */ CallSiteArray $createCallSiteArray() {
                String[] strArr = new String[1];
                $createCallSiteArray_1(strArr);
                return new CallSiteArray(_closure3.class, strArr);
            }

            private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
                strArr[0] = "maven";
            }

            private static /* synthetic */ CallSite[] $getCallSiteArray() {
                CallSiteArray callSiteArray;
                if ($callSiteArray == null || (callSiteArray = (CallSiteArray) $callSiteArray.get()) == null) {
                    callSiteArray = $createCallSiteArray();
                    $callSiteArray = new SoftReference(callSiteArray);
                }
                return callSiteArray.array;
            }

            /* JADX INFO: super call moved to the top of the method (can break code semantics) */
            public _closure3(Object obj, Object obj2, Reference reference) {
                super(obj, obj2);
                $getCallSiteArray();
                this.project = reference;
            }

            /* access modifiers changed from: protected */
            public /* synthetic */ MetaClass $getStaticMetaClass() {
                if (getClass() != _closure3.class) {
                    return ScriptBytecodeAdapter.initMetaClass(this);
                }
                ClassInfo classInfo = $staticClassInfo;
                if (classInfo == null) {
                    classInfo = ClassInfo.getClassInfo(getClass());
                    $staticClassInfo = classInfo;
                }
                return classInfo.getMetaClass();
            }

            public Object doCall() {
                $getCallSiteArray();
                return doCall((Object) null);
            }

            public Project getProject() {
                $getCallSiteArray();
                return (Project) ScriptBytecodeAdapter.castToType(this.project.get(), Project.class);
            }

            /* compiled from: RosJavaPlugin.groovy */
            public class _closure5 extends Closure implements GeneratedClosure {
                private static /* synthetic */ SoftReference $callSiteArray;
                private static /* synthetic */ ClassInfo $staticClassInfo;
                public static transient /* synthetic */ boolean __$stMC;
                private /* synthetic */ Reference project;

                private static /* synthetic */ CallSiteArray $createCallSiteArray() {
                    String[] strArr = new String[4];
                    $createCallSiteArray_1(strArr);
                    return new CallSiteArray(_closure5.class, strArr);
                }

                private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
                    strArr[0] = "url";
                    strArr[1] = "plus";
                    strArr[2] = "mavenDeploymentRepository";
                    strArr[3] = "ros";
                }

                private static /* synthetic */ CallSite[] $getCallSiteArray() {
                    CallSiteArray callSiteArray;
                    if ($callSiteArray == null || (callSiteArray = (CallSiteArray) $callSiteArray.get()) == null) {
                        callSiteArray = $createCallSiteArray();
                        $callSiteArray = new SoftReference(callSiteArray);
                    }
                    return callSiteArray.array;
                }

                /* JADX INFO: super call moved to the top of the method (can break code semantics) */
                public _closure5(Object obj, Object obj2, Reference reference) {
                    super(obj, obj2);
                    $getCallSiteArray();
                    this.project = reference;
                }

                /* access modifiers changed from: protected */
                public /* synthetic */ MetaClass $getStaticMetaClass() {
                    if (getClass() != _closure5.class) {
                        return ScriptBytecodeAdapter.initMetaClass(this);
                    }
                    ClassInfo classInfo = $staticClassInfo;
                    if (classInfo == null) {
                        classInfo = ClassInfo.getClassInfo(getClass());
                        $staticClassInfo = classInfo;
                    }
                    return classInfo.getMetaClass();
                }

                public Object doCall() {
                    $getCallSiteArray();
                    return doCall((Object) null);
                }

                public Project getProject() {
                    $getCallSiteArray();
                    return (Project) ScriptBytecodeAdapter.castToType(this.project.get(), Project.class);
                }

                public Object doCall(Object it) {
                    CallSite[] $getCallSiteArray = $getCallSiteArray();
                    return $getCallSiteArray[0].callCurrent(this, $getCallSiteArray[1].call("file://", $getCallSiteArray[2].callGetProperty($getCallSiteArray[3].callGetProperty(this.project.get()))));
                }
            }

            public Object doCall(Object it) {
                return $getCallSiteArray()[0].callCurrent(this, new _closure5(this, getThisObject(), this.project));
            }
        }
    }
}
