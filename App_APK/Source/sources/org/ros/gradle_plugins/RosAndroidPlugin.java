package org.ros.gradle_plugins;

import groovy.lang.Closure;
import groovy.lang.GroovyObject;
import groovy.lang.MetaClass;
import groovy.lang.Reference;
import java.lang.ref.SoftReference;
import org.codehaus.groovy.reflection.ClassInfo;
import org.codehaus.groovy.runtime.GeneratedClosure;
import org.codehaus.groovy.runtime.ScriptBytecodeAdapter;
import org.codehaus.groovy.runtime.callsite.CallSite;
import org.codehaus.groovy.runtime.callsite.CallSiteArray;
import org.gradle.api.Plugin;
import org.gradle.api.Project;

/* compiled from: RosAndroid.groovy */
public class RosAndroidPlugin implements Plugin<Project>, GroovyObject {
    private static /* synthetic */ SoftReference $callSiteArray;
    private static /* synthetic */ ClassInfo $staticClassInfo;
    public static transient /* synthetic */ boolean __$stMC;
    private transient /* synthetic */ MetaClass metaClass = $getStaticMetaClass();

    private static /* synthetic */ CallSiteArray $createCallSiteArray() {
        String[] strArr = new String[14];
        $createCallSiteArray_1(strArr);
        return new CallSiteArray(RosAndroidPlugin.class, strArr);
    }

    private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
        strArr[0] = "apply";
        strArr[1] = "create";
        strArr[2] = "extensions";
        strArr[3] = "rosandroid";
        strArr[4] = "uploadArchives";
        strArr[5] = "maybeCreate";
        strArr[6] = "configurations";
        strArr[7] = "exclude";
        strArr[8] = "compile";
        strArr[9] = "configurations";
        strArr[10] = "exclude";
        strArr[11] = "compile";
        strArr[12] = "configurations";
        strArr[13] = "afterEvaluate";
    }

    private static /* synthetic */ CallSite[] $getCallSiteArray() {
        CallSiteArray callSiteArray;
        if ($callSiteArray == null || (callSiteArray = (CallSiteArray) $callSiteArray.get()) == null) {
            callSiteArray = $createCallSiteArray();
            $callSiteArray = new SoftReference(callSiteArray);
        }
        return callSiteArray.array;
    }

    public RosAndroidPlugin() {
        $getCallSiteArray();
    }

    /* access modifiers changed from: protected */
    public /* synthetic */ MetaClass $getStaticMetaClass() {
        if (getClass() != RosAndroidPlugin.class) {
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

    public /* synthetic */ Object getProperty(String str) {
        return getMetaClass().getProperty(this, str);
    }

    public /* synthetic */ Object invokeMethod(String str, Object obj) {
        return getMetaClass().invokeMethod(this, str, obj);
    }

    public /* synthetic */ void setMetaClass(MetaClass metaClass2) {
        this.metaClass = metaClass2;
    }

    public /* synthetic */ void setProperty(String str, Object obj) {
        getMetaClass().setProperty(this, str, obj);
    }

    public void apply(Project project) {
        Reference project2 = new Reference(project);
        CallSite[] $getCallSiteArray = $getCallSiteArray();
        $getCallSiteArray[0].call((Project) project2.get(), ScriptBytecodeAdapter.createMap(new Object[]{"plugin", "ros"}));
        $getCallSiteArray[1].call($getCallSiteArray[2].callGetProperty((Project) project2.get()), "rosandroid", RosAndroidPluginExtension.class);
        ScriptBytecodeAdapter.setProperty("28.0.3", (Class) null, $getCallSiteArray[3].callGetProperty((Project) project2.get()), "buildToolsVersion");
        $getCallSiteArray[4].call((Project) project2.get(), new _apply_closure1(this, this, project2));
        $getCallSiteArray[5].call($getCallSiteArray[6].callGetProperty((Project) project2.get()), "compile");
        $getCallSiteArray[7].call($getCallSiteArray[8].callGetProperty($getCallSiteArray[9].callGetProperty((Project) project2.get())), ScriptBytecodeAdapter.createMap(new Object[]{"group", "junit"}));
        $getCallSiteArray[10].call($getCallSiteArray[11].callGetProperty($getCallSiteArray[12].callGetProperty((Project) project2.get())), ScriptBytecodeAdapter.createMap(new Object[]{"group", "xml-apis"}));
        $getCallSiteArray[13].call((Project) project2.get(), new _apply_closure2(this, this, project2));
    }

    /* compiled from: RosAndroid.groovy */
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
            strArr[0] = "mavenDeployer";
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

        /* compiled from: RosAndroid.groovy */
        public class _closure3 extends Closure implements GeneratedClosure {
            private static /* synthetic */ SoftReference $callSiteArray;
            private static /* synthetic */ ClassInfo $staticClassInfo;
            public static transient /* synthetic */ boolean __$stMC;
            private /* synthetic */ Reference project;

            private static /* synthetic */ CallSiteArray $createCallSiteArray() {
                String[] strArr = new String[4];
                $createCallSiteArray_1(strArr);
                return new CallSiteArray(_closure3.class, strArr);
            }

            private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
                strArr[0] = "repository";
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

            public Object doCall(Object it) {
                CallSite[] $getCallSiteArray = $getCallSiteArray();
                return $getCallSiteArray[0].callCurrent(this, ScriptBytecodeAdapter.createMap(new Object[]{"url", $getCallSiteArray[1].call("file://", $getCallSiteArray[2].callGetProperty($getCallSiteArray[3].callGetProperty(this.project.get())))}));
            }
        }

        public Object doCall(Object it) {
            CallSite[] $getCallSiteArray = $getCallSiteArray();
            return $getCallSiteArray[0].call($getCallSiteArray[1].callGroovyObjectGetProperty(this), new _closure3(this, getThisObject(), this.project));
        }
    }

    /* compiled from: RosAndroid.groovy */
    public class _apply_closure2 extends Closure implements GeneratedClosure {
        private static /* synthetic */ SoftReference $callSiteArray;
        private static /* synthetic */ ClassInfo $staticClassInfo;
        public static transient /* synthetic */ boolean __$stMC;
        private /* synthetic */ Reference project;

        private static /* synthetic */ CallSiteArray $createCallSiteArray() {
            String[] strArr = new String[1];
            $createCallSiteArray_1(strArr);
            return new CallSiteArray(_apply_closure2.class, strArr);
        }

        private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
            strArr[0] = "android";
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
        public _apply_closure2(Object obj, Object obj2, Reference reference) {
            super(obj, obj2);
            $getCallSiteArray();
            this.project = reference;
        }

        /* access modifiers changed from: protected */
        public /* synthetic */ MetaClass $getStaticMetaClass() {
            if (getClass() != _apply_closure2.class) {
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

        /* compiled from: RosAndroid.groovy */
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
                strArr[0] = "buildToolsVersion";
                strArr[1] = "buildToolsVersion";
                strArr[2] = "rosandroid";
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
            return $getCallSiteArray()[0].call(this.project.get(), new _closure4(this, getThisObject(), this.project));
        }
    }
}
