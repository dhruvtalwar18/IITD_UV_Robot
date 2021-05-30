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
import org.codehaus.groovy.runtime.typehandling.ShortTypeHandling;
import org.gradle.api.Plugin;
import org.gradle.api.Project;

/* compiled from: RosPlugin.groovy */
public class RosPlugin implements Plugin<Project>, GroovyObject {
    private static /* synthetic */ SoftReference $callSiteArray;
    private static /* synthetic */ ClassInfo $staticClassInfo;
    public static transient /* synthetic */ boolean __$stMC;
    private transient /* synthetic */ MetaClass metaClass = $getStaticMetaClass();

    private static /* synthetic */ CallSiteArray $createCallSiteArray() {
        String[] strArr = new String[11];
        $createCallSiteArray_1(strArr);
        return new CallSiteArray(RosPlugin.class, strArr);
    }

    private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
        strArr[0] = "apply";
        strArr[1] = "create";
        strArr[2] = "extensions";
        strArr[3] = "getenv";
        strArr[4] = "ros";
        strArr[5] = "getenv";
        strArr[6] = "ros";
        strArr[7] = "getenv";
        strArr[8] = "tokenize";
        strArr[9] = "ros";
        strArr[10] = "repositories";
    }

    private static /* synthetic */ CallSite[] $getCallSiteArray() {
        CallSiteArray callSiteArray;
        if ($callSiteArray == null || (callSiteArray = (CallSiteArray) $callSiteArray.get()) == null) {
            callSiteArray = $createCallSiteArray();
            $callSiteArray = new SoftReference(callSiteArray);
        }
        return callSiteArray.array;
    }

    public RosPlugin() {
        $getCallSiteArray();
    }

    /* access modifiers changed from: protected */
    public /* synthetic */ MetaClass $getStaticMetaClass() {
        if (getClass() != RosPlugin.class) {
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
        $getCallSiteArray[0].call((Project) project2.get(), ScriptBytecodeAdapter.createMap(new Object[]{"plugin", "maven"}));
        $getCallSiteArray[1].call($getCallSiteArray[2].callGetProperty((Project) project2.get()), "ros", RosPluginExtension.class);
        ScriptBytecodeAdapter.setProperty($getCallSiteArray[3].call(System.class, "ROS_MAVEN_REPOSITORY"), (Class) null, $getCallSiteArray[4].callGetProperty((Project) project2.get()), "mavenRepository");
        ScriptBytecodeAdapter.setProperty($getCallSiteArray[5].call(System.class, "ROS_MAVEN_DEPLOYMENT_REPOSITORY"), (Class) null, $getCallSiteArray[6].callGetProperty((Project) project2.get()), "mavenDeploymentRepository");
        String mavenPath = ShortTypeHandling.castToString($getCallSiteArray[7].call(System.class, "ROS_MAVEN_PATH"));
        if (ScriptBytecodeAdapter.compareNotEqual(mavenPath, (Object) null)) {
            ScriptBytecodeAdapter.setProperty($getCallSiteArray[8].call(mavenPath, ":"), (Class) null, $getCallSiteArray[9].callGetProperty((Project) project2.get()), "mavenPath");
        }
        $getCallSiteArray[10].call((Project) project2.get(), new _apply_closure1(this, this, project2));
    }

    /* compiled from: RosPlugin.groovy */
    public class _apply_closure1 extends Closure implements GeneratedClosure {
        private static /* synthetic */ SoftReference $callSiteArray;
        private static /* synthetic */ ClassInfo $staticClassInfo;
        public static transient /* synthetic */ boolean __$stMC;
        private /* synthetic */ Reference project;

        private static /* synthetic */ CallSiteArray $createCallSiteArray() {
            String[] strArr = new String[13];
            $createCallSiteArray_1(strArr);
            return new CallSiteArray(_apply_closure1.class, strArr);
        }

        private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
            strArr[0] = "mavenPath";
            strArr[1] = "ros";
            strArr[2] = "each";
            strArr[3] = "mavenPath";
            strArr[4] = "ros";
            strArr[5] = "mavenRepository";
            strArr[6] = "ros";
            strArr[7] = "maven";
            strArr[8] = "maven";
            strArr[9] = "mavenLocal";
            strArr[10] = "maven";
            strArr[11] = "maven";
            strArr[12] = "jcenter";
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

        public Object doCall(Object it) {
            CallSite[] $getCallSiteArray = $getCallSiteArray();
            if (ScriptBytecodeAdapter.compareNotEqual($getCallSiteArray[0].callGetProperty($getCallSiteArray[1].callGetProperty(this.project.get())), (Object) null)) {
                $getCallSiteArray[2].call($getCallSiteArray[3].callGetProperty($getCallSiteArray[4].callGetProperty(this.project.get())), new _closure2(this, getThisObject(), this.project));
            }
            if (ScriptBytecodeAdapter.compareNotEqual($getCallSiteArray[5].callGetProperty($getCallSiteArray[6].callGetProperty(this.project.get())), (Object) null)) {
                $getCallSiteArray[7].callCurrent(this, new _closure3(this, getThisObject(), this.project));
            }
            $getCallSiteArray[8].callCurrent(this, new _closure4(this, getThisObject()));
            $getCallSiteArray[9].callCurrent(this);
            $getCallSiteArray[10].callCurrent(this, new _closure5(this, getThisObject()));
            $getCallSiteArray[11].callCurrent(this, new _closure6(this, getThisObject()));
            return $getCallSiteArray[12].callCurrent(this);
        }

        /* compiled from: RosPlugin.groovy */
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

            public Project getProject() {
                $getCallSiteArray();
                return (Project) ScriptBytecodeAdapter.castToType(this.project.get(), Project.class);
            }

            /* compiled from: RosPlugin.groovy */
            public class _closure7 extends Closure implements GeneratedClosure {
                private static /* synthetic */ SoftReference $callSiteArray;
                private static /* synthetic */ ClassInfo $staticClassInfo;
                public static transient /* synthetic */ boolean __$stMC;
                private /* synthetic */ Reference path;
                private /* synthetic */ Reference project;

                private static /* synthetic */ CallSiteArray $createCallSiteArray() {
                    String[] strArr = new String[2];
                    $createCallSiteArray_1(strArr);
                    return new CallSiteArray(_closure7.class, strArr);
                }

                private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
                    strArr[0] = "url";
                    strArr[1] = "uri";
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
                public _closure7(Object obj, Object obj2, Reference reference, Reference reference2) {
                    super(obj, obj2);
                    $getCallSiteArray();
                    this.project = reference;
                    this.path = reference2;
                }

                /* access modifiers changed from: protected */
                public /* synthetic */ MetaClass $getStaticMetaClass() {
                    if (getClass() != _closure7.class) {
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

                public Object getPath() {
                    $getCallSiteArray();
                    return this.path.get();
                }

                public Project getProject() {
                    $getCallSiteArray();
                    return (Project) ScriptBytecodeAdapter.castToType(this.project.get(), Project.class);
                }

                public Object doCall(Object it) {
                    CallSite[] $getCallSiteArray = $getCallSiteArray();
                    return $getCallSiteArray[0].callCurrent(this, $getCallSiteArray[1].call(this.project.get(), this.path.get()));
                }
            }

            public Object doCall(Object path) {
                return $getCallSiteArray()[0].callCurrent(this, new _closure7(this, getThisObject(), this.project, new Reference(path)));
            }
        }

        /* compiled from: RosPlugin.groovy */
        public class _closure3 extends Closure implements GeneratedClosure {
            private static /* synthetic */ SoftReference $callSiteArray;
            private static /* synthetic */ ClassInfo $staticClassInfo;
            public static transient /* synthetic */ boolean __$stMC;
            private /* synthetic */ Reference project;

            private static /* synthetic */ CallSiteArray $createCallSiteArray() {
                String[] strArr = new String[3];
                $createCallSiteArray_1(strArr);
                return new CallSiteArray(_closure3.class, strArr);
            }

            private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
                strArr[0] = "url";
                strArr[1] = "mavenRepository";
                strArr[2] = "ros";
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
                return $getCallSiteArray[0].callCurrent(this, $getCallSiteArray[1].callGetProperty($getCallSiteArray[2].callGetProperty(this.project.get())));
            }
        }

        /* compiled from: RosPlugin.groovy */
        public class _closure4 extends Closure implements GeneratedClosure {
            private static /* synthetic */ SoftReference $callSiteArray;
            private static /* synthetic */ ClassInfo $staticClassInfo;
            public static transient /* synthetic */ boolean __$stMC;

            private static /* synthetic */ CallSiteArray $createCallSiteArray() {
                String[] strArr = new String[1];
                $createCallSiteArray_1(strArr);
                return new CallSiteArray(_closure4.class, strArr);
            }

            private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
                strArr[0] = "url";
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
            public _closure4(Object obj, Object obj2) {
                super(obj, obj2);
                $getCallSiteArray();
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

            public Object doCall(Object it) {
                return $getCallSiteArray()[0].callCurrent(this, "https://github.com/rosjava/rosjava_mvn_repo/raw/master");
            }
        }

        /* compiled from: RosPlugin.groovy */
        public class _closure5 extends Closure implements GeneratedClosure {
            private static /* synthetic */ SoftReference $callSiteArray;
            private static /* synthetic */ ClassInfo $staticClassInfo;
            public static transient /* synthetic */ boolean __$stMC;

            private static /* synthetic */ CallSiteArray $createCallSiteArray() {
                String[] strArr = new String[1];
                $createCallSiteArray_1(strArr);
                return new CallSiteArray(_closure5.class, strArr);
            }

            private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
                strArr[0] = "url";
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
            public _closure5(Object obj, Object obj2) {
                super(obj, obj2);
                $getCallSiteArray();
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

            public Object doCall(Object it) {
                return $getCallSiteArray()[0].callCurrent(this, "http://repository.springsource.com/maven/bundles/release");
            }
        }

        /* compiled from: RosPlugin.groovy */
        public class _closure6 extends Closure implements GeneratedClosure {
            private static /* synthetic */ SoftReference $callSiteArray;
            private static /* synthetic */ ClassInfo $staticClassInfo;
            public static transient /* synthetic */ boolean __$stMC;

            private static /* synthetic */ CallSiteArray $createCallSiteArray() {
                String[] strArr = new String[1];
                $createCallSiteArray_1(strArr);
                return new CallSiteArray(_closure6.class, strArr);
            }

            private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
                strArr[0] = "url";
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
            public _closure6(Object obj, Object obj2) {
                super(obj, obj2);
                $getCallSiteArray();
            }

            /* access modifiers changed from: protected */
            public /* synthetic */ MetaClass $getStaticMetaClass() {
                if (getClass() != _closure6.class) {
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

            public Object doCall(Object it) {
                return $getCallSiteArray()[0].callCurrent(this, "http://repository.springsource.com/maven/bundles/external");
            }
        }
    }
}
