package org.ros.gradle_plugins;

import groovy.lang.Closure;
import groovy.lang.GroovyObject;
import groovy.lang.MetaClass;
import java.lang.ref.SoftReference;
import org.apache.xmlrpc.serializer.TypeSerializerImpl;
import org.codehaus.groovy.reflection.ClassInfo;
import org.codehaus.groovy.runtime.BytecodeInterface8;
import org.codehaus.groovy.runtime.GStringImpl;
import org.codehaus.groovy.runtime.GeneratedClosure;
import org.codehaus.groovy.runtime.ScriptBytecodeAdapter;
import org.codehaus.groovy.runtime.callsite.CallSite;
import org.codehaus.groovy.runtime.callsite.CallSiteArray;
import org.codehaus.groovy.runtime.typehandling.DefaultTypeTransformation;
import org.gradle.api.Plugin;
import org.gradle.api.Project;
import org.ros.EnvironmentVariables;

/* compiled from: CatkinPlugin.groovy */
public class CatkinPlugin implements Plugin<Project>, GroovyObject {
    private static /* synthetic */ SoftReference $callSiteArray;
    private static /* synthetic */ ClassInfo $staticClassInfo;
    public static transient /* synthetic */ boolean __$stMC;
    private transient /* synthetic */ MetaClass metaClass = $getStaticMetaClass();
    private Project project;

    private static /* synthetic */ CallSiteArray $createCallSiteArray() {
        String[] strArr = new String[25];
        $createCallSiteArray_1(strArr);
        return new CallSiteArray(CatkinPlugin.class, strArr);
    }

    private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
        strArr[0] = "create";
        strArr[1] = "extensions";
        strArr[2] = "catkin";
        strArr[3] = "split";
        strArr[4] = EnvironmentVariables.ROS_PACKAGE_PATH;
        strArr[5] = "env";
        strArr[6] = "catkin";
        strArr[7] = "<$constructor$>";
        strArr[8] = "workspaces";
        strArr[9] = "catkin";
        strArr[10] = "catkin";
        strArr[11] = HttpPostBodyUtil.FILE;
        strArr[12] = "exists";
        strArr[13] = "getParent";
        strArr[14] = "getParentFile";
        strArr[15] = HttpPostBodyUtil.FILE;
        strArr[16] = HttpPostBodyUtil.FILE;
        strArr[17] = "<$constructor$>";
        strArr[18] = "catkin";
        strArr[19] = "generate";
        strArr[20] = "tree";
        strArr[21] = "catkin";
        strArr[22] = "setTasks";
        strArr[23] = "doLast";
        strArr[24] = "task";
    }

    private static /* synthetic */ CallSite[] $getCallSiteArray() {
        CallSiteArray callSiteArray;
        if ($callSiteArray == null || (callSiteArray = (CallSiteArray) $callSiteArray.get()) == null) {
            callSiteArray = $createCallSiteArray();
            $callSiteArray = new SoftReference(callSiteArray);
        }
        return callSiteArray.array;
    }

    public CatkinPlugin() {
        $getCallSiteArray();
    }

    /* access modifiers changed from: protected */
    public /* synthetic */ MetaClass $getStaticMetaClass() {
        if (getClass() != CatkinPlugin.class) {
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
        CallSite[] $getCallSiteArray = $getCallSiteArray();
        this.project = (Project) ScriptBytecodeAdapter.castToType(project2, Project.class);
        $getCallSiteArray[0].call($getCallSiteArray[1].callGetProperty(project2), "catkin", CatkinPluginExtension.class);
        ScriptBytecodeAdapter.setProperty(ScriptBytecodeAdapter.createList(new Object[0]), (Class) null, $getCallSiteArray[2].callGetProperty(project2), "workspaces");
        ScriptBytecodeAdapter.setProperty($getCallSiteArray[3].call(new GStringImpl(new Object[]{$getCallSiteArray[4].callGetProperty($getCallSiteArray[5].callGetProperty(System.class))}, new String[]{"", ""}), ":"), (Class) null, $getCallSiteArray[6].callGetProperty(project2), "workspaces");
        ScriptBytecodeAdapter.setProperty($getCallSiteArray[7].callConstructor(CatkinPackages.class, project2, $getCallSiteArray[8].callGetProperty($getCallSiteArray[9].callGetProperty(project2))), (Class) null, $getCallSiteArray[10].callGetProperty(project2), "tree");
        Object packageXml = $getCallSiteArray[11].call(project2, "package.xml");
        if (!DefaultTypeTransformation.booleanUnbox($getCallSiteArray[12].call(packageXml))) {
            Object call = $getCallSiteArray[13].call($getCallSiteArray[14].call($getCallSiteArray[15].callGroovyObjectGetProperty(this)));
            packageXml = $getCallSiteArray[16].call(project2, "package.xml");
        }
        if (ScriptBytecodeAdapter.compareNotEqual(packageXml, (Object) null)) {
            ScriptBytecodeAdapter.setProperty($getCallSiteArray[17].callConstructor(CatkinPackage.class, project2, packageXml), (Class) null, $getCallSiteArray[18].callGetProperty(project2), "pkg");
        }
        $getCallSiteArray[19].call($getCallSiteArray[20].callGetProperty($getCallSiteArray[21].callGetProperty(project2)));
        if (__$stMC || BytecodeInterface8.disabledStandardMetaClass()) {
            $getCallSiteArray[22].callCurrent(this);
        } else {
            setTasks();
        }
    }

    /* compiled from: CatkinPlugin.groovy */
    public class _setTasks_closure1 extends Closure implements GeneratedClosure {
        private static /* synthetic */ SoftReference $callSiteArray;
        private static /* synthetic */ ClassInfo $staticClassInfo;
        public static transient /* synthetic */ boolean __$stMC;

        private static /* synthetic */ CallSiteArray $createCallSiteArray() {
            String[] strArr = new String[12];
            $createCallSiteArray_1(strArr);
            return new CallSiteArray(_setTasks_closure1.class, strArr);
        }

        private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
            strArr[0] = "println";
            strArr[1] = "println";
            strArr[2] = "plus";
            strArr[3] = "workspaces";
            strArr[4] = "catkin";
            strArr[5] = "project";
            strArr[6] = "println";
            strArr[7] = "each";
            strArr[8] = "pkgs";
            strArr[9] = "tree";
            strArr[10] = "catkin";
            strArr[11] = "project";
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
        public _setTasks_closure1(Object obj, Object obj2) {
            super(obj, obj2);
            $getCallSiteArray();
        }

        /* access modifiers changed from: protected */
        public /* synthetic */ MetaClass $getStaticMetaClass() {
            if (getClass() != _setTasks_closure1.class) {
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
            CallSite[] $getCallSiteArray = $getCallSiteArray();
            $getCallSiteArray[0].callCurrent(this, "CatkinPlugin is happy, you should be too.");
            $getCallSiteArray[1].callCurrent(this, $getCallSiteArray[2].call("Catkin Workspaces.........", $getCallSiteArray[3].callGetProperty($getCallSiteArray[4].callGetProperty($getCallSiteArray[5].callGroovyObjectGetProperty(this)))));
            $getCallSiteArray[6].callCurrent(this, "Catkin Packages");
            return $getCallSiteArray[7].call($getCallSiteArray[8].callGetProperty($getCallSiteArray[9].callGetProperty($getCallSiteArray[10].callGetProperty($getCallSiteArray[11].callGroovyObjectGetProperty(this)))), new _closure2(this, getThisObject()));
        }

        /* compiled from: CatkinPlugin.groovy */
        public class _closure2 extends Closure implements GeneratedClosure {
            private static /* synthetic */ SoftReference $callSiteArray;
            private static /* synthetic */ ClassInfo $staticClassInfo;
            public static transient /* synthetic */ boolean __$stMC;

            private static /* synthetic */ CallSiteArray $createCallSiteArray() {
                String[] strArr = new String[3];
                $createCallSiteArray_1(strArr);
                return new CallSiteArray(_closure2.class, strArr);
            }

            private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
                strArr[0] = "println";
                strArr[1] = "toString";
                strArr[2] = TypeSerializerImpl.VALUE_TAG;
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
            public _closure2(Object obj, Object obj2) {
                super(obj, obj2);
                $getCallSiteArray();
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

            public Object doCall(Object pkg) {
                CallSite[] $getCallSiteArray = $getCallSiteArray();
                return $getCallSiteArray[0].callCurrent(this, $getCallSiteArray[1].call($getCallSiteArray[2].callGetProperty(pkg)));
            }
        }
    }

    public void setTasks() {
        CallSite[] $getCallSiteArray = $getCallSiteArray();
        $getCallSiteArray[23].call($getCallSiteArray[24].call(this.project, "catkinPackageInfo"), new _setTasks_closure1(this, this));
    }
}
