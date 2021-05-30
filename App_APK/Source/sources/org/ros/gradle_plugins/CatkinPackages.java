package org.ros.gradle_plugins;

import groovy.lang.Closure;
import groovy.lang.GroovyObject;
import groovy.lang.MetaClass;
import groovy.lang.Reference;
import java.lang.ref.SoftReference;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.apache.commons.httpclient.cookie.Cookie2;
import org.codehaus.groovy.reflection.ClassInfo;
import org.codehaus.groovy.runtime.GStringImpl;
import org.codehaus.groovy.runtime.GeneratedClosure;
import org.codehaus.groovy.runtime.ScriptBytecodeAdapter;
import org.codehaus.groovy.runtime.callsite.CallSite;
import org.codehaus.groovy.runtime.callsite.CallSiteArray;
import org.codehaus.groovy.runtime.typehandling.DefaultTypeTransformation;
import org.gradle.api.Project;
import org.gradle.api.tasks.JavaExec;

/* compiled from: CatkinPlugin.groovy */
public class CatkinPackages implements GroovyObject {
    private static /* synthetic */ SoftReference $callSiteArray;
    private static /* synthetic */ ClassInfo $staticClassInfo;
    public static transient /* synthetic */ boolean __$stMC;
    private transient /* synthetic */ MetaClass metaClass = $getStaticMetaClass();
    private Map<String, CatkinPackage> pkgs;
    private Project project;
    private List<String> workspaces;

    private static /* synthetic */ CallSiteArray $createCallSiteArray() {
        String[] strArr = new String[30];
        $createCallSiteArray_1(strArr);
        return new CallSiteArray(CatkinPackages.class, strArr);
    }

    private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
        strArr[0] = "size";
        strArr[1] = "each";
        strArr[2] = "getAt";
        strArr[3] = "each";
        strArr[4] = "dependencies";
        strArr[5] = "getAt";
        strArr[6] = "version";
        strArr[7] = "add";
        strArr[8] = "dependencies";
        strArr[9] = "getMessageDependencies";
        strArr[10] = "each";
        strArr[11] = "buildDir";
        strArr[12] = "create";
        strArr[13] = "tasks";
        strArr[14] = "plus";
        strArr[15] = "name";
        strArr[16] = "dir";
        strArr[17] = "outputs";
        strArr[18] = HttpPostBodyUtil.FILE;
        strArr[19] = "<$constructor$>";
        strArr[20] = "plus";
        strArr[21] = "directory";
        strArr[22] = "name";
        strArr[23] = "runtime";
        strArr[24] = "configurations";
        strArr[25] = "source";
        strArr[26] = "compileJava";
        strArr[27] = "tasks";
        strArr[28] = "files";
        strArr[29] = "outputs";
    }

    private static /* synthetic */ CallSite[] $getCallSiteArray() {
        CallSiteArray callSiteArray;
        if ($callSiteArray == null || (callSiteArray = (CallSiteArray) $callSiteArray.get()) == null) {
            callSiteArray = $createCallSiteArray();
            $callSiteArray = new SoftReference(callSiteArray);
        }
        return callSiteArray.array;
    }

    /* access modifiers changed from: protected */
    public /* synthetic */ MetaClass $getStaticMetaClass() {
        if (getClass() != CatkinPackages.class) {
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

    public Map<String, CatkinPackage> getPkgs() {
        return this.pkgs;
    }

    public Project getProject() {
        return this.project;
    }

    public /* synthetic */ Object getProperty(String str) {
        return getMetaClass().getProperty(this, str);
    }

    public List<String> getWorkspaces() {
        return this.workspaces;
    }

    public /* synthetic */ Object invokeMethod(String str, Object obj) {
        return getMetaClass().invokeMethod(this, str, obj);
    }

    public /* synthetic */ void setMetaClass(MetaClass metaClass2) {
        this.metaClass = metaClass2;
    }

    public void setPkgs(Map<String, CatkinPackage> map) {
        this.pkgs = map;
    }

    public void setProject(Project project2) {
        this.project = project2;
    }

    public /* synthetic */ void setProperty(String str, Object obj) {
        getMetaClass().setProperty(this, str, obj);
    }

    public void setWorkspaces(List<String> list) {
        this.workspaces = list;
    }

    public CatkinPackages(Project project2, List<String> workspaces2) {
        $getCallSiteArray();
        this.project = (Project) ScriptBytecodeAdapter.castToType(project2, Project.class);
        this.workspaces = (List) ScriptBytecodeAdapter.castToType(workspaces2, List.class);
        this.pkgs = ScriptBytecodeAdapter.createMap(new Object[0]);
    }

    public void generate() {
        CallSite[] $getCallSiteArray = $getCallSiteArray();
        if (ScriptBytecodeAdapter.compareEqual($getCallSiteArray[0].call(this.pkgs), 0)) {
            $getCallSiteArray[1].call(this.workspaces, new _generate_closure1(this, this));
        }
    }

    /* compiled from: CatkinPlugin.groovy */
    public class _generate_closure1 extends Closure implements GeneratedClosure {
        private static /* synthetic */ SoftReference $callSiteArray;
        private static /* synthetic */ ClassInfo $staticClassInfo;
        public static transient /* synthetic */ boolean __$stMC;

        private static /* synthetic */ CallSiteArray $createCallSiteArray() {
            String[] strArr = new String[3];
            $createCallSiteArray_1(strArr);
            return new CallSiteArray(_generate_closure1.class, strArr);
        }

        private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
            strArr[0] = "fileTree";
            strArr[1] = "project";
            strArr[2] = "each";
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
        public _generate_closure1(Object obj, Object obj2) {
            super(obj, obj2);
            $getCallSiteArray();
        }

        /* access modifiers changed from: protected */
        public /* synthetic */ MetaClass $getStaticMetaClass() {
            if (getClass() != _generate_closure1.class) {
                return ScriptBytecodeAdapter.initMetaClass(this);
            }
            ClassInfo classInfo = $staticClassInfo;
            if (classInfo == null) {
                classInfo = ClassInfo.getClassInfo(getClass());
                $staticClassInfo = classInfo;
            }
            return classInfo.getMetaClass();
        }

        public Object doCall(Object workspace) {
            CallSite[] $getCallSiteArray = $getCallSiteArray();
            return $getCallSiteArray[2].call($getCallSiteArray[0].call($getCallSiteArray[1].callGroovyObjectGetProperty(this), ScriptBytecodeAdapter.createMap(new Object[]{"dir", workspace, "include", "**/package.xml"})), new _closure4(this, getThisObject()));
        }

        /* compiled from: CatkinPlugin.groovy */
        public class _closure4 extends Closure implements GeneratedClosure {
            private static /* synthetic */ SoftReference $callSiteArray;
            private static /* synthetic */ ClassInfo $staticClassInfo;
            public static transient /* synthetic */ boolean __$stMC;

            private static /* synthetic */ CallSiteArray $createCallSiteArray() {
                String[] strArr = new String[29];
                $createCallSiteArray_1(strArr);
                return new CallSiteArray(_closure4.class, strArr);
            }

            private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
                strArr[0] = "<$constructor$>";
                strArr[1] = "project";
                strArr[2] = "containsKey";
                strArr[3] = "pkgs";
                strArr[4] = "name";
                strArr[5] = "version";
                strArr[6] = "getAt";
                strArr[7] = "pkgs";
                strArr[8] = "name";
                strArr[9] = "version";
                strArr[10] = "println";
                strArr[11] = "plus";
                strArr[12] = "plus";
                strArr[13] = "plus";
                strArr[14] = "plus";
                strArr[15] = "plus";
                strArr[16] = "plus";
                strArr[17] = "name";
                strArr[18] = "version";
                strArr[19] = "getAt";
                strArr[20] = "pkgs";
                strArr[21] = "name";
                strArr[22] = "version";
                strArr[23] = "putAt";
                strArr[24] = "pkgs";
                strArr[25] = "name";
                strArr[26] = "put";
                strArr[27] = "pkgs";
                strArr[28] = "name";
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

            public Object doCall(Object file) {
                CallSite[] $getCallSiteArray = $getCallSiteArray();
                Object pkg = $getCallSiteArray[0].callConstructor(CatkinPackage.class, $getCallSiteArray[1].callGroovyObjectGetProperty(this), file);
                if (!DefaultTypeTransformation.booleanUnbox($getCallSiteArray[2].call($getCallSiteArray[3].callGroovyObjectGetProperty(getThisObject()), $getCallSiteArray[4].callGetProperty(pkg)))) {
                    return $getCallSiteArray[26].call($getCallSiteArray[27].callGroovyObjectGetProperty(this), $getCallSiteArray[28].callGetProperty(pkg), pkg);
                }
                if (!ScriptBytecodeAdapter.compareLessThan($getCallSiteArray[5].callGetProperty($getCallSiteArray[6].call($getCallSiteArray[7].callGroovyObjectGetProperty(getThisObject()), $getCallSiteArray[8].callGetProperty(pkg))), $getCallSiteArray[9].callGetProperty(pkg))) {
                    return null;
                }
                $getCallSiteArray[10].callCurrent(this, $getCallSiteArray[11].call($getCallSiteArray[12].call($getCallSiteArray[13].call($getCallSiteArray[14].call($getCallSiteArray[15].call($getCallSiteArray[16].call("Catkin generate tree: replacing older version of ", $getCallSiteArray[17].callGetProperty(pkg)), "["), $getCallSiteArray[18].callGetProperty($getCallSiteArray[19].call($getCallSiteArray[20].callGroovyObjectGetProperty(getThisObject()), $getCallSiteArray[21].callGetProperty(pkg)))), "->"), $getCallSiteArray[22].callGetProperty(pkg)), "]"));
                $getCallSiteArray[23].call($getCallSiteArray[24].callGroovyObjectGetProperty(this), $getCallSiteArray[25].callGetProperty(pkg), pkg);
                return pkg;
            }
        }
    }

    public Boolean isMessagePackage(String package_name) {
        CallSite[] $getCallSiteArray = $getCallSiteArray();
        Reference result = new Reference(false);
        try {
            $getCallSiteArray[3].call($getCallSiteArray[4].callGetProperty($getCallSiteArray[2].call(this.pkgs, package_name)), new _isMessagePackage_closure2(this, this, result));
        } catch (NullPointerException e) {
            result.set(false);
        }
        return (Boolean) ScriptBytecodeAdapter.castToType(result.get(), Boolean.class);
    }

    /* compiled from: CatkinPlugin.groovy */
    public class _isMessagePackage_closure2 extends Closure implements GeneratedClosure {
        private static /* synthetic */ SoftReference $callSiteArray;
        private static /* synthetic */ ClassInfo $staticClassInfo;
        public static transient /* synthetic */ boolean __$stMC;
        private /* synthetic */ Reference result;

        private static /* synthetic */ CallSiteArray $createCallSiteArray() {
            String[] strArr = new String[1];
            $createCallSiteArray_1(strArr);
            return new CallSiteArray(_isMessagePackage_closure2.class, strArr);
        }

        private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
            strArr[0] = "equalsIgnoreCase";
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
        public _isMessagePackage_closure2(Object obj, Object obj2, Reference reference) {
            super(obj, obj2);
            $getCallSiteArray();
            this.result = reference;
        }

        /* access modifiers changed from: protected */
        public /* synthetic */ MetaClass $getStaticMetaClass() {
            if (getClass() != _isMessagePackage_closure2.class) {
                return ScriptBytecodeAdapter.initMetaClass(this);
            }
            ClassInfo classInfo = $staticClassInfo;
            if (classInfo == null) {
                classInfo = ClassInfo.getClassInfo(getClass());
                $staticClassInfo = classInfo;
            }
            return classInfo.getMetaClass();
        }

        public Object getResult() {
            $getCallSiteArray();
            return this.result.get();
        }

        public Object doCall(Object d) {
            if (!DefaultTypeTransformation.booleanUnbox($getCallSiteArray()[0].call(d, "message_generation"))) {
                return null;
            }
            this.result.set(true);
            return true;
        }
    }

    public void generateMessageArtifact(Project project2, String package_name) {
        Reference project3 = new Reference(project2);
        CallSite[] $getCallSiteArray = $getCallSiteArray();
        Object pkg = $getCallSiteArray[5].call(this.pkgs, package_name);
        ScriptBytecodeAdapter.setProperty($getCallSiteArray[6].callGetProperty(pkg), (Class) null, (Project) project3.get(), "version");
        $getCallSiteArray[7].call($getCallSiteArray[8].callGetProperty((Project) project3.get()), "compile", "org.ros.rosjava_bootstrap:message_generation:[0.3,0.4)");
        $getCallSiteArray[10].call((Set) ScriptBytecodeAdapter.castToType($getCallSiteArray[9].call(pkg), Set.class), new _generateMessageArtifact_closure3(this, this, project3));
        Object generatedSourcesDir = new GStringImpl(new Object[]{$getCallSiteArray[11].callGetProperty((Project) project3.get())}, new String[]{"", "/generated-src"});
        Object generateSourcesTask = $getCallSiteArray[12].call($getCallSiteArray[13].callGetProperty((Project) project3.get()), "generateSources", JavaExec.class);
        ScriptBytecodeAdapter.setProperty($getCallSiteArray[14].call("Generate sources for ", $getCallSiteArray[15].callGetProperty(pkg)), (Class) null, generateSourcesTask, "description");
        $getCallSiteArray[16].call($getCallSiteArray[17].callGetProperty(generateSourcesTask), $getCallSiteArray[18].call((Project) project3.get(), generatedSourcesDir));
        ScriptBytecodeAdapter.setProperty($getCallSiteArray[19].callConstructor(ArrayList.class, ScriptBytecodeAdapter.createList(new Object[]{generatedSourcesDir, $getCallSiteArray[20].call("--package-path=", $getCallSiteArray[21].callGetProperty(pkg)), $getCallSiteArray[22].callGetProperty(pkg)})), (Class) null, generateSourcesTask, "args");
        ScriptBytecodeAdapter.setProperty($getCallSiteArray[23].callGetProperty($getCallSiteArray[24].callGetProperty((Project) project3.get())), (Class) null, generateSourcesTask, "classpath");
        ScriptBytecodeAdapter.setProperty("org.ros.internal.message.GenerateInterfaces", (Class) null, generateSourcesTask, "main");
        $getCallSiteArray[25].call($getCallSiteArray[26].callGetProperty($getCallSiteArray[27].callGetProperty((Project) project3.get())), $getCallSiteArray[28].callGetProperty($getCallSiteArray[29].callGetProperty(generateSourcesTask)));
    }

    /* compiled from: CatkinPlugin.groovy */
    public class _generateMessageArtifact_closure3 extends Closure implements GeneratedClosure {
        private static /* synthetic */ SoftReference $callSiteArray;
        private static /* synthetic */ ClassInfo $staticClassInfo;
        public static transient /* synthetic */ boolean __$stMC;
        private /* synthetic */ Reference project;

        private static /* synthetic */ CallSiteArray $createCallSiteArray() {
            String[] strArr = new String[12];
            $createCallSiteArray_1(strArr);
            return new CallSiteArray(_generateMessageArtifact_closure3.class, strArr);
        }

        private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
            strArr[0] = "containsKey";
            strArr[1] = "getChildProjects";
            strArr[2] = "getParent";
            strArr[3] = "add";
            strArr[4] = "dependencies";
            strArr[5] = "project";
            strArr[6] = "dependencies";
            strArr[7] = "plus";
            strArr[8] = "add";
            strArr[9] = "dependencies";
            strArr[10] = "plus";
            strArr[11] = "plus";
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
        public _generateMessageArtifact_closure3(Object obj, Object obj2, Reference reference) {
            super(obj, obj2);
            $getCallSiteArray();
            this.project = reference;
        }

        /* access modifiers changed from: protected */
        public /* synthetic */ MetaClass $getStaticMetaClass() {
            if (getClass() != _generateMessageArtifact_closure3.class) {
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

        public Object doCall(Object d) {
            CallSite[] $getCallSiteArray = $getCallSiteArray();
            if (!DefaultTypeTransformation.booleanUnbox($getCallSiteArray[0].call($getCallSiteArray[1].call($getCallSiteArray[2].call(this.project.get())), d))) {
                return $getCallSiteArray[8].call($getCallSiteArray[9].callGetProperty(this.project.get()), "compile", $getCallSiteArray[10].call($getCallSiteArray[11].call("org.ros.rosjava_messages:", d), ":[0.0,)"));
            }
            return $getCallSiteArray[3].call($getCallSiteArray[4].callGetProperty(this.project.get()), "compile", $getCallSiteArray[5].call($getCallSiteArray[6].callGetProperty(this.project.get()), ScriptBytecodeAdapter.createMap(new Object[]{Cookie2.PATH, $getCallSiteArray[7].call(":", d)})));
        }
    }
}
