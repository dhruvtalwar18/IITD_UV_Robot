package org.ros.gradle_plugins;

import groovy.lang.Closure;
import groovy.lang.GroovyObject;
import groovy.lang.MetaClass;
import groovy.lang.Reference;
import groovy.util.XmlParser;
import java.io.File;
import java.lang.ref.SoftReference;
import java.util.Collection;
import java.util.Set;
import org.codehaus.groovy.reflection.ClassInfo;
import org.codehaus.groovy.runtime.GStringImpl;
import org.codehaus.groovy.runtime.GeneratedClosure;
import org.codehaus.groovy.runtime.ScriptBytecodeAdapter;
import org.codehaus.groovy.runtime.callsite.CallSite;
import org.codehaus.groovy.runtime.callsite.CallSiteArray;
import org.codehaus.groovy.runtime.typehandling.DefaultTypeTransformation;
import org.codehaus.groovy.runtime.typehandling.ShortTypeHandling;
import org.gradle.api.Project;

/* compiled from: CatkinPlugin.groovy */
public class CatkinPackage implements GroovyObject {
    private static /* synthetic */ SoftReference $callSiteArray;
    private static /* synthetic */ ClassInfo $staticClassInfo;
    public static transient /* synthetic */ boolean __$stMC;
    private Set<String> dependencies;
    private String directory;
    private transient /* synthetic */ MetaClass metaClass = $getStaticMetaClass();
    private String name;
    private Project project;
    private String version;

    private static /* synthetic */ CallSiteArray $createCallSiteArray() {
        String[] strArr = new String[12];
        $createCallSiteArray_1(strArr);
        return new CallSiteArray(CatkinPackage.class, strArr);
    }

    private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
        strArr[0] = "parse";
        strArr[1] = "<$constructor$>";
        strArr[2] = "parent";
        strArr[3] = "text";
        strArr[4] = "name";
        strArr[5] = "text";
        strArr[6] = "version";
        strArr[7] = "collect";
        strArr[8] = "build_depend";
        strArr[9] = "each";
        strArr[10] = "findAll";
        strArr[11] = "getTransitiveDependencies";
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
        if (getClass() != CatkinPackage.class) {
            return ScriptBytecodeAdapter.initMetaClass(this);
        }
        ClassInfo classInfo = $staticClassInfo;
        if (classInfo == null) {
            classInfo = ClassInfo.getClassInfo(getClass());
            $staticClassInfo = classInfo;
        }
        return classInfo.getMetaClass();
    }

    public Set<String> getDependencies() {
        return this.dependencies;
    }

    public String getDirectory() {
        return this.directory;
    }

    public /* synthetic */ MetaClass getMetaClass() {
        MetaClass metaClass2 = this.metaClass;
        if (metaClass2 != null) {
            return metaClass2;
        }
        this.metaClass = $getStaticMetaClass();
        return this.metaClass;
    }

    public String getName() {
        return this.name;
    }

    public Project getProject() {
        return this.project;
    }

    public /* synthetic */ Object getProperty(String str) {
        return getMetaClass().getProperty(this, str);
    }

    public String getVersion() {
        return this.version;
    }

    public /* synthetic */ Object invokeMethod(String str, Object obj) {
        return getMetaClass().invokeMethod(this, str, obj);
    }

    public void setDependencies(Set<String> set) {
        this.dependencies = set;
    }

    public void setDirectory(String str) {
        this.directory = str;
    }

    public /* synthetic */ void setMetaClass(MetaClass metaClass2) {
        this.metaClass = metaClass2;
    }

    public void setName(String str) {
        this.name = str;
    }

    public void setProject(Project project2) {
        this.project = project2;
    }

    public /* synthetic */ void setProperty(String str, Object obj) {
        getMetaClass().setProperty(this, str, obj);
    }

    public void setVersion(String str) {
        this.version = str;
    }

    public /* synthetic */ String super$1$toString() {
        return super.toString();
    }

    public CatkinPackage(Project project2, File packageXmlFilename) {
        CallSite[] $getCallSiteArray = $getCallSiteArray();
        this.project = (Project) ScriptBytecodeAdapter.castToType(project2, Project.class);
        Object packageXml = $getCallSiteArray[0].call($getCallSiteArray[1].callConstructor(XmlParser.class), packageXmlFilename);
        this.directory = ShortTypeHandling.castToString($getCallSiteArray[2].callGetProperty(packageXmlFilename));
        this.name = ShortTypeHandling.castToString($getCallSiteArray[3].call($getCallSiteArray[4].callGetProperty(packageXml)));
        this.version = ShortTypeHandling.castToString($getCallSiteArray[5].call($getCallSiteArray[6].callGetProperty(packageXml)));
        this.dependencies = (Set) ScriptBytecodeAdapter.castToType($getCallSiteArray[7].call($getCallSiteArray[8].callGetProperty(packageXml), new _closure1(this, this)), Set.class);
    }

    /* compiled from: CatkinPlugin.groovy */
    public class _closure1 extends Closure implements GeneratedClosure {
        private static /* synthetic */ SoftReference $callSiteArray;
        private static /* synthetic */ ClassInfo $staticClassInfo;
        public static transient /* synthetic */ boolean __$stMC;

        private static /* synthetic */ CallSiteArray $createCallSiteArray() {
            String[] strArr = new String[1];
            $createCallSiteArray_1(strArr);
            return new CallSiteArray(_closure1.class, strArr);
        }

        private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
            strArr[0] = "text";
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
        public _closure1(Object obj, Object obj2) {
            super(obj, obj2);
            $getCallSiteArray();
        }

        /* access modifiers changed from: protected */
        public /* synthetic */ MetaClass $getStaticMetaClass() {
            if (getClass() != _closure1.class) {
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
            return $getCallSiteArray()[0].call(it);
        }
    }

    public String toString() {
        $getCallSiteArray();
        return ShortTypeHandling.castToString(new GStringImpl(new Object[]{this.name, this.version, this.dependencies}, new String[]{"", " ", " ", ""}));
    }

    public Set<String> getTransitiveDependencies(Collection<String> dependencies2) {
        CallSite[] $getCallSiteArray = $getCallSiteArray();
        Reference result = new Reference((Set) ScriptBytecodeAdapter.castToType(ScriptBytecodeAdapter.createList(new Object[0]), Set.class));
        $getCallSiteArray[9].call(dependencies2, new _getTransitiveDependencies_closure2(this, this, result));
        return (Set) result.get();
    }

    /* compiled from: CatkinPlugin.groovy */
    public class _getTransitiveDependencies_closure2 extends Closure implements GeneratedClosure {
        private static /* synthetic */ SoftReference $callSiteArray;
        private static /* synthetic */ ClassInfo $staticClassInfo;
        public static transient /* synthetic */ boolean __$stMC;
        private /* synthetic */ Reference result;

        private static /* synthetic */ CallSiteArray $createCallSiteArray() {
            String[] strArr = new String[14];
            $createCallSiteArray_1(strArr);
            return new CallSiteArray(_getTransitiveDependencies_closure2.class, strArr);
        }

        private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
            strArr[0] = "containsKey";
            strArr[1] = "pkgs";
            strArr[2] = "tree";
            strArr[3] = "catkin";
            strArr[4] = "project";
            strArr[5] = "add";
            strArr[6] = "addAll";
            strArr[7] = "getTransitiveDependencies";
            strArr[8] = "dependencies";
            strArr[9] = "getAt";
            strArr[10] = "pkgs";
            strArr[11] = "tree";
            strArr[12] = "catkin";
            strArr[13] = "project";
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
        public _getTransitiveDependencies_closure2(Object obj, Object obj2, Reference reference) {
            super(obj, obj2);
            $getCallSiteArray();
            this.result = reference;
        }

        /* access modifiers changed from: protected */
        public /* synthetic */ MetaClass $getStaticMetaClass() {
            if (getClass() != _getTransitiveDependencies_closure2.class) {
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

        public Set getResult() {
            $getCallSiteArray();
            return (Set) ScriptBytecodeAdapter.castToType(this.result.get(), Set.class);
        }

        public Object doCall(Object it) {
            CallSite[] $getCallSiteArray = $getCallSiteArray();
            if (!DefaultTypeTransformation.booleanUnbox($getCallSiteArray[0].call($getCallSiteArray[1].callGetProperty($getCallSiteArray[2].callGetProperty($getCallSiteArray[3].callGetProperty($getCallSiteArray[4].callGroovyObjectGetProperty(this)))), it))) {
                return null;
            }
            $getCallSiteArray[5].call(this.result.get(), it);
            return $getCallSiteArray[6].call(this.result.get(), $getCallSiteArray[7].callCurrent(this, $getCallSiteArray[8].callGetProperty($getCallSiteArray[9].call($getCallSiteArray[10].callGetProperty($getCallSiteArray[11].callGetProperty($getCallSiteArray[12].callGetProperty($getCallSiteArray[13].callGroovyObjectGetProperty(this)))), it))));
        }
    }

    /* compiled from: CatkinPlugin.groovy */
    public class _getMessageDependencies_closure3 extends Closure implements GeneratedClosure {
        private static /* synthetic */ SoftReference $callSiteArray;
        private static /* synthetic */ ClassInfo $staticClassInfo;
        public static transient /* synthetic */ boolean __$stMC;

        private static /* synthetic */ CallSiteArray $createCallSiteArray() {
            String[] strArr = new String[12];
            $createCallSiteArray_1(strArr);
            return new CallSiteArray(_getMessageDependencies_closure3.class, strArr);
        }

        private static /* synthetic */ void $createCallSiteArray_1(String[] strArr) {
            strArr[0] = "containsKey";
            strArr[1] = "pkgs";
            strArr[2] = "tree";
            strArr[3] = "catkin";
            strArr[4] = "project";
            strArr[5] = "contains";
            strArr[6] = "dependencies";
            strArr[7] = "getAt";
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
        public _getMessageDependencies_closure3(Object obj, Object obj2) {
            super(obj, obj2);
            $getCallSiteArray();
        }

        /* access modifiers changed from: protected */
        public /* synthetic */ MetaClass $getStaticMetaClass() {
            if (getClass() != _getMessageDependencies_closure3.class) {
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
            boolean z = false;
            if (DefaultTypeTransformation.booleanUnbox($getCallSiteArray[0].call($getCallSiteArray[1].callGetProperty($getCallSiteArray[2].callGetProperty($getCallSiteArray[3].callGetProperty($getCallSiteArray[4].callGroovyObjectGetProperty(this)))), it)) && DefaultTypeTransformation.booleanUnbox($getCallSiteArray[5].call($getCallSiteArray[6].callGetProperty($getCallSiteArray[7].call($getCallSiteArray[8].callGetProperty($getCallSiteArray[9].callGetProperty($getCallSiteArray[10].callGetProperty($getCallSiteArray[11].callGroovyObjectGetProperty(this)))), it)), "message_generation"))) {
                z = true;
            }
            return Boolean.valueOf(z);
        }
    }

    public Set<String> getMessageDependencies() {
        CallSite[] $getCallSiteArray = $getCallSiteArray();
        return (Set) ScriptBytecodeAdapter.asType($getCallSiteArray[10].call($getCallSiteArray[11].callCurrent(this, this.dependencies), new _getMessageDependencies_closure3(this, this)), Set.class);
    }
}
