package org.ros.gradle_plugins;

import groovy.lang.GroovyObject;
import groovy.lang.MetaClass;
import java.lang.ref.SoftReference;
import java.util.List;
import org.codehaus.groovy.reflection.ClassInfo;
import org.codehaus.groovy.runtime.ScriptBytecodeAdapter;
import org.codehaus.groovy.runtime.callsite.CallSite;
import org.codehaus.groovy.runtime.callsite.CallSiteArray;

/* compiled from: CatkinPlugin.groovy */
public class CatkinPluginExtension implements GroovyObject {
    private static /* synthetic */ SoftReference $callSiteArray;
    private static /* synthetic */ ClassInfo $staticClassInfo;
    public static transient /* synthetic */ boolean __$stMC;
    private transient /* synthetic */ MetaClass metaClass = $getStaticMetaClass();
    private CatkinPackage pkg;
    private CatkinPackages tree;
    private List<String> workspaces;

    private static /* synthetic */ CallSiteArray $createCallSiteArray() {
        return new CallSiteArray(CatkinPluginExtension.class, new String[0]);
    }

    private static /* synthetic */ CallSite[] $getCallSiteArray() {
        CallSiteArray callSiteArray;
        if ($callSiteArray == null || (callSiteArray = (CallSiteArray) $callSiteArray.get()) == null) {
            callSiteArray = $createCallSiteArray();
            $callSiteArray = new SoftReference(callSiteArray);
        }
        return callSiteArray.array;
    }

    public CatkinPluginExtension() {
        $getCallSiteArray();
    }

    /* access modifiers changed from: protected */
    public /* synthetic */ MetaClass $getStaticMetaClass() {
        if (getClass() != CatkinPluginExtension.class) {
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

    public CatkinPackage getPkg() {
        return this.pkg;
    }

    public /* synthetic */ Object getProperty(String str) {
        return getMetaClass().getProperty(this, str);
    }

    public CatkinPackages getTree() {
        return this.tree;
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

    public void setPkg(CatkinPackage catkinPackage) {
        this.pkg = catkinPackage;
    }

    public /* synthetic */ void setProperty(String str, Object obj) {
        getMetaClass().setProperty(this, str, obj);
    }

    public void setTree(CatkinPackages catkinPackages) {
        this.tree = catkinPackages;
    }

    public void setWorkspaces(List<String> list) {
        this.workspaces = list;
    }
}
