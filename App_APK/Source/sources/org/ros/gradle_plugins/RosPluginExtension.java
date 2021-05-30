package org.ros.gradle_plugins;

import groovy.lang.GroovyObject;
import groovy.lang.MetaClass;
import java.lang.ref.SoftReference;
import java.util.List;
import org.codehaus.groovy.reflection.ClassInfo;
import org.codehaus.groovy.runtime.ScriptBytecodeAdapter;
import org.codehaus.groovy.runtime.callsite.CallSite;
import org.codehaus.groovy.runtime.callsite.CallSiteArray;
import org.codehaus.groovy.runtime.typehandling.ShortTypeHandling;

/* compiled from: RosPlugin.groovy */
public class RosPluginExtension implements GroovyObject {
    private static /* synthetic */ SoftReference $callSiteArray;
    private static /* synthetic */ ClassInfo $staticClassInfo;
    public static transient /* synthetic */ boolean __$stMC;
    private String mavenDeploymentRepository = ShortTypeHandling.castToString("");
    private List<String> mavenPath;
    private String mavenRepository = ShortTypeHandling.castToString("");
    private transient /* synthetic */ MetaClass metaClass = $getStaticMetaClass();

    private static /* synthetic */ CallSiteArray $createCallSiteArray() {
        return new CallSiteArray(RosPluginExtension.class, new String[0]);
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
        if (getClass() != RosPluginExtension.class) {
            return ScriptBytecodeAdapter.initMetaClass(this);
        }
        ClassInfo classInfo = $staticClassInfo;
        if (classInfo == null) {
            classInfo = ClassInfo.getClassInfo(getClass());
            $staticClassInfo = classInfo;
        }
        return classInfo.getMetaClass();
    }

    public String getMavenDeploymentRepository() {
        return this.mavenDeploymentRepository;
    }

    public List<String> getMavenPath() {
        return this.mavenPath;
    }

    public String getMavenRepository() {
        return this.mavenRepository;
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

    public void setMavenDeploymentRepository(String str) {
        this.mavenDeploymentRepository = str;
    }

    public void setMavenPath(List<String> list) {
        this.mavenPath = list;
    }

    public void setMavenRepository(String str) {
        this.mavenRepository = str;
    }

    public /* synthetic */ void setMetaClass(MetaClass metaClass2) {
        this.metaClass = metaClass2;
    }

    public /* synthetic */ void setProperty(String str, Object obj) {
        getMetaClass().setProperty(this, str, obj);
    }

    public RosPluginExtension() {
        $getCallSiteArray();
    }
}
