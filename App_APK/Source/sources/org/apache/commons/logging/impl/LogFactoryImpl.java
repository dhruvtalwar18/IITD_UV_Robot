package org.apache.commons.logging.impl;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.net.URL;
import java.security.AccessController;
import java.security.PrivilegedAction;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.Vector;
import org.apache.commons.io.IOUtils;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogConfigurationException;
import org.apache.commons.logging.LogFactory;

public class LogFactoryImpl extends LogFactory {
    public static final String ALLOW_FLAWED_CONTEXT_PROPERTY = "org.apache.commons.logging.Log.allowFlawedContext";
    public static final String ALLOW_FLAWED_DISCOVERY_PROPERTY = "org.apache.commons.logging.Log.allowFlawedDiscovery";
    public static final String ALLOW_FLAWED_HIERARCHY_PROPERTY = "org.apache.commons.logging.Log.allowFlawedHierarchy";
    private static final String LOGGING_IMPL_JDK14_LOGGER = "org.apache.commons.logging.impl.Jdk14Logger";
    private static final String LOGGING_IMPL_LOG4J_LOGGER = "org.apache.commons.logging.impl.Log4JLogger";
    private static final String LOGGING_IMPL_LUMBERJACK_LOGGER = "org.apache.commons.logging.impl.Jdk13LumberjackLogger";
    private static final String LOGGING_IMPL_SIMPLE_LOGGER = "org.apache.commons.logging.impl.SimpleLog";
    public static final String LOG_PROPERTY = "org.apache.commons.logging.Log";
    protected static final String LOG_PROPERTY_OLD = "org.apache.commons.logging.log";
    private static final String PKG_IMPL = "org.apache.commons.logging.impl.";
    private static final int PKG_LEN = PKG_IMPL.length();
    static /* synthetic */ Class class$java$lang$String;
    static /* synthetic */ Class class$org$apache$commons$logging$Log;
    static /* synthetic */ Class class$org$apache$commons$logging$LogFactory;
    static /* synthetic */ Class class$org$apache$commons$logging$impl$LogFactoryImpl;
    private static final String[] classesToDiscover = {LOGGING_IMPL_LOG4J_LOGGER, LOGGING_IMPL_JDK14_LOGGER, LOGGING_IMPL_LUMBERJACK_LOGGER, LOGGING_IMPL_SIMPLE_LOGGER};
    private boolean allowFlawedContext;
    private boolean allowFlawedDiscovery;
    private boolean allowFlawedHierarchy;
    protected Hashtable attributes = new Hashtable();
    private String diagnosticPrefix;
    protected Hashtable instances = new Hashtable();
    private String logClassName;
    protected Constructor logConstructor = null;
    protected Class[] logConstructorSignature;
    protected Method logMethod;
    protected Class[] logMethodSignature;
    private boolean useTCCL = true;

    public LogFactoryImpl() {
        Class cls;
        Class cls2;
        Class[] clsArr = new Class[1];
        if (class$java$lang$String == null) {
            cls = class$("java.lang.String");
            class$java$lang$String = cls;
        } else {
            cls = class$java$lang$String;
        }
        clsArr[0] = cls;
        this.logConstructorSignature = clsArr;
        this.logMethod = null;
        Class[] clsArr2 = new Class[1];
        if (class$org$apache$commons$logging$LogFactory == null) {
            cls2 = class$(LogFactory.FACTORY_PROPERTY);
            class$org$apache$commons$logging$LogFactory = cls2;
        } else {
            cls2 = class$org$apache$commons$logging$LogFactory;
        }
        clsArr2[0] = cls2;
        this.logMethodSignature = clsArr2;
        initDiagnostics();
        if (isDiagnosticsEnabled()) {
            logDiagnostic("Instance created.");
        }
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError(x1.getMessage());
        }
    }

    public Object getAttribute(String name) {
        return this.attributes.get(name);
    }

    public String[] getAttributeNames() {
        Vector names = new Vector();
        Enumeration keys = this.attributes.keys();
        while (keys.hasMoreElements()) {
            names.addElement((String) keys.nextElement());
        }
        String[] results = new String[names.size()];
        for (int i = 0; i < results.length; i++) {
            results[i] = (String) names.elementAt(i);
        }
        return results;
    }

    public Log getInstance(Class clazz) throws LogConfigurationException {
        return getInstance(clazz.getName());
    }

    public Log getInstance(String name) throws LogConfigurationException {
        Log instance = (Log) this.instances.get(name);
        if (instance != null) {
            return instance;
        }
        Log instance2 = newInstance(name);
        this.instances.put(name, instance2);
        return instance2;
    }

    public void release() {
        logDiagnostic("Releasing all known loggers");
        this.instances.clear();
    }

    public void removeAttribute(String name) {
        this.attributes.remove(name);
    }

    public void setAttribute(String name, Object value) {
        if (this.logConstructor != null) {
            logDiagnostic("setAttribute: call too late; configuration already performed.");
        }
        if (value == null) {
            this.attributes.remove(name);
        } else {
            this.attributes.put(name, value);
        }
        if (name.equals(LogFactory.TCCL_KEY)) {
            this.useTCCL = Boolean.valueOf(value.toString()).booleanValue();
        }
    }

    protected static ClassLoader getContextClassLoader() throws LogConfigurationException {
        return LogFactory.getContextClassLoader();
    }

    protected static boolean isDiagnosticsEnabled() {
        return LogFactory.isDiagnosticsEnabled();
    }

    protected static ClassLoader getClassLoader(Class clazz) {
        return LogFactory.getClassLoader(clazz);
    }

    private void initDiagnostics() {
        String classLoaderName;
        ClassLoader classLoader = getClassLoader(getClass());
        if (classLoader == null) {
            classLoaderName = "BOOTLOADER";
        } else {
            try {
                classLoaderName = LogFactory.objectId(classLoader);
            } catch (SecurityException e) {
                classLoaderName = "UNKNOWN";
            }
        }
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("[LogFactoryImpl@");
        stringBuffer.append(System.identityHashCode(this));
        stringBuffer.append(" from ");
        stringBuffer.append(classLoaderName);
        stringBuffer.append("] ");
        this.diagnosticPrefix = stringBuffer.toString();
    }

    /* access modifiers changed from: protected */
    public void logDiagnostic(String msg) {
        if (isDiagnosticsEnabled()) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append(this.diagnosticPrefix);
            stringBuffer.append(msg);
            LogFactory.logRawDiagnostic(stringBuffer.toString());
        }
    }

    /* access modifiers changed from: protected */
    public String getLogClassName() {
        if (this.logClassName == null) {
            discoverLogImplementation(getClass().getName());
        }
        return this.logClassName;
    }

    /* access modifiers changed from: protected */
    public Constructor getLogConstructor() throws LogConfigurationException {
        if (this.logConstructor == null) {
            discoverLogImplementation(getClass().getName());
        }
        return this.logConstructor;
    }

    /* access modifiers changed from: protected */
    public boolean isJdk13LumberjackAvailable() {
        return isLogLibraryAvailable("Jdk13Lumberjack", LOGGING_IMPL_LUMBERJACK_LOGGER);
    }

    /* access modifiers changed from: protected */
    public boolean isJdk14Available() {
        return isLogLibraryAvailable("Jdk14", LOGGING_IMPL_JDK14_LOGGER);
    }

    /* access modifiers changed from: protected */
    public boolean isLog4JAvailable() {
        return isLogLibraryAvailable("Log4J", LOGGING_IMPL_LOG4J_LOGGER);
    }

    /* access modifiers changed from: protected */
    public Log newInstance(String name) throws LogConfigurationException {
        Log instance;
        try {
            if (this.logConstructor == null) {
                instance = discoverLogImplementation(name);
            } else {
                instance = (Log) this.logConstructor.newInstance(new Object[]{name});
            }
            if (this.logMethod != null) {
                this.logMethod.invoke(instance, new Object[]{this});
            }
            return instance;
        } catch (LogConfigurationException lce) {
            throw lce;
        } catch (InvocationTargetException e) {
            Throwable c = e.getTargetException();
            if (c != null) {
                throw new LogConfigurationException(c);
            }
            throw new LogConfigurationException((Throwable) e);
        } catch (Throwable t) {
            throw new LogConfigurationException(t);
        }
    }

    private static ClassLoader getContextClassLoaderInternal() throws LogConfigurationException {
        return (ClassLoader) AccessController.doPrivileged(new PrivilegedAction() {
            public Object run() {
                return LogFactory.directGetContextClassLoader();
            }
        });
    }

    private static String getSystemProperty(final String key, final String def) throws SecurityException {
        return (String) AccessController.doPrivileged(new PrivilegedAction() {
            public Object run() {
                return System.getProperty(key, def);
            }
        });
    }

    private ClassLoader getParentClassLoader(final ClassLoader cl) {
        try {
            return (ClassLoader) AccessController.doPrivileged(new PrivilegedAction() {
                public Object run() {
                    return cl.getParent();
                }
            });
        } catch (SecurityException e) {
            logDiagnostic("[SECURITY] Unable to obtain parent classloader");
            return null;
        }
    }

    private boolean isLogLibraryAvailable(String name, String classname) {
        if (isDiagnosticsEnabled()) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Checking for '");
            stringBuffer.append(name);
            stringBuffer.append("'.");
            logDiagnostic(stringBuffer.toString());
        }
        try {
            if (createLogFromClass(classname, getClass().getName(), false) == null) {
                if (isDiagnosticsEnabled()) {
                    StringBuffer stringBuffer2 = new StringBuffer();
                    stringBuffer2.append("Did not find '");
                    stringBuffer2.append(name);
                    stringBuffer2.append("'.");
                    logDiagnostic(stringBuffer2.toString());
                }
                return false;
            } else if (!isDiagnosticsEnabled()) {
                return true;
            } else {
                StringBuffer stringBuffer3 = new StringBuffer();
                stringBuffer3.append("Found '");
                stringBuffer3.append(name);
                stringBuffer3.append("'.");
                logDiagnostic(stringBuffer3.toString());
                return true;
            }
        } catch (LogConfigurationException e) {
            if (isDiagnosticsEnabled()) {
                StringBuffer stringBuffer4 = new StringBuffer();
                stringBuffer4.append("Logging system '");
                stringBuffer4.append(name);
                stringBuffer4.append("' is available but not useable.");
                logDiagnostic(stringBuffer4.toString());
            }
            return false;
        }
    }

    private String getConfigurationValue(String property) {
        if (isDiagnosticsEnabled()) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("[ENV] Trying to get configuration for item ");
            stringBuffer.append(property);
            logDiagnostic(stringBuffer.toString());
        }
        Object valueObj = getAttribute(property);
        if (valueObj != null) {
            if (isDiagnosticsEnabled()) {
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append("[ENV] Found LogFactory attribute [");
                stringBuffer2.append(valueObj);
                stringBuffer2.append("] for ");
                stringBuffer2.append(property);
                logDiagnostic(stringBuffer2.toString());
            }
            return valueObj.toString();
        }
        if (isDiagnosticsEnabled()) {
            StringBuffer stringBuffer3 = new StringBuffer();
            stringBuffer3.append("[ENV] No LogFactory attribute found for ");
            stringBuffer3.append(property);
            logDiagnostic(stringBuffer3.toString());
        }
        try {
            String value = getSystemProperty(property, (String) null);
            if (value != null) {
                if (isDiagnosticsEnabled()) {
                    StringBuffer stringBuffer4 = new StringBuffer();
                    stringBuffer4.append("[ENV] Found system property [");
                    stringBuffer4.append(value);
                    stringBuffer4.append("] for ");
                    stringBuffer4.append(property);
                    logDiagnostic(stringBuffer4.toString());
                }
                return value;
            }
            if (isDiagnosticsEnabled()) {
                StringBuffer stringBuffer5 = new StringBuffer();
                stringBuffer5.append("[ENV] No system property found for property ");
                stringBuffer5.append(property);
                logDiagnostic(stringBuffer5.toString());
            }
            if (isDiagnosticsEnabled()) {
                StringBuffer stringBuffer6 = new StringBuffer();
                stringBuffer6.append("[ENV] No configuration defined for item ");
                stringBuffer6.append(property);
                logDiagnostic(stringBuffer6.toString());
            }
            return null;
        } catch (SecurityException e) {
            if (isDiagnosticsEnabled()) {
                StringBuffer stringBuffer7 = new StringBuffer();
                stringBuffer7.append("[ENV] Security prevented reading system property ");
                stringBuffer7.append(property);
                logDiagnostic(stringBuffer7.toString());
            }
        }
    }

    private boolean getBooleanConfiguration(String key, boolean dflt) {
        String val = getConfigurationValue(key);
        if (val == null) {
            return dflt;
        }
        return Boolean.valueOf(val).booleanValue();
    }

    private void initConfiguration() {
        this.allowFlawedContext = getBooleanConfiguration(ALLOW_FLAWED_CONTEXT_PROPERTY, true);
        this.allowFlawedDiscovery = getBooleanConfiguration(ALLOW_FLAWED_DISCOVERY_PROPERTY, true);
        this.allowFlawedHierarchy = getBooleanConfiguration(ALLOW_FLAWED_HIERARCHY_PROPERTY, true);
    }

    private Log discoverLogImplementation(String logCategory) throws LogConfigurationException {
        if (isDiagnosticsEnabled()) {
            logDiagnostic("Discovering a Log implementation...");
        }
        initConfiguration();
        Log result = null;
        String specifiedLogClassName = findUserSpecifiedLogClassName();
        if (specifiedLogClassName != null) {
            if (isDiagnosticsEnabled()) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Attempting to load user-specified log class '");
                stringBuffer.append(specifiedLogClassName);
                stringBuffer.append("'...");
                logDiagnostic(stringBuffer.toString());
            }
            Log result2 = createLogFromClass(specifiedLogClassName, logCategory, true);
            if (result2 != null) {
                return result2;
            }
            StringBuffer messageBuffer = new StringBuffer("User-specified log class '");
            messageBuffer.append(specifiedLogClassName);
            messageBuffer.append("' cannot be found or is not useable.");
            if (specifiedLogClassName != null) {
                informUponSimilarName(messageBuffer, specifiedLogClassName, LOGGING_IMPL_LOG4J_LOGGER);
                informUponSimilarName(messageBuffer, specifiedLogClassName, LOGGING_IMPL_JDK14_LOGGER);
                informUponSimilarName(messageBuffer, specifiedLogClassName, LOGGING_IMPL_LUMBERJACK_LOGGER);
                informUponSimilarName(messageBuffer, specifiedLogClassName, LOGGING_IMPL_SIMPLE_LOGGER);
            }
            throw new LogConfigurationException(messageBuffer.toString());
        }
        if (isDiagnosticsEnabled()) {
            logDiagnostic("No user-specified Log implementation; performing discovery using the standard supported logging implementations...");
        }
        for (int i = 0; i < classesToDiscover.length && result == null; i++) {
            result = createLogFromClass(classesToDiscover[i], logCategory, true);
        }
        if (result != null) {
            return result;
        }
        throw new LogConfigurationException("No suitable Log implementation");
    }

    private void informUponSimilarName(StringBuffer messageBuffer, String name, String candidate) {
        if (!name.equals(candidate)) {
            if (name.regionMatches(true, 0, candidate, 0, PKG_LEN + 5)) {
                messageBuffer.append(" Did you mean '");
                messageBuffer.append(candidate);
                messageBuffer.append("'?");
            }
        }
    }

    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r1v5, resolved type: java.lang.Object} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r0v10, resolved type: java.lang.String} */
    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private java.lang.String findUserSpecifiedLogClassName() {
        /*
            r5 = this;
            boolean r0 = isDiagnosticsEnabled()
            if (r0 == 0) goto L_0x000b
            java.lang.String r0 = "Trying to get log class from attribute 'org.apache.commons.logging.Log'"
            r5.logDiagnostic(r0)
        L_0x000b:
            java.lang.String r0 = "org.apache.commons.logging.Log"
            java.lang.Object r0 = r5.getAttribute(r0)
            java.lang.String r0 = (java.lang.String) r0
            if (r0 != 0) goto L_0x0029
            boolean r1 = isDiagnosticsEnabled()
            if (r1 == 0) goto L_0x0020
            java.lang.String r1 = "Trying to get log class from attribute 'org.apache.commons.logging.log'"
            r5.logDiagnostic(r1)
        L_0x0020:
            java.lang.String r1 = "org.apache.commons.logging.log"
            java.lang.Object r1 = r5.getAttribute(r1)
            r0 = r1
            java.lang.String r0 = (java.lang.String) r0
        L_0x0029:
            r1 = 0
            if (r0 != 0) goto L_0x005e
            boolean r2 = isDiagnosticsEnabled()
            if (r2 == 0) goto L_0x0037
            java.lang.String r2 = "Trying to get log class from system property 'org.apache.commons.logging.Log'"
            r5.logDiagnostic(r2)
        L_0x0037:
            java.lang.String r2 = "org.apache.commons.logging.Log"
            java.lang.String r2 = getSystemProperty(r2, r1)     // Catch:{ SecurityException -> 0x003f }
            r0 = r2
            goto L_0x005e
        L_0x003f:
            r2 = move-exception
            boolean r3 = isDiagnosticsEnabled()
            if (r3 == 0) goto L_0x005e
            java.lang.StringBuffer r3 = new java.lang.StringBuffer
            r3.<init>()
            java.lang.String r4 = "No access allowed to system property 'org.apache.commons.logging.Log' - "
            r3.append(r4)
            java.lang.String r4 = r2.getMessage()
            r3.append(r4)
            java.lang.String r3 = r3.toString()
            r5.logDiagnostic(r3)
        L_0x005e:
            if (r0 != 0) goto L_0x0092
            boolean r2 = isDiagnosticsEnabled()
            if (r2 == 0) goto L_0x006b
            java.lang.String r2 = "Trying to get log class from system property 'org.apache.commons.logging.log'"
            r5.logDiagnostic(r2)
        L_0x006b:
            java.lang.String r2 = "org.apache.commons.logging.log"
            java.lang.String r1 = getSystemProperty(r2, r1)     // Catch:{ SecurityException -> 0x0073 }
            r0 = r1
            goto L_0x0092
        L_0x0073:
            r1 = move-exception
            boolean r2 = isDiagnosticsEnabled()
            if (r2 == 0) goto L_0x0092
            java.lang.StringBuffer r2 = new java.lang.StringBuffer
            r2.<init>()
            java.lang.String r3 = "No access allowed to system property 'org.apache.commons.logging.log' - "
            r2.append(r3)
            java.lang.String r3 = r1.getMessage()
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r5.logDiagnostic(r2)
        L_0x0092:
            if (r0 == 0) goto L_0x0098
            java.lang.String r0 = r0.trim()
        L_0x0098:
            return r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.logging.impl.LogFactoryImpl.findUserSpecifiedLogClassName():java.lang.String");
    }

    private Log createLogFromClass(String logAdapterClassName, String logCategory, boolean affectState) throws LogConfigurationException {
        Class c;
        URL url;
        String str = logAdapterClassName;
        if (isDiagnosticsEnabled()) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Attempting to instantiate '");
            stringBuffer.append(str);
            stringBuffer.append("'");
            logDiagnostic(stringBuffer.toString());
        }
        Object[] params = {logCategory};
        Log logAdapter = null;
        ClassLoader currentCL = getBaseClassLoader();
        Class logAdapterClass = null;
        Constructor constructor = null;
        while (true) {
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append("Trying to load '");
            stringBuffer2.append(str);
            stringBuffer2.append("' from classloader ");
            stringBuffer2.append(LogFactory.objectId(currentCL));
            logDiagnostic(stringBuffer2.toString());
            try {
                if (isDiagnosticsEnabled()) {
                    StringBuffer stringBuffer3 = new StringBuffer();
                    stringBuffer3.append(str.replace('.', IOUtils.DIR_SEPARATOR_UNIX));
                    stringBuffer3.append(".class");
                    String resourceName = stringBuffer3.toString();
                    if (currentCL != null) {
                        url = currentCL.getResource(resourceName);
                    } else {
                        StringBuffer stringBuffer4 = new StringBuffer();
                        stringBuffer4.append(resourceName);
                        stringBuffer4.append(".class");
                        url = ClassLoader.getSystemResource(stringBuffer4.toString());
                    }
                    if (url == null) {
                        StringBuffer stringBuffer5 = new StringBuffer();
                        stringBuffer5.append("Class '");
                        stringBuffer5.append(str);
                        stringBuffer5.append("' [");
                        stringBuffer5.append(resourceName);
                        stringBuffer5.append("] cannot be found.");
                        logDiagnostic(stringBuffer5.toString());
                    } else {
                        StringBuffer stringBuffer6 = new StringBuffer();
                        stringBuffer6.append("Class '");
                        stringBuffer6.append(str);
                        stringBuffer6.append("' was found at '");
                        stringBuffer6.append(url);
                        stringBuffer6.append("'");
                        logDiagnostic(stringBuffer6.toString());
                    }
                }
                try {
                    c = Class.forName(str, true, currentCL);
                } catch (ClassNotFoundException e) {
                    ClassNotFoundException originalClassNotFoundException = e;
                    StringBuffer stringBuffer7 = new StringBuffer();
                    stringBuffer7.append("");
                    stringBuffer7.append(originalClassNotFoundException.getMessage());
                    String msg = stringBuffer7.toString();
                    StringBuffer stringBuffer8 = new StringBuffer();
                    stringBuffer8.append("The log adapter '");
                    stringBuffer8.append(str);
                    stringBuffer8.append("' is not available via classloader ");
                    stringBuffer8.append(LogFactory.objectId(currentCL));
                    stringBuffer8.append(": ");
                    stringBuffer8.append(msg.trim());
                    logDiagnostic(stringBuffer8.toString());
                    try {
                        c = Class.forName(logAdapterClassName);
                    } catch (ClassNotFoundException e2) {
                        StringBuffer stringBuffer9 = new StringBuffer();
                        stringBuffer9.append("");
                        stringBuffer9.append(e2.getMessage());
                        String msg2 = stringBuffer9.toString();
                        StringBuffer stringBuffer10 = new StringBuffer();
                        stringBuffer10.append("The log adapter '");
                        stringBuffer10.append(str);
                        stringBuffer10.append("' is not available via the LogFactoryImpl class classloader: ");
                        stringBuffer10.append(msg2.trim());
                        logDiagnostic(stringBuffer10.toString());
                    }
                }
                constructor = c.getConstructor(this.logConstructorSignature);
                Object newInstance = constructor.newInstance(params);
                if (newInstance instanceof Log) {
                    logAdapterClass = c;
                    logAdapter = newInstance;
                    break;
                }
                handleFlawedHierarchy(currentCL, c);
                if (currentCL == null) {
                    break;
                }
                currentCL = getParentClassLoader(currentCL);
            } catch (NoClassDefFoundError e3) {
                StringBuffer stringBuffer11 = new StringBuffer();
                stringBuffer11.append("");
                stringBuffer11.append(e3.getMessage());
                String msg3 = stringBuffer11.toString();
                StringBuffer stringBuffer12 = new StringBuffer();
                stringBuffer12.append("The log adapter '");
                stringBuffer12.append(str);
                stringBuffer12.append("' is missing dependencies when loaded via classloader ");
                stringBuffer12.append(LogFactory.objectId(currentCL));
                stringBuffer12.append(": ");
                stringBuffer12.append(msg3.trim());
                logDiagnostic(stringBuffer12.toString());
            } catch (ExceptionInInitializerError e4) {
                StringBuffer stringBuffer13 = new StringBuffer();
                stringBuffer13.append("");
                stringBuffer13.append(e4.getMessage());
                String msg4 = stringBuffer13.toString();
                StringBuffer stringBuffer14 = new StringBuffer();
                stringBuffer14.append("The log adapter '");
                stringBuffer14.append(str);
                stringBuffer14.append("' is unable to initialize itself when loaded via classloader ");
                stringBuffer14.append(LogFactory.objectId(currentCL));
                stringBuffer14.append(": ");
                stringBuffer14.append(msg4.trim());
                logDiagnostic(stringBuffer14.toString());
            } catch (LogConfigurationException e5) {
                throw e5;
            } catch (Throwable t) {
                handleFlawedDiscovery(str, currentCL, t);
            }
        }
        if (logAdapter != null && affectState) {
            this.logClassName = str;
            this.logConstructor = constructor;
            try {
                this.logMethod = logAdapterClass.getMethod("setLogFactory", this.logMethodSignature);
                StringBuffer stringBuffer15 = new StringBuffer();
                stringBuffer15.append("Found method setLogFactory(LogFactory) in '");
                stringBuffer15.append(str);
                stringBuffer15.append("'");
                logDiagnostic(stringBuffer15.toString());
            } catch (Throwable th) {
                this.logMethod = null;
                StringBuffer stringBuffer16 = new StringBuffer();
                stringBuffer16.append("[INFO] '");
                stringBuffer16.append(str);
                stringBuffer16.append("' from classloader ");
                stringBuffer16.append(LogFactory.objectId(currentCL));
                stringBuffer16.append(" does not declare optional method ");
                stringBuffer16.append("setLogFactory(LogFactory)");
                logDiagnostic(stringBuffer16.toString());
            }
            StringBuffer stringBuffer17 = new StringBuffer();
            stringBuffer17.append("Log adapter '");
            stringBuffer17.append(str);
            stringBuffer17.append("' from classloader ");
            stringBuffer17.append(LogFactory.objectId(logAdapterClass.getClassLoader()));
            stringBuffer17.append(" has been selected for use.");
            logDiagnostic(stringBuffer17.toString());
        }
        return logAdapter;
    }

    private ClassLoader getBaseClassLoader() throws LogConfigurationException {
        Class cls;
        if (class$org$apache$commons$logging$impl$LogFactoryImpl == null) {
            cls = class$(LogFactory.FACTORY_DEFAULT);
            class$org$apache$commons$logging$impl$LogFactoryImpl = cls;
        } else {
            cls = class$org$apache$commons$logging$impl$LogFactoryImpl;
        }
        ClassLoader thisClassLoader = getClassLoader(cls);
        if (!this.useTCCL) {
            return thisClassLoader;
        }
        ClassLoader contextClassLoader = getContextClassLoaderInternal();
        ClassLoader baseClassLoader = getLowestClassLoader(contextClassLoader, thisClassLoader);
        if (baseClassLoader != null) {
            if (baseClassLoader != contextClassLoader) {
                if (!this.allowFlawedContext) {
                    throw new LogConfigurationException("Bad classloader hierarchy; LogFactoryImpl was loaded via a classloader that is not related to the current context classloader.");
                } else if (isDiagnosticsEnabled()) {
                    logDiagnostic("Warning: the context classloader is an ancestor of the classloader that loaded LogFactoryImpl; it should be the same or a descendant. The application using commons-logging should ensure the context classloader is used correctly.");
                }
            }
            return baseClassLoader;
        } else if (this.allowFlawedContext) {
            if (isDiagnosticsEnabled()) {
                logDiagnostic("[WARNING] the context classloader is not part of a parent-child relationship with the classloader that loaded LogFactoryImpl.");
            }
            return contextClassLoader;
        } else {
            throw new LogConfigurationException("Bad classloader hierarchy; LogFactoryImpl was loaded via a classloader that is not related to the current context classloader.");
        }
    }

    private ClassLoader getLowestClassLoader(ClassLoader c1, ClassLoader c2) {
        if (c1 == null) {
            return c2;
        }
        if (c2 == null) {
            return c1;
        }
        for (ClassLoader current = c1; current != null; current = current.getParent()) {
            if (current == c2) {
                return c1;
            }
        }
        for (ClassLoader current2 = c2; current2 != null; current2 = current2.getParent()) {
            if (current2 == c1) {
                return c2;
            }
        }
        return null;
    }

    private void handleFlawedDiscovery(String logAdapterClassName, ClassLoader classLoader, Throwable discoveryFlaw) {
        Throwable cause;
        Throwable cause2;
        if (isDiagnosticsEnabled()) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Could not instantiate Log '");
            stringBuffer.append(logAdapterClassName);
            stringBuffer.append("' -- ");
            stringBuffer.append(discoveryFlaw.getClass().getName());
            stringBuffer.append(": ");
            stringBuffer.append(discoveryFlaw.getLocalizedMessage());
            logDiagnostic(stringBuffer.toString());
            if ((discoveryFlaw instanceof InvocationTargetException) && (cause = ((InvocationTargetException) discoveryFlaw).getTargetException()) != null) {
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append("... InvocationTargetException: ");
                stringBuffer2.append(cause.getClass().getName());
                stringBuffer2.append(": ");
                stringBuffer2.append(cause.getLocalizedMessage());
                logDiagnostic(stringBuffer2.toString());
                if ((cause instanceof ExceptionInInitializerError) && (cause2 = ((ExceptionInInitializerError) cause).getException()) != null) {
                    StringBuffer stringBuffer3 = new StringBuffer();
                    stringBuffer3.append("... ExceptionInInitializerError: ");
                    stringBuffer3.append(cause2.getClass().getName());
                    stringBuffer3.append(": ");
                    stringBuffer3.append(cause2.getLocalizedMessage());
                    logDiagnostic(stringBuffer3.toString());
                }
            }
        }
        if (!this.allowFlawedDiscovery) {
            throw new LogConfigurationException(discoveryFlaw);
        }
    }

    private void handleFlawedHierarchy(ClassLoader badClassLoader, Class badClass) throws LogConfigurationException {
        Class cls;
        Class cls2;
        Class cls3;
        Class cls4;
        boolean implementsLog = false;
        if (class$org$apache$commons$logging$Log == null) {
            cls = class$(LOG_PROPERTY);
            class$org$apache$commons$logging$Log = cls;
        } else {
            cls = class$org$apache$commons$logging$Log;
        }
        String logInterfaceName = cls.getName();
        Class[] interfaces = badClass.getInterfaces();
        int i = 0;
        while (true) {
            if (i >= interfaces.length) {
                break;
            } else if (logInterfaceName.equals(interfaces[i].getName())) {
                implementsLog = true;
                break;
            } else {
                i++;
            }
        }
        if (implementsLog) {
            if (isDiagnosticsEnabled()) {
                try {
                    if (class$org$apache$commons$logging$Log == null) {
                        cls4 = class$(LOG_PROPERTY);
                        class$org$apache$commons$logging$Log = cls4;
                    } else {
                        cls4 = class$org$apache$commons$logging$Log;
                    }
                    ClassLoader logInterfaceClassLoader = getClassLoader(cls4);
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("Class '");
                    stringBuffer.append(badClass.getName());
                    stringBuffer.append("' was found in classloader ");
                    stringBuffer.append(LogFactory.objectId(badClassLoader));
                    stringBuffer.append(". It is bound to a Log interface which is not");
                    stringBuffer.append(" the one loaded from classloader ");
                    stringBuffer.append(LogFactory.objectId(logInterfaceClassLoader));
                    logDiagnostic(stringBuffer.toString());
                } catch (Throwable th) {
                    StringBuffer stringBuffer2 = new StringBuffer();
                    stringBuffer2.append("Error while trying to output diagnostics about bad class '");
                    stringBuffer2.append(badClass);
                    stringBuffer2.append("'");
                    logDiagnostic(stringBuffer2.toString());
                }
            }
            if (!this.allowFlawedHierarchy) {
                StringBuffer msg = new StringBuffer();
                msg.append("Terminating logging for this context ");
                msg.append("due to bad log hierarchy. ");
                msg.append("You have more than one version of '");
                if (class$org$apache$commons$logging$Log == null) {
                    cls3 = class$(LOG_PROPERTY);
                    class$org$apache$commons$logging$Log = cls3;
                } else {
                    cls3 = class$org$apache$commons$logging$Log;
                }
                msg.append(cls3.getName());
                msg.append("' visible.");
                if (isDiagnosticsEnabled()) {
                    logDiagnostic(msg.toString());
                }
                throw new LogConfigurationException(msg.toString());
            } else if (isDiagnosticsEnabled()) {
                StringBuffer msg2 = new StringBuffer();
                msg2.append("Warning: bad log hierarchy. ");
                msg2.append("You have more than one version of '");
                if (class$org$apache$commons$logging$Log == null) {
                    cls2 = class$(LOG_PROPERTY);
                    class$org$apache$commons$logging$Log = cls2;
                } else {
                    cls2 = class$org$apache$commons$logging$Log;
                }
                msg2.append(cls2.getName());
                msg2.append("' visible.");
                logDiagnostic(msg2.toString());
            }
        } else if (!this.allowFlawedDiscovery) {
            StringBuffer msg3 = new StringBuffer();
            msg3.append("Terminating logging for this context. ");
            msg3.append("Log class '");
            msg3.append(badClass.getName());
            msg3.append("' does not implement the Log interface.");
            if (isDiagnosticsEnabled()) {
                logDiagnostic(msg3.toString());
            }
            throw new LogConfigurationException(msg3.toString());
        } else if (isDiagnosticsEnabled()) {
            StringBuffer msg4 = new StringBuffer();
            msg4.append("[WARNING] Log class '");
            msg4.append(badClass.getName());
            msg4.append("' does not implement the Log interface.");
            logDiagnostic(msg4.toString());
        }
    }
}
