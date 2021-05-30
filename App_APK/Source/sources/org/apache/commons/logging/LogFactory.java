package org.apache.commons.logging;

import java.io.BufferedReader;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.io.UnsupportedEncodingException;
import java.lang.reflect.InvocationTargetException;
import java.net.URL;
import java.security.AccessController;
import java.security.PrivilegedAction;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.Properties;
import org.bytedeco.javacpp.opencv_stitching;

public abstract class LogFactory {
    public static final String DIAGNOSTICS_DEST_PROPERTY = "org.apache.commons.logging.diagnostics.dest";
    public static final String FACTORY_DEFAULT = "org.apache.commons.logging.impl.LogFactoryImpl";
    public static final String FACTORY_PROPERTIES = "commons-logging.properties";
    public static final String FACTORY_PROPERTY = "org.apache.commons.logging.LogFactory";
    public static final String HASHTABLE_IMPLEMENTATION_PROPERTY = "org.apache.commons.logging.LogFactory.HashtableImpl";
    public static final String PRIORITY_KEY = "priority";
    protected static final String SERVICE_ID = "META-INF/services/org.apache.commons.logging.LogFactory";
    public static final String TCCL_KEY = "use_tccl";
    private static final String WEAK_HASHTABLE_CLASSNAME = "org.apache.commons.logging.impl.WeakHashtable";
    static /* synthetic */ Class class$java$lang$Thread;
    static /* synthetic */ Class class$org$apache$commons$logging$LogFactory;
    private static String diagnosticPrefix;
    private static PrintStream diagnosticsStream = null;
    protected static Hashtable factories;
    protected static LogFactory nullClassLoaderFactory = null;
    private static ClassLoader thisClassLoader;

    public abstract Object getAttribute(String str);

    public abstract String[] getAttributeNames();

    public abstract Log getInstance(Class cls) throws LogConfigurationException;

    public abstract Log getInstance(String str) throws LogConfigurationException;

    public abstract void release();

    public abstract void removeAttribute(String str);

    public abstract void setAttribute(String str, Object obj);

    static {
        Class cls;
        Class cls2;
        factories = null;
        if (class$org$apache$commons$logging$LogFactory == null) {
            cls = class$(FACTORY_PROPERTY);
            class$org$apache$commons$logging$LogFactory = cls;
        } else {
            cls = class$org$apache$commons$logging$LogFactory;
        }
        thisClassLoader = getClassLoader(cls);
        initDiagnostics();
        if (class$org$apache$commons$logging$LogFactory == null) {
            cls2 = class$(FACTORY_PROPERTY);
            class$org$apache$commons$logging$LogFactory = cls2;
        } else {
            cls2 = class$org$apache$commons$logging$LogFactory;
        }
        logClassLoaderEnvironment(cls2);
        factories = createFactoryStore();
        if (isDiagnosticsEnabled()) {
            logDiagnostic("BOOTSTRAP COMPLETED");
        }
    }

    protected LogFactory() {
    }

    private static final Hashtable createFactoryStore() {
        String storeImplementationClass = null;
        Hashtable result = null;
        try {
            storeImplementationClass = getSystemProperty(HASHTABLE_IMPLEMENTATION_PROPERTY, (String) null);
        } catch (SecurityException e) {
        }
        if (storeImplementationClass == null) {
            storeImplementationClass = WEAK_HASHTABLE_CLASSNAME;
        }
        try {
            result = (Hashtable) Class.forName(storeImplementationClass).newInstance();
        } catch (Throwable th) {
            if (!WEAK_HASHTABLE_CLASSNAME.equals(storeImplementationClass)) {
                if (isDiagnosticsEnabled()) {
                    logDiagnostic("[ERROR] LogFactory: Load of custom hashtable failed");
                } else {
                    System.err.println("[ERROR] LogFactory: Load of custom hashtable failed");
                }
            }
        }
        if (result == null) {
            return new Hashtable();
        }
        return result;
    }

    private static String trim(String src) {
        if (src == null) {
            return null;
        }
        return src.trim();
    }

    public static LogFactory getFactory() throws LogConfigurationException {
        BufferedReader rd;
        String useTCCLStr;
        ClassLoader contextClassLoader = getContextClassLoaderInternal();
        if (contextClassLoader == null && isDiagnosticsEnabled()) {
            logDiagnostic("Context classloader is null.");
        }
        LogFactory factory = getCachedFactory(contextClassLoader);
        if (factory != null) {
            return factory;
        }
        if (isDiagnosticsEnabled()) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("[LOOKUP] LogFactory implementation requested for the first time for context classloader ");
            stringBuffer.append(objectId(contextClassLoader));
            logDiagnostic(stringBuffer.toString());
            logHierarchy("[LOOKUP] ", contextClassLoader);
        }
        Properties props = getConfigurationFile(contextClassLoader, FACTORY_PROPERTIES);
        ClassLoader baseClassLoader = contextClassLoader;
        if (!(props == null || (useTCCLStr = props.getProperty(TCCL_KEY)) == null || Boolean.valueOf(useTCCLStr).booleanValue())) {
            baseClassLoader = thisClassLoader;
        }
        if (isDiagnosticsEnabled()) {
            logDiagnostic("[LOOKUP] Looking for system property [org.apache.commons.logging.LogFactory] to define the LogFactory subclass to use...");
        }
        try {
            String factoryClass = getSystemProperty(FACTORY_PROPERTY, (String) null);
            if (factoryClass != null) {
                if (isDiagnosticsEnabled()) {
                    StringBuffer stringBuffer2 = new StringBuffer();
                    stringBuffer2.append("[LOOKUP] Creating an instance of LogFactory class '");
                    stringBuffer2.append(factoryClass);
                    stringBuffer2.append("' as specified by system property ");
                    stringBuffer2.append(FACTORY_PROPERTY);
                    logDiagnostic(stringBuffer2.toString());
                }
                factory = newFactory(factoryClass, baseClassLoader, contextClassLoader);
            } else if (isDiagnosticsEnabled()) {
                logDiagnostic("[LOOKUP] No system property [org.apache.commons.logging.LogFactory] defined.");
            }
        } catch (SecurityException e) {
            if (isDiagnosticsEnabled()) {
                StringBuffer stringBuffer3 = new StringBuffer();
                stringBuffer3.append("[LOOKUP] A security exception occurred while trying to create an instance of the custom factory class: [");
                stringBuffer3.append(trim(e.getMessage()));
                stringBuffer3.append("]. Trying alternative implementations...");
                logDiagnostic(stringBuffer3.toString());
            }
        } catch (RuntimeException e2) {
            if (isDiagnosticsEnabled()) {
                StringBuffer stringBuffer4 = new StringBuffer();
                stringBuffer4.append("[LOOKUP] An exception occurred while trying to create an instance of the custom factory class: [");
                stringBuffer4.append(trim(e2.getMessage()));
                stringBuffer4.append("] as specified by a system property.");
                logDiagnostic(stringBuffer4.toString());
            }
            throw e2;
        }
        if (factory == null) {
            if (isDiagnosticsEnabled()) {
                logDiagnostic("[LOOKUP] Looking for a resource file of name [META-INF/services/org.apache.commons.logging.LogFactory] to define the LogFactory subclass to use...");
            }
            try {
                InputStream is = getResourceAsStream(contextClassLoader, SERVICE_ID);
                if (is != null) {
                    try {
                        rd = new BufferedReader(new InputStreamReader(is, "UTF-8"));
                    } catch (UnsupportedEncodingException e3) {
                        rd = new BufferedReader(new InputStreamReader(is));
                    }
                    String factoryClassName = rd.readLine();
                    rd.close();
                    if (factoryClassName != null && !"".equals(factoryClassName)) {
                        if (isDiagnosticsEnabled()) {
                            StringBuffer stringBuffer5 = new StringBuffer();
                            stringBuffer5.append("[LOOKUP]  Creating an instance of LogFactory class ");
                            stringBuffer5.append(factoryClassName);
                            stringBuffer5.append(" as specified by file '");
                            stringBuffer5.append(SERVICE_ID);
                            stringBuffer5.append("' which was present in the path of the context");
                            stringBuffer5.append(" classloader.");
                            logDiagnostic(stringBuffer5.toString());
                        }
                        factory = newFactory(factoryClassName, baseClassLoader, contextClassLoader);
                    }
                } else if (isDiagnosticsEnabled()) {
                    logDiagnostic("[LOOKUP] No resource file with name 'META-INF/services/org.apache.commons.logging.LogFactory' found.");
                }
            } catch (Exception ex) {
                if (isDiagnosticsEnabled()) {
                    StringBuffer stringBuffer6 = new StringBuffer();
                    stringBuffer6.append("[LOOKUP] A security exception occurred while trying to create an instance of the custom factory class: [");
                    stringBuffer6.append(trim(ex.getMessage()));
                    stringBuffer6.append("]. Trying alternative implementations...");
                    logDiagnostic(stringBuffer6.toString());
                }
            }
        }
        if (factory == null) {
            if (props != null) {
                if (isDiagnosticsEnabled()) {
                    logDiagnostic("[LOOKUP] Looking in properties file for entry with key 'org.apache.commons.logging.LogFactory' to define the LogFactory subclass to use...");
                }
                String factoryClass2 = props.getProperty(FACTORY_PROPERTY);
                if (factoryClass2 != null) {
                    if (isDiagnosticsEnabled()) {
                        StringBuffer stringBuffer7 = new StringBuffer();
                        stringBuffer7.append("[LOOKUP] Properties file specifies LogFactory subclass '");
                        stringBuffer7.append(factoryClass2);
                        stringBuffer7.append("'");
                        logDiagnostic(stringBuffer7.toString());
                    }
                    factory = newFactory(factoryClass2, baseClassLoader, contextClassLoader);
                } else if (isDiagnosticsEnabled()) {
                    logDiagnostic("[LOOKUP] Properties file has no entry specifying LogFactory subclass.");
                }
            } else if (isDiagnosticsEnabled()) {
                logDiagnostic("[LOOKUP] No properties file available to determine LogFactory subclass from..");
            }
        }
        if (factory == null) {
            if (isDiagnosticsEnabled()) {
                logDiagnostic("[LOOKUP] Loading the default LogFactory implementation 'org.apache.commons.logging.impl.LogFactoryImpl' via the same classloader that loaded this LogFactory class (ie not looking in the context classloader).");
            }
            factory = newFactory(FACTORY_DEFAULT, thisClassLoader, contextClassLoader);
        }
        if (factory != null) {
            cacheFactory(contextClassLoader, factory);
            if (props != null) {
                Enumeration names = props.propertyNames();
                while (names.hasMoreElements()) {
                    String name = (String) names.nextElement();
                    factory.setAttribute(name, props.getProperty(name));
                }
            }
        }
        return factory;
    }

    public static Log getLog(Class clazz) throws LogConfigurationException {
        return getFactory().getInstance(clazz);
    }

    public static Log getLog(String name) throws LogConfigurationException {
        return getFactory().getInstance(name);
    }

    public static void release(ClassLoader classLoader) {
        if (isDiagnosticsEnabled()) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Releasing factory for classloader ");
            stringBuffer.append(objectId(classLoader));
            logDiagnostic(stringBuffer.toString());
        }
        synchronized (factories) {
            if (classLoader == null) {
                try {
                    if (nullClassLoaderFactory != null) {
                        nullClassLoaderFactory.release();
                        nullClassLoaderFactory = null;
                    }
                } catch (Throwable th) {
                    throw th;
                }
            } else {
                LogFactory factory = (LogFactory) factories.get(classLoader);
                if (factory != null) {
                    factory.release();
                    factories.remove(classLoader);
                }
            }
        }
    }

    public static void releaseAll() {
        if (isDiagnosticsEnabled()) {
            logDiagnostic("Releasing factory for all classloaders.");
        }
        synchronized (factories) {
            Enumeration elements = factories.elements();
            while (elements.hasMoreElements()) {
                ((LogFactory) elements.nextElement()).release();
            }
            factories.clear();
            if (nullClassLoaderFactory != null) {
                nullClassLoaderFactory.release();
                nullClassLoaderFactory = null;
            }
        }
    }

    protected static ClassLoader getClassLoader(Class clazz) {
        try {
            return clazz.getClassLoader();
        } catch (SecurityException ex) {
            if (isDiagnosticsEnabled()) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Unable to get classloader for class '");
                stringBuffer.append(clazz);
                stringBuffer.append("' due to security restrictions - ");
                stringBuffer.append(ex.getMessage());
                logDiagnostic(stringBuffer.toString());
            }
            throw ex;
        }
    }

    protected static ClassLoader getContextClassLoader() throws LogConfigurationException {
        return directGetContextClassLoader();
    }

    private static ClassLoader getContextClassLoaderInternal() throws LogConfigurationException {
        return (ClassLoader) AccessController.doPrivileged(new PrivilegedAction() {
            public Object run() {
                return LogFactory.directGetContextClassLoader();
            }
        });
    }

    protected static ClassLoader directGetContextClassLoader() throws LogConfigurationException {
        Class cls;
        Class cls2;
        ClassLoader classLoader = null;
        try {
            if (class$java$lang$Thread == null) {
                cls2 = class$("java.lang.Thread");
                class$java$lang$Thread = cls2;
            } else {
                cls2 = class$java$lang$Thread;
            }
            classLoader = (ClassLoader) cls2.getMethod("getContextClassLoader", (Class[]) null).invoke(Thread.currentThread(), (Object[]) null);
        } catch (IllegalAccessException e) {
            throw new LogConfigurationException("Unexpected IllegalAccessException", e);
        } catch (InvocationTargetException e2) {
            if (!(e2.getTargetException() instanceof SecurityException)) {
                throw new LogConfigurationException("Unexpected InvocationTargetException", e2.getTargetException());
            }
        } catch (NoSuchMethodException e3) {
            if (class$org$apache$commons$logging$LogFactory == null) {
                cls = class$(FACTORY_PROPERTY);
                class$org$apache$commons$logging$LogFactory = cls;
            } else {
                cls = class$org$apache$commons$logging$LogFactory;
            }
            return getClassLoader(cls);
        }
        return classLoader;
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError(x1.getMessage());
        }
    }

    private static LogFactory getCachedFactory(ClassLoader contextClassLoader) {
        if (contextClassLoader == null) {
            return nullClassLoaderFactory;
        }
        return (LogFactory) factories.get(contextClassLoader);
    }

    private static void cacheFactory(ClassLoader classLoader, LogFactory factory) {
        if (factory == null) {
            return;
        }
        if (classLoader == null) {
            nullClassLoaderFactory = factory;
        } else {
            factories.put(classLoader, factory);
        }
    }

    protected static LogFactory newFactory(final String factoryClass, final ClassLoader classLoader, ClassLoader contextClassLoader) throws LogConfigurationException {
        Object result = AccessController.doPrivileged(new PrivilegedAction() {
            public Object run() {
                return LogFactory.createFactory(factoryClass, classLoader);
            }
        });
        if (result instanceof LogConfigurationException) {
            LogConfigurationException ex = (LogConfigurationException) result;
            if (isDiagnosticsEnabled()) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("An error occurred while loading the factory class:");
                stringBuffer.append(ex.getMessage());
                logDiagnostic(stringBuffer.toString());
            }
            throw ex;
        }
        if (isDiagnosticsEnabled()) {
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append("Created object ");
            stringBuffer2.append(objectId(result));
            stringBuffer2.append(" to manage classloader ");
            stringBuffer2.append(objectId(contextClassLoader));
            logDiagnostic(stringBuffer2.toString());
        }
        return (LogFactory) result;
    }

    protected static LogFactory newFactory(String factoryClass, ClassLoader classLoader) {
        return newFactory(factoryClass, classLoader, (ClassLoader) null);
    }

    protected static Object createFactory(String factoryClass, ClassLoader classLoader) {
        Class cls;
        String msg;
        Class cls2;
        Class cls3;
        Class cls4;
        if (classLoader != null) {
            try {
                Class logFactoryClass = classLoader.loadClass(factoryClass);
                if (class$org$apache$commons$logging$LogFactory == null) {
                    cls3 = class$(FACTORY_PROPERTY);
                    class$org$apache$commons$logging$LogFactory = cls3;
                } else {
                    cls3 = class$org$apache$commons$logging$LogFactory;
                }
                if (cls3.isAssignableFrom(logFactoryClass)) {
                    if (isDiagnosticsEnabled()) {
                        StringBuffer stringBuffer = new StringBuffer();
                        stringBuffer.append("Loaded class ");
                        stringBuffer.append(logFactoryClass.getName());
                        stringBuffer.append(" from classloader ");
                        stringBuffer.append(objectId(classLoader));
                        logDiagnostic(stringBuffer.toString());
                    }
                } else if (isDiagnosticsEnabled()) {
                    StringBuffer stringBuffer2 = new StringBuffer();
                    stringBuffer2.append("Factory class ");
                    stringBuffer2.append(logFactoryClass.getName());
                    stringBuffer2.append(" loaded from classloader ");
                    stringBuffer2.append(objectId(logFactoryClass.getClassLoader()));
                    stringBuffer2.append(" does not extend '");
                    if (class$org$apache$commons$logging$LogFactory == null) {
                        cls4 = class$(FACTORY_PROPERTY);
                        class$org$apache$commons$logging$LogFactory = cls4;
                    } else {
                        cls4 = class$org$apache$commons$logging$LogFactory;
                    }
                    stringBuffer2.append(cls4.getName());
                    stringBuffer2.append("' as loaded by this classloader.");
                    logDiagnostic(stringBuffer2.toString());
                    logHierarchy("[BAD CL TREE] ", classLoader);
                }
                return (LogFactory) logFactoryClass.newInstance();
            } catch (ClassNotFoundException ex) {
                if (classLoader == thisClassLoader) {
                    if (isDiagnosticsEnabled()) {
                        StringBuffer stringBuffer3 = new StringBuffer();
                        stringBuffer3.append("Unable to locate any class called '");
                        stringBuffer3.append(factoryClass);
                        stringBuffer3.append("' via classloader ");
                        stringBuffer3.append(objectId(classLoader));
                        logDiagnostic(stringBuffer3.toString());
                    }
                    throw ex;
                }
            } catch (NoClassDefFoundError e) {
                if (classLoader == thisClassLoader) {
                    if (isDiagnosticsEnabled()) {
                        StringBuffer stringBuffer4 = new StringBuffer();
                        stringBuffer4.append("Class '");
                        stringBuffer4.append(factoryClass);
                        stringBuffer4.append("' cannot be loaded");
                        stringBuffer4.append(" via classloader ");
                        stringBuffer4.append(objectId(classLoader));
                        stringBuffer4.append(" - it depends on some other class that cannot");
                        stringBuffer4.append(" be found.");
                        logDiagnostic(stringBuffer4.toString());
                    }
                    throw e;
                }
            } catch (ClassCastException e2) {
                if (classLoader == thisClassLoader) {
                    boolean implementsLogFactory = implementsLogFactory((Class) null);
                    StringBuffer stringBuffer5 = new StringBuffer();
                    stringBuffer5.append("The application has specified that a custom LogFactory implementation should be used but Class '");
                    stringBuffer5.append(factoryClass);
                    stringBuffer5.append("' cannot be converted to '");
                    if (class$org$apache$commons$logging$LogFactory == null) {
                        cls = class$(FACTORY_PROPERTY);
                        class$org$apache$commons$logging$LogFactory = cls;
                    } else {
                        cls = class$org$apache$commons$logging$LogFactory;
                    }
                    stringBuffer5.append(cls.getName());
                    stringBuffer5.append("'. ");
                    String msg2 = stringBuffer5.toString();
                    if (implementsLogFactory) {
                        StringBuffer stringBuffer6 = new StringBuffer();
                        stringBuffer6.append(msg2);
                        stringBuffer6.append("The conflict is caused by the presence of multiple LogFactory classes in incompatible classloaders. ");
                        stringBuffer6.append("Background can be found in http://commons.apache.org/logging/tech.html. ");
                        stringBuffer6.append("If you have not explicitly specified a custom LogFactory then it is likely that ");
                        stringBuffer6.append("the container has set one without your knowledge. ");
                        stringBuffer6.append("In this case, consider using the commons-logging-adapters.jar file or ");
                        stringBuffer6.append("specifying the standard LogFactory from the command line. ");
                        msg = stringBuffer6.toString();
                    } else {
                        StringBuffer stringBuffer7 = new StringBuffer();
                        stringBuffer7.append(msg2);
                        stringBuffer7.append("Please check the custom implementation. ");
                        msg = stringBuffer7.toString();
                    }
                    StringBuffer stringBuffer8 = new StringBuffer();
                    stringBuffer8.append(msg);
                    stringBuffer8.append("Help can be found @http://commons.apache.org/logging/troubleshooting.html.");
                    String msg3 = stringBuffer8.toString();
                    if (isDiagnosticsEnabled()) {
                        logDiagnostic(msg3);
                    }
                    throw new ClassCastException(msg3);
                }
            } catch (Exception e3) {
                if (isDiagnosticsEnabled()) {
                    logDiagnostic("Unable to create LogFactory instance.");
                }
                if (0 != 0) {
                    if (class$org$apache$commons$logging$LogFactory == null) {
                        cls2 = class$(FACTORY_PROPERTY);
                        class$org$apache$commons$logging$LogFactory = cls2;
                    } else {
                        cls2 = class$org$apache$commons$logging$LogFactory;
                    }
                    if (!cls2.isAssignableFrom((Class) null)) {
                        return new LogConfigurationException("The chosen LogFactory implementation does not extend LogFactory. Please check your configuration.", e3);
                    }
                }
                return new LogConfigurationException((Throwable) e3);
            }
        }
        if (isDiagnosticsEnabled()) {
            StringBuffer stringBuffer9 = new StringBuffer();
            stringBuffer9.append("Unable to load factory class via classloader ");
            stringBuffer9.append(objectId(classLoader));
            stringBuffer9.append(" - trying the classloader associated with this LogFactory.");
            logDiagnostic(stringBuffer9.toString());
        }
        return (LogFactory) Class.forName(factoryClass).newInstance();
    }

    private static boolean implementsLogFactory(Class logFactoryClass) {
        boolean implementsLogFactory = false;
        if (logFactoryClass != null) {
            try {
                ClassLoader logFactoryClassLoader = logFactoryClass.getClassLoader();
                if (logFactoryClassLoader == null) {
                    logDiagnostic("[CUSTOM LOG FACTORY] was loaded by the boot classloader");
                } else {
                    logHierarchy("[CUSTOM LOG FACTORY] ", logFactoryClassLoader);
                    implementsLogFactory = Class.forName(FACTORY_PROPERTY, false, logFactoryClassLoader).isAssignableFrom(logFactoryClass);
                    if (implementsLogFactory) {
                        StringBuffer stringBuffer = new StringBuffer();
                        stringBuffer.append("[CUSTOM LOG FACTORY] ");
                        stringBuffer.append(logFactoryClass.getName());
                        stringBuffer.append(" implements LogFactory but was loaded by an incompatible classloader.");
                        logDiagnostic(stringBuffer.toString());
                    } else {
                        StringBuffer stringBuffer2 = new StringBuffer();
                        stringBuffer2.append("[CUSTOM LOG FACTORY] ");
                        stringBuffer2.append(logFactoryClass.getName());
                        stringBuffer2.append(" does not implement LogFactory.");
                        logDiagnostic(stringBuffer2.toString());
                    }
                }
            } catch (SecurityException e) {
                StringBuffer stringBuffer3 = new StringBuffer();
                stringBuffer3.append("[CUSTOM LOG FACTORY] SecurityException thrown whilst trying to determine whether the compatibility was caused by a classloader conflict: ");
                stringBuffer3.append(e.getMessage());
                logDiagnostic(stringBuffer3.toString());
            } catch (LinkageError e2) {
                StringBuffer stringBuffer4 = new StringBuffer();
                stringBuffer4.append("[CUSTOM LOG FACTORY] LinkageError thrown whilst trying to determine whether the compatibility was caused by a classloader conflict: ");
                stringBuffer4.append(e2.getMessage());
                logDiagnostic(stringBuffer4.toString());
            } catch (ClassNotFoundException e3) {
                logDiagnostic("[CUSTOM LOG FACTORY] LogFactory class cannot be loaded by classloader which loaded the custom LogFactory implementation. Is the custom factory in the right classloader?");
            }
        }
        return implementsLogFactory;
    }

    private static InputStream getResourceAsStream(final ClassLoader loader, final String name) {
        return (InputStream) AccessController.doPrivileged(new PrivilegedAction() {
            public Object run() {
                if (loader != null) {
                    return loader.getResourceAsStream(name);
                }
                return ClassLoader.getSystemResourceAsStream(name);
            }
        });
    }

    private static Enumeration getResources(final ClassLoader loader, final String name) {
        return (Enumeration) AccessController.doPrivileged(new PrivilegedAction() {
            public Object run() {
                try {
                    if (loader != null) {
                        return loader.getResources(name);
                    }
                    return ClassLoader.getSystemResources(name);
                } catch (IOException e) {
                    if (LogFactory.isDiagnosticsEnabled()) {
                        StringBuffer stringBuffer = new StringBuffer();
                        stringBuffer.append("Exception while trying to find configuration file ");
                        stringBuffer.append(name);
                        stringBuffer.append(":");
                        stringBuffer.append(e.getMessage());
                        LogFactory.logDiagnostic(stringBuffer.toString());
                    }
                    return null;
                } catch (NoSuchMethodError e2) {
                    return null;
                }
            }
        });
    }

    private static Properties getProperties(final URL url) {
        return (Properties) AccessController.doPrivileged(new PrivilegedAction() {
            public Object run() {
                try {
                    InputStream stream = url.openStream();
                    if (stream == null) {
                        return null;
                    }
                    Properties props = new Properties();
                    props.load(stream);
                    stream.close();
                    return props;
                } catch (IOException e) {
                    if (!LogFactory.isDiagnosticsEnabled()) {
                        return null;
                    }
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("Unable to read URL ");
                    stringBuffer.append(url);
                    LogFactory.logDiagnostic(stringBuffer.toString());
                    return null;
                }
            }
        });
    }

    private static final Properties getConfigurationFile(ClassLoader classLoader, String fileName) {
        Properties props = null;
        double priority = opencv_stitching.Stitcher.ORIG_RESOL;
        URL propsUrl = null;
        try {
            Enumeration urls = getResources(classLoader, fileName);
            if (urls == null) {
                return null;
            }
            while (urls.hasMoreElements()) {
                URL url = (URL) urls.nextElement();
                Properties newProps = getProperties(url);
                if (newProps != null) {
                    if (props == null) {
                        propsUrl = url;
                        props = newProps;
                        String priorityStr = props.getProperty(PRIORITY_KEY);
                        priority = opencv_stitching.Stitcher.ORIG_RESOL;
                        if (priorityStr != null) {
                            priority = Double.parseDouble(priorityStr);
                        }
                        if (isDiagnosticsEnabled()) {
                            StringBuffer stringBuffer = new StringBuffer();
                            stringBuffer.append("[LOOKUP] Properties file found at '");
                            stringBuffer.append(url);
                            stringBuffer.append("'");
                            stringBuffer.append(" with priority ");
                            stringBuffer.append(priority);
                            logDiagnostic(stringBuffer.toString());
                        }
                    } else {
                        String newPriorityStr = newProps.getProperty(PRIORITY_KEY);
                        double newPriority = opencv_stitching.Stitcher.ORIG_RESOL;
                        if (newPriorityStr != null) {
                            newPriority = Double.parseDouble(newPriorityStr);
                        }
                        if (newPriority > priority) {
                            if (isDiagnosticsEnabled()) {
                                StringBuffer stringBuffer2 = new StringBuffer();
                                stringBuffer2.append("[LOOKUP] Properties file at '");
                                stringBuffer2.append(url);
                                stringBuffer2.append("'");
                                stringBuffer2.append(" with priority ");
                                stringBuffer2.append(newPriority);
                                stringBuffer2.append(" overrides file at '");
                                stringBuffer2.append(propsUrl);
                                stringBuffer2.append("'");
                                stringBuffer2.append(" with priority ");
                                stringBuffer2.append(priority);
                                logDiagnostic(stringBuffer2.toString());
                            }
                            propsUrl = url;
                            props = newProps;
                            priority = newPriority;
                        } else if (isDiagnosticsEnabled()) {
                            StringBuffer stringBuffer3 = new StringBuffer();
                            stringBuffer3.append("[LOOKUP] Properties file at '");
                            stringBuffer3.append(url);
                            stringBuffer3.append("'");
                            stringBuffer3.append(" with priority ");
                            stringBuffer3.append(newPriority);
                            stringBuffer3.append(" does not override file at '");
                            stringBuffer3.append(propsUrl);
                            stringBuffer3.append("'");
                            stringBuffer3.append(" with priority ");
                            stringBuffer3.append(priority);
                            logDiagnostic(stringBuffer3.toString());
                        }
                    }
                }
            }
            if (isDiagnosticsEnabled()) {
                if (props == null) {
                    StringBuffer stringBuffer4 = new StringBuffer();
                    stringBuffer4.append("[LOOKUP] No properties file of name '");
                    stringBuffer4.append(fileName);
                    stringBuffer4.append("' found.");
                    logDiagnostic(stringBuffer4.toString());
                } else {
                    StringBuffer stringBuffer5 = new StringBuffer();
                    stringBuffer5.append("[LOOKUP] Properties file of name '");
                    stringBuffer5.append(fileName);
                    stringBuffer5.append("' found at '");
                    stringBuffer5.append(propsUrl);
                    stringBuffer5.append('\"');
                    logDiagnostic(stringBuffer5.toString());
                }
            }
            return props;
        } catch (SecurityException e) {
            if (isDiagnosticsEnabled()) {
                logDiagnostic("SecurityException thrown while trying to find/read config files.");
            }
        }
    }

    private static String getSystemProperty(final String key, final String def) throws SecurityException {
        return (String) AccessController.doPrivileged(new PrivilegedAction() {
            public Object run() {
                return System.getProperty(key, def);
            }
        });
    }

    private static void initDiagnostics() {
        String classLoaderName;
        String classLoaderName2;
        try {
            String dest = getSystemProperty(DIAGNOSTICS_DEST_PROPERTY, (String) null);
            if (dest != null) {
                if (dest.equals("STDOUT")) {
                    diagnosticsStream = System.out;
                } else if (dest.equals("STDERR")) {
                    diagnosticsStream = System.err;
                } else {
                    try {
                        diagnosticsStream = new PrintStream(new FileOutputStream(dest, true));
                    } catch (IOException e) {
                        return;
                    }
                }
                try {
                    ClassLoader classLoader = thisClassLoader;
                    if (thisClassLoader == null) {
                        classLoaderName2 = "BOOTLOADER";
                    } else {
                        classLoaderName2 = objectId(classLoader);
                    }
                    classLoaderName = classLoaderName2;
                } catch (SecurityException e2) {
                    classLoaderName = "UNKNOWN";
                }
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("[LogFactory from ");
                stringBuffer.append(classLoaderName);
                stringBuffer.append("] ");
                diagnosticPrefix = stringBuffer.toString();
            }
        } catch (SecurityException e3) {
        }
    }

    protected static boolean isDiagnosticsEnabled() {
        return diagnosticsStream != null;
    }

    /* access modifiers changed from: private */
    public static final void logDiagnostic(String msg) {
        if (diagnosticsStream != null) {
            diagnosticsStream.print(diagnosticPrefix);
            diagnosticsStream.println(msg);
            diagnosticsStream.flush();
        }
    }

    protected static final void logRawDiagnostic(String msg) {
        if (diagnosticsStream != null) {
            diagnosticsStream.println(msg);
            diagnosticsStream.flush();
        }
    }

    private static void logClassLoaderEnvironment(Class clazz) {
        if (isDiagnosticsEnabled()) {
            try {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("[ENV] Extension directories (java.ext.dir): ");
                stringBuffer.append(System.getProperty("java.ext.dir"));
                logDiagnostic(stringBuffer.toString());
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append("[ENV] Application classpath (java.class.path): ");
                stringBuffer2.append(System.getProperty("java.class.path"));
                logDiagnostic(stringBuffer2.toString());
            } catch (SecurityException e) {
                logDiagnostic("[ENV] Security setting prevent interrogation of system classpaths.");
            }
            String className = clazz.getName();
            try {
                ClassLoader classLoader = getClassLoader(clazz);
                StringBuffer stringBuffer3 = new StringBuffer();
                stringBuffer3.append("[ENV] Class ");
                stringBuffer3.append(className);
                stringBuffer3.append(" was loaded via classloader ");
                stringBuffer3.append(objectId(classLoader));
                logDiagnostic(stringBuffer3.toString());
                StringBuffer stringBuffer4 = new StringBuffer();
                stringBuffer4.append("[ENV] Ancestry of classloader which loaded ");
                stringBuffer4.append(className);
                stringBuffer4.append(" is ");
                logHierarchy(stringBuffer4.toString(), classLoader);
            } catch (SecurityException e2) {
                StringBuffer stringBuffer5 = new StringBuffer();
                stringBuffer5.append("[ENV] Security forbids determining the classloader for ");
                stringBuffer5.append(className);
                logDiagnostic(stringBuffer5.toString());
            }
        }
    }

    private static void logHierarchy(String prefix, ClassLoader classLoader) {
        if (isDiagnosticsEnabled()) {
            if (classLoader != null) {
                String classLoaderString = classLoader.toString();
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append(prefix);
                stringBuffer.append(objectId(classLoader));
                stringBuffer.append(" == '");
                stringBuffer.append(classLoaderString);
                stringBuffer.append("'");
                logDiagnostic(stringBuffer.toString());
            }
            try {
                ClassLoader systemClassLoader = ClassLoader.getSystemClassLoader();
                if (classLoader != null) {
                    StringBuffer stringBuffer2 = new StringBuffer();
                    stringBuffer2.append(prefix);
                    stringBuffer2.append("ClassLoader tree:");
                    StringBuffer buf = new StringBuffer(stringBuffer2.toString());
                    do {
                        buf.append(objectId(classLoader));
                        if (classLoader == systemClassLoader) {
                            buf.append(" (SYSTEM) ");
                        }
                        try {
                            classLoader = classLoader.getParent();
                            buf.append(" --> ");
                        } catch (SecurityException e) {
                            buf.append(" --> SECRET");
                        }
                    } while (classLoader != null);
                    buf.append("BOOT");
                    logDiagnostic(buf.toString());
                }
            } catch (SecurityException e2) {
                StringBuffer stringBuffer3 = new StringBuffer();
                stringBuffer3.append(prefix);
                stringBuffer3.append("Security forbids determining the system classloader.");
                logDiagnostic(stringBuffer3.toString());
            }
        }
    }

    public static String objectId(Object o) {
        if (o == null) {
            return "null";
        }
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append(o.getClass().getName());
        stringBuffer.append("@");
        stringBuffer.append(System.identityHashCode(o));
        return stringBuffer.toString();
    }
}
