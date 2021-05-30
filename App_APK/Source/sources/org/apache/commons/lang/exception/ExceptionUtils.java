package org.apache.commons.lang.exception;

import java.io.PrintStream;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.sql.SQLException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.StringTokenizer;
import org.apache.commons.lang.ArrayUtils;
import org.apache.commons.lang.ClassUtils;
import org.apache.commons.lang.NullArgumentException;
import org.apache.commons.lang.StringUtils;
import org.apache.commons.lang.SystemUtils;

public class ExceptionUtils {
    private static String[] CAUSE_METHOD_NAMES = {"getCause", "getNextException", "getTargetException", "getException", "getSourceException", "getRootCause", "getCausedByException", "getNested", "getLinkedException", "getNestedException", "getLinkedCause", "getThrowable"};
    private static final Method THROWABLE_CAUSE_METHOD;
    private static final Method THROWABLE_INITCAUSE_METHOD;
    static final String WRAPPED_MARKER = " [wrapped] ";
    static /* synthetic */ Class class$java$lang$Throwable;

    static {
        Method causeMethod;
        Class cls;
        Class cls2;
        Class cls3;
        Method causeMethod2 = null;
        try {
            if (class$java$lang$Throwable == null) {
                cls3 = class$("java.lang.Throwable");
                class$java$lang$Throwable = cls3;
            } else {
                cls3 = class$java$lang$Throwable;
            }
            causeMethod2 = cls3.getMethod("getCause", (Class[]) null);
        } catch (Exception e) {
        }
        THROWABLE_CAUSE_METHOD = causeMethod2;
        try {
            if (class$java$lang$Throwable == null) {
                cls = class$("java.lang.Throwable");
                class$java$lang$Throwable = cls;
            } else {
                cls = class$java$lang$Throwable;
            }
            Class[] clsArr = new Class[1];
            if (class$java$lang$Throwable == null) {
                cls2 = class$("java.lang.Throwable");
                class$java$lang$Throwable = cls2;
            } else {
                cls2 = class$java$lang$Throwable;
            }
            clsArr[0] = cls2;
            causeMethod = cls.getMethod("initCause", clsArr);
        } catch (Exception e2) {
            causeMethod = null;
        }
        THROWABLE_INITCAUSE_METHOD = causeMethod;
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError(x1.getMessage());
        }
    }

    public static void addCauseMethodName(String methodName) {
        if (StringUtils.isNotEmpty(methodName) && !isCauseMethodName(methodName)) {
            List list = getCauseMethodNameList();
            if (list.add(methodName)) {
                synchronized (CAUSE_METHOD_NAMES) {
                    CAUSE_METHOD_NAMES = toArray(list);
                }
            }
        }
    }

    public static void removeCauseMethodName(String methodName) {
        if (StringUtils.isNotEmpty(methodName)) {
            List list = getCauseMethodNameList();
            if (list.remove(methodName)) {
                synchronized (CAUSE_METHOD_NAMES) {
                    CAUSE_METHOD_NAMES = toArray(list);
                }
            }
        }
    }

    public static boolean setCause(Throwable target, Throwable cause) {
        Class cls;
        if (target != null) {
            Object[] causeArgs = {cause};
            boolean modifiedTarget = false;
            if (THROWABLE_INITCAUSE_METHOD != null) {
                try {
                    THROWABLE_INITCAUSE_METHOD.invoke(target, causeArgs);
                    modifiedTarget = true;
                } catch (IllegalAccessException | InvocationTargetException e) {
                }
            }
            try {
                Class<?> cls2 = target.getClass();
                Class[] clsArr = new Class[1];
                if (class$java$lang$Throwable == null) {
                    cls = class$("java.lang.Throwable");
                    class$java$lang$Throwable = cls;
                } else {
                    cls = class$java$lang$Throwable;
                }
                clsArr[0] = cls;
                cls2.getMethod("setCause", clsArr).invoke(target, causeArgs);
                return true;
            } catch (IllegalAccessException | NoSuchMethodException | InvocationTargetException e2) {
                return modifiedTarget;
            }
        } else {
            throw new NullArgumentException("target");
        }
    }

    private static String[] toArray(List list) {
        return (String[]) list.toArray(new String[list.size()]);
    }

    private static ArrayList getCauseMethodNameList() {
        ArrayList arrayList;
        synchronized (CAUSE_METHOD_NAMES) {
            arrayList = new ArrayList(Arrays.asList(CAUSE_METHOD_NAMES));
        }
        return arrayList;
    }

    public static boolean isCauseMethodName(String methodName) {
        boolean z;
        synchronized (CAUSE_METHOD_NAMES) {
            z = ArrayUtils.indexOf((Object[]) CAUSE_METHOD_NAMES, (Object) methodName) >= 0;
        }
        return z;
    }

    public static Throwable getCause(Throwable throwable) {
        Throwable cause;
        synchronized (CAUSE_METHOD_NAMES) {
            cause = getCause(throwable, CAUSE_METHOD_NAMES);
        }
        return cause;
    }

    public static Throwable getCause(Throwable throwable, String[] methodNames) {
        if (throwable == null) {
            return null;
        }
        Throwable cause = getCauseUsingWellKnownTypes(throwable);
        if (cause != null) {
            return cause;
        }
        if (methodNames == null) {
            synchronized (CAUSE_METHOD_NAMES) {
                methodNames = CAUSE_METHOD_NAMES;
            }
        }
        int i = 0;
        while (i < methodNames.length && ((methodName = methodNames[i]) == null || (cause = getCauseUsingMethodName(throwable, methodName)) == null)) {
            i++;
        }
        if (cause == null) {
            return getCauseUsingFieldName(throwable, "detail");
        }
        return cause;
    }

    public static Throwable getRootCause(Throwable throwable) {
        List list = getThrowableList(throwable);
        if (list.size() < 2) {
            return null;
        }
        return (Throwable) list.get(list.size() - 1);
    }

    private static Throwable getCauseUsingWellKnownTypes(Throwable throwable) {
        if (throwable instanceof Nestable) {
            return ((Nestable) throwable).getCause();
        }
        if (throwable instanceof SQLException) {
            return ((SQLException) throwable).getNextException();
        }
        if (throwable instanceof InvocationTargetException) {
            return ((InvocationTargetException) throwable).getTargetException();
        }
        return null;
    }

    private static Throwable getCauseUsingMethodName(Throwable throwable, String methodName) {
        Class cls;
        Method method = null;
        try {
            method = throwable.getClass().getMethod(methodName, (Class[]) null);
        } catch (NoSuchMethodException | SecurityException e) {
        }
        if (method != null) {
            if (class$java$lang$Throwable == null) {
                cls = class$("java.lang.Throwable");
                class$java$lang$Throwable = cls;
            } else {
                cls = class$java$lang$Throwable;
            }
            if (cls.isAssignableFrom(method.getReturnType())) {
                try {
                    return (Throwable) method.invoke(throwable, ArrayUtils.EMPTY_OBJECT_ARRAY);
                } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e2) {
                }
            }
        }
        return null;
    }

    private static Throwable getCauseUsingFieldName(Throwable throwable, String fieldName) {
        Class cls;
        Field field = null;
        try {
            field = throwable.getClass().getField(fieldName);
        } catch (NoSuchFieldException | SecurityException e) {
        }
        if (field != null) {
            if (class$java$lang$Throwable == null) {
                cls = class$("java.lang.Throwable");
                class$java$lang$Throwable = cls;
            } else {
                cls = class$java$lang$Throwable;
            }
            if (cls.isAssignableFrom(field.getType())) {
                try {
                    return (Throwable) field.get(throwable);
                } catch (IllegalAccessException | IllegalArgumentException e2) {
                }
            }
        }
        return null;
    }

    public static boolean isThrowableNested() {
        return THROWABLE_CAUSE_METHOD != null;
    }

    /* JADX WARNING: Code restructure failed: missing block: B:41:0x005c, code lost:
        if (r1.getField("detail") == null) goto L_0x0063;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:42:0x005e, code lost:
        return true;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public static boolean isNestedThrowable(java.lang.Throwable r9) {
        /*
            r0 = 0
            if (r9 != 0) goto L_0x0004
            return r0
        L_0x0004:
            boolean r1 = r9 instanceof org.apache.commons.lang.exception.Nestable
            r2 = 1
            if (r1 == 0) goto L_0x000a
            return r2
        L_0x000a:
            boolean r1 = r9 instanceof java.sql.SQLException
            if (r1 == 0) goto L_0x000f
            return r2
        L_0x000f:
            boolean r1 = r9 instanceof java.lang.reflect.InvocationTargetException
            if (r1 == 0) goto L_0x0014
            return r2
        L_0x0014:
            boolean r1 = isThrowableNested()
            if (r1 == 0) goto L_0x001b
            return r2
        L_0x001b:
            java.lang.Class r1 = r9.getClass()
            java.lang.String[] r3 = CAUSE_METHOD_NAMES
            monitor-enter(r3)
            r4 = 0
            java.lang.String[] r5 = CAUSE_METHOD_NAMES     // Catch:{ all -> 0x0064 }
            int r5 = r5.length     // Catch:{ all -> 0x0064 }
        L_0x0026:
            if (r4 >= r5) goto L_0x0055
            java.lang.String[] r6 = CAUSE_METHOD_NAMES     // Catch:{ NoSuchMethodException -> 0x0050, SecurityException -> 0x004e }
            r6 = r6[r4]     // Catch:{ NoSuchMethodException -> 0x0050, SecurityException -> 0x004e }
            r7 = 0
            java.lang.reflect.Method r6 = r1.getMethod(r6, r7)     // Catch:{ NoSuchMethodException -> 0x0050, SecurityException -> 0x004e }
            if (r6 == 0) goto L_0x0051
            java.lang.Class r7 = class$java$lang$Throwable     // Catch:{ NoSuchMethodException -> 0x0050, SecurityException -> 0x004e }
            if (r7 != 0) goto L_0x0040
            java.lang.String r7 = "java.lang.Throwable"
            java.lang.Class r7 = class$(r7)     // Catch:{ NoSuchMethodException -> 0x0050, SecurityException -> 0x004e }
            class$java$lang$Throwable = r7     // Catch:{ NoSuchMethodException -> 0x0050, SecurityException -> 0x004e }
            goto L_0x0042
        L_0x0040:
            java.lang.Class r7 = class$java$lang$Throwable     // Catch:{ NoSuchMethodException -> 0x0050, SecurityException -> 0x004e }
        L_0x0042:
            java.lang.Class r8 = r6.getReturnType()     // Catch:{ NoSuchMethodException -> 0x0050, SecurityException -> 0x004e }
            boolean r7 = r7.isAssignableFrom(r8)     // Catch:{ NoSuchMethodException -> 0x0050, SecurityException -> 0x004e }
            if (r7 == 0) goto L_0x0051
            monitor-exit(r3)     // Catch:{ all -> 0x0064 }
            return r2
        L_0x004e:
            r6 = move-exception
            goto L_0x0052
        L_0x0050:
            r6 = move-exception
        L_0x0051:
        L_0x0052:
            int r4 = r4 + 1
            goto L_0x0026
        L_0x0055:
            monitor-exit(r3)     // Catch:{ all -> 0x0064 }
            java.lang.String r3 = "detail"
            java.lang.reflect.Field r3 = r1.getField(r3)     // Catch:{ NoSuchFieldException -> 0x0061, SecurityException -> 0x005f }
            if (r3 == 0) goto L_0x0062
            return r2
        L_0x005f:
            r2 = move-exception
            goto L_0x0063
        L_0x0061:
            r2 = move-exception
        L_0x0062:
        L_0x0063:
            return r0
        L_0x0064:
            r0 = move-exception
            monitor-exit(r3)     // Catch:{ all -> 0x0064 }
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.lang.exception.ExceptionUtils.isNestedThrowable(java.lang.Throwable):boolean");
    }

    public static int getThrowableCount(Throwable throwable) {
        return getThrowableList(throwable).size();
    }

    public static Throwable[] getThrowables(Throwable throwable) {
        List list = getThrowableList(throwable);
        return (Throwable[]) list.toArray(new Throwable[list.size()]);
    }

    public static List getThrowableList(Throwable throwable) {
        List list = new ArrayList();
        while (throwable != null && !list.contains(throwable)) {
            list.add(throwable);
            throwable = getCause(throwable);
        }
        return list;
    }

    public static int indexOfThrowable(Throwable throwable, Class clazz) {
        return indexOf(throwable, clazz, 0, false);
    }

    public static int indexOfThrowable(Throwable throwable, Class clazz, int fromIndex) {
        return indexOf(throwable, clazz, fromIndex, false);
    }

    public static int indexOfType(Throwable throwable, Class type) {
        return indexOf(throwable, type, 0, true);
    }

    public static int indexOfType(Throwable throwable, Class type, int fromIndex) {
        return indexOf(throwable, type, fromIndex, true);
    }

    private static int indexOf(Throwable throwable, Class type, int fromIndex, boolean subclass) {
        if (throwable == null || type == null) {
            return -1;
        }
        if (fromIndex < 0) {
            fromIndex = 0;
        }
        Throwable[] throwables = getThrowables(throwable);
        if (fromIndex >= throwables.length) {
            return -1;
        }
        if (subclass) {
            for (int i = fromIndex; i < throwables.length; i++) {
                if (type.isAssignableFrom(throwables[i].getClass())) {
                    return i;
                }
            }
        } else {
            for (int i2 = fromIndex; i2 < throwables.length; i2++) {
                if (type.equals(throwables[i2].getClass())) {
                    return i2;
                }
            }
        }
        return -1;
    }

    public static void printRootCauseStackTrace(Throwable throwable) {
        printRootCauseStackTrace(throwable, System.err);
    }

    public static void printRootCauseStackTrace(Throwable throwable, PrintStream stream) {
        if (throwable != null) {
            if (stream != null) {
                String[] trace = getRootCauseStackTrace(throwable);
                for (String println : trace) {
                    stream.println(println);
                }
                stream.flush();
                return;
            }
            throw new IllegalArgumentException("The PrintStream must not be null");
        }
    }

    public static void printRootCauseStackTrace(Throwable throwable, PrintWriter writer) {
        if (throwable != null) {
            if (writer != null) {
                String[] trace = getRootCauseStackTrace(throwable);
                for (String println : trace) {
                    writer.println(println);
                }
                writer.flush();
                return;
            }
            throw new IllegalArgumentException("The PrintWriter must not be null");
        }
    }

    public static String[] getRootCauseStackTrace(Throwable throwable) {
        if (throwable == null) {
            return ArrayUtils.EMPTY_STRING_ARRAY;
        }
        Throwable[] throwables = getThrowables(throwable);
        int count = throwables.length;
        ArrayList frames = new ArrayList();
        List nextTrace = getStackFrameList(throwables[count - 1]);
        int i = count;
        while (true) {
            i--;
            if (i < 0) {
                return (String[]) frames.toArray(new String[0]);
            }
            List trace = nextTrace;
            if (i != 0) {
                nextTrace = getStackFrameList(throwables[i - 1]);
                removeCommonFrames(trace, nextTrace);
            }
            if (i == count - 1) {
                frames.add(throwables[i].toString());
            } else {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append(WRAPPED_MARKER);
                stringBuffer.append(throwables[i].toString());
                frames.add(stringBuffer.toString());
            }
            for (int j = 0; j < trace.size(); j++) {
                frames.add(trace.get(j));
            }
        }
    }

    public static void removeCommonFrames(List causeFrames, List wrapperFrames) {
        if (causeFrames == null || wrapperFrames == null) {
            throw new IllegalArgumentException("The List must not be null");
        }
        int causeFrameIndex = causeFrames.size() - 1;
        int wrapperFrameIndex = wrapperFrames.size() - 1;
        while (causeFrameIndex >= 0 && wrapperFrameIndex >= 0) {
            if (((String) causeFrames.get(causeFrameIndex)).equals((String) wrapperFrames.get(wrapperFrameIndex))) {
                causeFrames.remove(causeFrameIndex);
            }
            causeFrameIndex--;
            wrapperFrameIndex--;
        }
    }

    public static String getFullStackTrace(Throwable throwable) {
        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw, true);
        Throwable[] ts = getThrowables(throwable);
        for (int i = 0; i < ts.length; i++) {
            ts[i].printStackTrace(pw);
            if (isNestedThrowable(ts[i])) {
                break;
            }
        }
        return sw.getBuffer().toString();
    }

    public static String getStackTrace(Throwable throwable) {
        StringWriter sw = new StringWriter();
        throwable.printStackTrace(new PrintWriter(sw, true));
        return sw.getBuffer().toString();
    }

    public static String[] getStackFrames(Throwable throwable) {
        if (throwable == null) {
            return ArrayUtils.EMPTY_STRING_ARRAY;
        }
        return getStackFrames(getStackTrace(throwable));
    }

    static String[] getStackFrames(String stackTrace) {
        StringTokenizer frames = new StringTokenizer(stackTrace, SystemUtils.LINE_SEPARATOR);
        List list = new ArrayList();
        while (frames.hasMoreTokens()) {
            list.add(frames.nextToken());
        }
        return toArray(list);
    }

    static List getStackFrameList(Throwable t) {
        StringTokenizer frames = new StringTokenizer(getStackTrace(t), SystemUtils.LINE_SEPARATOR);
        List list = new ArrayList();
        boolean traceStarted = false;
        while (frames.hasMoreTokens()) {
            String token = frames.nextToken();
            int at = token.indexOf("at");
            if (at != -1 && token.substring(0, at).trim().length() == 0) {
                traceStarted = true;
                list.add(token);
            } else if (traceStarted) {
                break;
            }
        }
        return list;
    }

    public static String getMessage(Throwable th) {
        if (th == null) {
            return "";
        }
        String clsName = ClassUtils.getShortClassName(th, (String) null);
        String msg = th.getMessage();
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append(clsName);
        stringBuffer.append(": ");
        stringBuffer.append(StringUtils.defaultString(msg));
        return stringBuffer.toString();
    }

    public static String getRootCauseMessage(Throwable th) {
        Throwable root = getRootCause(th);
        return getMessage(root == null ? th : root);
    }
}
