package com.google.common.reflect;

import com.google.common.annotations.Beta;
import com.google.common.base.Preconditions;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Proxy;

@Beta
public final class Reflection {
    public static String getPackageName(Class<?> cls) {
        return getPackageName(cls.getName());
    }

    public static String getPackageName(String classFullName) {
        int lastDot = classFullName.lastIndexOf(46);
        if (lastDot < 0) {
            return "";
        }
        return classFullName.substring(0, lastDot);
    }

    public static void initialize(Class<?>... classes) {
        Class<?>[] arr$ = classes;
        int len$ = arr$.length;
        int i$ = 0;
        while (i$ < len$) {
            Class<?> clazz = arr$[i$];
            try {
                Class.forName(clazz.getName(), true, clazz.getClassLoader());
                i$++;
            } catch (ClassNotFoundException e) {
                throw new AssertionError(e);
            }
        }
    }

    public static <T> T newProxy(Class<T> interfaceType, InvocationHandler handler) {
        Preconditions.checkNotNull(interfaceType);
        Preconditions.checkNotNull(handler);
        Preconditions.checkArgument(interfaceType.isInterface());
        return interfaceType.cast(Proxy.newProxyInstance(interfaceType.getClassLoader(), new Class[]{interfaceType}, handler));
    }

    private Reflection() {
    }
}
