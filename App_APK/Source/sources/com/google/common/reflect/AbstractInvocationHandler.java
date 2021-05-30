package com.google.common.reflect;

import com.google.common.annotations.Beta;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import javax.annotation.Nullable;

@Beta
public abstract class AbstractInvocationHandler implements InvocationHandler {
    private static final Object[] NO_ARGS = new Object[0];

    /* access modifiers changed from: protected */
    public abstract Object handleInvocation(Object obj, Method method, Object[] objArr) throws Throwable;

    public final Object invoke(Object proxy, Method method, @Nullable Object[] args) throws Throwable {
        if (args == null) {
            args = NO_ARGS;
        }
        if (args.length == 0 && method.getName().equals("hashCode")) {
            return Integer.valueOf(System.identityHashCode(proxy));
        }
        boolean z = true;
        if (args.length == 1 && method.getName().equals("equals") && method.getParameterTypes()[0] == Object.class) {
            if (proxy != args[0]) {
                z = false;
            }
            return Boolean.valueOf(z);
        } else if (args.length != 0 || !method.getName().equals("toString")) {
            return handleInvocation(proxy, method, args);
        } else {
            return toString();
        }
    }

    public String toString() {
        return super.toString();
    }
}
