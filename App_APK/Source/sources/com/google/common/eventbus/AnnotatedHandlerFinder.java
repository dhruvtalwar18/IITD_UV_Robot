package com.google.common.eventbus;

import com.google.common.collect.HashMultimap;
import com.google.common.collect.Multimap;
import java.lang.reflect.Method;

class AnnotatedHandlerFinder implements HandlerFindingStrategy {
    AnnotatedHandlerFinder() {
    }

    public Multimap<Class<?>, EventHandler> findAllHandlers(Object listener) {
        Multimap<Class<?>, EventHandler> methodsInListener = HashMultimap.create();
        for (Class clazz = listener.getClass(); clazz != null; clazz = clazz.getSuperclass()) {
            for (Method method : clazz.getMethods()) {
                if (((Subscribe) method.getAnnotation(Subscribe.class)) != null) {
                    Class<?>[] parameterTypes = method.getParameterTypes();
                    if (parameterTypes.length == 1) {
                        methodsInListener.put(parameterTypes[0], makeHandler(listener, method));
                    } else {
                        throw new IllegalArgumentException("Method " + method + " has @Subscribe annotation, but requires " + parameterTypes.length + " arguments.  Event handler methods " + "must require a single argument.");
                    }
                }
            }
        }
        return methodsInListener;
    }

    private static EventHandler makeHandler(Object listener, Method method) {
        if (methodIsDeclaredThreadSafe(method)) {
            return new EventHandler(listener, method);
        }
        return new SynchronizedEventHandler(listener, method);
    }

    private static boolean methodIsDeclaredThreadSafe(Method method) {
        return method.getAnnotation(AllowConcurrentEvents.class) != null;
    }
}
