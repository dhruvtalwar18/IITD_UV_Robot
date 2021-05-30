package com.google.common.eventbus;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

class SynchronizedEventHandler extends EventHandler {
    public SynchronizedEventHandler(Object target, Method method) {
        super(target, method);
    }

    public synchronized void handleEvent(Object event) throws InvocationTargetException {
        super.handleEvent(event);
    }
}
