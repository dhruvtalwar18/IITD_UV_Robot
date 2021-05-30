package com.google.common.eventbus;

import com.google.common.base.Preconditions;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

class EventHandler {
    private final Method method;
    private final Object target;

    EventHandler(Object target2, Method method2) {
        Preconditions.checkNotNull(target2, "EventHandler target cannot be null.");
        Preconditions.checkNotNull(method2, "EventHandler method cannot be null.");
        this.target = target2;
        this.method = method2;
        method2.setAccessible(true);
    }

    public void handleEvent(Object event) throws InvocationTargetException {
        try {
            this.method.invoke(this.target, new Object[]{event});
        } catch (IllegalArgumentException e) {
            throw new Error("Method rejected target/argument: " + event, e);
        } catch (IllegalAccessException e2) {
            throw new Error("Method became inaccessible: " + event, e2);
        } catch (InvocationTargetException e3) {
            if (e3.getCause() instanceof Error) {
                throw ((Error) e3.getCause());
            }
            throw e3;
        }
    }

    public String toString() {
        return "[wrapper " + this.method + "]";
    }

    public int hashCode() {
        return ((this.method.hashCode() + 31) * 31) + this.target.hashCode();
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        EventHandler other = (EventHandler) obj;
        if (!this.method.equals(other.method) || this.target != other.target) {
            return false;
        }
        return true;
    }
}
