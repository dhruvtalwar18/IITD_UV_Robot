package org.ros.internal.message;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import org.ros.internal.message.field.Field;
import org.ros.internal.message.field.MessageFields;

public class MessageProxyInvocationHandler implements InvocationHandler {
    private final MessageImpl messageImpl;

    MessageProxyInvocationHandler(MessageImpl messageImpl2) {
        this.messageImpl = messageImpl2;
    }

    public Object invoke(Object proxy, Method method, Object[] args) throws Throwable {
        String methodName = method.getName();
        MessageFields mesageFields = this.messageImpl.getMessageFields();
        Field getterField = mesageFields.getGetterField(methodName);
        if (getterField != null) {
            return getterField.getValue();
        }
        Field setterField = mesageFields.getSetterField(methodName);
        if (setterField == null) {
            return method.invoke(this.messageImpl, args);
        }
        setterField.setValue(args[0]);
        return null;
    }
}
