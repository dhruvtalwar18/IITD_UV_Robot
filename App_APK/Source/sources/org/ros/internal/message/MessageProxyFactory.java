package org.ros.internal.message;

import com.google.common.base.Preconditions;
import java.lang.reflect.Proxy;
import java.util.concurrent.atomic.AtomicInteger;
import org.ros.internal.message.context.MessageContextProvider;
import org.ros.message.MessageDeclaration;
import org.ros.message.MessageFactory;

public class MessageProxyFactory {
    private static final String HEADER_MESSAGE_TYPE = "std_msgs/Header";
    private static final String SEQUENCE_FIELD_NAME = "seq";
    private static final AtomicInteger SEQUENCE_NUMBER = new AtomicInteger(0);
    private final MessageContextProvider messageContextProvider;
    private final MessageInterfaceClassProvider messageInterfaceClassProvider;

    public MessageProxyFactory(MessageInterfaceClassProvider messageInterfaceClassProvider2, MessageFactory messageFactory) {
        this.messageInterfaceClassProvider = messageInterfaceClassProvider2;
        this.messageContextProvider = new MessageContextProvider(messageFactory);
    }

    public <T> T newMessageProxy(MessageDeclaration messageDeclaration) {
        Preconditions.checkNotNull(messageDeclaration);
        MessageImpl messageImpl = new MessageImpl(this.messageContextProvider.get(messageDeclaration));
        if (messageImpl.getType().equals("std_msgs/Header")) {
            messageImpl.setUInt32("seq", SEQUENCE_NUMBER.getAndIncrement());
        }
        return newProxy(this.messageInterfaceClassProvider.get(messageDeclaration.getType()), messageImpl);
    }

    private <T> T newProxy(Class<T> interfaceClass, MessageImpl messageImpl) {
        return Proxy.newProxyInstance(messageImpl.getClass().getClassLoader(), new Class[]{interfaceClass, GetInstance.class}, new MessageProxyInvocationHandler(messageImpl));
    }
}
