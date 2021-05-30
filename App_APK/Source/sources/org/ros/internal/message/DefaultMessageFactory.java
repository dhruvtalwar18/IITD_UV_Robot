package org.ros.internal.message;

import com.google.common.annotations.VisibleForTesting;
import org.ros.message.MessageDeclaration;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageFactory;

public class DefaultMessageFactory implements MessageFactory {
    private final MessageDefinitionProvider messageDefinitionProvider;
    private final DefaultMessageInterfaceClassProvider messageInterfaceClassProvider = new DefaultMessageInterfaceClassProvider();
    private final MessageProxyFactory messageProxyFactory = new MessageProxyFactory(getMessageInterfaceClassProvider(), this);

    public DefaultMessageFactory(MessageDefinitionProvider messageDefinitionProvider2) {
        this.messageDefinitionProvider = messageDefinitionProvider2;
    }

    public <T> T newFromType(String messageType) {
        return this.messageProxyFactory.newMessageProxy(MessageDeclaration.of(messageType, this.messageDefinitionProvider.get(messageType)));
    }

    /* access modifiers changed from: package-private */
    @VisibleForTesting
    public DefaultMessageInterfaceClassProvider getMessageInterfaceClassProvider() {
        return this.messageInterfaceClassProvider;
    }
}
