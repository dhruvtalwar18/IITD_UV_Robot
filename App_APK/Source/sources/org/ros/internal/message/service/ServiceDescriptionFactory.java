package org.ros.internal.message.service;

import org.ros.internal.message.Md5Generator;
import org.ros.message.MessageDefinitionProvider;

public class ServiceDescriptionFactory {
    private final Md5Generator md5Generator;
    private final MessageDefinitionProvider messageDefinitionProvider;

    public ServiceDescriptionFactory(MessageDefinitionProvider messageDefinitionProvider2) {
        this.messageDefinitionProvider = messageDefinitionProvider2;
        this.md5Generator = new Md5Generator(messageDefinitionProvider2);
    }

    public ServiceDescription newFromType(String serviceType) {
        return new ServiceDescription(serviceType, this.messageDefinitionProvider.get(serviceType), this.md5Generator.generate(serviceType));
    }

    public boolean hasType(String serviceType) {
        return this.messageDefinitionProvider.has(serviceType);
    }
}
