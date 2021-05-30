package org.ros.internal.message.definition;

import com.google.common.collect.Maps;
import java.io.File;
import java.util.Collection;
import java.util.HashSet;
import java.util.Map;
import org.apache.commons.io.FilenameUtils;
import org.ros.internal.message.StringFileProvider;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageIdentifier;

public class MessageDefinitionFileProvider implements MessageDefinitionProvider {
    private final Map<String, String> definitions = Maps.newConcurrentMap();
    private final Map<String, Collection<MessageIdentifier>> messageIdentifiers = Maps.newConcurrentMap();
    private final StringFileProvider stringFileProvider;

    public MessageDefinitionFileProvider(StringFileProvider stringFileProvider2) {
        this.stringFileProvider = stringFileProvider2;
    }

    private static String getParent(String filename) {
        return FilenameUtils.getFullPathNoEndSeparator(filename);
    }

    /* access modifiers changed from: protected */
    public static String getParentBaseName(String filename) {
        return FilenameUtils.getBaseName(getParent(filename));
    }

    private static MessageIdentifier fileToMessageIdentifier(File file) {
        String filename = file.getAbsolutePath();
        return MessageIdentifier.of(getParentBaseName(getParent(filename)), FilenameUtils.getBaseName(filename));
    }

    private void addDefinition(File file, String definition) {
        MessageIdentifier topicType = fileToMessageIdentifier(file);
        if (!this.definitions.containsKey(topicType.getType())) {
            this.definitions.put(topicType.getType(), definition);
            if (!this.messageIdentifiers.containsKey(topicType.getPackage())) {
                this.messageIdentifiers.put(topicType.getPackage(), new HashSet());
            }
            this.messageIdentifiers.get(topicType.getPackage()).add(topicType);
        }
    }

    public void update() {
        this.stringFileProvider.update();
        for (Map.Entry<File, String> entry : this.stringFileProvider.getStrings().entrySet()) {
            addDefinition(entry.getKey(), entry.getValue());
        }
    }

    public void addDirectory(File directory) {
        this.stringFileProvider.addDirectory(directory);
    }

    public Collection<String> getPackages() {
        return this.messageIdentifiers.keySet();
    }

    public Collection<MessageIdentifier> getMessageIdentifiersByPackage(String pkg) {
        return this.messageIdentifiers.get(pkg);
    }

    public String get(String type) {
        return this.definitions.get(type);
    }

    public boolean has(String type) {
        return this.definitions.containsKey(type);
    }
}
