package org.ros.message;

import com.google.common.base.Preconditions;

public class MessageDeclaration {
    private final String definition;
    private final MessageIdentifier messageIdentifier;

    public static MessageDeclaration of(String type, String definition2) {
        Preconditions.checkNotNull(type);
        Preconditions.checkNotNull(definition2);
        return new MessageDeclaration(MessageIdentifier.of(type), definition2);
    }

    public MessageDeclaration(MessageIdentifier messageIdentifier2, String definition2) {
        Preconditions.checkNotNull(messageIdentifier2);
        Preconditions.checkNotNull(definition2);
        this.messageIdentifier = messageIdentifier2;
        this.definition = definition2;
    }

    public MessageIdentifier getMessageIdentifier() {
        return this.messageIdentifier;
    }

    public String getType() {
        return this.messageIdentifier.getType();
    }

    public String getPackage() {
        return this.messageIdentifier.getPackage();
    }

    public String getName() {
        return this.messageIdentifier.getName();
    }

    public String getDefinition() {
        Preconditions.checkNotNull(this.definition);
        return this.definition;
    }

    public String toString() {
        return String.format("MessageDeclaration<%s>", new Object[]{this.messageIdentifier.toString()});
    }

    public int hashCode() {
        int i = 0;
        int result = ((1 * 31) + (this.definition == null ? 0 : this.definition.hashCode())) * 31;
        if (this.messageIdentifier != null) {
            i = this.messageIdentifier.hashCode();
        }
        return result + i;
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        MessageDeclaration other = (MessageDeclaration) obj;
        if (this.definition == null) {
            if (other.definition != null) {
                return false;
            }
        } else if (!this.definition.equals(other.definition)) {
            return false;
        }
        if (this.messageIdentifier == null) {
            if (other.messageIdentifier != null) {
                return false;
            }
        } else if (!this.messageIdentifier.equals(other.messageIdentifier)) {
            return false;
        }
        return true;
    }
}
