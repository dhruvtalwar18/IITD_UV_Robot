package org.ros.internal.message.topic;

import org.ros.message.MessageDeclaration;
import org.ros.message.MessageIdentifier;

public class TopicDescription extends MessageDeclaration {
    private final String md5Checksum;

    public TopicDescription(String type, String definition, String md5Checksum2) {
        super(MessageIdentifier.of(type), definition);
        this.md5Checksum = md5Checksum2;
    }

    public String getMd5Checksum() {
        return this.md5Checksum;
    }

    public String toString() {
        return "TopicDescription<" + getType() + ", " + this.md5Checksum + ">";
    }

    public int hashCode() {
        return (super.hashCode() * 31) + (this.md5Checksum == null ? 0 : this.md5Checksum.hashCode());
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!super.equals(obj) || getClass() != obj.getClass()) {
            return false;
        }
        TopicDescription other = (TopicDescription) obj;
        if (this.md5Checksum == null) {
            if (other.md5Checksum != null) {
                return false;
            }
        } else if (!this.md5Checksum.equals(other.md5Checksum)) {
            return false;
        }
        return true;
    }
}
