package org.ros.internal.message.service;

import java.util.List;
import org.ros.internal.message.definition.MessageDefinitionTupleParser;
import org.ros.message.MessageDeclaration;
import org.ros.message.MessageIdentifier;

public class ServiceDescription extends MessageDeclaration {
    private final String md5Checksum;
    private final String requestDefinition;
    private final String requestType;
    private final String responseDefinition;
    private final String responseType;

    public ServiceDescription(String type, String definition, String md5Checksum2) {
        super(MessageIdentifier.of(type), definition);
        this.md5Checksum = md5Checksum2;
        List<String> requestAndResponse = MessageDefinitionTupleParser.parse(definition, 2);
        this.requestType = type + "Request";
        this.responseType = type + "Response";
        this.requestDefinition = requestAndResponse.get(0);
        this.responseDefinition = requestAndResponse.get(1);
    }

    public String getMd5Checksum() {
        return this.md5Checksum;
    }

    public String getRequestType() {
        return this.requestType;
    }

    public String getRequestDefinition() {
        return this.requestDefinition;
    }

    public String getResponseType() {
        return this.responseType;
    }

    public String getResponseDefinition() {
        return this.responseDefinition;
    }

    public String toString() {
        return "ServiceDescription<" + getType() + ", " + this.md5Checksum + ">";
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
        ServiceDescription other = (ServiceDescription) obj;
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
