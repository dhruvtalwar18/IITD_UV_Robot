package org.ros.internal.transport;

import com.google.common.base.Preconditions;
import com.google.common.collect.Maps;
import java.nio.charset.Charset;
import java.util.Collections;
import java.util.Map;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.exception.RosRuntimeException;
import org.ros.internal.message.MessageBuffers;

public class ConnectionHeader {
    private static final boolean DEBUG = false;
    private static final Log log = LogFactory.getLog(ConnectionHeader.class);
    private final Map<String, String> fields = Maps.newConcurrentMap();

    public static ConnectionHeader decode(ChannelBuffer buffer) {
        Map<String, String> fields2 = Maps.newHashMap();
        int position = 0;
        int readableBytes = buffer.readableBytes();
        while (position < readableBytes) {
            int fieldSize = buffer.readInt();
            int position2 = position + 4;
            if (fieldSize == 0) {
                throw new IllegalStateException("Invalid 0 length handshake header field.");
            } else if (position2 + fieldSize <= readableBytes) {
                String field = decodeAsciiString(buffer, fieldSize);
                position = position2 + field.length();
                Preconditions.checkState(field.indexOf("=") > 0, String.format("Invalid field in handshake header: \"%s\"", new Object[]{field}));
                String[] keyAndValue = field.split("=");
                if (keyAndValue.length == 1) {
                    fields2.put(keyAndValue[0], "");
                } else {
                    fields2.put(keyAndValue[0], keyAndValue[1]);
                }
            } else {
                throw new IllegalStateException("Invalid line length handshake header field.");
            }
        }
        ConnectionHeader connectionHeader = new ConnectionHeader();
        connectionHeader.mergeFields(fields2);
        return connectionHeader;
    }

    private static String decodeAsciiString(ChannelBuffer buffer, int length) {
        return buffer.readBytes(length).toString(Charset.forName("US-ASCII"));
    }

    public ChannelBuffer encode() {
        ChannelBuffer buffer = MessageBuffers.dynamicBuffer();
        for (Map.Entry<String, String> entry : this.fields.entrySet()) {
            String field = entry.getKey() + "=" + entry.getValue();
            buffer.writeInt(field.length());
            buffer.writeBytes(field.getBytes(Charset.forName("US-ASCII")));
        }
        return buffer;
    }

    public void merge(ConnectionHeader other) {
        mergeFields(other.getFields());
    }

    public void mergeFields(Map<String, String> other) {
        for (Map.Entry<String, String> field : other.entrySet()) {
            addField(field.getKey(), field.getValue());
        }
    }

    public void addField(String name, String value) {
        if (!this.fields.containsKey(name) || this.fields.get(name).equals(value)) {
            this.fields.put(name, value);
        } else {
            throw new RosRuntimeException(String.format("Unable to merge field %s: %s != %s", new Object[]{name, value, this.fields.get(name)}));
        }
    }

    public Map<String, String> getFields() {
        return Collections.unmodifiableMap(this.fields);
    }

    public boolean hasField(String name) {
        return this.fields.containsKey(name);
    }

    public String getField(String name) {
        return this.fields.get(name);
    }

    public String toString() {
        return String.format("ConnectionHeader <%s>", new Object[]{this.fields.toString()});
    }

    public int hashCode() {
        return (1 * 31) + (this.fields == null ? 0 : this.fields.hashCode());
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        ConnectionHeader other = (ConnectionHeader) obj;
        if (this.fields == null) {
            if (other.fields != null) {
                return false;
            }
        } else if (!this.fields.equals(other.fields)) {
            return false;
        }
        return true;
    }
}
