package org.jboss.netty.handler.codec.http.multipart;

import java.io.IOException;
import org.apache.xmlrpc.serializer.TypeSerializerImpl;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.handler.codec.http.HttpConstants;
import org.jboss.netty.handler.codec.http.multipart.InterfaceHttpData;

public class DiskAttribute extends AbstractDiskHttpData implements Attribute {
    public static String baseDirectory;
    public static boolean deleteOnExitTemporaryFile = true;
    public static String postfix = ".att";
    public static String prefix = "Attr_";

    public DiskAttribute(String name) {
        super(name, HttpConstants.DEFAULT_CHARSET, 0);
    }

    public DiskAttribute(String name, String value) throws IOException {
        super(name, HttpConstants.DEFAULT_CHARSET, 0);
        setValue(value);
    }

    public InterfaceHttpData.HttpDataType getHttpDataType() {
        return InterfaceHttpData.HttpDataType.Attribute;
    }

    public String getValue() throws IOException {
        return new String(get(), this.charset.name());
    }

    public void setValue(String value) throws IOException {
        if (value != null) {
            ChannelBuffer buffer = ChannelBuffers.wrappedBuffer(value.getBytes(this.charset.name()));
            if (this.definedSize > 0) {
                this.definedSize = (long) buffer.readableBytes();
            }
            setContent(buffer);
            return;
        }
        throw new NullPointerException(TypeSerializerImpl.VALUE_TAG);
    }

    public void addContent(ChannelBuffer buffer, boolean last) throws IOException {
        int localsize = buffer.readableBytes();
        if (this.definedSize > 0 && this.definedSize < this.size + ((long) localsize)) {
            this.definedSize = this.size + ((long) localsize);
        }
        super.addContent(buffer, last);
    }

    public int hashCode() {
        return getName().hashCode();
    }

    public boolean equals(Object o) {
        if (!(o instanceof Attribute)) {
            return false;
        }
        return getName().equalsIgnoreCase(((Attribute) o).getName());
    }

    public int compareTo(InterfaceHttpData arg0) {
        if (arg0 instanceof Attribute) {
            return compareTo((Attribute) arg0);
        }
        throw new ClassCastException("Cannot compare " + getHttpDataType() + " with " + arg0.getHttpDataType());
    }

    public int compareTo(Attribute o) {
        return getName().compareToIgnoreCase(o.getName());
    }

    public String toString() {
        try {
            return getName() + "=" + getValue();
        } catch (IOException e) {
            return getName() + "=IoException";
        }
    }

    /* access modifiers changed from: protected */
    public boolean deleteOnExit() {
        return deleteOnExitTemporaryFile;
    }

    /* access modifiers changed from: protected */
    public String getBaseDirectory() {
        return baseDirectory;
    }

    /* access modifiers changed from: protected */
    public String getDiskFilename() {
        return getName() + postfix;
    }

    /* access modifiers changed from: protected */
    public String getPostfix() {
        return postfix;
    }

    /* access modifiers changed from: protected */
    public String getPrefix() {
        return prefix;
    }
}
