package org.apache.xmlrpc;

import java.util.TimeZone;
import org.apache.xmlrpc.common.XmlRpcHttpConfig;

public abstract class XmlRpcConfigImpl implements XmlRpcConfig, XmlRpcHttpConfig {
    private String basicEncoding;
    private boolean contentLengthOptional;
    private boolean enabledForExtensions;
    private String encoding;
    private TimeZone timeZone = TimeZone.getDefault();

    public boolean isEnabledForExtensions() {
        return this.enabledForExtensions;
    }

    public void setEnabledForExtensions(boolean pExtensions) {
        this.enabledForExtensions = pExtensions;
    }

    public void setBasicEncoding(String pEncoding) {
        this.basicEncoding = pEncoding;
    }

    public String getBasicEncoding() {
        return this.basicEncoding;
    }

    public void setEncoding(String pEncoding) {
        this.encoding = pEncoding;
    }

    public String getEncoding() {
        return this.encoding;
    }

    public boolean isContentLengthOptional() {
        return this.contentLengthOptional;
    }

    public void setContentLengthOptional(boolean pContentLengthOptional) {
        this.contentLengthOptional = pContentLengthOptional;
    }

    public TimeZone getTimeZone() {
        return this.timeZone;
    }

    public void setTimeZone(TimeZone pTimeZone) {
        this.timeZone = pTimeZone;
    }
}
