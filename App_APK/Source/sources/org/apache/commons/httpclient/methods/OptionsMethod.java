package org.apache.commons.httpclient.methods;

import java.util.Enumeration;
import java.util.StringTokenizer;
import java.util.Vector;
import org.apache.commons.httpclient.Header;
import org.apache.commons.httpclient.HttpConnection;
import org.apache.commons.httpclient.HttpMethodBase;
import org.apache.commons.httpclient.HttpState;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class OptionsMethod extends HttpMethodBase {
    private static final Log LOG;
    static /* synthetic */ Class class$org$apache$commons$httpclient$methods$OptionsMethod;
    private Vector methodsAllowed = new Vector();

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$methods$OptionsMethod == null) {
            cls = class$("org.apache.commons.httpclient.methods.OptionsMethod");
            class$org$apache$commons$httpclient$methods$OptionsMethod = cls;
        } else {
            cls = class$org$apache$commons$httpclient$methods$OptionsMethod;
        }
        LOG = LogFactory.getLog(cls);
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError(x1.getMessage());
        }
    }

    public OptionsMethod() {
    }

    public OptionsMethod(String uri) {
        super(uri);
    }

    public String getName() {
        return "OPTIONS";
    }

    public boolean isAllowed(String method) {
        checkUsed();
        return this.methodsAllowed.contains(method);
    }

    public Enumeration getAllowedMethods() {
        checkUsed();
        return this.methodsAllowed.elements();
    }

    /* access modifiers changed from: protected */
    public void processResponseHeaders(HttpState state, HttpConnection conn) {
        LOG.trace("enter OptionsMethod.processResponseHeaders(HttpState, HttpConnection)");
        Header allowHeader = getResponseHeader("allow");
        if (allowHeader != null) {
            StringTokenizer tokenizer = new StringTokenizer(allowHeader.getValue(), ",");
            while (tokenizer.hasMoreElements()) {
                this.methodsAllowed.addElement(tokenizer.nextToken().trim().toUpperCase());
            }
        }
    }

    public boolean needContentLength() {
        return false;
    }
}
