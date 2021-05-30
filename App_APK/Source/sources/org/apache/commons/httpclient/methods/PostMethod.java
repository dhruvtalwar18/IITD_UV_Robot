package org.apache.commons.httpclient.methods;

import java.util.Iterator;
import java.util.Vector;
import org.apache.commons.httpclient.NameValuePair;
import org.apache.commons.httpclient.util.EncodingUtil;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class PostMethod extends EntityEnclosingMethod {
    public static final String FORM_URL_ENCODED_CONTENT_TYPE = "application/x-www-form-urlencoded";
    private static final Log LOG;
    static /* synthetic */ Class class$org$apache$commons$httpclient$methods$PostMethod;
    private Vector params = new Vector();

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$methods$PostMethod == null) {
            cls = class$("org.apache.commons.httpclient.methods.PostMethod");
            class$org$apache$commons$httpclient$methods$PostMethod = cls;
        } else {
            cls = class$org$apache$commons$httpclient$methods$PostMethod;
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

    public PostMethod() {
    }

    public PostMethod(String uri) {
        super(uri);
    }

    public String getName() {
        return "POST";
    }

    /* access modifiers changed from: protected */
    public boolean hasRequestContent() {
        LOG.trace("enter PostMethod.hasRequestContent()");
        if (!this.params.isEmpty()) {
            return true;
        }
        return super.hasRequestContent();
    }

    /* access modifiers changed from: protected */
    public void clearRequestBody() {
        LOG.trace("enter PostMethod.clearRequestBody()");
        this.params.clear();
        super.clearRequestBody();
    }

    /* access modifiers changed from: protected */
    public RequestEntity generateRequestEntity() {
        if (!this.params.isEmpty()) {
            return new ByteArrayRequestEntity(EncodingUtil.getAsciiBytes(EncodingUtil.formUrlEncode(getParameters(), getRequestCharSet())), "application/x-www-form-urlencoded");
        }
        return super.generateRequestEntity();
    }

    public void setParameter(String parameterName, String parameterValue) {
        LOG.trace("enter PostMethod.setParameter(String, String)");
        removeParameter(parameterName);
        addParameter(parameterName, parameterValue);
    }

    public NameValuePair getParameter(String paramName) {
        LOG.trace("enter PostMethod.getParameter(String)");
        if (paramName == null) {
            return null;
        }
        Iterator iter = this.params.iterator();
        while (iter.hasNext()) {
            NameValuePair parameter = (NameValuePair) iter.next();
            if (paramName.equals(parameter.getName())) {
                return parameter;
            }
        }
        return null;
    }

    public NameValuePair[] getParameters() {
        LOG.trace("enter PostMethod.getParameters()");
        int numPairs = this.params.size();
        Object[] objectArr = this.params.toArray();
        NameValuePair[] nvPairArr = new NameValuePair[numPairs];
        for (int i = 0; i < numPairs; i++) {
            nvPairArr[i] = (NameValuePair) objectArr[i];
        }
        return nvPairArr;
    }

    public void addParameter(String paramName, String paramValue) throws IllegalArgumentException {
        LOG.trace("enter PostMethod.addParameter(String, String)");
        if (paramName == null || paramValue == null) {
            throw new IllegalArgumentException("Arguments to addParameter(String, String) cannot be null");
        }
        super.clearRequestBody();
        this.params.add(new NameValuePair(paramName, paramValue));
    }

    public void addParameter(NameValuePair param) throws IllegalArgumentException {
        LOG.trace("enter PostMethod.addParameter(NameValuePair)");
        if (param != null) {
            addParameter(param.getName(), param.getValue());
            return;
        }
        throw new IllegalArgumentException("NameValuePair may not be null");
    }

    public void addParameters(NameValuePair[] parameters) {
        LOG.trace("enter PostMethod.addParameters(NameValuePair[])");
        if (parameters == null) {
            LOG.warn("Attempt to addParameters(null) ignored");
            return;
        }
        super.clearRequestBody();
        for (NameValuePair add : parameters) {
            this.params.add(add);
        }
    }

    public boolean removeParameter(String paramName) throws IllegalArgumentException {
        LOG.trace("enter PostMethod.removeParameter(String)");
        if (paramName != null) {
            boolean removed = false;
            Iterator iter = this.params.iterator();
            while (iter.hasNext()) {
                if (paramName.equals(((NameValuePair) iter.next()).getName())) {
                    iter.remove();
                    removed = true;
                }
            }
            return removed;
        }
        throw new IllegalArgumentException("Argument passed to removeParameter(String) cannot be null");
    }

    public boolean removeParameter(String paramName, String paramValue) throws IllegalArgumentException {
        LOG.trace("enter PostMethod.removeParameter(String, String)");
        if (paramName == null) {
            throw new IllegalArgumentException("Parameter name may not be null");
        } else if (paramValue != null) {
            Iterator iter = this.params.iterator();
            while (iter.hasNext()) {
                NameValuePair pair = (NameValuePair) iter.next();
                if (paramName.equals(pair.getName()) && paramValue.equals(pair.getValue())) {
                    iter.remove();
                    return true;
                }
            }
            return false;
        } else {
            throw new IllegalArgumentException("Parameter value may not be null");
        }
    }

    public void setRequestBody(NameValuePair[] parametersBody) throws IllegalArgumentException {
        LOG.trace("enter PostMethod.setRequestBody(NameValuePair[])");
        if (parametersBody != null) {
            clearRequestBody();
            addParameters(parametersBody);
            return;
        }
        throw new IllegalArgumentException("Array of parameters may not be null");
    }
}
