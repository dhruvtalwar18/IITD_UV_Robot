package org.apache.commons.httpclient.params;

import java.io.Serializable;
import java.util.HashMap;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class DefaultHttpParams implements HttpParams, Serializable, Cloneable {
    private static final Log LOG;
    static /* synthetic */ Class class$org$apache$commons$httpclient$params$DefaultHttpParams;
    private static HttpParamsFactory httpParamsFactory = new DefaultHttpParamsFactory();
    private HttpParams defaults;
    private HashMap parameters;

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$params$DefaultHttpParams == null) {
            cls = class$("org.apache.commons.httpclient.params.DefaultHttpParams");
            class$org$apache$commons$httpclient$params$DefaultHttpParams = cls;
        } else {
            cls = class$org$apache$commons$httpclient$params$DefaultHttpParams;
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

    public static HttpParams getDefaultParams() {
        return httpParamsFactory.getDefaultParams();
    }

    public static void setHttpParamsFactory(HttpParamsFactory httpParamsFactory2) {
        if (httpParamsFactory2 != null) {
            httpParamsFactory = httpParamsFactory2;
            return;
        }
        throw new IllegalArgumentException("httpParamsFactory may not be null");
    }

    public DefaultHttpParams(HttpParams defaults2) {
        this.defaults = null;
        this.parameters = null;
        this.defaults = defaults2;
    }

    public DefaultHttpParams() {
        this(getDefaultParams());
    }

    public synchronized HttpParams getDefaults() {
        return this.defaults;
    }

    public synchronized void setDefaults(HttpParams params) {
        this.defaults = params;
    }

    public synchronized Object getParameter(String name) {
        Object param = null;
        if (this.parameters != null) {
            param = this.parameters.get(name);
        }
        if (param != null) {
            return param;
        }
        if (this.defaults == null) {
            return null;
        }
        return this.defaults.getParameter(name);
    }

    public synchronized void setParameter(String name, Object value) {
        if (this.parameters == null) {
            this.parameters = new HashMap();
        }
        this.parameters.put(name, value);
        if (LOG.isDebugEnabled()) {
            Log log = LOG;
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Set parameter ");
            stringBuffer.append(name);
            stringBuffer.append(" = ");
            stringBuffer.append(value);
            log.debug(stringBuffer.toString());
        }
    }

    public synchronized void setParameters(String[] names, Object value) {
        for (String parameter : names) {
            setParameter(parameter, value);
        }
    }

    public long getLongParameter(String name, long defaultValue) {
        Object param = getParameter(name);
        if (param == null) {
            return defaultValue;
        }
        return ((Long) param).longValue();
    }

    public void setLongParameter(String name, long value) {
        setParameter(name, new Long(value));
    }

    public int getIntParameter(String name, int defaultValue) {
        Object param = getParameter(name);
        if (param == null) {
            return defaultValue;
        }
        return ((Integer) param).intValue();
    }

    public void setIntParameter(String name, int value) {
        setParameter(name, new Integer(value));
    }

    public double getDoubleParameter(String name, double defaultValue) {
        Object param = getParameter(name);
        if (param == null) {
            return defaultValue;
        }
        return ((Double) param).doubleValue();
    }

    public void setDoubleParameter(String name, double value) {
        setParameter(name, new Double(value));
    }

    public boolean getBooleanParameter(String name, boolean defaultValue) {
        Object param = getParameter(name);
        if (param == null) {
            return defaultValue;
        }
        return ((Boolean) param).booleanValue();
    }

    public void setBooleanParameter(String name, boolean value) {
        setParameter(name, value ? Boolean.TRUE : Boolean.FALSE);
    }

    public boolean isParameterSet(String name) {
        return getParameter(name) != null;
    }

    public boolean isParameterSetLocally(String name) {
        return (this.parameters == null || this.parameters.get(name) == null) ? false : true;
    }

    public boolean isParameterTrue(String name) {
        return getBooleanParameter(name, false);
    }

    public boolean isParameterFalse(String name) {
        return !getBooleanParameter(name, false);
    }

    public void clear() {
        this.parameters = null;
    }

    public Object clone() throws CloneNotSupportedException {
        DefaultHttpParams clone = (DefaultHttpParams) super.clone();
        if (this.parameters != null) {
            clone.parameters = (HashMap) this.parameters.clone();
        }
        clone.setDefaults(this.defaults);
        return clone;
    }
}
