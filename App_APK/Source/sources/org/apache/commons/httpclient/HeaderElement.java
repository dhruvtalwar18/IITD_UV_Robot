package org.apache.commons.httpclient;

import java.util.ArrayList;
import java.util.List;
import org.apache.commons.httpclient.util.ParameterParser;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class HeaderElement extends NameValuePair {
    private static final Log LOG;
    static /* synthetic */ Class class$org$apache$commons$httpclient$HeaderElement;
    private NameValuePair[] parameters;

    public HeaderElement() {
        this((String) null, (String) null, (NameValuePair[]) null);
    }

    public HeaderElement(String name, String value) {
        this(name, value, (NameValuePair[]) null);
    }

    public HeaderElement(String name, String value, NameValuePair[] parameters2) {
        super(name, value);
        this.parameters = null;
        this.parameters = parameters2;
    }

    public HeaderElement(char[] chars, int offset, int length) {
        this();
        if (chars != null) {
            List params = new ParameterParser().parse(chars, offset, length, ';');
            if (params.size() > 0) {
                NameValuePair element = (NameValuePair) params.remove(0);
                setName(element.getName());
                setValue(element.getValue());
                if (params.size() > 0) {
                    this.parameters = (NameValuePair[]) params.toArray(new NameValuePair[params.size()]);
                }
            }
        }
    }

    public HeaderElement(char[] chars) {
        this(chars, 0, chars.length);
    }

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$HeaderElement == null) {
            cls = class$("org.apache.commons.httpclient.HeaderElement");
            class$org$apache$commons$httpclient$HeaderElement = cls;
        } else {
            cls = class$org$apache$commons$httpclient$HeaderElement;
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

    public NameValuePair[] getParameters() {
        return this.parameters;
    }

    public static final HeaderElement[] parseElements(char[] headerValue) {
        LOG.trace("enter HeaderElement.parseElements(char[])");
        if (headerValue == null) {
            return new HeaderElement[0];
        }
        List elements = new ArrayList();
        int len = headerValue.length;
        int from = 0;
        boolean qouted = false;
        for (int i = 0; i < len; i++) {
            char ch = headerValue[i];
            if (ch == '\"') {
                qouted = !qouted;
            }
            HeaderElement element = null;
            if (!qouted && ch == ',') {
                element = new HeaderElement(headerValue, from, i);
                from = i + 1;
            } else if (i == len - 1) {
                element = new HeaderElement(headerValue, from, len);
            }
            if (!(element == null || element.getName() == null)) {
                elements.add(element);
            }
        }
        return (HeaderElement[]) elements.toArray(new HeaderElement[elements.size()]);
    }

    public static final HeaderElement[] parseElements(String headerValue) {
        LOG.trace("enter HeaderElement.parseElements(String)");
        if (headerValue == null) {
            return new HeaderElement[0];
        }
        return parseElements(headerValue.toCharArray());
    }

    public static final HeaderElement[] parse(String headerValue) throws HttpException {
        LOG.trace("enter HeaderElement.parse(String)");
        if (headerValue == null) {
            return new HeaderElement[0];
        }
        return parseElements(headerValue.toCharArray());
    }

    public NameValuePair getParameterByName(String name) {
        LOG.trace("enter HeaderElement.getParameterByName(String)");
        if (name != null) {
            NameValuePair[] parameters2 = getParameters();
            if (parameters2 == null) {
                return null;
            }
            for (NameValuePair current : parameters2) {
                if (current.getName().equalsIgnoreCase(name)) {
                    return current;
                }
            }
            return null;
        }
        throw new IllegalArgumentException("Name may not be null");
    }
}
