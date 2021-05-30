package org.jboss.netty.handler.codec.rtsp;

import java.util.HashMap;
import java.util.Map;
import org.jboss.netty.handler.codec.http.HttpMethod;

public final class RtspMethods {
    public static final HttpMethod ANNOUNCE = new HttpMethod("ANNOUNCE");
    public static final HttpMethod DESCRIBE = new HttpMethod("DESCRIBE");
    public static final HttpMethod GET_PARAMETER = new HttpMethod("GET_PARAMETER");
    public static final HttpMethod OPTIONS = HttpMethod.OPTIONS;
    public static final HttpMethod PAUSE = new HttpMethod("PAUSE");
    public static final HttpMethod PLAY = new HttpMethod("PLAY");
    public static final HttpMethod RECORD = new HttpMethod("RECORD");
    public static final HttpMethod REDIRECT = new HttpMethod("REDIRECT");
    public static final HttpMethod SETUP = new HttpMethod("SETUP");
    public static final HttpMethod SET_PARAMETER = new HttpMethod("SET_PARAMETER");
    public static final HttpMethod TEARDOWN = new HttpMethod("TEARDOWN");
    private static final Map<String, HttpMethod> methodMap = new HashMap();

    static {
        methodMap.put(DESCRIBE.toString(), DESCRIBE);
        methodMap.put(ANNOUNCE.toString(), ANNOUNCE);
        methodMap.put(GET_PARAMETER.toString(), GET_PARAMETER);
        methodMap.put(OPTIONS.toString(), OPTIONS);
        methodMap.put(PAUSE.toString(), PAUSE);
        methodMap.put(PLAY.toString(), PLAY);
        methodMap.put(RECORD.toString(), RECORD);
        methodMap.put(REDIRECT.toString(), REDIRECT);
        methodMap.put(SETUP.toString(), SETUP);
        methodMap.put(SET_PARAMETER.toString(), SET_PARAMETER);
        methodMap.put(TEARDOWN.toString(), TEARDOWN);
    }

    public static HttpMethod valueOf(String name) {
        if (name != null) {
            String name2 = name.trim().toUpperCase();
            if (name2.length() != 0) {
                HttpMethod result = methodMap.get(name2);
                if (result != null) {
                    return result;
                }
                return new HttpMethod(name2);
            }
            throw new IllegalArgumentException("empty name");
        }
        throw new NullPointerException("name");
    }

    private RtspMethods() {
    }
}
