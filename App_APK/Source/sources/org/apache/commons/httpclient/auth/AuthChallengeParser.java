package org.apache.commons.httpclient.auth;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.apache.commons.httpclient.Header;
import org.apache.commons.httpclient.NameValuePair;
import org.apache.commons.httpclient.util.ParameterParser;

public final class AuthChallengeParser {
    public static String extractScheme(String challengeStr) throws MalformedChallengeException {
        String s;
        if (challengeStr != null) {
            int idx = challengeStr.indexOf(32);
            if (idx == -1) {
                s = challengeStr;
            } else {
                s = challengeStr.substring(0, idx);
            }
            if (!s.equals("")) {
                return s.toLowerCase();
            }
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Invalid challenge: ");
            stringBuffer.append(challengeStr);
            throw new MalformedChallengeException(stringBuffer.toString());
        }
        throw new IllegalArgumentException("Challenge may not be null");
    }

    public static Map extractParams(String challengeStr) throws MalformedChallengeException {
        if (challengeStr != null) {
            int idx = challengeStr.indexOf(32);
            if (idx != -1) {
                Map map = new HashMap();
                List params = new ParameterParser().parse(challengeStr.substring(idx + 1, challengeStr.length()), ',');
                for (int i = 0; i < params.size(); i++) {
                    NameValuePair param = (NameValuePair) params.get(i);
                    map.put(param.getName().toLowerCase(), param.getValue());
                }
                return map;
            }
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Invalid challenge: ");
            stringBuffer.append(challengeStr);
            throw new MalformedChallengeException(stringBuffer.toString());
        }
        throw new IllegalArgumentException("Challenge may not be null");
    }

    public static Map parseChallenges(Header[] headers) throws MalformedChallengeException {
        if (headers != null) {
            Map challengemap = new HashMap(headers.length);
            for (Header value : headers) {
                String challenge = value.getValue();
                challengemap.put(extractScheme(challenge), challenge);
            }
            return challengemap;
        }
        throw new IllegalArgumentException("Array of challenges may not be null");
    }
}
