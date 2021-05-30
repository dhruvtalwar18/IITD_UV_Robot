package org.apache.commons.httpclient.auth;

import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.List;
import java.util.StringTokenizer;
import org.apache.commons.httpclient.Credentials;
import org.apache.commons.httpclient.HttpClientError;
import org.apache.commons.httpclient.HttpMethod;
import org.apache.commons.httpclient.NameValuePair;
import org.apache.commons.httpclient.UsernamePasswordCredentials;
import org.apache.commons.httpclient.util.EncodingUtil;
import org.apache.commons.httpclient.util.ParameterFormatter;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class DigestScheme extends RFC2617Scheme {
    private static final char[] HEXADECIMAL = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
    private static final Log LOG;
    private static final String NC = "00000001";
    private static final int QOP_AUTH = 2;
    private static final int QOP_AUTH_INT = 1;
    private static final int QOP_MISSING = 0;
    static /* synthetic */ Class class$org$apache$commons$httpclient$auth$DigestScheme;
    private String cnonce;
    private boolean complete;
    private final ParameterFormatter formatter;
    private int qopVariant;

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$auth$DigestScheme == null) {
            cls = class$("org.apache.commons.httpclient.auth.DigestScheme");
            class$org$apache$commons$httpclient$auth$DigestScheme = cls;
        } else {
            cls = class$org$apache$commons$httpclient$auth$DigestScheme;
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

    public DigestScheme() {
        this.qopVariant = 0;
        this.complete = false;
        this.formatter = new ParameterFormatter();
    }

    public String getID() {
        String id = getRealm();
        String nonce = getParameter("nonce");
        if (nonce == null) {
            return id;
        }
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append(id);
        stringBuffer.append("-");
        stringBuffer.append(nonce);
        return stringBuffer.toString();
    }

    public DigestScheme(String challenge) throws MalformedChallengeException {
        this();
        processChallenge(challenge);
    }

    public void processChallenge(String challenge) throws MalformedChallengeException {
        super.processChallenge(challenge);
        if (getParameter("realm") == null) {
            throw new MalformedChallengeException("missing realm in challange");
        } else if (getParameter("nonce") != null) {
            boolean unsupportedQop = false;
            String qop = getParameter("qop");
            if (qop != null) {
                StringTokenizer tok = new StringTokenizer(qop, ",");
                while (true) {
                    if (!tok.hasMoreTokens()) {
                        break;
                    }
                    String variant = tok.nextToken().trim();
                    if (variant.equals("auth")) {
                        this.qopVariant = 2;
                        break;
                    } else if (variant.equals("auth-int")) {
                        this.qopVariant = 1;
                    } else {
                        unsupportedQop = true;
                        Log log = LOG;
                        StringBuffer stringBuffer = new StringBuffer();
                        stringBuffer.append("Unsupported qop detected: ");
                        stringBuffer.append(variant);
                        log.warn(stringBuffer.toString());
                    }
                }
            }
            if (!unsupportedQop || this.qopVariant != 0) {
                this.cnonce = createCnonce();
                this.complete = true;
                return;
            }
            throw new MalformedChallengeException("None of the qop methods is supported");
        } else {
            throw new MalformedChallengeException("missing nonce in challange");
        }
    }

    public boolean isComplete() {
        if ("true".equalsIgnoreCase(getParameter("stale"))) {
            return false;
        }
        return this.complete;
    }

    public String getSchemeName() {
        return "digest";
    }

    public boolean isConnectionBased() {
        return false;
    }

    public String authenticate(Credentials credentials, String method, String uri) throws AuthenticationException {
        LOG.trace("enter DigestScheme.authenticate(Credentials, String, String)");
        try {
            UsernamePasswordCredentials usernamepassword = (UsernamePasswordCredentials) credentials;
            getParameters().put("methodname", method);
            getParameters().put("uri", uri);
            String digest = createDigest(usernamepassword.getUserName(), usernamepassword.getPassword());
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Digest ");
            stringBuffer.append(createDigestHeader(usernamepassword.getUserName(), digest));
            return stringBuffer.toString();
        } catch (ClassCastException e) {
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append("Credentials cannot be used for digest authentication: ");
            stringBuffer2.append(credentials.getClass().getName());
            throw new InvalidCredentialsException(stringBuffer2.toString());
        }
    }

    public String authenticate(Credentials credentials, HttpMethod method) throws AuthenticationException {
        LOG.trace("enter DigestScheme.authenticate(Credentials, HttpMethod)");
        try {
            UsernamePasswordCredentials usernamepassword = (UsernamePasswordCredentials) credentials;
            getParameters().put("methodname", method.getName());
            StringBuffer buffer = new StringBuffer(method.getPath());
            String query = method.getQueryString();
            if (query != null) {
                if (query.indexOf("?") != 0) {
                    buffer.append("?");
                }
                buffer.append(method.getQueryString());
            }
            getParameters().put("uri", buffer.toString());
            if (getParameter("charset") == null) {
                getParameters().put("charset", method.getParams().getCredentialCharset());
            }
            String digest = createDigest(usernamepassword.getUserName(), usernamepassword.getPassword());
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Digest ");
            stringBuffer.append(createDigestHeader(usernamepassword.getUserName(), digest));
            return stringBuffer.toString();
        } catch (ClassCastException e) {
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append("Credentials cannot be used for digest authentication: ");
            stringBuffer2.append(credentials.getClass().getName());
            throw new InvalidCredentialsException(stringBuffer2.toString());
        }
    }

    private String createDigest(String uname, String pwd) throws AuthenticationException {
        String qopOption;
        LOG.trace("enter DigestScheme.createDigest(String, String, Map)");
        String uri = getParameter("uri");
        String realm = getParameter("realm");
        String nonce = getParameter("nonce");
        String qop = getParameter("qop");
        String method = getParameter("methodname");
        String algorithm = getParameter("algorithm");
        if (algorithm == null) {
            algorithm = "MD5";
        }
        String algorithm2 = algorithm;
        String charset = getParameter("charset");
        if (charset == null) {
            charset = "ISO-8859-1";
        }
        String charset2 = charset;
        if (this.qopVariant != 1) {
            try {
                MessageDigest md5Helper = MessageDigest.getInstance("MD5");
                StringBuffer tmp = new StringBuffer(uname.length() + realm.length() + pwd.length() + 2);
                tmp.append(uname);
                tmp.append(':');
                tmp.append(realm);
                tmp.append(':');
                tmp.append(pwd);
                String a1 = tmp.toString();
                if (algorithm2.equals("MD5-sess")) {
                    String tmp2 = encode(md5Helper.digest(EncodingUtil.getBytes(a1, charset2)));
                    Object obj = "MD5";
                    StringBuffer tmp3 = new StringBuffer(tmp2.length() + nonce.length() + this.cnonce.length() + 2);
                    tmp3.append(tmp2);
                    tmp3.append(':');
                    tmp3.append(nonce);
                    tmp3.append(':');
                    tmp3.append(this.cnonce);
                    a1 = tmp3.toString();
                } else {
                    String digAlg = "MD5";
                    if (!algorithm2.equals("MD5")) {
                        Log log = LOG;
                        StringBuffer stringBuffer = new StringBuffer();
                        stringBuffer.append("Unhandled algorithm ");
                        stringBuffer.append(algorithm2);
                        stringBuffer.append(" requested");
                        log.warn(stringBuffer.toString());
                    }
                }
                String md5a1 = encode(md5Helper.digest(EncodingUtil.getBytes(a1, charset2)));
                String a2 = null;
                String str = realm;
                if (this.qopVariant == 1) {
                    LOG.error("Unhandled qop auth-int");
                } else {
                    StringBuffer stringBuffer2 = new StringBuffer();
                    stringBuffer2.append(method);
                    stringBuffer2.append(":");
                    stringBuffer2.append(uri);
                    a2 = stringBuffer2.toString();
                }
                String md5a2 = encode(md5Helper.digest(EncodingUtil.getAsciiBytes(a2)));
                if (this.qopVariant == 0) {
                    String str2 = uri;
                    LOG.debug("Using null qop method");
                    StringBuffer tmp22 = new StringBuffer(md5a1.length() + nonce.length() + md5a2.length());
                    tmp22.append(md5a1);
                    tmp22.append(':');
                    tmp22.append(nonce);
                    tmp22.append(':');
                    tmp22.append(md5a2);
                    qopOption = tmp22.toString();
                    String str3 = qop;
                    String str4 = method;
                } else {
                    if (LOG.isDebugEnabled()) {
                        Log log2 = LOG;
                        StringBuffer stringBuffer3 = new StringBuffer();
                        String str5 = method;
                        stringBuffer3.append("Using qop method ");
                        stringBuffer3.append(qop);
                        log2.debug(stringBuffer3.toString());
                    }
                    String qopOption2 = getQopVariantString();
                    String str6 = qop;
                    StringBuffer tmp23 = new StringBuffer(md5a1.length() + nonce.length() + NC.length() + this.cnonce.length() + qopOption2.length() + md5a2.length() + 5);
                    tmp23.append(md5a1);
                    tmp23.append(':');
                    tmp23.append(nonce);
                    tmp23.append(':');
                    tmp23.append(NC);
                    tmp23.append(':');
                    tmp23.append(this.cnonce);
                    tmp23.append(':');
                    tmp23.append(qopOption2);
                    tmp23.append(':');
                    tmp23.append(md5a2);
                    qopOption = tmp23.toString();
                }
                return encode(md5Helper.digest(EncodingUtil.getAsciiBytes(qopOption)));
            } catch (Exception e) {
                String str7 = uname;
                String str8 = pwd;
                Object obj2 = "MD5";
                String str9 = uri;
                String str10 = realm;
                String str11 = qop;
                String str12 = method;
                throw new AuthenticationException("Unsupported algorithm in HTTP Digest authentication: MD5");
            }
        } else {
            String str13 = uname;
            String str14 = pwd;
            Object obj3 = "MD5";
            String str15 = uri;
            String str16 = realm;
            String str17 = qop;
            String str18 = method;
            LOG.warn("qop=auth-int is not supported");
            throw new AuthenticationException("Unsupported qop in HTTP Digest authentication");
        }
    }

    private String createDigestHeader(String uname, String digest) throws AuthenticationException {
        LOG.trace("enter DigestScheme.createDigestHeader(String, Map, String)");
        String uri = getParameter("uri");
        String realm = getParameter("realm");
        String nonce = getParameter("nonce");
        String opaque = getParameter("opaque");
        String algorithm = getParameter("algorithm");
        List params = new ArrayList(20);
        params.add(new NameValuePair("username", uname));
        params.add(new NameValuePair("realm", realm));
        params.add(new NameValuePair("nonce", nonce));
        params.add(new NameValuePair("uri", uri));
        params.add(new NameValuePair("response", digest));
        if (this.qopVariant != 0) {
            params.add(new NameValuePair("qop", getQopVariantString()));
            params.add(new NameValuePair("nc", NC));
            params.add(new NameValuePair("cnonce", this.cnonce));
        }
        if (algorithm != null) {
            params.add(new NameValuePair("algorithm", algorithm));
        }
        if (opaque != null) {
            params.add(new NameValuePair("opaque", opaque));
        }
        StringBuffer buffer = new StringBuffer();
        for (int i = 0; i < params.size(); i++) {
            NameValuePair param = (NameValuePair) params.get(i);
            if (i > 0) {
                buffer.append(", ");
            }
            boolean z = true;
            boolean noQuotes = "nc".equals(param.getName()) || "qop".equals(param.getName());
            ParameterFormatter parameterFormatter = this.formatter;
            if (noQuotes) {
                z = false;
            }
            parameterFormatter.setAlwaysUseQuotes(z);
            this.formatter.format(buffer, param);
        }
        return buffer.toString();
    }

    private String getQopVariantString() {
        if (this.qopVariant == 1) {
            return "auth-int";
        }
        return "auth";
    }

    private static String encode(byte[] binaryData) {
        LOG.trace("enter DigestScheme.encode(byte[])");
        if (binaryData.length != 16) {
            return null;
        }
        char[] buffer = new char[32];
        for (int i = 0; i < 16; i++) {
            buffer[i * 2] = HEXADECIMAL[(binaryData[i] & 240) >> 4];
            buffer[(i * 2) + 1] = HEXADECIMAL[binaryData[i] & 15];
        }
        return new String(buffer);
    }

    public static String createCnonce() {
        LOG.trace("enter DigestScheme.createCnonce()");
        try {
            return encode(MessageDigest.getInstance("MD5").digest(EncodingUtil.getAsciiBytes(Long.toString(System.currentTimeMillis()))));
        } catch (NoSuchAlgorithmException e) {
            throw new HttpClientError("Unsupported algorithm in HTTP Digest authentication: MD5");
        }
    }
}
