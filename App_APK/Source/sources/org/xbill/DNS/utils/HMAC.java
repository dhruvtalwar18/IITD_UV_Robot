package org.xbill.DNS.utils;

import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.Arrays;

public class HMAC {
    private static final byte IPAD = 54;
    private static final byte OPAD = 92;
    private static final byte PADLEN = 64;
    MessageDigest digest;
    private byte[] ipad;
    private byte[] opad;

    private void init(byte[] key) {
        if (key.length > 64) {
            key = this.digest.digest(key);
            this.digest.reset();
        }
        this.ipad = new byte[64];
        this.opad = new byte[64];
        int i = 0;
        while (i < key.length) {
            this.ipad[i] = (byte) (54 ^ key[i]);
            this.opad[i] = (byte) (92 ^ key[i]);
            i++;
        }
        while (i < 64) {
            this.ipad[i] = IPAD;
            this.opad[i] = OPAD;
            i++;
        }
        this.digest.update(this.ipad);
    }

    public HMAC(MessageDigest digest2, byte[] key) {
        digest2.reset();
        this.digest = digest2;
        init(key);
    }

    public HMAC(String digestName, byte[] key) {
        try {
            this.digest = MessageDigest.getInstance(digestName);
            init(key);
        } catch (NoSuchAlgorithmException e) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("unknown digest algorithm ");
            stringBuffer.append(digestName);
            throw new IllegalArgumentException(stringBuffer.toString());
        }
    }

    public void update(byte[] b, int offset, int length) {
        this.digest.update(b, offset, length);
    }

    public void update(byte[] b) {
        this.digest.update(b);
    }

    public byte[] sign() {
        byte[] output = this.digest.digest();
        this.digest.reset();
        this.digest.update(this.opad);
        return this.digest.digest(output);
    }

    public boolean verify(byte[] signature) {
        return Arrays.equals(signature, sign());
    }

    public void clear() {
        this.digest.reset();
        this.digest.update(this.ipad);
    }
}
