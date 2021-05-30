package org.apache.commons.httpclient.auth;

import com.google.common.base.Ascii;
import com.google.common.primitives.SignedBytes;
import java.security.InvalidKeyException;
import java.security.NoSuchAlgorithmException;
import javax.crypto.BadPaddingException;
import javax.crypto.Cipher;
import javax.crypto.IllegalBlockSizeException;
import javax.crypto.NoSuchPaddingException;
import javax.crypto.spec.SecretKeySpec;
import org.apache.commons.codec.binary.Base64;
import org.apache.commons.httpclient.util.EncodingUtil;
import rocon_app_manager_msgs.ErrorCodes;
import sensor_msgs.NavSatStatus;

final class NTLM {
    public static final String DEFAULT_CHARSET = "ASCII";
    private String credentialCharset = DEFAULT_CHARSET;
    private int currentPosition = 0;
    private byte[] currentResponse;

    NTLM() {
    }

    public final String getResponseFor(String message, String username, String password, String host, String domain) throws AuthenticationException {
        if (message == null || message.trim().equals("")) {
            return getType1Message(host, domain);
        }
        return getType3Message(username, password, host, domain, parseType2Message(message));
    }

    private Cipher getCipher(byte[] key) throws AuthenticationException {
        try {
            Cipher ecipher = Cipher.getInstance("DES/ECB/NoPadding");
            ecipher.init(1, new SecretKeySpec(setupKey(key), "DES"));
            return ecipher;
        } catch (NoSuchAlgorithmException e) {
            throw new AuthenticationException("DES encryption is not available.", e);
        } catch (InvalidKeyException e2) {
            throw new AuthenticationException("Invalid key for DES encryption.", e2);
        } catch (NoSuchPaddingException e3) {
            throw new AuthenticationException("NoPadding option for DES is not available.", e3);
        }
    }

    private byte[] setupKey(byte[] key56) {
        byte[] key = {(byte) ((key56[0] >> 1) & 255), (byte) ((((key56[0] & 1) << 6) | (((key56[1] & NavSatStatus.STATUS_NO_FIX) >> 2) & 255)) & 255), (byte) ((((key56[1] & 3) << 5) | (((key56[2] & NavSatStatus.STATUS_NO_FIX) >> 3) & 255)) & 255), (byte) ((((key56[2] & 7) << 4) | (((key56[3] & NavSatStatus.STATUS_NO_FIX) >> 4) & 255)) & 255), (byte) ((((key56[3] & Ascii.SI) << 3) | (((key56[4] & NavSatStatus.STATUS_NO_FIX) >> 5) & 255)) & 255), (byte) ((((key56[4] & 31) << 2) | (((key56[5] & NavSatStatus.STATUS_NO_FIX) >> 6) & 255)) & 255), (byte) ((((key56[5] & 63) << 1) | (((key56[6] & NavSatStatus.STATUS_NO_FIX) >> 7) & 255)) & 255), (byte) (key56[6] & Ascii.DEL)};
        for (int i = 0; i < key.length; i++) {
            key[i] = (byte) (key[i] << 1);
        }
        return key;
    }

    private byte[] encrypt(byte[] key, byte[] bytes) throws AuthenticationException {
        try {
            return getCipher(key).doFinal(bytes);
        } catch (IllegalBlockSizeException e) {
            throw new AuthenticationException("Invalid block size for DES encryption.", e);
        } catch (BadPaddingException e2) {
            throw new AuthenticationException("Data not padded correctly for DES encryption.", e2);
        }
    }

    private void prepareResponse(int length) {
        this.currentResponse = new byte[length];
        this.currentPosition = 0;
    }

    private void addByte(byte b) {
        this.currentResponse[this.currentPosition] = b;
        this.currentPosition++;
    }

    private void addBytes(byte[] bytes) {
        for (byte b : bytes) {
            this.currentResponse[this.currentPosition] = b;
            this.currentPosition++;
        }
    }

    private String getResponse() {
        byte[] tmp;
        if (this.currentResponse.length > this.currentPosition) {
            tmp = new byte[this.currentPosition];
            for (int i = 0; i < this.currentPosition; i++) {
                tmp[i] = this.currentResponse[i];
            }
        } else {
            tmp = this.currentResponse;
        }
        return EncodingUtil.getAsciiString(Base64.encodeBase64(tmp));
    }

    public String getType1Message(String host, String domain) {
        String host2 = host.toUpperCase();
        String domain2 = domain.toUpperCase();
        byte[] hostBytes = EncodingUtil.getBytes(host2, DEFAULT_CHARSET);
        byte[] domainBytes = EncodingUtil.getBytes(domain2, DEFAULT_CHARSET);
        prepareResponse(hostBytes.length + 32 + domainBytes.length);
        addBytes(EncodingUtil.getBytes("NTLMSSP", DEFAULT_CHARSET));
        addByte((byte) 0);
        addByte((byte) 1);
        addByte((byte) 0);
        addByte((byte) 0);
        addByte((byte) 0);
        addByte((byte) 6);
        addByte((byte) 82);
        addByte((byte) 0);
        addByte((byte) 0);
        byte[] domLen = convertShort(domainBytes.length);
        addByte(domLen[0]);
        addByte(domLen[1]);
        addByte(domLen[0]);
        addByte(domLen[1]);
        byte[] domOff = convertShort(hostBytes.length + 32);
        addByte(domOff[0]);
        addByte(domOff[1]);
        addByte((byte) 0);
        addByte((byte) 0);
        byte[] hostLen = convertShort(hostBytes.length);
        addByte(hostLen[0]);
        addByte(hostLen[1]);
        addByte(hostLen[0]);
        addByte(hostLen[1]);
        byte[] hostOff = convertShort(32);
        addByte(hostOff[0]);
        addByte(hostOff[1]);
        addByte((byte) 0);
        addByte((byte) 0);
        addBytes(hostBytes);
        addBytes(domainBytes);
        return getResponse();
    }

    public byte[] parseType2Message(String message) {
        byte[] msg = Base64.decodeBase64(EncodingUtil.getBytes(message, DEFAULT_CHARSET));
        byte[] nonce = new byte[8];
        for (int i = 0; i < 8; i++) {
            nonce[i] = msg[i + 24];
        }
        return nonce;
    }

    public String getType3Message(String user, String password, String host, String domain, byte[] nonce) throws AuthenticationException {
        String domain2 = domain.toUpperCase();
        String host2 = host.toUpperCase();
        String user2 = user.toUpperCase();
        byte[] domainBytes = EncodingUtil.getBytes(domain2, DEFAULT_CHARSET);
        byte[] hostBytes = EncodingUtil.getBytes(host2, DEFAULT_CHARSET);
        byte[] userBytes = EncodingUtil.getBytes(user2, this.credentialCharset);
        int domainLen = domainBytes.length;
        int hostLen = hostBytes.length;
        int userLen = userBytes.length;
        int finalLength = 0 + 64 + 24 + domainLen + userLen + hostLen;
        prepareResponse(finalLength);
        addBytes(EncodingUtil.getBytes("NTLMSSP", DEFAULT_CHARSET));
        addByte((byte) 0);
        addByte((byte) 3);
        addByte((byte) 0);
        addByte((byte) 0);
        addByte((byte) 0);
        addBytes(convertShort(24));
        addBytes(convertShort(24));
        addBytes(convertShort(finalLength - 24));
        addByte((byte) 0);
        addByte((byte) 0);
        addBytes(convertShort(0));
        addBytes(convertShort(0));
        addBytes(convertShort(finalLength));
        addByte((byte) 0);
        addByte((byte) 0);
        addBytes(convertShort(domainLen));
        addBytes(convertShort(domainLen));
        addBytes(convertShort(64));
        addByte((byte) 0);
        addByte((byte) 0);
        addBytes(convertShort(userLen));
        addBytes(convertShort(userLen));
        addBytes(convertShort(domainLen + 64));
        addByte((byte) 0);
        addByte((byte) 0);
        addBytes(convertShort(hostLen));
        addBytes(convertShort(hostLen));
        addBytes(convertShort(domainLen + 64 + userLen));
        for (int i = 0; i < 6; i++) {
            addByte((byte) 0);
        }
        addBytes(convertShort(finalLength));
        addByte((byte) 0);
        addByte((byte) 0);
        addByte((byte) 6);
        addByte((byte) 82);
        addByte((byte) 0);
        addByte((byte) 0);
        addBytes(domainBytes);
        addBytes(userBytes);
        addBytes(hostBytes);
        addBytes(hashPassword(password, nonce));
        return getResponse();
    }

    private byte[] hashPassword(String password, byte[] nonce) throws AuthenticationException {
        byte[] passw = EncodingUtil.getBytes(password.toUpperCase(), this.credentialCharset);
        byte[] lmPw1 = new byte[7];
        byte[] lmPw2 = new byte[7];
        int len = passw.length;
        if (len > 7) {
            len = 7;
        }
        int idx = 0;
        while (idx < len) {
            lmPw1[idx] = passw[idx];
            idx++;
        }
        while (idx < 7) {
            lmPw1[idx] = 0;
            idx++;
        }
        int len2 = passw.length;
        if (len2 > 14) {
            len2 = 14;
        }
        int idx2 = 7;
        while (idx2 < len2) {
            lmPw2[idx2 - 7] = passw[idx2];
            idx2++;
        }
        while (idx2 < 14) {
            lmPw2[idx2 - 7] = 0;
            idx2++;
        }
        byte[] magic = {75, 71, 83, ErrorCodes.NOT_CURRENT_REMOTE_CONTROLLER, SignedBytes.MAX_POWER_OF_TWO, ErrorCodes.INVITING_CONTROLLER_BLACKLISTED, ErrorCodes.CLIENT_CONNECTION_DISRUPTED, 37};
        byte[] lmHpw1 = encrypt(lmPw1, magic);
        byte[] lmHpw2 = encrypt(lmPw2, magic);
        byte[] lmHpw = new byte[21];
        for (int i = 0; i < lmHpw1.length; i++) {
            lmHpw[i] = lmHpw1[i];
        }
        for (int i2 = 0; i2 < lmHpw2.length; i2++) {
            lmHpw[i2 + 8] = lmHpw2[i2];
        }
        for (int i3 = 0; i3 < 5; i3++) {
            lmHpw[i3 + 16] = 0;
        }
        byte[] lmResp = new byte[24];
        calcResp(lmHpw, nonce, lmResp);
        return lmResp;
    }

    private void calcResp(byte[] keys, byte[] plaintext, byte[] results) throws AuthenticationException {
        byte[] keys1 = new byte[7];
        byte[] keys2 = new byte[7];
        byte[] keys3 = new byte[7];
        for (int i = 0; i < 7; i++) {
            keys1[i] = keys[i];
        }
        for (int i2 = 0; i2 < 7; i2++) {
            keys2[i2] = keys[i2 + 7];
        }
        for (int i3 = 0; i3 < 7; i3++) {
            keys3[i3] = keys[i3 + 14];
        }
        byte[] results1 = encrypt(keys1, plaintext);
        byte[] results2 = encrypt(keys2, plaintext);
        byte[] results3 = encrypt(keys3, plaintext);
        for (int i4 = 0; i4 < 8; i4++) {
            results[i4] = results1[i4];
        }
        for (int i5 = 0; i5 < 8; i5++) {
            results[i5 + 8] = results2[i5];
        }
        for (int i6 = 0; i6 < 8; i6++) {
            results[i6 + 16] = results3[i6];
        }
    }

    private byte[] convertShort(int num) {
        byte[] val = new byte[2];
        String hex = Integer.toString(num, 16);
        while (hex.length() < 4) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("0");
            stringBuffer.append(hex);
            hex = stringBuffer.toString();
        }
        String low = hex.substring(2, 4);
        String high = hex.substring(0, 2);
        val[0] = (byte) Integer.parseInt(low, 16);
        val[1] = (byte) Integer.parseInt(high, 16);
        return val;
    }

    public String getCredentialCharset() {
        return this.credentialCharset;
    }

    public void setCredentialCharset(String credentialCharset2) {
        this.credentialCharset = credentialCharset2;
    }
}
