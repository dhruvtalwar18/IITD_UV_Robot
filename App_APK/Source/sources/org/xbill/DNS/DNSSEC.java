package org.xbill.DNS;

import java.io.IOException;
import java.math.BigInteger;
import java.security.GeneralSecurityException;
import java.security.KeyFactory;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.security.PrivateKey;
import java.security.PublicKey;
import java.security.Signature;
import java.security.interfaces.DSAPrivateKey;
import java.security.interfaces.DSAPublicKey;
import java.security.interfaces.RSAPrivateKey;
import java.security.interfaces.RSAPublicKey;
import java.security.spec.DSAPublicKeySpec;
import java.security.spec.RSAPublicKeySpec;
import java.util.Arrays;
import java.util.Date;
import java.util.Iterator;

public class DNSSEC {
    private static final int ASN1_INT = 2;
    private static final int ASN1_SEQ = 48;
    private static final int DSA_LEN = 20;

    public static class Algorithm {
        public static final int DH = 2;
        public static final int DSA = 3;
        public static final int DSA_NSEC3_SHA1 = 6;
        public static final int ECC = 4;
        public static final int INDIRECT = 252;
        public static final int PRIVATEDNS = 253;
        public static final int PRIVATEOID = 254;
        public static final int RSAMD5 = 1;
        public static final int RSASHA1 = 5;
        public static final int RSASHA256 = 8;
        public static final int RSASHA512 = 10;
        public static final int RSA_NSEC3_SHA1 = 7;
        private static Mnemonic algs = new Mnemonic("DNSSEC algorithm", 2);

        private Algorithm() {
        }

        static {
            algs.setMaximum(255);
            algs.setNumericAllowed(true);
            algs.add(1, "RSAMD5");
            algs.add(2, "DH");
            algs.add(3, "DSA");
            algs.add(4, "ECC");
            algs.add(5, "RSASHA1");
            algs.add(6, "DSA-NSEC3-SHA1");
            algs.add(7, "RSA-NSEC3-SHA1");
            algs.add(8, "RSASHA256");
            algs.add(10, "RSASHA512");
            algs.add(252, "INDIRECT");
            algs.add(253, "PRIVATEDNS");
            algs.add(254, "PRIVATEOID");
        }

        public static String string(int alg) {
            return algs.getText(alg);
        }

        public static int value(String s) {
            return algs.getValue(s);
        }
    }

    private DNSSEC() {
    }

    private static void digestSIG(DNSOutput out, SIGBase sig) {
        out.writeU16(sig.getTypeCovered());
        out.writeU8(sig.getAlgorithm());
        out.writeU8(sig.getLabels());
        out.writeU32(sig.getOrigTTL());
        out.writeU32(sig.getExpire().getTime() / 1000);
        out.writeU32(sig.getTimeSigned().getTime() / 1000);
        out.writeU16(sig.getFootprint());
        sig.getSigner().toWireCanonical(out);
    }

    public static byte[] digestRRset(RRSIGRecord rrsig, RRset rrset) {
        DNSOutput out = new DNSOutput();
        digestSIG(out, rrsig);
        int size = rrset.size();
        Record[] records = new Record[size];
        Iterator it = rrset.rrs();
        Name name = rrset.getName();
        Name wild = null;
        int sigLabels = rrsig.getLabels() + 1;
        if (name.labels() > sigLabels) {
            wild = name.wild(name.labels() - sigLabels);
        }
        while (it.hasNext()) {
            size--;
            records[size] = (Record) it.next();
        }
        Arrays.sort(records);
        DNSOutput header = new DNSOutput();
        if (wild != null) {
            wild.toWireCanonical(header);
        } else {
            name.toWireCanonical(header);
        }
        header.writeU16(rrset.getType());
        header.writeU16(rrset.getDClass());
        header.writeU32(rrsig.getOrigTTL());
        for (Record rdataToWireCanonical : records) {
            out.writeByteArray(header.toByteArray());
            int lengthPosition = out.current();
            out.writeU16(0);
            out.writeByteArray(rdataToWireCanonical.rdataToWireCanonical());
            out.save();
            out.jump(lengthPosition);
            out.writeU16((out.current() - lengthPosition) - 2);
            out.restore();
        }
        return out.toByteArray();
    }

    public static byte[] digestMessage(SIGRecord sig, Message msg, byte[] previous) {
        DNSOutput out = new DNSOutput();
        digestSIG(out, sig);
        if (previous != null) {
            out.writeByteArray(previous);
        }
        msg.toWire(out);
        return out.toByteArray();
    }

    public static class DNSSECException extends Exception {
        DNSSECException(String s) {
            super(s);
        }
    }

    public static class UnsupportedAlgorithmException extends DNSSECException {
        /* JADX WARNING: Illegal instructions before constructor call */
        /* Code decompiled incorrectly, please refer to instructions dump. */
        UnsupportedAlgorithmException(int r3) {
            /*
                r2 = this;
                java.lang.StringBuffer r0 = new java.lang.StringBuffer
                r0.<init>()
                java.lang.String r1 = "Unsupported algorithm: "
                r0.append(r1)
                r0.append(r3)
                java.lang.String r0 = r0.toString()
                r2.<init>(r0)
                return
            */
            throw new UnsupportedOperationException("Method not decompiled: org.xbill.DNS.DNSSEC.UnsupportedAlgorithmException.<init>(int):void");
        }
    }

    public static class MalformedKeyException extends DNSSECException {
        /* JADX WARNING: Illegal instructions before constructor call */
        /* Code decompiled incorrectly, please refer to instructions dump. */
        MalformedKeyException(org.xbill.DNS.KEYBase r3) {
            /*
                r2 = this;
                java.lang.StringBuffer r0 = new java.lang.StringBuffer
                r0.<init>()
                java.lang.String r1 = "Invalid key data: "
                r0.append(r1)
                java.lang.String r1 = r3.rdataToString()
                r0.append(r1)
                java.lang.String r0 = r0.toString()
                r2.<init>(r0)
                return
            */
            throw new UnsupportedOperationException("Method not decompiled: org.xbill.DNS.DNSSEC.MalformedKeyException.<init>(org.xbill.DNS.KEYBase):void");
        }
    }

    public static class KeyMismatchException extends DNSSECException {
        private KEYBase key;
        private SIGBase sig;

        /* JADX WARNING: Illegal instructions before constructor call */
        /* Code decompiled incorrectly, please refer to instructions dump. */
        KeyMismatchException(org.xbill.DNS.KEYBase r3, org.xbill.DNS.SIGBase r4) {
            /*
                r2 = this;
                java.lang.StringBuffer r0 = new java.lang.StringBuffer
                r0.<init>()
                java.lang.String r1 = "key "
                r0.append(r1)
                org.xbill.DNS.Name r1 = r3.getName()
                r0.append(r1)
                java.lang.String r1 = "/"
                r0.append(r1)
                int r1 = r3.getAlgorithm()
                java.lang.String r1 = org.xbill.DNS.DNSSEC.Algorithm.string(r1)
                r0.append(r1)
                java.lang.String r1 = "/"
                r0.append(r1)
                int r1 = r3.getFootprint()
                r0.append(r1)
                java.lang.String r1 = " "
                r0.append(r1)
                java.lang.String r1 = "does not match signature "
                r0.append(r1)
                org.xbill.DNS.Name r1 = r4.getSigner()
                r0.append(r1)
                java.lang.String r1 = "/"
                r0.append(r1)
                int r1 = r4.getAlgorithm()
                java.lang.String r1 = org.xbill.DNS.DNSSEC.Algorithm.string(r1)
                r0.append(r1)
                java.lang.String r1 = "/"
                r0.append(r1)
                int r1 = r4.getFootprint()
                r0.append(r1)
                java.lang.String r0 = r0.toString()
                r2.<init>(r0)
                return
            */
            throw new UnsupportedOperationException("Method not decompiled: org.xbill.DNS.DNSSEC.KeyMismatchException.<init>(org.xbill.DNS.KEYBase, org.xbill.DNS.SIGBase):void");
        }
    }

    public static class SignatureExpiredException extends DNSSECException {
        private Date now;
        private Date when;

        SignatureExpiredException(Date when2, Date now2) {
            super("signature expired");
            this.when = when2;
            this.now = now2;
        }

        public Date getExpiration() {
            return this.when;
        }

        public Date getVerifyTime() {
            return this.now;
        }
    }

    public static class SignatureNotYetValidException extends DNSSECException {
        private Date now;
        private Date when;

        SignatureNotYetValidException(Date when2, Date now2) {
            super("signature is not yet valid");
            this.when = when2;
            this.now = now2;
        }

        public Date getExpiration() {
            return this.when;
        }

        public Date getVerifyTime() {
            return this.now;
        }
    }

    public static class SignatureVerificationException extends DNSSECException {
        SignatureVerificationException() {
            super("signature verification failed");
        }
    }

    public static class IncompatibleKeyException extends IllegalArgumentException {
        IncompatibleKeyException() {
            super("incompatible keys");
        }
    }

    private static int BigIntegerLength(BigInteger i) {
        return (i.bitLength() + 7) / 8;
    }

    private static BigInteger readBigInteger(DNSInput in, int len) throws IOException {
        return new BigInteger(1, in.readByteArray(len));
    }

    private static BigInteger readBigInteger(DNSInput in) {
        return new BigInteger(1, in.readByteArray());
    }

    private static void writeBigInteger(DNSOutput out, BigInteger val) {
        byte[] b = val.toByteArray();
        if (b[0] == 0) {
            out.writeByteArray(b, 1, b.length - 1);
        } else {
            out.writeByteArray(b);
        }
    }

    private static PublicKey toRSAPublicKey(KEYBase r) throws IOException, GeneralSecurityException {
        DNSInput in = new DNSInput(r.getKey());
        int exponentLength = in.readU8();
        if (exponentLength == 0) {
            exponentLength = in.readU16();
        }
        BigInteger exponent = readBigInteger(in, exponentLength);
        return KeyFactory.getInstance("RSA").generatePublic(new RSAPublicKeySpec(readBigInteger(in), exponent));
    }

    private static PublicKey toDSAPublicKey(KEYBase r) throws IOException, GeneralSecurityException, MalformedKeyException {
        DNSInput in = new DNSInput(r.getKey());
        int t = in.readU8();
        if (t <= 8) {
            BigInteger q = readBigInteger(in, 20);
            BigInteger p = readBigInteger(in, (t * 8) + 64);
            BigInteger g = readBigInteger(in, (t * 8) + 64);
            return KeyFactory.getInstance("DSA").generatePublic(new DSAPublicKeySpec(readBigInteger(in, (t * 8) + 64), p, q, g));
        }
        throw new MalformedKeyException(r);
    }

    static PublicKey toPublicKey(KEYBase r) throws DNSSECException {
        int alg = r.getAlgorithm();
        if (alg != 1) {
            if (alg != 3) {
                if (alg != 10) {
                    switch (alg) {
                        case 5:
                        case 7:
                        case 8:
                            break;
                        case 6:
                            break;
                        default:
                            try {
                                throw new UnsupportedAlgorithmException(alg);
                            } catch (IOException e) {
                                throw new MalformedKeyException(r);
                            } catch (GeneralSecurityException e2) {
                                throw new DNSSECException(e2.toString());
                            }
                    }
                }
            }
            return toDSAPublicKey(r);
        }
        return toRSAPublicKey(r);
    }

    private static byte[] fromRSAPublicKey(RSAPublicKey key) {
        DNSOutput out = new DNSOutput();
        BigInteger exponent = key.getPublicExponent();
        BigInteger modulus = key.getModulus();
        int exponentLength = BigIntegerLength(exponent);
        if (exponentLength < 256) {
            out.writeU8(exponentLength);
        } else {
            out.writeU8(0);
            out.writeU16(exponentLength);
        }
        writeBigInteger(out, exponent);
        writeBigInteger(out, modulus);
        return out.toByteArray();
    }

    private static byte[] fromDSAPublicKey(DSAPublicKey key) {
        DNSOutput out = new DNSOutput();
        BigInteger q = key.getParams().getQ();
        BigInteger p = key.getParams().getP();
        BigInteger g = key.getParams().getG();
        BigInteger y = key.getY();
        out.writeU8((p.toByteArray().length - 64) / 8);
        writeBigInteger(out, q);
        writeBigInteger(out, p);
        writeBigInteger(out, g);
        writeBigInteger(out, y);
        return out.toByteArray();
    }

    static byte[] fromPublicKey(PublicKey key, int alg) throws DNSSECException {
        if (alg != 1) {
            if (alg != 3) {
                if (alg != 10) {
                    switch (alg) {
                        case 5:
                        case 7:
                        case 8:
                            break;
                        case 6:
                            break;
                        default:
                            throw new UnsupportedAlgorithmException(alg);
                    }
                }
            }
            if (key instanceof DSAPublicKey) {
                return fromDSAPublicKey((DSAPublicKey) key);
            }
            throw new IncompatibleKeyException();
        }
        if (key instanceof RSAPublicKey) {
            return fromRSAPublicKey((RSAPublicKey) key);
        }
        throw new IncompatibleKeyException();
    }

    private static String algString(int alg) throws UnsupportedAlgorithmException {
        if (alg == 1) {
            return "MD5withRSA";
        }
        if (alg == 3) {
            return "SHA1withDSA";
        }
        if (alg == 10) {
            return "SHA512withRSA";
        }
        switch (alg) {
            case 5:
            case 7:
                return "SHA1withRSA";
            case 6:
                return "SHA1withDSA";
            case 8:
                return "SHA256withRSA";
            default:
                throw new UnsupportedAlgorithmException(alg);
        }
    }

    private static byte[] DSASignaturefromDNS(byte[] dns) throws DNSSECException, IOException {
        if (dns.length == 41) {
            DNSInput in = new DNSInput(dns);
            DNSOutput out = new DNSOutput();
            int readU8 = in.readU8();
            byte[] r = in.readByteArray(20);
            int rlen = 20;
            if (r[0] < 0) {
                rlen = 20 + 1;
            }
            byte[] s = in.readByteArray(20);
            int slen = 20;
            if (s[0] < 0) {
                slen = 20 + 1;
            }
            out.writeU8(48);
            out.writeU8(rlen + slen + 4);
            out.writeU8(2);
            out.writeU8(rlen);
            if (rlen > 20) {
                out.writeU8(0);
            }
            out.writeByteArray(r);
            out.writeU8(2);
            out.writeU8(slen);
            if (slen > 20) {
                out.writeU8(0);
            }
            out.writeByteArray(s);
            return out.toByteArray();
        }
        throw new SignatureVerificationException();
    }

    private static byte[] DSASignaturetoDNS(byte[] key, int t) throws IOException {
        DNSInput in = new DNSInput(key);
        DNSOutput out = new DNSOutput();
        out.writeU8(t);
        if (in.readU8() == 48) {
            int readU8 = in.readU8();
            if (in.readU8() == 2) {
                int rlen = in.readU8();
                if (rlen == 21) {
                    if (in.readU8() != 0) {
                        throw new IOException();
                    }
                } else if (rlen != 20) {
                    throw new IOException();
                }
                out.writeByteArray(in.readByteArray(20));
                if (in.readU8() == 2) {
                    int slen = in.readU8();
                    if (slen == 21) {
                        if (in.readU8() != 0) {
                            throw new IOException();
                        }
                    } else if (slen != 20) {
                        throw new IOException();
                    }
                    out.writeByteArray(in.readByteArray(20));
                    return out.toByteArray();
                }
                throw new IOException();
            }
            throw new IOException();
        }
        throw new IOException();
    }

    private static void verify(PublicKey key, int alg, byte[] data, byte[] signature) throws DNSSECException {
        if (key instanceof DSAPublicKey) {
            try {
                signature = DSASignaturefromDNS(signature);
            } catch (IOException e) {
                throw new IllegalStateException();
            }
        }
        try {
            Signature s = Signature.getInstance(algString(alg));
            s.initVerify(key);
            s.update(data);
            if (!s.verify(signature)) {
                throw new SignatureVerificationException();
            }
        } catch (GeneralSecurityException e2) {
            throw new DNSSECException(e2.toString());
        }
    }

    private static boolean matches(SIGBase sig, KEYBase key) {
        return key.getAlgorithm() == sig.getAlgorithm() && key.getFootprint() == sig.getFootprint() && key.getName().equals(sig.getSigner());
    }

    public static void verify(RRset rrset, RRSIGRecord rrsig, DNSKEYRecord key) throws DNSSECException {
        if (matches(rrsig, key)) {
            Date now = new Date();
            if (now.compareTo(rrsig.getExpire()) > 0) {
                throw new SignatureExpiredException(rrsig.getExpire(), now);
            } else if (now.compareTo(rrsig.getTimeSigned()) >= 0) {
                verify(key.getPublicKey(), rrsig.getAlgorithm(), digestRRset(rrsig, rrset), rrsig.getSignature());
            } else {
                throw new SignatureNotYetValidException(rrsig.getTimeSigned(), now);
            }
        } else {
            throw new KeyMismatchException(key, rrsig);
        }
    }

    private static byte[] sign(PrivateKey privkey, PublicKey pubkey, int alg, byte[] data) throws DNSSECException {
        try {
            Signature s = Signature.getInstance(algString(alg));
            s.initSign(privkey);
            s.update(data);
            byte[] signature = s.sign();
            if (!(pubkey instanceof DSAPublicKey)) {
                return signature;
            }
            try {
                return DSASignaturetoDNS(signature, (BigIntegerLength(((DSAPublicKey) pubkey).getParams().getP()) - 64) / 8);
            } catch (IOException e) {
                throw new IllegalStateException();
            }
        } catch (GeneralSecurityException e2) {
            throw new DNSSECException(e2.toString());
        }
    }

    static void checkAlgorithm(PrivateKey key, int alg) throws UnsupportedAlgorithmException {
        if (alg != 1) {
            if (alg != 3) {
                if (alg != 10) {
                    switch (alg) {
                        case 5:
                        case 7:
                        case 8:
                            break;
                        case 6:
                            break;
                        default:
                            throw new UnsupportedAlgorithmException(alg);
                    }
                }
            }
            if (!(key instanceof DSAPrivateKey)) {
                throw new IncompatibleKeyException();
            }
            return;
        }
        if (!(key instanceof RSAPrivateKey)) {
            throw new IncompatibleKeyException();
        }
    }

    public static RRSIGRecord sign(RRset rrset, DNSKEYRecord key, PrivateKey privkey, Date inception, Date expiration) throws DNSSECException {
        PrivateKey privateKey = privkey;
        int alg = key.getAlgorithm();
        checkAlgorithm(privateKey, alg);
        RRSIGRecord rrsig = new RRSIGRecord(rrset.getName(), rrset.getDClass(), rrset.getTTL(), rrset.getType(), alg, rrset.getTTL(), expiration, inception, key.getFootprint(), key.getName(), (byte[]) null);
        rrsig.setSignature(sign(privateKey, key.getPublicKey(), alg, digestRRset(rrsig, rrset)));
        return rrsig;
    }

    static SIGRecord signMessage(Message message, SIGRecord previous, KEYRecord key, PrivateKey privkey, Date inception, Date expiration) throws DNSSECException {
        PrivateKey privateKey = privkey;
        int alg = key.getAlgorithm();
        checkAlgorithm(privateKey, alg);
        SIGRecord sig = new SIGRecord(Name.root, 255, 0, 0, alg, 0, expiration, inception, key.getFootprint(), key.getName(), (byte[]) null);
        DNSOutput out = new DNSOutput();
        digestSIG(out, sig);
        if (previous != null) {
            out.writeByteArray(previous.getSignature());
        }
        message.toWire(out);
        sig.setSignature(sign(privateKey, key.getPublicKey(), alg, out.toByteArray()));
        return sig;
    }

    static void verifyMessage(Message message, byte[] bytes, SIGRecord sig, SIGRecord previous, KEYRecord key) throws DNSSECException {
        if (matches(sig, key)) {
            Date now = new Date();
            if (now.compareTo(sig.getExpire()) > 0) {
                throw new SignatureExpiredException(sig.getExpire(), now);
            } else if (now.compareTo(sig.getTimeSigned()) >= 0) {
                DNSOutput out = new DNSOutput();
                digestSIG(out, sig);
                if (previous != null) {
                    out.writeByteArray(previous.getSignature());
                }
                Header header = (Header) message.getHeader().clone();
                header.decCount(3);
                out.writeByteArray(header.toWire());
                out.writeByteArray(bytes, 12, message.sig0start - 12);
                verify(key.getPublicKey(), sig.getAlgorithm(), out.toByteArray(), sig.getSignature());
            } else {
                throw new SignatureNotYetValidException(sig.getTimeSigned(), now);
            }
        } else {
            throw new KeyMismatchException(key, sig);
        }
    }

    static byte[] generateDS(DNSKEYRecord key, int digestid) {
        MessageDigest digest;
        DNSOutput out = new DNSOutput();
        out.writeU16(key.getFootprint());
        out.writeU8(key.getAlgorithm());
        out.writeU8(digestid);
        switch (digestid) {
            case 1:
                digest = MessageDigest.getInstance("sha-1");
                break;
            case 2:
                digest = MessageDigest.getInstance("sha-256");
                break;
            default:
                try {
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("unknown DS digest type ");
                    stringBuffer.append(digestid);
                    throw new IllegalArgumentException(stringBuffer.toString());
                } catch (NoSuchAlgorithmException e) {
                    throw new IllegalStateException("no message digest support");
                }
        }
        digest.update(key.getName().toWire());
        digest.update(key.rdataToWireCanonical());
        out.writeByteArray(digest.digest());
        return out.toByteArray();
    }
}
