package org.xbill.DNS.utils;

import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import junit.framework.TestCase;

public class HMACTest extends TestCase {
    private static test_data[] tests = new test_data[7];

    private static class test_data {
        public byte[] data;
        public byte[] digest;
        public byte[] key;

        private test_data() {
        }
    }

    static {
        for (int i = 0; i < tests.length; i++) {
            tests[i] = new test_data();
        }
        tests[0].key = base16.fromString("0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b");
        tests[0].data = "Hi There".getBytes();
        tests[0].digest = base16.fromString("9294727a3638bb1c13f48ef8158bfc9d");
        tests[1].key = "Jefe".getBytes();
        tests[1].data = "what do ya want for nothing?".getBytes();
        tests[1].digest = base16.fromString("750c783e6ab0b503eaa86e310a5db738");
        tests[2].key = base16.fromString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
        tests[2].data = new byte[50];
        for (int i2 = 0; i2 < tests[2].data.length; i2++) {
            tests[2].data[i2] = -35;
        }
        tests[2].digest = base16.fromString("56be34521d144c88dbb8c733f0e8b3f6");
        tests[3].key = base16.fromString("0102030405060708090a0b0c0d0e0f10111213141516171819");
        tests[3].data = new byte[50];
        for (int i3 = 0; i3 < tests[3].data.length; i3++) {
            tests[3].data[i3] = -51;
        }
        tests[3].digest = base16.fromString("697eaf0aca3a3aea3a75164746ffaa79");
        tests[4].key = base16.fromString("0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c");
        tests[4].data = "Test With Truncation".getBytes();
        tests[4].digest = base16.fromString("56461ef2342edc00f9bab995690efd4c");
        tests[5].key = new byte[80];
        for (int i4 = 0; i4 < tests[5].key.length; i4++) {
            tests[5].key[i4] = -86;
        }
        tests[5].data = "Test Using Larger Than Block-Size Key - Hash Key First".getBytes();
        tests[5].digest = base16.fromString("6b1ab7fe4bd7bf8f0b62e6ce61b9d0cd");
        tests[6].key = new byte[80];
        for (int i5 = 0; i5 < tests[6].key.length; i5++) {
            tests[6].key[i5] = -86;
        }
        tests[6].data = "Test Using Larger Than Block-Size Key and Larger Than One Block-Size Data".getBytes();
        tests[6].digest = base16.fromString("6f630fad67cda0ee1fb1f562db3aa53e");
    }

    public HMACTest(String name) {
        super(name);
    }

    private void do_test(int i, HMAC h) throws CloneNotSupportedException {
        h.update(tests[i].data, 0, tests[i].data.length);
        byte[] out = h.sign();
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("test=");
        stringBuffer.append(i);
        assertEquals(stringBuffer.toString(), tests[i].digest.length, out.length);
        for (int j = 0; j < out.length; j++) {
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append("test=");
            stringBuffer2.append(i);
            assertEquals(stringBuffer2.toString(), tests[i].digest[j], out[j]);
        }
        h.clear();
        h.update(tests[i].data);
        assertTrue(h.verify(tests[i].digest));
        h.clear();
        h.update(tests[i].data, 0, tests[i].data.length);
        byte[] tmp = (byte[]) tests[i].digest.clone();
        tmp[tmp.length / 2] = -85;
        assertFalse(h.verify(tmp));
    }

    public void test_ctor_digest_key() throws NoSuchAlgorithmException, CloneNotSupportedException {
        for (int i = 0; i < tests.length; i++) {
            do_test(i, new HMAC(MessageDigest.getInstance("md5"), tests[i].key));
        }
    }

    public void test_ctor_digestName_key() throws NoSuchAlgorithmException, CloneNotSupportedException {
        for (int i = 0; i < tests.length; i++) {
            do_test(i, new HMAC("md5", tests[i].key));
        }
    }

    public void test_ctor_digestName_key_invalid() {
        try {
            new HMAC("no name", new byte[0]);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }
}
