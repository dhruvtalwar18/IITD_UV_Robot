package org.apache.commons.codec.language;

import org.apache.commons.codec.EncoderException;
import org.apache.commons.codec.StringEncoder;

public class Metaphone implements StringEncoder {
    private String frontv = "EIY";
    private int maxCodeLen = 4;
    private String varson = "CSPTG";
    private String vowels = "AEIOU";

    /* JADX WARNING: Code restructure failed: missing block: B:162:0x02b0, code lost:
        r6 = r6 + 1;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public java.lang.String metaphone(java.lang.String r16) {
        /*
            r15 = this;
            r0 = r15
            r1 = 0
            if (r16 == 0) goto L_0x02cc
            int r2 = r16.length()
            if (r2 != 0) goto L_0x000c
            goto L_0x02cc
        L_0x000c:
            int r2 = r16.length()
            r3 = 1
            if (r2 != r3) goto L_0x0018
            java.lang.String r2 = r16.toUpperCase()
            return r2
        L_0x0018:
            java.lang.String r2 = r16.toUpperCase()
            char[] r2 = r2.toCharArray()
            java.lang.StringBuffer r4 = new java.lang.StringBuffer
            r5 = 40
            r4.<init>(r5)
            java.lang.StringBuffer r5 = new java.lang.StringBuffer
            r6 = 10
            r5.<init>(r6)
            r6 = 0
            char r7 = r2[r6]
            r8 = 65
            r9 = 71
            r10 = 72
            r11 = 83
            r12 = 75
            if (r7 == r8) goto L_0x0081
            if (r7 == r9) goto L_0x0071
            if (r7 == r12) goto L_0x0071
            r8 = 80
            if (r7 == r8) goto L_0x0071
            switch(r7) {
                case 87: goto L_0x0052;
                case 88: goto L_0x004c;
                default: goto L_0x0048;
            }
        L_0x0048:
            r4.append(r2)
            goto L_0x0091
        L_0x004c:
            r2[r6] = r11
            r4.append(r2)
            goto L_0x0091
        L_0x0052:
            char r7 = r2[r3]
            r8 = 82
            if (r7 != r8) goto L_0x005e
            int r7 = r2.length
            int r7 = r7 - r3
            r4.append(r2, r3, r7)
            goto L_0x0091
        L_0x005e:
            char r7 = r2[r3]
            if (r7 != r10) goto L_0x006d
            int r7 = r2.length
            int r7 = r7 - r3
            r4.append(r2, r3, r7)
            r7 = 87
            r4.setCharAt(r6, r7)
            goto L_0x0091
        L_0x006d:
            r4.append(r2)
            goto L_0x0091
        L_0x0071:
            char r7 = r2[r3]
            r8 = 78
            if (r7 != r8) goto L_0x007d
            int r7 = r2.length
            int r7 = r7 - r3
            r4.append(r2, r3, r7)
            goto L_0x0091
        L_0x007d:
            r4.append(r2)
            goto L_0x0091
        L_0x0081:
            char r7 = r2[r3]
            r8 = 69
            if (r7 != r8) goto L_0x008d
            int r7 = r2.length
            int r7 = r7 - r3
            r4.append(r2, r3, r7)
            goto L_0x0091
        L_0x008d:
            r4.append(r2)
        L_0x0091:
            int r7 = r4.length()
        L_0x0096:
            int r8 = r5.length()
            int r13 = r15.getMaxCodeLen()
            if (r8 >= r13) goto L_0x02c7
            if (r6 >= r7) goto L_0x02c7
            char r8 = r4.charAt(r6)
            r13 = 67
            if (r8 == r13) goto L_0x00b6
            boolean r14 = r15.isPreviousChar(r4, r6, r8)
            if (r14 == 0) goto L_0x00b6
            int r6 = r6 + 1
            r13 = 71
            goto L_0x02b2
        L_0x00b6:
            r3 = 84
            r14 = 70
            r9 = 88
            switch(r8) {
                case 65: goto L_0x02a9;
                case 66: goto L_0x0294;
                case 67: goto L_0x0224;
                case 68: goto L_0x01f6;
                case 69: goto L_0x02a9;
                case 70: goto L_0x01f2;
                case 71: goto L_0x018a;
                case 72: goto L_0x0164;
                case 73: goto L_0x02a9;
                case 74: goto L_0x01f2;
                case 75: goto L_0x0152;
                case 76: goto L_0x01f2;
                case 77: goto L_0x01f2;
                case 78: goto L_0x01f2;
                case 79: goto L_0x02a9;
                case 80: goto L_0x0142;
                case 81: goto L_0x013d;
                case 82: goto L_0x01f2;
                case 83: goto L_0x011a;
                case 84: goto L_0x00e6;
                case 85: goto L_0x02a9;
                case 86: goto L_0x00e1;
                case 87: goto L_0x00ce;
                case 88: goto L_0x00c6;
                case 89: goto L_0x00ce;
                case 90: goto L_0x00c1;
                default: goto L_0x00bf;
            }
        L_0x00bf:
            goto L_0x01ee
        L_0x00c1:
            r5.append(r11)
            goto L_0x01ee
        L_0x00c6:
            r5.append(r12)
            r5.append(r11)
            goto L_0x01ee
        L_0x00ce:
            boolean r3 = r15.isLastChar(r7, r6)
            if (r3 != 0) goto L_0x01ee
            int r3 = r6 + 1
            boolean r3 = r15.isVowel(r4, r3)
            if (r3 == 0) goto L_0x01ee
            r5.append(r8)
            goto L_0x01ee
        L_0x00e1:
            r5.append(r14)
            goto L_0x01ee
        L_0x00e6:
            java.lang.String r13 = "TIA"
            boolean r13 = r15.regionMatch(r4, r6, r13)
            if (r13 != 0) goto L_0x0115
            java.lang.String r13 = "TIO"
            boolean r13 = r15.regionMatch(r4, r6, r13)
            if (r13 == 0) goto L_0x00f7
            goto L_0x0115
        L_0x00f7:
            java.lang.String r9 = "TCH"
            boolean r9 = r15.regionMatch(r4, r6, r9)
            if (r9 == 0) goto L_0x0101
            goto L_0x01ee
        L_0x0101:
            java.lang.String r9 = "TH"
            boolean r9 = r15.regionMatch(r4, r6, r9)
            if (r9 == 0) goto L_0x0110
            r3 = 48
            r5.append(r3)
            goto L_0x01ee
        L_0x0110:
            r5.append(r3)
            goto L_0x01ee
        L_0x0115:
            r5.append(r9)
            goto L_0x01ee
        L_0x011a:
            java.lang.String r3 = "SH"
            boolean r3 = r15.regionMatch(r4, r6, r3)
            if (r3 != 0) goto L_0x0138
            java.lang.String r3 = "SIO"
            boolean r3 = r15.regionMatch(r4, r6, r3)
            if (r3 != 0) goto L_0x0138
            java.lang.String r3 = "SIA"
            boolean r3 = r15.regionMatch(r4, r6, r3)
            if (r3 == 0) goto L_0x0133
            goto L_0x0138
        L_0x0133:
            r5.append(r11)
            goto L_0x01ee
        L_0x0138:
            r5.append(r9)
            goto L_0x01ee
        L_0x013d:
            r5.append(r12)
            goto L_0x01ee
        L_0x0142:
            boolean r3 = r15.isNextChar(r4, r6, r10)
            if (r3 == 0) goto L_0x014d
            r5.append(r14)
            goto L_0x01ee
        L_0x014d:
            r5.append(r8)
            goto L_0x01ee
        L_0x0152:
            if (r6 <= 0) goto L_0x015f
            boolean r3 = r15.isPreviousChar(r4, r6, r13)
            if (r3 != 0) goto L_0x01ee
            r5.append(r8)
            goto L_0x01ee
        L_0x015f:
            r5.append(r8)
            goto L_0x01ee
        L_0x0164:
            boolean r3 = r15.isLastChar(r7, r6)
            if (r3 == 0) goto L_0x016c
            goto L_0x01ee
        L_0x016c:
            if (r6 <= 0) goto L_0x017e
            java.lang.String r3 = r0.varson
            int r9 = r6 + -1
            char r9 = r4.charAt(r9)
            int r3 = r3.indexOf(r9)
            if (r3 < 0) goto L_0x017e
            goto L_0x01ee
        L_0x017e:
            int r3 = r6 + 1
            boolean r3 = r15.isVowel(r4, r3)
            if (r3 == 0) goto L_0x01ee
            r5.append(r10)
            goto L_0x01ee
        L_0x018a:
            int r3 = r6 + 1
            boolean r3 = r15.isLastChar(r7, r3)
            if (r3 == 0) goto L_0x0199
            boolean r3 = r15.isNextChar(r4, r6, r10)
            if (r3 == 0) goto L_0x0199
            goto L_0x01ee
        L_0x0199:
            int r3 = r6 + 1
            boolean r3 = r15.isLastChar(r7, r3)
            if (r3 != 0) goto L_0x01b0
            boolean r3 = r15.isNextChar(r4, r6, r10)
            if (r3 == 0) goto L_0x01b0
            int r3 = r6 + 2
            boolean r3 = r15.isVowel(r4, r3)
            if (r3 != 0) goto L_0x01b0
            goto L_0x01ee
        L_0x01b0:
            if (r6 <= 0) goto L_0x01c3
            java.lang.String r3 = "GN"
            boolean r3 = r15.regionMatch(r4, r6, r3)
            if (r3 != 0) goto L_0x01ee
            java.lang.String r3 = "GNED"
            boolean r3 = r15.regionMatch(r4, r6, r3)
            if (r3 == 0) goto L_0x01c3
            goto L_0x01ee
        L_0x01c3:
            r3 = 71
            boolean r9 = r15.isPreviousChar(r4, r6, r3)
            if (r9 == 0) goto L_0x01cd
            r1 = 1
            goto L_0x01ce
        L_0x01cd:
            r1 = 0
        L_0x01ce:
            boolean r3 = r15.isLastChar(r7, r6)
            if (r3 != 0) goto L_0x01ea
            java.lang.String r3 = r0.frontv
            int r9 = r6 + 1
            char r9 = r4.charAt(r9)
            int r3 = r3.indexOf(r9)
            if (r3 < 0) goto L_0x01ea
            if (r1 != 0) goto L_0x01ea
            r3 = 74
            r5.append(r3)
            goto L_0x01ee
        L_0x01ea:
            r5.append(r12)
        L_0x01ee:
            r13 = 71
            goto L_0x02b0
        L_0x01f2:
            r5.append(r8)
            goto L_0x01ee
        L_0x01f6:
            int r9 = r6 + 1
            boolean r9 = r15.isLastChar(r7, r9)
            if (r9 != 0) goto L_0x021d
            r13 = 71
            boolean r9 = r15.isNextChar(r4, r6, r13)
            if (r9 == 0) goto L_0x021f
            java.lang.String r9 = r0.frontv
            int r14 = r6 + 2
            char r14 = r4.charAt(r14)
            int r9 = r9.indexOf(r14)
            if (r9 < 0) goto L_0x021f
            r3 = 74
            r5.append(r3)
            int r6 = r6 + 2
            goto L_0x02b0
        L_0x021d:
            r13 = 71
        L_0x021f:
            r5.append(r3)
            goto L_0x02b0
        L_0x0224:
            r13 = 71
            boolean r3 = r15.isPreviousChar(r4, r6, r11)
            if (r3 == 0) goto L_0x0242
            boolean r3 = r15.isLastChar(r7, r6)
            if (r3 != 0) goto L_0x0242
            java.lang.String r3 = r0.frontv
            int r14 = r6 + 1
            char r14 = r4.charAt(r14)
            int r3 = r3.indexOf(r14)
            if (r3 < 0) goto L_0x0242
            goto L_0x02b0
        L_0x0242:
            java.lang.String r3 = "CIA"
            boolean r3 = r15.regionMatch(r4, r6, r3)
            if (r3 == 0) goto L_0x024e
            r5.append(r9)
            goto L_0x02b0
        L_0x024e:
            boolean r3 = r15.isLastChar(r7, r6)
            if (r3 != 0) goto L_0x0266
            java.lang.String r3 = r0.frontv
            int r14 = r6 + 1
            char r14 = r4.charAt(r14)
            int r3 = r3.indexOf(r14)
            if (r3 < 0) goto L_0x0266
            r5.append(r11)
            goto L_0x02b0
        L_0x0266:
            boolean r3 = r15.isPreviousChar(r4, r6, r11)
            if (r3 == 0) goto L_0x0276
            boolean r3 = r15.isNextChar(r4, r6, r10)
            if (r3 == 0) goto L_0x0276
            r5.append(r12)
            goto L_0x02b0
        L_0x0276:
            boolean r3 = r15.isNextChar(r4, r6, r10)
            if (r3 == 0) goto L_0x0290
            if (r6 != 0) goto L_0x028c
            r3 = 3
            if (r7 < r3) goto L_0x028c
            r3 = 2
            boolean r3 = r15.isVowel(r4, r3)
            if (r3 == 0) goto L_0x028c
            r5.append(r12)
            goto L_0x02b0
        L_0x028c:
            r5.append(r9)
            goto L_0x02b0
        L_0x0290:
            r5.append(r12)
            goto L_0x02b0
        L_0x0294:
            r13 = 71
            r3 = 77
            boolean r3 = r15.isPreviousChar(r4, r6, r3)
            if (r3 == 0) goto L_0x02a5
            boolean r3 = r15.isLastChar(r7, r6)
            if (r3 == 0) goto L_0x02a5
            goto L_0x02b0
        L_0x02a5:
            r5.append(r8)
            goto L_0x02b0
        L_0x02a9:
            r13 = 71
            if (r6 != 0) goto L_0x02b0
            r5.append(r8)
        L_0x02b0:
            r3 = 1
            int r6 = r6 + r3
        L_0x02b2:
            int r9 = r5.length()
            int r14 = r15.getMaxCodeLen()
            if (r9 <= r14) goto L_0x02c3
            int r9 = r15.getMaxCodeLen()
            r5.setLength(r9)
        L_0x02c3:
            r9 = 71
            goto L_0x0096
        L_0x02c7:
            java.lang.String r3 = r5.toString()
            return r3
        L_0x02cc:
            java.lang.String r2 = ""
            return r2
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.codec.language.Metaphone.metaphone(java.lang.String):java.lang.String");
    }

    private boolean isVowel(StringBuffer string, int index) {
        return this.vowels.indexOf(string.charAt(index)) >= 0;
    }

    private boolean isPreviousChar(StringBuffer string, int index, char c) {
        if (index <= 0 || index >= string.length()) {
            return false;
        }
        return string.charAt(index + -1) == c;
    }

    private boolean isNextChar(StringBuffer string, int index, char c) {
        if (index < 0) {
            return false;
        }
        boolean matches = true;
        if (index >= string.length() - 1) {
            return false;
        }
        if (string.charAt(index + 1) != c) {
            matches = false;
        }
        return matches;
    }

    private boolean regionMatch(StringBuffer string, int index, String test) {
        if (index < 0 || (test.length() + index) - 1 >= string.length()) {
            return false;
        }
        return string.substring(index, test.length() + index).equals(test);
    }

    private boolean isLastChar(int wdsz, int n) {
        return n + 1 == wdsz;
    }

    public Object encode(Object pObject) throws EncoderException {
        if (pObject instanceof String) {
            return metaphone((String) pObject);
        }
        throw new EncoderException("Parameter supplied to Metaphone encode is not of type java.lang.String");
    }

    public String encode(String pString) {
        return metaphone(pString);
    }

    public boolean isMetaphoneEqual(String str1, String str2) {
        return metaphone(str1).equals(metaphone(str2));
    }

    public int getMaxCodeLen() {
        return this.maxCodeLen;
    }

    public void setMaxCodeLen(int maxCodeLen2) {
        this.maxCodeLen = maxCodeLen2;
    }
}
