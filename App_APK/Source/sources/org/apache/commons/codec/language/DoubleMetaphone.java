package org.apache.commons.codec.language;

import org.apache.commons.codec.EncoderException;
import org.apache.commons.codec.StringEncoder;

public class DoubleMetaphone implements StringEncoder {
    private static final String[] ES_EP_EB_EL_EY_IB_IL_IN_IE_EI_ER = {"ES", "EP", "EB", "EL", "EY", "IB", "IL", "IN", "IE", "EI", "ER"};
    private static final String[] L_R_N_M_B_H_F_V_W_SPACE = {"L", "R", "N", "M", "B", "H", "F", "V", "W", " "};
    private static final String[] L_T_K_S_N_M_B_Z = {"L", "T", "K", "S", "N", "M", "B", "Z"};
    private static final String[] SILENT_START = {"GN", "KN", "PN", "WR", "PS"};
    private static final String VOWELS = "AEIOUY";
    protected int maxCodeLen = 4;

    public String doubleMetaphone(String value) {
        return doubleMetaphone(value, false);
    }

    public String doubleMetaphone(String value, boolean alternate) {
        String value2 = cleanInput(value);
        if (value2 == null) {
            return null;
        }
        boolean slavoGermanic = isSlavoGermanic(value2);
        int index = isSilentStart(value2);
        DoubleMetaphoneResult result = new DoubleMetaphoneResult(getMaxCodeLen());
        while (!result.isComplete() && index <= value2.length() - 1) {
            char charAt = value2.charAt(index);
            if (charAt == 199) {
                result.append('S');
                index++;
            } else if (charAt != 209) {
                switch (charAt) {
                    case 'A':
                    case 'E':
                    case 'I':
                    case 'O':
                    case 'U':
                    case 'Y':
                        index = handleAEIOUY(value2, result, index);
                        break;
                    case 'B':
                        result.append('P');
                        index = charAt(value2, index + 1) == 'B' ? index + 2 : index + 1;
                        break;
                    case 'C':
                        index = handleC(value2, result, index);
                        break;
                    case 'D':
                        index = handleD(value2, result, index);
                        break;
                    case 'F':
                        result.append('F');
                        index = charAt(value2, index + 1) == 'F' ? index + 2 : index + 1;
                        break;
                    case 'G':
                        index = handleG(value2, result, index, slavoGermanic);
                        break;
                    case 'H':
                        index = handleH(value2, result, index);
                        break;
                    case 'J':
                        index = handleJ(value2, result, index, slavoGermanic);
                        break;
                    case 'K':
                        result.append('K');
                        index = charAt(value2, index + 1) == 'K' ? index + 2 : index + 1;
                        break;
                    case 'L':
                        index = handleL(value2, result, index);
                        break;
                    case 'M':
                        result.append('M');
                        index = conditionM0(value2, index) ? index + 2 : index + 1;
                        break;
                    case 'N':
                        result.append('N');
                        index = charAt(value2, index + 1) == 'N' ? index + 2 : index + 1;
                        break;
                    case 'P':
                        index = handleP(value2, result, index);
                        break;
                    case 'Q':
                        result.append('K');
                        index = charAt(value2, index + 1) == 'Q' ? index + 2 : index + 1;
                        break;
                    case 'R':
                        index = handleR(value2, result, index, slavoGermanic);
                        break;
                    case 'S':
                        index = handleS(value2, result, index, slavoGermanic);
                        break;
                    case 'T':
                        index = handleT(value2, result, index);
                        break;
                    case 'V':
                        result.append('F');
                        index = charAt(value2, index + 1) == 'V' ? index + 2 : index + 1;
                        break;
                    case 'W':
                        index = handleW(value2, result, index);
                        break;
                    case 'X':
                        index = handleX(value2, result, index);
                        break;
                    case 'Z':
                        index = handleZ(value2, result, index, slavoGermanic);
                        break;
                    default:
                        index++;
                        break;
                }
            } else {
                result.append('N');
                index++;
            }
        }
        return alternate ? result.getAlternate() : result.getPrimary();
    }

    public Object encode(Object obj) throws EncoderException {
        if (obj instanceof String) {
            return doubleMetaphone((String) obj);
        }
        throw new EncoderException("DoubleMetaphone encode parameter is not of type String");
    }

    public String encode(String value) {
        return doubleMetaphone(value);
    }

    public boolean isDoubleMetaphoneEqual(String value1, String value2) {
        return isDoubleMetaphoneEqual(value1, value2, false);
    }

    public boolean isDoubleMetaphoneEqual(String value1, String value2, boolean alternate) {
        return doubleMetaphone(value1, alternate).equals(doubleMetaphone(value2, alternate));
    }

    public int getMaxCodeLen() {
        return this.maxCodeLen;
    }

    public void setMaxCodeLen(int maxCodeLen2) {
        this.maxCodeLen = maxCodeLen2;
    }

    private int handleAEIOUY(String value, DoubleMetaphoneResult result, int index) {
        if (index == 0) {
            result.append('A');
        }
        return index + 1;
    }

    private int handleC(String value, DoubleMetaphoneResult result, int index) {
        String str = value;
        DoubleMetaphoneResult doubleMetaphoneResult = result;
        int i = index;
        if (conditionC0(str, i)) {
            doubleMetaphoneResult.append('K');
            return i + 2;
        } else if (i == 0 && contains(str, i, 6, "CAESAR")) {
            doubleMetaphoneResult.append('S');
            return i + 2;
        } else if (contains(str, i, 2, "CH")) {
            return handleCH(value, result, index);
        } else {
            if (contains(str, i, 2, "CZ") && !contains(str, i - 2, 4, "WICZ")) {
                doubleMetaphoneResult.append('S', 'X');
                return i + 2;
            } else if (contains(str, i + 1, 3, "CIA")) {
                doubleMetaphoneResult.append('X');
                return i + 3;
            } else if (contains(str, i, 2, "CC") && (i != 1 || charAt(str, 0) != 'M')) {
                return handleCC(value, result, index);
            } else {
                if (contains(value, index, 2, "CK", "CG", "CQ")) {
                    doubleMetaphoneResult.append('K');
                    return i + 2;
                } else if (contains(value, index, 2, "CI", "CE", "CY")) {
                    if (contains(value, index, 3, "CIO", "CIE", "CIA")) {
                        doubleMetaphoneResult.append('S', 'X');
                    } else {
                        doubleMetaphoneResult.append('S');
                    }
                    return i + 2;
                } else {
                    doubleMetaphoneResult.append('K');
                    if (contains(value, i + 1, 2, " C", " Q", " G")) {
                        return i + 3;
                    }
                    if (!contains(value, i + 1, 1, "C", "K", "Q") || contains(str, i + 1, 2, "CE", "CI")) {
                        return i + 1;
                    }
                    return i + 2;
                }
            }
        }
    }

    private int handleCC(String value, DoubleMetaphoneResult result, int index) {
        if (!contains(value, index + 2, 1, "I", "E", "H") || contains(value, index + 2, 2, "HU")) {
            result.append('K');
            return index + 2;
        }
        if (!(index == 1 && charAt(value, index - 1) == 'A') && !contains(value, index - 1, 5, "UCCEE", "UCCES")) {
            result.append('X');
        } else {
            result.append("KS");
        }
        return index + 3;
    }

    private int handleCH(String value, DoubleMetaphoneResult result, int index) {
        if (index > 0 && contains(value, index, 4, "CHAE")) {
            result.append('K', 'X');
            return index + 2;
        } else if (conditionCH0(value, index)) {
            result.append('K');
            return index + 2;
        } else if (conditionCH1(value, index)) {
            result.append('K');
            return index + 2;
        } else {
            if (index <= 0) {
                result.append('X');
            } else if (contains(value, 0, 2, "MC")) {
                result.append('K');
            } else {
                result.append('X', 'K');
            }
            return index + 2;
        }
    }

    private int handleD(String value, DoubleMetaphoneResult result, int index) {
        if (contains(value, index, 2, "DG")) {
            if (contains(value, index + 2, 1, "I", "E", "Y")) {
                result.append('J');
                return index + 3;
            }
            result.append("TK");
            return index + 2;
        } else if (contains(value, index, 2, "DT", "DD")) {
            result.append('T');
            return index + 2;
        } else {
            result.append('T');
            return index + 1;
        }
    }

    private int handleG(String value, DoubleMetaphoneResult result, int index, boolean slavoGermanic) {
        int i;
        String str = value;
        DoubleMetaphoneResult doubleMetaphoneResult = result;
        int i2 = index;
        if (charAt(str, i2 + 1) == 'H') {
            return handleGH(value, result, index);
        }
        if (charAt(str, i2 + 1) == 'N') {
            if (i2 == 1 && isVowel(charAt(str, 0)) && !slavoGermanic) {
                doubleMetaphoneResult.append("KN", "N");
            } else if (contains(str, i2 + 2, 2, "EY") || charAt(str, i2 + 1) == 'Y' || slavoGermanic) {
                doubleMetaphoneResult.append("KN");
            } else {
                doubleMetaphoneResult.append("N", "KN");
            }
            return i2 + 2;
        } else if (contains(str, i2 + 1, 2, "LI") && !slavoGermanic) {
            doubleMetaphoneResult.append("KL", "L");
            return i2 + 2;
        } else if (i2 != 0 || (charAt(str, i2 + 1) != 'Y' && !contains(str, i2 + 1, 2, ES_EP_EB_EL_EY_IB_IL_IN_IE_EI_ER))) {
            if (contains(str, i2 + 1, 2, "ER") || charAt(str, i2 + 1) == 'Y') {
                i = 3;
                if (!contains(value, 0, 6, "DANGER", "RANGER", "MANGER") && !contains(str, i2 - 1, 1, "E", "I") && !contains(str, i2 - 1, 3, "RGY", "OGY")) {
                    doubleMetaphoneResult.append('K', 'J');
                    return i2 + 2;
                }
            } else {
                i = 3;
            }
            if (contains(value, i2 + 1, 1, "E", "I", "Y") || contains(str, i2 - 1, 4, "AGGI", "OGGI")) {
                if (contains(str, 0, 4, "VAN ", "VON ") || contains(str, 0, i, "SCH") || contains(str, i2 + 1, 2, "ET")) {
                    doubleMetaphoneResult.append('K');
                } else if (contains(str, i2 + 1, 4, "IER")) {
                    doubleMetaphoneResult.append('J');
                } else {
                    doubleMetaphoneResult.append('J', 'K');
                }
                return i2 + 2;
            } else if (charAt(str, i2 + 1) == 'G') {
                int index2 = i2 + 2;
                doubleMetaphoneResult.append('K');
                return index2;
            } else {
                int index3 = i2 + 1;
                doubleMetaphoneResult.append('K');
                return index3;
            }
        } else {
            doubleMetaphoneResult.append('K', 'J');
            return i2 + 2;
        }
    }

    /* JADX WARNING: Code restructure failed: missing block: B:15:0x004a, code lost:
        if (contains(r16, r11 - 2, 1, "B", "H", "D") == false) goto L_0x004c;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:18:0x005d, code lost:
        if (contains(r16, r11 - 3, 1, "B", "H", "D") == false) goto L_0x005f;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private int handleGH(java.lang.String r16, org.apache.commons.codec.language.DoubleMetaphone.DoubleMetaphoneResult r17, int r18) {
        /*
            r15 = this;
            r0 = r15
            r9 = r16
            r10 = r17
            r11 = r18
            r12 = 75
            r13 = 2
            if (r11 <= 0) goto L_0x001f
            int r1 = r11 + -1
            char r1 = r15.charAt(r9, r1)
            boolean r1 = r15.isVowel(r1)
            if (r1 != 0) goto L_0x001f
            r10.append((char) r12)
            int r1 = r11 + 2
            goto L_0x00a7
        L_0x001f:
            r14 = 73
            if (r11 != 0) goto L_0x0038
            int r1 = r11 + 2
            char r1 = r15.charAt(r9, r1)
            if (r1 != r14) goto L_0x0031
            r1 = 74
            r10.append((char) r1)
            goto L_0x0034
        L_0x0031:
            r10.append((char) r12)
        L_0x0034:
            int r1 = r11 + 2
            goto L_0x00a7
        L_0x0038:
            r7 = 1
            if (r11 <= r7) goto L_0x004c
            int r2 = r11 + -2
            r3 = 1
            java.lang.String r4 = "B"
            java.lang.String r5 = "H"
            java.lang.String r6 = "D"
            r1 = r16
            boolean r1 = contains(r1, r2, r3, r4, r5, r6)
            if (r1 != 0) goto L_0x006e
        L_0x004c:
            if (r11 <= r13) goto L_0x005f
            int r2 = r11 + -3
            r3 = 1
            java.lang.String r4 = "B"
            java.lang.String r5 = "H"
            java.lang.String r6 = "D"
            r1 = r16
            boolean r1 = contains(r1, r2, r3, r4, r5, r6)
            if (r1 != 0) goto L_0x006e
        L_0x005f:
            r1 = 3
            if (r11 <= r1) goto L_0x0071
            int r1 = r11 + -4
            java.lang.String r2 = "B"
            java.lang.String r3 = "H"
            boolean r1 = contains(r9, r1, r7, r2, r3)
            if (r1 == 0) goto L_0x0071
        L_0x006e:
            int r1 = r11 + 2
            goto L_0x00a7
        L_0x0071:
            if (r11 <= r13) goto L_0x0098
            int r1 = r11 + -1
            char r1 = r15.charAt(r9, r1)
            r2 = 85
            if (r1 != r2) goto L_0x0098
            int r2 = r11 + -3
            r3 = 1
            java.lang.String r4 = "C"
            java.lang.String r5 = "G"
            java.lang.String r6 = "L"
            java.lang.String r7 = "R"
            java.lang.String r8 = "T"
            r1 = r16
            boolean r1 = contains(r1, r2, r3, r4, r5, r6, r7, r8)
            if (r1 == 0) goto L_0x0098
            r1 = 70
            r10.append((char) r1)
            goto L_0x00a5
        L_0x0098:
            if (r11 <= 0) goto L_0x00a5
            int r1 = r11 + -1
            char r1 = r15.charAt(r9, r1)
            if (r1 == r14) goto L_0x00a5
            r10.append((char) r12)
        L_0x00a5:
            int r1 = r11 + 2
        L_0x00a7:
            return r1
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.codec.language.DoubleMetaphone.handleGH(java.lang.String, org.apache.commons.codec.language.DoubleMetaphone$DoubleMetaphoneResult, int):int");
    }

    private int handleH(String value, DoubleMetaphoneResult result, int index) {
        if ((index != 0 && !isVowel(charAt(value, index - 1))) || !isVowel(charAt(value, index + 1))) {
            return index + 1;
        }
        result.append('H');
        return index + 2;
    }

    private int handleJ(String value, DoubleMetaphoneResult result, int index, boolean slavoGermanic) {
        String str = value;
        DoubleMetaphoneResult doubleMetaphoneResult = result;
        int i = index;
        if (contains(value, index, 4, "JOSE") || contains(value, 0, 4, "SAN ")) {
            if ((i == 0 && charAt(value, i + 4) == ' ') || value.length() == 4 || contains(value, 0, 4, "SAN ")) {
                result.append('H');
            } else {
                result.append('J', 'H');
            }
            return i + 1;
        }
        if (i == 0 && !contains(value, index, 4, "JOSE")) {
            result.append('J', 'A');
        } else if (isVowel(charAt(value, i - 1)) && !slavoGermanic && (charAt(value, i + 1) == 'A' || charAt(value, i + 1) == 'O')) {
            result.append('J', 'H');
        } else if (i == value.length() - 1) {
            result.append('J', ' ');
        } else if (!contains(value, i + 1, 1, L_T_K_S_N_M_B_Z)) {
            if (!contains(value, i - 1, 1, "S", "K", "L")) {
                result.append('J');
            }
        }
        if (charAt(value, i + 1) == 'J') {
            return i + 2;
        }
        return i + 1;
    }

    private int handleL(String value, DoubleMetaphoneResult result, int index) {
        result.append('L');
        if (charAt(value, index + 1) != 'L') {
            return index + 1;
        }
        if (conditionL0(value, index)) {
            result.appendAlternate(' ');
        }
        return index + 2;
    }

    private int handleP(String value, DoubleMetaphoneResult result, int index) {
        if (charAt(value, index + 1) == 'H') {
            result.append('F');
            return index + 2;
        }
        result.append('P');
        return contains(value, index + 1, 1, "P", "B") ? index + 2 : index + 1;
    }

    private int handleR(String value, DoubleMetaphoneResult result, int index, boolean slavoGermanic) {
        if (index != value.length() - 1 || slavoGermanic || !contains(value, index - 2, 2, "IE") || contains(value, index - 4, 2, "ME", "MA")) {
            result.append('R');
        } else {
            result.appendAlternate('R');
        }
        return charAt(value, index + 1) == 'R' ? index + 2 : index + 1;
    }

    /* JADX WARNING: Code restructure failed: missing block: B:22:0x0079, code lost:
        if (contains(r15, r9 + 1, 1, "M", "N", "L", "W") == false) goto L_0x007b;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private int handleS(java.lang.String r15, org.apache.commons.codec.language.DoubleMetaphone.DoubleMetaphoneResult r16, int r17, boolean r18) {
        /*
            r14 = this;
            r7 = r15
            r8 = r16
            r9 = r17
            int r0 = r9 + -1
            java.lang.String r1 = "ISL"
            java.lang.String r2 = "YSL"
            r3 = 3
            boolean r0 = contains(r15, r0, r3, r1, r2)
            r10 = 1
            if (r0 == 0) goto L_0x0018
            int r0 = r9 + 1
        L_0x0015:
            r1 = r0
            goto L_0x00df
        L_0x0018:
            r11 = 88
            r12 = 83
            if (r9 != 0) goto L_0x002d
            r0 = 5
            java.lang.String r1 = "SUGAR"
            boolean r0 = contains((java.lang.String) r15, (int) r9, (int) r0, (java.lang.String) r1)
            if (r0 == 0) goto L_0x002d
            r8.append((char) r11, (char) r12)
            int r0 = r9 + 1
            goto L_0x0015
        L_0x002d:
            java.lang.String r0 = "SH"
            r13 = 2
            boolean r0 = contains((java.lang.String) r15, (int) r9, (int) r13, (java.lang.String) r0)
            if (r0 == 0) goto L_0x0052
            int r1 = r9 + 1
            r2 = 4
            java.lang.String r3 = "HEIM"
            java.lang.String r4 = "HOEK"
            java.lang.String r5 = "HOLM"
            java.lang.String r6 = "HOLZ"
            r0 = r15
            boolean r0 = contains(r0, r1, r2, r3, r4, r5, r6)
            if (r0 == 0) goto L_0x004c
            r8.append((char) r12)
            goto L_0x004f
        L_0x004c:
            r8.append((char) r11)
        L_0x004f:
            int r0 = r9 + 2
            goto L_0x0015
        L_0x0052:
            java.lang.String r0 = "SIO"
            java.lang.String r1 = "SIA"
            boolean r0 = contains(r15, r9, r3, r0, r1)
            if (r0 != 0) goto L_0x00d4
            r0 = 4
            java.lang.String r1 = "SIAN"
            boolean r0 = contains((java.lang.String) r15, (int) r9, (int) r0, (java.lang.String) r1)
            if (r0 == 0) goto L_0x0067
            goto L_0x00d4
        L_0x0067:
            if (r9 != 0) goto L_0x007b
            int r1 = r9 + 1
            r2 = 1
            java.lang.String r3 = "M"
            java.lang.String r4 = "N"
            java.lang.String r5 = "L"
            java.lang.String r6 = "W"
            r0 = r15
            boolean r0 = contains(r0, r1, r2, r3, r4, r5, r6)
            if (r0 != 0) goto L_0x0085
        L_0x007b:
            int r0 = r9 + 1
            java.lang.String r1 = "Z"
            boolean r0 = contains((java.lang.String) r15, (int) r0, (int) r10, (java.lang.String) r1)
            if (r0 == 0) goto L_0x0099
        L_0x0085:
            r8.append((char) r12, (char) r11)
            int r0 = r9 + 1
            java.lang.String r1 = "Z"
            boolean r0 = contains((java.lang.String) r15, (int) r0, (int) r10, (java.lang.String) r1)
            if (r0 == 0) goto L_0x0095
            int r0 = r9 + 2
            goto L_0x0097
        L_0x0095:
            int r0 = r9 + 1
        L_0x0097:
            goto L_0x0015
        L_0x0099:
            java.lang.String r0 = "SC"
            boolean r0 = contains((java.lang.String) r15, (int) r9, (int) r13, (java.lang.String) r0)
            if (r0 == 0) goto L_0x00a7
            int r0 = r14.handleSC(r15, r16, r17)
            goto L_0x0015
        L_0x00a7:
            int r0 = r15.length()
            int r0 = r0 - r10
            if (r9 != r0) goto L_0x00be
            int r0 = r9 + -2
            java.lang.String r1 = "AI"
            java.lang.String r2 = "OI"
            boolean r0 = contains(r15, r0, r13, r1, r2)
            if (r0 == 0) goto L_0x00be
            r8.appendAlternate((char) r12)
            goto L_0x00c1
        L_0x00be:
            r8.append((char) r12)
        L_0x00c1:
            int r0 = r9 + 1
            java.lang.String r1 = "S"
            java.lang.String r2 = "Z"
            boolean r0 = contains(r15, r0, r10, r1, r2)
            if (r0 == 0) goto L_0x00d0
            int r0 = r9 + 2
            goto L_0x00d2
        L_0x00d0:
            int r0 = r9 + 1
        L_0x00d2:
            goto L_0x0015
        L_0x00d4:
            if (r18 == 0) goto L_0x00da
            r8.append((char) r12)
            goto L_0x00dd
        L_0x00da:
            r8.append((char) r12, (char) r11)
        L_0x00dd:
            int r1 = r9 + 3
        L_0x00df:
            return r1
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.codec.language.DoubleMetaphone.handleS(java.lang.String, org.apache.commons.codec.language.DoubleMetaphone$DoubleMetaphoneResult, int, boolean):int");
    }

    private int handleSC(String value, DoubleMetaphoneResult result, int index) {
        if (charAt(value, index + 2) == 'H') {
            if (contains(value, index + 3, 2, "OO", "ER", "EN", "UY", "ED", "EM")) {
                if (contains(value, index + 3, 2, "ER", "EN")) {
                    result.append("X", "SK");
                } else {
                    result.append("SK");
                }
            } else if (index != 0 || isVowel(charAt(value, 3)) || charAt(value, 3) == 'W') {
                result.append('X');
            } else {
                result.append('X', 'S');
            }
        } else {
            if (contains(value, index + 2, 1, "I", "E", "Y")) {
                result.append('S');
            } else {
                result.append("SK");
            }
        }
        return index + 3;
    }

    private int handleT(String value, DoubleMetaphoneResult result, int index) {
        if (contains(value, index, 4, "TION")) {
            result.append('X');
            return index + 3;
        } else if (contains(value, index, 3, "TIA", "TCH")) {
            result.append('X');
            return index + 3;
        } else if (contains(value, index, 2, "TH") || contains(value, index, 3, "TTH")) {
            if (contains(value, index + 2, 2, "OM", "AM") || contains(value, 0, 4, "VAN ", "VON ") || contains(value, 0, 3, "SCH")) {
                result.append('T');
            } else {
                result.append('0', 'T');
            }
            return index + 2;
        } else {
            result.append('T');
            return contains(value, index + 1, 1, "T", "D") ? index + 2 : index + 1;
        }
    }

    private int handleW(String value, DoubleMetaphoneResult result, int index) {
        if (contains(value, index, 2, "WR")) {
            result.append('R');
            return index + 2;
        } else if (index != 0 || (!isVowel(charAt(value, index + 1)) && !contains(value, index, 2, "WH"))) {
            if (index != value.length() - 1 || !isVowel(charAt(value, index - 1))) {
                if (!contains(value, index - 1, 5, "EWSKI", "EWSKY", "OWSKI", "OWSKY") && !contains(value, 0, 3, "SCH")) {
                    if (!contains(value, index, 4, "WICZ", "WITZ")) {
                        return index + 1;
                    }
                    result.append("TS", "FX");
                    return index + 4;
                }
            }
            result.appendAlternate('F');
            return index + 1;
        } else {
            if (isVowel(charAt(value, index + 1))) {
                result.append('A', 'F');
            } else {
                result.append('A');
            }
            return index + 1;
        }
    }

    private int handleX(String value, DoubleMetaphoneResult result, int index) {
        if (index == 0) {
            result.append('S');
            return index + 1;
        }
        if (index != value.length() - 1 || (!contains(value, index - 3, 3, "IAU", "EAU") && !contains(value, index - 2, 2, "AU", "OU"))) {
            result.append("KS");
        }
        return contains(value, index + 1, 1, "C", "X") ? index + 2 : index + 1;
    }

    private int handleZ(String value, DoubleMetaphoneResult result, int index, boolean slavoGermanic) {
        if (charAt(value, index + 1) == 'H') {
            result.append('J');
            return index + 2;
        }
        if (contains(value, index + 1, 2, "ZO", "ZI", "ZA") || (slavoGermanic && index > 0 && charAt(value, index - 1) != 'T')) {
            result.append("S", "TS");
        } else {
            result.append('S');
        }
        return charAt(value, index + 1) == 'Z' ? index + 2 : index + 1;
    }

    private boolean conditionC0(String value, int index) {
        if (contains(value, index, 4, "CHIA")) {
            return true;
        }
        if (index <= 1 || isVowel(charAt(value, index - 2)) || !contains(value, index - 1, 3, "ACH")) {
            return false;
        }
        char c = charAt(value, index + 2);
        if ((c == 'I' || c == 'E') && !contains(value, index - 2, 6, "BACHER", "MACHER")) {
            return false;
        }
        return true;
    }

    private boolean conditionCH0(String value, int index) {
        if (index != 0) {
            return false;
        }
        if (!contains(value, index + 1, 5, "HARAC", "HARIS")) {
            if (!contains(value, index + 1, 3, "HOR", "HYM", "HIA", "HEM")) {
                return false;
            }
        }
        if (contains(value, 0, 5, "CHORE")) {
            return false;
        }
        return true;
    }

    private boolean conditionCH1(String value, int index) {
        if (contains(value, 0, 4, "VAN ", "VON ") || contains(value, 0, 3, "SCH")) {
            return true;
        }
        if (contains(value, index - 2, 6, "ORCHES", "ARCHIT", "ORCHID") || contains(value, index + 2, 1, "T", "S")) {
            return true;
        }
        return (contains(value, index + -1, 1, "A", "O", "U", "E") || index == 0) && (contains(value, index + 2, 1, L_R_N_M_B_H_F_V_W_SPACE) || index + 1 == value.length() - 1);
    }

    private boolean conditionL0(String value, int index) {
        if (index == value.length() - 3) {
            if (contains(value, index - 1, 4, "ILLO", "ILLA", "ALLE")) {
                return true;
            }
        }
        if ((contains(value, index - 1, 2, "AS", "OS") || contains(value, value.length() - 1, 1, "A", "O")) && contains(value, index - 1, 4, "ALLE")) {
            return true;
        }
        return false;
    }

    private boolean conditionM0(String value, int index) {
        if (charAt(value, index + 1) == 'M') {
            return true;
        }
        if (!contains(value, index - 1, 3, "UMB") || (index + 1 != value.length() - 1 && !contains(value, index + 2, 2, "ER"))) {
            return false;
        }
        return true;
    }

    private boolean isSlavoGermanic(String value) {
        return value.indexOf(87) > -1 || value.indexOf(75) > -1 || value.indexOf("CZ") > -1 || value.indexOf("WITZ") > -1;
    }

    private boolean isVowel(char ch) {
        return VOWELS.indexOf(ch) != -1;
    }

    private boolean isSilentStart(String value) {
        for (String startsWith : SILENT_START) {
            if (value.startsWith(startsWith)) {
                return true;
            }
        }
        return false;
    }

    private String cleanInput(String input) {
        if (input == null) {
            return null;
        }
        String input2 = input.trim();
        if (input2.length() == 0) {
            return null;
        }
        return input2.toUpperCase();
    }

    /* access modifiers changed from: protected */
    public char charAt(String value, int index) {
        if (index < 0 || index >= value.length()) {
            return 0;
        }
        return value.charAt(index);
    }

    private static boolean contains(String value, int start, int length, String criteria) {
        return contains(value, start, length, new String[]{criteria});
    }

    private static boolean contains(String value, int start, int length, String criteria1, String criteria2) {
        return contains(value, start, length, new String[]{criteria1, criteria2});
    }

    private static boolean contains(String value, int start, int length, String criteria1, String criteria2, String criteria3) {
        return contains(value, start, length, new String[]{criteria1, criteria2, criteria3});
    }

    private static boolean contains(String value, int start, int length, String criteria1, String criteria2, String criteria3, String criteria4) {
        return contains(value, start, length, new String[]{criteria1, criteria2, criteria3, criteria4});
    }

    private static boolean contains(String value, int start, int length, String criteria1, String criteria2, String criteria3, String criteria4, String criteria5) {
        return contains(value, start, length, new String[]{criteria1, criteria2, criteria3, criteria4, criteria5});
    }

    private static boolean contains(String value, int start, int length, String criteria1, String criteria2, String criteria3, String criteria4, String criteria5, String criteria6) {
        return contains(value, start, length, new String[]{criteria1, criteria2, criteria3, criteria4, criteria5, criteria6});
    }

    protected static boolean contains(String value, int start, int length, String[] criteria) {
        if (start < 0 || start + length > value.length()) {
            return false;
        }
        String target = value.substring(start, start + length);
        for (String equals : criteria) {
            if (target.equals(equals)) {
                return true;
            }
        }
        return false;
    }

    public class DoubleMetaphoneResult {
        private StringBuffer alternate = new StringBuffer(DoubleMetaphone.this.getMaxCodeLen());
        private int maxLength;
        private StringBuffer primary = new StringBuffer(DoubleMetaphone.this.getMaxCodeLen());

        public DoubleMetaphoneResult(int maxLength2) {
            this.maxLength = maxLength2;
        }

        public void append(char value) {
            appendPrimary(value);
            appendAlternate(value);
        }

        public void append(char primary2, char alternate2) {
            appendPrimary(primary2);
            appendAlternate(alternate2);
        }

        public void appendPrimary(char value) {
            if (this.primary.length() < this.maxLength) {
                this.primary.append(value);
            }
        }

        public void appendAlternate(char value) {
            if (this.alternate.length() < this.maxLength) {
                this.alternate.append(value);
            }
        }

        public void append(String value) {
            appendPrimary(value);
            appendAlternate(value);
        }

        public void append(String primary2, String alternate2) {
            appendPrimary(primary2);
            appendAlternate(alternate2);
        }

        public void appendPrimary(String value) {
            int addChars = this.maxLength - this.primary.length();
            if (value.length() <= addChars) {
                this.primary.append(value);
            } else {
                this.primary.append(value.substring(0, addChars));
            }
        }

        public void appendAlternate(String value) {
            int addChars = this.maxLength - this.alternate.length();
            if (value.length() <= addChars) {
                this.alternate.append(value);
            } else {
                this.alternate.append(value.substring(0, addChars));
            }
        }

        public String getPrimary() {
            return this.primary.toString();
        }

        public String getAlternate() {
            return this.alternate.toString();
        }

        public boolean isComplete() {
            return this.primary.length() >= this.maxLength && this.alternate.length() >= this.maxLength;
        }
    }
}
