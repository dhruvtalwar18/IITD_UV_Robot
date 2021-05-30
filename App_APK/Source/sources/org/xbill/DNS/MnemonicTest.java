package org.xbill.DNS;

import junit.framework.TestCase;

public class MnemonicTest extends TestCase {
    static /* synthetic */ Class class$org$xbill$DNS$MnemonicTest;
    private Mnemonic m_mn;

    public MnemonicTest(String name) {
        super(name);
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError().initCause(x1);
        }
    }

    public void setUp() {
        Class cls;
        StringBuffer stringBuffer = new StringBuffer();
        if (class$org$xbill$DNS$MnemonicTest == null) {
            cls = class$("org.xbill.DNS.MnemonicTest");
            class$org$xbill$DNS$MnemonicTest = cls;
        } else {
            cls = class$org$xbill$DNS$MnemonicTest;
        }
        stringBuffer.append(cls.getName());
        stringBuffer.append(" UPPER");
        this.m_mn = new Mnemonic(stringBuffer.toString(), 2);
    }

    public void test_toInteger() {
        Integer i = Mnemonic.toInteger(64);
        assertEquals(new Integer(64), i);
        Integer i2 = Mnemonic.toInteger(64);
        assertEquals(i, i2);
        assertNotSame(i, i2);
        Integer i3 = Mnemonic.toInteger(-1);
        assertEquals(new Integer(-1), i3);
        Integer i22 = Mnemonic.toInteger(-1);
        assertEquals(i3, i22);
        assertNotSame(i3, i22);
        Integer i4 = Mnemonic.toInteger(0);
        assertEquals(new Integer(0), i4);
        Integer i23 = Mnemonic.toInteger(0);
        assertEquals(i4, i23);
        assertSame(i4, i23);
        Integer i5 = Mnemonic.toInteger(63);
        assertEquals(new Integer(63), i5);
        Integer i24 = Mnemonic.toInteger(63);
        assertEquals(i5, i24);
        assertSame(i5, i24);
    }

    public void test_no_maximum() {
        try {
            this.m_mn.check(-1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
        try {
            this.m_mn.check(0);
        } catch (IllegalArgumentException e2) {
            fail(e2.getMessage());
        }
        try {
            this.m_mn.check(Integer.MAX_VALUE);
        } catch (IllegalArgumentException e3) {
            fail(e3.getMessage());
        }
        this.m_mn.setNumericAllowed(true);
        assertEquals(-1, this.m_mn.getValue("-2"));
        assertEquals(0, this.m_mn.getValue("0"));
        assertEquals(Integer.MAX_VALUE, this.m_mn.getValue("2147483647"));
    }

    public void test_setMaximum() {
        this.m_mn.setMaximum(15);
        try {
            this.m_mn.check(-1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
        try {
            this.m_mn.check(0);
        } catch (IllegalArgumentException e2) {
            fail(e2.getMessage());
        }
        try {
            this.m_mn.check(15);
        } catch (IllegalArgumentException e3) {
            fail(e3.getMessage());
        }
        try {
            this.m_mn.check(16);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e4) {
        }
        this.m_mn.setNumericAllowed(true);
        assertEquals(-1, this.m_mn.getValue("-2"));
        assertEquals(0, this.m_mn.getValue("0"));
        assertEquals(15, this.m_mn.getValue("15"));
        assertEquals(-1, this.m_mn.getValue("16"));
    }

    public void test_setPrefix() {
        String prefix = "A mixed CASE Prefix".toUpperCase();
        this.m_mn.setPrefix(prefix);
        String out = this.m_mn.getText(10);
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append(prefix);
        stringBuffer.append("10");
        assertEquals(stringBuffer.toString(), out);
        assertEquals(10, this.m_mn.getValue(out));
    }

    public void test_basic_operation() {
        this.m_mn.add(10, "Ten");
        this.m_mn.add(20, "Twenty");
        this.m_mn.addAlias(20, "Veinte");
        this.m_mn.add(30, "Thirty");
        assertEquals("TEN", this.m_mn.getText(10));
        assertEquals("TWENTY", this.m_mn.getText(20));
        assertEquals("THIRTY", this.m_mn.getText(30));
        assertEquals("40", this.m_mn.getText(40));
        assertEquals(10, this.m_mn.getValue("tEn"));
        assertEquals(20, this.m_mn.getValue("twenty"));
        assertEquals(20, this.m_mn.getValue("VeiNTe"));
        assertEquals(30, this.m_mn.getValue("THIRTY"));
    }

    public void test_basic_operation_lower() {
        Class cls;
        StringBuffer stringBuffer = new StringBuffer();
        if (class$org$xbill$DNS$MnemonicTest == null) {
            cls = class$("org.xbill.DNS.MnemonicTest");
            class$org$xbill$DNS$MnemonicTest = cls;
        } else {
            cls = class$org$xbill$DNS$MnemonicTest;
        }
        stringBuffer.append(cls.getName());
        stringBuffer.append(" LOWER");
        this.m_mn = new Mnemonic(stringBuffer.toString(), 3);
        this.m_mn.add(10, "Ten");
        this.m_mn.add(20, "Twenty");
        this.m_mn.addAlias(20, "Veinte");
        this.m_mn.add(30, "Thirty");
        assertEquals("ten", this.m_mn.getText(10));
        assertEquals("twenty", this.m_mn.getText(20));
        assertEquals("thirty", this.m_mn.getText(30));
        assertEquals("40", this.m_mn.getText(40));
        assertEquals(10, this.m_mn.getValue("tEn"));
        assertEquals(20, this.m_mn.getValue("twenty"));
        assertEquals(20, this.m_mn.getValue("VeiNTe"));
        assertEquals(30, this.m_mn.getValue("THIRTY"));
    }

    public void test_basic_operation_sensitive() {
        Class cls;
        StringBuffer stringBuffer = new StringBuffer();
        if (class$org$xbill$DNS$MnemonicTest == null) {
            cls = class$("org.xbill.DNS.MnemonicTest");
            class$org$xbill$DNS$MnemonicTest = cls;
        } else {
            cls = class$org$xbill$DNS$MnemonicTest;
        }
        stringBuffer.append(cls.getName());
        stringBuffer.append(" SENSITIVE");
        this.m_mn = new Mnemonic(stringBuffer.toString(), 1);
        this.m_mn.add(10, "Ten");
        this.m_mn.add(20, "Twenty");
        this.m_mn.addAlias(20, "Veinte");
        this.m_mn.add(30, "Thirty");
        assertEquals("Ten", this.m_mn.getText(10));
        assertEquals("Twenty", this.m_mn.getText(20));
        assertEquals("Thirty", this.m_mn.getText(30));
        assertEquals("40", this.m_mn.getText(40));
        assertEquals(10, this.m_mn.getValue("Ten"));
        assertEquals(-1, this.m_mn.getValue("twenty"));
        assertEquals(20, this.m_mn.getValue("Twenty"));
        assertEquals(-1, this.m_mn.getValue("VEINTE"));
        assertEquals(20, this.m_mn.getValue("Veinte"));
        assertEquals(30, this.m_mn.getValue("Thirty"));
    }

    public void test_invalid_numeric() {
        this.m_mn.setNumericAllowed(true);
        assertEquals(-1, this.m_mn.getValue("Not-A-Number"));
    }

    public void test_addAll() {
        this.m_mn.add(10, "Ten");
        this.m_mn.add(20, "Twenty");
        Mnemonic mn2 = new Mnemonic("second test Mnemonic", 2);
        mn2.add(20, "Twenty");
        mn2.addAlias(20, "Veinte");
        mn2.add(30, "Thirty");
        this.m_mn.addAll(mn2);
        assertEquals("TEN", this.m_mn.getText(10));
        assertEquals("TWENTY", this.m_mn.getText(20));
        assertEquals("THIRTY", this.m_mn.getText(30));
        assertEquals("40", this.m_mn.getText(40));
        assertEquals(10, this.m_mn.getValue("tEn"));
        assertEquals(20, this.m_mn.getValue("twenty"));
        assertEquals(20, this.m_mn.getValue("VeiNTe"));
        assertEquals(30, this.m_mn.getValue("THIRTY"));
    }
}
