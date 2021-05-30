package org.xbill.DNS.spi;

import java.lang.reflect.Proxy;
import sun.net.spi.nameservice.NameService;
import sun.net.spi.nameservice.NameServiceDescriptor;

public class DNSJavaNameServiceDescriptor implements NameServiceDescriptor {
    static /* synthetic */ Class class$sun$net$spi$nameservice$NameService;
    private static NameService nameService;

    static {
        Class cls;
        Class cls2;
        if (class$sun$net$spi$nameservice$NameService == null) {
            cls = class$("sun.net.spi.nameservice.NameService");
            class$sun$net$spi$nameservice$NameService = cls;
        } else {
            cls = class$sun$net$spi$nameservice$NameService;
        }
        ClassLoader loader = cls.getClassLoader();
        Class[] clsArr = new Class[1];
        if (class$sun$net$spi$nameservice$NameService == null) {
            cls2 = class$("sun.net.spi.nameservice.NameService");
            class$sun$net$spi$nameservice$NameService = cls2;
        } else {
            cls2 = class$sun$net$spi$nameservice$NameService;
        }
        clsArr[0] = cls2;
        nameService = (NameService) Proxy.newProxyInstance(loader, clsArr, new DNSJavaNameService());
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError().initCause(x1);
        }
    }

    public NameService createNameService() {
        return nameService;
    }

    public String getType() {
        return "dns";
    }

    public String getProviderName() {
        return "dnsjava";
    }
}
