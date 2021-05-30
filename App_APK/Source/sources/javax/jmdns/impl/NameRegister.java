package javax.jmdns.impl;

import java.net.InetAddress;

public interface NameRegister {

    public enum NameType {
        HOST,
        SERVICE
    }

    boolean checkName(InetAddress inetAddress, String str, NameType nameType);

    String incrementHostName(InetAddress inetAddress, String str, NameType nameType);

    void register(InetAddress inetAddress, String str, NameType nameType);

    public static class UniqueNamePerInterface implements NameRegister {
        public void register(InetAddress networkInterface, String name, NameType type) {
        }

        public boolean checkName(InetAddress networkInterface, String name, NameType type) {
            return false;
        }

        public String incrementHostName(InetAddress networkInterface, String name, NameType type) {
            return null;
        }
    }

    public static class UniqueNameAcrossInterface implements NameRegister {
        public void register(InetAddress networkInterface, String name, NameType type) {
        }

        public boolean checkName(InetAddress networkInterface, String name, NameType type) {
            return false;
        }

        public String incrementHostName(InetAddress networkInterface, String name, NameType type) {
            return null;
        }
    }

    public static class Factory {
        private static volatile NameRegister _register;

        public static void setRegistry(NameRegister register) throws IllegalStateException {
            if (_register != null) {
                throw new IllegalStateException("The register can only be set once.");
            } else if (register != null) {
                _register = register;
            }
        }

        public static NameRegister getRegistry() {
            if (_register == null) {
                _register = new UniqueNamePerInterface();
            }
            return _register;
        }
    }
}
