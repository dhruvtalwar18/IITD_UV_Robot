package org.apache.ws.commons.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import javax.xml.namespace.NamespaceContext;

public class NamespaceContextImpl implements NamespaceContext {
    private String cachedPrefix;
    private String cachedURI;
    private List prefixList;

    public void reset() {
        this.cachedPrefix = null;
        this.cachedURI = null;
        if (this.prefixList != null) {
            this.prefixList.clear();
        }
    }

    public void startPrefixMapping(String pPrefix, String pURI) {
        if (pPrefix == null) {
            throw new IllegalArgumentException("The namespace prefix must not be null.");
        } else if (pURI != null) {
            if (this.cachedURI != null) {
                if (this.prefixList == null) {
                    this.prefixList = new ArrayList();
                }
                this.prefixList.add(this.cachedPrefix);
                this.prefixList.add(this.cachedURI);
            }
            this.cachedURI = pURI;
            this.cachedPrefix = pPrefix;
        } else {
            throw new IllegalArgumentException("The namespace prefix must not be null.");
        }
    }

    public void endPrefixMapping(String pPrefix) {
        if (pPrefix == null) {
            throw new IllegalArgumentException("The namespace prefix must not be null.");
        } else if (!pPrefix.equals(this.cachedPrefix)) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("The prefix ");
            stringBuffer.append(pPrefix);
            stringBuffer.append(" isn't the prefix, which has been defined last.");
            throw new IllegalStateException(stringBuffer.toString());
        } else if (this.prefixList == null || this.prefixList.size() <= 0) {
            this.cachedURI = null;
            this.cachedPrefix = null;
        } else {
            this.cachedURI = this.prefixList.remove(this.prefixList.size() - 1).toString();
            this.cachedPrefix = this.prefixList.remove(this.prefixList.size() - 1).toString();
        }
    }

    public String getNamespaceURI(String pPrefix) {
        if (pPrefix != null) {
            if (this.cachedURI != null) {
                if (this.cachedPrefix.equals(pPrefix)) {
                    return this.cachedURI;
                }
                if (this.prefixList != null) {
                    for (int i = this.prefixList.size(); i > 0; i -= 2) {
                        if (pPrefix.equals(this.prefixList.get(i - 2))) {
                            return (String) this.prefixList.get(i - 1);
                        }
                    }
                }
            }
            if ("xml".equals(pPrefix)) {
                return "http://www.w3.org/XML/1998/namespace";
            }
            if ("xmlns".equals(pPrefix)) {
                return "http://www.w3.org/2000/xmlns/";
            }
            return null;
        }
        throw new IllegalArgumentException("The namespace prefix must not be null.");
    }

    public String getPrefix(String pURI) {
        if (pURI != null) {
            if (this.cachedURI != null) {
                if (this.cachedURI.equals(pURI)) {
                    return this.cachedPrefix;
                }
                if (this.prefixList != null) {
                    for (int i = this.prefixList.size(); i > 0; i -= 2) {
                        if (pURI.equals(this.prefixList.get(i - 1))) {
                            return (String) this.prefixList.get(i - 2);
                        }
                    }
                }
            }
            if ("http://www.w3.org/XML/1998/namespace".equals(pURI)) {
                return "xml";
            }
            if ("http://www.w3.org/2000/xmlns/".equals(pURI)) {
                return "xmlns";
            }
            return null;
        }
        throw new IllegalArgumentException("The namespace URI must not be null.");
    }

    public String getAttributePrefix(String pURI) {
        if (pURI == null) {
            throw new IllegalArgumentException("The namespace URI must not be null.");
        } else if (pURI.length() == 0) {
            return "";
        } else {
            if (this.cachedURI != null) {
                if (this.cachedURI.equals(pURI) && this.cachedPrefix.length() > 0) {
                    return this.cachedPrefix;
                }
                if (this.prefixList != null) {
                    for (int i = this.prefixList.size(); i > 0; i -= 2) {
                        if (pURI.equals(this.prefixList.get(i - 1))) {
                            String prefix = (String) this.prefixList.get(i - 2);
                            if (prefix.length() > 0) {
                                return prefix;
                            }
                        }
                    }
                }
            }
            if ("http://www.w3.org/XML/1998/namespace".equals(pURI)) {
                return "xml";
            }
            if ("http://www.w3.org/2000/xmlns/".equals(pURI)) {
                return "xmlns";
            }
            return null;
        }
    }

    public Iterator getPrefixes(String pURI) {
        if (pURI != null) {
            List list = new ArrayList();
            if (this.cachedURI != null) {
                if (this.cachedURI.equals(pURI)) {
                    list.add(this.cachedPrefix);
                }
                if (this.prefixList != null) {
                    for (int i = this.prefixList.size(); i > 0; i -= 2) {
                        if (pURI.equals(this.prefixList.get(i - 1))) {
                            list.add(this.prefixList.get(i - 2));
                        }
                    }
                }
            }
            if (pURI.equals("http://www.w3.org/2000/xmlns/")) {
                list.add("xmlns");
            } else if (pURI.equals("http://www.w3.org/XML/1998/namespace")) {
                list.add("xml");
            }
            return list.iterator();
        }
        throw new IllegalArgumentException("The namespace URI must not be null.");
    }

    public boolean isPrefixDeclared(String pPrefix) {
        if (this.cachedURI != null) {
            if (this.cachedPrefix != null && this.cachedPrefix.equals(pPrefix)) {
                return true;
            }
            if (this.prefixList != null) {
                for (int i = this.prefixList.size(); i > 0; i -= 2) {
                    if (this.prefixList.get(i - 2).equals(pPrefix)) {
                        return true;
                    }
                }
            }
        }
        return "xml".equals(pPrefix);
    }

    public int getContext() {
        int i = 0;
        int size = this.prefixList == null ? 0 : this.prefixList.size();
        if (this.cachedURI != null) {
            i = 2;
        }
        return size + i;
    }

    public String checkContext(int i) {
        if (getContext() == i) {
            return null;
        }
        String result = this.cachedPrefix;
        if (this.prefixList == null || this.prefixList.size() <= 0) {
            this.cachedURI = null;
            this.cachedPrefix = null;
        } else {
            this.cachedURI = this.prefixList.remove(this.prefixList.size() - 1).toString();
            this.cachedPrefix = this.prefixList.remove(this.prefixList.size() - 1).toString();
        }
        return result;
    }

    public List getPrefixes() {
        if (this.cachedPrefix == null) {
            return Collections.EMPTY_LIST;
        }
        if (this.prefixList == null) {
            return Collections.singletonList(this.cachedPrefix);
        }
        List result = new ArrayList(this.prefixList.size() + 1);
        for (int i = 0; i < this.prefixList.size(); i += 2) {
            result.add(this.prefixList.get(i));
        }
        result.add(this.cachedPrefix);
        return result;
    }
}
