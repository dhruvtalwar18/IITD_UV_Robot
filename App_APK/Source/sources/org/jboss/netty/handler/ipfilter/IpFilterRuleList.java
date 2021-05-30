package org.jboss.netty.handler.ipfilter;

import java.net.UnknownHostException;
import java.util.ArrayList;
import org.jboss.netty.logging.InternalLogger;
import org.jboss.netty.logging.InternalLoggerFactory;

public class IpFilterRuleList extends ArrayList<IpFilterRule> {
    private static final InternalLogger logger = InternalLoggerFactory.getInstance((Class<?>) IpFilterRuleList.class);
    private static final long serialVersionUID = -6164162941749588780L;

    public IpFilterRuleList(String rules) {
        parseRules(rules);
    }

    private void parseRules(String rules) {
        for (String rule : rules.split(",")) {
            parseRule(rule.trim());
        }
    }

    private void parseRule(String rule) {
        if (rule != null && rule.length() != 0) {
            if (rule.startsWith("+") || rule.startsWith("-")) {
                boolean allow = rule.startsWith("+");
                if (rule.charAt(1) == 'n' || rule.charAt(1) == 'i') {
                    add(new PatternRule(allow, rule.substring(1)));
                } else if (rule.charAt(1) == 'c') {
                    try {
                        add(new IpSubnetFilterRule(allow, rule.substring(3)));
                    } catch (UnknownHostException e) {
                        if (logger.isErrorEnabled()) {
                            InternalLogger internalLogger = logger;
                            internalLogger.error("error parsing ip filter " + rule, e);
                        }
                    }
                } else if (logger.isErrorEnabled()) {
                    InternalLogger internalLogger2 = logger;
                    internalLogger2.error("syntax error in ip filter rule:" + rule);
                }
            } else if (logger.isErrorEnabled()) {
                InternalLogger internalLogger3 = logger;
                internalLogger3.error("syntax error in ip filter rule:" + rule);
            }
        }
    }
}
