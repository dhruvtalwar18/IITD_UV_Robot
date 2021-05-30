package javax.jmdns.impl.tasks;

import java.util.HashSet;
import java.util.Set;
import java.util.Timer;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.jmdns.impl.DNSIncoming;
import javax.jmdns.impl.DNSOutgoing;
import javax.jmdns.impl.DNSQuestion;
import javax.jmdns.impl.DNSRecord;
import javax.jmdns.impl.JmDNSImpl;
import javax.jmdns.impl.constants.DNSConstants;

public class Responder extends DNSTask {
    static Logger logger = Logger.getLogger(Responder.class.getName());
    private final DNSIncoming _in;
    private final boolean _unicast;

    public Responder(JmDNSImpl jmDNSImpl, DNSIncoming in, int port) {
        super(jmDNSImpl);
        this._in = in;
        this._unicast = port != DNSConstants.MDNS_PORT;
    }

    public String getName() {
        StringBuilder sb = new StringBuilder();
        sb.append("Responder(");
        sb.append(getDns() != null ? getDns().getName() : "");
        sb.append(")");
        return sb.toString();
    }

    public String toString() {
        return super.toString() + " incomming: " + this._in;
    }

    public void start(Timer timer) {
        boolean iAmTheOnlyOne = true;
        for (DNSQuestion question : this._in.getQuestions()) {
            if (logger.isLoggable(Level.FINEST)) {
                Logger logger2 = logger;
                logger2.finest(getName() + "start() question=" + question);
            }
            iAmTheOnlyOne = question.iAmTheOnlyOne(getDns());
            if (!iAmTheOnlyOne) {
                break;
            }
        }
        int delay = (!iAmTheOnlyOne || this._in.isTruncated()) ? (JmDNSImpl.getRandom().nextInt(96) + 20) - this._in.elapseSinceArrival() : 0;
        if (delay < 0) {
            delay = 0;
        }
        if (logger.isLoggable(Level.FINEST)) {
            Logger logger3 = logger;
            logger3.finest(getName() + "start() Responder chosen delay=" + delay);
        }
        if (!getDns().isCanceling() && !getDns().isCanceled()) {
            timer.schedule(this, (long) delay);
        }
    }

    public void run() {
        getDns().respondToQuery(this._in);
        Set<DNSQuestion> questions = new HashSet<>();
        Set<DNSRecord> answers = new HashSet<>();
        if (getDns().isAnnounced()) {
            try {
                for (DNSQuestion question : this._in.getQuestions()) {
                    if (logger.isLoggable(Level.FINER)) {
                        Logger logger2 = logger;
                        logger2.finer(getName() + "run() JmDNS responding to: " + question);
                    }
                    if (this._unicast) {
                        questions.add(question);
                    }
                    question.addAnswers(getDns(), answers);
                }
                long now = System.currentTimeMillis();
                for (DNSRecord knownAnswer : this._in.getAnswers()) {
                    if (knownAnswer.isStale(now)) {
                        answers.remove(knownAnswer);
                        if (logger.isLoggable(Level.FINER)) {
                            Logger logger3 = logger;
                            logger3.finer(getName() + "JmDNS Responder Known Answer Removed");
                        }
                    }
                }
                if (!answers.isEmpty()) {
                    if (logger.isLoggable(Level.FINER)) {
                        Logger logger4 = logger;
                        logger4.finer(getName() + "run() JmDNS responding");
                    }
                    DNSOutgoing out = new DNSOutgoing(33792, !this._unicast, this._in.getSenderUDPPayload());
                    out.setId(this._in.getId());
                    for (DNSQuestion question2 : questions) {
                        if (question2 != null) {
                            out = addQuestion(out, question2);
                        }
                    }
                    for (DNSRecord answer : answers) {
                        if (answer != null) {
                            out = addAnswer(out, this._in, answer);
                        }
                    }
                    if (!out.isEmpty()) {
                        getDns().send(out);
                    }
                }
            } catch (Throwable e) {
                Logger logger5 = logger;
                Level level = Level.WARNING;
                logger5.log(level, getName() + "run() exception ", e);
                getDns().close();
            }
        }
    }
}
