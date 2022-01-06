package frc.robot;


import java.io.IOException;
import java.util.logging.FileHandler;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

public class Logging {

    public static Logger log;

    public static void initLogger() throws IOException {
        Logger logger = Logger.getLogger("Roborio");
        FileHandler fh = new FileHandler("/tmp/frc.log");
        logger.addHandler(fh);
        SimpleFormatter formatter = new SimpleFormatter();
        fh.setFormatter(formatter);
        log = logger;
    }

}
