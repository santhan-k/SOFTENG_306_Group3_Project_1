package wg;

import org.apache.commons.configuration.Configuration;
import org.apache.commons.configuration.ConfigurationException;
import org.apache.commons.configuration.PropertiesConfiguration;

public class WGConfig {
    
    public static final String CONFIGURATION_FILE = "wg.cfg";
    public static Configuration configuration;
    
    public static void load() {
        String config_file = CONFIGURATION_FILE;
        try {
            configuration = new PropertiesConfiguration(config_file);
        } catch(ConfigurationException e) {
            System.out.println("[WGConfig] Error while loading config file: " + e.getLocalizedMessage());
        }
    }
    
    // Special get string that will return null if key is not found
    public static String getString(String key) {
        String s = configuration.getString(key, null);

        if(s == null || s.trim().equals("")) {
            return null;
        } else {
            return s;
        }
    }
}