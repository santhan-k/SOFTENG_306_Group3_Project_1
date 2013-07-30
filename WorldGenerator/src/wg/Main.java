package wg;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class Main {
    
    private static int numRobots;
    private static String robotName;
    private static String fileName;
    private static PrintWriter out;
    
    public static void main(String [] args) {
        if(doesFileExist(WGConfig.CONFIGURATION_FILE)) {
            // Load settings
            WGConfig.load();
            numRobots = WGConfig.configuration.getInteger("number_of_robots", 1); //default to 1 robot
            robotName = WGConfig.configuration.getString("robot_name", "era"); //default name is era
            fileName = WGConfig.configuration.getString("file_name", "generated_world.world"); //default file name is generated_world.world
        } else {
            System.out.println("[Main] Could not find " + WGConfig.CONFIGURATION_FILE + ", using default number of robots of 1 and robot name \"era\"");
            numRobots = 1;
        }
        
        // Init PrintWriter
        try {
            // FileWriter false argument makes it overwrite previously written file
            // PrintWriter true argument makes it auto flush
            out = new PrintWriter(new FileWriter(FILE_NAME, false), true);
        } catch (IOException e) {
            System.out.println("[Main] Error when initiating PrintWriter: " + e.getLocalizedMessage());
            e.printStackTrace();
        }
        
        // Print world file
        printWorld();
    }
    
    public static void printWorld() {
        if(out != null) {
            out.println(
                        "define block model\n" +
                        "(\n" +
                        "  size [0.5 0.5 0.5]\n" +
                        "  gui_nose 0\n" +
                        ")\n" +
                        "\n" +
                        "define topurg laser\n" +
                        "(\n" +
                        "  range_max 30.0\n" +
                        "  fov 270.25\n" +
                        "  samples 1081\n" +
                        "  # generic model properties\n" +
                        "  color \"black\"\n" +
                        "  size [ 0.05 0.05 0.1 ]\n" +
                        ")\n" +
                        "\n" +
                        "define erratic position\n" +
                        "(\n" +
                        "  #size [0.415 0.392 0.25]\n" +
                        "  size [0.35 0.35 0.25]\n" +
                        "  origin [-0.05 0 0 0]\n" +
                        "  gui_nose 1\n" +
                        "  drive \"diff\"\n" +
                        "  topurg(pose [ 0.050 0.000 0 0.000 ])\n" +
                        ")\n" +
                        "define floorplan model\n" +
                        "(\n" +
                        "  # sombre, sensible, artistic\n" +
                        "  color \"gray30\"\n" +
                        "\n" +
                        "  # most maps will need a bounding box\n" +
                        "  boundary 1\n" +
                        "\n" +
                        "  gui_nose 0\n" +
                        "  gui_grid 0\n" +
                        "\n" +
                        "  gui_outline 0\n" +
                        "  gripper_return 0\n" +
                        "  fiducial_return 0\n" +
                        "  laser_return 1\n" +
                        ")\n" +
                        "\n" +
                        "# set the resolution of the underlying raytrace model in meters\n" +
                        "resolution 0.02\n" +
                        "\n" +
                        "interval_sim 100  # simulation timestep in milliseconds\n" +
                        "\n" +
                        "window\n" +
                        "(\n" +
                        "  size [ 745.000 448.000 ]\n" +
                        "\n" +
                        "  rotate [ 0.000 -1.560 ]\n" +
                        "  scale 28.806\n" +
                        ")\n" +
                        "\n" +
                        "# load an environment bitmap\n" +
                        "floorplan\n" +
                        "(\n" +
                        "  name \"willow\"\n" +
                        "  bitmap \"willow-full.pgm\"\n" +
                        "  size [54.0 58.7 0.5]\n" +
                        "  pose [ -29.350 27.000 0 90.000 ]\n" +
                        ")\n" +
                        "\n" +
                        "# throw in two robots");
            if(numRobots > 0) {
                out.println("erratic( pose [ -11.277 23.266 0 180.000 ] name \"" + robotName + "\" color \"blue\")");
            }
            for(int i = 2; i <= numRobots; i++) {
                out.println("erratic( pose [ -11.277 23.266 0 180.000 ] name \"" + robotName + i + " color \"blue\")");
            }
            out.println("block( pose [ -13.924 25.020 0 180.000 ] color \"red\")");
        } else {
            System.out.println("[Main] PrintWriter not initialized");
        }
    }
    
    public static boolean doesFileExist(String fileName) {
        File f = new File(fileName);
        if(f.exists()) {
            return true;
        } else {
            return false;
        }
    }
}