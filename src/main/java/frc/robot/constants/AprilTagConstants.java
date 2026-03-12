package frc.robot.constants;

/**
 * Contains constants related to AprilTag IDs and their physical measurements on the field.
 * These are used for vision processing and autonomous navigation.
 */
public class AprilTagConstants {
    public static final int[] kRedHubTags = {8,10,11};
    public static final int[] kBlueHubTags = {24,25,26,27};
    public static final int[] kRedTrenchInTags = {1,6};
    public static final int[] kBlueTrenchInTags = {17,22};
    public static final int[] kRedTrenchOutTags = {7,12};
    public static final int[] kBlueTrenchOUtTags = {23,28};
    public static final int[] kRedTowerTags = {15,16};
    public static final int[] kBlueTowerTags = {31,32};
    public static final int[] kRedDepotTags = {13,14};
    public static final int[] kBlueDepotTags = {29,30};

    public static final double kHubTagHeights = 44.25;
    public static final double kTrenchTagHeights = 35.0;
    public static final double kTowerTagHeights = 21.75;
    public static final double kDepotTagHeights = 21.75;
}