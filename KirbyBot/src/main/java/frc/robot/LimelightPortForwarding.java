import edu.wpi.first.net.PortForwarder;

public class LimelightPortForwarding () {
    PortForwarder PortForwarder = new PortForwarder();
    public void portForward () {
        // (robotIP):5801 will now point to a Limelight3A's (id 0) web interface stream:
        // (robotIP):5800 will now point to a Limelight3A's (id 0) video stream:
        PortForwarder.getInstance().add(5801, "172.29.0.1", 5801);
        PortForwarder.getInstance().add(5802, "172.29.0.1", 5802);
        PortForwarder.getInstance().add(5803, "172.29.0.1", 5803);
        PortForwarder.getInstance().add(5804, "172.29.0.1", 5804);
        PortForwarder.getInstance().add(5805, "172.29.0.1", 5805);
        PortForwarder.getInstance().add(5806, "172.29.0.1", 5806);
        PortForwarder.getInstance().add(5807, "172.29.0.1", 5807);
        PortForwarder.getInstance().add(5808, "172.29.0.1", 5808);
        PortForwarder.getInstance().add(5809, "172.29.0.1", 5809);

        // (robotIP):5811 will now point to a Limelight3A's (id 1) web interface stream:
        // (robotIP):5810 will now point to a Limelight3A's (id 1) video stream:
        PortForwarder.getInstance().add(5811, "172.29.1.1", 5801);
        PortForwarder.getInstance().add(5812, "172.29.1.1", 5802);
        PortForwarder.getInstance().add(5813, "172.29.1.1", 5803);
        PortForwarder.getInstance().add(5814, "172.29.1.1", 5804);
        PortForwarder.getInstance().add(5815, "172.29.1.1", 5805);
        PortForwarder.getInstance().add(5816, "172.29.1.1", 5806);
        PortForwarder.getInstance().add(5817, "172.29.1.1", 5807);
        PortForwarder.getInstance().add(5818, "172.29.1.1", 5808);
        PortForwarder.getInstance().add(5819, "172.29.1.1", 5809);
    }
}