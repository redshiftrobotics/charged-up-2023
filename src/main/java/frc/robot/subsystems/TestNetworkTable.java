package frc.robot.subsystems;

import java.io.Console;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestNetworkTable extends SubsystemBase {
	private final DoubleSubscriber subscriber;
	private final DoublePublisher publisher;

	public TestNetworkTable(DoubleTopic topic) {
		this.subscriber = topic.subscribe(0);
		this.publisher = topic.publish();

	}

	@Override
	public void periodic() {
		double value = subscriber.get();
		publisher.set(value + 1);
		SmartDashboard.putNumber("Networktables value",value+1 ) ;
	}

}
