package frc.robot.helpers;

public class Helpers {
	public static long TimeFunction(Runnable func) {
		long startTime = System.currentTimeMillis();
		func.run();
		return System.currentTimeMillis() - startTime;
	}

}
