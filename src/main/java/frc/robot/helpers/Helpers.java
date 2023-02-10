package frc.robot.helpers;

public class Helpers {

	/**
	 * A helper function that runs and times another method.
	 * @param func a Runnable to run and time, pass this via ClassName::FunctionName or a lambda.
	 * @return the time in milliseconds that the function took to run.
	 */
	public static long TimeFunction(Runnable func) {
		long startTime = System.currentTimeMillis();
		func.run();
		return System.currentTimeMillis() - startTime;
	}

}
