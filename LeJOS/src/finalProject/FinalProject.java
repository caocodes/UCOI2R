package finalProject;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Iterator;

import lejos.hardware.Button;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.robotics.Color;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.utility.Delay;

public class FinalProject {
	static Deque<MazeNode> nodes = new ArrayDeque<>();
	static Deque<MazeNode> exitPlan = new ArrayDeque<>();

	public static final float NODE_SIZE = 47.8f;
	public static final float MAZE_LENGTH = NODE_SIZE * 2;
	public static final float MAZE_WIDTH = NODE_SIZE * 2;
	public static final float MOVE_INC = NODE_SIZE / 2; // TODO check me. move increments in mm, stops midway between
														// two nodes and look for marker to ensure robot is centered
	public static final float MOVE_TOL = 10; // move tolerance mm
	public static final float HEAD_TOL = 15; // heading tolerance degrees
	public static final int BALL_COLOR = Color.BLUE; // 2
	public static final int EXIT_COLOR = Color.RED; // 0
	// TODO check my headings
	public static final float LEFT_HEADING = 0; // degs
	public static final float FWD_HEADING = 90; // degs
	public static final float RIGHT_HEADING = 180; // degs
	public static final float BACK_HEADING = 270; // degs

	public static void main(String[] args) {
		Pilot pilot = new Pilot();
		Navigator nav = new Navigator(pilot);
		TheClaw theClaw = new TheClaw(MotorPort.C);

		UltrasonicSensor ussr = new UltrasonicSensor(SensorPort.S1);
		ColorSensor colorSensor = new ColorSensor(SensorPort.S4);
		Mast mast = new Mast(MotorPort.B);

		MazeNode currentNode = new MazeNode(0, 0); // center of the left-bottom-most node
		currentNode.setVisited(true);
		nodes.push(currentNode);
		System.out.println("Node stack: " + nodes);
		System.out.println("Current node: " + currentNode);

		MazeNode frontNode = null;
		MazeNode leftNode = null;
		MazeNode rightNode = null;

		OdometryPoseProvider poseProvider = new OdometryPoseProvider(pilot);
		poseProvider.setPose(new Pose(currentNode.getX(), currentNode.getY(), pilot.getHeading()));
		nav.setPoseProvider(poseProvider);
		// TODO @soups might need to add gyro sensor here for better heading detection

		PilotMoveListener moveListener = new PilotMoveListener();
		pilot.addMoveListener(moveListener);

		// algorithm explaination:
		// https://youtu.be/Yzsp6l-neGo
		// https://youtu.be/GEpEoliVNNY
		while (!Button.ESCAPE.isDown()) {
			// Check ball
			int color = colorSensor.getColor();
			System.out.println("Check ball color: " + color);
			if ((color == BALL_COLOR) || (color == 1)) {
				pilot.travel(5);
				theClaw.grab();
				pilot.travel(-5);
				if (Math.abs(theClaw.checkRotation() - theClaw.getClosedRotation()) < 5) {
					pilot.setBallCaptured(true);

					if (!exitPlan.isEmpty()) {
						System.out.println("Exit plan: " + exitPlan);
						exit(pilot, theClaw);
						break;
					}
				}
			}

			// Look for neighbors: front, left right
			mast.lookFront();
			System.out.println("Look front");
			int distance = ussr.distance();
			System.out.println("distance: " + distance);
			if (distance > NODE_SIZE) {
				frontNode = makeFrontNode(pilot.getHeading(), currentNode.getX(), currentNode.getY());
				if (frontNode != null) {
					System.out.println("front node made: " + frontNode);
					currentNode.addNeighbor(frontNode);
				}
			}

			mast.lookLeft();
			System.out.println("Look left");
			distance = ussr.distance();
			System.out.println("distance: " + distance);
			if (distance > NODE_SIZE) {
				leftNode = makeLeftNode(pilot.getHeading(), currentNode.getX(), currentNode.getY());
				if (leftNode != null) {
					System.out.println("left node made: " + leftNode);
					currentNode.addNeighbor(leftNode);
				}
			}

			mast.lookRight();
			System.out.println("Look right");
			distance = ussr.distance();
			System.out.println("distance: " + distance);
			if (distance > NODE_SIZE) {
				rightNode = makeRightNode(pilot.getHeading(), currentNode.getX(), currentNode.getY());
				if (rightNode != null) {
					System.out.println("right node made: " + rightNode);
					currentNode.addNeighbor(rightNode);
				}
			}

			mast.lookFront(); // in case robots die

			MazeNode nextNode = currentNode.getUnvisitedNeighbor();
			if (nextNode == null) { // deadend or visited all neighbors already
				System.out.println("dead end / visited all");
				nextNode = retrace(pilot);
				currentNode = nodes.peek();
				System.out.println("retrace done");
				System.out.println("current node: " + currentNode);
				System.out.println("next node: " + nextNode);
			} else { // not deadend
				// check exit
				color = colorSensor.getColor();
				System.out.println("Check exit color: " + color);
				if (currentNode.isExitNode(MAZE_WIDTH, MAZE_LENGTH) || color == EXIT_COLOR) {
					System.out.println("Exited maze");
					if (pilot.isBallCaptured()) {
						System.out.println("Ball captured. Try claw");
						theClaw.release();
						break;
					} else {
						System.out.println("Ball not captured. Make exit plan");
						exitPlan.addAll(nodes);
						nextNode = retrace(pilot);
						currentNode = nodes.peek();
						System.out.println("retrace done");
						System.out.println("current node: " + currentNode);
						System.out.println("next node: " + nextNode);
					}
				}
			}

			nextNode.setCameFrom(currentNode);
			nodes.push(nextNode);
			System.out.println("Node stack: " + nodes);
			moveListener.setNode(nextNode);
			pilot.goTo(currentNode, nextNode);
			while (pilot.isMoving()) {
				Delay.msDelay(1000);
			}
			System.out.println("Finished moving");
			currentNode = nextNode;
			currentNode.setVisited(true);
			System.out.println("Heading: " + pilot.getHeading());
			System.out.println("current node: " + currentNode);
			Delay.msDelay(7000);
		}
	}

	/**
	 * @param heading
	 * @param x
	 * @param y
	 * @return node front of robot based on its heading
	 */
	public static MazeNode makeFrontNode(float heading, float x, float y) {
		if (inRange(heading, FWD_HEADING, HEAD_TOL)) { // facing forward 90 degs. front node is north
			return new MazeNode(x, y + NODE_SIZE);
		} else if (inRange(heading, LEFT_HEADING, HEAD_TOL)) { // facing left 0 degs. front node is west
			return new MazeNode(x - NODE_SIZE, y);
		} else if (inRange(heading, RIGHT_HEADING, HEAD_TOL)) { // facing right. 180 degs. front node is east
			return new MazeNode(x + NODE_SIZE, y);
		} else if (inRange(heading, BACK_HEADING, HEAD_TOL)) {
			return new MazeNode(x, y - NODE_SIZE);
		} else {
			System.out.println("ERROR: Heading OUT OF RANGE");
			return null;
		}
	}

	/**
	 * 
	 * @param heading
	 * @param x
	 * @param y
	 * @return node left of robot based on its heading
	 */
	public static MazeNode makeLeftNode(float heading, float x, float y) {
		if (inRange(heading, FWD_HEADING, HEAD_TOL)) { // facing forward 90 degs. left node is west
			return new MazeNode(x - NODE_SIZE, y);
		} else if (inRange(heading, LEFT_HEADING, HEAD_TOL)) { // facing left 0 degs. left node is south
			return new MazeNode(x, y - NODE_SIZE);
		} else if (inRange(heading, RIGHT_HEADING, HEAD_TOL)) { // facing right 180 degs. left node is north
			return new MazeNode(x, y + NODE_SIZE);
		} else if (inRange(heading, BACK_HEADING, HEAD_TOL)) {
			return new MazeNode(x + NODE_SIZE, y);
		} else {
			System.out.println("ERROR: Heading OUT OF RANGE");
			return null;
		}
	}

	/**
	 * 
	 * @param heading
	 * @param x
	 * @param y
	 * @return node right of robot based on its heading
	 */
	public static MazeNode makeRightNode(float heading, float x, float y) {
		if (inRange(heading, FWD_HEADING, HEAD_TOL)) { // facing forward 90 degs. right node is east
			return new MazeNode(x + NODE_SIZE, y);
		} else if (inRange(heading, LEFT_HEADING, HEAD_TOL)) { // facing left 0 degs. right node is north
			return new MazeNode(x, y + NODE_SIZE);
		} else if (inRange(heading, RIGHT_HEADING, HEAD_TOL)) { // facing right 180 degs. right node is south
			return new MazeNode(x, y - NODE_SIZE);
		} else if (inRange(heading, BACK_HEADING, HEAD_TOL)) {
			return new MazeNode(x - NODE_SIZE, y);
		} else {
			System.out.println("ERROR: Heading OUT OF RANGE");
			return null;
		}
	}

	/**
	 * @param value
	 * @param target
	 * @param tolerance
	 * @return true if value is within range of target while accounting for
	 *         tolerance
	 */
	public static boolean inRange(float value, float target, float tolerance) {
		return (target - tolerance) <= value && value <= (target + tolerance);
	}

	/**
	 * move back to first node in stack that has unvisited neighbor
	 */
	public static MazeNode retrace(Pilot pilot) {
		MazeNode currentNode = nodes.peek();
		while (currentNode.visitedAllNeighbors()) {
			currentNode = nodes.pop();
			MazeNode nextNode = nodes.peek();
			pilot.goTo(currentNode, nextNode);
			while (pilot.isMoving()) {
				Delay.msDelay(1000);
			}
			System.out.println("retraced move");
			currentNode = nextNode;
		}
		return nodes.peek().getUnvisitedNeighbor();
	}

	/**
	 * exit the maze
	 * 
	 * @param pilot
	 * @param theClaw
	 */
	public static void exit(Pilot pilot, TheClaw theClaw) {
		if (exitPlan.isEmpty() || nodes.isEmpty())
			return;
		Iterator<MazeNode> exitIt = exitPlan.descendingIterator();
		Iterator<MazeNode> nodeIt = nodes.descendingIterator();
		MazeNode common = null;
		while (exitIt.hasNext()) {
			MazeNode exitNode = exitIt.next();
			MazeNode node = nodeIt.next();
			if (exitNode.equals(node)) {
				common = exitNode;
				exitPlan.removeLast();
				nodes.removeLast();
			}
		}

		if (common != null) {
			exitPlan.addLast(common);
		}
		while (!exitPlan.isEmpty()) {
			nodes.addLast(exitPlan.removeLast());
		}

		while (nodes.size() > 1) {
			MazeNode currentNode = nodes.pop();
			MazeNode nextNode = nodes.peek();
			pilot.goTo(currentNode, nextNode);
			while (pilot.isMoving()) {
				Delay.msDelay(1000);
			}
		}
		theClaw.release();
	}
}
