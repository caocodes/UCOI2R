package finalProject;

import lejos.robotics.navigation.Move;
import lejos.robotics.navigation.MoveListener;
import lejos.robotics.navigation.MoveProvider;

public class PilotMoveListener implements MoveListener {
	private MazeNode node = null;
	
	public void setNode(MazeNode node) {
		this.node = node;
	}
	
	@Override
	public void moveStarted(Move event, MoveProvider mp) {}

	@Override
	public void moveStopped(Move event, MoveProvider mp) {
		if(node != null) {
			node.addMove(event);
		}
	}
}
