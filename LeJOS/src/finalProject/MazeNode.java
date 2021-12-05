package finalProject;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

import lejos.robotics.navigation.Move;

public class MazeNode {
	private float x;
	private float y;
	private boolean visited = false;
	private MazeNode cameFrom = null; // Node previously connected to this
	private List<Move> moves = new ArrayList<>(); // Moves to get from predecessor to this node
	private Set<MazeNode> neighbors = new LinkedHashSet<>();
	
	public MazeNode(float x, float y) {
		this.x = x;
		this.y = y;
	}
	
	public MazeNode(float x, float y, MazeNode cameFrom) {
		this(x,y);
		this.cameFrom = cameFrom;
	}
	
	public float getX() {
		return x;
	}
	
	public void setX(float x) {
		this.x = x;
	}
	
	public float getY() {
		return y;
	}
	
	public void setY(float y) {
		this.y = y;
	}
	
	public void setVisited(boolean visited) {
		this.visited = visited;
	}
	
	public boolean isVisited() {
		return visited;
	}
	
	public boolean isExitNode(float mazeWidth, float mazeHeight) {
		return x < 0 || x > mazeWidth || y > mazeHeight;
	}
		
	public List<Move> getMoves() {
		return moves;
	}
	
	public void addMove(Move move) {
		moves.add(move);
	}
	
	public Set<MazeNode> getNeighbors() {
		return neighbors;
	}
	
	public int getNeighborsCount() {
		return neighbors.size();
	}
	
	public boolean isOrigin() {
		return cameFrom == null;
	}
	
	public MazeNode getCameFrom() {
		return cameFrom;
	}
	
	public void setCameFrom(MazeNode cameFrom) {
		this.cameFrom = cameFrom;
	}
	
	/**
	 * Bi-directionally add neighbors
	 */
	public boolean addNeighbor(MazeNode otherNode) {
		otherNode.getNeighbors().add(this);
		return this.neighbors.add(otherNode);
	}
	
	public boolean removeNeighbor(MazeNode otherNode) {
		return neighbors.remove(otherNode);
	}
	
	/**
	 * Checks if all neighbors have been visited
	 */
	public boolean visitedAllNeighbors() {
		for(MazeNode neighbor: neighbors) {
			if(!neighbor.isVisited()) {
				return false;
			}
		}
		return true;
	}
	
	/**
	 * @return first unvisited neighbor
	 */
	public MazeNode getUnvisitedNeighbor() {
		for(MazeNode neighbor: neighbors) {
			if(!neighbor.isVisited()) {
				return neighbor;
			}
		}
		return null;
	}
	
	@Override
	public String toString() {
		return String.format("Node: (%.3f,%.3f), Visited: %b", x,y,visited);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + Float.floatToIntBits(x);
		result = prime * result + Float.floatToIntBits(y);
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		MazeNode other = (MazeNode) obj;
		if (Float.floatToIntBits(x) != Float.floatToIntBits(other.x))
			return false;
		if (Float.floatToIntBits(y) != Float.floatToIntBits(other.y))
			return false;
		return true;
	}
}
