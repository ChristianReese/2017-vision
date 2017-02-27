package first.frc.team2077.season2017.vision.trackers;

import org.opencv.core.Point;

public class Vertex 
{
	private Point pt;
	private double absAngle;
	
	public Vertex(int x, int y, double absAngle) {
		super();
		this.pt = new Point( x, y );
		this.absAngle = absAngle;
	}
	
	/**
	 * @return the x
	 */
	public int getX() {
		return (int) pt.x;
	}
	
	/**
	 * @return the y
	 */
	public int getY() {
		return (int) pt.y;
	}

	/**
	 * @return the point (x,y)
	 */
	public Point getPoint() {
		return pt;
	}

	/**
	 * @return the absAngle
	 */
	public double getAbsAngle() {
		return absAngle;
	}
	
	public boolean tooCloseTo(Vertex other) 
	{
		if (this == other)
			return true;
		if (other == null)
			return false;

		double xDist = other.pt.x - this.pt.x;
		double yDist = other.pt.y - this.pt.y;
		double dist = Math.sqrt( ( xDist * xDist ) + ( yDist * yDist ) );
		
		return ( dist < 5.0 );
	}
}
