package first.frc.team2077.season2017.vision.trackers;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.NoSuchElementException;

import org.opencv.core.Mat;
import org.opencv.core.Point;

public class Polygon implements Iterable< LineSegment >
{
	private static class PolygonLineSegmentIterator implements Iterator< LineSegment >
	{
		Iterator< Point > pointsIter = null;
		
		Point firstPoint = null;
		Point ptA = null;
		Point ptB = null;
		
		boolean hasNextFlag = false;
		
		static PolygonLineSegmentIterator makeIter( Iterator< Point > pointsIter )
		{
			PolygonLineSegmentIterator result = new PolygonLineSegmentIterator();
			
			if ( pointsIter != null )
			{
				if ( pointsIter.hasNext() )
				{
					result.firstPoint = pointsIter.next();
					result.ptA = result.firstPoint;
					
					result.hasNextFlag = pointsIter.hasNext();
					
					result.pointsIter = pointsIter;
				}
			}
			
			return result;
		}

		@Override
		public boolean hasNext() 
		{
			return hasNextFlag;
		}

		@Override
		public LineSegment next() 
		{
			if ( hasNextFlag == false )
			{
				throw new NoSuchElementException();
			}
			
			if ( ptB != null )
			{
				ptA = ptB;
			}
			
			if ( pointsIter.hasNext() )
			{
				ptB = pointsIter.next();
			}
			else
			{
				ptB = firstPoint;
				hasNextFlag = false;
			}
			
			return new LineSegment( ptA, ptB );
		}
		
	}
	
	private ArrayList<Point> points;
	
	public static Polygon createPolygon( ArrayList<Point> inputPoints )
	{
		Polygon result = new Polygon();
		result.points = new ArrayList<>();
		
		for ( Point point : inputPoints )
		{
			result.points.add( new Point( point.x, point.y ) );
		}
		
		return result;
	}
	
	public void draw( Mat output )
	{
		for ( LineSegment ls : this )
		{
			ls.draw( output );
			Utility.drawPoint( ls.getPt2(), Utility.white, 1, output );
		}
	}

	@Override
	public Iterator< LineSegment > iterator() 
	{
		if ( this.points != null )
		{
			return PolygonLineSegmentIterator.makeIter( this.points.iterator() );
		}
		
		return PolygonLineSegmentIterator.makeIter( null );
	}
}
