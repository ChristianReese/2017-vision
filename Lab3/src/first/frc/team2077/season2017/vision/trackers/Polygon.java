package first.frc.team2077.season2017.vision.trackers;

import java.util.List;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.NoSuchElementException;

import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
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
	
	private List<Point> points;
	
	public static Polygon constructPotentialTargetRectangle( Point[] basePolygon, Point[] overlapPolygon,
			int[] hullIndices, int[] overlapIndices, Mat output )
	{		
		LineSegment end1, end2;
		
		List<Point> resultPoints = new ArrayList<>();
		
		ArrayList<Integer> endLineSegmentIndices = findEndSegmentsIndices( 
						new Polygon( overlapPolygon, hullIndices ), hullIndices, overlapPolygon.length );
		
		if ( endLineSegmentIndices.size() >= 4 )
		{
			end1 = getLongestBaseLineSegment( basePolygon, 
					overlapIndices[endLineSegmentIndices.get(1)], 
					overlapIndices[endLineSegmentIndices.get(0)], output );
			end2 = getLongestBaseLineSegment( basePolygon, 
					overlapIndices[endLineSegmentIndices.get(3)], 
					overlapIndices[endLineSegmentIndices.get(2)], output );

			if ( ( end1 != null ) && ( end2 != null ) )
			{
				//end1.draw(3, output);
				//end2.draw(3, output);
		
				resultPoints.add( end1.getPt1() );
				resultPoints.add( end1.getPt2() );
				resultPoints.add( end2.getPt1() );
				resultPoints.add( end2.getPt2() );
				
				return new Polygon( resultPoints );
			}
		}
		
		return null;//new Polygon( overlapPolygon );
	}
	
	public Polygon( List<Point> inputPoints )
	{
		points = new ArrayList<>();
		
		for ( Point point : inputPoints )
		{
			points.add( new Point( point.x, point.y ) );
		}
	}
	
	public Polygon( Point[] inputPoints )
	{
		points = new ArrayList<>();
		
		for ( Point point : inputPoints )
		{
			points.add( new Point( point.x, point.y ) );
		}
	}
	
	public Polygon( Point[] inputPoints, int[] indices )
	{
		int inputPointsIdx = 0;
		
		points = new ArrayList<>();
		
		for ( int i = 0; i < indices.length; ++i )
		{
			try 
			{
				points.add( new Point( inputPoints[indices[i]].x, inputPoints[indices[i]].y ) );
			} 
			catch( ArrayIndexOutOfBoundsException ex )
			{
				ex.printStackTrace();
			}
		}
	}
	
	public void draw( Mat output )
	{
		for ( LineSegment ls : this )
		{
			ls.draw( 1, output );
			Utility.drawPoint( ls.getPt1(), Utility.white, 2, output );
			Utility.drawPoint( ls.getPt2(), Utility.white, 2, output );
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
	
	public LineSegment[] toLineSegmentArray()
	{
		LineSegment[] result = new LineSegment[points.size()];
		int i = 0;
		
		for ( LineSegment ls : this )
		{
			result[i++] = ls;
		}
		
		return result;
	}
	
	/**
	 * @param lsIndex Index of the line segment of the hull being evaluated
	 * @param hullIndices Indices of the hull on the polygon
	 * @param numTotalIndices Total number of points of the polygon that was hulled.
	 * @return True if the specified line segment is crossing a convexity defect, false otherwise.
	 */
	private static boolean isAConvexityDefect( int lsIndex, int[] hullIndices, int numTotalPoints )
	{
		int index1 = (int)Utility.mod( lsIndex, hullIndices.length );
		int index2 = (int)Utility.mod( lsIndex + 1, hullIndices.length );
		//int difference = (int)Utility.mod( hullIndices[ index2 ] - hullIndices[ index1 ], numTotalPoints );
		int difference = Math.abs( hullIndices[ index2 ] - hullIndices[ index1 ] );
		
		if ( difference >= ( numTotalPoints / 2 ) )
		{
			difference -= numTotalPoints;
			difference = Math.abs( difference );
		}
		
		return ( difference > 1 );
	}
	
	private static ArrayList<Integer> findEndSegmentsIndices( Polygon hullPolygon, int[] hullIndices, int numTotalPoints )
	{
		final double MAX_INTERPOLE_DIFFERENCE_TOLERANCE = 0.1;//0.5;
		
		final double MAX_LENGTH_DIFFERENCE = 10.0;//5.0; // pixels
		
		// Target dimension ratio: 5" / 2" = 2.5
		final double MIN_SIDE_RATIO = 1.0;//1.5; 
		final double MAX_SIDE_RATIO = 5.5;
		
		class LSPair
		{
			public int ls1Idx1;
			public int ls1Idx2;
			public int ls2Idx1;
			public int ls2Idx2;
			
			public double totalLength;
			
			public double interPolarDistanceDifference;
		}
		
		LineSegment[] segments = hullPolygon.toLineSegmentArray();
		ArrayList<LSPair> pairs = new ArrayList<>();
		LSPair bestLSPair = null;
		ArrayList<Integer> result = new ArrayList<>();
		
		for ( int i = 0; i < segments.length; ++i )
		{
			if ( !isAConvexityDefect( i, hullIndices, numTotalPoints ) )
			{
				for ( int j = 2; j <= segments.length - 2; ++j )
				{
					int adjustedIndex = (int)Utility.mod( j + i, segments.length );
					
					//if ( !segments[ i ].equals( segments[ adjustedIndex ] ) )
					{
						if ( !isAConvexityDefect( adjustedIndex, hullIndices, numTotalPoints ) )
						{
							double ls1Length = segments[i].calculateLength();
							double ls2Length = segments[adjustedIndex].calculateLength();

							LineSegment interPole1 = new LineSegment( segments[i].getPt1(), segments[adjustedIndex].getPt1() );
							LineSegment interPole2 = new LineSegment( segments[i].getPt2(), segments[adjustedIndex].getPt2() );
							
							Utility.correctIntersectingLSPair( interPole1, interPole2 );

							double interPole1Length = interPole1.calculateLength();
							double interPole2Length = interPole2.calculateLength();
							
							if ( Math.abs( ls2Length - ls1Length ) < MAX_LENGTH_DIFFERENCE )
							{
								double interpoleDistanceScaled = Math.abs( interPole2Length - interPole1Length )
										/ ( ls1Length + ls2Length );
								
								//System.out.println( interpoleDistanceScaled );
								
								if ( interpoleDistanceScaled < MAX_INTERPOLE_DIFFERENCE_TOLERANCE )
								{
									double sideRatio = ( interPole1Length + interPole2Length ) / ( ls1Length + ls2Length );
									if ( ( sideRatio >= MIN_SIDE_RATIO ) && ( sideRatio <= MAX_SIDE_RATIO ) )
									{
										//System.out.println(sideRatio);
										LSPair lsPair = new LSPair();
			
										lsPair.ls1Idx1 = hullIndices[ ( int ) Utility.mod( i, hullIndices.length ) ]; 
										lsPair.ls1Idx2 = hullIndices[ ( int ) Utility.mod( i + 1, hullIndices.length ) ]; 
										lsPair.ls2Idx1 = hullIndices[ ( int ) Utility.mod( adjustedIndex, hullIndices.length ) ]; 
										lsPair.ls2Idx2 = hullIndices[ ( int ) Utility.mod( adjustedIndex + 1, hullIndices.length ) ];
										
										lsPair.totalLength = ls1Length + ls2Length;
										
										pairs.add( lsPair );
									}
								}
							}
						}
					}
				}
			}
		}
		
		for ( LSPair currentPair : pairs )
		{
			if ( bestLSPair == null )
			{
				bestLSPair = currentPair;
			}
			else if ( currentPair.totalLength < bestLSPair.totalLength )
			{
				bestLSPair = currentPair;
			}
		}
		
		if ( bestLSPair != null )
		{
			result.add( bestLSPair.ls1Idx1 );
			result.add( bestLSPair.ls1Idx2 );
			result.add( bestLSPair.ls2Idx1 );
			result.add( bestLSPair.ls2Idx2 );
		}
		
		return result;
	}
	
	private static LineSegment getLongestBaseLineSegment( Point[] basePolygonPoints, int min, int max, Mat drawOutput )
	{
		final double MIN_LENGTH_FRACTION = 0.7;
		
		LineSegment bridge = new LineSegment( basePolygonPoints[min], basePolygonPoints[max] );	
		
		LineSegment longest = null;
		double longestLength = -1.0;
		
		int pointsInRange = max - min;
		
		if ( pointsInRange < 0 )
		{
			pointsInRange += basePolygonPoints.length;
		}
		
		for ( int i = 0; i < pointsInRange; ++i )
		{
			LineSegment current = new LineSegment( 
					basePolygonPoints[ (int)Utility.mod( min + i, basePolygonPoints.length ) ], 
					basePolygonPoints[ (int)Utility.mod( min + i + 1, basePolygonPoints.length ) ] );
			
			double currentLength = current.calculateLength();
			
			if ( currentLength > longestLength )
			{
				longest = current;
				longestLength = currentLength;
			}
		}
			
		//bridge.draw(3, drawOutput, Utility.yellow);
		
		//System.out.println( min + " " + (min+pointsInRange) );
		
		if ( longest != null )
		{
			if ( ( longestLength / bridge.calculateLength() ) >= MIN_LENGTH_FRACTION )
			{
				return longest;
			}
		}
		
		return null;//bridge;
	}
}
