package first.frc.team2077.season2017.vision.trackers;

import java.util.AbstractList;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Iterator;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class GearLiftTargetTracking 
{
	private static final int TRACE_PRECISION = 10;
	
	public static Mat processMat( Mat input, 
			double cameraWidth, double cameraHeight, 
			double hueMin, double hueMax, 
			double satMin, double satMax, 
			double lumMin, double lumMax, 
			boolean returnThresholdMat )
	{
		Mat output = input.clone();
		
		Mat thresholdMat = renderThreshold( input, hueMin, hueMax, satMin, satMax, lumMin, lumMax );
		ArrayList<Polygon> polygons = trackPolygons( input, thresholdMat );					
		ArrayList< CollinearLine > collinearLines = findCollinearLines( polygons );					
		TargetCandidate targetCandidate = getTargetCandidate( collinearLines, cameraWidth, cameraHeight );
		
		if ( targetCandidate != null )
		{
			targetCandidate.draw( output );
		}
		
		return ( returnThresholdMat ? thresholdMat : output );
	}
	
	private static Mat renderThreshold( Mat frame, 
			double hueMin, double hueMax, 
			double satMin, double satMax, 
			double lumMin, double lumMax )
	{
		/*double hueMin = ( hueMinSldr.getValue() );
		double hueMax = ( hueMaxSldr.getValue() );
		double satMin = ( satMinSldr.getValue() );
		double satMax = ( satMaxSldr.getValue() );
		double lumMin = ( lumMinSldr.getValue() );
		double lumMax = ( lumMaxSldr.getValue() );*/
		
		Mat thresholdMat = new Mat();
		
		Imgproc.cvtColor(frame, thresholdMat, Imgproc.COLOR_BGR2HLS);
		Core.inRange(thresholdMat, new Scalar(hueMin, lumMin, satMin), new Scalar(hueMax, lumMax, satMax), thresholdMat);
		
		return thresholdMat;
	}
	
	private static ArrayList<CollinearLine> findCollinearLines( ArrayList<Polygon> polygons )
	{
		ArrayList<CollinearLine> result = new ArrayList<>();
		
		for ( int i = 0; i < polygons.size(); ++i )
		{
			for ( int j = ( i + 1 ); j < polygons.size(); ++j )
			{
				findCollinearLines( polygons.get( i ), polygons.get( j ), result );
			}
		}
		
		return result;
	}
	
	private static TargetCandidate getTargetCandidate( ArrayList<CollinearLine> collinearLines,
			double cameraWidth, double cameraHeight )
	{
		TargetCandidate bestCandidate = null;
		
		for ( int i = 0; i < collinearLines.size(); ++i )
		{
			for ( int j = ( i + 1 ); j < collinearLines.size(); ++j )
			{
				TargetCandidate candidate = TargetCandidate.generateTargetCandidate( 
						collinearLines.get( i ), collinearLines.get( j ),
						cameraWidth, cameraHeight );
				
				if ( candidate != null )
				{
					if ( bestCandidate == null )
					{
						bestCandidate = candidate;
					}
					else if ( bestCandidate.getScore() < candidate.getScore() )
					{
						bestCandidate = candidate;
					}
				}
			}
		}
		
		if ( bestCandidate != null )
		{
			if ( bestCandidate.getScore() < TargetCandidate.MIN_SCORE )
			{
				return null;
			}
		}
		
		return bestCandidate;
	}
	
	private static void findCollinearLines( Polygon polygon1, Polygon polygon2, ArrayList<CollinearLine> result )
	{
		double dividendErrorDelta = 0.1;
		double divisorErrorDelta = 0.1;
		
		for ( LineSegment ls1 : polygon1 )
		{
			for ( LineSegment ls2 : polygon2 )
			{
				// TODO test here
				if ( ls1.isCollinearWith( ls2/*, dividendErrorDelta, divisorErrorDelta*/ ) )
				{
					result.add( CollinearLine.createCollinearLine( ls1, ls2, polygon1, polygon2 ) );
				}
			}
		}
	}
	
	private static ArrayList<Polygon> trackPolygons(Mat camFrame, Mat hlsMask)
	{
		ArrayList<Polygon> polygons = new ArrayList<>();
		
		boolean previousVal = false;
		boolean currentVal = false;
		boolean impact = false;
		//long prevTickCount = Core.getTickCount();
		//double elapsedTime = 0.0;
		
		Mat processedRecord = new Mat( hlsMask.rows(), hlsMask.cols(), hlsMask.type() );
		processedRecord.setTo( new Scalar( 0, 0, 0 ) );
		
		for ( int lineNumber = 0; lineNumber < TRACE_PRECISION; ++lineNumber )
		{
			int vertCoord = camFrame.height() * lineNumber / TRACE_PRECISION;
			
			for ( int horizCoord = 0; ( ( horizCoord < hlsMask.width() ) /*&& !impact*/ ); ++horizCoord )
			{
				previousVal = currentVal;
				currentVal = ( Utility.average( hlsMask.get( vertCoord, horizCoord ) ) > 0.01 );
				impact = ( currentVal != previousVal );
				
				//camFrame.put( vertCoord, horizCoord, yellow );
				
				if ( impact && ( Utility.average( processedRecord.get( vertCoord, horizCoord ) ) < 0.01 ) )
				{
					Polygon polygon = mapPolygon( horizCoord, vertCoord, camFrame, hlsMask, processedRecord );
					
					if ( polygon != null )
					{
						polygons.add( polygon );
					}
				}
			}
		}
		
		//elapsedTime = (double)( Core.getTickCount() - prevTickCount ) * 1000.0 / Core.getTickFrequency();
		//System.out.print("horizontalTrace time: ");
		//System.out.printf("%.5f", elapsedTime);
		//System.out.println(" ms");
		
		return polygons;
	}
	
	private static Polygon mapPolygon( int xImpactCoord, int yImpactCoord, Mat camFrame, 
			Mat hlsMask, Mat processedRecord )
	{
		Snapshot previousSnapshot = null;
		Snapshot currentSnapshot = null;
		ArrayList<Point> rawPointsArray = new ArrayList<>();
		int currentXCoord = xImpactCoord;
		int currentYCoord = yImpactCoord;
		int counter = 0;
		
		final int MAX_COUNT = 2250;
		
		try 
		{
			do
			{
				previousSnapshot = currentSnapshot;
				currentSnapshot = Snapshot.takeSnapshot( hlsMask, previousSnapshot, currentXCoord, currentYCoord );
				
				if ( currentSnapshot != null )
				{
					processedRecord.put( currentYCoord, currentXCoord, Utility.white );
					
					if ( ( Math.abs( currentSnapshot.getTurnAngle() ) > 60.0 )
							|| ( ( currentSnapshot.getTurnAngle() > 40.0 ) && currentSnapshot.isClockwise() )
							|| ( ( currentSnapshot.getTurnAngle() < -40.0 ) && !currentSnapshot.isClockwise() ) )
					{
						Vertex vertex = new Vertex( 
								currentSnapshot.getXCoord(), currentSnapshot.getYCoord(), 
								Math.abs( currentSnapshot.getTurnAngle() ) );
						rawPointsArray.add( vertex.getPoint() );
					}
	
					currentXCoord = currentSnapshot.getNextXCoord();
					currentYCoord = currentSnapshot.getNextYCoord();
				}
				
				++counter;
				
			} while ( ( currentSnapshot != null )
					&& ( ( currentXCoord != xImpactCoord ) || ( currentYCoord != yImpactCoord ) )
					&& ( counter < MAX_COUNT ) );
		} 
		catch ( ArrayIndexOutOfBoundsException ex )
		{
			ex.printStackTrace();
		}
		
		if ( !rawPointsArray.isEmpty() )
		{
			ArrayList<Point> pointFiltered = pointFilter( rawPointsArray );
			ArrayList<Point> lineFiltered = lineFilter( pointFiltered, ( camFrame.height() / ( TRACE_PRECISION * 3.0 ) ) );
				
			return Polygon.createPolygon( lineFiltered );
		}
		
		return null;
	}
	
	private static <E> E listWrappedGet( AbstractList<E> list, int index )
	{
		return list.get( Utility.mod( index, list.size() ) );
	}
	
	private static ArrayList<Point> lineFilter( ArrayList<Point> initialPointsArray, double minimumLineLength )
	{
		if ( initialPointsArray.size() > 2 )
		{
			Iterator<Point> iter = initialPointsArray.iterator();
			ArrayList<Point> filteredPointsArray = new ArrayList<>();

			Point firstPt = null;
			Point lastPt = null;
			Point pt1 = null;
			Point pt2 = iter.next();
			
			while ( iter.hasNext() )
			{
				pt1 = pt2;
				pt2 = iter.next();
				
				if ( Utility.getPointsDistance(pt1, pt2) >= minimumLineLength )
				{
					Point newPt1 = new Point( pt1.x, pt1.y );
					
					if ( !filteredPointsArray.isEmpty() )
					{
						Point previousPoint = filteredPointsArray.get( filteredPointsArray.size() - 1 );
						
						if ( Utility.getPointsDistance(previousPoint, pt1) < minimumLineLength )
						{
							filteredPointsArray.remove( filteredPointsArray.size() - 1 );
							
							newPt1.x = ( previousPoint.x + pt1.x ) / 2.0;
							newPt1.y = ( previousPoint.y + pt1.y ) / 2.0;
						}
					}
					
					filteredPointsArray.add( newPt1 );
					
					if ( firstPt == null )
					{
						firstPt = newPt1;
					}
					
					filteredPointsArray.add( new Point( pt2.x, pt2.y ) );
					
					lastPt = pt2;
				}
			}
			
			if ( ( firstPt != null ) &&( lastPt != null ) )
			{
				if ( Utility.getPointsDistance(lastPt, firstPt) < minimumLineLength )
				{
					filteredPointsArray.remove( filteredPointsArray.size() - 1 );
					firstPt.x = ( lastPt.x + firstPt.x ) / 2.0;
					firstPt.y = ( lastPt.y + firstPt.y ) / 2.0;
				}
			}
			
			return filteredPointsArray;
		}
		
		return initialPointsArray;
	}
	
	private static ArrayList<Point> pointFilter( ArrayList<Point> initialPointsArray )
	{
		ArrayList<Point> previousArray = initialPointsArray;
		ArrayList<Point> currentArray = initialPointsArray;
		
		do
		{
			previousArray = currentArray;
			currentArray = pointFilterPass( previousArray );
			
		} while ( ( previousArray.size() != currentArray.size() ) );
		
		return currentArray;
	}
	
	private static ArrayList<Point> pointFilterPass( ArrayList<Point> initialPointsArray )
	{
		final int START_IDX = 0;
		
		ArrayList<Point> linePoints = new ArrayList<>();
		ArrayDeque<Point> trace1Buffer = new ArrayDeque<>();
		ArrayDeque<Point> trace2Buffer = new ArrayDeque<>();
		int trace1Index = START_IDX;
		int trace2Index = START_IDX;
		
		if ( initialPointsArray.size() == 0 )
		{
			return initialPointsArray;
		}
		
		// Populate initial buffers
		trace1Buffer.addFirst( listWrappedGet( initialPointsArray, trace1Index - 1 ) );
		trace1Buffer.addFirst( listWrappedGet( initialPointsArray, trace1Index ) );
		trace1Buffer.addFirst( listWrappedGet( initialPointsArray, trace1Index + 1 ) );
		
		trace2Buffer.addFirst( listWrappedGet( initialPointsArray, trace1Index + 1 ) );
		trace2Buffer.addFirst( listWrappedGet( initialPointsArray, trace1Index ) );
		trace2Buffer.addFirst( listWrappedGet( initialPointsArray, trace1Index - 1 ) );
		
		boolean finished = false;
		
		do
		{
			int prevTrace1Index = trace1Index;
			int prevTrace2Index = trace2Index;
			
			trace1Index = updateTrace( trace1Index, initialPointsArray, linePoints, trace1Buffer, true, -1 );
			trace2Index = updateTrace( trace2Index, initialPointsArray, linePoints, trace2Buffer, false, prevTrace1Index );
			
			if ( trace1Index == trace2Index )
			{
				updateTrace( trace1Index, initialPointsArray, linePoints, trace1Buffer, true, -1 );
				finished = true;
			}
			else if ( prevTrace1Index != prevTrace2Index )
			{
				int prevIndexDifference = prevTrace2Index - prevTrace1Index;
				int currentIndexDifference = trace2Index - trace1Index;
				int prevIndexSign = Math.abs( prevIndexDifference ) / prevIndexDifference;
				int currentIndexSign = Math.abs( currentIndexDifference ) / currentIndexDifference;
				
				if ( prevIndexSign != currentIndexSign ) // Traces have overlapped, time to finish.
				{
					finished = true;
				}
			}
			
		} while ( !finished );
		
		if ( linePoints.size() >= 3 )
		{
			if ( Utility.getPointsDistance( linePoints.get( 0 ), linePoints.get( linePoints.size() - 1 ) ) <= 0.1 )
			{
				linePoints.remove( linePoints.size() - 1 );
			}
		}
		
		if ( linePoints.size() < 3 )
		{
			linePoints.clear();
		}
		
		return linePoints;
	}
	
	private static void getPointsFromTraceBuffer( ArrayDeque<Point> traceBuffer, Point pt1, Point pt2, Point pt3 )
	{
		Iterator<Point> ptIter = null;
		Point previousPoint = null;
		Point currentPoint = null;
		double x1 = 0.0;
		double y1 = 0.0;
		double x2 = 0.0;
		double y2 = 0.0;
		
		if ( ( traceBuffer != null ) && ( pt1 != null ) && ( pt2 != null ) && ( pt3 != null ) )
		{
			ptIter = traceBuffer.descendingIterator();
			
			while ( ptIter.hasNext() )
			{
				x1 = x2;
				y1 = y2;
				previousPoint = currentPoint;
				currentPoint = ptIter.next();
				
				if ( previousPoint != null )
				{
					x2 = currentPoint.x - previousPoint.x;
					y2 = currentPoint.y - previousPoint.y;
				}
			}
		}

		pt3.x = currentPoint.x;
		pt3.y = currentPoint.y;
		pt2.x = previousPoint.x;
		pt2.y = previousPoint.y;
		pt1.x = previousPoint.x - x1;
		pt1.y = previousPoint.y - y1;
	}
	
	private static int updateTrace( int currentIndex, ArrayList<Point> initialPoints, 
			ArrayList<Point> newPoints, ArrayDeque<Point> traceBuffer, boolean positiveDirection, int justProcessed )
	{
		final double MIN_TURN = 0.1;

		Point pt1 = new Point();
		Point pt2 = new Point();
		Point pt3 = new Point();
		
		int nextIncrement = ( positiveDirection ? 1 : -1 );		
		int newIndex = Utility.mod( currentIndex + nextIncrement, initialPoints.size() );
		
		if ( currentIndex != justProcessed )
		{
			getPointsFromTraceBuffer( traceBuffer, pt1, pt2, pt3 );
			
			if ( Utility.getTurn( pt1, pt2, pt3 ) > MIN_TURN )
			{
				Point extendedBackPoint = listWrappedGet( initialPoints, currentIndex + ( nextIncrement * -2 ) );
				Point extendedFrontPoint = listWrappedGet( initialPoints, currentIndex + ( nextIncrement * 2 ) );
				
				if ( ( Utility.getTurn( extendedBackPoint, pt2, pt3 ) > MIN_TURN ) 
					&& ( Utility.getTurn( pt1, pt2, extendedFrontPoint ) > MIN_TURN ) )
				{
					if ( positiveDirection )
					{
						newPoints.add( 0, pt2 );
					}
					else
					{
						newPoints.add( pt2 );
					}
				}
			}
		}
		
		traceBuffer.removeLast();
		traceBuffer.addFirst( listWrappedGet( initialPoints, newIndex + nextIncrement ) );
		
		return newIndex;
	}

}
