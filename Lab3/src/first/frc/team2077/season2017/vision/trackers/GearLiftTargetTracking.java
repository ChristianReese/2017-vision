package first.frc.team2077.season2017.vision.trackers;

import java.util.ArrayList;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class GearLiftTargetTracking 
{
	public static Mat processMat( Mat toProcess, 
			double cameraWidth, double cameraHeight, 
			double min1, double max1, 
			double min2, double max2, 
			double min3, double max3, 
			boolean returnThresholdMat )
	{
		long prevTickCount = Core.getTickCount();
		double elapsedTime = 0.0;
		
		Mat thresholdMat = new Mat();
		
		Core.inRange(toProcess, new Scalar(min1, min2, min3), new Scalar(max1, max2, max3), thresholdMat);
		
		ArrayList<Polygon> polygons = trackPolygons( toProcess, thresholdMat );	
		
		ArrayList< CollinearLine > collinearLines = findCollinearLines( polygons );					
		TargetCandidate targetCandidate = getTargetCandidate( collinearLines, cameraWidth, cameraHeight );
		
		if ( targetCandidate != null )
		{
			targetCandidate.draw( toProcess );
		}

		elapsedTime = (double)( Core.getTickCount() - prevTickCount ) * 1000.0 / Core.getTickFrequency();
		System.out.print("time: ");
		System.out.printf("%.5f", elapsedTime);
		System.out.println(" ms");
		
		return ( returnThresholdMat ? thresholdMat : toProcess );
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
		for ( LineSegment ls1 : polygon1 )
		{
			for ( LineSegment ls2 : polygon2 )
			{
				if ( ls1.isCollinearWith( ls2 ) )
				{
					result.add( CollinearLine.createCollinearLine( ls1, ls2, polygon1, polygon2 ) );
				}
			}
		}
	}
	
	private static ArrayList<Polygon> trackPolygons(Mat camFrame, Mat hlsMask)
	{
		final double MIN_ARC_LENGTH = 100.0;
		
		ArrayList<Polygon> polygons = new ArrayList<>();
		ArrayList<MatOfPoint> contours = new ArrayList<>();
		Mat hierarchy = new Mat();
		
		Imgproc.findContours(hlsMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
		
		for ( MatOfPoint contour : contours )
		{
			MatOfPoint2f contour2f = new MatOfPoint2f( contour.toArray() );		
			MatOfPoint2f approxCurve = new MatOfPoint2f();
			double arcLength = Imgproc.arcLength(contour2f, true);
			
			if ( arcLength >= MIN_ARC_LENGTH )
			{
				Imgproc.approxPolyDP(contour2f, approxCurve, 0.04 * arcLength, true);
				
				polygons.add( Polygon.createPolygon( approxCurve.toList() ) );
			}
		}
		
		return polygons;
	}

}
