package first.frc.team2077.season2017.vision.trackers;

import org.opencv.core.Mat;

public class Snapshot 
{
	private int xCoord;
	private int yCoord;
	
	private int startEdgeIndex;
	private int endEdgeIndex;
	private int nextStartEdgeIndex;
	
	private boolean clockwise;

	private double turnAngle; //[+]right turn, [-]left turn, in degrees.
	
	private boolean[][] shotData = new boolean[3][3]; // Indexed as [x][y]
	
	public static Snapshot takeSnapshot( Mat hlsMask, int xCoord, int yCoord )
	{
		return takeSnapshot( hlsMask, null, xCoord, yCoord );
	}
	
	public static Snapshot takeSnapshot( Mat hlsMask, Snapshot previousSnapshot, int xCoord, int yCoord )
	{
		Snapshot snapshot = new Snapshot();
		boolean foundEndEdgeIndex = false;

		snapshot.xCoord = xCoord;
		snapshot.yCoord = yCoord;
		
		// Make sure the center sample is a hit, otherwise this is invalid.
		if ( Utility.average( hlsMask.get( yCoord, xCoord ) ) > 0.01 )
		{
			snapshot.shotData[1][1] = true;
		}
		else
		{
			return null;
		}
		
		// Sample the hlsMask to populate the shotData array.
		for ( int i = 0; i < 3; ++i )
		{
			for ( int j = 0; j < 3; ++j )
			{
				int currentX = xCoord-1+i;
				int currentY = yCoord-1+j;
				
				if ( ( i != 1) || ( j != 1 ) )
				{
					if ( ( currentY < hlsMask.height() ) && ( currentX < hlsMask.width() )
							&& ( currentY >= 0 ) && ( currentX >= 0 ) )
					{
						snapshot.shotData[i][j] = ( Utility.average( hlsMask.get( currentY, currentX ) ) > 0.01 );
					}
					else
					{
						snapshot.shotData[i][j] = false;
					}
				}
			}
		}
		
		//snapshot.printSnapshotData();
		
		if ( previousSnapshot == null )
		{
			// Calculate initial starting edge index.
			boolean foundStartEdgeIndex = false;
			
			for ( int i = 1; ( ( i <= 8 ) && !foundStartEdgeIndex ); ++i )
			{
				if ( snapshot.shotDataGetReMapped( i ) != snapshot.shotDataGetReMapped( i + 1 ) )
				{
					snapshot.startEdgeIndex = i;
					snapshot.clockwise = snapshot.shotDataGetReMapped( i );
					foundStartEdgeIndex = true;
				}
			}
			
			if ( !foundStartEdgeIndex )
			{
				return null; // No valid path can be generated.
			}
		}
		else
		{
			// Get starting edge index from previous snapshot.
			snapshot.startEdgeIndex = previousSnapshot.nextStartEdgeIndex;
			snapshot.clockwise = snapshot.shotDataGetReMapped( snapshot.startEdgeIndex );
		}
		
		// Find the end edge index.
		for ( int i = 1; ( ( i < 8 ) && !foundEndEdgeIndex ); ++i )
		{
			int currentIndex = snapshot.startEdgeIndex + ( i * ( snapshot.clockwise ? 1 : -1 ) );
			
			if ( snapshot.shotDataGetReMapped( currentIndex )
					!= snapshot.shotDataGetReMapped( currentIndex + 1 ) )
			{
				snapshot.endEdgeIndex = wrap1To8Index( currentIndex );
				foundEndEdgeIndex = true;
			}
		}
		
		if ( !foundEndEdgeIndex )
		{
			return null; // No valid path can be generated.
		}
		
		// Calculate the starting edge index of the next waypoint snapshot.
		snapshot.nextStartEdgeIndex = wrap1To8Index( snapshot.endEdgeIndex + ( snapshot.clockwise ? 5 : 3 ) );
		
		// Calculate turn angle.
		snapshot.turnAngle = getTurnAngleFromEdgeStartIndexDifference( snapshot.nextStartEdgeIndex - snapshot.startEdgeIndex );
		
		//System.out.println( snapshot.turnAngle + " degrees" );
		
		return snapshot;
	}
	
	public static boolean isConjugate( Snapshot s1, Snapshot s2 )
	{
		return ( Math.abs( Utility.mod( s1.nextStartEdgeIndex, 8 ) - Utility.mod( s1.startEdgeIndex, 8 ) ) == 1 )
				&& ( s1.startEdgeIndex == s2.nextStartEdgeIndex );
	}
	
	private static int wrap1To8Index( int idx )
	{
		return Utility.mod( idx - 1, 8 ) + 1;
	}
	
	private static double getTurnAngleFromEdgeStartIndexDifference( int difference )
	{
		switch ( Utility.mod( difference, 8 ) )
		{
		case 0: return 0.0;
		case 1: return 45.0;
		case 2: return 90.0;
		case 3: return 135.0;
		case 4: return 180.0;
		case 5: return -135.0;
		case 6: return -90.0;
		case 7: return -45.0;
		}
		
		return 0.0; // Unreachable
	}
	
	private static int getUnwrappedXCoordinate( int idx )
	{
		/* Indices for snapshotBits:
		 * [ 1 | 2 | 3 ]
		 * [ 8 | X | 4 ]
		 * [ 7 | 6 | 5 ]
		 * 
		 * If each bit is indexed as: [MSB]8-7-6-5-4-3-2-1[LSB]
		 */
		switch ( wrap1To8Index( idx ) )
		{
		case 1: return 0;
		case 2: return 1;
		case 3: return 2;
		case 4: return 2;
		case 5: return 2;
		case 6: return 1;
		case 7: return 0;
		case 8: return 0;
		}
		
		return -1; // Unreachable
	}
	
	private static int getUnwrappedYCoordinate( int idx )
	{
		/* Indices for snapshotBits:
		 * [ 1 | 2 | 3 ]
		 * [ 8 | X | 4 ]
		 * [ 7 | 6 | 5 ]
		 * 
		 * If each bit is indexed as: [MSB]8-7-6-5-4-3-2-1[LSB]
		 */
		switch ( wrap1To8Index( idx ) )
		{
		case 1: return 0;
		case 2: return 0;
		case 3: return 0;
		case 4: return 1;
		case 5: return 2;
		case 6: return 2;
		case 7: return 2;
		case 8: return 1;
		}
		
		return -1; // Unreachable
	}
	
	/**
	 * @return the x coordinate
	 */
	public int getXCoord() {
		return xCoord;
	}
	
	/**
	 * @return the y coordinate
	 */
	public int getYCoord() {
		return yCoord;
	}
	
	/**
	 * @return the next x coordinate
	 */
	public int getNextXCoord() {
		return getUnwrappedXCoordinate( wrap1To8Index( endEdgeIndex + ( clockwise ? 1 : 0 ) ) ) - 1 + xCoord;
	}
	
	/**
	 * @return the next y coordinate
	 */
	public int getNextYCoord() {
		return getUnwrappedYCoordinate( wrap1To8Index( endEdgeIndex + ( clockwise ? 1 : 0 ) ) ) - 1 + yCoord;
	}
	
	public boolean getShotData( int x, int y )
	{
		if ( ( x < 3 ) && ( y < 3 ) )
		{
			return shotData[x][y];
		}
		
		return false;
	}
	
	/**
	 * @return the turnAngle
	 */
	public double getTurnAngle() {
		return turnAngle;
	}

	/**
	 * @return the clockwise
	 */
	public boolean isClockwise() {
		return clockwise;
	}
	
	private boolean shotDataGetReMapped( int idx )
	{		
		/* Indices for snapshotBits:
		 * [ 1 | 2 | 3 ]
		 * [ 8 | X | 4 ]
		 * [ 7 | 6 | 5 ]
		 * 
		 * If each bit is indexed as: [MSB]8-7-6-5-4-3-2-1[LSB]
		 */
		switch ( wrap1To8Index( idx ) )
		{
		case 1: return shotData[0][0];
		case 2: return shotData[1][0];
		case 3: return shotData[2][0];
		case 4: return shotData[2][1];
		case 5: return shotData[2][2];
		case 6: return shotData[1][2];
		case 7: return shotData[0][2];
		case 8: return shotData[0][1];
		}
		
		return false; // Unreachable
	}
	
	private void printSnapshotData()
	{
		System.out.println("----");
		System.out.print( shotData[0][0] ? "X" : "O" );
		System.out.print( shotData[1][0] ? "X" : "O" );
		System.out.print( shotData[2][0] ? "X" : "O" );
		System.out.println();
		System.out.print( shotData[0][1] ? "X" : "O" );
		System.out.print( shotData[1][1] ? "X" : "O" );
		System.out.print( shotData[2][1] ? "X" : "O" );
		System.out.println();
		System.out.print( shotData[0][2] ? "X" : "O" );
		System.out.print( shotData[1][2] ? "X" : "O" );
		System.out.print( shotData[2][2] ? "X" : "O" );
		System.out.println();
		System.out.println("----");
	}
}
