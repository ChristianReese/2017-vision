package it.polito.teaching.cv;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.concurrent.ScheduledExecutorService;

import org.opencv.core.Core;
import org.opencv.videoio.VideoCapture;

/**
 * The main class for a JavaFX application. It creates and handle the main
 * window with its resources (style, graphics, etc.).
 * 
 * This application handles a video stream and can convert its frames in gray
 * scale or color. Moreover, for each frame, it shows the corresponding
 * histogram and it is possible to add a logo in a corner of the video.
 * 
 * @author <a href="mailto:luigi.derussis@polito.it">Luigi De Russis</a>
 * @version 1.1 (2015-10-20)
 * @since 1.0 (2013-11-20)
 * 
 */
public class Video extends Application
{
	private static Video instance;

	public static class Config implements Serializable
	{
		private static final long serialVersionUID = 1946502478445282176L;
		
		public Config()
		{
			minHueStartVal = new Double(0.0);
			maxHueStartVal = new Double(180.0);
			minSatStartVal = new Double(0.0);
			maxSatStartVal = new Double(255.0);
			minLumStartVal = new Double(0.0);
			maxLumStartVal = new Double(255.0);
		}
		
		public Double minHueStartVal;
		public Double maxHueStartVal;
		public Double minSatStartVal;
		public Double maxSatStartVal;
		public Double minLumStartVal;
		public Double maxLumStartVal;
	}
	
	private Config configProxy;
	private ScheduledExecutorService guiTimer;
	private ScheduledExecutorService frameGrabTimer;
	private VideoCapture capture;

	public Video() 
	{
		instance = this;
		configProxy = new Config();
		guiTimer = null;
		frameGrabTimer = null;
		capture = null;
	}
	
	public static Video getInstance() 
	{
        return instance;
	}
	
	public Config getConfig()
	{
		return configProxy;
	}
	
	public void setGUIUpdateTimer(ScheduledExecutorService guiTimer)
	{
		this.guiTimer = guiTimer;
	}
	
	public void setFrameGrabTimer(ScheduledExecutorService frameGrabTimer)
	{
		this.frameGrabTimer = frameGrabTimer;
	}
	
	public void setVideoCapture(VideoCapture capture)
	{
		this.capture = capture;
	}
	
	@Override
	public void start(Stage primaryStage)
	{
		try
		{
			// load/initialize config data
			File configFile = new File("config.dat");
			
			if ( ( configFile.createNewFile() == false ) 
					&& ( new FileInputStream("config.dat").available() > 0 ) ) // If it already exists
			{
				FileInputStream fileIn = new FileInputStream("config.dat");
				ObjectInputStream objIn = new ObjectInputStream( fileIn );
				
				configProxy = (Config)objIn.readObject();
				
				if ( configProxy == null )
				{
					objIn.close();
					throw new NullPointerException();
				}
				
				objIn.close();
			}
			
			// load the FXML resource
			FXMLLoader loader = new FXMLLoader(getClass().getResource("Video.fxml"));
			// store the root element so that the controllers can use it
			BorderPane rootElement = (BorderPane) loader.load();
			// create and style a scene
			Scene scene = new Scene(rootElement, 800, 600);
			scene.getStylesheets().add(getClass().getResource("application.css").toExternalForm());
			// create the stage with the given title and the previously created
			// scene
			primaryStage.setTitle("Video processing");
			primaryStage.setScene(scene);
			// show the GUI
			primaryStage.show();
			
		}
		catch (Exception e)
		{
			e.printStackTrace();
		}
	}
	
	@Override
	public void stop()
	{
	    System.out.println("Stage is closing, saving configuration.");
	    
	    // Save file
	    FileOutputStream fileOut;
		try {
			fileOut = new FileOutputStream("config.dat");
			ObjectOutputStream objOut = new ObjectOutputStream( fileOut );
			
			objOut.writeObject(configProxy);
			
			objOut.close();
			
			System.out.println("Configuration saving successful!");
		} catch (IOException e) {
			System.err.println( "Error saving configuration file: " + e );
		}

	    if ( guiTimer != null )
	    {
	    	guiTimer.shutdown();
	    }
	    
	    if ( capture != null )
	    {
	    	capture.release();
	    }

	    if ( frameGrabTimer != null )
	    {
	    	frameGrabTimer.shutdown();
	    }
	}
	
	public static void main(String[] args)
	{
		// load the native OpenCV library
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		
		launch(args);
	}
}
