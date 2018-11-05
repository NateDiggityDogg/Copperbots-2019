package com.main;

import java.awt.Component;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.awt.image.WritableRaster;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;

import org.opencv.*;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Point;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



public class Main {
	
		static int camPort = 0;
		
	public static void main(String[] args) {
		System.out.printf("java.library.path: %s%n", System.getProperty("java.library.path"));
		System.loadLibrary("opencv_java343");
		
		//Declare Image Mat & Client-side Camera
		Mat mat = new Mat();
		VideoCapture capture = new VideoCapture(camPort);
		capture.open(camPort);
		
		//Create JFrame for Testing
		JFrame frame = new JFrame();
		frame.setVisible(true);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		capture.read(mat);
		BufferedImage img = createAwtImage(mat);
		frame.setSize(img.getWidth(),img.getHeight());
		
		//Connect to NetworkTable
		NetworkTableInstance homeInstance = NetworkTableInstance.getDefault();
		homeInstance.startClientTeam(2586);
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		NetworkTable table = homeInstance.getTable("Vision");
		System.out.println(homeInstance.isConnected());
		
		NetworkTableEntry xEntry = table.getEntry("X");	
		
		
		while(true) {
			capture.read(mat);
			CubePipe pipe = new CubePipe();
			pipe.process(mat);
			MatOfKeyPoint blobs = pipe.findBlobsOutput();
			mat = pipe.blurOutput();
			KeyPoint[] keys = blobs.toArray();
			for(int i = 0; i<= keys.length - 1; i++) {
			//	System.out.println("Blob # " + i + ": " + keys[i].size);
				Point p = keys[i].pt;
				System.out.println("Point # " + i + ": (" + p.x + ", " + p.y + ")");
			}
			img = createAwtImage(mat);
			ImageIcon image = new ImageIcon(img);
			JLabel label = new JLabel(image);
			frame.getContentPane().removeAll();
			frame.getContentPane().add(label);
			frame.getContentPane().repaint();
			frame.pack();

			xEntry.forceSetDouble(32);


		//	System.out.println("tick!");
			
		}
		
		
//		JFrame frame = new JFrame("Window");
//		JLabel vidpanel = new JLabel();
//		frame.setContentPane(vidpanel);
//	    frame.setVisible(true);
                    
	}
	
	public static BufferedImage createAwtImage(Mat mat) {

	    int type = 0;
	    if (mat.channels() == 1) {
	        type = BufferedImage.TYPE_BYTE_GRAY;
	    } else if (mat.channels() == 3) {
	        type = BufferedImage.TYPE_3BYTE_BGR;
	    } else {
	        return null;
	    }

	    BufferedImage image = new BufferedImage(mat.width(), mat.height(), type);
	    WritableRaster raster = image.getRaster();
	    DataBufferByte dataBuffer = (DataBufferByte) raster.getDataBuffer();
	    byte[] data = dataBuffer.getData();
	    mat.get(0, 0, data);

	    return image;
	}

}
