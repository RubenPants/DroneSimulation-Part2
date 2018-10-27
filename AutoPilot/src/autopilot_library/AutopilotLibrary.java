package autopilot_library;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

import java.nio.file.Files;
import java.nio.file.LinkOption;
import java.nio.file.StandardCopyOption;

/**
 * A class of cube detection algorithm JNI interfaces.
 *  Used by cube detection algorithms when wanting to detect cubes using C instead of JAVA.
 */
public class AutopilotLibrary {

	/**
	 * Load the C library.
	 */
	static {
	    try {
	    	  	System.loadLibrary("autopilot");
	    } catch (UnsatisfiedLinkError e) {
	    	  	loadLibFromJar("autopilot");
	    }
	}
	
	/**
	 * Load the C library from JAR file.
	 * 
	 * @param name The name of the C library that is to be loaded.
	 */
	private static void loadLibFromJar(String name) {
		name = System.mapLibraryName(name);
	    try {
	    	
	    		System.out.println(name);
	    		InputStream in = AutopilotLibrary.class.getResourceAsStream(name);
	    		File fileOut = new File(System.getProperty("java.io.tmpdir") + File.separator + name);
	    		int idx = 1;
	    		while (Files.exists(fileOut.toPath(), LinkOption.NOFOLLOW_LINKS))
	    			fileOut = new File(System.getProperty("java.io.tmpdir") + File.separator + name + "_" + idx++); 
	    		Files.copy(in, fileOut.toPath(), StandardCopyOption.REPLACE_EXISTING);
	        in.close();
	        System.load(fileOut.toString());
	    		
	    } catch (Exception e) {
	        System.out.println("An error occured while reading the library : " + e.getLocalizedMessage());
	    }
	}
	
	/**
	 * Copy the given source file to the given destination file.
	 * 
	 * @param 	source
	 * 			The source file to copy.
	 * @param 	dest
	 * 			The destination file to copy to.
	 * @throws 	IOException
	 * 			If an I/O error occurred.
	 */
	private static void copyFileUsingFileStreams(File source, File dest) throws IOException {
		InputStream input = null;
		OutputStream output = null;
		try {
			input = new FileInputStream(source);
			output = new FileOutputStream(dest);
			byte[] buf = new byte[1024];
			int bytesRead;
			while ((bytesRead = input.read(buf)) > 0) {
				output.write(buf, 0, bytesRead);
			}
		} catch (Exception e) {
			System.out.println("Error copying : " + e.getLocalizedMessage());
		} finally {
			input.close();
			output.close();
		}
	}
	
	/**
	 * Set up JNI.
	 */
	public static native void setup();
	
	/**
	 * Clean up JNI when it is no longer needed.
	 */
	public static native void cleanup();
	
	// --- Library functions (used to generate header) ---
	
	/**
	 * Locate all unit cubes in the given image with given parameters.
	 * 	If history is used, then cubes whose edges were previously seen are ignored.
	 * 
	 * @param 	pixels
	 * 			The image's pixels.
	 * @param 	width
	 * 			The image width.
	 * @param 	height
	 * 			The image height.
	 * @param 	useCache
	 * 			Flag denoting whether or not previously seen cubes are ignored if enough information
	 * 			about them was gathered.
	 * @param 	hasGround
	 * 			Flag denoting whether or not the given image contains a ground texture.
	 */
	public static native int[][] locateUnitCubes(byte[] pixels, int width, int height, boolean useCache, boolean hasGround);
	
}