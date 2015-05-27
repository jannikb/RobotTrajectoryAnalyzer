package robotTA.kinematics;

import java.io.IOException;
import java.util.Arrays;
import java.util.Collection;
import java.util.Hashtable;

import javax.xml.parsers.ParserConfigurationException;

import org.jannikbu.Urdf;
import org.jannikbu.Xml2Urdf;
import org.jannikbu.exceptions.InvalidUrdfNotation;
import org.jannikbu.joint.Joint;
import org.jannikbu.link.Link;
import org.xml.sax.SAXException;

import robotTA.kinematics.Frame;
import robotTA.types.JointState;
import Jama.Matrix;

public class FKSolver {

	private Hashtable<String, Frame> frames;
	private Hashtable<String, Double> jointPositions;
	private Frame rootFrame;
	private Urdf urdf;

	public FKSolver(Urdf urdf) {
		this.urdf = urdf;
		frames = new Hashtable<String, Frame>(urdf.getLinks().size());
		for (Link link : urdf.getLinks().values()) {
			frames.put(link.getName(), new Frame(link));
		}
		rootFrame = frames.get(urdf.getRootLink().getName());
		jointPositions = new Hashtable<String, Double>();
	}

	public Collection<Frame> calculate(JointState jointState) {
		for (int i = 0; i < jointState.getName().size(); i++) {
			jointPositions.put(urdf.getJoints().get(jointState.getName(i))
					.getChild(), jointState.getJointTrajectoryPoint().getPositions(i));
		}
		calculateTree(rootFrame.getStaticTransformation(), rootFrame);
		return frames.values();
	}

	private void calculateTree(Matrix transformation, Frame frame) {
		Matrix staticTrans = transformation.times(frame.getStaticTransformation());
		Matrix dynTrans;
		if (jointPositions.containsKey(frame.getName())) {
			dynTrans = staticTrans.times(frame.getJointPosition(jointPositions.get(frame.getName())));
		} else {
			dynTrans = staticTrans;
		}
		frame.setPosition(new double[] { dynTrans.get(0, 3),
				dynTrans.get(1, 3), dynTrans.get(2, 3) });
		
		// todo method to get rpy from rotation matrix
		for (Joint joint : urdf.getLinks().get(frame.getName())
				.getChildJoints()) {
			// System.out.println(joint.getChild());
			calculateTree(dynTrans, frames.get(joint.getChild()));
		}
	}

	

	public static void main(String[] args) {

//		class JSTestUrdf extends JointStateDummy {
//			public JSTestUrdf() {
//				setName(new String[] {"joint1", "joint2", "joint3"});
//				setPosition(new Double[] {Math.PI, Math.PI, 0.0});
//			}
//		}
//
//		try {
//			Urdf urdf = new Xml2Urdf("../UrdfMongo/urdf/test_urdf.urdf");
//			FKSolver fk = new FKSolver(urdf);
//			JointStateDummy js = new JSTestUrdf();
//			Collection<Frame> frames = fk.calculate(js);
//			System.out
//					.println("--------------------------------------------------");
//			for (Frame frame : frames) {
//				System.out.println(frame.getName()
//						+ Arrays.toString(frame.getPosition())
//						+ Arrays.deepToString(frame.getRotation().getArray()));
//				System.out.println(Arrays.deepToString(frame
//						.getStaticTransformation().getArray()));
//			}
//		} catch (SAXException | IOException | ParserConfigurationException
//				| InvalidUrdfNotation e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
	}
	
	
}
