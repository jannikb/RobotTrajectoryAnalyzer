package robotTA.kinematics;

import java.util.ArrayList;
import java.util.Arrays;

public class JointStateDummy {
	
	protected ArrayList<String> name;
	protected ArrayList<Double> position;
	
	public class DummyJointPoint {
	    
	    protected ArrayList<Double> position;
	    
	    public DummyJointPoint(ArrayList<Double> pos) {
	        position = pos;
	    }
	    
	    public double getPositions(int i) {
	        return position.get(i);
	    }
	}

	public ArrayList<String> getName() {
		return name;
	}

	public String getName(int i) {
		return name.get(i);
	}
	
	public void setName(String[] name) {
		this.name = new ArrayList<String>(Arrays.asList(name));
	}
	
	public DummyJointPoint getJointTrajectoryPoint() {
	    return new DummyJointPoint(position);
	}

	public void setPosition(Double[] position) {
		this.position =  new ArrayList<Double>(Arrays.asList(position));
	}
	
	

}
