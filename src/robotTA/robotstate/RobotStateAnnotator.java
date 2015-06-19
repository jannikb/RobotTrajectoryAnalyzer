package robotTA.robotstate;

import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Iterator;

import org.apache.uima.UimaContext;
import org.apache.uima.analysis_engine.AnalysisEngineProcessException;
import org.apache.uima.cas.FSIndex;
import org.apache.uima.fit.component.JCasAnnotator_ImplBase;
import org.apache.uima.fit.descriptor.ConfigurationParameter;
import org.apache.uima.jcas.JCas;
import org.apache.uima.jcas.tcas.Annotation;
import org.apache.uima.resource.ResourceInitializationException;
import urdfMongo.Mongo2Urdf;
import urdfMongo.Urdf;
import urdfMongo.joint.Joint.JointType;
import urdfMongo.joint.Limit;

import robotTA.kinematics.FKSolver;
import robotTA.kinematics.Frame;
import robotTA.types.DistanceToLimit;
import robotTA.types.JointState;
import robotTA.types.TaskSpacePosition;
import robotTA.utils.EasyArray;

import com.mongodb.DB;
import com.mongodb.MongoClient;

public class RobotStateAnnotator extends JCasAnnotator_ImplBase {

    public static final String DBADDRESS = "dbAddress";
    @ConfigurationParameter(name = DBADDRESS)
    private String dbAddress;
    
    public static final String DBNAME = "dbName";
    @ConfigurationParameter(name = DBNAME)
    private String dbName;
    
    private Urdf urdf;
    private FKSolver fkSolver;
	
    @Override
    public void initialize(UimaContext aContext)
            throws ResourceInitializationException {
        super.initialize(aContext);

        try {
            MongoClient mongoClient = new MongoClient(dbAddress, 27017);
            DB db = mongoClient.getDB(dbName);
            urdf = new Mongo2Urdf(db);
            fkSolver = new FKSolver(urdf);
        } catch (UnknownHostException e) {
            e.printStackTrace();
        }       
    }
    
	@Override
	public void process(JCas aJCas) throws AnalysisEngineProcessException {
		FSIndex<Annotation> jsIndex = aJCas.getAnnotationIndex(JointState.type);
        Iterator<Annotation> jsIter = jsIndex.iterator();
        JointState js;
        TaskSpacePosition tsPos; 
        DistanceToLimit dtLimit; 
        String name;
        Limit limit;
        ArrayList<String> names;
        ArrayList<Double> upperLimits;
        ArrayList<Double> lowerLimits;
        ArrayList<Double> efforts;
        ArrayList<Double> velocities;

        while (jsIter.hasNext()) {
            js = (JointState) jsIter.next();
            
            // Annotate task space position
            tsPos = new TaskSpacePosition(aJCas);
            for (Frame frame: fkSolver.calculate(js)) {
            	tsPos.setLinkName(frame.getName());
            	tsPos.setPosition(EasyArray.createDoubleArray(aJCas, frame.getPosition()));
            	tsPos.setRotation(EasyArray.createDoubleArray(aJCas, frame.getRpy()));
            }
            
            tsPos.setBegin(js.getBegin());
            tsPos.setEnd(js.getEnd());
            tsPos.addToIndexes();
            
            // Annotate limits
            dtLimit = new DistanceToLimit(aJCas); 
            names = new ArrayList<String>();
            upperLimits = new ArrayList<Double>();
            lowerLimits = new ArrayList<Double>();
            efforts = new ArrayList<Double>();
            velocities = new ArrayList<Double>();
            for (int i = 0; i < js.getName().size(); i++) {
                name = js.getName(i);
                if (hasLimits(name)) {
                    names.add(name);
                    limit = urdf.getJoints().get(name).getLimits();
                    upperLimits.add(Math.abs(limit.getUpperLimit() - js.getJointTrajectoryPoint().getPositions(i)));
                    lowerLimits.add(Math.abs(limit.getLowerLimit() - js.getJointTrajectoryPoint().getPositions(i)));
                    efforts.add(limit.getEffort() - Math.abs(js.getJointTrajectoryPoint().getEffort(i)));
                    velocities.add(limit.getVelocity() - Math.abs(js.getJointTrajectoryPoint().getVelocities(i)));
                } 
            }
            
            dtLimit.setUpperLimit(EasyArray.createDoubleArray(aJCas, upperLimits));
            dtLimit.setLowerLimit(EasyArray.createDoubleArray(aJCas, lowerLimits));
            dtLimit.setEffort(EasyArray.createDoubleArray(aJCas, efforts));
            dtLimit.setVelocity(EasyArray.createDoubleArray(aJCas, velocities));
            dtLimit.setJointNames(EasyArray.createStringArray(aJCas, names));
            dtLimit.setBegin(js.getBegin());
            dtLimit.setEnd(js.getEnd());
            dtLimit.addToIndexes();
        }
	}
	
	private boolean hasLimits(String jointName) {
	    JointType type = urdf.getJoints().get(jointName).getJointType();
	    return (type == JointType.REVOLUTE) || (type == JointType.PRISMATIC);
	}

}
