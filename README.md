# RobotTrajectoryAnalyzer
Import as existing maven project and add UIMA nature.

Depends on 
-https://github.com/jannikb/UrdfMongo

Add external JARs for uima/uimafit (https://uima.apache.org/downloads.cgi), java-mongo-driver (http://mongodb.github.io/mongo-java-driver/), jfreechart (http://www.jfree.org/jfreechart/download.html) and jama (http://math.nist.gov/javanumerics/jama/).

At the moment I still get this error: ```JCas type "robotTA.types.JointState" used in Java code,  but was not declared in the XML type descriptor.```
