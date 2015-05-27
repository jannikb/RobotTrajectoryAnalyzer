package robotTA.utils;

import java.util.ArrayList;

import org.apache.uima.jcas.JCas;
import org.apache.uima.jcas.cas.DoubleArray;
import org.apache.uima.jcas.cas.FloatArray;
import org.apache.uima.jcas.cas.IntegerArray;
import org.apache.uima.jcas.cas.StringArray;

public class EasyArray {
    
    public static StringArray createStringArray(JCas jcas, ArrayList<String> arrayList) {
        StringArray stringArray = new StringArray(jcas, arrayList.size());
        stringArray.copyFromArray((String[]) arrayList.toArray(new String[arrayList.size()]), 0, 0, arrayList.size());
        
        return stringArray;
    }
    
    public static IntegerArray createIntegerArray(JCas jcas, ArrayList<Integer> arrayList) {
        IntegerArray integerArray = new IntegerArray(jcas, arrayList.size());
        for(int i=0; i<arrayList.size(); i++) {
            integerArray.set(i, arrayList.get(i));
        }
        
        return integerArray;
    }
    
    public static FloatArray createFloatArray(JCas jcas, ArrayList<Float> arrayList) {
        FloatArray floatArray = new FloatArray(jcas, arrayList.size());
        for(int i=0; i<arrayList.size(); i++) {
            floatArray.set(i, arrayList.get(i));
        }
        
        return floatArray;
    }
    
    public static DoubleArray createDoubleArray(JCas jcas, ArrayList<Double> arrayList) {
        DoubleArray doubleArray = new DoubleArray(jcas, arrayList.size());
        for(int i=0; i<arrayList.size(); i++) {
            doubleArray.set(i, arrayList.get(i));
        }
        
        return doubleArray;
    }
    
    public static DoubleArray createDoubleArray(JCas jcas, double[] array) {
        DoubleArray doubleArray = new DoubleArray(jcas, array.length);
        for(int i=0; i<array.length; i++) {
            doubleArray.set(i, array[i]);
        }
        
        return doubleArray;
    }
    
    public static ArrayList<String> createArrayList(StringArray stringArray) {
        ArrayList<String> stringList = new ArrayList<String>(stringArray.size());
        
        for (int i = 0; i < stringArray.size(); i++) {
            stringList.add(stringArray.get(i));
        }
        
        return stringList;
    }
    
    public static int find(StringArray strings, String string) {
    	for (int i = 0; i <strings.size(); i++) {
    		if (string.equals(strings.get(i)))
    			return i;
    	}
    	return -1;
    }

}
