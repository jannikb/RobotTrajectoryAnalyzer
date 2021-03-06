package robotTA.gui;

import org.jfree.chart.plot.*;
import org.jfree.chart.axis.*;
import org.jfree.chart.renderer.xy.*;
import org.jfree.chart.*;
import org.jfree.ui.ApplicationFrame;
import org.jfree.data.xy.*;
/**
 *
 * @author rasch
 */
public class Demo_JFreeChart {

    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {

        //2 Moeglichkeiten ein Dataset zu erzeugen:

        //Moeglichkeit 1 mit DefaultXYDataset:
        double [][] A = {{1,2,5},{3,4,0}};

        DefaultXYDataset dataset = new DefaultXYDataset();
        dataset.addSeries("xy", A);
        

        // series2 enthaelt Punkte, die verbunden werden
        XYSeries series1 = new XYSeries("Punkte1");
        series1.add(0, 0);
        series1.add(1, 1);
        series1.add(2, 1);
        series1.add(3, 2);

        XYSeries series2 = new XYSeries("Punkte2");
        series2.add(1, 2);
        series2.add(2, 3);
        series2.add(3, 4);

        // Hinzufuegen von series1 und series2 zu der Datenmenge dataset
        XYSeriesCollection dataset2 = new XYSeriesCollection();
        dataset2.addSeries(series1);
        dataset2.addSeries(series2);

//2.Versuch mit XYPlot

//Achsenbezeichungen
            
        NumberAxis xax = new NumberAxis("x");
        NumberAxis yax = new NumberAxis("y");

//-------------------
        XYDotRenderer dot = new XYDotRenderer();
        // Groesse der Punkte
        dot.setDotHeight(5);
        dot.setDotWidth(5);
        XYPlot plot = new XYPlot(dataset2,xax,yax, dot);

        //"Punkte" entspricht der Ueberschrift des Fensters
        ApplicationFrame punkteframe = new ApplicationFrame("Punkte"); 

        JFreeChart chart2 = new JFreeChart(plot);


        ChartPanel chartPanel2 = new ChartPanel(chart2);
        punkteframe.setContentPane(chartPanel2);
        punkteframe.pack();
        punkteframe.setVisible(true);

//-----------------------------------------------------

        ApplicationFrame frame2 = new ApplicationFrame("Punkte mit Splines verbunden");
        XYSplineRenderer spline = new XYSplineRenderer();
        //System.out.println("spline precision = "+(spline.getPrecision()));
        spline.setPrecision(10);
        XYPlot plot2 = new XYPlot(dataset2,xax,yax, spline);


        JFreeChart chart3 = new JFreeChart(plot2);


        ChartPanel chartPanel3 = new ChartPanel(chart3);
        frame2.setContentPane(chartPanel3);
        frame2.pack();
        frame2.setVisible(true);


//-----------------------------------------------------

        ApplicationFrame frame4 = new ApplicationFrame("Punkte mit Linien verbunden (XYPlot)");
        XYLineAndShapeRenderer line = new XYLineAndShapeRenderer();
        XYPlot plot4 = new XYPlot(dataset2,xax,yax, line);


        JFreeChart chart4 = new JFreeChart(plot4);


        ChartPanel chartPanel4 = new ChartPanel(chart4);
        frame4.setContentPane(chartPanel4);
        frame4.pack();
        frame4.setVisible(true);


    }

}