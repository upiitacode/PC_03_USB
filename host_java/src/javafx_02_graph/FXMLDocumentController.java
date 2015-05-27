/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package javafx_02_graph;

import java.net.URL;
import java.nio.IntBuffer;
import java.util.ResourceBundle;
import javafx.application.Platform;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.XYChart;

/**
 *
 * @author Manuel
 */
public class FXMLDocumentController implements Initializable {

    @FXML
    LineChart<Double, Double> myLineChart;

    @FXML
    NumberAxis myAxis_x;

    @FXML
    NumberAxis myAxis_y;
    
    private static final int DLY_TIME_MS = 100; 
    private XYChart.Series series;
    private double x_index = -100;
    private volatile boolean run_Thread = false;
    private volatile boolean done_Thread = false;
    private Tiva_USB tiva;
    private IntBuffer buffer;
    @FXML
    private void start_button_handler(ActionEvent event) {
        if(run_Thread){
            //Display thread already running warning
        }else{
            tiva.connect();
            run_Thread=true;
            startTask(); 
        }
    }

    @FXML
    private void stop_button_handler(ActionEvent event) {
        if(true == run_Thread){
            run_Thread=false;
            while(false == done_Thread);
            tiva.disconnect();
        }
        
    }

    @Override
    public void initialize(URL url, ResourceBundle rb) {
        // TODO 
        myLineChart.setTitle("TivaScope");
        ObservableList<XYChart.Series<Double, Double>> lineChartData = FXCollections.observableArrayList();
        series = new XYChart.Series();
        series.setName("CH1");
        //populating the series with data
        for (int i = 0; i < 100; i++) {
            series.getData().add(new XYChart.Data(x_index*(DLY_TIME_MS/1000.0), 0));
            x_index += 1;
        }
        myAxis_x.setLowerBound((x_index-100)*(DLY_TIME_MS/1000.0));
        myAxis_x.setUpperBound(x_index*(DLY_TIME_MS/1000.0));
        lineChartData.add(series);
        myLineChart.setData(lineChartData);
        tiva = new Tiva_USB();
    }

    public void startTask() {
        // Create a Runnable
        Runnable task = () -> runTask();
        // Run the task in a background thread
        Thread backgroundThread = new Thread(task);
        // Terminate the running thread if the application exits
        backgroundThread.setDaemon(true);
        // Start the thread
        backgroundThread.start();
    }

    public void runTask() {
        done_Thread=false;
        while(run_Thread) {
            try {
                // Update the Label on the JavaFx Application Thread
                Platform.runLater(() -> updatePB());
                Thread.sleep(DLY_TIME_MS);
            } catch (InterruptedException e) {
                System.out.println(e.toString());
            }
        }
        done_Thread=true;
    }

    public void updatePB() {
        if(true == run_Thread){
            double y = tiva.read()*(3.3/4095);
            series.getData().remove(0);
            series.getData().add(new XYChart.Data(x_index*(DLY_TIME_MS/1000.0), y));
            x_index += 1;
            myAxis_x.setLowerBound((x_index-100)*(DLY_TIME_MS/1000.0));
            myAxis_x.setUpperBound(x_index*(DLY_TIME_MS/1000.0));   
        }
    }
}
