package org.mysimulationmodel;
import org.mysimulationmodel.simulation.agent.*;
import org.mysimulationmodel.simulation.common.*;
import org.mysimulationmodel.simulation.environment.CEnvironment;

import javax.swing.*;
import java.io.File;
import java.io.FileInputStream;
import java.util.*;
import java.util.logging.LogManager;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import java.util.stream.Stream;

/**
 * Multi-agent Simulation
 */
final class CMain
{

    private static final String FILE_HEADER = "scenarioid,time,id,x_axis,y_axis,velocity";
    static Map<String, ArrayList<String>> m_realdata = CCSVFileReaderForGA.readDataFromCSV2();
    static Map<Integer,Map<String,Double>> m_strategy = new HashMap<>();


    //private String m_name;
    static
    {
        LogManager.getLogManager().reset();
    }

    private CMain()
    {
    }

    protected static ArrayList<COutputFormat> runSimulation(double p_distance, double p_collisionchekingfactor, int p_randomization, int p_senarioid )
    {

        //m_realdata.forEach((k,m)-> System.out.println(k+" "+m));
        int m_pixelpermeter = CEnvironment.getpixelpermeter();
        CHostAgent l_hostagent;

        File l_ped = new File(System.getProperty("user.dir").concat("/agent.asl"));
        File l_car = new File(System.getProperty("user.dir").concat("/car.asl"));
        File l_host = new File(System.getProperty("user.dir").concat("/host.asl"));

        CEnvironment l_env = new CEnvironment();

        JFrame l_frame = new JFrame("Multiagent-based Simulation");
        l_frame.add( l_env );

        l_frame.setSize( l_env.getWidth(), l_env.getHeight() );
        l_frame.setVisible( true );
        l_frame.setDefaultCloseOperation( WindowConstants.EXIT_ON_CLOSE );
        l_env.setScenario(p_senarioid);


        List<CInputFormat> l_roadusers = CCsvFileReader.readDataFromCSV(System.getProperty("user.dir").concat("/start_end_citr_extended.csv"), p_senarioid);
        List<CInputFormat> l_pedestrians = l_roadusers.stream().filter( n -> n.m_roaduser_id.startsWith("p") ).collect(Collectors.toList());
        List<CInputFormat> l_cars = l_roadusers.stream().filter( n -> n.m_roaduser_id.startsWith("t") ).collect(Collectors.toList());
        int count = 0;
        int maxTries = 3;
        ArrayList<COutputFormat> m_output = new ArrayList<>();
        while(true) {
            try
                    (    final FileInputStream l_pedestrianstream = new FileInputStream( l_ped );
                         final FileInputStream l_carstream = new FileInputStream( l_car );
                         final FileInputStream l_hoststream = new FileInputStream( l_host )
                    )
            {
                Stream.concat(
                        new CPedestrianGenerator( l_pedestrianstream, l_env, l_pedestrians )
                                .generatemultiple(  l_pedestrians.size()),//l_pedestrians.size()

                        new CCarGenerator( l_carstream, l_env, l_cars )
                                .generatemultiple( l_cars.size() )

                ).collect( Collectors.toSet() );

                l_hostagent = new CHostGenarator( l_hoststream, l_env )
                        .generatesingle( 1 );
                break;

            }

            catch ( final Exception l_exception )
            {
                if (++count > maxTries)
                    l_exception.printStackTrace();
                else throw new RuntimeException();
            }
        }

        int l_start = l_roadusers.stream().map( i -> i.m_start_cycle ).sorted().collect(Collectors.toList()).get(0);
        int l_end = l_roadusers.stream().map( i -> i.m_numberofcycle ).sorted(Comparator.reverseOrder()).collect(Collectors.toList()).get(0);
        final float[] l_timestep = {l_start/2f};//0.5
        CHostAgent finalL_hostagent = l_hostagent;

        IntStream.range(l_start, l_end+1)//l_end-9 l_start, l_end -8
                .forEach( j ->
                {
                    if ( l_env.addPedtoInitializeLater().get(j) != null )
                    {
                        l_env.addPedtoInitializeLater().get(j).forEach( n ->
                        {
                            l_env.initialset( n );
                            l_env.initialPedestrian( n );
                        });
                    }

                    if ( l_env.addCartoInitializeLater().get(j) != null )
                    {
                        l_env.addCartoInitializeLater().get(j).forEach( n ->
                        {
                            l_env.initialset( n );
                            l_env.initialCar( n );
                        });
                    }

                    //long startTime = System.nanoTime();
                    try {
                        finalL_hostagent.updateParameter( p_distance*m_pixelpermeter, p_collisionchekingfactor, p_randomization );
                        finalL_hostagent.call();
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                    l_env.repaint();
                    l_env.getRoadUserinfo()
                            .forEach( i ->
                            {
                                try
                                {   i.call();
                                    //if(i.getType() == 2)System.out.println("dis "+i.m_distance);
                                    ArrayList<String> l_temp = m_realdata.get( new StringBuffer(new StringBuffer(String.valueOf(l_timestep[0]))
                                            .append(i.getname())).append(p_senarioid).toString());
                                    if(l_temp != null){
                                    double l_speed =  i.getVelocity().length() == 0.2 ? 0.0 : i.getVelocity().length();
                                    m_output.add( new COutputFormat( p_senarioid,l_timestep[0], i.getname(),i.getPosition().x, i.getPosition().y, l_speed*2 ) );}
                                    Thread.sleep(5);
                                }

                                catch ( final Exception l_exception )
                                {
                                    l_exception.printStackTrace();
                                    //throw new RuntimeException();
                                }

                            } );
                    l_timestep[0] = (float) (l_timestep[0] + 0.5);
                    l_env.setCurrentCycle(l_timestep[0]);
                } );

        m_strategy.putIfAbsent(p_senarioid, l_env.getStrategy());
        return m_output;
    }

    public static void main( final String[] p_args )
    {
        //to write simulation output
        ArrayList<ArrayList<COutputFormat>> m_output = new ArrayList<>();

        //to write simulation output
        final String l_name = System.getProperty("user.dir").concat("/CITRAllScenario.csv");


        long startTime = System.nanoTime();
        for(int i =1; i<27; i++)//3.4
        {
            m_output.add(
                    CMain.runSimulation(12.3, 9,
                            2, i));
        }
        long endTime = System.nanoTime();

        long durationInNano = (endTime - startTime);
        System.out.println("time "+durationInNano);

        ArrayList<COutputFormat> bla = new ArrayList<>();
        for( ArrayList<COutputFormat> out: m_output )
        {
            for (COutputFormat outt : out)
            {
                bla.add(outt);
            }
        }

        /*Map<Integer, Map<String, Double>> m_strategy2 = CCSVFileReaderForGA.readDataFromCSV3();
        final int[] counter = {1};
        m_strategy.forEach( (l,m)->
                {
                    if(!m_strategy2.get(l).isEmpty() && !m.isEmpty())
                        m.forEach((x,y)->
                                System.out.println(x+ " name "+y.equals(m_strategy2.get(l).get(x))+ " s "+ y+" r "+m_strategy2.get(l).get(x)));
                    System.out.println("next "+ counter[0]);
                    counter[0]++;
                }
        );*/

        CCsvReadWrite.writeCsvFile( l_name, bla, FILE_HEADER);

    }

}