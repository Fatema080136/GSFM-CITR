package org.mysimulationmodel.simulation.agent;

import org.lightjason.agentspeak.common.CCommon;
import org.lightjason.agentspeak.generator.IBaseAgentGenerator;
import org.mysimulationmodel.simulation.common.CInputFormat;
import org.mysimulationmodel.simulation.constant.CVariableBuilder;
import org.mysimulationmodel.simulation.environment.CEnvironment;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import javax.vecmath.Vector2d;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicLong;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * Created by fatema on 29.01.2018.
 * @todo make one extra force for car which is from the edges of the road: can be either attractive or repulsive:
 * if car goes outside of the edges forces will be the attractive force, if they are inside of the edges then it will get repulsive force
 * from edges so it can avoid touching edges --- the need of this force depends on the topology
 */
public class CCarGenerator extends IBaseAgentGenerator<IBaseRoadUser>
{
    /**
     * for fixed start and goal position
     */
    private CopyOnWriteArrayList<Vector2d> m_positions = new CopyOnWriteArrayList<>();
    private CopyOnWriteArrayList<Vector2d> m_goalpositions = new CopyOnWriteArrayList<>();
    private static final int m_pixelpermeter = CEnvironment.getpixelpermeter();
    /* //Gaussian distribution of speed
    private Random rand = new Random();
    private static final double m_GaussianMeanSpeed =  2.22;//*m_pixelpermeter;// 2.25:meter per half second
    private static final double m_GaussianStandardDeviationSpeed = 0.7;//*m_pixelpermeter;//0.675: meter per half second
    private static final double m_GaussianMeanMaxSpeed =  4.44;//*m_pixelpermeter;// 3.535: meter per half second
    private static final double m_GaussianStandardDeviationMaxSpeed = 0.7;//*m_pixelpermeter;//0.435: meter per half second
    */

    /**
     *environment
     */
    private final CEnvironment m_environment;
    private final List<CInputFormat> m_inputdata;



    /**
     * constructor of the generator
     * @param p_stream ASL code as any stream e.g. FileInputStream
     * @throws Exception Thrown if something goes wrong while generating agents.
     */
    public CCarGenerator( @Nonnull final InputStream p_stream, final CEnvironment p_environment, final List<CInputFormat> p_inputdata ) throws Exception
    {
        super(
                // input ASL stream
                p_stream,
                // a set with all possible actions for the agent
                Stream.concat(
                        // we use all build-in actions of LightJason
                        CCommon.actionsFromPackage(),
                        // use the actions which are defined inside the agent class
                        CCommon.actionsFromAgentClass( IBaseRoadUser.class )

                        // build the set with a collector
                ).collect( Collectors.toSet() ),
                // variable builder
                new CVariableBuilder()
        );

        m_environment = p_environment;
        m_inputdata = Collections.synchronizedList(p_inputdata);
    }

    /**
     * generator method of the agent
     * @param p_data any data which can be put from outside to the generator method
     * @return returns an agent
     */
    @Override
    public final IBaseRoadUser generatesingle( @Nullable final Object... p_data )
    {
        // create agent with a reference to the environment
        final IBaseRoadUser l_car = new IBaseRoadUser( m_configuration, m_environment,1.2 ); //1.38//0.278 // max speed 8.33 per second
        CInputFormat l_input = m_inputdata.remove(0);

        l_car.setPosition(  l_input.m_startx_axis*m_pixelpermeter,  l_input.m_starty_axis*m_pixelpermeter );
        l_car.setGoalPedestrian(  l_input.m_endx_axis*m_pixelpermeter,  l_input.m_endy_axis*m_pixelpermeter );

        l_car.setradius( 1.5*m_pixelpermeter );//5
        l_car.setLengthradius( 1.5*m_pixelpermeter );

        l_car.setname( l_input.m_roaduser_id );
        l_car.settype( 2 );
        l_car.setmaxforce( 4.5*m_pixelpermeter );

        l_car.setSpeed( l_input.m_speed/2f*m_pixelpermeter* m_environment.getTimestep() );
        l_car.setMaxSpeed( l_input.m_max_speed*m_pixelpermeter *m_environment.getTimestep() );

        l_car.setVelocity( l_car.getSpeed(), l_car.getGoalposition(), l_car.getPosition() );
        l_car.updateParameter(12.3, 7, 7);

        if (  l_input.m_start_cycle == 0 )
        {
            m_environment.initialset(l_car);
            // add car to the car's list
            m_environment.initialCar(l_car);
        }
        else
        {
            m_environment.addCartoInitializeLater().computeIfAbsent(l_input.m_start_cycle, k -> new ArrayList<>()).add(l_car);//l_input.m_start_cycle
        }
        l_car.setTrue( l_input.m_start_cycle );

        return l_car;

    }
}

