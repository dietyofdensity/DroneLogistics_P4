from __future__ import print_function
import logging
import time
from threading import Thread
from threading import Timer

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

import logging
import time
from threading import Thread

import cflib
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper

from vicon_dssdk import ViconDataStream

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
logging.basicConfig(level=logging.ERROR) # konfigurær logging
client = ViconDataStream.RetimingClient() # opret


class controllsystem: # contains all the pid controlles in the first block of the dynamic system aswell as the truncating limiters
    rollpid=0
    rolltrnc=0
    pitchpid=0
    pitchtrnc = 0
    forcepid=0
    forcetrnc=0
    setpoint=[0,0,0]

    def __init__(self,p1,p2,p3,t1,t2,t3,tc,dt): # sets up pid controllers with values from p1,p2,p3

        self.altpid = PID(4.5,0,7.5,0.000,0,20/1000,3500,-2000,10500)
        self.rollpid = PID(0.05,0,0.05,0.000,0,20/1000,0.18,(-0.18),580)
        self.pitchpid = PID(0.05,0,0.05,0.000,0,20/1000,0.18,(-0.18),580)

        #self.altpid = PID(4.5, 0, 7.5, 0.000, 0, 20 / 1000, , , )
        #self.rollpid = PID(0.8, 0, 1.2, 0.000, 0, 20 / 1000, , , )
        #self.pitchpid = PID(0.8, 0, 1.2, 0.000, 0, 20 / 1000, , , )




    def update(self , meas,sett): # take the xyz cordinate
        self.rollpid.Step(meas[0], sett[0])
        self.pitchpid.Step(meas[1], sett[1])
        a=0
        if(sett[2]<500):
            a=100

        self.altpid.Step(meas[2], sett[2]*1.1+a)

        return [ self.rollpid.command_sat,
                 self.pitchpid.command_sat,
                 self.altpid.command_sat]



class PID:
    """ This class implements a PID controller.
    """

    def __init__(self, Kp, Ki, Kd, Kaw, T_C, T, max, min, max_rate):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.Kaw = Kaw  # Anti-windup gain
        self.T_C = T_C  # Time constant for derivative filtering
        self.T = T  # Time step
        self.max = max  # Maximum command
        self.min = min  # Minimum command
        self.max_rate = max_rate  # Maximum rate of change of the command
        self.integral = 0  # Integral term
        self.err_prev = 0  # Previous error
        self.deriv_prev = 0  # Previous derivative
        self.command_sat_prev = 0  # Previous saturated command
        self.command_prev = 0  # Previous command
        self.command_sat = 0  # Current saturated command
        self.command = 0  # Current command

    def Step(self, measurement, setpoint):
        """ Execute a step of the PID controller.

        Inputs:
            measurement: current measurement of the process variable
            setpoint: desired value of the process variable
        """

        # Calculate error
        err = setpoint - measurement

        # Update integral term with anti-windup
        self.integral += self.Ki * err * self.T + self.Kaw * (self.command_sat_prev - self.command_prev) * self.T

        # Calculate filtered derivative
        deriv_filt = (err - self.err_prev + self.T_C * self.deriv_prev) / (self.T + self.T_C)
        self.err_prev = err
        self.deriv_prev = deriv_filt

        # Calculate command using PID equation
        self.command = self.Kp * err + self.integral + self.Kd * deriv_filt

        # Store previous command
        self.command_prev = self.command

        # Saturate command
        if self.command > self.max:
            self.command_sat = self.max
        elif self.command < self.min:
            self.command_sat = self.min
        else:
            self.command_sat = self.command

        # Apply rate limiter
        if self.command_sat > self.command_sat_prev + self.max_rate * self.T:
            self.command_sat = self.command_sat_prev + self.max_rate * self.T
        elif self.command_sat < self.command_sat_prev - self.max_rate * self.T:
            self.command_sat = self.command_sat_prev - self.max_rate * self.T

        # Store previous saturated command
        self.command_sat_prev = self.command_sat
def cos(x):
    return np.cos(x)
def sin(x):
    return np.sin(x)
class AltHoldExample: # controlls the connection to the dron and vicon

    def __init__(self, link_uri , client , name):
        """ Initialize and run the example with the specified link_uri """
        self.msperiod=20
        self.land=False
        self._cf = Crazyflie(rw_cache='./cache')

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)

        self._client = client
        self._Vicon_object_name = name
        self.msperiod = 20
        self.is_connected = True
        self.controller = controllsystem([1, 0, 1], [1, 0, 1], [7, 0.4, 3], [-0.17, 0.17], [-0.17, 0.17], [0.0, 5000.0],
                                         1,1)

        # Variable used to keep main loop occupied until disconnect
        print('Connecting to %s' % link_uri)
    land=False
    def landing(self):
        #self.land=True
        return
    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)
        time.sleep(1)
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        print("initiating motors")

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='quaternion', period_in_ms=self.msperiod)
        self._lg_stab.add_variable('kalman.q0', 'float')
        self._lg_stab.add_variable('kalman.q1', 'float')
        self._lg_stab.add_variable('kalman.q2', 'float')
        self._lg_stab.add_variable('kalman.q3', 'float')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        time.sleep(1)
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        print("initiating motors")
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')
        # calibrate the drone rotation
        #(VIPOS,VIROT) = self.getdronepos()
        # to do
        # add the drones roll pitch and yaw as logging parameters, might be in quaternions
        # add

        #dronerot=[ self._cf.param.get_value(),self._cf.param.get_value(),self._cf.param.get_value()]


        # Start a timer to disconnect in 10s
        print("timer start")
        t = Timer(80, self.landing)
        tp1= Timer(10, self.upstate)
        tp2= Timer(20, self.upstate)
        tp3 = Timer(30, self.upstate)
        tp4 = Timer(40, self.upstate)
        tp5 = Timer(50, self.upstate)
        tp6 = Timer(60, self.upstate)
        tp7 = Timer(70, self.upstate)


        self.pointstate=0
        print("timer started")
        t.start()
        tp1.start()
        tp2.start()
        tp3.start()
        tp4.start()
        tp5.start()
        tp6.start()
        tp7.start()

        #Thread(target=self._hover_test).start()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
    def upstate(self):
        self.pointstate+=1

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def getdronepos(self):
        try:
            self._client.UpdateFrame()
            ret=[0,0,0]
            VITARG = [0, 0, 0]

            subjectNames = self._client.GetSubjectNames()
            for subjectName in subjectNames:
                #print(subjectName)

                if(subjectName=="target"):
                    segmentNames = client.GetSegmentNames(subjectName)
                    # print(segmentNames)
                    for segmentName in segmentNames:
                        Vipos = client.GetSegmentGlobalTranslation(subjectName, segmentName)
                        # print( segmentName, 'has global rotation( helical )', client.GetSegmentGlobalRotationHelical( subjectName, segmentName ) )
                        VIrot = client.GetSegmentGlobalRotationEulerXYZ(subjectName, segmentName)
                        VITARG=Vipos[0]
                        ret[2]=VITARG
                else:
                    segmentNames = client.GetSegmentNames(subjectName)
                    #print(segmentNames)
                    for segmentName in segmentNames:
                        Vipos = client.GetSegmentGlobalTranslation(subjectName, segmentName)
                        # print( segmentName, 'has global rotation( helical )', client.GetSegmentGlobalRotationHelical( subjectName, segmentName ) )
                        VIrot = client.GetSegmentGlobalRotationEulerXYZ(subjectName, segmentName)
                        #print(str(Vipos) +"vipos")
                        ret[0]=Vipos[0]
                        ret[1]=VIrot[0]
            return ret
        except:
            print("no update")

    def regulation(self):
        return 2
    def rmat(self,yaw,pitch,roll):

        np.array([[cos(yaw) * cos(pitch), sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll),
                   sin(yaw) * sin(roll) - cos(yaw) * sin(pitch) * cos(roll)],
                  [-sin(yaw) * cos(pitch), cos(yaw) * cos(roll) - sin(yaw) * sin(pitch) * sin(roll),
                   cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll)],
                  [sin(pitch), -cos(pitch) * sin(roll), cos(pitch) * cos(roll)]])
    def rot2quat(self,mat):
        R11 = mat[0, 0]
        R22 = mat[1, 1]
        R33 = mat[2, 2]

        q1 = (1 / 2) * np.sqrt(1 + R11 + R22 + R33)
        q2 = (mat[2, 1] - mat[1, 2]) / (4 * q1)
        q3 = (mat[0, 2] - mat[2, 0]) / (4 * q1)
        q4 = (mat[1, 0] - mat[0, 1]) / (4 * q1)
        return [q1,q2,q3,q4]

    def quaternion_rotation_matrix(self,Q):
        """
        Covert a quaternion into a full three-dimensional rotation matrix.

        Input
        :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3)

        Output
        :return: A 3x3 element matrix representing the full 3D rotation matrix.
                 This rotation matrix converts a point in the local reference
                 frame to a point in the global reference frame.
        """
        # Extract the values from Q
        q0 = Q[0]
        q1 = Q[1]
        q2 = Q[2]
        q3 = Q[3]

        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)

        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)

        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1

        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                               [r10, r11, r12],
                               [r20, r21, r22]])

        return rot_matrix

    def force_to_int(self,force):
        a=0.0009
        b=-3.8404
        # force=a*pd+b

        # force/a - b=pd
        return (force*1000/a-b)

    def calibrateorigin(self,drone_rotation,vipos,virotation):
        """this function calculates the difference between the 0 rotations of the drone and the vicon system,
        then resets the drone rotation """
        print("quaternion of 0,0,0 matrix"+str(self.rot2quat( self.rmat(0,0,0) )))
        # convert the rotation from the vicon system into a rotation matrix,
        vimatrix=self.rmat(virotation[2],virotation[1],virotation[0])
        # if the inital position is aligned with the grid we should get a rotation that resembles a (0,0,0) rotation matrix
        # if there is an offset it may be considered static and can probably be fixed by adding in a fixed calibration matrix which
        # defines the offset between the rotation matrix assigned to the cf object in the vicon system and the drones actual rotation.
        print("quaternion of vicon matrix" +str(self.rot2quat( vimatrix)))

        dronemat=self.rmat((drone_rotation[2],drone_rotation[1],drone_rotation[0]))
        # this rotation matrix can be cheked against the drones current quaternions
        # ^^^ yaw might have to be negative as the drone uses left hand systems
        print("quaternion of current matrix" + str(self.rot2quat( dronemat)))


        vi2drone=np.matmul(vimatrix,np.invert(dronemat))
        R11=vi2drone[0,0]
        R22=vi2drone[1,1]
        R33=vi2drone[2,2]



        q1=(1/2) * np.sqrt(1+R11+R22+R33)
        q2 = (vi2drone[2, 1] - vi2drone[1, 2]) /(4* q1)
        q3 = (vi2drone[0, 2] - vi2drone[2, 0]) / (4* q1)
        q4 = (vi2drone[1, 0] - vi2drone[0, 1]) / (4* q1)
        # we should be able to just set the value, as we do this while the drone is not moveing
        # we should be able to calculate the static ofset the drone has in relation to the vicon system
        # we will realign this by inserting a yaw rotation
        self._cf.param.set_value("kalman.q0", q1)
        self._cf.param.set_value("kalman.q1", q2)
        self._cf.param.set_value("kalman.q2", q3)
        self._cf.param.set_value("kalman.q3", q4)

    pointstate=0
    pointcounts=0
    pointtarget=50*5
    #pointstates=[[0,0,1500],[1000,0,1500],[0,0,1500],[0,0,1000],[0,0,500]] # balast test
    pointstates = [[0, 0, 1000], [1000, 0, 1000], [1000, 0, 1700], [0, 0, 1700], [0, 0, 1000],[0,0,200]] # path test
    firstpos=False
    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        if(self.firstpos):
            VIret = self.getdronepos()  # get the drones position from the vicon system
            VIPOS = VIret[0]
            VIROT = VIret[1]
            TARG = VIret[2]
            self.pointstates=[[0, 0, 1000], [TARG[0], TARG[1], 1000], [TARG[0], TARG[1], TARG[2]+200]]
            self.firstpos=False



        print("landing:"+str(self.land))
        if self.land==False:
            VIret=self.getdronepos() # get the drones position from the vicon system
            VIPOS=VIret[0]
            VIROT=VIret[1]
            TARG=VIret[2]
            #print(f'[{timestamp}][{logconf.name}][zpos:{VIPOS}: ', end='')
            #for name, value in data.items():
            #    print(f'{name}: {value:3.3f} ', end='')
            #print()
            target=[0,0,0]


            try:
                target=self.pointstates[self.pointstate] # establish goal point
            except:
                self.land=True


            controlin=[ target[0]-VIPOS[0],  target[1]-VIPOS[1], target[2]-VIPOS[2]]
            retvar = self.controller.update(meas=VIPOS,sett=target) # run       the    refrence    value    throug    the    controll    systen
            #print(str(retvar[2]*13)+" was rvar, with input"+str(controlin)+": pos was"+str(VIPOS))
            int16max=65434
            #  maby think of of inverse clamping the sensitivety of the angular controls by how far the curent thrust is above hover
            #self._cf.commander.send_setpoint(np.rad2deg(-retvar[0]/588) ,-np.rad2deg(retvar[1]/588) ,0,int(retvar[2])*13) # send the generated comand to the drone
            thval=45500+int(retvar[2])*6

            if(thval > int16max):
                thval=int16max
            self._cf.commander.send_setpoint(-np.rad2deg(retvar[1] ), np.rad2deg(retvar[0] ), 0,thval)  # send the generated comand to the drone

            print("retvar "+str(-np.rad2deg(retvar[1] ))+" , "+str(np.rad2deg(retvar[0] ))+" thrust: "+ str(int(retvar[2])*8) + ": pos was:" + str(VIPOS))

            # PERHAPS THE pid does not handle negative setpoints well.
            if np.sqrt(np.power((controlin[0]),2)+np.power((controlin[1]),2)+np.power((controlin[2]),2))<10:
                self.pointcounts+=1
                if(self.pointcounts>self.pointtarget):
                    self.pointcounts=0
                    self.pointstate+=1
                try:
                    print("going to "+str(self.pointstates[self.pointstate]))
                except:
                    print("landing")
                    self.land=True
            else:
                self.pointcounts=self.pointcounts
                #self.pointcounts=0

        else:
            VIret = self.getdronepos()  # get the drones position from the vicon system
            VIPOS = VIret[0]
            VIROT = VIret[1]
            target = [0, 0, 260]  # establish goal point
            controlin = [target[0] - VIPOS[0], target[1] - VIPOS[1], target[2] - VIPOS[2]]
            retvar = self.controller.update(meas=VIPOS,
                                            sett=target)  # run       the    refrence    value    throug    the    controll    systen
            # print(str(retvar[2]*13)+" was rvar, with input"+str(controlin)+": pos was"+str(VIPOS))
            int16max = 65534
            #  maby think of of inverse clamping the sensitivety of the angular controls by how far the curent thrust is above hover
            # self._cf.commander.send_setpoint(np.rad2deg(-retvar[0]/588) ,-np.rad2deg(retvar[1]/588) ,0,int(retvar[2])*13) # send the generated comand to the drone
            self._cf.commander.send_setpoint(-np.rad2deg(retvar[1] ), np.rad2deg(retvar[0] ), 0, 42000 + int(retvar[2]) * 6)  # send the generated comand to the drone


            self._cf.commander.send_setpoint(0, 0, 0, 0)
            self._cf.close_link()

    HOVERACOUBT=0

    def _stab_log_data_targetland(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        print("landing:" + str(self.land))
        if self.land == False:
            VIret = self.getdronepos()  # get the drones position from the vicon system
            VIPOS = VIret[0]
            VIROT = VIret[1]
            TARG = VIret[2]
            # print(f'[{timestamp}][{logconf.name}][zpos:{VIPOS}: ', end='')
            # for name, value in data.items():
            #    print(f'{name}: {value:3.3f} ', end='')
            # print()
            target = [0, 0, 0]
            if(self.pointstate==0): # takeoff
                target = [0,0,1000]
            elif(self.pointstate==1): # move above target
                target = [TARG[0],TARG[1],1000]
            elif(self.pointstate==2): # lower toward target
                target = [TARG[0],TARG[1],TARG[2]+500]
            elif(self.pointstate==3): #land
                target = [TARG[0],TARG[1],TARG[2]+200]
                if np.sqrt(
                        np.power((VIPOS[0]-TARG[0]), 2) + np.power((VIPOS[1]-TARG[1]), 2) + np.power((VIPOS[2]-TARG[2]), 2)) < 100:
                    self.HOVERACOUBT += 1
                    if (self.HOVERACOUBT > 10):
                        self.HOVERACOUBT = 0
                        self.pointstate += 1
                        self.land=True
                    try:
                        print("going to " + str(self.pointstates[self.pointstate]))
                    except:
                        print("landing")
                        self.land = True
                else:
                    self.HOVERACOUBT=0
            elif(self.pointstate==4):
                self._cf.commander.send_setpoint(0,0,0,0)



            controlin = [target[0] - VIPOS[0], target[1] - VIPOS[1], target[2] - VIPOS[2]]
            retvar = self.controller.update(meas=VIPOS,
                                            sett=target)  # run       the    refrence    value    throug    the    controll    systen
            # print(str(retvar[2]*13)+" was rvar, with input"+str(controlin)+": pos was"+str(VIPOS))
            int16max = 65534
            #  maby think of of inverse clamping the sensitivety of the angular controls by how far the curent thrust is above hover
            # self._cf.commander.send_setpoint(np.rad2deg(-retvar[0]/588) ,-np.rad2deg(retvar[1]/588) ,0,int(retvar[2])*13) # send the generated comand to the drone
            if(self.land):
                self._cf.commander.send_setpoint(0,0,0,0)
            else:
                self._cf.commander.send_setpoint(-np.rad2deg(retvar[1]), np.rad2deg(retvar[0]), 0,
                                             45500 + int(retvar[2]) * 6)  # send the generated comand to the drone

                print("retvar " + str(-np.rad2deg(retvar[1])) + " , " + str(
                    np.rad2deg(retvar[0])) + " thrust: " + str(int(retvar[2]) * 8) + ": pos was:" + str(VIPOS))

            # PERHAPS THE pid does not handle negative setpoints well.


        else:

            self._cf.commander.send_setpoint(0, 0, 0, 0)
            self._cf.close_link()


    def _hover_test(self):
        print("sending initial thrust of 0")

        self._cf.commander.send_setpoint(0,0,0,0)
        time.sleep(0.5)
        self._cf.commander.send_setpoint(0, 0, 0, 40080)
        time.sleep(0.1)
        print("putting in althold")
        self._cf.param.set_value("flightmode.althold","1")

        print("Stay in althold for 7s")

        it=0

        while it<700:
            self._cf.commander.send_setpoint(0,0,0,37067)
            self._cf.param.set_value("flightmode.althold","1")
            time.sleep(0.01)
            it+=1

        print("Close connection")
        self._cf.commander.send_setpoint(0,0,0,0)
        self._cf.close_link()


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    client.Connect("192.168.10.1:801")

    # Check the version
    print('Version', client.GetVersion())

    client.SetAxisMapping(ViconDataStream.Client.AxisMapping.EForward, ViconDataStream.Client.AxisMapping.ELeft,
                          ViconDataStream.Client.AxisMapping.EUp)
    xAxis, yAxis, zAxis = client.GetAxisMapping()
    print('X Axis', xAxis, 'Y Axis', yAxis, 'Z Axis', zAxis)

    # client.SetMaximumPrediction( 10 )
    print('Maximum Prediction', client.MaximumPrediction())
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    for i in available:
        print(i[0])

    if len(available) > 0:
        le = AltHoldExample(uri,client,"cf")

    else:
        print('No Crazyflies found, cannot run example')
