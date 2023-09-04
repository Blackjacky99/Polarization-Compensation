import tkinter as tk

from scipy.optimize import minimize
from scipy.optimize import minimize_scalar
import numpy as np
import copy
import time
from ctypes import *
import clr
from System import Decimal
import time
import threading
from qncp import acq, optics, search
from numpy import inf
from tkinter import *


clr.AddReference("C:\\Windows\\Microsoft.NET\\assembly\GAC_64\\Newport.CONEXCC.CommandInterface\\v4.0_2.0.0.3__aab368c79b10b8be\\Newport.CONEXCC.CommandInterface.dll")

#to control the motor, we need to install thorlab kinesis, and the following .dll file is needed
#clr is a package called pythonnet which enables us to load .dll file
clr.AddReference('C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.DeviceManagerCLI.dll')
clr.AddReference('C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.TCube.DCServoCLI.dll')
clr.AddReference('C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.DeviceManagerCLI.dll')
clr.AddReference('C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.PolarizerCLI.dll')
clr.AddReference('C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.KCube.DCServoCLI.dll')
from Thorlabs.MotionControl.KCube.DCServoCLI import *
from Thorlabs.MotionControl.GenericMotorCLI.KCubeMotor import *
from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.PolarizerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.GenericMotorCLI.ControlParameters import *
from Thorlabs.MotionControl.GenericMotorCLI.AdvancedMotor import *
from Thorlabs.MotionControl.GenericMotorCLI.Settings import *
from Thorlabs.MotionControl.TCube.DCServoCLI import *

from CommandInterfaceConexCC import *
from ConexCC import *

#polarimeter from QNCP
def connect_Polarimeter():
    global pm_address1
    global pm1
    pm_address1 = search.list_resources()[0]
    pm1 = acq.thorlabs_polarimeter(pm_address1)
    pm1.trigger_data_collection()
    pm1.set_wavelength(795)


#-----connect and initialize the motor------
#serial number of the motor
Quarter1_serialNo_ = '83844099'
Quarter2_serialNo_ = '83847538'
Half_serialNo_ = '27004396'
SourceMotor1_com = "COM3"
SourceMotor2_com = "COM4"
is_ReadingPM = False

Source_H_motorPos = [47.5, 2.4]
Source_R_motorPos = [-48, 48]

DeviceManagerCLI.BuildDeviceList()

#initialize quarter1
def connect_quarter1(Quarter1_serialNo):
    global Quarter1_device
    Quarter1_device = TCubeDCServo.CreateTCubeDCServo(Quarter1_serialNo)
    Quarter1_device.Connect(Quarter1_serialNo)
    Quarter1_device.WaitForSettingsInitialized(6000)
    Quarter1_device.StartPolling(250)
    time.sleep(.1)
    Quarter1_device.EnableDevice()
    time.sleep(.1)
    Quarter1_motorConfiguration = Quarter1_device.LoadMotorConfiguration(Quarter1_serialNo);
    Quarter1_currentDeviceSettings = Quarter1_device.MotorDeviceSettings

#initialize quarter2
def connect_quarter2(Quarter2_serialNo):
    global Quarter2_device
    Quarter2_device = TCubeDCServo.CreateTCubeDCServo(Quarter2_serialNo)
    Quarter2_device.Connect(Quarter2_serialNo)
    Quarter2_device.WaitForSettingsInitialized(6000)
    Quarter2_device.StartPolling(250)
    time.sleep(.1)
    Quarter2_device.EnableDevice()
    time.sleep(.1)
    Quarter2_motorConfiguration = Quarter2_device.LoadMotorConfiguration(Quarter2_serialNo);
    Quarter2_currentDeviceSettings = Quarter2_device.MotorDeviceSettings

#initialize Half wave plate
def connect_half(Half_serialNo):
    global Half_device
    Half_device = KCubeDCServo.CreateKCubeDCServo(Half_serialNo)
    Half_device.Connect(Half_serialNo)
    Half_device.WaitForSettingsInitialized(6000)
    Half_device.StartPolling(250)
    time.sleep(.1)
    Half_device.EnableDevice()
    time.sleep(.1)
    device_info = Half_device.GetDeviceInfo()
    print(device_info.Description)
    m_config = Half_device.LoadMotorConfiguration(Half_serialNo, DeviceConfiguration.DeviceSettingsUseOptionType.UseFileSettings)
    m_config.DeviceSettingsName = "PRM1Z8"
    m_config.UpdateCurrentConfiguration()
    Half_device.SetSettings(Half_device.MotorDeviceSettings, True, False)



# move source waveplates to product a polarization (NewPort Motor)
def to_H():
    device1Target = Source_H_motorPos[0]
    device2Target = Source_H_motorPos[1]
    Tol = 0.1
    device1.move_absolute(device1Target)
    device2.move_absolute(device2Target)
    while device1.is_ready() == False or device2.is_ready() == False:
        time.sleep(.5)

    if (device1.get_cur_pos() - device1Target)>Tol or float(device2.get_cur_pos() - device2Target)>Tol:
        to_H()

def to_D():
    device1Target = 24.9
    device2Target = 2.3
    Tol = 0.1
    device1.move_absolute(device1Target)
    device2.move_absolute(device2Target)
    while device1.is_ready() == False or device2.is_ready() == False:
        time.sleep(.5)

    if (device1.get_cur_pos() - device1Target)>Tol or float(device2.get_cur_pos() - device2Target)>Tol:
        to_D()


def to_R():
    device1Target = Source_R_motorPos[0]
    device2Target = Source_R_motorPos[1]
    Tol = 0.1
    device1.move_absolute(device1Target)
    device2.move_absolute(device2Target)
    while device1.is_ready() == False or device2.is_ready() == False:
        time.sleep(.5)

    if (device1.get_cur_pos() - device1Target)>Tol or float(device2.get_cur_pos() - device2Target)>Tol:
        to_R()


def to_V():
    device1Target = 2.5
    device2Target = 2.5
    Tol = 0.1
    device1.move_absolute(device1Target)
    device2.move_absolute(device2Target)
    while device1.is_ready() == False or device2.is_ready() == False:
        time.sleep(.5)

    if (device1.get_cur_pos() - device1Target)>Tol or float(device2.get_cur_pos() - device2Target)>Tol:
        to_V()


def to_A():
    device1Target = -19.8
    device2Target = 3
    Tol = 0.1
    device1.move_absolute(device1Target)
    device2.move_absolute(device2Target)
    while device1.is_ready() == False or device2.is_ready() == False:
        time.sleep(.5)

    if (device1.get_cur_pos() - device1Target)>Tol or float(device2.get_cur_pos() - device2Target)>Tol:
        to_A()


def to_L():
    device1Target = 5
    device2Target = -42.6
    Tol = 0.1
    device1.move_absolute(device1Target)
    device2.move_absolute(device2Target)

    while device1.is_ready() == False or device2.is_ready() == False:
        time.sleep(.5)

    if (device1.get_cur_pos() - device1Target)>Tol or float(device2.get_cur_pos() - device2Target)>Tol:
        to_L()


def move_Quarter1(targetAngle):
    global Quarter1_is_complete
    Quarter1_is_complete = False
    Quarter1_device.MoveTo(Decimal(targetAngle), 60000)
    Quarter1_is_complete = True

def move_Quarter2(targetAngle):
    global Quarter2_is_complete
    Quarter2_is_complete = False
    Quarter2_device.MoveTo(Decimal(targetAngle), 60000)
    Quarter2_is_complete = True

def move_Half(targetAngle):
    global Half_is_complete
    Half_is_complete = False
    Half_device.MoveTo(Decimal(targetAngle), 60000)
    Half_is_complete = True


#get a rough position of half waveplate for S=[1 0 0] while browsing through half waveplate
def RecordPosition(num):
    global Half_is_complete
    global HalfTargetPosition
    while Half_is_complete != True:
        result1 = np.array([pm1.get_polarization_params()[3], pm1.get_polarization_params()[0], pm1.get_polarization_params()[1],pm1.get_polarization_params()[2]])
        if result1[1] - 1 < HalfPlate_search_tol:
            HalfTargetPosition = Half_device.Position

def search_HalfPlate():
    #read stokes vector from polarimeter while moving motor
    #(because we can only call one function from dll before it completed, so here use multiple threads to read the polarimeter at the same time)
    move_Half(0)
    Half_is_complete = False
    t1 = threading.Thread(target = move_Half, args=(180,))
    t2 = threading.Thread(target = RecordPosition, args=(10,))
    t1.start()
    t2.start()
    t1.join()
    t2.join()
    Half_device.MoveTo(HalfTargetPosition, 60000)
    Half_is_complete = True
    time.sleep(10)

    Half_jogPara = Half_device.GetJogParams()
    Half_jogPara.StepSize = Decimal(0.5)
    Half_jogPara.MaxVelocity = Decimal(5)
    Half_jogPara.JogMode = JogParametersBase.JogModes.SingleStep
    Half_device.SetJogParams(Half_jogPara)

    result = np.array([pm1.get_polarization_params()[0], pm1.get_polarization_params()[1], pm1.get_polarization_params()[2]])
    InitialDistance = np.sqrt((result[0]-1)**2+(result[1]-0)**2+(result[2]-0)**2)
    Half_device.MoveJog(MotorDirection.Forward, 0)
    time.sleep(1)
    result = np.array([pm1.get_polarization_params()[0], pm1.get_polarization_params()[1], pm1.get_polarization_params()[2]])
    if np.sqrt((result[0]-1)**2+(result[1]-0)**2+(result[2]-0)**2) > InitialDistance:
        while np.sqrt((result[0]-1)**2+(result[1]-0)**2+(result[2]-0)**2) > HalfPlate_search_tol:
            Half_device.MoveJog(MotorDirection.Backward, 0)
            time.sleep(1)
            result = np.array([pm1.get_polarization_params()[0], pm1.get_polarization_params()[1], pm1.get_polarization_params()[2]])
            print(result)
    elif np.sqrt((result[0]-1)**2+(result[1]-0)**2+(result[2]-0)**2) < InitialDistance:
        while np.sqrt((result[0]-1)**2+(result[1]-0)**2+(result[2]-0)**2) > HalfPlate_search_tol:
            Half_device.MoveJog(MotorDirection.Forward, 0)
            time.sleep(1)
            result = np.array([pm1.get_polarization_params()[0], pm1.get_polarization_params()[1], pm1.get_polarization_params()[2]])
            print(result)
    print('done')

#use minimize to find half wave plate position to achieve S=[1 0 0]
def search_HalfPlate2():
    res = minimize_scalar(HalfPlate_system, bounds=(0, 180), method='bounded')
    print('half wave search done')

class Trigger(Exception):
    pass

class ObjectiveFunctionWrapper:

    def __init__(self, fun, fun_tol=None):
        self.fun = fun
        self.best_x = None
        self.best_f = inf
        self.fun_tol = fun_tol or -inf
        self.number_of_f_evals = 0

    def __call__(self, x):
        _f = self.fun(x)
        self.number_of_f_evals += 1
        if _f < self.best_f:
            self.best_x, self.best_f = x, _f
        return _f

    def stop(self, *args):
        if self.best_f < self.fun_tol:
            raise Trigger


def QuarterPlates_system(rotation_input):
    theta1 = rotation_input[0]
    theta2 = rotation_input[1]
    global is_ReadingPM
    move_Quarter1(theta1)
    move_Quarter2(theta2)

    try:
        print('current position:')
        print(theta1 , theta2)
        is_ReadingPM = True
        result = np.array([pm1.get_polarization_params()[0],pm1.get_polarization_params()[1],pm1.get_polarization_params()[2]])
        is_ReadingPM = False
        time.sleep(0.1)
        print('distant=',np.sqrt((result[0]-0)**2+(result[1]-0)**2+(result[2]-1)**2))
        print('Stokes = ', result)
    except:
        time.sleep(2)
        print('current position:')
        print(theta1, theta2)
        is_ReadingPM = True
        result = np.array(
            [pm1.get_polarization_params()[0], pm1.get_polarization_params()[1], pm1.get_polarization_params()[2]])
        is_ReadingPM = False
        time.sleep(0.1)
        print('distant=', np.sqrt((result[0] - 0) ** 2 + (result[1] - 0) ** 2 + (result[2] - 1) ** 2))
        print('Stokes = ', result)

    return np.sqrt((result[0]-0)**2+(result[1]-0)**2+(result[2]-1)**2)


def HalfPlate_system(rotation_input):
    global is_ReadingPM
    move_Half(rotation_input)
    try:
        is_ReadingPM = True
        result = np.array([pm1.get_polarization_params()[0],pm1.get_polarization_params()[1],pm1.get_polarization_params()[2]])
        is_ReadingPM = False
        time.sleep(0.1)
        print('distant=',np.sqrt((result[0]-1)**2+(result[1]-0)**2+(result[2]-0)**2))
        print('Stokes = ', result)
    except:
        time.sleep(2)
        is_ReadingPM = True
        result = np.array(
            [pm1.get_polarization_params()[0], pm1.get_polarization_params()[1], pm1.get_polarization_params()[2]])
        is_ReadingPM = False
        time.sleep(0.1)
        print('distant=', np.sqrt((result[0] - 1) ** 2 + (result[1] - 0) ** 2 + (result[2] - 0) ** 2))
        print('Stokes = ', result)

    return np.sqrt((result[0]-1)**2+(result[1]-0)**2+(result[2]-0)**2)



def search_QuarterPlates():
    Quarter1_is_complete = True
    Quarter2_is_complete = True

    QuarterPlates_search_tol = 0.02
    QuarterSystem_wrapped = ObjectiveFunctionWrapper(QuarterPlates_system, QuarterPlates_search_tol)

    initial_guess = [85,85]
    move_Quarter1(initial_guess[0])
    move_Quarter2(initial_guess[1])

    bnds = ((0, 180), (0, 180))

    device1.read_cur_pos()
    device2.read_cur_pos()

    startTime = time.perf_counter()
    try:
        res = minimize(QuarterSystem_wrapped,initial_guess, method='Powell', bounds=bnds, callback=QuarterSystem_wrapped.stop)
        print("solution below tolerence was not found, move to the best")
        move_Quarter1(QuarterSystem_wrapped.best_x[0])
        move_Quarter2(QuarterSystem_wrapped.best_x[1])
    except Trigger:
        move_Quarter1(QuarterSystem_wrapped.best_x[0])
        move_Quarter2(QuarterSystem_wrapped.best_x[1])
        print('time elapse: ')
        print(time.perf_counter() - startTime)
        print(np.array([pm1.get_polarization_params()[0],pm1.get_polarization_params()[1],pm1.get_polarization_params()[2]]))

        print(f"Found f value below tolerance of {QuarterPlates_search_tol}\
                    in {QuarterSystem_wrapped.number_of_f_evals} f-evals:\
                    \nx = {QuarterSystem_wrapped.best_x}\
                    \nf(x) = {QuarterSystem_wrapped.best_f}")


def connect_sourceMotor1(com):
    global device1
    device1 = ConexCC(com, 10)

def connect_sourceMotor2(com):
    global device2
    device2 = ConexCC(com, 10)

connect_Polarimeter()


"""
connect_quarter1(Quarter1_serialNo_)
connect_quarter2(Quarter2_serialNo_)
connect_half(Half_serialNo_)

#NewPort Motor(for state preparation)
connect_sourceMotor1(SourceMotor1_com)
print("------")
connect_sourceMotor2(SourceMotor2_com)

to_R()
search_QuarterPlates()
to_H()
search_HalfPlate2()
"""



#--------User interface--------
def connectDevice():
    global Quarter1_serialNo_, Quarter1_serialNo_, Half_serialNo_, SourceMotor1_com, SourceMotor2_com
    try:
        Quarter1_serialNo_ = str(Quarter1_serial_box.get(1.0, 'end-1c'))
        Quarter2_serialNo_ = str(Quarter2_serial_box.get(1.0, 'end-1c'))
        Half_serialNo_ = str(Half_serial_box.get(1.0, 'end-1c'))
        SourceMotor1_com = str(Source_Motor1_box.get(1.0, 'end-1c'))
        SourceMotor2_com = str(Source_Motor2_box.get(1.0, 'end-1c'))
        connect_quarter1(Quarter1_serialNo_)
        connect_quarter2(Quarter2_serialNo_)
        connect_half(Half_serialNo_)
        # NewPort Motor(for state preparation)
        connect_sourceMotor1(SourceMotor1_com)
        print("------")
        connect_sourceMotor2(SourceMotor2_com)
        info_box.insert(END, '\n' + 'connect success')
    except Exception as error:
        print("An exception occurred:", error)
        info_box.insert(END,'\n'+str(error))


def PolCompensate():
    try:
        info_box.insert(END, '\n' + 'Polarization compensation procedure start')
        info_box.insert(END, '\n' + 'prepare source to R state')
        to_R()
        search_QuarterPlates()
        info_box.insert(END, '\n' + 'prepare source to H state')
        to_H()
        search_HalfPlate2()
        info_box.insert(END, '\n' + 'Polarization compensation end')
    except Exception as error:
        print("An exception occurred:", error)
        info_box.insert(END, '\n' + str(error))

def setSourceAngle():
    global Source_H_motorPos, Source_R_motorPos
    Source_H_motorPos = [round(float(Source1_H_angle_box1.get(1.0, 'end-1c')),2), round(float(Source1_H_angle_box2.get(1.0, 'end-1c')),2)]
    Source_R_motorPos = [round(float(Source1_R_angle_box1.get(1.0, 'end-1c')),2), round(float(Source1_R_angle_box2.get(1.0, 'end-1c')),2)]
    print(Source_H_motorPos)
    print(Source_R_motorPos)
    info_box.insert(END, '\n' + 'set source motor angle done')

root = Tk()
root.title('Polarization Compensation program')
root.geometry("500x400")
frame = Frame(root)
frame.pack()

stokes_text = Label(root, height = 10, width = 52)

Quarter1_serial_box = tk.Text(root)
Quarter2_serial_box = tk.Text(root)
Half_serial_box = tk.Text(root)
Quarter1_serial_label = Label(root)
Quarter2_serial_label = Label(root)
Half_serial_label = Label(root)

Source_Motor1_label = Label(root)
Source_Motor2_label = Label(root)
Source_Motor1_box = tk.Text(root)
Source_Motor2_box = tk.Text(root)
Source_Motor1_label.place(x=0, y=70, height=20, width=70)
Source_Motor2_label.place(x=0, y=90, height=20, width=70)
Source_Motor1_box.place(x=72, y=70, height=20, width=72)
Source_Motor2_box.place(x=72, y=90, height=20, width=72)
Source_Motor1_label['text'] ="Source1:"
Source_Motor2_label['text'] ="Source2:"
Source_Motor1_box.insert(END,str(SourceMotor1_com))
Source_Motor2_box.insert(END,str(SourceMotor2_com))


Quarter1_serial_label.place(x=0, y=10, height=20, width=70)
Quarter2_serial_label.place(x=0, y=30, height=20, width=70)
Half_serial_label.place(x=0, y=50, height=20, width=70)
Quarter1_serial_label['text'] = "Q1 serial:"
Quarter2_serial_label['text'] = "Q2 serial:"
Half_serial_label['text'] = "H1 serial:"
Quarter1_serial_box.place(x=72, y=10, height=20, width=72)
Quarter2_serial_box.place(x=72, y=30, height=20, width=72)
Half_serial_box.place(x=72, y=50, height=20, width=72)
Quarter1_serial_box.insert(END,str(Quarter1_serialNo_))
Quarter2_serial_box.insert(END,str(Quarter2_serialNo_))
Half_serial_box.insert(END,str(Half_serialNo_))

connect_btn = Button(root,text="Connect \n Devices",command = connectDevice)
connect_btn.place(x=160, y=45, height=50, width=72)

Source1_H_angle_label = Label(root)
Source1_H_angle_label['text'] ="Motor angle for H:"
Source1_H_angle_label.place(x=240, y=10, height=20, width=150)
Source1_H_angle_box1 = tk.Text(root)
Source1_H_angle_box2 = tk.Text(root)
Source1_H_angle_box1.place(x=260, y=35, height=20, width=32)
Source1_H_angle_box2.place(x=295, y=35, height=20, width=32)
Source1_H_angle_box1.insert(END,str(Source_H_motorPos[0]))
Source1_H_angle_box2.insert(END,str(Source_H_motorPos[1]))

Source1_R_angle_label = Label(root)
Source1_R_angle_label['text'] ="Motor angle for R:"
Source1_R_angle_label.place(x=240, y=60, height=20, width=150)
Source1_R_angle_box1 = tk.Text(root)
Source1_R_angle_box2 = tk.Text(root)
Source1_R_angle_box1.place(x=260, y=80, height=20, width=32)
Source1_R_angle_box2.place(x=295, y=80, height=20, width=32)
Source1_R_angle_box1.insert(END,str(Source_R_motorPos[0]))
Source1_R_angle_box2.insert(END,str(Source_R_motorPos[1]))

setSourceAngle_btn = Button(root,text="Set \n angles",command = setSourceAngle)
setSourceAngle_btn.place(x=380, y=45, height=50, width=72)

stokes_text.place(x = 0, y = 200)



def update_text():
    global is_ReadingPM
    if not is_ReadingPM:
        try:
            stokes_text['text'] = 'S = ' + str( np.array([round(pm1.get_polarization_params()[0],3),round(pm1.get_polarization_params()[1],3),round(pm1.get_polarization_params()[2],3)]))
        except:
            time.sleep(2)
            stokes_text['text'] = 'S = ' + str(np.array([round(pm1.get_polarization_params()[0], 3), round(pm1.get_polarization_params()[1], 3),round(pm1.get_polarization_params()[2], 3)]))
    root.after(600, update_text) # run itself again after 1000 ms
update_text()

def PolCompensate_Btn_press():
    task = threading.Thread(target=PolCompensate)
    task.start()

Pol_compensate_Btn = Button(root,text="Start \n Procedure",command = PolCompensate_Btn_press)
Pol_compensate_Btn.place(x=20, y=130, height=50, width=72)

info_box = tk.Text(root)
info_box.place(x=100, y=130, height=130, width=350)

tk.mainloop()
