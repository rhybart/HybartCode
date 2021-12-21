#!/usr/bin/env python3
#coding: utf-8
#import faulthandler; faulthandler.enable()
from coamp import CoAMP, dummyCoAMP
from processemg import EMGChannel
from time import time, sleep
from sys import argv
import newGUInstaf as newGUInstaf
import multiprocessing
from controlexo_test import dummyExo
import sender
import RPi.GPIO as GPIO
from DataLogger import dataLogger
from flexsea import flexsea as flex
from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe

dephy= flex.FlexSEA()
portsFilePath = 'ports.yaml'

def main():
    #Initialize the CoAMP EMG class, can be a test or actual run

    if len(argv)>1 and argv[1] == "test":
        Lcoamp = dummyCoAMP(0,0x150)
        Rcoamp = dummyCoAMP(1,0x150)
        Lcoamp.init_coamp()
        Rcoamp.init_coamp()
    else:
        Lcoamp = CoAMP(0,0x150) #Set the Left coamp to Can0
        Rcoamp = CoAMP(1,0x150) #Set the Right coamp to Can1
        Lcoamp.init_coamp()
        Lcoamp.start_can_stream()
        Rcoamp.init_coamp()
        Rcoamp.start_can_stream()
#


    channelColors = ['gray','brown','red','orange','yellow','green','blue','purple']
    #Initialize all EMGChannel Filters
    Lchannels = [EMGChannel("lsoleus"), EMGChannel("lta"), EMGChannel("llg"), EMGChannel("lmg")]
    Rchannels = [EMGChannel("rsoleus"), EMGChannel("rta"), EMGChannel("rlg"), EMGChannel("rmg")]

    #Initialize the Exo class
    ports, baudRate = fxu.load_ports_from_file(portsFilePath)
    if len(argv)>1 and argv[1] == "test":
        Lexo = dummyExo()
        Rexo = dummyExo()
        Lexo.init_exo()
        Rexo.init_exo()
    else:
        Rexo = dephy.open(ports[1],baudRate,0)
        dephy.start_streaming(Rexo,500,False)
        dephy.set_gains(Rexo,80,700,0,0,0,100)

        Lexo = dephy.open(ports[0],baudRate,0)
        dephy.start_streaming(Lexo,500,False)
        dephy.set_gains(Lexo,80,700,0,0,0,100)


        #right should always be in upper usb and left in lower

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(26,GPIO.OUT)
    sync_chan=GPIO.PWM(26, .25)
    sync_chan.start(0)
    #a pipe to communicate with the GUI
    main_conn, gui_conn = multiprocessing.Pipe() #For the GUI

    #default GUI parameters
    defaultParameters = [-1, -1, 0, 0, 0, 0, 0, 0, 0, 0,1]


    #Initialize parameters for Left and right specifically
    Lparameters=[-1 -1 -1 -1,-1]
    Rparameters= [-1 -1 -1 -1,-1]
    Lparameters=[defaultParameters[x] for x in [0,2,4,6,8]]
    Rparameters= [defaultParameters[y] for y in [1,3,5,7,9]]


    
    print("Please double check that the EMG electrodes are:")
    for color,chan in zip(channelColors,Lchannels):
        print(f"\t{chan.name}\t- {color}")
    #asks user to input file name to save data,
    #will overwrite a file with the same name
    baseFilename= input("\nType Name of File to Save: ")
    input("\nPress <Enter> To Begin The Test")
    #names of columns for data file
    Lcolumn_names= 'LTime, LTime_pipe, LExoTime, LSORaw, LTARaw, LSOFilt,'\
                  'LSetCurrent, LMotorCurrent, LMotorVoltage, LAnkleAngle,'\
                  'LMotorAngle,'\
                  'LLG, LMG, LAX, LAY, LAZ, LGX, LGY, LGZ,'

    Rcolumn_names= 'RTime, RTime_pipe, RExoTime,RSORaw, RTARaw, RSOFilt,'\
                  'RSetCurrent,  RMotorCurrent, RMotorVoltage, RAnkleAngle,'\
                  'RMotorAngle, RLG,'\
                  'RMG, RAX, RAY, RAZ, RGX, RGY, RGZ, sync'
    column_names_All= Lcolumn_names + Rcolumn_names


    gui_p = multiprocessing.Process(target=newGUInstaf.gui_event_loop, args=(gui_conn,defaultParameters,baseFilename), daemon=True)
    gui_p.start()
    #create data file
    dall=dataLogger(f"{baseFilename}_Data", column_names_All)
    start_Time = time()


    graphChange=1
    sync_sig=[]
    sync_start=False
    
    try:
        while True:

            if main_conn.poll():
             #If there is new data in the pipe
                newMsg = main_conn.recv() #Receive the new message
                if newMsg == "TERMINATE":
                 #If the GUI is closed/exited
                    dephy.send_motor_command(Lexo, fxe.FX_NONE, 0) #set exo to no current
                    dephy.stop_streaming(Lexo)
                    sleep(0.1)
                    dephy.close(Lexo)
                    dephy.send_motor_command(Rexo, fxe.FX_NONE, 0) #set exo to no current
                    dephy.stop_streaming(Rexo)
                    sleep(0.1)
                    dephy.close(Rexo)
 #saves all data from the data logger to the file
                    print("Stopping...")
                    print("Stopping sync...")
                    sync_chan.stop() #stops sending sync wave from channel 26
                    print('input channel closed')
                    GPIO.cleanup() #cleans up the Analog in/out channel
                    print("saving data...")
                    dall.writeOut()

                    break #Exit the main loop/end the program
                elif newMsg=="sync pressed" and sync_start==False:
                    sync_chan.ChangeDutyCycle(50)
                    sync_start=True
                #turns off the square wave if the sync gui value is anything but 1
                elif newMsg=="sync pressed" and sync_start==True:
                    sync_chan.ChangeDutyCycle(0)
                    sync_start=False
                else:
                    #Break up the message into left and right parameters
                    #gain, controller, thresh, and min current values for left and right
                    Lparameters=[newMsg[x] for x in [0,2,4,6,8]]
                    Rparameters= [newMsg[y] for y in [1,3,5,7,9]]
                    #sends those values from the gui to each leg's subprocess
                    graphChange=newMsg[10]

                    #if the sync gui value is 1, it sends 1 50% duty cycle 1 Hz square wave
                    #out of channel 26

            LemgRaw = Lcoamp.read_can_stream()
            RemgRaw = Rcoamp.read_can_stream()
            LfiltVal, LhpFilteredData, LrecData = Lchannels[0].process_sample(LemgRaw[0])
            LfiltVal2, LhpFilteredData2, LrecData= Lchannels[1].process_sample(LemgRaw[1])
            RfiltVal, RhpFilteredData, RrecData = Rchannels[0].process_sample(RemgRaw[0])
            RfiltVal2, RhpFilteredData2, RrecData= Rchannels[1].process_sample(RemgRaw[1])
            #If new variables are read from the gui assign them to newMsg

            #parameters read from GUI
            #must be a integers
            Lcont = Lparameters[0]
            Lgain = Lparameters[1]
            Lmincurr = Lparameters[2]
            Lthresh = Lparameters[3]
            Ltathresh=Lparameters[4]

            Rcont = Rparameters[0]
            Rgain = Rparameters[1]
            Rmincurr = Rparameters[2]
            Rthresh = Rparameters[3]
            Rtathresh=Rparameters[4]

            #exoTargetCurrent=gain*380
            LexoTargetCurrent = int(LfiltVal*Lgain/100) #based on the filtered soleus * the gain
            RexoTargetCurrent = int(RfiltVal*Rgain/100)
            #NOTE gain parameter is divided by 100 to allow finer control
            if LexoTargetCurrent > 7600:
                 #Safety mechanism to limit the maximum current sent to exo
                LexoTargetCurrent = 7600 #7600mA is equivalent to 20A along the q-axis
            if RexoTargetCurrent <-7600:
                RexoTargetCurrent= -7600

            #keeps the exo belt taut by not going to 0
            if (abs(LexoTargetCurrent) < abs(Lmincurr)) or (Lthresh!=0 and abs(LexoTargetCurrent) < abs(Lthresh)):
                LexoTargetCurrent = Lmincurr
            if (abs(RexoTargetCurrent) < abs(Rmincurr)) or (Rthresh!=0 and abs(RexoTargetCurrent) < abs(Rthresh)):
                RexoTargetCurrent = Rmincurr

            if LfiltVal2> Ltathresh and Ltathresh>0 and Lgain>0:
                #LexoTargetCurrent=-.009*LfiltVal*(Lgain/100)+700.2644
                LexoTargetCurrent=Lmincurr
            if RfiltVal2>Rtathresh and Rtathresh>0 and Rgain>0:
                #RexoTargetCurrent=-.009*RfiltVal*(Rgain/100)-700.2644
                RexoTargetCurrent=Rmincurr
            if Lcont == 1:
                #If the controller is set to "on" (1)
                dephy.send_motor_command(Lexo, fxe.FX_CURRENT, LexoTargetCurrent)
            else:
                dephy.send_motor_command(Lexo, fxe.FX_CURRENT, 0)
            LTime=time()
            if Rcont == 1:
                #If the controller is set to "on" (1)
                dephy.send_motor_command(Rexo, fxe.FX_CURRENT, RexoTargetCurrent)
            else:
                dephy.send_motor_command(Rexo, fxe.FX_CURRENT, 0)
            RTime=time()

#
            LexoState = dephy.read_device(Lexo)
            RexoState = dephy.read_device(Rexo)


            LSORaw=LemgRaw[0]
            LTARaw=LemgRaw[1]
            LSOFilt=LfiltVal
            LSetCurrent=LexoTargetCurrent,
            LLG=LemgRaw[2]
            LMG=LemgRaw[3]
            LTaFilt=LfiltVal2

            RSORaw=RemgRaw[0]
            RTARaw=RemgRaw[1]
            RSOFilt=RfiltVal
            RSetCurrent=RexoTargetCurrent
            RLG=RemgRaw[2]
            RMG=RemgRaw[3]
            RTAFilt=RfiltVal2

            LTime2=time() #including a time value here since this is when the
            #data is saved vs created, to see if there is a difference in the
            #timing of the control signal based on this time
            LExoTime=LexoState.state_time
            LMotorCurrent=LexoState.mot_cur
            LMotorVoltage=LexoState.mot_volt
            LAnkleAngle=LexoState.ank_ang
            LMotorAngle=LexoState.mot_ang
            LAX=LexoState.accelx
            LAY=LexoState.accely
            LAZ=LexoState.accelz
            LGX=LexoState.gyrox
            LGY=LexoState.gyroy
            LGZ=LexoState.gyroz


            RTime2= time()
            RExoTime=RexoState.state_time
            RMotorCurrent=RexoState.mot_cur
            RMotorVoltage=RexoState.mot_volt
            RAnkleAngle=RexoState.ank_ang
            RMotorAngle=RexoState.mot_ang
            RAX=RexoState.accelx
            RAY=RexoState.accely
            RAZ=RexoState.accelz
            RGX=RexoState.gyrox
            RGY=RexoState.gyroy
            RGZ=RexoState.gyroz
            sync_sig=GPIO.input(26)


            #creates an array for the data logger
            datalogleft= [LTime]+ [LTime2]+ [LExoTime]+ [LSORaw]+ [LTARaw]+ [LSOFilt]+\
                  [LSetCurrent]+ [LMotorCurrent]+ [LMotorVoltage]+ [LAnkleAngle]+\
                  [LMotorAngle]+\
                  [LLG]+ [LMG]+ [LAX]+ [LAY]+[LAZ]+ [LGX]+ [LGY]+ [LGZ]


            datalogright= [RTime]+ [RTime2]+ [RExoTime]+ [RSORaw]+ [RTARaw]+ [RSOFilt]+\
                  [RSetCurrent]+ [RMotorCurrent]+ [RMotorVoltage]+ [RAnkleAngle]+\
                  [RMotorAngle]+\
                  [RLG]+ [RMG]+ [RAX]+[RAY]+[RAZ]+ [RGX]+ [RGY]+ [RGZ]+ [sync_sig]

            #combines left and right data
            datalogall= datalogleft + datalogright
            #appends thenew data
            dall.appendData(datalogall)

            if graphChange==1:
                input_prefix='R'
                in1=RSORaw
                in2=RSetCurrent
                in3=RMotorCurrent
                in4=RTAFilt
            elif graphChange==2:
                input_prefix='L'
                in1=LSORaw
                in2=LSetCurrent
                in3=LMotorCurrent
                in4=LTAFilt
            elif graphChange==3:
                input_prefix='random'
                in1=sync_sig
                in2=LTARaw
                in3=RTARaw
                in4=RSOFilt

            #Send data to the GUI and Save it to File
            currentTime = time()
            elapsed_time = currentTime-start_Time

            sender.graph(elapsed_time, in1 , input_prefix+'Soleus','', in2 , 'Set Current', '', in3, input_prefix+'Motor Current','', in4, input_prefix+'TA Filt','')
            #maindata_conn.send([exoTargetCurrent]) #the data needs to be as an array
            #f.write(f"{currentTime:.6f},{LemgRaw[0]},{LemgRaw[1]},{LfiltVals[0]},{LfiltVals[1]},{RemgRaw[0]},{RemgRaw[1]},{RfiltVals[0]},{RfiltVals[1]},{LexoTargetCurrent},{LexoState.mot_cur},{LexoState.mot_volt},{LexoState.ank_ang},{LexoState.mot_ang},{RexoTargetCurrent},{RexoState.mot_cur},{RexoState.mot_volt},{RexoState.ank_ang},{RexoState.mot_ang}, {LemgRaw[2]},{LemgRaw[3]},{RemgRaw[2]},{RemgRaw[3]}, {LexoState.accelx}, {LexoState.accely}, {LexoState.accelz}, {LexoState.gyrox}, {LexoState.gyroy}, {LexoState.gyroz}, {RexoState.accelx}, {RexoState.accely}, {RexoState.accelz}, {RexoState.gyrox}, {RexoState.gyroy}, {RexoState.gyroz},{sync_sig}\n")

    except Exception as e:
        #if things close incorrectly, terminate the processes correctly
        print('ran into exception')
        dall.writeOut()
        GPIO.cleanup()
        dephy.send_motor_command(Lexo, fxe.FX_NONE, 0) #set exo to no current
        dephy.stop_streaming(Lexo)
        sleep(0.1)
        dephy.close(Lexo)
        dephy.send_motor_command(Rexo, fxe.FX_NONE, 0) #set exo to no current
        dephy.stop_streaming(Rexo)
        sleep(0.1)
        dephy.close(Rexo)
        print(e)

        #print('\nStopped via keyboard interrupt\n')
if __name__=="__main__":
    main()
