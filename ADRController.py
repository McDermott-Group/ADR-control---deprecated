SIM922_SLOT = 5 #Diode Temp Monitor
SIM921_SLOT = 1 #AC Resistance Bridge
SIM925_SLOT = 6 #multiplexer

MAGNET_VOLTAGE_LIMIT = 0.05     #Back EMF limit in Volts
MAG_UP_dV = 0.003               #[V/step] How much do we increase the voltage by every second when maggin up? HPD Manual uses 10mV=0.01V, 2.5V/30min=1.4mV/s ==> Let's use a middle rate of 3mV/step. (1 step is about 1s)
CURRENT_LIMIT = 9               #Max Current in Amps
VOLTAGE_LIMIT = 3               #Maxx Voltage in Volts.  At 9A, we usually get about 2.5-2.7V, so this shouldn't need to be more than 3
MAGNET_VOLTAGE_GAIN = 0.101       #Regulate loop uses a PID-esque process.  This is the gain constant.
dVdT_LIMIT = 0.008              #Keep dV/dt to under this value [V/s]
dIdt_MAGUP_LIMIT = 9./(30*60)   #limit on the rate at which we allow current to increase in amps/s (we want 9A over 30 min)
dIdt_REGULATE_LIMIT = 9./(40*60)#limit on the rate at which we allow current to change in amps/s (we want 9A over 40 min)

STEP_LENGTH = 1000              #How long is each regulation/mag up cycle in ms.  **Never set this less than 1000ms.**  The SRS SIM922 only measures once a second and this would cause runaway voltages/currents.

import visa, pyvisa
import matplotlib as mpl
mpl.use('TkAgg')
import numpy as np
import pylab, numpy
import time, datetime
import threading
import Tkinter
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg

"""Inherited from the Tkinter Entry widget, this just turns red when a limit is reached"""
class EntryWithAlert(Tkinter.Entry):
    def __init__(self, *args, **kwargs):
        self.limit = kwargs.pop('limit',False)
        self.variable = kwargs['textvariable']
        self.variable.trace('w',self.callback)
        Tkinter.Entry.__init__(self,*args,**kwargs)
        self.naturalBGColor = self.cget('disabledbackground')
    def callback(self,*dummy):
        if self.limit != False:
            x = self.variable.get()
            if x == '' or float(x) > float(self.limit):
                self.configure(disabledbackground='red')
            else:
                self.configure(disabledbackground=self.naturalBGColor)

"""Searches through available devices with *IDN? and returns device with name that includes deviceName."""
def getGPIB(deviceName):
    try: # Python 2.7
        rm = visa.ResourceManager()
        for address in rm.list_resources():
            if address[:4]=='GPIB':
                instrument = rm.get_instrument(address)
                if instrument.ask('*IDN?').lower().find(deviceName.lower()) >= 0:
                    return instrument
    except: pass
    try: #Python 2.6
        for address in visa.get_instruments_list():
            if address[:4]=='GPIB':
                instrument = visa.instrument(address)
                if instrument.ask('*IDN?').lower().find(deviceName.lower()) >= 0:
                    return instrument
    except: pass
    message = 'Cannot find resource with search string "'+deviceName+'".  Make sure to turn all instruments on, and if they already are, try turning them on and off again. \n'
    log.configure(state=Tkinter.NORMAL)
    log.insert(1.0,message)
    log.tag_add("redAlert", '1.0', '1.end')
    log.configure(state=Tkinter.DISABLED)
    return None

""" The SIM922 is a module that fits into the SIM900 mainframe, and in our setup, is used to measure
    both the temperature diodes (chan 1,2) and the voltage across the magnet (chan 3,4)."""
class SIM922:
    def __init__(self):
        #The SIM900 is the mainframe rack into which all the other modules fit (ex: sim922) and commands must go through it
        self.SIM900 = getGPIB('SIM900')
    def getDiodeTemperatures(self):
        self.SIM900.write("CONN %d,'xyz'" % SIM922_SLOT) # "xyz" is just an exit code to rever commands to go back to the SIM900
        self.SIM900.write("*CLS")
        diodeMonitorReturnString = self.SIM900.ask("TVAL? 0").strip('\x00')
        self.SIM900.write("xyz")
        temperatures = [float(x) for x in diodeMonitorReturnString.split(',')][:2]
        return temperatures
    def getMagnetVoltage(self):
        self.SIM900.write("CONN %d,'xyz'" % SIM922_SLOT)
        self.SIM900.write("*CLS")
        diodeMonitorReturnString = self.SIM900.ask("VOLT? 0").strip('\x00')
        self.SIM900.write("xyz")
        magnetVoltages = [float(x) for x in diodeMonitorReturnString.split(',')][2:]
        return (abs(magnetVoltages[0])+abs(magnetVoltages[1]))/2
""" This class implements both the SIM921 AC Resistance Bridge for measuring the RuOx temperature sensors,
    and the SIM925 Multiplexor to select which channel (read: RuOx detector) to read. The multiplexer clicks
    a lot if you want it to switch back and forth rapidly so as to measure the temperature on the AC bridge
    for both RuOx thermometers.  Therefore this is set to ignore the 1K stage.  If you want to measure both,
    change the False to True in the if(False) below """
class RuOxTemperatureMonitor:
    def __init__(self):
        self.SIM900 = getGPIB('SIM900')
    def getTemperatures(self):
        temperatures = []
        if(True):
            self.SIM900.write("CONN %d,'xyz'" %SIM925_SLOT)
            self.SIM900.write("CHAN 1")
            self.SIM900.write("xyz")
            self.SIM900.write("CONN %d,'xyz'" %SIM921_SLOT)
            self.SIM900.write("CURV 1")
            temperatures.append( float(self.SIM900.ask("TVAL?").strip('\x00')) )
            self.SIM900.write("xyz")
            self.SIM900.write("CONN %d,'xyz'" %SIM925_SLOT)
            self.SIM900.write("CHAN 2")
            self.SIM900.write("xyz")
        self.SIM900.write("CONN %d,'xyz'" %SIM921_SLOT)
        self.SIM900.write("CURV 2")
        temperatures.append( float(self.SIM900.ask("TVAL?").strip('\x00')) )
        self.SIM900.write("xyz")
        return temperatures
""" This class should be able to manage all controls for the power supply.  Setting Voltage and Current works
    just as it would if you were to turn the knobs on the power supply itself.  getObsReg() returns the operating
    mode of the power supply, ex: Current Control (CC). """
class PowerSupply:
    def __init__(self):
        self.instrument = getGPIB('6641A')
    def instrumentIsConnected(self):
        if self.instrument == None:
            return False
        else: return True
    def getCurrent(self):
        return float(self.instrument.ask('MEAS:CURR?').strip('\x00'))
    def getVoltage(self):
        return float(self.instrument.ask('MEAS:VOLT?').strip('\x00'))
    def setCurrent(self,I):
        self.instrument.write('CURR '+'{0:5.6f}'.format(I))
    def setVoltage(self,V):
        self.instrument.write('VOLT '+'{0:5.6f}'.format(V))
    def setOutputOn(self):
        print 'Turning Power Supply Output on.'
        self.instrument.write('OUTP ON')
    def reset(self):
        self.instrument.write('*RST')
    def getOpsReg(self):
        opCode = int(self.instrument.ask('STAT:OPER:COND?').strip('\x00'))
        codeList = [int(x) for x in "{0:016b}".format(opCode)]
        codeList.reverse()
        CAL = bool(codeList[0])
        WTG = bool(codeList[5])
        CV = bool(codeList[8])
        CC = bool(codeList[10])
        if CV: return 'CV Mode'
        elif CC: return 'CC Mode'
        else: return 'OutputOff'
    def initiate(self):
        if self.getCurrent()-0.01 >= CURRENT_LIMIT:
            message = 'Current too high!  Manually lower before trying to run again.  Please quit now. \n'
            log.configure(state=Tkinter.NORMAL)
            log.insert(1.0,message)
            log.tag_add("redAlert", '1.0', '1.end')
            log.configure(state=Tkinter.DISABLED)
        else:
            state = self.getOpsReg()
            if state == 'OutputOff':
                message = 'Output Off.  Setting Current to '+str(CURRENT_LIMIT)+' Amps and voltage to 0 Volts. \n'
                log.configure(state=Tkinter.NORMAL)
                log.insert(1.0,message)
                log.configure(state=Tkinter.DISABLED)
                self.reset()
                self.setCurrent(CURRENT_LIMIT)
                self.setVoltage(0)
                self.setOutputOn()
            elif state == 'CV Mode':
                message = 'Starting in CV Mode.  Setting Current to '+str(CURRENT_LIMIT)+' Amps. \n'
                log.configure(state=Tkinter.NORMAL)
                log.insert(1.0,message)
                log.configure(state=Tkinter.DISABLED)
                self.setCurrent( CURRENT_LIMIT )
            elif state == 'CC Mode':
                V_now = self.getVoltage()
                message = 'Starting in CC Mode.  Setting Current to '+str(CURRENT_LIMIT)+' Amps and voltage to '+str(V_now)+' Volts. \n'
                log.configure(state=Tkinter.NORMAL)
                log.insert(1.0,message)
                log.configure(state=Tkinter.DISABLED)
                V_now = self.getVoltage()
                self.setVoltage( V_now )
                self.setCurrent( CURRENT_LIMIT )

""" The magging up method, as per the HPD Manual, involves increasing the voltage in steps of MAG_UP_dV volts
    every cycle of the loop.  This cycle happens once every STEP_LENGTH seconds, nominally 1s (since the voltage
    monitor reads once a second).  Each cycle, the voltage across the magnet is read to get the backEMF.  If it
    is greater than the MAGNET_VOLTAGE_LIMIT, the voltage will not be raised until the next cycle for which the
    backEMF < MAGNET_VOLTAGE_LIMIT. Called when Mag Up button is pressed. """
def magUp():
    message = 'Beginning to mag up to '+str(CURRENT_LIMIT)+' Amps. \n'
    log.configure(state=Tkinter.NORMAL)
    log.insert(1.0,message)
    log.configure(state=Tkinter.DISABLED)
    sim922 = SIM922()
    ps = PowerSupply()
    if ps.instrumentIsConnected() == False:
        message = 'Power Supply not connected.  Please turn it on and restart program. \n'
        log.configure(state=Tkinter.NORMAL)
        log.insert(1.0,message)
        log.tag_add("redAlert", '1.0', '1.end')
        log.configure(state=Tkinter.DISABLED)
        return
    class local:
        _job = ''
        lastI = ps.getCurrent()
        lastTime = time.time()
    def cancelMagUp():
        if local._job is not None:
            root.after_cancel(local._job)
            local._job = None
        magUpButton.configure(text='Mag Up', command=magUp)
        regulateButton.configure(state=Tkinter.NORMAL)
        message = 'Magging up stopped at a current of '+str(ps.getCurrent())+' Amps. \n'
        log.configure(state=Tkinter.NORMAL)
        log.insert(1.0,message)
        log.configure(state=Tkinter.DISABLED)
    def increaseV():
        I_now = ps.getCurrent()
        dI = I_now-local.lastI
        dt = time.time() - local.lastTime
        if dt == 0: dt = 0.0000000001 #to prevent divide by zero error
        local.lastI = I_now
        local.lastTime = time.time()
        if I_now < CURRENT_LIMIT:
            if sim922.getMagnetVoltage() < MAGNET_VOLTAGE_LIMIT and abs(dI/dt) < dIdt_MAGUP_LIMIT:
                newVoltage = ps.getVoltage() + MAG_UP_dV
                if newVoltage < VOLTAGE_LIMIT:
                    ps.setVoltage(newVoltage)
                else: ps.setVoltage(VOLTAGE_MAX)
                #newCurrent = ps.getCurrent() + 0.005
                #ps.setCurrent(newCurrent)
            local._job = root.after(STEP_LENGTH, increaseV)
        else:
            magUpButton.configure(text='Mag Up', command=magUp)
            regulateButton.configure(state=Tkinter.NORMAL)
            message = 'Finished magging up. '+str(ps.getCurrent())+' Amps reached. \n'
            log.configure(state=Tkinter.NORMAL)
            log.insert(1.0,message)
            log.configure(state=Tkinter.DISABLED)
    magUpButton.configure(text='Stop Magging Up', command=cancelMagUp)
    regulateButton.configure(state=Tkinter.DISABLED)
    increaseV()

""" This function is almost equivalent to the old code that Steve Sendelbach had implemented in LabVIEW.  It is
    based on a PID controller.  I also added a VOLTAGE_LIMIT case.  The basics of it is that a new voltage V+dV is
    proposed.  dV is then limited as necessary, and the new voltage is set. As with magging up, regulate runs a cycle
    at approximately once per second (plus processing time). Called when regulate button is pressed. """
def regulate():
    T_target = float(regulateTempField.get())
    sim922 = SIM922()
    ruox = RuOxTemperatureMonitor()
    ps = PowerSupply()
    if ps.instrumentIsConnected() == False:
        message = 'Power Supply not connected.  Please turn it on and restart program. \n'
        log.configure(state=Tkinter.NORMAL)
        log.insert(1.0,message)
        log.tag_add("redAlert", '1.0', '1.end')
        log.configure(state=Tkinter.DISABLED)
        return
    message = 'Starting regulation cycle from '+str(ps.getCurrent())+' Amps. \n'
    log.configure(state=Tkinter.NORMAL)
    log.insert(1.0,message)
    log.configure(state=Tkinter.DISABLED)
    print 'beginning regulation'
    print 'V\tbackEMF\tdV/dT\tdV'
    class local:
        _job = ''
        lastTime = time.time()
        lastMagnetV = 0
        lastI = ps.getCurrent()
    def cancelRegulate():
        if local._job is not None:
            root.after_cancel(local._job)
            local._job = None
        regulateButton.configure(text='Regulate', command=regulate)
        magUpButton.configure(state=Tkinter.NORMAL)
        message = 'Regulation stopped at a current of '+str(ps.getCurrent())+' Amps. \n'
        log.configure(state=Tkinter.NORMAL)
        log.insert(1.0,message)
        log.configure(state=Tkinter.DISABLED)
    def oneRegCycle():
        backEMF = sim922.getMagnetVoltage()
        V_now = ps.getVoltage()
        I_now = ps.getCurrent()
        dI = I_now - local.lastI
        local.lastI = I_now
        print str(V_now)+'\t'+str(backEMF)+'\t',
        #propose new voltage
        dV = T_target-ruox.getTemperatures()[0]-backEMF*MAGNET_VOLTAGE_GAIN
        dT = time.time()-local.lastTime
        if dT == 0: dT = 0.0000000001 #to prevent divide by zero error
        local.lastTime = time.time()
        #hard current limit
        if ps.getCurrent() > CURRENT_LIMIT:
            if dV>0: dV=0
        #hard voltage limit
        if V_now + dV > VOLTAGE_LIMIT:
            dV = VOLTAGE_LIMIT - V_now
        # steady state limit
        if local.lastMagnetV == backEMF:
            if V_now < 0.6 or abs(dV) < 0.001:
                dV = min(max(dV,-MAGNET_VOLTAGE_LIMIT-backEMF),MAGNET_VOLTAGE_LIMIT-backEMF)
            else: dV = 0
        else: dV = min(max(dV,-MAGNET_VOLTAGE_LIMIT-backEMF),MAGNET_VOLTAGE_LIMIT-backEMF)
        lastMagnetV = backEMF
        # limit by hard voltage increase limit
        if abs(dV/dT) > dVdT_LIMIT:
            print str(dV/dT)+'\t',
            dV = dVdT_LIMIT*dT*(dV/abs(dV))
        # limit by hard current increase limit
        if abs(dI/dT) > dIdt_REGULATE_LIMIT:
            dV = 0
        # will voltage go negative?
        runCycleAgain = True
        if V_now+dV <= 0:
            ps.setVoltage(0)
            V_now = 0
            dV = 0
            runCycleAgain = False
        print str(dV)
        ps.setVoltage(V_now + dV)
        if runCycleAgain: local._job = root.after(STEP_LENGTH, oneRegCycle)
        else:
            regulateButton.configure(text='Regulate', command=regulate)
            magUpButton.configure(state=Tkinter.NORMAL)
            message = 'Regulation has completed.  Mag up and try again. \n'
            log.configure(state=Tkinter.NORMAL)
            log.insert(1.0,message)
            log.configure(state=Tkinter.DISABLED)
    regulateButton.configure(text='Stop Regulating', command=cancelRegulate)
    magUpButton.configure(state=Tkinter.DISABLED)
    oneRegCycle()


""" This takes care of the real time temperature plotting. It starts immediately upon starting the program, and never stops. """
tempHistory = []
def tempPlot():
    class local:
        i = 0
    sim922 = SIM922()
    ruox = RuOxTemperatureMonitor()
    ps = PowerSupply()
    if ps.instrumentIsConnected() == False:
        message = 'Power Supply not connected.  Please turn it on and restart program. \n'
        log.configure(state=Tkinter.NORMAL)
        log.insert(1.0,message)
        log.tag_add("redAlert", '1.0', '1.end')
        log.configure(state=Tkinter.DISABLED)
    ax = fig.add_subplot(111)
    ax2 = ax.twinx()
    ax.set_title('Realtime Temperature Readout')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Temparture [K]')
    stage60K, = ax.plot([],[])
    stage03K, = ax.plot([],[])
    stage50mK, = ax.plot([],[])
    def oneTempCycle():
        global currentBackEMF, currentI, currentV
        #update current and voltage data
        currentBackEMF.set( "{0:.3f}".format(sim922.getMagnetVoltage()) )
        if ps.instrumentIsConnected() == True:
            currentI.set( "{0:.3f}".format(ps.getCurrent()) )
            currentV.set( "{0:.3f}".format(ps.getVoltage()) )
        else:
            currentI.set('')
            currentV.set('')
        #append temp data
        newTemps = sim922.getDiodeTemperatures()
        newTemps.append(ruox.getTemperatures()[0])
        tempHistory.append(newTemps)
        stage60K.set_xdata(numpy.append(stage60K.get_xdata(),local.i))
        stage60K.set_ydata(numpy.append(stage60K.get_ydata(),tempHistory[-1][0]))
        stage03K.set_xdata(numpy.append(stage03K.get_xdata(),local.i))
        stage03K.set_ydata(numpy.append(stage03K.get_ydata(),tempHistory[-1][1]))
        stage50mK.set_xdata(numpy.append(stage50mK.get_xdata(),local.i))
        stage50mK.set_ydata(numpy.append(stage50mK.get_ydata(),tempHistory[-1][2]))
        #rescale axes, with the x being scaled by the slider
        try: ax.lines.remove(stage60K) #we dont actually want to see the 60K stage (it zooms out too much)
        except: pass
        ax.relim()
        xMin = local.i-wScale.get()
        if wScale.get() == 600: xMin = 0
        if len(tempHistory)>1: ax.set_xlim(xMin,local.i)
        ax.autoscale_view(scalex=False)
        #add labels with temps on the right
        labelLevels = newTemps
        for k in range(len(newTemps)):
            for j in range(k+1,len(newTemps)):
                minLabelSpread = (ax.get_ylim()[1]-ax.get_ylim()[0])/50
                if abs(newTemps[k]-newTemps[j])<minLabelSpread:
                    avgTemp = (newTemps[k]+newTemps[j])/2
                    labelLevels[k] = avgTemp + minLabelSpread/2*(newTemps[k]-avgTemp)/(abs(newTemps[k]-avgTemp)+0.0001) # + .0001 to prevent divide by 0 error
                    labelLevels[j] = avgTemp + minLabelSpread/2*(newTemps[j]-avgTemp)/(abs(newTemps[j]-avgTemp)+0.0001) # + .0001 to prevent divide by 0 error
        labels = ['60K','3K','RuOx']
        labels = [labels[l]+' ['+str(newTemps[l])+'K]' for l in range(len(labels))]
        pylab.yticks(labelLevels,labels)
        colorScheme = ['blue','green','red']
        for n in range(len(newTemps)):
            l = ax2.yaxis.get_ticklabels()[n]
            l.set_color(colorScheme[n])
        ax2.set_ylim(ax.get_ylim())
        canvas.draw()
        local.i+=1
        root.after(STEP_LENGTH,oneTempCycle)
    oneTempCycle()

""" called when the window is closed. Before quitting, we save the logfile to log.txt, and the temperatures to temperatures.txt"""
def _quit():
    filePath = 'Z:\\mcdermott-group\\ADR_log_files\\NEW_ADR'
    dt = datetime.datetime.now()
    dateAppend = dt.strftime("_%y%m%d_%I%M")
    try:
        np.savetxt(filePath+'\\temperatures'+dateAppend+'.txt',tempHistory,delimiter='\t')
        f = open(filePath+'\\log'+dateAppend+'.txt', 'w')
        f.write( log.get("1.0",Tkinter.END) )
        f.close()
    except Exception, e:
        print e
    root.quit()     # stops mainloop
    root.destroy()  # this is necessary on Windows to prevent
                    # Fatal Python Error: PyEval_RestoreThread: NULL tstate

root = Tkinter.Tk()
root.wm_title('ADR Magnet Controller')

#error/message box log
log = Tkinter.Text(master=root, height=5)
log.pack(side=Tkinter.TOP, fill=Tkinter.X)
log.tag_config("redAlert", background="red")
log.configure(state=Tkinter.DISABLED)

# temp plot
fig = pylab.figure()
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.show()
canvas.get_tk_widget().pack(side=Tkinter.TOP, fill=Tkinter.BOTH, expand=1)

#temp plot toolbar at bottom
toolbar = NavigationToolbar2TkAgg( canvas, root )
toolbar.update()
canvas._tkcanvas.pack(side=Tkinter.TOP, fill=Tkinter.BOTH, expand=1)

#scale to adjust time shown in temp plot
wScale = Tkinter.Scale(master=root,label="Time Displayed [s]", from_=1, to=600,sliderlength=30,length=500, orient=Tkinter.HORIZONTAL)
wScale.set(700)
wScale.pack(side=Tkinter.TOP)

magControlsFrame = Tkinter.Frame(root)
magControlsFrame.pack(side=Tkinter.TOP)

#mag up button
magUpButton = Tkinter.Button(master=magControlsFrame, text='Mag Up', command=magUp)
magUpButton.pack(side=Tkinter.LEFT)

#regulate button and temp field
regulateButton = Tkinter.Button(master=magControlsFrame, text='Regulate', command=regulate)
regulateButton.pack(side=Tkinter.LEFT)
Tkinter.Label(magControlsFrame, text=" at ").pack(side=Tkinter.LEFT)
regulateTempField = Tkinter.Entry(magControlsFrame)
regulateTempField.pack(side=Tkinter.LEFT)
regulateTempField.insert(0, "0.1")
Tkinter.Label(magControlsFrame, text="K").pack(side=Tkinter.LEFT)

monitorFrame = Tkinter.Frame(root)
monitorFrame.pack(side=Tkinter.TOP)

#shows current values for backEMF, current, voltage
currentBackEMF = Tkinter.StringVar() #current as in now, not as in amps
currentI = Tkinter.StringVar()
currentV = Tkinter.StringVar()
Tkinter.Label(monitorFrame, text="Back EMF = ").pack(side=Tkinter.LEFT)
backEMFField = EntryWithAlert(monitorFrame, textvariable=currentBackEMF, state=Tkinter.DISABLED, limit=MAGNET_VOLTAGE_LIMIT)
backEMFField.pack(side=Tkinter.LEFT)
Tkinter.Label(monitorFrame, text="(V)   I = ").pack(side=Tkinter.LEFT)
currentIField = EntryWithAlert(monitorFrame, textvariable=currentI, state=Tkinter.DISABLED, limit=CURRENT_LIMIT)
currentIField.pack(side=Tkinter.LEFT)
Tkinter.Label(monitorFrame, text="(A)   V = ").pack(side=Tkinter.LEFT)
currentVField = EntryWithAlert(monitorFrame, textvariable=currentV, state=Tkinter.DISABLED, limit=VOLTAGE_LIMIT)
currentVField.pack(side=Tkinter.LEFT)
Tkinter.Label(monitorFrame, text="(V)").pack(side=Tkinter.LEFT)

#start program
root.protocol("WM_DELETE_WINDOW", _quit)
root.after(100,tempPlot)
ps = PowerSupply()
if ps.instrumentIsConnected() == False:
    message = 'Power Supply not connected.  Please turn it on and restart program. \n'
    log.configure(state=Tkinter.NORMAL)
    log.insert(1.0,message)
    log.tag_add("redAlert", '1.0', '1.end')
    log.configure(state=Tkinter.DISABLED)
else:
    ps.initiate()
Tkinter.mainloop()
