"""
New in this version:
- new classes for Lakeshore 218 temp monitor, so we can use this program with the old ADRs.
- added support for new classes above (not all classes fully developed yet). Change at bottom.
- Changed the time scale to be in minutes instead of seconds, and extended it.
- changed the way the toolbar at the bottom works, corrected the way it zooms (nan wasn't working with max/min)
"""

SIM922_SLOT = 5 #Diode Temp Monitor
SIM921_SLOT = 1 #AC Resistance Bridge
SIM925_SLOT = 6 #multiplexer

MAGNET_VOLTAGE_LIMIT = 0.1      #Back EMF limit in Volts
MAG_UP_dV = 0.003               #[V/step] How much do we increase the voltage by every second when maggin up? HPD Manual uses 10mV=0.01V, 2.5V/30min=1.4mV/s ==> Let's use a middle rate of 3mV/step. (1 step is about 1s)
CURRENT_LIMIT = 9               #Max Current in Amps
VOLTAGE_LIMIT = 2               #Maxx Voltage in Volts.  At 9A, we usually get about 2.5-2.7V or 1.69V (with or without the external diode protection box), so this shouldn't need to be more than 3 or 2
PID_KD = 0.07 					#Regulate loop uses a PID-esque process.  This is the gain constant based on the magnet backEMF (derivative part).
PID_KP = 1						#proportional part of PID controller
PID_KI = 0						#not currently used, or even implemented
dVdT_LIMIT = 0.008              #Keep dV/dt to under this value [V/s]
dIdt_MAGUP_LIMIT = 9./(30*60)   #limit on the rate at which we allow current to increase in amps/s (we want 9A over 30 min)
dIdt_REGULATE_LIMIT = 9./(40*60)#limit on the rate at which we allow current to change in amps/s (we want 9A over 40 min)

STEP_LENGTH = 1000              #How long is each regulation/mag up cycle in ms.  **Never set this less than 1000ms.**  The SRS SIM922 only measures once a second and this would cause runaway voltages/currents.
FILE_PATH = 'C:\\Users\\McDermott\\Desktop\\ADR Magnet Controller\\Development\\Python ADRController\\test_temp_log'#'Z:\\mcdermott-group\\ADR_log_files\\NEW_ADR'

import visa, pyvisa
import matplotlib as mpl
mpl.use('TkAgg')
import pylab, numpy
import time, datetime
import Tkinter
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from operator import itemgetter

class GPIBError(Exception):
     def __init__(self, value):
         self.value = value
     def __str__(self):
         return 'Could not create device from string "' + str(self.value) + '"... Make sure all instruments are powered on and try again.'

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
    raise GPIBError(deviceName)
    message = 'Cannot find resource with search string "'+deviceName+'".  Make sure to turn all instruments on, and if they already are, try turning them on and off again. \n'
    log.log(message, alert=True)
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

""" This class implements the Lakeshore 218 temperature monitor for measuring the Diode Thermometers
    in the old ADRs."""
class LakeshoreTemperatureMonitor:
    def __init__(self):
        #The SIM900 is the mainframe rack into which all the other modules fit (ex: sim922) and commands must go through it
        self.lakeshore = getGPIB('Lakeshore')
    def getDiodeTemperatures(self):
        self.lakeshore.write("*CLS")
        diodeMonitorReturnString = self.lakeshore.ask("KRDG? 0").strip('\x00')
        temperatures = [float(x) for x in diodeMonitorReturnString.split(',')][6:8]
        return temperatures
		
""" This class implements both the SIM921 AC Resistance Bridge for measuring the RuOx temperature sensors,
    and the SIM925 Multiplexor to select which channel (read: RuOx detector) to read. The multiplexer clicks
    a lot if you want it to switch back and forth rapidly so as to measure the temperature on the AC bridge
    for both RuOx thermometers.  Therefore this is set to ignore the 1K stage.  If you want to measure both,
    change the False to True in the if(False) below """
class RuOxTemperatureMonitor:
    def __init__(self):
        self.SIM900 = getGPIB('SIM900')
        self.channel = 0 #channel 2 is the FAA pill.  GGG pill is chan 1
        self.lastTime = time.time()
        self.setChannel(2)
    def getTimeConstant(self): #returns time constant in ms
        timeConstCodes = {'-1':'filter off', '0':0.3, '1':1, '2':3, '3':10, '4':30, '5':100, '6':300}
        self.SIM900.write("CONN %d,'xyz'" %SIM921_SLOT)
        returnCode = self.SIM900.ask("TCON?").strip('\x00')
        t = timeConstCodes[returnCode]
        self.SIM900.write("xyz")
        return t
    def getTimeSinceChannelSet(self): #returns the time since the channel was changed in ms
        return time.time() - self.lastTime
    def getChannel(self):
        return self.channel
    def setChannel(self, channel):
        if self.channel != channel:
            self.channel = channel
            #set channel on multiplexer
            self.SIM900.write("CONN %d,'xyz'" %SIM925_SLOT)
            self.SIM900.write("CHAN %d" %channel)
            self.SIM900.write("xyz")
            #set curve on AC resistance bridge
            self.SIM900.write("CONN %d,'xyz'" %SIM921_SLOT)
            self.SIM900.write("CURV %d" %channel)
            self.SIM900.write("xyz")
            #set lastTime
            self.lastTime = time.time()
    def getTemperature(self):
        self.SIM900.write("CONN %d,'xyz'" %SIM921_SLOT)
        gpibstring = self.SIM900.ask("TVAL?")
        T = float(gpibstring.strip('\x00')) 
        self.SIM900.write("xyz")
        return T
		
""" This class should be able to manage all controls for the power supply.  Setting Voltage and Current works
    just as it would if you were to turn the knobs on the power supply itself.  getObsReg() returns the operating
    mode of the power supply, ex: Current Control (CC). """
class PowerSupply:
    def __init__(self,log):
        self.log = log
        self.initError = None
        try: self.instrument = getGPIB('6641A')
        except GPIBError as e:
             self.initError = str(e)
             self.instrument = None
    def instrumentIsConnected(self):
        if self.instrument == None:
            return (False, self.initError)
        else: return (True, self.initError)
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
            message = 'Current too high! Manually lower before trying to run again. Please quit now.\n'
            self.log.log(message, alert=True)
        else:
            state = self.getOpsReg()
            if state == 'OutputOff':
                message = 'Output Off. Setting Current to '+str(CURRENT_LIMIT)+' Amps and voltage to 0 Volts.\n'
                self.log.log(message)
                self.reset()
                self.setCurrent(CURRENT_LIMIT)
                self.setVoltage(0)
                self.setOutputOn()
            elif state == 'CV Mode':
                message = 'Starting in CV Mode. Setting Current to '+str(CURRENT_LIMIT)+' Amps.\n'
                self.log.log(message)
                self.setCurrent( CURRENT_LIMIT )
            elif state == 'CC Mode':
                V_now = self.getVoltage()
                message = 'Starting in CC Mode. Setting Current to '+str(CURRENT_LIMIT)+' Amps and voltage to '+str(V_now)+' Volts.\n'
                self.log.log(message)
                V_now = self.getVoltage()
                self.setVoltage( V_now )
                self.setCurrent( CURRENT_LIMIT )

"""Inherited from the Tkinter Entry widget, this just turns red when a limit is reached"""
class EntryWithAlert(Tkinter.Entry):
    def __init__(self, *args, **kwargs):
        self.upper_limit = kwargs.pop('upper_limit',False)
        self.lower_limit = kwargs.pop('lower_limit',False)
        self.variable = kwargs['textvariable']
        self.variable.trace('w',self.callback)
        Tkinter.Entry.__init__(self,*args,**kwargs)
        self.naturalBGColor = self.cget('disabledbackground')
    def callback(self,*dummy):
        if self.upper_limit != False or self.lower_limit != False:
            x = self.variable.get()
            if x == '' or float(x) > float(self.upper_limit) or float(x) < float(self.lower_limit):
                self.configure(disabledbackground='red')
            else:
                self.configure(disabledbackground=self.naturalBGColor)

"""This class inherits a Tkinter Text widget to make a simple log box.  It will log an entry,
and set the color to red if alert is set to True.  A time stamp is automatically added."""
class LogBox(Tkinter.Text):
    def __init__(self, *args, **kwargs):
        self.dateAppend = kwargs.pop('date_append','')
        Tkinter.Text.__init__(self,*args,**kwargs)
        self.tag_config("redAlert", background="red")
        self.configure(state=Tkinter.DISABLED)
    def log(self, message, alert=False):
        dt = datetime.datetime.now()
        messageWithTimeStamp = dt.strftime("[%m/%d/%y %H:%M:%S] ") + message
        self.configure(state=Tkinter.NORMAL)
        self.insert(1.0,messageWithTimeStamp)
        if alert: self.tag_add("redAlert", '1.0', '1.end')
        self.configure(state=Tkinter.DISABLED)
        with open(FILE_PATH+'\\log'+self.dateAppend+'.txt', 'a') as f:
            f.write( messageWithTimeStamp + '\n' )

class ADRController(Tkinter.Tk):
    def __init__(self,parent, diodeTempMonitor=None, ruoxTempMonitor=None, magnetVoltageMonitor=None):
        Tkinter.Tk.__init__(self,parent)
        self.parent = parent
        self.newTemps = [numpy.NaN,numpy.NaN,numpy.NaN,numpy.NaN]
        self.timeStamps = numpy.array([])
        # vars used during each measurement cycle
        self.isRegulating = False
        self.isMaggingUp = False
        self.cycle = 0
        self.GGGTemp = numpy.nan
        self.FAATemp = numpy.nan
        self.ruoxChan = 'FAA'
        self.startTime = time.time()
        dt = datetime.datetime.now()
        self.dateAppend = dt.strftime("_%y%m%d_%H%M")
        #initialize and start measurement loop
        self.initializeWindow()
        self.initializeInstruments(diodeTempMonitor, ruoxTempMonitor, magnetVoltageMonitor)
        self.executeExternalCommands()
        self.after(100, self.measurementCycle)
        self.after(200, self.renewPowerSupply)
    def initializeWindow(self):
        root = self
        #set up window
        self.wm_title('ADR Magnet Controller')
        w, h = self.winfo_screenwidth(), self.winfo_screenheight()
        self.geometry("%dx%d+0+0" % (w/2, 0.9*h))
        #error/message box log
        self.log = LogBox(master=root, height=5, date_append=self.dateAppend)
        self.log.pack(side=Tkinter.TOP, fill=Tkinter.X)
        # temp plot
        fig = pylab.figure()
        self.ax = fig.add_subplot(111)
        #self.ax2 = self.ax.twinx()
        self.ax.set_title('Realtime Temperature Readout\n\n\n')
        self.ax.set_xlabel('Time [s]')
        self.ax.set_ylabel('Temparture [K]')
        self.stage60K, = self.ax.plot([],[])
        self.stage03K, = self.ax.plot([],[])
        self.stageGGG, = self.ax.plot([],[])
        self.stageFAA, = self.ax.plot([],[])
        self.canvas = FigureCanvasTkAgg(fig, master=root)
        self.canvas.show()
        self.canvas.get_tk_widget().pack(side=Tkinter.TOP, fill=Tkinter.BOTH, expand=1)
        #temp plot toolbar at bottom
        self.toolbar = NavigationToolbar2TkAgg( self.canvas, root )
        self.toolbar.update()
        #self.toolbar.pack(side=Tkinter.BOTTOM, fill=Tkinter.X)
        self.canvas._tkcanvas.pack(side=Tkinter.TOP, fill=Tkinter.BOTH, expand=1)
        #which temp plots should I show? (checkboxes)
        tempSelectFrame = Tkinter.Frame(root)
        tempSelectFrame.pack(side=Tkinter.TOP)
        self.t60K = Tkinter.IntVar()
        self.t3K = Tkinter.IntVar()
        self.tGGG = Tkinter.IntVar()
        self.tFAA = Tkinter.IntVar()
        self.t60K.set(0)
        self.t3K.set(1)
        self.tGGG.set(0)
        self.tFAA.set(1)
        t1checkbox = Tkinter.Checkbutton(tempSelectFrame, text = '60K Stage', variable=self.t60K, fg='blue')
        t1checkbox.pack(side=Tkinter.LEFT)
        t2checkbox = Tkinter.Checkbutton(tempSelectFrame, text = '3K Stage', variable=self.t3K, fg='forest green')
        t2checkbox.pack(side=Tkinter.LEFT)
        t3checkbox = Tkinter.Checkbutton(tempSelectFrame, text = '1K Stage (GGG)', variable=self.tGGG, fg='red')
        t3checkbox.pack(side=Tkinter.LEFT)
        t4checkbox = Tkinter.Checkbutton(tempSelectFrame, text = '50mK Stage (FAA)', variable=self.tFAA, fg='dark turquoise')
        t4checkbox.pack(side=Tkinter.LEFT)
        #scale to adjust time shown in temp plot
        self.wScale = Tkinter.Scale(master=root,label="60*Cycles=Minutes Displayed (1 cycle ~ 1 sec)", from_=1, to=1440,sliderlength=30,length=500, orient=Tkinter.HORIZONTAL)
        self.wScale.set(1440)
        self.wScale.pack(side=Tkinter.TOP)
        #frame for mag up and regulate controls
        magControlsFrame = Tkinter.Frame(root)
        magControlsFrame.pack(side=Tkinter.TOP)
        #mag up button
        self.magUpButton = Tkinter.Button(master=magControlsFrame, text='Mag Up', command=self.magUp)
        self.magUpButton.pack(side=Tkinter.LEFT)
        #regulate button and temp field
        self.regulateButton = Tkinter.Button(master=magControlsFrame, text='Regulate', command=self.regulate)
        self.regulateButton.pack(side=Tkinter.LEFT)
        Tkinter.Label(magControlsFrame, text=" at ").pack(side=Tkinter.LEFT)
        self.regulateTempField = Tkinter.Entry(magControlsFrame)
        self.regulateTempField.pack(side=Tkinter.LEFT)
        self.regulateTempField.insert(0, "0.1")
        Tkinter.Label(magControlsFrame, text="K").pack(side=Tkinter.LEFT)
        #shows current values for backEMF, current, voltage
        monitorFrame = Tkinter.Frame(root)
        monitorFrame.pack(side=Tkinter.TOP)
        self.currentBackEMF = Tkinter.StringVar() #current as in now, not as in amps
        self.currentI = Tkinter.StringVar()
        self.currentV = Tkinter.StringVar()
        Tkinter.Label(monitorFrame, text="Back EMF = ").pack(side=Tkinter.LEFT)
        backEMFField = EntryWithAlert(monitorFrame, textvariable=self.currentBackEMF, state=Tkinter.DISABLED, upper_limit=MAGNET_VOLTAGE_LIMIT)
        backEMFField.pack(side=Tkinter.LEFT)
        Tkinter.Label(monitorFrame, text="(V)   I = ").pack(side=Tkinter.LEFT)
        currentIField = EntryWithAlert(monitorFrame, textvariable=self.currentI, state=Tkinter.DISABLED, upper_limit=CURRENT_LIMIT)
        currentIField.pack(side=Tkinter.LEFT)
        Tkinter.Label(monitorFrame, text="(A)   V = ").pack(side=Tkinter.LEFT)
        currentVField = EntryWithAlert(monitorFrame, textvariable=self.currentV, state=Tkinter.DISABLED, upper_limit=VOLTAGE_LIMIT)
        currentVField.pack(side=Tkinter.LEFT)
        Tkinter.Label(monitorFrame, text="(V)").pack(side=Tkinter.LEFT)
        #X BUTTON
        self.protocol("WM_DELETE_WINDOW", self._quit)
    def initializeInstruments(self, diodeTempMonitor, ruoxTempMonitor, magnetVoltageMonitor):
        """This method simply creates the instances of the power suply, sim922, and ruox temperature monitor."""
        self.ps = PowerSupply(self.log)
        con, err = self.ps.instrumentIsConnected()
        if con == False:
            self.log.log(err, alert=True)
            message = 'Power Supply not connected. Please turn it on and wait a minute or two.\n'
            self.log.log(message, alert=True)
        else:
            self.ps.initiate()
        if ruoxTempMonitor == None:
            try: ruoxTempMonitor = RuOxTemperatureMonitor()
            except GPIBError as e: self.log.log(str(e),alert=True)
        if diodeTempMonitor == None:
            try: diodeTempMonitor = SIM922()
            except GPIBError as e: self.log.log(str(e),alert=True)
        if magnetVoltageMonitor == None:
            try: magnetVoltageMonitor = SIM922()
            except GPIBError as e: self.log.log(str(e),alert=True)
        self.ruoxTempMonitor = ruoxTempMonitor
        self.diodeTempMonitor = diodeTempMonitor
        self.magnetVoltageMonitor = magnetVoltageMonitor
    def measurementCycle(self):
        """ This takes care of the real time temperature plotting. It starts immediately upon starting the program, and never stops. """
        i = self.cycle
        cycleStartTime = time.time()
        #update current and voltage data
        self.currentBackEMF.set( "{0:.3f}".format(self.magnetVoltageMonitor.getMagnetVoltage()) )
        if self.ps.instrumentIsConnected()[0] == True:
            try:
                self.currentI.set( "{0:.3f}".format(self.ps.getCurrent()) )
                self.currentV.set( "{0:.3f}".format(self.ps.getVoltage()) )
            except:
                self.currentI.set('')
                self.currentV.set('')
        else:
            self.currentI.set('')
            self.currentV.set('')
        #append diode temp data
        self.newTemps = self.diodeTempMonitor.getDiodeTemperatures()
        #append the ruox temps, but can only measure every once in a while b/c of time constant
        if self.ruoxTempMonitor.getTimeSinceChannelSet() >= 10*self.ruoxTempMonitor.getTimeConstant():
            if self.ruoxChan == 'GGG': self.GGGTemp = self.ruoxTempMonitor.getTemperature()
            elif self.ruoxChan == 'FAA': self.FAATemp = self.ruoxTempMonitor.getTemperature()
            if self.tGGG.get() == 1 and self.tFAA.get() == 1:
                if self.ruoxChan == 'GGG':
                    self.ruoxTempMonitor.setChannel(2)
                    self.ruoxChan = 'FAA'
                elif self.ruoxChan == 'FAA':
                    self.ruoxTempMonitor.setChannel(1)
                    self.ruoxChan = 'GGG'
            elif self.tGGG.get() == 1 and self.tFAA.get() == 0:
                self.ruoxTempMonitor.setChannel(1)
                self.ruoxChan = 'GGG'
            elif self.tGGG.get() == 0 and self.tFAA.get() == 1:
                self.ruoxTempMonitor.setChannel(2)
                self.ruoxChan = 'FAA'
        if self.tGGG.get() == 0 or self.GGGTemp == 20.0: self.GGGTemp = numpy.nan
        if self.tFAA.get() == 0 or self.FAATemp == 45.0: self.FAATemp = numpy.nan
        self.newTemps += [self.GGGTemp,self.FAATemp]
        #append temp array to tempHistory, soon have it save to file
        self.timeStamps = numpy.append(self.timeStamps,time.time() - self.startTime)
        #save temps in file
        with open(FILE_PATH+'\\temperatures'+self.dateAppend+'.txt','a') as f:
            f.write(str(self.timeStamps[-1]) + '\t' + '\t'.join(map(str,self.newTemps))+'\n')
        #set x limits
        xCyclesMin = max(0,i-60*self.wScale.get())
        if self.wScale.get() == 1440: xCyclesMin = 0
        xMin = self.timeStamps[xCyclesMin]
        #change data to plot
        self.stage60K.set_xdata(self.timeStamps)
        self.stage60K.set_ydata(numpy.append(self.stage60K.get_ydata(),self.newTemps[0]))
        self.stage03K.set_xdata(self.timeStamps)
        self.stage03K.set_ydata(numpy.append(self.stage03K.get_ydata(),self.newTemps[1]))
        self.stageGGG.set_xdata(self.timeStamps)
        self.stageGGG.set_ydata(numpy.append(self.stageGGG.get_ydata(),self.newTemps[2]))
        self.stageFAA.set_xdata(self.timeStamps)
        self.stageFAA.set_ydata(numpy.append(self.stageFAA.get_ydata(),self.newTemps[3]))
        #rescale axes, with the x being scaled by the slider
        if self.toolbar._active == 'HOME' or self.toolbar._active == None:
            ymin,ymax = 10000000, -10000000
            lineAndVar = {self.stage60K:self.t60K, self.stage03K:self.t3K, self.stageGGG:self.tGGG, self.stageFAA:self.tFAA}
            for line in lineAndVar.keys():
                if lineAndVar[line].get() == 0: line.set_visible(False)
                else:
                    line.set_visible(True)
                    ydata = line.get_ydata()[xCyclesMin:-1]
                    try:
                        ymin = min(ymin, numpy.nanmin(ydata))
                        ymax = max(ymax, numpy.nanmax(ydata))
                    except ValueError as e: pass
            if len(self.timeStamps)>1: 
                self.ax.set_xlim(xMin,self.timeStamps[-1])
                self.ax.set_ylim(ymin - (ymax-ymin)/10, ymax + (ymax-ymin)/10)
        self.updateLegend()
        self.canvas.draw()
        self.cycle += 1
        cycleLength = int(1000*(time.time() - cycleStartTime))
        self.after(max(0,STEP_LENGTH-cycleLength),self.measurementCycle)
    def updateLegend(self):
        """creates the legend at the top of the temperature plot."""
        labelOrder = ['60K','3K ','GGG','FAA']
        lines = [self.stage60K,self.stage03K,self.stageGGG,self.stageFAA]
        labels = [labelOrder[l]+' ['+"{0:.3f}".format(self.newTemps[l])+'K]' for l in range(len(labelOrder))]
        labels = [s.replace('1.#QOK','OoR') for s in labels]
        #self.ax.legend(lines,labels,loc=0)#,bbox_to_anchor=(1.01, 1)) #legend in upper right
        self.ax.legend(lines,labels,bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=4, mode="expand", borderaxespad=0.) #legend on top (if not using this, delete \n in title)
    def renewPowerSupply(self):
        """This runs once a minute and checks if the power supply has since been
            turned on or off, and refreshes the instance of it."""
        alreadyConnected,err = self.ps.instrumentIsConnected()
        self.ps = PowerSupply(self.log)
        if alreadyConnected == False and self.ps.instrumentIsConnected()[0] == True:
            self.ps.initiate()
        self.after(60*1000, self.renewPowerSupply)
    def executeExternalCommands(self):
        """Commands:
                ['magup'] - mags up to 9A
                ['magdown'] - mags all the way down, with temp goal of 0
                ['regulate'] - regulates at the temp already entered
                ['regulate',0.15] - regulates at 150mK
                ['gettemp'] - returns the temperature of the cold stage
        """
        address = ('localhost', 6000)     # family is deduced to be 'AF_INET'
        #listener = Listener(address, authkey='secret password')
        #conn = listener.accept()
        #print 'connection accepted from', listener.last_accepted
        """while True:
            msg = conn.recv()
            # do something with msg
            if msg == 'close':
                conn.close()
                break
        listener.close()"""
    def magUp(self):
         """ The magging up method, as per the HPD Manual, involves increasing the voltage in steps of MAG_UP_dV volts
         every cycle of the loop.  This cycle happens once every STEP_LENGTH seconds, nominally 1s (since the voltage
         monitor reads once a second).  Each cycle, the voltage across the magnet is read to get the backEMF.  If it
         is greater than the MAGNET_VOLTAGE_LIMIT, the voltage will not be raised until the next cycle for which the
         backEMF < MAGNET_VOLTAGE_LIMIT. Called when Mag Up button is pressed. """
         self.log.log('Beginning to mag up to '+str(CURRENT_LIMIT)+' Amps.\n')
         if self.ps.instrumentIsConnected()[0] == False:
             message = 'Cannot mag up: Power Supply not connected. Please turn it on and wait a minute or two.\n'
             self.log.log(message, alert=True)
             return
         self.isMaggingUp = True
         class local:
             _job = ''
             lastI = self.ps.getCurrent()
             lastTime = time.time()
         def cancelMagUp():
             if local._job is not None:
                 self.after_cancel(local._job)
                 local._job = None
                 self.isMaggingUp = False
             self.magUpButton.configure(text='Mag Up', command=self.magUp)
             self.regulateButton.configure(state=Tkinter.NORMAL)
             message = 'Magging up stopped at a current of '+str(self.ps.getCurrent())+' Amps.\n'
             self.log.log(message)
         def increaseV():
             if self.isMaggingUp:
                 startTime = time.time()
                 I_now = self.ps.getCurrent()
                 dI = I_now-local.lastI
                 dt = time.time() - local.lastTime
                 if dt == 0: dt = 0.0000000001 #to prevent divide by zero error
                 local.lastI = I_now
                 local.lastTime = time.time()
                 if I_now < CURRENT_LIMIT:
                     if self.magnetVoltageMonitor.getMagnetVoltage() < MAGNET_VOLTAGE_LIMIT and abs(dI/dt) < dIdt_MAGUP_LIMIT:
                         newVoltage = self.ps.getVoltage() + MAG_UP_dV
                         if newVoltage < VOLTAGE_LIMIT:
                             self.ps.setVoltage(newVoltage)
                         else: self.ps.setVoltage(VOLTAGE_MAX)
                         #newCurrent = ps.getCurrent() + 0.005
                         #ps.setCurrent(newCurrent)
                     cycleLength = int(1000*(time.time() - startTime))
                     local._job = self.after(max(0,STEP_LENGTH-cycleLength), increaseV)
                 else:
                     self.magUpButton.configure(text='Mag Up', command=self.magUp)
                     self.regulateButton.configure(state=Tkinter.NORMAL)
                     message = 'Finished magging up. '+str(self.ps.getCurrent())+' Amps reached.\n'
                     self.log.log(message)
         self.magUpButton.configure(text='Stop Magging Up', command=cancelMagUp)
         self.regulateButton.configure(state=Tkinter.DISABLED)
         increaseV()
    def regulate(self):
        """ This function is almost equivalent to the old code that Steve Sendelbach had implemented in LabVIEW.  It is
        based on a PID controller.  I also added a VOLTAGE_LIMIT case.  The basics of it is that a new voltage V+dV is
        proposed.  dV is then limited as necessary, and the new voltage is set. As with magging up, regulate runs a cycle
        at approximately once per second. Called when regulate button is pressed. """
        T_target = float(self.regulateTempField.get())
        if self.ps.instrumentIsConnected()[0] == False:
            message = 'Cannot regulate: Power Supply not connected.  Please turn it on and wait a minute or two.\n'
            self.log.log(message, alert=True)
            return
        message = 'Starting regulation cycle from '+str(self.ps.getCurrent())+' Amps.\n'
        self.log.log(message)
        self.isRegulating = True
        print 'beginning regulation'
        print 'V\tbackEMF\tdV/dT\tdV'
        class local:
            _job = ''
            lastTime = time.time()
            lastMagnetV = 0
            lastI = self.ps.getCurrent()
            FAATemp = self.ruoxTempMonitor.getTemperature()
            if FAATemp == numpy.nan: FAATemp = self.newTemps[1]
        def cancelRegulate():
            if local._job is not None:
                self.after_cancel(local._job)
                local._job = None
            self.regulateButton.configure(text='Regulate', command=self.regulate)
            self.magUpButton.configure(state=Tkinter.NORMAL)
            message = 'Regulation stopped at a current of '+str(self.ps.getCurrent())+' Amps.\n'
            self.log.log(message)
            self.isRegulating = False
        def oneRegCycle():
            if self.isRegulating:
                startTime = time.time()
                backEMF = self.magnetVoltageMonitor.getMagnetVoltage()
                V_now = self.ps.getVoltage()
                I_now = self.ps.getCurrent()
                dI = I_now - local.lastI
                local.lastI = I_now
                if self.ruoxTempMonitor.getChannel() == 2: local.FAATemp = self.ruoxTempMonitor.getTemperature()
                if local.FAATemp == numpy.nan: local.FAATemp = self.newTemps[1]
                print str(V_now)+'\t'+str(backEMF)+'\t',
                #propose new voltage
                dV = PID_KP*(T_target-local.FAATemp)-backEMF*PID_KD
                dT = time.time()-local.lastTime
                if dT == 0: dT = 0.0000000001 #to prevent divide by zero error
                local.lastTime = time.time()
                #hard current limit
                if self.ps.getCurrent() > CURRENT_LIMIT:
                    if dV>0: dV=0
                #hard voltage limit
                if V_now + dV > VOLTAGE_LIMIT:
                    dV = VOLTAGE_LIMIT - V_now
                # steady state limit
                if dV < 0:
                    dV = max(dV,backEMF-MAGNET_VOLTAGE_LIMIT)
                    if dV > 0: dV = 0
                if dV > 0:
                    dV = min(dV, MAGNET_VOLTAGE_LIMIT-backEMF)
                    if dV < 0: dV = 0
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
                    self.ps.setVoltage(0)
                    V_now = 0
                    dV = 0
                    runCycleAgain = False
                print str(dV)
                self.ps.setVoltage(V_now + dV)
                cycleTime = int(1000*(time.time() - startTime))
                if runCycleAgain: local._job = self.after(max(0,STEP_LENGTH-cycleTime), oneRegCycle)
                else:
                    self.regulateButton.configure(text='Regulate', command=self.regulate)
                    self.magUpButton.configure(state=Tkinter.NORMAL)
                    message = 'Regulation has completed. Mag up and try again.\n'
                    self.log.log(message)
        self.regulateButton.configure(text='Stop Regulating', command=cancelRegulate)
        self.magUpButton.configure(state=Tkinter.DISABLED)
        oneRegCycle()
    def _quit(self):
        """ called when the window is closed."""
        self.quit()     # stops mainloop
        self.destroy()  # this is necessary on Windows to prevent
                        # Fatal Python Error: PyEval_RestoreThread: NULL tstate

if __name__ == "__main__":
    """Define your instruments here.  This allows for easy exchange between different
    devices to monitor temperature, etc.  For example, the new and old ADR's use two
    different instruments to measure temperature: The SRS module and the Lakeview 218."""
    try: ruoxTempMonitor = RuOxTemperatureMonitor()
    except GPIBError as e: ruoxTempMonitor = None
    try: diodeTempMonitor = SIM922()
    except GPIBError as e: diodeTempMonitor = None
    magnetVoltageMonitor = diodeTempMonitor
    app = ADRController(None, diodeTempMonitor, ruoxTempMonitor, magnetVoltageMonitor)
    app.title('ADR Controller')
    app.mainloop()
