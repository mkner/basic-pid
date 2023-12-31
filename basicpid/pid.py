#
#
# BasicPid - Basic Python PID Controller  
# for discrete-time regulators
#
# (c) 2023, 2022  Mike Knerr
#
#

#from robobase import Object # as of 9/5, decoupled as of 11/14

#from robo_base import Object

class BasicPid(object): 
#class BasicPid(Object): just use standard python object
     """PID controller for discrete-time regulators""" 
     def __init__(self):
       super(BasicPid, self).__init__()
       self._id = id(self)
       self._name = "BasicPid"
       self._desc = "Easy to use classic PID controller"
       self._vers = "v0.02.07"  ## "v0.02.03 <-- ... v0.01.11b1"  #"v0.01.11b1 from dev v0.01.07b"
       self._about = "BasicPid is an easy to use PID controller designed for discrete-time regulators"

       #PID
       self._Kp = 1.0
       self._Ki = 0
       self._Kd = 0
       
       self._P = 0
       self._I = 0 # integral sumation
       self._D = 0
       
       self._e_prev = 0 # previous error (t-1)
       self._e_prev_prev = 0 # prev-previous error (t-2)
       
       self._pid_out = 0
       self._pid_out_prev = 0
     
       self._delta_time = 1 # in whatever time unit signal is in !!!
       
       self._setAllModesOff()
       self._mode_iterate = True # default summation by calling function
     
     # as of 0.2.6a functions from robobase.Object included here
     # so BasicPid is decoupled from that module/package
     
     # info functions return objects
     
     def getWhoami(self):
          return str(self._name+" "+self._vers) #+" "+self._model)
     
     def whoamiStr(self):
          return self.getWhoami()
     
     def getId(self):
          return self._id
     
     def getName(self):
          return self._name

     def getDesc(self):
          return self._desc
    
     def getVersion(self):
          return self._vers
    
     def getVers(self):
          return self._vers
    
     def getAbout(self):
          return self._about
    
     def setName(self,n):
         self._name = n
    
     def setDesc(self,d):
         self._desc = d
         
     def setAbout(self,a):
         self._about = a
         
     # shell mode versions print  
     
     def id(self):
          print(self._id)
         
     def whoami(self):
          print(self._name,self._vers) #,self._model)
          
     def name(self):
          print(self._name)

     def desc(self):
          print(self._desc)
    
     def version(self):
          print(self._vers)
    
     def vers(self):
          print(self._vers)
    
     def about(self):
          print(self._about)
          
     # PID functions start here     
     
     def reset(self):
         # reset for new run - clear out integrations
         # keeps gains intact
         self._P = 0
         self._I = 0 
         self._D = 0
         self._e_prev = 0 # previous error (t-1)
         self._e_prev_prev = 0 # prev-previous error (t-2)
         self._pid_out = 0
         self._pid_out_prev = 0
         
         
     def resetAll(self):
         # reset all like init
         # gains set to default, and time inc
           self._Kp = 1.0
           self._Ki = 0
           self._Kd = 0
           
           self._P = 0
           self._I = 0
           self._D = 0
           
           self._e_prev = 0 # previous error
           self._e_prev_prev = 0 # prev-previous error (t-2)
           
           self._delta_time = 1 # in whatever time unit signal is in!!!
           
           self._pid_out = 0
           self._pid_out_prev = 0
           
           # step integration done outside this object
           self.setIterateModeOn()
           
        
     def _setAllModesOff(self):
           self._mode_iterate = False
           self._mode_integrate = False
           
           
     def setIntegrateModeOn(self):
         self._setAllModesOff()
         self._mode_integrate = True
         
     def setIterateModeOn(self):
         self._setAllModesOff()
         self._mode_iterate = True
   
    
     def inIntegrateMode(self):
         return (self._mode_integrate == True)
     
     def inIterateMode(self):
         return (self._mode_iterate == True)
     
     def setTimeinc(self,time_inc):
         # make sure discrete time unit is same as with signal!
         if time_inc == 0:
             return
         time_inc = abs(time_inc) # just fix it
         self._delta_time = time_inc
        
     def getTimeinc(self):
         return self._delta_time
     
     def setKp(self,Kp):
        self._Kp = Kp
        
     def getKp(self):
        return self._Kp   
     
     def setKi(self,Ki):
        self._Ki = Ki
        
     def getKi(self):
        return self._Ki

     def setKd(self,Kd):
        self._Kd = Kd
        
     def getKd(self):
        return self._Kd
    
     def setGains(self,Kp,Ki,Kd):
        # isomorphic
        self._Kp=Kp
        self._Ki=Ki
        self._Kd=Kd
     
     def getGains(self):
         # make it easy
         return (self._Kp,self._Ki,self._Kd)
  
     def _calcPid(self, signal_ref, signal):#, prev_input):
       
        # for integrate mode  
        self._pid_out_prev = self._pid_out
        
        #proportional term always same regardless of mode
        e = signal_ref-signal
        #NEW
        #update 7/11 to braunl version to match I & D calcs
        self._P = e - self._e_prev
        #self._P = e
        P = self._P
        
        # integral term
        self._I = (e+self._e_prev)/2 #trapezoidal apx
        I = self._I
        
        # derivative term - time step integration version
        self._D = (e - 2*self._e_prev + self._e_prev_prev)
        D = self._D
           
        if self.inIntegrateMode():
            self._pid_out = self._pid_out_prev + self._Kp*P + self._Ki*I + self._Kd*D
        
        if self.inIterateMode():
            #timestep integration is done outside by calling function
            self._pid_out = self._Kp*P + self._Ki*I + self._Kd*D
        
        #update t-1, t-2
        self._e_prev_prev = self._e_prev
        self._e_prev = e

        return
    
     def getPid(self, signal_ref, signal):
         # calc & persistent values depend on mode
         self._calcPid(signal_ref, signal)
         return self._pid_out
     

     def getPidTuple(self,signal_ref,signal):
          # explicitiy return tuple form
          # irregardless of mode
            return(self._Kp,self._P,\
                   self._Ki,self._I,\
                   self._Kd,self._D)
     
     def pid(self, signal_ref, signal):
         #short form
         return self.getPid(signal_ref, signal)
         
     def get(self, signal_ref, signal):
         #short form
         return self.getPid(signal_ref, signal)

## SCRAP HEAP


